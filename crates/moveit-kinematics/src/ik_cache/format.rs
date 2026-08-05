// Copyright (c) 2017, Rice University
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_kinematics/cached_ik_kinematics_plugin/src/ik_cache.cpp
//     (`IKCache::saveCache`, and `initializeCache`'s read half)

//! The one place [`super::IkCache`]'s on-disk form is defined.
//!
//! # Why JSON, and not upstream's byte layout
//!
//! `IKCache::saveCache` writes three `unsigned int`s (entry count, DOF
//! count, tip count) followed by a `memcpy` of each entry's `tf2Scalar`
//! position, `tf2Scalar` orientation and `double` config, straight out of
//! host memory; `initializeCache` `memcpy`s them back. That layout is
//! host-endian, `tf2Scalar`-width-dependent (a `float` build and a
//! `double` build disagree on every field), carries no version tag, and is
//! read by exactly one C++ class and nothing else -- there is no producer
//! or consumer outside upstream's own process for a Rust port to stay
//! byte-compatible with, so this port does not try to. `PORTING-PLAN.md`
//! §80.2 records the same conclusion: the disk format is "a local serde
//! choice, not a port target".
//!
//! What it is instead: `serde_json`, the crate this workspace already
//! resolves with the `float_roundtrip` feature
//! (`tools/ci/check-serde-float-roundtrip.sh`, which exists because the
//! default parser reads 8.1% of this repo's fixture literals one ULP
//! wrong). With that feature every finite `f64` written here parses back
//! to the same bits, so a cache file is a lossless copy of the entries,
//! not an approximation of them -- and, unlike a `memcpy` of a struct, a
//! diff of two cache files is readable.
//!
//! # The choice is reversible
//!
//! Everything that knows what the bytes look like is in this file: the
//! document types below, their `serde` derives, and the two functions
//! [`to_json`]/[`from_json`]. [`super::IkCache`] and
//! [`super::CacheEntry`] derive no `serde` traits and name no format, so
//! swapping JSON for anything else (upstream's byte layout included) means
//! rewriting this file and nothing else.
//!
//! The document types deliberately do not reuse [`IkCacheOptions`] or
//! [`super::CacheEntry`] as their own serialized shape, even though the
//! fields line up today. Deriving `Serialize` on those would make the file
//! format a function of the in-memory types' field names, so a rename in
//! either would silently become a format break with no diff in this file
//! to review.
//!
//! # What is not ported
//!
//! Upstream's *when to save* policy, not just its bytes:
//!
//! - `IKCache::updateCache` calls `saveCache()` inline whenever the cache
//!   has grown 500 entries past its last save, or has just hit
//!   `max_cache_size_` -- so an IK query pays a synchronous disk write.
//!   This port saves only when a caller asks it to
//!   ([`super::IkCache::save`]).
//! - `~IKCache()` saves again on destruction, discarding any error: a
//!   `Drop` impl has nowhere to report a failed write, which is precisely
//!   why saving is an explicit, `Result`-returning call here.
//! - `initializeCache` derives the file *name* from the robot id, group
//!   name, tip frames and all three option values, then reads the file if
//!   `std::filesystem::exists` says so and starts empty if not. This port
//!   takes a path from the caller and reads exactly it. The name-mangling
//!   exists upstream to keep a cache built under one set of thresholds
//!   from being reused under another; [`from_json`] gets that property
//!   from the file's own contents instead (the options travel *in* the
//!   document, so a load cannot disagree with them), which is also why
//!   there is no "load, or silently start empty if absent" entry point:
//!   an absent file and an empty cache are different facts.

use serde::{Deserialize, Serialize};

use moveit_error::{Error, Result};
use moveit_geometry::{Isometry3, UnitQuaternion, Vector3};

use super::{CacheEntry, IkCache, IkCacheOptions};

/// How far a stored orientation's norm may sit from 1 and still be read
/// back as the unit quaternion it claims to be.
///
/// This is a *validity* bound, not an error budget. The quaternions this
/// module writes come from [`Isometry3::rotation`], which nalgebra keeps
/// normalized to within half an ULP of 1 -- measured, not assumed:
/// `|norm - 1|` over this module's own fixtures is `1.1102230246251565e-16`
/// (exactly `2^-53`), pinned by
/// `stored_rotations_sit_far_inside_the_unit_quaternion_tolerance`. So a
/// value that reaches this threshold is not a slightly-stale rotation but
/// a number that was never one -- the all-zero quaternion a hand-written or
/// truncated file yields, say, whose norm is 0. Seven orders of margin over
/// the measured figure keeps the check from ever firing on a rotation this
/// port produced, while still rejecting anything that would turn into a
/// garbage rotation.
const UNIT_QUATERNION_TOLERANCE: f64 = 1e-9;

/// The document's version tag, as an enum with exactly one variant rather
/// than a `String` compared at runtime: a document tagged anything else
/// fails to deserialize, so a `v2` file cannot reach the code below and be
/// read as though it were a `v1` one. Adding a version later means adding
/// a variant, which makes every `match` on it a compile error until it is
/// handled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
enum FormatTag {
    #[serde(rename = "moveit-rs.ik-cache.v1")]
    V1,
}

/// One `(pose, config)` pair on disk.
///
/// `position` and `orientation` are the same seven scalars upstream
/// `memcpy`s (`ik_cache.cpp:251-252`), in the same order: `x, y, z` then
/// `tf2::Quaternion`'s own `x, y, z, w` storage order -- which is *not*
/// nalgebra's `Quaternion::new(w, x, y, z)` argument order, so the two are
/// spelled out at every conversion below rather than passed positionally.
#[derive(Debug, Serialize, Deserialize)]
struct EntryDocument {
    position: [f64; 3],
    orientation: [f64; 4],
    config: Vec<f64>,
}

/// [`IkCacheOptions`]'s serialized twin -- see this module's doc for why
/// it is a separate type.
#[derive(Debug, Serialize, Deserialize)]
struct OptionsDocument {
    max_cache_size: usize,
    min_pose_distance: f64,
    min_config_distance: f64,
}

/// A whole cache file.
#[derive(Debug, Serialize, Deserialize)]
struct CacheDocument {
    format: FormatTag,
    /// Upstream's `num_dofs` header field. Upstream writes it and then
    /// never checks it against the group it is loading into
    /// (`doc/upstream-bugs.md`, `ik-cache-read-trusts-file-header`); here
    /// it is the field [`from_json`] validates first.
    num_joints: usize,
    options: OptionsDocument,
    entries: Vec<EntryDocument>,
}

/// Serialize `cache` as a cache file's exact contents.
///
/// # Errors
///
/// [`Error::Other`] if any stored coordinate or joint value is not finite.
/// `serde_json` renders a non-finite `f64` as `null`, and `null` does not
/// deserialize back into an `f64`: without this check the write would
/// report success and only a later read would fail, by which time the
/// entries it was meant to preserve are gone.
pub(super) fn to_json(cache: &IkCache) -> Result<String> {
    let mut entries = Vec::with_capacity(cache.entries.len());
    for (index, entry) in cache.entries.iter().enumerate() {
        let translation = entry.pose.translation.vector;
        let rotation = entry.pose.rotation.quaternion();
        let document = EntryDocument {
            position: [translation.x, translation.y, translation.z],
            orientation: [rotation.i, rotation.j, rotation.k, rotation.w],
            config: entry.config.clone(),
        };
        reject_non_finite(index, &document)?;
        entries.push(document);
    }

    let document = CacheDocument {
        format: FormatTag::V1,
        num_joints: cache.num_joints,
        options: OptionsDocument {
            max_cache_size: cache.options.max_cache_size,
            min_pose_distance: cache.options.min_pose_distance,
            min_config_distance: cache.options.min_config_distance,
        },
        entries,
    };
    let mut text = serde_json::to_string_pretty(&document)
        .map_err(|error| Error::other(format!("ik cache could not be serialized: {error}")))?;
    text.push('\n');
    Ok(text)
}

/// Rebuild an [`IkCache`] from a cache file's contents, for a solver with
/// `num_joints` joints.
///
/// # Errors
///
/// [`Error::Other`] if `text` is not a `v1` cache document, or if it is
/// one whose contents cannot be a cache for a `num_joints`-joint solver:
/// a declared joint count that is not `num_joints`, an entry whose config
/// is not that long, more entries than the document's own
/// `max_cache_size` admits, or an orientation that is not a unit
/// quaternion. Upstream checks none of these — see `doc/upstream-bugs.md`,
/// `ik-cache-read-trusts-file-header` and
/// `update-cache-capacity-as-size-limit`, for what each one lets through
/// there.
pub(super) fn from_json(text: &str, num_joints: usize) -> Result<IkCache> {
    let document: CacheDocument = serde_json::from_str(text)
        .map_err(|error| Error::other(format!("ik cache could not be read: {error}")))?;

    if document.num_joints != num_joints {
        return Err(Error::other(format!(
            "ik cache holds {}-joint configs, but this solver has {num_joints} joints",
            document.num_joints
        )));
    }
    if document.entries.len() > document.options.max_cache_size {
        return Err(Error::other(format!(
            "ik cache holds {} entries, past its own max_cache_size of {}",
            document.entries.len(),
            document.options.max_cache_size
        )));
    }

    let options = IkCacheOptions {
        max_cache_size: document.options.max_cache_size,
        min_pose_distance: document.options.min_pose_distance,
        min_config_distance: document.options.min_config_distance,
    };
    let mut cache = IkCache::new(&options, num_joints);
    for (index, entry) in document.entries.into_iter().enumerate() {
        if entry.config.len() != num_joints {
            return Err(Error::other(format!(
                "ik cache entry {index} has {} joint values, but the cache is {num_joints}-joint",
                entry.config.len()
            )));
        }
        cache.entries.push(CacheEntry {
            pose: isometry_from(index, &entry)?,
            config: entry.config,
        });
    }
    Ok(cache)
}

/// The seven scalars of `entry` as an [`Isometry3`].
///
/// The orientation is checked against [`UNIT_QUATERNION_TOLERANCE`] and
/// then taken as-is rather than run through `UnitQuaternion::try_new`,
/// which would renormalize: dividing a quaternion whose norm is `1` to
/// within a couple of ULP by that norm perturbs its components, and this
/// function's whole job is to give back the pose that was written, bit for
/// bit.
fn isometry_from(index: usize, entry: &EntryDocument) -> Result<Isometry3> {
    let [x, y, z, w] = entry.orientation;
    let quaternion = nalgebra::Quaternion::new(w, x, y, z);
    if (quaternion.norm() - 1.0).abs() > UNIT_QUATERNION_TOLERANCE {
        return Err(Error::other(format!(
            "ik cache entry {index} has orientation {:?}, which is not a unit quaternion",
            entry.orientation
        )));
    }
    let [px, py, pz] = entry.position;
    Ok(Isometry3::from_parts(
        Vector3::new(px, py, pz).into(),
        UnitQuaternion::new_unchecked(quaternion),
    ))
}

/// See [`to_json`]'s `# Errors`.
fn reject_non_finite(index: usize, entry: &EntryDocument) -> Result<()> {
    let finite = entry.position.iter().all(|value| value.is_finite())
        && entry.orientation.iter().all(|value| value.is_finite())
        && entry.config.iter().all(|value| value.is_finite());
    if finite {
        Ok(())
    } else {
        Err(Error::other(format!(
            "ik cache entry {index} holds a non-finite value, which JSON cannot represent"
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A two-entry, three-joint cache whose poses differ in both
    /// translation and rotation, so a round trip that dropped either half
    /// of a pose would show up as a changed entry rather than as an
    /// unchanged one.
    fn populated_cache() -> IkCache {
        let options = IkCacheOptions {
            max_cache_size: 7,
            min_pose_distance: 0.25,
            min_config_distance: 0.25,
        };
        let mut cache = IkCache::new(&options, 3);
        for (index, config) in [[0.1, -0.2, 0.3], [1.4, 1.5, -1.6]].into_iter().enumerate() {
            let pose = Isometry3::from_parts(
                Vector3::new(index as f64 + 0.125, -0.5, 2.75).into(),
                UnitQuaternion::from_euler_angles(0.3, -0.7, 1.1 * index as f64),
            );
            let nearest = cache.nearest(&pose);
            cache.update(&nearest, &pose, &config);
        }
        assert_eq!(cache.entries.len(), 2);
        cache
    }

    #[test]
    fn a_round_trip_returns_every_seed_bit_for_bit() {
        let cache = populated_cache();
        let read_back = from_json(&to_json(&cache).unwrap(), 3).unwrap();

        assert_eq!(read_back.entries.len(), cache.entries.len());
        for (before, after) in cache.entries.iter().zip(&read_back.entries) {
            assert_eq!(after.config, before.config);
            assert_eq!(
                after.pose.translation.vector,
                before.pose.translation.vector
            );
            assert_eq!(
                after.pose.rotation.quaternion().coords,
                before.pose.rotation.quaternion().coords
            );
        }
    }

    /// The three [`IkCacheOptions`] fields travel in the document, so a
    /// loaded cache gates insertions exactly as the saved one did.
    #[test]
    fn a_round_trip_returns_the_options_the_cache_was_saved_under() {
        let cache = populated_cache();
        let read_back = from_json(&to_json(&cache).unwrap(), 3).unwrap();

        assert_eq!(read_back.options, cache.options);
        assert_eq!(read_back.num_joints, cache.num_joints);
    }

    /// Every rejection test below starts from this exact document and
    /// changes one thing, so what separates an accepted document from a
    /// rejected one is only ever the field the test names. Written out as
    /// text rather than produced by [`to_json`] for the same reason: a
    /// malformed document is by definition one [`to_json`] would not
    /// write.
    fn document(tag: &str, num_joints: usize, max_cache_size: usize, entries: &str) -> String {
        format!(
            r#"{{"format":"{tag}",
                "num_joints":{num_joints},
                "options":{{"max_cache_size":{max_cache_size},
                            "min_pose_distance":0.25,
                            "min_config_distance":0.5}},
                "entries":[{entries}]}}"#
        )
    }

    /// One well-formed entry: identity orientation, three joint values.
    const ENTRY: &str = r#"{"position":[0.125,-0.5,2.75],
                            "orientation":[0.0,0.0,0.0,1.0],
                            "config":[0.1,-0.2,0.3]}"#;

    #[test]
    fn the_reference_document_every_rejection_test_mutates_is_itself_accepted() {
        let cache = from_json(&document("moveit-rs.ik-cache.v1", 3, 7, ENTRY), 3).unwrap();
        assert_eq!(cache.entries.len(), 1);
        assert_eq!(cache.entries[0].config, [0.1, -0.2, 0.3]);
    }

    /// Upstream reads `num_dofs` out of the file and never compares it to
    /// the group it is loading into, so a cache written for a 7-joint arm
    /// seeds a 3-joint one with 7 values and then indexes past the end of
    /// the shorter config in `configDistance2`.
    #[test]
    fn a_document_written_for_a_different_arm_is_rejected() {
        let error = from_json(&document("moveit-rs.ik-cache.v1", 3, 7, ENTRY), 7).unwrap_err();
        assert_eq!(
            error.to_string(),
            "ik cache holds 3-joint configs, but this solver has 7 joints"
        );
    }

    /// The document's own two joint counts have to agree with each other,
    /// not only with the solver: `num_joints` is one field and each
    /// entry's `config` carries its own length.
    #[test]
    fn an_entry_shorter_than_the_declared_joint_count_is_rejected() {
        let short = ENTRY.replace("[0.1,-0.2,0.3]", "[0.1,-0.2]");
        let text = document("moveit-rs.ik-cache.v1", 3, 7, &format!("{ENTRY},{short}"));
        let error = from_json(&text, 3).unwrap_err();
        assert_eq!(
            error.to_string(),
            "ik cache entry 1 has 2 joint values, but the cache is 3-joint"
        );
    }

    /// `IkCache::update` refuses to grow past `max_cache_size`, so a file
    /// claiming more entries than that was not written by this port. It
    /// matters because upstream's insert gate is `size() < capacity()`,
    /// and `capacity()` is whatever the last `reserve` left behind --
    /// loading an oversized file is exactly how that bound comes loose.
    #[test]
    fn more_entries_than_the_documents_own_max_cache_size_is_rejected() {
        let text = document("moveit-rs.ik-cache.v1", 3, 1, &format!("{ENTRY},{ENTRY}"));
        let error = from_json(&text, 3).unwrap_err();
        assert_eq!(
            error.to_string(),
            "ik cache holds 2 entries, past its own max_cache_size of 1"
        );
    }

    /// A truncated or hand-edited file can hold an all-zero orientation,
    /// which is not a rotation at all: normalizing it divides by zero, and
    /// the NaN that produces makes `pose_distance` return NaN for every
    /// query afterwards.
    #[test]
    fn an_orientation_that_is_not_a_unit_quaternion_is_rejected() {
        let zeroed = ENTRY.replace("[0.0,0.0,0.0,1.0]", "[0.0,0.0,0.0,0.0]");
        let error = from_json(&document("moveit-rs.ik-cache.v1", 3, 7, &zeroed), 3).unwrap_err();
        assert_eq!(
            error.to_string(),
            "ik cache entry 0 has orientation [0.0, 0.0, 0.0, 0.0], which is not a unit quaternion"
        );
    }

    /// The version tag is load-bearing, not decoration. The two documents
    /// below differ in that one string and nothing else, so the only thing
    /// that can separate the accepted one from the rejected one is the tag.
    #[test]
    fn a_document_tagged_with_another_version_does_not_load_as_v1() {
        let v2 = document("moveit-rs.ik-cache.v2", 3, 7, ENTRY);
        let Err(_) = from_json(&v2, 3) else {
            panic!("a document tagged v2 must not be read as a v1 document");
        };
    }

    /// `serde_json` writes a non-finite `f64` as `null`, which then fails
    /// to read back as a number -- so without this the save would succeed
    /// and the entries would be lost at the next load instead.
    #[test]
    fn saving_a_non_finite_joint_value_is_an_error_rather_than_a_lossy_write() {
        let mut cache = populated_cache();
        cache.entries[1].config[2] = f64::NAN;
        let error = to_json(&cache).unwrap_err();
        assert_eq!(
            error.to_string(),
            "ik cache entry 1 holds a non-finite value, which JSON cannot represent"
        );
    }

    /// The measurement [`UNIT_QUATERNION_TOLERANCE`]'s doc comment cites,
    /// re-run as an assertion so the constant's stated margin is checked
    /// against the rotations this module actually writes rather than
    /// against a number someone typed once.
    #[test]
    fn stored_rotations_sit_far_inside_the_unit_quaternion_tolerance() {
        let worst = populated_cache()
            .entries
            .iter()
            .map(|entry| (entry.pose.rotation.quaternion().norm() - 1.0).abs())
            .fold(0.0_f64, f64::max);
        assert!(
            worst <= 1.1102230246251565e-16,
            "measured normalization error {worst:e} exceeds the cited 1.1102230246251565e-16"
        );
        assert!(worst * 1e6 < UNIT_QUATERNION_TOLERANCE);
    }
}
