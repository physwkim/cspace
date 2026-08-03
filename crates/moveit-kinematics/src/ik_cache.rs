// Copyright (c) 2017, Rice University
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_kinematics/cached_ik_kinematics_plugin/include/moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_plugin.hpp
//   moveit_kinematics/cached_ik_kinematics_plugin/src/ik_cache.cpp

use moveit_geometry::Isometry3;

/// `IKCache::Options`, minus `cached_ik_path`. See this module's `#
/// Deviation` for why on-disk persistence has no field here.
#[derive(Debug, Clone, PartialEq)]
pub struct IkCacheOptions {
    /// `max_cache_size`: `IkCache::update` never grows the cache past this
    /// many entries.
    pub max_cache_size: usize,
    /// `min_pose_distance`: half of `IkCache::update`'s OR-gated novelty
    /// test — see that method's doc comment.
    pub min_pose_distance: f64,
    /// `min_joint_config_distance`: the other half of the same test. Stored
    /// un-squared, like upstream's `Options` field; squared once in
    /// `IkCache::new` to match `min_config_distance2_`.
    pub min_config_distance: f64,
}

impl Default for IkCacheOptions {
    fn default() -> Self {
        Self {
            max_cache_size: 5000,
            min_pose_distance: 1.0,
            min_config_distance: 1.0,
        }
    }
}

/// One cached `(pose, config)` pair, or the transient "nearest" result
/// [`IkCache::nearest`] returns -- upstream's `IKCache::IKEntry`,
/// single-tip only (see `crate::registry::KinematicsSolver`'s `#
/// Deviations`, item 1, for why this crate has no multi-tip shape to give
/// the cache either).
#[derive(Debug, Clone)]
pub(crate) struct CacheEntry {
    pose: Isometry3,
    config: Vec<f64>,
}

impl CacheEntry {
    pub(crate) fn config(&self) -> &[f64] {
        &self.config
    }
}

/// `IKCache::Pose::distance`: position L2 distance plus orientation
/// angular distance, added without any per-axis weighting.
/// `UnitQuaternion::angle_to` computes the angle of `a.rotation_to(b)`,
/// taking the real part's absolute value the same way `tf2::Quaternion::
/// angleShortestPath` does -- both report the *shorter* of the two arcs a
/// unit quaternion's double cover admits, always in `[0, pi]`.
fn pose_distance(a: &Isometry3, b: &Isometry3) -> f64 {
    (a.translation.vector - b.translation.vector).norm() + a.rotation.angle_to(&b.rotation)
}

/// `IKCache::configDistance2`: plain squared-Euclidean distance over joint
/// configs, no per-joint weighting. Panics (via the slice length
/// mismatch) if `a.len() != b.len()`, which cannot happen through this
/// module's own callers -- every config compared here came from the same
/// solver's `joint_names()`-length seed/solution space.
fn config_distance2(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| (x - y).powi(2)).sum()
}

/// A cache of inverse-kinematics solutions, keyed by end-effector pose --
/// `IKCache`, minus on-disk persistence and the GNAT nearest-neighbor
/// index.
///
/// # Deviations from upstream
///
/// 1. **No disk persistence.** `ik_cache.cpp`'s `saveCache`/
///    `initializeCache` read and write a raw, unversioned `memcpy` of
///    `double` fields with no endianness handling, keyed into a filename
///    by robot/group/cache name plus the size and distance thresholds.
///    Nothing outside that one C++ class ever reads that file, so a Rust
///    port owes it no byte-layout compatibility; this type simply does
///    not persist, and a caller who wants that can `serde`-serialize
///    [`IkCache`]'s entries independently. See `lib.rs`'s module doc,
///    point 3, for the fuller reasoning this decision was made from.
/// 2. **Linear scan, not a GNAT tree.** [`IkCache::nearest`] scans every
///    entry rather than porting `detail/NearestNeighborsGNAT.hpp` (755
///    lines implementing a general-purpose metric tree). Upstream's own
///    default `max_cache_size` is 5000, and every shipped example config
///    raises it to 10000; a linear scan over at most 10k entries of the
///    cheap [`pose_distance`] metric is microseconds, not a bottleneck
///    this type exists to avoid. See `lib.rs`'s module doc, point 2.
/// 3. **A different, but equally arbitrary, tie-break.** GNAT tree
///    traversal order for entries exactly tied on [`pose_distance`] is a
///    function of insertion order and internal pivot choices upstream
///    itself never documents or relies on as a specified behaviour. This
///    linear scan breaks ties by keeping the *first*-inserted entry
///    (strict `<`, not `<=`, when replacing the running best) -- a
///    concrete, deterministic rule rather than upstream's unspecified
///    one, not a claim of matching it bit-for-bit.
pub(crate) struct IkCache {
    entries: Vec<CacheEntry>,
    max_cache_size: usize,
    min_pose_distance: f64,
    min_config_distance2: f64,
}

impl IkCache {
    pub(crate) fn new(options: &IkCacheOptions) -> Self {
        Self {
            entries: Vec::with_capacity(options.max_cache_size),
            max_cache_size: options.max_cache_size,
            min_pose_distance: options.min_pose_distance,
            min_config_distance2: options.min_config_distance * options.min_config_distance,
        }
    }

    /// `IKCache::getBestApproximateIKSolution(const Pose&)`. An empty
    /// cache returns upstream's dummy entry: `pose` itself, paired with an
    /// all-zero config of length `num_joints`. That dummy is not a
    /// throwaway placeholder -- it is what [`IkCache::update`] gates the
    /// very first insertion against (see that method's doc comment), so a
    /// solver's first-ever solve only gets cached if the config it lands
    /// on differs from all-zeros by more than
    /// [`IkCacheOptions::min_config_distance`] (the pose-distance half of
    /// the gate is always `0.0` on an empty cache, since the dummy's pose
    /// *is* the query pose).
    pub(crate) fn nearest(&self, pose: &Isometry3, num_joints: usize) -> CacheEntry {
        let Some(best) = self
            .entries
            .iter()
            .min_by(|a, b| pose_distance(&a.pose, pose).total_cmp(&pose_distance(&b.pose, pose)))
        else {
            return CacheEntry {
                pose: *pose,
                config: vec![0.0; num_joints],
            };
        };
        best.clone()
    }

    /// `IKCache::updateCache(const IKEntry&, const Pose&, const
    /// vector<double>&)`: insert `(pose, config)` only if the cache has
    /// room (`entries.len() < max_cache_size`) AND `nearest` -- the same
    /// entry [`IkCache::nearest`] returned for this `pose` right before
    /// the caller's solve attempt, not a value re-queried here -- is
    /// "different enough" from the new entry in *either* space: pose
    /// distance over [`IkCacheOptions::min_pose_distance`], **or**
    /// squared config distance over the squared
    /// [`IkCacheOptions::min_config_distance`]. This is an OR, not an AND
    /// -- an entry close in pose but far in config (or vice versa) still
    /// gets cached, since either axis of novelty is enough to justify
    /// keeping both.
    pub(crate) fn update(&mut self, nearest: &CacheEntry, pose: &Isometry3, config: &[f64]) {
        if self.entries.len() >= self.max_cache_size {
            return;
        }
        let novel_pose = pose_distance(&nearest.pose, pose) > self.min_pose_distance;
        let novel_config = config_distance2(&nearest.config, config) > self.min_config_distance2;
        if novel_pose || novel_config {
            self.entries.push(CacheEntry {
                pose: *pose,
                config: config.to_vec(),
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use moveit_geometry::{UnitQuaternion, Vector3};

    fn pose_at(x: f64) -> Isometry3 {
        Isometry3::from_parts(Vector3::new(x, 0.0, 0.0).into(), Default::default())
    }

    #[test]
    fn empty_cache_returns_the_query_pose_paired_with_an_all_zero_config() {
        let cache = IkCache::new(&IkCacheOptions::default());
        let query = pose_at(3.0);
        let nearest = cache.nearest(&query, 4);
        assert_eq!(nearest.pose, query);
        assert_eq!(nearest.config(), [0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn nearest_picks_the_closer_of_two_entries_by_pose_distance() {
        let mut cache = IkCache::new(&IkCacheOptions {
            min_pose_distance: 0.0,
            min_config_distance: 0.0,
            ..IkCacheOptions::default()
        });
        let far_seed = cache.nearest(&pose_at(0.0), 1);
        cache.update(&far_seed, &pose_at(10.0), &[1.0]);
        let near_seed = cache.nearest(&pose_at(10.0), 1);
        cache.update(&near_seed, &pose_at(1.0), &[2.0]);

        let nearest = cache.nearest(&pose_at(0.9), 1);
        assert_eq!(nearest.config(), [2.0]);
    }

    fn pose_with_yaw(yaw: f64) -> Isometry3 {
        Isometry3::from_parts(
            Vector3::new(0.0, 0.0, 0.0).into(),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw),
        )
    }

    /// Every other test here holds orientation at identity and varies only
    /// position, so a `pose_distance` that silently dropped its
    /// `angle_to` term (position-only) would still pass them all. Both
    /// entries below share the same position; only orientation differs,
    /// isolating that term.
    #[test]
    fn nearest_picks_the_closer_of_two_entries_by_orientation_distance() {
        let mut cache = IkCache::new(&IkCacheOptions {
            min_pose_distance: 0.0,
            min_config_distance: 0.0,
            ..IkCacheOptions::default()
        });
        let far_seed = cache.nearest(&pose_with_yaw(0.0), 1);
        cache.update(&far_seed, &pose_with_yaw(2.0), &[1.0]);
        let near_seed = cache.nearest(&pose_with_yaw(2.0), 1);
        cache.update(&near_seed, &pose_with_yaw(0.2), &[2.0]);

        let nearest = cache.nearest(&pose_with_yaw(0.1), 1);
        assert_eq!(nearest.config(), [2.0]);
    }

    #[test]
    fn tie_break_keeps_the_first_inserted_entry() {
        let mut cache = IkCache::new(&IkCacheOptions {
            min_pose_distance: 0.0,
            min_config_distance: 0.0,
            max_cache_size: 5000,
        });
        let seed = cache.nearest(&pose_at(0.0), 1);
        cache.update(&seed, &pose_at(1.0), &[100.0]);
        let seed = cache.nearest(&pose_at(0.0), 1);
        cache.update(&seed, &pose_at(-1.0), &[200.0]);

        // Both entries are exactly `1.0` away from the query pose.
        let nearest = cache.nearest(&pose_at(0.0), 1);
        assert_eq!(nearest.config(), [100.0]);
    }

    #[test]
    fn update_inserts_when_pose_distance_alone_clears_the_threshold() {
        let mut cache = IkCache::new(&IkCacheOptions {
            min_pose_distance: 5.0,
            min_config_distance: 100.0,
            ..IkCacheOptions::default()
        });
        let seed = cache.nearest(&pose_at(0.0), 1);
        // Config distance is 0 (identical config), but pose distance (10)
        // clears `min_pose_distance` (5) -- the OR must still insert.
        cache.update(&seed, &pose_at(10.0), &[0.0]);
        assert_eq!(cache.entries.len(), 1);
    }

    #[test]
    fn update_inserts_when_config_distance_alone_clears_the_threshold() {
        let mut cache = IkCache::new(&IkCacheOptions {
            min_pose_distance: 100.0,
            min_config_distance: 1.0,
            ..IkCacheOptions::default()
        });
        let seed = cache.nearest(&pose_at(0.0), 1);
        // Pose distance is 0 (identical pose), but config distance (5)
        // clears `min_config_distance` (1) -- the OR must still insert.
        cache.update(&seed, &pose_at(0.0), &[5.0]);
        assert_eq!(cache.entries.len(), 1);
    }

    #[test]
    fn update_rejects_when_neither_distance_clears_its_threshold() {
        let mut cache = IkCache::new(&IkCacheOptions {
            min_pose_distance: 100.0,
            min_config_distance: 100.0,
            ..IkCacheOptions::default()
        });
        let seed = cache.nearest(&pose_at(0.0), 1);
        cache.update(&seed, &pose_at(0.1), &[0.1]);
        assert_eq!(cache.entries.len(), 0);
    }

    #[test]
    fn update_refuses_once_the_cache_is_full() {
        let mut cache = IkCache::new(&IkCacheOptions {
            max_cache_size: 1,
            min_pose_distance: 0.0,
            min_config_distance: 0.0,
        });
        let seed = cache.nearest(&pose_at(0.0), 1);
        cache.update(&seed, &pose_at(1.0), &[1.0]);
        assert_eq!(cache.entries.len(), 1);

        let seed = cache.nearest(&pose_at(50.0), 1);
        cache.update(&seed, &pose_at(50.0), &[50.0]);
        assert_eq!(
            cache.entries.len(),
            1,
            "cache must not grow past max_cache_size"
        );
    }

    /// `update`'s pose gate is strict `>`, matching upstream's `Pose::
    /// distance(...) > options_.min_pose_distance`. `pose_at(1.0)` against
    /// `pose_at(0.0)` gives a translation-only `pose_distance` of exactly
    /// `1.0` (rotation identical on both sides, so `angle_to` contributes
    /// exactly `0.0`; `(1.0 - 0.0).norm() == 1.0` exactly, no rounding).
    /// The config side is held at `[0.0]` vs the dummy entry's own
    /// `[0.0]`, so its distance is exactly `0.0` and cannot itself clear a
    /// gate.
    ///
    /// A geometric input cannot be nudged by one ULP and expect
    /// `pose_distance`'s output to move by exactly one ULP in turn --
    /// `.norm()` is a `sqrt`, a nonlinear transform that does not preserve
    /// ULP spacing the way `cart_to_jnt.rs:313`'s plain subtraction does
    /// (`f8fa9d8`). `min_pose_distance` itself has no such transform
    /// between the option and the comparison, so nudging *it* by one ULP
    /// (`f64::from_bits`) is the exact-boundary probe here, not the
    /// query pose.
    #[test]
    fn pose_gate_rejects_exactly_at_the_threshold_and_inserts_one_ulp_past_it() {
        let at_threshold = IkCacheOptions {
            max_cache_size: 5000,
            min_pose_distance: 1.0,
            min_config_distance: 1000.0,
        };
        let mut cache = IkCache::new(&at_threshold);
        let seed = cache.nearest(&pose_at(0.0), 1);
        cache.update(&seed, &pose_at(1.0), &[0.0]);
        assert_eq!(
            cache.entries.len(),
            0,
            "pose_distance == min_pose_distance must not clear a strict > gate"
        );

        let one_ulp_under = IkCacheOptions {
            max_cache_size: 5000,
            min_pose_distance: f64::from_bits(1.0f64.to_bits() - 1),
            min_config_distance: 1000.0,
        };
        let mut cache = IkCache::new(&one_ulp_under);
        let seed = cache.nearest(&pose_at(0.0), 1);
        cache.update(&seed, &pose_at(1.0), &[0.0]);
        assert_eq!(
            cache.entries.len(),
            1,
            "pose_distance one ULP past min_pose_distance must clear the gate"
        );
    }

    /// `update`'s config gate is strict `>` in the *squared* space:
    /// `configDistance2(...) > min_config_distance2_`, and
    /// `min_config_distance2_` is `min_config_distance * min_config_distance`
    /// (`IkCache::new`). With `min_config_distance = 1.0`, the threshold is
    /// exactly `1.0` (`1.0 * 1.0` has no rounding). Unlike the pose gate,
    /// `config_distance2` never takes a square root, so nudging the input
    /// by a chosen amount *can* move its output by exactly one ULP:
    /// `1.0.powi(2) + (2f64.powi(-26)).powi(2) == f64::from_bits(1.0_f64.
    /// to_bits() + 1)` bit-for-bit (`2.0_f64.powi(-26)` squares to exactly
    /// `2.0_f64.powi(-52)`, the ULP at `1.0`, and `1.0 + that` needs no
    /// rounding since the sum is itself exactly representable) --
    /// confirmed by compiling and running the actual arithmetic, not
    /// derived on paper alone.
    #[test]
    fn config_gate_rejects_exactly_at_the_threshold_and_inserts_one_ulp_past_it() {
        let options = IkCacheOptions {
            max_cache_size: 5000,
            min_pose_distance: 1000.0,
            min_config_distance: 1.0,
        };
        let mut cache = IkCache::new(&options);
        let seed = cache.nearest(&pose_at(0.0), 2);
        cache.update(&seed, &pose_at(0.0), &[1.0, 0.0]);
        assert_eq!(
            cache.entries.len(),
            0,
            "config_distance2 == min_config_distance2 must not clear a strict > gate"
        );

        let mut cache = IkCache::new(&options);
        let seed = cache.nearest(&pose_at(0.0), 2);
        let one_ulp_past = 2f64.powi(-26);
        cache.update(&seed, &pose_at(0.0), &[1.0, one_ulp_past]);
        assert_eq!(
            cache.entries.len(),
            1,
            "config_distance2 one ULP past min_config_distance2 must clear the gate"
        );
    }
}
