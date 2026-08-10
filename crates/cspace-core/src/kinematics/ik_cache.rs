// Copyright (c) 2017, Rice University
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_kinematics/cached_ik_kinematics_plugin/include/moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_plugin.hpp
//   moveit_kinematics/cached_ik_kinematics_plugin/src/ik_cache.cpp

use std::path::Path;

use crate::error::{Error, Result};
use crate::geometry::Isometry3;

mod format;

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
    /// `min_joint_config_distance`: the other half of the same test.
    ///
    /// Held un-squared, as upstream's `Options` field is, and squared at
    /// each comparison rather than once into a `min_config_distance2_`
    /// member as upstream's `initializeCache` does. The squaring has no
    /// exact inverse, so a stored square would have to be square-rooted
    /// back to write `format`'s `min_config_distance` field, and
    /// `sqrt(x * x)` is not `x` for every `x` — the cache would then load
    /// under a threshold a hair from the one it was saved under. One
    /// multiply per `IkCache::update` call buys the property that what
    /// the caller configured is exactly what a round trip returns.
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
/// single-tip only (see `crate::kinematics::registry::KinematicsSolver`'s `#
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

/// [`IkCache::update`]'s finiteness gate: every translation and rotation
/// component finite. `pose.rotation` is a `UnitQuaternion`, normalized at
/// construction from whatever its caller passed in -- normalizing a NaN
/// component keeps it NaN, it does not clear it.
fn is_finite_pose(pose: &Isometry3) -> bool {
    pose.translation.vector.iter().all(|c| c.is_finite())
        && pose.rotation.coords.iter().all(|c| c.is_finite())
}

/// `IKCache::configDistance2`: plain squared-Euclidean distance over joint
/// configs, no per-joint weighting.
///
/// `zip` truncates to the shorter slice rather than panicking, so this
/// function cannot itself detect a length disagreement -- the guarantee
/// that it never sees one is [`IkCache`]'s, not this function's: every
/// config that reaches an [`IkCache`] is exactly [`IkCache::num_joints`]
/// long, enforced at each of the three ways one can get in
/// ([`IkCache::nearest`]'s dummy is built at that length,
/// [`IkCache::update`] asserts it, and `format::from_json` rejects a
/// document that disagrees).
///
/// Upstream instead loops `i < config1.size()` while indexing `config2[i]`
/// unchecked, so the same disagreement reads out of bounds there -- see
/// `doc/upstream-bugs.md`, `get-best-approximate-static-dummy-stale`, for
/// how upstream reaches it.
fn config_distance2(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| (x - y).powi(2)).sum()
}

/// A cache of inverse-kinematics solutions, keyed by end-effector pose --
/// `IKCache`, minus the GNAT nearest-neighbor index.
///
/// # Deviations from upstream
///
/// 1. **A different on-disk format, and saving only when asked.**
///    `ik_cache.cpp`'s `saveCache`/`initializeCache` read and write a raw,
///    unversioned `memcpy` of host-endian `double` fields. Nothing outside
///    that one C++ class ever reads it, so this port owes it no
///    byte-layout compatibility and does not claim any: [`IkCache::save`]
///    and [`IkCache::load`] go through `format`, whose module doc states
///    what the format is, why it is `serde_json`, and which parts of
///    upstream's save *policy* (the every-500-entries write inside
///    `updateCache`, the write from `~IKCache`) are not ported.
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
#[derive(Debug)]
pub(crate) struct IkCache {
    entries: Vec<CacheEntry>,
    num_joints: usize,
    options: IkCacheOptions,
}

impl IkCache {
    /// `num_joints` is upstream's `initializeCache(..., num_joints, ...)`
    /// argument, held as a field rather than re-supplied per query.
    ///
    /// Upstream holds it the same way (`num_joints_`) but only ever uses it
    /// to size the empty-cache dummy; nothing there compares it against the
    /// configs actually stored, which is what lets a cache file written for
    /// a different arm be loaded and indexed out of bounds
    /// (`doc/upstream-bugs.md`, `ik-cache-read-trusts-file-header`). Here it
    /// is the cache's one joint count: every stored config is this long, and
    /// every seed handed out is too.
    pub(crate) fn new(options: &IkCacheOptions, num_joints: usize) -> Self {
        Self {
            entries: Vec::with_capacity(options.max_cache_size),
            num_joints,
            options: options.clone(),
        }
    }

    /// Write this cache to `path`, replacing whatever is there.
    ///
    /// # Errors
    ///
    /// Whatever `format::to_json` reports, or [`Error::Other`] naming
    /// `path` if the write itself fails.
    pub(crate) fn save(&self, path: &Path) -> Result<()> {
        let text = format::to_json(self)?;
        std::fs::write(path, text)
            .map_err(|error| Error::other(format!("writing {}: {error}", path.display())))
    }

    /// Read back a cache [`IkCache::save`] wrote, for a solver with
    /// `num_joints` joints.
    ///
    /// # Errors
    ///
    /// Whatever `format::from_json` reports -- including every way the
    /// file can fail to describe a cache for a `num_joints`-joint solver
    /// -- or [`Error::Other`] naming `path` if the read itself fails.
    pub(crate) fn load(path: &Path, num_joints: usize) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .map_err(|error| Error::other(format!("reading {}: {error}", path.display())))?;
        format::from_json(&text, num_joints)
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
    ///
    /// The dummy is rebuilt on every call. Upstream's is a function-local
    /// `static` initialized on the first call and then returned unchanged
    /// forever after, which is a bug, not an optimization -- see
    /// `doc/upstream-bugs.md`, `get-best-approximate-static-dummy-stale`.
    pub(crate) fn nearest(&self, pose: &Isometry3) -> CacheEntry {
        let Some(best) = self
            .entries
            .iter()
            .min_by(|a, b| pose_distance(&a.pose, pose).total_cmp(&pose_distance(&b.pose, pose)))
        else {
            return CacheEntry {
                pose: *pose,
                config: vec![0.0; self.num_joints],
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
    ///
    /// Upstream's room-to-grow half of that gate is
    /// `ik_cache_.size() < ik_cache_.capacity()`, not `< max_cache_size_`;
    /// see `doc/upstream-bugs.md`, `update-cache-capacity-as-size-limit`,
    /// for why those are not the same bound.
    ///
    /// # Panics
    ///
    /// If `config.len()` is not this cache's [`IkCache::num_joints`]. That
    /// is the invariant `config_distance2` relies on, and a violation is a
    /// caller error in the same sense a mis-sized `seed` is to
    /// [`crate::kinematics::KinematicsSolver::solve_with_options`] -- not an outcome to
    /// report.
    pub(crate) fn update(&mut self, nearest: &CacheEntry, pose: &Isometry3, config: &[f64]) {
        assert_eq!(
            config.len(),
            self.num_joints,
            "an IkCache holding {}-joint configs was handed a {}-joint one",
            self.num_joints,
            config.len()
        );
        if self.entries.len() >= self.options.max_cache_size {
            return;
        }
        // A non-finite pose or config must never reach `self.entries`,
        // independent of the novelty gate below: `novel_pose`/`novel_config`
        // decide only whether an entry is *different enough* to keep both,
        // and a NaN `pose_distance` makes `novel_pose` false (every NaN
        // comparison is), not "reject this entry" -- `novel_config` alone
        // being true (the ordinary case, since a fresh solve's config
        // rarely lands exactly on `nearest.config`) would still push a
        // NaN-poisoned pose in. Once in, [`IkCache::nearest`]'s `total_cmp`
        // never panics on it, but IEEE-754 `totalOrder` can still rank a
        // NaN distance as the minimum of every future query -- pinning a
        // wrong seed as "nearest" forever, for every pose, not just this
        // one.
        if !is_finite_pose(pose) || config.iter().any(|c| !c.is_finite()) {
            return;
        }
        let min_config_distance2 =
            self.options.min_config_distance * self.options.min_config_distance;
        let novel_pose = pose_distance(&nearest.pose, pose) > self.options.min_pose_distance;
        let novel_config = config_distance2(&nearest.config, config) > min_config_distance2;
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
    use crate::geometry::{UnitQuaternion, Vector3};

    fn pose_at(x: f64) -> Isometry3 {
        Isometry3::from_parts(Vector3::new(x, 0.0, 0.0).into(), Default::default())
    }

    #[test]
    fn empty_cache_returns_the_query_pose_paired_with_an_all_zero_config() {
        let cache = IkCache::new(&IkCacheOptions::default(), 4);
        let query = pose_at(3.0);
        let nearest = cache.nearest(&query);
        assert_eq!(nearest.pose, query);
        assert_eq!(nearest.config(), [0.0, 0.0, 0.0, 0.0]);
    }

    /// The boundary between "no entries" and "entries": with exactly one,
    /// `nearest` must return it whatever the query pose is, since the
    /// `min_by` has nothing to compare against and the empty-cache dummy
    /// must no longer be reachable.
    #[test]
    fn a_single_entry_is_returned_however_far_the_query_pose_is() {
        let mut cache = IkCache::new(
            &IkCacheOptions {
                min_pose_distance: 0.0,
                min_config_distance: 0.0,
                ..IkCacheOptions::default()
            },
            1,
        );
        let seed = cache.nearest(&pose_at(0.0));
        cache.update(&seed, &pose_at(1.0), &[42.0]);

        assert_eq!(cache.nearest(&pose_at(1.0)).config(), [42.0]);
        assert_eq!(cache.nearest(&pose_at(-900.0)).config(), [42.0]);
    }

    #[test]
    fn nearest_picks_the_closer_of_two_entries_by_pose_distance() {
        let mut cache = IkCache::new(
            &IkCacheOptions {
                min_pose_distance: 0.0,
                min_config_distance: 0.0,
                ..IkCacheOptions::default()
            },
            1,
        );
        let far_seed = cache.nearest(&pose_at(0.0));
        cache.update(&far_seed, &pose_at(10.0), &[1.0]);
        let near_seed = cache.nearest(&pose_at(10.0));
        cache.update(&near_seed, &pose_at(1.0), &[2.0]);

        let nearest = cache.nearest(&pose_at(0.9));
        assert_eq!(nearest.config(), [2.0]);
    }

    /// Confirmed on the branch before `IkCache::update`'s finiteness gate
    /// existed: `novel_pose` is `false` for a NaN `pose_distance` (every
    /// comparison against NaN is), but `novel_config` alone -- true here,
    /// since `[222.0]` is far from the seed's `[111.0]` -- still pushed the
    /// NaN-translation pose into `self.entries`. With a *negative*-signed
    /// NaN specifically, IEEE-754 `totalOrder` (what `total_cmp` uses)
    /// ranks it below every finite distance, so `nearest`'s `min_by`
    /// selected that entry for *every* future query pose, not just one
    /// that happened to be close to it: querying at `5.0`, genuinely
    /// closest to the untouched `[111.0]` entry, returned `[222.0]`
    /// instead -- a different, wrong answer handed back through
    /// `IkCache::nearest`'s return value, not merely a NaN observed
    /// mid-computation. A positive-signed NaN (Rust's plain `f64::NAN`)
    /// total-orders *above* every finite distance instead, so it is never
    /// selected -- silently wasting a cache slot rather than hijacking
    /// every lookup, which is why this test uses `-f64::NAN` to exercise
    /// the sign that actually corrupts a returned answer.
    #[test]
    fn update_rejects_a_pose_with_a_nan_translation_component() {
        let mut cache = IkCache::new(
            &IkCacheOptions {
                min_pose_distance: 0.0,
                min_config_distance: 0.0,
                ..IkCacheOptions::default()
            },
            1,
        );
        let seed1 = cache.nearest(&pose_at(0.0));
        cache.update(&seed1, &pose_at(5.0), &[111.0]);

        let nan_pose =
            Isometry3::from_parts(Vector3::new(-f64::NAN, 0.0, 0.0).into(), Default::default());
        let seed2 = cache.nearest(&pose_at(5.0));
        cache.update(&seed2, &nan_pose, &[222.0]);

        assert_eq!(
            cache.entries.len(),
            1,
            "a NaN-translation pose must not be cached, novel_config or not"
        );
        assert_eq!(
            cache.nearest(&pose_at(5.0)).config(),
            [111.0],
            "the genuinely closest entry must still win"
        );
    }

    #[test]
    fn update_rejects_a_nan_config_value() {
        let mut cache = IkCache::new(
            &IkCacheOptions {
                min_pose_distance: 0.0,
                min_config_distance: 0.0,
                ..IkCacheOptions::default()
            },
            1,
        );
        let seed = cache.nearest(&pose_at(0.0));
        cache.update(&seed, &pose_at(5.0), &[f64::NAN]);
        assert_eq!(
            cache.entries.len(),
            0,
            "a NaN config value must not be cached"
        );
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
        let mut cache = IkCache::new(
            &IkCacheOptions {
                min_pose_distance: 0.0,
                min_config_distance: 0.0,
                ..IkCacheOptions::default()
            },
            1,
        );
        let far_seed = cache.nearest(&pose_with_yaw(0.0));
        cache.update(&far_seed, &pose_with_yaw(2.0), &[1.0]);
        let near_seed = cache.nearest(&pose_with_yaw(2.0));
        cache.update(&near_seed, &pose_with_yaw(0.2), &[2.0]);

        let nearest = cache.nearest(&pose_with_yaw(0.1));
        assert_eq!(nearest.config(), [2.0]);
    }

    fn pose_at_with_yaw(x: f64, yaw: f64) -> Isometry3 {
        Isometry3::from_parts(
            Vector3::new(x, 0.0, 0.0).into(),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw),
        )
    }

    /// The two tests above each hold one term of [`pose_distance`] at zero,
    /// so either one alone would still pass against a metric that used
    /// `max` instead of `+`, or that scaled one term against the other.
    /// Here the two terms disagree about the winner and the *sum* decides:
    /// the query sits at the origin with yaw 0, so the near entry is
    /// `0.8 + 0.2 = 1.0` away and the far one `0.5 + 0.6 = 1.1`. Reading
    /// position alone picks the other entry (`0.5 < 0.8`), and so does
    /// `max(0.5, 0.6) < max(0.8, 0.2)`, and so does any weighting that
    /// scales the position term up.
    #[test]
    fn nearest_adds_the_two_terms_rather_than_ranking_on_either_one() {
        let mut cache = IkCache::new(
            &IkCacheOptions {
                min_pose_distance: 0.0,
                min_config_distance: 0.0,
                ..IkCacheOptions::default()
            },
            1,
        );
        let seed = cache.nearest(&pose_at_with_yaw(0.0, 0.0));
        cache.update(&seed, &pose_at_with_yaw(0.5, 0.6), &[1.0]);
        let seed = cache.nearest(&pose_at_with_yaw(0.5, 0.6));
        cache.update(&seed, &pose_at_with_yaw(0.8, 0.2), &[2.0]);

        let nearest = cache.nearest(&pose_at_with_yaw(0.0, 0.0));
        assert_eq!(nearest.config(), [2.0]);
    }

    #[test]
    fn tie_break_keeps_the_first_inserted_entry() {
        let mut cache = IkCache::new(
            &IkCacheOptions {
                min_pose_distance: 0.0,
                min_config_distance: 0.0,
                max_cache_size: 5000,
            },
            1,
        );
        let seed = cache.nearest(&pose_at(0.0));
        cache.update(&seed, &pose_at(1.0), &[100.0]);
        let seed = cache.nearest(&pose_at(0.0));
        cache.update(&seed, &pose_at(-1.0), &[200.0]);

        // Both entries are exactly `1.0` away from the query pose.
        let nearest = cache.nearest(&pose_at(0.0));
        assert_eq!(nearest.config(), [100.0]);
    }

    #[test]
    fn update_inserts_when_pose_distance_alone_clears_the_threshold() {
        let mut cache = IkCache::new(
            &IkCacheOptions {
                min_pose_distance: 5.0,
                min_config_distance: 100.0,
                ..IkCacheOptions::default()
            },
            1,
        );
        let seed = cache.nearest(&pose_at(0.0));
        // Config distance is 0 (identical config), but pose distance (10)
        // clears `min_pose_distance` (5) -- the OR must still insert.
        cache.update(&seed, &pose_at(10.0), &[0.0]);
        assert_eq!(cache.entries.len(), 1);
    }

    #[test]
    fn update_inserts_when_config_distance_alone_clears_the_threshold() {
        let mut cache = IkCache::new(
            &IkCacheOptions {
                min_pose_distance: 100.0,
                min_config_distance: 1.0,
                ..IkCacheOptions::default()
            },
            1,
        );
        let seed = cache.nearest(&pose_at(0.0));
        // Pose distance is 0 (identical pose), but config distance (5)
        // clears `min_config_distance` (1) -- the OR must still insert.
        cache.update(&seed, &pose_at(0.0), &[5.0]);
        assert_eq!(cache.entries.len(), 1);
    }

    #[test]
    fn update_rejects_when_neither_distance_clears_its_threshold() {
        let mut cache = IkCache::new(
            &IkCacheOptions {
                min_pose_distance: 100.0,
                min_config_distance: 100.0,
                ..IkCacheOptions::default()
            },
            1,
        );
        let seed = cache.nearest(&pose_at(0.0));
        cache.update(&seed, &pose_at(0.1), &[0.1]);
        assert_eq!(cache.entries.len(), 0);
    }

    /// The invariant `config_distance2`'s `zip` relies on, checked at the
    /// one place a config can enter a cache. Without the assertion the
    /// mismatch is silent: `zip` would compare the first joint only and
    /// store a 1-element config in a 2-joint cache, which the next
    /// `nearest` would then hand back as a seed of the wrong length.
    #[test]
    #[should_panic(expected = "an IkCache holding 2-joint configs was handed a 1-joint one")]
    fn update_rejects_a_config_that_is_not_the_caches_joint_count() {
        let mut cache = IkCache::new(&IkCacheOptions::default(), 2);
        let seed = cache.nearest(&pose_at(0.0));
        cache.update(&seed, &pose_at(1.0), &[5.0]);
    }

    #[test]
    fn update_refuses_once_the_cache_is_full() {
        let mut cache = IkCache::new(
            &IkCacheOptions {
                max_cache_size: 1,
                min_pose_distance: 0.0,
                min_config_distance: 0.0,
            },
            1,
        );
        let seed = cache.nearest(&pose_at(0.0));
        cache.update(&seed, &pose_at(1.0), &[1.0]);
        assert_eq!(cache.entries.len(), 1);

        let seed = cache.nearest(&pose_at(50.0));
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
        let mut cache = IkCache::new(&at_threshold, 1);
        let seed = cache.nearest(&pose_at(0.0));
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
        let mut cache = IkCache::new(&one_ulp_under, 1);
        let seed = cache.nearest(&pose_at(0.0));
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
    /// (formed inside [`IkCache::update`] here, rather than stored squared
    /// -- see [`IkCacheOptions::min_config_distance`] for why). With
    /// `min_config_distance = 1.0`, the threshold is
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
        let mut cache = IkCache::new(&options, 2);
        let seed = cache.nearest(&pose_at(0.0));
        cache.update(&seed, &pose_at(0.0), &[1.0, 0.0]);
        assert_eq!(
            cache.entries.len(),
            0,
            "config_distance2 == min_config_distance2 must not clear a strict > gate"
        );

        let mut cache = IkCache::new(&options, 2);
        let seed = cache.nearest(&pose_at(0.0));
        let one_ulp_past = 2f64.powi(-26);
        cache.update(&seed, &pose_at(0.0), &[1.0, one_ulp_past]);
        assert_eq!(
            cache.entries.len(),
            1,
            "config_distance2 one ULP past min_config_distance2 must clear the gate"
        );
    }

    /// A fresh, unique scratch directory per test, left for the OS to
    /// reclaim -- the same shape `cspace_core::model`'s `mesh_search_paths`
    /// tests use, for the same reason.
    fn scratch_dir() -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "cspace-kinematics-ik-cache-test-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// The round trip end to end: write a cache to a real file, read it
    /// back, and query the result for the same seeds. `format`'s own tests
    /// pin the document; this one pins the two `std::fs` calls between the
    /// document and the disk, and that a *loaded* cache answers `nearest`
    /// the way the saved one did rather than merely holding the right
    /// bytes.
    #[test]
    fn seeds_survive_a_round_trip_through_a_file() {
        let mut saved = IkCache::new(
            &IkCacheOptions {
                max_cache_size: 32,
                min_pose_distance: 0.25,
                min_config_distance: 0.25,
            },
            2,
        );
        for (x, config) in [(1.0, [0.5, -0.5]), (5.0, [1.5, -2.5])] {
            let seed = saved.nearest(&pose_at(x));
            saved.update(&seed, &pose_at(x), &config);
        }
        assert_eq!(saved.entries.len(), 2);

        let path = scratch_dir().join("panda_arm.ikcache.json");
        saved.save(&path).unwrap();
        let loaded = IkCache::load(&path, 2).unwrap();

        assert_eq!(loaded.nearest(&pose_at(1.1)).config(), [0.5, -0.5]);
        assert_eq!(loaded.nearest(&pose_at(4.9)).config(), [1.5, -2.5]);
        assert_eq!(loaded.options, saved.options);
    }
}
