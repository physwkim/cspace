// Copyright (c) 2017, Rice University
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_kinematics/cached_ik_kinematics_plugin/include/moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_plugin.hpp
//   moveit_kinematics/cached_ik_kinematics_plugin/include/moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_plugin-inl.hpp

use std::path::Path;

use cspace_error::Result;
use cspace_geometry::Isometry3;
use cspace_model::RobotModel;

use crate::ik_cache::{IkCache, IkCacheOptions};
use crate::lma::LevenbergMarquardtSolver;
use crate::newton_raphson::NewtonRaphsonSolver;
use crate::params::SolverParams;
use crate::registry::{KINEMATICS_SOLVERS, KinematicsSolver, SolveOptions, SolverRegistration};

/// `CachedIKKinematicsPlugin<KinematicsPlugin>`: wraps any
/// [`KinematicsSolver`] and, on every solve, first tries the wrapped
/// solver with the nearest previously-seen `(pose, solution)` pair as the
/// seed instead of the caller's own `seed`; if that fails, it retries with
/// the caller's original `seed` (upstream's own fallback, not a new
/// behaviour); a solution from either attempt is cached for next time.
///
/// # Deviation from upstream: six overrides collapse into one
///
/// Upstream's `CachedIKKinematicsPlugin<KinematicsPlugin>` overrides six
/// `KinematicsBase` virtuals with this identical caching pattern:
/// `initialize`, `getPositionIK`, and four `searchPositionIK` overloads
/// (base; +`consistency_limits`; +`solution_callback`;
/// +`consistency_limits`+`solution_callback`) --
/// `CachedMultiTipIKKinematicsPlugin` overrides two more for the rarer
/// multi-tip API, out of scope here (see
/// [`KinematicsSolver`]'s `# Deviations`, item 1). Five of the six
/// (everything but `initialize`) are byte-for-byte the same nearest-seed
/// try / original-seed fallback / cache-on-success logic, differing only
/// in which delegate overload they call -- confirmed by reading every
/// body in `cached_ik_kinematics_plugin-inl.hpp`, not assumed from the
/// name. This crate's own [`KinematicsSolver::solve_with_options`]
/// already collapses those same five upstream overloads into one method
/// (see that method's own doc comment, predating this type), so there is
/// only one method here to wrap. `initialize`, the sixth, has no
/// Rust-side counterpart to wrap: construction already happens once, in
/// [`SolverRegistration::construct`] (or [`CachedIkSolver::new`] directly),
/// with nothing split out into a later `initialize` call the way
/// upstream's plugin lifecycle needs.
///
/// # Deviation from upstream: no elapsed-time timeout budget
///
/// Upstream's fallback attempt runs with `timeout` reduced by however
/// long the nearest-seed attempt already took
/// (`diff.count()` in `cached_ik_kinematics_plugin-inl.hpp`), so the two
/// attempts together never exceed the caller's original `timeout`. This
/// crate has no wall-clock timeout at all (see [`SolverParams::max_restarts`]'s
/// doc comment) -- both attempts run with the same, full
/// `SolverParams::max_restarts` budget, so a cache-seed miss costs up to
/// twice the restarts a single [`crate::NewtonRaphsonSolver`]/
/// [`crate::LevenbergMarquardtSolver`] call would, not a
/// budget-constrained fraction of one call's worth.
///
/// # Why a caching solver must never be an implicit default
///
/// A [`CachedIkSolver`] is not a transparent drop-in for its wrapped
/// solver: for the same `(pose, seed)` query it can return a *different*
/// solution than calling the wrapped solver directly, even when the
/// caller's own `seed` is already exact. This is not a bug in this port --
/// it is upstream's own approximate-nearest-neighbor cache design. With an
/// empty cache, upstream's `IKCache::getBestApproximateIKSolution`
/// (`cached_ik_kinematics_plugin/src/ik_cache.cpp:159-168`) returns a
/// `static` all-zero dummy entry as the "nearest" match:
/// `std::make_pair(std::vector<Pose>(1, pose), std::vector<double>(num_joints_, 0.))`.
/// That all-zero seed is tried *first*, before the caller's own `seed`
/// (only tried as the fallback -- see the "no elapsed-time timeout budget"
/// deviation above for where that fallback is invoked from). A solver
/// that resolves each new IK request against a cold, empty cache -- which
/// is exactly what a caller doing one-shot resolution by name, rather than
/// holding onto a long-lived warmed instance, always does -- will silently
/// take a different solution branch than an uncached solver even when
/// nothing about the query changed. Anything that must match an uncached
/// solver's (or an external oracle's) output, or that resolves a solver
/// fresh per call rather than keeping one warm across many queries, must
/// not resolve to a `_cached` registration (see
/// [`crate::registry::DEFAULT_SOLVER_NAME`]'s own doc comment, which
/// points back here).
///
/// # Why this type has no oracle fixture
///
/// This crate's IK correctness (`newton_raphson`/`lma`) is checked
/// against `tools/moveit-oracle` live, through `tools/moveit-diff --ik`
/// runs -- there are no committed `Op::Ik` request/response JSON fixtures
/// for `cspace-kinematics` to replay (confirmed: no
/// `crates/cspace-kinematics/tests/fixtures/oracle-models.json`, unlike
/// every crate `tools/ci/verify-fixture-replay.sh` does cover). Either
/// way, `oracle.cpp`'s `Op::Ik` handler hand-transcribes
/// `KDLKinematicsPlugin::searchPositionIK`/`CartToJnt` directly (its own
/// doc comment says so) rather than loading the compiled plugin class
/// through `pluginlib::ClassLoader` -- confirmed by grep, the only
/// `pluginlib::ClassLoader` instantiation anywhere in `oracle.cpp` is for
/// `online_signal_smoothing`, unrelated to kinematics. There is no
/// `CachedIKKinematicsPlugin` in the oracle binary for a `--cached-ik`
/// flag to exercise even if `moveit-diff` grew one, and adding one would
/// not buy independent ground truth anyway: this type's caching
/// behaviour -- which seed gets tried, whether an entry gets inserted --
/// is exactly what this module's own `#[cfg(test)]` `FakeSolver`-based
/// tests already pin (see
/// `cache_hit_short_circuits_without_trying_the_callers_own_seed`/
/// `cache_miss_falls_back_to_the_callers_own_seed`), not a floating-point
/// computation an external oracle process would newly verify. The
/// wrapped solve itself (once a seed is chosen) is the same
/// [`KinematicsSolver::solve_with_options`] call
/// [`crate::NewtonRaphsonSolver`]/[`crate::LevenbergMarquardtSolver`]
/// already make under `moveit-diff --ik`.
pub struct CachedIkSolver<S> {
    inner: S,
    cache: IkCache,
}

impl<S: KinematicsSolver> CachedIkSolver<S> {
    /// Wrap `inner` with a fresh, empty `IkCache` configured by
    /// `options`.
    ///
    /// The cache's joint count comes from `inner` itself
    /// ([`KinematicsSolver::joint_names`]), which is the only count any
    /// seed or solution passing through this wrapper can have -- upstream
    /// reads the same number the same way
    /// (`KinematicsPlugin::getJointNames().size()`, passed to
    /// `initializeCache` in `cached_ik_kinematics_plugin-inl.hpp:64`).
    pub fn new(inner: S, options: IkCacheOptions) -> Self {
        let num_joints = inner.joint_names().len();
        Self {
            inner,
            cache: IkCache::new(&options, num_joints),
        }
    }

    /// Wrap `inner` with the cache [`CachedIkSolver::save_cache`] wrote to
    /// `path`, resuming from the seeds it holds.
    ///
    /// There is no `options` argument: the file carries the
    /// [`IkCacheOptions`] its entries were accumulated under, so a resumed
    /// cache gates insertions exactly as it did before. Upstream gets the
    /// same property by mangling all three option values into the file
    /// *name* and reading whichever file that name lands on; keeping them
    /// in the document means a caller cannot pair a file with options it
    /// was not built under, rather than merely being unlikely to.
    ///
    /// # Errors
    ///
    /// [`cspace_error::Error::Other`] if `path` cannot be read, is not a
    /// cache document, or is one written for a solver with a different
    /// joint count than `inner`'s. Upstream instead starts with an empty
    /// cache whenever `std::filesystem::exists` says no -- an absent file
    /// and an unreadable one are the same fact there, and neither is
    /// reported.
    pub fn from_cache_file(inner: S, path: &Path) -> Result<Self> {
        let cache = IkCache::load(path, inner.joint_names().len())?;
        Ok(Self { inner, cache })
    }

    /// Write this solver's accumulated cache to `path`, replacing whatever
    /// is there.
    ///
    /// Upstream saves from two places a caller never asks: inside
    /// `updateCache` once the cache has grown 500 entries past its last
    /// save, and again from `~IKCache`. Neither is ported -- see
    /// `ik_cache::format`'s module doc.
    ///
    /// # Errors
    ///
    /// [`cspace_error::Error::Other`] if `path` cannot be written, or if
    /// any cached value is one JSON cannot represent (see
    /// `ik_cache::format`'s `to_json`).
    pub fn save_cache(&self, path: &Path) -> Result<()> {
        self.cache.save(path)
    }
}

impl<S: KinematicsSolver> KinematicsSolver for CachedIkSolver<S> {
    fn group_name(&self) -> &str {
        self.inner.group_name()
    }

    fn joint_names(&self) -> &[String] {
        self.inner.joint_names()
    }

    fn base_frame(&self) -> &str {
        self.inner.base_frame()
    }

    fn tip_frame(&self) -> &str {
        self.inner.tip_frame()
    }

    /// Forwarded rather than inherited. The provided default would answer
    /// `[self.tip_frame()]`, which reports one tip for a wrapped solver
    /// that has several — a wrapper must not narrow what it wraps.
    fn tip_frames(&self) -> Vec<String> {
        self.inner.tip_frames()
    }

    /// The cache hit/miss branch: try `target`'s nearest cached seed
    /// first (`IkCache::nearest`), and only fall back to the caller's
    /// own `seed` if that attempt returns [`None`]. Either attempt
    /// succeeding updates the cache (`IkCache::update`) against the
    /// *same* `nearest` value looked up before either attempt ran, not a
    /// value re-queried afterward -- matching upstream, which passes the
    /// one `nearest` reference obtained at the top of the method into
    /// `cache_.updateCache` regardless of which attempt produced the
    /// solution.
    fn solve_with_options(
        &mut self,
        seed: &[f64],
        target: &Isometry3,
        options: &mut SolveOptions,
    ) -> Option<Vec<f64>> {
        let nearest = self.cache.nearest(target);
        let mut solution = self
            .inner
            .solve_with_options(nearest.config(), target, options);
        if solution.is_none() {
            solution = self.inner.solve_with_options(seed, target, options);
        }
        if let Some(ref solved) = solution {
            self.cache.update(&nearest, target, solved);
        }
        solution
    }
}

/// `IkCacheOptions::default()`, matching upstream's own
/// `IKCache::Options()` defaults exactly -- see
/// [`IkCacheOptions::default`]'s doc comment.
fn registered_cache_options() -> IkCacheOptions {
    IkCacheOptions::default()
}

// See `registry::KINEMATICS_SOLVERS`'s doc comment: this crate's
// `unsafe_code` lint is `allow`, so the `distributed_slice` macro's
// generated static needs no source-level suppression here.
//
// Upstream answers `SolverRegistration::construct`'s bare-fn-pointer
// shape (no generic `CachedIkSolver<S>` can itself appear once in
// `KINEMATICS_SOLVERS` parameterized by which solver it wraps) the same
// way: one `.cpp` file per instantiation
// (`cached_kdl_kinematics_plugin.cpp`, `cached_ur_kinematics_plugin.cpp`
// -- the latter excluded here, see `lib.rs`'s module doc). This crate
// ships two solvers, so two registrations, not a generic third entry.
#[linkme::distributed_slice(KINEMATICS_SOLVERS)]
static NEWTON_RAPHSON_CACHED: SolverRegistration = SolverRegistration {
    name: "newton_raphson_cached",
    construct: |model: &RobotModel,
                group_name: &str,
                params: &SolverParams|
     -> Result<Box<dyn KinematicsSolver>> {
        NewtonRaphsonSolver::new(model, group_name, params).map(|solver| {
            Box::new(CachedIkSolver::new(solver, registered_cache_options()))
                as Box<dyn KinematicsSolver>
        })
    },
};

#[linkme::distributed_slice(KINEMATICS_SOLVERS)]
static LMA_CACHED: SolverRegistration = SolverRegistration {
    name: "lma_cached",
    construct: |model: &RobotModel,
                group_name: &str,
                params: &SolverParams|
     -> Result<Box<dyn KinematicsSolver>> {
        LevenbergMarquardtSolver::new(model, group_name, params).map(|solver| {
            Box::new(CachedIkSolver::new(solver, registered_cache_options()))
                as Box<dyn KinematicsSolver>
        })
    },
};

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::rc::Rc;

    use super::*;

    /// A [`KinematicsSolver`] whose `solve_with_options` is scripted: it
    /// records every `seed` it is called with (through a shared handle a
    /// test can inspect after `solver` -- and this fake -- have been moved
    /// into a [`CachedIkSolver`]), and converges only when `seed` equals
    /// `accepts_seed`. That lets a test observe exactly which seed(s)
    /// [`CachedIkSolver`] tried, in what order, without depending on any
    /// real numeric solver's convergence behaviour.
    struct FakeSolver {
        joint_names: Vec<String>,
        accepts_seed: Vec<f64>,
        calls: Rc<RefCell<Vec<Vec<f64>>>>,
    }

    impl KinematicsSolver for FakeSolver {
        fn group_name(&self) -> &str {
            "fake"
        }

        fn joint_names(&self) -> &[String] {
            &self.joint_names
        }

        fn base_frame(&self) -> &str {
            "base"
        }

        fn tip_frame(&self) -> &str {
            "tip"
        }

        fn solve_with_options(
            &mut self,
            seed: &[f64],
            _target: &Isometry3,
            _options: &mut SolveOptions,
        ) -> Option<Vec<f64>> {
            self.calls.borrow_mut().push(seed.to_vec());
            (seed == self.accepts_seed.as_slice()).then(|| seed.to_vec())
        }
    }

    fn zero_gate_options() -> IkCacheOptions {
        IkCacheOptions {
            max_cache_size: 5000,
            min_pose_distance: 0.0,
            min_config_distance: 0.0,
        }
    }

    /// Pins the cache-miss half of the cache hit/miss branch: an empty
    /// cache's `nearest()` dummy (all-zero config) is tried first, and
    /// only once *that* fails does [`CachedIkSolver`] retry with the
    /// caller's own `seed` -- see `CachedIkSolver::solve_with_options`'s
    /// doc comment. Neutralizing the fallback (e.g. returning the
    /// nearest-seed attempt's `None` directly instead of retrying) makes
    /// this test fail: `result` would be `None`, not `Some([3.0])`.
    #[test]
    fn cache_miss_falls_back_to_the_callers_own_seed() {
        let calls = Rc::new(RefCell::new(Vec::new()));
        let inner = FakeSolver {
            joint_names: vec!["j1".to_string()],
            accepts_seed: vec![3.0],
            calls: Rc::clone(&calls),
        };
        let mut solver = CachedIkSolver::new(inner, zero_gate_options());
        let target = Isometry3::identity();

        let result = solver.solve_with_options(&[3.0], &target, &mut SolveOptions::default());

        assert_eq!(result, Some(vec![3.0]));
        assert_eq!(
            *calls.borrow(),
            vec![vec![0.0], vec![3.0]],
            "an empty cache must try the all-zero dummy seed before falling back to the caller's seed"
        );
    }

    /// Pins the cache-hit half of the cache hit/miss branch: once the
    /// cache holds a seed that itself converges, that nearest-seed
    /// attempt is *not* followed by a second attempt with the caller's
    /// own (different, and here rejecting) seed. Neutralizing the branch
    /// (e.g. always trying both seeds unconditionally) makes this test
    /// fail: `calls` would list two entries, not one.
    #[test]
    fn cache_hit_short_circuits_without_trying_the_callers_own_seed() {
        let calls = Rc::new(RefCell::new(Vec::new()));
        let inner = FakeSolver {
            joint_names: vec!["j1".to_string()],
            accepts_seed: vec![7.0],
            calls: Rc::clone(&calls),
        };
        let mut solver = CachedIkSolver::new(inner, zero_gate_options());
        let target = Isometry3::identity();

        // Prime the cache: the empty-cache dummy seed [0.0] is rejected,
        // the caller's own seed [7.0] converges and gets cached against
        // `target` (see `cache_miss_falls_back_to_the_callers_own_seed`).
        let primed = solver.solve_with_options(&[7.0], &target, &mut SolveOptions::default());
        assert_eq!(primed, Some(vec![7.0]));
        calls.borrow_mut().clear();

        // Solve again at the same pose with a *different*, rejecting
        // caller seed. The cache now holds [7.0] as `target`'s nearest
        // seed, so that's what gets tried -- and it converges immediately.
        let hit = solver.solve_with_options(&[99.0], &target, &mut SolveOptions::default());

        assert_eq!(hit, Some(vec![7.0]));
        assert_eq!(
            *calls.borrow(),
            vec![vec![7.0]],
            "a cache hit must not fall back to the caller's own seed"
        );
    }

    /// The other side of `if let Some(ref solved) = solution`: when both
    /// attempts fail there is nothing to cache, and caching anyway would
    /// poison every later query with a seed that is not a solution to
    /// anything. `zero_gate_options` opens both novelty gates, so the only
    /// thing that can keep this cache empty is the solve having failed.
    #[test]
    fn a_solve_that_fails_leaves_the_cache_empty() {
        let calls = Rc::new(RefCell::new(Vec::new()));
        let inner = FakeSolver {
            joint_names: vec!["j1".to_string()],
            accepts_seed: vec![f64::MAX],
            calls: Rc::clone(&calls),
        };
        let mut solver = CachedIkSolver::new(inner, zero_gate_options());
        let target = Isometry3::identity();

        let result = solver.solve_with_options(&[3.0], &target, &mut SolveOptions::default());

        assert_eq!(result, None);
        assert_eq!(
            *calls.borrow(),
            vec![vec![0.0], vec![3.0]],
            "both seeds must have been tried before giving up"
        );
        // Nothing was cached, so the next solve still starts from the
        // empty-cache dummy rather than from a remembered failure.
        calls.borrow_mut().clear();
        solver.solve_with_options(&[8.0], &target, &mut SolveOptions::default());
        assert_eq!(*calls.borrow(), vec![vec![0.0], vec![8.0]]);
    }

    /// [`CachedIkSolver::save_cache`] and
    /// [`CachedIkSolver::from_cache_file`] as a caller uses them: warm a
    /// cache through real solves, write it, and have a *new* wrapper around
    /// a fresh solver start from the seed the first one learned.
    #[test]
    fn a_saved_cache_seeds_the_next_solver_built_from_it() {
        let target = Isometry3::identity();
        let mut warm = CachedIkSolver::new(
            FakeSolver {
                joint_names: vec!["j1".to_string()],
                accepts_seed: vec![7.0],
                calls: Rc::new(RefCell::new(Vec::new())),
            },
            zero_gate_options(),
        );
        assert_eq!(
            warm.solve_with_options(&[7.0], &target, &mut SolveOptions::default()),
            Some(vec![7.0])
        );

        let path = std::env::temp_dir().join(format!(
            "cspace-kinematics-cached-solver-test-{}-{}.json",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        warm.save_cache(&path).unwrap();

        let calls = Rc::new(RefCell::new(Vec::new()));
        let mut resumed = CachedIkSolver::from_cache_file(
            FakeSolver {
                joint_names: vec!["j1".to_string()],
                accepts_seed: vec![7.0],
                calls: Rc::clone(&calls),
            },
            &path,
        )
        .unwrap();

        // A cold wrapper would try the all-zero dummy first (see
        // `cache_miss_falls_back_to_the_callers_own_seed`); this one has
        // `[7.0]` from the file, so that is the only seed tried.
        let hit = resumed.solve_with_options(&[99.0], &target, &mut SolveOptions::default());
        assert_eq!(hit, Some(vec![7.0]));
        assert_eq!(*calls.borrow(), vec![vec![7.0]]);
    }

    /// A cache file names the joint count it was built for, and
    /// [`CachedIkSolver::from_cache_file`] refuses one that disagrees with
    /// the solver it is wrapping -- the check upstream's `initializeCache`
    /// does not make (`doc/upstream-bugs.md`,
    /// `ik-cache-read-trusts-file-header`).
    #[test]
    fn a_cache_file_for_a_different_arm_is_refused() {
        let mut warm = CachedIkSolver::new(
            FakeSolver {
                joint_names: vec!["j1".to_string()],
                accepts_seed: vec![7.0],
                calls: Rc::new(RefCell::new(Vec::new())),
            },
            zero_gate_options(),
        );
        warm.solve_with_options(&[7.0], &Isometry3::identity(), &mut SolveOptions::default());

        let path = std::env::temp_dir().join(format!(
            "cspace-kinematics-cached-solver-mismatch-{}-{}.json",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        warm.save_cache(&path).unwrap();

        let two_joints = FakeSolver {
            joint_names: vec!["j1".to_string(), "j2".to_string()],
            accepts_seed: vec![7.0, 0.0],
            calls: Rc::new(RefCell::new(Vec::new())),
        };
        let Err(error) = CachedIkSolver::from_cache_file(two_joints, &path) else {
            panic!("a 1-joint cache file must not load into a 2-joint solver");
        };
        assert_eq!(
            error.to_string(),
            "ik cache holds 1-joint configs, but this solver has 2 joints"
        );
    }
}
