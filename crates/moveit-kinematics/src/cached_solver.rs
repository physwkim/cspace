// Copyright (c) 2017, Rice University
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_kinematics/cached_ik_kinematics_plugin/include/moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_plugin.hpp
//   moveit_kinematics/cached_ik_kinematics_plugin/include/moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_plugin-inl.hpp

use moveit_error::Result;
use moveit_geometry::Isometry3;
use moveit_model::RobotModel;

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
pub struct CachedIkSolver<S> {
    inner: S,
    cache: IkCache,
}

impl<S> CachedIkSolver<S> {
    /// Wrap `inner` with a fresh, empty `IkCache` configured by
    /// `options`.
    pub fn new(inner: S, options: IkCacheOptions) -> Self {
        Self {
            inner,
            cache: IkCache::new(&options),
        }
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
        let nearest = self.cache.nearest(target, seed.len());
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
    construct: |model: &RobotModel, group_name: &str, params: &SolverParams| -> Result<Box<dyn KinematicsSolver>> {
        NewtonRaphsonSolver::new(model, group_name, params).map(|solver| {
            Box::new(CachedIkSolver::new(solver, registered_cache_options())) as Box<dyn KinematicsSolver>
        })
    },
};

#[linkme::distributed_slice(KINEMATICS_SOLVERS)]
static LMA_CACHED: SolverRegistration = SolverRegistration {
    name: "lma_cached",
    construct: |model: &RobotModel, group_name: &str, params: &SolverParams| -> Result<Box<dyn KinematicsSolver>> {
        LevenbergMarquardtSolver::new(model, group_name, params).map(|solver| {
            Box::new(CachedIkSolver::new(solver, registered_cache_options())) as Box<dyn KinematicsSolver>
        })
    },
};
