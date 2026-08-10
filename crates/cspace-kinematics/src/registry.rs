// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematics_base/include/moveit/kinematics_base/kinematics_base.hpp

use cspace_error::{Error, Result};
use cspace_geometry::Isometry3;
use cspace_model::RobotModel;

use crate::params::SolverParams;

/// Bundles `searchPositionIK`'s two behaviour-affecting extras beyond the
/// `(seed, target)` pair [`KinematicsSolver::solve`] already takes:
/// `consistency_limits` (`KDLKinematicsPlugin::checkConsistency`'s
/// per-active-joint bound on how far a solution may land from `seed`) and
/// `solution_callback` (`IKCallbackFn`'s accept/reject hook, upstream used
/// for e.g. collision-checking a candidate before accepting it). Both
/// default to `None`, which reproduces [`KinematicsSolver::solve`] exactly
/// — see [`KinematicsSolver::solve_with_options`]'s doc comment for how
/// each one gates a converged attempt.
///
/// # Deviation from upstream: `consistency_limits` is reduced-space
///
/// Upstream's `consistency_limits` parameter is full-space
/// (`dimension_`-sized, one entry per joint including mimics), filtered
/// down to `consistency_limits_mimic` (active-joint-sized) immediately
/// before use inside `searchPositionIK` — but then passed to
/// `checkConsistency`, which loops `i < dimension_` while indexing that
/// same active-joint-sized vector: an out-of-bounds `std::vector::operator[]`
/// read on any chain with at least one mimic joint. `consistency_limits`
/// here is reduced-space from the start (one entry per
/// [`KinematicsSolver::joint_names`] entry, the same space `seed` and the
/// returned solution already live in), which makes that mismatch
/// impossible by construction rather than porting it.
#[derive(Default)]
pub struct SolveOptions<'a> {
    /// One bound per [`KinematicsSolver::joint_names`] entry: a converged
    /// solution is rejected (and the attempt retried, subject to
    /// [`SolverParams::max_restarts`]) unless `|seed[i] - solution[i]| <=
    /// consistency_limits[i]` for every `i`.
    pub consistency_limits: Option<&'a [f64]>,
    /// Called on every attempt that converges numerically, with the
    /// candidate solution in [`KinematicsSolver::joint_names`] order.
    /// Returning `false` rejects it (the attempt is retried, subject to
    /// [`SolverParams::max_restarts`], exactly like a
    /// [`SolveOptions::consistency_limits`] rejection) instead of upstream's
    /// `IKCallbackFn` writing a non-`SUCCESS` `MoveItErrorCodes` — see
    /// [`KinematicsSolver::solve`]'s `# Deviation` on `Option` replacing
    /// `bool` plus an out parameter for why this crate has no
    /// `MoveItErrorCodes` to write through.
    pub solution_callback: Option<&'a mut SolutionCallback<'a>>,
}

/// [`SolveOptions::solution_callback`]'s type, named so that field does not
/// trip clippy's `type_complexity` lint.
pub type SolutionCallback<'a> = dyn FnMut(&[f64]) -> bool + 'a;

/// Replaces upstream `kinematics::KinematicsBase`: the interface every
/// numeric IK solver in this crate implements.
///
/// # Deviations from upstream
///
/// 1. **One tip, one pose, one seed-vs-solution shape.** Upstream's
///    `KinematicsBase` also supports multi-tip whole-body IK
///    (`getPositionIK(ik_poses: Vec<Pose>, ...)`) and IK cost functions.
///    Neither is exercised by `kdl_kinematics_plugin` — the solver this
///    crate actually ports — which never overrides the multi-tip
///    overload's default (single-pose-only) implementation and rejects
///    any cost function outright. This trait keeps only the shape
///    `kdl_kinematics_plugin` exercises; [`SolveOptions`] carries
///    `consistency_limits` and `solution_callback`, the two extras it
///    *does* exercise. [`KinematicsSolver::solve_with_options`] therefore
///    stays single-pose where upstream's `searchPositionIK` takes an
///    `ik_poses` vector. [`KinematicsSolver::tip_frames`] is nonetheless
///    plural, because `setFromIK`'s tip matching and its fill of the tips
///    a caller left out are written against the plural and stay N-ary in
///    [`crate::resolve_ik_queries`]; it defaults to the single
///    [`KinematicsSolver::tip_frame`], so a chain solver states one tip
///    without having to say so.
/// 2. **No timeout, no `KinematicsQueryOptions`.** See
///    [`SolverParams::max_restarts`]'s doc comment for the timeout
///    replacement; `return_approximate_solution` and
///    `lock_redundant_joints` are not ported (`kdl_kinematics_plugin`
///    itself only ever reads the first from its query options, and this
///    port has no redundancy-discretization path for the second to lock
///    against).
/// 3. **Seed and solution are reduced-space (active joints only).** See
///    `chain::ChainInfo`'s doc comment.
pub trait KinematicsSolver {
    /// `getGroupName`.
    fn group_name(&self) -> &str;

    /// `getJointNames`: the seed/solution vector's order.
    fn joint_names(&self) -> &[String];

    /// `getBaseFrame`: the frame every [`KinematicsSolver::solve`] `target`
    /// is given in (`chain::ChainInfo::root_pose_world`'s output frame).
    fn base_frame(&self) -> &str;

    /// `getTipFrame`: this chain's tip link.
    fn tip_frame(&self) -> &str;

    /// `getTipFrames`: every frame this solver expects a target pose for,
    /// in the order [`crate::resolve_ik_queries`] fills them.
    ///
    /// The default is the one-element list `[tip_frame()]`, which is what a
    /// chain solver has and what every solver this crate ships reports —
    /// see this trait's `# Deviations`, item 1. A solver that genuinely
    /// takes several tips overrides this, and so must any wrapper that
    /// delegates to one: inheriting the default there would silently
    /// present a multi-tip solver as single-tip.
    ///
    /// Owned rather than `&[String]` so the default body can exist at all;
    /// [`fn@crate::set_from_ik`] calls it once per request, not per attempt.
    fn tip_frames(&self) -> Vec<String> {
        vec![self.tip_frame().to_owned()]
    }

    /// `searchPositionIK`'s single-pose case in its fullest form (`timeout`
    /// replaced by [`SolverParams::max_restarts`] — see that field's doc
    /// comment), folding in `getPositionIK`'s single-attempt behaviour
    /// through `max_restarts = 0` rather than a separate method, and
    /// `consistency_limits`/`solution_callback` through `options` rather
    /// than four separate overloads the way upstream's virtual-dispatch
    /// interface needs to.
    ///
    /// `seed` and the returned solution are in [`KinematicsSolver::joint_names`]
    /// order (reduced space — see `chain::ChainInfo`'s doc
    /// comment); `target` is this solver's tip link's desired pose in the
    /// chain's own base-link frame (upstream's `base_frame_` —
    /// `chain::ChainInfo::root_pose_world`), not the model's world
    /// frame. A candidate that converges numerically but that `options`
    /// rejects (either gate) does not count as "no solution" outright —
    /// the attempt is retried exactly like a non-converging one, up to
    /// [`SolverParams::max_restarts`].
    ///
    /// # Deviation from upstream: `Option`, not `bool` plus an out
    /// parameter
    ///
    /// Upstream returns `bool` and writes through `solution`/`error_code`.
    /// Non-convergence is an ordinary negative outcome upstream itself
    /// never treats as an exception (`NO_SOLUTION`/`TIMED_OUT` are
    /// `MoveItErrorCodes`, not a thrown `Exception`) — [`None`] here plays
    /// that role; nothing about calling this method with a validly-shaped
    /// `seed` can fail in the sense [`cspace_error::Error`] means.
    ///
    /// # Panics
    ///
    /// If `seed.len()`, or `options.consistency_limits`' length when it is
    /// [`Some`], does not equal
    /// [`KinematicsSolver::joint_names`]`().len()`. Upstream rejects a
    /// mis-sized `consistency_limits` with `NO_IK_SOLUTION`
    /// (`kdl_kinematics_plugin.cpp:329`); this port treats it as the caller
    /// error it is, the same way it already treats a mis-sized `seed`,
    /// rather than reporting it through the same [`None`] a genuine
    /// no-solution uses.
    fn solve_with_options(
        &mut self,
        seed: &[f64],
        target: &Isometry3,
        options: &mut SolveOptions,
    ) -> Option<Vec<f64>>;

    /// [`KinematicsSolver::solve_with_options`] with `consistency_limits`
    /// and `solution_callback` both `None` — every existing caller's shape.
    ///
    /// # Panics
    ///
    /// See [`KinematicsSolver::solve_with_options`].
    fn solve(&mut self, seed: &[f64], target: &Isometry3) -> Option<Vec<f64>> {
        self.solve_with_options(seed, target, &mut SolveOptions::default())
    }
}

/// One [`KinematicsSolver`] implementation's compile-time registration.
///
/// Replaces upstream's `CLASS_LOADER_REGISTER_CLASS(ConcreteType,
/// kinematics::KinematicsBase)` — a pluginlib macro that lets a class be
/// looked up by name from a `.so` pluginlib never has to link against at
/// build time. Per `PORTING-PLAN.md` decision D4, that runtime, string-keyed
/// `dlopen` lookup is not ported: every solver this crate ships is linked in
/// at compile time and simply appears in [`KINEMATICS_SOLVERS`], a
/// `linkme::distributed_slice` scanned once at startup rather than resolved
/// per plugin request.
pub struct SolverRegistration {
    /// The name a caller scanning [`KINEMATICS_SOLVERS`] matches on.
    /// `"newton_raphson"`/`"lma"` for this crate's own two solvers.
    pub name: &'static str,
    /// Build one instance for `(model, group)`, or an [`cspace_error::Error`]
    /// if this solver cannot be built for that group (see
    /// `chain::ChainInfo::build`'s `# Errors`).
    pub construct: fn(&RobotModel, &str, &SolverParams) -> Result<Box<dyn KinematicsSolver>>,
}

/// Every [`KinematicsSolver`] this crate (or, later, another crate linked
/// into the same binary) registers. See [`SolverRegistration`]'s doc comment
/// for why this replaces pluginlib rather than reproducing it.
///
/// See `Cargo.toml`'s `[lints.rust]` comment: this crate sets
/// `unsafe_code = "allow"` (rather than the workspace's `forbid`) because
/// the `distributed_slice` macro expands to a `#[link_section]` static,
/// which the lint flags unconditionally. No `unsafe` block exists anywhere
/// in this crate's own source.
#[linkme::distributed_slice]
pub static KINEMATICS_SOLVERS: [SolverRegistration];

/// The [`KINEMATICS_SOLVERS`] entry a caller resolves to when it wants the
/// solver every fixture's oracle actually used: `panda.urdf`'s and
/// `fanuc.urdf`'s `kinematics.yaml` both configure
/// `kdl_kinematics_plugin/KDLKinematicsPlugin`, whose velocity IK step is
/// `KDL::ChainIkSolverVelMimicSVD` — [`crate::NewtonRaphsonSolver`]'s own
/// doc comment already calls it "the solver that ports
/// `ChainIkSolverVelMimicSVD` as-is". `"lma"` is this port's own addition,
/// not a solver upstream ships; a caller that wants oracle-matching numeric
/// output must not resolve to it, or to either `_cached` wrapper (see
/// [`crate::CachedIkSolver`]'s doc comment for why a caching solver is never
/// the right implicit default).
pub const DEFAULT_SOLVER_NAME: &str = "newton_raphson";

/// Resolve exactly the [`KINEMATICS_SOLVERS`] entry named `name` for
/// `(model, group_name)`.
///
/// Selection is by [`SolverRegistration::name`], never by
/// [`KINEMATICS_SOLVERS`]'s iteration order. That order is
/// `linkme`'s link-section placement order, a function of the whole
/// workspace's dependency graph — any crate anywhere adding an unrelated
/// dependency can silently reorder it. A caller that picks "the first
/// registration that constructs" is not applying a selection rule; it is
/// letting the linker apply one it never wrote down (PORTING-PLAN.md
/// §177).
///
/// # Errors
///
/// [`Error::UnknownName`] if no [`KINEMATICS_SOLVERS`] entry is registered
/// under `name`. Whatever [`SolverRegistration::construct`] itself returns
/// if a matching entry exists but cannot build for `(model, group_name)`
/// (see `chain::ChainInfo::build`'s `# Errors`).
pub fn resolve_solver(
    model: &RobotModel,
    group_name: &str,
    name: &str,
    params: &SolverParams,
) -> Result<Box<dyn KinematicsSolver>> {
    let registration = KINEMATICS_SOLVERS
        .iter()
        .find(|registration| registration.name == name)
        .ok_or_else(|| Error::unknown_name("kinematics solver", name))?;
    (registration.construct)(model, group_name, params)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `KINEMATICS_SOLVERS`' membership must not depend on where `linkme`
    /// happened to place each registration in the link section (PORTING-PLAN.md
    /// §177) -- checked as a set, never indexed or compared as an ordered
    /// sequence, so this test cannot itself become order-dependent.
    #[test]
    fn every_expected_registration_exists_regardless_of_slice_order() {
        let names: std::collections::HashSet<&str> =
            KINEMATICS_SOLVERS.iter().map(|r| r.name).collect();
        for expected in [
            "lma",
            "newton_raphson",
            "lma_cached",
            "newton_raphson_cached",
        ] {
            assert!(names.contains(expected), "missing registration: {expected}");
        }
    }

    /// [`DEFAULT_SOLVER_NAME`] must name a registration that actually exists
    /// -- this is the one property [`resolve_solver`] depends on to fail
    /// loudly (`Error::UnknownName`) rather than silently resolve nothing,
    /// if a future rename ever drops it out of sync.
    #[test]
    fn default_solver_name_names_a_real_registration() {
        assert!(
            KINEMATICS_SOLVERS
                .iter()
                .any(|r| r.name == DEFAULT_SOLVER_NAME)
        );
    }
}
