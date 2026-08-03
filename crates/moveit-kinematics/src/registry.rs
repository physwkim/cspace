// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematics_base/include/moveit/kinematics_base/kinematics_base.hpp

use moveit_error::Result;
use moveit_geometry::Isometry3;
use moveit_model::RobotModel;

use crate::params::SolverParams;

/// Replaces upstream `kinematics::KinematicsBase`: the interface every
/// numeric IK solver in this crate implements.
///
/// # Deviations from upstream
///
/// 1. **One tip, one pose, one seed-vs-solution shape.** Upstream's
///    `KinematicsBase` supports multi-tip whole-body IK
///    (`getPositionIK(ik_poses: Vec<Pose>, ...)`), IK cost functions, an
///    `IKCallbackFn` collision hook and a discretization-method enum for
///    redundancy resolution. `kdl_kinematics_plugin` — the solver this
///    crate actually ports — uses none of that: it is single-tip,
///    single-pose, no callback, `NO_DISCRETIZATION` only. This trait keeps
///    only the shape `kdl_kinematics_plugin` exercises.
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

    /// `searchPositionIK`'s single-pose, no-callback, `NO_DISCRETIZATION`
    /// case, folding in `getPositionIK`'s single-attempt behaviour through
    /// [`SolverParams::max_restarts`] rather than a separate method.
    ///
    /// `seed` and the returned solution are in [`KinematicsSolver::joint_names`]
    /// order (reduced space — see `chain::ChainInfo`'s doc
    /// comment); `target` is this solver's tip link's desired pose in the
    /// chain's own base-link frame (upstream's `base_frame_` —
    /// `chain::ChainInfo::root_pose_world`), not the model's world
    /// frame.
    ///
    /// # Deviation from upstream: `Option`, not `bool` plus an out
    /// parameter
    ///
    /// Upstream returns `bool` and writes through `solution`/`error_code`.
    /// Non-convergence is an ordinary negative outcome upstream itself
    /// never treats as an exception (`NO_SOLUTION`/`TIMED_OUT` are
    /// `MoveItErrorCodes`, not a thrown `Exception`) — [`None`] here plays
    /// that role; nothing about calling this method with a validly-shaped
    /// `seed` can fail in the sense [`moveit_error::Error`] means.
    ///
    /// # Panics
    ///
    /// If `seed.len()` does not equal
    /// [`KinematicsSolver::joint_names`]`().len()`.
    fn solve(&mut self, seed: &[f64], target: &Isometry3) -> Option<Vec<f64>>;
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
    /// Build one instance for `(model, group)`, or an [`moveit_error::Error`]
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
