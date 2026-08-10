// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_state/include/moveit/robot_state/robot_state.hpp
//   moveit_core/robot_state/src/robot_state.cpp

//! `RobotState::setFromIK`: drive a [`KinematicsSolver`] from *frames the
//! caller names* rather than from the one tip link the solver was built
//! around, and write the answer back into a [`RobotState`].
//!
//! [`KinematicsSolver::solve_with_options`] takes a single pose already
//! expressed in the solver's own base frame, for the solver's own tip. Every
//! real caller has something else in hand: a pose of a gripper frame, or of
//! an attached object, stated in the model frame. The four jobs upstream's
//! `setFromIK` does between those two — resolving the caller's frame to a
//! solver tip, filling the tips the caller left out, applying a state-level
//! validity hook, and splitting a multi-tip request across subgroup solvers —
//! are what this module ports.
//!
//! # Where this lives, and why not next to `RobotState`
//!
//! Upstream `setFromIK` is a `RobotState` method. Here it cannot be: it needs
//! [`KinematicsSolver`], and `cspace_core::state` -> `cspace_core::kinematics` is a Cargo
//! cycle (`cspace_core::kinematics` already depends on `cspace_core::state`, see this
//! crate's `Cargo.toml`). So it is a free function in `cspace_core::kinematics`
//! taking `&mut RobotState`, over the edge that already exists — the same
//! placement, and the same reason, as [`crate::kinematics::CartesianInterpolator`]. No
//! new crate edge is added by this module.
//!
//! Attached bodies are the one thing that placement cannot reach directly:
//! this workspace keeps them on `cspace_planning::scene`'s `PlanningScene`, not on
//! `RobotState` (see that crate's `attached_body` module doc), and
//! `cspace_planning::scene` depends on `cspace_core::kinematics` transitively through
//! `cspace_planning::constraints`, so a `cspace_core::kinematics` -> `cspace_planning::scene` edge
//! would be a cycle too. [`AttachedFrames`] is how the caller injects them
//! instead: a one-method trait this module calls, implemented by whichever
//! crate holds both halves. [`NoAttachedFrames`] is the "the robot is
//! carrying nothing" answer.
//!
//! # Deviations from upstream
//!
//! 1. **The state is never left holding a rejected candidate.**
//!    Upstream hands the raw `RobotState*` to the
//!    `GroupStateValidityCallbackFn` without applying the candidate first,
//!    and every real callback applies it itself
//!    (`kinematics_service_capability.cpp:75-76`'s `isIKSolutionValid`,
//!    `trajectory_functions.cpp:581-582`'s `isStateColliding`, both
//!    `state->setJointGroupPositions(jmg, ik_solution); state->update();`).
//!    When no candidate is ever accepted, `setFromIK` returns `false` with
//!    the state holding the last *rejected* one. This port makes the
//!    opposite guarantee, by construction rather than by convention: see
//!    [`set_from_ik`]'s `# Invariant`. Recorded as
//!    `set-from-ik-leaves-a-rejected-candidate-in-the-state` in
//!    `doc/upstream-bugs.md`.
//!
//! 2. **The validity hook is given a state that already holds the
//!    candidate.** The direct consequence of item 1: this module applies the
//!    candidate, then calls [`GroupStateValidity`] with both the state and
//!    the same values in group-variable order. Upstream's two halves can
//!    disagree — a callback that forgets `setJointGroupPositions` silently
//!    validates whatever the state happened to hold — and here they cannot.
//!    Mimic joints are the concrete case: upstream's `ikCallbackFnAdapter`
//!    permutes the solver's full-space `ik_sol` (KDL's `dimension_` covers
//!    mimics) through the bijection, whereas
//!    [`KinematicsSolver::joint_names`] here is active-joints-only by
//!    design, so the group-variable slice can only carry correct mimic
//!    values if it is read back out of the state after the write.
//!
//! 3. **No wall-clock timeout, in either entry point.** `setFromIK`'s
//!    `timeout` and `setFromIKSubgroups`' `do { ... } while (elapsed <
//!    timeout)` are both excluded by `PORTING-PLAN.md` §4.9;
//!    [`SolverParams::max_restarts`](crate::kinematics::SolverParams::max_restarts)
//!    already bounds the per-solver re-seeding those loops drive, and
//!    [`set_from_ik_subgroups`]' `max_attempts` bounds the outer sweep.
//!
//! 4. **A multi-tip request is not routed to a multi-tip solver, because no
//!    solver in this crate is one.** [`resolve_ik_queries`] is fully N-ary —
//!    it matches every target to a tip and fills the rest — but
//!    [`KinematicsSolver::solve_with_options`] takes one pose (see
//!    [`KinematicsSolver`]'s own `# Deviations`, item 1), so [`set_from_ik`]
//!    rejects a solver reporting more than one
//!    [`KinematicsSolver::tip_frames`] entry rather than silently using only
//!    the first. That is upstream's own arrangement too, though by a
//!    shorter route than the plugin: `KDLKinematicsPlugin` — the solver
//!    this crate ports — does not override `supportsGroup`, so a multi-tip
//!    group reaches `KinematicsBase`' inherited one and fails it on
//!    `jmg->isChain()` alone (`kinematics_base.cpp:142-155`). That failure
//!    is exactly the branch that sends upstream into `setFromIKSubgroups`
//!    (`robot_state.cpp:1836-1866`), and [`set_from_ik_subgroups`] is that
//!    branch, ported.
//!
//! 5. **Consistency limits are one flat slice, not a vector of sets.**
//!    Upstream takes `vector<vector<double>>` and then rejects any size
//!    other than 0 or 1 for the single-solver path (`robot_state.cpp:1870-1877`),
//!    so the outer vector only ever carries what an `Option` carries. In
//!    [`set_from_ik_subgroups`], where the outer dimension would be real,
//!    each subgroup's limits ride on its own [`SolveOptions`] instead.

use crate::error::{Error, Result};
use crate::geometry::{Isometry3, Transforms};
use crate::model::joint::JointType;
use crate::model::{JointModelGroup, RobotModel};
use crate::state::{Posed, RobotState};

use crate::kinematics::registry::{KinematicsSolver, SolveOptions};

/// One `(pose, frame)` pair: upstream's parallel `poses_in[i]` / `tips_in[i]`.
///
/// Paired rather than parallel because upstream's own first act is to check
/// that the two vectors have the same length (`robot_state.cpp:1825-1829`) — a
/// check that cannot fail once the pair is the unit.
#[derive(Clone, Copy, Debug)]
pub struct IkTarget<'a> {
    /// Where `frame` should end up, in the model frame.
    pub pose: Isometry3,
    /// The frame `pose` describes. A link name, the model frame, or — when
    /// the caller supplies a non-empty [`AttachedFrames`] — an attached
    /// body or one of its subframes. A leading `/` is stripped, matching
    /// upstream's `pose_frame.substr(1)`.
    pub frame: &'a str,
}

/// A frame that is rigidly fixed to a link without being a link itself: an
/// attached body, or one of its subframes.
///
/// Both fields come from one [`AttachedFrames::attached_frame`] call
/// precisely so they cannot be fetched from different places and disagree —
/// `link_pose_frame` is only meaningful relative to `link_name`.
#[derive(Clone, Copy, Debug)]
pub struct AttachedFrame<'a> {
    /// The link the body is attached to (upstream
    /// `AttachedBody::getAttachedLinkName`).
    pub link_name: &'a str,
    /// The frame's pose in `link_name`'s own frame. Constant, because the
    /// attachment is rigid — which is the whole reason this is enough for
    /// [`set_from_ik`] to work without knowing anything else about the body.
    pub link_pose_frame: Isometry3,
}

/// The part of `RobotState`'s frame resolution that lives on `cspace_planning::scene`
/// in this workspace, injected rather than depended on.
///
/// Upstream `RobotState` answers `getLinkModelIncludingAttachedBodies` and
/// the attached-body tiers of `getFrameTransform` from its own
/// `attached_body_map_`. Here those live on `PlanningScene`, one layer up
/// (see `cspace_planning::scene`'s `attached_body` module doc), and a
/// `cspace_core::kinematics` -> `cspace_planning::scene` dependency would close a cycle
/// through `cspace_planning::constraints`. Implement this on whatever type does hold
/// them, or pass [`NoAttachedFrames`].
pub trait AttachedFrames {
    /// `getAttachedBody(frame)`/`hasSubframeTransform(frame)` collapsed into
    /// one lookup: [`None`] if `frame` is not an attached body or a subframe
    /// of one.
    ///
    /// Names that are links are *not* this trait's business — this module
    /// asks the [`RobotModel`] first and only falls through to here, matching
    /// `getLinkModelIncludingAttachedBodies`' own order
    /// (`robot_state.cpp:910-937`).
    fn attached_frame(&self, frame: &str) -> Option<AttachedFrame<'_>>;
}

/// The robot is carrying nothing: every [`AttachedFrames::attached_frame`]
/// lookup misses.
///
/// A named unit struct rather than an `impl AttachedFrames for ()`, so that
/// passing it is a visible statement about the robot and not something a
/// caller can arrive at by leaving an argument out.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoAttachedFrames;

impl AttachedFrames for NoAttachedFrames {
    fn attached_frame(&self, _frame: &str) -> Option<AttachedFrame<'_>> {
        None
    }
}

/// Upstream `GroupStateValidityCallbackFn`, the state-level accept/reject
/// hook `setFromIK` runs on a candidate solution.
///
/// Distinct from [`SolveOptions::solution_callback`], which this module
/// implements it *through*: that one is the solver's own `IKCallbackFn`, sees
/// only the reduced-space solution vector, and knows nothing about a
/// [`RobotState`]. This one is upstream's `bool(RobotState*, const
/// JointModelGroup*, const double* joint_group_variable_values)` — the hook a
/// collision check needs, because a collision check needs a posed state.
///
/// `values` is in [`JointModelGroup::variable_names`] order (upstream's
/// bijection target), and `state` already holds it: see this module's
/// `# Deviations`, item 2. Returning `false` rejects the candidate; the
/// solver then retries exactly as it would after a
/// [`SolveOptions::consistency_limits`] rejection.
pub type GroupStateValidity<'a, 'm> =
    dyn FnMut(&mut RobotState<'m>, &JointModelGroup, &[f64]) -> bool + 'a;

/// Everything [`set_from_ik`] needs beyond the state, the solver and the
/// targets: upstream's `constraint`, `consistency_limits`, and the attached
/// bodies upstream reads off the state itself.
///
/// Bundled rather than passed one by one because a caller that drives IK
/// through a longer computation carries them unchanged across every call —
/// [`crate::kinematics::CartesianInterpolator`] does exactly that, once per waypoint and
/// again for each bisection — and because [`IkContext::default`] is then the
/// whole of "plain IK, nothing attached, no extra gates".
pub struct IkContext<'a, 'm> {
    /// Where attached bodies and their subframes come from, since this crate
    /// cannot depend on the one that holds them. [`NoAttachedFrames`] if the
    /// robot is carrying nothing.
    pub attached: &'a dyn AttachedFrames,
    /// One bound per [`KinematicsSolver::joint_names`] entry, or [`None`] for
    /// upstream's empty `consistency_limits` vector. See
    /// [`SolveOptions::consistency_limits`], which this is passed straight
    /// through to.
    pub consistency_limits: Option<&'a [f64]>,
    /// Upstream's `GroupStateValidityCallbackFn`. See
    /// [`GroupStateValidity`].
    pub validity: Option<&'a mut GroupStateValidity<'a, 'm>>,
}

impl Default for IkContext<'_, '_> {
    fn default() -> Self {
        Self {
            attached: &NoAttachedFrames,
            consistency_limits: None,
            validity: None,
        }
    }
}

/// `setToIKSolverFrame`: restate `pose` (given in the model frame) in
/// `ik_frame`.
///
/// Upstream's `Transforms::sameFrame(ik_frame, model_frame)` short-circuit is
/// kept rather than folded into the general multiply: the model frame is not
/// required to name a link, and when it does not, the general path's
/// `global_link_transform` lookup is the failure that branch exists to avoid.
///
/// # Errors
///
/// [`Error::UnknownName`] if `ik_frame` is neither the model frame nor a
/// link. Upstream logs "The following IK frame does not exist" and returns
/// `false`.
fn to_solver_frame(posed: &Posed<'_, '_>, ik_frame: &str, pose: &Isometry3) -> Result<Isometry3> {
    if Transforms::same_frame(ik_frame, posed.model().model_frame()) {
        return Ok(*pose);
    }
    let ik_frame = ik_frame.strip_prefix('/').unwrap_or(ik_frame);
    Ok(posed.global_link_transform(ik_frame)?.inverse() * pose)
}

/// `RobotState::getRigidlyConnectedParentLinkModel(frame, nullptr)`: the link
/// whose motion carries `frame` with it, through fixed joints only.
///
/// Two frames sharing this link are interchangeable as an IK target, because
/// the transform between them cannot change — the fact the tip matching in
/// [`resolve_ik_queries`] is built on.
///
/// # Errors
///
/// [`Error::UnknownName`] if `frame` is neither a link nor a frame `attached`
/// knows.
fn rigid_parent_link(
    model: &RobotModel,
    attached: &dyn AttachedFrames,
    frame: &str,
) -> Result<usize> {
    let link_index = if model.has_link_model(frame) {
        model.link_model(frame)?.link_index()
    } else {
        let body = attached
            .attached_frame(frame)
            .ok_or_else(|| Error::unknown_name("IK frame", frame))?;
        model.link_model(body.link_name)?.link_index()
    };
    Ok(model.rigidly_connected_parent_link(link_index, None))
}

/// `RobotState::getFrameTransform(frame)` across both tiers: the model frame
/// and links from `cspace_core::state`, attached bodies and their subframes from
/// `attached`. Links are tried first, the same order
/// [`rigid_parent_link`] uses, so the two cannot resolve one name to
/// different things.
///
/// # Errors
///
/// [`Error::UnknownName`] if `frame` is neither of those.
fn frame_transform(
    posed: &Posed<'_, '_>,
    attached: &dyn AttachedFrames,
    frame: &str,
) -> Result<Isometry3> {
    let frame = frame.strip_prefix('/').unwrap_or(frame);
    let model = posed.model();
    if frame == model.model_frame() || model.has_link_model(frame) {
        return posed.frame_transform(frame);
    }
    let body = attached
        .attached_frame(frame)
        .ok_or_else(|| Error::unknown_name("IK frame", frame))?;
    Ok(posed.global_link_transform(body.link_name)? * body.link_pose_frame)
}

/// Every pose `solver` needs, in [`KinematicsSolver::tip_frames`] order and
/// in the solver's own base frame — upstream's `ik_queries` vector, built by
/// `setFromIK`'s two loops (`robot_state.cpp:1889-2007`).
///
/// The first loop matches each of `targets` to a solver tip it can reach:
/// directly, when the names agree, and otherwise through the rigid
/// connection they share, in which case the pose is carried across to the tip
/// by the constant transform between them. Each tip takes at most one target.
/// The second loop fills every tip no target claimed with that tip's
/// *current* pose, so a caller that names one tip of a two-tip solver is
/// asking for "move this one, hold the other where it is" rather than leaving
/// a pose undefined.
///
/// # Errors
///
/// - [`Error::UnknownName`] if a target's frame, or a solver tip, names
///   nothing this model or `attached` knows.
/// - [`Error::Other`] if some target is rigidly connected to no unclaimed
///   tip — which covers "more targets than tips" as the same failure rather
///   than as a separate arity check. Upstream logs "Cannot compute IK for
///   query %zu pose reference frame '%s'" and returns `false`; it is a caller
///   error here, not "no solution".
pub fn resolve_ik_queries(
    state: &mut RobotState<'_>,
    solver: &dyn KinematicsSolver,
    targets: &[IkTarget<'_>],
    attached: &dyn AttachedFrames,
) -> Result<Vec<Isometry3>> {
    let tips = solver.tip_frames();
    let base = solver.base_frame();
    let model = state.model();
    let posed = state.update();

    let mut queries = vec![Isometry3::identity(); tips.len()];
    let mut claimed = vec![false; tips.len()];

    for (i, target) in targets.iter().enumerate() {
        let pose_frame = target.frame.strip_prefix('/').unwrap_or(target.frame);
        let mut pose = to_solver_frame(&posed, base, &target.pose)?;

        let mut matched = None;
        for (tip_id, tip) in tips.iter().enumerate() {
            if claimed[tip_id] {
                continue;
            }
            let tip = tip.strip_prefix('/').unwrap_or(tip.as_str());
            if pose_frame == tip {
                matched = Some(tip_id);
                break;
            }
            if rigid_parent_link(model, attached, pose_frame)?
                == rigid_parent_link(model, attached, tip)?
            {
                // Both frames move with the same link, so this product is the
                // constant `pose_frame`-to-`tip` transform: asking the solver
                // for the tip here asks it for the same thing.
                pose = pose
                    * frame_transform(&posed, attached, pose_frame)?.inverse()
                    * frame_transform(&posed, attached, tip)?;
                matched = Some(tip_id);
                break;
            }
        }

        let Some(tip_id) = matched else {
            return Err(Error::other(format!(
                "IK target {i} frame {pose_frame:?} is not rigidly connected to any unclaimed \
                 tip frame of solver group {:?} (tip frames: {tips:?})",
                solver.group_name(),
            )));
        };
        claimed[tip_id] = true;
        queries[tip_id] = pose;
    }

    for (tip_id, tip) in tips.iter().enumerate() {
        if claimed[tip_id] {
            continue;
        }
        let current = frame_transform(&posed, attached, tip)?;
        queries[tip_id] = to_solver_frame(&posed, base, &current)?;
    }

    Ok(queries)
}

/// The group variables a solver's seed and solution vectors correspond to,
/// slot for slot: upstream's `getKinematicsSolverJointBijection`, carried by
/// name instead of by index.
///
/// Upstream builds an index permutation, because `ikCallbackFnAdapter` scatters
/// the solver's solution into group-variable order arithmetically. This module
/// cannot do that — [`KinematicsSolver::joint_names`] is active-joints-only,
/// so a mimic joint's group-variable slot has no solver entry to scatter from
/// — and instead writes the solution into the state by name and reads the
/// group's variables back out, which is what gives mimics their values (see
/// this module's `# Deviations`, item 2). These names are what that
/// permutation's indices were: the single list [`set_from_ik`]'s seed,
/// [`apply_and_read_group`]'s write and the final write of an accepted
/// solution are all indexed by, so no two of them can disagree about which
/// slot means which variable. Building it is also what makes
/// [`apply_and_read_group`]'s writes infallible.
///
/// A solver joint that is a *fixed* joint of `group` contributes no slot,
/// exactly as `computeJointVariableIndices` contributes none for it
/// (`joint_model_group.cpp:627-637`, whose `// skip reported fixed joints`
/// branch is 630-632): a fixed joint holds no variable, so a solver reporting
/// one is naming a joint it does not solve, and both the seed it is handed and
/// the solution it returns are one slot shorter than its name list.
///
/// # Errors
///
/// [`Error::UnknownName`] if a solver joint is neither a variable of `group`
/// nor a fixed joint of it — which includes a *multi-variable* joint of
/// `group` named by the joint rather than by its variables, since one slot
/// cannot carry its several values. Upstream logs "group '%s' does not contain
/// such a joint" and returns `false`.
fn solver_solution_variables<'j>(
    model: &RobotModel,
    group: &JointModelGroup,
    solver_joints: &'j [String],
) -> Result<Vec<&'j str>> {
    let mut variables = Vec::with_capacity(solver_joints.len());
    for name in solver_joints {
        if group.variable_names().iter().any(|v| v == name) {
            variables.push(name.as_str());
            continue;
        }
        if group.has_joint_model(name) && model.joint_model(name)?.joint_type() == JointType::Fixed
        {
            continue;
        }
        return Err(Error::unknown_name("group variable", name));
    }
    Ok(variables)
}

/// Write `solution` (slot for slot with `solution_variables`) into `state`,
/// then read the whole group's variables back out.
///
/// The read-back is not a convenience: mimic joints are group variables no
/// solver joint writes, and `RobotState::set_variable_position` is what gives
/// them their values. Deriving them here would be a second implementation of
/// the mimic rule.
///
/// # Panics
///
/// If any name involved is not a model variable, which
/// [`solver_solution_variables`] has already ruled out for
/// `solution_variables` and which cannot hold for a group's own variable list.
fn apply_and_read_group(
    state: &mut RobotState<'_>,
    group: &JointModelGroup,
    solution_variables: &[&str],
    solution: &[f64],
) -> Vec<f64> {
    for (name, value) in solution_variables.iter().zip(solution) {
        state
            .set_variable_position(name, *value)
            .expect("checked against the group's variable list before the solve");
    }
    group
        .variable_names()
        .iter()
        .map(|name| {
            state
                .variable_position(name)
                .expect("a group's variables are the model's variables")
        })
        .collect()
}

/// `RobotState::setFromIK`: solve `targets` with `solver` and write the answer
/// into `state`.
///
/// `targets` may name any frame rigidly connected to one of the solver's tips
/// — a gripper link, an attached object, a subframe of one — and may leave
/// tips out; [`resolve_ik_queries`] is the part that turns that into the
/// solver's own question. [`IkContext`] carries the rest of upstream's
/// parameter list.
///
/// Returns whether a solution was found. Failing to converge is not an
/// [`Error`] — see [`KinematicsSolver::solve_with_options`]'s own deviation
/// note.
///
/// # Invariant
///
/// **On anything but `Ok(true)`, `state` holds exactly the values it held on
/// entry; on `Ok(true)` it holds exactly the returned solution.** `state` is
/// the one place a caller reads the outcome from, so "solved" and "did not
/// solve" must not both be able to leave a solution in it. The gate is this
/// function: it snapshots `state`'s positions before the first solve and
/// restores them on every exit — including before applying the accepted
/// solution, since `validity` both sees the state and may write anywhere in
/// it. See this module's `# Deviations`, item 1, for what upstream does
/// instead.
///
/// # Errors
///
/// - Whatever [`resolve_ik_queries`] reports.
/// - [`Error::UnknownName`] if the solver's group is not in the model, or a
///   solver joint is neither one of its variables nor a fixed joint of it. A
///   fixed joint of the group takes no seed or solution slot, the way
///   upstream's `computeJointVariableIndices` gives it no bijection entry
///   (`joint_model_group.cpp:630-632`); anything else the group does not hold
///   is rejected.
/// - [`Error::Other`] if the solver reports more than one tip frame: see this
///   module's `# Deviations`, item 4.
pub fn set_from_ik<'m>(
    state: &mut RobotState<'m>,
    solver: &mut dyn KinematicsSolver,
    targets: &[IkTarget<'_>],
    ik: &mut IkContext<'_, 'm>,
) -> Result<bool> {
    let group = state.model().joint_model_group(solver.group_name())?;
    let joint_names = solver.joint_names().to_vec();
    let solution_variables = solver_solution_variables(state.model(), group, &joint_names)?;

    let queries = resolve_ik_queries(state, solver, targets, ik.attached)?;
    let [target] = queries.as_slice() else {
        return Err(Error::other(format!(
            "solver group {:?} reports {} tip frames; KinematicsSolver::solve_with_options takes \
             a single pose, so a request this wide belongs in set_from_ik_subgroups",
            solver.group_name(),
            queries.len(),
        )));
    };

    let entry_positions = state.positions().to_vec();
    let seed = solution_variables
        .iter()
        .map(|name| state.variable_position(name))
        .collect::<Result<Vec<f64>>>()?;

    // The hook reborrows `state`, so its borrow has to end before the state
    // is restored or the solution applied below. Written as two arms rather
    // than one `Option<Box<dyn FnMut ...>>`: `SolveOptions`' lifetime
    // parameter is invariant (it sits behind a `&mut`), which a boxed hook
    // cannot satisfy without outliving the very borrow that reaches it.
    let solution = match ik.validity {
        None => solver.solve_with_options(
            &seed,
            target,
            &mut SolveOptions {
                consistency_limits: ik.consistency_limits,
                solution_callback: None,
            },
        ),
        Some(ref mut validity) => {
            let hooked_state = &mut *state;
            let hooked_names = &solution_variables;
            let mut hook = move |candidate: &[f64]| {
                let group_values =
                    apply_and_read_group(hooked_state, group, hooked_names, candidate);
                validity(hooked_state, group, &group_values)
            };
            solver.solve_with_options(
                &seed,
                target,
                &mut SolveOptions {
                    consistency_limits: ik.consistency_limits,
                    solution_callback: Some(&mut hook),
                },
            )
        }
    };

    state.set_variable_positions(&entry_positions);
    let Some(solution) = solution else {
        return Ok(false);
    };
    for (name, value) in solution_variables.iter().zip(&solution) {
        state.set_variable_position(name, *value)?;
    }
    Ok(true)
}

/// `RobotState::setFromIKSubgroups`: one target per subgroup solver, solved
/// one subgroup at a time, with `validity` judging the assembled whole.
///
/// This is the branch upstream reaches when a multi-tip request meets a solver
/// that cannot take it (`robot_state.cpp:1836-1866`) — which, for the only
/// solver family this crate ships, is every multi-tip request. Each entry of
/// `solvers` names its own subgroup through [`KinematicsSolver::group_name`]
/// and is paired with `targets[i]`.
///
/// A sweep solves each subgroup in turn, seeding it from `state` as it then
/// stands — so a later subgroup sees what an earlier one just decided,
/// matching upstream's `setJointGroupPositions` inside the loop. If every
/// subgroup solves and `validity` accepts the assembled group vector, that
/// sweep is the answer. Otherwise the state is rewound and the next sweep
/// starts where the first one did; up to `max_attempts` sweeps run, and
/// `max_attempts == 0` solves nothing.
///
/// # Deviation: no random re-seeding between sweeps
///
/// Upstream seeds sweep 1 from the state and every later sweep from
/// `getVariableRandomPositions`. Here each solver already does that
/// internally — [`SolverParams::max_restarts`](crate::kinematics::SolverParams::max_restarts)
/// governs `search_position_ik`'s own random restarts — so a second sweep that
/// re-seeds from the state is not a repeat of the first. A second, outer
/// randomization would duplicate a knob the solver already owns.
///
/// # Invariant
///
/// The same one [`set_from_ik`] states, and for the same reason: on anything
/// but `Ok(true)`, `state` holds its entry values. Upstream does not — it
/// writes each subgroup's solution as that subgroup solves
/// (`robot_state.cpp:2229-2239`) and rewinds neither on the `break` that
/// abandons the sweep nor on the final `return false`. Recorded as
/// `set-from-ik-leaves-a-rejected-candidate-in-the-state` in
/// `doc/upstream-bugs.md`.
///
/// # Errors
///
/// - [`Error::Other`] if `solvers` and `targets` differ in length, if
///   `solvers` is empty, if a solver's group is not a subgroup of
///   `group_name`, if the same subgroup is supplied more than once, or if
///   the supplied solvers do not cover every one of `group_name`'s
///   sub-groups. Upstream checks the counts against `getSubgroups()`' length
///   and then takes the pairing positionally; pairing by the solver's own
///   group name instead makes a mis-ordered call impossible, and the
///   coverage check below reproduces upstream's count check without relying
///   on position.
/// - [`Error::UnknownName`] if `group_name` is not a group of the model.
/// - Whatever [`set_from_ik`] reports for an individual subgroup.
pub fn set_from_ik_subgroups<'m>(
    state: &mut RobotState<'m>,
    group_name: &str,
    solvers: &mut [Box<dyn KinematicsSolver>],
    targets: &[IkTarget<'_>],
    ik: &mut IkContext<'_, 'm>,
    max_attempts: usize,
) -> Result<bool> {
    let group = state.model().joint_model_group(group_name)?;
    if solvers.len() != targets.len() {
        return Err(Error::other(format!(
            "{} subgroup solvers for {} targets in group {group_name:?}",
            solvers.len(),
            targets.len(),
        )));
    }
    if solvers.is_empty() {
        return Err(Error::other(format!(
            "set_from_ik_subgroups needs at least one subgroup solver for group {group_name:?}"
        )));
    }
    let mut covered = std::collections::BTreeSet::new();
    for solver in solvers.iter() {
        if !group.is_subgroup(solver.group_name()) {
            return Err(Error::other(format!(
                "solver group {:?} is not a subgroup of {group_name:?}",
                solver.group_name(),
            )));
        }
        if !covered.insert(solver.group_name()) {
            return Err(Error::other(format!(
                "solver group {:?} was supplied more than once for {group_name:?}",
                solver.group_name(),
            )));
        }
    }
    // Upstream's positional count check (`poses_in.size() != sub_groups.size()`,
    // `robot_state.cpp:2062-2067`) against the model's *complete* sub-group
    // list -- not just "each supplied group is *a* sub-group" -- expressed
    // here as a coverage check, since this port pairs solvers to sub-groups
    // by name rather than by position. Without this, a caller supplying a
    // strict subset of the group's sub-groups (e.g. one arm's solver for a
    // two-arm group) silently sweeps only that subset, leaving the other
    // sub-group's joints untouched and still reporting `Ok(true)`.
    if covered.len() != group.subgroup_names().len() {
        return Err(Error::other(format!(
            "subgroup solvers cover {} of group {group_name:?}'s {} sub-groups",
            covered.len(),
            group.subgroup_names().len(),
        )));
    }

    // No validity hook and no consistency limits on the subgroup solves:
    // upstream runs the constraint once, on the assembled group, not once per
    // subgroup where most of the group's variables are still the previous
    // sweep's, and its `consistency_limits[sg]` is per-subgroup rather than
    // the whole group's (see this module's `# Deviations`, item 5).
    let mut sub_ik = IkContext {
        attached: ik.attached,
        consistency_limits: None,
        validity: None,
    };

    let entry_positions = state.positions().to_vec();
    for _ in 0..max_attempts {
        let mut swept = true;
        for (solver, target) in solvers.iter_mut().zip(targets) {
            if !set_from_ik(
                state,
                solver.as_mut(),
                std::slice::from_ref(target),
                &mut sub_ik,
            )? {
                swept = false;
                break;
            }
        }

        if swept {
            let swept_positions = state.positions().to_vec();
            let group_values = group
                .variable_names()
                .iter()
                .map(|name| state.variable_position(name))
                .collect::<Result<Vec<f64>>>()?;
            let accepted = match ik.validity {
                None => true,
                Some(ref mut validity) => validity(state, group, &group_values),
            };
            if accepted {
                // `validity` may have written the state; the sweep's own
                // answer is what this function promises.
                state.set_variable_positions(&swept_positions);
                return Ok(true);
            }
        }
        state.set_variable_positions(&entry_positions);
    }
    Ok(false)
}
