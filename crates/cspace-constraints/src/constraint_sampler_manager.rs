// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/constraint_samplers/include/moveit/constraint_samplers/constraint_sampler_manager.hpp
//   moveit_core/constraint_samplers/src/constraint_sampler_manager.cpp
//   (ConstraintSamplerManager::selectDefaultSampler)

//! [`select_default_sampler`]: round 10's port of upstream
//! `ConstraintSamplerManager::selectDefaultSampler` — the function that
//! decides, for one group and one constraint set, whether to hand back a
//! [`JointConstraintSampler`], an [`IkConstraintSamplerAdapter`], a
//! [`UnionConstraintSampler`] of several, or nothing.
//!
//! # `ConstraintSamplerManager` itself is not ported
//!
//! Upstream's `ConstraintSamplerManager` is two things: `selectSampler`, an
//! outer loop over a registered list of `ConstraintSamplerAllocatorPtr`
//! plugins (first one whose `canService()` returns true wins, falling back
//! to `selectDefaultSampler`); and `selectDefaultSampler` itself. The first
//! is runtime plugin-by-string dispatch — exactly what D4 already excludes
//! from this crate — so it is not ported at all; a caller who wants a
//! non-default sampler just constructs one directly, no registry needed.
//! What upstream calls "the manager" collapses, once that layer is gone, to
//! [`select_default_sampler`] alone: a free function, not a struct — there
//! is no `sampler_alloc_` list left to be a struct's field, and this
//! crate's other samplers ([`JointConstraintSampler`],
//! [`UnionConstraintSampler`]) already establish "no manager-with-registry"
//! as this crate's own shape.
//!
//! # The group→solver lookup becomes a caller-supplied argument
//!
//! Upstream reads `jmg->getGroupKinematics()` — a `(KinematicsSolver,
//! KinematicsSolverMap)` pair cached on `JointModelGroup` at model-load time
//! from SRDF `<group>` config plus `kinematics.yaml`'s plugin lookup — to
//! learn whether the group (or one of its immediate subgroups) has an IK
//! solver, and which one. `PORTING-PLAN.md` §68.4 already excludes that
//! lookup (D4; `cspace-model` carries no such map and is not touched this
//! round). [`select_default_sampler`]'s `solver` and `subgroup_solvers`
//! parameters are what the caller passes in its place — the same
//! caller-supplies-the-solver decision [`crate::IkConstraintSampler::new`]
//! already made, now threaded one level up.
//!
//! # Subgroup nesting is unbounded, matching upstream
//!
//! `constraint_sampler_manager.cpp:322-380`: Step C's loop calls
//! `selectDefaultSampler(scene, it->first->getName(), sub_constr)` —
//! the *whole* function, recursively, not a stripped inner variant — and
//! that call reads `jmg->getGroupKinematics()` fresh for whatever group it
//! was given. There is no depth check anywhere in the C++: recursion goes
//! exactly as deep as the model's group hierarchy and its
//! `kinematics.yaml` solver assignments do. Nothing in this port's own
//! algorithm caps it either: `select_default_sampler`'s `subgroup_solvers`
//! is `Vec<`[`SubgroupSolver`]`>`, and each [`SubgroupSolver`] carries its
//! *own* `subgroup_solvers` for the recursive call to pass on — the
//! caller-supplied tree upstream would instead discover lazily, one
//! `getGroupKinematics()` call per recursion level, from a single global
//! per-model map. None of this crate's three model fixtures (panda, fanuc,
//! dual-arm panda) define a group two subgroup-levels deep with a solver at
//! the bottom (`panda_arm_hand`'s only subgroups, `panda_arm` and `hand`,
//! are themselves leaf chains) — `tests/constraint_sampler_manager.rs`
//! covers depth 2 with a group hierarchy built in-test via
//! `SrdfModel::parse_str`, since no fixture demonstrates it.
//!
//! # No cross-link tie-break: `used` can hold at most one link, by construction
//!
//! Upstream's `IKConstraintSampler::configure` accepts a constrained link
//! that is either the solver's own tip frame or *fixed-jointed* to it
//! (`default_constraint_samplers.cpp`, the `fixed_links` walk around each
//! `getLinkModel()` check), so `used_l` there is a `std::map` that can
//! legitimately collect more than one link key from a single solver, with a
//! final `if (v < msv)` reduction loop (`constraint_sampler_manager.cpp:303`;
//! strict `<`, so a tie keeps `used_l`'s first — alphabetically-earliest —
//! entry) picking one winner. [`crate::IkConstraintSampler`] ports none of
//! that fixed-link bridging (see its own doc comment) and requires the
//! constrained link to equal `solver.tip_frame()` exactly. Since
//! [`collect_ik_candidate`] shares one solver (one fixed tip frame) across
//! every candidate it builds, every successful candidate names that same
//! tip frame — there is structurally never more than one link to choose
//! among. [`collect_ik_candidate`] returns `Option<(String,
//! IkConstraintSamplerAdapter)>` rather than a map precisely so that stays
//! true by construction: a >1-link reduction has no state to operate on, so
//! there is no runtime branch to leave untested or to get its tie direction
//! wrong. If fixed-link bridging is ever added to [`crate::IkConstraintSampler`],
//! this is the type to widen back into a map with a real reduction, at
//! which point upstream's own tie direction (above) is what to match.

use std::cell::RefCell;
use std::collections::BTreeSet;
use std::rc::Rc;

use cspace_error::Result;
use cspace_kinematics::KinematicsSolver;
use cspace_model::{JointModelGroup, RobotModel};

use crate::{
    Constraint, ConstraintSampler, IkConstraintSamplerAdapter, IkSamplingPose, JointConstraint,
    JointConstraintSampler, OrientationConstraint, PositionConstraint, UnionConstraintSampler,
};

/// One entry of a caller-supplied `subgroup_solvers` tree: `group_name` is
/// the subgroup this solver directly targets, `solver` is its allocator,
/// and `subgroup_solvers` is that same subgroup's *own* immediate-subgroup
/// solvers — what upstream would discover by calling
/// `jmg->getGroupKinematics().second` again after recursing into
/// `group_name` (see this module's doc comment on why recursion is
/// unbounded and must be threaded explicitly here instead).
pub struct SubgroupSolver {
    /// The subgroup this solver directly targets.
    pub group_name: String,
    /// `group_name`'s own IK allocator.
    pub solver: Box<dyn KinematicsSolver>,
    /// `group_name`'s own immediate-subgroup solvers, recursively.
    pub subgroup_solvers: Vec<SubgroupSolver>,
}

/// `ConstraintSamplerManager::selectDefaultSampler`.
///
/// `solver` stands in for upstream's `jmg->getGroupKinematics().first` — the
/// group's own IK solver, if any. `subgroup_solvers` stands in for
/// `jmg->getGroupKinematics().second` — one [`SubgroupSolver`] per immediate
/// subgroup that has one, each carrying its own further subgroup solvers in
/// turn (see this module's doc comment on why recursion is unbounded).
/// `max_attempts` is baked into any [`IkConstraintSamplerAdapter`] this
/// builds, replacing the `max_attempts` argument upstream's
/// `ConstraintSampler::sample` takes per-call (see `sampler.rs`'s own doc
/// comment on why [`ConstraintSampler::sample`] here takes no such
/// parameter).
///
/// # Errors
///
/// Whatever [`RobotModel::joint_model_group`] returns for a name in
/// `subgroup_solvers` that is not actually a subgroup of `group_name` in
/// `model` — upstream instead trusts its own internally-consistent
/// `KinematicsSolverMap`, but this port's map is caller-supplied, so a
/// caller-passed name that does not resolve is reported rather than
/// silently skipped. An unresolvable `group_name` itself is not an error
/// (see below): it matches upstream's own `if (!jmg) return
/// ConstraintSamplerPtr();`.
pub fn select_default_sampler(
    model: &RobotModel,
    group_name: &str,
    constraints: &[Constraint],
    solver: Option<Box<dyn KinematicsSolver>>,
    subgroup_solvers: Vec<SubgroupSolver>,
    max_attempts: u32,
) -> Result<Option<Box<dyn ConstraintSampler>>> {
    let Ok(group) = model.joint_model_group(group_name) else {
        return Ok(None);
    };
    let group = group.clone();

    let joints: Vec<&JointConstraint> = constraints
        .iter()
        .filter_map(|c| match c {
            Constraint::Joint(j) => Some(j),
            _ => None,
        })
        .collect();
    let positions: Vec<&PositionConstraint> = constraints
        .iter()
        .filter_map(|c| match c {
            Constraint::Position(p) => Some(p),
            _ => None,
        })
        .collect();
    let orientations: Vec<&OrientationConstraint> = constraints
        .iter()
        .filter_map(|c| match c {
            Constraint::Orientation(o) => Some(o),
            _ => None,
        })
        .collect();

    select_default_sampler_inner(
        model,
        &group,
        GroupConstraints {
            joints: &joints,
            positions: &positions,
            orientations: &orientations,
        },
        solver,
        subgroup_solvers,
        max_attempts,
    )
}

/// One group's constraints, split by kind — upstream's `moveit_msgs::msg::Constraints`
/// already carries `joint_constraints`/`position_constraints`/`orientation_constraints`
/// as three parallel vectors; this is that same split, over this crate's
/// already-resolved constraint types instead of raw messages. Bundled so
/// [`select_default_sampler_inner`]'s own recursion (Step C, below) has one
/// thing to narrow to a subgroup's constraints, not three.
struct GroupConstraints<'a> {
    joints: &'a [&'a JointConstraint],
    positions: &'a [&'a PositionConstraint],
    orientations: &'a [&'a OrientationConstraint],
}

fn select_default_sampler_inner(
    model: &RobotModel,
    group: &JointModelGroup,
    constraints: GroupConstraints<'_>,
    solver: Option<Box<dyn KinematicsSolver>>,
    subgroup_solvers: Vec<SubgroupSolver>,
    max_attempts: u32,
) -> Result<Option<Box<dyn ConstraintSampler>>> {
    let GroupConstraints {
        joints,
        positions,
        orientations,
    } = constraints;

    // Step A: if there are joint constraints, see whether they cover every
    // one of the group's own variables.
    let mut samplers: Vec<Box<dyn ConstraintSampler>> = Vec::new();
    if !joints.is_empty() {
        let owned: Vec<JointConstraint> = joints.iter().map(|j| (*j).clone()).collect();
        if let Ok(sampler) = JointConstraintSampler::new(model, group.name(), &owned) {
            if joint_coverage_is_full(group, joints) {
                return Ok(Some(Box::new(sampler)));
            }
            // Partial coverage: keep it as a fallback/union candidate, but
            // keep looking for position/orientation constraints too.
            samplers.push(Box::new(sampler));
        }
    }

    // Step B: this group's own IK solver, if any.
    if let Some(solver) = solver {
        let shared: Rc<RefCell<Box<dyn KinematicsSolver>>> = Rc::new(RefCell::new(solver));
        if let Some((_, winner)) =
            collect_ik_candidate(model, group, positions, orientations, &shared, max_attempts)
        {
            if samplers.is_empty() {
                return Ok(Some(Box::new(winner)));
            }
            samplers.push(Box::new(winner));
            return Ok(Some(Box::new(UnionConstraintSampler::new(
                model,
                group.name(),
                samplers,
            )?)));
        }
    }

    // Step C: subgroups with their own IK solvers.
    if !subgroup_solvers.is_empty() {
        let mut used_p: BTreeSet<usize> = BTreeSet::new();
        let mut used_o: BTreeSet<usize> = BTreeSet::new();
        let mut some_sampler_valid = false;

        for entry in subgroup_solvers {
            let SubgroupSolver {
                group_name: subgroup_name,
                solver: subgroup_solver,
                subgroup_solvers: nested_subgroup_solvers,
            } = entry;
            let Ok(subgroup) = model.joint_model_group(&subgroup_name) else {
                return Err(cspace_error::Error::unknown_name("group", &subgroup_name));
            };

            let mut sub_positions: Vec<&PositionConstraint> = Vec::new();
            for (i, pc) in positions.iter().enumerate() {
                if !used_p.contains(&i) && subgroup.has_link_model(pc.link_name()) {
                    used_p.insert(i);
                    sub_positions.push(pc);
                }
            }
            let mut sub_orientations: Vec<&OrientationConstraint> = Vec::new();
            for (i, oc) in orientations.iter().enumerate() {
                if !used_o.contains(&i) && subgroup.has_link_model(oc.link_name()) {
                    used_o.insert(i);
                    sub_orientations.push(oc);
                }
            }

            if sub_positions.is_empty() && sub_orientations.is_empty() {
                continue;
            }

            let subgroup = subgroup.clone();
            if let Some(cs) = select_default_sampler_inner(
                model,
                &subgroup,
                GroupConstraints {
                    joints: &[],
                    positions: &sub_positions,
                    orientations: &sub_orientations,
                },
                Some(subgroup_solver),
                nested_subgroup_solvers,
                max_attempts,
            )? {
                some_sampler_valid = true;
                samplers.push(cs);
            }
        }

        if some_sampler_valid {
            return Ok(Some(Box::new(UnionConstraintSampler::new(
                model,
                group.name(),
                samplers,
            )?)));
        }
    }

    // Step D/E: whatever Step A left behind (a partial-coverage joint
    // sampler), or nothing.
    Ok(samplers.pop())
}

/// The `joint_coverage` map: does `joints` cover every variable of `group`?
/// Upstream builds `joint_coverage` from `jmg->getVariableNames()` and marks
/// an entry true only when a constraint's `getJointVariableName()` is
/// literally a key of that map — this checks the same membership directly
/// rather than through an intermediate map.
fn joint_coverage_is_full(group: &JointModelGroup, joints: &[&JointConstraint]) -> bool {
    let covered: BTreeSet<&str> = joints
        .iter()
        .map(|j| j.joint_variable_name())
        .filter(|name| group.variable_names().iter().any(|v| v == name))
        .collect();
    group
        .variable_names()
        .iter()
        .all(|v| covered.contains(v.as_str()))
}

/// The position×orientation pairing loop, then the lone-position and
/// lone-orientation loops, each skipping the link already claimed by a full
/// pose match — `IKConstraintSampler::configure`/`getSamplingVolume` via
/// [`IkConstraintSamplerAdapter`]. `solver` names one fixed tip frame, so
/// every candidate [`IkConstraintSamplerAdapter::new`] can build here names
/// that same link (see this module's doc comment) — the return type is
/// `Option`, not a map, so a second, different-link entry has no state to
/// be written into.
fn collect_ik_candidate(
    model: &RobotModel,
    group: &JointModelGroup,
    positions: &[&PositionConstraint],
    orientations: &[&OrientationConstraint],
    solver: &Rc<RefCell<Box<dyn KinematicsSolver>>>,
    max_attempts: u32,
) -> Option<(String, IkConstraintSamplerAdapter)> {
    let mut used: Option<(String, IkConstraintSamplerAdapter)> = None;

    for pc in positions {
        for oc in orientations {
            if pc.link_name() != oc.link_name() {
                continue;
            }
            let sampling_pose = IkSamplingPose {
                position_constraint: Some((*pc).clone()),
                orientation_constraint: Some((*oc).clone()),
            };
            if let Ok(candidate) = IkConstraintSamplerAdapter::new(
                model,
                group,
                Rc::clone(solver),
                sampling_pose,
                max_attempts,
            ) {
                insert_or_replace_on_tie(&mut used, pc.link_name().to_string(), candidate);
            }
        }
    }

    // The link already claimed by a full pose match, if any: the
    // lone-position and lone-orientation loops below must not overwrite it.
    let claimed_by_full_pose: Option<String> = used.as_ref().map(|(link, _)| link.clone());

    for pc in positions {
        if claimed_by_full_pose.as_deref() == Some(pc.link_name()) {
            continue;
        }
        let sampling_pose = IkSamplingPose {
            position_constraint: Some((*pc).clone()),
            orientation_constraint: None,
        };
        if let Ok(candidate) = IkConstraintSamplerAdapter::new(
            model,
            group,
            Rc::clone(solver),
            sampling_pose,
            max_attempts,
        ) {
            insert_or_replace_on_tie(&mut used, pc.link_name().to_string(), candidate);
        }
    }

    for oc in orientations {
        if claimed_by_full_pose.as_deref() == Some(oc.link_name()) {
            continue;
        }
        let sampling_pose = IkSamplingPose {
            position_constraint: None,
            orientation_constraint: Some((*oc).clone()),
        };
        if let Ok(candidate) = IkConstraintSamplerAdapter::new(
            model,
            group,
            Rc::clone(solver),
            sampling_pose,
            max_attempts,
        ) {
            insert_or_replace_on_tie(&mut used, oc.link_name().to_string(), candidate);
        }
    }

    used
}

/// Per-link insertion: upstream's `if (used_l[link]->getSamplingVolume() <
/// iks->getSamplingVolume()) use = false;` keeps the existing entry only
/// when it is *strictly smaller* than the new candidate — on a tie, the new
/// (later-considered) candidate wins. `link` is always `used`'s existing key
/// when `used` is already `Some` (see this module's doc comment on why a
/// second, different link never reaches this function), so this only ever
/// compares two candidates for the same link, never decides between links.
fn insert_or_replace_on_tie(
    used: &mut Option<(String, IkConstraintSamplerAdapter)>,
    link: String,
    candidate: IkConstraintSamplerAdapter,
) {
    let keep_existing = used
        .as_ref()
        .is_some_and(|(_, existing)| existing.sampling_volume() < candidate.sampling_volume());
    if !keep_existing {
        *used = Some((link, candidate));
    }
}
