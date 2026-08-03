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
//! lookup (D4; `moveit-model` carries no such map and is not touched this
//! round). [`select_default_sampler`]'s `solver` and `subgroup_solvers`
//! parameters are what the caller passes in its place — the same
//! caller-supplies-the-solver decision [`crate::IkConstraintSampler::new`]
//! already made, now threaded one level up.
//!
//! # One level of subgroup nesting
//!
//! Upstream's subgroup recursion (`selectDefaultSampler` calling itself on
//! `it->first->getName()`) re-derives ITS OWN subgroup solvers by reading
//! the same global `getGroupKinematics()` map again — recursion can go as
//! deep as the model's group hierarchy does. This port's recursive call
//! passes `subgroup_solvers: vec![]`, so a subgroup-of-a-subgroup that
//! itself needs an IK solver is not resolved. None of this crate's three
//! model fixtures (panda, fanuc, dual-arm panda) define a group whose
//! *subgroup* is itself composite with its own further subgroups needing
//! solvers — `panda_arm_hand`'s only subgroups (`panda_arm`, `hand`) are
//! themselves leaf chains — so this is an untested, undemonstrated gap, not
//! a silently-wrong path exercised by anything this phase's done-criteria
//! checks.
//!
//! # `used`'s cross-link tie-break is unreachable through this port's own API
//!
//! Upstream's `IKConstraintSampler::configure` accepts a constrained link
//! that is either the solver's own tip frame or *fixed-jointed* to it
//! (`default_constraint_samplers.cpp`, the `fixed_links` walk around each
//! `getLinkModel()` check) — so `used_l` can legitimately collect more than
//! one link key from a single solver, and `selectDefaultSampler`'s final
//! `if (v < msv)` loop is how it picks one. [`crate::IkConstraintSampler`]
//! ports none of that fixed-link bridging (see its own doc comment) and
//! requires the constrained link to equal `solver.tip_frame()` exactly.
//! Since [`collect_ik_candidates`] shares one solver (one fixed tip frame)
//! across every candidate it builds, every successful candidate is keyed by
//! that same tip frame — `used` can hold at most one entry in this port,
//! never more. [`smallest_across_links`] is still written to match
//! upstream's own multi-key reduction faithfully (a direct transcription,
//! not dead weight to delete) so it needs no revisiting if fixed-link
//! bridging is ever added; today its `>1`-key branch is simply never taken,
//! which is why no test in this crate exercises it — the per-link tie-break
//! in [`insert_or_replace_on_tie`] (reachable: several pairings can name the
//! *same* link) is what the tests cover instead.

use std::cell::RefCell;
use std::collections::BTreeSet;
use std::rc::Rc;

use moveit_error::Result;
use moveit_kinematics::KinematicsSolver;
use moveit_model::{JointModelGroup, RobotModel};

use crate::{
    Constraint, ConstraintSampler, IkConstraintSamplerAdapter, IkSamplingPose, JointConstraint,
    JointConstraintSampler, OrientationConstraint, PositionConstraint, UnionConstraintSampler,
};

/// `ConstraintSamplerManager::selectDefaultSampler`.
///
/// `solver` stands in for upstream's `jmg->getGroupKinematics().first` — the
/// group's own IK solver, if any. `subgroup_solvers` stands in for
/// `jmg->getGroupKinematics().second` — one solver per immediate subgroup
/// that has one (see this module's doc comment on why only one level deep).
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
    subgroup_solvers: Vec<(String, Box<dyn KinematicsSolver>)>,
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
    subgroup_solvers: Vec<(String, Box<dyn KinematicsSolver>)>,
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
        let used =
            collect_ik_candidates(model, group, positions, orientations, &shared, max_attempts);
        if let Some(winner) = smallest_across_links(used) {
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

        for (subgroup_name, subgroup_solver) in subgroup_solvers {
            let Ok(subgroup) = model.joint_model_group(&subgroup_name) else {
                return Err(moveit_error::Error::unknown_name("group", &subgroup_name));
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
                vec![],
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
/// lone-orientation loops, each skipping links already claimed by a full
/// pose match — `IKConstraintSampler::configure`/`getSamplingVolume` via
/// [`IkConstraintSamplerAdapter`]. Returns one candidate per link that
/// matched at least one constraint, keyed by link name (a `BTreeMap` to
/// match upstream's own `std::map<std::string, IKConstraintSamplerPtr>`
/// iteration order, which the cross-link tie-break in
/// [`smallest_across_links`] depends on).
fn collect_ik_candidates(
    model: &RobotModel,
    group: &JointModelGroup,
    positions: &[&PositionConstraint],
    orientations: &[&OrientationConstraint],
    solver: &Rc<RefCell<Box<dyn KinematicsSolver>>>,
    max_attempts: u32,
) -> std::collections::BTreeMap<String, IkConstraintSamplerAdapter> {
    let mut used: std::collections::BTreeMap<String, IkConstraintSamplerAdapter> =
        std::collections::BTreeMap::new();

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

    // Links already claimed by a full pose match: the lone-position and
    // lone-orientation loops below must not overwrite them.
    let claimed_by_full_pose: BTreeSet<String> = used.keys().cloned().collect();

    for pc in positions {
        if claimed_by_full_pose.contains(pc.link_name()) {
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
        if claimed_by_full_pose.contains(oc.link_name()) {
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
/// (later-considered) candidate wins.
fn insert_or_replace_on_tie(
    used: &mut std::collections::BTreeMap<String, IkConstraintSamplerAdapter>,
    link: String,
    candidate: IkConstraintSamplerAdapter,
) {
    let keep_existing = used
        .get(&link)
        .is_some_and(|existing| existing.sampling_volume() < candidate.sampling_volume());
    if !keep_existing {
        used.insert(link, candidate);
    }
}

/// The cross-link reduction: if more than one link ended up with a
/// candidate, only the single smallest-volume one survives — a group has
/// one IK solver, so at most one link's pose can actually be IK-sampled at
/// once. Upstream's `if (v < msv) { iks = it->second; msv = v; }` is a
/// strict comparison, so on a tie the *first* (alphabetically-earliest
/// link name, since `used_l` is a `std::map`) candidate wins — the reverse
/// tie direction from [`insert_or_replace_on_tie`] above.
fn smallest_across_links(
    used: std::collections::BTreeMap<String, IkConstraintSamplerAdapter>,
) -> Option<IkConstraintSamplerAdapter> {
    used.into_values().fold(None, |best, candidate| match best {
        Some(b) if b.sampling_volume() <= candidate.sampling_volume() => Some(b),
        _ => Some(candidate),
    })
}
