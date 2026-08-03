// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2008-2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/joint_model_group.hpp
//   moveit_core/robot_model/src/joint_model_group.cpp

/// One SRDF `<group>`, resolved against a URDF: the joints and links it
/// contains, once chains, direct links and subgroups have all been expanded
/// to the joint set they imply.
///
/// Upstream `moveit::core::JointModelGroup`. Built and owned by
/// [`crate::robot_model::RobotModel`]; there is no public constructor. Every
/// index here is an index into
/// [`RobotModel::joint_models`](crate::robot_model::RobotModel::joint_models)/
/// [`RobotModel::link_models`](crate::robot_model::RobotModel::link_models).
///
/// # Deviation from upstream: narrower than `JointModelGroup`
///
/// Upstream's type also carries `joint_roots_`/`common_root_` (kinematic
/// subtree roots, for `RobotState` FK optimisation), `updated_link_model_*`
/// (which links move when this group's state changes), `is_chain_`/
/// `is_single_dof_` (convenience flags derived from the joint set),
/// `default_states_` (SRDF `<group_state>`), the end-effector fields
/// (`end_effector_name_`, `end_effector_parent_`,
/// `attached_end_effector_names_`) and the kinematics solver plumbing
/// (`group_kinematics_`). None of those are read by this phase's
/// done-criteria (link/joint counts, group composition, joint limits, mimic
/// relationships); they belong to `moveit-state` (Phase 2), the SRDF
/// end-effector/group-state elements (deferred alongside them), and
/// `moveit-kinematics` (Phase 4) respectively. `PORTING-PLAN.md` Phase 1
/// scopes this crate to "JointModelGroup, 서브그룹, KinematicChain 해석"
/// (subgroups, kinematic-chain resolution) — both of which this type does
/// carry: [`JointModelGroup::subgroup_names`] and the `<chain>`
/// element expansion in `RobotModel`'s group construction.
#[derive(Debug, Clone, PartialEq)]
pub struct JointModelGroup {
    pub(crate) name: String,
    pub(crate) joint_indices: Vec<usize>,
    pub(crate) joint_names: Vec<String>,
    pub(crate) active_joint_indices: Vec<usize>,
    pub(crate) active_joint_names: Vec<String>,
    pub(crate) fixed_joint_indices: Vec<usize>,
    pub(crate) mimic_joint_indices: Vec<usize>,
    pub(crate) variable_names: Vec<String>,
    pub(crate) link_indices: Vec<usize>,
    pub(crate) link_names: Vec<String>,
    pub(crate) subgroup_names: Vec<String>,
}

impl JointModelGroup {
    /// `getName`
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `hasJointModel`
    pub fn has_joint_model(&self, joint: &str) -> bool {
        self.joint_names.iter().any(|n| n == joint)
    }

    /// `hasLinkModel`
    pub fn has_link_model(&self, link: &str) -> bool {
        self.link_names.iter().any(|n| n == link)
    }

    /// `getJointModels`, as indices: every joint in this group (including
    /// fixed and mimic joints), in depth-first order.
    pub fn joint_indices(&self) -> &[usize] {
        &self.joint_indices
    }

    /// `getJointModelNames`
    pub fn joint_names(&self) -> &[String] {
        &self.joint_names
    }

    /// `getActiveJointModels`, as indices: joints with controllable DOF
    /// (excludes fixed and mimic joints).
    pub fn active_joint_indices(&self) -> &[usize] {
        &self.active_joint_indices
    }

    /// `getActiveJointModelNames`
    pub fn active_joint_names(&self) -> &[String] {
        &self.active_joint_names
    }

    /// `getFixedJointModels`, as indices.
    pub fn fixed_joint_indices(&self) -> &[usize] {
        &self.fixed_joint_indices
    }

    /// `getMimicJointModels`, as indices.
    pub fn mimic_joint_indices(&self) -> &[usize] {
        &self.mimic_joint_indices
    }

    /// `getVariableNames`: every variable name of every non-fixed joint in
    /// this group (active and mimic).
    pub fn variable_names(&self) -> &[String] {
        &self.variable_names
    }

    /// `getLinkModels`, as indices: every link that is the child of a joint
    /// in this group.
    pub fn link_indices(&self) -> &[usize] {
        &self.link_indices
    }

    /// `getLinkModelNames`
    pub fn link_names(&self) -> &[String] {
        &self.link_names
    }

    /// `getSubgroupNames`: the names of other groups whose joint set is a
    /// subset of this one's.
    pub fn subgroup_names(&self) -> &[String] {
        &self.subgroup_names
    }

    /// `isSubgroup`
    pub fn is_subgroup(&self, group: &str) -> bool {
        self.subgroup_names.iter().any(|n| n == group)
    }
}
