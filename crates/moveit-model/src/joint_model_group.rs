// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2008-2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/joint_model_group.hpp
//   moveit_core/robot_model/src/joint_model_group.cpp

use std::collections::{BTreeMap, HashMap};

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
/// (which links move when this group's state changes) and the kinematics
/// solver plumbing (`group_kinematics_`). None of those are read by this
/// phase's done-criteria (link/joint counts, group composition, joint
/// limits, mimic relationships); they belong to `moveit-state` (Phase 2) and
/// `moveit-kinematics` (Phase 4) respectively. `PORTING-PLAN.md` Phase 1
/// scopes this crate to "JointModelGroup, 서브그룹, KinematicChain 해석"
/// (subgroups, kinematic-chain resolution) — both of which this type does
/// carry: [`JointModelGroup::subgroup_names`] and the `<chain>` element
/// expansion in `RobotModel`'s group construction. (The end-effector fields
/// (`end_effector_name_`, `end_effector_parent_`,
/// `attached_end_effector_names_`) and SRDF `<group_state>` support
/// (`default_states_`) *are* carried — see
/// [`JointModelGroup::is_end_effector`] and
/// [`JointModelGroup::default_state_names`] respectively.)
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
    pub(crate) end_effector_name: Option<String>,
    pub(crate) end_effector_parent: Option<EndEffectorParent>,
    pub(crate) attached_end_effector_names: Vec<String>,
    pub(crate) default_state_names: Vec<String>,
    pub(crate) default_states: HashMap<String, BTreeMap<String, f64>>,
}

/// The group and link a [`JointModelGroup`] end effector is attached to.
/// Upstream `end_effector_parent_`, a `std::pair<std::string, std::string>`
/// defaulted to `("", "")` and mutated in place by `setEndEffectorParent`.
///
/// # Deviation from upstream
///
/// This only exists at all once [`JointModelGroup::set_end_effector_parent`]
/// has actually run — which upstream does exactly once per group that *is* an
/// end effector (see `RobotModel::buildGroupsInfoEndEffectors`) — so
/// [`JointModelGroup::end_effector_parent`] returns [`None`] for a
/// non-end-effector group rather than upstream's meaningless default pair.
/// [`group`](EndEffectorParent::group) is itself [`None`] when no parent
/// group could be identified, replacing upstream's `""` sentinel (the same
/// substitution `moveit_srdf::EndEffector::parent_group` already makes).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EndEffectorParent {
    /// The group the end effector is attached to, if one could be
    /// identified.
    pub group: Option<String>,
    /// The link the end effector is attached to. Always present — this is
    /// the SRDF `<end_effector parent_link="...">` attribute, not something
    /// resolution can fail to produce.
    pub link: String,
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

    /// `isEndEffector`: `!end_effector_name_.empty()`.
    pub fn is_end_effector(&self) -> bool {
        self.end_effector_name.is_some()
    }

    /// `getEndEffectorName`. Empty string if this group is not an end
    /// effector, matching upstream's `end_effector_name_`'s own default.
    pub fn end_effector_name(&self) -> &str {
        self.end_effector_name.as_deref().unwrap_or("")
    }

    /// `setEndEffectorName`.
    pub(crate) fn set_end_effector_name(&mut self, name: impl Into<String>) {
        self.end_effector_name = Some(name.into());
    }

    /// `getEndEffectorParentGroup`: the group and link this end effector is
    /// attached to, if this group is an end effector and a parent was ever
    /// set. See [`EndEffectorParent`]'s doc comment for how this differs
    /// from upstream's always-present default pair.
    pub fn end_effector_parent(&self) -> Option<&EndEffectorParent> {
        self.end_effector_parent.as_ref()
    }

    /// `setEndEffectorParent`.
    pub(crate) fn set_end_effector_parent(
        &mut self,
        group: Option<String>,
        link: impl Into<String>,
    ) {
        self.end_effector_parent = Some(EndEffectorParent {
            group,
            link: link.into(),
        });
    }

    /// `getAttachedEndEffectorNames`: the names of end-effector groups
    /// attached to (parented at a link within) this group.
    pub fn attached_end_effector_names(&self) -> &[String] {
        &self.attached_end_effector_names
    }

    /// `attachEndEffector`: record `eef_name` as attached to this group.
    /// Upstream does an unconditional `push_back` with no dedup; this port
    /// matches that.
    pub(crate) fn attach_end_effector(&mut self, eef_name: impl Into<String>) {
        self.attached_end_effector_names.push(eef_name.into());
    }

    /// `getDefaultStateNames`: the names of the SRDF `<group_state>`s known
    /// for this group, in document order.
    ///
    /// # Deviation from upstream
    ///
    /// If the same name is ever added twice (via
    /// [`JointModelGroup::add_default_state`]), upstream's
    /// `default_states_names_.push_back(name)` keeps both entries even
    /// though `default_states_[name]` silently keeps only the later value —
    /// this port reproduces that duplication exactly, rather than
    /// deduplicating, since the two names can't be told apart from the
    /// values alone and a caller iterating this list to look up
    /// [`JointModelGroup::variable_default_positions`] should see the same
    /// count upstream does.
    pub fn default_state_names(&self) -> &[String] {
        &self.default_state_names
    }

    /// `getVariableDefaultPositions(name, values)`: the named SRDF
    /// `<group_state>`'s variable-name-to-value map, or [`None`] if no state
    /// named `name` was ever added for this group (upstream's `bool` return,
    /// inverted to an [`Option`]).
    ///
    /// The map may not cover every variable in this group: upstream stores
    /// exactly the variables `RobotModel::buildGroupStates` could resolve
    /// (a `<joint>` value whose count didn't match the joint's variable
    /// count, or whose name isn't part of this group, is dropped rather
    /// than defaulted to a made-up value) and still keeps the state if
    /// anything at all resolved. A caller must check `contains_key` per
    /// variable rather than assuming full coverage.
    pub fn variable_default_positions(&self, name: &str) -> Option<&BTreeMap<String, f64>> {
        self.default_states.get(name)
    }

    /// `addDefaultState`.
    pub(crate) fn add_default_state(&mut self, name: String, state: BTreeMap<String, f64>) {
        self.default_state_names.push(name.clone());
        self.default_states.insert(name, state);
    }
}
