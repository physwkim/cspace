// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/link_model.hpp
//   moveit_core/robot_model/src/link_model.cpp

use moveit_geometry::Isometry3;

/// A link from the robot: its place in the kinematic tree and the constant
/// offset applied before any joint transform.
///
/// Upstream `moveit::core::LinkModel`. Built and owned by
/// [`crate::robot_model::RobotModel`]; there is no public constructor.
///
/// # Deviations from upstream
///
/// 1. **Cross-references are indices, not pointers.** Upstream stores
///    `parent_joint_model_`/`parent_link_model_`/`child_joint_models_` as raw
///    `const JointModel*`/`const LinkModel*`. This port has no raw pointers
///    into a sibling `Vec` — every reference here is an index into
///    [`RobotModel::link_models`](crate::robot_model::RobotModel::link_models)
///    or
///    [`RobotModel::joint_models`](crate::robot_model::RobotModel::joint_models),
///    resolved through the owning `RobotModel`'s accessors.
/// 2. **Collision/visual geometry is not here.** Upstream's `LinkModel` also
///    carries `shapes_`, `visual_mesh_*`, `collision_origin_transform_`,
///    `shape_extents_`, `centered_bounding_box_offset_` and
///    `associated_fixed_transforms_`. `PORTING-PLAN.md` §3 puts collision
///    geometry in `moveit-collision` (the parry backend), a later phase; the
///    Phase 1 done-criteria (link/joint counts, group composition, joint
///    limits, mimic relationships) do not read any of it, so it is not
///    built here.
#[derive(Debug, Clone, PartialEq)]
pub struct LinkModel {
    name: String,
    link_index: usize,
    parent_joint_index: usize,
    parent_link_index: Option<usize>,
    child_joint_indices: Vec<usize>,
    joint_origin_transform: Isometry3,
}

impl LinkModel {
    pub(crate) fn new(
        name: impl Into<String>,
        link_index: usize,
        parent_joint_index: usize,
        parent_link_index: Option<usize>,
        joint_origin_transform: Isometry3,
    ) -> Self {
        Self {
            name: name.into(),
            link_index,
            parent_joint_index,
            parent_link_index,
            child_joint_indices: Vec::new(),
            joint_origin_transform,
        }
    }

    pub(crate) fn add_child_joint_index(&mut self, joint_index: usize) {
        self.child_joint_indices.push(joint_index);
    }

    /// `getName`
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `getLinkIndex`: this link's position in
    /// [`RobotModel::link_models`](crate::robot_model::RobotModel::link_models),
    /// which is also the order links are visited when traversing the
    /// kinematic tree depth-first.
    pub fn link_index(&self) -> usize {
        self.link_index
    }

    /// `getParentJointModel`, as an index. There is always a parent joint —
    /// even the root link's parent is a joint (the SRDF virtual joint, or an
    /// assumed fixed joint if the SRDF names none).
    pub fn parent_joint_index(&self) -> usize {
        self.parent_joint_index
    }

    /// `getParentLinkModel`, as an index. [`None`] for the root link.
    pub fn parent_link_index(&self) -> Option<usize> {
        self.parent_link_index
    }

    /// `getChildJointModels`, as indices.
    pub fn child_joint_indices(&self) -> &[usize] {
        &self.child_joint_indices
    }

    /// `getJointOriginTransform`: the constant offset pre-applied before the
    /// parent joint's own transform.
    pub fn joint_origin_transform(&self) -> &Isometry3 {
        &self.joint_origin_transform
    }
}
