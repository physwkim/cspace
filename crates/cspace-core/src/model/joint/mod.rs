// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/joint_model.hpp
//   moveit_core/robot_model/include/moveit/robot_model/revolute_joint_model.hpp
//   moveit_core/robot_model/include/moveit/robot_model/prismatic_joint_model.hpp
//   moveit_core/robot_model/include/moveit/robot_model/planar_joint_model.hpp
//   moveit_core/robot_model/include/moveit/robot_model/floating_joint_model.hpp
//   moveit_core/robot_model/include/moveit/robot_model/fixed_joint_model.hpp

//! The joint model hierarchy: [`JointModel`] plus its five concrete kinds.
//!
//! This module intentionally stops at the joint layer. `LinkModel`,
//! `RobotModel` and `JointModelGroup` are out of scope for this crate (see
//! `PORTING-PLAN.md`); nothing here references them.

mod bounds;
mod fixed;
mod floating;
mod model;
mod planar;
mod prismatic;
mod revolute;
mod urdf;

pub use bounds::{JointLimits, VariableBounds};
pub use floating::FloatingJoint;
pub use model::{JointKind, JointModel, JointType, Mimic};
pub use planar::{PlanarJoint, PlanarMotionModel};
pub use prismatic::PrismaticJoint;
pub use revolute::RevoluteJoint;
pub use urdf::joint_model_from_urdf;
