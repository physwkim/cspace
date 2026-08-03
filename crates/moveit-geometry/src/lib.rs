// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/transforms/include/moveit/transforms/transforms.hpp
//   moveit_core/transforms/src/transforms.cpp
// and from geometric_shapes 2.3.3 (see shapes.rs's provenance comment).

//! Frame transforms and geometric primitives for moveit-rs.
//!
//! This crate carries [`Transforms`] (`moveit_core/transforms`) and the
//! `geometric_shapes` shape and body layers (see the [`shapes`] and
//! [`bodies`] module docs for scope and provenance).

pub mod bodies;
mod shapes;
mod transforms;

pub use shapes::{
    BoundingSphere, Cone, Cuboid, Cylinder, Mesh, OcTree, Plane, Shape, ShapeType, Sphere,
};
pub use transforms::Transforms;

/// Rigid-body transform. Replaces upstream `Eigen::Isometry3d`.
pub type Isometry3 = nalgebra::Isometry3<f64>;
/// 3-vector. Replaces upstream `Eigen::Vector3d`.
pub type Vector3 = nalgebra::Vector3<f64>;
/// Unit quaternion. Replaces upstream `Eigen::Quaterniond`.
pub type UnitQuaternion = nalgebra::UnitQuaternion<f64>;
/// Rotation matrix. Replaces upstream `Eigen::Matrix3d` where it holds a rotation.
pub type Rotation3 = nalgebra::Rotation3<f64>;
