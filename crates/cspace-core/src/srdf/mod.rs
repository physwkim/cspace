// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from srdfdom 2.0.8 — the SRDF parser that moveit2 @
// e017c91ee12984393a28ba246075c65f69cde3bf depends on. PORTING-PLAN.md §2
// records that no SRDF crate exists on crates.io, so this is written from
// scratch against:
//   srdfdom/include/srdfdom/model.h
//   srdfdom/src/model.cpp

//! SRDF parsing for moveit-rs.
//!
//! SRDF is the semantic half of a robot description: the URDF says what links
//! and joints exist, the SRDF says which of them form a planning group, what
//! poses that group has names for, which pairs never collide, and what joints
//! exist that the URDF does not know about. [`SrdfModel`] is one parsed
//! document.
//!
//! ```
//! use cspace_core::srdf::{SrdfModel, VirtualJointType};
//!
//! let model = SrdfModel::parse_str(
//!     r#"<robot name="arm">
//!          <virtual_joint name="base" type="fixed"
//!                         parent_frame="world" child_link="link0"/>
//!          <group name="arm"><chain base_link="link0" tip_link="link6"/></group>
//!          <group_state name="home" group="arm">
//!            <joint name="joint1" value="0.0"/>
//!          </group_state>
//!        </robot>"#,
//! )?;
//!
//! assert_eq!(model.name(), Some("arm"));
//! assert_eq!(model.groups()[0].chains[0].tip_link, "link6");
//! assert_eq!(model.virtual_joints()[0].joint_type, VirtualJointType::Fixed);
//! assert!(model.diagnostics().is_empty());
//! # Ok::<(), cspace_core::error::Error>(())
//! ```
//!
//! # This crate does not see the URDF
//!
//! Upstream srdfdom parses an SRDF *against* a `urdf::ModelInterface` and drops
//! every element that names a link or joint the URDF lacks. PORTING-PLAN.md §3
//! puts `cspace_core::srdf` below `cspace_core::model`, so no URDF is available here and
//! those checks belong to `cspace_core::model`, which holds both descriptions. The
//! full list of what that defers is on [`SrdfModel`]; the short version is that
//! an [`SrdfModel`] faithfully describes a document and does not assert that
//! any name in it exists.
//!
//! # Nothing is dropped silently
//!
//! Upstream logs what it discards to `console_bridge` and returns a model that
//! carries no record of it. Every such decision here becomes a [`Diagnostic`]
//! on the model, so a caller can tell a group that was never written from one
//! that was thrown away over a typo. The decisions themselves are upstream's.

mod diagnostic;
mod model;
mod parse;

pub use diagnostic::Diagnostic;
pub use model::{
    Chain, CollisionPair, EndEffector, Group, GroupState, JointProperty, LinkSpheres, Sphere,
    SrdfModel, VirtualJoint, VirtualJointType,
};
