// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Behaviorally derived from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_state/include/moveit/robot_state/attached_body.hpp

//! [`AttachedBody`]: geometry rigidly attached to a robot link.
//!
//! # Deviation from upstream
//!
//! Upstream stores attached bodies inside `moveit::core::RobotState` itself
//! (`RobotState::attachBody`/`getAttachedBody`/`hasAttachedBody`/...).
//! `moveit_state::RobotState` does not carry that concept yet — its own
//! crate doc lists "no attached bodies" under deferred scope. Rather than
//! let [`crate::PlanningScene`] shadow a second, parallel notion of
//! "attached" next to a `RobotState` that has none, this crate is the sole
//! owner of attached-body data for now:
//! [`crate::PlanningScene::attached_bodies`] is the one place this state
//! lives, not a cache duplicating something `RobotState` also tracks. When
//! `RobotState` gains attached-body support, this module's contents belong
//! there instead, and `PlanningScene` goes back to delegating to it — the
//! same relationship it already has with upstream's real design.
//!
//! Also unlike upstream, [`AttachedBody::shape_poses`] are stored directly
//! relative to the attach link's own frame, rather than relative to an
//! intermediate "pose in link" that itself holds the object's frame within
//! the link (upstream `AttachedBody::pose_`/`shape_poses_`: two levels).
//! Nothing here needs that second level: composing it away up front means
//! [`crate::PlanningScene::detach`] only ever needs one transform (the
//! link's current global pose) to recompute every shape's current global
//! pose, not two chained ones. [`AttachedBody::subframe_pose`] follows the
//! same one-level rule, for the same reason.
//!
//! One consequence of collapsing upstream's `pose_` away: this port has no
//! stored value standing in for "the body's own frame" the way upstream's
//! `pose_`/`global_pose_` do. [`crate::PlanningScene::frame_transform`]
//! resolving a bare attached-body id (upstream's `AttachedBody::getGlobalPose`
//! tier) therefore treats that missing `pose_` as `Isometry3::identity()` —
//! see that method's doc for why this is forced by the one-level design
//! already committed to here, not a free choice, and how the oracle's own
//! round-2 reconciliation (`attachBody`'s `pose` argument always `Identity`)
//! already made the same call.
//!
//! `detach_posture` (a `trajectory_msgs` type, D1) is not carried.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use moveit_collision::AttachedBodyGeometry;
use moveit_geometry::{Isometry3, Shape};

/// Geometry rigidly attached to a robot link. See the module doc for how
/// this differs from upstream `moveit::core::AttachedBody`.
#[derive(Debug, Clone)]
pub struct AttachedBody {
    id: String,
    link_name: String,
    shapes: Vec<Arc<Shape>>,
    shape_poses: Vec<Isometry3>,
    touch_links: BTreeSet<String>,
    subframes: BTreeMap<String, Isometry3>,
}

impl AttachedBody {
    pub(crate) fn new(
        id: String,
        link_name: String,
        shapes: Vec<Arc<Shape>>,
        shape_poses: Vec<Isometry3>,
        touch_links: BTreeSet<String>,
        subframes: BTreeMap<String, Isometry3>,
    ) -> Self {
        Self {
            id,
            link_name,
            shapes,
            shape_poses,
            touch_links,
            subframes,
        }
    }

    /// This body's id. Upstream `getName`.
    pub fn id(&self) -> &str {
        &self.id
    }

    /// The link this body is attached to. Upstream `getAttachedLinkName`.
    pub fn link_name(&self) -> &str {
        &self.link_name
    }

    /// This body's shapes. Upstream `getShapes`.
    pub fn shapes(&self) -> &[Arc<Shape>] {
        &self.shapes
    }

    /// Each shape's pose relative to [`AttachedBody::link_name`]'s own
    /// frame — see the module doc for why this is one level, not upstream's
    /// two. Upstream `getShapePoses()` composed with `getPose()`.
    pub fn shape_poses(&self) -> &[Isometry3] {
        &self.shape_poses
    }

    /// Links this body is allowed to touch without that counting as a
    /// collision. Upstream `getTouchLinks`.
    pub fn touch_links(&self) -> &BTreeSet<String> {
        &self.touch_links
    }

    /// This body's subframe pose relative to [`AttachedBody::link_name`]'s
    /// own frame, if `name` names one — see the module doc for why this is
    /// one level, not upstream's two. Upstream `getSubframeTransform`,
    /// restricted to the lookup key already having the body id stripped:
    /// upstream's own key is `"<id>/<name>"` (`attached_body.cpp:139-155`);
    /// callers here match that spelling explicitly (see
    /// [`crate::PlanningScene::frame_transform`]) rather than this method
    /// re-parsing it, the same split [`moveit_collision::World`] already
    /// uses between its subframe-suffix parsing and `Object::subframe_pose`.
    pub fn subframe_pose(&self, name: &str) -> Option<Isometry3> {
        self.subframes.get(name).copied()
    }

    /// Every subframe name on this body (bare, without the `"<id>/"` prefix
    /// — see [`AttachedBody::subframe_pose`]).
    pub fn subframe_names(&self) -> impl Iterator<Item = &str> {
        self.subframes.keys().map(String::as_str)
    }

    /// Borrows this body's fields as a [`moveit_collision::AttachedBodyGeometry`]
    /// — the view a [`moveit_collision::CollisionEnv`] backend needs, without
    /// `moveit-collision` depending back on this crate. See
    /// [`crate::PlanningScene`]'s "Collision checking" doc for how this is used.
    pub fn as_geometry(&self) -> AttachedBodyGeometry<'_> {
        AttachedBodyGeometry {
            id: &self.id,
            link_name: &self.link_name,
            shapes: &self.shapes,
            shape_poses: &self.shape_poses,
            touch_links: &self.touch_links,
        }
    }
}
