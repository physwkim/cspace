// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp
//     (newPlanningSceneCallback:711, newPlanningSceneMessage:739,
//      processAttachedCollisionObjectMsg:841)

//! The one scene every inbound topic mutates, and the one function that
//! mutates it.
//!
//! Upstream's `PlanningSceneMonitor` holds `scene_` behind a
//! `std::shared_mutex scene_update_mutex_`, and every callback that changes
//! it takes a `std::unique_lock` over exactly the mutation
//! (`planning_scene_monitor.cpp:748` for a scene message, `:852` for an
//! attached object) while readers take `LockedPlanningSceneRO`. Two
//! properties come out of that: a reader never observes a half-applied
//! scene, and a reader never mutates the monitored one.
//!
//! This node is single-threaded (`futures::executor::LocalPool` plus
//! `Node::spin_once`), so it gets both from ownership instead of from a
//! mutex. [`MonitoredScene`] is an `Arc` snapshot behind an `Rc<RefCell<..>>`
//! cell: [`apply`] clones the current scene, applies the change to the
//! clone, and installs it only on success, and [`snapshot`] hands out an
//! `Arc` that the caller can read for as long as it likes without blocking
//! the next update.
//!
//! # Why this is a module and not two copies of four lines
//!
//! Two subscriptions now change this scene -- `planning_scene` and
//! `attached_collision_object` -- and a third (`planning_scene_world`) is
//! still absent. Clone-apply-swap written out at each call site is one
//! mutator per site, and the failure mode is not hypothetical: a site that
//! installs the clone *before* checking the result publishes a half-applied
//! scene, and nothing but review catches the difference. [`apply`] is the
//! only function in this crate that writes the cell, so "a failed update
//! leaves the previous scene installed" holds for every caller by
//! construction rather than per site.
//!
//! # What this does not port
//!
//! `triggerSceneUpdateEvent(UPDATE_GEOMETRY)` (`:855`) has no counterpart:
//! upstream fans an update type out to registered callbacks so that
//! republishers and octomap monitors can react, and this node has no
//! subscriber list to fan out to. `updateFrameTransforms` (`:850`) is
//! likewise absent -- it pulls fresh transforms from a TF buffer, and this
//! node has no TF listener. Both are named here rather than left as a silent
//! difference between this and `processAttachedCollisionObjectMsg`.

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;

use moveit_error::Result;
use moveit_model::RobotModel;
use moveit_scene::PlanningScene;
use moveit_srdf::SrdfModel;
use r2r::moveit_msgs::msg as moveit_msgs;

use crate::scene::attached::apply_attached_collision_object;
use crate::scene::planning_scene::use_planning_scene_msg;

/// The monitored scene: upstream's `PlanningSceneMonitor::scene_` plus the
/// lock discipline around it, expressed as ownership.
///
/// `'static` because `futures::task::LocalSpawnExt::spawn_local` requires it
/// even for a same-thread task; the node leaks its `RobotModel` and
/// `SrdfModel` for the same reason.
pub type MonitoredScene = Rc<RefCell<Arc<PlanningScene<'static>>>>;

/// A fresh monitored scene for `model`/`srdf`.
pub fn new(model: &'static RobotModel, srdf: &'static SrdfModel) -> MonitoredScene {
    Rc::new(RefCell::new(Arc::new(PlanningScene::new(model, srdf))))
}

/// The current scene, as an `Arc` the caller may hold and read while later
/// updates install newer ones. Upstream's `LockedPlanningSceneRO`.
///
/// A reader that wants to *evaluate* against this scene (set a state, run a
/// collision query) should call [`PlanningScene::diff`] on it: the snapshot
/// itself is shared and must not be mutated.
pub fn snapshot(cell: &MonitoredScene) -> Arc<PlanningScene<'static>> {
    Arc::clone(&cell.borrow())
}

/// **The only function in this crate that writes a [`MonitoredScene`].**
///
/// Clone, apply, and install only on success -- so a rejected update leaves
/// the previously installed scene exactly as it was, and no reader can
/// observe a scene that a failed conversion stopped halfway through. Upstream
/// has no equivalent guarantee: `processAttachedCollisionObjectMsg` mutates
/// `scene_` in place under the lock and returns `false` after the scene has
/// already changed (`planning_scene_monitor.cpp:853`).
///
/// The borrow is released before returning, so a caller may `snapshot` again
/// immediately; it must not be held across an `.await`.
pub fn apply(
    cell: &MonitoredScene,
    change: impl FnOnce(&mut PlanningScene<'static>) -> Result<()>,
) -> Result<()> {
    let mut installed = cell.borrow_mut();
    let mut next = PlanningScene::cloned(&installed);
    change(&mut next)?;
    *installed = Arc::new(next);
    Ok(())
}

/// The `planning_scene` topic's callback body. Upstream
/// `newPlanningSceneCallback` (`planning_scene_monitor.cpp:711`), which hands
/// the message straight to `newPlanningSceneMessage` (`:739`).
pub fn apply_planning_scene_msg(
    cell: &MonitoredScene,
    msg: moveit_msgs::PlanningScene,
) -> Result<()> {
    apply(cell, |scene| use_planning_scene_msg(scene, msg))
}

/// The `attached_collision_object` topic's callback body. Upstream
/// `PlanningSceneMonitor::processAttachedCollisionObjectMsg`
/// (`planning_scene_monitor.cpp:841`), which delegates the whole decision to
/// `PlanningScene::processAttachedCollisionObjectMsg` (`:853`) -- ported as
/// [`apply_attached_collision_object`], the same function a `PlanningScene`
/// diff's `robot_state.attached_collision_objects` reaches. One owner, two
/// topics: whichever arrives second sees the first's result, and neither has
/// a private path into the scene.
pub fn apply_attached_collision_object_msg(
    cell: &MonitoredScene,
    msg: moveit_msgs::AttachedCollisionObject,
) -> Result<()> {
    apply(cell, |scene| apply_attached_collision_object(scene, msg))
}

#[cfg(test)]
mod tests {
    use super::*;
    use moveit_model::MeshSearchPaths;
    use r2r::geometry_msgs::msg as geometry_msgs;

    const URDF: &str = r#"<?xml version="1.0"?>
<robot name="one_joint">
  <link name="base_link"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>"#;
    const SRDF: &str = r#"<?xml version="1.0"?><robot name="one_joint"></robot>"#;

    /// Leaked because [`MonitoredScene`] is `'static` -- the node leaks the
    /// same two for the same reason, so the tests exercise the real type
    /// rather than a lifetime-relaxed stand-in.
    fn cell() -> MonitoredScene {
        let urdf = urdf_rs::read_from_string(URDF).expect("inline URDF must parse");
        let srdf = SrdfModel::parse_str(SRDF).expect("inline SRDF must parse");
        let model = RobotModel::from_urdf_and_srdf(&urdf, URDF, &srdf, &MeshSearchPaths::none())
            .expect("valid one-joint urdf");
        new(Box::leak(Box::new(model)), Box::leak(Box::new(srdf)))
    }

    fn identity_pose() -> geometry_msgs::Pose {
        geometry_msgs::Pose {
            position: geometry_msgs::Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        }
    }

    /// `operation`: 0 = ADD, 1 = REMOVE per `moveit_msgs/CollisionObject`.
    fn attached(id: &str, operation: u8) -> moveit_msgs::AttachedCollisionObject {
        moveit_msgs::AttachedCollisionObject {
            link_name: "tip".to_string(),
            object: moveit_msgs::CollisionObject {
                header: r2r::std_msgs::msg::Header {
                    frame_id: "base_link".to_string(),
                    ..Default::default()
                },
                pose: identity_pose(),
                id: id.to_string(),
                primitives: vec![r2r::shape_msgs::msg::SolidPrimitive {
                    type_: 2,
                    dimensions: vec![0.1],
                    polygon: Default::default(),
                }],
                primitive_poses: vec![identity_pose()],
                operation,
                ..Default::default()
            },
            touch_links: vec![],
            detach_posture: Default::default(),
            weight: 0.0,
        }
    }

    #[test]
    fn a_successful_change_installs_a_new_scene_and_leaves_the_old_snapshot_alone() {
        let cell = cell();
        let before = snapshot(&cell);
        apply(&cell, |s| {
            s.set_name("changed");
            Ok(())
        })
        .unwrap();
        assert_eq!(snapshot(&cell).name(), "changed");
        // The pre-update snapshot is the point of the `Arc`: a reader holding
        // one still sees what it read, which is what `LockedPlanningSceneRO`
        // gives upstream's readers for the duration of the lock.
        assert_ne!(before.name(), "changed");
    }

    #[test]
    fn a_failed_change_installs_nothing() {
        let cell = cell();
        apply(&cell, |s| {
            s.set_name("halfway");
            Err(moveit_error::Error::other("rejected"))
        })
        .unwrap_err();
        // Not merely "the error propagated": the closure *did* mutate its
        // clone before failing, and what is asserted is that the mutated
        // clone was never installed.
        assert_ne!(snapshot(&cell).name(), "halfway");
    }

    #[test]
    fn an_attach_from_the_topic_is_visible_to_the_next_snapshot() {
        let cell = cell();
        apply_attached_collision_object_msg(&cell, attached("held", 0)).unwrap();
        assert!(snapshot(&cell).has_attached_body("held"));
    }

    #[test]
    fn a_topic_attach_and_a_topic_detach_round_trip() {
        // The sequence a live client actually produces: attachObject then
        // detachObject, both on `attached_collision_object`. The two
        // cross-path tests below cover diff->topic and topic->diff and both
        // passed while this one did not exist -- so the ordering the wire
        // uses most was the one no test covered.
        let cell = cell();
        apply_attached_collision_object_msg(&cell, attached("held", 0)).unwrap();
        assert!(snapshot(&cell).has_attached_body("held"));

        apply_attached_collision_object_msg(&cell, attached("held", 1)).unwrap();
        assert!(!snapshot(&cell).has_attached_body("held"));
    }

    #[test]
    fn a_topic_detach_sees_an_attach_that_arrived_inside_a_scene_diff() {
        let cell = cell();
        // The whole point of one owner. The attach arrives inside a
        // `PlanningScene` diff, the detach arrives on
        // `attached_collision_object`, and the second must see the first.
        // Two mutators with private paths into the scene is exactly the
        // arrangement in which the detach could miss it.
        let mut diff = moveit_msgs::PlanningScene {
            is_diff: true,
            ..Default::default()
        };
        diff.robot_state.attached_collision_objects = vec![attached("held", 0)];
        apply_planning_scene_msg(&cell, diff).unwrap();
        assert!(snapshot(&cell).has_attached_body("held"));

        apply_attached_collision_object_msg(&cell, attached("held", 1)).unwrap();
        assert!(!snapshot(&cell).has_attached_body("held"));
    }

    #[test]
    fn a_scene_diff_detach_sees_an_attach_that_arrived_on_the_topic() {
        let cell = cell();
        // The reverse order, because "one owner" has to hold in both
        // directions: an ordering bug that only shows up when the topic goes
        // first would pass the test above.
        apply_attached_collision_object_msg(&cell, attached("held", 0)).unwrap();
        assert!(snapshot(&cell).has_attached_body("held"));

        let mut diff = moveit_msgs::PlanningScene {
            is_diff: true,
            ..Default::default()
        };
        // REMOVE inside a non-diff `RobotState` is upstream's rejected case
        // (`planning_scene.cpp:1238-1245`) -- so a scene diff cannot detach,
        // and this asserts the rejection names that rule rather than
        // silently leaving the body attached.
        diff.robot_state.attached_collision_objects = vec![attached("held", 1)];
        let err = apply_planning_scene_msg(&cell, diff)
            .unwrap_err()
            .to_string();
        assert!(
            err.contains("is not marked is_diff"),
            "expected the RobotState non-diff guard, got: {err}"
        );
        assert!(snapshot(&cell).has_attached_body("held"));
    }
}
