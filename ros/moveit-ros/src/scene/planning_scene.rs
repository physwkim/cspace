// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/planning_scene/src/planning_scene.cpp
//     (processPlanningSceneWorldMsg:1396, processOctomapMsg(OctomapWithPose):1478,
//      usePlanningSceneMsg:1405, setPlanningSceneMsg:1366, setPlanningSceneDiffMsg:1330)

//! `moveit_msgs/msg::{PlanningScene, PlanningSceneWorld}` <->
//! [`moveit_scene::PlanningScene`]. See `doc/message-mapping.md` §11.
//!
//! # Scope: `PlanningSceneWorld` only, not the full `PlanningScene` message
//!
//! [`apply_planning_scene_world`] ports `processPlanningSceneWorldMsg`
//! (`world.collision_objects` + `world.octomap`) -- the two fields the
//! round-3 brief names explicitly. It does **not** port the rest of
//! `usePlanningSceneMsg`/`setPlanningSceneMsg`/`setPlanningSceneDiffMsg`:
//! `robot_state`, `fixed_frame_transforms`, `allowed_collision_matrix`,
//! `link_padding`/`link_scale`, `object_colors`, or diff-vs-full dispatch
//! against a parent scene. Each of those is a real, separately-sized
//! conversion (`RobotState` already exists in `crate::state`; the rest have
//! no conversion in this crate yet) -- naming that gap here rather than
//! silently leaving it undiscoverable.
//!
//! [`is_diff`] and [`robot_model_name_matches`] are the two small, real
//! pieces of `PlanningScene` (the message) this module *does* cover: they
//! answer "does this scene already look like a diff scene" and "does this
//! scene's model match the name the message expects", both needed by
//! whatever future caller assembles the rest of `usePlanningSceneMsg`.

use moveit_error::{Error, Result};
use moveit_scene::PlanningScene;
use r2r::moveit_msgs::msg as moveit_msgs;

use super::collision_object::{OCTOMAP_NS, apply_collision_object};

/// `PlanningScene.is_diff`'s exact meaning: this scene has a parent it is
/// layered as a diff on top of. Upstream sets/reads `is_diff` as a plain
/// message field it round-trips (`GeometricShapesPlanningSceneMsgConversions`
/// has no core-side "am I a diff" query -- the *scene* object's own
/// diff-ness lives in whether `parent_` is set, e.g. `decoupleParent`,
/// `pushDiffs`). [`PlanningScene::parent`] is exactly that.
pub fn is_diff(scene: &PlanningScene<'_>) -> bool {
    scene.parent().is_some()
}

/// `PlanningScene.robot_model_name` compatibility check. Upstream's
/// `setPlanningSceneMsg` treats an *empty* `robot_model_name` as "no claim
/// made" (skips the compatibility check entirely, `:1370`) and a mismatched
/// non-empty name as a `RCLCPP_WARN`, not a hard error -- so this returns a
/// bool for the caller to log/decide on, rather than an `Err`.
pub fn robot_model_name_matches(scene: &PlanningScene<'_>, robot_model_name: &str) -> bool {
    robot_model_name.is_empty() || scene.robot_model().name() == robot_model_name
}

/// `PlanningSceneWorld.collision_objects` + `.octomap`. Upstream
/// `processPlanningSceneWorldMsg` (`planning_scene.cpp:1396`): every
/// collision object is applied in array order (first failure does not stop
/// the rest -- upstream ANDs a `bool` across all of them and keeps going;
/// this port instead stops at the first `Err`, a deliberate deviation since
/// "which of N objects failed" is more useful to a caller than "at least one
/// failed", and D6's own error-over-silent-partial-success stance already
/// prefers stopping over silently downgrading a failure to a dropped bit),
/// then the octomap always replaces any previous one.
pub fn apply_planning_scene_world(
    scene: &mut PlanningScene<'_>,
    world: moveit_msgs::PlanningSceneWorld,
) -> Result<()> {
    for collision_object in world.collision_objects {
        apply_collision_object(scene, collision_object)?;
    }
    apply_octomap(scene, world.octomap)
}

/// Upstream `processOctomapMsg(const octomap_msgs::msg::OctomapWithPose&)`
/// (`planning_scene.cpp:1478`).
///
/// # Structural gap: octomap binary payload decoding belongs to `moveit-octomap`
///
/// An empty `octomap.data` is a correct, real no-op -- upstream's own early
/// return (`:1483`) once the previous octomap has been cleared. A
/// **non-empty** payload requires decoding octomap's binary tree
/// serialization (`octomap_msgs::readTree` / `OcTree::readData`) into a
/// [`moveit_octomap::OcTree`] -- confirmed absent from that type's public
/// surface (only `OcTree::new(resolution)` builds an empty tree; `Node` is
/// `pub(crate)` and `OcTree::root` is private, so `ros/` cannot reach the
/// primitives a decoder would need even if it wanted to).
///
/// **Decided round 5: this decoder is `moveit-octomap`'s (p3-shapes'), not
/// `ros/`'s.** octomap's binary serialization is octomap's own format, not a
/// ROS format -- `octomap_msgs::readTree`/`readData` write `msg.data`
/// straight into `OcTree::readBinaryData`/`readData`, bypassing any file
/// header or the `AbstractOcTree` registry entirely. Deserializing a type's
/// own format is that type's owning crate's job; exposing `Node`/`root` as
/// `pub` just so `ros/` could decode here would invert encapsulation for
/// `ros/`'s convenience. See `doc/message-mapping.md` §11's "Structural
/// gaps" list for the full requirements spec this crate has written for
/// `moveit-octomap`'s owner (API signature, upstream file:line citations,
/// this call site, and the verification approach) -- not implemented here.
fn apply_octomap(
    scene: &mut PlanningScene<'_>,
    map: r2r::octomap_msgs::msg::OctomapWithPose,
) -> Result<()> {
    let _ = scene.remove_object(OCTOMAP_NS);

    if map.octomap.data.is_empty() {
        return Ok(());
    }
    if map.octomap.id != "OcTree" {
        return Err(Error::other(format!(
            "received octomap is of type '{}' but type 'OcTree' is expected (processOctomapMsg)",
            map.octomap.id
        )));
    }
    Err(Error::other(
        "non-empty Octomap.data cannot be converted: moveit_octomap::OcTree has no binary-payload \
         decoder (octomap_msgs::readTree/OcTree::readData is unported) (doc/message-mapping.md §11)",
    ))
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;
    use crate::state::tests::one_joint_model;
    use moveit_srdf::SrdfModel;

    fn empty_srdf() -> SrdfModel {
        SrdfModel::parse_str("<?xml version=\"1.0\"?><robot name=\"one_joint\"></robot>")
            .expect("empty SRDF must parse")
    }

    fn identity_pose() -> r2r::geometry_msgs::msg::Pose {
        r2r::geometry_msgs::msg::Pose {
            position: r2r::geometry_msgs::msg::Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: r2r::geometry_msgs::msg::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        }
    }

    fn octomap_with_pose(id: &str, data: Vec<i8>) -> r2r::octomap_msgs::msg::OctomapWithPose {
        r2r::octomap_msgs::msg::OctomapWithPose {
            header: Default::default(),
            origin: identity_pose(),
            octomap: r2r::octomap_msgs::msg::Octomap {
                header: Default::default(),
                binary: true,
                id: id.to_string(),
                resolution: 0.1,
                data,
            },
        }
    }

    #[test]
    fn is_diff_reflects_parent() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let scene = Arc::new(PlanningScene::new(&model, &srdf));
        assert!(!is_diff(&scene));
        let child = scene.diff();
        assert!(is_diff(&child));
    }

    #[test]
    fn robot_model_name_matches_empty_and_exact() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let scene = PlanningScene::new(&model, &srdf);
        assert!(robot_model_name_matches(&scene, ""));
        assert!(robot_model_name_matches(&scene, model.name()));
        assert!(!robot_model_name_matches(&scene, "some_other_robot"));
    }

    #[test]
    fn empty_collision_objects_and_empty_octomap_is_a_no_op() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let world = moveit_msgs::PlanningSceneWorld {
            collision_objects: vec![],
            octomap: octomap_with_pose("OcTree", vec![]),
        };
        apply_planning_scene_world(&mut scene, world).unwrap();
        assert!(scene.world().is_empty());
    }

    #[test]
    fn non_octree_octomap_type_is_rejected() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let world = moveit_msgs::PlanningSceneWorld {
            collision_objects: vec![],
            octomap: octomap_with_pose("ColorOcTree", vec![1]),
        };
        let err = apply_planning_scene_world(&mut scene, world).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "got: {err:?}");
    }

    #[test]
    fn nonempty_octree_payload_names_the_structural_gap() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let world = moveit_msgs::PlanningSceneWorld {
            collision_objects: vec![],
            octomap: octomap_with_pose("OcTree", vec![1, 2, 3]),
        };
        let err = apply_planning_scene_world(&mut scene, world).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "got: {err:?}");
    }
}
