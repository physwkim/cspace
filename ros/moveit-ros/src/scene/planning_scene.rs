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

use std::sync::Arc;

use moveit_error::{Error, Result};
use moveit_geometry::{Isometry3, OcTree as OcTreeShape, Shape};
use moveit_scene::PlanningScene;
use r2r::moveit_msgs::msg as moveit_msgs;

use super::collision_object::{OCTOMAP_NS, apply_collision_object};
use super::header_frame_transform;
use crate::geometry::Pose;

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
/// An empty `octomap.data` is a no-op -- upstream's own early return
/// (`:1483`) once the previous octomap has been cleared. A non-empty
/// payload is decoded by [`moveit_octomap::OcTree::read_binary_data`] or
/// [`moveit_octomap::OcTree::read_data`] (round 8: those two entry points
/// landed in `moveit-octomap`, closing the round-5/round-7 structural gap
/// this doc comment used to describe) and inserted the same way
/// `apply_collision_object` inserts every other shape kind
/// (`moveit_scene::PlanningScene::add_shape`, `src/scene/collision_object.rs:382`)
/// -- octomap is not a new insertion mechanism, just a new [`Shape`]
/// variant.
///
/// `msg.binary` selects the entry point, not a preference: the two wire
/// formats are structurally different (`read_binary_data`'s 2-bit-per-child
/// compact form vs. `read_data`'s per-node raw `f32`), so one function
/// cannot serve both and neither is a fallback for the other.
///
/// `data: Vec<i8>` (ROS has no unsigned byte array type) is recast `as u8`
/// before either entry point sees it -- `octomap_msgs::readTree`/`readData`
/// treat the wire bytes as raw octets, not signed values; the `i8` on the
/// Rust side is purely `r2r`'s message binding, not a semantic sign.
///
/// `map.origin` is relative to `map.header.frame_id`, not the world --
/// upstream composes `t = getFrameTransform(map.header.frame_id)` with the
/// converted origin (`p = t * p`, `:1494-1497`) before inserting, the same
/// header-to-world resolution `apply_collision_object`/`apply_attached_object`
/// already do for `CollisionObject.header`/`AttachedCollisionObject.object.header`
/// (`src/scene/collision_object.rs:358`, `src/scene/attached.rs:221`) --
/// `OctomapWithPose` is not a special case of that pattern, just another
/// message carrying a header-relative pose.
///
/// Resolved via [`super::header_frame_transform`], not
/// [`PlanningScene::frame_transform`] directly: upstream's `:1494` call has
/// no `knowsFrameTransform` guard in front of it (unlike
/// `processCollisionObjectAdd`'s `:1905`, which does), so an empty
/// `header.frame_id` reaches `getFrameTransform` and resolves to identity
/// through its own silent fallback rather than being rejected as an
/// unresolved name (PORTING-PLAN.md §183).
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

    let mut tree = moveit_octomap::OcTree::new(map.octomap.resolution);
    let bytes: Vec<u8> = map.octomap.data.iter().map(|&b| b as u8).collect();
    let decode_result = if map.octomap.binary {
        tree.read_binary_data(&bytes)
    } else {
        tree.read_data(&bytes)
    };
    decode_result.map_err(|e| Error::other(format!("octomap payload decode failed: {e}")))?;

    let header_transform = header_frame_transform(scene, &map.header.frame_id)?;
    let origin = header_transform * Isometry3::try_from(Pose(map.origin))?;
    let shape = Shape::OcTree(OcTreeShape::from_tree(Arc::new(tree)));
    scene.add_shape(OCTOMAP_NS, Arc::new(shape), origin);
    Ok(())
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

    fn octomap_with_pose(
        id: &str,
        frame_id: &str,
        binary: bool,
        data: Vec<i8>,
    ) -> r2r::octomap_msgs::msg::OctomapWithPose {
        r2r::octomap_msgs::msg::OctomapWithPose {
            header: r2r::std_msgs::msg::Header {
                frame_id: frame_id.to_string(),
                ..Default::default()
            },
            origin: identity_pose(),
            octomap: r2r::octomap_msgs::msg::Octomap {
                header: Default::default(),
                binary,
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
            octomap: octomap_with_pose("OcTree", model.model_frame(), true, vec![]),
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
            octomap: octomap_with_pose("ColorOcTree", model.model_frame(), true, vec![1]),
        };
        let err = apply_planning_scene_world(&mut scene, world).unwrap_err();
        // Not just the variant: `apply_octomap` has a sibling `Error::Other`
        // site (decode failure, hit by `truncated_octree_payload_is_rejected`
        // below) that a bare `matches!` cannot tell apart from this
        // type-name check.
        assert!(
            err.to_string().contains("type 'OcTree' is expected"),
            "got: {err:?}"
        );
    }

    /// Round 7's plan (`ea686a6`) named this exact byte fixture
    /// (`vec![1, 2, 3]`) as a risk: once a real decoder exists, `[1, 2, 3]`
    /// might turn out to be a degenerate *valid* bitstream instead of a
    /// malformed one, and if so that had to be reported rather than quietly
    /// swapped for different bytes. It is: `read_binary_node` reads its
    /// root's two child-packing bytes as `child1to4 = 1 (0b01)`,
    /// `child5to8 = 2 (0b10)` -- child 0 gets the `(1, 0)` "free leaf" code
    /// and child 4 gets the `(0, 1)` "occupied leaf" code, neither packed
    /// byte contains a `(1, 1)` "has children" code, so decoding never
    /// recurses and returns `Ok` without ever looking at the trailing third
    /// byte (matching `read_binary_data`'s own doc: "trailing bytes after a
    /// complete decode are not an error"). `vec![1, 2, 3]` is exercised
    /// directly below, as a **success** case with a named leaf shape, not
    /// as an error case.
    #[test]
    fn truncated_octree_payload_is_rejected() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let world = moveit_msgs::PlanningSceneWorld {
            collision_objects: vec![],
            // One byte: `read_binary_node` reads `child1to4` successfully,
            // then fails reading `child5to8` -- a real `UnexpectedEof`, not
            // a chosen-to-look-malformed value like the retired
            // `vec![1, 2, 3]` fixture turned out to be.
            octomap: octomap_with_pose("OcTree", model.model_frame(), true, vec![1]),
        };
        let err = apply_planning_scene_world(&mut scene, world).unwrap_err();
        // Sibling of `non_octree_octomap_type_is_rejected` above -- this one
        // must name the decode-failure branch, not the type-name check.
        assert!(
            err.to_string().contains("octomap payload decode failed"),
            "got: {err:?}"
        );
    }

    #[test]
    fn binary_octree_payload_is_decoded_and_inserted() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let world = moveit_msgs::PlanningSceneWorld {
            collision_objects: vec![],
            // See `truncated_octree_payload_is_rejected`'s doc comment for
            // the exact bit-level derivation of what these two bytes decode
            // to: root's child 0 a free leaf, child 4 an occupied leaf, no
            // recursion.
            octomap: octomap_with_pose("OcTree", model.model_frame(), true, vec![1, 2]),
        };
        apply_planning_scene_world(&mut scene, world).unwrap();

        let object = scene
            .world()
            .get_object(OCTOMAP_NS)
            .expect("octomap must be inserted at OCTOMAP_NS");
        let shapes = object.shapes();
        assert_eq!(shapes.len(), 1, "got: {shapes:?}");
        let Shape::OcTree(octree_shape) = shapes[0].shape().as_ref() else {
            panic!("got: {:?}", shapes[0].shape());
        };
        let tree = octree_shape.octree.as_ref().expect("tree must be decoded");
        let leaf_log_odds: Vec<f32> = tree.leaves().map(|l| l.log_odds()).collect();
        assert_eq!(leaf_log_odds.len(), 2, "got: {leaf_log_odds:?}");
        assert_eq!(
            tree.leaves().filter(|l| l.is_occupied()).count(),
            1,
            "got: {leaf_log_odds:?}"
        );
        assert_eq!(
            tree.leaves().filter(|l| !l.is_occupied()).count(),
            1,
            "got: {leaf_log_odds:?}"
        );
    }

    #[test]
    fn octomap_origin_with_norm_2_orientation_succeeds_and_normalizes() {
        // §211/§213's tenth-of-ten-minus-one site (the ninth of the nine
        // sharing the generic Pose rule): `apply_octomap`'s `map.origin` ->
        // `Isometry3::try_from(Pose(...))` at planning_scene.rs:147. Same
        // shape as geometry.rs's `pose_with_norm_2_orientation_succeeds_and_normalizes`,
        // run through this site's own full call chain instead of the bare
        // conversion, per PORTING-PLAN.md §215's per-site table.
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let mut world = moveit_msgs::PlanningSceneWorld {
            collision_objects: vec![],
            octomap: octomap_with_pose("OcTree", model.model_frame(), true, vec![1, 2]),
        };
        world.octomap.origin.orientation = r2r::geometry_msgs::msg::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 2.0,
        };
        apply_planning_scene_world(&mut scene, world).unwrap();
        let object = scene
            .world()
            .get_object(OCTOMAP_NS)
            .expect("octomap must be inserted at OCTOMAP_NS");
        assert!(
            (object.pose().rotation.into_inner().norm() - 1.0).abs() < 1e-12,
            "got: {:?}",
            object.pose().rotation
        );
    }

    #[test]
    fn full_octree_payload_is_decoded_and_inserted() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let world = moveit_msgs::PlanningSceneWorld {
            collision_objects: vec![],
            // `read_data_node`: 4-byte little-endian f32 root log-odds
            // (0.5f32 == 0x3F00_0000, LE bytes 0x00 0x00 0x00 0x3F), then a
            // 1-byte child bitmap of 0 -- a single-node tree, no children.
            octomap: octomap_with_pose("OcTree", model.model_frame(), false, vec![0, 0, 0, 63, 0]),
        };
        apply_planning_scene_world(&mut scene, world).unwrap();

        let object = scene
            .world()
            .get_object(OCTOMAP_NS)
            .expect("octomap must be inserted at OCTOMAP_NS");
        let shapes = object.shapes();
        assert_eq!(shapes.len(), 1, "got: {shapes:?}");
        let Shape::OcTree(octree_shape) = shapes[0].shape().as_ref() else {
            panic!("got: {:?}", shapes[0].shape());
        };
        let tree = octree_shape.octree.as_ref().expect("tree must be decoded");
        let leaf_log_odds: Vec<f32> = tree.leaves().map(|l| l.log_odds()).collect();
        assert_eq!(leaf_log_odds, vec![0.5], "got: {leaf_log_odds:?}");
    }

    #[test]
    fn octomap_replaces_any_previous_octree_at_the_reserved_id() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let first = moveit_msgs::PlanningSceneWorld {
            collision_objects: vec![],
            octomap: octomap_with_pose("OcTree", model.model_frame(), true, vec![1, 2]),
        };
        apply_planning_scene_world(&mut scene, first).unwrap();

        let second = moveit_msgs::PlanningSceneWorld {
            collision_objects: vec![],
            octomap: octomap_with_pose("OcTree", model.model_frame(), false, vec![0, 0, 0, 63, 0]),
        };
        apply_planning_scene_world(&mut scene, second).unwrap();

        let object = scene
            .world()
            .get_object(OCTOMAP_NS)
            .expect("octomap must still be present after the second apply");
        let shapes = object.shapes();
        assert_eq!(shapes.len(), 1, "got: {shapes:?}");
        let Shape::OcTree(octree_shape) = shapes[0].shape().as_ref() else {
            panic!("got: {:?}", shapes[0].shape());
        };
        let tree = octree_shape.octree.as_ref().expect("tree must be decoded");
        // The second payload's single root leaf, not the first's two
        // binary-format leaves -- proves `apply_octomap`'s
        // `scene.remove_object(OCTOMAP_NS)` actually discarded the first
        // tree rather than accumulating shapes across calls.
        assert_eq!(tree.leaves().count(), 1, "got: {:?}", scene.world());
    }

    /// `octomap_with_pose`'s other tests all cite the model frame as
    /// `header.frame_id`, which for `one_joint_model()`'s `base_link` root
    /// resolves to identity -- so those tests would pass identically
    /// whether or not `apply_octomap` actually composes
    /// `scene.frame_transform(&map.header.frame_id)`, the exact gap this
    /// crate's own claim audit caught (`doc/claim-audit/moveit-ros.md`,
    /// the `apply_octomap`/`getFrameTransform` row). This test rotates
    /// `j1` away from zero so `"tip"`'s global transform is not identity,
    /// then asserts the inserted shape's pose is the *composed* transform,
    /// not the bare `origin` -- a case that fails on the pre-fix code
    /// (which used `map.origin` directly) and passes on the fixed one.
    #[test]
    fn octomap_origin_is_composed_with_the_header_frame_transform() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        // 0.5 rad, inside `j1`'s `limit lower="-1" upper="1"` (`one_joint_model`'s URDF).
        scene
            .current_state_mut()
            .set_variable_position("j1", 0.5)
            .unwrap();
        let expected_tip_transform = scene
            .current_state_mut()
            .update()
            .global_link_transform("tip")
            .unwrap();
        assert_ne!(
            expected_tip_transform,
            Isometry3::identity(),
            "the fixture must actually exercise a non-identity frame, or this test cannot \
             distinguish composing the transform from skipping it"
        );

        let world = moveit_msgs::PlanningSceneWorld {
            collision_objects: vec![],
            octomap: octomap_with_pose("OcTree", "tip", true, vec![1, 2]),
        };
        apply_planning_scene_world(&mut scene, world).unwrap();

        let object = scene
            .world()
            .get_object(OCTOMAP_NS)
            .expect("octomap must be inserted at OCTOMAP_NS");
        let shapes = object.shapes();
        assert_eq!(shapes.len(), 1, "got: {shapes:?}");
        // origin is identity (`octomap_with_pose`'s own `identity_pose()`),
        // so the composed pose is exactly `expected_tip_transform` -- if
        // `apply_octomap` used `map.origin` bare, this would be identity
        // instead and the assertion below would fail.
        assert_eq!(shapes[0].global_pose(), expected_tip_transform);
    }

    /// An empty `header.frame_id` is upstream's own "already in world
    /// coordinates" default (PORTING-PLAN.md §183), not an unresolved name
    /// -- `processOctomapMsg(OctomapWithPose)` has no `knowsFrameTransform`
    /// guard before `getFrameTransform`, so this succeeds upstream via
    /// `Transforms::getTransform`'s empty-string-to-identity fallback. Before
    /// `header_frame_transform` existed, this message was rejected with
    /// `Err(UnknownName)` -- a real client sending a world-frame octomap
    /// with no `frame_id` set (the message's ordinary use, per §183.2) would
    /// have had every request refused.
    #[test]
    fn empty_header_frame_id_is_accepted_as_the_world_frame() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let world = moveit_msgs::PlanningSceneWorld {
            collision_objects: vec![],
            octomap: octomap_with_pose("OcTree", "", true, vec![1, 2]),
        };
        apply_planning_scene_world(&mut scene, world).unwrap();

        let object = scene
            .world()
            .get_object(OCTOMAP_NS)
            .expect("octomap must be inserted at OCTOMAP_NS");
        assert_eq!(object.shapes()[0].global_pose(), Isometry3::identity());
    }

    /// A non-empty but unresolvable `frame_id` is still rejected -- the
    /// empty-string carve-out in `header_frame_transform` must not swallow
    /// every unresolved name, only the specific "no frame stated" case.
    #[test]
    fn unresolvable_non_empty_header_frame_id_is_still_rejected() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let world = moveit_msgs::PlanningSceneWorld {
            collision_objects: vec![],
            octomap: octomap_with_pose("OcTree", "no-such-frame", true, vec![1, 2]),
        };
        let err = apply_planning_scene_world(&mut scene, world).unwrap_err();
        assert!(matches!(err, Error::UnknownName { .. }), "got: {err:?}");
    }
}
