// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/planning_scene/src/planning_scene.cpp
//     (processAttachedCollisionObjectMsg:1536-1760)

//! `moveit_msgs/msg::AttachedCollisionObject` <-> attached bodies on
//! [`cspace_planning::scene::PlanningScene`]. See `doc/message-mapping.md` §11.
//!
//! Like [`crate::scene::collision_object`], this is an imperative command,
//! not a `TryFrom` in both directions.
//!
//! # `weight` and `detach_posture` are silently dropped
//!
//! `AttachedCollisionObject.weight` ("The weight of the attached object, if
//! known") and `.detach_posture` (a `trajectory_msgs/JointTrajectory` for
//! whatever mechanism releases the object) have no field on
//! [`cspace_planning::scene::AttachedBody`] to receive them -- confirmed absent from
//! that type's full public surface, not merely unused by this module. Both
//! are documentation/orchestration metadata upstream itself only stores and
//! never reads back inside `planning_scene.cpp`'s own processing (no
//! invariant depends on either), matching this crate's existing D1 stance on
//! `object_recognition_msgs::ObjectType` (`crate::scene::collision_object`'s
//! own `type_` field, dropped the same way) -- so these are dropped rather
//! than rejected.
//!
//! Re-checked round 5 against `crates/cspace-planning/src/scene/attached_body.rs:56-63`
//! -- `AttachedBody`'s field list is still `id`/`link_name`/`shapes`/
//! `shape_poses`/`touch_links`/`subframes`, no `weight`. `weight` expires if
//! `AttachedBody` grows that field (`cspace_planning::scene`'s call); `detach_posture`
//! is D1-permanent, not pending-implementation -- it expires only on a
//! project-wide D1 revisit, same as `type_`'s `ObjectType` gap below.

use std::collections::BTreeMap;
use std::sync::Arc;

use cspace_core::error::{Error, Result};
use cspace_core::geometry::{Isometry3, Shape};
use cspace_planning::scene::PlanningScene;
use r2r::moveit_msgs::msg as moveit_msgs;

use super::collision_object::{
    CollisionObjectOperation, OCTOMAP_NS, shapes_and_poses_from_collision_object,
    subframes_from_parallel_arrays,
};
use super::header_frame_transform;

/// Apply one `AttachedCollisionObject` command. Upstream
/// `processAttachedCollisionObjectMsg` (`planning_scene.cpp:1536`).
pub fn apply_attached_collision_object(
    scene: &mut PlanningScene<'_>,
    msg: moveit_msgs::AttachedCollisionObject,
) -> Result<()> {
    if msg.object.id == OCTOMAP_NS {
        return Err(Error::other(format!(
            "the ID '{OCTOMAP_NS}' cannot be used for collision objects (name reserved)"
        )));
    }
    match CollisionObjectOperation::try_from(msg.object.operation)? {
        CollisionObjectOperation::Add => apply_attach(scene, msg, true),
        CollisionObjectOperation::Append => apply_attach(scene, msg, false),
        CollisionObjectOperation::Remove => apply_detach(scene, msg),
        CollisionObjectOperation::Move => Err(Error::other(
            "MOVE is not implemented for attached objects -- upstream itself has not \
             implemented it either: processAttachedCollisionObjectMsg's MOVE branch is\
             \n  RCLCPP_ERROR(getLogger(), \"Move for attached objects not yet implemented\");\
             \n(planning_scene.cpp:1762-1765)",
        )),
    }
}

/// ADD and APPEND. Upstream's ADD/APPEND branch (`:1567-1622`) is one
/// shared code path gated by `operation == ADD || !hasAttachedBody(id)`:
/// APPEND behaves exactly like ADD (fresh attach, no merge) whenever there
/// is no pre-existing attached body of the same id to append onto.
fn apply_attach(
    scene: &mut PlanningScene<'_>,
    msg: moveit_msgs::AttachedCollisionObject,
    is_add: bool,
) -> Result<()> {
    let moveit_msgs::AttachedCollisionObject {
        link_name,
        object,
        touch_links,
        detach_posture: _,
        weight: _,
    } = msg;
    let mut merged_touch_links: std::collections::BTreeSet<String> =
        touch_links.into_iter().collect();

    let no_geometry =
        object.primitives.is_empty() && object.meshes.is_empty() && object.planes.is_empty();

    // World-object promotion: gated specifically on `operation == ADD`
    // (`:1576`), *not* APPEND -- confirmed by reading the condition
    // directly, not assumed from ADD/APPEND being handled together above.
    if is_add && no_geometry {
        if scene.world().has_object(&object.id) {
            return scene.attach(&object.id, &link_name, merged_touch_links);
        }
        return Err(Error::other(format!(
            "attempted to attach object '{}' to link '{link_name}', but no geometry was \
             specified and the object does not exist in the collision world \
             (processAttachedCollisionObjectMsg)",
            object.id
        )));
    }

    let id = object.id.clone();

    let (mut shapes, mut shape_poses, mut subframes) =
        shapes_from_message_geometry(scene, &link_name, object)?;

    if shapes.is_empty() {
        return Err(Error::other(format!(
            "there is no geometry to attach to link '{link_name}' as part of attached body '{id}' \
             (processAttachedCollisionObjectMsg)"
        )));
    }

    // APPEND onto an existing attached body: merge, matching upstream's
    // `:1602-1622`. `is_add` always takes the fresh-attach path below
    // instead (`attach_new`'s insert replaces any same-named entry
    // outright), reproducing `clearAttachedBody` + `attachBody` in one step.
    if !is_add {
        if let Some(old) = scene.attached_body(&id) {
            shapes.extend(old.shapes().iter().cloned());
            shape_poses.extend(old.shape_poses().iter().copied());
            merged_touch_links.extend(old.touch_links().iter().cloned());
            for name in old.subframe_names() {
                let pose = old
                    .subframe_pose(name)
                    .expect("name was just listed by subframe_names");
                // Upstream's `std::map::insert` keeps the first value on a
                // duplicate key; the message's own subframes were inserted
                // first (`:1611`), so a name in both wins from the message,
                // not the old body (`:1612`).
                subframes.entry(name.to_owned()).or_insert(pose);
            }
        }
    }

    scene.attach_new(
        &id,
        &link_name,
        shapes,
        shape_poses,
        merged_touch_links,
        subframes,
    )
}

/// Link-relative `(shapes, shape_poses, subframes)`, ready for
/// [`cspace_planning::scene::PlanningScene::attach_new`].
type LinkRelativeGeometry = (Vec<Arc<Shape>>, Vec<Isometry3>, BTreeMap<String, Isometry3>);

/// The message-geometry path shared by ADD (with geometry) and APPEND:
/// converts `CollisionObject`'s shape arrays the same way
/// [`crate::scene::collision_object`] does for world objects, then
/// re-expresses every pose relative to `link_name` instead of the world --
/// composing `header_frame -> world -> link` in one step, mirroring
/// `PlanningScene::attach`'s own `link_transform.inverse() * object_pose *
/// s.pose()` (`scene.rs:1180`).
///
/// `header.frame_id` is resolved via [`super::header_frame_transform`], not
/// [`cspace_planning::scene::PlanningScene::frame_transform`] directly: upstream's
/// `getFrameTransform(object.object.header.frame_id)` call here
/// (`planning_scene.cpp:1606`) has no `knowsFrameTransform` guard in front
/// of it, so an empty `header.frame_id` resolves to identity through
/// `getFrameTransform`'s own silent fallback rather than being rejected
/// (PORTING-PLAN.md §183).
///
/// The whole `CollisionObject` is taken rather than its nine geometry fields
/// and its header frame spread across the signature: four of those nine are
/// `Vec<geometry_msgs::Pose>`, so any two of the four could be transposed at
/// the call site and still compile. The destructure below binds them by field
/// name, which is the same check the caller was performing by hand before
/// handing all nine straight back in positional order.
fn shapes_from_message_geometry(
    scene: &mut PlanningScene<'_>,
    link_name: &str,
    object: moveit_msgs::CollisionObject,
) -> Result<LinkRelativeGeometry> {
    let moveit_msgs::CollisionObject {
        header,
        pose,
        primitives,
        primitive_poses,
        meshes,
        mesh_poses,
        planes,
        plane_poses,
        subframe_names,
        subframe_poses,
        ..
    } = object;

    let (local_object_pose, shapes, local_shape_poses) = shapes_and_poses_from_collision_object(
        pose,
        primitives,
        primitive_poses,
        meshes,
        mesh_poses,
        planes,
        plane_poses,
    )?;
    let local_subframes = subframes_from_parallel_arrays(
        "processAttachedCollisionObjectMsg:1612/1615",
        subframe_names,
        subframe_poses,
    )?;

    let world_to_header = header_frame_transform(scene, &header.frame_id)?;
    let link_transform = {
        let posed = scene.current_state_mut().update();
        posed.global_link_transform(link_name)?
    };
    let link_to_header = link_transform.inverse() * world_to_header;
    let object_pose_in_link = link_to_header * local_object_pose;

    let shape_poses = local_shape_poses
        .into_iter()
        .map(|p| object_pose_in_link * p)
        .collect();
    let subframes = local_subframes
        .into_iter()
        .map(|(name, p)| (name, object_pose_in_link * p))
        .collect();

    Ok((shapes, shape_poses, subframes))
}

/// REMOVE (detach). Upstream's REMOVE branch (`:1631-1667`).
///
/// # Two asymmetries, both required explicitly (`scene.detach` alone gives
/// neither)
///
/// An empty `object.id` detaches every attached body (optionally filtered
/// by `link_name`) and **always succeeds, even if zero bodies matched** --
/// upstream's own return expression is `!attached_bodies.empty() ||
/// object.object.id.empty()` (`:1665`), true unconditionally when the id is
/// empty. A specific, non-empty id that resolves to nothing is a
/// **failure** -- the same expression is false when the id is non-empty and
/// nothing matched.
///
/// # A landmine *not* reproduced, and why
///
/// If `link_name` is given but does not name a real link on the robot,
/// upstream's `link_name.empty() ? nullptr : getLinkModel(link_name)`
/// (`:1633`) resolves to `nullptr`, and its `if (link_model) { filter } else
/// { match everything }` (`:1636`) then falls back to matching *every*
/// attached body -- a typo in `link_name` silently detaches unrelated
/// bodies instead of matching zero or erroring. This port does not
/// reproduce that fallback: a `link_name` that matches no attached body's
/// own `link_name()` simply filters to zero matches (a bogus link name can
/// never legitimately match a real attached body's link, since attach/
/// attach_new both reject unknown links up front), which is safer and more
/// predictable than "matches everything" -- but it is a deliberate,
/// documented parity deviation from `processAttachedCollisionObjectMsg`,
/// not an oversight.
fn apply_detach(
    scene: &mut PlanningScene<'_>,
    msg: moveit_msgs::AttachedCollisionObject,
) -> Result<()> {
    let id = msg.object.id;
    let link_name = msg.link_name;

    if id.is_empty() {
        let ids: Vec<String> = scene
            .attached_bodies()
            .filter(|body| link_name.is_empty() || body.link_name() == link_name)
            .map(|body| body.id().to_owned())
            .collect();
        for id in ids {
            scene.detach(&id)?;
        }
        return Ok(());
    }

    if let Some(body) = scene.attached_body(&id) {
        if !link_name.is_empty() && body.link_name() != link_name {
            return Err(Error::other(format!(
                "AttachedCollisionObject names link '{link_name}', but '{id}' is actually \
                 attached to '{}' -- leave link_name empty or name the correct link \
                 (processAttachedCollisionObjectMsg)",
                body.link_name()
            )));
        }
    }
    scene.detach(&id).map(|_| ())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cspace_core::model::{MeshSearchPaths, RobotModel};
    use cspace_core::srdf::SrdfModel;
    // Only the fixtures below name this alias now: the non-test code reaches
    // every pose through `moveit_msgs::CollisionObject`'s own fields.
    use r2r::geometry_msgs::msg as geometry_msgs;

    /// Asserts the call was rejected *for the reason named*, not merely
    /// that it was rejected. `apply_attach` has two independent
    /// `Error::Other` "no geometry" sites (world-object-promotion path vs.
    /// the message-geometry path) -- `matches!(err, Error::Other(_))` alone
    /// cannot tell a test that a routing bug swapped which branch fired
    /// (same shape as `cspace_planning::constraints`' `e3b40c6`).
    #[track_caller]
    fn assert_err_mentions<T: std::fmt::Debug>(
        result: std::result::Result<T, Error>,
        needle: &str,
    ) {
        let rendered = result
            .expect_err("expected this call to be rejected")
            .to_string();
        assert!(
            rendered.contains(needle),
            "expected the rejection to come from the branch that reports {needle:?}, got: {rendered}"
        );
    }

    fn two_link_model() -> RobotModel {
        let urdf_xml = r#"<?xml version="1.0"?>
<robot name="two_joint">
  <link name="base_link"/>
  <link name="mid"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="mid"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="mid"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>"#;
        let srdf_xml = r#"<?xml version="1.0"?><robot name="two_joint"></robot>"#;
        let urdf = urdf_rs::read_from_string(urdf_xml).expect("inline URDF must parse");
        let srdf = SrdfModel::parse_str(srdf_xml).expect("inline SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("valid two-joint urdf")
    }

    fn scene(model: &RobotModel) -> PlanningScene<'_> {
        let srdf =
            SrdfModel::parse_str("<?xml version=\"1.0\"?><robot name=\"two_joint\"></robot>")
                .expect("empty SRDF must parse");
        PlanningScene::new(model, &srdf)
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

    fn sphere(radius: f64) -> r2r::shape_msgs::msg::SolidPrimitive {
        r2r::shape_msgs::msg::SolidPrimitive {
            type_: 2,
            dimensions: vec![radius],
            polygon: Default::default(),
        }
    }

    fn base_collision_object(
        id: &str,
        model_frame: &str,
        operation: u8,
    ) -> moveit_msgs::CollisionObject {
        moveit_msgs::CollisionObject {
            header: r2r::std_msgs::msg::Header {
                frame_id: model_frame.to_string(),
                ..Default::default()
            },
            pose: identity_pose(),
            id: id.to_string(),
            type_: Default::default(),
            primitives: vec![sphere(0.1)],
            primitive_poses: vec![identity_pose()],
            meshes: vec![],
            mesh_poses: vec![],
            planes: vec![],
            plane_poses: vec![],
            subframe_names: vec![],
            subframe_poses: vec![],
            operation,
        }
    }

    fn base_attached(
        id: &str,
        link_name: &str,
        model_frame: &str,
        operation: u8,
    ) -> moveit_msgs::AttachedCollisionObject {
        moveit_msgs::AttachedCollisionObject {
            link_name: link_name.to_string(),
            object: base_collision_object(id, model_frame, operation),
            touch_links: vec![],
            detach_posture: Default::default(),
            weight: 0.0,
        }
    }

    #[test]
    fn octomap_ns_is_rejected() {
        let model = two_link_model();
        let mut sc = scene(&model);
        let msg = base_attached(OCTOMAP_NS, "tip", model.model_frame(), 0);
        let err = apply_attached_collision_object(&mut sc, msg).unwrap_err();
        // Not just the variant: `apply_attached_collision_object` has 4 more
        // sibling Error::Other sites (`move_is_rejected` below and the two
        // no-geometry checks `assert_err_mentions`-tested further down).
        assert!(err.to_string().contains("name reserved"), "got: {err:?}");
    }

    #[test]
    fn move_is_rejected() {
        let model = two_link_model();
        let mut sc = scene(&model);
        let msg = base_attached("box", "tip", model.model_frame(), 3);
        let err = apply_attached_collision_object(&mut sc, msg).unwrap_err();
        // Not just the variant: sibling of `octomap_ns_is_rejected` above.
        assert!(
            err.to_string().contains("not yet implemented"),
            "got: {err:?}"
        );
    }

    #[test]
    fn add_from_message_geometry_attaches_to_link() {
        let model = two_link_model();
        let mut sc = scene(&model);
        let msg = base_attached("box", "tip", model.model_frame(), 0);
        apply_attached_collision_object(&mut sc, msg).unwrap();
        let body = sc.attached_body("box").unwrap();
        assert_eq!(body.link_name(), "tip");
        assert_eq!(body.shapes().len(), 1);
    }

    /// An empty `object.header.frame_id` is accepted as the world frame
    /// (PORTING-PLAN.md §183) -- `processAttachedCollisionObjectMsg`'s
    /// not-in-world branch has no `knowsFrameTransform` guard before
    /// `getFrameTransform` (`planning_scene.cpp:1606`). Before
    /// `header_frame_transform` existed, this would have been rejected with
    /// `Err(UnknownName)`.
    #[test]
    fn add_from_message_geometry_accepts_empty_header_frame_id() {
        let model = two_link_model();
        let mut sc = scene(&model);
        let msg = base_attached("box", "tip", "", 0);
        apply_attached_collision_object(&mut sc, msg).unwrap();
        let body = sc.attached_body("box").unwrap();
        assert_eq!(body.link_name(), "tip");
        assert_eq!(body.shapes().len(), 1);
    }

    #[test]
    fn add_promotes_existing_world_object() {
        let model = two_link_model();
        let mut sc = scene(&model);
        super::super::collision_object::apply_collision_object(
            &mut sc,
            base_collision_object("box", model.model_frame(), 0),
        )
        .unwrap();
        assert!(sc.world().has_object("box"));

        let mut attach_msg = base_attached("box", "tip", model.model_frame(), 0);
        attach_msg.object.primitives = vec![];
        attach_msg.object.primitive_poses = vec![];
        apply_attached_collision_object(&mut sc, attach_msg).unwrap();

        assert!(!sc.world().has_object("box"));
        assert!(sc.has_attached_body("box"));
    }

    #[test]
    fn add_with_no_geometry_and_no_world_object_is_rejected() {
        let model = two_link_model();
        let mut sc = scene(&model);
        let mut msg = base_attached("box", "tip", model.model_frame(), 0);
        msg.object.primitives = vec![];
        msg.object.primitive_poses = vec![];
        assert_err_mentions(
            apply_attached_collision_object(&mut sc, msg),
            "does not exist in the collision world",
        );
    }

    #[test]
    fn add_replaces_existing_attached_body_instead_of_merging() {
        let model = two_link_model();
        let mut sc = scene(&model);
        apply_attached_collision_object(
            &mut sc,
            base_attached("box", "tip", model.model_frame(), 0),
        )
        .unwrap();
        let mut second = base_attached("box", "tip", model.model_frame(), 0);
        second.touch_links = vec!["should_not_survive".to_string()];
        apply_attached_collision_object(&mut sc, second).unwrap();
        let body = sc.attached_body("box").unwrap();
        assert!(body.touch_links().contains("should_not_survive"));
        // ADD replaces wholesale: since both messages carry identical
        // geometry, the meaningful assertion is that the shape count did
        // not double (a merge, like APPEND, would leave 2 shapes here).
        assert_eq!(body.shapes().len(), 1);
    }

    #[test]
    fn append_onto_existing_attached_body_merges_shapes_and_touch_links() {
        let model = two_link_model();
        let mut sc = scene(&model);
        let mut first = base_attached("box", "tip", model.model_frame(), 0);
        first.touch_links = vec!["tip".to_string()];
        apply_attached_collision_object(&mut sc, first).unwrap();

        let mut second = base_attached("box", "tip", model.model_frame(), 2);
        second.touch_links = vec!["mid".to_string()];
        apply_attached_collision_object(&mut sc, second).unwrap();

        let body = sc.attached_body("box").unwrap();
        assert_eq!(body.shapes().len(), 2);
        assert!(body.touch_links().contains("tip"));
        assert!(body.touch_links().contains("mid"));
    }

    #[test]
    fn append_subframe_conflict_keeps_new_message_value() {
        let model = two_link_model();
        let mut sc = scene(&model);
        let mut first = base_attached("box", "tip", model.model_frame(), 0);
        first.object.subframe_names = vec!["grip".to_string()];
        first.object.subframe_poses = vec![identity_pose()];
        apply_attached_collision_object(&mut sc, first).unwrap();

        let mut new_pose = identity_pose();
        new_pose.position.x = 5.0;
        let mut second = base_attached("box", "tip", model.model_frame(), 2);
        second.object.subframe_names = vec!["grip".to_string()];
        second.object.subframe_poses = vec![new_pose];
        apply_attached_collision_object(&mut sc, second).unwrap();

        let body = sc.attached_body("box").unwrap();
        let grip = body.subframe_pose("grip").unwrap();
        assert_eq!(
            grip.translation.vector.x, 5.0,
            "message subframe must win over old on conflict"
        );
    }

    #[test]
    fn append_with_no_new_geometry_is_rejected() {
        let model = two_link_model();
        let mut sc = scene(&model);
        apply_attached_collision_object(
            &mut sc,
            base_attached("box", "tip", model.model_frame(), 0),
        )
        .unwrap();

        let mut append = base_attached("box", "tip", model.model_frame(), 2);
        append.object.primitives = vec![];
        append.object.primitive_poses = vec![];
        assert_err_mentions(
            apply_attached_collision_object(&mut sc, append),
            "no geometry to attach",
        );
    }

    #[test]
    fn mismatched_subframe_array_lengths_are_rejected() {
        let model = two_link_model();
        let mut sc = scene(&model);
        let mut msg = base_attached("box", "tip", model.model_frame(), 0);
        msg.object.subframe_names = vec!["a".to_string(), "b".to_string()];
        msg.object.subframe_poses = vec![identity_pose()];
        let err = apply_attached_collision_object(&mut sc, msg).unwrap_err();
        // Not just the variant: `shapes_and_poses_from_collision_object`
        // (`parallel_shapes`, imported from `collision_object.rs`) is a
        // sibling Error::Construct site, reachable through the same
        // `shapes_from_message_geometry` call.
        assert!(
            err.to_string().contains("subframe_names has length"),
            "got: {err:?}"
        );
    }

    #[test]
    fn detach_specific_id() {
        let model = two_link_model();
        let mut sc = scene(&model);
        apply_attached_collision_object(
            &mut sc,
            base_attached("box", "tip", model.model_frame(), 0),
        )
        .unwrap();
        apply_attached_collision_object(
            &mut sc,
            base_attached("box", "tip", model.model_frame(), 1),
        )
        .unwrap();
        assert!(!sc.has_attached_body("box"));
        assert!(sc.world().has_object("box"));
    }

    #[test]
    fn detach_unknown_id_is_rejected() {
        let model = two_link_model();
        let mut sc = scene(&model);
        let err = apply_attached_collision_object(
            &mut sc,
            base_attached("box", "tip", model.model_frame(), 1),
        )
        .unwrap_err();
        // Not just the variant: `PlanningScene::detach`'s own "world already
        // has an object with that name" site, and `apply_detach`'s own
        // link-name-mismatch site (`detach_link_name_mismatch_is_rejected`
        // below), are siblings.
        assert!(
            err.to_string().contains("no attached body named"),
            "got: {err:?}"
        );
    }

    #[test]
    fn detach_link_name_mismatch_is_rejected() {
        let model = two_link_model();
        let mut sc = scene(&model);
        apply_attached_collision_object(
            &mut sc,
            base_attached("box", "tip", model.model_frame(), 0),
        )
        .unwrap();
        let err = apply_attached_collision_object(
            &mut sc,
            base_attached("box", "mid", model.model_frame(), 1),
        )
        .unwrap_err();
        // Not just the variant: sibling of `detach_unknown_id_is_rejected`
        // above.
        assert!(
            err.to_string().contains("is actually attached to"),
            "got: {err:?}"
        );
        assert!(
            sc.has_attached_body("box"),
            "rejected detach must leave the body attached"
        );
    }

    #[test]
    fn detach_empty_id_detaches_everything_and_succeeds_with_zero_matches() {
        let model = two_link_model();
        let mut sc = scene(&model);
        apply_attached_collision_object(
            &mut sc,
            base_attached("box", "tip", model.model_frame(), 0),
        )
        .unwrap();
        let mut detach_all = base_attached("", "", model.model_frame(), 1);
        detach_all.object.id = String::new();
        apply_attached_collision_object(&mut sc, detach_all.clone()).unwrap();
        assert!(!sc.has_attached_body("box"));
        // Zero attached bodies left: detaching again with empty id must
        // still succeed (upstream: `!attached_bodies.empty() ||
        // object.object.id.empty()` is true unconditionally when id is
        // empty).
        apply_attached_collision_object(&mut sc, detach_all).unwrap();
    }

    #[test]
    fn detach_filters_by_link_name() {
        let model = two_link_model();
        let mut sc = scene(&model);
        apply_attached_collision_object(
            &mut sc,
            base_attached("on_mid", "mid", model.model_frame(), 0),
        )
        .unwrap();
        apply_attached_collision_object(
            &mut sc,
            base_attached("on_tip", "tip", model.model_frame(), 0),
        )
        .unwrap();

        let mut detach_mid_only = base_attached("", "mid", model.model_frame(), 1);
        detach_mid_only.object.id = String::new();
        apply_attached_collision_object(&mut sc, detach_mid_only).unwrap();

        assert!(!sc.has_attached_body("on_mid"));
        assert!(
            sc.has_attached_body("on_tip"),
            "detach-by-link must not touch other links' bodies"
        );
    }
}
