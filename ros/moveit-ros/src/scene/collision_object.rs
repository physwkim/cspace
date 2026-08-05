// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2019, Universitaet Hamburg.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/planning_scene/src/planning_scene.cpp
//     (processCollisionObjectMsg:1758, shapesAndPosesFromCollisionObjectMessage:1800,
//      processCollisionObjectAdd:1887, processCollisionObjectRemove:1931,
//      processCollisionObjectMove:1953)
//   moveit_core/utils/src/message_checks.cpp (isEmpty(Pose):77)

//! `moveit_msgs/msg::CollisionObject` <-> world objects on
//! [`moveit_scene::PlanningScene`]. See `doc/message-mapping.md` §11.
//!
//! Unlike every other conversion in this crate, this is not a `TryFrom` in
//! both directions (D6's usual shape): `CollisionObject` is an imperative
//! *command* (ADD/REMOVE/APPEND/MOVE) applied to an existing scene, not a
//! value with a core-side isomorph to convert to and from. Upstream itself
//! reflects this -- `processCollisionObjectMsg` takes `(&mut PlanningScene,
//! &CollisionObject)` and returns `bool`, not a constructed value -- so
//! [`apply_collision_object`] takes the same shape: `&mut PlanningScene`
//! plus the message, `Result<()>` back.

use std::collections::BTreeMap;
use std::sync::Arc;

use moveit_error::{Error, Result};
use moveit_geometry::{Isometry3, Plane, Shape};
use moveit_scene::PlanningScene;
use r2r::geometry_msgs::msg as geometry_msgs;
use r2r::moveit_msgs::msg as moveit_msgs;
use r2r::shape_msgs::msg as shape_msgs;

use super::header_frame_transform;
use super::shapes::{MeshMsg, PlaneMsg};
use crate::constraints::position::SolidPrimitiveMsg;
use crate::geometry::Pose;

/// Upstream `PlanningScene::OCTOMAP_NS` (`planning_scene.hpp:113`): a
/// reserved collision-object id. `processCollisionObjectMsg` (`:1758`)
/// rejects it for every operation (ADD/REMOVE/APPEND/MOVE alike) --
/// *not* just ADD/APPEND, confirmed by reading the dispatcher, not assumed.
pub const OCTOMAP_NS: &str = "<octomap>";

/// `r2r` generates each `.msg`-declared `byte`/`int32` constant as its own
/// bindgen-derived type (`moveit_msgs::CollisionObject::ADD: _bindgen_ty_404`
/// etc, one anonymous single-variant `#[repr(u32)]` enum per constant) --
/// referencing those instead of re-declaring the numbers here means a
/// `third_party/moveit_msgs` repin that renumbers one of these is followed
/// automatically (`as u8` just casts whatever the new discriminant is), not
/// a silent divergence from a stale local literal. It is *not* the case
/// that renumbering becomes a compile error -- `as u8` accepts any value the
/// enum's single variant carries. Only deleting or renaming the constant
/// itself would fail to compile (PORTING-PLAN.md §191, corrected by the
/// coordinator: the previous wording of this comment claimed the wrong
/// benefit).
const ADD: u8 = moveit_msgs::CollisionObject::ADD as u8;
const REMOVE: u8 = moveit_msgs::CollisionObject::REMOVE as u8;
const APPEND: u8 = moveit_msgs::CollisionObject::APPEND as u8;
const MOVE: u8 = moveit_msgs::CollisionObject::MOVE as u8;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionObjectOperation {
    /// `CollisionObject::ADD` (0).
    Add,
    /// `CollisionObject::REMOVE` (1).
    Remove,
    /// `CollisionObject::APPEND` (2).
    Append,
    /// `CollisionObject::MOVE` (3).
    Move,
}

impl TryFrom<u8> for CollisionObjectOperation {
    type Error = Error;

    fn try_from(operation: u8) -> Result<Self> {
        match operation {
            ADD => Ok(Self::Add),
            REMOVE => Ok(Self::Remove),
            APPEND => Ok(Self::Append),
            MOVE => Ok(Self::Move),
            other => Err(Error::construct(format!(
                "CollisionObject.operation={other} is none of ADD(0)/REMOVE(1)/APPEND(2)/MOVE(3)"
            ))),
        }
    }
}

/// Upstream `moveit::core::isEmpty(const geometry_msgs::msg::Pose&)`
/// (`message_checks.cpp:77-79`): exactly the wire-default identity pose,
/// checked on the *raw* message fields -- not via a round trip through
/// [`Isometry3`], so this matches upstream's literal `== 0.0`/`== 1.0`
/// comparisons instead of comparing two already-parsed isometries.
fn is_empty_pose(pose: &geometry_msgs::Pose) -> bool {
    pose.position.x == 0.0
        && pose.position.y == 0.0
        && pose.position.z == 0.0
        && pose.orientation.x == 0.0
        && pose.orientation.y == 0.0
        && pose.orientation.z == 0.0
        && pose.orientation.w == 1.0
}

/// One `(shapes, shape_poses)` array pair from `CollisionObject`
/// (`primitives`/`primitive_poses`, `meshes`/`mesh_poses`,
/// `planes`/`plane_poses`), converted and reconciled to a common length.
/// Upstream `shapesAndPosesFromCollisionObjectMessage`'s `treat_shape_vectors`
/// lambda (`planning_scene.cpp:1852`).
///
/// # The asymmetric length rule
///
/// `items.len() < poses.len()` (more poses than shapes) is **rejected** --
/// upstream's separate upfront check (`:1805-1818`, one `RCLCPP_ERROR` per
/// array). `items.len() > poses.len()` (more shapes than poses) is
/// **tolerated**: missing trailing poses default to identity
/// (`:1852-1862`, "Assuming identity"). This is *not* the same rule as
/// `constraints::position`'s `BoundingVolume.primitives`/`primitive_poses`
/// check, which rejects on any length mismatch at all -- a real landmine if
/// this module had copy-pasted that convention instead of reading
/// `planning_scene.cpp` directly.
fn parallel_shapes<T>(
    items_field: &'static str,
    items: Vec<T>,
    poses_field: &'static str,
    poses: Vec<geometry_msgs::Pose>,
    convert: impl Fn(T) -> Result<Shape>,
) -> Result<Vec<(Arc<Shape>, Isometry3)>> {
    if items.len() < poses.len() {
        return Err(Error::construct(format!(
            "CollisionObject.{items_field} has length {} but {poses_field} has length {} \
             (more poses than shapes; shapesAndPosesFromCollisionObjectMessage rejects this)",
            items.len(),
            poses.len()
        )));
    }
    let mut poses = poses.into_iter();
    let mut out = Vec::with_capacity(items.len());
    for item in items {
        let shape = convert(item)?;
        let pose = match poses.next() {
            Some(p) => Isometry3::try_from(Pose(p))?,
            None => Isometry3::identity(),
        };
        out.push((Arc::new(shape), pose));
    }
    Ok(out)
}

/// Upstream `shapesAndPosesFromCollisionObjectMessage` (`planning_scene.cpp:1800`):
/// converts every geometry array (in primitive/mesh/plane order -- "order
/// matters", the same order [`apply_move`] concatenates pose arrays in) into
/// `(object_pose, shapes, shape_poses)`, where `shape_poses` are relative to
/// `object_pose`. `object_pose` here is still in the message's own `header`
/// frame -- callers compose it with the header frame's transform themselves
/// (see [`apply_add`]/`attached::shapes_from_message_geometry`), matching
/// upstream's own out-param name `header_to_pose_transform`.
///
/// # The single-shape pose swap
///
/// If there is exactly one shape total *and* `object.pose` is
/// [`is_empty_pose`], upstream promotes that one shape's own message pose to
/// be `object_pose`, and that shape's local pose becomes identity
/// (`:1823-1852`, `switch_object_pose_and_shape_pose`) -- not merely "assume
/// identity object pose", a landmine if read from the doc comment alone
/// rather than the lambda body.
pub(super) fn shapes_and_poses_from_collision_object(
    object_pose_msg: geometry_msgs::Pose,
    primitives: Vec<shape_msgs::SolidPrimitive>,
    primitive_poses: Vec<geometry_msgs::Pose>,
    meshes: Vec<shape_msgs::Mesh>,
    mesh_poses: Vec<geometry_msgs::Pose>,
    planes: Vec<shape_msgs::Plane>,
    plane_poses: Vec<geometry_msgs::Pose>,
) -> Result<(Isometry3, Vec<Arc<Shape>>, Vec<Isometry3>)> {
    let is_empty_object_pose = is_empty_pose(&object_pose_msg);

    let mut all = parallel_shapes(
        "primitives",
        primitives,
        "primitive_poses",
        primitive_poses,
        |p| Shape::try_from(SolidPrimitiveMsg(p)),
    )?;
    all.extend(parallel_shapes(
        "meshes",
        meshes,
        "mesh_poses",
        mesh_poses,
        |m| Shape::try_from(MeshMsg(m)),
    )?);
    all.extend(parallel_shapes(
        "planes",
        planes,
        "plane_poses",
        plane_poses,
        |p| Plane::try_from(PlaneMsg(p)).map(Shape::Plane),
    )?);

    let (shapes, mut shape_poses): (Vec<Arc<Shape>>, Vec<Isometry3>) = all.into_iter().unzip();

    let object_pose = if shapes.len() == 1 && is_empty_object_pose {
        let promoted = shape_poses[0];
        shape_poses[0] = Isometry3::identity();
        promoted
    } else {
        Isometry3::try_from(Pose(object_pose_msg))?
    };

    Ok((object_pose, shapes, shape_poses))
}

/// `subframe_names[]`/`subframe_poses[]`, exact-length only. Upstream
/// indexes `subframe_names[i]` for `i` in `0..subframe_poses.size()` with
/// **no length check at all**, for both world objects
/// (`processCollisionObjectAdd:1921/1924`) and attached ones
/// (`processAttachedCollisionObjectMsg:1612/1615`) -- a real out-of-bounds
/// read if `subframe_names` is shorter. This port rejects the mismatch
/// instead of reproducing that read, the same choice already made for
/// `shape_msgs::MeshTriangle.vertex_indices`/`Plane.coef` in
/// `crate::scene::shapes`. Shared by `crate::scene::attached`, since the
/// wire shape and the bug it does not reproduce are identical there.
pub(super) fn subframes_from_parallel_arrays(
    upstream_site: &'static str,
    subframe_names: Vec<String>,
    subframe_poses: Vec<geometry_msgs::Pose>,
) -> Result<BTreeMap<String, Isometry3>> {
    if subframe_names.len() != subframe_poses.len() {
        return Err(Error::construct(format!(
            "subframe_names has length {} but subframe_poses has length {} (upstream indexes \
             these without any length check at all -- {upstream_site} -- this port rejects the \
             mismatch instead of reproducing that out-of-bounds read)",
            subframe_names.len(),
            subframe_poses.len()
        )));
    }
    let mut map = BTreeMap::new();
    for (name, pose) in subframe_names.into_iter().zip(subframe_poses) {
        map.insert(name, Isometry3::try_from(Pose(pose))?);
    }
    Ok(map)
}

/// `CollisionObject.subframe_names`/`.subframe_poses` on a **world** object
/// (as opposed to an attached one, which `attach_new` already supports
/// directly via its own `subframes` parameter). Upstream
/// `World::setSubframesOfObject`, reached through
/// [`PlanningScene::set_subframes_of_object`] (`scene.rs:1078`, landed p1-fixtures
/// round 23, `de8886a`).
///
/// # Closed: no outcome enum needed, no scene-level side effect to reproduce
///
/// `set_subframes_of_object` returns a plain `bool`, not a `MoveObjectOutcome`-style
/// enum, because p1-fixtures read `World::setSubframesOfObject`'s body
/// (`world.cpp:365-378`) and found every failure mode collapses to one case --
/// unlike `moveObject`, there is no "found but unchanged" branch to
/// distinguish from "not found". And unlike [`PlanningScene::remove_object`],
/// there is no ACM/color/type bookkeeping to replay here: none of
/// `setSubframesOfObject`'s five call sites (`planning_scene.cpp:393, 1201,
/// 1743, 1927`, plus scene-file loading) touch those as a *consequence* of
/// the subframe assignment itself.
fn set_world_object_subframes(
    scene: &mut PlanningScene<'_>,
    id: &str,
    subframes: BTreeMap<String, Isometry3>,
) -> Result<()> {
    if scene.set_subframes_of_object(id, subframes) {
        Ok(())
    } else {
        Err(Error::other(format!(
            "tried to set subframes on world object '{id}', but it does not exist in this scene"
        )))
    }
}

/// `CollisionObject` MOVE's per-shape repose. Upstream `World::moveShapesInObject`,
/// reached through [`PlanningScene::move_shapes_in_object`] (`scene.rs:1055`,
/// landed p1-fixtures round 23, `de8886a`) -- same closed-gap reasoning as
/// [`set_world_object_subframes`]: `world.cpp:262-278` collapses every
/// failure to one case (unknown id or a shape-count mismatch, both already
/// caller-checked before this function runs), and `processCollisionObjectMove`
/// (`planning_scene.cpp:2004`) is the only call site, with no side effect
/// beyond the raw world mutation.
fn move_world_object_shapes(
    scene: &mut PlanningScene<'_>,
    id: &str,
    shape_poses: Vec<Isometry3>,
) -> Result<()> {
    if scene.move_shapes_in_object(id, &shape_poses) {
        Ok(())
    } else {
        Err(Error::other(format!(
            "tried to move the shapes of world object '{id}', but it does not exist in this \
             scene or its shape count does not match"
        )))
    }
}

/// Apply one `CollisionObject` command to `scene`'s world. Upstream
/// `processCollisionObjectMsg` (`planning_scene.cpp:1774`).
pub fn apply_collision_object(
    scene: &mut PlanningScene<'_>,
    msg: moveit_msgs::CollisionObject,
) -> Result<()> {
    if msg.id == OCTOMAP_NS {
        return Err(Error::other(format!(
            "the ID '{OCTOMAP_NS}' cannot be used for collision objects (name reserved)"
        )));
    }
    match CollisionObjectOperation::try_from(msg.operation)? {
        CollisionObjectOperation::Add => apply_add(scene, msg, true),
        CollisionObjectOperation::Append => apply_add(scene, msg, false),
        CollisionObjectOperation::Remove => apply_remove(scene, &msg.id),
        CollisionObjectOperation::Move => apply_move(scene, msg),
    }
}

/// ADD/APPEND, both funnel through here -- upstream `processCollisionObjectAdd`
/// (`planning_scene.cpp:1887`) handles both operations in one function,
/// differing only in whether an existing object is removed first.
fn apply_add(
    scene: &mut PlanningScene<'_>,
    msg: moveit_msgs::CollisionObject,
    replace_if_exists: bool,
) -> Result<()> {
    let moveit_msgs::CollisionObject {
        header,
        pose,
        id,
        type_: _, // D1 (object_recognition_msgs::msg::ObjectType); moveit-scene has no
        // object-type map to receive this either (its own `hasObjectType`/etc
        // bullets are D1), so there is nothing here to lose that a later
        // consumer would recover.
        primitives,
        primitive_poses,
        meshes,
        mesh_poses,
        planes,
        plane_poses,
        subframe_names,
        subframe_poses,
        ..
    } = msg;

    if primitives.is_empty() && meshes.is_empty() && planes.is_empty() {
        return Err(Error::other(
            "there are no shapes specified in the collision object message (processCollisionObjectAdd)",
        ));
    }
    // Length-validated up front, same as upstream's ordering (everything
    // about the message is checked before the world is touched); the actual
    // `set_world_object_subframes` call happens after the object exists
    // below, matching upstream's own "add shapes, then add subframes" order
    // (`processCollisionObjectAdd:1913-1927`).
    let subframes = subframes_from_parallel_arrays(
        "processCollisionObjectAdd:1921/1924",
        subframe_names,
        subframe_poses,
    )?;

    // Resolved before any mutation -- upstream checks `knowsFrameTransform`
    // before touching the world at all (`:1889`). Deliberately calls
    // `frame_transform` directly, not `header_frame_transform`: upstream's
    // own guard already rejects an empty `header.frame_id` here (an empty
    // string resolves no link/attached-body/world-transform tier, so
    // `knowsFrameTransform("")` is false), matching `frame_transform`'s
    // `Err` exactly -- unlike `apply_move`/`shapes_from_message_geometry`/
    // `apply_octomap`, upstream's ADD path has no silent-identity-on-empty
    // behavior to preserve here (PORTING-PLAN.md §183.3).
    let header_transform = scene.frame_transform(&header.frame_id)?;

    if replace_if_exists && scene.world().has_object(&id) {
        scene.remove_object(&id);
    }
    let creating_fresh = replace_if_exists || !scene.world().has_object(&id);

    let (local_object_pose, shapes, shape_poses) = shapes_and_poses_from_collision_object(
        pose,
        primitives,
        primitive_poses,
        meshes,
        mesh_poses,
        planes,
        plane_poses,
    )?;

    for (shape, shape_pose) in shapes.into_iter().zip(shape_poses) {
        // `PlanningScene::add_shape` always creates at an identity object
        // pose and discards the pose argument on an already-existing object
        // (`World::add_to_object`'s own deviation 9) -- exactly APPEND's
        // "existing object keeps its pose" semantics, and exactly what a
        // fresh ADD/APPEND-creates-new-object needs before the pose is set
        // below.
        scene.add_shape(&id, shape, shape_pose);
    }

    if creating_fresh {
        // The object was just created at identity pose above; composing
        // `header_transform * local_object_pose` with `move_object`
        // (`new_pose = transform * old_pose`, `old_pose = identity`) sets it
        // directly to the desired absolute pose -- see this module's own
        // doc for the general "delta from a known old pose" trick this
        // reproduces, forced by `PlanningScene` having no
        // add-with-a-real-object-pose entry point (only `World` does).
        scene.move_object(&id, header_transform * local_object_pose);
    }

    // Upstream calls `world_->setSubframesOfObject` unconditionally, even
    // with an empty map (`:1927`) -- on APPEND, that wholesale-replaces any
    // existing subframes with nothing whenever the message doesn't carry
    // its own. Now that the scene-level setter exists (§150.1 closed), this
    // is called unconditionally too, matching upstream exactly instead of
    // the interim "only call when there is data" behavior this port used
    // while the setter was missing.
    set_world_object_subframes(scene, &id, subframes)?;

    Ok(())
}

/// REMOVE. Upstream `processCollisionObjectRemove` (`planning_scene.cpp:1931`).
///
/// An empty `id` means "remove every object", **not** "remove the object
/// named the empty string" -- a real landmine if read from the field name
/// alone.
fn apply_remove(scene: &mut PlanningScene<'_>, id: &str) -> Result<()> {
    if id.is_empty() {
        scene.remove_all_objects();
        return Ok(());
    }
    if scene.remove_object(id) {
        Ok(())
    } else {
        Err(Error::other(format!(
            "tried to remove world object '{id}', but it does not exist in this scene"
        )))
    }
}

/// MOVE. Upstream `processCollisionObjectMove` (`planning_scene.cpp:1953`).
///
/// The object's absolute pose is **always** applied (unconditionally, even
/// if the shape-repose step below then fails) -- upstream itself has this
/// exact partial-effect shape: `setObjectPose` runs before the shape-count
/// check, with no rollback on that check's failure.
///
/// `header.frame_id` is resolved via [`super::header_frame_transform`], not
/// [`PlanningScene::frame_transform`] directly: unlike `apply_add`'s
/// `knowsFrameTransform`-guarded call below, upstream's
/// `getFrameTransform(object.header.frame_id)` here (`:1964`) has no guard
/// in front of it, so an empty `header.frame_id` resolves to identity
/// through `getFrameTransform`'s own silent fallback rather than being
/// rejected (PORTING-PLAN.md §183).
fn apply_move(scene: &mut PlanningScene<'_>, msg: moveit_msgs::CollisionObject) -> Result<()> {
    let moveit_msgs::CollisionObject {
        header,
        pose,
        id,
        primitives,
        primitive_poses,
        meshes,
        mesh_poses,
        planes,
        plane_poses,
        ..
    } = msg;

    if !scene.world().has_object(&id) {
        return Err(Error::other(format!("'{id}' does not exist. Cannot move.")));
    }

    // Geometry is ignored on MOVE (upstream logs a warning and proceeds,
    // `:1958-1962`); this crate has no logging framework wired up (see
    // `moveit_geometry::Plane::scale_and_padd`'s own no-op precedent), so it
    // is silently ignored the same way.
    let _ = (primitives, meshes, planes);

    let header_transform = header_frame_transform(scene, &header.frame_id)?;
    let new_object_pose = header_transform * Isometry3::try_from(Pose(pose))?;
    let old_pose = scene
        .world()
        .get_object(&id)
        .expect("has_object just confirmed this id exists")
        .pose();
    scene.move_object(&id, new_object_pose * old_pose.inverse());

    let mut shape_poses_msgs = primitive_poses;
    shape_poses_msgs.extend(mesh_poses);
    shape_poses_msgs.extend(plane_poses);
    if shape_poses_msgs.is_empty() {
        return Ok(());
    }

    let current_shape_count = scene
        .world()
        .get_object(&id)
        .expect("has_object just confirmed this id exists")
        .shapes()
        .len();
    if shape_poses_msgs.len() != current_shape_count {
        return Err(Error::other(format!(
            "move operation for object '{id}' must have same number of geometry poses \
             ({} supplied, {current_shape_count} shape(s) exist). Cannot move. \
             (the object's pose was still updated above, matching upstream's own \
             partial-effect behavior on this path)",
            shape_poses_msgs.len()
        )));
    }

    // Parsed to `Isometry3` before the repose call, matching upstream's own
    // ordering (`poseMsgToEigen` runs before `moveShapesInObject`,
    // `:1985-1998`) -- a malformed pose here is rejected independent of
    // whether the repose call below itself would have succeeded.
    let shape_poses = shape_poses_msgs
        .into_iter()
        .map(|p| Isometry3::try_from(Pose(p)))
        .collect::<Result<Vec<_>>>()?;
    move_world_object_shapes(scene, &id, shape_poses)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::tests::one_joint_model;
    use moveit_srdf::SrdfModel;

    /// Asserts the call was rejected *for the reason named*, not merely
    /// that it was rejected. `apply_move` has two independent `Error::Other`
    /// sites (unknown object id, mismatched shape-pose count) --
    /// `matches!(err, Error::Other(_))` alone cannot tell a test that a
    /// routing bug swapped which branch fired (same shape as
    /// `moveit-constraints`' `e3b40c6`).
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

    fn scene(model: &moveit_model::RobotModel) -> PlanningScene<'_> {
        let srdf =
            SrdfModel::parse_str("<?xml version=\"1.0\"?><robot name=\"one_joint\"></robot>")
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

    fn posed(x: f64, y: f64, z: f64) -> geometry_msgs::Pose {
        geometry_msgs::Pose {
            position: geometry_msgs::Point { x, y, z },
            orientation: geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        }
    }

    /// Same shape as `posed`, but with a non-unit (norm 2.0) `w` -- the
    /// value PORTING-PLAN.md §215's per-site table exercises at every
    /// `Isometry3::try_from(Pose(...))` call site in this file.
    fn posed_norm2(x: f64, y: f64, z: f64) -> geometry_msgs::Pose {
        geometry_msgs::Pose {
            position: geometry_msgs::Point { x, y, z },
            orientation: geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 2.0,
            },
        }
    }

    fn sphere_primitive(radius: f64) -> shape_msgs::SolidPrimitive {
        shape_msgs::SolidPrimitive {
            type_: 2, // SPHERE
            dimensions: vec![radius],
            polygon: Default::default(),
        }
    }

    fn base_object(id: &str, model_frame: &str, operation: u8) -> moveit_msgs::CollisionObject {
        moveit_msgs::CollisionObject {
            header: r2r::std_msgs::msg::Header {
                frame_id: model_frame.to_string(),
                ..Default::default()
            },
            pose: identity_pose(),
            id: id.to_string(),
            type_: Default::default(),
            primitives: vec![sphere_primitive(0.1)],
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

    #[test]
    fn octomap_ns_is_rejected_for_every_operation() {
        let model = one_joint_model();
        for op in [ADD, REMOVE, APPEND, MOVE] {
            let mut sc = scene(&model);
            let msg = base_object(OCTOMAP_NS, model.model_frame(), op);
            let err = apply_collision_object(&mut sc, msg).unwrap_err();
            // Not just the variant: `apply_add`'s own no-shapes check (hit by
            // `add_with_no_geometry_is_rejected` below) is a sibling
            // Error::Other site.
            assert!(
                err.to_string().contains("name reserved"),
                "op={op}, got: {err:?}"
            );
        }
    }

    #[test]
    fn add_creates_object_with_shape_and_pose() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut msg = base_object("box", model.model_frame(), ADD);
        msg.pose = posed(1.0, 2.0, 3.0);
        apply_collision_object(&mut sc, msg).unwrap();
        let obj = sc.world().get_object("box").unwrap();
        assert_eq!(obj.pose(), Isometry3::translation(1.0, 2.0, 3.0));
        assert_eq!(obj.shapes().len(), 1);
    }

    #[test]
    fn add_replaces_existing_object() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        apply_collision_object(&mut sc, base_object("box", model.model_frame(), ADD)).unwrap();
        let mut second = base_object("box", model.model_frame(), ADD);
        second.primitives = vec![sphere_primitive(0.2), sphere_primitive(0.3)];
        second.primitive_poses = vec![identity_pose(), identity_pose()];
        apply_collision_object(&mut sc, second).unwrap();
        assert_eq!(sc.world().get_object("box").unwrap().shapes().len(), 2);
    }

    #[test]
    fn append_onto_existing_object_keeps_old_pose_and_adds_shapes() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut first = base_object("box", model.model_frame(), ADD);
        first.pose = posed(1.0, 0.0, 0.0);
        apply_collision_object(&mut sc, first).unwrap();

        let mut second = base_object("box", model.model_frame(), APPEND);
        // A real, non-identity pose here must be ignored -- APPEND onto an
        // existing object never repositions it (World::add_to_object,
        // deviation 9).
        second.pose = posed(99.0, 99.0, 99.0);
        apply_collision_object(&mut sc, second).unwrap();

        let obj = sc.world().get_object("box").unwrap();
        assert_eq!(obj.pose(), Isometry3::translation(1.0, 0.0, 0.0));
        assert_eq!(obj.shapes().len(), 2);
    }

    #[test]
    fn append_onto_nonexistent_object_creates_it() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut msg = base_object("box", model.model_frame(), APPEND);
        msg.pose = posed(1.0, 0.0, 0.0);
        apply_collision_object(&mut sc, msg).unwrap();
        assert_eq!(
            sc.world().get_object("box").unwrap().pose(),
            Isometry3::translation(1.0, 0.0, 0.0)
        );
    }

    #[test]
    fn add_with_no_geometry_is_rejected() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut msg = base_object("box", model.model_frame(), ADD);
        msg.primitives = vec![];
        msg.primitive_poses = vec![];
        let err = apply_collision_object(&mut sc, msg).unwrap_err();
        // Not just the variant: `apply_collision_object`'s own OCTOMAP_NS
        // check (hit by `octomap_ns_is_rejected_for_every_operation` above)
        // is a sibling Error::Other site.
        assert!(
            err.to_string().contains("no shapes specified"),
            "got: {err:?}"
        );
    }

    // Assertion-discrimination sweep (round 8, folded-operand audit):
    // `apply_add`'s no-shapes guard is `primitives.is_empty() &&
    // meshes.is_empty() && planes.is_empty()`. Before this round no test
    // ever populated `meshes` or `planes` -- `add_with_no_geometry_is_rejected`
    // above only ever sees them empty, so a guard weakened to check
    // `primitives` alone would still pass every existing test. Isolating
    // mutation (drop one operand's `is_empty()` conjunct from the `&&`
    // chain, keep the fixture unchanged): the two tests below fail only
    // when their own operand's conjunct is the one dropped, because
    // dropping a conjunct makes the guard fire (reject) in *more* cases,
    // not fewer -- an object with only that shape kind starts being
    // rejected.
    #[test]
    fn add_with_only_a_mesh_is_accepted() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut msg = base_object("box", model.model_frame(), ADD);
        msg.primitives = vec![];
        msg.primitive_poses = vec![];
        msg.meshes = vec![shape_msgs::Mesh {
            triangles: vec![],
            vertices: vec![],
        }];
        msg.mesh_poses = vec![identity_pose()];
        apply_collision_object(&mut sc, msg).unwrap();
        assert!(sc.world().has_object("box"));
    }

    #[test]
    fn add_with_only_a_plane_is_accepted() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut msg = base_object("box", model.model_frame(), ADD);
        msg.primitives = vec![];
        msg.primitive_poses = vec![];
        msg.planes = vec![shape_msgs::Plane {
            coef: vec![0.0, 0.0, 1.0, 0.0],
        }];
        msg.plane_poses = vec![identity_pose()];
        apply_collision_object(&mut sc, msg).unwrap();
        assert!(sc.world().has_object("box"));
    }

    #[test]
    fn more_poses_than_primitives_is_rejected() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut msg = base_object("box", model.model_frame(), ADD);
        msg.primitive_poses.push(identity_pose());
        let err = apply_collision_object(&mut sc, msg).unwrap_err();
        // Not just the variant: `subframes_from_parallel_arrays` (hit by
        // `add_with_mismatched_subframe_array_lengths_is_rejected` below) and
        // the generic Pose conversion (hit by
        // `move_shape_repose_with_malformed_pose_is_rejected` below) are
        // sibling Error::Construct sites.
        assert!(
            err.to_string()
                .contains("CollisionObject.primitives has length"),
            "got: {err:?}"
        );
    }

    #[test]
    fn more_primitives_than_poses_defaults_missing_to_identity() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut msg = base_object("box", model.model_frame(), ADD);
        msg.primitives.push(sphere_primitive(0.2));
        // primitive_poses still has only one entry.
        apply_collision_object(&mut sc, msg).unwrap();
        let obj = sc.world().get_object("box").unwrap();
        assert_eq!(obj.shapes().len(), 2);
        assert_eq!(obj.shapes()[1].pose(), Isometry3::identity());
    }

    #[test]
    fn single_shape_with_empty_object_pose_promotes_shape_pose_to_object_pose() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut msg = base_object("box", model.model_frame(), ADD);
        msg.pose = identity_pose(); // empty
        msg.primitive_poses = vec![posed(1.0, 2.0, 3.0)]; // the one shape's own pose
        apply_collision_object(&mut sc, msg).unwrap();
        let obj = sc.world().get_object("box").unwrap();
        assert_eq!(obj.pose(), Isometry3::translation(1.0, 2.0, 3.0));
        assert_eq!(obj.shapes()[0].pose(), Isometry3::identity());
    }

    #[test]
    fn two_shapes_with_empty_object_pose_does_not_swap() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut msg = base_object("box", model.model_frame(), ADD);
        msg.pose = identity_pose();
        msg.primitives = vec![sphere_primitive(0.1), sphere_primitive(0.1)];
        msg.primitive_poses = vec![posed(1.0, 0.0, 0.0), posed(0.0, 1.0, 0.0)];
        apply_collision_object(&mut sc, msg).unwrap();
        let obj = sc.world().get_object("box").unwrap();
        assert_eq!(obj.pose(), Isometry3::identity());
        assert_eq!(
            obj.shapes()[0].pose(),
            Isometry3::translation(1.0, 0.0, 0.0)
        );
    }

    #[test]
    fn add_with_norm_2_orientation_on_object_and_shape_poses_succeeds_and_normalizes() {
        // PORTING-PLAN.md §215's per-site table: `object_pose` at :207 (two
        // shapes, so no single-shape pose-swap) and the shape pose at :142
        // share the generic Pose rule with the other eight sites -- run
        // through this call chain rather than the bare conversion alone.
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut msg = base_object("box", model.model_frame(), ADD);
        msg.pose = posed_norm2(1.0, 0.0, 0.0);
        msg.primitives = vec![sphere_primitive(0.1), sphere_primitive(0.1)];
        msg.primitive_poses = vec![posed_norm2(0.0, 1.0, 0.0), identity_pose()];
        apply_collision_object(&mut sc, msg).unwrap();
        let obj = sc.world().get_object("box").unwrap();
        let object_norm = obj.pose().rotation.into_inner().norm();
        let shape_norm = obj.shapes()[0].pose().rotation.into_inner().norm();
        assert!((object_norm - 1.0).abs() < 1e-12, "got: {object_norm}");
        assert!((shape_norm - 1.0).abs() < 1e-12, "got: {shape_norm}");
    }

    #[test]
    fn add_with_norm_2_orientation_on_subframe_pose_succeeds_and_normalizes() {
        // PORTING-PLAN.md §215's per-site table: `subframes_from_parallel_arrays`'s
        // `Isometry3::try_from(Pose(pose))` at :239.
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut msg = base_object("box", model.model_frame(), ADD);
        msg.subframe_names = vec!["tip".to_string()];
        msg.subframe_poses = vec![posed_norm2(1.0, 2.0, 3.0)];
        apply_collision_object(&mut sc, msg).unwrap();
        let obj = sc.world().get_object("box").unwrap();
        let norm = obj
            .subframe_pose("tip")
            .unwrap()
            .rotation
            .into_inner()
            .norm();
        assert!((norm - 1.0).abs() < 1e-12, "got: {norm}");
    }

    #[test]
    fn remove_specific_id() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        apply_collision_object(&mut sc, base_object("box", model.model_frame(), ADD)).unwrap();
        apply_collision_object(&mut sc, base_object("box", model.model_frame(), REMOVE)).unwrap();
        assert!(!sc.world().has_object("box"));
    }

    #[test]
    fn remove_unknown_id_is_rejected() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        let err = apply_collision_object(&mut sc, base_object("box", model.model_frame(), REMOVE))
            .unwrap_err();
        // Not just the variant: `apply_collision_object`'s own OCTOMAP_NS
        // check, `apply_add`'s own no-shapes check, and `apply_move`'s two
        // sites are sibling Error::Other sites reachable through the same
        // entry point via other operations/inputs.
        assert!(
            err.to_string().contains("tried to remove world object"),
            "got: {err:?}"
        );
    }

    #[test]
    fn remove_empty_id_removes_everything() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        apply_collision_object(&mut sc, base_object("a", model.model_frame(), ADD)).unwrap();
        apply_collision_object(&mut sc, base_object("b", model.model_frame(), ADD)).unwrap();
        apply_collision_object(&mut sc, base_object("", model.model_frame(), REMOVE)).unwrap();
        assert!(sc.world().is_empty());
    }

    #[test]
    fn move_requires_existing_object() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        assert_err_mentions(
            apply_collision_object(&mut sc, base_object("box", model.model_frame(), MOVE)),
            "does not exist. Cannot move",
        );
    }

    #[test]
    fn move_sets_absolute_pose_and_ignores_geometry() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        apply_collision_object(&mut sc, base_object("box", model.model_frame(), ADD)).unwrap();
        let mut mv = base_object("box", model.model_frame(), MOVE);
        mv.pose = posed(5.0, 0.0, 0.0);
        mv.primitive_poses = vec![]; // no shape repose requested
        apply_collision_object(&mut sc, mv).unwrap();
        let obj = sc.world().get_object("box").unwrap();
        assert_eq!(obj.pose(), Isometry3::translation(5.0, 0.0, 0.0));
        assert_eq!(obj.shapes().len(), 1, "MOVE must not touch geometry");
    }

    /// An empty `header.frame_id` on MOVE is accepted as the world frame
    /// (PORTING-PLAN.md §183) -- `processCollisionObjectMove` has no
    /// `knowsFrameTransform` guard before `getFrameTransform`, unlike ADD's
    /// `:1889`. Before `header_frame_transform` existed, this would have
    /// been rejected with `Err(UnknownName)`.
    #[test]
    fn move_with_empty_header_frame_id_is_accepted_as_the_world_frame() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        apply_collision_object(&mut sc, base_object("box", model.model_frame(), ADD)).unwrap();
        let mut mv = base_object("box", "", MOVE);
        mv.pose = posed(5.0, 0.0, 0.0);
        mv.primitive_poses = vec![];
        apply_collision_object(&mut sc, mv).unwrap();
        assert_eq!(
            sc.world().get_object("box").unwrap().pose(),
            Isometry3::translation(5.0, 0.0, 0.0)
        );
    }

    #[test]
    fn move_with_mismatched_pose_count_is_rejected_but_still_moves_the_object() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        apply_collision_object(&mut sc, base_object("box", model.model_frame(), ADD)).unwrap();
        let mut mv = base_object("box", model.model_frame(), MOVE);
        mv.pose = posed(5.0, 0.0, 0.0);
        mv.primitive_poses = vec![identity_pose(), identity_pose()]; // object has 1 shape
        assert_err_mentions(
            apply_collision_object(&mut sc, mv),
            "must have same number of geometry poses",
        );
        // Upstream's own partial-effect: the pose move already happened.
        assert_eq!(
            sc.world().get_object("box").unwrap().pose(),
            Isometry3::translation(5.0, 0.0, 0.0)
        );
    }

    #[test]
    fn move_shape_repose_with_matching_count_reposes_shapes() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        apply_collision_object(&mut sc, base_object("box", model.model_frame(), ADD)).unwrap();
        let mut mv = base_object("box", model.model_frame(), MOVE);
        mv.primitive_poses = vec![posed(0.0, 0.0, 1.0)]; // matches the 1 existing shape
        apply_collision_object(&mut sc, mv).unwrap();
        let obj = sc.world().get_object("box").unwrap();
        assert_eq!(
            obj.shapes()[0].pose(),
            Isometry3::translation(0.0, 0.0, 1.0)
        );
    }

    #[test]
    fn move_with_norm_2_orientation_on_object_and_shape_poses_succeeds_and_normalizes() {
        // PORTING-PLAN.md §215's per-site table: `apply_move`'s
        // `new_object_pose` conversion at :478 and the shape-repose
        // conversion at :515.
        let model = one_joint_model();
        let mut sc = scene(&model);
        apply_collision_object(&mut sc, base_object("box", model.model_frame(), ADD)).unwrap();
        let mut mv = base_object("box", model.model_frame(), MOVE);
        mv.pose = posed_norm2(5.0, 0.0, 0.0);
        mv.primitive_poses = vec![posed_norm2(0.0, 0.0, 1.0)]; // matches the 1 existing shape
        apply_collision_object(&mut sc, mv).unwrap();
        let obj = sc.world().get_object("box").unwrap();
        let object_norm = obj.pose().rotation.into_inner().norm();
        let shape_norm = obj.shapes()[0].pose().rotation.into_inner().norm();
        assert!((object_norm - 1.0).abs() < 1e-12, "got: {object_norm}");
        assert!((shape_norm - 1.0).abs() < 1e-12, "got: {shape_norm}");
    }

    #[test]
    fn add_with_subframes_sets_them_on_the_world_object() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut msg = base_object("box", model.model_frame(), ADD);
        msg.subframe_names = vec!["tip".to_string()];
        msg.subframe_poses = vec![posed(1.0, 2.0, 3.0)];
        apply_collision_object(&mut sc, msg).unwrap();
        let obj = sc.world().get_object("box").unwrap();
        assert_eq!(
            obj.subframe_pose("tip").unwrap(),
            Isometry3::translation(1.0, 2.0, 3.0)
        );
    }

    #[test]
    fn append_without_subframe_data_clears_existing_subframes() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut first = base_object("box", model.model_frame(), ADD);
        first.subframe_names = vec!["tip".to_string()];
        first.subframe_poses = vec![posed(1.0, 2.0, 3.0)];
        apply_collision_object(&mut sc, first).unwrap();
        assert!(
            sc.world()
                .get_object("box")
                .unwrap()
                .subframe_pose("tip")
                .is_some()
        );

        let second = base_object("box", model.model_frame(), APPEND); // no subframe_names/poses
        apply_collision_object(&mut sc, second).unwrap();
        assert!(
            sc.world()
                .get_object("box")
                .unwrap()
                .subframe_pose("tip")
                .is_none(),
            "upstream calls setSubframesOfObject unconditionally, even with an empty map \
             (planning_scene.cpp:1927) -- APPEND without subframe data must clear old ones"
        );
    }

    #[test]
    fn add_with_mismatched_subframe_array_lengths_is_rejected() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        let mut msg = base_object("box", model.model_frame(), ADD);
        msg.subframe_names = vec!["a".to_string(), "b".to_string()];
        msg.subframe_poses = vec![identity_pose()];
        let err = apply_collision_object(&mut sc, msg).unwrap_err();
        // Not just the variant: `parallel_shapes` (hit by
        // `more_poses_than_primitives_is_rejected` above) and the generic
        // Pose conversion (hit by
        // `move_shape_repose_with_malformed_pose_is_rejected` below) are
        // sibling Error::Construct sites.
        assert!(
            err.to_string().contains("subframe_names has length"),
            "got: {err:?}"
        );
    }

    #[test]
    fn add_without_subframes_succeeds() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        apply_collision_object(&mut sc, base_object("box", model.model_frame(), ADD)).unwrap();
        assert!(sc.world().has_object("box"));
    }

    // Assertion-discrimination sweep (round 9, `scene/collision_object.rs
    // :1089` follow-up): `apply_move` parses a `Pose` at two independent
    // call sites -- `:478`'s object-pose parse and `:515`'s shape-repose
    // parse -- both delegating to the same generic Pose rule and sharing
    // this exact "too close to zero..." text. Before this round no fixture
    // in this file ever gave `:478` a malformed `mv.pose`; every MOVE test
    // used `posed(..)`/`identity_pose()` there, so `:478`'s own error path
    // was *unreached*, not merely untested -- the flagged risk was live,
    // not hypothetical. The two tests below isolate each site: `:478`'s
    // `?` short-circuits before `:515` is ever constructed, so a malformed
    // `mv.pose` structurally cannot also exercise the shape-repose parse,
    // and vice versa (a malformed `primitive_poses` entry is parsed only
    // after `:478` already returned `Ok`). Bite-checked under docker:
    // neutralizing `:478` alone (forcing it to fall back to identity
    // instead of propagating) makes only `move_object_pose_with_
    // malformed_pose_is_rejected` fail (wrongly succeeds); neutralizing
    // `:515` alone makes only `move_shape_repose_with_malformed_pose_is_
    // rejected` fail. Sharing one needle across the two is not narrowed:
    // it is the same accepted "generic Pose rule, one message, N callers"
    // pattern already used throughout this crate (see `position.rs`/
    // `orientation.rs`'s §211/§213 comments and this file's own
    // `parallel_shapes` test), and each test's fixture makes only one
    // site reachable, so the shared text does not cost either test its
    // ability to prove which physical site actually fired.
    #[test]
    fn move_object_pose_with_malformed_pose_is_rejected() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        apply_collision_object(&mut sc, base_object("box", model.model_frame(), ADD)).unwrap();
        let mut mv = base_object("box", model.model_frame(), MOVE);
        mv.pose = geometry_msgs::Pose {
            position: geometry_msgs::Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            },
        };
        let err = apply_collision_object(&mut sc, mv).unwrap_err();
        assert!(
            err.to_string()
                .contains("too close to zero (or non-finite) to have a unit-norm representative"),
            "got: {err:?}"
        );
    }

    #[test]
    fn move_shape_repose_with_malformed_pose_is_rejected() {
        let model = one_joint_model();
        let mut sc = scene(&model);
        apply_collision_object(&mut sc, base_object("box", model.model_frame(), ADD)).unwrap();
        let mut mv = base_object("box", model.model_frame(), MOVE);
        mv.primitive_poses = vec![geometry_msgs::Pose {
            position: geometry_msgs::Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            },
        }];
        let err = apply_collision_object(&mut sc, mv).unwrap_err();
        // Not just the variant: `parallel_shapes` (hit by
        // `more_poses_than_primitives_is_rejected` above) and
        // `subframes_from_parallel_arrays` (hit by
        // `add_with_mismatched_subframe_array_lengths_is_rejected` above) are
        // sibling Error::Construct sites -- and `move_object_pose_with_
        // malformed_pose_is_rejected` above shares this same needle by
        // design (see its own comment): the two are bite-checked
        // independent, not colliding.
        assert!(
            err.to_string()
                .contains("too close to zero (or non-finite) to have a unit-norm representative"),
            "got: {err:?}"
        );
    }
}
