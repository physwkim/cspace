// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/planning_scene/src/planning_scene.cpp
//     (processPlanningSceneWorldMsg:1396, processOctomapMsg(OctomapWithPose):1478,
//      usePlanningSceneMsg:1405, setPlanningSceneMsg:1367, setPlanningSceneDiffMsg:1314)

//! `moveit_msgs/msg::{PlanningScene, PlanningSceneWorld}` <->
//! [`cspace_planning::scene::PlanningScene`]. See `doc/message-mapping.md` §11.
//!
//! # Why these live here and not in `cspace_planning::scene`
//!
//! `crates/cspace-planning`'s own symbol survey marks
//! `setPlanningSceneDiffMsg`/`setPlanningSceneMsg`/`usePlanningSceneMsg` (and
//! `processPlanningSceneWorldMsg` alongside them) as **D1**
//! (`scene.rs`, the "Message conversions" list). D1 is a *placement*
//! decision, not a "never port this" one -- PORTING-PLAN.md §0 spells it out:
//! "코어 크레이트는 ROS 타입을 일절 참조하지 않는다 ... ROS 2 연동은
//! 선택적 `cspace-ros` 크레이트 하나에만 존재하며, 그 크레이트만 r2r에
//! 의존하고 `From`/`Into` 변환을 제공한다". Every one of those four takes a
//! `moveit_msgs::msg::PlanningScene[World]`, so D1 puts them in *this*
//! crate. [`apply_planning_scene_world`] is the precedent already in the
//! tree: D1-marked in `scene.rs`, ported here.
//!
//! # The `is_diff` flag is consumed once, at the boundary
//!
//! Upstream carries `scene_msg.is_diff` all the way in and re-reads it at
//! three separate points -- `usePlanningSceneMsg` branches on it
//! (`planning_scene.cpp:1407`), `setPlanningSceneMsg` re-asserts it is false
//! (`:1369`), and `PlanningSceneMonitor::newPlanningSceneMessage` reads
//! `scene.is_diff` three more times and `scene.robot_state.is_diff` twice to
//! pick the full/diff arm and to classify the update type
//! (`planning_scene_monitor.cpp:759,778,795` and `:762,812`). That is a
//! boolean travelling
//! beside the value it qualifies, and every one of those re-reads is a place
//! the two can disagree.
//!
//! This port reads it exactly once, in [`PlanningSceneUpdate::from`], which
//! consumes the message into one of two wrappers. [`FullPlanningSceneMsg`]
//! and [`PlanningSceneDiffMsg`] have private fields and no other
//! constructor, so [`set_planning_scene_msg`] cannot be handed a message
//! whose `is_diff` is `true` and [`set_planning_scene_diff_msg`] cannot be
//! handed one whose `is_diff` is `false` -- upstream's `assert(scene_msg
//! .is_diff == false)` becomes a fact about the type rather than a runtime
//! check that only fires in a debug build.
//!
//! # Fields with no core representation
//!
//! `link_padding`, `link_scale` and `object_colors` are **rejected** when
//! non-empty rather than dropped, per D6 -- the private
//! `reject_unrepresentable_fields` records what each one would need (plain
//! backticks, not an intra-doc link: this is a public module doc and the
//! item is private, so a link here fails `cargo doc`). An empty
//! array is not a rejection: upstream's own `CollisionEnv::setPadding` over
//! an empty vector is a no-op, so "absent" and "stated as empty" mean the
//! same thing on the wire and here.

use std::sync::Arc;

use cspace_collision::{AllowedCollisionMatrix, AllowedCollisionType};
use cspace_core::error::{Error, Result};
use cspace_core::geometry::{Isometry3, OcTree as OcTreeShape, Shape, Transforms};
use cspace_core::state::RobotState as CoreRobotState;
use cspace_planning::scene::PlanningScene;
use r2r::moveit_msgs::msg as moveit_msgs;

use super::attached::apply_attached_collision_object;
use super::collision_object::{CollisionObjectOperation, OCTOMAP_NS, apply_collision_object};
use super::header_frame_transform;
use crate::geometry::{Pose, Transform};
use crate::state::RobotStateMsg;

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

/// A `moveit_msgs/PlanningScene` whose `is_diff` is **false**: a complete
/// scene that replaces whatever the target scene held.
///
/// Constructible only through [`PlanningSceneUpdate`] -- see the module doc.
#[derive(Debug, Clone)]
pub struct FullPlanningSceneMsg(moveit_msgs::PlanningScene);

/// A `moveit_msgs/PlanningScene` whose `is_diff` is **true**: a set of
/// changes layered onto whatever the target scene already holds.
///
/// Constructible only through [`PlanningSceneUpdate`] -- see the module doc.
#[derive(Debug, Clone)]
pub struct PlanningSceneDiffMsg(moveit_msgs::PlanningScene);

/// The two things a `moveit_msgs/PlanningScene` can be, decided once from
/// its `is_diff` flag.
///
/// This is the only place in this crate that reads `is_diff`; the module doc
/// explains why that matters.
#[derive(Debug, Clone)]
pub enum PlanningSceneUpdate {
    /// `is_diff == false`.
    Full(FullPlanningSceneMsg),
    /// `is_diff == true`.
    Diff(PlanningSceneDiffMsg),
}

impl From<moveit_msgs::PlanningScene> for PlanningSceneUpdate {
    /// Total and infallible: `is_diff` is a `bool`, so both branches are
    /// reachable and neither can fail. The wrappers' fields are private and
    /// this is their only construction site, which is what makes
    /// "`FullPlanningSceneMsg` implies `!is_diff`" hold by construction.
    fn from(msg: moveit_msgs::PlanningScene) -> Self {
        if msg.is_diff {
            Self::Diff(PlanningSceneDiffMsg(msg))
        } else {
            Self::Full(FullPlanningSceneMsg(msg))
        }
    }
}

/// Upstream `usePlanningSceneMsg` (`planning_scene.cpp:1405`): dispatch on
/// `is_diff` to [`set_planning_scene_diff_msg`] or
/// [`set_planning_scene_msg`].
///
/// Upstream's body is `if (scene_msg.is_diff) ... else ...` reading the flag
/// off a message both arms then re-read; here the flag is already gone by
/// the time an arm runs (module doc).
pub fn use_planning_scene_msg<'m>(
    scene: &mut PlanningScene<'m>,
    msg: moveit_msgs::PlanningScene,
) -> Result<()> {
    match PlanningSceneUpdate::from(msg) {
        PlanningSceneUpdate::Full(full) => set_planning_scene_msg(scene, full),
        PlanningSceneUpdate::Diff(diff) => set_planning_scene_diff_msg(scene, diff),
    }
}

/// The three `PlanningScene` fields with no representation anywhere in this
/// workspace's core crates, rejected when non-empty rather than dropped (D6;
/// module doc).
///
/// - `link_padding`/`link_scale`: upstream stores these on the scene's
///   *collision environment* (`collision_detector_->cenv_->setPadding`,
///   `planning_scene.cpp:1348-1349` in the diff arm, `:1386-1387` in the
///   full arm). This port's
///   `cspace_planning::scene::PlanningScene` owns no collision environment at all --
///   `check_collision` takes one as an argument -- and the padding/scale
///   state lives on `cspace_collision::ParryCollisionEnv` as its
///   `LinkPaddingScale`. Expires when a scene owns its environment, or when
///   this conversion is given one to write through to.
/// - `object_colors`: `object_colors_` is not ported at all
///   (`crates/cspace-planning/src/scene/scene.rs` records that in
///   `decouple_parent`'s own doc). Expires when `cspace_planning::scene::PlanningScene`
///   gains an object-color map.
fn reject_unrepresentable_fields(msg: &moveit_msgs::PlanningScene) -> Result<()> {
    if !msg.link_padding.is_empty() {
        return Err(Error::other(format!(
            "PlanningScene.link_padding has {} entr(ies) but cspace_planning::scene::PlanningScene \
             owns no collision environment to apply them to (see \
             cspace_collision::LinkPaddingScale)",
            msg.link_padding.len()
        )));
    }
    if !msg.link_scale.is_empty() {
        return Err(Error::other(format!(
            "PlanningScene.link_scale has {} entr(ies) but cspace_planning::scene::PlanningScene \
             owns no collision environment to apply them to (see \
             cspace_collision::LinkPaddingScale)",
            msg.link_scale.len()
        )));
    }
    if !msg.object_colors.is_empty() {
        return Err(Error::other(format!(
            "PlanningScene.object_colors has {} entr(ies) but cspace_planning::scene::PlanningScene \
             has no object-color map (object_colors_ is not ported)",
            msg.object_colors.len()
        )));
    }
    Ok(())
}

/// Upstream `Transforms::setTransforms` (`transforms.cpp:172`), which is a
/// loop over `setTransform(const geometry_msgs::msg::TransformStamped&)`
/// (`:151`): each entry is keyed by `header.frame_id` and must name
/// `target_frame_` as its `child_frame_id`.
///
/// It **merges**, it does not replace -- upstream's own body has no clear,
/// so a full scene message carrying two transforms on a scene that already
/// held a third keeps all three. `cspace_core::geometry::Transforms::set_all_transforms`
/// is the replacing variant and is deliberately not what this calls.
///
/// Deviation (D6): upstream logs `RCLCPP_ERROR` and silently skips an entry
/// whose `child_frame_id` is some other frame; this returns `Err` naming it.
/// A transform the caller supplied and this port ignored is exactly the
/// silently-absorbed failure D6 exists to prevent, and unlike upstream this
/// conversion has a caller that can report it.
fn apply_fixed_frame_transforms(
    scene: &mut PlanningScene<'_>,
    transforms: Vec<r2r::geometry_msgs::msg::TransformStamped>,
) -> Result<()> {
    let target_frame = scene.planning_frame().to_string();
    for stamped in transforms {
        if !Transforms::same_frame(&stamped.child_frame_id, &target_frame) {
            return Err(Error::other(format!(
                "PlanningScene.fixed_frame_transforms entry is to frame '{}', but frame \
                 '{target_frame}' was expected",
                stamped.child_frame_id
            )));
        }
        let isometry = Isometry3::try_from(Transform(stamped.transform))?;
        scene
            .transforms_mut()
            .set_transform(isometry, &stamped.header.frame_id)?;
    }
    Ok(())
}

/// `moveit_msgs/AllowedCollisionMatrix` -> [`AllowedCollisionMatrix`]
/// (orphan-rule wrapper, see `lib.rs`).
#[derive(Debug, Clone)]
pub struct AllowedCollisionMatrixMsg(pub moveit_msgs::AllowedCollisionMatrix);

/// Upstream's `getDefaultEntry(name1, name2, type)` merge rule
/// (`collision_matrix.cpp:330-364`): `NEVER` if either side says never, else
/// `CONDITIONAL` if either side is conditional, else `ALWAYS`; `None` when
/// neither name has a default.
///
/// `cspace_collision` has this as the private `default_for_pair`; it is not
/// on the public surface and `crates/cspace-collision` is outside this
/// round's fence, so the rule is restated here rather than exported. The
/// alternative -- calling the public `allowed_collision(n1, n2)` -- would
/// also consult *explicit* entries, and is only equivalent to this because
/// of the exact order upstream's loop below fills them in. Depending on that
/// ordering is what this restatement avoids.
fn combined_default(
    acm: &AllowedCollisionMatrix,
    name1: &str,
    name2: &str,
) -> Option<AllowedCollisionType> {
    let t1 = acm.default_entry(name1).map(|e| e.kind());
    let t2 = acm.default_entry(name2).map(|e| e.kind());
    match (t1, t2) {
        (None, None) => None,
        (Some(t), None) | (None, Some(t)) => Some(t),
        (Some(a), Some(b)) => Some(
            if a == AllowedCollisionType::Never || b == AllowedCollisionType::Never {
                AllowedCollisionType::Never
            } else if a == AllowedCollisionType::Conditional
                || b == AllowedCollisionType::Conditional
            {
                AllowedCollisionType::Conditional
            } else {
                AllowedCollisionType::Always
            },
        ),
    }
}

impl TryFrom<AllowedCollisionMatrixMsg> for AllowedCollisionMatrix {
    type Error = Error;

    /// Upstream `AllowedCollisionMatrix::AllowedCollisionMatrix(const
    /// moveit_msgs::msg::AllowedCollisionMatrix&)`
    /// (`collision_matrix.cpp:80`), field for field: defaults first, then
    /// the strict upper triangle of `entry_values`, and only the cells that
    /// *differ* from the combined default are stored explicitly. That last
    /// condition is not an optimization -- it decides which cells a later
    /// `getPlanningSceneMsg` round-trips as explicit entries.
    ///
    /// Deviation (D6): upstream's two length mismatches are `RCLCPP_ERROR` +
    /// `return`, which leaves a *partially built* matrix in the caller's
    /// hands with no way to tell. Both are `Err` here.
    fn try_from(wrapped: AllowedCollisionMatrixMsg) -> Result<Self> {
        let msg = wrapped.0;
        if msg.entry_names.len() != msg.entry_values.len()
            || msg.default_entry_names.len() != msg.default_entry_values.len()
        {
            return Err(Error::construct(format!(
                "AllowedCollisionMatrix: entry_names/entry_values are {}/{} and \
                 default_entry_names/default_entry_values are {}/{}; each pair must have \
                 equal length",
                msg.entry_names.len(),
                msg.entry_values.len(),
                msg.default_entry_names.len(),
                msg.default_entry_values.len()
            )));
        }

        let mut acm = AllowedCollisionMatrix::new();
        for (name, &allowed) in msg
            .default_entry_names
            .iter()
            .zip(msg.default_entry_values.iter())
        {
            acm.set_default_entry(name, allowed);
        }

        for (i, name_i) in msg.entry_names.iter().enumerate() {
            let enabled = &msg.entry_values[i].enabled;
            if enabled.len() != msg.entry_names.len() {
                return Err(Error::construct(format!(
                    "AllowedCollisionMatrix: row '{name_i}' has {} enabled flag(s) but the \
                     matrix has {} entry name(s); the matrix must be square",
                    enabled.len(),
                    msg.entry_names.len()
                )));
            }
            for (j, name_j) in msg.entry_names.iter().enumerate().skip(i + 1) {
                let allowed_default =
                    combined_default(&acm, name_i, name_j).unwrap_or(AllowedCollisionType::Never);
                let allowed_entry = if enabled[j] {
                    AllowedCollisionType::Always
                } else {
                    AllowedCollisionType::Never
                };
                if allowed_entry != allowed_default {
                    acm.set_entry(name_i, name_j, enabled[j]);
                }
            }
        }
        Ok(acm)
    }
}

/// Upstream `setCurrentState` (`planning_scene.cpp:1217`), the RobotState
/// overload -- **the single owner of "a `RobotState` message reaches the
/// scene"**, and with it of every `AttachedCollisionObject` that arrives
/// inside a `PlanningScene`.
///
/// Upstream does not let the joint-state path touch attached bodies at all:
/// it copies the message, clears `attached_collision_objects` on the copy
/// (`:1221-1222`), converts *that* into the robot state, and then loops the
/// original message's attached objects through
/// `processAttachedCollisionObjectMsg` (`:1246`) one at a time. This port
/// takes the same shape, with [`apply_attached_collision_object`] as the
/// owner -- the same function
/// [`crate::monitored_scene::apply_attached_collision_object_msg`] routes the
/// `attached_collision_object` topic through. There is no second path that
/// attaches a body to the scene, which is what stops a topic attach and a
/// scene diff from being two mutators whose order decides the answer.
///
/// # Only ADD is reachable from here, and that is upstream's rule
///
/// Upstream rejects a non-ADD operation when the `RobotState` is not itself
/// a diff (`:1238-1245`), because "modify what is already attached" has no
/// meaning against a state that claims to be the whole truth. It logs and
/// skips the object; this port returns `Err` instead, per D6 and matching
/// how [`AllowedCollisionMatrixMsg`] treats upstream's log-and-continue on a
/// malformed matrix.
///
/// `is_diff == true` never reaches that test: [`RobotStateMsg`] rejects it
/// first (a bare `&RobotModel` conversion has no parent to diff against), so
/// the guard here reduces to "every attached object stated inside a
/// `PlanningScene` must be an ADD". That is a real narrowing against
/// upstream and it is recorded rather than hidden -- the `attached_collision_object`
/// topic is the path that carries REMOVE and APPEND.
fn set_current_state_msg(
    scene: &mut PlanningScene<'_>,
    mut msg: moveit_msgs::RobotState,
) -> Result<()> {
    let attached = std::mem::take(&mut msg.attached_collision_objects);
    let is_diff = msg.is_diff;

    let state = CoreRobotState::try_from(RobotStateMsg {
        model: scene.robot_model(),
        msg,
    })?;
    scene.set_current_state(state);

    for object in attached {
        if !is_diff
            && CollisionObjectOperation::try_from(object.object.operation)?
                != CollisionObjectOperation::Add
        {
            return Err(Error::other(format!(
                "RobotState.attached_collision_objects['{}'] asks for operation {} on a \
                 RobotState that is not marked is_diff -- upstream ignores the object here \
                 (planning_scene.cpp:1238-1245); use the attached_collision_object topic \
                 for anything but ADD",
                object.object.id, object.object.operation
            )));
        }
        apply_attached_collision_object(scene, object)?;
    }
    Ok(())
}

/// Upstream `setPlanningSceneMsg` (`planning_scene.cpp:1367`): the message
/// *is* the scene afterwards.
///
/// Field order is upstream's, and the two departures are both places this
/// port has nothing to depart from:
///
/// - `object_types_.reset()` (`:1382`) has no counterpart --
///   `hasObjectType`/`getObjectType`/... are D1 in `cspace_planning::scene`
///   (`object_recognition_msgs::msg::ObjectType`) and no object-type map
///   exists to reset.
/// - `world_->clearObjects()` (`:1392`) is
///   [`PlanningScene::remove_all_objects`].
///
/// `name` is assigned unconditionally, including an empty one -- that is
/// upstream's `name_ = scene_msg.name;` (`:1371`) verbatim, and it is what
/// separates
/// this arm from the diff arm's `if (!scene_msg.name.empty())`.
pub fn set_planning_scene_msg<'m>(
    scene: &mut PlanningScene<'m>,
    full: FullPlanningSceneMsg,
) -> Result<()> {
    let msg = full.0;
    reject_unrepresentable_fields(&msg)?;

    scene.set_name(msg.name);
    warn_on_robot_model_mismatch(scene, &msg.robot_model_name);

    // `if (parent_) decoupleParent()` (`:1379-1380`): a full scene is not a diff
    // of anything, so whatever this scene was layered on is materialized and
    // dropped first.
    scene.decouple_parent();

    apply_fixed_frame_transforms(scene, msg.fixed_frame_transforms)?;
    set_current_state_msg(scene, msg.robot_state)?;
    scene.set_allowed_collision_matrix(AllowedCollisionMatrix::try_from(
        AllowedCollisionMatrixMsg(msg.allowed_collision_matrix),
    )?);

    scene.remove_all_objects();
    apply_planning_scene_world(scene, msg.world)
}

/// Upstream `setPlanningSceneDiffMsg` (`planning_scene.cpp:1314`): every
/// field is applied only if the message actually states it, so an unstated
/// field leaves the scene's own value alone.
///
/// The emptiness test for each field is upstream's own, not a uniform "is
/// this default" rule -- `robot_state` in particular is stated when *any* of
/// `multi_dof_joint_state.joint_names`, `joint_state.name` or
/// `attached_collision_objects` is non-empty (`:1338-1340`), and the octomap
/// is stated when `world.octomap.octomap.id` is non-empty (`:1361`), which
/// is why this arm cannot simply call [`apply_planning_scene_world`].
pub fn set_planning_scene_diff_msg<'m>(
    scene: &mut PlanningScene<'m>,
    diff: PlanningSceneDiffMsg,
) -> Result<()> {
    let msg = diff.0;
    reject_unrepresentable_fields(&msg)?;

    if !msg.name.is_empty() {
        scene.set_name(msg.name);
    }
    warn_on_robot_model_mismatch(scene, &msg.robot_model_name);

    if !msg.fixed_frame_transforms.is_empty() {
        apply_fixed_frame_transforms(scene, msg.fixed_frame_transforms)?;
    }

    let robot_state = msg.robot_state;
    if !robot_state.multi_dof_joint_state.joint_names.is_empty()
        || !robot_state.joint_state.name.is_empty()
        || !robot_state.attached_collision_objects.is_empty()
    {
        set_current_state_msg(scene, robot_state)?;
    }

    if !msg.allowed_collision_matrix.entry_names.is_empty() {
        scene.set_allowed_collision_matrix(AllowedCollisionMatrix::try_from(
            AllowedCollisionMatrixMsg(msg.allowed_collision_matrix),
        )?);
    }

    for collision_object in msg.world.collision_objects {
        apply_collision_object(scene, collision_object)?;
    }
    if !msg.world.octomap.octomap.id.is_empty() {
        apply_octomap(scene, msg.world.octomap)?;
    }
    Ok(())
}

/// Upstream's `RCLCPP_WARN` on a `robot_model_name` naming some other model
/// (`planning_scene.cpp:1322-1326` in the diff arm, `:1373-1377` in the full
/// arm -- the same three lines twice). A warning, not an error: see
/// [`robot_model_name_matches`] for why upstream treats a mismatch as
/// advisory.
fn warn_on_robot_model_mismatch(scene: &PlanningScene<'_>, robot_model_name: &str) {
    if !robot_model_name_matches(scene, robot_model_name) {
        eprintln!(
            "Setting the scene for model '{robot_model_name}' but model '{}' is loaded.",
            scene.robot_model().name()
        );
    }
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
/// payload is decoded by [`cspace_core::octomap::OcTree::read_binary_data`] or
/// [`cspace_core::octomap::OcTree::read_data`] (round 8: those two entry points
/// landed in `cspace_core::octomap`, closing the round-5/round-7 structural gap
/// this doc comment used to describe) and inserted the same way
/// `apply_collision_object` inserts every other shape kind
/// (`cspace_planning::scene::PlanningScene::add_shape`, `src/scene/collision_object.rs:382`)
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
    // `map.octomap.resolution` is untrusted wire data, but it is not
    // validated here -- `cspace_core::octomap::OcTree::read_binary_data`/
    // `read_data` (called just below) reject a non-positive or non-finite
    // resolution themselves now (`DecodeError::InvalidResolution`), the one
    // shared choke point every caller of either function already passes
    // through, in this crate or any other. See that error variant's own doc
    // for why the resolution invariant belongs there and not at each
    // untrusted-data boundary separately.
    let mut tree = cspace_core::octomap::OcTree::new(map.octomap.resolution);
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
    use cspace_core::srdf::SrdfModel;

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

    /// A `moveit_msgs/CollisionObject` ADD carrying one sphere at the model
    /// frame -- the same fixture shape `collision_object.rs`'s own tests use,
    /// repeated here because that module's is `#[cfg(test)]`-private.
    fn sphere_object(id: &str, model_frame: &str) -> moveit_msgs::CollisionObject {
        moveit_msgs::CollisionObject {
            header: r2r::std_msgs::msg::Header {
                frame_id: model_frame.to_string(),
                ..Default::default()
            },
            pose: identity_pose(),
            id: id.to_string(),
            type_: Default::default(),
            primitives: vec![r2r::shape_msgs::msg::SolidPrimitive {
                type_: 2, // SPHERE
                dimensions: vec![0.1],
                polygon: Default::default(),
            }],
            primitive_poses: vec![identity_pose()],
            meshes: vec![],
            mesh_poses: vec![],
            planes: vec![],
            plane_poses: vec![],
            subframe_names: vec![],
            subframe_poses: vec![],
            operation: 0, // ADD
        }
    }

    /// A `PlanningScene` message with the given `is_diff` and one world
    /// object, everything else default.
    fn scene_msg(is_diff: bool, model_frame: &str, object_id: &str) -> moveit_msgs::PlanningScene {
        moveit_msgs::PlanningScene {
            is_diff,
            world: moveit_msgs::PlanningSceneWorld {
                collision_objects: vec![sphere_object(object_id, model_frame)],
                octomap: octomap_with_pose("OcTree", model_frame, true, vec![]),
            },
            ..Default::default()
        }
    }

    fn transform_stamped(
        from_frame: &str,
        child_frame_id: &str,
        x: f64,
    ) -> r2r::geometry_msgs::msg::TransformStamped {
        r2r::geometry_msgs::msg::TransformStamped {
            header: r2r::std_msgs::msg::Header {
                frame_id: from_frame.to_string(),
                ..Default::default()
            },
            child_frame_id: child_frame_id.to_string(),
            transform: r2r::geometry_msgs::msg::Transform {
                translation: r2r::geometry_msgs::msg::Vector3 { x, y: 0.0, z: 0.0 },
                rotation: r2r::geometry_msgs::msg::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        }
    }

    #[test]
    fn is_diff_flag_selects_the_variant_and_nothing_else_reads_it() {
        let full = PlanningSceneUpdate::from(moveit_msgs::PlanningScene {
            is_diff: false,
            ..Default::default()
        });
        assert!(
            matches!(full, PlanningSceneUpdate::Full(_)),
            "got: {full:?}"
        );
        let diff = PlanningSceneUpdate::from(moveit_msgs::PlanningScene {
            is_diff: true,
            ..Default::default()
        });
        assert!(
            matches!(diff, PlanningSceneUpdate::Diff(_)),
            "got: {diff:?}"
        );
    }

    /// The one behavioural difference the whole full/diff split exists for:
    /// upstream's full arm calls `world_->clearObjects()` (`:1392`) and the
    /// diff arm does not. A `use_planning_scene_msg` that routed a diff into
    /// the full arm would drop `first` here.
    #[test]
    fn a_diff_adds_to_the_world_and_a_full_scene_replaces_it() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);

        use_planning_scene_msg(&mut scene, scene_msg(false, model.model_frame(), "first")).unwrap();
        assert!(scene.world().get_object("first").is_some());

        use_planning_scene_msg(&mut scene, scene_msg(true, model.model_frame(), "second")).unwrap();
        assert!(
            scene.world().get_object("first").is_some(),
            "a diff must not clear the world: {:?}",
            scene.world()
        );
        assert!(scene.world().get_object("second").is_some());

        use_planning_scene_msg(&mut scene, scene_msg(false, model.model_frame(), "third")).unwrap();
        assert!(
            scene.world().get_object("first").is_none()
                && scene.world().get_object("second").is_none(),
            "a full scene must clear the world: {:?}",
            scene.world()
        );
        assert!(scene.world().get_object("third").is_some());
    }

    /// `name_ = scene_msg.name` (`:1371`) vs. `if (!scene_msg.name.empty())`
    /// (`:1319-1320`) -- a second, independent place the two arms differ, so
    /// a mutation that fixes only the world-clearing difference still shows
    /// up.
    #[test]
    fn an_empty_name_clears_it_on_a_full_scene_and_is_ignored_on_a_diff() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);

        let mut named = scene_msg(false, model.model_frame(), "obj");
        named.name = "kitchen".to_string();
        use_planning_scene_msg(&mut scene, named).unwrap();
        assert_eq!(scene.name(), "kitchen");

        use_planning_scene_msg(&mut scene, scene_msg(true, model.model_frame(), "obj2")).unwrap();
        assert_eq!(scene.name(), "kitchen");

        use_planning_scene_msg(&mut scene, scene_msg(false, model.model_frame(), "obj3")).unwrap();
        assert_eq!(scene.name(), "");
    }

    #[test]
    fn a_full_scene_decouples_a_parent() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let parent = Arc::new(PlanningScene::new(&model, &srdf));
        let mut child = parent.diff();
        assert!(is_diff(&child));
        use_planning_scene_msg(&mut child, scene_msg(false, model.model_frame(), "obj")).unwrap();
        assert!(!is_diff(&child));
    }

    #[test]
    fn a_diff_keeps_its_parent() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let parent = Arc::new(PlanningScene::new(&model, &srdf));
        let mut child = parent.diff();
        use_planning_scene_msg(&mut child, scene_msg(true, model.model_frame(), "obj")).unwrap();
        assert!(is_diff(&child));
    }

    /// The diff arm's octomap guard (`:1361`), both sides of it. `world
    /// .octomap` with an empty `id` is "not stated" and is skipped entirely;
    /// the same message with `id == "OcTree"` is stated, so it reaches
    /// `apply_octomap`, whose leading `remove_object(OCTOMAP_NS)` clears the
    /// previous octree before the empty-`data` early return.
    ///
    /// Both legs carry identical, empty `data`, so the only thing that
    /// differs between them is the `id` the guard reads -- a test that
    /// changed the payload too could not say which of the two caused the
    /// change.
    #[test]
    fn a_diff_applies_its_octomap_only_when_the_id_is_stated() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);

        let with_octree = |scene: &mut PlanningScene<'_>| {
            let mut msg = scene_msg(false, model.model_frame(), "obj");
            msg.world.octomap = octomap_with_pose("OcTree", model.model_frame(), true, vec![1, 2]);
            use_planning_scene_msg(scene, msg).unwrap();
            assert!(scene.world().get_object(OCTOMAP_NS).is_some());
        };

        with_octree(&mut scene);
        let mut unnamed = scene_msg(true, model.model_frame(), "obj2");
        unnamed.world.octomap.octomap.id = String::new();
        use_planning_scene_msg(&mut scene, unnamed).unwrap();
        assert!(
            scene.world().get_object(OCTOMAP_NS).is_some(),
            "an unnamed octomap in a diff must not clear the octree: {:?}",
            scene.world()
        );

        with_octree(&mut scene);
        let named = scene_msg(true, model.model_frame(), "obj3");
        assert_eq!(named.world.octomap.octomap.id, "OcTree");
        assert!(named.world.octomap.octomap.data.is_empty());
        use_planning_scene_msg(&mut scene, named).unwrap();
        assert!(
            scene.world().get_object(OCTOMAP_NS).is_none(),
            "a stated but empty octomap in a diff must clear the octree: {:?}",
            scene.world()
        );
    }

    #[test]
    fn non_empty_link_padding_is_rejected() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let mut msg = scene_msg(false, model.model_frame(), "obj");
        msg.link_padding = vec![moveit_msgs::LinkPadding {
            link_name: "base_link".to_string(),
            padding: 0.01,
        }];
        let err = use_planning_scene_msg(&mut scene, msg).unwrap_err();
        assert!(err.to_string().contains("link_padding"), "got: {err:?}");
    }

    #[test]
    fn non_empty_link_scale_is_rejected() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let mut msg = scene_msg(true, model.model_frame(), "obj");
        msg.link_scale = vec![moveit_msgs::LinkScale {
            link_name: "base_link".to_string(),
            scale: 1.1,
        }];
        let err = use_planning_scene_msg(&mut scene, msg).unwrap_err();
        assert!(err.to_string().contains("link_scale"), "got: {err:?}");
    }

    #[test]
    fn non_empty_object_colors_is_rejected() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let mut msg = scene_msg(false, model.model_frame(), "obj");
        msg.object_colors = vec![moveit_msgs::ObjectColor {
            id: "obj".to_string(),
            color: Default::default(),
        }];
        let err = use_planning_scene_msg(&mut scene, msg).unwrap_err();
        assert!(err.to_string().contains("object_colors"), "got: {err:?}");
    }

    #[test]
    fn fixed_frame_transforms_are_merged_not_replaced() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let planning_frame = scene.planning_frame().to_string();

        let mut first = scene_msg(false, model.model_frame(), "obj");
        first.fixed_frame_transforms = vec![transform_stamped("table", &planning_frame, 1.0)];
        use_planning_scene_msg(&mut scene, first).unwrap();

        let mut second = scene_msg(true, model.model_frame(), "obj2");
        second.fixed_frame_transforms = vec![transform_stamped("shelf", &planning_frame, 2.0)];
        use_planning_scene_msg(&mut scene, second).unwrap();

        assert_eq!(
            scene.transforms().transform("table").unwrap().translation.x,
            1.0
        );
        assert_eq!(
            scene.transforms().transform("shelf").unwrap().translation.x,
            2.0
        );
    }

    #[test]
    fn a_fixed_frame_transform_to_another_frame_is_rejected() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let mut msg = scene_msg(false, model.model_frame(), "obj");
        msg.fixed_frame_transforms = vec![transform_stamped("table", "some_other_frame", 1.0)];
        let err = use_planning_scene_msg(&mut scene, msg).unwrap_err();
        assert!(
            err.to_string().contains("'some_other_frame'"),
            "got: {err:?}"
        );
    }

    #[test]
    fn robot_state_joint_positions_reach_the_scene() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let mut msg = scene_msg(false, model.model_frame(), "obj");
        msg.robot_state.joint_state.name = vec!["j1".to_string()];
        msg.robot_state.joint_state.position = vec![0.5];
        use_planning_scene_msg(&mut scene, msg).unwrap();
        assert_eq!(scene.current_state().variable_position("j1").unwrap(), 0.5);
    }

    /// The diff arm's `if` at `:1338-1340`: a `robot_state` that states
    /// nothing must leave the scene's own state alone, where the full arm
    /// resets it.
    #[test]
    fn an_unstated_robot_state_is_ignored_by_a_diff_and_reset_by_a_full_scene() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        scene
            .current_state_mut()
            .set_variable_position("j1", 0.5)
            .unwrap();

        use_planning_scene_msg(&mut scene, scene_msg(true, model.model_frame(), "obj")).unwrap();
        assert_eq!(scene.current_state().variable_position("j1").unwrap(), 0.5);

        use_planning_scene_msg(&mut scene, scene_msg(false, model.model_frame(), "obj2")).unwrap();
        assert_eq!(scene.current_state().variable_position("j1").unwrap(), 0.0);
    }

    #[test]
    fn acm_defaults_and_entries_convert() {
        let msg = moveit_msgs::AllowedCollisionMatrix {
            entry_names: vec!["a".to_string(), "b".to_string()],
            entry_values: vec![
                moveit_msgs::AllowedCollisionEntry {
                    enabled: vec![false, true],
                },
                moveit_msgs::AllowedCollisionEntry {
                    enabled: vec![true, false],
                },
            ],
            default_entry_names: vec!["c".to_string()],
            default_entry_values: vec![true],
        };
        let acm = AllowedCollisionMatrix::try_from(AllowedCollisionMatrixMsg(msg)).unwrap();
        // (a,b) is ALWAYS and neither has a default, so the combined default
        // is upstream's `NEVER` fallback and the cell differs -> stored.
        assert_eq!(
            acm.allowed_collision("a", "b").map(|e| e.kind()),
            Some(AllowedCollisionType::Always)
        );
        assert_eq!(
            acm.default_entry("c").map(|e| e.kind()),
            Some(AllowedCollisionType::Always)
        );
    }

    /// `if (allowed_entry != allowed_default)` (`collision_matrix.cpp:109`):
    /// a cell equal to the combined default is *not* stored as an explicit
    /// entry. Asserted through `has_pair_entry`, which sees the explicit
    /// table only -- `allowed_collision` would answer `Always` either way and
    /// could not tell the two apart.
    #[test]
    fn an_acm_cell_matching_its_default_is_not_stored_explicitly() {
        let msg = moveit_msgs::AllowedCollisionMatrix {
            entry_names: vec!["a".to_string(), "b".to_string()],
            entry_values: vec![
                moveit_msgs::AllowedCollisionEntry {
                    enabled: vec![false, true],
                },
                moveit_msgs::AllowedCollisionEntry {
                    enabled: vec![true, false],
                },
            ],
            default_entry_names: vec!["a".to_string(), "b".to_string()],
            default_entry_values: vec![true, true],
        };
        let acm = AllowedCollisionMatrix::try_from(AllowedCollisionMatrixMsg(msg)).unwrap();
        assert!(
            !acm.has_pair_entry("a", "b"),
            "(a,b) is ALWAYS and both defaults are ALWAYS, so it must not be stored: {acm:?}"
        );
        assert_eq!(
            acm.allowed_collision("a", "b").map(|e| e.kind()),
            Some(AllowedCollisionType::Always)
        );
    }

    /// The other half of the same rule: one `NEVER` default makes the
    /// combined default `NEVER` (`collision_matrix.cpp:350-353`), so the same
    /// `ALWAYS` cell now differs and *is* stored.
    #[test]
    fn one_never_default_makes_the_combined_default_never() {
        let msg = moveit_msgs::AllowedCollisionMatrix {
            entry_names: vec!["a".to_string(), "b".to_string()],
            entry_values: vec![
                moveit_msgs::AllowedCollisionEntry {
                    enabled: vec![false, true],
                },
                moveit_msgs::AllowedCollisionEntry {
                    enabled: vec![true, false],
                },
            ],
            default_entry_names: vec!["a".to_string(), "b".to_string()],
            default_entry_values: vec![true, false],
        };
        let acm = AllowedCollisionMatrix::try_from(AllowedCollisionMatrixMsg(msg)).unwrap();
        assert!(acm.has_pair_entry("a", "b"), "got: {acm:?}");
    }

    #[test]
    fn an_acm_with_mismatched_default_lengths_is_rejected() {
        let msg = moveit_msgs::AllowedCollisionMatrix {
            entry_names: vec![],
            entry_values: vec![],
            default_entry_names: vec!["a".to_string()],
            default_entry_values: vec![],
        };
        let err = AllowedCollisionMatrix::try_from(AllowedCollisionMatrixMsg(msg)).unwrap_err();
        assert!(err.to_string().contains("equal length"), "got: {err:?}");
    }

    #[test]
    fn a_non_square_acm_row_is_rejected() {
        let msg = moveit_msgs::AllowedCollisionMatrix {
            entry_names: vec!["a".to_string(), "b".to_string()],
            entry_values: vec![
                moveit_msgs::AllowedCollisionEntry {
                    enabled: vec![false],
                },
                moveit_msgs::AllowedCollisionEntry {
                    enabled: vec![true, false],
                },
            ],
            default_entry_names: vec![],
            default_entry_values: vec![],
        };
        let err = AllowedCollisionMatrix::try_from(AllowedCollisionMatrixMsg(msg)).unwrap_err();
        assert!(err.to_string().contains("must be square"), "got: {err:?}");
    }

    /// The `entry_*` half of the folded length guard (`:288-290`): one `if`
    /// over two independently named pairs, of which
    /// `an_acm_with_mismatched_default_lengths_is_rejected` above reaches only
    /// the `default_entry_*` half. Measured, not assumed: replacing
    /// `msg.entry_names.len() != msg.entry_values.len()` with `false` left the
    /// whole `cspace-ros` suite at 203 passed, so before this test the operand
    /// was a blind site in `doc/folded-operand-guards.md`'s sense -- and it is
    /// the operand that keeps `msg.entry_values[i]` (`:312`) from indexing past
    /// the end. The needle names the offending pair and its two lengths rather
    /// than the shared "equal length" tail, so it cannot pass on its sibling's
    /// failure.
    #[test]
    fn an_acm_with_mismatched_entry_lengths_is_rejected() {
        let msg = moveit_msgs::AllowedCollisionMatrix {
            entry_names: vec!["a".to_string()],
            entry_values: vec![],
            default_entry_names: vec![],
            default_entry_values: vec![],
        };
        let err = AllowedCollisionMatrix::try_from(AllowedCollisionMatrixMsg(msg)).unwrap_err();
        assert!(
            err.to_string()
                .contains("AllowedCollisionMatrix: entry_names/entry_values are 1/0"),
            "got: {err:?}"
        );
    }

    /// The diff arm applies the ACM only when `entry_names` is non-empty
    /// (`:1343-1344`), so a diff that states no matrix must not wipe the SRDF-
    /// derived one the scene already has.
    #[test]
    fn a_diff_with_an_empty_acm_leaves_the_existing_one_alone() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        scene
            .allowed_collision_matrix_mut()
            .set_entry("base_link", "tip", true);

        use_planning_scene_msg(&mut scene, scene_msg(true, model.model_frame(), "obj")).unwrap();
        assert!(
            scene
                .allowed_collision_matrix()
                .has_pair_entry("base_link", "tip")
        );

        use_planning_scene_msg(&mut scene, scene_msg(false, model.model_frame(), "obj2")).unwrap();
        assert!(
            !scene
                .allowed_collision_matrix()
                .has_pair_entry("base_link", "tip"),
            "a full scene replaces the ACM outright"
        );
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

    /// A zero resolution used to reach `OcTree::new` unrejected and decode
    /// successfully -- `read_binary_data` never touched `resolution` --
    /// while silently corrupting every leaf's coordinate to the world
    /// origin one level further down (`crates/cspace-core/src/octomap/tree.rs`'s
    /// `key_to_coord_axis`, a multiplication by `resolution` with no NaN or
    /// Infinity to trip a guard). `read_binary_data` now rejects it directly
    /// (`DecodeError::InvalidResolution`), reached here through the same
    /// `decode_result.map_err` this function already had.
    #[test]
    fn zero_resolution_octree_is_rejected() {
        let model = one_joint_model();
        let srdf = empty_srdf();
        let mut scene = PlanningScene::new(&model, &srdf);
        let mut world = moveit_msgs::PlanningSceneWorld {
            collision_objects: vec![],
            octomap: octomap_with_pose("OcTree", model.model_frame(), true, vec![1, 2]),
        };
        world.octomap.octomap.resolution = 0.0;
        let err = apply_planning_scene_world(&mut scene, world).unwrap_err();
        assert!(
            err.to_string().contains("octomap payload decode failed")
                && err.to_string().contains("resolution"),
            "got: {err:?}"
        );
    }

    /// Same boundary, the other two non-finite/non-positive values a wire
    /// message can carry that `DecodeError::InvalidResolution`'s own guard
    /// must also catch: a negative resolution (well-defined, not a
    /// NaN/Infinity producer, but still not a valid voxel size) and
    /// `f64::NAN` itself.
    #[test]
    fn negative_and_nan_resolution_octree_are_rejected() {
        for bad in [-0.1, f64::NAN] {
            let model = one_joint_model();
            let srdf = empty_srdf();
            let mut scene = PlanningScene::new(&model, &srdf);
            let mut world = moveit_msgs::PlanningSceneWorld {
                collision_objects: vec![],
                octomap: octomap_with_pose("OcTree", model.model_frame(), true, vec![1, 2]),
            };
            world.octomap.octomap.resolution = bad;
            let err = apply_planning_scene_world(&mut scene, world).unwrap_err();
            assert!(
                err.to_string().contains("octomap payload decode failed")
                    && err.to_string().contains("resolution"),
                "resolution {bad}, got: {err:?}"
            );
        }
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
    /// crate's own claim audit caught (`doc/claim-audit/cspace-ros.md`,
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
