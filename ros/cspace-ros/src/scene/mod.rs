// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `moveit_msgs::msg::{PlanningScene, PlanningSceneWorld, CollisionObject,
//! AttachedCollisionObject}` <-> `cspace_planning::scene`/`cspace_collision`.
//! See `doc/message-mapping.md` §11.

pub mod attached;
pub mod collision_object;
pub mod planning_scene;
pub mod shapes;

use cspace_core::error::Result;
use cspace_core::geometry::Isometry3;
use cspace_planning::scene::PlanningScene;

/// Resolves a message `header.frame_id` the way the upstream call sites that
/// call `getFrameTransform` *without* a preceding `knowsFrameTransform`
/// guard do (`processOctomapMsg(OctomapWithPose)`: `planning_scene.cpp:1494`;
/// `processAttachedCollisionObjectMsg`'s not-in-world branch: `:1606`;
/// `processCollisionObjectMove`: `:1964`) -- **not** every call site
/// (`processCollisionObjectAdd`'s `:1905` is guarded by `knowsFrameTransform`
/// at `:1889` and upstream rejects an empty frame_id there too, matching
/// [`PlanningScene::frame_transform`]'s `Err` directly; that call site must
/// keep calling `frame_transform` on its own, not this helper).
///
/// An empty `frame_id` is not an unresolved name -- it is the wire's stated
/// default of "already in world coordinates", and upstream's own fallback
/// (`Transforms::getTransform`, `transforms.cpp:110-126`: `if
/// (!from_frame.empty()) { ...lookup... }` then log-and-return identity)
/// resolves it to identity *without* going through the unresolved-name path
/// at all. `frame_transform`'s `Err` on an unresolved name is a deliberate,
/// pre-existing deviation from upstream's silent-identity-and-log fallback
/// (D6) -- but D6 exists to reject a typo'd frame name instead of silently
/// absorbing it into identity, and an empty string is not a typo, it is the
/// explicit default. Branching here keeps D6's actual purpose intact for
/// every non-empty, unresolved name while accepting the same wire input
/// upstream accepts (PORTING-PLAN.md §183).
pub(crate) fn header_frame_transform(
    scene: &mut PlanningScene<'_>,
    frame_id: &str,
) -> Result<Isometry3> {
    if frame_id.is_empty() {
        Ok(Isometry3::identity())
    } else {
        scene.frame_transform(frame_id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::tests::one_joint_model;
    use cspace_core::error::Error;
    use cspace_core::model::RobotModel;
    use cspace_core::srdf::SrdfModel;

    fn scene(model: &RobotModel) -> PlanningScene<'_> {
        let srdf =
            SrdfModel::parse_str("<?xml version=\"1.0\"?><robot name=\"one_joint\"></robot>")
                .expect("empty SRDF must parse");
        PlanningScene::new(model, &srdf)
    }

    /// The three boundary values this helper exists to tell apart
    /// (PORTING-PLAN.md §183.2): an empty `frame_id` is the wire's own
    /// "already in world coordinates" default, not an unresolved name.
    #[test]
    fn empty_frame_id_resolves_to_identity() {
        let model = one_joint_model();
        let mut scene = scene(&model);
        assert_eq!(
            header_frame_transform(&mut scene, "").unwrap(),
            Isometry3::identity()
        );
    }

    #[test]
    fn resolvable_frame_id_resolves_the_same_as_frame_transform() {
        let model = one_joint_model();
        let mut scene = scene(&model);
        let expected = scene.frame_transform(model.model_frame()).unwrap();
        assert_eq!(
            header_frame_transform(&mut scene, model.model_frame()).unwrap(),
            expected
        );
    }

    #[test]
    fn unresolvable_non_empty_frame_id_is_still_rejected() {
        let model = one_joint_model();
        let mut scene = scene(&model);
        let err = header_frame_transform(&mut scene, "no-such-frame").unwrap_err();
        assert!(matches!(err, Error::UnknownName { .. }), "got: {err:?}");
    }
}
