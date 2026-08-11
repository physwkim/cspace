// Copyright (c) 2019, Jens Petit
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection_bullet/src/collision_env_bullet.cpp

//! `CollisionEnvBullet::checkRobotCollisionHelperCCD` -- the two-state
//! robot-vs-world check, which is the one query MoveIt answers with a
//! continuous algorithm.
//!
//! Everything below the manager is [`cspace_bullet_cast`] and the
//! `cspace-bullet` crate under it; what is here is the part that lives in
//! `collision_env_bullet.cpp` and therefore under this crate's licence: which
//! collision objects the manager is given, in what order, and how the answer
//! is turned back into a [`CollisionResult`].
//!
//! # A manager per query
//!
//! `CollisionEnvBullet` keeps `manager_CCD_` for the environment's lifetime:
//! links are added in the constructor, world objects arrive through a
//! `World::Observer`, and only attached bodies are added and removed around
//! each query. [`check_robot_collision_continuous`] builds one, fills it and
//! drops it.
//!
//! That is a cost deviation and not a result deviation. The broadphase is a
//! function of the AABBs it is given and the order they were given in, both of
//! which are reproduced below; nothing in it carries over from one query to
//! the next that a second query would read. What it costs is one
//! `createShapePrimitive` per link and per world object per query -- including
//! one `btConvexHullComputer::compute` per mesh -- where upstream pays that
//! once. [`crate::env::CollisionEnv::check_robot_collision_continuous`] takes
//! `&self`, so there is no place to keep the manager without making the
//! backend's interior state observable to a caller that shares it across
//! threads.
//!
//! # Order, and why it is not incidental
//!
//! Objects are added in upstream's order: every link first, then every world
//! object, then this query's attached bodies. Within the links, *by name* --
//! upstream walks `robot_model_->getURDF()->links_`, a `std::map` keyed by
//! link name, not the model's own link order. Within the world objects, by id,
//! which is what `World`'s own map iteration gives on both sides.
//!
//! The order decides the answer whenever the request bounds the contacts.
//! `createProxy` announces overlaps as it inserts, `HashedOverlappingPairCache`
//! files each pair at a bucket derived from the pair array's *capacity*, and
//! `processAllOverlappingPairs` walks that array; so a different insertion
//! order is a different traversal order, and a `max_contacts` budget then keeps
//! a different subset.
//!
//! # What upstream's Bullet backend does not compute
//!
//! - **Costs.** `CollisionEnvBullet` never writes `cost_sources`, so
//!   [`CollisionResult::cost_sources`] stays [`None`] even for a request that
//!   asked (upstream leaves the field default-constructed, which is the same
//!   thing read through a different API).
//! - **A detailed distance.** Nothing on this path fills
//!   `res.distance_result`, so the distance -- when one was asked for -- is
//!   always [`CollisionDistance::Closest`].
//! - **`nearest_points`.** `addCastSingleResult` never assigns them and
//!   `collision_detection::Contact` gives them no initialiser
//!   (`collision_common.hpp:105`), so upstream's cast contacts carry whatever
//!   the stack held. There is no value to reproduce and this port writes
//!   zeros.
//! - **`group_name`.** `checkRobotCollisionHelperCCD` re-poses every link in
//!   `active_`, which is every link that had geometry to add, and consults no
//!   group. This is a difference from the same environment's *discrete*
//!   checks, which do filter by group ([`crate::parry`]'s
//!   `active_group_links`); it is upstream's difference between its two
//!   backends, not one introduced here.

use std::collections::BTreeMap;

use cspace_bullet_cast::cast_bvh_manager::{AddObjectError, BulletCastBvhManager};
use cspace_bullet_cast::cast_callback::{
    AllowedCollisionType as BulletAllowedCollisionType, AllowedCollisions,
};
use cspace_bullet_cast::collision_object::{CollisionObjectWrapper, convert_eigen_to_bt};
use cspace_bullet_cast::contact_test_data::{
    BodyType as BulletBodyType, CastRequest, CastResult, Contact as BulletContact,
};
use cspace_bullet_cast::shape_primitive::CollisionObjectType;
use cspace_core::error::{Error, Result};
use cspace_core::geometry::shapes::Shape;
use cspace_core::geometry::{Isometry3, Vector3};
use cspace_core::state::Posed;

use crate::common::{
    AttachedBodyGeometry, BodyType, CollisionDistance, CollisionRequest, CollisionResult, Contact,
    ContactData,
};
use crate::env::LinkPaddingScale;
use crate::matrix::{AllowedCollision, AllowedCollisionMatrix};
use crate::world::World;

/// `checkRobotCollisionHelperCCD(req, res, state1, state2, acm)`
/// (`collision_env_bullet.cpp:209-238`), with the manager's construction
/// (`:60-63`, `:253-275`) folded in ahead of it.
///
/// # Errors
///
/// [`Error`] when any body cannot be built or any pair cannot be checked --
/// see [`check_robot_collision_continuous`]'s own callers for why that is not
/// reported as "no collision". A mesh-shaped *attached* body is the one such
/// case reachable from well-formed input: it needs `btTriangleShapeEx`, which
/// this port does not carry.
pub fn check_robot_collision_continuous(
    world: &World,
    padding_scale: &LinkPaddingScale,
    request: &CollisionRequest,
    state1: &Posed<'_, '_>,
    state2: &Posed<'_, '_>,
    attached_bodies: &[AttachedBodyGeometry<'_>],
    acm: Option<&AllowedCollisionMatrix>,
) -> Result<CollisionResult> {
    let mut manager = BulletCastBvhManager::new();
    // `active_`, which holds link names only: an attached body is re-posed by
    // the loop that adds it, not by this one.
    let mut active: Vec<(String, Isometry3, Isometry3)> = Vec::new();

    // `for (link : robot_model_->getURDF()->links_) addLinkAsCollisionObject`
    // (`:60-63`), by link name -- see the module docs.
    let model = state1.model();
    let mut links: Vec<&cspace_core::model::LinkModel> = model.link_models().iter().collect();
    links.sort_by_key(|link| link.name());
    for link in links {
        // `if (!link->collision_array.empty())` (`:391`): a link with no
        // geometry is never added and never joins `active_`.
        if link.shapes().is_empty() {
            continue;
        }
        let scale = padding_scale.link_scale(link.name());
        let padding = padding_scale.link_padding(link.name());
        let index = link.link_index();

        let mut shapes = Vec::with_capacity(link.shapes().len());
        let mut poses = Vec::with_capacity(link.shapes().len());
        for link_shape in link.shapes() {
            shapes.push(scaled_padded(&link_shape.shape, scale, padding));
            poses.push(link_shape.origin_transform);
        }

        // `shape_poses.push_back(urdfPose2Eigen(i->origin))` (`:417`): the
        // link-*local* collision origins, unposed. The link is placed later,
        // by the `setCastCollisionObjectsTransform` loop below.
        //
        // Posing them here would build the same compound -- a child is stored
        // relative to pose 0, and a link is rigid, so the global prefix
        // cancels -- but not the same *world* transform, which the constructor
        // takes from pose 0 and `createProxy` reads as the AABB before any
        // cast transform is set. Add the links already posed and each proxy
        // enters the tree where the robot stands, overlapping almost nothing;
        // upstream's enter piled around the origin, overlapping each other.
        // The two broadphases then announce their overlaps in different
        // orders, and that order is the pair cache's -- which is what a
        // bounded `max_contacts` keeps a prefix of.
        add(
            &mut manager,
            link.name(),
            BodyType::RobotLink,
            &shapes,
            &poses,
            true,
            None,
        )?;
        active.push((
            link.name().to_owned(),
            state1.global_link_transform_at(index) * link.shapes()[0].origin_transform,
            state2.global_link_transform_at(index) * link.shapes()[0].origin_transform,
        ));
    }

    // `notifyObserverAllObjects(observer_handle_, World::CREATE)` -> `addToManager`
    // (`:253-275`). World objects are never scaled or padded, and never swept.
    for (id, object) in world.iter() {
        let mut shapes = Vec::with_capacity(object.shapes().len());
        let mut poses = Vec::with_capacity(object.shapes().len());
        for entry in object.shapes() {
            shapes.push((**entry.shape()).clone());
            poses.push(object.pose() * entry.pose());
        }
        if shapes.is_empty() {
            continue;
        }
        add(
            &mut manager,
            id,
            BodyType::WorldObject,
            &shapes,
            &poses,
            false,
            None,
        )?;
    }

    // `addAttachedObjects(state1, attached_cows)` then, per body, an
    // `addCollisionObject` immediately followed by its
    // `setCastCollisionObjectsTransform` (`:216-225`) -- interleaved, and
    // ahead of the link loop below, because both the insertion order and the
    // re-posing order reach the pair cache.
    //
    // Unlike the FCL backend's, these shapes are *not* scaled or padded by the
    // attached link's entry: the constructor upstream calls here takes them as
    // they are. Their poses arrive global
    // (`getGlobalCollisionBodyTransforms()`), so unlike a link's they need no
    // composition beyond the attached link's own.
    for body in attached_bodies {
        if body.shapes.is_empty() {
            continue;
        }
        let link_pose = state1.global_link_transform(body.link_name)?;
        let shapes: Vec<Shape> = body.shapes.iter().map(|shape| (**shape).clone()).collect();
        let poses: Vec<Isometry3> = body
            .shape_poses
            .iter()
            .map(|pose| link_pose * pose)
            .collect();
        add(
            &mut manager,
            body.id,
            BodyType::RobotAttached,
            &shapes,
            &poses,
            true,
            Some(body),
        )?;
        let pose_2 = state2.global_link_transform(body.link_name)? * body.shape_poses[0];
        manager.set_cast_collision_objects_transform(
            body.id,
            convert_eigen_to_bt(&poses[0]),
            convert_eigen_to_bt(&pose_2),
        );
    }

    // `for (link : active_) setCastCollisionObjectsTransform(link,
    // state1.getCollisionBodyTransform(link, 0),
    // state2.getCollisionBodyTransform(link, 0))` (`:226-231`).
    for (name, tf1, tf2) in &active {
        manager.set_cast_collision_objects_transform(
            name,
            convert_eigen_to_bt(tf1),
            convert_eigen_to_bt(tf2),
        );
    }

    let cast_request = CastRequest {
        distance: request.distance,
        contacts: request.contacts,
        max_contacts: request.max_contacts,
        max_contacts_per_pair: request.max_contacts_per_pair,
    };
    let bridge = acm.map(AcmBridge);
    let result = manager
        .contact_test(
            &cast_request,
            bridge.as_ref().map(|b| b as &dyn AllowedCollisions),
        )
        .map_err(|error| {
            Error::other(format!(
                "continuous robot-collision checking could not finish: {error:?}"
            ))
        })?;

    Ok(convert_result(request, result))
}

/// One `CollisionObjectWrapper` into the manager, with upstream's `try`/`catch`
/// replaced by propagation.
///
/// `addAttachedObjects` catches and logs "Not adding `<name>` due to bad
/// arguments" (`:356-359`) and `addLinkAsCollisionObject` does the same
/// (`:445-449`). Both are catching the constructor's `throw std::exception()`
/// -- a shape/pose/type count mismatch, which cannot happen from the call
/// sites above because the three vectors are built together. The errors that
/// *are* reachable are the ones upstream does not throw for at all: a shape
/// `createShapePrimitive` refuses, where upstream returns a null shape and
/// dereferences it one line later. Dropping the body instead would make the
/// query answer "nothing here" about geometry it never looked at.
fn add(
    manager: &mut BulletCastBvhManager,
    name: &str,
    body_type: BodyType,
    shapes: &[Shape],
    poses: &[Isometry3],
    active: bool,
    attached: Option<&AttachedBodyGeometry<'_>>,
) -> Result<()> {
    let types: Vec<CollisionObjectType> = shapes.iter().map(collision_object_type).collect();
    let mut cow = CollisionObjectWrapper::new(
        name,
        bullet_body_type(body_type),
        shapes,
        poses,
        &types,
        active,
    )
    .map_err(|error| {
        Error::other(format!(
            "collision object {name:?} could not be built for the continuous check: \
                     {error:?}"
        ))
    })?;
    if let Some(attached) = attached {
        cow.touch_links = attached.touch_links.clone();
    }
    manager
        .add_collision_object(cow)
        .map_err(|error| match error {
            AddObjectError::DuplicateName(name) => Error::other(format!(
                "two collision objects are named {name:?}; a link, a world object and an attached \
             body all share one namespace in the continuous check"
            )),
            AddObjectError::Cast(error) => Error::other(format!(
                "collision object {name:?} could not be made to sweep: {error:?}"
            )),
        })
}

/// `if (shape->type == shapes::MESH) CONVEX_HULL else USE_SHAPE_TYPE`
/// (`collision_env_bullet.cpp:257-267`, `:389-425`).
///
/// `addAttachedObjects` is the exception and picks `USE_SHAPE_TYPE` for every
/// shape including a mesh (`:345-346`) -- which is the branch that needs
/// `btTriangleShapeEx`. Reproduced rather than smoothed over: choosing the hull
/// there would make a mesh-shaped attached body check against a *different*
/// solid than upstream's, which is worse than not checking it.
fn collision_object_type(shape: &Shape) -> CollisionObjectType {
    match shape {
        Shape::Mesh(_) => CollisionObjectType::ConvexHull,
        _ => CollisionObjectType::UseShapeType,
    }
}

/// `scaleAndPadd(getLinkScale(name), getLinkPadding(name))` behind upstream's
/// own guard (`collision_env_bullet.cpp:409-413`), which skips the call when
/// neither differs from its neutral value.
fn scaled_padded(shape: &Shape, scale: f64, padding: f64) -> Shape {
    let mut shape = shape.clone();
    if (scale - 1.0).abs() < f64::EPSILON && padding.abs() < f64::EPSILON {
        return shape;
    }
    if let Shape::Mesh(mesh) = &mut shape {
        if mesh.vertex_normals.is_none() {
            mesh.compute_vertex_normals();
        }
    }
    shape.scale_and_padd(scale, padding).expect(
        "every dimension is non-negative by construction and any mesh has had its vertex normals \
         computed just above, so scale_and_padd cannot fail here",
    );
    shape
}

/// `acm_` as the cast callbacks read it.
struct AcmBridge<'a>(&'a AllowedCollisionMatrix);

impl AllowedCollisions for AcmBridge<'_> {
    fn allowed_collision(&self, body_1: &str, body_2: &str) -> Option<BulletAllowedCollisionType> {
        self.0
            .allowed_collision(body_1, body_2)
            .map(|allowed| match allowed {
                AllowedCollision::Never => BulletAllowedCollisionType::Never,
                AllowedCollision::Always => BulletAllowedCollisionType::Always,
                // `acmCheck` reads the type and never the predicate
                // (`bullet_utils.cpp:49-83`): a conditional pair is skipped
                // outright, without the contact that would decide it ever
                // being computed.
                AllowedCollision::Conditional(_) => BulletAllowedCollisionType::Conditional,
            })
    }
}

fn bullet_body_type(body_type: BodyType) -> BulletBodyType {
    match body_type {
        BodyType::RobotLink => BulletBodyType::RobotLink,
        BodyType::RobotAttached => BulletBodyType::RobotAttached,
        BodyType::WorldObject => BulletBodyType::WorldObject,
    }
}

fn body_type(body_type: BulletBodyType) -> BodyType {
    match body_type {
        BulletBodyType::RobotLink => BodyType::RobotLink,
        BulletBodyType::RobotAttached => BodyType::RobotAttached,
        BulletBodyType::WorldObject => BodyType::WorldObject,
    }
}

/// The `CollisionResult` upstream has been writing into all along, assembled
/// from the fields `processResult` actually set -- see the module docs for the
/// three it never sets.
fn convert_result(request: &CollisionRequest, cast: CastResult) -> CollisionResult {
    CollisionResult {
        collision: cast.collision,
        distance: request
            .distance
            .then_some(CollisionDistance::Closest(cast.distance)),
        contacts: request.contacts.then(|| ContactData {
            by_pair: cast
                .contacts
                .into_iter()
                .map(|(key, contacts)| (key, contacts.iter().map(convert_contact).collect()))
                .collect::<BTreeMap<_, _>>(),
        }),
        cost_sources: None,
    }
}

fn convert_contact(contact: &BulletContact) -> Contact {
    // A Bullet `btVector3` is `float` in the configuration this port
    // reproduces, so every component widens exactly.
    Contact {
        pos: Vector3::new(
            contact.pos.x.into(),
            contact.pos.y.into(),
            contact.pos.z.into(),
        ),
        normal: Vector3::new(
            contact.normal.x.into(),
            contact.normal.y.into(),
            contact.normal.z.into(),
        ),
        depth: contact.depth.into(),
        body_name_1: contact.body_name_1.clone(),
        body_type_1: body_type(contact.body_type_1),
        body_name_2: contact.body_name_2.clone(),
        body_type_2: body_type(contact.body_type_2),
        percent_interpolation: contact.percent_interpolation.into(),
        // See the module docs: upstream's cast path leaves these
        // uninitialised, so there is nothing to widen.
        nearest_points: [Vector3::zeros(); 2],
    }
}
