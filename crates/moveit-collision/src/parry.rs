// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Behaviorally derived from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection_fcl/include/moveit/collision_detection_fcl/collision_common.hpp
//   moveit_core/collision_detection_fcl/src/collision_common.cpp
//   moveit_core/collision_detection_fcl/src/collision_env_fcl.cpp
//
// This is not a line-by-line port of any one upstream file: there is no
// `parry` backend upstream to port from, so this reproduces the FCL
// backend's *observable behavior* (the `collisionCallback`/`distanceCallback`
// algorithms, and which upstream request fields those two functions actually
// read) on top of `parry3d-f64` instead of FCL's own narrow phase.

//! A [`CollisionEnv`] backend for `moveit_state::RobotState`, built on
//! `parry3d-f64`.
//!
//! # Design: one globally-posed part per shape
//!
//! Upstream's `FCLObject` holds `std::vector<FCLCollisionObjectPtr>
//! collision_objects_` — one FCL `CollisionObject` **per shape**, each
//! carrying that shape's own global pose. `constructFCLObjectWorld` pushes
//! one per `Object::shapes_[i]` at `global_shape_poses_[i]`;
//! `constructFCLObjectRobot` pushes one per robot geometry at
//! `getCollisionBodyTransform(link, shape_index)`; and both
//! `checkRobotCollisionHelper` and `distanceRobotHelper` loop over that
//! vector, invoking the broadphase once per collision object. Nothing is
//! combined anywhere.
//!
//! [`PosedBody`] reproduces exactly that: [`PosedBody::parts`] is one
//! `(global pose, shape)` per shape, and a body-vs-body check is the cross
//! product of the two bodies' parts.
//!
//! Combining a body's shapes into a single `parry` shape instead would be
//! both a deviation and unsound: `parry` treats
//! [`parry3d_f64::shape::TriMesh`] as a composite shape, and
//! `Compound::new` panics (`"Nested composite shapes are not allowed"`) as
//! soon as one part is a mesh — which [`World::add_shapes_to_object`] makes
//! reachable from public API for any scene object carrying a mesh alongside
//! anything else.
//!
//! # Deviations from upstream
//!
//! 1. **`group_name` is inert.** Verified by reading `collision_env_fcl.cpp`
//!    in full: `checkSelfCollision`/`checkRobotCollision`/`distanceSelf`/
//!    `distanceRobot` never call `enableGroup`/read `active_components_only_`
//!    at all — that machinery is wired up only by the RobotModel-needing
//!    convenience overloads this crate's `env` module already declines to
//!    port (`distanceSelf(state)`, `distanceRobot(state, verbose)`, ...). So
//!    this backend does not filter by group either, matching upstream's real
//!    (if surprising) behavior rather than the narrower one a fresh
//!    implementation might guess at.
//! 2. **World objects are never padded or scaled.** Verified from
//!    `constructFCLObjectWorld` (calls the two-argument
//!    `createCollisionGeometry(shape, obj)` overload, no scale/padding) versus
//!    `constructFCLObjectRobot` (uses the padding/scale-taking overload via
//!    the cached `robot_geoms_`). [`LinkPaddingScale`] is consulted only when
//!    converting a [`moveit_model::LinkModel`]'s shapes, never a
//!    [`crate::World`] object's.
//! 3. **`CollisionRequest::pad_environment_collisions`/`pad_self_collisions`
//!    are not read.** Grepped all of `moveit_core/`: both fields are read
//!    only by `planning_scene.cpp` (out of this crate's scope), which
//!    switches between two whole `CollisionEnv` instances — one padded, one
//!    not — rather than either field ever reaching a `CollisionEnv` backend.
//!    Neither `collision_env_fcl.cpp` nor `collision_common.cpp` reference
//!    either field, so this backend always applies whatever
//!    [`LinkPaddingScale`] it was constructed with, matching the real FCL
//!    backend.
//! 4. **At most one [`Contact`] per *part* pair.**
//!    `parry3d_f64::query::contact` returns a single closest/deepest point
//!    per shape pair, where FCL's narrow phase can report several contact
//!    points for one object pair (e.g. mesh-mesh, or a box corner against a
//!    face). A body pair therefore yields at most `a.parts.len() *
//!    b.parts.len()` contacts here, against upstream's unbounded-per-pair
//!    narrow phase. [`CollisionRequest::max_contacts_per_pair`] is applied
//!    (see [`accumulate_collision`]) and does bind whenever either body
//!    carries several shapes.
//! 5. **Always a full contact query, never FCL's cheap boolean-only path.**
//!    `collisionCallback`'s `NEVER`/no-entry branch runs a cheap,
//!    contact-data-free `fcl::collide` once the storage budget
//!    (`max_contacts`) is exhausted, since only the `collision` flag is still
//!    needed at that point. This backend always calls
//!    `parry3d_f64::query::contact` (prediction `0.0`), which yields the
//!    collision flag *and* contact data in one call; the extra data is simply
//!    discarded once the budget is spent. Observably identical output, no
//!    optimization pass ported.
//! 6. **Signed distance and nearest points both come from one `contact` call.**
//!    Upstream's `distanceCallback` runs `fcl::distance` for the unsigned
//!    distance and nearest points, then — only when `enable_signed_distance`
//!    and the pair turns out to be touching or penetrating — re-runs
//!    `fcl::collide` (up to 200 contacts) and takes the *maximum* penetration
//!    depth across every contact found as the signed distance. This backend
//!    instead calls `parry3d_f64::query::contact` once per pair (see
//!    deviation 4: at most one contact exists here anyway), and reads
//!    `Contact::dist` directly as the signed distance, clamping it to `>= 0`
//!    when `enable_signed_distance` was not requested. `nearest_points` and
//!    `normal` are likewise read from that same call's `point1`/`point2`/
//!    `normal1` rather than a second FCL-specific query.
//! 7. **No early exit on `distanceSelf`/`distanceRobot`.** Upstream's
//!    `distanceCallback` sets `cdata->done = true` (stopping the broadphase
//!    traversal) as soon as a collision is confirmed and
//!    `enable_signed_distance` was not requested — which pairs end up in
//!    `DistanceResult::distances` after that point depends on FCL's
//!    broadphase (AABB tree) traversal order, which this port does not
//!    reproduce (there is no broadphase here at all; every ACM-permitted
//!    pair is evaluated in link/object order every time). This backend
//!    therefore always evaluates every pair exhaustively: `distances` here
//!    can be a superset of what a given upstream run would report, but
//!    `minimum_distance` and `collision` — the two fields every real caller
//!    actually reads — are order-independent and match either way.
//! 8. **Cylinder/Cone axis convention.** `moveit_geometry::Cylinder`/`Cone`
//!    are z-aligned (a cone's tip at `+z`); `parry3d_f64::shape::Cylinder`/
//!    `Cone` are always y-aligned (a cone's apex at `+y`, verified by reading
//!    `parry3d-f64`'s own `cone.rs`). [`axis_fix`] is the fixed +90°
//!    rotation about x that maps parry's `+y` onto moveit's `+z`, correcting
//!    both the axis and (for [`moveit_geometry::Cone`]) the apex direction in
//!    one rotation.
//! 9. **A degenerate [`moveit_geometry::Plane`] (`a = b = c = 0`) converts to
//!    no shape at all.** [`moveit_geometry::Plane::new`] does not validate
//!    its coefficients (an infinite plane has no notion of a negative
//!    dimension to reject), so this case is reachable; a plane with no normal
//!    has no well-defined half-space to build, so [`convert_shape`] excludes
//!    it from collision geometry rather than construct a `HalfSpace` with a
//!    zero-length (and therefore un-normalizable) normal.
//! 10. **[`moveit_geometry::Shape::OcTree`] converts to no shape at all.**
//!     This port's [`moveit_geometry::OcTree`] carries no tree payload (see
//!     its own doc comment: no mature Rust `octomap` binding exists yet), so
//!     there is no geometry here to convert regardless of backend — the same
//!     gap the FCL backend would face if this crate ever gained one.
//! 11. **`check_robot_collision_continuous` returns [`Error`].** See
//!     [`crate::CollisionEnv::check_robot_collision_continuous`]'s own doc:
//!     upstream's FCL backend does not implement this case either, silently
//!     leaving `res` untouched; this backend has no swept/conservative-
//!     advancement query wired up, and returns an explicit error rather than
//!     an approximation that would misreport a real path collision as clear.

use std::collections::BTreeMap;

use moveit_error::{Error, Result};
use moveit_geometry::{Isometry3, Shape, Vector3};
use moveit_model::LinkModel;
use moveit_state::Posed;

use parry3d_f64::math::{Pose, Vector as ParryVector};
use parry3d_f64::query::{self, Contact as ParryContact};
use parry3d_f64::shape::{
    Ball, Cone as ParryCone, Cuboid as ParryCuboid, Cylinder as ParryCylinder, HalfSpace,
    SharedShape, TriMesh,
};

use crate::common::{
    BodyType, CollisionRequest, CollisionResult, Contact, ContactData, DistanceRequest,
    DistanceRequestType, DistanceResult, DistanceResultsData,
};
use crate::env::{CollisionEnv, LinkPaddingScale};
use crate::matrix::{AllowedCollision, AllowedCollisionMatrix};
use crate::world::{Object, World};

/// A practical, effectively-unbounded search distance for `parry`'s own
/// prediction-margin arithmetic. Passing `f64::MAX` directly (upstream's own
/// default [`DistanceRequest::distance_threshold`]) risks overflowing to
/// `+inf` inside `parry`'s internal AABB-inflation math, since a finite AABB
/// corner offset by `f64::MAX` overflows under IEEE 754 addition. No real
/// robot geometry is within a million metres of itself, so clamping the
/// value actually sent to `parry` here changes no reachable behavior; the
/// logical threshold used for accumulation and the strict-`<` boundary check
/// is never clamped, only the query's own prediction argument.
const EFFECTIVELY_UNBOUNDED: f64 = 1.0e6;

/// Clamps a logical threshold down to a prediction margin `parry` will
/// accept. Upstream's threshold is a *signed* value under
/// [`DistanceRequest::enable_signed_distance`] and
/// [`DistanceRequestType::Global`]'s running `minimum_distance` accumulator
/// (see [`accumulate_distance`]): once one penetrating pair has been found,
/// every later pair's threshold argument is that penetration's negative
/// depth, not a search radius. `parry3d_f64::bounding_volume::Aabb::loosened`
/// panics on a negative margin ("The loosening margin must be
/// non-negative"), so the lower bound must be clamped here too, not only the
/// upper one -- a negative margin is otherwise reachable the first time a
/// deeply-penetrating pair updates the accumulator before every pair has been
/// visited. Clamping to `0.0` (rather than leaving the query out entirely)
/// still finds any pair at least as penetrating, matching
/// [`accumulate_collision`]'s own prediction-`0.0` convention for a
/// touching-or-penetrating-only query.
fn bounded_prediction(threshold: f64) -> f64 {
    threshold.clamp(0.0, EFFECTIVELY_UNBOUNDED)
}

/// The fixed rotation that maps `parry3d_f64`'s y-aligned `Cylinder`/`Cone`
/// convention onto `moveit_geometry`'s z-aligned one: +90° about x sends
/// local `(0, 1, 0)` to `(0, 0, 1)`, fixing the axis for both shapes and (for
/// `Cone`) the apex direction (parry: apex at `+y`; moveit: tip at `+z`) in
/// the same rotation. See the module doc, deviation 8.
fn axis_fix() -> Isometry3 {
    Isometry3::rotation(Vector3::x() * std::f64::consts::FRAC_PI_2)
}

fn to_pose(iso: Isometry3) -> Pose {
    iso.into()
}

fn from_parry_vector(v: ParryVector) -> Vector3 {
    Vector3::new(v.x, v.y, v.z)
}

/// Convert one [`Shape`] into a `parry` shape, plus the extra local
/// transform (identity, [`axis_fix`], or a plane offset) needed to align
/// `parry`'s axis convention with upstream's. `None` for
/// [`Shape::OcTree`] and a degenerate [`Shape::Plane`] — see the module doc,
/// deviations 9 and 10.
fn convert_shape(shape: &Shape) -> Option<(SharedShape, Isometry3)> {
    match shape {
        Shape::Sphere(s) => Some((SharedShape::new(Ball::new(s.radius)), Isometry3::identity())),
        Shape::Cylinder(c) => Some((
            SharedShape::new(ParryCylinder::new(c.length * 0.5, c.radius)),
            axis_fix(),
        )),
        Shape::Cone(c) => Some((
            SharedShape::new(ParryCone::new(c.length * 0.5, c.radius)),
            axis_fix(),
        )),
        Shape::Cuboid(b) => Some((
            SharedShape::new(ParryCuboid::new(ParryVector::new(
                b.size[0] * 0.5,
                b.size[1] * 0.5,
                b.size[2] * 0.5,
            ))),
            Isometry3::identity(),
        )),
        Shape::Plane(p) => {
            let magnitude = (p.a * p.a + p.b * p.b + p.c * p.c).sqrt();
            if magnitude == 0.0 {
                return None;
            }
            let normal = ParryVector::new(p.a / magnitude, p.b / magnitude, p.c / magnitude);
            // p*n_hat, the plane's signed offset along its own unit normal:
            // p = -d/|n|, n_hat = n/|n|, so p*n_hat = -d*n/|n|^2.
            let offset = -p.d / (magnitude * magnitude);
            let translation = Vector3::new(p.a, p.b, p.c) * offset;
            Some((
                SharedShape::new(HalfSpace::new(normal)),
                Isometry3::translation(translation.x, translation.y, translation.z),
            ))
        }
        Shape::Mesh(m) => {
            let vertices = m
                .vertices
                .iter()
                .map(|v| ParryVector::new(v.x, v.y, v.z))
                .collect();
            TriMesh::new(vertices, m.triangles.clone())
                .ok()
                .map(|mesh| (SharedShape::new(mesh), Isometry3::identity()))
        }
        Shape::OcTree(_) => None,
    }
}

/// [`Shape::scale_and_padd`] on a clone, for a robot link's own collision
/// shape (never a world object's — see the module doc, deviation 2).
///
/// # Panics
///
/// Never, in practice: [`LinkModel::shapes`] can never contain
/// [`Shape::Mesh`] (`LinkModel`'s own doc comment: mesh collision geometry is
/// skipped entirely by the URDF loader, never stored), and every other shape
/// variant's dimensions are already validated non-negative at construction —
/// scaling by a validated-positive [`LinkPaddingScale::link_scale`] and
/// adding a validated-non-negative [`LinkPaddingScale::link_padding`] can
/// never make them negative. [`Shape::scale_and_padd`] can only fail for
/// those two reasons, both unreachable here.
fn scaled_padded_shape(shape: &Shape, scale: f64, padding: f64) -> Shape {
    let mut shape = shape.clone();
    shape.scale_and_padd(scale, padding).expect(
        "robot link collision shapes are never Shape::Mesh and are already non-negative, \
         so scale_and_padd cannot fail here",
    );
    shape
}

/// One named collision body — a robot link or a world object — as the list
/// of globally-posed shapes upstream's `FCLObject::collision_objects_` holds
/// for it. See the module doc.
struct PosedBody {
    name: String,
    body_type: BodyType,
    /// One `(global pose, shape)` per shape, upstream's
    /// `global_shape_poses_[i]` / `getCollisionBodyTransform(link, i)`.
    /// Never empty: [`pose_parts`] returns `None` rather than build a body
    /// with nothing to check.
    parts: Vec<(Pose, SharedShape)>,
}

/// Compose each shape part's body-relative pose with the body's own `pose`,
/// yielding the global poses [`PosedBody::parts`] stores. `None` if `parts`
/// is empty, so a body with no convertible geometry is dropped rather than
/// carried as an empty one.
fn pose_parts(
    parts: Vec<(Pose, SharedShape)>,
    pose: Isometry3,
) -> Option<Vec<(Pose, SharedShape)>> {
    if parts.is_empty() {
        return None;
    }
    let body_pose = to_pose(pose);
    Some(
        parts
            .into_iter()
            .map(|(part_pose, shape)| (body_pose * part_pose, shape))
            .collect(),
    )
}

/// One robot link's [`PosedBody`], scaled and padded per `padding_scale`.
/// `None` if the link has no (convertible) collision geometry at all —
/// matching upstream's `getLinkModelsWithCollisionGeometry()` filter, which
/// this crate's [`LinkPaddingScale`] doc already reproduces for the
/// padding/scale bookkeeping itself.
fn link_body(
    link: &LinkModel,
    pose: Isometry3,
    padding_scale: &LinkPaddingScale,
) -> Option<PosedBody> {
    let scale = padding_scale.link_scale(link.name());
    let padding = padding_scale.link_padding(link.name());
    let parts: Vec<(Pose, SharedShape)> = link
        .shapes()
        .iter()
        .filter_map(|link_shape| {
            let shape = scaled_padded_shape(&link_shape.shape, scale, padding);
            let (parry_shape, extra) = convert_shape(&shape)?;
            Some((to_pose(link_shape.origin_transform * extra), parry_shape))
        })
        .collect();
    Some(PosedBody {
        name: link.name().to_owned(),
        body_type: BodyType::RobotLink,
        parts: pose_parts(parts, pose)?,
    })
}

/// One world object's [`PosedBody`], unscaled and unpadded (module doc,
/// deviation 2). `None` if the object has no (convertible) shapes.
fn object_body(id: &str, object: &Object) -> Option<PosedBody> {
    let parts: Vec<(Pose, SharedShape)> = object
        .shapes()
        .iter()
        .filter_map(|entry| {
            let (parry_shape, extra) = convert_shape(entry.shape())?;
            Some((to_pose(entry.pose() * extra), parry_shape))
        })
        .collect();
    Some(PosedBody {
        name: id.to_owned(),
        body_type: BodyType::WorldObject,
        parts: pose_parts(parts, object.pose())?,
    })
}

fn robot_bodies(state: &Posed<'_, '_>, padding_scale: &LinkPaddingScale) -> Vec<PosedBody> {
    state
        .model()
        .link_models()
        .iter()
        .filter_map(|link| {
            let pose = state.global_link_transform_at(link.link_index());
            link_body(link, pose, padding_scale)
        })
        .collect()
}

fn world_bodies(world: &World) -> Vec<PosedBody> {
    world
        .iter()
        .filter_map(|(id, object)| object_body(id, object))
        .collect()
}

/// Every unordered pair among `bodies` (`i < j`), for self-collision.
fn self_pairs(bodies: &[PosedBody]) -> impl Iterator<Item = (&PosedBody, &PosedBody)> {
    (0..bodies.len())
        .flat_map(move |i| (i + 1..bodies.len()).map(move |j| (i, j)))
        .map(move |(i, j)| (&bodies[i], &bodies[j]))
}

/// The full cross product of `a` and `b`, for robot-vs-world checks.
fn cross_pairs<'a>(
    a: &'a [PosedBody],
    b: &'a [PosedBody],
) -> impl Iterator<Item = (&'a PosedBody, &'a PosedBody)> {
    a.iter().flat_map(move |x| b.iter().map(move |y| (x, y)))
}

/// Every `(pose, shape)` combination of two bodies' parts — the cross product
/// upstream gets for free by registering each body's collision objects
/// individually with the broadphase manager (`FCLObject::registerTo`) and
/// then calling `collide`/`distance` once per collision object.
fn part_pairs<'a>(
    a: &'a PosedBody,
    b: &'a PosedBody,
) -> impl Iterator<
    Item = (
        &'a Pose,
        &'a dyn parry3d_f64::shape::Shape,
        &'a Pose,
        &'a dyn parry3d_f64::shape::Shape,
    ),
> {
    a.parts.iter().flat_map(move |(a_pose, a_shape)| {
        b.parts
            .iter()
            .map(move |(b_pose, b_shape)| (a_pose, a_shape.as_ref(), b_pose, b_shape.as_ref()))
    })
}

/// `fcl2contact`, adapted from `parry3d_f64::query::Contact`'s fields:
/// `pos` is the midpoint of the two surface points (upstream's own `pos`
/// comes from FCL-internal contact geometry with no documented meaning
/// beyond "contact position"; a midpoint is a reasoned, defensible stand-in),
/// `normal` is `normal1` ("points from shape 1 toward shape 2", matching
/// upstream's own convention of a normal pointing from the first body to the
/// second), `depth` is `-dist` clamped to `>= 0`, and `nearest_points` is
/// `[point1, point2]` — populated here even though upstream's own
/// `fcl2contact` leaves `Contact::nearest_points` untouched for a narrow-
/// phase contact (only `DistanceResultsData::nearest_points` is ever set
/// upstream); `parry` gives us both points for free from the same query, so
/// this is a strict improvement over upstream's indeterminate field, not a
/// behavior this crate's tests can observe upstream ever relying on.
fn to_contact(
    pc: &ParryContact,
    name1: &str,
    type1: BodyType,
    name2: &str,
    type2: BodyType,
) -> Contact {
    let point1 = from_parry_vector(pc.point1);
    let point2 = from_parry_vector(pc.point2);
    Contact {
        pos: (point1 + point2) * 0.5,
        normal: from_parry_vector(pc.normal1),
        depth: (-pc.dist).max(0.0),
        body_name_1: name1.to_owned(),
        body_type_1: type1,
        body_name_2: name2.to_owned(),
        body_type_2: type2,
        percent_interpolation: 0.0,
        nearest_points: [point1, point2],
    }
}

/// `collisionCallback`'s per-pair algorithm (see the module doc, deviations
/// 4 and 5), folded over every candidate pair:
///
/// - [`AllowedCollision::Always`]: skip the pair, no query at all.
/// - Real contact found (`parry3d_f64::query::contact`, prediction `0.0`,
///   so only touching/penetrating pairs yield `Some`) and
///   [`AllowedCollision::Conditional`]: the predicate decides — rejected
///   (`false`) is a collision, accepted (`true`) is silently not.
/// - Real contact found and [`AllowedCollision::Never`] or no entry:
///   unconditionally a collision.
/// - `Err`/`None` from the query (no contact, or an unsupported shape-pair
///   combination — unreachable for the `Ball`/`Cuboid`/`Cylinder`/`Cone`/
///   `HalfSpace`/`TriMesh` compounds this backend builds, all pairwise-
///   supported by `parry`'s default query dispatcher): not a collision.
///
/// The `collision` flag is set independent of the storage budget
/// (`request.max_contacts`, `request.max_contacts_per_pair`) for every pair,
/// matching upstream's own invariant.
fn accumulate_collision<'a>(
    pairs: impl Iterator<Item = (&'a PosedBody, &'a PosedBody)>,
    request: &CollisionRequest,
    acm: Option<&AllowedCollisionMatrix>,
) -> CollisionResult {
    let mut collision = false;
    let mut by_pair: BTreeMap<(String, String), Vec<Contact>> = BTreeMap::new();
    let mut stored_total = 0usize;
    for (a, b) in pairs {
        let allowed = acm.and_then(|m| m.allowed_collision(&a.name, &b.name));
        if matches!(allowed, Some(AllowedCollision::Always)) {
            continue;
        }
        for (a_pose, a_shape, b_pose, b_shape) in part_pairs(a, b) {
            let Ok(Some(contact)) = query::contact(a_pose, a_shape, b_pose, b_shape, 0.0) else {
                continue;
            };
            let mut c = to_contact(&contact, &a.name, a.body_type, &b.name, b.body_type);
            let is_collision = match allowed {
                Some(AllowedCollision::Conditional(ref predicate)) => !predicate(&mut c),
                Some(AllowedCollision::Never) | None => true,
                Some(AllowedCollision::Always) => unreachable!("filtered out above"),
            };
            if !is_collision {
                continue;
            }
            collision = true;
            if request.contacts && stored_total < request.max_contacts {
                let bucket = by_pair.entry((a.name.clone(), b.name.clone())).or_default();
                if bucket.len() < request.max_contacts_per_pair {
                    bucket.push(c);
                    stored_total += 1;
                }
            }
        }
    }
    CollisionResult {
        collision,
        distance: None,
        contacts: request.contacts.then_some(ContactData { by_pair }),
        cost_sources: None,
    }
}

/// `distanceCallback`'s per-pair algorithm (see the module doc, deviations 6
/// and 7): [`AllowedCollision::Always`] skips the pair (upstream:
/// `always_allow_collision`, the *only* ACM outcome the distance callback
/// checks — `Never`/`Conditional` are not special-cased for a distance
/// query); otherwise the pair's distance is computed and folded into
/// `minimum_distance` and (for every [`DistanceRequestType`] but `Global`)
/// `distances`, per upstream's exact accumulation rule for that type.
fn accumulate_distance<'a>(
    pairs: impl Iterator<Item = (&'a PosedBody, &'a PosedBody)>,
    request: &DistanceRequest<'_>,
) -> DistanceResult {
    let mut result = DistanceResult::default();
    for (a, b) in pairs {
        if let Some(acm) = request.acm
            && matches!(
                acm.allowed_collision(&a.name, &b.name),
                Some(AllowedCollision::Always)
            )
        {
            continue;
        }
        let key = (a.name.clone(), b.name.clone());
        // Per *part* pair, not per body pair: each of upstream's collision
        // objects reaches `distanceCallback` as its own invocation, which
        // re-reads the running `minimum_distance`/`distances` state to pick
        // its threshold. Recomputing inside the loop is what reproduces that.
        for (a_pose, a_shape, b_pose, b_shape) in part_pairs(a, b) {
            let threshold = match request.request_type {
                DistanceRequestType::Global => result.minimum_distance.distance,
                DistanceRequestType::Limited => {
                    if result
                        .distances
                        .get(&key)
                        .is_some_and(|existing| existing.len() >= request.max_contacts_per_body)
                    {
                        continue;
                    }
                    request.distance_threshold
                }
                DistanceRequestType::Single => result
                    .distances
                    .get(&key)
                    .map_or(request.distance_threshold, |existing| existing[0].distance),
                DistanceRequestType::All => request.distance_threshold,
            };
            let Ok(Some(contact)) = query::contact(
                a_pose,
                a_shape,
                b_pose,
                b_shape,
                bounded_prediction(threshold),
            ) else {
                continue;
            };
            if contact.dist >= threshold {
                continue;
            }
            let distance_value = if request.enable_signed_distance {
                contact.dist
            } else {
                contact.dist.max(0.0)
            };
            let mut data = DistanceResultsData {
                distance: distance_value,
                nearest_points: [Vector3::zeros(); 2],
                link_names: [a.name.clone(), b.name.clone()],
                body_types: [a.body_type, b.body_type],
                normal: Vector3::zeros(),
            };
            if request.enable_nearest_points {
                if distance_value <= 0.0 {
                    let p = from_parry_vector(contact.point1);
                    data.nearest_points = [p, p];
                } else {
                    let p1 = from_parry_vector(contact.point1);
                    let p2 = from_parry_vector(contact.point2);
                    data.normal = (p2 - p1).normalize();
                    data.nearest_points = [p1, p2];
                }
            }
            if data.distance < result.minimum_distance.distance {
                result.minimum_distance = data.clone();
            }
            if data.distance <= 0.0 {
                result.collision = true;
            }
            match request.request_type {
                DistanceRequestType::Global => {}
                DistanceRequestType::All | DistanceRequestType::Limited => {
                    result.distances.entry(key.clone()).or_default().push(data);
                }
                DistanceRequestType::Single => {
                    let bucket = result.distances.entry(key.clone()).or_default();
                    if bucket.is_empty() {
                        bucket.push(data);
                    } else if data.distance < bucket[0].distance {
                        bucket[0] = data;
                    }
                }
            }
        }
    }
    result
}

/// A [`CollisionEnv`] backend for `moveit_state::RobotState`
/// (`Posed<'s, 'm>`), over `parry3d-f64`. See the module doc for scope and
/// deviations from upstream's FCL backend.
#[derive(Debug, Clone, Default)]
pub struct ParryCollisionEnv {
    world: World,
    padding_scale: LinkPaddingScale,
}

impl ParryCollisionEnv {
    /// Build a backend over `world`, applying `padding_scale` to robot links
    /// only (module doc, deviation 2).
    pub fn new(world: World, padding_scale: LinkPaddingScale) -> Self {
        Self {
            world,
            padding_scale,
        }
    }

    /// The collision world this backend checks the robot against.
    pub fn world(&self) -> &World {
        &self.world
    }

    /// Mutable access to the collision world.
    pub fn world_mut(&mut self) -> &mut World {
        &mut self.world
    }

    /// The per-link padding/scale this backend applies to robot geometry.
    pub fn padding_scale(&self) -> &LinkPaddingScale {
        &self.padding_scale
    }

    /// Mutable access to the per-link padding/scale.
    pub fn padding_scale_mut(&mut self) -> &mut LinkPaddingScale {
        &mut self.padding_scale
    }
}

impl<'s, 'm> CollisionEnv<Posed<'s, 'm>> for ParryCollisionEnv {
    fn check_self_collision(
        &self,
        request: &CollisionRequest,
        state: &Posed<'s, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
    ) -> CollisionResult {
        let bodies = robot_bodies(state, &self.padding_scale);
        accumulate_collision(self_pairs(&bodies), request, acm)
    }

    fn check_robot_collision(
        &self,
        request: &CollisionRequest,
        state: &Posed<'s, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
    ) -> CollisionResult {
        let robot = robot_bodies(state, &self.padding_scale);
        let world = world_bodies(&self.world);
        accumulate_collision(cross_pairs(&robot, &world), request, acm)
    }

    fn check_robot_collision_continuous(
        &self,
        _request: &CollisionRequest,
        _state1: &Posed<'s, 'm>,
        _state2: &Posed<'s, 'm>,
        _acm: Option<&AllowedCollisionMatrix>,
    ) -> Result<CollisionResult> {
        Err(Error::other(
            "continuous robot-collision checking is not implemented by ParryCollisionEnv: no \
             swept/conservative-advancement query is wired up, and approximating it (e.g. \
             sampling the path, or only checking the end state) would silently misreport a real \
             path collision as clear",
        ))
    }

    fn distance_self(
        &self,
        request: &DistanceRequest<'_>,
        state: &Posed<'s, 'm>,
    ) -> DistanceResult {
        let bodies = robot_bodies(state, &self.padding_scale);
        accumulate_distance(self_pairs(&bodies), request)
    }

    fn distance_robot(
        &self,
        request: &DistanceRequest<'_>,
        state: &Posed<'s, 'm>,
    ) -> DistanceResult {
        let robot = robot_bodies(state, &self.padding_scale);
        let world = world_bodies(&self.world);
        accumulate_distance(cross_pairs(&robot, &world), request)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use approx::assert_relative_eq;
    use moveit_geometry::{Cuboid, OcTree, Plane, Shape, Sphere};
    use moveit_model::RobotModel;
    use moveit_srdf::SrdfModel;
    use moveit_state::RobotState;

    use super::*;

    // Geometry-level tests: `convert_shape`, `axis_fix`, `to_contact`.

    #[test]
    fn convert_shape_sphere_is_a_ball_at_the_origin() {
        let (_shape, extra) = convert_shape(&Shape::Sphere(Sphere::new(2.0).unwrap())).unwrap();
        assert_eq!(extra, Isometry3::identity());
    }

    #[test]
    fn convert_shape_degenerate_plane_is_excluded() {
        let plane = Shape::Plane(Plane::new(0.0, 0.0, 0.0, 1.0));
        assert!(convert_shape(&plane).is_none());
    }

    #[test]
    fn convert_shape_plane_offset_matches_hesse_normal_form() {
        // x = 3 (a=1, b=0, c=0, d=-3): signed offset from the origin along
        // the unit normal (1, 0, 0) is 3.
        let plane = Shape::Plane(Plane::new(1.0, 0.0, 0.0, -3.0));
        let (_shape, extra) = convert_shape(&plane).unwrap();
        assert_relative_eq!(extra.translation.vector.x, 3.0, epsilon = 1e-12);
        assert_relative_eq!(extra.translation.vector.y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(extra.translation.vector.z, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn convert_shape_octree_is_excluded() {
        assert!(convert_shape(&Shape::OcTree(OcTree::new())).is_none());
    }

    #[test]
    fn axis_fix_maps_parry_y_up_onto_moveit_z_up() {
        let fixed = axis_fix() * Vector3::new(0.0, 1.0, 0.0);
        assert_relative_eq!(fixed.x, 0.0, epsilon = 1e-12);
        assert_relative_eq!(fixed.y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(fixed.z, 1.0, epsilon = 1e-12);
    }

    #[test]
    fn to_contact_maps_fields_per_fcl2contact_convention() {
        let pc = ParryContact::new(
            ParryVector::new(0.0, 0.0, 0.0),
            ParryVector::new(1.0, 0.0, 0.0),
            ParryVector::new(1.0, 0.0, 0.0),
            ParryVector::new(-1.0, 0.0, 0.0),
            -0.5,
        );
        let c = to_contact(&pc, "a", BodyType::RobotLink, "b", BodyType::WorldObject);
        assert_relative_eq!(c.pos, Vector3::new(0.5, 0.0, 0.0));
        assert_relative_eq!(c.normal, Vector3::new(1.0, 0.0, 0.0));
        assert_relative_eq!(c.depth, 0.5);
        assert_eq!(c.body_name_1, "a");
        assert_eq!(c.body_type_1, BodyType::RobotLink);
        assert_eq!(c.body_name_2, "b");
        assert_eq!(c.body_type_2, BodyType::WorldObject);
        assert_eq!(c.percent_interpolation, 0.0);
        assert_eq!(c.nearest_points[0], Vector3::new(0.0, 0.0, 0.0));
        assert_eq!(c.nearest_points[1], Vector3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn scaled_padded_shape_grows_a_cuboid_by_scale_then_padding() {
        // Upstream applies scale before padding; a unit half-extent scaled
        // by 2 then padded by 0.5 is 2.5, not (1 + 0.5) * 2 = 3.
        let shape = Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0).unwrap());
        let scaled = scaled_padded_shape(&shape, 2.0, 0.5);
        match scaled {
            Shape::Cuboid(c) => assert_relative_eq!(c.size[0], 5.0),
            other => panic!("expected Cuboid, got {other:?}"),
        }
    }

    // Fixture: a fixed-base robot with a shapeless `base` link and two
    // independent floating-joint children `p`/`q`, each a 1x1x1 box, so each
    // can be posed to an arbitrary independent global transform via
    // `RobotState::set_joint_transform`. Mirrors the fixture pattern already
    // established in `moveit_model::robot_model`'s own test module.

    const FIXED_BASE_SRDF: &str = r#"<robot name="test">
        <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
    </robot>"#;

    fn box_link(name: &str) -> String {
        format!(
            r#"<link name="{name}">
                <collision><geometry><box size="1 1 1"/></geometry></collision>
            </link>"#
        )
    }

    fn floating_joint(name: &str, parent: &str, child: &str) -> String {
        format!(
            r#"<joint name="{name}" type="floating">
                <parent link="{parent}"/>
                <child link="{child}"/>
            </joint>"#
        )
    }

    fn build_model(link_names: &[&str]) -> RobotModel {
        let links_and_joints: String = link_names
            .iter()
            .map(|name| {
                format!(
                    "{}{}",
                    box_link(name),
                    floating_joint(&format!("joint_{name}"), "base", name)
                )
            })
            .collect();
        let urdf_xml =
            format!(r#"<robot name="test"><link name="base"/>{links_and_joints}</robot>"#);
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("test URDF must parse");
        let srdf = SrdfModel::parse_str(FIXED_BASE_SRDF).expect("test SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf)
            .expect("test fixture model must build")
    }

    fn state_with_links_at<'m>(
        model: &'m RobotModel,
        poses: &[(&str, Isometry3)],
    ) -> RobotState<'m> {
        let mut state = RobotState::new(model);
        for (link, pose) in poses {
            state
                .set_joint_transform(&format!("joint_{link}"), pose)
                .expect("floating joint transform must set");
        }
        state
    }

    #[test]
    fn check_self_collision_detects_overlapping_boxes() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();

        let result = env.check_self_collision(&CollisionRequest::default(), &posed, None);

        assert!(result.collision);
    }

    #[test]
    fn check_self_collision_reports_free_when_boxes_are_apart() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(5.0, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();

        let result = env.check_self_collision(&CollisionRequest::default(), &posed, None);

        assert!(!result.collision);
    }

    #[test]
    fn check_self_collision_always_entry_suppresses_an_otherwise_colliding_pair() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("p", "q", true);

        let result = env.check_self_collision(&CollisionRequest::default(), &posed, Some(&acm));

        assert!(!result.collision);
    }

    #[test]
    fn check_self_collision_conditional_entry_predicate_decides() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_conditional_entry("p", "q", Arc::new(|_: &mut Contact| true));

        let result = env.check_self_collision(&CollisionRequest::default(), &posed, Some(&acm));

        assert!(!result.collision);
    }

    #[test]
    fn check_self_collision_conditional_entry_predicate_can_still_report_collision() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_conditional_entry("p", "q", Arc::new(|_: &mut Contact| false));

        let result = env.check_self_collision(&CollisionRequest::default(), &posed, Some(&acm));

        assert!(result.collision);
    }

    #[test]
    fn check_self_collision_remove_entry_restores_default_behavior() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("p", "q", true);
        acm.remove_entry("p", "q");

        let result = env.check_self_collision(&CollisionRequest::default(), &posed, Some(&acm));

        assert!(result.collision);
    }

    #[test]
    fn check_self_collision_max_contacts_budget_caps_stored_contacts_across_pairs() {
        // p, q, r all mutually overlapping at the origin: 3 colliding pairs.
        let model = build_model(&["p", "q", "r"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::identity()),
                ("r", Isometry3::identity()),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = CollisionRequest {
            contacts: true,
            max_contacts: 2,
            max_contacts_per_pair: 1,
            ..CollisionRequest::default()
        };

        let result = env.check_self_collision(&request, &posed, None);

        assert!(result.collision);
        let stored: usize = result.contacts.expect("contacts requested").count();
        assert_eq!(stored, 2);
    }

    #[test]
    fn check_self_collision_still_reports_collision_with_a_spent_contact_budget() {
        // `max_contacts: 0` is not a hypothetical. `CollisionEnv::check_collision`
        // subtracts the self-check's contact count from the request before
        // calling the robot check (PORTING-PLAN.md 10.5), saturating at zero, so
        // a self-check that fills the budget hands the robot check exactly this
        // request. The budget governs how many contacts are *stored*; it must
        // never govern whether a collision is *found*. A backend that folded the
        // two together would report a clear scene for the overlapping pair here,
        // and the caller has no way to tell that apart from a genuinely clear one.
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[("p", Isometry3::identity()), ("q", Isometry3::identity())],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = CollisionRequest {
            contacts: true,
            max_contacts: 0,
            ..CollisionRequest::default()
        };

        let result = env.check_self_collision(&request, &posed, None);

        assert!(
            result.collision,
            "a spent contact budget must not suppress the collision flag"
        );
        assert_eq!(
            result.contacts.expect("contacts requested").count(),
            0,
            "a spent contact budget must store nothing"
        );
    }

    #[test]
    fn check_robot_collision_detects_overlap_with_a_world_object() {
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::identity(),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let result = env.check_robot_collision(&CollisionRequest::default(), &posed, None);

        assert!(result.collision);
    }

    #[test]
    fn check_robot_collision_reports_free_when_world_object_is_far_away() {
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::translation(10.0, 0.0, 0.0),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let result = env.check_robot_collision(&CollisionRequest::default(), &posed, None);

        assert!(!result.collision);
    }

    #[test]
    fn distance_self_reports_the_gap_between_separated_boxes() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(2.0, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();

        let result = env.distance_self(&DistanceRequest::default(), &posed);

        assert!(!result.collision);
        assert_relative_eq!(result.minimum_distance.distance, 1.0, epsilon = 1e-9);
    }

    #[test]
    fn distance_self_clamps_to_zero_without_signed_distance() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = DistanceRequest {
            enable_signed_distance: false,
            ..DistanceRequest::default()
        };

        let result = env.distance_self(&request, &posed);

        assert!(result.collision);
        assert_relative_eq!(result.minimum_distance.distance, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn distance_self_reports_negative_penetration_with_signed_distance_enabled() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = DistanceRequest {
            enable_signed_distance: true,
            ..DistanceRequest::default()
        };

        let result = env.distance_self(&request, &posed);

        assert!(result.collision);
        assert_relative_eq!(result.minimum_distance.distance, -0.5, epsilon = 1e-9);
    }

    #[test]
    fn distance_self_does_not_panic_when_an_earlier_pair_deeply_penetrates() {
        // p, q, r all identically posed: three mutually, deeply overlapping
        // pairs. `DistanceRequestType::Global` (the default) folds every
        // pair's threshold into the running `minimum_distance`, so whichever
        // pair is visited first drives it deeply negative; every later pair
        // must still be queryable rather than handed that negative value as
        // `parry`'s prediction margin (`bounded_prediction` used to pass it
        // through unclamped on the low end, and `parry` panics on a negative
        // margin).
        let model = build_model(&["p", "q", "r"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::identity()),
                ("r", Isometry3::identity()),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = DistanceRequest {
            enable_signed_distance: true,
            ..DistanceRequest::default()
        };

        let result = env.distance_self(&request, &posed);

        assert!(result.collision);
        assert_relative_eq!(result.minimum_distance.distance, -1.0, epsilon = 1e-9);
    }

    #[test]
    fn distance_self_always_entry_skips_the_pair_entirely() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("p", "q", true);
        let request = DistanceRequest {
            acm: Some(&acm),
            ..DistanceRequest::default()
        };

        let result = env.distance_self(&request, &posed);

        assert!(!result.collision);
        assert_eq!(result.minimum_distance.distance, f64::MAX);
    }

    #[test]
    fn distance_self_never_entry_has_no_effect_unlike_collision_checking() {
        // Unlike `check_self_collision`, `Never`/`Conditional` ACM entries
        // have no effect on a distance query at all (module doc, deviation
        // 6): only `Always` skips a pair.
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("p", "q", false);
        let request = DistanceRequest {
            acm: Some(&acm),
            enable_signed_distance: true,
            ..DistanceRequest::default()
        };

        let result = env.distance_self(&request, &posed);

        assert!(result.collision);
        assert_relative_eq!(result.minimum_distance.distance, -0.5, epsilon = 1e-9);
    }

    #[test]
    fn distance_robot_reports_the_gap_to_a_world_object() {
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::translation(2.0, 0.0, 0.0),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let result = env.distance_robot(&DistanceRequest::default(), &posed);

        assert!(!result.collision);
        assert_relative_eq!(result.minimum_distance.distance, 1.0, epsilon = 1e-9);
    }

    #[test]
    fn check_robot_collision_continuous_returns_an_error_rather_than_approximating() {
        let model = build_model(&["p"]);
        let mut state1 = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let mut state2 =
            state_with_links_at(&model, &[("p", Isometry3::translation(1.0, 0.0, 0.0))]);
        let posed1 = state1.update();
        let posed2 = state2.update();
        let env = ParryCollisionEnv::default();

        let result = env.check_robot_collision_continuous(
            &CollisionRequest::default(),
            &posed1,
            &posed2,
            None,
        );

        assert!(result.is_err());
    }
}
