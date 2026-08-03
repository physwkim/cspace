// Copyright (c) 2010, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_distance_field/include/moveit/collision_distance_field/collision_distance_field_types.hpp
//   moveit_core/collision_distance_field/src/collision_distance_field_types.cpp

//! Sphere/point decomposition of a body, plus a posed wrapper around
//! [`PropagationDistanceField`], for the collision-distance-field collision
//! checker.
//!
//! # Scope
//!
//! This is the first slice of `moveit_core/collision_distance_field`
//! (PORTING-PLAN.md §5 Phase 3): `collision_distance_field_types.hpp`/`.cpp`
//! only, taken first because -- unlike the rest of that directory -- it has
//! no `RobotState`/`RobotModel` dependency at all, so it belongs here rather
//! than waiting on the model-porting work. The collision environment itself
//! (`CollisionEnvDistanceField`, which uses these types against a
//! `RobotModel`) is not ported.
//!
//! Not ported from this pair of files:
//!
//! - `PosedBodyPointDecomposition`'s `octomap::OcTree` constructor: no
//!   `octomap` binding exists in this workspace (PORTING-PLAN.md's own gap
//!   analysis flags `octomap` as "성숙도 미달" / not mature enough, pending a
//!   Phase 3 evaluation of a from-scratch implementation) and this crate owns
//!   none of that design, so it cannot be invented here without guessing at
//!   a design another phase owns -- the same reasoning [`DistanceField`]'s
//!   module doc already applies to `addOcTreeToField`.
//! - `getCollisionSphereMarkers`, `getProximityGradientMarkers`,
//!   `getCollisionMarkers`: build `visualization_msgs::msg::MarkerArray` for
//!   RViz. PORTING-PLAN.md D1 keeps ROS message types out of every crate but
//!   the optional `moveit-ros`.
//! - `BodyDecompositionVector`: declared upstream only as a forward
//!   declaration for friending (`class BodyDecompositionVector;`, "forward
//!   declaration required for friending apparently") -- grepping the whole
//!   `collision_distance_field` directory finds no definition anywhere.
//!   There is nothing to port.
//!
//! # Design: composition, not inheritance
//!
//! Upstream `PosedDistanceField : public distance_field::PropagationDistanceField`
//! is public inheritance. [`PosedDistanceField`] instead wraps a
//! [`PropagationDistanceField`] plus a pose -- see its doc comment for why
//! composition matches upstream's *actual* behaviour better than a trait
//! object would: upstream itself only re-derives one method
//! (`getDistanceGradient`) through the pose, so a Rust `Deref` would
//! misrepresent every other query as pose-aware when upstream leaves it
//! unposed.
//!
//! # Known upstream defects, ported byte-for-byte rather than silently fixed
//!
//! - [`do_bounding_spheres_intersect`] compares a squared distance against
//!   an unsquared radius sum -- see that function's own doc comment.
//! - [`BodyDecomposition::relative_cylinder_pose`] reads uninitialized
//!   memory upstream for a Sphere-only body -- see
//!   [`BodyDecomposition`]'s own doc comment and
//!   [`determine_collision_spheres`]'s.
//!
//! # Upstream test coverage
//!
//! `test/test_collision_distance_field.cpp` has five `TEST_F` cases, and
//! every one of them builds a `RobotState`/`RobotModel` and a
//! `CollisionEnvDistanceField` in `SetUp()` -- there is no case in that file
//! that exercises this slice without a `RobotModel`. Nothing from it is
//! ported. Verification instead relies on the `collision_distance_field_types`
//! oracle op (`tests/collision_distance_field_types_parity.rs`) plus
//! invariant-boundary unit tests below.

use std::collections::HashMap;
use std::sync::Arc;

use moveit_error::{Error, Result};
use moveit_geometry::bodies::{Body, merge_bounding_spheres};
use moveit_geometry::{BoundingSphere, Isometry3, Shape};
use nalgebra::{Point3, Vector3};

use crate::distance_field::{DistanceField, DistanceGradient};
use crate::find_internal_points::find_internal_points_convex;
use crate::propagation::PropagationDistanceField;
use crate::voxel_grid::GridGeometry;

/// Apply `pose` to `v` as if `v` were a point: `pose.rotation * v +
/// pose.translation`.
///
/// Every occurrence of `someIsometry * Eigen::Vector3d(...)` in the upstream
/// source this module ports is Eigen's documented "vector transformed as if
/// it were a point" behaviour: `Eigen::Transform::operator*` on a plain
/// (non-homogeneous) `Dim`-vector applies the full affine transform,
/// translation included -- confirmed by reading
/// `Eigen/src/Geometry/Transform.h`'s `transform_right_product_impl<..., 2,
/// 1>` (`res = T.linear() * other + T.translation()`), not inferred. This
/// holds even where the vector being transformed is conceptually a
/// direction rather than a point (see [`PosedDistanceField::distance_gradient`]'s
/// doc comment) -- Eigen's operator does not distinguish the two.
///
/// nalgebra's own `Isometry3 * Vector3` instead treats `Vector3` as a free
/// vector and applies rotation only (confirmed by reading nalgebra's
/// `isometry_ops.rs`: `Isometry * Vector` dispatches to
/// `self.rotation.transform_vector`, versus `Isometry * Point` which
/// includes the translation), so it cannot be used directly here without
/// silently dropping the translation term upstream's operator actually
/// applies.
fn transform_as_point(pose: &Isometry3, v: Vector3<f64>) -> Vector3<f64> {
    (pose * Point3::from(v)).coords
}

/// Upstream `collision_detection::CollisionType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionType {
    /// `NONE`.
    None = 0,
    /// `SELF`. Renamed from upstream's bare `SELF` -- `Self` is a reserved
    /// word in Rust.
    SelfCollision = 1,
    /// `INTRA`.
    Intra = 2,
    /// `ENVIRONMENT`.
    Environment = 3,
}

/// How a sphere-gradient query should be evaluated.
///
/// Upstream passes these five as five trailing parameters to both
/// `getCollisionSphereGradients` overloads. Grouping them is not only about
/// the argument count: `subtract_radii` and `stop_at_first_collision` are
/// two `bool`s separated by a single `f64`, so transposing them at a call
/// site compiles and silently changes what the query means. Named fields
/// make that transposition unrepresentable.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SphereGradientQuery {
    /// Recorded in [`GradientInfo::types`] for each sphere the query
    /// improves on.
    pub collision_type: CollisionType,
    /// Penetration below which a sphere is not counted as in collision.
    pub tolerance: f64,
    /// Subtract each sphere's radius from the sampled distance, turning a
    /// centre distance into a surface distance.
    pub subtract_radii: bool,
    /// Distances at or above this are ignored entirely.
    pub maximum_value: f64,
    /// Return as soon as any sphere is in collision, leaving the remaining
    /// spheres' gradients unwritten.
    pub stop_at_first_collision: bool,
}

/// A sphere approximating part of a body's geometry, in the body's own
/// (unposed) frame. Upstream `collision_detection::CollisionSphere`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CollisionSphere {
    /// `relative_vec_`: the sphere's center, relative to the body's frame.
    pub relative_vec: Vector3<f64>,
    /// `radius_`.
    pub radius: f64,
}

impl CollisionSphere {
    /// Upstream `CollisionSphere(rel, radius)`.
    pub fn new(relative_vec: Vector3<f64>, radius: f64) -> Self {
        Self {
            relative_vec,
            radius,
        }
    }
}

/// The result of a collision-sphere-against-distance-field query. Upstream
/// `collision_detection::GradientInfo`.
///
/// Upstream's `GradientInfo` carries no invariant tying the lengths of
/// `distances`/`gradients`/`types`/`sphere_radii`/`sphere_locations`
/// together -- callers are expected to size them all to match the sphere
/// list being queried before calling
/// [`PosedDistanceField::get_collision_sphere_gradients`]/[`get_collision_sphere_gradients`],
/// which index into `distances`/`types`/`gradients` without resizing them
/// ("assumes gradient is properly initialized", upstream's own comment).
/// This port keeps that contract rather than inventing a `resize`-on-query
/// that upstream does not have.
#[derive(Debug, Clone, PartialEq)]
pub struct GradientInfo {
    /// `closest_distance`.
    pub closest_distance: f64,
    /// `collision`.
    pub collision: bool,
    /// `sphere_locations`.
    pub sphere_locations: Vec<Vector3<f64>>,
    /// `distances`.
    pub distances: Vec<f64>,
    /// `gradients`.
    pub gradients: Vec<Vector3<f64>>,
    /// `types`.
    pub types: Vec<CollisionType>,
    /// `sphere_radii`.
    pub sphere_radii: Vec<f64>,
    /// `joint_name`.
    pub joint_name: String,
}

impl Default for GradientInfo {
    /// Upstream `GradientInfo()`.
    fn default() -> Self {
        Self {
            closest_distance: f64::MAX,
            collision: false,
            sphere_locations: Vec::new(),
            distances: Vec::new(),
            gradients: Vec::new(),
            types: Vec::new(),
            sphere_radii: Vec::new(),
            joint_name: String::new(),
        }
    }
}

impl GradientInfo {
    /// Upstream `GradientInfo::clear`.
    ///
    /// Upstream's `clear()` does not clear `types` -- every other vector
    /// field is cleared, `types` is not. Ported as-is rather than made
    /// symmetric: this is a faithful port of an asymmetry in the literal
    /// upstream source, not an omission introduced here.
    pub fn clear(&mut self) {
        self.closest_distance = f64::MAX;
        self.collision = false;
        self.sphere_locations.clear();
        self.distances.clear();
        self.gradients.clear();
        self.sphere_radii.clear();
        self.joint_name.clear();
    }
}

/// Upstream `collision_detection::ProximityInfo`. Not read or written by
/// anything else in this module (or in
/// `test_collision_distance_field.cpp`) -- it is a plain data type used by
/// the self-collision code this task does not port.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ProximityInfo {
    /// `link_name`.
    pub link_name: String,
    /// `attached_object_name`.
    pub attached_object_name: String,
    /// `proximity`.
    pub proximity: f64,
    /// `sphere_index`.
    pub sphere_index: u32,
    /// `att_index`.
    pub att_index: u32,
    /// `closest_point`.
    pub closest_point: Vector3<f64>,
    /// `closest_gradient`.
    pub closest_gradient: Vector3<f64>,
}

/// A [`PropagationDistanceField`] plus the pose it is queried through.
///
/// Upstream `PosedDistanceField : public distance_field::PropagationDistanceField`,
/// public inheritance. This port uses composition instead: a private
/// [`PropagationDistanceField`] plus `pose`, reached through explicit
/// accessors ([`PosedDistanceField::field`]/[`PosedDistanceField::field_mut`])
/// rather than `Deref`.
///
/// That is a deliberate choice, not D4's default reflex: upstream's own
/// `PosedDistanceField` only re-derives *one* method through the pose --
/// `getDistanceGradient`, overridden (non-virtually; it hides the base
/// method by name, C++ has no virtual dispatch here) to transform the query
/// point and gradient. Every other query
/// (`getDistance`/`worldToGrid`/`getResolution`/...) is inherited unposed:
/// calling `posed_field.getResolution()` in upstream just answers from the
/// wrapped field's own local frame, with no pose involved at all. A `Deref`
/// impl would route every trait method through auto-deref and make the
/// whole surface *look* pose-aware at the call site, which would misstate
/// what upstream actually does. The explicit accessor keeps that asymmetry
/// visible instead of hiding it behind indirection.
///
/// [`PosedDistanceField::distance_gradient`] is accordingly an inherent
/// method here, not a [`DistanceField`] impl -- it shadows (matching
/// upstream's non-virtual name-hiding, not an override) the field's own
/// [`DistanceField::distance_gradient`], and exists only on
/// [`PosedDistanceField`] itself.
pub struct PosedDistanceField {
    field: PropagationDistanceField,
    pose: Isometry3,
}

impl PosedDistanceField {
    /// Upstream `PosedDistanceField(size, origin, resolution, max_distance,
    /// propagate_negative_distances = false)`.
    ///
    /// # Errors
    ///
    /// See [`PropagationDistanceField::new`].
    pub fn new(
        size: Vector3<f64>,
        origin: Vector3<f64>,
        resolution: f64,
        max_distance: f64,
        propagate_negative_distances: bool,
    ) -> Result<Self> {
        let geometry = GridGeometry::new(size, origin, resolution)?;
        let field =
            PropagationDistanceField::new(geometry, max_distance, propagate_negative_distances)?;
        Ok(Self {
            field,
            pose: Isometry3::identity(),
        })
    }

    /// Upstream `PosedDistanceField::updatePose`.
    pub fn update_pose(&mut self, transform: Isometry3) {
        self.pose = transform;
    }

    /// Upstream `PosedDistanceField::getPose`.
    pub fn pose(&self) -> &Isometry3 {
        &self.pose
    }

    /// The wrapped, unposed field. See this type's doc comment for why this
    /// port exposes it through an explicit accessor rather than `Deref`.
    pub fn field(&self) -> &PropagationDistanceField {
        &self.field
    }

    /// Mutable access to the wrapped field, e.g. for
    /// [`DistanceField::add_points_to_field`].
    pub fn field_mut(&mut self) -> &mut PropagationDistanceField {
        &mut self.field
    }

    /// Upstream `PosedDistanceField::getDistanceGradient`.
    ///
    /// `x, y, z` are a query point in the *world* frame. Read from the
    /// literal upstream source rather than inferred, because the two halves
    /// of this method disagree in an easy-to-get-backwards way:
    ///
    /// - The query point is rotated into the field's local frame by this
    ///   pose's inverse **rotation only** (`pose_.linear().transpose()`
    ///   upstream) -- *not* the full inverse pose, so a non-zero pose
    ///   translation does not shift the query point before the lookup.
    /// - The gradient returned by the wrapped field's own
    ///   [`DistanceField::distance_gradient`] is then transformed back by
    ///   the **full** pose (`pose_ * gradient` upstream, i.e.
    ///   `transform_as_point` -- see that function's doc for why this
    ///   really does add `pose`'s translation to a gradient/direction
    ///   vector, not just rotate it).
    ///
    /// This crate's oracle-parity test (`collision_distance_field_types_parity.rs`)
    /// exercises a pose with a non-zero translation specifically to catch a
    /// "fix" that made the query-point transform use the full inverse pose,
    /// or made the gradient transform rotation-only.
    pub fn distance_gradient(&self, x: f64, y: f64, z: f64) -> DistanceGradient {
        let rel_pos = self.pose.rotation.inverse() * Vector3::new(x, y, z);
        let inner = self
            .field
            .distance_gradient(rel_pos.x, rel_pos.y, rel_pos.z);
        DistanceGradient {
            distance: inner.distance,
            gradient: transform_as_point(&self.pose, inner.gradient),
            in_bounds: inner.in_bounds,
        }
    }

    /// Upstream `PosedDistanceField::getCollisionSphereGradients`.
    ///
    /// # Deviation from upstream
    ///
    /// This method differs from the free [`get_collision_sphere_gradients`]
    /// in two ways upstream itself is inconsistent about -- ported
    /// faithfully rather than unified, since unifying them would be a
    /// behaviour change, not a parity fix:
    ///
    /// - The out-of-bounds guard compares `grad.norm() > 0` here, versus
    ///   `grad.norm() > EPSILON` (`1e-4`) in the free function.
    /// - `dist` is `abs()`-ed after subtracting the sphere radius here, but
    ///   not in the free function.
    ///
    /// Both guards are dead on every path this port's
    /// [`DistanceField::distance_gradient`] can produce: it always reports
    /// gradient `(0, 0, 0)` exactly when `!in_bounds` (see
    /// `distance_field.cpp`'s `getDistanceGradient`, which zeroes the
    /// out-parameters before returning on the out-of-bounds path -- this
    /// port's [`DistanceField::distance_gradient`] mirrors that), so
    /// `grad.norm() > 0` and `grad.norm() > EPSILON` are both always false
    /// regardless of which threshold is used. Kept for structural parity
    /// with upstream, not because either branch is reachable.
    pub fn get_collision_sphere_gradients(
        &self,
        sphere_list: &[CollisionSphere],
        sphere_centers: &[Vector3<f64>],
        gradient: &mut GradientInfo,
        query: &SphereGradientQuery,
    ) -> bool {
        let &SphereGradientQuery {
            collision_type,
            tolerance,
            subtract_radii,
            maximum_value,
            stop_at_first_collision,
        } = query;

        let mut in_collision = false;
        for (i, sphere) in sphere_list.iter().enumerate() {
            let p = sphere_centers[i];
            let result = self.distance_gradient(p.x, p.y, p.z);
            if !result.in_bounds && result.gradient.norm() > 0.0 {
                return true;
            }

            let mut dist = result.distance;
            if dist < maximum_value {
                if subtract_radii {
                    dist -= sphere.radius;
                    if dist < 0.0 && -dist >= tolerance {
                        in_collision = true;
                    }
                    dist = dist.abs();
                } else if sphere.radius - dist > tolerance {
                    in_collision = true;
                }

                if dist < gradient.closest_distance {
                    gradient.closest_distance = dist;
                }
                if dist < gradient.distances[i] {
                    gradient.types[i] = collision_type;
                    gradient.distances[i] = dist;
                    gradient.gradients[i] = result.gradient;
                }
            }

            if stop_at_first_collision && in_collision {
                return true;
            }
        }
        in_collision
    }
}

/// Upstream free function `determineCollisionSpheres`.
///
/// Upstream's own comment flags returning `std::vector<CollisionSphere>` by
/// value as "BAD ... allocation errors will happen" and asks for it to be
/// changed to an output parameter; that complaint is about C++ value
/// semantics; Rust's `Vec<T>` return is the idiomatic, zero-extra-copy
/// (move/NRVO) way to hand back an owned collection, so it does not carry
/// over. The recompute-per-call design itself is kept as-is.
///
/// # Deviation from upstream: `num_points == 0` guard
///
/// Upstream computes `num_points = ceil(cyl.length / (cyl.radius / 2.0))`
/// as an `unsigned int`, then loops `for (i = 1; i < num_points - 1; i++)`.
/// If `cyl.length` is exactly `0.0` (a degenerate bounding cylinder),
/// `num_points` is `0` and `num_points - 1` underflows to `UINT_MAX` in C++
/// (defined wraparound for unsigned arithmetic, but not a defined *loop
/// bound* -- the loop would run into the billions before `i` could ever
/// reach it). This port uses `num_points.saturating_sub(1)`, so the same
/// input yields an empty range (zero collision spheres from the cylinder
/// branch) instead. None of this module's test fixtures reach this case
/// (Sphere never takes the cylinder branch at all; Box/Cylinder/Mesh here
/// all have non-degenerate bounding cylinders), so this is an unreachable
/// input made *safe* rather than a behaviour this port can currently
/// validate against upstream.
///
/// # `relative_transform` is left untouched on the sphere branch
///
/// Matching upstream, the `body: Body::Sphere(_)` branch never writes
/// `relative_transform` -- only the cylinder/box/mesh branch does. See
/// [`BodyDecomposition`]'s doc comment for why upstream's version of this is
/// a genuine uninitialized-memory defect, and why this port's version is not
/// (this port's caller seeds `relative_transform` from
/// `Isometry3::identity()`, not an uninitialized stack slot).
pub fn determine_collision_spheres(
    body: &Body,
    relative_transform: &mut Isometry3,
) -> Vec<CollisionSphere> {
    let mut spheres = Vec::new();

    if matches!(body, Body::Sphere(_)) {
        spheres.push(CollisionSphere::new(
            body.pose().translation.vector,
            body.dimensions()[0],
        ));
    } else {
        let cyl = body.compute_bounding_cylinder();
        let num_points = (cyl.length / (cyl.radius / 2.0)).ceil() as u32;
        let spacing = cyl.length / (f64::from(num_points) - 1.0);
        *relative_transform = cyl.pose;

        for i in 1..num_points.saturating_sub(1) {
            let offset = Vector3::new(0.0, 0.0, -cyl.length / 2.0 + f64::from(i) * spacing);
            spheres.push(CollisionSphere::new(
                transform_as_point(relative_transform, offset),
                cyl.radius,
            ));
        }
    }

    spheres
}

/// Upstream free function `getCollisionSphereGradients`, taking an unposed
/// `distance_field::DistanceField*` directly rather than a
/// [`PosedDistanceField`]. See
/// [`PosedDistanceField::get_collision_sphere_gradients`]'s "Deviation from
/// upstream" doc for the two places these two overloads disagree, preserved
/// here rather than unified.
pub fn get_collision_sphere_gradients(
    distance_field: &dyn DistanceField,
    sphere_list: &[CollisionSphere],
    sphere_centers: &[Vector3<f64>],
    gradient: &mut GradientInfo,
    query: &SphereGradientQuery,
) -> bool {
    const EPSILON: f64 = 0.0001;

    let &SphereGradientQuery {
        collision_type,
        tolerance,
        subtract_radii,
        maximum_value,
        stop_at_first_collision,
    } = query;

    let mut in_collision = false;
    for (i, sphere) in sphere_list.iter().enumerate() {
        let p = sphere_centers[i];
        let result = distance_field.distance_gradient(p.x, p.y, p.z);
        if !result.in_bounds && result.gradient.norm() > EPSILON {
            return true;
        }

        let mut dist = result.distance;
        if dist < maximum_value {
            if subtract_radii {
                dist -= sphere.radius;
                if dist < 0.0 && -dist >= tolerance {
                    in_collision = true;
                }
            } else if sphere.radius - dist > tolerance {
                in_collision = true;
            }

            if dist < gradient.closest_distance {
                gradient.closest_distance = dist;
            }
            if dist < gradient.distances[i] {
                gradient.types[i] = collision_type;
                gradient.distances[i] = dist;
                gradient.gradients[i] = result.gradient;
            }
        }

        if stop_at_first_collision && in_collision {
            return true;
        }
    }
    in_collision
}

/// Upstream free function `getCollisionSphereCollision` (the boolean-only
/// overload).
pub fn get_collision_sphere_collision(
    distance_field: &dyn DistanceField,
    sphere_list: &[CollisionSphere],
    sphere_centers: &[Vector3<f64>],
    maximum_value: f64,
    tolerance: f64,
) -> bool {
    for (i, sphere) in sphere_list.iter().enumerate() {
        let p = sphere_centers[i];
        let result = distance_field.distance_gradient(p.x, p.y, p.z);
        if !result.in_bounds && result.gradient.norm() > 0.0 {
            return true;
        }
        if maximum_value > result.distance && sphere.radius - result.distance > tolerance {
            return true;
        }
    }
    false
}

/// Upstream free function `getCollisionSphereCollision` (the overload that
/// also collects up to `num_coll` colliding sphere indices into `colls`).
///
/// `num_coll == 0` upstream means "report on the first collision, without
/// collecting anything" -- ported as-is via the same early return, not
/// treated as "collect indefinitely" or "never report".
pub fn get_collision_sphere_collisions(
    distance_field: &dyn DistanceField,
    sphere_list: &[CollisionSphere],
    sphere_centers: &[Vector3<f64>],
    maximum_value: f64,
    tolerance: f64,
    num_coll: u32,
    colls: &mut Vec<u32>,
) -> bool {
    colls.clear();
    for (i, sphere) in sphere_list.iter().enumerate() {
        let p = sphere_centers[i];
        let result = distance_field.distance_gradient(p.x, p.y, p.z);
        if !result.in_bounds && result.gradient.norm() > 0.0 {
            return true;
        }
        if maximum_value > result.distance && sphere.radius - result.distance > tolerance {
            if num_coll == 0 {
                return true;
            }
            colls.push(i as u32);
            if colls.len() as u32 >= num_coll {
                return true;
            }
        }
    }
    !colls.is_empty()
}

/// A shape (or several) decomposed into collision spheres and interior
/// sample points, in the shape's own (unposed) frame. Upstream
/// `collision_detection::BodyDecomposition`.
///
/// Upstream's `bodies::BodyVector` member is a thin
/// `Vec<Body>`-plus-first-hit-query wrapper this workspace already declined
/// to port (see `moveit_geometry::bodies`'s module doc); `BodyDecomposition`
/// itself never uses the query, only iteration and count, so this port
/// holds a plain `Vec<Body>` directly.
///
/// # Known upstream defect: `relative_cylinder_pose_` is uninitialized for a Sphere body
///
/// Upstream's `determineCollisionSpheres` only writes its
/// `Eigen::Isometry3d& relative_transform` output parameter on the
/// cylinder/box/mesh branch (`*relative_transform = cyl.getPose();`); the
/// sphere branch returns without touching it at all. `relative_transform` is
/// `BodyDecomposition::relative_cylinder_pose_`, a plain `Eigen::Isometry3d`
/// member with no default initializer, so for a Sphere-only
/// `BodyDecomposition` that member is left holding whatever bytes were on
/// the stack/heap at construction -- confirmed empirically, not just read
/// from source: two independent Sphere-shaped requests to the oracle's
/// `collision_distance_field_types` op returned wildly different,
/// obviously-garbage values for it (e.g. `8.242212364724648e+115`,
/// `1.63e-322`, `4.32753216e-315`), non-reproducible across runs. This
/// port's [`determine_collision_spheres`] has the identical
/// leave-untouched-on-the-Sphere-branch behaviour (see its own doc comment),
/// but because [`BodyDecomposition::from_shapes`] seeds `relative_transform`
/// from `Isometry3::identity()` rather than an uninitialized stack slot, a
/// Sphere-only decomposition here deterministically reports identity instead
/// of garbage -- a divergence from upstream's actual (undefined) behaviour
/// that is *more* defined, not less, and therefore not "fixed" to match:
/// there is no defined upstream value to match. The oracle-parity test
/// (`collision_distance_field_types_parity.rs`) accordingly excludes
/// `relative_cylinder_pose()` from comparison for every Sphere-shaped
/// fixture case.
pub struct BodyDecomposition {
    bodies: Vec<Body>,
    relative_cylinder_pose: Isometry3,
    relative_bounding_sphere: BoundingSphere,
    sphere_radii: Vec<f64>,
    collision_spheres: Vec<CollisionSphere>,
    relative_collision_points: Vec<Vector3<f64>>,
}

impl BodyDecomposition {
    /// The padding upstream's single-shape constructor applies when the
    /// caller does not override it (`BodyDecomposition(shape, resolution,
    /// padding = 0.01)`). Rust has no default parameters, so this constant
    /// makes that default explicit at call sites that want it, via
    /// [`BodyDecomposition::new`].
    pub const DEFAULT_PADDING: f64 = 0.01;

    /// Upstream `BodyDecomposition(shape, resolution, padding)`.
    ///
    /// # Errors
    ///
    /// See [`BodyDecomposition::from_shapes`].
    pub fn new(shape: &Shape, resolution: f64, padding: f64) -> Result<Self> {
        Self::from_shapes(
            std::slice::from_ref(shape),
            &[Isometry3::identity()],
            resolution,
            padding,
        )
    }

    /// Upstream `BodyDecomposition(shapes, poses, resolution, padding)`
    /// (which upstream's `init` implements for both constructors).
    ///
    /// # Errors
    ///
    /// [`moveit_error::Error::Construct`] if any of `shapes` has no
    /// `bodies::` counterpart -- see [`moveit_geometry::bodies::Body::from_shape`].
    ///
    /// `shapes` and `poses` must be the same length; a length mismatch
    /// panics via the `zip`-then-indexing below, matching upstream's own
    /// unchecked `shapes[i]`/`poses[i]` indexing in `init`.
    pub fn from_shapes(
        shapes: &[Shape],
        poses: &[Isometry3],
        resolution: f64,
        padding: f64,
    ) -> Result<Self> {
        assert_eq!(
            shapes.len(),
            poses.len(),
            "BodyDecomposition::from_shapes: {} shapes but {} poses",
            shapes.len(),
            poses.len()
        );

        let mut bodies = Vec::with_capacity(shapes.len());
        for (shape, pose) in shapes.iter().zip(poses) {
            let mut body = Body::from_shape(shape)?.ok_or_else(|| {
                Error::construct(format!(
                    "BodyDecomposition shapes must be Sphere, Cylinder, Cuboid, or Mesh, got {shape:?}"
                ))
            })?;
            body.set_pose(*pose);
            body.set_padding(padding)?;
            bodies.push(body);
        }

        let mut collision_spheres = Vec::new();
        let mut relative_collision_points = Vec::new();
        let mut relative_cylinder_pose = Isometry3::identity();
        for body in &bodies {
            collision_spheres.extend(determine_collision_spheres(
                body,
                &mut relative_cylinder_pose,
            ));

            let mut body_points = Vec::new();
            find_internal_points_convex(body, resolution, &mut body_points);
            relative_collision_points.extend(body_points);
        }

        let sphere_radii = collision_spheres.iter().map(|s| s.radius).collect();

        let bounding_spheres: Vec<BoundingSphere> =
            bodies.iter().map(Body::compute_bounding_sphere).collect();
        let relative_bounding_sphere = merge_bounding_spheres(&bounding_spheres);

        Ok(Self {
            bodies,
            relative_cylinder_pose,
            relative_bounding_sphere,
            sphere_radii,
            collision_spheres,
            relative_collision_points,
        })
    }

    /// Upstream `BodyDecomposition::replaceCollisionSpheres`.
    pub fn replace_collision_spheres(
        &mut self,
        new_collision_spheres: Vec<CollisionSphere>,
        new_relative_cylinder_pose: Isometry3,
    ) {
        self.collision_spheres = new_collision_spheres;
        self.relative_cylinder_pose = new_relative_cylinder_pose;
    }

    /// Upstream `BodyDecomposition::getCollisionSpheres`.
    pub fn collision_spheres(&self) -> &[CollisionSphere] {
        &self.collision_spheres
    }

    /// Upstream `BodyDecomposition::getSphereRadii`.
    pub fn sphere_radii(&self) -> &[f64] {
        &self.sphere_radii
    }

    /// Upstream `BodyDecomposition::getCollisionPoints`.
    pub fn collision_points(&self) -> &[Vector3<f64>] {
        &self.relative_collision_points
    }

    /// Upstream `BodyDecomposition::getBody`. Panics if `i` is out of range,
    /// matching this crate's established panic-not-UB stance on unchecked
    /// upstream indexing (see e.g. `VoxelGrid::get_cell`'s doc comment).
    pub fn body(&self, i: usize) -> &Body {
        &self.bodies[i]
    }

    /// Upstream `BodyDecomposition::getBodiesCount`.
    pub fn bodies_count(&self) -> usize {
        self.bodies.len()
    }

    /// Upstream `BodyDecomposition::getRelativeCylinderPose`.
    pub fn relative_cylinder_pose(&self) -> Isometry3 {
        self.relative_cylinder_pose
    }

    /// Upstream `BodyDecomposition::getRelativeBoundingSphere`.
    pub fn relative_bounding_sphere(&self) -> BoundingSphere {
        self.relative_bounding_sphere
    }
}

/// A [`BodyDecomposition`] posed into a reference frame, tracking sphere
/// centers as a separate cache alongside the shared unposed decomposition.
/// Upstream `collision_detection::PosedBodySphereDecomposition`.
///
/// Upstream holds `body_decomposition_` as a `BodyDecompositionConstPtr`
/// (`shared_ptr<const BodyDecomposition>`), so the same unposed
/// decomposition can be shared across multiple posed instances (e.g. one
/// per robot state query, all re-posing the same link geometry). This port
/// uses [`Arc<BodyDecomposition>`] for the same sharing.
pub struct PosedBodySphereDecomposition {
    body_decomposition: Arc<BodyDecomposition>,
    posed_bounding_sphere_center: Vector3<f64>,
    posed_collision_points: Vec<Vector3<f64>>,
    sphere_centers: Vec<Vector3<f64>>,
}

impl PosedBodySphereDecomposition {
    /// Upstream `PosedBodySphereDecomposition(body_decomposition)`.
    pub fn new(body_decomposition: Arc<BodyDecomposition>) -> Self {
        let posed_bounding_sphere_center = body_decomposition.relative_bounding_sphere().center;
        let sphere_centers = vec![Vector3::zeros(); body_decomposition.collision_spheres().len()];
        let mut this = Self {
            body_decomposition,
            posed_bounding_sphere_center,
            posed_collision_points: Vec::new(),
            sphere_centers,
        };
        this.update_pose(Isometry3::identity());
        this
    }

    /// Upstream `PosedBodySphereDecomposition::getCollisionSpheres`.
    pub fn collision_spheres(&self) -> &[CollisionSphere] {
        self.body_decomposition.collision_spheres()
    }

    /// Upstream `PosedBodySphereDecomposition::getSphereCenters`.
    pub fn sphere_centers(&self) -> &[Vector3<f64>] {
        &self.sphere_centers
    }

    /// Upstream `PosedBodySphereDecomposition::getCollisionPoints`.
    pub fn collision_points(&self) -> &[Vector3<f64>] {
        &self.posed_collision_points
    }

    /// Upstream `PosedBodySphereDecomposition::getSphereRadii`.
    pub fn sphere_radii(&self) -> &[f64] {
        self.body_decomposition.sphere_radii()
    }

    /// Upstream `PosedBodySphereDecomposition::getBoundingSphereCenter`.
    pub fn bounding_sphere_center(&self) -> Vector3<f64> {
        self.posed_bounding_sphere_center
    }

    /// Upstream `PosedBodySphereDecomposition::getBoundingSphereRadius`.
    pub fn bounding_sphere_radius(&self) -> f64 {
        self.body_decomposition.relative_bounding_sphere().radius
    }

    /// Upstream `PosedBodySphereDecomposition::updatePose`: `trans` is
    /// assumed to already be expressed in the reference frame the caller
    /// wants (upstream's doc comment: "assumed to be in reference frame").
    ///
    /// Indexes `sphere_centers` up to `body_decomposition`'s current sphere
    /// count without resizing it first, matching upstream (which never
    /// resizes `sphere_centers_` inside `updatePose`, only once in the
    /// constructor) -- panics if `body_decomposition`'s sphere count grew
    /// since this decomposition was constructed, the same sharp edge
    /// upstream has as unchecked-index undefined behaviour.
    pub fn update_pose(&mut self, trans: Isometry3) {
        self.posed_bounding_sphere_center = transform_as_point(
            &trans,
            self.body_decomposition.relative_bounding_sphere().center,
        );
        for (i, sphere) in self
            .body_decomposition
            .collision_spheres()
            .iter()
            .enumerate()
        {
            self.sphere_centers[i] = transform_as_point(&trans, sphere.relative_vec);
        }
        if !self.body_decomposition.collision_points().is_empty() {
            self.posed_collision_points = self
                .body_decomposition
                .collision_points()
                .iter()
                .map(|p| transform_as_point(&trans, *p))
                .collect();
        }
    }
}

/// A [`BodyDecomposition`]'s interior sample points, posed into a reference
/// frame. Upstream `collision_detection::PosedBodyPointDecomposition`.
pub struct PosedBodyPointDecomposition {
    /// `None` only for a decomposition built from an octree upstream (not
    /// ported -- see this module's doc); every constructor this port
    /// carries sets this to `Some`.
    body_decomposition: Option<Arc<BodyDecomposition>>,
    posed_collision_points: Vec<Vector3<f64>>,
}

impl PosedBodyPointDecomposition {
    /// Upstream `PosedBodyPointDecomposition(body_decomposition)`: posed at
    /// identity (upstream copies `getCollisionPoints()` verbatim rather than
    /// calling `updatePose(Identity())`, but `transform_as_point` at
    /// identity is the identity function, so the result is the same).
    pub fn new(body_decomposition: Arc<BodyDecomposition>) -> Self {
        let posed_collision_points = body_decomposition.collision_points().to_vec();
        Self {
            body_decomposition: Some(body_decomposition),
            posed_collision_points,
        }
    }

    /// Upstream `PosedBodyPointDecomposition(body_decomposition, pose)`.
    pub fn with_pose(body_decomposition: Arc<BodyDecomposition>, pose: Isometry3) -> Self {
        let mut this = Self {
            body_decomposition: Some(body_decomposition),
            posed_collision_points: Vec::new(),
        };
        this.update_pose(pose);
        this
    }

    /// Upstream `PosedBodyPointDecomposition::getCollisionPoints`.
    pub fn collision_points(&self) -> &[Vector3<f64>] {
        &self.posed_collision_points
    }

    /// Upstream `PosedBodyPointDecomposition::updatePose`. A no-op when this
    /// decomposition has no [`BodyDecomposition`] to re-derive points from
    /// (would only happen for the unported octree constructor upstream),
    /// matching upstream's `if (body_decomposition_)` guard.
    pub fn update_pose(&mut self, trans: Isometry3) {
        if let Some(bd) = &self.body_decomposition {
            self.posed_collision_points = bd
                .collision_points()
                .iter()
                .map(|p| transform_as_point(&trans, *p))
                .collect();
        }
    }
}

/// A collection of [`PosedBodySphereDecomposition`]s, plus their collision
/// spheres/centers/radii flattened into one contiguous set for batch
/// queries. Upstream `collision_detection::PosedBodySphereDecompositionVector`.
///
/// Upstream stores `decomp_vector_` as `std::vector<PosedBodySphereDecompositionPtr>`
/// (shared_ptr elements) so a caller can keep its own handle to a
/// decomposition after adding it to the vector. Nothing in this module's
/// scope demonstrates a need for that shared mutable aliasing -- the vector
/// is the sole owner in every use this crate has -- so this port holds
/// [`PosedBodySphereDecomposition`] by value instead of behind a shared
/// pointer.
#[derive(Default)]
pub struct PosedBodySphereDecompositionVector {
    decomp_vector: Vec<PosedBodySphereDecomposition>,
    collision_spheres: Vec<CollisionSphere>,
    posed_collision_spheres: Vec<Vector3<f64>>,
    sphere_radii: Vec<f64>,
    sphere_index_map: HashMap<usize, usize>,
}

impl PosedBodySphereDecompositionVector {
    /// Upstream `PosedBodySphereDecompositionVector()`.
    pub fn new() -> Self {
        Self::default()
    }

    /// Upstream `PosedBodySphereDecompositionVector::getCollisionSpheres`.
    pub fn collision_spheres(&self) -> &[CollisionSphere] {
        &self.collision_spheres
    }

    /// Upstream `PosedBodySphereDecompositionVector::getSphereCenters`.
    pub fn sphere_centers(&self) -> &[Vector3<f64>] {
        &self.posed_collision_spheres
    }

    /// Upstream `PosedBodySphereDecompositionVector::getSphereRadii`.
    pub fn sphere_radii(&self) -> &[f64] {
        &self.sphere_radii
    }

    /// Upstream `PosedBodySphereDecompositionVector::addToVector`.
    pub fn add_to_vector(&mut self, bd: PosedBodySphereDecomposition) {
        self.sphere_index_map
            .insert(self.decomp_vector.len(), self.collision_spheres.len());
        self.collision_spheres
            .extend_from_slice(bd.collision_spheres());
        self.posed_collision_spheres
            .extend_from_slice(bd.sphere_centers());
        self.sphere_radii.extend_from_slice(bd.sphere_radii());
        self.decomp_vector.push(bd);
    }

    /// Upstream `PosedBodySphereDecompositionVector::getSize`.
    pub fn len(&self) -> usize {
        self.decomp_vector.len()
    }

    /// `true` when this vector has no decompositions.
    pub fn is_empty(&self) -> bool {
        self.decomp_vector.is_empty()
    }

    /// Upstream `PosedBodySphereDecompositionVector::getPosedBodySphereDecomposition`.
    /// `None` for an out-of-range index, replacing upstream's
    /// log-then-return-a-null-shared_ptr (no ROS logging in this crate --
    /// PORTING-PLAN.md D1 -- and `Option` is the idiomatic Rust equivalent
    /// of a nullable pointer).
    pub fn get(&self, i: usize) -> Option<&PosedBodySphereDecomposition> {
        self.decomp_vector.get(i)
    }

    /// Upstream `PosedBodySphereDecompositionVector::updatePose`. `false`
    /// for an out-of-range `ind`, replacing upstream's
    /// log-a-warning-and-return (same D1 rationale as
    /// [`PosedBodySphereDecompositionVector::get`]).
    pub fn update_pose(&mut self, ind: usize, pose: Isometry3) -> bool {
        let Some(decomp) = self.decomp_vector.get_mut(ind) else {
            return false;
        };
        decomp.update_pose(pose);
        let base = self.sphere_index_map[&ind];
        for (i, center) in decomp.sphere_centers().iter().enumerate() {
            self.posed_collision_spheres[base + i] = *center;
        }
        true
    }
}

/// A collection of [`PosedBodyPointDecomposition`]s. Upstream
/// `collision_detection::PosedBodyPointDecompositionVector`. See
/// [`PosedBodySphereDecompositionVector`]'s doc comment for why this port
/// holds elements by value rather than behind a shared pointer.
#[derive(Default)]
pub struct PosedBodyPointDecompositionVector {
    decomp_vector: Vec<PosedBodyPointDecomposition>,
}

impl PosedBodyPointDecompositionVector {
    /// Upstream `PosedBodyPointDecompositionVector()`.
    pub fn new() -> Self {
        Self::default()
    }

    /// Upstream `PosedBodyPointDecompositionVector::getCollisionPoints`:
    /// concatenates every sub-decomposition's points into a fresh `Vec` on
    /// every call (not cached) -- ported as-is.
    pub fn collision_points(&self) -> Vec<Vector3<f64>> {
        self.decomp_vector
            .iter()
            .flat_map(|d| d.collision_points().iter().copied())
            .collect()
    }

    /// Upstream `PosedBodyPointDecompositionVector::addToVector`.
    pub fn add_to_vector(&mut self, bd: PosedBodyPointDecomposition) {
        self.decomp_vector.push(bd);
    }

    /// Upstream `PosedBodyPointDecompositionVector::getSize`.
    pub fn len(&self) -> usize {
        self.decomp_vector.len()
    }

    /// `true` when this vector has no decompositions.
    pub fn is_empty(&self) -> bool {
        self.decomp_vector.is_empty()
    }

    /// Upstream `PosedBodyPointDecompositionVector::getPosedBodyDecomposition`.
    /// See [`PosedBodySphereDecompositionVector::get`]'s doc for the
    /// `Option`-replaces-nullable-shared_ptr rationale.
    pub fn get(&self, i: usize) -> Option<&PosedBodyPointDecomposition> {
        self.decomp_vector.get(i)
    }

    /// Upstream `PosedBodyPointDecompositionVector::updatePose`. See
    /// [`PosedBodySphereDecompositionVector::update_pose`]'s doc for the
    /// `bool`-replaces-log-and-return rationale.
    pub fn update_pose(&mut self, ind: usize, pose: Isometry3) -> bool {
        let Some(decomp) = self.decomp_vector.get_mut(ind) else {
            return false;
        };
        decomp.update_pose(pose);
        true
    }
}

/// Upstream free function `doBoundingSpheresIntersect`.
///
/// # Upstream defect, preserved
///
/// Upstream compares `(p1_center - p2_center).squaredNorm()` (a **squared**
/// distance) against `p1_radius + p2_radius` (an **unsquared** sum of
/// radii) -- dimensionally inconsistent, so the two sides are comparable
/// only by coincidence rather than by construction. This looks like
/// upstream meant to square the right-hand side (or take `.norm()` on the
/// left) and never caught it: `test_collision_distance_field.cpp` has no
/// case that reaches this function at all (see this module's doc), so
/// nothing has ever exercised it. Ported byte-for-byte, since this task's
/// mandate is matching upstream's actual behaviour, not upstream's intent.
/// **Do not** "fix" this by squaring the right-hand side without raising it
/// as its own change -- that alters what counts as intersecting, which is a
/// semantic change, not a parity fix.
///
/// ## It errs in both directions, and one of them is unsafe
///
/// Writing `d` for the true centre distance and `s` for the radius sum,
/// upstream asks `d² < s` where it means `d < s`. The two disagree on
/// either side of `d = 1`, in opposite directions:
///
/// - `d < 1` shrinks under squaring, so `d² < s` holds where `d < s` does
///   not: a **false positive**, reporting an intersection between spheres
///   that are apart. For a broad-phase test that is merely wasteful --
///   the narrow phase that follows rejects it.
/// - `d > 1` grows under squaring, so `d² ≥ s` holds where `d < s` does:
///   a **false negative**, reporting no intersection between spheres that
///   genuinely overlap. `s = 3, d = 2` is such a case. A broad-phase test
///   that answers "no" is not corrected by anything downstream, because
///   nothing downstream runs -- the pair is culled.
///
/// The false-negative branch needs a radius sum above 1 (metres, as
/// everything here is), which whole-link bounding spheres on a
/// PR2-sized robot reach. So this is not only a performance wart; it can
/// drop a colliding pair. That is the reason it is worth raising as its
/// own change rather than leaving as a curiosity, and it is why both
/// directions are pinned by tests below.
pub fn do_bounding_spheres_intersect(
    p1: &PosedBodySphereDecomposition,
    p2: &PosedBodySphereDecomposition,
) -> bool {
    let dist = (p1.bounding_sphere_center() - p2.bounding_sphere_center()).norm_squared();
    dist < (p1.bounding_sphere_radius() + p2.bounding_sphere_radius())
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use nalgebra::{Translation3, UnitQuaternion};

    use super::*;
    use moveit_geometry::{Cylinder, Sphere};

    fn sphere_body(radius: f64, translation: Vector3<f64>) -> Body {
        let mut body = Body::from_shape(&Shape::Sphere(Sphere::new(radius).unwrap()))
            .unwrap()
            .unwrap();
        body.set_pose(Isometry3::from_parts(
            Translation3::from(translation),
            UnitQuaternion::identity(),
        ));
        body.set_padding(0.0).unwrap();
        body
    }

    /// Invariant boundary: `determine_collision_spheres`'s branch selection
    /// is exactly `Body::Sphere` vs everything else -- the sphere branch
    /// must produce one sphere at the body's own pose and radius, and must
    /// leave `relative_transform` byte-for-byte untouched (see
    /// [`determine_collision_spheres`]'s own doc comment on this).
    #[test]
    fn determine_collision_spheres_sphere_branch_leaves_relative_transform_untouched() {
        let body = sphere_body(0.3, Vector3::new(1.0, 2.0, 3.0));
        let sentinel =
            Isometry3::from_parts(Translation3::new(9.0, 9.0, 9.0), UnitQuaternion::identity());
        let mut relative_transform = sentinel;

        let spheres = determine_collision_spheres(&body, &mut relative_transform);

        assert_eq!(spheres.len(), 1);
        assert_eq!(spheres[0].relative_vec, Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(spheres[0].radius, 0.3);
        assert_eq!(
            relative_transform, sentinel,
            "sphere branch must not write relative_transform"
        );
    }

    /// The complementary boundary: a non-sphere body takes the cylinder
    /// branch, which both produces more than one sphere and does write
    /// `relative_transform` (to the bounding cylinder's own pose).
    #[test]
    fn determine_collision_spheres_cylinder_branch_writes_relative_transform() {
        let mut body = Body::from_shape(&Shape::Cylinder(Cylinder::new(0.2, 1.0).unwrap()))
            .unwrap()
            .unwrap();
        body.set_padding(0.0).unwrap();
        let mut relative_transform = Isometry3::identity();

        let spheres = determine_collision_spheres(&body, &mut relative_transform);

        // num_points = ceil(1.0 / (0.2 / 2.0)) = 10, loop i in 1..9 -> 8 spheres.
        assert_eq!(spheres.len(), 8);
        for sphere in &spheres {
            assert_relative_eq!(sphere.radius, 0.2);
        }
        assert_eq!(
            relative_transform,
            body.compute_bounding_cylinder().pose,
            "cylinder branch must write relative_transform to the bounding cylinder's pose"
        );
    }

    /// Upstream `GradientInfo::clear()` asymmetry, ported as-is (see
    /// [`GradientInfo::clear`]'s doc): every field is reset except `types`.
    #[test]
    fn gradient_info_clear_does_not_clear_types() {
        let mut info = GradientInfo {
            closest_distance: 5.0,
            collision: true,
            sphere_locations: vec![Vector3::new(1.0, 1.0, 1.0)],
            distances: vec![1.0],
            gradients: vec![Vector3::new(1.0, 0.0, 0.0)],
            types: vec![CollisionType::SelfCollision],
            sphere_radii: vec![0.1],
            joint_name: "joint".to_string(),
        };

        info.clear();

        assert_eq!(info.closest_distance, f64::MAX);
        assert!(!info.collision);
        assert!(info.sphere_locations.is_empty());
        assert!(info.distances.is_empty());
        assert!(info.gradients.is_empty());
        assert!(info.sphere_radii.is_empty());
        assert!(info.joint_name.is_empty());
        assert_eq!(
            info.types,
            vec![CollisionType::SelfCollision],
            "clear() must not clear types, matching upstream's asymmetry"
        );
    }

    /// Invariant boundary: an out-of-range index on
    /// [`PosedBodySphereDecompositionVector`] must report absence
    /// (`None`/`false`), not panic or silently no-op past the check.
    #[test]
    fn posed_body_sphere_decomposition_vector_out_of_range_is_none_and_false() {
        let mut vector = PosedBodySphereDecompositionVector::new();
        assert!(vector.get(0).is_none());
        assert!(!vector.update_pose(0, Isometry3::identity()));
    }

    /// Same boundary as above, for [`PosedBodyPointDecompositionVector`].
    #[test]
    fn posed_body_point_decomposition_vector_out_of_range_is_none_and_false() {
        let mut vector = PosedBodyPointDecompositionVector::new();
        assert!(vector.get(0).is_none());
        assert!(!vector.update_pose(0, Isometry3::identity()));
    }

    /// Invariant boundary: [`PosedBodyPointDecomposition::update_pose`] is a
    /// no-op when there is no [`BodyDecomposition`] to re-derive points from
    /// (upstream's `if (body_decomposition_)` guard) -- only reachable in
    /// this port via direct construction, since every public constructor
    /// this crate carries sets `Some` (the octree constructor that would
    /// leave it `None` upstream is not ported; see this module's doc).
    #[test]
    fn posed_body_point_decomposition_update_pose_is_noop_without_body_decomposition() {
        let original_points = vec![Vector3::new(1.0, 2.0, 3.0)];
        let mut decomp = PosedBodyPointDecomposition {
            body_decomposition: None,
            posed_collision_points: original_points.clone(),
        };

        decomp.update_pose(Isometry3::from_parts(
            Translation3::new(5.0, 5.0, 5.0),
            UnitQuaternion::identity(),
        ));

        assert_eq!(decomp.collision_points(), original_points.as_slice());
    }

    /// Upstream `BodyDecomposition::getBody` has no bounds check (unchecked
    /// `bodies_[i]`); this port's [`BodyDecomposition::body`] panics instead
    /// of reading out of bounds, matching this crate's established
    /// panic-not-UB stance (see that method's own doc comment).
    #[test]
    #[should_panic(expected = "index out of bounds")]
    fn body_decomposition_body_panics_out_of_range() {
        let body_decomposition =
            BodyDecomposition::new(&Shape::Sphere(Sphere::new(0.1).unwrap()), 0.05, 0.0).unwrap();
        let _ = body_decomposition.body(1);
    }

    /// Locks in [`do_bounding_spheres_intersect`]'s documented upstream
    /// defect: two bounding spheres whose centers are 0.9 apart with radii
    /// summing to 0.85 (so they do **not** actually touch: 0.9 > 0.85) are
    /// still reported as intersecting, because upstream compares the
    /// **squared** center distance (0.81) against the **unsquared** radius
    /// sum (0.85). This test exists to catch a well-intentioned "fix" to
    /// this function landing silently -- see that function's own doc comment
    /// for why it must not be changed without a separate, explicit sign-off.
    #[test]
    fn do_bounding_spheres_intersect_reports_false_positive_for_non_touching_spheres() {
        let bd1 = Arc::new(
            BodyDecomposition::new(&Shape::Sphere(Sphere::new(0.4).unwrap()), 0.05, 0.0).unwrap(),
        );
        let bd2 = Arc::new(
            BodyDecomposition::new(&Shape::Sphere(Sphere::new(0.45).unwrap()), 0.05, 0.0).unwrap(),
        );

        let mut p1 = PosedBodySphereDecomposition::new(bd1);
        p1.update_pose(Isometry3::identity());
        let mut p2 = PosedBodySphereDecomposition::new(bd2);
        p2.update_pose(Isometry3::from_parts(
            Translation3::new(0.9, 0.0, 0.0),
            UnitQuaternion::identity(),
        ));

        let true_distance = (p1.bounding_sphere_center() - p2.bounding_sphere_center()).norm();
        let radius_sum = p1.bounding_sphere_radius() + p2.bounding_sphere_radius();
        assert!(
            true_distance > radius_sum,
            "test setup must place the spheres genuinely apart"
        );

        assert!(
            do_bounding_spheres_intersect(&p1, &p2),
            "documented defect: squared-distance-vs-unsquared-radii-sum reports a false positive here"
        );
    }

    /// The other side of the same defect, and the side that can lose a
    /// collision rather than merely waste one. Two spheres of radius 1.5 m
    /// whose centres are 2.0 m apart overlap by a full metre, but upstream
    /// compares the **squared** distance (4.0) against the **unsquared**
    /// radius sum (3.0) and culls the pair.
    ///
    /// Separate from the false-positive case above on purpose: they are
    /// opposite sides of the `d = 1` crossover, so one test passing says
    /// nothing about the other. A "fix" that squared the left side instead
    /// of the right would keep the false-positive test green and only this
    /// one would catch it.
    #[test]
    fn do_bounding_spheres_intersect_misses_a_genuine_overlap_past_the_unit_crossover() {
        let bd1 = Arc::new(
            BodyDecomposition::new(&Shape::Sphere(Sphere::new(1.5).unwrap()), 0.05, 0.0).unwrap(),
        );
        let bd2 = Arc::new(
            BodyDecomposition::new(&Shape::Sphere(Sphere::new(1.5).unwrap()), 0.05, 0.0).unwrap(),
        );

        let mut p1 = PosedBodySphereDecomposition::new(bd1);
        p1.update_pose(Isometry3::identity());
        let mut p2 = PosedBodySphereDecomposition::new(bd2);
        p2.update_pose(Isometry3::from_parts(
            Translation3::new(2.0, 0.0, 0.0),
            UnitQuaternion::identity(),
        ));

        let true_distance = (p1.bounding_sphere_center() - p2.bounding_sphere_center()).norm();
        let radius_sum = p1.bounding_sphere_radius() + p2.bounding_sphere_radius();
        assert!(
            true_distance < radius_sum,
            "test setup must place the spheres genuinely overlapping: \
             distance {true_distance} is not below radius sum {radius_sum}"
        );
        assert!(
            true_distance > 1.0,
            "the false-negative branch only exists past the d = 1 crossover"
        );

        assert!(
            !do_bounding_spheres_intersect(&p1, &p2),
            "documented defect: this overlapping pair is culled, because the squared \
             distance ({}) is compared against the unsquared radius sum ({radius_sum})",
            true_distance * true_distance
        );
    }
}
