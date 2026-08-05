// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Behaviorally derived from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_state/include/moveit/robot_state/attached_body.hpp

//! [`AttachedBody`]: geometry rigidly attached to a robot link.
//!
//! # Deviation from upstream
//!
//! Upstream stores attached bodies inside `moveit::core::RobotState` itself
//! (`RobotState::attachBody`/`getAttachedBody`/`hasAttachedBody`/...).
//! `moveit_state::RobotState` does not carry that concept yet — its own
//! crate doc lists "no attached bodies" under deferred scope. Rather than
//! let [`crate::PlanningScene`] shadow a second, parallel notion of
//! "attached" next to a `RobotState` that has none, this crate is the sole
//! owner of attached-body data for now:
//! [`crate::PlanningScene::attached_bodies`] is the one place this state
//! lives, not a cache duplicating something `RobotState` also tracks. When
//! `RobotState` gains attached-body support, this module's contents belong
//! there instead, and `PlanningScene` goes back to delegating to it — the
//! same relationship it already has with upstream's real design.
//!
//! Also unlike upstream, [`AttachedBody::shape_poses`] are stored directly
//! relative to the attach link's own frame, rather than relative to an
//! intermediate "pose in link" that itself holds the object's frame within
//! the link (upstream `AttachedBody::pose_`/`shape_poses_`: two levels).
//! Nothing here needs that second level: composing it away up front means
//! [`crate::PlanningScene::detach`] only ever needs one transform (the
//! link's current global pose) to recompute every shape's current global
//! pose, not two chained ones. [`AttachedBody::subframe_pose`] follows the
//! same one-level rule, for the same reason.
//!
//! One consequence of collapsing upstream's `pose_` away: this port has no
//! stored value standing in for "the body's own frame" the way upstream's
//! `pose_`/`global_pose_` do. [`crate::PlanningScene::frame_transform`]
//! resolving a bare attached-body id (upstream's `AttachedBody::getGlobalPose`
//! tier) therefore treats that missing `pose_` as `Isometry3::identity()` —
//! see that method's doc for why this is forced by the one-level design
//! already committed to here, not a free choice, and how the oracle's own
//! round-2 reconciliation (`attachBody`'s `pose` argument always `Identity`)
//! already made the same call.
//!
//! `detach_posture` (a `trajectory_msgs` type, D1) is not carried.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use moveit_collision::AttachedBodyGeometry;
use moveit_error::Result;
use moveit_geometry::{Isometry3, Shape};

/// Geometry rigidly attached to a robot link. See the module doc for how
/// this differs from upstream `moveit::core::AttachedBody`.
#[derive(Debug, Clone)]
pub struct AttachedBody {
    id: String,
    link_name: String,
    shapes: Vec<Arc<Shape>>,
    shape_poses: Vec<Isometry3>,
    touch_links: BTreeSet<String>,
    subframes: BTreeMap<String, Isometry3>,
}

impl AttachedBody {
    pub(crate) fn new(
        id: String,
        link_name: String,
        shapes: Vec<Arc<Shape>>,
        shape_poses: Vec<Isometry3>,
        touch_links: BTreeSet<String>,
        subframes: BTreeMap<String, Isometry3>,
    ) -> Self {
        Self {
            id,
            link_name,
            shapes,
            shape_poses,
            touch_links,
            subframes,
        }
    }

    /// This body's id. Upstream `getName`.
    pub fn id(&self) -> &str {
        &self.id
    }

    /// The link this body is attached to. Upstream `getAttachedLinkName`.
    pub fn link_name(&self) -> &str {
        &self.link_name
    }

    /// This body's shapes. Upstream `getShapes`.
    pub fn shapes(&self) -> &[Arc<Shape>] {
        &self.shapes
    }

    /// Each shape's pose relative to [`AttachedBody::link_name`]'s own
    /// frame — see the module doc for why this is one level, not upstream's
    /// two. Upstream `getShapePoses()` composed with `getPose()`.
    pub fn shape_poses(&self) -> &[Isometry3] {
        &self.shape_poses
    }

    /// Links this body is allowed to touch without that counting as a
    /// collision. Upstream `getTouchLinks`.
    pub fn touch_links(&self) -> &BTreeSet<String> {
        &self.touch_links
    }

    /// This body's subframe pose relative to [`AttachedBody::link_name`]'s
    /// own frame, if `name` names one — see the module doc for why this is
    /// one level, not upstream's two. Upstream `getSubframeTransform`,
    /// restricted to the lookup key already having the body id stripped:
    /// upstream's own key is `"<id>/<name>"` (`attached_body.cpp:139-155`);
    /// callers here match that spelling explicitly (see
    /// [`crate::PlanningScene::frame_transform`]) rather than this method
    /// re-parsing it, the same split [`moveit_collision::World`] already
    /// uses between its subframe-suffix parsing and `Object::subframe_pose`.
    pub fn subframe_pose(&self, name: &str) -> Option<Isometry3> {
        self.subframes.get(name).copied()
    }

    /// Every subframe name on this body (bare, without the `"<id>/"` prefix
    /// — see [`AttachedBody::subframe_pose`]).
    pub fn subframe_names(&self) -> impl Iterator<Item = &str> {
        self.subframes.keys().map(String::as_str)
    }

    /// Borrows this body's fields as a [`moveit_collision::AttachedBodyGeometry`]
    /// — the view a [`moveit_collision::CollisionEnv`] backend needs, without
    /// `moveit-collision` depending back on this crate. See
    /// [`crate::PlanningScene`]'s "Collision checking" doc for how this is used.
    pub fn as_geometry(&self) -> AttachedBodyGeometry<'_> {
        AttachedBodyGeometry {
            id: &self.id,
            link_name: &self.link_name,
            shapes: &self.shapes,
            shape_poses: &self.shape_poses,
            touch_links: &self.touch_links,
        }
    }

    /// Uniformly scale every shape on this body. Upstream `setScale`
    /// (`attached_body.cpp:86-103`).
    ///
    /// # Which shapes are mutated in place, and which are cloned
    ///
    /// Upstream branches per shape on `shape.use_count() == 1`: sole owner,
    /// `const_cast` the `ShapeConstPtr` and scale through it; otherwise
    /// `clone()`, scale the copy, and `reset` the vector entry onto it.
    /// [`Arc::make_mut`] is that policy in one call, with one difference:
    /// it clones when the strong count exceeds one **or** any
    /// [`std::sync::Weak`] is outstanding, whereas `use_count()` counts
    /// strong owners only.
    ///
    /// That difference is reachable here, and is in fact systematic for any
    /// body that has ever been distance-field decomposed:
    /// `moveit-distance-field`'s process-wide body-decomposition cache
    /// stores an `Arc::downgrade(shape)` beside every entry
    /// (`crates/moveit-distance-field/src/collision_common_distance_field.rs:511`)
    /// and never evicts, and attached-body shapes reach it through that
    /// crate's `attached_body_sphere_decomposition` and
    /// `attached_body_point_decomposition`. Once a body has been decomposed,
    /// every call here clones its shapes even though this body holds the
    /// only strong reference. The direction is the safe one -- no shape
    /// another party can still observe is mutated underneath it -- and the
    /// clone also gives that cache a fresh key, rather than leaving a
    /// decomposition computed for the pre-scale dimensions keyed at the
    /// address the post-scale shape now occupies.
    ///
    /// # Errors
    ///
    /// Upstream's `setScale` is `void` and its loop has no failure path.
    /// This one propagates [`moveit_geometry::Shape::scale`]'s error, which
    /// this port has because a scale driving a dimension below zero is
    /// rejected there. Whether the `geometric_shapes` original can fail at
    /// the same point is not established here -- that package is not under
    /// the pinned `moveit2` checkout and was not read for this change -- so
    /// what follows is this port's own contract, not a parity claim.
    ///
    /// Application is not transactional: the `?` leaves every shape before
    /// the failing one carrying its new dimensions, matching the partial
    /// state upstream's own loop would leave on an early exit. The failing
    /// shape keeps its old dimensions, but [`Arc::make_mut`] has already
    /// replaced a shared `Arc` with an unshared clone of it by the time the
    /// error is returned -- upstream's clone branch never reaches its
    /// `shape.reset(copy)`, so there the original stays shared.
    pub fn set_scale(&mut self, scale: f64) -> Result<()> {
        self.apply_to_shapes(|shape| shape.scale(scale))
    }

    /// Add uniform padding to every shape on this body. Upstream
    /// `setPadding` (`attached_body.cpp:120-137`), whose body is upstream
    /// `setScale`'s with `padd(padding)` in place of `scale(scale)`.
    ///
    /// # Errors
    ///
    /// [`moveit_geometry::Shape::padd`]'s error, under exactly the sharing,
    /// cloning and partial-application rules documented on
    /// [`AttachedBody::set_scale`]. Two inputs reach it: a padding more
    /// negative than a shape's smallest dimension, and a
    /// [`moveit_geometry::Shape::Mesh`] still carrying `vertex_normals: None`,
    /// which is what a mesh built by `moveit_geometry::Mesh::new` has until
    /// `compute_vertex_normals` runs. Upstream cannot see the second one --
    /// every `geometric_shapes` creation entry point computes the normals
    /// before the mesh escapes -- so it is this port's own reachable path, and
    /// the error is returned rather than asserted away.
    pub fn set_padding(&mut self, padding: f64) -> Result<()> {
        self.apply_to_shapes(|shape| shape.padd(padding))
    }

    /// The loop upstream writes out twice, once in `setScale` and once in
    /// `setPadding`, with only the per-shape call differing. See
    /// [`AttachedBody::set_scale`] for what [`Arc::make_mut`] does and does
    /// not reproduce of upstream's `use_count() == 1` branch.
    fn apply_to_shapes(&mut self, mut update: impl FnMut(&mut Shape) -> Result<()>) -> Result<()> {
        for shape in &mut self.shapes {
            update(Arc::make_mut(shape))?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use moveit_geometry::Sphere;

    use super::*;

    /// Every radius, scale and padding in this module is a dyadic rational
    /// (a multiple of a power of two), so every product and sum below is
    /// exact in IEEE-754 double and the assertions are `assert_eq!` on the
    /// nose. There is no tolerance to size because there is no rounding:
    /// `0.25 * 2.0`, `0.5 - 0.375` and `1.0 * 0.25 + 0.0` are each
    /// representable exactly.
    fn body_with_spheres(radii: &[f64]) -> AttachedBody {
        AttachedBody::new(
            "attached".to_owned(),
            "link".to_owned(),
            radii
                .iter()
                .map(|&r| {
                    Arc::new(Shape::Sphere(
                        Sphere::new(r).expect("fixture radii are non-negative"),
                    ))
                })
                .collect(),
            radii.iter().map(|_| Isometry3::identity()).collect(),
            BTreeSet::new(),
            BTreeMap::new(),
        )
    }

    fn radius(shape: &Shape) -> f64 {
        match shape {
            Shape::Sphere(sphere) => sphere.radius,
            other => panic!("this module's fixtures hold spheres only, got {other:?}"),
        }
    }

    /// Upstream's `else` branch: a shape someone else also holds is copied,
    /// the copy is scaled, and the body's entry is repointed at it. The
    /// other owner must not see the new dimensions.
    #[test]
    fn a_shape_shared_with_another_owner_is_cloned_rather_than_mutated_in_place() {
        let mut body = body_with_spheres(&[0.25]);
        let other_owner = Arc::clone(&body.shapes()[0]);

        body.set_scale(2.0)
            .expect("scaling a 0.25 sphere by 2 is valid");

        assert!(
            !Arc::ptr_eq(&body.shapes()[0], &other_owner),
            "the body's entry must be a different allocation from the one the other owner holds"
        );
        assert_eq!(
            radius(&other_owner),
            0.25,
            "the other owner's shape must not move underneath it"
        );
        assert_eq!(radius(&body.shapes()[0]), 0.5);
    }

    /// Upstream's `use_count() == 1` branch: sole owner, so the allocation is
    /// reused rather than replaced.
    #[test]
    fn an_unshared_shape_is_mutated_in_place_without_a_clone() {
        let mut body = body_with_spheres(&[0.25]);
        let before = Arc::as_ptr(&body.shapes()[0]);
        assert_eq!(
            Arc::strong_count(&body.shapes()[0]),
            1,
            "the fixture must leave the body as the sole strong owner"
        );

        body.set_scale(2.0)
            .expect("scaling a 0.25 sphere by 2 is valid");

        assert_eq!(
            Arc::as_ptr(&body.shapes()[0]),
            before,
            "sole strong owner and no outstanding Weak: the allocation must be reused"
        );
        assert_eq!(radius(&body.shapes()[0]), 0.5);
    }

    /// The one place [`Arc::make_mut`] is *not* upstream's `use_count() == 1`:
    /// a live [`std::sync::Weak`] forces a clone that `use_count()`, which
    /// counts strong owners only, would not. See [`AttachedBody::set_scale`]
    /// for the producer of such a `Weak` in this tree and why the extra clone
    /// is the safe direction.
    #[test]
    fn an_outstanding_weak_forces_a_clone_upstreams_use_count_would_not() {
        let mut body = body_with_spheres(&[0.25]);
        let before = Arc::as_ptr(&body.shapes()[0]);
        let weak = Arc::downgrade(&body.shapes()[0]);
        assert_eq!(
            Arc::strong_count(&body.shapes()[0]),
            1,
            "the body is still the only strong owner -- upstream would take the in-place branch"
        );

        body.set_scale(2.0)
            .expect("scaling a 0.25 sphere by 2 is valid");

        assert_ne!(
            Arc::as_ptr(&body.shapes()[0]),
            before,
            "Arc::make_mut clones while any Weak is outstanding"
        );
        assert_eq!(radius(&body.shapes()[0]), 0.5);
        assert_eq!(
            weak.strong_count(),
            0,
            "make_mut moved the value out of the old allocation, leaving the Weak un-upgradable"
        );
    }

    /// Boundary: the identity scale. It still runs the whole loop, so this
    /// also pins that a no-op update does not churn allocations.
    #[test]
    fn a_scale_of_one_changes_no_dimension_and_no_allocation() {
        let mut body = body_with_spheres(&[0.25, 0.5]);
        let before: Vec<_> = body.shapes().iter().map(Arc::as_ptr).collect();

        body.set_scale(1.0).expect("the identity scale is valid");

        assert_eq!(
            body.shapes().iter().map(Arc::as_ptr).collect::<Vec<_>>(),
            before
        );
        assert_eq!(radius(&body.shapes()[0]), 0.25);
        assert_eq!(radius(&body.shapes()[1]), 0.5);
    }

    /// Boundary: the identity padding, the additive counterpart of
    /// [`a_scale_of_one_changes_no_dimension_and_no_allocation`].
    #[test]
    fn a_padding_of_zero_changes_no_dimension_and_no_allocation() {
        let mut body = body_with_spheres(&[0.25, 0.5]);
        let before: Vec<_> = body.shapes().iter().map(Arc::as_ptr).collect();

        body.set_padding(0.0).expect("zero padding is valid");

        assert_eq!(
            body.shapes().iter().map(Arc::as_ptr).collect::<Vec<_>>(),
            before
        );
        assert_eq!(radius(&body.shapes()[0]), 0.25);
        assert_eq!(radius(&body.shapes()[1]), 0.5);
    }

    /// Boundary: negative padding is legal and shrinks, right up to the point
    /// where it would drive a dimension below zero.
    #[test]
    fn negative_padding_shrinks_a_shape_that_can_absorb_it() {
        let mut body = body_with_spheres(&[0.5]);

        body.set_padding(-0.375)
            .expect("0.5 - 0.375 is still a valid radius");

        assert_eq!(radius(&body.shapes()[0]), 0.125);
    }

    /// Boundary: negative padding that overruns a shape is rejected, and the
    /// rejection is not transactional -- the shapes already updated keep
    /// their new dimensions. The fixture is ordered so the failure lands on
    /// the *second* shape, which is the only arrangement that can tell
    /// "stopped at the failure" apart from "rolled everything back".
    #[test]
    fn negative_padding_larger_than_a_shape_is_rejected_after_updating_its_predecessors() {
        let mut body = body_with_spheres(&[0.5, 0.25]);

        let err = body
            .set_padding(-0.375)
            .expect_err("0.25 - 0.375 is a negative radius");
        assert_eq!(
            err.to_string(),
            "construction failed: Sphere radius must be non-negative.",
            "the error must be the shape layer's own dimension rejection, \
             rendered whole rather than substring-matched -- `Error::Construct` \
             is a shared catch-all and only the message says which check fired \
             (see `moveit_error::Error`'s own doc on that)"
        );

        assert_eq!(
            radius(&body.shapes()[0]),
            0.125,
            "the shape before the failure keeps the padding already applied to it"
        );
        assert_eq!(
            radius(&body.shapes()[1]),
            0.25,
            "the failing shape itself is left at its old dimensions"
        );
    }

    /// Boundary: no shapes at all. Both loops run zero times and report
    /// success, as upstream's `for` over an empty `shapes_` does.
    #[test]
    fn a_body_with_no_shapes_accepts_both_and_stays_empty() {
        let mut body = body_with_spheres(&[]);

        body.set_scale(2.0)
            .expect("a shapeless body scales trivially");
        body.set_padding(-1.0)
            .expect("a shapeless body has no dimension for padding to overrun");

        assert_eq!(body.shapes().len(), 0);
    }
}
