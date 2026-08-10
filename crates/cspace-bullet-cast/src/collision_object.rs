// Copyright (c) 2017, Southwest Research Institute
// Copyright (c) 2013, John Schulman
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-2-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection_bullet/include/moveit/collision_detection_bullet/bullet_integration/bullet_utils.hpp
//   moveit_core/collision_detection_bullet/src/bullet_integration/bullet_utils.cpp

//! `CollisionObjectWrapper` -- one named body as the continuous check holds it:
//! a Bullet shape, one world transform the shape's own poses are relative to,
//! and the two filter bitfields that decide which pairs the broadphase offers.
//!
//! # What is not carried
//!
//! `shapes_`, `shape_poses_` and `collision_object_types_` are stored upstream
//! and read by exactly two things: `clone`, which passes them straight back
//! into the protected constructor that re-stores them, and `sameObject`, which
//! has no caller in `collision_detection_bullet` at all. Here [`Clone`] shares
//! the built shape through its [`Arc`]s -- which is what upstream's copied
//! `data_` vector achieves -- so the geometry a shape was built from is
//! consumed by [`CollisionObjectWrapper::new`] and not kept.
//!
//! `manage` and `data_` likewise: they are C++'s way of hanging shape lifetime
//! off the wrapper, and the [`Arc`] in `compound::Shape::Convex` already is
//! that lifetime.
//!
//! # The broadphase handle
//!
//! `setBroadphaseHandle` and the proxy-side copy of the two filter bitfields
//! (`bullet_utils.cpp:284-288`) are not here: the proxy is the broadphase's,
//! and the manager that owns the broadphase is the only thing that may write
//! it. [`update_collision_object_filters`] therefore writes the wrapper's two
//! fields and nothing else, and the manager syncs its proxy from them --
//! stated here because until the manager exists that is an invariant with an
//! owner and no enforcement.

use std::collections::BTreeSet;
use std::sync::Arc;

use cspace_bullet::broadphase_proxy::CollisionFilterGroup;
use cspace_bullet::compound::{CompoundShape, Shape as BulletShape};
use cspace_bullet::linear_math::{Matrix3, Scalar, Transform, Vec3};
use cspace_core::geometry::Isometry3;
use cspace_core::geometry::shapes::Shape;

use crate::cast_hull_shape::{
    BULLET_COMPOUND_USE_DYNAMIC_AABB, BULLET_DEFAULT_CONTACT_DISTANCE, BULLET_MARGIN,
};
use crate::contact_test_data::BodyType;
use crate::shape_primitive::{CollisionObjectType, ShapeError, create_shape_primitive};

/// `convertEigenToBt(const Eigen::Isometry3d&)` (`bullet_utils.hpp:92-101`).
///
/// Two conversions in one: the rotation block and the translation, each
/// element `static_cast<btScalar>`-ed. With `btScalar = f32` that cast is the
/// f64-to-f32 rounding every pose on this path goes through exactly once, on
/// the way in.
#[must_use]
pub fn convert_eigen_to_bt(t: &Isometry3) -> Transform {
    let m = t.to_homogeneous();
    let row = |i: usize| {
        Vec3::new(
            m[(i, 0)] as Scalar,
            m[(i, 1)] as Scalar,
            m[(i, 2)] as Scalar,
        )
    };
    let translation = t.translation.vector;
    Transform::new(
        Matrix3::from_rows(row(0), row(1), row(2)),
        Vec3::new(
            translation[0] as Scalar,
            translation[1] as Scalar,
            translation[2] as Scalar,
        ),
    )
}

/// Why a collision object could not be built.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ObjectError {
    /// `throw std::exception()` (`bullet_utils.cpp:553-558`) -- the four-way
    /// check the constructor opens with. Upstream throws one exception for all
    /// of them; the variants below say which arm fired, because they are four
    /// different mistakes by the caller.
    NoShapes,
    /// A shape count and a pose count that differ, or a shape count and a type
    /// count that differ.
    MismatchedLengths {
        /// `shapes.size()`.
        shapes: usize,
        /// `shape_poses.size()`.
        poses: usize,
        /// `collision_object_types.size()`.
        types: usize,
    },
    /// `assert(!name.empty())` (`bullet_utils.cpp:561`).
    EmptyName,
    /// One of the shapes could not be converted -- see [`ShapeError`].
    Shape(ShapeError),
}

impl From<ShapeError> for ObjectError {
    fn from(error: ShapeError) -> Self {
        Self::Shape(error)
    }
}

/// `CollisionObjectWrapper` (`bullet_utils.hpp:109-234`).
pub struct CollisionObjectWrapper {
    /// `name_` -- unique across the manager's map, which is keyed by it.
    name: String,
    /// `type_id_`.
    type_id: BodyType,
    /// `m_collisionFilterGroup` -- which group this object is in.
    pub collision_filter_group: CollisionFilterGroup,
    /// `m_collisionFilterMask` -- which groups it is checked against.
    pub collision_filter_mask: CollisionFilterGroup,
    /// `m_enabled` -- whether the object takes part in a check at all.
    pub enabled: bool,
    /// `touch_links` -- the robot links an attached body is allowed to touch.
    pub touch_links: BTreeSet<String>,
    /// `btCollisionObject::m_worldTransform`. Every shape pose is relative to
    /// this, which is why a multi-shape object stores its children relative to
    /// the *first* shape's pose.
    world_transform: Transform,
    /// `btCollisionObject::m_contactProcessingThreshold` -- how far apart two
    /// objects may be and still be reported. Also the amount
    /// [`Self::get_aabb`] pads by.
    contact_processing_threshold: Scalar,
    /// `btCollisionObject::m_collisionShape`.
    collision_shape: BulletShape,
}

impl CollisionObjectWrapper {
    /// The standard constructor (`bullet_utils.cpp:542-605`).
    ///
    /// `active` picks the filter pair directly rather than through
    /// [`update_collision_object_filters`]: the constructor writes the same two
    /// assignments inline, and a freshly built object has no broadphase proxy
    /// to keep in step.
    ///
    /// # Errors
    ///
    /// [`ObjectError`] for the four ways upstream throws or asserts, and for a
    /// shape `createShapePrimitive` refuses.
    pub fn new(
        name: &str,
        type_id: BodyType,
        shapes: &[Shape],
        shape_poses: &[Isometry3],
        collision_object_types: &[CollisionObjectType],
        active: bool,
    ) -> Result<Self, ObjectError> {
        if shapes.is_empty() || shape_poses.is_empty() || collision_object_types.is_empty() {
            return Err(ObjectError::NoShapes);
        }
        if shapes.len() != shape_poses.len() || shapes.len() != collision_object_types.len() {
            return Err(ObjectError::MismatchedLengths {
                shapes: shapes.len(),
                poses: shape_poses.len(),
                types: collision_object_types.len(),
            });
        }
        if name.is_empty() {
            return Err(ObjectError::EmptyName);
        }

        let (collision_filter_group, collision_filter_mask) = filters_for(active);

        let collision_shape = if shapes.len() == 1 {
            let mut shape = create_shape_primitive(&shapes[0], collision_object_types[0])?;
            shape.set_margin(BULLET_MARGIN);
            shape
        } else {
            let mut compound = CompoundShape::new(BULLET_COMPOUND_USE_DYNAMIC_AABB);
            // margin on compound seems to have no effect when positive but has
            // an effect when negative (`bullet_utils.cpp:586`)
            compound.set_margin(BULLET_MARGIN);

            let inv_world = shape_poses[0].inverse();
            for j in 0..shapes.len() {
                // `if (subshape != nullptr)` (`bullet_utils.cpp:596`): a shape
                // upstream refuses is dropped from the compound rather than
                // failing the object. A shape *this port* has not reached yet
                // is not the same thing and is propagated -- dropping it would
                // silently under-report collisions with nothing to read.
                let mut subshape =
                    match create_shape_primitive(&shapes[j], collision_object_types[j]) {
                        Ok(subshape) => subshape,
                        Err(error) if error.is_upstream_null() => continue,
                        Err(error) => return Err(error.into()),
                    };
                subshape.set_margin(BULLET_MARGIN);
                compound
                    .add_child_shape(convert_eigen_to_bt(&(inv_world * shape_poses[j])), subshape);
            }
            BulletShape::Compound(compound)
        };

        Ok(Self {
            name: name.to_owned(),
            type_id,
            collision_filter_group,
            collision_filter_mask,
            enabled: true,
            touch_links: BTreeSet::new(),
            world_transform: convert_eigen_to_bt(&shape_poses[0]),
            contact_processing_threshold: BULLET_DEFAULT_CONTACT_DISTANCE,
            collision_shape,
        })
    }

    /// `getName`.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `getTypeID`.
    #[must_use]
    pub fn type_id(&self) -> BodyType {
        self.type_id
    }

    /// `btCollisionObject::getWorldTransform`.
    #[must_use]
    pub fn world_transform(&self) -> Transform {
        self.world_transform
    }

    /// `btCollisionObject::setWorldTransform`.
    pub fn set_world_transform(&mut self, transform: Transform) {
        self.world_transform = transform;
    }

    /// `btCollisionObject::getContactProcessingThreshold`.
    #[must_use]
    pub fn contact_processing_threshold(&self) -> Scalar {
        self.contact_processing_threshold
    }

    /// `btCollisionObject::setContactProcessingThreshold`.
    pub fn set_contact_processing_threshold(&mut self, threshold: Scalar) {
        self.contact_processing_threshold = threshold;
    }

    /// `btCollisionObject::getCollisionShape`.
    #[must_use]
    pub fn collision_shape(&self) -> &BulletShape {
        &self.collision_shape
    }

    /// `btCollisionObject::setCollisionShape` and the non-const
    /// `getCollisionShape` in one, crate-visible.
    ///
    /// Not public: the only writers upstream has are
    /// `makeCastCollisionObject`, which swaps the shape for a swept one, and
    /// `setCastCollisionObjectsTransform`, which re-poses that swept shape's
    /// children -- both of them [`crate::cast_object`], both of them inside
    /// this crate. A wrapper whose shape a caller could replace at will would
    /// leave the object's world transform describing geometry that is no
    /// longer there.
    pub(crate) fn collision_shape_mut(&mut self) -> &mut BulletShape {
        &mut self.collision_shape
    }

    /// `getAABB` (`bullet_utils.hpp:169-177`).
    ///
    /// Padded by the contact-processing threshold on every face, so a pair
    /// that is within the requested distance but not yet touching still
    /// reaches the narrow phase. The comment upstream leaves here -- "note
    /// that bullet expands each AABB by 4 cm" -- is about
    /// `btDbvtBroadphase`'s own `gDbvtMargin`, which is applied on top of
    /// this and not by it.
    #[must_use]
    pub fn get_aabb(&self) -> (Vec3, Vec3) {
        let (mut aabb_min, mut aabb_max) = self.collision_shape.get_aabb(&self.world_transform);
        let distance = self.contact_processing_threshold;
        let contact_threshold = Vec3::new(distance, distance, distance);
        aabb_min -= contact_threshold;
        aabb_max += contact_threshold;
        (aabb_min, aabb_max)
    }

    /// `clone` (`bullet_utils.hpp:181-194`).
    ///
    /// "Clones the collision objects but not the collision shape which is
    /// const": the shape is shared, not copied, so a clone's narrow phase runs
    /// against the very same convex data. The broadphase handle is the one
    /// field deliberately *not* carried over -- upstream nulls it, and here it
    /// does not exist -- because a clone has not been added to any broadphase.
    #[must_use]
    pub fn clone_object(&self) -> Self {
        Self {
            name: self.name.clone(),
            type_id: self.type_id,
            collision_filter_group: self.collision_filter_group,
            collision_filter_mask: self.collision_filter_mask,
            enabled: self.enabled,
            touch_links: self.touch_links.clone(),
            world_transform: self.world_transform,
            contact_processing_threshold: self.contact_processing_threshold,
            collision_shape: clone_shape(&self.collision_shape),
        }
    }
}

/// The two filter bitfields, for an active object and for a static one.
///
/// The single place the pair is written: the constructor
/// (`bullet_utils.cpp:563-572`) and `updateCollisionObjectFilters`
/// (`bullet_utils.cpp:272-282`) spell out the same two assignments, and a
/// wrapper whose group says kinematic while its mask says static-only is a
/// state neither of them can produce.
fn filters_for(active: bool) -> (CollisionFilterGroup, CollisionFilterGroup) {
    if active {
        (
            CollisionFilterGroup::KINEMATIC,
            CollisionFilterGroup::KINEMATIC.union(CollisionFilterGroup::STATIC),
        )
    } else {
        (
            CollisionFilterGroup::STATIC,
            CollisionFilterGroup::KINEMATIC,
        )
    }
}

/// `updateCollisionObjectFilters` (`bullet_utils.cpp:270-291`), less the
/// proxy-side copy -- see the module docs.
///
/// `active` empty means every link is active. That rule lives in
/// `isLinkActive` (`contact_checker_common.hpp:50-53`), whose file is
/// Apache-2.0 while this crate is BSD-2-Clause, so it is stated here rather
/// than transcribed: an empty list is "no active set was given", not "nothing
/// is active".
pub fn update_collision_object_filters(active: &[String], cow: &mut CollisionObjectWrapper) {
    let is_active = active.is_empty() || active.iter().any(|link| link == cow.name());
    let (group, mask) = filters_for(is_active);
    cow.collision_filter_group = group;
    cow.collision_filter_mask = mask;
}

/// `isOnlyKinematic` (`bullet_utils.hpp:520-524`).
///
/// True for a self-collision pair: two links of the robot, both active. The
/// broadphase filter uses it to *reject* such a pair on the robot-vs-world
/// check and to require it on the self check.
#[must_use]
pub fn is_only_kinematic(cow0: &CollisionObjectWrapper, cow1: &CollisionObjectWrapper) -> bool {
    cow0.collision_filter_group == CollisionFilterGroup::KINEMATIC
        && cow1.collision_filter_group == CollisionFilterGroup::KINEMATIC
}

/// A [`BulletShape`] sharing every convex leaf with the original, which is what
/// upstream's copied `data_` vector of `shared_ptr` amounts to.
///
/// A compound is rebuilt rather than shared because it owns a mutable
/// `btDbvt`; its *children* are the shared part.
fn clone_shape(shape: &BulletShape) -> BulletShape {
    match shape {
        BulletShape::Convex(convex) => BulletShape::Convex(Arc::clone(convex)),
        BulletShape::Compound(compound) => {
            let mut new_compound = CompoundShape::new(BULLET_COMPOUND_USE_DYNAMIC_AABB);
            for i in 0..compound.num_child_shapes() {
                new_compound.add_child_shape(
                    *compound.child_transform(i),
                    clone_shape(compound.child_shape(i)),
                );
            }
            new_compound.set_margin(compound.margin());
            BulletShape::Compound(new_compound)
        }
    }
}

#[cfg(test)]
mod tests {
    use cspace_core::geometry::Vector3;
    use cspace_core::geometry::shapes::{Cuboid, Sphere};

    use super::*;

    fn cuboid(size: f64) -> Shape {
        Shape::Cuboid(Cuboid {
            size: [size, size, size],
        })
    }

    fn at(x: f64, y: f64, z: f64) -> Isometry3 {
        Isometry3::translation(x, y, z)
    }

    fn one_box() -> CollisionObjectWrapper {
        CollisionObjectWrapper::new(
            "link",
            BodyType::RobotLink,
            &[cuboid(2.0)],
            &[at(1.0, 0.0, 0.0)],
            &[CollisionObjectType::UseShapeType],
            true,
        )
        .unwrap()
    }

    /// A single shape does not get a compound wrapped round it, and the
    /// object's world transform is that shape's pose.
    #[test]
    fn a_single_shape_becomes_the_collision_shape_itself() {
        let cow = one_box();
        assert!(matches!(cow.collision_shape(), BulletShape::Convex(_)));
        assert_eq!(cow.world_transform().origin, Vec3::new(1.0, 0.0, 0.0));
    }

    /// Several shapes become a compound whose children are relative to the
    /// *first* pose, not to the world -- so child 0 sits at the origin of the
    /// compound however far from the world origin the object is.
    #[test]
    fn several_shapes_are_posed_relative_to_the_first() {
        let cow = CollisionObjectWrapper::new(
            "link",
            BodyType::RobotLink,
            &[cuboid(1.0), cuboid(1.0)],
            &[at(10.0, 0.0, 0.0), at(10.0, 3.0, 0.0)],
            &[CollisionObjectType::UseShapeType; 2],
            true,
        )
        .unwrap();

        let BulletShape::Compound(compound) = cow.collision_shape() else {
            panic!("two shapes make a compound");
        };
        assert_eq!(compound.num_child_shapes(), 2);
        assert_eq!(compound.child_transform(0).origin, Vec3::new(0.0, 0.0, 0.0));
        assert_eq!(compound.child_transform(1).origin, Vec3::new(0.0, 3.0, 0.0));
        assert_eq!(cow.world_transform().origin, Vec3::new(10.0, 0.0, 0.0));
    }

    /// The AABB is padded by the contact threshold, which is what makes a
    /// nonzero requested distance reach the narrow phase at all.
    #[test]
    fn the_aabb_grows_by_the_contact_threshold() {
        let mut cow = one_box();
        let (tight_min, tight_max) = cow.get_aabb();
        cow.set_contact_processing_threshold(0.5);
        let (padded_min, padded_max) = cow.get_aabb();

        assert_eq!(tight_min, Vec3::new(0.0, -1.0, -1.0));
        assert_eq!(tight_max, Vec3::new(2.0, 1.0, 1.0));
        assert_eq!(padded_min, Vec3::new(-0.5, -1.5, -1.5));
        assert_eq!(padded_max, Vec3::new(2.5, 1.5, 1.5));
    }

    /// The four constructor refusals, each named rather than folded into one
    /// `std::exception`.
    #[test]
    fn the_constructor_refuses_each_malformed_input_distinguishably() {
        let mk =
            |name: &str, shapes: &[Shape], poses: &[Isometry3], types: &[CollisionObjectType]| {
                CollisionObjectWrapper::new(name, BodyType::RobotLink, shapes, poses, types, true)
                    .err()
            };
        let box_ = [cuboid(1.0)];
        let pose = [at(0.0, 0.0, 0.0)];
        let types = [CollisionObjectType::UseShapeType];

        assert_eq!(mk("link", &[], &pose, &types), Some(ObjectError::NoShapes));
        assert_eq!(mk("link", &box_, &[], &types), Some(ObjectError::NoShapes));
        assert_eq!(mk("link", &box_, &pose, &[]), Some(ObjectError::NoShapes));
        assert_eq!(
            mk("link", &box_, &[pose[0], pose[0]], &types),
            Some(ObjectError::MismatchedLengths {
                shapes: 1,
                poses: 2,
                types: 1
            })
        );
        assert_eq!(mk("", &box_, &pose, &types), Some(ObjectError::EmptyName));
    }

    /// An inactive object is in the static group and is checked only against
    /// kinematic ones, and `updateCollisionObjectFilters` moves an object
    /// between the two by name.
    #[test]
    fn the_active_list_decides_the_filter_pair_and_an_empty_list_means_all() {
        let mut cow = one_box();
        assert_eq!(cow.collision_filter_group, CollisionFilterGroup::KINEMATIC);

        update_collision_object_filters(&["other".to_owned()], &mut cow);
        assert_eq!(cow.collision_filter_group, CollisionFilterGroup::STATIC);
        assert_eq!(cow.collision_filter_mask, CollisionFilterGroup::KINEMATIC);

        update_collision_object_filters(&[], &mut cow);
        assert_eq!(cow.collision_filter_group, CollisionFilterGroup::KINEMATIC);
        assert_eq!(
            cow.collision_filter_mask,
            CollisionFilterGroup::KINEMATIC.union(CollisionFilterGroup::STATIC)
        );

        update_collision_object_filters(&["link".to_owned()], &mut cow);
        assert_eq!(cow.collision_filter_group, CollisionFilterGroup::KINEMATIC);
    }

    /// `isOnlyKinematic` is what tells a self-collision pair from a
    /// robot-versus-world one.
    #[test]
    fn only_two_active_objects_are_only_kinematic() {
        let active = one_box();
        let mut inactive = one_box();
        update_collision_object_filters(&["other".to_owned()], &mut inactive);

        assert!(is_only_kinematic(&active, &active));
        assert!(!is_only_kinematic(&active, &inactive));
        assert!(!is_only_kinematic(&inactive, &inactive));
    }

    /// A clone shares its convex leaves rather than copying them, which is the
    /// property `makeCastCollisionObject` depends on: the cast object sweeps
    /// the same shape the original stands on.
    #[test]
    fn a_clone_shares_the_shape_and_copies_everything_else() {
        let mut cow = one_box();
        cow.touch_links.insert("gripper".to_owned());
        cow.set_contact_processing_threshold(0.25);

        let clone = cow.clone_object();
        assert_eq!(clone.name(), "link");
        assert_eq!(clone.touch_links, cow.touch_links);
        assert_eq!(clone.contact_processing_threshold(), 0.25);

        let (BulletShape::Convex(original), BulletShape::Convex(cloned)) =
            (cow.collision_shape(), clone.collision_shape())
        else {
            panic!("a single shape stays convex");
        };
        assert!(Arc::ptr_eq(original, cloned), "the shape is shared");
    }

    /// A compound is rebuilt on clone -- it owns a mutable tree -- but its
    /// children are still the same shapes.
    #[test]
    fn a_cloned_compound_is_a_new_tree_over_the_same_children() {
        let cow = CollisionObjectWrapper::new(
            "link",
            BodyType::RobotLink,
            &[cuboid(1.0), Shape::Sphere(Sphere { radius: 0.5 })],
            &[at(0.0, 0.0, 0.0), at(2.0, 0.0, 0.0)],
            &[CollisionObjectType::UseShapeType; 2],
            true,
        )
        .unwrap();
        let clone = cow.clone_object();

        let (BulletShape::Compound(original), BulletShape::Compound(cloned)) =
            (cow.collision_shape(), clone.collision_shape())
        else {
            panic!("two shapes make a compound");
        };
        assert_eq!(cloned.num_child_shapes(), original.num_child_shapes());
        for i in 0..original.num_child_shapes() {
            assert_eq!(cloned.child_transform(i), original.child_transform(i));
            let (BulletShape::Convex(a), BulletShape::Convex(b)) =
                (original.child_shape(i), cloned.child_shape(i))
            else {
                panic!("both children are convex");
            };
            assert!(Arc::ptr_eq(a, b), "child {i} is shared");
        }
        assert!(
            cloned.dynamic_aabb_tree().is_some(),
            "the clone has its own tree"
        );
    }

    /// The f64-to-f32 rounding happens once, on the way in, and the basis is
    /// row-major -- a transposed conversion would put the translation in the
    /// right place and the rotation in the wrong one, which no origin check
    /// would catch.
    #[test]
    fn the_conversion_keeps_rows_as_rows() {
        let quarter_turn_about_z = Isometry3::new(
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::z() * std::f64::consts::FRAC_PI_2,
        );
        let bt = convert_eigen_to_bt(&quarter_turn_about_z);

        assert_eq!(bt.origin, Vec3::new(1.0, 2.0, 3.0));
        // +x maps to +y, so row 1 -- not row 0 -- is the one with the 1 in
        // column 0.
        assert!((bt.basis.rows[0].x - 0.0).abs() < 1e-7);
        assert!((bt.basis.rows[0].y + 1.0).abs() < 1e-7);
        assert!((bt.basis.rows[1].x - 1.0).abs() < 1e-7);
        assert!((bt.basis.rows[2].z - 1.0).abs() < 1e-7);
    }
}
