// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/CollisionShapes/btCompoundShape.h
//   bullet3/src/BulletCollision/CollisionShapes/btCompoundShape.cpp

//! `btCompoundShape` -- several posed shapes behind one collision shape, and
//! the [`Dbvt`] that decides the order their children are visited in.
//!
//! # Why the continuous path cannot avoid this
//!
//! Every collision object MoveIt builds is a compound: `createShapePrimitive`
//! wraps even a single box in one (`bullet_utils.cpp:584-601`), and
//! `makeCastCollisionObject` rebuilds that compound with each convex child
//! replaced by a `CastHullShape` -- and each *compound* child replaced by a
//! second compound of `CastHullShape` (`bullet_utils.cpp:315-361`). So the
//! continuous path dispatches compound-vs-compound at the top and can nest one
//! more level below it, which is why [`Shape`] is recursive.
//!
//! # A sum type, not a downcast
//!
//! Upstream a child is a `btCollisionShape*` and the traversals ask
//! `getShapeType()` and `static_cast`. Here a child is a [`Shape`], whose two
//! variants are exactly the two cases those casts distinguish. The dispatch
//! then matches instead of casting, so "this pointer said compound but is not
//! one" is not a state the port can reach. [`Shape::shape_type`] still exists
//! and still reports the upstream value, because
//! `btCollisionDispatcher::findAlgorithm` selects on that value and a
//! `CastHullShape` -- which this crate does not define -- reports
//! `CUSTOM_CONVEX_SHAPE_TYPE` through the same trait method.
//!
//! # Not ported
//!
//! `removeChildShapeByIndex` and `removeChildShape`: the continuous path
//! builds each compound with `addChildShape` and afterwards only ever calls
//! `updateChildTransform` (`bullet_cast_bvh_manager.cpp:102`, `:115`).
//!
//! The `static_cast<CastHullShape*>` that `setCastCollisionObjectsTransform`
//! applies to a convex child (`bullet_cast_bvh_manager.cpp:101`, `:114`) has
//! no equivalent here, and [`CompoundShape::child_shape_mut`] does not offer
//! one: the `Convex` arm holds an `Arc` the cast layer shares with the
//! compound, and it re-poses the shape through that shared handle. What the
//! mutable accessor is for is the *compound* child of a compound, whose own
//! tree the same function refreshes.
//!
//! `m_updateRevision`: its only readers are the two compound algorithms'
//! child-algorithm caches, which this crate does not carry -- see
//! `crate::compound_algorithm` for why those caches have no observable effect
//! here. It is also never bumped on this path: only `addChildShape` and the
//! two removes increment it, and `updateChildTransform` does not.
//!
//! `setLocalScaling`, `calculateLocalInertia`,
//! `calculatePrincipalAxisTransform`, `createAabbTreeFromChildren` and
//! `serialize`: written by or for the dynamics pipeline and the serializer,
//! neither of which exists here. `m_localScaling` holds its constructed
//! `(1,1,1)` for the whole life of every compound MoveIt builds, so the
//! multiplications it would take part in are all by one.

use std::sync::Arc;

use crate::broadphase_proxy::BroadphaseNativeType;
use crate::dbvt::{Dbvt, DbvtVolume};
use crate::linear_math::{BT_LARGE_FLOAT, Scalar, Transform, Vec3};
use crate::shapes::ConvexShape;

/// A `btCollisionShape*` as the compound traversals actually use it: either
/// something convex that the narrow phase can take support points from, or
/// another compound to descend into.
///
/// See the module docs for why this is a sum type rather than a trait object
/// with a downcast.
pub enum Shape {
    /// Anything reaching the narrow phase, including shapes defined outside
    /// this crate -- MoveIt's `CastHullShape` is the one the continuous path
    /// actually puts here.
    ///
    /// Shared, not owned, because upstream shares it: a child is a
    /// `btCollisionShape*` whose lifetime hangs off the collision object's
    /// `data_` vector of `std::shared_ptr<void>`, and
    /// `makeCastCollisionObject` builds a second compound whose children are
    /// `CastHullShape`s **wrapping these same pointers**
    /// (`bullet_utils.cpp:323-332`). A uniquely-owned child would force that
    /// rebuild to copy a shape it is documented not to copy --
    /// `CollisionObjectWrapper::clone` says "clones the collision objects but
    /// not the collision shape which is const".
    Convex(Arc<dyn ConvexShape>),
    /// `btCompoundShape`.
    Compound(CompoundShape),
}

impl Shape {
    /// `btCollisionShape::getShapeType`.
    #[must_use]
    pub fn shape_type(&self) -> BroadphaseNativeType {
        match self {
            Self::Convex(shape) => shape.shape_type(),
            Self::Compound(_) => BroadphaseNativeType::COMPOUND_SHAPE,
        }
    }

    /// `btCollisionShape::getAabb`.
    #[must_use]
    pub fn get_aabb(&self, t: &Transform) -> (Vec3, Vec3) {
        match self {
            Self::Convex(shape) => shape.get_aabb(t),
            Self::Compound(compound) => compound.get_aabb(t),
        }
    }

    /// `btCollisionShape::getMargin`.
    #[must_use]
    pub fn margin(&self) -> Scalar {
        match self {
            Self::Convex(shape) => shape.margin(),
            Self::Compound(compound) => compound.margin(),
        }
    }

    /// `btCollisionShape::setMargin`, on a shape that is not yet shared.
    ///
    /// Every `setMargin` call on this path is the `BULLET_MARGIN` one a
    /// freshly-built shape takes from its builder -- `bullet_utils.cpp:577`,
    /// `:587`, `:599`, and the three in `makeCastCollisionObject` -- and at
    /// each of them the shape has exactly one owner, so the `Arc` is unique.
    /// A shape already handed to a compound is const from then on: nothing on
    /// the continuous path re-margins one.
    ///
    /// # Panics
    ///
    /// If the shape is already shared, which would silently margin nothing.
    pub fn set_margin(&mut self, margin: Scalar) {
        match self {
            Self::Convex(shape) => Arc::get_mut(shape)
                .expect("a shape is margined by its builder, before it is shared")
                .set_margin(margin),
            Self::Compound(compound) => compound.set_margin(margin),
        }
    }
}

/// `btCompoundShapeChild` (`btCompoundShape.h:30-40`).
///
/// `m_childShapeType` and `m_childMargin` are absent: upstream caches them
/// beside the pointer and then reads them back only in `operator==` and in
/// `btCompoundShape`'s serializer. Every live query goes through the child
/// shape itself, which this port holds by value.
pub struct CompoundShapeChild {
    /// `m_transform` -- the child's pose in the compound's frame.
    pub transform: Transform,
    /// `m_childShape`.
    pub shape: Shape,
    /// `m_node` -- the leaf this child owns in the compound's tree, `None`
    /// when the compound was built without one.
    pub node: Option<usize>,
}

/// `btCompoundShape` (`btCompoundShape.h:95-...`).
pub struct CompoundShape {
    /// `m_children`.
    children: Vec<CompoundShapeChild>,
    /// `m_localAabbMin`.
    local_aabb_min: Vec3,
    /// `m_localAabbMax`.
    local_aabb_max: Vec3,
    /// `m_dynamicAabbTree`. MoveIt passes `enableDynamicAabbTree = true` at
    /// every construction site (`BULLET_COMPOUND_USE_DYNAMIC_AABB`,
    /// `bullet_utils.hpp:57`), so on the continuous path this is always
    /// `Some` -- but the `None` arm is what the traversals' index-order
    /// fallback is for, and it is reachable for any caller that asks.
    dynamic_aabb_tree: Option<Dbvt>,
    /// `m_collisionMargin`.
    collision_margin: Scalar,
}

impl CompoundShape {
    /// `btCompoundShape(enableDynamicAabbTree, initialChildCapacity)`
    /// (`btCompoundShape.cpp:21-39`).
    ///
    /// `initialChildCapacity` is a `reserve` and has no effect on any result,
    /// so it is not a parameter here.
    #[must_use]
    pub fn new(enable_dynamic_aabb_tree: bool) -> Self {
        Self {
            children: Vec::new(),
            local_aabb_min: Vec3::new(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT),
            local_aabb_max: Vec3::new(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT),
            dynamic_aabb_tree: enable_dynamic_aabb_tree.then(Dbvt::new),
            collision_margin: 0.0,
        }
    }

    /// `addChildShape` (`btCompoundShape.cpp:50-84`).
    ///
    /// The local AABB grows here per axis rather than through
    /// [`CompoundShape::recalculate_local_aabb`], which is why an empty
    /// compound keeps the inverted `+/-BT_LARGE_FLOAT` extents rather than a
    /// zero box -- `get_aabb` special-cases that.
    pub fn add_child_shape(&mut self, local_transform: Transform, shape: Shape) {
        let (local_aabb_min, local_aabb_max) = shape.get_aabb(&local_transform);
        for i in 0..3 {
            if self.local_aabb_min[i] > local_aabb_min[i] {
                self.local_aabb_min[i] = local_aabb_min[i];
            }
            if self.local_aabb_max[i] < local_aabb_max[i] {
                self.local_aabb_max[i] = local_aabb_max[i];
            }
        }

        let index = self.children.len();
        let node = self.dynamic_aabb_tree.as_mut().map(|tree| {
            tree.insert(
                DbvtVolume::from_mm(local_aabb_min, local_aabb_max),
                i32::try_from(index).expect("fewer than i32::MAX children"),
            )
        });

        self.children.push(CompoundShapeChild {
            transform: local_transform,
            shape,
            node,
        });
    }

    /// `getNumChildShapes`.
    #[must_use]
    pub fn num_child_shapes(&self) -> usize {
        self.children.len()
    }

    /// `getChildShape` (`btCompoundShape.h:95-98`).
    #[must_use]
    pub fn child_shape(&self, index: usize) -> &Shape {
        &self.children[index].shape
    }

    /// `getChildShape`, mutably -- what a nested compound needs, because its
    /// own `updateChildTransform` and `recalculateLocalAabb` are called on it
    /// through its parent (`bullet_cast_bvh_manager.cpp:106-117`).
    ///
    /// A *convex* child is not reachable for mutation through here: the arm
    /// holds an `Arc` the caller shares with whoever built it, which is the
    /// point -- see the module docs on the downcast this replaces.
    pub fn child_shape_mut(&mut self, index: usize) -> &mut Shape {
        &mut self.children[index].shape
    }

    /// `getChildTransform` (`btCompoundShape.h:100-107`).
    #[must_use]
    pub fn child_transform(&self, index: usize) -> &Transform {
        &self.children[index].transform
    }

    /// `getDynamicAabbTree`.
    #[must_use]
    pub fn dynamic_aabb_tree(&self) -> Option<&Dbvt> {
        self.dynamic_aabb_tree.as_ref()
    }

    /// `updateChildTransform` (`btCompoundShape.cpp:86-105`).
    ///
    /// Note what it does *not* do: bump the update revision. That is why the
    /// compound algorithms' child caches are never invalidated by a state
    /// change on this path -- see the module docs.
    pub fn update_child_transform(
        &mut self,
        child_index: usize,
        new_child_transform: Transform,
        should_recalculate_local_aabb: bool,
    ) {
        self.children[child_index].transform = new_child_transform;

        if let (Some(node), true) = (
            self.children[child_index].node,
            self.dynamic_aabb_tree.is_some(),
        ) {
            let (local_aabb_min, local_aabb_max) = self.children[child_index]
                .shape
                .get_aabb(&new_child_transform);
            let bounds = DbvtVolume::from_mm(local_aabb_min, local_aabb_max);
            self.dynamic_aabb_tree
                .as_mut()
                .expect("checked just above")
                .update(node, bounds);
        }

        if should_recalculate_local_aabb {
            self.recalculate_local_aabb();
        }
    }

    /// `recalculateLocalAabb` (`btCompoundShape.cpp:137-158`).
    pub fn recalculate_local_aabb(&mut self) {
        self.local_aabb_min = Vec3::new(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
        self.local_aabb_max = Vec3::new(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);

        for j in 0..self.children.len() {
            let (local_aabb_min, local_aabb_max) =
                self.children[j].shape.get_aabb(&self.children[j].transform);
            for i in 0..3 {
                if self.local_aabb_min[i] > local_aabb_min[i] {
                    self.local_aabb_min[i] = local_aabb_min[i];
                }
                if self.local_aabb_max[i] < local_aabb_max[i] {
                    self.local_aabb_max[i] = local_aabb_max[i];
                }
            }
        }
    }

    /// `getAabb` (`btCompoundShape.cpp:161-181`).
    ///
    /// The margin is added to the *half extents*, so it widens the box by one
    /// margin on each side before the basis rotates it -- not the same as
    /// growing the world box afterwards, once the basis is not axis-aligned.
    #[must_use]
    pub fn get_aabb(&self, trans: &Transform) -> (Vec3, Vec3) {
        let mut local_half_extents = 0.5 * (self.local_aabb_max - self.local_aabb_min);
        let mut local_center = 0.5 * (self.local_aabb_max + self.local_aabb_min);

        if self.children.is_empty() {
            local_half_extents = Vec3::zero();
            local_center = Vec3::zero();
        }
        local_half_extents += Vec3::new(self.margin(), self.margin(), self.margin());

        let abs_b = trans.basis.absolute();
        let center = trans.transform_point(local_center);
        let extent = local_half_extents.dot3(abs_b[0], abs_b[1], abs_b[2]);

        (center - extent, center + extent)
    }

    /// `getMargin`.
    #[must_use]
    pub fn margin(&self) -> Scalar {
        self.collision_margin
    }

    /// `setMargin`.
    pub fn set_margin(&mut self, margin: Scalar) {
        self.collision_margin = margin;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::probe_fixture::{at, probe_shapes, rot60_at, row};

    /// The `comp_*` rows of `tools/bullet-epa-reference/build.sh`'s stdout.
    ///
    /// Every row builds the same three children in the same order, so what
    /// separates them is the query transform, the margin, and the one
    /// `updateChildTransform` some of them perform:
    ///
    /// - `comp_aabb_rot60` against `comp_aabb_margin_rot60` is where the
    ///   margin can be seen entering the *half extents* before the basis
    ///   rotates them; under the identity the two placements agree.
    /// - `comp_update_no_recalc` against `comp_update_recalc` is the stale
    ///   local AABB MoveIt deliberately keeps -- both of
    ///   `bullet_cast_bvh_manager.cpp`'s calls pass `false`, and the tree has
    ///   already reshaped underneath it (`1 2 0` becomes `2 1 0`).
    /// - `comp_no_tree` is the only row whose compound has no tree, and it is
    ///   the one that visits nothing.
    ///
    /// Fields: `name|min.xyz|max.xyz|visited|data...`.
    const BULLET_REFERENCE: &str = "\
comp_aabb_id|-0.5|-0.5|-0.5|2.5|3.29999995|0.5|3|1|2|0
comp_aabb_rot60|-0.766666651|1.16666651|1.5|3.16666675|6.0333333|5.69999981|3|1|2|0
comp_aabb_margin_rot60|-1.1833334|0.749999762|1.08333302|3.58333349|6.44999981|6.11666679|3|1|2|0
comp_update_no_recalc|-0.5|-0.5|-0.5|2.5|3.29999995|0.5|3|2|1|0
comp_update_recalc|-0.5|-0.5|-0.5|5.5|3.29999995|0.5|3|2|1|0
comp_aabb_empty|1|2|3|1|2|3|0
comp_no_tree|-0.5|-0.5|-0.5|2.5|3.29999995|0.5|0
comp_line4|-0.5|-0.5|-0.5|6.5|0.5|0.5|4|3|2|1|0
";

    /// `probe.cpp`'s `three`: `unit_box` at the origin, `sphere` at `(2,0,0)`,
    /// `cyl` at `(0,3,0)`, added in that order.
    fn three(enable_tree: bool) -> CompoundShape {
        let (unit_box, _, _, sphere, _, cyl, _, _) = probe_shapes();
        let mut c = CompoundShape::new(enable_tree);
        c.add_child_shape(at(0.0, 0.0, 0.0), Shape::Convex(Arc::new(unit_box)));
        c.add_child_shape(at(2.0, 0.0, 0.0), Shape::Convex(Arc::new(sphere)));
        c.add_child_shape(at(0.0, 3.0, 0.0), Shape::Convex(Arc::new(cyl)));
        c
    }

    /// One row: the compound's AABB under `t`, and its tree's leaf order.
    fn check(bad: &mut Vec<String>, name: &str, c: &CompoundShape, t: &Transform) {
        let (mn, mx) = c.get_aabb(t);

        let mut seen = Vec::new();
        if let Some(tree) = c.dynamic_aabb_tree() {
            let all = DbvtVolume::from_mm(Vec3::new(-1e6, -1e6, -1e6), Vec3::new(1e6, 1e6, 1e6));
            let mut stack = Vec::new();
            tree.collide_tv_no_stack_alloc(tree.root, &all, &mut stack, &mut |t, n| {
                seen.push(t.node(n).data);
            });
        }

        let f = row(BULLET_REFERENCE, name, 8 + seen.len());
        let want_min = Vec3::new(
            f[1].parse().unwrap(),
            f[2].parse().unwrap(),
            f[3].parse().unwrap(),
        );
        let want_max = Vec3::new(
            f[4].parse().unwrap(),
            f[5].parse().unwrap(),
            f[6].parse().unwrap(),
        );
        let want_visited: usize = f[7].parse().unwrap();
        let want: Vec<i32> = f[8..].iter().map(|v| v.parse().unwrap()).collect();

        crate::probe_fixture::diff_vec3(bad, name, "min", mn, want_min);
        crate::probe_fixture::diff_vec3(bad, name, "max", mx, want_max);
        if seen.len() != want_visited || seen != want {
            bad.push(format!("{name}.order: port {seen:?}, bullet {want:?}"));
        }
    }

    #[test]
    fn bullet_reference_compound() {
        let mut bad = Vec::new();

        let mut c = three(true);
        check(&mut bad, "comp_aabb_id", &c, &Transform::identity());
        check(&mut bad, "comp_aabb_rot60", &c, &rot60_at(1.0, 2.0, 3.0));

        c.set_margin(0.25);
        check(
            &mut bad,
            "comp_aabb_margin_rot60",
            &c,
            &rot60_at(1.0, 2.0, 3.0),
        );
        c.set_margin(0.0);

        c.update_child_transform(1, at(5.0, 0.0, 0.0), false);
        check(
            &mut bad,
            "comp_update_no_recalc",
            &c,
            &Transform::identity(),
        );
        c.recalculate_local_aabb();
        check(&mut bad, "comp_update_recalc", &c, &Transform::identity());

        let empty = CompoundShape::new(true);
        check(
            &mut bad,
            "comp_aabb_empty",
            &empty,
            &rot60_at(1.0, 2.0, 3.0),
        );

        let no_tree = three(false);
        check(&mut bad, "comp_no_tree", &no_tree, &Transform::identity());

        let (unit_box, ..) = probe_shapes();
        let mut line = CompoundShape::new(true);
        for i in 0..4 {
            line.add_child_shape(
                at(f64::from(i) as Scalar * 2.0, 0.0, 0.0),
                Shape::Convex(Arc::new(unit_box)),
            );
        }
        check(&mut bad, "comp_line4", &line, &Transform::identity());

        assert!(
            bad.is_empty(),
            "{} deviations:\n{}",
            bad.len(),
            bad.join("\n")
        );
    }

    /// A compound child reports `COMPOUND_SHAPE_PROXYTYPE`, which is what
    /// `findAlgorithm` needs to see to descend rather than hand the pair to
    /// the narrow phase.
    #[test]
    fn a_nested_compound_reports_the_compound_type() {
        let mut outer = CompoundShape::new(true);
        outer.add_child_shape(at(1.0, 0.0, 0.0), Shape::Compound(three(true)));

        assert_eq!(
            outer.child_shape(0).shape_type(),
            BroadphaseNativeType::COMPOUND_SHAPE
        );
        assert!(outer.child_shape(0).shape_type().is_compound());
    }
}
