// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h
//   bullet3/src/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp
//   bullet3/src/BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.h
//   bullet3/src/BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.cpp
//   bullet3/src/BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h

//! `btCompoundCollisionAlgorithm` and
//! `btCompoundCompoundCollisionAlgorithm` -- the two traversals that turn one
//! query over a pair of objects into a sequence of convex-convex queries over
//! their children, and [`process_collision`], the entry point that picks
//! between them and the convex-convex case the way
//! `btCollisionDispatcher::findAlgorithm` does.
//!
//! # The algorithm objects are gone; the dispatch is not
//!
//! Upstream an algorithm is an object: `findAlgorithm` allocates one per pair,
//! it remembers a manifold and an array of child algorithms, and
//! `processCollision` is a method on it. Here it is one function per algorithm
//! and the object's state is not carried, because on the continuous path none
//! of that state is observable:
//!
//! - **`m_childCollisionAlgorithms` / `m_childCollisionAlgorithmCache`** hold
//!   child algorithms so that a repeated query can reuse them.
//!   `btConvexConvexAlgorithm` -- the only child algorithm this crate carries
//!   -- keeps nothing across calls: `processCollision` builds a fresh
//!   `btGjkPairDetector`, resets the simplex solver, and writes into a
//!   manifold whose point cache MoveIt's bridge never fills. Reusing one is
//!   therefore indistinguishable from building one.
//!
//! - **`m_compoundShapeRevision`** exists to notice a compound whose children
//!   changed and rebuild those caches. `updateChildTransform` does not bump
//!   `m_updateRevision` (`btCompoundShape.cpp:86-105`), and MoveIt's
//!   continuous path performs no other edit after the build
//!   (`bullet_cast_bvh_manager.cpp:102`, `:115`), so the revision never
//!   changes and the rebuild never fires.
//!
//! - **The two trailing "remove non-overlapping child pairs" loops**
//!   (`btCompoundCollisionAlgorithm.cpp:302-341`,
//!   `btCompoundCompoundCollisionAlgorithm.cpp:349-405`) free entries out of
//!   those same caches. With no caches there is nothing to free, and neither
//!   loop touches `resultOut`.
//!
//! - **The two manifold-refresh loops** (`:256-278` and `:311-339`) call
//!   `refreshContactPoints` for every child manifold that has contacts. No
//!   manifold on this path ever has one; see [`crate::manifold`].
//!
//! What that leaves is the part that decides *which* child pairs reach the
//! narrow phase and *how they are labelled*, which is all observable: the
//! dispatch order, the `setShapeIdentifiers` pair each child dispatch is
//! tagged with, and the composed child world transform every contact is
//! computed against.
//!
//! # The wrapper, and the swap that is not one
//!
//! [`CollisionObjectWrapper`] is `btCollisionObjectWrapper` reduced to the
//! three fields the traversal reads: the shape, the world transform, and the
//! identity of the collision object behind it. `m_parent` is not carried
//! because nothing in `BulletCollision` reads it, and `m_preTransform` is not
//! carried because its only readers are `ProcessChildShape` composing it for
//! the next level down and `btSoftBody`'s two cluster-collision helpers.
//!
//! Both leaf callbacks bracket the child dispatch with
//! `setBody0Wrap`/`setBody1Wrap`. That has no effect here, and the reason is
//! not that the port drops it: the child wrapper is built with
//! `m_compoundColObjWrap->getCollisionObject()`, i.e. it *shares* the
//! collision object with the wrapper it replaces. `btManifoldResult`'s only
//! reads through those pointers are `getBody0Internal()` -- the object, hence
//! unchanged -- and, in `addContactPoint`, `getCollisionObject()
//! ->getWorldTransform()`, which is the object's transform and not the
//! wrapper's. MoveIt's bridge reads the same two things
//! (`bullet_utils.hpp:571-630`). The child's own world transform does matter,
//! and it reaches the narrow phase as the wrapper this module passes down,
//! not through the result.
//!
//! # Which table a child pair is looked up in
//!
//! `ProcessChildShape` and `btCompoundCompoundLeafCallback::Process` both
//! choose between `BT_CLOSEST_POINT_ALGORITHMS` and
//! `BT_CONTACT_POINT_ALGORITHMS` on `m_closestPointDistanceThreshold > 0`,
//! independently of the table the *enclosing* query was looked up in. On
//! MoveIt's continuous path the threshold is always zero
//! (`bullet_bvh_manager.cpp:55`), so a top-level query that
//! `TesseractCollisionPairCallback::processOverlap` looked up in the
//! closest-points table dispatches its children through the contact-points
//! one. The two tables differ in exactly one cell -- box against box -- so
//! that fork is only observable there, and it is observable: a box child
//! against a box resolves to `btBoxBoxCollisionAlgorithm`, which this crate
//! does not carry and which [`process_collision`] reports as
//! [`UnportedAlgorithm`] rather than silently routing into GJK.

use crate::broadphase_proxy::BroadphaseNativeType;
use crate::collision_object_wrapper::CollisionObjectWrapper;
use crate::compound::{CompoundShape, Shape};
use crate::convex_convex;
use crate::dbvt::{Dbvt, DbvtVolume, intersect};
use crate::dispatch::{Algorithm, DispatchTable, find_algorithm};
use crate::linear_math::{Scalar, Transform, Vec3, test_aabb_against_aabb2, transform_aabb};
use crate::manifold::{ManifoldResult, PersistentManifold};

/// A pair whose create-func names an algorithm this crate does not carry.
///
/// Upstream every cell of both tables returns something callable, so there is
/// no error to return and no caller that could check one. Here the unported
/// cells are the majority -- see [`crate::dispatch`] for which and why -- and
/// a query that reaches one has produced no contacts and must not be read as
/// having found none.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UnportedAlgorithm {
    /// What `findAlgorithm` answered for the pair.
    pub algorithm: Algorithm,
    /// `body0Wrap`'s shape type.
    pub proxy_type0: BroadphaseNativeType,
    /// `body1Wrap`'s shape type.
    pub proxy_type1: BroadphaseNativeType,
}

/// One `processCollision` on the algorithm `findAlgorithm(table, ...)` names
/// for this pair.
///
/// `table` is the table *this* call was looked up in --
/// `DispatchTable::ClosestPoints` for a top-level query, since that is what
/// `TesseractCollisionPairCallback::processOverlap` asks for
/// (`bullet_utils.cpp:528`). What the children are looked up in is not this
/// argument; see the module docs.
///
/// # Errors
///
/// [`UnportedAlgorithm`] if this pair, or any child pair reached below it,
/// resolves to an algorithm this crate does not carry. The traversal stops at
/// the first one -- the query has already failed, and continuing would report
/// a contact set missing whatever that pair would have added.
pub fn process_collision<'a>(
    body0: &CollisionObjectWrapper<'a>,
    body1: &CollisionObjectWrapper<'a>,
    table: DispatchTable,
    result_out: &mut dyn ManifoldResult<'a>,
) -> Result<(), UnportedAlgorithm> {
    let proxy_type0 = body0.shape.shape_type();
    let proxy_type1 = body1.shape.shape_type();
    let algorithm = find_algorithm(table, proxy_type0, proxy_type1);

    match (body0.shape, body1.shape) {
        (Shape::Convex(min0), Shape::Convex(min1)) => {
            // The only arm where the answer can be an algorithm other than
            // the one the shapes suggest: two convex proxy types resolve to
            // sphere-sphere, sphere-triangle, box-box or convex-plane before
            // they reach the convex-convex row. The compound arms below
            // cannot disagree -- `COMPOUND_SHAPE_PROXYTYPE` is neither convex
            // nor concave, so it falls past every earlier row.
            if algorithm != Algorithm::ConvexConvex {
                return Err(UnportedAlgorithm {
                    algorithm,
                    proxy_type0,
                    proxy_type1,
                });
            }
            // `getNewManifold(body0Wrap->getCollisionObject(),
            // body1Wrap->getCollisionObject())` (`btConvexConvexAlgorithm.cpp:278`)
            // -- the two objects *this* dispatch was given, which for a swapped
            // compound arm are not the two the enclosing query named.
            let manifold = PersistentManifold::new(body0.object_id, body1.object_id);
            convex_convex::process_collision(
                min0.as_ref(),
                &body0.world_transform,
                min1.as_ref(),
                &body1.world_transform,
                manifold,
                result_out,
            );
            Ok(())
        }
        (Shape::Compound(compound0), Shape::Compound(compound1)) => {
            compound_compound_process_collision(body0, compound0, body1, compound1, result_out)
        }
        (Shape::Compound(compound), Shape::Convex(_)) => {
            compound_process_collision(body0, compound, body1, result_out)
        }
        (Shape::Convex(_), Shape::Compound(compound)) => {
            // `SwappedCreateFunc` -- `m_isSwapped` is true, so the compound is
            // `body1Wrap` and the convex is the other object.
            compound_process_collision(body1, compound, body0, result_out)
        }
    }
}

/// Which table a child pair is looked up in
/// (`btCompoundCollisionAlgorithm.cpp:150-166`,
/// `btCompoundCompoundCollisionAlgorithm.cpp:167-186`).
fn child_table(result_out: &mut dyn ManifoldResult<'_>) -> DispatchTable {
    if result_out.state().closest_point_distance_threshold > 0.0 {
        DispatchTable::ClosestPoints
    } else {
        DispatchTable::ContactPoints
    }
}

/// `btCompoundCollisionAlgorithm::processCollision` (`:230-341`).
///
/// `col_obj_wrap` is upstream's `colObjWrap` -- the compound side, already
/// resolved through `m_isSwapped` by the caller -- and `compound_shape` is its
/// shape. Passing the compound in rather than re-matching it here is what
/// keeps `m_isSwapped` from being a second, independent claim about which side
/// is which.
fn compound_process_collision<'a>(
    col_obj_wrap: &CollisionObjectWrapper<'a>,
    compound_shape: &'a CompoundShape,
    other_obj_wrap: &CollisionObjectWrapper<'a>,
    result_out: &mut dyn ManifoldResult<'a>,
) -> Result<(), UnportedAlgorithm> {
    // `if (m_childCollisionAlgorithms.size() == 0) return;` (`:249-250`).
    // That array is sized to the child count by `preallocateChildAlgorithms`
    // (`:44-72`) and never resized elsewhere, so the test is on the compound.
    if compound_shape.num_child_shapes() == 0 {
        return Ok(());
    }

    let Some(tree) = compound_shape.dynamic_aabb_tree() else {
        // `else` (`:295-304`): every child, in index order, culled only by
        // `TestAabbAgainstAabb2` inside `ProcessChildShape`.
        for index in 0..compound_shape.num_child_shapes() {
            process_child_shape(
                col_obj_wrap,
                compound_shape,
                other_obj_wrap,
                index,
                result_out,
            )?;
        }
        return Ok(());
    };

    let other_in_compound_space =
        col_obj_wrap.world_transform.inverse() * other_obj_wrap.world_transform;
    let (mut local_aabb_min, mut local_aabb_max) =
        other_obj_wrap.shape.get_aabb(&other_in_compound_space);
    let threshold = result_out.state().closest_point_distance_threshold;
    let extra_extends = Vec3::new(threshold, threshold, threshold);
    local_aabb_min -= extra_extends;
    local_aabb_max += extra_extends;
    let bounds = DbvtVolume::from_mm(local_aabb_min, local_aabb_max);

    // `collideTVNoStackAlloc` has no way to stop early and no way to fail, so
    // the first unported pair is held here and the remaining leaves are
    // skipped rather than dispatched into a result that is already wrong.
    let mut failure = None;
    let mut stack = Vec::new();
    tree.collide_tv_no_stack_alloc(tree.root, &bounds, &mut stack, &mut |visited, leaf| {
        if failure.is_some() {
            return;
        }
        let index = usize::try_from(visited.node(leaf).data).expect("child indices are >= 0");
        if let Err(unported) = process_child_shape(
            col_obj_wrap,
            compound_shape,
            other_obj_wrap,
            index,
            result_out,
        ) {
            failure = Some(unported);
        }
    });

    match failure {
        Some(unported) => Err(unported),
        None => Ok(()),
    }
}

/// `btCompoundLeafCallback::ProcessChildShape` (`:110-208`).
///
/// `gCompoundChildShapePairCallback` is not ported: it is a global filter hook
/// that is null unless an application installs one, and MoveIt does not.
fn process_child_shape<'a>(
    compound_col_obj_wrap: &CollisionObjectWrapper<'a>,
    compound_shape: &'a CompoundShape,
    other_obj_wrap: &CollisionObjectWrapper<'a>,
    index: usize,
    result_out: &mut dyn ManifoldResult<'a>,
) -> Result<(), UnportedAlgorithm> {
    let child_shape = compound_shape.child_shape(index);
    let org_trans = compound_col_obj_wrap.world_transform;
    let child_trans = *compound_shape.child_transform(index);
    let new_child_world_trans = org_trans * child_trans;

    let (mut aabb_min0, mut aabb_max0) = child_shape.get_aabb(&new_child_world_trans);
    let threshold = result_out.state().closest_point_distance_threshold;
    let extend_aabb = Vec3::new(threshold, threshold, threshold);
    aabb_min0 -= extend_aabb;
    aabb_max0 += extend_aabb;

    let (aabb_min1, aabb_max1) = other_obj_wrap
        .shape
        .get_aabb(&other_obj_wrap.world_transform);

    if !test_aabb_against_aabb2(aabb_min0, aabb_max0, aabb_min1, aabb_max1) {
        return Ok(());
    }

    let compound_wrap = compound_col_obj_wrap.child(child_shape, new_child_world_trans);
    let child_index = i32::try_from(index).expect("fewer than i32::MAX children");

    // `if (m_resultOut->getBody0Internal() == m_compoundColObjWrap
    // ->getCollisionObject())` (`:170`) -- whether the compound is the
    // result's body 0, which is the same question as whether this algorithm
    // was created swapped, asked of the object rather than of a flag. The same
    // test decides which side the child wrapper replaces, and which side is
    // put back afterwards (`:194-201`).
    let compound_is_body0 =
        result_out.state().body0_wrap.object_id == compound_col_obj_wrap.object_id;
    let displaced = if compound_is_body0 {
        result_out.set_shape_identifiers_a(-1, child_index);
        result_out.set_body0_wrap(compound_wrap)
    } else {
        result_out.set_shape_identifiers_b(-1, child_index);
        result_out.set_body1_wrap(compound_wrap)
    };

    // `algo->processCollision(&compoundWrap, m_otherObjWrap, ...)` (`:183`):
    // the child is body 0 and the other object body 1 whichever way round the
    // enclosing query was, so a swapped compound query reports its contacts
    // against the opposite operand from the one the caller passed first.
    let table = child_table(result_out);
    let outcome = process_collision(&compound_wrap, other_obj_wrap, table, result_out);

    // Upstream asks the question a second time (`:194`) rather than reusing
    // the first answer, and gets the same one: the wrapper it just installed
    // names the same collision object as the one it displaced, so the test
    // cannot have flipped. Hoisting it keeps the two branches from being two
    // independent claims about which side the compound is.
    if compound_is_body0 {
        result_out.set_body0_wrap(displaced);
    } else {
        result_out.set_body1_wrap(displaced);
    }

    outcome
}

/// `btCompoundCompoundCollisionAlgorithm::processCollision` (`:285-407`).
fn compound_compound_process_collision<'a>(
    col0_obj_wrap: &CollisionObjectWrapper<'a>,
    compound_shape0: &'a CompoundShape,
    col1_obj_wrap: &CollisionObjectWrapper<'a>,
    compound_shape1: &'a CompoundShape,
    result_out: &mut dyn ManifoldResult<'a>,
) -> Result<(), UnportedAlgorithm> {
    let (Some(tree0), Some(tree1)) = (
        compound_shape0.dynamic_aabb_tree(),
        compound_shape1.dynamic_aabb_tree(),
    ) else {
        // `if (!tree0 || !tree1) return btCompoundCollisionAlgorithm
        // ::processCollision(body0Wrap, body1Wrap, ...)` (`:297-300`). The
        // base's `m_isSwapped` is false for both create-funcs of this
        // algorithm (`btCompoundCompoundCollisionAlgorithm.h:63-79`), so the
        // *first* operand is treated as the compound and the second as the
        // other object -- which one still has a tree does not enter into it.
        return compound_process_collision(
            col0_obj_wrap,
            compound_shape0,
            col1_obj_wrap,
            result_out,
        );
    };

    let xform = col0_obj_wrap.world_transform.inverse() * col1_obj_wrap.world_transform;
    let distance_threshold = result_out.state().closest_point_distance_threshold;

    let mut callback = CompoundCompoundLeafCallback {
        compound0_col_obj_wrap: *col0_obj_wrap,
        compound_shape0,
        compound1_col_obj_wrap: *col1_obj_wrap,
        compound_shape1,
        result_out,
    };

    my_collide_tt(
        tree0,
        tree1,
        &xform,
        distance_threshold,
        &mut |leaf0, leaf1| {
            let child_index0 =
                usize::try_from(tree0.node(leaf0).data).expect("child indices are >= 0");
            let child_index1 =
                usize::try_from(tree1.node(leaf1).data).expect("child indices are >= 0");
            callback.process(child_index0, child_index1)
        },
    )
}

/// `btCompoundCompoundLeafCallback`
/// (`btCompoundCompoundCollisionAlgorithm.cpp:90-212`), reduced to the fields
/// [`CompoundCompoundLeafCallback::process`] reads.
///
/// A struct rather than a closure because that is what it is upstream, and
/// because the two sides are otherwise six loose arguments that nothing
/// prevents being passed in the wrong order.
struct CompoundCompoundLeafCallback<'a, 'r> {
    compound0_col_obj_wrap: CollisionObjectWrapper<'a>,
    compound_shape0: &'a CompoundShape,
    compound1_col_obj_wrap: CollisionObjectWrapper<'a>,
    compound_shape1: &'a CompoundShape,
    result_out: &'r mut dyn ManifoldResult<'a>,
}

impl CompoundCompoundLeafCallback<'_, '_> {
    /// `Process(leaf0, leaf1)` (`:114-212`), taking the child indices the
    /// caller has already read out of the two leaves.
    ///
    /// The threshold grows box 0 only. That asymmetry is upstream's and is
    /// kept: `ProcessChildShape` does the same, and the pair-removal loop that
    /// grows *both* (`:377-391`) is bookkeeping over a cache this port does
    /// not carry.
    fn process(
        &mut self,
        child_index0: usize,
        child_index1: usize,
    ) -> Result<(), UnportedAlgorithm> {
        let child_shape0 = self.compound_shape0.child_shape(child_index0);
        let child_shape1 = self.compound_shape1.child_shape(child_index1);

        let new_child_world_trans0 = self.compound0_col_obj_wrap.world_transform
            * *self.compound_shape0.child_transform(child_index0);
        let new_child_world_trans1 = self.compound1_col_obj_wrap.world_transform
            * *self.compound_shape1.child_transform(child_index1);

        let (mut aabb_min0, mut aabb_max0) = child_shape0.get_aabb(&new_child_world_trans0);
        let (aabb_min1, aabb_max1) = child_shape1.get_aabb(&new_child_world_trans1);

        let threshold = self.result_out.state().closest_point_distance_threshold;
        let threshold_vec = Vec3::new(threshold, threshold, threshold);
        aabb_min0 -= threshold_vec;
        aabb_max0 += threshold_vec;

        if !test_aabb_against_aabb2(aabb_min0, aabb_max0, aabb_min1, aabb_max1) {
            return Ok(());
        }

        let compound_wrap0 = self
            .compound0_col_obj_wrap
            .child(child_shape0, new_child_world_trans0);
        let compound_wrap1 = self
            .compound1_col_obj_wrap
            .child(child_shape1, new_child_world_trans1);

        // Both wrappers and both identifiers, unconditionally (`:195-199`):
        // with a compound on each side there is no swap to detect, and each
        // child names one of them.
        let displaced0 = self.result_out.set_body0_wrap(compound_wrap0);
        let displaced1 = self.result_out.set_body1_wrap(compound_wrap1);
        self.result_out.set_shape_identifiers_a(
            -1,
            i32::try_from(child_index0).expect("fewer than i32::MAX children"),
        );
        self.result_out.set_shape_identifiers_b(
            -1,
            i32::try_from(child_index1).expect("fewer than i32::MAX children"),
        );

        let table = child_table(self.result_out);
        let outcome = process_collision(&compound_wrap0, &compound_wrap1, table, self.result_out);

        self.result_out.set_body0_wrap(displaced0);
        self.result_out.set_body1_wrap(displaced1);

        outcome
    }
}

/// `MycollideTT` (`btCompoundCompoundCollisionAlgorithm.cpp:226-283`).
///
/// A second tree traversal, not `btDbvt::collideTT`: it walks *two* trees and
/// tests each pair through [`my_intersect`], which brings tree 1's volume into
/// tree 0's frame. The push order below is upstream's, and it is what decides
/// the order child pairs reach the narrow phase -- the stack is LIFO, so the
/// last pair pushed is the first visited.
fn my_collide_tt(
    tree0: &Dbvt,
    tree1: &Dbvt,
    xform: &Transform,
    distance_threshold: Scalar,
    process: &mut impl FnMut(usize, usize) -> Result<(), UnportedAlgorithm>,
) -> Result<(), UnportedAlgorithm> {
    let (Some(root0), Some(root1)) = (tree0.root, tree1.root) else {
        return Ok(());
    };

    let mut stack = vec![(root0, root1)];
    while let Some((a, b)) = stack.pop() {
        let node_a = tree0.node(a);
        let node_b = tree1.node(b);
        if !my_intersect(&node_a.volume, &node_b.volume, xform, distance_threshold) {
            continue;
        }
        let (a0, a1) = (node_a.child[0], node_a.child[1]);
        let (b0, b1) = (node_b.child[0], node_b.child[1]);
        match (a0.zip(a1), b0.zip(b1)) {
            (Some((a0, a1)), Some((b0, b1))) => {
                stack.push((a0, b0));
                stack.push((a1, b0));
                stack.push((a0, b1));
                stack.push((a1, b1));
            }
            (Some((a0, a1)), None) => {
                stack.push((a0, b));
                stack.push((a1, b));
            }
            (None, Some((b0, b1))) => {
                stack.push((a, b0));
                stack.push((a, b1));
            }
            (None, None) => process(a, b)?,
        }
    }
    Ok(())
}

/// `MyIntersect` (`btCompoundCompoundCollisionAlgorithm.cpp:215-224`).
///
/// `b` is a leaf volume of the *second* tree, in that compound's own frame;
/// `xform` carries it into the first compound's. The re-boxing that costs is
/// why this test is looser than the world-space one
/// [`CompoundCompoundLeafCallback::process`] then applies -- under any rotation a box
/// transformed and re-fitted is larger than the shape's own transformed box,
/// so pairs reach the leaf callback that its `TestAabbAgainstAabb2` rejects.
fn my_intersect(
    a: &DbvtVolume,
    b: &DbvtVolume,
    xform: &Transform,
    distance_threshold: Scalar,
) -> bool {
    let (mut newmin, mut newmax) = transform_aabb(b.mi, b.mx, 0.0, xform);
    let d = Vec3::new(distance_threshold, distance_threshold, distance_threshold);
    newmin -= d;
    newmax += d;
    intersect(a, &DbvtVolume::from_mm(newmin, newmax))
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;
    use crate::discrete_detector::Result as DetectorResult;
    use crate::linear_math::Matrix3;
    use crate::manifold::ManifoldResultState;
    use crate::probe_fixture::{IDENTITY, at, diff, diff_vec3, probe_shapes, rot60_at, row};
    use crate::shapes::ConvexShape;

    /// `probe.cpp`'s `TraceResult`: MoveIt's bridge, plus a record of the two
    /// things the traversals do to a result -- the `setShapeIdentifiers` call
    /// that precedes every child dispatch, and the identifiers in force when a
    /// contact is reported -- plus every contact's geometry.
    ///
    /// Every contact's, not just the last: a child whose composed world
    /// transform is wrong changes the contacts *that child* reported, and with
    /// one slot the last child to write vouches for all of them.
    struct TraceResult<'a> {
        state: ManifoldResultState<'a>,
        dispatches: Vec<String>,
        contacts: Vec<String>,
        geometry: Vec<(Vec3, Vec3, Scalar)>,
        /// The two body wraps' own transforms at each contact, which is what
        /// `addCastSingleResult` reads back out of them.
        wraps: Vec<(Transform, Transform)>,
        /// `isSwapped` at each contact -- which operand the manifold was
        /// created around, against which one the result calls body 0.
        swapped: Vec<bool>,
    }

    impl<'a> TraceResult<'a> {
        fn new(
            wrap0: CollisionObjectWrapper<'a>,
            wrap1: CollisionObjectWrapper<'a>,
            closest_point_distance_threshold: Scalar,
        ) -> Self {
            let mut state = ManifoldResultState::new(wrap0, wrap1);
            state.closest_point_distance_threshold = closest_point_distance_threshold;
            Self {
                state,
                dispatches: Vec::new(),
                contacts: Vec::new(),
                geometry: Vec::new(),
                wraps: Vec::new(),
                swapped: Vec::new(),
            }
        }
    }

    impl DetectorResult for TraceResult<'_> {
        fn add_contact_point(
            &mut self,
            normal_on_b_in_world: Vec3,
            point_in_world: Vec3,
            depth: Scalar,
        ) {
            self.contacts
                .push(format!("{}:{}", self.state.index0, self.state.index1));
            self.geometry
                .push((normal_on_b_in_world, point_in_world, depth));
            self.wraps.push((
                self.state.body0_wrap.world_transform,
                self.state.body1_wrap.world_transform,
            ));
            self.swapped.push(self.state.is_swapped());
        }
    }

    impl<'a> ManifoldResult<'a> for TraceResult<'a> {
        fn state(&mut self) -> &mut ManifoldResultState<'a> {
            &mut self.state
        }

        fn set_shape_identifiers_a(&mut self, part_id0: i32, index0: i32) {
            self.state.part_id0 = part_id0;
            self.state.index0 = index0;
            self.dispatches.push(format!("A{part_id0}:{index0}"));
        }

        fn set_shape_identifiers_b(&mut self, part_id1: i32, index1: i32) {
            self.state.part_id1 = part_id1;
            self.state.index1 = index1;
            self.dispatches.push(format!("B{part_id1}:{index1}"));
        }
    }

    fn convex(shape: impl ConvexShape + 'static) -> Shape {
        Shape::Convex(Arc::new(shape))
    }

    fn unit_box() -> Shape {
        convex(probe_shapes().0)
    }
    fn margin_box() -> Shape {
        convex(probe_shapes().2)
    }
    fn sphere() -> Shape {
        convex(probe_shapes().3)
    }
    fn cyl() -> Shape {
        convex(probe_shapes().5)
    }

    /// Three of one shape a unit apart along x, so their AABBs touch exactly
    /// and a query over the middle one clears both culls on all three.
    fn line3(enable_dynamic_aabb_tree: bool, child: fn() -> Shape) -> Shape {
        let mut compound = CompoundShape::new(enable_dynamic_aabb_tree);
        for i in 0..3 {
            compound.add_child_shape(at(i as Scalar, 0.0, 0.0), child());
        }
        Shape::Compound(compound)
    }

    /// Two `margin_box` children, no tree -- so `TestAabbAgainstAabb2` is the
    /// only cull between the query and the narrow phase.
    fn no_tree_pair() -> Shape {
        let mut compound = CompoundShape::new(false);
        compound.add_child_shape(IDENTITY, margin_box());
        compound.add_child_shape(at(3.0, 0.0, 0.0), margin_box());
        Shape::Compound(compound)
    }

    fn no_tree_cyl2() -> Shape {
        let mut compound = CompoundShape::new(false);
        compound.add_child_shape(IDENTITY, cyl());
        compound.add_child_shape(at(2.0, 0.0, 0.0), cyl());
        Shape::Compound(compound)
    }

    fn empty() -> Shape {
        Shape::Compound(CompoundShape::new(true))
    }

    /// A compound whose second child is itself a compound.
    fn nested() -> Shape {
        let mut inner = CompoundShape::new(true);
        inner.add_child_shape(IDENTITY, unit_box());
        inner.add_child_shape(at(1.0, 0.0, 0.0), cyl());

        let mut outer = CompoundShape::new(true);
        outer.add_child_shape(IDENTITY, unit_box());
        outer.add_child_shape(at(1.5, 0.0, 0.0), Shape::Compound(inner));
        Shape::Compound(outer)
    }

    fn pair2() -> Shape {
        let mut compound = CompoundShape::new(true);
        compound.add_child_shape(IDENTITY, cyl());
        compound.add_child_shape(at(1.0, 0.0, 0.0), sphere());
        Shape::Compound(compound)
    }

    /// The `cc{dispatch,contact,point}_*` rows of
    /// `tools/bullet-epa-reference/build.sh`'s stdout, verbatim: the real
    /// `btCompoundCollisionAlgorithm` and
    /// `btCompoundCompoundCollisionAlgorithm` from bullet3 @ `7dee3436`,
    /// reached through the dispatcher `BulletBVHManager` configures and asked
    /// for `BT_CLOSEST_POINT_ALGORITHMS`.
    ///
    /// `ccdispatch` is the sequence of `setShapeIdentifiersA`/`B` calls, which
    /// is the child dispatch order with the index each child was tagged with;
    /// `cccontact` is the identifier pair in force at each contact; each
    /// `ccpoint_<case>_<k>` is contact `k`'s geometry, which is what pins the
    /// composed world transform of the child that reported it.
    ///
    /// Every child pair has a sphere, a cylinder or a compound on one side --
    /// what a `CastHullShape` guarantees on the real continuous path. The
    /// `line3_box` rows are the exception and are read by their own test.
    const BULLET_REFERENCE: &str = "\
ccdispatch_line3_sphere|3|A-1:2|A-1:1|A-1:0
cccontact_line3_sphere|3|2:-1|1:-1|0:-1
ccpoint_line3_sphere_0|1|0|0|1.5|0|0|0
ccpoint_line3_sphere_1|-1|2.09549555e-09|1.88939193e-05|0.500009418|0.00151802995|0.00155581255|-0.999990582
ccpoint_line3_sphere_2|-1|0|0|0.5|0|0|0
ccdispatch_sphere_line3|3|B-1:2|B-1:1|B-1:0
cccontact_sphere_line3|3|-1:2|-1:1|-1:0
ccpoint_sphere_line3_0|1|0|0|1.5|0|0|0
ccpoint_sphere_line3_1|-1|2.09549555e-09|1.88939193e-05|0.500009418|0.00151802995|0.00155581255|-0.999990582
ccpoint_sphere_line3_2|-1|0|0|0.5|0|0|0
ccdispatch_line3_sphere_off|3|A-1:2|A-1:1|A-1:0
cccontact_line3_sphere_off|2|1:-1|0:-1
ccpoint_line3_sphere_off_0|-1.11758723e-07|-1|-7.45058131e-08|0.99999994|0.399999976|0.299999952|-0.100000024
ccpoint_line3_sphere_off_1|-0.780868888|-0.624695063|2.32717063e-08|0.609565556|0.587652445|0.300000012|0.140312374
ccdispatch_line3_rot60_sphere|2|A-1:1|A-1:0
cccontact_line3_rot60_sphere|2|1:-1|0:-1
ccpoint_line3_rot60_sphere_0|-0.298142165|0.596285105|-0.745355904|0.850928903|0.298142552|-0.372677952|-0.425464392
ccpoint_line3_rot60_sphere_1|-0.666666329|0.33333385|-0.666666687|0.666666865|0.166666925|-0.333333343|-0.433333337
ccdispatch_line3_sphere_far|0
cccontact_line3_sphere_far|0
ccdispatch_no_tree3_sphere|3|A-1:0|A-1:1|A-1:2
cccontact_no_tree3_sphere|3|0:-1|1:-1|2:-1
ccpoint_no_tree3_sphere_0|-1|0|0|0.5|0|0|0
ccpoint_no_tree3_sphere_1|-1|2.09549555e-09|1.88939193e-05|0.500009418|0.00151802995|0.00155581255|-0.999990582
ccpoint_no_tree3_sphere_2|1|0|0|1.5|0|0|0
ccdispatch_no_tree_window|0
cccontact_no_tree_window|0
ccdispatch_empty_sphere|0
cccontact_empty_sphere|0
ccdispatch_line3_sphere_threshold|3|A-1:2|A-1:1|A-1:0
cccontact_line3_sphere_threshold|3|2:-1|1:-1|0:-1
ccpoint_line3_sphere_threshold_0|1|0|0|1.5|0|0|0
ccpoint_line3_sphere_threshold_1|-1|2.09549555e-09|1.88939193e-05|0.500009418|0.00151802995|0.00155581255|-0.999990582
ccpoint_line3_sphere_threshold_2|-1|0|0|0.5|0|0|0
ccdispatch_no_tree_window_threshold|1|A-1:0
cccontact_no_tree_window_threshold|1|0:-1
ccpoint_no_tree_window_threshold_0|-1|0|0|0.50999999|0|0|0.00999993086
ccdispatch_line3_sphere_far_threshold|1|A-1:2
cccontact_line3_sphere_far_threshold|1|2:-1
ccpoint_line3_sphere_far_threshold_0|-1|0|0|3.4000001|0|0|0.900000095
ccdispatch_line3_box|3|A-1:2|A-1:1|A-1:0
cccontact_line3_box|12|2:-1|2:-1|2:-1|2:-1|1:-1|1:-1|1:-1|1:-1|0:-1|0:-1|0:-1|0:-1
ccpoint_line3_box_0|1|0|0|1.5|0.5|0.5|-0
ccpoint_line3_box_1|1|0|0|1.5|0.5|-0.5|-0
ccpoint_line3_box_2|1|0|0|1.5|-0.5|-0.5|-0
ccpoint_line3_box_3|1|0|0|1.5|-0.5|0.5|-0
ccpoint_line3_box_4|-1|-0|-0|0.5|0.5|0.5|-1
ccpoint_line3_box_5|-1|-0|-0|0.5|0.5|-0.5|-1
ccpoint_line3_box_6|-1|-0|-0|0.5|-0.5|-0.5|-1
ccpoint_line3_box_7|-1|-0|-0|0.5|-0.5|0.5|-1
ccpoint_line3_box_8|-1|-0|-0|0.5|0.5|0.5|-0
ccpoint_line3_box_9|-1|-0|-0|0.5|0.5|-0.5|-0
ccpoint_line3_box_10|-1|-0|-0|0.5|-0.5|-0.5|-0
ccpoint_line3_box_11|-1|-0|-0|0.5|-0.5|0.5|-0
ccdispatch_line3_box_threshold|3|A-1:2|A-1:1|A-1:0
cccontact_line3_box_threshold|3|2:-1|1:-1|0:-1
ccpoint_line3_box_threshold_0|1|0|0|1.5|0|0|-7.4505806e-09
ccpoint_line3_box_threshold_1|8.07007098e-07|1.95515986e-06|-1|0.99997288|3.13998462e-05|-0.499995828|-0.999995828
ccpoint_line3_box_threshold_2|-1|1.8626456e-06|-2.23517463e-06|0.5|0.239583358|-4.47035049e-08|-7.4505806e-09
ccdispatch_nested_sphere|3|A-1:1|A-1:1|A-1:0
cccontact_nested_sphere|2|1:-1|0:-1
ccpoint_nested_sphere_0|0.999999166|0.00133262319|5.28370365e-05|2.24999952|0.000666311593|2.64185182e-05|-0.0498942733
ccpoint_nested_sphere_1|-1|1.39699352e-09|1.04664305e-05|1.25000513|-0.0010695149|0.0010849355|-0.749994814
ccdispatch_sphere_nested|3|B-1:1|B-1:1|B-1:0
cccontact_sphere_nested|2|-1:1|-1:0
ccpoint_sphere_nested_0|0.999999166|0.00133262319|5.28370365e-05|2.24999952|0.000666311593|2.64185182e-05|-0.0498942733
ccpoint_sphere_nested_1|-1|1.39699352e-09|1.04664305e-05|1.25000513|-0.0010695149|0.0010849355|-0.749994814
ccdispatch_line3_pair2|6|A-1:2|B-1:1|A-1:1|B-1:1|A-1:1|B-1:0
cccontact_line3_pair2|3|2:1|1:1|1:0
ccpoint_line3_pair2_0|-1|2.09549555e-09|1.88939193e-05|1.50000942|0.00151802995|0.00155581255|-0.999990582
ccpoint_line3_pair2_1|-1|0|0|1.5|0|0|0
ccpoint_line3_pair2_2|-0.159731388|-0.982497215|-0.0958388522|0.951237261|-0.108198941|0.440672517|-0.619033754
ccdispatch_line3_pair2_rot60|10|A-1:2|B-1:1|A-1:1|B-1:1|A-1:2|B-1:0|A-1:1|B-1:0|A-1:0|B-1:0
cccontact_line3_pair2_rot60|5|2:1|1:1|2:0|1:0|0:0
ccpoint_line3_pair2_rot60_0|-1.67638035e-07|-1|1.67638035e-07|1.66666663|0.26666671|-0.333333254|-0.23333329
ccpoint_line3_pair2_rot60_1|-0.529999077|-0.847998261|0|1.40166724|0.34266758|-0.333333343|-0.185533881
ccpoint_line3_pair2_rot60_2|1|-8.83235487e-07|-1.04679759e-06|1.55693996|0.022708619|0.154414341|-0.0569399893
ccpoint_line3_pair2_rot60_3|1.75059868e-05|-1|1.64535231e-05|1.26339972|-0.349501312|0.261849672|-0.849501312
ccpoint_line3_pair2_rot60_4|-1|0|-2.6169954e-07|0.44306004|0.177066773|-0.154526666|-0.0569399595
ccdispatch_line3_pair2_threshold|10|A-1:2|B-1:1|A-1:1|B-1:1|A-1:2|B-1:0|A-1:1|B-1:0|A-1:0|B-1:0
cccontact_line3_pair2_threshold|5|2:1|1:1|2:0|1:0|0:0
ccpoint_line3_pair2_threshold_0|-1|2.09549555e-09|1.88939193e-05|1.50000942|0.00151802995|0.00155581255|-0.999990582
ccpoint_line3_pair2_threshold_1|-1|0|0|1.5|0|0|0
ccpoint_line3_pair2_threshold_2|0.99999994|-1.15988132e-05|0|1.2998656|-0.000190082472|0.5|0.200134411
ccpoint_line3_pair2_threshold_3|-0.159731388|-0.982497215|-0.0958388522|0.951237261|-0.108198941|0.440672517|-0.619033754
ccpoint_line3_pair2_threshold_4|-1|1.04118882e-07|0|0.700000167|0.0001991928|0.5|0.200000137
ccdispatch_line3_pair2_far|0
cccontact_line3_pair2_far|0
ccdispatch_line3_sph3|12|A-1:2|B-1:2|A-1:2|B-1:1|A-1:1|B-1:1|A-1:2|B-1:0|A-1:1|B-1:0|A-1:0|B-1:0
cccontact_line3_sph3|6|2:2|2:1|1:1|2:0|1:0|0:0
ccpoint_line3_sph3_0|-1|0|0|2.5|0|0|0
ccpoint_line3_sph3_1|-1|2.09549555e-09|1.88939193e-05|1.50000942|0.00151802995|0.00155581255|-0.999990582
ccpoint_line3_sph3_2|-1|0|0|1.5|0|0|0
ccpoint_line3_sph3_3|1|0|0|1.5|0|0|0
ccpoint_line3_sph3_4|-1|2.09549555e-09|1.88939193e-05|0.500009418|0.00151802995|0.00155581255|-0.999990582
ccpoint_line3_sph3_5|-1|0|0|0.5|0|0|0
ccdispatch_line3_rot60_sph3|6|A-1:1|B-1:1|A-1:1|B-1:0|A-1:0|B-1:0
cccontact_line3_rot60_sph3|3|1:1|1:0|0:0
ccpoint_line3_rot60_sph3_0|-0.723962307|0.472791731|-0.502341151|1.63801885|0.236395866|-0.251170576|0.252034247
ccpoint_line3_rot60_sph3_1|-0.298142165|0.596285105|-0.745355904|0.850928903|0.298142552|-0.372677952|-0.425464392
ccpoint_line3_rot60_sph3_2|-0.666666329|0.33333385|-0.666666687|0.666666865|0.166666925|-0.333333343|-0.433333337
ccdispatch_line3_sph3_rot60|6|A-1:2|B-1:1|A-1:2|B-1:0|A-1:1|B-1:0
cccontact_line3_sph3_rot60|3|2:1|2:0|1:0
ccpoint_line3_sph3_rot60_0|-1|-1.62051811e-06|2.62635695e-06|1.96667218|0.364724725|-0.332749993|-0.533327818
ccpoint_line3_sph3_rot60_1|-1.61787614e-06|1|2.73485603e-05|1.80148149|0.199984491|0.00157925789|-0.699984491
ccpoint_line3_sph3_rot60_2|-1|9.93410936e-08|-1.4901164e-07|1.29999995|-0.299999952|-7.45058202e-08|-0.200000048
ccdispatch_line3_sph3_rot60_threshold|6|A-1:2|B-1:1|A-1:2|B-1:0|A-1:1|B-1:0
cccontact_line3_sph3_rot60_threshold|3|2:1|2:0|1:0
ccpoint_line3_sph3_rot60_threshold_0|-1|0|0|2.03666663|0.366666675|-0.333333343|-0.463333368
ccpoint_line3_sph3_rot60_threshold_1|0|1|3.98740303e-06|1.87076998|0.199997962|0.00058265496|-0.699997962
ccpoint_line3_sph3_rot60_threshold_2|-1|8.05468261e-08|-1.20820232e-07|1.37|-0.299999952|-6.0410116e-08|-0.130000055
ccdispatch_line3_sph4|14|A-1:2|B-1:3|A-1:2|B-1:2|A-1:1|B-1:2|A-1:2|B-1:1|A-1:1|B-1:1|A-1:1|B-1:0|A-1:0|B-1:0
cccontact_line3_sph4|7|2:3|2:2|1:2|2:1|1:1|1:0|0:0
ccpoint_line3_sph4_0|-1|5.9604556e-08|5.9604556e-08|1.94999993|8.07642891e-07|8.07642891e-07|-0.550000072
ccpoint_line3_sph4_1|1|2.26140992e-06|1.64921965e-10|2.24999881|0.000758750481|4.31976296e-05|-0.749998868
ccpoint_line3_sph4_2|-1|0|0|1.25|0|0|-0.25
ccpoint_line3_sph4_3|1|0|0|1.54999995|0|0|-0.0499999821
ccpoint_line3_sph4_4|-1|1.77751117e-05|1.77751117e-05|0.549999952|4.56309681e-06|4.54819565e-06|-0.950000048
ccpoint_line3_sph4_5|1|0|0|0.850000024|0|0|-0.349999994
ccpoint_line3_sph4_6|-1|4.10625034e-06|5.37303257e-10|-0.149997905|0.000720873126|0.000715526403|-0.64999789
ccdispatch_line3_notree_cyl2|3|A-1:2|A-1:1|B-1:0
cccontact_line3_notree_cyl2|1|1:0
ccpoint_line3_notree_cyl2_0|-0.105855271|0.977158487|-0.184271634|1.095294|0.5|-0.0247920901|-0.651679814
ccdispatch_notree_cyl2_line3|2|A-1:1|B-1:1
cccontact_notree_cyl2_line3|1|1:1
ccpoint_notree_cyl2_line3_0|-0.159731388|-0.982497215|-0.0958388522|1.95123732|-0.108198941|0.440672517|-0.619033754
";

    /// One `n|entry|entry|...` row, with the entries checked against the count
    /// the row states -- so a row that lost an entry fails as a row-shape error
    /// rather than as a shorter sequence the port happens to match.
    fn trace_row(name: &str) -> Vec<&'static str> {
        let line = BULLET_REFERENCE
            .lines()
            .find(|l| l.split('|').next() == Some(name))
            .unwrap_or_else(|| panic!("{name}: no such row in BULLET_REFERENCE"));
        let f: Vec<&str> = line.split('|').collect();
        let count: usize = f[1]
            .parse()
            .unwrap_or_else(|e| panic!("{name}: field 1 ({:?}): {e}", f[1]));
        assert_eq!(
            f.len() - 2,
            count,
            "{name}: the row states {count} entries and carries {}",
            f.len() - 2
        );
        f[2..].to_vec()
    }

    /// Every case above except `line3_box`, which
    /// [`a_box_child_pair_is_reported_unported_not_routed_into_gjk`] reads.
    #[test]
    fn bullet_reference_compound_algorithm() {
        let mut bad = Vec::new();
        let mut covered: Vec<String> = vec!["line3_box".to_string()];

        let mut case = |name: &str,
                        shape0: &Shape,
                        t0: Transform,
                        shape1: &Shape,
                        t1: Transform,
                        closest_point_distance_threshold: Scalar| {
            covered.push(name.to_string());
            let wrap0 = CollisionObjectWrapper::new(shape0, t0, 0);
            let wrap1 = CollisionObjectWrapper::new(shape1, t1, 1);
            let mut out = TraceResult::new(wrap0, wrap1, closest_point_distance_threshold);

            if let Err(unported) =
                process_collision(&wrap0, &wrap1, DispatchTable::ClosestPoints, &mut out)
            {
                bad.push(format!("{name}: stopped on {unported:?}"));
                return;
            }

            for (what, got, want) in [
                (
                    "dispatches",
                    &out.dispatches,
                    trace_row(&format!("ccdispatch_{name}")),
                ),
                (
                    "contacts",
                    &out.contacts,
                    trace_row(&format!("cccontact_{name}")),
                ),
            ] {
                if got.len() != want.len() || got.iter().zip(&want).any(|(g, w)| g != w) {
                    bad.push(format!(
                        "{name}.{what}: port [{}], bullet [{}]",
                        got.join(" "),
                        want.join(" ")
                    ));
                }
            }

            for (k, &(normal, point, depth)) in out.geometry.iter().enumerate() {
                let point_row = format!("ccpoint_{name}_{k}");
                let f = row(BULLET_REFERENCE, &point_row, 8);
                let n = |i: usize| -> Scalar {
                    f[i].parse()
                        .unwrap_or_else(|e| panic!("{point_row}: field {i} ({:?}): {e}", f[i]))
                };
                let at_k = format!("{name}[{k}]");
                diff_vec3(
                    &mut bad,
                    &at_k,
                    "normal",
                    normal,
                    Vec3::new(n(1), n(2), n(3)),
                );
                diff_vec3(&mut bad, &at_k, "point", point, Vec3::new(n(4), n(5), n(6)));
                diff(&mut bad, &at_k, "depth", depth, n(7));
            }
        };

        let line3 = line3(true, unit_box);
        let no_tree3 = line3_no_tree();
        let sph3 = {
            let mut compound = CompoundShape::new(true);
            for i in 0..3 {
                compound.add_child_shape(at(i as Scalar, 0.0, 0.0), sphere());
            }
            Shape::Compound(compound)
        };
        // Four children at 0.7 spacing: not `line3`'s tree shape translated,
        // which is what a symmetric pair of trees makes `MycollideTT`'s
        // internal/internal push order its own mirror image under.
        let sph4 = {
            let mut compound = CompoundShape::new(true);
            for i in 0..4 {
                compound.add_child_shape(at(i as Scalar * 0.7, 0.0, 0.0), sphere());
            }
            Shape::Compound(compound)
        };
        let sphere = sphere();

        // A compound against a convex, both ways round: `A` versus `B` in the
        // trace is the swap detection's answer.
        case(
            "line3_sphere",
            &line3,
            IDENTITY,
            &sphere,
            at(1.0, 0.0, 0.0),
            0.0,
        );
        case(
            "sphere_line3",
            &sphere,
            at(1.0, 0.0, 0.0),
            &line3,
            IDENTITY,
            0.0,
        );
        case(
            "line3_sphere_off",
            &line3,
            IDENTITY,
            &sphere,
            at(1.0, 0.9, 0.3),
            0.0,
        );
        // A rotated compound: the tree query runs in compound space and the
        // per-child test in world space, so the two culls stop agreeing.
        case(
            "line3_rot60_sphere",
            &line3,
            rot60_at(0.2, 0.1, 0.0),
            &sphere,
            at(1.0, 0.0, 0.0),
            0.0,
        );
        case(
            "line3_sphere_far",
            &line3,
            IDENTITY,
            &sphere,
            at(9.0, 0.0, 0.0),
            0.0,
        );

        // No tree: index order, and `TestAabbAgainstAabb2` as the only cull.
        case(
            "no_tree3_sphere",
            &no_tree3,
            IDENTITY,
            &sphere,
            at(1.0, 0.0, 0.0),
            0.0,
        );
        case(
            "no_tree_window",
            &no_tree_pair(),
            IDENTITY,
            &sphere,
            at(1.01, 0.0, 0.0),
            0.0,
        );
        case("empty_sphere", &empty(), IDENTITY, &sphere, IDENTITY, 0.0);

        // A non-zero threshold, which is what makes `extendAabb`,
        // `extraExtends` and `thresholdVec` add something.
        case(
            "line3_sphere_threshold",
            &line3,
            IDENTITY,
            &sphere,
            at(1.0, 0.0, 0.0),
            0.25,
        );
        case(
            "no_tree_window_threshold",
            &no_tree_pair(),
            IDENTITY,
            &sphere,
            at(1.01, 0.0, 0.0),
            0.25,
        );
        case(
            "line3_sphere_far_threshold",
            &line3,
            IDENTITY,
            &sphere,
            at(3.9, 0.0, 0.0),
            1.0,
        );

        case(
            "line3_box_threshold",
            &line3,
            IDENTITY,
            &margin_box(),
            at(1.0, 0.0, 0.0),
            0.25,
        );

        // A compound child, which re-enters the same traversal one level down.
        case(
            "nested_sphere",
            &nested(),
            IDENTITY,
            &sphere,
            at(1.75, 0.0, 0.0),
            0.0,
        );
        case(
            "sphere_nested",
            &sphere,
            at(1.75, 0.0, 0.0),
            &nested(),
            IDENTITY,
            0.0,
        );

        // Compound against compound -- `MycollideTT` rather than
        // `collideTVNoStackAlloc`.
        case(
            "line3_pair2",
            &line3,
            IDENTITY,
            &pair2(),
            at(1.0, 0.0, 0.0),
            0.0,
        );
        case(
            "line3_pair2_rot60",
            &line3,
            IDENTITY,
            &pair2(),
            rot60_at(1.0, 0.1, 0.0),
            0.0,
        );
        case(
            "line3_pair2_threshold",
            &line3,
            IDENTITY,
            &pair2(),
            at(1.0, 0.0, 0.0),
            0.25,
        );
        case(
            "line3_pair2_far",
            &line3,
            IDENTITY,
            &pair2(),
            at(9.0, 0.0, 0.0),
            0.0,
        );

        // Three children each way, which is what reaches `MycollideTT`'s
        // internal/internal arm at the root and both internal/leaf arms below
        // it with enough surviving pairs for the push order to show.
        case(
            "line3_sph3",
            &line3,
            IDENTITY,
            &sph3,
            at(1.0, 0.0, 0.0),
            0.0,
        );
        case(
            "line3_rot60_sph3",
            &line3,
            rot60_at(0.2, 0.1, 0.0),
            &sph3,
            at(1.0, 0.0, 0.0),
            0.0,
        );
        // `MyIntersect` re-boxes tree 1's leaf volume through `xform`, which
        // under this rotation inflates each sphere's local cube from +/-0.5 to
        // +/-0.8333 for the tree test while the leaf callback still measures
        // the sphere's own +/-0.5 world box. Child pair (2,2) sits in that gap.
        case(
            "line3_sph3_rot60",
            &line3,
            IDENTITY,
            &sph3,
            rot60_at(1.8, -0.3, 0.0),
            0.0,
        );
        // The same pair at 1.20 apart against a 0.15 threshold: `thresholdVec`
        // grows box 0 only, which leaves 0.05 of separation.
        case(
            "line3_sph3_rot60_threshold",
            &line3,
            IDENTITY,
            &sph3,
            rot60_at(1.87, -0.3, 0.0),
            0.15,
        );

        // Three against four, offset so that seven of the twelve pairs survive
        // and no pair sits on an exact AABB tie.
        case(
            "line3_sph4",
            &line3,
            IDENTITY,
            &sph4,
            at(0.35, 0.0, 0.0),
            0.0,
        );

        // One side without a tree: the whole query drops back to the
        // single-compound algorithm, which treats the *first* operand as the
        // compound whichever one lost its tree.
        case(
            "line3_notree_cyl2",
            &line3,
            IDENTITY,
            &no_tree_cyl2(),
            at(1.0, 0.0, 0.0),
            0.0,
        );
        case(
            "notree_cyl2_line3",
            &no_tree_cyl2(),
            IDENTITY,
            &line3,
            at(1.0, 0.0, 0.0),
            0.0,
        );

        let mut want: Vec<String> = BULLET_REFERENCE
            .lines()
            .filter_map(|l| l.split('|').next())
            .filter_map(|n| n.strip_prefix("ccdispatch_"))
            .map(str::to_string)
            .collect();
        want.sort();
        covered.sort();
        assert_eq!(
            covered, want,
            "the cases and BULLET_REFERENCE disagree on which rows exist"
        );
        assert!(
            bad.is_empty(),
            "{} deviations:\n{}",
            bad.len(),
            bad.join("\n")
        );
    }

    /// `no_tree3` -- `line3` with the dynamic tree switched off.
    fn line3_no_tree() -> Shape {
        line3(false, unit_box)
    }

    /// Every contact the fixture counts has a geometry row, and no row is
    /// stranded: without this a `ccpoint_` row for a contact the `cccontact_`
    /// count does not reach is never read by anything.
    #[test]
    fn every_contact_row_has_exactly_one_geometry_row() {
        let point_rows = BULLET_REFERENCE
            .lines()
            .filter(|l| l.starts_with("ccpoint_"))
            .count();
        let counted: usize = BULLET_REFERENCE
            .lines()
            .filter(|l| l.starts_with("cccontact_"))
            .map(|l| {
                let n = l.split('|').nth(1).expect("a cccontact row states a count");
                n.parse::<usize>()
                    .unwrap_or_else(|e| panic!("{l}: field 1 ({n:?}): {e}"))
            })
            .sum();
        assert_eq!(point_rows, counted);
    }

    /// The one cell the two create-func tables disagree on, reached as a child
    /// pair.
    ///
    /// With the threshold at zero the children are looked up in the
    /// contact-points table, where two boxes are `btBoxBoxCollisionAlgorithm`
    /// -- which this crate does not carry. The port stops there rather than
    /// routing the pair into GJK, and bullet's own rows show why that would be
    /// wrong: the box-box detector answers with four contacts per child where
    /// GJK answers with one.
    #[test]
    fn a_box_child_pair_is_reported_unported_not_routed_into_gjk() {
        let compound = line3(true, unit_box);
        let other = margin_box();
        let t1 = at(1.0, 0.0, 0.0);
        let wrap0 = CollisionObjectWrapper::new(&compound, IDENTITY, 0);
        let wrap1 = CollisionObjectWrapper::new(&other, t1, 1);
        let mut out = TraceResult::new(wrap0, wrap1, 0.0);

        let unported = process_collision(&wrap0, &wrap1, DispatchTable::ClosestPoints, &mut out)
            .expect_err("box against box has no port");
        assert_eq!(
            unported,
            UnportedAlgorithm {
                algorithm: Algorithm::BoxBox,
                proxy_type0: BroadphaseNativeType::BOX_SHAPE,
                proxy_type1: BroadphaseNativeType::BOX_SHAPE,
            }
        );

        // Stopped at the first child rather than walking the rest into a result
        // that is already short of contacts.
        assert_eq!(out.dispatches, ["A-1:2"]);
        assert!(out.contacts.is_empty());

        assert_eq!(
            trace_row("ccdispatch_line3_box"),
            ["A-1:2", "A-1:1", "A-1:0"]
        );
        assert_eq!(trace_row("cccontact_line3_box").len(), 12);
        assert_eq!(trace_row("cccontact_line3_box_threshold").len(), 3);
    }

    /// The top-level `table` argument is the caller's, not a constant: the same
    /// pair answers differently in the two tables, and only the closest-points
    /// one -- what `processOverlap` asks for -- is ported.
    #[test]
    fn the_top_level_table_argument_reaches_find_algorithm() {
        let a = margin_box();
        let b = margin_box();
        let wrap0 = CollisionObjectWrapper::new(&a, IDENTITY, 0);
        let wrap1 = CollisionObjectWrapper::new(&b, at(1.0, 0.0, 0.0), 1);

        let mut out = TraceResult::new(wrap0, wrap1, 0.0);
        assert!(process_collision(&wrap0, &wrap1, DispatchTable::ClosestPoints, &mut out).is_ok());

        let mut out = TraceResult::new(wrap0, wrap1, 0.0);
        assert_eq!(
            process_collision(&wrap0, &wrap1, DispatchTable::ContactPoints, &mut out)
                .expect_err("box against box has no port in the contact-points table")
                .algorithm,
            Algorithm::BoxBox
        );
    }

    /// `MyIntersect` re-fits tree 1's leaf box after transforming it, so under
    /// a rotation it is strictly looser than the world-space test the leaf
    /// callback then applies -- which is what makes that second test able to
    /// reject a pair the traversal reached.
    #[test]
    fn my_intersect_is_looser_than_the_world_space_test_under_rotation() {
        let unit = DbvtVolume::from_mm(Vec3::new(-0.5, -0.5, -0.5), Vec3::new(0.5, 0.5, 0.5));
        let xform = rot60_at(0.0, 0.0, 0.0);
        let (min, max) = transform_aabb(unit.mi, unit.mx, 0.0, &xform);
        assert!(
            min.x < -0.5 && max.x > 0.5,
            "a rotated unit cube re-fits to {min:?}..{max:?}, which is not larger"
        );

        // A neighbour the re-fitted box reaches and the tight one does not: at
        // 1.05 apart the two unit cubes are clear of each other, but the re-fit
        // spans +/-0.8333.
        let neighbour = DbvtVolume::from_mm(Vec3::new(0.55, -0.5, -0.5), Vec3::new(1.55, 0.5, 0.5));
        assert!(my_intersect(&neighbour, &unit, &xform, 0.0));
        assert!(!test_aabb_against_aabb2(
            neighbour.mi,
            neighbour.mx,
            unit.mi,
            unit.mx
        ));
    }

    /// The result's body wrap is the *child's* while that child is dispatched,
    /// and the wrapper it displaced afterwards.
    ///
    /// Nothing in this crate reads it: the traversal passes the child wrapper
    /// as an argument as well, so a port that dropped `setBody0Wrap` entirely
    /// -- as this one did -- produced identical contacts here. What reads it is
    /// `addCastSingleResult`, which recovers the swept child's shape and pose
    /// from `first_col_obj_wrap` (`bullet_utils.hpp:470-473`) and has no other
    /// way to reach them.
    #[test]
    fn a_child_dispatch_installs_and_restores_the_body_wrap() {
        // No tree, so the two children are visited in index order and the
        // *last* one installed sits at `at(2, 0, 0)` -- a restore that never
        // runs is then visible in the final state. A fixture whose last child
        // is at the identity cannot see it: the value left behind is the one
        // the restore would have written.
        let mut compound = CompoundShape::new(false);
        compound.add_child_shape(at(1.0, 0.0, 0.0), unit_box());
        compound.add_child_shape(at(2.0, 0.0, 0.0), unit_box());
        let compound = Shape::Compound(compound);
        let other = sphere();
        let t1 = at(1.5, 0.0, 0.0);
        let wrap0 = CollisionObjectWrapper::new(&compound, IDENTITY, 0);
        let wrap1 = CollisionObjectWrapper::new(&other, t1, 1);
        let mut out = TraceResult::new(wrap0, wrap1, 0.0);

        process_collision(&wrap0, &wrap1, DispatchTable::ClosestPoints, &mut out)
            .expect("box children against a sphere are all convex-convex");

        assert_eq!(out.dispatches, ["A-1:0", "A-1:1"]);
        assert_eq!(out.wraps.len(), 2, "both children reach a contact");
        for (i, (body0, body1)) in out.wraps.iter().enumerate() {
            let child: Scalar = out.contacts[i]
                .split(':')
                .next()
                .and_then(|index| index.parse().ok())
                .expect("the contact records the child index it was dispatched under");
            assert_eq!(
                body0.origin,
                at(child + 1.0, 0.0, 0.0).origin,
                "body 0's wrap is child {child}'s world transform, not the compound's"
            );
            assert_eq!(*body1, t1, "the convex side is never replaced");
        }

        assert_eq!(
            out.state.body0_wrap.world_transform, IDENTITY,
            "the compound's own wrapper is back once the traversal returns"
        );
        assert_eq!(
            out.state.body0_wrap.object_transform, IDENTITY,
            "the object's transform is the same at every depth"
        );
    }

    /// `isSwapped` is the manifold's body 0 against the result's, and the
    /// swapped compound arm is what makes them differ: the child is dispatched
    /// as body 0 of its own query while the result's body 0 stays the convex
    /// operand the caller passed first.
    ///
    /// `addContactPoint` reads it to decide which world position is the cast
    /// object's (`bullet_utils.hpp:588-601`), so a manifold that could not
    /// answer -- as this port's could not, having no bodies at all -- reports
    /// every pair unswapped and reads the wrong point for half of them.
    #[test]
    fn a_swapped_compound_arm_reports_its_contacts_as_swapped() {
        let compound = line3(true, unit_box);
        let other = sphere();
        let t = at(1.0, 0.0, 0.6);

        let compound_first = CollisionObjectWrapper::new(&compound, IDENTITY, 0);
        let convex_second = CollisionObjectWrapper::new(&other, t, 1);
        let mut out = TraceResult::new(compound_first, convex_second, 0.0);
        process_collision(
            &compound_first,
            &convex_second,
            DispatchTable::ClosestPoints,
            &mut out,
        )
        .expect("boxes against a sphere are convex-convex");
        assert!(!out.swapped.is_empty(), "the fixture must reach a contact");
        assert!(
            out.swapped.iter().all(|s| !s),
            "the compound is the result's body 0 and its child's query's body 0"
        );

        let convex_first = CollisionObjectWrapper::new(&other, t, 0);
        let compound_second = CollisionObjectWrapper::new(&compound, IDENTITY, 1);
        let mut out = TraceResult::new(convex_first, compound_second, 0.0);
        process_collision(
            &convex_first,
            &compound_second,
            DispatchTable::ClosestPoints,
            &mut out,
        )
        .expect("boxes against a sphere are convex-convex");
        assert!(!out.swapped.is_empty(), "the fixture must reach a contact");
        assert!(
            out.swapped.iter().all(|s| *s),
            "`SwappedCreateFunc` dispatches the child first, the result still calls the sphere body 0"
        );
    }

    /// With a compound on each side both wraps are installed and both are put
    /// back -- the second half of what
    /// [`a_child_dispatch_installs_and_restores_the_body_wrap`] pins for one.
    ///
    /// Every child sits at a non-identity local transform on both sides, so
    /// neither restore can be satisfied by the value a missing one would leave.
    #[test]
    fn a_compound_compound_child_pair_installs_and_restores_both_wraps() {
        let mut boxes = CompoundShape::new(true);
        boxes.add_child_shape(at(1.0, 0.0, 0.0), unit_box());
        boxes.add_child_shape(at(2.0, 0.0, 0.0), unit_box());
        let boxes = Shape::Compound(boxes);

        let mut spheres = CompoundShape::new(true);
        spheres.add_child_shape(at(1.0, 0.0, 0.0), sphere());
        spheres.add_child_shape(at(2.0, 0.0, 0.0), sphere());
        let spheres = Shape::Compound(spheres);

        let t1 = at(0.0, 0.0, 0.9);
        let wrap0 = CollisionObjectWrapper::new(&boxes, IDENTITY, 0);
        let wrap1 = CollisionObjectWrapper::new(&spheres, t1, 1);
        let mut out = TraceResult::new(wrap0, wrap1, 0.0);

        process_collision(&wrap0, &wrap1, DispatchTable::ClosestPoints, &mut out)
            .expect("boxes against spheres are all convex-convex");

        assert!(!out.wraps.is_empty(), "the fixture must reach a contact");
        for (i, (body0, body1)) in out.wraps.iter().enumerate() {
            let mut indices = out.contacts[i].split(':');
            let child0: Scalar = indices.next().and_then(|c| c.parse().ok()).expect("index0");
            let child1: Scalar = indices.next().and_then(|c| c.parse().ok()).expect("index1");
            assert_eq!(body0.origin, at(child0 + 1.0, 0.0, 0.0).origin);
            assert_eq!(body1.origin, (t1 * at(child1 + 1.0, 0.0, 0.0)).origin);
        }

        assert_eq!(out.state.body0_wrap.world_transform, IDENTITY);
        assert_eq!(out.state.body1_wrap.world_transform, t1);
    }

    /// A nested compound reports `COMPOUND_SHAPE_PROXYTYPE`, so its dispatch
    /// goes back through the compound rows rather than into the narrow phase.
    #[test]
    fn a_compound_child_dispatches_as_a_compound() {
        let Shape::Compound(outer) = nested() else {
            panic!("nested() builds a compound");
        };
        assert_eq!(
            outer.child_shape(1).shape_type(),
            BroadphaseNativeType::COMPOUND_SHAPE
        );
        assert_eq!(
            find_algorithm(
                DispatchTable::ContactPoints,
                BroadphaseNativeType::COMPOUND_SHAPE,
                BroadphaseNativeType::SPHERE_SHAPE,
            ),
            Algorithm::Compound
        );
    }

    /// An empty compound returns before it can read a tree. Upstream's
    /// `collideTVNoStackAlloc` opens with `if (root)` and this port's takes an
    /// `Option`, so the early return changes no answer -- it is kept because it
    /// is upstream's, and this is what it rests on.
    #[test]
    fn an_empty_compound_returns_before_the_tree_branch() {
        let compound = empty();
        let other = sphere();
        let wrap0 = CollisionObjectWrapper::new(&compound, IDENTITY, 0);
        let wrap1 = CollisionObjectWrapper::new(&other, IDENTITY, 1);
        let mut out = TraceResult::new(wrap0, wrap1, 0.0);
        assert!(process_collision(&wrap0, &wrap1, DispatchTable::ClosestPoints, &mut out).is_ok());
        assert!(out.dispatches.is_empty());

        let Shape::Compound(inner) = &compound else {
            panic!("empty() builds a compound");
        };
        assert_eq!(inner.num_child_shapes(), 0);
        assert_eq!(inner.dynamic_aabb_tree().and_then(|tree| tree.root), None);
    }

    /// `IDENTITY` is the transform the fixtures compose every child against, so
    /// a composed child world transform is only comparable to bullet's if it is
    /// the actual identity.
    #[test]
    fn the_identity_fixture_is_the_identity() {
        assert_eq!(IDENTITY.basis, Matrix3::identity());
        assert_eq!(IDENTITY.origin, Vec3::zero());
    }
}
