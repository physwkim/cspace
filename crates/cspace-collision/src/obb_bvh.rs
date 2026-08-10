// Copyright (c) 2011-2014, Willow Garage, Inc.
// Copyright (c) 2014-2016, Open Source Robotics Foundation
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from fcl @ e5efcc41b57b2d0da3bf183480f1298a6d531f44 (0.7.0-17-ge5efcc4):
//   include/fcl/math/bv/OBB.h (OBB<S>)
//   include/fcl/math/bv/OBB-inl.h (OBB<S>::size, overlap, obbDisjoint)
//   include/fcl/geometry/bvh/BV_node.h (BVNode<BV>)
//   include/fcl/geometry/bvh/BVH_model-inl.h (BVHModel<BV>::buildTree,
//     BVHModel<BV>::recursiveBuildTree)
//   include/fcl/geometry/bvh/detail/BV_splitter-inl.h
//     (ComputeRuleMeanImpl<S, OBB<S>>)
//   include/fcl/narrowphase/detail/traversal/traversal_recurse-inl.h
//     (collisionRecurse, and its MeshCollisionTraversalNodeOBB<S> overload's
//     carried relative frame)
//   include/fcl/narrowphase/detail/traversal/collision/mesh_collision_traversal_node-inl.h
//     (MeshCollisionTraversalNodeOBB<S>::BVTesting)
//   include/fcl/narrowphase/detail/traversal/collision/bvh_collision_traversal_node-inl.h
//     (BVHCollisionTraversalNode<BV>::firstOverSecond)
//   include/fcl/narrowphase/collision_request-inl.h
//     (CollisionRequest<S>::isSatisfied)

//! `fcl::BVHModel<fcl::OBBRSSd>`'s oriented-box hierarchy, and the two-tree
//! descent `fcl::MeshCollisionTraversalNode` runs over a pair of them.
//!
//! # Why a second hierarchy exists at all
//!
//! `parry3d_f64`'s `TriMesh` already carries a `Bvh`, and
//! `crate::parry::trimesh_pair_contact` descends two of them together. What
//! it cannot do is bound a triangle any tighter than an axis-aligned box,
//! because axis-aligned boxes are what that `Bvh` stores. For a mesh whose
//! triangles are long and diagonal -- which is most of a robot link's
//! collision mesh -- that box is far larger than the triangle in it, and the
//! looseness compounds at every level of the descent.
//!
//! Measured on fanuc/cage under STOMP, with the axis-aligned hierarchy: `600`
//! leaf pairs per mesh pair reach the leaf test, and the exact triangle test
//! there rejects `586` of them. The bound is `40x` looser than the answer.
//! Upstream closes that gap by fitting each node's box to the triangles it
//! covers rather than to the world axes, which is what this module is.
//!
//! # Scope
//!
//! Collision only, `OBB` only -- not the `RSS` half of `OBBRSS`, which
//! upstream uses for its distance queries (`fcl::MeshDistanceTraversalNode`)
//! and which nothing here calls. The port that would need it is the
//! separation-distance branch, which reaches `parry`'s own dispatch.

use parry3d_f64::math::{Matrix, Vector};
use std::ops::ControlFlow;

/// One node's oriented box, in its mesh's own frame.
///
/// Field-for-field `fcl::OBB<double>` (`fcl/math/bv/OBB.h`): `axis`'s
/// *columns* are the box's three axes, `center` is upstream's `To`, and
/// `extent` is the half-width along each axis. The names differ from
/// upstream's only where upstream's are opaque.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Obb {
    pub(crate) axis: Matrix,
    pub(crate) center: Vector,
    pub(crate) extent: Vector,
}

impl Obb {
    /// `fcl::OBB<S>::size` (`OBB-inl.h:206`), which is what upstream's
    /// descent-order rule compares -- the squared norm, not a volume.
    fn size(&self) -> f64 {
        self.extent.length_squared()
    }
}

/// `fcl::obbDisjoint` (`OBB-inl.h:398`): the fifteen-axis separating-axis
/// test between two boxes, given the second box's rotation `b` and
/// translation `t` **expressed in the first box's frame**, and the two boxes'
/// half-extents.
///
/// `true` means provably disjoint. The `reps` epsilon is upstream's own and
/// inflates the absolute rotation matrix, so it can only make the answer more
/// conservative: it keeps a cross-product axis that has collapsed to zero
/// (parallel box axes) from separating the boxes on a direction that does not
/// exist.
///
/// The nine cross axes are unnormalised, which is why a caller that wants a
/// margin cannot simply add one here -- it has to grow the half-extents
/// instead, as [`ObbTree::leaf_pairs`] does.
fn obb_disjoint(b: &Matrix, t: &Vector, ea: &Vector, eb: &Vector) -> bool {
    const REPS: f64 = 1.0e-6;
    let reps = Vector::splat(REPS);
    let bf = Matrix::from_cols(
        b.x_axis.abs() + reps,
        b.y_axis.abs() + reps,
        b.z_axis.abs() + reps,
    );
    // `bf.row(i)` in upstream's row-major reading is the vector of |b[i][j]|
    // over j, which in glam's column-major storage is the i-th component of
    // each column.
    let row = |i: usize| Vector::new(bf.x_axis[i], bf.y_axis[i], bf.z_axis[i]);

    // The three axes of the first box.
    for i in 0..3 {
        if t[i].abs() > ea[i] + row(i).dot(*eb) {
            return true;
        }
    }
    // The three axes of the second box.
    for j in 0..3 {
        if t.dot(b.col(j)).abs() > eb[j] + bf.col(j).dot(*ea) {
            return true;
        }
    }
    // The nine cross products of one axis from each.
    for i in 0..3 {
        let (i1, i2) = ((i + 1) % 3, (i + 2) % 3);
        for j in 0..3 {
            let (j1, j2) = ((j + 1) % 3, (j + 2) % 3);
            let s = t[i2] * b.col(j)[i1] - t[i1] * b.col(j)[i2];
            let ra = ea[i1] * bf.col(j)[i2] + ea[i2] * bf.col(j)[i1];
            let rb = eb[j1] * bf.col(j2)[i] + eb[j2] * bf.col(j1)[i];
            if s.abs() > ra + rb {
                return true;
            }
        }
    }
    false
}

/// A node of the hierarchy: `fcl::BVNode<BV>` with upstream's negative
/// `first_child` encoding replaced by a sentinel.
///
/// Upstream stores a leaf's triangle as `-(primitive + 1)` in the same field
/// the children go in (`BVH_model-inl.h:884`); here the two are separate and
/// `first_child == NO_CHILD` is what makes a node a leaf. Children are always
/// adjacent, so one index reaches both.
#[derive(Clone, Copy, Debug)]
struct Node {
    obb: Obb,
    first_child: u32,
    primitive: u32,
}

const NO_CHILD: u32 = u32::MAX;

impl Node {
    fn is_leaf(&self) -> bool {
        self.first_child == NO_CHILD
    }
}

/// Everything a two-tree descent holds fixed: the other tree, the relative
/// pose, the margin, and the sink for the pairs it finds.
///
/// Only the two node indices change from one level to the next, so the rest
/// is threaded as one borrow rather than as six recursion parameters.
struct Descent<'a, F: FnMut(u32, u32) -> ControlFlow<()>> {
    other: &'a ObbTree,
    rot12: Matrix,
    t12: Vector,
    grow: Vector,
    leaf: &'a mut F,
}

/// The second tree's node box carried into the first tree's frame: the two
/// products of `rot12`/`t12` with that node that [`ObbTree::recurse`]'s
/// overlap test needs.
///
/// Both depend on the second node alone, and `firstOverSecond` moves exactly
/// one side per level, so on every step that descends the *first* tree they
/// are the parent's values unchanged. Threading them down the recursion is
/// what makes that reuse possible; rebuilding them at each node, as this
/// descent used to, recomputes a matrix product per node pair that changed
/// only every other level.
///
/// # Upstream
///
/// Threading a relative frame down the descent rather than rebuilding it at
/// each node pair is upstream's idea, not this port's. `collisionRecurse`'s
/// `MeshCollisionTraversalNodeOBB<S>` overload
/// (`traversal_recurse-inl.h:134`) takes `(R, T)` as arguments and updates
/// them at every step -- `:164`-`:166` on a first-tree descent, `:190`-`:196`
/// on a second-tree one. What it buys is visible in the two `BVTesting`
/// overloads: the carried one (`mesh_collision_traversal_node-inl.h:325`)
/// hands `(Rc, Tc)` straight to `obbDisjoint`, while the uncarried one
/// (`:292`) calls `overlap(R, T, bv1, bv2)`, which rebuilds the frame from
/// both boxes on every call.
///
/// This port carries half of it. `Carried` holds the products that depend on
/// the second node; [`ObbTree::recurse`] still applies the first node's
/// `axis.transpose()` itself, once per node pair, so the first tree's half of
/// upstream's saving is still on the table. Taking it would *re-associate*
/// the floating-point products rather than merely reuse them, and so could
/// move an overlap verdict -- which is the difference between that step and
/// this one, and the reason this one stopped here.
///
/// The reuse is safe by construction rather than by discipline: [`Carried`]
/// is only ever built by [`Carried::of`] from the node it describes, and the
/// only call that passes a parent's value through is the one that leaves `i2`
/// alone. There is no path that pairs it with a different second node.
///
/// Bit-for-bit the same values the per-node form computed -- same operands,
/// same association -- so no overlap verdict moves.
#[derive(Clone, Copy)]
struct Carried {
    /// `rot12 * n2.axis`.
    axis: Matrix,
    /// `rot12 * n2.center + t12`.
    center: Vector,
}

impl Carried {
    fn of(rot12: &Matrix, t12: &Vector, obb: &Obb) -> Self {
        Self {
            axis: *rot12 * obb.axis,
            center: *rot12 * obb.center + *t12,
        }
    }
}

/// [`Descent`]'s one-tree counterpart, for a descent whose other side is a
/// single shape rather than a second hierarchy: only the node index changes
/// from one level to the next, so the rest is threaded as one borrow.
///
/// There is no relative pose here. The node test is the caller's
/// (`reaches`), which already knows where its own shape sits, so this tree
/// never has to put the two into one frame the way [`Descent`] does.
struct ShapeDescent<'a, R: FnMut(&Obb) -> bool, F: FnMut(u32) -> ControlFlow<()>> {
    grow: Vector,
    reaches: &'a mut R,
    leaf: &'a mut F,
}

/// An oriented-box hierarchy over one mesh's triangles, built once and reused
/// for every query against that mesh.
///
/// Every leaf holds exactly one triangle, as upstream's does by construction
/// -- `recursiveBuildTree` stops splitting only at `num_primitives == 1`
/// (`BVH_model-inl.h:882`).
#[derive(Clone, Debug)]
pub(crate) struct ObbTree {
    nodes: Vec<Node>,
}

impl ObbTree {
    /// `fcl::BVHModel<OBB>::buildTree` (`BVH_model-inl.h:833`) over the
    /// triangles of one mesh, in that mesh's own frame.
    ///
    /// Returns `None` for a mesh with no triangles: upstream refuses the same
    /// input (`BVH_ERR_BUILD_EMPTY_MODEL`), and a tree with no root has no
    /// node for the descent to start at.
    pub(crate) fn build(vertices: &[Vector], triangles: &[[u32; 3]]) -> Option<Self> {
        if triangles.is_empty() {
            return None;
        }
        // Upstream permutes `primitive_indices` in place and records ranges
        // into it; this keeps the same permutation and the same ranges.
        let mut order: Vec<u32> = (0..triangles.len() as u32).collect();
        let mut tree = ObbTree {
            nodes: Vec::with_capacity(2 * triangles.len()),
        };
        tree.nodes.push(Node {
            obb: Obb {
                axis: Matrix::IDENTITY,
                center: Vector::ZERO,
                extent: Vector::ZERO,
            },
            first_child: NO_CHILD,
            primitive: 0,
        });
        tree.build_node(0, vertices, triangles, &mut order, 0, triangles.len());
        Some(tree)
    }

    /// `recursiveBuildTree` (`BVH_model-inl.h:868`): fit this node's box to
    /// the triangles in `[first, first + count)`, then split them in two by
    /// upstream's mean rule and recurse.
    fn build_node(
        &mut self,
        node: usize,
        vertices: &[Vector],
        triangles: &[[u32; 3]],
        order: &mut [u32],
        first: usize,
        count: usize,
    ) {
        let mut points = Vec::with_capacity(count * 3);
        for &t in &order[first..first + count] {
            for v in triangles[t as usize] {
                points.push(vertices[v as usize]);
            }
        }
        let (axis, center, extent) = crate::obb_fit::fit_obb(&points);
        self.nodes[node].obb = Obb {
            axis,
            center,
            extent,
        };

        if count == 1 {
            self.nodes[node].first_child = NO_CHILD;
            self.nodes[node].primitive = order[first];
            return;
        }

        // `computeRule_mean` for an `OBB` (`BV_splitter-inl.h:274`): project
        // onto the box's first axis -- the one `fit_obb` puts the largest
        // eigenvalue on -- and cut at the mean of the triangle centroids.
        let split_vector = axis.col(0);
        let mut sum = 0.0;
        for &t in &order[first..first + count] {
            for v in triangles[t as usize] {
                sum += vertices[v as usize].dot(split_vector);
            }
        }
        let split_value = sum / (3.0 * count as f64);

        let centroid = |t: u32| {
            let [a, b, c] = triangles[t as usize];
            (vertices[a as usize] + vertices[b as usize] + vertices[c as usize]) / 3.0
        };
        let mut left = 0;
        for i in 0..count {
            if centroid(order[first + i]).dot(split_vector) <= split_value {
                order.swap(first + i, first + left);
                left += 1;
            }
        }
        // Upstream's own fallback for a rule that put everything on one side
        // (`BVH_model-inl.h:929`), which a symmetric mesh reaches often.
        if left == 0 || left == count {
            left = count / 2;
        }

        let first_child = self.nodes.len() as u32;
        self.nodes[node].first_child = first_child;
        let blank = Node {
            obb: self.nodes[node].obb,
            first_child: NO_CHILD,
            primitive: 0,
        };
        self.nodes.push(blank);
        self.nodes.push(blank);
        self.build_node(
            first_child as usize,
            vertices,
            triangles,
            order,
            first,
            left,
        );
        self.build_node(
            first_child as usize + 1,
            vertices,
            triangles,
            order,
            first + left,
            count - left,
        );
    }

    /// `fcl::collisionRecurse` over two `BVHModel<OBB>`s
    /// (`traversal_recurse-inl.h:84`): calls `leaf` once per triangle pair
    /// whose boxes are not provably `prediction` apart.
    ///
    /// `rot12`/`t12` are the second mesh's rotation and translation in the
    /// first's frame.
    ///
    /// `prediction` grows the *first* box rather than entering the
    /// separating-axis test as a margin, because that test's nine cross axes
    /// are unnormalised, so a margin added there would not be a distance.
    /// Growing one box is both sound and sufficient: a point within
    /// `prediction` of a box has `|local_k| <= extent_k + prediction` on every
    /// axis, so the grown box contains the whole `prediction`-neighbourhood of
    /// the original, and two triangles that close is two boxes that close.
    /// Growing both, which reads as the symmetric thing to do, would admit
    /// pairs up to `2 * prediction` apart -- sound but needlessly loose.
    ///
    /// Upstream has no counterpart to this: `fcl::collide` descends at zero
    /// margin, and it is the port's distance path that reaches
    /// `crate::parry::part_contact` with a `prediction` above zero.
    ///
    /// `leaf` returning [`ControlFlow::Break`] ends the descent, which is
    /// upstream's `canStop()`: `collisionRecurse` checks it between the two
    /// child recursions (`traversal_recurse-inl.h:171`), and
    /// `CollisionRequest::isSatisfied` makes it true once the caller has the
    /// contacts it asked for (`collision_request-inl.h:77-82`). A caller that
    /// wants every pair simply never breaks.
    pub(crate) fn leaf_pairs(
        &self,
        other: &ObbTree,
        rot12: &Matrix,
        t12: &Vector,
        prediction: f64,
        leaf: &mut impl FnMut(u32, u32) -> ControlFlow<()>,
    ) {
        let mut descent = Descent {
            other,
            rot12: *rot12,
            t12: *t12,
            grow: Vector::splat(prediction),
            leaf,
        };
        let root2 = Carried::of(rot12, t12, &other.nodes[0].obb);
        let _ = self.recurse(&mut descent, 0, 0, &root2);
    }

    fn recurse(
        &self,
        d: &mut Descent<'_, impl FnMut(u32, u32) -> ControlFlow<()>>,
        i1: u32,
        i2: u32,
        c2: &Carried,
    ) -> ControlFlow<()> {
        let n1 = &self.nodes[i1 as usize];
        let n2 = &d.other.nodes[i2 as usize];

        // `fcl::overlap(R0, T0, b1, b2)` (`OBB-inl.h:383`), inlined: the
        // second box's frame expressed in the first box's. The half of that
        // change of frame which depends only on the second node arrives in
        // `c2` -- see [`Carried`].
        let into_n1 = n1.obb.axis.transpose();
        let rot = into_n1 * c2.axis;
        let t = into_n1 * (c2.center - n1.obb.center);
        if obb_disjoint(&rot, &t, &(n1.obb.extent + d.grow), &n2.obb.extent) {
            return ControlFlow::Continue(());
        }

        let (l1, l2) = (n1.is_leaf(), n2.is_leaf());
        if l1 && l2 {
            return (d.leaf)(n1.primitive, n2.primitive);
        }
        // `firstOverSecond` (`bvh_collision_traversal_node-inl.h:78`):
        // descend whichever side is bigger, and never descend a leaf.
        if l2 || (!l1 && n1.obb.size() > n2.obb.size()) {
            // First tree descends, so the second node -- and with it `c2` --
            // is the same one both children face.
            let a = n1.first_child;
            self.recurse(d, a, i2, c2)?;
            self.recurse(d, a + 1, i2, c2)
        } else {
            // Second tree descends: each child needs its own carry, and gets
            // it from its own box.
            let a = n2.first_child;
            let (ca, cb) = (
                Carried::of(&d.rot12, &d.t12, &d.other.nodes[a as usize].obb),
                Carried::of(&d.rot12, &d.t12, &d.other.nodes[a as usize + 1].obb),
            );
            self.recurse(d, i1, a, &ca)?;
            self.recurse(d, i1, a + 1, &cb)
        }
    }

    /// The same descent with one tree instead of two: calls `leaf` once per
    /// triangle whose node box `reaches` admits, and drops a whole subtree
    /// the first time it does not.
    ///
    /// This is the shape the mesh-against-one-shape traversal takes -- the
    /// second side is a leaf at every level, so the recursion only ever
    /// descends this tree, and `firstOverSecond`'s bigger-side rule has
    /// nothing to choose between. It is not a second algorithm; it is
    /// [`Self::leaf_pairs`] with the second tree's descent removed.
    ///
    /// `reaches` is handed the node's box already grown by `prediction` on
    /// every axis, for the reason [`Self::leaf_pairs`] grows the first box
    /// there: a point within `prediction` of a box has `|local_k| <=
    /// extent_k + prediction` on every axis, so the grown box contains the
    /// whole `prediction`-neighbourhood of the original. The caller's test is
    /// therefore an intersection question, never a distance one -- which is
    /// what lets it test the box against a *shape* rather than against
    /// another box.
    ///
    /// `leaf` returning [`ControlFlow::Break`] ends the descent, as in
    /// [`Self::leaf_pairs`].
    pub(crate) fn leaves_reaching(
        &self,
        prediction: f64,
        reaches: &mut impl FnMut(&Obb) -> bool,
        leaf: &mut impl FnMut(u32) -> ControlFlow<()>,
    ) {
        let mut descent = ShapeDescent {
            grow: Vector::splat(prediction),
            reaches,
            leaf,
        };
        let _ = self.recurse_shape(&mut descent, 0);
    }

    fn recurse_shape(
        &self,
        d: &mut ShapeDescent<'_, impl FnMut(&Obb) -> bool, impl FnMut(u32) -> ControlFlow<()>>,
        i: u32,
    ) -> ControlFlow<()> {
        let n = &self.nodes[i as usize];
        let grown = Obb {
            extent: n.obb.extent + d.grow,
            ..n.obb
        };
        if !(d.reaches)(&grown) {
            return ControlFlow::Continue(());
        }
        if n.is_leaf() {
            return (d.leaf)(n.primitive);
        }
        self.recurse_shape(d, n.first_child)?;
        self.recurse_shape(d, n.first_child + 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::RngExt as _;
    use rand::SeedableRng as _;

    /// A sphere's triangles, as a mesh with a real hierarchy over it.
    fn sphere(subdivisions: usize) -> (Vec<Vector>, Vec<[u32; 3]>) {
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        let n = subdivisions;
        for i in 0..=n {
            let theta = std::f64::consts::PI * i as f64 / n as f64;
            for j in 0..=n {
                let phi = 2.0 * std::f64::consts::PI * j as f64 / n as f64;
                vertices.push(Vector::new(
                    theta.sin() * phi.cos(),
                    theta.sin() * phi.sin(),
                    theta.cos(),
                ));
            }
        }
        let idx = |i: usize, j: usize| (i * (n + 1) + j) as u32;
        for i in 0..n {
            for j in 0..n {
                triangles.push([idx(i, j), idx(i + 1, j), idx(i, j + 1)]);
                triangles.push([idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1)]);
            }
        }
        (vertices, triangles)
    }

    fn random_quat(rng: &mut rand_chacha::ChaCha8Rng) -> parry3d_f64::math::Rotation {
        let axis = loop {
            let v = Vector::new(
                rng.random_range(-1.0..1.0),
                rng.random_range(-1.0..1.0),
                rng.random_range(-1.0..1.0),
            );
            if v.length_squared() > 1.0e-6 {
                break v.normalize();
            }
        };
        parry3d_f64::math::Rotation::from_axis_angle(axis, rng.random_range(-3.2..3.2))
    }

    fn random_rotation(rng: &mut rand_chacha::ChaCha8Rng) -> Matrix {
        Matrix::from_quat(random_quat(rng))
    }

    /// Every node's box has to contain every triangle under it, or the
    /// descent can drop a pair that touches. Checked at every node, not only
    /// at the root, because a rebuilt child box is a fresh chance to lose a
    /// vertex.
    #[test]
    fn every_node_box_contains_every_triangle_beneath_it() {
        let (vertices, triangles) = sphere(12);
        let tree = ObbTree::build(&vertices, &triangles).expect("non-empty mesh");

        fn walk(
            tree: &ObbTree,
            node: usize,
            vertices: &[Vector],
            triangles: &[[u32; 3]],
            checked: &mut usize,
        ) -> Vec<u32> {
            let n = tree.nodes[node];
            let mine = if n.is_leaf() {
                vec![n.primitive]
            } else {
                let mut v = walk(tree, n.first_child as usize, vertices, triangles, checked);
                v.extend(walk(
                    tree,
                    n.first_child as usize + 1,
                    vertices,
                    triangles,
                    checked,
                ));
                v
            };
            for &t in &mine {
                for v in triangles[t as usize] {
                    let local = n.obb.axis.transpose() * (vertices[v as usize] - n.obb.center);
                    for k in 0..3 {
                        assert!(
                            local[k].abs() <= n.obb.extent[k] + 1.0e-9,
                            "node {node} does not contain triangle {t}"
                        );
                    }
                    *checked += 1;
                }
            }
            mine
        }

        let mut checked = 0;
        let all = walk(&tree, 0, &vertices, &triangles, &mut checked);
        assert_eq!(all.len(), triangles.len(), "a triangle went missing");
        // 8136 on this mesh: 288 triangles x 3 vertices x the ~9.4 ancestors
        // each one has. The guard is against a walk that visited a root and
        // stopped, not a bound on the shape of the tree.
        assert!(checked > 6_000, "only {checked} containments checked");
    }

    /// Every triangle exactly once, which is what makes the leaf pairs a
    /// partition of the mesh rather than a sample of it.
    #[test]
    fn the_build_places_every_triangle_in_exactly_one_leaf() {
        let (vertices, triangles) = sphere(9);
        let tree = ObbTree::build(&vertices, &triangles).expect("non-empty mesh");
        let mut seen = vec![0usize; triangles.len()];
        for n in &tree.nodes {
            if n.is_leaf() {
                seen[n.primitive as usize] += 1;
            }
        }
        assert!(
            seen.iter().all(|&c| c == 1),
            "{} triangles are not in exactly one leaf",
            seen.iter().filter(|&&c| c != 1).count()
        );
    }

    #[test]
    fn an_empty_mesh_has_no_tree() {
        assert!(ObbTree::build(&[], &[]).is_none());
    }

    /// [`obb_disjoint`] is a rejection, so what it owes is that it never
    /// separates two boxes that overlap. Swept against a brute force over the
    /// two boxes' corner points: if any corner of one box is inside the
    /// other, they overlap, and the test must not report disjoint.
    #[test]
    fn obb_disjoint_never_separates_two_boxes_that_share_a_corner() {
        let mut rng = rand_chacha::ChaCha8Rng::seed_from_u64(20260809);
        let mut overlapping = 0;

        for _ in 0..20_000 {
            let ea = Vector::new(
                rng.random_range(0.05..0.6),
                rng.random_range(0.05..0.6),
                rng.random_range(0.05..0.6),
            );
            let eb = Vector::new(
                rng.random_range(0.05..0.6),
                rng.random_range(0.05..0.6),
                rng.random_range(0.05..0.6),
            );
            let rot = random_rotation(&mut rng);
            let t = Vector::new(
                rng.random_range(-1.2..1.2),
                rng.random_range(-1.2..1.2),
                rng.random_range(-1.2..1.2),
            );

            // A corner of the second box, in the first box's frame.
            let inside = |signs: [f64; 3]| {
                let corner =
                    rot * Vector::new(signs[0] * eb.x, signs[1] * eb.y, signs[2] * eb.z) + t;
                (0..3).all(|k| corner[k].abs() <= ea[k])
            };
            let shares_corner = [-1.0, 1.0].iter().any(|&x| {
                [-1.0, 1.0]
                    .iter()
                    .any(|&y| [-1.0, 1.0].iter().any(|&z| inside([x, y, z])))
            });
            if shares_corner {
                overlapping += 1;
                assert!(
                    !obb_disjoint(&rot, &t, &ea, &eb),
                    "separated two boxes sharing a corner"
                );
            }
        }

        // Otherwise the assertion holds on a sweep where no pair overlapped.
        assert!(overlapping > 500, "only {overlapping} pairs overlapped");
    }

    /// The descent's whole contract: it may emit pairs that turn out to be
    /// far apart, but it may never *drop* a pair that is within `prediction`.
    /// Checked against the exhaustive set of triangle pairs, whose exact
    /// separation `parry` computes.
    ///
    /// This is what would break silently if a node's box did not contain its
    /// triangles, if [`obb_disjoint`] were not conservative, or if `grow`
    /// were applied on the wrong side -- and every one of those would show up
    /// as a missed collision in the planner, not as a crash.
    #[test]
    fn the_descent_emits_every_triangle_pair_within_the_prediction() {
        use parry3d_f64::query;
        use parry3d_f64::shape::Triangle;

        let (v1, t1) = sphere(7);
        let (v2, t2) = sphere(6);
        let tree1 = ObbTree::build(&v1, &t1).expect("non-empty mesh");
        let tree2 = ObbTree::build(&v2, &t2).expect("non-empty mesh");

        let mut rng = rand_chacha::ChaCha8Rng::seed_from_u64(20260809);
        let mut close_total = 0;
        let mut emitted_total = 0;

        for _ in 0..40 {
            let quat12 = random_quat(&mut rng);
            let rot12 = Matrix::from_quat(quat12);
            let t12 = Vector::new(
                rng.random_range(-2.4..2.4),
                rng.random_range(-2.4..2.4),
                rng.random_range(-2.4..2.4),
            );
            for prediction in [0.0, 0.05] {
                let mut emitted = std::collections::HashSet::new();
                tree1.leaf_pairs(&tree2, &rot12, &t12, prediction, &mut |a, b| {
                    emitted.insert((a, b));
                    ControlFlow::Continue(())
                });
                emitted_total += emitted.len();

                let tri = |v: &[Vector], t: [u32; 3]| {
                    Triangle::new(v[t[0] as usize], v[t[1] as usize], v[t[2] as usize])
                };
                let pose12 = parry3d_f64::math::Pose {
                    rotation: quat12,
                    translation: t12,
                };
                for (a, &ta) in t1.iter().enumerate() {
                    let tri_a = tri(&v1, ta);
                    for (b, &tb) in t2.iter().enumerate() {
                        let tri_b = tri(&v2, tb);
                        let d = query::distance(
                            &parry3d_f64::math::Pose::IDENTITY,
                            &tri_a,
                            &pose12,
                            &tri_b,
                        )
                        .expect("triangle-triangle distance is supported");
                        // Strictly inside the margin, so a pair sitting
                        // exactly on the boundary -- where the box test and
                        // the exact test may round opposite ways -- does not
                        // decide the assertion.
                        if d < prediction * 0.99 || (prediction == 0.0 && d == 0.0) {
                            close_total += 1;
                            assert!(
                                emitted.contains(&(a as u32, b as u32)),
                                "dropped triangle pair ({a}, {b}) at separation {d}"
                            );
                        }
                    }
                }
            }
        }

        // Otherwise the assertion above held over a sweep with nothing close
        // in it, and over one that emitted everything regardless.
        assert!(close_total > 200, "only {close_total} close pairs swept");
        let all_pairs = 40 * 2 * t1.len() * t2.len();
        assert!(
            emitted_total * 4 < all_pairs,
            "the descent emitted {emitted_total} of {all_pairs} pairs -- it is not pruning"
        );
    }

    /// Two boxes far apart on every axis must be rejected, or the test proves
    /// nothing and the descent below it would visit the whole mesh.
    #[test]
    fn obb_disjoint_separates_two_boxes_that_are_far_apart() {
        let e = Vector::splat(0.5);
        assert!(obb_disjoint(
            &Matrix::IDENTITY,
            &Vector::new(3.0, 0.0, 0.0),
            &e,
            &e
        ));
        assert!(!obb_disjoint(
            &Matrix::IDENTITY,
            &Vector::new(0.5, 0.0, 0.0),
            &e,
            &e
        ));
    }
}
