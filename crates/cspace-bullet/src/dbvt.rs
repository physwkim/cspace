// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2003-2007 Erwin Coumans  https://bulletphysics.org
// btDbvt implementation by Nathanael Presson
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/BroadphaseCollision/btDbvt.h
//   bullet3/src/BulletCollision/BroadphaseCollision/btDbvt.cpp

//! `btDbvt` -- the dynamic AABB tree a `btCompoundShape` culls its children
//! with, and the reason child order is not simply `0..n`.
//!
//! # Why this is here at all
//!
//! `btCompoundShape`'s constructor allocates one of these whenever
//! `enableDynamicAabbTree` is set, and MoveIt sets it at every one of its four
//! construction sites -- `BULLET_COMPOUND_USE_DYNAMIC_AABB` is `true`
//! (`bullet_utils.hpp:56`). With a tree present,
//! `btCompoundCollisionAlgorithm::processCollision` takes the
//! `collideTVNoStackAlloc` branch and visits children in *tree* order, which
//! depends on the insertion history rather than on the child index. That order
//! is observable: MoveIt's callback sets `pair_done` and `done`
//! (`bullet_utils.hpp:571-630`), so which child reports first decides which
//! contact survives.
//!
//! # Pointers to indices
//!
//! Upstream links nodes with raw pointers and recycles exactly one freed node
//! through `m_free`. This port stores nodes in a [`Vec`] and links them with
//! `Option<usize>`; [`Dbvt`]'s `free` field holds the same single recycled slot, so a
//! sequence of inserts and removes reuses storage at the same points upstream
//! does. Node identity is never an output -- only the leaf payloads and the
//! order they are visited in -- but pointer equality *is* an input to
//! `collideTT`'s `p.a == p.b` test, and index equality is the same relation
//! over live nodes.
//!
//! Upstream's `btDbvtNode` is a union: a leaf's `childs[0]` slot holds the
//! payload, and `childs[1] == 0` is what makes it a leaf. Here the payload is
//! its own field and [`Node::child`] is `[None; 2]` on a leaf, which is the
//! same predicate written without the aliasing.
//!
//! The free slot is reached only through [`Dbvt::remove`], and the continuous
//! path never removes: it builds each compound with `addChildShape` and then
//! only ever calls `updateChildTransform`
//! (`bullet_cast_bvh_manager.cpp:102`, `:115`), which goes through
//! `remove_leaf` without freeing the leaf itself. `remove` is ported because
//! `btCompoundShape::removeChildShapeByIndex` calls it and because it is the
//! only caller of `delete_node`, so leaving it out would leave that half of
//! the arena unexercised rather than unwritten -- but on the CCD path the free
//! slot stays `None` and `create_node` always appends.
//!
//! # Not ported
//!
//! `optimizeBottomUp`, `optimizeTopDown`, `optimizeIncremental`, `write`,
//! `clone`, the ray tests, `collideKDOP`, `collideOCL`, `collideTU`, and the
//! `sStkNPS`-based traversals. `btCompoundShape` calls only `insert`, `update`
//! and `remove`; `btCompoundCollisionAlgorithm` calls only
//! `collideTVNoStackAlloc`; and `btCompoundCompoundCollisionAlgorithm` uses
//! its own `MycollideTT`, a file-static in that translation unit rather than a
//! `btDbvt` member. Nothing on the continuous path reaches the rest.
//!
//! `m_lkhd` is likewise absent: it is written only by
//! `btDbvtBroadphase::setAabbForceUpdate`, and a compound's tree is never
//! owned by a broadphase, so it holds its constructed `-1` for the whole life
//! of every tree this crate builds. `update` therefore reinserts from the
//! root, which is what `m_lkhd < 0` selects.
//!
//! Unqualified citations in this file are lines in
//! `bullet_cast_bvh_manager.cpp`; a citation of any other file names
//! that file.

use crate::linear_math::{Scalar, Vec3};

/// `btDbvtAabbMm` (`btDbvt.h:131-172`), the tree's bounding volume.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DbvtVolume {
    /// `mi`.
    pub mi: Vec3,
    /// `mx`.
    pub mx: Vec3,
}

impl DbvtVolume {
    /// `btDbvtAabbMm::FromMM` (`btDbvt.h:479-485`).
    #[must_use]
    pub const fn from_mm(mi: Vec3, mx: Vec3) -> Self {
        Self { mi, mx }
    }

    /// `Center` (`btDbvt.h:134`).
    #[must_use]
    pub fn center(&self) -> Vec3 {
        (self.mi + self.mx) / 2.0
    }

    /// `Lengths` (`btDbvt.h:135`).
    #[must_use]
    pub fn lengths(&self) -> Vec3 {
        self.mx - self.mi
    }

    /// `Contain` (`btDbvt.h:538-546`).
    #[must_use]
    pub fn contain(&self, a: &Self) -> bool {
        self.mi.x <= a.mi.x
            && self.mi.y <= a.mi.y
            && self.mi.z <= a.mi.z
            && self.mx.x >= a.mx.x
            && self.mx.y >= a.mx.y
            && self.mx.z >= a.mx.z
    }

    /// `Expand` (`btDbvt.h:514-518`).
    pub fn expand(&mut self, e: Vec3) {
        self.mi -= e;
        self.mx += e;
    }
}

/// `Intersect(a, b)` (`btDbvt.h:621-641`), the generic arm.
///
/// The SSE arm above it is compiled only under `BT_USE_SSE`, which
/// `btScalar.h:216-244` leaves undefined on non-Apple Linux -- the
/// configuration `tools/bullet-epa-reference/build.sh` reproduces and the one
/// this crate is a port of.
#[must_use]
pub fn intersect(a: &DbvtVolume, b: &DbvtVolume) -> bool {
    a.mi.x <= b.mx.x
        && a.mx.x >= b.mi.x
        && a.mi.y <= b.mx.y
        && a.mx.y >= b.mi.y
        && a.mi.z <= b.mx.z
        && a.mx.z >= b.mi.z
}

/// `Proximity(a, b)` (`btDbvt.h:655-660`) -- the L1 distance between the two
/// volumes' *doubled* centres, not their centres.
#[must_use]
pub fn proximity(a: &DbvtVolume, b: &DbvtVolume) -> Scalar {
    let d = (a.mi + a.mx) - (b.mi + b.mx);
    d.x.abs() + d.y.abs() + d.z.abs()
}

/// `Select(o, a, b)` (`btDbvt.h:663-741`), the generic arm -- which of `a`
/// and `b` the new volume `o` should descend into.
///
/// This is the whole of the tree's shape policy: a tie goes to `b`, because
/// the comparison is strict.
#[must_use]
pub fn select(o: &DbvtVolume, a: &DbvtVolume, b: &DbvtVolume) -> usize {
    usize::from(proximity(o, a) >= proximity(o, b))
}

/// `Merge(a, b, r)` (`btDbvt.h:744-765`), the generic arm.
#[must_use]
pub fn merge(a: &DbvtVolume, b: &DbvtVolume) -> DbvtVolume {
    let mut r = DbvtVolume::from_mm(Vec3::zero(), Vec3::zero());
    for i in 0..3 {
        r.mi[i] = if a.mi[i] < b.mi[i] { a.mi[i] } else { b.mi[i] };
        r.mx[i] = if a.mx[i] > b.mx[i] { a.mx[i] } else { b.mx[i] };
    }
    r
}

/// `NotEqual(a, b)` (`btDbvt.h:768-776`) -- six float comparisons, so a
/// volume that differs only in a signed zero counts as equal, exactly as
/// upstream's does.
#[must_use]
pub fn not_equal(a: &DbvtVolume, b: &DbvtVolume) -> bool {
    a.mi.x != b.mi.x
        || a.mi.y != b.mi.y
        || a.mi.z != b.mi.z
        || a.mx.x != b.mx.x
        || a.mx.y != b.mx.y
        || a.mx.z != b.mx.z
}

/// `btDbvtNode` (`btDbvt.h:176-188`).
#[derive(Clone, Copy, Debug)]
pub struct Node {
    /// `volume`.
    pub volume: DbvtVolume,
    /// `parent`.
    pub parent: Option<usize>,
    /// `childs`. Both `None` marks a leaf; see the module docs on the union.
    pub child: [Option<usize>; 2],
    /// `dataAsInt` -- the child index a `btCompoundShape` stores here.
    pub data: i32,
}

impl Node {
    /// `isleaf` (`btDbvt.h:184`).
    #[must_use]
    pub fn is_leaf(&self) -> bool {
        self.child[1].is_none()
    }

    /// `isinternal` (`btDbvt.h:185`).
    #[must_use]
    pub fn is_internal(&self) -> bool {
        !self.is_leaf()
    }
}

/// `btDbvt` (`btDbvt.h:228-...`), reduced to the operations a compound shape
/// and its two collision algorithms reach.
#[derive(Clone, Debug, Default)]
pub struct Dbvt {
    /// The node storage. Upstream allocates each node separately; an index
    /// into this is what replaces the pointer. Slots left behind by
    /// [`Dbvt::delete_node`] are not compacted, so an index stays valid for
    /// the life of the tree.
    nodes: Vec<Node>,
    /// `m_root`.
    pub root: Option<usize>,
    /// `m_free` -- the single node `deletenode` holds back for the next
    /// `createnode` to reuse (`btDbvt.cpp:71-76`). Everything it displaces is
    /// released; here it is simply left unreferenced in [`Dbvt::nodes`].
    free: Option<usize>,
    /// `m_leaves`.
    pub leaves: usize,
}

impl Dbvt {
    /// `btDbvt::btDbvt` (`btDbvt.cpp:461-468`).
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The node at `index`.
    ///
    /// # Panics
    ///
    /// If `index` is not a slot this tree allocated.
    #[must_use]
    pub fn node(&self, index: usize) -> &Node {
        &self.nodes[index]
    }

    /// `indexof(node)` (`btDbvt.cpp:31-34`) -- which of its parent's two
    /// slots a node sits in.
    fn index_of(&self, node: usize) -> usize {
        let parent = self.nodes[node].parent.expect("indexof needs a parent");
        usize::from(self.nodes[parent].child[1] == Some(node))
    }

    /// `createnode(pdbvt, parent, volume, data)` (`btDbvt.cpp:90-122`).
    ///
    /// The two-volume overload is spelt at its call site as a [`merge`],
    /// which is all it adds.
    fn create_node(&mut self, parent: Option<usize>, volume: DbvtVolume, data: i32) -> usize {
        let node = Node {
            volume,
            parent,
            // `node->childs[1] = 0` with `data` written into the union's other
            // half: a fresh node is always a leaf.
            child: [None, None],
            data,
        };
        if let Some(index) = self.free.take() {
            self.nodes[index] = node;
            index
        } else {
            self.nodes.push(node);
            self.nodes.len() - 1
        }
    }

    /// `deletenode(pdbvt, node)` (`btDbvt.cpp:71-76`).
    fn delete_node(&mut self, node: usize) {
        self.free = Some(node);
    }

    /// `insertleaf(pdbvt, root, leaf)` (`btDbvt.cpp:137-183`).
    fn insert_leaf(&mut self, root: Option<usize>, leaf: usize) {
        // `if (!pdbvt->m_root)` -- the test is on the tree's root, not on the
        // `root` argument. The two are `None` together: `insert` passes
        // `m_root`, and `removeleaf` returns `0` only on the path that just
        // cleared it.
        let (Some(mut root), true) = (root, self.root.is_some()) else {
            self.root = Some(leaf);
            self.nodes[leaf].parent = None;
            return;
        };

        if self.nodes[root].is_internal() {
            loop {
                let c0 = self.nodes[root].child[0].expect("internal node has two children");
                let c1 = self.nodes[root].child[1].expect("internal node has two children");
                let pick = select(
                    &self.nodes[leaf].volume,
                    &self.nodes[c0].volume,
                    &self.nodes[c1].volume,
                );
                root = if pick == 0 { c0 } else { c1 };
                if self.nodes[root].is_leaf() {
                    break;
                }
            }
        }

        let prev = self.nodes[root].parent;
        let merged = merge(&self.nodes[leaf].volume, &self.nodes[root].volume);
        let mut node = self.create_node(prev, merged, 0);

        if let Some(mut prev) = prev {
            let slot = self.index_of(root);
            self.nodes[prev].child[slot] = Some(node);
            self.nodes[node].child[0] = Some(root);
            self.nodes[root].parent = Some(node);
            self.nodes[node].child[1] = Some(leaf);
            self.nodes[leaf].parent = Some(node);
            loop {
                if self.nodes[prev].volume.contain(&self.nodes[node].volume) {
                    break;
                }
                let c0 = self.nodes[prev].child[0].expect("internal node has two children");
                let c1 = self.nodes[prev].child[1].expect("internal node has two children");
                self.nodes[prev].volume = merge(&self.nodes[c0].volume, &self.nodes[c1].volume);
                node = prev;
                match self.nodes[node].parent {
                    Some(p) => prev = p,
                    None => break,
                }
            }
        } else {
            self.nodes[node].child[0] = Some(root);
            self.nodes[root].parent = Some(node);
            self.nodes[node].child[1] = Some(leaf);
            self.nodes[leaf].parent = Some(node);
            self.root = Some(node);
        }
    }

    /// `removeleaf(pdbvt, leaf)` (`btDbvt.cpp:188-224`).
    ///
    /// Returns the node the caller should reinsert from, which is `None` only
    /// when the removed leaf was the whole tree.
    fn remove_leaf(&mut self, leaf: usize) -> Option<usize> {
        if self.root == Some(leaf) {
            self.root = None;
            return None;
        }

        let parent = self.nodes[leaf]
            .parent
            .expect("a non-root leaf has a parent");
        let prev = self.nodes[parent].parent;
        let sibling =
            self.nodes[parent].child[1 - self.index_of(leaf)].expect("a parent has two children");

        let Some(mut prev) = prev else {
            self.root = Some(sibling);
            self.nodes[sibling].parent = None;
            self.delete_node(parent);
            return self.root;
        };

        let slot = self.index_of(parent);
        self.nodes[prev].child[slot] = Some(sibling);
        self.nodes[sibling].parent = Some(prev);
        self.delete_node(parent);

        loop {
            let pb = self.nodes[prev].volume;
            let c0 = self.nodes[prev].child[0].expect("internal node has two children");
            let c1 = self.nodes[prev].child[1].expect("internal node has two children");
            self.nodes[prev].volume = merge(&self.nodes[c0].volume, &self.nodes[c1].volume);
            if !not_equal(&pb, &self.nodes[prev].volume) {
                return Some(prev);
            }
            match self.nodes[prev].parent {
                Some(p) => prev = p,
                // The `while (prev)` fell out, so `return (prev ? prev :
                // pdbvt->m_root)` returns the root.
                None => return self.root,
            }
        }
    }

    /// `btDbvt::insert(volume, data)` (`btDbvt.cpp:531-537`).
    pub fn insert(&mut self, volume: DbvtVolume, data: i32) -> usize {
        let leaf = self.create_node(None, volume, data);
        self.insert_leaf(self.root, leaf);
        self.leaves += 1;
        leaf
    }

    /// `btDbvt::update(leaf, volume)` (`btDbvt.cpp:563-578`), with `m_lkhd`
    /// at its constructed `-1`; see the module docs.
    pub fn update(&mut self, leaf: usize, volume: DbvtVolume) {
        // `if (root) { ... root = m_root; }` -- the returned node is used
        // only as a yes/no, because `m_lkhd` is `-1` and the `else` arm
        // reinserts from the root.
        let root = self.remove_leaf(leaf).and(self.root);
        self.nodes[leaf].volume = volume;
        self.insert_leaf(root, leaf);
    }

    /// `btDbvt::remove(leaf)` (`btDbvt.cpp:611-616`).
    pub fn remove(&mut self, leaf: usize) {
        self.remove_leaf(leaf);
        self.delete_node(leaf);
        self.leaves -= 1;
    }

    /// `btDbvt::collideTVNoStackAlloc(root, volume, stack, policy)`
    /// (`btDbvt.h:1187-1218`).
    ///
    /// The caller's `stack` is upstream's reused scratch buffer; passing it in
    /// is the only difference from `collideTV`, and it is what
    /// `btCompoundCollisionAlgorithm` calls. `policy` is handed each visited
    /// leaf's index.
    ///
    /// The stack is a LIFO seeded with the root, and each internal node pushes
    /// `childs[0]` then `childs[1]`, so leaves come out right subtree first.
    /// That is the visit order MoveIt's `pair_done` can stop early in.
    pub fn collide_tv_no_stack_alloc(
        &self,
        root: Option<usize>,
        volume: &DbvtVolume,
        stack: &mut Vec<usize>,
        policy: &mut impl FnMut(&Self, usize),
    ) {
        let Some(root) = root else { return };
        stack.clear();
        stack.push(root);
        while let Some(n) = stack.pop() {
            if intersect(&self.nodes[n].volume, volume) {
                if self.nodes[n].is_internal() {
                    stack.push(self.nodes[n].child[0].expect("internal node has two children"));
                    stack.push(self.nodes[n].child[1].expect("internal node has two children"));
                } else {
                    policy(self, n);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `cube(x, y, z)` from `probe.cpp` -- a unit cube centred on the point.
    fn cube(x: Scalar, y: Scalar, z: Scalar) -> DbvtVolume {
        DbvtVolume::from_mm(
            Vec3::new(x - 0.5, y - 0.5, z - 0.5),
            Vec3::new(x + 0.5, y + 0.5, z + 0.5),
        )
    }

    /// The `all` query volume `probe.cpp` uses for the unculled rows.
    fn everything() -> DbvtVolume {
        DbvtVolume::from_mm(Vec3::new(-1e6, -1e6, -1e6), Vec3::new(1e6, 1e6, 1e6))
    }

    /// The `dbvt_*` rows of `tools/bullet-epa-reference/build.sh`'s stdout,
    /// verbatim: the real `btDbvt` from bullet3 @ `7dee3436`, built by
    /// `insert` in index order and walked by `collideTVNoStackAlloc`.
    ///
    /// The visit order is the whole output. `dbvt_line4` comes back
    /// `3 2 1 0`, which is what makes this module necessary at all: a
    /// compound that iterated `0..n` would agree with Bullet on *which*
    /// children overlap and disagree on which one reports first, and
    /// MoveIt's `pair_done` stops at the first.
    ///
    /// Fields: `name|visited|leaves|data...`.
    const BULLET_REFERENCE: &str = "\
dbvt_line4|4|4|3|2|1|0
dbvt_line8|8|8|7|6|5|4|3|2|1|0
dbvt_cull|1|4|1
dbvt_cull_far|1|4|3
dbvt_touch_lo|1|4|0
dbvt_touch_hi|1|4|3
dbvt_update|4|4|0|3|2|1
dbvt_remove|3|3|3|2|0
dbvt_grid|9|9|8|5|2|7|4|1|6|3|0
dbvt_cube8|8|8|7|3|5|1|6|2|4|0
";

    /// `probe.cpp`'s `line4`.
    fn line4() -> Vec<Vec3> {
        (0..4)
            .map(|i| Vec3::new(f64::from(i) as Scalar * 2.0, 0.0, 0.0))
            .collect()
    }

    /// `probe.cpp`'s `line8`.
    fn line8() -> Vec<Vec3> {
        (0..8)
            .map(|i| Vec3::new(f64::from(i) as Scalar * 2.0, 0.0, 0.0))
            .collect()
    }

    /// `probe.cpp`'s `grid`, in its own `gx * 3 + gy` order.
    fn grid() -> Vec<Vec3> {
        let mut g = vec![Vec3::zero(); 9];
        for gx in 0..3 {
            for gy in 0..3 {
                g[gx * 3 + gy] = Vec3::new(
                    f64::from(gx as i32) as Scalar * 2.0,
                    f64::from(gy as i32) as Scalar * 2.0,
                    0.0,
                );
            }
        }
        g
    }

    /// `probe.cpp`'s `cube8`, in its own `cx * 4 + cy * 2 + cz` order.
    ///
    /// Every other row is planar in z, where [`proximity`]'s `btFabs` on that
    /// component cannot change a `Select`; here all three axes differ at once.
    fn cube8() -> Vec<Vec3> {
        let mut c = vec![Vec3::zero(); 8];
        for cx in 0..2usize {
            for cy in 0..2usize {
                for cz in 0..2usize {
                    c[cx * 4 + cy * 2 + cz] = Vec3::new(
                        f64::from(cx as i32) as Scalar * 3.0,
                        f64::from(cy as i32) as Scalar * 3.0,
                        f64::from(cz as i32) as Scalar * 3.0,
                    );
                }
            }
        }
        c
    }

    /// One parsed row of [`BULLET_REFERENCE`]: `(visited, leaves, data)`.
    ///
    /// The row's own `visited` field is checked against the number of data
    /// fields that follow it, so a row that lost or gained an entry fails
    /// here as a row-shape error rather than shortening what is compared.
    fn reference(name: &str) -> (usize, usize, Vec<i32>) {
        let line = BULLET_REFERENCE
            .lines()
            .find(|l| l.split('|').next() == Some(name))
            .unwrap_or_else(|| panic!("{name}: no such row in BULLET_REFERENCE"));
        let f: Vec<&str> = line.split('|').collect();
        let visited: usize = f[1]
            .parse()
            .unwrap_or_else(|e| panic!("{name}: field 1 ({:?}): {e}", f[1]));
        let leaves: usize = f[2]
            .parse()
            .unwrap_or_else(|e| panic!("{name}: field 2 ({:?}): {e}", f[2]));
        assert_eq!(
            f.len(),
            3 + visited,
            "{name}: {} fields for a visited count of {visited}",
            f.len()
        );
        let data = f[3..]
            .iter()
            .map(|v| {
                v.parse()
                    .unwrap_or_else(|e| panic!("{name}: data field ({v:?}): {e}"))
            })
            .collect();
        (visited, leaves, data)
    }

    /// The tree one `probe.cpp` row builds.
    ///
    /// `op` matches `probe.cpp`'s: `0` none, `1` update child 0 to
    /// `cube(100, 0, 0)`, `2` remove child 1 -- the two edits
    /// `btCompoundShape` performs on a tree it has already built.
    fn build(centres: &[Vec3], op: u8) -> Dbvt {
        let mut tree = Dbvt::new();
        let leaves: Vec<usize> = centres
            .iter()
            .enumerate()
            .map(|(i, c)| {
                tree.insert(
                    cube(c.x, c.y, c.z),
                    i32::try_from(i).expect("fewer than i32::MAX children"),
                )
            })
            .collect();
        match op {
            1 => tree.update(leaves[0], cube(100.0, 0.0, 0.0)),
            2 => tree.remove(leaves[1]),
            _ => {}
        }
        tree
    }

    /// The `(centres, query, op)` of every [`BULLET_REFERENCE`] row.
    fn cases() -> Vec<(&'static str, Vec<Vec3>, DbvtVolume, u8)> {
        vec![
            ("dbvt_line4", line4(), everything(), 0),
            ("dbvt_line8", line8(), everything(), 0),
            ("dbvt_cull", line4(), cube(2.0, 0.0, 0.0), 0),
            ("dbvt_cull_far", line4(), cube(6.0, 0.0, 0.0), 0),
            ("dbvt_touch_lo", line4(), cube(-1.0, -1.0, -1.0), 0),
            ("dbvt_touch_hi", line4(), cube(7.0, 1.0, 1.0), 0),
            ("dbvt_update", line4(), everything(), 1),
            ("dbvt_remove", line4(), everything(), 2),
            ("dbvt_grid", grid(), everything(), 0),
            ("dbvt_cube8", cube8(), everything(), 0),
        ]
    }

    /// Every `collideTVNoStackAlloc` row, against the port.
    #[test]
    fn bullet_reference_leaf_order() {
        let mut bad = Vec::new();

        for (name, centres, query, op) in cases() {
            let tree = build(&centres, op);

            let mut seen = Vec::new();
            let mut stack = Vec::new();
            tree.collide_tv_no_stack_alloc(tree.root, &query, &mut stack, &mut |t, n| {
                seen.push(t.node(n).data);
            });

            let (want_visited, want_leaves, want) = reference(name);

            if seen.len() != want_visited || seen != want {
                bad.push(format!("{name}.order: port {seen:?}, bullet {want:?}"));
            }
            if tree.leaves != want_leaves {
                bad.push(format!(
                    "{name}.leaves: port {}, bullet {want_leaves}",
                    tree.leaves
                ));
            }
        }

        assert!(
            bad.is_empty(),
            "{} deviations:\n{}",
            bad.len(),
            bad.join("\n")
        );
    }

    /// An internal node's volume is the merge of its two children's, exactly
    /// -- which is what makes `Contain`'s early break an optimisation.
    ///
    /// `Contain` is called from one place, `insert_leaf`'s ascent, to stop
    /// climbing once an ancestor already reaches the child it just gained.
    /// Under this invariant the assignment that break skips would write back
    /// the value already stored, so a `Contain` that answers `false` too
    /// often costs a recomputation and changes nothing; only one that answers
    /// `true` too often leaves an ancestor short, and `dbvt_cull_far` and
    /// `dbvt_touch_hi` fail when it does. Asserting the invariant is what
    /// turns that from an argument into a check.
    #[test]
    fn every_internal_volume_is_the_merge_of_its_children() {
        for (name, centres, _, op) in cases() {
            let tree = build(&centres, op);
            // Descend from the root: a node on the free list keeps its old
            // `child[0]` as the next-free link, so only reachability says
            // which nodes are live.
            let mut walk = tree.root.into_iter().collect::<Vec<_>>();
            while let Some(index) = walk.pop() {
                let node = tree.node(index);
                let (Some(a), Some(b)) = (node.child[0], node.child[1]) else {
                    continue;
                };
                let want = merge(&tree.node(a).volume, &tree.node(b).volume);
                assert!(
                    !not_equal(&node.volume, &want),
                    "{name}: node {index} holds {:?}, its children merge to {want:?}",
                    node.volume
                );
                walk.push(a);
                walk.push(b);
            }
        }
    }

    /// Removing the last leaf empties the tree rather than leaving a root
    /// whose children are gone. No probe row covers it: `removeChildShape`
    /// down to zero children is reachable only through
    /// `BulletBVHManager::removeCollisionObject`, and the tree there is
    /// discarded with the object.
    #[test]
    fn removing_the_only_leaf_empties_the_tree() {
        let mut tree = Dbvt::new();
        let leaf = tree.insert(cube(0.0, 0.0, 0.0), 0);
        tree.remove(leaf);
        assert_eq!(tree.root, None);
        assert_eq!(tree.leaves, 0);

        let mut seen = Vec::new();
        let mut stack = Vec::new();
        tree.collide_tv_no_stack_alloc(tree.root, &everything(), &mut stack, &mut |t, n| {
            seen.push(t.node(n).data);
        });
        assert!(seen.is_empty(), "{seen:?}");
    }

    /// A tie in `Proximity` goes to child 1: `Select` reads
    /// `Proximity(o, a) < Proximity(o, b) ? 0 : 1`, so equal proximities fall
    /// to the `1` arm. `dbvt_grid` is the row that reaches this, but it
    /// cannot say *why* it took the branch it took.
    #[test]
    fn select_breaks_a_tie_towards_the_second_child() {
        let o = cube(0.0, 0.0, 0.0);
        let a = cube(1.0, 0.0, 0.0);
        let b = cube(-1.0, 0.0, 0.0);
        assert_eq!(proximity(&o, &a), proximity(&o, &b));
        assert_eq!(select(&o, &a, &b), 1);
    }
}
