// Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from octomap 1.9.7 (see key.rs's provenance comment for how the
// version was matched):
//   include/octomap/OcTreeIterator.hxx (`iterator_base`, `leaf_iterator`,
//     `leaf_bbx_iterator`, `tree_iterator`)
//
// `tree_iterator` (every node, inner and leaf) is ported as `TreeNodes` --
// round 13's item 1. Its upstream consumer is
// `collision_distance_field/src/collision_distance_field_types.cpp:355`'s
// `PosedBodyPointDecomposition(const shared_ptr<const octomap::OcTree>&)`
// constructor, which walks `begin_tree()`/`end_tree()` directly (occupied
// leaves become collision points, occupied inner nodes are skipped since
// they are represented at finer depth by their own children); that
// constructor itself belongs to `cspace-distance-field`, not this crate.

use nalgebra::Point3;

use crate::octomap::key::{KeyType, OcTreeKey, compute_child_key, compute_index_key};
use crate::octomap::node::Node;
use crate::octomap::tree::OcTree;

struct StackElem<'a> {
    node: &'a Node,
    key: OcTreeKey,
    depth: u32,
}

/// Pushes `node`'s existing children onto `stack` in reverse index order (so
/// child 0, if present, ends up on top), matching upstream
/// `iterator_base::singleIncrement`'s descent step. `filter` additionally
/// rejects a child before pushing it -- `leaf_bbx_iterator` uses this for its
/// bounding-box overlap test; plain [`Leaves`] always accepts.
fn push_children<'a>(
    stack: &mut Vec<StackElem<'a>>,
    node: &'a Node,
    key: OcTreeKey,
    depth: u32,
    mut filter: impl FnMut(OcTreeKey, KeyType) -> bool,
) {
    let child_depth = depth + 1;
    let center_offset = (OcTree::TREE_MAX_VAL >> child_depth) as KeyType;
    for i in (0u8..8).rev() {
        if let Some(child) = node.child(i as usize) {
            let child_key = compute_child_key(i, center_offset, key);
            if filter(child_key, center_offset) {
                stack.push(StackElem {
                    node: child,
                    key: child_key,
                    depth: child_depth,
                });
            }
        }
    }
}

/// Iterator over every leaf in an [`OcTree`] (pre-order: a node's child 0
/// subtree before child 1's, and so on). A "leaf" is any childless node,
/// regardless of depth -- a subtree collapsed by [`OcTree::prune`] is a
/// single leaf at a coarser depth, exactly as upstream's iterator reports it.
/// Upstream `leaf_iterator`.
pub struct Leaves<'a> {
    tree: &'a OcTree,
    stack: Vec<StackElem<'a>>,
}

impl<'a> Leaves<'a> {
    pub(crate) fn new(tree: &'a OcTree) -> Self {
        let mut stack = Vec::new();
        if let Some(root) = tree.root() {
            stack.push(StackElem {
                node: root,
                key: OcTree::root_key(),
                depth: 0,
            });
        }
        Self { tree, stack }
    }
}

impl<'a> Iterator for Leaves<'a> {
    type Item = Leaf<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(elem) = self.stack.pop() {
            if elem.node.has_children() && elem.depth < OcTree::TREE_DEPTH {
                push_children(&mut self.stack, elem.node, elem.key, elem.depth, |_, _| {
                    true
                });
                continue;
            }
            return Some(Leaf {
                tree: self.tree,
                key: elem.key,
                depth: elem.depth,
                log_odds: elem.node.log_odds,
            });
        }
        None
    }
}

/// Iterator over every leaf whose voxel overlaps an axis-aligned bounding
/// box (inclusive of `min`/`max`). Upstream `leaf_bbx_iterator`, using its
/// key-space (not float-coordinate) constructor to avoid a second rounding
/// step: `min`/`max` are converted to keys once, up front.
///
/// Per upstream's own doc comment: due to rounding and discretization, a
/// yielded node's float-coordinate center can appear just outside `[min,
/// max]`, but the node's full voxel volume always includes some part of the
/// query box.
pub struct LeavesInBbx<'a> {
    tree: &'a OcTree,
    stack: Vec<StackElem<'a>>,
    min_key: OcTreeKey,
    max_key: OcTreeKey,
}

impl<'a> LeavesInBbx<'a> {
    /// `None` if `min` or `max` is outside the tree's representable
    /// coordinate range (matching upstream's `coordToKeyChecked` failure
    /// case, which upstream handles by producing an immediately-empty
    /// iterator; this port surfaces that as `None` instead since there is no
    /// sentinel "empty" `OcTree` reference to hand back).
    pub(crate) fn new(tree: &'a OcTree, min: Point3<f64>, max: Point3<f64>) -> Option<Self> {
        let min_key = tree.coord_to_key_checked(min)?;
        let max_key = tree.coord_to_key_checked(max)?;
        let mut stack = Vec::new();
        if let Some(root) = tree.root() {
            stack.push(StackElem {
                node: root,
                key: OcTree::root_key(),
                depth: 0,
            });
        }
        Some(Self {
            tree,
            stack,
            min_key,
            max_key,
        })
    }
}

impl<'a> Iterator for LeavesInBbx<'a> {
    type Item = Leaf<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(elem) = self.stack.pop() {
            if elem.node.has_children() && elem.depth < OcTree::TREE_DEPTH {
                let (min_key, max_key) = (self.min_key, self.max_key);
                push_children(&mut self.stack, elem.node, elem.key, elem.depth, |k, o| {
                    // Upstream computes `s.key[axis] +/- center_offset_key`
                    // as a comparison operand, never reassigning it into a
                    // `key_type` (`uint16_t`) variable -- so C++'s integer
                    // promotion widens both operands to `int` first, and the
                    // arithmetic never wraps at 16 bits. Widening to `i32`
                    // here is not optional: at inner-node depths
                    // `k[axis] + o` legitimately exceeds `u16::MAX` (e.g.
                    // `49152 + 16384 = 65536`), and comparing that against
                    // `min_key`/`max_key` after a `u16` wraparound would
                    // silently reject a real overlap.
                    (0..3).all(|axis| {
                        i32::from(min_key[axis]) <= i32::from(k[axis]) + i32::from(o)
                            && i32::from(max_key[axis]) >= i32::from(k[axis]) - i32::from(o)
                    })
                });
                continue;
            }
            return Some(Leaf {
                tree: self.tree,
                key: elem.key,
                depth: elem.depth,
                log_odds: elem.node.log_odds,
            });
        }
        None
    }
}

/// A single leaf yielded by [`Leaves`] or [`LeavesInBbx`]: its key, the depth
/// it was found at, and its occupancy. Not a direct upstream type --
/// upstream's iterator dereferences to the node itself plus free functions
/// (`getCoordinate()`, `getSize()`) on the iterator; this bundles the same
/// information into one value since Rust iterators can't return a live
/// reference to iterator-internal state alongside tree-level lookups.
pub struct Leaf<'a> {
    tree: &'a OcTree,
    key: OcTreeKey,
    depth: u32,
    log_odds: f32,
}

impl Leaf<'_> {
    /// Upstream `iterator_base::getKey`.
    pub fn key(&self) -> OcTreeKey {
        self.key
    }

    /// Upstream `iterator_base::getIndexKey`: the key shared by every
    /// address at this leaf's own (possibly coarser-than-finest) depth,
    /// masking off the bits [`Self::key`] carries below that level.
    pub fn index_key(&self) -> OcTreeKey {
        compute_index_key(OcTree::TREE_DEPTH - self.depth, self.key)
    }

    /// Upstream `iterator_base::getDepth`.
    pub fn depth(&self) -> u32 {
        self.depth
    }

    /// Upstream `iterator_base::getCoordinate`.
    pub fn coordinate(&self) -> Point3<f64> {
        self.tree.key_to_coord_at_depth(self.key, self.depth)
    }

    /// Upstream `iterator_base::getSize`.
    pub fn size(&self) -> f64 {
        self.tree.node_size(self.depth)
    }

    /// Upstream `OcTreeNode::getLogOdds` on the dereferenced node.
    pub fn log_odds(&self) -> f32 {
        self.log_odds
    }

    /// Upstream `OcTreeNode::getOccupancy`.
    pub fn occupancy(&self) -> f64 {
        crate::octomap::tree::probability(f64::from(self.log_odds))
    }

    /// Upstream `AbstractOccupancyOcTree::isNodeOccupied` applied to this leaf.
    pub fn is_occupied(&self) -> bool {
        self.tree.is_node_occupied_log_odds(self.log_odds)
    }
}

/// Iterator over every node in an [`OcTree`] -- inner nodes as well as
/// leaves -- in the same pre-order as [`Leaves`] (a node before its
/// children, child 0's subtree before child 1's). Upstream `tree_iterator`.
///
/// Unlike [`Leaves`], which decides per-node whether to yield it (leaf) or
/// descend into it (inner node), `tree_iterator` does both for every node:
/// upstream's `singleIncrement` unconditionally queues an already-visited
/// node's children (a no-op if it has none) once its depth is below the
/// traversal's max depth, regardless of whether the node is itself a leaf.
/// [`TreeNode::is_leaf`] surfaces upstream `tree_iterator::isLeaf` for
/// callers that need to distinguish the two after the fact.
pub struct TreeNodes<'a> {
    tree: &'a OcTree,
    stack: Vec<StackElem<'a>>,
}

impl<'a> TreeNodes<'a> {
    pub(crate) fn new(tree: &'a OcTree) -> Self {
        let mut stack = Vec::new();
        if let Some(root) = tree.root() {
            stack.push(StackElem {
                node: root,
                key: OcTree::root_key(),
                depth: 0,
            });
        }
        Self { tree, stack }
    }
}

impl<'a> Iterator for TreeNodes<'a> {
    type Item = TreeNode<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let elem = self.stack.pop()?;
        if elem.depth < OcTree::TREE_DEPTH {
            push_children(&mut self.stack, elem.node, elem.key, elem.depth, |_, _| {
                true
            });
        }
        Some(TreeNode {
            tree: self.tree,
            node: elem.node,
            key: elem.key,
            depth: elem.depth,
        })
    }
}

/// A single node yielded by [`TreeNodes`]: its key, the depth it was found
/// at, and its occupancy -- inner node or leaf alike. Not a direct upstream
/// type, for the same reason [`Leaf`] isn't: upstream's iterator
/// dereferences to the node itself plus free accessor functions on the
/// iterator, which Rust's `Iterator` can't yield as one live borrow.
pub struct TreeNode<'a> {
    tree: &'a OcTree,
    node: &'a Node,
    key: OcTreeKey,
    depth: u32,
}

impl TreeNode<'_> {
    /// Upstream `iterator_base::getKey`.
    pub fn key(&self) -> OcTreeKey {
        self.key
    }

    /// Upstream `iterator_base::getIndexKey`.
    pub fn index_key(&self) -> OcTreeKey {
        compute_index_key(OcTree::TREE_DEPTH - self.depth, self.key)
    }

    /// Upstream `iterator_base::getDepth`.
    pub fn depth(&self) -> u32 {
        self.depth
    }

    /// Upstream `iterator_base::getCoordinate` (and, per-axis, `getX`/`getY`/`getZ`).
    pub fn coordinate(&self) -> Point3<f64> {
        self.tree.key_to_coord_at_depth(self.key, self.depth)
    }

    /// Upstream `iterator_base::getSize`.
    pub fn size(&self) -> f64 {
        self.tree.node_size(self.depth)
    }

    /// Upstream `tree_iterator::isLeaf`: no children, or already at the
    /// traversal's max depth.
    pub fn is_leaf(&self) -> bool {
        !self.node.has_children() || self.depth == OcTree::TREE_DEPTH
    }

    /// Upstream `OcTreeNode::getLogOdds` on the dereferenced node.
    pub fn log_odds(&self) -> f32 {
        self.node.log_odds
    }

    /// Upstream `OcTreeNode::getOccupancy`.
    pub fn occupancy(&self) -> f64 {
        crate::octomap::tree::probability(f64::from(self.node.log_odds))
    }

    /// Upstream `AbstractOccupancyOcTree::isNodeOccupied` applied to this node.
    pub fn is_occupied(&self) -> bool {
        self.tree.is_node_occupied_log_odds(self.node.log_odds)
    }
}
