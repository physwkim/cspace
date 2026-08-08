// Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from octomap 1.9.7 (see key.rs's provenance comment for how the
// version was matched):
//   include/octomap/OcTreeNode.h
//   include/octomap/OcTreeDataNode.h, OcTreeDataNode.hxx
//   include/octomap/OcTreeBaseImpl.h, OcTreeBaseImpl.hxx (child bookkeeping:
//     createNodeChild, deleteNodeChild, isNodeCollapsible, pruneNode,
//     expandNode, allocNodeChildren)

/// One node of the tree: a log-odds occupancy value plus up to 8 children.
/// Upstream splits this across `OcTreeDataNode<float>` (payload + children
/// pointer array) and `OcTreeNode` (occupancy accessors); this port merges
/// them since D4 rules out upstream's node/tree template hierarchy
/// (`OcTreeDataNode<T>` templated on payload type, `OcTreeBaseImpl<NODE,
/// INTERFACE>` templated on node and interface type) in favor of one
/// concrete type, matching the crate's scope: only the plain occupancy
/// `OcTree` is ported, not `ColorOcTree`/`CountingOcTree`/`OcTreeStamped`,
/// none of which moveit2 ever references.
///
/// Upstream represents "no children" as a `NULL` children-array pointer and
/// "child `i` absent" as `children[i] == NULL` within an allocated array --
/// two independent levels of nullability, because `createNodeChild` (called
/// once per descended path during an update) allocates the whole 8-slot
/// array up front but only ever populates the one index actually needed,
/// leaving the other 7 slots `NULL` in steady state, not just transiently.
/// `Option<Box<[Option<Node>; 8]>>` mirrors that exactly: the outer `Option`
/// is "array allocated at all", the inner per-slot `Option` is "this child
/// exists".
#[derive(Debug, Clone)]
pub(crate) struct Node {
    /// Log-odds occupancy value. Upstream `OcTreeDataNode<float>::value`,
    /// read through `OcTreeNode::getLogOdds`/`setLogOdds`.
    pub(crate) log_odds: f32,
    children: Option<Box<[Option<Node>; 8]>>,
}

impl Node {
    /// Upstream `new NODE()` at a freshly created child: `OcTreeDataNode`'s
    /// default constructor leaves `value` with no initializer in the header
    /// -- the actual zero-initialization lives in `OcTreeNode`'s constructor
    /// definition, compiled into `liboctomap.so` and not shipped as source.
    /// `0.0` (log-odds for probability `0.5`, "unknown") is the only value
    /// consistent with `updateNodeRecurs`'s use of `addValue` (which adds
    /// onto whatever a freshly created node already holds) actually
    /// producing the documented sensor-model behavior, and matches the
    /// occupancy-mapping literature's standard "unknown" prior; Step 3's
    /// oracle probe cross-checks this against the shipped binary (a fresh,
    /// never-updated node's occupancy must read back as exactly `0.5`).
    pub(crate) fn new() -> Self {
        Self {
            log_odds: 0.0,
            children: None,
        }
    }

    /// Upstream `OcTreeBaseImpl::nodeHasChildren`.
    pub(crate) fn has_children(&self) -> bool {
        self.children
            .as_ref()
            .is_some_and(|c| c.iter().any(Option::is_some))
    }

    /// Upstream `OcTreeBaseImpl::nodeChildExists` + `getNodeChild` combined:
    /// `None` covers both "no children array" and "this slot is empty".
    pub(crate) fn child(&self, idx: usize) -> Option<&Node> {
        self.children.as_ref()?[idx].as_ref()
    }

    pub(crate) fn child_mut(&mut self, idx: usize) -> Option<&mut Node> {
        self.children.as_mut()?[idx].as_mut()
    }

    /// Upstream `OcTreeBaseImpl::allocNodeChildren` + `createNodeChild`.
    ///
    /// # Deviation: explicit `debug_assert!` for "slot not already occupied"
    ///
    /// Upstream's `createNodeChild` guards the same precondition with
    /// `assert (node->children[childIdx] == NULL);` (`OcTreeBaseImpl.hxx:178`)
    /// -- in C++, calling it on an occupied slot leaks the orphaned child's
    /// subtree (the raw pointer is overwritten, never `delete`d) in a
    /// release (`NDEBUG`) build. This port cannot leak that way --
    /// `children[idx] = Some(Node::new())` drops the old `Node` (and
    /// recursively its own children) instead of leaking it -- but an
    /// overwrite would still silently discard real occupancy data, a logic
    /// bug upstream's assert exists to catch in debug builds. Every current
    /// caller ([`Node::expand`], `update_node_recurs`,
    /// `create_binary_children`) already guards this by construction, so
    /// this is currently unreachable -- the same "safe by caller discipline,
    /// not by construction" situation as `OcTree::search`'s `debug_assert!`
    /// (Task G). `create_child` is `pub(crate)`, so a future same-crate
    /// caller could still violate it silently without this check.
    pub(crate) fn create_child(&mut self, idx: usize) -> &mut Node {
        let children = self
            .children
            .get_or_insert_with(|| Box::new(std::array::from_fn(|_| None)));
        debug_assert!(
            children[idx].is_none(),
            "create_child: slot {idx} is already occupied; this would silently \
             discard its existing subtree instead of creating a fresh child"
        );
        children[idx] = Some(Node::new());
        children[idx].as_mut().expect("just inserted")
    }

    /// Upstream `OcTreeBaseImpl::expandNode`: reverse of pruning, creating
    /// all 8 children with the parent's current value (`copyData`).
    ///
    /// Upstream's own precondition, `assert(!nodeHasChildren(node));`
    /// (`OcTreeBaseImpl.hxx:258`), compiles out under `NDEBUG` -- a release
    /// build calling this on an already-expanded node falls through to
    /// `createNodeChild`'s own release-mode UB on the first already-occupied
    /// slot ([`Self::create_child`]'s own doc has that half). `debug_assert!`
    /// matches upstream's NDEBUG semantics: checked in debug, compiled out
    /// in release, same as upstream's `assert()`. Task G.
    pub(crate) fn expand(&mut self) {
        debug_assert!(!self.has_children());
        let value = self.log_odds;
        for idx in 0..8 {
            self.create_child(idx).log_odds = value;
        }
    }

    /// Upstream `OcTreeBaseImpl::isNodeCollapsible`: all 8 children exist,
    /// none has children of its own, and all 8 hold the identical value.
    pub(crate) fn is_collapsible(&self) -> bool {
        let Some(first) = self.child(0) else {
            return false;
        };
        if first.has_children() {
            return false;
        }
        (1..8).all(|i| {
            self.child(i)
                .is_some_and(|c| !c.has_children() && c.log_odds == first.log_odds)
        })
    }

    /// Upstream `OcTreeBaseImpl::pruneNode`: collapse 8 identical leaf
    /// children into this node, if collapsible.
    pub(crate) fn prune(&mut self) -> bool {
        if !self.is_collapsible() {
            return false;
        }
        self.log_odds = self.child(0).expect("checked by is_collapsible").log_odds;
        self.children = None;
        true
    }

    /// Upstream `OcTreeNode::getMaxChildLogOdds` + `updateOccupancyChildren`:
    /// a parent's occupancy is conservatively the maximum of its children's
    /// (an unpruned occupied child must keep its parent reachable as
    /// occupied at coarser depths).
    pub(crate) fn update_occupancy_from_children(&mut self) {
        let max = (0..8)
            .filter_map(|i| self.child(i))
            .map(|c| c.log_odds)
            .fold(f32::NEG_INFINITY, f32::max);
        self.log_odds = max;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fresh_node_has_no_children_and_zero_log_odds() {
        let n = Node::new();
        assert_eq!(n.log_odds, 0.0);
        assert!(!n.has_children());
        assert!(n.child(0).is_none());
    }

    #[test]
    fn create_child_populates_exactly_one_of_eight_slots() {
        let mut n = Node::new();
        n.create_child(3).log_odds = 1.5;
        assert!(n.has_children());
        assert!(n.child(3).is_some());
        for i in [0, 1, 2, 4, 5, 6, 7] {
            assert!(n.child(i).is_none(), "slot {i} should stay empty");
        }
    }

    #[test]
    fn expand_gives_all_eight_children_the_parent_value() {
        let mut n = Node::new();
        n.log_odds = 2.0;
        n.expand();
        for i in 0..8 {
            assert_eq!(n.child(i).unwrap().log_odds, 2.0);
        }
    }

    #[test]
    #[should_panic(expected = "assertion failed: !self.has_children()")]
    fn expand_on_an_already_expanded_node_panics_in_debug() {
        // Upstream `assert(!nodeHasChildren(node));` (`OcTreeBaseImpl.hxx:258`)
        // -- pre-fix, this doc comment did not cite the precondition at all
        // and nothing pinned the debug_assert! actually firing.
        let mut n = Node::new();
        n.expand();
        n.expand(); // already has 8 children
    }

    #[test]
    fn is_collapsible_false_when_a_child_is_missing() {
        let mut n = Node::new();
        for i in 0..7 {
            n.create_child(i).log_odds = 1.0;
        }
        assert!(!n.is_collapsible(), "child 7 is missing");
    }

    #[test]
    fn is_collapsible_false_when_a_child_has_grandchildren() {
        let mut n = Node::new();
        n.expand();
        n.child_mut(0).unwrap().expand();
        assert!(!n.is_collapsible());
    }

    #[test]
    fn is_collapsible_false_when_values_differ() {
        let mut n = Node::new();
        n.expand();
        n.child_mut(3).unwrap().log_odds = 9.0;
        assert!(!n.is_collapsible());
    }

    #[test]
    fn prune_collapses_eight_identical_children_and_drops_the_array() {
        let mut n = Node::new();
        n.expand(); // all 8 children now share n's (0.0) value
        n.child_mut(0).unwrap().log_odds = 3.25;
        for i in 1..8 {
            n.child_mut(i).unwrap().log_odds = 3.25;
        }
        assert!(n.prune());
        assert_eq!(n.log_odds, 3.25);
        assert!(!n.has_children());
    }

    #[test]
    fn update_occupancy_from_children_takes_the_max() {
        let mut n = Node::new();
        for i in 0..8 {
            n.create_child(i).log_odds = i as f32 - 4.0;
        }
        n.update_occupancy_from_children();
        assert_eq!(n.log_odds, 3.0); // child 7: 7 - 4 = 3
    }

    #[test]
    #[should_panic(expected = "create_child: slot")]
    fn create_child_on_an_already_occupied_slot_panics_in_debug() {
        // Upstream `assert (node->children[childIdx] == NULL);`
        // (`OcTreeBaseImpl.hxx:178`) -- pre-fix, this silently overwrote
        // (and correctly dropped, but still discarded) the existing child
        // instead of panicking at all.
        let mut n = Node::new();
        n.create_child(0);
        n.create_child(0); // slot 0 is already occupied
    }
}
