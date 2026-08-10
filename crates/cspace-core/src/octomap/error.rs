// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Error type for [`crate::octomap::OcTree::read_binary_data`]/[`crate::octomap::OcTree::read_data`].
//!
//! Not a port of a named upstream type -- upstream `readBinaryData`/
//! `readBinaryNode` (`third_party/octomap/octomap/include/octomap/
//! OccupancyOcTreeBase.hxx`) and `readData`/`readNodesRecurs`
//! (`OcTreeBaseImpl.hxx`) read from a C++ `std::istream`, which fails
//! silently: a short read leaves the destination unmodified (or, for a
//! freshly declared local with no initializer, indeterminate) and only sets
//! the stream's failbit, which none of those four functions ever checks
//! before using the value; "decode into a tree that already has a root" is
//! logged with `OCTOMAP_ERROR_STR` and the call returns as if it had
//! succeeded. This crate's two entry points read from a caller-supplied
//! `&[u8]` instead of a stream that can be silently in a failed state, so
//! every one of those points becomes a typed variant here.

/// Why [`crate::octomap::OcTree::read_binary_data`] or [`crate::octomap::OcTree::read_data`]
/// failed to decode a byte slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum DecodeError {
    /// Upstream `readBinaryData`/`readData`: `if (this->root) {
    /// OCTOMAP_ERROR_STR("Trying to read into an existing tree."); return s;
    /// }` -- decoding into a tree that already has content is refused, not
    /// merged. Upstream logs and silently no-ops, leaving the tree
    /// unchanged and the caller with no indication anything went wrong;
    /// this port refuses with a typed error instead.
    #[error(
        "cannot decode into an OcTree that already has a root; decode into a freshly constructed OcTree::new(resolution)"
    )]
    TreeAlreadyPopulated,

    /// The byte slice ended before a node or child record the wire format
    /// says must follow. Upstream's `istream::read` on a short read neither
    /// throws nor guarantees the destination is untouched; every recursive
    /// step here returns `Err` at the exact point upstream would have read
    /// past the end of the buffer instead.
    #[error("unexpected end of input while decoding an octree node")]
    UnexpectedEof,

    /// Node/child recursion nested past [`crate::octomap::OcTree::TREE_DEPTH`] (16)
    /// levels, the deepest an octree with a 16-bit key can represent.
    /// Upstream has no depth bound here at all: `readBinaryNode`/
    /// `readNodesRecurs` recurse for as many "has children" bits as the
    /// stream contains, with no upstream caller ever handing them anything
    /// but a stream that was itself produced by `writeBinaryNode`/
    /// `writeNodesRecurs` from a real, depth-bounded tree, so this never
    /// fires on trusted input -- but `&[u8]` from a decoded ROS message is
    /// exactly the untrusted case that bound protects: this port caps
    /// recursion at the one depth a real tree can ever reach rather than
    /// growing the call stack without bound on crafted input.
    #[error(
        "octree node nesting exceeded the maximum tree depth ({} levels)",
        crate::octomap::OcTree::TREE_DEPTH
    )]
    MaxDepthExceeded,

    /// This crate's own invariant, not an upstream port -- upstream's
    /// `OcTreeBaseImpl` constructor and `setResolution`
    /// (`third_party/octomap/octomap/include/octomap/OcTreeBaseImpl.hxx:
    /// 46-59,156-158`) store `resolution` unchecked and divide `1./
    /// resolution` unguarded, an upstream defect this port does not
    /// inherit.
    ///
    /// Checked here, at decode, rather than in [`crate::octomap::OcTree::new`]
    /// itself or at each untrusted-data boundary that calls it: a
    /// non-positive or non-finite resolution does not fail loudly on its
    /// own. `resolution_factor` becomes `+-Infinity` or `NaN`, but the only
    /// function that reads it (`coord_to_key_checked_axis`) already rejects
    /// every coordinate via its own NaN/Infinity-safe range check --
    /// `update_node` and every other coordinate-keyed write silently
    /// no-ops, leaving the tree empty, not corrupted. Decode is different:
    /// [`crate::octomap::OcTree::read_binary_data`]/[`crate::octomap::OcTree::read_data`]
    /// never touch `resolution` at all, so they succeed and populate real
    /// leaves -- and every leaf's own coordinate and size then come from
    /// `key_to_coord_axis`, `(key - TREE_MAX_VAL + 0.5) * self.resolution`,
    /// a *multiplication*, not a division, with no NaN or Infinity for a
    /// comparison guard to catch. At `resolution == 0.0` that silently
    /// collapses every occupied leaf's coordinate and size to `0.0`
    /// regardless of its real key -- measured (a leaf decoded at
    /// `resolution = 0.0` reports `is_occupied == true`, `coordinate ==
    /// (0.0, 0.0, -0.0)`, `size == 0.0`, in place of its real position),
    /// and `crate::octomap::octree_collision::compound_from_octree`
    /// (`crates/cspace-core`) turns that into a real `Some(Compound)`
    /// of zero-volume boxes stacked at the world origin: a planning scene
    /// that silently drops every real octomap obstacle rather than
    /// reporting one. Decode is the one place every caller that can
    /// populate a tree with real content already passes through, in this
    /// crate or any other, so rejecting here closes the family at its one
    /// shared choke point instead of patching `key_to_coord_axis` or each
    /// downstream consumer separately.
    #[error("octree resolution must be a positive, finite value")]
    InvalidResolution,
}
