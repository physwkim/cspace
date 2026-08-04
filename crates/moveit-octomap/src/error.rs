// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Error type for [`crate::OcTree::read_binary_data`]/[`crate::OcTree::read_data`].
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

/// Why [`crate::OcTree::read_binary_data`] or [`crate::OcTree::read_data`]
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

    /// Node/child recursion nested past [`crate::OcTree::TREE_DEPTH`] (16)
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
        crate::OcTree::TREE_DEPTH
    )]
    MaxDepthExceeded,
}
