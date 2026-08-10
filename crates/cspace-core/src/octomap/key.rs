// Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from octomap 1.9.7 (Debian package liboctomap-dev 1.9.7+dfsg-3.1build3,
// version confirmed by octomap-config.cmake's OCTOMAP_VERSION inside the
// cspace oracle container; see lib.rs's module docs for how this crate's
// existence was decided and what was and was not ported):
//   include/octomap/OcTreeKey.h

use std::collections::HashSet;

/// Per-axis discrete voxel address. Upstream `octomap::key_type` (`uint16_t`).
///
/// Upstream's tree has a fixed depth of 16, so a coordinate is addressed by
/// counting voxels from the tree's origin in a `2^16`-wide range per axis;
/// [`OcTree::TREE_MAX_VAL`](crate::octomap::OcTree::TREE_MAX_VAL) (`32768`, upstream
/// `tree_max_val`) is the coordinate of the tree center.
pub type KeyType = u16;

/// 3D addressing key for a voxel. Upstream `octomap::OcTreeKey`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct OcTreeKey {
    k: [KeyType; 3],
}

impl OcTreeKey {
    /// Upstream `OcTreeKey(key_type, key_type, key_type)`.
    pub fn new(x: KeyType, y: KeyType, z: KeyType) -> Self {
        Self { k: [x, y, z] }
    }

    /// Upstream `OcTreeKey::operator[]`.
    pub fn get(&self, axis: usize) -> KeyType {
        self.k[axis]
    }
}

impl std::ops::Index<usize> for OcTreeKey {
    type Output = KeyType;
    fn index(&self, axis: usize) -> &KeyType {
        &self.k[axis]
    }
}

/// Hash set of keys. Upstream `octomap::KeySet` (an `unordered_set` keyed on
/// `OcTreeKey::KeyHash`). This port reuses `OcTreeKey`'s `#[derive(Hash)]`
/// (a componentwise hash via `std::hash::Hash`'s tuple-like derive) rather
/// than porting upstream's specific multiplier hash
/// (`k[0] + 1447*k[1] + 345637*k[2]`): nothing observes the hash *values*,
/// only set membership, and `HashSet`'s bucket layout is already an
/// implementation detail upstream callers don't rely on either.
pub type KeySet = HashSet<OcTreeKey>;

/// Ordered list of keys traversed by a ray. Upstream `octomap::KeyRay`.
///
/// Upstream backs this with a fixed-capacity `std::vector` (max size
/// `100000`) to avoid reallocation during ray casting and asserts if that
/// capacity is exceeded. This port uses a growable `Vec`: the capacity limit
/// was upstream's own allocation optimization, not a semantic bound other
/// code depends on, so there is nothing to preserve by refusing rays longer
/// than 100000 voxels.
pub type KeyRay = Vec<OcTreeKey>;

/// Computes the key of a child node while descending the tree.
///
/// Upstream `computeChildKey`. `pos` is the child index (`0..8`, one bit per
/// axis); `center_offset_key` is `tree_max_val >> depth` at the child's
/// depth.
pub fn compute_child_key(pos: u8, center_offset_key: KeyType, parent_key: OcTreeKey) -> OcTreeKey {
    let axis = |bit: u8, i: usize| -> KeyType {
        if pos & bit != 0 {
            parent_key[i] + center_offset_key
        } else {
            parent_key[i] - center_offset_key - KeyType::from(center_offset_key == 0)
        }
    };
    OcTreeKey::new(axis(1, 0), axis(2, 1), axis(4, 2))
}

/// Computes which of the 8 children `key` falls under at tree depth `depth`
/// (bit position counted from the finest level). Upstream `computeChildIdx`.
///
/// # Deviation: the shift runs in `u32`, not `KeyType`
///
/// Upstream's `depth` is `int`, and `key.k[i]` (`uint16_t`) integer-promotes
/// to `int` before `&` (`OcTreeKey.h:207-219`), so `1 << depth` for `depth`
/// in `[16, 30]` is well-defined 32-bit arithmetic that only sets bits the
/// promoted `key.k[i]` can never have -- always false, not UB. A direct
/// `KeyType` (`u16`) shift does not have that headroom: `1u16 << depth`
/// overflows the moment `depth >= 16` (this function is `pub`, so `depth`
/// is caller-controlled). Shifting in `u32` instead mirrors the promotion
/// and reproduces the always-false result through `depth = 31`.
pub fn compute_child_idx(key: OcTreeKey, depth: u32) -> u8 {
    let mut pos = 0u8;
    if u32::from(key[0]) & (1u32 << depth) != 0 {
        pos += 1;
    }
    if u32::from(key[1]) & (1u32 << depth) != 0 {
        pos += 2;
    }
    if u32::from(key[2]) & (1u32 << depth) != 0 {
        pos += 4;
    }
    pos
}

/// Generates the unique key shared by all keys at a given tree `level`
/// (counted from the bottom: `level = tree_depth - depth`). Upstream
/// `computeIndexKey`.
pub fn compute_index_key(level: u32, key: OcTreeKey) -> OcTreeKey {
    if level == 0 {
        return key;
    }
    let mask = (0xffffu32 << level) as KeyType;
    OcTreeKey::new(key[0] & mask, key[1] & mask, key[2] & mask)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compute_child_idx_reads_one_bit_per_axis() {
        let key = OcTreeKey::new(0b0010, 0b0000, 0b0010);
        assert_eq!(compute_child_idx(key, 1), 1 | 4);
        assert_eq!(compute_child_idx(key, 0), 0);
    }

    #[test]
    fn compute_child_key_round_trips_through_compute_child_idx() {
        // A child key built by compute_child_key must itself decode to the
        // same child index at the same depth bit -- this is the invariant
        // computeChildIdx and computeChildKey both exist to preserve during
        // tree descent (search() and updateNodeRecurs() rely on it).
        //
        // Verified against OcTreeBaseImpl::search's actual descent loop
        // (`for i=(tree_depth-1); i>=diff; --i) pos = computeChildIdx(key, i)`):
        // the very first step, choosing among the ROOT's children, reads bit
        // 15 (tree_depth - 1), not bit 14 -- computeChildIdx's `depth`
        // parameter is the bit index of the *parent* being descended from,
        // not of the child being decoded.
        let parent = OcTreeKey::new(32768, 32768, 32768);
        let depth_bit = 15; // bit read to choose among the root's children
        let offset = 32768 >> 1; // center_offset_key at the children's depth (1)
        for pos in 0u8..8 {
            let child = compute_child_key(pos, offset, parent);
            assert_eq!(compute_child_idx(child, depth_bit), pos);
        }
    }

    #[test]
    fn compute_child_key_center_offset_zero_uses_the_minus_one_branch() {
        // At the finest level center_offset_key is 0, and upstream's
        // "- (center_offset_key ? 0 : 1)" branch only fires here -- verify
        // both the negative (bit clear) and positive (bit set) cases.
        let parent = OcTreeKey::new(100, 100, 100);
        assert_eq!(compute_child_key(0, 0, parent), OcTreeKey::new(99, 99, 99));
        assert_eq!(
            compute_child_key(0b111, 0, parent),
            OcTreeKey::new(100, 100, 100)
        );
    }

    #[test]
    fn compute_index_key_masks_low_bits() {
        let key = OcTreeKey::new(0b1011, 0b1011, 0b1011);
        assert_eq!(compute_index_key(0, key), key);
        assert_eq!(
            compute_index_key(2, key),
            OcTreeKey::new(0b1000, 0b1000, 0b1000)
        );
    }

    #[test]
    fn compute_child_idx_depth_at_or_above_key_width_is_always_false() {
        // Upstream `computeChildIdx(const OcTreeKey& key, int depth)`:
        // `key.k[i]` (`uint16_t`) integer-promotes to `int` before `&`, so
        // for `depth` in [16, 30] `1 << depth` only sets bits the promoted
        // `key.k[i]` can never have -- the condition is always false,
        // well-defined, regardless of `key`'s value (`OcTreeKey.h:207-219`).
        // `KeyType = u16` here means a naive `1 << depth` port overflows a
        // u16 shift the moment `depth >= 16`. Doing the shift in `u32`
        // instead (mirroring the promotion) reproduces upstream's
        // always-false result through `depth = 31`, one bit further than
        // upstream's own well-defined range.
        let key = OcTreeKey::new(u16::MAX, u16::MAX, u16::MAX);
        for depth in 16..32 {
            assert_eq!(
                compute_child_idx(key, depth),
                0,
                "depth {depth}: upstream's int-promoted (uint16_t & (1 << depth)) is \
                 always false once depth >= 16, regardless of key's value"
            );
        }
    }
}
