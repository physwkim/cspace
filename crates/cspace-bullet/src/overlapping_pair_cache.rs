// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/BroadphaseCollision/btOverlappingPairCache.h
//   bullet3/src/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp
//   bullet3/src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.h
//   bullet3/src/LinearMath/btAlignedObjectArray.h

//! `btHashedOverlappingPairCache` -- the open-addressed pair set
//! [`crate::dbvt_broadphase::DbvtBroadphase`] adds every AABB overlap to, and
//! the thing that decides what order the consumer sees them in.
//!
//! # Order is the output, not the set
//!
//! MoveIt's continuous check walks this cache with
//! `processAllOverlappingPairs` and stops early: its callback sets `done` when
//! a contact ends the query, and `contact_test_data.res` is truncated at
//! `max_contacts`. So which pair is visited *first* decides which contacts are
//! reported, and a port that agrees on the pair set while disagreeing on the
//! order is wrong in a way no set comparison can see.
//!
//! The visit order is [`HashedOverlappingPairCache::overlapping_pair_array`]'s
//! index order, and three things decide it, all of which are ported literally
//! here rather than approximated:
//!
//! - **Insertion appends.** `internalAddPair` puts a new pair at the end.
//! - **Removal swaps with the last.** `removeOverlappingPair` moves the final
//!   pair into the hole rather than shifting, so removing pair *i* renumbers
//!   exactly one other pair -- and `processAllOverlappingPairs` does not
//!   advance `i` after a removal, so the moved-in pair is visited next, at the
//!   index the removed one occupied.
//! - **The array's capacity is the hash mask.** `getHash(...) & (capacity - 1)`
//!   reads `btAlignedObjectArray::capacity()`, whose growth policy is
//!   `size ? size * 2 : 1` from a constructor-time `reserve(2)`, not a
//!   `Vec`'s. The private `PairArray` models that capacity explicitly for
//!   exactly this reason: a Rust `Vec`'s own capacity is an allocator's
//!   business and would silently give a different mask.
//!
//! # What the hash decides, and what it does not
//!
//! The mix is Thomas Wang's, reproduced bit for bit in
//! [`HashedOverlappingPairCache::get_hash`]. It decides which bucket chain a
//! pair lands in -- and *not* the array order, because `internalFindPair`
//! walks a chain comparing unique ids rather than trusting the bucket. Any
//! self-consistent hash would therefore produce the same pair sequence, which
//! is why this module's fixture asserts against the probe's `m_hashTable` and
//! `m_next` contents directly (`dbvt_broadphase`'s `bp_*_ht` / `bp_*_nx`
//! rows) instead of inferring the mix from pair order it cannot constrain.
//!
//! # Deliberately absent
//!
//! `m_algorithm` and `m_internalInfo1`. `btBroadphasePair` carries a
//! `btCollisionAlgorithm*` for the dispatcher to cache narrow-phase state in;
//! this crate has no `btCollisionAlgorithm` and MoveIt's continuous callback
//! never sets one, so `cleanOverlappingPair` -- whose whole body is
//! `if (pair.m_algorithm && dispatcher)` -- has nothing to do. That is why
//! [`HashedOverlappingPairCache::clean_proxy_from_pairs`] is a traversal whose
//! callback always answers `false`: it is upstream's, and upstream's is a
//! no-op here. It stays because the consumer calls it before `destroyProxy`.
//!
//! The `m_ghostPairCallback` hooks (`btGhostObject`), `sortOverlappingPairs`,
//! the `btDispatcherInfo`-taking `processAllOverlappingPairs` overload (which
//! sorts by unique id when `m_deterministicOverlappingPairs` is set; MoveIt's
//! `btDispatcherInfo` leaves it false and calls the two-argument overload),
//! `btSortedOverlappingPairCache` and `btNullPairCache` are all likewise
//! unreachable from the continuous path and are not ported.

use crate::broadphase_proxy::CollisionFilterGroup;
use crate::linear_math::Vec3;

/// `BT_NULL_PAIR` (`btOverlappingPairCache.h:46`).
///
/// Upstream spells it `const int BT_NULL_PAIR = 0xffffffff`, which is a
/// conversion of an `unsigned int` to `int`; every target this port is built
/// for is two's complement, so the stored and compared value is `-1`. It is
/// only ever compared against and never used as an index without that
/// comparison guarding it first.
const NULL_PAIR: i32 = -1;

/// `btBroadphaseProxy*` -- a proxy's identity, as an index into the arena the
/// broadphase owns.
///
/// Upstream this is a raw pointer and the cache dereferences it for
/// `m_uniqueId`, `m_collisionFilterGroup` and `m_collisionFilterMask`. Here
/// the cache is handed the arena as a [`PairProxies`] alongside the handle,
/// which is the same three reads through one more indirection.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ProxyHandle(pub usize);

impl ProxyHandle {
    /// The null proxy pointer a default-constructed `btBroadphasePair` holds.
    ///
    /// Reachable only as the placeholder `PairArray::expand_non_initializing`
    /// pushes into the slot upstream leaves as raw memory; every caller
    /// overwrites it before anything reads it.
    pub const NULL: Self = Self(usize::MAX);
}

/// `btBroadphaseProxy` (`btBroadphaseProxy.h:84-169`), less the shape-type
/// predicates -- those are [`crate::broadphase_proxy::BroadphaseNativeType`],
/// the other half of the same header.
///
/// It lives in this module rather than beside them because the pair cache is
/// what dereferences it: `needsBroadphaseCollision` reads the two filter
/// words and every hash reads `m_uniqueId`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BroadphaseProxy {
    /// `m_clientObject`, upstream a `void*` the broadphase never dereferences
    /// -- it is the caller's own handle on whatever the proxy stands for. A
    /// `usize` here for the same reason: the port must be able to carry it
    /// through and hand it back, and must not be able to follow it.
    pub client_object: usize,
    /// `m_collisionFilterGroup`.
    pub collision_filter_group: CollisionFilterGroup,
    /// `m_collisionFilterMask`, the same bitfield read the other way round --
    /// which is why it is the same type and not a second one.
    pub collision_filter_mask: CollisionFilterGroup,
    /// `m_uniqueId`, assigned `++m_gid` at `createProxy`, so it is the
    /// creation index plus one. Every pair stores its two proxies ordered by
    /// it, and the hash is taken over the ordered pair.
    pub unique_id: i32,
    /// `m_aabbMin`.
    pub aabb_min: Vec3,
    /// `m_aabbMax`.
    pub aabb_max: Vec3,
}

impl BroadphaseProxy {
    /// `btBroadphaseProxy::getUid` (`btBroadphaseProxy.h:111-114`).
    #[must_use]
    pub fn get_uid(&self) -> i32 {
        self.unique_id
    }
}

/// The proxy arena, as the pair cache sees it.
///
/// One method, because one dereference is all upstream's cache performs on a
/// `btBroadphaseProxy*`.
pub trait PairProxies {
    /// The proxy `handle` names.
    ///
    /// # Panics
    ///
    /// If `handle` is not one this arena issued.
    fn proxy(&self, handle: ProxyHandle) -> &BroadphaseProxy;
}

/// `btBroadphasePair` (`btBroadphaseProxy.h:177-216`).
///
/// See the module docs for why `m_algorithm` and `m_internalInfo1` are absent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BroadphasePair {
    /// `m_pProxy0` -- the lower `m_uniqueId` of the two.
    pub proxy0: ProxyHandle,
    /// `m_pProxy1` -- the higher.
    pub proxy1: ProxyHandle,
}

impl BroadphasePair {
    /// The default-constructed pair (`btBroadphaseProxy.h:180-186`): two null
    /// proxies. See [`ProxyHandle::NULL`] for the one place it is reachable.
    pub const NULL: Self = Self {
        proxy0: ProxyHandle::NULL,
        proxy1: ProxyHandle::NULL,
    };

    /// `btBroadphasePair(btBroadphaseProxy&, btBroadphaseProxy&)`
    /// (`btBroadphaseProxy.h:190-206`) -- "keep them sorted, so the std::set
    /// operations work".
    ///
    /// The comparison is `<`, so equal unique ids would put the *second*
    /// argument first; ids are unique, so that arm is unreachable.
    #[must_use]
    pub fn new(proxies: &dyn PairProxies, proxy0: ProxyHandle, proxy1: ProxyHandle) -> Self {
        if proxies.proxy(proxy0).unique_id < proxies.proxy(proxy1).unique_id {
            Self { proxy0, proxy1 }
        } else {
            Self {
                proxy0: proxy1,
                proxy1: proxy0,
            }
        }
    }
}

/// `btOverlapFilterCallback` (`btOverlappingPairCache.h:37-44`) -- the hook
/// MoveIt installs to cull pairs the ACM already excludes.
pub trait OverlapFilterCallback {
    /// `needBroadphaseCollision(proxy0, proxy1)`: `true` when the pair should
    /// be kept.
    ///
    /// The arena is passed through because a handle is not self-describing;
    /// upstream's callback reads the same fields off the two pointers, and
    /// MoveIt's reads `m_clientObject` to reach its collision object.
    fn need_broadphase_collision(
        &self,
        proxies: &dyn PairProxies,
        proxy0: ProxyHandle,
        proxy1: ProxyHandle,
    ) -> bool;
}

/// `btAlignedObjectArray<btBroadphasePair>`, reduced to the four operations
/// the cache performs on it and -- the point of the type -- to its *capacity*.
///
/// `capacity` is not storage bookkeeping here, it is the hash mask:
/// `getHash(...) & (m_overlappingPairArray.capacity() - 1)`. Upstream's array
/// grows by `allocSize(size) = size ? size * 2 : 1` from the constructor's
/// `reserve(2)`, so the sequence is 2, 4, 8, 16, ... and every value is a
/// power of two, which is what makes `& (capacity - 1)` a mask at all. A
/// `Vec`'s capacity follows its allocator instead and would put pairs in
/// different buckets, so it is tracked explicitly and `Vec` is left to hold
/// nothing but the elements.
#[derive(Clone, Debug, Default)]
struct PairArray {
    data: Vec<BroadphasePair>,
    capacity: i32,
}

impl PairArray {
    /// `allocSize(size)` (`btAlignedObjectArray.h:68-71`).
    fn alloc_size(size: i32) -> i32 {
        if size != 0 { size * 2 } else { 1 }
    }

    /// `size()`.
    fn size(&self) -> i32 {
        i32::try_from(self.data.len()).expect("fewer than i32::MAX overlapping pairs")
    }

    /// `capacity()`.
    fn capacity(&self) -> i32 {
        self.capacity
    }

    /// `reserve(_Count)` (`btAlignedObjectArray.h:280-299`).
    fn reserve(&mut self, count: i32) {
        if self.capacity < count {
            self.data
                .reserve(usize::try_from(count).expect("a positive reservation"));
            self.capacity = count;
        }
    }

    /// `expandNonInitializing()` (`btAlignedObjectArray.h:230-240`), returning
    /// the index of the new slot rather than a reference to it.
    ///
    /// Upstream hands back raw memory that `internalAddPair` placement-news
    /// into *after* it may have called `growTables`; the placeholder pushed
    /// here stands in for that uninitialized slot. `growTables` re-hashes
    /// `m_hashTable.size()` pairs, which is the count *before* this call, so
    /// it never reads the placeholder.
    fn expand_non_initializing(&mut self) -> usize {
        let sz = self.size();
        if sz == self.capacity() {
            self.reserve(Self::alloc_size(sz));
        }
        self.data.push(BroadphasePair::NULL);
        usize::try_from(sz).expect("a non-negative size")
    }

    /// `pop_back()`.
    fn pop_back(&mut self) {
        self.data.pop().expect("pop_back on a non-empty array");
    }
}

/// `btHashedOverlappingPairCache` (`btOverlappingPairCache.h:87-254`).
#[derive(Default)]
pub struct HashedOverlappingPairCache {
    /// `m_overlappingPairArray`.
    overlapping_pair_array: PairArray,
    /// `m_overlapFilterCallback`.
    overlap_filter_callback: Option<Box<dyn OverlapFilterCallback>>,
    /// `m_hashTable` -- bucket head indices into the pair array.
    hash_table: Vec<i32>,
    /// `m_next` -- the chain link out of pair `i`.
    next: Vec<i32>,
}

impl HashedOverlappingPairCache {
    /// `btHashedOverlappingPairCache::btHashedOverlappingPairCache`
    /// (`btOverlappingPairCache.cpp:24-30`).
    ///
    /// The `reserve(2)` before `growTables()` is what sets the first mask: it
    /// is the reason a cache holding one pair hashes modulo 2 rather than
    /// modulo 1.
    #[must_use]
    pub fn new() -> Self {
        let mut cache = Self::default();
        let initial_allocated_size = 2;
        cache.overlapping_pair_array.reserve(initial_allocated_size);
        cache.grow_tables_initial();
        cache
    }

    /// `getHash(proxyId1, proxyId2)` (`btOverlappingPairCache.h:203-215`) --
    /// Thomas Wang's integer hash over the two ids packed into one word.
    ///
    /// The pack is `proxyId1 | (proxyId2 << 16)`, so ids above 65535 alias:
    /// upstream's comment on the commented-out variant above it says the
    /// assumption outright. It is not a defect the port may fix -- the
    /// aliasing costs a longer bucket chain and nothing else, because
    /// `internalFindPair` compares the ids themselves.
    #[must_use]
    pub fn get_hash(proxy_id1: u32, proxy_id2: u32) -> u32 {
        let mut key = proxy_id1 | (proxy_id2 << 16);
        key = key.wrapping_add(!(key << 15));
        key ^= key >> 10;
        key = key.wrapping_add(key << 3);
        key ^= key >> 6;
        key = key.wrapping_add(!(key << 11));
        key ^= key >> 16;
        key
    }

    /// `getHash(...) & (m_overlappingPairArray.capacity() - 1)`, the bucket
    /// index, as it is spelled at all four of its call sites.
    /// The two `int`-to-`unsigned` conversions and the one back are C++'s own
    /// (`static_cast<int>(getHash(static_cast<unsigned int>(proxyId1), ...) &
    /// (capacity - 1))`), and are wrapping reinterpretations on both sides.
    fn bucket(&self, proxy_id1: i32, proxy_id2: i32) -> usize {
        let mask = (self.overlapping_pair_array.capacity() - 1) as u32;
        let hash = Self::get_hash(proxy_id1 as u32, proxy_id2 as u32) & mask;
        hash as usize
    }

    /// `equalsPair(pair, proxyId1, proxyId2)`
    /// (`btOverlappingPairCache.h:182-185`) -- unsorted, as upstream's comment
    /// at `internalFindPair` notes.
    fn equals_pair(
        &self,
        proxies: &dyn PairProxies,
        index: usize,
        proxy_id1: i32,
        proxy_id2: i32,
    ) -> bool {
        let pair = self.overlapping_pair_array.data[index];
        proxies.proxy(pair.proxy0).get_uid() == proxy_id1
            && proxies.proxy(pair.proxy1).get_uid() == proxy_id2
    }

    /// `internalFindPair(proxy0, proxy1, hash)`
    /// (`btOverlappingPairCache.h:217-241`), taking the two ids the caller
    /// already read rather than the proxies it read them from.
    fn internal_find_pair(
        &self,
        proxies: &dyn PairProxies,
        proxy_id1: i32,
        proxy_id2: i32,
        hash: usize,
    ) -> Option<usize> {
        let mut index = self.hash_table[hash];
        while index != NULL_PAIR {
            let at = usize::try_from(index).expect("a non-negative chain index");
            if self.equals_pair(proxies, at, proxy_id1, proxy_id2) {
                return Some(at);
            }
            index = self.next[at];
        }
        None
    }

    /// `needsBroadphaseCollision(proxy0, proxy1)`
    /// (`btOverlappingPairCache.h:108-117`).
    ///
    /// The built-in test is asymmetric in its two halves and symmetric as a
    /// whole: each proxy's group must be in the other's mask.
    #[must_use]
    pub fn needs_broadphase_collision(
        &self,
        proxies: &dyn PairProxies,
        proxy0: ProxyHandle,
        proxy1: ProxyHandle,
    ) -> bool {
        if let Some(callback) = self.overlap_filter_callback.as_ref() {
            return callback.need_broadphase_collision(proxies, proxy0, proxy1);
        }

        let a = proxies.proxy(proxy0);
        let b = proxies.proxy(proxy1);
        let mut collides = a.collision_filter_group.intersects(b.collision_filter_mask);
        collides = collides && b.collision_filter_group.intersects(a.collision_filter_mask);
        collides
    }

    /// `addOverlappingPair(proxy0, proxy1)`
    /// (`btOverlappingPairCache.h:121-127`), returning the pair's index rather
    /// than a pointer to it.
    ///
    /// `None` is upstream's null return: the filter rejected the pair.
    pub fn add_overlapping_pair(
        &mut self,
        proxies: &dyn PairProxies,
        proxy0: ProxyHandle,
        proxy1: ProxyHandle,
    ) -> Option<usize> {
        if !self.needs_broadphase_collision(proxies, proxy0, proxy1) {
            return None;
        }
        Some(self.internal_add_pair(proxies, proxy0, proxy1))
    }

    /// `internalAddPair(proxy0, proxy1)`
    /// (`btOverlappingPairCache.cpp:174-227`).
    ///
    /// The hash is taken twice on the growing path and that is not redundant:
    /// the first is against the old capacity, to look the pair up, and the
    /// second against the new one, because `growTables` changed the mask under
    /// it.
    fn internal_add_pair(
        &mut self,
        proxies: &dyn PairProxies,
        proxy0: ProxyHandle,
        proxy1: ProxyHandle,
    ) -> usize {
        let (proxy0, proxy1) = if proxies.proxy(proxy0).unique_id > proxies.proxy(proxy1).unique_id
        {
            (proxy1, proxy0)
        } else {
            (proxy0, proxy1)
        };
        let proxy_id1 = proxies.proxy(proxy0).get_uid();
        let proxy_id2 = proxies.proxy(proxy1).get_uid();

        let mut hash = self.bucket(proxy_id1, proxy_id2);
        if let Some(index) = self.internal_find_pair(proxies, proxy_id1, proxy_id2, hash) {
            return index;
        }

        let count = self.overlapping_pair_array.size();
        let old_capacity = self.overlapping_pair_array.capacity();
        let index = self.overlapping_pair_array.expand_non_initializing();
        let new_capacity = self.overlapping_pair_array.capacity();

        if old_capacity < new_capacity {
            self.grow_tables(proxies);
            hash = self.bucket(proxy_id1, proxy_id2);
        }

        self.overlapping_pair_array.data[index] = BroadphasePair::new(proxies, proxy0, proxy1);

        let count = usize::try_from(count).expect("a non-negative pair count");
        self.next[count] = self.hash_table[hash];
        self.hash_table[hash] = i32::try_from(count).expect("fewer than i32::MAX pairs");

        index
    }

    /// `growTables()` (`btOverlappingPairCache.cpp:137-172`) as the
    /// constructor reaches it, with no pairs to re-hash.
    fn grow_tables_initial(&mut self) {
        let new_capacity = self.overlapping_pair_array.capacity();
        let new_capacity = usize::try_from(new_capacity).expect("a positive capacity");
        if self.hash_table.len() < new_capacity {
            self.hash_table.resize(new_capacity, NULL_PAIR);
            self.next.resize(new_capacity, NULL_PAIR);
            self.hash_table.fill(NULL_PAIR);
            self.next.fill(NULL_PAIR);
        }
    }

    /// `growTables()` (`btOverlappingPairCache.cpp:137-172`).
    ///
    /// The re-hash loop runs to `curHashtableSize` -- the table's *old* length
    /// -- and not to the pair array's length, which is one larger at the only
    /// call site that reaches this. That is exact rather than lucky: growth
    /// happens when and only when `size == capacity`, and the table's length
    /// is that same capacity, so the old length is precisely the number of
    /// initialized pairs.
    fn grow_tables(&mut self, proxies: &dyn PairProxies) {
        let new_capacity =
            usize::try_from(self.overlapping_pair_array.capacity()).expect("a positive capacity");

        if self.hash_table.len() < new_capacity {
            let cur_hashtable_size = self.hash_table.len();

            self.hash_table.resize(new_capacity, NULL_PAIR);
            self.next.resize(new_capacity, NULL_PAIR);
            self.hash_table.fill(NULL_PAIR);
            self.next.fill(NULL_PAIR);

            for i in 0..cur_hashtable_size {
                let pair = self.overlapping_pair_array.data[i];
                let proxy_id1 = proxies.proxy(pair.proxy0).get_uid();
                let proxy_id2 = proxies.proxy(pair.proxy1).get_uid();
                let hash_value = self.bucket(proxy_id1, proxy_id2);
                self.next[i] = self.hash_table[hash_value];
                self.hash_table[hash_value] = i32::try_from(i).expect("fewer than i32::MAX pairs");
            }
        }
    }

    /// `findPair(proxy0, proxy1)` (`btOverlappingPairCache.cpp:102-133`),
    /// returning the pair's index.
    ///
    /// Upstream's `if (hash >= m_hashTable.size()) return NULL` guard is kept
    /// as a debug assertion instead: the table's length is the capacity the
    /// mask was taken from, so the bucket is always in range, and a `Vec`
    /// index would panic rather than answer "not found" if that ever stopped
    /// holding.
    #[must_use]
    pub fn find_pair(
        &self,
        proxies: &dyn PairProxies,
        proxy0: ProxyHandle,
        proxy1: ProxyHandle,
    ) -> Option<usize> {
        let (proxy0, proxy1) = if proxies.proxy(proxy0).unique_id > proxies.proxy(proxy1).unique_id
        {
            (proxy1, proxy0)
        } else {
            (proxy0, proxy1)
        };
        let proxy_id1 = proxies.proxy(proxy0).get_uid();
        let proxy_id2 = proxies.proxy(proxy1).get_uid();

        let hash = self.bucket(proxy_id1, proxy_id2);
        debug_assert!(hash < self.hash_table.len());
        self.internal_find_pair(proxies, proxy_id1, proxy_id2, hash)
    }

    /// `removeOverlappingPair(proxy0, proxy1, dispatcher)`
    /// (`btOverlappingPairCache.cpp:229-329`). `true` when a pair was found
    /// and removed.
    ///
    /// The last pair is moved into the hole, which is what makes the visit
    /// order depend on removal history rather than on insertion order alone.
    /// Upstream re-hashes that last pair from the ids *as stored*, without the
    /// `m_uniqueId` swap the other three hash sites perform -- its own comment
    /// there reads "missing swap here too, Nat." It needs no swap: a stored
    /// pair is already ordered, so this is the same value the swap would have
    /// produced, and the line is reproduced as written.
    pub fn remove_overlapping_pair(
        &mut self,
        proxies: &dyn PairProxies,
        proxy0: ProxyHandle,
        proxy1: ProxyHandle,
    ) -> bool {
        let (proxy0, proxy1) = if proxies.proxy(proxy0).unique_id > proxies.proxy(proxy1).unique_id
        {
            (proxy1, proxy0)
        } else {
            (proxy0, proxy1)
        };
        let proxy_id1 = proxies.proxy(proxy0).get_uid();
        let proxy_id2 = proxies.proxy(proxy1).get_uid();

        let hash = self.bucket(proxy_id1, proxy_id2);
        let Some(pair_index) = self.internal_find_pair(proxies, proxy_id1, proxy_id2, hash) else {
            return false;
        };

        // `cleanOverlappingPair(*pair, dispatcher)`: nothing to free -- see
        // the module docs on `m_algorithm`.

        let pair_index_i32 = i32::try_from(pair_index).expect("fewer than i32::MAX pairs");

        // Remove the pair from the hash table.
        let mut index = self.hash_table[hash];
        debug_assert!(index != NULL_PAIR);

        let mut previous = NULL_PAIR;
        while index != pair_index_i32 {
            previous = index;
            index = self.next[usize::try_from(index).expect("a non-negative chain index")];
        }

        if previous != NULL_PAIR {
            let previous = usize::try_from(previous).expect("a non-negative chain index");
            self.next[previous] = self.next[pair_index];
        } else {
            self.hash_table[hash] = self.next[pair_index];
        }

        // We now move the last pair into the spot of the pair being removed.
        let last_pair_index = self.overlapping_pair_array.size() - 1;

        // If the removed pair is the last pair, we are done.
        if last_pair_index == pair_index_i32 {
            self.overlapping_pair_array.pop_back();
            return true;
        }

        let last_pair_index = usize::try_from(last_pair_index).expect("a non-negative pair index");
        let last_pair_index_i32 =
            i32::try_from(last_pair_index).expect("fewer than i32::MAX pairs");

        // Remove the last pair from the hash table.
        let last = self.overlapping_pair_array.data[last_pair_index];
        let last_hash = self.bucket(
            proxies.proxy(last.proxy0).get_uid(),
            proxies.proxy(last.proxy1).get_uid(),
        );

        index = self.hash_table[last_hash];
        debug_assert!(index != NULL_PAIR);

        previous = NULL_PAIR;
        while index != last_pair_index_i32 {
            previous = index;
            index = self.next[usize::try_from(index).expect("a non-negative chain index")];
        }

        if previous != NULL_PAIR {
            let previous = usize::try_from(previous).expect("a non-negative chain index");
            self.next[previous] = self.next[last_pair_index];
        } else {
            self.hash_table[last_hash] = self.next[last_pair_index];
        }

        // Copy the last pair into the removed pair's spot.
        self.overlapping_pair_array.data[pair_index] =
            self.overlapping_pair_array.data[last_pair_index];

        // Insert the last pair into the hash table.
        self.next[pair_index] = self.hash_table[last_hash];
        self.hash_table[last_hash] = pair_index_i32;

        self.overlapping_pair_array.pop_back();

        true
    }

    /// `processAllOverlappingPairs(callback, dispatcher)`
    /// (`btOverlappingPairCache.cpp:332-350`).
    ///
    /// `i` is *not* advanced when the callback asks for removal, because the
    /// pair that was moved into that slot has not been visited yet. That is
    /// the whole of the interaction between removal order and visit order.
    ///
    /// The callback is handed a copy of the pair rather than a mutable
    /// reference: upstream's mutable access exists so `cleanOverlappingPair`
    /// can null `m_algorithm`, and this port has no such field.
    pub fn process_all_overlapping_pairs(
        &mut self,
        proxies: &dyn PairProxies,
        callback: &mut dyn FnMut(&dyn PairProxies, &BroadphasePair) -> bool,
    ) {
        let mut i = 0usize;
        while i < self.overlapping_pair_array.data.len() {
            let pair = self.overlapping_pair_array.data[i];
            if callback(proxies, &pair) {
                self.remove_overlapping_pair(proxies, pair.proxy0, pair.proxy1);
            } else {
                i += 1;
            }
        }
    }

    /// `removeOverlappingPairsContainingProxy(proxy, dispatcher)`
    /// (`btOverlappingPairCache.cpp:79-100`).
    pub fn remove_overlapping_pairs_containing_proxy(
        &mut self,
        proxies: &dyn PairProxies,
        proxy: ProxyHandle,
    ) {
        self.process_all_overlapping_pairs(proxies, &mut |_, pair| {
            pair.proxy0 == proxy || pair.proxy1 == proxy
        });
    }

    /// `cleanProxyFromPairs(proxy, dispatcher)`
    /// (`btOverlappingPairCache.cpp:48-77`).
    ///
    /// Upstream's callback frees the cached `btCollisionAlgorithm` of every
    /// pair mentioning `proxy` and always returns `false`, so it removes
    /// nothing. This crate has no algorithm to free, which leaves the
    /// traversal and its `false`. It is kept rather than emptied because the
    /// consumer calls it before `destroyProxy` and the shape of that sequence
    /// is part of what this port has to reproduce.
    pub fn clean_proxy_from_pairs(&mut self, proxies: &dyn PairProxies, proxy: ProxyHandle) {
        let _ = proxy;
        self.process_all_overlapping_pairs(proxies, &mut |_, _| false);
    }

    /// `getOverlappingPairArray()` (`btOverlappingPairCache.h:145-153`).
    #[must_use]
    pub fn overlapping_pair_array(&self) -> &[BroadphasePair] {
        &self.overlapping_pair_array.data
    }

    /// `getNumOverlappingPairs()` (`btOverlappingPairCache.h:172-175`).
    #[must_use]
    pub fn num_overlapping_pairs(&self) -> i32 {
        self.overlapping_pair_array.size()
    }

    /// The pair array's `btAlignedObjectArray::capacity()`, which is the hash
    /// mask plus one. Exposed because it is a fixture field, not because
    /// anything outside this module steers on it.
    #[must_use]
    pub fn pair_array_capacity(&self) -> i32 {
        self.overlapping_pair_array.capacity()
    }

    /// `m_hashTable`. Exposed for the same reason as
    /// [`Self::pair_array_capacity`]: it is what pins
    /// [`Self::get_hash`]'s mix, which the pair order cannot.
    #[must_use]
    pub fn hash_table(&self) -> &[i32] {
        &self.hash_table
    }

    /// `m_next`.
    ///
    /// Entries at and above [`Self::num_overlapping_pairs`] are whatever the
    /// last write left there -- `growTables` clears the whole array, and a
    /// removal never clears the slot it vacates. That is upstream's state and
    /// the fixture asserts it, because a port that tidied up would be a port
    /// that had changed something.
    #[must_use]
    pub fn next_table(&self) -> &[i32] {
        &self.next
    }

    /// `setOverlapFilterCallback(callback)`
    /// (`btOverlappingPairCache.h:167-170`).
    pub fn set_overlap_filter_callback(
        &mut self,
        callback: Option<Box<dyn OverlapFilterCallback>>,
    ) {
        self.overlap_filter_callback = callback;
    }

    /// `getOverlapFilterCallback()` (`btOverlappingPairCache.h:162-165`) --
    /// whether one is installed, which is the only thing this port's callers
    /// can ask of it.
    #[must_use]
    pub fn has_overlap_filter_callback(&self) -> bool {
        self.overlap_filter_callback.is_some()
    }

    /// `hasDeferredRemoval()` (`btOverlappingPairCache.h:243-246`).
    ///
    /// `false`, which is why `btDbvtBroadphase::performDeferredRemoval` has no
    /// body to run and is not ported; see
    /// [`crate::dbvt_broadphase`]'s module docs.
    #[must_use]
    pub fn has_deferred_removal(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A bare arena: the pair cache reads nothing but these fields.
    struct Arena(Vec<BroadphaseProxy>);

    impl PairProxies for Arena {
        fn proxy(&self, handle: ProxyHandle) -> &BroadphaseProxy {
            &self.0[handle.0]
        }
    }

    fn arena(n: i32) -> Arena {
        Arena(
            (0..n)
                .map(|i| BroadphaseProxy {
                    client_object: usize::try_from(i).unwrap(),
                    collision_filter_group: CollisionFilterGroup::DEFAULT,
                    collision_filter_mask: CollisionFilterGroup::ALL,
                    unique_id: i + 1,
                    aabb_min: Vec3::zero(),
                    aabb_max: Vec3::zero(),
                })
                .collect(),
        )
    }

    /// `btAlignedObjectArray`'s growth, which is the hash mask.
    ///
    /// Not a `Vec`'s: this is the sequence 2, 4, 8, 16 from the constructor's
    /// `reserve(2)`, and every step of it is a power of two because
    /// `& (capacity - 1)` is only a mask while that holds.
    #[test]
    fn capacity_doubles_from_two_and_is_always_a_power_of_two() {
        let proxies = arena(64);
        let mut cache = HashedOverlappingPairCache::new();
        assert_eq!(cache.pair_array_capacity(), 2);

        let mut seen = vec![2];
        for step in 0..20usize {
            // Distinct pairs, so every add appends.
            let a = ProxyHandle(step % 32);
            let b = ProxyHandle(32 + step % 32);
            cache.add_overlapping_pair(&proxies, a, b);
            let c = cache.pair_array_capacity();
            if *seen.last().unwrap() != c {
                seen.push(c);
            }
        }

        assert_eq!(seen, vec![2, 4, 8, 16, 32]);
        for c in seen {
            assert!(
                u32::try_from(c).unwrap().is_power_of_two(),
                "capacity {c} is not a power of two"
            );
            assert_eq!(
                cache.hash_table().len(),
                usize::try_from(cache.pair_array_capacity()).unwrap()
            );
        }
    }

    /// The filter's two halves are each one-directional; the conjunction is
    /// what makes it symmetric. A group that is in the other's mask but whose
    /// own mask excludes the other must not collide.
    #[test]
    fn needs_broadphase_collision_requires_both_directions() {
        let mut proxies = arena(2);
        let cache = HashedOverlappingPairCache::new();

        proxies.0[0].collision_filter_group = CollisionFilterGroup::DEFAULT;
        proxies.0[0].collision_filter_mask = CollisionFilterGroup::STATIC;
        proxies.0[1].collision_filter_group = CollisionFilterGroup::STATIC;
        proxies.0[1].collision_filter_mask = CollisionFilterGroup::DEFAULT;
        assert!(cache.needs_broadphase_collision(&proxies, ProxyHandle(0), ProxyHandle(1)));

        // proxy1's group is still in proxy0's mask, but proxy0's group is no
        // longer in proxy1's.
        proxies.0[1].collision_filter_mask = CollisionFilterGroup::KINEMATIC;
        assert!(!cache.needs_broadphase_collision(&proxies, ProxyHandle(0), ProxyHandle(1)));
        assert!(!cache.needs_broadphase_collision(&proxies, ProxyHandle(1), ProxyHandle(0)));
    }

    /// A pair is stored ordered by `m_uniqueId` whichever way round it is
    /// added, and adding it again finds the first rather than appending.
    #[test]
    fn a_pair_is_sorted_by_uid_and_added_once() {
        let proxies = arena(4);
        let mut cache = HashedOverlappingPairCache::new();

        cache.add_overlapping_pair(&proxies, ProxyHandle(2), ProxyHandle(1));
        assert_eq!(cache.num_overlapping_pairs(), 1);
        assert_eq!(cache.overlapping_pair_array()[0].proxy0, ProxyHandle(1));
        assert_eq!(cache.overlapping_pair_array()[0].proxy1, ProxyHandle(2));

        cache.add_overlapping_pair(&proxies, ProxyHandle(1), ProxyHandle(2));
        assert_eq!(cache.num_overlapping_pairs(), 1);
    }

    /// Removal moves the *last* pair into the hole. Nothing else may move.
    #[test]
    fn removal_fills_the_hole_with_the_last_pair() {
        let proxies = arena(8);
        let mut cache = HashedOverlappingPairCache::new();
        for i in 0..4 {
            cache.add_overlapping_pair(&proxies, ProxyHandle(i), ProxyHandle(i + 4));
        }
        let before: Vec<BroadphasePair> = cache.overlapping_pair_array().to_vec();
        assert_eq!(before.len(), 4);

        assert!(cache.remove_overlapping_pair(&proxies, ProxyHandle(1), ProxyHandle(5)));

        let after: Vec<BroadphasePair> = cache.overlapping_pair_array().to_vec();
        assert_eq!(after, vec![before[0], before[3], before[2]]);
        assert!(
            cache
                .find_pair(&proxies, ProxyHandle(1), ProxyHandle(5))
                .is_none()
        );
        for (i, p) in after.iter().enumerate() {
            assert_eq!(cache.find_pair(&proxies, p.proxy0, p.proxy1), Some(i));
        }
    }

    /// Removing the last pair takes the early return, which skips the
    /// re-insertion entirely -- the boundary the case above cannot reach.
    #[test]
    fn removing_the_last_pair_leaves_the_others_in_place() {
        let proxies = arena(8);
        let mut cache = HashedOverlappingPairCache::new();
        for i in 0..4 {
            cache.add_overlapping_pair(&proxies, ProxyHandle(i), ProxyHandle(i + 4));
        }
        let before: Vec<BroadphasePair> = cache.overlapping_pair_array().to_vec();

        assert!(cache.remove_overlapping_pair(&proxies, ProxyHandle(3), ProxyHandle(7)));

        assert_eq!(cache.overlapping_pair_array(), &before[..3]);
        for (i, p) in before[..3].iter().enumerate() {
            assert_eq!(cache.find_pair(&proxies, p.proxy0, p.proxy1), Some(i));
        }
    }

    /// Every pair is still findable after the table has grown and re-hashed
    /// several times -- the property `growTables`' re-hash loop exists for,
    /// and the one that fails if its bound is the pair count rather than the
    /// table's old length.
    #[test]
    fn every_pair_survives_the_growth_rehash() {
        let proxies = arena(64);
        let mut cache = HashedOverlappingPairCache::new();
        let mut added = Vec::new();
        for i in 0..24usize {
            let a = ProxyHandle(i % 32);
            let b = ProxyHandle(32 + (i * 7) % 32);
            if cache.add_overlapping_pair(&proxies, a, b).is_some() {
                added.push((a, b));
            }
        }
        assert!(cache.pair_array_capacity() >= 32);

        for (a, b) in added {
            let found = cache.find_pair(&proxies, a, b).expect("pair still present");
            let pair = cache.overlapping_pair_array()[found];
            let (lo, hi) = if proxies.proxy(a).unique_id < proxies.proxy(b).unique_id {
                (a, b)
            } else {
                (b, a)
            };
            assert_eq!((pair.proxy0, pair.proxy1), (lo, hi));
        }
    }

    /// The callback asking for removal must not advance the cursor: the pair
    /// swapped into that slot has not been seen yet. Removing every pair
    /// mentioning one proxy is the case that fails when it does advance.
    #[test]
    fn process_all_does_not_skip_the_pair_swapped_into_the_hole() {
        let proxies = arena(8);
        let mut cache = HashedOverlappingPairCache::new();
        // Every pair mentions proxy 0, so a cursor that advanced past the
        // swapped-in pair would leave pairs behind.
        for i in 1..6 {
            cache.add_overlapping_pair(&proxies, ProxyHandle(0), ProxyHandle(i));
        }
        assert_eq!(cache.num_overlapping_pairs(), 5);

        cache.remove_overlapping_pairs_containing_proxy(&proxies, ProxyHandle(0));
        assert_eq!(cache.num_overlapping_pairs(), 0);
    }
}
