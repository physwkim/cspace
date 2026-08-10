// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2003-2007 Erwin Coumans  https://bulletphysics.org
// btDbvtBroadphase implementation by Nathanael Presson
// btDbvt implementation by Nathanael Presson
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/BroadphaseCollision/btDbvtBroadphase.h
//   bullet3/src/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp
//   bullet3/src/BulletCollision/BroadphaseCollision/btDbvt.h
//   bullet3/src/BulletCollision/BroadphaseCollision/btDbvt.cpp

//! `btDbvtBroadphase` -- two [`Dbvt`] trees, a staging list that migrates
//! settled proxies from one to the other, and the
//! [`HashedOverlappingPairCache`] every AABB overlap is announced to.
//!
//! # What the consumer needs from it
//!
//! MoveIt's `BulletCastBVHManager` drives exactly six entry points:
//! `createProxy`, `setAabb`, `calculateOverlappingPairs`,
//! `getOverlappingPairCache()->processAllOverlappingPairs`,
//! `cleanProxyFromPairs` and `destroyProxy`, plus a
//! `btOverlapFilterCallback` on the cache. Its callback stops early -- `done`
//! ends the query and `max_contacts` truncates the result -- so the *order*
//! pairs come back in decides which contacts are reported. Everything in this
//! module exists to reproduce that order, not merely the pair set. See
//! [`crate::overlapping_pair_cache`]'s module docs for the cache's half of it;
//! this module's half is the tree traversal order that decides which pair is
//! appended first.
//!
//! # `btDbvt` members that live here
//!
//! [`crate::dbvt`] ports the tree a `btCompoundShape` builds, which reaches
//! `insert`, `update(leaf, volume)`, `remove` and `collideTVNoStackAlloc` and
//! nothing else. A broadphase reaches four more `btDbvt` members, and they are
//! implemented here rather than there:
//!
//! - [`collide_tt_persistent_stack`] (`btDbvt.h:1015-1077`),
//! - [`update_with_velocity_and_margin`] (`btDbvt.cpp:583-590`),
//! - [`signed_expand`] (`btDbvt.h:521-534`),
//! - `DbvtBroadphase::optimize_incremental` (`btDbvt.cpp:511-529`), whose
//!   `m_opath` cursor is `DbvtBroadphase::opath` here -- both private, because
//!   nothing outside this module calls either.
//!
//! That placement is a file-ownership artefact of how this port was split
//! across parallel branches, not a claim about where the code belongs: all
//! four are `btDbvt`'s and reach [`Dbvt`] only through its public surface, so
//! moving them into `dbvt.rs` -- taking `m_opath` with them -- is a pure
//! relocation whenever the branches merge.
//!
//! # `sort()` is a no-op, and that is why this port is possible
//!
//! `optimizeIncremental` descends the tree through
//! `sort(node, m_root)->childs[...]`, and `sort` (`btDbvt.cpp:418-446`)
//! branches on `if (p > n)` -- a comparison of two *heap addresses*, which a
//! Rust port storing nodes in a `Vec` cannot reproduce and which C++ leaves
//! unspecified for unrelated pointers in the first place.
//!
//! It does not have to reproduce it. `sort` rotates `n` above its parent `p`
//! and swaps their volumes, which exchanges the two nodes' *identities* and
//! leaves the tree's shape, volumes and leaf set exactly as they were: it is a
//! memory-layout optimisation that puts parents at lower addresses, and it
//! returns whichever node now sits where `n` sat, so the `->childs[...]` that
//! follows selects the same subtree either way. Nothing else in the broadphase
//! compares node addresses -- `collideTT`'s `p.a == p.b` and `indexof`'s
//! `parent->childs[1] == node` are both structural.
//!
//! That is an argument, so it was measured: the whole scenario set this
//! module's fixture is built from was run against the real
//! `btDbvtBroadphase` under three allocators -- glibc `malloc`, a bump
//! allocator with increasing addresses, and one with *decreasing* addresses,
//! which inverts every decision `sort` can make -- and all three produced
//! byte-identical pair rows. So the descent is ported with `sort` elided and
//! the elision is not an approximation.
//!
//! # Proxies are arena slots, and slots are never reused
//!
//! Upstream `btAlignedAlloc`s each `btDbvtProxy` and `btAlignedFree`s it in
//! `destroyProxy`. Here [`ProxyArena`] appends and never reclaims. Nothing
//! observable depends on the difference: no code compares proxy addresses for
//! order (only for identity), and `destroyProxy` purges every pair mentioning
//! the proxy before releasing it, so no live reference to a destroyed handle
//! can exist for a reused slot to alias. Using a destroyed handle is upstream's
//! use-after-free; here it reads a stale proxy instead of undefined memory,
//! which is not better, only quieter.
//!
//! # Deliberately absent
//!
//! `performDeferredRemoval` is not ported. Its entire body is guarded by
//! `m_paircache->hasDeferredRemoval()`, which
//! [`HashedOverlappingPairCache::has_deferred_removal`] answers `false` and
//! which no other cache in this crate can answer differently, so it is
//! `calculateOverlappingPairs`'s tail and does nothing. Porting it would mean
//! porting `btBroadphasePairSortPredicate` -- which reads the `m_algorithm`
//! this crate has no way to set -- and `btAlignedObjectArray::quickSort`, to
//! stand behind an `if (false)`.
//!
//! `rayTest`, `aabbTest`, `getBroadphaseAabb`, `optimize` (the top-down
//! rebuild), `printStats`, `resetPool`, `setAabbForceUpdate` and `benchmark`
//! are likewise unported: none is on the continuous path, and `resetPool`'s
//! body runs only when the broadphase is already empty.
//!
//! `m_deferedcollide` is `false` for the whole life of every broadphase this
//! crate builds -- the constructor sets it and nothing writes it again -- so
//! `collide`'s two `collideTTpersistentStack` calls over the whole trees are
//! dead. They are noted at the site rather than written, because writing them
//! behind a field that cannot become `true` is the same dead code with more
//! surface.

use crate::broadphase_proxy::{BroadphaseNativeType, CollisionFilterGroup};
use crate::dbvt::{Dbvt, DbvtVolume, intersect};
use crate::linear_math::{Scalar, Vec3};
use crate::overlapping_pair_cache::{
    BroadphasePair, BroadphaseProxy, HashedOverlappingPairCache, OverlapFilterCallback,
    PairProxies, ProxyHandle,
};

/// `gDbvtMargin` (`btDbvtBroadphase.cpp:20`) -- the slack `setAabb` grows a
/// moving proxy's volume by, so small motions do not re-insert the leaf.
pub const DBVT_MARGIN: Scalar = 0.05;

/// `DYNAMIC_SET` (`btDbvtBroadphase.h:67`).
pub const DYNAMIC_SET: usize = 0;
/// `FIXED_SET` (`btDbvtBroadphase.h:68`).
pub const FIXED_SET: usize = 1;
/// `STAGECOUNT` (`btDbvtBroadphase.h:69`) -- also the stage number a proxy
/// carries once it has migrated to the fixed set, which is why
/// `m_stageRoots` has `STAGECOUNT + 1` entries.
pub const STAGECOUNT: i32 = 2;

/// `btDbvtAabbMm::SignedExpand(e)` (`btDbvt.h:521-534`).
///
/// Grows the box along each axis in the direction `e` points, so a velocity
/// hint extends the leading face only. `e[i] > 0` is a strict test, so a
/// negative zero grows the *minimum* by `-0.0`, which is the identity.
pub fn signed_expand(volume: &mut DbvtVolume, e: Vec3) {
    if e.x > 0.0 {
        volume.mx.x += e.x;
    } else {
        volume.mi.x += e.x;
    }
    if e.y > 0.0 {
        volume.mx.y += e.y;
    } else {
        volume.mi.y += e.y;
    }
    if e.z > 0.0 {
        volume.mx.z += e.z;
    } else {
        volume.mi.z += e.z;
    }
}

/// `btDbvt::update(leaf, volume, velocity, margin)`
/// (`btDbvt.cpp:583-590`). `true` when the leaf was re-inserted.
///
/// The early `Contain` is the whole optimisation: a leaf whose stored volume
/// already covers the new AABB keeps its place in the tree, and the caller
/// treats that as "no new overlaps to look for".
pub fn update_with_velocity_and_margin(
    tree: &mut Dbvt,
    leaf: usize,
    mut volume: DbvtVolume,
    velocity: Vec3,
    margin: Scalar,
) -> bool {
    if tree.node(leaf).volume.contain(&volume) {
        return false;
    }
    volume.expand(Vec3::new(margin, margin, margin));
    signed_expand(&mut volume, velocity);
    tree.update(leaf, volume);
    true
}

/// `btDbvt::collideTTpersistentStack(root0, root1, policy)`
/// (`btDbvt.h:1015-1077`), collecting the `(a, b)` leaf pairs the policy would
/// have been handed, in the order it would have been handed them.
///
/// The two roots need not belong to the same tree: `setAabb` walks the fixed
/// set against a leaf of the dynamic one. `same_tree` is what upstream's
/// `p.a == p.b` pointer comparison means once nodes are per-tree indices --
/// two indices are the same node only when they index the same tree. Passing
/// `false` for a self-collision would turn the `a == b` short-circuit into a
/// full self-cross-product, which is why it is a parameter and not an index
/// comparison.
///
/// Upstream's stack persists across calls purely to avoid reallocating; its
/// contents above `depth` are never read, so a local one is the same
/// traversal.
pub fn collide_tt_persistent_stack(
    tree_a: &Dbvt,
    root0: Option<usize>,
    tree_b: &Dbvt,
    root1: Option<usize>,
    same_tree: bool,
    out: &mut Vec<(usize, usize)>,
) {
    let (Some(root0), Some(root1)) = (root0, root1) else {
        return;
    };

    let mut stack: Vec<(usize, usize)> = Vec::with_capacity(128);
    stack.push((root0, root1));
    while let Some((a, b)) = stack.pop() {
        if same_tree && a == b {
            if tree_a.node(a).is_internal() {
                let a0 = tree_a.node(a).child[0].expect("internal node has two children");
                let a1 = tree_a.node(a).child[1].expect("internal node has two children");
                stack.push((a0, a0));
                stack.push((a1, a1));
                stack.push((a0, a1));
            }
        } else if intersect(&tree_a.node(a).volume, &tree_b.node(b).volume) {
            if tree_a.node(a).is_internal() {
                let a0 = tree_a.node(a).child[0].expect("internal node has two children");
                let a1 = tree_a.node(a).child[1].expect("internal node has two children");
                if tree_b.node(b).is_internal() {
                    let b0 = tree_b.node(b).child[0].expect("internal node has two children");
                    let b1 = tree_b.node(b).child[1].expect("internal node has two children");
                    stack.push((a0, b0));
                    stack.push((a1, b0));
                    stack.push((a0, b1));
                    stack.push((a1, b1));
                } else {
                    stack.push((a0, b));
                    stack.push((a1, b));
                }
            } else if tree_b.node(b).is_internal() {
                let b0 = tree_b.node(b).child[0].expect("internal node has two children");
                let b1 = tree_b.node(b).child[1].expect("internal node has two children");
                stack.push((a, b0));
                stack.push((a, b1));
            } else {
                out.push((a, b));
            }
        }
    }
}

/// `btDbvtProxy` (`btDbvtBroadphase.h:43-55`), with its `btBroadphaseProxy`
/// base flattened in as [`DbvtProxy::base`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DbvtProxy {
    /// The `btBroadphaseProxy` half.
    pub base: BroadphaseProxy,
    /// `leaf` -- the node index in whichever of the two sets `stage` says.
    pub leaf: usize,
    /// `links[2]` -- the previous and next entries of the stage list this
    /// proxy is threaded on.
    pub links: [Option<ProxyHandle>; 2],
    /// `stage`. `STAGECOUNT` means the fixed set; anything below it is a
    /// dynamic-set stage number and indexes `m_stageRoots` directly.
    pub stage: i32,
}

/// The proxy arena. Upstream is one `btAlignedAlloc` per proxy; see the module
/// docs for why slots here are never reused.
#[derive(Clone, Debug, Default)]
pub struct ProxyArena {
    proxies: Vec<DbvtProxy>,
}

impl ProxyArena {
    /// The proxy `handle` names.
    ///
    /// # Panics
    ///
    /// If `handle` is not one this arena issued.
    #[must_use]
    pub fn get(&self, handle: ProxyHandle) -> &DbvtProxy {
        &self.proxies[handle.0]
    }

    /// How many proxies have ever been created, destroyed ones included.
    #[must_use]
    pub fn len(&self) -> usize {
        self.proxies.len()
    }

    /// Whether no proxy has ever been created.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.proxies.is_empty()
    }

    fn get_mut(&mut self, handle: ProxyHandle) -> &mut DbvtProxy {
        &mut self.proxies[handle.0]
    }
}

impl PairProxies for ProxyArena {
    fn proxy(&self, handle: ProxyHandle) -> &BroadphaseProxy {
        &self.proxies[handle.0].base
    }
}

/// `btDbvtBroadphase` (`btDbvtBroadphase.h:62-144`).
#[derive(Default)]
pub struct DbvtBroadphase {
    /// `m_sets[2]`.
    sets: [Dbvt; 2],
    /// `btDbvt::m_opath`, one per set -- see the module docs on why it is
    /// here rather than in [`Dbvt`].
    opath: [u32; 2],
    /// `m_stageRoots[STAGECOUNT + 1]`.
    stage_roots: [Option<ProxyHandle>; 3],
    /// The proxies themselves, which upstream reaches through the pointers
    /// stored in tree nodes, stage lists and pairs.
    proxies: ProxyArena,
    /// `m_paircache`. Always this crate's own: upstream's constructor takes
    /// one and allocates a `btHashedOverlappingPairCache` when passed null,
    /// and MoveIt passes null.
    paircache: HashedOverlappingPairCache,
    /// `m_prediction`.
    prediction: Scalar,
    /// `m_stageCurrent`.
    stage_current: i32,
    /// `m_fupdates`.
    fupdates: i32,
    /// `m_dupdates`.
    dupdates: i32,
    /// `m_cupdates`.
    cupdates: i32,
    /// `m_newpairs` -- how many times the tree collider announced a pair since
    /// the last `collide`, counted per announcement rather than per pair
    /// actually created, exactly as upstream's `++pbp->m_newpairs` does.
    newpairs: i32,
    /// `m_fixedleft`.
    fixedleft: i32,
    /// `m_updates_call`.
    updates_call: u32,
    /// `m_updates_done`.
    updates_done: u32,
    /// `m_updates_ratio`.
    updates_ratio: Scalar,
    /// `m_pid`.
    pid: i32,
    /// `m_cid` -- where the next incremental cleanup sweep starts.
    cid: i32,
    /// `m_gid` -- the unique-id counter.
    gid: i32,
    /// `m_deferedcollide`.
    deferedcollide: bool,
    /// `m_needcleanup`.
    needcleanup: bool,
}

impl DbvtBroadphase {
    /// `btDbvtBroadphase::btDbvtBroadphase(paircache = 0)`
    /// (`btDbvtBroadphase.cpp:131-162`).
    ///
    /// `m_needcleanup` starts `true` and `m_newpairs` starts at `1`, which is
    /// why the first `calculateOverlappingPairs` sweeps one pair even before
    /// anything has moved.
    #[must_use]
    pub fn new() -> Self {
        Self {
            sets: [Dbvt::new(), Dbvt::new()],
            opath: [0, 0],
            stage_roots: [None; 3],
            proxies: ProxyArena::default(),
            paircache: HashedOverlappingPairCache::new(),
            prediction: 0.0,
            stage_current: 0,
            fupdates: 1,
            dupdates: 0,
            cupdates: 10,
            newpairs: 1,
            fixedleft: 0,
            updates_call: 0,
            updates_done: 0,
            updates_ratio: 0.0,
            pid: 0,
            cid: 0,
            gid: 0,
            deferedcollide: false,
            needcleanup: true,
        }
    }

    /// `getOverlappingPairCache()` (`btDbvtBroadphase.cpp:640-643`).
    #[must_use]
    pub fn overlapping_pair_cache(&self) -> &HashedOverlappingPairCache {
        &self.paircache
    }

    /// The proxy arena, for a caller that needs to read back what it created.
    #[must_use]
    pub fn proxies(&self) -> &ProxyArena {
        &self.proxies
    }

    /// `getOverlappingPairCache()` (`btDbvtBroadphase.cpp:634-637`), split so
    /// that the cache can be driven while the arena it dereferences stays
    /// readable.
    ///
    /// Upstream hands back one pointer and the proxies are reachable through
    /// the pairs; here the two are separate borrows of the same broadphase, so
    /// they have to come out together. This is what
    /// `processAllOverlappingPairs` is called through.
    pub fn pair_cache_and_proxies(&mut self) -> (&mut HashedOverlappingPairCache, &ProxyArena) {
        (&mut self.paircache, &self.proxies)
    }

    /// `getOverlappingPairCache()->setOverlapFilterCallback(callback)`.
    pub fn set_overlap_filter_callback(
        &mut self,
        callback: Option<Box<dyn OverlapFilterCallback>>,
    ) {
        self.paircache.set_overlap_filter_callback(callback);
    }

    /// `setVelocityPrediction(prediction)` (`btDbvtBroadphase.h:128-131`).
    ///
    /// Ported because `setAabb` reads it; nothing in MoveIt calls it, so it
    /// holds its constructed `0` and the velocity term of every update is the
    /// zero vector.
    pub fn set_velocity_prediction(&mut self, prediction: Scalar) {
        self.prediction = prediction;
    }

    /// `getVelocityPrediction()` (`btDbvtBroadphase.h:132-135`).
    #[must_use]
    pub fn velocity_prediction(&self) -> Scalar {
        self.prediction
    }

    /// `getAabb(proxy, aabbMin, aabbMax)` (`btDbvtBroadphase.cpp:219-224`) --
    /// the proxy's own stored AABB, not its leaf's volume, which `setAabb`
    /// may have grown by the margin.
    #[must_use]
    pub fn get_aabb(&self, proxy: ProxyHandle) -> (Vec3, Vec3) {
        let p = self.proxies.get(proxy);
        (p.base.aabb_min, p.base.aabb_max)
    }

    /// `listappend(item, list)` (`btDbvtBroadphase.cpp:53-60`), with `list`
    /// named by its index in `m_stageRoots`.
    fn list_append(&mut self, item: ProxyHandle, list: usize) {
        self.proxies.get_mut(item).links[0] = None;
        self.proxies.get_mut(item).links[1] = self.stage_roots[list];
        if let Some(head) = self.stage_roots[list] {
            self.proxies.get_mut(head).links[0] = Some(item);
        }
        self.stage_roots[list] = Some(item);
    }

    /// `listremove(item, list)` (`btDbvtBroadphase.cpp:63-71`).
    fn list_remove(&mut self, item: ProxyHandle, list: usize) {
        let links = self.proxies.get(item).links;
        if let Some(prev) = links[0] {
            self.proxies.get_mut(prev).links[1] = links[1];
        } else {
            self.stage_roots[list] = links[1];
        }
        if let Some(next) = links[1] {
            self.proxies.get_mut(next).links[0] = links[0];
        }
    }

    /// The set a proxy's `leaf` indexes, which its `stage` decides --
    /// `destroyProxy` reads the same rule (`btDbvtBroadphase.cpp:209-212`).
    fn set_of(&self, proxy: ProxyHandle) -> usize {
        if self.proxies.get(proxy).stage == STAGECOUNT {
            FIXED_SET
        } else {
            DYNAMIC_SET
        }
    }

    /// `btDbvtTreeCollider::Process(na, nb)`
    /// (`btDbvtBroadphase.cpp:106-119`) over an already-collected list of node
    /// pairs.
    ///
    /// Collecting first and announcing after is not a reordering: the
    /// traversals read only the trees and the collider writes only the pair
    /// cache and `m_newpairs`, so no announcement can affect a later visit.
    /// It is what lets one `&self.sets[..]` borrow finish before
    /// `&mut self.paircache` begins.
    fn announce(&mut self, hits: &[(usize, usize)], set_a: usize, set_b: usize) {
        for &(na, nb) in hits {
            if set_a == set_b && na == nb {
                continue;
            }
            let pa = ProxyHandle(
                usize::try_from(self.sets[set_a].node(na).data).expect("a proxy handle"),
            );
            let pb = ProxyHandle(
                usize::try_from(self.sets[set_b].node(nb).data).expect("a proxy handle"),
            );
            self.paircache.add_overlapping_pair(&self.proxies, pa, pb);
            self.newpairs += 1;
        }
    }

    /// `createProxy(aabbMin, aabbMax, shapeType, userPtr, group, mask,
    /// dispatcher)` (`btDbvtBroadphase.cpp:175-202`).
    ///
    /// `shape_type` is upstream's unnamed parameter -- the broadphase never
    /// reads it -- and the dispatcher argument is dropped: this crate has no
    /// `btDispatcher`, and every use of one on this path is freeing a
    /// collision algorithm that cannot exist here.
    pub fn create_proxy(
        &mut self,
        aabb_min: Vec3,
        aabb_max: Vec3,
        shape_type: BroadphaseNativeType,
        user_ptr: usize,
        collision_filter_group: CollisionFilterGroup,
        collision_filter_mask: CollisionFilterGroup,
    ) -> ProxyHandle {
        let _ = shape_type;

        let handle = ProxyHandle(self.proxies.proxies.len());
        let aabb = DbvtVolume::from_mm(aabb_min, aabb_max);

        self.gid += 1;
        self.proxies.proxies.push(DbvtProxy {
            base: BroadphaseProxy {
                client_object: user_ptr,
                collision_filter_group,
                collision_filter_mask,
                unique_id: self.gid,
                aabb_min,
                aabb_max,
            },
            leaf: 0,
            links: [None, None],
            stage: self.stage_current,
        });

        let leaf = self.sets[DYNAMIC_SET].insert(
            aabb,
            i32::try_from(handle.0).expect("fewer than i32::MAX proxies"),
        );
        self.proxies.get_mut(handle).leaf = leaf;
        self.list_append(
            handle,
            usize::try_from(self.stage_current).expect("a non-negative stage"),
        );

        if !self.deferedcollide {
            let leaf_of_new = self.proxies.get(handle).leaf;
            for set in [DYNAMIC_SET, FIXED_SET] {
                // `collideTV(m_sets[set].m_root, aabb, collider)`, whose
                // `Process(n)` is `Process(n, proxy->leaf)`.
                let mut stack = Vec::new();
                let mut hits = Vec::new();
                let root = self.sets[set].root;
                self.sets[set].collide_tv_no_stack_alloc(root, &aabb, &mut stack, &mut |_, n| {
                    hits.push((n, leaf_of_new));
                });
                self.announce(&hits, set, DYNAMIC_SET);
            }
        }

        handle
    }

    /// `destroyProxy(proxy, dispatcher)` (`btDbvtBroadphase.cpp:205-217`).
    pub fn destroy_proxy(&mut self, proxy: ProxyHandle) {
        let stage = self.proxies.get(proxy).stage;
        let leaf = self.proxies.get(proxy).leaf;
        if stage == STAGECOUNT {
            self.sets[FIXED_SET].remove(leaf);
        } else {
            self.sets[DYNAMIC_SET].remove(leaf);
        }
        self.list_remove(proxy, usize::try_from(stage).expect("a non-negative stage"));
        self.paircache
            .remove_overlapping_pairs_containing_proxy(&self.proxies, proxy);
        self.needcleanup = true;
    }

    /// `setAabb(proxy, aabbMin, aabbMax, dispatcher)`
    /// (`btDbvtBroadphase.cpp:311-373`).
    ///
    /// `DBVT_BP_PREVENTFALSEUPDATE` is `0`, so the `NotEqual` guard around the
    /// whole body is compiled out upstream and is not written here.
    ///
    /// The velocity hint is built from the proxy's *old* AABB and its sign
    /// comes from the motion of the minimum corner; with `m_prediction` at its
    /// constructed zero it is the zero vector and [`signed_expand`] leaves the
    /// volume alone, which is the case MoveIt runs in.
    pub fn set_aabb(&mut self, proxy: ProxyHandle, aabb_min: Vec3, aabb_max: Vec3) {
        let aabb = DbvtVolume::from_mm(aabb_min, aabb_max);
        let mut docollide = false;
        let stage = self.proxies.get(proxy).stage;

        if stage == STAGECOUNT {
            // fixed -> dynamic set
            let leaf = self.proxies.get(proxy).leaf;
            self.sets[FIXED_SET].remove(leaf);
            let leaf = self.sets[DYNAMIC_SET].insert(
                aabb,
                i32::try_from(proxy.0).expect("fewer than i32::MAX proxies"),
            );
            self.proxies.get_mut(proxy).leaf = leaf;
            docollide = true;
        } else {
            // dynamic set
            self.updates_call += 1;
            let leaf = self.proxies.get(proxy).leaf;
            if intersect(&self.sets[DYNAMIC_SET].node(leaf).volume, &aabb) {
                // Moving
                let p = self.proxies.get(proxy);
                let delta = aabb_min - p.base.aabb_min;
                let mut velocity = ((p.base.aabb_max - p.base.aabb_min) / 2.0) * self.prediction;
                if delta.x < 0.0 {
                    velocity.x = -velocity.x;
                }
                if delta.y < 0.0 {
                    velocity.y = -velocity.y;
                }
                if delta.z < 0.0 {
                    velocity.z = -velocity.z;
                }
                if update_with_velocity_and_margin(
                    &mut self.sets[DYNAMIC_SET],
                    leaf,
                    aabb,
                    velocity,
                    DBVT_MARGIN,
                ) {
                    self.updates_done += 1;
                    docollide = true;
                }
            } else {
                // Teleporting
                self.sets[DYNAMIC_SET].update(leaf, aabb);
                self.updates_done += 1;
                docollide = true;
            }
        }

        self.list_remove(proxy, usize::try_from(stage).expect("a non-negative stage"));
        {
            let p = self.proxies.get_mut(proxy);
            p.base.aabb_min = aabb_min;
            p.base.aabb_max = aabb_max;
        }
        self.proxies.get_mut(proxy).stage = self.stage_current;
        self.list_append(
            proxy,
            usize::try_from(self.stage_current).expect("a non-negative stage"),
        );

        if docollide {
            self.needcleanup = true;
            if !self.deferedcollide {
                let leaf = self.proxies.get(proxy).leaf;
                // The fixed set first, then the dynamic one -- the order the
                // two pairs a move creates are appended in.
                for set in [FIXED_SET, DYNAMIC_SET] {
                    let mut hits = Vec::new();
                    collide_tt_persistent_stack(
                        &self.sets[set],
                        self.sets[set].root,
                        &self.sets[DYNAMIC_SET],
                        Some(leaf),
                        set == DYNAMIC_SET,
                        &mut hits,
                    );
                    self.announce(&hits, set, DYNAMIC_SET);
                }
            }
        }
    }

    /// `calculateOverlappingPairs(dispatcher)`
    /// (`btDbvtBroadphase.cpp:417-441`).
    ///
    /// The `performDeferredRemoval` tail is not ported; see the module docs.
    pub fn calculate_overlapping_pairs(&mut self) {
        self.collide();
    }

    /// `btDbvt::optimizeIncremental(passes)` (`btDbvt.cpp:511-529`) on one of
    /// the two sets, with `sort` elided -- see the module docs for why that is
    /// exact rather than approximate.
    ///
    /// Each pass walks the bits of `m_opath` from the root to a leaf and
    /// re-inserts that leaf, which is how the tree keeps rebalancing under a
    /// stream of updates. `update(node)` there is the `lookahead` overload at
    /// its default `-1`, which reinserts from the root; with the volume left
    /// as it is, that is [`Dbvt::update`].
    fn optimize_incremental(&mut self, set: usize, passes: i32) {
        let mut passes = if passes < 0 {
            i32::try_from(self.sets[set].leaves).expect("fewer than i32::MAX leaves")
        } else {
            passes
        };

        if self.sets[set].root.is_some() && passes > 0 {
            loop {
                let mut node = self.sets[set].root.expect("a non-empty tree");
                let mut bit = 0u32;
                while self.sets[set].node(node).is_internal() {
                    let which = ((self.opath[set] >> bit) & 1) as usize;
                    node = self.sets[set].node(node).child[which]
                        .expect("internal node has two children");
                    bit = (bit + 1) & (u32::BITS - 1);
                }
                let volume = self.sets[set].node(node).volume;
                self.sets[set].update(node, volume);
                self.opath[set] = self.opath[set].wrapping_add(1);

                passes -= 1;
                if passes == 0 {
                    break;
                }
            }
        }
    }

    /// `collide(dispatcher)` (`btDbvtBroadphase.cpp:512-624`).
    fn collide(&mut self) {
        // optimize
        let leaves0 = i32::try_from(self.sets[DYNAMIC_SET].leaves).expect("fewer than i32::MAX");
        self.optimize_incremental(DYNAMIC_SET, 1 + (leaves0 * self.dupdates) / 100);
        if self.fixedleft != 0 {
            let leaves1 = i32::try_from(self.sets[FIXED_SET].leaves).expect("fewer than i32::MAX");
            let count = 1 + (leaves1 * self.fupdates) / 100;
            self.optimize_incremental(FIXED_SET, 1 + (leaves1 * self.fupdates) / 100);
            self.fixedleft = std::cmp::max(0, self.fixedleft - count);
        }

        // dynamic -> fixed set
        self.stage_current = (self.stage_current + 1) % STAGECOUNT;
        let mut current =
            self.stage_roots[usize::try_from(self.stage_current).expect("a non-negative stage")];
        if current.is_some() {
            while let Some(cur) = current {
                let next = self.proxies.get(cur).links[1];
                let stage = self.proxies.get(cur).stage;
                self.list_remove(cur, usize::try_from(stage).expect("a non-negative stage"));
                self.list_append(
                    cur,
                    usize::try_from(STAGECOUNT).expect("a non-negative stage"),
                );
                let leaf = self.proxies.get(cur).leaf;
                self.sets[DYNAMIC_SET].remove(leaf);
                let p = self.proxies.get(cur);
                let cur_aabb = DbvtVolume::from_mm(p.base.aabb_min, p.base.aabb_max);
                let leaf = self.sets[FIXED_SET].insert(
                    cur_aabb,
                    i32::try_from(cur.0).expect("fewer than i32::MAX proxies"),
                );
                let p = self.proxies.get_mut(cur);
                p.leaf = leaf;
                p.stage = STAGECOUNT;
                current = next;
            }
            self.fixedleft =
                i32::try_from(self.sets[FIXED_SET].leaves).expect("fewer than i32::MAX leaves");
            self.needcleanup = true;
        }

        // `collide dynamics`: both branches are `if (m_deferedcollide)`, which
        // is `false` for the life of this broadphase. See the module docs.

        // clean up
        if self.needcleanup {
            let size = self.paircache.num_overlapping_pairs();
            if size > 0 {
                let mut ni = std::cmp::min(
                    size,
                    std::cmp::max(self.newpairs, (size * self.cupdates) / 100),
                );
                let mut i = 0i32;
                while i < ni {
                    // `pairs.size()` is re-read every iteration upstream, and
                    // it shrinks as pairs go.
                    let size = self.paircache.num_overlapping_pairs();
                    let at = usize::try_from((self.cid + i) % size).expect("a non-negative index");
                    let p = self.paircache.overlapping_pair_array()[at];
                    if !intersect(&self.leaf_volume(p.proxy0), &self.leaf_volume(p.proxy1)) {
                        self.paircache
                            .remove_overlapping_pair(&self.proxies, p.proxy0, p.proxy1);
                        ni -= 1;
                        i -= 1;
                    }
                    i += 1;
                }
                let size = self.paircache.num_overlapping_pairs();
                self.cid = if size > 0 { (self.cid + ni) % size } else { 0 };
            }
        }

        self.pid += 1;
        self.newpairs = 1;
        self.needcleanup = false;
        self.updates_ratio = if self.updates_call > 0 {
            self.updates_done as Scalar / self.updates_call as Scalar
        } else {
            0.0
        };
        self.updates_done /= 2;
        self.updates_call /= 2;
    }

    /// A proxy's leaf volume, which is `pa->leaf->volume` at the cleanup
    /// sweep -- the volume the *tree* holds, margin and all, not the AABB the
    /// caller last set.
    fn leaf_volume(&self, proxy: ProxyHandle) -> DbvtVolume {
        let set = self.set_of(proxy);
        self.sets[set].node(self.proxies.get(proxy).leaf).volume
    }
}

/// Reads a `Dbvt` the broadphase owns, for tests and fixtures that need to see
/// which set a proxy landed in.
impl DbvtBroadphase {
    /// `m_sets[set]`.
    ///
    /// # Panics
    ///
    /// If `set` is neither [`DYNAMIC_SET`] nor [`FIXED_SET`].
    #[must_use]
    pub fn set(&self, set: usize) -> &Dbvt {
        &self.sets[set]
    }

    /// `m_pid` -- how many times `collide` has run.
    #[must_use]
    pub fn parse_id(&self) -> i32 {
        self.pid
    }

    /// `m_cid` -- where the next cleanup sweep starts.
    #[must_use]
    pub fn cleanup_index(&self) -> i32 {
        self.cid
    }

    /// `m_updates_ratio`.
    #[must_use]
    pub fn updates_ratio(&self) -> Scalar {
        self.updates_ratio
    }
}

/// Walks the pair cache the way MoveIt does, collecting the
/// `(client_object, client_object)` of every pair in visit order.
///
/// Its own function because the fixture, the tests and any consumer all want
/// the same thing -- the sequence, not the set -- and because the borrow split
/// it needs ([`DbvtBroadphase::pair_cache_and_proxies`]) is the non-obvious
/// part of calling `processAllOverlappingPairs` at all.
#[must_use]
pub fn visit_order(broadphase: &mut DbvtBroadphase) -> Vec<(usize, usize)> {
    let mut seen = Vec::new();
    let (cache, proxies) = broadphase.pair_cache_and_proxies();
    cache.process_all_overlapping_pairs(proxies, &mut |proxies, pair: &BroadphasePair| {
        seen.push((
            proxies.proxy(pair.proxy0).client_object,
            proxies.proxy(pair.proxy1).client_object,
        ));
        false
    });
    seen
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::overlapping_pair_cache::BroadphasePair;
    use crate::probe_fixture::row;

    /// The `bp_*` rows of `tools/bullet-epa-reference/build.sh`'s stdout.
    ///
    /// One row per overlapping pair, in the order the real
    /// `processAllOverlappingPairs` handed it to the callback, which is the
    /// output MoveIt's continuous check consumes -- see the module docs of
    /// [`crate::overlapping_pair_cache`] for why the set alone is not enough.
    ///
    /// Row shapes:
    /// - `bp_<phase>|<pairs>|<capacity>` -- the count and the pair array's
    ///   `btAlignedObjectArray::capacity()`, which is the hash mask plus one.
    /// - `bp_<phase>_p<i>|<obj0>|<obj1>|<uid0>|<uid1>` -- pair `i` in visit
    ///   order, each proxy named by its creation index and its `m_uniqueId`.
    /// - `bp_<phase>_ht|<n>|...` and `bp_<phase>_nx|<n>|...` -- `m_hashTable`
    ///   and `m_next`. These are here because the pair order cannot see the
    ///   hash: `internalFindPair` walks a bucket chain comparing unique ids,
    ///   so every self-consistent mix yields the same array and only the
    ///   buckets themselves pin Thomas Wang's.
    const BULLET_REFERENCE: &str = "\
bp_grid_new|0|2
bp_grid_new_ht|2|-1|-1
bp_grid_new_nx|2|-1|-1
bp_grid_c1|0|2
bp_grid_c1_ht|2|-1|-1
bp_grid_c1_nx|2|-1|-1
bp_chain_new|5|8
bp_chain_new_p0|0|1|1|2
bp_chain_new_p1|1|2|2|3
bp_chain_new_p2|2|3|3|4
bp_chain_new_p3|3|4|4|5
bp_chain_new_p4|4|5|5|6
bp_chain_new_ht|8|-1|-1|2|1|-1|4|3|-1
bp_chain_new_nx|8|-1|-1|-1|0|-1|-1|-1|-1
bp_chain_c1|5|8
bp_chain_c1_p0|0|1|1|2
bp_chain_c1_p1|1|2|2|3
bp_chain_c1_p2|2|3|3|4
bp_chain_c1_p3|3|4|4|5
bp_chain_c1_p4|4|5|5|6
bp_chain_c2|5|8
bp_chain_c2_p0|0|1|1|2
bp_chain_c2_p1|1|2|2|3
bp_chain_c2_p2|2|3|3|4
bp_chain_c2_p3|3|4|4|5
bp_chain_c2_p4|4|5|5|6
bp_chain_add|7|8
bp_chain_add_p0|0|1|1|2
bp_chain_add_p1|1|2|2|3
bp_chain_add_p2|2|3|3|4
bp_chain_add_p3|3|4|4|5
bp_chain_add_p4|4|5|5|6
bp_chain_add_p5|2|6|3|7
bp_chain_add_p6|3|6|4|7
bp_chain_add_ht|8|5|6|2|1|-1|4|3|-1
bp_chain_add_nx|8|-1|-1|-1|0|-1|-1|-1|-1
bp_chain_c3|7|8
bp_chain_c3_p0|0|1|1|2
bp_chain_c3_p1|1|2|2|3
bp_chain_c3_p2|2|3|3|4
bp_chain_c3_p3|3|4|4|5
bp_chain_c3_p4|4|5|5|6
bp_chain_c3_p5|2|6|3|7
bp_chain_c3_p6|3|6|4|7
bp_hub_new|5|8
bp_hub_new_p0|4|5|5|6
bp_hub_new_p1|3|5|4|6
bp_hub_new_p2|2|5|3|6
bp_hub_new_p3|1|5|2|6
bp_hub_new_p4|0|5|1|6
bp_hub_new_ht|8|-1|3|-1|-1|1|0|4|-1
bp_hub_new_nx|8|-1|-1|-1|-1|2|-1|-1|-1
bp_hub_c1|5|8
bp_hub_c1_p0|4|5|5|6
bp_hub_c1_p1|3|5|4|6
bp_hub_c1_p2|2|5|3|6
bp_hub_c1_p3|1|5|2|6
bp_hub_c1_p4|0|5|1|6
bp_filter_new|7|8
bp_filter_new_p0|0|1|1|2
bp_filter_new_p1|1|2|2|3
bp_filter_new_p2|2|3|3|4
bp_filter_new_p3|3|4|4|5
bp_filter_new_p4|4|5|5|6
bp_filter_new_p5|5|6|6|7
bp_filter_new_p6|6|7|7|8
bp_filter_new_ht|8|5|-1|6|1|-1|4|3|-1
bp_filter_new_nx|8|-1|-1|-1|0|-1|-1|2|-1
bp_filter_c1|7|8
bp_filter_c1_p0|0|1|1|2
bp_filter_c1_p1|1|2|2|3
bp_filter_c1_p2|2|3|3|4
bp_filter_c1_p3|3|4|4|5
bp_filter_c1_p4|4|5|5|6
bp_filter_c1_p5|5|6|6|7
bp_filter_c1_p6|6|7|7|8
bp_filtercb_new|6|8
bp_filtercb_new_p0|0|2|1|3
bp_filtercb_new_p1|1|3|2|4
bp_filtercb_new_p2|2|4|3|5
bp_filtercb_new_p3|3|5|4|6
bp_filtercb_new_p4|4|6|5|7
bp_filtercb_new_p5|5|7|6|8
bp_filtercb_new_ht|8|2|0|-1|-1|5|1|4|-1
bp_filtercb_new_nx|8|-1|-1|-1|-1|-1|3|-1|-1
bp_filtercb_c1|6|8
bp_filtercb_c1_p0|0|2|1|3
bp_filtercb_c1_p1|1|3|2|4
bp_filtercb_c1_p2|2|4|3|5
bp_filtercb_c1_p3|3|5|4|6
bp_filtercb_c1_p4|4|6|5|7
bp_filtercb_c1_p5|5|7|6|8
bp_move_new|4|4
bp_move_new_p0|0|1|1|2
bp_move_new_p1|1|2|2|3
bp_move_new_p2|2|3|3|4
bp_move_new_p3|3|4|4|5
bp_move_c1|4|4
bp_move_c1_p0|0|1|1|2
bp_move_c1_p1|1|2|2|3
bp_move_c1_p2|2|3|3|4
bp_move_c1_p3|3|4|4|5
bp_move_c2|4|4
bp_move_c2_p0|0|1|1|2
bp_move_c2_p1|1|2|2|3
bp_move_c2_p2|2|3|3|4
bp_move_c2_p3|3|4|4|5
bp_move_tiny|4|4
bp_move_tiny_p0|0|1|1|2
bp_move_tiny_p1|1|2|2|3
bp_move_tiny_p2|2|3|3|4
bp_move_tiny_p3|3|4|4|5
bp_move_tiny_ht|4|-1|-1|3|1
bp_move_tiny_nx|4|-1|-1|0|2
bp_move_c3|4|4
bp_move_c3_p0|0|1|1|2
bp_move_c3_p1|1|2|2|3
bp_move_c3_p2|2|3|3|4
bp_move_c3_p3|3|4|4|5
bp_move_far|6|8
bp_move_far_p0|0|1|1|2
bp_move_far_p1|1|2|2|3
bp_move_far_p2|2|3|3|4
bp_move_far_p3|3|4|4|5
bp_move_far_p4|0|4|1|5
bp_move_far_p5|0|3|1|4
bp_move_far_ht|8|-1|-1|4|1|-1|-1|5|-1
bp_move_far_nx|8|-1|-1|-1|0|2|3|-1|-1
bp_move_c4|5|8
bp_move_c4_p0|0|3|1|4
bp_move_c4_p1|1|2|2|3
bp_move_c4_p2|2|3|3|4
bp_move_c4_p3|3|4|4|5
bp_move_c4_p4|0|4|1|5
bp_move_c4_ht|8|-1|-1|4|1|-1|-1|0|-1
bp_move_c4_nx|8|3|-1|-1|-1|2|3|-1|-1
bp_move_c5|5|8
bp_move_c5_p0|0|3|1|4
bp_move_c5_p1|1|2|2|3
bp_move_c5_p2|2|3|3|4
bp_move_c5_p3|3|4|4|5
bp_move_c5_p4|0|4|1|5
bp_move_c5_ht|8|-1|-1|4|1|-1|-1|0|-1
bp_move_c5_nx|8|3|-1|-1|-1|2|3|-1|-1
bp_destroy_c1|5|8
bp_destroy_c1_p0|0|1|1|2
bp_destroy_c1_p1|1|2|2|3
bp_destroy_c1_p2|2|3|3|4
bp_destroy_c1_p3|3|4|4|5
bp_destroy_c1_p4|4|5|5|6
bp_destroy_c1_ht|8|-1|-1|2|1|-1|4|3|-1
bp_destroy_c1_nx|8|-1|-1|-1|0|-1|-1|-1|-1
bp_destroy_gone|3|8
bp_destroy_gone_p0|0|1|1|2
bp_destroy_gone_p1|4|5|5|6
bp_destroy_gone_p2|3|4|4|5
bp_destroy_gone_ht|8|-1|-1|-1|-1|-1|1|2|-1
bp_destroy_gone_nx|8|-1|-1|0|0|-1|-1|-1|-1
bp_destroy_c2|3|8
bp_destroy_c2_p0|0|1|1|2
bp_destroy_c2_p1|4|5|5|6
bp_destroy_c2_p2|3|4|4|5
bp_destroy_readd|5|8
bp_destroy_readd_p0|0|1|1|2
bp_destroy_readd_p1|4|5|5|6
bp_destroy_readd_p2|3|4|4|5
bp_destroy_readd_p3|1|6|2|7
bp_destroy_readd_p4|3|6|4|7
bp_destroy_readd_ht|8|-1|4|-1|3|-1|1|2|-1
bp_destroy_readd_nx|8|-1|-1|0|-1|-1|-1|-1|-1
bp_grow_new|21|32
bp_grow_new_p0|0|1|1|2
bp_grow_new_p1|1|2|2|3
bp_grow_new_p2|0|2|1|3
bp_grow_new_p3|2|3|3|4
bp_grow_new_p4|1|3|2|4
bp_grow_new_p5|3|4|4|5
bp_grow_new_p6|2|4|3|5
bp_grow_new_p7|4|5|5|6
bp_grow_new_p8|3|5|4|6
bp_grow_new_p9|5|6|6|7
bp_grow_new_p10|4|6|5|7
bp_grow_new_p11|6|7|7|8
bp_grow_new_p12|5|7|6|8
bp_grow_new_p13|7|8|8|9
bp_grow_new_p14|6|8|7|9
bp_grow_new_p15|8|9|9|10
bp_grow_new_p16|7|9|8|10
bp_grow_new_p17|9|10|10|11
bp_grow_new_p18|8|10|9|11
bp_grow_new_p19|10|11|11|12
bp_grow_new_p20|9|11|10|12
bp_grow_new_ht|32|6|17|16|-1|8|15|10|-1|-1|-1|-1|18|12|20|0|-1|9|-1|-1|-1|19|4|5|-1|-1|-1|11|-1|14|-1|-1|-1
bp_grow_new_nx|32|-1|-1|-1|-1|-1|-1|-1|-1|-1|-1|-1|3|-1|-1|13|-1|-1|2|1|-1|7|-1|-1|-1|-1|-1|-1|-1|-1|-1|-1|-1
bp_grow_c1|21|32
bp_grow_c1_p0|0|1|1|2
bp_grow_c1_p1|1|2|2|3
bp_grow_c1_p2|0|2|1|3
bp_grow_c1_p3|2|3|3|4
bp_grow_c1_p4|1|3|2|4
bp_grow_c1_p5|3|4|4|5
bp_grow_c1_p6|2|4|3|5
bp_grow_c1_p7|4|5|5|6
bp_grow_c1_p8|3|5|4|6
bp_grow_c1_p9|5|6|6|7
bp_grow_c1_p10|4|6|5|7
bp_grow_c1_p11|6|7|7|8
bp_grow_c1_p12|5|7|6|8
bp_grow_c1_p13|7|8|8|9
bp_grow_c1_p14|6|8|7|9
bp_grow_c1_p15|8|9|9|10
bp_grow_c1_p16|7|9|8|10
bp_grow_c1_p17|9|10|10|11
bp_grow_c1_p18|8|10|9|11
bp_grow_c1_p19|10|11|11|12
bp_grow_c1_p20|9|11|10|12
bp_grow_c1_ht|32|6|17|16|-1|8|15|10|-1|-1|-1|-1|18|12|20|0|-1|9|-1|-1|-1|19|4|5|-1|-1|-1|11|-1|14|-1|-1|-1
bp_grow_c1_nx|32|-1|-1|-1|-1|-1|-1|-1|-1|-1|-1|-1|3|-1|-1|13|-1|-1|2|1|-1|7|-1|-1|-1|-1|-1|-1|-1|-1|-1|-1|-1
bp_last_c1|3|4
bp_last_c1_p0|0|1|1|2
bp_last_c1_p1|1|2|2|3
bp_last_c1_p2|2|3|3|4
bp_last_c1_ht|4|-1|-1|2|1
bp_last_c1_nx|4|-1|-1|0|-1
bp_last_gone|2|4
bp_last_gone_p0|0|1|1|2
bp_last_gone_p1|1|2|2|3
bp_last_gone_ht|4|-1|-1|0|1
bp_last_gone_nx|4|-1|-1|0|-1
bp_asym_new|2|2
bp_asym_new_p0|1|2|2|3
bp_asym_new_p1|2|3|3|4
bp_asym_new_ht|2|1|0
bp_asym_new_nx|2|-1|-1
bp_asym_c1|2|2
bp_asym_c1_p0|1|2|2|3
bp_asym_c1_p1|2|3|3|4
bp_opath_c6|7|8
bp_opath_c6_p0|0|1|1|2
bp_opath_c6_p1|1|2|2|3
bp_opath_c6_p2|2|3|3|4
bp_opath_c6_p3|3|4|4|5
bp_opath_c6_p4|4|5|5|6
bp_opath_c6_p5|5|6|6|7
bp_opath_c6_p6|6|7|7|8
bp_opath_c6_ht|8|5|-1|6|1|-1|4|3|-1
bp_opath_c6_nx|8|-1|-1|-1|0|-1|-1|2|-1
bp_opath_hub|15|16
bp_opath_hub_p0|0|1|1|2
bp_opath_hub_p1|1|2|2|3
bp_opath_hub_p2|2|3|3|4
bp_opath_hub_p3|3|4|4|5
bp_opath_hub_p4|4|5|5|6
bp_opath_hub_p5|5|6|6|7
bp_opath_hub_p6|6|7|7|8
bp_opath_hub_p7|0|8|1|9
bp_opath_hub_p8|1|8|2|9
bp_opath_hub_p9|2|8|3|9
bp_opath_hub_p10|3|8|4|9
bp_opath_hub_p11|4|8|5|9
bp_opath_hub_p12|5|8|6|9
bp_opath_hub_p13|7|8|8|9
bp_opath_hub_p14|6|8|7|9
bp_opath_hub_ht|16|9|-1|-1|-1|-1|-1|7|-1|10|11|6|8|14|4|0|-1
bp_opath_hub_nx|16|-1|-1|-1|-1|-1|-1|2|3|1|5|-1|-1|-1|12|13|-1
bp_margin_new|2|2
bp_margin_new_p0|0|1|1|2
bp_margin_new_p1|1|2|2|3
bp_margin_set|3|4
bp_margin_set_p0|0|1|1|2
bp_margin_set_p1|1|2|2|3
bp_margin_set_p2|2|3|3|4
bp_margin_set_ht|4|-1|-1|2|1
bp_margin_set_nx|4|-1|-1|0|-1
bp_margin_c1|3|4
bp_margin_c1_p0|0|1|1|2
bp_margin_c1_p1|1|2|2|3
bp_margin_c1_p2|2|3|3|4
bp_margin_c1_ht|4|-1|-1|2|1
bp_margin_c1_nx|4|-1|-1|0|-1
bp_tt_two|10
bp_tt_two_p0|3|103
bp_tt_two_p1|2|103
bp_tt_two_p2|3|102
bp_tt_two_p3|2|102
bp_tt_two_p4|1|102
bp_tt_two_p5|2|101
bp_tt_two_p6|1|101
bp_tt_two_p7|0|101
bp_tt_two_p8|1|100
bp_tt_two_p9|0|100
bp_tt_self|3
bp_tt_self_p0|0|1
bp_tt_self_p1|1|2
bp_tt_self_p2|2|3
";

    const DEF: CollisionFilterGroup = CollisionFilterGroup::DEFAULT;
    const STAT: CollisionFilterGroup = CollisionFilterGroup::STATIC;
    const ALL: CollisionFilterGroup = CollisionFilterGroup::ALL;

    /// `probe.cpp`'s `BpSameGroupFilter`.
    struct SameGroupFilter;

    impl OverlapFilterCallback for SameGroupFilter {
        fn need_broadphase_collision(
            &self,
            proxies: &dyn PairProxies,
            proxy0: ProxyHandle,
            proxy1: ProxyHandle,
        ) -> bool {
            proxies
                .proxy(proxy0)
                .collision_filter_group
                .intersects(proxies.proxy(proxy1).collision_filter_group)
        }
    }

    /// `probe.cpp`'s `bpbox`.
    fn bpbox(
        bph: &mut DbvtBroadphase,
        i: i16,
        centre: Vec3,
        h: Scalar,
        group: CollisionFilterGroup,
        mask: CollisionFilterGroup,
    ) -> ProxyHandle {
        let extent = Vec3::new(h, h, h);
        bph.create_proxy(
            centre - extent,
            centre + extent,
            BroadphaseNativeType::BOX_SHAPE,
            usize::try_from(i).expect("a non-negative creation index"),
            group,
            mask,
        )
    }

    /// The four fields a `bp_<phase>_p<i>` row carries, in visit order.
    fn walk(bph: &mut DbvtBroadphase) -> Vec<(usize, usize, i32, i32)> {
        let mut seen = Vec::new();
        let (cache, proxies) = bph.pair_cache_and_proxies();
        cache.process_all_overlapping_pairs(proxies, &mut |proxies, pair: &BroadphasePair| {
            let a = proxies.proxy(pair.proxy0);
            let b = proxies.proxy(pair.proxy1);
            seen.push((a.client_object, b.client_object, a.unique_id, b.unique_id));
            false
        });
        seen
    }

    /// `probe.cpp`'s `cube`.
    fn cube(x: Scalar, y: Scalar, z: Scalar) -> DbvtVolume {
        DbvtVolume::from_mm(
            Vec3::new(x - 0.5, y - 0.5, z - 0.5),
            Vec3::new(x + 0.5, y + 0.5, z + 0.5),
        )
    }

    /// `probe.cpp`'s `bp_tt` -- [`collide_tt_persistent_stack`] on two whole
    /// trees, and on one tree against itself.
    ///
    /// Its own rows because the broadphase cannot reach either shape:
    /// `setAabb` passes a leaf as the second argument, so the four-way child
    /// push and the `p.a == p.b` branch never run on a broadphase row, and
    /// nothing else would pin their order.
    fn check_tt(
        bad: &mut Vec<String>,
        covered: &mut Vec<String>,
        name: &str,
        na: i16,
        nb: i16,
        self_collide: bool,
    ) {
        let mut ta = Dbvt::new();
        let mut tb = Dbvt::new();
        for i in 0..na {
            ta.insert(cube(Scalar::from(i) * 0.8, 0.0, 0.0), i32::from(i));
        }
        for i in 0..nb {
            tb.insert(cube(Scalar::from(i) * 0.8, 0.4, 0.0), 100 + i32::from(i));
        }

        let mut out = Vec::new();
        if self_collide {
            collide_tt_persistent_stack(&ta, ta.root, &ta, ta.root, true, &mut out);
        } else {
            collide_tt_persistent_stack(&ta, ta.root, &tb, tb.root, false, &mut out);
        }
        let other = if self_collide { &ta } else { &tb };
        let pairs: Vec<(i32, i32)> = out
            .iter()
            .map(|&(a, b)| (ta.node(a).data, other.node(b).data))
            .collect();

        let head_name = format!("bp_tt_{name}");
        let head = row(BULLET_REFERENCE, &head_name, 2);
        covered.push(head_name.clone());
        let want_pairs: usize = head[1].parse().expect("a pair count");
        if pairs.len() != want_pairs {
            bad.push(format!(
                "{head_name}: port {} pairs, bullet {want_pairs}",
                pairs.len()
            ));
        }

        for i in 0..want_pairs {
            let pname = format!("bp_tt_{name}_p{i}");
            let f = row(BULLET_REFERENCE, &pname, 3);
            covered.push(pname.clone());
            let Some(&(a, b)) = pairs.get(i) else {
                continue; // the count difference above already names this.
            };
            for (k, field) in ["a", "b"].iter().enumerate() {
                let want: i32 = f[k + 1].parse().expect("an integer field");
                let got = if k == 0 { a } else { b };
                if got != want {
                    bad.push(format!("{pname}.{field}: port {got}, bullet {want}"));
                }
            }
        }
    }

    /// One phase against the fixture: the pair count, the capacity, every pair
    /// row in order, and -- where the probe printed them -- the two hash
    /// arrays.
    ///
    /// `covered` accumulates every row name consulted so
    /// [`every_fixture_row_is_asserted`] can prove none was skipped;
    /// differences accumulate into `bad` rather than asserting, so one run
    /// reports every phase that moved instead of the first.
    fn check(
        bad: &mut Vec<String>,
        covered: &mut Vec<String>,
        phase: &str,
        bph: &mut DbvtBroadphase,
        hash: bool,
    ) {
        let name = format!("bp_{phase}");
        let head = row(BULLET_REFERENCE, &name, 3);
        covered.push(name.clone());

        let want_pairs: usize = head[1].parse().expect("a pair count");
        let want_capacity: i32 = head[2].parse().expect("a capacity");

        let pairs = walk(bph);
        if pairs.len() != want_pairs {
            bad.push(format!(
                "{name}: port {} pairs, bullet {want_pairs}",
                pairs.len()
            ));
        }
        let capacity = bph.overlapping_pair_cache().pair_array_capacity();
        if capacity != want_capacity {
            bad.push(format!(
                "{name}.capacity: port {capacity}, bullet {want_capacity}"
            ));
        }

        // `visit_order` is the accessor a consumer reaches for; it must be the
        // same walk, not a second one.
        let order: Vec<(usize, usize)> = pairs.iter().map(|&(a, b, _, _)| (a, b)).collect();
        if visit_order(bph) != order {
            bad.push(format!("{name}: visit_order disagrees with its own walk"));
        }

        for i in 0..want_pairs {
            let pname = format!("bp_{phase}_p{i}");
            let f = row(BULLET_REFERENCE, &pname, 5);
            covered.push(pname.clone());

            let Some(&(obj0, obj1, uid0, uid1)) = pairs.get(i) else {
                continue; // the count difference above already names this.
            };
            let got = [
                i64::try_from(obj0).expect("a client object below i64::MAX"),
                i64::try_from(obj1).expect("a client object below i64::MAX"),
                i64::from(uid0),
                i64::from(uid1),
            ];
            for (k, field) in ["obj0", "obj1", "uid0", "uid1"].iter().enumerate() {
                let want: i64 = f[k + 1].parse().expect("an integer field");
                if got[k] != want {
                    bad.push(format!("{pname}.{field}: port {}, bullet {want}", got[k]));
                }
            }
        }

        if !hash {
            return;
        }
        let arity = 2 + usize::try_from(want_capacity).expect("a non-negative capacity");
        for (suffix, got) in [
            ("ht", bph.overlapping_pair_cache().hash_table()),
            ("nx", bph.overlapping_pair_cache().next_table()),
        ] {
            let rname = format!("bp_{phase}_{suffix}");
            let f = row(BULLET_REFERENCE, &rname, arity);
            covered.push(rname.clone());

            let want_len: usize = f[1].parse().expect("a table length");
            if got.len() != want_len {
                bad.push(format!(
                    "{rname}: port {} entries, bullet {want_len}",
                    got.len()
                ));
                continue;
            }
            for (i, &entry) in got.iter().enumerate() {
                let want: i32 = f[i + 2].parse().expect("a table entry");
                if entry != want {
                    bad.push(format!("{rname}[{i}]: port {entry}, bullet {want}"));
                }
            }
        }
    }

    /// Every scenario, run once, so a single failure report names every phase
    /// that moved.
    ///
    /// Returns the row names it consulted, for [`every_fixture_row_is_asserted`].
    fn replay(bad: &mut Vec<String>) -> Vec<String> {
        let mut covered = Vec::new();
        let c = &mut covered;

        // Disjoint boxes on a 3x3 grid: no pair at any stage.
        {
            let mut bph = DbvtBroadphase::new();
            for i in 0..9 {
                bpbox(
                    &mut bph,
                    i,
                    Vec3::new(Scalar::from(i / 3) * 5.0, Scalar::from(i % 3) * 5.0, 0.0),
                    0.4,
                    DEF,
                    ALL,
                );
            }
            check(bad, c, "grid_new", &mut bph, true);
            bph.calculate_overlapping_pairs();
            check(bad, c, "grid_c1", &mut bph, true);
        }

        // A chain, each box overlapping its two neighbours. By `c2` the first
        // stage's proxies have migrated into the fixed set, so `add` is a
        // `createProxy` that must collide against *both* sets to find its
        // neighbours.
        {
            let mut bph = DbvtBroadphase::new();
            for i in 0..6 {
                bpbox(
                    &mut bph,
                    i,
                    Vec3::new(Scalar::from(i), 0.0, 0.0),
                    0.6,
                    DEF,
                    ALL,
                );
            }
            check(bad, c, "chain_new", &mut bph, true);
            bph.calculate_overlapping_pairs();
            check(bad, c, "chain_c1", &mut bph, false);
            bph.calculate_overlapping_pairs();
            check(bad, c, "chain_c2", &mut bph, false);
            bpbox(&mut bph, 6, Vec3::new(2.5, 0.0, 0.0), 0.6, DEF, ALL);
            check(bad, c, "chain_add", &mut bph, true);
            bph.calculate_overlapping_pairs();
            check(bad, c, "chain_c3", &mut bph, false);
        }

        // One proxy overlapping every other, created last: its own `collideTV`
        // emits all five pairs in the tree's descent order, not creation order.
        {
            let mut bph = DbvtBroadphase::new();
            for i in 0..5 {
                bpbox(
                    &mut bph,
                    i,
                    Vec3::new(Scalar::from(i) * 2.0, 0.0, 0.0),
                    0.4,
                    DEF,
                    ALL,
                );
            }
            bpbox(&mut bph, 5, Vec3::new(4.0, 0.0, 0.0), 5.0, DEF, ALL);
            check(bad, c, "hub_new", &mut bph, true);
            bph.calculate_overlapping_pairs();
            check(bad, c, "hub_c1", &mut bph, false);
        }

        // Spacing 0.5 against half-extent 0.6, so each proxy overlaps the two
        // either side of it: 13 candidates, of which the built-in group/mask
        // test admits only the ones between the two groups.
        {
            let mut bph = DbvtBroadphase::new();
            for i in 0..8 {
                let (group, mask) = if i % 2 == 1 { (STAT, DEF) } else { (DEF, STAT) };
                bpbox(
                    &mut bph,
                    i,
                    Vec3::new(Scalar::from(i) * 0.5, 0.0, 0.0),
                    0.6,
                    group,
                    mask,
                );
            }
            check(bad, c, "filter_new", &mut bph, true);
            bph.calculate_overlapping_pairs();
            check(bad, c, "filter_c1", &mut bph, false);
        }

        // The same 13 candidates with every mask `AllFilter` -- the built-in
        // test would keep all 13 -- behind a callback that keeps exactly the
        // complement of what the built-in kept above.
        {
            let mut bph = DbvtBroadphase::new();
            bph.set_overlap_filter_callback(Some(Box::new(SameGroupFilter)));
            for i in 0..8 {
                let group = if i % 2 == 1 { STAT } else { DEF };
                bpbox(
                    &mut bph,
                    i,
                    Vec3::new(Scalar::from(i) * 0.5, 0.0, 0.0),
                    0.6,
                    group,
                    ALL,
                );
            }
            check(bad, c, "filtercb_new", &mut bph, true);
            bph.calculate_overlapping_pairs();
            check(bad, c, "filtercb_c1", &mut bph, false);
        }

        // `setAabb`, both branches. `tiny` moves a proxy already in the fixed
        // set by less than the margin -- the fixed-to-dynamic path runs
        // whatever the distance, and re-finds the pairs it already has rather
        // than appending duplicates. `far` moves one across the chain, adding
        // pairs at once and leaving the pairs it left behind stale; those die
        // in `collide`'s cleanup sweep, which is what `c4` shows.
        {
            let mut bph = DbvtBroadphase::new();
            let mut p = Vec::new();
            for i in 0..5 {
                p.push(bpbox(
                    &mut bph,
                    i,
                    Vec3::new(Scalar::from(i), 0.0, 0.0),
                    0.6,
                    DEF,
                    ALL,
                ));
            }
            check(bad, c, "move_new", &mut bph, false);
            bph.calculate_overlapping_pairs();
            check(bad, c, "move_c1", &mut bph, false);
            bph.calculate_overlapping_pairs();
            check(bad, c, "move_c2", &mut bph, false);
            bph.set_aabb(p[2], Vec3::new(1.41, -0.6, -0.6), Vec3::new(2.61, 0.6, 0.6));
            check(bad, c, "move_tiny", &mut bph, true);
            bph.calculate_overlapping_pairs();
            check(bad, c, "move_c3", &mut bph, false);
            bph.set_aabb(p[0], Vec3::new(3.4, -0.6, -0.6), Vec3::new(4.6, 0.6, 0.6));
            check(bad, c, "move_far", &mut bph, true);
            bph.calculate_overlapping_pairs();
            check(bad, c, "move_c4", &mut bph, true);
            bph.calculate_overlapping_pairs();
            check(bad, c, "move_c5", &mut bph, true);
        }

        // `destroyProxy` on a proxy in the middle of the pair array, through
        // the consumer's own sequence. Each of its pairs is removed by
        // swapping the last pair into the hole, so this is where a port that
        // shifts instead of swapping diverges. `readd` then reuses the vacated
        // position in the tree without reusing the unique id.
        {
            let mut bph = DbvtBroadphase::new();
            let mut p = Vec::new();
            for i in 0..6 {
                p.push(bpbox(
                    &mut bph,
                    i,
                    Vec3::new(Scalar::from(i), 0.0, 0.0),
                    0.6,
                    DEF,
                    ALL,
                ));
            }
            bph.calculate_overlapping_pairs();
            check(bad, c, "destroy_c1", &mut bph, true);
            let (cache, proxies) = bph.pair_cache_and_proxies();
            cache.clean_proxy_from_pairs(proxies, p[2]);
            bph.destroy_proxy(p[2]);
            check(bad, c, "destroy_gone", &mut bph, true);
            bph.calculate_overlapping_pairs();
            check(bad, c, "destroy_c2", &mut bph, false);
            bpbox(&mut bph, 6, Vec3::new(2.0, 0.0, 0.0), 0.6, DEF, ALL);
            check(bad, c, "destroy_readd", &mut bph, true);
        }

        // 21 pairs, so the pair array's capacity walks 2, 4, 8, 16, 32 and the
        // table is rehashed four times, each rehash re-deriving every bucket
        // through a different mask.
        {
            let mut bph = DbvtBroadphase::new();
            for i in 0..12 {
                bpbox(
                    &mut bph,
                    i,
                    Vec3::new(Scalar::from(i) * 0.5, 0.0, 0.0),
                    0.6,
                    DEF,
                    ALL,
                );
            }
            check(bad, c, "grow_new", &mut bph, true);
            bph.calculate_overlapping_pairs();
            check(bad, c, "grow_c1", &mut bph, true);
        }

        // Removing the *last* pair in the array, which is the one case
        // `removeOverlappingPair` answers with the early `pop_back` rather than
        // the swap: p3's only pair is (2,3) and it is the last of three.
        {
            let mut bph = DbvtBroadphase::new();
            let mut p = Vec::new();
            for i in 0..4 {
                p.push(bpbox(
                    &mut bph,
                    i,
                    Vec3::new(Scalar::from(i), 0.0, 0.0),
                    0.6,
                    DEF,
                    ALL,
                ));
            }
            bph.calculate_overlapping_pairs();
            check(bad, c, "last_c1", &mut bph, true);
            let (cache, proxies) = bph.pair_cache_and_proxies();
            cache.clean_proxy_from_pairs(proxies, p[3]);
            bph.destroy_proxy(p[3]);
            check(bad, c, "last_gone", &mut bph, true);
        }

        // A filter assignment the *second* direction decides. p0's group is in
        // p1's mask, so the first half of `needsBroadphaseCollision` passes;
        // p1's group is not in p0's, so only the conjunction rejects (0,1).
        {
            let mut bph = DbvtBroadphase::new();
            bpbox(
                &mut bph,
                0,
                Vec3::zero(),
                0.6,
                DEF,
                CollisionFilterGroup::KINEMATIC,
            );
            bpbox(&mut bph, 1, Vec3::new(1.0, 0.0, 0.0), 0.6, STAT, DEF);
            bpbox(&mut bph, 2, Vec3::new(2.0, 0.0, 0.0), 0.6, DEF, ALL);
            bpbox(&mut bph, 3, Vec3::new(3.0, 0.0, 0.0), 0.6, DEF, ALL);
            check(bad, c, "asym_new", &mut bph, true);
            bph.calculate_overlapping_pairs();
            check(bad, c, "asym_c1", &mut bph, false);
        }

        // Six `collide` passes before the query, so `optimize_incremental` has
        // walked six `m_opath` bit patterns and removed-and-reinserted six
        // leaves. The hub created afterwards reads the resulting tree shape out
        // as its `collideTV` order -- which is the only way that shape becomes
        // observable at all, and it is not the creation order: the fixture has
        // proxy 7 ahead of proxy 6.
        {
            let mut bph = DbvtBroadphase::new();
            for i in 0..8 {
                bpbox(
                    &mut bph,
                    i,
                    Vec3::new(Scalar::from(i), 0.0, 0.0),
                    0.6,
                    DEF,
                    ALL,
                );
            }
            for _ in 0..6 {
                bph.calculate_overlapping_pairs();
            }
            check(bad, c, "opath_c6", &mut bph, true);
            bpbox(&mut bph, 8, Vec3::new(3.5, 0.0, 0.0), 10.0, DEF, ALL);
            check(bad, c, "opath_hub", &mut bph, true);
        }

        // `set_aabb` on a proxy still in the dynamic set -- no `collide` has
        // run, so it takes the `intersect(leaf.volume, aabb)` "Moving" branch,
        // the only path that reads [`DBVT_MARGIN`]. The move is 0.02, which its
        // leaf volume does not contain, so the update expands by the margin; p3
        // starts 0.03 beyond the moved box and 0.02 inside the expanded one, so
        // the pair that appears exists only because the margin does.
        {
            let mut bph = DbvtBroadphase::new();
            let mut p = Vec::new();
            for (i, x) in [0.0, 1.0, 2.0, 3.25].into_iter().enumerate() {
                let i = i16::try_from(i).expect("four proxies");
                p.push(bpbox(&mut bph, i, Vec3::new(x, 0.0, 0.0), 0.6, DEF, ALL));
            }
            check(bad, c, "margin_new", &mut bph, false);
            bph.set_aabb(p[2], Vec3::new(1.42, -0.6, -0.6), Vec3::new(2.62, 0.6, 0.6));
            check(bad, c, "margin_set", &mut bph, true);
            bph.calculate_overlapping_pairs();
            check(bad, c, "margin_c1", &mut bph, true);
        }

        // The two `collide_tt_persistent_stack` shapes no broadphase row
        // reaches: two whole trees, and one tree against itself.
        check_tt(bad, c, "two", 4, 4, false);
        check_tt(bad, c, "self", 4, 0, true);

        covered
    }

    /// Every pair, in Bullet's own visit order, across every phase.
    #[test]
    fn pair_order_matches_bullet() {
        let mut bad = Vec::new();
        replay(&mut bad);
        assert!(
            bad.is_empty(),
            "{} deviations from bullet:\n{}",
            bad.len(),
            bad.join("\n")
        );
    }

    /// The fixture and the replay cover the same rows.
    ///
    /// Without this a phase deleted from `replay` -- or a row added to
    /// `BULLET_REFERENCE` and never consulted -- would leave the suite green
    /// while asserting less than it claims to.
    #[test]
    fn every_fixture_row_is_asserted() {
        let mut bad = Vec::new();
        let mut covered = replay(&mut bad);
        covered.sort();
        let before = covered.len();
        covered.dedup();
        assert_eq!(before, covered.len(), "a row was asserted twice");

        let mut want: Vec<String> = BULLET_REFERENCE
            .lines()
            .map(|l| l.split('|').next().expect("a row name").to_string())
            .collect();
        want.sort();

        assert_eq!(covered, want);
    }
}
