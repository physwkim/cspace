// Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from octomap 1.9.7 (see key.rs's provenance comment for how the
// version was matched):
//   include/octomap/OcTreeBaseImpl.h, OcTreeBaseImpl.hxx
//   include/octomap/OccupancyOcTreeBase.h, OccupancyOcTreeBase.hxx
//   include/octomap/AbstractOccupancyOcTree.h
//   include/octomap/octomap_utils.h (logodds / probability)
//   include/octomap/OcTree.h (the concrete, non-template `OcTree`; its
//     sensor-model defaults live in OcTree.cpp, compiled into
//     liboctomap.so.1.9 and not shipped as source -- see the `DEFAULT_*`
//     constants below for how those five numbers were confirmed rather than
//     guessed)

use nalgebra::{Point3, Vector3};

use crate::key::{KeyRay, KeySet, KeyType, OcTreeKey, compute_child_idx};
use crate::node::Node;

/// Upstream `octomap::logodds`: `log(p / (1 - p))`, computed in `f64` and
/// rounded to `f32` at the end, matching upstream's `(float) log(...)` cast.
fn logodds(probability: f64) -> f32 {
    (probability / (1.0 - probability)).ln() as f32
}

/// Upstream `octomap::probability`: the inverse of [`logodds`].
pub(crate) fn probability(log_odds: f64) -> f64 {
    1.0 - (1.0 / (1.0 + log_odds.exp()))
}

/// A probabilistic occupancy octree. Upstream `octomap::OcTree` (via
/// `OccupancyOcTreeBase<OcTreeNode>` and `OcTreeBaseImpl<OcTreeNode,
/// AbstractOccupancyOcTree>`).
///
/// # What this port does not carry over
///
/// - **No `AbstractOcTree` registry.** Upstream self-registers each concrete
///   tree type (`OcTree`, `ColorOcTree`, ...) into a runtime factory keyed by
///   `className()`, so e.g. a binary file's header can look up the right
///   type to deserialize into. That is a pluginlib-shaped runtime-type
///   registry, and PORTING-PLAN.md's D4 rules out runtime polymorphism in
///   favor of compile-time dispatch: this crate has exactly one concrete
///   tree type, so nothing needs registering or looking up.
/// - **Only the plain occupancy node is ported**, not `ColorOcTreeNode`,
///   `CountingOcTreeNode`, or `OcTreeNodeStamped` -- moveit2 never
///   references any of those (confirmed by a workspace-wide
///   `grep -rl "octomap::"` over `moveit_core`/`moveit_ros`, see this
///   change's commit body for the file:line inventory).
/// - **`changed_keys` / `enableChangeDetection` is not ported.** No moveit2
///   caller enables change detection; it is an opt-in upstream feature nothing
///   in this port's target surface exercises.
/// - **`setNodeValue` (the direct, non-relative set) is not ported.** The one
///   moveit2 call site that conceptually wants "set to the clamp minimum"
///   (`lazy_free_space_updater.cpp`'s `tree_->updateNode(it, lg_0)`, where
///   `lg_0 = clampingThresMinLog - clampingThresMaxLog`) does so through the
///   *relative* `updateNode(key, log_odds_update)` primitive instead, adding
///   a delta so large that the result clamps to `clamping_thres_min`
///   regardless of the node's prior value -- so `update_node_log_odds`
///   already covers it.
/// - **`computeDiscreteUpdate`/BBX-limited updates are not ported.** No
///   moveit2 caller passes `discretize=true` or calls `useBBXLimit`; both
///   sensor updaters build their own key sets by hand instead of calling
///   upstream's `insertPointCloud` family at all (see this crate's docs and
///   the commit body's Step 1 API-surface inventory).
///
/// # Representation: pointer-linked nodes, not a flat keyed map
///
/// The tree is a genuine parent-child node tree (`Node`, `Box`-owned
/// instead of upstream's manually `new`/`delete`d raw pointers), not a flat
/// `HashMap<OcTreeKey, f32>` at the finest resolution. This is not just
/// fidelity to upstream: [`Self::prune`] and the eager per-update pruning in
/// [`Self::update_node`] collapse 8 identical children into their parent,
/// and multi-resolution leaf iteration ([`Self::leaves`]) reports a node's
/// size at whatever depth it was collapsed to. A flat map has no notion of
/// "level" for either of those to collapse into or report from -- it would
/// have to simulate the tree internally anyway, at which point it is not
/// actually flat. The pointer tree is the structurally necessary
/// representation for the semantics this crate's callers need, not an
/// arbitrary upstream-fidelity choice.
pub struct OcTree {
    root: Option<Box<Node>>,
    resolution: f64,
    resolution_factor: f64,
    clamping_thres_min: f32,
    clamping_thres_max: f32,
    prob_hit_log: f32,
    prob_miss_log: f32,
    occ_prob_thres_log: f32,
}

impl OcTree {
    /// Upstream `tree_depth`: fixed maximum tree depth.
    pub const TREE_DEPTH: u32 = 16;
    /// Upstream `tree_max_val`: the key-space coordinate of the tree center
    /// (`2^15`), fixed alongside `TREE_DEPTH` since both come from the same
    /// non-template `OcTreeBaseImpl(double)` constructor upstream's `OcTree`
    /// actually uses (the `tree_depth`/`tree_max_val`-parameterized
    /// constructor exists only for hypothetical subclasses; nothing in
    /// moveit2 or this crate's scope needs one).
    pub const TREE_MAX_VAL: i32 = 32768;

    /// Sensor-model defaults for a freshly constructed tree. Upstream's
    /// `OcTree` constructor sets these via `setProbHit`/`setProbMiss`/
    /// `setClampingThresMin`/`setClampingThresMax` with literal probabilities
    /// baked into `OcTree.cpp` -- compiled into `liboctomap.so.1.9`, not
    /// shipped as header source, so these five numbers cannot be read from
    /// any file on this machine. They were instead measured directly off the
    /// real `liboctomap.so.1.9.7` inside the oracle container: a throwaway
    /// probe (`octomap::OcTree(0.1)`, then `getProbHit()`/`getProbHitLog()`/
    /// etc.) printed the exact log-odds this port now hardcodes, and those
    /// measured log-odds round-trip through this file's own `logodds()`
    /// applied to the human-readable probabilities below to within `f32`
    /// rounding -- see this change's commit body for the full probe output.
    const DEFAULT_PROB_HIT: f64 = 0.7;
    const DEFAULT_PROB_MISS: f64 = 0.4;
    const DEFAULT_CLAMPING_THRES_MIN: f64 = 0.1192;
    const DEFAULT_CLAMPING_THRES_MAX: f64 = 0.971;
    const DEFAULT_OCCUPANCY_THRES: f64 = 0.5;

    /// Upstream `OcTree(double resolution)`.
    pub fn new(resolution: f64) -> Self {
        Self {
            root: None,
            resolution,
            resolution_factor: 1.0 / resolution,
            clamping_thres_min: logodds(Self::DEFAULT_CLAMPING_THRES_MIN),
            clamping_thres_max: logodds(Self::DEFAULT_CLAMPING_THRES_MAX),
            prob_hit_log: logodds(Self::DEFAULT_PROB_HIT),
            prob_miss_log: logodds(Self::DEFAULT_PROB_MISS),
            occ_prob_thres_log: logodds(Self::DEFAULT_OCCUPANCY_THRES),
        }
    }

    /// Upstream `getResolution`.
    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    /// Upstream `getProbHitLog`.
    pub fn prob_hit_log(&self) -> f32 {
        self.prob_hit_log
    }
    /// Upstream `getProbMissLog`.
    pub fn prob_miss_log(&self) -> f32 {
        self.prob_miss_log
    }
    /// Upstream `getClampingThresMinLog`.
    pub fn clamping_thres_min_log(&self) -> f32 {
        self.clamping_thres_min
    }
    /// Upstream `getClampingThresMaxLog`.
    pub fn clamping_thres_max_log(&self) -> f32 {
        self.clamping_thres_max
    }
    /// Upstream `getOccupancyThresLog`.
    pub fn occupancy_thres_log(&self) -> f32 {
        self.occ_prob_thres_log
    }

    /// Upstream `getNodeSize`: the metric size of a voxel at `depth` (0:
    /// root, [`Self::TREE_DEPTH`]: finest resolution).
    pub fn node_size(&self, depth: u32) -> f64 {
        assert!(depth <= Self::TREE_DEPTH);
        self.resolution * f64::from(1u32 << (Self::TREE_DEPTH - depth))
    }

    /// Upstream `isNodeOccupied`: compares a node's log-odds against the
    /// tree's occupancy threshold. Note the `>=`: a never-updated node
    /// freshly touched by an update (log-odds `0.0`, probability exactly
    /// `0.5`) reads as occupied under the default threshold, since
    /// `occ_prob_thres_log` is also exactly `0.0` -- confirmed against the
    /// real binary, not assumed (see `Self::DEFAULT_OCCUPANCY_THRES`'s
    /// doc comment).
    pub fn is_node_occupied_log_odds(&self, log_odds: f32) -> bool {
        log_odds >= self.occ_prob_thres_log
    }

    /// Upstream `keyToCoord(key_type)` at the finest depth.
    fn key_to_coord_axis(&self, key: KeyType) -> f64 {
        (f64::from(key) - f64::from(Self::TREE_MAX_VAL) + 0.5) * self.resolution
    }

    /// Upstream `keyToCoord(key_type, depth)`.
    fn key_to_coord_axis_at_depth(&self, key: KeyType, depth: u32) -> f64 {
        if depth == 0 {
            0.0
        } else if depth == Self::TREE_DEPTH {
            self.key_to_coord_axis(key)
        } else {
            let shift = Self::TREE_DEPTH - depth;
            (((f64::from(key) - f64::from(Self::TREE_MAX_VAL)) / f64::from(1u32 << shift)).floor()
                + 0.5)
                * self.node_size(depth)
        }
    }

    /// Upstream `keyToCoord(const OcTreeKey&, depth)`.
    pub(crate) fn key_to_coord_at_depth(&self, key: OcTreeKey, depth: u32) -> Point3<f64> {
        Point3::new(
            self.key_to_coord_axis_at_depth(key[0], depth),
            self.key_to_coord_axis_at_depth(key[1], depth),
            self.key_to_coord_axis_at_depth(key[2], depth),
        )
    }

    /// Upstream `coordToKeyChecked(double, key_type&)` for one axis.
    fn coord_to_key_checked_axis(&self, coord: f64) -> Option<KeyType> {
        let scaled =
            (self.resolution_factor * coord).floor() as i64 + i64::from(Self::TREE_MAX_VAL);
        if scaled >= 0 && scaled < 2 * i64::from(Self::TREE_MAX_VAL) {
            Some(scaled as KeyType)
        } else {
            None
        }
    }

    /// Upstream `coordToKeyChecked(const point3d&, OcTreeKey&)`. `None` if
    /// any axis is out of the tree's representable range (upstream:
    /// `false`).
    pub fn coord_to_key_checked(&self, p: Point3<f64>) -> Option<OcTreeKey> {
        Some(OcTreeKey::new(
            self.coord_to_key_checked_axis(p.x)?,
            self.coord_to_key_checked_axis(p.y)?,
            self.coord_to_key_checked_axis(p.z)?,
        ))
    }

    pub(crate) fn root_key() -> OcTreeKey {
        let c = Self::TREE_MAX_VAL as KeyType;
        OcTreeKey::new(c, c, c)
    }

    /// Read-only access to the root node, for [`crate::iter`]'s stack-based
    /// traversal.
    pub(crate) fn root(&self) -> Option<&Node> {
        self.root.as_deref()
    }

    /// Upstream `search(const OcTreeKey&, depth)`. `depth == 0` means the
    /// finest resolution, matching upstream's own `0`-means-full-depth
    /// convention.
    pub(crate) fn search(&self, key: OcTreeKey, depth: u32) -> Option<&Node> {
        let mut cur = self.root.as_deref()?;
        let depth = if depth == 0 { Self::TREE_DEPTH } else { depth };
        let diff = Self::TREE_DEPTH - depth;
        for i in (diff..Self::TREE_DEPTH).rev() {
            let pos = compute_child_idx(key, i) as usize;
            match cur.child(pos) {
                Some(child) => cur = child,
                None => {
                    return if cur.has_children() { None } else { Some(cur) };
                }
            }
        }
        Some(cur)
    }

    /// Log-odds occupancy at `point`, or `None` if unmapped (out of tree
    /// bounds, or never touched by an update). Convenience wrapper; not a
    /// direct upstream method (upstream callers use `search` + `getLogOdds`
    /// inline).
    pub fn log_odds_at(&self, point: Point3<f64>) -> Option<f32> {
        let key = self.coord_to_key_checked(point)?;
        self.search(key, 0).map(|n| n.log_odds)
    }

    /// Occupancy probability at `point` (`[0, 1]`), or `None` if unmapped.
    pub fn occupancy_at(&self, point: Point3<f64>) -> Option<f64> {
        self.log_odds_at(point).map(|lo| probability(f64::from(lo)))
    }

    /// Upstream `isNodeOccupied` applied to whatever `search` finds at
    /// `point`, or `None` if unmapped.
    pub fn is_occupied(&self, point: Point3<f64>) -> Option<bool> {
        self.log_odds_at(point)
            .map(|lo| self.is_node_occupied_log_odds(lo))
    }

    /// Upstream `getNumLeafNodes`/`getNumLeafNodesRecurs`: a fresh recursive
    /// count, not a maintained counter -- upstream itself recomputes this on
    /// every call rather than tracking it incrementally, so this port does
    /// too.
    pub fn num_leaf_nodes(&self) -> usize {
        fn recurs(node: &Node) -> usize {
            if !node.has_children() {
                return 1;
            }
            (0..8).filter_map(|i| node.child(i)).map(recurs).sum()
        }
        self.root.as_deref().map(recurs).unwrap_or(0)
    }

    /// Total node count (inner + leaf), for tests observing that [`Self::prune`]
    /// actually collapsed a subtree. Not a direct upstream method (upstream's
    /// `size()` returns a maintained `tree_size` counter this port does not
    /// keep, for the same reason it does not keep one for leaf counts).
    pub fn num_nodes(&self) -> usize {
        fn recurs(node: &Node) -> usize {
            1 + (0..8)
                .filter_map(|i| node.child(i))
                .map(recurs)
                .sum::<usize>()
        }
        self.root.as_deref().map(recurs).unwrap_or(0)
    }

    /// Upstream `updateNode(const OcTreeKey&, float, bool)`: adds
    /// `log_odds_update` to the node at `key`, clamped to the tree's
    /// clamping thresholds. Upstream's early-return optimization for an
    /// already-saturated node (`search` first, skip if already at threshold
    /// and the update would push further past it) is not ported: it is a
    /// pure performance optimization upstream's own doc comment calls out
    /// as situational ("may cause an overhead in some configuration"), and
    /// skipping it cannot change the final clamped value, only how many
    /// nodes get transiently expanded and re-pruned to reach it.
    pub fn update_node_log_odds_by_key(
        &mut self,
        key: OcTreeKey,
        log_odds_update: f32,
        lazy_eval: bool,
    ) {
        let params = UpdateParams {
            log_odds_update,
            lazy_eval,
            clamp_min: self.clamping_thres_min,
            clamp_max: self.clamping_thres_max,
        };
        let created_root = self.root.is_none();
        if created_root {
            self.root = Some(Box::new(Node::new()));
        }
        update_node_recurs(
            self.root.as_mut().expect("just ensured"),
            created_root,
            key,
            0,
            &params,
        );
    }

    /// Upstream `updateNode(const OcTreeKey&, bool, bool)`.
    pub fn update_node_by_key(&mut self, key: OcTreeKey, occupied: bool, lazy_eval: bool) {
        let log_odds = if occupied {
            self.prob_hit_log
        } else {
            self.prob_miss_log
        };
        self.update_node_log_odds_by_key(key, log_odds, lazy_eval);
    }

    /// Upstream `updateNode(const point3d&, float, bool)`. Returns `false`
    /// if `point` is out of the tree's representable range (matching
    /// upstream returning a null node pointer).
    pub fn update_node_log_odds(
        &mut self,
        point: Point3<f64>,
        log_odds_update: f32,
        lazy_eval: bool,
    ) -> bool {
        match self.coord_to_key_checked(point) {
            Some(key) => {
                self.update_node_log_odds_by_key(key, log_odds_update, lazy_eval);
                true
            }
            None => false,
        }
    }

    /// Upstream `updateNode(const point3d&, bool, bool)`.
    pub fn update_node(&mut self, point: Point3<f64>, occupied: bool, lazy_eval: bool) -> bool {
        let log_odds = if occupied {
            self.prob_hit_log
        } else {
            self.prob_miss_log
        };
        self.update_node_log_odds(point, log_odds, lazy_eval)
    }

    /// Upstream `updateInnerOccupancy`/`updateInnerOccupancyRecurs`: after a
    /// batch of `lazy_eval = true` updates, propagate children's occupancy
    /// up through the inner nodes they left stale. This is the second half
    /// of the "lazy-eval update pattern" the sensor pipeline depends on
    /// (`pointcloud_octomap_updater.cpp`/`lazy_free_space_updater.cpp`
    /// coalesce many rays' worth of key updates before touching the tree at
    /// all; this port's equivalent caller would do the coalescing the same
    /// way those do, then call this once instead of re-pruning after every
    /// single key).
    pub fn update_inner_occupancy(&mut self) {
        fn recurs(node: &mut Node) {
            if node.has_children() {
                for i in 0..8 {
                    if let Some(child) = node.child_mut(i) {
                        recurs(child);
                    }
                }
                node.update_occupancy_from_children();
            }
        }
        if let Some(root) = self.root.as_mut() {
            recurs(root);
        }
    }

    /// Upstream `prune`/`pruneRecurs`: a bottom-up sweep collapsing every
    /// collapsible subtree, one tree level at a time, stopping as soon as a
    /// level collapses nothing.
    pub fn prune(&mut self) {
        fn recurs(node: &mut Node, depth: u32, max_depth: u32, num_pruned: &mut usize) {
            if depth < max_depth {
                for i in 0..8 {
                    if let Some(child) = node.child_mut(i) {
                        recurs(child, depth + 1, max_depth, num_pruned);
                    }
                }
            } else if node.prune() {
                *num_pruned += 1;
            }
        }
        let Some(root) = self.root.as_mut() else {
            return;
        };
        for depth in (1..Self::TREE_DEPTH).rev() {
            let mut num_pruned = 0;
            recurs(root, 0, depth, &mut num_pruned);
            if num_pruned == 0 {
                break;
            }
        }
    }

    /// Upstream `computeRayKeys`: Amanatides & Woo fast voxel traversal
    /// ("A Faster Voxel Traversal Algorithm for Ray Tracing"), a 3D DDA.
    /// Returns the keys of every voxel the ray from `origin` to `end`
    /// passes through, excluding `end` itself. `None` if either endpoint is
    /// out of the tree's representable range (matching upstream's `false`).
    pub fn compute_ray_keys(&self, origin: Point3<f64>, end: Point3<f64>) -> Option<KeyRay> {
        let key_origin = self.coord_to_key_checked(origin)?;
        let key_end = self.coord_to_key_checked(end)?;

        let mut ray = KeyRay::new();
        if key_origin == key_end {
            return Some(ray);
        }
        ray.push(key_origin);

        let direction: Vector3<f64> = end - origin;
        let length = direction.norm();
        let direction = direction / length;

        let mut step = [0i32; 3];
        let mut t_max = [f64::MAX; 3];
        let mut t_delta = [f64::MAX; 3];
        let mut current_key = key_origin;

        for i in 0..3 {
            step[i] = if direction[i] > 0.0 {
                1
            } else if direction[i] < 0.0 {
                -1
            } else {
                0
            };
            if step[i] != 0 {
                let voxel_border = self.key_to_coord_axis(current_key[i])
                    + f64::from(step[i]) * self.resolution * 0.5;
                t_max[i] = (voxel_border - origin[i]) / direction[i];
                t_delta[i] = self.resolution / direction[i].abs();
            }
        }

        loop {
            let dim = if t_max[0] < t_max[1] {
                if t_max[0] < t_max[2] { 0 } else { 2 }
            } else if t_max[1] < t_max[2] {
                1
            } else {
                2
            };

            current_key = step_key(current_key, dim, step[dim]);
            t_max[dim] += t_delta[dim];

            if current_key == key_end {
                break;
            }

            let dist_from_origin = t_max[0].min(t_max[1]).min(t_max[2]);
            if dist_from_origin > length {
                break;
            }
            ray.push(current_key);
        }

        Some(ray)
    }

    /// Upstream `computeUpdate`: the batch helper behind `insertPointCloud`,
    /// computing every free and occupied key a point cloud touches at once
    /// (occupied cells win ties -- the sets are made disjoint, occupied
    /// side). This port omits upstream's BBX-limited branch (`useBBXLimit`)
    /// since no moveit2 caller ever enables it.
    pub fn compute_update(
        &self,
        points: &[Point3<f64>],
        origin: Point3<f64>,
        max_range: Option<f64>,
    ) -> (KeySet, KeySet) {
        let mut free_cells = KeySet::new();
        let mut occupied_cells = KeySet::new();

        for &p in points {
            let within_range = max_range.is_none_or(|r| (p - origin).norm() <= r);
            if within_range {
                if let Some(ray) = self.compute_ray_keys(origin, p) {
                    free_cells.extend(ray);
                }
                if let Some(key) = self.coord_to_key_checked(p) {
                    occupied_cells.insert(key);
                }
            } else if let Some(r) = max_range {
                let direction = (p - origin).normalize();
                let new_end = origin + direction * r;
                if let Some(ray) = self.compute_ray_keys(origin, new_end) {
                    free_cells.extend(ray);
                }
            }
        }

        free_cells.retain(|k| !occupied_cells.contains(k));
        (free_cells, occupied_cells)
    }

    /// Upstream `integrateMissOnRay`.
    fn integrate_miss_on_ray(
        &mut self,
        origin: Point3<f64>,
        end: Point3<f64>,
        lazy_eval: bool,
    ) -> bool {
        match self.compute_ray_keys(origin, end) {
            Some(ray) => {
                let miss = self.prob_miss_log;
                for key in ray {
                    self.update_node_log_odds_by_key(key, miss, lazy_eval);
                }
                true
            }
            None => false,
        }
    }

    /// Upstream `begin_leafs()`/`end_leafs()`: iterate every leaf (a node
    /// with no children, at whatever depth it was pruned to) in the tree.
    /// See [`crate::iter::Leaves`].
    pub fn leaves(&self) -> crate::iter::Leaves<'_> {
        crate::iter::Leaves::new(self)
    }

    /// Upstream `begin_leafs_bbx(min, max)`/`end_leafs_bbx()`: iterate every
    /// leaf whose voxel overlaps the axis-aligned box `[min, max]`. `None`
    /// if `min` or `max` is out of the tree's representable range. See
    /// [`crate::iter::LeavesInBbx`].
    pub fn leaves_in_bbx(
        &self,
        min: Point3<f64>,
        max: Point3<f64>,
    ) -> Option<crate::iter::LeavesInBbx<'_>> {
        crate::iter::LeavesInBbx::new(self, min, max)
    }

    /// Upstream `insertRay`: integrates a miss along the whole ray and, if
    /// the ray was not cut short by `max_range`, a hit at `end`. A ray cut
    /// short by `max_range` records only the miss up to the cut point --
    /// upstream does not guess at occupancy beyond the sensor's range.
    pub fn insert_ray(
        &mut self,
        origin: Point3<f64>,
        end: Point3<f64>,
        max_range: Option<f64>,
        lazy_eval: bool,
    ) -> bool {
        if let Some(r) = max_range
            && (end - origin).norm() > r
        {
            let direction = (end - origin).normalize();
            let new_end = origin + direction * r;
            return self.integrate_miss_on_ray(origin, new_end, lazy_eval);
        }
        if !self.integrate_miss_on_ray(origin, end, lazy_eval) {
            return false;
        }
        self.update_node(end, true, lazy_eval);
        true
    }
}

/// Upstream `computeChildKey`'s `+/-1` step on one axis, in `OcTreeKey`
/// space. Upstream relies on `key_type` (`uint16_t`) wraparound at the
/// bounds of representable key-space; `wrapping_add`/`wrapping_sub` is the
/// literal Rust equivalent of that well-defined C++ unsigned overflow.
fn step_key(key: OcTreeKey, axis: usize, step: i32) -> OcTreeKey {
    let mut k = [key[0], key[1], key[2]];
    k[axis] = match step {
        1 => k[axis].wrapping_add(1),
        -1 => k[axis].wrapping_sub(1),
        _ => k[axis],
    };
    OcTreeKey::new(k[0], k[1], k[2])
}

/// Bundles one `updateNode` call's per-call parameters so
/// [`update_node_recurs`] takes one value per genuinely distinct piece of
/// recursion state (`node`, `node_just_created`, `key`, `depth`) instead of
/// growing an argument per field; the alternative was `#[allow(clippy::
/// too_many_arguments)]`, which this project's lint policy forbids as a way
/// to silence rather than fix a shape problem.
struct UpdateParams {
    log_odds_update: f32,
    lazy_eval: bool,
    clamp_min: f32,
    clamp_max: f32,
}

/// Upstream `OccupancyOcTreeBase::updateNodeRecurs`.
fn update_node_recurs(
    node: &mut Node,
    node_just_created: bool,
    key: OcTreeKey,
    depth: u32,
    params: &UpdateParams,
) {
    if depth < OcTree::TREE_DEPTH {
        let pos = compute_child_idx(key, OcTree::TREE_DEPTH - 1 - depth) as usize;
        let created_now = if node.child(pos).is_none() {
            if !node.has_children() && !node_just_created {
                node.expand();
                false
            } else {
                node.create_child(pos);
                true
            }
        } else {
            false
        };
        update_node_recurs(
            node.child_mut(pos).expect("just ensured to exist"),
            created_now,
            key,
            depth + 1,
            params,
        );
        if !params.lazy_eval && !node.prune() {
            node.update_occupancy_from_children();
        }
    } else {
        node.log_odds =
            (node.log_odds + params.log_odds_update).clamp(params.clamp_min, params.clamp_max);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repeated_hits_converge_to_clamp_but_not_past_it() {
        let mut tree = OcTree::new(0.1);
        let p = Point3::new(0.05, 0.05, 0.05);
        for _ in 0..1000 {
            tree.update_node(p, true, false);
        }
        assert_eq!(tree.log_odds_at(p).unwrap(), tree.clamping_thres_max_log());
        assert!(tree.is_occupied(p).unwrap());
    }

    #[test]
    fn miss_sequence_drives_occupancy_below_threshold() {
        let mut tree = OcTree::new(0.1);
        let p = Point3::new(0.05, 0.05, 0.05);
        tree.update_node(p, true, false);
        assert!(tree.is_occupied(p).unwrap());
        for _ in 0..10 {
            tree.update_node(p, false, false);
        }
        assert!(!tree.is_occupied(p).unwrap());
        assert!(tree.occupancy_at(p).unwrap() < 0.5);
        assert_eq!(tree.log_odds_at(p).unwrap(), tree.clamping_thres_min_log());
    }

    #[test]
    fn prune_collapses_eight_uniform_children_and_preserves_their_value() {
        let mut tree = OcTree::new(1.0);
        let hit = tree.prob_hit_log();
        // The 8 finest-level children of one common depth-15 parent: fix
        // every bit except the least significant one, which ranges over
        // {0, 1} on each axis. Two numerically adjacent keys are NOT
        // guaranteed to be siblings this way (e.g. 32767/32768 diverge at
        // the root, not the leaf) -- picking a key with no power-of-two
        // boundary nearby (100/101) avoids that trap.
        let mut keys = Vec::new();
        for &kx in &[100u16, 101] {
            for &ky in &[100u16, 101] {
                for &kz in &[100u16, 101] {
                    keys.push(OcTreeKey::new(kx, ky, kz));
                }
            }
        }
        for &k in &keys {
            // lazy_eval = true defers pruning, so all 8 leaves genuinely
            // exist as separate nodes for the explicit prune() below to
            // collapse, instead of the eager (non-lazy) path folding them
            // back together mid-batch after the 8th write.
            tree.update_node_log_odds_by_key(k, hit, true);
        }
        let nodes_before = tree.num_nodes();
        tree.prune();
        let nodes_after = tree.num_nodes();
        assert!(
            nodes_after < nodes_before,
            "prune should have collapsed the 8 identical siblings (before={nodes_before}, after={nodes_after})"
        );
        for &k in &keys {
            assert_eq!(tree.search(k, 0).unwrap().log_odds, hit);
        }
    }

    #[test]
    fn ray_with_end_outside_tree_bounds_returns_none() {
        let tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        // resolution 0.1 with tree_max_val = 32768 represents roughly
        // [-3276.8, 3276.7]; well outside that on either side must fail.
        assert!(
            tree.compute_ray_keys(origin, Point3::new(1.0e9, 0.0, 0.0))
                .is_none()
        );
        assert!(
            tree.compute_ray_keys(origin, Point3::new(-1.0e9, 0.0, 0.0))
                .is_none()
        );
    }

    #[test]
    fn ray_with_origin_outside_tree_bounds_returns_none() {
        let tree = OcTree::new(0.1);
        let origin = Point3::new(1.0e9, 0.0, 0.0);
        let end = Point3::new(0.0, 0.0, 0.0);
        assert!(tree.compute_ray_keys(origin, end).is_none());
    }

    #[test]
    fn zero_log_odds_is_occupied_under_the_default_threshold() {
        // Documents the non-obvious upstream boundary: isNodeOccupied uses
        // `>=`, and the default occupancy threshold (probability 0.5) is
        // exactly log-odds 0.0, so a node that nets to exactly unknown
        // reads as occupied, not unoccupied.
        let tree = OcTree::new(0.1);
        assert_eq!(tree.occupancy_thres_log(), 0.0);
        assert!(tree.is_node_occupied_log_odds(0.0));
    }

    #[test]
    fn unmapped_coordinate_has_no_occupancy() {
        let tree = OcTree::new(0.1);
        let p = Point3::new(5.0, 5.0, 5.0);
        assert!(tree.log_odds_at(p).is_none());
        assert!(tree.is_occupied(p).is_none());
    }

    #[test]
    fn node_size_scales_by_depth() {
        let tree = OcTree::new(0.1);
        assert_eq!(tree.node_size(OcTree::TREE_DEPTH), 0.1);
        assert_eq!(
            tree.node_size(0),
            0.1 * f64::from(1u32 << OcTree::TREE_DEPTH)
        );
    }

    #[test]
    fn insert_ray_marks_the_endpoint_occupied_and_the_path_free() {
        let mut tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(1.0, 0.0, 0.0);
        assert!(tree.insert_ray(origin, end, None, false));
        assert!(tree.is_occupied(end).unwrap());
        assert!(!tree.is_occupied(Point3::new(0.5, 0.0, 0.0)).unwrap());
    }

    #[test]
    fn insert_ray_cut_short_by_max_range_records_only_a_miss() {
        let mut tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(10.0, 0.0, 0.0);
        assert!(tree.insert_ray(origin, end, Some(1.0), false));
        // The ray was cut short at max_range, so upstream never guesses at
        // occupancy for `end`: it stays unmapped, not marked free or occupied.
        assert!(tree.log_odds_at(end).is_none());
        assert!(!tree.is_occupied(Point3::new(0.5, 0.0, 0.0)).unwrap());
    }

    #[test]
    fn compute_update_keeps_occupied_and_free_cells_disjoint() {
        let tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let hit = Point3::new(0.5, 0.0, 0.0);
        let (free, occupied) = tree.compute_update(&[hit], origin, None);
        let hit_key = tree.coord_to_key_checked(hit).unwrap();
        assert!(occupied.contains(&hit_key));
        assert!(!free.contains(&hit_key));
        assert!(!free.is_empty());
    }

    #[test]
    fn leaves_yields_every_leaf_with_its_coordinate_and_value() {
        let mut tree = OcTree::new(1.0);
        let hit_point = Point3::new(10.5, 0.5, 0.5);
        let miss_point = Point3::new(-10.5, 0.5, 0.5);
        tree.update_node(hit_point, true, false);
        tree.update_node(miss_point, false, false);

        let leaves: Vec<_> = tree.leaves().collect();
        assert_eq!(leaves.len(), 2);

        let mut saw_hit = false;
        let mut saw_miss = false;
        for leaf in &leaves {
            if (leaf.coordinate() - hit_point).norm() < 1e-9 {
                assert_eq!(leaf.log_odds(), tree.prob_hit_log());
                saw_hit = true;
            }
            if (leaf.coordinate() - miss_point).norm() < 1e-9 {
                assert_eq!(leaf.log_odds(), tree.prob_miss_log());
                saw_miss = true;
            }
        }
        assert!(saw_hit && saw_miss);
    }

    #[test]
    fn leaves_in_bbx_only_yields_leaves_overlapping_the_box() {
        let mut tree = OcTree::new(1.0);
        let inside = Point3::new(0.5, 0.5, 0.5);
        let outside = Point3::new(50.5, 0.5, 0.5);
        tree.update_node(inside, true, false);
        tree.update_node(outside, true, false);

        let leaves: Vec<_> = tree
            .leaves_in_bbx(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0))
            .expect("bbx corners are in range")
            .collect();
        assert_eq!(leaves.len(), 1);
        assert!((leaves[0].coordinate() - inside).norm() < 1e-9);
    }

    #[test]
    fn leaves_in_bbx_returns_none_for_an_out_of_range_corner() {
        let tree = OcTree::new(0.1);
        assert!(
            tree.leaves_in_bbx(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0e9, 0.0, 0.0))
                .is_none()
        );
    }
}
