// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_distance_field/include/moveit/collision_distance_field/collision_common_distance_field.hpp
//   moveit_core/collision_distance_field/src/collision_common_distance_field.cpp

//! The `RobotState`/`RobotModel`-dependent slice of `collision_common_distance_field`:
//! turning a [`Shape`] or a [`moveit_collision::Object`] into the posed
//! sphere/point decompositions [`crate::collision_distance_field_types`]
//! defines, with the same identity-keyed shape cache upstream uses.
//!
//! # Scope
//!
//! Ported: [`get_body_decomposition_cache_entry`] (upstream
//! `getBodyDecompositionCacheEntry`) and
//! [`collision_object_point_decomposition`] (upstream
//! `getCollisionObjectPointDecomposition`) — the two free functions that
//! need nothing beyond a [`Shape`]/[`moveit_collision::Object`] and are
//! therefore buildable from primitives this workspace already has.
//!
//! Deferred, and why:
//!
//! - **`GroupStateRepresentation`/`DistanceFieldCacheEntry`, as populatable
//!   structs.** Upstream's `DistanceFieldCacheEntry::link_names_` is "names
//!   of all links in the group and all links below the group (links that
//!   will move if any of the joints in the group move)" — i.e.
//!   `JointModelGroup::getUpdatedLinkModelNames()`. That query does not
//!   exist anywhere in this workspace yet: `moveit-model`'s own
//!   `JointModelGroup` doc comment explicitly defers `updated_link_model_*`
//!   to `moveit-state`, and `moveit-state` does not have it either. Without
//!   it there is no way to populate `link_names_`/`link_has_geometry_`/
//!   `link_body_indices_` faithfully, so these two struct are not ported
//!   this round. The actual cache-key-forming/-checking logic these structs
//!   exist to support --
//!   `generateDistanceFieldCacheEntry`/`getGroupStateRepresentation`/`compareCacheEntryToState`
//!   -- lives in `collision_env_distance_field.cpp`, explicitly out of scope
//!   this round; see the next paragraph for what was read there to inform
//!   this decision.
//! - **`getAttachedBodySphereDecomposition`/`getAttachedBodyPointDecomposition`.**
//!   Both take a `moveit::core::AttachedBody*`. `AttachedBody` does not
//!   exist anywhere in this workspace (`moveit-state`, `moveit-model`,
//!   `moveit-collision` all grepped clean) -- there is no type to accept as
//!   a parameter.
//! - **`getBodySphereVisualizationMarkers`.** Builds a
//!   `visualization_msgs::msg::MarkerArray` for RViz. PORTING-PLAN.md D1
//!   keeps ROS message types out of every crate but the optional
//!   `moveit-ros`.
//!
//! # What `collision_env_distance_field.cpp` was read for, without porting it
//!
//! `generateDistanceFieldCacheEntry` builds `state_values_` (every model
//! variable's position, mimic joints excluded) and `state_check_indices_`
//! (indices into `state_values_` for variables *not* in the group's active
//! joints -- i.e. the joints a cached entry promises did not move).
//! `compareCacheEntryToState` then re-reads the current `RobotState`'s
//! values at exactly those indices and invalidates the cache entry if any
//! differs from the stored value by more than `EPSILON = 0.001`. That is
//! the actual "cache key": a `DistanceFieldCacheEntry` is valid for a new
//! `RobotState` iff every joint variable *outside* the group is unchanged
//! to within `EPSILON`. Note this ties the key's shape to a specific group
//! -- a general, group-independent cache would need a different design.
//! Neither function is ported here; they need `JointModelGroup`'s
//! updated-link-model query (see above) to build `link_names_` in the first
//! place, so porting only the compare logic without a way to construct a
//! comparable entry would not close any gap this crate can verify.
//!
//! # Known upstream defect, ported byte-for-byte: the shape cache is resolution-blind
//!
//! [`get_body_decomposition_cache_entry`] preserves a genuine, upstream-
//! acknowledged defect: the cache is keyed *solely* on shape identity, but
//! the cached [`BodyDecomposition`] value also depends on `resolution` (it
//! is threaded into [`find_internal_points_convex`] via
//! [`BodyDecomposition::from_shapes`], which changes the interior point
//! sampling). Upstream's own comment in `getBodyDecompositionCacheEntry`
//! is literally `// TODO - deal with changing resolution?` -- the gap is
//! not inferred here, it is upstream's own admission. Calling this function
//! twice for the same shape with two different resolutions returns the
//! *first* call's decomposition both times; nothing about the mismatch is
//! observable from a single query in isolation, only from calling it twice
//! with different `resolution` arguments for the same shape and comparing
//! against a fresh, uncached rebuild -- see this module's tests.
//!
//! This is exactly the "a cache whose key omits something the value depends
//! on" defect class this file's task description flagged. It is preserved
//! rather than fixed for the same reason
//! [`crate::do_bounding_spheres_intersect`]'s squared-vs-unsquared defect
//! is preserved: this task's mandate is matching upstream's actual
//! behaviour, not upstream's intent, and "fixing" a cache key is a
//! behaviour change that needs its own sign-off, not a silent parity
//! deviation.
//!
//! In current upstream usage (per `collision_env_distance_field.cpp`, read
//! but not ported) this cache is reached only for world objects and
//! attached bodies -- single, reused shapes queried repeatedly at a fixed
//! resolution in practice. Robot links go through `addLinkBodyDecompositions`
//! instead, which builds each link's `BodyDecomposition` directly, once,
//! from *all* of that link's shapes together, and is itself in the
//! out-of-scope file.
//!
//! # Cache key: `Arc` identity via raw address, not a `Weak` upstream port
//!
//! Upstream keys `BodyDecompositionCache::map_` on `shapes::ShapeConstWeakPtr`
//! compared with `std::owner_less` -- ownership-based identity, not shape
//! *value* equality, and with no liveness check at lookup time (a lookup
//! against a since-expired weak pointer key that still exists in the map is
//! simply never matched by a live query, since `owner_less` still orders it
//! consistently). A `HashMap` needs `Hash`/`Eq`, which `std::sync::Weak`
//! does not implement in Rust, so this port keys on `Arc::as_ptr(shape) as
//! usize` instead: the pointer's numeric address. This is an equally
//! faithful identity comparison -- two live `Arc`s alias the same
//! allocation iff `Arc::as_ptr` agrees, exactly matching `owner_less` -- and
//! is simpler and sound: the address is never dereferenced, only compared,
//! so a key surviving past its shape's last `Arc` being dropped is inert
//! (it just never matches a future live shape, since addresses are not
//! reused while any `Arc` referencing this cache could still exist --
//! ported values are held by `Arc<BodyDecomposition>` and returned to
//! callers, not by the shape itself, so there is no reuse hazard from the
//! cache's own entries).
//!
//! # Locking pattern matches upstream, redundant-computation race included
//!
//! Upstream's `getBodyDecompositionCacheEntry` takes the lock, checks for
//! an existing entry, and *releases the lock* before constructing a fresh
//! `BodyDecomposition` (an allocation-heavy, unbounded-cost operation upstream
//! evidently did not want to hold a mutex through), then re-locks only to
//! insert. Two threads racing on the same not-yet-cached shape can both
//! miss the first check, both build a fresh decomposition, and both insert
//! -- upstream's `cache.map_[wptr] = bdcp;` on the second lock
//! unconditionally overwrites, so the second inserter's result simply wins,
//! with the first thread's work discarded and returned to its own caller
//! (both callers still get a valid, self-consistent -- if not
//! object-identical -- `BodyDecomposition`). This port reproduces that
//! exact lock/unlock/build/lock/insert shape (including the unconditional
//! overwrite) rather than closing the race with a single held lock, since
//! closing it would be a straightforward correctness improvement upstream
//! itself does not have and this task's mandate is matching upstream
//! behaviour.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

use moveit_collision::Object;
use moveit_error::Result;
use moveit_geometry::Shape;

use crate::collision_distance_field_types::{
    BodyDecomposition, PosedBodyPointDecomposition, PosedBodyPointDecompositionVector,
};

/// The process-wide shape decomposition cache. Upstream's file-local
/// `getBodyDecompositionCache()` function-local `static BodyDecompositionCache
/// cache;`. Upstream's `clean_count_`/`MAX_CLEAN_COUNT` bookkeeping exists
/// only to gate a cache-eviction pass upstream itself never implemented
/// (`// TODO - clean cache`, right where `clean_count_` would be read) --
/// there is no behaviour to port, so this cache never evicts, matching
/// upstream's actual (not intended) behaviour exactly.
fn body_decomposition_cache() -> &'static Mutex<HashMap<usize, Arc<BodyDecomposition>>> {
    static CACHE: OnceLock<Mutex<HashMap<usize, Arc<BodyDecomposition>>>> = OnceLock::new();
    CACHE.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Upstream free function `getBodyDecompositionCacheEntry`.
///
/// See this module's doc comment for the resolution-blind cache-key defect
/// this preserves, and for why the key is `Arc::as_ptr(shape) as usize`
/// rather than a `Weak`-based port of `shapes::ShapeConstWeakPtr`.
///
/// `padding` is always [`BodyDecomposition::DEFAULT_PADDING`], matching
/// upstream's `BodyDecomposition(shape, resolution)` two-argument
/// constructor call, which relies on that header's `padding = 0.01` default
/// argument -- Rust has no default parameters, so this port hardcodes the
/// same value at this one call site rather than accepting a `padding`
/// parameter this function's own upstream signature does not have.
///
/// # Errors
///
/// See [`BodyDecomposition::new`].
pub fn get_body_decomposition_cache_entry(
    shape: &Arc<Shape>,
    resolution: f64,
) -> Result<Arc<BodyDecomposition>> {
    let key = Arc::as_ptr(shape) as usize;

    {
        let cache = body_decomposition_cache()
            .lock()
            .expect("body decomposition cache mutex poisoned");
        if let Some(existing) = cache.get(&key) {
            return Ok(Arc::clone(existing));
        }
    }

    let decomposition = Arc::new(BodyDecomposition::new(
        shape,
        resolution,
        BodyDecomposition::DEFAULT_PADDING,
    )?);

    let mut cache = body_decomposition_cache()
        .lock()
        .expect("body decomposition cache mutex poisoned");
    cache.insert(key, Arc::clone(&decomposition));
    Ok(decomposition)
}

/// Upstream free function `getCollisionObjectPointDecomposition`.
///
/// Upstream builds each [`PosedBodyPointDecomposition`] at identity via the
/// one-argument constructor, adds it to the result vector, then re-poses it
/// in place through the vector's own `updatePose`. This port calls
/// [`PosedBodyPointDecomposition::with_pose`] directly instead -- posing
/// once at construction rather than once at identity and then again at the
/// real pose -- which lands on the same final `posed_collision_points` for
/// every point (`transform_as_point` at identity is the identity function,
/// so upstream's extra step is redundant, not semantically different; see
/// [`PosedBodyPointDecomposition::new`]'s own doc comment for the same
/// observation).
///
/// # Errors
///
/// [`moveit_error::Error::Construct`] if any of `obj`'s shapes has no
/// `bodies::` counterpart -- see [`get_body_decomposition_cache_entry`]/
/// [`BodyDecomposition::new`].
pub fn collision_object_point_decomposition(
    obj: &Object,
    resolution: f64,
) -> Result<PosedBodyPointDecompositionVector> {
    let mut result = PosedBodyPointDecompositionVector::new();
    for shape_entry in obj.shapes() {
        let body_decomposition =
            get_body_decomposition_cache_entry(shape_entry.shape(), resolution)?;
        result.add_to_vector(PosedBodyPointDecomposition::with_pose(
            body_decomposition,
            shape_entry.global_pose(),
        ));
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use moveit_geometry::{Isometry3, Sphere};
    use nalgebra::Translation3;

    fn sphere_shape(radius: f64) -> Arc<Shape> {
        Arc::new(Shape::Sphere(Sphere::new(radius).unwrap()))
    }

    /// Invariant boundary: two different `Arc<Shape>` instances with
    /// *identical* shape values must get independent cache entries --
    /// proving the cache is keyed on identity, not on shape value, matching
    /// upstream's `owner_less`-on-`ShapeConstWeakPtr` semantics (see this
    /// module's doc comment).
    #[test]
    fn cache_entry_is_keyed_on_shape_identity_not_shape_value() {
        let shape_a = sphere_shape(0.2);
        let shape_b = sphere_shape(0.2);

        let entry_a = get_body_decomposition_cache_entry(&shape_a, 0.05).unwrap();
        let entry_b = get_body_decomposition_cache_entry(&shape_b, 0.05).unwrap();

        assert!(
            !Arc::ptr_eq(&entry_a, &entry_b),
            "two distinct Arc<Shape> with equal values must not share a cache entry"
        );
    }

    /// The same shape queried twice returns the exact same cached `Arc`,
    /// not merely an equal one -- upstream's whole point in having a cache
    /// at all.
    #[test]
    fn cache_entry_is_reused_for_the_same_shape() {
        let shape = sphere_shape(0.2);

        let first = get_body_decomposition_cache_entry(&shape, 0.05).unwrap();
        let second = get_body_decomposition_cache_entry(&shape, 0.05).unwrap();

        assert!(
            Arc::ptr_eq(&first, &second),
            "the same Arc<Shape> queried twice must hit the cache"
        );
    }

    /// The documented defect, pinned concretely: the *same* `Arc<Shape>`
    /// queried at two different resolutions returns the *first* call's
    /// decomposition both times, even though a fresh, uncached rebuild at
    /// the second resolution produces a different interior point count.
    /// This is the exact shape of bug the module doc warns will not show up
    /// in any single-query test -- it only appears by calling the cached
    /// entry point twice and comparing against an uncached rebuild.
    #[test]
    fn cache_entry_ignores_a_changed_resolution_for_the_same_shape() {
        // A sphere large enough that resolution actually changes the
        // interior point count `find_internal_points_convex` produces.
        let shape = sphere_shape(1.0);

        let coarse = get_body_decomposition_cache_entry(&shape, 0.5).unwrap();
        let cached_at_fine_request = get_body_decomposition_cache_entry(&shape, 0.01).unwrap();

        assert!(
            Arc::ptr_eq(&coarse, &cached_at_fine_request),
            "documented defect: a second call with a different resolution still \
             returns the first call's cached decomposition"
        );

        let fresh_fine =
            BodyDecomposition::new(&shape, 0.01, BodyDecomposition::DEFAULT_PADDING).unwrap();
        assert_ne!(
            cached_at_fine_request.collision_points().len(),
            fresh_fine.collision_points().len(),
            "test setup must pick resolutions that actually produce different point counts, \
             otherwise this test cannot distinguish the cached-stale value from a correct one"
        );
    }

    #[test]
    fn collision_object_point_decomposition_poses_every_shape_by_its_global_pose() {
        let mut world = moveit_collision::World::new();
        let object_pose = Isometry3::from_parts(
            Translation3::new(1.0, 0.0, 0.0),
            nalgebra::UnitQuaternion::identity(),
        );
        let shape_pose = Isometry3::from_parts(
            Translation3::new(0.0, 2.0, 0.0),
            nalgebra::UnitQuaternion::identity(),
        );
        world
            .add_to_object(
                "obj",
                object_pose,
                &[sphere_shape(0.3)],
                std::slice::from_ref(&shape_pose),
            )
            .unwrap();
        let obj = world.get_object("obj").unwrap();

        let result = collision_object_point_decomposition(&obj, 0.05).unwrap();

        assert_eq!(result.len(), 1);
        let global_pose = object_pose * shape_pose;
        let expected = result.get(0).unwrap().collision_points().to_vec();
        // Every posed point must equal the unposed point transformed by the
        // shape's global pose, not merely the object's own pose.
        let direct = PosedBodyPointDecomposition::with_pose(
            get_body_decomposition_cache_entry(obj.shapes()[0].shape(), 0.05).unwrap(),
            global_pose,
        );
        assert_eq!(expected, direct.collision_points());
    }
}
