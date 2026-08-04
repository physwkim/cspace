# moveit-distance-field claim audit

Per PORTING-PLAN.md §175. One row per doc-comment claim about upstream
behavior, found in `crates/moveit-distance-field/src/*.rs`. Appended as
found, not batched. `evidence` is an upstream `file:line` actually opened
this round — not inferred from the port.

Upstream root: `/home/stevek/work/moveit2` @ `e017c91ee12984393a28ba246075c65f69cde3bf`
(`moveit_core/distance_field/`, `moveit_core/collision_distance_field/`).

Round 27 (this file's first population): re-derived from source. The
round-26 93-item list (2 background agents, 50+43 items) was lost to
context compaction before being written to disk (see §175) and is not
recoverable; this table does not attempt to reconcile against it.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| lib.rs:41-43 | The two continuous-state `checkRobotCollision` overloads are unported because upstream itself stubs both to "not implemented" | CONFIRMED | collision_env_distance_field.cpp:1502-1515 | |
| lib.rs:43-44 | The header-inline `distanceSelf`/`distanceRobot` stubs unconditionally return `0.0` or log "Not implemented", never reaching the `gsr`-cache machinery | CONFIRMED | collision_env_distance_field.hpp:109-124,179-199 | |
| lib.rs:53-54 | `DistanceField` is upstream's abstract base class (D4: ported as a trait instead) | CONFIRMED | distance_field.hpp:92 | |
| lib.rs:291-297 | `distance_field.hpp` has 32 public methods (26 ported, 2 unported, 4 D-excluded), 2 protected methods, 8 protected fields | CONFIRMED | distance_field.hpp:92-615 | |
| lib.rs:302-337 | Per-symbol table for `distance_field.hpp` names/signatures | CONFIRMED | distance_field.hpp:92-615 | |
| lib.rs:345-346,984-991 | `inv_twice_resolution_` is declared `int` while computed as `1.0/(2.0*resolution_)` — mistyped, silently truncating | CONFIRMED | distance_field.hpp:614; distance_field.cpp:67 | |
| lib.rs:348-373 | `collision_distance_field_types.hpp` symbol/method counts (34 public methods + 11 ctors + 5 free fns + 3 D-excluded; `BodyDecomposition` has 8 methods) | CONFIRMED | collision_distance_field_types.hpp:63-536 | |
| lib.rs:377-436 | Per-symbol table for `collision_distance_field_types.hpp` names/signatures | CONFIRMED | collision_distance_field_types.hpp:63-536 | |
| lib.rs:593-598 | `BodyDecompositionVector` is forward-declared/friended but never defined anywhere upstream | CONFIRMED | collision_distance_field_types.hpp:226,230 (full-tree grep, no other hit) | |
| lib.rs:611-618 | `PosedBodyPointDecomposition::from_octree` iterates every octree node, not just occupied leaves — faithfully-reproduced upstream behavior | CONFIRMED | collision_distance_field_types.cpp:355-365 | |
| lib.rs:549-570 | `collision_common_distance_field.hpp` symbol table names exist as claimed | CONFIRMED | collision_common_distance_field.hpp:47-181 | |
| lib.rs:555-558 | `getAttachedBodySphereDecomposition`'s sole upstream caller is `getGroupStateRepresentation` | CONFIRMED | collision_env_distance_field.cpp:1238-1239 | |
| lib.rs:563-567 | `getAttachedBodyPointDecomposition`'s sole upstream caller is `generateDistanceFieldCacheEntry`'s non-group-link loop | CONFIRMED | collision_env_distance_field.cpp:907-929 | |
| lib.rs:628-661 | `CollisionEnvDistanceField`'s unported surface enumeration | CONFIRMED | collision_env_distance_field.hpp:59-309 | |
| lib.rs:668-670 | `checkSelfCollisionHelper`: all 4 `checkSelfCollision` overloads collapse into it | CONFIRMED | collision_env_distance_field.hpp:92-104; .cpp:235-272 | |
| lib.rs:671-672 | `checkCollision`'s 2 real overloads out of 4 total | CONFIRMED | collision_env_distance_field.hpp:148-158; .cpp:1382-1429 | |
| lib.rs:673-674 | `checkRobotCollision`'s 2 real (non-continuous) overloads | CONFIRMED | collision_env_distance_field.cpp:1447-1500 | |
| lib.rs:675-678 | `getCollisionGradients`'s upstream `res` parameter is commented `/*res*/`, never read/written | CONFIRMED | collision_env_distance_field.cpp:1517 vs hpp:213 | |
| lib.rs:697-702 | `getPosedLinkBodySphereDecomposition`/`getPosedLinkBodyPointDecomposition` are trivial wrappers, only 2 callers | CONFIRMED | collision_env_distance_field.cpp:1081-1098,920,1178 | |
| lib.rs:707-716 | `getSelfCollisions`'s loop bound covers link + attached-body counts | CONFIRMED | collision_env_distance_field.cpp:278-320 | |
| lib.rs:717-724 | `getSelfProximityGradients` has no attached-body gap: upstream's own loop never extends past `link_names_.size()` — dead code in the C++ itself | CONFIRMED | collision_env_distance_field.cpp:359 | |
| lib.rs:725-728 | `getIntraGroupCollisions` checks link-attached and attached-attached pairs in upstream | CONFIRMED | collision_env_distance_field.cpp:430-508 | |
| lib.rs:729-732 | `getIntraGroupProximityGradients` has the same attached-body coverage as `getIntraGroupCollisions` | CONFIRMED | collision_env_distance_field.cpp:644-652 | |
| lib.rs:733-738 | `getEnvironmentCollisions`'s loop bound is link + attached-body counts | CONFIRMED | collision_env_distance_field.cpp:1565 | |
| lib.rs:739-748 | `getEnvironmentProximityGradients`'s loop bound never advances past link count — dead code in the C++ itself | CONFIRMED | collision_env_distance_field.cpp:1649 | |
| lib.rs:749-752 | `updatedPaddingOrScaling` is a no-op override with empty `{}` body | CONFIRMED | collision_env_distance_field.hpp:270 | |
| lib.rs:769-774 | `update_cache_lock_` exists only as a `const`-method workaround (`const_cast`) | CONFIRMED | collision_env_distance_field.cpp:170-171 | |
| lib.rs:777-784 | `in_group_update_map_` is not touched by `generateCollisionCheckingStructures`'s own body; read only inside `generateDistanceFieldCacheEntry` | CONFIRMED | collision_env_distance_field.cpp:158-175 vs :909 | |
| lib.rs:786-794 | `pregenerated_group_state_representation_map_` is populated only inside `initialize()` | CONFIRMED | collision_env_distance_field.cpp:126-154 | |
| lib.rs:791-794 | Read in exactly one place: `getGroupStateRepresentation`'s "already pregenerated" branch | CONFIRMED | collision_env_distance_field.cpp:1212-1227 | |
| lib.rs:840-843 | `planning_scene_` is built once in `initialize()`, used only to source a default-empty ACM for the pregeneration loop | CONFIRMED | collision_env_distance_field.cpp:139,153 | |
| lib.rs:865-863 | `.h` forwarding shims are all identical 3-line deprecated shims citing moveit/moveit2#3113 | CONFIRMED | distance_field.h:1-52; collision_common_distance_field.h:1-52 | |
| lib.rs:865-923 | `voxel_grid.hpp` per-symbol table names/signatures | CONFIRMED | voxel_grid.hpp:44-334 | |
| lib.rs:915-916 | `num_cells_total_` is not separately stored by this port; upstream has it as a field | CONFIRMED | voxel_grid.hpp:295 | |
| lib.rs:917-922 | `data_ptrs_` is dead in upstream itself — full-tree grep returns only its own declaration | CONFIRMED | voxel_grid.hpp:288 (grep, no other hit) | |
| lib.rs:924-991 | `distance_field.hpp` per-symbol table names/signatures | CONFIRMED | distance_field.hpp:92-615 | |
| lib.rs:943-946 | `addOcTreeToField`'s algorithm: occupancy-filter + subdivision, ported as default trait method | CONFIRMED | distance_field.cpp:240-291 | |
| lib.rs:993-1052 | `propagation_distance_field.hpp` per-symbol table names/signatures | CONFIRMED | propagation_distance_field.hpp:47-608 | |
| lib.rs:1021-1024 | `getNearestCell` closes a real upstream UB-in-practice pointer-aliasing defect rather than reproducing it | CONFIRMED (cross-reference resolves to propagation.rs:305-342's own doc, already independently CONFIRMED at row above; opened `getNearestCell` itself and traced its full body: `pos = cell->closest_point_` can hold the `UNINITIALIZED` sentinel for a cell farther than `max_distance` from every obstacle, `getCell(pos...)` reads that sentinel as a grid index unconditionally per upstream's own "MUST be valid or data corruption (SEGFAULTS) will occur" doc, and the resulting out-of-bounds-but-undereferenced pointer fails the `ncell == cell` identity check upstream relies on to detect "unknown" -- so upstream's own contract ("if nearest cell is unknown, return nullptr") is unmet for this ordinary, reachable query. This port validates `pos` first instead of relying on pointer identity. Note: "pointer aliasing" is a loose label for the mechanism -- the defect is an out-of-bounds index construction that evades an identity-based validity check, not classic same-object aliasing -- but no fact in the claim is wrong, so this is a terminology note, not a fix) | propagation_distance_field.hpp:396-437 | |
| lib.rs:1067-1080 | No Euclidean-vs-Manhattan distance-metric mode to port: all 3 ctors take one bool; `PropagationDistanceField` is the only `public DistanceField` subclass | CONFIRMED | propagation_distance_field.hpp:172,198,223,144; full-tree grep for manhattan/chebyshev/euclidean → 0 hits | |
| lib.rs:1082-1087 | `findInternalPointsConvex` ported as `find_internal_points_convex`, generic over `ConvexBody` vs upstream's concrete `bodies::Body` | CONFIRMED | find_internal_points.hpp:54 | |
| find_internal_points.rs:11-20 | `ConvexBody` trait narrows dependency to `computeBoundingSphere`+`containsPoint`; convex correctness is caller's responsibility | CONFIRMED | find_internal_points.hpp:44-54; find_internal_points.cpp:39-64 | |
| find_internal_points.rs:22-24 | `bounding_sphere` doc quotes upstream `bodies::Body::computeBoundingSphere` | CONFIRMED | geometric_shapes/bodies.h:52-56,234 | |
| find_internal_points.rs:25-26 | `contains_point` doc quotes upstream `bodies::Body::containsPoint` | CONFIRMED | geometric_shapes/bodies.h:210 | |
| find_internal_points.rs:29-31 | `find_internal_points_convex` implements upstream's grid-construction/containment-test algorithm | CONFIRMED | find_internal_points.cpp:39-64 | |
| propagation.rs:17-33 | `PropDistanceFieldVoxel` ctors: upstream's `(int,int)` ctor leaves `update_direction_`/`negative_update_direction_` uninitialized; port additionally sets them to `UNINITIALIZED` | CONFIRMED | propagation_distance_field.hpp:82-116,589-602 | |
| propagation.rs:36-40 | `distance_square` = upstream `distance_square_` | CONFIRMED | propagation_distance_field.hpp:106 | |
| propagation.rs:42-44 | `negative_distance_square` = upstream `negative_distance_square_` | CONFIRMED | propagation_distance_field.hpp:107 | |
| propagation.rs:84-92 | `build_neighborhoods` = upstream `initNeighborhoods` algorithm | CONFIRMED | propagation_distance_field.cpp:526-582 | |
| propagation.rs:135-138 | `squared_distance` = upstream `(a-b).squaredNorm()` on `Vector3i` | CONFIRMED | propagation_distance_field.cpp:426,483 | |
| propagation.rs:169-179 | Struct maps to `PropagationDistanceField`; octree/istream ctors and stream I/O exist upstream, simply not ported | CONFIRMED | propagation_distance_field.hpp:172-223,358,376; .cpp:56-84,635-760 | |
| propagation.rs:180-186 | Upstream `initialize()` computes `max_distance_sq_` with no finite/positive check, narrowing into `int` | CONFIRMED | propagation_distance_field.cpp:86-104; .hpp:565 | |
| propagation.rs:199-217 | `max_distance_sq_` narrowing shape matches `inv_twice_resolution_`'s | CONFIRMED | propagation_distance_field.cpp:88; distance_field.hpp:614; .cpp:67 | |
| propagation.rs:234-238 | `new()` maps to upstream ctor `PropagationDistanceField(...)` | CONFIRMED | propagation_distance_field.hpp:172-174 | |
| propagation.rs:240-257 | `propagate_negative_distances` gates the same call sites as upstream `propagate_negative_`, at exact cited lines; `reset()` still unconditionally zeroes negative fields when disabled | CONFIRMED | propagation_distance_field.cpp:226,240,251,312,333,384,506-522; .hpp:165-167 | |
| propagation.rs:292-295 | `cell()` maps to `getCell`, "MUST be valid or SEGFAULTS" doc | CONFIRMED | propagation_distance_field.hpp:384-398 | |
| propagation.rs:300,565,598,670,682,740 | Trivial 1:1 mappings (max_distance_squared, propagate_positive/negative, add/remove_points_to_field, reset) | CONFIRMED | propagation_distance_field.hpp:450-453; .cpp:177,198,390,446,506 | |
| propagation.rs:146-167 | `NearestCell` mirrors `getNearestCell`'s out-params/return contract | CONFIRMED | propagation_distance_field.hpp:400-440 | |
| propagation.rs:305-342 | Upstream `getNearestCell` unconditionally dereferences `pos` without validity-checking despite its own "MUST be valid" doc — fails its own "return nullptr if unknown" contract for cells farther than max_distance | CONFIRMED | propagation_distance_field.hpp:418-440,384-398 | |
| propagation.rs:392 | `add_new_obstacle_voxels` maps to `addNewObstacleVoxels` | CONFIRMED | propagation_distance_field.cpp:221 | |
| propagation.rs:483-491 | `remove_obstacle_voxels` maps to `removeObstacleVoxels`; upstream's `negative_stack` there is dead code | CONFIRMED | propagation_distance_field.cpp:303-388 (declared :307, reserved :314, never read again) | |
| propagation.rs:694-709 | `old_not_new`/`new_not_old` both "feed `addNewObstacleVoxels`" | EXPIRED — `old_not_new` actually feeds `removeObstacleVoxels`; only the filtered `new_not_old` feeds `addNewObstacleVoxels` | propagation_distance_field.cpp:164-175 | 3b7fb22 |
| voxel_grid.rs:11-13 | `Dimension` maps to upstream `Dimension` (`DIM_X`/`DIM_Y`/`DIM_Z`) | CONFIRMED | voxel_grid.hpp:46-52 | |
| voxel_grid.rs:24-32 | Upstream spells size/origin as six separate `f64` ctor args | CONFIRMED | voxel_grid.hpp:91-92 | |
| voxel_grid.rs:60-66 | `GridGeometry::new` validates what upstream's ctor/`resize()` does not | CONFIRMED | voxel_grid.hpp:363-396 | |
| voxel_grid.rs:67-92 | Upstream's `num_cells_ = size_ * oo_resolution_` narrows into `int` with no overflow guard | CONFIRMED | voxel_grid.hpp:294,384 | |
| voxel_grid.rs:122-125 | `VoxelGrid<T>` maps to upstream `VoxelGrid<T>` | CONFIRMED | voxel_grid.hpp:60-62 | |
| voxel_grid.rs:129-133 | Upstream default ctor + `resize()` exist for pre-size-known construction | CONFIRMED | voxel_grid.hpp:96-121,346-361,363-396 | |
| voxel_grid.rs:134-142 | Upstream ctor allocates `new T[n]` default-constructed; `PropDistanceFieldVoxel`'s default ctor leaves it uninitialized | CONFIRMED | voxel_grid.hpp:393-396; propagation_distance_field.hpp:600-602 | |
| voxel_grid.rs:143-146 | `getCell`/`setCell` document "corruption and/or SEGFAULTS" | CONFIRMED | voxel_grid.hpp:152-154,171-172 | |
| voxel_grid.rs:147-159 | Upstream declares exactly five `Eigen::Vector3`-taking convenience overloads, each a one-line forward | CONFIRMED | voxel_grid.hpp:137,157,159,180,273,464-467,481-491,499-503,410-414 | |
| voxel_grid.rs:160-166 | `data_ptrs_` is dead code in upstream itself | CONFIRMED | voxel_grid.hpp:288 (sole occurrence) | |
| voxel_grid.rs:182-186 | `VoxelGrid::new` maps to upstream ctor | CONFIRMED | voxel_grid.hpp:91-92,339-344 | |
| voxel_grid.rs:226,231,236,241,246,251,261,284 | Trivial 1:1 mappings (reset, size, resolution, origin, num_cells, is_cell_valid×2, set_cell) | CONFIRMED | voxel_grid.hpp:196-221,272-283,405-420,429-450,494-503,532-535 | |
| voxel_grid.rs:270-272 | `get_cell()` maps to `getCell(int,int,int) const`; panics in place of "corruption/SEGFAULTS" | CONFIRMED | voxel_grid.hpp:156-159,470-479 | |
| voxel_grid.rs:277-278 | `get_cell_mut()` maps to non-const `getCell` overload | CONFIRMED | voxel_grid.hpp:156,470-473 | |
| voxel_grid.rs:290-292 | `get()` maps to `operator()(double,double,double)`, out-of-bounds returns default object | CONFIRMED | voxel_grid.hpp:136,453-461 | |
| voxel_grid.rs:303-312 | `cell_from_location` = `getCellFromLocation`, pre-shifted-origin form algebraically identical to documented formula | CONFIRMED | voxel_grid.hpp:506-523 | |
| voxel_grid.rs:313-336 | Non-finite `loc` handling deviates from upstream UB; sentinel matches "computed even if invalid" contract | CONFIRMED | voxel_grid.hpp:244-259,506-523 | |
| voxel_grid.rs:345-349 | `location_from_cell` = `getLocationFromCell` | CONFIRMED | voxel_grid.hpp:324-333,526-529 | |
| voxel_grid.rs:351-361 | `grid_to_world` collapses two upstream overloads into one | CONFIRMED | voxel_grid.hpp:241-242,538-551 | |
| voxel_grid.rs:370-380 | `world_to_grid` collapses two upstream overloads; matches "computed even if invalid" contract | CONFIRMED | voxel_grid.hpp:260-261,554-569,244-259 | |
| voxel_grid.rs:446-449 (test) | `size/resolution` exactly at `i32::MAX` is well-defined upstream too | CONFIRMED | voxel_grid.hpp:384 | |
| voxel_grid.rs:456-460 (test) | One cell past `i32::MAX` is UB upstream, no value to match | CONFIRMED | voxel_grid.hpp:384 | |
| collision_common_distance_field.rs:9-30 | Module ports `getBodyDecompositionCacheEntry`, `getCollisionObjectPointDecomposition`, `getAttachedBodySphereDecomposition`/`getAttachedBodyPointDecomposition`, relies on `getUpdatedLinkModelNames`/`getUpdatedLinkModelsWithGeometryNames` | CONFIRMED | collision_common_distance_field.cpp:77,101,115,129; collision_env_distance_field.cpp:145,735 | |
| collision_common_distance_field.rs:32-34 | `GroupStateRepresentation` ported from upstream, owning only 4 named members | CONFIRMED | collision_common_distance_field.hpp:56-103 | |
| collision_common_distance_field.rs:40-44 | `getGroupStateRepresentation`/`updateGroupStateRepresentationState`/`getDistanceFieldCacheEntry` all live in collision_env_distance_field.cpp | CONFIRMED | collision_env_distance_field.cpp:202,1103,1157 | |
| collision_common_distance_field.rs:65-76 | `getAttachedBodySphereDecomposition`/`getAttachedBodyPointDecomposition`'s sole respective callers | CONFIRMED | collision_env_distance_field.cpp:928,1239 (grep, no other callers) | |
| collision_common_distance_field.rs:85-103 | `state_values_` includes mimic joints despite stale header comment claiming exclusion; `EPSILON=0.001` | CONFIRMED | collision_env_distance_field.cpp:55,874-896,1254-1279; collision_common_distance_field.hpp:124-127 | |
| collision_common_distance_field.rs:105-119 | Shape cache keyed only on identity, ignoring resolution — upstream's own comment says `// TODO - deal with changing resolution?` | CONFIRMED | collision_common_distance_field.cpp:79 | |
| collision_common_distance_field.rs:130-136 | Cache reached only for world objects/attached bodies; robot links go through `addLinkBodyDecompositions` | CONFIRMED | collision_common_distance_field.cpp:108,122,136; collision_env_distance_field.cpp:960-979,1763 | |
| collision_common_distance_field.rs:138-145 | Cache keyed on `ShapeConstWeakPtr` compared with `std::owner_less` | CONFIRMED | collision_common_distance_field.cpp:57-60 | |
| collision_common_distance_field.rs:147-161 | A `weak_ptr` map key keeps the control block alive for the entry's lifetime | CONFIRMED | collision_common_distance_field.cpp:57-60 (std library semantics) | |
| collision_common_distance_field.rs:163-172 | Cache never evicts, matching upstream's own `// TODO - clean cache`; `clean_count_` written but never read | CONFIRMED | collision_common_distance_field.cpp:62-98 | |
| collision_common_distance_field.rs:174-191 | Upstream locks/checks/unlocks/builds/relocks and unconditionally overwrites via `cache.map_[wptr] = bdcp;` | CONFIRMED | collision_common_distance_field.cpp:77-99 | |
| collision_common_distance_field.rs:212-226 | `compareCacheEntryToState` re-derives attached bodies from `dfce->state_->getAttachedBodies()` at comparison time | CONFIRMED | collision_env_distance_field.cpp:1282 | |
| collision_common_distance_field.rs:228-233 | `compareCacheEntryToAllowedCollisionMatrix`'s own attached-bodies fetch is dead code | CONFIRMED | collision_env_distance_field.cpp:1314-1370 (fetched :1322-1323, never referenced again) | |
| collision_common_distance_field.rs:247-252 | Upstream compares `getShapes()[j] != getShapes()[j]`, pointer identity via `shared_ptr::operator!=` | CONFIRMED | collision_env_distance_field.cpp:1305 | |
| collision_common_distance_field.rs:295-306 | `attached_body_names_`/`attached_body_link_state_indices_` populated one entry per attached body on a geometry-bearing, ACM-tracked link | CONFIRMED | collision_env_distance_field.cpp:770-823 | |
| collision_common_distance_field.rs:307-314 | `acm_` stays default (empty) when `generateDistanceFieldCacheEntry`'s `acm` arg is null | CONFIRMED | collision_env_distance_field.cpp:724-732 | |
| collision_common_distance_field.rs:335-343 | `distance_field_` covers every link not in the group, including attached-body points on such links | CONFIRMED | collision_env_distance_field.cpp:907-933 | |
| collision_common_distance_field.rs:344-347 | `link_names_` comes from `getUpdatedLinkModelNames` | CONFIRMED | collision_env_distance_field.cpp:735 | |
| collision_common_distance_field.rs:355-361 | Upstream searches `getUpdatedLinkModels()` for the matching link, stores its position in `link_state_indices_` | CONFIRMED | collision_env_distance_field.cpp:744,754-762 | |
| collision_common_distance_field.rs:369-377 | `attached_bodies` is not an upstream field name | CONFIRMED | collision_common_distance_field.hpp:111-167 (no such field) | |
| collision_common_distance_field.rs:378-387 | `self_collision_enabled_` has one entry per link then one per attached body | CONFIRMED | collision_env_distance_field.cpp:745,799-808,840-849 | |
| collision_common_distance_field.rs:388-390 | `intra_group_collision_enabled_` is a square matrix, same indexing both axes | CONFIRMED | collision_env_distance_field.cpp:741-746,783 | |
| collision_common_distance_field.rs:393-401 | `GroupStateRepresentation` built by `getGroupStateRepresentation`, refreshed by `updateGroupStateRepresentationState` | CONFIRMED | collision_env_distance_field.cpp:1103,1157 | |
| collision_common_distance_field.rs:404-415 | `dfce_` is a shared, reference-counted `DistanceFieldCacheEntryConstPtr` | CONFIRMED | collision_common_distance_field.hpp:84 | |
| collision_common_distance_field.rs:416-422 | `attached_body_decompositions_` populated one entry per `dfce.attached_body_names_` | CONFIRMED | collision_env_distance_field.cpp:1229-1251 | |
| collision_common_distance_field.rs:423-429 | Custom copy ctor exists only for the "pregenerated" reuse path | CONFIRMED | collision_common_distance_field.hpp:61-81; collision_env_distance_field.cpp:1214 | |
| collision_common_distance_field.rs:434-436 | `link_body_decompositions_` null for a link without geometry | CONFIRMED | collision_env_distance_field.cpp:1206-1209 | |
| collision_common_distance_field.rs:441-443 | `link_distance_fields_` likewise null for a link without geometry | CONFIRMED | collision_env_distance_field.cpp:1208 | |
| collision_common_distance_field.rs:455-464 | Cache is a function-local `static` inside file-local `getBodyDecompositionCache()` | CONFIRMED | collision_common_distance_field.cpp:71-75 | |
| collision_common_distance_field.rs:471-482 | `padding` resolves to header default `0.01` | CONFIRMED | collision_distance_field_types.hpp (ctor default); collision_common_distance_field.cpp:91 | |
| collision_common_distance_field.rs:515-532 | Upstream builds via one-arg ctor then `updatePose`, functionally equal to posing once | CONFIRMED | collision_common_distance_field.cpp:101-113; collision_distance_field_types.cpp:342-346,367-378 | |
| collision_common_distance_field.rs:549-560 | `getAttachedBodySphereDecomposition` poses via already-world-resolved `getGlobalCollisionBodyTransforms()[i]` | CONFIRMED | collision_common_distance_field.cpp:115-127 | |
| collision_common_distance_field.rs:582-585 | `getAttachedBodyPointDecomposition` reads the same transforms | CONFIRMED | collision_common_distance_field.cpp:129-141 | |
| distance_field.rs:18-19 | `bodies::Body::computeBoundingSphere`/`containsPoint` are pure virtual, dispatched per body kind | CONFIRMED | geometric_shapes/bodies.h:203-234 | |
| distance_field.rs:31-39 | `getShapePoints`/`addShapeToField`/`moveShapeInField` never call `setScale`/`setPadding` on the constructed body | CONFIRMED | distance_field.cpp:208-231,293-312 | |
| distance_field.rs:50-61 | `getOcTreePoints` (protected) has exactly two callers | CONFIRMED | distance_field.cpp:211-220,286-291 | |
| distance_field.rs:63-68 | `bbx_min`/`bbx_max` are `gridToWorld(0,0,0)`/`gridToWorld(num_x,num_y,num_z)` | CONFIRMED | distance_field.cpp:243-256 | |
| distance_field.rs:70-73 | A failed bbox-to-key conversion yields an immediately-empty iterator, not an error | CONFIRMED | octomap/OcTreeIterator.hxx:335-360 | |
| distance_field.rs:75-91 | Upstream's own loop has identical `<=`-accumulation susceptibility | CONFIRMED | distance_field.cpp:270-278 | |
| distance_field.rs:92-103 | Fixture-pinned octree-points cases match the real moveit2 C++ oracle bit-for-bit | CONFIRMED (corrected from sub-agent's UNVERIFIABLE — the oracle is reachable in this sandbox via `sg docker`) | `tools/ci/verify-fixture-replay.sh` reports `identical moveit-distance-field/octree_points` against the live oracle, this round | |
| distance_field.rs:129-140 | `tree_iterator`/`leaf_iterator`/`leaf_bbx_iterator` are three distinct upstream octomap classes | CONFIRMED | octomap/OcTreeIterator.hxx:207,263,335 | |
| distance_field.rs:178-180 | `DistanceGradient` is the return-value shape of `getDistanceGradient`'s out-parameters | CONFIRMED | distance_field.hpp:313-314 | |
| distance_field.rs:190-192 | `in_bounds` false within one cell of the boundary (1-cell gradient padding) | CONFIRMED | distance_field.cpp:80-89 | |
| distance_field.rs:198-201 | `DistanceField` is an abstract base class | CONFIRMED | distance_field.hpp:92-93 | |
| distance_field.rs:205-208 | Base class also declares 4 shape methods + 4 RViz marker methods | CONFIRMED | distance_field.hpp:190,205,225,237,430,446,474,493 | |
| distance_field.rs:210-219 | Those 4 functions are non-virtual; shapes go through `createEmptyBodyFromShapeType` | CONFIRMED | distance_field.hpp; geometric_shapes/body_operations.cpp:37-60 | |
| distance_field.rs:228-239 | `getShapePoints` special-cases OcTree, ignores pose; null octree payload is a null-pointer deref upstream | CONFIRMED | distance_field.cpp:211-220 | |
| distance_field.rs:240-244 | `moveShapeInField` special-cases OcTree as a no-op | CONFIRMED | distance_field.cpp:296-300 | |
| distance_field.rs:245-252 | `removeShapeFromField` has no OcTree special case | CONFIRMED | distance_field.cpp:314-324 | |
| distance_field.rs:254-261 | `add_octree_to_field`'s algorithm differs from the octree ctor's unfiltered full-tree walk | CONFIRMED | collision_distance_field_types.cpp:355-365 | |
| distance_field.rs:269-272 | `writeToStream`/`readFromStream` zlib-compress via `boost::iostreams` | CONFIRMED | propagation_distance_field.cpp:41,646-648,726-727 | |
| distance_field.rs:273-277 | Octree-and-bbox ctor overload is separate from `addOcTreeToField` | CONFIRMED | propagation_distance_field.hpp:198 | |
| distance_field.rs:278-279 | `setPoint`'s only caller is `getProjectionPlanes` | CONFIRMED | distance_field.cpp:437,523,534,545 | |
| distance_field.rs:280-286 | Base ctor takes seven size/origin/resolution args | CONFIRMED | distance_field.hpp:107-108 | |
| distance_field.rs:288-303 | 8 getter mappings (size/origin ×3, resolution, uninitialized_distance) | CONFIRMED | distance_field.hpp:502-579 | |
| distance_field.rs:305-312 | 4 pure-virtual mappings (add/remove/update points, reset) | CONFIRMED | distance_field.hpp:119,134,159,243 | |
| distance_field.rs:314-318 | `distance(x,y,z)` returns `uninitialized_distance` outside the grid, not a panic | CONFIRMED | voxel_grid.hpp:453-461; propagation_distance_field.cpp:594-596 | |
| distance_field.rs:319-323 | `distance_cell` maps to `getDistance(int,int,int)`, "must be valid or corruption" | CONFIRMED | propagation_distance_field.hpp:387,403 | |
| distance_field.rs:332-338 | Upstream's `gridToWorld` bool return is hard-coded `true` by its only implementer | CONFIRMED | propagation_distance_field.cpp:624-628 | |
| distance_field.rs:339-342 | `world_to_grid`'s bool reports validity; indices still computed when invalid | CONFIRMED | propagation_distance_field.cpp:630-633; voxel_grid.hpp:554-560 | |
| distance_field.rs:344-349 | `getDistanceGradient` has a single shared non-virtual implementation | CONFIRMED | distance_field.cpp:73-97 | |
| distance_field.rs:368-389 | `inv_twice_resolution_` stored as `int`, silently truncating | CONFIRMED | distance_field.hpp:614; distance_field.cpp:67 | |
| distance_field.rs:404-426 | Rust `as i32` saturates out-of-range `f64`; C++ narrowing UB | CONFIRMED | distance_field.cpp:67; language semantics | |
| distance_field.rs:487-493 | `move_shape_in_field` removes-old/adds-new via one `update_points_in_field` call | CONFIRMED | distance_field.cpp:293-312 | |
| distance_field.rs:520-527 | `add_octree_to_field`: leaf ≤ resolution contributes center; larger leaf subdivided at resolution spacing | CONFIRMED | distance_field.cpp:261-283 | |
| distance_field.rs:736-742 | `add_shape_to_field` must special-case OcTree per `getShapePoints` | CONFIRMED | distance_field.cpp:211-220 | |
| distance_field.rs:764-769 | Upstream's null-octree-payload equivalent dereferences unconditionally | CONFIRMED | distance_field.cpp:211-220 | |
| distance_field.rs:782-786 | `move_shape_in_field` must special-case OcTree as a no-op matching `RCLCPP_WARN(...); return;` | CONFIRMED | distance_field.cpp:296-300 | |
| distance_field.rs:810-821 | Both upstream and port accumulate the subdivision loop via repeated float `+=` | CONFIRMED | distance_field.cpp:268-278 | |
| distance_field.rs:1181-1190 | At resolution=0.03, upstream's truncated `int` value is 16 vs untruncated 16.666... | CONFIRMED | distance_field.cpp:67 | |
| distance_field.rs:1248-1256 | At resolution=0.51, upstream's own `int inv_twice_resolution_` is also 0 | CONFIRMED | distance_field.cpp:67 | |
| collision_distance_field_types.rs:25-28 | `getCollisionSphereMarkers`/`getProximityGradientMarkers`/`getCollisionMarkers` build RViz `MarkerArray`, hence not ported | CONFIRMED | collision_distance_field_types.hpp:522-535 | |
| collision_distance_field_types.rs:29-33 | `BodyDecompositionVector` is only a forward declaration, no definition anywhere | CONFIRMED | collision_distance_field_types.hpp:225-230 (grep, only hit) | |
| collision_distance_field_types.rs:37-44 | `PosedDistanceField : public PropagationDistanceField` only re-derives `getDistanceGradient` through the pose | CONFIRMED | collision_distance_field_types.hpp:121-200 | |
| collision_distance_field_types.rs:57-67 | `test_collision_distance_field.cpp` has six `TEST_F`, `SetUp()` builds RobotModel/ACM/CollisionEnvDistanceField but not RobotState | CONFIRMED | test_collision_distance_field.cpp:58-66,85 | |
| collision_distance_field_types.rs:83-108 | `Isometry * Vector3d`'s `<...,2,1>` specialization computes the same result as the generic `<...,2,RhsCols>` formula | CONFIRMED | Eigen/src/Geometry/Transform.h:1305-1349 | |
| collision_distance_field_types.rs:113-125 | `CollisionType`: `NONE=0,SELF=1,INTRA=2,ENVIRONMENT=3`; `SelfCollision` renamed for Rust's reserved `Self` | CONFIRMED | collision_distance_field_types.hpp:63-69 | |
| collision_distance_field_types.rs:127-134 | Both `getCollisionSphereGradients` overloads take 5 trailing params; two bools separated by one double | CONFIRMED | collision_distance_field_types.hpp:193-196,209-213 | |
| collision_distance_field_types.rs:152-170 | `CollisionSphere{relative_vec_,radius_}` ctor | CONFIRMED | collision_distance_field_types.hpp:71-82 | |
| collision_distance_field_types.rs:172-183 | `GradientInfo` has no length-invariant across vectors; callers must presize | CONFIRMED | collision_distance_field_types.cpp:93,156 | |
| collision_distance_field_types.rs:204-206 | `GradientInfo()` sets `closest_distance=DBL_MAX`, `collision=false` | CONFIRMED | collision_distance_field_types.hpp:86-88 | |
| collision_distance_field_types.rs:220-226 | `GradientInfo::clear()` clears every vector field except `types` | CONFIRMED | collision_distance_field_types.hpp:101-110 | |
| collision_distance_field_types.rs:238-241 | `ProximityInfo` is not read/written anywhere else in the module/test file | CONFIRMED | grep, only its own declaration | |
| collision_distance_field_types.rs:260-285 | Same composition-vs-inheritance claim as :37-44 | CONFIRMED | collision_distance_field_types.hpp:121-200 | |
| collision_distance_field_types.rs:291-297 | `PosedDistanceField(size,origin,resolution,max_distance,propagate_negative_distances=false)` | CONFIRMED | collision_distance_field_types.hpp:126-132 | |
| collision_distance_field_types.rs:314-322 | `PosedDistanceField::updatePose`/`getPose` | CONFIRMED | collision_distance_field_types.hpp:134-142 | |
| collision_distance_field_types.rs:336-367 | Query point rotated by `pose_.linear().transpose()` only; returned gradient via full `pose_ * Vector3d` | CONFIRMED | collision_distance_field_types.hpp:167,171 | |
| collision_distance_field_types.rs:372-382 | Method vs free-function divergence: `grad.norm()>0` vs `>EPSILON`; `dist.abs()` only in the method | CONFIRMED | collision_distance_field_types.cpp:47,102,119 vs 165,173-181 | |
| collision_distance_field_types.rs:383-401 | `DistanceField::getDistanceGradient` is non-virtual, zeroes gradient before derived-class logic on out-of-bounds | CONFIRMED | distance_field.cpp:73-97; distance_field.hpp:313-314 | |
| collision_distance_field_types.rs:466-471 | Upstream's own comment calls returning `vector<CollisionSphere>` by value "BAD" | CONFIRMED | collision_distance_field_types.hpp:202-204 | |
| collision_distance_field_types.rs:473-487 | `num_points=ceil(length/(radius/2.0))` as `unsigned int`; `length==0.0` underflows `num_points-1` | CONFIRMED | collision_distance_field_types.cpp:72,76 | |
| collision_distance_field_types.rs:489-513 | `radius==0.0` with `length>0.0` is real/constructible via `Box::computeBoundingCylinder`; ratio is +inf | CONFIRMED | geometric_shapes/bodies.cpp:642-671; collision_distance_field_types.cpp:72 | |
| collision_distance_field_types.rs:515-522 | `Body::Sphere` branch of `determineCollisionSpheres` never writes `relative_transform` | CONFIRMED | collision_distance_field_types.cpp:63-82 | |
| collision_distance_field_types.rs:557-564 | Free `getCollisionSphereGradients` takes unposed `DistanceField*` not `PosedDistanceField` | CONFIRMED | collision_distance_field_types.hpp:209-213; .cpp:150-209 | |
| collision_distance_field_types.rs:618-624 | Free `getCollisionSphereCollision` bool-only overload carries `!in_bounds && grad.norm()>0` guard | CONFIRMED | collision_distance_field_types.hpp:215-218; .cpp:211-236 | |
| collision_distance_field_types.rs:645-655 | `num_coll==0` overload: report-first-collision-collect-nothing semantics | CONFIRMED | collision_distance_field_types.cpp:238-271 | |
| collision_distance_field_types.rs:689-693 | `bodies::BodyVector` is a thin Body-vector-plus-first-hit-query wrapper | CONFIRMED | geometric_shapes/bodies.h:557-600 | |
| collision_distance_field_types.rs:695-719 | `relative_cylinder_pose_` has no initializer, left garbage for Sphere-only decomposition | CONFIRMED (structural); numeric garbage-value claim not re-run this round | collision_distance_field_types.hpp:242; .cpp:63-82,277-290 | |
| collision_distance_field_types.rs:730-741 | Single-shape ctor default padding is 0.01 | CONFIRMED | collision_distance_field_types.hpp:235 | |
| collision_distance_field_types.rs:751-761 | `BodyDecomposition(shapes,poses,resolution,padding)` shares `init()`; upstream indexes with no bounds check | CONFIRMED | collision_distance_field_types.cpp:277-290,296-299 | |
| collision_distance_field_types.rs:818 | `BodyDecomposition::replaceCollisionSpheres` | CONFIRMED | collision_distance_field_types.hpp:244-251 | |
| collision_distance_field_types.rs:828-841 | `getCollisionSpheres`/`getSphereRadii`/`getCollisionPoints` accessors | CONFIRMED | collision_distance_field_types.hpp:253-266 | |
| collision_distance_field_types.rs:843-845 | `getBody` has no bounds check (unchecked `bodies_[i]`) | EXPIRED — `getBody` delegates to `BodyVector::getBody`, which bounds-checks and returns `nullptr` | geometric_shapes bodies.cpp:1382-1391 | ac28a2c |
| collision_distance_field_types.rs:850-863 | `getBodiesCount`/`getRelativeCylinderPose`/`getRelativeBoundingSphere` | CONFIRMED | collision_distance_field_types.hpp:273-286 | |
| collision_distance_field_types.rs:868-874 | `body_decomposition_` held as `BodyDecompositionConstPtr` (`shared_ptr<const T>`), enabling sharing | CONFIRMED | macros/declare_ptr.hpp:44-49; collision_distance_field_types.hpp:301-306,342 | |
| collision_distance_field_types.rs:883-895 | `PosedBodySphereDecomposition(body_decomposition)`: sets bounding-sphere center, resizes, calls `updatePose(Identity())` | CONFIRMED | collision_distance_field_types.cpp:380-386 | |
| collision_distance_field_types.rs:897-925 | 6 accessor mappings | CONFIRMED | collision_distance_field_types.hpp:308-335 | |
| collision_distance_field_types.rs:927-936 | `updatePose`'s `trans` "assumed to be in reference frame"; `sphere_centers_` indexed without resizing | CONFIRMED | collision_distance_field_types.hpp:337-338; .cpp:384,388-406 | |
| collision_distance_field_types.rs:961-970 | `body_decomposition_` null only for the octree ctor | CONFIRMED | collision_distance_field_types.cpp:342-357 | |
| collision_distance_field_types.rs:973-985 | `PosedBodyPointDecomposition(body_decomposition)` posed at identity via verbatim copy, not `updatePose(Identity())` | CONFIRMED | collision_distance_field_types.cpp:342-353 | |
| collision_distance_field_types.rs:995-1017 | Octree ctor walks full-tree iterator with no occupancy filter; `reserve()` undersized from `getNumLeafNodes()` | CONFIRMED | collision_distance_field_types.cpp:355-365 | |
| collision_distance_field_types.rs:1034-1037 | `updatePose` is a no-op guarded by `if (body_decomposition_)` | CONFIRMED | collision_distance_field_types.cpp:367-378 | |
| collision_distance_field_types.rs:1049-1059 | `decomp_vector_` stored as `vector<PosedBodySphereDecompositionPtr>` | CONFIRMED | collision_distance_field_types.hpp:439 | |
| collision_distance_field_types.rs:1112-1116 | `getPosedBodySphereDecomposition` logs then returns null for out-of-range `i` | CONFIRMED | collision_distance_field_types.hpp:412-420 | |
| collision_distance_field_types.rs:1121-1124 | `updatePose` warns and returns for out-of-range `ind` | CONFIRMED | collision_distance_field_types.hpp:422-434 | |
| collision_distance_field_types.rs:1138-1141 | `PosedBodyPointDecompositionVector`, shared-pointer elements | CONFIRMED | collision_distance_field_types.hpp:446-504 | |
| collision_distance_field_types.rs:1153-1155 | `getCollisionPoints` concatenates fresh on every call, not cached | CONFIRMED | collision_distance_field_types.hpp:455-463 | |
| collision_distance_field_types.rs:1197-1213 | `doBoundingSpheresIntersect` compares squared center distance against unsquared radius sum | CONFIRMED | collision_distance_field_types.cpp:408-418 | |
| collision_distance_field_types.rs:1205-1209 | No test case reaches `doBoundingSpheresIntersect`, nothing has ever exercised it | EXPIRED — `LinksInCollision`'s `setEntry(...,false)` leaves that pair's intra-group check enabled; `checkSelfCollision`→`checkSelfCollisionHelper`→`getIntraGroupCollisions` reaches this function | collision_env_distance_field.cpp:192-198,449-453; collision_matrix.cpp:154-157; test_collision_distance_field.cpp:135-137 | 866085a |
| collision_distance_field_types.rs:1426-1431 | Every public constructor sets `Some`; octree ctor that would leave `None` upstream not ported | EXPIRED — `from_octree` is ported in this same file and sets `None`, exercised by `from_octree_update_pose_is_a_no_op` | collision_distance_field_types.rs:1025-1032,1591 (in-port evidence — this was a stale self-contradiction, not an upstream-fact error) | 62757db |
| collision_distance_field_types.rs:1448-1451 | Second occurrence of the same "no bounds check" claim | EXPIRED — same as :843-845 | geometric_shapes bodies.cpp:1382-1391 | ac28a2c |
| collision_env_distance_field.rs:9-23 | Module doc lists ported functions as `CollisionEnvDistanceField` counterparts | CONFIRMED | collision_env_distance_field.cpp:158-175,710-958,1382-1559 | |
| collision_env_distance_field.rs:73-93 | Quotes `generateCollisionCheckingStructures`'s 3-step body | CONFIRMED | collision_env_distance_field.cpp:158-175 | |
| collision_env_distance_field.rs:106-111 | `update_cache_lock_` is a `const`-method `const_cast` workaround | CONFIRMED | collision_env_distance_field.cpp:170-171; hpp:298 | |
| collision_env_distance_field.rs:117-134 | Seven callers guard the same way; `in_group_update_map_` built once per group at construction | CONFIRMED | collision_env_distance_field.cpp:183-186,1393-1396,1426-1428,1459-1461,1488-1490,1524-1526,1545-1547,141-155 | |
| collision_env_distance_field.rs:137-144 | `getPosedLinkBodySphereDecomposition`/`getPosedLinkBodyPointDecomposition` trivial wrappers | CONFIRMED | collision_env_distance_field.cpp:1080-1101 | |
| collision_env_distance_field.rs:145-170 | `getAttachedBodySphereDecomposition`/`getAttachedBodyPointDecomposition` sole callers | CONFIRMED | collision_env_distance_field.cpp:1239,928,775,797-798,825 | |
| collision_env_distance_field.rs:172-186 | `checkSelfCollisionHelper` and 6 sibling bodies guard `gsr` the same way | CONFIRMED | collision_env_distance_field.cpp:183-186+cited sites | |
| collision_env_distance_field.rs:188-196 | Difference table: generate_distance_field asymmetry, unused `res`, missing `if(!done)` guard | CONFIRMED | collision_env_distance_field.cpp:1461,1490,1517,1553-1556 | |
| collision_env_distance_field.rs:198-202 | Two continuous overloads both stub "not implemented" | CONFIRMED | collision_env_distance_field.cpp:1502-1515 | |
| collision_env_distance_field.rs:204-220 | No self-collision overload takes `env_distance_field`; distance* stubs never reach `gsr` | CONFIRMED | collision_env_distance_field.hpp:109-199; .cpp:1447-1500 | |
| collision_env_distance_field.rs:238-248 | Fourth `checkSelfCollision` overload logs a warning when `gsr` already populated | CONFIRMED | collision_env_distance_field.cpp:260-272 | |
| collision_env_distance_field.rs:250-267 | `createCollisionModelMarker`/`distanceSelf`/`distanceRobot`/World-observer wiring unported | CONFIRMED | collision_env_distance_field.cpp:74-96,981-1047; .hpp:272-284; moveit-collision/world.rs:40-42 | |
| collision_env_distance_field.rs:273-285 | Upstream's own test suite never exercises `addLinkBodyDecompositions`/cache-entry construction in isolation | CONFIRMED | test_collision_distance_field.cpp (grep of all TEST_F) | |
| collision_env_distance_field.rs:287-299 | Second `addLinkBodyDecompositions` overload also calls `replaceCollisionSpheres` | CONFIRMED | collision_env_distance_field.cpp:1049-1078 | |
| collision_env_distance_field.rs:301-312 | `getLinkModelsWithCollisionGeometry()` filter matches construction-time filter | CONFIRMED | robot_model.cpp:873-879 | |
| collision_env_distance_field.rs:361-364 | `LinkBodyDecompositions` matches `link_body_decomposition_vector_`+index map | CONFIRMED | collision_env_distance_field.hpp:295-296 | |
| collision_env_distance_field.rs:366-379 | Built from all shapes together, default padding 0.0, robot_model order | CONFIRMED | collision_env_distance_field.cpp:960-978; collision_env.cpp:166-177; robot_model.cpp:870-879 | |
| collision_env_distance_field.rs:452-458 | Center-to-corner shift done inline at ctor call site | CONFIRMED | collision_env_distance_field.cpp:931-933 | |
| collision_env_distance_field.rs:466-484 | Reads robot_model_/resolution_/index map/in_group_update_map_ off self | CONFIRMED | collision_env_distance_field.cpp:141-155,710-712,735,773 | |
| collision_env_distance_field.rs:488-497 | `link_state_indices` computed directly since both vectors derive from the same sorted source | CONFIRMED | joint_model_group.cpp:261-278; collision_env_distance_field.cpp:744,754-762 | |
| collision_env_distance_field.rs:498-504 | "Already has a field" skip not ported since `dfce` is freshly built | CONFIRMED | collision_env_distance_field.cpp:714,898-905 | |
| collision_env_distance_field.rs:505-508 | "No link state found" skip not ported, same reasoning | CONFIRMED | collision_env_distance_field.cpp:744-768 | |
| collision_env_distance_field.rs:518-523 | Attached-body population only runs when `acm` is Some | CONFIRMED | collision_env_distance_field.cpp:775,797-823,825-829 | |
| collision_env_distance_field.rs:691-695 | `STATE_CHECK_EPSILON=0.001` matches file-local `EPSILON` | CONFIRMED | collision_env_distance_field.cpp:55 | |
| collision_env_distance_field.rs:697-712 | `compare_cache_entry_to_state` false conditions enumeration | CONFIRMED | collision_env_distance_field.cpp:1254-1312 | |
| collision_env_distance_field.rs:738-755 | `compare_cache_entry_to_allowed_collision_matrix`'s local `attached_bodies` is dead code | CONFIRMED | collision_env_distance_field.cpp:1314-1370 | |
| collision_env_distance_field.rs:784-797 | `get_distance_field_cache_entry` is const, returns unchanged on every accepting path | CONFIRMED | collision_env_distance_field.cpp:202-233 | |
| collision_env_distance_field.rs:820-892 | Pregenerated branch's `sphere_locations` source and both branches sharing decomposition/pose | CONFIRMED | collision_env_distance_field.cpp:1157-1227,1224 | |
| collision_env_distance_field.rs:844-874 | Pregenerated branch populated only in `initialize()`, taken by every post-construction call | CONFIRMED | collision_env_distance_field.cpp:126-156,867-872,1161,1212 | |
| collision_env_distance_field.rs:893-906 | Attached-body loop runs unconditionally after fresh-build/pregenerated branch | CONFIRMED | collision_env_distance_field.cpp:1229-1251,1239 | |
| collision_env_distance_field.rs:907-926 | Fresh-build branch's `gradients.resize(n)` has no fill, unlike update path's real zero-fill | CONFIRMED | collision_env_distance_field.cpp:1201 vs 1117-1118 | |
| collision_env_distance_field.rs:928-943 | Fresh-build path's `getAttachedBody` has no null check, unlike update path | CONFIRMED | collision_env_distance_field.cpp:1239 vs 1125-1130 | |
| collision_env_distance_field.rs:1038-1059 | `update_group_state_representation_state` reproduces upstream's own "might be incorrect" TODO | CONFIRMED | collision_env_distance_field.cpp:1103-1155,1132-1137 | |
| collision_env_distance_field.rs:1055-1059 | This reset is a real zero-fill, unlike the fresh-build path | CONFIRMED | collision_env_distance_field.cpp:1117-1118,1150-1151 | |
| collision_env_distance_field.rs:1061-1073 | Port hard-errors where upstream logs+continues on null attached body | CONFIRMED | collision_env_distance_field.cpp:1125-1130 | |
| collision_env_distance_field.rs:1148-1169 | `DistanceFieldCollisionCache` struct: default tolerance, persistent cache slot | CONFIRMED | collision_env_distance_field.hpp:54,299 | |
| collision_env_distance_field.rs:1171-1207 | `new()` loops every joint group building cache entry + pregenerated GSR | CONFIRMED | collision_env_distance_field.cpp:126-156 | |
| collision_env_distance_field.rs:1221-1243 | Same 3-step sequence + lock workaround (duplicate of :73-93/:106-111) | CONFIRMED | collision_env_distance_field.cpp:158-175,170-171 | |
| collision_env_distance_field.rs:1290-1346 | `checkSelfCollisionHelper` collapses 4 overloads via `acm` | CONFIRMED | collision_env_distance_field.cpp:177-200,235-272 | |
| collision_env_distance_field.rs:1382-1399 | 4 overloads; environment collisions from `distance_field_cache_entry_world_` | CONFIRMED | collision_env_distance_field.cpp:1382-1443,1408,1441 | |
| collision_env_distance_field.rs:1446-1468 | 2 real overloads, generate_distance_field asymmetry preserved | CONFIRMED | collision_env_distance_field.cpp:1447-1500 | |
| collision_env_distance_field.rs:1507-1516 | Unused `res` param; only external caller is a pure forwarding wrapper | CONFIRMED | collision_env_distance_field.cpp:1517-1538; collision_env_hybrid.cpp:172-177 | |
| collision_env_distance_field.rs:1551-1563 | No `if(!done)` guard between phases, discards return values | CONFIRMED | collision_env_distance_field.cpp:1540-1559 | |
| collision_env_distance_field.rs:1611-1642 | `getSelfCollisions` loop bound/branches/reporting; no null-check on `distance_field_` | CONFIRMED | collision_env_distance_field.cpp:274-353,302 | |
| collision_env_distance_field.rs:1739-1762 | `getSelfProximityGradients`'s own narrower loop makes its `is_link`-false branch dead code | CONFIRMED | collision_env_distance_field.cpp:355-428,359,375-380 | |
| collision_env_distance_field.rs:1843-1879 | `getIntraGroupCollisions` loop bound; unreachable `i==j` guard; contact-reporting reads unconditionally | CONFIRMED | collision_env_distance_field.cpp:430-642,437,439-442,601,604-611,533-545 | |
| collision_env_distance_field.rs:2027-2045 | No pre-filter; unreachable `i==j` guard; `in_collision` local never written; caller discards return | CONFIRMED | collision_env_distance_field.cpp:644-709,1534 | |
| collision_env_distance_field.rs:2100-2123 | `getEnvironmentCollisions` loop bound/branches/reporting; unused `link_name` local | CONFIRMED | collision_env_distance_field.cpp:1561-1643,1568 | |
| collision_env_distance_field.rs:2216-2231 | `getEnvironmentProximityGradients`'s own narrower loop makes its branch dead code | CONFIRMED | collision_env_distance_field.cpp:1645-1681,1649 | |
| collision_env_distance_field.rs:2267-2287 | Non-group-link-with-no-geometry attached bodies are invisible "exactly like `generate_distance_field_cache_entry`'s `attached_body_names` (:910-925): outer loop continues before inner attached-body loop runs" | EXPIRED — `:910-925` is this same function's own loop (already correctly cited as `:945-950`), not `attached_body_names_`'s real population site (`:770-825`, a different loop over group-member links); the exclusion here is the pre-filtered `getLinkModelsWithCollisionGeometry()` iteration source, not an in-loop continue (the continue at `:916` is gated on group membership) | collision_env_distance_field.cpp:910,916,927,770-825 | 26f4f6c |
| collision_env_distance_field.rs:3045-3051 | Attached bodies only enumerated inside the `if(acm)` branch; `acm: None` leaves `attached_body_names` empty | CONFIRMED | collision_env_distance_field.cpp:775,797-823,825 | |
| collision_env_distance_field.rs:3112-3118 | Non-group-distance-field loop iterates `getLinkModelsWithCollisionGeometry()`, skips in-group links | CONFIRMED | collision_env_distance_field.cpp:910,914-917 | |
| collision_env_distance_field.rs:3370-3376 | `group_state_representation`'s attached-body loop sets `sphere_locations` at fresh-build time, cited at `:1249` | EXPIRED — the `sphere_locations` assignment is at `:1246-1247`; `:1249` is the following `sphere_radii` assignment. Underlying asymmetry claim correct, only the line number was wrong | collision_env_distance_field.cpp:1246-1249 | 69b0000 |
| collision_env_distance_field.rs:3477-3484 | Upstream's attached-body lookup has no null check | CONFIRMED | collision_env_distance_field.cpp:1238-1239 vs 1125-1129 | |
| collision_env_distance_field.rs:3569-3576 | Upstream's own suspicious attached-body-count check compares decomposition-vector size vs shape count | CONFIRMED | collision_env_distance_field.cpp:1132-1137 | |
| collision_env_distance_field.rs:3630-3636 | Count mismatch trips upstream's suspicious `continue`, skipping the re-pose | CONFIRMED | collision_env_distance_field.cpp:1133-1136 | |
| collision_env_distance_field.rs:4036-4042 | `done` means "contact budget spent", not "a collision found"; generous `max_contacts` keeps accumulating | CONFIRMED | collision_env_distance_field.cpp:352 | |
| collision_env_distance_field.rs:4093-4098 | `get_all_collisions` has no `if !done` guard between phases, unlike `check_collision` | CONFIRMED | collision_env_distance_field.cpp:1540-1559 vs 1401-1409 | |
| collision_env_distance_field.rs:4317-4322 | Per-link loop only visits the group's own updated links; a non-updated-link attached body is never enumerated | CONFIRMED | collision_env_distance_field.cpp:735,797-823 | |
| collision_env_distance_field.rs:4366-4371 | Same `if(acm)`-only enumeration claim as :3045-3051 | CONFIRMED | collision_env_distance_field.cpp:775,825 | |
| collision_env_distance_field.rs:4230-4237 | An attached body coincident with the group's own field is reported by `get_self_collisions` against "self" | CONFIRMED | collision_env_distance_field.cpp:278,327 | |
| collision_env_distance_field.rs:4618-4625 | An environment point at the coincident attached-body origin registers as an environment collision | CONFIRMED | collision_env_distance_field.cpp:1565 | |
| collision_env_distance_field.rs:4708-4717 | `get_intra_group_proximity_gradients` folds attached-body distances into its own slot; self/environment siblings never touch an attached-body index | CONFIRMED | collision_env_distance_field.cpp:650 vs 359,1649 | |
| collision_env_distance_field.rs:4518-4526 | `touch_links.contains(link_name)` check only reached while iterating the attaching link itself | CONFIRMED | collision_env_distance_field.cpp:797-798,816 | |
| fixtures/pr2.srdf | pr2's `right_arm` self-collides at every joint configuration under this SRDF (measured directly: 230/230 sampled states self-collided, round 29's F3 investigation) -- not a gap this port introduced: `fixtures/pr2.srdf` is byte-identical to upstream's own `moveit_resources_pr2_description/srdf/robot.xml` test fixture (its one `disable_collisions` entry and `<!-- and many more disable_collisions tags -->` comment are upstream's own choices), so any test built on "pr2's right_arm has a collision-free default state" cannot exist against this fixture without authoring new `disable_collisions` entries and diverging from the file it is checked against | CONFIRMED | `diff fixtures/pr2.srdf third_party/moveit_resources/pr2_description/srdf/robot.xml` (zero output); `tools/ci/verify-fixture-provenance.sh` names this exact mapping | |
