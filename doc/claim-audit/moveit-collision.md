# Claim audit — moveit-collision

Format and rule: PORTING-PLAN.md §175. One row per claim a doc comment in
this tree makes about upstream. `evidence` is a file:line I opened myself
this round, not an inference from the port's own behavior. Appended as
found, not batched at report time.

## §171 dispatch-mechanism citations (round with commits `c6161b9`, `2607bef`)

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `src/parry.rs` (`cost_sources_for_part_pair` doc) | `fcl::CollisionRequest`'s 5th positional ctor arg, `use_approximate_cost_`, defaults to `true` | CONFIRMED | `fcl/include/fcl/narrowphase/collision_request.h:101` | `c6161b9` |
| `src/parry.rs` (same doc) | `moveit_core` never overrides `use_approximate_cost` — all 3 `fcl::CollisionRequestd(...)` call sites in `collision_common.cpp` pass exactly 4 positional args, never a 5th | CONFIRMED | `moveit_core/collision_detection_fcl/src/collision_common.cpp:227,303,364` | `c6161b9` |
| `src/parry.rs` (same doc) | `collision_func_matrix-inl.h` reads `use_approximate_cost` at exactly 4 sites (OcTree↔BVH, BVH↔OcTree, `BVHShapeCollider::collide`, `orientedBVHShapeCollide`), each running a cost-disabled traversal for contacts then `constructBox(obj->getBV(0).bv, ...)` for a second, cost-only pass | CONFIRMED | `fcl/include/fcl/narrowphase/detail/collision_func_matrix-inl.h:184,199,237,252,330,348,391,405` | `c6161b9` |
| `src/parry.rs` (same doc) | `MeshShapeCollisionTraversalNode::leafTesting`'s per-triangle `addCostSource` is unreached under moveit (only runs when `use_approximate_cost == false`, never requested) | CONFIRMED | `fcl/include/fcl/narrowphase/detail/traversal/collision/mesh_shape_collision_traversal_node-inl.h:112,123` | `c6161b9` |
| `src/parry.rs` (octree paragraph) | `octree_solver-inl.h` never reads `use_approximate_cost` anywhere in the file; `OcTreeShapeIntersectRecurse` calls `addCostSource` once per occupied leaf unconditionally | CONFIRMED | `fcl/include/fcl/narrowphase/detail/traversal/octree/octree_solver-inl.h` (full-file grep: 0 hits for `use_approximate_cost`; `addCostSource` at `:353,605,672,704,872,1002,1025`) | `c6161b9` |
| `src/parry.rs` (`mesh_shape_cost_sources` doc) | `TriMesh::aabb(pos)` is `self.bvh.root_aabb().transform_by(pos)`, and `Shape::compute_aabb` for `TriMesh` calls that identical method — so the mesh's BVH-root box and its `compute_aabb` are the same value by construction | CONFIRMED | `parry3d-f64-0.30.0/src/shape/trimesh.rs:1748-1749`, `src/shape/shape.rs:1101-1103` (`~/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/`) | `c6161b9` |
| `src/common.rs` (`CostSource` Ord/Eq doc) | Upstream's `operator<` tie-break chain ends at `aabb_min` and never reaches `aabb_max` — two sources tied on `cost*volume`, `cost`, `aabb_min` but differing only in `aabb_max` compare `Equal` upstream | CONFIRMED | `moveit_core/collision_detection/include/moveit/collision_detection/collision_common.hpp:128-141` | `2607bef` |
| `src/parry.rs` (`triangle_world_aabb` doc, pre-existing text, re-verified this round) | Matches `mesh_collision_traversal_node-inl.h`'s `AABB<S>(p1, p2, p3)` built from vertices already baked into world coordinates | CONFIRMED | `fcl/include/fcl/narrowphase/detail/traversal/collision/mesh_collision_traversal_node-inl.h:187,196,607,616,700,709` (pattern recurs at each narrowphase entry point; no single line owns it) | (pre-existing, not this round) |

## §172 upstream-first narrowing sweep — negative result, logged for §153.1 expiry tracking

Anchor: `int`/`size_t`/`unsigned`/`static_cast<(int\|unsigned\|size_t\|...)>`
whose initializing expression is floating-point, in the upstream C++ files
this crate's Rust sources cite. Swept via `rg` (`static_cast<...>`,
C-style `(int)`/`(size_t)`/`(unsigned)` casts, `floor/ceil/round/sqrt/pow`
near an integer declaration, integer decls with a float-literal or
division RHS) against each file below, output enumerated on screen, then
`aabb.cpp` additionally read in full. Every file: 0 hits.

| where (upstream file swept) | claim | verdict | evidence |
|---|---|---|---|
| `collision_detection_fcl/include/.../collision_common.hpp` | no int/size_t/unsigned decl or `static_cast` with a floating-point initializer | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `collision_detection_fcl/src/collision_common.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `collision_detection_fcl/src/collision_env_fcl.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `collision_detection/include/.../collision_common.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `collision_detection/include/.../collision_env.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `collision_detection/include/.../collision_matrix.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `collision_detection/include/.../collision_octomap_filter.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `collision_detection/include/.../world.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `collision_detection/src/collision_common.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `collision_detection/src/collision_env.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `collision_detection/src/collision_matrix.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `collision_detection/src/collision_octomap_filter.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `collision_detection/src/collision_tools.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `collision_detection/src/world.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `planning_scene/src/planning_scene.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |

Port-side secondary sweep (`as (i8\|i16\|i32\|i64\|isize\|u8\|u16\|u32\|u64\|usize)`
receiving an `f64` expression, in this crate's own `src/`): 3 raw hits, all
`distinct` — `parry.rs:957` (`Arc::as_ptr(tree) as *const () as usize`,
pointer-to-integer, not float-derived), `parry.rs:1470`
(`mesh_a.num_triangles() as u32`, `num_triangles()` returns `usize`
confirmed at `trimesh.rs:1833`, integer-to-integer), `parry.rs:1957` (doc
comment referencing the line-957 pattern, not executable code). 0 real
hits.

Expires (§153.1): if a future round adds an `int`/`size_t`/`unsigned`
declaration with a floating-point initializer to any file above, or a new
upstream file citing floating-point-narrowing joins this crate's cited
set, this table's `CONFIRMED (absent)` rows must be re-swept, not assumed
to still hold.
