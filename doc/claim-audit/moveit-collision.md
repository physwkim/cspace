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
| `src/parry.rs` (`mesh_shape_cost_sources` doc) | `TriMesh::aabb(pos)` is `self.bvh.root_aabb().transform_by(pos)`, and `Shape::compute_aabb` for `TriMesh` calls that identical method — so the mesh's BVH-root box and its `compute_aabb` are the same value by construction | CONFIRMED | `parry3d-f64-0.30.0/src/shape/trimesh.rs:1748-1749`, `parry3d-f64-0.30.0/src/shape/shape.rs:1101-1103` (`~/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/`) | `c6161b9` |
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
pointer-to-integer, not float-derived), `parry.rs:1488`
(`mesh_a.num_triangles() as u32`, `num_triangles()` returns `usize`
confirmed at `parry3d-f64-0.30.0/src/shape/trimesh.rs:1833`, integer-to-integer), `parry.rs:1975` (doc
comment referencing the line-957 pattern, not executable code). 0 real
hits.

Expires (§153.1): if a future round adds an `int`/`size_t`/`unsigned`
declaration with a floating-point initializer to any file above, or a new
upstream file citing floating-point-narrowing joins this crate's cited
set, this table's `CONFIRMED (absent)` rows must be re-swept, not assumed
to still hold.

## `mesh_shape_cost_sources` OBB-fit citations (round with commit pending)

Closes the AABB-vs-OBBRSS bounds gap `c6161b9`'s dispatch fix left open
(id 8 matched to 1e-13, ids 2/3/4/6 were 0.003-0.07m off): the mesh's
root bound was fit as a plain axis-aligned `Bvh` root, but
`moveit_core` always builds `fcl::BVHModel<fcl::OBBRSSd>`, whose root
bound is *oriented*. [`mesh_world_obb_aabb`] fits and axis-aligns that
oriented box instead, via `parry3d_f64::utils::obb`.

| where | claim | verdict | evidence |
|---|---|---|---|
| `src/parry.rs` (`cost_sources_for_part_pair`/`mesh_shape_cost_sources` doc) | `moveit_core` always instantiates `fcl::BVHModel<fcl::OBBRSSd>` for collision geometry, never a plain-AABB BV type | CONFIRMED | `moveit_core/collision_detection_fcl/src/collision_common.cpp:949-1006`, every `createCollisionGeometry<...>` call site |
| `src/parry.rs` (same doc) | `constructBox` for `OBBRSS` uses only the inner `obb` field, discarding the RSS radius entirely | CONFIRMED | `fcl/include/fcl/geometry/shape/utility-inl.h:1083-1088` and `:1156-1163` (two overloads, same reduction) |
| `src/parry.rs` (`mesh_shape_cost_sources` doc) | FCL's own OBB fit (`FitImpl<S, OBB<S>>::run`) derives axis via `getCovariance`+`eigen_old`, center/extent via a separate `getExtentAndCenter` call using that axis | CONFIRMED | `fcl/include/fcl/geometry/bvh/detail/BV_fitter-inl.h:301-324` |
| `src/parry.rs` (same doc) | `getCovariance` sums over each triangle's 3 vertices individually (not deduplicated mesh vertices), weighting a shared vertex once per incident triangle | CONFIRMED | `fcl/include/fcl/math/geometry-inl.h:1349-1379` |
| `src/parry.rs` (same doc) | the final cost-source AABB is `computeBV` (axis-aligned bound of the oriented box) intersected with the other shape's own AABB via `overlap_part`, matching this port's `.intersection(&other_world_aabb)` | CONFIRMED | `fcl/include/fcl/narrowphase/detail/traversal/collision/shape_collision_traversal_node-inl.h:116-133` |
| `src/parry.rs` (`mesh_world_obb_aabb` doc) | `parry3d_f64::utils::obb(pts)` fits via `crate::utils::cov` + `nalgebra::SymmetricEigen`, projecting all points onto the resulting axes for center/extent — a PCA fit structurally analogous to FCL's but not bit-identical (no shared eigenvector tie-break convention) | CONFIRMED | `~/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/parry3d-f64-0.30.0/src/utils/obb.rs` (full file read) |

Measured accuracy (this crate's own instrumentation of
`moveit-scene/tests/cost_sources_parity.rs`'s two `#[ignore]`d tests,
read-only, fully reverted after — not this crate's own fixture data,
oracle `moveit-rs/oracle:3537df47121b8c7f`): state-op ids 2/3/4/6/8 all
match the oracle's `aabb_min`/`aabb_max` to `9e-14`..`4e-13` (previously
0.003-0.07m off); path-op ids 3/4/5/6's survivor counts now match the
oracle exactly (previously 1v5/1v4/1v0/5v6). State-op id 5 (9 actual vs
2 expected) is unaffected and unrelated — a `moveit-scene`
group-filter defect, not `moveit-collision`'s.

Expires (§153.1): if `parry3d_f64` changes `utils::obb`'s fitting
algorithm (a different eigensolver, a different point-projection
convention), or a future oracle rebuild changes any of the above cited
FCL/moveit_core lines, this table's measured-accuracy numbers must be
re-measured, not assumed to still hold.

## Deviation 1 was wrong: `group_name` is not inert upstream (round with commit pending)

Corrects both this file's line above ("a `moveit-scene` group-filter
defect, not `moveit-collision`'s") and the module doc's former deviation
1 ("this backend does not filter by group either, matching upstream's
real (if surprising) behavior"). Both were written from `env.rs`'s
default `check_collision` (which indeed never touches `group_name`) and
never opened `collision_env_fcl.cpp`'s `checkSelfCollisionHelper`/
`checkRobotCollisionHelper` bodies to check whether *they* do.

| where | claim | verdict | evidence |
|---|---|---|---|
| former module doc deviation 1 | `checkSelfCollision`/`checkRobotCollision` never call `enableGroup`/read `active_components_only_` | REFUTED | `collision_env_fcl.cpp:281` and `:336`: `checkSelfCollisionHelper`/`checkRobotCollisionHelper` both call `cd.enableGroup(getRobotModel())` unconditionally, every call |
| new module doc deviation 1 | `CollisionData::enableGroup` resolves `req_->group_name` to `JointModelGroup::getUpdatedLinkModelsSet()` when the model has that group, else `nullptr` | CONFIRMED | `collision_detection_fcl/collision_common.cpp:1012-1022` |
| new module doc deviation 1 | `collisionCallback` skips a pair only when *neither* side resolves to an active link (world objects never resolve to one, so a robot-vs-world pair is kept iff the robot link is active; a self-pair is kept iff *either* link is) | CONFIRMED | `collision_detection_fcl/collision_common.cpp:79-94` |
| new module doc deviation 1 | `distanceCallback` reads the identical `active_components_only` from `DistanceRequest`, so `distance_self`/`distance_robot` need the same filter even though `distanceSelf`/`distanceRobot` themselves never call `enableGroup` (their caller is expected to have already populated it) | CONFIRMED | `collision_detection_fcl/collision_common.cpp:482-500` (callback); `collision_env_fcl.cpp:288-290`/`345-347` (the caller populating it before calling `distanceSelf`/`distanceRobot`) |
| new module doc deviation 1 | `JointModelGroup::getUpdatedLinkModelsSet()` is the union of every joint root's descendant links (fixed-joint descendants included), and `moveit-model`'s already-ported `JointModelGroup::updated_link_names` computes the identical set | CONFIRMED | `moveit_core/robot_model/src/joint_model_group.cpp:250-260`; `crates/moveit-model/src/joint_model_group.rs:315-334` (own crate, pre-existing, ported correctly — the bug was only in `moveit-collision` never calling it) |

Measured (this crate's `parry::tests::check_robot_collision_group_name_*`/
`check_self_collision_group_name_*`/`distance_robot_group_name_*`, own
fixtures, no oracle needed — the OR-vs-AND filter shape and the
unknown-group fallback are structural, not numeric): fixed. Isolated via
`git stash` on `src/parry.rs` alone against `moveit-scene`'s (untouched)
`panda_cost_sources_blocked_by_mesh_shape_cost_sources`: without this
round's fix the test fails exactly as filed, `case id 5: count mismatch
left: 9 right: 2`; with it, all cases in that test (`#[ignore]`d,
`moveit-scene`'s own bookkeeping to remove) pass at the existing `1e-9`
threshold. So was `panda_path_cost_sources_blocked_by_mesh_shape_cost_sources`,
already-fixed by a prior round's OBB fit, not this one — isolated the same
way, confirmed unaffected by this round's `parry.rs` change alone.

Expires (§153.1): if a future round adds a `moveit-collision` entry
point that receives a `CollisionRequest`/`DistanceRequest` but bypasses
`check_self_collision`/`check_robot_collision`/`distance_self`/
`distance_robot` (e.g. a batched or continuous variant), that entry
point needs its own `active_group_links`/`pair_in_active_group` wiring —
not assumed inherited from this fix.

## `mesh_world_obb_aabb`'s per-corner weighting is now pinned by a test (round with commit pending)

The OBB-fit round's own citation (`geometry-inl.h:1349-1379`, `getCovariance`
sums over each triangle's 3 vertices individually) was never contradicted by
any existing test: every mesh fixture in this file gives each vertex exactly
one incident triangle, so a regression to `mesh.vertices()` (deduplicated)
would pass every one of them unnoticed — the exact failure mode §167.5 and
the `moveit-error` `Display` audit already caught once each, a citation
correct when written and never re-checked as the surrounding code moved.

| where | claim | verdict | evidence |
|---|---|---|---|
| new test `mesh_world_obb_aabb_weights_by_triangle_corner_not_deduplicated_vertex` | a mesh where one vertex (`v0`) is incident to 5 triangles against 2 for each other vertex measurably distinguishes per-corner-weighted fitting from deduplicated-vertex fitting | CONFIRMED, by construction | fit computed both ways with the actual `parry3d_f64::utils::obb` this crate calls; AABBs differ by `~0.2`-`0.5` per component, run in `python3` first (pure-Python 2D PCA, no dependency available) before committing to the Rust construction |
| same test | this test fails if `mesh_world_obb_aabb` is changed to fit `mesh.vertices()` (deduplicated) instead of the per-corner flattening | CONFIRMED | temporarily edited the function to `mesh.vertices().to_vec()`, ran the test, watched it fail (`component 0: actual -10.87... vs expected -11.21...`) while the pre-existing rotation test kept passing under the same edit, then reverted and re-ran to confirm both pass again |

Explicit boundary (what this test does and does not catch, since an
unstated boundary is exactly what let deviation 1 sit uncorrected above):
catches a regression to deduplicated-vertex fitting, or to any other
scheme that would weight `v0` differently from a 5:2 ratio against
`v1..v5`. Does not catch a change to a *different* per-corner-weighted
algorithm (a different eigensolver, a different point-projection
convention) that still preserves that 5:2 weighting — `parry3d_f64::utils::obb`'s
own fit is already audited above as PCA-via-covariance, not claimed
bit-identical to FCL's `FitImpl<S, OBB<S>>::run`.

## Correcting my own over-claim: the `group_name` fix explains id 5 and the path-op cases, not the 115 `visibility_cone` mismatches (round 21)

My own prior report claimed the `group_name` fix above (deviation 1) "is
the direct cause of 105 of 115 pr2 `visibility_cone` depth mismatches per
`PORTING-PLAN.md` §119.1/§120.1". `PORTING-PLAN.md` §119.1 says the
opposite: it *refutes* a traversal-order explanation for those 115 and
names the cause as `moveit-collision`'s deviation 6, already documented —
two independent penetration-depth approximations for the same touching
pair disagreeing, not a pair the group filter kept or dropped. §120.1's
`105/115` is the size of the `touching == 1` sub-population within a
285-case cross-tab (`touching | n | pass | fail`: `1 | 129 | 24 | 105`,
`>=2 | | 4 | 10`), not an attribution to any fix — and structurally
cannot be one: a group filter can only keep or drop a *pair*, never
change a kept pair's own reported *depth*.

Verified structurally, not merely re-asserted: `VisibilityConstraint::
cone_collision_result` (`crates/moveit-constraints/src/visibility.rs`)
builds its `CollisionRequest` as `CollisionRequest { contacts: true,
max_contacts, ..Default::default() }` — `group_name` is never set on that
struct anywhere in the file (`rg -n group_name
crates/moveit-constraints/src/visibility.rs`, 0 hits), and
`CollisionRequest::default()` (`common.rs:267-283`) sets `group_name:
None` explicitly. This round's own `active_group_links` (`parry.rs`)
returns `None` whenever its `group_name: Option<&str>` argument is `None`
(the leading `group_name?`), and every one of the four `CollisionEnv`
methods short-circuits its filter with `active.as_ref().is_none_or(...)`
when that is `None` — no candidate pair is ever dropped. So the
`group_name` fix is provably inert on every path `decide_cone` can reach,
not merely unlikely to matter; no before/after count is needed because
there is no code path for the fix to change.

| where | claim | verdict | evidence |
|---|---|---|---|
| this section | `VisibilityConstraint::cone_collision_result` never sets `CollisionRequest::group_name` | CONFIRMED | `crates/moveit-constraints/src/visibility.rs`, `rg -n group_name`, 0 hits in that file |
| this section | `CollisionRequest::default().group_name == None` | CONFIRMED | `crates/moveit-collision/src/common.rs:267-283` |
| this section | `active_group_links(state, None)` returns `None`, and every filtered `CollisionEnv` method treats `None` as "no filtering" | CONFIRMED | `crates/moveit-collision/src/parry.rs`, this round's own `585a79e` |

Current status of the 115, superseding §119.1/§120.1's own "10 residual,
not yet closed" (their own later round, not mine): `PORTING-PLAN.md` §169
closed the `touching >= 2` question — 0 such cases across a fresh
2,400-case sweep, and geometrically unreachable under the current
generator + `pr2.urdf` fixture (tightest link pair needs `0.0232` reach,
the generator's own max possible reach is `0.0150`). Reproduced
independently this round, not merely cited: `cargo run --release --example
visibility_cone_depth_sweep -p moveit-constraints -- --seed 20260805
--cases 600` gives `touching==0: 300, touching==1: 300, touching>=2: 0`;
the same binary's `--geometry-gaps` flag gives the identical `0.0232`/
`0.0150` figures §169 reports. So every currently-reproducible
`visibility_cone` mismatch — not just the `touching==1` `105/115` — is
explained by deviation 6 alone; see deviation 6(b)'s own module doc
(round 21) for the direct confirmation on case 104 specifically.

Expires (§153.1): if the visibility-cone generator's `target_radius`
range or `sensor_offset` ever grows past `0.0232` (this population's own
tightest-pair reach), or `pr2.urdf`'s caster-wheel geometry changes,
§169's geometric-unreachability argument must be re-measured, not assumed
to still hold.

## Deviation 6(b)'s narrow-phase magnitude bias also explains `visibility_cone` case 104 (round 21)

Extends round 16/17's own base_link-vs-caster-wheel finding (this file's
`mesh_shape_cost_sources` OBB-fit section above and `parry.rs`'s deviation
6(b) own doc) to a second, independent mesh. Full method, numbers, and the
one caught-and-fixed methodology error (misapplied parry's own Y-axis
`axis_fix` to libccd's `ccd_cyl_t`, which is natively Z-axis per
`testsuites/support.c`) are in `parry.rs`'s deviation 6(b) doc itself,
round 21's own paragraph — not duplicated here.

| where | claim | verdict | evidence |
|---|---|---|---|
| `parry.rs` deviation 6(b), round 21 | case 104's own winning cone-mesh triangle, reconstructed from `tools/moveit-diff`'s captured spec and this crate's own `RobotModel`/`RobotState` FK, reproduces this backend's already-captured reference depth (`2.08696987934593702e-2`) | CONFIRMED | live probe this round (`crates/moveit-collision/tests/collision_parity.rs`, temporary, not committed — see this file's own precedent of external repros not becoming committed tests): `2.08696987934592244e-2`, `~1.5e-14` relative to the captured reference |
| same | the real, unmodified `ccdMPRPenetration` (libccd `v2.1`, `CCD_DOUBLE`, round 16/17's own build) on that exact triangle/cylinder pair reproduces the oracle's own reported depth for case 104 | CONFIRMED | `7.47919999515277989e-2` vs oracle's `7.47914550966356367e-2`, `5.4e-7` absolute / `~7.3ppm` relative |
| same | `ccd_cyl_t`'s own support function uses the cylinder's real (Z) axis directly, not parry's Y-axis convention | CONFIRMED | `/home/stevek/work/libccd/src/testsuites/support.c:62,68` (`ccdVec3Z(&dir)`); cross-checked against round 16's own `gen_cases.py`, which already used the same Z convention |

Expires (§153.1): re-open if a future moveit2 pin changes FCL/libccd's own
narrow-phase algorithm (deviation 6(b)'s own expiry condition, inherited
directly since this is the same mechanism, not a new one).

## Round 21's own case-104 finding, measured across a sample instead of one case (round 25)

The coordinator's round-24 feedback flagged that the section above rests
on one measured case and asked whether "the winning-triangle identification
and the vertex-1-at-centroid observation" generalize. Committed as an
in-tree test rather than left as prose:
`crates/moveit-collision/tests/collision_parity.rs`'s
`visibility_cone_near_placement_interpenetrates_through_the_touched_links_own_centroid`,
15 `(joint_state, target_radius, cone_sides)` combinations (3 real pr2
joint states from `load_self_wheel_oracle_points` × 5 `(radius,
cone_sides)` pairs spanning `visibility_cone_depth_sweep.rs`'s own
near-branch ranges).

Measuring it split what looked like one claim into two:

| where | claim | verdict | evidence |
|---|---|---|---|
| this test | every sampled near-placement's cone vertices stay inside the touched cylinder's own inscribed sphere and interpenetrate through its centroid by construction | CONFIRMED, generalizes | all 15/15: target-center vertex within `1e-9` of the cylinder's own local origin, `depth < 0.0` |
| this test | the *winning* (max-depth) triangle specifically contains the target-center vertex, the way case 104's own `[5, 1, 6]` did | REFUTED as a general rule | true in only 4/15 sampled combinations; the rest won through a triangle sharing the sensor vertex instead |

Case 104 was this mechanism's most visible instance, not its typical
shape. The claim that still holds and that deviation 6(b)'s doc actually
needs — every such case interpenetrates through the link's own centroid,
which is why this population's magnitudes are unusually large — does not
depend on which vertex the winning triangle happens to share, so this
refinement narrows the evidence without reopening the 115's own closure
above.

Expires (§153.1): if a future round changes `load_self_wheel_oracle_points`'s
joint states, `visibility_cone_depth_sweep.rs`'s near-branch
`target_radius`/`cone_sides` ranges, or pr2's caster-wheel geometry, the
4/15 rate must be re-measured, not assumed to still hold — the test's own
`assert_eq!(winning_triangle_had_vertex_one, 4, ...)` will fail loudly if
it moves.

## The out-of-tree libccd comparison harness is committed (round 26)

Round 25's committed harness (`tools/mpr-vs-epa/`) replaces this section's
own earlier round-25 draft, which pointed at an uncommitted scratch file
(§201: a C program that produced the number closing deviation 6 is
evidence for a claim this file depends on, not disposable scaffolding —
it belongs in-tree). Two pieces, one reconstruction:

- `crates/moveit-collision/examples/case104_mpr_input.rs` (this crate):
  reconstructs case 104 (pr2 FK from `tools/moveit-diff`'s own captured
  `joint_values`, `VisibilityConstraint::cone_mesh`'s formula reproduced
  the same way `collision_parity.rs`'s own generalization test reproduces
  it, this backend's own deepest-triangle-vs-cylinder search) and prints
  the winning triangle plus cylinder geometry to stdout — self-checked
  against the captured reference depth on every run (`assert!` against
  `-2.08696987934593702e-2`), so a future drift in either copy of the
  `cone_mesh` formula fails loudly instead of emitting silently-stale
  numbers.
- `tools/mpr-vs-epa/mpr_case104.c` + `build.sh`: takes that reconstruction
  on stdin and runs the real, unmodified `ccdMPRPenetration` (libccd `v2.1`,
  `CCD_DOUBLE`, `build.sh` pins and verifies the tag) on it — never
  re-deriving the triangle itself, exactly the `parry.rs` deviation-6(b)
  doc's own worked example, now reproducible from source instead of prose.

| where | claim | verdict | evidence |
|---|---|---|---|
| this harness | end-to-end (`cargo run --release --example case104_mpr_input -p moveit-collision \| tools/mpr-vs-epa/build/mpr_case104`) reproduces both previously-reported numbers | CONFIRMED | stdout `mpr_depth=7.47919999515277989e-02`, stderr EPA depth `-0.020869698793459224` — both match round 21/25's own reported figures exactly |

`tools/ci/` itself belongs to the orchestrator, not this panel (task
brief, standing scope rule) — `tools/ci/verify-mpr-vs-epa.sh` is not
written here. Spec for whoever writes it:

- Detect libccd the same way `build.sh` does: `git -C "$LIBCCD_SRC"
  describe --tags --exact-match` must read `v2.1`; default `LIBCCD_SRC` to
  `/home/stevek/work/libccd`, override via env var elsewhere.
- On found: `tools/mpr-vs-epa/build.sh`, then run the pipeline in this
  file's own table above. Assert `ccdMPRPenetration`'s depth is within
  some small relative tolerance of the oracle's own captured
  `7.47914550966356367e-2` (this round's own measurement: `~7.3ppm`; a
  tolerance in the `1e-5`–`1e-4` relative range gives margin without being
  vacuous) and print both numbers and their ratio.
- On not found: print a clearly-labeled `SKIP` line (not silence — §196)
  naming exactly what was not checked, and exit success. A CI system that
  reads only the exit code must not be able to mistake this for a real
  pass; the printed line is the only thing standing between "skipped" and
  "passed" for a human reading the log.

## The "MPR always deeper" framing was one case; N=945 falsifies it (round 27)

The coordinator's own round-27 charge: "One case cannot establish a
class... feed it every `visibility_cone` case where the port and the
oracle disagree on depth, and report the MPR-vs-EPA gap for each." Round
21/25/26's own case 104 is one case; `examples/visibility_cone_mpr_sweep.rs`
(new, this round) generalizes it — near-placement `visibility_cone` cases
anchored at every pr2 cylinder-wheel link, oracle ground truth from
`Op::Collision` (not `Op::Constraints`, see that file's own module doc for
why), this backend's own EPA depth from the identical reconstruction
`case104_mpr_input.rs` already uses, and `mpr_case104` fed the winning
triangle for every case where oracle and EPA disagree past `1e-4`.

| where | claim | verdict | evidence |
|---|---|---|---|
| this sweep | `visibility_cone_mpr_sweep --seed 4 --cases 1000` (pr2): of 945 mismatches with a real MPR reading, how many agree with round 21's "MPR always deeper" | REFUTED as a universal rule | 853 deeper (90.3%), 83 within float noise `<1e-9` (8.8%), **9 genuinely shallower (0.95%)**, gaps `-0.0088`..`-0.0147` — four orders of magnitude past the noise floor |
| same | gap magnitude correlated with penetration depth or triangle size | not strongly, either way | `pearson(gap, epa_depth) = 0.107`, `pearson(gap, triangle_size) = -0.182` |
| same | the 9 shallow cases share a distinguishing signature | CONFIRMED | 8 of 9 read `mpr_depth = 1.700000e-2` to six significant figures; pr2 wheel collision cylinders are `length="0.034"` (`fixtures/pr2.urdf`), `0.034 / 2 == 0.017` exactly — the cylinder's own half-length, not an arbitrary constant |
| same | the plateau is a wrong-triangle artifact (FCL/this reconstruction picked different winning triangles), not `ccdMPRPenetration` itself | REFUTED for 9/9 (round 28 below confirms case 623 too) | `mpr_case104` was fed the *same* triangle this reconstruction's own EPA search names as deepest; feeding that exact triangle directly (bypassing the oracle's pipeline entirely) reproduces `1.700000e-2` independently |
| same | case 623 (the 9th) reproduces the same plateau mechanism | CONFIRMED (round 28), and the oracle-side discrepancy itself CONFIRMED (round 29) as triangle selection, not a second mechanism | oracle's own `collision` op reports `7.479e-2` (a normal deep value) for the same (cone, link) pair where this reconstruction's own deepest triangle, fed directly to `mpr_case104` and instrumented the same way as the other 8, shows the identical `iterations=0`/axis-locked/`length/2` signature; round 29 below re-queries the same pair at `max_contacts_per_pair=32` on oracle image `700e7be54cb0a61f` and gets both values back in one response, 16 contacts, not 1 |

`parry.rs`'s deviation 6(b) doc is updated in the same commit with this
round's own paragraph (round 27), not a separate write-up — the doc that
made the now-falsified "by construction, always deeper" claim is the one
that needs correcting, and duplicating the numbers here without updating
there would leave a stale claim in the file that actually governs the
port's own behavior.

**Item 2 (what an independent witness catches and does not).**
`case104_mpr_input.rs`'s only self-check was `CAPTURED_REFERENCE_DEPTH`, a
constant derived by that same reconstruction the first time it ran — it
catches a later *regression* in the formula, never a bug already present
when the constant was captured (§188). `visibility_cone_mpr_sweep.rs`'s
`Op::Collision` ground truth is a real independent witness for the
*reconstruction* (wrong link, wrong joint, sign/scale error): such a bug
would make this backend disagree with the oracle's own FCL-computed depth
on nearly every case, not produce a plausible-looking self-consistent
pair. But it is not a perfect witness for *narrow-phase triangle
selection* specifically: `CollisionRequest::max_contacts_per_pair`
defaults to `1` (`collision_detection/collision_common.hpp:176`, confirmed by reading the
oracle's own vendored copy) even under `Op::Collision`, so the oracle
reports whichever single triangle FCL's own narrow-phase traversal found
first for a mesh-vs-shape pair, not necessarily this reconstruction's own
deepest — case 623 above is a real, measured instance of exactly that gap,
not a hypothetical one.

**Oracle-extension request for the orchestrator** (not implemented here,
`tools/moveit-oracle/` is out of this panel's scope): a settable
`max_contacts_per_pair` on the `collision` op's request, or an
all-contacts variant of `contactsToJson` for `robot_contacts` (report
every candidate triangle's own depth for a mesh-vs-shape pair, not only
`contact_list.front()`), would let a future round directly check whether
case 623's oracle-reported triangle matches this reconstruction's own
deepest-EPA pick — resolving the one open discrepancy above by
measurement instead of leaving it as a named-but-unconfirmed possibility.

Expires (§153.1): re-measure if the visibility-cone generator's
near-placement ranges or pr2's caster-wheel cylinder geometry change (same
expiry as round 25's own generalization test) — the `9/945` and `1.700e-2`
figures are this seed's own sweep, not an invariant of the mechanism's
existence.

## Round 28: the mechanism inside `ccdMPRPenetration`, and the exact oracle-extension request

The coordinator's round-28 charge, quoting it precisely because it sets
the bar this section has to clear: "the harness reproduces the number, it
does not explain it... establish *mechanically* whether the depth it
returns can be pinned to a support-point extent rather than the true
penetration... Either confirm it with the code path named line by line,
or refute it... Do not report the correlation again as if it were the
mechanism." Round 27's `pearson(gap, epa_depth) = 0.107` /
`pearson(gap, triangle_size) = -0.182` were exactly that — a correlation,
not a mechanism — and are not re-cited here as if they explained anything.

**Method.** Built libccd (`LIBCCD_SRC=/home/stevek/work/libccd`, tag
`v2.1`, confirmed clean before and after) with `-DMPR_DIAG` in a scratch
build directory, adding `fprintf(stderr, ...)` tracing inside
`mpr.c`'s own `findPenetr` loop — iteration count, `portalDir`'s own
outward normal, all three existing portal vertices, the new candidate
`v4`, and `portalReachTolerance`'s own `dv1..dv4`/`dot1..dot3` values —
without changing any of the algorithm's actual decision logic. Linked the
already-committed, unmodified `mpr_case104.c` against the instrumented
`.so`. Added `--dump-case <idx>` to `visibility_cone_mpr_sweep.rs` (this
round, committed) so each of the 9 shallow cases' *exact* fed geometry —
the same bytes the sweep already sends `mpr_case104` — could be re-fed to
the instrumented binary without re-deriving anything. Ran all 9 shallow
cases (40, 125, 179, 347, 395, 623, 862, 868, 970) plus 3 sampled deep
cases (4, 10, 41) as a contrast sample. Reverted the libccd checkout to
clean `v2.1` afterward (`git status`/`git diff --stat` both empty,
verified) — see `tools/mpr-vs-epa/README.md`'s new section for why the
instrumented copy itself is not committed (vendoring even a diagnostic
copy of upstream's own source is exactly the drift risk this harness's
whole design avoids).

| where | claim | verdict | evidence |
|---|---|---|---|
| this round | all 9 shallow cases stop at `findPenetr`'s very first iteration | CONFIRMED, 9/9 | every case's diagnostic log is exactly 2 lines: one iteration-0 trace line, one STOP line |
| same | the portal's own outward normal (`portalDir`) is exactly axis-locked at that first check | CONFIRMED, 9/9 | `dir=(0,0,1)` (or `(0,0,0.999999999999999889)`, float noise, for case 862) in every one of the 9 |
| same | all three existing portal vertices, and the new candidate, already sit at `z = length/2` | CONFIRMED, 9/9 | `v1.z=v2.z=v3.z=v4.z=1.700000e-2` exactly in every one of the 9 |
| same | `cylSupport` (`testsuites/support.c:54-69`) returns the cap's own *center*, not a rim point, when direction is exactly axial | CONFIRMED by source read | `zdist = sqrt(dir.x²+dir.y²)`; `ccdIsZero(zdist)` branch (`:60-62`) returns `(0,0,sign(dir.z)*height/2)` — matches every measured `v4` exactly |
| same | `portalReachTolerance` measures improvement strictly along `dir`, so a frozen-`z` axial portal reads zero improvement regardless of unrefined `x,y` spread | CONFIRMED by source read + measurement | `mpr.c:511-534`; measured `min(dv4-dv{1,2,3}) = 0.0` exactly in all 9, `<= mpr_tolerance (1e-10)` |
| same | `depth` is the point-to-triangle distance from the origin to the *frozen* portal face, not to the new candidate | CONFIRMED by source read | `mpr.c:325-330`; frozen face lies entirely in the `z=length/2` plane in all 9, giving `depth = length/2` exactly |
| same | 3 sampled deep cases (4, 10, 41) show the opposite at every step of this chain | CONFIRMED | non-axial `dir` at iteration 0 (`(0.707,0.135,-0.695)`, `(0.026,-0.654,0.756)`, `(0.600,-0.572,0.560)`), `16`–`24` real iterations, tolerance gap shrinking geometrically from `~2.3e-2`-`2.6e-2` to `1e-10`, depths (`0.0677`/`0.0734`/`0.0748`) with no simple relationship to cylinder dimensions |
| same | case 623's own winning triangle reproduces the identical 9/9 signature | CONFIRMED | same `iterations=0`/axis-locked/frozen-`z=length/2` trace as the other 8 — isolates case 623's own discrepancy to *which triangle* the oracle evaluated, not a second MPR mechanism (round 29 below measures that isolated question directly) |

**What is still open, stated plainly rather than folded into a tidier
claim**: *why* the portal discovery phases (`discoverPortal`/
`refinePortal`, both upstream of `findPenetr` and not instrumented this
round) land on an exactly-axis-locked portal for these particular 9
triangle/cylinder configurations and not the other 936, was not traced —
the mechanism above explains what an axis-locked portal *does*, not what
causes one to form for only some near-placement cases. Not asserted as
fact: a geometric conjecture that the winning triangle's own plane
happens to be close to perpendicular to the cylinder axis for these 9 is
consistent with the measurements but unmeasured this round.

**Item 3 — is the `max_contacts_per_pair` extension still needed after
this?** At the time this round wrote the request (`crates/moveit-
collision/doc/oracle-request-collision-max-contacts-per-pair.md`), yes,
and only for case 623's own question, not for the plateau mechanism (that
question was already closed). The request shipped on `main` before this
round's own report went out (`47a271c`, oracle image
`700e7be54cb0a61f`, `PORTING-PLAN.md` §212) — round 29 below uses it to
answer the question the request posed, closing case 623 by direct
measurement rather than leaving it a named-but-unconfirmed possibility.

Expires (§153.1): re-open the mechanism claim only if a future libccd pin
changes `mpr.c`'s own portal-refinement algorithm (same expiry class as
round 16/17/21's own finding) or `testsuites/support.c`'s `cylSupport`
degenerate-direction branch. The "why does `dir` land axial for these 9"
open question above has no expiry — it was never closed, so there is
nothing to re-open.

## Round 29: case 623 closed by direct measurement, on the shipped `max_contacts_per_pair` extension

The coordinator built round 28's requested extension and merged it before
this round's own report went out: `47a271c` ("moveit-oracle: let the
collision op report every contact of a pair"), oracle image
`moveit-rs/oracle:700e7be54cb0a61f` (stamp moved from `043ed31a2186fe4e`,
recorded in `PORTING-PLAN.md` §212, verified byte-identical against the
old image on the existing 52-fixture corpus by `verify-fixture-replay.sh`
per the coordinator's own §212 note). This branch was rebased onto that
merge (`git merge main`, fast-forward, no conflicts) before this
section's own measurement was taken, so the number below is against
`700e7be54cb0a61f`, not the `043ed31a2186fe4e` this crate's earlier rounds
were captured against. Re-ran the full seed-4, 1000-case default sweep
(no new flags) against `700e7be54cb0a61f` before trusting any of round
27/28's own numbers under the new image rather than assuming the
coordinator's own byte-identical claim covers this crate's own sweep too:
identical result — 945 mismatches, 853 deeper / 83 equal-within-noise / 9
shallower, the same 9 case indices (40, 125, 179, 347, 395, 623, 862,
868, 970), same gap values to the printed precision. Round 27/28's own
numbers stand unchanged under the new image; this round's own new
measurement (case 623's 16-contact dump, below) is the only new capture
against `700e7be54cb0a61f`.

**What was measured.** Added `--max-contacts-per-pair <N>` and
`--dump-contacts <idx>` to `visibility_cone_mpr_sweep.rs` (this round):
the former threads a `max_contacts_per_pair` field through to the
`collision` op's own request (new field on `Op::Collision`, omitted by
default so every other invocation of this binary is unaffected — the
oracle itself defaults it to `1`, unchanged); the latter, given a case
index, prints every contact the oracle returned for the touched
(cone, link) pair at that index and exits, instead of running the usual
EPA/MPR comparison. Ran:

```
visibility_cone_mpr_sweep --urdf <abs>/pr2.urdf --srdf <abs>/pr2.srdf \
    --seed 4 --cases 624 --dump-contacts 623 --max-contacts-per-pair 32 \
    --oracle <abs>/tools/moveit-oracle/run-oracle.sh
```

against `moveit-rs/oracle:700e7be54cb0a61f` via `sg docker`. `32` was
chosen to exceed the case's own cone mesh triangle count
(`cone_sides = 3 + 623 % 6 = 9`, `2 * cone_sides = 18` triangles) with
margin, so no legitimate contact could be truncated by the request field
itself.

**Result:** the `(cone, fr_caster_r_wheel_link)` pair returned **16**
contacts in one response (`touching` was previously reported as this
pair's single entry, at the old default of `1`), not 1:

- 13 entries cluster at `1.700000e-2`-`1.700000e-2` (to 6 significant
  figures) — the plateau, matching this backend's own EPA/MPR reading of
  its own winning triangle.
- 3 entries cluster at `7.359598e-2`-`7.479154e-2` — the deep value,
  `0.07479153841188203` among them, matching `parry.rs`'s own
  deviation-6(b) doc's recorded figure for this case (`7.479e-2`, the only
  precision at which round 27/28 had recorded it) to 4 significant
  figures.

Both this backend's own value and the oracle's own originally-reported
value are present, simultaneously, in FCL's own contact list for the
identical robot state and object. This is not consistent-with; it is the
measurement round 28's own Item 3 named as the thing that would settle
it: `max_contacts_per_pair = 1` was reporting exactly one of at least two
genuinely-touching triangles, and the one it reported for case 623
happened not to be the one this backend's own exhaustive deepest-first
search names as deepest.

| where | claim | verdict | evidence |
|---|---|---|---|
| this round | case 623's oracle-reported depth and this backend's own reading are both real, simultaneously-touching triangles of the same pair | CONFIRMED | 16-entry contact list at `max_contacts_per_pair=32`: 13 near `1.700e-2`, 3 near `7.36e-2`-`7.479e-2`, one measurement, one oracle response |
| same | the `max_contacts_per_pair=1` default was truncating to one of several candidates, not reporting a uniquely-correct one | CONFIRMED | both this backend's own value and the oracle's original single-reported value are members of the same 16-entry list — neither is absent from what FCL itself found |

**Case 623 is closed.** It is the same 9/9 `ccdMPRPenetration` mechanism
as the other 8 cases, observed through an oracle response that, before
this field existed, could only ever show one of several genuinely-
touching candidate triangles per pair. Item 3's own request is fulfilled
and used, not merely shipped.

Expires (§153.1): re-open only if `CollisionRequest::max_contacts_per_pair`'s
own semantics change upstream, or if `oracle.cpp`'s `allContactsToJson`
wiring is reverted or altered. Does not re-open round 28's own mechanism
finding, which this round leaves untouched.

## Round 30: the radius plateau is a different mechanism, and round 29's own count was smaller than the tool already covers

Round 29 counted `mpr == 7.479200e-2` (the cylinder's own radius) in 101 of
586 `mpr=`-bearing lines and `oracle == 1.700000e-2` (`length/2`) in 381 of
623 `oracle=`-bearing lines, by hand, from raw sweep output through case 623.
Two open questions followed: whether the radius plateau has its own
degenerate branch in `support.c` the way `ccdIsZero(zdist)` produces the
`length/2` one, and whether those counts hold over the sweep's own full
population rather than the range one case (623) happened to bound them to.

**Counting.** Added a `Reading` struct and `print_plateau_histogram` to
`visibility_cone_mpr_sweep.rs`: every oracle/EPA/MPR reading is classified
against its own case's cylinder `radius`/`length/2` at a `1e-6` relative
tolerance (`"axial(length/2)"` / `"radial(radius)"` / `"other"`), and the
sweep prints the histogram itself at the end of every run, plus `(n=...)` on
the Pearson line. Re-ran the full default sweep (seed 4, `--cases 1000`,
`moveit-rs/oracle:700e7be54cb0a61f`):

```
oracle plateau histogram (n=1000): axial(length/2)=619 (61.9%)  radial(radius)=0 (0.0%)  other=381 (38.1%)
epa plateau histogram (n=1000): axial(length/2)=0 (0.0%)  radial(radius)=0 (0.0%)  other=1000 (100.0%)
mpr plateau histogram (n=945): axial(length/2)=9 (1.0%)  radial(radius)=153 (16.2%)  other=783 (82.9%)
pearson(gap, epa_depth)=0.1066  pearson(gap, triangle_size)=-0.1823  (n=945)
```

and, separately, at `--cases 624` (round 29's own range, to check the two
counts agree at the same point): `oracle` axial `381/624` (round 29: `381 of
623`), `mpr` radial `101/587` (round 29: `101 of 586`), `mpr` axial `6/587`.
Both agree with round 29's own hand count to the last digit at the shared
range — round 29 was not wrong, its count was just smaller than the
population the same tool already had. `853`/`9`/`83` deep/shallow/noise-equal
of `945` and the 9 sign-flip case indices are unchanged from round 27/28/29
under this image, reproduced again this round rather than assumed.

**The radial(radius) plateau: sampling and instrumentation.** `153` cases
have `mpr == radius`; drew the first 20 by index (`4, 5, 17, 28, 51, 60, 71,
98, 110, 116, 134, 141, 147, 161, 162, 166, 167, 173, 181, 182` — includes
case 4, the case round 29 flagged as its own "mechanism not firing" contrast
that turned out to sit on this plateau) as the primary sample, plus a
10-case contrast set of the first 10 `other`-bucket (neither-plateau)
mismatches by index (`0, 1, 2, 6, 7, 8, 9, 10, 11, 12`). `20` and `10` were
chosen to be an order of magnitude above round 28's own 9-case/3-case
sample, at a size where reading every trace individually (rather than only
aggregate stats) is still tractable within this round; both counts are
stated here as sample sizes, not claimed as the full 153/783 populations.
Built libccd from the pinned `v2.1` source with `-DMPR_DIAG` (same scratch,
uncommitted instrumentation pattern as round 28 — see
`tools/mpr-vs-epa/README.md`'s own "Reproducing round 28's mechanism
finding" section for why nothing from this build goes in the tree), fed each
sampled case's `--dump-case` geometry to the instrumented binary, kept
stderr.

**Result: not the same mechanism as the axial plateau.** All 20/20
radial-plateau traces show genuine multi-iteration convergence
(`iterations` 15 or 16 in every one, never 0) — `findPenetr`'s own `while(1)`
loop (`mpr.c:317`, `portalDir`/`__ccdSupport` per iteration at `:319-320`),
its tolerance gap (`portalReachTolerance`'s own `dv4 - dv{1,2,3}`,
`mpr.c:511-534`) shrinking geometrically to below `mpr_tolerance` rather than
hitting the exact `0.0` a degenerate branch returning the same point twice
produces. Case 4's own sequence: `2.545e-2` (iteration 0) -> `6.852e-10`
(iteration 13) -> `1.713e-10` (14) -> `4.283e-11` (15, stop), crossing
`1e-10` naturally; that exact `0.0` freeze value never appears in any of the
20 traces. `dir`'s own `z` component converges toward, but never reaches,
exactly `0` (case 4: `-0.6946` at iteration 0, `-3.97e-9` at 13, `-8.30e-10`
at the iteration-15 stop) — `ccdSign`'s own `val < CCD_EPS` tie
(`vec3.h:206-219`, the literal `ccdSign(0)==0` case) never fires; `zdist`
(`support.c:58-59`) stays well clear of `ccdIsZero`'s threshold throughout.

**What actually produces the radius value.** `support.c:54-69`'s cylinder
support function has two branches, and *both* set `z = ccdSign(dz) *
height/2` (`:62` in the `ccdIsZero(zdist)` branch, `:68` in the general
one) — this is the analytically-exact support point for a solid capped
cylinder (a disc x interval product domain: a linear functional over a
product domain maximizes each factor independently, so whenever `dz != 0`,
however small, the true maximizing `z` is exactly one rim, never a point on
the smooth curved belt between the caps). The belt is never a reachable MPR
support point except at the exact tie `dz==0`. Every one of the 20 sampled
traces' final portal vertices sit almost exactly on the two rim circles
(radius match to 9-10 significant figures; `z` exactly `+height/2` on some
vertices and exactly `-height/2` on others within the *same* converged
triangle — case 4's final `v1.z=-0.017, v2.z=+0.017, v3.z=-0.017`). That
convergence exists because the near-placement generator places one cone
mesh vertex exactly on the touched cylinder's own axis, every time:
`visibility_cone_mpr_sweep.rs:766` sets `anchor = cyl_frame.translation
.vector` and `:776` builds the cone's own "target" vertex (mesh vertex
index 1) at exactly that point; transformed into the cylinder's own local
frame for the MPR feed (`:391-392`), that vertex lands at bit-exact
`(0.0, 0.0, 0.0)` — confirmed in the dump for all 20/20 sampled
radial-plateau cases (the 10-case contrast sample splits 4/10 with that
vertex present, 6/10 without). With a triangle vertex exactly on-axis, the
true nearest boundary feature for that vertex is symmetric between the two
rims and independent of height, so the portal's refinement genuinely
converges its outward normal toward purely radial (`dir.z -> 0`) using only
the rim points `ccdSupport` can ever return — and the resulting depth
converges to the radius because that genuinely is the distance from an
on-axis point to the cylinder's own curved boundary. It is not the true
minimum-translation depth of the whole triangle, only of that one vertex;
this backend's own EPA, using all three vertices together, reports a
smaller, correct depth in the large majority of cases (the `853/945`
"deeper" figure above).

The contrast sample separates two other populations. 4/10 (cases `0, 6, 7,
9`) also have the on-axis vertex but land in `other`: `dir` locks to
*exactly* `(0,0,+/-1)` (the discrete `ccdIsZero(zdist)` freeze round 28
traced for the axial plateau) but after 3-5 real iterations rather than at
iteration 0 (`iterations` 4, 3, 3, 5), so the frozen triangle carries real
pre-freeze structure and the resulting depth (`6.286e-2`, `3.677e-2`,
`3.196e-2`, `6.585e-2`) is neither plateau — same freeze mechanism as round
28's axial cases, a later stopping point, no plateau value. The remaining
6/10 (no on-axis vertex) show a continuum: 4 of them (cases `2, 8, 10, 12`)
converge over 20-24 real iterations to `0.0703`-`0.0734`, close to but
outside the plateau's `1e-6` relative tolerance around `0.074792` —
near-but-not-exact axis proximity gives a near-but-not-exact radius reading,
consistent with a genuine geometric limit and not a discrete branch; the
other 2 (cases `1, 11`) converge in 2 iterations to `0.0189`/`0.0337`,
unrelated to either dimension.

| where | claim | verdict | evidence |
|---|---|---|---|
| this round | radial(radius) plateau is a second literal `ccdIsZero`-style degenerate branch, distinct copy of round 28's axial mechanism | REFUTED | 0/20 sampled traces hit `iterations=0` or the exact `min(dv4-dv123)=0.0` freeze signature; all 20 converge over 15-16 real iterations |
| this round | radial(radius) plateau is `ccdSupport`'s exact rim-only support range converging toward the radius for a query dominated by an on-axis vertex | CONFIRMED | 20/20 sampled cases have a mesh vertex at bit-exact cylinder-local `(0,0,0)` (generator's own near-placement construction, `visibility_cone_mpr_sweep.rs:766,776,391-392`); `dir.z -> 0` (not `==0`) over 15-16 iterations in all 20; final portal vertices land on both rim circles at radius-matching (x,y) |
| this round | round 29's `101/586`, `381/623` counts hold over the sweep's full population, not just the range they were counted in | CONFIRMED | full `n=1000`/`945` recount agrees with the `n=624`/`587` range at the shared point to the last digit; new fuller figures (`619/1000`, `9+153=162/945`) stated above |

**Deviation 6(b), re-scoped.** `parry.rs`'s own deviation-6(b) doc is
updated in the same commit as this section: the "MPR deeper than EPA by
construction, 9 shallow outliers" framing is kept for the `82.9%`
(`783/945`) `other`-bucket mismatches where a real MPR convergence happened,
and a new paragraph states plainly that for a majority of oracle readings
(`619/1000`, `61.9%`) and `17.1%` of MPR mismatches (`162/945`, `9` axial +
`153` radial), *neither* channel is reporting a real narrow-phase depth for
this pair shape — both plateau on one of the cylinder's own two dimensions,
via different mechanisms, while this backend's own EPA never does (`0/1000`
on either plateau). FCL's own internal cause for its `length/2` plateau
(`619/1000` of oracle readings) was not traced this round — instrumenting
FCL's own narrow-phase is outside `tools/mpr-vs-epa/`'s libccd-only design
and outside this crate's own scope.

**UNFIXED / not this round.** FCL's own mechanism for its `length/2`
plateau (oracle side, `619/1000`) — libccd's own mechanism is now traced
end-to-end (round 28 axial, round 30 radial), FCL's is not, and FCL is not
this panel's own source to instrument.

Expires (§153.1): re-open if a future libccd/FCL pin changes either
narrow-phase's own convergence behavior, or if `pr2.urdf`'s wheel-cylinder
dimensions change (the exact plateau values are this fixture's own
`radius="0.074792"`/`length="0.034"`, not universal constants).
