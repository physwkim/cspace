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
| new module doc deviation 1 | `CollisionData::enableGroup` resolves `req_->group_name` to `JointModelGroup::getUpdatedLinkModelsSet()` when the model has that group, else `nullptr` | CONFIRMED | `collision_common.cpp:1012-1022` |
| new module doc deviation 1 | `collisionCallback` skips a pair only when *neither* side resolves to an active link (world objects never resolve to one, so a robot-vs-world pair is kept iff the robot link is active; a self-pair is kept iff *either* link is) | CONFIRMED | `collision_common.cpp:79-94` |
| new module doc deviation 1 | `distanceCallback` reads the identical `active_components_only` from `DistanceRequest`, so `distance_self`/`distance_robot` need the same filter even though `distanceSelf`/`distanceRobot` themselves never call `enableGroup` (their caller is expected to have already populated it) | CONFIRMED | `collision_common.cpp:482-500` (callback); `collision_env_fcl.cpp:288-290`/`345-347` (the caller populating it before calling `distanceSelf`/`distanceRobot`) |
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

## Item deferred to the orchestrator: an out-of-tree libccd comparison gate (round 25)

The coordinator's round-24 feedback also asked for a `tools/ci/` script
that builds libccd when present and drives both `ccdMPRPenetration` and
this backend's own EPA on case 104's triangle/cylinder pair, loudly
skipping (not silently) when libccd is absent. `tools/ci/` belongs to the
orchestrator, not this panel (task brief, standing scope rule) — not
written here. Spec, so the orchestrator or whichever panel owns
`tools/ci/` doesn't have to re-derive it from this file's prose:

- Detect libccd the same way this round's own scratch build did: look for
  a system install first (`pkg-config --exists ccd` or a `ccd/ccd.h`
  header on the include path), then optionally a pinned from-source build
  at a fixed path/tag (`v2.1`, `CCD_DOUBLE`) if the orchestrator wants
  reproducibility independent of the host's package version.
- On found: build and run a small C harness (round 21's own
  `dev7/mpr_case104.c` scratch file is a working reference — not
  committed anywhere, ask this panel for a copy if a starting point
  helps) against case 104's fixed triangle/cylinder numbers (both already
  committed: the triangle in this crate's
  `visibility_cone_near_placement_interpenetrates_through_the_touched_links_own_centroid`/
  case-104's own doc comment in `parry.rs`, the cylinder radius/length in
  `pr2.urdf`'s `bl_caster_l_wheel_link`). Assert `ccdMPRPenetration`'s
  depth is within some small relative tolerance of the oracle's own
  captured `7.47914550966356367e-2` (this round's own measurement:
  `~7.3ppm`; a tolerance in the `1e-5`–`1e-4` relative range gives margin
  without being vacuous) and print both numbers and their ratio.
- On not found: print a clearly-labeled `SKIP` line (not silence — §196)
  naming exactly what was not checked, and exit success. A CI system that
  reads only the exit code must not be able to mistake this for a real
  pass; the printed line is the only thing standing between "skipped" and
  "passed" for a human reading the log.
