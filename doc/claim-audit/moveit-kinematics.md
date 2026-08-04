# Claim audit — moveit-kinematics

Type-b claim audit per PORTING-PLAN.md §175. This crate has no plugin/adapter
dispatch layer the way moveit-planners-pilz does (see
[`doc/claim-audit/moveit-planners-pilz.md`](../claim-audit/moveit-planners-pilz.md)
for the crate where that risk actually materialized), so this pass is a
grep-plus-read sweep rather than a full per-claim upstream re-derivation.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `src/lib.rs:33-42` | `kdl_kinematics_plugin` builds a `KDL::Chain` from the URDF and gets its Jacobian/FK from KDL's own solvers (`ChainJntToJacSolver`, `ChainFkSolverPos_recursive`); this port replaces both with `moveit_state::Posed::jacobian`/`global_link_transform`. | CONFIRMED | `kdl_kinematics_plugin.cpp` — `KDLKinematicsPlugin::initialize` builds `kdl_chain_` via `tree.getChain(...)`; `getPositionFK`/velocity-IK paths construct `ChainFkSolverPos_recursive`/`ChainJntToJacSolver` against it. Matches the claim; not a dispatch-reachability claim (no earlier branch could redirect around this), so lower audit priority than the pilz pattern, but checked anyway since it's the crate's central architectural claim. |  |
| `src/params.rs:35-36` | `joint_weights`'s default-then-override behavior matches `KDLKinematicsPlugin::getJointWeights`. | CONFIRMED (spot check only, not full re-derivation this round) | `kdl_kinematics_plugin.cpp`'s `getJointWeights` — default `1.0` per joint, overridden by name from parsed weights. Consistent with the doc claim. |  |
| `src/cached_solver.rs` (`CachedIkSolver`, PORTING-PLAN.md §177.1) | `lma_cached` returning a different IK solution than `lma` for the same `(pose, seed)` query, even when the caller's seed is already exact, is upstream's own approximate-nearest-neighbor cache design (empty-cache tries an all-zero dummy seed first, caller's own seed only as fallback) — not a bug this port introduces via wrong cache keying/seeding. | CONFIRMED | `cached_ik_kinematics_plugin/src/ik_cache.cpp:159-168` (`IKCache::getBestApproximateIKSolution`): `if (ik_cache_.empty()) { static IKEntry dummy = std::make_pair(std::vector<Pose>(1, pose), std::vector<double>(num_joints_, 0.)); return dummy; }` — confirms the all-zero-first behavior on a cold cache is upstream's own code, matching this port's `CachedIkSolver` exactly. Doc comment added to `CachedIkSolver` recording this as "why a caching solver must never be an implicit default." | `697d83c` |
| `src/registry.rs` (`KINEMATICS_SOLVERS`, PORTING-PLAN.md §177) | Six call sites across `moveit-planners-pilz` used `KINEMATICS_SOLVERS.iter().find_map(...)` ("first registration that constructs") as an implicit selection rule; `SolverRegistration::name`'s own doc comment already documented `name` as "the name a caller scanning `KINEMATICS_SOLVERS` matches on," so a name-keyed contract existed but no call site used it — the linker's link-section order (a byproduct of the whole workspace's dependency graph, not a value in the source) was silently standing in for a selection rule. | CONFIRMED (internal-consistency defect, not an upstream-claim mismatch) | Reproduced directly: adding `thiserror` to an unrelated crate (`moveit-octomap`) flipped `KINEMATICS_SOLVERS`'s iteration order from `["lma", "newton_raphson", "lma_cached", "newton_raphson_cached"]` to `["lma_cached", "newton_raphson_cached", "lma", "newton_raphson"]`, changing which solver every `.find_map` site silently picked. Fixed with one owner API, `resolve_solver(model, group_name, name, params)`, resolving by an explicit `name` value in the source; all six sites in `moveit-planners-pilz` now route through it. `DEFAULT_SOLVER_NAME = "newton_raphson"` chosen because panda's/fanuc's `kinematics.yaml` both configure `kdl_kinematics_plugin/KDLKinematicsPlugin`, whose only velocity-IK path upstream ever builds is `KDL::ChainIkSolverVelMimicSVD` (confirmed by grepping `kdl_kinematics_plugin.cpp` for LMA/Levenberg — no such option exists upstream), which `NewtonRaphsonSolver`'s own doc comment already calls "the solver that ports `ChainIkSolverVelMimicSVD` as-is"; `"lma"` is this port's own addition, not a faithful port of anything upstream ships. | `3c32e94` |

## Swept, no claim found needing verification

`src/chain.rs:518`, `src/velocity.rs:95`, `src/lma.rs:4` — self-referential or
"see this method's own doc comment" pointers, not claims about a separate
upstream code path's reachability.

## §172 two-anchor narrowing sweep — negative result, logged for §153.1 expiry tracking

Anchor 1 (upstream, run first per §172): `static_cast<int|size_t|unsigned|
long|short>`, C-style `(int)`/`(size_t)`/`(unsigned)`/`(long)` casts, an
`int`/`size_t`/`unsigned`/`long`/`short` declaration whose RHS is a float
literal or a division, and `floor`/`ceil`/`round`/`sqrt`/`pow` near an
integer declaration — swept via `rg` against every upstream file this
crate's `Ported from` headers cite (`kinematics_base.{hpp,cpp}`,
`kdl_kinematics_plugin.{hpp,cpp}`, `joint_mimic.hpp`,
`cached_ik_kinematics_plugin.hpp`, `cached_ik_kinematics_plugin-inl.hpp`,
`ik_cache.cpp`), each file present and read at the path cited. All four
sweep patterns: 0 hits across the whole set.

Anchor 2 (port side): `as (i8|i16|i32|i64|isize|u8|u16|u32|u64|usize)` in
`crates/moveit-kinematics/src/`, enumerated on screen. 0 hits.

| where (upstream file swept, anchor 1) | claim | verdict | evidence |
|---|---|---|---|
| `kinematics_base/include/.../kinematics_base.hpp` | no int/size_t/unsigned/long/short decl or cast with a floating-point initializer | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `kinematics_base/src/kinematics_base.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `kdl_kinematics_plugin/include/.../kdl_kinematics_plugin.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `kdl_kinematics_plugin/include/.../joint_mimic.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `cached_ik_kinematics_plugin/include/.../cached_ik_kinematics_plugin.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `cached_ik_kinematics_plugin/include/.../cached_ik_kinematics_plugin-inl.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `cached_ik_kinematics_plugin/src/ik_cache.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |

Anchor 2 hit, classified: none — `rg` returned 0 matches in
`crates/moveit-kinematics/src/`, so there is nothing to enumerate this
round.

Expires (§153.1): if a future round adds an int/size_t/unsigned/long/short
declaration with a floating-point initializer to any file above, adds a
new upstream file citing floating-point narrowing to this crate's cited
set, or adds an `as iNN/uNN/usize/isize` cast receiving an `f64` expression
to `crates/moveit-kinematics/src/`, this table's `CONFIRMED (absent)` rows
must be re-swept, not assumed to still hold.
