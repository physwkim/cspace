# Claim audit — moveit-kinematics

Type-b claim audit per PORTING-PLAN.md §175. This crate has no plugin/adapter
dispatch layer the way moveit-planners-pilz does (see
[`doc/claim-audit/moveit-planners-pilz.md`](../claim-audit/moveit-planners-pilz.md)
for the crate where that risk actually materialized), so this pass is a
grep-plus-read sweep rather than a full per-claim upstream re-derivation.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-kinematics/src/lib.rs:33-42` | `kdl_kinematics_plugin` builds a `KDL::Chain` from the URDF and gets its Jacobian/FK from KDL's own solvers (`ChainJntToJacSolver`, `ChainFkSolverPos_recursive`); this port replaces both with `moveit_state::Posed::jacobian`/`global_link_transform`. | CONFIRMED | `kdl_kinematics_plugin.cpp` — `KDLKinematicsPlugin::initialize` builds `kdl_chain_` via `tree.getChain(...)`; `getPositionFK`/velocity-IK paths construct `ChainFkSolverPos_recursive`/`ChainJntToJacSolver` against it. Matches the claim; not a dispatch-reachability claim (no earlier branch could redirect around this), so lower audit priority than the pilz pattern, but checked anyway since it's the crate's central architectural claim. |  |
| `src/params.rs:35-36` | `joint_weights`'s default-then-override behavior matches `KDLKinematicsPlugin::getJointWeights`. | CONFIRMED (spot check only, not full re-derivation this round) | `kdl_kinematics_plugin.cpp`'s `getJointWeights` — default `1.0` per joint, overridden by name from parsed weights. Consistent with the doc claim. |  |
| `src/cached_solver.rs` (`CachedIkSolver`, PORTING-PLAN.md §177.1) | `lma_cached` returning a different IK solution than `lma` for the same `(pose, seed)` query, even when the caller's seed is already exact, is upstream's own approximate-nearest-neighbor cache design (empty-cache tries an all-zero dummy seed first, caller's own seed only as fallback) — not a bug this port introduces via wrong cache keying/seeding. | CONFIRMED | `cached_ik_kinematics_plugin/src/ik_cache.cpp:159-168` (`IKCache::getBestApproximateIKSolution`): `if (ik_cache_.empty()) { static IKEntry dummy = std::make_pair(std::vector<Pose>(1, pose), std::vector<double>(num_joints_, 0.)); return dummy; }` — confirms the all-zero-first behavior on a cold cache is upstream's own code, matching this port's `CachedIkSolver` exactly. Doc comment added to `CachedIkSolver` recording this as "why a caching solver must never be an implicit default." | `697d83c` |
| `src/registry.rs` (`KINEMATICS_SOLVERS`, PORTING-PLAN.md §177) | Six call sites across `moveit-planners-pilz` used `KINEMATICS_SOLVERS.iter().find_map(...)` ("first registration that constructs") as an implicit selection rule; `SolverRegistration::name`'s own doc comment already documented `name` as "the name a caller scanning `KINEMATICS_SOLVERS` matches on," so a name-keyed contract existed but no call site used it — the linker's link-section order (a byproduct of the whole workspace's dependency graph, not a value in the source) was silently standing in for a selection rule. | CONFIRMED (internal-consistency defect, not an upstream-claim mismatch) | Reproduced directly: adding `thiserror` to an unrelated crate (`moveit-octomap`) flipped `KINEMATICS_SOLVERS`'s iteration order from `["lma", "newton_raphson", "lma_cached", "newton_raphson_cached"]` to `["lma_cached", "newton_raphson_cached", "lma", "newton_raphson"]`, changing which solver every `.find_map` site silently picked. Fixed with one owner API, `resolve_solver(model, group_name, name, params)`, resolving by an explicit `name` value in the source; all six sites in `moveit-planners-pilz` now route through it. `DEFAULT_SOLVER_NAME = "newton_raphson"` chosen because panda's/fanuc's `kinematics.yaml` both configure `kdl_kinematics_plugin/KDLKinematicsPlugin`, whose only velocity-IK path upstream ever builds is `KDL::ChainIkSolverVelMimicSVD` (confirmed by grepping `kdl_kinematics_plugin.cpp` for LMA/Levenberg — no such option exists upstream), which `NewtonRaphsonSolver`'s own doc comment already calls "the solver that ports `ChainIkSolverVelMimicSVD` as-is"; `"lma"` is this port's own addition, not a faithful port of anything upstream ships. | `3c32e94` |

## Swept, no claim found needing verification

`src/chain.rs:518` (`build_rejects_an_in_chain_mimic_whose_master_is_outside_the_group`), `src/velocity.rs:95`, `src/lma.rs:4` — self-referential or
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

## Round 10 follow-up — `src/set_from_ik.rs` (`setFromIK`/`setFromIKSubgroups`)

The rows above predate this file. `crates/moveit-kinematics/src/set_from_ik.rs` is the crate's one port
of a `RobotState` method rather than of a plugin, so every claim it makes
is about `moveit_core/robot_state/src/robot_state.cpp` and its neighbours
rather than about `kdl_kinematics_plugin.cpp`. Each row below was
re-derived by opening the cited range in the pinned checkout
(`e017c91ee12984393a28ba246075c65f69cde3bf`) this round; none is carried
over from the port's own commit message or from a prior round's reading.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-kinematics/src/set_from_ik.rs:346-367` (`resolve_ik_queries`' matching loop) | Two frames sharing `getRigidlyConnectedParentLinkModel` are interchangeable as an IK target, and the pose is carried across by `pose * pose_parent_to_frame.inverse() * tip_parent_to_tip`. | CONFIRMED | `robot_state.cpp:1922-1945`, read in full. `1938` is `if (pose_parent == tip_parent)` and `1941` is the identical product. One thing the upstream *names* hide and the port's do not: `pose_parent_to_frame` (`:1930`) and `tip_parent_to_tip` (`:1937`) are both `getFrameTransform(...)` results, i.e. model-frame-to-frame, not parent-to-frame — so the product is `(model→frame)⁻¹ * (model→tip)`, the constant frame-to-tip transform, and the port's `frame_transform(...)` pair is the same quantity under an honest name. `pose` is already in the solver base by then on both sides (`:1901` / `crates/moveit-kinematics/src/set_from_ik.rs:343`), so the result is base-to-tip on both. | |
| `crates/moveit-kinematics/src/set_from_ik.rs:264-278` (`rigid_parent_link`) | Resolves a link first and an `AttachedFrames` frame second, and does **not** accept the model frame, so a target naming the model frame is an error. | CONFIRMED | `robot_state.cpp:939-945` (`RobotState::getRigidlyConnectedParentLinkModel`) forwards `getLinkModelIncludingAttachedBodies(frame)` into the model-level walk. That resolver, `:910-937`, has exactly three tiers — link, attached body, attached-body subframe — and no model-frame tier, unlike `getFrameInfo` (`:1345-1350`), which does have one. A model-frame target therefore yields `nullptr` at `:942`, `robot_model.cpp:1369-1370` passes the `nullptr` straight back, and `robot_state.cpp:1925-1929` turns it into `return false`. The port returns `Error::UnknownName` at the same boundary rather than `Ok(false)`, which is the documented "caller error, not no-solution" split. | |
| `crates/moveit-kinematics/src/set_from_ik.rs:289-303` (`frame_transform`) | Model frame and links come from `moveit-state`, attached bodies and subframes from the injected `AttachedFrames`, links tried first so this and `rigid_parent_link` cannot resolve one name two ways. | CONFIRMED | `robot_state.cpp:1338-1384` (`getFrameInfo`, which `getFrameTransform` is a thin wrapper over at `:1314-1336`, the const overload's whole body being one `getFrameInfo` call plus a not-found warning): leading-`/` strip, then model frame (`:1345-1350`), then link (`:1351-1355`), then attached body by name (`:1359-1367`), then attached-body subframe (`:1370-1379`). The port collapses the last two into one `AttachedFrames::attached_frame` call because both answer with the same attached link, which is what `:1363` and `:1375` return. | |
| `crates/moveit-kinematics/src/set_from_ik.rs:380-386` (`resolve_ik_queries`' fill loop) | A solver tip no target claimed is filled with the pose that tip currently holds. | CONFIRMED, with one difference in the *failure* case | `robot_state.cpp:1977-2007`. The port uses `frame_transform(&posed, attached, tip)` where upstream uses `getGlobalLinkTransform(solver_tip_frame)` (`:1992`) — link-only, and its `std::string` overload feeds `getLinkModel`'s `nullptr` into a dereference for a tip that is not a link. Upstream keeps that unreachable by validating the tip at plugin init instead (`kdl_kinematics_plugin.cpp:179-183`). The port's lookup answers for the same links and returns `Err` instead of dereferencing for anything else, so no reachable behaviour differs. | |
| `crates/moveit-kinematics/src/set_from_ik.rs:245-251` (`to_solver_frame`) | `Transforms::sameFrame(ik_frame, model_frame)` short-circuits before any link lookup, because the model frame need not name a link. | CONFIRMED | `robot_state.cpp:1771-1786` (`setToIKSolverFrame`): `:1774` is the `sameFrame` guard and the `getLinkModel` call is inside it at `:1776-1777`, so upstream also never looks a link up for a model-frame base. `:1778-1782` is the error the port's `Error::UnknownName` replaces. | |
| `crates/moveit-kinematics/src/set_from_ik.rs:422-440` (`solver_solution_variables`) | The name check upstream performed while building the bijection is kept; the permutation itself has no consumer. | **REFUTED as first written, fixed this round** | `joint_model_group.cpp:627-637` is not only a name check: `:630-632` is `// skip reported fixed joints`, which lets a solver report a joint of the group holding no variable and gives it no bijection entry. The port rejected that name instead. `:166-168` and `:176-177` are why the skip is needed at all — a joint with `getVariableCount() == 0` goes to `fixed_joints_` and never enters `joint_variables_index_map_`. The permutation's *slot correspondence* also turned out to have a consumer, three in fact (the seed, the hook's write, and the final write), which is what made a `continue` in the check alone unsafe. | `0f6edb7` |
| `crates/moveit-kinematics/src/lib.rs:234-239`, `crates/moveit-kinematics/src/set_from_ik.rs:79-93` (deviation 4) | Upstream's base `supportsGroup` is "this group is a chain and its tip is my tip", and `KDLKinematicsPlugin` fails it for a multi-tip group. | **REFUTED, corrected this round** | `kinematics_base.cpp:142-155` is the whole implementation: `if (!jmg->isChain()) { ...; return false; } return true;`. There is no tip comparison in it, and `rg supportsGroup moveit_kinematics/ moveit_core/kinematics_base/` returns only the base declaration (`kinematics_base.hpp:510`) and this definition — no plugin overrides it. The conclusion the port draws from the claim survives (a multi-tip group is not a chain, so `robot_state.cpp:1841-1851` does set `valid_solver = false` and `:1856-1860` does forward to `setFromIKSubgroups`), but by `isChain()` alone. | `e22900b` |
| `crates/moveit-kinematics/src/set_from_ik.rs:45-57` (deviation 1) | Upstream hands the raw `RobotState*` to the validity callback without applying the candidate, and every real callback applies it itself. | CONFIRMED | Both cited callbacks opened: `kinematics_service_capability.cpp:70-79` (`isIKSolutionValid`) opens with `state->setJointGroupPositions(jmg, ik_solution); state->update();` at `:75-76`, and `trajectory_functions.cpp:576-589` (`isStateColliding`) with the same pair at `:581-582`. The caller side is `ikCallbackFnAdapter` (`robot_state.cpp:1746-1763`), which permutes `ik_sol` through the bijection into `solution` and passes `&solution[0]` — never touching the state. | |
