# Claim audit — moveit-planning

Prose-citation audit against upstream MoveIt2 pinned at `e017c91e`. Scope:
every citation in `crates/moveit-planning/src/*.rs`. Same round/method as
`moveit-scene.md` — see that file for the full methodology note
(subagent first pass, spot-checked by me and independently by the
orchestrator; "fine" sites counted but not all individually itemized
below).

**Ownership correction:** an earlier round of this file's own report text
filed these findings as "not mine" — the crate-ownership map
(`doc/crate-ownership.md`) assigns `moveit-planning` to `p1-fixtures`.
That was wrong; recorded here so the mistake doesn't repeat.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-planning/src/request_adapters/check_start_state_bounds.rs:54-57` (pre-fix) | `satisfies_position_bounds` with `margin=0.0` cited as "upstream's own default", at `robot_state.hpp:1417` | EXPIRED, and the derived *behavior* (not just the citation) was wrong | opened `moveit/robot_state/include/moveit/robot_state/robot_state.hpp:1419`: `satisfiesBounds(jmodel, margin)` is `satisfiesPositionBounds(joint, margin) && (!has_velocity_ \|\| satisfiesVelocityBounds(joint, margin))`. The port called only the position half — a start state with out-of-bounds velocities was silently accepted here and rejected upstream. Line was 1417 (`satisfiesBounds(const JointModel*, margin)`, position-only overload) vs. the real combined-check overload at 1419; `grep`-ing the upstream `.cpp` alone does not find this, the check is inline in the header one call level down. | `5347903` |
| `crates/moveit-planning/src/pipeline.rs:20` | `planner_map_`'s type (`unordered_map<string, PlannerManagerPtr>`) cited at `planning_pipeline.hpp:259` | EXPIRED (category a) | opened `planning_pipeline.hpp:259` — that's `planning_pipeline_parameters::Params pipeline_parameters_;`, an unrelated field. Real `planner_map_` declaration is `hpp:263`. (`pipeline.rs:175` separately and correctly cites `hpp:259` for `pipeline_parameters_` itself.) | `0d03824` |
| `crates/moveit-planning/src/pipeline.rs:303` | `JointConstraint` construction/field-setting cited at `planning_pipeline.cpp:68-70` | EXPIRED (category a) | opened `planning_pipeline.cpp:68-70` — a comment plus the `for`-loop header; actual construction (`new_joint_constraint`, `.joint_name =`, `.position =`) is at `cpp:71-73`. | `2e17cc5` |
| `crates/moveit-planning/src/response.rs:99` | quotes `"The full starting state used for planning"` cited at `planning_response.hpp:66` | EXPIRED (category a) | opened `planning_response.hpp` — that doc comment is on line 64; line 66 is `std::string planner_id;`, an unrelated field. | `07da239` |
| `crates/moveit-planning/src/pipeline.rs:48` | trajectory-constraints feedforward happens "before ... the second and every later planner" (framed as position-based, i.e. never the first planner) | EXPIRED (category b) | opened `planning_pipeline.cpp:294-302` — the actual gate is `if (res.trajectory)`, state-based not position-based. `res` is a caller-owned, non-const, pre-populatable output param, so a pre-populated `res` would make the *first* planner also get the feedforward, contradicting the doc's "never before the first" framing. | `770deaa` |
| `crates/moveit-planning/src/pipeline.rs:199` | `getName` (among others) "read out of `pipeline_parameters_` or `planner_map_`" | EXPIRED (category b) | opened `planning_pipeline.hpp:222-226` — `getName()` returns `parameter_namespace_`, a separate member, not either cited container. | `8acd953` |
| `crates/moveit-planning/src/pipeline.rs:317` | `JointConstraint::configure` "only rejects a *negative* tolerance", cited `kinematic_constraint.cpp:146-151` | EXPIRED (category b) | opened `kinematic_constraint.cpp:243-260` (uncited) — the same function also silently substitutes `joint_tolerance_above_`/`joint_tolerance_below_` to `epsilon` when position+tolerance violates joint bounds, a second uncited tolerance-modifying path that contradicts the "only" claim. Checked `moveit-constraints::JointConstraint::new` (`crates/moveit-constraints/src/joint.rs:170-174`, read-only) — it already ports this second path correctly, so this was a documentation-only gap, not a behavioral one. | `a7542f2` |
| `crates/moveit-planning/src/response.rs:47-48` | pilz's `move_group_sequence_service.cpp:128` / `move_group_sequence_action.cpp:264` cited as fill sites for `planning_interface::MotionPlanResponse::planning_time` | EXPIRED (category b) | opened both — both actually set `.planning_time` on `moveit_msgs::msg::MotionSequenceResponse`, a same-named field on an unrelated message type (matched by field name, not type, during the original port). `trajectory_generator.cpp:267,277` cited in the same list is correct. | `6f8d0b6` |
| `crates/moveit-planning/src/response_adapters/add_ruckig_traj_smoothing.rs:33` | routes through "the same ... convenience wrapper upstream's own `trajectory_tools.cpp:70-76` uses" | EXPIRED (category b) | opened `add_ruckig_traj_smoothing.cpp:81` — calls its own `smoother_.applySmoothing(...)` member directly, never the `trajectory_tools.cpp` free function. | `8195829` |
| `crates/moveit-planning/src/pipeline.rs` (Semantic 1: planner-chain feedforward) | `planning_pipeline.cpp:295-302` (section), `:299` (`if (res.trajectory)` gate), `:301` (the overwrite statement), `getTrajectoryConstraints` cited `:57-73` | EXPIRED (wrong-anchor-right-claim), fixed this round | `:295-302`/`:299`/`:301` all pinpoint-exact against the real function body. `getTrajectoryConstraints`'s true span is `:57-79`, not `:57-73` — the citation stopped at the per-joint field assignments, excluding the outer waypoint loop's close, the `push_back` into `trajectory_constraints`, and the final `return` statement. Recurs at 3 sites (module doc, `Feedforward` error variant doc, the private helper's own doc). | `f2f6301` |
| `crates/moveit-planning/src/pipeline.rs` (Semantic 2: response-adapter chain gate) | `planning_pipeline.cpp:332-351` (section), `:333` (`if (res.error_code)`) | CONFIRMED | both pinpoint-exact — `:332` is the section's leading comment, `:351` its closing brace; `:333` is literally `if (res.error_code)`. | |
| `crates/moveit-planning/src/pipeline.rs` (Semantic 3: `active_` flag, six manual resets) | `planning_pipeline.hpp:217-220` (`isActive`), `cpp:259` (entry set), `:289`/`:313`/`:327`/`:347`/`:357`/`:372` (the six resets) | CONFIRMED | opened all eight — every one pinpoint-exact (`:259` is `active_ = true;`; each of the six reset lines is exactly `active_ = false;` on its named exit path: request-adapter failure, context-creation failure, planner failure, response-adapter failure, the `catch` block, and the final line). | |
| `crates/moveit-planning/src/pipeline.rs` (Semantic 4: `planner_id` fallback) | `cpp:361-369` (section), `:364-367` (the `RCLCPP_WARN` call) | CONFIRMED | both exact — `:361` is the leading comment, `:369` the closing brace; `:364-367` is the full multi-line `RCLCPP_WARN(...)` call, closing at `:367`. | |
| `crates/moveit-planning/src/pipeline.rs` (Semantic 5: final result reflects last state) | `cpp:372-373` | CONFIRMED | exact — `:372` is `active_ = false;`, `:373` is `return static_cast<bool>(res);`. | |
| `crates/moveit-planning/src/pipeline.rs` (D1 exclusions: `publishPipelineState`/`publish_received_requests`/`node_`/`pipeline_parameters_`/`terminate`/deprecated block) | `hpp:250`, `hpp:187`, `hpp:257`, `hpp:259`, `hpp:190`+`cpp:376-385`, `hpp:134-173` | CONFIRMED | opened all six — every anchor lands on the exact declaration named (`publishPipelineState`'s signature, `publish_received_requests`'s parameter, the `node_` field, the `pipeline_parameters_` field, `terminate()`'s declaration and its exact `:376-385` body, the deprecated block's exact `[[deprecated(...)]]` span). `hpp:134-173`'s true closing brace is `:174`, 1 line past the citation, but every named function in the block is fully inside `:134-173`. | |
| `crates/moveit-planning/src/pipeline.rs` (D1-deferred: `getPlannerPluginNames`/`getRequestAdapterPluginNames`/`getResponseAdapterPluginNames`/`getPlannerManager`) | `hpp:193-208` (three `get*PluginNames`), `hpp:229-236` (`getPlannerManager`) | CONFIRMED | `hpp:193-208` is pinpoint-exact (covers exactly the three named methods, start to close). `hpp:229-236`'s true closing brace is `:237`, 1 line past the citation, but the described body (`if (planner_map_.find(...) == end()) { ...; return nullptr; } return planner_map_.at(...);`) is fully inside `:229-236`. | |
| `crates/moveit-planning/src/pipeline.rs` (`Planner` trait replaces `getPlanningContext`+`solve`) | `cpp:304-320` (target), `:308-315` (nullptr-context failure), `:323-328` (post-`solve` failure) | CONFIRMED | `:304-320` and `:308-315` are pinpoint-exact (the latter is literally the `if (!context) { ... }` block's opening through closing brace). `:323-328`'s true closing brace is `:329`, 1 line past the citation, but the described `if (!res.error_code) { ...; return false; }` body is fully inside `:323-328`. | |
| `crates/moveit-planning/src/pipeline.rs` (`trajectory_constraints_for`'s weight substitution) | `kinematic_constraint.cpp:263-270` | CONFIRMED | exact — `if (jc.weight <= std::numeric_limits<double>::epsilon()) { ...; constraint_weight_ = 1.0; } else constraint_weight_ = jc.weight;` spans exactly `:263-270`. | |
| `crates/moveit-planning/src/pipeline.rs` (`generate_plan` replaces `PlanningPipeline::generatePlan`, module-doc + `generate_plan`'s own doc) | `planning_pipeline.cpp:251-374` (cited twice) | CONFIRMED | exact — `generatePlan`'s definition spans precisely `:251-374`, both occurrences. | |
| `crates/moveit-planning/src/response.rs` (D8 delta audit: `MotionPlanResponse`'s 6 members) | `planning_response.hpp:48-70` (struct), `ompl_interface/.../model_based_planning_context.cpp:799`, `chomp/.../chomp_planning_context.cpp:62`, `stomp/.../stomp_moveit_planning_context.cpp:277`, `pilz_industrial_motion_planner/.../trajectory_generator.cpp:267,277`, `moveit_py/.../planning_response.cpp:52,92` | CONFIRMED | opened all six — every `.planning_time =`/`.start_state` fill site is pinpoint-exact (including both pilz sites, `setSuccessResponse`/`setFailureResponse`); `moveit_py:92`'s `def_property(..., nullptr, ...)` confirms the "read-only" claim exactly. `hpp:48-70`'s true struct close is `:73`, 3 lines past the citation, but every one of the 5 data members plus the start of `operator bool` (the claim's "6th member") is inside `:48-70`. | |
| `crates/moveit-planning/src/response.rs` (table-row correction, not a code defect) | this file's prior aggregate row (line 27, now removed) listed `kinematic_constraint/src/utils.cpp:623-675` and `moveit/robot_state/robot_state.hpp:1417` as citations belonging to `response.rs` | table error, no `response.rs` fix needed | `rg -n '623\|675\|1417\|utils' crates/moveit-planning/src/response.rs` — zero hits. Neither citation exists in this file. `utils.cpp:623-675` is actually `resolve_constraint_frames.rs`'s own citation (see its row below); `robot_state.hpp:1417` is `check_start_state_bounds.rs`'s pre-fix citation from row 1 above (already corrected to `:1419` there, commit `5347903`). The prior aggregate row conflated citations from three different files under one `response.rs` label. | |
| `crates/moveit-planning/src/request_adapters/resolve_constraint_frames.rs` | `kinematic_constraint/src/utils.cpp:623-675` (`resolveConstraintFrames`), adapter's own `cpp:41-83` (symbol audit), `:65` (`RCLCPP_DEBUG`), `:73` (`SUCCESS` return), `:66-73` (return value discarded) | CONFIRMED | opened all five — `cpp:41-83` and `:65`/`:73`/`:66-73` are pinpoint-exact against the adapter file. `utils.cpp:623-675`'s true closing brace is `:676`, 1 line past the citation, but both constraint loops (position, orientation) and every described rewrite (`link_name`, `target_point_offset`/`orientation` folding) are fully inside `:623-675`. | |

## Summary

- 16 citation sites total (3 category-a, 6 category-b, 7 confirmed-fine
  aggregate) + 1 velocity-bounds *behavioral* defect found via one of the
  original category-b citations, which was the highest-priority item of
  the whole audit
- All 17 EXPIRED/behavioral findings fixed in Round 11, one commit each:
  `5347903` (velocity-bounds behavioral fix), `0d03824`, `2e17cc5`,
  `07da239` (category-a), `770deaa`, `8acd953`, `a7542f2`, `6f8d0b6`,
  `8195829` (category-b)
- Round `33519d7` (this round): the 3 aggregate rows above ("13 other
  citations" in `pipeline.rs`, the 5-citation `response.rs` bucket, and
  `resolve_constraint_frames.rs`'s single un-reopened citation) are
  replaced by 13 individually-opened rows covering every distinct
  citation location in this crate. Found 1 new EXPIRED citation
  (`getTrajectoryConstraints`, `:57-73` → `:57-79`, recurring at 3 sites,
  fixed as `f2f6301`) and 1 table-only error (the `response.rs` bucket
  row had attributed 2 citations from two other files to `response.rs` —
  corrected, no code change needed). Every other citation opened this
  round is CONFIRMED, several with the same "range ends a line or two
  short of the true closing brace but excludes no described content"
  pattern already established in `moveit-scene.md`.
- Combined EXPIRED rate: 18 of ~29 distinct citation locations (62%) —
  the original 16-citation-sites count undercounted the crate's real
  citation surface once the buckets were opened individually, same
  effect `moveit-scene.md` found.

## §172 narrowing sweep

Run this round, both directions, corrected ordering (upstream-first).

- Upstream-first: swept every upstream file this crate provenances that
  carries real ported numeric logic — `planning_response_adapter_plugins/src/{add_ruckig_traj_smoothing,add_time_optimal_parameterization,display_motion_path,validate_path}.cpp`,
  `planning_request_adapter_plugins/src/{check_for_stacked_constraints,check_start_state_bounds,check_start_state_collision,resolve_constraint_frames,validate_workspace_bounds}.cpp`,
  `planning_pipeline/src/planning_pipeline.cpp`,
  `planning_pipeline/include/.../planning_pipeline.hpp`,
  `planning_interface/include/.../planning_response.hpp` — for
  `int`/`unsigned`/`size_t`/`std::size_t`/`long`/`uint32_t`/`int32_t`
  declarations or `static_cast<...>` narrowing a floating-point
  initializer. 1 hit: `validate_path.cpp:93`,
  `std::size_t state_count = res.trajectory->getWayPointCount();` — a
  waypoint count, a true integer quantity, `distinct`. **0 real
  narrowing sites.**
  Excluded from the sweep, one-line doc-citation-only sites (cited for a
  single field/message claim, not files whose numeric logic this crate
  ports): `kinematic_constraint/src/utils.cpp`,
  `ompl_interface/src/model_based_planning_context.cpp:799`,
  `chomp/chomp_interface/src/chomp_planning_context.cpp:62`,
  `stomp/src/stomp_moveit_planning_context.cpp:277`,
  `move_group_sequence_service.cpp:128`,
  `move_group_sequence_action.cpp:264`,
  `trajectory_generator.cpp:267,277`, `trajectory_tools.cpp:70-76`.
- Port-side: `rg '\bas\s+(i8|i16|i32|i64|i128|isize|u8|u16|u32|u64|u128|usize)\b'`
  across `crates/moveit-planning/src` — **0 hits**.
- Both directions swept, both zero, no fix needed.

## Addendum: `add_time_optimal_parameterization.rs:91`'s `resample_dt`, checked and not a moveit-planning defect

The orchestrator flagged `resample_dt: f64` as "stored unvalidated" in
`AddTimeOptimalParameterization::new` and passed through to
`apply_totg_time_parameterization`, citing a guard supposedly at
`moveit-trajectory`'s `time_optimal_trajectory_generation.rs:737`.
Checked directly, not accepted on citation: this worktree's branch point
is `d8608ae`, and `git log --oneline HEAD..main -- crates/moveit-trajectory`
shows `main` is ahead by several commits including `01cc605 fix(trajectory):
reject invalid resample_dt instead of hanging or truncating` and
`92625bb moveit-trajectory: close resample_dt boundary coverage, verify
totg_compute_time_stamps parity` — p6-totg's fix for exactly this defect
already landed on `main`, just not yet merged into this worktree. Line
`:737` in this worktree's stale copy is inside a test helper
(`fixture_path`), not a guard — the guard exists on `main`, not here.

Given that, the question actually in `moveit-planning`'s scope is
narrower: does `AddTimeOptimalParameterization::new` itself need to
validate eagerly, or is deferring to `.adapt()`'s `Result` enough? Checked
`crates/moveit-planning/src/error.rs:62`:
`#[error("{adapter}: failed to compute a trajectory: {source}")]` — the
top-level message already names `"AddTimeOptimalParameterization"`, the
adapter the caller actually called; the underlying `moveit_trajectory`
error is `source`, not the whole message. So "a caller gets the error
from a crate it did not call" does not hold once `main`'s fix is merged
in: the error is correctly labeled at the point of the actual call.
Every other constructor in this crate (`ValidateWorkspaceBounds::new`,
`CheckStartStateBounds::new`) is also an infallible builder that defers
validation to `.adapt()`'s `Result`, so eagerly validating only this one
constructor would be inconsistent with the crate's own established
convention, not a fix for a live defect. No `moveit-planning` change
made; recorded here instead of silently dropped.

## `planning_pipeline_interfaces` (Round 12): undocumented gap, per-file verdict

`moveit_ros/planning/planning_pipeline_interfaces/src/` (4 files) was
neither ported nor excluded anywhere in `lib.rs` before this round —
`rg -i 'PlanResponsesContainer|solution_selection|stopping_criterion|parallel'`
over `crates/moveit-planning/src/` found one hit, an unrelated comment.
Each of the 4 files opened and classified individually, not collapsed:

| file | verdict | evidence |
|---|---|---|
| `plan_responses_container.cpp` | **gap, ported** | `PlanResponsesContainer`: `std::mutex`-guarded `std::vector<MotionPlanResponse>`, `pushBack`/`getSolutions`. No ROS type, no `rclcpp`. Ported as [`crate::plan_responses::PlanResponsesContainer`], commit `0fca5a6`. |
| `solution_selection_functions.cpp` | **gap, ported** | One function, `getShortestSolution` — not two ("shortest-path/shortest-duration") as the round's opening hypothesis guessed; the header (`solution_selection_functions.hpp`) declares only this one. No ROS type. Ported as [`crate::plan_responses::shortest_solution`], commit `0fca5a6`. |
| `stopping_criterion_function.cpp` | **gap, ported** | One function, `stopAtFirstSolution`. No ROS type, and unlike the other three files, no `RCLCPP_*` macro at all. Ported as [`crate::plan_responses::stop_at_first_solution`], commit `0fca5a6`. |
| `planning_pipeline_interfaces.cpp` | **mixed — 4 functions, 4 individual verdicts, not one** | see below |

`planning_pipeline_interfaces.cpp` itself has 4 functions, each opened
separately:

- `getLogger()` — D1 exclude: `rclcpp::Logger`, pure logging, matches this
  crate's existing convention for every other `RCLCPP_*` site
  (`add_ruckig_traj_smoothing.rs`'s module doc, "Symbol audit").
- `createPlanningPipelineMap(pipeline_names, robot_model, node, ...)` — D1
  exclude: takes `const rclcpp::Node::SharedPtr& node` directly, to load
  pipeline parameters from the ROS parameter server. This is the actual
  Node-coupled function in this file.
- `planWithSinglePipeline(request, scene, planning_pipelines)` — **not a
  gap**: looks `request.pipeline_id` up in `planning_pipelines` (a
  `std::unordered_map<string, PlanningPipelinePtr>`), calls
  `pipeline->generatePlan`, and defensively normalizes an unset error code
  to `FAILURE`. `crate::request`'s own doc (`:56-59`) already excludes
  `pipeline_id` for the identical reason — "selects *among* pipelines, a
  caller/orchestration concern, and this workspace has exactly one
  pipeline" — and the error-code-normalization half of this function's job
  is structurally superseded by [`crate::pipeline::generate_plan`]'s own
  `Result` typing (every failure path already carries an error; there is
  no "returned `false`, error code left at `SUCCESS`" case a `Result` can
  represent). So every piece of `planWithSinglePipeline` is either already
  deferred (name lookup) or already generalized (error normalization), not
  a fresh port target. **Expires** the moment this crate (not a downstream
  registry crate) gains its own concrete, name-keyed pipeline/planner map —
  at that point this function's name-lookup half becomes a real gap.
- `planWithParallelPipelines(requests, scene, planning_pipelines, stop_cb, select_fn)`
  — **not a gap this round, but for a narrower reason than the round's
  opening hypothesis**: the brief that opened this round attributed the
  `rclcpp::Node` coupling to this function; checked directly, it does not
  take one — `createPlanningPipelineMap` does (see above), and this is the
  correction the brief invited if the reading were wrong. What
  `planWithParallelPipelines` actually depends on is the same
  `unordered_map<string, PlanningPipelinePtr>` `planWithSinglePipeline`
  does (it calls that function once per spawned thread) — the same
  deferred-registry reason, not ROS. Its own substance beyond that
  lookup — spawn one thread per request, push each outcome into a
  [`PlanResponsesContainer`], poll `stopping_criterion_callback` after each
  push and terminate the remaining pipelines if it fires, then apply
  `solution_selection_function` (or return everything) — has no ROS
  coupling in it either, and nothing has ported it this round: doing so
  needs `Box<dyn Planner<'m>>` (or an equivalent) to be usable across
  `std::thread::scope` threads, which was out of scope to design and prove
  in the same round as the 3 pure files above. **Expires** the moment (a)
  this crate gains a concrete pipeline/planner map (same trigger as
  `planWithSinglePipeline`) or (b) a caller needs concurrent multi-pipeline
  planning against the existing `&[Box<dyn Planner<'m>>]` shape — at that
  point [`crate::plan_responses::PlanResponsesContainer`]/
  [`crate::plan_responses::shortest_solution`]/
  [`crate::plan_responses::stop_at_first_solution`] ported this round are
  the pieces such an implementation would compose, already oracle-testable
  in isolation.

### Tests

[`crate::plan_responses`]'s own `#[cfg(test)]` module pins the tie-break
`shortest_solution` cannot get wrong silently: empty input, all-success
(shorter wins regardless of position), any success beats any failure,
exact-tie-keeps-first, and all-failure-keeps-first — the last two are the
boundary `std::min_element`'s strict-improvement-only replacement produces
and a narrative "one success, one failure" test alone would not catch.
`stop_at_first_solution` is covered for empty/all-failure/mixed-with-one-success.
