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
| `crates/moveit-planning/src/pipeline.rs:20` | `planner_map_`'s type (`unordered_map<string, PlannerManagerPtr>`) cited at `planning_pipeline.hpp:259` | EXPIRED (category a) | opened `planning_pipeline.hpp:259` — that's `planning_pipeline_parameters::Params pipeline_parameters_;`, an unrelated field. Real `planner_map_` declaration is `hpp:263`. (`pipeline.rs:175` separately and correctly cites `hpp:259` for `pipeline_parameters_` itself.) | |
| `crates/moveit-planning/src/pipeline.rs:303` | `JointConstraint` construction/field-setting cited at `planning_pipeline.cpp:68-70` | EXPIRED (category a) | opened `planning_pipeline.cpp:68-70` — a comment plus the `for`-loop header; actual construction (`new_joint_constraint`, `.joint_name =`, `.position =`) is at `cpp:71-73`. | |
| `crates/moveit-planning/src/response.rs:99` | quotes `"The full starting state used for planning"` cited at `planning_response.hpp:66` | EXPIRED (category a) | opened `planning_response.hpp` — that doc comment is on line 64; line 66 is `std::string planner_id;`, an unrelated field. | |
| `crates/moveit-planning/src/pipeline.rs:48` | trajectory-constraints feedforward happens "before ... the second and every later planner" (framed as position-based, i.e. never the first planner) | EXPIRED (category b) | opened `planning_pipeline.cpp:294-302` — the actual gate is `if (res.trajectory)`, state-based not position-based. `res` is a caller-owned, non-const, pre-populatable output param, so a pre-populated `res` would make the *first* planner also get the feedforward, contradicting the doc's "never before the first" framing. | |
| `crates/moveit-planning/src/pipeline.rs:199` | `getName` (among others) "read out of `pipeline_parameters_` or `planner_map_`" | EXPIRED (category b) | opened `planning_pipeline.hpp:222-226` — `getName()` returns `parameter_namespace_`, a separate member, not either cited container. | |
| `crates/moveit-planning/src/pipeline.rs:317` | `JointConstraint::configure` "only rejects a *negative* tolerance", cited `kinematic_constraint.cpp:146-151` | EXPIRED (category b) | opened `kinematic_constraint.cpp:243-260` (uncited) — the same function also silently substitutes `joint_tolerance_above_`/`joint_tolerance_below_` to `epsilon` when position+tolerance violates joint bounds, a second uncited tolerance-modifying path that contradicts the "only" claim. | |
| `crates/moveit-planning/src/response.rs:47-48` | pilz's `move_group_sequence_service.cpp:128` / `move_group_sequence_action.cpp:264` cited as fill sites for `planning_interface::MotionPlanResponse::planning_time` | EXPIRED (category b) | opened both — both actually set `.planning_time` on `moveit_msgs::msg::MotionSequenceResponse`, a same-named field on an unrelated message type (matched by field name, not type, during the original port). `trajectory_generator.cpp:267,277` cited in the same list is correct. | |
| `crates/moveit-planning/src/response_adapters/add_ruckig_traj_smoothing.rs:33` | routes through "the same ... convenience wrapper upstream's own `trajectory_tools.cpp:70-76` uses" | EXPIRED (category b) | opened `add_ruckig_traj_smoothing.cpp:81` — calls its own `smoother_.applySmoothing(...)` member directly, never the `trajectory_tools.cpp` free function. | |
| `crates/moveit-planning/src/pipeline.rs` (13 other citations, `planning_pipeline.cpp`/`.hpp`, `transforms.hpp:113`) | various | CONFIRMED (aggregate) | subagent-reported only, not individually re-opened by me | |
| `crates/moveit-planning/src/response.rs` (remaining citations: `kinematic_constraint/src/utils.cpp:623-675`, `ompl_interface/.../model_based_planning_context.cpp:799`, `chomp/.../chomp_planning_context.cpp:62`, `stomp/.../stomp_moveit_planning_context.cpp:277`, `moveit/robot_state/robot_state.hpp:1417`) | various | CONFIRMED (aggregate) | subagent-reported only, not individually re-opened by me | |
| `crates/moveit-planning/src/request_adapters/resolve_constraint_frames.rs` (citation `kinematic_constraint/src/utils.cpp:623-675`) | (fine, per subagent) | CONFIRMED | subagent-reported only, not individually re-opened by me | |

## Summary

- 17 citation sites total (3 category-a, 7 category-b, 7 confirmed-fine
  aggregate) + 1 velocity-bounds *behavioral* defect found via one of the
  category-b citations, which is the highest-priority item of the whole
  audit
- Fixed this round: the velocity-bounds gap (`5347903`)
- Remaining 10 EXPIRED citation findings (3a + 7b, excluding the already-
  fixed velocity-bounds one) are queued for remediation, one commit per
  finding, this same round

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
