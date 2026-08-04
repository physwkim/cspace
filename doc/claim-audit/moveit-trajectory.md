# Claim audit — `moveit-trajectory`

Per `PORTING-PLAN.md` §175: one row per claim about upstream behaviour
found in this crate, verified against the actual upstream source (not
inferred from the port), appended as found rather than batched at
report time.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `src/trajectory.rs`, `Trajectory::integrate_forward` doc | "upstream's own two call sites do [pass `output.trajectory_` for both]" | EXPIRED | `moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp:370` is the *only* call site of `integrateForward` (confirmed via `rg -rn integrateForward moveit_core/trajectory_processing/`: one declaration at `.hpp:166`, one call at `.cpp:370`, one definition at `.cpp:564` — zero other calls) | `705ea76` |
| `src/time_optimal_trajectory_generation.rs`, `totg_compute_time_stamps_silently_collapses_duplicate_waypoints_matching_upstream` doc | for two identical input waypoints, upstream's `totgComputeTimeStamps` degenerates to a 1-waypoint `Ok`/`true` result via the *same* `points.size() == 1` early return firing in both of its internal `computeTimeStamps` calls — not via `new_resample_dt` (`0.0` in this case) ever being read as a divisor | CONFIRMED | `time_optimal_trajectory_generation.cpp:1137-1160` (`totgComputeTimeStamps`, two sequential `computeTimeStamps` calls against the same mutated `trajectory`) and `:1219-1226` (`points.size() == 1` early return, `return true` before `sample_count` is computed) — read directly. Independently reproduced against this crate's own `totg_compute_time_stamps` (temporary `#[test]` probe, output `Ok(())`/`way_point_count == 1`, not committed) before writing the permanent regression test | `92625bb` |
| `src/planner.rs` (`moveit-planners-chomp`, listed under that crate's own file below) | — | — | — | — |

## Background-agent audit — lost to compaction, not re-verifiable as given

An earlier background agent in this session (Item 4 of the governing
task: "audit upstream claim used as basis, type-b vs type-a") reported
**3 claimed TYPE_B findings and 7 claimed TYPE_A findings out of 60
candidates examined** across this crate, `moveit-planners-chomp`, and
`moveit-smoothing`. The itemized list (file:line, claim text, proposed
verdict) was in conversation context only and did not survive a
subsequent context compaction — exactly the failure mode §175
describes. No item from that list is recorded here as `CONFIRMED` or
`EXPIRED`; treating the surviving aggregate counts ("3 TYPE_B, 7
TYPE_A") as evidence without the underlying file:line/claim/evidence
triple would be citing a conclusion with no reproducible basis.

**Verdict: UNVERIFIABLE(itemized list lost to context compaction before
independent verification; only the two rows above were independently
re-derived and verified this session).** Re-running that background
audit and appending each item here *as found* (not batched) is the
correct way to close this gap in a future round.
