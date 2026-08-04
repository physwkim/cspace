# Claim audit — `moveit-planners-chomp`

Per `PORTING-PLAN.md` §175: one row per claim about upstream behaviour
found in this crate, verified against the actual upstream source (not
inferred from the port), appended as found rather than batched at
report time.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `src/planner.rs`, `solve`'s recovery loop / `validate_recovery_time_limit` doc | upstream implicitly (no `static_cast`) narrows `planning_time_limit_ + 5` (a `double`) into `setRecoveryParams`'s `int planning_time_limit` parameter — undefined behaviour in C++ outside `int`'s range, silently saturating/overflowing under a naive Rust `as i32` transcription | CONFIRMED | `moveit_planners/chomp/chomp_motion_planner/src/chomp_planner.cpp:177-181` (the call site, no cast) and `moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_parameters.hpp:58` (`setRecoveryParams`'s declared `int planning_time_limit` parameter) — both read directly | `2d54e7d` |
| `src/trajectory.rs`, `fill_in_from_trajectory`'s `prev_idx`/`fraction` narrowing | this Rust site's unguarded `f64 -> usize` truncation matches upstream's own equally unguarded `static_cast<double>(...) ; std::trunc(...)` narrowing in `fillInFromTrajectory` — `distinct` from the `§172` `resample_dt`/`discretization` defect family, not the same defect | CONFIRMED (as `distinct`) | `moveit_planners/chomp/chomp_motion_planner/src/chomp_trajectory.cpp:185-213` — read directly | (no fix; classification only) |

## Background-agent audit — see `moveit-trajectory.md`

The Item-4 background agent's aggregate "3 TYPE_B / 7 TYPE_A out of 60
candidates" count spanned this crate too. Same disposition as recorded
there: **UNVERIFIABLE(itemized list lost to context compaction)** —
none of those specific claims are re-derivable from this session's
surviving context, so none are recorded here as confirmed or expired.
