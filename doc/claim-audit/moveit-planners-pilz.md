# Claim audit — moveit-planners-pilz

Type-b claim audit per PORTING-PLAN.md §175: verifying that "upstream does
X"-shaped claims in this crate's doc comments cite code that is actually
correct and actually on the live execution path, not just accurately
transcribed from a source that turns out not to say what the citation implies.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `src/trajectory_functions.rs:218-220` (pre-fix) | The two-`Vec` `computeLinkFK(robot_state, link_name, joint_names, joint_positions, pose)` overload is "a thin adapter that zips its two vectors into the same map this overload already takes, with no logic of its own" — i.e. it exists and forwards. | EXPIRED | `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_functions.hpp:93-95` declares the overload; `src/trajectory_functions.cpp` (full file, grepped for `computeLinkFK`) has no definition of it anywhere; every call site (`trajectory_generator_circ.cpp:127,164`, `trajectory_generator_lin.cpp:105,142`, `trajectory_generator_polyline.cpp:127`) passes the 4-arg `map<string,double>` overload, confirmed by `trajectory_generator.hpp:126-127`'s field types (`std::map<std::string, double> start_joint_position`/`goal_joint_position`). The overload is declared, never defined, never called — dead upstream code, not a forwarding adapter. | `73065cc` |
| `src/lib.rs` (`# Deliberately not ported` section) | The five ROS-facing files excluded from this crate (`move_group_sequence_*`, `planning_context_loader*`, `pilz_industrial_motion_planner.cpp`, `command_list_manager.*`, `plan_components_builder.*`) never compute a LIN/PTP/CIRC trajectory themselves — they only route requests to the analytical types this crate does port. | CONFIRMED | `pilz_industrial_motion_planner.cpp` (full 223-line file read): `CommandPlanner::initialize`/`getPlanningContext`/`canServiceRequest` do plugin loading, request validation and dispatch only, no trajectory math. `plan_components_builder.cpp:75-105`'s `PlanComponentsBuilder::blend()` delegates to `blender_->blend(...)` (the `TrajectoryBlenderTransitionWindow`, already flagged in this crate's own doc as "not yet in scope, planned for later rounds") rather than computing a blend itself. |  |
| `src/lib.rs` (pre-fix header) | `Ported from moveit_planners/pilz_industrial_motion_planner/` (bare directory) implicitly claims this crate's own retention obligation is scoped to "the package it ported", when the license gate actually resolves a bare directory citation to every file beneath it and exempts it from retention checking (`len(resolved) == 1` in `verify-upstream-license-provenance.sh`) — masking whether retention was ever really checked. | EXPIRED | Direct measurement this round: the directory resolves to 145 files carrying 6 distinct (year, holder) pairs, not the "130 files' worth of rights holders" §167.2 assumed when it created the exemption. Of those 6, only `2018, Pilz GmbH & Co. KG` belongs to a file this crate actually ports (the other 5 belong to files under the same directory this crate never touches: `move_group_sequence_*`, `command_list_manager.*`, etc.). Narrowed the citation to the 23 files each module's own header already cites; retention now runs for real (348 files checked, down from 471) and still reports zero findings. | `944b4a8` |

## Swept, no claim found needing verification

`src/velocity_profile.rs`, `src/path_circle.rs`, `src/trajectory_generator{,_circ,_lin,_ptp}.rs`, `src/limits.rs`,
`src/cartesian_trajectory.rs` — grepped for the same "upstream does X" /
adapter / dispatch shape (`PlannerManager|planner_manager|registerPlannerType|
createPlanningContext|planning_context|adapter`). No hits outside the three
rows above.

Note on row 11 above (`src/lib.rs`, `# Deliberately not ported`): its
parenthetical "(`TrajectoryBlenderTransitionWindow`, already flagged in
this crate's own doc as 'not yet in scope, planned for later rounds')" is
now stale — this crate ported `TrajectoryBlenderTransitionWindow` in
`b3e39c3` (this round), so `plan_components_builder.cpp:75-105`'s
`blender_->blend(...)` call now targets a type this crate does port. The
row's own CONFIRMED claim (these five files never compute a trajectory
themselves) still holds; only that one parenthetical is out of date. Not
re-marked EXPIRED since the claim itself is unaffected — flagged here so
a future pass does not cite the stale parenthetical as current.

## §172 two-anchor narrowing sweep — negative result on anchor 1, one anchor-2 hit classified `distinct`

Anchor 1 (upstream, run first per §172): `static_cast<int|size_t|unsigned|
long|short>`, C-style `(int)`/`(size_t)`/`(unsigned)`/`(long)` casts, an
`int`/`size_t`/`unsigned`/`long`/`short` declaration whose RHS is a float
literal or a division, and `floor`/`ceil`/`round`/`sqrt`/`pow` near an
integer declaration — swept via `rg` against every upstream file this
crate's `lib.rs` header and each module's own header cite (the 27-file set
listed in `lib.rs`'s crate-level citation, including this round's four new
files: `trajectory_blender.hpp`, `trajectory_blender_transition_window.{hpp,cpp}`,
`trajectory_blend_request.hpp`, `trajectory_blend_response.hpp`), each
file present and read at the path cited. All four sweep patterns: 0 hits
across the whole set.

Anchor 2 (port side): `as (i8|i16|i32|i64|isize|u8|u16|u32|u64|usize)` in
`crates/moveit-planners-pilz/src/`, enumerated on screen: exactly one hit.

| where (anchor) | claim | verdict | evidence |
|---|---|---|---|
| upstream set (27 files, anchor 1) | no int/size_t/unsigned/long/short decl or cast with a floating-point initializer, in any cited file | CONFIRMED (absent) | every file opened, `rg` swept, 0 hits |
| `src/velocity_profile.rs:423` (anchor 2) | `((x > 0.0) as i32 - (x < 0.0) as i32) as f64`, upstream's `sign(x) = (x > 0) - (x < 0)` | `distinct` — not a float→int narrowing site. `(x > 0.0)`/`(x < 0.0)` are `bool`, so `as i32` casts a boolean to exactly `0`/`1` (no fractional value, no UB/saturation ambiguity between C++ and Rust — that ambiguity only exists for `double`-to-integer casts). The final `as f64` casts an already-exact small `i32` (`-1`/`0`/`1`) to `f64`, which is always lossless. Neither cast receives a floating-point *value* being truncated. |

Expires (§153.1): if a future round adds an int/size_t/unsigned/long/short
declaration with a floating-point initializer to any file in the upstream
set, adds a new upstream file citing floating-point narrowing to this
crate's cited set, or adds an `as iNN/uNN/usize/isize` cast in
`crates/moveit-planners-pilz/src/` that receives a non-boolean `f64`
expression, this table's rows must be re-swept, not assumed to still hold.
