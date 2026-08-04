# Claim audit — moveit-planners-pilz

Type-b claim audit per PORTING-PLAN.md §175: verifying that "upstream does
X"-shaped claims in this crate's doc comments cite code that is actually
correct and actually on the live execution path, not just accurately
transcribed from a source that turns out not to say what the citation implies.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `src/trajectory_functions.rs:217-220` (pre-fix) | The two-`Vec` `computeLinkFK(robot_state, link_name, joint_names, joint_positions, pose)` overload is "a thin adapter that zips its two vectors into the same map this overload already takes, with no logic of its own" — i.e. it exists and forwards. | EXPIRED | `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_functions.hpp:93-95` declares the overload; `src/trajectory_functions.cpp` (full file, grepped for `computeLinkFK`) has no definition of it anywhere; every call site (`trajectory_generator_circ.cpp:127,164`, `trajectory_generator_lin.cpp:105,142`, `trajectory_generator_polyline.cpp:127`) passes the 4-arg `map<string,double>` overload, confirmed by `trajectory_generator.hpp:126-127`'s field types (`std::map<std::string, double> start_joint_position`/`goal_joint_position`). The overload is declared, never defined, never called — dead upstream code, not a forwarding adapter. | `73065cc` |
| `src/lib.rs` (`# Deliberately not ported` section) | The five ROS-facing files excluded from this crate (`move_group_sequence_*`, `planning_context_loader*`, `pilz_industrial_motion_planner.cpp`, `command_list_manager.*`, `plan_components_builder.*`) never compute a LIN/PTP/CIRC trajectory themselves — they only route requests to the analytical types this crate does port. | CONFIRMED | `pilz_industrial_motion_planner.cpp` (full 223-line file read): `CommandPlanner::initialize`/`getPlanningContext`/`canServiceRequest` do plugin loading, request validation and dispatch only, no trajectory math. `plan_components_builder.cpp:75-105`'s `PlanComponentsBuilder::blend()` delegates to `blender_->blend(...)` (the `TrajectoryBlenderTransitionWindow`, already flagged in this crate's own doc as "not yet in scope, planned for later rounds") rather than computing a blend itself. |  |
| `src/lib.rs` (pre-fix header) | `Ported from moveit_planners/pilz_industrial_motion_planner/` (bare directory) implicitly claims this crate's own retention obligation is scoped to "the package it ported", when the license gate actually resolves a bare directory citation to every file beneath it and exempts it from retention checking (`len(resolved) == 1` in `verify-upstream-license-provenance.sh`) — masking whether retention was ever really checked. | EXPIRED | Direct measurement this round: the directory resolves to 145 files carrying 6 distinct (year, holder) pairs, not the "130 files' worth of rights holders" §167.2 assumed when it created the exemption. Of those 6, only `2018, Pilz GmbH & Co. KG` belongs to a file this crate actually ports (the other 5 belong to files under the same directory this crate never touches: `move_group_sequence_*`, `command_list_manager.*`, etc.). Narrowed the citation to the 23 files each module's own header already cites; retention now runs for real (348 files checked, down from 471) and still reports zero findings. | `944b4a8` |

## Swept, no claim found needing verification

`src/velocity_profile.rs`, `src/path_circle.rs`, `src/trajectory_generator{,_circ,_lin,_ptp}.rs`, `src/limits.rs`,
`src/cartesian_trajectory.rs` — grepped for the same "upstream does X" /
adapter / dispatch shape (`PlannerManager|planner_manager|registerPlannerType|
createPlanningContext|planning_context|adapter`). No hits outside the three
rows above.
