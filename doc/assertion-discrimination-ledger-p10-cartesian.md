# Assertion-discrimination ledger — p10-cartesian (`moveit-kinematics::cartesian_interpolator`)

The eight sites the sweep's scanner emits for the Cartesian interpolator
port. Seven are `via:new` propagations of a single helper body —
`Percentage::new`'s
`assert!((0.0..=1.0).contains(&value), "Percentage values must be between
0 and 1, inclusive, got {value}")` at
`crates/moveit-kinematics/src/cartesian_interpolator.rs:408`, which is
itself tagged `helper_body` and so sits outside the corpus — and one is a
new test assertion.

## Method

Every row's evidence is a mutation applied to this worktree, run with
`cargo nextest run -p moveit-kinematics` (48 tests, 48 passing at
baseline), and reverted, with the revert confirmed by an empty
`git diff` on the mutated file. No row here is justified by reading the
code alone, and no row is justified by a bite run in some earlier round.

## 1. `via:new` rows that never reach `Percentage::new` (3)

The scanner propagates a helper body to call sites by matching the helper's
*name*, so `new(` also matches a `fn new(...)` definition line and any
other type's `new()`. These three rows are that false positive. The
precedent for recording rather than laundering one is p1-fixtures'
`tree.rs:1781` (`out_of_bounds_coordinate_has_no_occupancy_even_when_the_tree_center_is_mapped`) row.

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-kinematics/src/cartesian_interpolator.rs:264` | scanner row `via:new:src:new(translation: f64, rotation: f64)` — the *signature* of `MaxEefStep::new`, not a call of anything | — | not-this-family | The whole body is `Self { translation, rotation }`: no assertion, no call. Nothing to bite, because no mutation of an assertion is possible at a line that contains none. |
| `crates/moveit-kinematics/src/cartesian_interpolator.rs:432` | scanner row `via:new:src:new(group_name: &'a str, link_name: &'a str, max_step: MaxEefStep)` — the signature of `CartesianInterpolator::new` | — | not-this-family | Body is a single struct literal filling the six fields; no assertion, no call. Same reason as `:297`. |
| `crates/moveit-kinematics/src/cartesian_interpolator.rs:625` | scanner row `via:new:src:new()` — `Vec::new()` in `through_waypoints` | — | not-this-family | `alloc::vec::Vec::new` has no assertion. Positively distinguished from the real site in the same function: bite N2 below moves `through_waypoints`' own `Percentage::new` operand out of range and the resulting panic reads `got 1.5375`, a value `Vec::new()` has no way to produce. |

## 2. Real `Percentage::new` call sites (4)

Each of these four does reach the range guard, so the question is whether a
failure at the guard is attributable to one of them rather than collapsed
across all four. Each was bitten by moving *its own* operand out of range;
the panic prints the operand, so each bite names its own site.

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-kinematics/src/cartesian_interpolator.rs:553` | `to_pose`'s return value, operand `run.achieved` | every test that runs a Cartesian path (11 of 13) | single-branch | Bite N1 (`Percentage::new(run.achieved + 1.0)`): panic `got 1.1219512195121952`, i.e. this file's `5/41 + 1`, and 11 tests fail. `achieved` has exactly one write site — `PathRun::validate_and_improve_interval`'s accept leaf — fed by `percentage = i / steps` with `1 <= i <= steps` and the bisection's `percentage - half_width`, both strictly inside `(0, 1]`, so no second cause can reach the guard from here. |
| `crates/moveit-kinematics/src/cartesian_interpolator.rs:649` | `through_waypoints`' return value, operand `solved` | `through_waypoints_accumulates_per_segment_and_drops_the_seam` | single-branch | Bite N2 (`Percentage::new(solved + 1.0)`): panic `got 1.5375`, which is this fixture's `0.5 + (3/40)/2` plus one, and exactly one test fails. The other twelve stay green, so the site is reached from this entry point only. |
| `crates/moveit-kinematics/src/cartesian_interpolator.rs:854` | `check_joint_space_jump`'s jump branch, operand `index as f64 / waypoints.len() as f64` | the 5 tests whose path contains a jump | single-branch | Bite N3 (`Percentage::new(solved + 1.0)`): panic `got 1.3333333333333333` = `1/3 + 1`, the prismatic fixture's fraction. Fails `a_relative_threshold_truncates_at_the_branch_flip`, `an_absolute_threshold_truncates_at_the_revolute_branch_flip`, `the_absolute_rule_measures_prismatic_joints_against_its_prismatic_bound`, and the two rule-comparison tests. |
| `crates/moveit-kinematics/src/cartesian_interpolator.rs:856` | the same function's no-jump branch, literal operand `1.0` | the 4 tests whose path contains no jump | single-branch | Bite N4 (`Percentage::new(1.5)`): panic `got 1.5`. Fails `a_disabled_threshold_leaves_the_branch_flip_in_place`, `an_empty_path_is_not_measured_for_jumps`, and the two rule-comparison tests. |

N3 and N4 are each other's isolating mutation for the two arms of
`check_joint_space_jump`. Three tests fail under N3 only, two under N4
only, and the two that fail under both are the rule-comparison tests, which
deliberately call the function once per arm — seven distinct tests, which
is every jump-detection test in the file, partitioned by which arm they
take.

## 3. New test assertion (1)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-kinematics/tests/cartesian_interpolator.rs:409` | `assert!(solution.is_none(), ...)` — one max step past the reported stop point has no IK solution, probed from the last waypoint's own configuration and from the start's | `unreachable_path_stops_at_its_last_reachable_waypoint` | discriminating | Bite O1: probe at `fraction + 0.0 / steps` (the stop point itself) instead of `fraction + 1.0 / steps`. That one test FAILS, printing the solution it found (`Some([-1.2245864248389362e-12, 0.6270198827775235, ...])`), and the other twelve stay green — so the `is_none()` is a statement about this pose and flips when the pose does, rather than a solver that never converges. Two further probes place the edge: `+ 0.25 / steps` and `+ 0.5 / steps` both still pass, so the port stops within a quarter of a max step of the true reachability limit, and the assertion's own probe distance is well clear of it. |

## Gate scope

`-p moveit-kinematics` for every bite and for the baseline above. The
full-workspace `--workspace` variants are owed before `git push` under the
standing rule and were not run for this ledger.

## UNFIXED

None. All eight sites carry a verdict backed by a mutation run this round,
and the orphan set returns to zero without touching
`doc/assertion-discrimination-orphans.txt`.
