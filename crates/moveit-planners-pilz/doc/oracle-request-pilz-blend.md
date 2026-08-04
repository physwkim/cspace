# Oracle request: `TrajectoryBlenderTransitionWindow::blend()` parity check

Not an implementation — this is a request document for the human
orchestrator (`tools/moveit-oracle/` is not this worker's file). Filed per
PORTING-PLAN.md §185, closing the gap that round's port
(`crates/moveit-planners-pilz/src/trajectory_blender_transition_window.rs`,
commit `b3e39c3`) left open: LIN/PTP/CIRC are each measured to `1e-6` by
the existing `pilz_trajectory` op; the 966-line blender that landed this
round has no equivalent external measurement, only nine — actually twelve,
see "Overlap with this port's existing tests" below — self-consistency
tests.

## The gap

`blend()`'s job is to replace the shared stop between two already-planned
trajectories with a smoothed Cartesian transition. Every number it
produces (which waypoints survive from each side, the shape of the
transition itself) depends on three pieces of geometry this port
transcribed from upstream but has never checked against upstream's own
output: `searchIntersectionPoints`'s two indices, `determineTrajectoryAlignment`'s
branch choice, and `blendTrajectoryCartesian`'s quintic-smoothstep sampling.
A bug in any of the three would not necessarily show up as a crash or a
rejected request — it would show up as a blend that runs to completion and
returns a trajectory, just the wrong one. That failure mode already
happened once this round during the port itself (see "Why the alignment
branch needs its own field" below) and was only caught by a hand-written
assertion, not by any oracle comparison.

## Upstream symbol: public, already known

Confirmed directly against the four headers this port's module doc already
cites (re-read this round, not assumed carried over):

```cpp
// trajectory_blend_request.hpp
struct TrajectoryBlendRequest
{
  std::string group_name;
  std::string link_name;
  robot_trajectory::RobotTrajectoryPtr first_trajectory;
  robot_trajectory::RobotTrajectoryPtr second_trajectory;
  double blend_radius;
};

// trajectory_blender_transition_window.hpp
class TrajectoryBlenderTransitionWindow : public TrajectoryBlender {
public:
  bool blend(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const TrajectoryBlendRequest& req,
             TrajectoryBlendResponse& res) override;
  ...
};
```

`blend()` is `public`, callable given a `PlanningSceneConstPtr`, a
constructed `TrajectoryBlendRequest`, and a `LimitsContainer` (passed to
`TrajectoryBlenderTransitionWindow`'s constructor, not `blend()` itself —
matches this port's `blend(ctx, planner_limits, req)` signature, which
threads `planner_limits` as a parameter instead of storing it on a
constructed blender object; see the module's own `# TrajectoryBlender, the
abstract base class, is not ported` doc section for why there is no
blender *object* on this port's side to hold it).

## What `TrajectoryBlendRequest` actually carries — and what it does not

Read in full this round (`trajectory_blend_request.hpp`, quoted above):
`first_trajectory`/`second_trajectory` are bare `RobotTrajectoryPtr`s.
Nothing in the struct, and nothing `blend()`/`validateRequest`/
`searchIntersectionPoints`/`determineTrajectoryAlignment`/
`blendTrajectoryCartesian` ever reads, carries which generator (PTP, LIN,
or CIRC) produced either trajectory — every one of those functions only
ever calls `RobotTrajectory` accessors (`getWayPoint`, `getWayPointCount`,
`getFirstWayPoint`, `getLastWayPoint`, `getWayPointDurationFromPrevious`)
and Cartesian-geometry helpers (`linearSearchIntersectionPoint`,
`getFrameTransform`) that work identically regardless of provenance. This
port's own `blend()` (`trajectory_blender_transition_window.rs:224`)
mirrors that exactly: its only inputs are the two owned `RobotTrajectory`s,
`group_name`, `link_name`, `blend_radius`.

**Answering the first of the three questions directly: PTP-into-LIN and
LIN-into-CIRC do not exercise different code paths in `blendTrajectory` —
there is only one code path, and it is blind to generator type.** One
generator pairing exercises the algorithm fully; generator variety is
already covered by the existing `pilz_trajectory` op and would add nothing
here. This request uses **LIN-into-LIN** for both cases below, chosen
(not PTP or CIRC) only because a straight-line Cartesian path makes the
geometry easy to sanity-check by hand when reviewing the harness output —
not because any other generator pairing would exercise different blend
logic.

What *does* vary the blend's control flow is pure geometry local to each
trajectory's approach to the shared boundary: `searchIntersectionPoints`
walks backward from `first_trajectory`'s last waypoint and forward from
`second_trajectory`'s first waypoint, counting how many waypoints of each
fall within `blend_radius` of the boundary pose. `determineTrajectoryAlignment`
then compares those two counts. This port established directly (see
"Cases needed" below) that two LIN segments generated with the *same*
`max_velocity_scaling_factor` produce symmetric counts on both sides
(`determineTrajectoryAlignment`'s `else` branch, unconditionally, across
every `blend_radius` and every segment length tried), while an asymmetric
pair — same segment geometry, second segment's `max_velocity_scaling_factor`
raised — breaks the symmetry and flips to the `way_point_count_1 >
way_point_count_2` branch. Both cases below hold every other parameter
fixed and vary only that one field, specifically so a future reader can
see which single input controls the branch.

## Why the alignment branch needs its own field, not just waypoint comparison

This port's own `blend_trajectory_cartesian_first_sample_stays_near_pose1_last_sample_reaches_pose2`
test (in `trajectory_blender_transition_window.rs`'s test module) was
written wrong on the first attempt this round: it asserted the blended
trajectory's last sample matched `second_trajectory`'s waypoint at index
`0`, when upstream's own `blendTrajectoryCartesian` (lines 205-262 of the
`.cpp`) reassigns `blend_sample_pose2` to `second_trajectory`'s waypoint at
`second_intersection_index` partway through the loop. The test passed
anyway on an earlier, differently-shaped fixture, because that fixture's
`second_intersection_index` happened to be `0` — the wrong assertion and
the right answer coincided. It was only caught by re-reading the upstream
`.cpp` line-by-line during this same round, not by any test failing.

That is a first-hand demonstration of exactly the risk in the brief: a
waypoint-only comparison can pass with the wrong intersection index or the
wrong alignment branch, provided the specific request happens not to make
the two produce visibly different output. **The response therefore must
emit `first_intersection_index`, `second_intersection_index`, and
`determine_trajectory_alignment`'s branch outcome (or `blend_align_index`,
which is equivalent and lets the Rust side infer the branch by re-deriving
`way_point_count_1`/`way_point_count_2` itself) as separate top-level
fields, not something to be inferred from the waypoint arrays.** This is
also the tighter comparison of the two: `searchIntersectionPoints` only
calls `getFrameTransform` (FK), never IK, so its indices carry none of the
IK-solver divergence that `lin_panda_arm_matches_the_oracle`'s own module
doc already documents and budgets `POSITION_TOLERANCE` for (see that
file's `# A known IkContext-level self-collision deviation` section and
its `POSITION_TOLERANCE` doc comment). The indices should therefore be
asserted for **exact integer equality**, independent of, and strictly
stronger than, whatever float tolerance the eventual waypoint comparison
needs — a mismatched index with coincidentally-close waypoints is exactly
the failure this document exists to make visible.

`sampling_time` does not need a separate output field: both segments are
generated with the same `sampling_time` input (echoed in the request,
below), and `determineAndCheckSamplingTime`'s own recovery logic is
already independently unit-tested in `trajectory_functions.rs`. Emitting
it again here would test that unit a second time under a different name,
not add coverage.

## Request JSON shape

One top-level `start_state` (the pose before segment 1) plus a
`segments` array of exactly 2 entries. **The op must generate segment 1
first, then generate segment 2 starting from segment 1's own actual last
waypoint — not from an independently supplied `start_state` for segment
2.** This mirrors upstream's real call sequence exactly:
`plan_components_builder.cpp:75-105`'s `PlanComponentsBuilder::blend()`
passes `blend_request.first_trajectory = traj_tail_` where `traj_tail_` is
the literal `RobotTrajectoryPtr` a prior planning step produced, not a
freshly re-planned trajectory that merely starts at the same nominal pose.
Two independently-solved IK results for the same Cartesian corner can
differ by enough to fail `validateRequest`'s `isRobotStateEqual` boundary
check on `panda_arm`'s redundant kinematics — exactly the divergence
`lin_panda_arm_matches_the_oracle`'s own module doc already measured
(`5.7e-6` per joint between two solvers converging on the same pose). If
the harness generates each segment independently instead of chaining, some
fraction of requests would fail for a reason that has nothing to do with
blending. Chaining removes that failure mode by construction rather than
by picking parameters carefully enough to avoid it.

```json
{
  "op": "pilz_blend",
  "group_name": "panda_arm",
  "link_name": "panda_link8",
  "sampling_time": 0.1,
  "blend_radius": 0.05,
  "joint_limits": { "...": "identical shape to pilz_trajectory's joint_limits, same panda values" },
  "cartesian_limits": {
    "max_trans_vel": 1.0,
    "max_trans_acc": 2.25,
    "max_trans_dec": -5.0,
    "max_rot_vel": 1.57
  },
  "start_state": {
    "panda_joint1": 0.0,
    "panda_joint2": -0.785,
    "panda_joint3": 0.0,
    "panda_joint4": -2.356,
    "panda_joint5": 0.0,
    "panda_joint6": 1.571,
    "panda_joint7": 0.785
  },
  "segments": [
    {
      "generator": "lin",
      "max_velocity_scaling_factor": 0.1,
      "max_acceleration_scaling_factor": 0.1,
      "goal": {
        "kind": "cartesian",
        "link_name": "panda_link8",
        "position": [0.40701957005161055, -5.221329615610066e-12, 0.5902695582766445],
        "orientation": [0.9239556994689483, -0.38249949727920757, 1.324932583900579e-12, 3.2004117663522442e-12]
      }
    },
    {
      "generator": "lin",
      "max_velocity_scaling_factor": 0.1,
      "max_acceleration_scaling_factor": 0.1,
      "goal": {
        "kind": "cartesian",
        "link_name": "panda_link8",
        "position": [0.40701957005161055, 0.1, 0.5902695582766445],
        "orientation": [0.9239556994689483, -0.38249949727920757, 1.324932583900579e-12, 3.2004117663522442e-12]
      }
    }
  ]
}
```

Field notes:

- `link_name`: **not arbitrary — it must be the planning group's IK
  solver tip frame.** `plan_components_builder.cpp:83` sets
  `blend_request.link_name = getSolverTipFrame(model_->getJointModelGroup(group_name))`.
  For `panda_arm` that is `panda_link8`, the same value already used by
  every existing `pilz_trajectory` fixture in this crate.
- `sampling_time`, `joint_limits`, `cartesian_limits`: identical field
  names and semantics to the existing `pilz_trajectory` op
  (`tools/moveit-oracle/src/oracle.cpp`'s `pilzTrajectory`) — reuse that
  vocabulary verbatim rather than inventing a parallel shape. The
  `joint_limits`/`cartesian_limits` values above are the exact panda
  values already committed in
  `crates/moveit-planners-pilz/tests/fixtures/panda_lin_request.json`.
- `start_state`: the pose before segment 1 only. There is no per-segment
  `start_state` field — see the chaining requirement above.
- `segments[i].generator`: included for shape-compatibility with
  `pilz_trajectory` and left as `"lin"` for every case in this request (see
  "Upstream symbol" above for why generator variety is out of scope for
  this specific gap). If a future round wants generator-mix coverage for
  some other reason, the field already exists to support it.
- `segments[i].goal`, `max_velocity_scaling_factor`,
  `max_acceleration_scaling_factor`: identical shape and semantics to
  `pilz_trajectory`'s own `goal`/`max_velocity_scaling_factor`/
  `max_acceleration_scaling_factor` fields, one copy per segment.
- The two goal positions and the shared orientation above are not
  arbitrary placeholders: segment 1's goal is the exact goal from the
  already-oracle-verified `panda_lin_request.json` fixture (ready pose,
  `+0.1m` along `+x`). Segment 2's goal is `+0.1m` along `+y` from that
  same corner, chosen to produce a real direction change (a genuine
  corner, not a straight continuation) so the blend is non-degenerate.
  This port generated both segments locally with its own
  `TrajectoryGeneratorLin` against this exact JSON (values transcribed
  directly from that run, not hand-computed) and confirmed both succeed
  (`error_code` `SUCCESS`) and that `blend()` completes without error on
  the result — this request is not speculative, it is a request to
  reproduce a call this port has already made successfully once, against
  the real oracle model.

## Cases needed: 2, one per side of the alignment-branch boundary

Both cases share every field above (including `blend_radius: 0.05` and
segment 1) and differ **only** in segment 2's `max_velocity_scaling_factor`
/`max_acceleration_scaling_factor`:

| case | segment 2 `max_velocity_scaling_factor` | `determineTrajectoryAlignment` branch (measured, this port's own code, same inputs) | segment 1 waypoints | segment 2 waypoints |
|---|---:|---|---:|---:|
| A — symmetric speed | `0.1` (same as segment 1) | `else` (`way_point_count_1 == way_point_count_2 == 8`) | 16 | 16 |
| B — asymmetric speed | `0.3` | `way_point_count_1 > way_point_count_2` branch (`8 > 4`) | 16 | 9 |

These numbers come from running this port's own `search_intersection_points`/
`determine_trajectory_alignment`/`blend` directly against the two locally-generated
LIN trajectories described above, at `blend_radius = 0.05`, and are reported
here as **this port's own values**, not a prediction of what the oracle
will return — the entire point of this request is to find out whether the
oracle agrees. Both cases ran `blend()` to completion successfully on this
port's side (`first_trajectory`/`blend_trajectory`/`second_trajectory` all
non-empty: case A produced `8`/`8`/`8` waypoints, case B produced
`8`/`8`/`5`), so neither is expected to hit `validateRequest`'s or
`searchIntersectionPoints`'s rejection paths on the oracle side either —
if one does, that mismatch is itself the finding.

No third "degenerate" (`blend_radius` too large, `InvalidMotionPlan`) case
is requested here: that path is pure boundary arithmetic with no IK and no
Cartesian sampling involved, already exhaustively covered by this port's
own `validate_request_rejects_blend_radius_at_or_below_zero` and the
`search_intersection_points_*` tests (see below) with no upstream call
needed to check it — `linearSearchIntersectionPoint` returning "not found"
is a pure geometric fact about the fixture's own trajectories, not
something two different implementations could reasonably disagree about
the way IK convergence or floating-point index arithmetic could.

## Response shape

```json
{
  "error_code": 1,
  "first_intersection_index": 8,
  "second_intersection_index": 7,
  "blend_align_index": 8,
  "sampling_time": 0.1,
  "group_variable_names": ["panda_joint1", "..."],
  "first_trajectory": { "waypoints": [ { "positions": {}, "velocities": {}, "accelerations": {}, "time_from_start": 0.0 } ] },
  "blend_trajectory": { "waypoints": [ "... same per-waypoint shape ..." ] },
  "second_trajectory": { "waypoints": [ "... same per-waypoint shape ..." ] }
}
```

- `error_code`: same convention as `pilz_trajectory` (`1` = `SUCCESS`).
  Since `blend()` has three distinct failure points internally
  (segment-1 generation, segment-2 generation, the blend itself), any
  non-`SUCCESS` response should also say **which** stage failed — a
  `"stage"` field (`"segment1"`/`"segment2"`/`"blend"`) alongside
  `error_code` — since a segment-generation failure is not a finding about
  the blender at all, and this port's own tests already separately cover
  `blend()`'s own rejection paths (see "Overlap" below) without needing an
  oracle round-trip. Neither case in this request is expected to fail, so
  this is a contract for handling an unexpected result, not for these two
  cases' expected path.
- `first_intersection_index`, `second_intersection_index`, `blend_align_index`:
  as argued above, required as separate integer fields — this is the
  actual object of this comparison, not a convenience. Compare these for
  exact equality, not within a float tolerance.
- `sampling_time`: echoed for a self-checking response (mismatch would
  mean `determineAndCheckSamplingTime` itself diverges, a distinct and
  more serious finding than a blend-geometry mismatch), not because this
  request expects it to vary.
- `first_trajectory`/`blend_trajectory`/`second_trajectory`: each the
  **full waypoint array** in exactly `pilz_trajectory`'s existing
  per-waypoint shape (`positions`/`velocities`/`accelerations`/
  `time_from_start`, each a per-variable object), not just `blend_trajectory`
  alone. `first_trajectory`/`second_trajectory` are upstream's *truncated*
  remainders (the part of each original segment outside the blend sphere),
  so their waypoint counts are themselves a second, independent check on
  `first_intersection_index`/`second_intersection_index` (`first_trajectory`'s
  count must equal `first_intersection_index`; `second_trajectory`'s count
  must equal `second_trajectory`'s original length minus
  `second_intersection_index + 1`). Comparing all three keeps that
  cross-check meaningful — comparing `blend_trajectory` alone would not
  catch a truncation-boundary bug in the other two.
- `group_variable_names`: same convention as `pilz_trajectory`, needed to
  key `positions`/`velocities`/`accelerations` unambiguously.
- No `planning_time` field, for the same reason `pilz_trajectory` never
  emits one (`oracle.cpp`'s own comment: non-reproducible wall-clock,
  would need permanent per-fixture exclusion entries).

Precision/tolerance is not specified here since it is not yet known: the
waypoint arrays inherit the same IK-solver divergence this crate's LIN/CIRC
fixtures already budget for (`POSITION_TOLERANCE`/`VELOCITY_TOLERANCE`/
`ACCELERATION_TOLERANCE` in `pilz_trajectory_lin_parity.rs`), and the right
tolerance for *this* op can only be set from a measured maximum divergence
once real responses exist, per `CLAUDE.md`'s "Size test tolerances from
measurement" — this port will not guess a number and will not reuse LIN's
numbers unmeasured.

## Overlap with this port's existing tests

The brief's count of "nine" is off by three: the module currently has
**twelve** tests, not nine — `validate_request` alone has six cases
(unknown group, unknown link, non-positive blend radius, boundary
mismatch, sampling-time mismatch, and one accept case), not five. Full
count: `validate_request` ×6, `determine_trajectory_alignment` ×2,
`search_intersection_points` ×2, `blend_trajectory_cartesian` ×1 (the
boundary-value test whose earlier wrong assertion motivated this
document's "Why the alignment branch needs its own field" section above),
and one end-to-end `blend_produces_a_continuous_trajectory_through_the_shared_boundary`.

None of the twelve would become redundant if this op is built — they test
a different property than the op would:

- The six `validate_request` cases and the two `search_intersection_points`
  "found"/"not-found" cases are Rust-side **input-validation and
  boundary-arithmetic** checks: given a made-up malformed or well-formed
  request, does this port's own code accept/reject it the way its own
  logic says it should. There is no upstream numeric output to compare
  against for a rejection — a rejection is a boolean outcome this port's
  code either gets right or wrong on its own terms, already exercised
  without any oracle call. (Contrast this with `lin_panda_arm_matches_the_
  oracle`'s existing "rejects the same request the oracle rejects" pattern:
  that pattern exists for LIN/CIRC specifically because their acceptance
  path runs through solver-dependent IK reachability, which genuinely can
  diverge between implementations. `validate_request`'s checks are pure
  arithmetic/string comparison with no solver in the loop, so that
  precedent does not transfer here.)
- The two `determine_trajectory_alignment` tests pass **hand-picked**
  intersection indices directly (`assert_eq!(determine_trajectory_alignment(&req, 2, 3), 7)`)
  to confirm the function's own branch arithmetic is correct for chosen
  numbers — they never call `search_intersection_points` on real geometry,
  so they say nothing about whether upstream's `searchIntersectionPoints`
  would produce the *same* indices this port's version does on a real
  trajectory. That is exactly what this request's cases newly check.
- `blend_trajectory_cartesian`'s existing test is a **self-consistency**
  check: it asserts the blend's first/last samples equal poses computed by
  the same function under test, which catches wiring bugs (as it already
  did once this round) but cannot catch e.g. a slerp-direction sign error
  or an off-by-one in the smoothstep parameter that still satisfies
  "approaches pose2 monotonically." Only a real upstream value can catch
  that class of bug.
- The end-to-end test only asserts **structural** properties (segment
  counts non-increasing, blend segment non-empty) with zero numeric
  comparison to any external source.

None of the twelve compares a single number this port produced against a
number upstream produced. The oracle op would be the first thing in this
module that does.

## How this port will use the response

Once received, this port adds
`crates/moveit-planners-pilz/tests/pilz_blend_parity.rs`, following
`pilz_trajectory_lin_parity.rs`'s existing structure: generate both
segments locally with `TrajectoryGeneratorLin`, call `blend()`, and assert
`first_intersection_index`/`second_intersection_index`/`blend_align_index`
for exact equality and every waypoint field within a tolerance measured
from the actual response (not guessed). If case B's response confirms the
`way_point_count_1 > way_point_count_2` branch actually fired on the
oracle side too, that closes the branch-coverage gap this document exists
for. If either case's indices disagree with this port's own values while
the final waypoints still end up numerically close, that is precisely the
"branch inverted, output coincidentally similar" failure this document
was written to make visible rather than silently pass — and, per this
round's brief, deciding what to do about such a finding is brought back to
the orchestrator rather than resolved unilaterally here.
