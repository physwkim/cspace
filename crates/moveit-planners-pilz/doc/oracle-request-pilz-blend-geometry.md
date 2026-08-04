# Oracle request: `pilz_blend` geometry sweep — cases C and D

Not an implementation — request document for the human orchestrator, same
convention as `oracle-request-pilz-blend.md`. Same op (`pilz_blend`), same
request/response JSON shape as that document; this file only specifies
what differs for two new cases and does not repeat settled material
(upstream symbol, chaining requirement, field notes, tolerance policy) —
read that document first.

## The gap this closes

Cases A and B (`oracle-request-pilz-blend.md`) pin one geometry — LIN-into-
LIN on panda, corner at the ready pose +0.1m/+x then +0.1m/+y,
`blend_radius: 0.05` — and vary only segment 2's speed scaling. Both give
`first_intersection_index == 8`. So `search_intersection_points`'s
backward/forward walks are each exercised at exactly one index value, and
`blend_align_index`'s arithmetic is pinned at one specific input rather
than checked across a range. Cases C and D below move the geometry itself
instead: a different `blend_radius` (case C) and a different corner angle
(case D), each holding every other field at case A's value.

Both predictions below come from running this port's own
`search_intersection_points` locally against the exact segments each case
requests (a temporary probe test, written, run, and reverted — not left in
the tree), the same method `oracle-request-pilz-blend.md`'s case A/B
numbers came from.

## Case C: `blend_radius: 0.08` (same corner and speed as case A)

Every field identical to case A except `blend_radius`. Segment 1 and
segment 2 (goal `[0.40701957005161055, 0.1, 0.5902695582766445]`, both
scaling factors `0.1`, chained onto segment 1's own last waypoint) are
unchanged from case A.

```json
{
  "op": "pilz_blend",
  "group_name": "panda_arm",
  "link_name": "panda_link8",
  "sampling_time": 0.1,
  "blend_radius": 0.08,
  "joint_limits": { "...": "same as oracle-request-pilz-blend.md's case A" },
  "cartesian_limits": { "...": "same as oracle-request-pilz-blend.md's case A" },
  "start_state": { "...": "same as oracle-request-pilz-blend.md's case A" },
  "segments": [
    { "generator": "lin", "max_velocity_scaling_factor": 0.1, "max_acceleration_scaling_factor": 0.1,
      "goal": { "kind": "cartesian", "link_name": "panda_link8",
        "position": [0.40701957005161055, -5.221329615610066e-12, 0.5902695582766445],
        "orientation": [0.9239556994689483, -0.38249949727920757, 1.324932583900579e-12, 3.2004117663522442e-12] } },
    { "generator": "lin", "max_velocity_scaling_factor": 0.1, "max_acceleration_scaling_factor": 0.1,
      "goal": { "kind": "cartesian", "link_name": "panda_link8",
        "position": [0.40701957005161055, 0.1, 0.5902695582766445],
        "orientation": [0.9239556994689483, -0.38249949727920757, 1.324932583900579e-12, 3.2004117663522442e-12] } }
  ]
}
```

**Predicted:** `first_intersection_index = 5`, `second_intersection_index = 10`,
`blend_align_index`/branch = `else` (`way_point_count_1 == way_point_count_2 == 11`),
`first_trajectory` waypoints unchanged at `16`, `second_trajectory` waypoints `16`.

**What this discriminates that A/B cannot.** A/B only ever probed
`blend_radius = 0.05`; the two index values (`8`/`7`) are pinned at
exactly one point. Case C is not expected to newly exercise the alignment
*branch* — a local sweep over `blend_radius` at case A's exact symmetric
geometry (`0.02→(11,4)`, `0.03→(10,5)`, `0.08→(5,10)`, `0.1→(1,14)`, all
`else`) shows the branch stays `else` at every radius tried, because
symmetric-speed segments have identical waypoint density by construction
— that is what "symmetric" means here, and case B already covers the
other branch. What case C *does* newly exercise is
`linear_search_intersection_point`'s backward/forward walk arithmetic at
index values genuinely different from `8`/`7`: a stopping-condition or
off-by-one bug masked near the middle of a 16-waypoint trajectory (index
8) would not necessarily be masked nearer an end (index 5 or 10).
`0.08` was chosen over the other three viable candidates because its
indices sit furthest from both A/B's existing `8`/`7` and the
trajectory's own boundaries (`0`/`15`) — `0.1`'s `first_intersection_index
= 1` is one step from the fully-degenerate case and a sharper edge-case
than this round asked for. `0.12` was tried locally and rejected
(`Code(InvalidMotionPlan)`, exceeds either trajectory's reach); not
requested here, for the same reason A/B's own document gives for omitting
a rejection-path case — pure boundary arithmetic with no IK or Cartesian
sampling involved, not something two implementations could reasonably
disagree about.

## Case D: 150° corner (same `blend_radius` and speed as case A)

Segment 2's goal changes from `+0.1m` along `+y` (case A's corner, 90°
from segment 1's `+x` direction) to `+0.1m` at 150° from segment 1's
direction. `blend_radius` (`0.05`), both scaling factors (`0.1`), and the
chaining requirement are unchanged from case A.

```json
{
  "op": "pilz_blend",
  "group_name": "panda_arm",
  "link_name": "panda_link8",
  "sampling_time": 0.1,
  "blend_radius": 0.05,
  "joint_limits": { "...": "same as oracle-request-pilz-blend.md's case A" },
  "cartesian_limits": { "...": "same as oracle-request-pilz-blend.md's case A" },
  "start_state": { "...": "same as oracle-request-pilz-blend.md's case A" },
  "segments": [
    { "generator": "lin", "max_velocity_scaling_factor": 0.1, "max_acceleration_scaling_factor": 0.1,
      "goal": { "kind": "cartesian", "link_name": "panda_link8",
        "position": [0.40701957005161055, -5.221329615610066e-12, 0.5902695582766445],
        "orientation": [0.9239556994689483, -0.38249949727920757, 1.324932583900579e-12, 3.2004117663522442e-12] } },
    { "generator": "lin", "max_velocity_scaling_factor": 0.1, "max_acceleration_scaling_factor": 0.1,
      "goal": { "kind": "cartesian", "link_name": "panda_link8",
        "position": [0.32041702967316665, 0.05000000000000000, 0.59026955827664451],
        "orientation": [0.9239556994689483, -0.38249949727920757, 1.324932583900579e-12, 3.2004117663522442e-12] } }
  ]
}
```

(Segment 2's goal position above is transcribed directly from this port's
own local run — `corner + 0.1 * (cos150°, sin150°, 0)` — not hand-computed.)

**Predicted: identical to case A** — `first_intersection_index = 8`,
`second_intersection_index = 7`, branch `else`. This is a measured
prediction, not an assumption offered to satisfy the brief's format: three
angle candidates (45°, 120°, 150°) were run locally holding `blend_radius`
and corner-to-goal distance fixed at case A's values, and all three
reproduced case A's indices exactly.

This is explained by `search_intersection_points`'s own structure, and is
itself a finding this round measured rather than assumed:
`first_intersection_index` is a function only of `first_trajectory`'s own
waypoints' distance to the fixed boundary pose (`circ_pose`, computed from
`first_trajectory`'s own last waypoint, before segment 2 is ever
generated) — it cannot depend on segment 2's direction at all.
`second_intersection_index` is a function only of `second_trajectory`'s
own waypoints' distance to that same fixed center; the walk reads
distance-to-center along its own path, never the other trajectory's
direction. Since segment 2's distance-per-waypoint profile (`0.1m` total
at `0.1` scaling) is the same regardless of which direction it points, its
distance-to-corner sequence — and therefore its intersection index — is
angle-invariant here. **A pure angle change, with distance and speed held
fixed, cannot move either index; that is not a gap in this port's
understanding of the algorithm, it is what the algorithm does.**

So this prediction matching is not a weak or trivial result: if the
oracle disagrees with it, that specifically means either (a) upstream's
`searchIntersectionPoints` has some angle-dependent term this port's
version dropped, or (b) IK convergence at a sharper corner produces
measurably different waypoint spacing despite the nominal `0.1m`/`0.1`-
scaling inputs being identical — a possibility this port cannot rule out
from local FK-only geometry, since `search_intersection_points` itself
never calls IK but the segment *generation* upstream of it does. If the
indices do match as predicted, case D's value is elsewhere: it is the
first case in this module exercising `blend_trajectory_cartesian`'s
slerp/quintic sampling at a corner substantially sharper than A/B's 90° —
a slerp-direction sign error or interpolation bug invisible at a gentle
90° turn could still show up in the **waypoint arrays** at 150° even with
identical indices. So case D's response should be compared fully on the
waypoint arrays, not only on the index fields; the index match itself is
offered as the falsifiable prediction the brief asked for.

150° was chosen over the other two candidates tried locally (45°, 120°)
because it is the sharpest deviation from A/B's existing 90° — closest to
a near-reversal of direction — maximizing the chance of exposing a
slerp-arithmetic bug if one exists; 45°/120° are both closer to A/B's
already-covered corner and would be a weaker version of the same test.

## Response shape, tolerance

Unchanged from `oracle-request-pilz-blend.md` — same `pilz_blend` op,
same fields (`error_code`, `first_intersection_index`,
`second_intersection_index`, `blend_align_index`, `sampling_time`,
`group_variable_names`, `first_trajectory`/`blend_trajectory`/
`second_trajectory` waypoint arrays). Both cases are expected to succeed
(`error_code` `SUCCESS`) on this port's own side — neither case was
rejected locally — so no new rejection-path handling is needed. Tolerance
for the waypoint arrays is not set here for the same reason the original
document gives: it will be measured from the actual responses, not
guessed or carried over from LIN's numbers.

## How this port will use the response

Extends `crates/moveit-planners-pilz/tests/pilz_blend_parity.rs` with two
more fixture-backed cases, same structure as the existing case A/B tests:
assert `first_intersection_index`/`second_intersection_index`/
`blend_align_index` for exact integer equality, waypoint fields within a
tolerance re-measured from all four cases' responses combined (not case
C/D alone, since a wider case set can only raise a measured maximum, never
lower it). If case C's indices disagree with this port's local values,
that is a `search_intersection_points` divergence at a radius A/B never
exercised. If case D's indices disagree with the identical-to-A prediction
above, that falsifies this round's own explanation of why the walk is
angle-invariant and is reported back rather than resolved unilaterally
here, per the standing brief.
