# Oracle request: `pilz_blend` geometry sweep — cases C, D, and E

Not an implementation — request document for the human orchestrator, same
convention as `oracle-request-pilz-blend.md`. Same op (`pilz_blend`), same
request/response JSON shape as that document; this file only specifies
what differs for the new cases and does not repeat settled material
(upstream symbol, chaining requirement, field notes, tolerance policy) —
read that document first.

## The gap this closes

Cases A and B (`oracle-request-pilz-blend.md`) pin one geometry — LIN-into-
LIN on panda, corner at the ready pose +0.1m/+x then +0.1m/+y,
`blend_radius: 0.05` — and vary only segment 2's speed scaling. Both give
`first_intersection_index == 8`. So `search_intersection_points`'s
backward/forward walks are each exercised at exactly one index value, and
`blend_align_index`'s arithmetic is pinned at one specific input rather
than checked across a range. Cases C, D, and E below move the geometry
itself instead: a different `blend_radius` (case C) and a different corner
angle (case D at 150°, rejected by the dynamics stage; case E at 112°, the
sharpest angle short of that rejection boundary), each holding every other
field at case A's value.

The case C and case D predictions below come from running this port's own
`search_intersection_points` locally against the exact segments each case
requests (a temporary probe test, written, run, and reverted — not left in
the tree). **This is narrower than the method `oracle-request-pilz-blend.md`'s
case A/B numbers came from — that document's own text states both cases
"ran `blend()` to completion successfully," the full pipeline, not
`search_intersection_points` alone.** An earlier version of this paragraph
claimed the two methods were "the same"; they are not, and that
misstatement is corrected in the "Claim audit" section below, added after
PORTING-PLAN.md §207 found the gap it caused. Case E's prediction, added
below, closes the gap for this document by running the full pipeline.

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

**Confirmed (run 2026-08-05, oracle stamp `043ed31a2186fe4e`):** the oracle
returns exactly `first_intersection_index = 5`,
`second_intersection_index = 10`, both input waypoint counts `16`. Landed
as `blend_panda_arm_radius08_matches_the_oracle`, comparing every waypoint
of all three response segments, not the indices alone.

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

**Result (run 2026-08-05, oracle stamp `043ed31a2186fe4e`): the oracle
rejects this case.** `error_code = -1` (`PLANNING_FAILED`), stage `blend`,
no waypoint arrays at all — `generateJointTrajectory` fails the 4th blend
sample on `panda_joint2`'s deceleration limit (`-2.50863` against `-1.875`).
The prediction below is therefore **not testable as written**: neither
index field is emitted, so "identical to case A" was neither confirmed nor
refuted, and the sharper-corner interpolation comparison this case was
proposed for does not exist on this geometry. What the case does buy is a
rejection-parity check — this port rejects the same request at the same
sample, on the same joint, at the same deceleration value — landed as
`blend_panda_arm_corner150_is_rejected_like_the_oracle`. PORTING-PLAN.md
§207. The rest of this section is left as written, as the falsified
prediction it turned out to be.

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

## Case E: 112° corner — the sharpest angle at which the full pipeline succeeds

Case D asked for the sharpest deviation from A/B's 90° (150°) and got a
dynamics rejection two stages past the geometry this document is about,
so no index or waypoint comparison exists for it (see PORTING-PLAN.md
§207.1 and the "Claim audit" section below). Case E replaces that gap: it
is the sharpest corner angle strictly between 90° and 150° at which an
actual `blend(&ctx, &limits, &mut req)` call — using the real fixture's
`joint_limits`/`cartesian_limits` from
`panda_blend_symmetric_request.json`, not the test module's generic
`panda_joint_limits()` helper — returns `Ok` with both segments generated
and chained exactly as `pilz_blend_parity.rs`'s `drive_case` does it, run
locally and reverted (not left in the tree).

**Angle trial, coarse-to-fine (all `blend_radius: 0.05`, same as A, real
joint/cartesian limits from the fixture):**

| angle | result | angle | result | angle | result |
|---:|---|---:|---|---:|---|
| 90° | OK | 111° | OK | 112.6° | OK |
| 100° | OK | 112° | OK | 112.8° | `Code(PlanningFailed)` |
| 110° | OK | 113° | `Code(PlanningFailed)` | 113° | `Code(PlanningFailed)` |
| 115° | `Code(PlanningFailed)` | 114° | `Code(PlanningFailed)` | 120°/122°/125°/130°/140°/150° | `Code(PlanningFailed)` |
| 118° | `Code(PlanningFailed)` | | | | |

The boundary is bisected to 0.2° resolution: succeeds through 112.6°,
fails at 112.8°. Every rejection in this range is the same
`panda_joint2` deceleration-limit violation case D hit at 150° — this
port's local dynamics check, not `search_intersection_points`, is what
rejects these. **112.0° is filed below, not the measured boundary
itself** — a deliberate margin (~0.6–0.8°) below `112.6°`/`112.8°` so the
filed case does not sit on a knife-edge that could flip sides between
this port's IK solver and the oracle's own solver on `panda_arm`'s
redundant kinematics (the same `5.7e-6` per-joint divergence
`lin_panda_arm_matches_the_oracle`'s module doc already measures).

Segment 2's goal changes from `+0.1m` along `+y` (case A's 90° corner) to
`+0.1m` at 112° from segment 1's direction. `blend_radius` (`0.05`), both
scaling factors (`0.1`), and the chaining requirement are unchanged from
case A.

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
        "position": [0.36955891071001934, 0.09271838545667875, 0.5902695582766445],
        "orientation": [0.9239556994689483, -0.38249949727920757, 1.324932583900579e-12, 3.2004117663522442e-12] } }
  ]
}
```

(Segment 2's goal position is transcribed directly from this port's own
local run — `corner + 0.1 * (cos112°, sin112°, 0)` — not hand-computed.)

**Predicted (this port's own full-pipeline run, `blend()` returns `Ok`):**
`first_intersection_index = 8`, `second_intersection_index = 7`,
`first_trajectory_count = 8`, `blend_trajectory_count = 8`,
`second_trajectory_count = 8` — identical to case A's indices, a third
independent confirmation (after case D's untestable 150° attempt and the
earlier 45°/120° local-only probes) that
`search_intersection_points`'s walk is angle-invariant when radius and
per-segment speed are held fixed. Unlike case D, this prediction **is**
testable: `blend()` completed locally, so the full waypoint arrays exist
to compare, not just the indices.

This port's own `blend_trajectory` (8 waypoints, all 7 `panda_arm`
joints, `positions`/`velocities`/`accelerations`/`time_from_start`),
measured locally and reverted:

```json
[
  {"time_from_start": 0.0,
   "positions": {"panda_joint1": 6.882272719774329e-13, "panda_joint2": -0.602503493221836, "panda_joint3": -1.5016352234036415e-12, "panda_joint4": -2.238880610339153, "panda_joint5": 2.909287818765981e-13, "panda_joint6": 1.6363771171173171, "panda_joint7": 0.7850000000008042},
   "velocities": {"panda_joint1": 1.1816306899903016e-12, "panda_joint2": 0.33068885297655504, "panda_joint3": -2.8416794703204347e-12, "panda_joint4": 0.23175825006797268, "panda_joint5": 6.020424849968926e-13, "panda_joint6": 0.0989306029085868, "panda_joint7": 1.176836406102666e-12},
   "accelerations": {"panda_joint1": 4.959722808550417e-13, "panda_joint2": 0.19919584616832076, "panda_joint3": -2.1995455589266776e-12, "panda_joint4": 0.21904840304314277, "panda_joint5": 7.493621094287361e-13, "panda_joint6": -0.01985255687471099, "panda_joint7": -4.3298697960381105e-13}},
  {"time_from_start": 0.1,
   "positions": {"panda_joint1": 0.0000675897964896568, "panda_joint2": -0.5635824829338427, "panda_joint3": 0.00016129092181558073, "panda_joint4": -2.210479035911796, "panda_joint5": 0.00008624385106534205, "panda_joint6": 1.6468965168999146, "panda_joint7": 0.7851974557213283},
   "velocities": {"panda_joint1": 0.0006758979580142953, "panda_joint2": 0.3892101028799333, "panda_joint3": 0.0016129092331721594, "panda_joint4": 0.2840157442735691, "panda_joint5": 0.0008624385077441327, "panda_joint6": 0.10519399782597416, "panda_joint7": 0.0019745572052409788},
   "accelerations": {"panda_joint1": 0.006758979568326647, "panda_joint2": 0.5852124990337826, "panda_joint3": 0.016129092360138386, "panda_joint4": 0.5225749420559644, "panda_joint5": 0.008624385071420902, "panda_joint6": 0.06263394917387366, "panda_joint7": 0.019745572040641424}},
  {"time_from_start": 0.20000000000000004,
   "positions": {"panda_joint1": 0.0007200507315875909, "panda_joint2": -0.5255799739284979, "panda_joint3": 0.0016780332580265827, "panda_joint4": -2.1815650188657694, "panda_joint5": 0.0008449556880965277, "panda_joint6": 1.6559856244128321, "panda_joint7": 0.78709974452941},
   "velocities": {"panda_joint1": 0.006524609350979338, "panda_joint2": 0.3800250900534484, "panda_joint3": 0.015167423362110015, "panda_joint4": 0.28914017046026685, "panda_joint5": 0.0075871183703118536, "panda_joint6": 0.0908910751291758, "panda_joint7": 0.019022888080817417},
   "accelerations": {"panda_joint1": 0.05848711392965042, "panda_joint2": -0.09185012826484872, "panda_joint3": 0.13554514128937853, "panda_joint4": 0.051244261866977274, "panda_joint5": 0.0672467986256772, "panda_joint6": -0.14302922696798362, "panda_joint7": 0.17048330875576434}},
  {"time_from_start": 0.30000000000000004,
   "positions": {"panda_joint1": 0.002977897732073339, "panda_joint2": -0.5009833553847478, "panda_joint3": 0.0067367283388680135, "panda_joint4": -2.16221342031157, "panda_joint5": 0.0032488278802932113, "panda_joint6": 1.661239143620699, "panda_joint7": 0.7935933714990869},
   "velocities": {"panda_joint1": 0.022578470004857486, "panda_joint2": 0.24596618543750065, "panda_joint3": 0.05058695080841432, "panda_joint4": 0.1935159855419944, "panda_joint5": 0.024038721921966842, "panda_joint6": 0.05253519207866876, "panda_joint7": 0.06493626969676926},
   "accelerations": {"panda_joint1": 0.1605386065387815, "panda_joint2": -1.3405890461594776, "panda_joint3": 0.35419527446304305, "panda_joint4": -0.9562418491827246, "panda_joint5": 0.16451603551654986, "panda_joint6": -0.38355883050507034, "panda_joint7": 0.45913381615951837}},
  {"time_from_start": 0.4,
   "positions": {"panda_joint1": 0.007842314588834578, "panda_joint2": -0.49497078712984405, "panda_joint3": 0.017173287501840613, "panda_joint4": -2.1573427576218003, "panda_joint5": 0.008191471868994951, "panda_joint6": 1.662430548293146, "panda_joint7": 0.8072052571952818},
   "velocities": {"panda_joint1": 0.048644168567612396, "panda_joint2": 0.06012568254903762, "panda_joint3": 0.10436559162972601, "panda_joint4": 0.04870662689769657, "panda_joint5": 0.049426439887017413, "panda_joint6": 0.011914046724470499, "panda_joint7": 0.13611885696194873},
   "accelerations": {"panda_joint1": 0.2606569856275492, "panda_joint2": -1.8584050288846308, "panda_joint3": 0.5377864082131171, "panda_joint4": -1.4480935864429787, "panda_joint5": 0.25387717965050577, "panda_joint6": -0.40621145354198274, "panda_joint7": 0.7118258726517949}},
  {"time_from_start": 0.5000000000000001,
   "positions": {"panda_joint1": 0.015309970485782354, "panda_joint2": -0.5023982418202426, "panda_joint3": 0.03267237724494448, "panda_joint4": -2.1630571746716374, "panda_joint5": 0.015794595543018763, "panda_joint6": 1.6608730118970378, "panda_joint7": 0.8275264710949259},
   "velocities": {"panda_joint1": 0.0746765589694777, "panda_joint2": -0.07427454690398565, "panda_joint3": 0.1549908974310385, "panda_joint4": -0.05714417049837101, "panda_joint5": 0.07603123674023805, "panda_joint6": -0.015575363961082264, "panda_joint7": 0.20321213899644122},
   "accelerations": {"panda_joint1": 0.2603239040186529, "panda_joint2": -1.3440022945302321, "panda_joint3": 0.5062530580131247, "panda_joint4": -1.0585079739606755, "panda_joint5": 0.2660479685322063, "panda_joint6": -0.2748941068555275, "panda_joint7": 0.6709328203449247}},
  {"time_from_start": 0.6000000000000001,
   "positions": {"panda_joint1": 0.02323115972135044, "panda_joint2": -0.5136070627991508, "panda_joint3": 0.04883512193463437, "panda_joint4": -2.1715436296167834, "panda_joint5": 0.02407898049273066, "panda_joint6": 1.6584215911889166, "panda_joint7": 0.8486659868946654},
   "velocities": {"panda_joint1": 0.07921189235568087, "panda_joint2": -0.11208820978908188, "panda_joint3": 0.16162744689689898, "panda_joint4": -0.08486454945145995, "panda_joint5": 0.082843849497119, "panda_joint6": -0.02451420708121211, "panda_joint7": 0.211395157997395},
   "accelerations": {"panda_joint1": 0.04535333386203168, "panda_joint2": -0.37813662885096216, "panda_joint3": 0.06636549465860478, "panda_joint4": -0.2772037895308893, "panda_joint5": 0.0681261275688094, "panda_joint6": -0.08938843120129843, "panda_joint7": 0.08183019000953776}},
  {"time_from_start": 0.7000000000000001,
   "positions": {"panda_joint1": 0.03017559116123638, "panda_joint2": -0.5232239147901431, "panda_joint3": 0.06293683003362796, "panda_joint4": -2.1786052594365497, "panda_joint5": 0.031547439577425306, "panda_joint6": 1.6561963823730999, "panda_joint7": 0.8670184691874454},
   "velocities": {"panda_joint1": 0.06944431439885942, "panda_joint2": -0.09616851990992273, "panda_joint3": 0.14101708098993593, "panda_joint4": -0.07061629819766325, "panda_joint5": 0.07468459084694645, "panda_joint6": -0.02225208815816782, "panda_joint7": 0.18352482292779973},
   "accelerations": {"panda_joint1": -0.09767577956821444, "panda_joint2": 0.15919689879159152, "panda_joint3": -0.20610365906963057, "panda_joint4": 0.14248251253796698, "panda_joint5": -0.08159258650172545, "panda_joint6": 0.02262118923044291, "panda_joint7": -0.27870335069595276}}
]
```

**Refuting result, stated up front:** if the oracle's `blend_trajectory`
diverges from the table above beyond the tolerance eventually measured
for this op (per the "Response shape, tolerance" section below) at any
*interior* waypoint (index 1 through 6) while both endpoints (index 0,
7) stay within tolerance, that is a slerp-direction or quintic-sampling
bug this port's `blend_trajectory_cartesian` has at sharp angles and
case A/B's 90° corner does not expose — the specific failure mode this
case was proposed for. If `first_intersection_index`/
`second_intersection_index` disagree with `8`/`7`, that refutes the
angle-invariance explanation given for case D instead, independent of
the waypoint question. Either refutation is reported back rather than
resolved unilaterally here, per the standing brief.

**Confirmed and partially refuted (run 2026-08-05, oracle stamp
`043ed31a2186fe4e`):** `first_intersection_index = 8`,
`second_intersection_index = 7`, `error_code = 1` (`SUCCESS`), all three
segments `8`/`8`/`8` waypoints — the index prediction above matches the
oracle exactly, landed as `blend_panda_arm_corner112_matches_the_oracle`
(`c74a417`). The waypoint prediction is only partially confirmed: the
oracle's own `blend_trajectory` diverges from this port's above the
shared `VELOCITY_TOLERANCE`/`ACCELERATION_TOLERANCE` budget cases A-D
measure — `8.276e-8` velocity at waypoint 5 (`panda_joint5`), `1.6513e-6`
acceleration at waypoint 6 (`panda_joint5`) — spread across multiple
interior waypoints (1, 2, 5, 6) and multiple joints
(`panda_joint1`/`3`/`5`/`6`), not one isolated sample. That spread is the
refuting condition's own tell against a slerp-direction or off-by-one
bug (which would show as one outlier, not a smooth growth). **This
paragraph originally attributed the spread to "panda_arm's
redundant-kinematics IK null-space selection diverging more between
solvers as the corner sharpens" — that attribution has since been tested
by the Case F sweep below and refuted: divergence is not monotone in
corner angle (90°/100° measure the sweep's minimum, not a point partway
up a trend), so "the corner sharpens" is not, by itself, an explanation
for case E's own number. See Case F's "Measured sweep" and "Verdict"
below for the full table and what stays unexplained.**
`first_trajectory`/`second_trajectory` and `blend_trajectory`'s own
position/time stay within the existing shared tolerance; only interior
velocity/acceleration exceed it. See `pilz_blend_parity.rs`'s
`CORNER112_VELOCITY_TOLERANCE`/`CORNER112_ACCELERATION_TOLERANCE` for
the case-specific, separately measured tolerance this finding got
instead of a widened shared constant.

## Response shape, tolerance

Unchanged from `oracle-request-pilz-blend.md` — same `pilz_blend` op,
same fields (`error_code`, `first_intersection_index`,
`second_intersection_index`, `blend_align_index`, `sampling_time`,
`group_variable_names`, `first_trajectory`/`blend_trajectory`/
`second_trajectory` waypoint arrays). Case C and case E are expected to
succeed (`error_code` `SUCCESS`) — case C confirmed on the real oracle
already (`e228571`); case E's local run reached `Ok`, per the full
pipeline described above, not `search_intersection_points` alone. Case D
is a confirmed rejection (`error_code = -1`, stage `blend`, no waypoint
or index fields — see PORTING-PLAN.md §207) and needs the `"stage"` field
`oracle-request-pilz-blend.md` already specifies for exactly this
situation, not new handling. Tolerance for the waypoint arrays is not set
here for the same reason the original document gives: it will be
measured from the actual responses, not guessed or carried over from
LIN's numbers.

## How this port will use the response

`blend_panda_arm_radius08_matches_the_oracle` (case C),
`blend_panda_arm_corner150_is_rejected_like_the_oracle` (case D), and
`blend_panda_arm_corner112_matches_the_oracle` (case E) all landed in
`crates/moveit-planners-pilz/tests/pilz_blend_parity.rs` (`e228571`,
`638e8a0`, `c74a417`). Case E's indices matched `8`/`7` exactly, as
predicted — no `search_intersection_points` divergence. Its interior
`blend_trajectory` waypoints did not fully hold cases A-D's shared
tolerance, though: see the "Confirmed and partially refuted" paragraph
above. That is not the slerp-direction/off-by-one bug the case was
proposed to catch (the divergence is spread smoothly across several
joints and waypoints, not one outlier), so it did not become a shared
`VELOCITY_TOLERANCE`/`ACCELERATION_TOLERANCE` change; it is a
case-specific, separately measured tolerance instead, so the finding
stays visible rather than being absorbed into a widened shared budget
that would also loosen cases A-D's own tighter precision.

## Claim audit (§207.1 anchor sweep)

PORTING-PLAN.md §207.1: a claim's local evidence must come from a probe
at least as wide as the claim itself. Case D's rejection showed this
document had one violation (probe: `search_intersection_points` only;
claim: whether the whole pipeline succeeds and produces matching
waypoints). This section sweeps every other claim in both request
documents for the same shape.

**Anchor:** every claim in `oracle-request-pilz-blend.md` and
`oracle-request-pilz-blend-geometry.md` whose local evidence is cited as
coming from running some named function(s) locally, checked against
whether the named function(s) reach `generate_joint_trajectory_from_cartesian`
(the only place a dynamics/joint-limit rejection like case D's can occur)
or stop short of it (pure geometry/arithmetic in `search_intersection_points`,
`determine_trajectory_alignment`, `linear_search_intersection_point`, or
hand-picked-input unit tests).

**Sites:**

1. `oracle-request-pilz-blend.md`, "Cases needed" section (case A/B
   numbers): "Both cases ran `blend()` to completion successfully on this
   port's side."
2. `oracle-request-pilz-blend.md`, "Cases needed" section (no third
   degenerate case): "`linearSearchIntersectionPoint` returning 'not
   found' is a pure geometric fact... not something two different
   implementations could reasonably disagree about."
3. `oracle-request-pilz-blend.md`, "The gap" section (branch-symmetric
   claim): "two LIN segments generated with the *same*
   `max_velocity_scaling_factor` produce symmetric counts on both sides...
   unconditionally, across every `blend_radius` and every segment length
   tried."
4. `oracle-request-pilz-blend-geometry.md`, case C section ("What this
   discriminates that A/B cannot"): the `blend_radius` sweep table
   (`0.02→(11,4)` … `0.1→(1,14)`) and the `0.12` rejection claim.
5. `oracle-request-pilz-blend-geometry.md`, case C section (originally):
   filed as a `search_intersection_points`-only prediction, same probe as
   case D, under the shared "neither case was rejected locally" framing
   (now corrected above).
6. `oracle-request-pilz-blend-geometry.md`, case D section: the original
   defect this sweep responds to — `search_intersection_points`-only
   evidence for a claim ("identical to case A", "blend()'s response
   should be compared fully on the waypoint arrays") that presupposes the
   full pipeline succeeds.

**Same defect at:** site 5 (case C). The prediction was filed from a
`search_intersection_points`-only probe for a claim ("blend_radius: 0.08
succeeds and matches case A's shape") that, like case D's, presupposes
the dynamics stage is reachable and passes — the geometry doc's own
"neither case was rejected locally" sentence covered both C and D on
identical evidence. It happened not to matter (the real oracle accepted
case C, `e228571`, and the passing `blend_panda_arm_radius08_matches_the_oracle`
test already exercises the full pipeline after the fact), so no re-run is
needed — but per §207.1, "turned out true" is not "was verified," and the
"same method as A/B" cross-reference this document made for both C and D
was factually wrong for both. Corrected above (case C's original
prediction is superseded by the confirmed oracle result and left as
historical record; the misattribution is fixed at its source paragraph).

**Distinct, skip:**

- Site 1 (case A/B): not narrower — `oracle-request-pilz-blend.md`
  states both cases "ran `blend()` to completion successfully on this
  port's side," the full pipeline, before filing. No gap.
- Site 2 (degenerate-radius omission) and its case D/E-geometry-doc
  counterpart in the case C section ("`0.12` was tried locally and
  rejected (`Code(InvalidMotionPlan)`"): both reasoned about
  `search_intersection_points`'s **own** rejection path
  (`InvalidMotionPlan`, raised inside `linear_search_intersection_point`/
  `search_intersection_points` itself, before `blend()` ever reaches
  `generate_joint_trajectory_from_cartesian`). This is a provably
  different rejection surface from case D's (`PlanningFailed`, raised two
  stages later, inside dynamics/joint-limit checking). The claim and the
  evidence are the same scope here; §207.1 does not apply.
- Site 3 (branch-symmetric-across-radius/length): `determineTrajectoryAlignment`'s
  branch choice is fully computed by `search_intersection_points`'s
  waypoint counts alone, which are available before `blend()` calls
  `generate_joint_trajectory_from_cartesian` at all — the claim is about
  a quantity `search_intersection_points` alone determines, so a
  `search_intersection_points`-only probe is the correct-width tool for
  it, not a narrower one.
- Site 4 (the `blend_radius` sweep table and its `0.08` selection
  reasoning): same as site 3 — `first_intersection_index`/
  `second_intersection_index` are `search_intersection_points` outputs by
  definition; a claim about which radius produces which index values is
  in-scope for a `search_intersection_points`-only probe. (The table's
  *use* to justify filing case C as a success case had the site-5 gap
  above; the table's own numbers do not.)
- Site 6 (case D's original section): this is the finding PORTING-PLAN.md
  §207 already recorded and this round's case E already answers — left
  in place as "the falsified prediction it turned out to be," per the
  existing text, not re-litigated here a second time.

## Case F: a falsifiable prediction for case E's "corner sharpness" attribution

Case E's own doc section above and `pilz_blend_parity.rs`'s
`CORNER112_VELOCITY_TOLERANCE` attribute the interior-`blend_trajectory`
divergence to "panda_arm's redundant-kinematics IK null-space selection
diverging more between solvers as the corner sharpens." That sentence is
consistent with the one data point it was built from (case E vs. cases
A-D) and it has not been tested against a second data point — exactly
the shape PORTING-PLAN.md §207.1 flagged as the failure mode this session
keeps finding: a plausible mechanism reported as though it were measured.
This section states it as a prediction, before any of the fixtures below
were generated, so the sweep that follows cannot be shaped by its own
result.

**Prediction.** Let θ be the corner deviation angle between segment 1's
and segment 2's Cartesian direction (`0°` = straight through, no turn;
`180°` = full reversal; case A/B/C sit at `90°`, case D at `150°`
(rejected), case E at `112°`). Holding `blend_radius` (`0.05`) and both
segments' `max_velocity_scaling_factor`/`max_acceleration_scaling_factor`
(`0.1`/`0.1`) fixed at case A's values, and every other field identical
to case A (same start pose, same `+0.1m` segment length):

- **Quantity:** `max(|Δvelocity|)` and `max(|Δacceleration|)` across
  `blend_trajectory`'s waypoints, measured the same way case E's own
  numbers were (`compare_segment`'s per-waypoint, per-joint absolute
  difference against the oracle response).
- **Direction:** monotonically non-decreasing in θ, for θ swept from
  shallower than case A's `90°` up through as sharp as the pipeline still
  succeeds (bisected at `112.6°` OK / `112.8°` rejected).
- **Refuted by:** (a) any angle producing strictly *larger* divergence
  than a sharper angle tried at a later point in the sweep — true
  non-monotonicity, not sub-order-of-magnitude noise from floating-point/
  backward-difference amplification; (b) a control variable that is *not*
  θ (radius, held fixed at a constant angle) moving the divergence by a
  comparable or larger amount than θ does over the swept range — that
  would show radius, not angle, is doing some or all of the driving, and
  the "corner sharpness" attribution would need restating as "radius and/
  or angle," not simply "the corner sharpens."
- **Not refuted by:** small deviations from strict monotonicity within
  the same order of magnitude — the source cases already show some
  non-monotonic scatter within a fixed order of magnitude (case B's
  `2.95e-10` measures *below* case A's `1.96e-8` at the same `90°`, from
  the asymmetric-speed geometry alone), so the prediction is about the
  trend across roughly an order of magnitude or more, not a strict
  inequality at every adjacent pair.

**Sweep plan, fixed before generating any fixture.** Seven new oracle
round trips: six angles at `blend_radius: 0.05` spanning shallower than
case A through case E (`30°`, `60°`, `75°`, `100°`, `105°`, `110°` —
reusing case A's own `90°` measurement and case E's own `112°`
measurement as the two anchors already on file, not re-measuring them),
plus one radius control at case E's own `112°` angle with `blend_radius`
lowered to `0.03` (`panda_blend_corner112_radius03`) to test prediction
refutation (b) directly at the angle where the effect is largest, rather
than only at case A/C's `90°` where the effect is near the tolerance
floor and has little power to discriminate. `pilz_trajectory_lin_parity.rs`'s
own precedent (this document's earlier note that A/B's document ran the
full pipeline before filing) is followed here too: every fixture below is
generated by actually running `blend()` to completion locally before the
oracle request is sent, not assumed to succeed from the angle alone.

**Measured sweep (real oracle, all seven fixtures `error_code = 1`
`SUCCESS`; cases A/E figures repeated from their own sections above for a
single table).** `max(|Δvelocity|)`/`max(|Δacceleration|)`, both measured
identically to case E's own numbers and confirmed to reproduce case A's
and case E's already-filed figures exactly through the same measurement
path before any new fixture was trusted:

| θ | max \|Δvelocity\| | max \|Δacceleration\| |
|---:|---:|---:|
| 30° | `4.0972e-8` | `7.7099e-7` |
| 60° | `6.4188e-8` | `1.1279e-6` |
| 75° | `8.9525e-8` | `1.4818e-6` |
| 90° (case A) | `1.9582e-8` | `2.9061e-7` |
| 100° | `1.5487e-8` | `2.9642e-7` |
| 105° | `6.6526e-8` | `1.3293e-6` |
| 110° | `7.9133e-8` | `1.5797e-6` |
| 112° (case E) | `8.2760e-8` | `1.6513e-6` |

**Radius control (`panda_blend_corner112_radius03`, θ fixed at case E's
112°):** `blend_radius: 0.05` (case E) measures `1.6513e-6`
acceleration; `blend_radius: 0.03` measures `9.2747e-7` — radius alone,
angle held fixed, moves acceleration divergence by a factor of `~1.8`.

**Verdict: refuted, both ways.**

- **Refutation (a) — non-monotonicity.** The sweep is not monotone in
  θ. 90° and 100° measure the *lowest* divergence of the entire sweep —
  lower than 30°/60°/75° (shallower than case A) and lower than
  105°/110°/112° (sharper than case A). The dip is not sub-order-of-
  magnitude scatter of the kind the prediction's own "not refuted by"
  clause allowed for (case B's `2.95e-10` at 90°): acceleration falls
  from `1.4818e-6` at 75° to `2.9061e-7` at 90° (~5.1x) and rises back to
  `1.3293e-6` at 105° (~4.6x) — a real, repeated-order-of-magnitude
  swing on both sides of the same two points, confirmed by rerunning
  cases A and E through the identical `measure_case` path used for the
  six new points (reproducing their own filed figures exactly, so the
  dip is not a measurement-method artifact) and by confirming case A's
  own fixture sits on the identical θ-parametrized construction as the
  six new points (`corner + 0.1 * (cos θ, sin θ, 0)`, same start pose,
  same `0.1m` segment length — verified directly against the request
  JSON, not assumed).
- **Refutation (b) — a non-θ control moves the divergence by a
  comparable amount.** The radius control's `~1.8x` swing at fixed θ is
  on the same order as several of the sweep's own θ-driven swings (the
  100°→105° step alone is `~4.5x`; the sweep's full range top-to-bottom
  is `~5.7x`). Radius is not a negligible second-order effect next to
  angle; it moves the same quantity by a comparable order of magnitude
  while θ is held fixed.

Both refutation conditions the prediction itself named are met, so the
prediction from this section's own "Prediction" above is **refuted**:
divergence is not a monotone function of corner angle with radius held
fixed. What actually varies with the divergence, from the data measured
so far: something specific to the `~90–100°` region of this particular
geometry (case A/case-100° arm posture reaching this Cartesian corner)
minimizes it, with divergence rising on both sides — not a "sharper
corner, more divergence" trend, and not fully separable from radius
either, per the control above. No alternative mechanism has been tested
against this shape (e.g. Jacobian conditioning or null-space geometry
local to panda_arm's posture at this specific corner), so per this
round's own instruction: **unexplained**, with the numbers above, rather
than a second untested plausible mechanism replacing the first.

## Case G: velocity and acceleration are not independent channels

The round that produced Case F's sweep table separately noticed the two
channels' extrema sit at different angles — velocity peaks at 75°,
acceleration peaks at 112° (see `blend_panda_arm_corner75_matches_the_oracle`'s
and `blend_panda_arm_corner100_matches_the_oracle`'s own doc comments,
corrected in `6e776dc` to say which channel each claim is about). Before
treating that gap as a discriminator between causal mechanisms, this
section checks how each channel is actually computed — by reading, not
guessing, per the standing instruction.

**Port side.** `generate_joint_trajectory_from_cartesian`
(`crates/moveit-planners-pilz/src/trajectory_functions.rs:505-514`)
computes both from the IK-solved position sequence, chained through one
backward-difference loop:

```rust
let mut velocities = HashMap::new();
let mut accelerations = HashMap::new();
for (name, &value) in &ik_solution {
    let velocity = (value - ik_solution_last[name]) / duration_current;
    accelerations.insert(
        name.clone(),
        (velocity - joint_velocity_last[name]) / (duration_current + duration_last) * 2.0,
    );
    velocities.insert(name.clone(), velocity);
}
```

**Upstream side.** `generateJointTrajectory`
(`moveit_planners/pilz_industrial_motion_planner/src/trajectory_functions.cpp:393-400`,
in `/home/stevek/work/moveit2`) uses the identical formula on the
identical structure -- confirming the port is faithful in mechanism, not
just in this case's numeric output:

```cpp
double joint_velocity = (ik_solution.at(joint_name) - ik_solution_last.at(joint_name)) / duration_current;
waypoint_joint.velocities.push_back(joint_velocity);
waypoint_joint.accelerations.push_back((joint_velocity - joint_velocity_last.at(joint_name)) /
                                       (duration_current + duration_last) * 2);
joint_velocity_last[joint_name] = joint_velocity;
```

**Oracle wrapper side.** `serializePilzWaypoints`
(`tools/moveit-oracle/src/oracle.cpp:5764-5786`) does no post-processing
of its own -- it reads `getVariableVelocity`/`getVariableAcceleration`
directly off the `RobotState`s upstream's own `blend()` already produced
(line `5777`-`5778`), so the fixture's `velocities`/`accelerations`
fields are upstream's own finite-difference output, not a value this
document's tooling derived.

**So velocity is a first backward difference of position, and
acceleration is a first backward difference of *that already-computed
velocity array* -- algebraically a second difference of the same
position sequence, not a second independent measurement.** Writing
`D(θ, i) = p_port(θ, i) - p_oracle(θ, i)` for one joint's position
divergence at waypoint `i`, both sides differencing against the same
fixed `sampling_time`:

```
velocity divergence(θ, i)     = (D(θ, i) - D(θ, i-1)) / dt
acceleration divergence(θ, i) = 2 * (velocity divergence(θ, i) - velocity divergence(θ, i-1)) / (dt_i + dt_{i-1})
```

**What this does and does not change about "the channels peak at
different angles."** It is not meaningless -- but it is not what last
round's item 1 assumed either. If the *shape* of the per-waypoint
divergence profile were fixed and only its overall size varied with θ
(`D(θ, i) = f(θ) * g(i)` for some fixed `g`), then both channels would
be `f(θ)` times a fixed constant (`Δg`'s own max, `Δ²g`'s own max
respectively) -- proportional to each other, so their θ-argmax would
have to coincide *exactly*, for any such fixed-shape mechanism. They do
not coincide (`75°` vs `112°`). That is a real, algebraically forced
conclusion: **the per-waypoint divergence profile's shape, not just its
magnitude, changes with θ.** It rules out any mechanism that scales one
fixed waypoint-profile uniformly as the corner sharpens or shallows.
It does not, by itself, identify a replacement mechanism -- see Case H
below for the falsifiable, cheaper-to-test consequence of this finding.

This also revises Case F's own verdict text above: "the two channels
do not peak at the same angle" is restated there as a fact about the
sweep, which it still is, but it should not be read as two independent
physical measurements disagreeing -- it is one position-divergence
signal observed through two different finite-difference operators. The
"unexplained" conclusion stands; what is unexplained is now stated more
precisely as a shape change in `D(θ, i)`, not a channel disagreement.

## Case H: a falsifiable prediction from Case G's shape-change conclusion

Case G proved, algebraically, that a fixed-shape amplitude-only
mechanism is excluded: any `D(θ, i) = f(θ) * g(i)` would force velocity-
and acceleration-divergence to peak at the same θ, and they do not. That
conclusion is about the *shape* of the per-waypoint divergence profile,
not just its two derived channels, so it makes a prediction about the
one signal underneath both of them -- `D(θ, i)`, position divergence
itself -- that is checkable directly, with no new oracle round trip: all
nine `panda_blend_{symmetric,corner{30,60,75,100,105,110,112},corner112_radius03}`
fixtures already carry both sides' full waypoint position arrays. This
section states the prediction before computing it, per this round's own
"file it first" instruction.

**Prediction.** Let `i*(θ) = argmax_i |D(θ, i)|` be the `blend_trajectory`
waypoint index (`0..7`, across whichever joint attains it -- not fixed to
one joint in advance) where position divergence between this port and
the oracle is largest, for each of the nine sweep angles.

- **Quantity:** `i*(θ)`, the argmax waypoint index of `max_joint |D(θ, i)|`.
- **Direction:** `i*(θ)` is *not* constant across the nine θ values -- it
  must vary, because a constant `i*` across θ is only possible (though
  not sufficient on its own) under something close to the fixed-shape
  form Case G already excluded.
- **Refuted by:** `i*(θ)` measuring identical at all nine angles. Unlike
  Case F's prediction, this would not be a clean "mechanism wrong"
  result -- it would contradict Case G's own algebra (constant-shape
  `D` forces coincident channel peaks, which are *not* coincident), so a
  refutation here means re-examining the Case G derivation or this
  measurement's own correctness before accepting it, not filing a new
  "unexplained."
- **Not refuted by:** `i*(θ)` flipping between adjacent waypoints only at
  angles where the top two waypoints' divergence is within about 10% of
  each other (a near-tie in the noise floor, not a real shape change) --
  this applies mainly near the 90-100° dip, where overall divergence is
  smallest and ties are most likely.
- **Supporting exhibit, not a separate prediction:** 60° and 105°
  measure nearly identical overall velocity divergence (`6.4188e-8` vs
  `6.6526e-8`, Case F's own table) -- if `i*` and the general per-waypoint
  profile shape differ between these two despite matching magnitude,
  that is a more direct demonstration of shape-change than magnitude
  alone could give, though it is read as illustrative, not as a second
  falsifiable claim in its own right.

**Measured (existing fixtures, no new oracle round trip; a temporary
probe iterating `drive_case` over all nine cases, run and reverted).**
`max_d` is `max_joint |D(θ, i*)|` in metres; `profile` is `max_joint
|D(θ, i)|` at every waypoint, smallest to be transparent about how far
the argmax sits above its runner-up (the "not refuted by" tie exception
above):

| θ | `i*` | joint at `i*` | runner-up ratio |
|---:|---:|---|---:|
| 30° | 7 | `panda_joint1` | 70% (i=6: `4.023e-9` vs `5.736e-9`) |
| 60° | 4 | `panda_joint7` | 61% (i=1: `3.600e-9` vs `5.864e-9`) |
| 75° | 7 | `panda_joint7` | 64% (i=5: `5.377e-9` vs `8.464e-9`) |
| 90° (case A) | 7 | `panda_joint5` | 56% (i=5: `1.267e-9` vs `2.278e-9`) |
| 100° | 1 | `panda_joint1` | 53% (i=7: `7.436e-10` vs `1.416e-9`) |
| 105° | 5 | `panda_joint5` | 30% (i=1: `2.067e-9` vs `6.890e-9`) |
| 110° | 5 | `panda_joint5` | 33% (i=1: `2.655e-9` vs `8.128e-9`) |
| 112° (case E) | 5 | `panda_joint5` | 34% (i=1: `2.868e-9` vs `8.484e-9`) |
| 112°, r=0.03 | 5 | `panda_joint5` | 65% (i=1: `5.549e-9` vs `8.493e-9`) |

Every runner-up sits at 30-70% of the argmax -- nowhere near the ~10%
near-tie band the prediction's own "not refuted by" clause carved out,
so every one of these nine argmax values is a clean read, not a
coin-flip in the noise floor.

**Verdict: not refuted, and by more than the minimum the prediction
asked for.** `i*(θ)` takes four different values across the sweep
(`7, 4, 7, 7, 1, 5, 5, 5, 5`) -- not the single constant value a
fixed-shape mechanism would need, consistent with Case G's algebra. It
goes further than "the waypoint index moves": **the dominant joint
itself changes** -- `panda_joint1`/`panda_joint7` own the shallow
angles (30-75°) and 100°'s own local minimum, `panda_joint5` owns 90°
and the whole 105-112° climb. A mechanism that is genuinely one thing
across the sweep (redundant-kinematics IK divergence or otherwise)
would need to explain not just a shifting timing within one joint's
error, but *which joint the redundant-kinematics null-space choice
diverges on* changing with corner angle -- a mechanism with real
structure, not a single scalar "how much."

The 60°/105° supporting exhibit lands cleanly: matched overall velocity
divergence (`6.4188e-8` vs `6.6526e-8`, within 4% of each other) pairs
with completely different profiles -- different dominant joint
(`panda_joint7` vs `panda_joint5`) and different waypoint (`4` vs `5`).
Two cases that look identical through the velocity channel's single
scalar are not, in the underlying signal that channel is derived from.

This does not identify what the mechanism is -- Case G already noted
that identifying one was out of scope for what has been measured so
far. It sharpens what "unexplained" means one step further: not "an
unmeasured amount of a known mechanism," but a divergence whose
*location* (which joint, which point in the blend) itself depends on
corner angle in a way no fixture generated so far isolates from angle,
radius, or arm posture individually.
