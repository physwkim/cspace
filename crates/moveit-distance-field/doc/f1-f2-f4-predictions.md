# F1/F2/F4 predictions, committed before the `mode` branch lands

Companion to `oracle-request-hybrid-collision-env-distance-field.md` (read
that first for the case table, the `mode` field shape, and the refuting
result stated per case). CLAUDE.md §207: a prediction written after the
oracle answers cannot test whether the question was well-posed: this
document exists so F1/F2/F4 have a committed, concrete answer on file
before `oracle.cpp`'s `mode`/`"robot_only"` branch exists to run them
against. `tools/moveit-oracle/` is not this worker's file; nothing here
was run against that branch — everything below is derived from upstream
source (`collision_env_distance_field.cpp`, `collision_distance_field_types.cpp`)
and from real geometry already returned by the current, unmodified
`group_state_representation` op (stamp `700e7be54cb0a61f`, superseding
`043ed31a2186fe4e`; the sphere-center/radius numbers below were re-queried
against `700e7be54cb0a61f` after PORTING-PLAN.md §212's rebuild — this
op's `gradients: true`/no-`mode` path is unchanged by that rebuild, and
`verify-fixture-replay.sh`'s own 52/52 `identical` result is the evidence
for that, not an assumption made here).

## Shared derivation

`mode: "robot_only"` selects `checkRobotCollision` →
`getEnvironmentCollisions` (`collision_env_distance_field.cpp:1561-1643`).
With `req.contacts = true` (every case below sets `contacts: true`, since
the verdict is the object of the comparison, not incidental — see the
parent doc's "Response shape" section), each robot link's spheres are
tested via the boolean+contacts-list overload of `getCollisionSphereCollision`
(`collision_distance_field_types.cpp:238-271`):

```cpp
double dist = distance_field->getDistanceGradient(p.x(), p.y(), p.z(), ...);
if (maximum_value > dist && (sphere_list[i].radius_ - dist > tolerance))
  /* collision */
```

- `maximum_value` = `max_propogation_distance_` = `DEFAULT_MAX_PROPOGATION_DISTANCE`
  = `0.25` (`collision_env_distance_field.hpp:55`, confirmed the oracle's
  `CollisionEnvDistanceField env(model_, world)` construction in
  `groupStateRepresentation` uses no explicit override).
- `tolerance` = `collision_tolerance_` = `DEFAULT_COLLISION_TOLERANCE` =
  `0.0` (`:54`, same all-default-args confirmation).
- The field is a `PropagationDistanceField` with a hard propagation cutoff:
  a point farther than `0.25` m from every occupied voxel reads back
  `dist >= 0.25`, at which point `maximum_value > dist` is false and the
  sphere cannot register a collision regardless of its own radius. This is
  why "clearly clear" only needs geometric margin past `0.25 + object
  radius`, not an exact field simulation.
- On a hit, `getEnvironmentCollisions` sets `res.collision = true`,
  `gsr->gradients_[i].collision = true`, and pushes one `Contact` per
  colliding sphere (capped at `max_contacts_per_pair`, default `1`) keyed
  by `(body_name_1, "environment")` into `res.contacts`
  (`:1596-1629`). This is the boolean-collision path, **not**
  `getCollisionGradients`/`getEnvironmentProximityGradients` — that
  function computes the same `dist` but never writes its result into
  `gradients_[i].collision` anywhere in its body (confirmed by reading
  `:1645-1681` in full), which is exactly why F1/F2/F4 use `mode:
  "robot_only"` + `contacts: true` rather than `"gradients": true` (the
  two are mutually exclusive on this op besides — `oracle.cpp:3686-3689`).
- `Contact` JSON shape, from `contactToJson` (`oracle.cpp:2372-2383`,
  reused by `allContactsToJson` for this op's `contacts` field,
  `oracle.cpp:3774`): `body_name_1`, `body_type_1`, `shape_kinds_1`,
  `body_name_2`, `body_type_2`, `shape_kinds_2`, `depth`. **No `pos`
  field** — an earlier draft of this document assumed one from the
  contact's `pos` member; `contactToJson` does not serialize it, so no
  prediction below states a contact position.
- `body_name_2`/`body_type_2` are always the literal sentinel `"environment"`
  / `world_object` (`:1615-1616`), never the specific object id that
  caused the hit — there is no per-object identity kept once points are
  merged into the shared distance field. `shapeKindsFor(WORLD_OBJECT,
  "environment", world)` returns `null` because `world.hasObject("environment")`
  is false for every case here (none of the requested object ids below is
  literally `"environment"`) — `oracle.cpp:2287-2289`.
- `depth` is always `0.0`: `getEnvironmentCollisions` never sets
  `Contact::depth`, so it carries only its default member initializer,
  not a real penetration measurement.
- `shape_kinds_1` for a robot link is `model_->getLinkModel(name)->getShapes()`
  mapped through `shapeTypeName` (`oracle.cpp:2275-2280`, `:268-286`).

All three cases below reuse `group: "right_arm"`, `joint_values: {}`
(the default pose), `use_acm: true` — ACM has no bearing on this path
(`checkRobotCollision` never touches `self_collision_enabled_`/
`intra_group_collision_enabled_`; those are read only by
`getSelfCollisions`/`getIntraGroupCollisions`, never called on this
branch), included only for parity with F2/F3/F4 sharing one request
shape family. Sphere-center/radius numbers are the real values the
current (unmodified-by-§212) `group_state_representation` op returns for
`right_arm` at `joint_values: {}` — re-queried this round against
`700e7be54cb0a61f`, `gradients: true`, no `objects`:

| link | bounding center | bounding radius |
|---|---|---|
| `r_shoulder_pan_link` | `[-0.0434, -0.1823, 0.7909]` | `0.3476` |
| `r_upper_arm_link` | `[0.3549, -0.1874, 0.9454]` | `0.3573` |
| `r_wrist_roll_link` | `[0.5669, -0.1885, 1.2763]` | `0.0208` |
| `r_wrist_roll_link` sphere 0 center | `[0.5713817184580412, -0.188, 1.2808756763013027]` | `0.02070292775642306` |
| `r_gripper_palm_link` | `[0.5383, -0.1885, 1.3176]` | `0.0574` |

(Full 16-link table was queried; only the rows load-bearing for the
predictions below are reproduced here.)

## F1: clearly clear

**Request:**

```json
{
  "op": "group_state_representation",
  "group": "right_arm",
  "joint_values": {},
  "use_acm": true,
  "mode": "robot_only",
  "contacts": true,
  "objects": [
    { "id": "env_sphere_clear", "pose": "translation [0.55, 1.0, 1.28], identity rotation",
      "shape": { "type": "sphere", "radius": 0.05 } }
  ]
}
```

**Why this placement is clear:** the field distance `getCollisionSphereCollision`
reads for a robot sphere at `dist_c` from the object's own center is
`dist_c - object_radius` (the object occupies a full sphere of radius
`0.05` around its pose, not a point); the no-collision condition is
`dist >= robot_sphere_radius` (since every relevant radius here is well
under `max_propogation_distance_ = 0.25`, so that cap never binds — see
"Shared derivation"). Bounding a specific collision sphere's own radius
by its link's bounding radius is conservative in the safe direction
(every real collision sphere is `<=` its link's bounding radius), so a
sufficient condition for the whole link is:

```
(object_center to link_bounding_center distance) >= object_radius + 2 * link_bounding_radius
```

(the `2x` covers both the object's own radius being subtracted from the
center-to-center distance, and an individual collision sphere sitting up
to `bounding_radius` off the bounding center, toward the object, doubly
conservative rather than computing each link's real per-sphere centers).
Every `right_arm` link's bounding sphere sits at `y` between `-0.222`
and `-0.158`; placing the object at `y = 1.0` leaves `>= 1.18` m to any
bounding center. The tightest case, `r_upper_arm_link` (bounding radius
`0.3573`, center-to-object-center `1.249` m — computed as the full 3D
distance, not `y` alone), needs `0.05 + 2*0.3573 = 0.765` m and has
`1.249` m, a `0.48` m margin to spare. `r_shoulder_pan_link` (bounding
radius `0.3476`, distance `1.410`) needs `0.745` m and has `1.410`, a
`0.66` m margin. No other link's bounding radius exceeds `0.36`, so
neither is the binding case.

**Predicted response:**

- `collision`: `false`
- `contacts`: `[]` (empty array)
- every link with `has_link_decomposition: true`: `gradient.collision: false`

**Refuting result** (restated from the parent doc): any link reporting
`gradient.collision: true`, or `collision: true` at the top level,
refutes "clearly clear" — the placement needs re-posing, not a report of
a port defect, since F1's geometry is chosen by this document, not by
upstream.

## F2: posed to overlap `r_wrist_roll_link`

**Request:** identical shape to the existing, already-committed
`group_state_representation_gradients_request.json` id 2's object
(`env_sphere`, radius `0.05`, translation
`[0.5713817184580412, -0.188, 1.2808756763013027]` — `r_wrist_roll_link`'s
own first collision-sphere center, radius `0.0207`), but requested
through the new `mode: "robot_only"` + `contacts: true` path instead of
`gradients: true`:

```json
{
  "op": "group_state_representation",
  "group": "right_arm",
  "joint_values": {},
  "use_acm": true,
  "mode": "robot_only",
  "contacts": true,
  "objects": [
    { "id": "env_sphere", "pose": "translation [0.5713817184580412, -0.188, 1.2808756763013027], identity rotation",
      "shape": { "type": "sphere", "radius": 0.05 } }
  ]
}
```

**Why this is known to overlap, not just guessed:** the existing
`group_state_representation_gradients_response.json` id 2 already shows,
for this exact object at this exact pose, `r_wrist_roll_link`'s
`gradient.distances: [0.0, 0.0]` and `gradient.types: [3, 3]` (i.e.
`ENVIRONMENT` on both of its two collision spheres) — a proven, already
oracle-returned `dist = 0.0` at this placement, not a placement newly
proposed by this document. `getCollisionSphereCollision`'s condition,
with `dist = 0.0`, `tolerance = 0.0`: `0.25 > 0.0` (true) and
`0.0207 - 0.0 > 0.0` (true, since the link's own sphere radius is
positive) — both required conjuncts hold, so `checkRobotCollision`'s
boolean-collision path (not yet exercised by that existing fixture, which
only ever ran the gradients path) must also report a hit here.

**Predicted response:**

- `collision`: `true`
- `contacts`: two entries — two distinct `(body_name_1, "environment")`
  pairs, `max_contacts_per_pair` defaulting to `1` caps each pair's own
  list at one, not the number of pairs:
  ```json
  [
    {
      "body_name_1": "r_wrist_roll_link",
      "body_type_1": "robot_link",
      "shape_kinds_1": ["mesh"],
      "body_name_2": "environment",
      "body_type_2": "world_object",
      "shape_kinds_2": null,
      "depth": 0.0
    },
    {
      "body_name_1": "r_gripper_palm_link",
      "body_type_1": "robot_link",
      "shape_kinds_1": ["mesh"],
      "body_name_2": "environment",
      "body_type_2": "world_object",
      "shape_kinds_2": null,
      "depth": 0.0
    }
  ]
  ```
  (`shape_kinds_1: ["mesh"]` on both — each link's only `<collision>`
  geometry in `fixtures/pr2.urdf` is a `<mesh>`, confirmed by reading
  both link blocks directly, not assumed from the visual mesh. The
  second entry follows from "Open uncertainty" below, resolved before
  this document was committed, not after.)
- `r_wrist_roll_link`'s and `r_gripper_palm_link`'s `gradient.collision`:
  `true`. Every other link's `gradient.collision`: `false` (no other
  link's bounding sphere comes within `0.30` m of `[0.5714, -0.188,
  1.2809]`, the sufficient no-collision distance derived in "Shared
  derivation").

**Refuting result:** `collision: false`, or `collision: true` on any link
other than `r_wrist_roll_link`/`r_gripper_palm_link`, directly refutes
`getEnvironmentCollisions`'s port — the first refutation possible for
this method, since no prior fixture ever drove its boolean-collision
branch against non-empty geometry.

**Open uncertainty, stated rather than hidden:** whether
`r_gripper_palm_link` also registers is not fully closed by a
bounding-sphere-only check — its bounding sphere (radius `0.0574`) is
only `0.0483` m from the object center, i.e. the *bounding* spheres
overlap, which says nothing on its own about the individual *collision*
spheres. Resolved using `r_gripper_palm_link`'s real per-sphere centers
(already on hand from the same query: two spheres at
`[0.5406049056116286, -0.1709882693787416, 1.3133223412959347]` and
`[0.5406049056116286, -0.2049912154475848, 1.3133223412959347]`, radius
`0.05264214509414166` each). Distance from the object center
`[0.5713817184580412, -0.188, 1.2808756763013027]` to the nearer of the
two, `[0.5406049056116286, -0.2049912154475848, 1.3133223412959347]`:
`sqrt(0.03078^2 + 0.01699^2 + (-0.03245)^2) = 0.04784` m center-to-center.
The field distance the collision check actually reads is
`center_to_center - object_radius = 0.04784 - 0.05 = -0.00216`, i.e. the
robot sphere's own center already falls inside the object's volume, so
`dist` clamps to `0` (there is no negative distance; the query point is
itself an occupied voxel). `radius_robot_sphere(0.05264) - dist(0) =
0.05264 > tolerance(0)` — collision. **This changes the prediction:
expect `r_gripper_palm_link.gradient.collision: true` as well**, with a
second contact entry `{"body_name_1": "r_gripper_palm_link", ...}` in
`contacts`. Left both the original bounding-sphere-only question and
this resolution in the document rather than silently replacing one with
the other, since the gap between them (a bounding-sphere check is not a
per-sphere check, and center-to-center distance is not field distance
without subtracting the object's own radius) is itself worth recording
as the thing an earlier pass of this same derivation got wrong.

## F4: two objects, one overlapping, one not

**Request:** F1's clear object and F2's overlapping object in the same
request:

```json
{
  "op": "group_state_representation",
  "group": "right_arm",
  "joint_values": {},
  "use_acm": true,
  "mode": "robot_only",
  "contacts": true,
  "objects": [
    { "id": "env_sphere", "pose": "translation [0.5713817184580412, -0.188, 1.2808756763013027], identity rotation",
      "shape": { "type": "sphere", "radius": 0.05 } },
    { "id": "env_sphere_clear", "pose": "translation [0.55, 1.0, 1.28], identity rotation",
      "shape": { "type": "sphere", "radius": 0.05 } }
  ]
}
```

**Predicted response:** the union of F2's result with F1's non-effect —
`env_sphere_clear` contributes no occupied voxel within range of any
`right_arm` link (same margin argument as F1, unaffected by
`env_sphere`'s presence: `add_points_to_field` merges point sets, it does
not let one object's points suppress another's), so the merged field's
per-sphere nearest-distance is identical to F2's single-object field for
every robot collision sphere.

- `collision`: `true`
- `contacts`: same entries as F2 (`r_wrist_roll_link` and, per the
  correction above, `r_gripper_palm_link`), no third entry attributable
  to `env_sphere_clear`
- `r_wrist_roll_link.gradient.collision` and
  `r_gripper_palm_link.gradient.collision`: `true`; every other link's:
  `false`
- Because `body_name_2` is always the fixed sentinel `"environment"`
  regardless of which object actually caused a hit (see shared
  derivation above), F4's `contacts` cannot distinguish "hit `env_sphere`"
  from "hit `env_sphere_clear`" by content alone — only by the fact that
  no contact should appear at all near `env_sphere_clear`'s position.
  This is exactly why F1's own margin has to hold independently: F4
  cannot verify "the clear sphere stayed clear" from `contacts` content,
  only from the *absence* of any contact/`gradient.collision` on a link
  whose spheres are nowhere near `env_sphere_clear`.

**Refuting result:** any contact or `gradient.collision: true` on a link
whose nearest collision sphere is not within `0.30` m of either object
center is direct evidence of cross-talk between environment objects in
the per-object accumulation this port's `add_points_to_field` sequence
performs — the one failure mode F1/F2 individually cannot exercise, since
each has only one object in its world.
