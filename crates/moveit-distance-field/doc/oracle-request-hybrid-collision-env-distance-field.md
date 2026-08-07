# Oracle request: `CollisionEnvHybrid`'s four `*DistanceField` entry points

Not an implementation — request document for the human orchestrator, same
convention as `crates/moveit-planners-pilz/doc/oracle-request-pilz-blend.md`
(read that document first for the shared vocabulary this one reuses:
`objects`, tolerance-measured-not-guessed, exact-vs-tolerance field
splits). `tools/moveit-oracle/` is not this worker's file.

## The gap

PORTING-PLAN.md §186 measured `CollisionEnvHybrid`'s exclusion (inherited
from `CollisionEnvFCL`, assumed unportable under D4.5's FCL removal) and
found it wrong by direct count: 18 of 22 members are pure passthroughs to
`cenv_distance_` (this crate's own `DistanceFieldCollisionCache`), 3 are
constructor base-inits carrying nothing FCL-specific, and the one real
FCL-coupled member (`setWorld`, rebuilding FCL's broadphase cache) has no
analogue to port at all because `ParryCollisionEnv` keeps no such cache
(`parry.rs`, `world_bodies` recomputed fresh every `check_*` call). Round
29 landed the port as `HybridCollisionEnv` (`69044f5`, `5e18441`).

Landing the file is not the same claim as landing correctness. Every one
of `HybridCollisionEnv`'s four `*DistanceField` methods is a one-line
forward — confirmed directly against
`collision_env_hybrid.cpp:75-184` (quoted in full below) — to a
`DistanceFieldCollisionCache` method this crate already ports and, for
three of the four, already oracle-tests **indirectly**, through
`crates/moveit-distance-field/tests/collision_env_distance_field_parity.rs`
calling `DistanceFieldCollisionCache`'s methods directly rather than
through `HybridCollisionEnv`. Whether that indirect coverage actually
closes the gap for each method — rather than merely calling the same
function name with different, weaker inputs — is not something a `grep`
for the call site can answer; it required reading what each existing
fixture's environment field actually contains. It does not close it
uniformly. The breakdown below is that reading, not an assumption.

## Upstream symbol: public, already known

```cpp
// collision_env_hybrid.hpp:88-140
void checkSelfCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                      const moveit::core::RobotState& state) const;               // + 3 overloads (gsr/acm/acm+gsr)
void checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                  const moveit::core::RobotState& state) const;                    // + 3 overloads
void checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                       const moveit::core::RobotState& state) const;                // + 3 overloads
void getCollisionGradients(const CollisionRequest& req, CollisionResult& res,
                            const moveit::core::RobotState& state, const AllowedCollisionMatrix* acm,
                            GroupStateRepresentationPtr& gsr) const;

// collision_env_hybrid.cpp:75-184 — every overload of the first three, and getCollisionGradients, in full:
void CollisionEnvHybrid::checkSelfCollisionDistanceField(...) const { cenv_distance_->checkSelfCollision(req, res, state[, acm][, gsr]); }
void CollisionEnvHybrid::checkCollisionDistanceField(...)     const { cenv_distance_->checkCollision(req, res, state[, acm][, gsr]); }
void CollisionEnvHybrid::checkRobotCollisionDistanceField(...) const { cenv_distance_->checkRobotCollision(req, res, state[, acm][, gsr]); }
void CollisionEnvHybrid::getCollisionGradients(...)            const { cenv_distance_->getCollisionGradients(req, res, state, acm, gsr); }
```

No new logic anywhere in these twelve method bodies (four names × up to
four overloads each, collapsed to one entry point per name on this port's
side the same way this crate's own `HybridCollisionEnv` module doc
already argues for the other collapsed arities). `cenv_distance_` is a
`CollisionEnvDistanceField`, upstream's own base class for this crate's
`DistanceFieldCollisionCache`. So this request is really about four
`DistanceFieldCollisionCache` methods
(`check_self_collision`/`check_collision`/`check_robot_collision`/
`get_collision_gradients`, `collision_env_distance_field.rs:1353,1406,
1479,1527`) and one new piece of glue
(`HybridCollisionEnv::build_env_distance_field`,
`collision_env_hybrid.rs:324`, which upstream does not have as a
separate step at all — see "What `build_env_distance_field` replaces"
below) — not new upstream call sequences to discover, but an audit of
which of the four already has real oracle evidence behind it.

## What's already covered, and what is not — read per fixture, not per call site

`tools/moveit-oracle/src/oracle.cpp` already has two ops that construct a
`CollisionEnvDistanceField` directly and drive these same four methods:
`distance_field_cache_entry` (→ `checkSelfCollision`) and
`group_state_representation` (→ `checkCollision` or, with
`"gradients": true`, `getCollisionGradients`). Both already accept
`request["objects"]` (`addRequestObjects`, `oracle.cpp:2621-2631`, single-shape
`{id, pose, shape}` schema shared with the `collision` op) to populate a
real `World` before constructing the env — this is not new plumbing to
request, it exists and is wired today.

- **`check_self_collision_distance_field`** (→ `checkSelfCollision`):
  fully covered. `checkSelfCollision` never reads the environment at all
  (self + intra-group only — `collision_env_distance_field.cpp:177-200`),
  and `check_self_collision_matches_the_oracle_with_contacts_and_attached_bodies`
  already compares this exhaustively against `distance_field_cache_entry`'s
  fixture. **No new oracle case needed for the underlying logic.**

- **`get_collision_gradients`**: covered, including the one path that
  matters — the environment-gradient branch
  (`get_environment_proximity_gradients`) is not merely called, it is
  called against a *real, non-empty* world object.
  `group_state_representation_gradients_request.json`'s id `2` sets
  `"objects": [{"id": "env_sphere", "shape": {"type": "sphere", "radius":
  0.05}, ...}]`, and
  `group_state_representation_gradients_matches_the_oracle`
  (`collision_env_distance_field_parity.rs:1745`) builds its own
  `PropagationDistanceField` from that same object via `World::new()` →
  `collision_object_point_decomposition` → `add_points_to_field` — **the
  exact sequence `HybridCollisionEnv::build_env_distance_field` runs
  against `self.parry.world()`.** That sequence is proven correct against
  upstream already; what is not yet proven is that `HybridCollisionEnv`'s
  own wrapper (which holds the `World` inside `self.parry` instead of
  taking a `&dyn DistanceField` parameter directly) reaches the same
  place. **No new oracle case needed for the underlying logic** — the
  remaining gap here is Rust-side only (see "How this port will use the
  response").

- **`check_collision_distance_field`** (→ `checkCollision`): **partially
  covered, and the missing part is the one this document exists for.**
  `checkCollision` tries self, then intra-group, then environment, and
  short-circuits on the first hit
  (`collision_env_distance_field.cpp:1389-1412`). The only existing
  oracle-compared `check_collision` call,
  `check_collision_matches_the_oracle_with_contacts_and_attached_bodies`
  (`collision_env_distance_field_parity.rs:1468`), passes `empty_env` — a
  `PropagationDistanceField` with zero points added, by construction,
  every fixture id. An empty field can only ever answer "no collision";
  `get_environment_collisions`'s actual comparison logic (distance
  thresholding, contact recording, `EnvironmentType`) has never been
  exercised through this op against real geometry. This is not visible
  from a `grep` for `checkCollision`/`get_environment_collisions` call
  sites — both are called on every fixture id, with a real return value
  — only from reading what the field they're called against actually
  contains.

- **`check_robot_collision_distance_field`** (→
  `CollisionEnvDistanceField::checkRobotCollision`): **zero coverage of
  any kind.** `rg -n 'checkRobotCollision' tools/moveit-oracle/src/oracle.cpp`
  does have hits (`oracle.cpp:2491`, `:5488`) — but both are
  `collision_detection::CollisionEnvFCL::checkRobotCollision`, a
  same-named but entirely distinct virtual method on a different backend
  class, called from the `collision` and `octree_in_world` ops as ground
  truth for `moveit-collision::ParryCollisionEnv`, not for this crate.
  Confirmed by reading both call sites: both construct `CollisionEnvFCL
  env(model_, world)`, never a `CollisionEnvDistanceField`. Neither
  `distance_field_cache_entry` nor `group_state_representation` — the two
  ops that actually construct a `CollisionEnvDistanceField` — ever calls
  its `checkRobotCollision`; both only ever reach
  `checkSelfCollision`/`checkCollision`/`getCollisionGradients`. This is
  the reading this section opened with: a name match at the call-site
  level is not evidence of coverage for *this* method, on *this* class.
  Unlike `checkCollision`, `checkRobotCollision`
  skips self- and intra-group collision entirely and goes straight to
  `getEnvironmentCollisions`
  (`collision_env_distance_field.cpp:1454-1471`) — a genuinely distinct
  code path (`generateCollisionCheckingStructures`'s last argument is
  `false` for the no-acm overload, `true` for the acm overload, vs.
  `checkCollision`'s always-`true`; this only affects whether
  `dfce->distance_field_` — the **self**-collision field, unrelated to the
  environment field the response actually reads — gets built, so it does
  not change what the response below would contain either way). This is
  the one case that needs new C++, not just a new fixture — see "Does the
  existing oracle image serve this" below.

## What `build_env_distance_field` replaces

**Note (post-round-31 correction, kept for the historical record; this
document's own conclusions below are unaffected):** the "no observer hook"
claim in this section's original text was refuted. `moveit_collision::World`
has no *callback* registration, but every mutator returns a `Notification`
describing what changed, and `World::all_objects_as_notifications` exists
to replay every current object to a new consumer — the mechanism this
document assumed did not exist. `HybridCollisionEnv::build_env_distance_field`
(the per-call rebuild described below) no longer exists as of this crate's
own `mutate_world`/`apply_notification`/`env_field` redesign
(`collision_env_hybrid.rs`), which maintains one persistent environment
field incrementally, matching upstream's own `notifyObjectChange`. This
does not change any conclusion below about oracle coverage: the sequence
`self.parry.world().iter()` → `collision_object_point_decomposition` →
`add_points_to_field` this section describes is still exactly what runs,
just once per mutation instead of once per call, so the oracle-coverage
reasoning that follows still applies unchanged to `apply_notification`
in place of `build_env_distance_field`.

Upstream's `CollisionEnvDistanceField` maintains
`distance_field_cache_entry_world_` incrementally, updated by `setWorld`/
`addToObject`/`removeObject` observer callbacks
(`collision_env_distance_field.hpp:59-309`). `HybridCollisionEnv`'s
original port rebuilt the same information fresh on every call instead:
`self.parry.world().iter()` → `collision_object_point_decomposition` (per
object) → `add_points_to_field`. This is not new logic to verify against
upstream on its own terms — it is the identical sequence
`group_state_representation_gradients_matches_the_oracle` already
performs by hand to build its comparison field (cited above), and
`collision_object_point_decomposition` itself has its own dedicated
oracle op (`collisionObjectPointDecomposition`, `oracle.cpp:4304-4333`) already
exercised by `collision_object_point_decomposition_parity.rs`. What is
untested is specifically the *reachability* claim — that walking a real
multi-object `World` through this exact sequence and handing the result
to `check_collision`/`check_robot_collision`/`get_collision_gradients`
produces the same numbers as upstream's incrementally-maintained field —
not the sequence's individual steps, each already covered elsewhere.

## Request JSON shape: one new case on `group_state_representation`

Reuses `group_state_representation`'s existing request shape in full
(`group`, `joint_values`, `use_acm`, `attached_bodies`, `objects` — see
that op's own doc comment, `oracle.cpp:3985-4148`) plus one new field:

```json
{
  "op": "group_state_representation",
  "group": "right_arm",
  "joint_values": { "...": "any valid pr2 right_arm configuration" },
  "use_acm": true,
  "mode": "robot_only",
  "objects": [
    { "id": "env_sphere", "pose": "...row-major 4x4...",
      "shape": { "type": "sphere", "radius": 0.05 } }
  ]
}
```

- `mode` (new field, optional, defaults to the existing `checkCollision`
  behavior): `"robot_only"` selects `checkRobotCollision` instead of
  `checkCollision`. Named for what upstream's own method is called
  (`checkRobotCollision`, i.e. "collision against the robot's
  environment only," not a `self`/`robot` naming this document invented)
  rather than a boolean, so a third mode is not a breaking shape change
  later. Mutually exclusive with `"gradients": true` the same way that
  field is already mutually exclusive with `"contacts": true`
  (`oracle.cpp:4173-4176`) — `checkRobotCollision` and
  `getCollisionGradients` are different upstream calls, not two flags on
  one call.
- Every other field: unchanged from the existing op. `objects` is
  requested non-empty for every case below — a `checkRobotCollision`
  request against an empty world can only ever return "no collision,"
  which the existing (fully covered) `check_self_collision` cases already
  establish as reachable; the entire point of this request is the
  non-trivial environment-collision numbers only a populated world
  produces.

## Cases needed: 4

| case | `group` | `mode` | `objects` | what it closes |
|---|---|---|---|---|
| F1 | `right_arm` | `robot_only` | one sphere, clearly clear of the arm at its default pose | `checkRobotCollision`'s no-collision path; first-ever call of this method through any oracle op |
| F2 | `right_arm` | `robot_only` | one sphere, posed to overlap a known link at its default pose | `checkRobotCollision`'s collision-found path — `getEnvironmentCollisions`'s actual distance/contact-recording logic, still entirely unverified even after F1 |
| F3 | `right_arm` | (default, i.e. `checkCollision`) | one sphere, posed to overlap a link, **and** at a joint state to be chosen so self/intra-group collision is absent (verify against the response's per-link `gradient.collision`/`gradient.types` fields, dumped unconditionally regardless of `mode` — do not assume any currently-committed fixture's joint values already satisfy this; none was confirmed to when this document was written, see below) | `checkCollision`'s environment branch actually firing — every existing `checkCollision` fixture uses `empty_env`, so this branch has a return value today but has never been checked against one |
| F4 | `right_arm` | `robot_only` | two spheres, one overlapping, one not, at the same request | `getEnvironmentCollisions`'s per-object loop and `max_contacts`/`contacts` interaction with more than one environment body — F1-F3 each use one object, so this is the first case with more than a single-body environment |

Only F3 needs **no C++ change** — `mode` absent selects the exact path
`group_state_representation` already runs today, so it can be filed
immediately, with zero waiting, as a new fixture entry with non-empty
`objects` under the existing default mode. F1, F2, and F4 all set
`"mode": "robot_only"`, a field that does not exist on the op yet, so
all three need the dispatch-branch addition described in "Does the
existing oracle image serve this" below before they can run — not just
F1/F2 as an earlier draft of this table's surrounding prose said before
the mismatch was caught rereading it against the table itself. Listed
together in one table anyway, rather than F3 split into its own
document, because its "refuting result" is the same one this whole
request is about (an untested environment branch), and splitting it
would duplicate the case-table format for one row.

On F3's joint-state requirement: `group_state_representation_request.json`
(ids 1-4, no `contacts` field) and
`group_state_representation_contacts_request.json` id 3 (`contacts:
false`, `joint_values: {}`) were both checked as candidates while
drafting this document. Neither is usable as cited: the former's
per-link `gradient.collision` values were not actually read against
upstream's `getSelfCollisions`/`getIntraGroupCollisions` "false branch
returns on first hit and writes neither" short-circuit note in this same
op's own doc comment (`oracle.cpp:4100-4109`) — meaning "every link
reports `collision: false`" in a `contacts`-absent response is
ambiguous between "no self-collision" and "a self-collision exists but
the per-link flag was never reached," not evidence either way without
rereading the actual upstream branch condition against the actual joint
values, which was not done here. Whoever files F3 should pick a state
and confirm no self-collision the reliable way: a `contacts: true`
request at the same joint values reporting `collision: false`.

**That requirement is unsatisfiable, and the reason is upstream's own
fixture — do not go looking for a joint state.** `right_arm` cannot be
posed self/intra-collision-free under this repo's `pr2.srdf`: 230
sampled states (30 whole-model, 200 `right_arm`-scoped draws inside real
joint limits) every one reported self-collision, and
`collision_env_distance_field.cpp:760-850` sets
`self_collision_enabled_`/`intra_group_collision_enabled_` purely from
`acm->getEntry(...) == ALWAYS` with no kinematic-adjacency fallback, so
no joint angle can substitute for a missing SRDF entry. The SRDF has
exactly one `<disable_collisions>` entry followed by the literal comment
`<!-- and many more disable_collisions tags -->` (panda has 34, fanuc
10).

That is not a truncation this port introduced. `fixtures/pr2.srdf` is
**byte-identical** to `third_party/moveit_resources/pr2_description/srdf/robot.xml`
(verified by `diff`, no output) — the file `verify-fixture-provenance.sh`
already names as its source. Upstream's own test resource is the
abbreviated one, comment and all, alongside a `<group_state>` whose body
is `<!-- ... the rest of the joint values... -->`; it is an illustrative
SRDF, not a working PR2 description, and upstream ships it as the PR2
test fixture anyway. No complete PR2 SRDF exists on this host or in the
oracle container to source the missing entries from.

So authoring the missing entries is the wrong move twice over: it would
make the fixture diverge from the upstream file it is checked against,
and every pr2 oracle fixture in the tree was captured with the oracle
loading this exact SRDF, so changing it invalidates all of them.

**File F3 with a paired control instead.** The "self/intra absent"
requirement was only ever a way to make the environment branch's
contribution *identifiable* in the response; it is not the only way, and
it is the one that happens to be unreachable. Run F3 twice at the
identical joint state and identical ACM, once with the sphere and once
with `empty_env`, and difference the two responses: the robot state is
unchanged between them, so the self and intra-group contributions are
identical by construction and cancel, and every per-link
`gradient.collision`/`gradient.types` entry that *differs* is the
environment branch's own output. This identifies the branch positively
rather than inferring it from the absence of everything else, needs no
SRDF change, no new group, and no image change — `mode` absent still
selects `checkCollision`, which is what F3 exists to exercise.

The F3-vs-F2 cross-check below survives this: it compares F3's
environment verdict against F2's `robot_only` run at the same
`objects`/state pair. With the control pair it compares F3's *difference*
against F2's answer, which is the same comparison with the nuisance term
removed rather than assumed away.

**Refuting result, stated per case up front.** For F1: any nonzero
`gradients[i].closest_distance` collision report or `collision: true`
on any link refutes "clearly clear" and the case needs re-posing, not a
port defect — check this before treating a mismatch as a finding. For
F2: `collision: false`, or a `collision: true` on a link other than the
one the sphere was posed against, is a direct refutation of
`get_environment_collisions`'s port — the first one possible for this
method. For F3: the *difference* between its two paired runs (sphere vs.
`empty_env` at the identical state, per the control described above)
disagreeing with the same `objects`/state pair run through F2's
`robot_only` mode refutes either the branch-selection logic or the
short-circuit condition, whichever this port's local replay narrows it
to. The earlier form of this line said the two "should agree, since
self/intra report nothing so `checkCollision` degrades to exactly
`checkRobotCollision`'s answer" — self/intra never report nothing on
this fixture, so the degradation it assumed does not happen and the
comparison has to be against the difference, not against F3's raw
verdict. A second refuting result comes free with the control: if the
`empty_env` run and the sphere run report *identical* per-link
`gradient.collision`/`gradient.types`, the environment branch did not
fire at all and the case needs re-posing before anything is concluded
from it. For F4: the sphere marked "not overlapping" appearing in
`contacts`/lowering `closest_distance` below the true clear sphere's own
value is cross-talk between environment objects this port's per-object
loop should not produce.

## Response shape

Unchanged from `group_state_representation`'s existing response
(`oracle.cpp:4277-4294`: `group_name`, `links[]` each with
`has_link_decomposition`/`bounding_sphere_*`/`sphere_centers`/
`sphere_radii`/`collision_points_count`/`field_pose`/`gradient`, plus
`collision`/`contacts` when `"contacts": true`). No new output field for
`mode` itself — the mode selects which upstream call produces the
existing fields, it does not add a new one. `contacts: true` is
requested for every case in the table above (not optional here, unlike
the existing op's default) since the collision/no-collision verdict
*is* the object of this comparison, not incidental.

## Tolerance

Not set here, per this crate's own established policy (`distance_field`
op's tolerance, CLAUDE.md's "Size test tolerances from measurement"):
measured from the real responses once they exist, not carried over from
`group_state_representation_gradients_matches_the_oracle`'s existing
tolerance unmeasured — a `checkRobotCollision`/environment-`checkCollision`
comparison exercises different code (`get_environment_collisions`'s
distance thresholding vs. that test's gradient path) and nothing
guarantees the same bound applies. `closest_distance`/sphere-center
fields get a measured float tolerance; `collision`/`contacts`/link
identity fields are exact-match, same split this crate's other parity
tests already use.

## Does the existing oracle image serve this, or does the stamp move

**Both, split by case.** F3 needs no image change at all — it is a new
fixture entry on the existing `group_state_representation` op's existing
default path, using the `objects` field that already exists and is
already wired through `addRequestObjects`. F1/F2/F4 need one small,
localized C++ change: a `mode`/`"robot_only"` branch in
`groupStateRepresentation` (`oracle.cpp:4148`) that calls
`env.checkRobotCollision(req, res, *state_, use_acm ? &acm : nullptr,
gsr)` instead of `env.checkCollision(...)`, gated the same way the
existing `want_gradients`/`want_contacts` branches already are —
confirmed by reading the whole handler,
`groupStateRepresentation`(`oracle.cpp:4148-4295`): no other line
references `dfce->
distance_field_` (the field `checkRobotCollision`'s `generate_distance_
field=false` path skips building), so the rest of the response-dumping
code needs no change to serve the new branch. This is a few lines inside
an existing op, not a new op and not a new `find_package`/library link:
`collision_env_hybrid.hpp` is not included today (confirmed —
`collision_env_distance_field.hpp` is the only distance-field header
`oracle.cpp` includes, per its `#include` block; the two `checkRobotCollision`
hits elsewhere in this file, cited above, are `CollisionEnvFCL`'s, not
`CollisionEnvDistanceField`'s), so this op already talks to
`CollisionEnvDistanceField` directly, the same class this crate's own
`DistanceFieldCollisionCache` ports, never `CollisionEnvHybrid` — the new
branch calls a method already reachable through an object already
constructed on the line above it, no new include and no new FCL linkage.
**The stamp moves once, for the C++ change serving F1/F2/F4; F3 can be
filed against the current image unchanged.**

## How this port will use the response

F3 (and, once the C++ lands, F1/F2/F4) become a new
`crates/moveit-distance-field/tests/collision_env_hybrid_parity.rs`,
structured like `collision_env_distance_field_parity.rs`'s existing
`*_matches_the_oracle` tests but calling through
`HybridCollisionEnv::check_collision_distance_field`/
`check_robot_collision_distance_field` directly (a `World` built from
each case's `objects`, held in `HybridCollisionEnv::new`'s own `world`
parameter — not a hand-built `PropagationDistanceField` passed in as a
bare argument the way the existing `DistanceFieldCollisionCache`-level
tests do) so the comparison exercises `build_env_distance_field` itself,
not just the primitives it composes. This is the first test in the crate
that calls any `HybridCollisionEnv` method against a real oracle fixture
at all; every existing `collision_env_hybrid.rs` test
(`check_robot_collision_distance_field_reflects_a_world_swap_on_the_next_call`
and its neighbors) is self-consistency only, no external ground truth.
If F1/F2 disagree with this port's `check_robot_collision`, or F3
disagrees with F2 on the same state, either is reported back rather than
resolved unilaterally here, per the standing brief.
