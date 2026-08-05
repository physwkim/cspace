# Oracle request: settable `max_contacts_per_pair` on the `collision` op

**Status: fulfilled and used.** The coordinator shipped this request as
written (`47a271c`, "moveit-oracle: let the collision op report every
contact of a pair", oracle image `moveit-rs/oracle:700e7be54cb0a61f`,
`PORTING-PLAN.md` §212) before this document's own request even reached
them through the normal report channel — the two op changes in that
commit were already on `main` when this round's report was written. Round
29 of `doc/claim-audit/moveit-collision.md` re-runs case 623 against the
shipped field and gets a direct answer: the `(cone,
fr_caster_r_wheel_link)` pair holds 16 contacts at `max_contacts_per_pair
= 32`, not 1 — both this backend's own `~1.700e-2` reading and the
oracle's originally-reported `~7.479e-2` are members of the same list.
Case 623 is closed by that measurement; see `parry.rs`'s deviation-6(b)
doc and `doc/claim-audit/moveit-collision.md`'s round 29 for the full
result. The request below is left as written, unedited, as the record of
what was asked for and why — it is what round 29 measured against.

Not an implementation — request document for the human orchestrator, same
convention as `crates/moveit-distance-field/doc/oracle-request-hybrid-
collision-env-distance-field.md` and `crates/moveit-planners-pilz/doc/
oracle-request-pilz-blend.md`. `tools/moveit-oracle/` is not this panel's
file.

## The gap

`PORTING-PLAN.md`'s deviation-6(b) doc (`crates/moveit-collision/src/
parry.rs`) and `doc/claim-audit/moveit-collision.md`'s round-27/28
sections measure 945 `visibility_cone` cases where this backend's own EPA
depth disagrees with the oracle's own reported depth, and mechanically
confirm (round 28) that 9 of those are libccd's `ccdMPRPenetration`
converging to the touched cylinder's own half-length (a portal-refinement
artifact, traced line-by-line through `mpr.c`/`support.c`) rather than
the true penetration depth — reproduced independently by feeding this
backend's own winning triangle directly to `mpr_case104`, bypassing the
oracle entirely.

One of those 9, case 623, does not fit even the confirmed mechanism: this
backend's own winning triangle, run through the instrumented
`ccdMPRPenetration`, shows the *identical* plateau signature the other 8
show — but the oracle's own `collision` op reports `7.479e-2` (a normal,
non-plateau depth) for the same `(cone, fr_caster_r_wheel_link)` pair.
Since the plateau mechanism is now confirmed to fire on this backend's own
winning triangle too, the only remaining explanation is that the oracle's
own FCL narrow-phase evaluated a *different* triangle of the cone mesh
than the one this backend's own exhaustive search names as deepest.

`CollisionRequest::max_contacts_per_pair` (`collision_detection/collision_common.hpp:176`)
defaults to `1`, and `oracle.cpp`'s `collision()` op
(`crates/moveit-collision/examples/visibility_cone_mpr_sweep.rs`'s own
ground truth) never overrides it — `contactsToJson` (`oracle.cpp:2326-
2338`) reports `contact_list.front()` alone, so the oracle's own response
can never show more than one candidate triangle's depth for a mesh-vs-
cylinder pair, whatever FCL's narrow-phase actually tested. This request
asks for exactly the field that would let a future round see every
candidate FCL found for a pair, not just the first — settling case 623 by
direct measurement instead of leaving it a named-but-unconfirmed
possibility.

## What already exists, and what does not

Two pieces of the infrastructure this request needs are **already in
`oracle.cpp`**, just not wired to the `collision` op:

- `allContactsToJson` (`oracle.cpp:2364-2374`) already exists and is
  already used by two other ops (`distanceFieldCacheEntry`,
  `groupStateRepresentation`). It emits every entry of every pair's own
  `contact_list`, using the identical 7-field shape `contactToJson`
  already produces (`body_name_1`, `body_type_1`, `shape_kinds_1`,
  `body_name_2`, `body_type_2`, `shape_kinds_2`, `depth`) — no new JSON
  shape is needed, only calling the function that already exists.
- A settable `max_contacts_per_pair` request field already exists on
  those same two ops (`oracle.cpp:3391-3392`, `:3605-3606`):
  `req.max_contacts_per_pair = request.value("max_contacts_per_pair",
  static_cast<std::size_t>(1));` — the exact pattern this request asks be
  mirrored onto `collision()`.

What does **not** exist: the `collision` op (`oracle.cpp:2162-2208`, the
op `visibility_cone_mpr_sweep.rs` actually calls) hardcodes
`self_req.max_contacts_per_pair`/`robot_req.max_contacts_per_pair` at
their default (never set, so `1`), and its return statement calls
`contactsToJson`, not `allContactsToJson`. Those two ops that already
have the field use `CollisionEnvDistanceField`, not `CollisionEnvFCL` —
and their own `Contact::depth` is always `0.0` (documented in their own
comment at `oracle.cpp:3361-3368`/`:3575-3582`: upstream only sets `pos`
and body identities on that path), so they cannot answer this question at
all even with the field already exposed. This request is specifically for
`collision()`, the one op whose `Contact::depth` carries a real
penetration measurement.

## Request JSON shape

Add one optional field to the `collision` op's request, sibling to the
existing `joint_values`/`objects`/`attached_bodies`:

```json
{
  "op": "collision",
  "joint_values": { "...": "..." },
  "objects": [ { "...": "..." } ],
  "max_contacts_per_pair": 64
}
```

- **Field name**: `max_contacts_per_pair` — reuses the exact name the two
  distance-field ops already use for the identical upstream concept, not
  a new vocabulary word.
- **Type**: unsigned integer, optional, default `1` (today's behavior,
  byte-identical to every existing `collision`-op fixture when the field
  is absent).
- **Where it sits**: `collision()`'s own request object
  (`oracle.cpp:2162`), read the same way the two existing ops read it:

```cpp
self_req.max_contacts_per_pair =
    request.value("max_contacts_per_pair", static_cast<std::size_t>(1));
robot_req.max_contacts_per_pair =
    request.value("max_contacts_per_pair", static_cast<std::size_t>(1));
```

## Response shape

Swap `contactsToJson(self_res.contacts, *world)` /
`contactsToJson(robot_res.contacts, *world)` in `collision()`'s return
statement (`oracle.cpp:2202`, `:2206`) for `allContactsToJson(...)` — the
function already used by the two other ops, already producing the
identical per-contact object shape. At the default `max_contacts_per_pair
= 1`, FCL's own `contact_list` never holds more than one entry per pair,
so `allContactsToJson` degenerates to exactly `contactsToJson`'s own
output — **every existing `collision`-op fixture stays byte-identical**;
this is a pure superset when the new field is set past `1`. No new field
is added to the per-contact object itself (still `body_name_1`,
`body_type_1`, `shape_kinds_1`, `body_name_2`, `body_type_2`,
`shape_kinds_2`, `depth` — no `pos`/`normal`, unchanged from `collision`'s
own existing documented exclusion at `oracle.cpp:2140-2142`): `depth`
alone is sufficient to answer this specific question, since the two
candidate values (`~0.017`, the plateau; `~0.0748`, the deep value) are
already known from this panel's own independent analysis and do not need
a contact position to distinguish.

```json
{
  "self_contacts": [ ],
  "robot_contacts": [
    { "body_name_1": "cone", "body_name_2": "fr_caster_r_wheel_link", "depth": 0.017, "...": "..." },
    { "body_name_1": "cone", "body_name_2": "fr_caster_r_wheel_link", "depth": 0.07479, "...": "..." }
  ]
}
```

## Smallest fixture that discriminates the explanation

Case 623 of `visibility_cone_mpr_sweep --seed 4 --cases 1000` (pr2),
already fully specified and reproducible from committed code — no new
fixture file needed, just the existing sweep re-run with the new field.
Re-issue that exact case's own `Op::Collision` request (joint values from
`RandomStates{count:1000, seed:4}`'s 624th state, cone mesh built the same
way for `fr_caster_r_wheel_link`) with `max_contacts_per_pair` set high
enough to cover the cone mesh's own triangle count for that case (the
generator's `cone_sides = 3 + idx % 6`; for `idx=623`, `cone_sides = 9`,
`2 * cone_sides = 18` triangles — a safe bound like `32` covers it with
margin) instead of the default `1`:

- **If `robot_contacts` for the `(cone, fr_caster_r_wheel_link)` pair now
  contains an entry near `0.017` (the plateau depth this backend's own
  winning triangle produces) *and* an entry near `0.0748` (today's single
  reported value)**: confirms FCL evaluated (at least) two distinct
  triangles for this one pair, and the discrepancy is exactly the
  triangle-selection question this request exists to answer — closes
  case 623 with the `max_contacts_per_pair = 1` explanation.
- **If only one entry appears, at `~0.0748`**: refutes that explanation
  for case 623 specifically; FCL's own narrow-phase never found the
  plateau-triangle contact at all for this pair, and case 623's own cause
  is still open, now with a real candidate (a broad-phase/BVH difference,
  not narrow-phase-per-pair truncation) ruled in rather than assumed.

## How this panel will use the response

Re-run `visibility_cone_mpr_sweep.rs`'s own case-623 request (or a
dedicated one-case reproduction of it) with the new field set, read
`robot_contacts`' own `depth` values for the touched pair, and update
`doc/claim-audit/moveit-collision.md`'s round-28 section and `parry.rs`'s
deviation-6(b) doc with whichever of the two outcomes above the response
shows — closing case 623 by measurement, the same standard every other
claim in this deviation's own long history has been held to.
