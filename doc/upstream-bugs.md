# Upstream bugs

A defect that exists in the C++ MoveIt sources this workspace ports from.
Its entry belongs here, not in a code comment alone, and **the default is
that the port does not reproduce it**.

This inverts the rule the port ran under until now. Several comments in the
tree say some version of *"this looks like an upstream bug, but the brief
for this port is to transcribe the numerics as written, not to fix
behaviour no test here contradicts"* — that brief no longer holds. Those
sites are listed below with their current status so the change is visible
rather than silent.

## What belongs here

An upstream **bug**: behaviour a competent reviewer of the C++ would call
wrong — out-of-bounds indexing, a NaN escaping into a returned value, state
overwritten by two branches that were meant to be exclusive, a guard the
upstream author's own comment doubts.

An upstream **deviation** is a different thing and does not belong here.
Deviations are behaviour this port changes for reasons of language, safety
or API shape, and they are classified `D1`..`D25` in `PORTING-PLAN.md`.
A bug that we decline to reproduce will usually also acquire a deviation
class; record it in both places and cross-reference.

Not a bug: behaviour that merely surprises. `Trajectory::velocity` being a
step function of time reads like an oversight and is entry 4 below, but it
is upstream's actual, oracle-confirmed behaviour — the question of whether
to keep it is a *parity* decision, not a defect report.

## Entry format

Append entries; do not renumber existing ones.

```
### N. <one-line symptom> — <status>

**Upstream:** <file:line in the C++ source, with the version or path root>
**Port:**     <file:line in this workspace>
**Symptom:**  what the C++ does that is wrong, concretely.
**Evidence:** how we know — oracle run, upstream's own TODO, a read of the
              control flow. Say which; a read is the weakest.
**Status:**   `not-reproduced` | `reproduced-pending-decision` | `reproduced-deliberately`
**Cost of not reproducing:** which parity tests or oracle comparisons move
              if we deviate. If none, say none.
```

`reproduced-deliberately` needs a stated reason and a signature — it is the
exception now, not the default.

---

### 1. Two non-exclusive `if` blocks double-increment `iteration_` — reproduced-pending-decision

**Upstream:** `chomp_motion_planner/src/chomp_optimizer.cpp:367-410`
**Port:** `crates/moveit-planners-chomp/src/optimizer.rs:909`
**Symptom:** the `iteration_ % 10 == 0` mesh-to-mesh check and the
`!filter_mode_` collision-threshold check are two separate,
unconditionally-evaluated `if` blocks rather than `if`/`else if`. Both can
fire in one pass: the first increments `iteration_` and zeroes
`num_collision_free_iterations_`, then the second increments `iteration_`
a *second* time in the same loop pass and overwrites
`num_collision_free_iterations_` with `max_iterations_after_collision_free_`.
**Evidence:** read of upstream control flow. Not oracle-confirmed.
**Cost of not reproducing:** unmeasured — CHOMP iteration counts would
change, so any test pinning an iteration count or a converged trajectory
is at risk.

### 2. Contact-reporting branch indexes `link_body_decompositions_` out of bounds — not-reproduced

**Upstream:** `collision_env_distance_field.cpp:601`
**Port:** `crates/moveit-distance-field/src/collision_env_distance_field.rs:1871`
**Symptom:** the branch unconditionally reads
`con.pos = gsr->link_body_decompositions_[i]->getSphereCenters()[k]`
without branching on `i_is_link`, unlike `body_type_1` immediately below
(`:604-611`). When `i` is an attached-body index this reads past the end of
a vector sized `num_links` — undefined behaviour.
**Evidence:** read of upstream control flow, cross-checked in round 26
(an earlier citation of `:534`/`:537-544` was wrong; the bug claim was not).
**Status:** already not reproduced — safe Rust cannot express it.
**Cost of not reproducing:** none. Already the shipped behaviour.

### 3. Attached-body count check upstream's own comment doubts — reproduced-pending-decision

**Upstream:** `collision_env_distance_field.cpp:1132-1137`, carrying the
comment `TODO: This logic for checking attached body count might be incorrect`
**Port:** `crates/moveit-distance-field/src/collision_env_distance_field.rs:1055`
**Symptom:** upstream's own author flagged the comparison as possibly
wrong and it was shipped unchanged.
**Evidence:** upstream's own TODO. What the check *should* compare has not
been established here.
**Cost of not reproducing:** unknown until the correct comparison is
established. Do not deviate before that.

### 4. `getVelocity` never re-derives its time step against the query time — reproduced-deliberately

**Upstream:** `trajectory_processing`'s `Trajectory::getVelocity`
**Port:** `crates/moveit-trajectory/src/trajectory.rs:217`
**Symptom:** it computes `path_pos`/`path_vel` from the *full* enclosing
segment's step (`it->time_ - previous->time_`) rather than against the
query time the way `getPosition` does (`time - previous->time_`). For any
`time` inside a segment it returns the value at `current.time_` — a step
function of `time`, constant within each segment. `acceleration` shares the
property via `segment_endpoint_state`.
**Evidence:** oracle. An earlier, "fixed" version of this port disagreed
with the `totg` oracle op; see `tests/totg_parity.rs`.
**Status:** reproduced deliberately. This is the one entry where reproducing
is clearly right — it is upstream's observable behaviour, confirmed against
a running oracle, and the port's parity claim is built on it.
**Cost of not reproducing:** `tests/totg_parity.rs` fails. Deviating here
means abandoning totg parity, which is a much larger decision than a bug fix.

### 5. `Path_Circle` has no both-zero guard, so `scale_rot` can be NaN — reproduced-pending-decision

**Upstream:** pilz `path_circle`, which unlike `Path_Line` has no guard for
`oalpha == 0.0 && dist == 0.0`
**Port:** `crates/moveit-planners-pilz/src/path_circle.rs:311`
**Symptom:** in the `else` arm, `scale_rot = oalpha / dist`. With both zero
this is `0.0 / 0.0` = NaN, which escapes into the constructed path.
**Evidence:** read, plus the asymmetry with `Path_Line`, which *does* guard.
**Cost of not reproducing:** unmeasured. A `Path_Line`-shaped guard
returning an error is the obvious candidate, and no pilz parity test is
known to depend on the NaN — that needs checking, not assuming.

---

## Open question

Entries 1, 3 and 5 are `reproduced-pending-decision`: they were ported
faithfully under the old brief and have not been revisited. Fixing them is
a behaviour change against an oracle-verified port, so each needs its
cost-of-not-reproducing line filled in with a measurement rather than the
"unmeasured" placeholders above before anything is changed.
