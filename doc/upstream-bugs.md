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

### 6. Timing-loop division has no zero-relative-velocity guard, so `time_`/`getDuration` can be NaN or +inf — reproduced-pending-decision

**Upstream:** `trajectory_processing/src/time_optimal_trajectory_generation.cpp:405`:
`it->time_ = previous->time_ + (it->path_pos_ - previous->path_pos_) / ((it->path_vel_ + previous->path_vel_) / 2.0);`
— no guard against the denominator being `0.0`.
**Port:** `crates/moveit-trajectory/src/trajectory.rs:182`, the direct
transcription of the same division in `Trajectory::create`'s timing pass.
**Symptom:** when `path_vel + previous.path_vel == 0.0` (a `max_velocity`
component of `0.0` on an axis the path still moves along, or a
zero-length segment), the division is `0.0/0.0` (NaN) or `x/0.0` (`+inf`)
depending on whether the position-delta numerator itself rounds to
exactly `0.0` at the path's absolute scale — both are reachable and the
NaN escapes into `Trajectory::create`'s `Ok` return via `getDuration`.
**Evidence:** read of upstream control flow (no guard at `:405`) plus two
already-passing tests in this port that deliberately construct and
document both outcomes:
`trajectory.rs::a_zero_length_path_produces_a_nan_duration_trajectory`
(NaN, zero-length path) and
`trajectory.rs::a_max_velocity_component_of_zero_crawls_rather_than_invalidating`
(NaN or `+inf` depending on path scale, nonzero-length path). Reachable
through the public `compute_time_stamps_with_limits` API too, not just
direct `Trajectory::create`/`Path::create` calls — confirmed this round
by `time_optimal_trajectory_generation.rs::resample_dt_over_a_nan_duration_is_rejected`
(a custom `0.0` velocity limit on a moving joint, panda_arm, 1e-5-scale
path). Not oracle-confirmed; no `totg_parity` case is known to exercise
either branch.
**Cost of not reproducing:** unmeasured precisely, but bounded: this
port's own `do_time_parameterization_calculations` already added a
downstream safety net (`!raw_sample_count.is_finite() ||
raw_sample_count > MAX_RESAMPLE_SAMPLE_COUNT`, itself not present
upstream — see this module's "Deviations from upstream" note) that
catches the NaN/`+inf` one call later and returns `Err` either way, so no
currently-passing test asserts success past this point for either
scenario. Fixing `:405` at the source would move where the `Err` fires
(inside `Trajectory::create` instead of
`do_time_parameterization_calculations`) and require rewriting
`trajectory.rs`'s two tests above (they currently assert the NaN/`+inf`
`Ok` outcome directly) plus re-deriving whether
`resample_dt_over_a_nan_duration_is_rejected`'s message changes. No
`totg_parity` oracle comparison is known to depend on the NaN/`+inf`
value itself (only on `getVelocity`'s step-function behavior, entry 4) —
that needs checking, not assuming, before deviating.

---

## Open question

Entries 1, 3, 5 and 6 are `reproduced-pending-decision`: they were ported
faithfully under the old brief and have not been revisited. Fixing them is
a behaviour change against an oracle-verified port, so each needs its
cost-of-not-reproducing line filled in with a measurement rather than the
"unmeasured" placeholders above before anything is changed. Entry 6's
line is partially measured (reachability through the public API is
confirmed, by a live test, not just by reading) but still lacks the one
number that would decide it: whether any `totg_parity` oracle case
depends on the current NaN/`+inf` value.
