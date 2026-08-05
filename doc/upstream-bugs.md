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

### 6. `MultivariateGaussian`'s Cholesky factor is computed unconditionally, so a non-positive-definite covariance produces `NaN` samples with no signal — not-reproduced

**Upstream:** `moveit_planners/stomp/include/stomp_moveit/math/multivariate_gaussian.hpp:86`
(`covariance_cholesky_(covariance_.llt().matrixL())` in the constructor's
init list) and the identical construction in
`moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/multivariate_gaussian.hpp:73`.
**Port:** `crates/moveit-sampling/src/multivariate_gaussian.rs:79` (`MultivariateGaussian::new`).
**Symptom:** neither constructor ever checks Eigen's `LLT::info()`. For a
covariance that is not positive-definite, `matrixL()` still returns a
matrix — built from the square root of a negative pivot, `NaN` — and every
subsequent `sample()` call silently produces `NaN` waypoints with no error
or log at the call site.
**Evidence:** read of upstream construction (`llt()`/`matrixL()` called
unconditionally, `info()` never read in either header). Not oracle-confirmed
against a real non-positive-definite covariance, since this port makes that
input unconstructable rather than exercisable.
**Status:** already not reproduced. `MultivariateGaussian::new` returns
`Option<Self>`, `None` for a shape mismatch or a covariance whose
`nalgebra::Cholesky::new` fails (module doc's "Deviation: construction can
fail", `multivariate_gaussian.rs:51-61`). No D-number is cited in-source for
this deviation; the module doc comment is the only existing record.
**Cost of not reproducing:** none. Already the shipped behaviour — every
caller in this workspace already goes through the fallible constructor.

### 7. An in-chain mimic joint whose master sits outside the group is silently dropped from `mimic_joints_`, desynchronising every later index into it — not-reproduced

**Upstream:** `moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp:166`
(`dimension_ = ...getActiveJointModels().size() + ...getMimicJointModels().size()`,
full-space, counts every mimic regardless of its master) against
`:197-226`'s `mimic_joints_`-building loop, which only pushes an entry for a
mimic joint whose master also satisfies
`joint_model_group_->hasJointModel(...)` (`:214,216`) — one whose master is
outside the group falls through both `if` blocks with nothing pushed, so
`mimic_joints_.size() < dimension_` and every `mimic_joints_[i]` lookup
after the drop point (e.g. `:339`, `:520`) walks against the wrong index.
**Port:** `crates/moveit-kinematics/src/chain.rs:145` (`ChainInfo::build`),
guard at `:259-263`.
**Symptom:** desynchronised index into a same-named-but-shorter vector —
the same "index survives past where its data does" family as entry 2
above, one level higher (the collection is silently short, not silently
wrong-length-read).
**Evidence:** read of upstream control flow (`:166` vs `:197-226` cross-checked
directly against the checked-out `moveit2` source this round).
**Status:** already not reproduced. `ChainInfo::build` rejects this input
with a construction `Error` instead (`chain.rs:259-263`,
tested by `build_rejects_an_in_chain_mimic_whose_master_is_outside_the_group`,
confirmed discriminating via live bite this round — see this worker's own
`doc/assertion-discrimination-ledger-p1-fixtures.md`, Round 4). The port's
own doc comment (`chain.rs:119-130`) already names this as a deviation from
upstream's silent behaviour, but cites no D-number.
**Cost of not reproducing:** none. Already the shipped behaviour, and no
fixture in this workspace has a mimic joint on a chain whose master sits
outside the chain's own group (per the same doc comment).

### 8. `checkConsistency` loops full-space `dimension_` while indexing a reduced-space (mimic-filtered) `consistency_limits` vector — out-of-bounds — not-reproduced

**Upstream:** `moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp:84-93`
(`checkConsistency`'s `for (i = 0; i < dimension_; ++i) ... consistency_limits[i]`)
called at `:392` with `consistency_limits_mimic` — built at `:326-340` by
filtering the caller's full-space `consistency_limits` down to one entry
per *active* joint (mimic joints excluded), so its size is `<= dimension_`
whenever the chain has at least one mimic joint.
**Port:** `crates/moveit-kinematics/src/registry.rs:24-36` (`SolveOptions::consistency_limits`
doc comment naming the deviation), enforced by
`crates/moveit-kinematics/src/cart_to_jnt.rs:308` (`satisfies_consistency`).
**Symptom:** `std::vector::operator[]` read past the end of
`consistency_limits_mimic` for any chain with a mimic joint and a caller
that supplies `consistency_limits` — undefined behaviour, not a caught
error.
**Evidence:** read of upstream control flow, function signature at
`:84-86`, its one call site at `:392`, and the reduced-space vector's
construction at `:326-340`, all cross-checked against the checked-out
`moveit2` source this round.
**Status:** already not reproduced. `SolveOptions::consistency_limits` is
reduced-space (one entry per `KinematicsSolver::joint_names`, the same
space `seed`/the solution already live in) from the start, so the
mismatched-length read this port's own doc comment describes is
impossible by construction. No D-number is cited in-source.
**Cost of not reproducing:** none. Already the shipped behaviour.

### 9. `initialize`'s acceleration-bound extraction advances its flat write index once per *joint*, not once per *variable*, silently keeping only a multi-variable joint's last variable's bound — not-reproduced

**Upstream:** `moveit_core/online_signal_smoothing/src/acceleration_filter.cpp:189-207`
(`initialize`): outer loop over `joint_bounds` (one entry per active
*joint*), inner loop over `*joint_bound` (one entry per that joint's
*variables*) writing `min_acceleration_limits_[ind]`/`max_acceleration_limits_[ind]`,
but `ind++` (`:206`) sits at the *outer* loop's scope, so a joint with more
than one variable overwrites the same `ind` slot on every inner iteration
and only the last variable's bound survives; the next joint's bound then
lands in a slot that should have been the skipped variables', and every
joint after the first multi-variable one is off by the excess variable
count.
**Port:** `crates/moveit-smoothing/src/acceleration_filter.rs:144`
(`joint_acceleration_bounds`), guard at `:153-159`.
**Symptom:** silent bound misalignment for any group containing a
multi-variable active joint (planar, floating) — not an error, a wrong
numeric bound applied to the wrong joint from that point on.
**Evidence:** read of upstream control flow (`ind++` placement at `:206`,
outside the inner `for` at `:192-205`, cross-checked against the checked-out
`moveit2` source this round).
**Status:** already not reproduced. `joint_acceleration_bounds` rejects a
multi-variable active joint with a dedicated `Error` before it can reach
this misalignment (`acceleration_filter.rs:153-159`, tested by
`multi_dof_active_joint_is_a_typed_error_not_a_silent_last_variable_wins`,
confirmed discriminating via live bite this round). The port's own doc
comment (`acceleration_filter.rs:48-83`) already names this as "upstream's
per-*joint* (not per-*variable*) index-advance bug," avoided by
construction under this port's single-DOF-active-joint contract, but cites
no D-number.
**Cost of not reproducing:** none. Already the shipped behaviour, and this
port has no fixture with a multi-variable active joint feeding
`AccelerationLimitedFilter` (its own doc comment: "there is no fixture
robot in this workspace with a multi-DOF active joint whose correct
per-variable bound behaviour could be derived independently").

### 10. `doSmoothing`'s length-check variable is misnamed and reads the wrong argument — reproduced-pending-decision

**Upstream:** `moveit_core/online_signal_smoothing/src/acceleration_filter.cpp:312-313`
(`const size_t num_positions = velocities.size(); if (num_positions != num_joints_)`),
with the mismatch surfaced in an error message that names "the positions
parameter" (`:314-319`) while the value actually checked is
`velocities.size()`.
**Port:** `crates/moveit-smoothing/src/acceleration_filter.rs:326`
(`AccelerationLimitedFilter::do_smoothing`), the check at `:328-334`.
**Symptom:** a `positions`/`velocities` pair that disagree in length is
accepted or rejected by this check based on `velocities`' length alone;
`positions`' own length is checked only indirectly, by the very next
`else if` (against `last_positions_.size()`, not `num_joints_` directly).
A caller passing a wrong-length `velocities` with a correctly-sized
`positions` gets an error message misattributing the fault to
"the joint positions parameter." This port transcribes the same
misattribution verbatim (`acceleration_filter.rs:84-94`'s doc comment
already flags it as "transcribed... rather than 'fixed'").
**Evidence:** read of upstream source, cross-checked against the
checked-out `moveit2` source this round (`:312-313`). Not oracle-confirmed
— `tests/fixtures/acceleration_filter_{request,response}.json`'s oracle
comparison only ever calls `do_smoothing` with correctly-matched-length
arrays, so it does not exercise either length-check branch, let alone
distinguish which of the two fires.
**Status:** reproduced, unexamined since being ported under the old brief.
No numeric output is affected either way (a length mismatch always
produces *an* error; this only picks which of two `Error::Other` messages
comes back, and only when exactly one of `positions`/`velocities` is
wrong-length while the other happens to already equal `num_joints`) — but
a competent reviewer of the C++ would call the variable name and the
message's argument attribution wrong.
**Cost of not reproducing:** unmeasured in the sense that no test in this
crate currently exercises the specific case that would distinguish "check
against `velocities.len()`" from "check against `positions.len()`" (a call
where exactly one of the two disagrees with `num_joints` while the other
happens to match `last_positions_.len()` too) — no oracle comparison and
no unit test here pins the current message text for that case, so
changing the check to read `positions.len()` directly would not currently
break any test in this workspace. Confirming that for certain would need a
new test for the distinguishing case first, not an assumption.

---

## Open question

Entries 1, 3, 5 and 10 are `reproduced-pending-decision`: they were ported
faithfully under the old brief and have not been revisited. Fixing them is
a behaviour change against an oracle-verified port, so each needs its
cost-of-not-reproducing line filled in with a measurement rather than the
"unmeasured" placeholders above before anything is changed.

Entries 6-9 (added by `p1-fixtures`'s fence sweep: `moveit-sampling`,
`moveit-kinematics`) are already `not-reproduced` — the port structurally
avoids each one rather than transcribing it, and each already carries a
doc comment describing the deviation, but none cites a `PORTING-PLAN.md`
D-number. Assigning one (or confirming none applies) is unassigned work,
not a decision blocked on measurement the way 1/3/5/10 are.
