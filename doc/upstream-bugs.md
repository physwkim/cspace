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

### 6. `getMaxPayload` indexes `max_torques_` in the wrong joint-index space — reproduced-pending-decision

**Upstream:** `moveit_core/dynamics_solver/src/dynamics_solver.cpp:126` (`num_joints_ =
kdl_chain_.getNrOfJoints()`, active-joints-only count — KDL chains omit
fixed joints structurally) versus `:132-144` (`max_torques_` built by
iterating `joint_model_group_->getJointModelNames()`, the *full*
fixed-joint-inclusive space, pushing `0.0` for any name with no URDF
limits, which every fixed joint has) versus `:246-254` and `:271-284`
(`getMaxPayload`'s two saturation/payload loops, both `for (i = 0; i <
num_joints_; ++i)` but reading `max_torques_[i]`).
**Port:** `crates/moveit-state/src/dynamics.rs:75-93` (module doc,
"Deviation from upstream: `getMaxPayload`'s indexing bug is replicated")
and `:518` (`DynamicsSolver::max_payload`).
**Symptom:** `max_torques_` is indexed in the full joint-model space it
was built over, but both loops in `getMaxPayload` bound `i` to
`num_joints_`, the *active*-joint count. For a chain with a fixed joint
strictly before its last active joint (pr2's `right_arm`:
`r_upper_arm_joint`, fixed, precedes `r_elbow_flex_joint`, active, in
joint-model order), a real joint's torque is compared against a
*different* joint's limit — one that is fixed and therefore always
`0.0` — which saturates the check immediately and forces `payload =
0.0` for every input.
**Evidence:** oracle. `tools/moveit-oracle/src/oracle.cpp:1290-1302`
(the `dynamics()` endpoint's doc comment) states the same mechanism
independently, naming the same two upstream sites and the same
`right_arm`/`r_upper_arm_joint`/`r_elbow_flex_joint` example. The
captured fixture `crates/moveit-state/tests/fixtures/pr2_dynamics.json`
confirms it operationally: all 7 `right_arm` cases have
`max_payload.payload == 0.0`, each saturating on a different
`joint_saturated` index (3, 1, 1, ...) — consistent with the
mismatch landing on whichever joint the misaligned index happens to
hit, not a genuine physical saturation.
I checked whether this also explains `fanuc_dynamics.json`'s `manipulator`
group, which shows the identical all-`0.0`-payload pattern across all 7
cases: `crates/moveit-state/tests/fixtures/fanuc.srdf`'s `manipulator`
group is `<chain base_link="base_link" tip_link="tool0"/>`, and
`fanuc.urdf`'s only joints along that chain are six consecutive
`revolute` joints (`joint_1`..`joint_6`) followed by one trailing fixed
joint (`joint_6-tool0`) *after* the last active joint, not before — the
structural precondition this bug needs (a fixed joint preceding the
last active one) is absent. Fanuc's all-zero pattern is not evidence of
this bug and its cause is unestablished; do not cite it as a second
instance without separately investigating fanuc's URDF effort limits.
**Status:** reproduced-pending-decision. The only ground truth available
to verify `max_payload` against (`pr2_dynamics.json`, captured from the
real oracle) reflects the buggy behavior; there is no ground truth to
verify a "fixed" version against.
**Cost of not reproducing:** `crates/moveit-state/tests/dynamics_parity.rs::pr2_dynamics_matches_the_oracle`
(line 217) would fail outright — all 7 `right_arm` cases' expected
`max_payload.payload`/`joint_saturated` values were captured under the
buggy behavior and a corrected index would change every one of them,
and no corrected-oracle fixture exists to re-capture against.
`fanuc_dynamics_matches_the_oracle` (line 205) is unaffected per the
structural check above.

---

## Open question

Entries 1, 3 and 5 are `reproduced-pending-decision`: they were ported
faithfully under the old brief and have not been revisited. Fixing them is
a behaviour change against an oracle-verified port, so each needs its
cost-of-not-reproducing line filled in with a measurement rather than the
"unmeasured" placeholders above before anything is changed.
