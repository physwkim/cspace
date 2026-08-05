# Upstream bugs

Upstream is `moveit2` at `e017c91ee12984393a28ba246075c65f69cde3bf`,
checked out at `/home/stevek/work/moveit2` (`PORTING-PLAN.md:3`). Every
`file:line` below is repository-relative to that checkout and was read
there. `kdl-path-circle-nan-scale-rot` is the exception: its upstream is
orocos KDL, a separate project, and it names the in-tree `third_party/`
checkout instead.

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
or API shape, and they are classified `D1`..`D14` in `PORTING-PLAN.md` —
each a policy with its own section, not a per-site record. A bug we decline
to reproduce acquires a deviation class only when an existing policy is
what makes us decline; otherwise this ledger is its whole record.

Not a bug: behaviour that merely surprises. `Trajectory::velocity` being a
step function of time reads like an oversight and is
`totg-velocity-step-function` below, but it is upstream's actual,
oracle-confirmed behaviour — the question of whether
to keep it is a *parity* decision, not a defect report.

## Entry format

**Entries are identified by a slug, not a number.** The slug is kebab-case
and names the upstream symbol plus the defect
(`get-max-payload-index-space`), so it is unique by construction: two
panels working in different crates cannot collide, and if they do pick the
same slug they found the same bug and it is one entry. Append anywhere;
never rename a slug once it is cited.

This replaced sequential numbering after four panels on parallel worktrees
each appended an "entry 6" in one round, leaving every report that cited
"entry 6" pointing at a different bug. A number is assigned by position in
a file that several branches append to at once; a slug is assigned by the
subject, which is the thing that is actually unique.

```
### `<slug>` — <one-line symptom> — <status>

**Upstream:** <file:line in the C++ source, with the version or path root>
**Port:**     <file:line in this workspace>
**Symptom:**  what the C++ does that is wrong, concretely.
**Evidence:** how we know — oracle run, upstream's own TODO, a read of the
              control flow. Say which; a read is the weakest.
**Status:**   `not-reproduced` | `reproduced-deliberately` | `reproduced-grandfathered`
**Cost of not reproducing:** which parity tests or oracle comparisons move
              if we deviate. If none, say none.
```

`reproduced-deliberately` needs a stated reason and a signature — it is the
exception now, not the default.

`reproduced-grandfathered` is closed to new entries. It marks the bugs
already in the tree when the policy inverted, which the user decided on
2026-08-05 to leave in place; see "Decision on the pre-policy entries"
below. A bug found from now on is `not-reproduced` unless someone argues
`reproduced-deliberately` for it.

## Index

| slug | status |
|---|---|
| `chomp-iteration-double-increment` | reproduced-grandfathered |
| `distance-field-contact-index-oob` | not-reproduced |
| `attached-body-count-check` | reproduced-grandfathered |
| `totg-velocity-step-function` | reproduced-deliberately |
| `kdl-path-circle-nan-scale-rot` | reproduced-grandfathered |
| `inv-twice-resolution-int-truncation` | reproduced-deliberately |
| `max-distance-sq-narrowing` | not-reproduced |
| `get-shortest-solution-empty-deref` | not-reproduced |
| `multivariate-gaussian-cholesky-unchecked` | not-reproduced |
| `mimic-master-outside-group-dropped` | not-reproduced |
| `check-consistency-index-space-mismatch` | not-reproduced |
| `acceleration-bounds-per-joint-advance` | not-reproduced |
| `do-smoothing-length-check-operand` | reproduced-grandfathered |
| `get-max-payload-index-space` | reproduced-grandfathered |
| `cost-source-nan-blind-compare` | not-reproduced |

---

### `chomp-iteration-double-increment` — Two non-exclusive `if` blocks double-increment `iteration_` — reproduced-grandfathered

**Upstream:** `moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:368-410`
(verified at the pinned `e017c91e`: `if (iteration_ % 10 == 0)` sets
`num_collision_free_iterations_ = 0` and `iteration_++`; the separate
`if (!parameters_->filter_mode_)` at `:406` sets it to
`max_iterations_after_collision_free_` and increments `iteration_` again)
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

### `distance-field-contact-index-oob` — Contact-reporting branch indexes `link_body_decompositions_` out of bounds — not-reproduced

**Upstream:** `moveit_core/collision_distance_field/src/collision_env_distance_field.cpp:601`
(verified: `con.pos = gsr->link_body_decompositions_[i]->getSphereCenters()[k];`
unconditional, with `if (i_is_link)` first appearing at `:604` for `body_type_1`)
**Port:** `crates/moveit-distance-field/src/collision_env_distance_field.rs:1871`
**Symptom:** the branch unconditionally reads
`con.pos = gsr->link_body_decompositions_[i]->getSphereCenters()[k]`
without branching on `i_is_link`, unlike `body_type_1` immediately below
(`:604-611`). When `i` is an attached-body index this reads past the end of
a vector sized `num_links` — undefined behaviour.
**Evidence:** read of upstream control flow, cross-checked in round 26
(an earlier citation of `:534`/`:537-544` was wrong; the bug claim was not).
**Status:** already not reproduced — safe Rust cannot express it.
**Deviation:** no `D1`..`D14` policy applies. Out-of-bounds indexing being
unrepresentable is a Rust-safety fact, not a project decision anyone
signed off on — this ledger entry is the whole record.
**Cost of not reproducing:** none. Already the shipped behaviour.

### `attached-body-count-check` — Attached-body count check upstream's own comment doubts — reproduced-grandfathered

**Upstream:** `moveit_core/collision_distance_field/src/collision_env_distance_field.cpp:1132`
(verified verbatim: `// TODO: This logic for checking attached body count might
be incorrect`, guarding
`gsr->attached_body_decompositions_.size() != att->getShapes().size()`)
**Port:** `crates/moveit-distance-field/src/collision_env_distance_field.rs:1055`
**Symptom:** upstream's own author flagged the comparison as possibly
wrong and it was shipped unchanged.
**Evidence:** upstream's own TODO. What the check *should* compare has not
been established here.
**Cost of not reproducing:** unknown until the correct comparison is
established. Do not deviate before that.

### `totg-velocity-step-function` — `getVelocity` never re-derives its time step against the query time — reproduced-deliberately

**Upstream:** `moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp:881-895`
(verified: `getPosition` re-assigns `time_step = time - previous->time_` at
`:874` before computing `path_pos`; `getVelocity` never does, computing
`path_pos` and `path_vel` at `:891-893` from the `:887` full-segment step)
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

### `kdl-path-circle-nan-scale-rot` — `Path_Circle` has no both-zero guard, so `scale_rot` can be NaN — reproduced-grandfathered

**Upstream:** `third_party/orocos_kinematics_dynamics/orocos_kdl/src/path_circle.cpp:91-96`
— KDL is a separate project from `moveit2`, and this workspace already
carries it at tag `1.5.1` (`db25b7e`). `third_party/` is untracked, so it
is present in the primary checkout and absent from `caucus` worktrees.
Both `path_circle.cpp` and `path_line.cpp` are byte-identical between
`1.5.1` and current KDL master (`c73370f0`), so the asymmetry below is not
a version artefact; `git log -L` dates the `Path_Circle` `else` arm to
`d545368` (2020) and the `Path_Line` guard to at least `c6f0842` (2008).
`moveit2`'s `pilz_industrial_motion_planner/src/path_circle_generator.cpp`
is the *caller*; the missing guard is in KDL itself.
**Port:** `crates/moveit-planners-pilz/src/path_circle.rs:312`
**Symptom:** `PathCircle::new`'s `else` arm (path_circle.rs:302-312) runs
whenever `oalpha*eqradius > dist` is false, including when both are zero,
and computes `scale_rot = oalpha/dist` with `dist = geometry.alpha *
radius`. `radius` is guarded `>= eps` earlier in the same function, so
`dist == 0.0` requires `geometry.alpha == 0.0` exactly — a zero-sweep
circle, i.e. `start`'s and `goal`'s Cartesian *positions* coincide (see
the reachability finding below). `oalpha == 0.0` is reached far more
easily than exact rotation equality: `get_rot_angle`
(`crate::path_line::get_rot_angle`, `path_line.rs:164-169`) snaps any
rotation angle below its `eps` argument to exactly `0.0`, pinned by
`get_rot_angle_below_eps_snaps_to_exactly_zero` — so a *near*-identical,
not only a bit-identical, start/goal orientation reaches this side of the
condition. When both hold, `scale_rot` is `0.0/0.0` = NaN, escaping into
the constructed path.
**Evidence:** verified verbatim in the checkout above, and the asymmetry is
upstream's own: `orocos_kdl/src/path_line.cpp:67-83` carries the comment
`// Only modify if non zero (prevent division by zero)` above a three-way
guard whose third arm is commented `// both were zero`. KDL recognised this
division and fixed it in `Path_Line`; `Path_Circle` never got the same
treatment upstream.

**Correction (this round, p9-ros):** this entry previously said the port
"faithfully reproduces" the missing guard, framing it as an inherited,
unexamined artefact of line-by-line transcription. That framing is wrong.
`path_circle.rs`'s own module doc (`:66-87`, "Why this file stays
BSD-3-Clause") and `PORTING-PLAN.md`'s **D9** (`§141`: `Path_Circle`
derived independently from circular geometry, not transcribed line by
line — required because KDL is LGPL-2.1-or-later and this workspace is
BSD-3-Clause) and **D11** (`§152`: extends D9's independent-derivation
rule to `path_line.rs`/`velocity_profile_trap.rs`/`dynamics.rs`) together
establish that no line of `orocos_kdl/src/path_circle.cpp` was ever
copied into this port — `PathCircle` is built from elementary vector
algebra with no KDL source open at construction time. There is no line
for a missing guard to survive *from*. Matching upstream's NaN-producing
lack of a both-zero guard was therefore a **choice made inside that
independent derivation**, not an artefact of faithful transcription — the
port's own doc comment (`path_circle.rs:309-311`) already names it as a
choice: "Upstream has no 'both zero' guard here, unlike `Path_Line` ...
faithfully reproducing that." The grandfathering decision below is
unaffected by this correction — the shipped behaviour does not change
either way — but the record should read "a deliberate parity choice made
during an independent derivation, later left unexamined for whether it is
still the right choice," not "an unfixed instance of a known bug."

**Reachability, confirmed by reading `build_path`'s two branches**
(`crates/moveit-planners-pilz/src/trajectory_generator_circ.rs:314-335`,
the only production caller of `circle_from_center`/`circle_from_interim`/
`PathCircle::new`): neither construction kind can reach `PathCircle::new`
with `dist == 0.0`.
- `CircPathConstraintKind::Center`: `circle_from_center` hardcodes
  `CircleGeometry.aux_point = goal` (`path_circle.rs:157`).
  `PathCircle::new`'s plane guard (`:289-295`) tests `goal - center`
  against `x_axis = normalize(start - center)`; whenever `start`/`goal`
  are close enough for `geometry.alpha` (computed by the same `cosines()`
  law-of-cosines formula from the `start`/`goal`/`center` distances) to
  round to exactly `0.0`, `goal - center` and `x_axis` are, by the same
  small-angle geometry, close enough to parallel that the plane guard's
  `z_norm < eps` (`eps = MAX_COLINEAR_NORM = 1e-5` for this kind) fires
  first at any physically ordinary Cartesian coordinate magnitude (metres
  to kilometres) — the angular precision needed for `acos` to round to
  exactly `1.0` in `f64` (~1e-8 rad) is finer than the guard's `1e-5`
  threshold.
- `CircPathConstraintKind::Interim`: `circle_from_interim` computes
  `w = (interim - start).cross(goal - start)` and rejects
  `w.norm() < MAX_COLINEAR_NORM` (`path_circle.rs:180-189`) *before*
  `geometry.alpha` is ever computed. `goal - start == 0` (the exact case,
  and the only case that can drive `geometry.alpha` all the way to `0.0`
  here) makes `w` exactly the zero vector regardless of the client-chosen
  `interim` point, so this guard rejects unconditionally first.

Neither `cmd_specific_request_validation` nor `extract_motion_plan_info`
(the request-validation steps upstream of `build_path`) rejects a
coincident start/goal earlier — the guard that actually catches it is
`circle_from_center`/`circle_from_interim`/`PathCircle::new`'s own
plane/colinearity check. This is a read, not an oracle run or a new test:
a pathologically large coordinate magnitude (roughly beyond `1e8`, where
the two small-angle quantities above could plausibly decouple) is outside
what this read rules out, and is not exercised by any fixture in this
workspace.
**Status:** reproduced-grandfathered. Closed to further measurement per
the 2026-08-05 decision below.
**Cost of not reproducing:** none demonstrated. No live caller in this
workspace reaches the NaN branch (see the reachability finding above,
superseding this entry's earlier "no known reaching caller in the pilz
tests" hedge, which had no evidence behind it when it was written). A
`Path_Line`-shaped guard returning an error remains the obvious candidate
if this is ever revisited.

### `inv-twice-resolution-int-truncation` — `inv_twice_resolution_` mistyped as `int`, silently truncating — reproduced-deliberately

**Upstream:** `moveit_core/distance_field/include/moveit/distance_field/distance_field.hpp:614`
(`int inv_twice_resolution_;` among otherwise-`double` fields) and
`moveit_core/distance_field/src/distance_field.cpp:67`
(`inv_twice_resolution_(1.0 / (2.0 * resolution_))`, a `double` expression
narrowed into that `int` field at construction), used unconditionally at
`:91-93` (`gradient_x/y/z = (...) * inv_twice_resolution_`) — verified at
the pinned `e017c91e`.
**Port:** `crates/moveit-distance-field/src/distance_field.rs:427`
(`let inv_twice_resolution = (1.0 / (2.0 * self.resolution())) as i32 as
f64;`), documented in-line at `:368-426` and again in
`crates/moveit-distance-field/src/lib.rs:991-998`.
**Symptom:** `1.0 / (2.0 * resolution)` is truncated toward zero into an
`int` on every construction. For most resolutions this changes the
gradient multiplier from the mathematically exact value to upstream's
truncated one — e.g. `resolution = 0.03` gives `16.666...` untruncated
but upstream's stored multiplier is `16`. Past `resolution >= 0.51` the
truncated multiplier is `0`, so every returned gradient is identically
zero on both sides of the port.
**Evidence:** oracle, boundary-pinned. Round 26 found the two originally-
ported upstream tests both happen to use resolutions (`0.1`, `0.02`)
where the ratio is already an exact integer, so the truncation was a
no-op there and the divergence went unmeasured until then; the port now
casts through `i32` to reproduce the truncation bit-for-bit rather than
matching it by coincidence, and
`distance_gradient_truncates_inv_twice_resolution_like_upstreams_int_
field`/`distance_gradient_multiplier_is_one_at_the_zero_boundary`/
`distance_gradient_multiplier_is_zero_just_past_the_boundary`
(`distance_field.rs:1190-1260`) pin it, including the `0.5`/`0.51` zero
boundary. Cross-referenced in `PORTING-PLAN.md` §172.1 case 1.
**Status:** reproduced deliberately — this is genuine parity (upstream's
own gradient is equally truncated), not a residual bug, and this crate's
mandate is matching upstream's actual behaviour rather than its intent
(see `crate::get_body_decomposition_cache_entry`'s doc for the same
principle applied elsewhere in this crate).
**Cost of not reproducing:** the three tests named above fail; any
resolution not equal to an exact-integer-ratio value would silently
diverge from upstream by a measurable, non-error gradient-magnitude
difference.

One further boundary inside the same guard does **not** match upstream
and is left that way deliberately (`PORTING-PLAN.md` §153.1 — expires if
upstream ever changes `inv_twice_resolution_`'s declared type away from
`int`): below `resolution ≈ 2.328e-10`, `1.0/(2.0*resolution)` exceeds
`i32::MAX`, where Rust's `as i32` saturates (well-defined since Rust
1.45) but C++'s narrowing conversion of an out-of-range `double` is UB —
there is no upstream value to match even in principle. This sub-case is
undocumented-and-unreachable in this crate's current tests/oracle
fixtures (the smallest resolution used anywhere is two orders of
magnitude larger), not silently wrong in a reachable case today, but is
recorded here per `distance_field.rs:404-426` rather than left for the
next audit to rediscover.

### `max-distance-sq-narrowing` — `max_distance_sq_`'s narrowing would OOM if unguarded — not-reproduced

**Upstream:** `moveit_core/distance_field/src/propagation_distance_field.cpp:88`
(`max_distance_sq_ = ceil(max_distance_ / resolution_) *
ceil(max_distance_ / resolution_);`, a `double` product narrowed into an
`int` field with no range check before it) used at `:95-96,99-100` to
size `bucket_queue_`/`negative_bucket_queue_`/`sqrt_table_` — verified at
the pinned `e017c91e`.
**Port:** `crates/moveit-distance-field/src/propagation.rs:218`
(`checked_max_distance_sq`), documented in-line at `:199-217`.
**Symptom:** past `n > 46340` (`n*n > i32::MAX`, where `n =
ceil(max_distance/resolution)`), C++'s narrowing is UB with no upstream
value to match; an unguarded Rust `as i32` would instead saturate to
`i32::MAX`, and `PropagationDistanceField::new` sizes three collections
from that value — an attempted allocation of three `2^31`-length
collections (OOM), not merely a wrong number.
**Evidence:** read of upstream control flow, cross-referenced in
`PORTING-PLAN.md` §172.1 case 2.
**Status:** already not reproduced — `checked_max_distance_sq` rejects
`max_distance_sq_f > f64::from(i32::MAX)` before ever casting, returning
`Error::Construct` instead of saturating into the allocation.
**Deviation:** no `D1`..`D14` policy applies. `PORTING-PLAN.md` §172
documents the float-to-int narrowing family this belongs to at length but
was never assigned a `D` number — this ledger entry is the whole record.
**Cost of not reproducing:** none. Already the shipped behaviour; a
valid-per-this-guard value like `46340 * 46340` (~2.1 billion) is itself
far too large to build a field around, so the boundary is tested via the
standalone `checked_max_distance_sq` helper rather than by actually
constructing a field at that size.

### `get-shortest-solution-empty-deref` — `getShortestSolution` dereferences `min_element` on a possibly-empty vector — not-reproduced

**Upstream:** `moveit_ros/planning/planning_pipeline_interfaces/src/
solution_selection_functions.cpp:47-64` (`getShortestSolution`: `const
auto shortest_trajectory = std::min_element(solutions.begin(),
solutions.end(), ...); ... return *shortest_trajectory;` at `:64`,
unconditional, with no check that `solutions` is non-empty) — verified at
the pinned `e017c91e`.
**Port:** `crates/moveit-planning/src/plan_responses.rs:111`
(`shortest_solution`), documented in-line at `:42-49`.
**Symptom:** on an empty `solutions` vector, `std::min_element` returns
`solutions.end()`, and dereferencing that iterator is undefined
behaviour.
**Evidence:** read of upstream control flow. `min_element`'s empty-input
return contract (`first` when `[first,last)` is empty, i.e. `end()`
in this call) is documented cppreference behaviour, not something that
needs a runtime probe.
**Status:** already not reproduced — the port's `shortest_solution`
returns `Option<&PlanOutcome<'_>>`, `None` for the empty case, a typed
empty case rather than a panic or a fabricated element. For non-empty
input the tie-break matches upstream's comparator exactly (see the
function's own doc for the strict-improvement-fold reasoning against
`Iterator::min_by`'s different tie-break contract).
**Deviation:** no `D1`..`D14` policy applies. Returning `Option` instead of
dereferencing an out-of-range iterator is a local API-shape choice on this
one function, not an instance of a project-wide policy — this ledger
entry is the whole record.
**Cost of not reproducing:** none. Already the shipped behaviour, and
"reproducing" this one is not meaningfully possible in safe Rust without
introducing a panic where upstream has UB — not a like-for-like trade.

### `multivariate-gaussian-cholesky-unchecked` — `MultivariateGaussian`'s Cholesky factor is computed unconditionally, so a non-positive-definite covariance produces `NaN` samples with no signal — not-reproduced

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
**Deviation:** no `D1`..`D14` policy applies. Fallible construction here is
a local API-shape choice already named in the module's own doc comment,
not an instance of a project-wide policy — this ledger entry is the whole
record.
**Cost of not reproducing:** none. Already the shipped behaviour — every
caller in this workspace already goes through the fallible constructor.

### `mimic-master-outside-group-dropped` — An in-chain mimic joint whose master sits outside the group is silently dropped from `mimic_joints_`, desynchronising every later index into it — not-reproduced

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
the same "index survives past where its data does" family as `distance-field-contact-index-oob`
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
**Deviation:** no `D1`..`D14` policy applies. `ChainInfo::build` rejecting
this input is a local construction-time validation choice, not an
instance of a project-wide policy — this ledger entry is the whole
record.
**Cost of not reproducing:** none. Already the shipped behaviour, and no
fixture in this workspace has a mimic joint on a chain whose master sits
outside the chain's own group (per the same doc comment).

### `check-consistency-index-space-mismatch` — `checkConsistency` loops full-space `dimension_` while indexing a reduced-space (mimic-filtered) `consistency_limits` vector — out-of-bounds — not-reproduced

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
**Deviation:** no `D1`..`D14` policy applies. `SolveOptions::consistency_limits`
being reduced-space from the start is a local type-shape choice, not an
instance of a project-wide policy — this ledger entry is the whole
record.
**Cost of not reproducing:** none. Already the shipped behaviour.

### `acceleration-bounds-per-joint-advance` — `initialize`'s acceleration-bound extraction advances its flat write index once per *joint*, not once per *variable*, silently keeping only a multi-variable joint's last variable's bound — not-reproduced

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
**Deviation:** no `D1`..`D14` policy applies. `joint_acceleration_bounds`
rejecting a multi-variable active joint at construction time is a local
API-shape choice already named in the port's own doc comment, not an
instance of a project-wide policy — this ledger entry is the whole
record.
**Cost of not reproducing:** none. Already the shipped behaviour, and this
port has no fixture with a multi-variable active joint feeding
`AccelerationLimitedFilter` (its own doc comment: "there is no fixture
robot in this workspace with a multi-DOF active joint whose correct
per-variable bound behaviour could be derived independently").

### `do-smoothing-length-check-operand` — `doSmoothing`'s length-check variable is misnamed and reads the wrong argument — reproduced-grandfathered

**Upstream:** `moveit_core/online_signal_smoothing/src/acceleration_filter.cpp:311-312`
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

### `get-max-payload-index-space` — `getMaxPayload` indexes `max_torques_` in the wrong joint-index space — reproduced-grandfathered

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
**Status:** reproduced-grandfathered. The only ground truth available
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

### `cost-source-nan-blind-compare` — `CostSource::operator<` compares `double`s with bare `<`/`>`, silently blind to `NaN` — not-reproduced

**Upstream:** `collision_common.hpp:128-140`
**Port:** `crates/moveit-collision/src/common.rs:118` (doc comment on the
`Ord`/`Eq` impl for `CostSource`, found while sweeping the crate for
assertion-discrimination coverage, not newly introduced this round)
**Symptom:** `operator<` chains bare `double` `<`/`>` comparisons
(`c1 > c2` / `c1 < c2` / `cost < other.cost` / `cost > other.cost` /
`aabb_min < other.aabb_min`). Every comparison against `NaN` is `false`, so
a `NaN` cost or AABB bound sorts as neither greater nor less than anything
`std::set` compares it against — `std::set` (strict-weak-order lookup)
would treat it as equivalent to whatever it's compared against first,
silently coalescing a `NaN`-carrying entry with an unrelated one.
**Evidence:** read of upstream control flow (`collision_common.hpp:128-140`,
checked against the pinned `e017c91ee` checkout). Not oracle-confirmed —
no test here constructs a `NaN` cost or AABB bound.
**Status:** already not reproduced. The port's own doc comment (`common.rs
:105-121`) states the reason: `Ord`/`Eq` are implemented with
[`f64::total_cmp`] instead of a bare `<`/`>` chain, specifically to give a
total order for every bit pattern including `NaN`. This was written as a
"Deviation from upstream" in-code but has no `D`-number in `PORTING-PLAN.md`
(searched for `CostSource`/`NaN`/`total_cmp`/`operator<` near the `D1`..`D25`
list; none matched) — flagging that gap here rather than assigning one
myself.
**Cost of not reproducing:** none measured to date — well-formed geometry
never produces `NaN` here per the same comment, and no test in this crate
constructs a `NaN` cost or AABB bound to check what upstream's `std::set`
would actually do with one. Unlike `distance-field-contact-index-oob`, "safe Rust cannot express it"
does not apply: a bare `<`/`>` `Ord` impl is expressible in Rust (it would
just panic or misbehave under `#[derive(Ord)]`/manual impl using
`partial_cmp().unwrap()`), so this one was an active choice, not a language
constraint.

---

## Decision on the pre-policy entries

Asked on 2026-08-05 whether to measure-then-deviate, deviate immediately,
or document only: **document only, code unchanged.**
`chomp-iteration-double-increment`, `attached-body-count-check` and
`kdl-path-circle-nan-scale-rot` are `reproduced-grandfathered` and stay as
they are. `do-smoothing-length-check-operand` and
`get-max-payload-index-space` were found after the decision but are the
same class — already in the tree, ported verbatim under the old brief — so
they are grandfathered on the same reasoning rather than treated as new
findings. Their
cost-of-not-reproducing lines keep the "unmeasured"/"unknown" placeholders,
which is now accurate rather than an outstanding task — no measurement is
owed, because nothing is being changed.

The inverted policy is **forward-looking**. It binds bugs found from here
on; it does not reopen behaviour already ported and gated. Anyone who wants
to move an entry off `reproduced-grandfathered` needs a fresh decision, not
this document.

`totg-velocity-step-function` is separately `reproduced-deliberately` — it is the one entry with
a positive argument for reproducing (the totg oracle), so it does not
depend on the grandfathering above.

The reason for grandfathering rather than fixing is that each is a
behaviour change against a port whose parity is oracle-verified, and none
has a demonstrated failure in this workspace: `chomp-iteration-double-increment` is a read of upstream
control flow with no oracle run behind it, `attached-body-count-check` has no established
correct comparison to change *to*, `kdl-path-circle-nan-scale-rot`'s NaN has
no reaching caller in this workspace (confirmed by reading `build_path`'s
two branches — see that entry's own reachability finding), and
`do-smoothing-length-check-operand`'s misattributed message text is
pinned by no test that distinguishes the two operands. Deviating on any of
them would trade a verified behaviour for an unverified one.

## Closed (round p9-ros, 2026-08-05)

Both items previously open here are resolved:

- **`kdl-path-circle-nan-scale-rot`'s justification.** **D9** (`§141`)
  rules that `orocos_kdl`'s `Path_Circle` is *not* transcribed line by
  line but derived independently from circular geometry, and **D11**
  (`§152`) extends that to `path_line.rs`, `velocity_profile_trap.rs` and
  `dynamics.rs`. That entry's "faithfully reproduces"/"unfixed instance"
  framing has been rewritten in place to say what it actually is: a
  deliberate parity choice made inside an independent derivation. Its
  reachability was also checked by reading `build_path`'s two branches —
  no live caller in this workspace can reach the NaN. The
  `reproduced-grandfathered` status is unchanged; only the reasoning on
  the record was wrong, not the decision.
- **Whether the `not-reproduced` entries need a `D` class.** Checked each
  of `distance-field-contact-index-oob`, `max-distance-sq-narrowing`,
  `get-shortest-solution-empty-deref`,
  `multivariate-gaussian-cholesky-unchecked`,
  `mimic-master-outside-group-dropped`,
  `check-consistency-index-space-mismatch` and
  `acceleration-bounds-per-joint-advance` against `PORTING-PLAN.md`'s full
  `D1`..`D14` registry. None maps to an existing policy — each is a local
  API-shape or construction-time-validation choice scoped to its own
  function, not an instance of a project-wide decision the user signed
  off on. Each now carries a `**Deviation:**` line saying so explicitly,
  so the gap is recorded rather than silently left for the next reader to
  wonder about. No new `D` number was invented for any of them, per this
  round's instruction.
