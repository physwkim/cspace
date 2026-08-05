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
| `totg-timing-zero-velocity-division` | reproduced-grandfathered |
| `polyline-filter-waypoints-stale-index` | reproduced-deliberately |
| `polyline-header-redeclares-lin-exceptions` | not-reproduced |
| `plan-components-builder-const-build-mutates` | not-reproduced |
| `extract-blend-radii-empty-list-underflow` | not-reproduced |
| `ik-cache-read-trusts-file-header` | not-reproduced |
| `get-best-approximate-static-dummy-stale` | not-reproduced |
| `update-cache-capacity-as-size-limit` | not-reproduced |
| `save-cache-empty-path-guard-falls-through` | not-reproduced |
| `cached-ik-accumulate-return-discarded` | not-reproduced |
| `ik-cache-map-first-update-dropped` | not-reproduced |
| `set-from-ik-zero-timeout-is-not-single-attempt` | not-reproduced |
| `validate-and-improve-interval-percentage-discarded` | not-reproduced |
| `fcl-distance-sentinel-survives-zero-contacts` | not-reproduced |
| `aggregated-limits-drops-rejected-joint-silently` | not-reproduced |
| `check-position-bounds-multidof-adjacent-members` | not-reproduced |
| `count-samples-per-second-returns-a-ratio` | not-reproduced |
| `all-valid-distance-robot-hides-base-overload` | not-reproduced |
| `distance-callback-max-contact-depth` | not-reproduced |
| `pr2-collision-test-asserts-unwritten-result` | not-reproduced |

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
**Status:** `reproduced-grandfathered`. Named as grandfathered by the
2026-08-05 decision below, which is where the reasoning is; this line only
puts the status where the entry format asks for it.
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
**Status:** `reproduced-grandfathered`. Named as grandfathered by the
2026-08-05 decision below, which is where the reasoning is; this line only
puts the status where the entry format asks for it.
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
**Status:** `reproduced-grandfathered`, unexamined since being ported under
the old brief.
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

### `totg-timing-zero-velocity-division` — Timing-loop division has no zero-relative-velocity guard, so `time_`/`getDuration` can be NaN or +inf — reproduced-grandfathered

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
direct `Trajectory::create`/`Path::create` calls — confirmed by
`time_optimal_trajectory_generation.rs::resample_dt_over_a_nan_duration_is_rejected`
(a custom `0.0` velocity limit on a moving joint, panda_arm, 1e-5-scale
path). Not oracle-confirmed; no `totg_parity` case is known to exercise
either branch.
**Status:** reproduced-grandfathered. Ported verbatim under the old
faithful-transcription brief; found and documented this round, after the
2026-08-05 decision was announced but before it reached this worktree —
same class as `do-smoothing-length-check-operand`/`get-max-payload-index-space`
(already in the tree, not a new post-decision finding), grandfathered on
the same reasoning.
**Cost of not reproducing:** unmeasured, and that is now accurate rather
than an outstanding task — no measurement is owed, because nothing is
being changed. This port's own `do_time_parameterization_calculations`
already added a downstream safety net (`!raw_sample_count.is_finite() ||
raw_sample_count > MAX_RESAMPLE_SAMPLE_COUNT`, itself not present
upstream) that catches the NaN/`+inf` one call later and returns `Err`
either way, so no currently-passing test asserts success past this point
for either scenario.

### `polyline-filter-waypoints-stale-index` — `filterWaypoints` compares against an index into the *input* list that counts *kept* waypoints, so after the first drop it measures against a waypoint it already dropped — reproduced-deliberately

**Upstream:** `moveit_planners/pilz_industrial_motion_planner/src/path_polyline_generator.cpp:60-85`
(`PathPolylineGenerator::filterWaypoints`). `last_added_point_indx` starts
at `-1` and is incremented once per *kept* waypoint (`:80`), while the
`last_point` lambda (`:71`) reads `waypoints[last_added_point_indx]` — an
index into the *input* vector.
**Port:** `crates/moveit-planners-pilz/src/path_polyline_generator.rs`,
`filter_waypoints`.
**Symptom:** the two counters agree only while nothing has been dropped.
Each drop shifts them apart by one, so from the first waypoint kept after a
drop onwards, the distance is measured against an earlier input waypoint —
including against a waypoint this same filter dropped for being too close.
The filter's whole purpose is to guarantee every surviving segment exceeds
`MIN_SEGMENT_LENGTH` before `Path_RoundedComposite::Add` sees it; past the
first drop it no longer does, so `Add`'s own `Not_Feasible` codes 2/3 can
still fire on input the filter was supposed to have cleaned.
**Evidence:** oracle-confirmed. `filter_waypoints_compares_against_an_input_index_not_the_last_kept_pose`
in the port constructs the four-waypoint case where the two behaviours
differ (4 kept under upstream's rule, 3 under the intended one) and asserts
upstream's outcome; `panda_polyline_staleindex_{request,response}.json` and
`polyline_panda_arm_reproduces_the_stale_filter_index_the_oracle_has` then
run that same case through the live oracle, which rejects it with
`INVALID_MOTION_PLAN` and logs `rounding circle of a point is bigger than
the distance with one of the neighbor points` — the `Not_Feasible` path
this entry's **Symptom** predicts. The port reaches the same code, and
correcting the filter (bitten: `filtered.last()` instead of
`waypoints[last_added_point_indx]`) flips that test to `SUCCESS` and no
other test in the crate. The read of upstream source at the pinned sha is
still what identifies the two counters; it is no longer the only evidence.
**Status:** reproduced-deliberately — the same positive argument as
`totg-velocity-step-function`, not grandfathering. `POLYLINE`'s only
available ground truth is the C++ implementation itself, and this bug
changes *which waypoints reach the path*, so a corrected filter would make
every case containing a drop diverge from the oracle by construction. That
would leave `POLYLINE` with no comparable parity surface at all, which is a
worse outcome than carrying a filter that under-filters in a way the
downstream `Add` still rejects loudly.
**Cost of reproducing:** a waypoint list with a near-duplicate followed by
more waypoints reaches `Add` with a segment shorter than
`MIN_SEGMENT_LENGTH` and is rejected with `Not_Feasible` code 2 or 3
instead of being silently cleaned. That is an error return, not a wrong
trajectory — now measured rather than predicted, by the `staleindex`
fixture above.

The revisit this entry used to name ("if a `POLYLINE` oracle op lands and
its fixtures show the divergence mattering") has happened, and the answer
is to keep reproducing. The op landed with
`tools/moveit-oracle/src/pilz_polyline_factory.cpp`; the divergence does
matter — it changes the returned error code, not just the waypoint list —
and that is precisely what makes reproducing it the right call: a
corrected filter would return `SUCCESS` where the oracle returns `-2`, so
every drop-containing case would diverge from ground truth by
construction. The `reproduced-deliberately` status stands, with the
argument now resting on a measurement instead of a prediction.

---

### `polyline-header-redeclares-lin-exceptions` — `trajectory_generator_polyline.hpp` redefines three exception classes `trajectory_generator_lin.hpp` already defines, so no translation unit can include both — not-reproduced

**Upstream:** `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_generator_polyline.hpp:49,51,52`
against `.../trajectory_generator_lin.hpp:48,50,51` (verified at the pinned
`e017c91e`): `LinTrajectoryConversionFailure`, `JointNumberMismatch` and
`LinInverseForGoalIncalculable`, each declared in both headers, in the same
`pilz_industrial_motion_planner` namespace, through the same
`CREATE_MOVEIT_ERROR_CODE_EXCEPTION` macro — which expands to a full class
definition, not a forward declaration or a `using`.
**Port:** none — see Status.
**Symptom:** `#include`ing both headers in one translation unit fails to
compile with `redefinition of class 'LinTrajectoryConversionFailure'` (and
the other two). The `#pragma once` in each header does not help: these are
two distinct headers each defining the same three names. The POLYLINE
generator was added by copying the LIN header, and the three `Lin`-prefixed
names came along with it although POLYLINE raises only
`LinTrajectoryConversionFailure` and `LinInverseForGoalIncalculable` under
those names by coincidence of the copy. Upstream never hits the error
because every `trajectory_generator_*.cpp` includes only its own header,
and no upstream consumer instantiates more than one generator directly —
they go through `planning_context_loader_*` plugins, each its own library.
**Evidence:** an actual build failure, not a read. `tools/moveit-oracle`
wants all four PTP/LIN/CIRC/POLYLINE generators constructible from one
file; adding the POLYLINE include to `src/oracle.cpp` produced exactly the
three `redefinition of class` errors above, against upstream sources at the
pinned sha inside the oracle image.
**Status:** `not-reproduced`. There is nothing to reproduce: the port has
no macro that emits a class per error code and no per-generator exception
type at all — `moveit-planners-pilz` returns
`Err(Error::Code(MoveItErrorCode::…))` from every generator, so the same
code raised by two generators is the same enum variant by construction, not
two colliding definitions. The defect is a property of the C++ header
layout, and the port has no header layout.
**Deviation:** none of `D1`..`D14` applies. This is not a behavioural
choice — the port's error representation predates POLYLINE and was not
selected to avoid this.
**Cost of not reproducing:** none for the port. The cost lands on the
oracle instead: `tools/moveit-oracle/src/pilz_polyline_factory.{hpp,cpp}`
exists solely to keep `trajectory_generator_polyline.hpp` in a translation
unit of its own and hand `oracle.cpp` back a base-class
`std::unique_ptr<TrajectoryGenerator>`. Delete that indirection and the
oracle stops building. If upstream ever removes the duplicate declarations,
the factory can be inlined back into `oracle.cpp`.

---

### `plan-components-builder-const-build-mutates` — `PlanComponentsBuilder::build() const` appends the tail into the builder's own last trajectory, so a second call appends it twice — not-reproduced

**Upstream:** `moveit_planners/pilz_industrial_motion_planner/src/plan_components_builder.cpp:43-52`
(verified at the pinned `e017c91e`), declared `build() const` at
`include/pilz_industrial_motion_planner/plan_components_builder.hpp:110`.
**Port:** `crates/moveit-planners-pilz/src/plan_components_builder.rs`,
`PlanComponentsBuilder::build`.
**Symptom:** `std::vector<RobotTrajectoryPtr> res_vec{ traj_cont_ }` copies
the *vector of pointers*, not the trajectories, so `res_vec.back()` and
`traj_cont_.back()` are the same `RobotTrajectory`. Line 49 then calls
`appendWithStrictTimeIncrease(*(res_vec.back()), *traj_tail_)`, which
mutates it. `build()` is `const` and takes no `traj_tail_` copy, so calling
it twice on one builder appends the tail twice — the second result carries
the tail's waypoints duplicated, at times that
`appendWithStrictTimeIncrease` keeps strictly increasing, so nothing
downstream rejects it. Every earlier `build()` result aliases the same
trajectory and changes underneath its holder as well.
**Evidence:** a read of the control flow, plus a read of the single caller.
`CommandListManager::solve` calls `plan_comp_builder_.reset()`
(`src/command_list_manager.cpp:106`), then `append` in a loop, then exactly
one `build()` (`:116`), and `reset()` clears both members — so the second
call never happens upstream and the defect is unreached. Not
oracle-confirmed: the oracle exposes single motion commands, not sequences,
so it has no operation that would call `build` at all.
**Status:** `not-reproduced`. This port's `build(mut self)` consumes the
builder, which makes the second call not merely unreached but
unrepresentable — the type system rejects it, so no test or future caller
can reach the duplicated-tail state. That is strictly stronger than
upstream's "the one caller happens not to do it".
**Deviation:** none of `D1`..`D14` applies. Consuming `self` is the
ownership shape Rust gives a builder whose output moves out of it; it was
not selected to route around this defect, it removes it as a side effect.
**Cost of not reproducing:** none. There is no oracle op and no parity test
on the sequence path, and the reproducing behaviour has no caller upstream
to compare against. The visible cost is on the port's API instead:
`reset()` is not ported, because a consuming `build` means a new sequence
is a new builder — which is what `reset()` was emulating for a builder
held as a long-lived member.

---

### `extract-blend-radii-empty-list-underflow` — `CommandListManager::extractBlendRadii` loops to `size() - 1` on an unsigned zero — not-reproduced

**Upstream:** `moveit_planners/pilz_industrial_motion_planner/src/command_list_manager.cpp:219-232`
(verified at the pinned `e017c91e`).
**Port:** `crates/moveit-planners-pilz/src/command_list_manager.rs`,
`CommandListManager::extract_blend_radii`.
**Symptom:** `RadiiCont radii(req_list.items.size(), 0.);` followed by
`for (RadiiCont::size_type i = 0; i < (radii.size() - 1); ++i)`. On an empty
list `radii.size()` is `0` and `radii.size() - 1` is `SIZE_MAX`, so the loop
condition holds and the body runs `req_list.items.at(0)` on an empty vector.
`std::vector::at` is the bounds-checked accessor, so this throws
`std::out_of_range` rather than reading out of bounds — but out of a function
whose declared failure mode is the `MoveItErrorCodeException` family, so the
`move_group` sequence service catches nothing and the exception escapes as an
unhandled one.
**Evidence:** a read of the control flow, plus a read of the single caller.
`CommandListManager::solve` returns `RobotTrajCont()` on
`req_list.items.empty()` before anything else (`:91-94`), so the only path
that reaches `extractBlendRadii` has at least one item and the loop is never
entered with an empty list. Not oracle-confirmed: the oracle has no sequence
op.
**Status:** `not-reproduced`. The port iterates `items.windows(2)` instead of
an index range, so the empty case is a zero-iteration loop by construction —
there is no subtraction to underflow and no accessor to go out of range. This
is stronger than upstream's guard, which lives in a different function.
**Deviation:** none of `D1`..`D14` applies. Pair iteration is the ordinary
Rust way to write "for each consecutive pair"; it was not selected to route
around this defect.
**Cost of not reproducing:** none. There is no oracle op and no parity test on
the sequence path, and the reproducing behaviour has no caller upstream to
compare against.

---

### `ik-cache-read-trusts-file-header` — `initializeCache` sizes every buffer from unchecked file-supplied counts and never compares the file's DOF count with the solver's — not-reproduced

**Upstream:** `moveit_kinematics/cached_ik_kinematics_plugin/src/ik_cache.cpp:96-141`
(verified at the pinned `e017c91e`).
**Port:** `crates/moveit-kinematics/src/ik_cache/format.rs`, `from_json`.
**Symptom:** four separate ways a cache file's own header is believed.
`last_saved_cache_size_`, `num_dofs` and `num_tips` are read with three
unchecked `cache_file.read` calls; the latter two are uninitialized locals
(`:101`, `:103`), so a header shorter than twelve bytes leaves them
indeterminate and every size below is computed from whatever was on the
stack. `bufsize` is then `num_tips * 7 * sizeof(tf2Scalar) + num_dofs *
sizeof(double)` in `unsigned int` arithmetic and reaches `new
char[bufsize]` (`:115`) — the products wrap rather than fail. `num_dofs` is
never compared against the `num_joints` argument, which is not stored until
`:143`, *after* the entries are loaded: a cache written for a different arm
loads silently, and its configs are handed to the solver as seeds of the
wrong length. Finally the per-entry `cache_file.read(buffer, bufsize)`
(`:124`) is unchecked and `entry` is reused across iterations, so a
truncated file leaves the previous entry's bytes in `buffer` and
`push_back`s that entry once per remaining iteration, up to the count the
file itself declared. `entry.second.resize(num_dofs)` followed by
`memcpy(&entry.second[0], ...)` (`:118`, `:131`) also subscripts an empty
vector when `num_dofs` is zero.
**Evidence:** a read of the control flow. Not oracle-confirmed: no oracle
operation reads a cache file, and this port's format is not upstream's, so
there is nothing byte-level to compare against.
**Status:** `not-reproduced`. `from_json` deserializes into typed documents
— lengths come from the JSON arrays themselves, not from a declared count —
and then rejects a document whose `num_joints` is not the solver's, an
entry whose config is not that long, more entries than the document's own
`max_cache_size` admits, and an orientation that is not a unit quaternion.
A truncated file is a parse error, not a duplicated tail.
**Deviation:** none of `D1`..`D14` applies. `PORTING-PLAN.md` §80.2 already
rules the on-disk format a local choice rather than a port target, which
removes the byte-layout failure modes as a side effect; the four content
checks are a separate decision, local to this one function.
**Cost of not reproducing:** none. No parity test and no oracle operation
touches the cache file. The cost lands on the API instead: reading a cache
is fallible here, so `IkCache::load` and `CachedIkSolver::from_cache_file`
return `Result` where upstream's `initializeCache` returns `void`.

---

### `get-best-approximate-static-dummy-stale` — The empty-cache reply is a function-local `static`, so every later empty-cache query gets the first one's pose and joint count — not-reproduced

**Upstream:** `moveit_kinematics/cached_ik_kinematics_plugin/src/ik_cache.cpp:163`
and `:174` (the two `IKCache::getBestApproximateIKSolution` overloads), and
`:318` (`IKCacheMap::getBestApproximateIKSolution`'s copy of the same
shape), all verified at the pinned `e017c91e`.
**Port:** `crates/moveit-kinematics/src/ik_cache.rs`, `IkCache::nearest`.
**Symptom:** `static IKEntry dummy = std::make_pair(std::vector<Pose>(1,
pose), std::vector<double>(num_joints_, 0.))` is a *function-local*
`static`: it is constructed on the first call that finds a cache empty and
returned by `const` reference unchanged for the life of the process, for
every `IKCache` instance, since the storage belongs to the function and not
to the object. Both halves then go stale. The pose half is the first
caller's target: `getPositionIK`
(`cached_ik_kinematics_plugin-inl.hpp:96-100`) hands the same entry to
`updateCache`, whose novelty gate computes `nearest.first[0].distance(pose)`
— against a fresh dummy that term is zero by construction, against the
stale one it is the distance to an unrelated request's pose, so the gate
that is meant to be decided by config distance alone can be opened by the
pose term. The config half freezes the first caller's `num_joints_`, and
`nearest.second` is passed straight to the wrapped plugin as its seed
(`-inl.hpp:97`), so a second arm with a different DOF count is seeded with a
vector of the wrong length.
**Evidence:** a read of the control flow, plus a read of the one caller
that consumes both halves. Not oracle-confirmed: reaching it needs two
`IKCache` instances in one process, which no oracle operation sets up.
**Status:** `not-reproduced`. `IkCache::nearest` returns an owned
`CacheEntry` built from the query pose and the cache's own `num_joints`, so
there is no storage to go stale and nothing shared between instances.
**Deviation:** none of `D1`..`D14` applies. Returning an owned value rather
than a reference into shared storage is the ownership shape Rust gives a
value the caller then hands to `update`; it was not chosen to route around
this defect.
**Cost of not reproducing:** none. The all-zero seed is tried first on a
cold cache either way — that is upstream's design, recorded as CONFIRMED in
`doc/claim-audit/moveit-kinematics.md` — and only the staleness is dropped.

---

### `update-cache-capacity-as-size-limit` — `updateCache` bounds the cache by `capacity()`, which a cache file can raise above the configured `max_cache_size_` — not-reproduced

**Upstream:** `moveit_kinematics/cached_ik_kinematics_plugin/src/ik_cache.cpp:183`
and `:197` (both `IKCache::updateCache` overloads), with `:75` and `:119`
the two `reserve` calls that set the bound, verified at the pinned
`e017c91e`.
**Port:** `crates/moveit-kinematics/src/ik_cache.rs`, `IkCache::update`;
`crates/moveit-kinematics/src/ik_cache/format.rs`, `from_json`.
**Symptom:** the insert gate is `ik_cache_.size() <
ik_cache_.capacity()` — the limit enforced is the allocator's, not the
`max_cache_size` the user configured. `initializeCache` establishes it with
`ik_cache_.reserve(max_cache_size_)` (`:75`), and `reserve` guarantees only
`capacity() >= n`, so the two coincide by implementation habit rather than
by contract. They come apart for a concrete reason, not a hypothetical one:
`:119` calls `ik_cache_.reserve(last_saved_cache_size_)` with the entry
count read straight out of the cache file (see
`ik-cache-read-trusts-file-header`), and `reserve` never shrinks a vector.
A file declaring more entries than `max_cache_size_` therefore raises the
effective limit above the configured one permanently, and the cache grows
past its own maximum for the rest of the process. The save trigger cannot
catch up either: `ik_cache_.size() == max_cache_size_` (`:189`, `:218`) is
an equality, so once the size is past that value it never fires again and
only the every-500-entries clause remains.
**Evidence:** a read of the control flow, plus `reserve`'s specified
postcondition (`capacity() >= n`, not `== n`). Not oracle-confirmed.
**Status:** `not-reproduced`. `IkCache::update` gates on `self.entries.len()
>= self.options.max_cache_size`, and `from_json` refuses a document holding
more entries than its own recorded `max_cache_size` — so the loaded state
cannot start out above the limit either. The `Vec`'s capacity is never
consulted.
**Deviation:** none of `D1`..`D14` applies. This is a local choice about
which of two numbers a bound reads, plus one construction-time check in the
reader.
**Cost of not reproducing:** none. No parity test exercises a full cache;
`max_cache_size` reaches nothing outside this crate.

---

### `save-cache-empty-path-guard-falls-through` — `saveCache`'s uninitialized-path guard logs and then runs the write anyway, subscripting a cache the guard implies is empty — not-reproduced

**Upstream:** `moveit_kinematics/cached_ik_kinematics_plugin/src/ik_cache.cpp:224-257`,
the guard at `:226-227`, verified at the pinned `e017c91e`.
**Port:** `crates/moveit-kinematics/src/ik_cache.rs`, `IkCache::save`.
**Symptom:** `if (cache_file_name_.empty()) RCLCPP_ERROR(getLogger(), "can't
save cache before initialization");` has no `return`. Execution continues
into the write: `:231` constructs an `ofstream` on the empty path, which
fails and sets the stream's error state; `:235-236` read `ik_cache_[0]` for
the tip count and config size; `:239-256` write to the failed stream. The
one state the guard names — `cache_file_name_` empty, so `initializeCache`
never ran — is also the state in which `ik_cache_` is empty, so
`ik_cache_[0]` subscripts an empty vector. That subscript is kept in bounds
only by the two callers' incidental non-emptiness (`~IKCache` at `:64-67`
tests `!ik_cache_.empty()` first, and `updateCache` calls `saveCache`
immediately after a `push_back`), not by anything in the function;
`saveCache` is `protected` (`cached_ik_kinematics_plugin.hpp:131`, in the
section opened at `:127`), so no third caller exists to reach it today.
Separately, `saveCache` returns `void` and never inspects the stream, so a
write that produced nothing — an unwritable directory, a full disk, this
empty path — is indistinguishable from one that succeeded, including to
`~IKCache`, whose whole body is this call.
**Evidence:** a read of the control flow, plus a read of the two call sites
and of the declaration's access section. Not oracle-confirmed.
**Status:** `not-reproduced`. `IkCache::save` takes the path as an argument
rather than holding an initialize-time member, so "saved before
initialization" is not a representable state; it returns `Result`, and the
`std::fs::write` error is mapped with the path in the message.
`CachedIkSolver::save_cache` propagates it.
**Deviation:** none of `D1`..`D14` applies. Passing the path per call and
returning `Result` are local API-shape choices in this crate.
**Cost of not reproducing:** none. The upstream member also carries the
mangled filename built at `:88-91`, which this port does not have — see
`crates/moveit-kinematics/src/ik_cache/format.rs`'s module doc for why the
filename's `max_cache_size`/threshold components are not reproduced.

---

### `cached-ik-accumulate-return-discarded` — Three `std::accumulate` calls that build a cache's name throw the result away, so the name loses every component after the first — not-reproduced

**Upstream:** `moveit_kinematics/cached_ik_kinematics_plugin/include/moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_plugin-inl.hpp:81-83`
and `moveit_kinematics/cached_ik_kinematics_plugin/src/ik_cache.cpp:342-349`
(`IKCacheMap::getKey`), verified at the pinned `e017c91e`.
**Port:** none. `CachedIkSolver` wraps one solver for one group and takes
its cache path from the caller, so it has no name to build; `IKCacheMap`
has no port at all (`D4` replaces pluginlib's plugin-per-group shape with
the `KINEMATICS_SOLVERS` registry).
**Symptom:** `std::accumulate(first, last, init)` returns the accumulated
value and leaves `init` untouched. All three calls discard it. At
`-inl.hpp:82`, `cache_name` is initialized to `base_frame` and the
accumulate meant to append the tip frames does nothing, so `cache_name`
stays the bare base frame; the cache file is then named
`robot_id + group_name + "_" + cache_name + ...` (`ik_cache.cpp:88-91`), so
two initializations of one group under different tip frames — which is what
`CachedMultiTipIKKinematicsPlugin` exists for — resolve to the same file and
share seeds whose poses are poses of different links. In
`IKCacheMap::getKey` the same mistake is fatal rather than partial: `key`
starts empty, both accumulates are discarded, and the only surviving
statement is `key += '_'`, so the function returns `"_"` for every input.
Every distinct (fixed, active) joint-name pair therefore maps to one map
entry and one cache.
**Evidence:** a read of `std::accumulate`'s contract and of the three call
sites. Reachability differs sharply between them and is unmeasured for
both: `-inl.hpp:82` is on the live `initialize` path, while `IKCacheMap` is
constructed and called nowhere in the upstream tree — a search for the
identifier finds only its own declaration and definitions — so `getKey` is
dead code that would become live the moment anything used the class.
**Status:** `not-reproduced`. Nothing in this port derives a cache identity
from string concatenation; `CachedIkSolver::from_cache_file` and
`save_cache` name the file explicitly, and the joint count the file was
written for is checked against the solver's on load rather than encoded
into a name.
**Deviation:** `D4` is what removes the plugin-per-group naming scheme these
functions serve; the discarded return itself maps to no `D` class.
**Cost of not reproducing:** none. No parity test and no oracle operation
constructs a cache name.

---

### `ik-cache-map-first-update-dropped` — `IKCacheMap::updateCache` creates the missing cache and returns without storing the solution that caused it to be created — not-reproduced

**Upstream:** `moveit_kinematics/cached_ik_kinematics_plugin/src/ik_cache.cpp:323-339`,
verified at the pinned `e017c91e`.
**Port:** none — `IKCacheMap` has no port, for the reason given under
`cached-ik-accumulate-return-discarded`.
**Symptom:** the function looks the key up and branches. The found branch
forwards to `IKCache::updateCache(nearest, poses, config)`. The missing
branch inserts a null entry, `new IKCache`s it, calls `initializeCache` on
it — and stops. `nearest`, `poses` and `config`, the whole point of the
call, are never used on that path, so the first solution for any key is
always discarded and only the second onwards is cached. Two smaller things
ride along: the new cache is initialized through the four-argument
`initializeCache` overload, taking default `Options` rather than the
configured ones the single-cache path passes, and the inner `auto it`
shadows the outer one, which is what makes the missing store easy to read
past.
**Evidence:** a read of the control flow. Unreachable in the upstream tree
as it stands: nothing constructs `IKCacheMap`. Not oracle-confirmed.
**Status:** `not-reproduced`. Recorded rather than ported: this port's
`CachedIkSolver` holds one `IkCache` for one solver, and `IkCache::update`
has no create-on-miss path to forget to finish.
**Deviation:** none of `D1`..`D14` applies to the dropped store itself;
`D4` is what removes the class it lives in.
**Cost of not reproducing:** none. No caller, no parity test, no oracle
operation.

---

### `set-from-ik-zero-timeout-is-not-single-attempt` — `computeCartesianPath` passes `timeout = 0.0` to get one deterministic IK attempt and gets 0.5 s of random re-seeding instead — not-reproduced

**Upstream:** `moveit_core/robot_state/src/cartesian_interpolator.cpp:453-456`
(verified at the pinned `e017c91e`), and the same `0.0` at `:260` and `:94`.
The reinterpretation is
`moveit_core/robot_state/src/robot_state.cpp:2010-2011`; the value it
substitutes is `moveit_core/robot_model/include/moveit/robot_model/joint_model_group.hpp:74`;
what the solver then does with it is
`moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp:369-409`.
**Port:** `crates/moveit-kinematics/src/cartesian_interpolator.rs`,
`PathRun::solve_link_pose`.
**Symptom:** `cartesian_interpolator.cpp:453-454` states the intent in a
comment — "Explicitly use a single IK attempt only (by setting a timeout of
0.0), using the current state as the seed. Random seeding (of additional
attempts) would create large joint-space jumps" — and then passes `0.0`.
`setFromIK` reads that argument as a sentinel, not as a value:
`if (timeout < std::numeric_limits<double>::epsilon()) timeout =
jmg->getDefaultIKTimeout()`, and `KinematicsSolver::KinematicsSolver()`
initialises `default_ik_timeout_(0.5)`. So the request reaching
`searchPositionIK` is a 0.5-second budget, and that function's body is
`do { ++attempt; if (attempt > 1) getRandomConfiguration(...); CartToJnt(...);
... } while (!timedOut(start_time, timeout))` — precisely the random
re-seeding the comment says it is avoiding, for half a second per waypoint.
`0.0` is the one value that cannot mean "no time"; every positive value
smaller than the default would have.
**Evidence:** a read of the control flow across the four files cited above.
Not oracle-confirmed — the divergence is wall-clock-dependent by
construction (it only shows up when the first attempt fails and there is
time left to retry), so an oracle comparison would be a race, not a
measurement. The two facts the read rests on are single lines and were each
read at the pinned sha: the `< epsilon` substitution and the `(0.5)` member
initializer.
**Status:** `not-reproduced`.
**Deviation:** `D1` in the sense that the wall-clock timeout is not ported
at all — `crate::SolverParams::max_restarts`, a retry *count*, replaced it
crate-wide before this module existed, and that replacement is documented at
`SolverParams::max_restarts` and in `moveit-kinematics`'s own crate doc
("§4.9, no wall-clock timeout"). The consequence for this bug is structural
rather than a fix applied to it: with no timeout parameter there is no
sentinel value left to reinterpret, so the port cannot express the input
that triggers this. A caller wanting the single deterministic attempt the
upstream comment describes builds its solver with `max_restarts = 0` and
gets exactly that; a caller leaving the default gets a *bounded, seeded,
reproducible* retry count rather than an unbounded wall-clock one.
**Cost of not reproducing:** none measurable. There is no oracle op for
`computeCartesianPath`, and the reproducing behaviour is not
deterministic, so there is no number that would move. The visible cost is
that this port's waypoint IK is only as persistent as the caller's
`max_restarts`: a target the upstream 0.5-second re-seeding loop would have
eventually found from a random seed, and this port's `max_restarts = 0`
caller will not, shortens the achieved fraction. That is the behaviour the
upstream comment asked for.

---

### `validate-and-improve-interval-percentage-discarded` — the bisection computes a partial-progress fraction into a `double&` that every caller drops, so the returned fraction can understate the trajectory returned with it — not-reproduced

**Upstream:** `moveit_core/robot_state/src/cartesian_interpolator.cpp:63-109`
(`validateAndImproveInterval`, `double& percentage` at `:65`), with its only
caller at `:253-269`.
**Port:** `crates/moveit-kinematics/src/cartesian_interpolator.rs`,
`PathRun::validate_and_improve_interval` and `PathRun::achieved`.
**Symptom:** `percentage` is taken by reference and written on the
subdivision path — `:100-101` saves `old_percentage` and sets `percentage =
percentage - half_width` before recursing into the first half, `:106`
restores it before the second. The value that survives a *failure* is
therefore the parameter of the deepest sub-interval that was entered, which
is the partial progress the by-reference parameter exists to report. No
caller reads it. In `computeCartesianPath`'s loop the call sits inside
`if (!setFromIK(...) || !validateAndImproveInterval(..., percentage, ...))
break;` (`:260-264`) and `last_valid_percentage = percentage` is at `:268`,
*after* the `break` — so on every failing path the mutated value is
discarded, and on every succeeding path `percentage` has been restored to
the value it entered with. The parameter is dead output.
What that costs is not just a dead parameter: `traj` is never rolled back.
If an interval bisects, its first half validates (pushing the mid state at
`:86`) and its second half then fails, the returned trajectory holds a
waypoint at parameter `percentage - half_width` while the returned fraction
is `(i - 1) / steps` — the trajectory is longer than the fraction claims,
and the two no longer name the same point on the path.
**Evidence:** a read of the control flow. The claim rests on three
statements, each a single line read at the pinned sha: the by-reference
parameter (`:65`), the `break` (`:264`) preceding the only read of
`percentage` (`:268`), and the unconditional `traj.push_back` (`:86`) with
no matching erase anywhere in the file. Not oracle-confirmed: there is no
oracle op for `computeCartesianPath`, and reaching the divergent case needs
an interval that bisects once, accepts its first half and fails its second —
constructible in principle, not something the port can measure against C++
here. It has since been constructed on the port side:
`crates/moveit-kinematics/tests/cartesian_interpolator.rs`'s
`a_rejected_path_keeps_the_fraction_its_deepest_accepted_leaf_reached`
drives `translational: 1e-8, max_resolution: 1e-5` on the 0.10 m fixture,
where the recursion accepts its leftmost leaf twelve levels down and then
fails, and pins the divergence exactly: the port returns `1/11 / 4096`
(`2.21946022727272733e-5`, the accepted leaf's parameter) where upstream's
`last_valid_percentage` would still be `0.0`. Still not oracle-confirmed —
the construction fixes what each side reports, not a measurement of C++.
**Status:** `not-reproduced`.
**Deviation:** none of `D1`..`D14` applies; this is a bug the port declines
under the default policy, not an instance of a project-wide decision. The
port splits the parameter's two meanings rather than patching the caller:
`percentage` stays a by-value input (the interval's end parameter) and the
achieved fraction becomes `PathRun::achieved`, written at the single site
that appends a waypoint. That makes the invariant hold by construction —
the returned fraction is the path parameter of the last waypoint in the
returned trajectory, on success and failure alike — rather than making the
caller remember to read an out-parameter it currently does not.
**Cost of not reproducing:** none against upstream numbers; no oracle op and
no parity test covers this path. The behavioural difference is confined to
the case described above: where upstream returns `(i - 1) / steps`, this
port returns the strictly larger parameter of the waypoint it actually
returned. A caller that trusted upstream's fraction to be a *lower* bound on
the trajectory keeps that guarantee; one that trusted it to be exact was
already wrong upstream.

---

### `fcl-distance-sentinel-survives-zero-contacts` — The signed-distance path zeroes the contact geometry but leaves FCL's `-1` sentinel in `distance` when `fcl::collide` finds no contact — not-reproduced

**Upstream:** `moveit_core/collision_detection_fcl/src/collision_common.cpp:613`
(`dist_result.distance = fcl_result.min_distance;`), `:636`
(`if (distance <= 0 && cdata->req->enable_signed_distance)`), `:647-648`
(`std::size_t contacts = fcl::collide(o1, o2, coll_req, coll_res); if (contacts > 0)`),
`:663` (`dist_result.distance = -contact.penetration_depth;`). The `if` at
648 has no `else`.
**Port:** `crates/moveit-collision/src/parry.rs:2271`
**Symptom:** For a pair FCL reports as touching or penetrating
(`distance <= 0`) with `enable_signed_distance` set, line 613 has already
stored `fcl_result.min_distance` — which for an in-collision pair is FCL's
`-1` *sentinel*, not a length. Lines 638-640 then unconditionally zero
`nearest_points[0]`, `nearest_points[1]` and `normal`, and only the
`contacts > 0` branch replaces the sentinel with `-penetration_depth`. At
exact tangency `fcl::collide` returns zero contacts, so that branch is
skipped and the callback returns `distance == -1.0` with both nearest
points at the origin and a zero normal: a record asserting a metre of
penetration while carrying no geometry that could support it. The value
then reaches `DistanceResult::minimum_distance` through a plain `<`, where
it beats every genuine penetration depth in the same scene, so the scene's
reported minimum becomes the sentinel. `enable_signed_distance` is the flag
whose whole purpose is to replace the sentinel with a signed depth; on this
path it performs the zeroing half and not the replacing half.
**Evidence:** both a read of the control flow above and an oracle run — the
strongest pairing in this file. Sweeping the floor's top face through the
tangency point isolates the discontinuity, `e017c91ee`, seed-free:

| floor top `z` | true gap | `robot_collision` | `robot_distance` |
|---|---|---|---|
| `-1e-3`  | `+1e-3`  | `false` | `+1.000000000000001e-3` |
| `-1e-7`  | `+1e-7`  | `false` | `+1.000000000028756e-7` |
| `-1e-9`  | `+1e-9`  | `false` | `+9.999999994736568e-10` |
| `-1e-15` | `+1e-15` | `false` | `+1.038912551220369e-15` |
| `0`      | `0`      | `false` | **`-1.000000000000000e0`** |
| `+1e-15` | `-1e-15` | `true`  | `-1.129411566063279e-15` |
| `+1e-9`  | `-1e-9`  | `true`  | `-9.999999994737827e-10` |
| `+1e-3`  | `-1e-3`  | `true`  | `-1.000000000000001e-3` |

The function is continuous to `~1e-15` on both sides and jumps by `1e15` at
the single point between them. That the jump is the sentinel and not geometry
is confirmed by the `collision` column: at the tie upstream also reports
`false`, which is what "zero contacts were found" means, and it is exactly the
branch at line 648 that the zero-contact case skips.

That column is *not* itself a convention the port could adopt. `fcl::collide`
dispatches per shape pair, and the other exactly-touching pair in this
workspace answers the other way: `octree_world_collision_response.json` case 4
— an octree leaf whose `-x` face lands exactly on a robot box's `+x` face —
returns `robot_collision: true` with `robot_distance: -0.0`, having found a
contact. Same zero gap, opposite answers, and only the pair that found no
contact leaks the sentinel.

The rate is prbt-specific, not general. 10,000 seeded prbt states
(`tools/ci/verify-phase3-collision-sweep.sh`, seed 1): the oracle's reported
minimum robot distance is `-1.0` on `floor/prbt_base_link` in **10,000 /
10,000** states. Across the other four
fixtures' 29,152 disagreeing states (panda 9,543, fanuc 6,113,
dual_arm_panda 3,508, pr2 9,988) the sentinel appears **0** times. That is
geometry, not luck — `fixtures/prbt.urdf`'s `prbt_base_link` collision
cylinder (`length 0.13`, `origin z 0.065`) puts its bottom face at exactly
`z = 0`, which is exactly the top face of the floor box in
`tools/moveit-diff/src/main.rs`'s scene, so every sampled state hits the
tangency case regardless of joint values.
**Status:** `not-reproduced`, and structurally so rather than by choice.
`parry.rs:2271` takes `contact.dist` from `parry3d_f64::query::contact` and
substitutes no sentinel on any path, so `-1.0` is not constructible here.
**Deviation:** none of `D1`..`D14` applies. This is not a policy the port
adopted to route around the defect — parry simply has no in-collision
sentinel to leak.
**Cost of not reproducing:** measured, and it is the entirety of prbt's
`distance` column in `PORTING-PLAN.md` §218.4 — 10,000/10,000 states
disagree with the oracle, at max\|Δ\| `1.000000e0`, which is exactly
`|-1.0 - (-2.775558e-17)|` and so is the sentinel gap rather than a metre of
geometric error. Reproducing it is not reachable even if parity were
preferred: the port would first have to agree with FCL that the tangent pair
has zero contacts, and `query::contact` returns a contact at
`dist = -2.775558e-17` for it. The number obtained would still be a sentinel
rather than a distance, so §218.3 records this as a Phase 3 finding instead
of moving the fixture or the floor to make it disappear.
The reproducer above is pinned as tests in
`crates/moveit-collision/tests/exact_tangency_boundary.rs`:
`no_sentinel_escapes_at_the_tie` fails if this backend ever acquires the
sentinel, and `the_tie_is_decided_below_one_ulp` measures the `-2.775558e-17`
that the `bool` half of the disagreement turns on.

---

### `aggregated-limits-drops-rejected-joint-silently` — `getAggregatedLimits` discards `addLimit`'s `bool`, so a joint its own rule 4 made invalid vanishes from the container — not-reproduced

**Upstream:** `moveit_planners/pilz_industrial_motion_planner/src/joint_limits_aggregator.cpp:109`
(`container.addLimit(joint_model->getName(), joint_limit);`, return value
unused), with the rejecting condition at
`moveit_planners/pilz_industrial_motion_planner/src/joint_limits_container.cpp:55-59`
(`if (joint_limit.has_deceleration_limits && joint_limit.max_deceleration >= 0) … return false;`)
and the rule that produces it at `joint_limits_aggregator.cpp:102-106`.
**Port:** `crates/moveit-planners-pilz/src/joint_limits_aggregator.rs`,
`aggregate_limits` and `AggregationError::NonNegativeDeceleration`.
**Symptom:** `addLimit` has two rejection causes and returns `false` for
both — a non-negative `max_deceleration`, and a name already in the map.
`getAggregatedLimits` calls it as a statement. The joint is then simply
absent from the returned container, with no exception, no return code and
nothing in the signature to say so, while the loop carries on to the next
joint. The first cause is reachable from the aggregator's *own* rule 4:
`:104-105` sets `max_deceleration = -max_acceleration` and
`has_deceleration_limits = true` whenever an override gives acceleration but
not deceleration, so an override of `max_acceleration: 0.0` produces
`-0.0`, and `-0.0 >= 0` is true. A YAML file may hold that value: the
parameter path (`joint_limits_rosparam.hpp`) has no negativity gate of its
own. Upstream's own `JointLimitsAggregator.ExpectedMapSize`
(`test/unit_tests/src/unittest_joint_limits_aggregator.cpp:79-86`) asserts
`getActiveJointModels().size() == container.getCount()`, which is exactly
the invariant this discarded `bool` can break; it passes only because that
fixture's YAML has no zero acceleration in it.
**Evidence:** a read of the two functions at the pinned sha — the unused
return at `:109`, the `return false` at `joint_limits_container.cpp:58`,
and rule 4 at `:102-106`. Not run against C++: reaching it needs a
parameter server and a YAML file, neither of which exists in this
workspace. The port side is measured:
`a_zero_acceleration_override_is_reported_not_dropped` in
`joint_limits_aggregator.rs` drives exactly that override and gets
`NonNegativeDeceleration { max_deceleration: -0.0 }`,
and mutation `A10` in
`doc/assertion-discrimination-ledger-p10-jointlimits.md` — restoring
upstream's discarded `bool` — fails that test and no other.
**Status:** `not-reproduced`.
**Deviation:** none of `D1`..`D14` applies. The fix is structural rather
than a check bolted onto the call: `has_limit` is asked *before*
`add_limit`, so by construction a `false` from `add_limit` has only one
possible cause left, and the two causes become two distinct variants
(`DuplicateJoint`, `NonNegativeDeceleration`) instead of one silent drop.
**Cost of not reproducing:** none against upstream numbers. No oracle op
and no parity test covers `getAggregatedLimits`. The behavioural
difference is confined to inputs upstream drops silently: where upstream
returns a container short one joint, this port returns `Err`. A caller that
checked the container's size — as upstream's own test does — was already
looking for this failure.

---

### `check-position-bounds-multidof-adjacent-members` — the bounds checks pass the address of one `double` member to an interface that reads three or seven, so a multi-DOF joint is checked against whatever follows it in the struct — not-reproduced

**Upstream:** `moveit_planners/pilz_industrial_motion_planner/src/joint_limits_aggregator.cpp:172`
and `:179` (`joint_model->satisfiesPositionBounds(&joint_limit.min_position)`,
`…(&joint_limit.max_position)`) and `:190`
(`joint_model->satisfiesVelocityBounds(&joint_limit.max_velocity)`).
The readers: `moveit_core/robot_model/src/planar_joint_model.cpp:299-307`
(`for (unsigned int i = 0; i < 3; ++i)`),
`moveit_core/robot_model/src/floating_joint_model.cpp:167-177`
(`values[0]`..`values[6]`), and the default
`moveit_core/robot_model/src/joint_model.cpp:117-131`
(`for (i = 0; i < other_bounds.size(); ++i)`).
**Port:** `crates/moveit-planners-pilz/src/joint_limits_aggregator.rs`,
`check_bounds_check_is_supported` and
`AggregationError::MultiDofBoundsCheck`.
**Symptom:** `JointModel::satisfiesPositionBounds(const double*)`
(`moveit_core/robot_model/include/moveit/robot_model/joint_model.hpp:280`)
takes a pointer to a *variable vector*, and each override reads one element
per variable. The aggregator hands it the address of a single `double` data
member. For a one-variable joint that is correct by accident — one
variable, one element read. For a planar joint the planar override reads
`values[0..2]`; for a floating joint the floating override reads
`values[0..6]`; and the same holds for `satisfiesVelocityBounds`, whose
default implementation iterates `other_bounds.size()`. Reading past the end
of the `double` that was pointed at is undefined behaviour, and whatever
bytes it lands on are not positions or velocities of that joint, so the
check answers on data unrelated to the question. The two
`update*FromJointModel` helpers show upstream knew multi-DOF joints reach
this code — each has a `default:` arm warning "Multi-DOF-Joint '…' not
supported" that pins the limit to zero (`joint_limits_aggregator.cpp:131-136`,
`:159-162`) — but those arms are on the branch taken when the joint has
*no* override. The branch that has one goes to the bounds check instead,
where no such arm exists.
**Evidence:** a read of the three call sites and the three readers at the
pinned sha. Not run against C++, and not reachable from upstream's own
fixture: it needs an active planar or floating joint *and* a parameter
entry setting `has_position_limits` or `has_velocity_limits` for that
joint. The `LCOV_EXCL` markers around the multi-DOF arms say upstream's
coverage does not reach this shape either. The port side is measured:
`a_position_override_on_a_multi_variable_joint_is_rejected` and
`a_velocity_override_on_a_multi_variable_joint_is_rejected` in
`joint_limits_aggregator.rs` drive a planar and a floating joint, and
mutations `A35`/`A34` in
`doc/assertion-discrimination-ledger-p10-jointlimits.md` fail one each.
**Status:** `not-reproduced`.
**Deviation:** none of `D1`..`D14` applies. The port answers
`MultiDofBoundsCheck { joint, variable_count }` from one guard covering
both dimensions rather than a per-dimension patch: position and velocity
are broken by the same cause, and a uniform rule at the one place a bounds
check is about to happen leaves no second boundary to special-case later.
Joints with *zero* variables are deliberately not guarded —
`satisfies_position_bounds` and `satisfies_velocity_bounds` both answer
`true` for them without reading the slice, which is upstream's
`FixedJointModel` behaviour and not a defect.
**Cost of not reproducing:** none against upstream numbers. No oracle op
and no parity test covers `getAggregatedLimits`, and the divergence is
confined to inputs on which upstream's answer is undefined: where upstream
would accept or reject a multi-DOF joint's override on unrelated bytes,
this port returns `Err`.

---

## Decision on the pre-policy entries

Asked on 2026-08-05 whether to measure-then-deviate, deviate immediately,
or document only: **document only, code unchanged.**
`chomp-iteration-double-increment`, `attached-body-count-check` and
`kdl-path-circle-nan-scale-rot` are `reproduced-grandfathered` and stay as
they are. `do-smoothing-length-check-operand`,
`get-max-payload-index-space` and `totg-timing-zero-velocity-division`
were found after the decision but are the
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

---

### `count-samples-per-second-returns-a-ratio` — `countSamplesPerSecond` returns a unitless success ratio, not a rate — not-reproduced

**Upstream:** `moveit_core/constraint_samplers/src/constraint_sampler_tools.cpp:72-94`
(the `(sampler, reference_state)` overload; the `(constr, scene, group)`
sibling at `:65-70` is the same name forwarding to it)
**Port:** none — `PORTING-PLAN.md` §225.1 decides the function non-port, and
`crates/moveit-constraints/src/lib.rs`'s declaration audit tags it
`decided`.
**Symptom:** the function accumulates `total` (attempts) and `valid`
(successes) over a loop it runs until one wall-clock second has elapsed
(`:82` `rclcpp::Clock().now() + rclcpp::Duration::from_seconds(1)`, `:92`
the `while`), then returns `static_cast<double>(valid) /
static_cast<double>(total)` (`:93`). That is the fraction of `sample()`
calls that succeeded — dimensionless, and *invariant* under machine speed:
a machine twice as fast raises both `valid` and `total` and leaves the
quotient where it was. A samples-per-second figure is `valid` divided by
the elapsed seconds; elapsed time is never measured (only compared against
`end`) and never divides anything. The name is the function's only
specification — there is no doc comment on either declaration
(`constraint_sampler_tools.hpp:54,56`) — and the body contradicts it.
**Evidence:** a read of the function body at the pinned `e017c91ee`
checkout, plus a read of its one caller. No oracle run: this workspace has
no ROS 2 toolchain to build `moveit_core` with, so the returned number has
not been observed. The read is unambiguous, though — `valid`, `total` and
the returned quotient are the only three quantities in the body, and no
duration value is ever converted to a number.
**Status:** `not-reproduced`.
**Deviation:** none maps. §225.1 declines the whole function on
determinism and cost grounds (a wall-clock-bounded loop, ≥ 1 s per call,
whose output nothing in this workspace could assert on), not on any
`D1`..`D14` policy; the name/return mismatch is an additional reason, not
the deciding one.
**Cost of not reproducing:** none. Upstream's only caller anywhere in
`moveit_core`/`moveit_planners`/`moveit_ros` is its own
`moveit_msgs`-taking sibling, which forwards the `double` straight out as
its own return value without inspecting it, so no upstream branch, threshold
or assertion reads the number either. The quantity itself is measured
deterministically here by
`crates/moveit-constraints/tests/sampler_self_validation.rs`'s
per-configuration `attempted`/`produced` accounting, which fails the sweep
when a sampler's yield falls short of its quota.

### `all-valid-distance-robot-hides-base-overload` — `CollisionEnvAllValid::distanceRobot(state)` returns `0.0` through the concrete type and `max()` through the base pointer callers actually hold — not-reproduced

**Upstream:** `moveit_core/collision_detection/include/moveit/collision_detection/allvalid/collision_env_allvalid.hpp:63-64`
and `src/allvalid/collision_env_allvalid.cpp:114-123` (the two
`virtual double distanceRobot(...)` definitions), against
`include/moveit/collision_detection/collision_env.hpp:202` and `:220` (the
base's `inline double distanceRobot(state, bool verbose = false)` and its
`acm` sibling)
**Port:** `crates/moveit-collision/src/all_valid.rs` — `AllValidCollisionEnv`
implements only the `(request, state, ..) -> DistanceResult` form, so the
split cannot be expressed; `PORTING-PLAN.md` §225.3 records the choice.
**Symptom:** `CollisionEnvAllValid` declares
`virtual double distanceRobot(const moveit::core::RobotState& state) const`
returning `0.0`. It reads like an override of the base's same-named
convenience accessor, and is not one: the base's is
`inline double distanceRobot(const RobotState& state, bool verbose = false) const`
— a **different signature** and, decisively, **non-virtual**. So the derived
declaration overrides nothing; it introduces a new function and, by C++
name hiding, suppresses the base's overloads for unqualified lookup on a
`CollisionEnvAllValid` object. One query then has two answers, selected by
the static type of the expression:

- through the concrete type — `CollisionEnvAllValid v; v.distanceRobot(s)`
  — the derived function runs and returns `0.0`;
- through a base reference or the `CollisionEnvPtr` that
  `CollisionDetectorAllocatorAllValid::allocateEnv` hands back — which is
  how every `PlanningScene` caller reaches this class — the base's inline
  runs, calls the pure-virtual `distanceRobot(req, res, state)` (whose
  all-valid override sets only `res.collision = false`,
  `collision_env_allvalid.cpp:108-112`), and returns
  `res.minimum_distance.distance`, still at `DistanceResultsData::clear()`'s
  `std::numeric_limits<double>::max()` (`collision_common.hpp:356`, reset at
  `:286`).

`0.0` and `max()` are not two spellings of one answer — for a backend whose
entire contract is "nothing is ever in collision", `0.0` is the collision
boundary (`DistanceResultsData::distance`'s own doc: `<= 0` means in
collision) and `max()` is unbounded clearance. The declaration that is dead
through the allocator path is the one carrying the author's intent.

The same shape hits `distanceSelf` harder: `CollisionEnvAllValid` declares
only the three-argument `distanceSelf(req, res, state) override`
(`collision_env_allvalid.hpp:73`), which hides *both* base convenience
overloads (`collision_env.hpp:167,179`), so `v.distanceSelf(s)` on the
concrete type is a compile error rather than a wrong number.

**Evidence:** a read of the four declarations at the pinned `e017c91ee`
checkout, plus a reproduction of the C++ mechanism — not of MoveIt — in a
standalone 50-line model that copies only the declaration shapes
(`CollisionEnv` with a pure-virtual `distanceRobot(req,res,state)`, a
non-virtual `inline double distanceRobot(state, bool = false)` returning
`res.minimum_distance_distance`, a `DistanceResult` whose distance member
starts at `numeric_limits<double>::max()`, and a derived class declaring
`virtual double distanceRobot(const State&) const { return 0.0; }` plus the
three-argument `override`). Built with `g++ -std=c++17`, it prints

```
concrete  v.distanceRobot(s) = 0
base ref  e.distanceRobot(s) = 1.79769e+308
base ref  e.distanceSelf(s)  = 1.79769e+308
```

and, with the concrete-type `distanceSelf` call enabled, fails to compile
with `no matching function for call to 'CollisionEnvAllValid::distanceSelf(State&)'`
/ `candidate expects 3 arguments, 1 provided`. MoveIt itself was **not**
built: this workspace has no ROS 2 toolchain, so the numbers above come
from the model, and what they establish is that the language rule behaves
as claimed on these exact declaration shapes.

Upstream's own test does not see it. `test/test_all_valid.cpp` is one
`TEST(AllValid, Instantiate)` that constructs a `CollisionEnvAllValid` from
an empty `urdf::ModelInterface` and calls no method on it.

**Status:** `not-reproduced`.
**Deviation:** none maps. The port has no `distance_robot(state)`
convenience overload at all — `CollisionEnv`'s Rust form takes the
`DistanceRequest` explicitly because both upstream conveniences must call
`req.enableGroup(getRobotModel())` first — so there is no second answer to
choose between. `AllValidCollisionEnv::distance_robot` returns
`DistanceResult::default()`, whose `minimum_distance.distance` is
`f64::MAX`: the value the base overload produces, and the one consistent
with the class's contract.
**Cost of not reproducing:** none measurable here. No parity test or oracle
comparison covers `CollisionEnvAllValid` — the port's null backend has no
oracle, because upstream's own coverage of it is the instantiate-only test
above. Within this tree the choice is pinned by
`crates/moveit-collision/src/all_valid.rs`'s
`distance_queries_report_maximum_clearance` and by
`crates/moveit-scene/tests/all_valid_selection.rs`'s
`distance_to_collision_through_the_null_backend_is_maximum_clearance`,
whose isolating mutation is exactly this bug's `0.0`
(`doc/assertion-discrimination-ledger-p10-samplers.md`, M5 and M6).

---

### `distance-callback-max-contact-depth` — `distanceCallback` reports the **largest** of up to 200 contact depths as the pair's signed distance, so a penetration depth grows without bound in the *other* body's width — not-reproduced

**Upstream:** `moveit_core/collision_detection_fcl/src/collision_common.cpp:646`
(`coll_req.num_max_contacts = 200;`), `:650-659` (`double max_dist = 0; ... if
(contact.penetration_depth > max_dist)`), `:662-663` (`const fcl::Contactd&
contact = coll_res.getContact(max_index); dist_result.distance =
-contact.penetration_depth;`).
**Port:** `crates/moveit-collision/src/parry.rs`, `accumulate_distance`
**Symptom:** a penetration depth is the length of the shortest translation
that separates two bodies, so it cannot depend on how *wide* the other body
is — widening a floor slab sideways puts no new material between the robot
and the surface it rests in. Upstream's value does depend on it. For a mesh
link `fcl::collide` runs per triangle, and a triangle lying wholly inside a
large box has no separating axis, so FCL reports an escape along a lateral
direction whose length grows with the box. Lines 650-659 then select the
**maximum** over the contact set, which promotes exactly that artifact over
the ~200 geometrically sane contacts beside it. The result is published as
`DistanceResultsData::distance` and, through the plain `<` in the caller,
becomes the whole scene's `minimum_distance`.
**Evidence:** oracle runs at `e017c91ee`, `tools/moveit-oracle/run-oracle.sh`
with `fixtures/panda.urdf`. `panda_link0` in its default state, floor slab
`L x L x 0.1` with its top face held at `z = +0.05` in every case, so the true
overlap is `0.05 m` throughout and only `L` changes:

| `L` (m) | oracle `robot_distance` |
|---|---|
| `0.4`  | `-2.252549386999574e-1` |
| `1.0`  | `-6.442843086554950e-1` |
| `2.0`  | `-1.349881199903309e0` |
| `4.0`  | `-2.763223704646119e0` |
| `8.0`  | `-3.999482770392206e0` |
| `20.0` | `-9.999482770392206e0` |

A factor of 44 across a sweep in which the overlap never moved, and for
`L >= 8` the value is exactly `L/2 - 0.000517` — the half-width of the slab,
which is the lateral escape rather than the vertical one. Dumping the contact
set with `max_contacts_per_pair: 200` separates upstream's `max` selection
from FCL's individual depths:

| `L` | contacts on `panda_link0/floor` | min depth | median depth | max depth | depths above `0.309272` |
|---|---|---|---|---|---|
| `4.0`  | 100 | `0.041524344` | `0.049999832` | `2.763223705` | 32 of 100 |
| `20.0` | 100 | `0.041524344` | `0.049999698` | `9.999482770` | 14 of 100 |

The median is the correct `~0.05` at both widths and is itself width-invariant
to `1.3e-7`; `0.309272` is `panda_link0`'s own diameter, measured as twice the
largest `|vertex|` over the 200 triangles of
`fixtures/meshes/panda_description/meshes/collision/link0.stl`. So every order
statistic below the maximum is bounded by the geometry and the maximum is not,
and upstream's line 655 is what selects the one that is not. That places the
defect on the MoveIt side of the FCL boundary: taking the minimum or the
median of the same contact set would answer correctly.
**Status:** `not-reproduced`. This backend answers from
`parry3d_f64::query::contact`, which returns one contact per pair carrying the
minimum-translation distance, so there is no set to take a maximum over. Its
measured value is `-0.05003249277506257` at `L` of `0.4`, `1.0`, `4.0` and
`20.0` — spread exactly `0.000000e0`, and within `3.3e-5` of the oracle's own
median contact.
**Deviation:** none of `D1`..`D14` applies. The port is not routing around the
defect by policy; it never accumulates a per-triangle contact set at this
layer, so the selection rule that produces the artifact does not exist here.
**Cost of not reproducing:** measured, and it is the entirety of the `distance`
column that `PORTING-PLAN.md` §5 Phase 3 records as missed on panda, fanuc and
pr2 (`2,897x`-`27,384x`). Reproducing it would mean adopting a quantity that is
unbounded in the size of an unrelated object, and would take
`crates/moveit-collision/tests/penetration_depth_scale_invariance.rs` —
`depth_is_invariant_to_floor_width` and
`depth_never_exceeds_the_links_own_diameter` — with it. Widening the clause's
`1e-4` tolerance was never available either: at `L = 20` the divergence is
`9.95 m`, and it grows with `L` without limit, so no fixed tolerance both
admits it and detects anything.

### `pr2-collision-test-asserts-unwritten-result` — Two `ASSERT_FALSE`s read a `CollisionResult` no call ever wrote — not-reproduced

**Upstream:** `moveit_core/collision_detection/include/moveit/collision_detection/test_collision_common_pr2.hpp:280-282`
and `:526-528`.
**Port:**     `crates/moveit-collision/tests/upstream_pr2_harness.rs:186-193`
(the `TestChangingShapeSize` half) — the `ContactPositions` half is not
restated at all, being blocked on `updateStateWithLinkAt`
(`PORTING-PLAN.md` §227.3).
**Symptom:**  Both sites assert on a default-constructed result instead of
the one the call under test filled in. `ContactPositions` declares
`CollisionResult res3;`, calls `checkSelfCollision(req, res2, ...)` — into
`res2`, the result of the *previous* stanza — and then asserts
`ASSERT_FALSE(res3.collision)`. `TestChangingShapeSize` declares
`CollisionRequest req1; CollisionResult res1;` and asserts
`ASSERT_FALSE(res1.collision)` with no call between the declaration and the
assertion. `CollisionResult::collision` has the in-class initialiser
`= false` (`collision_common.hpp:353`), so both assertions hold
unconditionally: they would still pass if the collision checker returned
`true` for every query, or were deleted.

The consequence is not that a test is untidy — it is that upstream's suite
does not check the two things these lines look like they check. The final
non-collision of `ContactPositions`' third pose (two gripper palms 3 m
apart, one rotated) is unverified upstream, and `TestChangingShapeSize`'s
pre-loop baseline is unverified upstream.
**Evidence:** a read of the two stanzas plus the member's initialiser,
quoted above; no oracle run is needed, because the defect is that the value
read is never written rather than that a computed value is wrong. This is
the weakest evidence class, and it is sufficient here only because the
control flow is four lines long in each case.
**Status:**   `not-reproduced`. `upstream_pr2_harness.rs` restates
`TestChangingShapeSize`'s loop assertion (`:543`) and drops the vacuous
`:528`; a Rust test that asserted on an unwritten `CollisionResult` would
not compile against this port's API anyway, `check_robot_collision`
returning its result by value rather than filling an out-parameter, so the
shape that produced the bug does not exist here.
**Cost of not reproducing:** none. Nothing in this port compares against
these two assertions — they are inside a header
`doc/port-coverage.md` classifies `decided-non-port`, and the parity suites
compare against the oracle binary, not against upstream's GoogleTest cases.
