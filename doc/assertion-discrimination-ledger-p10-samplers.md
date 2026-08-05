# Assertion-discrimination ledger — p10-samplers (`AllValidCollisionEnv`)

The eight tests this panel added, all of them for the null collision
backend ported in `crates/moveit-collision/src/all_valid.rs` and its
selection path in `crates/moveit-scene/tests/all_valid_selection.rs`, and
the nine mutations that say what each one discriminates.

The panel's Group A (`constraint_sampler.cpp`,
`constraint_sampler_tools.{hpp,cpp}`) added no assertions — it resolved to
one `ported-elsewhere` and one `decided-non-port` reclassification plus a
`doc/upstream-bugs.md` entry, so it has no rows here.

A null detector is the one backend whose correct answer is also the answer
you get from never calling it, so "the test passes" carries less than usual
and every row below is a mutation, not a reading.

## Method

Every row's evidence is a mutation applied to this worktree, run as

    cargo nextest run -p moveit-collision -E 'test(all_valid)'      # 4 tests
    cargo nextest run -p moveit-scene -E 'binary(all_valid_selection)'  # 4 tests

with all 8 passing at baseline, then reverted. Both mutated files are new
and therefore untracked, so `git checkout --` cannot revert them: the
revert is a byte-for-byte `cmp` against a pristine copy taken before the
first mutation, and every row below was confirmed `[restored]` by that
`cmp` before the next mutation was applied. (M1's first revert attempt used
`git checkout --` and silently did nothing; the file was restored by hand
and the baseline re-run before continuing.)

Where a mutation takes down more than one test, the row says so and names
the family. No row claims exclusivity that a mutation did not show.

## 1. `nothing_collides` — the shared result builder

| # | mutation | fails | stays green | verdict |
|---|---|---|---|---|
| M1 | `collision: false` → `collision: true` | 5 — `optional_result_fields_follow_the_request_not_the_findings`, `check_collision_default_merges_two_empty_halves`, `continuous_check_answers_rather_than_erroring`, `check_collision_answers_by_the_backend_the_caller_named`, `is_state_colliding_answers_by_the_backend_the_caller_named` | `distance_queries_report_maximum_clearance`, `the_fixture_collides_under_a_real_backend` | family-of-5 |
| M2 | `distance: request.distance.then_some(..)` → `distance: Some(..)` (present regardless of the request) | 1 — `optional_result_fields_follow_the_request_not_the_findings` | the other 7 | discriminating |
| M3 | `contacts: request.contacts.then(..)` → `contacts: Some(..)` | 1 — `optional_result_fields_follow_the_request_not_the_findings` | the other 7 | discriminating |
| M9 | `cost_sources: request.cost.then(Vec::new)` → `cost_sources: Some(Vec::new())` | 1 — `optional_result_fields_follow_the_request_not_the_findings` | the other 7 | discriminating |
| M4 | `CollisionDistance::Closest(f64::MAX)` → `Closest(0.0)` | 2 — `optional_result_fields_follow_the_request_not_the_findings`, `check_collision_default_merges_two_empty_halves` | the other 6 | family-of-2 |

M1 is a family by construction, not by accident: `collision: false` is the
one claim this whole class makes, so every test that exercises a
collision-shaped query must move when it moves. The two that stay green are
the two that do not go through `nothing_collides` at all — the distance
queries and the parry-only control — which is what makes the family the
right boundary rather than a collapsed signal.

M2, M3 and M9 are each other's confirmation that the three `Option` fields
are tracked separately: neutralizing one leaves the other two's assertions
in the same test still checking, and the panic names a different line each
time (section 5). All three fail the same single test because that test is
deliberately the one place the request-tracking contract is stated; no
other test sets `contacts` or `cost` at all.

M4 distinguishes the *value* from the *presence*. It fails
`check_collision_default_merges_two_empty_halves`, which M2, M3 and M9 leave
green — so that test is not a duplicate of the fields test, it is the merge
path (`check_collision`'s provided default runs self- then robot-collision
and combines them) carrying the distance through.

M4 leaves both scene tests green, which is the useful negative: the scene's
`distance_to_collision` does **not** route through `CollisionResult`'s
distance field. It reaches `distance_robot`, which is M6.

## 2. The distance queries

| # | mutation | fails | stays green | verdict |
|---|---|---|---|---|
| M5 | `distance_self` returns `minimum_distance.distance = 0.0` (upstream's hidden-overload answer) instead of `DistanceResult::default()` | 1 — `distance_queries_report_maximum_clearance` | the other 7 | discriminating |
| M6 | `distance_robot` returns `minimum_distance.distance = 0.0` | 2 — `distance_queries_report_maximum_clearance`, `distance_to_collision_through_the_null_backend_is_maximum_clearance` | the other 6 | family-of-2 |

M5 and M6 are each other's isolating mutation for the two distance methods:
each fails the unit test, and only M6 also fails the scene test, which pins
that `PlanningScene::distance_to_collision` reaches `distance_robot` and not
`distance_self`. `0.0` is not an arbitrary wrong value — it is the number
upstream's `CollisionEnvAllValid::distanceRobot(state)` returns
(`collision_env_allvalid.cpp:114-123`), so this mutation asks the exact
question the port had to answer, and the answer is recorded in
`doc/upstream-bugs.md`'s `all-valid-distance-robot-hides-base-overload`.

## 3. The continuous form

| # | mutation | fails | stays green | verdict |
|---|---|---|---|---|
| M7 | `check_robot_collision_continuous` returns `Err(Error::other(..))`, as `ParryCollisionEnv` does | 1 — `continuous_check_answers_rather_than_erroring` | the other 7 | discriminating |

The mutation is `ParryCollisionEnv`'s actual body
(`crates/moveit-collision/src/parry.rs:2385`), so this row shows the test
distinguishes this backend's contract from the other backend's rather than
merely from a panic.

## 4. Selection — the row this port exists to prove

| # | mutation | fails | stays green | verdict |
|---|---|---|---|---|
| M8 | in `check_collision_answers_by_the_backend_the_caller_named`, name `&parry` where the test names `&AllValidCollisionEnv` (the scene, state, ACM and request all unchanged) | 1 — `check_collision_answers_by_the_backend_the_caller_named` | the other 3 scene tests | discriminating |

M8 is the one that answers the brief's requirement that a null detector's
test prove it is *selected*, not merely that it returns `false`. The
mutation changes nothing but the type named at the `E: CollisionEnv<..>`
parameter, and the answer flips — so the answer is a function of the named
backend. A test whose `false` came from a scene that never called a backend
would be unmoved by M8.

M6 is the second, independent selection proof and does not rest on `false`
at all: `f64::MAX` is a value only `AllValidCollisionEnv::distance_robot`
produces in this tree, and it stops being produced the moment that method's
body changes.

## 5. The three coarse `is_none()` sites the sweep's scanner emits

`tools/ci/count-coarse-assertions.py` classes `assert!(x.is_none())` as
coarse because `None` has, in general, more than one cause and the
assertion names none of them. These three are the only sites this panel
adds to the sweep's corpus. Each row's evidence is the mutation that makes
*that* line, and only that line, fire — the panic message quotes the
assertion, so the attribution is the runtime's, not a reading.

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-collision/src/all_valid.rs:197` | `assert!(r.distance.is_none())` — a request with `distance: false` gets no distance slot | `optional_result_fields_follow_the_request_not_the_findings` | discriminating | M2 (`distance: Some(CollisionDistance::Closest(f64::MAX))`, dropping the `request.distance` gate) panics with `assertion failed: r.distance.is_none()` and nothing else in the file moves. `None` here has exactly one producer — `bool::then_some` on `request.distance` — so the coarse predicate has a single cause. (The panic prints line 195 under M2 rather than 197 because the mutation collapses a three-line expression to one; the pristine line is 197.) |
| `crates/moveit-collision/src/all_valid.rs:198` | `assert!(r.contacts.is_none())` | `optional_result_fields_follow_the_request_not_the_findings` | discriminating | M3 (`contacts: Some(ContactData::default())`) panics at `all_valid.rs:198:9`, `assertion failed: r.contacts.is_none()`. Single producer, `bool::then` on `request.contacts`. |
| `crates/moveit-collision/src/all_valid.rs:199` | `assert!(r.cost_sources.is_none())` | `optional_result_fields_follow_the_request_not_the_findings` | discriminating | M9 (`cost_sources: Some(Vec::new())`) panics at `all_valid.rs:199:9`, `assertion failed: r.cost_sources.is_none()`. Single producer, `bool::then` on `request.cost`. |

The three mutations are each other's isolating evidence: each fires one of
the three lines and leaves the other two passing inside the same test body,
so the test does not collapse "the result is empty" into one signal. That
is the property the coarse class exists to doubt, and it is why these are
recorded as ledger rows rather than pushed into
`doc/assertion-discrimination-orphans.txt`.

## What is *not* covered

- `the_fixture_collides_under_a_real_backend` is the control, and no
  mutation of `all_valid.rs` can move it — by design, since its whole job
  is to fail loudly if the fixture ever stops colliding and makes the other
  three scene tests vacuous. Its own discriminating mutation would be a
  fixture change (move the sphere out of reach), which is a change to the
  premise rather than to a guard, and it is not recorded as a guard row
  here.
- `AllValidCollisionEnv`'s `Debug`/`Clone`/`Copy`/`Default`/`PartialEq`
  derives are asserted by nothing and are not claimed to be.
