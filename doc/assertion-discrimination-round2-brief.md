# Assertion-discrimination sweep — widened anchor (round 2)

Read this whole file before starting. It supersedes the anchor in your
previous brief.

## 1. The old anchor named the symptom, not the family

The sweep anchor you were given was:

    assert!\(\s*[^;]{0,200}?\.(is_err|is_none)\(\)

That is one *shape* of the defect, not the defect. The family is:

> **an assertion that cannot name which branch of the call under test
> produced the error.**

`matches!(err, Error::Other(_))` is exactly as blind as `.is_err()`
whenever the function under test has more than one `Error::other(...)`
site — and most do. That shape was invisible to the old anchor.

Second anchor, run it too:

    matches!\([^,]+,\s*Error::\w+

84 further sites workspace-wide: `Error::Other` 42, `Error::Construct`
33, `Error::UnknownName` 9. The `Error::Other` ones are the blind
majority — `Error::other(...)` is constructed at 104 sites across the
tree, so matching the variant discriminates almost nothing.

`Error::UnknownName { kind, name }` is the exception: it carries
structured fields, so matching it *can* discriminate — but only if the
assertion checks `kind`/`name`, not just the variant.

## 2. Worked example, measured in-tree

`moveit-planners-chomp`'s
`assign_chomp_trajectory_point_rejects_a_multi_dof_active_joint`
asserts `matches!(err, Error::Other(_))`. That function
(`trajectory.rs:558`) has **two** `Error::other` sites: the
active-joint-count guard at `:566` and the multi-DOF guard at `:574`.
Swap the two messages and the test still passes — it cannot say which
guard fired. Deleting the guard does make it fail, which is why the
delete-the-guard bite-check passed and the defect survived anyway.

## 3. The bite-check standard — two mutations, not one

Deleting the guard is necessary but **not sufficient**. A test earns
"discriminating" only if it also survives the sibling-branch mutation:

1. **Reachability bite:** no-op the guard → the test must FAIL.
   (Proves the test exercises the guard at all.)
2. **Discrimination bite:** leave the guard in place and give it the
   *sibling* branch's error message/variant → the test must STILL FAIL.
   (Proves the test names *this* branch, not merely "some error".)

A test that passes (1) and fails (2) is a site to fix, not a site to
tick off.

### 3a. When the sibling has no payload to swap (`Option`-returning calls)

Added after p3-acm's `moveit-collision` round. If the call under test
returns `Option` and two sibling guards both produce a bare `None`,
mutation (2) does not exist — there is no message or variant to give the
sibling, because the API itself has already discarded the distinction.
Do not report those sites as `discriminating` on the strength of
mutation (1) alone, and do not report them as untestable either. Use the
**isolating mutation**:

> Neutralize guard A only. The test that names A must FAIL, **and the
> test that names sibling B must stay GREEN.** Then neutralize B only and
> check the mirror. Run both directions and say so.

The green half is the part that carries the information — it is what
rules out "the input reached B all along". p3-acm's `matrix.rs` `entry()`
work is the model: two sequential guards (row missing / row present but
key missing), mutations M1 and M2 applied and reverted separately, each
breaking only its own guard's tests.

Where several assertions sit in one test, comment out the earlier ones so
the failure isolates to the line you are actually classifying —
`assert!` short-circuits, so an earlier assert failing tells you nothing
about a later one.

**Count causes, not `None` tokens.** "The function has one literal
`None`, so it is single-branch" is not a proof, and neither is
"it returns `Option`, so this sweep does not apply". Measured example:
`moveit-kinematics`' `search_position_ik` has exactly **one** `None` in
its body — the one after the retry loop — and **three** distinct causes
that reach it, each a `continue` inside the loop: `cart_to_jnt` returning
`None`, the consistency-limit check failing, and the solution callback
rejecting. A token count says 1. The answer is 3. Count the paths that
*reach* the return, not the returns.

(Both of that function's tests do survive the isolating mutation —
no-op the consistency guard and
`consistency_limit_gates_a_convergent_solution_by_distance_from_seed`
fails while `solution_callback_gates_acceptance_independent_of_convergence`
stays green. They are `discriminating`. The point is that the
`Option`-returning shape is what made them *look* exempt, and the
exemption would have been wrong on a function one `continue` away.)

**But the honest verdict may be neither.** If two sibling guards collapse
to an indistinguishable `None` *and an in-tree caller needs to know which
fired*, the defect is the API, not the test. Check the call sites before
concluding; do not reason from the signature. That is a D6-shaped finding
(an ambiguous result silently collapsed) and it goes in your report as a
finding, not as a test verdict.

The model to copy is `a74a310`
(`moveit-geometry`, `cylinder_negative_radius_is_an_error`): it asserts
`err.to_string().contains("radius")`, and its doc comment records the
message-swap bite-check that proved it. `be9fd46` in the same crate is
the other model — there the sibling is `try_convex_hull` failing on its
own, which a bare `.is_err()` could not be told apart from.

Prefer, in order:
- structured fields where they exist (`Error::UnknownName { kind, name }`)
- `err.to_string().contains("<distinguishing phrase>")`
- `assert_err_mentions` (`moveit-constraints/tests/decide.rs`) where it fits

Do **not** introduce new `Error` variants to make assertions easier —
that question was asked and decided; the rationale is in
`moveit-error/src/lib.rs`'s module docs. Structure the error only when
the distinguishing fact is data a *caller* would read.

## 4. Per-crate site counts

> **CORRECTION, issued at `f6cbbb5`. The table further down this section
> is wrong and is kept only so you can see what you were given.** Use the
> corrected table immediately below instead.
>
> I re-measured on a tree that is byte-identical to `712dafe` for
> `moveit-collision` (`git diff 712dafe f6cbbb5 -- crates/moveit-collision`
> is empty) and got **42**, not the 55 that table claims. Workspace-wide
> the first anchor reads **237** at `712dafe`, not 335. The second anchor
> reproduces exactly at 84, which is how I know the method is sound and
> the first figure was the error. I have not reconstructed what produced
> 335; it is not any variant I could find (`assert_eq`/`assert_ne`
> included: 237; `is_ok`/`is_some` included: 319; bare occurrences
> anywhere: 267).
>
> **Do not treat any number as a target.** Counting this family by regex
> is unreliable in both directions, measured at `f6cbbb5`:
>
> - **It over-counts.** A bare `rg '\.(is_err|is_none)\(\)'` matches the
>   doc comments *you have been writing this round* — `// a bare
>   `.is_err()` cannot say which fired`. In `moveit-trajectory` that is 11
>   of 27 hits.
> - **It under-counts.** The `assert!\(\s*[^;]{0,200}?` anchor misses the
>   multi-line `assert!(\n    result.is_err(),\n    "message"\n)` form and
>   anything whose assert body contains a `;`. 23 real sites workspace-wide.
>
> So the anchor is a **starting point, not a census**. Enumerate by
> reading the test modules of your crates, and report the count *you*
> measured together with the exact command that produced it. If your
> number disagrees with mine, yours wins — say so and show the command.
>
> **Second correction, from p6-totg: the anchor2 column below is also
> slightly low.** I ran `matches!\([^,]+,\s*Error::\w+` without `-U`, so
> it misses `matches!` calls split across lines. Re-run with `-U`, the gap
> is 2 sites workspace-wide and both are in `moveit-planners-chomp` —
> 13 not 11 now, 20 not 18 at `712dafe`. Every other crate is unchanged.
> Use `-U` on **both** anchors.
>
> **Third correction, from p9-ros: anchor2 also over-counts comments.**
> I flagged this for the bare `is_err` anchor and then failed to apply it
> to anchor2. `rg -U 'matches!\([^,]+,\s*Error::\w+'` matches the doc
> comments the panels wrote *about this defect family* — lines like
> ``/// `matches!(err, Error::Other(_))` alone cannot tell a test that…``.
> Workspace-wide that is **11 false positives**: `ros/moveit-ros` 39→34,
> `moveit-planners-chomp` 13→8, `moveit-geometry` 2→1. Anchor2 is
> **57**, not 68. Filter comment lines on both anchors.
>
> So the anchor is wrong in three separate ways, each found by a
> different panel: it under-counts multi-line calls without `-U`
> (p6-totg), it over-counts doc comments (p9-ros), and it misses
> `assert!` bodies containing a `;` or exceeding the `{0,200}` cap (me).
> This is why the deliverable is the classified table, not a number.
>
> Independently measured by the owning panels, all confirmed by me:
> `moveit-collision` 42 (I said 55) · `moveit-planners-pilz` 30 (37) ·
> `moveit-trajectory` 33 (47) · `moveit-planners-chomp` 26 (24) ·
> `moveit-planners-stomp` 7–8 (19) · `moveit-octomap` 9 (15) ·
> `moveit-geometry` 34–38 (59) · `moveit-state` 9 (12) ·
> `moveit-planners-sbp` 4 (7) · `moveit-kinematics` 3 (5) ·
> `moveit-scene` 13 (13, correct) · `moveit-metrics` 0 (0, correct).
> Every disagreement was in the same direction: mine was high.
>
> Corrected counts at `f6cbbb5`. `bare` excludes comment-only lines and
> still includes any production-code `.is_none()`, so it is an upper
> bound; `anchor1` is a lower bound. The true count is between them.
>
> | crate | anchor1 (low) | non-comment bare (high) | anchor2 |
> |---|---|---|---|
> | moveit-collision | 42 | 42 | 0 |
> | moveit-geometry | 34 | 35 | 1 |
> | ros/moveit-ros | 2 | 2 | 50 |
> | moveit-planners-pilz | 24 | 25 | 1 |
> | moveit-distance-field | 18 | 20 | 0 |
> | moveit-model | 16 | 20 | 0 |
> | moveit-trajectory | 16 | 16 | 0 |
> | moveit-scene | 13 | 15 | 0 |
> | moveit-octomap | 9 | 13 | 0 |
> | moveit-state | 9 | 10 | 0 |
> | moveit-planners-stomp | 8 | 8 | 0 |
> | moveit-planners-chomp | 6 | 6 | 11 |
> | moveit-smoothing | 0 | 0 | 11 |
> | moveit-constraints | 3 | 8 | 3 |
> | moveit-planners-sbp | 4 | 5 | 0 |
> | moveit-sampling | 4 | 4 | 0 |
> | moveit-kinematics | 2 | 3 | 1 |
> | moveit-planning | 2 | 2 | 0 |
> | moveit-srdf | 0 | 1 | 0 |
> | moveit-error | 0 | 0 | 0 |
>
> 212–235 first-anchor sites plus 78 second-anchor sites remain at
> `f6cbbb5`. The **routing table at the end of this section is
> unaffected** — every crate is still owned by the same panel.
>
> **Fourth correction, opened by p1-fixtures and measured out by me at
> `0bf4707`. Anchor2 is blind in three further ways, and together they
> hide 42 sites — more than half again what it finds.** p1-fixtures
> reported `moveit-metrics` anchor2 = 3 where I had twice restated 0.
> They were right and I was wrong; I checked, and the reason generalises:
>
> 1. **A comma inside the scrutinee.** `matches!\([^,]+,\s*Error::\w+`
>    requires the `Error::` to sit right after the *first* comma. Any
>    scrutinee that is itself a call with two or more arguments —
>    ``matches!(\n metrics.manipulability(&posed, "no_such_group", false),\n Err(Error::UnknownName { .. })\n)`` — consumes that comma and the
>    anchor never matches. `-U` does not help; `[^,]+` cannot cross a
>    comma no matter what else is set.
> 2. **A fully-qualified path.** The anchor wants a literal `Error::`
>    immediately after the comma, so `moveit_error::Error::UnknownName`
>    is invisible (`moveit-planners-sbp/src/planning_scene_validity.rs:420`,
>    `moveit-state/tests/jacobian.rs:211`).
> 3. **Crate-local error enums.** The anchor names `Error::` only, so
>    `PlanError`, `PipelineError`, `ResponseAdapterError`, `DecodeError`
>    and `Diagnostic` are outside it entirely. `moveit-planning`'s whole
>    `PipelineError` family and `moveit-planners-sbp`'s `PlanError` sites
>    were never counted. `moveit-octomap`'s `DecodeError` sites were found
>    only because p3-distance-field enumerated by hand instead of trusting
>    the anchor.
>
> **Use these instead. They are enum-agnostic, path-agnostic and
> comma-tolerant**, and they anchor on the *assertion*, which is what the
> family is actually about:
>
> ```
> rg -U -c 'assert!\(\s*matches!\('                        crates/ ros/ tools/
> rg -U -c 'assert!\(\s*[\s\S]{0,300}?\.(is_err|is_none)\(\)' crates/ ros/ tools/
> ```
>
> **Fifth correction, measured at `f831242`. Both published numbers are
> wrong, and the cross-check that was said to validate them measures a
> different anchor.**
>
> **(a) 103 and 422 are `-o | wc -l` artefacts.** `rg -o` prints the
> matched *text*, so a match spanning N lines contributes N lines to
> `wc -l`. The bare anchor spans up to 300 characters, which is why it
> inflates worst. The commands printed above — `rg -U -c` — are the
> right ones and were not what produced those numbers:
>
> | anchor | `-o \| wc -l` | `rg -U -c` | enumerated |
> |---|---|---|---|
> | `assert!\(\s*matches!\(` | 103 | **85** | **85** |
> | `assert!\(\s*[\s\S]{0,300}?\.(is_err\|is_none)\(\)` | 422 | **205** | **205** |
>
> `rg -U -c` agrees with enumeration on **every file in the tree**, both
> anchors, zero disagreements, and dropping comment-only lines changes
> neither total. So `rg -U -c` is trustworthy as printed; only
> `-o | wc -l` is not. The real figures are **85** and **205**.
>
> **(b) The 42/35 cross-check belongs to the second anchor, not the
> first.** The claim was that "the new first anchor sums to exactly 42
> for `moveit-collision` and 35 for `moveit-geometry`". Those figures are
> the **bare** anchor's. `moveit-collision`'s `assert!(matches!(` count is
> **2**; its bare count is 42. `moveit-geometry` is 5 and 35. The
> cross-check is real and still reproduces p3-acm's and p3-shapes'
> independently verified numbers — it just validates the bare anchor.
> `assert!(matches!(` has no independent validation, so treat its 85 as
> unconfirmed and enumerate your own crate.
>
> **(c) The seven-row table below is not the whole hole.** It was derived
> as "`assert!(matches!(` hits in files with **no** anchor2 hit", which
> misses two things: files anchor2 saw but under-counted *within*
> (`ros/moveit-ros/src/constraints/set.rs` 3 vs 1,
> `.../orientation.rs` 2 vs 1), and zero-anchor2 files simply not listed
> (`moveit-geometry/src/bodies.rs` 4, `moveit-srdf/tests/boundaries.rs` 2,
> `moveit-distance-field/src/collision_env_distance_field.rs` 2,
> `moveit-collision/src/world.rs` 2, `moveit-model/src/robot_model.rs` 2,
> `moveit-model/src/joint/urdf.rs` 3, `moveit-scene/src/scene.rs` 1,
> `moveit-constraints/tests/decide.rs` 1). Anchor2 cannot show **39
> sites across 17 files**, not 19 across 7. Several of those files sit in
> crates whose owning panel enumerated by hand rather than by anchor and
> are therefore already covered — but that is a per-crate question for
> the owner to answer, not something the anchor settles.
>
> At `0bf4707` these give ~~**103**~~ **85** and ~~**422**~~ **205**. The 422 is not comparable
> to the old first-anchor column — it counts every `assert!` wrapper, so
> it over-counts where one assert holds several calls; treat it as an
> upper bound and enumerate. ~~The 103, against the published anchor2's 61,
> is the real hole. Cross-check: the new first anchor sums to exactly 42
> for `moveit-collision`, reproducing p3-acm's independently-verified
> figure, and 35 for `moveit-geometry`, reproducing p3-shapes'.~~
> **Superseded by (a) and (b) above: the figures are 85 and 205, and the
> 42/35 cross-check validates the bare anchor, not this one.**
>
> **Sites the round has therefore never seen** (`assert!(matches!(` hits
> in files with no anchor2 hit, comments excluded, enumerated not
> counted):
>
> | file | sites | error type | owner |
> |---|---|---|---|
> | `moveit-planners-pilz/src/trajectory_blender_transition_window.rs` | 7 | `Error::Code` | unrouted |
> | `moveit-planning/src/pipeline.rs` | 4 | `PipelineError::*` | unrouted |
> | `moveit-metrics/src/lib.rs` | 3 | `Error::UnknownName` | p1-fixtures |
> | `moveit-planners-sbp/src/registry.rs` | 2 | `PlanError::*` | p1-robotmodel |
> | `moveit-planning/src/response_adapters/add_time_optimal_parameterization.rs` | 1 | `ResponseAdapterError` | unrouted |
> | `moveit-planners-sbp/src/planning_scene_validity.rs` | 1 | qualified `Error::` | p1-robotmodel |
> | `moveit-state/tests/jacobian.rs` | 1 | qualified `Error::` | p1-robotmodel |
>
> `Error::Code` is the shape the brief's own worked example warns about —
> a shared constant that cannot tell sibling guards apart — so the seven
> pilz blender sites are the highest-yield unswept block in the tree.
>
> So the anchor is wrong in **six** independent ways, five of them found
> by a panel rather than by me. Do not treat any anchor as the
> population. The count is a starting point for enumeration; the
> enumeration is the deliverable.

### Superseded table (measured at 712dafe, first-anchor column wrong)

| crate | `is_err`/`is_none` | `matches!` | total | owner |
|---|---|---|---|---|
| moveit-geometry | 58 | 1 | 59 | p3-shapes |
| **ros/moveit-ros** | 7 | **50** | **57** | **p9-ros (newly routed)** |
| moveit-collision | 55 | 0 | 55 | p3-acm |
| moveit-trajectory | 47 | 0 | 47 | p6-totg |
| moveit-planners-pilz | 37 | 0 | 37 | p1-joints |
| moveit-distance-field | 30 | 0 | 30 | p3-distance-field |
| moveit-planners-chomp | 6 | 18 | 24 | p6-totg |
| moveit-planners-stomp | 19 | 0 | 19 | p3-shapes |
| moveit-octomap | 15 | 0 | 15 | p3-distance-field |
| moveit-model | 15 | 0 | 15 | p1-robotmodel (done) |
| moveit-scene | 13 | 0 | 13 | p9-ros |
| moveit-state | 12 | 0 | 12 | p1-fixtures |
| **moveit-smoothing** | 0 | **11** | **11** | **p1-fixtures (newly routed)** |
| moveit-planners-sbp | 7 | 0 | 7 | p1-fixtures |
| moveit-constraints | 4 | 3 | 7 | p1-robotmodel (done) |
| moveit-kinematics | 4 | 1 | 5 | p1-fixtures |
| moveit-sampling | 4 | 0 | 4 | p3-shapes |
| moveit-planning | 2 | 0 | 2 | p9-ros |

419 sites, 18 crates. The earlier "337 / 55 files" figure counted only
the first anchor.

**Final routing.**

> **CORRECTION.** The paragraph below claimed I had read all eight system
> prompts directly and that four panels had no scope sentence. I had not,
> and that is wrong for three of the four. p3-distance-field refused an
> out-of-fence assignment and quoted its charter back at me; I then read
> all eight from `/proc/<pid>/cmdline` and it was right. The actual state:
>
> | panel | scope sentence | kind |
> |---|---|---|
> | p3-distance-field | "you own `crates/moveit-distance-field` (new) and nothing else" | live, unqualified |
> | p3-shapes | "you own `crates/moveit-geometry` and nothing else" | live, unqualified |
> | p6-totg | "you own `crates/moveit-trajectory` (new) and nothing else" | live, unqualified |
> | p3-acm | "You own exactly these crates: `moveit-model/`, `moveit-collision/`" | live |
> | p1-fixtures | "You own exactly these crates: `moveit-scene/`, `moveit-metrics/`" | live |
> | p1-joints | "create `crates/moveit-model` containing ONLY the joint layer" | Phase-1, superseded |
> | p1-robotmodel | "complete `crates/moveit-model` by adding LinkModel…" | Phase-1, superseded |
> | p9-ros | none | — |
>
> **Seven of eight are fenced, not four.** The routing table below assigns
> crates outside three of those fences, and two panels acted on it before
> anyone caught it: p3-shapes → `moveit-planners-stomp`/`moveit-sampling`,
> p6-totg → `moveit-planners-chomp`. That work is merged into `main`.
>
> Those assignments stand as **explicit orchestrator overrides**, narrow
> and on the record: extended scope for assertion-discrimination work
> only — no porting, no API changes, no new crates — and they do not
> rewrite any panel's charter. A panel that would rather not accept a
> scope override may decline and have the crate re-routed; declining is a
> legitimate answer, and p3-distance-field's refusal was correct.

The paragraph below is retained as issued:

- **p1-fixtures** and **p3-acm** carry a live ownership fence ("You own
  exactly these crates"). Obey it — it is current.
- **p1-joints** and **p1-robotmodel** carry the original Phase-1 briefs
  ("create `crates/moveit-model` containing ONLY the joint layer"). Those
  are superseded and have been for many rounds — p1-joints has owned
  `moveit-planners-pilz` throughout. Do not decline a row below on their
  authority.
- ~~**p3-distance-field, p3-shapes, p6-totg, p9-ros** have no scope
  sentence at all.~~ **Wrong — only p9-ros.** See the correction above.

| panel | crates | sites |
|---|---|---|
| p1-joints | moveit-planners-pilz | 37 |
| p1-robotmodel | moveit-state, moveit-smoothing, moveit-planners-sbp, moveit-kinematics | 35 |
| p3-distance-field | moveit-distance-field, moveit-octomap | 45 |
| p3-shapes | moveit-geometry, moveit-planners-stomp, moveit-sampling | 82 |
| p6-totg | moveit-trajectory, moveit-planners-chomp | 71 |
| p9-ros | ros/moveit-ros, moveit-planning | 59 |
| p3-acm | moveit-collision | 55 |
| p1-fixtures | moveit-scene, moveit-metrics | 13 |

p1-robotmodel's own rows (moveit-model 15, moveit-constraints 7) are
already closed, so it takes the four crates no charter covers. That is the
whole 419; nothing is unrouted now.

## 5. What to report

Per crate you own, a classified table — one row per site:

- `discriminating` — already passes both bites; say which mutation you ran
- `fixed` — was blind, now names its branch; cite the commit
- `single-branch` — the call under test has exactly one error site, so
  the variant *is* the discrimination; say which site, and say how you
  established it is the only one (an `rg` count of `Error::` constructors
  inside that function, not an eyeball)

`single-branch` is a real verdict, not an escape hatch — but it is the
one an unmeasured sweep will over-use, so it carries the burden of proof.

One commit per site-family fixed, per the repo's one-commit-per-finding
rule. Gate with `-p <crate>` scope; name the scope in your report.
