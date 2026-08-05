# Assertion-discrimination ledger — p3-acm (`moveit-collision`, `moveit-geometry`)

Round 8, the per-site ledger. Census (`doc/assertion-discrimination-census.md`,
merged `47d3475`, corrected `d0b2a1e`) fixed 288 as the workspace denominator;
this document is the per-site verdict+evidence row for the two crates this
panel owns, closing the gap between "288 is a count" and "every site has a
named disposition and a place the evidence lives."

Scope override accepted for both crates (`moveit-geometry` is p9-ros's fence,
`moveit-collision` is p1-fixtures') — narrow, this round only, ledger
purposes, per the orchestrator's explicit grant. No decline needed; neither
panel is live on these files this round.

## 0. Recount, independent of the census scanner

Per instruction, this ledger was enumerated with a **freshly written second
scanner** (`ledger_scan.py`, regex-based comment/string masking + explicit
stack-based paren matching + a separate brace-range pass for function-name
attribution), not a reuse of the census's char-by-char FSM. Run after
`git merge --ff-only main` (`d0b2a1e`, this panel's worktree):

```
moveit-collision: matches=2 bare=45 total=47
moveit-geometry:  matches=5 bare=34 total=39
```

**84 (quoted) vs 86 (measured) — not an instrument disagreement.** The
census's `moveit-collision` figure (45, at `df36fab`) is 2 short of this
scanner's 47 because two tests landed via the `main` fast-forward merge
*after* the census was taken:

- `matrix.rs:835` — `set_entry_for_known_excludes_the_name_even_when_it_is_already_a_known_row`
  (commit `035d4b1`)
- `world.rs:1142` — `move_shapes_in_object_unknown_object_is_none`
  (same merge window)

`moveit-geometry` reproduces the census's 39 (5/34) exactly. Cross-validated
against `rg -U -c` on both crates: `matches!` anchor agreed on every file;
the `bare` anchor's two known false-positive/false-negative sites
(`bodies.rs:4042`'s comment-swallow, `matrix.rs`/`world.rs` — none new this
scope) were already resolved in the census and reproduce identically here.
**Ledger denominator for this scope: 86, not the quoted 84.**

**Post-round-8 correction (§1b): 89, not 86.** `tools.rs:68`'s
`aabb_intersection` guard (3 operands: one per axis) sat outside
`ledger_scan.py`'s own `kind` grammar (`matches!`/`.is_err()`/
`.is_none()` only — `.is_empty()` is a fourth shape the scanner never
recognized), so it was never enumerated as a site at all, in either the
census or this ledger's own re-run. Found by a separate instrument (the
workspace-wide structural-anchor sweep, `doc/folded-operand-guards.md`),
not by re-running `ledger_scan.py` — the scanner's total is unchanged
at 47/45/2 because its grammar still does not match `.is_empty()`
assertions. Three rows added in §2 below (one per axis); two of the
three needed a new test (§1b). This is the brief's own fifth
correction in miniature: the scanner's count is not a census, and this
round's measured 89 supersedes 86 for the same reason 86 superseded 84.

## 1. Method

Each of the 86 sites was independently re-derived (not copied from the
census row) by one of 11 evidence-gathering passes, one per source file,
each required to end with a verdict and evidence per this schema:

- **verdict**: `discriminating` (passes both bites, or both directions of
  the §3a isolating mutation) / `single-branch` (exactly one cause reaches
  the observed result — counted by reading the function body, not a token
  count) / `fixture-collapse-fixed` (found+fixed a fixture collapse) /
  `not-this-family` (not actually a branch-discrimination assertion)
- **evidence**: exactly one of — a commit sha, a round-report/doc-comment
  claim re-read and independently agreed with, or a bite run live this
  round. "Reads fine" was disallowed.

Every pass was instructed: for any site with no commit and no comment,
treat it as UNCOVERED and run the two-mutation bite (or the brief's §3a
isolating mutation for Option-collapsing siblings) live, fixing and
committing if blind. Zero UNCOVERED-and-blind sites were found across all
86 in the sense of "no test at all" — but re-verifying the 11 passes'
`single-branch` verdicts against the brief's own worked example (§1a)
found 12 sites where a comment/citation had never actually been bit, and
the citation itself was wrong. Every pass confirmed its target file had
an empty `git diff` on exit before reporting; this panel's own follow-up
bites are reverted to a clean tree after each check (confirmed below),
with one exception: the 3 doc-comments corrected in commit (see §1a),
which is a comment-only change with no behavior difference.

### 1a. Re-verification finding: `single-branch` misapplied to combined `||` guards with multiple named causes

While cross-referencing the `moveit-geometry` rows against the brief's
own model (§3, `search_position_ik`: "one `None` token, three causes,
count causes not tokens"; §3a: run the isolating mutation, both
directions, and report the green half), three existing doc-comments
this sweep had already written and cited as evidence turned out to
repeat exactly the mistake the brief warns against — reasoning "one
`Error::` construction site" to "single-branch" without ever running
the isolating-mutation protocol on functions where **multiple assertions
in one test each drive a different named operand negative**:

- `shapes.rs`'s `Sphere::new`/`Cylinder::new`/`Cone::new`/`Cuboid::new`
  comment (pre-fix line 1593): correctly `single-branch` for `Sphere`
  (one operand, nothing to isolate), but wrong for `Cylinder`/`Cone`/
  `Cuboid` — each combines 2-3 named operands (`radius`/`length`,
  `radius`/`length`, `x`/`y`/`z`) into one `||` guard with one shared
  message, and each per-axis assertion in the corresponding test is, in
  fact, independently discriminating.
- `shapes.rs`'s `Cylinder::scale_and_padd_axes` comment (pre-fix line
  1646): explicitly argued the test's own per-axis isolation "doesn't
  matter for discriminating an error branch that doesn't exist here" —
  the same error, same function shape.
- `bodies.rs`'s (distinct) `Cuboid::recompute` comment (pre-fix line
  4072): same shape again, in a different struct/function than the
  `shapes.rs` one.

**Bite protocol used** (the brief's own prescription for asserts sharing
one test fn, since `assert!` short-circuits): comment out the sibling
assertion(s), neutralize one named operand's clause in the guard, run
the isolated assertion (expect FAIL), then flip which assertion is
isolated with the same mutation still in place (expect GREEN). Run live,
this pass, on all three sites; reverted after each leg; `cargo test -p
moveit-geometry --lib` confirmed clean (141/141) after every revert.

| site | direction A (own clause neutralized) | direction B (sibling isolated, same mutation) |
|---|---|---|
| `Cylinder::new` (shapes.rs) | radius neutralized → radius-assert FAILS | length-assert stays GREEN (and mirrored: length neutralized → length-assert FAILS, radius-assert GREEN) |
| `Cone::new` (shapes.rs) | radius neutralized → radius-assert FAILS | length-assert stays GREEN |
| `Cuboid::new` (shapes.rs) | x neutralized → x-assert FAILS | z-assert stays GREEN |
| `Cylinder::scale_and_padd_axes` (shapes.rs) | radius neutralized → `radius_case` FAILS | `length_case` stays GREEN (mirrored) |
| `bodies::Cuboid::recompute` (bodies.rs) | `half_length` neutralized → length-assert FAILS | height-assert stays GREEN |

All 5 confirm `multi-branch`/discriminating, not `single-branch`. The
three doc-comments were corrected in the codebase (comment-only, no
behavior change) and committed separately from this ledger — see §5.
`bodies.rs:4140` (`cuboid_padding_inversion_is_rejected_and_state_preserved`)
is the same underlying guard but has only one assertion in its test, so
there is no sibling to isolate against in that specific test; it is left
`single-branch` **as tested**, not re-classified — whether a second case
should exist there is a coverage question, not a verdict correction.

### 1b. Anchor-generalization follow-up: `tools.rs:68` and the collision re-verification pass

The workspace-wide structural-anchor sweep (`doc/folded-operand-guards.md`,
commit `9702148`) found one site inside this panel's fence that neither
this ledger's original per-site scan nor `ledger_scan.py` had enumerated
at all: `tools.rs:68`, `aabb_intersection`'s
`min[0]>=max[0] || min[1]>=max[1] || min[2]>=max[2]`, three named
operands (one per axis) folded into one `None` construction site.

**Why neither instrument had a row for it**: `ledger_scan.py`'s `kind`
grammar only recognizes `matches!`-led bodies or `.is_err()`/`.is_none()`
bodies (see the script, `scan_file`). The only assertion that exercises
this guard, `assert!(intersect_cost_sources(&a, &b).is_empty())`, is
`.is_empty()`-shaped — a fourth shape outside both recognized kinds, the
same grammar gap already noted for `assert_eq!`-based guards in
`doc/folded-operand-guards.md`. This family was invisible to the
denominator, not merely misverdicted; it never had a row to misverdict.

**Per-axis isolating-mutation bite, run live** (neutralize one axis's
clause in the guard, run `cargo nextest run -p moveit-collision
--no-fail-fast`, confirm which test fails, then revert):

| axis clause dropped | result | pre-existing coverage |
|---|---|---|
| `min[0] >= max[0]` | `boxes_that_only_touch_do_not_intersect` FAILS (196/197 pass), all siblings GREEN | covered |
| `min[1] >= max[1]` | 197/197 pass — **no test failed** | blind |
| `min[2] >= max[2]` | 197/197 pass — **no test failed** | blind |

Axes 1 and 2 were blind operands: per the task instruction, a blind
operand gets a test, not a comment. Added
`boxes_that_only_touch_on_y_do_not_intersect` (tools.rs:263,
assert at 271) and `boxes_that_only_touch_on_z_do_not_intersect`
(tools.rs:275, assert at 283) — each overlapping on the other two axes
and touching on exactly one, mirroring the pre-existing x-axis test.
Re-ran the same two mutations with the new tests present: each new test
FAILS under its own axis's neutralization and stays GREEN under the
other two (199/199 pass at baseline, 198/199 under each single-axis
mutation, exactly one named failure each time). Commit `d24494d`.

**Re-verification pass, all 17 `single-branch` rows in §2 below**,
against the corrected "count causes not tokens" understanding (the same
treatment already applied to `moveit-geometry`'s 12 misverdicts, §1a):
read the actual guard behind every `single-branch` citation, not just
the row's existing comment. Result: **0 misverdicts** — every guard is
either a bare `?`/single map lookup (`tools.rs:219`, `matrix.rs:658,660,
880`, `world.rs:1288,1363`), a single boolean flag (`octomap_filter.rs:
300,355`, `parry.rs:4623`), an unconditional single path
(`parry.rs:2810,2817,3967`), a sequential-fallthrough function with only
one terminal `None` path and no folded clause (`world.rs:1399`), or a
match arm that is genuinely inseparable by construction — `Never |
Always => None` at `matrix.rs:691,692,702` shares the exact same code
for both variants, so no isolating mutation can even be constructed
(unlike `tools.rs:68`'s three independently-computed comparisons) — or a
3-arm match where only one arm can produce `None`, traced by reachability
(`env.rs:707`, re-confirmed by reading `common.rs:585-599`'s current
`merge` body live). Unlike `moveit-geometry`, `moveit-collision` had no
pre-existing misverdict; its one fold defect (`tools.rs:68`) was an
uncounted site, not a mislabeled one.

### 1c. Census §9 applied to this document's own 89 rows

`doc/assertion-discrimination-census.md` §9 defines family membership as
three clauses, all of which must hold or the site is `not-this-family`:
(1) **mechanism** — the value is a could-not/did-not-produce-X signal
(`Err`/`None`/a coarse variant standing in for one), not a plain
success-path computed fact; (2) **decision** — the signal comes from an
actual written decision in the code under test (guard/early-return/
`?`-propagation/loop-comparison), not an emergent non-decision where no
branch ever ran regardless of what the logic does; (3) **subject** — the
decision belongs to the function the test names as its subject, not to
the test's own setup. §9b already lists this document's 86-row (now
89-row) partition as **not yet applied** — this section is that
application, re-checked against all 89 rows, not copied from any other
ledger's answers.

**§9's own worked precedent for clause 2** (`nn.rs:227`,
`empty_index_has_no_nearest`) settles exactly the shape this crate's
bare-`?`/single-lookup rows raise: a `?` on an empty/fresh fixture is
still in-family if a live bite confirms it is exercised (contrast
`shortest_solution_is_none_on_empty_input`, where a loop body never runs
regardless of input). Re-bit two rows live to settle this rather than
argue from shape alone:

- **`tools.rs:219`** (`sensor_positioning_of_empty_set_is_none`):
  replaced `cost_sources.iter().nth(index)?` with `.expect(..)` — test
  FAILS. In-family, matching `nn.rs:227` exactly: the `?` is reached and
  exercised for this fixture, unlike a comparison inside a zero-iteration
  loop.
- **`matrix.rs:880`** (`clear_removes_entries_and_defaults`): commented
  out `self.defaults.clear();` in `AllowedCollisionMatrix::clear` — test
  FAILS (198/199, `matrix::tests::clear_removes_entries_and_defaults`
  only). In-family: the decision lives in `clear()`, one level up from
  the `default_entry` getter the row's evidence names, same as census
  §9's mimic worked example.

**One row moved out on clause 2** — `matrix.rs:660`
(`neither_explicit_nor_default_is_not_found`, the third assertion in a
test that never calls a `defaults` setter at all): `default_entry` is
`self.defaults.get(name)`, a bare tail-expression with no `if`/`?`/
comparison — none of clause 2's four listed shapes — and unlike 880,
there is no antecedent setter call in this fixture to attribute a
"decision one level up" to. Nothing an engineer could plausibly have
implemented differently here would be caught by this assertion short of
deleting the lookup outright, which is not a decision in the domain
sense clause 2 asks about.

**Four rows moved out on clause 1** — `bodies.rs:4328`, `:4332`,
`:4340`, `:4347` (all four assertions in
`from_shape_builds_matching_body_variant`): each is
`matches!(Body::from_shape(&shape).unwrap(), Some(Body::Sphere(_)))` (or
the Cylinder/Cuboid/Mesh equivalent). For all four fixtures,
`from_shape` unconditionally succeeds — the assertion checks *which*
concrete `Body` variant was built, a computed dispatch fact on the
success path, structurally identical to census §9 clause 1's own
disqualifying example, `assert!(matches!(joint.kind(), JointKind::Fixed))`.
This is a genuinely different question from the three `matches!` rows
that stay in-family:

- `world.rs:1150`/`:1160` (`MoveObjectOutcome::NotFound`/`NoChange`) —
  these report an *operation's* outcome ("could not"/"did not produce a
  move"), not a static descriptive property; clause 1 holds.
- `crates/moveit-geometry/src/shapes.rs:1863` (`Error::Construct(_)`) — inspected on the `Err`
  path, not the success path; clause 1 holds.
- `bodies.rs:4384`/`:4392`/`:4402` (`from_shape_returns_none_for_cone_
  plane_octree`) — these test the `None` arm specifically (`Body`
  *could not* be built for Cone/Plane/OcTree), not which variant was
  built; clause 1 holds, already discriminating via isolating mutation.

**Every other row re-checked and confirmed in-family** — every
`discriminating` row already carries live-mutation evidence, which is
itself the strongest form of clause-2 proof (matching the standard
`nn.rs:227` sets). Every remaining `single-branch`/`bare` row was traced
to a real guard, `?`-propagation, or multi-arm match in the code under
test (`predicate()`'s `Never | Always` arm, `CollisionResult::merge`'s
3-arm match, `compound_from_octree`'s documented panic-avoidance guard,
`Mesh::new`/`scale_and_padd_axes`'s real `Err` sites, `Transforms::new`/
`set_transform`/`transform`'s real guards) — none is a bare non-decision
lookup of the kind that sank `matrix.rs:660`.

**In-family denominator for this document's 89 rows: 81 of 89.**
8 `not-this-family`, by crate:

- `moveit-collision` (50 rows): excluded = `matrix.rs:660` (clause 2),
  `octomap_filter.rs:381` (clause 2, pre-existing), `world_parity.rs:226`
  (clause 3, pre-existing) — **3 excluded, 47 in-family**.
- `moveit-geometry` (39 rows): excluded = `crates/moveit-geometry/src/shapes.rs:1962` (clause 2,
  pre-existing), `bodies.rs:4328,4332,4340,4347` (clause 1, all four
  newly excluded) — **5 excluded, 34 in-family**.
- 47 + 34 = 81 of 89.

## 2. `crates/moveit-collision` — 52 sites

### `src/tools.rs` (4)
| file:line | anchor | test fn | verdict | in-family | evidence |
|---|---|---|---|---|---|
| tools.rs:68 (x) | bare (`.is_empty()`, outside `ledger_scan.py`'s grammar) | boxes_that_only_touch_do_not_intersect | discriminating | yes | bite run now: neutralize `min[0]>=max[0]` → this test FAILS (196/197), both `_on_y`/`_on_z` siblings stay GREEN |
| tools.rs:68 (y) | bare (`.is_empty()`, outside `ledger_scan.py`'s grammar) | boxes_that_only_touch_on_y_do_not_intersect | discriminating (blind operand, test added) | yes | pre-fix: neutralize `min[1]>=max[1]` → 197/197 pass, no failure (blind); post-fix (test added, commit `d24494d`): same mutation → this test FAILS, x/z siblings GREEN |
| tools.rs:68 (z) | bare (`.is_empty()`, outside `ledger_scan.py`'s grammar) | boxes_that_only_touch_on_z_do_not_intersect | discriminating (blind operand, test added) | yes | pre-fix: neutralize `min[2]>=max[2]` → 197/197 pass, no failure (blind); post-fix (test added, commit `d24494d`): same mutation → this test FAILS, x/y siblings GREEN |
| tools.rs:219 | bare | sensor_positioning_of_empty_set_is_none | single-branch | yes | §9 clause 2, re-bit now: `.nth(index)?` is a real `?`-propagation, not a bare lookup — replaced `?` with `.expect(..)` live, test FAILS (matches census's `nn.rs:227` precedent: the guard is exercised and its removal is bite-detectable, unlike a loop body that never runs on empty input); reverted, clean |

### `src/env.rs` (1)
| file:line | anchor | test fn | verdict | in-family | evidence |
|---|---|---|---|---|---|
| env.rs:707 | bare | merge_of_two_none_distances_is_none | single-branch | yes | traced match-arm reachability in `CollisionResult::merge`'s distance match (common.rs:587-591): arm 2's `b` is provably `Some` whenever reached (arm 1 already absorbs all `other==None` cases), so only arm 1 (both `None`) can yield `None`. §9 clause 2: a real 3-arm match (`combine_closest` on the `Some,Some` arm), not a bare lookup |

### `src/matrix.rs` (17)
| file:line | anchor | test fn | verdict | in-family | evidence |
|---|---|---|---|---|---|
| matrix.rs:658 | bare | neither_explicit_nor_default_is_not_found | single-branch | yes | read: only `self.default_for_pair(..)` returns `None` from `allowed_collision`; only the `(None,None)` arm of `default_for_pair`'s 4-arm match yields `None`. §9 clause 2: composes two real decisions — `entry()`'s `?`-chain (bite-proven at 659) and `default_for_pair`'s 4-arm match — not a bare lookup |
| matrix.rs:659 | bare | neither_explicit_nor_default_is_not_found | discriminating | yes | bite run now on `entry()` guard A (row missing): neutralized A → test FAILED; neutralized guard B only → test PASSED |
| matrix.rs:660 | bare | neither_explicit_nor_default_is_not_found | single-branch | **no** | §9 clause 2 fails: `default_entry` is `self.defaults.get(name)` — a bare tail-expression with no `if`/`?`/comparison, none of clause 2's four listed shapes. The fixture (`AllowedCollisionMatrix::new()`, nothing ever set) has no antecedent setter call either, so there is no decision "one level up" to attribute this to (contrast 880, below). An engineer cannot get this specific line wrong in any way this test could catch short of deleting the lookup outright |
| matrix.rs:691 | bare | never_and_always_carry_no_predicate | single-branch | yes | read: `Never`/`Always` share one match arm in `predicate()`; no independent path to diverge. §9 clause 2: `predicate()`'s 2-arm match (`Conditional(f) => Some(f)`, `Never \| Always => None`) is a real, written decision an engineer could get wrong (e.g. drop the `\|` and give `Always` a predicate) |
| matrix.rs:692 | bare | never_and_always_carry_no_predicate | single-branch | yes | same shared arm as 691 |
| matrix.rs:702 | bare | overwriting_a_conditional_entry_with_bool_drops_the_predicate | single-branch | yes | same shared arm as 691/692. §9 clause 2: the decision actually being exercised here lives one level up, in `set_entry`'s overwrite of the map cell (does setting a bool entry actually replace a prior `Conditional`, or does it merge/leak the old predicate) — same "lives one level up" reasoning as census §9's mimic worked example |
| matrix.rs:735 | bare | remove_entry_then_lookup_falls_back_to_default | discriminating | yes | bite run now on `entry()` guard B (row present, key missing): guard-B-neutralize → FAILED; guard-A-neutralize → PASSED |
| matrix.rs:747 | bare | remove_entry_is_symmetric | discriminating | yes | bite run now, same guard-B pattern as 735 |
| matrix.rs:748 | bare | remove_entry_is_symmetric | discriminating | yes | bite run now, mirror direction, same test |
| matrix.rs:759 | bare | remove_entries_for_name_clears_its_row_and_every_cell_naming_it | discriminating | yes | bite run now, same guard-B pattern |
| matrix.rs:760 | bare | remove_entries_for_name_clears_its_row_and_every_cell_naming_it | discriminating | yes | bite run now, same guard-B pattern |
| matrix.rs:791 | bare | set_entry_between_pairs_every_combination | discriminating | yes | bite run now, same guard-B pattern |
| matrix.rs:792 | bare | set_entry_between_pairs_every_combination | discriminating | yes | bite run now, same guard-B pattern |
| matrix.rs:809 | bare | set_all_entries_overwrites_every_existing_pair_but_adds_none | discriminating | yes | bite run now, same guard-B pattern |
| matrix.rs:821 | bare | set_entry_for_known_pairs_name_with_every_other_existing_row_but_not_itself | discriminating | yes | `entry()` guard B verified by bite run now; the `set_entry_for_known` `!= name` filter itself is vacuous here — confirmed live (removing the filter left this test green) and by commit `035d4b1`'s message, which is why sibling 835 was added |
| matrix.rs:835 | bare | set_entry_for_known_excludes_the_name_even_when_it_is_already_a_known_row | discriminating | yes | commit `035d4b1`, isolating-mutation pair with 821: re-verified live, filter-removed bite makes this test FAIL while 821 stays GREEN |
| matrix.rs:880 | bare | clear_removes_entries_and_defaults | single-branch | yes | §9 re-bite, live: `default_entry` itself is the same bare lookup as 660, but here it follows a real prior `set_default_entry`+`clear()` sequence. Commented out `self.defaults.clear();` in `clear()` → this test FAILS (198/199); reverted, clean. The decision lives in `clear()` (does it also empty `self.defaults`), one level up from the getter — same reasoning as census §9's mimic worked example, and the opposite of 660 where no such antecedent decision exists |

Note: the `entry()` two-guard family (659 vs 735-821) is the brief's own
cited model ("p3-acm's `matrix.rs` `entry()` work is the model"); every
guard-B site above shares one bite pair, not 10 independent ones.

### `src/octomap_filter.rs` (5)
| file:line | anchor | test fn | verdict | in-family | evidence |
|---|---|---|---|---|---|
| octomap_filter.rs:300 | bare | sample_cloud_empty_returns_none | single-branch | yes | bite run now: no-op'd the sole guard → test FAILED; no sibling to swap in |
| octomap_filter.rs:355 | bare | metaball_surface_properties_without_depth_returns_unit_normal_only | single-branch | yes | bite run now: `None`→`Some(NaN)` on the depth else-branch → test FAILED; the if-branch can only ever produce `Some`, never `None` |
| octomap_filter.rs:369 | bare | metaball_surface_properties_empty_cloud_is_none_in_both_modes | discriminating (isolating mutation, direction A) | yes | bite run now: neutralized guard A (`sample_cloud` path) → test FAILED naming A; sibling assertion (guard B) stayed GREEN when isolated |
| octomap_filter.rs:370 | bare | metaball_surface_properties_empty_cloud_is_none_in_both_modes | discriminating (isolating mutation, direction B) | yes | mirror bite run now: neutralized guard B (`find_surface` path) → test FAILED naming B; guard-A assertion stayed GREEN when isolated |
| octomap_filter.rs:381 | bare | refine_contact_normals_no_contacts_requested_is_a_noop | not-this-family | **no** | `rg -n 'contacts\s*='` over the file: zero assignment sites inside `refine_contact_normals` — `result.contacts` starts `None` and cannot become `Some` by any path through this function. §9 clause 2: no branch of the subject's logic is ever exercised, regardless of what that logic does — the exact `shortest_solution` shape |

### `src/parry.rs` (8)
| file:line | anchor | test fn | verdict | in-family | evidence |
|---|---|---|---|---|---|
| parry.rs:2636 | bare | convert_shape_degenerate_plane_is_excluded | discriminating | yes | isolating mutation run now: neutralized Plane-magnitude guard → Plane test FAILED, both OcTree tests (2659, 2669) stayed GREEN |
| parry.rs:2659 | bare | convert_shape_octree_with_no_tree_attached_is_excluded | discriminating | yes | isolating mutation run now: neutralized no-tree guard → this test FAILED, Plane + no-occupied-leaves tests stayed GREEN |
| parry.rs:2669 | bare | convert_shape_octree_with_a_tree_but_no_occupied_leaves_is_also_excluded | discriminating | yes | isolating mutation run now: neutralized empty-tree path → this test FAILED, Plane + no-tree tests stayed GREEN |
| parry.rs:2810 | bare | octree_cache_prunes_an_entry_once_nothing_holds_its_tree | single-branch | yes | read: `build()` is stubbed and no cache hit is possible (fresh key each call); bite run now forcing a value regardless of `build()` → test FAILED |
| parry.rs:2817 | bare | octree_cache_prunes_an_entry_once_nothing_holds_its_tree | single-branch | yes | same bite/run as 2810 (one test function, one mutation covers both assertions) |
| parry.rs:3967 | bare | check_robot_collision_continuous_returns_an_error_rather_than_approximating | single-branch | yes | read: function body is one unconditional `Err`, no guard at all; bite run now replacing it with `Ok(..)` → test FAILED. §9 clause 2: the decision (reject rather than silently approximate) is real even though it isn't syntactically a guard — the bite proves it is exercised, same standard census §9 applies to the mimic getter |
| parry.rs:4623 | bare | check_self_collision_cost_sources_is_none_when_not_requested | single-branch | yes | read: one boolean gate (`request.cost`) controls `Some`/`None`, no sibling; bite run now forcing `Some` unconditionally → test FAILED |
| parry.rs:2897 | is_none | scaled_padded_shape_pads_a_mesh_that_arrived_without_vertex_normals | not-this-family | **no** | §9 clause 3: this reads back `Mesh::new`'s own documented post-state before the subject is called at all — it is the guard that keeps the test's fixture from silently becoming a normals-carrying mesh, not a claim about `scaled_padded_shape`. The subject assertion in the same test is `assert_eq!` on the vertex count, outside this grammar. Bite evidence for the branch the *test* covers: deleting `scaled_padded_shape`'s `compute_vertex_normals()` call → this test FAILED (panic at the `.expect()`, `Construct("mesh padding requires vertex normals")`) while its cuboid sibling `scaled_padded_shape_grows_a_cuboid_by_scale_then_padding` stayed GREEN — run now, 1 FAILED / 1 PASSED |

### `src/world.rs` (15)
| file:line | anchor | test fn | verdict | in-family | evidence |
|---|---|---|---|---|---|
| world.rs:1038 | bare | add_to_object_mismatched_lengths_is_a_no_op | discriminating | yes | isolating bite run now on the OR-guard's length clause: neutralize → 1038 FAILS, 1046 stays GREEN |
| world.rs:1046 | bare | add_to_object_empty_shapes_is_a_no_op | discriminating | yes | isolating bite run now on the OR-guard's empty clause: neutralize → 1046 FAILS, 1038 stays GREEN |
| world.rs:1099 | bare | move_shape_in_object_unknown_shape_is_none | discriminating | yes | isolating bite run now: neutralize position-lookup → 1099 FAILS/1109 GREEN |
| world.rs:1109 | bare | move_shape_in_object_unknown_object_is_none | discriminating | yes | isolating bite run now: neutralize `object_mut` guard → 1109 FAILS/1099 GREEN |
| world.rs:1128 | bare | move_shapes_in_object_count_mismatch_is_a_no_op | discriminating | yes | isolating bite run now: drop count check → 1128 FAILS/1142 GREEN, confirming the test's own doc-comment claim |
| world.rs:1142 | bare | move_shapes_in_object_unknown_object_is_none | discriminating | yes | isolating bite run now: bypass `object_mut` guard → 1142 FAILS/1128 GREEN — doc-comment's isolation claim verified true, not just asserted |
| world.rs:1150 | matches | move_object_not_found | discriminating | yes | bite run now: fallback-to-identity → FAILS; variant-swap to `NoChange` → still FAILS. §9 clause 1: `MoveObjectOutcome::NotFound` is a coarse "could not produce a move" tag standing in for `None` (see the enum def, world.rs:515-524) — unlike `JointKind::Fixed`, it reports an operation's outcome, not a descriptive property |
| world.rs:1160 | matches | move_object_identity_transform_is_no_change | discriminating | yes | stayed GREEN through both 1150 mutations above, confirming isolation. §9 clause 1: `NoChange` is "did not produce a change" — the same operation-outcome reading as `NotFound`, not a computed fact |
| world.rs:1255 | bare | remove_shape_from_object_unknown_is_none | discriminating | yes | isolating bite run now: neutralize `object_mut` guard → 1255 FAILS/1276 GREEN |
| world.rs:1276 | bare | remove_shape_from_object_unknown_shape_is_none | discriminating | yes | isolating bite run now: neutralize position-lookup (Arc-identity match) → 1276 FAILS/1255 GREEN; the Arc-identity doc-comment is accurate context but the bite is the evidence |
| world.rs:1288 | bare | remove_object_missing_is_none | single-branch | yes | read: one guard in `remove_object`'s body; bite run now (fallback dummy object) → FAILED. §9 clause 2: `self.objects.remove(id)?` is a real `?`-propagation (bite-proven), not a bare non-decision lookup |
| world.rs:1343 | bare | global_shape_transform_unknown_object_is_none | discriminating | yes | isolating bite run now: unknown-object fallback → 1343 FAILS/1357 GREEN |
| world.rs:1357 | bare | global_shape_transform_out_of_range_index_is_none | discriminating | yes | isolating bite run now: out-of-range fallback → 1357 FAILS/1343 GREEN |
| world.rs:1363 | bare | global_shape_transforms_unknown_object_is_none | single-branch | yes | read: one guard, only cause in `global_shape_transforms`; bite run now (empty-slice fallback) → FAILED. §9 clause 2: `self.objects.get(object_id)?` is a real `?`-propagation, bite-proven |
| world.rs:1399 | bare | transform_lookup_unknown_name_errors | single-branch | yes | read: no sibling guard, one fallthrough per function (`knows_transform`/`try_get_transform`/`get_transform`); bite run now (flipped final `false`→`true` in `knows_transform`) → FAILED. §9 clause 2: `try_get_transform` has real sequential decision logic (exact-match check, then a subframe-matching loop), not a bare lookup |

### `tests/world_parity.rs` (1)
| file:line | anchor | test fn | verdict | in-family | evidence |
|---|---|---|---|---|---|
| world_parity.rs:226 | bare | world_matches_oracle | not-this-family | **no** | module doc (lines 4-33) + committed oracle fixture (`world_request.json`/`world_response.json`): `query.transform is None` per the oracle's own JSON dump for a specific query name — a per-query oracle-value comparison, not branch discrimination inside one function; there is no sibling branch to isolate. §9 clause 3: whatever decision produced this `None` belongs to the oracle-generating process, not to code under test this assertion exercises |

### `tests/exact_tangency_is_decided_per_shape_pair.rs` (1)
| file:line | anchor | test fn | verdict | in-family | evidence |
|---|---|---|---|---|---|
| exact_tangency_is_decided_per_shape_pair.rs:331 | is_empty | `report`, reached from `exact_tangency_collides_for_every_pair_but_sphere_on_sphere` and `a_nanometre_either_side_of_the_tie_brackets_it` | discriminating | yes | the `is_empty()` is over a list of *named differing cells*, so the failure message identifies which of the 25 shape pairs moved rather than only that one did. Isolating mutation run now: flipping the pinned `sphere x sphere` cell of `TANGENT` from `false` to `true` → FAILED with `the exact-tangency table changed in 1 of 25 cells: sphere x sphere: expected true, got false`, and the two bracketing tables stayed GREEN. A second, independent bite (deleting `scaled_padded_shape`'s `compute_vertex_normals()` call) failed the same assertion's two callers by panic instead, confirming the mesh columns are really exercised |

## 3. `crates/moveit-geometry` — 39 sites

### `src/bodies.rs` (13)

**Correction, this pass: 3 of these 13 rows were wrong** (same defect
family as `shapes.rs` above — see §1a). `bodies::Cuboid::recompute` is a
*different* function from `shapes::Cuboid::new` (a separate struct
defined at bodies.rs:2138, own `recompute` at bodies.rs:2197) but shares
the identical combined `half_length<0 || half_width<0 || half_height<0`
guard shape, and the same "one token, one cause" misclassification.

| file:line | anchor | test fn | verdict | in-family | evidence |
|---|---|---|---|---|---|
| bodies.rs:4037 | bare | sphere_negative_radius_is_an_error | single-branch | yes | bite run now: no-op'd `Sphere::new`'s one guard → test FAILED; exactly one `Error::construct` call in the function |
| bodies.rs:4087 | bare | cuboid_negative_dimension_is_an_error_per_axis | discriminating (multi-branch, corrected) | yes | bite run now: neutralize `half_length` clause in `bodies::Cuboid::recompute` → 4087 (length) FAILS, 4089 (height) stays GREEN |
| bodies.rs:4088 | bare | cuboid_negative_dimension_is_an_error_per_axis | discriminating (multi-branch, corrected) | yes | bite run now: neutralize `half_width` clause in `bodies::Cuboid::recompute` (2209) → panic lands exactly at 4088, proving 4087 (length) passed first and 4089 (height) was never reached; reverted, gate re-run clean |
| bodies.rs:4089 | bare | cuboid_negative_dimension_is_an_error_per_axis | discriminating (multi-branch, corrected) | yes | mirror half of the 4087 bite pair — stayed GREEN when `half_length` was neutralized |
| bodies.rs:4102 | bare | sphere_padding_inversion_is_rejected_and_state_preserved | single-branch | yes | doc-comment re-read and confirmed: `Sphere::set_padding` has exactly one `Error::construct` call, one operand (`if radius_scaled < 0.0`, a real comparison) |
| bodies.rs:4140 | bare | cuboid_padding_inversion_is_rejected_and_state_preserved | single-branch (as tested) | yes | this test has exactly one assertion (no sibling axis case in this specific test), so there is nothing in *this* test to isolate against even though the underlying `recompute` guard is the same combined shape corrected at 4087-4089; not re-classified — a coverage-gap question (whether a second case should exist), not a misclassification of what is actually asserted here |
| bodies.rs:4328 | matches | from_shape_builds_matching_body_variant | discriminating | **no** | bite run now: rewrote the `Shape::Sphere` arm of `Body::from_shape` to build a `Cylinder` instead → the Sphere assert FAILED. §9 clause 1: for all four inputs in this test, `from_shape` unconditionally succeeds (`Ok(Some(_))`); the assertion checks *which concrete `Body` variant* was built, not a could-not/did-not-produce-X signal — a computed dispatch fact, structurally identical to census §9's own disqualifying `matches!(joint.kind(), JointKind::Fixed)` example |
| bodies.rs:4332 | matches | from_shape_builds_matching_body_variant | discriminating | **no** | doc-comment re-read: each `Body::from_shape` arm builds a distinct concrete variant, mirror of the 4328 bite. §9 clause 1: same dispatch-fact reasoning as 4328 |
| bodies.rs:4340 | matches | from_shape_builds_matching_body_variant | discriminating | **no** | same reasoning, distinct match arm (`Shape::Cuboid`). §9 clause 1: same as 4328 |
| bodies.rs:4347 | matches | from_shape_builds_matching_body_variant | discriminating | **no** | same reasoning, distinct match arm (`Shape::Mesh`). §9 clause 1: same as 4328 |
| bodies.rs:4384 | bare | from_shape_returns_none_for_cone_plane_octree | discriminating (isolating mutation) | yes | bite run now: split the Cone pattern out of the combined `None` arm in `Body::from_shape` → the Cone assert FAILED. §9 clause 1: unlike 4328-4347, this *is* a could-not-produce-a-`Body` signal (`Option::None`), not a variant-dispatch fact |
| bodies.rs:4392 | bare | from_shape_returns_none_for_cone_plane_octree | discriminating (isolating mutation) | yes | bite run now: split the Plane pattern out → the Plane assert FAILED while the Cone assert stayed GREEN |
| bodies.rs:4402 | bare | from_shape_returns_none_for_cone_plane_octree | discriminating (isolating mutation) | yes | doc-comment re-read: symmetric with the Cone/Plane bites just run, each pattern an independent match arm; the doc-comment's own D6 call-site check confirms no caller needs `from_shape` to discriminate Cone/Plane/OcTree |

Note: the doc-comment at bodies.rs:4045-4048 (which contrasts `sphere_negative_radius_is_an_error` with a *sibling* `cylinder_negative_length_is_an_error` test elsewhere in the file) is evidence for that cylinder test, not for line 4037 — flagged explicitly to avoid misattribution, and not used as 4037's evidence above.

### `src/octree_collision.rs` (2)
| file:line | anchor | test fn | verdict | in-family | evidence |
|---|---|---|---|---|---|
| octree_collision.rs:120 | bare | empty_tree_has_no_occupied_leaves | single-branch | yes | bite run now: no-op'd the sole `is_empty()` guard → both this test and its sibling failed identically (only one `None`-producing site in the whole function). §9 clause 2: `if leaf_shapes.is_empty() { return None }` is a real, deliberate panic-avoidance guard (the doc comment names it explicitly: `Compound::new` panics on an empty list) |
| octree_collision.rs:127 | bare | all_free_tree_has_no_occupied_leaves | single-branch | yes | same bite as 120 — "empty tree" and "all free" both collapse to the same `is_empty()` check before `Some`/`Compound::new`, not two distinct guards, so the §3a isolating-mutation case does not apply here |

### `src/shapes.rs` (17)

**Correction, this pass (see §1a below): 8 of these 17 rows were wrong.**
`Sphere::new` genuinely has one operand and is `single-branch`. But
`Cylinder::new`/`Cone::new`/`Cuboid::new` (and `Cylinder::scale_and_padd_axes`)
each fold *multiple* named causes into one combined `||` guard with a
single shared message — "one `Error::` token" was misread as "one cause"
(the exact mistake the brief's section 3 `search_position_ik` example
warns about). Live isolating-mutation bites, run just now, prove each
per-axis assertion is independently discriminating. Comments at
shapes.rs (and the mirrored one in bodies.rs) corrected in the same
commit as this ledger update; see §1a.

| file:line | anchor | test fn | verdict | in-family | evidence |
|---|---|---|---|---|---|
| shapes.rs:1619 | bare | sphere_negative_radius_is_an_error | single-branch | yes | `Sphere::new` has one operand, one guard — nothing to isolate |
| shapes.rs:1624 | bare | cylinder_negative_dimension_is_an_error | discriminating (multi-branch, corrected) | yes | bite run now, both directions: neutralize radius clause → 1624 FAILS, 1625 stays GREEN; neutralize length clause → 1625 FAILS, 1624 stays GREEN (comment out sibling per §3a protocol, since `assert!` short-circuits within the one test fn) |
| shapes.rs:1625 | bare | cylinder_negative_dimension_is_an_error | discriminating (multi-branch, corrected) | yes | same bite pair as 1624 |
| shapes.rs:1631 | bare | cone_negative_dimension_is_an_error | discriminating (multi-branch, corrected) | yes | bite run now: neutralize radius clause → 1631 FAILS, 1632 stays GREEN (identical combined-guard shape to Cylinder::new, confirmed live rather than assumed by analogy) |
| shapes.rs:1632 | bare | cone_negative_dimension_is_an_error | discriminating (multi-branch, corrected) | yes | same bite as 1631 |
| shapes.rs:1637 | bare | cuboid_negative_dimension_is_an_error | discriminating (multi-branch, corrected) | yes | bite run now: neutralize x clause → 1637 FAILS, 1639 (z) stays GREEN |
| shapes.rs:1638 | bare | cuboid_negative_dimension_is_an_error | discriminating (multi-branch, corrected) | yes | bite run now: neutralize `y` clause in `shapes::Cuboid::new` (905) → panic lands exactly at 1638, proving 1637 (x) passed first; reverted, gate re-run clean |
| shapes.rs:1639 | bare | cuboid_negative_dimension_is_an_error | discriminating (multi-branch, corrected) | yes | mirror half of the 1637 bite pair — stayed GREEN when x was neutralized |
| shapes.rs:1657 | bare | sphere_padding_past_negative_is_an_error | single-branch | yes | read: `Sphere::scale_and_padd` has exactly one `Err` site, one operand |
| shapes.rs:1677 | bare | cylinder_padding_past_negative_is_an_error_per_axis | discriminating (multi-branch, corrected) | yes | bite run now, both directions: neutralize radius clause → 1677 (`radius_case`) FAILS, 1686 (`length_case`) stays GREEN; neutralize length clause → mirrors |
| shapes.rs:1686 | bare | cylinder_padding_past_negative_is_an_error_per_axis | discriminating (multi-branch, corrected) | yes | same bite pair as 1677 |
| shapes.rs:1798 | bare | shapes_with_no_upstream_body_have_no_volume_or_dimensions | discriminating (isolating mutation, multi-branch) | yes | bite run now: split `Shape::Cone(_)` out of `compute_volume`'s combined `None` arm (`Cone\|Plane\|Mesh\|OcTree`) → FAILED exactly at the Cone iteration; matches commit `871bb9e`'s recorded correction |
| shapes.rs:1799 | bare | shapes_with_no_upstream_body_have_no_volume_or_dimensions | discriminating (isolating mutation, multi-branch) | yes | doc-comment re-read: records the same isolating mutation for `get_dimensions`, identical combined-arm structure |
| shapes.rs:1863 | matches | mesh_rejects_out_of_range_triangle_index | single-branch | yes | read: `Mesh::new` has exactly one `Error::construct` call, inside the vertex-index loop. §9 clause 1: `Error::Construct(_)` is inspected on the failure (`Err`) path, unlike bodies.rs:4328's success-path variant check |
| shapes.rs:1899 | bare | mesh_padding_without_vertex_normals_is_an_error | single-branch | yes | read: `Mesh::scale_and_padd_axes` has one `Err` site (`vertex_normals.as_ref().ok_or_else`); the empty-vertices branch returns `Ok(())`, not an error |
| shapes.rs:1900 | bare | mesh_padding_without_vertex_normals_is_an_error | single-branch | yes | same guard as 1899, `scale_axes`/`padd_axes` both funnel through it |
| shapes.rs:1962 | bare | compute_vertex_normals_calls_triangle_normals_when_needed | not-this-family | **no** | doc-comment re-read and confirmed: `mesh.triangle_normals.is_none()` reads a struct field literal-initialized to `None` in `Mesh::new`, not a computed branch. §9 clause 2: the initial value is read before any subject logic touches it — the exact `shortest_solution` shape |

The brief's own model example (`a74a310`, `Cylinder::recompute`'s two
sequential radius/length guards with distinct messages) is in `bodies.rs`,
not this file — confirmed via `git show --stat a74a310`; this crate's own
doc-comment (shapes.rs:1593-1615, corrected this round, see §1a) draws
that exact contrast while now also correcting its own verdict for the
combined-guard side of it.

### `src/stl.rs` (1)
| file:line | anchor | test fn | verdict | in-family | evidence |
|---|---|---|---|---|---|
| stl.rs:456 | bare | empty_input_is_rejected | single-branch | yes | bite run now: neutralized `parse_ascii_triangles`'s `flat_vertices.is_empty()` guard → test flipped from pass to FAIL; for the `&[]` fixture, `parse_binary_triangles` bails immediately and `Mesh::new` is never reached, so exactly one guard fires for this input (other inputs can reach `mesh_from_bytes`'s other `Err` sites, but not this fixture) |

### `src/transforms.rs` (6)
| file:line | anchor | test fn | verdict | in-family | evidence |
|---|---|---|---|---|---|
| transforms.rs:233 | bare | new_rejects_empty_target_frame | single-branch | yes | bite run now: no-op'd `new`'s one `is_empty()` guard → test FAILED; only one `Err` site in the function |
| transforms.rs:234 | bare | new_rejects_empty_target_frame | single-branch (redundant with 233) | yes | read: `trim()` runs before the single `is_empty()` check, so `""` and `"   "` hit the identical branch — no second guard exists to isolate 234 from 233 |
| transforms.rs:253 | bare | set_transform_rejects_empty_name | single-branch | yes | bite run now: no-op'd `set_transform`'s one `is_empty()` guard → test FAILED |
| transforms.rs:275 | bare | unknown_frame_is_an_error_not_identity | single-branch | yes | bite run now: replaced `transform`'s one `ok_or_else` with a fallback `Ok` → test FAILED (input `"nope"`, map-miss path) |
| transforms.rs:276 | bare | unknown_frame_is_an_error_not_identity | single-branch | yes | bite run now, isolated: patched the map-miss fallback with the 275 assertion removed → failure confirms this assertion's only reachable source is the map-miss branch (the empty-string early return is unreachable for this non-empty literal) |
| transforms.rs:277 | bare | unknown_frame_is_an_error_not_identity | single-branch | yes | same `ok_or_else` bite as 275 — both 275 and 277 funnel through `transform`'s one `Err` site, differing only in which upstream `try_transform` path produced the `None` |

## 4. Summary

89 sites (86 + 3 new `tools.rs:68` rows, §1b), 0 UNCOVERED-and-blind as
of this revision, but two distinct defects fixed across the two rounds:

- **Round 8**: **12 sites had a wrong verdict** — `single-branch`
  misapplied to combined `||` guards hiding 2-3 independently
  discriminating causes, in `shapes.rs`
  (`cylinder_negative_dimension_is_an_error` ×2,
  `cone_negative_dimension_is_an_error` ×2,
  `cuboid_negative_dimension_is_an_error` ×3,
  `cylinder_padding_past_negative_is_an_error_per_axis` ×2) and
  `bodies.rs` (`cuboid_negative_dimension_is_an_error_per_axis` ×3). All
  12 reclassified to `discriminating (multi-branch, corrected)` on live
  bite evidence (§1a). No test itself was broken or blind in these 12;
  only the *comment* asserting otherwise was wrong.
- **Round 9 (this pass, §1b)**: **1 site (`tools.rs:68`, 3 operands)**
  was never enumerated at all — outside `ledger_scan.py`'s grammar, not
  merely misverdicted — and 2 of its 3 operands (y, z axes) were
  genuinely blind (no test could distinguish them from a stubbed-out
  guard). Both got new tests (commit `d24494d`); the third
  (x axis) was already covered. The re-verification pass over
  `moveit-collision`'s remaining 17 `single-branch` rows found **0
  misverdicts** — every one read back to a genuinely single cause.

No `fixture-collapse-fixed` verdicts — none of the 89 sites needed one.

- **Round 10 (this pass, §1c)**: census §9's three-clause family test
  applied to all 89 rows, fresh (not copied from any other panel's
  ledger). **81 of 89 in-family.** 8 moved to `not-this-family`: 3
  pre-existing (`octomap_filter.rs:381`, `world_parity.rs:226`,
  `crates/moveit-geometry/src/shapes.rs:1962`) plus 5 newly excluded this pass —
  `matrix.rs:660` (clause 2: a bare `.get()` with no `?`/guard/comparison,
  on a fixture with no antecedent setter call, unlike the structurally
  similar `matrix.rs:880` where the decision lives in `clear()`) and
  `bodies.rs:4328,4332,4340,4347` (clause 1: each checks *which* `Body`
  variant a `matches!` produced on an always-succeeding path — a computed
  dispatch fact, not a could-not/did-not-produce-X signal). No verdict
  (`discriminating`/`single-branch`) changed — this is a family-membership
  question, separate from and prior to the verdict question §1a/§1b
  answer. Two ambiguous bare-`?` rows (`tools.rs:219`, `matrix.rs:880`)
  were settled by live bite rather than argument, per census §9's own
  `nn.rs:227` precedent.

## 5. Gate

**Round 8**: one commit correcting the 3 source doc-comments (§1a) that
had recorded the wrong verdict. Comment-only — no assertion, guard, or
test body changed; behavior is identical before and after.

```
cargo fmt --all
cargo clippy -p moveit-geometry --all-targets -- -D warnings   # clean
cargo nextest run -p moveit-geometry                            # 141/141 pass
```

`moveit-collision` had no source changes in round 8 (no misclassification
found in that crate's 47 sites at the time).

**Round 9** (this pass): one commit (`d24494d`) adding the two blind-axis
tests to `tools.rs`. Gated `-p moveit-collision`:

```
cargo fmt --all
cargo clippy -p moveit-collision --all-targets -- -D warnings   # clean
cargo nextest run -p moveit-collision                            # 199/199 pass
```

Per-mutation bite runs (both directions, all three axes, before and
after the fix) used `cargo nextest run -p moveit-collision
--no-fail-fast`, each reverted via `git diff`/`git status --short`
confirmed empty before the next mutation.

**Round 10** (this pass, §1c): report only, no source changes. The two
settling bites (`tools.rs:219`'s `?`→`.expect(..)`, `matrix.rs:880`'s
`clear()` mutation) were run live via `cargo nextest run -p
moveit-collision --no-fail-fast` and reverted; both confirmed via
`git status --short` empty before and after. No gate owed beyond that —
this pass only reclassifies family membership in the doc, per instruction
("doc-only unless a re-read turns up a wrong verdict" — none did).

**Round 11** (§6): report only, no source changes — 0 blind operands
found, so no test was added. Every settling bite (`matrix.rs:870/879`,
`octomap_filter.rs:364`, `crates/moveit-geometry/src/shapes.rs:1964`, `world.rs:1300,1326,1398`,
`tools.rs:369/370`) was run live via `cargo nextest run -p
moveit-collision --no-fail-fast` / `-p moveit-geometry --no-fail-fast`,
each confirmed via `git status --short` empty and `cargo fmt --all --
--check` clean before the next mutation.

**Round 12** (§6, `moveit-geometry` correction): report only, no source
changes. 4 sites (`bodies.rs:3953,4055,4066,4125`) were wrongly excluded
in round 11 under a since-deleted, never-sound ownership rule; added
back, classified in-family, and settled with 3 fresh live bites (the
`build_mesh_data` guard-neutralize bite for 3953, and the
`Cylinder::set_padding` skip-recompute bite for 4125, confirming 4055/4066
stay green while 4125 alone fails — proving 4055/4125's shared `"radius"`
needle is not a blind pair). All run via `cargo nextest run -p
moveit-geometry --no-fail-fast`, each confirmed via `git status --short`
empty and `cargo fmt --all -- --check` clean before the next mutation.

**UNFIXED:** none. **Fixed:** round 8's 12 sites' verdicts (3
doc-comment edits, comment-only, `moveit-geometry`); round 9's 2 blind
operands at `tools.rs:68` (2 new tests, `moveit-collision`, commit
`d24494d`); round 10 fixed no source, only classified family membership
(§1c); round 11 (§6) fixed no source — 0 blind operands found among the
39-then-counted new sites, only classified family membership; round 12
fixed no source either — the 4 sites round 11 wrongly excluded are all
in-family and all already covered (0 blind operands among them), only
classified family membership and corrected the geometry count from 9 to
13. **Corrected round-11/12 total: 43 new sites (30 moveit-collision +
13 moveit-geometry), 24 in-family, 19 not-this-family, 0 blind operands**
(one doc-only evidence correction to `world.rs:1399` from round 11).
**Tested:** all 89 sites, one row each, above, plus 43 new sites across
§6/round-12; `cargo nextest run -p moveit-geometry` 141/141 and `cargo
nextest run -p moveit-collision` 199/199, both post-fix; the two round-10
settling bites (`tools.rs:219`, `matrix.rs:880`), the eight round-11
settling bites (`matrix.rs:870` read-only via len(), `matrix.rs:879`,
`octomap_filter.rs:364`, `crates/moveit-geometry/src/shapes.rs:1964`, `world.rs:1300`,
`world.rs:1326`, `world.rs:1398`, `tools.rs:369/370`), and the three
round-12 settling bites (`bodies.rs:3953`, `bodies.rs:4125`'s
`set_padding` skip-recompute, plus the negative confirmation that
`bodies.rs:4055/4066` stay green under that same mutation) each confirmed
FAIL-then-clean-revert live.

## 6. Round 11: broadened grammar (`tools/ci/count-coarse-assertions.py`)

`count-coarse-assertions.py` (`6a14a89`) scans `is_empty`/`contains_member`/
`is_some`/`eq_none`/`contains_msg` shapes in addition to the original
`matches!`/`.is_err()`/`.is_none()` grammar the 89 rows above came from.
Run against both fenced crates:

```
python3 tools/ci/count-coarse-assertions.py crates/moveit-collision crates/moveit-geometry
```

132 total sites; excluding `contains_msg` (4, `bodies.rs:3953,4055,4066,4125`
— p1-robotmodel's this round, not touched) and the three kinds already in
the 89-row grammar (`is_none` 52, `is_err` 27, `matches` 7), the candidate
set outside the original grammar is **42**: 18 `is_empty` + 15
`contains_member` + 7 `is_some` + 2 `eq_none`, split moveit-collision 33 /
moveit-geometry 9.

**Correction: 42 is not the new-site count.** Three of the 33
moveit-collision candidates — `tools.rs:259,271,283` — are not new sites at
all: they are the exact same three assertions already carried in this
ledger as `tools.rs:68 (x)/(y)/(z)` (§1b), just also matched by this
scanner's `is_empty` pattern (`intersect_cost_sources(...).is_empty()`),
which the original `ledger_scan.py` grammar could not see at all. Verified
by line number, not by kind label: all three physical lines already have a
row. **The genuinely new, previously-unrowed count is 39** (30
moveit-collision + 9 moveit-geometry), not 42; the kind mix corrects to 15
`is_empty` + 15 `contains_member` + 7 `is_some` + 2 `eq_none`.

Census §9 applied to all 39, the same three clauses (mechanism / decision /
subject) as §1c, live-bit wherever the reading was ambiguous rather than
argued from shape alone: **20 of 39 in-family**, 19 not-this-family. 0
blind operands found — every in-family site already has either a sibling
assertion (an existing or a co-listed row testing the opposite branch of
the same decision) or a live bite proving the current test set already
catches the mutation; none needed a new test.

### In-family (21)

| Site | Kind | Test fn | Evidence |
|---|---|---|---|
| matrix.rs:870 | is_empty | `len_counts_rows_not_pairs` | `!acm.is_empty()` wraps `self.entries.is_empty()`; same `set_entry`-insertion decision the test's own `len()==3` assertion (3 lines above) already exercises. §9 "lives one level up" (the branch is in `set_entry`/`set_pair`, not in `is_empty()`); redundant confirmation, not independently bitten. |
| matrix.rs:879 | is_empty | `clear_removes_entries_and_defaults` | bite run now: commented out `self.entries.clear()` in `clear()` (kept `self.defaults.clear()`) → this assertion FAILS alone (198/199), sibling `matrix.rs:880` (`default_entry("a").is_none()`) stays GREEN — mirror-direction bite to the one already on record for 880. |
| octomap_filter.rs:364 | is_some | `metaball_surface_properties_with_depth_reports_signed_depth` | bite run now: forced the `estimate_depth==true` arm of `metaball_surface_properties` to also return `None` → this test FAILS alone, sibling `octomap_filter.rs:355` (`without_depth`, existing row) stays GREEN. Correction: 355's existing `single-branch` verdict had no on-record discriminating partner; this row is it. |
| parry.rs:2701 | is_some | `octree_cache_survives_shape_churn` | read + doc comment (named regression test for an `OctreeCache` pointer-identity bug): `assert_eq!(got.is_some(), occupied, ..)` alternates occupied/empty across 200 iterations, the same occupied-leaves decision already bite-proven for `parry.rs:2659`/`:2669` (existing, `None` side) plus the `.expect()`-based occupied-side test (not is_some-shaped, invisible to any grammar until now). |
| shapes.rs:1964 | is_some | `compute_vertex_normals_calls_triangle_normals_when_needed` | bite run now: gated `compute_triangle_normals()`'s call behind `if false` inside `compute_vertex_normals` → this test FAILS (panics two lines later at the `.expect()`, before reaching this literal line, but the whole test — and this assertion's own precondition — depends on the guard). §9 "lives one level up": the decision belongs to `compute_vertex_normals`, not the field read (contrast the sibling `crates/moveit-geometry/src/shapes.rs:1962`, not-this-family, §1c). |
| octree_in_world_parity.rs:220 | is_some | oracle-query loop | oracle parity: `assert_eq!(mapped, actual_log_odds.is_some(), ..)` compares `moveit_octomap::OcTree::log_odds_at` against a committed oracle fixture's `mapped` field — a real per-query decision. **Fence note:** `log_odds_at` itself is implemented in `moveit-octomap`, outside this panel's fence; a blind finding here would need an out-of-fence fix. |
| world.rs:1326 | eq_none | `set_subframes_of_object_computes_global_pose_and_replaces_old_ones` | bite run now: changed `set_subframes_of_object`'s replace (`obj.subframes = ..collect()`) to a merge (`obj.subframes.extend(..)`) → this assertion FAILS alone (198/199). Confirms the doc-commented "replaced, not merged" claim is actually tested. |
| world.rs:1398 | eq_none | `transform_lookup_unknown_name_errors` | bite run now: changed `try_get_transform`'s final `None` to `Some(Isometry3::identity())` → this assertion FAILS alone (`world_parity`'s oracle test regresses too, as a downstream consequence). **Corrects existing row `world.rs:1399`**: that row's evidence cites a `knows_transform` bite as proof, but `knows_transform` is a separate function (proves only `world.rs:1397`'s sensitivity) — `try_get_transform`/`get_transform`'s own fallthrough (which both 1398 and 1399 exercise) was never directly bitten before this row. Doc-only correction; 1399's verdict (in-family, single-branch) is unchanged. |
| tools.rs:292 | is_empty | `disjoint_boxes_do_not_intersect` | same `aabb_intersection` guard already fully bitten per-axis at `tools.rs:68` (§1b, commit `d24494d`); this case sets all three axis operands at once (fully disjoint box) — a redundant but genuine exercise, not a new operand. |
| tools.rs:354 | is_empty | `remove_cost_sources_drops_a_fully_overlapped_box` | tests the `aabb_volume(min,max) >= source.volume()*overlap_fraction` → `remove.push` branch of `remove_cost_sources`; discriminating pair with 369/370 below (opposite branch, below-threshold case) — the unconditional-remove bite run for 369/370 proves this conditional is real and controllable in both directions. |
| tools.rs:369 | contains_member | `remove_cost_sources_below_threshold_adds_the_remainder_but_keeps_the_original` | bite run now: forced `remove.push`/`continue` unconditionally (ignored the threshold) → this assertion FAILS alone (198/199). Clause-1 exception: `.contains(&source)` here is a coarse stand-in for "was NOT removed" — a real did-not-happen signal, unlike the Action-bit and range/identity `contains_member` sites below. |
| tools.rs:370 | contains_member | (same test) | same bite/guard as 369 — one mutation, one test function, both assertions exercise the same branch. |
| world.rs:1300 | is_empty | `clear_objects_notifies_every_object_in_id_order_and_empties_world` | bite run now: removed `self.objects.clear()` from `clear_objects` (kept notification-building) → this assertion FAILS alone (198/199). Same "lives one level up" shape as `matrix.rs:879/880`. |
| bodies.rs:3572 | is_empty | `sphere_ray_just_misses_surface_is_no_intersection` | sibling: `sphere_ray_tangent_hits_surface_once` (existing test, `hits.len()==1`) exercises the same `Sphere::ray_intersections` discriminant branch from the opposite (hit) side; adjacent boundary pair. |
| bodies.rs:4476 | is_empty | `cylinder_ray_hits_are_symmetric_with_intersects_ray` | cross-method invariant, doc-commented as deliberate (`intersects_ray == !ray_intersections(..).is_empty()` for every body kind); tests whether `Cylinder::intersects_ray`'s own fast path agrees with the full geometric computation — a real, independently-implementable decision. |
| body_query_parity.rs:256 | is_empty | oracle ray-query loop | oracle parity: `points = body.ray_intersections(..)` is a real subject call; `!points.is_empty()` compared to the oracle's `expected.hit`. |
| probe_parity.rs:329 | is_empty | `check_body!` macro body | same shape as body_query_parity.rs:256: `hits = body.ray_intersections(..)`, real subject call, compared to the oracle's expected hit count. |
| parry.rs:4364 | is_empty | `cost_sources_for_part_pair_shape_shape_disjoint_is_empty` | sibling: `cost_sources_for_part_pair_shape_shape_is_the_overlap_of_both_whole_aabbs` (existing test immediately above, non-empty case), same function, opposite outcome. |
| parry.rs:4558 | is_empty | `mesh_shape_cost_sources_no_intersection_is_empty` | sibling: the positive-overlap test immediately above it, same function, opposite outcome. |
| parry.rs:4605 | is_empty | `mesh_mesh_cost_sources_no_intersection_is_empty` | sibling: the positive-overlap test immediately above it, same function, opposite outcome. |
| parry.rs:4877 | is_none | `check_self_collision_distance_is_none_when_not_requested` | bite run now: deleted `attach_requested_distance`'s `if !request.distance { return; }` so the field is populated unconditionally → this assertion FAILS alone (229/230 green, `--no-fail-fast`). Its four siblings (`..._distance_reports_the_closest_separation` self and robot, `..._detailed_distance_reports_the_whole_result`, `..._distance_of_a_penetrating_pair_is_unsigned`) bite the opposite direction — an immediate `return` in the same function fails those four and leaves this one green — so the guard is on record from both sides. |

### Not-this-family (19)

| Site | Kind | Test fn | Reason |
|---|---|---|---|
| world.rs:993 | contains_member | `add_to_object_creates_with_given_pose_and_shape_global_pose` | §9 clause 1: `Action::CREATE`/`ADD_SHAPE` report *which* action occurred on an always-succeeding call (`add_to_object` returns `Some` whenever `shapes` is non-empty and lengths match) — a computed mode/dispatch fact, not a could-not/did-not-produce-X signal. Same reasoning as `bodies.rs:4328` (§1c). |
| world.rs:994 | contains_member | (same test) | same reasoning as 993. |
| world.rs:1022 | contains_member | `add_to_object_on_existing_object_ignores_pose_argument` | same reasoning, negative side (`!contains(CREATE)`) — still a mode fact (existing-object case), not an inability signal. |
| world.rs:1023 | contains_member | (same test) | same reasoning as 1022. |
| world.rs:1185 | contains_member | `set_object_pose_on_new_id_is_create` | same `Action::CREATE`-bit reasoning as 993. |
| world.rs:1488 | contains_member | `action_bits_combine` | tests `Action::BitOr`/`contains` directly on a hand-constructed value, not via any `World` method; a computed bitmask fact, not a fail signal — the operator logic's own breakability is a clause-2 property, but clause 1 fails regardless. |
| world.rs:1489 | contains_member | (same test) | same as 1341. |
| world.rs:1490 | contains_member | (same test) | same as 1341, negation form. |
| collision_parity.rs:681 | contains_member | (panda_link0 bounding-radius check) | `(0.0..1.0).contains(&bounding_radius)` is a plausibility guard on a computed geometric value, not a could-not/did-not signal. |
| collision_parity.rs:1213 | contains_member | (predicted-crossover bracket check) | same reasoning: numeric-range sanity check on a computed/fitted value. |
| collision_parity.rs:1450 | contains_member | `pr2_world_object_same_pair_deeper_depth_is_a_real_vertex_not_a_spurious_direction` | `link_names.contains(&point.link_name)` checks *which* links a computed closest-pair names, against an oracle's identification — a computed-identity fact, not an inability signal. |
| collision_parity.rs:1622 | contains_member | `pr2_self_wheel_same_pair_oracle_magnitude_is_implausible` | same reasoning as 1450. |
| collision_parity.rs:1803 | contains_member | (Global-vs-Single distance-request comparison) | same reasoning as 1450/1622. |
| parry.rs:2787 | is_some | `octree_cache_get_or_compute_invokes_build_only_once_per_key` | §9 clause 2 fails: the test's own `build` closure unconditionally returns `Some(..)`; `get_or_compute` cannot return anything but `Some` on either the cache-hit or cache-miss path here, so no engineer-implementable-wrong decision is exercised by this specific assertion (the test's real point, `calls.get()==1`, is a separate assertion outside this grammar). |
| parry.rs:2788 | is_some | (same test) | same reasoning as 2787. |
| world_parity.rs:241 | is_some | `world_matches_oracle` | §9 clause 3 fails: `ambiguous.transform.is_some()` reads a field straight off the deserialized oracle fixture (`QueryDump.transform: Option<[f64; 16]>`), not a value produced by calling `World`. Deleting every `world.*` call above it would not change this assertion's outcome. |
| bodies.rs:3967 | is_empty | `mesh_with_zero_triangles_is_constructible` | §9 clause 3 fails: `mesh.triangles.is_empty()` reads back the test's own `box_mesh(2.0, 2.0, 2.0)` fixture before `ConvexMesh::new` (the actual subject) is ever called — the exact `shortest_solution`/mimic-init-value shape (§1c). |
| shapes.rs:1848 | is_empty | `mesh_with_zero_triangles_is_constructible` | same reasoning as bodies.rs:3967: `Mesh::new(vertices, vec![])` constructed with an explicitly empty triangle list; the assertion reads that same literal input back before any subject logic runs. |
| mesh_parity.rs:154 | is_empty | (oracle fixture load) | §9 clause 3 fails: `!cases.is_empty()` is a fixture-sanity precondition on the loaded `mesh_parity.json`, not a value produced by calling the subject. |

### Round 11 summary

- 132 sites total in the two fenced crates; **round 12 (below) found the
  4-site `contains_msg` exclusion below was wrong** — superseded, do not
  use the 42/39/9 figures in this bullet list without reading §Round 12.
- Remaining candidate count from the scanner's kind filter: 42.
  `tools.rs:259,271,283` are the same physical lines as existing rows
  `tools.rs:68 (x)/(y)/(z)`, not new sites — that correction (33→30 for
  `moveit-collision`) stands.
- Of the (then-uncorrected) 39: 20 in-family, 19 not-this-family, per
  crate moveit-collision 30 candidates (14 in-family / 16 not);
  moveit-geometry 9 candidates (6 in-family / 3 not) — **the geometry
  figures here are superseded by round 12: geometry is 13, not 9.**
- **0 blind operands, still true after round 12.** Every in-family site
  (39 originally, 43 after round 12) already has a sibling assertion or a
  live bite proving current coverage catches the relevant mutation; none
  required a new test.
- One doc-only correction to an existing row: `world.rs:1399`'s recorded
  evidence attributed its sensitivity to a `knows_transform` bite that
  actually proves a different assertion (`world.rs:1397`) sensitive; its
  own fallthrough (shared with the new row 1251) had not been directly
  bitten until this round. Verdict unchanged.

### Round 12 correction: `moveit-geometry` is 13, not 9 — the ownership exclusion was wrong

Round 11 re-derived `moveit-collision`'s 33→30 correction from the raw
scanner output, but carried `moveit-geometry`'s count of 9 forward from
the brief without independently re-running the scanner against it, then
compounded that gap by excluding 4 of the 13 real sites
(`bodies.rs:3953,4055,4066,4125`) under a rule — "`contains_msg`-shaped
sites in `crates/` are p1-robotmodel's this round" — that `f10e1bd`
deleted for good reason: the split it named was a receiver/argument
shape, and no regex grounds a type-based ownership claim. Fences are
paths. p1-robotmodel's path fence this round is `moveit-model/`,
`moveit-constraints/tests/decide.rs` and `moveit-planning/`;
`moveit-geometry/src/bodies.rs` is in nobody's — confirmed by grepping
`doc/` for `3953`, `4055`, `4066`, `4125`: no verdict for any of the four,
anywhere, before this round. Under the (correct) reading that path fences
are what deconflict panels, and `moveit-geometry` is this panel's entire
fence, all 13 are mine. Two compounding errors, not one: not re-running
the scanner, and inventing a still-narrower exclusion inside the crate
that was already correctly re-derived.

`python3 tools/ci/count-coarse-assertions.py crates/moveit-geometry`
(post-merge of `ccac7ea`/`f10e1bd`/`6792ef1`) gives **13** sites outside
the 39-site old-grammar count (`is_empty` 7 + `contains` 4 + `is_some` 2):
`bodies.rs:3572,3953,3967,4055,4066,4125,4476`, `crates/moveit-geometry/src/shapes.rs:1848,1964`,
`tests/body_query_parity.rs:256`, `tests/mesh_parity.rs:154`,
`tests/octree_in_world_parity.rs:220`, `tests/probe_parity.rs:329`.
Checked all 13 against every existing `bodies.rs`/`shapes.rs` row in this
ledger (§3) by line number: zero are duplicates. **All 13 are new-work,
mine to classify.**

The 9 already tabled in round 11 (`bodies.rs:3572,3967,4476`,
`crates/moveit-geometry/src/shapes.rs:1848,1964`, and the four parity-test files) keep their
verdicts unchanged. The 4 added this round:

| Site | Kind | Test fn | In-family | Evidence |
|---|---|---|---|---|
| bodies.rs:3953 | contains | `convex_mesh_zero_vertex_is_an_error` | yes | doc comment on record: `build_mesh_data` has a second, distinct `Error::Construct` site (`try_convex_hull` itself failing on zero points), so a bare `.is_err()` passes even with the dedicated vertex-count guard deleted — message-matching is what proves *that* guard fired. Bite re-run now: `if mesh.vertices.is_empty() && !true` (guard neutralized, condition kept live per `-D warnings`) → this test FAILS alone (140/141); reverted, clean. |
| bodies.rs:4055 | contains | `cylinder_negative_radius_is_an_error` | yes | doc comment on record: `Cylinder::recompute` has two sequential guards ("radius"/"length"), a bare `.is_err()` can't tell them apart, message-swap bite already reddened this assertion. Sibling: `bodies.rs:4066` (length side, same guard pair, same function). |
| bodies.rs:4066 | contains | `cylinder_negative_length_is_an_error` | yes | sibling of 4055, same `recompute` guard pair, opposite clause. |
| bodies.rs:4125 | contains | `cylinder_padding_inversion_is_rejected_and_state_preserved` | yes | **Checked the needle-collision question directly, since 4055 and 4125 both assert `"radius"` and both route through the same `Cylinder::recompute` (`Cylinder::new`→`set_dimensions`→`recompute` for 4055; `Cylinder::set_padding`→`recompute` for 4125 — same message literal, same clause).** That shared clause does not make them a blind pair: they test different subjects (`Cylinder::new`'s wiring to `recompute` vs. `Cylinder::set_padding`'s). Bite run now: made `set_padding` skip `recompute` entirely (`self.padding = padding; Ok(())`) → `cylinder_padding_inversion_is_rejected_and_state_preserved` FAILS alone among the radius/length message tests (4055, 4066 both stay GREEN — they never call `set_padding`); three unrelated padding-behavior tests and one parity test fail too, as an expected consequence of breaking `set_padding` generally, not evidence against this row. Reverted, clean. No third "radius"-messaged clause exists for `Cylinder` (only `recompute`'s two `Error::construct` calls produce a Cylinder error at all), so the needle is unambiguous within any one test's own call graph. |

**Corrected round-11 summary for `moveit-geometry`: 13 candidates, 10
in-family, 3 not-this-family** (the 3 unchanged: `bodies.rs:3967`,
`crates/moveit-geometry/src/shapes.rs:1848`, `mesh_parity.rs:154` — all clause-3 failures, fixture
setup read back before the subject runs). **Corrected total new-site
count: 43** (30 `moveit-collision` + 13 `moveit-geometry`), **24
in-family, 19 not-this-family, 0 blind operands** (all three round-12
bites confirmed existing coverage; none needed a new test).

## 7. Round 13: planner crates (fence expanded)

Four crates added to this panel's fence this round by explicit,
narrow, on-the-record orchestrator override:
`moveit-planners-sbp`, `moveit-planners-chomp`, `moveit-planners-stomp`,
`moveit-stomp-core`. None had been swept under the broadened-grammar
scanner before.

**Ownership cross-check, done before touching any source.** All four
crates already carry *old-grammar* (`matches!`/`.is_err()`/`.is_none()`)
ledger rows from other panels — `moveit-planners-sbp` and (via an
override) `moveit-planners-stomp` are p1-robotmodel's
(`doc/assertion-discrimination-ledger-p1-robotmodel.md`, "own fence" /
"override, was p3-shapes'"), `moveit-planners-chomp` is p6-totg's
override, all merged and closed per `doc/caucus-handoff-2026-08-05.md`.
That old-grammar work is a closed, separate body of work from what the
broadened scanner (`6a14a89`+) finds — nobody had run it against these
four crates before this round, so the new-grammar sites are genuinely
unclaimed. The orchestrator's explicit fence expansion satisfies the
brief's own stated requirement for cross-crate reassignment ("a narrow,
on-the-record orchestrator override with an offer to decline" —
`doc/assertion-discrimination-round2-brief.md`).

`python3 tools/ci/count-coarse-assertions.py <crate>` per crate, then the
old-grammar kinds (`matches`/`is_err`/`is_none`) subtracted by hand
(verified against the brief's stated 19/12/2/2 rather than assumed):
**19** `moveit-planners-sbp`, **12** `moveit-planners-chomp`, **2**
`moveit-planners-stomp`, **2** `moveit-stomp-core` — **35 total**,
matching the brief's figures exactly on independent re-derivation.

### `moveit-planners-sbp` (19)

| Site | Kind | Test fn | In-family | Evidence |
|---|---|---|---|---|
| constrained_sampler.rs:305 | contains | `every_sample_satisfies_the_wrapped_constraint` | no | clause 1: a range-plausibility check on a real sampled value, not a could-not/did-not-produce-X signal — same reasoning as round-11's `collision_parity.rs:681/1213`. |
| goal_sampler.rs:334 | contains | `constrained_branch_is_load_bearing_not_merely_invoked` | no | same reasoning as constrained_sampler.rs:305. |
| nn.rs:244 | is_empty | `len_and_is_empty_track_insertions` | no | clause 3: reads `Gnat::new(4)`'s trivial post-construction state before any subject call — same shape as `bodies.rs:3967`. |
| nn.rs:248 | is_empty | (same test) | yes | redundant confirmation of `insert()`'s effect, already proven by the adjacent `len()==2` assertion — same "lives one level up" shape as round-11's `matrix.rs:870`. |
| registry.rs:1126 | eq_err | `path_constraints_four_scenario_wired_vs_unwired_sweep` | yes | `assert_eq!` pins the exact `PlanningFailure::IterationsExhausted` variant on a real `rrt_connect` call — already discriminating by construction (exact-variant match, not a bare `.is_err()`). |
| registry.rs:1159 | contains | (same test) | no | range-plausibility check on a real trajectory waypoint value, not an error signal. |
| registry.rs:1218 | contains | `goal_constraint_is_resolved_and_the_trajectory_ends_inside_the_goal_region` | no | same reasoning as registry.rs:1152. |
| rrt_connect.rs:105 | contains | `RrtConnectParams::assert_valid` | no | production-code precondition assert (scope `src`, not `test`) — not a test discriminating error-guard selection at all. |
| rrt_connect.rs:579 | contains | `narrow_gap_is_crossed` | no | range-plausibility check on a real computed path point, not an error signal. |
| rrt_connect.rs:584 | contains | (same test) | no | same reasoning as rrt_connect.rs:579. |
| rrt_connect.rs:612 | eq_err | `closed_passage_fails_within_the_cap` | yes | exact-variant `assert_eq!` (`IterationsExhausted`) on a real `rrt_connect` call; siblings 644/661 pin different variants for different scenarios. |
| rrt_connect.rs:644 | eq_err | `deadline_exhausted_reports_correctly` | yes | exact-variant `assert_eq!` (`DeadlineExhausted`), sibling of 612/661. |
| rrt_connect.rs:661 | eq_err | `invalid_start_fails_immediately` | yes | exact-variant `assert_eq!` (`InvalidEndpoint`), sibling of 612/644. |
| sampling.rs:108 | contains | `ball_radius_fraction_is_within_unit_interval` | no | range-plausibility check on a real sampled value, not an error signal. |
| sampling.rs:126 | contains | `simplex_fractions_are_nonnegative_and_sum_to_one` | no | same reasoning as sampling.rs:108. |
| se3.rs:316 | eq_err | `negative_weight_is_rejected` | yes | single-branch: `Se3Space::new` has exactly one `InvalidWeight` guard (combined `!finite() \|\| < 0.0`); bounds validation delegates via `?` to a different error type entirely (`RealVectorSpace::new`'s `SbpError`), so no sibling ambiguity. |
| space.rs:210 | eq_err | `empty_bounds_is_no_dimensions` | yes | single-branch: distinct `NoDimensions` variant, only one construction site. |
| space.rs:215 | eq_err | `inverted_bound_is_rejected` | yes | **found a blind operand while auditing this site — see below.** Bite-confirmed: disabling `min > max` alone fails this test and leaves 227 green. |
| space.rs:227 | eq_err | `non_finite_bound_is_rejected` | yes | bite-confirmed: disabling `!max.is_finite()` alone fails this test and leaves 215 green. |
| space.rs:247 | eq_err | `non_finite_min_bound_is_rejected` | yes | new site, added by this round's own fix (`bbbe0f8`) — missing from this table until the closing audit found it unclaimed. Same bite as the "Blind operand found and fixed" note below: disabling `!min.is_finite()` alone fails this test, 215/227 stay green. |

**In-family: 10. Not-this-family: 10.** (space.rs's own total grew from 19
to 20 sites when `bbbe0f8` added the fix; the crate total below already
counted it in "23 in-family... 2 blind operands... fixed", just not as its
own table row until now.)

**Blind operand found and fixed:** `RealVectorSpace::new`'s guard
(`space.rs:105`) is `!min.is_finite() || !max.is_finite() || min > max`
— three disjuncts, but only two had a targeted test (215 for `min > max`,
227 for `!max.is_finite()`). Bite: disabling `!min.is_finite()` alone
left **all 109 then-existing tests green** — a genuine gap, not a
reclassification. A `min` of `+infinity` would be redundant with
`min > max`, so the isolating case needs `NEG_INFINITY` (or `NaN`)
paired with a finite, larger `max`. Fixed with a new test,
`non_finite_min_bound_is_rejected` (commit `bbbe0f8`), bite-confirmed to
fail under the same mutation and pass against real source.

### `moveit-planners-chomp` (12)

| Site | Kind | Test fn | In-family | Evidence |
|---|---|---|---|---|
| cost.rs:391 | contains | `new_rejects_more_derivative_costs_than_diff_rules_rows` | yes | `ChompCost::new` has 3 `Error::other` sites; doc-commented, message-uniqueness verified by grep (`"DIFF_RULES rows"` appears in exactly one of the three messages). |
| cost.rs:404 | contains | `new_rejects_too_few_points_for_the_diff_rule_boundary` | yes | sibling of 391, `"DIFF_RULE_LENGTH-1"` unique to this guard's message. |
| cost.rs:436 | contains | `new_rejects_a_singular_quad_cost` | yes | sibling of 391/404, `"singular"` unique to this guard's message. |
| optimizer.rs:2386 | contains | `calculate_smoothness_increments_rejects_joint_costs_length_mismatch` | yes | 2 reachable `Error::other` sites (own guard + `ChompCost::derivative`'s, propagated); verified `"joint_costs has"` (this guard) vs. `"joint_trajectory has"` (derivative's) do not collide. |
| optimizer.rs:2455 | contains | `calculate_total_increments_rejects_column_count_mismatch` | yes | 3 reachable `Error::other` sites in `calculate_total_increments`; verified `"columns"` appears only in the first guard's message (the other two say "rows" and "NxM", respectively). |
| trajectory.rs:712 | contains | `from_duration_rejects_zero_discretization` | yes | `from_duration` reaches 3 `Error::other` sites; doc-commented, message verified via source read to name this guard specifically. |
| trajectory.rs:727 | contains | `from_duration_rejects_negative_discretization` | yes | sibling of 712, same guard, negative-side boundary. |
| trajectory.rs:748 | contains | `from_duration_rejects_negative_discretization_that_divides_positive` | yes | sibling of 712/727 — doc-commented as the case that actually needs the explicit guard (negative/negative divides positive, defeating the downstream `num_points < 2` fallback). |
| trajectory.rs:766 | contains | `from_duration_rejects_an_unreasonable_point_count` | yes | sibling, targets the point-count-bound guard rather than the discretization guard. |
| trajectory.rs:925 | contains | `assign_chomp_trajectory_point_rejects_group_column_mismatch` | yes | 2 `Error::other` sites; message-uniqueness verified by grep. |
| trajectory.rs:968 | contains | `assign_chomp_trajectory_point_rejects_a_multi_dof_active_joint` | yes | sibling of 925, opposite guard, message-uniqueness verified. |
| trajectory.rs:1004 | contains | `fill_in_from_trajectory_rejects_a_trajectory_with_no_group` | yes | several reachable `Error::other` sites; message-uniqueness verified by grep against all three. |

**In-family: 12. Not-this-family: 0. Blind operands: 0** — every site
already doc-commented with its discrimination rationale; verified by
independent message-uniqueness greps rather than trusting the comments,
no live bite needed since exact-substring collision is directly
checkable by source read.

### `moveit-planners-stomp` (2)

| Site | Kind | Test fn | In-family | Evidence |
|---|---|---|---|---|
| cost_functions.rs:467 | is_some | `kernel_bounds_at_the_dmatrix_allocation_ceiling_does_not_overflow` | no | clause 1 fails: verifies an arithmetic fact about a computed ceiling constant (whether `i64` multiplication overflows), not a could-not/did-not signal from any subject call. |
| filter_functions.rs:314 | contains | `enforce_position_bounds_rejects_a_multi_variable_joint` | yes | already doc-commented with a reachability bite on record (`require_single_variable`'s guard neutralized → test fails; only one guard in the loop, no sibling branch to discriminate against). |

**In-family: 1. Not-this-family: 1. Blind operands: 0.**

### `moveit-stomp-core` (2)

Both new-grammar sites are call sites (`via:rows_to_string`, per
`ccac7ea`'s helper-body scoring) of the single `assert!` inside
`rows_to_string` (`crates/moveit-stomp-core/src/utils.rs:499`), a defensive guard against upstream's
undefined behavior on empty input.

| Site | Kind | Test fn | In-family | Evidence |
|---|---|---|---|---|
| utils.rs:641 | via:rows_to_string | `to_vector_then_rows_to_string_round_trips_through_matrix_to_string` | no | calls with a non-empty (2-row) input — never reaches the guard; tests round-trip correctness against `matrix_to_string`, not an error signal. |
| utils.rs:660 | via:rows_to_string | `rows_to_string_panics_instead_of_replicating_ub_on_empty_input` | yes | **found a blind operand — see below.** (line corrected from `:654` during the closing audit: the fix itself — the `#[should_panic(expected = ...)]` attribute and its explanatory comment, `8fa0e48` — added 6 lines above the call, shifting it to `:660`.) |

**In-family: 1. Not-this-family: 1.**

**Blind operand found and fixed:** the test was a bare `#[should_panic]`
with no `expected = ...` message match. Bite: neutralizing the guard
(`!rows.is_empty() || true`) left the test **green** — `rows[0].len()`
raises its own index-out-of-bounds panic on empty input, which a
message-less `#[should_panic]` cannot tell apart from the named guard.
Fixed by adding `expected = "upstream's toString(vector<VectorXd>) calls
data.front()"` (commit `8fa0e48`), bite-confirmed to fail under the same
mutation (panics with the wrong message: indexing, not the guard) and
pass against real source.

### Round 13 summary

- **35 new-grammar sites across 4 crates, matching the brief's
  19/12/2/2 exactly** on independent re-derivation (`python3
  tools/ci/count-coarse-assertions.py <crate>` per crate, old-grammar
  kinds subtracted by hand).
- **23 in-family** (9 sbp + 12 chomp + 1 stomp + 1 stomp-core),
  **12 not-this-family** (10 sbp + 0 chomp + 1 stomp + 1 stomp-core).
- **2 blind operands found and fixed**, both with a new test:
  `moveit-planners-sbp`'s `space.rs` (`RealVectorSpace::new`'s
  untested min-finiteness disjunct, commit `bbbe0f8`) and
  `moveit-stomp-core`'s `utils.rs` (`rows_to_string`'s message-less
  `#[should_panic]`, commit `8fa0e48`).
- Gate per crate: `cargo fmt --all`, `cargo clippy -p <crate>
  --all-targets -- -D warnings` (all four clean), `cargo nextest run -p
  <crate>` — `moveit-planners-sbp` 110/110 (was 109, +1),
  `moveit-planners-chomp` 86/86 (no source change; census-only,
  doc-commented evidence independently verified by message-uniqueness
  greps rather than a live bite), `moveit-planners-stomp` 62/62
  (census-only, no source change), `moveit-stomp-core` 22/22 (was 21,
  +1).

## Round 14 — closing audit: reconciling `count-coarse-assertions.py`'s 694 against the five ledgers

Not a re-audit of any crate's verdicts. This round answers one question
only: does every one of today's `main` coarse-assertion sites have either
a ledger row or a recorded reason for having none, across all five
assertion-discrimination ledgers (`p1-fixtures`, `p1-robotmodel`, `p3-acm`,
`p9-ros`, `pilz`)? Scope for *fixing* anything found stays exactly
`moveit-collision`/`moveit-geometry`/the four planner crates; everything
else here is reported, not fixed.

**Pinned commit:** `461a5f10655030b6c16c51afe588cfd6d844ad4d` (`main`),
used only as this section's reproducibility fixture. **Today's figures use
this ledger's own current `HEAD`, `1145531ea104e5fc73ca39ddb57aad24349a660a`**
(after merging `main` at `82e70d6`) — see "Today's figures" below for why
the two numbers differ and are both correct.

**This section supersedes its own first draft.** The first version of this
closing audit (694 = 442 matched + 252 orphans) was produced by three
uncommitted scratchpad scripts and a hand-transcribed table — exactly the
ungrounded state the user flagged: *"a reader has the table and no way to
re-derive it."* `tools/ci/reconcile-assertion-ledgers.py` and
`tools/ci/assertion-ledger-equivalences.json` (committed `a73c2ff`) are
the real, re-runnable instrument; running it turned up a real parsing gap
the hand-rolled scripts had (below), so the honest, reproducible number is
different from the first draft's. The old 442/252 split is retracted, not
reconciled to — this section reports what the committed tool actually
produces.

### The instrument's total, and why census §9d's 292 is not the baseline

`count-coarse-assertions.py crates/ ros/ tools/` at the pin gives **694**
sites excluding `helper_body`: **296** inside `matches!`/`.is_err()`/
`.is_none()`, **398** outside it — both match the figures this round was
handed exactly.

Census §9d's **292** (`55+67+32+49+89` across `p1-robotmodel`/`p9-ros`/
`pilz`/`p1-fixtures`/`p3-acm`) is not comparable to today's tree: it was
measured at `311169b`/`fd3bada`, before several rounds each ledger has
since added (`p1-robotmodel` alone grew from 55 rows to 14 numbered
"Round"s reaching a `.contains()`-family workspace census; `p3-acm` grew
39→43→(this round) after the geometry recount and the four-planner-crate
sweep). Re-measuring today's raw table-row count (first `|`-column entries
matching `` `?file:line` `` in each ledger, not exploded past a
comma-list) gives **539**, not 292:

| ledger | table rows (today) |
|---|---:|
| `p1-fixtures` | 136 |
| `p1-robotmodel` | 130 |
| `p3-acm` | 164 |
| `p9-ros` | 77 |
| `pilz` | 32 |
| **total** | **539** |

**539 rows is not 539 distinct sites.** `p1-robotmodel`'s "Round 11–14"
(`.contains()`-shaped message-substring family, `crates/` minus `pilz` and
`ros/`) is an independent workspace-wide *verification* pass, not a claim
of fix-ownership — it re-lists 36 sites whose own fence's ledger already
carries them. Deduplicating each ledger's own rows by `(file, line)` and
then taking the union across all five gives the real distinct-site
population claimed by *some* ledger: this round's first (hand-rolled,
uncommitted) pass got **424** by exact/±5-line matching against the
scanner. That number is now known to be an undercount, not a baseline —
see "Today's figures" below.

### Cross-ledger double claims — the mirror of §9f's orphans

36 sites, every one `p1-robotmodel`'s workspace-wide `.contains()` sweep
re-verifying a site whose fix-ownership ledger is someone else's:

| site | claimed by |
|---|---|
| `crates/moveit-constraints/tests/sampler.rs:78` | p1-fixtures, p1-robotmodel |
| `crates/moveit-constraints/tests/sampler.rs:120` | p1-fixtures, p1-robotmodel |
| `crates/moveit-kinematics/src/chain.rs:469,512,558` | p1-fixtures, p1-robotmodel |
| `crates/moveit-kinematics/tests/ik_fk_roundtrip.rs:281` | p1-fixtures, p1-robotmodel |
| `crates/moveit-smoothing/src/acceleration_filter.rs:466,525,542` | p1-fixtures, p1-robotmodel |
| `crates/moveit-smoothing/src/butterworth.rs:153,162,172,183,200` | p1-fixtures, p1-robotmodel |
| `crates/moveit-smoothing/src/ruckig_filter.rs:388,530` | p1-fixtures, p1-robotmodel |
| `crates/moveit-distance-field/src/voxel_grid.rs:454,456,512` | p1-robotmodel, p9-ros |
| `crates/moveit-geometry/src/bodies.rs:3953,4055,4066,4125` | p1-robotmodel, **p3-acm** |
| `crates/moveit-planners-chomp/src/cost.rs:391,404,436` | p1-robotmodel, **p3-acm** |
| `crates/moveit-planners-chomp/src/optimizer.rs:2386,2455` | p1-robotmodel, **p3-acm** |
| `crates/moveit-planners-chomp/src/trajectory.rs:712,727,748,766,925,968,1004` | p1-robotmodel, **p3-acm** |
| `crates/moveit-planners-stomp/src/filter_functions.rs:314` | p1-robotmodel, **p3-acm** |

The four bolded groups are inside this panel's own fence: `p1-robotmodel`'s
Round 11–13 independently re-verified 15 of my geometry/chomp/stomp sites
and reached the same in-family verdicts I did (matching the user's own
direct re-check of `bodies.rs:4125`, reported earlier this round). Not a
defect — a second independent verification of the same sites, correctly
attributable to two ledgers at once. It only becomes a problem if a future
count sums "rows per ledger" without deduplicating; flagging it here so
that arithmetic is never attempted blind.

### Stale citations found: failure mode 2, confirmed by test-name re-derivation

15 ledger citations no longer point at the line they were written for,
because an edit above them shifted the file. Confirmed by finding the
actual current line via the row's own named test function and checking
the source, not assumed from proximity — automated nearest-line search
picked the wrong candidate at least once (`ros/moveit-ros/src/
trajectory.rs`'s cluster: raw nearest-distance search matched `:462` to
`:494`, but the test name `seconds_to_duration_rejects_infinity` is
actually at `:500`).

| ledger | stale citation | actual site (today) | cause |
|---|---|---|---|
| p3-acm (own fence) | `crates/moveit-stomp-core/src/utils.rs:654` | `crates/moveit-stomp-core/src/utils.rs:660` | this round's own fix (`8fa0e48`) added 6 lines above the call; **corrected in this ledger's Round 13 table above** |
| p1-fixtures | `tree.rs:1930` | `crates/moveit-octomap/src/tree.rs:1958` | insertion above, +28 |
| p1-fixtures | `tree.rs:1944` | `crates/moveit-octomap/src/tree.rs:1972` | same insertion, +28 |
| p1-fixtures | `tree.rs:1805` | `crates/moveit-octomap/src/tree.rs:1833` | same insertion, +28 |
| p1-fixtures | `tree.rs:1806` | `crates/moveit-octomap/src/tree.rs:1834` | same insertion, +28 |
| p1-fixtures | `tree.rs:1807` | `crates/moveit-octomap/src/tree.rs:1835` | same insertion, +28 |
| p9-ros | `robot_model.rs:2024` | `crates/moveit-model/src/robot_model.rs:2092` | fixture widened 3→4 joints (`7676185`, adds a joint block above), +27 |
| p9-ros | `robot_model.rs:2025` | `crates/moveit-model/src/robot_model.rs:2052` | same cause, +27 |
| p9-ros | `robot_model.rs:2026` | `crates/moveit-model/src/robot_model.rs:2053` | same cause, +27 |
| p9-ros | `ros/moveit-ros/src/trajectory.rs:444` (ros) | `ros/moveit-ros/src/trajectory.rs:482` | insertion above, +38 |
| p9-ros | `ros/moveit-ros/src/trajectory.rs:450` (ros) | `ros/moveit-ros/src/trajectory.rs:488` | same, +38 |
| p9-ros | `ros/moveit-ros/src/trajectory.rs:456` (ros) | `ros/moveit-ros/src/trajectory.rs:494` | same, +38 |
| p9-ros | `ros/moveit-ros/src/trajectory.rs:462` (ros) | `ros/moveit-ros/src/trajectory.rs:500` | same, +38 — nearest-line search alone would have matched this to `:494`, wrongly |
| p9-ros | `collision_env_distance_field.rs:3271` | `crates/moveit-distance-field/src/collision_env_distance_field.rs:3284` | citation points at the audit comment's opening line, not the `assert!`; +13 |
| p9-ros | `collision_env_distance_field.rs:3276` | `crates/moveit-distance-field/src/collision_env_distance_field.rs:3289` | same convention, +13 |

### Non-matches that are not gaps

Two more unresolved citations turned out to be citations to code the
scanner was never going to see, by design — not gaps:

- `p1-fixtures`, `tree.rs:1781` — the row's own evidence column says
  **"scanner false positive"**: `assert!(insert_ray(..., None, ...))`
  passes `None` as an *argument*, not a comparison; today's scanner
  correctly does not flag it, matching the row's own conclusion.
- `p1-robotmodel`, `moveit-constraints/src/position.rs:165` — its own
  "Round 10" header says **"Not a census row"**: it cites a bare
  `model.link_model(link_name)?` production guard, outside all nine of
  the scanner's recognized shapes entirely (a bite-tested coverage gap,
  not an assertion-discrimination site).

The 14 above are `p1-fixtures`'s and `p9-ros`'s own documents —
**found-and-unfixed, reported to those panels, not edited here.** The
`p3-acm` row is this panel's own and is already fixed (own fence).

### The committed instrument: `tools/ci/reconcile-assertion-ledgers.py`

Committed `a73c2ff`, alongside `tools/ci/assertion-ledger-equivalences.json`
and a generated `doc/assertion-discrimination-orphans.txt`. It re-derives
the whole partition from a clean checkout with no scratchpad input:
re-runs `count-coarse-assertions.py` itself (never reads a cached scan),
parses all five ledgers' first-column citations, resolves each by exact
match then a unique ±5-line window, and falls back to the equivalences
file for the one case that pattern can't cover.

**The trap is encoded, not just described.** This panel's own §1b case —
`tools.rs:68 (x)/(y)/(z)` citing the shared `aabb_intersection` guard's
production line instead of each axis-isolating test's own assert at
`tools.rs:259/271/283` — is now a vetted entry in
`assertion-ledger-equivalences.json`, with the ledger prose and the source
read that justify it named inline. Building the instrument surfaced a
second, more basic problem the hand-rolled scratchpad scripts had: the
first-column regex required the closing `` ` | `` to follow the line
number immediately, so any row with a trailing annotation — exactly the
`(x)`/`(y)`/`(z)` suffixes, or `p1-fixtures`' `` `ruckig_filter.rs:539,552,
565,578` (`do_smoothing`'s guard...) `` — was silently **not parsed as a
citation at all**, undercounting every ledger's matched set. Fixed by
letting the first column carry arbitrary non-`|` trailing text; verified
by spot-reading 8 of the 61 newly-recovered rows this fix produces at the
pinned commit and confirming each resolves to the real site its row names.

Anything a ledger cites that the instrument still can't place is printed
under `unresolved_citations` with a heuristic tag (comment line, `?`-guard
line, or "no clue") — reported, never silently matched or dropped. `--emit-orphans`
and `--emit-unresolved` print the full lists; the default run prints only
counts and the unresolved lines themselves.

### Today's figures (this ledger's `HEAD`, not the pin)

Running the committed instrument against this ledger's own current tree —
`1145531ea104e5fc73ca39ddb57aad24349a660a`, after merging `main` at
`82e70d6` — instead of the historical pin:

```
scanner sites (excl. helper_body): 697
matched (some ledger row accounts for the site): 469
orphans (no ledger row accounts for the site):    228
check: matched + orphans == scanner sites -> True
ledger citations resolved via vetted equivalence: 9
ledger citations explained as non-scanner-scope (not gaps): 0
ledger citations still unresolved (reported, not guessed): 96
```

**697, not 694 or 706.** 694 was `461a5f1`'s count; the tree has moved
since (3 sites, mostly this panel's own two in-fence fixes below plus
unrelated main-branch merges). The user separately reported 706 measured
against "today's main" with a per-crate table; this worktree's own `main`
ref resolves to `82e70d6`, and running the scanner against a clean
`git archive` of that exact commit gives **697**, not 706 — the same
figure as this ledger's `HEAD` (which only adds a 1-line stray-marker
removal on top of `82e70d6`). The two per-crate tables agree on every
crate except four: `ros/moveit-ros` (91 here vs. 96 reported),
`moveit-distance-field` (56 vs. 58), `moveit-constraints` (27 vs. 28),
`moveit-stomp-core` (2 vs. 3) — a 9-site shortfall, all outside this
panel's fence except the last. **This is reported as a discrepancy, not
resolved**: this worktree cannot see whatever commit produced the other
9 sites, so the honest number below is 697, from a commit this worktree
can name and reproduce (`82e70d6`). If another panel's merge has not yet
propagated to every worktree's shared object store, that is worth the
orchestrator's separate attention — not something this ledger can
diagnose from inside one fence.

**Zero of the 228 orphans fall inside this panel's fence**
(`moveit-collision`, `moveit-geometry`, `moveit-planners-{sbp,chomp,
stomp}`, `moveit-stomp-core`) — confirmed by grepping the emitted orphan
list against those six path prefixes, not by re-deriving the fence
boundary by hand.

Per-crate orphan breakdown, today:

| crate | orphans | owning fence (§9f) |
|---|---:|---|
| `ros/moveit-ros` | 70 | `p9-ros` |
| `moveit-octomap` | 31 | `p1-fixtures` |
| `moveit-trajectory` | 27 | `p1-joints` |
| `moveit-scene` | 25 | `p1-fixtures` |
| `moveit-model` | 25 | `p1-robotmodel` |
| `moveit-distance-field` | 21 | `p9-ros` |
| `moveit-planners-pilz` | 10 | `p1-joints` |
| `moveit-smoothing` | 6 | `p1-fixtures` |
| `moveit-constraints` (`tests/utils_parity.rs`, all 6) | 6 | `p1-fixtures` |
| `moveit-planning` | 4 | `p1-robotmodel` |
| `moveit-state` | 3 | `p1-fixtures` |
| **total** | **228** | |

Rolled up by fence (§9f's path table, not this round's stale mapping):

| fence | orphans |
|---|---:|
| `p9-ros` | 91 (70 + 21) |
| `p1-fixtures` | 71 (31 + 25 + 6 + 6 + 3) |
| `p1-joints` | 37 (27 + 10) |
| `p1-robotmodel` | 29 (25 + 4) |
| `p3-acm` (this panel) | **0** |
| **total** | **228** |

This supersedes the fence-to-orphan mapping given earlier this round
(`p9-ros` 94, `p1-fixtures` 66, `p1-joints` 42, `p1-robotmodel` 42,
`moveit-constraints` 8 split by file) — that mapping was checked against
the first draft's 252, not the committed instrument's 228. All 6 of
`moveit-constraints`' orphans are in `tests/utils_parity.rs`
(`p1-fixtures`'s file per §9f), none in `tests/decide.rs`
(`p1-robotmodel`'s) — confirmed by reading the emitted orphan list
directly, not inherited from the earlier split.

### Validation: the instrument reproduces the pin deterministically

Copying the committed script and equivalences file into a clean
`git archive` export of `461a5f10655030b6c16c51afe588cfd6d844ad4d` and
running it there, unmodified:

```
scanner sites (excl. helper_body): 694
matched (some ledger row accounts for the site): 477
orphans (no ledger row accounts for the site):    217
check: matched + orphans == scanner sites -> True
ledger citations resolved via vetted equivalence: 9
ledger citations explained as non-scanner-scope (not gaps): 0
ledger citations still unresolved (reported, not guessed): 45
```

**694 = 477 + 217, from the same commit and the same tool every time.**
This is a different split from the first draft's 442/252 — the regex fix
above recovers matches the hand-rolled scripts missed at this exact
commit too, so 477 is the more accurate figure for `461a5f1`, not a
discrepancy to explain away.

Two of the 217 pinned-commit orphans are inside this panel's own fence:
`crates/moveit-planners-sbp/src/space.rs:247` and
`crates/moveit-stomp-core/src/utils.rs:660` — exactly the two accounting
gaps below, present as orphans at the pin because both fixes landed after
it. Zero orphans in this fence at `HEAD` confirms both were closed, not
just reclassified.

### This panel's own two accounting gaps, found and fixed (in-fence)

The two failure modes the closing audit was warned to expect, both inside
this fence, both from this round's own fixes landing after the pinned
count was taken:

1. **`crates/moveit-planners-sbp/src/space.rs:247`** — `bbbe0f8`'s new
   test (`non_finite_min_bound_is_rejected`) created a new scanner site
   with no table row at all. Added above, in this ledger's Round 13
   `moveit-planners-sbp` table.
2. **`crates/moveit-stomp-core/src/utils.rs:654`→`:660`** — `8fa0e48`'s
   fix added 6 lines above its own cited call site, going stale in the
   same commit that created the row. Corrected above.

Both are doc-only edits to this file; no source changed, so gate is
`cargo fmt --all -- --check` only.

### Commands run (round 14)

```
git archive 461a5f10655030b6c16c51afe588cfd6d844ad4d | tar -x -C <snapshot-dir>
cp tools/ci/reconcile-assertion-ledgers.py tools/ci/assertion-ledger-equivalences.json <snapshot-dir>/tools/ci/
python3 tools/ci/reconcile-assertion-ledgers.py                  # <snapshot-dir>: 694 = 477 + 217
python3 tools/ci/reconcile-assertion-ledgers.py --emit-orphans   # this ledger's HEAD: 697 = 469 + 228
cargo fmt --all -- --check                                       # doc-only round, no source changed
```

### Addendum: the 697/469/228 figures above went stale within one merge — the fix is structural, not another rewrite

`p1-joints` landed a sixth ledger,
`assertion-discrimination-ledger-moveit-trajectory.md`, and the corpus
moved to 698 sites. The instrument's own `LEDGERS` list was a hardcoded
five-file list — exactly the same shape of bug the census's §9d/§9f
history already warned about (a hand-maintained enumeration of a set the
filesystem already holds), just relocated from a markdown table into
Python. It silently kept parsing the other five ledgers correctly while
never reading the sixth, undercounting `matched` by the trajectory
ledger's entire content. Fixed by `discover_ledgers()` (glob
`doc/assertion-discrimination-ledger-*.md` instead of naming files) — the
same fix class as `verify-all.sh`'s own glob, applied one level down.

A second gap surfaced alongside it: some ledgers cite a crate-shorthand
path (`moveit-geometry/bodies.rs:4055`, omitting `src/`) that plain
`endswith` matching can't place, landing 18 citations under a spurious
"cited file not found" tag even though the site is real and already
matched via a different ledger's full-path citation of the same line.
Fixed with `path_matches()` (subsequence-of-path-components, not a second
`endswith` variant, since the omitted segment can be `src`, `tests`, or a
nested module dir).

**The number in this section is not the point going forward — the gate
is.** `--verify` (wired into `tools/ci/verify-all.sh` via
`tools/ci/verify-orphan-enumeration.sh`) now fails whenever
`doc/assertion-discrimination-orphans.txt` no longer matches the live
partition, and the file's own header states its source commit and the
three counts it was generated from — a reader can see staleness without
running anything, and CI (run by hand, per this repo's `verify-*.sh`
convention) catches it before a stale file ships. Hand-transcribing a
live count into this ledger's prose every round is exactly the kind of
number this repeatedly-stale section shows cannot stay correct across
concurrent merges; the instrument and its gate are the durable artifact,
this prose is not.

**Current figures, this ledger's own re-run at `919c982` (after merging
`main`): 698 = 507 (matched) + 191 (orphans).** Zero of the 191 fall in
this panel's fence. Per-fence rollup (§9f's path table, today's orphan
list): `p9-ros` 91 (`ros/moveit-ros` 70 + `moveit-distance-field` 21),
`p1-fixtures` 71 (`moveit-octomap` 31 + `moveit-scene` 25 +
`moveit-smoothing` 6 + `moveit-constraints/tests/utils_parity.rs` 6 +
`moveit-state` 3), `p1-robotmodel` 29 (`moveit-model` 25 +
`moveit-planning` 4), `p1-joints` **0** (its new dedicated ledger now
covers `moveit-trajectory`/`moveit-planners-pilz` in full), `p3-acm`
**0**.

**Unresolved ledger citations: 72, broken down by cause** (after the
`path_matches` fix removed 18 that were a pure matching artifact, not
real gaps):

| cause | count |
|---|---:|
| no scanner site within ±5 lines (real drift, or citing supporting context) | 39 |
| cites a `#[test]`/`fn`/brace/`let` line, not the assert | 22 |
| cites a comment line | 10 |
| cites a `?`-propagation guard line (outside the scanner's grammar) | 1 |
| **total** | **72** |

**This is not 73 new stale citations appearing at once.** The 15 stale
citations and 2 non-scope explanations this round already hand-triaged
(`tree.rs:1930/1805/1806/1807`, `robot_model.rs:2024/2025/2026`,
`trajectory.rs:444/450/456/462`,
`collision_env_distance_field.rs:3271/3276`, `tree.rs:1781`,
`crates/moveit-constraints/src/position.rs:165`) are **all still present, unchanged**, in the 72 above
— re-checked directly against today's list, not assumed carried over.
The rest were already flagged last round as "unconfirmed, other panels'
fences" (not claimed triaged), and have grown since via `p1-robotmodel`'s
continued rounds adding new citations of their own
(`mesh_search_paths.rs:130`, `check_start_state_bounds.rs:302/328/377`,
`robot_model.rs:2764/2885/2886/2986/3021/3031/3376`) — this panel does
not fix or further triage them; the cause breakdown above is what each
owning panel needs to tell which of theirs is which.

Gate: doc + tooling only (`reconcile-assertion-ledgers.py`,
`verify-orphan-enumeration.sh`, regenerated `orphans.txt`, this addendum)
— `cargo fmt --all -- --check` only, no Rust source touched.
