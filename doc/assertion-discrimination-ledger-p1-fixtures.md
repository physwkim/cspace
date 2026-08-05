# Assertion-discrimination ledger — p1-fixtures

Per-site record for the 7 crates assigned this round: `moveit-scene` and
`moveit-metrics` (original fence) plus five on-the-record scope
overrides, this round only, all five accepted and none declined:
`moveit-octomap` (p3-acm's), `moveit-state` (p3-acm's),
`moveit-constraints` (p1-robotmodel's), `moveit-smoothing` (p3-acm's),
`moveit-srdf` (p9-ros's).

Enumerated with a paren-depth scanner (`census.py`) I wrote this round
from `doc/assertion-discrimination-census.md`'s §1 prose description of
its algorithm (mask comments, depth-track parens, classify `matches!`
(priority) vs bare `.is_err()`/`.is_none()`, match-start dedup) — not
from that document's own `census.py` source. That script is retained in
a different panel's (p9-ros's) worktree scratchpad per the census
doc's §6, a path outside this panel's access; I never read it. This is
therefore independent replication from a spec, not a re-run of the
same code, re-run against the current tree (post `git merge --ff-only
main`, which fast-forwarded cleanly since this branch was already an
ancestor).

## Reconciliation against the census figures

| crate | census (matches!+bare) | this scan | match? |
|---|---|---|---|
| moveit-scene | 1+13=14 | 1+12=13 | **no — see below** |
| moveit-octomap | 0+10=10 | 0+10=10 | yes |
| moveit-state | 1+9=10 | 1+9=10 | yes |
| moveit-constraints | 4+5=9 | 4+5=9 | yes |
| moveit-metrics | 3+0=3 | 3+0=3 | yes |
| moveit-smoothing | 2+0=2 | 2+0=2 | yes |
| moveit-srdf | 2+0=2 | 2+0=2 | yes |
| **total** | **50** | **49** | |

**Finding: moveit-scene is a fix-driven count change, not an instrument
disagreement.** `git diff df36fab HEAD -- crates/moveit-scene/` is
exactly one commit's worth of change, and `git log --oneline
df36fab..HEAD -- crates/moveit-scene/` shows only `bedc8ea` (my own,
this session, prior round) touching the crate. `git merge-base
--is-ancestor bedc8ea df36fab` returns false — `bedc8ea` postdates the
census's measurement point. That commit converted
`assert!(child.attached_bodies().next().is_none())` into an
`assert_eq!` (out of the bare-`.is_none()`-inside-`assert!` family
entirely, onto a discriminating equality check), which is exactly why
the current tree has 12 bare sites where the census counted 13. Both
scanners are correct; they were run against different trees. 49 is the
correct current denominator for this round's 7 crates, not 50.

No other crate showed a disagreement.

## Folded multi-operand condition audit (this round)

p3-acm found 12 sites in `moveit-geometry` where a `single-branch`
verdict rested on a constructor/producer *count* — one `Err`/one
message — while the guard's condition itself folded two or more named
operands into one `||`/`&&`. A count of construction sites cannot see
that shape: it proves there is one *site*, not that there is one
*coverable branch*. Only an isolating mutation (neutralize one
operand's clause, confirm which assertion fails, then the mirror) can
tell `single-branch` from `discriminating` when the condition is
folded.

Audited: the 22 rows that were `single-branch`/`structural` before the
previous round's reclassification added 9 more into that bucket
(`scene.rs`'s `decouple_parent`/`frame_transform` rows — themselves
already checked against a *different* failure mode, single-producer-site,
last round; none of those 9 fold multiple named operands into one
condition, so they are correctly out of this audit's scope). Per-row
disposition:

- **`node.rs:153`** (moveit-octomap) — `create_child`'s guard is one
  predicate (`self.children.as_deref()?.get(i)`) over one value (slot
  index `i`), not a folded condition. Signature does not apply.
- **`invariants.rs:585,586,587,588,589,594,595,596,597`** (moveit-state,
  9 rows) — re-read all 9 accessor bodies (`state.rs:369-513`): each
  has exactly one `?` on one named operand (`name`), no folded OR/AND.
  Signature does not apply to any of the 9.
- **`constraint_sampler_manager.rs:172`** (moveit-constraints) — guard
  is `if regions.len() != 1` in `with_updated_position`, one condition
  over one value (region count). Signature does not apply.
- **`decide.rs:183,184`** (moveit-constraints) — **matches the
  signature.** `JointConstraint::new`'s guard (`joint.rs:120`) is
  `tolerance_above < 0.0 || tolerance_below < 0.0`, one `Err::construct`
  folding two named operands. Bit both directions: neutralizing
  `tolerance_above`'s clause alone (`if false && tolerance_above < 0.0
  || tolerance_below < 0.0`) failed the assertion at `decide.rs:183`
  exactly; neutralizing `tolerance_below`'s clause alone failed at
  `decide.rs:184` exactly. The two existing test call sites
  (`tolerance_above=-0.1,tolerance_below=0.1` at 183;
  `tolerance_above=0.1,tolerance_below=-0.1` at 184) already isolate
  one operand each — genuinely `discriminating`, no fix needed. `joint.rs`
  reverted clean after each bite. **Verdict corrected below.**
- **`utils_parity.rs:221,641,885,896,942`** (moveit-constraints, 5 rows)
  — re-checked each guard: `221` is a single `?` on group lookup, `641`
  is `regions.len() != 1` (one value), `885`/`896` are
  `resolve_frame_to_link`'s single closure-result `None`, `942` is a
  single `if` on the `XyzEuler`-tolerance-across-a-frame-change
  condition (one boolean expression, not an OR/AND over distinct named
  operands — `utils.rs:796-802` re-read to confirm). Signature does not
  apply to any of the 5.
- **`lib.rs:1068,1072`** (moveit-metrics) — `KinematicsMetrics::group`'s
  `is_chain()` check and the `?` on `joint_model_group` are two
  *separate* construction sites (not one folded condition); this pair
  was already correctly split from the family these two rows test.
  Signature does not apply.
- **`acceleration_filter.rs:552`** (moveit-smoothing) — **matches the
  signature**, and is the user's named live candidate. Guard is
  `positions.len() != num_joints || velocities.len() != num_joints`,
  one `Error::other` folding two named operands. `reset_rejects_a_mismatched_length`
  mismatches both `positions` (len 1) and `velocities` (len 1) against
  `num_joints=2` in one fixture — no sibling test isolates either
  operand. Bit both directions: neutralizing `positions`'s clause alone
  left the test PASSING (the surviving `velocities` clause alone still
  tripped the shared error); neutralizing `velocities`'s clause alone
  also left the test PASSING. **Neither operand's own clause was ever
  individually exercised — a genuine blind site**, not a discrimination
  gap. Fixed: added `reset_rejects_a_positions_only_mismatch`/
  `reset_rejects_a_velocities_only_mismatch`, each mismatching exactly
  one array; bite-verified each new test fails when its own clause is
  disabled. Commit `3c2d72f`. **Verdict corrected below.**
- **`ruckig_filter.rs:546`** (moveit-smoothing) — **matches the
  signature**, structurally identical 3-clause OR
  (`positions.len() != num_joints || velocities.len() != num_joints ||
  accelerations.len() != num_joints`, `ruckig_filter.rs:326-329`).
  Same single-fixture-mismatches-everything shape
  (`reset_rejects_a_mismatched_length` mismatches all three arrays at
  once). Bit all three directions individually: each left the test
  PASSING with that operand's clause disabled — same blind site, three
  ways. Fixed: added `reset_rejects_a_positions_only_mismatch`/
  `reset_rejects_a_velocities_only_mismatch`/
  `reset_rejects_an_accelerations_only_mismatch`, each mismatching
  exactly one array; bite-verified each fails when its own clause is
  disabled. Commit `2829ca2`. **Verdict corrected below.**

**Count checked: 22. Count matching the signature: 3 guards / 4 rows**
(`decide.rs:183/184` sharing one guard; `acceleration_filter.rs:552`;
`ruckig_filter.rs:546`). **Outcome: `decide.rs:183/184` — genuinely
discriminating, existing tests already isolate each operand, no source
fix, verdict-only correction. `acceleration_filter.rs:552` and
`ruckig_filter.rs:546` — genuine blind sites, neither existing test
isolated any individual operand; both fixed this round with new
isolating tests, bite-verified, gated `-p moveit-smoothing`.**

## Census §9 — family membership

Applied `doc/assertion-discrimination-census.md` §9's three clauses
(mechanism / decision / subject) to all 49 rows, fresh — not copied
from any other ledger's answers, per your instruction. Every row's
disposition and reason is recorded per-crate above, in the `in-family`
column plus a prose note under the two crates with an exclusion.

**Checked: 49. Moved to `not-this-family`: 2. In-family: 47/49.**

- **`scene.rs:2150`** (moveit-scene) — clause 1 (mechanism) fails.
  `matches!(outcome, MoveObjectOutcome::Moved(_))` targets the
  success-with-effect arm of a 3-variant enum, not a "could not/did not
  produce X" tag — `NotFound`/`NoChange` would have held clause 1,
  `Moved` does not. Previously `discriminating`; that verdict answered
  a different, prior question (§9's own vocabulary section: `not-this-
  family` "is not a fourth answer to that question; it is 'the
  question does not apply'"), so the two are not in tension.
- **`constraint_sampler_manager.rs:172`** (moveit-constraints) — clause
  2 (decision) fails. The test's own fixture (empty joints, no solver,
  no subgroup solvers) never lets Steps A/B/C touch a real operand, so
  the `samplers.pop()` fallthrough this assertion inspects is an
  untouched accumulator, not a decision — the same shape as census
  §9's own `shortest_solution_is_none_on_empty_input` exclusion. The
  source's own doc comment already states the assertion would not
  change if the guard it was originally meant to test were deleted.

No row was moved *into* the family (none of the 47 remaining rows
fails any clause on re-read) and no row's within-family verdict
changed as a result of this pass — both exclusions are pure
reclassifications with no source defect to fix, matching the census's
own p1-robotmodel precedent (`not-this-family` rows there also carried
no fix). Neither exclusion required a bite: clause 1 is decided by
reading the enum's variant semantics, clause 2 by reading which
branches the fixture's arguments can reach — an isolating mutation
cannot settle "is there a decision here" when the answer is "no
decision runs at all."

**This closes p1-fixtures' half of census §9b's outstanding pair.**
139 rows were already classified before this task (51 p1-robotmodel +
58 p9-ros + 30 pilz). Adding this crate-set's 47 in-family rows: **186
of 203** classified rows are in-family (139 + 47), out of **49 + 154 =
203** rows now classified. `p3-acm` (86 → 89 rows) remains outstanding
— until it lands, the workspace-wide in-family count is bounded, not
final: **at least 186** (rows already classified), **at most 186 + 89
= 275** (if every p3-acm row turns out in-family), against a **292**
syntactic denominator (§9b, post-`tools.rs` correction). 292 itself
remains a floor, not the true syntactic population, for the reason
§9b's own closing paragraph states (the scanner's grammar misses
`.is_empty()`/`assert_eq!`-shaped assertions).

## Verdict × evidence cross-tab

A first pass of this ledger used "structural" to justify `discriminating`
in 9 rows that actually had no second producing site to discriminate
from — a single-producer argument proves `single-branch`, never
`discriminating`. Corrected then; see the per-row notes for which rows
moved and why. This round's folded-multi-operand-condition audit
(above) corrected 4 more rows the same direction, for a different
reason: a construction-site count cannot see a folded condition's
independently-coverable operands, so `single-branch`/`structural` was
also the wrong verdict for `decide.rs:183/184`,
`acceleration_filter.rs:552`, and `ruckig_filter.rs:546` — the first
pair by omission (a bite was owed and now exists), the latter two by
blind site (a bite exposed unexercised operands, since fixed).

| verdict | evidence | count |
|---|---|---:|
| single-branch | structural | 27 |
| discriminating | bite | 14 |
| single-branch | bite | 2 |
| discriminating | structural | 2 |
| discriminating | round-report | 1 |
| discriminating | sha | 2 |
| fixed | sha | 1 |
| **total** | | **49** |

## Evidence key

- **commit** — a commit sha whose diff is itself the isolating-mutation
  proof (a fix, or a prior round's bite recorded in the commit).
- **bite (this round)** — I ran a live isolating mutation just now,
  observed the assertion flip, and reverted. Working tree was
  confirmed clean (`git status --short` empty) both before and after
  every mutation in this round.
- **structural** — direct source read proves the guard is the *only*
  possible producer of the asserted outcome (single `?`/single
  combined condition/single match arm feeding the assertion), so no
  mutation could produce a second, distinguishable outcome to bite
  against. Cited with the exact line(s) read. **This evidence type can
  only support a `single-branch` verdict** — "there is exactly one
  producing site" is, by definition, an argument that there is nothing
  to discriminate. A `discriminating` verdict needs either a bite
  (two distinct producing sites, each isolated) or a cited commit/
  round-report that ran one.
- **round-report** — a prior round's claim, made and confirmed within
  this same session, re-read verbatim from the session transcript
  rather than recalled from memory, and independently agreed with.
  Cited with enough of the original text to be checked without
  re-running anything.
- **doc-recorded bite** — the source's own doc comment states a
  mutation already run (not by me, this round) and its measured
  result, in enough detail to independently verify by reading the
  cited call sites.

## moveit-scene (13 sites)

| file:line | anchor | test fn | verdict | evidence | in-family |
|---|---|---|---|---|---|
| scene.rs:2150 | matches! | `diff_scene_records_a_move_only_change_for_an_existing_object` | discriminating | round-report — this same session, earlier round: ran 3 bites on `PlanningScene::move_object` (`scene.rs:976`), reverted after each. (1) reachability: forced outcome to `NotFound` → test fails at the `matches!`. (2) discrimination: forced outcome to sibling `NoChange` → test fails at the same `matches!`. (3) payload: kept `Moved` but swapped the notification's `Action` to `CREATE` → the wildcarded `Moved(_)` still passes, but the test's own second assertion (`diff.get("box").unwrap() == Action::MOVE_SHAPE`) catches it. You re-read and confirmed this in-session ("Verified `scene.rs:2150` and agreed — `MoveObjectOutcome`'s three variants map 1:1 to the three guards, bites 1 and 2 both fail... `discriminating`, no fix"). No commit — no fix was needed, gate was `-p moveit-scene` clean at the time. | **not-this-family** — see below |
| scene.rs:2326 | bare `.is_none()` | (unnamed, `WorldDiff::get`-adjacent) | discriminating | bite (this round) — `WorldDiff::get`'s single guard (`world_diff.rs:104-106`) is the sole producer; see `world_diff.rs:315` bite below, same guard shape | yes |
| scene.rs:2524 | bare `.is_none()` | `decouple_parent_then_mutating_the_former_parent_is_not_observed` | single-branch | structural — `decouple_parent` (`scene.rs:2003-2021`) has exactly one `self.parent = None;` site (verified by `rg 'self\.parent = ' crates/moveit-scene/src/scene.rs`). Corrected from `discriminating`: "exactly one producing site" proves there is nothing to discriminate from, not that there is | yes |
| scene.rs:2556 | bare `.is_none()` | `decouple_parent_materializes_the_inherited_transforms_map` | single-branch | structural — same single `self.parent = None;` site as 2524. Corrected from `discriminating`, same reason | yes |
| scene.rs:2591 | bare `.is_none()` | `decouple_parent_then_the_childs_inherited_attached_body_frame_still_resolves` | single-branch | structural — same single `self.parent = None;` site. Corrected from `discriminating`, same reason | yes |
| scene.rs:2616 | bare `.is_none()` | `decouple_parent_then_the_childs_inherited_world_object_still_resolves` | single-branch | structural — same single `self.parent = None;` site. Corrected from `discriminating`, same reason | yes |
| scene.rs:2698 | bare `.is_none()` | `clear_diffs_resets_a_diverged_child_to_a_fresh_diff_against_the_parent` | discriminating | bite (this round) — removed `self.acm = Layered::Inherited;` from `clear_diffs`, assertion flipped, reverted | yes |
| scene.rs:2729 | bare `.is_err()` | `frame_transform_resolves_the_model_frame_and_a_link_name` | single-branch | structural — re-read `frame_transform` (`scene.rs:1312-1332`) line by line: it has two possible `Err`-producing statements, `posed.global_link_transform(link_name)?` (reached only when `frame_id` resolves to an attached body, `scene.rs:1323`) and the final fallthrough `self.transforms().transform(frame_id).copied()` (`scene.rs:1331`, reached when no tier matched at all). `"world"` resolves in no tier and is not an attached-body name, so only the fallthrough fires. No test in this file's `.is_err()`/`matches!` family exercises the attached-body error path (`rg 'frame_transform\(' crates/moveit-scene/` across `scene.rs` and `frame_transform_parity.rs`), so within the population of sites actually asserted, there is only one reachable producer. Corrected from `discriminating`: the earlier evidence ("isolates... from... tested moments earlier") named tiers that succeed, not a second `Err`-producing sibling — a real second producer exists in the function but nothing in this family tests it | yes |
| scene.rs:2814 | bare `.is_err()` | `frame_transform_reports_a_name_resolving_in_no_tier` | single-branch | structural — same fallthrough-only reasoning as 2729; `"nothing"` is not an attached-body name either. Corrected from `discriminating`, same reason | yes |
| scene.rs:2873 | bare `.is_err()` | `frame_transform_tier_six_absent_name_is_still_unknown` | single-branch | structural — same fallthrough-only reasoning; `"no_such_frame"` is not an attached-body name. Corrected from `discriminating`, same reason | yes |
| scene.rs:2882 | bare `.is_err()` | `frame_transform_tier_six_empty_name_is_unknown` | single-branch | structural — same fallthrough-only reasoning; `""` is not an attached-body name. Corrected from `discriminating`, same reason | yes |
| world_diff.rs:315 | bare `.is_none()` | `set_with_uninitialized_action_erases_the_entry` | discriminating | bite (this round) — neutralized `WorldDiff::set`'s UNINITIALIZED branch (`if false && ...`), assertion flipped, reverted | yes |
| frame_transform_parity.rs:254 | bare `.is_err()` | `panda_frame_transform_matches_the_oracle` | single-branch | structural — same `frame_transform` fallthrough-only reasoning as 2729/2814/2873/2882; `query.name == "nothing"` is not an attached-body name in this fixture (`build_scene`'s attach loop only populates from `request.attached_bodies`). Corrected from `discriminating`, same reason | yes |

**`scene.rs:2150` moved to `not-this-family` (clause 1, mechanism).**
Re-read `World::move_object` (`crates/moveit-collision/src/world.rs:586-595`):
`self.objects.get(id)` miss → `NotFound`; `eigen_is_approx(transform,
identity)` → `NoChange`; otherwise → `Moved(notification)`.
`MoveObjectOutcome::NotFound`/`NoChange` are genuine "could not/did not
produce X" tags (clause 1's own wording fits them exactly) — but this
row's assertion, `matches!(outcome, MoveObjectOutcome::Moved(_))`,
targets the *third* arm: the one produced when the operation
**succeeded** with a real effect. Clause 1 is explicit that this
family is "not a plain, informative value returned on a success
path... because such a value... already names in full which fact
about the world produced it" — `Moved(_)` is exactly that: the
wildcard discards only the notification payload (checked separately by
this test's own second assertion), not which of the three decisions
fired. Checking which arm of an already-fully-discriminating 3-variant
enum landed is a dispatch question — the same shape census §9 clause 1
excludes for `matches!(joint.kind(), JointKind::Fixed)`, generalized
from a getter to a return-value dispatch. The prior `discriminating`
verdict is not wrong as an answer to the old question (the bites are
real, and a bug swapping which guard fires would be caught) — but §9
is a different, prior question, and `Moved(_)` fails it. No fix owed:
`not-this-family` is a classification, not a defect.

## moveit-octomap (10 sites)

| file:line | anchor | test fn | verdict | evidence | in-family |
|---|---|---|---|---|---|
| node.rs:143 | bare `.is_none()` | `fresh_node_has_no_children_and_zero_log_odds` | single-branch | bite (this round) — made `Node::new()` eagerly allocate the children array (eliminating the "no array" cause); assertion still passed, proving this test cannot distinguish "no array" from "array present, slot empty" — matches the function's own doc comment (`node.rs:66-67`, "`None` covers both...", a deliberate 2-cause union mirroring upstream's `nodeChildExists`+`getNodeChild`) | yes — `child()`'s `self.children.as_ref()?[idx]` genuinely executes on a fresh node (§9's own `nn.rs:227`/`self.root.as_ref()?` precedent: a written guard that runs even on an "empty" fixture is still a decision) |
| node.rs:153 | bare `.is_none()` | `create_child_populates_exactly_one_of_eight_slots` | single-branch | structural — fixture calls `create_child(3)` first, which allocates the array (`get_or_insert_with`, `node.rs:78-80`); only the "array present, slot `i` empty" cause is reachable, the "no array" cause is structurally excluded by this fixture | yes |
| tree.rs:1729 | bare `.is_none()` | `ray_with_end_outside_tree_bounds_returns_none` | discriminating | bite (this round, §3a mirror) — neutralized the `end`-guard in `compute_ray_keys` (`.unwrap_or_else(Self::root_key)`), this assertion flipped, reverted | yes |
| tree.rs:1733 | bare `.is_none()` | `ray_with_end_outside_tree_bounds_returns_none` | discriminating | bite (this round) — same test, second assertion (`origin` far-negative case), covered by the same end-guard mutation above | yes |
| tree.rs:1744 | bare `.is_none()` | `ray_with_origin_outside_tree_bounds_returns_none` | discriminating | bite (this round, §3a mirror) — neutralized the `origin`-guard independently, this assertion flipped while the end-guard mutation left it green; reverted | yes |
| tree.rs:1762 | bare `.is_none()` | `unmapped_coordinate_has_no_occupancy` | discriminating | bite (this round) — `search` (`tree.rs:876-890`) has two `None` producers, `self.root.as_deref()?` (empty tree) and the loop's `has_children()`-gated arm (ambiguous partial structure); these are real siblings, so ran both directions. Bite 1: gave the root-absent guard a fallback empty node (temp `Node::EMPTY` const + `.unwrap_or(&Node::EMPTY)` in `search`) — this test FAILED (log_odds_at/is_occupied became `Some` instead of `None`), tree.rs:1794 stayed green. Bite 2: neutralized the inner `has_children()`-gated arm to always `Some(cur)` — this test stayed green (unaffected, root already absent so the loop is never entered), tree.rs:1794 FAILED. Both mutations reverted; `git status --short`/`git diff --stat` empty after | yes |
| tree.rs:1763 | bare `.is_none()` | `unmapped_coordinate_has_no_occupancy` | discriminating | bite (this round) — same test, same bite pair as tree.rs:1762 above (`is_occupied` composes `log_odds_at`, same root-absent cause) | yes |
| tree.rs:1794 | bare `.is_none()` | `insert_ray_cut_short_by_max_range_records_only_a_miss` | discriminating | bite (this round) — see tree.rs:1762's bite pair; this test flips under bite 2 (inner `has_children()` guard neutralized) and stays green under bite 1 (root-absent guard neutralized), the mirror image of 1762/1763, confirming it exercises the second, distinct `None` producer | yes |
| tree.rs:1930 | bare `.is_none()` | `leaves_in_bbx_returns_none_for_an_out_of_range_max` | discriminating | commit — doc comment (`tree.rs:1924-1926`) and prior isolating mutation recorded at `0d10a11`; `git show 0d10a11 --stat` confirms it touches this test/guard pair | yes |
| tree.rs:1944 | bare `.is_none()` | `leaves_in_bbx_returns_none_for_an_out_of_range_min` | discriminating | commit — doc comment (`tree.rs:1936-1940`, explicitly states "before this test existed the `min` guard had no coverage at all") and prior isolating mutation recorded at `567342f`; `git show 567342f --stat` confirms | yes |

All 10 octomap rows hold every clause: mechanism is `Option::None` on a
lookup/traversal (clause 1), each guard is a written comparison run
against real coordinates/indices, never a vacuous/empty-input skip
(clause 2 — 8 of the 10 are bite-confirmed, which is itself the clause-2
operational test passing), and each belongs to the tree/node method the
test names as its subject (clause 3). No exclusions.

## moveit-state (10 sites)

| file:line | anchor | test fn | verdict | evidence | in-family |
|---|---|---|---|---|---|
| invariants.rs:585 | bare `.is_err()` | `unknown_name_is_an_error_not_a_panic_for_every_new_accessor` | single-branch | structural — `variable_velocity` (`state.rs:369-`) is a single-`?`-site accessor; one name-lookup guard, no second producer | yes |
| invariants.rs:586 | bare `.is_err()` | same fn | single-branch | structural — `variable_acceleration`, same single-`?`-site shape | yes |
| invariants.rs:587 | bare `.is_err()` | same fn | single-branch | structural — `variable_effort`, same shape | yes |
| invariants.rs:588 | bare `.is_err()` | same fn | single-branch | structural — `set_variable_velocity`, same shape | yes |
| invariants.rs:589 | bare `.is_err()` | same fn | single-branch | structural — `set_variable_acceleration`, same shape | yes |
| invariants.rs:594 | bare `.is_err()` | same fn | single-branch | structural — `set_variable_effort`, same shape | yes |
| invariants.rs:595 | bare `.is_err()` | same fn | single-branch | structural — `joint_velocity`, same shape | yes |
| invariants.rs:596 | bare `.is_err()` | same fn | single-branch | structural — `joint_acceleration`, same shape | yes |
| invariants.rs:597 | bare `.is_err()` | same fn | single-branch | structural — `joint_effort`, same shape | yes |
| jacobian.rs:211 | matches! | `an_unknown_group_name_is_unknown_name_not_not_a_chain` | discriminating | bite (this round) — I could not locate a citable p1-robotmodel bite for this site: `git log --all` on `jacobian.rs`/`state.rs` shows no commit past the original port (`9230ad3`), no `doc/` file mentions it (including the newly-merged `assertion-discrimination-ledger-p1-robotmodel.md`, whose 7 crates are `moveit-trajectory`/`-planners-chomp`/`-planners-sbp`/`-planners-stomp`/`-planning`/`-sampling`/`-kinematics` — not `moveit-state`), and `Posed::jacobian` (`state.rs:1121-1146`) carries no doc-recorded mutation. Ran it myself instead: `Posed::jacobian` has two real sibling `Err` sites, `model.joint_model_group(group)?` (`state.rs:1123`) and the `is_chain()` guard's `Error::other(...)` (`state.rs:1125-1129`). Bite 1 (reachability): made the group-lookup guard fall back to `"panda_arm"` on failure instead of propagating — test FAILED, `unwrap_err()` panicked on an `Ok` (proves the guard is necessary for any error at all). Bite 2 (discrimination/sibling-swap): kept the guard failing but swapped its `Err` to `Error::other(...)` instead of propagating `UnknownName` — test FAILED at the `matches!`, proving the assertion discriminates the sibling. Both reverted; `git status --short`/`git diff --stat` empty after | yes |

All 10 hold every clause: `Error::UnknownName`/`Error::Other` are
canonical "could not produce X" signals (clause 1); each accessor's `?`
on a caller-supplied name is a written guard tested with a real unknown
name, never an empty/vacuous input (clause 2); each belongs to the
accessor or `Posed::jacobian` the test names (clause 3). No exclusions.

Each of the 9 `invariants.rs` sites is individually single-branch (its
own accessor has exactly one guard), but the 9 together are what
discriminates the accessor family from each other — this is the same
"one row = one call site" shape as round 8's matrix.rs table, not a
finding.

## moveit-constraints (9 sites)

| file:line | anchor | test fn | verdict | evidence | in-family |
|---|---|---|---|---|---|
| constraint_sampler_manager.rs:172 | bare `.is_none()` | `no_constraints_and_no_solver_returns_none` | single-branch | structural — re-traced `select_default_sampler`/`_inner`'s full Step A/B/C/D control flow (`constraint_sampler_manager.rs:141-`); no test anywhere in the file exercises the *other* None-producer (`select_default_sampler:149-151`'s unknown-group-name early return — `rg 'select_default_sampler\('` on the test file shows every call uses a valid `"panda_arm"`/`"panda_arm_hand"` group name), so only the Step-D fallthrough is reachable and tested | **not-this-family** — see below |
| decide.rs:183 | bare `.is_err()` | `new_rejects_negative_tolerance` | discriminating | bite (this round) — corrected from `single-branch`/`structural`: `JointConstraint::new`'s guard (`joint.rs:120`, `tolerance_above < 0.0 \|\| tolerance_below < 0.0`) is one `Err::construct` site but folds two named operands, which a construction-site count cannot see through. Neutralizing `tolerance_above`'s clause alone (`if false && tolerance_above < 0.0 \|\| tolerance_below < 0.0`) failed this assertion exactly. The test's own fixture (`tolerance_above=-0.1, tolerance_below=0.1`) already isolates this operand. `joint.rs` reverted clean after | yes |
| decide.rs:184 | bare `.is_err()` | `new_rejects_negative_tolerance` | discriminating | bite (this round) — mirror of 183: neutralizing `tolerance_below`'s clause alone (`if tolerance_above < 0.0 \|\| false && tolerance_below < 0.0`) failed this assertion exactly. The test's own fixture (`tolerance_above=0.1, tolerance_below=-0.1`) already isolates this operand. `joint.rs` reverted clean after | yes |
| decide.rs:210 | matches! | `new_rejects_unknown_joint` | fixed | commit `83e3c1c` — this session's own prior fix, asserts the specific `Error::UnknownName{kind:"joint",..}` variant/field rather than bare `.is_err()` | yes |
| utils_parity.rs:221 | matches! | `unknown_group_is_error` | single-branch | structural — `construct_goal_joint_constraints`'s only reachable guard for an unknown group name is `model.joint_model_group(group_name)?` (`utils.rs:234`); the loop body's two further `?` sites are unreached when this one fails first | yes |
| utils_parity.rs:641 | matches! | `multi_region_constraint_is_error` | single-branch | structural — `update_position_constraint` has exactly one `Error::Other`-producing site (`utils.rs:605-609`, converting `with_updated_position`'s `None` for a >1-region constraint) | yes |
| utils_parity.rs:885 | bare `.is_none()` | `an_unrecognised_frame_is_none` | single-branch | structural — `resolve_position_constraint_frame` (`utils.rs:727-747`) has one `None`-producing path, `resolve_frame_to_link`'s own single `None` cause (`utils.rs:641`, the closure result — the two other tiers only ever return `Some` or fall through, never `None`) | yes |
| utils_parity.rs:896 | bare `.is_none()` | same fn, second assertion | single-branch | structural — `resolve_orientation_constraint_frame` (`utils.rs:777-805`) shares the identical single `resolve_frame_to_link` None cause | yes |
| utils_parity.rs:942 | matches! | `xyz_euler_tolerance_across_a_real_frame_change_is_an_error` | single-branch | structural — `resolve_orientation_constraint_frame`'s only `Error::Other`-producing site is the `XyzEuler`-tolerance-across-a-frame-change guard (`utils.rs:796-802`), a single `if` | yes |

**`constraint_sampler_manager.rs:172` moved to `not-this-family` (clause
2, decision).** Re-read `select_default_sampler_inner`
(`constraint_sampler_manager.rs:202-312`) and the test's own fixture
(`tests/constraint_sampler_manager.rs:169-177`, `no_constraints_and_no_
solver_returns_none`): `joints=&[]`, `solver=None`,
`subgroup_solvers=vec![]`. Steps A (`if !joints.is_empty()`), B (`if
let Some(solver) = solver`), and C (`if !subgroup_solvers.is_empty()`)
all guard on empty/`None` inputs and never touch a real element — the
final `Ok(samplers.pop())` pops a `Vec` that was declared empty
(`let mut samplers = Vec::new()`) and never pushed to, because no
branch of A/B/C's body ever ran. This is exactly census §9 clause 2's
excluded shape ("an accumulator whose initial value is simply never
touched because there was nothing to iterate" —
`shortest_solution_is_none_on_empty_input`), not a `nn.rs:227`-style
guard that still runs on an empty structure. The test file's own doc
comment (`tests/constraint_sampler_manager.rs:105-113`) already states
this explicitly: "Step D/E's fallthrough produces `Ok(None)` for *any*
group, real or not... a bare `result.is_none()` here would still pass
even if the unresolvable-group early return were deleted." No engineer
mistake at Step D/E could make this assertion fail differently — there
is no decision here to get wrong. No fix owed: this is a
classification, not a defect, and the source's own comment already
documents the limitation.

## moveit-metrics (3 sites)

| file:line | anchor | test fn | verdict | evidence | in-family |
|---|---|---|---|---|---|
| lib.rs:1068 | matches! | `unknown_group_is_unknown_name` | single-branch | structural + doc-recorded — doc comment (`lib.rs:1053-1059`) states `manipulability` and `manipulability_index` "share `KinematicsMetrics::group`'s `self.model.joint_model_group(group)?` call verbatim"; single guard | yes |
| lib.rs:1072 | matches! | same fn, second assertion | single-branch | structural + doc-recorded — same shared single guard, `manipulability_index` caller | yes |
| lib.rs:1139 | matches! | `manipulability_ellipsoid_rejects_the_same_bad_groups` | single-branch | doc-recorded bite — doc comment (`lib.rs:1102-1123`) records a mutation already run and its result: replacing `self.group(...)?` with `let _ = self.group(...);` inside `manipulability_ellipsoid` "leaves the variant-level assertions below unchanged" because `state.jacobian`'s own re-check produces the byte-identical `UnknownName` either way; this line's assertion does *not* pin `self.group`'s own call site (only the message-level check at 1132-1138, outside this family, does that) | yes — the recorded mutation shows a real, engineer-could-implement-wrong decision (which call site produces the error) still belonging to `manipulability_ellipsoid`; the ambiguity is *which* subject-side decision fires, not whether one does |

All 3 use a real unknown group name (never an empty/vacuous group
argument), so clause 2 holds on real data, not a skipped comparison.

## moveit-smoothing (2 sites)

| file:line | anchor | test fn | verdict | evidence | in-family |
|---|---|---|---|---|---|
| acceleration_filter.rs:552 | matches! | `reset_rejects_a_mismatched_length` | discriminating | bite (this round) — corrected from `single-branch`/`structural`: `reset`'s guard (`acceleration_filter.rs:302`, `positions.len() != num_joints \|\| velocities.len() != num_joints`) is one `Error::Other` site folding two named operands. This test's fixture mismatches both `positions` and `velocities` at once, so neither existing test isolated either clause — bit both directions (`false && positions...`, then `positions... \|\| false && velocities...`), both left this test PASSING: a genuine blind site, not just an undercounted branch. Fixed by adding `reset_rejects_a_positions_only_mismatch`/`reset_rejects_a_velocities_only_mismatch` (each mismatching exactly one array), bite-verified to fail when their own clause is disabled. Commit `3c2d72f`, gated `-p moveit-smoothing` | yes |
| ruckig_filter.rs:546 | matches! | `reset_rejects_a_mismatched_length` | discriminating | bite (this round) — corrected from `single-branch`/`structural`: same shape as acceleration_filter.rs:552, 3-clause OR (`ruckig_filter.rs:326-329`) folding `positions`/`velocities`/`accelerations`. This test's fixture mismatches all three at once. Bit all three directions individually — each left this test PASSING, the same blind site three ways. Fixed by adding `reset_rejects_a_positions_only_mismatch`/`reset_rejects_a_velocities_only_mismatch`/`reset_rejects_an_accelerations_only_mismatch`, bite-verified each fails when its own clause is disabled. Commit `2829ca2`, gated `-p moveit-smoothing` | yes |

Both bite-confirmed against real mismatched-length data (never an
empty-input skip), so clause 2 holds on real operands.

## moveit-srdf (2 sites)

| file:line | anchor | test fn | verdict | evidence | in-family |
|---|---|---|---|---|---|
| boundaries.rs:106 | matches! | `a_virtual_joint_missing_any_required_attribute_is_dropped` | discriminating | structural — I could not locate a citable p9-ros `Parser::required` bite for this site (`git log --all` on `boundaries.rs`/`parse.rs` shows only `0ca2af8` (initial port) and `b819ec1` (a *different* test, `malformed_xml_is_an_error`); no `doc/` file mentions it). Traced the real mechanism instead: `load_virtual_joints` (`parse.rs:107-127`) calls `Parser::required` (`parse.rs:82-98`) four times in sequence, once per attribute, each call passing its own `&'static str` literal (`"name"`/`"child_link"`/`"parent_frame"`/`"type"`) that `required` stores verbatim into `Diagnostic::MissingAttribute{attribute,..}`. These are four distinct call sites with four distinct compile-time literals, not one shared constant — genuine siblings, structurally guaranteed distinguishable by the type system, no mutation needed to prove it | yes |
| boundaries.rs:422 | matches! | `an_unparsable_joint_value_drops_the_joint_instead_of_storing_zero` | discriminating | structural — same search as boundaries.rs:106 above, same absence of a citable prior bite. Traced the real mechanism: `load_group_states` (`parse.rs:249-300`) has exactly one `MalformedValue`-producing site for a joint value (`parse.rs:285-290`, hardcoded `attribute: "value"`) — single-branch *within this function*. But `rg 'Diagnostic::MalformedValue' crates/moveit-srdf/src/parse.rs` finds two more sibling sites elsewhere in the same parser, `parse.rs:364` (`attribute: "center"`) and `parse.rs:373` (`attribute: "radius"`), for the sphere-collision parser. The `matches!`'s `attribute == "value"` check is what distinguishes this site's diagnostic from those two real siblings at the population level, even though none of the three are reachable from each other's fixtures | yes |

Both fixtures supply real malformed/missing XML (a specific attribute
actually absent, a specific value actually unparsable), not an empty
document that would skip the parser's comparisons — clause 2 holds on
real operands; `Diagnostic::MissingAttribute`/`MalformedValue` are
canonical "could not parse X" tags (clause 1); both belong to the
`load_virtual_joints`/`load_group_states` parser routines the tests
name (clause 3).

## Coarse-assertion sweep beyond the old grammar (`count-coarse-assertions.py`)

`tools/ci/count-coarse-assertions.py` (`6a14a89`, later fixed for the
assertion-helper mechanism/site defect at `ccac7ea`) enumerates
assertions in a grammar wider than the `matches!`/`.is_err()`/
`.is_none()` one the 49 rows above came from: `is_some`, `is_empty`,
`contains_member`, `contains_msg`, `eq_none` (`assert_eq!(x, None)`),
`eq_err` (`assert_eq!(x, Err(..))`). Run against this round's 7 crates
(`python3 tools/ci/count-coarse-assertions.py crates/moveit-octomap
crates/moveit-scene crates/moveit-constraints crates/moveit-srdf
crates/moveit-state crates/moveit-metrics crates/moveit-smoothing`,
after `git merge main` twice — once for `6a14a89`, again for
`ccac7ea`), excluding every line tagged `matches`/`is_err`/`is_none`
(old grammar, already in the 49 rows) and every line tagged
`contains_msg` or `via:<fn>` where `<fn>` is a `contains_msg`-shaped
helper (this round's deconfliction: rendered-error-message sites in
`crates/` belong to p1-robotmodel).

**Both counts I was handed this round needed a correction, found by
re-measuring, not by reconciling against the number I was given:**

- **72 → 71.** `tree.rs:1781`'s `eq_none` tag is a scanner false
  positive: `assert!(tree.insert_ray(origin, end, None, false))` is a
  plain boolean assertion on `insert_ray`'s return value; the `None`
  is `insert_ray`'s `max_range: Option<f64>` argument, not an
  equality-with-`None` check. Confirmed by reading the line (`grep -vE
  'assert_eq!'` over every `eq_none`/`eq_err` hit found exactly one
  line whose macro is plain `assert!`, this one) — no other false
  positive of this shape exists in the 72. Real syntactic population:
  **71** (octomap 29, scene 25, constraints 8, srdf 6, state 3,
  metrics 0, smoothing 0).
- **80 → 71, not 80.** Mid-task you reported 80 after the mechanism/
  site fix (`ccac7ea`) surfaced 8 new sites the old tool counted as
  zero. All 8 are `via:assert_err_mentions` call sites in
  `crates/moveit-constraints/tests/decide.rs` — every one of them.
  `assert_err_mentions` (`decide.rs:78-86`) does exactly one thing:
  `result.expect_err(...).to_string()` then `assert!(rendered.contains
  (needle), ...)` — a rendered-error-message check, `contains_msg` in
  substance, tagged `contains_msg` at its own definition line
  (`decide.rs:83:contains_msg:helper_body:...`). The fix that surfaced
  these 8 changed *how the tool reports* a message-content helper's
  call sites (mechanism vs. site), not *what kind of assertion* they
  are — they were already excluded from my 72 by the same
  `contains_msg`-belongs-to-p1-robotmodel deconfliction the original
  task stated; the mechanism/site fix just made them visible enough to
  need excluding a second time, under a different tag. Verified: `grep
  -oE ':via:[A-Za-z_]+:' <scanner output> | sort -u` over all 7 crates
  returns exactly one function name, `assert_err_mentions`, with
  exactly 8 call sites, all in `decide.rs`, all `contains_msg`-shaped
  by reading the helper body. 71 stands.

**Filter, reproducibly:** keep only lines whose kind-tag set is drawn
purely from `{contains_member, is_empty, eq_none, eq_err, is_some}` —
i.e. exclude any line tagged `matches`, `is_err`, `is_none`,
`contains_msg`, any line whose kind starts `via:`, and any line scoped
`helper_body`. Applied via `awk`/`grep` over the scanner's raw output,
cross-checked by hand-counting the per-kind and per-crate breakdown
against the number I was given — both matched (20 `contains_member`,
18 `is_empty`, 13 `eq_none`, 13 `eq_err`, 8 `is_some`; per-crate 29/25/
8/6/3/0/0) once the `tree.rs:1781` false positive was pulled out.

### Per-crate verdicts, census §9 three clauses (mechanism / decision / subject)

**In-family denominator: 41 of 71.** Two blind sites found and fixed
(both moveit-octomap, both `eq_none`/`is_some`). No blind site found in
scene, constraints, srdf, or state — every in-family row there was
already discriminating, mostly because the test authors had already
paired the coarse assertion with a same-test companion (a preceding
non-vacuous state check, a same-test contrasting sibling case, or a
more specific diagnostic/variant assertion two lines away) that rules
out the sibling cause.

#### moveit-octomap (29 real sites, 23 in-family, 2 blind — both fixed)

| file:line | kind | verdict | in-family | note |
|---|---|---|---|---|
| tree.rs:1781 | eq_none | **scanner false positive** | n/a | `assert!(insert_ray(..., None, ...))` — `None` is an argument, not a comparison |
| node.rs:151 | is_some | not-this-family | no | clause 2 — `create_child(3)` unconditionally populates slot 3 two lines above; no decision between cause and observation |
| tree.rs:1805 | contains_member | not-this-family | no | clause 1 — `occupied.contains(&hit_key)` is membership in `compute_update`'s actual computed classification, not an absence signal |
| tree.rs:1806 | contains_member | not-this-family | no | clause 1, same reasoning |
| tree.rs:1807 | is_empty | not-this-family | no | clause 2 — general "ray tracing produced *some* free cells" sanity check, no crafted decision boundary |
| tree.rs:1824/1837/1852/1862/1868/1878/1879 | eq_none ×7 | **in-family, was partly blind** | yes | `coord_to_key_checked_axis`'s folded 3-operand guard (`is_finite() && >= min && < max`, one `return None` site). Bite-confirmed (3 live mutations, each reverted): `>= min` and `< max` are each independently caught by a dedicated boundary test (−32768.5/1e300 for min, 32768.0/1e300 for max) — genuinely discriminating. `is_finite()` is **dead**: neutralizing it alone left all 9 tests green, because IEEE 754 comparisons with NaN are always false and ±infinity always falls outside the finite `[min, max)` range, so the other two clauses already reject every non-finite input on their own. **Fixed** (`3e0430d`): removed the dead conjunct — no test could ever have closed this since no input makes it decisive, so the fix is deletion, not a new test |
| tree.rs:1980/1990/1996/2002/2010/2020/2030/2049/2062/2223/2256 | eq_err ×11 | in-family, discriminating | yes | `DecodeError::UnexpectedEof`/`TreeAlreadyPopulated`/`MaxDepthExceeded` are each reached from 2 production call sites (one per of `read_binary_data`/`read_data`, or one per `read_binary_node`/`read_data_node`'s shared `Cursor::read_u8`/`read_f32_le`). Checked the `DecodeError` catch-all risk you flagged directly: every one of the 11 tests' own in-source comments (`tree.rs:2040-2047`, `2057-2060`) trace the exact single reachable call site for that test's crafted byte length, ruling out every sibling explicitly (e.g. "`TreeAlreadyPopulated` is excluded (fresh tree) and `MaxDepthExceeded` is unreachable (3 bytes cannot recurse to depth 16)"). None is a bare catch-all — each traces to one call site |
| decode_parity.rs:200/237 | eq_err ×2 | in-family, discriminating | yes | same `UnexpectedEof`-on-empty-input reasoning as `tree.rs:1996/2002`, run across the oracle fixture corpus |
| decode_parity.rs:195/232 | is_empty ×2 | not-this-family | no | clause 3 — subject is the *oracle's* fixture data (`expected_nodes.is_empty()`), a fixture self-consistency check, not the code under test |
| octomap_parity.rs:216 | is_some | **in-family, was blind** | yes | `log_odds_at`'s `None` has 2 causes (`coord_to_key_checked` out-of-range vs. `search` finds nothing); every fixture query point and every existing unit test only ever reached the second. Bite: replacing the out-of-bounds guard with a silent clamp to the tree center left all 67 tests green. **Fixed** (`08181da`): new test populates the tree center then queries a genuinely out-of-bounds point, so the same bite now fails |
| octomap_parity.rs:246 | is_some | in-family, not blind | yes | shares `log_odds_at`, now covered by the same fix; the out-of-bounds cause is structurally unreachable via `OccupancyByKey` (`key_to_coord` on a `u16` key is always in-range by construction), so only the `search`-null cause applies here, and that's discriminated per fixture row (mapped vs. unmapped rows both present) |
| octomap_parity.rs:277 | is_some | in-family, discriminating | yes | `compute_ray_keys`'s two `None` causes (origin vs. end out-of-bounds) are each separately exercised by dedicated fixture rows (`[0,0,0]→[1e9,0,0]` and `[1e9,0,0]→[0,0,0]`), matching the crate's own dedicated unit tests `ray_with_{origin,end}_outside_tree_bounds_returns_none` |

#### moveit-scene (25 real sites, 11 in-family, 0 blind)

All 10 `contains_member` hits (`scene.rs:2119/2120`, `world_diff.rs:
158/159/198/225/226/247/248/287/288/289`) are `Action` bitflag checks —
not-this-family, clause 1: the bitflag set is the diff algorithm's own
informative computed classification ("which actions occurred"), not a
stand-in for an operation's inability to do something.

`world_diff.rs:330/331` (`a_fresh_diff_is_empty`) and `decide.rs`-style
fresh-constructor checks — not-this-family, clause 2: `WorldDiff::new()`
is `Self::default()`, no decision to get wrong.

The remaining 11 (`scene.rs:2122/2137/2318/2337/2686/2692/2703/3354/
3378`, `world_diff.rs:297/323`) are in-family and every one is already
discriminating — each pairs its coarse assertion with a same-test
non-vacuous setup (a mutation immediately before the check that proves
the collection/link *was* populated, ruling out the "never touched"
sibling), e.g. `scene.rs:2692`'s doc comment explicitly names the bug
class it exists to catch ("`clear_diffs` resetting `attached_bodies`/
`acm` to empty ... would be indistinguishable from correctly
re-inheriting the parent's ... state"), and `scene.rs:3354`/`3378`
each sit next to a sibling test proving the same collection is
non-empty under different input.

#### moveit-constraints (8 real sites, 3 in-family, 0 blind)

| file:line | kind | verdict | in-family | note |
|---|---|---|---|---|
| decide.rs:1161 | eq_none | in-family, discriminating | yes | `max_view_angle()`/`max_range_angle()` are `mimic()`-shaped getters; decision lives in `VisibilityConstraint::new`'s `normalize_angle_criterion` call, one call site per field |
| decide.rs:1166 | eq_none | in-family, discriminating | yes | same shape, own call site |
| decide.rs:1257 | is_empty | not-this-family | no | clause 2 — `KinematicConstraintSet::new()` is `Self::default()` |
| sampler.rs:194/200 | contains_member ×2 | not-this-family | no | clause 1 — `(min..=max).contains(&v)` validates numeric correctness of a sampled value, not an absence signal |
| utils_parity.rs:580/602 | is_empty ×2 | not-this-family | no | clause 2 — `update_{orientation,position}_constraint`'s search loop runs over an empty `KinematicConstraintSet::new()`; the loop body's comparison never executes, matching census §9's `shortest_solution_is_none_on_empty_input` exclusion exactly |
| utils_parity.rs:698 | is_empty | in-family, discriminating | yes | `merge_constraints` drops a genuinely non-overlapping pair (`low > high`); sibling test `overlapping_windows_merge_to_the_intersection` proves the merge logic isn't vacuously always-empty |

#### moveit-srdf (6 real sites, 4 in-family, 0 blind)

| file:line | kind | verdict | in-family | note |
|---|---|---|---|---|
| boundaries.rs:71 | eq_none | in-family, discriminating | yes | sibling test (`an_empty_robot_element_is_a_valid_empty_model`) proves `model.name()` returns `Some` when present |
| boundaries.rs:141 | eq_none | in-family, discriminating | yes | same test's very next assertion contrasts absent (`None`) against present-but-empty (`Some(String::new())`) for `parent_group` |
| boundaries.rs:317 | contains_member | not-this-family | no | clause 1 — `Diagnostic::UnknownGroup { element, name, group }` fully names which fact produced it, same as `DecodeError`/`Action` |
| boundaries.rs:418 | is_empty | in-family, discriminating | yes | `load_group_states` has 3 structurally distinct drop guards (missing name / missing value / unparsable value); the companion assertion (`matches!(diagnostics, [MalformedValue { attribute: "value", .. }])`) pins this test to the third, ruling out the other two |
| boundaries.rs:444 | is_empty | in-family, discriminating | yes | fixture is 2 joints, each isolating a different one of the other 2 drop guards (missing name, missing value); breaking either guard would insert its joint and flip this assertion |
| fixtures.rs:209 | is_empty | not-this-family | no | clause 2 — PANDA.srdf has zero `<joint_property>` elements; the accumulator is never touched, matching the `shortest_solution` exclusion |

#### moveit-state (3 real sites, 0 in-family)

All 3 (`invariants.rs:100/140/368`) are `(-PI..=PI).contains(&wrapped)`
— not-this-family, clause 1: validating that an angle-wrapping
computation's numeric output landed in range, not an absence signal.

#### moveit-metrics / moveit-smoothing

0 sites each, matching the count you were given.

## Sites needing a fix this round

Two, both found by this round's folded-multi-operand-condition audit
(see above), both fixed:

- `acceleration_filter.rs:552`/`reset` (moveit-smoothing) — neither
  the `positions` nor the `velocities` clause of the guard's OR
  condition was individually exercised by any existing test. Fixed:
  `reset_rejects_a_positions_only_mismatch`/
  `reset_rejects_a_velocities_only_mismatch` added, commit `3c2d72f`.
- `ruckig_filter.rs:546`/`reset` (moveit-smoothing) — none of the
  `positions`/`velocities`/`accelerations` clauses of the guard's
  3-clause OR condition was individually exercised. Fixed:
  `reset_rejects_a_positions_only_mismatch`/
  `reset_rejects_a_velocities_only_mismatch`/
  `reset_rejects_an_accelerations_only_mismatch` added, commit
  `2829ca2`.

Prior to this round's folded-condition audit, every site in this
round's 7 crates was already discriminating, provably single-branch by
direct source read, or previously fixed — no blind/never-covered site
was found by the earlier verdict/evidence review (in contrast to round
8's `matrix.rs:678`/`set_entry_for_known` fixture collapse). That
review found evidence-shape defects (a single-producer argument
mislabeled `discriminating`, two citations I could not locate) but no
site whose *behavior* was wrong. This round's folded-condition
signature found two behavior-level blind sites the earlier
count-based review structurally could not see.

**Census §9 pass: none.** Applying §9's three clauses moved 2 rows to
`not-this-family` (see that section above), but `not-this-family` is a
classification, not a defect — neither exclusion names a site whose
behavior needs to change, only a site the family question never
applied to. No source touched, no commit, no gate owed for this pass.

**Coarse-assertion sweep (72-raw/71-real): two, both moveit-octomap,
both fixed:**

- `tree.rs:840-849`/`coord_to_key_checked_axis` — the guard's
  `is_finite()` conjunct is dead (IEEE 754 comparison semantics make
  the other two conjuncts already reject every non-finite input).
  Fixed by deleting it, `3e0430d` — no test could ever have closed this
  since no input makes the clause decisive.
- `tree.rs:903-906`/`log_odds_at` — the out-of-bounds `None` cause
  (`coord_to_key_checked` failing) had zero coverage anywhere in the
  crate; every existing test, including `octomap_parity.rs`'s fixture
  corpus, only ever reached the in-bounds-but-unmapped `None` cause.
  Fixed by adding `out_of_bounds_coordinate_has_no_occupancy_even_when
  _the_tree_center_is_mapped`, `08181da`.

## Commands run

```
git merge --ff-only main
python3 <scratchpad>/census.py . moveit-scene moveit-octomap moveit-state \
  moveit-constraints moveit-metrics moveit-smoothing moveit-srdf
git log --oneline df36fab..HEAD -- crates/moveit-scene/
git diff df36fab HEAD -- crates/moveit-scene/
git merge-base --is-ancestor bedc8ea df36fab   # exit 1 (false)
git show 0d10a11 --stat
git show 567342f --stat
# node.rs:143 bite — Node::new() eagerly allocates the children array,
# eliminating the outer-None cause:
cargo test -p moveit-octomap --lib node::tests -- --nocapture

# --- verdict/evidence review round ---
git merge --ff-only main   # picks up p1-robotmodel's merged ledger, e5590c3
git log --all --oneline -- crates/moveit-state/tests/jacobian.rs crates/moveit-state/src/state.rs
git log --all --oneline -- crates/moveit-srdf/tests/boundaries.rs
grep -rn "jacobian\|Parser::required" doc/*.md
# tree.rs:1762/1763/1794 — search()'s two None producers, both directions:
cargo test -p moveit-octomap --lib -- tree::tests::unmapped_coordinate_has_no_occupancy \
  tree::tests::insert_ray_cut_short_by_max_range_records_only_a_miss --nocapture
# jacobian.rs:211 — Posed::jacobian's two Err sites, both directions:
cargo test -p moveit-state --test jacobian -- an_unknown_group_name_is_unknown_name_not_not_a_chain --nocapture
git status --short   # empty, both before and after every mutation
git diff --stat       # empty at end of round

# --- folded multi-operand condition audit ---
# decide.rs:183/184 — joint.rs:120's tolerance OR-guard, both directions:
cargo test -p moveit-constraints --test decide -- new_rejects_negative_tolerance --nocapture
# acceleration_filter.rs:552 — reset's positions/velocities OR-guard, both directions:
cargo test -p moveit-smoothing --lib -- acceleration_filter::tests::reset_rejects_a_mismatched_length --nocapture
# ruckig_filter.rs:546 — reset's 3-clause OR-guard, all three directions:
cargo test -p moveit-smoothing --lib -- ruckig_filter::tests::reset_rejects_a_mismatched_length --nocapture
# fix bite-verification, new isolating tests both crates:
cargo test -p moveit-smoothing --lib -- acceleration_filter::tests::reset_rejects --nocapture
cargo test -p moveit-smoothing --lib -- ruckig_filter::tests::reset_rejects --nocapture
cargo fmt --all
cargo clippy -p moveit-smoothing --all-targets -- -D warnings
cargo nextest run -p moveit-smoothing

# --- census §9 family-membership pass ---
# read fresh from main, not the worktree copy:
# /home/stevek/work/moveit-rs/doc/assertion-discrimination-census.md
sed -n '586,595p' crates/moveit-collision/src/world.rs   # World::move_object's 3 arms
sed -n '105,177p' crates/moveit-constraints/tests/constraint_sampler_manager.rs
sed -n '141,312p' crates/moveit-constraints/src/constraint_sampler_manager.rs
sed -n '2003,2021p' crates/moveit-scene/src/scene.rs      # decouple_parent
sed -n '52,83p' crates/moveit-octomap/src/node.rs          # Node::new/child/create_child

# --- coarse-assertion sweep (count-coarse-assertions.py) ---
git merge main   # 6a14a89, the scanner
python3 tools/ci/count-coarse-assertions.py crates/moveit-octomap crates/moveit-scene \
  crates/moveit-constraints crates/moveit-srdf crates/moveit-state \
  crates/moveit-metrics crates/moveit-smoothing
grep -E ':eq_none:|:eq_err:' <output> | grep -v 'assert_eq!'   # tree.rs:1781 false positive
# coord_to_key_checked_axis bite, all three folded conjuncts, each reverted:
cargo nextest run -p moveit-octomap coord_to_key_checked_axis
# fix + gate:
cargo fmt --all && cargo clippy -p moveit-octomap --all-targets -- -D warnings
cargo nextest run -p moveit-octomap   # 67/67, then 68/68 after the log_odds_at fix
# log_odds_at bite (silent clamp to root_key), reverted:
cargo nextest run -p moveit-octomap out_of_bounds_coordinate_has_no_occupancy
git merge main   # ccac7ea, the assertion-helper mechanism/site fix
python3 tools/ci/count-coarse-assertions.py crates/moveit-octomap crates/moveit-scene \
  crates/moveit-constraints crates/moveit-srdf crates/moveit-state \
  crates/moveit-metrics crates/moveit-smoothing   # 81 raw incl. my own new test's is_some
grep -oE ':via:[A-Za-z_]+:' <output> | sort -u   # one fn: assert_err_mentions, 8 call sites
```

## Gate

This round's earlier verdict/evidence review changed no source (no
fix was needed there). This round's folded-multi-operand-condition
audit did change source: `crates/moveit-smoothing/src/acceleration_filter.rs`
and `crates/moveit-smoothing/src/ruckig_filter.rs`, one new-test commit
each (`3c2d72f`, `2829ca2`). Gated `-p moveit-smoothing`:
`cargo fmt --all` (clean), `cargo clippy -p moveit-smoothing
--all-targets -- -D warnings` (clean, zero warnings), `cargo nextest
run -p moveit-smoothing` (36/36 passed, including the 5 new isolating
tests). `decide.rs`'s two bites needed no fix (existing tests already
discriminate) and `joint.rs` is confirmed reverted clean
(`git diff --stat` empty). The census §9 pass changed no source in any
crate — both exclusions are classifications, read directly from
already-committed code and an already-existing doc comment, with
nothing to mutate or fix. This document lives under `doc/`, outside
any crate — `cargo fmt --all -- --check` run per your instruction,
clean.

The coarse-assertion sweep changed source twice, both in
`crates/moveit-octomap/src/tree.rs`, one commit each: the dead
`is_finite()` conjunct removed from `coord_to_key_checked_axis`
(`3e0430d`) and the new `out_of_bounds_coordinate_has_no_occupancy_
even_when_the_tree_center_is_mapped` test closing `log_odds_at`'s
out-of-bounds blind spot (`08181da`). Gated `-p moveit-octomap` after
each: `cargo fmt --all` (clean both times), `cargo clippy -p
moveit-octomap --all-targets -- -D warnings` (clean, zero warnings,
both times), `cargo nextest run -p moveit-octomap` (67/67 after the
first fix, 68/68 after the second). Every other crate in this sweep
(scene, constraints, srdf, state) needed no fix — every in-family row
found was already discriminating by direct source read, so no gate was
owed for them.

## UNFIXED

None.
