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

- **`node.rs:153` (`assert!(n.child(i).is_none(), "slot {i} should stay empty");`)** (moveit-octomap) — `create_child`'s guard is one
  predicate (`self.children.as_deref()?.get(i)`) over one value (slot
  index `i`), not a folded condition. Signature does not apply.
- **`invariants.rs:585,586,587,588,589,594,595,596,597` (`assert!(state.variable_velocity("no_such_joint").is_err());`)** (moveit-state,
  9 rows) — re-read all 9 accessor bodies (`crates/moveit-state/src/state.rs:369-513` (`pub fn set_variable_efforts(&mut self, values: &[f64]) {`)): each
  has exactly one `?` on one named operand (`name`), no folded OR/AND.
  Signature does not apply to any of the 9.
- **`crates/moveit-constraints/tests/constraint_sampler_manager.rs:172` (`assert!(`)** (moveit-constraints) — guard
  is `if regions.len() != 1` in `with_updated_position`, one condition
  over one value (region count). Signature does not apply.
- **`decide.rs:183,184`** (moveit-constraints) — **matches the
  signature.** `JointConstraint::new`'s guard (`crates/moveit-constraints/src/joint.rs:120`) is
  `tolerance_above < 0.0 || tolerance_below < 0.0`, one `Err::construct`
  folding two named operands. Bit both directions: neutralizing
  `tolerance_above`'s clause alone (`if false && tolerance_above < 0.0 || tolerance_below < 0.0`) failed the assertion at `decide.rs:183` (`assert!(JointConstraint::new(&model, "panda_joint1", 0.0, -0.1, 0.1, 1.0).is_err());`)
  exactly; neutralizing `tolerance_below`'s clause alone failed at
  `decide.rs:184` (`assert!(JointConstraint::new(&model, "panda_joint1", 0.0, 0.1, -0.1, 1.0).is_err());`) exactly. The two existing test call sites
  (`tolerance_above=-0.1,tolerance_below=0.1` at 183;
  `tolerance_above=0.1,tolerance_below=-0.1` at 184) already isolate
  one operand each — genuinely `discriminating`, no fix needed. `joint.rs`
  reverted clean after each bite. **Verdict corrected below.**
- **`utils_parity.rs:221,641,885,896,942` (`.unwrap_err();`)** (moveit-constraints, 5 rows)
  — re-checked each guard: `221` is a single `?` on group lookup, `641`
  is `regions.len() != 1` (one value), `885`/`896` are
  `resolve_frame_to_link`'s single closure-result `None`, `942` is a
  single `if` on the `XyzEuler`-tolerance-across-a-frame-change
  condition (one boolean expression, not an OR/AND over distinct named
  operands — `crates/moveit-constraints/src/utils.rs:796-802` re-read to confirm). Signature does not
  apply to any of the 5.
- **`crates/moveit-metrics/src/lib.rs:1068,1072` (`assert!(matches!(`)** (moveit-metrics) — `KinematicsMetrics::group`'s
  `is_chain()` check and the `?` on `joint_model_group` are two
  *separate* construction sites (not one folded condition); this pair
  was already correctly split from the family these two rows test.
  Signature does not apply.
- **`acceleration_filter.rs:302`** (moveit-smoothing) — **matches the
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
- **`ruckig_filter.rs:326-329` (`if positions.len() != num_joints`)** (moveit-smoothing) — **matches the
  signature**, structurally identical 3-clause OR
  (`positions.len() != num_joints || velocities.len() != num_joints ||
  accelerations.len() != num_joints`, `ruckig_filter.rs:326-329` (`if positions.len() != num_joints`)).
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
(`decide.rs:183/184` sharing one guard; `acceleration_filter.rs:302` (`if positions.len() != num_joints || velocities.len() != num_joints {`);
`ruckig_filter.rs:326-329` (`if positions.len() != num_joints`)). **Outcome: `decide.rs:183/184` — genuinely
discriminating, existing tests already isolate each operand, no source
fix, verdict-only correction. `acceleration_filter.rs:302` (`if positions.len() != num_joints || velocities.len() != num_joints {`) and
`ruckig_filter.rs:326-329` (`if positions.len() != num_joints`) — genuine blind sites, neither existing test
isolated any individual operand; both fixed this round with new
isolating tests, bite-verified, gated `-p moveit-smoothing`.**

## Census §9 — family membership

Applied `doc/assertion-discrimination-census.md` §9's three clauses
(mechanism / decision / subject) to all 49 rows, fresh — not copied
from any other ledger's answers, per your instruction. Every row's
disposition and reason is recorded per-crate above, in the `in-family`
column plus a prose note under the two crates with an exclusion.

**Checked: 49. Moved to `not-this-family`: 2. In-family: 47/49.**

- **`scene.rs:2210`** (moveit-scene) — clause 1 (mechanism) fails.
  `matches!(outcome, MoveObjectOutcome::Moved(_))` targets the
  success-with-effect arm of a 3-variant enum, not a "could not/did not
  produce X" tag — `NotFound`/`NoChange` would have held clause 1,
  `Moved` does not. Previously `discriminating`; that verdict answered
  a different, prior question (§9's own vocabulary section: `not-this-
  family` "is not a fourth answer to that question; it is 'the
  question does not apply'"), so the two are not in tension.
- **`crates/moveit-constraints/tests/constraint_sampler_manager.rs:172` (`assert!(`)** (moveit-constraints) — clause
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
`acceleration_filter.rs:302` (`if positions.len() != num_joints || velocities.len() != num_joints {`), and `ruckig_filter.rs:326-329` (`if positions.len() != num_joints`) — the first
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
| scene.rs:2210 | matches! | `diff_scene_records_a_move_only_change_for_an_existing_object` | discriminating | round-report — this same session, earlier round: ran 3 bites on `PlanningScene::move_object` (`scene.rs:1009` (`pub fn move_object(&mut self, id: &str, transform: Isometry3) -> MoveObjectOutcome {`)), reverted after each. (1) reachability: forced outcome to `NotFound` → test fails at the `matches!`. (2) discrimination: forced outcome to sibling `NoChange` → test fails at the same `matches!`. (3) payload: kept `Moved` but swapped the notification's `Action` to `CREATE` → the wildcarded `Moved(_)` still passes, but the test's own second assertion (`diff.get("box").unwrap() == Action::MOVE_SHAPE`) catches it. You re-read and confirmed this in-session ("Verified `scene.rs:2210` and agreed — `MoveObjectOutcome`'s three variants map 1:1 to the three guards, bites 1 and 2 both fail... `discriminating`, no fix"). No commit — no fix was needed, gate was `-p moveit-scene` clean at the time. | **not-this-family** — see below |
| scene.rs:2386 | bare `.is_none()` | (unnamed, `WorldDiff::get`-adjacent) | discriminating | bite (this round) — `WorldDiff::get`'s single guard (`world_diff.rs:104-106` (`pub fn get(&self, id: &str) -> Option<Action> {`)) is the sole producer; see `world_diff.rs:315` bite below, same guard shape | yes |
| scene.rs:2584 | bare `.is_none()` | `decouple_parent_then_mutating_the_former_parent_is_not_observed` | single-branch | structural — `decouple_parent` (`scene.rs:2036-2054`) has exactly one `self.parent = None;` site (verified by `rg 'self\.parent = ' crates/moveit-scene/src/scene.rs`). Corrected from `discriminating`: "exactly one producing site" proves there is nothing to discriminate from, not that there is | yes |
| scene.rs:2616 | bare `.is_none()` | `decouple_parent_materializes_the_inherited_transforms_map` | single-branch | structural — same single `self.parent = None;` site as 2557. Corrected from `discriminating`, same reason | yes |
| scene.rs:2651 | bare `.is_none()` | `decouple_parent_then_the_childs_inherited_attached_body_frame_still_resolves` | single-branch | structural — same single `self.parent = None;` site. Corrected from `discriminating`, same reason | yes |
| scene.rs:2676 | bare `.is_none()` | `decouple_parent_then_the_childs_inherited_world_object_still_resolves` | single-branch | structural — same single `self.parent = None;` site. Corrected from `discriminating`, same reason | yes |
| scene.rs:2758 | bare `.is_none()` | `clear_diffs_resets_a_diverged_child_to_a_fresh_diff_against_the_parent` | discriminating | bite (this round) — removed `self.acm = Layered::Inherited;` from `clear_diffs`, assertion flipped, reverted | yes |
| scene.rs:2789 | bare `.is_err()` | `frame_transform_resolves_the_model_frame_and_a_link_name` | single-branch | structural — re-read `frame_transform` (`scene.rs:1345-1365`) line by line: it has two possible `Err`-producing statements, `posed.global_link_transform(link_name)?` (reached only when `frame_id` resolves to an attached body, `scene.rs:1356`) and the final fallthrough `self.transforms().transform(frame_id).copied()` (`scene.rs:1364`, reached when no tier matched at all). `"world"` resolves in no tier and is not an attached-body name, so only the fallthrough fires. No test in this file's `.is_err()`/`matches!` family exercises the attached-body error path (`rg 'frame_transform\(' crates/moveit-scene/` across `scene.rs` and `frame_transform_parity.rs`), so within the population of sites actually asserted, there is only one reachable producer. Corrected from `discriminating`: the earlier evidence ("isolates... from... tested moments earlier") named tiers that succeed, not a second `Err`-producing sibling — a real second producer exists in the function but nothing in this family tests it | yes |
| scene.rs:2874 | bare `.is_err()` | `frame_transform_reports_a_name_resolving_in_no_tier` | single-branch | structural — same fallthrough-only reasoning as 2762; `"nothing"` is not an attached-body name either. Corrected from `discriminating`, same reason | yes |
| scene.rs:2933 | bare `.is_err()` | `frame_transform_tier_six_absent_name_is_still_unknown` | single-branch | structural — same fallthrough-only reasoning; `"no_such_frame"` is not an attached-body name. Corrected from `discriminating`, same reason | yes |
| scene.rs:2942 | bare `.is_err()` | `frame_transform_tier_six_empty_name_is_unknown` | single-branch | structural — same fallthrough-only reasoning; `""` is not an attached-body name. Corrected from `discriminating`, same reason | yes |
| world_diff.rs:315 | bare `.is_none()` | `set_with_uninitialized_action_erases_the_entry` | discriminating | bite (this round) — neutralized `WorldDiff::set`'s UNINITIALIZED branch (`if false && ...`), assertion flipped, reverted | yes |
| frame_transform_parity.rs:254 | bare `.is_err()` | `panda_frame_transform_matches_the_oracle` | single-branch | structural — same `frame_transform` fallthrough-only reasoning as 2759/2844/2903/2912; `query.name == "nothing"` is not an attached-body name in this fixture (`build_scene`'s attach loop only populates from `request.attached_bodies`). Corrected from `discriminating`, same reason | yes |

**`scene.rs:2210` moved to `not-this-family` (clause 1, mechanism).**
Re-read `World::move_object` (`crates/moveit-collision/src/world.rs:733-742`):
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
| node.rs:143 | bare `.is_none()` | `fresh_node_has_no_children_and_zero_log_odds` | single-branch | bite (this round) — made `Node::new()` eagerly allocate the children array (eliminating the "no array" cause); assertion still passed, proving this test cannot distinguish "no array" from "array present, slot empty" — matches the function's own doc comment (`node.rs:66-67` (`/// Upstream`), "`None` covers both...", a deliberate 2-cause union mirroring upstream's `nodeChildExists`+`getNodeChild`) | yes — `child()`'s `self.children.as_ref()?[idx]` genuinely executes on a fresh node (§9's own `nn.rs:227`/`self.root.as_ref()?` precedent: a written guard that runs even on an "empty" fixture is still a decision) |
| node.rs:153 | bare `.is_none()` | `create_child_populates_exactly_one_of_eight_slots` | single-branch | structural — fixture calls `create_child(3)` first, which allocates the array (`get_or_insert_with`, `node.rs:78-80`); only the "array present, slot `i` empty" cause is reachable, the "no array" cause is structurally excluded by this fixture | yes |
| tree.rs:1736 | bare `.is_none()` | `ray_with_end_outside_tree_bounds_returns_none` | discriminating | bite (this round, §3a mirror) — neutralized the `end`-guard in `compute_ray_keys` (`.unwrap_or_else(Self::root_key)`), this assertion flipped, reverted. Renumbered from a stale tree.rs line 1729 | yes |
| tree.rs:1740 | bare `.is_none()` | `ray_with_end_outside_tree_bounds_returns_none` | discriminating | bite (this round) — same test, second assertion, covered by the same end-guard mutation above. Renumbered from a stale tree.rs line 1733 | yes |
| tree.rs:1751 | bare `.is_none()` | `ray_with_origin_outside_tree_bounds_returns_none` | discriminating | bite (this round, §3a mirror) — neutralized the `origin`-guard independently, this assertion flipped while the end-guard mutation left it green; reverted. Renumbered from a stale tree.rs line 1744 | yes |
| tree.rs:1769 | bare `.is_none()` | `unmapped_coordinate_has_no_occupancy` | discriminating | bite (this round) — `search` (`tree.rs:883-897`, signature line to closing brace; cited `:876-890` before a merge shifted it) has two `None` producers, `self.root.as_deref()?` (empty tree) and the loop's `has_children()`-gated arm (ambiguous partial structure); these are real siblings, so ran both directions. Bite 1: gave the root-absent guard a fallback empty node (temp `Node::EMPTY` const + `.unwrap_or(&Node::EMPTY)` in `search`) — this test FAILED (log_odds_at/is_occupied became `Some` instead of `None`), tree.rs:1822 stayed green. Bite 2: neutralized the inner `has_children()`-gated arm to always `Some(cur)` — this test stayed green (unaffected, root already absent so the loop is never entered), tree.rs:1822 FAILED. Both mutations reverted; `git status --short`/`git diff --stat` empty after. Renumbered from a stale tree.rs line 1762 | yes |
| tree.rs:1770 | bare `.is_none()` | `unmapped_coordinate_has_no_occupancy` | discriminating | bite (this round) — same test, same bite pair as tree.rs:1769 above (`is_occupied` composes `log_odds_at`, same root-absent cause). Renumbered from a stale tree.rs line 1763 | yes |
| tree.rs:1822 | bare `.is_none()` | `insert_ray_cut_short_by_max_range_records_only_a_miss` | discriminating | bite (this round) — see tree.rs:1769's bite pair; this test flips under bite 2 (inner `has_children()` guard neutralized) and stays green under bite 1 (root-absent guard neutralized), the mirror image of 1769/1770, confirming it exercises the second, distinct `None` producer. Renumbered from a stale tree.rs line 1794, which under nearest-line matching in the current file is ambiguous between two unrelated real sites (`tree.rs:1790`/`1791`) | yes |
| tree.rs:1958 | bare `.is_none()` | `leaves_in_bbx_returns_none_for_an_out_of_range_max` | discriminating | commit — doc comment (`tree.rs:1952-1954` (`/// Distinct from`)) and prior isolating mutation recorded at `0d10a11`; `git show 0d10a11 --stat` confirms it touches this test/guard pair. Renumbered from a stale tree.rs line 1930 | yes |
| tree.rs:1972 | bare `.is_none()` | `leaves_in_bbx_returns_none_for_an_out_of_range_min` | discriminating | commit — doc comment (`tree.rs:1964-1968` (`/// Distinct from`), explicitly states "before this test existed the `min` guard had no coverage at all") and prior isolating mutation recorded at `567342f`; `git show 567342f --stat` confirms. Renumbered from a stale tree.rs line 1944 | yes |

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
| jacobian.rs:211 | matches! | `an_unknown_group_name_is_unknown_name_not_not_a_chain` | discriminating | bite (this round) — I could not locate a citable p1-robotmodel bite for this site: `git log --all` on `jacobian.rs`/`state.rs` shows no commit past the original port (`9230ad3`), no `doc/` file mentions it (including the newly-merged `assertion-discrimination-ledger-p1-robotmodel.md`, whose 7 crates are `moveit-trajectory`/`-planners-chomp`/`-planners-sbp`/`-planners-stomp`/`-planning`/`-sampling`/`-kinematics` — not `moveit-state`), and `Posed::jacobian` (`crates/moveit-state/src/state.rs:1121-1146` (`regardless of its (here, empty) input slice.`)) carries no doc-recorded mutation. Ran it myself instead: `Posed::jacobian` has two real sibling `Err` sites, `model.joint_model_group(group)?` (`crates/moveit-state/src/state.rs:1123` (`/// because in this port: the root link's`)) and the `is_chain()` guard's `Error::other(...)` (`crates/moveit-state/src/state.rs:1125-1129` (`regardless of its (here, empty) input slice.`)). Bite 1 (reachability): made the group-lookup guard fall back to `"panda_arm"` on failure instead of propagating — test FAILED, `unwrap_err()` panicked on an `Ok` (proves the guard is necessary for any error at all). Bite 2 (discrimination/sibling-swap): kept the guard failing but swapped its `Err` to `Error::other(...)` instead of propagating `UnknownName` — test FAILED at the `matches!`, proving the assertion discriminates the sibling. Both reverted; `git status --short`/`git diff --stat` empty after | yes |

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

## moveit-constraints (10 sites)

| file:line | anchor | test fn | verdict | evidence | in-family |
|---|---|---|---|---|---|
| constraint_sampler_manager.rs:172 | bare `.is_none()` | `no_constraints_and_no_solver_returns_none` | single-branch | structural — re-traced `select_default_sampler`/`_inner`'s full Step A/B/C/D control flow (`constraint_sampler_manager.rs:141-`); no test anywhere in the file exercises the *other* None-producer (`select_default_sampler:149-151`'s unknown-group-name early return — `rg 'select_default_sampler\('` on the test file shows every call uses a valid `"panda_arm"`/`"panda_arm_hand"` group name), so only the Step-D fallthrough is reachable and tested | **not-this-family** — see below |
| decide.rs:183 | bare `.is_err()` | `new_rejects_negative_tolerance` | discriminating | bite (this round) — corrected from `single-branch`/`structural`: `JointConstraint::new`'s guard (`crates/moveit-constraints/src/joint.rs:120`, `tolerance_above < 0.0 \|\| tolerance_below < 0.0`) is one `Err::construct` site but folds two named operands, which a construction-site count cannot see through. Neutralizing `tolerance_above`'s clause alone (`if false && tolerance_above < 0.0 \|\| tolerance_below < 0.0`) failed this assertion exactly. The test's own fixture (`tolerance_above=-0.1, tolerance_below=0.1`) already isolates this operand. `joint.rs` reverted clean after | yes |
| decide.rs:184 | bare `.is_err()` | `new_rejects_negative_tolerance` | discriminating | bite (this round) — mirror of 183: neutralizing `tolerance_below`'s clause alone (`if tolerance_above < 0.0 \|\| false && tolerance_below < 0.0`) failed this assertion exactly. The test's own fixture (`tolerance_above=0.1, tolerance_below=-0.1`) already isolates this operand. `joint.rs` reverted clean after | yes |
| decide.rs:210 | matches! | `new_rejects_unknown_joint` | fixed | commit `83e3c1c` — this session's own prior fix, asserts the specific `Error::UnknownName{kind:"joint",..}` variant/field rather than bare `.is_err()` | yes |
| utils_parity.rs:222 | matches! | `unknown_group_is_error` | single-branch | structural — `construct_goal_joint_constraints`'s only reachable guard for an unknown group name is `model.joint_model_group(group_name)?` (`crates/moveit-constraints/src/utils.rs:234`); the loop body's two further `?` sites are unreached when this one fails first | yes |
| utils_parity.rs:729 | matches! | `multi_region_constraint_is_error` | single-branch | structural — `update_position_constraint` has exactly one `Error::Other`-producing site (`crates/moveit-constraints/src/utils.rs:605-609`, converting `with_updated_position`'s `None` for a >1-region constraint); `with_updated_position`'s own `?`-propagated errors are `Error::UnknownName`, never `Error::Other`, so there is no second producer to conflate — re-verified `2026-08-05`, line renumbered from a stale utils_parity.rs line 641 | yes |
| utils_parity.rs:973 | bare `.is_none()` | `an_unrecognised_frame_is_none` | single-branch | structural — `resolve_position_constraint_frame` (`crates/moveit-constraints/src/utils.rs:727-747`) has one `None`-producing path, `resolve_frame_to_link`'s own single `None` cause (`crates/moveit-constraints/src/utils.rs:641` (`Ok(resolve_attached_frame(frame_id))`), the closure result — the two other tiers only ever return `Some` or fall through, never `None`) — line renumbered from a stale utils_parity.rs line 885 | yes |
| utils_parity.rs:984 | bare `.is_none()` | same fn, second assertion | single-branch | structural — `resolve_orientation_constraint_frame` (`crates/moveit-constraints/src/utils.rs:777-805`) shares the identical single `resolve_frame_to_link` None cause — line renumbered from a stale utils_parity.rs line 896 | yes |
| utils_parity.rs:1030 | matches! | `xyz_euler_tolerance_across_a_real_frame_change_is_an_error` | single-branch | structural — `resolve_orientation_constraint_frame`'s only `Error::Other`-producing site is the `XyzEuler`-tolerance-across-a-frame-change guard (`crates/moveit-constraints/src/utils.rs:796-802` (`if matches!(tolerance, OrientationTolerance::XyzEuler { .. }) {`)), a single `if` — line renumbered from a stale utils_parity.rs line 942 | yes |
| sampler_self_validation.rs:637 | bare `.is_empty()` | `every_sampled_state_satisfies_its_own_constraints` | discriminating | bite (2026-08-05, the round that added the file) — two independent mutations, each reverted after, and neither silent: (a) nudging every sampled state by `+0.5` on `panda_joint1` before it is decided fired this assertion for all seven sampler configurations (2178/10000 states still satisfied, so the message named which sampler and how many); (b) `MAX_IK_ATTEMPTS = 0` fired it for the five IK-backed configurations with `produced 0 of its N states ... -- a vacuous 100%`, the zero-production branch that exists precisely so a sampler that converges on nothing cannot report a perfect rate. `failures` is a `Vec<String>` this test builds from per-sampler counters, so the `is_empty()` shape is a collected-diagnostics assertion, not a single-branch guard: the two `assert_eq!`s below it restate the same totals as numbers | yes |

**`crates/moveit-constraints/tests/constraint_sampler_manager.rs:172` (`assert!(`) moved to `not-this-family` (clause
2, decision).** Re-read `select_default_sampler_inner`
(`crates/moveit-constraints/src/constraint_sampler_manager.rs:202-312`) and the test's own fixture
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
`shortest_solution_is_none_on_empty_input`), not a `nn.rs:227` (`assert!(gnat.nearest(&space, &vec![0.0]).is_none());`)-style
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
| lib.rs:1068 | matches! | `unknown_group_is_unknown_name` | single-branch | structural + doc-recorded — doc comment (`crates/moveit-metrics/src/lib.rs:1053-1059`) states `manipulability` and `manipulability_index` "share `KinematicsMetrics::group`'s `self.model.joint_model_group(group)?` call verbatim"; single guard | yes |
| lib.rs:1072 | matches! | same fn, second assertion | single-branch | structural + doc-recorded — same shared single guard, `manipulability_index` caller | yes |
| lib.rs:1139 | matches! | `manipulability_ellipsoid_rejects_the_same_bad_groups` | single-branch | doc-recorded bite — doc comment (`crates/moveit-metrics/src/lib.rs:1102-1123`) records a mutation already run and its result: replacing `self.group(...)?` with `let _ = self.group(...);` inside `manipulability_ellipsoid` "leaves the variant-level assertions below unchanged" because `state.jacobian`'s own re-check produces the byte-identical `UnknownName` either way; this line's assertion does *not* pin `self.group`'s own call site (only the message-level check at 1132-1138, outside this family, does that) | yes — the recorded mutation shows a real, engineer-could-implement-wrong decision (which call site produces the error) still belonging to `manipulability_ellipsoid`; the ambiguity is *which* subject-side decision fires, not whether one does |

All 3 use a real unknown group name (never an empty/vacuous group
argument), so clause 2 holds on real data, not a skipped comparison.

## moveit-smoothing (2 sites)

| file:line | anchor | test fn | verdict | evidence | in-family |
|---|---|---|---|---|---|
| acceleration_filter.rs:575 | matches! | `reset_rejects_a_mismatched_length` | discriminating | bite (this round) — corrected from `single-branch`/`structural`: `reset`'s guard (`acceleration_filter.rs:302`, `positions.len() != num_joints \|\| velocities.len() != num_joints`) is one `Error::Other` site folding two named operands. This test's fixture mismatches both `positions` and `velocities` at once, so neither existing test isolated either clause — bit both directions (`false && positions...`, then `positions... \|\| false && velocities...`), both left this test PASSING: a genuine blind site, not just an undercounted branch. Fixed by adding `reset_rejects_a_positions_only_mismatch`/`reset_rejects_a_velocities_only_mismatch` (each mismatching exactly one array), bite-verified to fail when their own clause is disabled. Commit `3c2d72f`, gated `-p moveit-smoothing`. Line renumbered from a stale `:552`, shifted +23 by Round 6's `do_smoothing_rejects_a_length_mismatch` insertion | yes |
| acceleration_filter.rs:582 | matches! | `reset_rejects_a_positions_only_mismatch` | discriminating | bite-verified fix for the row above; own row added this round — this exact site had only ever been referenced by name, never cited. Line renumbered from a stale `:559`, same +23 shift | yes |
| acceleration_filter.rs:589 | matches! | `reset_rejects_a_velocities_only_mismatch` | discriminating | bite-verified fix for the row above; own row added this round. Line renumbered from a stale `:566`, same +23 shift | yes |
| ruckig_filter.rs:659 | matches! | `reset_rejects_a_mismatched_length` | discriminating | bite (this round) — corrected from `single-branch`/`structural`: same shape as acceleration_filter.rs:552, 3-clause OR (`ruckig_filter.rs:326-329` (`if positions.len() != num_joints`)) folding `positions`/`velocities`/`accelerations`. This test's fixture mismatches all three at once. Bit all three directions individually — each left this test PASSING, the same blind site three ways. Fixed by adding `reset_rejects_a_positions_only_mismatch`/`reset_rejects_a_velocities_only_mismatch`/`reset_rejects_an_accelerations_only_mismatch`, bite-verified each fails when its own clause is disabled. Commit `2829ca2`, gated `-p moveit-smoothing`. Line renumbered from a stale ruckig_filter.rs line 546, then again from `:650` (+9, Round 6's `ruckig_filter.rs:289` (`.map_err(|error| Error::other(format!("ruckig update failed: {error}")))?;`) comment fix) | yes |
| ruckig_filter.rs:666 | matches! | `reset_rejects_a_positions_only_mismatch` | discriminating | bite-verified fix for the row above; own row added this round. Renumbered from a stale `:657`, same +9 shift | yes |
| ruckig_filter.rs:673 | matches! | `reset_rejects_a_velocities_only_mismatch` | discriminating | bite-verified fix for the row above; own row added this round. Renumbered from a stale `:664`, same +9 shift | yes |
| ruckig_filter.rs:680 | matches! | `reset_rejects_an_accelerations_only_mismatch` | discriminating | bite-verified fix for the row above; own row added this round. Renumbered from a stale `:671`, same +9 shift | yes |

Both bite-confirmed against real mismatched-length data (never an
empty-input skip), so clause 2 holds on real operands.

## moveit-srdf (2 sites)

| file:line | anchor | test fn | verdict | evidence | in-family |
|---|---|---|---|---|---|
| boundaries.rs:106 | matches! | `a_virtual_joint_missing_any_required_attribute_is_dropped` | discriminating | structural — I could not locate a citable p9-ros `Parser::required` bite for this site (`git log --all` on `boundaries.rs`/`parse.rs` shows only `0ca2af8` (initial port) and `b819ec1` (a *different* test, `malformed_xml_is_an_error`); no `doc/` file mentions it). Traced the real mechanism instead: `load_virtual_joints` (`parse.rs:107-127`) calls `Parser::required` (`parse.rs:82-98`) four times in sequence, once per attribute, each call passing its own `&'static str` literal (`"name"`/`"child_link"`/`"parent_frame"`/`"type"`) that `required` stores verbatim into `Diagnostic::MissingAttribute{attribute,..}`. These are four distinct call sites with four distinct compile-time literals, not one shared constant — genuine siblings, structurally guaranteed distinguishable by the type system, no mutation needed to prove it | yes |
| boundaries.rs:422 | matches! | `an_unparsable_joint_value_drops_the_joint_instead_of_storing_zero` | discriminating | structural — same search as boundaries.rs:106 above, same absence of a citable prior bite. Traced the real mechanism: `load_group_states` (`parse.rs:249-300`) has exactly one `MalformedValue`-producing site for a joint value (`parse.rs:285-290`, hardcoded `attribute: "value"`) — single-branch *within this function*. But `rg 'Diagnostic::MalformedValue' crates/moveit-srdf/src/parse.rs` finds two more sibling sites elsewhere in the same parser, `parse.rs:364` (`self.warn(Diagnostic::MalformedValue {`) (`attribute: "center"`) and `parse.rs:373` (`self.warn(Diagnostic::MalformedValue {`) (`attribute: "radius"`), for the sphere-collision parser. The `matches!`'s `attribute == "value"` check is what distinguishes this site's diagnostic from those two real siblings at the population level, even though none of the three are reachable from each other's fixtures | yes |

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

- **72 → 71.** `tree.rs:1781` (`/// clamped an out-of-bounds point instead of rejecting it would find`)'s `eq_none` tag is a scanner false
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
8/6/3/0/0) once the `tree.rs:1781` (`/// clamped an out-of-bounds point instead of rejecting it would find`) false positive was pulled out.

### Per-crate verdicts, census §9 three clauses (mechanism / decision / subject)

**In-family denominator: 41 of 71.** Two blind sites found and fixed
(both moveit-octomap, both `eq_none`/`is_some`). No blind site found in
scene, constraints, srdf, or state — every in-family row there was
already discriminating, mostly because the test authors had already
paired the coarse assertion with a same-test companion (a preceding
non-vacuous state check, a same-test contrasting sibling case, or a
more specific diagnostic/variant assertion two lines away) that rules
out the sibling cause.

#### moveit-octomap (30 real sites, 34 in-family, 2 blind — both fixed)

**Row-count correction (orphan reconciliation, this round):** the
original 23-in-family figure only counted sites that already had a
table row. Ten more of the 29 real sites were in-family and
already-audited (bitten or doc-recorded) but referenced only by test
name in this section's own prose, `octomap_parity.rs:277` (`assert_eq!(`)'s note, or
the "Commands run" log below — never given their own row, which made
`tools/ci/reconcile-assertion-ledgers.py` (a table-row scanner) report
them as orphans despite being covered. One more (`tree.rs:1822` (`assert!(tree.log_odds_at(end).is_none());`), folded
into the `tree.rs:1769, 1770` row below) was found only because fixing
the stale tree.rs line 1824 citation removed an accidental window-match
that had been covering it — `29 real sites` becomes `30` with that
addition. The rows below marked
"Previously uncovered by any table row" / "had never itself been given
a table row" / "never given its own table row" are those ten sites;
none required a new bite — each cites evidence that already existed.

| file:line | kind | verdict | in-family | note |
|---|---|---|---|---|
| tree.rs:1781 | eq_none | **scanner false positive** | n/a | `assert!(insert_ray(..., None, ...))` — `None` is an argument, not a comparison |
| node.rs:151 | is_some | not-this-family | no | clause 2 — `create_child(3)` unconditionally populates slot 3 two lines above; no decision between cause and observation |
| tree.rs:1833 | contains_member | not-this-family | no | clause 1 — `occupied.contains(&hit_key)` is membership in `compute_update`'s actual computed classification, not an absence signal — renumbered from a stale tree.rs line 1805 |
| tree.rs:1834 | contains_member | not-this-family | no | clause 1, same reasoning — renumbered from a stale tree.rs line 1806 |
| tree.rs:1835 | is_empty | not-this-family | no | clause 2 — general "ray tracing produced *some* free cells" sanity check, no crafted decision boundary — renumbered from a stale tree.rs line 1807 |
| tree.rs:1852, 1865, 1880, 1890, 1896, 1906, 1907 | eq_none ×7 | **in-family, was partly blind** | yes | `coord_to_key_checked_axis`'s folded 3-operand guard (`is_finite() && >= min && < max`, one `return None` site). Bite-confirmed (3 live mutations, each reverted): `>= min` and `< max` are each independently caught by a dedicated boundary test (−32768.5/1e300 for min, 32768.0/1e300 for max) — genuinely discriminating. `is_finite()` is **dead**: neutralizing it alone left all 9 tests green, because IEEE 754 comparisons with NaN are always false and ±infinity always falls outside the finite `[min, max)` range, so the other two clauses already reject every non-finite input on their own. **Fixed** (`3e0430d`): removed the dead conjunct — no test could ever have closed this since no input makes it decisive, so the fix is deletion, not a new test. Line list renumbered from a stale `tree.rs:1824/1837/1852/1862/1868/1878/1879`; also switched to comma separation — the reconcile instrument's citation grammar only multi-extracts a comma-separated first column, so the original `/`-separated list was silently read as a single citation (`1824`) and the other six read as orphans regardless of line accuracy |
| tree.rs:1958, 1972 | is_none ×2 | in-family, discriminating | yes | `leaves_in_bbx_returns_none_for_an_out_of_range_{max,min}` — `LeavesInBbx::new` checks `min` then `max`, each behind its own `?`; doc-recorded bite (the source's own comment on the `min` test, `tree.rs:1964-1968` (`/// Distinct from`)): before that test existed, neutralizing the `min` guard left all 66 tests green, and each guard now isolates to its own test. Previously uncovered by any table row (only referenced implicitly, never cited) |
| tree.rs:1736, 1740, 1751 | is_none ×3 | in-family, discriminating | yes | `ray_with_end_outside_tree_bounds_returns_none` (1736/1740, both ends of the ray) and `ray_with_origin_outside_tree_bounds_returns_none` (1751) — the crate's own dedicated unit-level coverage of `compute_ray_keys`'s two `None` causes that `octomap_parity.rs:277` (`assert_eq!(`)'s note below already names but never cited directly |
| tree.rs:1769, 1770, 1822 | is_none ×3 | in-family, discriminating | yes | `unmapped_coordinate_has_no_occupancy` (1769/1770) and `insert_ray_cut_short_by_max_range_records_only_a_miss` (1822) both exercise `log_odds_at`'s in-range-but-unmapped `None` cause; paired with the out-of-bounds cause's own dedicated test (`tree.rs:1787, 1790, 1791` below) they jointly discriminate `log_odds_at`'s two causes, same reasoning as `octomap_parity.rs:216` (`assert_eq!(mapped, actual.is_some(), "{ctx}: occupancy mapped mismatch");`). Bite already recorded under "Commands run" below (`tree.rs:1762/1763/1794` old numbers) but never given its own table row. `tree.rs:1822` was previously mis-covered by an accidental window-match onto the pre-fix tree.rs line 1824 citation, not a genuine row — caught when fixing that citation exposed it as a real orphan |
| tree.rs:1787, 1790, 1791 | is_some, is_none, is_none | in-family, discriminating | yes | `out_of_bounds_coordinate_has_no_occupancy_even_when_the_tree_center_is_mapped` — this *is* the `08181da` fix's own new test (see `octomap_parity.rs:216` (`assert_eq!(mapped, actual.is_some(), "{ctx}: occupancy mapped mismatch");`) below); it is the direct source-level evidence for that fix and had never itself been given a table row |
| tree.rs:2008, 2018, 2024, 2030, 2038, 2048, 2058, 2077, 2090, 2251, 2284 | eq_err ×11 | in-family, discriminating | yes | `DecodeError::UnexpectedEof`/`TreeAlreadyPopulated`/`MaxDepthExceeded` are each reached from 2 production call sites (one per of `read_binary_data`/`read_data`, or one per `read_binary_node`/`read_data_node`'s shared `Cursor::read_u8`/`read_f32_le`). Checked the `DecodeError` catch-all risk you flagged directly: every one of the 11 tests' own in-source comments trace the exact single reachable call site for that test's crafted byte length, ruling out every sibling explicitly (e.g. "`TreeAlreadyPopulated` is excluded (fresh tree) and `MaxDepthExceeded` is unreachable (3 bytes cannot recurse to depth 16)"). None is a bare catch-all — each traces to one call site. Line list renumbered from a stale `tree.rs:1980/1990/1996/2002/2010/2020/2030/2049/2062/2223/2256`; also switched to comma separation, same reason as the `coord_to_key_checked_axis` row above |
| decode_parity.rs:200, 237 | eq_err ×2 | in-family, discriminating | yes | same `UnexpectedEof`-on-empty-input reasoning as `tree.rs:2024/2030`, run across the oracle fixture corpus. Switched to comma separation (numbers themselves unchanged — no drift here, only the `/`-separator parsing gap) |
| decode_parity.rs:195, 232 | is_empty ×2 | not-this-family | no | clause 3 — subject is the *oracle's* fixture data (`expected_nodes.is_empty()`), a fixture self-consistency check, not the code under test. Switched to comma separation (numbers themselves unchanged) |
| octomap_parity.rs:216 | is_some | **in-family, was blind** | yes | `log_odds_at`'s `None` has 2 causes (`coord_to_key_checked` out-of-range vs. `search` finds nothing); every fixture query point and every existing unit test only ever reached the second. Bite: replacing the out-of-bounds guard with a silent clamp to the tree center left all 67 tests green. **Fixed** (`08181da`): new test populates the tree center then queries a genuinely out-of-bounds point, so the same bite now fails |
| octomap_parity.rs:246 | is_some | in-family, not blind | yes | shares `log_odds_at`, now covered by the same fix; the out-of-bounds cause is structurally unreachable via `OccupancyByKey` (`key_to_coord` on a `u16` key is always in-range by construction), so only the `search`-null cause applies here, and that's discriminated per fixture row (mapped vs. unmapped rows both present) |
| octomap_parity.rs:277 | is_some | in-family, discriminating | yes | `compute_ray_keys`'s two `None` causes (origin vs. end out-of-bounds) are each separately exercised by dedicated fixture rows (`[0,0,0]→[1e9,0,0]` and `[1e9,0,0]→[0,0,0]`), matching the crate's own dedicated unit tests `ray_with_{origin,end}_outside_tree_bounds_returns_none` (now given their own row, `tree.rs:1736, 1740, 1751` above) |

#### moveit-scene (25 real sites, 11 in-family, 0 blind)

**Row-count correction (orphan reconciliation, this round):** this
section previously stated its verdicts as prose paragraphs naming
line-slash-lists (`scene.rs:2179/2150`, `world_diff.rs:158/159/...`)
instead of per-site table rows. `tools/ci/reconcile-assertion-ledgers.
py`'s citation parser only reads markdown table rows starting with
`|` — a prose paragraph, however precisely it names its lines, is
invisible to it. All 25 sites reported as orphans despite every one
already having a verdict and reasoning below; none needed new
investigation, only a table row. Converted to a table, one row per
site, same reasoning as before.

| file:line | kind | verdict | in-family | note |
|---|---|---|---|---|
| scene.rs:2179 | contains_member | not-this-family | no | clause 1 — `Action` bitflag membership check is the diff algorithm's own informative computed classification ("which actions occurred"), not a stand-in for an operation's inability to do something |
| scene.rs:2180 | contains_member | not-this-family | no | same reasoning |
| world_diff.rs:158 | contains_member | not-this-family | no | same `Action` bitflag reasoning |
| world_diff.rs:159 | contains_member | not-this-family | no | same reasoning |
| world_diff.rs:198 | contains_member | not-this-family | no | same reasoning |
| world_diff.rs:225 | contains_member | not-this-family | no | same reasoning |
| world_diff.rs:226 | contains_member | not-this-family | no | same reasoning |
| world_diff.rs:247 | contains_member | not-this-family | no | same reasoning |
| world_diff.rs:248 | contains_member | not-this-family | no | same reasoning |
| world_diff.rs:287 | contains_member | not-this-family | no | same reasoning |
| world_diff.rs:288 | contains_member | not-this-family | no | same reasoning |
| world_diff.rs:289 | contains_member | not-this-family | no | same reasoning |
| world_diff.rs:330 | is_empty | not-this-family | no | clause 2 — `a_fresh_diff_is_empty`: `WorldDiff::new()` is `Self::default()`, no decision to get wrong |
| world_diff.rs:331 | is_empty | not-this-family | no | same fresh-constructor reasoning |
| scene.rs:2182 | is_empty | in-family, discriminating | yes | pairs its coarse assertion with a same-test non-vacuous setup (a mutation immediately before the check proves the collection *was* populated), ruling out the "never touched" sibling |
| scene.rs:2197 | is_empty | in-family, discriminating | yes | same pattern |
| scene.rs:2378 | eq_none | in-family, discriminating | yes | same pattern |
| scene.rs:2397 | is_some | in-family, discriminating | yes | same pattern |
| scene.rs:2746 | is_some | in-family, discriminating | yes | same pattern |
| scene.rs:2752 | is_some | in-family, discriminating | yes | doc comment explicitly names the bug class it exists to catch ("`clear_diffs` resetting `attached_bodies`/`acm` to empty ... would be indistinguishable from correctly re-inheriting the parent's ... state") |
| scene.rs:2763 | is_empty | in-family, discriminating | yes | same pattern |
| scene.rs:3414 | is_empty | in-family, discriminating | yes | sits next to a sibling test proving the same collection is non-empty under different input |
| scene.rs:3438 | is_empty | in-family, discriminating | yes | same sibling-test pattern |
| world_diff.rs:297 | is_some | in-family, discriminating | yes | pairs its coarse assertion with a same-test non-vacuous setup |
| world_diff.rs:323 | is_empty | in-family, discriminating | yes | same pattern |

#### moveit-constraints (8 real sites, 3 in-family, 0 blind)

| file:line | kind | verdict | in-family | note |
|---|---|---|---|---|
| decide.rs:1161 | eq_none | in-family, discriminating | yes | `max_view_angle()`/`max_range_angle()` are `mimic()`-shaped getters; decision lives in `VisibilityConstraint::new`'s `normalize_angle_criterion` call, one call site per field |
| decide.rs:1166 | eq_none | in-family, discriminating | yes | same shape, own call site |
| decide.rs:1257 | is_empty | not-this-family | no | clause 2 — `KinematicConstraintSet::new()` is `Self::default()` |
| sampler.rs:194/200 | contains_member ×2 | not-this-family | no | clause 1 — `(min..=max).contains(&v)` validates numeric correctness of a sampled value, not an absence signal |
| utils_parity.rs:581, 647 | is_empty ×2 | not-this-family | no | clause 2 — `update_{orientation,position}_constraint`'s search loop runs over an empty `KinematicConstraintSet::new()`; the loop body's comparison never executes, matching census §9's `shortest_solution_is_none_on_empty_input` exclusion exactly — lines renumbered from a stale `utils_parity.rs:580/602` (this session's own `9b2bff6` inserted the sibling `mismatched_link_name_leaves_constraint_untouched` tests directly after each, at different offsets: `+1` and `+45`) |
| utils_parity.rs:786 | is_empty | in-family, discriminating | yes | `merge_constraints` drops a genuinely non-overlapping pair (`low > high`); sibling test `overlapping_windows_merge_to_the_intersection` proves the merge logic isn't vacuously always-empty — line renumbered from a stale utils_parity.rs line 698, which under nearest-line matching in the current file lands on an unrelated test (`multi_region_constraint_is_error`, not `non_overlapping_windows_are_dropped`) |

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

**Row-count correction (orphan reconciliation, this round):** stated
as prose, not a table row, which `tools/ci/reconcile-assertion-ledgers.
py` cannot see — see the identical mechanism documented under
moveit-scene above. Converted to a table; verdict unchanged.

| file:line | kind | verdict | in-family | note |
|---|---|---|---|---|
| invariants.rs:100 | contains_member | not-this-family | no | `(-PI..=PI).contains(&wrapped)` in `enforce_bounds_wraps_an_unbounded_continuous_joint_into_pi_range` — validating that an angle-wrapping computation's numeric output landed in range, not an absence signal |
| invariants.rs:140 | contains_member | not-this-family | no | same range check in `harmonize_positions_rewraps_without_changing_the_transform` |
| invariants.rs:368 | contains_member | not-this-family | no | same range check, `theta`-bounds-enforcement test |

#### moveit-metrics / moveit-smoothing

0 sites each, matching the count you were given.

## Sites needing a fix this round

Two, both found by this round's folded-multi-operand-condition audit
(see above), both fixed:

- `acceleration_filter.rs:302` (`if positions.len() != num_joints || velocities.len() != num_joints {`)/`reset` (moveit-smoothing) — neither
  the `positions` nor the `velocities` clause of the guard's OR
  condition was individually exercised by any existing test. Fixed:
  `reset_rejects_a_positions_only_mismatch`/
  `reset_rejects_a_velocities_only_mismatch` added, commit `3c2d72f`.
- `ruckig_filter.rs:326-329` (`if positions.len() != num_joints`)/`reset` (moveit-smoothing) — none of the
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
8's `matrix.rs:821` (`assert!(acm.entry("a", "a").is_none());`)/`set_entry_for_known` fixture collapse). That
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
sed -n '2033,2051p' crates/moveit-scene/src/scene.rs      # decouple_parent
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

## Round 3: five-crate fence + 7 stranded sites (37 sites)

Fence for this round, path-based per the standing correction above
(the `contains_msg`-belongs-to-p1-robotmodel kind rule is dead — the
scanner no longer emits that kind at all): `crates/moveit-smoothing`,
`crates/moveit-kinematics`, `tools/moveit-diff`, `crates/moveit-sampling`,
`crates/moveit-test-support`, plus a mid-round addition,
`crates/moveit-constraints/tests/sampler.rs` and
`crates/moveit-constraints/tests/utils_parity.rs` (`decide.rs` stays
p1-robotmodel's).

Re-derived independently rather than trusted, per instruction ("mine
have been wrong four times this sweep"): `python3
tools/ci/count-coarse-assertions.py <path>`, filtered to exclude
`matches`/`is_err`/`is_none` (the old grammar), one run per path. Both
given counts checked out exactly on independent re-derivation —
no correction owed this round.

| Path | Given | Re-derived |
|---|---|---|
| `crates/moveit-smoothing` | 11 | 11 |
| `crates/moveit-kinematics` | 9 | 9 |
| `tools/moveit-diff` | 8 | 8 |
| `crates/moveit-sampling` | 1 | 1 |
| `crates/moveit-test-support` | 1 | 1 |
| `crates/moveit-constraints/tests/sampler.rs` | 4 | 4 |
| `crates/moveit-constraints/tests/utils_parity.rs` | 3 | 3 |
| **Total** | **37** | **37** |

### moveit-smoothing (11 sites)

| Site | Kind | Verdict | Evidence |
|---|---|---|---|
| `acceleration_filter.rs:466` (`assert!(`) | contains | in-family | unique substring vs. sibling single-DOF guard; test's own comment records a prior message-swap bite |
| `acceleration_filter.rs:525` (`assert!(`) | contains | in-family | `contains("planar_joint") && contains('3')` — only the single-DOF guard emits a bare digit; structurally unique |
| `acceleration_filter.rs:542` (`assert!(`) | contains | in-family, discriminating | `do_smoothing_rejects_a_length_mismatch`, added Round 6 — the guard this site tests (`:329`) had zero coverage anywhere in the workspace before this round; bite-verified (neutralizing `:329` alone fails only this test, falling through to the sibling `:335` guard's distinct message) |
| `acceleration_filter.rs:565` (`assert!(`) | contains | in-family | unique substring vs. `do_smoothing`'s other (non-folded) guard. Line renumbered from a stale `:542`, shifted +23 by Round 6's insertion above it |
| `butterworth.rs:153` (`assert!(err.to_string().contains("unstable"), "{err}");`) | contains | in-family | unique substring ("unstable") vs. 3 sibling `Error::construct` sites; comment records a prior message-swap bite against each |
| `butterworth.rs:162` (`assert!(err.to_string().contains("scale_term_"), "{err}");`) | contains | in-family | "scale_term_" unique vs. "infinite feedback_term_"/"...unstable"/"...feedback term of 0" |
| `butterworth.rs:172` (`assert!(`) | contains | in-family | boundary case `coeff == 1.0` exactly on the EPSILON guard |
| `butterworth.rs:183` (`assert!(`) | contains | in-family | distinct boundary (`coeff == 1 + 1e-10`) of the *same* branch as 172 — two boundary tests of one discriminating branch, not a duplicate |
| `butterworth.rs:200` (`assert!(err.to_string().contains("feedback_term_"), "{err}");`) | contains | in-family | "feedback_term_" (underscored) is textually disjoint from site 4's "feedback term" (spaced) |
| `ruckig_filter.rs:388` (`assert!(`) | contains | in-family | unique substring vs. 3 sibling guards; comment records a prior message-swap bite |
| `ruckig_filter.rs:530` (`assert!(`) | contains | in-family | same name+digit pattern as `acceleration_filter.rs:525` (`assert!(`), same reasoning — renumbered from a stale ruckig_filter.rs line 465; this is `multi_dof_active_joint_is_a_typed_error_not_a_silent_last_variable_wins`, given a full discriminating-verdict row under Round 5 below |
| `ruckig_filter.rs:613` (`assert!(err.to_string().contains("must each have length"), "{err}");`) (`do_smoothing`'s length guard) | contains | **BLIND — fixed** | folded 3-clause OR guard, structurally identical to `reset`'s (fixed in Task 1, `2829ca2`) but never itself isolated — see below. Line renumbered from a stale ruckig_filter.rs line 539, then `:604` (+9, Round 6's `ruckig_filter.rs:289` (`.map_err(|error| Error::other(format!("ruckig update failed: {error}")))?;`) comment fix) |
| `ruckig_filter.rs:626` (`assert!(err.to_string().contains("must each have length"), "{err}");`) | contains | in-family, discriminating | bite-verified fix for the row above (`do_smoothing_rejects_a_positions_only_mismatch`); own row added this round. Renumbered from a stale `:617`, same +9 shift |
| `ruckig_filter.rs:639` (`assert!(err.to_string().contains("must each have length"), "{err}");`) | contains | in-family, discriminating | bite-verified fix (`do_smoothing_rejects_a_velocities_only_mismatch`); own row added this round. Renumbered from a stale `:630`, same +9 shift |
| `ruckig_filter.rs:652` (`assert!(err.to_string().contains("must each have length"), "{err}");`) | contains | in-family, discriminating | bite-verified fix (`do_smoothing_rejects_an_accelerations_only_mismatch`); own row added this round. Renumbered from a stale `:643`, same +9 shift |

**Fix**: `do_smoothing`'s guard (`positions.len() != num_joints ||
velocities.len() != num_joints || accelerations.len() != num_joints`)
had exactly one test, which broke all three lengths at once. Bite
(remove the `positions` clause) left that test green — confirmed live,
reverted. Added `do_smoothing_rejects_a_{positions,velocities,an_
accelerations}_only_mismatch`, mirroring `reset`'s existing three.
Bit each clause individually (`--no-fail-fast`): each bite failed only
its own new test, left the other two and `reset`'s siblings green.
Commit `b2b5e86`. Gated `-p moveit-smoothing`: `cargo fmt --all`
(clean), `cargo clippy -p moveit-smoothing --all-targets -- -D
warnings` (clean), `cargo nextest run -p moveit-smoothing` (39/39
passed).

### moveit-kinematics (9 sites)

| Site | Kind | Verdict | Evidence |
|---|---|---|---|
| `cart_to_jnt.rs:550` (`assert!(`) | is_some | in-family | sole zero-distance convergence check; default-options regression, doc-scoped |
| `cart_to_jnt.rs:644` (`assert!(`) | is_some | in-family | paired in the same test with an `is_none()` tight-limit case (line 667) — both branches of the consistency gate exercised |
| `cart_to_jnt.rs:707` (`assert!(`) | is_some | in-family | paired in the same test with an `is_none()` always-rejecting-callback case (line 738), plus call-count assertions on both |
| `chain.rs:469` (`assert!(err.to_string().contains("not a chain"), "got: {err}");`) | contains | in-family | "not a chain" unique vs. 3 sibling `Error::other` sites in `build` |
| `chain.rs:512` (`assert!(err.to_string().contains("DOF"), "got: {err}");`) | contains | in-family | "DOF" unique vs. siblings |
| `chain.rs:558` (`assert!(`) | contains | in-family | "not itself in the group" unique vs. siblings |
| `chain.rs:676` (`assert_eq!(chain.root_link_index, None);`) (`root_link_index == None`) | eq_none | in-family — confirmed by live bite | see below |
| `crates/moveit-kinematics/src/registry.rs:271` (`assert!(names.contains(expected), "missing registration: {expected}");`) | contains | in-family | set-membership loop, one descriptive message per expected name |
| `ik_fk_roundtrip.rs:281` (`assert!(err.to_string().contains("not a chain"), "got: {err}");`) | contains | in-family | same "not a chain" text as `chain.rs:469` (`assert!(err.to_string().contains("not a chain"), "got: {err}");`), one layer up through `NewtonRaphsonSolver::new` |

**`chain.rs:676` bite**: this is the only direct assertion on
`root_link_index`, and the one same-crate test that reaches the
`Some(...)` branch textually (`base_frame_and_tip_frame_resolve_to_
the_chain_endpoint_link_names`) uses a fixture where the chain's
computed root link happens to *equal* the model's own root link name —
so `Some(idx-of-"root")` and the `None`-fallback both print `"root"`,
and that test cannot by itself tell the branches apart. Bit
`root_link_index` to unconditionally `None` and ran the full crate:
`cargo nextest run -p moveit-kinematics --no-fail-fast chain::` stayed
7/7 green (confirming the local-file blind spot), but the *full-crate*
run failed 2 tests — `pr2_right_arm_continuous_joints_round_trip` and
`pr2_gripper_mimic_chain_round_trips` in `tests/ik_fk_roundtrip.rs`,
both of which use pr2 fixtures where the chain's root joint sits
mid-tree. Reverted (`diff` confirmed clean). Verdict: in-family — the
`Some` branch is discriminated by IK/FK roundtrip integration tests
outside this file, not by any single assertion in-file, but the
census's three-clause test is about whether *something* in the suite
discriminates the decision, not whether the cited assertion does alone.
No fix — nothing here is actually blind.

No fix needed for moveit-kinematics.

### moveit-sampling (1 site) / moveit-test-support (1 site)

| Site | Kind | Verdict | Evidence |
|---|---|---|---|
| `multivariate_gaussian.rs:213` (`positive_definite_covariance_constructs`) | is_some | in-family | the sole `is_some` case in a suite of 5 boundary tests, each a distinct negative (`is_none`) case: mismatched dims, non-square, indefinite, zero/PSD-not-PD |
| `moveit-test-support/src/lib.rs:88` (`assert!(`) (`assert_group_has_updated_links`) | is_empty | **not-this-family** (corrected below) | fixture-precondition helper called before the calling crate's real subject; see the clause-3 re-audit table |

### tools/moveit-diff (20 sites)

| Site | Kind | Verdict | Evidence |
|---|---|---|---|
| `main.rs:3834` (`assert!(`) | is_empty | **not-this-family** (corrected below) | its own message says "for this diagnostic to mean anything" — a precondition on `parry_representable_link_names`, not on the collision decision this test pins; see the clause-3 re-audit table |
| `main.rs:3922` (`assert!(`) | is_empty | in-family | the pinned regression itself; paired one line above with an explicit `touched > 0` per-link guard against exactly the vacuous-pass failure mode the doc comment names |
| `main.rs:3533` (`assert!(stats.divergent.is_empty());`) | is_empty | in-family | subject is `compare_ik`: `stats` goes in `&mut` and is read back immediately, and `divergent` is a field only that call pushes to. Emptiness alone would be vacuous (deleting the call leaves `IkStats::default()`, also empty), so the row rests on the mutation instead — forcing `solved_by` to `Some("rust")` on the `(true, true) | (false, false)` arm fails this line and no other, while the two sibling tests pin the same field's opposite outcome from the same call shape |
| `harness.rs:70` (`assert!(`) | contains | in-family | unique stdout line |
| `harness.rs:74` (`assert!(`) | contains | in-family | unique stdout line |
| `harness.rs:95` (`assert!(`) | contains | in-family | secondary corroboration; primary discriminator is the paired `assert_eq!(status.code(), Some(1))` in the same test |
| `harness.rs:113` (`assert!(`) | contains | in-family | same text as 95, different invocation (`--stats-json`) — the test's real check is the JSON body that follows; this is a stdout-not-corrupted sanity check |
| `harness.rs:154` (`assert!(`) | contains | **not-this-family** (corrected below) | asserts on `fake-oracle.py`'s own file text, read by the test via `std::fs::read_to_string`; no crate code runs before it — see the clause-3 re-audit table |
| `harness.rs:160` (`assert!(`) | contains | **not-this-family** (corrected below) | same file read, positive-control sibling of 154 |
| `main.rs:4150` (`a_one_ulp_limit_change_reddens_only_joint_limits`) | any / is_some | **not-this-family** | the scanner token sits in the *mutation closure* — `find(...any(\|(lo, _)\| lo.is_some()))` picks which bound to perturb — not in the assertion's predicate. The assertion is `assert_eq!(failing_clauses(...), vec!["joint_limits"])`, an exact vector equality naming the one clause that must redden and, by exclusion, the four that must not |
| `main.rs:4173` (`a_flipped_position_bounded_reddens_only_joint_limits`) | is_empty | **not-this-family** | same shape: `!j.position_bounded.is_empty()` selects the joint to perturb; the assertion itself is the exact `assert_eq!(..., vec!["joint_limits"])` |
| `main.rs:4192` (`an_added_mimic_relation_reddens_only_mimic`) | all / is_none | **not-this-family** | same shape; the assertion is the exact `assert_eq!(..., vec!["mimic"])` |
| `main.rs:4194` (`assert!(`) (inside the same test) | all / is_none | **not-this-family** (fixture precondition) | `m.joint_details.iter().all(\|j\| j.mimic.is_none())` pins that prbt genuinely has no mimic joint. Same category as `main.rs:3834` (`assert!(`) — a precondition, not the subject decision — but here for the inverse reason: prbt's live `mimic` clause compares an empty set against an empty set, so "mimic agrees" carries no information by itself. This line establishes that the emptiness is real, and the enclosing `assert_eq!` then shows the clause still reddens when a relation is added |

| `main.rs:4249` (`an_unresolved_collision_mesh_is_refused`) | contains | in-family | asserts the refusal names a link. The subject is `reject_dropped_collision_meshes`, whose whole value is the *content* of what it says -- the behaviour it replaces already stopped the run in one sense (it produced a wrong table) and the defect was that nothing named a cause. Deleting the `format!` of the diagnostic list leaves this red |
| `main.rs:4250` (`assert!(`) (same test) | contains | in-family | asserts the refusal names the directory searched. Paired with `main.rs:4249` (`assert!(err.contains("base_link"), "must name a link:\n{err}");`) rather than redundant with it: a message naming the link but not the search root does not tell a reader that `third_party/` is what is missing, which is the actual remedy |
| `main.rs:4254` (`assert!(err.contains('7'), "must count the dropped elements:\n{err}");`) (same test) | contains | in-family | asserts the count (7 fanuc collision meshes). Discriminates a refusal that fired on one element from one that enumerated all of them |
| `main.rs:4284` (`a_robot_with_no_collision_mesh_still_runs_without_any_search_path`) | any / is_none | **not-this-family** (fixture precondition) | pins that prbt genuinely has no `<mesh>` collision element, which is what makes the `Ok(())` on the next line mean "no meshes to lose" rather than "meshes lost but tolerated". Same category as `main.rs:3834` (`assert!(`) |
| `main.rs:3072` (`the_default_oracle_seed_collides_with_the_matching_case_seed`) | contains | in-family | `reject_colliding_oracle_streams` has exactly one `Err` site and one `Ok` site (`main.rs:82-96`), so the enclosing `expect_err` is single-branch and pins reachability only. This line is the discriminating half: it asserts the refusal *names the stream collision*, which is the whole reason the guard exists — a refusal that fired for some other reason would leave a caller re-running with a differently wrong seed. Replacing the message body while keeping the `Err` leaves the `expect_err` green and reddens this |
| `main.rs:3102` (`a_negative_case_seed_is_judged_as_the_stream_it_selects`) | is_err | single-branch | structural: one `Err` site, so `.is_err()` alone cannot name a branch. It is not blind because of the `.is_ok()` on `(-1, 1)` eight lines below: the pair is what pins the `seed as u32` reinterpretation. A guard written `seed == oracle as i32` passes the `.is_ok()` and fails this line; one that waved negatives through entirely fails this line and passes the other |
| `main.rs:3119` (`zero_is_an_ordinary_seed_on_both_sides`) | is_err | single-branch | same single-`Err` structure, same paired shape: `(0, 0)` must refuse and `(0, 42)` must not. What the pair excludes is a guard that treats `0` as "unset" and skips the comparison — that variant passes the `.is_ok()` and fails this line |

No fix needed for tools/moveit-diff. Of the eight sites added the round
before, three are exact `assert_eq!`s whose scanner token lives in the
perturbation closure rather than the predicate, three assert the *content*
of a refusal message (its whole purpose), and two are preconditions
labelled as such. The three seed-collision sites added with the Phase 4 (a)
fix are the paired shape their rows describe: one `Err` site means the
`is_err` half is structural, and the discrimination lives in the `is_ok`
sibling asserted in the same test.

### 7 stranded sites (constraints test files)

| Site | Kind | Verdict | Evidence |
|---|---|---|---|
| `crates/moveit-constraints/tests/sampler.rs:78` (`assert!(`) | contains | in-family | "panda_joint1" unique vs. sibling `Error::other` site (which embeds `group_name`, not the joint variable name) |
| `crates/moveit-constraints/tests/sampler.rs:120` (`assert!(`) | contains | in-family | "panda_arm" unique vs. sibling (which embeds joint variable name, not group name); input validity also rules out the `UnknownName` path |
| `crates/moveit-constraints/tests/sampler.rs:194` (`assert!(`) | contains (range) | in-family | per-iteration bound check, single documented production path |
| `crates/moveit-constraints/tests/sampler.rs:200` (`assert!(`) | contains (range) | in-family | same, tightened window |
| `utils_parity.rs:581` (`assert!(set.is_empty());`) | is_empty | in-family | paired with `assert!(!updated)` in the same test — two views of the same not-found branch. Renumbered from a stale `utils_parity.rs:580`; superseded by the census §9 pass below, which reclassifies this `not-this-family` (clause 2 — empty-fixture exclusion) |
| `utils_parity.rs:647` (`assert!(set.is_empty());`) | is_empty | in-family | same pattern, position-constraint sibling. Renumbered from a stale utils_parity.rs line 602; same census §9 supersession as the row above |
| `utils_parity.rs:786` (`assert!(merged.is_empty());`) | is_empty | in-family | one of three distinctly-asserted branches (`merge`→intersect/drop/keep), each with its own test and its own specific check. Renumbered from a stale utils_parity.rs line 698 |

No fix needed for the 7 stranded sites.

### Round 3 clause-3 re-audit (operational test, all 37)

The original pass above used "explicit anti-vacuous-pass guards already
present" as a reason to keep sites in-family. That reasoning is inverted:
an anti-vacuous guard is itself a precondition on the fixture, which is
exactly the shape census §9 clause 3 excludes. Re-applied clause 3's own
operational test to all 37 sites, not the four already flagged: *identify
the function this TEST's own name/doc claims to verify (its subject);
then ask — if you deleted the call to that specific function, would this
assertion's outcome be unaffected?* If yes, not-this-family.

Two settled precedents from `doc/assertion-discrimination-census.md` §9
anchor this pass:

- **The `fs::read`/`trajectory.group()` shape (not-this-family):** the
  checked value comes from a call the test performs directly (a std
  function, or a getter on an object built by a *different* function than
  the one the test's own name names as subject), with no subject-side
  decision in between.
- **The `mimic().is_none()` shape (in-family):** a getter reading a field
  that the subject's *own* call, invoked earlier in this same test,
  computed or mutated — "a getter reading a field the subject mutated is
  exactly as much a member as a guard the subject evaluates directly."

| Site | Test's own subject | Delete-the-call test | Verdict |
|---|---|---|---|
| `acceleration_filter.rs:466` (`assert!(`) | `joint_acceleration_bounds` | `err` is `joint_acceleration_bounds(...).unwrap_err()` directly; delete that call and the assertion has nothing to inspect | in-family |
| `acceleration_filter.rs:525` (`assert!(`) | `joint_acceleration_bounds` | same shape — `message` comes straight from `joint_acceleration_bounds(...).unwrap_err()` | in-family |
| `acceleration_filter.rs:542` (`assert!(`) | `do_smoothing` | `err` is `filter.do_smoothing(...).unwrap_err()` directly; added Round 6, previously uncovered | in-family |
| `acceleration_filter.rs:565` (`assert!(`) | `do_smoothing` | same shape, renumbered from a stale `:542` (shifted +23 by Round 6's insertion above it) | in-family |
| `butterworth.rs:153` (`assert!(err.to_string().contains("unstable"), "{err}");`) | `ButterworthFilter::new` | `new` is simultaneously the construction *and* the decision (no separate arrange-function to defer to); `err` is its direct return | in-family |
| `butterworth.rs:162` (`assert!(err.to_string().contains("scale_term_"), "{err}");`) | `ButterworthFilter::new` | same | in-family |
| `butterworth.rs:172` (`assert!(`) | `ButterworthFilter::new` | same | in-family |
| `butterworth.rs:183` (`assert!(`) | `ButterworthFilter::new` | same (distinct boundary of the same branch as 172) | in-family |
| `butterworth.rs:200` (`assert!(err.to_string().contains("feedback_term_"), "{err}");`) | `ButterworthFilter::new` | same | in-family |
| `ruckig_filter.rs:388` (`assert!(`) | `joint_vel_accel_jerk_bounds` | `err` is its direct return | in-family |
| `ruckig_filter.rs:530` (`assert!(`) | `joint_vel_accel_jerk_bounds` | same | in-family (renumbered from a stale ruckig_filter.rs line 465) |
| `ruckig_filter.rs:613,626,639,652` (`assert!(err.to_string().contains("must each have length"), "{err}");`) (`do_smoothing`'s guard, incl. this round's 3 new tests) | `do_smoothing` | `err` is its direct return | in-family (renumbered from a stale ruckig_filter.rs lines 539, 552, 565, 578, then `:604,617,630,643`, +9, Round 6's `ruckig_filter.rs:289` (`.map_err(|error| Error::other(format!("ruckig update failed: {error}")))?;`) comment fix) |
| `cart_to_jnt.rs:550` (`assert!(`) | `search_position_ik` | `solution` is its direct return; the trivial seed==target fixture still exercises `search_position_ik`'s own written tolerance comparison (unlike the census's `shortest_solution`-on-empty-input clause-2 failure, where the comparison never runs at all) | in-family |
| `cart_to_jnt.rs:644` (`assert!(`) | `search_position_ik` | `solution` is its direct return, paired in the same test with an `is_none()` tight-limit case exercising the same guard's other branch | in-family |
| `cart_to_jnt.rs:707` (`assert!(`) | `search_position_ik` | `solution` is its direct return, paired with an `is_none()` always-rejecting-callback case and call-count assertions on both | in-family |
| `chain.rs:469` (`assert!(err.to_string().contains("not a chain"), "got: {err}");`) | `ChainInfo::build` | `err` is its direct return | in-family |
| `chain.rs:512` (`assert!(err.to_string().contains("DOF"), "got: {err}");`) | `ChainInfo::build` | `err` is its direct return | in-family |
| `chain.rs:558` (`assert!(`) | `ChainInfo::build` | `err` is its direct return | in-family |
| `chain.rs:676` | `ChainInfo::build` | `chain.root_link_index` is a field `build` itself computes and *also* uses, in the same match expression, to derive `base_frame` — there is no separate "arrange" function here the way `RobotTrajectory::new` is separate from `apply_smoothing`; `build` is both construction and decision in one call. Confirmed further by this round's live cross-crate bite (forcing `root_link_index` to always `None` failed `ik_fk_roundtrip.rs`'s pr2 tests) | in-family |
| `crates/moveit-kinematics/src/registry.rs:271` (`assert!(names.contains(expected), "missing registration: {expected}");`) | the crate's own solver registry (`KINEMATICS_SOLVERS`) | no function call to delete — the "subject" is each solver module's own `#[distributed_slice(KINEMATICS_SOLVERS)]` declaration; deleting one (e.g. `lma`'s) directly flips this assertion. Closest call in this population: it is a static aggregate, not a runtime branch, but per this crate's own documented history (`distributed_slice ordering is not a contract` — a dependency-graph change once silently flipped which solver `pilz` resolved), membership here is genuine, non-tautological production behavior a change could break | in-family, argued rather than assumed |
| `ik_fk_roundtrip.rs:281` (`assert!(err.to_string().contains("not a chain"), "got: {err}");`) | `NewtonRaphsonSolver::new` (which itself calls `ChainInfo::build`) | `err` is its direct return, one layer up | in-family |
| `multivariate_gaussian.rs:213` | `MultivariateGaussian::new` | checked inline on the constructor's own return, no intermediate object | in-family |
| `moveit-test-support/src/lib.rs:88` (`assert!(`) | *(the calling crate's actual subject — this function is a shared fixture-precondition helper, not itself a decision under test)* | `assert_group_has_updated_links` is called by *other* crates' fixture builders, before those crates' own subject call. Deleting the call to whatever the calling test's real subject is (e.g. `generate_distance_field_cache_entry`) leaves this assertion's outcome completely unaffected — it depends only on the URDF/SRDF fixture's static joint configuration. Same shape as `crates/moveit-trajectory/tests/ruckig_smoothing.rs:203`'s `trajectory.group().is_none()`, just packaged as a shared helper instead of an inline check | **not-this-family** (moved) |
| `main.rs:3834` (`assert!(`) | the collision/near-placement decision this test pins (`decide_cone`'s tie-break, checked at `main.rs:3922` (`assert!(`)) | `eligible.is_empty()`'s own message says it outright: "for this diagnostic to mean anything." `eligible` comes from `parry_representable_link_names(&model)`, not from the collision-checking loop below. Deleting the call to the actual subject (`env.check_robot_collision`, the loop that produces `ambiguous`) leaves this assertion completely unaffected | **not-this-family** (moved) |
| `main.rs:3922` (`assert!(`) | `env.check_robot_collision` / `decide_cone`'s tie-break | `ambiguous` is built from `touched_link_counts`, populated once per link by calling `env.check_robot_collision(...)` inside the loop — a genuine per-call subject decision, not a value the test constructed itself. Deleting that call empties `touched_link_counts` and changes this assertion | in-family |
| `harness.rs:70` (`assert!(`) | the `moveit-diff` runner binary itself (this whole test file's subject) | `stdout` is the captured output of actually executing `CARGO_BIN_EXE_moveit-diff`; deleting that `Command::output()` call removes `stdout` entirely | in-family |
| `harness.rs:74` (`assert!(`) | same | same | in-family |
| `harness.rs:95` (`assert!(`) | same | `stdout`/exit code both come from running the binary; this line is a secondary corroboration of the paired `assert_eq!(status.code(), Some(1))` in the same test, not a precondition for it | in-family |
| `harness.rs:113` (`assert!(`) | same | same output, different invocation (`--stats-json`); confirms the flag doesn't corrupt the human-readable summary the JSON assertions that follow depend on being unaffected | in-family |
| `harness.rs:154` (`assert!(`) | *(no crate subject runs in this test at all)* | `fake` is `std::fs::read_to_string("fake-oracle.py")`, called by the test itself. No `moveit-diff` code runs before this assertion — the exact `fs::read` shape census §9 names verbatim | **not-this-family** (moved) |
| `harness.rs:160` (`assert!(`) | same | same — positive-control sibling of 154, same file read | **not-this-family** (moved) |
| `crates/moveit-constraints/tests/sampler.rs:78` (`assert!(`) | `JointConstraintSampler::new` | `err` is its direct return | in-family |
| `crates/moveit-constraints/tests/sampler.rs:120` (`assert!(`) | `JointConstraintSampler::new` | `err` is its direct return | in-family |
| `crates/moveit-constraints/tests/sampler.rs:194` (`assert!(`) | `JointConstraintSampler::sample` | `v` is `state.variable_position(name)`, read back after `sampler.sample(&mut state, &mut rng)` wrote it this same iteration — the `mimic().is_none()` shape exactly: a getter on state the subject just mutated | in-family |
| `crates/moveit-constraints/tests/sampler.rs:200` (`assert!(`) | `JointConstraintSampler::sample` | same shape, same iteration | in-family |
| `utils_parity.rs:581` | `update_orientation_constraint` | `set` is passed `&mut` into `update_orientation_constraint`; `set.is_empty()` reads back whether that call pushed into it — the subject's own side effect, not a value the test constructed independently | in-family (renumbered from a stale utils_parity.rs line 580) |
| `utils_parity.rs:647` (`assert!(set.is_empty());`) | `update_position_constraint` | same shape | in-family (renumbered from a stale utils_parity.rs line 602) |
| `utils_parity.rs:786` (`assert!(merged.is_empty());`) | `merge_constraints` | `merged` is its direct return | in-family (renumbered from a stale utils_parity.rs line 698) |

### Round 3 summary (corrected)

37 sites re-derived and re-audited against clause 3's operational test.
**33 in-family, 4 not-this-family** — a correction from the original
pass's 37/0, which used "an anti-vacuous guard is present" as a reason to
keep a site in-family; that reasoning had the clause backwards. The four
moved:

- `moveit-test-support/src/lib.rs:88` (`assert!(`) — fixture-precondition helper,
  same shape as the census's own `crates/moveit-trajectory/tests/ruckig_smoothing.rs:203` precedent.
- `tools/moveit-diff/src/main.rs:3834` (`assert!(`) — its own message names it a
  precondition ("for this diagnostic to mean anything").
- `tools/moveit-diff/tests/harness.rs:154,160` (`assert!(`) — assert on
  `fake-oracle.py`'s file text, read by the test itself; no crate code
  runs before either assertion, the census's `fs::read` shape verbatim.

The remaining 33 survive because, in this population, the flagged
assertion's value overwhelmingly comes either directly from unwrapping
the named subject's own `Result`/`Option` return (the majority shape —
`X::new(...).unwrap_err()` or `subject_fn(...).unwrap_err()`, where
construction and decision are the same call, so there is no separate
arrange-function to defer to), or from a getter reading state the
subject's own call just mutated in the same test (the `mimic().is_none()`
precedent — `sampler.rs:194/200`, `utils_parity.rs:580/602`). One site,
`crates/moveit-kinematics/src/registry.rs:271` (`assert!(names.contains(expected), "missing registration: {expected}");`), is argued rather than assumed: it has no function call
to delete, but its "subject" (each solver's own `#[distributed_slice]`
registration) is genuine, breakable production behavior, not a
tautological restatement of source text. `chain.rs:676` (`assert_eq!(chain.root_link_index, None);`) is confirmed
doubly: the operational test alone puts it in-family (`build` is the only
call, and it's simultaneously construction and decision), and this
round's live cross-crate bite independently confirmed the `Some(...)`
branch is discriminated, just not by any single in-file assertion.

No verdict change implies a source fix — the four moved sites were never
claimed blind, they are removed from the family entirely, so there is
nothing to isolate. **Corrected in-family count for this round: 33.**

One genuine blind site was found and fixed independently of this
re-audit: `ruckig_filter.rs`'s `do_smoothing` folded OR-guard (commit
`b2b5e86`), in-family under both the original and corrected reasoning.

## Round 4: funnel-bite audit (census §9g)

§9g's finding on `MeshSearchPaths::resolve` is distinct from clause 3: a
test can be in-family (its assertion's value really does depend on the
subject call) and still be *blind*, if the subject routes two or more
guards into one undifferentiated `None`/`Err` and the test's fixture
trips an earlier guard than the one its assertion claims to target.
Reading the source cannot catch this — only biting each guard (neutralize
it, confirm its own test fails while every sibling test on the same
subject stays green, `--no-fail-fast`) can. Two traps this round's bites
were checked against: (1) a mutation that cannot change any observable
outcome proves nothing regardless of the result (the user's own
`split_once('/')?` → `.unwrap_or((rest, ""))` example); (2) `is_some`
checks are structurally exempt — a positive result requires *every* guard
to pass, so there is no "which guard produced this same positive signal"
ambiguity the way multiple guards can share one negative signal.

### Candidates identified and their guard counts

| Subject | Guard/Err sites | In this audit? |
|---|---|---|
| `JointConstraintSampler::new` | 2 (`Err::other` × 2) | bit, §Bites below |
| `ChainInfo::build` | 5 (`?` group lookup, not-a-chain, DOF≠1, unsupported type — untested, mimic-master-outside-group) | ~~3 tested guards bit~~ **all 5 accounted for — see Round 6's `Error::UnknownName` note below** |
| `update_orientation_constraint` / `update_position_constraint` | 1 guard each (link-name match), but funnel through an **empty-loop vacuity**, not a message collision | bit — **blind, fixed** |
| `merge_constraints` | 1 drop path for this fixture shape | excluded, see below |
| `joint_acceleration_bounds` | 2 (`Err::other` × 2) | bit |
| `AccelerationLimitedFilter::do_smoothing` (2-arg) | 2 (`Err::other` × 2) | 1 bit directly, 1 already message-swap bite-checked (spot-confirmed by the sibling bite) |
| `ButterworthFilter::new` | 4 (`Err::construct` × 4) | ~~1 bit directly (spot-check per instruction)~~ **all 4 bit — see Round 6** |
| `JointConstraintSampler::sample` (`crates/moveit-constraints/tests/sampler.rs:194,200` (`assert!(`)) | not a `None`/`Err` funnel — numeric range check on subject-mutated state (`mimic().is_none()` shape) | excluded |
| `cart_to_jnt.rs:550,644,707` (`assert!(`), `multivariate_gaussian.rs:213` (`assert!(MultivariateGaussian::new(mean, covariance).is_some());`) | `is_some` positive checks | excluded (structural exemption above) |
| `crates/moveit-kinematics/src/registry.rs:271` (`assert!(names.contains(expected), "missing registration: {expected}");`) | static `#[distributed_slice]` aggregate, no `?`-chain | excluded |
| `harness.rs:70,74,95,113` (`assert!(`) | integration tests already execute the real `moveit-diff` binary end-to-end — no separate read-vs-run gap | excluded |
| `ruckig_filter.rs::joint_vel_accel_jerk_bounds` | 2 (`Err::other` × 2) | not independently re-bit this round — same file, same annotated-and-confirmed pattern as the sibling `joint_acceleration_bounds` bites, itself spot-checked |

### Bites performed and results

All bites used `&& !true` on the guard condition (or, for the one
`let...else` guard, `.or(Some(&0))` on the lookup) to keep both operands
referenced under `-D warnings`, confirmed via
`cargo nextest run -p <crate> --no-fail-fast`, then reverted via a
pre-bite backup + `diff` before moving to the next site.

- **`JointConstraintSampler::new`** (`crates/moveit-constraints/src/sampler.rs:213` (`/// upstream's "there are no possible values for the joint" — a`),`:224`): bit each of the two `Err::other` guards (empty-intersection, no-valid-constraint-for-group) independently. Each bite failed exactly the test targeting it (`configure_fails_on_empty_intersection_between_two_constraints`, `configure_fails_when_the_only_constraint_is_on_a_joint_outside_the_group`) while the other stayed green. **Discriminating, not blind.**
- **`ChainInfo::build`** (`crates/moveit-kinematics/src/chain.rs:147` (`if !group.is_chain() {`),`:185`,`:259`): bit the not-a-chain, DOF≠1, and mimic-master-outside-group guards independently. Each bite failed exactly its own unit test (`build_rejects_a_non_chain_group`, `build_rejects_a_multi_dof_joint`, `build_rejects_an_in_chain_mimic_whose_master_is_outside_the_group`) while the others stayed green; the DOF bite additionally showed the fixture falls through to the *next* guard (unsupported-type) under a still-different message, so `contains("DOF")` remains a real discriminator rather than an accidental pass. Also re-bit the not-a-chain guard through the cross-crate integration test `crates/moveit-kinematics/tests/ik_fk_roundtrip.rs:281` (`assert!(err.to_string().contains("not a chain"), "got: {err}");`)'s `constructing_a_solver_on_a_non_chain_group_is_an_error` (via `NewtonRaphsonSolver::new` → `ChainInfo::build`, its only fallible call) — only that test and its unit-test sibling failed, all 33 other kinematics tests stayed green. **Discriminating, not blind.** The untested unsupported-type guard (`chain.rs:196` (`return Err(Error::other(format!(`)) has no assertion at all, so it is not a census site and is out of this audit's scope — noted, not fixed. The fifth guard, the `?` group lookup (`chain.rs:146` (`let group = model.joint_model_group(group_name)?;`)), is exempt from needing a bite at all — see Round 6, below.
- **`update_orientation_constraint` / `update_position_constraint`** (`crates/moveit-constraints/src/utils.rs:516` (`if oc.link_name() != link_name {`),`:597`): bit each link-name-comparison guard completely. Both `not_found_returns_false` tests (`utils_parity.rs:580`,`:602`) **stayed green** — confirmed blind. Root cause: both fixtures construct an *empty* `KinematicConstraintSet`, so `for c in constraints.constraints_mut()` never iterates and `set.is_empty()` holds regardless of what the guard decides — the census's own `shortest_solution_is_none_on_empty_input` vacuous-fixture shape. **Fixed** (commit `9b2bff6`): added `mismatched_link_name_leaves_constraint_untouched` to each boundary module, constructing one non-matching constraint so the loop body actually runs; re-biting the same guards against the new tests now fails only the new test in each module, with `not_found_returns_false` (and, for position, `multi_region_constraint_is_error`) staying green — confirmed the new tests could not have passed as a behaviour-preserving no-op, since the bite visibly flips `updated` from `false` to `true` and reconstructs the surviving constraint.
- **`joint_acceleration_bounds`** (`crates/moveit-smoothing/src/acceleration_filter.rs:153` (`if joint.variable_names().len() != 1 {`),`:162`): bit the single-DOF-active-joint guard directly — only `multi_dof_active_joint_is_a_typed_error_not_a_silent_last_variable_wins` failed, `joint_acceleration_bounds_fails_without_acceleration_limits` (its claimed message-swap sibling) stayed green, confirming the existing "message-swap bite-checked" comment.
- **`AccelerationLimitedFilter::do_smoothing`** (`acceleration_filter.rs:335` (`if self.last_positions.len() != positions.len() {`), the reset-before-check guard): bit directly — only `do_smoothing_before_reset_is_an_error` failed (via an index-out-of-bounds panic once the length check no longer short-circuits, still a discriminating failure), all 38 other smoothing tests stayed green.
- **`ButterworthFilter::new`** (`crates/moveit-smoothing/src/butterworth.rs:80`, the `coeff < 1.0` guard): bit directly — only `coefficient_below_one_is_rejected` failed; its three sibling tests (`coefficient_of_negative_one_makes_scale_term_infinite`, `coefficient_of_exactly_one_makes_feedback_term_zero`, `coefficient_of_infinity_makes_feedback_term_infinite`) all stayed green, confirming the existing "message-swap bite-checked against each of them" comment.

### Exclusions, with reasons on the record

- **`merge_constraints`** (`crates/moveit-constraints/src/utils.rs:147`, tested at `utils_parity.rs:786`): not a `None`/`Err`-funnel shape at all — the function has no fallible return. Its one `is_empty()` boundary test (`non_overlapping_windows_are_dropped`) uses a fixture with exactly one joint constraint per side on the same variable name, so `merged` ends up empty via exactly one internal drop path (`a.merged(b)` returning `None`) — there is no second guard that could produce the same empty result for this fixture shape, so there is nothing to disambiguate. Excluded as "single drop path," not bit.
- **`crates/moveit-constraints/tests/sampler.rs:194,200` (`assert!(`)**: `JointConstraintSampler::sample`'s two assertions read `state.variable_position(name)` back after `sampler.sample` wrote it in the same iteration — a getter on subject-mutated state (the `mimic().is_none()` shape from Round 3), not a guard-funnel. `sample` itself has no `None`/`Err` branch (its doc comment: "always succeeds"). Excluded, not bit.
- **`cart_to_jnt.rs:550,644,707` (`assert!(`), `multivariate_gaussian.rs:213`**: all `is_some()`/positive-result checks. Structurally exempt — see this section's opening paragraph. Excluded, not bit.
- **`crates/moveit-kinematics/src/registry.rs:271` (`assert!(names.contains(expected), "missing registration: {expected}");`)**: static `#[distributed_slice]` aggregate with no `?`-chain or sequential-guard structure to fold into a single signal. Excluded, not bit (also already argued-and-kept in-family for clause 3 in Round 3, a separate question).
- **`harness.rs:70,74,95,113` (`assert!(`)**: these integration tests spawn and run the real `moveit-diff` binary end-to-end and assert on its actual stdout — there is no separate "read the source vs. run the code" gap the way a static-source-read test has, so a funnel inside the binary's own internals would show up as a wrong assertion outcome, not a silently-passing one. Not independently bit this round (out of fence to modify `tools/moveit-diff/src/main.rs`'s internals beyond the two `main.rs` sites already in the ledger); reasoning recorded rather than assumed.
- **`ruckig_filter.rs::joint_vel_accel_jerk_bounds`**: same `Err::other` × 2 shape as `joint_acceleration_bounds`, in the same crate, carrying the same "message-swap bite-checked" comment convention. Given `joint_acceleration_bounds`'s identical-shaped bites (above) and `ButterworthFilter::new`'s bite both independently confirmed their own "message-swap bite-checked" claims this round, this site's claim is corroborated by pattern rather than independently re-bit — flagged here rather than silently trusted.

### Result

One new blind site found and fixed this round:
`update_orientation_constraint`/`update_position_constraint`'s
`not_found_returns_false` tests (`utils_parity.rs:580`,`:602`), commit
`9b2bff6`. Every other candidate subject with 2+ guards funneling into
one signal was bit and confirmed discriminating; every exclusion is
listed above with its reason.

## UNFIXED

- ~~`tools/moveit-diff/src/main.rs:3922` (`assert!(`)'s `near_placement_never_touches_more_than_one_link_at_once` is `#[ignore]`d and needs `third_party/moveit_resources`; that directory does not exist in this worktree...~~ **Closed by the merger.** The premise was wrong: `third_party/moveit_resources` exists and is populated in the primary checkout. It is untracked, so `git worktree` never materialises it — the absence is a property of every `caucus` worktree, not of this machine, and the `find /` that reported nothing was run from inside one. Run from `/home/stevek/work/moveit-rs`, `cargo nextest run -p moveit-diff --run-ignored all -E 'test(near_placement_never_touches_more_than_one_link_at_once)'` **passes**: 17 links checked, 17 with a near-placement touching ≥1 other link, 0 ambiguous. The diagnostic's conclusion therefore stands — `decide_cone`'s `max_contacts: 1` tie-break is ruled out as the source of the 115-case distance mismatch.
- ~~`ruckig_filter.rs::joint_vel_accel_jerk_bounds` and `ChainInfo::build`'s untested unsupported-joint-type guard (`chain.rs:196` (`return Err(Error::other(format!(`), no assertion exists) are not independently re-verified/covered — see the Exclusions and Bites sections above for why each was left as-is rather than bit or newly tested.~~ **Closed — see Round 5 below.** `chain.rs:193-200` (`joint.joint_type(),`) is proven unreachable by enumeration, not merely untested; `ruckig_filter.rs::joint_vel_accel_jerk_bounds` had two of its four guards (`velocity_bounded`, `jerk_bounded`) with zero test coverage, not merely "corroborated by pattern" as this line previously assumed — both are now tested and isolating-mutation-confirmed.

## Round 5: `ChainInfo::build`'s type guard and `joint_vel_accel_jerk_bounds`, re-examined under §3a

Both items above turned on a read, not a bite, and the read was wrong in
one case and incomplete in the other. This round replaces both reads with
enumeration plus live bites, `--no-fail-fast`, operands kept live
(`&& !true`), each mutation reverted and diffed clean before the next.

### `ChainInfo::build`'s type guard (`chain.rs:192-200` (`if !matches!(`)) is unreachable, not blind

`chain.rs:186-192` (DOF ≠ 1 → `Err`) and `:193-200` (neither `Revolute`
nor `Prismatic` → `Err`) are immediate siblings, both `Error::other`, the
funnel shape §9g asks about. No assertion in this crate exercises
`:193-200` — confirmed by `rg` for its message text
(`"unsupported type"`) across `crates/moveit-kinematics/{src,tests}`: the
only hit is the guard's own `format!` call.

That absence is not corrigible by writing a test, because no input can
reach the guard's `Err` arm:

1. `JointModel` is constructible only through
   `new_revolute`/`new_prismatic`/`new_planar`/`new_floating`/`new_fixed`
   (`moveit-model/src/joint/model.rs`) — no public struct literal, no
   setter that touches `kind` or `variable_names`' length exists outside
   that file (`rg` for `variable_names\s*=|\.push\(|\.pop\(|\.remove\(|\.truncate\(`
   in that file: zero hits outside the five constructors).
2. `joint_type()` (`crates/moveit-model/src/joint/model.rs:289-297` (`pub fn joint_type(&self) -> JointType {`)) is a pure, exhaustive match on
   `kind`, fixed 1:1 with whichever constructor built the value.
3. `variable_count()` is `variable_names.len()`, fixed at construction:
   `Revolute`→1, `Prismatic`→1, `Planar`→3, `Floating`→7, `Fixed`→0
   (`crates/moveit-model/src/joint/model.rs:164-260`, each constructor's own `bounds`/`locals` literal).
4. `chain.rs:180-201`'s loop only reaches `:192` after `:182-184` skips
   `Fixed` (`continue`) and `:185-191` returns `Err` for `variable_count()
   != 1` — eliminating `Planar` (3) and `Floating` (7). Only `Revolute`
   and `Prismatic` (both 1) survive to `:192`, and both satisfy `:193`'s
   `matches!` trivially.
5. `joint/urdf.rs:54-94` (the only URDF→`JointModel` builder in this
   workspace) routes every `UrdfJointType` variant through exactly those
   five constructors — no back door from a URDF file either.

So `variable_count() == 1` implies `joint_type() ∈ {Revolute, Prismatic}`
always, by construction, in this port. `:193-200`'s `Err` arm is dead
code from an input perspective — not an untested guard, an unreachable
one. Per this task's own instruction, no test is written for it: a test
that reaches `:193` cannot exist without breaking the enumeration above,
and forcing one (e.g. hand-constructing a `JointModel` with mismatched
`kind`/`variable_names` through no public API) would test a state this
port cannot produce.

The bite matrix confirms the enumeration rather than substituting for
it — a green result here is read as "consistent with unreachable," not
independently as "unreachable," since a behaviour-preserving mutation
also produces all-green (this session's own recorded trap).

| Guard neutralized | Mechanism | Expected if reachable | Actual | 35-test suite |
|---|---|---|---|---|
| `:193-200` (type) alone | `&& !true` appended to the `matches!`-negation | some test fails | **none fail** | 35/35 green |
| `:186-192` (DOF≠1) alone | `&& !true` appended to `!= 1` | `build_rejects_a_multi_dof_joint` fails | **fails**, all else green | 34/35 (1 expected fail) |

The second row is the control: it proves the harness and the mutation
technique can and do surface a real regression in this exact function,
so the first row's all-green is not an artifact of a broken bite — it is
the predicted result of an unreachable branch. Both mutations reverted;
`diff` against the pre-bite copy clean before the next step.

### `ruckig_filter.rs::joint_vel_accel_jerk_bounds` (`:128-170`) — two of four guards had zero coverage, now fixed

Four sibling `Error::other` guards, all funneling into the same error
type, exactly the shape flagged as "corroborated by pattern rather than
independently re-bit" in Round 4's exclusions:

- `:138-144` DOF ≠ 1 → `Err`
- `:147-151` `!velocity_bounded` → `Err`
- `:154-158` `!acceleration_bounded` → `Err`
- `:161-165` `!jerk_bounded` → `Err`

Re-checking against `main`'s current tip (post-merge, `#[test]` scan of
the whole file): `:138-144` and `:154-158` each had a dedicated test
(`multi_dof_active_joint_is_a_typed_error_not_a_silent_last_variable_wins`,
`joint_vel_accel_jerk_bounds_fails_without_acceleration_limits`).
`:147-151` and `:161-165` had **none** — `rg` for `"velocity limit
defined"` and `"jerk limit defined"` inside `crates/moveit-smoothing/`
matched only the guards' own `format!` calls, not any test. This was not
"corroborated by pattern," it was untested, and unlike the `chain.rs`
case above, both are reachable: `group.active_joint_names()`
(`robot_model.rs:1479-1501` (`joint_set.extend(self.expand_chain(base_idx, tip_idx));`)) admits any non-fixed, non-mimic joint
regardless of DOF, and `panda_arm`'s own URDF already has joints with
velocity bounds set and jerk bounds unset (the existing
`joint_vel_accel_jerk_bounds_fails_without_acceleration_limits` test's
own doc comment says as much) — clearing velocity, or setting
acceleration without jerk, are both ordinary fixture mutations, not
constructions this port makes impossible.

Added `joint_vel_accel_jerk_bounds_fails_without_velocity_limits`
(clears `has_velocity_limits` on every `panda_arm` joint via
`set_variable_bounds_from_limits`) and
`joint_vel_accel_jerk_bounds_fails_without_jerk_limits` (sets
acceleration bounds only, leaving `panda_arm`'s already-absent jerk
bounds untouched) to `ruckig_filter.rs`'s test module.

Four-way isolating-mutation bite, each guard neutralized alone
(`&& !true`), `--no-fail-fast`, reverted and diffed clean before the
next:

| Guard neutralized | Own test | Sibling guards' tests | Full suite |
|---|---|---|---|
| `:138-144` DOF≠1 | `multi_dof_active_joint_is_a_typed_error_not_a_silent_last_variable_wins` **fails** | 3 others green | 40/41 |
| `:147-151` velocity | `joint_vel_accel_jerk_bounds_fails_without_velocity_limits` **fails** | 3 others green | 40/41 |
| `:154-158` acceleration | `joint_vel_accel_jerk_bounds_fails_without_acceleration_limits` **fails** (falls through to the jerk guard's message, doesn't match `"acceleration limit defined"`) | 3 others green | 40/41 |
| `:161-165` jerk | `joint_vel_accel_jerk_bounds_fails_without_jerk_limits` **fails**, plus `ruckig_filter_parity::ruckig_filter_matches_the_oracle` **fails** (`"case 0: expected initialize to fail"` — an oracle fixture case independently depends on this guard) | 3 others green | 39/41 |

All four guards are independently discriminating: each mutation kills
exactly its own test (jerk also kills one oracle-parity case, a second,
independent witness for the same guard — not a sibling collision) while
the other three guards' tests, and the success-path test, stay green.
Gate: `cargo fmt --all`; `cargo clippy -p moveit-kinematics --all-targets
-- -D warnings` and `cargo clippy -p moveit-smoothing --all-targets --
-D warnings`, both clean; `cargo nextest run -p moveit-kinematics`
(35/35) and `cargo nextest run -p moveit-smoothing` (41/41), both green.

**Row-count correction (orphan reconciliation, this round):** the bite
matrix above cites the four guards' own source lines (`:138-144`
through `:161-165`), not each test's assert line in the test file —
exactly the false-orphan shape this round's task warned about. Table
rows, citing the assert lines directly:

| file:line | kind | verdict | in-family | note |
|---|---|---|---|---|
| ruckig_filter.rs:423 | contains | in-family, discriminating | yes | `joint_vel_accel_jerk_bounds_fails_without_velocity_limits` — bite-confirmed above (`:147-151` neutralized, only this test fails) |
| ruckig_filter.rs:456 | contains | in-family, discriminating | yes | `joint_vel_accel_jerk_bounds_fails_without_jerk_limits` — bite-confirmed above (`:161-165` neutralized, this test plus an independent oracle-parity case fail) |
| ruckig_filter.rs:530 | contains | in-family, discriminating | yes | `multi_dof_active_joint_is_a_typed_error_not_a_silent_last_variable_wins` — bite-confirmed above (`:138-144` DOF≠1 guard neutralized, only this test fails); `contains("planar_joint") && contains('3')` is one assertion site folding two conjuncts, but no sibling guard in this function can produce that joint-name+variable-count message, so there is no second producer to conflate |

(`ruckig_filter.rs:388` (`assert!(`) and `:465`, the acceleration guard and the
pre-existing message-swap-bite-checked assertion, already have rows in
the `moveit-smoothing (11 sites)` table above.)

## Round 6: `ButterworthFilter::new`'s remaining 3 guards, bit

Round 4 bit only guard 3 (`coeff < 1.0`, `butterworth.rs:80`,
tested at `:153`); guards 1, 2, and 4 rested on the comments'
message-uniqueness argument alone, self-flagged in Round 4's table as
"spot-check per instruction." Each of the four guards' messages
(`butterworth.rs:70,75,80,85`) contains a needle unique to that guard —
`feedback_term_` only in guard 1's text, `scale_term_` only in guard
2's, `unstable` only in guard 3's, `resulted in feedback term of 0`
only in guard 4's — so message-uniqueness already discriminates all
four: an assertion passing proves the returned message carried that
guard's unique substring, and no sibling guard's message contains it.
**That discrimination claim does not need a bite and this round does
not upgrade it to one.** What a bite adds, and what was still missing,
is proof each guard is individually load-bearing — that neutralizing
it changes the outcome, rather than being dead code a sibling guard
already masks.

Three-way isolating-mutation bite, each guard neutralized alone
(`&& !true`), `--no-fail-fast`, reverted and `diff`-confirmed clean
against a pre-bite backup before the next:

| Guard neutralized | Test run | Fails how | Sibling tests |
|---|---|---|---|
| `:70` `feedback_term.is_infinite()` | `coefficient_of_infinity_makes_feedback_term_infinite` (coeff=inf) | `unwrap_err()` panics on an `Ok` value (`scale_term: 0.0, feedback_term: -inf`) — guard 2 can't fire (`scale_term` finite at exactly 0), coeff not `< 1.0`, `|feedback_term|` not `< EPSILON` | all 40 others green |
| `:75` `scale_term.is_infinite()` | `coefficient_of_negative_one_makes_scale_term_infinite` (coeff=-1) | the `.contains("scale_term_")` assertion itself panics, **not** `unwrap_err()` — `feedback_term` (2.0) is finite so guard 1 stays quiet, but coeff `-1 < 1.0` so guard 3 fires and the returned message is `"...unstable"`, which does not contain `scale_term_` | all 40 others green |
| `:85` `feedback_term.abs() < EPSILON` | `coefficient_of_exactly_one_makes_feedback_term_zero` (coeff=1.0) **and** `coefficient_just_above_one_is_still_within_the_feedback_term_epsilon_band` (coeff=1+1e-10) | both `unwrap_err()` panic on `Ok` values — no other guard's condition holds at either coefficient | all 39 others green |

All three outcomes were predicted in full (including which of the two
failure shapes — `unwrap_err()` panic vs. assertion-text panic — each
guard would produce) before running, and every run matched the
prediction exactly; no contradiction to report. Combined with Round
4's direct bite of guard 3, all four `ButterworthFilter::new` guards
are now confirmed individually load-bearing, on top of the
message-uniqueness discrimination that already held for all four.
**Verdict: discriminating (by message uniqueness, unchanged from
Round 4) and, as of this round, load-bearing (by bite, all 4 of 4).**
Gate: `cargo fmt --all`; `cargo clippy -p moveit-smoothing --all-targets
-- -D warnings` clean; `cargo nextest run -p moveit-smoothing` (41/41)
green.

### `chain.rs:146` (`let group = model.joint_model_group(group_name)?;`)'s group-lookup guard is exempt, not untested — variant uniqueness, not just message uniqueness

`ChainInfo::build`'s fifth guard, `model.joint_model_group(group_name)?`
(`chain.rs:146`), tested at `chain.rs:488` via
`assert!(matches!(err, Error::UnknownName { .. }))`
(`build_reports_an_unknown_group_as_unknown_name`). `ChainInfo::build`
contains exactly one `?` and every other exit is `Error::other(...)` —
confirmed by reading the whole function body, not just grepping for
`return Err`, since a missed `?` would not show up as a `return`. No
sibling guard can construct an `Error::UnknownName`, so there is no
guard in this function whose neutralization could make
`chain.rs:488`'s assertion pass by accident: it is not merely that the
guards' *messages* happen to differ (the property `is_some` and the
message-uniqueness argument above both rest on), it is that the guards
return different *enum variants*, and the assertion checks the variant,
not the message. Variant uniqueness is a strictly stronger guarantee
than message uniqueness — a future edit could accidentally duplicate a
message string across two `Error::other(format!(...))` call sites and
still compile, but could not accidentally return the wrong variant from
a different `return Err(Error::other(...))` statement without changing
that statement itself. Same class of exemption as the `is_some`
structural exemption from Round 4's opening paragraph (a signal only
one code path can produce needs no bite), applied here to a second,
independent case rather than being a new rule. Not bit; no bite is
needed for this reason to hold, and none would strengthen it further.

`ChainInfo::build`'s five guards are therefore now fully accounted for:
3 bit directly this round (`chain.rs:469,512,558` (`assert!(err.to_string().contains("not a chain"), "got: {err}");`)), 1 proven unreachable
by construction (`chain.rs:193-200` (`joint.joint_type(),`), Round 5), 1 exempt by variant
uniqueness (`chain.rs:146` (`let group = model.joint_model_group(group_name)?;`)/`:488`, this section).

### `do_smoothing`'s velocities-length guard (`acceleration_filter.rs:329` (`if num_positions != num_joints {`)) — the one blind spot this whole sweep could not see

`do_smoothing_before_reset_is_an_error`'s comment claimed its sibling
guard (`:329`, `num_positions != num_joints`, reading `velocities.len()`
per the upstream quirk documented in this module's doc comment) was
"message-swap bite-checked against it." That bite could not have
happened: `rg` for the guard's message text
(`"the length of the joint positions parameter"`) across the whole
workspace found exactly one hit, the guard's own `format!` call —
there was no test to bite. Fixed in two commits: the comment corrected
to state that plainly (`40f0ba5`), and
`do_smoothing_rejects_a_length_mismatch` added and bite-verified
(`e3794b5`) — neutralizing `:329` alone fails only the new test,
falling through to `:335`'s distinct "Make sure the reset was called"
message, all 41 other tests staying green.

This is the structural point, not just a fixed instance: a guard with
no test produces no assertion, and `count-coarse-assertions.py` (like
every funnel-bite audit built on top of it, including this whole
sweep) can only see assertions. A blind-by-omission guard — as opposed
to a blind-by-ambiguity one, where a test exists but targets the wrong
guard — is invisible to the corpus by construction, not by a gap in
this audit's method. The only way to find one is what surfaced this
one: reading a "bite-checked" comment's claim and checking whether the
test it names to have bit against actually exists, rather than trusting
the claim. Checked by `rg` for every `message-swap bite-checked`
comment in this fence's crates and confirming each one's *named
sibling test* actually exists and asserts the claimed message: 3 of
the remaining 4 (`joint_acceleration_bounds`, `ButterworthFilter::new`,
`ruckig_filter.rs::joint_vel_accel_jerk_bounds`) do. The 4th did not —
see below.

Fixing the resulting three-site citation drift in `acceleration_filter.rs`
(`:552→:575`, `:559→:582`, `:566→:589`, plus `:542` itself now pointing
at the new test rather than the guard it used to cite) is its own commit
(`3fb5c65`) — the false-orphan trap this sweep has already found four
other ways, a fifth: a source edit shifting line numbers underneath a
table citation this task's own change caused, not an artifact of
someone else's edit.

### `ruckig_filter.rs`'s "ruckig update failed" site (`:289`) — unreachable, not blind, a second `chain.rs:193-200` (`joint.joint_type(),`)

The 4th `message-swap bite-checked` comment that named a sibling with
no test of its own: `do_smoothing_rejects_a_mismatched_length`
(`ruckig_filter.rs:592`) claimed a bite against `:289`'s
`.map_err(|error| Error::other(format!("ruckig update failed:
{error}")))?`. `rg` for `"ruckig update failed"` across the whole
workspace found exactly one hit, the guard's own `format!` call —
same missing-bite shape as `acceleration_filter.rs:329` (`if num_positions != num_joints {`) above. But
unlike that guard, this one cannot be fixed by adding a test: it is
unreachable.

`RuckigFilter` fixes `ruckig: Ruckig<0, IgnoreErrorHandler>`
(`ruckig_filter.rs:176-179`, struct body ending `have_initial_output: bool,`). Reading the `rsruckig` 3.0.0 crate source
(`~/.cargo/registry/src/.../rsruckig-3.0.0/src/rsruckig/`): every
`Err(RuckigError::...)` construction in the whole crate
(`rsruckig-3.0.0/src/rsruckig/error.rs:103,107`) is reached only through the error-handler type
parameter's `handle_calculator_error`/`handle_validation_error`
methods (`rsruckig-3.0.0/src/rsruckig/ruckig.rs:409`, six sites in `calculator_target.rs`, one in
`calculator_waypoints.rs` — all gated the same way, `rg` for
`handle_calculator_error|handle_validation_error` across that crate's
`src/rsruckig/` confirms no bare `Err(RuckigError` construction exists
outside those two methods). `IgnoreErrorHandler`'s implementation of
both (`rsruckig-3.0.0/src/rsruckig/error.rs:115-121`) unconditionally returns `Ok(())` — its own
doc comment on the trait (`rsruckig-3.0.0/src/rsruckig/error.rs:82,94`) says `Err` propagates only
"when using `ThrowErrorHandler`". `RuckigFilter::new` never
constructs a `ThrowErrorHandler` variant; `Ruckig<0, IgnoreErrorHandler>`
is the only configuration this port uses. So `self.ruckig.update(...)`
at `ruckig_filter.rs:288` (`.update(&self.input, &mut self.output)`) can never return `Err` in this binary,
making `:289`'s `map_err`/`?` dead code from an input perspective —
the same enumeration-not-read standard Round 5 applied to
`chain.rs:193-200` (`joint.joint_type(),`), this time over a dependency's source rather than
this workspace's own. No bite matrix is offered for the same reason
Round 5 didn't write a test for the unreachable `chain.rs` guard: a
green bite result here would be indistinguishable from a
behaviour-preserving no-op, and the read above is exhaustive over
every `Err` construction site in the dependency, not a sample.

Fixed: comment corrected (`2063576`) to state the guard is
unreachable, not "bite-checked," and why. No test added — none is
possible without changing `RuckigFilter`'s error-handler choice, which
this task does not authorize. **Verdict: unreachable, not blind** —
the fourth site the `message-swap bite-checked` anchor sweep found,
and the second guard in this ledger's population (after
`chain.rs:193-200` (`joint.joint_type(),`)) that a false "untested"/"unbit" framing concealed
was actually structurally dead code.
