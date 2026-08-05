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
  against. Cited with the exact line(s) read.
- **doc-recorded bite** — the source's own doc comment states a
  mutation already run (not by me, this round) and its measured
  result, in enough detail to independently verify by reading the
  cited call sites.

## moveit-scene (13 sites)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| scene.rs:2150 | matches! | `diff_scene_records_a_move_only_change_for_an_existing_object` | discriminating | structural — `MoveObjectOutcome` is a multi-variant enum (`Moved`/others); `matches!(_, Moved(_))` names the specific variant `move_object` must produce, distinct from its other outcomes |
| scene.rs:2326 | bare `.is_none()` | (unnamed, `WorldDiff::get`-adjacent) | discriminating | bite (this round) — `WorldDiff::get`'s single guard (`world_diff.rs:104-106`) is the sole producer; see `world_diff.rs:315` bite below, same guard shape |
| scene.rs:2524 | bare `.is_none()` | `decouple_parent_then_mutating_the_former_parent_is_not_observed` | discriminating | structural — `decouple_parent` (`scene.rs:2003-2021`) has exactly one `self.parent = None;` site (verified by `rg 'self\.parent = ' crates/moveit-scene/src/scene.rs`) |
| scene.rs:2556 | bare `.is_none()` | `decouple_parent_materializes_the_inherited_transforms_map` | discriminating | structural — same single `self.parent = None;` site as 2524 |
| scene.rs:2591 | bare `.is_none()` | `decouple_parent_then_the_childs_inherited_attached_body_frame_still_resolves` | discriminating | structural — same single `self.parent = None;` site |
| scene.rs:2616 | bare `.is_none()` | `decouple_parent_then_the_childs_inherited_world_object_still_resolves` | discriminating | structural — same single `self.parent = None;` site |
| scene.rs:2698 | bare `.is_none()` | `clear_diffs_resets_a_diverged_child_to_a_fresh_diff_against_the_parent` | discriminating | bite (this round) — removed `self.acm = Layered::Inherited;` from `clear_diffs`, assertion flipped, reverted |
| scene.rs:2729 | bare `.is_err()` | `frame_transform_resolves_the_model_frame_and_a_link_name` | discriminating | structural — `frame_transform`'s 6-tier ladder (`scene.rs:1312-1332`); "world" resolves in none of the 6 tiers while the same test's earlier assertions (2721-2727) hit tiers 1/2 successfully, so this line specifically isolates the "no tier matched" outcome from the "a tier matched" outcomes tested moments earlier in the same fn |
| scene.rs:2814 | bare `.is_err()` | `frame_transform_reports_a_name_resolving_in_no_tier` | discriminating | structural — same 6-tier ladder, dedicated single-purpose test |
| scene.rs:2873 | bare `.is_err()` | `frame_transform_tier_six_absent_name_is_still_unknown` | discriminating | structural — per file's own tier-6 comment; `attached.is_none()` fixture construction confirmed via `frame_transform_parity.rs`'s `build_scene` (attach loop only populates from `request.attached_bodies`) |
| scene.rs:2882 | bare `.is_err()` | `frame_transform_tier_six_empty_name_is_unknown` | discriminating | structural — same tier-6 exclusivity, empty-string boundary case |
| world_diff.rs:315 | bare `.is_none()` | `set_with_uninitialized_action_erases_the_entry` | discriminating | bite (this round) — neutralized `WorldDiff::set`'s UNINITIALIZED branch (`if false && ...`), assertion flipped, reverted |
| frame_transform_parity.rs:254 | bare `.is_err()` | `panda_frame_transform_matches_the_oracle` | discriminating | structural — oracle-parity test, `query.name == "nothing"` branch isolated from the general `knows_transform` assertion by the file's own `if` guard at line 251 |

## moveit-octomap (10 sites)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| node.rs:143 | bare `.is_none()` | `fresh_node_has_no_children_and_zero_log_odds` | single-branch | bite (this round) — made `Node::new()` eagerly allocate the children array (eliminating the "no array" cause); assertion still passed, proving this test cannot distinguish "no array" from "array present, slot empty" — matches the function's own doc comment (`node.rs:66-67`, "`None` covers both...", a deliberate 2-cause union mirroring upstream's `nodeChildExists`+`getNodeChild`) |
| node.rs:153 | bare `.is_none()` | `create_child_populates_exactly_one_of_eight_slots` | single-branch | structural — fixture calls `create_child(3)` first, which allocates the array (`get_or_insert_with`, `node.rs:78-80`); only the "array present, slot `i` empty" cause is reachable, the "no array" cause is structurally excluded by this fixture |
| tree.rs:1729 | bare `.is_none()` | `ray_with_end_outside_tree_bounds_returns_none` | discriminating | bite (this round, §3a mirror) — neutralized the `end`-guard in `compute_ray_keys` (`.unwrap_or_else(Self::root_key)`), this assertion flipped, reverted |
| tree.rs:1733 | bare `.is_none()` | `ray_with_end_outside_tree_bounds_returns_none` | discriminating | bite (this round) — same test, second assertion (`origin` far-negative case), covered by the same end-guard mutation above |
| tree.rs:1744 | bare `.is_none()` | `ray_with_origin_outside_tree_bounds_returns_none` | discriminating | bite (this round, §3a mirror) — neutralized the `origin`-guard independently, this assertion flipped while the end-guard mutation left it green; reverted |
| tree.rs:1762 | bare `.is_none()` | `unmapped_coordinate_has_no_occupancy` | discriminating | structural — `log_odds_at`→`search` (`tree.rs:876-890`) has two None producers, `self.root.as_deref()?` (empty tree) and the loop's `has_children()`-gated arm (ambiguous partial structure); fresh `OcTree::new(0.1)` has no root, so only the first fires here, distinct from tree.rs:1794 below which fires the second |
| tree.rs:1763 | bare `.is_none()` | `unmapped_coordinate_has_no_occupancy` | discriminating | structural — same test, `is_occupied` composes `log_odds_at` (`tree.rs:908-910`), same root-absent cause |
| tree.rs:1794 | bare `.is_none()` | `insert_ray_cut_short_by_max_range_records_only_a_miss` | discriminating | structural — tree has a root here (prior `insert_ray` calls built structure up to the max-range cutoff), so `self.root.as_deref()?` cannot fire; `end` is beyond the cutoff and was never traversed, so `search`'s loop hits the `cur.has_children()==true` arm (`tree.rs:885`) instead — the second, distinct None-producer from 1762/1763 |
| tree.rs:1930 | bare `.is_none()` | `leaves_in_bbx_returns_none_for_an_out_of_range_max` | discriminating | commit — doc comment (`tree.rs:1924-1926`) and prior isolating mutation recorded at `0d10a11`; `git show 0d10a11 --stat` confirms it touches this test/guard pair |
| tree.rs:1944 | bare `.is_none()` | `leaves_in_bbx_returns_none_for_an_out_of_range_min` | discriminating | commit — doc comment (`tree.rs:1936-1940`, explicitly states "before this test existed the `min` guard had no coverage at all") and prior isolating mutation recorded at `567342f`; `git show 567342f --stat` confirms |

## moveit-state (10 sites)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| invariants.rs:585 | bare `.is_err()` | `unknown_name_is_an_error_not_a_panic_for_every_new_accessor` | single-branch | structural — `variable_velocity` (`state.rs:369-`) is a single-`?`-site accessor; one name-lookup guard, no second producer |
| invariants.rs:586 | bare `.is_err()` | same fn | single-branch | structural — `variable_acceleration`, same single-`?`-site shape |
| invariants.rs:587 | bare `.is_err()` | same fn | single-branch | structural — `variable_effort`, same shape |
| invariants.rs:588 | bare `.is_err()` | same fn | single-branch | structural — `set_variable_velocity`, same shape |
| invariants.rs:589 | bare `.is_err()` | same fn | single-branch | structural — `set_variable_acceleration`, same shape |
| invariants.rs:594 | bare `.is_err()` | same fn | single-branch | structural — `set_variable_effort`, same shape |
| invariants.rs:595 | bare `.is_err()` | same fn | single-branch | structural — `joint_velocity`, same shape |
| invariants.rs:596 | bare `.is_err()` | same fn | single-branch | structural — `joint_acceleration`, same shape |
| invariants.rs:597 | bare `.is_err()` | same fn | single-branch | structural — `joint_effort`, same shape |
| jacobian.rs:211 | matches! | `an_unknown_group_name_is_unknown_name_not_not_a_chain` | discriminating | structural — `matches!(err, Error::UnknownName{..})` names the specific variant `RobotModel::joint_model_group` (`robot_model.rs:605-618`, single guard) produces, distinct from the sibling `jacobian_on_an_unsupported_group_type_errors` test's "not a chain" `Error::Other` message check (`jacobian.rs:189-195`) — doc comment at 198-200 states this explicitly |

Each of the 9 `invariants.rs` sites is individually single-branch (its
own accessor has exactly one guard), but the 9 together are what
discriminates the accessor family from each other — this is the same
"one row = one call site" shape as round 8's matrix.rs table, not a
finding.

## moveit-constraints (9 sites)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| constraint_sampler_manager.rs:172 | bare `.is_none()` | `no_constraints_and_no_solver_returns_none` | single-branch | structural — re-traced `select_default_sampler`/`_inner`'s full Step A/B/C/D control flow (`constraint_sampler_manager.rs:141-`); no test anywhere in the file exercises the *other* None-producer (`select_default_sampler:149-151`'s unknown-group-name early return — `rg 'select_default_sampler\('` on the test file shows every call uses a valid `"panda_arm"`/`"panda_arm_hand"` group name), so only the Step-D fallthrough is reachable and tested |
| decide.rs:183 | bare `.is_err()` | `new_rejects_negative_tolerance` | single-branch | structural — `JointConstraint::new`'s tolerance check is one combined OR-condition (`joint.rs:120`, `tolerance_above < 0.0 \|\| tolerance_below < 0.0`), a single guard; this line and 184 cannot be discriminated from each other because the source has only one branch to hit |
| decide.rs:184 | bare `.is_err()` | `new_rejects_negative_tolerance` | single-branch | structural — same single OR-guard as 183 |
| decide.rs:210 | matches! | `new_rejects_unknown_joint` | fixed | commit `83e3c1c` — this session's own prior fix, asserts the specific `Error::UnknownName{kind:"joint",..}` variant/field rather than bare `.is_err()` |
| utils_parity.rs:221 | matches! | `unknown_group_is_error` | single-branch | structural — `construct_goal_joint_constraints`'s only reachable guard for an unknown group name is `model.joint_model_group(group_name)?` (`utils.rs:234`); the loop body's two further `?` sites are unreached when this one fails first |
| utils_parity.rs:641 | matches! | `multi_region_constraint_is_error` | single-branch | structural — `update_position_constraint` has exactly one `Error::Other`-producing site (`utils.rs:605-609`, converting `with_updated_position`'s `None` for a >1-region constraint) |
| utils_parity.rs:885 | bare `.is_none()` | `an_unrecognised_frame_is_none` | single-branch | structural — `resolve_position_constraint_frame` (`utils.rs:727-747`) has one `None`-producing path, `resolve_frame_to_link`'s own single `None` cause (`utils.rs:641`, the closure result — the two other tiers only ever return `Some` or fall through, never `None`) |
| utils_parity.rs:896 | bare `.is_none()` | same fn, second assertion | single-branch | structural — `resolve_orientation_constraint_frame` (`utils.rs:777-805`) shares the identical single `resolve_frame_to_link` None cause |
| utils_parity.rs:942 | matches! | `xyz_euler_tolerance_across_a_real_frame_change_is_an_error` | single-branch | structural — `resolve_orientation_constraint_frame`'s only `Error::Other`-producing site is the `XyzEuler`-tolerance-across-a-frame-change guard (`utils.rs:796-802`), a single `if` |

## moveit-metrics (3 sites)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| lib.rs:1068 | matches! | `unknown_group_is_unknown_name` | single-branch | structural + doc-recorded — doc comment (`lib.rs:1053-1059`) states `manipulability` and `manipulability_index` "share `KinematicsMetrics::group`'s `self.model.joint_model_group(group)?` call verbatim"; single guard |
| lib.rs:1072 | matches! | same fn, second assertion | single-branch | structural + doc-recorded — same shared single guard, `manipulability_index` caller |
| lib.rs:1139 | matches! | `manipulability_ellipsoid_rejects_the_same_bad_groups` | single-branch | doc-recorded bite — doc comment (`lib.rs:1102-1123`) records a mutation already run and its result: replacing `self.group(...)?` with `let _ = self.group(...);` inside `manipulability_ellipsoid` "leaves the variant-level assertions below unchanged" because `state.jacobian`'s own re-check produces the byte-identical `UnknownName` either way; this line's assertion does *not* pin `self.group`'s own call site (only the message-level check at 1132-1138, outside this family, does that) |

## moveit-smoothing (2 sites)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| acceleration_filter.rs:552 | matches! | `reset_rejects_a_mismatched_length` | single-branch | structural — `reset`'s only `Error::Other` site is one combined OR-condition (`acceleration_filter.rs:302`, `positions.len() != num_joints \|\| velocities.len() != num_joints`) |
| ruckig_filter.rs:546 | matches! | `reset_rejects_a_mismatched_length` | single-branch | structural — same shape, `ruckig_filter.rs:326-329`, 3-clause OR-condition, still one guard |

## moveit-srdf (2 sites)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| boundaries.rs:106 | matches! | `a_virtual_joint_missing_any_required_attribute_is_dropped` | discriminating | structural — parametrized over 4 distinct missing attributes (name/child_link/parent_frame/type), each loop iteration's `matches!` checks the specific `attribute: a` field against that iteration's own `attribute` var, so each of the 4 cases is its own isolating check within the same test |
| boundaries.rs:422 | matches! | `an_unparsable_joint_value_drops_the_joint_instead_of_storing_zero` | discriminating | structural — parametrized over 5 malformed raw strings, all producing `MalformedValue{attribute:"value"}`; the `matches!` names the specific diagnostic variant and field, distinguishing it from other `Diagnostic` variants the parser could emit |

## Sites needing a fix this round

None. Every site in this round's 7 crates was already discriminating,
provably single-branch by direct source read, or previously fixed —
no blind/never-covered site was found (in contrast to round 8's
`matrix.rs:678`/`set_entry_for_known` fixture collapse). `node.rs:143`
was the one site carrying only structural/doc-comment justification
going into this round; it now also has a fresh bite.

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
git status --short   # empty, both before and after every mutation
git diff --stat       # empty at end of round
```

## Gate

No source in any of the 7 crates was changed this round (no fix was
needed), so no `-p <crate>` fmt/clippy/nextest gate is owed for a
commit — there is no commit. `node.rs:143`'s bite mutation was
reverted, confirmed via `git status --short`/`git diff --stat` both
returning empty. This document lives under `doc/`, outside any crate,
matching the merged census document's own precedent (no cargo gate
applies to a doc-only deliverable).

## UNFIXED

None.
