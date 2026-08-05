# Assertion-discrimination ledger — p9-ros (round 8)

Per-site record for the three crates delegated this round: `ros/moveit-ros`,
`moveit-distance-field` (p1-joints' fence, scope override accepted), and
`moveit-model` (p1-robotmodel's fence, scope override accepted). Both
overrides were accepted — I had direct working context on both crates from
earlier rounds and no reason to decline.

Enumerated with a hand-rolled paren-depth + comment-masking scanner
(`ledger_scan.py`, same method as `doc/assertion-discrimination-census.md`:
mask `//`/`/* */` comments string/char-literal-aware, find `assert!(`, track
paren depth over the unmasked text to the true close-paren, classify
`matches!` vs `bare` by priority). Cross-checked per-crate against
`rg -U -c 'assert!\(\s*matches!\('` and
`rg -U -c 'assert!\(\s*[\s\S]{0,300}?\.(is_err|is_none)\('` — exact agreement
on all three crates. The scanner is not the source of any reconciliation
delta below; every delta is real git-history drift.

## Verdict taxonomy — what `not-this-family` means

Governed by census §9 (`doc/assertion-discrimination-census.md`, merged
`95eb25b`, authored by p1-robotmodel) — superseding this document's own
earlier ad hoc rule statement, which was directionally right but not the
authority. §9's three clauses, all must hold for family membership; any
one failing is `not-this-family`:

1. **Mechanism** — the value is a failure/absence signal (`Err`, `None`,
   or a coarse "failed"/"found nothing" tag), not a plain informative
   success-path value that already names in full what produced it.
2. **Decision** — the signal comes from a written decision in the subject
   (guard, early return, `?`, a comparison actually performed on a real
   element), not an emergent non-decision. Operational test: could an
   engineer have implemented this specific decision wrong in a way a
   mutation here would exercise?
3. **Subject** — the decision belongs to the function the test targets,
   not to the test's own setup. Operational test: if you deleted the call
   to the code under test, would the assertion's outcome be unaffected?
   If yes, it was never about the subject.

Every row below cites which clause(s) apply, not a re-derived paraphrase.

## Reconciliation against the census (`47d3475`, measured at `df36fab`)

| crate | census (matches!+bare) | measured now | delta | cause |
|---|---|---|---|---|
| `ros/moveit-ros` | 26+2=28 | 25+2=27 | -1 | `b0a9e1b` (this branch, pre-round-8) converted `state.rs`'s `assert!(matches!(err, Error::UnknownName{..}))` to `assert_err_mentions(...)` — real fix, landed on a sibling branch to `df36fab` before the two merged, not an instrument miss. `state.rs` now has zero anchor-matching sites. |
| `moveit-distance-field` | 2+18=20 | 2+16=18 | -2 | Two of this branch's own pre-round-8 commits converted bare sites to message-content checks: `08976b8` (`.is_err()` → `match` on structured `kind`, drops the `assert!` shape entirely) and `5ea2418` (`.is_err()` → `err.to_string().contains(...)`). Both topologically parallel to `df36fab`, same branch-merge situation as above. |
| `moveit-model` | 5+15=20 | 5+17=22 | +2 | `72f5eca` (merged after `df36fab`) added two sites: `46cd26b`'s new test `get_end_effector_unknown_name_is_an_error` (`robot_model.rs:2719`) and `7676185`'s added `j4` mimic check when the cycle-clear fixture was widened from 3 joints to 4 (`robot_model.rs:2028`). Real new-test growth, not an instrument miss. |

No crate-row disagreement is an instrument bug; all three are named, dated,
`git log`-verified drift since `df36fab`.

## `ros/moveit-ros` (27 sites: 25 matches! + 2 bare)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| constraints/joint.rs:93 | matches! | unknown_joint_name_is_rejected | discriminating | `9c05e4f` |
| constraints/orientation.rs:176 | matches! | converts_with_xyz_euler_parameterization | not-this-family | §9 clause 1 — `c.tolerance()` is read after `.unwrap()` on a successful build; `OrientationTolerance::XyzEuler{..}` is a computed success-path enum tag, not a failure/absence signal |
| constraints/orientation.rs:262 | matches! | unknown_frame_is_rejected | discriminating | `3303a7e` |
| constraints/position.rs:292 | matches! | prism_is_rejected | single-branch | bite run just now — `Error::Other` has exactly one producing arm (`PRISM`) in `TryFrom<SolidPrimitiveMsg> for Shape` |
| constraints/set.rs:163 | matches! | aggregates_joint_constraints | not-this-family | §9 clause 1 — `set.constraints()[0]` read after `.unwrap()` on a successful build; `Constraint::Joint(_)` is a computed success-path enum tag |
| constraints/set.rs:183 | matches! | one_bad_element_fails_the_whole_conversion | discriminating | `a7d8682` |
| constraints/set.rs:421 | matches! | (visibility round-trip) | not-this-family | §9 clause 1 — same shape as `:163`, `Constraint::Visibility(_)` on a successful build |
| constraints/visibility.rs:218 | matches! | (sensor_view_direction rejects unknown) | single-branch | bite run just now — single `other =>` arm in `TryFrom<SensorViewDirectionMsg>` |
| geometry.rs:300 | matches! | zero_quaternion_is_rejected | single-branch | bite run just now — one combined guard in `TryFrom<Quaternion> for UnitQuaternion` |
| geometry.rs:312 | matches! | nan_quaternion_is_rejected | single-branch | bite run just now — same guard as above |
| geometry.rs:398 | matches! | orientation_zero_quaternion_is_rejected | single-branch | bite run just now — one combined guard in `TryFrom<OrientationConstraintQuaternion>` |
| geometry.rs:410 | matches! | orientation_nan_quaternion_is_rejected | single-branch | bite run just now — same guard |
| geometry.rs:433 | matches! | orientation_norm_just_outside_the_1e_minus_3_tolerance_is_rejected | single-branch | bite run just now — same guard |
| geometry.rs:449 | matches! | orientation_norm_far_from_one_is_rejected_not_silently_renormalized | single-branch | bite run just now — same guard |
| geometry.rs:522 | matches! | pose_with_degenerate_orientation_fails | single-branch | bite run just now — position leg (`TryFrom<Point>`) is unconditionally `Ok` ("Total in practice" doc comment), so only the orientation guard can fire |
| planning.rs:397 | bare | converts_minimal_request | fixture-collapse-fixed | `9d829a8` (own earlier commit) |
| planning.rs:508 | matches! | multi_dof_joint_trajectory_is_rejected_not_silently_dropped | single-branch | bite run just now — only `Error::Other` site in this impl; delegate produces `Error::Construct` |
| scene/collision_object.rs:983 | bare | append_without_subframe_data_clears_existing_subframes | discriminating | §9 all three clauses hold — clause 1: `subframe_pose("tip").is_none()` is a genuine absence signal (retained vs. cleared); clause 2: `apply_add`'s unconditional subframe-replace is a written decision an engineer could gate on non-empty (bite: wrapping it in `if !subframes.is_empty()` flips the assertion); clause 3: deleting the second `apply_collision_object` call leaves the first object's subframe in place, changing the outcome. Bite run once, before I knew the ros gate was paid (docker, targeted single test — see cost note in the round report) |
| scene/mod.rs:94 | matches! | unresolvable_non_empty_frame_id_is_still_rejected | single-branch | bite run just now — `header_frame_transform`'s only Err path is the single `scene.frame_transform` call |
| scene/planning_scene.rs:519 | matches! | unresolvable_non_empty_header_frame_id_is_still_rejected | single-branch | bite run just now — `Error::UnknownName` reachable only via `header_frame_transform` (see scene/mod.rs:94); the file's other two error sites are `Error::Other`/`Error::Construct`, different variants |
| scene/shapes.rs:203 | matches! | plane_wrong_coef_length_is_rejected | single-branch | bite run just now — single `Error::construct` site in `TryFrom<PlaneMsg> for Plane`; `Plane::new` is infallible |
| trajectory.rs:383 | matches! | add_suffix_way_point_rejects_a_nonzero_first_dt | single-branch | bite run just now — `moveit_trajectory::RobotTrajectory::add_suffix_way_point`'s single `first_duration_error()` site |
| trajectory.rs:444 | matches! | seconds_to_duration_rejects_just_above_i32_max_seconds | single-branch | bite run just now — `seconds_to_duration`'s one combined guard |
| trajectory.rs:450 | matches! | seconds_to_duration_rejects_negative | single-branch | bite run just now — same guard |
| trajectory.rs:456 | matches! | seconds_to_duration_rejects_nan | single-branch | bite run just now — same guard |
| trajectory.rs:462 | matches! | seconds_to_duration_rejects_infinity | single-branch | bite run just now — same guard |
| trajectory.rs:490 | matches! | negative_cumulative_duration_from_an_unvalidated_trajectory_is_rejected | single-branch | bite run just now — `TryFrom<RobotTrajectory> for JointTrajectoryMsgOut`'s sole `?` site is `seconds_to_duration` |

No blind/never-covered site survived inspection in `ros/moveit-ros`; the
27th matches this crate's own scan count exactly. Zero commits this round.

## `moveit-distance-field` (18 sites: 2 matches! + 16 bare) — scope override, p1-joints' fence

Every site already carries a dedicated round-report commit from this
branch's earlier rounds; all re-read and agreed with, none re-argued.

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| collision_distance_field_types.rs:1434 | bare | (`PosedBodySphereDecompositionVector::get` bounds) | single-branch | `37bd860` |
| collision_distance_field_types.rs:1446 | bare | (`PosedBodyPointDecompositionVector::get` bounds) | single-branch | `37bd860` |
| collision_distance_field_types.rs:1595 | bare | (`from_octree` leaves body_decomposition None) | single-branch | `f650ef2` |
| collision_env_distance_field.rs:2740 | bare | get_distance_field_cache_entry_returns_none_when_current_is_none | discriminating | `0116b87` |
| collision_env_distance_field.rs:2751 | bare | (group-name mismatch guard) | discriminating | `0116b87` |
| collision_env_distance_field.rs:2769 | bare | (state-mismatch guard) | discriminating | `0116b87` |
| collision_env_distance_field.rs:2793 | bare | (acm-mismatch guard) | discriminating | `0116b87` |
| collision_env_distance_field.rs:2809 | matches! | (acm:None skips acm check, returns same entry) | discriminating | `0116b87` — `ptr::eq` identity check, not a bare presence check |
| collision_env_distance_field.rs:2826 | matches! | (agreeing state+acm returns same entry) | discriminating | `0116b87` — same |
| collision_env_distance_field.rs:3271 | bare | (post-update link_body_decompositions None) | single-branch | `7e69a8c` |
| collision_env_distance_field.rs:3276 | bare | (post-update link_distance_fields None) | single-branch | `7e69a8c` |
| collision_env_distance_field.rs:3734 | bare | (generate_collision_checking_structures: generate_distance_field=false) | single-branch | `202c88f` |
| collision_env_distance_field.rs:3757 | bare | (generate_collision_checking_structures: no-geometry case) | single-branch | `202c88f` |
| distance_field.rs:788 | bare | (octree-missing-payload error) | single-branch | `233cc77` |
| propagation.rs:823 | bare | checked_max_distance_sq boundary rejection | single-branch | `246c1a8` |
| voxel_grid.rs:491 | bare | new_rejects_size_over_resolution_one_past_the_i32_boundary | single-branch | `5ea2418` |
| tests/collision_env_distance_field_parity.rs:819 | bare | (parity fixture link-decomposition None) | single-branch | `d74c9f5` |
| tests/oracle_parity.rs:264 | bare | (nearest_cell sentinel voxel-presence check) | discriminating | `705f580` — the companion `assert_eq!(actual.distance, expected.distance)` three lines above pins the class among the 5 `voxel: None` return sites; bite-checked (see commit body) |

Zero blind sites, zero commits this round.

## `moveit-model` (22 sites: 5 matches! + 17 bare) — scope override, p1-robotmodel's fence

Most of these 22 sites had no dedicated round-report commit before this
round; verdicts are a bite run just now (direct read of the function under
test, confirming the number of Error/None-producing sites reachable from
that test's call path), except `joint/urdf.rs:388` and the mimic sites
(`robot_model.rs:2025,2026,2027,2046,2078`, `joint/model.rs:1040,1047`),
which round 6's report (`01KZ7P9B03SWRKQWQV1DXB4EEA-7.md`, absolute path
under `.caucus/sessions/`, not reachable from this worktree — `.caucus` is
gitignored) already covered; those were re-bitten this round rather than
cited blind, and agree with the prior report. `robot_model.rs:2028` (j4)
is new since that report and had never been bitten by anyone until now.

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| joint/model.rs:983 | bare | local_variable_index_errors_on_unknown_variable | single-branch | bite run just now — `local_variable_index` has one Err site |
| joint/model.rs:990 | bare | set_continuous_errors_on_non_revolute_joint | single-branch | bite run just now — `set_continuous` has one Err site |
| joint/model.rs:1040 | bare | mimic_set_get_clear_round_trip (pre-set) | not-this-family | §9 clause 3 (and 2) — asserted immediately after `JointModel::new_prismatic(...)`, before `set_mimic`/`clear_mimic` run; deleting the test's later `set_mimic`/`clear_mimic` calls leaves this assertion unaffected, so it was never about the subject those calls exercise. `new_prismatic`'s `mimic: None` default is an unconditional field init, not a comparison the constructor performs (clause 2 non-decision, same shape as the census's `shortest_solution_is_none_on_empty_input` example). Corrects my own prior-turn misclassification — bite-testing `new_single_variable`'s default is a real mutation but not evidence about the *subject this test names*; §9's clause-3 test settles it, not the mutation's mere existence |
| joint/model.rs:1047 | bare | mimic_set_get_clear_round_trip (post-clear) | discriminating | §9 all three clauses hold — clause 1: `.mimic().is_none()` is a genuine absence signal; clause 2: `clear_mimic()`'s unconditional `self.mimic = None` is a written decision an engineer could omit (bite: no-op mutation fails the assertion); clause 3: deleting the immediately-preceding `joint.clear_mimic()` call changes the outcome (mimic stays `Some` from the earlier `set_mimic`). `01KZ7P9B03SWRKQWQV1DXB4EEA-7.md` + re-bitten this round |
| joint/urdf.rs:366 | matches! | fixed_floating_and_planar_produce_the_matching_kind | not-this-family | §9 clause 1 — the census's own worked example, verbatim: `.kind()` after `.unwrap()` on a successful build; `JointKind::Fixed` is a computed success-path enum tag |
| joint/urdf.rs:372 | matches! | (same test, Floating) | not-this-family | §9 clause 1 — same shape, `JointKind::Floating(_)` |
| joint/urdf.rs:378 | matches! | (same test, Planar) | not-this-family | §9 clause 1 — same shape, `JointKind::Planar(_)` |
| joint/urdf.rs:388 | bare | spherical_joint_type_is_rejected | single-branch | `9834193` — `joint_model_from_urdf` has exactly one `Error::construct` site (Spherical arm) |
| robot_model.rs:1962 | bare | (joint_model_mut unknown name) | single-branch | bite run just now — `joint_model_mut` has one Err site |
| robot_model.rs:2024 | matches! | mimic_mutual_cycle_clears_every_mimic_in_the_model | discriminating | bite run just now — `[Diagnostic::MimicCycle]` slice pattern requires both exact length 1 and the named variant |
| robot_model.rs:2025 | bare | mimic_mutual_cycle_clears_every_mimic_in_the_model (j1) | fixture-collapse-fixed | §9's own worked example (census §9, "Worked resolution of the disputed post-build state check") is this exact site: all three clauses hold — clause 1 (`.mimic().is_none()` collapses "never had one" and "had one, cleared" into one signal), clause 2 (the build/clear routine's per-joint null decision), clause 3 (belongs to the routine this test names). `7676185` — fixture gave j1 a mimic outside the cycle; two isolating mutations (narrow-to-cycle-members, skip-clear-entirely) both now fail here |
| robot_model.rs:2026 | bare | (j2) | discriminating | §9 same three clauses as :2025 (identical mechanism/decision/subject — same routine, same getter). `01KZ7P9B03SWRKQWQV1DXB4EEA-7.md` + re-bitten — skip-clear-entirely fails here (clause 2's decision is real for j2); narrow-to-cycle-members does NOT fail here (j2 is itself a cycle member, cleared under any correct scoping) — in-family and non-vacuous for "clearing happens", just not independent evidence for the whole-model-vs-cycle-only scope claim, which only j1/j4 carry |
| robot_model.rs:2027 | bare | (j3) | discriminating | same as :2026 — §9 clauses hold identically; `01KZ7P9B03SWRKQWQV1DXB4EEA-7.md` + re-bitten, same caveat |
| robot_model.rs:2028 | bare | (j4, new in `7676185`) | fixture-collapse-fixed | §9 same three clauses as :2025. `9661d4c` (this round) — was blind under every mutation tested (no `<mimic>` tag of its own, same defect as pre-fix j1); fixed by giving j4 a real out-of-cycle mimic on new leaf joint j5, re-verified failing under the narrow-scope mutation post-fix. Never bitten before this round |
| robot_model.rs:2046 | bare | mimic_of_unknown_joint_is_dropped_with_a_diagnostic | discriminating | §9 same three clauses — different routine (single-joint clear on `MimicUnknownJoint`), same mechanism/decision/subject shape. `01KZ7P9B03SWRKQWQV1DXB4EEA-7.md` (as old line 2010) + re-bitten — mutated the single-joint clear to a no-op; assertion fails |
| robot_model.rs:2078 | bare | mimic_with_mismatched_dof_is_dropped_with_a_diagnostic | discriminating | §9 same three clauses (`MimicDofMismatch` sibling of :2046). `01KZ7P9B03SWRKQWQV1DXB4EEA-7.md` (as old line 2042) + re-bitten — same mutation, same result |
| robot_model.rs:2413 | bare | mesh_collision_resolving_to_an_unreadable_file_is_skipped_with_a_diagnostic | not-this-family | §9 clause 3 — the census's own worked example, verbatim: `assert!(std::fs::read(&path).is_err(), "precondition: ...")`; the test calls `std::fs::read` itself, no subject code runs before this assertion |
| robot_model.rs:2485 | matches! | mesh_collision_resolving_to_a_valid_stl_file_builds_a_mesh_shape | not-this-family | §9 clause 1 — `shapes[0].shape` read after `.expect("builds")`; `Shape::Mesh(_)` is a computed success-path enum tag, same shape as `JointKind::Fixed` |
| robot_model.rs:2700 | bare | (get_end_effector("arm"), real group but not an end effector) | single-branch | bite run just now — despite the doc comment naming two conceptual causes, `get_end_effector` is one `.filter(...).ok_or_else(...)` chain: exactly one `Error::unknown_name` construction site |
| robot_model.rs:2719 | bare | get_end_effector_unknown_name_is_an_error (new in `46cd26b`) | single-branch | bite run just now — same single call site as 2700 |
| robot_model.rs:2943 | bare | group_state_where_every_joint_value_is_unusable_stores_no_state_at_all | single-branch | bite run just now — `variable_default_positions` is a one-line `self.default_states.get(name)` delegation (`joint_model_group.rs:257`) |
| robot_model.rs:2953 | bare | variable_default_positions_returns_none_for_unknown_state_name | single-branch | bite run just now — same delegation |

Corrected this round in two passes (see verdict-taxonomy note above): first,
9 sites were reclassified out of `not-this-family` on the theory that a
post-build state check is categorically outside the family — re-bitten
individually, not reasoned from shape. Second, applying census §9's formal
three-clause test to all 13 originally-`not-this-family` model rows (not
just the 9 already flagged) found one of those 9 was itself too coarse:
`joint/model.rs:1040` (the *pre-set* `mimic_set_get_clear_round_trip`
assertion) fails §9 clause 3 — it reads a value the subject never decided,
since it runs before either `set_mimic` or `clear_mimic` — and moves back to
`not-this-family`. `:1047` (the post-*clear*-set assertion in the same test)
passes all three clauses and stays `discriminating`. `robot_model.rs:2025`
(j1) is census §9's own worked example verbatim; `robot_model.rs:2028` (j4)
was blind under every mutation tried and had never been bitten before —
fixed this round by `9661d4c`.

Final breakdown: 6 discriminating, 8 single-branch, 2 fixture-collapse-fixed
(`7676185` for j1, `9661d4c` for j4, this round), 6 genuinely
`not-this-family` under §9 (`joint/model.rs:1040` clause 3,
`joint/urdf.rs:366,372,378` clause 1 `JointKind` variants,
`robot_model.rs:2413` clause 3 fixture precondition, `robot_model.rs:2485`
clause 1 `Shape::Mesh` variant). 6+8+2+6 = 22. No site here reads as a
hidden D6 finding: `robot_model.rs:2700`/`2719`'s doc comment flags two
conceptual input scenarios, but both trace to the same single construction
call, not two guards collapsing to an indistinguishable shared
`None`/error. One commit this round: `9661d4c`.

## Summary

68 sites assigned this round (27 + 18 + 22 above — note this is 67, not 68;
the census's own 68 = 28+20+20 included the `ros/moveit-ros` site
(`state.rs`) that `b0a9e1b` removed before this round started, so the
delegated 68 and the measured 67 differ by exactly that one already-reconciled
site).

**Revised in two rounds.** First pass: `not-this-family` had drifted to mean
"not about `Result`/`Option`" instead of a defined membership test, and
misclassified 9 post-build business-state sites on that basis —
`robot_model.rs:2025,2026,2027,2028,2046,2078`, `joint/model.rs:1040,1047`,
`scene/collision_object.rs:983`. Reclassified by isolating mutation (see the
two crate tables for per-site evidence), not by re-reasoning from shape.

Second pass: with `not-this-family` formally defined as census §9 (three
clauses — mechanism, decision, subject; merged `95eb25b`, authored by
p1-robotmodel), re-checked all 17 originally-`not-this-family` rows against
the clauses directly rather than accepting the first pass's 9-reclassified/
7-confirmed/1-uncounted split as given. One row was too coarse:
`joint/model.rs:1040` had been swept into the first pass's 9 as
`discriminating`, but fails §9 clause 3 (it reads a value — `mimic()`
before either `set_mimic` or `clear_mimic` runs — that the subject never
decided) and moves back to `not-this-family`. Every other row's clause
result agrees with the first pass's bite-check result, including
`robot_model.rs:2025`, which turns out to be census §9's own worked example
verbatim.

Final verdicts: 17 discriminating (4 ros + 7 distance-field + 6 model), 38
single-branch (19 ros + 11 distance-field + 8 model), 3 fixture-collapse-fixed
(1 ros pre-existing + 2 model: `7676185`, `9661d4c`), 9 `not-this-family`
under §9 (3 ros clause-1 success-path checks + 6 model: `joint/model.rs:1040`
clause 3, 3 `JointKind` clause-1 variants, one clause-3 precondition, one
clause-1 shape-variant check). 17+38+3+9 = 67. One commit this round:
`9661d4c`.

**In-family denominator, the way p1-robotmodel reported it (51/55):**
in-family = total minus §9-excluded `not-this-family`. `ros/moveit-ros`:
24/27. `moveit-distance-field`: 18/18 (zero `not-this-family` rows).
`moveit-model`: 16/22. Combined: **58/67** in-family across the three
crates this round covers.

## §10. Wider-grammar sweep (`tools/ci/count-coarse-assertions.py`, round 9)

The brief for this round asked for the `contains_msg` / `is_empty` / `eq_none`
/ `contains_member` / `is_some` sites the census's original `matches!`/
`.is_err()`/`.is_none()` grammar cannot see, quoting 67 total hits on `ros/`
with 41 (listed as 29 `contains_msg` + 6 `is_empty` + 4 `eq_none` + 4
`contains_member` + 1 `is_some` + 2 `is_none`) outside the old grammar. Two
corrections to that framing, both load-bearing for what follows:

**The 67 baseline predates this branch's own merge.** `git merge-base
--is-ancestor` shows `6a14a89` (the instrument's commit) is not an ancestor
of `f0855c5` (this round's own merge of the seven folded-operand-guard
tests) — the two landed on sibling branches, and `6a14a89`'s 67 was counted
before `f0855c5`'s tests existed. Re-running `python3
tools/ci/count-coarse-assertions.py ros` on this branch today (after `git
merge main`, which fast-forwarded to `91f4ebb`, `6a14a89` included) gives
**72**, not 67. The delta of exactly 5 is my own round-8 commits: two
`contains_msg` (`position.rs`'s `meshes_alone`/`mesh_poses_alone` tests),
one `matches` (`planning.rs`'s `multi_dof_joint_trajectory_points` test),
two `eq_none` (`conversion_coverage.rs`'s two `parse_conversion` unit
tests) — all bite-checked under docker in that round, so this is not new
uncovered ground, just an accounting update: **72 total, 44 outside the old
grammar** (26 `matches` + 0 `is_err` + 2 `is_none` = 28 old-grammar; the
quoted breakdown's own arithmetic — 29+6+4+4+1+2 — sums to 46, not 41,
independent of the baseline issue).

**The tool's documented blind spot undercounts the real total further, and
by more than the quoted 25.** `assert_err_mentions(result, needle)` is
defined separately per file (`state.rs:192`, `trajectory.rs:248`,
`planning.rs:331`, `scene/attached.rs:323`, `scene/collision_object.rs:533`
— five independent copies, not a shared import) and renders on one line,
asserts `rendered.contains(needle)` on the next — invisible to the tool's
60-byte lookback, exactly as documented. Two things the tool's own output
additionally obscures: its 5 helper-*definition* lines (one `rendered.
contains(needle)` per file) show up as ordinary `contains_msg` hits and are
not real per-test assertions — they're the generic body every call site
below delegates to. Subtracting those: 44 tool-visible new-grammar hits −
5 helper bodies = **39 real tool-visible sites**. Grepping
`assert_err_mentions(` call sites directly (excluding each file's own
generic `fn assert_err_mentions<T: ...>(` definition, which never matches
the literal `assert_err_mentions(` pattern because of the `<T: ...>`
between the name and the paren): `state.rs` 11, `trajectory.rs` 6,
`planning.rs` 2, `scene/attached.rs` 2, `scene/collision_object.rs` 2 —
**23 hidden call sites**, not the quoted 25.

**Real total: 39 + 23 = 62 new-grammar assertion sites**, against the
quoted 41 (which itself undercounts by the 5-commit accounting gap above)
and the tool's own 44 tool-visible count.

### Per-site enumeration and verdicts

Worked directly (`constraints/orientation.rs`, `constraints/position.rs`,
`constraints/set.rs`, `constraints/visibility.rs` — 12 sites) plus five
parallel forks, one per remaining file group, each independently applying
census §9's three clauses and enumerating every sibling Err/None-producing
site reachable from the same subject function before ruling on collision.

| file | sites | in-family verdict | collision verdict |
|---|---:|---|---|
| `constraints/orientation.rs` | 3 (`:194,217,244`) | in-family | CLEAN — pre-existing sibling-collision comments confirmed correct by reading |
| `constraints/position.rs` | 6 (`:308,392,411,432,456,472`) | in-family | CLEAN — `dim()`'s `{field}`-interpolated messages, `PositionConstraint::new`'s three distinct `Error::construct` texts, and the meshes-guard's shared-but-same-branch needle all verified against `crates/moveit-constraints/src/position.rs` |
| `constraints/set.rs` | 1 (`:148`) | **not-this-family** (clause 2) | n/a — `empty_constraints_is_empty_set` is the census's own vacuous-accumulator shape verbatim: all four input vecs empty, none of the four `for` loops in `KinematicConstraintSet::try_from` iterate, `set` is never touched |
| `constraints/visibility.rs` | 2 (`:384,385`) | in-family | CLEAN — `normalize_angle_criterion`'s `>EPS` filter is the sole physical producer of `None` for each field; wire wrapper always constructs `Some(msg.field)` first, never `None` directly |
| `conversion_coverage.rs` | 6 (`:227,232,386,414,437,454`) | in-family (all 6) | CLEAN — `:227/:232` are my own round-8 `parse_conversion` tests (already bite-checked); `:386` self-identifies via interpolated `t.from`/`t.to`/`t.covered_by` in its panic text; `:414/437/454` populate disjoint vecs from disjoint scan logic, each panic interpolates the full offending list |
| `scene/shapes.rs` | 2 (`:161,179`) | in-family | CLEAN — `"expected exactly 3"` is `shapes.rs`'s sole `MeshTriangle`-length format string; `"only 1 vertices exist"` traces to the single physical site `moveit-geometry/src/shapes.rs:1133`, confirmed by grep not reproduced elsewhere reachable from `TryFrom<MeshMsg>` |
| `scene/planning_scene.rs` | 3 (`:234,251,287`) | in-family | CLEAN — `:234`'s only reachable emptying branch is `apply_octomap`'s `remove_all`-shaped path for this fixture; `:251/:287` already sibling-documented and grep-confirmed unique |
| `scene/attached.rs` | 8 direct (`:442,452,532,553,554,612,649,671`) + 2 hidden `assert_err_mentions` (`:513,594`) | in-family except `:532` | `:532` **not-this-family** (clause 2/3) — ADD-path `merged_touch_links` is a straight `.collect()` with the merge branch gated `if !is_add`, i.e. never entered; the real "replace not merge" claim rides on the adjacent `shapes().len()==1` assertion instead. Everything else CLEAN, including `:553/:554` (`BTreeSet::contains` is exact-element not substring match, and `moveit-scene`'s `attach_new`/`AttachedBody` store touch_links verbatim with no auto-inclusion of `link_name`, ruling out the fixture's own "tip" link name as a spurious source) |
| `scene/collision_object.rs` | 8 direct (`:635,710,771,880,893,1016,1050,1089`) + 2 hidden `assert_err_mentions` (`:900,948`) | in-family (all 10) | CLEAN, with one flagged **latent risk, not a live collision**: `:1089`'s message is produced by two physical call sites inside `apply_move` (object-pose parse, then shape-repose parse — both delegate to the same generic quaternion-norm check), and only the shape-repose one is reachable because this test's `mv.pose` stays `identity_pose()`; a future edit that also corrupts `mv.pose` would misattribute to the wrong branch. Same accepted "generic Pose rule, one message, N callers" pattern already named elsewhere in the crate (`position.rs`/`orientation.rs`'s §211/§213 comments) — noted for `apply_move` specifically for the first time, not fixed (no live test exercises it wrong today, so there is nothing to bite-check) |
| `state.rs` | 11 hidden `assert_err_mentions` (`:259,283,310,333,355,370,389,408,436,454,472`) | in-family (all 11) | CLEAN — `set_parallel_array`'s `{field}`-prefixed length/unknown-name messages keep position/velocity/effort textually distinct; the four `multi_dof_joint_state` sites intentionally share one message, discriminated by guard-clause mutation already bite-checked last round, not by text |
| `trajectory.rs` | 6 hidden `assert_err_mentions` (`:297,314,337,360,398,421`) | in-family (all 6) | CLEAN — length-mismatch messages are `{field}`-interpolated; `:297`/`:421` share one needle by design (same branch, redundant coverage, not a collision) |
| `planning.rs` | 2 hidden `assert_err_mentions` (`:427,440`) | in-family (both) | CLEAN — `TryFrom<PlanningRequestMsg>`'s only two `Error::Other` sites, already named as siblings in the function's own doc comment |

**Totals: 62 sites examined, 60 in-family, 2 not-this-family
(`constraints/set.rs:148`, `scene/attached.rs:532`), 0 collisions, 1 latent
risk flagged but not live (`scene/collision_object.rs:1089`).**

### Needles collided: 0

No fix was owed under this round's instructions ("a colliding needle is a
finding — narrow it to text unique to the target branch... Prove each fix
the §3a way") because no needle collided. The two `not-this-family` calls
follow the same clause-2/clause-3 reasoning the census itself already
established for `plan_responses.rs` and `shortest_solution_is_none_on_
empty_input` — neither is a new category, both are direct re-applications.
No commit this section (nothing to fix); `ros/verify-ros-interop.sh` was
re-run at the end of the round regardless, to confirm the working tree
still gates clean after the `git merge main` fast-forward and this
doc-only addition (fmt/clippy/`cargo test` 170/170/`cargo doc`, all pass —
see the round's own report for the exact run).

## §11. `moveit-distance-field` wide-grammar sweep (round 10)

Fence for this round: `crates/moveit-distance-field` in the root
`[workspace]` (not `ros/moveit-ros`), assigned after a two-panel collision
on `crates/moveit-planning` was traced to a stale hand-off (a `last_note`
read as a fresh assignment) and resolved by redrawing the round's fence
rather than reverting the one commit already made under it. `ros/` stays
closed at 63 per §10 unless something new merges into it.

Enumerated with `python3 tools/ci/count-coarse-assertions.py
crates/moveit-distance-field` on `main` post-`f10e1bd`/`6792ef1`
(`contains_msg`/`contains_member` merged into one `contains` kind;
`None`/`Err(..)` now score only as a top-level equality-macro operand,
which silently retired 5 false-positive `eq_none`/`eq_err` tags this
sweep hit under the pre-fix instrument — `collision_env_distance_field.rs`
`:2740/:2751/:2769`'s `.is_none()` calls and `registry.rs`-shaped
`matches!`/`Err(` combinations elsewhere had `None,`/`Err(` matching
inside a function-call argument list, not a real `assert_eq!(x, None)`).
**39 sites**, all `test` scope, 0 `helper_body`. Confirmed independently
against a hand-derived figure (34, under the pre-merge instrument) before
the fence redraw folded in the 5 sites a since-superseded cross-crate
carve-out (`contains_msg`/`via:` sites anywhere in `crates/` belong to
p1-robotmodel) had held out — `oracle_parity.rs:296,304`
(`via:check_scenario`) and `voxel_grid.rs:454,456,512` (rendered
error-message `contains`, now folded into the merged kind). With the crate
under single ownership this round, that carve-out no longer has a second
owner to route to, so all 39 are enumerated here.

| file:line | kind | verdict | evidence |
|---|---|---|---|
| `collision_distance_field_types.rs:1410-1414` (5) | `is_empty` | **not-this-family** (clause 2) | `GradientInfo::clear()` unconditionally clears each field (`.clear()`/reassignment, no guard) — no decision, just five independent unconditional resets; `gradient_info_clear_does_not_clear_types` is a state-transition check, not a branch discrimination |
| `collision_env_distance_field.rs:2368` | `is_empty` | **not-this-family** (clause 1) | `assert_eq!(index_map.contains_key(...), !link_model.shapes().is_empty(), ...)` compares two presence/membership booleans against an independently-computed oracle value, not a coarse fail/absence signal |
| `collision_env_distance_field.rs:3081` (orig `:3068`) | `is_empty` | **in-family, BLIND — FIXED** (`9c42c11`) | `generate_distance_field_cache_entry_never_populates_attached_bodies_when_acm_is_none` attached its fixture to `r_gripper_palm_link`, which resolves to zero shapes under `MeshSearchPaths::none()` — the `if !link.shapes().is_empty()` gate (`:566`) already empties `attached_body_names` on its own, so the test never touched the `acm == None` gate (`:574`) it names. Bite-check: moving the attached-body population loop outside the `acm` gate left this test green. Fix: retargeted the fixture at `r_gripper_motor_accelerometer_link` (has shapes, per the sibling `acm_is_some` test's own comment) — re-ran the same mutation, now fails correctly and only this test |
| `collision_env_distance_field.rs:3127` (orig `:3114`) | `is_empty` | in-family, CLEAN | `excludes_an_attached_body_on_a_non_group_link` — bite-checked: neutralizing the `filter(|ab| ab.link_name == link_name.as_str())` match to accept everything makes this test fail and only this test |
| `collision_env_distance_field.rs:3158` (orig `:3145`) | `is_empty` | **not-this-family** (clause 3) | Explicitly labeled `"test precondition: ..."` in its own panic text — asserts about the fixture model, not `generate_distance_field_cache_entry` |
| `collision_env_distance_field.rs:3414` (orig `:3401`) | `is_empty` | **not-this-family** (clause 2) | `group_state_representation`'s attached-body loop (`:1009-1033`) sets `sphere_locations` unconditionally, unlike the link loop's real `if !dfce.link_has_geometry[i]` gate — no branch governs this assertion's emptiness within the function under test |
| `collision_env_distance_field.rs:3775` (orig `:3762`) | `is_some` | in-family, CLEAN — single-branch | Sibling of `:3770`'s `is_none()` (`generate_distance_field: false` → no field); `generate_distance_field_cache_entry` has exactly one `distance_field:` construction site (already established and bite-checked at `:3745`'s own "ASSERTION-DISCRIMINATION AUDIT (round 2)" comment) |
| `collision_env_distance_field.rs:4451` (orig `:4438`) | `is_empty` | in-family, CLEAN | `attached_body_on_an_out_of_group_link_is_invisible_to_collision_checks`, via `check_self_collision` → same filter as `:3127`; bite-checked together (combined mutation run) — this test stayed green, confirming it's insensitive to the acm gate, sensitive only to the same filter mismatch |
| `collision_env_distance_field.rs:4500` (orig `:4487`) | `is_empty` | in-family, CLEAN | `attached_body_is_invisible_when_acm_is_none`, via `check_self_collision`, fixture link `"mid"` has real inline-box shapes (`two_link_model_and_srdf`) unlike `:3068`'s original bug — bite-checked together with `:3127`/`:4451`: this test failed under the acm-gate mutation, the other two stayed green |
| `collision_env_distance_field.rs:4920` (orig `:4907`) | `contains` | in-family, CLEAN | `get_intra_group_proximity_gradients_updates_an_attached_bodys_gradient_slot` — bite-checked: dropping the `types[k] = CollisionType::Intra` writes in `get_intra_group_proximity_gradients` (`:2092,2097`) failed only this test plus the downstream oracle-parity test measuring the same field, nothing else |
| `find_internal_points.rs:91` | `is_empty` | in-family, CLEAN — single-branch | `every_returned_point_is_inside_the_body` — bite-checked: gating `body.contains_point` to always-false failed this test plus the two other tests in the same file/parity suite that also depend on `find_internal_points_convex`'s output, nothing unrelated |
| `voxel_grid.rs:454,456` | `contains` (was `contains_msg`) | in-family, CLEAN — folded operand | `rejects_non_positive_resolution` checks zero and negative resolution both hit `GridGeometry::new`'s single `!(resolution.is_finite() && resolution > 0.0)` guard (`:94`) — one guard, one message, two invalid operands; matches `doc/folded-operand-guards.md`'s accepted shape, not a collision to fix |
| `voxel_grid.rs:512` | `contains` (was `contains_msg`) | in-family, CLEAN | `new_rejects_a_pathologically_fine_resolution_on_the_y_axis` — already carries its own prior-round isolation trail (`:496-509`'s comments: "pins that the guard applies per-axis, not just to size.x", cross-referencing the sibling `x`-axis overflow test) |
| `collision_common_distance_field_parity.rs:319` | `is_empty` | **not-this-family** (clause 3) | `"fixture link {} has no collision geometry on this port -- pick a different link"` — fixture-selection precondition, not a decision in the code under test |
| `collision_env_distance_field_parity.rs:{211,403,551,758,1253,1471,1748,1920}`, `collision_env_hybrid_parity.rs:{236,410}` (10) | `is_empty` | **not-this-family** (clause 3), all 10 | `model.diagnostics().is_empty()` guards against a silently-narrowed comparison from a failed mesh resolve — a precondition for the real assertions below it, not a decision belonging to the function under test |
| `collision_env_distance_field_parity.rs:{397,550,752}` (3) | `is_empty` | **not-this-family** (clause 3), all 3 | `"fixture must carry at least one case"` — fixture-loading precondition |
| `collision_env_distance_field_parity.rs:479,637` | `is_some` | in-family, CLEAN, both | `assert_eq!(entry.distance_field.is_some(), expected.has_field, "has_field")` — per-request oracle-driven comparison, same single-construction-site mechanism as `:3775` |
| `collision_env_hybrid_parity.rs:347` | `is_empty` | in-family, CLEAN | Aggregate re-statement of per-element `assert_eq!(gradient.collision, ...)` checks already run above in the same loop; redundant-by-design, not blind |
| `collision_env_hybrid_parity.rs:540` | `is_empty` | in-family, CLEAN — pre-existing audit | Own comment names it: `"the second, free refuting result this round's brief calls out"` — deliberately kept as an explicit boundary check alongside the primary `assert_eq!(differing_links, F2_COLLIDING_LINKS, ...)`, from an earlier round of this same sweep |
| `oracle_parity.rs:296,304` | `via:check_scenario` | in-family, CLEAN, both | Helper body (`:264,:279`, `scope: helper_body`, excluded from this count) already carries its own "ASSERTION-DISCRIMINATION AUDIT (round 2)" comment with bite-check evidence for `nearest_cell`'s five `voxel: None` sites — pre-existing, not redone |
| `shape_points_parity.rs:209` | `is_empty` | in-family, CLEAN | `missing.is_empty() && extra.is_empty()` set-difference parity check — confirmed by the same `find_internal_points.rs:91` bite-check above (this test failed under that mutation too) |

**Totals: 39 sites, 17 in-family (1 blind, fixed; 16 clean), 22
not-this-family (5 clause-2 unconditional-reset, 1 clause-2
unconditional-population, 1 clause-1 membership/oracle-comparison, 13
clause-3 fixture/test preconditions), 0 unresolved.**

One commit this section: `9c42c11` (the blind-fixture fix above). Gated
`-p moveit-distance-field`: `cargo fmt --all`, `cargo clippy -p
moveit-distance-field --all-targets -- -D warnings` (clean), `cargo
nextest run -p moveit-distance-field` (138/138 after the fix; every
bite-check mutation in this section was reverted before the next one and
the working tree was clean before this commit and remains clean after
it).
