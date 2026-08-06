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
| `moveit-model` | 5+15=20 | 5+17=22 | +2 | `72f5eca` (merged after `df36fab`) added two sites: `46cd26b`'s new test `get_end_effector_unknown_name_is_an_error` (`robot_model.rs:2856`) and `7676185`'s added `j4` mimic check when the cycle-clear fixture was widened from 3 joints to 4 (`robot_model.rs:2096` (`mimic_mutual_cycle_clears_every_mimic_in_the_model`)). Real new-test growth, not an instrument miss. |

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
| planning.rs:561 | bare | converts_minimal_request | fixture-collapse-fixed | `9d829a8` (own earlier commit) |
| planning.rs:1033 | matches! | multi_dof_joint_trajectory_is_rejected_not_silently_dropped | single-branch | bite run just now — only `Error::Other` site in this impl; delegate produces `Error::Construct` |
| scene/collision_object.rs:1026 | bare | append_without_subframe_data_clears_existing_subframes | discriminating | §9 all three clauses hold — clause 1: `subframe_pose("tip").is_none()` is a genuine absence signal (retained vs. cleared); clause 2: `apply_add`'s unconditional subframe-replace is a written decision an engineer could gate on non-empty (bite: wrapping it in `if !subframes.is_empty()` flips the assertion); clause 3: deleting the second `apply_collision_object` call leaves the first object's subframe in place, changing the outcome. Bite run once, before I knew the ros gate was paid (docker, targeted single test — see cost note in the round report) |
| scene/mod.rs:94 | matches! | unresolvable_non_empty_frame_id_is_still_rejected | single-branch | bite run just now — `header_frame_transform`'s only Err path is the single `scene.frame_transform` call |
| scene/planning_scene.rs:1364 | matches! | unresolvable_non_empty_header_frame_id_is_still_rejected | single-branch | bite run just now — `Error::UnknownName` reachable only via `header_frame_transform` (see scene/mod.rs:94); the file's other two error sites are `Error::Other`/`Error::Construct`, different variants |
| scene/shapes.rs:203 | matches! | plane_wrong_coef_length_is_rejected | single-branch | bite run just now — single `Error::construct` site in `TryFrom<PlaneMsg> for Plane`; `Plane::new` is infallible |
| trajectory.rs:383 | matches! | add_suffix_way_point_rejects_a_nonzero_first_dt | single-branch | bite run just now — `moveit_trajectory::RobotTrajectory::add_suffix_way_point`'s single `first_duration_error()` site |
| trajectory.rs:482 | matches! | seconds_to_duration_rejects_just_above_i32_max_seconds | single-branch | bite run just now — `seconds_to_duration`'s one combined guard |
| trajectory.rs:488 | matches! | seconds_to_duration_rejects_negative | single-branch | bite run just now — same guard |
| trajectory.rs:494 | matches! | seconds_to_duration_rejects_nan | single-branch | bite run just now — same guard |
| trajectory.rs:500 | matches! | seconds_to_duration_rejects_infinity | single-branch | bite run just now — same guard |
| trajectory.rs:528 | matches! | negative_cumulative_duration_from_an_unvalidated_trajectory_is_rejected | single-branch | bite run just now — `TryFrom<RobotTrajectory> for JointTrajectoryMsgOut`'s sole `?` site is `seconds_to_duration` |

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
| collision_env_distance_field.rs:3284 | bare | (post-update link_body_decompositions None) | single-branch | `7e69a8c` |
| collision_env_distance_field.rs:3289 | bare | (post-update link_distance_fields None) | single-branch | `7e69a8c` |
| collision_env_distance_field.rs:3747 | bare | (generate_collision_checking_structures: generate_distance_field=false) | single-branch | `202c88f` |
| collision_env_distance_field.rs:3770 | bare | (generate_collision_checking_structures: second test's first call, same generate_distance_field=false reasoning as `:3747`) | single-branch | `202c88f` |
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
that test's call path), except `joint/urdf.rs:388` (`spherical_joint_type_is_rejected`) and the mimic sites
(`robot_model.rs:2052,2053,2054,2078,2110` -- fixture literal `<link name="base"/>` at `:2052`, `joint/model.rs:1040,1047` (`mimic_set_get_clear_round_trip`)),
which round 6's report (`01KZ7P9B03SWRKQWQV1DXB4EEA-7.md`, absolute path
under `.caucus/sessions/`, not reachable from this worktree — `.caucus` is
gitignored) already covered; those were re-bitten this round rather than
cited blind, and agree with the prior report. `robot_model.rs:2055` (`mimic_mutual_cycle_clears_every_mimic_in_the_model`) (j4)
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
| robot_model.rs:2003 | bare | (joint_model_mut unknown name) | single-branch | bite run just now — `joint_model_mut` has one Err site |
| robot_model.rs:2092 | matches! | mimic_mutual_cycle_clears_every_mimic_in_the_model | discriminating | bite run just now — `[Diagnostic::MimicCycle]` slice pattern requires both exact length 1 and the named variant |
| robot_model.rs:2093 | bare | mimic_mutual_cycle_clears_every_mimic_in_the_model (j1) | fixture-collapse-fixed | §9's own worked example (census §9, "Worked resolution of the disputed post-build state check") is this exact site: all three clauses hold — clause 1 (`.mimic().is_none()` collapses "never had one" and "had one, cleared" into one signal), clause 2 (the build/clear routine's per-joint null decision), clause 3 (belongs to the routine this test names). `7676185` — fixture gave j1 a mimic outside the cycle; two isolating mutations (narrow-to-cycle-members, skip-clear-entirely) both now fail here |
| robot_model.rs:2094 | bare | (j2) | discriminating | §9 same three clauses as :2052 (identical mechanism/decision/subject — same routine, same getter). `01KZ7P9B03SWRKQWQV1DXB4EEA-7.md` + re-bitten — skip-clear-entirely fails here (clause 2's decision is real for j2); narrow-to-cycle-members does NOT fail here (j2 is itself a cycle member, cleared under any correct scoping) — in-family and non-vacuous for "clearing happens", just not independent evidence for the whole-model-vs-cycle-only scope claim, which only j1/j4 carry |
| robot_model.rs:2095 | bare | (j3) | discriminating | same as :2053 — §9 clauses hold identically; `01KZ7P9B03SWRKQWQV1DXB4EEA-7.md` + re-bitten, same caveat |
| robot_model.rs:2096 | bare | (j4, new in `7676185`) | fixture-collapse-fixed | §9 same three clauses as :2052. `9661d4c` (this round) — was blind under every mutation tested (no `<mimic>` tag of its own, same defect as pre-fix j1); fixed by giving j4 a real out-of-cycle mimic on new leaf joint j5, re-verified failing under the narrow-scope mutation post-fix. Never bitten before this round |
| robot_model.rs:2119 | bare | mimic_of_unknown_joint_is_dropped_with_a_diagnostic | discriminating | §9 same three clauses — different routine (single-joint clear on `MimicUnknownJoint`), same mechanism/decision/subject shape. `01KZ7P9B03SWRKQWQV1DXB4EEA-7.md` (as old line 2010) + re-bitten — mutated the single-joint clear to a no-op; assertion fails |
| robot_model.rs:2151 | bare | mimic_with_mismatched_dof_is_dropped_with_a_diagnostic | discriminating | §9 same three clauses (`MimicDofMismatch` sibling of :2078). `01KZ7P9B03SWRKQWQV1DXB4EEA-7.md` (as old line 2042) + re-bitten — same mutation, same result |
| robot_model.rs:2533 | bare | mesh_collision_resolving_to_an_unreadable_file_is_skipped_with_a_diagnostic | not-this-family | §9 clause 3 — the census's own worked example, verbatim: `assert!(std::fs::read(&path).is_err(), "precondition: ...")`; the test calls `std::fs::read` itself, no subject code runs before this assertion |
| robot_model.rs:2605 | matches! | mesh_collision_resolving_to_a_valid_stl_file_builds_a_mesh_shape | not-this-family | §9 clause 1 — `shapes[0].shape` read after `.expect("builds")`; `Shape::Mesh(_)` is a computed success-path enum tag, same shape as `JointKind::Fixed` |
| robot_model.rs:2837 | bare | (get_end_effector("arm"), real group but not an end effector) | single-branch | bite run just now — despite the doc comment naming two conceptual causes, `get_end_effector` is one `.filter(...).ok_or_else(...)` chain: exactly one `Error::unknown_name` construction site |
| robot_model.rs:2856 | bare | get_end_effector_unknown_name_is_an_error (new in `46cd26b`) | single-branch | bite run just now — same single call site as 2796 |
| robot_model.rs:3080 | bare | group_state_where_every_joint_value_is_unusable_stores_no_state_at_all | single-branch | bite run just now — `variable_default_positions` is a one-line `self.default_states.get(name)` delegation (`joint_model_group.rs:257`) |
| robot_model.rs:3090 | bare | variable_default_positions_returns_none_for_unknown_state_name | single-branch | bite run just now — same delegation |

Corrected this round in two passes (see verdict-taxonomy note above): first,
9 sites were reclassified out of `not-this-family` on the theory that a
post-build state check is categorically outside the family — re-bitten
individually, not reasoned from shape. Second, applying census §9's formal
three-clause test to all 13 originally-`not-this-family` model rows (not
just the 9 already flagged) found one of those 9 was itself too coarse:
`joint/model.rs:1040` (`mimic_set_get_clear_round_trip`) (the *pre-set* `mimic_set_get_clear_round_trip`
assertion) fails §9 clause 3 — it reads a value the subject never decided,
since it runs before either `set_mimic` or `clear_mimic` — and moves back to
`not-this-family`. `:1047` (the post-*clear*-set assertion in the same test)
passes all three clauses and stays `discriminating`. `robot_model.rs:2052` (`mimic_mutual_cycle_clears_every_mimic_in_the_model`)
(j1) is census §9's own worked example verbatim; `robot_model.rs:2055` (`mimic_mutual_cycle_clears_every_mimic_in_the_model`) (j4)
was blind under every mutation tried and had never been bitten before —
fixed this round by `9661d4c`.

Final breakdown: 6 discriminating, 8 single-branch, 2 fixture-collapse-fixed
(`7676185` for j1, `9661d4c` for j4, this round), 6 genuinely
`not-this-family` under §9 (`joint/model.rs:1040` (`mimic_set_get_clear_round_trip`) clause 3,
`joint/urdf.rs:366,372,378` (`fixed_floating_and_planar_produce_the_matching_kind`) clause 1 `JointKind` variants,
`robot_model.rs:2492` (`mesh_collision_whose_package_is_not_in_any_search_path_is_skipped_with_a_diagnostic`) clause 3 fixture precondition, `robot_model.rs:2564` (`mesh_collision_resolving_to_an_unreadable_file_is_skipped_with_a_diagnostic`)
clause 1 `Shape::Mesh` variant). 6+8+2+6 = 22. No site here reads as a
hidden D6 finding: `robot_model.rs:2796` (`end_effector_wires_name_and_falls_back_to_fewest_joints_parent`)/`2815`'s doc comment flags two
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
`robot_model.rs:2052,2053,2054,2055,2078,2110` (fixture literal `<link name="base"/>` at `:2052`), `joint/model.rs:1040,1047` (`mimic_set_get_clear_round_trip`),
`scene/collision_object.rs:1026` (`append_without_subframe_data_clears_existing_subframes`). Reclassified by isolating mutation (see the
two crate tables for per-site evidence), not by re-reasoning from shape.

Second pass: with `not-this-family` formally defined as census §9 (three
clauses — mechanism, decision, subject; merged `95eb25b`, authored by
p1-robotmodel), re-checked all 17 originally-`not-this-family` rows against
the clauses directly rather than accepting the first pass's 9-reclassified/
7-confirmed/1-uncounted split as given. One row was too coarse:
`joint/model.rs:1040` (`mimic_set_get_clear_round_trip`) had been swept into the first pass's 9 as
`discriminating`, but fails §9 clause 3 (it reads a value — `mimic()`
before either `set_mimic` or `clear_mimic` runs — that the subject never
decided) and moves back to `not-this-family`. Every other row's clause
result agrees with the first pass's bite-check result, including
`robot_model.rs:2052` (`mimic_mutual_cycle_clears_every_mimic_in_the_model`), which turns out to be census §9's own worked example
verbatim.

Final verdicts: 17 discriminating (4 ros + 7 distance-field + 6 model), 38
single-branch (19 ros + 11 distance-field + 8 model), 3 fixture-collapse-fixed
(1 ros pre-existing + 2 model: `7676185`, `9661d4c`), 9 `not-this-family`
under §9 (3 ros clause-1 success-path checks + 6 model: `joint/model.rs:1040` (`mimic_set_get_clear_round_trip`)
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
defined separately per file (`ros/moveit-ros/src/state.rs:192` (`assert_err_mentions`), `ros/moveit-ros/src/trajectory.rs:248` (`assert_err_mentions`),
`planning.rs:385` (`assert_err_mentions`), `scene/attached.rs:323` (`assert_err_mentions`), `scene/collision_object.rs:533` (`assert_err_mentions`)
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

Reformatted this round (round p9-ros orphan-reconciliation): the original
table's first column named the file only, with the actual line numbers one
column over inside a `` count (`:l1,l2,...`) `` cell — a shape
`tools/ci/reconcile-assertion-ledgers.py`'s `FIRST_COL_RE` cannot parse (it
requires `file:line[,line...]` in the first cell itself), so every site
below silently reported as an orphan under the committed instrument despite
having a row here the whole time. Reformatted so column 1 is `file:line,...`
directly; the per-file site count that used to open column 2 is now stated
in prose inside the evidence column instead of dropped.

Re-anchored (`group_name` fix, this file's `state.rs`/`planning.rs` rows
only): filling `MotionPlanResponse.group_name` shifted both files, so all 16
citations here moved — `state.rs` +24, `planning.rs` +24 above old `:306`
and +60 above old `:462`. Each new line was confirmed by matching the row's
own named test function to its current line, never by nearest-line
proximity: 7 of the 16 stale citations still window-matched a *neighbouring*
site (old `:283`'s citation landed exactly on what is now old `:259`'s
assertion), so `--verify` flagged only 9 of them and the other 7 would have
read as correct while pointing one test off.

Re-anchored again (§236's expiry tripwires, `planning.rs` rows only): the two
`*_boundaries_are_not_observable_on_the_core_request` tests and the module-doc
bullet that points at them shifted this file twice over — `git diff -U0` gives
exactly two hunks, `@@ -22 +22,7 @@` and `@@ -486,0 +493,112 @@`, so every
citation moved **+6** and those below old `:486` moved **+118**. Old
`:421,451,464` -> `:427,457,470`; old `:568,581` -> `:686,699`; the prose
citations of the `assert_err_mentions` helper definition (`:355` -> `:361`) and
of the folded-operand test's own comment (`:571-574` -> `:689-692`) moved with
them. Confirmed the same way as above, by matching each row's own named test
function to the line: `--verify` flagged 5 of the 5 planning.rs citations this
time (a uniform +6 exceeds `NEARBY_WINDOW = 5`, so none of them could
window-match a neighbour and read as correct), and the 11 `state.rs` rows did
not move at all.

| file:line | in-family verdict | collision verdict |
|---|---|---|
| `constraints/orientation.rs:194-196` (`invalid_parameterization_is_rejected`), `constraints/orientation.rs:217-219` (`degenerate_orientation_is_rejected`), `constraints/orientation.rs:244-246` (`orientation_norm_2_is_rejected_end_to_end_unlike_a_scene_pose`) | in-family | CLEAN (3 sites) — pre-existing sibling-collision comments confirmed correct by reading |
| `constraints/position.rs:308,392,411,432,456,472` | in-family | CLEAN (6 sites) — `dim()`'s `{field}`-interpolated messages (e.g. `:308`'s own `.contains("BOX_Y")`), `PositionConstraint::new`'s three distinct `Error::construct` texts, and the meshes-guard's shared-but-same-branch needle all verified against `crates/moveit-constraints/src/position.rs` |
| `constraints/set.rs:148` | **not-this-family** (clause 2) | n/a — `empty_constraints_is_empty_set` is the census's own vacuous-accumulator shape verbatim: all four input vecs empty, none of the four `for` loops in `KinematicConstraintSet::try_from` iterate, `set` is never touched |
| `constraints/visibility.rs:384,385` (`negative_target_radius_activates_but_negative_angles_stay_inactive`) | in-family | CLEAN (2 sites) — `normalize_angle_criterion`'s `>EPS` filter is the sole physical producer of `None` for each field; wire wrapper always constructs `Some(msg.field)` first, never `None` directly |
| `conversion_coverage.rs:227,232,386,414,437,454` | in-family (all 6) | CLEAN — `:227/:232` are my own round-8 `parse_conversion` tests (already bite-checked); `:386` self-identifies via interpolated `t.from`/`t.to`/`t.covered_by` in its panic text; `:414/437/454` populate disjoint vecs from disjoint scan logic, each panic interpolates the full offending list |
| `scene/shapes.rs:161-163` (`mesh_triangle_with_wrong_vertex_count_is_rejected`), `scene/shapes.rs:179-181` (`mesh_out_of_range_vertex_index_is_rejected`) | in-family | CLEAN (2 sites) — `"expected exactly 3"` is `shapes.rs`'s sole `MeshTriangle`-length format string; the "only N vertices exist" wording traces to the single physical site `moveit-geometry/src/shapes.rs:1133` (source: `"mesh triangle references vertex {idx}, but only {n} vertices exist"`), confirmed by grep not reproduced elsewhere reachable from `TryFrom<MeshMsg>` |
| `scene/planning_scene.rs:1079` (`empty_collision_objects_and_empty_octomap_is_a_no_op`), `scene/planning_scene.rs:1096-1099` (`non_octree_octomap_type_is_rejected`), `scene/planning_scene.rs:1132-1135` (`truncated_octree_payload_is_rejected`) | in-family | CLEAN (3 sites) — `:1079`'s only reachable emptying branch is `apply_octomap`'s `remove_all`-shaped path for this fixture; `:1096/:1132` already sibling-documented and grep-confirmed unique |
| `scene/attached.rs:442,452,513,532,553,554,594,612,649,671` | in-family except `:532` | `:532` **not-this-family** (clause 2/3) — ADD-path `merged_touch_links` is a straight `.collect()` with the merge branch gated `if !is_add`, i.e. never entered; the real "replace not merge" claim rides on the adjacent `shapes().len()==1` assertion instead. Everything else CLEAN (8 direct + 2 hidden `assert_err_mentions` at `:513,594`), including `:553/:554` (`BTreeSet::contains` is exact-element not substring match, and `moveit-scene`'s `attach_new`/`AttachedBody` store touch_links verbatim with no auto-inclusion of `link_name`, ruling out the fixture's own "tip" link name as a spurious source) |
| `scene/collision_object.rs:635,710,771,880,893,900,948,1016,1050,1108,1143` | in-family (all 11) | CLEAN (9 direct + 2 hidden `assert_err_mentions` at `:900,948`), including the pair this round corrected: `:1108`/`:1143` (`move_object_pose_with_malformed_pose_is_rejected`/`move_shape_repose_with_malformed_pose_is_rejected`) replace the old single `:1089` citation, which this table had flagged as "latent risk, not a live collision" — `4c56148` ("test(ros): reach apply_move's object-pose parse, close the :1089 gap") already fixed exactly that gap by adding the first test, and its own doc comment (`:1064-1087` in the live source) states the bite-check: neutralizing `apply_move`'s object-pose parse (`:478`) alone fails only `move_object_pose_with_malformed_pose_is_rejected`, neutralizing the shape-repose parse (`:515`) alone fails only `move_shape_repose_with_malformed_pose_is_rejected`. §13 (this ledger's own round-12 write-up) checked `4c56148`'s ancestry and wrongly treated the pre-existing `:1089` row as proof this was already ledgered — it was ledgered as *unfixed*, and the fix had already landed; §13 read "a row exists" as "nothing changed" without checking whether the row's own verdict still held. Corrected here, not re-bitten (the source comment's own trail is the bite-check) |
| `ros/moveit-ros/src/state.rs:283,307,334,357,379,394,413,432,460,478,496` | in-family (all 11) | CLEAN (11 hidden `assert_err_mentions`) — `set_parallel_array`'s `{field}`-prefixed length/unknown-name messages keep position/velocity/effort textually distinct; the four `multi_dof_joint_state` sites intentionally share one message, discriminated by guard-clause mutation already bite-checked last round, not by text |
| `ros/moveit-ros/src/trajectory.rs:297,314,337,360,398,421` | in-family (all 6) | CLEAN (6 hidden `assert_err_mentions`) — length-mismatch messages are `{field}`-interpolated; `:297`/`:421` share one needle by design (same branch, redundant coverage, not a collision) |
| `planning.rs:817,1046` | in-family (2 sites, was 3) | CLEAN (1 hidden `assert_err_mentions` at `:817` + `:1046`) — **re-derived after p11-startstate (`10f571f`), which invalidated this row's old `:481,494,1046` citation, not just shifted it.** The old `:481` site, `nondefault_start_state_is_rejected_not_silently_dropped`, tested a blanket "any non-default `start_state` field is rejected" branch that p11-startstate **deleted outright**: `start_state` is now representable via a `StartState` sum type (`CurrentState`/`Overriding`), so position/velocity/name are accepted, not rejected. The name survives only inside a comment at `planning.rs:812` (`crates/moveit-planning::start_state`'s module doc names the same split); the function is gone. The old `:494` site survives, renumbered: its `assert_err_mentions` call is now `planning.rs:817-820` (`nonempty_reference_trajectories_is_rejected_not_silently_dropped`). The row's other claim — "`:475/:488` are `TryFrom<PlanningRequestMsg>`'s only two `Error::Other` sites" — was already wrong *before* this merge (branch-point `:475/:488` landed on `let mut msg = valid_request(&model);` and `fn nonempty_reference_trajectories...`, not on any `Error::Other` construction); re-derived directly against the current body: `TryFrom<PlanningRequestMsg>::try_from` (`planning.rs:266`) now has exactly **one** own `Error::other` site, `planning.rs:278-281` (`reference_trajectories`) — the former second site did not move with a line shift, it moved to a *different* impl, `TryFrom<StartStateMsg>` (`planning.rs:118-166`), which has its own two `Error::other` sites: `planning.rs:150-154` (`attached_collision_objects`) and `planning.rs:162-166` (`multi_dof_joint_state`), backed respectively by `a_start_state_with_attached_collision_objects_is_rejected_not_silently_dropped` (`planning.rs:784-787`) and `a_start_state_with_multi_dof_joints_is_rejected_not_silently_dropped` (`planning.rs:773-776`). Those two tests' needles ("`start_state.attached_collision_objects is not representable`" / "`start_state.multi_dof_joint_state has no core`") are textually distinct from each other and from every other needle in this file — checked by reading, not re-run through the coarse-assertions scanner — but they are **new code this ledger has never censused**, not a re-derivation of the deleted `:481` site, so they are named here for visibility and left OUT of this row's site count and the Totals below; a future full `tools/ci/count-coarse-assertions.py` sweep should pick them up as new sites. `:1046` (`multi_dof_joint_trajectory_points_is_rejected_not_silently_dropped`) is unaffected by any of the above (unrelated impl, `TryFrom<RobotTrajectoryMsg>`) and did not shift; it remains a folded-operand sibling of `:1033`'s already-covered `multi_dof_joint_trajectory_is_rejected_not_silently_dropped`: the guard is `!mdjt.joint_names.is_empty() \|\| !mdjt.points.is_empty()` (one guard, two operands), and the test's own comment (`:1036-1039` in the live source, "round 8, folded-operand audit") already states `joint_names` had a test but `points` did not before this test existed — the accepted `doc/folded-operand-guards.md` shape, matched here rather than re-derived, this is not a new finding |

**Totals: 62 sites examined (62 pre-existing + `planning.rs:1046` (`multi_dof_joint_trajectory_points_is_rejected_not_silently_dropped`), added this
round, minus 1 for `planning.rs:481`'s `nondefault_start_state_is_rejected_not_silently_dropped`, deleted by p11-startstate `10f571f` — see the `planning.rs:817,1046` row above), 60 in-family, 2 not-this-family (`constraints/set.rs:148` (`empty_constraints_is_empty_set`),
`scene/attached.rs:532` (`add_replaces_existing_attached_body_instead_of_merging`)), 0 collisions, 0 latent risks flagged but not live
(`scene/collision_object.rs:1089` (`move_object_pose_with_malformed_pose_is_rejected`)'s risk was already closed by `4c56148`
before this round; the table above now says so instead of the opposite).**

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
(both call sites read `check_scenario(`) and `voxel_grid.rs:454,456,512`
(rendered error-message `contains("must be finite and positive")`, now
folded into the merged kind). With the crate
under single ownership this round, that carve-out no longer has a second
owner to route to, so all 39 are enumerated here.

| file:line | kind | verdict | evidence |
|---|---|---|---|
| `collision_distance_field_types.rs:1410,1411,1412,1413,1414` | `is_empty` | **not-this-family** (clause 2) | `GradientInfo::clear()` unconditionally clears each field (`.clear()`/reassignment, no guard) — no decision, just five independent unconditional resets; `gradient_info_clear_does_not_clear_types` is a state-transition check, not a branch discrimination. Reformatted this round from a `1410-1414` hyphen range, which `tools/ci/reconcile-assertion-ledgers.py`'s citation parser (deliberately, per its own comment) does not expand — it silently kept only `:1410`, orphaning `:1411-1414` under the committed instrument despite this row covering them the whole time |
| `collision_env_distance_field.rs:2368` (`only_links_with_shapes_get_a_decomposition`) | `is_empty` | **not-this-family** (clause 1) | `assert_eq!(index_map.contains_key(...), !link_model.shapes().is_empty(), ...)` compares two presence/membership booleans against an independently-computed oracle value, not a coarse fail/absence signal |
| `collision_env_distance_field.rs:3081` (orig `:3068`) | `is_empty` | **in-family, BLIND — FIXED** (`9c42c11`) | `generate_distance_field_cache_entry_never_populates_attached_bodies_when_acm_is_none` attached its fixture to `r_gripper_palm_link`, which resolves to zero shapes under `MeshSearchPaths::none()` — the `if !link.shapes().is_empty()` gate (`:566`) already empties `attached_body_names` on its own, so the test never touched the `acm == None` gate (`:574`) it names. Bite-check: moving the attached-body population loop outside the `acm` gate left this test green. Fix: retargeted the fixture at `r_gripper_motor_accelerometer_link` (has shapes, per the sibling `acm_is_some` test's own comment) — re-ran the same mutation, now fails correctly and only this test |
| `collision_env_distance_field.rs:3127` (`generate_distance_field_cache_entry_excludes_an_attached_body_on_a_non_group_link`) (orig `:3114`) | `is_empty` | in-family, CLEAN | `excludes_an_attached_body_on_a_non_group_link` — bite-checked: neutralizing the `filter(|ab| ab.link_name == link_name.as_str())` match to accept everything makes this test fail and only this test |
| `collision_env_distance_field.rs:3158` (`build_non_group_distance_field_includes_a_non_group_attached_body_as_an_obstacle`) (orig `:3145`) | `is_empty` | **not-this-family** (clause 3) | Explicitly labeled `"test precondition: ..."` in its own panic text — asserts about the fixture model, not `generate_distance_field_cache_entry` |
| `collision_env_distance_field.rs:3414` (`group_state_representation_builds_a_gradient_slot_for_an_attached_body`) (orig `:3401`) | `is_empty` | **not-this-family** (clause 2) | `group_state_representation`'s attached-body loop (`:1009-1033`) sets `sphere_locations` unconditionally, unlike the link loop's real `if !dfce.link_has_geometry[i]` gate — no branch governs this assertion's emptiness within the function under test |
| `collision_env_distance_field.rs:3775` (`generate_collision_checking_structures_rebuilds_when_a_distance_field_becomes_required`) (orig `:3762`) | `is_some` | in-family, CLEAN — single-branch | Sibling of `:3770`'s `is_none()` (`generate_distance_field: false` → no field); `generate_distance_field_cache_entry` has exactly one `distance_field:` construction site (already established and bite-checked at `:3745`'s own "ASSERTION-DISCRIMINATION AUDIT (round 2)" comment) |
| `collision_env_distance_field.rs:4451` (orig `:4438`) | `is_empty` | in-family, CLEAN | `attached_body_on_an_out_of_group_link_is_invisible_to_collision_checks`, via `check_self_collision` → same filter as `:3127`; bite-checked together (combined mutation run) — this test stayed green, confirming it's insensitive to the acm gate, sensitive only to the same filter mismatch |
| `collision_env_distance_field.rs:4500` (orig `:4487`) | `is_empty` | in-family, CLEAN | `attached_body_is_invisible_when_acm_is_none`, via `check_self_collision`, fixture link `"mid"` has real inline-box shapes (`two_link_model_and_srdf`) unlike `:3068`'s original bug — bite-checked together with `:3127`/`:4451`: this test failed under the acm-gate mutation, the other two stayed green |
| `collision_env_distance_field.rs:4920` (orig `:4907`) | `contains` | in-family, CLEAN | `get_intra_group_proximity_gradients_updates_an_attached_bodys_gradient_slot` — bite-checked: dropping the `types[k] = CollisionType::Intra` writes in `get_intra_group_proximity_gradients` (`:2092,2097`) failed only this test plus the downstream oracle-parity test measuring the same field, nothing else |
| `find_internal_points.rs:91` | `is_empty` | in-family, CLEAN — single-branch | `every_returned_point_is_inside_the_body` — bite-checked: gating `body.contains_point` to always-false failed this test plus the two other tests in the same file/parity suite that also depend on `find_internal_points_convex`'s output, nothing unrelated |
| `voxel_grid.rs:454,456` | `contains` (was `contains_msg`) | in-family, CLEAN — folded operand | `rejects_non_positive_resolution` checks zero and negative resolution both hit `GridGeometry::new`'s single `!(resolution.is_finite() && resolution > 0.0)` guard (`:94`) — one guard, one message, two invalid operands; matches `doc/folded-operand-guards.md`'s accepted shape, not a collision to fix |
| `voxel_grid.rs:512` | `contains` (was `contains_msg`) | in-family, CLEAN | `new_rejects_a_pathologically_fine_resolution_on_the_y_axis` — already carries its own prior-round isolation trail (`:496-509`'s comments: "pins that the guard applies per-axis, not just to size.x", cross-referencing the sibling `x`-axis overflow test) |
| `collision_common_distance_field_parity.rs:319` (`link_body_decomposition_matches_the_oracle`) | `is_empty` | **not-this-family** (clause 3) | `"fixture link {} has no collision geometry on this port -- pick a different link"` — fixture-selection precondition, not a decision in the code under test |
| `tests/collision_env_distance_field_parity.rs:211,403,551,758,1253,1471,1748,1920` | `is_empty` | **not-this-family** (clause 3), all 8 | `model.diagnostics().is_empty()` guards against a silently-narrowed comparison from a failed mesh resolve — a precondition for the real assertions below it, not a decision belonging to the function under test. Reformatted this round: the original row's `{...}`-braced list is not the parser's expected comma-list shape (`FIRST_COL_RE` expects `\d+(?:\s*,\s*\d+)*` directly after the colon, not a `{`), so every site here orphaned under the committed instrument despite the row covering them |
| `tests/collision_env_hybrid_parity.rs:236,410` | `is_empty` | **not-this-family** (clause 3), both | Same guard shape as the row above, split into its own file:line citation this round for the same parser reason |
| `tests/collision_env_distance_field_parity.rs:397,550,752` | `is_empty` | **not-this-family** (clause 3), all 3 | `"fixture must carry at least one case"` — fixture-loading precondition. Reformatted this round for the same `{...}`-brace parser reason as the row above |
| `collision_env_distance_field_parity.rs:479,637` | `is_some` | in-family, CLEAN, both | Both sites open with `assert_eq!(`; the full call is `assert_eq!(entry.distance_field.is_some(), expected.has_field, "has_field")` — per-request oracle-driven comparison, same single-construction-site mechanism as `:3775` |
| `collision_env_hybrid_parity.rs:347` (`check_robot_collision_distance_field_matches_the_oracle_robot_only_mode`) | `is_empty` | in-family, CLEAN | Aggregate re-statement of per-element `assert_eq!(gradient.collision, ...)` checks already run above in the same loop; redundant-by-design, not blind |
| `collision_env_hybrid_parity.rs:540` (`check_collision_distance_field_environment_branch_paired_control`) | `is_empty` | in-family, CLEAN — pre-existing audit | Own comment names it: `"the second, free refuting result this round's brief calls out"` — deliberately kept as an explicit boundary check alongside the primary `assert_eq!(differing_links, F2_COLLIDING_LINKS, ...)`, from an earlier round of this same sweep |
| `oracle_parity.rs:296,304` | `via:check_scenario` | in-family, CLEAN, both | Both sites read `check_scenario(`. Helper body (`:264,:279`, `scope: helper_body`, excluded from this count) already carries its own "ASSERTION-DISCRIMINATION AUDIT (round 2)" comment with bite-check evidence for `nearest_cell`'s five `voxel: None` sites — pre-existing, not redone |
| `shape_points_parity.rs:209` (`find_internal_points_convex_matches_the_oracle_for_every_shape`) | `is_empty` | in-family, CLEAN | `missing.is_empty() && extra.is_empty()` set-difference parity check — confirmed by the same `find_internal_points.rs:91` bite-check above (this test failed under that mutation too) |

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

## §12. Tree-wide needle-collision audit (round 11)

Fence this round: read-only, tree-wide, excluding `ros/moveit-ros` (§9/§10,
closed at 63) and `crates/moveit-distance-field` (§11, closed at 39, mine
to edit) — no edits permitted anywhere else. Pinned at `2a4bd2a`
(2026-08-05T13:38:34+09:00, this ledger's own prior commit). The
wide-grammar population is moving under other panels' concurrent work
(orchestrator reported 392 -> 398 in the hour this round ran), so every
count below is a snapshot at that commit, not a standing total — re-running
the instrument on a later `main` will not reproduce these figures exactly.

### Method, and what a read can and cannot establish

Static reading only: no bite-check mutation on any crate outside
`ros/moveit-ros`/`moveit-distance-field`, per this round's no-edit
constraint. Census §9g is the standing caveat on that: a read distinguishes
"this fixture targets guard B" from nothing, not from "this fixture reaches
guard B" — only a mutation settles reachability, and `MeshSearchPaths::
resolve` (a chain of `?`s collapsing to one undifferentiated `None`) is the
demonstrated case where reading got that wrong twice (once by p1-robotmodel,
once by the orchestrator's own inverse-trap mutation).

Every verdict in this section rests on the *weaker* of the two things a
read can establish: needle uniqueness as a property of the reachable
message set (does more than one message a test's call path can reach
contain this exact substring), not reachability of a specific guard for a
specific input. That is a real limit, not a formality — but it is a
different question from §9g's, and the corpus here is structurally
resistant to §9g's specific failure in a way `MeshSearchPaths::resolve`
is not: every site below is a message-*content* check
(`err.to_string().contains(text)` / `detail.contains(text)`), not a bare
`Err(_)`/`None` collapse. If a wrong guard fired instead of the traced one,
it would (by the guard sequences read) emit *different* text, and the
`.contains(needle)` assertion would fail. None of the reviewed tests
currently fail: every crate whose source was read this round was run
read-only (`cargo nextest run -p <crate>`, no edits) and is green —
`moveit-geometry` 141/141, `moveit-kinematics` 35/35, `moveit-state` 44/44,
`moveit-constraints` 100/100, `moveit-srdf` 48/48, `moveit-planners-chomp`
86/86, `moveit-planners-pilz` 145/145, `moveit-planners-stomp` 62/62,
`moveit-smoothing` 36/36, `moveit-trajectory` 112/112, `moveit-model`
134/134, `moveit-planning` 41/41 — **984/984**. That is evidence the traced
guard is what actually executes, but it is evidence by absence-of-failure
under the current fixture set, not a mutation-confirmed reachability
proof. No site in this round's corpus has `MeshSearchPaths::resolve`'s
shape, so §9g's specific blind spot does not apply to what was checked —
but the general limit, that reading cannot promote "targets" to "reaches,"
still holds for every verdict below.

### Corpus, independently enumerated

`python3 tools/ci/count-coarse-assertions.py crates tools` at `2a4bd2a`,
excluding `ros/`:

- `contains`-kind, non-`helper_body`: **141** sites. Reproduces the
  orchestrator's quoted figure exactly.
- `via:<fn>` helper-call sites: **12**. Reproduces the orchestrator's
  quoted figure exactly.

### The 78/63 split, and the rule behind it

The instrument's merged `contains` kind (`f10e1bd`) deliberately does not
distinguish rendered-error-message content checks from membership/range
checks (`bitflags.contains(Action::X)`, `(min..max).contains(&v)`,
`Vec.contains(&item)`) — "the split is a fact about the receiver's type and
no regex has types." That is a judgment call correctly left to a reader,
not a gap in the instrument. Rule applied: a site is message-shaped if its
receiver is an `Error`'s rendered text or a `String`/`&str` field
destructured from one (e.g. a `detail` matched out of an error enum
variant); it is non-message if the receiver is a collection or bitflag set.
Of the 141:

- **78 message-shaped** — in scope: could this exact substring also be
  produced by some *other* message a reachable branch could emit.
- **63 non-message** — out of scope for this audit. Concentrated in
  `moveit-collision` (15) and `moveit-scene` (12): both crates were read in
  full and confirmed to have zero message-shaped `contains` sites at all —
  every hit there is `Vec::contains`/bitflag `Action` membership.

### 27 flagged candidates, and how each was decided

A tree-wide `rg -F <needle>` over the 78 message-shaped needles flagged 27
whose exact literal text occurs more than once anywhere in the tree; the
other 51 are unique tree-wide by construction (a substring with exactly one
producer cannot collide with anything) and were not individually re-traced
beyond that uniqueness check. All 27 were traced by reading the guard
sequence of the function each test actually calls:

| file:line | needle | verdict | reachable-message evidence |
|---|---|---|---|
| `moveit-geometry/src/bodies.rs:4055` (`cylinder_negative_radius_is_an_error`) | `"radius"` | **CLEAN — reverses the round's seed** | `Cylinder::new(-1.0, 1.0)` and (`:4125`) `.set_padding(-2.0)` on `Cylinder::new(1.0, 1.0)` both funnel into `Cylinder::recompute` (`bodies.rs:1845` -- `fn recompute(&mut self`), two *separate ordered* guards (`radius_scaled < 0.0`, then `half_length < 0.0`), not a folded `\|\|` — distinct messages `"Cylinder radius must be non-negative."`/`"Cylinder length must be non-negative."`; `"radius"` is a substring of only the first. See "The seed reversal" below. |
| `moveit-geometry/src/bodies.rs:4066` (`cylinder_negative_length_is_an_error`) | `"length"` | CLEAN | Same `recompute`, same two guards; unique to the second. |
| `moveit-kinematics/src/chain.rs:469` + `tests/ik_fk_roundtrip.rs:281` | `"not a chain"` | CLEAN | `NewtonRaphsonSolver::new` -> `new_with_seed` -> `ChainInfo::build(model, group_name)?` — the exact same single guard (`chain.rs:149` -- `{group_name}' is not a chain`, inside `build`) both tests reach; confirmed by reading `new_with_seed`'s body, not assumed from the doc comment alone. |
| `moveit-kinematics/src/chain.rs:512` (`build_rejects_a_multi_dof_joint`) | `"DOF"` | CLEAN | One reachable `Err` site (`chain.rs:187` -- `{group_name}' includes joint`, inside `build`); the only other tree-wide occurrence of the substring is inside an `.expect()` panic message on an internal invariant, not a `Result` path this test's `unwrap_err()` could observe. |
| `moveit-state/tests/jacobian.rs:140,159` (both open `assert!(`, one line above their own `err.to_string().contains("not a chain")`) | `"not a chain"` | CLEAN | `jacobian()`'s only guard is `crates/moveit-state/src/state.rs:1127` (own doc comment reads `Isometry3::identity()`, inside `recompute_link_transforms_from`); the crate's other `"not a chain"` message (`dynamics.rs:448` -- reads `group_name:?} is not a chain`, inside `new`) belongs to `DynamicsSolver::new`, unreachable from `jacobian()`. |
| `moveit-state/tests/jacobian.rs:192` (`a_lone_floating_joint_is_a_trivial_chain_but_an_unsupported_joint_type`) | `"unsupported type"` | CLEAN | Single tree-wide producer (`crates/moveit-state/src/state.rs:1203` -- `self.0.first_variable_index[joint_index]`, inside `joint_transform`). |
| `moveit-trajectory/src/time_optimal_trajectory_generation.rs:1086,1113,1179,1569` | `"exceeding the"` | CLEAN | One producer (`:850`), gated by the function's own `raw_sample_count` bounds check. Re-derived after merging main's TOTG timing-loop upstream-bug commits, which shifted every line in this file; `:1564` (`b12b358`, "cover the NaN branch of the resample sample-count guard") is a new fourth call site added by that merge, same producer, same needle — added here rather than left uncited. |
| `moveit-trajectory/src/trajectory.rs:1434,1440,1446` (`upstream_test_relevant_zero_max_accelerations_invalidate_trajectory`) | `DISTINGUISHING_PHRASE` const | CLEAN | Pre-existing bite-check documented in the test's own comment (round-unspecified, prior to this sweep); independently re-confirmed against message #3's text (`Trajectory::create`'s three `Error::construct` sites, `:123/:165/:174`). |
| `moveit-trajectory/src/trajectory.rs:1473` (`upstream_test_time_step_zero_makes_trajectory_invalid`) | `"the time step is <= 0.0"` | CLEAN | Unique among the same three sites. |
| `moveit-trajectory/src/time_optimal_trajectory_generation.rs:1435` (`mimic_joint_group_is_a_typed_error_not_a_panic`) | `"4"`/`'7'` | **N/A** | Not a branch-discrimination needle — single reachable typed error for the active-vs-full variable-count mismatch; the numbers are formatted data, not branch-selection text. |
| `moveit-trajectory/src/time_optimal_trajectory_generation.rs:1496` | `DISTINGUISHING_PHRASE` const | CLEAN | Stale citation, corrected: the row's "negative assertion, rules a branch out" description matched an earlier version of this test, per the test's own doc comment ("bite-confirmed... against the old negative check") -- the live assertion is `message.contains(DISTINGUISHING_PHRASE)` (`"after integrateForward and integrateBackward"`), a positive check already bite-confirmed by its own author against the model-bounds-fallback branch's distinct `"invalid max_acceleration"` message. Line re-derived again after merging main's TOTG upstream-bug commits (was line 1482, this ledger's own prior round; shifted by that merge, not a repeat of the same staleness). |
| `moveit-trajectory/src/robot_trajectory.rs:876` (`velocity_acceleration_and_effort_columns_appear_when_the_waypoint_carries_them`) | `" pos "` | **N/A** | Checks `Display` column-header text, not an error message. |
| `moveit-trajectory/tests/robot_trajectory.rs:736,751,766` (drift-corrected this round: dropped `:704`, which is the out-of-range-index test's `"out of bounds"` check, not a `"dirty: None"` site -- only 3 `"dirty: None"` sites exist tree-wide, confirmed by `rg`) | `"dirty: None"` | **N/A** | `Debug`-format checks on a struct field, not error-branch discrimination; unique tree-wide regardless. Each cited line reads `assert!(`, one line above its own `debug.contains("dirty: None")`. |
| `moveit-trajectory/tests/ruckig_smoothing.rs:208` (`no_group_set_is_an_error`) | `"did not set the group"` | CLEAN | `apply_smoothing` -> `validate_group` is the only reachable producer; TOTG's identically-worded message (`time_optimal_trajectory_generation.rs:702` -- `Error::other("it looks like the planner`, inside `validate_group`) is a different module's function, unreachable from `apply_smoothing`. |
| `moveit-planners-chomp/src/cost.rs:436` | `"singular"` | CLEAN | `ChompCost::new`'s only "singular" message is its own (`cost.rs:177` -- `Error::other("quad_cost is singular`, inside `new`); `optimizer.rs:1359` (`Error::other("jacobian_jacobian_tranpose is singular")`, inside `calculate_pseudo_inverse`)'s belongs to an unrelated, unreachable function. |
| `moveit-planners-chomp/src/optimizer.rs:2386` | `"joint_costs has"` | CLEAN | Pre-existing test doc comment already correct: `calculate_smoothness_increments`'s own guard (`:377`) vs. `ChompCost::derivative`'s guard (`"joint_trajectory has..."`, textually distinct) — confirmed by reading both function bodies. |
| `moveit-planners-chomp/src/optimizer.rs:2455` | `"columns"` | CLEAN | Pre-existing test doc comment already correct; unique among `calculate_total_increments`'s 3 `Error::other` sites. |
| `moveit-planners-pilz/src/path_circle.rs:590` (`half_circle_from_center_has_no_determinable_plane`) | `"colinear"` | CLEAN | `PathCircle::new`'s only colinear-text guard is its own (`:293`); `circle_from_interim`'s (`:187`) belongs to a different, uncalled function. |
| `moveit-planners-stomp/src/filter_functions.rs:314` (`enforce_position_bounds_rejects_a_multi_variable_joint`) | `"world_joint"` | CLEAN | Data value (a joint name), not branch-selection text; pre-existing doc comment already documents this as the loop's only guard. |
| `moveit-smoothing/src/butterworth.rs:162,200` | `"scale_term_"`/`"feedback_term_"` | CLEAN | One producer each. |
| `moveit-smoothing/src/ruckig_filter.rs:613` | `"must each have length"` | CLEAN | `do_smoothing`'s own guard (`:268`); identical text in `reset` (`:331`, uncalled from `do_smoothing`) and `AccelerationFilter` (different struct entirely) is unreachable from this test's call path. |
| `moveit-srdf/tests/boundaries.rs:55` (`a_root_element_other_than_robot_is_an_error`) | `"robot"` | CLEAN | Already fixed and merged this session (`83f8ea0`, p1-joints, prior round) — re-confirmed, not re-fixed. |
| `moveit-constraints/tests/sampler.rs:78,120` (both open `assert!(`) | `"panda_joint1"`/`"panda_arm"` | CLEAN | `JointConstraintSampler::new`'s 2 guards format disjoint data (joint name vs. group name); neither substring appears in the other's message. |
| `moveit-distance-field/src/voxel_grid.rs:512` (`new_rejects_a_pathologically_fine_resolution_on_the_y_axis`) | `"size.y"` | CLEAN | Already resolved in this ledger's own §11 sweep; not re-derived, only reused. |

### Zero-collision, stated per crate

Per the round's instruction that an unstated absence is indistinguishable
from an unchecked one:

- **Zero collisions in `moveit-geometry`.** 4 message-shaped sites, all
  checked (`bodies.rs:4055,4066,4125` -- all three open `assert!(`, above,
  plus `:3953` unflagged/unique).
- **Zero collisions in `moveit-kinematics`.** 4 sites, all checked
  (`chain.rs:469,512,558` -- each opens `assert!(`, above; `:558`'s needle
  traced but not tabled — same "DOF" guard as `:512`, same verdict).
- **Zero collisions in `moveit-state`.** 3 sites, all checked
  (`jacobian.rs:140,159,192` -- all three open `assert!(`, above).
- **Zero collisions in `moveit-constraints`.** 2 `contains` sites checked
  (`crates/moveit-constraints/tests/sampler.rs:78,120`, both `assert!(`, above) plus all 8 `via:assert_err_mentions` sites
  (below).
- **Zero collisions in `moveit-srdf`.** 2 sites checked (`boundaries.rs:
  49,55`); `:55` already fixed by another panel this session, re-confirmed
  clean here.
- **Zero collisions in `moveit-planners-chomp`.** 12 sites; the 5 flagged
  (`cost.rs:436` (`new_rejects_a_singular_quad_cost`), `optimizer.rs:2386,2455` -- both `assert!(`) checked above by reading;
  `cost.rs:391,404` (both `assert!(`) and `crates/moveit-planners-chomp/src/trajectory.rs:712,727,748,766,925,968,1004`
  (7, all `assert!(`) unflagged/unique tree-wide, not individually re-traced beyond that.
- **Zero collisions in `moveit-planners-pilz`.** 5 sites; `path_circle.
  rs:590` checked above; `path_circle.rs:558` (`zero_radius_is_rejected`) and
  `trajectory_generator_ptp.rs:449` (`constructor_rejects_missing_joint_limits`),
  `trajectory_generator_ptp.rs:466` (`constructor_rejects_unknown_group`),
  `trajectory_generator_ptp.rs:495` (`constructor_rejects_a_group_missing_an_acceleration_limit`)
  (4) unflagged/unique, not individually re-traced.
- **Zero collisions in `moveit-planners-stomp`.** 1 site
  (`filter_functions.rs:314` (`enforce_position_bounds_rejects_a_multi_variable_joint`)), checked above.
- **Zero collisions in `moveit-smoothing`.** 11 sites; the 3 flagged
  (`butterworth.rs:162,200` -- both `assert!(`, `ruckig_filter.rs:613` (`do_smoothing_rejects_a_mismatched_length`)) checked above;
  `acceleration_filter.rs:466,525,542`, `butterworth.rs:153,172,183`,
  `ruckig_filter.rs:388,530` (all `assert!(`; 8) unflagged/unique, not individually
  re-traced.
- **Zero collisions in `moveit-trajectory`.** 24 sites (23 pre-existing +
  `time_optimal_trajectory_generation.rs:1569` (`resample_dt_over_a_nan_duration_is_rejected`), added by main's TOTG
  upstream-bug merge); the flagged subset (`crates/moveit-trajectory/src/robot_trajectory.rs:876` (`velocity_acceleration_and_effort_columns_appear_when_the_waypoint_carries_them`),
  `time_optimal_trajectory_generation.rs:1086,1113,1179,1435,1496,1569`
  (four read `.is_err_and(|e|`),
  `crates/moveit-trajectory/src/trajectory.rs:1434,1440,1446,1473` (all `assert!(`),
  `tests/robot_trajectory.rs:736,751,766` (drift-corrected, see §12's own
  table row), `tests/ruckig_smoothing.
  rs:208`) checked above; `path.rs:217,223,236,242,318` (all `assert!(`;
  already doc-commented as `Path::create`'s 3-guard family, re-confirmed by
  reading), `time_optimal_trajectory_generation.rs:1596,1602`
  (`"num_waypoints > 1"`, doc-commented, re-confirmed), `tests/
  robot_trajectory.rs:514` (`"duration_from_previous[0] must be 0.0"`,
  doc-commented, re-confirmed) checked; all unflagged remainder unique
  tree-wide. Line numbers in this paragraph re-derived after merging
  main's TOTG upstream-bug commits, which shifted every line in
  `time_optimal_trajectory_generation.rs`.
- **Zero collisions in `moveit-model`.** 6 sites; `robot_model.rs:2287,
  2302` (root-link `[]`/`names` arms, doc-commented, re-confirmed),
  `:2635` (`"Box dimensions"` — same literal message duplicated at
  `bodies.rs:2210` -- `Error::construct("Box dimensions must be non-negative.")`, inside `recompute` -- and `crates/moveit-geometry/src/shapes.rs:906,930` (same literal `Error::construct("Box dimensions`), but byte-identical text across
  all three, not a discrimination-defeating collision — noted as a
  code-duplication smell, not a finding), `:2472,2512,2542` (mesh-load
  `detail` trio, three distinct messages in one match arm, `:990/:999/
  :1011`) all checked above.
- **Zero collisions in `moveit-planning`.** 2 sites (`check_start_state_
  collision.rs:161,162`) checked above (single message-construction site,
  `"engulfing_box"` is echoed scene data not branch text) — **read-only**;
  this crate is not mine to edit (p1-robotmodel's fence).
- **Not applicable: `moveit-collision`, `moveit-scene`.** Zero
  message-shaped `contains` sites in either crate (see the 78/63 split
  above) — nothing in scope, not something checked and found clean.
- **`moveit-distance-field`.** Closed under §11 (this ledger, same crate,
  prior round); not re-audited here.

### `via:` bucket (12), all examined

- **2, `moveit-distance-field/tests/oracle_parity.rs:296,304`
  (both sites read `check_scenario(`).** Already covered and closed
  under §11's own table row for these exact lines; not re-derived.
- **2, `moveit-stomp-core/src/utils.rs:641,654`
  (`via:rows_to_string`).** **N/A** — `assert_eq!`/`#[should_panic]`
  formatting tests on a `Display` helper, not error-branch checks at all.
- **8, `moveit-constraints/tests/decide.rs`
  (`via:assert_err_mentions`, lines 362,380,438,508,534,553,742,781).**
  Read every call's actual needle argument (the tool's `text` field
  truncates at 120 chars and does not resolve a helper's argument, so this
  required opening the source): `PositionConstraint::new`'s 6 needles
  (`"no frame specified for position constraint"`, `"needs at least one
  constraint region"`, `"has no bodies:: counterpart"`, `"convex mesh body
  requires at least one vertex"`, `r#"no link named "no_such_link""#`,
  `r#"no frame named "no_such_frame""#`) are pairwise-distinct text with no
  substring overlap. `OrientationConstraint::new`'s 2 needles
  (`r#"no frame named "no_such_frame""#`, `r#"no frame named """#`) are a
  different function from `PositionConstraint::new` (so the identical
  `"no_such_frame"` text at line 553/742 is not a cross-function
  collision), and the empty-string variant (`:781`) is not a substring of
  the unknown-frame variant (the literal quoted text differs). **CLEAN, all
  8.**

### The `bodies.rs:4055` (`cylinder_negative_radius_is_an_error`)/`:4125` reversal

The round's seed candidate ("both `.contains("radius")` on a rendered
error") came from scan output showing two hits for the same needle at two
call sites — exactly the shape a real collision has. It is not one here:
both tests reach the *same* function (`Cylinder::recompute`), which has
two separately-ordered guards, not one folded condition covering both
inputs. `"radius"` names only the first guard's message and is not a
substring of the second's. The orchestrator independently re-derived the
same conclusion by naming the actual field/variable identifiers
(`radius_scaled`, `half_length`) and the ordering argument (deleting the
first guard lets the second fire and reddens the assertion) — a
bite-check-shaped argument, even though no edit was made to confirm it.
p3-acm reached the same clean verdict independently on the same pair. The
reusable lesson, per the round's own instruction: two `.contains()` hits on
the same needle in scan output is the *symptom* that looks like a
collision; whether it is one turns on whether the two call sites reach one
function with ordered, distinctly-worded guards (not a collision) or two
functions/one folded condition sharing a message (would be).

### Totals

984/984 tests green across the 12 crates read this round (enumerated
above), all read-only — no edits to any crate outside this ledger's own
`moveit-distance-field`/`ros/moveit-ros`. **Zero collisions found** across
78 message-shaped `contains` sites (27 individually traced, 51 unique
tree-wide by construction) and 12 `via:` sites. No fix was owed under this
round's division of labour ("you report, it fixes") because none of the
findings required one, including the seed candidate. No commit this
section for the audit's substance (nothing to fix); this section itself is
the first commit of the round's output, since the prior round's report had
existed only under `.caucus/` (gitignored, would not have survived the
session) until this write-up.

Gated doc-only: `cargo fmt --all -- --check` (clean).

## §13. Audit of the sites this sweep itself created (round 12)

Task: the session went from `0bf4707` (session start) to `cc492d6` (main,
this round's starting point) adding net sites that mostly have no ledger
row, because in most files the owning panel's fence closed before the fix
landed — "a sweep whose output is unswept." Read-only, tree-wide except
this ledger's own two crates (`ros/moveit-ros`, `moveit-distance-field`),
which I may fix directly; everything else is report-only, routed by the
orchestrator.

### The diff, independently derived — and where it disagrees with the quoted figures

`git worktree add --detach <scratch> 0bf4707`, copied today's
`tools/ci/count-coarse-assertions.py` into it (the script itself postdates
`0bf4707` — `git log --oneline --follow` shows it was committed at
`6a14a89`, which is not an ancestor of `0bf4707`; running the *current*
instrument against the *old* tree, per this round's own instruction, is the
only way to get a comparable count), ran it in both trees, diffed.

Total (`crates/`+`ros/`, excluding `tools/`, matching what the quoted
per-file figures cover): **663 -> 687 (+24)**, 17 files changed, not 19.
This does not reproduce the round's quoted 662/694/+32/19-files — the
round's own instruction was to derive this independently and not trust
those numbers, so the disagreement is reported rather than reconciled
away. Two more discrepancies worth naming because they are not just
off-by-one: `ruckig_filter.rs` is `4 -> 7` here, not the quoted `4 -> 10`;
`tree.rs` is `28 -> 29`, not `28 -> 32`. `mesh_search_paths.rs` and
`space.rs` (quoted as `each +1`) both changed substantially by `git diff
--stat` (63 and 20 lines) but their coarse-grammar site *count* is
unchanged (5 and 3 both trees) — the new test code in those two files
doesn't use the tracked grammar at all, so the round's file list including
them may be measuring a different, wider notion of "moved" than this
instrument's site count. `state.rs`, `trajectory.rs` (ros) and
`collision_object.rs` reproduce the quoted *delta* exactly (+5/+2/+1) but
not the quoted baseline (off by one low in each case). "Three files went
down" is not reproduced either — only two, `moveit-planning/src/
pipeline.rs` (4 -> 1) and `moveit-distance-field/src/
collision_env_distance_field.rs` (20 -> 19), appear in this diff.

Per-file delta (`crates/`+`ros/`, excluding `tools/`, this instrument, this
pin):

| file | before -> after |
|---|---|
| `moveit-collision/src/matrix.rs` | 18 -> 19 |
| `moveit-collision/src/tools.rs` | 6 -> 8 |
| `moveit-collision/src/world.rs` | 24 -> 26 |
| `moveit-constraints/tests/decide.rs` | 14 -> 15 (net; one `matches!` line and one `via:` line both changed text at line 210, cancelling — see below) |
| `moveit-distance-field/src/collision_env_distance_field.rs` | 20 -> 19 |
| `moveit-model/src/robot_model.rs` | 36 -> 38 |
| `moveit-octomap/src/tree.rs` | 28 -> 29 |
| `moveit-planners-pilz/src/trajectory_blender_transition_window.rs` | 9 -> 10 |
| `moveit-planning/src/pipeline.rs` | 4 -> 1 |
| `moveit-smoothing/src/acceleration_filter.rs` | 4 -> 6 |
| `moveit-smoothing/src/ruckig_filter.rs` | 4 -> 7 |
| `ros/moveit-ros/src/constraints/position.rs` | 5 -> 7 |
| `ros/moveit-ros/src/conversion_coverage.rs` | 4 -> 6 |
| `ros/moveit-ros/src/planning.rs` | 5 -> 6 |
| `ros/moveit-ros/src/scene/collision_object.rs` | 12 -> 13 |
| `ros/moveit-ros/src/state.rs` | 7 -> 12 |
| `ros/moveit-ros/src/trajectory.rs` | 11 -> 13 |

### The two fenced crates: already closed, not re-derived

`git merge-base --is-ancestor` against every commit touching `ros/
moveit-ros` in this diff (`0377809`, `d038c9a`, `7d52e2e`, `9d829a8`,
`4c56148`, `e318294`, `ef59041`, `b0a9e1b`, `0148392`) confirms all nine
predate `2a4bd2a` (this ledger's own §10/§11 write-up commit) — this is my
own earlier-round work, already ledgered. Grepping the ledger's existing
§10 table for the specific lines confirms every one of the ros/ additions
above already has a CLEAN row there: `position.rs` (`:456,472`,
"meshes is not supported"), `conversion_coverage.rs` (`:227,232`),
`planning.rs:1033` (`multi_dof_joint_trajectory_is_rejected_not_silently_dropped`), `collision_object.rs` (10 sites incl. `:1089`'s flagged
latent-not-live risk), `state.rs` (11 hidden `assert_err_mentions` incl.
`:334,357,379,432,460,478,496` — the four `multi_dof_joint_state` sites'
own comment already states isolating-mutation bite evidence: "neutralize
one operand's clause to false... each of the four tests below fails only
when its own operand's clause is neutralized"), `ros/moveit-ros/src/trajectory.rs:383` (`add_suffix_way_point_rejects_a_nonzero_first_dt`). Per
the standing rule not to re-audit a finished round, these are cited, not
redone. `moveit-distance-field`'s one change (`collision_env_distance_
field.rs`, 20 -> 19) is the sweep working: `08976b8` replaced a bare
`result.is_err()` with a `match` on `Error::UnknownName{kind,..}`, and the
replacement's own comment states the bite-check ("swapping this guard's
kind to 'link' left a bare is_err() green"). Nothing to fix in either
fenced crate.

### `decide.rs`: a false alarm in my own diff tooling

The raw diff showed a `matches!` line both added and removed at line 210
with identical (truncated) text — an artifact of my own line-matching
heuristic pairing two occurrences of literally the same text at different
line numbers, not a real change. The real, single addition is
`crates/moveit-constraints/tests/decide.rs:534` (`new_rejects_unknown_link`)
(`via:assert_err_mentions`, `PositionConstraint::new`'s unknown-link case)
— already in this ledger's §12 table (`"no link named \"no_such_link\""`,
verified CLEAN, unique among `PositionConstraint::new`'s six needles).

### Findings — report-only, routed by the orchestrator

**1. `moveit-collision/src/matrix.rs` — `AllowedCollisionMatrix::entry`'s
`?`-chain funnel, and a new test that does not test what its own comment
claims.**

```rust
pub fn entry(&self, name1: &str, name2: &str) -> Option<&AllowedCollision> {
    self.entries.get(name1)?.get(name2)
}
```

Two `None`-producing points in one chained expression — the census §9g
shape (`self.entries.get(name1)?` = no row for `name1` at all;
`.get(name2)` = row exists but no entry for `name2`) — collapsing to one
bare `Option`, exactly like `MeshSearchPaths::resolve`. Two tests now call
`entry("a", "a").is_none()`: the pre-existing
`set_entry_for_known_pairs_name_with_every_other_existing_row_but_not_
itself` (`:821`) and the new `set_entry_for_known_excludes_the_name_even_
when_it_is_already_a_known_row` (`:835`, `035d4b1`). The new test's own
name, and the old test's own comment ("'a' itself was not a known row
before the call, so it cannot be paired with itself"), both read as if
these two tests exercise the *outer* guard (no row at all) and the
*inner* guard (self-key excluded from an existing row) respectively —
exactly the "targets a different guard" claim census §9g warns a read
cannot establish on its own.

Traced `set_entry_for_known` itself (not assumed):

```rust
pub fn set_entry_for_known(&mut self, name: &str, allowed: bool) {
    let known: Vec<String> = self.entries.keys()
        .filter(|k| k.as_str() != name).cloned().collect();
    for other in known {
        self.set_entry(name, &other, allowed);
    }
}
```

This unconditionally creates (or extends) `entries[name]` by pairing
`name` with every *other* known row — `name` itself is filtered out of
`known` before the loop, so `entries[name]` never gets a `name` key,
**but the loop's own side effect guarantees `entries.get(name)` is `Some`
by the time the assertion runs**, regardless of whether `name` had a row
before the call. Working the two tests' fixtures through this: `:821`
starts with `entries = {"b":{"c"},"c":{"b"}}`, no row for `"a"` — but
`set_entry_for_known("a", true)` itself creates `entries["a"] =
{"b":...,"c":...}` as a side effect of the very call being tested, before
`entry("a","a")` is ever evaluated. `:835` starts with an existing
`entries["a"] = {"z":...}` and the same call extends it to `{"z":...,
"b":...,"c":...}`. In **both** cases `entries.get("a")` is `Some` at
assertion time, and only the inner `.get("a")` (self-key absent from an
existing row) can return `None`. The two tests do not exercise the two
different guards their names and comments claim — they exercise the
*same* one, twice. The outer guard (`entries.get(name1)` genuinely
missing) has **zero coverage** in this file after this addition, exactly
as before it: this sweep grew the folded-into-`None` blind spot's
population by one confidently-mislabeled test rather than closing it. Not
mine to fix (`moveit-collision` is not my fence) — flagging for whoever
owns it; a fix needs either a structural change (report which side missed,
e.g. by returning a small enum instead of `Option`) or, at minimum, a
genuine bite-check on both tests before trusting either label.

**2. `moveit-planners-pilz/src/trajectory_blender_transition_window.rs` —
`validate_request`'s `InvalidMotionPlan` is shared by three guards, and
the new test's own reasoning addresses one of the other two, not both.**

```rust
if req.blend_radius <= 0.0 { return Err(Error::Code(MoveItErrorCode::InvalidMotionPlan)); }
...
if !is_robot_state_equal(...) { return Err(Error::Code(MoveItErrorCode::InvalidMotionPlan)); }
let sampling_time = determine_and_check_sampling_time(...)
    .ok_or(Error::Code(MoveItErrorCode::InvalidMotionPlan))?;
if !is_robot_state_stationary(...) || !is_robot_state_stationary(...) {
    return Err(Error::Code(MoveItErrorCode::InvalidMotionPlan));
}
```

Four guards total; the first (`blend_radius <= 0.0`) is a different code
(`InvalidGroupName`/`InvalidLinkName` guards above it are different again)
and is not implicated (`req.blend_radius = 0.05` in the new test, so it
cannot fire). The other three — boundary-mismatch (`is_robot_state_
equal`), sampling-time determination, and stationarity — all produce the
textually and structurally identical `Err(Error::Code(MoveItErrorCode::
InvalidMotionPlan))`, and the new test (`:842`,
`stationarity_failure_is_rejected` or similarly named, targeting
stationarity) asserts only `matches!(validate_request(&req),
Err(Error::Code(MoveItErrorCode::InvalidMotionPlan)))` — the variant and
code, nothing finer, because there is nothing finer in the value to check;
this is not a text-discrimination gap the way the `contains()` sites in
§12 are, it is the funnel itself. The test's own comment reasons out
*one* of the other two candidates: "a one-sided perturbation would trip
the boundary-mismatch guard instead of the stationarity guard this test
targets. Matching velocities keep [the boundary check] satisfied while
still failing is_robot_state_stationary on each end individually" — a
read-only argument (no stated mutation), and it says nothing about the
third candidate, `determine_and_check_sampling_time`'s guard, at all. Not
mine to fix (`moveit-planners-pilz` is not my fence) — flagging for
whoever owns it: either bite-check that this fixture's sampling-time
determination genuinely succeeds before stationarity is reached, or widen
the assertion to check a message/field that actually distinguishes the
three.

### Everything else read this round: clean, and why a read was enough here

Unlike the two findings above, the remaining new sites are either (a) a
single guard with no sibling to confuse it with, or (b) a folded-operand
guard (`doc/folded-operand-guards.md`'s accepted shape — one guard, several
invalid-operand combinations, each isolated by its own test), or (c)
already carries in-line bite-check evidence from its own author, not just
a read:

- `moveit-collision/src/tools.rs:283,292` — `intersect_cost_sources`
  delegates to `aabb_intersection`'s single 3-operand (`x`/`y`/`z`) folded
  guard; the two new tests isolate the `y` and `z` operands the same way a
  pre-existing sibling (predates `0bf4707`) isolates `x`. One guard, one
  message, three operands — the accepted pattern, not a collision.
- `moveit-collision/src/world.rs:1142,1276` — `move_shapes_in_object` and
  `remove_shape_from_object` each have two *sequential* (not chained-in-
  one-expression) `None`-producing points; both new tests' fixtures make
  the first guard's precondition false by construction (`move_shapes_in_
  object_unknown_object_is_none` uses an empty `World::new()`, so the
  object-missing `?` fires before the count-check is reached at all;
  `remove_shape_from_object_unknown_shape_is_none` adds the object first,
  so the object-missing `?` cannot fire, leaving only the shape-lookup
  `?`). Confirmed by reading guard order plus each test's own setup, not
  assumed — this is the `Cylinder::recompute` shape (ordered, distinctly-
  reached guards), not the `entry()` shape (one chained expression, both
  `?`s live for the same call).
- `moveit-model/src/robot_model.rs:2055` (`mimic_mutual_cycle_clears_every_mimic_in_the_model`) — a fixture-quality fix
  (`9661d4c`, giving `j4` a real mimic so a cycle-clear check isn't
  vacuous), a plain field read (`.mimic().is_none()`), not a multi-guard
  function.
- `moveit-model/src/robot_model.rs:2796` (`end_effector_wires_name_and_falls_back_to_fewest_joints_parent`) — `get_end_effector`'s own body
  is `self.groups.get(name).filter(is_end_effector).ok_or_else(...)`: a
  *single* `Error::unknown_name` construction site reachable through two
  distinct input categories (name not a group at all vs. name a group but
  not an end effector). Confirmed by reading — there is no second
  message to discriminate against because there is no second call site;
  this is the folded-operand pattern applied to an `Option` chain rather
  than a boolean `||`, and the new test's own comment already states this
  correctly.
- `moveit-octomap/src/tree.rs:1944` (`leaves_in_bbx_only_yields_leaves_overlapping_the_box`) — already bite-check-verified by its
  own author, not just read: "Before this test existed the min guard had
  no coverage at all -- neutralizing it left all 66 tests green. Now each
  guard isolates to its own test: neutralizing min fails only this one,
  neutralizing max only the sibling." Present-tense, mutation-stated
  evidence, matching the standard this session's own census §9g section
  demands.
- `moveit-smoothing/src/acceleration_filter.rs:559,566` (`do_smoothing_before_reset_is_an_error`),
  `moveit-smoothing/src/ruckig_filter.rs:553,560,567` (`do_smoothing_respects_velocity_and_acceleration_bounds_throughout`) — both `reset`
  functions have exactly one folded guard each (2-operand and 3-operand
  respectively); the new tests isolate the previously-unisolated operands
  the same way `do_smoothing`'s sibling guards were already isolated
  (`ruckig_filter.rs`, covered in this ledger's §12). One guard per
  function, no sibling `Error::Other` site in either to confuse it with.

### Totals

984 tests confirmed green in §12 covered every crate this section reused
verdicts from; no new test run was needed to check counts (nothing here
was edited). Independently-derived delta: 663 -> 687 (+24) across 17
files, not the quoted 662 -> 694 (+32) across 19 — reported as measured,
not reconciled to the quote. **2 findings, both report-only** (`moveit-
collision/src/matrix.rs:835`, `moveit-planners-pilz/src/trajectory_
blender_transition_window.rs:842`) for the orchestrator to route. **0
fixes owed in either of this ledger's two fenced crates** — both
already closed under §10-§12, confirmed by ancestry check and ledger
grep, not re-audited. No commit for source in this round (nothing in
`ros/moveit-ros`/`moveit-distance-field` needed a change); this section is
the round's persisted output.

Gated doc-only: `cargo fmt --all -- --check` (clean).

## §14. The two sites this round created (§236's expiry tripwires)

Fence: `ros/moveit-ros` only. §236 decides *not* to port
`setMotionPlanRequest`'s request normalization
(`moveit_core/planning_interface/src/planning_interface.cpp:92-103`), and
§236.4 makes that decision expire by test rather than by review: two
assertions that fail the moment either normalized field becomes observable
on this port's core `PlanningRequest`. Both are coarse `assert!` sites, so
both are in this sweep's grammar and owe a row here.

| file:line | in-family verdict | collision verdict |
|---|---|---|
| `planning.rs:905,943` | in-family (both) | CLEAN (2 sites) — same shape, disjoint subjects: `:905` (`allowed_planning_time_boundaries_are_not_observable_on_the_core_request`) tables 5 boundaries (`-1.0`, `0.0`, `f64::EPSILON`, `5.0`, `f64::NAN`), `:943` (`num_planning_attempts_boundaries_are_not_observable_on_the_core_request`) tables 4 (`-1`, `0`, `1`, `2`); each maps its own single `MotionPlanRequest` field through `TryFrom<PlanningRequestMsg>` and compares `format!("{req:?}")` against the table's first row. The needle cannot collide because the panic text interpolates the field name *and* `observable`, the list of boundary labels that differed — a failure names which of the 5-or-4 rows moved, so the two tests cannot be confused with each other or with a neighbouring row of their own table |

Clause-by-clause against census §9, since "asserts a set is empty" is
exactly the vacuous-accumulator shape §10 ruled `not-this-family` twice:

1. **Mechanism.** `observable.is_empty()` is a found-nothing tag, but it is
   not vacuous — `rows` is a 5-element (resp. 4-element) array literal in
   the test itself, and `labels_differing_from_the_first`
   (`planning.rs:526-538`) iterates `rows[1..]`, so the comparison runs 4
   (resp. 3) times on every run. The failure message interpolates the
   offending labels rather than reporting a bare count.
2. **Decision.** The decision under test is §236's, and it lives in the
   subject: `PlanningRequest` (`crates/moveit-planning/src/request.rs:201-243`,
   opens with field `pub group_name: String,`) has no `allowed_planning_time`
   and no `num_planning_attempts` field, and
   `request.rs:89-105` states in prose that this is a decision, not an
   oversight. A mutation that reverses the decision is precisely the
   mutation §236.4 names.
3. **Subject.** Bite-checked, not argued: adding both fields to core
   `PlanningRequest` and mapping them in `TryFrom<PlanningRequestMsg>` made
   both tests fail, each naming every boundary label that then differed —
   e.g. `MotionPlanRequest.allowed_planning_time reached PlanningRequest at
   ["0.0, the msg default, clamped to 1.0 without a log", "f64::EPSILON,
   positive so upstream keeps it", "5.0, a normal budget", "f64::NAN, which
   fails \`<= 0.0\` so upstream keeps it"], differing from the row for
   "-1.0, which upstream logs about and clamps to 1.0"`. Deleting the
   `PlanningRequest::try_from` call fails compilation, so clause 3's weaker
   form holds too. The mutation was reverted; `MUTATION` marker count is 0
   in both files.

### Totals

**2 sites added, 2 in-family, 0 not-this-family, 0 collisions, 0 fixes
owed.** §10's `planning.rs` row is unchanged in substance and re-anchored
only (see §10's second re-anchoring note). Running total for
`ros/moveit-ros` under the wide grammar: 63 sites in §10 + these 2 = 65.
Gated with the round's own list, including
`sg docker -c './tools/ci/verify-ros-interop.sh'` (the only gate that
compiles `ros/moveit-ros`) and `./tools/ci/verify-orphan-enumeration.sh`.
