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
