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

- `matrix.rs:692` — `set_entry_for_known_excludes_the_name_even_when_it_is_already_a_known_row`
  (commit `035d4b1`)
- `world.rs:995` — `move_shapes_in_object_unknown_object_is_none`
  (same merge window)

`moveit-geometry` reproduces the census's 39 (5/34) exactly. Cross-validated
against `rg -U -c` on both crates: `matches!` anchor agreed on every file;
the `bare` anchor's two known false-positive/false-negative sites
(`bodies.rs:4042`'s comment-swallow, `matrix.rs`/`world.rs` — none new this
scope) were already resolved in the census and reproduce identically here.
**Ledger denominator for this scope: 86, not the quoted 84.**

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

## 2. `crates/moveit-collision` — 47 sites

### `src/tools.rs` (1)
| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| tools.rs:219 | bare | sensor_positioning_of_empty_set_is_none | single-branch | read full 4-line `sensor_positioning` body: exactly one `?` on `iter().nth(index)`, no other `None`-producing path |

### `src/env.rs` (1)
| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| env.rs:674 | bare | merge_of_two_none_distances_is_none | single-branch | traced match-arm reachability in `CollisionResult::merge`'s distance match (common.rs:585-589): arm 2's `b` is provably `Some` whenever reached (arm 1 already absorbs all `other==None` cases), so only arm 1 (both `None`) can yield `None` |

### `src/matrix.rs` (17)
| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| matrix.rs:515 | bare | neither_explicit_nor_default_is_not_found | single-branch | read: only `self.default_for_pair(..)` returns `None` from `allowed_collision`; only the `(None,None)` arm of `default_for_pair`'s 4-arm match yields `None` |
| matrix.rs:516 | bare | neither_explicit_nor_default_is_not_found | discriminating | bite run now on `entry()` guard A (row missing): neutralized A → test FAILED; neutralized guard B only → test PASSED |
| matrix.rs:517 | bare | neither_explicit_nor_default_is_not_found | single-branch | read: `default_entry` is one map lookup, one `None`-producing site |
| matrix.rs:548 | bare | never_and_always_carry_no_predicate | single-branch | read: `Never`/`Always` share one match arm in `predicate()`; no independent path to diverge |
| matrix.rs:549 | bare | never_and_always_carry_no_predicate | single-branch | same shared arm as 548 |
| matrix.rs:559 | bare | overwriting_a_conditional_entry_with_bool_drops_the_predicate | single-branch | same shared arm as 548/549 |
| matrix.rs:592 | bare | remove_entry_then_lookup_falls_back_to_default | discriminating | bite run now on `entry()` guard B (row present, key missing): guard-B-neutralize → FAILED; guard-A-neutralize → PASSED |
| matrix.rs:604 | bare | remove_entry_is_symmetric | discriminating | bite run now, same guard-B pattern as 592 |
| matrix.rs:605 | bare | remove_entry_is_symmetric | discriminating | bite run now, mirror direction, same test |
| matrix.rs:616 | bare | remove_entries_for_name_clears_its_row_and_every_cell_naming_it | discriminating | bite run now, same guard-B pattern |
| matrix.rs:617 | bare | remove_entries_for_name_clears_its_row_and_every_cell_naming_it | discriminating | bite run now, same guard-B pattern |
| matrix.rs:648 | bare | set_entry_between_pairs_every_combination | discriminating | bite run now, same guard-B pattern |
| matrix.rs:649 | bare | set_entry_between_pairs_every_combination | discriminating | bite run now, same guard-B pattern |
| matrix.rs:666 | bare | set_all_entries_overwrites_every_existing_pair_but_adds_none | discriminating | bite run now, same guard-B pattern |
| matrix.rs:678 | bare | set_entry_for_known_pairs_name_with_every_other_existing_row_but_not_itself | discriminating | `entry()` guard B verified by bite run now; the `set_entry_for_known` `!= name` filter itself is vacuous here — confirmed live (removing the filter left this test green) and by commit `035d4b1`'s message, which is why sibling 692 was added |
| matrix.rs:692 | bare | set_entry_for_known_excludes_the_name_even_when_it_is_already_a_known_row | discriminating | commit `035d4b1`, isolating-mutation pair with 678: re-verified live, filter-removed bite makes this test FAIL while 678 stays GREEN |
| matrix.rs:737 | bare | clear_removes_entries_and_defaults | single-branch | same as 517 (`default_entry`) |

Note: the `entry()` two-guard family (516 vs 592-678) is the brief's own
cited model ("p3-acm's `matrix.rs` `entry()` work is the model"); every
guard-B site above shares one bite pair, not 10 independent ones.

### `src/octomap_filter.rs` (5)
| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| octomap_filter.rs:300 | bare | sample_cloud_empty_returns_none | single-branch | bite run now: no-op'd the sole guard → test FAILED; no sibling to swap in |
| octomap_filter.rs:355 | bare | metaball_surface_properties_without_depth_returns_unit_normal_only | single-branch | bite run now: `None`→`Some(NaN)` on the depth else-branch → test FAILED; the if-branch can only ever produce `Some`, never `None` |
| octomap_filter.rs:369 | bare | metaball_surface_properties_empty_cloud_is_none_in_both_modes | discriminating (isolating mutation, direction A) | bite run now: neutralized guard A (`sample_cloud` path) → test FAILED naming A; sibling assertion (guard B) stayed GREEN when isolated |
| octomap_filter.rs:370 | bare | metaball_surface_properties_empty_cloud_is_none_in_both_modes | discriminating (isolating mutation, direction B) | mirror bite run now: neutralized guard B (`find_surface` path) → test FAILED naming B; guard-A assertion stayed GREEN when isolated |
| octomap_filter.rs:381 | bare | refine_contact_normals_no_contacts_requested_is_a_noop | not-this-family | `rg -n 'contacts\s*='` over the file: zero assignment sites inside `refine_contact_normals` — `result.contacts` starts `None` and cannot become `Some` by any path through this function; the assertion is tautological |

### `src/parry.rs` (7)
| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| parry.rs:2442 | bare | convert_shape_degenerate_plane_is_excluded | discriminating | isolating mutation run now: neutralized Plane-magnitude guard → Plane test FAILED, both OcTree tests (2465, 2475) stayed GREEN |
| parry.rs:2465 | bare | convert_shape_octree_with_no_tree_attached_is_excluded | discriminating | isolating mutation run now: neutralized no-tree guard → this test FAILED, Plane + no-occupied-leaves tests stayed GREEN |
| parry.rs:2475 | bare | convert_shape_octree_with_a_tree_but_no_occupied_leaves_is_also_excluded | discriminating | isolating mutation run now: neutralized empty-tree path → this test FAILED, Plane + no-tree tests stayed GREEN |
| parry.rs:2616 | bare | octree_cache_prunes_an_entry_once_nothing_holds_its_tree | single-branch | read: `build()` is stubbed and no cache hit is possible (fresh key each call); bite run now forcing a value regardless of `build()` → test FAILED |
| parry.rs:2623 | bare | octree_cache_prunes_an_entry_once_nothing_holds_its_tree | single-branch | same bite/run as 2616 (one test function, one mutation covers both assertions) |
| parry.rs:3472 | bare | check_robot_collision_continuous_returns_an_error_rather_than_approximating | single-branch | read: function body is one unconditional `Err`, no guard at all; bite run now replacing it with `Ok(..)` → test FAILED |
| parry.rs:4065 | bare | check_self_collision_cost_sources_is_none_when_not_requested | single-branch | read: one boolean gate (`request.cost`) controls `Some`/`None`, no sibling; bite run now forcing `Some` unconditionally → test FAILED |

### `src/world.rs` (15)
| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| world.rs:891 | bare | add_to_object_mismatched_lengths_is_a_no_op | discriminating | isolating bite run now on the OR-guard's length clause: neutralize → 891 FAILS, 899 stays GREEN |
| world.rs:899 | bare | add_to_object_empty_shapes_is_a_no_op | discriminating | isolating bite run now on the OR-guard's empty clause: neutralize → 899 FAILS, 891 stays GREEN |
| world.rs:952 | bare | move_shape_in_object_unknown_shape_is_none | discriminating | isolating bite run now: neutralize position-lookup → 952 FAILS/962 GREEN |
| world.rs:962 | bare | move_shape_in_object_unknown_object_is_none | discriminating | isolating bite run now: neutralize `object_mut` guard → 962 FAILS/952 GREEN |
| world.rs:981 | bare | move_shapes_in_object_count_mismatch_is_a_no_op | discriminating | isolating bite run now: drop count check → 981 FAILS/995 GREEN, confirming the test's own doc-comment claim |
| world.rs:995 | bare | move_shapes_in_object_unknown_object_is_none | discriminating | isolating bite run now: bypass `object_mut` guard → 995 FAILS/981 GREEN — doc-comment's isolation claim verified true, not just asserted |
| world.rs:1003 | matches | move_object_not_found | discriminating | bite run now: fallback-to-identity → FAILS; variant-swap to `NoChange` → still FAILS |
| world.rs:1013 | matches | move_object_identity_transform_is_no_change | discriminating | stayed GREEN through both 1003 mutations above, confirming isolation |
| world.rs:1108 | bare | remove_shape_from_object_unknown_is_none | discriminating | isolating bite run now: neutralize `object_mut` guard → 1108 FAILS/1129 GREEN |
| world.rs:1129 | bare | remove_shape_from_object_unknown_shape_is_none | discriminating | isolating bite run now: neutralize position-lookup (Arc-identity match) → 1129 FAILS/1108 GREEN; the Arc-identity doc-comment is accurate context but the bite is the evidence |
| world.rs:1141 | bare | remove_object_missing_is_none | single-branch | read: one guard in `remove_object`'s body; bite run now (fallback dummy object) → FAILED |
| world.rs:1196 | bare | global_shape_transform_unknown_object_is_none | discriminating | isolating bite run now: unknown-object fallback → 1196 FAILS/1210 GREEN |
| world.rs:1210 | bare | global_shape_transform_out_of_range_index_is_none | discriminating | isolating bite run now: out-of-range fallback → 1210 FAILS/1196 GREEN |
| world.rs:1216 | bare | global_shape_transforms_unknown_object_is_none | single-branch | read: one guard, only cause in `global_shape_transforms`; bite run now (empty-slice fallback) → FAILED |
| world.rs:1252 | bare | transform_lookup_unknown_name_errors | single-branch | read: no sibling guard, one fallthrough per function (`knows_transform`/`try_get_transform`/`get_transform`); bite run now (flipped final `false`→`true` in `knows_transform`) → FAILED |

### `tests/world_parity.rs` (1)
| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| world_parity.rs:226 | bare | world_matches_oracle | not-this-family | module doc (lines 4-33) + committed oracle fixture (`world_request.json`/`world_response.json`): `query.transform is None` per the oracle's own JSON dump for a specific query name — a per-query oracle-value comparison, not branch discrimination inside one function; there is no sibling branch to isolate |

## 3. `crates/moveit-geometry` — 39 sites

### `src/bodies.rs` (13)

**Correction, this pass: 3 of these 13 rows were wrong** (same defect
family as `shapes.rs` above — see §1a). `bodies::Cuboid::recompute` is a
*different* function from `shapes::Cuboid::new` (a separate struct
defined at bodies.rs:2138, own `recompute` at bodies.rs:2197) but shares
the identical combined `half_length<0 || half_width<0 || half_height<0`
guard shape, and the same "one token, one cause" misclassification.

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| bodies.rs:4037 | bare | sphere_negative_radius_is_an_error | single-branch | bite run now: no-op'd `Sphere::new`'s one guard → test FAILED; exactly one `Error::construct` call in the function |
| bodies.rs:4087 | bare | cuboid_negative_dimension_is_an_error_per_axis | discriminating (multi-branch, corrected) | bite run now: neutralize `half_length` clause in `bodies::Cuboid::recompute` → 4087 (length) FAILS, 4089 (height) stays GREEN |
| bodies.rs:4088 | bare | cuboid_negative_dimension_is_an_error_per_axis | discriminating (multi-branch, corrected) | same combined 3-way guard as 4087/4089; width is symmetric with length and height (not independently bit this pass, structure identical) |
| bodies.rs:4089 | bare | cuboid_negative_dimension_is_an_error_per_axis | discriminating (multi-branch, corrected) | mirror half of the 4087 bite pair — stayed GREEN when `half_length` was neutralized |
| bodies.rs:4102 | bare | sphere_padding_inversion_is_rejected_and_state_preserved | single-branch | doc-comment re-read and confirmed: `Sphere::set_padding` has exactly one `Error::construct` call, one operand |
| bodies.rs:4140 | bare | cuboid_padding_inversion_is_rejected_and_state_preserved | single-branch (as tested) | this test has exactly one assertion (no sibling axis case in this specific test), so there is nothing in *this* test to isolate against even though the underlying `recompute` guard is the same combined shape corrected at 4087-4089; not re-classified — a coverage-gap question (whether a second case should exist), not a misclassification of what is actually asserted here |
| bodies.rs:4328 | matches | from_shape_builds_matching_body_variant | discriminating | bite run now: rewrote the `Shape::Sphere` arm of `Body::from_shape` to build a `Cylinder` instead → the Sphere assert FAILED |
| bodies.rs:4332 | matches | from_shape_builds_matching_body_variant | discriminating | doc-comment re-read: each `Body::from_shape` arm builds a distinct concrete variant, mirror of the 4328 bite |
| bodies.rs:4340 | matches | from_shape_builds_matching_body_variant | discriminating | same reasoning, distinct match arm (`Shape::Cuboid`) |
| bodies.rs:4347 | matches | from_shape_builds_matching_body_variant | discriminating | same reasoning, distinct match arm (`Shape::Mesh`) |
| bodies.rs:4384 | bare | from_shape_returns_none_for_cone_plane_octree | discriminating (isolating mutation) | bite run now: split the Cone pattern out of the combined `None` arm in `Body::from_shape` → the Cone assert FAILED |
| bodies.rs:4392 | bare | from_shape_returns_none_for_cone_plane_octree | discriminating (isolating mutation) | bite run now: split the Plane pattern out → the Plane assert FAILED while the Cone assert stayed GREEN |
| bodies.rs:4402 | bare | from_shape_returns_none_for_cone_plane_octree | discriminating (isolating mutation) | doc-comment re-read: symmetric with the Cone/Plane bites just run, each pattern an independent match arm; the doc-comment's own D6 call-site check confirms no caller needs `from_shape` to discriminate Cone/Plane/OcTree |

Note: the doc-comment at bodies.rs:4045-4048 (which contrasts `sphere_negative_radius_is_an_error` with a *sibling* `cylinder_negative_length_is_an_error` test elsewhere in the file) is evidence for that cylinder test, not for line 4037 — flagged explicitly to avoid misattribution, and not used as 4037's evidence above.

### `src/octree_collision.rs` (2)
| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| octree_collision.rs:120 | bare | empty_tree_has_no_occupied_leaves | single-branch | bite run now: no-op'd the sole `is_empty()` guard → both this test and its sibling failed identically (only one `None`-producing site in the whole function) |
| octree_collision.rs:127 | bare | all_free_tree_has_no_occupied_leaves | single-branch | same bite as 120 — "empty tree" and "all free" both collapse to the same `is_empty()` check before `Some`/`Compound::new`, not two distinct guards, so the §3a isolating-mutation case does not apply here |

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

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| shapes.rs:1618 | bare | sphere_negative_radius_is_an_error | single-branch | `Sphere::new` has one operand, one guard — nothing to isolate |
| shapes.rs:1623 | bare | cylinder_negative_dimension_is_an_error | discriminating (multi-branch, corrected) | bite run now, both directions: neutralize radius clause → 1623 FAILS, 1624 stays GREEN; neutralize length clause → 1624 FAILS, 1623 stays GREEN (comment out sibling per §3a protocol, since `assert!` short-circuits within the one test fn) |
| shapes.rs:1624 | bare | cylinder_negative_dimension_is_an_error | discriminating (multi-branch, corrected) | same bite pair as 1623 |
| shapes.rs:1630 | bare | cone_negative_dimension_is_an_error | discriminating (multi-branch, corrected) | bite run now: neutralize radius clause → 1630 FAILS, 1631 stays GREEN (identical combined-guard shape to Cylinder::new, confirmed live rather than assumed by analogy) |
| shapes.rs:1631 | bare | cone_negative_dimension_is_an_error | discriminating (multi-branch, corrected) | same bite as 1630 |
| shapes.rs:1636 | bare | cuboid_negative_dimension_is_an_error | discriminating (multi-branch, corrected) | bite run now: neutralize x clause → 1636 FAILS, 1638 (z) stays GREEN |
| shapes.rs:1637 | bare | cuboid_negative_dimension_is_an_error | discriminating (multi-branch, corrected) | same combined 3-way guard as 1636/1638; y is symmetric with x and z (not independently bit this pass, structure identical) |
| shapes.rs:1638 | bare | cuboid_negative_dimension_is_an_error | discriminating (multi-branch, corrected) | mirror half of the 1636 bite pair — stayed GREEN when x was neutralized |
| shapes.rs:1656 | bare | sphere_padding_past_negative_is_an_error | single-branch | read: `Sphere::scale_and_padd` has exactly one `Err` site, one operand |
| shapes.rs:1676 | bare | cylinder_padding_past_negative_is_an_error_per_axis | discriminating (multi-branch, corrected) | bite run now, both directions: neutralize radius clause → 1676 (`radius_case`) FAILS, 1685 (`length_case`) stays GREEN; neutralize length clause → mirrors |
| shapes.rs:1685 | bare | cylinder_padding_past_negative_is_an_error_per_axis | discriminating (multi-branch, corrected) | same bite pair as 1676 |
| shapes.rs:1797 | bare | shapes_with_no_upstream_body_have_no_volume_or_dimensions | discriminating (isolating mutation, multi-branch) | bite run now: split `Shape::Cone(_)` out of `compute_volume`'s combined `None` arm (`Cone\|Plane\|Mesh\|OcTree`) → FAILED exactly at the Cone iteration; matches commit `871bb9e`'s recorded correction |
| shapes.rs:1798 | bare | shapes_with_no_upstream_body_have_no_volume_or_dimensions | discriminating (isolating mutation, multi-branch) | doc-comment re-read: records the same isolating mutation for `get_dimensions`, identical combined-arm structure |
| shapes.rs:1862 | matches | mesh_rejects_out_of_range_triangle_index | single-branch | read: `Mesh::new` has exactly one `Error::construct` call, inside the vertex-index loop |
| shapes.rs:1898 | bare | mesh_padding_without_vertex_normals_is_an_error | single-branch | read: `Mesh::scale_and_padd_axes` has one `Err` site (`vertex_normals.as_ref().ok_or_else`); the empty-vertices branch returns `Ok(())`, not an error |
| shapes.rs:1899 | bare | mesh_padding_without_vertex_normals_is_an_error | single-branch | same guard as 1898, `scale_axes`/`padd_axes` both funnel through it |
| shapes.rs:1961 | bare | compute_vertex_normals_calls_triangle_normals_when_needed | not-this-family | doc-comment re-read and confirmed: `mesh.triangle_normals.is_none()` reads a struct field literal-initialized to `None` in `Mesh::new`, not a computed branch |

The brief's own model example (`a74a310`, `Cylinder::recompute`'s two
sequential radius/length guards with distinct messages) is in `bodies.rs`,
not this file — confirmed via `git show --stat a74a310`; this crate's own
doc-comment (shapes.rs:1593-1615, corrected this round, see §1a) draws
that exact contrast while now also correcting its own verdict for the
combined-guard side of it.

### `src/stl.rs` (1)
| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| stl.rs:456 | bare | empty_input_is_rejected | single-branch | bite run now: neutralized `parse_ascii_triangles`'s `flat_vertices.is_empty()` guard → test flipped from pass to FAIL; for the `&[]` fixture, `parse_binary_triangles` bails immediately and `Mesh::new` is never reached, so exactly one guard fires for this input (other inputs can reach `mesh_from_bytes`'s other `Err` sites, but not this fixture) |

### `src/transforms.rs` (6)
| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| transforms.rs:233 | bare | new_rejects_empty_target_frame | single-branch | bite run now: no-op'd `new`'s one `is_empty()` guard → test FAILED; only one `Err` site in the function |
| transforms.rs:234 | bare | new_rejects_empty_target_frame | single-branch (redundant with 233) | read: `trim()` runs before the single `is_empty()` check, so `""` and `"   "` hit the identical branch — no second guard exists to isolate 234 from 233 |
| transforms.rs:253 | bare | set_transform_rejects_empty_name | single-branch | bite run now: no-op'd `set_transform`'s one `is_empty()` guard → test FAILED |
| transforms.rs:275 | bare | unknown_frame_is_an_error_not_identity | single-branch | bite run now: replaced `transform`'s one `ok_or_else` with a fallback `Ok` → test FAILED (input `"nope"`, map-miss path) |
| transforms.rs:276 | bare | unknown_frame_is_an_error_not_identity | single-branch | bite run now, isolated: patched the map-miss fallback with the 275 assertion removed → failure confirms this assertion's only reachable source is the map-miss branch (the empty-string early return is unreachable for this non-empty literal) |
| transforms.rs:277 | bare | unknown_frame_is_an_error_not_identity | single-branch | same `ok_or_else` bite as 275 — both 275 and 277 funnel through `transform`'s one `Err` site, differing only in which upstream `try_transform` path produced the `None` |

## 4. Summary

86 sites, 0 UNCOVERED-and-blind (no site lacked a test entirely), but
**12 sites had a wrong verdict** — `single-branch` misapplied to combined
`||` guards hiding 2-3 independently discriminating causes, in
`shapes.rs` (`cylinder_negative_dimension_is_an_error` ×2,
`cone_negative_dimension_is_an_error` ×2,
`cuboid_negative_dimension_is_an_error` ×3,
`cylinder_padding_past_negative_is_an_error_per_axis` ×2) and `bodies.rs`
(`cuboid_negative_dimension_is_an_error_per_axis` ×3). All 12 reclassified
to `discriminating (multi-branch, corrected)` on live bite evidence (§1a).
No test itself was broken or blind — every one of these 12 already passes
and already discriminates its own operand; only the *comment* asserting
otherwise was wrong, and it is what this pass fixes. No
`fixture-collapse-fixed` verdicts — none of the 86 sites needed one.

## 5. Gate

One commit this round: correcting the 3 source doc-comments (§1a) that
had recorded the wrong verdict. Comment-only — no assertion, guard, or
test body changed; behavior is identical before and after.

```
cargo fmt --all
cargo clippy -p moveit-geometry --all-targets -- -D warnings   # clean
cargo nextest run -p moveit-geometry                            # 141/141 pass
```

`moveit-collision` had no source changes this round (no misclassification
found in that crate's 47 sites), so no gate is owed there beyond the
evidence-gathering passes' own reverts, each confirmed via `git diff`/
`git status --short` empty on exit.

**UNFIXED:** none. **Fixed:** 12 sites' verdicts, corrected via 3
doc-comment edits (one commit, comment-only, `moveit-geometry`).
**Tested:** all 86 sites, one row each, above; `cargo nextest run -p
moveit-geometry` 141/141 after the fix.
