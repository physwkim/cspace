# LGPL provenance audit — round 22, item 1 (PORTING-PLAN.md §151/§152, D11)

Measurement only. No code or header text in any of the three files below was
changed to produce this report — the rewrite happens in follow-up commits
*after* this report is committed, using this table as the scope evidence.
Same method as `crates/moveit-planners-pilz/doc/lgpl-provenance-audit.md`
(round 21).

## Why round 21's sweep missed this

Round 21's population was "files citing `third_party/orocos_kinematics_dynamics/`
(`orocos_kdl`) as their `Ported from` target" — every file's header names an
`orocos_kdl` path directly, so grepping the workspace for that path finds
them. This round's population is different: `chainiksolver_vel_mimic_svd.{hpp,cpp}`
lives *inside* moveit2's own tree, at
`moveit_kinematics/kdl_kinematics_plugin/{src,include}/...`, cited the same
way this crate's legitimately-BSD citations are (`kdl_kinematics_plugin.cpp`,
`kinematics_base.hpp`). A citation-path grep for `orocos_kdl`/`third_party`
does not find it — nothing about the *path* signals LGPL. **The structural
cause: a moveit2 file's citation path does not tell you its license: moveit2
vendors at least one LGPL-licensed file (KDL's mimic-aware IK velocity
solver) inside its own BSD-licensed tree, under its own directory structure,
with the vendored file's original LGPL header left intact inside it.**
Confirming a citation is safe requires opening the cited upstream file
itself and reading its own header — not just checking which top-level
moveit2/orocos_kdl directory it lives under.

## Method

Same four buckets as round 21's audit, restated for this crate's citations:

1. **transcription** — sentence structure/variable names/branch order follow
   the LGPL original. Cited by the original's exact line range.
2. **independently derivable** — same result, derived directly from
   elementary math/standard algorithms. The derivation is written out as an
   actual paragraph, not asserted.
3. **interface fact only** — constant values, signatures, unit conventions,
   argument order, member layout, or (specific to this crate's prose-heavy
   `lib.rs`) *naming which Rust item corresponds to which upstream symbol* —
   not authorial expression.
4. **derived from moveit2 (BSD), not the LGPL file** — sourced from
   moveit2's own BSD `kdl_kinematics_plugin.{hpp,cpp}` /
   `kinematics_base.{hpp,cpp}` / `joint_mimic.hpp`, not from
   `chainiksolver_vel_mimic_svd.{hpp,cpp}`. No LGPL provenance question
   applies to this bucket regardless of how tight the correspondence is.

Every line of every file is assigned to exactly one of: file-header **meta**,
**original** (prose/scaffolding with no upstream counterpart — blank lines,
`use` imports, struct/impl scaffolding, this port's own tests and deviation
writeups), or one of the four numbered buckets. Sums below were checked to
equal each file's exact `wc -l` line count.

## `velocity.rs` (181 lines)

Header (pre-fix) cites `chainiksolver_vel_mimic_svd.cpp` only, and carries
`Copyright (c) 2013, Sachin Chitta, Willow Garage` — that copyright line is
the LGPL file's own (`chainiksolver_vel_mimic_svd.{hpp,cpp}`'s header:
`Copyright (C) 2007 Ruben Smits`, LGPL-2.1-or-later; `Copyright (C) 2013
Sachin Chitta, Willow Garage` for the mimic-joint modification, inside that
same LGPL file). Nothing in this file is BSD-sourced, so unlike the other
two files below there is no bucket-4 content and no BSD copyright to keep.

| Region | Lines | Span | Classification | Basis |
|---|---|---|---|---|
| File header | 1-6 | 6 | meta | Contains the defect: LGPL copyright (line 1), LGPL-only `Ported from` |
| Imports + blanks | 7-11 | 5 | original | — |
| `fold_jacobian` doc + fn | 12-30 | 19 | **1** | Doc explicitly justifies the accumulation as "exactly as upstream's `result = vel1 + multiplier * vel2` accumulation does"; the fn body's loop (iterate `i`, read `map_index[i]`/`multiplier[i]`, accumulate the scaled full-space entry into the reduced column) is the same accumulate-into-column structure as `jacToJacReduced`, `chainiksolver_vel_mimic_svd.cpp:73-84` |
| blank | 31 | 1 | original | — |
| `expand_to_full` doc + fn | 32-41 | 10 | **1** | Doc explicitly says "matching upstream's own expansion at the end of `ChainIkSolverVelMimicSVD::CartToJnt`"; the fn body's per-index `reduced[map_index[i]] * multiplier[i]` is the same formula as `CartToJnt`'s closing loop, `chainiksolver_vel_mimic_svd.cpp:112-113` |
| blank | 42 | 1 | original | — |
| `solve_velocity` doc: identity + convention | 43-54 | 12 | **3** | Names which upstream method this corresponds to (`ChainIkSolverVelMimicSVD::CartToJnt`, a pointer for cross-reference, not expression) and states the 6-row linear/angular twist row convention — an interface fact every caller must already agree on to pass valid data, not this file's invention |
| blank + `# Deviation` doc section | 55-66 | 12 | original | This port's own analysis of why it hand-rolls the SVD reconstruction instead of calling `nalgebra::SVD::solve` — explains a difference from upstream, has no upstream counterpart to derive from |
| `solve_velocity` signature + reduce/dims | 67-78 | 12 | original | Parameter list, `fold_jacobian` call, reading matrix dimensions — no expression choice available in extracting `.nrows()`/`.ncols()` |
| Weighting (Jacobian + twist) | 79-85 | 7 | **2** | See derivation below |
| blank | 86 | 1 | original | — |
| SVD compute + field extraction | 87-96 | 10 | original | `nalgebra`-specific glue (`.expect` on `Option` fields `SVD::new(_, true, true)` never leaves empty) — this port's own library's API shape, no upstream counterpart |
| Pseudo-inverse reconstruction | 97-103 | 7 | **2** | See derivation below |
| fn close + blank | 104-105 | 2 | original | — |
| tests module | 106-181 | 76 | original | This port's own tests |

**Bucket-2 derivations:**

- **Weighting (lines 79-85):** for a weighted least-squares problem
  `min_x ‖W_x(Jx − b)‖`, substituting `x = W_q y` gives
  `min_y ‖(W_x J W_q) y − W_x b‖` — the weighted Jacobian is `J` with each
  row scaled by its Cartesian weight and each column scaled by its joint
  weight, and the weighted target is `b` scaled by the Cartesian weight.
  Scaling a matrix's rows by one vector and its columns by another is the
  definition of pre/post-multiplying by two diagonal matrices; there is
  exactly one way to do it element-wise, so independent authorship produces
  the identical row/column scaling regardless of prior exposure to
  `Eigen::DiagonalMatrix::asDiagonal()`. (Upstream's own header comment
  states the formula this expands, `W_q * (W_x * J * W_q)^# * W_x * v_in`
  — a documented interface fact, the standard WLS reduction, not
  KDL-authored expression.)
- **Pseudo-inverse reconstruction (lines 97-103):** the Moore-Penrose
  pseudo-inverse via SVD is the textbook identity `A^+ = V Σ^+ U^T`, where
  `Σ^+` replaces each singular value with `f(σ_i)` (its reciprocal for the
  exact pseudo-inverse; a truncated or damped variant for a regularized
  one). Unpacking that identity into `x = A^+ b` as matrix-vector
  operations gives exactly `U^T b` first, scale each entry by `f(σ_i)`,
  then multiply by `V` — there is no other order these three matrix
  multiplies can be grouped into given the same three factors, so this is
  the direct expansion of the formula, not a copy of any codebase's loop
  (in particular, not a copy of `chainiksolver_vel_mimic_svd.cpp`'s
  `qdot_out_reduced_.noalias() = svd_.solve(vin.topRows(rows))`, which is a
  single opaque `solve()` call into `Eigen::JacobiSVD`'s own C++ internals
  this port cannot call or read the implementation of via `nalgebra`).

**Totals:** meta 6 · original 120 (5+1+1+12+12+1+10+2+76) · bucket 1
(transcription) **29** (19+10) · bucket 2 **14** (7+7) · bucket 3
(interface fact) **12** · bucket 4 **0**. Sum: 6+120+29+14+12+0 = 181 ✓.

Of LGPL-relevant content (buckets 1-3, 55 lines): transcription
29/55 ≈ **52.7%**, independently-derivable 14/55 ≈ **25.5%**, interface-fact
12/55 ≈ **21.8%**.

**Conclusion: (C) partial rewrite**, and this file also needs the copyright
line dropped regardless of bucket content, since nothing here is
BSD-sourced. `fold_jacobian` and `expand_to_full` (29 lines, both function
body *and* doc) are bucket 1 and need D11 rewrite. `solve_velocity`'s core
weighting and pseudo-inverse reconstruction (14 lines) are already bucket 2
— correctly independently derived — but its surrounding doc text still
needs the upstream-citation-as-justification phrasing removed from the two
bucket-1 functions' comments (that phrasing is itself part of what bucket 1
measures: doc text asserting "exactly as upstream does" is evidence of
transcription, not just the code).

## `newton_raphson.rs` (166 lines)

Header (pre-fix) cites both `chainiksolver_vel_mimic_svd.cpp` and `.hpp`
under `Ported from`, with no LGPL copyright line (only `Copyright (c) 2008,
Willow Garage, Inc.`, which is `kinematics_base.hpp`'s own BSD copyright —
present in this file already for a different reason, see below). Unlike
`velocity.rs`, this file's defect is **not** transcription — it is a
**miscited header**: nothing in this file's content actually derives from
`chainiksolver_vel_mimic_svd.{hpp,cpp}`'s text at all.

| Region | Lines | Span | Classification | Basis |
|---|---|---|---|---|
| File header | 1-7 | 7 | meta | Contains the defect: cites the LGPL file under `Ported from` though nothing here is sourced from it |
| Imports + blanks | 8-21 | 14 | original | — |
| `NewtonRaphsonSolver` struct doc | 22-28 | 7 | **3** | Names `Eigen::JacobiSVD::setThreshold`'s own documented RELATIVE-mode semantics — Eigen is MPL2-licensed and this is citing Eigen's *public API contract*, not `chainiksolver_vel_mimic_svd.cpp`'s text; the LGPL file's only role here is that its constructor happens to call `setThreshold`, an interface fact (which threshold mode is in effect), not expression |
| Struct decl + blanks | 29-41 | 13 | original | This port's own fields (`model`,`chain`,`params`,`joint_weights`,`rng`) — `rng` in particular has no KDL/Eigen counterpart at all |
| `new`/`new_with_seed` docs + bodies + impl braces | 42-95 | 54 | original | Corresponds to `KDLKinematicsPlugin::initialize`'s solver-construction tail (BSD, `kdl_kinematics_plugin.cpp`) per this crate's own `lib.rs` symbol-coverage audit — not `chainiksolver_vel_mimic_svd`; the RNG-seeding deviation writeup is this port's own reasoning |
| `KinematicsSolver` impl: `group_name`/`joint_names`/`base_frame`/`tip_frame` + blanks | 96-112 | 17 | original | Trivial delegation to `self.chain.*` — `ChainInfo` accessors, this crate's own type |
| `solve_with_options` sig + `svd_threshold` local | 113-119 | 7 | original | — |
| `pinv` closure | 120-126 | 7 | **2** | See derivation below |
| Comment block (state initialization) + body rest + impl close | 127-154 | 28 | original | This port's own reasoning about `RobotState::new`/`set_to_default_values`, no upstream counterpart |
| blank + registration comment + `distributed_slice` static | 155-166 | 12 | original | This crate's own compile-time registry mechanism (D4), no upstream counterpart |

**Bucket-2 derivation:**

- **`pinv` closure (lines 120-126):** `if s > threshold * smax { 1.0 / s }
  else { 0.0 }` is the direct statement of "truncate to zero any singular
  value whose ratio to the largest singular value is at or below the
  threshold, otherwise take its reciprocal" — the RELATIVE-threshold
  truncated pseudo-inverse rule, which is Eigen's own public,
  documented `setThreshold` semantics (referenced by name in the struct
  doc above), not a rule stated anywhere in
  `chainiksolver_vel_mimic_svd.{hpp,cpp}`'s own text — that file only calls
  `svd_.setThreshold(threshold)` and later `svd_.solve(...)`; the truncation
  rule itself lives inside Eigen's `JacobiSVD` implementation, which this
  port has never read (`nalgebra` is a different, unrelated codebase).
  Independently arriving at "compare `s` against a relative threshold, else
  reciprocal" from Eigen's own public contract is the only way to reproduce
  documented RELATIVE-threshold behavior at all.

**Totals:** meta 7 · original 145 (14+13+54+17+7+28+12) · bucket 1
(transcription) **0** · bucket 2 **7** · bucket 3 (interface fact) **7** ·
bucket 4 **0**. Sum: 7+145+0+7+7+0 = 166 ✓.

**Conclusion: (B) header-citation-only defect.** Zero bucket-1 lines — no
rewrite needed. The header wrongly lists
`chainiksolver_vel_mimic_svd.{cpp,hpp}` under `Ported from`; this file's
actual content derives from moveit2's own BSD `kdl_kinematics_plugin.cpp`
(`initialize`'s construction tail) and from Eigen's own public API
semantics, neither of which is the LGPL file. Fix: correct the citation,
not the code.

## `lib.rs` (467 lines)

Header (pre-fix) cites both `kinematics_base.{hpp,cpp}` and
`kdl_kinematics_plugin.{hpp,cpp}` (BSD) alongside
`chainiksolver_vel_mimic_svd.{hpp,cpp}` (LGPL) and `joint_mimic.hpp` (BSD —
confirmed by reading its own header: `Copyright (c) 2012, Willow Garage,
Inc.`, `Author: Sachin Chitta`, no LGPL text anywhere in the file), and
carries both `Copyright (c) 2007, Ruben Smits` and `Copyright (c) 2013,
Sachin Chitta, Willow Garage` — both are the LGPL file's own copyright
lines (confirmed: neither name appears as a *copyright holder* — as
opposed to an `Author:` attribution comment, which BSD files use for
Sachin Chitta and carries no license implication — in any of the BSD files
this header also cites).

Content-wise, this file is almost entirely module-level doc: a
symbol-by-symbol audit of which Rust item ports which upstream method,
plus five `mod` declarations and six `pub use` re-exports. Read in full for
this measurement: it does not, anywhere, quote or paraphrase a sentence of
`chainiksolver_vel_mimic_svd.{hpp,cpp}`'s own comments or code — it names
*symbols* (`jacToJacReduced`, `CartToJnt`, `isPositionOnly`, ...) and states
this port's own design decisions about them in this port's own prose. That
is squarely interface-fact reuse (bucket 3): naming which upstream method a
Rust item corresponds to is a pointer, not expression, by the same
reasoning `path_circle.rs`'s "Why this file stays BSD-3-Clause" section
already applies to argument roles and calling conventions.

| Region | Lines | Span | Classification | Basis |
|---|---|---|---|---|
| File header | 1-15 | 15 | meta | Contains the defect: LGPL copyright (lines 3-4), LGPL file citations (lines 13-14) |
| blank | 16 | 1 | original | — |
| Module doc (crate overview, symbol-coverage audit, `mod`/`pub use`) | 17-467 | 451 | **3** | Pure prose naming upstream symbols and this port's own Rust items — no upstream text is quoted or paraphrased anywhere in this file. One wording defect independent of bucket, not requiring code rewrite: lines 151-152 say `jacToJacReduced` — ported as `velocity::fold_jacobian`; the inverse qdot-expansion loop is ported as `velocity::expand_to_full`" — "ported as" overstates it for content round 22 re-derives from first principles; needs to read "re-derived as" once `velocity.rs`'s fix lands |

**Totals:** meta 15 · original 1 · bucket 1 **0** · bucket 2 **0** ·
bucket 3 (interface fact) **451** · bucket 4 **0**. Sum: 15+1+451 = 467 ✓.

**Conclusion: (B) header-citation-and-copyright-only defect.** Zero
bucket-1 lines — no prose rewrite needed anywhere in the 451-line audit
body. Drop the two LGPL copyright lines, remove the two
`chainiksolver_vel_mimic_svd.{cpp,hpp}` citations from `Ported from` (kept:
`kinematics_base.{hpp,cpp}`, `kdl_kinematics_plugin.{hpp,cpp}`,
`joint_mimic.hpp` — all confirmed BSD), add a "Why this file stays
BSD-3-Clause" section pointing to `velocity.rs`'s own (since that is where
the actual re-derivation lives), and fix the "ported as" → "re-derived as"
wording at lines 151-152.

## Cross-file summary

| File | Total | meta | original | B1 transcription | B2 derivable | B3 interface | B4 moveit2-BSD | Verdict |
|---|---|---|---|---|---|---|---|---|
| `velocity.rs` | 181 | 6 | 120 | 29 | 14 | 12 | 0 | (C) — rewrite `fold_jacobian`,`expand_to_full`; drop LGPL copyright |
| `newton_raphson.rs` | 166 | 7 | 145 | 0 | 7 | 7 | 0 | (B) — header citation only, no code rewrite |
| `lib.rs` | 467 | 15 | 1 | 0 | 0 | 451 | 0 | (B) — header citation + copyright only, one wording fix |

39 lines across the three files (29 in `velocity.rs`, 0 elsewhere) are
bucket-1 transcription and are the D11 rewrite scope for the next round of
commits. The other two files' defect is entirely in their header
copyright/citation block, not in any transcribed expression — confirming
the brief's framing: the LGPL exposure here is narrower than round 21's
(one file's two functions), but it reaches three files because the
miscitation itself (an LGPL file's path copy-pasted into a sibling file's
header without checking whether that sibling's *content* actually came from
it) propagated from `lib.rs` into `newton_raphson.rs` — both cite the LGPL
file without ever having transcribed from it.

## Addendum — `cart_to_jnt.rs`, found by the post-fix re-sweep

Not in the brief's named population. Found after the three files above were
fixed, by re-running the anchor sweep (`rg 'Sachin Chitta|Ruben Smits'
crates/moveit-kinematics/src/`) as CLAUDE.md's "Fixes from reported
defects" protocol requires after any citation-based fix — a citation is a
sample, not the population, and this file's defect uses a different
signal than the other three: it carries the LGPL file's exact copyright
line (`Copyright (c) 2013, Sachin Chitta, Willow Garage`) with **no
`chainiksolver_vel_mimic_svd` path citation anywhere in the file** — the
round 22 brief's own sweep (path-citation grep) structurally cannot see a
bare copyright string with no accompanying citation, which is why it
surfaced only here, not in the original population.

`cart_to_jnt.rs`'s `Ported from` list names only
`kdl_kinematics_plugin.cpp` (confirmed BSD, `Copyright (c) 2012, Willow
Garage, Inc.`, `Author: Sachin Chitta, David Lu!!, Ugo Cupcic` — Chitta is
credited, not a copyright holder) at three accurate line ranges
(`CartToJnt` 417-497, `clipToJointLimits` 499-522, `searchPositionIK`
303-415). The file was read in full (798 lines, all of `cart_to_jnt`,
`clip_to_joint_limits`, `apply_full`, `error_twist`,
`random_configuration`, `near_by_configuration`,
`satisfies_consistency`, `search_position_ik`, and the test module) to
confirm this: it never calls, quotes, or paraphrases anything from
`chainiksolver_vel_mimic_svd.{hpp,cpp}` — its only interaction with the
LGPL-adjacent velocity solve is a black-box call to `solve_velocity`
(fixed above, `velocity.rs`), passed as an opaque function value.

**Classification: header-copyright-leak only, zero bucket-1/2/3/4
content to account for** — there is no LGPL citation to remove from
`Ported from` (there never was one) and no upstream-justification prose
to strip (the file's doc comments cite only `KDLKinematicsPlugin`
methods, all BSD). The single defect is line 2's copyright grant itself,
which is not sourced from any file this header cites — not even from
`kdl_kinematics_plugin.cpp`, whose own copyright is `Copyright (c) 2012,
Willow Garage, Inc.` with no Chitta copyright line at all. This is a
copy-paste artifact (most likely picked up from a sibling file during
authoring, before this round's fix), not a mis-scoped citation.

**Fix:** drop line 2 (`Copyright (c) 2013, Sachin Chitta, Willow
Garage`) from the header; no other line in the file changes. Confirmed by
re-running the anchor sweep after this fix: no remaining occurrence of
`Sachin Chitta, Willow Garage` or `Ruben Smits` anywhere in
`crates/moveit-kinematics/src/` outside explanatory doc prose in `lib.rs`
and `velocity.rs`, and no remaining `chainiksolver_vel_mimic_svd`
reference outside the three files' own "why this file stays BSD" prose
and this document.

Line 1's `Copyright (c) 2008, Willow Garage, Inc.` is left unchanged.
This is a **distinct, pre-existing inaccuracy, not the same defect**: the
file's actual cited source (`kdl_kinematics_plugin.cpp`) is dated 2012,
not 2008, so the year is wrong — but the copyright holder
(`Willow Garage, Inc.`) and license class (BSD) are both correct, so this
is not a license-compliance defect and is outside D11/§151/§152's scope
(LGPL-vs-BSD boundary), which is what this round's brief asked for. Not
fixed here; noted so it is not silently carried forward as if unnoticed.
