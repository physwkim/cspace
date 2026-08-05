# Assertion-discrimination census, reconciled

Assertion-discrimination round 2 brief §4, closure artifact. Report only —
no source changes. Measured at `df36fab` / merge `f9c03cf` (current worktree
HEAD at the time of this census).

Three instruments disagreed this round by 1-2 sites per crate: the
orchestrator's `matches!`-after-`assert!` regex, p9-ros's paren-depth-tracked
scan, and each panel's hand enumeration. This document is one instrument,
stated precisely, applied to the whole tree, cross-validated line-by-line
against the brief's own recommended `rg` commands, with every disagreement
opened and resolved by reading the source — not by re-running the regex.

## 1. The instrument

For every `.rs` file under `crates/`, `ros/`, `tools/` (excluding `target/`):

1. Mask out comments — `//...` line comments and `/* ... */` block comments
   (nesting-aware), replaced with spaces so line numbers are unaffected.
   String-literal contents are left untouched. This is what excludes doc
   comments that quote the defect pattern as prose (`` `matches!(err,
   Error::Other(_))` alone cannot tell a test that... ``) — the false
   positive the brief's third and fifth corrections both hit.
2. Find every literal `assert!(`. For each, find its matching close-paren
   by tracking paren depth over the **unmasked** original text (so a `)`
   inside a string doesn't close the macro early). This gives the full
   assert body with no length cap, unlike the brief's own recommended
   `[\s\S]{0,300}?` anchor2 regex.
3. Classify each `assert!(...)` site at most once (match-start dedup),
   in priority order:
   - **`matches!`** — immediately after `assert!(`, modulo whitespace and
     newlines, the next token is `matches!(`. Enum-agnostic and
     path-agnostic by construction (no `Error::` literal anywhere in the
     rule), so it does not miss `PipelineError`, `PlanError`,
     `ResponseAdapterError`, `DecodeError`, `Diagnostic`, or any
     fully-qualified `moveit_error::Error::…` path — the blind spots the
     brief's fourth correction found in the old anchor2.
   - **`bare`** — not `matches!`, and the full assert body (no cap)
     contains `.is_err()` or `.is_none()` anywhere.
   - otherwise not counted (`assert_eq!`, `.is_ok()`, `.is_some()`, bool
     flags, domain-specific comparisons — not this family).
4. No restriction to `#[test]`/`#[cfg(test)]` scope, matching the brief's
   own final commands; in this tree essentially every hit is in test code
   regardless.

Ambiguous cases and the rule applied:

- **`matches!` inside a larger boolean** (`a.is_ok() || matches!(err, ...)`)
  — not counted as `matches!`, because the token immediately after
  `assert!(` is not `matches!(`. None were found in this tree; recorded so
  the rule is explicit rather than silently decided by what the regex
  happens to match.
- **A `matches!` call whose own body incidentally contains `.is_err()`/
  `.is_none()` as text** — counted once, as `matches!`, never double
  counted as `bare`. Pathological, not observed, foreclosed by the
  priority order regardless.
- **Doc-comment code samples** — masked like any other comment, so never
  counted, per the brief's third/fifth corrections.
- **Production-code `.is_none()`** (not inside an `assert!`) — not
  counted; the family is about assertions, and the rule only ever looks
  inside an `assert!(...)` body.

## 2. Cross-validation against the brief's own recommended commands

```
rg -U -c 'assert!\(\s*matches!\('                          crates/ ros/ tools/
rg -U -c 'assert!\(\s*[\s\S]{0,300}?\.(is_err|is_none)\(\)' crates/ ros/ tools/
```

Run against the identical tree, summed per crate, and compared line-by-line
against this instrument's output:

- **`matches!` anchor: zero disagreement**, all 20 non-zero crates, exact
  match on every row (82 = 82).
- **`bare` anchor: two crates disagreed**, both opened and resolved:

  - **`moveit-geometry`**: `rg` reports 35, this instrument reports 34.
    `rg -o --byte-offset` shows the extra match starts at
    `bodies.rs:4042` (`assert!(Sphere::new(0.0).is_ok());` — an `.is_ok()`
    assert, not part of this family) and, because that line contains
    neither `is_err` nor `is_none`, the non-greedy `{0,300}?` keeps
    consuming forward *through the next test's doc comment* until it hits
    the literal text `` `.is_err()` `` quoted in prose at `bodies.rs:4048`
    (`/// single combined \`||\` guard -- a bare \`.is_err()\` cannot tell
    the two apart`). One assert with no real hit, glued by the regex to an
    unrelated comment 6 lines later. **This is a new false-positive mode**,
    distinct from the three the brief already recorded: it isn't a comment
    containing the assert, it's a non-matching assert whose lookahead
    window swallows a *later, unrelated* comment. Confirmed by reading
    `bodies.rs:4037-4048`; real count is 34.
  - **`moveit-constraints`**: `rg` reports 4, this instrument reports 5.
    `utils_parity.rs:879` (`assert!(resolve_position_constraint_frame(...)
    .unwrap().is_none())`) spans lines 879-889 with heavy indentation and
    a 6-argument call; its own `.is_none()` sits past the 300-char cap, so
    the anchor2 regex fails to match starting there at all and instead
    matches only the *next* assert at `:890`
    (`resolve_orientation_constraint_frame`, same shape). This is exactly
    the under-count mode the brief's own corrections already named ("misses
    ... anything whose assert body ... exceeds the `{0,300}` cap") —
    reproduced here on a currently-live file. Confirmed by reading
    `utils_parity.rs:879-904`: both are genuine, distinct sites; real
    count is 5.

  These two disagreements happen to cancel in the workspace-wide sum
  (+1/-1), which is why the two totals matched exactly (288 = 288) despite
  two crate rows each being wrong under `rg`. **A total agreeing is not
  evidence every row agrees** — this is the same shape of trap as
  half-a-table vouching for the whole; both rows had to be opened
  individually to catch it.

- **Zero-hit crates independently confirmed**, not merely absent from
  output: `moveit-error` (2 `assert!` total, neither matches — one is
  `.is_success()`, not `.is_none()`), `moveit-stomp-core` (14 `assert!`,
  `rg -l` for either anchor returns no file), `moveit-test-support` (2
  `assert!`, same), `tools/moveit-diff` (31 `assert!` across `main.rs` +
  `harness.rs`, all domain comparisons or `.is_null()` — a serde-JSON
  method, not `Option::is_none` — or already-destructured `matches!` used
  as a plain boolean expression outside any `assert!`, e.g.
  `main.rs:1057`).

## 3. Spot-reads

At least 2 hits opened per non-zero crate (source line + 1 line of
context each), confirming the classification is a real `assert!` of the
stated shape, not a scanner artifact. All 20 non-zero crates checked;
zero misclassifications found beyond the two `rg`-side disagreements in
§2 (which are `rg` errors, not errors in this instrument — confirmed by
reading the source directly rather than trusting either scanner's printed
number).

## 4. Final table

| crate | `matches!` | `bare` | total |
|---|---:|---:|---:|
| `moveit-collision` | 2 | 43 | 45 |
| `moveit-geometry` | 5 | 34 | 39 |
| `moveit-planners-pilz` | 15 | 16 | 31 |
| `ros/moveit-ros` | 26 | 2 | 28 |
| `moveit-distance-field` | 2 | 18 | 20 |
| `moveit-model` | 5 | 15 | 20 |
| `moveit-trajectory` | 0 | 16 | 16 |
| `moveit-planners-chomp` | 8 | 6 | 14 |
| `moveit-scene` | 1 | 13 | 14 |
| `moveit-octomap` | 0 | 10 | 10 |
| `moveit-state` | 1 | 9 | 10 |
| `moveit-constraints` | 4 | 5 | 9 |
| `moveit-planners-sbp` | 3 | 4 | 7 |
| `moveit-planners-stomp` | 0 | 7 | 7 |
| `moveit-planning` | 2 | 2 | 4 |
| `moveit-sampling` | 0 | 4 | 4 |
| `moveit-kinematics` | 1 | 2 | 3 |
| `moveit-metrics` | 3 | 0 | 3 |
| `moveit-smoothing` | 2 | 0 | 2 |
| `moveit-srdf` | 2 | 0 | 2 |
| `moveit-error` | 0 | 0 | 0 |
| `moveit-stomp-core` | 0 | 0 | 0 |
| `moveit-test-support` | 0 | 0 | 0 |
| `tools/moveit-diff` | 0 | 0 | 0 |
| **TOTAL** | **82** | **206** | **288** |

24 crate/tool directories checked (20 workspace crates + `ros/moveit-ros` +
`tools/moveit-diff`, which is the only `tools/` member with `.rs` files —
`moveit-oracle` is Python/C++, `mpr-vs-epa` is C, `ci` is shell). **288 is
the sweep's denominator, measured at `df36fab`.**

## 5. Reconciliation against every per-crate figure this session quoted

The count moves for two different reasons, and conflating them would
misreport the sweep's own progress:

- **Real reason: fixes converting sites out of the family.** A site that
  gets its `matches!` destructured into a `match`/`assert_eq!`, or its
  bare `.is_err()` replaced with `err.to_string().contains(...)`, stops
  matching either anchor — it is *closed*, not hidden.
- **Real reason: new tests merged after an earlier measurement.** Sibling
  panels keep landing commits; a crate's count can go up between two
  honest measurements with no instrument disagreement at all.
- **Instrument reason: an anchor's own known blind spots** (300-char cap,
  comment inclusion, comma/path/enum blindness) — the failure mode, not a
  code change.

| crate | prior figure (source) | this census | direction | why |
|---|---|---:|---|---|
| `moveit-trajectory` | 16 bare / 0 matches (this panel, hand enumeration, all 16 mutation-confirmed) | 16 / 0 | unchanged | exact match — independent confirmation |
| `moveit-geometry` | "mine 6/35 vs p9-ros's 5/34" (orchestrator, this conversation) | 5 / 34 | matches p9-ros | orchestrator's own 6 was wrong; p9-ros's 5/34 reproduces exactly |
| `moveit-planning` | "4 matches!, 2 bare" (orchestrator quote, dispatch) / this panel's own hand count of 5 matches! + 2 bare = 7 pre-fix | 2 / 2 | down from 7 | **not an instrument disagreement** — `c9ea56f` and `df36fab` (this session) converted 3 of the 5 `matches!` sites (`pipeline.rs:670,651,687`) into `match`/`assert_eq!` destructuring. Only `pipeline.rs:634` (`NoPlanners`, nullary variant) and `add_time_optimal_parameterization.rs:336` (single-branch) remain as `matches!`; `plan_responses.rs:208,214` remain `bare` (container-passthrough / vacuous-loop-result, no defect, not fixed). The orchestrator's quoted 4 was itself never reproduced by either enumeration — this panel's own pre-fix count was 5, not 4 — so both a corrected pre-fix baseline (5, not 4) and a fix (5→2) are folded into this one row. |
| `moveit-collision` | 42 bare / 0 matches (brief §4 corrected table, measured at `f6cbbb5`) | 43 / 2 | up | **not an instrument disagreement** — `git log --oneline -- crates/moveit-collision` shows 90+ commits after `f6cbbb5` (MPR-plateau investigation, `deviation 6` work, `585a79e` group_name fix, `e6c9945` new shape-absent-guard coverage). New test code landed; not a re-measurement of the same tree. |
| `moveit-constraints` | 3 anchor1(low)/8 bare(high)/3 anchor2 at `f6cbbb5`; 4 matches/3 bare in the 712dafe superseded table | 4 / 5 | — | `f6cbbb5`'s own table gives a low/high bracket (3-8) that this census's 5 falls inside; not a fresh disagreement, and p1-robotmodel's `new_rejects_unknown_joint` fix (top-of-branch commit, this round) already changed one site's shape since either snapshot. |
| every other crate | see brief §4 corrected table | see §4 above | mixed | not independently re-derived line-by-line here beyond the spot-reads in §3; `moveit-trajectory` and `moveit-geometry` are the two rows this panel can certify from first-hand mutation/enumeration work this round, plus its own `moveit-planning`/`moveit-collision` work above. |

The `moveit-planning` and `moveit-geometry` rows are the two that matter
most for what this document is for: they show a quoted count can be wrong
*and* the true count can still move afterward for an entirely separate,
legitimate reason (a landed fix). A reconciliation that reports only "5 vs
4, resolved" without also naming the fix-driven 5→2 would understate how
much of the family this round actually closed.

## 6. Commands run

```
python3 <census.py>                                                  # this instrument, full tree
rg -U -c 'assert!\(\s*matches!\('                          crates/ ros/ tools/
rg -U -c 'assert!\(\s*[\s\S]{0,300}?\.(is_err|is_none)\(\)' crates/ ros/ tools/
rg -U -o --byte-offset 'assert!\(\s*[\s\S]{0,300}?\.(is_err|is_none)\(\)' crates/moveit-geometry/src/bodies.rs
rg -l 'assert!\(\s*matches!\(|\.is_err\(\)|\.is_none\(\)' crates/moveit-error/ crates/moveit-stomp-core/ crates/moveit-test-support/ tools/moveit-diff/
git log --oneline -- crates/moveit-collision | wc -l
```

`census.py`'s full source (mask-comments + paren-depth-matching + priority
classification, as described in §1) is retained at
`/tmp/claude-1000/.../scratchpad/census.py` in this panel's worktree
session — reproducible on request, not committed (scratch tooling, not a
workspace artifact).

## 7. Scope and gate

Report only. No `crates/` source touched by this task — the two commits
this panel made to `moveit-planning` (`c9ea56f`, `df36fab`) predate and are
independent of this census and were already gated and reported separately.
This document lives at repo root `doc/`, outside any crate, so no
`cargo fmt`/`clippy`/`nextest` scope applies to it.

**UNFIXED:** none opened by this task — it is a measurement, not a fix.
Recorded, not actioned (outside this task's scope): the new
`rg`-swallows-a-later-comment false-positive mode in §2 is worth folding
into the brief's own corrections list if another panel re-runs the raw
regex commands instead of this instrument.
