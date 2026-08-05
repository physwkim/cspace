# Assertion-discrimination census, reconciled

Assertion-discrimination round 2 brief §4, closure artifact. Report only —
no source changes. Measured at `df36fab` / merge `f9c03cf` (current worktree
HEAD at the time of this census).

> **`df36fab` is a panel worktree tip, not `main`.** Everything below was
> measured on a branch that was missing commits already merged to `main`,
> so **288 was never the `main`-tree figure** — see §8 for the `main`
> measurement and the per-crate deltas. The instrument analysis in §1-§3 and
> the reconciliation in §5-§7 are unaffected; only the denominator is.

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
    The two asserts are `utils_parity.rs:879`
    (`assert!(resolve_position_constraint_frame(...).unwrap().is_none())`,
    body 260 chars) and `:890`
    (`resolve_orientation_constraint_frame`, same shape but carrying an
    inline `OrientationTolerance::RotationVector { .. }` literal, body
    **430** chars). The capped regex matches `:879`, whose `.is_none()`
    falls inside the window, and misses `:890`, whose does not. This is
    exactly the under-count mode the brief's own corrections already named
    ("misses ... anything whose assert body ... exceeds the `{0,300}`
    cap") — reproduced here on a currently-live file. Confirmed by reading
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
  `!matches!(shape, Shape::Mesh(_) | Shape::OcTree(_))` at
  `main.rs:1674`).

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

## 8. The `main`-tree measurement

The table in §4 was measured on `df36fab`, a panel worktree tip. Five
commits that were already merged to `main` at the time are not ancestors of
it (`git merge-base --is-ancestor <sha> df36fab` is false for each):
`b0a9e1b`, `08976b8`, `5ea2418`, `46cd26b`, `7676185`. Two more landed
after: `bb4c487`, `bedc8ea`. So §4's 288 is a branch figure, and four
panels — and the round-8 ledger briefs — used it as if it were `main`'s.

Re-measured on `main` at `6192370` with an independently written scanner
implementing §1's rule from its prose (mask comments, paren-depth match,
`matches!` priority over bare, match-start dedup):

| crate | §4 (`df36fab`) | `main` @ `6192370` | delta | cause |
|---|---:|---:|---:|---|
| `moveit-collision` | 45 | 47 | +2 | `3c9cdca`, `7e0bb55`, `035d4b1` merged after |
| `moveit-planners-pilz` | 31 | 32 | +1 | `bb4c487` added one `assert!(matches!(` |
| `moveit-model` | 20 | 22 | +2 | `46cd26b`, `7676185` |
| `ros/moveit-ros` | 28 | 27 | -1 | `b0a9e1b` converted a bare `matches!` out of the family |
| `moveit-distance-field` | 20 | 18 | -2 | `08976b8`, `5ea2418` converted two `.is_err()` sites out |
| `moveit-scene` | 14 | 13 | -1 | `bedc8ea` converted one `.is_none()` to `assert_eq!` |
| all others | — | unchanged | 0 | — |
| **TOTAL** | **288** | **289** | **+1** | |

**`main` @ `6192370`: 82 `matches!` + 207 bare = 289.**

Every delta is a moved tree, not an instrument disagreement — each is a
named commit that either added a site or converted one out of the family.
Three panels re-measured their own crates against §4 independently and
their figures agree with this table exactly, per crate, in every case.

The lesson is the one §2 already states, one level up: a census must name
the tree it measured, and a reader must check that the tree is the one they
care about. §4 named its tree honestly; the readers, this orchestrator
included, did not check what that name resolved to.

## 9. Family membership — the rule three ledgers disagreed about

§1's 289 is a **syntactic** pre-filter: every `assert!(matches!(...))` and
every `assert!(...)` whose body contains a bare `.is_err()`/`.is_none()`.
It says nothing about whether a given site is actually an instance of the
defect this whole sweep hunts. Three ledgers this round used a fourth
verdict, `not-this-family`, to say "this site matched §1's syntax but isn't
really one of these" — p1-robotmodel (this document's author) on 4 of 55
rows, p9-ros on 17, p1-fixtures on 0 of 49 — and nobody had written down
what the category meant. That gap produced a real error: p9-ros excluded
`robot_model.rs:2025`, the exact site `7676185` (this session) proved
fixture-vacuous and fixed. This section is the rule, so the next reader
gets the same answer independently of who is asking.

**The defect, restated precisely.** Every verdict in this sweep —
`discriminating`, `single-branch`, `fixture-collapse-fixed` — is an answer
to one question about one assertion: *can this assertion name what
produced its result?* `not-this-family` is the answer "that question does
not apply here" — not "the answer is yes." A site is a **member** of the
family — eligible for that question, whatever the answer turns out to be
— only if all three of the following hold. Any one failing means the
question was never applicable, and the verdict is `not-this-family`.

1. **Mechanism.** The value the assertion inspects is (or is derived
   without added computation from) a signal the code under test uses to
   report *"this operation could not, or did not, produce X"* — a
   `Result::Err`, an `Option::None`, or a boolean/variant standing in for
   one of these (a coarse "found nothing" or "it failed" tag). It is
   **not** a plain, informative value returned on a success path — a
   computed enum tag, a numeric result, a struct field — because such a
   value, when it equals a specific constant, already names in full which
   fact about the world produced it; there is no separate "why" being
   discarded. `assert!(matches!(joint.kind(), JointKind::Fixed))` fails
   this clause: `kind()` is not reporting an inability to do anything, it
   is *reporting a computed fact*, and the fact you observe already is the
   full explanation of itself. This is the same distinction the workspace
   already drew when it disproved "exhaustive match on an input variant is
   never blind": whether an arm's *pattern* matched is a dispatch
   question, and dispatch questions sit outside this family; whether an
   arm's *body* computed the right value is a numeric/logic-correctness
   question, also outside this family. This family is specifically about
   failure/absence signals losing the reason they fired.

2. **Decision.** The signal is produced by an actual, written decision in
   the code under test — a guard (`if cond { return Err(..)/None }`), an
   early return, a `?`-propagation, a loop accumulator whose value depends
   on a comparison the code performs on at least one real element — not an
   *emergent non-decision*: an accumulator whose initial value is simply
   never touched because there was nothing to iterate, or any other case
   where no branch of the subject's logic was actually exercised,
   regardless of what that logic does. The operational test: could an
   engineer have implemented *this specific decision* wrong, in a way a
   mutation at this site could exercise? `shortest_solution_is_none_on_
   empty_input` fails this clause — for an empty slice, the comparison
   logic never runs; `best` stays at its declared initial value no matter
   how that logic is changed, so there is no decision here to get wrong.
   Contrast `nn.rs:227`'s `empty_index_has_no_nearest`: `Gnat::nearest`'s
   `self.root.as_ref()?` is a **written** guard an engineer could have
   omitted or gotten backwards, and the bite (§ledger) confirms removing
   it changes the outcome — clause 2 holds even though, like the
   `shortest_solution` case, the fixture is "empty."

3. **Subject.** That decision belongs to the function or method this test
   exists to verify — not to a step the test performs directly as its own
   setup (calling a std/library function to probe the environment or the
   filesystem, constructing a value the test then inspects without ever
   invoking the subject, or reading back a value the test itself pushed
   into a container moments earlier with no subject-side decision in
   between). The operational test: if you deleted the call to the code
   under test from this test function, would the assertion's outcome be
   unaffected? If yes, the assertion was never about the subject.
   `assert!(std::fs::read(&path).is_err())` labelled `"precondition: ..."`
   fails this clause outright — the test calls `std::fs::read` itself;
   no crate code runs before this assertion at all. `plan_responses.rs`'s
   `solutions[1].is_err()` fails it too, differently: `PlanResponsesContainer`
   is exercised, but the value under test (`Err(failure())`) was supplied
   by the test two lines above and passed through unchanged — the
   container has no comparison capable of turning that `Err` into
   something else, so there is no subject-side decision this assertion
   could be blind to (this one could also be read as a clause-2 failure —
   a passthrough is a decision with no branches — the two clauses overlap
   at the boundary, which is fine: either failing excludes the site).

**Worked resolution of the disputed post-build state check.**
`assert!(j1.mimic().is_none())`, asserted *after* a build that is supposed
to clear it, is the case this document's author's position and the
orchestrator's position converged on, for the same reason: clause 1 holds
(`Option::None` on a mimic-relationship getter is a genuine absence
signal — a joint either has a mimic or it structurally does not, and
`None` collapses "never had one" and "had one, and it was cleared" into
the identical observable). Clause 2 holds (the model-build/mimic-clearing
logic contains a written decision — for each joint, whether to null its
`mimic` field — that an engineer could implement wrong). Clause 3 holds
(that decision belongs to the build/clear routine this test's own name,
"clears every mimic in the model," claims to verify). All three clauses
hold, so the site is **in family**, regardless of the fact that `mimic()`
itself is a plain getter with no branching of its own — the decision
clause 2 and clause 3 look for lives one level up, in the code that calls
the getter's setter, not in the getter. `7676185` is exactly what a
member of this family with only one reachable cause in its fixture (the
old fixture never gave j1 a mimic, so only the "never had one" history was
reachable — a fixture-collapse, not a `not-this-family` exclusion) looks
like once repaired.

This directly generalizes: **the family is not "assertions about a
fallible call's return value"** (that would wrongly exclude the mimic
case, since `mimic()` itself never fails) — **it is "assertions whose
observed absence/failure signal cannot, by itself, distinguish the
subject's intended decision from another decision or non-decision that
produces the identical signal."** A getter reading a field the subject
mutated is exactly as much a member as a guard the subject evaluates
directly, provided all three clauses hold.

**Relationship to the existing verdicts.** `discriminating`,
`single-branch`, and `fixture-collapse-fixed` are all answers to the
family question for sites that passed all three clauses — they differ
only in how many real causes the audit found reachable from the test's
own fixture (one, several-but-distinguished, or several-and-conflated-
until-fixed). `not-this-family` is not a fourth answer to that question;
it is "the question does not apply," decided by the three clauses above,
independently of any bite.

**Applied to this document's own 55 rows** (`doc/assertion-discrimination-
ledger-p1-robotmodel.md`): re-checked all 55 against the three clauses
above, not just the 4 already marked `not-this-family`.

- The 4 existing `not-this-family` rows (`crates/moveit-trajectory/tests/ruckig_smoothing.rs:199`,
  `moveit-planners-chomp/trajectory.rs:997` — both clause-3 failures,
  identical shape to the `fs::read` precondition case; `plan_responses.rs
  :208` — clause-3/clause-2 failure, a container passthrough;
  `plan_responses.rs:214` — clause-2 failure, a vacuous empty-input
  accumulator) all survive re-examination under the stated clauses. None
  is moved.
- None of the remaining 51 rows (`discriminating`, `single-branch`, or
  the resample_dt/`validate_recovery_time_limit`/`extract_seed_trajectory`
  combined-condition or multi-continue guards) fails any clause: every one
  is a genuine `Err`/`None` signal (clause 1), produced by a written guard
  in the code under test (clause 2), belonging to the function the test
  names as its subject (clause 3). No corrections were needed.

**In-family denominator for these 7 crates: 51 of 55** (`moveit-trajectory`
15/16, `moveit-planners-chomp` 12/14, `moveit-planners-sbp` 7/7,
`moveit-planners-stomp` 7/7, `moveit-planning` 2/4, `moveit-sampling` 4/4,
`moveit-kinematics` 3/3). Extrapolating this ratio to the other panels'
crates is not valid — `not-this-family` density depends on how each crate
happens to phrase its precondition/round-trip checks, not on crate size —
so the workspace-wide in-family denominator is not computed here; it
requires each panel to apply this section's three clauses to its own rows,
the same way p9-ros and p1-fixtures were asked to.

### 9a. The verdict vocabulary, including `joint-collapse`

§9 defines *membership*. Once a site is a member, exactly one of these is
its verdict, and no other term may be used without being added here first
— an undefined verdict is what §9 was written to stop, and introducing a
new one in a ledger reopens the same gap one level down.

- **`discriminating`** — the assertion can name what produced its result.
  Evidence: an isolating mutation, in both directions where siblings exist.
- **`single-branch`** — exactly one cause can reach the assertion, so there
  is nothing to name. Evidence: the guard's *condition*, not the
  constructor count (see `doc/folded-operand-guards.md` — a condition that
  folds N named operands into one construction site is N branches).
- **`fixture-collapse-fixed`** — the assertion could not name its cause,
  the fixture was the reason, and it was fixed this sweep.
- **`joint-collapse`** — the assertion cannot name its cause because the
  fixture makes two or more real branches fire together, *and* that is
  correct rather than a defect. This is a narrow verdict with a burden of
  proof, not a place to put hard cases. All three must hold:
  1. every branch that fires jointly here is *independently covered by
     some other test* — so no guard is left with zero coverage;
  2. the subject deliberately collapses the branches into one signal, and
     that collapse is recorded in the code, not inferred;
  3. no caller distinguishes the branches either.

  `moveit-planners-pilz`'s `trajectory_blender_transition_window.rs:1211`
  is the only instance. It qualifies: neutralizing
  `search_intersection_points`'s first `ok_or` alone fails
  `search_intersection_points_rejects_when_first_trajectory_never_reaches_the_blend_radius`
  and neutralizing the second alone fails its `..._second_...` sibling, so
  clause 1 holds and neither guard is uncovered (verified by the
  orchestrator, not inferred from the doc comment); the function's own doc
  comment records the shared error code as matching upstream's bool-only
  `searchIntersectionPoints`, giving clause 2; and `blend`, the only
  caller, never distinguishes the two causes, giving clause 3.

  Without clause 1 this verdict would be indistinguishable from an
  uncovered guard wearing a justification, which is the failure mode the
  whole sweep exists to catch.

**Correction of record:** `doc/assertion-discrimination-ledger-pilz.md`
cited "census §9's D6 exemption" for this site. The census has no D6 in any
section, so the pointer resolves to nothing. D6 itself is real and defined
— `PORTING-PLAN.md:10883` §129.3, "호환은 `TryFrom` 양방향 변환으로만" —
and it is cited throughout `ros/moveit-ros/src/`. It is a rule about not
absorbing failure into a silent default: `moveit_msgs` is wider than the
core types, so conversion is `TryFrom` and rejects rather than defaults.
`PORTING-PLAN.md:15261` §199.2 draws its operative boundary — an
unresolvable lookup stays `Err` (D6), a wire default upstream itself gives
meaning to follows upstream (D14). Neither reading licenses collapsing two
error sites that share a variant, and neither mentions pilz. So the row was
wrong twice: wrong document, and the real D6 does not carry the claim. Its
substance was right and its evidence was real; this subsection is what it
should have cited.

Note also which way the round-2 brief points D6. `doc/assertion-
discrimination-round2-brief.md:105` calls an API that silently collapses
two distinguishable causes "a D6-shaped finding" — the analogy names a
**defect to report**, not an exemption to claim. The pilz row inverted it.

An earlier version of this paragraph said "the brief's single use of 'D6'
is a passing adjective, not an exemption clause." The second half of that
holds — the brief's use is analogical and marks a finding. The first half,
as the orchestrator wrote it to p1-robotmodel ("there is no D6"), was
overreach: it was checked against the census only, never against
`PORTING-PLAN.md`, where D6 has been a defined deviation class since §129.3.

## 9b. The workspace in-family denominator — not yet quotable

§1's 289 is a syntactic candidate count, not the sweep's real denominator
— §9 exists because membership in the family is a separate question, and
§9's own closing paragraph already warns that the in-family ratio is not
transferable between crates (it depends on how each crate happens to
phrase its precondition/round-trip checks, not on crate size). Five row
sets cover the 289 syntactic sites; as of this section, three have had §9
applied and two have not:

| Ledger | Rows | In-family | Not-this-family | §9 applied? |
|---|---|---|---|---|
| `assertion-discrimination-ledger-p1-robotmodel.md` | 55 | 51 | 4 | yes |
| `assertion-discrimination-ledger-p9-ros.md` | 67 | 58 | 9 | yes |
| `assertion-discrimination-ledger-pilz.md` | 32 | 30 | 2 (+1 `joint-collapse`, in-family) | yes |
| `assertion-discrimination-ledger-p1-fixtures.md` | 49 | — | — | **no** |
| `assertion-discrimination-ledger-p3-acm.md` | 86 → 89 | — | — | **no** |

55 + 67 + 32 + 49 + 86 = 289, reconciling exactly against §8's main-tree
count — the five row sets partition the syntactic population with no gap
and no double-count.

**Superseded after this section was written (`d24494d`/`7b92bcc`, merged
`37246a3`).** p3-acm's ledger §1b adds three rows for
`moveit-collision/src/tools.rs:68` (`aabb_intersection`'s
`min[0]>=max[0] || min[1]>=max[1] || min[2]>=max[2]`, one row per axis),
taking its row count 86 → 89 and the partition 289 → **292**. The 289 in
§8 is not wrong and does not move: it is what `ledger_scan.py`'s grammar
measures, and that grammar recognizes only `matches!`-led, `.is_err()`
and `.is_none()` assertion bodies. This site's sole assertion is
`assert!(intersect_cost_sources(&a, &b).is_empty())` — a fourth shape the
scanner cannot see, found by the structural-anchor sweep
(`doc/folded-operand-guards.md`) instead. So 289 is a floor set by one
instrument's grammar, not the syntactic population; every `.is_empty()`-
and `assert_eq!`-shaped assertion in the workspace is still uncounted.
Two of the three axes were blind operands and now have tests; the
per-axis isolating mutation was re-run independently at the merge and
each axis fails exactly one test with the other 198 green.

**154 of 289 syntactic sites classified under §9. 139 of those 154 are
in-family** (51 + 58 + 30). Both figures independently re-derived from
each ledger's own summary section while writing this section, not
transcribed from a prior report: p9-ros's summary states 17
discriminating + 38 single-branch + 3 fixture-collapse-fixed = 58/67,
matching what was relayed; p1-fixtures' and p3-acm's crate-section
headers sum to 49 and 86 respectively, confirming their row counts
independent of the §9 question.

**Outstanding: `p1-fixtures` (49 rows, moveit-scene/octomap/state/
constraints/metrics/smoothing/srdf) and `p3-acm` (86 rows,
moveit-collision/moveit-geometry).** Neither has had census §9 applied —
`p1-fixtures` used `not-this-family` zero times across its 49 rows, from
before the category had a written definition; `p3-acm`'s 86 rows have
never been checked against it at all. Until both land, the true
workspace-wide in-family count is not computable, only bounded: at least
139 (the 154 already classified), at most 139 + 138 = 277 (if every
remaining row happened to be in-family; 138 = 49 + 89 after the `tools.rs`
correction above). **139/292 undercounts** the true figure by however many
of the outstanding 138 turn out in-family — that number is unknown, not
small-by-assumption. And 292 itself is a floor, for the grammar reason
stated above.

**`p1-fixtures` has since landed (`6c56767`, merged `30bd5b0`): 47 of its
49 rows are in-family.** Running total **186 in-family of 203 classified**
(51 + 58 + 30 + 47), leaving `p3-acm`'s 89 rows as the only unclassified
set and narrowing the bound to **≥186, ≤275** against the 292 floor. Its
two reclassifications are worth reading as worked §9 examples, because
neither is a wrong verdict being corrected — both are right answers to the
pre-§9 question:

- `moveit-scene/src/scene.rs:2180` fails **clause 1 (mechanism)**.
  `matches!(outcome, MoveObjectOutcome::Moved(_))` targets the
  success-with-effect arm of a three-variant outcome
  (`world.rs:733-742`: `NotFound`, `NoChange`, `Moved`). The sibling two
  variants *would* satisfy clause 1; `Moved` is a success-path value, so
  the assertion is not this family even though it does discriminate.
- `moveit-constraints/tests/constraint_sampler_manager.rs:172` fails
  **clause 2 (decision)**. Its fixture is empty on every axis
  (`joints=&[]`, `solver=None`, `subgroup_solvers=vec![]`), so Steps A/B/C
  never touch a real operand and the `None` it asserts is an untouched
  accumulator, not a written decision. The file's own doc comment at
  `:105-113` already says so, naming this test while arguing the opposite
  for its neighbour.

The 289 figure remains correct as the syntactic pre-filter count (§1,
§8). It is not the sweep's in-family denominator, and no report from this
sweep should quote 289 as if it were — the same substitution error §8
caught in the 288-vs-289 worktree/main-tree mismatch, one layer further
in: a syntactically-scanned population standing in for a semantically
verified one.

## 9c. `ledger_scan.py` was never committed — and the instrument that replaces it

`doc/assertion-discrimination-ledger-p9-ros.md:10` and
`doc/assertion-discrimination-ledger-p3-acm.md` (nine citations) attribute
their enumeration to a scanner named `ledger_scan.py`. **That file exists
in no commit, no worktree, and no checkout.** `find` across the repository
and `git log --all -- '*ledger_scan*'` both come back empty; it was written
in some panel's scratch space and never landed. p1-joints found this when a
task asked it to read the script's grammar and it could not locate the
file.

Be exact about what this does and does not reach. **§1 does not name that
script and does not depend on it** — §1 specifies its instrument in prose,
step by step (mask comments nesting-aware, find `assert!(`, track paren
depth over the *unmasked* text, classify `matches!` before `bare` with
match-start dedup), and §2 cross-validates that specification against two
`rg` commands, resolving both disagreements by reading source. That is
reproducible from the description by anyone willing to re-implement it. So
289 is not in question here, and §8's independent main-tree re-derivation
stands.

What is in question is narrower and still real: the *grammar* claim. The
statement that the scanner recognizes `matches!` / `.is_err()` /
`.is_none()` and nothing else — the claim that makes `tools.rs:68`'s
`.is_empty()` site invisible, and the claim §9b above leans on when it says
"289 is what `ledger_scan.py`'s grammar measures" — is sourced to a file
nobody can open. It is corroborated from the other direction (`tools.rs:68`
genuinely had no row in either enumeration) but it is not verifiable, and
§9b should have said so instead of citing the script as if it were in the
tree. §1's prose grammar, which *is* readable, says the same thing at step
3: anything that is not `matches!` and contains neither `.is_err()` nor
`.is_none()` is "not counted". Cite that, not the missing script.

`tools/ci/count-coarse-assertions.py` is committed in its place, following
the `count-*` convention already in `tools/ci/`: it lists raw textual hits
and classifies nothing. Its grammar is deliberately wider — `matches`,
`is_err`, `is_none`, `is_some`, `is_empty`, `contains`, `eq_none`,
`eq_err` — and it masks comments, strings and raw strings before matching,
so it sees multi-line assertions that a line-based `rg` cannot.

**686 candidate sites** over `crates/`, `ros/` and `tools/`: 678 in test
scope, 8 in `src` scope. Nine further hits are assertion-helper *bodies*
(scope `helper_body`) and are excluded from that total; their 35 call
sites are counted instead, as kind `via:<fn>`. See §9e for why that
distinction is worth the machinery.

| kind | sites | | kind | sites |
|---|---:|---|---|---:|
| `contains` | 170 | | `via:<fn>` | 35 |
| `is_none` | 120 | | `eq_none` | 32 |
| `is_empty` | 105 | | `is_some` | 27 |
| `matches` | 88 | | `eq_err` | 23 |
| `is_err` | 86 | | (two kinds on one site) | 0 |

This figure superseded an earlier 655 three times, and every revision was
the instrument getting *less* clever rather than more:

- Helper bodies and their call sites (`ccac7ea`). 655 counted five copies
  of `assert_err_mentions`'s own `assert!` as five sites and none of the
  23 places that call it. §9e is the finding that forced this.
- `contains_msg` / `contains_member` collapsed into one `contains`. Two
  rules were tried for that split and both misfile real sites here: a
  backwards look for a rendering call cannot see through a closure
  parameter (`|d| d.contains("only STL")`, five sites in `crates/`),
  and reading the argument instead calls
  `body.touch_links().contains("should_not_survive")`
  (`scene/attached.rs:532`) a message search. Which one a `.contains` is
  depends on the receiver's *type*, and the instrument has no types. It
  now emits one kind and leaves the call to §9 clause 1, where every
  other kind's membership is already decided by reading.
- `None` and `Err(..)` count only as an *operand* of an equality macro.
  The rule used to scan the whole assertion body for a comma-`None`, so
  it fired on `assert!(tree.insert_ray(origin, end, None, false))`
  (`moveit-octomap/src/tree.rs:1781`) — `None` is `max_range`, an argument
  handed to the code under test, not a value asserted about. `p1-fixtures`
  found that one by reading a hit rather than deducting from a total.

That last correction is why the "two kinds on one site" row is now zero
where it was 17. Every one of those 17 was `assert!(matches!(x, Err(..)))`
picking up a spurious second tag; the sites themselves were never at risk,
and only `tree.rs:1781` left the census entirely.

**686 is not a corrected 289 and the two must never be subtracted.** They
are different grammars over scopes that were never stated to be the same.
The number's use is as a *ceiling* the sweep can audit against, and as the
first figure in this document anyone else can reproduce. It is a generous
ceiling: a large part of the 170 `contains` sites is collection
membership, which fails clause 1 outright — but the instrument cannot say
which part, and no figure in this document may be derived by guessing at
the share.

Two things its first run showed that the narrow grammar could not. The
line-based check that has been standing in for it misses multi-line
assertions entirely — on `crates/moveit-distance-field/src/voxel_grid.rs`
a hand `rg` finds 2 sites where the instrument finds 4, and the two it
misses (`:456`, `:512`) are both real. And `ros/moveit-ros` comes to 67,
exactly p9-ros's row count; that is worth *checking*, not celebrating —
two instruments agreeing on a total is the shape that hid two
cancelling per-crate row errors earlier in this sweep.

A worked non-finding, recorded because it is the trap this kind counts
invite: `voxel_grid.rs:454` and `:456` assert the same needle ("must be
finite and positive") against two different inputs, which reads as blind
from the scan output alone. It is not. The test's own doc comment records
the mutation — the sibling overflow guard's message never contains that
text, so the shared needle still names one branch. The scan output is the
question, never the verdict.

### 9d. The denominator closes: 267 in-family of 292

`p3-acm` landed its §9 pass over all 89 rows (`311169b`, merged
`fd3bada`): **81 in-family**, `moveit-collision` 47/50 and
`moveit-geometry` 34/39. That was the last unclassified set, so §9b's
bound collapses to a figure:

| ledger | rows | in-family | not-this-family |
|---|---:|---:|---:|
| `p1-robotmodel` | 55 | 51 | 4 |
| `p9-ros` | 67 | 58 | 9 |
| `pilz` | 32 | 30 | 2 |
| `p1-fixtures` | 49 | 47 | 2 |
| `p3-acm` | 89 | 81 | 8 |
| **total** | **292** | **267** | **25** |

**267 of 292.** Not 139/289, not 186/203, and not a bound — every row in
every ledger has now had all three clauses applied by the panel that owns
it. No verdict (`discriminating` / `single-branch` / `fixture-collapse-
fixed` / `joint-collapse`) changed in any of the three §9 passes: family
membership is a prior and separate question, and a row can be correctly
`discriminating` and still not be this family.

Read 267/292 for exactly what it is. It is the count of assertion sites,
inside one instrument's grammar, that this sweep judged to be about a
coarse-fail signal produced by a written decision in the code under test.
It is **not** a claim that 267 assertions discriminate — that is the
per-row verdict column, not this ratio — and §9c's 686 shows the grammar
itself is a floor. The 25 exclusions concentrate in two shapes worth
naming, since between them they account for 20 of the 25:

- **clause 1, success-path values** (`bodies.rs:4328/4332/4340/4347`,
  `scene.rs:2180`): the assertion checks *which* thing was built on a path
  that always succeeds. Computed dispatch, not a failure signal.
- **clause 2, no decision to be wrong about** (`matrix.rs:660`,
  `octomap_filter.rs:381`, `crates/moveit-geometry/src/shapes.rs:1962`,
  `crates/moveit-constraints/tests/constraint_sampler_manager.rs:172`): a fresh map, a field with zero
  assignment sites, a literal-initialized value, an accumulator no step
  touched. Nothing in the subject decided anything, so no mutation there
  could exercise a wrong implementation.

The clause-2 line is finer than it looks, and `matrix.rs` carries both
sides of it in one file: `:660` reads `default_entry("a")` on a
freshly-`new()`ed matrix (out — no antecedent setter, nothing decided),
while `:880` reads the same getter after `clear()` (in — commenting out
`self.defaults.clear()` fails `:880`'s test alone, 198/199, confirmed at
the merge). Same method, same assertion shape, opposite membership,
settled by running a mutation rather than by arguing from the signature.

### 9e. The instrument undercounts wherever a helper renders the error

`p9-ros` ran the first wide-grammar pass (`207a297`, merged `2dd3169`) and
its deliverable is the count, not the findings: **62 sites in `ros/`, where
this document's brief had quoted 41. Zero colliding needles.**

The arithmetic, reproduced at the merge:

| | |
|---|---:|
| `count-coarse-assertions.py ros` | 72 |
| of those, outside the `matches!`/`.is_err()`/`.is_none()` grammar | 44 |
| minus `assert_err_mentions` helper *bodies* (5 copies, one per file) | 39 |
| plus `assert_err_mentions` *call sites*, invisible to the tool | 23 |
| **real total** | **62** |

Both corrections are structural, not clerical, and they cut in opposite
directions. The tool counts the helper's own `assert!(rendered.contains(
needle), ...)` line once per file — five hits that are one assertion
*mechanism*, not five assertion sites. And the 23 call sites it could not
see at all: a call to `assert_err_mentions` contains no assertion text, so
nothing in the tool's grammar matched it. The instrument's own header
stated this limit; the brief quoting 41 did not apply it.

Both corrections are now in the tool (`ccac7ea`): a body that asserts on
its own parameter is emitted as `helper_body`, and each of its call sites
as `via:<fn>`. Running it today gives **63**, not 62, and the extra site
is real rather than a regrading — `4c56148` ("reach `apply_move`'s
object-pose parse") split `collision_object.rs:1089` into `:1108` and
`:1143` after `p9-ros` measured. 62 was right at `2dd3169`; 63 is right at
this document's HEAD. The pair is worth keeping side by side, because a
hand count and an instrument disagreeing by one is exactly the shape that
gets "reconciled" by adjusting the older number instead of asking what
landed in between.

The quoted 41 was also measured before `f0855c5` merged, which added five
sites of `p9-ros`'s own already-bite-checked tests. **Two independent
errors in one number, and the panel found both by re-measuring rather than
reconciling** — the same lesson §8 recorded when 288 turned out never to
have been `main`'s figure.

Counting call sites of a helper by hand is its own trap: `rg -c` merges
same-line hits, and of the 24 non-definition occurrences of
`assert_err_mentions` in `ros/`, one (`scene/attached.rs:441`) is inside a
comment. 23 is right; the naive occurrences-minus-definitions arithmetic
gives 24.

**Zero collisions** means no needle in `ros/` is also a substring of a
sibling branch's message. Spot-verified at the merge on `state.rs`, which
holds 11 of the 23 call sites: `"JointState.position has length"` comes
only from `set_parallel_array`'s length guard (`:74`), and the wrapper at
`:85` renders `"JointState.{field}: {e}"` — a colon where the other has
` has`, so neither is a prefix of the other; the three
`"JointState.{field}: no variable named"` needles separate by field name.
The four sites sharing `"multi_dof_joint_state has no core
representation"` are one guard with four operands, not four branches —
they are separated by the four isolating mutations in `0148392`, not by
the needle, which is the correct division of labour between a needle and a
mutation.

One latent risk is recorded unfixed and should stay that way:
`scene/collision_object.rs:1089` has two physical call sites inside
`apply_move` sharing one message, and only one is reachable by the current
fixture — so there is nothing live to bite-check today, and a fix would be
a guess. A future edit touching `mv.pose` in that test would misattribute
the failure.

### 9f. The fences are paths, and the partition is checked

Every round of this sweep until now fenced panels by *kind*: `contains_msg`
sites in `crates/` were p1-robotmodel's, `is_empty`/`eq_none` sites were
p1-fixtures', and so on. §9c records why that vocabulary no longer exists.
The failure it caused is worth stating separately from the instrument fix,
because it is not a counting error and no count would have shown it.

A kind fence and a path fence disagree about who owns a site, and when the
kind fence is the one a panel applies while a path fence is the one another
panel was briefed with, sites fall through the gap. Four did:
`moveit-geometry/src/bodies.rs:3953,4055,4066,4125`. `p3-acm` excluded them
from its geometry table as "`contains_msg`-shaped, therefore
p1-robotmodel's" — correct under the rule it was given — and
p1-robotmodel's fence names `moveit-model/`, `moveit-constraints/tests/
decide.rs` and `moveit-planning/`, which does not contain `moveit-geometry`.
Neither panel was wrong. `rg` over `doc/` found no verdict for any of the
four in any ledger.

Two of those four are the reason this matters rather than being
bookkeeping: `bodies.rs:4055` and `:4125` both assert `.contains("radius")`
against a rendered error. If the two constructor guards behind them can
each produce a message containing that substring, neither site names which
one fired — a colliding-needle pair of exactly the shape §4 exists to find,
sitting in the one gap where no panel was looking.

The fences are now paths, and the partition is machine-checkable rather
than argued. Over the 392 sites outside the `matches!`/`.is_err()`/
`.is_none()` grammar:

| panel | fence | sites |
|---|---|---:|
| `p9-ros` | `ros/moveit-ros/`, `crates/moveit-distance-field/` | 102 |
| `p1-fixtures` | `moveit-octomap/`, `moveit-scene/`, `moveit-srdf/`, `moveit-state/`, `moveit-smoothing/`, `moveit-kinematics/`, `moveit-sampling/`, `moveit-test-support/`, `tools/moveit-diff/`, `moveit-constraints/tests/{sampler,utils_parity}.rs` | 105 |
| `p3-acm` | `moveit-collision/`, `moveit-geometry/`, `moveit-planners-{sbp,chomp,stomp}/`, `moveit-stomp-core/` | 81 |
| `p1-joints` | `moveit-trajectory/`, `moveit-planners-pilz/` | 52 |
| `p1-robotmodel` | `moveit-model/`, `moveit-constraints/tests/decide.rs`, `moveit-planning/` | 52 |
| **total** | | **392** |

The check that matters is not the total — it is that every site falls in
**exactly one** fence. Run the instrument, prefix-match each path against
the table, and count sites matching zero fences and sites matching more
than one. Both must be zero. When first run it reported four orphans in
`bodies.rs` and three plus four in `moveit-constraints/tests/`, all seven of
the latter stranded the same way; the table above is the state after those
were assigned. A kind fence cannot be audited this way at all, which is the
substantive argument for paths over any repair to the kind vocabulary.

### 9g. A source read produced two false `discriminating` verdicts

`p1-robotmodel` closed its 52-site fence with **0 blind** (§9f's partition,
merge `1ddb2ac`), the great majority of rows verdicted by reading the
source. It then flagged a coverage gap as explicitly out of scope: of
`MeshSearchPaths::resolve`'s four `None`-producing guards
(`mesh_search_paths.rs:86-89`), two had no test at all.

Sent back to prove the two *tested* guards rather than supplement them, it
found both were blind:

> `:130` (`unknown_package_does_not_resolve`) and `:139,140`
> (`non_package_uri_does_not_resolve`) were both blind — each survived
> neutralization of the guard it was supposed to test, because neither
> fixture had a real file at the joined path, so the chain fell through to
> `is_file()` failing for an unrelated reason.

Both had been verdicted `discriminating` one round earlier. The reading was
not careless — the four guards reject disjoint *inputs*, which is what a
read can see, and each test does supply an input only its own guard
rejects. What a read cannot see is that the fixture never reached the guard
at all: `resolve` is a chain of four `?`s returning one bare `None`, so a
fixture that fails an *earlier* condition than the one it targets produces
a passing test for the wrong reason. Rebuilt on a fixture with a real file
at the joined path (`b64806d`, merged `ded3470`), all four isolate one at a
time.

Reproduced here at the merge, independently of the panel's own run:

| mutation | tests failing |
|---|---|
| `candidate.is_file()` → `(candidate.is_file() \|\| true)` | `missing_file_does_not_resolve` only, 135 green |
| `self.packages.get(package)` → `.or_else(\|\| self.packages.values().next())` | `unknown_package_does_not_resolve` only, 135 green |

The other three tests staying green under the `is_file()` mutation is the
half that carries the information: before `b64806d` they rode that guard,
and now they do not.

One failed attempt is worth recording, because it is the trap on this side
of the check. Mutating `:87` to `rest.split_once('/').unwrap_or((rest,
""))` fails no test — and that is correct, not a gap. `dir.join("")` is
`dir`, which is never a file, so the mutation is behaviour-preserving on
every input. **A mutation that changes no behaviour proves nothing about
the test**, and reading a green result from one as evidence of blindness
is the mirror image of the error this section is about.

The rule this settles, for every remaining fence: a verdict of
`discriminating` reached by reading is a hypothesis. Across the five
ledgers, read-justified rows are already the small minority against
bite-justified ones, which is why this surfaced as two sites rather than
fifty. Where a function funnels several guards into one undifferentiated
`None` or `Err`, reading cannot distinguish "this fixture targets guard B"
from "this fixture reaches guard B", and only a mutation can.
