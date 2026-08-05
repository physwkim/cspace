# Assertion-discrimination ledger — CSV conversions (`moveit-state`)

The three sites the sweep's scanner emits for
`crates/moveit-state/tests/csv_conversions.rs`, the test file added with
`moveit_state::conversions` (the CSV half of upstream
`robot_state/conversions.{hpp,cpp}`). All three are the `contains` kind;
the file's other assertions are `assert_eq!` against exact values,
`assert_ne!`, or `assert!` on `ends_with`, none of which the scanner's
kinds cover.

A separate file rather than rows appended to
`doc/assertion-discrimination-ledger-cached-ik.md`, whose own title and
preamble scope it to `moveit-kinematics`' cached-IK round — the same reason
`discover_ledgers` globs rather than hardcodes.

Verdict legend (same as the other ledgers): **discriminating** (proven to
distinguish ≥2 sibling outcomes), **single-branch** (exactly one
construction site reaches this assertion — nothing to discriminate),
**joint-collapse** (≥2 real sibling sites are asserted at once here and it
cannot say which was wrong), **not-this-family** (excluded by census §9).

## Method

Every row's evidence is a mutation applied to this worktree, run with
`cargo nextest run -p moveit-state --no-fail-fast` (58 tests, 58 passing at
baseline), and reverted, with the revert confirmed by an empty `git status
--porcelain`. Each bite is reported by the *panic line*, not only by the
failing test name: two of the three sites live in one test, and `assert!`
aborts at the first failure, so a test name alone cannot say which of them
fired.

## Sites (3)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-state/tests/csv_conversions.rs:91` | `assert!(!text.contains(','), ...)` — no comma survives when `;` was the separator | `the_separator_argument_replaces_the_comma` | joint-collapse | Bites D1 and D1'. `robot_state_to_csv` passes `separator` to `push_line` at two call sites (`conversions.rs:100` for the header line, `:106` for the value line) and both land in the same `text`. D1 hard-codes `','` at the header call: 1 test fails, panicking at `csv_conversions.rs:91`. D1' hard-codes `','` at the value call instead: this same site panics at `:91` again (plus `the_separator_a_line_was_written_with_reads_it_back` at `:119`). Two different defective sites, one identical failure here, no attribution — that is the collapse. |
| `crates/moveit-state/tests/csv_conversions.rs:92` | `assert!(text.contains(';'), ...)` — the separator that was asked for actually appears | `the_separator_argument_replaces_the_comma` | joint-collapse | Bite D2 makes `push_line`'s single `out.push(separator)` emit `'|'`: `:91` passes (no comma leaked) and this site is the one that panics, at `csv_conversions.rs:92`, so it is not a restatement of `:91`. Its blindness is measured rather than argued: bite D2' takes the separator away from the *header* call only (`'|'` at `conversions.rs:100`), and `the_separator_argument_replaces_the_comma` passes at both `:91` and `:92` while `the_header_line_is_the_only_difference_include_header_makes` fails at `:73`. A `;` anywhere in `text` satisfies this site, so it cannot tell the header line from the value line. |
| `crates/moveit-state/tests/csv_conversions.rs:188` | `assert!(line.contains(&FIRST_POSITION.to_string()), ...)` — the writer emitted all 16 significant digits, not upstream's six (`doc/upstream-bugs.md`, `robot-state-to-stream-default-ostream-precision`) | `a_round_trip_returns_every_position_bit_for_bit` | single-branch | Exactly one expression formats a value: `.map(f64::to_string)` at `conversions.rs:105`. Bite D3 replaces it with `format!("{v:e}")` — chosen because it is *still* an exact round trip, so the preceding `assert_eq!(read.positions(), written.positions())` at `:187` passes and execution reaches this site. 1 test fails, panicking at `csv_conversions.rs:188`. The `{:.6}` bite (C5 in the port's own bite set) is not the evidence here: it fails at `:187` first, and this site never runs. |

## Why `:91` and `:92` are not merged into one assertion

They fail under disjoint mutations — D1/D1' reach `:91` with `:92` never
evaluated, D2 reaches `:92` with `:91` passing — so collapsing them into a
single `assert_eq!(text, expected_text)` would trade two independently
falsifiable claims for one that also pins the field count and the trailing
separator, which
`the_ungrouped_line_has_no_trailing_separator` and
`the_header_line_is_the_only_difference_include_header_makes` already own.
The collapse recorded above is between the header and value *call sites*,
not between these two assertions.
