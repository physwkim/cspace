# Assertion-discrimination ledger — interpolation (`moveit-kinematics`, `moveit-state`)

The one site the sweep's scanner emits for the round that ported
`RobotState::interpolate`'s three overloads (`crates/moveit-state`,
`tests/interpolate_state.rs`) and pinned `CartesianInterpolator::
interpolate_pose`'s translation blend as an f64 program
(`crates/moveit-kinematics/src/cartesian_interpolator.rs`, `mod tests`).

A separate file rather than rows appended to
`doc/assertion-discrimination-ledger-cached-ik.md`, whose title and
preamble scope it to `moveit-kinematics`' cached-IK round, or to
`doc/assertion-discrimination-ledger-p10-cartesian.md`, which is another
panel's file — the same reason `discover_ledgers` globs rather than
hardcodes, and the reason `p10-conversions` gives for its own split.

`crates/moveit-state/tests/interpolate_state.rs` contributes no row. It
had one, `assert!(error.to_string().contains("NaN or inf"))` at `:80`,
until writing this ledger surfaced that the sibling tests for the same
guard —  `infinite_t_is_refused_by_the_whole_model_form` and
`nan_t_is_refused_by_the_group_form` — asserted nothing at all about
*which* error came back, and `interpolate_group` has a second error
branch (`Error::UnknownName` from `joint_model_group`). All three now
route through one `assert_is_the_bounds_exception` helper comparing
`to_string()` whole against `"Interpolation parameter is NaN or inf."`,
which is `Error::Other`'s `Display` verbatim. That is strictly stronger
than the substring test it replaces and, being an `assert_eq!` against an
exact value, is not a coarse-assertion kind — the site is gone because the
assertion got tighter, not because it was rephrased around the scanner.

Verdict legend (same as the other ledgers): **discriminating** (proven to
distinguish ≥2 sibling outcomes), **single-branch** (exactly one
construction site reaches this assertion — nothing to discriminate),
**joint-collapse** (≥2 real sibling sites are asserted at once here and it
cannot say which was wrong), **not-this-family** (excluded by census §9).

## Method

The row's evidence is a mutation applied to this worktree, run with
`cargo nextest run -p moveit-kinematics` (98 tests, 98 passing at
baseline), and reverted, with the revert confirmed by re-reading the
mutated hunk. It is not justified by reading the code alone.

## `cartesian_interpolator.rs` (1)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-kinematics/src/cartesian_interpolator.rs:1059` | scanner row `via:new:test:new(x, y, z)` — `Vector3::new` in `mod tests`' `pose` helper, propagated here because the scanner matches the helper's *name* | `the_last_waypoint_is_exactly_the_target_pose`, `the_first_waypoint_is_exactly_the_start_pose`, `the_interior_matches_upstreams_two_term_blend_bitwise` | not-this-family | bite P3, below |

**bite P3 (run this round, reverted).** The assertion the scanner
propagates to this line is `Percentage::new`'s range guard at `:375`
(itself tagged `helper_body` and outside the corpus). Replacing that
guard's condition with `false`, so *every* call of `Percentage::new`
panics, leaves the three tests that reach `:1059` green and fails 16
others (98 run, 82 passed, 16 failed). A site whose tests are unaffected
by making the guard unconditional does not reach the guard. The same
family, and the same reason, as `p10-cartesian`'s section 1 rows for
`:264`, `:432` and `:625`; `:1059` differs from `:264`/`:432` only in
being a real call rather than a `fn new` signature line, which is why it
gets a bite instead of a read.

`nalgebra`'s `Vector3::new` is a `#[inline]` componentwise constructor —
there is no assertion inside it to discriminate anything against, and the
three tests' own discriminating power is carried by the exact `assert_eq!`
comparisons on `Isometry3` coordinates, whose fixture constants `NEAR` and
`FAR` were chosen by sweep precisely so the two candidate spellings of the
blend disagree on them (`the_interior_matches_upstreams_two_term_blend_
bitwise` asserts `differed > 0` so the fixture proves that itself).
