# Assertion-discrimination ledger — cached IK (`moveit-kinematics`)

Produced by p10-cachedik while porting upstream's `IKCache` persistence
(`ik_cache/format.rs`, `IkCache::save`/`load`, `CachedIkSolver::save_cache`/
`from_cache_file`). It classifies the one coarse-assertion site that round
added; it does **not** re-open the crate's existing rows, which live in
`doc/assertion-discrimination-ledger-p1-fixtures.md` (§ "moveit-kinematics
(9 sites)", `chain.rs` and `tests/ik_fk_roundtrip.rs`) and
`doc/assertion-discrimination-ledger-p1-robotmodel.md` (§ "moveit-kinematics
(3)"). A separate file rather than a row appended to either of those for
the reason `reconcile-assertion-ledgers.py`'s `discover_ledgers` already
records: ledgers are globbed, panels own their own file, and appending to
another panel's file across worktrees only manufactures merge conflicts.

The round's other new tests were written against exact values — an exact
`Err` message compared with `assert_eq!`, `let Err(_) = … else { panic!() }`
for the one case whose message is `serde_json`'s and not this crate's — so
they are not coarse-assertion sites at all and need no rows. The single
exception below is `assert_eq!(result, None)`, kept in that shape because
rephrasing an honest assertion purely to slip past the scanner would be
gaming the gate rather than accounting to it.

Verdict legend (same as the other ledgers): **discriminating** (proven to
distinguish ≥2 sibling outcomes, by bite, isolating fixture design, or
structural single-code argument), **single-branch** (exactly one possible
construction site reaches this assertion — nothing to discriminate),
**joint-collapse** (≥2 real sibling guards fire together at this
assertion's fixture and it cannot say which), **not-this-family** (excluded
by one of census §9's three clauses).

Evidence legend: **bite** (a fresh reachability mutation run this round,
reverted after confirming), **read** (a structural argument from the source,
stated so it can be checked).

## `cached_solver.rs` (1)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-kinematics/src/cached_solver.rs:419` | `CachedIkSolver::solve_with_options`'s returned `Option<Vec<f64>>` when neither attempt solved | `a_solve_that_fails_leaves_the_cache_empty` | single-branch | read + bite, below |

**read.** `solve_with_options` has exactly one `None`-producing expression
reachable at its `return`: the value of the local `solution`, which is
whatever `self.inner.solve_with_options` last returned. `None` on the
`KinematicsSolver` trait is payload-free — it means "no solution" and
carries no cause — so there are no sibling outcomes for `assert_eq!(result,
None)` to be blind to. Both of the two syntactic assignments to `solution`
produce that same one variant, and the returned `None` is reachable only in
the state where both were `None`.

**bite (run this round, reverted).** The fact this test *does* carry beyond
"it failed" — that both the cached seed and the caller's own seed were
tried before giving up — is asserted by the adjacent `*calls.borrow()`
comparison, not by the `None` site. Confirmed by mutating the fallback's
condition from `if solution.is_none()` to `if solution.is_none() &&
seed.is_empty()`, so the caller's seed is never retried: the `None` site
still passes (the cached-seed attempt failed on its own), while the `calls`
assertion in the same test fails, along with
`cache_miss_falls_back_to_the_callers_own_seed`,
`cache_hit_short_circuits_without_trying_the_callers_own_seed` and
`a_saved_cache_seeds_the_next_solver_built_from_it`. Reverted; `git diff
--stat` clean for that hunk afterwards.

The complementary mutation — caching on failure under the caller's seed
(`None => self.cache.update(&nearest, target, seed)`) — fails this test at
its *third* assertion (the second solve starts from the poisoned entry
`[3.0]` instead of the empty-cache dummy) and no other test in the crate,
which is what makes the test a guard on "a failed solve is not inserted"
rather than a restatement of the `None`.
