# Claim audit — tools/ci gate hygiene

Not a per-crate C++ parity audit (those live in `doc/claim-audit/moveit-*.md`
next to it). This file tracks a different kind of claim: a `tools/ci/*.sh`
or `ros/*.sh` gate's claim that it checked something. Same convention as
`doc/claim-audit/moveit-planners-sbp.md`'s `## §195 anchor sweep` section --
a mechanical anchor plus an honest statement of where it stops, so a future
round can re-run the search instead of re-reading every script by hand.

## §119 anchor sweep (round 18) — inverse-comparison / vacuous-pass

The brief that opened round 18 asked for this round's own two anchors
(below) to be recorded as re-runnable commands, and for round 17's "19-site
inverse vacuous pass" sweep's anchor to be recorded alongside them. That
sweep's mechanical anchor, as given:

**Mechanical anchor:** `rg -n '\bdiff\b|-eq |-ne |assert_eq|!= |== ' tools/ci/*.sh tools/ci/*.pl ros/*.sh`

I could not locate round 17's original 19-site classification in git
history to carry forward verbatim: round 17's three merged commits
(`8538ec1`, `51a0544`, `6281b43`) are the `moveit-scene`/`moveit-metrics`
narrowing-sweep and branch-enumeration doc work, not a `tools/ci` sweep, and
no other commit anywhere in this repo's history matches `vacuous`,
`invert`, or `backward` under `tools/ci` or `ros`. Rather than restate a
number I cannot reproduce, I re-ran the anchor above fresh, today, against
the tree as it stands after this round's own fixes.

**Result today: 75 raw hits, not 19** — the tree has grown across the
intervening rounds (new gates, `verify-fixture-replay.sh`'s Python
comparisons, this round's own `check-no-dead-status-capture.sh`). Read
every hit: each one gates the correct direction. None silently reports a
pass on the failure case it names — every `-eq 0`/`-ne 0`/`!=`/`==` found
either fails when a count is zero/empty (`gate-lib.sh`'s `require_nonempty`
and its callers), fails when two computed sets disagree
(`verify-fixture-replay.sh`'s `actual_by_id.keys() != expected_by_id.keys()`,
`check-license-matches-upstream.sh`'s declared-vs-derived license
comparison), or is an intentional, loudly-printed `SKIP`
(`verify-mpr-vs-epa.sh`'s `tag != "v2.1"`).

**Where the mechanical anchor stops:** it only sees a literal comparison
operator on the line it matches. It cannot see a comparison that is never
reached at all because the value it would compare was never produced --
the producer failed silently upstream and the loop that would have called
the comparison ran zero times. That gap is exactly Anchors A and B below,
neither of which contains a `diff`/`-eq`/`-ne`/`assert_eq`/`!=`/`==` token
on the line that actually loses the diagnostic, so this anchor's 0 hits
above is not evidence they don't exist -- it is evidence this anchor
cannot see them. Both are closed this round; use *their* anchors, not this
one, to check whether either regresses.

## §119 anchor sweep (round 18) — dead handler / silent-skip families

Round 18's own sweep, following on from the coordinator's finding in
`804a697` (fixed in `48ef7ce`): "any command whose failure is meant to be
handled, but which `set -e` aborts before the handler runs."

### Anchor A — bare assignment aborts before its own handler

A plain `x="$(cmd)"` (or `x="$(cmd1 && cmd2)"`, or a pipe ending in a
component with a legitimate nonzero-on-no-match exit, e.g. `grep`) aborts
the whole script *at the assignment*, under `set -e`/`pipefail`, before any
downstream code written to handle exactly that case (`if [[ -z "$x" ]]`, a
Python-side "could not parse" branch) ever runs.

**Mechanical anchor (narrow slice, this round's new gate):**
`tools/ci/check-no-dead-status-capture.sh` -- catches only the sub-shape
where the very next non-blank, non-comment line after the unguarded close
is a bare `var=$?`. Verified this round to fire on a synthetic
reproduction of the exact `48ef7ce` shape and to report zero hits on the
current tree.

**Mechanical anchor (this round's hand sweep, no gate -- needs semantic
reading, see the gate's own header for why):** re-read every
`="\$\(` assignment in `tools/ci/*.sh`, `tools/moveit-oracle/*.sh`,
`ros/*.sh`, `tools/mpr-vs-epa/*.sh` for a component whose legitimate
nonzero exit (a `grep` no-match, a second `&&`-chained command) is not
neutralized by a trailing `|| true` or `|| var=$?` before a downstream
handler that assumes it can run.

| site | classification | disposition |
|---|---|---|
| `ros/verify-ros-interop.sh`, `unit_summary=`/`actual_tests=` (two sites, same line pattern) | same defect -- `grep`'s no-match exit (1) propagated through `pipefail`, aborting before the `-z` handler below each | fixed, `ea9e421` |
| `tools/ci/verify-mpr-vs-epa.sh`, `epa_line=` | same defect -- `&&`-chained `cargo run && grep` made a merely-absent "EPA depth=" line abort the assignment before python's `one()` handler, written for exactly that case, ran | fixed, `ea9e421` |
| `ros/verify-ros-interop.sh`, `test_output=`/`test_status=$?` | same defect, already fixed by the coordinator before this round started | fixed, `48ef7ce` (not mine) |
| `tools/moveit-oracle/run-oracle.sh`, `tools/ci/count-narrowing-sweep.sh`, `tools/ci/count-public-declarations.sh`, `tools/moveit-oracle/build.sh`, `ros/build.sh`, `ros/entrypoint.sh`, `tools/moveit-oracle/entrypoint.sh`, `tools/mpr-vs-epa/build.sh` | distinct -- each risky-looking assignment already carries its own `|| true` neutralized by an explicit downstream comparison, or has no comparable pattern at all | no action |

### Anchor B — process substitution swallows a producer's failure

`done < <(producer)` / `mapfile -t arr < <(producer)` discards the
producer's exit status entirely (unlike a plain assignment, this does
*not* abort under `set -e`), so a failing producer yields zero
lines/rows, the consuming loop runs zero iterations, and -- where there is
no `require_nonempty`-style guard, or the guard's role is inverted because
emptiness is the gate's own *pass* condition -- the gate reports OK having
examined nothing.

**Mechanical anchor:** `rg -n '< <\(|mapfile .* < <\(' tools/ci/*.sh tools/moveit-oracle/*.sh ros/*.sh tools/mpr-vs-epa/*.sh`

**Where the mechanical anchor stops:** it finds every process-substitution
read: it cannot tell, by itself, whether the read is already followed by a
`require_nonempty`-equivalent guard on the *right* condition (many are; see
`gate-lib.sh`'s own header for the "emptiness is the pass condition" vs.
"emptiness is the fail condition" distinction that decides which sites
need fixing). Each hit needs that one-line judgment call by hand.

| site | classification | disposition |
|---|---|---|
| `tools/ci/check-lints-not-silently-dropped.sh`, `done < <(workspace_keys "$group")` | same defect -- a `workspace_keys` failure would silently skip every key check for that group | fixed, `dd86041` |
| `tools/ci/check-license-matches-upstream.sh`, `mapfile -t sources < <(git ls-files ...)` | same defect -- a broken `git ls-files` (the `.git`-less-export scenario `check-audit-scripts-not-copied.sh` already hit once) is indistinguishable from "this crate genuinely has no `.rs` files" | fixed, `dd86041` |
| `tools/ci/check-workspace-dep-inheritance.sh`, `done < <(printf ... \| awk ... "$manifest")` | same defect -- an `awk` parse failure on `$manifest` drives zero inline-dependency findings for that manifest | fixed, `dd86041` |
| `tools/ci/check-audit-scripts-not-copied.sh` | same defect, already fixed in an earlier round | fixed, `c8150ed` (not mine) |
| `tools/ci/check-license-matches-upstream.sh:44`, `mapfile -t manifests < <(git ls-files ...)` | distinct -- immediately followed by `require_nonempty "${#manifests[@]}"`, whose *fail* condition is the same empty result a producer failure would also yield | no action |
| `tools/ci/check-license-matches-upstream.sh:102`, `mapfile -t distinct < <(printf '%s\n' "${ids[@]}" \| sort -u)` | distinct -- `${ids[@]}` is an in-memory array already built earlier in the same loop iteration, not an external command whose failure this line could newly introduce | no action |
| `tools/ci/check-lints-not-silently-dropped.sh:58`, `mapfile -t manifests < <(git ls-files ...)` | distinct -- immediately followed by `require_nonempty` | no action |
| `tools/ci/check-workspace-dep-inheritance.sh:29,35`, two `mapfile ... < <(git ls-files ...)` | distinct -- both immediately followed by `require_nonempty` | no action |
| `tools/ci/check-no-dead-status-capture.sh:46` (this round's own new gate), `mapfile -t scripts < <(git ls-files ...)` | distinct -- immediately followed by `require_nonempty` | no action |
| `tools/ci/verify-vendored-fixture-tests.sh:35`, `mapfile -t TESTS < <(rg ... \| rg ...)` | distinct -- a fully-failed `rg` pipeline drives both `TESTS` and the independently-computed `attr_count` to 0, which does not satisfy the `-ne` mismatch check but is caught one line later by the script's own `${#TESTS[@]} -eq 0` guard; verified by reading, not merely assumed, since the mismatch check alone would not have caught it | no action |
| `tools/ci/verify-fixture-replay.sh:218`, `read -r urdf_name srdf_name < <(python3 -c '...')` | distinct, different failure mode than the rest of this family -- under this script's own `set -euo pipefail`, a producer that exits with zero bytes of stdout makes `read` itself return nonzero, which aborts the whole script immediately (verified: `bash -c 'set -euo pipefail; read -r a b < <(true); echo reached'` never prints "reached", exits 1). That is a loud, whole-script abort with the producer's own stderr (e.g. a Python traceback) still visible, not a silent zero-iteration pass -- the defect this anchor otherwise hunts for | no action |

`check-fixture-format.sh`, `verify-upstream-license-provenance.sh`, and
`verify-oracle-sweep.sh` do not match the anchor above at all in the
current tree -- confirmed by re-running it, not by memory.

Re-run both anchors together before trusting either family stayed closed:

```
tools/ci/check-no-dead-status-capture.sh
rg -n '< <\(|mapfile .* < <\(' tools/ci/*.sh tools/moveit-oracle/*.sh ros/*.sh tools/mpr-vs-epa/*.sh
```

## §243 `verify-all.sh` full sweep and the coverage boundary of every member

Round brief: run every `verify-*.sh` including the `PHASE3_SWEEP=1` member,
report each on its own line; then enumerate what each `check-*.sh`/`verify-*.sh`
gate actually parses and what a failure shape would slip through it, by
reading each script rather than trusting its header's own account of itself.

### §243.1 Full sweep, from a worktree fast-forwarded to this round's tip

| script | result |
|---|---|
| `verify-clean-checkout.sh` | PASS |
| `verify-constraint-sweep.sh` | PASS (2056/2056 constraint combinations) |
| `verify-continuous-reseed-wrap.sh` | PASS |
| `verify-fixture-provenance.sh` | PASS -- see §243.2 for why this needed a worktree-local fix first |
| `verify-fixture-replay.sh` | PASS |
| `verify-mpr-vs-epa.sh` | PASS |
| `verify-oracle-sweep.sh` | PASS |
| `verify-orphan-enumeration.sh` | PASS (0 orphans) |
| `verify-phase2-state-sweep.sh` | PASS, all 5 robots (panda/prbt/fanuc/dual_arm_panda/pr2) |
| `verify-phase3-collision-sweep.sh` (`PHASE3_SWEEP=1`) | **UNMET, confirmed on 2 of 5 robots; run did not finish -- see §243.3** |
| `verify-phase7-benchmark.sh` | not run -- opt-in, out of this round's scope, not silently counted as pass |
| `verify-port-coverage.sh` | not independently re-run this round; not silently counted as pass |
| `verify-private-doc-links.sh` | PASS (own documented gap: `#[cfg(test)]` unreachable, see §243.2 row) |
| `verify-ros-interop.sh` | not independently re-run this round; not silently counted as pass |
| `verify-sampler-self-validation.sh` | PASS |
| `verify-upstream-license-provenance.sh` | PASS -- correction below |
| `verify-vendored-fixture-tests.sh` | PASS |

`verify-phase7-benchmark.sh`, `verify-port-coverage.sh`, and
`verify-ros-interop.sh` were not run this round (the first is opt-in and
outside this round's ask; the other two need `$MOVEIT2_SRC` / a live ROS 2
graph this task did not ask for). Naming them here rather than omitting them
is the point of "a member that skipped is not a member that passed" --
silence would read as thirteen tested when eighteen exist.

**Correction to an earlier round's report:** `verify-upstream-license-provenance.sh`
was previously reported as exiting 1 because `third_party/` is "absent from
this machine." Re-checked from this fast-forwarded worktree: `third_party/`
is gitignored and lives at the shared session repo root
(`~/work/moveit-rs/third_party/`), invisible from a per-worktree checkout by
construction -- the same worktree-invisibility trap that produced a false
"missing" claim once already this session (see
[[untracked-dirs-are-invisible-from-worktrees]]). With
`THIRD_PARTY_SRC=/home/stevek/work/moveit-rs/third_party` set (the env
override this script already supports) it passes cleanly. `verify-fixture-provenance.sh`
and `verify-vendored-fixture-tests.sh` hardcode a relative
`third_party/moveit_resources` with no env override, so those needed a
local, gitignored `third_party -> .../third_party` symlink in this worktree
instead -- confirmed safe: `verify-clean-checkout.sh` clones into a fresh
temp dir via `git clone --local`, which does not follow an untracked,
gitignored symlink. The earlier "missing from this machine" report was a
worktree artifact, not a regression.

### §243.2 What each gate parses, and a concrete failure it would not catch

Read every `check-*.sh` and `verify-*.sh` (32 scripts including this
round's own new one), not sampled. Rows already fully covered by an
existing header's own honest account of itself are marked so; the rest are
this round's own reading.

| script | parses | escapes it | concrete failure |
|---|---|---|---|
| `check-audit-scripts-not-copied.sh` | files under an `audit/`-named dir whose basename starts with `count` | a copy placed outside `audit/`, or renamed off `count*` | `count-relative-eq.pl` copied to `crates/foo/scripts/relative_eq.pl` re-diverges exactly like the incident this gate exists for, undetected |
| `check-dep-direction.sh` | `cargo metadata --no-deps` from the root `[workspace]` | any crate outside that workspace | `ros/moveit-ros` has its own `[workspace]` (D2/§129) and is structurally invisible to this check, same boundary `verify-ros-interop.sh`'s header names as already having bitten once |
| `check-fixture-format.sh` | filesystem glob of fixture files (not `git ls-files`) | -- (glob-vs-git direction only widens the check, not narrows it) | none found; noted for completeness |
| `check-license-matches-upstream.sh` | SPDX header on every tracked `.rs` file | non-`.rs` tracked files (`.py`, `.pl` audit tools, `build.rs`... `build.rs` is `.rs` so covered; genuinely non-`.rs` sources are not) | a vendored/adapted Python audit tool carrying a different upstream license has no SPDX convention here for this check to even examine |
| `check-lints-not-silently-dropped.sh` | *presence* of each workspace lint key in an opted-out crate's `[lints]` table -- own header: "requirement is presence, not value" | a restated key set to a **weaker** value | `[lints.rust] missing_docs = "warn"` (workspace says `"deny"`) passes this gate; that crate's missing-docs lint is now unenforced and nothing says so |
| `check-no-lint-suppression.sh` | `^\s*#!?\[\s*(allow\|expect)\s*\(` -- an attribute anchored at line start | `#[cfg_attr(cond, allow(lint))]` -- the suppression is nested inside `cfg_attr`, never the line's own head token | verified this round: piping a synthetic `#[cfg_attr(not(test), allow(dead_code))]` through the exact pattern produces zero matches (would pass as clean); confirmed 0 current occurrences of this shape anywhere in `crates/tools/ros` -- a live, currently-unexploited gap, not an active violation |
| `check-no-dead-status-capture.sh` | one narrow sub-shape (`var=$(cmd)` immediately followed by a bare `var=$?`) | own header lists 5 sibling blind spots explicitly (pipefail-abort family, `&&`-chain variant, process-substitution family, `set -e` disabled in a function/subshell, `$?` read >1 line after assignment) | already fully documented by the script itself |
| `check-phase-status.sh` | the one canonical `### 완료 조건 현황표` table's row citations against `##`/`###` headings, re-scoped on ANY heading level | a phase claim made in prose *outside* that one table (an inline "§5 Phase 3 UNMET" sentence elsewhere in the file) | such a sentence can drift from the table silently; this gate only holds the table itself consistent, not every prose restatement of a verdict |
| `check-pilz-tolerance-overrides.sh` | presence of a `#[should_panic]` companion test per `*_TOLERANCE` constant, by name pairing only | a companion test that exists and panics, but for the wrong reason / measures the wrong quantity | a `#[should_panic]` test whose assertion doesn't actually exercise the tolerance it's named for still satisfies this gate -- the [[delete-what-a-test-tests]] shape, at the constant-pairing level rather than the whole-test level |
| `check-porting-plan-sections.sh` | `##`-`####` heading `§NNN` uniqueness in `PORTING-PLAN.md`; `§243` placeholder absence tree-wide | a `§NNN` **citation** elsewhere in the tree pointing at a number that was reassigned away | see §243.3 -- this is this round's second finding, deliberately left open rather than closed with a naive gate |
| `check-serde-float-roundtrip.sh` | `[workspace.dependencies].serde_json` carries `float_roundtrip` | a member crate pinning its own `serde_json` version directly, bypassing the workspace table | not independently re-verified this round whether that specific escape is reachable; flagged, not asserted |
| `check-test-doc-links.sh` (this round, new) | bracket-style intra-doc links in `crates/*/tests/*.rs` | `#[cfg(test)] mod tests` blocks embedded in `src/*.rs`; rustdoc disambiguators/glob-imports/trait-inheritance/prelude; nested-group `use` imports; `as`-aliases | own header documents all four; see §243.4 |
| `check-upstream-bugs-index.sh` | `doc/upstream-bugs.md`'s `## Index` table vs its `###` entries | not independently re-audited for a new escape this round | header-level trust only |
| `check-workspace-dep-inheritance.sh` | inter-crate (`crates/*`/`tools/*`) deps route through `[workspace.dependencies]` | the same drift shape for an *external* dependency | `check-serde-float-roundtrip.sh` covers exactly one hand-picked instance (`serde_json`); no script generalizes the rule to "every external dep resolves through the workspace table" |
| `verify-all.sh` | each member's own exit code | a member that fails to `exit` nonzero on its own real failure | the aggregator trusts each member completely; it cannot see inside one |
| `verify-clean-checkout.sh` | steps extracted from `ci.yml`'s `rust` job | a step living in a different `ci.yml` job block this script does not parse | not independently re-verified this round which job blocks it extracts from |
| `verify-fixture-provenance.sh` / `verify-upstream-license-provenance.sh` / `verify-vendored-fixture-tests.sh` | `third_party/` (gitignored, worktree-relative) | a per-worktree checkout without an env override or local symlink | see §243.1 correction -- reproduced and fixed this round, not merely re-asserted |
| `verify-oracle-sweep.sh` / `verify-constraint-sweep.sh` / `verify-phase2-state-sweep.sh` / `verify-phase3-collision-sweep.sh` | N random or boundary states per robot, against the live oracle | a divergence on a state none of these draws | `verify-phase2-state-sweep.sh`'s own header already makes this explicit: boundary-value sweeps and random sweeps can each miss what the other finds |
| `verify-private-doc-links.sh` | `cargo doc --document-private-items` reachable items | `#[cfg(test)] mod tests` inside `src/*.rs` (dev-deps don't link into a doc build) | own header already states this; confirmed this round `cargo doc --help` has no `--test`/`--tests` target flag at all, so this is not presently closable via rustdoc |
| `verify-ros-interop.sh` | thin caller into `ros/moveit-ros`'s own separate gate | own header's "what this does NOT check" section (no live ROS 2 graph) | already documented by the script itself |
| `verify-sampler-self-validation.sh` | one named `#[ignore]`d test | a *different* test later marked `#[ignore]` for the same "nothing invokes it" reason | it is a pointer to one test, not a scan for the pattern across the tree |

Every row not listed above (`verify-continuous-reseed-wrap.sh`,
`verify-fixture-replay.sh`, `verify-mpr-vs-epa.sh`, `verify-orphan-enumeration.sh`,
`verify-phase7-benchmark.sh`, `verify-port-coverage.sh`) was read this round
and its own header's account of its scope matched what its body actually
does; no new escape found in the time available. That is not the same claim
as "no escape exists" -- it is the honest boundary of this round's reading.

### §243.3 `verify-phase3-collision-sweep.sh`: UNMET on the two robots that finished, run incomplete

Run with `PHASE3_SWEEP=1 THIRD_PARTY_SRC=... sg docker -c tools/ci/verify-phase3-collision-sweep.sh`
(10000 cases, seed 1, the script's own default). The script's own header
records a measured 4815s (80m15s) total wall clock across all 5 robots from
its first full run; this round's run did not complete within the time
available and was left running unattended rather than forced to a false
result:

| robot | `collision: bool` disagreements | `distance: f64` disagreements | verdict |
|---|---|---|---|
| panda | 0/10000 | 9543/10000 (95.430%) | **UNMET** (distance clause) |
| prbt | 6854/10000 (68.540%) | 10000/10000 (100.000%) | **UNMET** (both clauses) |
| fanuc | not finished | not finished | not measured this round |
| dual_arm_panda | not measured | not measured | not measured this round |
| pr2 | not measured | not measured | not measured this round |

Both `panda` and `prbt` already fail the script's own pass condition
(`bool_bad == 0 && dist_bad == 0 && errored == 0` on every robot) regardless
of the remaining three -- §5 Phase 3's completion condition is UNMET on the
tree as it stands today, independent of anything this round did or could fix.
This is collision/distance-field code (`crates/moveit-collision`,
`crates/moveit-distance-field`), outside this panel's fence
(`moveit-scene`, `moveit-metrics` only) -- reported, not touched. The
background run was left in place rather than killed; its log is at
`/tmp/claude-1000/-home-stevek-work-moveit-rs--caucus-worktrees-H8F9NKVWVW-p1-fixtures-920dace3-1/b9b3e817-af83-4ec2-8008-75705b47845f/scratchpad/phase3-sweep.log`
on this machine, but a fresh run is the reliable way to reproduce this: the
script's sampling is seeded and reproducible per its own header.

### §243.4 The costliest gap found and closed: `tools/ci/check-test-doc-links.sh`

`verify-private-doc-links.sh` closes the "reachable via `cargo doc
--document-private-items`" half of the doc-link coverage question; its own
comment already states the `#[cfg(test)]` half stays open because a doc
build never links dev-dependencies. What neither that script nor any other
gate covers: `crates/*/tests/*.rs` integration test files are not reached by
`cargo doc` *at all* -- confirmed this round, `cargo doc --help` lists
`--lib`/`--bins`/`--bin`/`--examples`/`--example` as its only target
selectors, no `--test`/`--tests` exists. 337 bracket-style intra-doc links
(`` [`Ident`] ``) across this workspace's test files at the time this was
written were checked by nothing: not `cargo doc`, not `cargo clippy` (link
resolution is a rustdoc lint), not `cargo test` (a bracket link is not a
doctest).

`tools/ci/check-test-doc-links.sh` closes this without invoking rustdoc: a
narrow, four-tier same-crate/imported-crate/fully-qualified-crate/local
resolver over `git ls-files`-tracked `.rs` text, deliberately narrower than
what rustdoc itself resolves (documented in the script's own header) so
that what it cannot check reads as a skip, not a pass.

**Discrimination, run this round:** the checker's first real run against the
unmutated tree failed on `crates/moveit-planners-pilz/tests/pilz_trajectory_lin_parity.rs:361`,
`` [`moveit_error::Error::Code`] ``. Investigated rather than dismissed as
noise: `Error::Code(MoveItErrorCode)` is a real variant
(`crates/moveit-error/src/lib.rs:102`), occurring exactly once in
`moveit-error`'s entire tracked corpus -- the declaration line itself,
referenced only from this one cross-crate doc comment. The checker's
resolver had two tiers (a `fn/struct/enum/.../mod NAME` definition regex,
and a "bare identifier occurs >= 2 times" fallback for what that regex
cannot see) and neither one matches an enum variant's own declaration line
with only one occurrence in-crate. Root cause fixed at source: a third tier
(`VARIANT_RE_TEMPLATE`, an identifier at line-start followed by `(`, `{`, or
`,`) added to `resolved_in()`. Re-run clean: 370 checked, 0 failures.
Discrimination proven by planting `Error::CodeXyzzy` at the same site --
checker fails, reports the exact site and target crate, exit 1 -- then
reverting; `git diff --stat` on that file is empty.

### §243.5 A gap found, deliberately not closed: `check-porting-plan-sections.sh`'s stale cross-references

Its own header names the incident this round was asked to look for: a
`§226 -> §227` renumber at merge left four `§226` references in
`moveit-planners-pilz/src/lib.rs` pointing at a different panel's section.
Read the script in full: `all_ids` (every numbered heading in
`PORTING-PLAN.md`) is computed, but never compared against a `§NNN`
citation anywhere else in the tree. The gate only prevents the
*collision* (two sections claiming one number); a citation surviving a
renumber unchanged is not checked by anything.

A same-quality gate for this is not the small addition it looks like.
Built and ran a citation-vs-`all_ids` cross-check this round (investigative
only, not committed) against every tracked `.md`/`.rs` file: 4 raw hits,
and all 4 are false positives under the naive form of this check --

- `crates/moveit-geometry/src/bodies.rs:557,1065` cite `§4.4.1` of
  *Ericson, Real-Time Collision Detection*, a textbook, unrelated to this
  repo's numbering entirely;
- `ros/moveit-ros/doc/message-mapping.md:888` cites `§17.5` of that
  document's **own** independent local numbering scheme, not
  `PORTING-PLAN.md`'s;
- `doc/claim-audit/moveit-kinematics.md:13` cites `§177.1`, a real and
  currently-valid citation -- `PORTING-PLAN.md:14058` has
  `**§177.1 두 번째 사실 ...**` as a bold-text pseudo-subsection, which
  `check-porting-plan-sections.sh`'s own heading-only regex
  (`^(#{2,4}) `) never captures into `all_ids` in the first place.

So a gate here would need to also parse bold pseudo-subsections as valid
targets, and distinguish a `PORTING-PLAN.md`-referring `§NNN` from an
external-literature or another-document's-own-numbering use of the same `§`
character -- built naively, it misfires on all 4 of today's legitimate
citations, which is a checker that "stops being read"
(`check-pilz-tolerance-overrides.sh`'s own words for exactly this failure
mode). Left open for a future round with this scoping already worked out,
rather than shipped half-right this one.

**Merge note (this section is closed, not open).** `tools/ci/check-section-references.sh`
landed as `bb77aa8` while this round was still running, and it is the gate
this subsection scoped: the external-literature case is an allowlist keyed on
a line substring (`tools/ci/section-reference-external.json`, one entry, the
Ericson textbook), `message-mapping.md`'s own `§17.5` resolves against that
file's own headings, and the `§177.1` pseudo-subsections were promoted to
real `### §177.N` headings rather than taught to the parser. It reports
`2895 '§N' references across 208 tracked files`. The count in this section's
title is also stale as of the merge: `tools/ci` now holds 16 `check-*`
members and 19 `verify-*.sh`, not the 32 total measured here.
