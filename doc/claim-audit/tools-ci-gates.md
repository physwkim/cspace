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

- `crates/moveit-geometry/src/bodies.rs:557,1065` cite
  *Ericson, Real-Time Collision Detection* §4.4.1 -- a textbook, unrelated
  to this repo's numbering entirely;
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

### §243.6 A third gate family: `git ls-files` counts an unmerged file once per stage

Found at merge, not by a gate. `check-citation-drift.py` printed
`3742 .rs citations across 72 tracked .md files` mid-merge and
`2165 across 62` on the same tree once the merge was committed — every
still-conflicted path had been read two or three times, and each of its
citations checked, reported and counted that many times. During an
unresolved merge `git ls-files` emits one row per index stage (1 = base,
2 = ours, 3 = theirs), and only `--deduplicate` (git ≥ 2.31; 2.43 here)
collapses them.

The inflation is not the dangerous part — a FAIL printed three times is
still a FAIL. What matters is that the totals these gates print are their
own evidence of coverage, and a run that triple-counts is indistinguishable
from a corpus that grew. The same file's `OK` line is what a later round
quotes as "2,160 citations checked".

`f720642` swept this anchor across eight scripts and ten sites earlier in
the round. `check-citation-drift.py` arrived afterwards, via the p3-acm
merge, and `check-test-doc-links.sh:78,81` were added by §243.4 in the same
window — so all three postdate the sweep that would have caught them. Fixed
here at all three sites. `check-fixture-format.sh` stays classified distinct
for the reason its own header gives: it globs the filesystem deliberately,
so that `git archive` exports remain checkable.

| site | classification | disposition |
|---|---|---|
| `tools/ci/check-citation-drift.py:74`, `git ls-files -z` | same defect — the corpus is every tracked `.md`, so a conflicted document is parsed once per stage | fixed, this round |
| `tools/ci/check-test-doc-links.sh:78`, `git ls-files -- 'crates/*/tests/*.rs'` | same defect — a conflicted test file's links are checked and counted per stage | fixed, this round |
| `tools/ci/check-test-doc-links.sh:81`, `git ls-files -- 'crates/*/*.rs' 'crates/*/**/*.rs'` | same defect — inflates the link-target namespace, which can only mask a dangling link, never create one | fixed, this round |
| `tools/ci/check-fixture-format.sh` | distinct — filesystem glob by design, so `git archive` exports stay checkable | no action |

## Silent removal at merge — the family `check-porting-plan-sections.sh` could not see

A gate that checks a set for *collisions* says nothing about *absence*. That
was the whole of `check-porting-plan-sections.sh` until this round: two
sections claiming one number failed, a section that had stopped existing did
not. The two are the same parallel-append shape seen from opposite sides, and
the second one is quieter: a number that is gone leaves no artefact to trip
over, and `git diff --stat` reports `1 file changed`.

### The incident, replayed from git

`8f9db1f` removed §250 from its own branch, correctly. `c1debc6` had taken the
number the merge assigned by splicing `main[j:]`, and that slice runs to
end-of-file, so it had carried main's §250 onto a branch that never wrote it.
Meanwhile main kept extending §250. Merging that branch gave git a one-sided
deletion of a block the other side had edited, and it took the deletion.

Reproduced without re-running the merge:

```
git merge-tree --write-tree 10a9c13 8f9db1f     # -> tree fdb8ba6, base c1debc6
git show fdb8ba6:PORTING-PLAN.md | rg -c '^## §250'   # -> 0
```

§250 spans lines 22685-22867 of `10a9c13:PORTING-PLAN.md`, 183 lines. In that
raw auto-merge, **165 of them are gone with no conflict marker at all**; the
18-line tail (22850-22867), which abuts §251, is the only part that reached
the conflict block. A reviewer who read the conflict and resolved it would
have shipped the other 165 without ever seeing them. The merge as committed
(`20652c6`) does carry §250 -- it was caught by hand, which is the point:
nothing mechanical would have.

### Two designs, measured over the 2391 commits reachable at 53cce21

Continuity of the number sequence is the obvious rule and it does not survive
measurement. 222 and 223 are unassigned on main today (written without the
section sigil throughout this subsection: `check-section-references.sh` reads
`§N` as a citation, and a citation to an unassigned number dangles by
definition). Declaring exactly those two and failing on any other gap:

| design | fires on | what it is blind to |
|---|---|---|
| absence since the parent | 2 of 2391 commits — `8f9db1f` (250) and `3a8727c` (216, renumbered to §218), both genuine and both declared | nothing at `##` granularity |
| continuity + declared gaps | 99 of 2391 commits, in 14 distinct gap shapes, 12 of them on the first-parent line of main itself | removal of the highest number, which creates no gap |

The false-failure rate alone would have settled it -- a gap is the *normal*
transient state while parallel branches take numbers out of order, so the
declared-gap list is an exception list that has to be edited mid-round, in a
file every branch touches. But the blindness is the disqualifying half.
Deleting §253 from today's tree leaves the gap set at exactly the two declared
numbers, so continuity passes. History has 234 distinct maxima, meaning every
section is the highest one during the round that writes it: each one spends
its most exposed period in the state continuity cannot see. §250 was catchable
by continuity only through the accident that §251 had already landed.

### The rule, and how a legitimate removal declares itself

Every commit reachable from HEAD must still carry every top-level number each
of its parents carried. Not HEAD against its parents: CI runs once per push,
at the tip, so a removal in any earlier commit of that push is invisible to a
one-step rule -- and `8f9db1f` was exactly such a commit.

This layer is about commits only, and says nothing whatever about an
uncommitted tree; the subsection after next is the half that does.

A deliberate removal says so in a commit message:

```
Plan-section-removed: <number> from <parent-sha> -- <why>
```

It names the *parent's* sha because a commit cannot know its own before it
exists, and because on a merge the removal is relative to one specific parent
(§250 was present in one and absent in the other). It may live in any
reachable commit, not only the one doing the removal -- which is what lets
`8f9db1f` and `3a8727c`, both older than this check, be declared by the commit
that adds it, and what a merger needs, since by then the offending commit is
already in the graph. Two-sided like the unresolvable list in
`upstream-citation-exemptions.json`: a declaration matching no actual removal
fails too, so this cannot accumulate permissions that outlive what they
permitted.

Environment: the check needs real history, so `ci.yml` now checks out with
`fetch-depth: 0`. A shallow checkout is a hard failure, not a skip, because
`git rev-list --parents -1 HEAD` in a depth-1 clone prints the commit with no
parents at all -- the comparison would find nothing and report OK. Measured
cost 2.5 s: 2392 commits over 2833 parent edges, 418 distinct revisions of the
plan, each parsed once from a single `git grep '^[#`]'` (2.1 s against 8.9 s
for decoding and splitting the blobs, verified identical on all 418).

### The second layer: the working tree, which no commit covers

The rule above is about commits, and that is not the state the §250 loss was
ever reviewable in. `git merge` stops with a conflict, leaves the resolution
in the working tree, and makes no commit; the gates get run there, on a tree
that has already lost the section, and a commit-versus-parent rule has nothing
to compare. `59cc402` closes that by applying the same rule once more, HEAD's
section set against the file this script already read. Re-derived here rather
than taken from that commit's message:

```
# synthetic: drop one section, do not commit it
python3 - <<'EOF'
import pathlib
p = pathlib.Path("PORTING-PLAN.md"); l = p.read_text(encoding="utf-8").split("\n")
a = next(i for i, t in enumerate(l) if t.startswith("## §253 "))
b = next(i for i, t in enumerate(l) if i > a and t.startswith("## "))
p.write_text("\n".join(l[:a] + l[b:]), encoding="utf-8")
EOF
git show 03e6390:tools/ci/check-porting-plan-sections.sh > /tmp/pre.sh   # the gate before 59cc402
```

| tree state | gate before `59cc402` | gate at `59cc402` |
|---|---|---|
| §253 (interior) dropped, uncommitted | **OK**, `254 top-level sections` silently printed as `253`, headings `1137` as `1132` | FAIL, one line, naming §253 and HEAD |
| §254 (today's highest) dropped, uncommitted | **OK**, headings printed as `1130` | FAIL, one line |
| the same §253 drop committed | FAIL (the history walk) | FAIL |
| committed with `Plan-section-removed: 253 from <parent>` | — | OK, both layers clear |

The OK lines are the point: the counts they print are what a later round quotes
as evidence of coverage, and they moved without anything saying so.

The historical merge replayed in that same state is sharper than the synthetic
drop, and corrects an easy overstatement — the old gate did not pass on it, it
just said nothing about §250:

```
git checkout -b tmp-replay 10a9c13 && git merge --no-commit --no-ff 8f9db1f
git status --porcelain PORTING-PLAN.md          # -> UU PORTING-PLAN.md, no commit
rg -c '^## §250 ' PORTING-PLAN.md               # -> 0 in the tree
git show HEAD:PORTING-PLAN.md | rg -c '^## §250 '   # -> 1 at HEAD
```

Four conflict markers, the whole of §250's heading outside them. The old gate
reported six failures there and every one of them was about the branch's own
unnumbered placeholders; the loss of §250 produced no line. The new layer adds
exactly one, naming §250 and telling the reader to check the whole file rather
than the conflict block.

No declaration can excuse this layer: a trailer names a parent sha, and an
uncommitted removal has no commit to carry one. That is not a gap -- committing
the removal with its declaration satisfies the history walk *and* makes the
tree equal HEAD, so the set difference is empty. The two layers close on the
one action.

### Mutations

| mutation | result |
|---|---|
| §250 deleted wholesale in a commit | FAIL, naming §250 and the parent, one line |
| §253 or §254 deleted in the working tree only | FAIL, one line, exclusive — OK before `59cc402` |
| the real `10a9c13` × `8f9db1f` merge, left unresolved in the working tree | FAIL naming §250, on top of the six placeholder failures the old gate already gave |
| the same tree, with only the deletion guard neutralised | OK — nothing else in the script catches it |
| the same tree, removal declared against that parent | OK |
| a declaration for a removal that never happened | FAIL, "the declaration outlived what it permitted" |
| a declaration naming a sha no reachable commit has | FAIL |
| a past revision whose plan has no heading line at all | FAIL — a family: the parse-to-nothing message plus the 253 removals it causes, listed to 20 with the remainder counted |
| a past revision with a fence left open after the last heading | FAIL, exclusive |
| the whole gate run in a `--depth=1` clone | FAIL, naming `fetch-depth: 0` |
| duplicate number / the unassigned-number placeholder unfenced in the plan / the same placeholder in another tracked file / unclosed fence / zero `##` sections, each in the working tree | FAIL, one line each — the four pre-existing guards, re-proved after the parse was factored into `scan()` |
| the placeholder inside a fence | OK — still not a placeholder |

(The placeholder token is not spelled out anywhere in this file, because the
scan that finds it now reads every tracked file and this is one -- writing it
as prose would fail the gate this subsection documents. It is no longer
spelled out in `check-porting-plan-sections.sh` either; the subsection below
is why.)

### The placeholder scan's scope, and why the token left the gate

The scan used to read `.md` and `.rs` only. The justifying sentence in the
header was mine and it is false: "sections are cited in documentation and in
doc comments, never in build or CI scripts", against 123 section citations in
33 tracked files that are neither -- 93 in 21 `.sh`, 11 in 4 `.py`, 11 in
`tools/moveit-oracle/src/oracle.cpp`, 3 in 3 `.toml`, 3 in 2 `.json`, 1 in
`tools/mpr-vs-epa/mpr_case104.c`, 1 in `ros/Dockerfile` (re-derived on the
tree after merging `b4638bb`). A placeholder duly went through the gap:
`p11-planningfailed` wrote one into the `ros/verify-ros-interop.sh` comment
that ends "with nothing looking at it." and it reached the trunk in `a746945`
-- line 203 there, and still line 203 after the merge. The other gate did not see it either -- `check-section-references.sh`
does read `.sh`, but its reference pattern requires a digit after the sigil,
so an unassigned placeholder matches nothing there by construction.

The suffix list existed for exactly one real reason: the script spelled the
token out eleven times and would have failed on itself. The file-kind rule was
that self-exclusion in disguise, and the justification was written afterwards.
So the token is now assembled from an escape (`PLACEHOLDER` is `"\u00a7NEW"`, the sigil
written as its code point) and
named in two pieces in the prose; the file's bytes no longer contain it; and
the scan has no exception left. Every path `git ls-files` prints is read,
`PORTING-PLAN.md` excepted only because it is scanned separately, with the
fence rule it needs. Deliberately not a longer suffix list: widening to
`check-section-references.sh`'s own `SCANNED_SUFFIXES` would still have missed
`ros/Dockerfile`, which has no suffix, and the `.c`/`.cpp` citations.

Matching on bytes rather than decoded text closes the other silent skip. 35
tracked files are not valid UTF-8 -- the binary meshes under `fixtures/` -- and
the old loop's `except UnicodeDecodeError: continue` meant a file it could not
read was a file it did not check. Now the only way a path goes unread is an
`OSError`, which is a failure naming the file. The number of files actually
read is on the OK line (739 of 740, the plan being the 740th), and a scan
that read none is a failure of its own.

PRE is the gate at `f0aa8fe`'s parent, POST the gate as it stands. Each
mutation is one placeholder inserted into one file, everything else clean:

| mutation | PRE | POST |
|---|---|---|
| `a746945`'s `ros/verify-ros-interop.sh` with its assigned number (255, written here without the sigil: this branch has no such heading yet) put back to a placeholder — the bytes that reached the trunk | OK | FAIL, `ros/verify-ros-interop.sh:203` |
| a placeholder in `.config/nextest.toml`, `.github/workflows/ci.yml`, `tools/ci/section-reference-external.json`, `tools/ci/check-citation-drift.py` | OK | FAIL, one line each |
| a placeholder in `ros/Dockerfile` (no suffix), `tools/moveit-oracle/src/oracle.cpp`, `tools/mpr-vs-epa/mpr_case104.c` | OK | FAIL, one line each |
| a placeholder in `doc/upstream-bugs.md` and in `crates/moveit-collision/src/lib.rs` — the kinds the old scope already covered | FAIL | FAIL, same line |
| a placeholder inside a fence in `PORTING-PLAN.md`, then the same token outside it | — | OK, then FAIL |
| the scan handed nothing but `PORTING-PLAN.md`, so `scanned == 0` | — | FAIL, "covered one file out of the tree" — the OK line's file count is what makes this visible |
| `ros/Dockerfile` at mode 000 | — | FAIL naming it and the `errno`, where the old loop's `continue` said nothing |
| the tree unmutated | OK | OK, 739 files read |

### Where this stops: sub-section numbers

The rule covers `##` numbers only. Eight parent-pairs in this history lose a
`###`/`####` number while the top-level heading survives, six of them at
merges. All eight are renumbering, not loss, and each was checked by content
rather than by number: `24.5` is today's `§36.5` ("남는 것"), `31.4` is
`§35.4` ("게이트"), `174.1` is `§176.1` ("경계"), and the three separate
branches that each numbered a subsection `226.5` have theirs at `§227.5`,
`§228.5` and `§238.5`. So extending the rule one level down would fire on
renumbering -- a routine merge operation, eight times in this history -- and
would need a second declaration for it. Not built here; a silent removal at
sub-section granularity is currently detected by nothing.

### Does the same family reach the other append-only documents?

`check-porting-plan-sections.sh` closes one document. The shape it closes --
a document several branches append to, where a one-sided deletion merges
without a conflict and nothing downstream misses the deleted block -- is not
specific to `PORTING-PLAN.md`, so the question is which of the other tracked
documents are in it. Answered by deletion, not by reading: for each of the 66
tracked documents (`git ls-files '*.md' 'doc/*.txt'`), delete the last `##`
block a branch would have appended -- or one body line where the file has no
`##` heading -- run every `tools/ci/check-*` plus
`verify-orphan-enumeration.sh`, `verify-port-coverage.sh`,
`verify-declaration-audits.sh` and `verify-upstream-citations.sh`, and record
which of them says the block is gone. Then restore from a byte copy and check
the tree is clean.

Seven of the 66 are seen by something; one has nothing to delete; the other
58 are not seen by anything.

| document | what notices the deletion |
|---|---|
| `PORTING-PLAN.md` | `check-porting-plan-sections.sh` (this round's rule) and `check-section-references.sh`, which resolves the inbound `§N` references |
| `doc/port-coverage.md` | two separate things: `verify-port-coverage.sh` compares its 87 rows against the set `measure-port-coverage.py` computes from the upstream tree, so a dropped row is `MISSING ROW` (measured by dropping one row); and `verify-declaration-audits.sh` resolves `doc/declaration-audit-coverage.md`'s two `doc/port-coverage.md:211` citations, so dropping the block at 211 fails with `past its 209 lines` |
| `doc/declaration-audit-coverage.md` | `verify-declaration-audits.sh`, the same shape against the 158 ported files |
| `doc/assertion-discrimination-ledger-cached-ik.md`, `-p10-interpolation.md`, `-p9-ros.md` | `verify-orphan-enumeration.sh`: the deleted block carried accounting rows, so its scanner sites became orphans |
| `doc/upstream-bugs.md` | `check-upstream-bugs-index.sh` and `verify-upstream-citations.sh` -- but only for this deletion's shape; see below |
| `doc/assertion-discrimination-orphans.txt` | nothing to delete: the live orphan set is empty today, so the file is header-only. It is still protected -- adding one fabricated entry gives `FAIL ... is stale: 0 site(s)`, which is `--verify` comparing the file against a set it recomputes |
| the other 58 | nothing |

The protection is always one of two things, and neither is a property of the
document: the row set is recomputed from outside the document (`port-coverage`,
`declaration-audit-coverage`, the orphan snapshot, the ledger rows that
account for scanner sites), or something else in the tree cites the block by
name (`PORTING-PLAN.md`'s `§N`). A document whose blocks are prose nobody
recomputes and nobody cites -- every `doc/claim-audit/*.md`, every
`doc/assertion-discrimination-ledger-*.md` block that is not an accounting
row, `doc/assertion-discrimination-census.md`, the per-crate `doc/*.md` under
`crates/` -- has neither.

`doc/upstream-bugs.md` deserves its own line, because the gate it has is the
one that looks most like it should cover this and does not.
`check-upstream-bugs-index.sh` pairs the `## Index` table against the `###`
entries. Deleting the last `##` block -- `## Closed (round p9-ros,
2026-08-05)`, 992 lines -- takes the 15 `###` entries inside it and leaves
their Index rows, so the pairing fails; that is the row above. But a branch
appends an entry as an Index row *and* a body block, so that is what a
one-sided deletion takes: delete both halves of
`distance-field-contact-index-oob` and every gate above passes. The pairing is a consistency check between two halves of the
document; it cannot see an entry that left as a whole.

#### Which of them the same two layers would fit

The layers themselves are document-independent -- child-vs-parent over the
reachable history, working-tree-vs-HEAD, and a commit trailer to declare a
deliberate removal. What is not document-independent is the key: the rule is
only usable if a block's identity survives ordinary editing. Measured the same
way the `##`-number rule was, by replaying the key sets over every commit that
touched each document:

| document | commits touching it | merges where both parents differ | commits that remove a key |
|---|---|---|---|
| `doc/upstream-bugs.md`, keyed on the `### \`slug\`` | 118 | 15 | **0** |
| `doc/upstream-bugs.md`, keyed on `##` heading text | 118 | 15 | 0 |
| `doc/assertion-discrimination-census.md`, `##` text | 24 | 0 | 0 |
| `doc/claim-audit/moveit-sampling.md`, `##` text | 5 | 0 | 0 |
| `doc/claim-audit/moveit-model.md`, `##` text | 5 | 0 | 0 |
| `doc/claim-audit/tools-ci-gates.md`, `##` text | 7 | 1 | 1 |
| `doc/assertion-discrimination-ledger-p10-samplers.md`, `##` text | 5 | 0 | 0 |
| `doc/assertion-discrimination-ledger-pilz.md`, `##` text | 14 | 0 | **5** |
| `PORTING-PLAN.md`, `##` text | 424 | 42 | 48 |

`doc/upstream-bugs.md` is the one that fits, and it fits because it already
carries the rule that makes a key stable: "Append anywhere; never rename a
slug once it is cited." Zero slug removals in 118 commits, across 15 merges
where both parents had touched the file -- the same measurement that picked
design (A) for `PORTING-PLAN.md`, with the same answer. The 15 two-sided
merges are the exposure: that is 15 chances for git to have taken a one-sided
deletion, and nothing would have said so.

Heading text is not a usable key anywhere else, and the two failing rows say
why in two different ways. `ledger-pilz.md` removes five keys because its
headings carry a running count -- `## Summary (12, then 13, now 26)` becomes
`## Summary (12, then 13, then 26, now 27)` -- so the heading is edited in
place by design and a heading-keyed rule fires on every update.
`tools-ci-gates.md`'s one removal is the merge that assigned a number to a
section heading that had been written with the unassigned-number placeholder,
which is the merge doing its job. `PORTING-PLAN.md` shows both at 48 commits,
which is exactly why the rule live in `check-porting-plan-sections.sh` keys on
the *number* and treats the placeholder separately, and fires on 2 commits in
2412 rather than 48.

So: the family reaches 58 of the 66 documents, one of them (`doc/upstream-bugs.md`)
has a key stable enough for the two layers today and a measured zero
false-positive rate over its whole history, and the rest need a stable
per-block identity before any such rule can be written -- a slug, or a number,
or anything but the prose of the heading. Neither the rule for
`doc/upstream-bugs.md` nor a key for the other 57 is built here.
