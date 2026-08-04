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
