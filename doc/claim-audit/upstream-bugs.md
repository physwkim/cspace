# Claim audit — doc/upstream-bugs.md

`doc/upstream-bugs.md` holds 46 entries; `check-upstream-bugs-index.sh`
confirms the document is internally consistent (slug/status/order/contract
fields agree between the Index table and each entry), but that gate never
re-opens a single upstream line — an entry can be internally consistent
and still cite the wrong lines, or cite the right lines for the wrong
mechanism. This audit re-opens each entry's cited upstream lines against
`/home/stevek/work/moveit2` at the pinned `e017c91ee12984393a28ba246075c65f69cde3bf`
(verified clean and at that exact SHA before any entry below was opened)
and asks, per §3a of this round's brief: is the cited line the statement
the entry describes, does a range that claims to *be* a function's span
really end on its closing brace, and does the described defect actually
follow from what the citation shows.

Scope: the FENCE for this worktree is `crates/moveit-model/` and
`crates/moveit-collision/`. An entry is in scope here when its **Port:**
citation names a file under one of those two crates. Grepping
`^\*\*Port:\*\*` for `moveit-model`/`moveit-collision` across all 46
entries found 5 in scope, all `moveit-collision` (no entry cites
`moveit-model`). The other 41 are enumerated by slug below, **not
re-opened this round** — recorded that way per this round's brief, not
folded into an aggregate "the rest are fine" row.

Verdict values: `CONFIRMED` (citation and mechanism both hold),
`EXPIRED-citation` (the cited line/range is wrong but the defect claim
holds once you read the right lines), `EXPIRED-mechanism` (the citation
is fine but the defect as described does not actually follow).

## In-fence entries (5) — personally re-opened this round

| slug | citation verdict | mechanism verdict | evidence | commit |
|---|---|---|---|---|
| `cost-source-nan-blind-compare` | EXPIRED-citation | CONFIRMED | Upstream cited `collision_common.hpp:128-140`; `operator<`'s real span (verified `cat -n`) is `128-141` — the range stopped one line short of the function's own closing brace, cited range excluded it. Port's own doc comment (`common.rs:126`) already had the correct `128-141`; only this document's copy was stale. Mechanism re-verified against the actual `Ord` impl (`common.rs:155-171`): the `c1`/`c2` volume-weighted-cost compare, then bare `cost` compare, then `aabb_min` tie-break chain matches upstream's `operator<` exactly, with `f64::total_cmp` substituted for upstream's bare `<`/`>` at each of the 3 tie-break steps — the deviation claim (upstream is `NaN`-blind, the port is not) holds. | `145c2e4` |
| `fcl-distance-sentinel-survives-zero-contacts` | pending | pending | | |
| `all-valid-distance-robot-hides-base-overload` | pending | pending | | |
| `distance-callback-max-contact-depth` | pending | pending | | |
| `pr2-collision-test-asserts-unwritten-result` | pending | pending | | |

## Out-of-fence entries (41) — not re-opened this round

Citing a file outside `crates/moveit-model/`/`crates/moveit-collision/`;
re-opening these is another panel's or another round's work, not fixed or
verified here.

`chomp-iteration-double-increment`, `distance-field-contact-index-oob`,
`attached-body-count-check`, `totg-velocity-step-function`,
`kdl-path-circle-nan-scale-rot`, `inv-twice-resolution-int-truncation`,
`max-distance-sq-narrowing`, `get-shortest-solution-empty-deref`,
`multivariate-gaussian-cholesky-unchecked`,
`mimic-master-outside-group-dropped`,
`check-consistency-index-space-mismatch`,
`acceleration-bounds-per-joint-advance`, `do-smoothing-length-check-operand`,
`get-max-payload-index-space`, `totg-timing-zero-velocity-division`,
`polyline-filter-waypoints-stale-index`,
`polyline-header-redeclares-lin-exceptions`,
`plan-components-builder-const-build-mutates`,
`extract-blend-radii-empty-list-underflow`,
`ik-cache-read-trusts-file-header`,
`get-best-approximate-static-dummy-stale`,
`update-cache-capacity-as-size-limit`,
`save-cache-empty-path-guard-falls-through`,
`cached-ik-accumulate-return-discarded`, `ik-cache-map-first-update-dropped`,
`set-from-ik-zero-timeout-is-not-single-attempt`,
`validate-and-improve-interval-percentage-discarded`,
`aggregated-limits-drops-rejected-joint-silently`,
`check-position-bounds-multidof-adjacent-members`,
`count-samples-per-second-returns-a-ratio`,
`stream-to-robot-state-missing-variable-falls-through`,
`robot-state-to-stream-group-lookup-unchecked`,
`stream-to-robot-state-bypasses-dirty-flags`,
`robot-state-to-stream-default-ostream-precision`,
`set-from-ik-leaves-a-rejected-candidate-in-the-state`,
`set-from-ik-subgroups-timeout-truncated-to-whole-seconds`,
`pilz-detailed-response-pushes-null-trajectory`,
`to-string-truncates-to-six-significant-digits`,
`set-motion-plan-request-time-guard-polarity`,
`collision-callback-logs-contact-stored-when-dropped`,
`check-collision-unpadded-discards-its-own-request-copy`.

41 slugs, computed by set-subtracting the 5 in-fence slugs above from the
full 46-row Index table programmatically (not hand-counted), against the
same 46 `check-upstream-bugs-index.sh` confirms.

## Summary

In progress. Updated per entry as each is re-opened; see commits above.
