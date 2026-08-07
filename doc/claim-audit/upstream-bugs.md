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
| `cost-source-nan-blind-compare` | EXPIRED-citation | CONFIRMED | Upstream cited `collision_detection/collision_common.hpp:128-140`; `operator<`'s real span (verified `cat -n`) is `128-141` — the range stopped one line short of the function's own closing brace, cited range excluded it. Port's own doc comment (`common.rs:126`) already had the correct `128-141`; only this document's copy was stale. Mechanism re-verified against the actual `Ord` impl (`common.rs:155-171`): the `c1`/`c2` volume-weighted-cost compare, then bare `cost` compare, then `aabb_min` tie-break chain matches upstream's `operator<` exactly, with `f64::total_cmp` substituted for upstream's bare `<`/`>` at each of the 3 tie-break steps — the deviation claim (upstream is `NaN`-blind, the port is not) holds. | `145c2e4` |
| `fcl-distance-sentinel-survives-zero-contacts` | CONFIRMED | CONFIRMED | All four upstream citations verify exactly against `collision_common.cpp` (`cat -n`): `:613` `dist_result.distance = fcl_result.min_distance;`, `:636` `if (distance <= 0 && cdata->req->enable_signed_distance)`, `:647-648` `std::size_t contacts = fcl::collide(...);` / `if (contacts > 0)`, `:663` `dist_result.distance = -contact.penetration_depth;` — no drift, no wrong-anchor. Confirmed the `if` at `:648` has no `else` by reading through to its closing brace at `:691`; the sentinel written at `:613` and zeroed-but-unreplaced when `contacts == 0` reaches `cdata->res->minimum_distance` via the plain `<` at `:694`, exactly as described. Port citation `parry.rs:2457` (`let distance_value = if request.enable_signed_distance { contact.dist } else { contact.dist.max(0.0) };`) confirmed as the site with no sentinel substitution on any path. Both regression tests (`no_sentinel_escapes_at_the_tie`, `the_tie_is_decided_below_one_ulp`) confirmed present in `tests/exact_tangency_boundary.rs`. | (none — no drift found) |
| `all-valid-distance-robot-hides-base-overload` | EXPIRED-citation | CONFIRMED | Core name-hiding citations all verify exactly (`cat -n`): `collision_env_allvalid.hpp:63-64` (`distanceRobot` declarations) and `:73` (`distanceSelf` declaration); `collision_env_allvalid.cpp:108-112` (3-arg `distanceRobot` override, `res.collision = false` only) and `:114-123` (both convenience `distanceRobot` overloads returning `0.0`); `collision_env.hpp:167,179` (base `distanceSelf` convenience overloads) and `:202,220` (base `distanceRobot` convenience overloads — confirmed non-virtual, different signature from the derived declarations, exactly the shape that produces hiding-not-overriding). Found and fixed one citation-attribution defect: the sentence "...`std::numeric_limits<double>::max()` (`collision_detection/collision_common.hpp:263`, reset at `:286`)" attributed the `max()` claim to `:263`, but `:263` is `double distance;` (the field declaration + its own doc comment at `:262`, quoted uncited by the *next* sentence) — the actual `distance = std::numeric_limits<double>::max();` assignment is at `:286`, confirmed by reading `DistanceResultsData::clear()` directly. Reordered the citation so `:286` anchors the `max()` claim and `:262-263` is named separately for the field's own doc comment. Mechanism re-verified structurally: `DistanceResult::minimum_distance` is a plain `DistanceResultsData` field (`collision_detection/collision_common.hpp:313-331`), default-constructed via `clear()`, so the base-pointer path genuinely lands on `f64::MAX`. Port (`all_valid.rs:162-169`) matches: `distance_robot` returns `DistanceResult::default()`, i.e. `f64::MAX`, exactly the "right one" outcome the entry's own analysis reaches. Both named regression tests confirmed: `distance_queries_report_maximum_clearance` (`all_valid.rs:243`), `distance_to_collision_through_the_null_backend_is_maximum_clearance` (`crates/moveit-scene/tests/all_valid_selection.rs:130`, out-of-fence, read-only). Cross-checked upstream `collision_env_allvalid.cpp:142-146` (`distanceSelf`, `res.collision = false` only) against the port's own doc comment at `all_valid.rs:144` — exact. | `d0874b4` |
| `distance-callback-max-contact-depth` | EXPIRED-citation | CONFIRMED | Upstream FCL max-selection citations exact (`cat -n` on `collision_common.cpp`): `:646` `coll_req.num_max_contacts = 200;`, `:650-659` the `max_dist`/`max_index` loop with `if (contact.penetration_depth > max_dist)` at `:655`, `:662-663` `getContact(max_index)` / `dist_result.distance = -contact.penetration_depth;`. Found two citation-attribution defects in this entry's own `PORTING-PLAN.md` self-citations, both introduced by the prior round's `c5e1294` and never caught: "§218.4's per-robot table (`!PORTING-PLAN.md:17026`)" pointed at the "다른 쌍 (pair-flip)" bullet, not the table (actual table `!PORTING-PLAN.md:17008-17012`, panda row `!PORTING-PLAN.md:17010`); "The `27,384x` figure §218.4 (`!PORTING-PLAN.md:16973`)" pointed at a blank line one line before the §218.4 heading, not the line stating `27,384배` (actual `!PORTING-PLAN.md:16979-16980`). Fixed both. Mechanism re-verified: port's `accumulate_distance` (`parry.rs:2536-2593`) calls `query::contact` once per part-pair via `parry3d_f64` — no per-triangle contact set, no max-selection loop, confirming "no set to take a maximum over" as described. Both named regression tests confirmed present: `depth_is_invariant_to_floor_width` and `depth_never_exceeds_the_links_own_diameter` in `crates/moveit-collision/tests/penetration_depth_scale_invariance.rs`. Cross-checked `!PORTING-PLAN.md:807` (UNMET verdict row) and §229.3's heading (`!PORTING-PLAN.md:19113`, names the same `27,384배` figure) — both exact. | `34f630d` |
| `pr2-collision-test-asserts-unwritten-result` | pending | pending | | |

## Correction, this round

The round-2 brief that opened this document assumed no prior audit of
`doc/upstream-bugs.md` existed. It did: p10-samplers re-verified all 44
entries the document held at the time (three questions per entry —
upstream citation, port citation, absence-or-expiry claim), fixed seven
findings (`6d7a8d1`, `c5e1294`, `c98c1d3`, `9fa8ca1`, `ef382c1`,
`c2727ea`, `d38fe3c`), and merged as `9746ca0` — already an ancestor of
this round's starting commit. The user corrected the brief mid-round
(this document's entries 1-3 above were already re-opened before the
correction landed and stand; `c5e1294`'s citations in entry 4 above were
found broken by this round's own re-check and are the fourth commit
above). The corrected task: (1) re-verify `kdl-path-circle-nan-scale-rot`
against `third_party/orocos_kinematics_dynamics/` — its Port field
(`crates/moveit-planners-pilz/src/path_circle.rs`) is out of fence, so
this was read-only; (2) audit the two entries added to the document
since `9746ca0`'s merge; (3) read the bare `:NNN` continuation citations
p10-samplers bounds-checked but did not re-open for content, entry by
entry.

### `kdl-path-circle-nan-scale-rot` — read-only, out of fence

Out of fence: Port is `crates/moveit-planners-pilz/src/path_circle.rs`.
Read `path_circle.cpp:91-96` and `path_line.cpp:67-83` directly from
`/home/stevek/work/moveit-rs/third_party/orocos_kinematics_dynamics/orocos_kdl/src/`
(the absolute path — `third_party/` is gitignored and untracked, so it
never reaches a `caucus` worktree, but is present and populated in the
primary checkout this path names). Both citations verify exactly:
`path_circle.cpp:91-96` is the `else` arm computing
`scalerot = oalpha/pathlength` with `pathlength = dist`, and
`path_line.cpp:67-83` is the three-way guard whose third arm is commented
"// both were zero" — the asymmetry the entry describes. No `UNVERIFIED`
marker exists in the
committed entry; it already reads "Evidence: verified verbatim in the
checkout above" and its Upstream field already states the untracked/
present-at-primary-checkout framing correctly. A repo-wide grep for the
false premise ("gitignored" + "absent"/"unverif") found no other
instance of it live in any committed document — every other `third_party/`
reference already uses the correct untracked-but-present framing
(`doc/claim-audit/moveit-srdf.md:37`, `doc/claim-audit/tools-ci-gates.md:168-179`,
`doc/assertion-discrimination-ledger-p1-fixtures.md:1084`, which records
the identical false premise closed in a different entry by an earlier
merger). The premise the correction described does not appear in any
file in this tree; whatever recorded it was a panel report, never
committed. No edit made — nothing to fix, and the crate Port field is
out of fence regardless.

## Entries added since `9746ca0` (2) — audited this round, no Port to fence

Neither cites a `crates/` file at all (`Port: none`), so neither could
be in- or out-of-fence by this document's own rule; both were explicitly
assigned this round. Identified via
`diff <(git show ceb496a:doc/upstream-bugs.md | slugs) <(git show 21bc24b:doc/upstream-bugs.md | slugs)`
— `ceb496a` is the merge-base of p10-samplers' branch and the `main` it
merged into (44 entries on both sides); `21bc24b` is `main` immediately
before that merge (46 entries) — the two panels that added these ran
concurrently with p10-samplers' audit, which is why the audit's own
"44" was already stale by the time it merged.

| slug | citation verdict | mechanism verdict | evidence | commit |
|---|---|---|---|---|
| `collision-callback-logs-contact-stored-when-dropped` | EXPIRED-citation | CONFIRMED | Upstream `collision_common.cpp` read in full (`cat -n`, lines 205-270). The `else if (cdata->req_->verbose)` arm citation (`:261-268`) is exact, ending on its own closing brace. Found and fixed one citation-attribution defect: the entry attributed "the `want_contact_count > 0` test" to `:254`, but `:254` is `cdata->res_->contact_count++;` — the actual `if (want_contact_count > 0)` test is two lines above, at `:250`. The budget range `:212-215`, the storing arm `:257-258` and the discarding arm's literal `:263-267` all verify exactly. The 17-site `verbose` read-site list (`:110,122,138,151,176,188,232,255,261,320,368,402,414,514,530,544,557`) matches `rg -n 'req_?->verbose'` exactly, no fewer no more; spot-checked three (`:232`, `:261`, `:557`) as single-statement `RCLCPP_*` blocks, as claimed. Port claim ("no crate in this workspace reads `CollisionRequest::verbose`") re-verified: `rg 'tracing::\|log::(debug\|info\|warn\|error)' crates` returns nothing. §239.3 heading exists. | `d3d0ba7` |
| `check-collision-unpadded-discards-its-own-request-copy` | EXPIRED-citation | CONFIRMED | Upstream `planning_scene.cpp:450-511` read in full (`cat -n`). Found and fixed two citation-range defects, both the same shape as this round's opening `getCostSources` example (a range claiming to *be* a function span that stops short of the closing brace) but the second is worse: the entry's citation for the *second* broken overload, `501-508` (written without the leading colon here so this row is not itself read as a live citation), excludes not only the closing brace (actual `:510`) but the broken statement itself — `checkCollision(req, res, robot_state, acm);`, the very line that is this entry's defect, sits at `:509`, outside the cited range. Fixed to `:501-510` — and that was still wrong at the other end, caught at merge by `tools/ci/verify-upstream-citations.sh`: `:501` is the blank line before the definition, which opens at `:502`. Now `:502-510`. The same start-side slip was in `:456-463` (definition opens at `:457`), fixed with it. Fixing a range's end does not check its start, and reading the body to confirm the defect is inside the range does not either. The fourth "correct sibling" citation, `:491-499`, also stopped one line short of its closing brace (actual `:500`); fixed to `:491-500`. The first broken overload's citation (`:456-463`) and the other three correct siblings (`:465-471`, `:473-480`, `:482-489`) all verify exactly, ending on their own closing braces. The deprecation-notice citation (`planning_scene.hpp:379`) and both port-side cross-references (`crates/moveit-scene/src/scene.rs:348-352`, D4 decision; `:566-576`, the two-flags/D4 redesign discussion) verify exactly — read-only, `moveit-scene` is out of fence, no edit made there. | `3c88a3f` |

## Out-of-fence entries (39) — not re-opened this round

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
`set-motion-plan-request-time-guard-polarity`.

39 slugs: the 46-row Index table, minus the 5 in-fence slugs above and
the 2 `Port: none` slugs re-opened in the "added since `9746ca0`"
section above (both computed programmatically, not hand-counted).

## Summary

In progress. Updated per entry as each is re-opened; see commits above.
