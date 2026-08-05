# Caucus handoff — assertion-discrimination round 2

> **Superseded in part by the session that consumed it (2026-08-05).**
> The handoff was acted on and the following claims in it are now known
> wrong. Corrected in full in
> `doc/assertion-discrimination-round2-brief.md`'s fifth correction;
> repeated here so nobody re-derives from this file alone.
>
> - **§3's 103 and 422 are `-o | wc -l` artefacts.** `rg -o` prints the
>   matched *text*, so a match spanning N lines contributes N lines to
>   `wc -l`. The real figures are **85** and **205**, and the `rg -U -c`
>   commands §3 actually prints agree with a match-start enumeration on
>   every file in the tree.
> - **§3's 42/35 cross-check measures the bare anchor, not
>   `assert!(matches!(`.** `moveit-collision` is 2 under
>   `assert!(matches!(` and 42 under the bare anchor. The cross-check is
>   valid; it just validates the other anchor.
> - **§3's seven-row "never seen" table is short by more than half.**
>   Derived as "files with no anchor2 hit", it misses files anchor2 saw
>   but under-counted *within*. The true set is **39 sites across 17
>   files**. All are dispatched and closed by this session except the
>   `ros/moveit-ros` rows.
> - **A fourth anchor2 blind spot**, found by p1-joints: sites asserting
>   `Some(Body::_)` rather than `Err(Error::_)` are outside anchor2's
>   shape entirely (`moveit-geometry/src/bodies.rs`, 4 sites).
> - **The anchor set is now closed.** `matches!` totals 110 workspace-wide
>   — 85 inside `assert!`, 25 production control flow — and all 139
>   `unwrap_err`/`expect_err` call sites bind or assert their error, so no
>   discarded-error shape exists. The two anchors are the whole family.
>
> §1's "not merged" item is done: `83e3c1c` merged at `f831242`.

Written 2026-08-05 so the caucus session can be torn down and restarted
without losing state. `.caucus/` is gitignored and its worktrees are
disposable; everything a new session needs is either in git or copied
into `doc/` by this handoff.

Preserved alongside this file:

- `doc/assertion-discrimination-round2-brief.md` — verbatim copy of
  `.caucus/briefs/assertion-discrimination-round2.md` as of the fourth
  correction. **This is the governing brief.** Copy it back to
  `.caucus/briefs/` when the new session starts.
- `doc/gate-4a8eaab.md`, `doc/gate-0bf4707.md` — the full green gate
  logs for both merge points.

---

## 1. Tree state

- `main` is at **`0bf4707`**, working tree clean apart from the
  untracked files this handoff adds under `doc/`.
- Old caucus session id: `01KZ33ZXRDAPWCJC5REEQZSC40`. Panel worktrees
  live under `.caucus/worktrees/`, branches under
  `refs/heads/caucus/5REEQZSC40/…`. All but one are fully merged.

Merge history this round, newest first:

| merge | branch | content |
|---|---|---|
| `0bf4707` | p3-shapes | §3a isolating-mutation + D6 call-site evidence, doc-only, geometry/stomp/sampling |
| `bb322d7` | p3-distance-field | `moveit-octomap` garbage-input tests pinned to `Err(DecodeError::UnexpectedEof)` |
| `83f8ea0` | p1-joints | `moveit-srdf` `malformed_xml_is_an_error` discriminated from its sibling root guard |
| `4a8eaab` | p9-ros | 11 blind `matches!` sites in `scene/attached.rs`, `scene/collision_object.rs` |
| `9b3b866` | p3-acm | `CollisionResult::merge`'s dead `(None, None)` arm removed |
| `fcecbca` | p1-robotmodel | `planning_scene_validity`'s `UnknownName` discriminated by kind and name |

### Not merged

**`caucus/5REEQZSC40/p1-fixtures-920dace3-2` → `83e3c1c`**
`test(constraints): assert which name kind new_rejects_unknown_joint hits`.
One-line change in `crates/moveit-constraints/tests/decide.rs:212`,
narrowing `Err(Error::UnknownName { .. })` to
`Err(Error::UnknownName { kind: "joint", .. })`. p1-fixtures ran the
discrimination bite and it **failed before the fix** — neutering the
`joint_model` lookup let `variable_index` (kind `"variable"`) fire the
same variant on the same input, masked by the bare `{ .. }` pattern.
This is a real blind site; merge it.

Their gate for it: `-p moveit-constraints` (99/99) plus full-workspace
clippy and nextest 1646/1646, all 10 `check-*.sh`, and three docker
verify scripts individually. Re-gate after merging regardless.

## 2. Gate status

- **`4a8eaab` — fully green.** Workspace `fmt --check`, `clippy
  --workspace --all-targets -D warnings`, `nextest --workspace` 1646
  run / 1646 passed / 2 skipped, `test --doc --workspace`, `cargo doc
  --workspace --no-deps`, `verify-private-doc-links.sh`, all 10
  `tools/ci/check-*.sh`, `sg docker -c ./tools/ci/verify-all.sh`
  (all 10 verify scripts), and the separate `ros/moveit-ros` docker
  gate on `ros-dev:latest` (`fmt --check`, `clippy --all-targets -D
  warnings`, `cargo test` **154 passed / 0 failed**). Log preserved at
  `doc/gate-4a8eaab.md`.
- **`0bf4707` — fully green**, same set, finished after this handoff was
  first written. `nextest --workspace` 1646 run / 1646 passed / 2
  skipped; all 10 `check-*.sh` OK; `sg docker -c
  ./tools/ci/verify-all.sh` reported all 10 verify scripts passed,
  including the 2 vendored-fixture tests. Log preserved at
  `doc/gate-0bf4707.md`. Its closing line reads `3 dirty` — that is the
  three untracked files this handoff added under `doc/`, nothing else.
  The `ros/moveit-ros` docker gate was **not** re-run at `0bf4707`
  because none of the three merges touch `ros/`; its last full run is
  the green one at `4a8eaab`.

`ros/moveit-ros` is outside the root workspace (D5), so it needs the
docker `ros-dev:latest` gate, never a `-p` scope. `verify-clean-checkout.sh`
builds from **committed** state and `verify-vendored-fixture-tests.sh`
builds from the **working tree**, so HEAD must not move and the tree
must not be dirtied while docker runs.

## 3. The finding that is still open, and why it matters

p1-fixtures reported `moveit-metrics` anchor2 = 3 where I had twice
restated 0. They were right. Checking why exposed that the second
anchor I published,

    matches!\([^,]+,\s*Error::\w+

is blind in three further ways, and together they hide **42 sites** —
more than half again what it finds:

1. **A comma inside the scrutinee.** The anchor wants `Error::` right
   after the *first* comma, so any scrutinee that is itself a call with
   two or more arguments eats that comma. `-U` does not help; `[^,]+`
   cannot cross a comma under any flag.
2. **A fully-qualified path.** `moveit_error::Error::UnknownName` does
   not match a literal `Error::` after the comma.
3. **Crate-local error enums.** The anchor names `Error::` only, so
   `PlanError`, `PipelineError`, `ResponseAdapterError`, `DecodeError`
   and `Diagnostic` were never in scope at all.

Use these instead — enum-agnostic, path-agnostic, comma-tolerant, and
anchored on the *assertion*, which is what the family is about:

```
rg -U -c 'assert!\(\s*matches!\('                          crates/ ros/ tools/
rg -U -c 'assert!\(\s*[\s\S]{0,300}?\.(is_err|is_none)\(\)' crates/ ros/ tools/
```

At `0bf4707` these give **103** and **422**. The 422 is not comparable
to the old first-anchor column — it counts `assert!` wrappers, so it
over-counts where one assert holds several calls; treat it as an upper
bound and enumerate. The 103 against the published anchor2's 61 is the
real hole. Cross-check that held: the new first anchor sums to exactly
42 for `moveit-collision`, reproducing p3-acm's independently verified
figure, and 35 for `moveit-geometry`, reproducing p3-shapes'.

This is the brief's **fourth correction**; it is already written into
`doc/assertion-discrimination-round2-brief.md`. The anchor is now known
to be wrong in six independent ways, five of them found by a panel
rather than by me.

### Sites the round has never seen

`assert!(matches!(` hits in files with **no** anchor2 hit, comments
excluded, enumerated rather than counted, measured at `0bf4707`:

| file | sites | error type | dispatch state |
|---|---|---|---|
| `crates/moveit-planners-pilz/src/trajectory_blender_transition_window.rs` | 7 (`:762 :767 :791 :807 :909 :925 :1150`) | `Error::Code` | dispatched to p1-joints, **no report received** |
| `crates/moveit-planning/src/pipeline.rs` | 4 (`:634 :651 :670 :687`) | `PipelineError::{NoPlanners,Request,Planner,Response}` | dispatched to p9-ros, **no report received** |
| `crates/moveit-metrics/src/lib.rs` | 3 (`:1068 :1072 :1139`) | `Error::UnknownName` | **not dispatched** |
| `crates/moveit-planners-sbp/src/registry.rs` | 2 (`:851 :1330`) | `PlanError::{Sbp,NoGoalSample}` | **not dispatched** |
| `crates/moveit-planning/src/response_adapters/add_time_optimal_parameterization.rs` | 1 (`:337`) | `ResponseAdapterError::Failed` | dispatched to p9-ros, **no report received** |
| `crates/moveit-planners-sbp/src/planning_scene_validity.rs` | 1 (`:420`) | qualified `Error::UnknownName` | **not dispatched** |
| `crates/moveit-state/tests/jacobian.rs` | 1 (`:211`) | qualified `Error::UnknownName` | **not dispatched** |

The seven pilz sites are the highest-yield block: four assert on
`validate_request(&req)` and three on `search_intersection_points(&mut
req)`, all against `Err(Error::Code …)`. `Error::Code` is exactly the
shape the brief's worked example warns about — a shared constant that
cannot tell sibling guards apart.

I asked p1-joints whether `moveit-planners-pilz` had been swept at all
and got no answer before the teardown, so I settled it from history
instead: it **was** swept — `47b3de8`
(`check_start_state`/`check_joint_goal`/`check_cartesian_goal`/`resolve_solver`),
`183b1fa` (`TrajectoryGeneratorPtp::new`, `plan_ptp`) and `1b64709`
(`PathCircle::new`) are all this family, all merged. What no round
reached is `trajectory_blender_transition_window.rs`: `git log` on that
file shows no commit from this family. The crate was swept; the anchor
could not see that one file. That is the cleanest demonstration of the
anchor defect in the tree — a swept crate with an untouched seven-site
block inside it.

p1-joints' own pilz report confirms it from the other side. They
enumerated 30 first-anchor sites and listed the files: `path_circle.rs`
4, `limits.rs` 2, `trajectory_generator_ptp.rs` 5,
`trajectory_generator.rs` 15, `pilz_trajectory_circ_parity.rs` 2,
`pilz_trajectory_lin_parity.rs` 2 — and wrote "confirmed no other `.rs`
files in the crate exist." `trajectory_blender_transition_window.rs`
does exist and holds 7 `assert!(matches!(` sites. The sweeping panel
did not see the file because the anchor did not put it in front of
them.

The four `PipelineError` variants are differently named, so variant
matching *may* discriminate — but only if each variant has exactly one
construction site inside the pipeline. That has not been counted.

## 4. Work dispatched to panels that will be lost on restart

Three prompts went out and no report came back. Re-issue them.

**p3-acm** (owned `moveit-model/`, `moveit-collision/`) — two items:

1. A coverage gap in `moveit-collision` found from outside by p1-joints,
   passed on as a claim to re-derive, not as fact. Their claim:
   `World::remove_shape_from_object` (`crates/moveit-collision/src/world.rs:629-656`)
   has two `?`-early-return `None` sites — `object_mut(id)?` at `:634`
   and `.position(...)?` at `:638` — but only one test,
   `remove_shape_from_object_unknown_is_none`, whose fixture is a bare
   `World::new()` with no object created, so it can only reach guard 1.
   Guard 2 (object exists, shape absent) has zero coverage. They
   contrast `move_shape_in_object`, which has one isolating test per
   guard. One premise in their report is now stale and must not be
   carried forward: they wrote that p3-acm's pass for `moveit-collision`
   "is not on `main`" because `git diff 712dafe -- crates/moveit-collision/`
   was empty. That was true when they wrote it; `a1dc494` merged at
   `9b3b866` afterwards. Ask for: the two-guard count re-derived from source; §3a's
   isolating mutation run in both directions to establish which guard
   the existing test names; and if guard 2 is genuinely untested, a new
   isolating test rather than a widened existing one.
2. Sweep `crates/moveit-model` — nobody has, and it is inside their
   fence. It shows 3 sites in `src/joint/urdf.rs` and 2 in
   `src/robot_model.rs` under the new first anchor.

**p1-joints** — the seven pilz `trajectory_blender_transition_window.rs`
sites. The "was pilz swept at all" question in the original dispatch is
answered above; drop it when re-issuing.

**p9-ros** — the five `moveit-planning` sites
(`PipelineError` ×4, `ResponseAdapterError` ×1), with the
per-variant construction-site count as the thing that decides whether
variant matching discriminates.

Never dispatched: `moveit-metrics` ×3 (p1-fixtures' own crate; they
found these), and `moveit-planners-sbp` ×3 + `moveit-state/tests/jacobian.rs`
×1 (p1-robotmodel's crates).

## 5. Panel roster, for rebuilding the new session

| role | fence, as its own charter states it | round-2 outcome |
|---|---|---|
| p1-joints | Phase-1 `moveit-model` joint layer (charter superseded by the brief's routing table) | `moveit-srdf` fixed and merged; audited `moveit-collision` and `moveit-geometry` from outside without editing them, and found the `remove_shape_from_object` gap |
| p1-robotmodel | Phase-1 `moveit-model` completion (superseded) | six "not applicable" sites re-classified under §3a with executed mutations; `sample_goal` ×2 corrected to *discriminating*, `Gnat::nearest` and `resolve_constraint_sampler` confirmed single-branch |
| p1-fixtures | "You own exactly these crates: `moveit-scene/`, `moveit-metrics/`" | `moveit-scene` 13/13 done earlier; `moveit-constraints` swept under an explicit override, **7** sites not 6, one real fix (`83e3c1c`, unmerged) |
| p3-acm | "You own exactly these crates: `moveit-model/`, `moveit-collision/`" | `moveit-collision` 42 sites (three-way verified), all 7 multi-guard functions closed against real call sites, dead arm removed (`a1dc494`) |
| p3-shapes | "you own `crates/moveit-geometry` and nothing else" | geometry 35 / stomp 7 / sampling 4, self-corrected from the stale 59/19/4 it had been handed; `sample_goal_state` and `extract_seed_trajectory` shown to have **zero** in-tree callers |
| p3-distance-field | "you own `crates/moveit-distance-field` (new) and nothing else" | distance-field 20 + octomap 11 (octomap under an explicit narrow override), `DecodeError` variants pinned |
| p6-totg | "you own `crates/moveit-trajectory` (new) and nothing else" | trajectory 33 + chomp 26 (chomp under an explicit override), 30 sites fixed across 13 merged commits |
| p9-ros | no fence stated | `ros/moveit-ros` 36 sites not the brief's 50, 21 fixed with both bites run in docker; `moveit-planning` was closed at 2 sites and that closure is now **void** (see §3) |

Six of eight charters carry an explicit "and nothing else" fence. Any
cross-crate assignment needs a narrow, on-the-record orchestrator
override with an offer to decline — not a claim that the fence does not
apply. Getting this wrong once already caused p6-totg to revert and
then reapply four commits (`db867b9`, `f4b0ba4`, `67446b2`, `ec34083`);
those stay as they are, no rewrite, no explanatory no-op commit.

## 6. Standing method — do not re-derive it

- **Two-mutation bite standard.** (1) Reachability: no-op the guard, the
  test must FAIL. (2) Discrimination: keep the guard, give it the
  *sibling* branch's message or variant, the test must STILL FAIL. A
  test that passes (1) and fails (2) is a site to fix.
- **§3a, for `Option`-returning calls** where two sibling guards both
  produce a bare `None` and mutation (2) does not exist: neutralize
  guard A alone — A's test must FAIL **and sibling B's test must stay
  GREEN** — then check the mirror. The green half carries the
  information. Run both directions and say so.
- **Count causes, not `None` tokens.** `moveit-kinematics`'
  `search_position_ik` has one literal `None` and three `continue`
  paths reaching it.
- **The honest verdict may be neither.** If two sibling guards collapse
  to an indistinguishable `None` *and an in-tree caller needs to know
  which fired*, the defect is the API, not the test (D6).
- **The count is never the deliverable.** Every published number this
  round was wrong; five of the six anchor defects were found by a panel
  re-measuring rather than reconciling. Tell panels their number wins
  over the brief's and that they must show the command.
- Verify every panel claim independently before relaying it. Doing so
  caught errors in both directions this round.
- Caucus notifications are systematically stale, and panels cite
  pre-rebase SHAs that exist as dangling objects but are not in `main`;
  check the branch tip and confirm byte-identity by content before
  concluding anything was lost or double-applied.
- Use three-dot `git diff main...branch` to see a branch's own changes.
  Two-dot on a behind-branch shows main's newer commits in reverse.

## 7. First actions in the new session

1. Merge `83e3c1c` from `caucus/5REEQZSC40/p1-fixtures-920dace3-2` and
   gate it (`-p moveit-constraints` plus the workspace and docker sets).
2. Copy `doc/assertion-discrimination-round2-brief.md` back to
   `.caucus/briefs/` and re-spawn the panels in §5.
3. Re-issue the three lost dispatches in §4 and dispatch the four never
   sent.
4. Decide whether the four files this handoff adds under `doc/` should
   be committed or left untracked — they were written while a gate was
   in flight and are deliberately uncommitted. Note `.gitignore:6` is
   `*.log`, which is why the gate logs are saved as `.md`.
