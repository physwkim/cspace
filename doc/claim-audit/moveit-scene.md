# Claim audit — moveit-scene

Prose-citation audit (`Ported from`/`Used by`/backtick `file.cpp:NNN` style
citations, not copyright headers — those are §166/§167, already closed)
against upstream MoveIt2 pinned at `e017c91e`. Scope: every citation in
`crates/moveit-scene/src/*.rs`. Ordered by Rust file, then by line.

Round: p1-fixtures, following `PORTING-PLAN.md` §171 (a citation that opened
correctly but whose derived claim was stale — this audit hunts for more of
that specific failure mode, not just broken links).

Method: a subagent did the first pass over all 41 citation sites in this
crate (40 in `scene.rs`, 1 in `attached_body.rs`); I independently
re-opened and re-verified `scene.rs:663`, `:823`, `:1614`, and the
`pipeline.rs:20`/`:303`/`response.rs:99` sites that belong to
`moveit-planning` (not this file, see below) myself before trusting them.
The ~26 "fine" sites in `scene.rs` lines 240-940 and the ~15 "fine" sites
in `scene.rs` lines 1100-3020 were counted but not individually itemized
by the subagent in its return — this file records what was actually
verified with a citation, not a claim of 100% itemized coverage. Re-running
with per-item output is future work if this list needs to be closed out
fully.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-scene/src/scene.rs:377` | `distanceToCollisionUnpadded` cited at `planning_scene.cpp:461-509` | EXPIRED | opened `planning_scene.cpp:461-509` — that range is `checkCollisionUnpadded`'s body (forwards to `checkCollision`), not `distanceToCollisionUnpadded`. Real function is inline at `planning_scene.hpp:553-609` (4 overloads: `:553,561,588,598`). The claim itself ("each overload only forwards") holds against the real function, but the citation points at the wrong one. | `9e07d32` |
| `crates/moveit-scene/src/scene.rs:1991` | `decoupleParent`'s materialization of `scene_transforms_`, cited `planning_scene.cpp:343-344` | EXPIRED | opened `planning_scene.cpp:343-344` — inside `pushDiffs()`, guarded by `if (scene_transforms_.has_value())`, opposite direction of data flow from what's described. Real `decoupleParent` materialization is `planning_scene.cpp:1260-1264`, guarded by `if (!scene_transforms_.has_value())`. | `679a8f6` |
| `crates/moveit-scene/src/scene.rs:2555` | same claim, second occurrence, same citation `planning_scene.cpp:344` | EXPIRED | same as above — `:344` is inside `pushDiffs()`, not `decoupleParent`. | `679a8f6` |
| `crates/moveit-scene/src/scene.rs:1822-1837` | 5 per-statement citations for the `path_cost_sources` sequence (`cs.insert` at `:2465`, `cs_start.swap` at `:2467-2468`, truncation loop at `:2471-2481`, `removeCostSources` at `:2483`, `removeOverlapping` at `:2484`) | EXPIRED | opened `planning_scene.cpp` around each via `rg -n` (exact line numbers, not manual counting) — every anchor was off by 6-7 lines from the real statement (`cs.insert` actually at `:2472`, `cs_start.swap` at `:2474`, truncation if/else at `:2477-2487`, `removeCostSources` at `:2489`, `removeOverlapping` at `:2490`). The described ordering/behavior is correct against the real lines; only the per-statement anchors were misplaced, consistently by the same offset. | `f19d2e6` |
| `crates/moveit-scene/src/scene.rs:663` | `scene_transforms_` is "reset at `[PlanningScene::diff]` time (`planning_scene.cpp:1264`)" | EXPIRED | opened `planning_scene.cpp:1264` — that's inside `decoupleParent()` and it *sets* the value (`setAllTransforms`), not resets it. `diff()`'s own constructor never touches `scene_transforms_`. Real `.reset()` is `planning_scene.cpp:331`, inside `clearDiffs()`. Verified directly (not just via subagent). | `8abeca0` |
| `crates/moveit-scene/src/scene.rs:823` | `getPlanningFrame` is "always the robot model's frame, since that is the only value `PlanningScene::initialize` ever passes to it (`planning_scene.cpp:192`)" | EXPIRED | `rg -n 'make_shared<SceneTransforms>' moveit_core/planning_scene/src/planning_scene.cpp` inside the moveit2 checkout returns 4 sites (192, 686, 1263, 1333), not just `initialize()`. End conclusion (planning frame is always the robot model's frame) is still true, but the stated exclusivity/mechanism ("the only value ... ever passes") is false — the real invariant is `SceneTransforms`'s constructor always computes the robot-model frame regardless of caller. Verified directly. | `f4fbcc0` |
| `crates/moveit-scene/src/scene.rs:1599` | the `moveit_msgs::Constraints` overload (cited `:2245`/`:2269`) is "a construct-then-delegate wrapper ... build a `KinematicConstraintSet` from the message, then call this exact method" | EXPIRED | opened `planning_scene.cpp:2245` and `:2269` — only `:2245` (which delegates to `:2253`, the overload that actually builds the `KinematicConstraintSet`) matches the claim. `:2269` takes an already-native `KinematicConstraintSet&` and only converts the *state* argument from a message, not the constraint set. | `1e8bfb6` |
| `crates/moveit-scene/src/scene.rs:1614` | `isStateColliding(state, group, verbose)` at `planning_scene.cpp:2217` "overwrites whatever `group_name` a caller-built request carried" | EXPIRED | opened `planning_scene.cpp:2217` (real body at `:2219`) — that function has no `CollisionRequest` parameter at all; `req` is always freshly, locally constructed inside it. There is no caller-built request for `group` to overwrite. Verified directly. | `c9fcf9e` |
| `crates/moveit-scene/src/attached_body.rs` (citation `attached_body.cpp:139-155`) | (not itemized individually by subagent — reported as part of the 40-fine-sites bucket for the batch covering this file) | CONFIRMED | subagent-reported only, not independently re-opened by me | |
| `crates/moveit-scene/src/scene.rs` lines 240-940 (~26 sites, excluding `:377`, `:663`, `:823` above) | various — `planning_scene.cpp`/`world.hpp`/`world.cpp`/`kinematic_constraint.cpp` citations | CONFIRMED (aggregate) | subagent-reported only, not individually re-opened by me except `:663`/`:823` above | |
| `crates/moveit-scene/src/scene.rs` lines 1100-3020 (~15 sites, excluding `:1599`, `:1614`, `:1822-1837`, `:1991`, `:2555` above) | various — `planning_scene.cpp` citations | CONFIRMED (aggregate) | subagent-reported only, not individually re-opened by me | |

## Summary

- 41 sites total (40 `scene.rs` + 1 `attached_body.rs`)
- EXPIRED (category b — citation opens fine, claim is wrong/stale): 8, all above
- EXPIRED (category a — citation doesn't open on the claimed logic): counted
  within the same 8 rows above where the citation target itself was wrong
  (`:377`, `:1991`/`:2555`, `:1822-1837`) — 4 of the 8 rows are "wrong
  target", 4 are "right target, wrong claim" (`:663`, `:823`, `:1599`,
  `:1614`)
- CONFIRMED (aggregate, not individually itemized): 33
- All 8 EXPIRED findings fixed this round (Round 11), one commit each:
  `8abeca0`, `f4fbcc0`, `1e8bfb6`, `c9fcf9e`, `9e07d32`, `679a8f6`
  (covers both `:1991` and `:2555`), `f19d2e6`.

## §172 narrowing sweep (separate exercise, same round)

Not a citation audit, but recorded here for the same compaction-risk
reason §175 flags — see `moveit-metrics.md` for the sibling record.

Round 17: this section's prior figures (a hand sweep reporting 19+1+2=22
hits total) were never actually mechanical — running the convention as a
script (`tools/ci/count-narrowing-sweep.sh`, committed this round) finds
**140** raw hits across the same 8 upstream files (the file list above
this round also corrects an off-by-one: it names 8 files, not 7). The
old count undercounted every file except `attached_body.cpp` and
`attached_body.hpp` (1 and 0, both still correct) and `kinematics_metrics.cpp`
in the sibling crate (`moveit-metrics.md`, still exactly 4 — unaffected).
The gap is largest in `robot_state.cpp` (2 claimed vs. 76 actual): the
old sweep's own claim text ("2 ... `static_cast<std::size_t>(s.rows())`")
describes only that file's 2 `static_cast` hits and silently omitted
every plain declaration in it, most likely because the file is large and
was undercounted rather than actually swept in full — this is why the
convention needed to become a command rather than staying prose.

- Upstream-first direction: `tools/ci/count-narrowing-sweep.sh` against
  the 8 upstream files that provenance this crate's citations —
  `planning_scene/src/planning_scene.cpp`,
  `planning_scene/include/.../planning_scene.hpp`,
  `robot_state/src/robot_state.cpp`,
  `robot_state/include/.../attached_body.hpp`,
  `robot_state/src/attached_body.cpp`,
  `collision_detection/src/world.cpp`,
  `collision_detection/include/.../world.hpp`,
  `kinematic_constraints/src/kinematic_constraint.cpp` — for `int`/
  `unsigned`/`size_t`/`std::size_t`/`long`/`uint32_t`/`int32_t`
  declarations or `static_cast<...>` to one of those types. Per-file raw
  hit counts: `planning_scene.cpp` 24, `planning_scene.hpp` 4,
  `robot_state.cpp` 76 (74 declarations + 2 `static_cast`),
  `attached_body.hpp` 0, `attached_body.cpp` 1, `world.cpp` 10,
  `world.hpp` 4, `kinematic_constraint.cpp` 21. Total **140**.
  - 2 of the 140 are false-positive text matches, not real
    declarations: `world.hpp:138` (`std::size_t size() const` — a
    method whose *return type* is `std::size_t`, not a declaration
    named `size`) and `kinematic_constraint.cpp:947` (`new unsigned
    int[m->triangle_count * 3]` — an array-new expression, not a
    declaration named `int`). The script cannot tell these apart from a
    real declaration by text alone; noted here per the script's own
    documented limitation.
  - A further ~13 are bare function-signature parameters or class
    fields with no initializer at all (e.g. the `getCostSources`
    overloads' `std::size_t max_costs` parameter, repeated across both
    `planning_scene.cpp` and `planning_scene.hpp`; `world.hpp`'s
    `int shape_index`/`Action(int v)`/`int action_`) — nothing to
    narrow into, since there is no initializer.
  - Every remaining hit's initializer (where present) draws from an
    integer-typed source: `.size()`, `.rows()`, `.getVariableCount()`,
    `.getFirstVariableIndex()`/`.getVariableIndex()`/`.getLinkIndex()`/
    `.getJointIndex()`, other integer variables, integer literals, a
    ternary between two integer literals, or an enum constant
    (`ADD_SHAPE`). Checked every non-`for`-loop hit's source line
    individually this round — none narrows a floating-point value.
  - **0 real narrowing sites**, all `distinct`.
- Port-side direction: `rg '\bas\s+(i8|i16|i32|i64|i128|isize|u8|u16|u32|u64|u128|usize)\b'`
  across `crates/moveit-scene` (src + tests) — **0 hits**.
- Both directions swept, both zero, no fix needed.

## §79 count: `assert_relative_eq!`/`relative_eq!` epsilon/max_relative sites

Round 17: see `moveit-metrics.md`'s sibling section for the full
writeup — the command covers both crates together. This crate's own
call (`tests/frame_transform_parity.rs:148`) is `both` (`epsilon` and
`max_relative` set); 0 `epsilon`-only or neither-set sites here.

## Defect attribution audit

Not a citation-vs-upstream row (this table's usual subject), but the same
`where`/`claim`/`verdict`/`evidence`/`commit` shape applies to a claim
about *where a defect lives*, and this file is the right place to record
one going stale, per §175.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-scene/tests/cost_sources_parity.rs` (`panda_cost_sources_blocked_by_mesh_shape_cost_sources`'s `#[ignore]` reason) | state-op id 5 (`group_name="hand"`, `9` actual vs `2` expected) is a group-filtering defect in `PlanningScene::cost_sources`, this crate's own code | EXPIRED | Independently isolated, not relayed: reverting only the `parry.rs` hunk of `moveit-collision`'s `585a79e` and rerunning reproduces the exact `count mismatch left: 9 right: 2`; restoring it passes, and `cargo nextest run -p moveit-scene --run-ignored all` gives 86/86. The defect was `group_name` filtering being entirely unimplemented in `moveit-collision`'s `check_self_collision`/`check_robot_collision`/`distance_self`/`distance_robot` — all four received the field and never read it, against that crate's own module doc claiming the omission matched upstream (it does not: `cd.enableGroup(getRobotModel())` is unconditional at `collision_env_fcl.cpp:281,336`). `PlanningScene::cost_sources` never had a bug; it correctly passed `group_name` down to a backend that silently dropped it. | `585a79e` (fix, `moveit-collision`) |

The attribution was made from outside the crate that held the defect, by
reasoning about which layer *should* own group filtering rather than by a
count or a probe against either crate — the same shape as `PORTING-PLAN.md`
§139, §185, and §186 (an inheritance/dependency *relationship* used to
conclude a call count, refuted each time by actually counting). Here the
relationship was architectural ownership rather than inheritance, but the
failure mode is the same: a relationship stood in for a measurement, and
the measurement said otherwise.
