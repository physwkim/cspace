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
| `crates/moveit-scene/src/scene.rs:377` | `distanceToCollisionUnpadded` cited at `planning_scene.cpp:461-509` | EXPIRED | opened `planning_scene.cpp:461-509` — that range is `checkCollisionUnpadded`'s body (forwards to `checkCollision`), not `distanceToCollisionUnpadded`. Real function is inline at `planning_scene.hpp:553-601`. The claim itself ("each overload only forwards") holds against the real function, but the citation points at the wrong one. | |
| `crates/moveit-scene/src/scene.rs:1991` | `decoupleParent`'s materialization of `scene_transforms_`, cited `planning_scene.cpp:343-344` | EXPIRED | opened `planning_scene.cpp:343-344` — inside `pushDiffs()`, guarded by `if (scene_transforms_.has_value())`, opposite direction of data flow from what's described. Real `decoupleParent` materialization is `planning_scene.cpp:1260-1264`, guarded by `if (!scene_transforms_.has_value())`. | |
| `crates/moveit-scene/src/scene.rs:2555` | same claim, second occurrence, same citation `planning_scene.cpp:344` | EXPIRED | same as above — `:344` is inside `pushDiffs()`, not `decoupleParent`. | |
| `crates/moveit-scene/src/scene.rs:1822-1837` | 5 per-statement citations for the `path_cost_sources` sequence (`cs.insert` at `:2465`, `cs_start.swap` at `:2467-2468`, truncation loop at `:2471-2481`, `removeCostSources` at `:2483`, `removeOverlapping` at `:2484`) | EXPIRED | opened `planning_scene.cpp` around each — every anchor is off by 6-7 lines from the real statement (`cs.insert` actually at `:2472`, `cs_start.swap` at `:2474`, truncation loop at `:2482-2487`, `removeCostSources` at `:2489`, `removeOverlapping` at `:2490`). The described ordering/behavior is correct against the real lines; only the per-statement anchors are misplaced, consistently by the same offset. | |
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
- None of the 8 EXPIRED findings fixed yet this round — the routing
  instruction asked for the audit and category counts, not remediation;
  see the p1-fixtures round report for the explicit scope decision.

## §172 narrowing sweep (separate exercise, same round)

Not a citation audit, but recorded here for the same compaction-risk
reason §175 flags — see `moveit-metrics.md` for the sibling record.

- Upstream-first direction: swept the 7 upstream files that provenance
  this crate's citations — `planning_scene/src/planning_scene.cpp`,
  `planning_scene/include/.../planning_scene.hpp`, `robot_state/src/robot_state.cpp`,
  `robot_state/include/.../attached_body.hpp`, `robot_state/src/attached_body.cpp`,
  `collision_detection/src/world.cpp`, `collision_detection/include/.../world.hpp`,
  `kinematic_constraints/src/kinematic_constraint.cpp` — for `int`/`unsigned`/
  `size_t`/`std::size_t`/`long`/`uint32_t`/`int32_t` declarations or
  `static_cast<...>` narrowing a floating-point initializer. Hits: 19
  across `planning_scene.cpp` (loop counters bound by `.size()`, waypoint
  counts, shape counts), 1 in `attached_body.cpp` (loop counter), 2 in
  `robot_state.cpp` (`static_cast<std::size_t>(s.rows())`, an Eigen row
  count, not a float). Every hit is a real integer quantity — **0 real
  narrowing sites**, all `distinct`.
- Port-side direction: `rg '\bas\s+(i8|i16|i32|i64|i128|isize|u8|u16|u32|u64|u128|usize)\b'`
  across `crates/moveit-scene` (src + tests) — **0 hits**.
- Both directions swept, both zero, no fix needed.
