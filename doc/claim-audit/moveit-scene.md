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
re-opened and re-verified `scene.rs:693`, `:853`, `:1644`, and the
`pipeline.rs:20`/`:303`/`response.rs:99` sites that belong to
`moveit-planning` (not this file, see below) myself before trusting them.

Round `c0061a9` (this round): the two aggregate rows this note used to
describe (`scene.rs` lines 240-940 and 1100-3020, "~26"/"~15 sites,
counted but not individually itemized") are closed out. Every citation in
both ranges, plus every citation in `attached_body.rs`, was opened
individually against `moveit2` at the pinned commit via `rg -n` (exact
anchor, never manual line counting — see `PORTING-PLAN.md`'s own
`:1822-1837` finding in this same file's table for what manual counting
produces). 42 rows now cover this crate's citations with no aggregate
remaining; see the table and Summary below.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-scene/src/scene.rs:377` | `distanceToCollisionUnpadded` cited at `planning_scene.cpp:461-509` | EXPIRED | opened `planning_scene.cpp:461-509` — that range is `checkCollisionUnpadded`'s body (forwards to `checkCollision`), not `distanceToCollisionUnpadded`. Real function is inline at `planning_scene.hpp:553-609` (4 overloads: `:553,561,588,598`). The claim itself ("each overload only forwards") holds against the real function, but the citation points at the wrong one. | `9e07d32` |
| `crates/moveit-scene/src/scene.rs:2021` | `decoupleParent`'s materialization of `scene_transforms_`, cited `planning_scene.cpp:343-344` | EXPIRED | opened `planning_scene.cpp:343-344` — inside `pushDiffs()`, guarded by `if (scene_transforms_.has_value())`, opposite direction of data flow from what's described. Real `decoupleParent` materialization is `planning_scene.cpp:1260-1264`, guarded by `if (!scene_transforms_.has_value())`. | `679a8f6` |
| `crates/moveit-scene/src/scene.rs:2585` | same claim, second occurrence, same citation `planning_scene.cpp:344` | EXPIRED | same as above — `:344` is inside `pushDiffs()`, not `decoupleParent`. | `679a8f6` |
| `crates/moveit-scene/src/scene.rs:1852-1867` | 5 per-statement citations for the `path_cost_sources` sequence (`cs.insert` at `:2465`, `cs_start.swap` at `:2467-2468`, truncation loop at `:2471-2481`, `removeCostSources` at `:2483`, `removeOverlapping` at `:2484`) | EXPIRED | opened `planning_scene.cpp` around each via `rg -n` (exact line numbers, not manual counting) — every anchor was off by 6-7 lines from the real statement (`cs.insert` actually at `:2472`, `cs_start.swap` at `:2474`, truncation if/else at `:2477-2487`, `removeCostSources` at `:2489`, `removeOverlapping` at `:2490`). The described ordering/behavior is correct against the real lines; only the per-statement anchors were misplaced, consistently by the same offset. | `f19d2e6` |
| `crates/moveit-scene/src/scene.rs:693` | `scene_transforms_` is "reset at `[PlanningScene::diff]` time (`planning_scene.cpp:1264`)" | EXPIRED | opened `planning_scene.cpp:1264` — that's inside `decoupleParent()` and it *sets* the value (`setAllTransforms`), not resets it. `diff()`'s own constructor never touches `scene_transforms_`. Real `.reset()` is `planning_scene.cpp:331`, inside `clearDiffs()`. Verified directly (not just via subagent). | `8abeca0` |
| `crates/moveit-scene/src/scene.rs:853` | `getPlanningFrame` is "always the robot model's frame, since that is the only value `PlanningScene::initialize` ever passes to it (`planning_scene.cpp:192`)" | EXPIRED | `rg -n 'make_shared<SceneTransforms>' moveit_core/planning_scene/src/planning_scene.cpp` inside the moveit2 checkout returns 4 sites (192, 686, 1263, 1333), not just `initialize()`. End conclusion (planning frame is always the robot model's frame) is still true, but the stated exclusivity/mechanism ("the only value ... ever passes") is false — the real invariant is `SceneTransforms`'s constructor always computes the robot-model frame regardless of caller. Verified directly. | `f4fbcc0` |
| `crates/moveit-scene/src/scene.rs:1629` | the `moveit_msgs::Constraints` overload (cited `:2245`/`:2269`) is "a construct-then-delegate wrapper ... build a `KinematicConstraintSet` from the message, then call this exact method" | EXPIRED | opened `planning_scene.cpp:2245` and `:2269` — only `:2245` (which delegates to `:2253`, the overload that actually builds the `KinematicConstraintSet`) matches the claim. `:2269` takes an already-native `KinematicConstraintSet&` and only converts the *state* argument from a message, not the constraint set. | `1e8bfb6` |
| `crates/moveit-scene/src/scene.rs:1644` | `isStateColliding(state, group, verbose)` at `planning_scene.cpp:2217` "overwrites whatever `group_name` a caller-built request carried" | EXPIRED | opened `planning_scene.cpp:2217` (real body at `:2219`) — that function has no `CollisionRequest` parameter at all; `req` is always freshly, locally constructed inside it. There is no caller-built request for `group` to overwrite. Verified directly. | `c9fcf9e` |
| `crates/moveit-scene/src/attached_body.rs:117` (`subframe_pose`) | key restricted to lookup with the body id stripped; upstream's own key is `"<id>/<name>"`, cited `attached_body.cpp:139-155` | CONFIRMED | opened `attached_body.cpp:139-155` — exact: `getSubframeTransform` starts at `:139`, closes at `:155`; `:141`/`:143` build the key exactly as `frame_name.rfind(id_, 0) == 0 && frame_name[id_.length()] == '/'` then `substr(id_.length() + 1)`. | |
| `crates/moveit-scene/src/attached_body.rs:147` (`set_scale`) | Upstream `setScale`, cited `attached_body.cpp:86-103` | CONFIRMED | opened `attached_body.cpp:86-103` — exact: function starts `:86`, closes `:103`. `use_count() == 1` branch matches the doc's description of `Arc::make_mut`'s divergence. | |
| `crates/moveit-scene/src/attached_body.rs:196` (`set_padding`) | Upstream `setPadding`, cited `attached_body.cpp:120-137` | CONFIRMED | opened `attached_body.cpp:120-137` — exact: function starts `:120`, closes `:137`, identical shape to `setScale` with `padd` in place of `scale`. | |
| `crates/moveit-scene/src/scene.rs:257-260` (`getTransforms` overloads) | cited `planning_scene.hpp:184,197,200` + `planning_scene.cpp:671` | CONFIRMED | opened all four — each line is the exact overload/definition claimed. | |
| `crates/moveit-scene/src/scene.rs:282-284`, `:1276-1307`, `:2913` (`getFrameTransform` tier 6 / `frame_transform`'s doc / a test comment) | falls through to `Transforms::getTransform`, cited `planning_scene.cpp:2050` | EXPIRED (wrong-anchor-right-claim) | opened `:2050` — that's `getWorld()->getTransform(frame_id, frame_found)` (tier 2, the world lookup), not the base-class fallback. The real `return getTransforms().Transforms::getTransform(frame_id);` statement is `:2053`. Same wrong anchor recurs at 3 sites. | `8ab4e88` |
| `crates/moveit-scene/src/scene.rs:288-289`, `:1337` (`knowsFrameTransform`) | cited `planning_scene.cpp:2056`/`:2061` | CONFIRMED | opened both — `:2056` is the id-only overload, `:2061` is the explicit-state overload; both exact. | |
| `crates/moveit-scene/src/scene.rs:300-304` (`getCollisionDetectorName` branch) | cited `planning_scene.cpp:291`/`:304` | CONFIRMED | opened both — exact `if (collision_detector_name != getCollisionDetectorName())` branches inside `getCollisionEnv`/`getCollisionEnvUnpadded`. | |
| `crates/moveit-scene/src/scene.rs:319-327` (`getCollisionEnv` family, `allocateCollisionDetector`) | cited `planning_scene.cpp:255-286` (allocator) and `:288-311` (name-lookup branches) | CONFIRMED | `allocateCollisionDetector` spans exactly `:255-286`, pinpoint. The `:288-311` range fully contains both cited branches (`:291`, `:304`); the true end of the second function is `:314`, 3 lines past the citation, but the branches themselves are entirely inside `:288-311`. | |
| `crates/moveit-scene/src/scene.rs:390` (`saveGeometryToStream`/`loadGeometryFromStream`) | cited `planning_scene.cpp:1062`/`:1152` | CONFIRMED | opened both — `shapes::saveAsText`/`shapes::constructShapeFromText` are literally on those lines. | |
| `crates/moveit-scene/src/scene.rs:456-457` (`setAttachedBodyUpdateCallback`) | cited `attached_body.hpp:52` + `planning_scene.cpp:630,643,1229,1270,1554` | CONFIRMED | opened all five — exact. | |
| `crates/moveit-scene/src/scene.rs:468-471` (`setCollisionObjectUpdateCallback`) | cited `world.hpp:304` + `world.cpp:220,326,650,655` | CONFIRMED | opened all four — exact. | |
| `crates/moveit-scene/src/scene.rs:490`, `:1675` (`isStateFeasible` unconditional `true`) | cited `planning_scene.cpp:2227-2243` | CONFIRMED | opened `:2227-2243` — exact function span, both overloads (message-state `:2227-2236`, native-state `:2238-2243`); native overload's unconditional `true` fallback confirmed present. | |
| `crates/moveit-scene/src/scene.rs:496` (`motion_feasibility_` never read) | no line cited; claim is that the field is unused | CONFIRMED | `rg motion_feasibility_` across `planning_scene.cpp`/`.hpp` in the pinned checkout: only declared, set via `setMotionFeasibilityPredicate`, never read anywhere including inside `isPathValid`'s full body. | |
| `crates/moveit-scene/src/scene.rs:523-526`, `:1788-1801` (`cost_sources`, state-taking pair) | cited `planning_scene.cpp:2493-2506` | EXPIRED (wrong-anchor-right-claim) | opened `:2493-2506` — the two overloads' true span is `:2493-2510`; `:2506` stops on `creq.cost = true;`, excluding the `checkCollision`/`cres.cost_sources.swap(costs)` calls the very next paragraph names as the whole point of the citation. | `2eb2999` |
| `crates/moveit-scene/src/scene.rs:526` (module-summary citation for `path_cost_sources`) | cited `planning_scene.cpp:2451-2489` | EXPIRED (wrong-anchor-right-claim) | opened `:2451-2489` — excludes the `removeOverlapping` call at `:2490` that the same paragraph's own sentence 3 lines later calls part of the "load-bearing... call order". The sibling per-method citation at `scene.rs:1826` already said `:2451-2490`; this row disagreed with it. | `2eb2999` |
| `crates/moveit-scene/src/scene.rs:1856-1884` (`path_cost_sources`'s per-method doc, 6 sub-anchors) | `planning_scene.cpp:2451-2490` (pair), `:2472` (`cs.insert`), `:2474` (`cs_start.swap`), `:2477-2487` (truncation), `:2489` (`removeCostSources`), `:2490` (`removeOverlapping`) | CONFIRMED | opened all six — every one pinpoint-exact against the real statement, including the truncation if/else block's exact `:2477-2487` boundary. | |
| `crates/moveit-scene/src/scene.rs:544` (`printKnownObjects`) | body "is exactly" `planning_scene.cpp:2512-2531` | EXPIRED (wrong-anchor-right-claim) | opened `:2512-2531` — real closing brace is `:2533`; `:2531` stops 2 lines short of the trailing separator-line print and the brace. Claim text explicitly asserts precision ("is exactly that"). | `6ef3032` |
| `crates/moveit-scene/src/scene.rs:869` (`SceneTransforms::isFixedFrame` override) | cited `planning_scene.cpp:123` | CONFIRMED | exact — `bool isFixedFrame(...) override` is literally on `:123`. | |
| `crates/moveit-scene/src/scene.rs:873-876` ("four `configure(msg, tf)` sites") | `kinematic_constraint.cpp`'s `configure(msg, tf)` count | EXPIRED | `rg -n '::configure\(' kinematic_constraint.cpp` finds 4 definitions, but only 3 take a `Transforms&` (`PositionConstraint`, `OrientationConstraint`, `VisibilityConstraint` — `JointConstraint::configure` takes no `tf`). "Four" was conflating the method count with the 4 `tf.isFixedFrame(...)` call sites (`VisibilityConstraint::configure` checks two frame ids), which are correctly enumerated elsewhere on `knows_frame_transform`'s own doc (`:382,622,848,861`). | `2157210` |
| `crates/moveit-scene/src/scene.rs:900`, `:1380` (`isFixedFrame` leading-`/` strip) | cited `planning_scene.cpp:127-134` / `:123-135` | CONFIRMED | opened both — the described branches (`Transforms::isFixedFrame` base check, then the leading-`/`-strip `if`) are fully inside both ranges; `:123-135`'s true function close is `:137`, 2 lines past the citation, but nothing described is excluded. | |
| `crates/moveit-scene/src/scene.rs:926`/`:923` (`World::knowsTransform` ordering) | cited `world.cpp:145`/`:150` | CONFIRMED | opened both — `:145` is the exact `objects_.find(name)` object-id check, `:150` the exact `else // Then objects' subframes` branch start. | |
| `crates/moveit-scene/src/scene.rs:1162` (subframe carry-through on `attach`) | cited `planning_scene.cpp:1590` | CONFIRMED | exact — `subframe_poses = obj_in_world->subframe_poses_;` is literally on `:1590`. | |
| `crates/moveit-scene/src/scene.rs:1267` (`setSubframesOfObject` after `addToObject` on `detach`) | cited `planning_scene.cpp:1743` | CONFIRMED | exact — `world_->setSubframesOfObject(...)` immediately follows `addToObject` at `:1742`. | |
| `crates/moveit-scene/src/scene.rs:1306-1337` (`frame_transform`'s ladder, `:2036` half) | cited `planning_scene.cpp:2036` | CONFIRMED | exact — `getFrameTransform(state, frame_id)`'s definition starts at `:2036`. (The `:2050` half of this same doc block is the EXPIRED row above.) | |
| `crates/moveit-scene/src/scene.rs:1376` (`RobotState::knowsFrameTransform` has no model-frame special case) | cited `robot_state.cpp:1386-1405` | CONFIRMED | opened `:1386-1405` — pinpoint-exact function span; body only checks `hasLinkModel`/attached-body map/attached-body subframes, no model-frame check anywhere in it. | |
| `crates/moveit-scene/src/scene.rs:1415-1418` ("four `isFixedFrame` callers", `knows_frame_transform`'s doc) | cited `kinematic_constraint.cpp:382,622,848,861` | CONFIRMED | `rg -n isFixedFrame kinematic_constraint.cpp` returns exactly those four lines, all `tf.isFixedFrame(...)` calls — pinpoint-exact. Distinct claim from the miscounted `:843-846` row above (this one counts call sites, correctly, and gives correct line numbers). | |
| `crates/moveit-scene/src/scene.rs:1566` (`getCollidingPairs` group_name overloads) | cited `planning_scene.hpp:492-495` | CONFIRMED | opened `:492-495` — anchors the first of the four `group_name`-taking overloads (confirmed 4 exist: `:492,501,512,522`, vs. 2 without); true closing brace of the cited one is `:496`, 1 line past the citation, but it is a representative anchor, not a claim of covering all four locations. | |
| `crates/moveit-scene/src/scene.rs:1636-1647` (`isStateConstrained` family) | cited `planning_scene.cpp:2277` (target), `:2245`/`:2253`/`:2269` (sub-anchors) | CONFIRMED | opened all four — exact function-start matches, including `:2269` correctly described as "a distinct overload" (converts only the state, not the constraint set). | |
| `crates/moveit-scene/src/scene.rs:1655-1666` (`isStateColliding` family) | cited `planning_scene.cpp:2217` (target), `:2197`, `:2219` | CONFIRMED | opened all three — exact; `:2219` is literally `collision_detection::CollisionRequest req;`, the first line of the always-local-request body. | |
| `crates/moveit-scene/src/scene.rs:1681` (`isStateValid`) | cited `planning_scene.cpp:2313` | CONFIRMED | exact — `isStateValid(state, KinematicConstraintSet, group, verbose)` starts at `:2313`. | |
| `crates/moveit-scene/src/scene.rs:1736` (`isPathValid`) | cited `planning_scene.cpp:2365` | CONFIRMED | exact — the `RobotTrajectory`-taking `isPathValid` starts at `:2365`. | |
| `crates/moveit-scene/src/scene.rs:1745` (`isPathValid` loop body, no interpolation) | cited `planning_scene.cpp:2376-2422` | CONFIRMED | opened `:2376-2422` — exact loop span (`for` at `:2376` through its closing brace at `:2422`); body only ever reads `trajectory.getWayPoint(i)`, confirming "no state between two requested waypoints is ever constructed or checked". | |
| `crates/moveit-scene/src/scene.rs:2032`, `:2566` (`decouple_parent`'s `scene_transforms_` materialization) | cited `planning_scene.cpp:1260-1264` | CONFIRMED | opened both occurrences — described content (`has_value()` check, `emplace`, `setAllTransforms`) is fully inside `:1260-1264`; true closing brace is `:1265`, 1 line past. | |
| `crates/moveit-scene/src/scene.rs:2854` (`frame_transform_prefers_the_attached_body_tier...` test) | cited `planning_scene.cpp:2036` | CONFIRMED | exact — anchors only the function start for an ordering claim between tier 2 and tier 5, correctly. | |

## Summary

- 42 table rows total: 8 from Round 11 (already itemized) + 34 opened and
  itemized this round (3 `attached_body.rs` + 31 `scene.rs`). This
  supersedes the prior "41 sites (40 scene.rs + 1 attached_body.rs)"
  estimate — the real number of distinct citation locations, once every
  module-summary and per-method doc comment is opened individually, is
  higher (`attached_body.rs` alone has 3 citations, not 1; several
  `scene.rs` claims — `getFrameTransform`'s tier 6,
  `getCostSources`/`isFixedFrame`'s call counts — recur at 2-3 separate
  doc-comment locations that each needed opening on their own), which is
  exactly the "your measured number beats the brief's" effect a bucket
  hides.
- EXPIRED this round: 5 distinct findings across rows 13, 22, 23, 25, 27
  (`:2050`→`:2053`, recurring at 3 physical sites; `getCostSources`
  state-pair range; `getCostSources` module-summary path range;
  `printKnownObjects` range; "four `configure(msg,tf)` sites" miscount).
  The first four are wrong-anchor-right-claim (citation lands short of or
  beside the described content, but the underlying claim about upstream
  holds once you read the right lines); the last is a substantive count
  error, not a line-anchor problem. Fixed one commit each: `8ab4e88`,
  `2eb2999` (covers both `getCostSources` rows), `6ef3032`, `2157210`.
- CONFIRMED this round: the remaining 29 rows opened this round — every
  citation independently re-verified against the pinned upstream source,
  no aggregate row left in this file.
- EXPIRED from the prior round (Round 11), already fixed: 8, listed above
  this section — `8abeca0`, `f4fbcc0`, `1e8bfb6`, `c9fcf9e`, `9e07d32`,
  `679a8f6` (covers both `:1991` and `:2555`), `f19d2e6`.
- Combined EXPIRED rate across both rounds: 13 of 42 rows (31%) — far
  below the 8-of-8 (100%) rate the Round-11 itemized subset alone showed,
  confirming that subset was not representative of the buckets it sat
  inside once those buckets were actually opened.

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
