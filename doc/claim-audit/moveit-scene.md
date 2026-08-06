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
re-opened and re-verified `scene.rs:693` (whose doc reads `clearDiffs()`
at that line), `:853`, `:1644`, and the `pipeline.rs:20`/`:303`/
`response.rs:98-99` sites that belong to `moveit-planning` (not this
file, see below) myself before trusting them — `pipeline.rs:20` reads
`unordered_map<string, PlannerManagerPtr>`, and `response.rs:98-99`
reads `moveit_msgs::msg::RobotState start_state;`.

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
| `crates/moveit-scene/src/scene.rs:377-378` | `distanceToCollisionUnpadded` cited at `planning_scene.cpp:461-509` | EXPIRED | opened `planning_scene.cpp:461-509` — that range is `checkCollisionUnpadded`'s body (forwards to `checkCollision`), not `distanceToCollisionUnpadded`. Real function is inline at `planning_scene.hpp`, in 4 overloads `:553-557`, `:561-564`, `:588-593`, `:598-602` — not one range, because two `distanceToCollision` overloads sit between them at `:569-583`. The claim itself ("each overload only forwards") holds against the real function, but the citation points at the wrong one; the doc comment there reads `getCollisionEnvUnpadded()->distanceRobot(...)`. | `9e07d32` |
| `decouple_parent` (`crates/moveit-scene/src/scene.rs:2032-2033`) | `decoupleParent`'s materialization of `scene_transforms_`, cited `planning_scene.cpp:343-344` | EXPIRED | opened `planning_scene.cpp:343-344` — inside `pushDiffs()`, guarded by `if (scene_transforms_.has_value())`, opposite direction of data flow from what's described. Real `decoupleParent` materialization is `planning_scene.cpp:1260-1264`, guarded by `if (!scene_transforms_.has_value())`. This round: the `where` column previously cited scene.rs line 2021, inside `push_diffs`'s world-diff replay loop (subframe collection, no `scene_transforms_` anywhere near it) — the doc comment that actually states "`scene_transforms_` is layered ... and is materialized here" is `decouple_parent`'s own, re-derived to `:2032-2033`. | `679a8f6`, then this round |
| `crates/moveit-scene/src/scene.rs:2585` (`decouple_parent_then_mutating_the_former_parent_is_not_observed`) | same claim, second occurrence, same citation `planning_scene.cpp:344` | EXPIRED | same as above — `:344` is inside `pushDiffs()`, not `decoupleParent`. | `679a8f6` |
| `path_cost_sources` (`crates/moveit-scene/src/scene.rs:1863-1887`) | 5 per-statement citations for the `path_cost_sources` sequence (`cs.insert` at `:2465`, `cs_start.swap` at `:2467-2468`, truncation loop at `:2471-2481`, `removeCostSources` at `:2483`, `removeOverlapping` at `:2484`) | EXPIRED | opened `planning_scene.cpp` around each via `rg -n` (exact line numbers, not manual counting) — every anchor was off by 6-7 lines from the real statement (`cs.insert` actually at `:2472`, `cs_start.swap` at `:2474`, truncation if/else at `:2477-2487`, `removeCostSources` at `:2489`, `removeOverlapping` at `:2490`). The described ordering/behavior is correct against the real lines; only the per-statement anchors were misplaced, consistently by the same offset. This round: the `where` column previously cited scene.rs lines 1852-1867, which reaches only the first of the 5 named bullets (`cs.insert`, at `:1866-1867`) and misses the other 4 entirely (`cs_start.swap` at `:1869`, truncation at `:1871-1875`, `removeCostSources` at `:1876-1880`, `removeOverlapping` at `:1881-1887`); re-derived to the full 5-bullet span. | `f19d2e6`, then this round |
| `crates/moveit-scene/src/scene.rs:693` | `scene_transforms_` is "reset at `[PlanningScene::diff]` time (`planning_scene.cpp:1264`)" | EXPIRED | opened `planning_scene.cpp:1264` — that's inside `decoupleParent()` and it *sets* the value (`setAllTransforms`), not resets it. `diff()`'s own constructor never touches `scene_transforms_`. Real `.reset()` is `planning_scene.cpp:331`, inside `clearDiffs()`. Verified directly (not just via subagent). | `8abeca0` |
| `planning_frame` (`crates/moveit-scene/src/scene.rs:853-854`) | `getPlanningFrame` is "always the robot model's frame, since that is the only value `PlanningScene::initialize` ever passes to it (`planning_scene.cpp:192`)" | EXPIRED | `rg -n 'make_shared<SceneTransforms>' moveit_core/planning_scene/src/planning_scene.cpp` inside the moveit2 checkout returns 4 sites (192, 686, 1263, 1333), not just `initialize()`. End conclusion (planning frame is always the robot model's frame) is still true, but the stated exclusivity/mechanism ("the only value ... ever passes") is false — the real invariant is `SceneTransforms`'s constructor always computes the robot-model frame regardless of caller. Verified directly. | `f4fbcc0` |
| `is_state_constrained` (`crates/moveit-scene/src/scene.rs:1639-1647`) | the `moveit_msgs::Constraints` overload (cited `:2245`/`:2269`) is "a construct-then-delegate wrapper ... build a `KinematicConstraintSet` from the message, then call this exact method" | EXPIRED | opened `planning_scene.cpp:2245` and `:2269` — only `:2245` (which delegates to `:2253`, the overload that actually builds the `KinematicConstraintSet`) matches the claim. `:2269` takes an already-native `KinematicConstraintSet&` and only converts the *state* argument from a message, not the constraint set. This round: the `where` column previously cited scene.rs line 1629, inside `colliding_links` (`getCollidingLinks`'s port), a different method entirely with no `moveit_msgs::Constraints` discussion anywhere near it; the doc comment this claim is actually about is `is_state_constrained`'s, re-derived to `:1639-1647`. | `1e8bfb6`, then this round |
| `is_state_colliding` (`crates/moveit-scene/src/scene.rs:1664-1669`) | `isStateColliding(state, group, verbose)` at `planning_scene.cpp:2217` "overwrites whatever `group_name` a caller-built request carried" | EXPIRED | opened `planning_scene.cpp:2217` (real body at `:2219`) — that function has no `CollisionRequest` parameter at all; `req` is always freshly, locally constructed inside it. There is no caller-built request for `group` to overwrite. Verified directly. This round: the `where` column previously cited scene.rs line 1644, inside `is_state_constrained`'s doc (a sibling method's doc comment, not this one's), where nothing discusses `group_name` or a `CollisionRequest`; the doc comment that actually says "`group` lives on `request.group_name` ... always builds its own local `CollisionRequest` internally" is `is_state_colliding`'s, re-derived to `:1664-1669`. | `c9fcf9e`, then this round |
| `crates/moveit-scene/src/attached_body.rs:117` (`subframe_pose`) | key restricted to lookup with the body id stripped; upstream's own key is `"<id>/<name>"`, cited `attached_body.cpp:139-155` | CONFIRMED | opened `attached_body.cpp:139-155` — exact: `getSubframeTransform` starts at `:139`, closes at `:155`; `:141`/`:143` build the key exactly as `frame_name.rfind(id_, 0) == 0 && frame_name[id_.length()] == '/'` then `substr(id_.length() + 1)`. | |
| `set_scale` (`crates/moveit-scene/src/attached_body.rs:147-148`) | Upstream `setScale`, cited `attached_body.cpp:86-103` | CONFIRMED | opened `attached_body.cpp:86-103` — exact: function starts `:86`, closes `:103`. `use_count() == 1` branch matches the doc's description of `Arc::make_mut`'s divergence. | |
| `set_padding` (`crates/moveit-scene/src/attached_body.rs:196-197`) | Upstream `setPadding`, cited `attached_body.cpp:120-137` | CONFIRMED | opened `attached_body.cpp:120-137` — exact: function starts `:120`, closes `:137`, identical shape to `setScale` with `padd` in place of `scale`. | |
| `crates/moveit-scene/src/scene.rs:257-260` (`getTransforms` overloads) | cited `planning_scene.hpp:184,197,200` + `planning_scene.cpp:671` | CONFIRMED | opened all four — each line is the exact overload/definition claimed; the doc itself reads `getTransformsNonConst()` at `:260`. | |
| `crates/moveit-scene/src/scene.rs:282-284`, `:1276-1307`, `:2913` (`getFrameTransform` tier 6 / `frame_transform`'s doc / a test comment) | falls through to `Transforms::getTransform`, cited `planning_scene.cpp:2050` | EXPIRED (wrong-anchor-right-claim) | opened `:2050` — that's `getWorld()->getTransform(frame_id, frame_found)` (tier 2, the world lookup), not the base-class fallback. The real `return getTransforms().Transforms::getTransform(frame_id);` statement is `:2053`. Same wrong anchor recurs at 3 sites. | `8ab4e88` |
| `crates/moveit-scene/src/scene.rs:288-289`, `:1337` (`knowsFrameTransform`) | cited `planning_scene.cpp:2056`/`:2061` | CONFIRMED | opened both — `:2056` is the id-only overload, `:2061` is the explicit-state overload; both exact; `:288-289`'s own text reads `PlanningScene::knows_frame_transform`. | |
| `crates/moveit-scene/src/scene.rs:300-304` (`getCollisionDetectorName` branch) | cited `planning_scene.cpp:291`/`:304` | CONFIRMED | opened both — exact `if (collision_detector_name != getCollisionDetectorName())` branches inside `getCollisionEnv`/`getCollisionEnvUnpadded(name)`. | |
| `crates/moveit-scene/src/scene.rs:315-327` (`getCollisionEnv` family, `allocateCollisionDetector`) | cited `planning_scene.cpp:255-286` (allocator) and `:288-311` (name-lookup branches) | CONFIRMED | `allocateCollisionDetector` spans exactly `:255-286`, pinpoint. The `:288-311` range fully contains both cited branches (`:291`, `:304`); the true end of the second function is `:314`, 3 lines past the citation, but the branches themselves are entirely inside `:288-311`; the bullet itself opens `getCollisionEnv(name)` among the overloads it names. | |
| `crates/moveit-scene/src/scene.rs:389-390` (`saveGeometryToStream`/`loadGeometryFromStream`) | cited `planning_scene.cpp:1062`/`:1152` | CONFIRMED | opened both — `shapes::saveAsText`/`shapes::constructShapeFromText` are literally on those lines, matching the doc's own `shapes::saveAsText` mention. | |
| `crates/moveit-scene/src/scene.rs:456-458` (`setAttachedBodyUpdateCallback`) | cited `attached_body.hpp:52` + `planning_scene.cpp:630,643,1229,1270,1554` | CONFIRMED | opened all five — exact; `AttachedBodyCallback` is quoted in the doc itself as `void(AttachedBody*, bool)`. | |
| `crates/moveit-scene/src/scene.rs:468-471` (`setCollisionObjectUpdateCallback`) | cited `world.hpp:304` + `planning_scene.cpp:220,326,650,655` | CONFIRMED | opened all four — exact; `ObserverCallbackFn` is quoted in the doc itself as `void(const ObjectConstPtr&, Action)`. | |
| `crates/moveit-scene/src/scene.rs:490-491`, `:1675` (`isStateFeasible` unconditional `true`) | cited `planning_scene.cpp:2227-2243` | CONFIRMED | opened `:2227-2243` — exact function span, both overloads (message-state `:2227-2236`, native-state `:2238-2243`); native overload's unconditional `true` fallback confirmed present, and the doc names it as what `PlanningScene::is_state_valid` takes. | |
| `crates/moveit-scene/src/scene.rs:496` (`motion_feasibility_` never read) | no line cited; claim is that the field is unused | CONFIRMED | `rg motion_feasibility_` across `planning_scene.cpp`/`.hpp` in the pinned checkout: only declared, set via `setMotionFeasibilityPredicate`, never read anywhere including inside `isPathValid`'s full body. | |
| `crates/moveit-scene/src/scene.rs:523-526`, `:1788-1801` (`cost_sources`, state-taking pair) | cited `planning_scene.cpp:2493-2506` | EXPIRED (wrong-anchor-right-claim) | opened `:2493-2506` — the two overloads' true span is `:2493-2510`; `:2506` stops on `creq.cost = true;`, excluding the `checkCollision`/`cres.cost_sources.swap(costs)` calls the very next paragraph names as the whole point of the citation. | `2eb2999` |
| `crates/moveit-scene/src/scene.rs:525-526` (module-summary citation for `path_cost_sources`) | cited `planning_scene.cpp:2451-2489` | EXPIRED (wrong-anchor-right-claim) | opened `:2451-2489` — excludes the `removeOverlapping` call at `:2490` that the same paragraph's own sentence 3 lines later calls part of the "load-bearing... call order". Fixed to `:2451-2490` in `2eb2999`, agreeing with the sibling per-method citation — and both were still one line short: the pair's second overload opens at `:2457` and its closing brace is `:2491`, so the real span is `:2451-2491`. Re-fixed at merge; see the row below; `:525` itself names `PlanningScene::path_cost_sources`. | `2eb2999`, then this merge |
| `path_cost_sources` (`crates/moveit-scene/src/scene.rs:1857-1887`) (`path_cost_sources`'s per-method doc, 6 sub-anchors) | `planning_scene.cpp:2451-2491` (pair), `:2472` (`cs.insert`), `:2474` (`cs_start.swap`), `:2477-2487` (truncation), `:2489` (`removeCostSources`), `:2490` (`removeOverlapping`) | CONFIRMED (corrected: the pair range was one line short) | opened all six. The five single-statement anchors are pinpoint-exact, including the truncation if/else block's exact `:2477-2487` boundary. The pair range was not: it read `:2451-2490`, and `:2490` is `removeOverlapping`, the last *statement* — the second overload's closing brace is `:2491`. This row's original "CONFIRMED" was reached by checking that everything the prose names falls inside the range, which a range one line short of the brace still satisfies; that is the failure mode, not a slip. `tools/ci/verify-upstream-citations.sh` is what showed it mechanically. Corrected here to `:2451-2491`. This round: the `where` column's own citation had the same one-line-short shape — it read scene.rs lines 1856-1884, one line before the doc even starts (`:1856` is blank) and stopping mid-way through the `removeCostSources` bullet, missing `removeOverlapping` (`:1881-1887`) entirely; re-derived to the doc's real span, `:1857-1887`. | `2eb2999`, then this merge, then this round |
| `crates/moveit-scene/src/scene.rs:543-544` (`printKnownObjects`) | body "is exactly" `planning_scene.cpp:2512-2531` | EXPIRED (wrong-anchor-right-claim) | opened `:2512-2531` — real closing brace is `:2533`; `:2531` stops 2 lines short of the trailing separator-line print and the brace. Claim text explicitly asserts precision ("is exactly that"); `:543` itself names `PlanningScene::attached_bodies`. | `6ef3032` |
| `crates/moveit-scene/src/scene.rs:869` (`SceneTransforms::isFixedFrame` override) | cited `planning_scene.cpp:123` | CONFIRMED | exact — `bool isFixedFrame(...) override` is literally on `:123`. | |
| `crates/moveit-scene/src/scene.rs:873-876` ("four `configure(msg, tf)` sites") | `kinematic_constraint.cpp`'s `configure(msg, tf)` count | EXPIRED | `rg -n '::configure\(' kinematic_constraint.cpp` finds 4 definitions, but only 3 take a `Transforms&` (`PositionConstraint`, `OrientationConstraint`, `VisibilityConstraint` — `JointConstraint::configure` takes no `tf`). "Four" was conflating the method count with the 4 `tf.isFixedFrame(...)` call sites (`VisibilityConstraint::configure` checks two frame ids), which are correctly enumerated elsewhere on `knows_frame_transform`'s own doc (`:382,622,848,861`). | `2157210` |
| `transforms_with_world_objects` (`crates/moveit-scene/src/scene.rs:900-901`), `:1380` (`isFixedFrame` leading-`/` strip) | cited `planning_scene.cpp:127-134` / `:123-135` | CONFIRMED | opened both — the described branches (`Transforms::isFixedFrame` base check, then the leading-`/`-strip `if`) are fully inside both ranges; `:123-135`'s true function close is `:137`, 2 lines past the citation, but nothing described is excluded. | |
| `crates/moveit-scene/src/scene.rs:926`/`:923` (`World::knowsTransform` ordering) | cited `world.cpp:145`/`:150` | CONFIRMED | opened both — `:145` is the exact `objects_.find(name)` object-id check, `:150` the exact `else // Then objects' subframes` branch start. | |
| `attach` (`crates/moveit-scene/src/scene.rs:1162-1163`) (subframe carry-through on `attach`) | cited `planning_scene.cpp:1590` | CONFIRMED | exact — `subframe_poses = obj_in_world->subframe_poses_;` is literally on `:1590`. | |
| `detach` (`crates/moveit-scene/src/scene.rs:1267-1268`) (`setSubframesOfObject` after `addToObject` on `detach`) | cited `planning_scene.cpp:1743` | CONFIRMED | exact — `world_->setSubframesOfObject(...)` immediately follows `addToObject` at `:1742`. | |
| `crates/moveit-scene/src/scene.rs:1306-1337` (`frame_transform`'s ladder, `:2036` half) | cited `planning_scene.cpp:2036` | CONFIRMED | exact — `getFrameTransform(state, frame_id)`'s definition starts at `:2036`. (The `:2050` half of this same doc block is the EXPIRED row above.) | |
| `knows_frame_transform` (`crates/moveit-scene/src/scene.rs:1376-1377`) (`RobotState::knowsFrameTransform` has no model-frame special case) | cited `robot_state.cpp:1386-1405` | CONFIRMED | opened `:1386-1405` — pinpoint-exact function span; body only checks `hasLinkModel`/attached-body map/attached-body subframes, no model-frame check anywhere in it. | |
| `knows_frame_transform` (`crates/moveit-scene/src/scene.rs:1415-1418`) ("four `isFixedFrame` callers", `knows_frame_transform`'s doc) | cited `kinematic_constraint.cpp:382,622,848,861` | CONFIRMED | `rg -n isFixedFrame kinematic_constraint.cpp` returns exactly those four lines, all `tf.isFixedFrame(...)` calls — pinpoint-exact. Distinct claim from the miscounted `:843-846` row above (this one counts call sites, correctly, and gives correct line numbers). | |
| `colliding_pairs` (`crates/moveit-scene/src/scene.rs:1566-1567`) (`getCollidingPairs` group_name overloads) | cited `planning_scene.hpp:492-495` | CONFIRMED | opened `:492-495` — anchors the first of the four `group_name`-taking overloads (confirmed 4 exist: `:492,501,512,522`, vs. 2 without); true closing brace of the cited one is `:496`, 1 line past the citation, but it is a representative anchor, not a claim of covering all four locations. | |
| `crates/moveit-scene/src/scene.rs:1636-1647` (`isStateConstrained` family) | cited `planning_scene.cpp:2277` (target), `:2245`/`:2253`/`:2269` (sub-anchors) | CONFIRMED | opened all four — exact function-start matches, including `:2269` correctly described as "a distinct overload" (converts only the state, not the constraint set); the doc itself opens `isStateConstrained(state, KinematicConstraintSet, verbose)`. | |
| `crates/moveit-scene/src/scene.rs:1655-1666` (`isStateColliding` family) | cited `planning_scene.cpp:2217` (target), `:2197`, `:2219` | CONFIRMED | opened all three — exact; `:2219` is literally `collision_detection::CollisionRequest req;`, the first line of the always-local-request body; the doc itself opens `isStateColliding(state, group, verbose)`. | |
| `is_state_valid` (`crates/moveit-scene/src/scene.rs:1681-1682`) (`isStateValid`) | cited `planning_scene.cpp:2313` | CONFIRMED | exact — `isStateValid(state, KinematicConstraintSet, group, verbose)` starts at `:2313`. | |
| `is_path_valid` (`crates/moveit-scene/src/scene.rs:1736-1737`) (`isPathValid`) | cited `planning_scene.cpp:2365` | CONFIRMED | exact — the `RobotTrajectory`-taking `isPathValid` starts at `:2365`. | |
| `is_path_valid` (`crates/moveit-scene/src/scene.rs:1745-1746`) (`isPathValid` loop body, no interpolation) | cited `planning_scene.cpp:2376-2422` | CONFIRMED | opened `:2376-2422` — exact loop span (`for` at `:2376` through its closing brace at `:2422`); body only ever reads `trajectory.getWayPoint(i)`, confirming "no state between two requested waypoints is ever constructed or checked". | |
| `crates/moveit-scene/src/scene.rs:2032-2033` (`decouple_parent`), `:2566` (`decouple_parent`'s `scene_transforms_` materialization) | cited `planning_scene.cpp:1260-1264` | CONFIRMED | opened both occurrences — described content (`has_value()` check, `emplace`, `setAllTransforms`) is fully inside `:1260-1264`; true closing brace is `:1265`, 1 line past. | |
| `crates/moveit-scene/src/scene.rs:2884` (`frame_transform_prefers_the_attached_body_tier_over_a_same_named_world_object`) (test) | cited `planning_scene.cpp:2036` | CONFIRMED | exact — anchors only the function start for an ordering claim between tier 2 and tier 5, correctly. This round: the `where` column previously cited scene.rs line 2854, inside the unrelated `frame_transform_falls_through_to_the_world_for_an_object_and_its_subframe` test (a `BTreeMap::insert` building that test's fixture, no tier-ordering discussion anywhere near it) — the comment that actually states the tier 2/tier 5 ordering and cites `:2036` is in the test the row's own label already names, re-derived to `scene.rs:2884`. | this round |

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
call, inside `assert_isometry_eq` (`tests/frame_transform_parity.rs:147-148`),
is `both` (`epsilon` and `max_relative` set); 0 `epsilon`-only or
neither-set sites here.

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

## Round 10 follow-up — `AttachedFrames for PlanningScene`

The seam `set_from_ik` uses to reach attached bodies had no production
implementor until this round; only `NoAttachedFrames` and a test double
existed. Every claim its doc and its tests make about upstream is
re-derived below from the pinned checkout
(`e017c91ee12984393a28ba246075c65f69cde3bf`), opened line by line.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-scene/src/scene.rs:2086-2090` | upstream `setFromIK` resolves its target frame through `getLinkModelIncludingAttachedBodies` (`robot_state.cpp:910-937`), reading the state's own `attached_body_map_` | CONFIRMED | `setFromIK` calls `getRigidlyConnectedParentLinkModel(pose_frame)` at `robot_state.cpp:1924` and `(solver_tip_frame)` at `:1931`; that overload (`:939-945`) opens with `getLinkModelIncludingAttachedBodies(frame)` at `:942`. `:910-937` is the whole of that function: link tier `:913-916`, attached-body-id tier `:919-923`, subframe tier `:926-933`, `nullptr` at `:936`. The three tiers and their order are what this impl's delegate reproduces. | this commit |
| `crates/moveit-scene/tests/attached_frames_reach_ik.rs:8-15` | the pose half comes from `getFrameTransform` (`:1930`, `:1937`), which reaches the same map through `getFrameInfo` (`:1338-1384`) | CONFIRMED | `setFromIK` takes `pose_parent_to_frame = getFrameTransform(pose_frame)` at `:1930` and `tip_parent_to_tip = getFrameTransform(solver_tip_frame)` at `:1937`. The const overload (`:1320-1336`) forwards to `getFrameInfo` at `:1324`. `getFrameInfo` spans `:1338-1384` with four tiers — model frame `:1345-1350`, link `:1351-1355`, attached-body id `:1359-1367`, subframe `:1370-1379` — one more than `getLinkModelIncludingAttachedBodies`, which has no model-frame tier. The port keeps that same asymmetry: `moveit-kinematics`' `frame_transform` tries the model frame first, `rigid_parent_link` does not. The header itself opens `RobotState::setFromIK` at `:8`. | this commit |
| `attached_frame` (`crates/moveit-scene/src/scene.rs:1297-1306`) (delegated to by the impl) | a bare attached-body id resolves at identity in its attach link's frame | DEVIATION, deliberate | Upstream's id tier returns `jt->second->getGlobalPose()` (`robot_state.cpp:1362`), a pose the body carries in `AttachedBody::pose_`; this port's `AttachedBody` has no such field (see its module doc) and the id names the attach link's own frame. The two agree exactly when `pose_` is identity, which is what `processAttachedCollisionObjectMsg`'s ADD branch produces for message-supplied shapes. `a_bare_attached_body_id_resolves_with_no_local_offset` pins the port side so the deviation cannot drift silently. | this commit |
| `crates/moveit-scene/src/scene.rs:2090-2094` | a `moveit-scene` -> `moveit-kinematics` dependency is legal; only the reverse would be a cycle | CONFIRMED | `crates/moveit-constraints/Cargo.toml` already depends on `moveit-kinematics`, and `moveit-scene` already depended on `moveit-constraints`, so the edge existed transitively before this round and adding it directly closes nothing new. `tools/ci/check-dep-direction.sh` passes; it bans ROS client libraries in core crates and says nothing about this pair; `:2090` itself names `moveit_kinematics::AttachedFrames`. | this commit |
