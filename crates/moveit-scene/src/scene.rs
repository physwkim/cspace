// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.hpp
//   moveit_core/planning_scene/src/planning_scene.cpp

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use moveit_collision::{
    Action, AllowedCollisionMatrix, AttachedBodyGeometry, BodyType, CollisionEnv, CollisionRequest,
    CollisionResult, Contact, CostSource, DistanceRequest, MoveObjectOutcome, Notification, World,
    remove_cost_sources, remove_overlapping,
};
use moveit_constraints::KinematicConstraintSet;
use moveit_error::{Error, Result};
use moveit_geometry::{Isometry3, Shape, Transforms};
use moveit_model::RobotModel;
use moveit_srdf::SrdfModel;
use moveit_state::{Posed, RobotState};

use crate::attached_body::AttachedBody;
use crate::layered::Layered;
use crate::world_diff::WorldDiff;

/// The result of [`PlanningScene::is_path_valid`]: overall validity plus
/// which waypoint indices failed. Upstream returns a plain `bool` and
/// writes indices into a caller-supplied `std::vector<std::size_t>*
/// invalid_index` out-param (`nullptr` meaning "don't bother, short-circuit
/// on the first failure instead") — this port always computes the full set
/// and returns both together, see [`PlanningScene::is_path_valid`]'s own
/// doc for why.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PathValidity {
    /// Whether every waypoint was valid (and, if checked, the goal was
    /// satisfied) — equivalent to `invalid_waypoints.is_empty()`.
    pub valid: bool,
    /// Indices into the `waypoints` slice that failed, in ascending order.
    pub invalid_waypoints: Vec<usize>,
}

/// The environment a planning instance reasons about: the world, the ACM,
/// the current [`RobotState`], attached bodies, and (for a diff scene) a
/// parent to fall back on. Upstream `planning_scene::PlanningScene`.
///
/// # Scope
///
/// Full audit of every *public* symbol in upstream
/// `planning_scene::PlanningScene`, read from `planning_scene.hpp` itself
/// (`planning_scene.h` is a deprecated `#pragma message`-only shim that
/// `#include`s the `.hpp` and declares nothing of its own — see
/// `create_deprecated_headers.py` in its header comment). One line each:
/// `ported as <symbol>` / `D1 excludes it (<message type>)` / `distinct
/// (<reason>)` / `unported (<reason>)`. Private/protected members
/// (`initialize`, the `CollisionDetector` struct, the process-*Add/Remove/Move
/// helpers, data fields) are implementation detail, not audited here.
///
/// Re-walked symbol-by-symbol this round (round 22) against the current
/// `planning_scene.hpp` (`public:` block, its current lines 93-926) and
/// against the current tree, not just the header: 60 audit bullets below,
/// every one landing in one of the four buckets above — zero `unported, in
/// scope` gaps survive the walk. Round 21 reclassified `getCostSources`
/// from `blocked` to `unported, in scope` once p3-acm's `6890fdd` closed
/// the backend gap that `blocked` verdict rested on, and flagged the
/// then-standing "zero gaps" claim as false as of that reclassification;
/// this round closes that last gap by porting `getCostSources` itself — see
/// its bullet below, now `ported`. Every
/// overload count already recorded here (`checkCollision`/`checkSelfCollision`/
/// `getCollidingLinks`/`getCollidingPairs` at 6 each, `distanceToCollision`/
/// `distanceToCollisionUnpadded`/`isStateConstrained`/`getCostSources` at 4
/// each, `isStateValid` at 5, `isPathValid` at 8) was reverified against a
/// fresh `grep -c` of the header and still holds. The one thing the walk
/// found this round: the deleted copy constructor and copy-assignment
/// operator (`PlanningScene(const PlanningScene&) = delete` /
/// `operator=(const PlanningScene&) = delete`, both textually `public:`)
/// had never had a bullet — added below, immediately after the two real
/// constructors.
///
/// A sibling panel's `setGroupStateValidityCallback` finding (PORTING-PLAN.md's
/// record of that round: a symbol read from the header as a
/// "sampler-side diagnostic" setter turned out, on reading the upstream
/// `.cpp`, to gate IK solution acceptance via `IKCallbackFn`) prompted a
/// re-check this round of every `distinct`/`unported`/`blocked` bullet
/// below that had been judged from `planning_scene.hpp`'s signature alone,
/// without a `planning_scene.cpp` citation backing it:
/// `getCollisionDetectorName`, the `getCollisionEnv`/`getCollisionEnvUnpadded`
/// family, `checkCollisionUnpadded`, `distanceToCollisionUnpadded`,
/// `setAttachedBodyUpdateCallback`, `setCollisionObjectUpdateCallback`,
/// `printKnownObjects`, and `allocateCollisionDetector` — each now carries
/// its own upstream-`.cpp` citation and verdict inline, below. None
/// reclassify: every branch a "distinct" symbol here participates in
/// either lives inside machinery D4 already replaced wholesale (the
/// collision-detector-plugin family) or is a `void`-returning notification
/// callback structurally incapable of gating a decision the way a
/// `bool`-returning one can — the distinguishing fact
/// `setGroupStateValidityCallback`'s own re-classification turned on.
///
/// Counting convention (so a future round can mechanically reproduce this
/// number): one bullet per raw `public:` declaration, with same-named
/// overloads folded into a single bullet carrying an explicit `(N
/// overloads)` count; a multi-line signature counts once, at its first
/// line; the destructor and the two `= delete`d copy special members each
/// get their own bullet (the deleted pair share one, since both express
/// the same "not copyable" idiom); an inline-body accessor (e.g.
/// `getName()`'s `{ return name_; }`) counts identically to a
/// declaration-only member — definition location doesn't matter, `public:`
/// visibility does; `using`/`typedef` aliases declared at namespace scope
/// above the class (`StateFeasibilityFn`, `MotionFeasibilityFn`,
/// `ObjectColorMap`, `ObjectTypeMap`) are types referenced by member
/// signatures, not members themselves, and are not counted.
///
/// The paragraph above was incomplete: mechanically reproducing it this
/// round (below) surfaced a second folding step it never named. Same-named
/// overload folding alone takes the header's 151 raw `public:` declarations
/// down to 85 distinct symbol names, not 60 — a further step, editorial
/// clustering of *different*-named declarations that share one purpose,
/// does the rest: `getName`/`setName` (a getter/setter pair), `OCTOMAP_NS`/
/// `DEFAULT_SCENE_NAME` (two constants introduced together), and the
/// six-symbol object-color and object-type accessor families each collapse
/// onto one bullet despite having no name in common. This step was already
/// in use throughout the list below; it just wasn't written down.
///
/// Reproduced this round (run from the `moveit2` checkout root):
///
/// ```text
/// $ python3 - << 'EOF'
/// import re
/// from collections import Counter
/// path = "moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.hpp"
/// lines = open(path).readlines()
/// block = lines[92:926]
/// text = "".join(block)
/// def strip_comments_keep_lines(s):
///     out, i, n = [], 0, len(s)
///     while i < n:
///         if s[i:i+2] == '/*':
///             j = s.find('*/', i+2); j = n if j == -1 else j+2
///             out.append('\n' * s.count('\n', i, j)); i = j
///         else:
///             out.append(s[i]); i += 1
///     return ''.join(out)
/// clean = strip_comments_keep_lines(text)
/// clean = '\n'.join(l[:l.find('//')] if '//' in l else l for l in clean.split('\n'))
/// depth, decls, body_stack, decl_start = 0, [], [], None
/// for lineno, raw in enumerate(clean.split('\n'), start=1):
///     if depth == 0 and raw.strip() and decl_start is None:
///         decl_start = lineno
///     for ch in raw:
///         if ch == '{':
///             depth += 1
///             if depth == 1:
///                 body_stack.append(decl_start or lineno)
///         elif ch == '}':
///             if depth == 1:
///                 decls.append((body_stack.pop(), lineno)); decl_start = None
///             depth -= 1
///         elif ch == ';' and depth == 0:
///             decls.append((decl_start or lineno, lineno)); decl_start = None
/// names = []
/// for s, e in decls:
///     snippet = re.sub(r'\s+', ' ', ' '.join(l.strip() for l in clean.split('\n')[s-1:e] if l.strip()))
///     if snippet.startswith('~'):
///         names.append(snippet[:snippet.index('(')].strip()); continue
///     if snippet.startswith('PlanningScene& operator='):
///         names.append('operator='); continue
///     if snippet.startswith('PlanningScene('):
///         names.append('PlanningScene(ctor)'); continue
///     m = re.match(r'static const std::string (\w+);', snippet)
///     if m:
///         names.append(m.group(1)); continue
///     idents = [g for g in re.findall(r'([A-Za-z_]\w*)\s*\(', snippet) if g != 'deprecated']
///     names.append(idents[0] if idents else '<static-member>')
/// print(f"raw declarations: {len(decls)}, distinct names: {len(set(names))}")
/// for name, cnt in sorted(Counter(names).items(), key=lambda x: (-x[1], x[0])):
///     if cnt > 1:
///         print(f"  {name}: {cnt} overloads")
/// EOF
/// raw declarations: 151, distinct names: 85
///   isPathValid: 8 overloads
///   checkCollision: 6 overloads
///   checkCollisionUnpadded: 6 overloads
///   checkSelfCollision: 6 overloads
///   getCollidingLinks: 6 overloads
///   getCollidingPairs: 6 overloads
///   isStateColliding: 5 overloads
///   isStateValid: 5 overloads
///   distanceToCollision: 4 overloads
///   distanceToCollisionUnpadded: 4 overloads
///   getCostSources: 4 overloads
///   getFrameTransform: 4 overloads
///   isStateConstrained: 4 overloads
///   PlanningScene(ctor): 2 overloads
///   diff: 2 overloads
///   getCollisionEnv: 2 overloads
///   getCollisionEnvUnpadded: 2 overloads
///   getPlanningSceneMsg: 2 overloads
///   getTransforms: 2 overloads
///   isStateFeasible: 2 overloads
///   knowsFrameTransform: 2 overloads
///   loadGeometryFromStream: 2 overloads
///   processOctomapMsg: 2 overloads
///   setCurrentState: 2 overloads
/// ```
///
/// Every overload count printed above matches the count already recorded
/// inline on its bullet below, and every one of the 85 distinct names is
/// referenced by exactly one of the 60 bullets — the header and this list
/// still agree.
///
/// ## Construction, identity, parent/child
///
/// - `PlanningScene(RobotModel, World)` / `PlanningScene(urdf, srdf, World)`
///   — ported as [`PlanningScene::new`]/[`PlanningScene::with_world`].
/// - `PlanningScene(const PlanningScene&) = delete`, `operator=(const
///   PlanningScene&) = delete` — not portable symbols: Rust structs are
///   neither `Copy` nor `Clone` unless derived, so implicit copy
///   construction and copy assignment are already absent by default: this
///   type derives neither, so both are already unavailable without any
///   explicit deletion to port.
/// - `OCTOMAP_NS`, `DEFAULT_SCENE_NAME` — D1 (octomap/message-round-trip
///   naming constants, unused without message handling).
/// - `~PlanningScene` — not a portable symbol; nothing here needs a
///   user-visible destructor.
/// - `getName`/`setName` — ported as [`PlanningScene::name`]/[`PlanningScene::set_name`].
/// - `diff() const` — ported as [`PlanningScene::diff`].
/// - `diff(moveit_msgs::msg::PlanningScene&)` — D1 (`moveit_msgs::msg::PlanningScene`).
/// - `getParent` — ported as [`PlanningScene::parent`].
/// - `clone` (static) — ported as [`PlanningScene::cloned`].
/// - `clearDiffs` — ported as [`PlanningScene::clear_diffs`].
/// - `pushDiffs` — ported as [`PlanningScene::push_diffs`].
/// - `decoupleParent` — ported as [`PlanningScene::decouple_parent`].
///
/// ## Robot model and state
///
/// - `getRobotModel` — ported as [`PlanningScene::robot_model`].
/// - `getCurrentState`/`getCurrentStateNonConst` — ported as
///   [`PlanningScene::current_state`]/[`PlanningScene::current_state_mut`].
/// - `getCurrentStateUpdated(moveit_msgs::msg::RobotState)` — D1
///   (`moveit_msgs::msg::RobotState`).
/// - `setCurrentState(moveit_msgs::msg::RobotState)` — D1 (`moveit_msgs::msg::RobotState`).
/// - `setCurrentState(RobotState)` — ported as [`PlanningScene::set_current_state`].
///
/// ## Frames
///
/// - `getPlanningFrame` — ported as [`PlanningScene::planning_frame`].
/// - `getTransforms`/`getTransformsNonConst` — ported as
///   [`PlanningScene::transforms`]/[`PlanningScene::transforms_mut`],
///   returning `&`/`&mut` [`moveit_geometry::Transforms`]. That type is a
///   pre-existing, already-tested port of the ROS-free core of upstream
///   `moveit_core/transforms` (`crates/moveit-geometry/src/transforms.rs`,
///   present since this workspace's first commits) -- an earlier revision
///   of this doc claimed no such crate existed anywhere in this workspace;
///   that claim was wrong, and this round found and corrected it rather
///   than building a second, duplicate implementation here. Upstream
///   actually declares three overloads here, not two: `getTransforms()
///   const` (`:184`), a non-const `getTransforms()` (`:197`, `.cpp:671`)
///   whose only addition is `getCurrentStateNonConst().update()` before
///   delegating to the const form, and `getTransformsNonConst()` (`:200`).
///   The middle one collapses into [`PlanningScene::transforms`] here, not
///   a second `&mut self` method: its state-refresh exists so a caller
///   holding the polymorphic `SceneTransforms&` gets fresh link/attached-
///   body transforms before querying it, and `moveit_geometry::Transforms`
///   as returned here never reads robot-state link transforms at all (see
///   [`PlanningScene::frame_transform`]'s tier split) -- so there is
///   nothing for a state refresh to keep fresh.
///   `setTransform(const Eigen::Isometry3d&, const std::string&)` (the
///   message-free overload -- `transforms.hpp:113`) and
///   `setAllTransforms`/`getAllTransforms` are ported as
///   [`moveit_geometry::Transforms::set_transform`]/
///   [`set_all_transforms`](moveit_geometry::Transforms::set_all_transforms)/
///   [`all_transforms`](moveit_geometry::Transforms::all_transforms) on that
///   type directly, reachable here via `getTransformsNonConst`'s mutable
///   accessor. `setTransform(TransformStamped)` and `copyTransforms` — D1
///   (`geometry_msgs::msg::TransformStamped`).
/// - `getFrameTransform` (id-only, and explicit-`RobotState` overloads) —
///   ported as [`PlanningScene::frame_transform`]; the explicit-state
///   overloads collapse into the self-state form the same way collision
///   checking's const/non-const pairs collapse below — upstream's own
///   id-only overload already delegates to the explicit-state one against
///   `getCurrentState()` (`planning_scene.cpp:2019`/`:2036`). Its third
///   tier, `getTransforms().Transforms::getTransform(frame_id)`
///   (`:2053`), is now [`PlanningScene::frame_transform`]'s own tier 6 --
///   see that method's doc for how the non-recursion upstream's explicit
///   `Transforms::` qualifier enforces is reproduced structurally here.
/// - `knowsFrameTransform` (id-only, and explicit-state) — ported as
///   [`PlanningScene::knows_frame_transform`]; same collapse
///   (`planning_scene.cpp:2056`/`:2061`); see that method's doc for the
///   `SceneTransforms::isFixedFrame` override, and
///   [`PlanningScene::transforms_with_world_objects`] (no upstream
///   equivalent -- an addition, not a port) for the value that reproduces
///   it for `moveit-constraints`'s callers.
///
/// ## World, collision detector, ACM
///
/// - `getCollisionDetectorName` — distinct: names the active
///   `CollisionDetectorAllocator` plugin; D4 has no plugin registry to name
///   (see `allocateCollisionDetector` below). Previously judged from the
///   header signature alone; checked against `planning_scene.cpp` this
///   round. Its only use in that file *is* a branch condition
///   (`planning_scene.cpp:291`/`:304`, inside `getCollisionEnv(name)`/
///   `getCollisionEnvUnpadded(name)`: `if (collision_detector_name !=
///   getCollisionDetectorName())` logs an error and falls back to the
///   active env) — but that branch lives entirely inside the
///   already-D4-obsoleted name-lookup overloads below, which this port
///   cannot reach a mismatch for (there is only ever one collision
///   backend), so the branch it gates is unreachable by construction here,
///   not merely unexercised by choice.
/// - `getWorld`/`getWorldNonConst` — ported as [`PlanningScene::world`]
///   (const); mutation goes through typed methods
///   ([`PlanningScene::add_shape`]/[`PlanningScene::move_object`]/
///   [`PlanningScene::remove_object`]/[`PlanningScene::remove_all_objects`])
///   rather than a raw `&mut World` accessor.
/// - `getCollisionEnv`/`getCollisionEnvUnpadded`/`getCollisionEnv(name)`/
///   `getCollisionEnvUnpadded(name)`/`getCollisionEnvNonConst` — distinct:
///   the padded/unpadded dual-`CollisionEnv`-per-plugin machinery; D4's
///   redesign replaced it with the caller supplying one concrete `E` per
///   call (see "Collision checking" below). Previously judged from the
///   header signature alone; checked against `planning_scene.cpp` this
///   round (`allocateCollisionDetector`, `:255-286`, and the name-lookup
///   branches at `:288-311` cited under `getCollisionDetectorName` above):
///   what these decide is *which allocator-constructed `CollisionEnv`
///   pointer* a caller gets back, a decision D4 already made once and
///   permanently by giving every `PlanningScene` exactly one
///   `E: CollisionEnv` supplied by the caller — there is no second backend
///   this branch could ever route to here.
/// - `getAllowedCollisionMatrix`/`getAllowedCollisionMatrixNonConst`/
///   `setAllowedCollisionMatrix` — ported as
///   [`PlanningScene::allowed_collision_matrix`]/
///   [`PlanningScene::allowed_collision_matrix_mut`]/
///   [`PlanningScene::set_allowed_collision_matrix`].
/// - `removeAllCollisionObjects` — ported as [`PlanningScene::remove_all_objects`].
///
/// ## Collision checking
///
/// See also the "Collision checking" section below for the shared `E:
/// CollisionEnv` design this whole group is built on.
///
/// - `isStateColliding` (current-state and explicit-state overloads) —
///   ported as [`PlanningScene::is_state_colliding`] (see its own doc); the
///   `moveit_msgs::msg::RobotState` overload — D1.
/// - `checkCollision` (6 overloads, not 7 — recounted this round) — ported as
///   [`PlanningScene::check_collision`]; the explicit-different-ACM
///   overloads are not ported — a caller wanting a one-off ACM already has
///   `env.check_collision(request, &posed, &attached, Some(&other_acm))`
///   directly.
/// - `checkCollisionUnpadded` (6 overloads, upstream `[[deprecated]]`) —
///   distinct: the same padded/unpadded dual-env machinery as
///   `getCollisionEnvUnpadded` above; D4 obsoletes it. Same
///   `getCollisionEnvUnpadded()` route already checked against
///   `planning_scene.cpp` under `getCollisionEnv` above — no separate
///   branch of its own.
/// - `checkSelfCollision` (6 overloads) — ported as
///   [`PlanningScene::check_self_collision`]; explicit-ACM overloads not
///   ported, same reasoning as `checkCollision`.
/// - (no upstream equivalent) — [`PlanningScene::check_robot_collision`] is
///   an addition, not a port: upstream's `checkCollision` family has no
///   robot-vs-world-only entry point at the `PlanningScene` level (see its
///   own doc).
/// - `getCollidingLinks` (6 overloads, not 5 — recounted this round) —
///   ported as [`PlanningScene::colliding_links`]; explicit-ACM overloads
///   not ported, same reasoning.
/// - `getCollidingPairs` (6 overloads, not 5, four with `group_name` not
///   one — recounted this round) — ported as
///   [`PlanningScene::colliding_pairs`], `group_name` folded in as
///   `Option<&str>` (D13, see its own doc); explicit-ACM overloads not
///   ported, same reasoning.
///
/// ## Distance
///
/// - `distanceToCollision` (4 overloads) — ported as
///   [`PlanningScene::distance_to_collision`] (see its own doc);
///   explicit-ACM overloads not ported, same reasoning as `checkCollision`.
/// - `distanceToCollisionUnpadded` (4 overloads) — distinct: same
///   padded/unpadded machinery as `checkCollisionUnpadded`, D4 obsoletes it
///   (`planning_scene.hpp:553-609`: each overload only forwards to
///   `getCollisionEnvUnpadded()->distanceRobot(...)`, no branch of its own
///   beyond the const/non-const `updateCollisionBodyTransforms()` forward
///   already reproduced by `PlanningScene::distance_to_collision`'s own
///   `&mut`/`&` split).
///
/// ## Message round-tripping (D1 except the first entry: this is a
/// ROS-independent core crate)
///
/// - `saveGeometryToStream`/`loadGeometryFromStream` — **distinct, decided,
///   not deferred**: round 7/8 (§59.4) left this open on "no consumer has
///   asked for `.scene` file interop", the same unmet-falsifier shape
///   `moveit-geometry`'s `shapes::saveAsText`/`constructShapeFromText`
///   deferral mirrors (`planning_scene.cpp:1062`/`:1152` is this format's
///   shape payload). A falsifier only two panels can satisfy by each
///   waiting on the other's silence never closes on its own, so this round
///   answers the real question instead: does this port intend to support
///   loading a user-authored scene file, or round-tripping a scene through
///   disk, at all? No. Every real upstream caller of this pair (searched
///   with `rg loadGeometryFromStream saveGeometryToStream` across the whole
///   `moveit2` tree) is `moveit_ros` tooling wrapped around a live ROS node,
///   not a plain file utility: `move_group`'s
///   `{load,save}_geometry_to_file_service_capability.cpp` (ROS service
///   handlers), `warehouse/{import_from_text,save_as_text}.cpp` (database
///   import/export against a running warehouse node), the RViz Motion
///   Planning panel's Scene tab, and `planning_components_tools/
///   publish_scene_from_text.cpp` — which reads a `.scene` file only to
///   `rclcpp::Node::make_shared` a node and publish it as a
///   `moveit_msgs::msg::PlanningScene`, not to do anything with the geometry
///   itself outside ROS. `moveit_py`'s binding is the same shape one layer
///   removed (`moveit_py` is its own out-of-scope rewrite, `PORTING-PLAN.md`
///   §4.7). The read/write functions themselves are ROS-free — `istream`/
///   `ostream`, no `moveit_msgs` — but every reason anything upstream has to
///   call them is ROS-coupled, and `moveit-ros` (the one crate that could
///   ever carry ROS-coupled tooling, `PORTING-PLAN.md` §0/§4.7) has no crate
///   yet and no plan naming this workflow. This is a positive scope
///   statement, not an absence of demand: `.scene` file interop is out of
///   scope because every real reason to want it is out of scope, the same
///   way `getObjectColor`'s `std_msgs::msg::ColorRGBA` typing was a storage
///   choice and not this format's real gate (§59.4 already established that
///   half). Both deferrals fall together on this answer.
/// - `getPlanningSceneDiffMsg`/`getPlanningSceneMsg` (2 overloads) — D1
///   (`moveit_msgs::msg::PlanningScene`, `moveit_msgs::msg::PlanningSceneComponents`).
/// - `getCollisionObjectMsg`/`getCollisionObjectMsgs` — D1
///   (`moveit_msgs::msg::CollisionObject`).
/// - `getAttachedCollisionObjectMsg`/`getAttachedCollisionObjectMsgs` — D1
///   (`moveit_msgs::msg::AttachedCollisionObject`).
/// - `getOctomapMsg` — D1 (`octomap_msgs::msg::OctomapWithPose`).
/// - `getObjectColorMsgs` — D1 (`moveit_msgs::msg::ObjectColor`; also part
///   of the object-color family below).
/// - `setPlanningSceneDiffMsg`/`setPlanningSceneMsg`/`usePlanningSceneMsg`
///   — D1 (`moveit_msgs::msg::PlanningScene`).
/// - `shapesAndPosesFromCollisionObjectMessage` — D1
///   (`moveit_msgs::msg::CollisionObject`).
/// - `processCollisionObjectMsg` — D1 at the symbol
///   (`moveit_msgs::msg::CollisionObject`), but its ADD/MOVE/REMOVE
///   branches are ported natively as [`PlanningScene::add_shape`]/
///   [`PlanningScene::move_object`]/[`PlanningScene::remove_object`].
/// - `processAttachedCollisionObjectMsg` — D1 at the symbol
///   (`moveit_msgs::msg::AttachedCollisionObject`), but its ADD/REMOVE
///   branches are ported natively as [`PlanningScene::attach`]/
///   [`PlanningScene::attach_new`]/[`PlanningScene::detach`].
/// - `processPlanningSceneWorldMsg` — D1 (`moveit_msgs::msg::PlanningSceneWorld`).
/// - `processOctomapMsg` (2 overloads) — D1
///   (`octomap_msgs::msg::OctomapWithPose`, `octomap_msgs::msg::Octomap`).
/// - `processOctomapPtr` — D1-adjacent: takes a raw `octomap::OcTree`
///   pointer rather than a message directly, but exists solely to serve the
///   octomap message-processing path above; no octomap handling exists
///   anywhere in this port.
///
/// ## World-change callbacks
///
/// - `setAttachedBodyUpdateCallback` — distinct: a plain-Rust-representable
///   callback hook (unlike a message type, nothing forces its exclusion),
///   but nothing in this port's reach ever registers one — the same "no
///   caller reaches the branch that would need it" reasoning
///   [`PlanningScene::is_state_valid`]'s own doc gives for the dropped
///   `StateFeasibilityFn` (see "Feasibility predicates" below). Previously
///   backed only by that analogy, not a direct read; checked this round
///   against `attached_body.hpp:52` and every invocation site in
///   `planning_scene.cpp` (`:630`, `:643`, `:1229`, `:1270`, `:1554`).
///   `AttachedBodyCallback` is `void(AttachedBody*, bool)` — a `void`
///   return, structurally incapable of gating anything the way
///   p1-robotmodel's `setGroupStateValidityCallback` finding did (§105.1:
///   that callback's `bool` return is fed straight into `IKCallbackFn` as
///   an accept/reject decision). This one is a pure notification hook by
///   its type, not merely by absence of a registered caller.
/// - `setCollisionObjectUpdateCallback` — same reasoning, and additionally
///   structurally obsoleted: [`moveit_collision::World`] replaced upstream's
///   `addObserver`/`ObserverCallbackFn` registration with every mutator
///   returning `Option<Notification>` directly, so there is no observer
///   slot left to wire a callback into. Same check applied: `world.hpp:304`
///   declares `ObserverCallbackFn` as `void(const ObjectConstPtr&, Action)`
///   — again `void`-returning, confirmed at its call sites
///   (`planning_scene.cpp:220`, `:326`, `:650`, `:655`).
///
/// ## Object colors and types
///
/// - `hasObjectColor`/`getObjectColor`/`getOriginalObjectColor`/
///   `setObjectColor`/`removeObjectColor`/`getKnownObjectColors` — D1
///   (`std_msgs::msg::ColorRGBA`).
/// - `hasObjectType`/`getObjectType`/`setObjectType`/`removeObjectType`/
///   `getKnownObjectTypes` — D1 (`object_recognition_msgs::msg::ObjectType`).
///
/// ## Feasibility predicates
///
/// - `setStateFeasibilityPredicate`/`getStateFeasibilityPredicate` —
///   distinct: the `StateFeasibilityFn` accessor pair; no field exists to
///   accessor since nothing registers a predicate (see `isStateValid`'s "No
///   feasibility step" doc).
/// - `isStateFeasible` (`moveit_msgs::msg::RobotState` and `RobotState`
///   overloads) — the `RobotState` overload is not ported as its own
///   symbol: with no predicate ever registered, its only reachable branch
///   is the unconditional `true` (`planning_scene.cpp:2227-2243`), which
///   [`PlanningScene::is_state_valid`] takes by omitting the step outright
///   (see its own "No feasibility step" doc). The message overload — D1
///   (`moveit_msgs::msg::RobotState`).
/// - `setMotionFeasibilityPredicate`/`getMotionFeasibilityPredicate` —
///   distinct: the `MotionFeasibilityFn` accessor pair, for a field
///   (`motion_feasibility_`) confirmed by reading `planning_scene.cpp` in
///   full to be stored but never *read* anywhere upstream either — not even
///   by `isPathValid`. Dead code in the reference implementation itself,
///   not merely unreachable from this port.
///
/// ## State and path validity
///
/// - `isStateConstrained` (4 overloads) — ported as
///   [`PlanningScene::is_state_constrained`] (`RobotState` +
///   `KinematicConstraintSet` form, see its own doc); the 3
///   `moveit_msgs`-involving overloads — D1.
/// - `isStateValid` (5 overloads) — ported as [`PlanningScene::is_state_valid`]
///   (`RobotState` + `KinematicConstraintSet` form); the 4
///   `moveit_msgs`-involving overloads — D1.
/// - `isPathValid` (8 overloads) — ported as [`PlanningScene::is_path_valid`]
///   (see its own doc); 7 overloads carry a `moveit_msgs` type somewhere in
///   their signature — D1. The remaining message-free overload
///   (`robot_trajectory::RobotTrajectory` only, no constraints) is not a D1
///   exclusion: it depends on `robot_trajectory::RobotTrajectory`, owned by
///   `moveit-trajectory`/p6-totg, not a message type. This port's
///   `is_path_valid` takes `&[RobotState<'m>]` instead — a deliberate
///   dependency-boundary choice (avoids a dependency edge on a crate this
///   one does not own, for a type whose only relevant content here is its
///   ordered waypoint sequence), not a gap.
///
/// ## Cost sources and diagnostics
///
/// - `getCostSources` (all four overloads) — ported this round as
///   [`PlanningScene::cost_sources`] (the `state`-taking pair,
///   `planning_scene.cpp:2493-2510`) and [`PlanningScene::path_cost_sources`]
///   (the `trajectory`-taking pair, `planning_scene.cpp:2451-2491`), each
///   collapsing its own group_name-defaulting overload into one method the
///   same way [`PlanningScene::is_state_valid`]/[`PlanningScene::is_path_valid`]
///   already do (`group_name: Option<&str>`, `None` for upstream's default
///   `std::string()`). The `trajectory`-taking overloads take
///   `&[RobotState<'m>]`, not a `moveit_trajectory::RobotTrajectory` —
///   [`PlanningScene::is_path_valid`]'s own already-documented
///   dependency-boundary choice, not a new one. See both methods' own doc
///   comments for the per-line body citation and the load-bearing
///   `remove_cost_sources`-then-`remove_overlapping` call order.
///   Previously `blocked` (p3-acm's `6890fdd`, merged to `main` via
///   `044a788f`, replaced `moveit_collision::ParryCollisionEnv`'s hardcoded
///   `cost_sources: None` with real computation — round 21 reclassified to
///   `unported, in scope` once that landed).
/// - `printKnownObjects` — distinct: `std::ostream` debug formatting with
///   no algorithmic content; everything it prints is already public via
///   [`PlanningScene::world`]'s `object_ids` and
///   [`PlanningScene::attached_bodies`]. Previously judged from the header
///   signature alone; its body (`planning_scene.cpp:2512-2533`) is exactly
///   that — two loops over `getWorld()->getObjectIds()` and
///   `getCurrentState().getAttachedBodies(...)` writing `<<` to `out`, no
///   conditional whose outcome differs from what those two accessors
///   already return.
/// - `allocateCollisionDetector` — distinct: registers a
///   `CollisionDetectorAllocator` plugin; D4's redesign already replaced
///   that dual-env-per-plugin machinery with the caller owning one concrete
///   `E`, so this type carries no `collision_detector_` field for it to act
///   on. Previously judged from the header signature alone; checked its
///   body (`planning_scene.cpp:255-286`) this round — the only branch
///   inside it (`if (parent_detector)`, copy-construct vs. build-fresh) is
///   the same clone/fresh-construct split
///   [`PlanningScene::cloned`]/[`PlanningScene::new`] already reproduce at
///   the type level, not a decision this port drops.
///
/// # Collision checking
///
/// [`PlanningScene::check_collision`]/[`PlanningScene::check_self_collision`]/
/// [`PlanningScene::check_robot_collision`]/[`PlanningScene::distance_to_collision`]/
/// [`PlanningScene::colliding_pairs`]/[`PlanningScene::colliding_links`] are
/// generic over a caller-supplied `E: CollisionEnv<Posed<'_, 'm>>` (in
/// practice [`moveit_collision::ParryCollisionEnv`]) rather than owning one —
/// upstream's `PlanningScene` owns *two* (`getCollisionEnv`/
/// `getCollisionEnvUnpadded`, one per `CollisionDetectorAllocator` plugin),
/// switched per check by `CollisionRequest::pad_environment_collisions`/
/// `pad_self_collisions` — two flags this port does not carry (see
/// [`moveit_collision::CollisionRequest`]). D4's compile-time-registry
/// redesign replaces that plugin selection with the caller choosing (and
/// owning) a concrete `E` directly, so every method below applies whatever
/// padding `E` itself was built with (see `ParryCollisionEnv`'s own doc) to
/// both the self- and robot-collision checks alike.
///
/// Those two upstream environments differ in nothing but their padding:
/// `planning_scene.cpp:275-276` builds both from the same `world_` and the
/// same `getRobotModel()`, and `cenv_unpadded_` is never handed to
/// `setPadding`/`setScale`/`copyPadding` afterwards (`:249-252`, `:365-366`,
/// `:1348-1349`, `:1386-1387` all name `cenv_` alone). "Unpadded" therefore
/// means exactly `LinkPaddingScale::default()`, so a caller needing
/// upstream's unpadded environment builds `let mut u = env.clone();
/// *u.padding_scale_mut() = LinkPaddingScale::default();` — a clone that
/// shares the world's `Arc<Object>` contents and the octree cache.
///
/// **Deviation — which half the padding reaches.** Upstream's two defaults
/// are asymmetric: `pad_environment_collisions` is `true` and
/// `pad_self_collisions` is `false` (`collision_common.hpp:154`, `:157`),
/// and nothing in the whole `moveit2` tree ever assigns the second. Its
/// effective rule is thus "robot-vs-world padded, self never padded", while
/// [`PlanningScene::check_collision`] has one `E` and pads both halves — so
/// with a padded `E` its self half can report a collision upstream would
/// not. This is latent, not live: no caller in this workspace hands a
/// non-default [`moveit_collision::LinkPaddingScale`] to a [`PlanningScene`]
/// method (the two that build one call
/// [`moveit_collision::CollisionEnv::check_robot_collision`] directly,
/// mirroring upstream's own `test_collision_common_panda.hpp:215-233`), and
/// upstream's one `move_group`-reachable unpadded caller,
/// `PlanExecution::isRemainingPathValid` (`plan_execution.cpp:268-353`), is
/// not ported. Both sides of the boundary are pinned in
/// `moveit-scene/tests/padding_reaches_the_scenes_self_half.rs`. Re-open when
/// a caller does need upstream's unpadded self half: the fix is for that
/// caller to pass the unpadded clone above, not for this type to take a
/// second `E`.
///
/// Every check/distance method also passes `self`'s
/// [`PlanningScene::attached_bodies`] to `env` as
/// [`moveit_collision::AttachedBodyGeometry`] borrows. This follows upstream
/// `CollisionEnvFCL::constructFCLObjectRobot`, which folds
/// `state.getAttachedBodies()` into the *same* `FCLObject` used for
/// self-collision, robot-vs-world collision, and both distance queries —
/// attached geometry is part of what "the robot" means to every one of
/// these checks, not an optional extra passed to some and not others.
///
/// Every method here also collapses upstream's separate const/non-const
/// overload pairs (`checkCollision(state) const` vs `checkCollision(state)`,
/// the latter calling `updateCollisionBodyTransforms()` first) into one
/// `&mut self` method that always calls [`RobotState::update`] — `update`
/// already no-ops when nothing is dirty (see its own doc comment), so this
/// reproduces both overloads' observable behavior without a separate
/// already-clean fast path to maintain.
///
/// [`PlanningScene::check_collision`] delegates to
/// [`moveit_collision::CollisionEnv::check_collision`]'s existing default
/// (self-collision first, then robot-collision with the *remaining* contact
/// budget, merged) rather than upstream's own `PlanningScene::checkCollision`
/// body, which checks robot-collision first and returns early once
/// `res.contacts.size() >= req.max_contacts` (a *pair* count, not a total
/// contact count) — a different order and a different early-exit condition.
/// This is a deliberate deviation, not an oversight: `check_collision`'s
/// default already has its own boundary tests (`max_contacts: 0` must not
/// suppress the collision flag) and is the one piece of budget-subtraction
/// logic this port maintains for every backend, so this method consumes it
/// rather than re-deriving upstream's dual-env order on top.
///
/// `E`'s own [`World`] (`ParryCollisionEnv::world`) is a value the caller
/// constructs, not a live view of [`PlanningScene::world`]: unlike
/// upstream's `CollisionEnv`, which upstream's own `PlanningScene`
/// constructs internally and keeps in sync via `World`'s observer callback
/// (`notifyObjectChange`), this crate's `E` is handed in by the caller at
/// every call. A caller wanting `E` to see this scene's world passes
/// `env: &ParryCollisionEnv::new(scene.world().clone(), padding_scale)` — a
/// cheap call thanks to [`World`]'s own copy-on-write [`Clone`] — and must
/// re-clone after any world mutation the caller wants reflected.
///
/// # The parent/child design
///
/// Upstream's child scene holds a `PlanningSceneConstPtr parent_`
/// (`shared_ptr` to a *const* view of the same object some other, mutable
/// `PlanningScenePtr` may still be updating) and every accessor for a
/// `std::optional`-backed field falls through to the parent when the
/// child's own optional is empty. Two things are true about that design at
/// once: it is genuinely *live* — a mutation the parent's owner makes after
/// the child was created is visible through the child's next read, for as
/// long as the child has not diverged — and it is the "implicit value plus
/// re-derived-every-read fallthrough" shape this project treats as a defect
/// source (see this crate's private `layered` module doc).
///
/// This port keeps the live fallthrough — the crate-private `Layered<T>`
/// exists to make that safe rather than to remove it, see its own doc — but
/// deliberately does **not** keep upstream's *mutable-parent-aliasing*
/// half: `parent` here is `Arc<PlanningScene<'m>>`, an immutable snapshot
/// captured at [`PlanningScene::diff`] time. If the scene that was diffed
/// is later mutated through some other handle, the child does **not**
/// observe it. That is a real, deliberate semantic deviation, not a gap:
/// reproducing upstream's version faithfully would need `Arc<Mutex<...>>`
/// (or `Rc<RefCell<...>>`) for every layered field, i.e. shared mutable
/// state visible through a nominally-`const` pointer — exactly the
/// aliased-mutability hazard Rust's ownership model exists to rule out, for
/// a usage pattern (a long-lived parent scene mutated *after* a child was
/// handed out, with the child expected to observe it) that is not the
/// primary way `PlanningScene::diff` is used in practice: the common
/// pattern is diff → read-only planning against the frozen child →
/// [`PlanningScene::push_diffs`] the result back, not concurrent mutation of
/// both. [`PlanningScene::decouple_parent`] remains the explicit,
/// documented way to freeze a child's inherited state and stop tracking a
/// parent at all — it does the same materialization this deviation would
/// otherwise force at `diff()` time, just deferred until the caller asks
/// for it.
pub struct PlanningScene<'m> {
    name: String,
    parent: Option<Arc<PlanningScene<'m>>>,
    robot_model: &'m RobotModel,
    robot_state: Layered<RobotState<'m>>,
    world: World,
    /// `Some` only for a diff (child) scene — upstream `world_diff_`,
    /// `nullptr unless this is a diff scene`.
    world_diff: Option<WorldDiff>,
    acm: Layered<AllowedCollisionMatrix>,
    /// The extra-fixed-frame map: this scene's own, or the parent's.
    /// Upstream `scene_transforms_`, a `SceneTransformsPtr` reset in
    /// `clearDiffs()` (`planning_scene.cpp:331`) exactly like
    /// [`PlanningScene::current_state`] and
    /// [`PlanningScene::allowed_collision_matrix`] above, so it gets the
    /// same [`Layered`] treatment.
    transforms: Layered<Transforms>,
    /// Never layered: every scene, root or child, owns its own copy,
    /// seeded by cloning the parent's at [`PlanningScene::diff`] time — the
    /// same "always owned, seeded by clone" treatment upstream gives
    /// `world_` itself. See [`AttachedBody`]'s module doc for why this
    /// state lives here at all instead of on [`RobotState`].
    attached_bodies: BTreeMap<String, AttachedBody>,
}

impl<'m> PlanningScene<'m> {
    /// A root scene over an empty [`World`], with a [`RobotState`] at its
    /// default values and an ACM built from `srdf`. Upstream's
    /// `PlanningScene(robot_model, world)` constructor with `world`
    /// defaulted, followed by `initialize()`'s state/ACM setup.
    pub fn new(robot_model: &'m RobotModel, srdf: &SrdfModel) -> Self {
        Self::with_world(robot_model, srdf, World::new())
    }

    /// Like [`PlanningScene::new`], but starting from `world` instead of an
    /// empty one. Upstream `PlanningScene(robot_model, world)`.
    pub fn with_world(robot_model: &'m RobotModel, srdf: &SrdfModel, world: World) -> Self {
        let mut robot_state = RobotState::new(robot_model);
        robot_state.set_to_default_values();
        let transforms = Transforms::new(robot_model.model_frame())
            .expect("RobotModel::model_frame is non-empty by construction");
        Self {
            name: String::new(),
            parent: None,
            robot_model,
            robot_state: Layered::Own(robot_state),
            world,
            world_diff: None,
            acm: Layered::Own(AllowedCollisionMatrix::from_srdf(srdf)),
            transforms: Layered::Own(transforms),
            attached_bodies: BTreeMap::new(),
        }
    }

    /// This scene's name. Empty by default. Upstream `getName`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Set this scene's name. Upstream `setName`.
    pub fn set_name(&mut self, name: impl Into<String>) {
        self.name = name.into();
    }

    /// The parent this scene diffs against, if it is a diff scene. Upstream
    /// `getParent` — see the type doc for why this is an immutable
    /// snapshot, not upstream's live-aliased const view.
    pub fn parent(&self) -> Option<&PlanningScene<'m>> {
        self.parent.as_deref()
    }

    /// The robot model this scene was built for. Upstream `getRobotModel`.
    pub fn robot_model(&self) -> &'m RobotModel {
        self.robot_model
    }

    // ---- current state --------------------------------------------------

    /// The state the robot is assumed to be in: this scene's own, or (via
    /// `Layered::resolve`) the parent's. Upstream `getCurrentState`.
    pub fn current_state(&self) -> &RobotState<'m> {
        self.robot_state.resolve(|| {
            self.parent
                .as_ref()
                .expect("Layered::Inherited robot_state requires a parent scene")
                .current_state()
        })
    }

    /// Mutable access to the current state, materializing this scene's own
    /// copy (cloned from the resolved value) first if it was inherited.
    /// Upstream `getCurrentStateNonConst`.
    pub fn current_state_mut(&mut self) -> &mut RobotState<'m> {
        if !self.robot_state.is_own() {
            let cloned = self.current_state().clone();
            self.robot_state = Layered::Own(cloned);
        }
        match &mut self.robot_state {
            Layered::Own(state) => state,
            Layered::Inherited => unreachable!("just materialized above"),
        }
    }

    /// Replace the current state outright. Upstream `setCurrentState`.
    pub fn set_current_state(&mut self, state: RobotState<'m>) {
        self.robot_state = Layered::Own(state);
    }

    // ---- allowed collision matrix ----------------------------------------

    /// This scene's ACM: its own, or the parent's. Upstream
    /// `getAllowedCollisionMatrix`.
    pub fn allowed_collision_matrix(&self) -> &AllowedCollisionMatrix {
        self.acm.resolve(|| {
            self.parent
                .as_ref()
                .expect("Layered::Inherited acm requires a parent scene")
                .allowed_collision_matrix()
        })
    }

    /// Mutable access to the ACM, materializing this scene's own copy first
    /// if it was inherited. Upstream `getAllowedCollisionMatrixNonConst`.
    pub fn allowed_collision_matrix_mut(&mut self) -> &mut AllowedCollisionMatrix {
        if !self.acm.is_own() {
            let cloned = self.allowed_collision_matrix().clone();
            self.acm = Layered::Own(cloned);
        }
        match &mut self.acm {
            Layered::Own(acm) => acm,
            Layered::Inherited => unreachable!("just materialized above"),
        }
    }

    /// Replace the ACM outright. Upstream `setAllowedCollisionMatrix`.
    pub fn set_allowed_collision_matrix(&mut self, acm: AllowedCollisionMatrix) {
        self.acm = Layered::Own(acm);
    }

    // ---- transforms ---------------------------------------------------

    /// The extra-fixed-frame map: this scene's own, or the parent's.
    /// Upstream `getTransforms`.
    pub fn transforms(&self) -> &Transforms {
        self.transforms.resolve(|| {
            self.parent
                .as_ref()
                .expect("Layered::Inherited transforms requires a parent scene")
                .transforms()
        })
    }

    /// Mutable access to the extra-fixed-frame map, materializing this
    /// scene's own copy first if it was inherited. Upstream
    /// `getTransformsNonConst`.
    pub fn transforms_mut(&mut self) -> &mut Transforms {
        if !self.transforms.is_own() {
            let cloned = self.transforms().clone();
            self.transforms = Layered::Own(cloned);
        }
        match &mut self.transforms {
            Layered::Own(transforms) => transforms,
            Layered::Inherited => unreachable!("just materialized above"),
        }
    }

    /// The frame every [`PlanningScene::transforms`] entry maps into, and
    /// the frame [`PlanningScene::frame_transform`] resolves everything
    /// against. Upstream `getPlanningFrame`, which returns the target frame
    /// the scene's `SceneTransforms` was constructed with -- always the
    /// robot model's frame, because `SceneTransforms`'s own constructor
    /// hardcodes `scene->getRobotModel()->getModelFrame()`
    /// (`planning_scene.cpp:114`) regardless of which of its 4 call sites
    /// (`:192, 686, 1263, 1333`) constructs it.
    pub fn planning_frame(&self) -> &str {
        self.transforms().target_frame()
    }

    /// A one-time [`moveit_geometry::Transforms`] snapshot: this scene's own
    /// [`PlanningScene::transforms`] map, plus an entry for every current
    /// world object and object subframe. The value a caller building a
    /// constraint against this scene (`moveit_constraints::PositionConstraint::new`
    /// and its `Orientation`/`Visibility` siblings) should pass in place of
    /// [`PlanningScene::transforms`] alone -- see that method's doc for why
    /// the plain map is not enough on its own.
    ///
    /// # Why a snapshot, not a live view
    ///
    /// Upstream's `SceneTransforms::isFixedFrame` (`planning_scene.cpp:123`)
    /// overrides the base class to also answer `true` for a world object or
    /// object subframe (leading `/` stripped), via
    /// `knowsObjectFrame -> getWorld()->knowsTransform`. That override is
    /// reachable because `kinematic_constraint.cpp`'s three
    /// `configure(msg, tf)` methods (`PositionConstraint`/
    /// `OrientationConstraint`/`VisibilityConstraint`, four `isFixedFrame`
    /// call sites total -- see [`PlanningScene::knows_frame_transform`]'s
    /// doc) receive `tf` as a plain `Transforms&` that is *actually* a
    /// `SceneTransforms&` underneath, so `tf.isFixedFrame(...)` dispatches
    /// polymorphically to the override.
    ///
    /// [`moveit_geometry::Transforms`] has no such polymorphism -- it is one
    /// concrete, non-virtual type everywhere in this workspace, and
    /// `moveit-constraints`'s ported `PositionConstraint`/`OrientationConstraint`/
    /// `VisibilityConstraint` all key their fixed/mobile split on its
    /// [`moveit_geometry::Transforms::can_transform`], which -- being the
    /// same non-virtual method upstream's *base* `Transforms::isFixedFrame`
    /// is (see [`PlanningScene::knows_frame_transform`]'s doc) -- can only
    /// agree with the *scene-backed* override if the value handed to it
    /// already contains every name that override would say yes to. Hence a
    /// value, not a reference: this clones [`PlanningScene::transforms`]'s
    /// map and folds in [`World::iter`]'s current object and subframe poses
    /// once, at call time -- matching upstream's own timing, since
    /// `PositionConstraint::configure` (`resolve_frame`'s upstream sibling)
    /// resolves Fixed-vs-Mobile once, at construction, not on every
    /// `decide()` (see `moveit-constraints::position::resolve_frame`'s doc).
    /// A later change to this scene's world does not retroactively change
    /// what an already-taken snapshot answers, which is the same staleness
    /// upstream itself accepts by resolving once.
    ///
    /// Each object/subframe name is stored bare *and* `/`-prefixed, but the
    /// two are not symmetric: `isFixedFrame` checks the *unstripped* string
    /// against the base map first and only strips **one leading `/`** before
    /// the object check (`planning_scene.cpp:127-134`), so for a name `N`
    /// that does not itself start with `/`, `isFixedFrame(N)` and
    /// `isFixedFrame("/" + N)` are both `true`. [`moveit_geometry::Transforms::can_transform`]
    /// is one flat map lookup on the literal string it is given -- it cannot
    /// strip anything itself -- so matching both call shapes needs both keys
    /// present, which is what the bare-plus-`/`-prefixed insert below does.
    ///
    /// That insert must *not* run unconditionally on the object/subframe id
    /// itself, though: for an id `N` that already starts with `/` (a world
    /// object literally named e.g. `/obj`), the correct bare-`N` query is
    /// `isFixedFrame("/obj")`, which strips the query's own leading `/` and
    /// checks `knowsObjectFrame("obj")` -- **not** `knowsObjectFrame("/obj")`
    /// -- so it is `false` unless some other object is literally named
    /// `obj`. Only `isFixedFrame("//obj")` (one more leading `/`, stripped
    /// down to the literal id `/obj`) is `true`. Inserting the bare id
    /// unconditionally, as an earlier version of this method did, makes
    /// `can_transform("/obj")` wrongly agree with `isFixedFrame("/obj")`'s
    /// *stripped* meaning instead of disagreeing the way `knowsObjectFrame`
    /// actually does. `insert_object_frame` (below) encodes the fix: skip the bare
    /// insert whenever the name already starts with `/`; always insert the
    /// `/`-prefixed form.
    ///
    /// One further collision `knowsObjectFrame` resolves that a flat
    /// bare/`/`-prefixed insert does not, by itself: `World::knowsTransform`
    /// checks exact object ids *before* it ever considers the
    /// `object_id + "/" + subframe_name` subframe form
    /// (`world.cpp:145` runs before the subframe loop at `:150`) -- so a
    /// world object literally named e.g. `a/b` always wins over a different
    /// object `a` with a subframe `b`, even though both would otherwise
    /// insert the same composite key `a/b`. This method resolves that the
    /// same way: every object's own id is inserted first, and a subframe
    /// composite is skipped (via [`World::has_object`]) wherever it collides
    /// with another object's literal id. A subframe composite colliding with
    /// a *different* object's subframe composite (nested-`/` subframe or
    /// object names) is not modeled -- SRDF/`moveit_msgs::CollisionObject`
    /// ids and subframe names do not contain `/` in practice, and
    /// upstream's own resolution for that case depends on `std::map`
    /// iteration order among ambiguous prefixes, not just which composite
    /// strings exist.
    ///
    /// Robot-state link and attached-body names are deliberately excluded:
    /// `SceneTransforms::isFixedFrame` does not consult them either --
    /// `knowsObjectFrame` only ever calls `getWorld()->knowsTransform`, never
    /// `getCurrentState()`. (`SceneTransforms::canTransform`, a different
    /// override nothing in `kinematic_constraint.cpp` calls, does reach the
    /// robot state via `scene_->knowsFrameTransform` -- the two overrides are
    /// not the same set upstream either, so folding robot-state frames in
    /// here would over-match, not under-match.)
    pub fn transforms_with_world_objects(&self) -> Transforms {
        let mut snapshot = self.transforms().clone();

        // Object ids first, unconditionally -- `World::knowsTransform`
        // checks exact object ids before it ever considers a subframe
        // composite (`world.cpp:145`), so an object's own frame must win
        // any later collision with another object's subframe composite.
        for (id, object) in self.world.iter() {
            insert_object_frame(&mut snapshot, id, object.pose());
        }

        for (id, object) in self.world.iter() {
            for name in object.subframe_names() {
                let composite = format!("{id}/{name}");
                if self.world.has_object(&composite) {
                    continue;
                }
                if let Some(pose) = object.global_subframe_pose(name) {
                    insert_object_frame(&mut snapshot, &composite, pose);
                }
            }
        }
        snapshot
    }

    // ---- world ------------------------------------------------------------

    /// The world this scene sees. Upstream `getWorld`.
    pub fn world(&self) -> &World {
        &self.world
    }

    /// Feed a notification produced by a `self.world` mutation into
    /// `self.world_diff`, if this is a diff scene. The one funnel every
    /// world-mutating method below goes through, so "does this scene track
    /// a diff" is answered in exactly one place.
    fn track(&mut self, notification: Option<Notification>) {
        if let (Some(diff), Some(notification)) = (&mut self.world_diff, &notification) {
            diff.record(notification);
        }
    }

    fn track_all(&mut self, notifications: &[Notification]) {
        if let Some(diff) = &mut self.world_diff {
            diff.record_all(notifications);
        }
    }

    /// Add a single shape as a new (or augmented) world object. Upstream
    /// `World::addToObject`, reached through the scene so the change is
    /// tracked.
    pub fn add_shape(&mut self, id: &str, shape: Arc<Shape>, pose: Isometry3) {
        let notification = self.world.add_shape(id, shape, pose);
        self.track(notification);
    }

    /// Move an existing object by `transform` (world-frame composition).
    /// Upstream `World::moveObject`.
    pub fn move_object(&mut self, id: &str, transform: Isometry3) -> MoveObjectOutcome {
        let outcome = self.world.move_object(id, transform);
        if let MoveObjectOutcome::Moved(notification) = &outcome {
            self.track(Some(notification.clone()));
        }
        outcome
    }

    /// Remove a world object entirely. Upstream `processCollisionObjectRemove`:
    /// removes the object and — unlike attach, which never touches the
    /// ACM (see [`PlanningScene::attach`]'s doc) — prunes the ACM entry for
    /// `id` too, since this really is the object leaving the scene for
    /// good. Returns whether an object was actually removed.
    pub fn remove_object(&mut self, id: &str) -> bool {
        let Some(notification) = self.world.remove_object(id) else {
            return false;
        };
        self.track(Some(notification));
        self.allowed_collision_matrix_mut().remove_entries_for(id);
        true
    }

    /// Remove every world object. Upstream `removeAllCollisionObjects`,
    /// pruning each removed id's ACM entry the same way
    /// [`PlanningScene::remove_object`] does.
    pub fn remove_all_objects(&mut self) {
        let ids = self.world.object_ids();
        let notifications = self.world.clear_objects();
        self.track_all(&notifications);
        let acm = self.allowed_collision_matrix_mut();
        for id in &ids {
            acm.remove_entries_for(id);
        }
    }

    /// Replace the poses of an existing object's shapes, leaving the
    /// object's own pose untouched. Upstream `World::moveShapesInObject`,
    /// called inline from `processCollisionObjectMove`
    /// (planning_scene.cpp:2004) with no scene-level side effect around it
    /// beyond the raw world mutation — unlike [`PlanningScene::remove_object`]
    /// there is no ACM entry to prune here, since the object identity and
    /// its ACM entry are unaffected by moving its shapes.
    ///
    /// `false`, with no mutation, if `id` is unknown or `shape_poses.len()`
    /// does not match the object's current shape count — both collapse to
    /// the same upstream `false` (world.cpp:262-278), so unlike
    /// [`PlanningScene::move_object`] there is no second, "found but
    /// unchanged" case to distinguish: every failure here really is "no
    /// shapes moved," so a plain `bool` carries the same information a
    /// `MoveObjectOutcome`-style enum would.
    pub fn move_shapes_in_object(&mut self, id: &str, shape_poses: &[Isometry3]) -> bool {
        let Some(notification) = self.world.move_shapes_in_object(id, shape_poses) else {
            return false;
        };
        self.track(Some(notification));
        true
    }

    /// Replace an object's entire subframe map. Upstream
    /// `World::setSubframesOfObject`, called inline from several composite
    /// functions (`processCollisionObjectAdd`, `detachObject`,
    /// `decoupleParent`, scene-file loading — planning_scene.cpp:393,
    /// 1201, 1743, 1927) with no scene-level side effect of its own at any
    /// of those call sites: none of them touch the ACM, object color, or
    /// object type as a consequence of the subframe assignment itself.
    ///
    /// An empty map removes every subframe the object had. Produces no
    /// [`Notification`] — matching `World::set_subframes_of_object`'s own
    /// doc, upstream's `setSubframesOfObject` does not call `notify` at
    /// all (world.cpp:365-378), a real asymmetry with every other mutator
    /// in this section, not an oversight.
    ///
    /// `false`, with no mutation, if `id` is unknown.
    pub fn set_subframes_of_object(
        &mut self,
        id: &str,
        subframe_poses: BTreeMap<String, Isometry3>,
    ) -> bool {
        self.world.set_subframes_of_object(id, subframe_poses)
    }

    // ---- attached bodies ----------------------------------------------------

    /// Every attached body, by id.
    pub fn attached_bodies(&self) -> impl Iterator<Item = &AttachedBody> {
        self.attached_bodies.values()
    }

    /// The attached body named `id`, if any. Upstream
    /// `RobotState::getAttachedBody`.
    pub fn attached_body(&self, id: &str) -> Option<&AttachedBody> {
        self.attached_bodies.get(id)
    }

    /// Whether a body named `id` is currently attached. Upstream
    /// `RobotState::hasAttachedBody`.
    pub fn has_attached_body(&self, id: &str) -> bool {
        self.attached_bodies.contains_key(id)
    }

    /// Attach the world object `id` to `link_name`, removing it from the
    /// world. Upstream `processAttachedCollisionObjectMsg`'s ADD branch,
    /// restricted to the "object already exists in the world" case — see
    /// [`PlanningScene::attach_new`] for attaching geometry that is not
    /// already a world object.
    ///
    /// # Deviation from the task brief, confirmed against upstream
    ///
    /// Neither this method nor [`PlanningScene::detach`] touches the ACM.
    /// Reading `processAttachedCollisionObjectMsg` and
    /// `RobotState::attachBody` in full turned up no ACM mutation on
    /// attach/detach anywhere in `moveit_core` — upstream's own
    /// `UpdateACMAfterObjectRemoval` test (`test_planning_scene.cpp`) has a
    /// caller set an explicit ACM entry by hand (`acm.setEntry(object_name,
    /// hand_group_links, true)`) and then shows it survives an attach/detach
    /// round-trip untouched. The real, narrower invariant upstream protects
    /// — visible in `pushDiffs`' `"if object is attached, it should not be
    /// removed from the ACM"` guard — is that an id's ACM entry must survive
    /// as long as the id exists *either* in the world *or* as an attached
    /// body; only an outright deletion
    /// ([`PlanningScene::remove_object`]/[`PlanningScene::remove_all_objects`])
    /// may prune it. [`PlanningScene::push_diffs`] reproduces that exact
    /// guard.
    pub fn attach(
        &mut self,
        id: &str,
        link_name: &str,
        touch_links: BTreeSet<String>,
    ) -> Result<()> {
        if !self.robot_model.has_link_model(link_name) {
            return Err(Error::other(format!("no such link: {link_name}")));
        }
        let Some(object) = self.world.get_object(id) else {
            return Err(Error::other(format!(
                "attaching '{id}' requires it to already exist in the world; use attach_new for \
                 geometry that is not already a world object"
            )));
        };
        let link_transform = {
            let posed = self.current_state_mut().update();
            posed.global_link_transform(link_name)?
        };
        let object_pose = object.pose();
        let shapes: Vec<Arc<Shape>> = object
            .shapes()
            .iter()
            .map(|s| Arc::clone(s.shape()))
            .collect();
        let shape_poses: Vec<Isometry3> = object
            .shapes()
            .iter()
            .map(|s| link_transform.inverse() * object_pose * s.pose())
            .collect();
        // Same one-level composition as `shape_poses` above, for the
        // object's subframes -- upstream carries `obj_in_world->subframe_poses_`
        // (object-relative) into the new `AttachedBody` untouched
        // (`planning_scene.cpp:1590`) because its own two-level `pose_`
        // absorbs the link offset; this port has no `pose_` to absorb it
        // into (see `AttachedBody`'s module doc), so it is folded in here
        // instead.
        let subframes: BTreeMap<String, Isometry3> = object
            .subframe_names()
            .map(|name| {
                let pose = object
                    .subframe_pose(name)
                    .expect("name was just listed by subframe_names");
                (
                    name.to_owned(),
                    link_transform.inverse() * object_pose * pose,
                )
            })
            .collect();

        let notification = self.world.remove_object(id);
        self.track(notification);

        self.attached_bodies.insert(
            id.to_owned(),
            AttachedBody::new(
                id.to_owned(),
                link_name.to_owned(),
                shapes,
                shape_poses,
                touch_links,
                subframes,
            ),
        );
        Ok(())
    }

    /// Attach geometry that is not already a world object. Upstream
    /// `processAttachedCollisionObjectMsg`'s ADD branch, message-shapes
    /// case. `shape_poses`/`subframes` are relative to `link_name`'s own
    /// frame (see [`AttachedBody`]'s module doc).
    pub fn attach_new(
        &mut self,
        id: &str,
        link_name: &str,
        shapes: Vec<Arc<Shape>>,
        shape_poses: Vec<Isometry3>,
        touch_links: BTreeSet<String>,
        subframes: BTreeMap<String, Isometry3>,
    ) -> Result<()> {
        if !self.robot_model.has_link_model(link_name) {
            return Err(Error::other(format!("no such link: {link_name}")));
        }
        if shapes.is_empty() || shapes.len() != shape_poses.len() {
            return Err(Error::other(
                "attach_new requires at least one shape, with one pose per shape",
            ));
        }
        self.attached_bodies.insert(
            id.to_owned(),
            AttachedBody::new(
                id.to_owned(),
                link_name.to_owned(),
                shapes,
                shape_poses,
                touch_links,
                subframes,
            ),
        );
        Ok(())
    }

    /// Detach `id`, adding its geometry back to the world at its current
    /// global pose. Upstream `processAttachedCollisionObjectMsg`'s REMOVE
    /// (detach) branch. See [`PlanningScene::attach`]'s doc for why this
    /// does not touch the ACM.
    ///
    /// Errors, leaving the body still attached, if the world already has an
    /// object named `id` — upstream instead warns and silently drops the
    /// detached geometry rather than overwrite it; this port surfaces that
    /// as an error instead of a silent geometry loss.
    pub fn detach(&mut self, id: &str) -> Result<AttachedBody> {
        let Some(body) = self.attached_bodies.get(id) else {
            return Err(Error::other(format!("no attached body named '{id}'")));
        };
        if self.world.has_object(id) {
            return Err(Error::other(format!(
                "cannot detach '{id}': the world already has an object with that name"
            )));
        }
        let link_name = body.link_name().to_owned();
        let link_transform = {
            let posed = self.current_state_mut().update();
            posed.global_link_transform(&link_name)?
        };
        let body = self
            .attached_bodies
            .remove(id)
            .expect("just confirmed present above");
        let notification =
            self.world
                .add_to_object(id, link_transform, body.shapes(), body.shape_poses());
        self.track(notification);
        // `body`'s subframes are already relative to `link_name` (see the
        // module doc), and the object we just created is posed at exactly
        // `link_transform` -- the same "no transform needed" case
        // `add_to_object` above relies on for shapes. Upstream:
        // `world_->setSubframesOfObject` right after `addToObject`
        // (`planning_scene.cpp:1743`), which produces no notification either
        // (`World::set_subframes_of_object`'s own doc).
        let subframes: BTreeMap<String, Isometry3> = body
            .subframe_names()
            .map(|name| {
                (
                    name.to_owned(),
                    body.subframe_pose(name)
                        .expect("name was just listed by subframe_names"),
                )
            })
            .collect();
        self.world.set_subframes_of_object(id, subframes);
        Ok(body)
    }

    // ---- frames -------------------------------------------------------------

    /// Which attached body `frame_id` names, if any, as `(link_name, pose
    /// local to that link)` -- identity for a bare attached-body id
    /// (upstream `AttachedBody::getGlobalPose`'s `pose_`, which this port's
    /// one-level design has no field for — see [`AttachedBody`]'s module
    /// doc), the stored subframe pose for `"<id>/<subframe>"` otherwise.
    /// Looked up before [`PlanningScene::frame_transform`] poses the state,
    /// so this immutable borrow of [`PlanningScene::attached_bodies`] ends
    /// before [`PlanningScene::current_state_mut`]'s exclusive one begins —
    /// same shape as [`PlanningScene::attached_body_snapshot`].
    fn attached_frame(&self, frame_id: &str) -> Option<(&str, Isometry3)> {
        if let Some(body) = self.attached_bodies.get(frame_id) {
            return Some((body.link_name(), Isometry3::identity()));
        }
        self.attached_bodies.values().find_map(|body| {
            let suffix = frame_id.strip_prefix(body.id())?.strip_prefix('/')?;
            let pose = body.subframe_pose(suffix)?;
            Some((body.link_name(), pose))
        })
    }

    /// `getFrameTransform`: the global transform to `frame_id`, upstream
    /// `planning_scene.cpp:2036`'s ladder --
    ///
    /// 1. a leading `/` is stripped
    /// 2. the model frame, or a link name -- [`Posed::frame_transform`]
    ///    (upstream folds this and tiers 3-4 into one `RobotState::getFrameInfo`
    ///    call; this port's attached bodies live on [`PlanningScene`], not
    ///    [`moveit_state::RobotState`] — see [`AttachedBody`]'s module doc —
    ///    so tiers 3-4 are this method's own work instead)
    /// 3. an attached-body id -- that body's global pose (its attach link's
    ///    global transform; see the private `attached_frame` helper for why
    ///    there is no separate body-local offset to compose in here)
    /// 4. an attached-body subframe (`"<id>/<subframe>"`) -- that subframe's
    ///    global pose
    /// 5. a world object id or object subframe -- [`World::get_transform`]
    /// 6. the extra-fixed-frame map -- [`PlanningScene::transforms`],
    ///    [`moveit_geometry::Transforms::transform`]
    ///
    /// Tier 6 closes the "no extra-fixed-frame tier" deviation §59.1/§59.2
    /// found: upstream falls through to `Transforms::getTransform`
    /// (`planning_scene.cpp:2053`, the base class -- not the
    /// `SceneTransforms::getTransform` override tiers 3-4 above delegate to)
    /// as a final resort, and [`moveit_geometry::Transforms`] already ported
    /// that exact base class (`crates/moveit-geometry/src/transforms.rs`,
    /// present since this workspace's very first commits -- §59's "no crate
    /// exists" claim was wrong, missed by both the brief and this audit; see
    /// [`PlanningScene::transforms`]'s doc). No polymorphism is needed to
    /// call it non-recursively the way upstream's explicit `Transforms::`
    /// qualification is: this method's own tier 6 call is a plain field
    /// method call on [`moveit_geometry::Transforms`], which has no path
    /// back into [`PlanningScene::frame_transform`] to recurse through --
    /// the non-recursion upstream enforces with a qualifier is structural
    /// here, by construction, not by convention.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `frame_id` resolves in no tier.
    pub fn frame_transform(&mut self, frame_id: &str) -> Result<Isometry3> {
        let frame_id = frame_id.strip_prefix('/').unwrap_or(frame_id);
        let attached = self
            .attached_frame(frame_id)
            .map(|(link_name, pose)| (link_name.to_owned(), pose));

        let posed = self.current_state_mut().update();
        if let Ok(transform) = posed.frame_transform(frame_id) {
            return Ok(transform);
        }
        if let Some((link_name, local_pose)) = &attached {
            let link_transform = posed.global_link_transform(link_name)?;
            return Ok(link_transform * local_pose);
        }

        if let Ok(transform) = self.world.get_transform(frame_id) {
            return Ok(transform);
        }

        self.transforms().transform(frame_id).copied()
    }

    /// `knowsFrameTransform`: whether [`PlanningScene::frame_transform`]
    /// would resolve `frame_id`, without computing a fresh transform (a pure
    /// name lookup — needs no [`PlanningScene::current_state_mut`], unlike
    /// `frame_transform` itself). Upstream `planning_scene.cpp:2061`, the
    /// same tiers 1-5, tier 6 (the extra-fixed-frame map) excluded for the
    /// same reason -- see `frame_transform`'s own doc.
    ///
    /// # The model frame is checked directly, not through tier 1's `RobotState`
    ///
    /// [`moveit_state::RobotState::knows_frame_transform`] does not
    /// special-case the model frame (see its own doc) -- confirmed against
    /// upstream `RobotState::knowsFrameTransform`
    /// (`robot_state.cpp:1386-1405`), which really only checks
    /// `hasLinkModel`/attached bodies. Naively porting `PlanningScene::
    /// knowsFrameTransform` as "tier 1's `RobotState`, then attached bodies,
    /// then the world" would therefore report `false` for a model frame that
    /// is not itself a link name (true for panda's floating virtual joint --
    /// `model_frame() == "world"`, not `"panda_link0"`) even though
    /// [`PlanningScene::frame_transform`] resolves the same name via
    /// [`moveit_state::Posed::frame_transform`]'s own model-frame check.
    ///
    /// Upstream does not have that gap: its `PlanningScene::
    /// knowsFrameTransform` reaches `true` for the model frame anyway, but
    /// through the *extra-fixed-frame tier* -- `SceneTransforms`'s base
    /// `Transforms` constructor seeds `transforms_map_[target_frame_] =
    /// Identity()` with `target_frame_` set to the model frame
    /// (`planning_scene.cpp`'s `SceneTransforms` ctor forwards
    /// `getRobotModel()->getModelFrame()`), so the otherwise-empty map
    /// always trivially "knows" its own target frame. This is not a guess:
    /// it was confirmed live against the oracle's `frame_transform` op
    /// (`knows_transform: true` for `"world"` on panda, with no attached
    /// bodies or world objects registered at all) after a naive port
    /// without this tier returned `false` for the same request.
    ///
    /// [`PlanningScene::transforms`] closes this gap for real: its
    /// [`moveit_geometry::Transforms::new`] seeds the identity entry for the
    /// model frame exactly as `SceneTransforms`'s constructor does, so
    /// `self.transforms().can_transform(frame_id)` reaches `true` for the
    /// model frame through the same mechanism upstream uses, not a
    /// restated special case. The old `frame_id == model_frame` check this
    /// doc used to justify is now redundant and removed.
    ///
    /// # `SceneTransforms::isFixedFrame`'s leading-`/` and object-frame
    /// delegation: not ported, and here is why it still does not fire
    ///
    /// Upstream `SceneTransforms` also overrides `isFixedFrame`
    /// (`planning_scene.cpp:123-135`) to strip a leading `/` and consult
    /// `knowsObjectFrame` before falling back to the base class. That
    /// override exists so that code holding a bare `moveit::core::
    /// Transforms&` -- not knowing it is actually scene-backed -- still
    /// gets scene-aware answers. All of `moveit_core`'s `isFixedFrame`
    /// callers (`rg isFixedFrame`) are the four sites in
    /// `kinematic_constraint.cpp` (`:382`, `:622`, `:848`, `:861`, inside
    /// `PositionConstraint`/`OrientationConstraint`/`VisibilityConstraint::
    /// configure`), always `tf.isFixedFrame(header.frame_id)` deciding
    /// whether a constraint's reference frame is resolved once now (fixed)
    /// or re-resolved through robot state on every `decide()` (mobile).
    ///
    /// Round 7 called that crate wholesale-unported; it is not, as of round 9
    /// -- `moveit-constraints::{PositionConstraint, OrientationConstraint,
    /// VisibilityConstraint}` all exist and each reproduces this exact
    /// fixed/mobile split (`position::resolve_frame`,
    /// `orientation.rs:209`, `visibility.rs:79`), each keyed on
    /// `tf.can_transform(frame_id)` -- the base-class half of `isFixedFrame`
    /// this crate's [`moveit_geometry::Transforms`] already carries. So the
    /// falsifier's premise updated, but as of round 9 its answer had not
    /// flipped: no call site anywhere in this workspace threaded a
    /// [`PlanningScene`]-derived [`moveit_geometry::Transforms`] into any of
    /// the three constructors, so the world-object half of `isFixedFrame`
    /// had no live caller to diverge from upstream on.
    ///
    /// This round builds that side: [`PlanningScene::transforms_with_world_objects`]
    /// is the value to hand `PositionConstraint::new`/etc instead of
    /// [`PlanningScene::transforms`] -- see its own doc for why a snapshot
    /// value, not `self.transforms()` as-is, is what closes the gap. The
    /// falsifier now fires the moment `moveit-constraints`'s constructors are
    /// called with it in place of a bare `self.transforms()`; that wiring is
    /// `moveit-constraints`'s own call sites, not this crate's, so it stays
    /// on this list until that lands.
    pub fn knows_frame_transform(&self, frame_id: &str) -> bool {
        let frame_id = frame_id.strip_prefix('/').unwrap_or(frame_id);
        self.current_state().knows_frame_transform(frame_id)
            || self.attached_frame(frame_id).is_some()
            || self.world.knows_transform(frame_id)
            || self.transforms().can_transform(frame_id)
    }

    // ---- collision checking -----------------------------------------------
    //
    // See the type doc's "Collision checking" section for the overload
    // collapse (const/non-const, padded/unpadded env selection) and the
    // documented deviation in `check_collision`'s self-then-robot order.

    /// This scene's attached bodies, cloned out of the `attached_bodies`
    /// field so the resulting owned [`AttachedBody`] values do not keep
    /// `self` borrowed — [`PlanningScene::current_state_mut`] needs
    /// `&mut self` right after this, and each entry is cheap to clone
    /// ([`std::sync::Arc`] shapes, small `Vec`/`BTreeSet` poses and
    /// touch-links).
    fn attached_body_snapshot(&self) -> Vec<AttachedBody> {
        self.attached_bodies.values().cloned().collect()
    }

    /// Check `self` for both self- and robot-collision against `env`, using
    /// this scene's own [`PlanningScene::allowed_collision_matrix`] and
    /// [`PlanningScene::attached_bodies`]. Upstream
    /// `checkCollision`/`checkCollisionUnpadded`, collapsed — see the type
    /// doc.
    pub fn check_collision<E>(&mut self, env: &E, request: &CollisionRequest) -> CollisionResult
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let acm = self.allowed_collision_matrix().clone();
        let attached_bodies = self.attached_body_snapshot();
        let posed = self.current_state_mut().update();
        let attached: Vec<AttachedBodyGeometry<'_>> = attached_bodies
            .iter()
            .map(AttachedBody::as_geometry)
            .collect();
        env.check_collision(request, &posed, &attached, Some(&acm))
    }

    /// Check `self` for self-collision only against `env`, including this
    /// scene's [`PlanningScene::attached_bodies`]. Upstream
    /// `checkSelfCollision`.
    pub fn check_self_collision<E>(
        &mut self,
        env: &E,
        request: &CollisionRequest,
    ) -> CollisionResult
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let acm = self.allowed_collision_matrix().clone();
        let attached_bodies = self.attached_body_snapshot();
        let posed = self.current_state_mut().update();
        let attached: Vec<AttachedBodyGeometry<'_>> = attached_bodies
            .iter()
            .map(AttachedBody::as_geometry)
            .collect();
        env.check_self_collision(request, &posed, &attached, Some(&acm))
    }

    /// Check `self` against the world only (no self-collision) against
    /// `env`. Upstream's `checkCollision` family has no standalone
    /// robot-vs-world-only entry point at the `PlanningScene` level (only
    /// the combined `checkCollision`); this exposes
    /// [`moveit_collision::CollisionEnv::check_robot_collision`] directly
    /// through the scene's own state/ACM for a caller that wants that half
    /// alone, e.g. to build [`PlanningScene::colliding_pairs`]-style
    /// diagnostics without paying for a self-collision pass.
    pub fn check_robot_collision<E>(
        &mut self,
        env: &E,
        request: &CollisionRequest,
    ) -> CollisionResult
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let acm = self.allowed_collision_matrix().clone();
        let attached_bodies = self.attached_body_snapshot();
        let posed = self.current_state_mut().update();
        let attached: Vec<AttachedBodyGeometry<'_>> = attached_bodies
            .iter()
            .map(AttachedBody::as_geometry)
            .collect();
        env.check_robot_collision(request, &posed, &attached, Some(&acm))
    }

    /// The distance between the robot at `self`'s current state and the
    /// nearest world collision, ignoring self-collisions. Upstream
    /// `distanceToCollision`/`distanceToCollisionUnpadded`, collapsed (see
    /// the type doc) — always against this scene's own
    /// [`PlanningScene::allowed_collision_matrix`], matching every upstream
    /// overload that does not take an explicit different `acm` (the ones
    /// that do are not ported: a caller wanting a one-off ACM already has
    /// `env.distance_robot` directly, with `DistanceRequest { acm:
    /// Some(&other_acm), .. }`).
    pub fn distance_to_collision<E>(&mut self, env: &E) -> f64
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let acm = self.allowed_collision_matrix().clone();
        let attached_bodies = self.attached_body_snapshot();
        let posed = self.current_state_mut().update();
        let attached: Vec<AttachedBodyGeometry<'_>> = attached_bodies
            .iter()
            .map(AttachedBody::as_geometry)
            .collect();
        let request = DistanceRequest {
            acm: Some(&acm),
            ..Default::default()
        };
        env.distance_robot(&request, &posed, &attached)
            .minimum_distance
            .distance
    }

    /// Every colliding pair (self- and robot-collision alike) for `self`'s
    /// current state against `env`, keyed the same way
    /// [`moveit_collision::ContactData::by_pair`] is. Upstream
    /// `getCollidingPairs`, its four `group_name`-taking overloads
    /// (`planning_scene.hpp:492-495`) folded into this one method the way
    /// [`PlanningScene::cost_sources`]/[`PlanningScene::is_state_valid`]
    /// already collapse their own group_name-defaulting overload pairs —
    /// `group_name: None` is upstream's default-constructed `std::string()`
    /// (D13, `PORTING-PLAN.md` §197).
    ///
    /// # Deviation from upstream
    ///
    /// `req.max_contacts` is upstream's
    /// `getLinkModelsWithCollisionGeometry().size() + 1`; this port's
    /// [`RobotModel`] has no such query (see `moveit-model::robot_model`'s
    /// doc), so this uses every link with a non-empty
    /// [`moveit_model::LinkModel::shapes`] instead — a superset of links
    /// that actually convert to collision geometry (a link could still hold
    /// only [`Shape::OcTree`]/a degenerate [`Shape::Plane`], see `parry`'s
    /// module doc deviations 9–10), so this can only make the budget larger
    /// than upstream's, never smaller — the ceiling this exists to avoid
    /// hitting is never hit early.
    pub fn colliding_pairs<E>(
        &mut self,
        env: &E,
        group_name: Option<&str>,
    ) -> BTreeMap<(String, String), Vec<Contact>>
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let max_contacts = self
            .robot_model
            .link_models()
            .iter()
            .filter(|link| !link.shapes().is_empty())
            .count()
            + 1;
        let request = CollisionRequest {
            group_name: group_name.map(str::to_string),
            contacts: true,
            max_contacts,
            max_contacts_per_pair: 1,
            ..Default::default()
        };
        self.check_collision(env, &request)
            .contacts
            .map(|contacts| contacts.by_pair)
            .unwrap_or_default()
    }

    /// Every robot link involved in a collision for `self`'s current state
    /// against `env`. Upstream `getCollidingLinks`.
    pub fn colliding_links<E>(&mut self, env: &E) -> Vec<String>
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let mut links = Vec::new();
        for contacts in self.colliding_pairs(env, None).values() {
            for contact in contacts {
                if contact.body_type_1 == BodyType::RobotLink {
                    links.push(contact.body_name_1.clone());
                }
                if contact.body_type_2 == BodyType::RobotLink {
                    links.push(contact.body_name_2.clone());
                }
            }
        }
        links
    }

    // ---- state / path validity --------------------------------------------

    /// Whether `self`'s current state satisfies `constraints`. Upstream
    /// `isStateConstrained(state, KinematicConstraintSet, verbose)`
    /// (`planning_scene.cpp:2277`) — upstream's `moveit_msgs::Constraints`
    /// overload (`:2245`, message state, message constraints) delegates to
    /// a native-state/message-constraints overload (`:2253`) that builds a
    /// `KinematicConstraintSet` from the message and calls this exact
    /// method; that chain is not ported: D1 has no `moveit_msgs` type to
    /// build one from. (`:2269` is a distinct overload — message state,
    /// already-native `KinematicConstraintSet` — that converts only the
    /// state and never builds a constraint set from a message; it is not
    /// part of this chain.) `verbose`'s
    /// `RCLCPP_INFO` diagnostics are dropped for the same reason
    /// [`moveit_constraints::KinematicConstraintSet::decide`] itself
    /// carries no `verbose` parameter.
    pub fn is_state_constrained(&mut self, constraints: &KinematicConstraintSet) -> bool {
        let posed = self.current_state_mut().update();
        constraints.decide(&posed).satisfied
    }

    /// Whether `self`'s current state, checked against `env`, collides.
    /// Upstream `isStateColliding(state, group, verbose)`
    /// (`planning_scene.cpp:2217`) — the `moveit_msgs::RobotState` overload
    /// (`:2197`) is a construct-then-delegate wrapper and is not ported: D1
    /// has no `moveit_msgs` type to build one from. The current-state
    /// convenience overload (`isStateColliding(group, verbose)`, no
    /// explicit state) collapses into this one `&mut self` method the same
    /// way every other method here collapses upstream's const/non-const
    /// pairs (see the type doc's "Collision checking" section). `group`
    /// lives on `request.group_name` rather than as a separate parameter,
    /// matching how every other method here threads
    /// [`CollisionRequest`] through — upstream's own version takes no
    /// request at all, only a bare `group` string, and always builds its
    /// own local `CollisionRequest` internally (`planning_scene.cpp:2219`)
    /// to carry it. `verbose` is dropped for the same reason the rest of
    /// this family drops it: `CollisionRequest::verbose` is itself a
    /// stored-but-never-read field in this port (confirmed: no backend
    /// consults it), matching upstream's own `RCLCPP_INFO`-only use.
    pub fn is_state_colliding<E>(&mut self, env: &E, request: &CollisionRequest) -> bool
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        self.check_collision(env, request).collision
    }

    /// Whether `self`'s current state, checked against `env`, is free of
    /// collision and satisfies `constraints` (`None` means unconstrained).
    /// Upstream `isStateValid(state, KinematicConstraintSet, group,
    /// verbose)` (`planning_scene.cpp:2313`).
    ///
    /// # Ordering
    ///
    /// Collision is checked before constraints, matching upstream's own
    /// order exactly: a state that fails both is reported as a collision
    /// failure here, the same as upstream's short-circuiting `if
    /// (isStateColliding(...)) return false;` before it ever reaches
    /// `isStateConstrained` — this method calls
    /// [`PlanningScene::is_state_colliding`] rather than re-deriving that
    /// ordering inline.
    ///
    /// # No feasibility step
    ///
    /// Upstream's real overload also checks `isStateFeasible` — a
    /// caller-registered `StateFeasibilityFn` predicate — between collision
    /// and constraints. This port does not carry that predicate: unlike
    /// `moveit_msgs::Constraints` (excluded by D1, a ROS message type), a
    /// `Fn(&Posed) -> bool` predicate is a plain Rust concern and would be
    /// straightforward to store on its own. What it is not is exercised by
    /// anything in this port's reach: nothing here ever calls the upstream
    /// equivalent of `setStateFeasibilityPredicate`, so every caller of
    /// this method already takes upstream's "no predicate registered"
    /// branch, which is unconditionally `true`
    /// (`planning_scene.cpp:2227-2243`) — exactly the branch this port
    /// takes by omitting the step outright. Adding a stored `Arc<dyn
    /// Fn(...)>` field (with diff-scene inheritance semantics to match, see
    /// [`PlanningScene::diff`]'s own doc on what is and is not inherited)
    /// for a predicate no call site can register would be speculative
    /// configurability for a hypothetical future caller; one that actually
    /// needs it can add the field then, informed by that caller's real
    /// requirements.
    pub fn is_state_valid<E>(
        &mut self,
        env: &E,
        request: &CollisionRequest,
        constraints: Option<&KinematicConstraintSet>,
    ) -> bool
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        if self.is_state_colliding(env, request) {
            return false;
        }
        match constraints {
            Some(constraints) => self.is_state_constrained(constraints),
            None => true,
        }
    }

    /// Whether every waypoint in `waypoints` is
    /// [`PlanningScene::is_state_valid`] against `path_constraints`, and the
    /// *last* waypoint additionally satisfies at least one of
    /// `goal_constraints` (empty means no goal check). Upstream
    /// `isPathValid(RobotTrajectory, path_constraints, goal_constraints,
    /// group, verbose, invalid_index)` (`planning_scene.cpp:2365`), driven
    /// end to end (not re-derived) by `oracle.cpp`'s `is_state_valid` op,
    /// which is this method's ground truth. The other four `isPathValid`
    /// overloads (`moveit_msgs::RobotState`/`RobotTrajectory` message
    /// conversions) are not ported: D1, no `moveit_msgs` type to convert
    /// from.
    ///
    /// # No interpolation between waypoints
    ///
    /// Confirmed from upstream's own loop body (`planning_scene.cpp:2376-2422`):
    /// it only ever reads `trajectory.getWayPoint(i)` for `i` in
    /// `0..getWayPointCount()`. No state between two requested waypoints is
    /// ever constructed or checked, on either side of this port — a
    /// collision that only a continuous sweep between two individually
    /// collision-free waypoints would catch is invisible to `isPathValid`
    /// upstream, and to `is_path_valid` here too.
    ///
    /// # Every waypoint sees the same attached bodies
    ///
    /// This port's [`AttachedBody`] lives on `PlanningScene`, not
    /// per-`RobotState` the way upstream's does (see [`AttachedBody`]'s
    /// module doc) — `self`'s current attached-body set applies to every
    /// waypoint checked here, matching `oracle.cpp`'s `is_state_valid` op,
    /// which attaches once to the scene and copies that state (attached
    /// bodies included) into every waypoint it builds.
    ///
    /// `self`'s current state is restored before this returns, whatever the
    /// last waypoint checked was: unlike upstream (whose `isPathValid`
    /// never touches `current_state_` — every waypoint state is local to
    /// its loop), this port's only avenue to check an arbitrary state's
    /// collision is [`PlanningScene::current_state_mut`] (see
    /// [`PlanningScene::check_collision`]'s own doc), so each waypoint is
    /// installed as the current state in turn and the original is put back
    /// before returning.
    ///
    /// Invalid waypoint indices are collected, not short-circuited on the
    /// first failure: upstream's `invalid_index == nullptr` fast path is a
    /// performance choice with no observable difference in the returned
    /// bool, and a Rust caller of a method returning [`PathValidity`]
    /// always wants the full diagnostic. A waypoint that fails for state
    /// validity *and* (being last) an unmet goal contributes its index once
    /// here, not twice the way upstream's raw `invalid_index` vector can —
    /// see `oracle.cpp`'s own doc for why the oracle side deliberately does
    /// *not* make the same dedup, so a disagreement on how many times
    /// upstream pushes an index stays visible during development even
    /// though this method's own contract only promises a set of indices.
    pub fn is_path_valid<E>(
        &mut self,
        env: &E,
        request: &CollisionRequest,
        waypoints: &[RobotState<'m>],
        path_constraints: Option<&KinematicConstraintSet>,
        goal_constraints: &[KinematicConstraintSet],
    ) -> PathValidity
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let saved_state = self.current_state().clone();
        let mut invalid_waypoints = Vec::new();
        let waypoint_count = waypoints.len();
        for (i, waypoint) in waypoints.iter().enumerate() {
            self.set_current_state(waypoint.clone());
            let mut valid = self.is_state_valid(env, request, path_constraints);
            if i + 1 == waypoint_count && !goal_constraints.is_empty() {
                let satisfies_goal = goal_constraints
                    .iter()
                    .any(|goal| self.is_state_constrained(goal));
                if !satisfies_goal {
                    valid = false;
                }
            }
            if !valid {
                invalid_waypoints.push(i);
            }
        }
        self.set_current_state(saved_state);
        PathValidity {
            valid: invalid_waypoints.is_empty(),
            invalid_waypoints,
        }
    }

    /// Upstream's `getCostSources(state, max_costs, costs)`/`getCostSources(state,
    /// max_costs, group_name, costs)` (`planning_scene.cpp:2493-2510`),
    /// collapsed into one method the way [`PlanningScene::is_state_valid`]
    /// already collapses its own group_name-defaulting overload pair —
    /// `group_name: None` is upstream's default-constructed `std::string()`,
    /// matching [`CollisionRequest::group_name`]'s own `Option` convention.
    ///
    /// Unlike [`PlanningScene::path_cost_sources`], upstream's single-state
    /// overload does not call `removeCostSources`/`removeOverlapping` at
    /// all — confirmed from its body, which only ever does `checkCollision`
    /// then `cres.cost_sources.swap(costs)`. Returned in [`CostSource`]'s
    /// `Ord` order (most-costly-first, matching `std::set<CostSource>`'s own
    /// iteration order), truncated to `max_costs` by
    /// [`CollisionRequest::max_cost_sources`] at the collision-check layer.
    pub fn cost_sources<E>(
        &mut self,
        env: &E,
        group_name: Option<&str>,
        max_costs: usize,
    ) -> BTreeSet<CostSource>
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let request = CollisionRequest {
            group_name: group_name.map(str::to_string),
            cost: true,
            max_cost_sources: max_costs,
            ..Default::default()
        };
        self.check_collision(env, &request)
            .cost_sources
            .unwrap_or_default()
            .into_iter()
            .collect()
    }

    /// Upstream's `getCostSources(trajectory, max_costs, costs, overlap_fraction)`/
    /// `getCostSources(trajectory, max_costs, group_name, costs, overlap_fraction)`
    /// (`planning_scene.cpp:2451-2491`), collapsed the same way
    /// [`PlanningScene::cost_sources`] collapses its own pair.
    ///
    /// Ported faithfully from the body, not re-derived:
    /// - Every waypoint's [`PlanningScene::cost_sources`] (same `request`,
    ///   built once, reused for every `checkCollision` call — matching
    ///   upstream building `creq` once outside its loop) is unioned into the
    ///   running set (`cs.insert(cres.cost_sources.begin(), cres.cost_sources.end())`,
    ///   cpp:2472), regardless of waypoint index — including the first.
    /// - The *first* waypoint's own cost sources are captured a second time,
    ///   separately, as `cs_start` (`cs_start.swap(cres.cost_sources)`,
    ///   cpp:2474) — not removed from the union, just also kept aside.
    /// - The union is truncated to `max_costs`, keeping the first `max_costs`
    ///   entries in [`CostSource`]'s `Ord` (most-costly-first) order
    ///   (cpp:2477-2487) — matching [`PlanningScene::cost_sources`]'s own
    ///   truncation, but this time over the union across all waypoints
    ///   rather than one `checkCollision` call.
    /// - [`moveit_collision::remove_cost_sources`] runs first, against
    ///   `cs_start` (cpp:2489) — drops truncated-union entries that overlap
    ///   the *start* state's own cost sources by at least `overlap_fraction`
    ///   (see that function's own doc for the split-not-drop behavior on a
    ///   sub-threshold overlap, ported as-is).
    /// - [`moveit_collision::remove_overlapping`] runs second (cpp:2490),
    ///   deduplicating what survives against *itself*. This order is
    ///   load-bearing, not incidental: swapping it would let a source that
    ///   `remove_cost_sources` was about to drop first eliminate a mutually
    ///   overlapping sibling via `remove_overlapping`, changing which one
    ///   survives — upstream's two-call sequence is reproduced in the same
    ///   order here.
    ///
    /// Every waypoint is installed as [`PlanningScene::current_state_mut`]
    /// in turn and the original is restored before returning — the same
    /// mechanism and the same restore-before-return contract
    /// [`PlanningScene::is_path_valid`]'s own doc already documents, for the
    /// same reason (this port's only avenue to check an arbitrary state is
    /// the scene's current state; upstream's overload never touches
    /// `current_state_` at all, taking `trajectory.getWayPoint(i)` directly).
    pub fn path_cost_sources<E>(
        &mut self,
        env: &E,
        waypoints: &[RobotState<'m>],
        group_name: Option<&str>,
        max_costs: usize,
        overlap_fraction: f64,
    ) -> BTreeSet<CostSource>
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let saved_state = self.current_state().clone();
        let mut cs = BTreeSet::new();
        let mut cs_start = BTreeSet::new();
        for (i, waypoint) in waypoints.iter().enumerate() {
            self.set_current_state(waypoint.clone());
            let sources = self.cost_sources(env, group_name, max_costs);
            if i == 0 {
                cs_start = sources.clone();
            }
            cs.extend(sources);
        }
        self.set_current_state(saved_state);

        let costs: BTreeSet<CostSource> = if cs.len() <= max_costs {
            cs
        } else {
            cs.into_iter().take(max_costs).collect()
        };

        let costs = remove_cost_sources(&costs, &cs_start, overlap_fraction);
        remove_overlapping(&costs, overlap_fraction)
    }

    // ---- diff / decouple ----------------------------------------------------

    /// A new child scene that diffs against `self`: an empty
    /// [`WorldDiff`], a cloned [`World`] snapshot (cheap: see
    /// [`World`]'s own copy-on-write `Clone`), inherited
    /// transforms/state/ACM, and a cloned attached-body set. Upstream
    /// `diff()`.
    pub fn diff(self: &Arc<Self>) -> PlanningScene<'m> {
        PlanningScene {
            name: String::new(),
            parent: Some(Arc::clone(self)),
            robot_model: self.robot_model,
            robot_state: Layered::Inherited,
            world: self.world.clone(),
            world_diff: Some(WorldDiff::new()),
            acm: Layered::Inherited,
            transforms: Layered::Inherited,
            attached_bodies: self.attached_bodies.clone(),
        }
    }

    /// A standalone copy of `scene`, decoupled from any parent even if
    /// `scene` itself has one. Upstream static `PlanningScene::clone`.
    pub fn cloned(scene: &Arc<PlanningScene<'m>>) -> PlanningScene<'m> {
        let mut result = scene.diff();
        result.decouple_parent();
        result.name = scene.name.clone();
        result
    }

    /// If this scene has a parent, apply what changed here — the
    /// extra-fixed-frame map, current state, ACM, and world changes — onto
    /// `target`. A no-op if this scene has no parent. Upstream `pushDiffs`.
    ///
    /// The world-change replay preserves the one ACM subtlety upstream is
    /// careful about: an id whose only recorded action here is a *pure*
    /// [`Action::DESTROY`] (exact equality, not "contains `DESTROY`" — an id
    /// destroyed and then recreated within this diff, per
    /// [`WorldDiff::record`](crate::WorldDiff::record)'s
    /// coalescing, is *not* pure `DESTROY` and takes the other branch) has
    /// its `target` ACM entry pruned too — *unless* `target` already
    /// considers that id an attached body, exactly mirroring upstream's
    /// `if (!scene->getCurrentState().hasAttachedBody(it.first))` guard via
    /// [`PlanningScene::has_attached_body`].
    pub fn push_diffs(&self, target: &mut PlanningScene<'m>) {
        if self.parent.is_none() {
            return;
        }
        if let Layered::Own(transforms) = &self.transforms {
            let all = transforms.all_transforms().clone();
            target.transforms_mut().set_all_transforms(all);
        }
        if let Layered::Own(state) = &self.robot_state {
            target.set_current_state(state.clone());
        }
        if let Layered::Own(acm) = &self.acm {
            target.set_allowed_collision_matrix(acm.clone());
        }
        let Some(diff) = &self.world_diff else {
            return;
        };
        for (id, action) in diff.changes() {
            if *action == Action::DESTROY {
                let notification = target.world.remove_object(id);
                target.track(notification);
                if !target.has_attached_body(id) {
                    target.allowed_collision_matrix_mut().remove_entries_for(id);
                }
            } else if let Some(object) = self.world.get_object(id) {
                let notification = target.world.remove_object(id);
                target.track(notification);
                let shapes: Vec<Arc<Shape>> = object
                    .shapes()
                    .iter()
                    .map(|s| Arc::clone(s.shape()))
                    .collect();
                let shape_poses: Vec<Isometry3> =
                    object.shapes().iter().map(|s| s.pose()).collect();
                let notification =
                    target
                        .world
                        .add_to_object(id, object.pose(), &shapes, &shape_poses);
                target.track(notification);
                let subframes: BTreeMap<String, Isometry3> = object
                    .subframe_names()
                    .map(|name| {
                        (
                            name.to_owned(),
                            object
                                .subframe_pose(name)
                                .expect("name was just listed by subframe_names"),
                        )
                    })
                    .collect();
                target.world.set_subframes_of_object(id, subframes);
            }
        }
    }

    /// Materialize every inherited field locally, discard the world diff
    /// (nothing left to diff against), and drop the parent. A no-op if this
    /// scene has no parent. Upstream `decoupleParent`, scoped to the fields
    /// this port carries (`object_colors_`/`object_types_` are not ported —
    /// see the type's scope doc; `scene_transforms_` is layered like
    /// `robot_state_`/`acm_` and is materialized here alongside them,
    /// matching upstream `planning_scene.cpp:1260-1264`).
    pub fn decouple_parent(&mut self) {
        if self.parent.is_none() {
            return;
        }
        if !self.transforms.is_own() {
            let cloned = self.transforms().clone();
            self.transforms = Layered::Own(cloned);
        }
        if !self.robot_state.is_own() {
            let cloned = self.current_state().clone();
            self.robot_state = Layered::Own(cloned);
        }
        if !self.acm.is_own() {
            let cloned = self.allowed_collision_matrix().clone();
            self.acm = Layered::Own(cloned);
        }
        self.world_diff = None;
        self.parent = None;
    }

    /// Reset a diff scene back to a fresh diff against its parent's
    /// *current* state: any locally materialized `transforms`/`robot_state`/
    /// `acm`/`attached_bodies` are discarded and re-inherited, `world`
    /// reclones the parent's, and `world_diff` starts empty again. A no-op
    /// if this scene has no parent (upstream: `if (!parent_) return;`).
    /// Upstream `clearDiffs`, scoped to the fields this port carries — see
    /// [`PlanningScene::decouple_parent`]'s doc for which upstream fields
    /// that already excludes.
    ///
    /// The counterpart to [`PlanningScene::decouple_parent`]: that method
    /// freezes everything inherited in place and severs the parent link;
    /// this keeps the parent link and un-freezes everything back to it, so
    /// a diff scene that diverged while used for read-only planning can be
    /// handed back for reuse without `parent.diff()`-ing a brand new one.
    pub fn clear_diffs(&mut self) {
        let Some(parent) = self.parent.clone() else {
            return;
        };
        self.world = parent.world.clone();
        self.world_diff = Some(WorldDiff::new());
        self.transforms = Layered::Inherited;
        self.robot_state = Layered::Inherited;
        self.acm = Layered::Inherited;
        self.attached_bodies = parent.attached_bodies.clone();
    }
}

/// Insert `name`'s pose into `snapshot` under every key `isFixedFrame`'s
/// unstripped-then-one-leading-`/`-stripped check would resolve `name` to.
/// See [`PlanningScene::transforms_with_world_objects`]'s doc for why the
/// bare form is skipped when `name` itself already starts with `/`.
fn insert_object_frame(snapshot: &mut Transforms, name: &str, pose: Isometry3) {
    if !name.starts_with('/') {
        let _ = snapshot.set_transform(pose, name);
    }
    let _ = snapshot.set_transform(pose, format!("/{name}"));
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use moveit_collision::{AllowedCollisionType, LinkPaddingScale, ParryCollisionEnv};
    use moveit_constraints::{Constraint, JointConstraint};
    use moveit_geometry::Cuboid;
    use moveit_model::MeshSearchPaths;
    use moveit_srdf::SrdfModel;

    use super::*;

    // Fixture: a fixed-base robot with a shapeless `base` link and a `hand`
    // link one fixed joint away, so `attach`/`detach` have a real link to
    // attach to without needing a state to be posed first.
    const SRDF_XML: &str = r#"<robot name="test">
        <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
    </robot>"#;

    const URDF_XML: &str = r#"<robot name="test">
        <link name="base"/>
        <link name="hand"/>
        <joint name="hand_joint" type="fixed">
            <parent link="base"/>
            <child link="hand"/>
            <origin xyz="0 0 1"/>
        </joint>
    </robot>"#;

    fn build_model() -> RobotModel {
        let urdf = urdf_rs::read_from_string(URDF_XML).expect("test URDF must parse");
        let srdf = SrdfModel::parse_str(SRDF_XML).expect("test SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, URDF_XML, &srdf, &MeshSearchPaths::none())
            .expect("test fixture model must build")
    }

    fn srdf() -> SrdfModel {
        SrdfModel::parse_str(SRDF_XML).expect("test SRDF must parse")
    }

    fn cuboid_shape() -> Arc<Shape> {
        Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()))
    }

    // ---- diff: added vs removed vs moved world objects ---------------------

    #[test]
    fn diff_scene_records_an_added_world_object_without_touching_the_parent() {
        let model = build_model();
        let root = Arc::new(PlanningScene::new(&model, &srdf()));
        let mut child = root.diff();

        child.add_shape("box", cuboid_shape(), Isometry3::identity());

        let diff = child
            .world_diff
            .as_ref()
            .expect("child scene must track a diff");
        assert!(diff.get("box").unwrap().contains(Action::CREATE));
        assert!(diff.get("box").unwrap().contains(Action::ADD_SHAPE));
        assert_eq!(child.world().object_ids(), vec!["box".to_owned()]);
        assert!(root.world().object_ids().is_empty());
    }

    #[test]
    fn diff_scene_records_a_removed_world_object_without_touching_the_parent() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.add_shape("box", cuboid_shape(), Isometry3::identity());
        let root = Arc::new(root);
        let mut child = root.diff();

        assert!(child.remove_object("box"));

        let diff = child.world_diff.as_ref().unwrap();
        assert_eq!(diff.get("box").unwrap(), Action::DESTROY);
        assert!(child.world().object_ids().is_empty());
        assert_eq!(root.world().object_ids(), vec!["box".to_owned()]);
    }

    #[test]
    fn diff_scene_records_a_move_only_change_for_an_existing_object() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.add_shape("box", cuboid_shape(), Isometry3::identity());
        let root = Arc::new(root);
        let mut child = root.diff();

        let outcome = child.move_object("box", Isometry3::translation(1.0, 0.0, 0.0));
        assert!(matches!(outcome, MoveObjectOutcome::Moved(_)));

        let diff = child.world_diff.as_ref().unwrap();
        assert_eq!(diff.get("box").unwrap(), Action::MOVE_SHAPE);
    }

    // ---- attach/detach: the ACM round-trips exactly, because neither touches it ----

    #[test]
    fn attach_then_detach_round_trips_the_acm_exactly() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape("box", cuboid_shape(), Isometry3::identity());
        scene
            .allowed_collision_matrix_mut()
            .set_entry("box", "hand", true);
        let before = scene.allowed_collision_matrix().all_entry_names();

        scene.attach("box", "hand", BTreeSet::new()).unwrap();
        assert!(scene.has_attached_body("box"));
        assert!(!scene.world().has_object("box"));
        assert_eq!(scene.allowed_collision_matrix().all_entry_names(), before);

        scene.detach("box").unwrap();
        assert!(!scene.has_attached_body("box"));
        assert!(scene.world().has_object("box"));
        assert_eq!(scene.allowed_collision_matrix().all_entry_names(), before);
        assert_eq!(
            scene
                .allowed_collision_matrix()
                .entry("box", "hand")
                .unwrap()
                .kind(),
            AllowedCollisionType::Always
        );
    }

    #[test]
    fn attach_folds_the_world_objects_subframe_into_a_link_relative_pose() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape("box", cuboid_shape(), Isometry3::identity());
        let mut subframes = BTreeMap::new();
        subframes.insert("tip".to_owned(), Isometry3::translation(0.0, 0.0, 0.5));
        scene.world.set_subframes_of_object("box", subframes);

        scene.attach("box", "hand", BTreeSet::new()).unwrap();

        // "hand" sits at (0, 0, 1) relative to "base" (the fixture's fixed
        // joint origin) and "box" was at the world's identity pose, so the
        // link-relative subframe pose is `hand_global.inverse() * identity *
        // (0, 0, 0.5)` = `(0, 0, -0.5)`.
        let body = scene.attached_body("box").unwrap();
        assert_eq!(
            body.subframe_pose("tip"),
            Some(Isometry3::translation(0.0, 0.0, -0.5))
        );
    }

    #[test]
    fn detach_writes_the_attached_bodys_subframes_back_onto_the_world_object() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        let mut subframes = BTreeMap::new();
        subframes.insert("tip".to_owned(), Isometry3::translation(0.0, 0.0, 0.5));
        scene
            .attach_new(
                "box",
                "hand",
                vec![cuboid_shape()],
                vec![Isometry3::identity()],
                BTreeSet::new(),
                subframes,
            )
            .unwrap();

        scene.detach("box").unwrap();

        let object = scene.world().get_object("box").unwrap();
        assert_eq!(
            object.subframe_pose("tip"),
            Some(Isometry3::translation(0.0, 0.0, 0.5))
        );
    }

    #[test]
    fn remove_object_prunes_the_acm_entry_but_attach_leaves_it_alone() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape("box", cuboid_shape(), Isometry3::identity());
        scene
            .allowed_collision_matrix_mut()
            .set_entry("box", "hand", true);
        assert!(scene.allowed_collision_matrix().has_entry("box"));

        scene.remove_object("box");

        assert!(!scene.allowed_collision_matrix().has_entry("box"));
    }

    // ---- move_shapes_in_object / set_subframes_of_object ---------------------

    #[test]
    fn move_shapes_in_object_missing_object_is_false() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());

        assert!(!scene.move_shapes_in_object("box", &[Isometry3::identity()]));
    }

    #[test]
    fn move_shapes_in_object_shape_count_mismatch_is_false() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape("box", cuboid_shape(), Isometry3::identity());

        assert!(
            !scene.move_shapes_in_object("box", &[Isometry3::identity(), Isometry3::identity()])
        );
        assert_eq!(
            scene.world().get_object("box").unwrap().shapes()[0].pose(),
            Isometry3::identity()
        );
    }

    #[test]
    fn move_shapes_in_object_moves_the_shape_and_tracks_the_notification() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.add_shape("box", cuboid_shape(), Isometry3::identity());
        let root = Arc::new(root);
        let mut child = root.diff();

        let new_pose = Isometry3::translation(1.0, 0.0, 0.0);
        assert!(child.move_shapes_in_object("box", &[new_pose]));

        assert_eq!(
            child.world().get_object("box").unwrap().shapes()[0].pose(),
            new_pose
        );
        let diff = child.world_diff.as_ref().unwrap();
        assert_eq!(diff.get("box").unwrap(), Action::MOVE_SHAPE);
    }

    #[test]
    fn set_subframes_of_object_missing_object_is_false() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());

        assert!(!scene.set_subframes_of_object("box", BTreeMap::new()));
    }

    #[test]
    fn set_subframes_of_object_overwrites_existing_subframes_and_produces_no_notification() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.add_shape("box", cuboid_shape(), Isometry3::identity());
        let mut old_subframes = BTreeMap::new();
        old_subframes.insert("old".to_owned(), Isometry3::translation(0.0, 0.0, 0.5));
        assert!(root.set_subframes_of_object("box", old_subframes));
        let root = Arc::new(root);
        let mut child = root.diff();

        let mut new_subframes = BTreeMap::new();
        new_subframes.insert("new".to_owned(), Isometry3::translation(1.0, 0.0, 0.0));
        assert!(child.set_subframes_of_object("box", new_subframes));

        let object = child.world().get_object("box").unwrap();
        assert_eq!(object.subframe_pose("old"), None);
        assert_eq!(
            object.subframe_pose("new"),
            Some(Isometry3::translation(1.0, 0.0, 0.0))
        );
        // Upstream `setSubframesOfObject` never calls `notify` — the world
        // diff records nothing for "box", not even a no-op entry.
        let diff = child.world_diff.as_ref().unwrap();
        assert!(diff.get("box").is_none());
    }

    #[test]
    fn set_subframes_of_object_empty_map_removes_every_subframe() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape("box", cuboid_shape(), Isometry3::identity());
        let mut subframes = BTreeMap::new();
        subframes.insert("tip".to_owned(), Isometry3::translation(0.0, 0.0, 0.5));
        assert!(scene.set_subframes_of_object("box", subframes));
        assert!(
            scene
                .world()
                .get_object("box")
                .unwrap()
                .subframe_pose("tip")
                .is_some()
        );

        assert!(scene.set_subframes_of_object("box", BTreeMap::new()));

        assert_eq!(
            scene
                .world()
                .get_object("box")
                .unwrap()
                .subframe_names()
                .count(),
            0
        );
    }

    // ---- push_diffs: the "attached, not deleted" ACM guard ------------------

    #[test]
    fn push_diffs_propagates_a_locally_own_transforms_map() {
        let model = build_model();
        let root = PlanningScene::new(&model, &srdf());
        let root = Arc::new(root);
        let mut child = root.diff();
        child
            .transforms_mut()
            .set_transform(Isometry3::translation(1.0, 0.0, 0.0), "map")
            .unwrap();

        let mut target = PlanningScene::new(&model, &srdf());
        child.push_diffs(&mut target);

        assert_eq!(
            target.frame_transform("map").unwrap(),
            Isometry3::translation(1.0, 0.0, 0.0)
        );
    }

    #[test]
    fn push_diffs_prunes_the_acm_entry_for_a_genuinely_destroyed_object() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.add_shape("box", cuboid_shape(), Isometry3::identity());
        root.allowed_collision_matrix_mut()
            .set_entry("box", "hand", true);
        let root = Arc::new(root);
        let mut child = root.diff();
        child.remove_object("box");

        let mut target = PlanningScene::new(&model, &srdf());
        target.add_shape("box", cuboid_shape(), Isometry3::identity());
        target
            .allowed_collision_matrix_mut()
            .set_entry("box", "hand", true);

        child.push_diffs(&mut target);

        assert!(!target.world().has_object("box"));
        assert!(!target.allowed_collision_matrix().has_entry("box"));
    }

    #[test]
    fn push_diffs_preserves_the_acm_entry_when_the_target_still_has_it_attached() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.add_shape("box", cuboid_shape(), Isometry3::identity());
        root.allowed_collision_matrix_mut()
            .set_entry("box", "hand", true);
        let root = Arc::new(root);
        let mut child = root.diff();
        // Attaching removes "box" from the child's world, producing the same
        // pure-DESTROY diff entry a real deletion would.
        child.attach("box", "hand", BTreeSet::new()).unwrap();

        let mut target = PlanningScene::new(&model, &srdf());
        target.add_shape("box", cuboid_shape(), Isometry3::identity());
        target
            .allowed_collision_matrix_mut()
            .set_entry("box", "hand", true);
        // The target independently attached "box" too — this is what the
        // guard actually reads.
        target.attach("box", "hand", BTreeSet::new()).unwrap();

        child.push_diffs(&mut target);

        assert!(target.allowed_collision_matrix().has_entry("box"));
    }

    #[test]
    fn push_diffs_is_a_no_op_for_a_root_scene() {
        let model = build_model();
        let root = PlanningScene::new(&model, &srdf());
        let mut target = PlanningScene::new(&model, &srdf());
        target.add_shape("box", cuboid_shape(), Isometry3::identity());

        root.push_diffs(&mut target);

        assert!(target.world().has_object("box"));
    }

    // ---- parent fallthrough vs override --------------------------------------

    #[test]
    fn child_falls_through_to_the_parent_acm_for_a_pair_it_never_touched() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.allowed_collision_matrix_mut()
            .set_entry("a", "b", true);
        let root = Arc::new(root);
        let child = root.diff();

        assert!(!child.acm.is_own());
        assert_eq!(
            child
                .allowed_collision_matrix()
                .entry("a", "b")
                .unwrap()
                .kind(),
            AllowedCollisionType::Always
        );
    }

    #[test]
    fn child_override_diverges_from_the_parent_without_mutating_it() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.allowed_collision_matrix_mut()
            .set_entry("c", "d", false);
        let root = Arc::new(root);
        let mut child = root.diff();

        child
            .allowed_collision_matrix_mut()
            .set_entry("c", "d", true);

        assert!(child.acm.is_own());
        assert_eq!(
            child
                .allowed_collision_matrix()
                .entry("c", "d")
                .unwrap()
                .kind(),
            AllowedCollisionType::Always
        );
        assert_eq!(
            root.allowed_collision_matrix()
                .entry("c", "d")
                .unwrap()
                .kind(),
            AllowedCollisionType::Never
        );
    }

    #[test]
    fn child_current_state_falls_through_until_mutated_then_materializes_its_own() {
        let model = build_model();
        let root = Arc::new(PlanningScene::new(&model, &srdf()));
        let mut child = root.diff();

        assert!(!child.robot_state.is_own());
        assert_eq!(
            child.current_state().positions(),
            root.current_state().positions()
        );

        child.current_state_mut();
        assert!(child.robot_state.is_own());
    }

    // ---- decouple_parent: isolation from later parent mutation --------------

    #[test]
    fn decouple_parent_then_mutating_the_former_parent_is_not_observed() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.allowed_collision_matrix_mut()
            .set_entry("a", "b", true);
        let mut root = Arc::new(root);
        let mut child = root.diff();

        child.decouple_parent();
        assert!(child.parent().is_none());

        Arc::get_mut(&mut root)
            .expect("sole owner: child dropped its Arc clone in decouple_parent")
            .allowed_collision_matrix_mut()
            .set_entry("a", "b", false);

        assert_eq!(
            child
                .allowed_collision_matrix()
                .entry("a", "b")
                .unwrap()
                .kind(),
            AllowedCollisionType::Always
        );
    }

    #[test]
    fn decouple_parent_materializes_the_inherited_transforms_map() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.transforms_mut()
            .set_transform(Isometry3::translation(1.0, 0.0, 0.0), "map")
            .unwrap();
        let mut root = Arc::new(root);
        let mut child = root.diff();
        assert_eq!(
            child.frame_transform("map").unwrap(),
            Isometry3::translation(1.0, 0.0, 0.0)
        );

        child.decouple_parent();
        assert!(child.parent().is_none());

        Arc::get_mut(&mut root)
            .expect("sole owner: child dropped its Arc clone in decouple_parent")
            .transforms_mut()
            .set_transform(Isometry3::translation(2.0, 0.0, 0.0), "map")
            .unwrap();

        // The child's copy was materialized at `decouple_parent` time, so
        // the parent's later mutation of "map" is not observed -- upstream
        // `planning_scene.cpp:1260-1264`'s `scene_transforms_` copy,
        // matching `robot_state`/`acm`'s existing decouple treatment above.
        assert_eq!(
            child.frame_transform("map").unwrap(),
            Isometry3::translation(1.0, 0.0, 0.0)
        );
    }

    #[test]
    fn decouple_parent_then_the_childs_inherited_attached_body_frame_still_resolves() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.attach_new(
            "box",
            "hand",
            vec![cuboid_shape()],
            vec![Isometry3::translation(0.3, 0.0, 0.0)],
            BTreeSet::new(),
            BTreeMap::new(),
        )
        .unwrap();
        let root = Arc::new(root);
        let mut child = root.diff();

        child.decouple_parent();
        assert!(child.parent().is_none());

        // `attached_bodies` is cloned eagerly at `diff()` regardless of
        // decoupling, but resolving its frame still routes through
        // `current_state_mut`, which was `Layered::Inherited` before
        // `decouple_parent` materialized it -- this exercises that
        // materialization happened correctly rather than leaving a
        // dangling inherited state with no parent to resolve against.
        assert_eq!(
            child.frame_transform("box").unwrap(),
            Isometry3::translation(0.0, 0.0, 1.0)
        );
        assert!(child.knows_frame_transform("box"));
    }

    #[test]
    fn decouple_parent_then_the_childs_inherited_world_object_still_resolves() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.add_shape("crate", cuboid_shape(), Isometry3::identity());
        root.move_object("crate", Isometry3::translation(2.0, 0.0, 0.0));
        let root = Arc::new(root);
        let mut child = root.diff();

        child.decouple_parent();
        assert!(child.parent().is_none());

        // `world` is a full clone at `diff()` time, not layered, so this
        // confirms `decouple_parent` (which only touches `transforms`/
        // `robot_state`/`acm`/`world_diff`/`parent`) leaves that already-materialized
        // world content intact rather than discarding it along with the
        // diff-tracking it does clear.
        assert_eq!(child.world().object_ids(), vec!["crate".to_owned()]);
        assert_eq!(
            child.frame_transform("crate").unwrap(),
            Isometry3::translation(2.0, 0.0, 0.0)
        );
        assert!(child.knows_frame_transform("crate"));
    }

    #[test]
    fn clear_diffs_on_a_root_scene_is_a_no_op() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.add_shape("box", cuboid_shape(), Isometry3::identity());

        root.clear_diffs();

        assert_eq!(root.world().object_ids(), vec!["box".to_owned()]);
    }

    #[test]
    fn clear_diffs_resets_a_diverged_child_to_a_fresh_diff_against_the_parent() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.add_shape("floor", cuboid_shape(), Isometry3::identity());
        // The parent carries its own attached body and ACM entry, distinct
        // from what the child diverges with below -- otherwise `clear_diffs`
        // resetting `attached_bodies`/`acm` to empty (a bug) would be
        // indistinguishable from correctly re-inheriting the parent's (then
        // also empty) state. See `clear_diffs`'s own `parent.attached_bodies
        // .clone()`/`Layered::Inherited` lines: this is what tells them
        // apart from a `BTreeMap::new()`/fresh-`Own`-ACM stand-in.
        root.attach_new(
            "root_held",
            "hand",
            vec![cuboid_shape()],
            vec![Isometry3::identity()],
            BTreeSet::new(),
            BTreeMap::new(),
        )
        .unwrap();
        root.allowed_collision_matrix_mut()
            .set_entry("root_a", "root_b", true);
        let root = Arc::new(root);
        let mut child = root.diff();

        child.add_shape("box", cuboid_shape(), Isometry3::translation(1.0, 0.0, 0.0));
        assert!(child.remove_object("floor"));
        child
            .allowed_collision_matrix_mut()
            .set_entry("a", "b", true);
        child
            .attach_new(
                "held",
                "hand",
                vec![cuboid_shape()],
                vec![Isometry3::identity()],
                BTreeSet::new(),
                BTreeMap::new(),
            )
            .unwrap();

        child.clear_diffs();

        assert!(child.parent().is_some());
        assert_eq!(child.world().object_ids(), vec!["floor".to_owned()]);
        assert_eq!(
            child.attached_bodies().map(|b| b.id()).collect::<Vec<_>>(),
            vec!["root_held"]
        );
        assert!(
            child
                .allowed_collision_matrix()
                .entry("root_a", "root_b")
                .is_some()
        );
        assert!(child.allowed_collision_matrix().entry("a", "b").is_none());
        let diff = child
            .world_diff
            .as_ref()
            .expect("child scene must track a diff");
        assert!(diff.is_empty());
    }

    // ---- frames: the six-tier ladder ----------------------------------------

    #[test]
    fn frame_transform_resolves_the_model_frame_and_a_link_name() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());

        // This fixture's virtual joint is `type="fixed"`, so
        // `RobotModel::model_frame` is the root link name ("base"), not the
        // virtual joint's `parent_frame` ("world") -- `parent_frame` only
        // becomes the model frame for planar/floating virtual joints. "base"
        // therefore exercises the model-frame and link-name tiers at once;
        // "world" is not a frame this model knows at all.
        assert_eq!(model.model_frame(), "base");
        assert_eq!(
            scene.frame_transform("base").unwrap(),
            Isometry3::identity()
        );
        assert_eq!(
            scene.frame_transform("hand").unwrap(),
            Isometry3::translation(0.0, 0.0, 1.0)
        );
        assert!(scene.knows_frame_transform("base"));
        assert!(scene.frame_transform("world").is_err());
        assert!(!scene.knows_frame_transform("world"));
    }

    #[test]
    fn frame_transform_resolves_an_attached_bodys_bare_id_to_its_links_transform() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .attach_new(
                "box",
                "hand",
                vec![cuboid_shape()],
                vec![Isometry3::translation(0.3, 0.0, 0.0)],
                BTreeSet::new(),
                BTreeMap::new(),
            )
            .unwrap();

        // A bare attached-body id resolves to its attach link's own global
        // pose, not the shape's -- see `attached_frame`'s doc for why there
        // is no separate body-local offset in this port's one-level design.
        assert_eq!(
            scene.frame_transform("box").unwrap(),
            Isometry3::translation(0.0, 0.0, 1.0)
        );
        assert!(scene.knows_frame_transform("box"));
    }

    #[test]
    fn frame_transform_resolves_an_attached_bodys_subframe() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        let mut subframes = BTreeMap::new();
        subframes.insert("tip".to_owned(), Isometry3::translation(0.0, 0.0, 0.2));
        scene
            .attach_new(
                "box",
                "hand",
                vec![cuboid_shape()],
                vec![Isometry3::identity()],
                BTreeSet::new(),
                subframes,
            )
            .unwrap();

        assert_eq!(
            scene.frame_transform("box/tip").unwrap(),
            Isometry3::translation(0.0, 0.0, 1.2)
        );
        assert!(scene.knows_frame_transform("box/tip"));
    }

    #[test]
    fn frame_transform_falls_through_to_the_world_for_an_object_and_its_subframe() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        // `add_shape` poses the shape relative to the object, not the
        // object itself (`World::add_shape` always creates the object at
        // `Isometry3::identity()`) -- `move_object` is what sets the
        // object's own pose, which is what `frame_transform`'s world tier
        // (`World::get_transform`) reports back.
        scene.add_shape("crate", cuboid_shape(), Isometry3::identity());
        scene.move_object("crate", Isometry3::translation(2.0, 0.0, 0.0));
        let mut subframes = BTreeMap::new();
        subframes.insert("lid".to_owned(), Isometry3::translation(0.0, 0.0, 0.1));
        scene.world.set_subframes_of_object("crate", subframes);

        assert_eq!(
            scene.frame_transform("crate").unwrap(),
            Isometry3::translation(2.0, 0.0, 0.0)
        );
        assert_eq!(
            scene.frame_transform("crate/lid").unwrap(),
            Isometry3::translation(2.0, 0.0, 0.1)
        );
        assert!(scene.knows_frame_transform("crate"));
        assert!(scene.knows_frame_transform("crate/lid"));
    }

    #[test]
    fn frame_transform_reports_a_name_resolving_in_no_tier() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());

        assert!(scene.frame_transform("nothing").is_err());
        assert!(!scene.knows_frame_transform("nothing"));
    }

    #[test]
    fn frame_transform_prefers_the_attached_body_tier_over_a_same_named_world_object() {
        // The ladder checks attached bodies (tiers 3-4) strictly before the
        // world (tier 5) -- upstream's own order, `RobotState::getFrameInfo`
        // (folded into `state.getFrameTransform`, tier 2 here) runs before
        // `World::getTransform` in `PlanningScene::getFrameTransform`
        // (`planning_scene.cpp:2036`). A world object and an attached body
        // sharing a name should not be reachable in practice ([`PlanningScene::attach`]
        // removes the world object of the same id first), so this exercises
        // the ladder's ordering directly rather than a realistic scene.
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape("dup", cuboid_shape(), Isometry3::translation(9.0, 0.0, 0.0));
        scene
            .attach_new(
                "dup",
                "hand",
                vec![cuboid_shape()],
                vec![Isometry3::identity()],
                BTreeSet::new(),
                BTreeMap::new(),
            )
            .unwrap();

        assert_eq!(
            scene.frame_transform("dup").unwrap(),
            Isometry3::translation(0.0, 0.0, 1.0)
        );
    }

    #[test]
    fn frame_transform_falls_through_to_the_extra_fixed_frame_map() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .transforms_mut()
            .set_transform(Isometry3::translation(5.0, 0.0, 0.0), "map")
            .unwrap();

        assert_eq!(
            scene.frame_transform("map").unwrap(),
            Isometry3::translation(5.0, 0.0, 0.0)
        );
        assert!(scene.knows_frame_transform("map"));
    }

    #[test]
    fn frame_transform_tier_six_absent_name_is_still_unknown() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .transforms_mut()
            .set_transform(Isometry3::translation(5.0, 0.0, 0.0), "map")
            .unwrap();

        assert!(scene.frame_transform("no_such_frame").is_err());
        assert!(!scene.knows_frame_transform("no_such_frame"));
    }

    #[test]
    fn frame_transform_tier_six_empty_name_is_unknown() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());

        assert!(scene.frame_transform("").is_err());
        assert!(!scene.knows_frame_transform(""));
    }

    #[test]
    fn frame_transform_leading_slash_reaches_the_extra_fixed_frame_map() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .transforms_mut()
            .set_transform(Isometry3::translation(5.0, 0.0, 0.0), "map")
            .unwrap();

        // Both `frame_transform` and `knows_frame_transform` strip a leading
        // `/` before dispatching to any tier -- confirmed here through tier
        // 6 specifically, since the earlier tiers already cover it for link
        // names via the model-frame test above.
        assert_eq!(
            scene.frame_transform("/map").unwrap(),
            Isometry3::translation(5.0, 0.0, 0.0)
        );
        assert!(scene.knows_frame_transform("/map"));
    }

    #[test]
    fn frame_transform_prefers_the_link_tier_over_a_same_named_extra_fixed_frame() {
        // "hand" is a real link. Registering "hand" in the extra-fixed-frame
        // map too must not shadow the link tier -- tier 2 (link names) runs
        // strictly before tier 6 (the extra-fixed-frame map) in the ladder,
        // matching upstream's order (`state.getFrameTransform` before
        // `getTransforms().Transforms::getTransform`,
        // `planning_scene.cpp:2036`/`:2053`).
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .transforms_mut()
            .set_transform(Isometry3::translation(9.0, 9.0, 9.0), "hand")
            .unwrap();

        assert_eq!(
            scene.frame_transform("hand").unwrap(),
            Isometry3::translation(0.0, 0.0, 1.0)
        );
    }

    #[test]
    fn planning_frame_is_the_model_frame() {
        let model = build_model();
        let scene = PlanningScene::new(&model, &srdf());
        assert_eq!(scene.planning_frame(), model.model_frame());
        assert_eq!(scene.planning_frame(), "base");
    }

    #[test]
    fn transforms_with_world_objects_matches_transforms_when_the_world_is_empty() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .transforms_mut()
            .set_transform(Isometry3::translation(5.0, 0.0, 0.0), "map")
            .unwrap();

        assert_eq!(scene.transforms_with_world_objects(), *scene.transforms());
    }

    #[test]
    fn transforms_with_world_objects_answers_can_transform_for_a_bare_and_slash_prefixed_object_id()
    {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape("crate", cuboid_shape(), Isometry3::identity());
        scene.move_object("crate", Isometry3::translation(2.0, 0.0, 0.0));

        let snapshot = scene.transforms_with_world_objects();

        assert!(snapshot.can_transform("crate"));
        assert!(snapshot.can_transform("/crate"));
        assert_eq!(
            *snapshot.transform("crate").unwrap(),
            Isometry3::translation(2.0, 0.0, 0.0)
        );
        assert_eq!(
            *snapshot.transform("/crate").unwrap(),
            Isometry3::translation(2.0, 0.0, 0.0)
        );
    }

    #[test]
    fn transforms_with_world_objects_answers_can_transform_for_a_bare_and_slash_prefixed_subframe()
    {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape("crate", cuboid_shape(), Isometry3::identity());
        scene.move_object("crate", Isometry3::translation(2.0, 0.0, 0.0));
        let mut subframes = BTreeMap::new();
        subframes.insert("lid".to_owned(), Isometry3::translation(0.0, 0.0, 0.1));
        scene.world.set_subframes_of_object("crate", subframes);

        let snapshot = scene.transforms_with_world_objects();

        assert!(snapshot.can_transform("crate/lid"));
        assert!(snapshot.can_transform("/crate/lid"));
        assert_eq!(
            *snapshot.transform("crate/lid").unwrap(),
            Isometry3::translation(2.0, 0.0, 0.1)
        );
    }

    #[test]
    fn transforms_with_world_objects_leaves_a_name_resolving_in_no_tier_unknown() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape("crate", cuboid_shape(), Isometry3::identity());

        let snapshot = scene.transforms_with_world_objects();

        assert!(!snapshot.can_transform("nothing"));
    }

    #[test]
    fn transforms_with_world_objects_does_not_over_match_a_slash_led_object_id() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape("/obj", cuboid_shape(), Isometry3::identity());
        scene.move_object("/obj", Isometry3::translation(3.0, 0.0, 0.0));

        let snapshot = scene.transforms_with_world_objects();

        // isFixedFrame("/obj") strips one leading '/' and checks
        // knowsObjectFrame("obj") -- a different object's id, which does not
        // exist here -- so the bare id must NOT resolve, even though it is
        // the object's own literal name.
        assert!(!snapshot.can_transform("/obj"));
        assert!(!snapshot.can_transform("obj"));
        // isFixedFrame("//obj") strips one leading '/' down to the literal
        // id "/obj", which does exist.
        assert!(snapshot.can_transform("//obj"));
        assert_eq!(
            *snapshot.transform("//obj").unwrap(),
            Isometry3::translation(3.0, 0.0, 0.0)
        );
    }

    #[test]
    fn transforms_with_world_objects_does_not_double_prefix_a_plain_object_id() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape("obj", cuboid_shape(), Isometry3::identity());

        let snapshot = scene.transforms_with_world_objects();

        assert!(!snapshot.can_transform("//obj"));
    }

    #[test]
    fn transforms_with_world_objects_prefers_an_object_id_over_a_colliding_subframe_composite() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        // Object "a" with subframe "b" would compose to the same key "a/b"
        // as a distinct top-level object literally named "a/b". Upstream
        // resolves object ids before subframes (`world.cpp:145`), so the
        // top-level object's own pose must win.
        scene.add_shape("a", cuboid_shape(), Isometry3::identity());
        let mut subframes = BTreeMap::new();
        subframes.insert("b".to_owned(), Isometry3::translation(0.0, 0.0, 0.1));
        scene.world.set_subframes_of_object("a", subframes);
        scene.add_shape("a/b", cuboid_shape(), Isometry3::identity());
        scene.move_object("a/b", Isometry3::translation(9.0, 0.0, 0.0));

        let snapshot = scene.transforms_with_world_objects();

        assert!(snapshot.can_transform("a/b"));
        assert_eq!(
            *snapshot.transform("a/b").unwrap(),
            Isometry3::translation(9.0, 0.0, 0.0)
        );
    }

    #[test]
    fn transforms_with_world_objects_never_answers_can_transform_for_an_empty_name() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape("crate", cuboid_shape(), Isometry3::identity());

        let snapshot = scene.transforms_with_world_objects();

        assert!(!snapshot.can_transform(""));
    }

    // ---- collision checking -----------------------------------------------

    // Fixture: a fixed-base robot with two independent floating-joint boxes,
    // `p`/`q`, so each can be posed to an arbitrary independent global
    // transform. Mirrors `moveit_collision::parry`'s own test fixture.

    fn box_link(name: &str) -> String {
        format!(
            r#"<link name="{name}">
                <collision><geometry><box size="1 1 1"/></geometry></collision>
            </link>"#
        )
    }

    fn floating_joint(name: &str, parent: &str, child: &str) -> String {
        format!(
            r#"<joint name="{name}" type="floating">
                <parent link="{parent}"/>
                <child link="{child}"/>
            </joint>"#
        )
    }

    fn build_collision_model() -> RobotModel {
        let urdf_xml = format!(
            r#"<robot name="test">
                <link name="base"/>
                {p}{joint_p}{q}{joint_q}
            </robot>"#,
            p = box_link("p"),
            joint_p = floating_joint("joint_p", "base", "p"),
            q = box_link("q"),
            joint_q = floating_joint("joint_q", "base", "q"),
        );
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("test URDF must parse");
        let srdf = SrdfModel::parse_str(SRDF_XML).expect("test SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("test fixture model must build")
    }

    /// Adds a third, unrelated link `r` to [`build_collision_model`]'s
    /// `p`/`q` pair, and an SRDF group containing only `r` -- neither side
    /// of the `p`/`q` collision pair is ever in this group, so it isolates
    /// [`PlanningScene::colliding_pairs`]'s `group_name` parameter from its
    /// own `None` case.
    fn build_collision_model_with_unrelated_group() -> (RobotModel, SrdfModel) {
        let urdf_xml = format!(
            r#"<robot name="test">
                <link name="base"/>
                {p}{joint_p}{q}{joint_q}{r}{joint_r}
            </robot>"#,
            p = box_link("p"),
            joint_p = floating_joint("joint_p", "base", "p"),
            q = box_link("q"),
            joint_q = floating_joint("joint_q", "base", "q"),
            r = box_link("r"),
            joint_r = floating_joint("joint_r", "base", "r"),
        );
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("test URDF must parse");
        let srdf_xml = format!(
            "{}{}",
            SRDF_XML.trim_end_matches("</robot>"),
            r#"<group name="r_only"><joint name="joint_r"/></group></robot>"#
        );
        let srdf = SrdfModel::parse_str(&srdf_xml).expect("test SRDF must parse");
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("test fixture model must build");
        (model, srdf)
    }

    #[test]
    fn check_self_collision_reports_overlapping_links() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .current_state_mut()
            .set_joint_transform("joint_q", &Isometry3::translation(0.5, 0.0, 0.0))
            .unwrap();
        let env = ParryCollisionEnv::default();

        let result = scene.check_self_collision(&env, &CollisionRequest::default());

        assert!(result.collision);
    }

    #[test]
    fn check_self_collision_reports_clear_when_links_are_apart() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .current_state_mut()
            .set_joint_transform("joint_q", &Isometry3::translation(5.0, 0.0, 0.0))
            .unwrap();
        let env = ParryCollisionEnv::default();

        let result = scene.check_self_collision(&env, &CollisionRequest::default());

        assert!(!result.collision);
    }

    #[test]
    fn is_state_colliding_reports_the_current_states_self_collision() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .current_state_mut()
            .set_joint_transform("joint_q", &Isometry3::translation(0.5, 0.0, 0.0))
            .unwrap();
        let env = ParryCollisionEnv::default();

        assert!(scene.is_state_colliding(&env, &CollisionRequest::default()));
    }

    #[test]
    fn is_state_colliding_is_false_when_links_are_apart() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .current_state_mut()
            .set_joint_transform("joint_q", &Isometry3::translation(5.0, 0.0, 0.0))
            .unwrap();
        let env = ParryCollisionEnv::default();

        assert!(!scene.is_state_colliding(&env, &CollisionRequest::default()));
    }

    #[test]
    fn check_robot_collision_sees_the_scenes_world_once_cloned_into_env() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::identity(),
        );
        let env = ParryCollisionEnv::new(scene.world().clone(), LinkPaddingScale::default());

        let result = scene.check_robot_collision(&env, &CollisionRequest::default());

        assert!(result.collision);
    }

    #[test]
    fn check_robot_collision_does_not_see_a_world_object_added_after_the_env_was_built() {
        // Documents the "env's world is a value, not a live view" contract
        // from the type doc's "Collision checking" section.
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        let env = ParryCollisionEnv::new(scene.world().clone(), LinkPaddingScale::default());
        scene.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::identity(),
        );

        let result = scene.check_robot_collision(&env, &CollisionRequest::default());

        assert!(!result.collision);
    }

    #[test]
    fn check_collision_finds_self_collision_even_when_the_world_is_clear() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .current_state_mut()
            .set_joint_transform("joint_q", &Isometry3::translation(0.5, 0.0, 0.0))
            .unwrap();
        let env = ParryCollisionEnv::default();

        let result = scene.check_collision(&env, &CollisionRequest::default());

        assert!(result.collision);
    }

    #[test]
    fn is_path_valid_collects_every_invalid_waypoint_not_just_the_first() {
        // Measures the type doc's claim on `PathValidity` ("this port always
        // computes the full set") and `is_path_valid`'s own claim ("Invalid
        // waypoint indices are collected, not short-circuited on the first
        // failure") -- neither had a regression test: every existing
        // `is_path_valid` case (`is_state_valid_parity.rs`'s fixture) has at
        // most one invalid waypoint per path, which cannot distinguish
        // "collects all" from "stops at the first hit" -- both produce the
        // same one-element `invalid_waypoints`. Two waypoints here, `joint_q`
        // moved to `x = 5.0` and `x = 6.0` (both collision-free per
        // `is_state_colliding_is_false_when_links_are_apart`, so collision
        // cannot be the thing failing either one), against a path constraint
        // neither position satisfies (`trans_x == 0.0 ± 0.01`) -- a
        // short-circuiting implementation would report only `[0]`.
        let model = build_collision_model();
        let scene = PlanningScene::new(&model, &srdf());
        let mut waypoint_0 = scene.current_state().clone();
        waypoint_0
            .set_joint_transform("joint_q", &Isometry3::translation(5.0, 0.0, 0.0))
            .unwrap();
        let mut waypoint_1 = scene.current_state().clone();
        waypoint_1
            .set_joint_transform("joint_q", &Isometry3::translation(6.0, 0.0, 0.0))
            .unwrap();
        let mut scene = scene;
        let mut path_constraints = KinematicConstraintSet::new();
        path_constraints.push(Constraint::Joint(
            JointConstraint::new(&model, "joint_q/trans_x", 0.0, 0.01, 0.01, 1.0).unwrap(),
        ));
        let env = ParryCollisionEnv::default();

        let result = scene.is_path_valid(
            &env,
            &CollisionRequest::default(),
            &[waypoint_0, waypoint_1],
            Some(&path_constraints),
            &[],
        );

        assert!(!result.valid);
        assert_eq!(result.invalid_waypoints, vec![0, 1]);
    }

    #[test]
    fn distance_to_collision_reports_the_gap_to_a_world_object() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::translation(2.0, 0.0, 0.0),
        );
        let env = ParryCollisionEnv::new(scene.world().clone(), LinkPaddingScale::default());

        let distance = scene.distance_to_collision(&env);

        assert!((distance - 1.0).abs() < 1e-9);
    }

    #[test]
    fn colliding_pairs_and_colliding_links_report_the_overlapping_self_collision_pair() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .current_state_mut()
            .set_joint_transform("joint_q", &Isometry3::translation(0.5, 0.0, 0.0))
            .unwrap();
        let env = ParryCollisionEnv::default();

        let pairs = scene.colliding_pairs(&env, None);
        assert_eq!(pairs.len(), 1);
        assert!(pairs.contains_key(&("p".to_string(), "q".to_string())));

        let mut links = scene.colliding_links(&env);
        links.sort();
        assert_eq!(links, vec!["p".to_string(), "q".to_string()]);
    }

    /// D13 (`PORTING-PLAN.md` §197): `group_name` must actually change what
    /// [`PlanningScene::colliding_pairs`] returns, not merely compile. `p`
    /// and `q` collide; `"r_only"` contains neither, so filtering by it
    /// must drop the pair entirely, while `None` keeps it -- the same
    /// unrelated-group setup `moveit_collision::parry`'s own
    /// `check_self_collision_group_name_*` tests use.
    #[test]
    fn colliding_pairs_group_name_drops_a_pair_with_neither_side_in_the_group() {
        let (model, srdf) = build_collision_model_with_unrelated_group();
        let mut scene = PlanningScene::new(&model, &srdf);
        scene
            .current_state_mut()
            .set_joint_transform("joint_q", &Isometry3::translation(0.5, 0.0, 0.0))
            .unwrap();
        // `r` defaults to the same identity pose as `p` (both floating
        // joints), which would otherwise coincide and collide too --  move
        // it away so the only colliding pair is `p`/`q`.
        scene
            .current_state_mut()
            .set_joint_transform("joint_r", &Isometry3::translation(10.0, 10.0, 10.0))
            .unwrap();
        let env = ParryCollisionEnv::default();

        let unfiltered = scene.colliding_pairs(&env, None);
        assert_eq!(unfiltered.len(), 1);
        assert!(unfiltered.contains_key(&("p".to_string(), "q".to_string())));

        let filtered = scene.colliding_pairs(&env, Some("r_only"));
        assert!(
            filtered.is_empty(),
            "group_name=\"r_only\" contains neither p nor q, so the pair must be dropped, got {filtered:?}"
        );
    }

    // ---- cost sources -------------------------------------------------------

    // `p` stays at the identity transform throughout (box spans [-0.5, 0.5]^3
    // on every axis); every case below only moves `q` via `joint_q`, so an
    // overlap box's AABB is exactly computable by hand from `q`'s translation.

    #[test]
    fn cost_sources_is_empty_when_nothing_collides() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .current_state_mut()
            .set_joint_transform("joint_q", &Isometry3::translation(100.0, 0.0, 0.0))
            .unwrap();
        let env = ParryCollisionEnv::default();

        let sources = scene.cost_sources(&env, None, 10);

        assert!(sources.is_empty());
    }

    #[test]
    fn cost_sources_reports_the_one_overlap_box() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .current_state_mut()
            .set_joint_transform("joint_q", &Isometry3::translation(0.5, 0.0, 0.0))
            .unwrap();
        let env = ParryCollisionEnv::default();

        let sources: Vec<CostSource> = scene.cost_sources(&env, None, 10).into_iter().collect();

        assert_eq!(sources.len(), 1);
        assert_eq!(sources[0].aabb_min, [0.0, -0.5, -0.5]);
        assert_eq!(sources[0].aabb_max, [0.5, 0.5, 0.5]);
        assert_eq!(sources[0].cost, 1.0);
    }

    fn waypoint_with_q_at<'m>(scene: &PlanningScene<'m>, dx: f64, dy: f64) -> RobotState<'m> {
        let mut state = scene.current_state().clone();
        state
            .set_joint_transform("joint_q", &Isometry3::translation(dx, dy, 0.0))
            .unwrap();
        state
    }

    #[test]
    fn path_cost_sources_unions_non_overlapping_sources_across_waypoints() {
        // Start waypoint has `q` far away (empty `cs_start`); the two later
        // waypoints put `q` on opposite sides of `p`, each producing a
        // disjoint overlap box (a gap between x = 0.0 and x = 0.25), so both
        // survive `remove_overlapping` untouched.
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        let start = waypoint_with_q_at(&scene, 100.0, 0.0);
        let left = waypoint_with_q_at(&scene, -0.5, 0.0);
        let right = waypoint_with_q_at(&scene, 0.75, 0.0);
        let env = ParryCollisionEnv::default();

        let sources: Vec<CostSource> = scene
            .path_cost_sources(&env, &[start, left, right], None, 10, 0.5)
            .into_iter()
            .collect();

        assert_eq!(sources.len(), 2);
        assert_eq!(sources[0].aabb_min, [-0.5, -0.5, -0.5]);
        assert_eq!(sources[0].aabb_max, [0.0, 0.5, 0.5]);
        assert_eq!(sources[1].aabb_min, [0.25, -0.5, -0.5]);
        assert_eq!(sources[1].aabb_max, [0.5, 0.5, 0.5]);
    }

    #[test]
    fn path_cost_sources_merges_a_fully_overlapping_pair_into_the_more_costly_one() {
        // `q` at `dx = 0.5` gives a volume-0.5 box; `q` at `dx = 0.6` gives a
        // volume-0.4 box that is a strict subset of the first
        // (x in [0.1, 0.5] vs [0.0, 0.5]) -- a full overlap relative to the
        // smaller box's own volume, so `remove_overlapping` drops it and
        // keeps only the more costly (bigger) one.
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        let start = waypoint_with_q_at(&scene, 100.0, 0.0);
        let bigger = waypoint_with_q_at(&scene, 0.5, 0.0);
        let smaller = waypoint_with_q_at(&scene, 0.6, 0.0);
        let env = ParryCollisionEnv::default();

        let sources: Vec<CostSource> = scene
            .path_cost_sources(&env, &[start, bigger, smaller], None, 10, 0.5)
            .into_iter()
            .collect();

        assert_eq!(sources.len(), 1);
        assert_eq!(sources[0].aabb_min, [0.0, -0.5, -0.5]);
        assert_eq!(sources[0].aabb_max, [0.5, 0.5, 0.5]);
    }

    #[test]
    fn path_cost_sources_keeps_a_partial_overlap_below_threshold() {
        // `q` at `dx = -0.2` (volume 0.8) and `dx = 0.3` (volume 0.7) overlap
        // by 0.5 -- below `0.8 * overlap_fraction (0.9) = 0.72`, so neither
        // is dropped: both survive as distinct, still-overlapping sources.
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        let start = waypoint_with_q_at(&scene, 100.0, 0.0);
        let a = waypoint_with_q_at(&scene, 0.3, 0.0);
        let b = waypoint_with_q_at(&scene, -0.2, 0.0);
        let env = ParryCollisionEnv::default();

        let sources = scene.path_cost_sources(&env, &[start, a, b], None, 10, 0.9);

        assert_eq!(sources.len(), 2);
    }

    #[test]
    fn path_cost_sources_truncates_to_max_costs_keeping_the_most_costly() {
        // Same disjoint pair as the "non-overlapping" case (volumes 0.5 and
        // 0.3), but `max_costs = 1`: the union truncation
        // (`cs.into_iter().take(max_costs)`) keeps only the more costly one.
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        let start = waypoint_with_q_at(&scene, 100.0, 0.0);
        let bigger = waypoint_with_q_at(&scene, -0.5, 0.0);
        let smaller = waypoint_with_q_at(&scene, 0.7, 0.0);
        let env = ParryCollisionEnv::default();

        let sources: Vec<CostSource> = scene
            .path_cost_sources(&env, &[start, bigger, smaller], None, 1, 0.5)
            .into_iter()
            .collect();

        assert_eq!(sources.len(), 1);
        assert_eq!(sources[0].aabb_min, [-0.5, -0.5, -0.5]);
        assert_eq!(sources[0].aabb_max, [0.0, 0.5, 0.5]);
    }

    #[test]
    fn path_cost_sources_keeps_all_sources_when_fewer_than_max_costs() {
        // A single real cost source (`q` at `dx = 0.5`) against
        // `max_costs = 10`: `cs.len() <= max_costs` takes the no-truncation
        // branch, so the one source is returned untouched.
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        let start = waypoint_with_q_at(&scene, 100.0, 0.0);
        let colliding = waypoint_with_q_at(&scene, 0.5, 0.0);
        let env = ParryCollisionEnv::default();

        let sources: Vec<CostSource> = scene
            .path_cost_sources(&env, &[start, colliding], None, 10, 0.5)
            .into_iter()
            .collect();

        assert_eq!(sources.len(), 1);
        assert_eq!(sources[0].aabb_min, [0.0, -0.5, -0.5]);
        assert_eq!(sources[0].aabb_max, [0.5, 0.5, 0.5]);
    }
}
