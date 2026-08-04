# `moveit_msgs`/`geometry_msgs`/... ↔ core-crate type mapping

Phase 9 round 1 (`PORTING-PLAN.md` §5, §129). Every `moveit_msgs` type this
round could reach, its core-crate counterpart, and every spot where the
mapping is not 1:1 -- per the round-1 brief: "대응이 1:1이 아닌 자리를 전부
이름으로 적어라." Message field text below is quoted from
`/ws/src/moveit_msgs/msg/*.msg` inside `moveit-rs/oracle:8ed8a9395b730b08`
(the pinned upstream checkout the oracle image builds from) and from
`/opt/ros/rolling/share/{trajectory_msgs,sensor_msgs}/msg/*.msg` inside the
same image -- read directly, not recalled from memory.

**Status legend**

- **CODED** — real `TryFrom` impl exists in `src/`, container-verified
  (`cargo test` passing, see this round's report).
- **TABLE ONLY** — mapping designed below, no code yet. `moveit_msgs` itself
  is not built into `ros/`'s image this round (see `ros/Dockerfile`'s header
  comment) — only `geometry_msgs`/`std_msgs`/etc. primitives are, so nothing
  targeting `moveit_msgs::msg::*` can compile here yet regardless of design
  readiness.
- **DEFERRED** — out of round-1 scope by the brief (service/action layer,
  scene subscription) or blocked on an owner decision named below.

## 0. The orphan-rule convention (applies to every row below)

Every `TryFrom` in this crate targets a local newtype wrapper around the
`r2r`-generated message type, never the bare `r2r` type directly against a
bare core-crate type. See `src/lib.rs`'s doc comment for the full mechanism
and an empirical `E0117` reproduction. This is not one-off boilerplate: it
applies to *every* row in every table below, including ones not yet coded.

## 1. Geometry primitives — **CODED** (`src/geometry.rs`)

| `geometry_msgs`/`std_msgs` | Core type | Direction | 1:1? | Notes |
|---|---|---|---|---|
| `Point{x,y,z: float64}` | `moveit_geometry::Vector3` (`nalgebra::Vector3<f64>`) | both | Not 1:1 (many-to-one) | `Point` and `Vector3` msgs both land on the same core type — core has no separate `Point`. Matches upstream (`tf2_eigen` also treats both as `Eigen::Vector3d`), not a narrowing this port introduced. Both directions total (`Error` present only for D6's uniform `TryFrom` surface). |
| `Vector3{x,y,z: float64}` | `moveit_geometry::Vector3` | both | Not 1:1 (many-to-one) | Same target as `Point`, see above. |
| `Quaternion{x,y,z,w: float64}` | `moveit_geometry::UnitQuaternion` (`nalgebra::UnitQuaternion<f64>`) | both | **Not 1:1 (lossy/fallible msg→core)** | Msg→core **fails** iff norm is zero or non-finite (e.g. the wire default `{0,0,0,0}` for an unset field — the concrete case the brief asks for one test per failure condition). A merely *near*-unit input is silently renormalized (`UnitQuaternion::new_normalize`), matching upstream's own unconditional `Eigen::Quaterniond::normalized()` in its msg→Eigen conversions — this is reading wire rounding noise, not defaulting on failure. Core→msg is always `Ok` (a `UnitQuaternion` is unit by construction). |
| `Pose{position: Point, orientation: Quaternion}` | `moveit_geometry::Isometry3` (`nalgebra::Isometry3<f64>`) | both | Not 1:1 (fails exactly when `Quaternion`'s conversion does) | Composed from the two rows above; no independent failure mode. |
| `Transform{translation: Vector3, rotation: Quaternion}` | `moveit_geometry::Isometry3` | — | same shape as `Pose` | **Not coded this round** — same failure/success logic as `Pose`, just different field names on the wire side (`sensor_msgs/MultiDOFJointState.transforms` and `geometry_msgs/TransformStamped` both use this). Add when a caller needs it (§5, §6 below). |
| `std_msgs/Header{stamp, frame_id}` | *(none)* | msg→core only, lossy | **N/A** | No core type carries a frame_id or timestamp at all (confirmed against `moveit-geometry`, `moveit-model`, `moveit-state`; see §7). Every message type that embeds a `Header` (`PositionConstraint`, `OrientationConstraint`, `CollisionObject`, `JointTrajectory`, `JointState`, ...) loses `stamp` and `frame_id` on the msg→core direction unless the caller captures them separately before calling into a core-type constructor that itself takes a `frame_id: &str` parameter (`PositionConstraint::new`, `OrientationConstraint::new`, `VisibilityConstraint`'s `SensorSpec`/`TargetSpec` all do — see §4-§6). There is no core→msg direction to speak of: nothing produces a `Header` to convert into. |

## 2. `MoveItErrorCodes` — no conversion needed, already wire-exact

`crates/moveit-error::MoveItErrorCode` is declared (its own doc comment)
"wire-exact with `moveit_msgs/msg/MoveItErrorCodes.msg` specifically so a ROS
interop crate can reuse it without its own lookup table" — confirmed against
the actual `.msg` (`val: int32` plus 29 named constants, `SUCCESS=1`,
`FAILURE=99999`, etc.):
`MoveItErrorCode::as_i32()`/`From<i32>` already round-trip every value,
including unrecognized ones (`Unknown(i32)` catch-all, so this direction
alone is total, not fallible — `From`, correctly, not `TryFrom`). Wrapping
`r2r::moveit_msgs::msg::MoveItErrorCodes` (`{val: i32}`) in a one-line local
newtype and delegating to the existing `From<i32>`/`as_i32()` is all that a
later round needs; no new logic.

## 3. `JointLimits` — **TABLE ONLY**, ready to code once `moveit_msgs` is in the image

| `moveit_msgs/JointLimits` field | `moveit_model::JointLimits` field | 1:1? |
|---|---|---|
| `joint_name: string` | `joint_name: String` | yes |
| `has_position_limits/min_position/max_position` | same names, same types | yes |
| `has_velocity_limits/max_velocity` | same names, same types | yes (upstream and core both assume `min_velocity = -max_velocity`, so there is no separate `min_velocity` field on either side) |
| `has_acceleration_limits/max_acceleration` | same | yes |
| `has_jerk_limits/max_jerk` | same | yes |

Field-name and field-type identical, confirmed against
`crates/moveit-model/src/joint/bounds.rs:85-111`
(`JointModel::variable_bounds_msg()` already builds this shape from a
`VariableBounds`, per `crates/moveit-model`'s own survey). This is a total
`From`-shaped conversion in both directions in practice; still `TryFrom` per
D6's uniform surface.

## 4. `JointConstraint` — **TABLE ONLY**

| `moveit_msgs/JointConstraint` field | `moveit_constraints::JointConstraint` field | 1:1? |
|---|---|---|
| `joint_name: string` | `joint_variable_name: String` (+ private `local_variable_name`/`variable_index`, resolved against a `RobotModel` at construction) | **Not 1:1** — core's `JointConstraint::new` needs a `&RobotModel` to resolve `joint_name` into a variable index (rejects unknown names: this is the `UnknownName` failure case). The wire string alone is not enough; msg→core conversion is not a pure function of the message, it also takes the model. |
| `position: float64` | `position: f64` | yes |
| `tolerance_above/tolerance_below: float64` | same names | yes — no redesign needed here, unlike Orientation/Visibility below |
| `weight: float64` | `weight: f64` | yes, unconditional (no `has_weight` companion on either side) |

No `frame_id` on either side — a joint-space constraint is inherently
frame-free, consistent with D6, and the only genuine name here (not
frame-name).

## 5. `PositionConstraint` — **TABLE ONLY**

```
std_msgs/Header header
string link_name
geometry_msgs/Vector3 target_point_offset
BoundingVolume constraint_region      # {primitives[], primitive_poses[], meshes[], mesh_poses[]}
float64 weight
```

| Wire | Core (`moveit_constraints::PositionConstraint`) | 1:1? |
|---|---|---|
| `header.frame_id` | `reference_frame()` (inside private `ReferenceFrame::{Fixed,Mobile}`) | Not exactly 1:1: core's `Fixed`/`Mobile` split (pre-resolved-once vs. re-resolved-per-`decide()`) has **no wire representation at all** — `moveit_msgs::PositionConstraint` carries only the one `frame_id`; upstream MoveIt decides fixed-vs-mobile by checking, at construction time, whether the frame name is a known robot link and re-resolving per-call only if so. `moveit-ros`'s `TryFrom` needs the same robot-model-aware lookup, not a pure message-field read — same shape as `JointConstraint`'s `joint_name` above, i.e. this conversion also needs a `&RobotModel` in scope, not just the message. |
| `link_name: string` | `link_name: String` | yes, but see `UnknownName` failure below |
| `target_point_offset: Vector3` | `offset: Vector3` (core) | yes, via §1's `Vector3` conversion (total) |
| `constraint_region: BoundingVolume` | `ConstraintRegion{body: Body, pose: Isometry3}` **inside** `Fixed`/`Mobile`'s `regions: Vec<ConstraintRegion>` | **Not 1:1 (structural, and the sharpest failure surface in this message)**. Wire `BoundingVolume` is 4 **parallel arrays** (`primitives[]`/`primitive_poses[]`, `meshes[]`/`mesh_poses[]`) that must be equal-length pairwise or the message is malformed (nothing in the IDL enforces this — the brief's "길이가 어긋난 병렬 배열" case, concretely instantiated here). Core's `ConstraintRegion` is one `{body, pose}` struct per region, `body: Body` being a sum type over sphere/cylinder/cuboid/mesh (from `moveit_geometry`). A `TryFrom` must: (a) reject `primitives.len() != primitive_poses.len()` and `meshes.len() != mesh_poses.len()` explicitly (a length mismatch is exactly the kind of malformed input `From` would silently truncate-or-panic on and `TryFrom` must reject with a named error), (b) convert each `shape_msgs/SolidPrimitive` to a `moveit_geometry::Shape`/`Body` (not scoped by this round — `moveit-collision`/`moveit-geometry`'s `shape_msgs` mapping is its own table, not attempted here), (c) `BoundingVolume` has no `planes[]` field at all (unlike `CollisionObject`, §8) so core's `Body` variants reachable from a `PositionConstraint` are a strict subset of what `CollisionObject` can carry. |
| `weight: float64` | `weight: f64` | yes |

No tolerance field on the wire (matches core: tolerance is expressed entirely
by the `constraint_region`'s geometry/size on both sides — not a gap, a
correct match).

## 6. `OrientationConstraint` — **TABLE ONLY**

```
std_msgs/Header header
geometry_msgs/Quaternion orientation
string link_name
float64 absolute_x_axis_tolerance
float64 absolute_y_axis_tolerance
float64 absolute_z_axis_tolerance
uint8 parameterization   # XYZ_EULER_ANGLES=0 (default), ROTATION_VECTOR=1
float64 weight
```

| Wire | Core (`moveit_constraints::OrientationConstraint`) | 1:1? |
|---|---|---|
| `header.frame_id` | `reference_frame()` inside `OrientationTarget::{Fixed,Mobile}` | same not-1:1 shape as `PositionConstraint.header.frame_id` above (needs `&RobotModel`, Fixed/Mobile is not on the wire). |
| `orientation: Quaternion` | `desired_r_in_frame_id: Rotation3` (+ `OrientationTarget`'s `rotation_matrix`/`rotation_matrix_inv`) | via §1's `Quaternion` conversion (**can fail: degenerate quaternion**), then quaternion→rotation-matrix (total once a valid `UnitQuaternion` exists). |
| `link_name: string` | `link_name: String` | yes, `UnknownName` failure possible (link not in model) |
| `absolute_x/y/z_axis_tolerance` + `parameterization` (2 wire fields → 1 tagged enum) | `tolerance: OrientationTolerance::{XyzEuler{x,y,z}, RotationVector{x,y,z}}` | **Not 1:1 (needs an explicit, named decode, not a derived-discriminant cast)**. `parameterization=0` (`XYZ_EULER_ANGLES`, the *default* value of an unset `uint8` field) must map to `XyzEuler`, `parameterization=1` (`ROTATION_VECTOR`) to `RotationVector`; **any other value (2-255) is invalid and must be an explicit `TryFrom` failure**, not silently coerced to one of the two variants — the message's own comment only documents 0/1 as meaningful. |
| `weight: float64` | `weight: f64` | yes |

## 7. `VisibilityConstraint` — **TABLE ONLY**

```
float64 target_radius
geometry_msgs/PoseStamped target_pose
int32 cone_sides
geometry_msgs/PoseStamped sensor_pose
float64 max_view_angle
float64 max_range_angle
uint8 SENSOR_Z=0
uint8 SENSOR_Y=1
uint8 SENSOR_X=2
uint8 sensor_view_direction
float64 weight
```

| Wire | Core (`moveit_constraints::VisibilityConstraint`) | 1:1? |
|---|---|---|
| `target_radius: float64` (0.0 = disabled, by convention, not enforced by the type) | `target_radius: Option<f64>` | **Not 1:1 (sentinel → `Option`)**. `crates/moveit-constraints`'s own `normalize_criterion` treats near-zero as `None`; the exact epsilon it bisects against needs to be reused here, not re-derived, once this is coded (named as a follow-up, not re-measured in this doc). |
| `max_view_angle`, `max_range_angle` | same `Option<f64>` pattern | same as above |
| `target_pose: PoseStamped` (`{header, pose}`) | `target: FramedPose::{Fixed,Mobile}{frame, pose}` | header.frame_id → Fixed/Mobile lookup (same not-1:1 shape as §5/§6), `pose` → `Pose` conversion (§1, can fail on degenerate quaternion) |
| `sensor_pose: PoseStamped` | `sensor: FramedPose` | same as `target_pose` |
| `cone_sides: int32` | `cone_sides: usize` | **Not 1:1**: wire allows negative or 0/1/2 (meaningless — a cone needs ≥3 sides per the message's own doc comment, `"This value should always be 3 or more"`, not enforced by the type); a `TryFrom` must reject `cone_sides < 3` explicitly (this is the exact upstream-documented-but-type-unenforced case D6 is written for) as well as reject negative values before the `i32 → usize` cast (a naive `as usize` cast on `-1i32` silently becomes `usize::MAX`, which the brief's "실패가 조용한 기본값이 된다" warns about almost verbatim). |
| `SENSOR_Z=0`/`SENSOR_Y=1`/`SENSOR_X=2`, `sensor_view_direction: uint8` | `SensorViewDirection::{SensorX, SensorY, SensorZ}` | **Not 1:1 — a landmine, not just a gap.** The core enum's *declared* variant order is `SensorX, SensorY, SensorZ` (natural reading order), but the *wire* encoding is the reverse (`SensorZ=0, SensorY=1, SensorX=2` — confirmed both from the `.msg` constants above and from `moveit-constraints`'s own doc comment on `axis_column()`, which spells out "upstream indexes this as `col(2 - sensor_view_direction_)`"). A conversion written by matching on derived/positional discriminants (e.g. `unsafe { transmute }`, or `[SensorX, SensorY, SensorZ][val as usize]`) would silently swap X and Z. The `TryFrom` must match the three named wire constants explicitly (`0 => SensorZ, 1 => SensorY, 2 => SensorX, _ => Err(...)`), never positionally. No existing wire-conversion helper is on `SensorViewDirection` today (`crates/moveit-constraints/src/visibility.rs` searched; none) — this is entirely `moveit-ros`'s work, per D2, not something to ask the `moveit-constraints` owner to add. |
| `weight: float64` | `weight: f64` | yes |

## 8. `Constraints` (top-level, → `KinematicConstraintSet`) — **TABLE ONLY**

```
string name
JointConstraint[] joint_constraints
PositionConstraint[] position_constraints
OrientationConstraint[] orientation_constraints
VisibilityConstraint[] visibility_constraints
```

vs. `moveit_constraints::KinematicConstraintSet{ constraints: Vec<Constraint> }`
where `Constraint` is `enum { Joint, Position, Orientation, Visibility }`
(one flat vec of a sum type, not four parallel arrays):

- **`name: string` has no home on the core side at all** — `KinematicConstraintSet` carries no name field (confirmed: its whole surface is `push`/`constraints`/`is_empty`/`len`/`decide`/`decide_each`). msg→core drops it; core→msg has nothing to put there (empty string, or the caller must carry the name out-of-band if it matters for a later re-serialization — named here, not resolved, since no round-1 code depends on it).
- msg→core: iterate all 4 arrays in order, `push`ing one `Constraint::X(...)` per element (using §4-§7's per-element conversions, any one of which can fail the whole `Constraints`).
- core→msg: the reverse -- partition the flat `Vec<Constraint>` back into 4 arrays by variant. This is a real many-to-one/one-to-many pair: the wire's 4-array-of-arrays shape and the core's 1-array-of-sum-type shape carry the same information but the array **order across types is not preserved** by either side's natural iteration (e.g. a wire message with `[joint, position, joint]` order becomes core `[joint, joint, position]`-then-`[position]` on the way back, i.e. two joints then a position — **round-trip is not order-identical across constraint *types*, only within each type**). Worth flagging explicitly since D6 asks for exactly this kind of non-identity.

## 9. `RobotState` — **TABLE ONLY**, composed from multiple core sources

```
sensor_msgs/JointState joint_state              # {header, name[], position[], velocity[], effort[]}
sensor_msgs/MultiDOFJointState multi_dof_joint_state  # {header, joint_names[], transforms[], twist[], wrench[]}
AttachedCollisionObject[] attached_collision_objects
bool is_diff
```

There is **no single core type isomorphic to `moveit_msgs/RobotState`** —
confirmed by survey, not assumed. A `TryFrom` targeting it must compose:

| Wire field | Core source | 1:1? |
|---|---|---|
| `joint_state.name[]`/`position[]`/`velocity[]`/`effort[]` | `RobotModel::variable_names()` zipped with `RobotState::positions()`/`velocities()`/`effort()` (each gated by its own `has_velocity`/`has_acceleration`/`has_effort` flag on the core side) | **Not 1:1**: core stores positions as one flat `Vec<f64>` indexed by *global variable index across the whole model* (or a group, via `JointModelGroup::variable_names()`); wire `JointState` is an unordered `name[]`/`value[]` pair with **no ordering guarantee at all** (upstream code look up by name, never by position). msg→core must build a name→index map and reject any name not in `RobotModel::variable_names()` (`UnknownName`) rather than positionally zip. Wire also allows `position[]`/`velocity[]`/`effort[]` to have *different lengths* from `name[]` or from each other (`"All arrays in this message should have the same size, or be empty"` — a convention, not a type-enforced invariant) — each independently must be validated before use, another concrete `TryFrom`-must-reject case matching the brief's "길이가 어긋난 병렬 배열." |
| `multi_dof_joint_state.{joint_names,transforms,twist,wrench}` | **no core equivalent** | **Genuine gap.** Per `moveit-state`'s survey: a floating/planar *virtual joint*'s variables live inside the same flat `positions` vec as every other joint (e.g. `virtual_joint/trans_x`), never split into a separate multi-DOF array. A `TryFrom` targeting `RobotState`'s `multi_dof_joint_state` would need to special-case the model's root/virtual joint's variable names and re-derive a `Transform` from them (using §1's `Transform` shape, not yet coded) — `twist`/`wrench` (velocity/force on the multi-DOF joint) have no core representation to source from at all; core→msg for those two arrays can only ever emit empty arrays, which is itself a documented loss, not an oversight. |
| `attached_collision_objects[]` | `moveit_scene::PlanningScene::attached_bodies()` — **not on `RobotState` at all** | **Structural, not a field gap.** Upstream nests attached objects inside `RobotState`; this port deliberately keeps them on `PlanningScene` instead (module doc, `attached_body.rs:12-23`: `RobotState` "does not carry that concept yet"). A `moveit_msgs::RobotState` → core conversion is therefore not just `TryFrom<RobotState msg> for moveit_state::RobotState` — it needs a `&PlanningScene` (or at minimum a `&BTreeMap<String, AttachedBody>`) in scope too, same "conversion needs more context than the message alone" shape as §4/§5/§6's frame-lookup cases, but one level higher (crate-level, not just model-level). |
| `is_diff: bool` | **no core equivalent on `RobotState`** | The diff/non-diff distinction exists only at `PlanningScene`'s level (`PlanningScene::parent().is_some()`, see §11) — `RobotState` itself has no notion of it. msg→core: if `is_diff` is set and the caller expects diff semantics, that has to be handled by the caller composing scenes, not by this conversion; core→msg: no source, must be supplied by the caller's context (0/false is not always the right default to invent silently). |

## 10. `RobotTrajectory` — **TABLE ONLY**

```
trajectory_msgs/JointTrajectory joint_trajectory   # {header, joint_names[], points[]}
trajectory_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory
```
`JointTrajectoryPoint = {positions[], velocities[], accelerations[], effort[], time_from_start: Duration}`

vs. `moveit_trajectory::RobotTrajectory{ waypoints: VecDeque<RobotState>, duration_from_previous: VecDeque<f64> }`:

| Wire | Core | 1:1? |
|---|---|---|
| `joint_trajectory.joint_names[]` | `RobotModel`/`JointModelGroup::variable_names()` | same name-vs-index reconciliation as `RobotState.joint_state` above |
| `points[i].{positions,velocities,accelerations,effort}` | `waypoints[i]`, a **full `RobotState`** per point (not a lightweight point struct) | Same per-point mapping as `RobotState.joint_state`, applied waypoint-by-waypoint. Each of velocities/accelerations/effort is independently optional on both sides (core's `has_velocity`/`has_acceleration`/`has_effort`, wire's "may be empty") — must line up per-point, and upstream allows this to vary *point to point* within one trajectory (a real `JointTrajectory` from a real planner may have velocities on some points and not others), which core's per-`RobotState` flags can represent identically — the fiddly part is only bookkeeping through the loop, not a structural mismatch here. |
| `points[i].time_from_start: Duration` (cumulative from trajectory start) | `duration_from_previous[i]` (**per-segment delta**, `way_point_duration_from_start(i)` computes the cumulative sum on demand) | **Not 1:1 (different accumulation basis).** msg→core: `duration_from_previous[i] = time_from_start[i] - time_from_start[i-1]` (and must be `>= 0` — a decreasing `time_from_start` across points is malformed input, another explicit-rejection case; core's own setters already enforce `duration_from_previous[0] == 0` structurally, matching wire point 0's `time_from_start` implicitly being the start). core→msg: the reverse cumulative sum, via `way_point_duration_from_start(i)` which already exists. Not lossy, just not a direct field copy — flagging so no one is tempted to copy `duration_from_previous[i]` straight into `time_from_start[i]` (an off-by-accumulation bug, not a compile error). |
| `multi_dof_joint_trajectory` | **no core equivalent**, same reasoning as `RobotState.multi_dof_joint_state` (§9) | same gap, same fix shape (virtual-joint special case), not yet coded |

## 11. `PlanningScene`/`PlanningSceneWorld`/`CollisionObject`/`AttachedCollisionObject` — **TABLE ONLY**

`PlanningScene` fields not already covered by `RobotState` (§9):

| Wire | Core (`moveit_scene::PlanningScene`) | 1:1? |
|---|---|---|
| `name: string` | `name()`/`set_name()` | yes |
| `robot_model_name: string` | **no field** | Core carries a live `&RobotModel` reference, never just its name string; msg→core can only use this to *validate against* an already-loaded model (`RobotModel::name()`), not to load one — a `PlanningScene` can't be constructed from the message alone, it needs the model supplied out-of-band. Not really a "loss," but worth naming since it means this `TryFrom` is never `TryFrom<PlanningScene msg>` alone, always `(&RobotModel, PlanningScene msg) -> ...`-shaped. |
| `fixed_frame_transforms: TransformStamped[]` | `Transforms` (`transforms()`/`transforms_mut()`) | not read this round — `moveit_geometry::Transforms`'s own field layout is out of this survey's scope; named as a follow-up |
| `allowed_collision_matrix` | `AllowedCollisionMatrix` | **not read this round** — defined in `moveit-collision`, out of this survey's scope per the round-1 brief's crate list; its own mapping table is follow-up work, not attempted here (the wire type itself was pulled above, §-adjacent, for completeness only) |
| `link_padding[]`/`link_scale[]` | **no field on `PlanningScene` at all** | **Genuine gap, not yet a documented one anywhere else.** These live on `moveit_collision::LinkPaddingScale`, passed as a separate argument to collision-checking calls, not stored on the scene. A `TryFrom<PlanningScene msg> for moveit_scene::PlanningScene` cannot round-trip this data through the scene type at all — it would need to return a second value (`(PlanningScene, LinkPaddingScale)`) alongside the scene, or drop it, and either choice needs sign-off since it changes the shape of the conversion's return type, not just its error cases. |
| `object_colors: ObjectColor[]` | **D1-excluded entirely** | `moveit-scene`'s own doc comment already states this needs `std_msgs::msg::ColorRGBA` and is out of scope by D1 (core is ROS-independent) — msg→core always drops this array; there is no core representation to build it back from on core→msg either. Not new, just confirmed still true. |
| `world: PlanningSceneWorld{collision_objects[], octomap}` | `World` (`moveit-collision`, used via `PlanningScene::world()`) | `collision_objects[]` mapping is `CollisionObject`, below; `octomap: octomap_msgs/OctomapWithPose` mapping is **not read this round** (`moveit-octomap` is out of the requested crate list) |
| `is_diff: bool` | **not a stored field — structural** | `parent: Option<Arc<PlanningScene>>` being `Some` **is** "is a diff" on the core side. msg→core: `is_diff=true` implies the conversion must be handed a parent scene to attach to (again, more context than the message alone carries); core→msg: derive as `scene.parent().is_some()`, a pure function of the core value, no loss. |

`CollisionObject` (nested inside `PlanningSceneWorld.collision_objects[]`,
and inside `AttachedCollisionObject.object` below):

| Wire | Core (`moveit_scene::AttachedBody` for the attached case; world objects are `moveit-collision::World`, not read this round) | 1:1? |
|---|---|---|
| `header` | dropped, same as §1's `Header` row | lossy, documented once, applies here too |
| `pose: Pose` + `primitives[]`/`primitive_poses[]` (+ meshes, + planes) | `AttachedBody::shapes()` + `shape_poses()`, **one-level** (each shape's pose is already resolved relative to the attach link directly) | **Not 1:1 (composition collapsed).** Wire composes two levels — object pose × each primitive's own pose — core flattens to one: `shape_poses()` are relative to the link directly, per `attached_body.rs:25-33`'s own module doc (an explicit design deviation from upstream's two-level `pose_`/`shape_poses_`, already recorded there, not new). msg→core must pre-multiply `pose * primitive_poses[i]` (and `pose * mesh_poses[i]`) before storing; core→msg has no way to recover a meaningful "object pose" to factor back out (any decomposition is a `moveit-ros` policy choice, e.g. always emit `pose = identity` and put everything in `primitive_poses`/`mesh_poses` — needs naming/sign-off, not resolved here). |
| `planes[]`/`plane_poses[]` | `moveit_geometry::Plane{a,b,c,d}` (a `Shape::Plane` variant does exist -- confirmed by reading `crates/moveit-geometry/src/shapes.rs:989-1000` directly, correcting an earlier pass over this table that assumed otherwise) | `shape_msgs/Plane` is `{coef: float64[4]}` (`coef[0..3]` = `a,b,c,d` per the `.msg`'s own comment) vs. core's 4 named fields -- 1:1 field-for-field once unpacked, **but** `coef` is IDL-fixed-size-4 while nothing in the message type stops a malformed producer from sending a different length depending on how `r2r` represents a fixed-size array (`Vec<f64>` vs `[f64; 4]` -- **not checked this round**, needs confirming against `r2r`-generated code before this row can move to CODED); if it's a `Vec`, a length check is another explicit-rejection case in the same family as `BoundingVolume`'s parallel arrays (§5). |
| `id: string` | `AttachedBody::id()` | yes |
| `type: object_recognition_msgs/ObjectType` | **no core equivalent found** | not read further this round (would need an `moveit-scene`/`moveit-collision` field search beyond this survey's scope) |
| `subframe_names[]`/`subframe_poses[]` | `subframe_pose(name)`/`subframe_names()` | yes, but wire's parallel-array length-mismatch risk applies here too (same class as §5's `BoundingVolume`) |
| `operation: byte` (`ADD=0/REMOVE=1/APPEND=2/MOVE=3`) | **no field — expressed as which method is called** (`PlanningScene::attach`/`attach_new`/`detach`) | **Structural, matches upstream's own `processAttachedCollisionObjectMsg` branching** (`moveit-scene`'s own doc comment cites this explicitly) — not a loss, a different encoding of the same dispatch. A `TryFrom<AttachedCollisionObject msg> for AttachedBody` alone cannot capture `operation`; it has to return an enum/tagged value telling the caller *which `PlanningScene` method to call*, not just a plain `AttachedBody`. |

`AttachedCollisionObject`-only fields (wrapping the `CollisionObject` above):

| Wire | Core | 1:1? |
|---|---|---|
| `link_name: string` | `AttachedBody::link_name()` | yes |
| `touch_links[]` | `AttachedBody::touch_links(): &BTreeSet<String>` | yes (`Vec` → `BTreeSet`: wire allows duplicates, core's set silently dedupes — worth a one-line note in the eventual impl doc, not a failure case) |
| `detach_posture: trajectory_msgs/JointTrajectory` | **D1-excluded** (`attached_body.rs:45`, explicit `"D1"` in the core source) | confirmed still true, not new |
| `weight: float64` | **no field anywhere on `AttachedBody`** | **Genuine gap, not previously documented elsewhere.** msg→core: silently dropped (there is nowhere to put it); core→msg: nothing to source it from, must default (and "if known" in the wire comment already signals upstream itself treats 0/absent as the common case, so `0.0` as the core→msg default is at least consistent with upstream's own convention — still worth a one-line comment at the call site rather than an unstated default). |

## 12. Every "코어에 없는 필드" — cross-reference (brief's item 3, bullet 1)

Every wire field with no core-side representation, gathered in one place so
none is only discoverable by reading a specific row above:

- `std_msgs/Header` (`stamp`, `frame_id`) wherever embedded — §1, applies everywhere a message nests a `Header`.
- `sensor_msgs/MultiDOFJointState`'s `twist[]`/`wrench[]` — §9.
- `moveit_msgs/RobotState.is_diff` — §9 (the *distinction* exists one level up, on `PlanningScene`, but not on `RobotState` itself).
- `moveit_msgs/PlanningScene.{link_padding[], link_scale[], object_colors[]}` — §11.
- `moveit_msgs/AttachedCollisionObject.{detach_posture, weight}` — §11.
- `moveit_msgs/Constraints.name` — §8.

## 13. Every "코어에만 있는 것" — cross-reference (brief's item 3, bullet 3)

- `moveit_model::LinkModel`'s `mass`/`center_of_mass`/`inertia` — no wire counterpart in any message this round touched; upstream's own `LinkModel` doesn't carry these either (a core-only addition, documented in `link_model.rs`'s own deviation-6 comment). Lost on core→msg for any conversion that would want to emit link dynamics (none of this round's messages do).
- `moveit_constraints::PositionConstraint`/`OrientationConstraint`'s `Fixed`/`Mobile` re-resolution-strategy split — §5/§6, an internal optimization with no wire field to carry it; core→msg direction just drops which variant was chosen (both serialize to the same `frame_id` string).

## 14. Round-trip non-identity — the brief's "가장 중요한 발견"

Every case above where `core -> msg -> core` (or `msg -> core -> msg`) is
**not** the identity, gathered:

1. **Quaternion renormalization** (§1): a near-unit input's exact wire bits
   are not preserved core→msg→core if the input wasn't already bit-exact
   unit norm — the *value* round-trips within float precision, the *bits*
   do not. Deliberate (matches upstream), not a bug.
2. **`Constraints` array-order across types** (§8): wire
   `[joint, position, joint]` → core flat vec → wire again does not
   reproduce the original interleaving, only the per-type sub-order.
3. **`RobotState.is_diff`** (§9) and **`PlanningScene.is_diff`** (§11): the
   wire boolean and the core `parent: Option<Arc<...>>` carry the same
   *information* but core→msg synthesizes it fresh each time — a
   `PlanningScene` built directly (not via msg→core) with a parent has
   `is_diff` "true" on the way out despite never having come from a message
   that said so. Not really non-identity so much as "the flag is derived,
   not stored" — flagged because a naive implementer might expect to find a
   stored bool to copy.
4. **`AttachedCollisionObject`'s two-level→one-level pose collapse** (§11):
   `pose × primitive_poses[i]` msg→core, then **no defined inverse** core→msg
   (any split back into `pose` + `primitive_poses[i]` is a policy choice,
   not a mathematical fact) — this is not "not identity," it's "the reverse
   direction is underdetermined," a stronger claim worth its own line.
5. **`RobotTrajectory`'s cumulative-vs-delta duration** (§10): value-identical
   round trip (the cumulative sum and its deltas carry the same information
   losslessly), called out only because a direct field-copy implementation
   would silently produce a *value*-non-identical round trip — a
   correctness risk, not a designed loss.

## 15. What still needs a decision (not resolved in this doc)

- **`AttachedCollisionObject.pose`/`primitive_poses` split on core→msg**
  (§11, item 4 above) — needs a named policy (e.g. "core→msg always emits
  `pose = Isometry3::identity()`, every shape's resolved pose goes into
  `primitive_poses`/`mesh_poses`") before that direction can be coded.
- **`PlanningScene.link_padding`/`link_scale`** (§11) — needs a decision on
  whether `TryFrom<PlanningScene msg>` returns a tuple
  `(PlanningScene, LinkPaddingScale)` or drops the data, since neither is a
  "just write the obvious code" case.
- **`moveit_geometry::Plane` vs `shape_msgs/Plane` field shape** (§11) —
  flagged as unread this round; blocks `CollisionObject.planes[]` moving
  from TABLE ONLY to CODED.
- **`moveit-collision`'s `AllowedCollisionMatrix`/`World`/`LinkPaddingScale`
  field layouts** — out of this round's requested crate survey entirely;
  every row above that touches them is a stub, not a real mapping.
