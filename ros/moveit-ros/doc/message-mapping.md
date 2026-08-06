# `moveit_msgs`/`geometry_msgs`/... ↔ core-crate type mapping

Phase 9 round 1 (`PORTING-PLAN.md` §5, §129). Every `moveit_msgs` type this
round could reach, its core-crate counterpart, and every spot where the
mapping is not 1:1 -- per the round-1 brief: "대응이 1:1이 아닌 자리를 전부
이름으로 적어라." Message field text below is quoted from
`/ws/src/moveit_msgs/msg/*.msg` inside `moveit-rs/oracle:8ed8a9395b730b08`
(the pinned upstream checkout the oracle image builds from) and from
`/opt/ros/rolling/share/{trajectory_msgs,sensor_msgs}/msg/*.msg` inside the
same image -- read directly, not recalled from memory.

**Round 2 update**: `moveit_msgs` is now built into `ros/`'s image
(`ros/Dockerfile`), so every **TABLE ONLY** row from round 1 that this
round's brief prioritized is now **CODED** — §4 (`src/constraints/joint.rs`),
§5 (`src/constraints/position.rs`), §6 (`src/constraints/orientation.rs`),
§7 msg→core only (`src/constraints/visibility.rs`), §8
(`src/constraints/set.rs`), §9 (`src/state.rs`), §10's `joint_trajectory`
field (`src/trajectory.rs`, `trajectory_msgs/JointTrajectory` directly) and
full `moveit_msgs/RobotTrajectory` (`src/planning.rs`), and a new §16 for
`MotionPlanRequest`/`MotionPlanResponse` (`src/planning.rs`). Two round-1
claims below were corrected in the process (marked **[R2 CORRECTION]**
inline) after `moveit_msgs` became available to check them against.
§3/§11 remain **TABLE ONLY** — outside this round's priority list.

**Round 3 update**: §11 is now **CODED** — `CollisionObject`
(`src/scene/collision_object.rs`), `AttachedCollisionObject`
(`src/scene/attached.rs`), `shape_msgs::{Mesh,Plane}`
(`src/scene/shapes.rs`), and `PlanningSceneWorld`'s two named fields
plus `PlanningScene.is_diff`/`robot_model_name` (`src/scene/planning_scene.rs`).
§3 remains **TABLE ONLY** (lowest priority, not reached this round).

**Round 4 update**: §3 is now **CODED** (`src/model.rs`) — confirmed
against the actual `moveit_msgs/msg/JointLimits.msg` (10 fields, `ros2`/
`humble` branches of `moveit/moveit_msgs`), not just the round-1 table.
No field was added or dropped versus the round-1 design.

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
| `Quaternion{x,y,z,w: float64}` | `moveit_geometry::UnitQuaternion` (`nalgebra::UnitQuaternion<f64>`) | both | **Not 1:1 (lossy/fallible msg→core)** | **[R15 CORRECTION, PORTING-PLAN.md §211]** This row is the *generic* rule, reached by nine of this crate's ten Quaternion/Pose conversion sites. Msg→core **fails** iff norm is zero or non-finite (e.g. the wire default `{0,0,0,0}` for an unset field — the concrete case the brief asks for one test per failure condition). Any other norm, however far from 1.0 (`norm == 2.0` included), is silently renormalized (`UnitQuaternion::new_normalize`), matching upstream's own unconditional `quaternion.normalize()` in both `planning_scene.cpp`'s `utilities::poseMsgToEigen` and `tf2_eigen.hpp`'s `fromMsg(Pose, Isometry3d&)` — round 14 (`4ff563d`) briefly narrowed this row's own threshold to `\|norm-1\|>1e-3` on the mistaken belief that all ten sites shared one upstream rule; they do not. The tenth site, `OrientationConstraint.orientation`, keeps that narrower `\|norm-1\|>1e-3` threshold under its own name (`OrientationConstraintQuaternion`, §6 below) because its own upstream reaching path (`OrientationConstraint::configure`) really does apply it — the two rules are real and distinct, not a single one described two different ways. Core→msg is always `Ok` (a `UnitQuaternion` is unit by construction). |
| `Pose{position: Point, orientation: Quaternion}` | `moveit_geometry::Isometry3` (`nalgebra::Isometry3<f64>`) | both | Not 1:1 (fails exactly when `Quaternion`'s conversion does) | Composed from the two rows above; no independent failure mode. |
| `Transform{translation: Vector3, rotation: Quaternion}` | `moveit_geometry::Isometry3` | — | same shape as `Pose` | **Not coded this round** — same failure/success logic as `Pose`, just different field names on the wire side (`sensor_msgs/MultiDOFJointState.transforms` and `geometry_msgs/TransformStamped` both use this). Add when a caller needs it (§5, §6 below). |
| `std_msgs/Header{stamp, frame_id}` | *(none)* | msg→core only, lossy | **N/A** | No core type carries a frame_id or timestamp at all (confirmed against `moveit-geometry`, `moveit-model`, `moveit-state`; see §7). Every message type that embeds a `Header` (`PositionConstraint`, `OrientationConstraint`, `CollisionObject`, `JointTrajectory`, `JointState`, ...) loses `stamp` and `frame_id` on the msg→core direction unless the caller captures them separately before calling into a core-type constructor that itself takes a `frame_id: &str` parameter (`PositionConstraint::new`, `OrientationConstraint::new`, `VisibilityConstraint`'s `SensorSpec`/`TargetSpec` all do — see §4-§6). There is no core→msg direction to speak of: nothing produces a `Header` to convert into. |

## 2. `MoveItErrorCodes` — `val` needs no conversion; `message`/`source` are a genuine gap

`crates/moveit-error::MoveItErrorCode` is declared (its own doc comment)
"wire-exact with `moveit_msgs/msg/MoveItErrorCodes.msg` specifically so a ROS
interop crate can reuse it without its own lookup table" — confirmed against
the actual `.msg`: `val: int32` plus 29 named constants (`SUCCESS=1`,
`FAILURE=99999`, etc.).
`MoveItErrorCode::as_i32()`/`From<i32>` already round-trip every `val`,
including unrecognized ones (`Unknown(i32)` catch-all, so this direction
alone is total, not fallible — `From`, correctly, not `TryFrom`).

**[R2 CORRECTION]** Round 1 stopped at `val` without checking the message
further because `moveit_msgs` was not yet in the image. Round 2 confirmed
(both from `third_party/moveit_msgs/msg/MoveItErrorCodes.msg` and the
r2r-generated struct) the wire message also carries `message: string` and
`source: string` — round 1's claim that `val` alone made this "wire-exact,
no conversion needed" was therefore incomplete: `MoveItErrorCode` has no
field for either string. This is a genuine, previously-undocumented gap:
msg→core can only use `val` and must drop `message`/`source`; core→msg has
no source for either and must emit empty strings. Wrapping
`r2r::moveit_msgs::msg::MoveItErrorCodes` in a one-line local newtype and
delegating `val` to the existing `From<i32>`/`as_i32()` (emitting
`message`/`source` as `String::new()` on the way out) is still all the *code*
a later round needs — the correction is to this doc's completeness claim,
not to the conversion's difficulty.

## 3. `JointLimits` — **CODED** (`src/model.rs`)

| `moveit_msgs/JointLimits` field | `moveit_model::JointLimits` field | 1:1? |
|---|---|---|
| `joint_name: string` | `joint_name: String` | yes |
| `has_position_limits/min_position/max_position` | same names, same types | yes |
| `has_velocity_limits/max_velocity` | same names, same types | yes (upstream and core both assume `min_velocity = -max_velocity`, so there is no separate `min_velocity` field on either side) |
| `has_acceleration_limits/max_acceleration` | same | yes |
| `has_jerk_limits/max_jerk` | same | yes |

Field-name and field-type identical, confirmed against
`crates/moveit-model/src/joint/bounds.rs:85-111` (first field `pub joint_name: String,`)
(`JointModel::variable_bounds_msg()` already builds this shape from a
`VariableBounds`, per `crates/moveit-model`'s own survey) **and, this
round, against the wire message itself** (`moveit/moveit_msgs`, `ros2`
and `humble` branches, `msg/JointLimits.msg`): exactly the 10 fields in
the table above, no `has_effort_limits`/`max_effort` or anything else —
zero fields on either side left unaccounted for. This is a total
`From`-shaped conversion in both directions in practice; still `TryFrom`
per D6's uniform surface. Coded as `JointLimitsMsg`/`JointLimitsMsgOut`
(`src/model.rs`), the same orphan-rule wrapper pair every other
conversion in this crate uses (§0).

## 4. `JointConstraint` — **CODED** (`src/constraints/joint.rs`)

| `moveit_msgs/JointConstraint` field | `moveit_constraints::JointConstraint` field | 1:1? |
|---|---|---|
| `joint_name: string` | `joint_variable_name: String` (+ private `local_variable_name`/`variable_index`, resolved against a `RobotModel` at construction) | **Not 1:1** — core's `JointConstraint::new` needs a `&RobotModel` to resolve `joint_name` into a variable index (rejects unknown names: this is the `UnknownName` failure case). The wire string alone is not enough; msg→core conversion is not a pure function of the message, it also takes the model. |
| `position: float64` | `position: f64` | yes |
| `tolerance_above/tolerance_below: float64` | same names | yes — no redesign needed here, unlike Orientation/Visibility below |
| `weight: float64` | `weight: f64` | yes, unconditional (no `has_weight` companion on either side) |

No `frame_id` on either side — a joint-space constraint is inherently
frame-free, consistent with D6, and the only genuine name here (not
frame-name).

## 5. `PositionConstraint` — **CODED** (`src/constraints/position.rs`)

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
| `constraint_region: BoundingVolume` | `ConstraintRegion{body: Body, pose: Isometry3}` **inside** `Fixed`/`Mobile`'s `regions: Vec<ConstraintRegion>` | **Not 1:1 (structural, and the sharpest failure surface in this message)**. Wire `BoundingVolume` is 4 **parallel arrays** (`primitives[]`/`primitive_poses[]`, `meshes[]`/`mesh_poses[]`) that must be equal-length pairwise or the message is malformed (nothing in the IDL enforces this — the brief's "길이가 어긋난 병렬 배열" case, concretely instantiated here). Core's `ConstraintRegion` is one `{body, pose}` struct per region, `body: Body` being a sum type over sphere/cylinder/cuboid/mesh (from `moveit_geometry`). The `TryFrom`: (a) rejects `primitives.len() != primitive_poses.len()` explicitly (`Error::Construct`), (b) rejects any non-empty `meshes[]`/`mesh_poses[]` explicitly (`Error::Other` — mesh shapes are out of scope this round, dropped-and-rejected, not silently ignored), (c) converts each `shape_msgs/SolidPrimitive` via the sub-table below. |
| `weight: float64` | `weight: f64` | yes |

No tolerance field on the wire (matches core: tolerance is expressed entirely
by the `constraint_region`'s geometry/size on both sides — not a gap, a
correct match).

**[R2 CORRECTION]** Round 1 named the `shape_msgs/SolidPrimitive` ↔
`moveit_geometry::Shape`/`Body` sub-mapping as "not scoped by this round."
Round 2 coded it (`src/constraints/position.rs`), and it contains its own
landmine, same shape as §7's `sensor_view_direction`:

| `SolidPrimitive.type_` | `Shape`/`Body` variant | `dimensions[]` order |
|---|---|---|
| `BOX=1` | `Cuboid` | `[BOX_X=0, BOX_Y=1, BOX_Z=2]` — positional, no swap (`Cuboid::new(x,y,z)`) |
| `SPHERE=2` | `Sphere` | `[SPHERE_RADIUS=0]` |
| `CYLINDER=3` | `Cylinder` | **`[CYLINDER_HEIGHT=0, CYLINDER_RADIUS=1]`** — the reverse of `Cylinder::new(radius, length)`'s argument order. A naive `dimensions[0]` → radius mapping silently swaps radius and length. |
| `CONE=4` | `Cone` at the `Shape` level, but **unconditionally rejected one layer down** | same height-then-radius order/landmine as `CYLINDER` at the `Shape::try_from(SolidPrimitiveMsg)` step — but **[R5, previously-undocumented gap]** every `CONE` constraint region then fails inside `moveit_constraints::PositionConstraint::new` (`crates/moveit-constraints/src/position.rs:181`, reads `Error::construct(format!(`, `Body::from_shape(shape)?.ok_or_else(...)`): `moveit_geometry::Body::from_shape` returns `Ok(None)` for `Shape::Cone` (drift-corrected this round: `bodies.rs:3114`, not the stale `:3065` -- `Shape::Cone(_) | Shape::Plane(_) | Shape::OcTree(_) => None,`, alongside `Shape::Plane`/`Shape::OcTree`) because `Body` (the bounding-volume sum type `ConstraintRegion` stores) has no `Cone` variant at all — only `Sphere`/`Cylinder`/`Cuboid`/`ConvexMesh`. So a `PositionConstraint` message whose `constraint_region.primitives` contains any `SolidPrimitive{type_: CONE}` always fails end-to-end, regardless of dimension order — this was true since round 2, not a regression, and had zero test coverage at either layer (`position.rs`'s own tests cover `CYLINDER`'s dimension order and `PRISM`'s rejection, never `CONE` at all). Fixed this round: added `solid_primitive_cone_dimension_order_is_height_then_radius` (mirrors the `CYLINDER` test) and `cone_constraint_region_is_rejected_end_to_end` (drives it through the full `PositionConstraint::try_from`, not just `Shape::try_from`), both in `src/constraints/position.rs`. Expires if `moveit_geometry::Body` ever grows a `Cone` variant (`moveit-geometry`'s scope, not this crate's) — until then this row's "1:1?" is **no**, not "yes with a landmine." |
| `PRISM=5` | **no counterpart** | rejected (`Error::Other`) — expires if `moveit_geometry::Shape`/`Body` ever grows a Prism variant (unlikely: `PRISM` support was removed from upstream octomap/FCL-era shape sets years ago, `moveit-geometry`'s call, not this crate's) |

msg→core also guards each `dimensions[]` index with a bounds check
(`Error::Construct` on a too-short array) rather than an `as usize`-style
panic on out-of-range indexing — the wire's own comment only promises
"length ≤ 3," never a minimum. core→msg uses `Body::dimensions()` with the
equivalent swap for `Cylinder` (`[radius, length]` on the core side →
`[length, radius]` on the wire) and explicitly rejects `Body::ConvexMesh`
(`Error::Other`, no `SolidPrimitive` representation — permanent: `shape_msgs/
SolidPrimitive` itself has no mesh-shaped variant, a wire-format fact outside
this project's control, not an absence pending a future moveit-rs change).

## 6. `OrientationConstraint` — **CODED** (`src/constraints/orientation.rs`)

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
| `orientation: Quaternion` | `desired_r_in_frame_id: Rotation3` (+ `OrientationTarget`'s `rotation_matrix`/`rotation_matrix_inv`) | **[R15, PORTING-PLAN.md §211]** *Not* §1's generic `Quaternion` conversion — this is the one site in this crate that reaches `OrientationConstraint::configure`'s own upstream rule (`kinematic_constraint.cpp:609-615`: `\|norm-1\|>1e-3` is "probably incorrect"), so it goes through the dedicated `OrientationConstraintQuaternion` (`geometry.rs`), not the generic one every other Pose/Quaternion site in this crate uses. Can fail (degenerate or, unlike §1's generic rule, merely far-from-unit quaternion), then quaternion→rotation-matrix (total once a valid `UnitQuaternion` exists). |
| `link_name: string` | `link_name: String` | yes, `UnknownName` failure possible (link not in model) |
| `absolute_x/y/z_axis_tolerance` + `parameterization` (2 wire fields → 1 tagged enum) | `tolerance: OrientationTolerance::{XyzEuler{x,y,z}, RotationVector{x,y,z}}` | **Not 1:1 (needs an explicit, named decode, not a derived-discriminant cast)**. `parameterization=0` (`XYZ_EULER_ANGLES`, the *default* value of an unset `uint8` field) must map to `XyzEuler`, `parameterization=1` (`ROTATION_VECTOR`) to `RotationVector`; **any other value (2-255) is invalid and must be an explicit `TryFrom` failure**, not silently coerced to one of the two variants — the message's own comment only documents 0/1 as meaningful. |
| `weight: float64` | `weight: f64` | yes |

## 7. `VisibilityConstraint` — **CODED, msg→core only** (`src/constraints/visibility.rs`)

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
| `cone_sides: int32` | `cone_sides: usize` | **[R14 CORRECTION]** Round 2's `TryFrom` rejected `cone_sides < 0` outright before the `i32 → usize` cast, to avoid a naive `as usize` on a negative value silently becoming `usize::MAX`. Round 14 (`p1-robotmodel` reading `kinematic_constraint.cpp:818-829` line-by-line) found that upstream's own guard is `if (vc.cone_sides < 3)`, which clamps *any* value below 3 — negative included — up to 3, and never fails; `cone_sides_` (C++ `unsigned int`) is only ever assigned from `vc.cone_sides` in the `>= 3` branch, so upstream's own guard order already prevents the wraparound without an explicit negative check. The reject was therefore stricter than both upstream and this crate's own `VisibilityConstraint::new` (which already clamps 0/1/2 up to 3). Fixed by reordering the cast: `msg.cone_sides.max(0) as usize` removes the wraparound risk (`as usize` on a non-negative `i32` is always exact) without rejecting anything the core clamp would accept anyway. Negative values (including `i32::MIN`) now clamp to 3 like every other sub-3 value, proven by `negative_cone_sides_is_clamped_to_three_not_rejected` and the boundary case `i32_min_cone_sides_does_not_wrap_around_when_cast`; `cone_sides_below_3_is_clamped_not_rejected` (round 2) still covers the non-negative sub-3 case. |
| `SENSOR_Z=0`/`SENSOR_Y=1`/`SENSOR_X=2`, `sensor_view_direction: uint8` | `SensorViewDirection::{SensorX, SensorY, SensorZ}` | **Not 1:1 — a landmine, not just a gap.** The core enum's *declared* variant order is `SensorX, SensorY, SensorZ` (natural reading order), but the *wire* encoding is the reverse (`SensorZ=0, SensorY=1, SensorX=2` — confirmed both from the `.msg` constants above and from `moveit-constraints`'s own doc comment on `axis_column()`, which spells out "upstream indexes this as `col(2 - sensor_view_direction_)`"). A conversion written by matching on derived/positional discriminants (e.g. `unsafe { transmute }`, or `[SensorX, SensorY, SensorZ][val as usize]`) would silently swap X and Z. **Fixed in round 2** (mandatory this round): the coded `TryFrom` matches the three named wire constants explicitly (`0 => SensorZ, 1 => SensorY, 2 => SensorX, _ => Err(...)`), never positionally, with a dedicated test
(`sensor_view_direction_matches_wire_constants_not_position`) that would fail under a positional/derived-discriminant cast. No existing wire-conversion helper is on `SensorViewDirection` today (`crates/moveit-constraints/src/visibility.rs` searched; none) — this is entirely `moveit-ros`'s work, per D2, not something to ask the `moveit-constraints` owner to add. |
| `weight: float64` | `weight: f64` | yes |

**[R5 CORRECTION — EXPIRED] core→msg is now implemented.** Through round 4
this row said core→msg was blocked on missing
`moveit_constraints::VisibilityConstraint` public accessors, listing
`new`/`sensor_frame()`/`target_frame()`/`cone_sides()`/`enabled()`/`decide()`/
`cone_touching_link_count()` as the type's complete surface. That claim went
stale without anyone re-checking it: `grep -n "pub fn "` against the
**current** `crates/moveit-constraints/src/visibility.rs` shows `sensor()`,
`target()`, `sensor_view_direction()`, `target_radius()`, `max_view_angle()`,
`max_range_angle()`, and `weight()` have all since landed — exactly the
accessor list this doc had been requesting. Fixed this round:
`TryFrom<VisibilityConstraint> for VisibilityConstraintMsgOut`
(`src/constraints/visibility.rs`) maps every field (`target_radius`/
`max_view_angle`/`max_range_angle` back through wire's `0.0`-means-unconstrained
convention via `.unwrap_or(0.0)`, the reverse of `normalize_criterion`), and
§8's `ConstraintsMsgOut` no longer rejects a `Constraint::Visibility` member.
Round-tripped with distinct values per field
(`round_trip_through_msg` in `visibility.rs`) so a mixed-up accessor (e.g.
`target_radius` read where `max_view_angle` belongs, or `sensor`/`target`
swapped) fails the test instead of hiding behind a repeated constant.

## 8. `Constraints` (top-level, → `KinematicConstraintSet`) — **CODED** (`src/constraints/set.rs`)

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

- **`name: string` has no home on the core side at all** — `KinematicConstraintSet` carries no name field (re-checked this round against `crates/moveit-constraints/src/set.rs:45-49`, own doc comment `kinematic_constraints::KinematicConstraintSet`: the struct is still exactly `{ constraints: Vec<Constraint> }`). msg→core drops it; core→msg has nothing to put there (empty string, or the caller must carry the name out-of-band if it matters for a later re-serialization — named here, not resolved, since no round-1 code depends on it). Expires if `KinematicConstraintSet` grows a `name` field; `moveit-constraints`'s call, not this crate's.
- msg→core: iterate all 4 arrays in order, `push`ing one `Constraint::X(...)` per element (using §4-§7's per-element conversions, any one of which can fail the whole `Constraints`).
- core→msg: the reverse -- partition the flat `Vec<Constraint>` back into 4 arrays by variant. This is a real many-to-one/one-to-many pair: the wire's 4-array-of-arrays shape and the core's 1-array-of-sum-type shape carry the same information but the array **order across types is not preserved** by either side's natural iteration (e.g. a wire message with `[joint, position, joint]` order becomes core `[joint, joint, position]`-then-`[position]` on the way back, i.e. two joints then a position — **round-trip is not order-identical across constraint *types*, only within each type**). Worth flagging explicitly since D6 asks for exactly this kind of non-identity.
- **Coded in round 2; core→msg's `Visibility` gap closed round 5.** msg→core
  is exactly the "any element failing fails the whole conversion" rule
  above, container-verified. core→msg **[R5 CORRECTION — EXPIRED]**: through
  round 4 this errored on any `Constraint::Visibility` member, inheriting
  §7's then-blocked core→msg direction. §7's accessors landed in
  `moveit-constraints` since (re-checked this round, not assumed) — a
  `KinematicConstraintSet` containing a `VisibilityConstraint` now round-trips
  through a message like every other variant (`visibility_member_round_trips`,
  `src/constraints/set.rs`).

## 9. `RobotState` — **CODED, `joint_state` portion only** (`src/state.rs`)

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
| `multi_dof_joint_state.{joint_names,transforms,twist,wrench}` | **no core equivalent** | **Genuine gap.** Per `moveit-state`'s survey (re-checked this round against `crates/moveit-state/src/state.rs` and `crates/moveit-trajectory/src/*.rs` — no `multi_dof`/`MultiDOF` field on either core type): a floating/planar *virtual joint*'s variables live inside the same flat `positions` vec as every other joint (e.g. `virtual_joint/trans_x`), never split into a separate multi-DOF array. A `TryFrom` targeting `RobotState`'s `multi_dof_joint_state` would need to special-case the model's root/virtual joint's variable names and re-derive a `Transform` from them (using §1's `Transform` shape, not yet coded) — `twist`/`wrench` (velocity/force on the multi-DOF joint) have no core representation to source from at all; core→msg for those two arrays can only ever emit empty arrays, which is itself a documented loss, not an oversight. `joint_names`/`transforms` expire (become codeable) once §1's `Transform` conversion is written and this crate special-cases the virtual joint; `twist`/`wrench` are permanent (no core field could ever source them without `moveit-state` adding velocity/force storage for the virtual joint, which is `moveit-state`'s call). |
| `attached_collision_objects[]` | `moveit_scene::PlanningScene::attached_bodies()` — **not on `RobotState` at all** | **Structural, not a field gap.** Upstream nests attached objects inside `RobotState`; this port deliberately keeps them on `PlanningScene` instead (module doc, `attached_body.rs:12-23`, opens `moveit::core::RobotState`: `RobotState` does not carry that concept yet). A `moveit_msgs::RobotState` → core conversion is therefore not just `TryFrom<RobotState msg> for moveit_state::RobotState` — it needs a `&PlanningScene` (or at minimum a `&BTreeMap<String, AttachedBody>`) in scope too, same "conversion needs more context than the message alone" shape as §4/§5/§6's frame-lookup cases, but one level higher (crate-level, not just model-level). |
| `is_diff: bool` | **no core equivalent on `RobotState`** | The diff/non-diff distinction exists only at `PlanningScene`'s level (`PlanningScene::parent().is_some()`, see §11) — `RobotState` itself has no notion of it. msg→core: if `is_diff` is set and the caller expects diff semantics, that has to be handled by the caller composing scenes, not by this conversion; core→msg: no source, must be supplied by the caller's context (0/false is not always the right default to invent silently). |

**Coded in round 2** (`joint_state` field only). msg→core resolves
`joint_state.name[]` against `RobotModel::variable_names()` (rejecting an
unknown name via `UnknownName`), validates `position[]`/`velocity[]`/
`effort[]` are each either empty or exactly as long as `name[]`
(`Error::Construct` otherwise, the "malformed parallel array" case named
above), and **rejects** (does not silently drop) `is_diff=true`,
non-empty `attached_collision_objects`, or a non-empty
`multi_dof_joint_state` — all three per D6, same rule as `PlanningRequest`'s
`start_state` guard in §16. core→msg is total. **New gap found while coding
(not in round 1's table):** `sensor_msgs/JointState` has no `acceleration`
field at all — core's `accelerations()` therefore has no wire home on this
message and is silently dropped on core→msg; this is the reverse of the
round-1 table's wire→core gaps (a core-only field with nowhere on the wire
to go, not a wire field the core drops).

## 10. `RobotTrajectory` — **CODED** (`trajectory_msgs/JointTrajectory` in `src/trajectory.rs`; full `moveit_msgs/RobotTrajectory` wrapper in `src/planning.rs`)

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
| `multi_dof_joint_trajectory` | **no core equivalent**, same reasoning as `RobotState.multi_dof_joint_state` (§9) | same gap, same fix shape (virtual-joint special case), not yet coded. Re-checked this round against `crates/moveit-trajectory/src/*.rs` — still absent. Same expiry as §9's row: codeable once §1's `Transform` conversion exists and the virtual joint is special-cased there; this row just reuses that fix once it lands. |

**Coded in round 2.** `src/trajectory.rs`'s `JointTrajectoryMsg`/
`JointTrajectoryMsgOut` handle `trajectory_msgs/JointTrajectory` directly
(the bare wire type, no `moveit_msgs` wrapper): msg→core validates
`positions.len() == joint_names.len()` per point (mandatory field),
**rejects** a nonzero `points[0].time_from_start` and a decreasing
`time_from_start` across points (`Error::Construct`, exactly the
cumulative-vs-delta risk named above), then delegates velocities/
accelerations/effort to the same length-or-empty check as §9's
`set_parallel_array`. core→msg is total, accumulating the cumulative time
from each waypoint's `duration_from_previous`.
`src/planning.rs`'s `RobotTrajectoryMsg`/`RobotTrajectoryMsgOut` wrap the
full `moveit_msgs/RobotTrajectory` message: delegates `joint_trajectory` to
the above and **rejects** a non-empty `multi_dof_joint_trajectory`
(`Error::Other`), per the gap already named in the row above — dropped is
not an option under D6 once the field is actually populated.

## 11. `PlanningScene`/`PlanningSceneWorld`/`CollisionObject`/`AttachedCollisionObject` — **CODED** (round 3, `src/scene/`)

Ported against moveit2 @ `e017c91ee12984393a28ba246075c65f69cde3bf`'s
`moveit_core/planning_scene/src/planning_scene.cpp`. Unlike every other
row in this document, `CollisionObject`/`AttachedCollisionObject` are not
a `TryFrom` in both directions: upstream's own
`processCollisionObjectMsg`/`processAttachedCollisionObjectMsg` take
`(&mut PlanningScene, &Msg) -> bool`, an imperative command against an
existing scene, not a value conversion — `apply_collision_object`/
`apply_attached_collision_object` take the same shape (`&mut PlanningScene`
in, `Result<()>` out).

Landmines confirmed against the pinned source and closed with a
regression test each (all in `src/scene/collision_object.rs` unless
noted):

- **Asymmetric parallel-array length rule** (`shapesAndPosesFromCollisionObjectMessage`,
  `:1800-1862`): more poses than shapes is **rejected**; more shapes than
  poses **tolerated**, missing trailing poses default to identity. Checked
  independently per shape type (primitives/meshes/planes), before any
  shape is constructed — not the same rule as §5's `BoundingVolume`, which
  rejects on any mismatch at all.
- **Single-shape object/shape pose swap** (`:1823-1852`): exactly one shape
  total, plus an empty (identity) `object.pose`, promotes that shape's own
  message pose to be the object pose and resets the shape's local pose to
  identity — not merely "assume identity object pose."
- **`OCTOMAP_NS` (`planning_scene.hpp:113`, `"<octomap>"`) rejected for
  every operation** (ADD/REMOVE/APPEND/MOVE), checked in the top-level
  dispatcher, plus separately in the attached-object dispatcher — not
  just for ADD/APPEND.
- **MOVE's own unconditional partial-effect order** (`processCollisionObjectMove`,
  `:1953`): the object's absolute pose is applied *before* the per-shape
  repose count is checked, so a shape-count mismatch still leaves the
  object moved. Reproduced faithfully, not "fixed."
- **AttachedCollisionObject ADD's world-object-promotion branch is gated
  on `operation == ADD` specifically** (`:1576`), not APPEND — APPEND with
  empty message geometry always fails the message-geometry path's
  shapes-non-empty check instead.
- **AttachedCollisionObject APPEND merges** shapes/poses (concatenated),
  touch_links (unioned), and subframes (message value wins on a name
  collision, matching `std::map::insert`'s keep-first semantics against
  upstream's insertion order, `:1611-1612`) onto any existing attached body
  of the same id.
- **REMOVE (detach)'s two asymmetries**, neither given for free by
  `moveit_scene::PlanningScene::detach` alone: an empty `object.id`
  detaches every (optionally link-filtered) attached body and **always
  succeeds even with zero matches**; a specific non-empty id that matches
  nothing is a **failure**; a specific id whose actual `link_name` doesn't
  match the message's stated one is a hard error (`src/scene/attached.rs`).
- **`shape_msgs::MeshTriangle.vertex_indices`/`Plane.coef` are `Vec`, not
  fixed arrays**, confirmed against `r2r`'s cached bindgen output (not
  assumed from the `.msg` source's `[3]`/`[4]` syntax) — length-checked
  rather than indexed out of bounds (`src/scene/shapes.rs`).
- One landmine deliberately **not** reproduced: if `AttachedCollisionObject.link_name`
  is given but names no real link, upstream's detach-filter falls back to
  matching *every* attached body (`:1633-1636`); this port matches zero
  instead (a bogus link name can never equal a real attached body's link),
  documented as an intentional parity deviation in `apply_detach`'s doc
  comment.

Structural gaps (no reachable `PlanningScene`-level API, named rather than
patched around):

- ~~World-object subframes (ADD/APPEND) and MOVE's per-shape repose:
  waiting on `moveit-scene`, not a permanent gap.~~ **Resolved round 6.**
  `PlanningScene::move_shapes_in_object`/`set_subframes_of_object`
  (`scene.rs:1055`, own doc comment reads `PlanningScene::move_object`/`:1078`) landed on p1-fixtures round 23 (`de8886a`,
  `PORTING-PLAN.md` §150.1 closed) — plain `bool` returns, no outcome enum
  needed, since p1-fixtures read `World::moveShapesInObject`/
  `setSubframesOfObject`'s bodies (`world.cpp:262-280`/`:365-380`) and found
  every failure mode collapses to one case (unlike `moveObject`, there is no
  "found but unchanged" branch), and none of the five call sites these
  reach (`planning_scene.cpp:393, 1201, 1743, 1927, 2004`) touch ACM/color/
  type as a side effect of the assignment/repose itself. `set_world_object_subframes`/
  `move_world_object_shapes` (`src/scene/collision_object.rs`) now call
  through to them directly, and ADD/APPEND's subframe setter call is
  unconditional again, matching upstream's own always-call behavior.
- AttachedCollisionObject's own MOVE: rejected, matching upstream, which
  has no MOVE branch in `processAttachedCollisionObjectMsg` either.
- ~~`Octomap.data`'s binary payload: decided round 5, belongs to
  `moveit-octomap` (p3-shapes), not `ros/`.~~ **Resolved round 8.**
  `OcTree::read_binary_data`/`read_data` (`crates/moveit-octomap/src/tree.rs:1244-1246` (`read_binary_data`)/`:1272`)
  landed (`7eb794c`); `apply_octomap` (`src/scene/planning_scene.rs`) now
  dispatches on `map.octomap.binary`, decodes into a freshly-constructed
  `OcTree::new(map.octomap.resolution)`, and inserts the result via
  `PlanningScene::add_shape` the same way every other shape kind is
  inserted. Round-7's own dispatch plan (`ea686a6`) had flagged one risk
  before it could be checked against a real decoder: whether the existing
  `vec![1, 2, 3]` "malformed payload" test fixture would turn out to be a
  degenerate but *valid* bitstream once a real decoder existed. It is --
  `read_binary_node` decodes the first two bytes into two leaf children
  (no `(1, 1)` "has children" code in either byte) and returns `Ok`
  without ever consuming the third, matching `read_binary_data`'s own
  documented "trailing bytes are not an error." That fixture is now
  exercised as a **success** case (`binary_octree_payload_is_decoded_and_inserted`);
  the rejection case (`truncated_octree_payload_is_rejected`) uses a
  genuinely truncated one-byte payload instead. Requirements spec below
  kept for the historical record of what was asked for and why. Upstream:
  `moveit_core`'s
  `createOctomap` (`planning_scene.cpp:1417-1435`) always constructs a
  concrete `octomap::OcTree` -- `AbstractOcTree`'s runtime-type
  registry/factory (`createTree`) is never on this call path -- then for
  `map.binary==true` calls `octomap_msgs::readTree` (`octomap_msgs`
  `ros2` branch, `include/octomap_msgs/conversions.h:70-76`), which
  writes `msg.data` into a `stringstream` and calls
  `octree->readBinaryData(datastream)` directly: no `.bt`-file header
  text, no id/resolution re-encoding (both are already separate message
  fields). For `map.binary==false` it calls `om->readData(datastream)`
  the same way (`fullMsgToMap`, `octomap_msgs/conversions.h:55-66`, is the
  identical
  pattern). `readBinaryData`/`readBinaryNode`
  (`OccupancyOcTreeBase.hxx:931-1022`, 82 lines) is a recursive
  2-bit-per-child bitstream (free-leaf/occupied-leaf/has-children/unknown,
  clamped to `clamping_thres_min`/`_max`); `readData`/`readNodesRecurs`/
  `OcTreeDataNode::readData` (`OcTreeBaseImpl.hxx:801-844` +
  `OcTreeDataNode.hxx:114-117`, 47 lines) is a recursive
  1-bit-per-child-exists bitmap plus a raw `f32` log-odds read per node --
  neither needs the registry/factory machinery (moveit_core's call site
  never exercises it) or `AbstractOcTree::read`'s file-header parsing (the
  ROS wire format skips it entirely). Every primitive the decoder would
  call -- `Node::create_child`, the `log_odds` field,
  `Node::update_occupancy_from_children`, `OcTree::root`,
  `clamping_thres_min`/`_max` -- already exists in `moveit-octomap`, but
  `Node` is `pub(crate)` and `OcTree::root` is private: `ros/`, which only
  sees `moveit-octomap`'s public surface, cannot reach any of them
  regardless of how small the decoder is. Spec for `moveit-octomap`: two
  `pub` entry points on `OcTree` that decode into an already
  resolution-constructed tree, e.g. `read_binary_data(&mut self, &[u8])
  -> Result<(), E>` / `read_data(&mut self, &[u8]) -> Result<(), E>`,
  mirroring `createOctomap`'s own `OcTree::new(resolution)`-then-decode
  shape. No external crate: no maintained octomap-format crate exists on
  crates.io (`lib.rs`'s module docs already record this), and the format
  itself carries no compression or external encoding that would justify
  one for roughly 130 lines of recursive bit/byte-stream logic. **Call
  site:** `apply_octomap` (`src/scene/planning_scene.rs`), reached from
  `apply_planning_scene_world` via `PlanningSceneWorld.octomap` --
  `Octomap.data` arrives on the `world.octomap` field of any
  `moveit_msgs/PlanningSceneWorld` this crate is given (`ApplyPlanningSceneWorld`
  and the `world` half of `PlanningScene`/diff messages both carry this
  shape); once the two entry points exist, `apply_octomap`'s current
  `Err(Error::other(...))` branch becomes `map.octomap.data` dispatched to
  `read_binary_data`/`read_data` based on `map.octomap.binary`, matching
  `createOctomap`'s own dispatch. **Verification:** bytes would come from
  the oracle -- capture a populated `octomap::OcTree` via
  `octomap_msgs::binaryMsgFromMap`/`fullMsgToMap` (same C++ call upstream's
  `moveit_core` itself uses to produce `Octomap.data`,
  `octomap_msgs/conversions.h:70-76`/`:55-66`) against a small fixture tree with a handful of updated nodes, the
  same byte-fixture pattern §149/§157 already use for other oracle
  comparisons; compare the decoded `moveit_octomap::OcTree`'s leaf
  occupancy/log-odds against the oracle's own tree via the existing
  leaf-iteration surface (`moveit-octomap`'s iterators, `src/iter.rs`), not
  a raw byte comparison (the binary format is lossy-compressed at
  `clamping_thres_min`/`_max`, so byte-for-byte round-trip is not the right
  invariant -- decoded-tree-content equality is). Until `moveit-octomap`
  adds those two entry points, an empty payload stays a correct no-op
  (matching upstream's own early return on `msg.data.size() == 0`) and a
  non-empty one is rejected rather than silently dropped.

Not attempted this round: the rest of `usePlanningSceneMsg`/`setPlanningSceneMsg`/
`setPlanningSceneDiffMsg` (`robot_state`, `fixed_frame_transforms`,
`allowed_collision_matrix`, `link_padding`/`link_scale`, `object_colors`,
diff-vs-full dispatch) — each is a separately-sized conversion in its own
right; `src/scene/planning_scene.rs` covers only `PlanningSceneWorld` and
the two small `is_diff`/`robot_model_name` helpers.

`PlanningScene` fields not already covered by `RobotState` (§9):

| Wire | Core (`moveit_scene::PlanningScene`) | 1:1? |
|---|---|---|
| `name: string` | `name()`/`set_name()` | yes |
| `robot_model_name: string` | **no field** | Core carries a live `&RobotModel` reference, never just its name string; msg→core can only use this to *validate against* an already-loaded model (`RobotModel::name()`), not to load one — a `PlanningScene` can't be constructed from the message alone, it needs the model supplied out-of-band. Not really a "loss," but worth naming since it means this `TryFrom` is never `TryFrom<PlanningScene msg>` alone, always `(&RobotModel, PlanningScene msg) -> ...`-shaped. |
| `fixed_frame_transforms: TransformStamped[]` | `Transforms` (`transforms()`/`transforms_mut()`) | not read this round — `moveit_geometry::Transforms`'s own field layout is out of this survey's scope; named as a follow-up |
| `allowed_collision_matrix` | `AllowedCollisionMatrix` | **not read this round** — defined in `moveit-collision`, out of this survey's scope per the round-1 brief's crate list; its own mapping table is follow-up work, not attempted here (the wire type itself was pulled above, §-adjacent, for completeness only) |
| `link_padding[]`/`link_scale[]` | **no field on `PlanningScene` at all** | **Genuine gap, not yet a documented one anywhere else.** These live on `moveit_collision::LinkPaddingScale` (re-checked this round, `crates/moveit-collision/src/env.rs:329-351` -- own doc comment names `Self::with_links` -- still a standalone struct, not a `PlanningScene` field), passed as a separate argument to collision-checking calls, not stored on the scene. A `TryFrom<PlanningScene msg> for moveit_scene::PlanningScene` cannot round-trip this data through the scene type at all — it would need to return a second value (`(PlanningScene, LinkPaddingScale)`) alongside the scene, or drop it, and either choice needs sign-off since it changes the shape of the conversion's return type, not just its error cases. Expires if `PlanningScene` grows a field carrying `LinkPaddingScale` (or the sign-off above picks the second-return-value shape and someone implements it here); `moveit-scene`'s call either way. |
| `object_colors: ObjectColor[]` | **D1-excluded entirely** | Re-checked this round against `crates/moveit-scene/src/scene.rs:476-477` (drift-corrected this round, was the stale `:1972`) — `moveit-scene`'s own doc comment still states this needs `std_msgs::msg::ColorRGBA` and is out of scope by D1 (core is ROS-independent) — msg→core always drops this array; there is no core representation to build it back from on core→msg either. Permanent by design, not a pending-implementation gap: only resolves if D1 itself is revisited (core stops being ROS-independent), which is a project-wide decision, not something this crate or `moveit-scene` can trigger unilaterally. |
| `world: PlanningSceneWorld{collision_objects[], octomap}` | `World` (`moveit-collision`, used via `PlanningScene::world()`) | `collision_objects[]` mapping is `CollisionObject`, below. `octomap: octomap_msgs/OctomapWithPose`: **[R3]** an empty `octomap.data` is a correct no-op (`apply_planning_scene_world`, `src/scene/planning_scene.rs`); a non-empty payload is decoded via `moveit_octomap::OcTree::read_binary_data`/`read_data` and inserted as a `Shape::OcTree` — **[R8]** see the "Structural gaps" list above for the decoder's landing and this crate's dispatch. |
| `is_diff: bool` | **not a stored field — structural** | `parent: Option<Arc<PlanningScene>>` being `Some` **is** "is a diff" on the core side. msg→core: `is_diff=true` implies the conversion must be handed a parent scene to attach to (again, more context than the message alone carries); core→msg: derive as `scene.parent().is_some()`, a pure function of the core value, no loss. |

`CollisionObject` (nested inside `PlanningSceneWorld.collision_objects[]`,
and inside `AttachedCollisionObject.object` below):

| Wire | Core (`moveit_scene::AttachedBody` for the attached case; `moveit_collision::World`, reached via `PlanningScene::world()`/`add_shape`/`move_object`/`remove_object`, for world objects — **[R3]** read and used, no longer a stub) | 1:1? |
|---|---|---|
| `header` | dropped, same as §1's `Header` row | lossy, documented once, applies here too |
| `pose: Pose` + `primitives[]`/`primitive_poses[]` (+ meshes, + planes) | `AttachedBody::shapes()` + `shape_poses()`, **one-level** (each shape's pose is already resolved relative to the attach link directly) | **Not 1:1 (composition collapsed).** Wire composes two levels — object pose × each primitive's own pose — core flattens to one: `shape_poses()` are relative to the link directly, per `attached_body.rs:25-33`'s own module doc (an explicit design deviation from upstream's two-level `pose_`/`shape_poses_`, already recorded there, not new). msg→core must pre-multiply `pose * primitive_poses[i]` (and `pose * mesh_poses[i]`) before storing; core→msg has no way to recover a meaningful "object pose" to factor back out (any decomposition is a `moveit-ros` policy choice, e.g. always emit `pose = identity` and put everything in `primitive_poses`/`mesh_poses` — needs naming/sign-off, not resolved here). |
| `planes[]`/`plane_poses[]` | `moveit_geometry::Plane{a,b,c,d}` (a `Shape::Plane` variant does exist -- confirmed by reading `crates/moveit-geometry/src/shapes.rs:989-1000` directly, own attribute `#[derive(Debug, Clone, Copy, PartialEq, Default)]`, correcting an earlier pass over this table that assumed otherwise) | `shape_msgs/Plane` is `{coef: float64[4]}` (`coef[0..3]` = `a,b,c,d` per the `.msg`'s own comment) vs. core's 4 named fields -- 1:1 field-for-field once unpacked. **[R3 CORRECTION]** `coef` is `Vec<f64>` in `r2r`'s generated bindings, not `[f64; 4]` — confirmed by reading the cached bindgen output directly, not assumed from the `.msg` source's `[4]` syntax. Length-checked (`src/scene/shapes.rs`), same family as `BoundingVolume`'s parallel arrays (§5). |
| `id: string` | `AttachedBody::id()` | yes |
| `type: object_recognition_msgs/ObjectType` | **no core equivalent** (re-checked this round against `crates/moveit-scene/src/scene.rs:478-479` — still D1-cited, `object_recognition_msgs::msg::ObjectType`) | genuinely dropped, msg→core and core→msg alike; D1-shaped (orchestration/tagging metadata, no invariant to violate), same treatment as `weight` below. Permanent by design (D1), not pending-implementation — expires only on a project-wide D1 revisit. |
| `subframe_names[]`/`subframe_poses[]` | `subframe_pose(name)`/`subframe_names()` | yes, but **[R3 CORRECTION]** upstream itself indexes these two arrays without any length check at all (`planning_scene.cpp:1596-1599`, an out-of-bounds read if `subframe_names` is shorter) — this port rejects a length mismatch instead of reproducing that read (`src/scene/attached.rs`), a deliberate parity deviation, not an oversight. |
| `operation: byte` (`ADD=0/REMOVE=1/APPEND=2/MOVE=3`) | **no field — expressed as which method is called** (`PlanningScene::attach`/`attach_new`/`detach`) | **Structural, matches upstream's own `processAttachedCollisionObjectMsg` branching** (`moveit-scene`'s own doc comment cites this explicitly) — not a loss, a different encoding of the same dispatch. `apply_attached_collision_object` (`src/scene/attached.rs`) is the dispatcher; it is not a plain `TryFrom<AttachedCollisionObject msg> for AttachedBody`, for exactly this reason. |

`AttachedCollisionObject`-only fields (wrapping the `CollisionObject` above):

| Wire | Core | 1:1? |
|---|---|---|
| `link_name: string` | `AttachedBody::link_name()` | yes |
| `touch_links[]` | `AttachedBody::touch_links(): &BTreeSet<String>` | yes (`Vec` → `BTreeSet`: wire allows duplicates, core's set silently dedupes — worth a one-line note in the eventual impl doc, not a failure case) |
| `detach_posture: trajectory_msgs/JointTrajectory` | **D1-excluded** (re-checked this round, `attached_body.rs:45`, own text reads `is not carried.`, explicit `"D1"` in the core source) | confirmed still true, not new; genuinely dropped, msg→core and core→msg alike (§11's CODED note above). Permanent by design (D1), not pending-implementation — expires only on a project-wide D1 revisit. |
| `weight: float64` | **no field anywhere on `AttachedBody`** | **Genuine gap.** msg→core: silently dropped (there is nowhere to put it, and upstream's own `processAttachedCollisionObjectMsg` never reads it back either — purely advisory metadata, "if known"); core→msg: not attempted (no core→msg direction exists for this message family at all, see the CODED note above), so there is no default-value question to resolve. Re-checked this round against `crates/moveit-scene/src/attached_body.rs:56-63` — `AttachedBody`'s field list is still `id`/`link_name`/`shapes`/`shape_poses`/`touch_links`/`subframes`, no `weight`. Expires if `AttachedBody` grows a `weight` field; `moveit-scene`'s call, not this crate's. |

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
- **`moveit-collision`'s `AllowedCollisionMatrix`/`World`/`LinkPaddingScale`
  field layouts** — out of this round's requested crate survey entirely;
  every row above that touches them is a stub, not a real mapping.
- ~~**`moveit_constraints::VisibilityConstraint` missing accessors** (§7,
  round 2)~~ — **[R5] resolved.** The requested accessors landed in
  `moveit-constraints`; core→msg is coded (§7/§8, `src/constraints/visibility.rs`).

## 16. `MotionPlanRequest`/`MotionPlanResponse` — **CODED** (`src/planning.rs`, round 2, lowest-priority item in this round's brief)

`moveit_planning::PlanningRequest`'s own doc comment
(`crates/moveit-planning/src/request.rs`) already states its scope: it
carries only the six fields its own request adapters read (`group_name`,
`goal_constraints`, `path_constraints`, `workspace_parameters` →
`workspace_bounds`, `max_velocity_scaling_factor`,
`max_acceleration_scaling_factor`) and deliberately excludes
planner-selection/tuning concerns — this is a pre-existing, documented
design choice in `moveit-planning`, not something this round's conversion
invented.

**[R3 CORRECTION]** `p1-fixtures` round 20 added
`PlanningRequest::{trajectory_constraints, planner_id}` and
`PlanningResponse::planner_id` to `moveit-planning` after this round's
round 2 was written — round 2's `..Default::default()`/`Error::other`
handling of `MotionPlanRequest.trajectory_constraints` (and the
then-correct "no field" claim for `planner_id`) went stale in the same
merge and broke this crate's build (round 3 item 0, fixed with its own
commit; `ros/moveit-ros` is outside the root workspace, D5, so no gate
saw the two crates disagree until a round-3 brief ran
`verify-ros-interop.sh` by hand). The table below is corrected to match
current `moveit-planning`.

| `MotionPlanRequest` field | `PlanningRequest` field | 1:1? |
|---|---|---|
| `workspace_parameters: WorkspaceParameters{header, min_corner, max_corner}` | `workspace_bounds: WorkspaceBounds{min_corner, max_corner}` | `header` dropped (metadata, same treatment as §1); `min_corner`/`max_corner` via §1's `Vector3` conversion (total) |
| `start_state: RobotState` | **no field** | **Rejected, not dropped**, if non-default (per D6): assuming a different start state than the one requested would change what the plan actually solves for — same reasoning as §9's `is_diff`/`attached_collision_objects` guards, applied one level up. Re-checked this round (§157 audit) against `crates/moveit-planning/src/request.rs:201-243` (field `pub group_name: String,`) — still absent. Expires if `PlanningRequest` grows a `start_state` field; `moveit-planning`'s call, not this crate's. |
| `goal_constraints: Constraints[]` | `goal_constraints: Vec<KinematicConstraintSet>` | via §8's `Constraints` conversion per element |
| `path_constraints: Constraints` (single, wire has no "unset" state — an all-empty `Constraints` is the convention) | `path_constraints: Option<KinematicConstraintSet>` | an all-4-arrays-empty `Constraints` maps to `None`; anything else via §8 |
| `trajectory_constraints: TrajectoryConstraints{constraints: Constraints[]}` | `trajectory_constraints: Vec<KinematicConstraintSet>` | **[R3]** now representable — mapped via §8's `Constraints` conversion per element, exactly like `goal_constraints` above, just a different field |
| `reference_trajectories: GenericTrajectory[]` | **no field** | **Rejected if non-empty** — real seed-trajectory content with nowhere to go. Re-checked this round against `crates/moveit-planning/src/request.rs:201-243` (field `pub group_name: String,`) — still absent. Expires if `PlanningRequest` grows a `reference_trajectories` field; `moveit-planning`'s call, not this crate's. |
| `planner_id: string` | `planner_id: String` | **[R3]** now representable — mapped directly (`""` = unset on both sides) |
| `pipeline_id`/`num_planning_attempts`/`allowed_planning_time`/`cartesian_speed_limited_link`/`max_cartesian_speed`/`smoothness_level` | **no field** | **Dropped, not rejected** — planner-orchestration metadata, no invariant a dropped tuning knob could violate, same documented-scope reasoning as `PlanningRequest`'s own doc comment on planner-specific tuning. Re-checked this round against `crates/moveit-planning/src/request.rs:210-258` (field `pub group_name: String,`) — none of these six landed. `pipeline_id` is dropped by *this conversion* only: upstream reads it one layer up, in `MoveGroupCapability::resolvePlanningPipeline` (`move_group_capability.cpp:223-246`), and so does this crate — `move_group::resolve_planning_pipeline` takes it off the message before the conversion runs, so it selects the planner without ever reaching `PlanningRequest`. Expires per-field if `PlanningRequest` grows a matching field; `moveit-planning`'s call, not this crate's. |
| `group_name: string` | `group_name: String` | yes |
| `max_velocity_scaling_factor`/`max_acceleration_scaling_factor: float64` | same names | yes |

`MotionPlanResponse` vs. `moveit_planning::PlanningResponse{ trajectory: RobotTrajectory<'m>, planner_id: String }`:

| `MotionPlanResponse` field | `PlanningResponse` field | 1:1? |
|---|---|---|
| `trajectory_start: RobotState` | `start_state: RobotState<'m>` | **[R5 CORRECTION — EXPIRED]** This row said "no field, dropped" through round 4. `PlanningResponse::start_state` landed in `moveit-planning` in the same merge that broke this crate's build (round 4→5 merge, `crates/moveit-planning/src/response.rs:121` -- own doc comment reads `before ever touching it. A`); the "no field" claim went stale at that instant and nobody re-checked it before this round. Fixed at the merge point (`c8dd883`, not redone here): mapped via §9's `RobotStateMsg`/`RobotStateMsgOut` both ways, round-trip tested with a start state distinct from the trajectory's own first waypoint so a wrong implementation that reconstructs `start_state` from the trajectory instead of decoding `trajectory_start` cannot pass. See `src/planning.rs`'s `TryFrom<PlanningResponseMsg>` doc comment for the full note. |
| `group_name: string` | **no field** | dropped — leave as is (round 5 brief: already known, not re-litigated) |
| `trajectory: moveit_msgs/RobotTrajectory` | `trajectory: RobotTrajectory<'m>` | via §10's `RobotTrajectoryMsg`/`RobotTrajectoryMsgOut` (rejects non-empty `multi_dof_joint_trajectory`) |
| `planning_time: float64` | **no field** | **Concluded round 6; the premise expired in D8.** p1-fixtures grounded this (`crates/moveit-planning/src/response.rs`): every upstream fill site sits inside a `PlanningContext`-equivalent's own `solve()` (`ompl_interface`/`chomp`/`stomp`/`pilz`, all cited by file:line there), never `PlanningPipeline::generatePlan` itself. Round 6 added "and no crate in this workspace implements a concrete planner" — D8 ended that half: `moveit_planners_sbp::registry::RrtConnectManager` implements `moveit_planning::PlannerManager` and its context implements `moveit_planning::PlanningContext`, so the fill site exists and is unfilled. Still dropped, not rejected, and still not this crate's call: it is `RrtConnectContext::solve`'s, and PORTING-PLAN.md §138.3 removed wall-clock timing from every oracle response, so nothing in this workspace could check a value it produced. |
| `error_code: MoveItErrorCodes` | **no field** (`PlanningResponse`'s own doc comment: `error_code` is this crate's `Result` return instead — a `PlanningResponse` value only ever exists once a solve already succeeded) | msg→core: dropped (a `PlanningResponse` cannot represent a failure `error_code` at all — a message carrying a failure code has to be handled by the caller *before* attempting this `TryFrom`, not by it); core→msg: synthesizes `SUCCESS` (`val=1`), since a `PlanningResponse` value existing at all already implies success — structural (the whole type only exists post-success), not an absence pending future work, so no expiry condition applies |
| **no field** | `planner_id: String` | **[R3, genuine gap the other direction]** `moveit-planning`'s own doc comment on `PlanningResponse::planner_id` claims it matches "an unset `moveit_msgs::msg::MotionPlanResponse::planner_id`" — checked against both `third_party/moveit_msgs/msg/MotionPlanResponse.msg` and the r2r-generated struct (fields: `trajectory_start`/`group_name`/`trajectory`/`planning_time`/`error_code` only): **`MotionPlanResponse` has no `planner_id` field at all.** This is a core-only field with nowhere on *this* message to go — msg→core always produces `""`; core→msg always drops it, regardless of what the `PlanningResponse` value carries. Named here as a documentation correction for `moveit-planning`'s owner, not worked around by inventing a wire field. |

## 17. Schema-drift risk classification (round 10, PORTING-PLAN.md §183's follow-up)

Judgment only, no gate: for every `moveit_msgs` field §1-16 above already
documents as touched (read or written, not dropped/rejected-only), which
ones are the kind that can silently change *meaning* under a
`third_party/moveit_msgs` repin without breaking compilation. §183's own
defect (an empty `frame_id` meaning "world") is the concrete instance of
one of these categories that has already materialized once.

**Anchor:** every field access enumerated by re-reading `src/model.rs`,
`src/state.rs`, `src/planning.rs`, `src/constraints/{joint,position,
orientation,visibility,set}.rs`, `src/scene/{collision_object,attached,
planning_scene}.rs` against the r2r-generated struct definitions
(`target/debug/build/r2r-*/out/moveit_msgs.rs`, `octomap_msgs.rs`) —
not `rg` alone, since a struct-destructure pattern (`let X { a, b, .. } =
msg`) doesn't textually contain every field name it drops.

Four categories, in the order §183 asked for:

| Category | What can drift without a compile break |
|---|---|
| **unit-bearing scalar** | Same `f64`, different physical meaning (radians↔degrees, seconds↔nanoseconds, a scale factor's reference point) |
| **frame-relative pose** | Interpretation depends on a `header.frame_id`/`Header` this port may or may not resolve the same way upstream does — §183's own category |
| **enum integer value** | A `u8`/`i32` whose meaning is a wire-declared named constant, not the type system — silently wrong if the port hardcodes the number instead of the generated constant and the constant is ever renumbered |
| **default-has-meaning** | A "zero"/"empty" value that is itself a valid, meaningful input (not "unset") — §183's exact defect shape |

### 17.1 Touched-field counts by type

Counting only fields this crate actually reads or writes per §1-16 (not
fields checked-and-rejected-if-non-default, e.g. `RobotState.is_diff`,
and not fields dropped outright, e.g. `AttachedCollisionObject.weight`):

| Type (§ above) | Total fields | Touched | Untouched (dropped/rejected-guard) |
|---|---|---|---|
| `JointLimits` (§3) | 10 | 10 | 0 |
| `JointConstraint` (§4) | 5 | 5 | 0 |
| `PositionConstraint` (§5) | 5 | 5 | 0 (`constraint_region: BoundingVolume`'s 4 nested fields also touched) |
| `OrientationConstraint` (§6) | 8 | 8 | 0 |
| `VisibilityConstraint` (§7) | 8 | 8 | 0 |
| `Constraints` (§8) | 5 | 4 | 1 (`name`, never read, always `""` on write) |
| `RobotState` (§9) | 4 | 1 fully (`joint_state`) | 3 checked-and-rejected-if-non-default, not converted (`is_diff`, `attached_collision_objects`, `multi_dof_joint_state`) |
| `RobotTrajectory` (§10) | 2 | 1 (`joint_trajectory`) | 1 checked-and-rejected (`multi_dof_joint_trajectory`) |
| `CollisionObject` (§11) | 13 | 12 | 1 (`type_: ObjectType`, D1-dropped) |
| `AttachedCollisionObject` (§11) | 5 | 3 (`link_name`, `object`, `touch_links`) | 2 dropped (`weight`, `detach_posture`) |
| `PlanningSceneWorld` (§11) | 2 | 2 | 0 |
| `PlanningScene` top-level (§11) | 10 | 2 (`is_diff` via `is_diff()`, `robot_model_name` via `robot_model_name_matches()`) | 8 (the rest is the module doc's own named scope gap: `robot_state`, `fixed_frame_transforms`, `allowed_collision_matrix`, `link_padding`/`link_scale`, `object_colors`, `name`, `world` is separately counted above) |
| `MotionPlanRequest` (§16) | 16 | 8 | 8 (6 planner-tuning fields dropped, `start_state`/`reference_trajectories` rejected-if-non-default) |
| `WorkspaceParameters` (nested in above) | 3 | 2 (`min_corner`/`max_corner`) | 1 (`header`, confirmed genuinely inert this round — see 17.3) |
| `MotionPlanResponse` (§16) | 5 | 3 (`trajectory_start`, `trajectory`, `error_code` write-only) | 2 dropped (`group_name`, `planning_time`) |
| `MoveItErrorCodes` (§2) | 3 | 1 (`val`) | 2 dropped (`message`, `source`) |
| `Octomap`/`OctomapWithPose` (octomap_msgs, §183's own scope) | 5 + 3 | all 8 | 0 |

**Totals: 109 fields across 17 types, 75 touched, 34 dropped or
guard-checked-only.**

### 17.2 Touched fields by risk category

- **frame-relative pose** (12 sites): `CollisionObject.header`(×2 call
  sites, `collision_object.rs:358,457`, `:358` reads `subframe_poses,`), `AttachedCollisionObject.object.
  header`(`attached.rs:219-222` (`shapes_from_message_geometry`)), `OctomapWithPose.header`(`planning_scene.rs`,
  §183, already fixed), `PositionConstraint.header`/`OrientationConstraint.
  header`(captured as a string and handed to `crates/moveit-constraints`
  for lazy per-`decide()` resolution — that crate's own frame-resolution
  code is the boundary to check next, not re-verified against upstream
  this round), `VisibilityConstraint.sensor_pose.header`/`.target_pose.
  header`(same deferred-to-`moveit-constraints` shape). Of these, only the
  four resolved directly by `scene::header_frame_transform` are verified
  against upstream's own guard-vs-no-guard split (§183); the constraint
  family's three sites are UNVERIFIED this round — flagged, not fixed.

- **default-has-meaning** (2 confirmed, 1 already fixed): the `frame_id`
  empty-string case (§183, fixed `aa45d37`); `CollisionObject.operation`
  bare `u8` has no "unset" value at all (always one of the four), so it
  is not in this category despite being adjacent.

- **enum integer value** (2 findings; both fixed round 12, `993506d`/
  `6c2d884`, PORTING-PLAN.md §191.2):
  - `planning.rs`: `error_code: moveit_msgs::MoveItErrorCodes { val: 1, ..
    }` hardcoded the literal `1` for `SUCCESS` instead of referencing the
    r2r-generated `moveit_msgs::msg::MoveItErrorCodes::SUCCESS` constant
    (confirmed generated: `target/.../moveit_msgs.rs:6473`, one of 29
    named constants per §2's own audit). Fixed to `val:
    moveit_msgs::MoveItErrorCodes::SUCCESS as i32`.
  - `collision_object.rs:46-49` (own doc comment reads `bindgen-derived type (`): local `const ADD: u8 = 0`/`REMOVE = 1`/
    `APPEND = 2`/`MOVE = 3` duplicated the r2r-generated `moveit_msgs::msg::
    CollisionObject::{ADD,REMOVE,APPEND,MOVE}` constants (confirmed
    generated: `target/.../moveit_msgs.rs:4935-4938`) instead of
    referencing them. Fixed to cast from the generated constants. **What
    the fix actually buys, corrected after this doc first shipped:** a
    repin that renumbers one of these is followed automatically (`as u8`
    casts whatever the new discriminant is) — it does *not* turn a repin
    into a compile error, since `as` on a fieldless enum accepts any
    value the variant carries. Only deleting or renaming the constant
    itself would fail to compile. The earlier draft of this row (and the
    matching source comment) claimed the compile-error benefit instead of
    the auto-follows-repin benefit; both are corrected now.
  - `VisibilityConstraint.sensor_view_direction` (§175's own claim-audit
    row, `constraints/visibility.rs:36-37`, own doc comment reads `moveit_constraints::visibility`) and `OrientationConstraint.
    parameterization` (deferred to `crates/moveit-constraints/src/
    orientation.rs:74`, which cites its own upstream line) are the same
    category but **already independently verified against upstream this
    session/crate** — not new findings, listed here only for completeness
    of the count.

- **unit-bearing scalar** (≈20 sites: every `min_position`/`max_velocity`/
  `max_acceleration`/`max_jerk`/`tolerance_above`/`tolerance_below`/
  `absolute_*_axis_tolerance`/`target_radius`/`max_view_angle`/
  `max_range_angle`/`weight`/`max_velocity_scaling_factor`/
  `max_acceleration_scaling_factor`). No evidence of current drift found
  (every one of these is a 1:1 same-name-same-type field per §1-16's
  existing per-field audit) — flagged as the largest category by count,
  not because any specific one is suspected wrong.

### 17.3 One row re-checked, not reopened

§16's existing `header dropped (metadata, same treatment as §1)` row for
`WorkspaceParameters.header` was re-verified against upstream this round
(not previously checked at the call-site level, only asserted by analogy
to §1): `ompl_interface/src/model_based_planning_context.cpp:433-449`
(`setPlanningVolume`) uses only `min_corner`/`max_corner`, never
`wparams.header` — confirmed genuinely inert upstream too, not a live
drift risk. Included here to show the check was run, not skipped, given
this doc's own frame-relative-pose category made it the most likely
candidate for a second §183-shaped surprise.

### 17.4 What a real gate would need (estimate only, not built)

To catch a future repin's silent field-meaning change (not just a
compile break), a gate would need, per touched field: (1) the exact
upstream `.msg` comment/semantics at the pinned commit, snapshotted; (2)
a diff against the same field's comment/semantics at the new pin; (3) a
human or an LLM judgment call on whether the diff is cosmetic or
meaning-changing, since `.msg` comments are prose, not a machine-checkable
contract. Building steps (1)-(2) is mechanical (a script diffing
`third_party/moveit_msgs/msg/*.msg` comments across the two pinned
commits, ~75 touched fields to diff); step (3) is not mechanizable — it
is exactly the by-hand judgment this section just did once. Estimate: a
day to build the mechanical diff-and-flag script, and this section's
own depth of re-review (roughly a day, per this round) every time the
pin actually moves and the script flags something. Not proposed as a
merge gate — repins are rare and reviewable by hand at the time; a
standing script would mostly diff nothing.

### 17.5 The default-has-meaning family, re-swept by field property (round 12, PORTING-PLAN.md §191)

§191 found that anchoring §183's fix search on `frame_transform\(` cut the
defect family to that function's own shape, and named
**default-has-meaning** (17.2) as the property-level anchor that should
have been used instead. This section re-runs the sweep against that
anchor: every field in this crate's own code (not `crates/moveit-*`,
whose validation is a separate crate's call) where a wire default
(empty string, `0`, an empty array, or the identity `Pose`/`Quaternion`)
reaches a point that could reject it, checked against the matching
upstream function line-by-line.

**Sites** (file:line, upstream line, verdict):

- `scene/mod.rs:43` (`header_frame_transform`, reads `frame_id.is_empty()`) — the §183 site itself,
  already fixed (`aa45d37`).
- `scene/collision_object.rs:339` (`apply_add`, reads `plane_poses,`, no shapes) —
  `planning_scene.cpp:1894`: upstream also errors on empty
  primitives/meshes/planes. Matches.
- `scene/collision_object.rs:420` (`apply_remove`, reads `REMOVE. Upstream`, empty `id`) —
  `:1933`: upstream's `object.id.empty()` also means "remove
  everything". Matches (and already documented + tested,
  `remove_empty_id_removes_everything`).
- `scene/collision_object.rs:483` (`apply_move`, reads `.pose();`, empty
  `shape_poses_msgs`) — `:1973`: upstream's
  `!primitive_poses.empty() || ...` guard is the same "no pose data ->
  skip the repose step, still Ok" shape. Matches.
- `scene/attached.rs:95,100` (`apply_attach`, `:95` reads
  `object.primitives.is_empty()`, no-geometry world
  promotion) — `:1563` (ADD/APPEND gate) + `:1579` (the no-geometry
  check itself, further gated to ADD-only): exactly `is_add &&
  no_geometry`. Matches.
- `scene/attached.rs:142` (`apply_attach`, `shapes.is_empty()` after
  conversion) — `:1620`: upstream's own
  `if (shapes.empty()) return false`. Matches (already tested,
  `add_with_no_geometry_and_no_world_object_is_rejected` covers the
  no-world-object variant of the same check).
- `scene/attached.rs:285,288,298` (`apply_detach`, empty `id`/
  `link_name`) — `:1665`: upstream's `!attached_bodies.empty() ||
  object.object.id.empty()` return expression, already reproduced and
  documented in this module's own doc comment. Matches.
- `scene/planning_scene.rs:90` (drift: row previously cited `:59`, which
  is now well before `robot_model_name_matches`, a stale line number)
  (`robot_model_name_matches`, reads
  `robot_model_name.is_empty()`, empty
  name) — `setPlanningSceneMsg:1370`: upstream skips the compatibility
  check entirely on an empty name. Matches.
- `scene/planning_scene.rs:516` (drift: row previously cited `:127`,
  stale after `p11-scenetopic` grew this file 522 -> 1367 lines)
  (`apply_octomap`, reads
  `map.octomap.data.is_empty()`, empty
  `octomap.data`) — `:1483`: upstream's own early return once the
  previous octomap is cleared. Matches.
- `ros/moveit-ros/src/state.rs:68,75,84-87` (`:68` reads `names: &[String],`; `is_diff`/`attached_collision_objects`/
  `multi_dof_joint_state` non-default) — **not this shape**: these
  reject a *non-default* value because there is no core field to carry
  it (a structural gap, `RobotState`'s own doc comment), the opposite
  polarity from §183 (which rejected the *default*). Matches D6's
  intended behavior, not a defect. Each of the three now carries its
  own expiry condition (PORTING-PLAN.md §153.1, `state.rs`'s module
  doc): `multi_dof_joint_state` clears if `moveit_state::RobotState`
  gains multi-DOF support; `attached_collision_objects`/`is_diff`
  clear only if this crate adds a `&mut PlanningScene`-aware
  conversion entry point, not if `moveit-state` changes.
- `ros/moveit-ros/src/trajectory.rs:142` (own comment reads `// == 0.0`, for `JointTrajectoryPoint[0].time_from_start`
  nonzero) — same opposite-polarity shape as `state.rs` above:
  rejects a non-default value `RobotTrajectory` cannot represent, not
  a rejected default. Expiry noted inline (§153.1): only
  `moveit_trajectory::RobotTrajectory`'s own `duration_from_previous[0]
  == 0.0` invariant changing clears this, not a new field anywhere.
- `planning.rs:150-154,162-166,278-281` (re-derived: row previously cited
  `:130,138`, which named a single blanket `start_state` rejection that
  `p11-startstate` (`10f571f`) deleted outright — `start_state` is now a
  `StartState` sum type, so the old citation points at code that no longer
  exists. It split into two per-field rejections, `attached_collision_objects`
  (`:150-154`) and `multi_dof_joint_state` (`:162-166`) on `StartState`,
  plus the unchanged `reference_trajectories` rejection (`:278-281`) on
  `PlanningRequest`; `:150` reads `attached_collision_objects.is_empty()`)
  — same opposite-polarity shape again, already named D6-consistent in
  this module's own doc comment. Expiry noted inline (§153.1): all three
  clear if `moveit_planning::StartState`/`PlanningRequest` gain the
  matching field, unlike `state.rs`'s gap above which needs a new
  conversion entry point instead.

Every site above already has a boundary test proving the behavior
checked here (see the test names cited inline) — this sweep did not
find a test gap either.

**One cross-crate observation, resolved since (round 13 update):**
this paragraph originally cited a line in `moveit-constraints`'
`position.rs` and a line in its `joint.rs` (both since renumbered by
later edits) as rejecting `weight <= EPS` with
`Err` where upstream substitutes `1.0`, and (wrongly) filed it as a
deliberate, out-of-scope policy decision rather than the same
default-has-meaning defect this section sweeps for — the misreading
the coordinator caught (PORTING-PLAN.md D14/§199). All four
constructors (`joint.rs`, `position.rs`, `orientation.rs`,
`visibility.rs`) now normalize `weight <= EPS` to `1.0`, matching
`kinematic_constraint.cpp:263/450/641/871` (`551b719`). This crate's
own four `TryFrom` impls were confirmed pure pass-throughs at fix
time (no separate wire-side check to also update), and its own
regression coverage lives in `src/constraints/set.rs`'s four
`unspecified_*_weight_is_normalized_to_one_not_rejected` tests —
originally written asserting the *old* `Err` behavior as a tripwire
(PORTING-PLAN.md §205) before `551b719` landed, confirmed to go red
on that merge, then flipped to assert `weight() == 1.0` (`932b7bf`).

**Conclusion:** no new same-defect site found in `ros/moveit-ros`
itself. §183 was the family's only site in this crate; the anchor
correction (§191) widened the *search*, not the *result*.

### 17.6 Which of §17.5's four expiry conditions are self-revealing (round 13)

§153.1 requires naming what clears an absence-reasoned rejection, but a
condition documented in a comment still relies on someone reading that
comment at the right moment. The four expiry conditions `d4ca334` added
are not equally reliant on that: one of them is caught by the compiler
itself the moment its trigger fires, the other three are not. Checked
empirically, not just by inspection, since a plausible-looking claim
about compile errors was already wrong once this round (§17.2's
corrected renumbering claim).

- **`planning.rs:130-135` (reads `type Error = Error;`; `start_state`/`reference_trajectories`) —
  mechanically self-revealing.** The `Ok(PlanningRequest { ... })`
  construction at `planning.rs:180-188` (`try_from`) is an exhaustive struct
  literal (no `..Default::default()`) even though `PlanningRequest`
  derives `Default`. Verified by temporarily adding an undocumented
  field to `crates/moveit-planning::PlanningRequest` and rebuilding
  `ros/moveit-ros` in the docker toolchain: the build failed with
  `error[E0063]: missing field `__temp_compile_check_field` in
  initializer of `PlanningRequest`` pointing at exactly this call
  site (the temporary field, and the edit to
  `crates/moveit-planning/src/request.rs`, were reverted immediately
  after; neither was committed). A new `PlanningRequest` field forces
  a person to this exact line without anyone needing to remember the
  comment exists.
- **`ros/moveit-ros/src/state.rs:68,75,84-87` (`:68` reads `names: &[String],`; `is_diff`/`attached_collision_objects`/
  `multi_dof_joint_state`) — requires human memory.** Neither
  `TryFrom<RobotStateMsg>` nor `TryFrom<CoreRobotState>` builds
  `CoreRobotState` through a struct literal (`CoreRobotState::new`,
  an opaque constructor, is the only construction site) and
  `multi_dof_joint_state` support arriving on `moveit_state::RobotState`
  changes no signature this file calls. The `attached_collision_objects`/
  `is_diff` gap is even further from compiler-visible: its expiry is
  authoring a conversion entry point that does not exist yet, so there
  is nothing for a future edit to newly fail against.
- **`ros/moveit-ros/src/trajectory.rs:142` (own comment reads `// == 0.0`; nonzero `time_from_start[0]`) — not
  compiler-enforced, but now runtime-tripwired (round 13 follow-up,
  after D14 proved the tripwire pattern viable and the coordinator
  asked this classification be pushed on again).** The rejection
  guards `RobotTrajectory::add_suffix_way_point`'s own runtime
  invariant (`duration_from_previous.is_empty() && dt != 0.0` — read
  directly in `crates/moveit-trajectory/src/robot_trajectory.rs:261-263` (`add_suffix_way_point`)),
  not a type or field `TryFrom<JointTrajectoryMsg>` constructs
  exhaustively, so no E0063-style compile break is possible here. But
  unlike `state.rs`'s two conditions below, `add_suffix_way_point`
  *already exists and already enforces the exact invariant today* —
  there is a live call path to assert against, which is what a
  tripwire needs. Added
  `trajectory::tests::add_suffix_way_point_rejects_a_nonzero_first_dt`,
  calling `add_suffix_way_point` directly (bypassing this crate's own
  `TryFrom` and its own duplicate guard) and asserting the current
  `Err`; it goes red the moment that invariant relaxes. One caveat
  documented at `ros/moveit-ros/src/trajectory.rs:148` (reads `// field cannot clear this`)'s own expiry comment: this crate's
  `i == 0 && t != 0.0` check fires *before* `add_suffix_way_point` is
  ever called (it exists only to give a wire-specific message), so
  the wire-level `nonzero_start_time_is_rejected` test would **not**
  go red alongside the tripwire — a person still has to read the
  tripwire's failure and separately update or remove that duplicate
  check. Partially self-revealing, not fully: the underlying fact is
  now mechanically caught, the crate-local duplicate is not.
- **`ros/moveit-ros/src/state.rs:68,75,84-87` (`:68` reads `names: &[String],`) — checked whether the same runtime-tripwire
  approach applies; it does not.** A tripwire needs an *existing* call
  path whose current answer changes; both `multi_dof_joint_state` and
  `attached_collision_objects`/`is_diff` name the *arrival* of a
  capability with no call path yet to assert anything about.
  `multi_dof`/`MultiDof` has zero hits anywhere in
  `crates/moveit-model/src` (checked directly, not inferred) — there
  is no symbol, field, or function whose current behavior a test could
  pin. `attached_collision_objects`/`is_diff`'s expiry is this crate
  *authoring* a new `&mut PlanningScene`-aware conversion function —
  before that function exists, there is nothing to call and watch
  fail. A tripwire cannot test for the absence of an API; it can only
  watch an existing one change answers. Genuinely requires human
  memory, confirmed by the same test this round's `trajectory.rs` case
  passed and failed on: is there a call path today.

Net: one site is compiler-enforced (`planning.rs`), one is now
runtime-tripwired though not fully (`trajectory.rs`, still needs a
human to retire its own duplicate wire-side check once the tripwire
fires), and two remain pure §153.1 documented-expiry cases with no
mechanical backstop at all (`state.rs`). The `planning.rs` comment and
`trajectory.rs`'s tripwire both stay alongside their mechanical
backstops since they explain *why*, which a compile error or a failing
assertion alone would not.

### 17.7 Tripwire inventory (round 13, PORTING-PLAN.md §205)

Every test in this crate currently asserting the *current, wrong*
behavior of something outside this crate's own control, kept live so
it goes red automatically when that changes (as opposed to `#[ignore]`,
which this session has closed twice for staying silently green either
way — §184, §197.3). Re-check this list whenever one fires or a new
one is added; a tripwire with no "fires on / do this" instruction
sitting next to it is only half-built, since the person who sees it go
red still needs to know what to do.

**Live (one):**

- `trajectory::tests::add_suffix_way_point_rejects_a_nonzero_first_dt`
  (`trajectory.rs`, added round 13). **Fires on:**
  `moveit_trajectory::RobotTrajectory::add_suffix_way_point`'s
  `duration_from_previous[0] == 0.0` invariant relaxing to accept a
  nonzero first `dt`. **Do this:** the test's own doc comment says to
  go update or remove `trajectory.rs`'s own `i == 0 && t != 0.0` check
  (the duplicate, wire-specific-message guard one function up), since
  it no longer describes a real core limitation once this fails.

**Retired this round (four, fired and flipped already — no longer
tripwires):** `constraints::set::tests::unspecified_{joint,position,
orientation,visibility}_weight_is_normalized_to_one_not_rejected`.
Written in round 13 asserting the *old* `Err` behavior; D14 (`551b719`)
landed in the same merge round, all four went red on the gate, and the
coordinator flipped them to assert `weight() == 1.0` (`932b7bf`). They
are now ordinary regression tests, not tripwires — nothing further to
watch here.

**Not a tripwire, noted so it is not mistaken for one:**
`conversion_coverage.rs`'s `ONE_DIRECTIONAL`/`TRANSITIVELY_COVERED`
exemption lists (module doc, this file) re-verify their own premise on
every test run, but they watch an *internal* structural invariant of
this crate's own conversions, not an external crate's behavior this
crate has no control over — a different, already-self-checking
mechanism, not the "waiting on someone else's change" shape this
section inventories.

## 18. Per-site quaternion-norm boundary table (round 16, PORTING-PLAN.md §215)

Round 14's `4ff563d` made the generic `Isometry3::try_from(Pose(...))`
rule reject `norm == 2.0`, reasoning from `OrientationConstraint`'s own
1e-3 threshold — a threshold that belongs to only *one* of this rule's
ten reaching call sites. `f2a7847` split this back into two named
rules (`OrientationConstraintQuaternion`, strict; the generic rule,
permissive) so each site reaches the rule its own upstream function
actually implements. This section states, per site, what a caller
observes at the five boundary values the brief asked for, and whether
that claim was run at *this exact site* or reasoned from code-path
identity with a site that was.

**Method.** All ten sites resolve to exactly one of two functions:

- **Generic** (`geometry.rs`'s `TryFrom<Quaternion> for UnitQuaternion`
  / `TryFrom<Pose> for Isometry3`): unconditionally renormalizes any
  finite, nonzero-norm quaternion; rejects only exact-zero-norm and
  non-finite (`NaN`/`inf`) components. No 1e-3 threshold anywhere.
- **Strict** (`OrientationConstraintQuaternion`'s own
  `TryFrom`): additionally rejects `|norm - 1.0| > 1e-3`, matching
  `OrientationConstraint::configure`'s own suspicion threshold
  (`kinematic_constraint.cpp`, cited in `f2a7847`).

Nine of the ten sites call the generic rule with **zero intervening
logic** between the wire field and the call (confirmed by reading each
call site directly — `rg` anchor:
`Isometry3::try_from\(Pose\(|OrientationConstraintQuaternion`,
enumerated in §213.1/§213.2). Because the code path is byte-identical,
`geometry.rs`'s own boundary tests on the generic rule are valid
evidence for what those nine sites do — but "valid evidence via
code-path identity" is not the same claim as "run at this site", so
the two are marked distinctly below. `norm == 2.0` is the one value
this round runs as a real, this-site integration test at all nine
(the value `4ff563d` actually broke), rather than resting on the
code-identity argument alone.

**Legend.** ✅*site* = a test exercises this exact call site end to
end. ✅*generic*/✅*strict* = not run at this site; reasoned from the
named function's own `geometry.rs` unit test, valid only because the
call site's code is the unconditional, unbranched call named above.

| # | Site (file:line) | Rule | norm=2.0 | norm=1.0011 | norm=1.0009 | all-zero | NaN |
|---|---|---|---|---|---|---|---|
| 1 | `geometry.rs` (the rule itself) | generic | ✅site `norm_far_from_one_is_renormalized_not_rejected` → Ok, renormalized | ✅site `norm_just_outside_orientation_rules_1e_minus_3_tolerance_is_still_accepted_here` → Ok | ✅site `norm_just_inside_orientation_rules_1e_minus_3_tolerance_is_also_accepted_here` → Ok | ✅site `zero_quaternion_is_rejected` → `Error::Construct` | ✅site `nan_quaternion_is_rejected` → `Error::Construct` |
| 2 | `constraints/orientation.rs:92-93` (reads `UnitQuaternion::try_from(OrientationConstraintQuaternion(msg.orientation))?;`, `OrientationConstraint.orientation`) | strict | ✅site `orientation_norm_2_is_rejected_end_to_end_unlike_a_scene_pose` → `Error::Construct` | ✅generic-fn `orientation_norm_just_outside_the_1e_minus_3_tolerance_is_rejected` → `Error::Construct` (not run through `orientation.rs`'s own conversion end to end) | ✅generic-fn `orientation_norm_just_inside_the_1e_minus_3_tolerance_is_accepted` → Ok (not run end to end) | ✅generic-fn `orientation_zero_quaternion_is_rejected` → `Error::Construct` (not run end to end) | ✅generic-fn `orientation_nan_quaternion_is_rejected` → `Error::Construct` (not run end to end) |
| 3 | `constraints/position.rs:161` (reads `Isometry3::try_from(Pose(pose))?;`, `BoundingVolume.primitive_poses`/`mesh_poses`) | generic | ✅site `region_pose_with_norm_2_orientation_succeeds_and_normalizes` → Ok, renormalized | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → `Error::Construct` | ✅generic-fn (row 1) → `Error::Construct` |
| 4 | `constraints/visibility.rs:114` (`sensor_pose`) | generic | ✅site `sensor_and_target_pose_with_norm_2_orientation_succeed_and_normalize` → Ok, renormalized | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → `Error::Construct` | ✅generic-fn (row 1) → `Error::Construct` |
| 5 | `constraints/visibility.rs:115` (`target_pose`) | generic | ✅site (same test as row 4) → Ok, renormalized | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → `Error::Construct` | ✅generic-fn (row 1) → `Error::Construct` |
| 6 | `scene/collision_object.rs:142` (reads `Isometry3::try_from(Pose(p))?,`, per-shape pose, ADD/APPEND) | generic | ✅site `add_with_norm_2_orientation_on_object_and_shape_poses_succeeds_and_normalizes` → Ok, renormalized | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → `Error::Construct` | ✅generic-fn (row 1) → `Error::Construct` |
| 7 | `scene/collision_object.rs:207` (reads `Isometry3::try_from(Pose(object_pose_msg))?`, object pose, ADD/APPEND, non-promoted) | generic | ✅site (same test as row 6) → Ok, renormalized | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → `Error::Construct` | ✅generic-fn (row 1) → `Error::Construct` |
| 8 | `scene/collision_object.rs:239` (reads `Isometry3::try_from(Pose(pose))?);`, `subframe_poses`) | generic | ✅site `add_with_norm_2_orientation_on_subframe_pose_succeeds_and_normalizes` → Ok, renormalized | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → `Error::Construct` | ✅generic-fn (row 1) → `Error::Construct` |
| 9 | `scene/collision_object.rs:478` (reads `Isometry3::try_from(Pose(pose))?;`, object pose, MOVE) | generic | ✅site `move_with_norm_2_orientation_on_object_and_shape_poses_succeeds_and_normalizes` → Ok, renormalized | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → `Error::Construct` | ✅generic-fn (row 1) → `Error::Construct` |
| 10 | `scene/collision_object.rs:515` (reads `Isometry3::try_from(Pose(p)))`, per-shape pose, MOVE repose) | generic | ✅site (same test as row 9) → Ok, renormalized | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → `Error::Construct` | ✅generic-fn (row 1) → `Error::Construct` |
| — | `scene/planning_scene.rs:536` (drift: row previously cited `:147`, stale after `p11-scenetopic` grew this file 522 -> 1367 lines) (reads `Isometry3::try_from(Pose(map.origin))?;`, octomap `origin`) | generic | ✅site `octomap_origin_with_norm_2_orientation_succeeds_and_normalizes` → Ok, renormalized | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → Ok | ✅generic-fn (row 1) → `Error::Construct` | ✅generic-fn (row 1) → `Error::Construct` |

(This is 11 rows for "ten sites" — §213.2's count of nine generic-rule
sites did not separately list the octomap `origin` at
`planning_scene.rs:536` (reads `Isometry3::try_from(Pose(map.origin))?;`);
it belongs to the same generic rule and is
included here for completeness. The brief's "ten sites" is the
Quaternion/Pose-reaching total from §211/`f2a7847`: nine generic +
one strict = ten *distinct wire fields*, and `visibility.rs`'s
`sensor_pose`/`target_pose` are two fields sharing one test, same as
`collision_object.rs`'s object/shape-pose pairs sharing one test per
operation.)

**Explicitly unverified: none.** Every row's `norm=2.0` column is a
this-site integration test added this round. Every row's other four
columns rely on the code-path-identity argument stated above, which is
sound *only* because each call site was individually confirmed
(§213.1's `rg` sweep, re-checked while writing this table) to be a
bare, unbranched `Isometry3::try_from(Pose(...))` or
`UnitQuaternion::try_from(OrientationConstraintQuaternion(...))` call
— if any site ever gains a branch between the wire field and that
call (e.g. a per-site default-substitution or clamp), this table's
reasoned cells for that row stop being valid and must be re-run as
real site-level tests, not just re-asserted.
