// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Wire protocol shared with the C++ oracle (`tools/moveit-oracle`).
//!
//! One JSON object per line in each direction. The oracle is launched once
//! with a URDF/SRDF pair and then answers requests until stdin closes, so the
//! cost of parsing the robot description is paid once per run rather than once
//! per case.
//!
//! Keep this file and `tools/moveit-oracle/src/oracle.cpp` in step — the C++
//! side hand-rolls the same shapes.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

/// A request sent to the oracle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Request {
    /// Correlates the response. Monotonic within a run.
    pub id: u64,
    /// What to compute.
    #[serde(flatten)]
    pub op: Op,
}

/// The operations the oracle understands.
///
/// Phase 1 needs [`Op::ModelInfo`]; Phase 2 needs [`Op::Fk`] and
/// [`Op::Jacobian`]. Later phases extend this enum; the oracle answers
/// `ok: false` for an op it does not implement, which is what lets a newer
/// runner talk to an older oracle binary without a version handshake.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum Op {
    /// Structural facts about the loaded `RobotModel`.
    ModelInfo,
    /// Forward kinematics for the named links at the given joint values.
    Fk {
        /// Joint name to position. Joints omitted keep their default value.
        joint_values: BTreeMap<String, f64>,
        /// Links to report. Empty means every link in the model.
        #[serde(default)]
        links: Vec<String>,
    },
    /// Draw `count` whole-model random states from the oracle's own sampler.
    ///
    /// The oracle owns randomness so that floating-joint quaternions come out
    /// normalized, bounds are respected per joint type and mimic values are
    /// derived — none of which a variable-by-variable sampler here would get
    /// right. A run is reproducible from `seed`.
    RandomStates {
        /// How many states to draw.
        count: usize,
        /// Seed for `random_numbers::RandomNumberGenerator`.
        seed: i32,
    },
    /// Geometric Jacobian of `group` at the given joint values.
    Jacobian {
        /// Joint model group name.
        group: String,
        /// Joint name to position.
        joint_values: BTreeMap<String, f64>,
    },
    /// The `AllowedCollisionMatrix` built from the loaded SRDF's
    /// `disable_collisions`/`enable_collisions`/`disable_default_collisions`,
    /// dumped entry by entry.
    Acm,
    /// Ground truth for the `moveit-collision` `World` port. Builds a
    /// `collision_detection::World` directly from `objects` — no `RobotModel`
    /// involved, since `World` has none — and dumps every object's pose,
    /// per-shape pose/global pose and per-subframe pose/global pose, plus
    /// `knowsTransform`/`getTransform` answers for each name in `queries`.
    World {
        /// Objects to build the world from, in request order (the response's
        /// own object order is the world's, i.e. sorted by id).
        objects: Vec<WorldObjectSpec>,
        /// Names to resolve with `knowsTransform`/`getTransform` after every
        /// object above has been added.
        #[serde(default)]
        queries: Vec<String>,
    },
    /// Ground truth for the `moveit-distance-field` `PropagationDistanceField`
    /// port. Builds a field directly from `geometry`/`max_distance`/
    /// `propagate_negative` -- no `RobotModel` involved, `distance_field` has
    /// none either -- adds `occupied_cells` (explicit integer grid
    /// coordinates, not shapes: `moveit-geometry`'s `bodies` port is not
    /// merged yet), then dumps `getDistance`, `getDistanceGradient` and
    /// `getNearestCell` for every cell in `queries`.
    DistanceField {
        /// Size, origin and resolution of the grid.
        geometry: DistanceFieldGeometry,
        /// `PropagationDistanceField`'s `max_distance` constructor argument.
        max_distance: f64,
        /// Whether the field also propagates distances inward from
        /// unoccupied cells.
        propagate_negative: bool,
        /// Cells to seed as obstacles before any query, as integer grid
        /// coordinates.
        occupied_cells: Vec<[i32; 3]>,
        /// Cells to query, as integer grid coordinates. Coordinates outside
        /// `[0, num_cells)` on any axis are valid input -- the response's
        /// `in_grid` reports whether a given query landed inside the grid.
        queries: Vec<[i32; 3]>,
    },
    /// Ground truth for `find_internal_points_convex`
    /// (`distance_field::findInternalPointsConvex`) -- the shape-to-points
    /// step [`Op::DistanceField`] does not exercise, since that op takes
    /// `occupied_cells` as an explicit input and starts only after this
    /// step. Builds the `bodies::Body` for `shape` exactly as upstream
    /// `DistanceField::getShapePoints` does: `createEmptyBodyFromShapeType`,
    /// then `setDimensionsDirty`, `setPoseDirty` and `updateInternalData`,
    /// which never touch scale or padding (so both stay at 1.0/0.0), poses
    /// it at `pose`, and returns every point `findInternalPointsConvex`
    /// finds on the `resolution`-spaced grid.
    ShapePoints {
        /// The shape to sample.
        shape: ShapeSpec,
        /// The shape's pose, row-major 4x4.
        pose: [f64; 16],
        /// Grid spacing `findInternalPointsConvex` samples at.
        resolution: f64,
    },
    /// Ground truth for the `moveit-constraints` `KinematicConstraintSet`
    /// port. Builds a `moveit_msgs::msg::Constraints` from `constraints`
    /// (one message vector per constraint kind, joint/position/orientation/
    /// visibility, each filled in request order — the same order
    /// `KinematicConstraintSet::add(msg, tf)` walks internally, and the order
    /// [`ConstraintsResult::results`] reports back in), sets `joint_values` on
    /// top of the model defaults exactly as [`Op::Fk`] does, builds a
    /// `KinematicConstraintSet` against a `Transforms(model_->getModelFrame())`
    /// (identity-only — no TF listener, matching this port's own
    /// `Transforms::new(model.model_frame())`), and calls
    /// `decide(state, results)`.
    Constraints {
        /// Joint name to position. Joints omitted keep their default value.
        joint_values: BTreeMap<String, f64>,
        /// The constraints to build and evaluate.
        constraints: ConstraintsSpec,
    },
}

/// The constraints to build for one [`Op::Constraints`] request, grouped by
/// kind and in the exact order `KinematicConstraintSet::add` walks them
/// (joint, then position, then orientation, then visibility) — the same
/// order [`ConstraintsResult::results`] reports back in, which is what lets a
/// flat result index be correlated back to a specific input constraint.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ConstraintsSpec {
    /// `Constraints::joint_constraints`.
    #[serde(default)]
    pub joint_constraints: Vec<JointConstraintSpec>,
    /// `Constraints::position_constraints`.
    #[serde(default)]
    pub position_constraints: Vec<PositionConstraintSpec>,
    /// `Constraints::orientation_constraints`.
    #[serde(default)]
    pub orientation_constraints: Vec<OrientationConstraintSpec>,
    /// `Constraints::visibility_constraints`.
    #[serde(default)]
    pub visibility_constraints: Vec<VisibilityConstraintSpec>,
}

/// One `moveit_msgs::msg::JointConstraint`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointConstraintSpec {
    /// `joint_name`, following upstream's own `"joint"` /
    /// `"joint/local_variable"` convention.
    pub joint_name: String,
    /// `position`.
    pub position: f64,
    /// `tolerance_above`.
    pub tolerance_above: f64,
    /// `tolerance_below`.
    pub tolerance_below: f64,
    /// `weight`.
    pub weight: f64,
}

/// One region of a `moveit_msgs::msg::BoundingVolume.primitives` (with its
/// paired `primitive_poses` entry) — meshes are not needed by this
/// differential test, so [`ConstraintsSpec`] only ever builds the primitive
/// half of a `BoundingVolume`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConstraintRegionSpec {
    /// The region's shape.
    pub shape: ShapeSpec,
    /// The region's pose relative to `PositionConstraintSpec::frame_id`,
    /// row-major 4x4.
    pub pose: [f64; 16],
}

/// One `moveit_msgs::msg::PositionConstraint`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PositionConstraintSpec {
    /// `header.frame_id`.
    pub frame_id: String,
    /// `link_name`.
    pub link_name: String,
    /// `target_point_offset`.
    pub target_point_offset: [f64; 3],
    /// `constraint_region.primitives`/`primitive_poses`, paired.
    pub regions: Vec<ConstraintRegionSpec>,
    /// `weight`.
    pub weight: f64,
}

/// One `moveit_msgs::msg::OrientationConstraint`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OrientationConstraintSpec {
    /// `header.frame_id`.
    pub frame_id: String,
    /// `link_name`.
    pub link_name: String,
    /// `orientation`, as `[x, y, z, w]`.
    pub orientation: [f64; 4],
    /// `parameterization` plus the three `absolute_*_axis_tolerance` fields.
    pub tolerance: OrientationToleranceSpec,
    /// `weight`.
    pub weight: f64,
}

/// `parameterization` plus the three `absolute_*_axis_tolerance` fields of a
/// `moveit_msgs::msg::OrientationConstraint`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "parameterization", rename_all = "snake_case")]
pub enum OrientationToleranceSpec {
    /// `parameterization = XYZ_EULER_ANGLES` (`0`).
    XyzEuler {
        /// `absolute_x_axis_tolerance`.
        x: f64,
        /// `absolute_y_axis_tolerance`.
        y: f64,
        /// `absolute_z_axis_tolerance`.
        z: f64,
    },
    /// `parameterization = ROTATION_VECTOR` (`1`).
    RotationVector {
        /// `absolute_x_axis_tolerance`.
        x: f64,
        /// `absolute_y_axis_tolerance`.
        y: f64,
        /// `absolute_z_axis_tolerance`.
        z: f64,
    },
}

/// One `moveit_msgs::msg::VisibilityConstraint`. Only the criteria decidable
/// without a collision backend are exercised by the differential test that
/// drives this (view-angle/range-angle) — see `moveit-constraints`' own
/// module docs for why `target_radius` alone cannot be.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VisibilityConstraintSpec {
    /// `sensor_pose`'s `header.frame_id`.
    pub sensor_frame_id: String,
    /// `sensor_pose`, row-major 4x4.
    pub sensor_pose: [f64; 16],
    /// `sensor_view_direction`: `"sensor_x"`, `"sensor_y"` or `"sensor_z"`.
    pub sensor_view_direction: String,
    /// `target_pose`'s `header.frame_id`.
    pub target_frame_id: String,
    /// `target_pose`, row-major 4x4.
    pub target_pose: [f64; 16],
    /// `cone_sides`.
    pub cone_sides: usize,
    /// `target_radius`; `None` encodes upstream's `0.0` (unconstrained).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target_radius: Option<f64>,
    /// `max_view_angle`; `None` encodes upstream's `0.0` (unconstrained).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_view_angle: Option<f64>,
    /// `max_range_angle`; `None` encodes upstream's `0.0` (unconstrained).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_range_angle: Option<f64>,
    /// `weight`.
    pub weight: f64,
}

/// A shape for [`Op::ShapePoints`] -- the four variants
/// `bodies::createEmptyBodyFromShapeType` has a case for (`Cone`/`Plane`/
/// `OcTree` fall through to a null body upstream and have no `ShapeSpec`
/// here).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ShapeSpec {
    /// `shapes::Sphere`.
    Sphere {
        /// Radius.
        radius: f64,
    },
    /// `shapes::Box`.
    Box {
        /// Extent along x, y, z.
        size: [f64; 3],
    },
    /// `shapes::Cylinder`.
    Cylinder {
        /// Radius.
        radius: f64,
        /// Length along the shape's local z axis.
        length: f64,
    },
    /// `shapes::Mesh`, via `bodies::ConvexMesh`.
    Mesh {
        /// Vertex positions.
        vertices: Vec<[f64; 3]>,
        /// Triangle vertex-index triples. Upstream's own
        /// `ConvexMesh::useDimensions` ignores this entirely and recomputes
        /// a convex hull from `vertices` alone via qhull -- present here
        /// only so the oracle's `shapes::Mesh` constructor (which always
        /// allocates a `triangles` array sized to match) has a value to put
        /// in it.
        triangles: Vec<[u32; 3]>,
    },
}

/// Size, origin and resolution of the grid built by [`Op::DistanceField`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DistanceFieldGeometry {
    /// World-space extent along x, y, z.
    pub size: [f64; 3],
    /// World-space location of cell `(0, 0, 0)`'s corner.
    pub origin: [f64; 3],
    /// The edge length of one (cubic) cell.
    pub resolution: f64,
}

/// One object to build in [`Op::World`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorldObjectSpec {
    /// The object's id.
    pub id: String,
    /// The object's pose, row-major 4x4.
    pub pose: [f64; 16],
    /// Per-shape pose relative to the object's own pose, row-major 4x4 each.
    /// Empty means a shapeless object (`World::setObjectPose`, not
    /// `addToObject`) — needed for the `knowsTransform`/`getTransform`
    /// ambiguity case, which turns on an object existing at all, not on it
    /// carrying shapes. Each shape is a dummy 0.1m sphere on the oracle
    /// side: only pose composition is under test here, not shape geometry.
    #[serde(default)]
    pub shape_poses: Vec<[f64; 16]>,
    /// Subframe name to pose relative to the object's own pose, row-major
    /// 4x4 each.
    #[serde(default)]
    pub subframes: BTreeMap<String, [f64; 16]>,
}

/// A response from the oracle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Response {
    /// Echoes [`Request::id`].
    pub id: u64,
    /// Whether `result` is present.
    pub ok: bool,
    /// Present when `ok`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub result: Option<OracleResult>,
    /// Present when `!ok`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// The payload of a successful [`Response`].
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum OracleResult {
    /// Answer to [`Op::ModelInfo`].
    ModelInfo(ModelInfo),
    /// Answer to [`Op::Fk`].
    Fk(FkResult),
    /// Answer to [`Op::RandomStates`].
    RandomStates(RandomStatesResult),
    /// Answer to [`Op::Jacobian`].
    Jacobian(JacobianResult),
    /// Answer to [`Op::Acm`].
    Acm(AcmResult),
    /// Answer to [`Op::World`].
    World(WorldResult),
    /// Answer to [`Op::DistanceField`].
    DistanceField(DistanceFieldResult),
    /// Answer to [`Op::ShapePoints`].
    ShapePoints(ShapePointsResult),
    /// Answer to [`Op::Constraints`].
    Constraints(ConstraintsResult),
}

/// Structural facts about a `RobotModel`, used by the Phase 1 completion check.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ModelInfo {
    /// `RobotModel::getName()`.
    pub name: String,
    /// `RobotModel::getModelFrame()`.
    pub model_frame: String,
    /// `RobotModel::getRootLinkName()`.
    pub root_link: String,
    /// Every link name, in the model's own order.
    pub links: Vec<String>,
    /// Every joint name, in the model's own order.
    pub joints: Vec<String>,
    /// Per-joint facts.
    pub joint_details: Vec<JointDetail>,
    /// Group name to the joint names it contains.
    pub groups: BTreeMap<String, Vec<String>>,
}

/// Per-joint facts compared in the Phase 1 completion check.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointDetail {
    /// Joint name.
    pub name: String,
    /// `JointModel::getTypeName()`: `Revolute`, `Prismatic`, `Planar`,
    /// `Floating`, `Fixed`, `Unknown`. Capitalized exactly so — upstream
    /// returns these strings verbatim from a switch on its type enum, not the
    /// enumerator spelling.
    pub type_name: String,
    /// `JointModel::getVariableNames()`, in the model's own order.
    ///
    /// The oracle reports these rather than the runner deriving them: MoveIt
    /// names a multi-DOF joint's variables `<joint>/trans_x`, `<joint>/rot_w`
    /// and so on, and any convention invented here would silently disagree.
    pub variable_names: Vec<String>,
    /// Per-variable `(min, max)` position bounds, parallel to
    /// `variable_names`. `None` on a side means unbounded: JSON has no
    /// infinity, and a floating joint's translation limits are infinite while
    /// `position_bounded` still reads true.
    pub bounds: Vec<(Option<f64>, Option<f64>)>,
    /// Per-variable `VariableBounds::position_bounded_`, parallel to
    /// `variable_names`.
    pub position_bounded: Vec<bool>,
    /// The joint this one mimics, if any, with its multiplier and offset.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mimic: Option<Mimic>,
}

/// A mimic relationship.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Mimic {
    /// Name of the mimicked joint.
    pub joint: String,
    /// `value = multiplier * mimicked + offset`.
    pub multiplier: f64,
    /// See `multiplier`.
    pub offset: f64,
}

/// Answer to [`Op::Fk`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FkResult {
    /// Link name to its global transform, row-major 4x4.
    pub link_transforms: BTreeMap<String, [f64; 16]>,
}

/// Answer to [`Op::RandomStates`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RandomStatesResult {
    /// One map of variable name to position per state.
    pub states: Vec<BTreeMap<String, f64>>,
}

/// Answer to [`Op::Jacobian`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JacobianResult {
    /// Row count (6).
    pub rows: usize,
    /// Column count (group DOF).
    pub cols: usize,
    /// Row-major `rows * cols` entries.
    pub data: Vec<f64>,
}

/// Answer to [`Op::Acm`].
///
/// Mirrors `collision_detection::AllowedCollisionMatrix`'s own storage split
/// rather than its merged `getAllowedCollision` view: [`AcmResult::entries`]
/// is the explicit-entry table (`entries_`) and [`AcmResult::defaults`] is the
/// per-name default table (`default_entries_`), each dumped independently —
/// the same tables `moveit-collision`'s `AllowedCollisionMatrix` keeps.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AcmResult {
    /// Every name known to the matrix (`getAllEntryNames`), sorted.
    pub names: Vec<String>,
    /// Every explicit pair entry. Symmetric pairs are reported once, with
    /// `link1 <= link2` in `names` order — the oracle emits only the upper
    /// triangle since `getEntry(a, b, ..)` and `getEntry(b, a, ..)` always
    /// agree.
    pub entries: Vec<AcmEntry>,
    /// Per-name default, keyed by name. `"NEVER"`, `"ALWAYS"` or
    /// `"CONDITIONAL"`, matching [`AcmEntry::kind`].
    pub defaults: BTreeMap<String, String>,
}

/// One explicit pair entry in an [`AcmResult`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AcmEntry {
    /// First link of the pair.
    pub link1: String,
    /// Second link of the pair.
    pub link2: String,
    /// `"NEVER"`, `"ALWAYS"` or `"CONDITIONAL"` — the spelling
    /// `AllowedCollision::Type`'s enumerators print as, transcribed verbatim
    /// rather than reinvented, the same rule `JointDetail::type_name` follows.
    #[serde(rename = "type")]
    pub kind: String,
}

/// Answer to [`Op::World`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorldResult {
    /// Every object built, sorted by id (`World::getObjectIds`'s own order).
    pub objects: Vec<WorldObjectResult>,
    /// One answer per name in [`Op::World`]'s `queries`, in request order.
    pub queries: Vec<WorldQueryResult>,
}

/// One object's dumped state in a [`WorldResult`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorldObjectResult {
    /// The object's id.
    pub id: String,
    /// The object's pose, row-major 4x4.
    pub pose: [f64; 16],
    /// Per-shape pose and global pose, in the object's own shape order.
    pub shapes: Vec<WorldShapeResult>,
    /// Subframe name to its pose and global pose.
    pub subframes: BTreeMap<String, WorldSubframeResult>,
}

/// One shape's pose and global pose in a [`WorldObjectResult`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorldShapeResult {
    /// Pose relative to the object's own pose, row-major 4x4.
    pub pose: [f64; 16],
    /// Pose in the world frame, row-major 4x4.
    pub global_pose: [f64; 16],
}

/// One subframe's pose and global pose in a [`WorldObjectResult`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorldSubframeResult {
    /// Pose relative to the object's own pose, row-major 4x4.
    pub pose: [f64; 16],
    /// Pose in the world frame, row-major 4x4.
    pub global_pose: [f64; 16],
}

/// Answer to one name in [`Op::World`]'s `queries`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorldQueryResult {
    /// The queried name, echoed back.
    pub name: String,
    /// `World::knowsTransform(name)`.
    pub knows_transform: bool,
    /// `World::getTransform(name, frame_found)`, row-major 4x4, present only
    /// when `frame_found` came back `true`. This can be `Some` while
    /// [`WorldQueryResult::knows_transform`] is `false` — see `world.rs`'s
    /// module docs — for a subframe name colliding with a sibling object's
    /// name; that is the documented upstream ambiguity, not a bug in either
    /// field.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub transform: Option<[f64; 16]>,
}

/// Answer to [`Op::DistanceField`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DistanceFieldResult {
    /// One answer per cell in [`Op::DistanceField`]'s `queries`, in request
    /// order.
    pub queries: Vec<DistanceFieldQueryResult>,
}

/// Answer to one cell in [`Op::DistanceField`]'s `queries`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DistanceFieldQueryResult {
    /// The queried cell, echoed back.
    pub cell: [i32; 3],
    /// `PropagationDistanceField::isCellValid(x, y, z)`.
    pub in_grid: bool,
    /// `gridToWorld(x, y, z)` -- computed for every query regardless of
    /// `in_grid`, matching upstream's own unconditional arithmetic.
    pub world: [f64; 3],
    /// `getDistance(double, double, double)` at [`Self::world`]. Safe to call
    /// for any world point, in or out of the grid.
    pub distance_world: f64,
    /// `getDistance(int, int, int)` at [`Self::cell`]. `None` when
    /// `!in_grid`: upstream documents this overload as needing a valid cell
    /// "or corruption occurs".
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance_cell: Option<f64>,
    /// `getDistanceGradient` at [`Self::world`]. Handles an out-of-grid point
    /// itself (`in_bounds: false`, zero gradient), so this is always present.
    pub gradient: DistanceFieldGradient,
    /// `getNearestCell(x, y, z, ...)` at [`Self::cell`]. `None` when
    /// `!in_grid`, for the same reason as [`Self::distance_cell`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub nearest: Option<DistanceFieldNearest>,
}

/// `getDistanceGradient`'s result, dumped in a [`DistanceFieldQueryResult`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DistanceFieldGradient {
    /// The distance value returned alongside the gradient.
    pub distance: f64,
    /// The gradient vector. Zero when `!in_bounds`.
    pub gradient: [f64; 3],
    /// `false` within one cell of the grid boundary, where a gradient needs
    /// padding this query cannot have.
    pub in_bounds: bool,
}

/// `getNearestCell`'s result, dumped in a [`DistanceFieldQueryResult`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DistanceFieldNearest {
    /// Signed distance: negative inside an obstacle, positive outside, zero
    /// when unknown.
    pub distance: f64,
    /// The nearest cell's position.
    pub position: [i32; 3],
    /// Whether upstream's `getNearestCell` returned a non-null pointer.
    ///
    /// Deliberately not the neighbor voxel's own fields: for a query that
    /// reaches the `PropDistanceFieldVoxel::UNINITIALIZED` (`-1, -1, -1`)
    /// sentinel path -- a cell farther than `max_distance` from every
    /// obstacle, never visited by propagation -- upstream reads
    /// `voxel_grid_->getCell(-1, -1, -1)` unguarded and returns that address
    /// as non-null, which is well-defined to *form* but memory-unsafe to
    /// dereference. `true` here for such a query is the documented upstream
    /// defect this crate's own `nearest_cell` closes by returning `None`
    /// instead -- see `propagation.rs`'s deviation doc.
    pub voxel_present: bool,
}

/// Answer to [`Op::ShapePoints`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ShapePointsResult {
    /// Every point `findInternalPointsConvex` found, in upstream's own
    /// nested-loop enumeration order (x outer, y middle, z inner) -- not
    /// deduplicated or sorted, since points are compared as a set.
    pub points: Vec<[f64; 3]>,
}

/// Answer to [`Op::Constraints`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ConstraintResult {
    /// `ConstraintEvaluationResult::satisfied`.
    pub satisfied: bool,
    /// `ConstraintEvaluationResult::distance`.
    pub distance: f64,
}

/// Answer to [`Op::Constraints`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConstraintsResult {
    /// One entry per constraint in [`ConstraintsSpec`], flattened in
    /// joint/position/orientation/visibility order (matching
    /// `KinematicConstraintSet::add`'s own internal call order).
    pub results: Vec<ConstraintResult>,
}
