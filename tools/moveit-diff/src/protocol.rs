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
    /// Ground truth for `moveit-collision::ParryCollisionEnv`
    /// (PORTING-PLAN.md §5's Phase 3 completion condition):
    /// `CollisionEnvFCL::checkSelfCollision`/`checkRobotCollision`/
    /// `distanceSelf`/`distanceRobot` at `joint_values`, filtered through the
    /// SRDF-derived `AllowedCollisionMatrix` (the same construction
    /// [`Op::Acm`] dumps — built fresh from the loaded SRDF on the oracle
    /// side, and independently via `AllowedCollisionMatrix::from_srdf` on the
    /// runner side, rather than sent over the wire, since `acm_parity.rs`
    /// already differentially tests that the two constructions agree).
    Collision {
        /// Joint name to position. Joints omitted keep their default value.
        joint_values: BTreeMap<String, f64>,
        /// World objects `checkRobotCollision`/`distanceRobot` check the
        /// robot against, built with real shapes (unlike [`Op::World`]'s
        /// dummy spheres, which only exercise pose composition).
        objects: Vec<CollisionObjectSpec>,
        /// Bodies to attach to the state via `RobotState::attachBody` before
        /// running any check, ground truth for `moveit_scene::AttachedBody`/
        /// `moveit_collision::AttachedBodyGeometry`. Empty means the plain
        /// [`Op::Collision`] behavior every existing fixture already
        /// exercises.
        #[serde(default)]
        attached_bodies: Vec<AttachedBodySpec>,
    },
    /// IK success-rate comparison for `group` (Phase 4's completion
    /// condition -- see `PORTING-PLAN.md`). Both sides solve independently:
    /// the oracle via a hand-transcribed `KDLKinematicsPlugin::searchPositionIK`
    /// over the real, vendored `ChainIkSolverVelMimicSVD` (see
    /// `tools/moveit-oracle/src/oracle.cpp`'s `ik()`), this side's own
    /// `moveit_kinematics::NewtonRaphsonSolver` -- the direct port of that
    /// same upstream solver, not `LevenbergMarquardtSolver`, which has no
    /// upstream counterpart to compare a success rate against.
    ///
    /// No target pose or seed crosses the wire. `joint_values` is a full
    /// model configuration (same shape as [`Op::Fk`]/[`Op::Jacobian`]) drawn
    /// from [`Op::RandomStates`], so it is reachable by construction; each
    /// side runs its own forward kinematics on `group`'s own chain to get
    /// the target pose in the chain's base-link frame. The seed is likewise
    /// computed independently on each side, deterministically, as each
    /// active joint's own `(min + max) / 2` -- safe to do without agreeing
    /// over the wire because [`Op::ModelInfo`]'s `JointDetail::bounds`
    /// already has to agree between the two sides for the model_info
    /// comparison to pass, continuous joints included (see that field's doc
    /// comment on the finite `[-pi, pi]` convention).
    Ik {
        /// Joint model group name; must be a chain group.
        group: String,
        /// Full joint values defining the reachable target pose (see
        /// [`Op::Fk`]'s `joint_values`).
        joint_values: BTreeMap<String, f64>,
        /// Whether to run position-only IK.
        position_only: bool,
        /// `moveit_kinematics::SolverParams::max_restarts` on this side,
        /// `kMaxRestarts` on the oracle side. Sent explicitly, not defaulted
        /// on either side, because it is the knob the round-2 investigation
        /// into moveit-rs's IK success-rate gap needs to isolate: at `0`,
        /// both sides run exactly one deterministic attempt from the
        /// identical bounds-midpoint seed, with no randomness anywhere on
        /// either side, so a surviving gap cannot be restart-RNG divergence
        /// (the oracle draws restart reseeds from `random_numbers::
        /// RandomNumberGenerator`, a boost mt19937 stream; this side from
        /// `ChaCha8Rng` -- two independent streams whose per-case outcomes
        /// were never comparable at `max_restarts > 0`).
        max_restarts: u32,
        /// `searchPositionIK`'s `consistency_limits` parameter, full-space
        /// (one entry per chain joint, active and mimic alike) and keyed by
        /// joint name rather than upstream's positional
        /// `dimension_`-sized `Vec<f64>`, for the same reason
        /// `joint_values` is a map: the oracle's and this side's chain
        /// traversals never have to agree on an encounter order. Empty (the
        /// default) means "no consistency limits", matching
        /// `moveit_kinematics::registry::SolveOptions::consistency_limits`
        /// being [`None`].
        ///
        /// Both sides reduce this full-space map to their own
        /// active-joint-only vector independently -- the oracle via
        /// `oracle.cpp`'s `ik()` (mirroring `KDLKinematicsPlugin::
        /// searchPositionIK`'s own filter to `consistency_limits_mimic`),
        /// this side via `rust_impl::IkSolver::solve_case` reading
        /// [`Op::Ik::consistency_limits`] at each of its own
        /// active-joint names. Doing the reduction on each side from the
        /// same full-space input is what makes this an oracle-side check of
        /// that reduction rather than a pre-reduced, circular one -- see
        /// `PORTING-PLAN.md`'s round-4 `p1-joints` section for the
        /// `checkConsistency` out-of-bounds read this shape is designed not
        /// to reproduce.
        #[serde(default)]
        consistency_limits: BTreeMap<String, f64>,
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

/// One world object for [`Op::Collision`]: a real shape at a pose, unlike
/// [`WorldObjectSpec`]'s dummy sphere (that op tests pose composition only;
/// this one needs actual geometry for a non-trivial collision/distance
/// answer).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CollisionObjectSpec {
    /// The object's id.
    pub id: String,
    /// The object's pose, row-major 4x4.
    pub pose: [f64; 16],
    /// The object's collision shape.
    pub shape: ShapeSpec,
}

/// One body for [`Op::Collision`]'s `attached_bodies`: ground truth for
/// `RobotState::attachBody(id, pose, shapes, shape_poses, touch_links,
/// link_name)`. `pose` (upstream's separate object-pose level between the
/// link and its shapes) is not a field here: the oracle side always passes
/// `Eigen::Isometry3d::Identity()` for it, matching
/// `moveit_scene::AttachedBody`'s own one-level design (its own module doc)
/// where `shape_poses` are already relative to the link frame directly.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AttachedBodySpec {
    /// `AttachedBody::getName()`.
    pub id: String,
    /// The link to attach to.
    pub link_name: String,
    /// This body's shapes, parallel to `shape_poses`.
    pub shapes: Vec<ShapeSpec>,
    /// Each shape's pose relative to `link_name`'s own frame, row-major 4x4,
    /// parallel to `shapes`.
    pub shape_poses: Vec<[f64; 16]>,
    /// `AttachedBody::getTouchLinks()`.
    #[serde(default)]
    pub touch_links: Vec<String>,
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
    /// Answer to [`Op::Collision`].
    Collision(CollisionCheckResult),
    /// Answer to [`Op::Ik`].
    Ik(IkResult),
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

/// Identifies the pair of bodies behind one `DistanceResultsData::minimum_distance`
/// — the oracle's `distancePairToJson`, this port's [`crate::rust_impl::collision`].
/// Named directly off `DistanceResultsData::link_names`/`body_types` on both
/// sides rather than inferred, so a `distance differs` report can say *which*
/// pair each side picked instead of only that the scalars disagree — round 8
/// item 1's diagnostic addition, after pr2 case 7552 sat unnamed for three
/// rounds with only the scalar to go on. `None` when upstream's
/// `DistanceResultsData::clear()` state was never overwritten (no other body
/// existed to measure against). Deliberately excludes shape kinds: the
/// oracle's own `self_distance_pair`/`robot_distance_pair` JSON carries them
/// too, but naming the pair only needs a link/body-type identity, and this
/// struct is deserialized against that JSON with unmatched fields simply
/// ignored (no `deny_unknown_fields`), so the oracle can carry more than this
/// side names without desyncing the wire format.
///
/// What round 8 item 1's pair names actually showed, against pr2 at 300
/// cases (`--seed 20260804`, `fixtures/pr2.urdf`/`.srdf`): every
/// `self_distance` "distance differs" failure traces to just two pairs on
/// this side, not three unrelated ones. `base_bellow_link`/`torso_lift_link`
/// produces both `-5.29289090633392e-2` (177/300, the dominant plateau) and
/// `-4.91695723318727e-2` (once, case 18) plus a dozen more nearby-but-
/// distinct values — the pair drifts in a narrow band (~-0.047 to -0.053) as
/// `torso_lift_joint` moves, rather than sitting at one frozen number. The
/// remaining 123/300 are eight *different* pairs — `base_link` against each
/// of the eight `*_caster_*_wheel_link`s — that collapse onto the same
/// `-4.65920000000832e-2` to ~13 significant digits: each caster wheel's
/// mesh is rotationally symmetric about its own roll axis, so the wheel-roll
/// continuous joint's rotation cannot change the closest-point distance to
/// `base_link` at all, and float noise is all that's left to tell the eight
/// apart. (Two earlier counts of this same run were both wrong, and not for
/// the same reason: `340/600`/`246/600` doubled the denominator by counting
/// both a case's inline `FAIL` line and the run's own end-of-log `Nx
/// (first: ...)` aggregate line for it (PORTING-PLAN.md §53.2); the fix
/// that followed still landed on `170/300`. That script no longer exists to
/// inspect, but round 10's re-derivation hit an analogous bug before
/// correcting it: a naive `vs rust [...]` match against the *whole*
/// "distance differs" message double-counts, since the message names a rust
/// pair twice -- once for `self_distance`, once for `robot_distance` -- and
/// gave a `189`/`299` split that does not even sum to 300. Scoping the match
/// to the text before `, robot oracle` is what gives the `177/300` above,
/// confirmed against PORTING-PLAN.md §60.3's independent reproduction of
/// the same run.) The oracle's own minimum on the
/// same cases is a different pair almost every time (mostly
/// `*_gripper_*_finger*` pairs at `-1e-2`..`-1e-1`) -- consistent with
/// p3-acm's case-7552 finding that this is a pair-ranking flip, not a
/// same-pair depth drift: this port's self-distance search finds these two
/// near-static pairs and apparently never gets to weigh the gripper pairs
/// against them. Diagnostic only, handed to p3-acm (who own
/// `moveit-collision`) rather than fixed here.
///
/// Round 9 extended this to `robot_distance` (robot vs. world object) and
/// to whether a pair disagreement is even a defect: at 3000 cases (same
/// seed/fixture), `self` pairs disagree on 2935/3000 (97.8%), *all* of
/// which also exceed `cfg.tol_distance` -- the self side is essentially
/// always wrong on pr2. `robot` pairs disagree far more often (2647/3000,
/// 88.2%) but almost all are benign ties: pr2's eight caster wheels are
/// equidistant from `floor`, so any of them is a correct nearest-body
/// answer and the scalar still agrees to `~1e-12`. Only 9/3000 (0.30%)
/// robot-side disagreements also exceed tolerance, and all nine involve a
/// gripper finger against `floor`: 7 are pair-ranking flips (this port
/// picks a `*_finger_tip_link`, the oracle picks the adjacent
/// `*_finger_link`/`*_gripper_palm_link`), 2 are same-pair depth-only
/// disagreements (`collision[422]`, `collision[2996]`, both sides name
/// `*_finger_tip_link`/`floor` but disagree on the distance). All 9 of 9
/// have this port's answer *deeper* (more negative) than the oracle's --
/// one-directional within this species, unlike the self-collision example
/// above (this port shallower) -- so §43's "port answers shallower" framing
/// does not generalize across species even though within the
/// gripper-vs-floor species the direction is consistent. See
/// PORTING-PLAN.md §53.3 and this round's report for the full case list.
/// Diagnostic only.
///
/// PORTING-PLAN.md §67.2 found a same-pair-and-value-diverges species on the
/// self side too -- 62/3000, unlike the robot side's 2/3000, a case where
/// both sides pick the identical pair and only the scalar disagrees, so no
/// ranking bug (§56) can be the cause. Round 11 broke the 62 down by pair
/// identity (same seed/fixture, via `--stats-json`'s
/// `distance_pairs.self_same_pair_histogram`): 52/62 are
/// `base_bellow_link`/`torso_lift_link`, the already-explained §56 plateau
/// (`min(0.052928909, ramp(t))`, this port correct). The remaining 10/62
/// split across five distinct `base_link`/`*_caster_*_wheel_link` pairs
/// (1, 1, 1, 3, 4 occurrences) -- new, and *not* explained by §56's
/// min-of-two-candidates mechanism, which was derived specifically for the
/// bellow/torso pair's two competing contact directions and says nothing
/// about a caster wheel's geometry. Whether it is the same TriMesh
/// per-triangle-MTD underestimate §56.4 already flags as a general risk
/// (not proven, only not yet ruled out for this pair family) is unmeasured.
/// Diagnostic only, handed to p3-acm alongside the histogram itself.
///
/// Round 11 also settled whether the robot side's 2/3000 (0.067%) is a
/// genuine tail or an undersampled reading of the self side's ~2% rate: at
/// 30000 cases (same seed/fixture, 10x), self same-pair divergence is
/// 532/30000 (1.77%) and robot is 14/30000 (0.047%) -- both rates held
/// within a factor of 1.4 of their 3000-case values instead of converging
/// toward each other, and the ~38x gap (30000-case ratio) matches the
/// 3000-case sample's ~31x. If the robot side were really drawing from the
/// same ~2% population, 30000 cases would read roughly 600 hits, not 14 --
/// so this is a real rate difference intrinsic to the pair populations, not
/// sampling noise, exactly the alternative PORTING-PLAN.md §67.2 raised
/// (both sides share `accumulate_distance`, `parry.rs:1109`/`:1125` calling
/// `:902`, per p3-acm). The 30000-case pair composition is consistent with
/// the 3000-case read too: self is 440/532 (82.7%) bellow/torso plateau
/// plus 92/532 (17.3%) spread over all eight `base_link`/caster-wheel
/// pairs now (7-16 each, roughly even -- every caster wheel eventually
/// shows up, not just the five seen at 3000 cases); robot's 14 are all
/// `floor`/`*_gripper_*_finger_tip_link`, split 5/4/3/2 across the four
/// fingers.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DistancePair {
    /// `DistanceResultsData::link_names[0]`.
    pub body_name_1: String,
    /// `DistanceResultsData::body_types[0]`, named via the oracle's
    /// `bodyTypeName`/this port's own equivalent: `"robot_link"`,
    /// `"robot_attached"` or `"world_object"`.
    pub body_type_1: String,
    /// `DistanceResultsData::link_names[1]`.
    pub body_name_2: String,
    /// `DistanceResultsData::body_types[1]`, named the same way as
    /// `body_type_1`.
    pub body_type_2: String,
}

/// Answer to [`Op::Collision`].
///
/// Contact/nearest-point coordinates are deliberately absent: PORTING-PLAN.md
/// §4.5 records their exclusion from Phase 3's completion condition as a
/// verification limit, not an oversight — see `crates/moveit-collision/src/parry.rs`'s
/// module doc, deviations 4 and 6, for why a coordinate-level comparison
/// would not be meaningful here even if it were attempted.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CollisionCheckResult {
    /// `CollisionEnvFCL::checkSelfCollision`'s `CollisionResult::collision`.
    pub self_collision: bool,
    /// `CollisionEnvFCL::distanceSelf`'s `DistanceResult::minimum_distance.distance`,
    /// signed (`enable_signed_distance = true`).
    pub self_distance: f64,
    /// The pair behind `self_distance`. See [`DistancePair`].
    pub self_distance_pair: Option<DistancePair>,
    /// `CollisionEnvFCL::checkRobotCollision`'s `CollisionResult::collision`.
    pub robot_collision: bool,
    /// `CollisionEnvFCL::distanceRobot`'s `DistanceResult::minimum_distance.distance`,
    /// signed (`enable_signed_distance = true`).
    pub robot_distance: f64,
    /// The pair behind `robot_distance`. See [`DistancePair`].
    pub robot_distance_pair: Option<DistancePair>,
}

/// Answer to [`Op::Ik`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IkResult {
    /// Whether `searchPositionIK`-equivalent found a solution within this
    /// side's own iteration/restart budget.
    pub success: bool,
    /// The solved joint values, one entry per `group`'s active (non-mimic)
    /// joint, keyed by name so the two sides never need to agree on a
    /// vector order over the wire. Present only when `success`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub solution: Option<BTreeMap<String, f64>>,
}
