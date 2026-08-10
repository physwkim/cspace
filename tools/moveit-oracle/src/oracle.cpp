// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Differential-test oracle: loads a URDF/SRDF pair with the C++ MoveIt 2
// implementation and answers JSON requests about it, one per line on stdin.
//
// Built against moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf.
// The wire shapes here must stay in step with
// tools/moveit-diff/src/protocol.rs.
//
// This binary links only moveit_core, with one exception: the
// `acceleration_filter` op (see its own comment below) constructs a single,
// never-spun rclcpp::Node because AccelerationLimitedPlugin::initialize's own
// signature leaves no other way to reach it, and loads the plugin through
// pluginlib::ClassLoader rather than a direct CMake target link, since
// moveit_core's build only puts that library into its pluginlib description
// index (`moveit_core_pluginTargets`), not the `moveit_coreTargets` CMake
// export set every other library here is pulled in through. That op aside,
// nothing in this binary starts discovery, runs a spin loop, or touches
// /clock.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shapes.h>
#include <nlohmann/json.hpp>
#include <octomap/octomap.h>

#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/collision_detection/world.hpp>
#include <moveit/collision_distance_field/collision_common_distance_field.hpp>
#include <moveit/collision_detection_fcl/collision_env_fcl.hpp>
#include <moveit/collision_distance_field/collision_distance_field_types.hpp>
#include <moveit/collision_distance_field/collision_env_distance_field.hpp>
// For the chomp_plan op's hybrid planning scene -- CHOMP's optimizer requires
// it, see `chompPlan`'s doc comment.
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.hpp>
#include <moveit/distance_field/find_internal_points.hpp>
#include <moveit/distance_field/propagation_distance_field.hpp>
#include <moveit/dynamics_solver/dynamics_solver.hpp>
#include <moveit/kinematic_constraints/kinematic_constraint.hpp>
#include <moveit/kinematics_metrics/kinematics_metrics.hpp>
#include <moveit/online_signal_smoothing/smoothing_base_class.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_model/revolute_joint_model.hpp>
#include <random_numbers/random_numbers.h>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/transforms/transforms.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/bounding_volume.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/visibility_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// The `ik` op's solver: KDL::ChainIkSolverVelMimicSVD, vendored verbatim (see
// that header's own comment for why) plus kdl_parser to build a KDL::Chain
// straight from the same urdf::ModelInterface this oracle already parses.
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "third_party/kdl_kinematics_plugin/chainiksolver_vel_mimic_svd.hpp"

// The `acceleration_filter` op's plugin load and its one-off, never-spun
// rclcpp::Node -- see the file header above and that op's own comment.
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

// The `plan` op's C++ baseline planner. Plain OMPL, not
// moveit_planners_ompl: OMPL 1.7 is already in the base image (headers,
// libompl.so and omplConfig.cmake all under /opt/ros/${ROS_DISTRO}), so this
// costs no widening of MOVEIT2_PACKAGES, whereas moveit_planners_ompl would
// pull moveit_ros_planning and its rclcpp/tf2_ros tree into the colcon build.
// See `plan()`'s own comment for what that choice does and does not buy.
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>

#include <moveit/robot_model/floating_joint_model.hpp>
#include <moveit/robot_model/planar_joint_model.hpp>

// The `pilz_trajectory` op's four generators. Unlike every other planner in
// this file these are reached by direct construction, not through pluginlib:
// `TrajectoryGeneratorPTP`/`LIN`/`CIRC`/`Polyline` all export their
// `(RobotModelConstPtr, LimitsContainer, group_name)` constructor and inherit
// the one public entry point `TrajectoryGenerator::generate`, all in
// the installed shared objects (verified by `nm -DC`, PORTING-PLAN.md §130).
// So this costs a widened MOVEIT2_PACKAGES but no rclcpp::Node.
//
// `Polyline` is the exception on two counts. Upstream puts
// `trajectory_generator_polyline.cpp` in its own
// `planning_context_loader_polyline` target, which CMakeLists.txt therefore
// links separately; and its header cannot be included here at all, because it
// redeclares three of `trajectory_generator_lin.hpp`'s exception classes. It
// is reached through `pilz_polyline_factory.hpp` instead -- that file's own
// header explains the redefinition.
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/robot_state/conversions.hpp>

// The `chomp_quad_cost_inverse` op. The real upstream `ChompCost` rather
// than a transcription of its constructor: the question this op answers is
// whether Eigen's inverse differs from nalgebra's, so any hand-copied
// matrix assembly would put a second, untested variable in the comparison
// and a transcription slip would read as a decomposition difference.
#include <chomp_motion_planner/chomp_cost.hpp>
#include <chomp_motion_planner/chomp_trajectory.hpp>
// For the `chomp_plan` op's real `chomp::ChompPlanner::solve` -- the entry
// point `chomp_interface`'s pluginlib wrapper forwards to unchanged.
#include <chomp_motion_planner/chomp_planner.hpp>
#include <chomp_motion_planner/chomp_parameters.hpp>
// For the `stomp_plan` op. These resolve into /ws/src/moveit2 rather than an
// installed package: `moveit_planners_stomp` is not one of this image's built
// packages, and its planning code is compiled into the oracle from upstream's
// own sources instead -- see CMakeLists.txt and `stompPlan`'s doc comment.
#include <stomp_moveit/stomp_moveit_planning_context.hpp>
#include <moveit_planners_stomp/stomp_moveit_parameters.hpp>
// `rsl::rng` is the single random source both C++ planners draw from; see
// `seedRslOnce`.
#include <rsl/random.hpp>
#include <pilz_industrial_motion_planner/joint_limits_container.hpp>
#include <pilz_industrial_motion_planner/limits_container.hpp>
#include <pilz_industrial_motion_planner/trajectory_generator_circ.hpp>
#include <pilz_industrial_motion_planner/trajectory_generator_lin.hpp>
#include <pilz_industrial_motion_planner/trajectory_generator_ptp.hpp>

#include "pilz_polyline_factory.hpp"

// The `pilz_blend` op. Same story as the three generators above: `blend` is
// the one public entry point and `TrajectoryBlenderTransitionWindow` exports
// its `(const LimitsContainer&)` constructor, both already inside
// `pilz_industrial_motion_planner::trajectory_generation_common`, which
// CMakeLists.txt has linked since the `pilz_trajectory` op landed. No new
// link target and no rclcpp::Node.
#include <pilz_industrial_motion_planner/trajectory_blend_request.hpp>
#include <pilz_industrial_motion_planner/trajectory_blend_response.hpp>
#include <pilz_industrial_motion_planner/trajectory_blender_transition_window.hpp>

using json = nlohmann::json;

namespace
{

std::string readFile(const std::string& path)
{
  std::ifstream in(path);
  if (!in)
    throw std::runtime_error("cannot open " + path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

/// null for a non-finite limit; JSON has no representation for infinity.
json finiteOrNull(double v)
{
  return std::isfinite(v) ? json(v) : json(nullptr);
}

/// Row-major 4x4, matching FkResult::link_transforms in protocol.rs.
json toRowMajor4x4(const Eigen::Isometry3d& t)
{
  const Eigen::Matrix4d m = t.matrix();
  json out = json::array();
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      out.push_back(m(r, c));
  return out;
}

/// Inverse of toRowMajor4x4. `Eigen::Isometry3d::matrix()` is a plain 4x4
/// affine matrix with no runtime orthogonality check, so assigning any
/// caller-supplied 4x4 into it round-trips exactly -- the request is trusted
/// to carry a genuine isometry, the same trust nalgebra::Isometry3 encodes by
/// construction on the Rust side.
Eigen::Isometry3d fromRowMajor4x4(const json& arr)
{
  Eigen::Matrix4d m;
  auto it = arr.begin();
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      m(r, c) = (*it++).get<double>();
  Eigen::Isometry3d t;
  t.matrix() = m;
  return t;
}

/// `geometry_msgs::msg::Pose` for `t`, matching `rust_impl::to_row_major_4x4`'s
/// own row-major encoding on the wire (this helper builds a message, not JSON
/// -- the wire pose a `constraints` request carries is decoded via
/// `fromRowMajor4x4` first, then re-expressed as a `Pose` message here for
/// `moveit_msgs`' own fields to hold).
geometry_msgs::msg::Pose isometryToPoseMsg(const Eigen::Isometry3d& t)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = t.translation().x();
  pose.position.y = t.translation().y();
  pose.position.z = t.translation().z();
  const Eigen::Quaterniond q(t.rotation());
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

/// `shape_msgs::msg::SolidPrimitive` for one [`Op::Constraints`] region's
/// `shape` field (see `protocol.rs`'s `ShapeSpec`) -- `"mesh"` is not handled
/// here since a `SolidPrimitive` has no mesh case; the caller routes a mesh
/// region into `BoundingVolume::meshes`/`mesh_poses` instead (see
/// `positionConstraintFromJson`).
///
/// `"cone"` is a real `SolidPrimitive` type (`CONE`/`CONE_HEIGHT`/
/// `CONE_RADIUS` all exist on this build's `shape_msgs`) and
/// `shapes::constructShapeFromMsg` builds a genuine `shapes::Cone` from one
/// (`shape_operations.cpp:101-106`), so this function accepts it -- but its
/// only caller, `positionConstraintFromJson`, rejects a `"cone"` region
/// before it reaches here: `kinematic_constraints::PositionConstraint::
/// configure` feeds every `constraint_region.primitives[i]` straight to
/// `bodies::createEmptyBodyFromShapeType` with no null check
/// (`kinematic_constraint.cpp:411-412`), and `bodies::` has no `CONE` case,
/// so a cone constraint region crashes real upstream MoveIt exactly as it
/// would crash this oracle -- there is no ground truth this could be
/// compared against. See `requireBodySupport`'s doc for the `bodies::` gap
/// this is the constraint-region side of.
shape_msgs::msg::SolidPrimitive solidPrimitiveFromJson(const json& shape_json)
{
  shape_msgs::msg::SolidPrimitive primitive;
  const std::string type = shape_json.at("type").get<std::string>();
  if (type == "sphere")
  {
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions = { shape_json.at("radius").get<double>() };
  }
  else if (type == "box")
  {
    const auto size = shape_json.at("size").get<std::array<double, 3>>();
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = { size[0], size[1], size[2] };
  }
  else if (type == "cylinder")
  {
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] =
        shape_json.at("length").get<double>();
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] =
        shape_json.at("radius").get<double>();
  }
  else if (type == "cone")
  {
    primitive.type = shape_msgs::msg::SolidPrimitive::CONE;
    primitive.dimensions.resize(2);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CONE_HEIGHT] =
        shape_json.at("length").get<double>();
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CONE_RADIUS] =
        shape_json.at("radius").get<double>();
  }
  else
  {
    throw std::runtime_error("constraints: unsupported primitive shape type " + type);
  }
  return primitive;
}

/// `Eigen::Isometry3d` -> `KDL::Frame`, for the `ik` op's target pose --
/// `tf2_kdl::fromMsg` is not available here (this oracle links no ROS
/// message/tf2 packages), so this goes straight from the isometry instead of
/// via a `geometry_msgs::msg::Pose` the way `KDLKinematicsPlugin::
/// searchPositionIK` does.
KDL::Frame toKdlFrame(const Eigen::Isometry3d& t)
{
  const Eigen::Quaterniond q(t.rotation());
  const Eigen::Vector3d& p = t.translation();
  return { KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w()), KDL::Vector(p.x(), p.y(), p.z()) };
}

/// Matches the `kind` strings `cspace_core::model::Diagnostic::UnsupportedLinkGeometry`
/// and the `link_details[].shape_types` wire field use — this oracle's own
/// naming, not upstream's (`shapes::Shape` has no built-in name accessor;
/// `ShapeType`'s `operator<<` exists but is a debug print, not a stable wire
/// contract).
std::string shapeTypeName(shapes::ShapeType type)
{
  switch (type)
  {
    case shapes::SPHERE:
      return "sphere";
    case shapes::CYLINDER:
      return "cylinder";
    case shapes::CONE:
      return "cone";
    case shapes::BOX:
      return "box";
    case shapes::PLANE:
      return "plane";
    case shapes::MESH:
      return "mesh";
    case shapes::OCTREE:
      return "octree";
    case shapes::UNKNOWN_SHAPE:
      break;
  }
  throw std::runtime_error("unknown shapes::ShapeType");
}

/// Matches `collision_detection::BodyTypes::Type`'s three variants -- this
/// oracle's own naming for the `body_type_1`/`body_type_2` fields the
/// `collision` op's `self_contacts`/`robot_contacts` entries carry, same
/// convention as `shapeTypeName` above.
std::string bodyTypeName(collision_detection::BodyType type)
{
  switch (type)
  {
    case collision_detection::BodyTypes::ROBOT_LINK:
      return "robot_link";
    case collision_detection::BodyTypes::ROBOT_ATTACHED:
      return "robot_attached";
    case collision_detection::BodyTypes::WORLD_OBJECT:
      return "world_object";
  }
  throw std::runtime_error("unknown collision_detection::BodyType");
}

/// Builds the `shapes::Shape` named by `request["shape"]["type"]` --
/// `"sphere"`, `"box"`, `"cylinder"`, `"cone"` or `"mesh"`. Shared by every op
/// that takes a shape over the wire, including `addRequestObjects` (so both
/// FCL-based ops -- `collision`, `pair_signed_distance`, `scene_diff_collision`
/// -- and `bodies::`-based ones -- `shapePoints`, `bodyQuery`,
/// `collisionDistanceFieldTypes`, `collisionObjectPointDecomposition`,
/// `distanceFieldCacheEntry`, `groupStateRepresentation` -- reach it) and
/// `shapePoints`/`collisionDistanceFieldTypes` directly.
///
/// `"cone"` is real FCL geometry (`collision_common.cpp:894-897` builds
/// `fcl::Coned(radius, length)` for `shapes::CONE`) but `bodies::` has no
/// case for it -- `body_operations.cpp`'s `createEmptyBodyFromShapeType`
/// switches on `BOX`/`SPHERE`/`CYLINDER`/`MESH` only and returns `nullptr`
/// otherwise, which every caller of it (oracle and upstream alike)
/// dereferences without checking. Every `bodies::`-based op above therefore
/// guards its own shape(s) with `requireBodySupport` before this function's
/// wider result can reach that dereference; see that function's doc for the
/// full call-site list this was audited against.
std::shared_ptr<shapes::Shape> parseShape(const std::string& type, const json& shape_json)
{
  if (type == "sphere")
  {
    return std::make_shared<shapes::Sphere>(shape_json.at("radius").get<double>());
  }
  if (type == "box")
  {
    const auto size = shape_json.at("size").get<std::array<double, 3>>();
    return std::make_shared<shapes::Box>(size[0], size[1], size[2]);
  }
  if (type == "cylinder")
  {
    return std::make_shared<shapes::Cylinder>(shape_json.at("radius").get<double>(),
                                               shape_json.at("length").get<double>());
  }
  if (type == "cone")
  {
    return std::make_shared<shapes::Cone>(shape_json.at("radius").get<double>(),
                                           shape_json.at("length").get<double>());
  }
  if (type == "mesh")
  {
    const auto vertices_json = shape_json.at("vertices");
    const auto triangles_json = shape_json.at("triangles");
    auto mesh = std::make_shared<shapes::Mesh>(vertices_json.size(), triangles_json.size());
    for (std::size_t i = 0; i < vertices_json.size(); ++i)
    {
      const auto v = vertices_json[i].get<std::array<double, 3>>();
      mesh->vertices[3 * i] = v[0];
      mesh->vertices[3 * i + 1] = v[1];
      mesh->vertices[3 * i + 2] = v[2];
    }
    for (std::size_t i = 0; i < triangles_json.size(); ++i)
    {
      const auto t = triangles_json[i].get<std::array<unsigned int, 3>>();
      mesh->triangles[3 * i] = t[0];
      mesh->triangles[3 * i + 1] = t[1];
      mesh->triangles[3 * i + 2] = t[2];
    }
    return mesh;
  }
  throw std::runtime_error("unsupported shape type " + type);
}

/// Throws unless `bodies::createEmptyBodyFromShapeType` can actually build a
/// body for `type`. Calls that exact factory -- the same one every direct
/// caller below reaches, and the same one upstream's own
/// `BodyDecomposition`/`CollisionEnvDistanceField`/`PositionConstraint`
/// reach internally -- and checks its real answer rather than re-deriving
/// which types it supports in a second switch that could drift from it.
///
/// `parseShape` now accepts `"cone"` for the FCL-based ops that genuinely
/// support it, but `bodies::` does not, and every caller of
/// `createEmptyBodyFromShapeType` (oracle code and upstream code alike)
/// dereferences its result without a null check -- so a `Cone` reaching one
/// unguarded would be a process-ending null-pointer dereference instead of a
/// clean per-request error. Audited call sites, each guarded with this
/// before it can build or feed a `bodies::Body`:
///   - `shapePoints`, `bodyQuery` -- call `createEmptyBodyFromShapeType`
///     directly.
///   - `collisionDistanceFieldTypes` -- via its `BodyDecomposition`
///     constructor.
///   - `collisionObjectPointDecomposition` -- via upstream's
///     `getCollisionObjectPointDecomposition` -> `BodyDecomposition::init`
///     -> `bodies::BodyVector::addBody` -> `bodies::createBodyFromShape`.
///   - `distanceFieldCacheEntry`, `groupStateRepresentation` -- via
///     `CollisionEnvDistanceField`'s constructor, which unconditionally
///     calls `generateDistanceFieldCacheEntryWorld` ->
///     `updateDistanceObject` -> `getBodyDecompositionCacheEntry` for every
///     non-octree shape in the request's world objects.
/// `positionConstraintFromJson`'s constraint-region primitives reach the
/// same class of crash a different way -- `PositionConstraint::configure`
/// (`kinematic_constraint.cpp:411`) -- and are guarded separately there,
/// since `solidPrimitiveFromJson` has exactly the one call site.
void requireBodySupport(shapes::ShapeType type, const std::string& op)
{
  bodies::Body* probe = bodies::createEmptyBodyFromShapeType(type);
  if (!probe)
    throw std::runtime_error(op + ": bodies:: has no case for shape type " + shapeTypeName(type));
  delete probe;
}

/// `requireBodySupport` over every shape of every object already in `world`
/// -- for `distanceFieldCacheEntry`/`groupStateRepresentation`, whose
/// `CollisionEnvDistanceField` constructor walks the same set
/// unconditionally (`generateDistanceFieldCacheEntryWorld`) before either op
/// gets to do anything else with it.
void requireBodySupportForWorld(const collision_detection::World& world, const std::string& op)
{
  for (const auto& [id, object] : world)
  {
    (void)id;
    for (const shapes::ShapeConstPtr& shape : object->shapes_)
      requireBodySupport(shape->type, op);
  }
}

/// `requireBodySupport` over every shape of every attached body already on
/// `state` -- the same two ops `requireBodySupportForWorld` covers also
/// attach bodies (`applyAttachedBodies`, shared with the FCL-based ops that
/// legitimately want `cone`), and `CollisionEnvDistanceField`'s attached-body
/// decomposition (`getAttachedBodyPointDecomposition`/
/// `getAttachedBodySphereDecomposition`, `collision_env_distance_field.cpp:
/// 928,1239`) reaches `bodies::` the same unguarded way a world object does.
void requireBodySupportForAttachedBodies(const moveit::core::RobotState& state, const std::string& op)
{
  std::vector<const moveit::core::AttachedBody*> attached;
  state.getAttachedBodies(attached);
  for (const moveit::core::AttachedBody* body : attached)
    for (const shapes::ShapeConstPtr& shape : body->getShapes())
      requireBodySupport(shape->type, op);
}

/// `KDLKinematicsPlugin::clipToJointLimits`, transcribed directly (see
/// `Oracle::ik`'s own doc comment for why this is hand-transcribed rather
/// than vendored): per full-space DOF, clamp `q_delta[i]` so `q[i] +
/// q_delta[i]` cannot leave `[joint_min[i], joint_max[i]]`, and down-weight
/// the clipped DOF's *master* column (`weighting[mimic_joints[i].map_index]
/// = 0.01`) for the solver's next call. `weighting` is reset to all-`1.0` on
/// every call, matching upstream's own `weighting.setOnes()` as this
/// function's first statement.
void clipToJointLimits(const std::vector<double>& joint_min, const std::vector<double>& joint_max,
                       const std::vector<kdl_kinematics_plugin::JointMimic>& mimic_joints, const KDL::JntArray& q,
                       KDL::JntArray& q_delta, Eigen::ArrayXd& weighting)
{
  weighting.setOnes();
  for (std::size_t i = 0; i < q.rows(); ++i)
  {
    const double delta_max = joint_max[i] - q(i);
    const double delta_min = joint_min[i] - q(i);
    if (q_delta(i) > delta_max)
      q_delta(i) = delta_max;
    else if (q_delta(i) < delta_min)
      q_delta(i) = delta_min;
    else
      continue;
    weighting[mimic_joints[i].map_index] = 0.01;
  }
}

/// Matches AllowedCollisionType's variant names in protocol.rs.
std::string allowedCollisionTypeToString(collision_detection::AllowedCollision::Type type)
{
  switch (type)
  {
    case collision_detection::AllowedCollision::NEVER:
      return "NEVER";
    case collision_detection::AllowedCollision::ALWAYS:
      return "ALWAYS";
    case collision_detection::AllowedCollision::CONDITIONAL:
      return "CONDITIONAL";
  }
  throw std::runtime_error("unknown AllowedCollision::Type");
}

/// `moveit_msgs::msg::JointConstraint` for one `request["constraints"]
/// ["joint_constraints"]` entry -- see `protocol.rs`'s `JointConstraintSpec`.
moveit_msgs::msg::JointConstraint jointConstraintFromJson(const json& j)
{
  moveit_msgs::msg::JointConstraint jc;
  jc.joint_name = j.at("joint_name").get<std::string>();
  jc.position = j.at("position").get<double>();
  jc.tolerance_above = j.at("tolerance_above").get<double>();
  jc.tolerance_below = j.at("tolerance_below").get<double>();
  jc.weight = j.at("weight").get<double>();
  return jc;
}

/// `moveit_msgs::msg::PositionConstraint` for one `request["constraints"]
/// ["position_constraints"]` entry -- see `protocol.rs`'s
/// `PositionConstraintSpec`/`ConstraintRegionSpec`. A `"mesh"` region routes
/// to `constraint_region.meshes`/`mesh_poses` since a `SolidPrimitive` (what
/// `solidPrimitiveFromJson` builds) has no mesh case; every other shape
/// routes to `constraint_region.primitives`/`primitive_poses`.
moveit_msgs::msg::PositionConstraint positionConstraintFromJson(const json& j)
{
  moveit_msgs::msg::PositionConstraint pc;
  pc.header.frame_id = j.at("frame_id").get<std::string>();
  pc.link_name = j.at("link_name").get<std::string>();
  const auto offset = j.at("target_point_offset").get<std::array<double, 3>>();
  pc.target_point_offset.x = offset[0];
  pc.target_point_offset.y = offset[1];
  pc.target_point_offset.z = offset[2];

  for (const auto& region_json : j.at("regions"))
  {
    const json& shape_json = region_json.at("shape");
    const Eigen::Isometry3d pose = fromRowMajor4x4(region_json.at("pose"));
    if (shape_json.at("type").get<std::string>() == "mesh")
    {
      shape_msgs::msg::Mesh mesh;
      const auto vertices_json = shape_json.at("vertices");
      const auto triangles_json = shape_json.at("triangles");
      mesh.vertices.resize(vertices_json.size());
      for (std::size_t i = 0; i < vertices_json.size(); ++i)
      {
        const auto v = vertices_json[i].get<std::array<double, 3>>();
        mesh.vertices[i].x = v[0];
        mesh.vertices[i].y = v[1];
        mesh.vertices[i].z = v[2];
      }
      mesh.triangles.resize(triangles_json.size());
      for (std::size_t i = 0; i < triangles_json.size(); ++i)
      {
        const auto t = triangles_json[i].get<std::array<std::uint32_t, 3>>();
        mesh.triangles[i].vertex_indices = { t[0], t[1], t[2] };
      }
      pc.constraint_region.meshes.push_back(mesh);
      pc.constraint_region.mesh_poses.push_back(isometryToPoseMsg(pose));
    }
    else
    {
      // `solidPrimitiveFromJson`'s own doc: a `"cone"` region reaches
      // `PositionConstraint::configure`'s unchecked `bodies::` dereference --
      // an upstream crash, not a comparable answer -- so it is rejected here
      // rather than built and handed to it.
      if (shape_json.at("type").get<std::string>() == "cone")
        throw std::runtime_error("position_constraints: cone region would crash upstream's "
                                  "PositionConstraint::configure (bodies:: has no CONE case) -- "
                                  "not a comparable request");
      pc.constraint_region.primitives.push_back(solidPrimitiveFromJson(shape_json));
      pc.constraint_region.primitive_poses.push_back(isometryToPoseMsg(pose));
    }
  }

  pc.weight = j.at("weight").get<double>();
  return pc;
}

/// `moveit_msgs::msg::OrientationConstraint` for one `request["constraints"]
/// ["orientation_constraints"]` entry -- see `protocol.rs`'s
/// `OrientationConstraintSpec`/`OrientationToleranceSpec`.
moveit_msgs::msg::OrientationConstraint orientationConstraintFromJson(const json& j)
{
  moveit_msgs::msg::OrientationConstraint oc;
  oc.header.frame_id = j.at("frame_id").get<std::string>();
  oc.link_name = j.at("link_name").get<std::string>();

  const auto q = j.at("orientation").get<std::array<double, 4>>();
  oc.orientation.x = q[0];
  oc.orientation.y = q[1];
  oc.orientation.z = q[2];
  oc.orientation.w = q[3];

  const json& tol = j.at("tolerance");
  const std::string parameterization = tol.at("parameterization").get<std::string>();
  if (parameterization == "xyz_euler")
    oc.parameterization = moveit_msgs::msg::OrientationConstraint::XYZ_EULER_ANGLES;
  else if (parameterization == "rotation_vector")
    oc.parameterization = moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR;
  else
    throw std::runtime_error("constraints: unknown orientation parameterization " + parameterization);
  oc.absolute_x_axis_tolerance = tol.at("x").get<double>();
  oc.absolute_y_axis_tolerance = tol.at("y").get<double>();
  oc.absolute_z_axis_tolerance = tol.at("z").get<double>();

  oc.weight = j.at("weight").get<double>();
  return oc;
}

/// `moveit_msgs::msg::VisibilityConstraint` for one `request["constraints"]
/// ["visibility_constraints"]` entry -- see `protocol.rs`'s
/// `VisibilityConstraintSpec`. `target_radius`/`max_view_angle`/
/// `max_range_angle` default to `0.0` (upstream's own "unconstrained"
/// sentinel) when the JSON field is absent, matching the `Option<f64>` ->
/// omitted-when-`None` encoding `protocol.rs` uses for all three.
moveit_msgs::msg::VisibilityConstraint visibilityConstraintFromJson(const json& j)
{
  moveit_msgs::msg::VisibilityConstraint vc;

  vc.sensor_pose.header.frame_id = j.at("sensor_frame_id").get<std::string>();
  vc.sensor_pose.pose = isometryToPoseMsg(fromRowMajor4x4(j.at("sensor_pose")));

  const std::string direction = j.at("sensor_view_direction").get<std::string>();
  if (direction == "sensor_x")
    vc.sensor_view_direction = moveit_msgs::msg::VisibilityConstraint::SENSOR_X;
  else if (direction == "sensor_y")
    vc.sensor_view_direction = moveit_msgs::msg::VisibilityConstraint::SENSOR_Y;
  else if (direction == "sensor_z")
    vc.sensor_view_direction = moveit_msgs::msg::VisibilityConstraint::SENSOR_Z;
  else
    throw std::runtime_error("constraints: unknown sensor_view_direction " + direction);

  vc.target_pose.header.frame_id = j.at("target_frame_id").get<std::string>();
  vc.target_pose.pose = isometryToPoseMsg(fromRowMajor4x4(j.at("target_pose")));

  vc.cone_sides = j.at("cone_sides").get<std::int32_t>();
  vc.target_radius = j.value("target_radius", 0.0);
  vc.max_view_angle = j.value("max_view_angle", 0.0);
  vc.max_range_angle = j.value("max_range_angle", 0.0);
  vc.weight = j.at("weight").get<double>();
  return vc;
}

/// Builds a `moveit_msgs::msg::Constraints` from a JSON object shaped like
/// `Op::Constraints`'s own `constraints` field (`protocol.rs`'s
/// `ConstraintsSpec`) -- kept as its own free function rather than sharing
/// `Oracle::constraints`'s inline block, since that op belongs to a
/// different owner. Used by `isStateValid` for `path_constraints` and each
/// entry of `goal_constraints`.
moveit_msgs::msg::Constraints constraintsMsgFromJson(const json& spec)
{
  moveit_msgs::msg::Constraints msg;
  if (spec.contains("joint_constraints"))
    for (const auto& jc_json : spec.at("joint_constraints"))
      msg.joint_constraints.push_back(jointConstraintFromJson(jc_json));
  if (spec.contains("position_constraints"))
    for (const auto& pc_json : spec.at("position_constraints"))
      msg.position_constraints.push_back(positionConstraintFromJson(pc_json));
  if (spec.contains("orientation_constraints"))
    for (const auto& oc_json : spec.at("orientation_constraints"))
      msg.orientation_constraints.push_back(orientationConstraintFromJson(oc_json));
  if (spec.contains("visibility_constraints"))
    for (const auto& vc_json : spec.at("visibility_constraints"))
      msg.visibility_constraints.push_back(visibilityConstraintFromJson(vc_json));
  return msg;
}

namespace ob = ompl::base;
namespace og = ompl::geometric;

/// `UNBOUNDED_TRANSLATION_HALF_EXTENT` in `cspace-planners-sbp`'s
/// `joint_model_group_space.rs`, restated here because the two sides have to
/// substitute the *same* half-extent for a non-finite translation bound or
/// their spaces are not the same space. See that constant's own doc comment
/// for why a substitution is needed at all.
constexpr double PLAN_UNBOUNDED_TRANSLATION_HALF_EXTENT = 10.0;

/// `bounded_axis` in the same file: the joint's own bounds when both are
/// finite, the fixed half-extent either side of zero otherwise.
std::pair<double, double> planBoundedAxis(const moveit::core::VariableBounds& bounds)
{
  if (std::isfinite(bounds.min_position_) && std::isfinite(bounds.max_position_))
    return { bounds.min_position_, bounds.max_position_ };
  return { -PLAN_UNBOUNDED_TRANSLATION_HALF_EXTENT, PLAN_UNBOUNDED_TRANSLATION_HALF_EXTENT };
}

/// Where one OMPL subspace of the `plan` op's state space reads and writes in
/// a `RobotState`'s flat variable array -- `SubspaceSlot` in
/// `joint_model_group_space.rs`, mirrored one for one including that type's
/// split of a planar joint into three independent subspaces.
struct PlanSubspaceSlot
{
  enum class Kind
  {
    RealVector,  ///< one variable
    So2,         ///< one variable
    Se3,         ///< seven: trans x/y/z then rot x/y/z/w, `FloatingJointModel` order
  };
  Kind kind;
  std::array<int, 7> variables{};
};

/// The distance `ompl::base::SO3StateSpace` reports for a rotation of known
/// angle `PI/2` -- measured, not assumed.
///
/// `cspace-planners-sbp`'s `Se3Space::distance` weights the *full* rotation
/// angle (its `rotation_distance` is an atan2 chord form of `2*acos(|dot|)`;
/// see that function's own comment for the halved-angle bug that convention
/// once caused there). An SO3 subspace mirroring it therefore needs weight
/// `angular_distance_weight * (PI/2) / so3DistanceOfQuarterTurn()`. OMPL's
/// header declares `distance` without documenting its formula and this image
/// ships no OMPL `.cpp`, so the factor is read off the linked library at run
/// time and reported back in the response (`space.so3_quarter_turn_distance`)
/// rather than hard-coded from a reading of source that is not present here.
double so3DistanceOfQuarterTurn()
{
  auto so3 = std::make_shared<ob::SO3StateSpace>();
  ob::State* a = so3->allocState();
  ob::State* b = so3->allocState();
  a->as<ob::SO3StateSpace::StateType>()->setIdentity();
  auto* qb = b->as<ob::SO3StateSpace::StateType>();
  qb->x = 0.0;
  qb->y = 0.0;
  qb->z = std::sin(M_PI / 4.0);
  qb->w = std::cos(M_PI / 4.0);
  const double d = so3->distance(a, b);
  so3->freeState(a);
  so3->freeState(b);
  return d;
}

/// What `main` passed to `--planner-rng-seed`, or unset when it passed
/// nothing.
///
/// File scope rather than an `Oracle` member because the seeding it records
/// has to happen before an `Oracle` (and so before any `RobotModel`) exists --
/// see the seeding block in `main`. `chomp_plan` and `stomp_plan` echo it so a
/// result file says which stream produced it instead of leaving that to the
/// command line that is no longer around.
std::optional<std::uint32_t> g_planner_rng_seed;

/// `g_planner_rng_seed` as JSON: the number, or null when the process was left
/// on `rsl::rng`'s own `std::random_device` seeding.
json plannerRngSeedJson()
{
  return g_planner_rng_seed ? json(*g_planner_rng_seed) : json(nullptr);
}

/// The OMPL mirror of one `JointModelGroupSpace`, plus the per-subspace
/// description echoed back so the Rust side can assert the two constructions
/// agree without having to trust this one.
struct PlanSpace
{
  ob::StateSpacePtr space;
  std::vector<PlanSubspaceSlot> layout;
  json description;
  double so3_quarter_turn_distance = 0.0;
};

/// Builds `JointModelGroupSpace::new(model, group->getName())`'s OMPL twin.
///
/// Dispatch and weights follow that constructor exactly: bounded revolute and
/// prismatic to a one-axis `RealVectorStateSpace` weighted `1/(max-min)`,
/// continuous revolute to `SO2StateSpace` weighted `1/(2*PI)`, planar to
/// `x`/`y` real axes plus a `theta` SO2 weighted
/// `angular_distance_weight/(2*PI)`, floating to a
/// `RealVectorStateSpace(3) + SO3StateSpace` compound weighted `1/extent`
/// with `extent = |translation ranges| + PI/2 * angular_distance_weight`.
/// A fixed or mimic joint contributes nothing on either side --
/// `getActiveJointModels()` already excludes both.
PlanSpace buildPlanSpace(const moveit::core::JointModelGroup* group)
{
  PlanSpace out;
  out.so3_quarter_turn_distance = so3DistanceOfQuarterTurn();
  out.description = json::array();
  auto compound = std::make_shared<ob::CompoundStateSpace>();

  const auto add_real_axis = [&](const std::string& joint_name, const moveit::core::VariableBounds& bounds,
                                 int variable) {
    const auto [min, max] = planBoundedAxis(bounds);
    auto axis = std::make_shared<ob::RealVectorStateSpace>(1);
    ob::RealVectorBounds axis_bounds(1);
    axis_bounds.setLow(0, min);
    axis_bounds.setHigh(0, max);
    axis->setBounds(axis_bounds);
    const double weight = 1.0 / (max - min);
    compound->addSubspace(axis, weight);
    out.layout.push_back({ PlanSubspaceSlot::Kind::RealVector, { variable } });
    out.description.push_back(json{ { "joint", joint_name },
                                    { "type", "real_vector" },
                                    { "weight", weight },
                                    { "bounds", json::array({ min, max }) } });
  };

  const auto add_so2 = [&](const std::string& joint_name, double weight, int variable) {
    compound->addSubspace(std::make_shared<ob::SO2StateSpace>(), weight);
    out.layout.push_back({ PlanSubspaceSlot::Kind::So2, { variable } });
    out.description.push_back(json{ { "joint", joint_name }, { "type", "so2" }, { "weight", weight } });
  };

  for (const moveit::core::JointModel* joint : group->getActiveJointModels())
  {
    const int first = joint->getFirstVariableIndex();
    const std::vector<moveit::core::VariableBounds>& bounds = joint->getVariableBounds();

    switch (joint->getType())
    {
      case moveit::core::JointModel::REVOLUTE:
        if (static_cast<const moveit::core::RevoluteJointModel*>(joint)->isContinuous())
          add_so2(joint->getName(), 1.0 / (2.0 * M_PI), first);
        else
          add_real_axis(joint->getName(), bounds[0], first);
        break;

      case moveit::core::JointModel::PRISMATIC:
        add_real_axis(joint->getName(), bounds[0], first);
        break;

      case moveit::core::JointModel::PLANAR:
      {
        add_real_axis(joint->getName(), bounds[0], first);
        add_real_axis(joint->getName(), bounds[1], first + 1);
        const double angular_weight =
            static_cast<const moveit::core::PlanarJointModel*>(joint)->getAngularDistanceWeight();
        add_so2(joint->getName(), angular_weight / (2.0 * M_PI), first + 2);
        break;
      }

      case moveit::core::JointModel::FLOATING:
      {
        const double angular_weight =
            static_cast<const moveit::core::FloatingJointModel*>(joint)->getAngularDistanceWeight();

        auto translation = std::make_shared<ob::RealVectorStateSpace>(3);
        ob::RealVectorBounds translation_bounds(3);
        double squared_extent = 0.0;
        for (int axis = 0; axis < 3; ++axis)
        {
          const auto [min, max] = planBoundedAxis(bounds[axis]);
          translation_bounds.setLow(axis, min);
          translation_bounds.setHigh(axis, max);
          squared_extent += (max - min) * (max - min);
        }
        translation->setBounds(translation_bounds);
        const double extent = std::sqrt(squared_extent) + M_PI * 0.5 * angular_weight;

        // Not `ob::SE3StateSpace`: that type fixes its own rotation weight at
        // 1.0 against OMPL's SO3 distance convention, which is not the
        // full-angle convention `Se3Space::distance` uses. Composing the two
        // subspaces here is what lets the rotation weight be the calibrated
        // `angular_distance_weight` the Rust side actually applies.
        auto se3 = std::make_shared<ob::CompoundStateSpace>();
        const double rotation_weight = angular_weight * (M_PI / 2.0) / out.so3_quarter_turn_distance;
        se3->addSubspace(translation, 1.0);
        se3->addSubspace(std::make_shared<ob::SO3StateSpace>(), rotation_weight);

        compound->addSubspace(se3, 1.0 / extent);
        out.layout.push_back({ PlanSubspaceSlot::Kind::Se3,
                               { first, first + 1, first + 2, first + 3, first + 4, first + 5, first + 6 } });
        out.description.push_back(json{ { "joint", joint->getName() },
                                        { "type", "se3" },
                                        { "weight", 1.0 / extent },
                                        { "rotation_weight", rotation_weight },
                                        { "angular_distance_weight", angular_weight } });
        break;
      }

      default:
        throw std::runtime_error("plan: joint " + joint->getName() + " has a type this op does not build a space for");
    }
  }

  out.space = compound;
  return out;
}

/// Writes an OMPL state into `robot_state`'s variables, then `update()`s it.
///
/// `setVariablePosition(index, value)` rather than a write through
/// `getVariablePositions()`: the latter hands out a raw `double*` and leaves
/// the dirty-transform bookkeeping to the caller, so a subsequent `update()`
/// can legitimately decide there is nothing to recompute.
void planStateToRobotState(const ob::State* state, const std::vector<PlanSubspaceSlot>& layout,
                           moveit::core::RobotState& robot_state)
{
  const auto* compound = state->as<ob::CompoundState>();
  for (std::size_t i = 0; i < layout.size(); ++i)
  {
    const PlanSubspaceSlot& slot = layout[i];
    switch (slot.kind)
    {
      case PlanSubspaceSlot::Kind::RealVector:
        robot_state.setVariablePosition(slot.variables[0],
                                        compound->as<ob::RealVectorStateSpace::StateType>(i)->values[0]);
        break;
      case PlanSubspaceSlot::Kind::So2:
        robot_state.setVariablePosition(slot.variables[0], compound->as<ob::SO2StateSpace::StateType>(i)->value);
        break;
      case PlanSubspaceSlot::Kind::Se3:
      {
        const auto* se3 = compound->as<ob::CompoundState>(i);
        const auto* translation = se3->as<ob::RealVectorStateSpace::StateType>(0);
        const auto* rotation = se3->as<ob::SO3StateSpace::StateType>(1);
        for (int axis = 0; axis < 3; ++axis)
          robot_state.setVariablePosition(slot.variables[axis], translation->values[axis]);
        robot_state.setVariablePosition(slot.variables[3], rotation->x);
        robot_state.setVariablePosition(slot.variables[4], rotation->y);
        robot_state.setVariablePosition(slot.variables[5], rotation->z);
        robot_state.setVariablePosition(slot.variables[6], rotation->w);
        break;
      }
    }
  }
  robot_state.update();
}

/// The inverse of `planStateToRobotState`, for start and goal states.
void robotStateToPlanState(const moveit::core::RobotState& robot_state,
                           const std::vector<PlanSubspaceSlot>& layout, ob::State* state)
{
  auto* compound = state->as<ob::CompoundState>();
  for (std::size_t i = 0; i < layout.size(); ++i)
  {
    const PlanSubspaceSlot& slot = layout[i];
    switch (slot.kind)
    {
      case PlanSubspaceSlot::Kind::RealVector:
        compound->as<ob::RealVectorStateSpace::StateType>(i)->values[0] =
            robot_state.getVariablePosition(slot.variables[0]);
        break;
      case PlanSubspaceSlot::Kind::So2:
        compound->as<ob::SO2StateSpace::StateType>(i)->value = robot_state.getVariablePosition(slot.variables[0]);
        break;
      case PlanSubspaceSlot::Kind::Se3:
      {
        auto* se3 = compound->as<ob::CompoundState>(i);
        auto* translation = se3->as<ob::RealVectorStateSpace::StateType>(0);
        auto* rotation = se3->as<ob::SO3StateSpace::StateType>(1);
        for (int axis = 0; axis < 3; ++axis)
          translation->values[axis] = robot_state.getVariablePosition(slot.variables[axis]);
        rotation->x = robot_state.getVariablePosition(slot.variables[3]);
        rotation->y = robot_state.getVariablePosition(slot.variables[4]);
        rotation->z = robot_state.getVariablePosition(slot.variables[5]);
        rotation->w = robot_state.getVariablePosition(slot.variables[6]);
        break;
      }
    }
  }
}

/// `PlanningSceneValidityChecker` in `cspace-planners-sbp`: writes the
/// candidate into the scene's own current state and asks the scene.
///
/// `group` is `""` (whole robot), matching the `is_state_valid` op's own
/// choice and for the same reason -- this port's `ParryCollisionEnv` never
/// reads `CollisionRequest::group_name`, so narrowing here would make the two
/// sides check different geometry for a reason unrelated to planning.
class PlanValidityChecker : public ob::StateValidityChecker
{
public:
  PlanValidityChecker(const ob::SpaceInformationPtr& si, planning_scene::PlanningScene& scene,
                      const std::vector<PlanSubspaceSlot>& layout)
    : ob::StateValidityChecker(si), scene_(scene), layout_(layout)
  {
  }

  bool isValid(const ob::State* state) const override
  {
    moveit::core::RobotState& robot_state = scene_.getCurrentStateNonConst();
    planStateToRobotState(state, layout_, robot_state);
    return scene_.isStateValid(robot_state, "", false);
  }

private:
  planning_scene::PlanningScene& scene_;
  const std::vector<PlanSubspaceSlot>& layout_;
};

class Oracle
{
public:
  Oracle(const std::string& urdf_path, const std::string& srdf_path, std::uint32_t ik_rng_seed = 42)
    : ik_rng_(ik_rng_seed)
  {
    const std::string urdf_xml = readFile(urdf_path);
    const std::string srdf_xml = readFile(srdf_path);

    urdf_model_ = urdf::parseURDF(urdf_xml);
    if (!urdf_model_)
      throw std::runtime_error("failed to parse URDF at " + urdf_path);

    srdf_model_ = std::make_shared<srdf::Model>();
    if (!srdf_model_->initString(*urdf_model_, srdf_xml))
      throw std::runtime_error("failed to parse SRDF at " + srdf_path);

    model_ = std::make_shared<moveit::core::RobotModel>(urdf_model_, srdf_model_);
    state_ = std::make_unique<moveit::core::RobotState>(model_);
    state_->setToDefaultValues();
    state_->update();

    // For the `ik` op: one KDL::Tree built once, `getChain`-sliced per
    // request into whatever group's own base/tip the request names.
    if (!kdl_parser::treeFromUrdfModel(*urdf_model_, kdl_tree_))
      throw std::runtime_error("failed to build a KDL::Tree from the URDF at " + urdf_path);
  }

  /// Applies `limits` to `joint_model` for the rest of the current request.
  ///
  /// `JointModel::setVariableBounds` writes through to `model_`, which this
  /// process builds once and every later request reads. Three ops apply a
  /// case's limits that way -- `totgRobotTrajectoryCase`,
  /// `accelerationFilterCase`, `setJointAccelerationVelocityJerkBounds` --
  /// mirroring the read/mutate/write pattern `joint_limits.yaml` loaders use
  /// upstream. The write used to simply stay: a `totg` case carrying
  /// "acceleration_bounds" changed what a later `ruckig` request in the same
  /// process computed, because `ruckig`'s RobotModel-bounds overload reads
  /// exactly the field that was overwritten.
  ///
  /// Nothing downstream could see that. A fixture is captured and replayed one
  /// per process, so each committed response stays self-consistent no matter
  /// what leaks; what a leak corrupts is a run that puts several ops in one
  /// process -- `moveit-diff`, and fixture capture if it is ever batched. It
  /// was found by the combined pass in `tools/ci/verify-fixture-replay.sh`,
  /// which concatenates every fixture sharing a robot model into one process
  /// for exactly this reason.
  ///
  /// Recording the replaced bounds here rather than restoring at each call
  /// site is what makes the scoping hold when an op throws part way through
  /// applying a case's limits ("unknown joint" is reachable mid-loop), and
  /// what makes a later op inherit it without knowing this exists.
  void applyJointBounds(moveit::core::JointModel* joint_model,
                        const std::vector<moveit_msgs::msg::JointLimits>& limits)
  {
    replaced_bounds_.emplace_back(joint_model, joint_model->getVariableBoundsMsg());
    joint_model->setVariableBounds(limits);
  }

  /// Puts back every bound `applyJointBounds` replaced during this request.
  ///
  /// Most recent first, because a request may override the same joint twice
  /// and it is the first entry that holds the bounds the request started from.
  void restoreJointBounds()
  {
    for (auto it = replaced_bounds_.rbegin(); it != replaced_bounds_.rend(); ++it)
      it->first->setVariableBounds(it->second);
    replaced_bounds_.clear();
  }

  /// Ends every joint-bound override with the request that made it.
  ///
  /// Held by `handle`, the single point every op is dispatched from, so the
  /// scoping covers the throwing paths too -- `handle`'s caller turns an
  /// exception into an `ok: false` response and reads the next request from
  /// the same stdin, on the same `model_`.
  struct ScopedJointBounds
  {
    Oracle& oracle;
    ~ScopedJointBounds()
    {
      oracle.restoreJointBounds();
    }
  };

  json handle(const json& request)
  {
    ScopedJointBounds scoped_bounds{ *this };
    const std::string op = request.at("op").get<std::string>();
    if (op == "model_info")
      return modelInfo();
    if (op == "fk")
      return fk(request);
    if (op == "jacobian")
      return jacobian(request);
    if (op == "random_states")
      return randomStates(request);
    if (op == "enforce_bounds")
      return enforceBoundsOp(request);
    if (op == "mimic_propagate")
      return mimicPropagateOp(request);
    if (op == "interpolate")
      return interpolateOp(request);
    if (op == "state_interpolate")
      return stateInterpolateOp(request);
    if (op == "kinematics_metrics")
      return kinematicsMetrics(request);
    if (op == "acm")
      return acm();
    if (op == "collision")
      return collision(request);
    if (op == "pair_signed_distance")
      return pairSignedDistance(request);
    if (op == "world")
      return world(request);
    if (op == "frame_transform")
      return frameTransform(request);
    if (op == "is_state_valid")
      return isStateValid(request);
    if (op == "scene_diff_collision")
      return sceneDiffCollision(request);
    if (op == "cost_sources")
      return costSources(request);
    if (op == "path_cost_sources")
      return pathCostSources(request);
    if (op == "octree_points")
      return octreePoints(request);
    if (op == "distance_field")
      return distanceField(request);
    if (op == "shape_points")
      return shapePoints(request);
    if (op == "mesh")
      return meshOp(request);
    if (op == "common_root")
      return commonRoot(request);
    if (op == "collision_distance_field_types")
      return collisionDistanceFieldTypes(request);
    if (op == "collision_sphere_free_functions")
      return collisionSphereFreeFunctions(request);
    if (op == "distance_field_cache_entry")
      return distanceFieldCacheEntry(request);
    if (op == "group_state_representation")
      return groupStateRepresentation(request);
    if (op == "dynamics")
      return dynamics(request);
    if (op == "collision_object_point_decomposition")
      return collisionObjectPointDecomposition(request);
    if (op == "link_body_decomposition")
      return linkBodyDecomposition(request);
    if (op == "link_models_with_collision_geometry")
      return linkModelsWithCollisionGeometry();
    if (op == "constraints")
      return constraints(request);
    if (op == "octomap")
      return octomapOp(request);
    if (op == "ik")
      return ik(request);
    if (op == "octree_in_world")
      return octreeInWorld(request);
    if (op == "octree_shape_query")
      return octreeShapeQuery(request);
    if (op == "ruckig")
      return ruckig(request);
    if (op == "body_query")
      return bodyQuery(request);
    if (op == "totg")
      return totg(request);
    if (op == "totg_path")
      return totgPath(request);
    if (op == "acceleration_filter")
      return accelerationFilter(request);
    if (op == "plan")
      return plan(request);
    if (op == "ruckig_filter")
      return ruckigFilter(request);
    if (op == "pilz_trajectory")
      return pilzTrajectory(request);
    if (op == "pilz_blend")
      return pilzBlend(request);
    if (op == "chomp_quad_cost_inverse")
      return chompQuadCostInverse(request);
    if (op == "chomp_plan")
      return chompPlan(request);
    if (op == "stomp_plan")
      return stompPlan(request);
    throw std::runtime_error("unsupported op: " + op);
  }

private:
  json modelInfo() const
  {
    json out;
    out["name"] = model_->getName();
    out["model_frame"] = model_->getModelFrame();
    out["root_link"] = model_->getRootLinkName();
    out["links"] = model_->getLinkModelNames();
    out["joints"] = model_->getJointModelNames();

    json details = json::array();
    for (const moveit::core::JointModel* joint : model_->getJointModels())
    {
      json d;
      d["name"] = joint->getName();
      d["type_name"] = joint->getTypeName();
      d["variable_names"] = joint->getVariableNames();

      json bounds = json::array();
      json bounded = json::array();
      for (const moveit::core::VariableBounds& b : joint->getVariableBounds())
      {
        // JSON cannot carry an infinity, and a floating joint's translation
        // bounds are infinite while still reporting position_bounded_ true.
        // Emitting null for a non-finite limit says "unbounded" explicitly
        // instead of letting nlohmann turn it into an untyped null downstream.
        bounds.push_back(json::array({ finiteOrNull(b.min_position_), finiteOrNull(b.max_position_) }));
        bounded.push_back(b.position_bounded_);
      }
      d["bounds"] = bounds;
      d["position_bounded"] = bounded;

      if (const moveit::core::JointModel* mimic = joint->getMimic())
      {
        d["mimic"] = json{ { "joint", mimic->getName() },
                           { "multiplier", joint->getMimicFactor() },
                           { "offset", joint->getMimicOffset() } };
      }
      details.push_back(d);
    }
    out["joint_details"] = details;

    json groups = json::object();
    for (const moveit::core::JointModelGroup* group : model_->getJointModelGroups())
      groups[group->getName()] = group->getJointModelNames();
    out["groups"] = groups;

    out["link_details"] = linkDetails();
    out["group_end_effectors"] = groupEndEffectors();
    out["group_states"] = groupStates();
    out["group_is_chain"] = groupIsChain();
    out["group_joint_roots"] = groupJointRoots();
    out["joint_descendant_links"] = jointDescendantLinks();
    out["group_updated_links"] = groupUpdatedLinks();

    return out;
  }

  /// Ground truth for `cspace-model`'s `LinkModel` collision/visual geometry
  /// (`LinkModel`'s doc comment, deviation 4, in the Rust port): one entry per
  /// link, every `<collision>` element's shape kind in file order (this
  /// oracle's own naming, see `shapeTypeName`), the centered bounding-box
  /// offset (`null` per component when non-finite -- a link with no
  /// collision geometry has `NaN` here, same as the Rust port, since upstream
  /// `LinkModel::setGeometry` calls `aabb.center()` unconditionally), and the
  /// visual mesh metadata (present only when there is one, i.e.
  /// `getVisualMeshFilename()` is non-empty).
  json linkDetails() const
  {
    json out = json::array();
    for (const moveit::core::LinkModel* link : model_->getLinkModels())
    {
      json l;
      l["name"] = link->getName();

      json shape_types = json::array();
      for (const shapes::ShapeConstPtr& shape : link->getShapes())
        shape_types.push_back(shapeTypeName(shape->type));
      l["shape_types"] = shape_types;

      const Eigen::Vector3d& offset = link->getCenteredBoundingBoxOffset();
      l["centered_bounding_box_offset"] =
          json::array({ finiteOrNull(offset.x()), finiteOrNull(offset.y()), finiteOrNull(offset.z()) });

      const std::string& mesh_filename = link->getVisualMeshFilename();
      if (mesh_filename.empty())
      {
        l["visual_mesh_filename"] = nullptr;
      }
      else
      {
        l["visual_mesh_filename"] = mesh_filename;
        l["visual_mesh_origin"] = toRowMajor4x4(link->getVisualMeshOrigin());
        const Eigen::Vector3d& scale = link->getVisualMeshScale();
        l["visual_mesh_scale"] = json::array({ scale.x(), scale.y(), scale.z() });
      }

      out.push_back(l);
    }
    return out;
  }

  /// Ground truth for `cspace-model`'s `JointModelGroup` end-effector fields
  /// (`is_end_effector`/`end_effector_name`/`end_effector_parent`/
  /// `attached_end_effector_names`), keyed by group name. `end_effector_name`
  /// is `null` for a group that is not an end effector
  /// (`!group->isEndEffector()`); `end_effector_parent` is `null` under the
  /// same condition -- `getEndEffectorParentGroup()` returns a `("", "")`
  /// pair upstream never assigns to a non-end-effector group, which this
  /// tells apart from the WARN case of a resolved end effector with no
  /// identifiable parent GROUP (empty `group`, non-empty `link`, since
  /// `parent_link_` always comes from a required SRDF attribute).
  json groupEndEffectors() const
  {
    json out = json::object();
    for (const moveit::core::JointModelGroup* group : model_->getJointModelGroups())
    {
      json g;
      g["end_effector_name"] = group->isEndEffector() ? json(group->getEndEffectorName()) : json(nullptr);
      g["attached_end_effector_names"] = group->getAttachedEndEffectorNames();

      const std::pair<std::string, std::string>& parent = group->getEndEffectorParentGroup();
      if (parent.first.empty() && parent.second.empty())
      {
        g["end_effector_parent"] = nullptr;
      }
      else
      {
        g["end_effector_parent"] = json{ { "group", parent.first.empty() ? json(nullptr) : json(parent.first) },
                                          { "link", parent.second } };
      }
      out[group->getName()] = g;
    }
    return out;
  }

  json groupStates() const
  {
    json out = json::object();
    for (const moveit::core::JointModelGroup* group : model_->getJointModelGroups())
    {
      json states = json::object();
      for (const std::string& name : group->getDefaultStateNames())
      {
        std::map<std::string, double> values;
        if (group->getVariableDefaultPositions(name, values))
        {
          json vals = json::object();
          for (const auto& kv : values)
            vals[kv.first] = kv.second;
          states[name] = vals;
        }
      }
      if (!states.empty())
        out[group->getName()] = states;
    }
    return out;
  }

  /// Ground truth for `cspace-model`'s `JointModelGroup::is_chain`, keyed by
  /// group name.
  json groupIsChain() const
  {
    json out = json::object();
    for (const moveit::core::JointModelGroup* group : model_->getJointModelGroups())
      out[group->getName()] = group->isChain();
    return out;
  }

  /// Ground truth for `cspace-model`'s `JointModelGroup::joint_roots`, keyed
  /// by group name: the names of every joint in `joint_roots_`.
  json groupJointRoots() const
  {
    json out = json::object();
    for (const moveit::core::JointModelGroup* group : model_->getJointModelGroups())
    {
      json roots = json::array();
      for (const moveit::core::JointModel* joint : group->getJointRoots())
        roots.push_back(joint->getName());
      out[group->getName()] = roots;
    }
    return out;
  }

  /// Ground truth for `cspace-model`'s `RobotModel::descendant_link_indices`
  /// (upstream `JointModel::getDescendantLinkModels`), keyed by joint name:
  /// the names of every descendant link. Compared as a set on the Rust side
  /// -- upstream's own vector is DFS-insertion-ordered, not index-ordered,
  /// and nothing downstream (`updated_link_model_*`, itself a re-sorted
  /// `std::set` union) depends on that order surviving.
  json jointDescendantLinks() const
  {
    json out = json::object();
    for (const moveit::core::JointModel* joint : model_->getJointModels())
    {
      json links = json::array();
      for (const moveit::core::LinkModel* link : joint->getDescendantLinkModels())
        links.push_back(link->getName());
      out[joint->getName()] = links;
    }
    return out;
  }

  /// Ground truth for `cspace-model`'s `JointModelGroup::updated_link_names`/
  /// `updated_link_with_geometry_names`, keyed by group name.
  json groupUpdatedLinks() const
  {
    json out = json::object();
    for (const moveit::core::JointModelGroup* group : model_->getJointModelGroups())
    {
      out[group->getName()] = json{
        { "updated_link_names", group->getUpdatedLinkModelNames() },
        { "updated_link_with_geometry_names", group->getUpdatedLinkModelsWithGeometryNames() },
      };
    }
    return out;
  }

  /// Ground truth for `cspace-model`'s `RobotModel::get_common_root`:
  /// `{"pairs": [["joint_a", "joint_b"], ...]}` -> the named common-root
  /// joint for each pair, in request order.
  json commonRoot(const json& request) const
  {
    json out = json::array();
    for (const json& pair : request.at("pairs"))
    {
      const std::string a_name = pair.at(0).get<std::string>();
      const std::string b_name = pair.at(1).get<std::string>();
      if (!model_->hasJointModel(a_name))
        throw std::runtime_error("unknown joint: " + a_name);
      if (!model_->hasJointModel(b_name))
        throw std::runtime_error("unknown joint: " + b_name);
      const moveit::core::JointModel* a = model_->getJointModel(a_name);
      const moveit::core::JointModel* b = model_->getJointModel(b_name);
      const moveit::core::JointModel* common = model_->getCommonRoot(a, b);
      out.push_back(json{ { "a", a_name },
                           { "b", b_name },
                           { "common_root", common ? json(common->getName()) : json(nullptr) } });
    }
    return out;
  }

  /// Ground truth for `dynamics_solver::DynamicsSolver`, MoveIt's KDL-backed
  /// recursive Newton-Euler inverse dynamics wrapper. `ChainIdSolver_RNE`'s
  /// own `.cpp` is not present anywhere local to this repo -- only its
  /// compiled `.so` and the header declaration -- so there is nothing to
  /// port against yet; this endpoint drives the real, compiled solver
  /// directly, so the numbers are authoritative ground truth regardless of
  /// where a future Rust port's own implementation comes from.
  ///
  /// `group` must be a chain with no mimic joint and a joint root with a
  /// parent link -- the same three preconditions `DynamicsSolver`'s own
  /// constructor checks, logging and leaving `getGroup() == nullptr` on
  /// failure rather than throwing. Reproduced here so a request that fails
  /// one throws a specific error instead of this method dereferencing a
  /// null group three lines later.
  ///
  /// `joint_values`/`joint_velocities`/`joint_accelerations` are keyed by
  /// joint name (not variable name): every joint `DynamicsSolver` accepts is
  /// 1-DOF by construction (`getActiveJointModelNames()`, the same order
  /// `getTorques`'s own doc comment specifies), so a joint name is
  /// unambiguous here the way it would not be for a planar or floating
  /// joint. External wrenches are always zero: nothing in this crate's
  /// verification plan needs them, and `getMaxPayload`/`getPayloadTorques`
  /// exercise the one non-zero wrench upstream ever applies (a payload at
  /// the tip) internally.
  ///
  /// `max_payload.payload` can come back JSON `null` for a zero-gravity
  /// request: `getMaxPayload` divides by `gravity_` unconditionally, and
  /// nlohmann's `dump()` already turns the resulting non-finite double into
  /// `null` the same way `finiteOrNull` does elsewhere in this file -- this
  /// is upstream's own 0/0 for that input, not a bug this wrapper
  /// introduces, so it is captured rather than special-cased away.
  ///
  /// `max_torques` is `solver.getMaxTorques()` verbatim, indexed by
  /// `getJointModelNames()` (fixed joints included, appended as `0.0`), NOT
  /// by `joint_names`/`torques`/`payload_torques`'s active-only index space
  /// -- upstream's own constructor builds it over the former but
  /// `getMaxPayload`'s saturation check reads it with the latter's indices.
  /// For a chain with a fixed joint strictly before its last active joint
  /// (pr2's `right_arm`, `r_upper_arm_joint` before `r_elbow_flex_joint`)
  /// this makes `getMaxPayload` compare one joint's real gravity torque
  /// against a *different* (fixed, always-`0.0`-limit) joint's slot, which
  /// is why this endpoint's captured `pr2_dynamics.json` payload is always
  /// `0.0` -- a real upstream defect this ground truth preserves as-is
  /// rather than working around, since a future port needs to know it is
  /// there to decide whether to replicate or deliberately diverge from it.
  json dynamics(const json& request) const
  {
    const std::string group_name = request.at("group").get<std::string>();
    const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_name);
    if (!group)
      throw std::runtime_error("unknown group: " + group_name);
    if (!group->isChain())
      throw std::runtime_error("group '" + group_name + "' is not a chain");
    if (!group->getMimicJointModels().empty())
      throw std::runtime_error("group '" + group_name + "' has a mimic joint");
    if (group->getJointRoots().empty() || !group->getJointRoots()[0]->getParentLinkModel())
      throw std::runtime_error("group '" + group_name + "' has no parent link");

    const auto gravity_arr = request.at("gravity").get<std::array<double, 3>>();
    geometry_msgs::msg::Vector3 gravity;
    gravity.x = gravity_arr[0];
    gravity.y = gravity_arr[1];
    gravity.z = gravity_arr[2];

    dynamics_solver::DynamicsSolver solver(model_, group_name, gravity);
    if (!solver.getGroup())
      throw std::runtime_error("DynamicsSolver failed to initialize for group '" + group_name + "'");

    const std::vector<std::string> joint_names = group->getActiveJointModelNames();
    const std::size_t n = joint_names.size();

    auto readPerJoint = [&](const char* field) {
      const json& values = request.at(field);
      std::vector<double> out(n);
      for (std::size_t i = 0; i < n; ++i)
      {
        if (!values.contains(joint_names[i]))
          throw std::runtime_error(std::string(field) + " missing joint " + joint_names[i]);
        out[i] = values.at(joint_names[i]).get<double>();
      }
      return out;
    };

    const std::vector<double> angles = readPerJoint("joint_values");
    const std::vector<double> velocities = readPerJoint("joint_velocities");
    const std::vector<double> accelerations = readPerJoint("joint_accelerations");
    const double payload = request.at("payload").get<double>();

    // group->getLinkModels() is exactly the KDL chain's segment set: both
    // walk the same joint list from base to tip (moveit_core's
    // JointModelGroup constructor builds link_model_vector_ as one entry per
    // joint in the group, fixed joints included; DynamicsSolver's kdl_chain_
    // is `tree.getChain(base_name_, tip_name_)` over that identical base/tip
    // pair), so this is the wrench vector size getTorques expects without
    // reaching into DynamicsSolver's private num_segments_.
    const std::vector<geometry_msgs::msg::Wrench> zero_wrenches(group->getLinkModels().size());

    std::vector<double> torques(n, 0.0);
    if (!solver.getTorques(angles, velocities, accelerations, zero_wrenches, torques))
      throw std::runtime_error("getTorques failed for group '" + group_name + "'");

    json out;
    out["joint_names"] = joint_names;
    out["torques"] = torques;
    out["max_torques"] = solver.getMaxTorques();

    double max_payload = 0.0;
    unsigned int joint_saturated = 0;
    if (solver.getMaxPayload(angles, max_payload, joint_saturated))
      out["max_payload"] = json{ { "payload", max_payload }, { "joint_saturated", joint_saturated } };
    else
      out["max_payload"] = nullptr;

    std::vector<double> payload_torques(n, 0.0);
    if (solver.getPayloadTorques(angles, payload, payload_torques))
      out["payload_torques"] = payload_torques;
    else
      out["payload_torques"] = nullptr;

    return out;
  }

  /// Reset `*state_` to the model defaults, then apply the request's joint
  /// values on top.
  ///
  /// Reset first: leaving the previous case's values in place would make a
  /// result depend on request order, which would quietly hide a disagreement
  /// on any variable the request omits.
  ///
  /// `clearAttachedBodies()` is part of that reset and not the caller's job.
  /// `setToDefaultValues()` resets joint positions but -- verified by reading
  /// it -- never touches the attached-body list, and `state_` outlives every
  /// request, so an attachment installed by one request is still there for the
  /// next one. Two ops (`collision`, `frameTransform`) used to clear it
  /// themselves right after calling this; the other eight callers did not, and
  /// nothing distinguished the two groups except that someone had thought
  /// about it. Clearing here makes "after `applyJointValues`, `*state_` is the
  /// model defaults with nothing attached" true on every path instead of on
  /// the paths that remembered, so an op that attaches nothing cannot observe
  /// one that does. Ops that *do* attach still attach after this returns,
  /// unchanged.
  ///
  /// `isStateValid` is not in that group: it clears the attached bodies of a
  /// `PlanningScene`'s own `getCurrentStateNonConst()`, which it constructs
  /// per call, so that state has no cross-request lifetime to leak through.
  void applyJointValues(const json& request)
  {
    applyJointValuesTo(*state_, request);
  }

  /// The body of `applyJointValues`, against an arbitrary state.
  ///
  /// Split out for the same reason `applyAttachedBodies` takes its state by
  /// reference: `sceneDiffCollision` poses a `PlanningScene`'s own
  /// `getCurrentStateNonConst()`, not the shared `*state_`, and a second
  /// hand-written copy of this loop is exactly the drift that doc comment
  /// records having already happened once. Every word of `applyJointValues`'
  /// doc above still describes what this does to `*state_` -- the reset, the
  /// `clearAttachedBodies()` and the unknown-variable throw are unchanged and
  /// unconditional, just reachable from a second caller now.
  void applyJointValuesTo(moveit::core::RobotState& state, const json& request)
  {
    state.setToDefaultValues();
    state.clearAttachedBodies();
    const json& values = request.at("joint_values");
    for (auto it = values.begin(); it != values.end(); ++it)
    {
      const std::string& variable = it.key();
      if (!hasVariable(variable))
        throw std::runtime_error("unknown joint variable: " + variable);
      state.setVariablePosition(variable, it.value().get<double>());
    }
    state.update();
  }

  /// Attach `request["attached_bodies"]` (optional, defaults to none) to
  /// `state`, then `update()` it.
  ///
  /// Takes the state by reference rather than always writing `*state_`
  /// because `isStateValid` attaches to a `PlanningScene`'s own
  /// `getCurrentStateNonConst()`, not the shared one.
  ///
  /// This body used to be copy-pasted into `collision`, `frameTransform` and
  /// `isStateValid` -- three copies of the same 25 lines, and the two
  /// distance-field ops that also need it would otherwise have made five. Two of the
  /// three had already drifted: only `frameTransform`'s parsed `subframes`.
  /// The 8-argument `attachBody` with an empty `FixedTransformsMap` is
  /// exactly the 6-argument overload (both trailing parameters are defaulted
  /// to those same empty values), so parsing subframes when present and
  /// passing an empty map otherwise reproduces all three call sites without
  /// changing any of them.
  ///
  /// `pose` is always identity: it is the transform from the link to the
  /// attached body's own frame, upstream's separate level between the link
  /// and its shapes, and `cspace_planning::scene::AttachedBody`'s one-level design has
  /// `shape_poses` relative to the link frame directly -- see `protocol.rs`'s
  /// `AttachedBodySpec` doc.
  void applyAttachedBodies(moveit::core::RobotState& state, const json& request)
  {
    for (const auto& attached_json : request.value("attached_bodies", json::array()))
    {
      const std::string id = attached_json.at("id").get<std::string>();
      const std::string link_name = attached_json.at("link_name").get<std::string>();
      const auto& shapes_json = attached_json.at("shapes");
      const auto& shape_poses_json = attached_json.at("shape_poses");
      std::vector<shapes::ShapeConstPtr> shapes;
      EigenSTL::vector_Isometry3d shape_poses;
      for (std::size_t i = 0; i < shapes_json.size(); ++i)
      {
        const json& shape_json = shapes_json.at(i);
        const std::string shape_type = shape_json.at("type").get<std::string>();
        shapes.push_back(parseShape(shape_type, shape_json));
        shape_poses.push_back(fromRowMajor4x4(shape_poses_json.at(i)));
      }
      std::set<std::string> touch_links;
      if (attached_json.contains("touch_links"))
      {
        for (const auto& link_json : attached_json.at("touch_links"))
        {
          touch_links.insert(link_json.get<std::string>());
        }
      }
      moveit::core::FixedTransformsMap subframes;
      if (attached_json.contains("subframes"))
      {
        for (auto it = attached_json.at("subframes").begin(); it != attached_json.at("subframes").end(); ++it)
          subframes[it.key()] = fromRowMajor4x4(it.value());
      }
      state.attachBody(id, Eigen::Isometry3d::Identity(), shapes, shape_poses, touch_links, link_name,
                       trajectory_msgs::msg::JointTrajectory(), subframes);
    }
    state.update();
  }

  bool hasVariable(const std::string& name) const
  {
    const std::vector<std::string>& names = model_->getVariableNames();
    return std::find(names.begin(), names.end(), name) != names.end();
  }

  json fk(const json& request)
  {
    applyJointValues(request);

    std::vector<std::string> links;
    if (request.contains("links") && !request.at("links").empty())
      links = request.at("links").get<std::vector<std::string>>();
    else
      links = model_->getLinkModelNames();

    json transforms = json::object();
    for (const std::string& link : links)
    {
      if (!model_->hasLinkModel(link))
        throw std::runtime_error("unknown link: " + link);
      transforms[link] = toRowMajor4x4(state_->getGlobalLinkTransform(link));
    }
    return json{ { "link_transforms", transforms } };
  }

  /// Draw whole-model random states with MoveIt's own sampler.
  ///
  /// The oracle owns the randomness rather than the runner:
  /// RobotModel::getVariableRandomPositions normalizes a floating joint's
  /// quaternion, respects each joint type's bounds and derives mimic values.
  /// A runner sampling variable-by-variable would get all three wrong and the
  /// resulting disagreements would be defects in the test, not in the port.
  json randomStates(const json& request)
  {
    const std::size_t count = request.at("count").get<std::size_t>();
    random_numbers::RandomNumberGenerator rng(request.at("seed").get<int>());

    json states = json::array();
    std::vector<double> values(model_->getVariableCount());
    const std::vector<std::string>& names = model_->getVariableNames();
    for (std::size_t i = 0; i < count; ++i)
    {
      model_->getVariableRandomPositions(rng, values.data());
      json state = json::object();
      for (std::size_t v = 0; v < names.size(); ++v)
        state[names[v]] = values[v];
      states.push_back(state);
    }
    return json{ { "states", states } };
  }

  /// Read `request[key]` as a complete variable vector, in the model's own
  /// variable order.
  ///
  /// Complete, not partial: every other op that takes joint values starts
  /// from `setToDefaultValues()` and overrides what the request names, which
  /// is right for FK (an omitted variable is "whatever the model says") and
  /// wrong for the three ops below, where the whole point of a case is the
  /// exact vector it starts from. An omitted variable there would silently
  /// substitute a default and the resulting disagreement would be about the
  /// substitution, not about the operation. Both directions are rejected:
  /// a missing name and a name this model does not have.
  std::vector<double> readFullPositions(const json& request, const char* key) const
  {
    const std::vector<std::string>& names = model_->getVariableNames();
    const json& in = request.at(key);
    if (in.size() != names.size())
      throw std::runtime_error(std::string(key) + " has " + std::to_string(in.size()) +
                               " entries, model has " + std::to_string(names.size()) + " variables");
    std::vector<double> values(names.size());
    for (std::size_t i = 0; i < names.size(); ++i)
    {
      const auto it = in.find(names[i]);
      if (it == in.end())
        throw std::runtime_error(std::string(key) + " is missing variable: " + names[i]);
      values[i] = it->get<double>();
    }
    return values;
  }

  /// `*state_`'s whole position vector as name -> value.
  json positionsToJson() const
  {
    const std::vector<std::string>& names = model_->getVariableNames();
    const double* positions = state_->getVariablePositions();
    json out = json::object();
    for (std::size_t v = 0; v < names.size(); ++v)
      out[names[v]] = positions[v];
    return out;
  }

  /// Every joint of the model -- active *and* mimic -- whose own
  /// `satisfiesPositionBounds(margin 0)` is false for `*state_`'s current
  /// values.
  ///
  /// Over `getJointModels()` rather than `getActiveJointModels()`, and that
  /// difference is the measurement: `RobotState::enforceBounds()` iterates
  /// the active vector, which excludes mimic joints, so the only thing that
  /// can ever write a mimic variable is `updateMimicJoint` propagating from
  /// its master. A mimic whose master is clamped therefore lands at
  /// `factor * clamped + offset` with its own bounds never consulted. Asking
  /// only the active joints would report that state as in-bounds.
  json outOfBoundsJoints() const
  {
    json out = json::array();
    for (const moveit::core::JointModel* joint : model_->getJointModels())
    {
      if (joint->getVariableCount() == 0)
        continue;
      const double* values = state_->getJointPositions(joint);
      if (!joint->satisfiesPositionBounds(values, joint->getVariableBounds(), 0.0))
        out.push_back(joint->getName());
    }
    return out;
  }

  /// `RobotState::enforceBounds()` over a caller-supplied complete variable
  /// vector: PORTING-PLAN.md §5 Phase 2's "관절 한계 클램핑" clause.
  ///
  /// The vector is installed with `setVariablePositions(const double*)`, the
  /// raw memcpy overload, precisely so that no mimic propagation runs before
  /// `enforceBounds()` does -- upstream documents that overload as assuming
  /// "the full state includes mimic joint values". A request can therefore
  /// hand in a mimic variable that disagrees with its master, which is the
  /// only way to observe whether `enforceBounds` re-derives it (it does iff
  /// the master's own `enforcePositionBounds` returned true, which is
  /// unconditional for revolute and change-gated for prismatic).
  json enforceBoundsOp(const json& request)
  {
    const std::vector<double> values = readFullPositions(request, "positions");
    state_->setVariablePositions(values.data());
    state_->enforceBounds();
    return json{ { "enforced", positionsToJson() }, { "out_of_bounds", outOfBoundsJoints() } };
  }

  /// `RobotState::setJointPositions(joint, values)` for each entry of
  /// `joint_positions`, from the model defaults: PORTING-PLAN.md §5 Phase 2's
  /// "mimic 전파" clause. Every mimic variable in the answer got there through
  /// `updateMimicJoint` and nothing else.
  ///
  /// Naming a mimic joint is an error rather than a write. Two writes to the
  /// same mimic relationship -- one direct, one propagated -- would make the
  /// answer depend on the order this loop happens to visit the object's keys,
  /// and a request cannot express an order at all (nlohmann's json object is
  /// key-sorted). Rejecting the input is what keeps the op a function of its
  /// request.
  json mimicPropagateOp(const json& request)
  {
    state_->setToDefaultValues();
    const json& in = request.at("joint_positions");
    for (auto it = in.begin(); it != in.end(); ++it)
    {
      const moveit::core::JointModel* joint = model_->getJointModel(it.key());
      if (!joint)
        throw std::runtime_error("unknown joint: " + it.key());
      if (joint->getMimic())
        throw std::runtime_error("joint_positions names the mimic joint " + it.key() +
                                 "; only the joints it mimics may be written");
      const std::vector<double> values = it.value().get<std::vector<double>>();
      if (values.size() != joint->getVariableCount())
        throw std::runtime_error("joint " + it.key() + " has " + std::to_string(joint->getVariableCount()) +
                                 " variables, request gave " + std::to_string(values.size()));
      state_->setJointPositions(joint, values.data());
    }
    return json{ { "propagated", positionsToJson() } };
  }

  /// `JointModel::interpolate(from, to, t, state)` for one joint:
  /// PORTING-PLAN.md §5 Phase 2's "floating/planar 조인트 보간" clause.
  ///
  /// The joint-level call, not `RobotState::interpolate`, because that is
  /// where every type-specific rule lives -- the continuous-revolute and
  /// planar wrap branches, the floating slerp and its near-identical
  /// shortcut, the diff-drive turn/drive/turn split. `RobotState::interpolate`
  /// adds the loop over active joints and a mimic pass; those are
  /// `stateInterpolateOp` below and not this op, because their *composition*
  /// is not what `mimic_propagate` measures -- there a mimic tracks a value a
  /// setter wrote, here it tracks a value interpolation produced, and the two
  /// differ wherever the master's interpolation is not affine in `t`.
  ///
  /// No `RobotState` is touched at all: upstream's `interpolate` is a pure
  /// function of `from`, `to`, `t` and the joint's own configuration, so
  /// routing it through a state would add a memcpy whose disagreement could
  /// not be told apart from an interpolation one.
  json interpolateOp(const json& request)
  {
    const std::string joint_name = request.at("joint").get<std::string>();
    const moveit::core::JointModel* joint = model_->getJointModel(joint_name);
    if (!joint)
      throw std::runtime_error("unknown joint: " + joint_name);
    const std::size_t count = joint->getVariableCount();
    const std::vector<double> from = request.at("from").get<std::vector<double>>();
    const std::vector<double> to = request.at("to").get<std::vector<double>>();
    if (from.size() != count || to.size() != count)
      throw std::runtime_error("joint " + joint_name + " has " + std::to_string(count) +
                               " variables, request gave from=" + std::to_string(from.size()) +
                               " to=" + std::to_string(to.size()));
    std::vector<double> out(count);
    joint->interpolate(from.data(), to.data(), request.at("t").get<double>(), out.data());
    return json{ { "interpolated", out } };
  }

  /// `RobotState::interpolate` in all three of its overloads: the whole-model
  /// loop (`robot_state.cpp:1138` -> `RobotModel::interpolate`,
  /// `robot_model.cpp:1518`), the group loop (`:1147`) and the single-joint
  /// form (`:1159`).
  ///
  /// `scope` picks the overload. The three are separate measurements, not one
  /// with a filter, because they disagree on the same inputs:
  ///
  /// - the model form propagates **every** mimic in the model;
  /// - the group form propagates only `group->getMimicJointModels()`, so a
  ///   mimic outside the group keeps whatever the destination state already
  ///   held;
  /// - the joint form propagates only that joint's own followers.
  ///
  /// Which is why `seed` exists and is a full vector: the destination's
  /// prior contents are *part of the answer* for the last two, and a
  /// destination the caller cannot set is a destination whose untouched
  /// variables cannot be checked. The three states are local rather than
  /// `state_` so that `from`, `to` and the destination are three distinct
  /// position arrays, as upstream's own callers have them.
  json stateInterpolateOp(const json& request)
  {
    const std::vector<double> from = readFullPositions(request, "from");
    const std::vector<double> to = readFullPositions(request, "to");
    const std::vector<double> seed = readFullPositions(request, "seed");
    const double t = request.at("t").get<double>();
    const std::string scope = request.at("scope").get<std::string>();

    moveit::core::RobotState a(model_), b(model_), out(model_);
    a.setVariablePositions(from.data());
    b.setVariablePositions(to.data());
    out.setVariablePositions(seed.data());

    if (scope == "model")
    {
      a.interpolate(b, t, out);
    }
    else if (scope == "group")
    {
      const std::string group_name = request.at("group").get<std::string>();
      const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_name);
      if (!group)
        throw std::runtime_error("unknown group: " + group_name);
      a.interpolate(b, t, out, group);
    }
    else if (scope == "joint")
    {
      const std::string joint_name = request.at("joint").get<std::string>();
      const moveit::core::JointModel* joint = model_->getJointModel(joint_name);
      if (!joint)
        throw std::runtime_error("unknown joint: " + joint_name);
      a.interpolate(b, t, out, joint);
    }
    else
    {
      throw std::runtime_error("scope must be model, group or joint, got: " + scope);
    }

    const std::vector<std::string>& names = model_->getVariableNames();
    const double* positions = out.getVariablePositions();
    json result = json::object();
    for (std::size_t v = 0; v < names.size(); ++v)
      result[names[v]] = positions[v];
    return json{ { "state_interpolated", result } };
  }

  json jacobian(const json& request)
  {
    const std::string group_name = request.at("group").get<std::string>();
    const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_name);
    if (!group)
      throw std::runtime_error("unknown group: " + group_name);

    applyJointValues(request);

    Eigen::MatrixXd j;
    if (!state_->getJacobian(group, group->getLinkModels().back(), Eigen::Vector3d::Zero(), j))
      throw std::runtime_error("getJacobian failed for group " + group_name);

    json data = json::array();
    for (Eigen::Index r = 0; r < j.rows(); ++r)
      for (Eigen::Index c = 0; c < j.cols(); ++c)
        data.push_back(j(r, c));

    return json{ { "rows", static_cast<std::size_t>(j.rows()) },
                 { "cols", static_cast<std::size_t>(j.cols()) },
                 { "data", data } };
  }

  /// Ground truth for `cspace-metrics`' `KinematicsMetrics`: all four public
  /// values (`getManipulabilityIndex`/`getManipulability`, each at
  /// `translation` true and false, plus `getManipulabilityEllipsoid`) from
  /// `kinematics_metrics::KinematicsMetrics`, at `count` random whole-model
  /// states drawn the same way `randomStates` draws them -- the oracle owns
  /// the randomness (see that op's own comment for why a runner-side sampler
  /// would not be comparable).
  ///
  /// Eigenvalues/eigenvectors come back as `Eigen::MatrixXcd` (complex)
  /// upstream; both real and imaginary parts are dumped rather than
  /// discarding the imaginary half, so the Rust side can assert it is
  /// negligible instead of assuming it -- see `cspace-metrics`'s own doc
  /// comment on why `nalgebra::SymmetricEigen` (real-only) is still a valid
  /// port of a matrix that upstream's own type only defensively allows to be
  /// complex.
  json kinematicsMetrics(const json& request)
  {
    const std::string group_name = request.at("group").get<std::string>();
    const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_name);
    if (!group)
      throw std::runtime_error("unknown group: " + group_name);

    const std::size_t count = request.at("count").get<std::size_t>();
    const double penalty_multiplier = request.at("penalty_multiplier").get<double>();
    random_numbers::RandomNumberGenerator rng(request.at("seed").get<int>());

    kinematics_metrics::KinematicsMetrics metrics(model_);
    metrics.setPenaltyMultiplier(penalty_multiplier);

    std::vector<double> values(model_->getVariableCount());
    const std::vector<std::string>& names = model_->getVariableNames();

    json states = json::array();
    for (std::size_t i = 0; i < count; ++i)
    {
      model_->getVariableRandomPositions(rng, values.data());
      state_->setVariablePositions(values);
      state_->update();

      json joint_values = json::object();
      for (std::size_t v = 0; v < names.size(); ++v)
        joint_values[names[v]] = values[v];

      double index_full = 0.0, index_translation = 0.0;
      double manipulability_full = 0.0, manipulability_translation = 0.0;
      const bool ok_index_full = metrics.getManipulabilityIndex(*state_, group, index_full, false);
      const bool ok_index_translation =
          metrics.getManipulabilityIndex(*state_, group, index_translation, true);
      const bool ok_manipulability_full = metrics.getManipulability(*state_, group, manipulability_full, false);
      const bool ok_manipulability_translation =
          metrics.getManipulability(*state_, group, manipulability_translation, true);

      Eigen::MatrixXcd eigen_values, eigen_vectors;
      const bool ok_ellipsoid = metrics.getManipulabilityEllipsoid(*state_, group, eigen_values, eigen_vectors);

      if (!ok_index_full || !ok_index_translation || !ok_manipulability_full || !ok_manipulability_translation ||
          !ok_ellipsoid)
        throw std::runtime_error("kinematics_metrics: group " + group_name + " is not a chain");

      json eigenvalues_real = json::array();
      json eigenvalues_imag = json::array();
      for (Eigen::Index r = 0; r < eigen_values.rows(); ++r)
      {
        eigenvalues_real.push_back(eigen_values(r, 0).real());
        eigenvalues_imag.push_back(eigen_values(r, 0).imag());
      }

      json eigenvectors_real = json::array();
      json eigenvectors_imag = json::array();
      for (Eigen::Index r = 0; r < eigen_vectors.rows(); ++r)
      {
        for (Eigen::Index c = 0; c < eigen_vectors.cols(); ++c)
        {
          eigenvectors_real.push_back(eigen_vectors(r, c).real());
          eigenvectors_imag.push_back(eigen_vectors(r, c).imag());
        }
      }

      states.push_back(json{
          { "joint_values", joint_values },
          { "manipulability_index_full", index_full },
          { "manipulability_index_translation", index_translation },
          { "manipulability_full", manipulability_full },
          { "manipulability_translation", manipulability_translation },
          { "ellipsoid_eigenvalues_real", eigenvalues_real },
          { "ellipsoid_eigenvalues_imag", eigenvalues_imag },
          { "ellipsoid_eigenvectors_real", eigenvectors_real },
          { "ellipsoid_eigenvectors_imag", eigenvectors_imag },
      });
    }

    return json{ { "group", group_name }, { "penalty_multiplier", penalty_multiplier }, { "states", states } };
  }

  /// The `AllowedCollisionMatrix` `PlanningScene` itself builds: straight from
  /// the loaded SRDF's `disable_collisions`/`enable_collisions`/
  /// `disable_default_collisions`. Shared by `acm()` (which dumps it) and
  /// `collision()` (which filters both `CollisionEnvFCL` checks with it), so
  /// the two never risk building it two different ways.
  collision_detection::AllowedCollisionMatrix buildAcm() const
  {
    return collision_detection::AllowedCollisionMatrix(*model_->getSRDF());
  }

  /// One draw of `searchPositionIK`'s reseed formula
  /// (kdl_kinematics_plugin.cpp:373-382), continuous-joint-aware
  /// (`RevoluteJointModel::getVariableRandomPositionsNearBy`,
  /// revolute_joint_model.cpp:122-136). Factored out so `ik()`'s real reseed
  /// loop and its `reseed_probe` diagnostic below draw from one definition
  /// instead of two copies that could drift apart.
  double sampleReseed(bool continuous, double near, double limit, double min, double max)
  {
    if (continuous)
    {
      // Unclamped `near +/- limit`, then wrapped into `(-pi, pi]` -- not
      // clamped to the joint's (here: reporting-only, [-pi, pi]) bounds.
      // Matches this port's own fix in `near_by_configuration`
      // (crates/cspace-core/src/kinematics/cart_to_jnt.rs).
      double value = ik_rng_.uniformReal(near - limit, near + limit);
      if (value <= -M_PI || value > M_PI)
      {
        value = std::fmod(value, 2.0 * M_PI);
        if (value <= -M_PI)
          value += 2.0 * M_PI;
        else if (value > M_PI)
          value -= 2.0 * M_PI;
      }
      return value;
    }
    return ik_rng_.uniformReal(std::max(min, near - limit), std::min(max, near + limit));
  }

  /// Ground truth for the `ik` op (Phase 4's completion condition -- see
  /// `PORTING-PLAN.md` and `tools/moveit-diff/src/protocol.rs`'s `Op::Ik`
  /// doc comment): `KDLKinematicsPlugin::searchPositionIK`/`CartToJnt`/
  /// `clipToJointLimits`, hand-transcribed here rather than vendored --
  /// unlike `ChainIkSolverVelMimicSVD`, `KDLKinematicsPlugin` itself is
  /// soaked in `rclcpp::Node`/`moveit_ros_planning` (see
  /// `chainiksolver_vel_mimic_svd.hpp`'s own vendoring note) -- reading the
  /// real `KDL::ChainIkSolverVelMimicSVD` at every step, so this is genuine
  /// upstream ground truth for the numerically hardest part (the mimic-aware
  /// SVD fold), not a second copy of this port's own algorithm.
  ///
  /// # Deviation from upstream: fixed retry count, not a wall-clock timeout
  ///
  /// `searchPositionIK`'s `do { ... } while (!timedOut(start_time,
  /// timeout))` retries until wall-clock time runs out, which is not
  /// reproducible and not comparable to a fixed budget. This mirrors
  /// `cspace_core::kinematics::SolverParams::max_restarts`'s own identical
  /// deviation on the Rust side. The retry count itself comes from the
  /// request (`Op::Ik::max_restarts`), not a fixed constant -- round 2 of
  /// the IK success-rate investigation needs to run both sides with
  /// restarts disabled (`max_restarts = 0`) to isolate whether a gap is
  /// restart-RNG divergence (the oracle's own reseed draws come from
  /// `ik_rng_`, a boost mt19937 stream, independent of the Rust side's
  /// `ChaCha8Rng`) or a real algorithmic difference.
  ///
  /// # Deviation from upstream: `checkConsistency`'s out-of-bounds read is
  /// not reproduced
  ///
  /// `request["consistency_limits"]` is `searchPositionIK`'s own parameter,
  /// full-space and keyed by joint name (see `Op::Ik::consistency_limits`'s
  /// doc comment). This method performs upstream's own reduction to
  /// `consistency_limits_mimic` and then checks each active joint's
  /// distance from the seed against its own reduced-space bound -- the
  /// semantics `checkConsistency` clearly intends, but not the letter of
  /// how it is written: that function loops `i < dimension_` (full-space)
  /// while indexing `consistency_limits_mimic[i]`, an active-joint-sized
  /// (not `dimension_`-sized) vector -- a real out-of-bounds
  /// `std::vector::operator[]` read whenever the chain has at least one
  /// mimic joint, confirmed against `kdl_kinematics_plugin.cpp:84-94` and
  /// recorded in `PORTING-PLAN.md`'s round-4 `p1-joints` section. This op
  /// exists to be oracle-comparable ground truth, not to reproduce a read
  /// past the end of a `std::vector` -- so it implements the intended
  /// check instead.
  json ik(const json& request)
  {
    // Diagnostic-only probe: draw `count` samples from `sampleReseed` in
    // isolation for one named joint, bypassing the group/FK/solve machinery
    // below entirely. `ik()`'s own response reports only the final
    // solved-or-failed outcome, never the intermediate reseed draws the loop
    // below actually produces, so nothing else in this file can answer "did
    // this joint's reseed wrap or clamp?" -- see
    // `tools/ci/verify-continuous-reseed-wrap.sh`, which uses this to pin
    // `RevoluteJointModel`'s wrap-not-clamp behaviour (round 6's fix, this
    // file's `sampleReseed`) against a regression nothing else here would
    // notice.
    if (request.contains("reseed_probe"))
    {
      const json& probe = request.at("reseed_probe");
      const std::string joint_name = probe.at("joint").get<std::string>();
      const double near = probe.at("near").get<double>();
      const double limit = probe.at("limit").get<double>();
      const unsigned int count = probe.at("count").get<unsigned int>();

      const moveit::core::JointModel* jm = model_->getJointModel(joint_name);
      if (!jm)
        throw std::runtime_error("reseed_probe: unknown joint '" + joint_name + "'");
      const auto* revolute = dynamic_cast<const moveit::core::RevoluteJointModel*>(jm);
      const bool continuous = revolute != nullptr && revolute->isContinuous();
      const moveit::core::VariableBounds& bounds = jm->getVariableBounds()[0];

      json draws = json::array();
      for (unsigned int i = 0; i < count; ++i)
        draws.push_back(sampleReseed(continuous, near, limit, bounds.min_position_, bounds.max_position_));

      return json{ { "continuous", continuous }, { "draws", draws } };
    }

    constexpr unsigned int kMaxSolverIterations = 500;   // SolverParams::max_solver_iterations default
    constexpr double kEpsilon = 0.00001;                 // SolverParams::epsilon default
    constexpr double kSvdThreshold = 0.001;               // SolverParams::svd_threshold default
    const unsigned int max_restarts = request.at("max_restarts").get<unsigned int>();

    const std::string group_name = request.at("group").get<std::string>();
    const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_name);
    if (!group)
      throw std::runtime_error("unknown group: " + group_name);
    if (!group->isChain())
      throw std::runtime_error("group '" + group_name + "' is not a chain; only chain groups are supported");
    if (!group->isSingleDOFJoints())
      throw std::runtime_error("group '" + group_name + "' includes joints that have more than 1 DOF");

    const bool position_only = request.at("position_only").get<bool>();

    // Target pose: FK at `joint_values`, expressed in this chain's own
    // base-link frame -- the frame KDL::Chain's implicit base is (matches
    // `cspace_core::kinematics::chain::ChainInfo::root_pose_world`).
    applyJointValues(request);
    const moveit::core::LinkModel* tip_link = group->getLinkModels().back();
    const moveit::core::LinkModel* root_link = group->getLinkModels().front()->getParentLinkModel();
    if (!root_link)
      throw std::runtime_error("group '" + group_name + "' starts at the model root; ik() does not support that");

    const Eigen::Isometry3d tip_pose_world = state_->getGlobalLinkTransform(tip_link);
    const Eigen::Isometry3d root_pose_world = state_->getGlobalLinkTransform(root_link);
    const Eigen::Isometry3d target = root_pose_world.inverse() * tip_pose_world;
    const KDL::Frame pose_desired = toKdlFrame(target);

    KDL::Chain kdl_chain;
    if (!kdl_tree_.getChain(root_link->getName(), tip_link->getName(), kdl_chain))
      throw std::runtime_error("could not extract a KDL chain from '" + root_link->getName() + "' to '" +
                                tip_link->getName() + "'");

    // Mimic joints: `KDLKinematicsPlugin::initialize`'s own two-pass build
    // (walk the chain segments once, recording every active joint's own
    // `map_index` in encounter order; a second pass then resolves each
    // mimic's `map_index` from its master's), transcribed faithfully
    // including upstream's own silent-drop behaviour for an in-chain mimic
    // whose master is outside the group -- none of this port's `--ik`
    // fixtures (`panda_arm`, `manipulator`, `left_panda_arm`, `right_arm`)
    // have a mimic joint on the arm chain at all, so that upstream edge
    // case is never reached here.
    std::vector<kdl_kinematics_plugin::JointMimic> mimic_joints;
    std::vector<std::string> active_joint_names;
    std::vector<std::size_t> active_full_index;
    unsigned int joint_counter = 0;
    for (unsigned int i = 0; i < kdl_chain.getNrOfSegments(); ++i)
    {
      const moveit::core::JointModel* jm = model_->getJointModel(kdl_chain.segments[i].getJoint().getName());

      if (jm->getMimic() == nullptr && jm->getVariableCount() > 0)
      {
        kdl_kinematics_plugin::JointMimic mimic_joint;
        mimic_joint.reset(joint_counter);
        mimic_joint.joint_name = jm->getName();
        mimic_joint.active = true;
        mimic_joints.push_back(mimic_joint);
        active_joint_names.push_back(jm->getName());
        active_full_index.push_back(mimic_joints.size() - 1);
        ++joint_counter;
        continue;
      }
      if (group->hasJointModel(jm->getName()) && jm->getMimic() && group->hasJointModel(jm->getMimic()->getName()))
      {
        kdl_kinematics_plugin::JointMimic mimic_joint;
        mimic_joint.joint_name = jm->getName();
        mimic_joint.offset = jm->getMimicOffset();
        mimic_joint.multiplier = jm->getMimicFactor();
        mimic_joints.push_back(mimic_joint);
      }
    }
    for (kdl_kinematics_plugin::JointMimic& mimic_joint : mimic_joints)
    {
      if (mimic_joint.active)
        continue;
      const moveit::core::JointModel* master = model_->getJointModel(mimic_joint.joint_name)->getMimic();
      for (const kdl_kinematics_plugin::JointMimic& other : mimic_joints)
      {
        if (other.joint_name == master->getName())
          mimic_joint.map_index = other.map_index;
      }
    }

    std::vector<double> joint_min(mimic_joints.size());
    std::vector<double> joint_max(mimic_joints.size());
    for (std::size_t i = 0; i < mimic_joints.size(); ++i)
    {
      const moveit::core::VariableBounds& b = model_->getJointModel(mimic_joints[i].joint_name)->getVariableBounds()[0];
      joint_min[i] = b.min_position_;
      joint_max[i] = b.max_position_;
    }

    // Deterministic, bounds-midpoint seed -- see `Op::Ik`'s doc comment for
    // why this never needs to cross the wire, and `buildQFull` for how a
    // reduced-space (active-joint-only) seed becomes the full-space
    // `KDL::JntArray` this solver actually iterates on (a mimic's own
    // full-space entry is its master's value transformed by
    // `multiplier`/`offset`, matching `cspace_core::state::RobotState`'s own
    // mimic derivation the Rust side relies on via `set_variable_position`).
    auto buildQFull = [&](const std::vector<double>& active_values) {
      KDL::JntArray q_full(mimic_joints.size());
      for (std::size_t i = 0; i < mimic_joints.size(); ++i)
      {
        const double master_value = active_values[mimic_joints[i].map_index];
        q_full(i) =
            mimic_joints[i].active ? master_value : mimic_joints[i].multiplier * master_value + mimic_joints[i].offset;
      }
      return q_full;
    };

    std::vector<double> seed_active(active_joint_names.size());
    for (std::size_t k = 0; k < active_joint_names.size(); ++k)
      seed_active[k] = (joint_min[active_full_index[k]] + joint_max[active_full_index[k]]) / 2.0;

    // Which active joints are continuous revolute -- `RevoluteJointModel`
    // is the only joint type in a single-DOF chain group with a
    // `getVariableRandomPositionsNearBy` branch of its own
    // (`revolute_joint_model.cpp:122-136`); `PrismaticJointModel`'s own
    // version (`prismatic_joint_model.cpp:91-96`) has one formula, matching
    // the non-continuous branch below exactly. Planar and floating joints
    // cannot appear here at all -- `isSingleDOFJoints()` (this op's own
    // early check) excludes them.
    std::vector<bool> active_continuous(active_joint_names.size());
    for (std::size_t k = 0; k < active_joint_names.size(); ++k)
    {
      const auto* revolute =
          dynamic_cast<const moveit::core::RevoluteJointModel*>(model_->getJointModel(active_joint_names[k]));
      active_continuous[k] = revolute != nullptr && revolute->isContinuous();
    }

    // `KDLKinematicsPlugin::searchPositionIK`'s own reduction to
    // `consistency_limits_mimic` (lines 325-341: filter the caller's
    // full-space `consistency_limits` down to one entry per *active* joint),
    // transcribed here by joint name rather than by `dimension_`-position so
    // this side and the Rust side never have to agree on a full-space
    // encounter order. `request["consistency_limits"]` carries an entry for
    // every full-space (active + mimic) joint, matching upstream's parameter
    // shape, even though only the active-joint entries are ever read here --
    // exactly like upstream's own `consistency_limits[i]` for a mimic
    // joint's index, which is present in the caller's vector but never
    // pushed onto `consistency_limits_mimic`.
    std::vector<double> consistency_limits_mimic;
    const bool has_consistency_limits =
        request.contains("consistency_limits") && !request.at("consistency_limits").empty();
    if (has_consistency_limits)
    {
      const json& limits = request.at("consistency_limits");
      consistency_limits_mimic.reserve(active_joint_names.size());
      for (const std::string& name : active_joint_names)
      {
        if (!limits.contains(name))
          throw std::runtime_error("consistency_limits missing entry for active joint '" + name + "'");
        consistency_limits_mimic.push_back(limits.at(name).get<double>());
      }
    }

    // `KDLKinematicsPlugin::checkConsistency`'s intent -- reject a converged
    // solution unless every *active* joint stayed within its own
    // `consistency_limits_mimic` bound of the original seed -- reimplemented
    // to loop over `consistency_limits_mimic`'s own (active-joint) size
    // rather than `dimension_`. See `PORTING-PLAN.md`'s round-4 `p1-joints`
    // section, confirming upstream's `checkConsistency` loops `i <
    // dimension_` while indexing that same active-joint-sized vector -- an
    // out-of-bounds `std::vector::operator[]` read on any chain with a mimic
    // joint (`kdl_kinematics_plugin.cpp:84-94`). This reimplementation
    // indexes `consistency_limits_mimic[k]` only for `k` in
    // `[0, active_joint_names.size())`, so it cannot trip that read: each
    // active joint's own full-space value (`q_seed_full`/`q_out` at
    // `active_full_index[k]`) is what upstream's `jnt_seed_state`/
    // `jnt_pos_out` (both full-space) actually hold at that position, so
    // this is the bug-free version of the same per-active-joint check, not a
    // different one.
    auto consistencyOk = [&](const KDL::JntArray& q_seed_full, const KDL::JntArray& q_out_full) {
      if (!has_consistency_limits)
        return true;
      for (std::size_t k = 0; k < active_joint_names.size(); ++k)
      {
        const std::size_t full_i = active_full_index[k];
        if (std::fabs(q_seed_full(full_i) - q_out_full(full_i)) > consistency_limits_mimic[k])
          return false;
      }
      return true;
    };

    KDL::ChainIkSolverVelMimicSVD ik_solver_vel(kdl_chain, mimic_joints, position_only, kSvdThreshold);
    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

    Eigen::Matrix<double, 6, 1> cartesian_weights;
    cartesian_weights.topRows<3>().setConstant(1.0);
    cartesian_weights.bottomRows<3>().setConstant(position_only ? 0.0 : 1.0);
    const Eigen::VectorXd joint_weights = Eigen::VectorXd::Constant(active_joint_names.size(), 1.0);

    // `KDLKinematicsPlugin::CartToJnt`, transcribed directly.
    auto cartToJnt = [&](const KDL::JntArray& q_init, KDL::JntArray& q_out) {
      double last_delta_twist_norm = std::numeric_limits<double>::max();
      double step_size = 1.0;
      KDL::Frame f;
      KDL::Twist delta_twist;
      KDL::JntArray delta_q(q_init.rows());
      KDL::JntArray q_backup(q_init.rows());
      Eigen::ArrayXd extra_joint_weights(joint_weights.rows());
      extra_joint_weights.setOnes();

      q_out = q_init;
      bool success = false;
      for (unsigned int iter = 0; iter < kMaxSolverIterations; ++iter)
      {
        fk_solver.JntToCart(q_out, f);
        delta_twist = KDL::diff(f, pose_desired);

        const double position_error = delta_twist.vel.Norm();
        const double orientation_error = ik_solver_vel.isPositionOnly() ? 0.0 : delta_twist.rot.Norm();
        const double delta_twist_norm = std::max(position_error, orientation_error);
        if (delta_twist_norm <= kEpsilon)
        {
          success = true;
          break;
        }

        if (delta_twist_norm >= last_delta_twist_norm)
        {
          const double old_step_size = step_size;
          step_size *= std::min(0.2, last_delta_twist_norm / delta_twist_norm);
          KDL::Multiply(delta_q, step_size / old_step_size, delta_q);
          q_out = q_backup;
        }
        else
        {
          q_backup = q_out;
          step_size = 1.0;
          last_delta_twist_norm = delta_twist_norm;
          ik_solver_vel.CartToJnt(q_out, delta_twist, delta_q, extra_joint_weights * joint_weights.array(),
                                  cartesian_weights);
        }

        clipToJointLimits(joint_min, joint_max, mimic_joints, q_out, delta_q, extra_joint_weights);

        const double delta_q_norm = delta_q.data.lpNorm<1>();
        if (delta_q_norm < kEpsilon)
        {
          if (step_size < kEpsilon)
            break;
          last_delta_twist_norm = std::numeric_limits<double>::max();
          delta_q.data.setRandom();
          delta_q.data *= std::min(0.1, delta_twist_norm);
          clipToJointLimits(joint_min, joint_max, mimic_joints, q_out, delta_q, extra_joint_weights);
          extra_joint_weights.setOnes();
        }

        KDL::Add(q_out, delta_q, q_out);
      }
      return success;
    };

    // Consistency is always measured against the *original* seed, never a
    // retry's own reseed point -- matches `searchPositionIK`'s
    // `jnt_seed_state`, set once before the retry loop and never
    // reassigned inside it.
    const KDL::JntArray q_seed_full = buildQFull(seed_active);

    KDL::JntArray q_out(mimic_joints.size());
    bool success = cartToJnt(q_seed_full, q_out) && consistencyOk(q_seed_full, q_out);
    for (unsigned int attempt = 0; attempt < max_restarts && !success; ++attempt)
    {
      // `searchPositionIK`'s own branch (kdl_kinematics_plugin.cpp:373-382):
      // reseed near the *original* seed, clamped to each active joint's own
      // consistency_limits_mimic bound, whenever limits are active; only
      // fall back to a full-range draw when they are not. Sampling full-range
      // unconditionally here (as this loop did before this fix) starves the
      // consistency check under a tight limit -- almost every full-range
      // reseed lands too far from `q_seed_full` to pass `consistencyOk`
      // even when `cartToJnt` converges, which is a real bug, not RNG noise
      // (see PORTING-PLAN.md's round-5 `p1-joints` section).
      std::vector<double> reseed_active(active_joint_names.size());
      for (std::size_t k = 0; k < active_joint_names.size(); ++k)
      {
        const std::size_t full_i = active_full_index[k];
        if (has_consistency_limits)
        {
          // `searchPositionIK`'s own branch (kdl_kinematics_plugin.cpp:373-382):
          // reseed near the *original* seed, clamped to each active joint's
          // own consistency_limits_mimic bound, whenever limits are active;
          // only fall back to a full-range draw when they are not (the
          // `else` branch below). `sampleReseed` is what actually branches
          // on continuity -- see its own doc comment.
          reseed_active[k] = sampleReseed(active_continuous[k], seed_active[k], consistency_limits_mimic[k],
                                           joint_min[full_i], joint_max[full_i]);
        }
        else
        {
          reseed_active[k] = ik_rng_.uniformReal(joint_min[full_i], joint_max[full_i]);
        }
      }
      success = cartToJnt(buildQFull(reseed_active), q_out) && consistencyOk(q_seed_full, q_out);
    }

    if (!success)
      return json{ { "success", false } };

    json solution = json::object();
    for (std::size_t k = 0; k < active_joint_names.size(); ++k)
      solution[active_joint_names[k]] = q_out(active_full_index[k]);
    return json{ { "success", true }, { "solution", solution } };
  }

  /// Ground truth for the `cspace-collision` differential test: builds an
  /// `AllowedCollisionMatrix` the same way `PlanningScene` does, from the
  /// loaded SRDF's `disable_collisions`/`enable_collisions`/
  /// `disable_default_collisions`, and dumps every explicit entry plus every
  /// default entry. `AllowedCollision::CONDITIONAL` never appears here: the
  /// SRDF-driven constructor only ever calls the bool-taking `setEntry`, never
  /// the predicate overload.
  json acm() const
  {
    collision_detection::AllowedCollisionMatrix matrix = buildAcm();

    std::vector<std::string> names;
    matrix.getAllEntryNames(names);

    json entries = json::array();
    for (std::size_t i = 0; i < names.size(); ++i)
    {
      for (std::size_t j = i; j < names.size(); ++j)
      {
        collision_detection::AllowedCollision::Type type;
        if (matrix.getEntry(names[i], names[j], type))
        {
          entries.push_back(json{ { "link1", names[i] },
                                   { "link2", names[j] },
                                   { "type", allowedCollisionTypeToString(type) } });
        }
      }
    }

    json defaults = json::object();
    for (const std::string& name : names)
    {
      collision_detection::AllowedCollision::Type type;
      if (matrix.getDefaultEntry(name, type))
        defaults[name] = allowedCollisionTypeToString(type);
    }

    return json{ { "names", names }, { "entries", entries }, { "defaults", defaults } };
  }

  /// Ground truth for `cspace-collision::ParryCollisionEnv` (PORTING-PLAN.md
  /// §5's Phase 3 completion condition): `CollisionEnvFCL::checkSelfCollision`,
  /// `checkRobotCollision`, `distanceSelf` and `distanceRobot` at the request's
  /// joint values, filtered through the same SRDF-derived ACM `acm()` dumps
  /// (`buildAcm()` — one construction, not two independently-typed ones).
  ///
  /// `request["objects"]` builds a `collision_detection::World` of real
  /// shapes (`parseShape`, the same parser `shapePoints`/
  /// `collisionDistanceFieldTypes` use) rather than `world()`'s dummy
  /// spheres: `world()` exists to test pose composition only, but a
  /// collision check needs the actual geometry to produce a non-trivial
  /// `robot_collision`/`robot_distance` answer.
  ///
  /// `request["attached_bodies"]` (optional, defaults to none) is applied to
  /// `*state_` via `attachBody` before any check runs, ground truth for
  /// `cspace_planning::scene::AttachedBody`/`cspace_collision::AttachedBodyGeometry`.
  /// No clearing is needed here: `applyJointValues` has already left `*state_`
  /// with nothing attached, on every path rather than only this one -- see its
  /// doc for why that moved. `attachBody`'s own `pose` parameter (the transform from the
  /// link to the attached body's own frame, upstream's separate level
  /// between the link and its shapes) is always identity here, matching
  /// `cspace_planning::scene::AttachedBody`'s one-level design where `shape_poses` are
  /// already relative to the link frame directly -- see `protocol.rs`'s
  /// `AttachedBodySpec` doc.
  ///
  /// Both `CollisionRequest`s now set `contacts = true` (round-8 item 1):
  /// `self_contacts`/`robot_contacts` in the response name every contacting
  /// pair's body names, `BodyType`s and each side's shape kinds
  /// (`contactsToJson`/`shapeKindsFor` below), so a boolean-agreeing,
  /// depth-differing case can be diagnosed on first sight instead of by
  /// re-deriving the sweep's seed after the fact -- the gap round 7's item 2
  /// (pr2 case 7552) hit and could not close from the boolean/depth pair
  /// alone. [`Contact`] coordinates (`pos`/`normal`/`nearest_points`/
  /// `percent_interpolation`) stay excluded from the rust/oracle comparison
  /// per §4.5's recorded verification limit -- see
  /// `crates/cspace-collision/tests/collision_parity.rs` and
  /// `tools/moveit-diff`'s own collision comparison for why; only the pair's
  /// identity and depth are diagnostic additions here, not a widening of
  /// what parity asserts equal. `max_contacts` is raised from the
  /// default-constructed `1` so more than one simultaneously-contacting pair
  /// is nameable, not just the first the backend happens to visit;
  /// `max_contacts_per_pair` is a request field defaulting to `1`, raised
  /// when a caller needs to see whether FCL's narrow phase picked a different
  /// contact than an exhaustive search would (see its comment in the handler
  /// body, and p3-acm's case 623). Both
  /// `DistanceRequest`s set `enable_signed_distance = true`: a request that
  /// left it `false` could never surface deviation 6 (this port's
  /// single-`query::contact()`-call signed distance versus FCL's `distance`
  /// then, only for a penetrating pair, a second up-to-200-contact `collide`
  /// pass taking the deepest penetration) at all, since an unsigned distance
  /// is clamped to `>= 0` on both sides regardless of that deviation.
  /// `self_distance_pair`/`robot_distance_pair` (`distancePairToJson` below)
  /// name the pair behind `self_distance`/`robot_distance` directly from
  /// `DistanceResultsData::link_names`/`body_types` -- pr2 case 7552 is a
  /// `*_distance` disagreement, not a `*_collision` one, so this field, not
  /// `contactsToJson`'s, is what actually names its pair.
  json collision(const json& request)
  {
    applyJointValues(request);
    applyAttachedBodies(*state_, request);

    collision_detection::AllowedCollisionMatrix acm = buildAcm();

    auto world = std::make_shared<collision_detection::World>();
    addRequestObjects(*world, request);

    collision_detection::CollisionEnvFCL env(model_, world);

    // Requested by p3-acm to settle case 623 of the seed-4 pr2 sweep, where
    // this op reports a deep `robot_distance` for the same (cone, link) pair
    // whose own deepest triangle, fed to libccd's MPR directly, reads a
    // plateau value. The candidate explanation is that FCL's narrow phase
    // reported a *different* triangle than the port's exhaustive search
    // found, and at `max_contacts_per_pair = 1` -- MoveIt's own default, and
    // this op's until now -- that is unobservable: exactly one contact per
    // pair comes back and there is nothing to compare it against.
    const std::size_t max_contacts_per_pair =
        request.value("max_contacts_per_pair", static_cast<std::size_t>(1));
    if (max_contacts_per_pair == 0)
      throw std::runtime_error("max_contacts_per_pair must be >= 1");

    collision_detection::CollisionRequest self_req;
    self_req.contacts = true;
    self_req.max_contacts = 100;
    self_req.max_contacts_per_pair = max_contacts_per_pair;
    collision_detection::CollisionResult self_res;
    env.checkSelfCollision(self_req, self_res, *state_, acm);

    collision_detection::CollisionRequest robot_req;
    robot_req.contacts = true;
    robot_req.max_contacts = 100;
    robot_req.max_contacts_per_pair = max_contacts_per_pair;
    collision_detection::CollisionResult robot_res;
    env.checkRobotCollision(robot_req, robot_res, *state_, acm);

    collision_detection::DistanceRequest self_dreq;
    self_dreq.enable_signed_distance = true;
    self_dreq.acm = &acm;
    collision_detection::DistanceResult self_dres;
    env.distanceSelf(self_dreq, self_dres, *state_);

    collision_detection::DistanceRequest robot_dreq;
    robot_dreq.enable_signed_distance = true;
    robot_dreq.acm = &acm;
    collision_detection::DistanceResult robot_dres;
    env.distanceRobot(robot_dreq, robot_dres, *state_);

    return json{
      { "self_collision", self_res.collision },
      { "self_distance", self_dres.minimum_distance.distance },
      { "self_distance_pair", distancePairToJson(self_dres.minimum_distance, *world) },
      // `allContactsToJson`, not `contactsToJson`, on both of these -- and
      // that is not a fixture-rewriting change. `contactsToJson` emits one
      // entry per non-empty pair (`contact_list.front()`); `allContactsToJson`
      // emits every element of every list, in the same `ContactMap` iteration
      // order. At `max_contacts_per_pair == 1` no list holds more than one
      // element, so the two produce the identical array by construction, which
      // is what keeps every fixture committed before this field existed
      // byte-identical and what `verify-fixture-replay.sh` checks. Making the
      // dump follow the field instead of branching on it keeps one rule here
      // -- "report up to `max_contacts_per_pair` contacts per pair" -- rather
      // than a response shape that means different things depending on a
      // request value.
      { "self_contacts", allContactsToJson(self_res.contacts, *world) },
      { "robot_collision", robot_res.collision },
      { "robot_distance", robot_dres.minimum_distance.distance },
      { "robot_distance_pair", distancePairToJson(robot_dres.minimum_distance, *world) },
      { "robot_contacts", allContactsToJson(robot_res.contacts, *world) },
    };
  }

  /// FCL's own signed distance for exactly one robot-link / world-object
  /// pair, computed by calling `fcl::distance()` directly with FCL's
  /// `DistanceRequest::enable_signed_distance = true` -- bypassing
  /// `collision_detection_fcl`'s `distanceCallback` entirely.
  ///
  /// `distanceCallback` (`collision_common.cpp:603`) builds its own FCL
  /// request as `fcl::DistanceRequestd(cdata->req->enable_nearest_points)` --
  /// one positional argument, so FCL's own `enable_signed_distance` stays at
  /// its default `false` no matter what MoveIt's *own*, differently-scoped
  /// `DistanceRequest::enable_signed_distance` (`cdata->req->
  /// enable_signed_distance`) says. That MoveIt-level flag only gates a
  /// separate, hand-rolled fallback a few lines down (`:636-680`): re-run the
  /// pair through `fcl::collide` (up to 200 contacts) and take the one with
  /// the largest `penetration_depth`. FCL's own native signed-distance path
  /// -- `ShapeDistanceTraversalNode::leafTesting` dispatching to
  /// `nsolver->shapeSignedDistance` when `this->request.enable_signed_distance
  /// == true` (`shape_distance_traversal_node-inl.h:85-92`, fcl 0.7.0) -- is
  /// never reached from `distanceCallback`, for any shape pair.
  ///
  /// That native path is real signed distance (libccd's EPA,
  /// `ccdGJKSignedDist`, `gjk_libccd-inl.h:2186-2226`) for any
  /// primitive-primitive pair under `GST_LIBCCD`; fcl's own
  /// `distance_request.h` documents this combination as "SD_1": exact signed
  /// distance via GJK/EPA. It is what `distanceCallback`'s workaround stands
  /// in for, and unlike that workaround it has no ordering dependence
  /// (nothing here visits a second pair, so
  /// `distance-callback-threshold-suppresses-deeper-pairs` cannot fire) and
  /// no multi-contact-set ambiguity (EPA reports the one deepest witness, not
  /// a vote among up to 200 contacts from a separate `fcl::collide` call, so
  /// neither `fcl-distance-sentinel-survives-zero-contacts` nor
  /// `distance-callback-max-contact-depth` can fire either). This is what
  /// lets `box x box` be measured at all: `distanceCallback`'s workaround was
  /// the only reason it was excluded from the existing sphere-probe corpus,
  /// and this op does not use that workaround.
  ///
  /// Mesh is a different case and this op does NOT extend to it. Its leaf
  /// distance test, `ShapeMeshDistanceTraversalNode::leafTesting`, calls
  /// `nsolver->shapeTriangleDistance` unconditionally
  /// (`shape_mesh_distance_traversal_node-inl.h:88`) -- there is no signed
  /// variant to opt into at that level, `enable_signed_distance` is never
  /// read on the mesh leaf path, and fcl's own request-flag documentation
  /// puts "mesh and octree" at "SD_2" (penetration by an external workaround)
  /// even when signed distance is requested. Calling this op against a mesh
  /// link answers with the same kind of unreliable value `distanceCallback`
  /// already produces at `:603` -- callers must not treat that as ground
  /// truth.
  ///
  /// Both objects are built with MoveIt's own, unmodified
  /// `collision_detection::createCollisionGeometry`/`transform2fcl` -- the
  /// same factory `CollisionEnvFCL` itself uses -- so the geometry is
  /// identical to what any other op would see; the only thing this op does
  /// differently is which FCL entry point it calls, and with which request.
  ///
  /// `request["link"]` names the one robot link (exactly one collision
  /// shape, the same constraint the existing sphere-probe corpus places on
  /// its targets) and `request["objects"]` (parsed by `addRequestObjects`)
  /// must hold exactly one world object with exactly one shape -- this op
  /// answers for one pair, nothing broader.
  json pairSignedDistance(const json& request)
  {
    applyJointValues(request);

    const std::string link_name = request.at("link").get<std::string>();
    if (!model_->hasLinkModel(link_name))
      throw std::runtime_error("pair_signed_distance: no such link: " + link_name);
    const moveit::core::LinkModel* link = model_->getLinkModel(link_name);
    const std::vector<shapes::ShapeConstPtr>& link_shapes = link->getShapes();
    if (link_shapes.size() != 1)
      throw std::runtime_error("pair_signed_distance: " + link_name +
                                " does not have exactly one collision shape");

    collision_detection::FCLGeometryConstPtr link_geom =
        collision_detection::createCollisionGeometry(link_shapes[0], link, 0);
    const Eigen::Isometry3d link_shape_pose =
        state_->getGlobalLinkTransform(link) * link->getCollisionOriginTransforms()[0];
    fcl::CollisionObjectd link_object(link_geom->collision_geometry_,
                                       collision_detection::transform2fcl(link_shape_pose));

    auto world = std::make_shared<collision_detection::World>();
    addRequestObjects(*world, request);
    const json& objects = request.at("objects");
    if (objects.size() != 1)
      throw std::runtime_error("pair_signed_distance: objects must hold exactly one object");
    const std::string object_id = objects.at(0).at("id").get<std::string>();
    collision_detection::World::ObjectConstPtr world_object = world->getObject(object_id);
    if (!world_object || world_object->shapes_.size() != 1)
      throw std::runtime_error("pair_signed_distance: world object must have exactly one shape");

    collision_detection::FCLGeometryConstPtr object_geom =
        collision_detection::createCollisionGeometry(world_object->shapes_[0], world_object.get());
    fcl::CollisionObjectd world_object_fcl(
        object_geom->collision_geometry_,
        collision_detection::transform2fcl(world_object->global_shape_poses_[0]));

    fcl::DistanceRequestd fcl_request(/*enable_nearest_points_=*/false,
                                       /*enable_signed_distance=*/true);
    fcl::DistanceResultd fcl_result;
    fcl::distance(&link_object, &world_object_fcl, fcl_request, fcl_result);

    return json{
      { "fcl_signed_distance", fcl_result.min_distance },
    };
  }

  /// Shape kinds for one `collision`-op contact-report body -- reuses
  /// `shapeTypeName`, the same naming `link_details[].shape_types` (see
  /// `linkDetails` above) already uses, so a `robot_link` entry here reads
  /// identically to that op's own `shape_types`. `BodyTypes::ROBOT_ATTACHED`
  /// looks up `*state_` (attached bodies are per-request, applied in
  /// `collision()` before either check runs) rather than `model_`, since an
  /// attached body has no `LinkModel`.
  /// `null` when `name` does not denote a body, an array of shape kinds when
  /// it does -- including the empty array, which means a real body that has
  /// no collision geometry (several pr2 links are like that). The two are
  /// deliberately different values: one meaning per value, so a fixture
  /// reader can tell "no shapes" from "no such body" without knowing which
  /// op produced it.
  ///
  /// Not every `Contact` names a body on both sides.
  /// `CollisionEnvDistanceField` compares each link against one aggregated
  /// distance field rather than against another body, so there is no second
  /// body to name and it writes a sentinel instead: `"self"` typed
  /// `ROBOT_LINK` (`collision_env_distance_field.cpp:326-327`) and
  /// `"environment"` typed `WORLD_OBJECT` (`:1615-1616`). Neither exists in
  /// the model or the world, and the type tag is part of the sentinel, not a
  /// promise -- so dispatching on the type alone and dereferencing the
  /// lookup, which is what this did before, killed the process on the first
  /// such contact (`getLinkModel("self")` returns null after logging).
  ///
  /// The guard is a presence test rather than a comparison against those two
  /// literals on purpose: it is the same rule for every op and every body
  /// type, so a third sentinel added upstream lands as `null` in a fixture
  /// instead of as a crash, and no op needs a boundary case for the ops that
  /// emit sentinels.
  json shapeKindsFor(collision_detection::BodyType type, const std::string& name,
                      const collision_detection::World& world) const
  {
    json kinds = json::array();
    switch (type)
    {
      case collision_detection::BodyTypes::ROBOT_LINK:
        if (!model_->hasLinkModel(name))
          return nullptr;
        for (const shapes::ShapeConstPtr& shape : model_->getLinkModel(name)->getShapes())
          kinds.push_back(shapeTypeName(shape->type));
        break;
      case collision_detection::BodyTypes::ROBOT_ATTACHED:
        if (!state_->getAttachedBody(name))
          return nullptr;
        for (const shapes::ShapeConstPtr& shape : state_->getAttachedBody(name)->getShapes())
          kinds.push_back(shapeTypeName(shape->type));
        break;
      case collision_detection::BodyTypes::WORLD_OBJECT:
        if (!world.hasObject(name))
          return nullptr;
        for (const shapes::ShapeConstPtr& shape : world.getObject(name)->shapes_)
          kinds.push_back(shapeTypeName(shape->type));
        break;
    }
    return kinds;
  }

  /// One `GradientInfo` as JSON. Split out so the attached-body gradient
  /// slots report the same object shape as the per-link ones rather than a
  /// second hand-written copy.
  ///
  /// `gradients` (the `std::vector<Eigen::Vector3d>` member) is deliberately
  /// absent -- see `groupStateRepresentation`'s doc for why it holds
  /// indeterminate bytes and cannot be compared against anything.
  json gradientInfoToJson(const collision_detection::GradientInfo& g) const
  {
    json types_out = json::array();
    for (collision_detection::CollisionType t : g.types)
      types_out.push_back(static_cast<int>(t));
    return json{ { "closest_distance", g.closest_distance },
                 { "collision", g.collision },
                 { "types", types_out },
                 { "distances", g.distances },
                 { "sphere_radii", g.sphere_radii },
                 { "joint_name", g.joint_name },
                 { "sphere_locations_count", g.sphere_locations.size() } };
  }

  /// Add `request["objects"]` (optional, defaults to none) to `world`.
  ///
  /// The single-shape object schema `{id, pose, shape}` that `collision`,
  /// `is_state_valid` and the OMPL planning op all read, in one place: the
  /// loop body was copy-pasted three times identically, and the two
  /// distance-field ops that now need a world would have made five.
  ///
  /// `frameTransform`'s loop is deliberately not folded in -- it parses a
  /// per-object `subframes` map as well, and there is no request field that
  /// distinguishes "sent no subframes" from "sent an empty map" in a way the
  /// other three callers would agree on. The multi-shape builders
  /// (`collisionObjectPointDecomposition`, the octree ops) read a different
  /// schema entirely and are untouched.
  void addRequestObjects(collision_detection::World& world, const json& request)
  {
    for (const auto& object_json : request.value("objects", json::array()))
    {
      const std::string id = object_json.at("id").get<std::string>();
      const Eigen::Isometry3d pose = fromRowMajor4x4(object_json.at("pose"));
      const json& shape_json = object_json.at("shape");
      std::shared_ptr<shapes::Shape> shape = parseShape(shape_json.at("type").get<std::string>(), shape_json);
      world.addToObject(id, pose, { shape }, { Eigen::Isometry3d::Identity() });
    }
  }

  /// One entry per contacting pair in a `CollisionResult::ContactMap` --
  /// body names, `BodyType`s (`bodyTypeName`) and each side's shape kinds
  /// (`shapeKindsFor`), plus the representative contact's `depth`. Added for
  /// round-8 item 1: a case whose booleans agree and only its depth differs
  /// (deviation 6's species, or pr2 case 7552 before this round) previously
  /// could not be traced to a link pair after the fact at all. Deliberately
  /// excludes `pos`/`normal`/`nearest_points`/`percent_interpolation` -- see
  /// `collision()`'s own doc comment for why those stay outside the parity
  /// comparison. `max_contacts_per_pair` is left at its default `1`
  /// (`collision()` only raises `max_contacts`), so `contact_list` always
  /// holds exactly one entry per pair here; guarding on `empty()` rather
  /// than assuming that stays true if a future caller changes the request.
  json contactsToJson(const collision_detection::CollisionResult::ContactMap& contacts,
                       const collision_detection::World& world) const
  {
    json out = json::array();
    for (const auto& entry : contacts)
    {
      const std::vector<collision_detection::Contact>& contact_list = entry.second;
      if (contact_list.empty())
        continue;
      out.push_back(contactToJson(contact_list.front(), world));
    }
    return out;
  }

  /// One `Contact` as JSON. Split out of `contactsToJson` so that
  /// `allContactsToJson` reports the same object shape rather than a second
  /// hand-written copy of these seven fields.
  json contactToJson(const collision_detection::Contact& c, const collision_detection::World& world) const
  {
    return json{
      { "body_name_1", c.body_name_1 },
      { "body_type_1", bodyTypeName(c.body_type_1) },
      { "shape_kinds_1", shapeKindsFor(c.body_type_1, c.body_name_1, world) },
      { "body_name_2", c.body_name_2 },
      { "body_type_2", bodyTypeName(c.body_type_2) },
      { "shape_kinds_2", shapeKindsFor(c.body_type_2, c.body_name_2, world) },
      { "depth", c.depth },
    };
  }

  /// Every contact of every pair, not just each pair's first.
  ///
  /// `contactsToJson` reports `contact_list.front()` alone, and its callers'
  /// committed fixtures assert that shape; changing it would rewrite them.
  /// The distance-field ops need the whole list instead -- their
  /// `max_contacts_per_pair` request field exists precisely to make a pair
  /// carry more than one contact, and a front-only dump would make that
  /// field unobservable.
  json allContactsToJson(const collision_detection::CollisionResult::ContactMap& contacts,
                         const collision_detection::World& world) const
  {
    json out = json::array();
    for (const auto& entry : contacts)
    {
      for (const collision_detection::Contact& c : entry.second)
        out.push_back(contactToJson(c, world));
    }
    return out;
  }

  /// Names the pair behind a `distanceSelf`/`distanceRobot` scalar directly
  /// from `DistanceResultsData::link_names`/`body_types` -- unlike
  /// `contactsToJson`, this needs no `CollisionRequest`-side inference at
  /// all, since upstream's distance query already carries the pair that
  /// produced `minimum_distance` on the very struct that carries the
  /// number. Added for round-8 item 1 alongside `contactsToJson`: pr2 case
  /// 7552 (`collision_parity.rs`) is a `self_distance`/`robot_distance`
  /// disagreement, not a `self_collision`/`robot_collision` one, so this is
  /// the field that actually answers it, rather than the contacts side.
  /// `link_names[0]`/`[1]` are empty together
  /// (`DistanceResultsData::clear()`) when nothing was ever found -- e.g. no
  /// other body exists to measure against -- so `null` is returned rather
  /// than looking up an empty name.
  json distancePairToJson(const collision_detection::DistanceResultsData& d,
                           const collision_detection::World& world) const
  {
    if (d.link_names[0].empty() || d.link_names[1].empty())
      return nullptr;
    return json{
      { "body_name_1", d.link_names[0] },
      { "body_type_1", bodyTypeName(d.body_types[0]) },
      { "shape_kinds_1", shapeKindsFor(d.body_types[0], d.link_names[0], world) },
      { "body_name_2", d.link_names[1] },
      { "body_type_2", bodyTypeName(d.body_types[1]) },
      { "shape_kinds_2", shapeKindsFor(d.body_types[1], d.link_names[1], world) },
    };
  }

  /// Ground truth for `PlanningScene::frame_transform`/`knows_frame_transform`
  /// (`cspace-scene`). Builds a real `planning_scene::PlanningScene` from
  /// `model_` so upstream's own three-tier ladder
  /// (`planning_scene.cpp:2036`/`:2061`) runs unmodified: `RobotState::
  /// getFrameInfo` (model frame, link, attached-body id/subframe --
  /// attached bodies applied to `*state_` exactly as `collision()` does,
  /// plus an optional per-body `subframes` map), then `World::getTransform`/
  /// `knowsTransform` (world objects fed from `request["objects"]`, same
  /// shape as the `world` op, with an optional per-object `subframes` map),
  /// then the TF tier (`SceneTransforms`, always empty here since nothing
  /// ever calls `setTransforms` on it -- it can only ever contribute "not
  /// found", the same "no TF tier" gap `PlanningScene::frame_transform`'s
  /// own doc records).
  ///
  /// `PlanningScene::getFrameTransform`'s documented contract is "return
  /// identity when no transform is available, use `knowsFrameTransform` to
  /// tell the two apart" -- so `transform` in the response is always
  /// upstream's actual return value (identity when unresolved), reported
  /// alongside `knows_transform` rather than folded into one optional. This
  /// also captures `world.rs`'s documented `knowsTransform`/`getTransform`
  /// ambiguity (a subframe name colliding with a sibling object's name)
  /// exactly as it surfaces through the *scene*'s ladder, from upstream
  /// directly, not re-derived from reading world.cpp.
  json frameTransform(const json& request)
  {
    applyJointValues(request);
    applyAttachedBodies(*state_, request);

    planning_scene::PlanningScene scene(model_);
    scene.getCurrentStateNonConst() = *state_;

    for (const auto& object_json : request.value("objects", json::array()))
    {
      const std::string id = object_json.at("id").get<std::string>();
      const Eigen::Isometry3d pose = fromRowMajor4x4(object_json.at("pose"));
      const json& shape_json = object_json.at("shape");
      const std::string shape_type = shape_json.at("type").get<std::string>();
      std::shared_ptr<shapes::Shape> shape = parseShape(shape_type, shape_json);
      scene.getWorldNonConst()->addToObject(id, pose, { shape }, { Eigen::Isometry3d::Identity() });

      if (object_json.contains("subframes") && !object_json.at("subframes").empty())
      {
        moveit::core::FixedTransformsMap subframes;
        for (auto it = object_json.at("subframes").begin(); it != object_json.at("subframes").end(); ++it)
          subframes[it.key()] = fromRowMajor4x4(it.value());
        scene.getWorldNonConst()->setSubframesOfObject(id, subframes);
      }
    }

    json queries_out = json::array();
    for (const auto& query_json : request.at("queries"))
    {
      const std::string name = query_json.get<std::string>();
      const bool knows = scene.knowsFrameTransform(name);
      const Eigen::Isometry3d transform = scene.getFrameTransform(name);
      queries_out.push_back(json{
        { "name", name },
        { "knows_transform", knows },
        { "transform", toRowMajor4x4(transform) },
      });
    }

    return json{ { "queries", queries_out } };
  }

  /// One `CostSource` as JSON. Shared by both cost-source ops so the pair
  /// cannot drift into two spellings of the same three fields.
  ///
  /// `getVolume()` is not emitted: it is `(aabb_max - aabb_min)`'s product,
  /// derivable from what is here, and emitting a derived quantity would let
  /// a fixture agree on the product while disagreeing on the bounds.
  json costSourceToJson(const collision_detection::CostSource& cs) const
  {
    return json{ { "aabb_min", cs.aabb_min }, { "aabb_max", cs.aabb_max }, { "cost", cs.cost } };
  }

  /// `std::set<CostSource>` in iteration order, which is already
  /// most-costly-first: `CostSource::operator<` sorts on `cost * getVolume()`
  /// descending, tie-breaking on `cost` then `aabb_min`
  /// (`collision_common.hpp:128-141`). The order is emitted as-is rather than
  /// re-sorted here, so a Rust-side container whose ordering disagrees shows
  /// up as a fixture mismatch instead of being normalised away.
  json costSourcesToJson(const std::set<collision_detection::CostSource>& costs) const
  {
    json out = json::array();
    for (const collision_detection::CostSource& cs : costs)
      out.push_back(costSourceToJson(cs));
    return out;
  }

  /// Ground truth for `PlanningScene::cost_sources` -- upstream's
  /// `PlanningScene::getCostSources(const RobotState&, std::size_t, const
  /// std::string&, std::set<CostSource>&)` (`planning_scene.cpp:2499-2510`).
  ///
  /// This is the `state` half of the four overloads, and it is deliberately
  /// *not* symmetric with the trajectory half: it runs one `checkCollision`
  /// with `creq.cost = true` and swaps the result out, with no
  /// `removeCostSources`/`removeOverlapping` pass. p1-fixtures found that
  /// asymmetry by reading the two bodies rather than assuming the pairs
  /// matched; this op exists so the Rust port's copy of it is checked against
  /// upstream instead of against that reading.
  ///
  /// `group_name` is optional and defaults to `""`, which upstream's own
  /// two-argument overload passes to mean "the complete robot"
  /// (`collision_common.hpp:149-150`) -- so the omitted case exercises the
  /// same path the omitting overload does, and no separate op is needed for
  /// it. `objects`/`attached_bodies` read the same schemas as `collision`.
  json costSources(const json& request)
  {
    planning_scene::PlanningScene scene(model_);
    addRequestObjects(*scene.getWorldNonConst(), request);

    applyJointValues(request);
    applyAttachedBodies(*state_, request);

    const auto max_costs = request.at("max_costs").get<std::size_t>();
    const std::string group_name = request.value("group_name", std::string());

    std::set<collision_detection::CostSource> costs;
    scene.getCostSources(*state_, max_costs, group_name, costs);

    return json{ { "cost_sources", costSourcesToJson(costs) } };
  }

  /// Ground truth for `PlanningScene::path_cost_sources` -- upstream's
  /// trajectory overload (`planning_scene.cpp:2457-2491`).
  ///
  /// The three steps after the per-waypoint union are order-dependent and
  /// reproduced in upstream's order: truncate the union to `max_costs`,
  /// then `removeCostSources(costs, cs_start, overlap_fraction)` against the
  /// *first waypoint's* sources alone, then `removeOverlapping(costs,
  /// overlap_fraction)` (`:2477-2490`). `cs_start` is captured by `swap` at
  /// `i == 0`, so it is the first waypoint's set and not a copy of the
  /// running union -- a distinction only a trajectory whose first waypoint
  /// collides can show, which is why `waypoints` is a required field with no
  /// default rather than something this op will synthesise.
  ///
  /// Every waypoint carries the same attached bodies and the same world, for
  /// the reason `isStateValid` gives at its own doc: attachments here live on
  /// the state, and a waypoint state is a copy of `*state_` with only the
  /// named joint variables overwritten.
  json pathCostSources(const json& request)
  {
    planning_scene::PlanningScene scene(model_);
    addRequestObjects(*scene.getWorldNonConst(), request);

    applyJointValues(request);
    applyAttachedBodies(*state_, request);

    robot_trajectory::RobotTrajectory trajectory(model_);
    for (const auto& wp_json : request.at("waypoints"))
    {
      moveit::core::RobotState waypoint_state(*state_);
      for (auto it = wp_json.begin(); it != wp_json.end(); ++it)
      {
        if (!hasVariable(it.key()))
          throw std::runtime_error("unknown joint variable: " + it.key());
        waypoint_state.setVariablePosition(it.key(), it.value().get<double>());
      }
      waypoint_state.update();
      trajectory.addSuffixWayPoint(waypoint_state, 0.1);
    }

    const auto max_costs = request.at("max_costs").get<std::size_t>();
    const std::string group_name = request.value("group_name", std::string());
    const double overlap_fraction = request.at("overlap_fraction").get<double>();

    std::set<collision_detection::CostSource> costs;
    scene.getCostSources(trajectory, max_costs, group_name, costs, overlap_fraction);

    return json{ { "cost_sources", costSourcesToJson(costs) } };
  }

  /// Ground truth for `PlanningScene::is_state_valid`/`is_state_constrained`/
  /// `is_path_valid` (`cspace-scene`). Builds a real
  /// `planning_scene::PlanningScene` the same way `frameTransform` does, so
  /// the FCL-backed collision env its own constructor wires up by default
  /// (`planning_scene.cpp:200`, `allocateCollisionDetector
  /// (CollisionDetectorAllocatorFCL::create())`) is what actually answers
  /// every collision check here -- not a hand-rolled `CollisionEnvFCL` like
  /// `collision()` builds. `objects`/`attached_bodies` are fed in with the
  /// same shapes those two ops already use.
  ///
  /// One request checks a whole *path*, not just one state: `waypoints`
  /// (one entry means plain `isStateValid`), `path_constraints` applied to
  /// every waypoint, `goal_constraints` (checked disjunctively, one
  /// `isStateConstrained` per entry) applied only to the last. This calls
  /// the real `PlanningScene::isPathValid(trajectory, path_constraints,
  /// goal_constraints, group, verbose, invalid_index)` end to end
  /// (`planning_scene.cpp:2365`) rather than re-deriving its per-waypoint
  /// loop here, so what this returns is upstream's own composition of
  /// collision/feasibility/constraints, not this port's understanding of
  /// it. `group` is always `""` (whole robot, `RobotTrajectory`'s own
  /// single-argument constructor -- no `JointModelGroup` at all): this
  /// port's `ParryCollisionEnv` never reads `CollisionRequest::group_name`
  /// (see `cspace-collision`'s `colliding_pairs` deviation doc), so
  /// requesting anything narrower here would make the two sides check
  /// different geometry for a reason unrelated to what this op tests. The
  /// feasibility predicate (`state_feasibility_`) is never registered on
  /// `scene`, matching this port's decision not to carry a caller-supplied
  /// predicate at all (see `cspace-scene::PlanningScene::is_state_valid`'s
  /// own doc) -- both sides therefore always take upstream's "no predicate
  /// registered -> true" branch.
  ///
  /// Every waypoint gets the *same* attached bodies: this port's
  /// `AttachedBody` lives on `PlanningScene`, not per-`RobotState` the way
  /// upstream's does (see `attached_body.rs`'s module doc), so a waypoint
  /// state here is built by copy-constructing `scene`'s current state
  /// (attached bodies and all) and then overwriting only the joint
  /// variables named in that waypoint -- upstream's own
  /// `RobotState::setToDefaultValues` never touches the attached-body list,
  /// so this carries the same bodies through every waypoint without
  /// replaying `attachBody` once per waypoint.
  ///
  /// `invalid_index` is returned verbatim (as `invalid_waypoints`) rather
  /// than deduplicated here: upstream can push the same waypoint index
  /// twice (once for state validity, once for an unmet goal, when both
  /// fail on the last waypoint) -- that raw shape is preserved so a
  /// disagreement in *how many* times an index was pushed is visible to
  /// the Rust side rather than silently absorbed on this side.
  json isStateValid(const json& request)
  {
    planning_scene::PlanningScene scene(model_);

    scene.getCurrentStateNonConst().clearAttachedBodies();
    applyAttachedBodies(scene.getCurrentStateNonConst(), request);

    addRequestObjects(*scene.getWorldNonConst(), request);

    const moveit_msgs::msg::Constraints path_constraints_msg =
        constraintsMsgFromJson(request.value("path_constraints", json::object()));

    std::vector<moveit_msgs::msg::Constraints> goal_constraints_msg;
    for (const auto& gc_json : request.value("goal_constraints", json::array()))
      goal_constraints_msg.push_back(constraintsMsgFromJson(gc_json));

    robot_trajectory::RobotTrajectory trajectory(model_);
    for (const auto& wp_json : request.at("waypoints"))
    {
      moveit::core::RobotState waypoint_state(scene.getCurrentState());
      waypoint_state.setToDefaultValues();
      for (auto it = wp_json.begin(); it != wp_json.end(); ++it)
      {
        if (!hasVariable(it.key()))
          throw std::runtime_error("unknown joint variable: " + it.key());
        waypoint_state.setVariablePosition(it.key(), it.value().get<double>());
      }
      waypoint_state.update();
      trajectory.addSuffixWayPoint(waypoint_state, 0.1);
    }

    std::vector<std::size_t> invalid_index;
    const bool valid =
        scene.isPathValid(trajectory, path_constraints_msg, goal_constraints_msg, "", true, &invalid_index);

    json invalid_out = json::array();
    for (std::size_t idx : invalid_index)
      invalid_out.push_back(idx);

    return json{ { "valid", valid }, { "invalid_waypoints", invalid_out } };
  }

  /// Ground truth for `cspace-scene`'s diff layer -- PORTING-PLAN.md §5's
  /// third Phase 5 completion condition, "씬 diff 적용 후 충돌 결과가 오라클과
  /// 100% 일치".
  ///
  /// Builds a base `planning_scene::PlanningScene` from the request the same
  /// way `isStateValid` does (`applyJointValuesTo`/`applyAttachedBodies`/
  /// `addRequestObjects` -- the shared parsers, not a fourth copy), collision-
  /// checks it, calls upstream `PlanningScene::diff()` for a *real* child
  /// scene, applies `request["diff"]`'s actions to that child, and collision-
  /// checks both the child and the parent again.
  ///
  /// Three summaries, not one, because the property under test is not just
  /// "the child answers X": upstream's diff is copy-on-write over four
  /// separately-inherited layers (`world_` snapshot + `WorldDiff`,
  /// `robot_state_`, `acm_`, transforms -- `planning_scene.cpp:203-227` and
  /// the `getCurrentStateNonConst`/`getAllowedCollisionMatrixNonConst`
  /// copy-on-write pair at `:625`/`:659`), and this port mirrors it with
  /// `Layered::Inherited` (`crates/cspace-planning/src/scene/scene.rs`'s `diff()`). A
  /// child-only response could not tell a correct diff from one that mutated
  /// its parent, which is the failure mode copy-on-write exists to prevent;
  /// `parent_before` and `parent_after` make that observable, and the two
  /// being equal is itself part of what the Rust side asserts.
  ///
  /// The child's checks go through the child's *own* `getCollisionEnv()`,
  /// which `allocateCollisionDetector(parent->alloc_, parent->detector_)`
  /// built on the child's own `world_` (`planning_scene.cpp:255-282`), so a
  /// world edit made on the child is seen by the child's env and by nothing
  /// else. Hand-rolling a `CollisionEnvFCL` per summary the way `collision()`
  /// does would bypass exactly the wiring under test.
  ///
  /// `self_*` is reported alongside `robot_*` but see
  /// `crates/cspace-planning/tests/scene_diff_collision_parity.rs` for why only
  /// the latter is compared against this port on pr2.
  json sceneDiffCollision(const json& request)
  {
    auto parent = std::make_shared<planning_scene::PlanningScene>(model_);
    applyJointValuesTo(parent->getCurrentStateNonConst(), request);
    applyAttachedBodies(parent->getCurrentStateNonConst(), request);
    for (const auto& object_json : request.value("objects", json::array()))
      addSceneObject(*parent->getWorldNonConst(), object_json);

    json parent_before = sceneCollisionSummary(*parent);

    planning_scene::PlanningScenePtr child = parent->diff();
    applySceneDiff(*child, request.at("diff"));

    json child_summary = sceneCollisionSummary(*child);
    json parent_after = sceneCollisionSummary(*parent);

    return json{
      { "parent_before", parent_before }, { "child", child_summary }, { "parent_after", parent_after }
    };
  }

  /// Apply one `request["diff"]` array to `scene`: the five diff kinds Phase
  /// 5's completion condition names -- a world object added, a world object
  /// removed, a body attached to a link, a body detached, and an ACM entry
  /// changed.
  ///
  /// `attach` is routed through `applyAttachedBodies` with a one-element
  /// array rather than calling `attachBody` again here: an attachment made by
  /// a diff must be built exactly the way the base scene's are (identity
  /// `pose`, `touch_links`, `subframes`), and that helper's own doc records
  /// three hand-written copies of that code having already drifted apart.
  /// It is `attachBody` alone, touching neither the world nor the ACM, which
  /// is upstream's ADD branch for message-carried shapes and this port's
  /// `PlanningScene::attach_new`.
  ///
  /// `remove_object` and `detach` are *not* the bare `World`/`RobotState`
  /// calls their names suggest, because the methods this op exists to check
  /// are not bare either. `remove_object` prunes the id's ACM entry
  /// (`processCollisionObjectRemove`, `planning_scene.cpp:1473`; this port's
  /// `PlanningScene::remove_object`), and `detach` puts the body's geometry
  /// back into the world at its global pose, subframes and all, *before*
  /// clearing it off the state (the REMOVE branch at
  /// `planning_scene.cpp:1728-1755`; this port's `PlanningScene::detach`).
  /// A `clearAttachedBody`-only detach here would have made this op agree
  /// with a Rust side that did the same thing and with nothing else -- the
  /// two would have matched each other while both diverged from upstream.
  ///
  /// `remove_object`/`detach` throw on an absent id rather than returning the
  /// upstream `false` silently. Both are the point of their case: a fixture
  /// naming an id the base scene never had would otherwise capture "removed
  /// nothing" as ground truth, and a Rust side that also removed nothing
  /// would agree with it. `detach` throws on a world id collision too, where
  /// upstream warns and drops the geometry -- this port's `detach` returns an
  /// error there for the same reason (see its doc), so a fixture cannot
  /// capture the silent-loss path as if it were the normal one.
  void applySceneDiff(planning_scene::PlanningScene& scene, const json& diff)
  {
    for (const auto& action_json : diff)
    {
      const std::string action = action_json.at("action").get<std::string>();
      if (action == "add_object")
      {
        addSceneObject(*scene.getWorldNonConst(), action_json);
      }
      else if (action == "remove_object")
      {
        const std::string id = action_json.at("id").get<std::string>();
        if (!scene.getWorldNonConst()->removeObject(id))
          throw std::runtime_error("remove_object: no such world object: " + id);
        scene.getAllowedCollisionMatrixNonConst().removeEntry(id);
      }
      else if (action == "attach")
      {
        const json as_request = json{ { "attached_bodies", json::array({ action_json }) } };
        applyAttachedBodies(scene.getCurrentStateNonConst(), as_request);
      }
      else if (action == "detach")
      {
        const std::string id = action_json.at("id").get<std::string>();
        const moveit::core::AttachedBody* body = scene.getCurrentStateNonConst().getAttachedBody(id);
        if (!body)
          throw std::runtime_error("detach: no such attached body: " + id);
        if (scene.getWorldNonConst()->hasObject(id))
          throw std::runtime_error("detach: the world already has an object named " + id);
        scene.getWorldNonConst()->addToObject(id, body->getGlobalPose(), body->getShapes(), body->getShapePoses());
        scene.getWorldNonConst()->setSubframesOfObject(id, body->getSubframes());
        scene.getCurrentStateNonConst().clearAttachedBody(id);
        scene.getCurrentStateNonConst().update();
      }
      else if (action == "set_acm_entry")
      {
        scene.getAllowedCollisionMatrixNonConst().setEntry(action_json.at("first").get<std::string>(),
                                                           action_json.at("second").get<std::string>(),
                                                           action_json.at("allowed").get<bool>());
      }
      else
      {
        throw std::runtime_error("unsupported scene diff action: " + action);
      }
    }
  }

  /// One `{id, pose, shape}` object into `world`, at `addToObject(id, shape,
  /// shape_pose)` -- the *three*-argument overload, object pose identity and
  /// `pose` carried on the shape.
  ///
  /// `addRequestObjects` uses the four-argument one instead (object pose
  /// `pose`, shape pose identity), and for an id the world does not yet have
  /// the two are the same geometry, which is why four ops share it happily.
  /// They stop being the same the moment an id is added *twice*: upstream
  /// discards the `pose` argument entirely when the object already exists
  /// (`world.cpp`'s `addToObject`, `obj->pose_ = pose` sits inside the
  /// `if (!obj)` branch), so the second call's shape lands at the object's
  /// original pose in one spelling and at `pose` relative to it in the other.
  /// This op's `add_object` diff action does exactly that, and this port's
  /// `PlanningScene::add_shape` is the three-argument overload -- so both the
  /// base world and the diff are built that way here rather than reusing the
  /// shared helper and comparing two different placements.
  void addSceneObject(collision_detection::World& world, const json& object_json)
  {
    const std::string id = object_json.at("id").get<std::string>();
    const Eigen::Isometry3d pose = fromRowMajor4x4(object_json.at("pose"));
    const json& shape_json = object_json.at("shape");
    std::shared_ptr<shapes::Shape> shape = parseShape(shape_json.at("type").get<std::string>(), shape_json);
    world.addToObject(id, Eigen::Isometry3d::Identity(), { shape }, { pose });
  }

  /// One `PlanningScene`'s collision answer plus the two id lists that say
  /// which layer a diff actually landed on.
  ///
  /// The ids are not decoration: "world object added" placed clear of the
  /// robot changes no boolean and no distance, so a response carrying only
  /// the four collision numbers could not distinguish that diff from one that
  /// did nothing at all. `world_object_ids` and `attached_body_ids` come from
  /// `std::map`-backed containers on both sides (`World::objects_`,
  /// `RobotState::attached_body_map_`), so their order is sorted and stable
  /// rather than insertion-dependent.
  ///
  /// Same `CollisionRequest`/`DistanceRequest` settings as `collision()`:
  /// `contacts`/`max_contacts` raised so a result is diagnosable, and
  /// `enable_signed_distance = true` so a penetration reports its depth
  /// instead of clamping to zero (see `collision()`'s doc for the full
  /// reasoning). `contacts` themselves are not reported here -- the diff
  /// layer is what is under test, and `collision()` already carries the
  /// contact-level comparison.
  json sceneCollisionSummary(planning_scene::PlanningScene& scene) const
  {
    const moveit::core::RobotState& state = scene.getCurrentState();
    const collision_detection::AllowedCollisionMatrix& acm = scene.getAllowedCollisionMatrix();
    const collision_detection::CollisionEnvConstPtr& env = scene.getCollisionEnv();

    collision_detection::CollisionRequest self_req;
    self_req.contacts = true;
    self_req.max_contacts = 100;
    collision_detection::CollisionResult self_res;
    env->checkSelfCollision(self_req, self_res, state, acm);

    collision_detection::CollisionRequest robot_req;
    robot_req.contacts = true;
    robot_req.max_contacts = 100;
    collision_detection::CollisionResult robot_res;
    env->checkRobotCollision(robot_req, robot_res, state, acm);

    collision_detection::DistanceRequest self_dreq;
    self_dreq.enable_signed_distance = true;
    self_dreq.acm = &acm;
    collision_detection::DistanceResult self_dres;
    env->distanceSelf(self_dreq, self_dres, state);

    collision_detection::DistanceRequest robot_dreq;
    robot_dreq.enable_signed_distance = true;
    robot_dreq.acm = &acm;
    collision_detection::DistanceResult robot_dres;
    env->distanceRobot(robot_dreq, robot_dres, state);

    json object_ids = json::array();
    for (const std::string& id : scene.getWorld()->getObjectIds())
      object_ids.push_back(id);

    std::vector<const moveit::core::AttachedBody*> bodies;
    state.getAttachedBodies(bodies);
    json attached_ids = json::array();
    for (const moveit::core::AttachedBody* body : bodies)
      attached_ids.push_back(body->getName());

    return json{
      { "self_collision", self_res.collision },
      { "self_distance", self_dres.minimum_distance.distance },
      { "robot_collision", robot_res.collision },
      { "robot_distance", robot_dres.minimum_distance.distance },
      { "world_object_ids", object_ids },
      { "attached_body_ids", attached_ids },
    };
  }

  /// Ground truth for the `cspace-collision` World port. Builds a
  /// `collision_detection::World` straight from the request -- World has no
  /// RobotModel dependency, so `model_`/`state_` are untouched here -- one
  /// object per `request["objects"]` entry (a dummy 0.1m sphere per shape
  /// pose, since only pose composition is under test, not shape geometry),
  /// then dumps every object's pose, per-shape pose/global pose and
  /// per-subframe pose/global pose. `request["queries"]` is answered with
  /// both `knowsTransform` and `getTransform`: the two are deliberately
  /// queried against the same real, unmodified upstream methods so a name
  /// where they disagree (a subframe name colliding with a sibling object's
  /// name -- see world.rs's module docs) is observed from upstream directly,
  /// not re-derived from a Rust reading of world.cpp.
  json world(const json& request) const
  {
    collision_detection::World w;

    for (const auto& object_json : request.at("objects"))
    {
      const std::string id = object_json.at("id").get<std::string>();
      const Eigen::Isometry3d pose = fromRowMajor4x4(object_json.at("pose"));

      std::vector<shapes::ShapeConstPtr> shape_ptrs;
      EigenSTL::vector_Isometry3d shape_poses;
      for (const auto& shape_pose_json : object_json.at("shape_poses"))
      {
        shape_ptrs.push_back(std::make_shared<shapes::Sphere>(0.1));
        shape_poses.push_back(fromRowMajor4x4(shape_pose_json));
      }
      if (!shape_ptrs.empty())
        w.addToObject(id, pose, shape_ptrs, shape_poses);
      else
        w.setObjectPose(id, pose);

      if (object_json.contains("subframes") && !object_json.at("subframes").empty())
      {
        moveit::core::FixedTransformsMap subframes;
        for (auto it = object_json.at("subframes").begin(); it != object_json.at("subframes").end(); ++it)
          subframes[it.key()] = fromRowMajor4x4(it.value());
        w.setSubframesOfObject(id, subframes);
      }
    }

    json objects_out = json::array();
    for (const std::string& id : w.getObjectIds())
    {
      collision_detection::World::ObjectConstPtr obj = w.getObject(id);
      const EigenSTL::vector_Isometry3d& global_shape_poses = w.getGlobalShapeTransforms(id);

      json shapes_out = json::array();
      for (std::size_t i = 0; i < obj->shape_poses_.size(); ++i)
      {
        shapes_out.push_back(json{ { "pose", toRowMajor4x4(obj->shape_poses_[i]) },
                                    { "global_pose", toRowMajor4x4(global_shape_poses[i]) } });
      }

      json subframes_out = json::object();
      for (const auto& [name, subframe_pose] : obj->subframe_poses_)
      {
        subframes_out[name] = json{ { "pose", toRowMajor4x4(subframe_pose) },
                                     { "global_pose", toRowMajor4x4(obj->global_subframe_poses_.at(name)) } };
      }

      objects_out.push_back(json{ { "id", id },
                                   { "pose", toRowMajor4x4(obj->pose_) },
                                   { "shapes", shapes_out },
                                   { "subframes", subframes_out } });
    }

    json queries_out = json::array();
    if (request.contains("queries"))
    {
      for (const auto& query_json : request.at("queries"))
      {
        const std::string name = query_json.get<std::string>();
        bool frame_found = false;
        const Eigen::Isometry3d& transform = w.getTransform(name, frame_found);
        queries_out.push_back(json{ { "name", name },
                                     { "knows_transform", w.knowsTransform(name) },
                                     { "transform", frame_found ? json(toRowMajor4x4(transform)) : json(nullptr) } });
      }
    }

    return json{ { "objects", objects_out }, { "queries", queries_out } };
  }

  /// Ground truth for the `cspace-distance-field` `PropagationDistanceField`
  /// port. Builds a field straight from `geometry`/`max_distance`/
  /// `propagate_negative` -- no `RobotModel` involved, `distance_field` has
  /// none either -- adds `occupied_cells` (explicit integer grid coordinates,
  /// converted to world points via `gridToWorld` so the obstacle set the
  /// field actually stores is exactly the requested cells with no separate
  /// shape-sampling path to drift from it), then for every cell in `queries`
  /// dumps `getDistance` (both the world-point and, when the cell is
  /// in-grid, the cell-indexed overload), `getDistanceGradient`, and, when
  /// the cell is in-grid, `getNearestCell`.
  ///
  /// `getDistance(int,int,int)` and `getNearestCell` are documented upstream
  /// as needing a valid cell "or corruption occurs" -- they are only called
  /// for `in_grid` cells, matching the same guard `PropagationDistanceField`
  /// callers elsewhere in this port already apply. `getDistance(double,...)`
  /// and `getDistanceGradient` handle an out-of-grid world point themselves
  /// (`VoxelGrid::operator()` / the `gx < 1 || ...` bounds check), so those
  /// two are safe to call unconditionally and are dumped for every query.
  ///
  /// `nearest.voxel_present` deliberately does not dump the neighbor voxel's
  /// own fields: for a query that reaches `getNearestCell`'s
  /// `PropDistanceFieldVoxel::UNINITIALIZED` (`-1,-1,-1`) sentinel path --
  /// a cell farther than `max_distance` from every obstacle, never visited
  /// by propagation -- upstream reads `voxel_grid_->getCell(-1, -1, -1)`
  /// unguarded and returns that address as a non-null pointer (see
  /// `propagation.rs`'s `nearest_cell` deviation doc). The pointer itself is
  /// well-defined to *form* (pure address arithmetic on `T* data_`, never
  /// dereferenced here or by upstream's own `ncell == cell` comparison); only
  /// reading through it would be memory-unsafe, so this dump reports
  /// presence/absence and never a field of the pointee. That non-null result
  /// for an unvisited cell -- not a crash -- is the empirical evidence this
  /// fixture exists to capture: it is what upstream actually returns where
  /// this crate's `nearest_cell` instead reports `voxel: None`.
  json distanceField(const json& request) const
  {
    const json& geom = request.at("geometry");
    const auto size = geom.at("size").get<std::array<double, 3>>();
    const auto origin = geom.at("origin").get<std::array<double, 3>>();
    const double resolution = geom.at("resolution").get<double>();
    const double max_distance = request.at("max_distance").get<double>();
    const bool propagate_negative = request.at("propagate_negative").get<bool>();

    distance_field::PropagationDistanceField field(size[0], size[1], size[2], resolution, origin[0], origin[1],
                                                    origin[2], max_distance, propagate_negative);

    EigenSTL::vector_Vector3d occupied_points;
    for (const auto& cell_json : request.at("occupied_cells"))
    {
      const auto cell = cell_json.get<std::array<int, 3>>();
      double wx = NAN;
      double wy = NAN;
      double wz = NAN;
      field.gridToWorld(cell[0], cell[1], cell[2], wx, wy, wz);
      occupied_points.emplace_back(wx, wy, wz);
    }
    field.addPointsToField(occupied_points);

    json queries_out = json::array();
    for (const auto& query_json : request.at("queries"))
    {
      const auto cell = query_json.get<std::array<int, 3>>();
      const int x = cell[0];
      const int y = cell[1];
      const int z = cell[2];
      const bool in_grid = field.isCellValid(x, y, z);

      double wx = NAN;
      double wy = NAN;
      double wz = NAN;
      field.gridToWorld(x, y, z, wx, wy, wz);

      double gradient_x = NAN;
      double gradient_y = NAN;
      double gradient_z = NAN;
      bool in_bounds = false;
      const double gradient_distance = field.getDistanceGradient(wx, wy, wz, gradient_x, gradient_y, gradient_z, in_bounds);

      json entry;
      entry["cell"] = json::array({ x, y, z });
      entry["in_grid"] = in_grid;
      entry["world"] = json::array({ wx, wy, wz });
      entry["distance_world"] = field.getDistance(wx, wy, wz);
      entry["gradient"] = json{ { "distance", gradient_distance },
                                 { "gradient", json::array({ gradient_x, gradient_y, gradient_z }) },
                                 { "in_bounds", in_bounds } };

      if (in_grid)
      {
        entry["distance_cell"] = field.getDistance(x, y, z);

        double nearest_distance = NAN;
        Eigen::Vector3i nearest_pos;
        const distance_field::PropDistanceFieldVoxel* nearest =
            field.getNearestCell(x, y, z, nearest_distance, nearest_pos);
        entry["nearest"] = json{ { "distance", nearest_distance },
                                  { "position", json::array({ nearest_pos.x(), nearest_pos.y(), nearest_pos.z() }) },
                                  { "voxel_present", nearest != nullptr } };
      }
      else
      {
        entry["distance_cell"] = nullptr;
        entry["nearest"] = nullptr;
      }

      queries_out.push_back(entry);
    }

    return json{ { "queries", queries_out } };
  }

  /// Ground truth for `DistanceField::getOcTreePoints`
  /// (`distance_field.cpp:240-283`), the leaf-subdivision loop this port
  /// reimplements as `distance_field.rs`'s `octree_points`. Requested by
  /// p3-distance-field: the port's own tests can only show what the port
  /// does, and the open question is a cross-language one -- for a leaf
  /// larger than the field resolution, upstream walks
  /// `for (double x = getX() - ceil_val; x <= getX() + ceil_val; x +=
  /// resolution_)` on each axis, accumulating the loop variable by repeated
  /// addition while comparing against a separately-rounded bound, so whether
  /// the last face survives depends on floating-point accumulation that a
  /// C++ compiler is free to contract (FMA) differently from Rust.
  ///
  /// `getOcTreePoints` is `protected` (`distance_field.hpp:587`) and derives
  /// its bounding box from the field's own grid extent rather than taking
  /// one, so this op builds a real `PropagationDistanceField` with the
  /// requested geometry and reaches the method through a derived class --
  /// not through `addOcTreeToField`, whose `addPointsToField` step would
  /// collapse several points into one cell and hide exactly the per-point
  /// count this op exists to compare.
  ///
  /// Deliberately dumps every point in emission order, not just the count:
  /// two implementations can agree on how many points they produce and still
  /// disagree on which, and the count alone would not distinguish "the last
  /// face is missing" from "the whole grid is shifted by one step".
  ///
  /// An optional `bbx: {min: [x,y,z], max: [x,y,z]}` adds a `leaves_bbx`
  /// array in the same `{coordinate, size, occupied}` shape as `leaves`,
  /// walked with `begin_leafs_bbx(point3d, point3d)` /
  /// `end_leafs_bbx()` (`octomap/OcTreeBaseImpl.h:337-345`, public).
  /// Requested by p3-shapes. `leaves` above walks `leaf_iterator`, which is
  /// a *different* upstream class from `leaf_bbx_iterator`: pinning one says
  /// nothing about the other's emission order, exactly the reasoning
  /// `leaves_parity.rs` used to justify measuring `leaf_iterator` instead of
  /// inferring it from `tree_iterator`. The port's only consumer of
  /// `leaves_in_bbx` reads three of `Leaf`'s eight accessors, so this op
  /// emits the same three -- `key`/`index_key`/`depth`/`log_odds`/
  /// `occupancy` have no consumer in any crate and no fixture here would
  /// have a reader to compare against. That is a statement about today's
  /// consumers rather than about the accessors, and it expires the first
  /// time any crate reads one of the five -- at which point this op has to
  /// start emitting it.
  json octreePoints(const json& request) const
  {
    struct PointExposer : distance_field::PropagationDistanceField
    {
      using PropagationDistanceField::PropagationDistanceField;
      using DistanceField::getOcTreePoints;
    };

    const json& geom = request.at("geometry");
    const auto size = geom.at("size").get<std::array<double, 3>>();
    const auto origin = geom.at("origin").get<std::array<double, 3>>();
    const double resolution = geom.at("resolution").get<double>();

    octomap::OcTree tree(request.at("octree_resolution").get<double>());
    applyOctomapActions(tree, request.at("actions"));

    const PointExposer field(size[0], size[1], size[2], resolution, origin[0], origin[1], origin[2],
                             /*max_distance=*/resolution, /*propagate_negative=*/false);

    EigenSTL::vector_Vector3d points;
    field.getOcTreePoints(&tree, &points);

    json points_out = json::array();
    for (const Eigen::Vector3d& point : points)
      points_out.push_back(json::array({ point.x(), point.y(), point.z() }));

    json leaves_out = json::array();
    for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it)
    {
      leaves_out.push_back(json{ { "coordinate", json::array({ it.getX(), it.getY(), it.getZ() }) },
                                 { "size", it.getSize() },
                                 { "occupied", tree.isNodeOccupied(*it) } });
    }

    json out{ { "points", points_out }, { "count", points.size() }, { "leaves", leaves_out } };

    if (request.contains("bbx"))
    {
      const json& bbx = request.at("bbx");
      const auto bbx_min = bbx.at("min").get<std::array<double, 3>>();
      const auto bbx_max = bbx.at("max").get<std::array<double, 3>>();
      const octomap::point3d min(bbx_min[0], bbx_min[1], bbx_min[2]);
      const octomap::point3d max(bbx_max[0], bbx_max[1], bbx_max[2]);

      json bbx_leaves_out = json::array();
      for (octomap::OcTree::leaf_bbx_iterator it = tree.begin_leafs_bbx(min, max), end = tree.end_leafs_bbx();
           it != end; ++it)
      {
        bbx_leaves_out.push_back(json{ { "coordinate", json::array({ it.getX(), it.getY(), it.getZ() }) },
                                       { "size", it.getSize() },
                                       { "occupied", tree.isNodeOccupied(*it) } });
      }
      out["leaves_bbx"] = bbx_leaves_out;
    }

    return out;
  }

  /// Ground truth for `find_internal_points_convex`
  /// (`distance_field::findInternalPointsConvex`), the piece of
  /// `cspace-distance-field`'s shape-to-obstacle-points path that the
  /// `distance_field` op above does not exercise: that op takes
  /// `occupied_cells` as an explicit input, starting only after this step.
  /// This mirrors upstream `DistanceField::getShapePoints` exactly: builds
  /// the `bodies::Body` the same way (`createEmptyBodyFromShapeType` +
  /// `setDimensionsDirty` + `setPoseDirty` + `updateInternalData`, never
  /// touching `setScale`/`setPadding` -- see `posed_body` in
  /// `distance_field.rs` for why this port's own construction hard-codes
  /// scale 1.0/padding 0.0 to match), then calls `findInternalPointsConvex`
  /// directly and dumps the resulting point list. `request["shape"]["type"]`
  /// is one of `"sphere"`, `"box"`, `"cylinder"`, `"mesh"` -- the four
  /// `bodies::` has a case for in `createEmptyBodyFromShapeType`.
  json shapePoints(const json& request) const
  {
    const json& shape_json = request.at("shape");
    const std::string type = shape_json.at("type").get<std::string>();
    const Eigen::Isometry3d pose = fromRowMajor4x4(request.at("pose"));
    const double resolution = request.at("resolution").get<double>();

    std::shared_ptr<shapes::Shape> shape = parseShape(type, shape_json);
    requireBodySupport(shape->type, "shape_points");

    bodies::Body* body = bodies::createEmptyBodyFromShapeType(shape->type);
    body->setDimensionsDirty(shape.get());
    body->setPoseDirty(pose);
    body->updateInternalData();

    EigenSTL::vector_Vector3d points;
    distance_field::findInternalPointsConvex(*body, resolution, points);
    delete body;

    json points_out = json::array();
    for (const Eigen::Vector3d& p : points)
      points_out.push_back(json::array({ p.x(), p.y(), p.z() }));

    return json{ { "points", points_out } };
  }

  /// Ground truth for the `cspace-geometry` STL loader
  /// (`crates/cspace-core/src/geometry/stl.rs`). Calls
  /// `shapes::createMeshFromResource` exactly as `RobotModel::constructShape`
  /// does for a URDF `<mesh>` element -- real Assimp, real
  /// `aiProcess_JoinIdenticalVertices` merging, real `package://` resolution
  /// against this image's colcon-built `moveit_resources` workspace -- so a
  /// resource path and scale go in and the vertex/triangle counts, vertex
  /// positions and triangle indices Assimp actually produced come out, with
  /// no `RobotModel` construction involved on either side of this op.
  ///
  /// `triangles` is emitted because vertex data alone cannot settle what it
  /// looked like it settled. `collision_common.cpp:902-920` builds the FCL
  /// mesh as `BVHModel<OBBRSSd>` from *both* arrays -- `points[i]` in the
  /// order `shapes::Mesh` stores them, and `tri_indices` indexing into that
  /// order -- so two meshes agreeing on the vertex set, the vertex count and
  /// even the vertex order can still build different BVHs, traverse in a
  /// different order, and report a different deepest point. That is exactly
  /// the residual `parry.rs`'s deviation 6 documents. The vertex order half
  /// is already measured and agrees (36 of 36 meshes, see PORTING-PLAN.md
  /// §164); the triangle half had no ground truth to measure against at all
  /// until this field.
  json meshOp(const json& request) const
  {
    const std::string resource = request.at("resource").get<std::string>();
    const auto scale_json = request.at("scale").get<std::array<double, 3>>();
    const Eigen::Vector3d scale(scale_json[0], scale_json[1], scale_json[2]);

    std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(resource, scale));
    if (!mesh)
      throw std::runtime_error("failed to load mesh resource " + resource);

    json vertices_out = json::array();
    for (unsigned int i = 0; i < mesh->vertex_count; ++i)
    {
      vertices_out.push_back(json::array({ mesh->vertices[3 * i], mesh->vertices[3 * i + 1],
                                            mesh->vertices[3 * i + 2] }));
    }

    json triangles_out = json::array();
    for (unsigned int i = 0; i < mesh->triangle_count; ++i)
    {
      triangles_out.push_back(json::array(
          { mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2] }));
    }

    return json{ { "vertex_count", mesh->vertex_count },
                 { "triangle_count", mesh->triangle_count },
                 { "vertices", vertices_out },
                 { "triangles", triangles_out } };
  }

  /// Ground truth for `collision_distance_field_types.{hpp,cpp}`: the two
  /// pieces `shapePoints`/`distanceField` above do not exercise --
  /// `BodyDecomposition`'s sphere decomposition
  /// (`determineCollisionSpheres` + `computeBoundingSphere`) and
  /// `PosedDistanceField`'s pose-transform order, both directly and through
  /// `getCollisionSphereGradients`.
  ///
  /// `request["shape_pose"]` poses the shape `BodyDecomposition` is built
  /// from -- note this uses the *vector* constructor
  /// (`BodyDecomposition(shapes, poses, resolution, padding)`), not the
  /// single-shape one, because upstream's single-shape constructor always
  /// passes `Eigen::Isometry3d::Identity()` for the pose regardless of what
  /// the caller wants (see `collision_distance_field_types.cpp`), which
  /// would make a non-identity `shape_pose` untestable through it.
  ///
  /// The collision-sphere-gradient integration poses each decomposed
  /// sphere's `relative_vec_` by that same `shape_pose` before querying --
  /// matching what a real caller does through
  /// `PosedBodySphereDecomposition::updatePose(shape_pose)`, without
  /// depending on that (unported) type here.
  json collisionDistanceFieldTypes(const json& request) const
  {
    const json& shape_json = request.at("shape");
    const std::string type = shape_json.at("type").get<std::string>();
    const Eigen::Isometry3d shape_pose = fromRowMajor4x4(request.at("shape_pose"));
    const double resolution = request.at("resolution").get<double>();
    const double padding = request.at("padding").get<double>();

    shapes::ShapeConstPtr shape = parseShape(type, shape_json);
    requireBodySupport(shape->type, "collision_distance_field_types");
    std::vector<shapes::ShapeConstPtr> shapes_vec{ shape };
    EigenSTL::vector_Isometry3d poses_vec{ shape_pose };
    collision_detection::BodyDecomposition body_decomp(shapes_vec, poses_vec, resolution, padding);

    json spheres_out = json::array();
    for (const collision_detection::CollisionSphere& cs : body_decomp.getCollisionSpheres())
    {
      spheres_out.push_back(json{ { "relative_vec", json::array({ cs.relative_vec_.x(), cs.relative_vec_.y(),
                                                                    cs.relative_vec_.z() }) },
                                   { "radius", cs.radius_ } });
    }

    json out;
    out["collision_spheres"] = spheres_out;
    out["relative_cylinder_pose"] = toRowMajor4x4(body_decomp.getRelativeCylinderPose());
    out["bounding_sphere"] = json{
      { "center", json::array({ body_decomp.getRelativeBoundingSphere().center.x(),
                                 body_decomp.getRelativeBoundingSphere().center.y(),
                                 body_decomp.getRelativeBoundingSphere().center.z() }) },
      { "radius", body_decomp.getRelativeBoundingSphere().radius }
    };

    const json& field_json = request.at("posed_field");
    const json& geom = field_json.at("geometry");
    const auto size = geom.at("size").get<std::array<double, 3>>();
    const auto origin = geom.at("origin").get<std::array<double, 3>>();
    const double field_resolution = geom.at("resolution").get<double>();
    const double max_distance = field_json.at("max_distance").get<double>();
    const bool propagate_negative = field_json.at("propagate_negative").get<bool>();
    const Eigen::Isometry3d field_pose = fromRowMajor4x4(field_json.at("field_pose"));

    collision_detection::PosedDistanceField posed_field(Eigen::Vector3d(size[0], size[1], size[2]),
                                                         Eigen::Vector3d(origin[0], origin[1], origin[2]),
                                                         field_resolution, max_distance, propagate_negative);

    EigenSTL::vector_Vector3d occupied_points;
    for (const auto& cell_json : field_json.at("occupied_cells"))
    {
      const auto cell = cell_json.get<std::array<int, 3>>();
      double wx = NAN;
      double wy = NAN;
      double wz = NAN;
      posed_field.gridToWorld(cell[0], cell[1], cell[2], wx, wy, wz);
      occupied_points.emplace_back(wx, wy, wz);
    }
    posed_field.addPointsToField(occupied_points);
    posed_field.updatePose(field_pose);

    json gradients_out = json::array();
    for (const auto& q : request.at("gradient_queries"))
    {
      const auto pt = q.get<std::array<double, 3>>();
      double gx = NAN;
      double gy = NAN;
      double gz = NAN;
      bool in_bounds = false;
      const double dist = posed_field.getDistanceGradient(pt[0], pt[1], pt[2], gx, gy, gz, in_bounds);
      gradients_out.push_back(
          json{ { "distance", dist }, { "gradient", json::array({ gx, gy, gz }) }, { "in_bounds", in_bounds } });
    }
    out["gradients"] = gradients_out;

    EigenSTL::vector_Vector3d sphere_centers;
    for (const collision_detection::CollisionSphere& cs : body_decomp.getCollisionSpheres())
      sphere_centers.push_back(shape_pose * cs.relative_vec_);

    collision_detection::GradientInfo grad_info;
    const std::size_t n_spheres = body_decomp.getCollisionSpheres().size();
    grad_info.distances.assign(n_spheres, DBL_MAX);
    grad_info.types.assign(n_spheres, collision_detection::NONE);
    grad_info.gradients.assign(n_spheres, Eigen::Vector3d::Zero());

    const bool collision = posed_field.getCollisionSphereGradients(
        body_decomp.getCollisionSpheres(), sphere_centers, grad_info, collision_detection::SELF,
        /*tolerance=*/0.0, /*subtract_radii=*/true, /*maximum_value=*/1.0e6, /*stop_at_first_collision=*/false);

    json distances_out = json::array();
    json types_out = json::array();
    json grads_out = json::array();
    for (std::size_t i = 0; i < n_spheres; ++i)
    {
      distances_out.push_back(grad_info.distances[i]);
      types_out.push_back(static_cast<int>(grad_info.types[i]));
      grads_out.push_back(
          json::array({ grad_info.gradients[i].x(), grad_info.gradients[i].y(), grad_info.gradients[i].z() }));
    }
    out["collision_gradient"] = json{ { "closest_distance", grad_info.closest_distance },
                                       { "collision", collision },
                                       { "distances", distances_out },
                                       { "types", types_out },
                                       { "gradients", grads_out } };

    return out;
  }

  /// Ground truth for the three free functions in
  /// `collision_distance_field_types.{hpp,cpp}` that
  /// `collisionDistanceFieldTypes` above never calls: the free
  /// `getCollisionSphereGradients(const distance_field::DistanceField*, ...)`
  /// and both `getCollisionSphereCollision` overloads (bool-only, and the
  /// `num_coll`/`colls` variant). `collisionDistanceFieldTypes` only drives
  /// `PosedDistanceField::getCollisionSphereGradients`, the *member*
  /// overload -- upstream's two `getCollisionSphereGradients` overloads
  /// disagree with each other on two points (the out-of-bounds threshold is
  /// `grad.norm() > 0` on the member vs `grad.norm() > EPSILON` on the free
  /// function; the member `std::abs()`s `dist` after subtracting the sphere
  /// radius, the free function does not), so a fixture that only exercises
  /// the member proves nothing about the free function's own arithmetic.
  ///
  /// Builds a raw `distance_field::PropagationDistanceField` directly (no
  /// `PosedDistanceField`, no pose, no `BodyDecomposition`/shape parsing --
  /// the free functions take a bare `const distance_field::DistanceField*`
  /// and a caller-supplied sphere list already in the field's own frame, so
  /// there is nothing upstream for a pose to apply here), then calls all
  /// three free functions against the same `spheres` list and reports every
  /// result. `request["spheres"]` is deliberately a mix of one sphere deep
  /// inside an occupied cell (negative post-radius-subtraction `dist`,
  /// exercising the abs()-vs-not divergence noted above) and spheres placed
  /// to land on both sides of `maximum_value`/`tolerance`, so `collision`
  /// is `true` for at least one sphere and `false` for at least one other --
  /// a fixture where every sphere trivially misses would leave the
  /// `in_collision = true` branches, and the `num_coll` early-return paths
  /// in the second `getCollisionSphereCollision` overload, unexercised.
  json collisionSphereFreeFunctions(const json& request) const
  {
    const json& geom = request.at("geometry");
    const auto size = geom.at("size").get<std::array<double, 3>>();
    const auto origin = geom.at("origin").get<std::array<double, 3>>();
    const double resolution = geom.at("resolution").get<double>();
    const double max_distance = request.at("max_distance").get<double>();
    const bool propagate_negative = request.at("propagate_negative").get<bool>();

    distance_field::PropagationDistanceField field(size[0], size[1], size[2], resolution, origin[0], origin[1],
                                                    origin[2], max_distance, propagate_negative);

    EigenSTL::vector_Vector3d occupied_points;
    for (const auto& cell_json : request.at("occupied_cells"))
    {
      const auto cell = cell_json.get<std::array<int, 3>>();
      double wx = NAN;
      double wy = NAN;
      double wz = NAN;
      field.gridToWorld(cell[0], cell[1], cell[2], wx, wy, wz);
      occupied_points.emplace_back(wx, wy, wz);
    }
    field.addPointsToField(occupied_points);

    std::vector<collision_detection::CollisionSphere> sphere_list;
    EigenSTL::vector_Vector3d sphere_centers;
    for (const auto& sphere_json : request.at("spheres"))
    {
      const auto center = sphere_json.at("center").get<std::array<double, 3>>();
      const double radius = sphere_json.at("radius").get<double>();
      sphere_list.emplace_back(Eigen::Vector3d(center[0], center[1], center[2]), radius);
      sphere_centers.emplace_back(center[0], center[1], center[2]);
    }

    const double maximum_value = request.at("maximum_value").get<double>();
    const double tolerance = request.at("tolerance").get<double>();
    const bool subtract_radii = request.at("subtract_radii").get<bool>();
    const unsigned int num_coll = request.at("num_coll").get<unsigned int>();

    const distance_field::DistanceField* raw_field = &field;

    collision_detection::GradientInfo grad_info;
    const std::size_t n_spheres = sphere_list.size();
    grad_info.distances.assign(n_spheres, DBL_MAX);
    grad_info.types.assign(n_spheres, collision_detection::NONE);
    grad_info.gradients.assign(n_spheres, Eigen::Vector3d::Zero());

    const bool gradients_collision = collision_detection::getCollisionSphereGradients(
        raw_field, sphere_list, sphere_centers, grad_info, collision_detection::SELF, tolerance, subtract_radii,
        maximum_value, /*stop_at_first_collision=*/false);

    json distances_out = json::array();
    json types_out = json::array();
    json grads_out = json::array();
    for (std::size_t i = 0; i < n_spheres; ++i)
    {
      distances_out.push_back(grad_info.distances[i]);
      types_out.push_back(static_cast<int>(grad_info.types[i]));
      grads_out.push_back(
          json::array({ grad_info.gradients[i].x(), grad_info.gradients[i].y(), grad_info.gradients[i].z() }));
    }

    const bool collision_bool = collision_detection::getCollisionSphereCollision(
        raw_field, sphere_list, sphere_centers, maximum_value, tolerance);

    std::vector<unsigned int> colls;
    const bool collision_with_limit = collision_detection::getCollisionSphereCollision(
        raw_field, sphere_list, sphere_centers, maximum_value, tolerance, num_coll, colls);

    return json{
      { "gradients", json{ { "closest_distance", grad_info.closest_distance },
                            { "collision", gradients_collision },
                            { "distances", distances_out },
                            { "types", types_out },
                            { "gradients", grads_out } } },
      { "collision_bool", collision_bool },
      { "collision_with_limit", json{ { "collision", collision_with_limit }, { "colls", colls } } },
    };
  }

  /// Ground truth for `cspace-distance-field`'s
  /// `generate_distance_field_cache_entry` (upstream
  /// `CollisionEnvDistanceField::generateDistanceFieldCacheEntry`, a
  /// `protected` method). Rather than subclassing to expose it directly,
  /// this drives it through the same public path upstream's own callers
  /// use: `checkSelfCollision(req, res, state, acm)` internally calls
  /// `generateCollisionCheckingStructures` ->
  /// `generateDistanceFieldCacheEntry` and caches the result, which the
  /// public `getLastDistanceFieldEntry()` then returns -- so this exercises
  /// the exact call path `test_collision_distance_field.cpp` itself relies
  /// on, not a synthetic shortcut.
  ///
  /// Every scalar/vector bookkeeping field of `DistanceFieldCacheEntry` is
  /// dumped for byte-exact comparison. `distance_field_` itself is not
  /// dumped cell-by-cell: `PropagationDistanceField`'s own propagation
  /// correctness is independently oracle-verified elsewhere in this
  /// workspace (the `distance_field` op), so what this op actually needs to
  /// prove is narrower -- that `generateDistanceFieldCacheEntry` selects the
  /// right *input* points and wires them into a field at all. `has_field`
  /// answers "did generation run", and `distance_queries` (evaluated
  /// against the real field via `getDistance`) answers "did the selected
  /// points actually land in it" -- both real, if not exhaustive, checks of
  /// the new logic surface, without re-deriving `PropagationDistanceField`
  /// correctness a second time.
  ///
  /// `request["objects"]` (optional, defaults to none) populates the
  /// environment the check runs against, using the same single-shape schema
  /// as `collision` (see `addRequestObjects`). Without it the environment
  /// distance field is empty, so every environment branch -- including the
  /// `"environment"` sentinel contact and `getEnvironmentProximityGradients`
  /// -- is unreachable from a fixture.
  ///
  /// The world-taking constructor is used unconditionally rather than only
  /// when `objects` is non-empty. That it is equivalent for the empty case
  /// is measured, not assumed: all 43 fixture pairs committed at the time of
  /// that change replay identical across it (the corpus has grown since --
  /// this is the measurement's scope, not a running count), and the four
  /// `distance_field_cache_entry`
  /// / `group_state_representation` pairs among them send no `objects` at
  /// all. A conditional would have been a second code path reachable only by
  /// fixtures that do not exist yet.
  ///
  /// `request["attached_bodies"]` (optional, defaults to none) is attached
  /// before the check, so the attached-body half of the group's
  /// `DistanceFieldCacheEntry` is reachable at all -- without it
  /// `attached_body_names_` is always empty and every attached-body branch
  /// below it is untestable.
  ///
  /// It only becomes non-empty with `use_acm` *true*, and that is upstream's
  /// behaviour rather than this op's: the whole attached-body enumeration
  /// sits inside `generateDistanceFieldCacheEntry`'s `if (acm)`
  /// (`collision_env_distance_field.cpp:775`, the pushes at `:801-802`), so a
  /// null ACM leaves `attached_body_names_` empty no matter what is attached
  /// to the state. Measured, not inferred: the same request with `use_acm`
  /// false returns `attached_body_names: []` and with it true returns
  /// `["payload"]`.
  ///
  /// `request["contacts"]` (optional, defaults to `false`) selects upstream's
  /// contact-enumerating path and, when true, adds `collision` and `contacts`
  /// to the response. It is a request field rather than always-on because
  /// `req.contacts` is not an output-only switch upstream: the `true` branch
  /// of `getSelfCollisions` (`collision_env_distance_field.cpp:298-338`)
  /// writes `gsr->gradients_[i].types[col] = SELF` and
  /// `gradients_[i].collision = true` and keeps scanning, while the `false`
  /// branch (`:341-349`) returns on the first colliding link and writes
  /// neither. Those two fields are already dumped by this op, so flipping the
  /// switch would silently change what every existing fixture asserts.
  /// Making it a request field keeps old fixtures byte-identical and puts the
  /// mode in the fixture that asked for it. `max_contacts` /
  /// `max_contacts_per_pair` are exposed for the same reason -- the branch's
  /// scan bound is `std::min(req.max_contacts_per_pair, req.max_contacts -
  /// res.contact_count)`, so a fixture that cannot set them cannot reach the
  /// multi-contact shape at all.
  ///
  /// The difference is measurable rather than argued: one `contacts`-true
  /// request against pr2's `right_arm` reports `collision` on 5 of 22 links'
  /// gradients and `SELF` in one link's `types`, and the identical request
  /// with `contacts` absent reports 0 and 0.
  ///
  /// Two caveats for whoever writes the fixtures. Contact `depth` is always
  /// `0.0` on these paths -- upstream sets only `pos` and the two body
  /// identities (`:308-327`, `:1600-1616`), and `Contact::depth`'s `= 0.0`
  /// default member initializer (`collision_common.hpp:84`) is what supplies
  /// the value, so it is reproducible but carries no penetration
  /// measurement. And `body_name_2` is frequently not a body: see
  /// `shapeKindsFor`'s doc for the `"self"`/`"environment"` sentinels and the
  /// `null` `shape_kinds` they produce.
  json distanceFieldCacheEntry(const json& request)
  {
    applyJointValues(request);
    applyAttachedBodies(*state_, request);
    requireBodySupportForAttachedBodies(*state_, "distance_field_cache_entry");

    const std::string group_name = request.at("group").get<std::string>();
    if (!model_->hasJointModelGroup(group_name))
      throw std::runtime_error("unknown group: " + group_name);

    const bool use_acm = request.value("use_acm", true);
    collision_detection::AllowedCollisionMatrix acm = buildAcm();

    collision_detection::WorldPtr world = std::make_shared<collision_detection::World>();
    addRequestObjects(*world, request);
    requireBodySupportForWorld(*world, "distance_field_cache_entry");
    collision_detection::CollisionEnvDistanceField env(model_, world);

    collision_detection::CollisionRequest req;
    req.group_name = group_name;
    const bool want_contacts = request.value("contacts", false);
    if (want_contacts)
    {
      req.contacts = true;
      req.max_contacts = request.value("max_contacts", static_cast<std::size_t>(100));
      req.max_contacts_per_pair = request.value("max_contacts_per_pair", static_cast<std::size_t>(1));
    }
    collision_detection::CollisionResult res;
    if (use_acm)
      env.checkSelfCollision(req, res, *state_, acm);
    else
      env.checkSelfCollision(req, res, *state_);

    collision_detection::DistanceFieldCacheEntryConstPtr dfce = env.getLastDistanceFieldEntry();
    if (!dfce)
      throw std::runtime_error("no DistanceFieldCacheEntry produced for group " + group_name);

    json out;
    out["group_name"] = dfce->group_name_;
    out["link_names"] = dfce->link_names_;
    out["link_has_geometry"] = dfce->link_has_geometry_;
    out["link_body_indices"] = dfce->link_body_indices_;
    out["link_state_indices"] = dfce->link_state_indices_;
    out["self_collision_enabled"] = dfce->self_collision_enabled_;
    out["intra_group_collision_enabled"] = dfce->intra_group_collision_enabled_;
    out["attached_body_names"] = dfce->attached_body_names_;
    out["attached_body_link_state_indices"] = dfce->attached_body_link_state_indices_;
    out["state_check_indices"] = dfce->state_check_indices_;
    out["state_values"] = dfce->state_values_;
    out["has_field"] = static_cast<bool>(dfce->distance_field_);

    json distance_queries_out = json::array();
    if (dfce->distance_field_ && request.contains("distance_queries"))
    {
      for (const auto& q : request.at("distance_queries"))
      {
        const auto pt = q.get<std::array<double, 3>>();
        distance_queries_out.push_back(dfce->distance_field_->getDistance(pt[0], pt[1], pt[2]));
      }
    }
    out["distance_queries"] = distance_queries_out;

    if (want_contacts)
    {
      out["collision"] = res.collision;
      out["contacts"] = allContactsToJson(res.contacts, *env.getWorld());
    }

    return out;
  }

  /// Ground truth for `cspace-distance-field`'s `group_state_representation`
  /// (upstream `CollisionEnvDistanceField::getGroupStateRepresentation`, a
  /// `protected` method) and, indirectly, `get_distance_field_cache_entry`
  /// (upstream `getDistanceFieldCacheEntry`). Driven through a public path
  /// similar to the `distance_field_cache_entry` op's (see that op's own
  /// doc comment), but `checkCollision` rather than `checkSelfCollision`:
  /// only `checkCollision`/`checkRobotCollision`/`getCollisionGradients`/
  /// `getAllCollisions` write `last_gsr_` (`collision_env_distance_field.cpp`,
  /// grepped to confirm -- `checkSelfCollision` never does), so
  /// `getLastGroupStateRepresentation()` would return null after
  /// `checkSelfCollision` alone. `checkCollision` ->
  /// `generateCollisionCheckingStructures` -> `getGroupStateRepresentation`,
  /// with the built `GroupStateRepresentation` read back via the public
  /// `getLastGroupStateRepresentation()`.
  ///
  /// # This op does *not* isolate `getGroupStateRepresentation`'s "fresh"
  /// branch the way `cspace-distance-field`'s `group_state_representation`
  /// implements it -- read this before trusting a field-by-field diff
  ///
  /// `CollisionEnvDistanceField(model_)`'s constructor runs `initialize()`,
  /// which -- for *every* joint model group, eagerly, once, at construction
  /// -- builds a fresh `GroupStateRepresentation` and stores it in
  /// `pregenerated_group_state_representation_map_`
  /// (`collision_env_distance_field.cpp:126-156`). Every later
  /// `generateDistanceFieldCacheEntry` call (this op's included, since
  /// `distance_field_cache_entry_` starts null so `getDistanceFieldCacheEntry`
  /// always misses) copies that pointer onto the new `dfce`
  /// (`dfce->pregenerated_group_state_representation_ = it->second;`,
  /// `collision_env_distance_field.cpp:869`), so `getGroupStateRepresentation`
  /// here always takes the **pregenerated** `else` branch, not the fresh one
  /// `cspace-distance-field`'s `group_state_representation` ports (that
  /// function's own doc comment explains why the pregenerated branch is
  /// unreachable *for this port*, specifically -- this port never builds the
  /// map behind it -- not that upstream's own runtime never reaches it; this
  /// op is the concrete case where it does). Two consequences for comparing
  /// this op's output against that Rust function's result:
  ///
  /// - The pregenerated branch's `else` clause additionally sets
  ///   `gradients_[i].sphere_locations = link_body_decompositions_[i]->getSphereCenters()`
  ///   (`collision_env_distance_field.cpp:1224`) after re-posing to the
  ///   current state -- the fresh branch never touches `sphere_locations` at
  ///   all. So `sphere_locations_count` below is always the link's sphere
  ///   count here, never `0`.
  ///
  ///   This used to be the end of it: the Rust port left the field empty, so
  ///   the two were not comparable. That is no longer true. `moveit-rs` round
  ///   25 closed the gap in the other direction -- its fresh-build path reads
  ///   `link_bd.sphere_centers()` directly, reproducing the value upstream
  ///   only produces on the pregenerated branch -- so this field now *is*
  ///   comparable, and this op is what measures that. See the parity test's
  ///   own doc comment.
  /// - `checkCollision` runs the full self/intra-group/environment collision
  ///   pipeline *after* `getGroupStateRepresentation` returns, and that
  ///   pipeline can mutate `closest_distance`/`collision`/`types`/`distances`
  ///   in place for any sphere actually found in collision -- construction
  ///   alone (what the Rust port's own scope covers) never does. Those four
  ///   fields therefore match a from-scratch construction only for a case
  ///   where no sphere overlaps, and the committed fixtures now sit on both
  ///   sides of that line: `group_state_representation_request.json`'s four
  ///   cases report `collision: false` on every link, while
  ///   `group_state_representation_contacts_request.json` ids 1 and 2 report
  ///   `collision: true` on 13 links of 22 (id 3 on none). A consumer
  ///   comparing construction against this dump has to gate on the per-link
  ///   flag rather than assume the fixture it happens to read is overlap-free.
  ///
  /// Every per-link field `getGroupStateRepresentation` writes
  /// deterministically (on *either* branch) is dumped: `has_link_decomposition`
  /// (was `link_body_decompositions_[i]` built at all), the posed bounding
  /// sphere/collision spheres/collision points, the posed distance field's
  /// pose, and every `GradientInfo` field *except* `gradients` -- the fresh
  /// branch's per-link block resizes that vector with no fill value
  /// (`gradients_[i].gradients.resize(count)`, no second argument), which
  /// value-initializes each new `Eigen::Vector3d` via its no-op default
  /// constructor, i.e. leaves it holding whatever bytes were already at that
  /// heap address -- genuinely indeterminate, not a reproducible zero or NaN
  /// (confirmed the same way `collision_distance_field_types_parity.rs`'s
  /// `relative_cylinder_pose` exclusion was: unrelated runs of this exact op
  /// returned different garbage floats for it), and the pregenerated branch's
  /// copy carries that same indeterminate snapshot forward from whichever
  /// state `initialize()` built it at. There is no defined upstream value
  /// here to dump or compare against, so it is omitted rather than captured
  /// misleadingly.
  ///
  /// `request["objects"]` (optional, defaults to none) populates the
  /// environment the check runs against, using the same single-shape schema
  /// as `collision` (see `addRequestObjects`). Without it the environment
  /// distance field is empty, so every environment branch -- including the
  /// `"environment"` sentinel contact and `getEnvironmentProximityGradients`
  /// -- is unreachable from a fixture.
  ///
  /// The world-taking constructor is used unconditionally rather than only
  /// when `objects` is non-empty. That it is equivalent for the empty case
  /// is measured, not assumed: all 43 fixture pairs committed at the time of
  /// that change replay identical across it (the corpus has grown since --
  /// this is the measurement's scope, not a running count), and the four
  /// `distance_field_cache_entry`
  /// / `group_state_representation` pairs among them send no `objects` at
  /// all. A conditional would have been a second code path reachable only by
  /// fixtures that do not exist yet.
  ///
  /// `request["attached_bodies"]` (optional, defaults to none) is attached
  /// before the check, so the attached-body half of the group's
  /// `DistanceFieldCacheEntry` is reachable at all -- without it
  /// `attached_body_names_` is always empty and every attached-body branch
  /// below it is untestable.
  ///
  /// It only becomes non-empty with `use_acm` *true*, and that is upstream's
  /// behaviour rather than this op's: the whole attached-body enumeration
  /// sits inside `generateDistanceFieldCacheEntry`'s `if (acm)`
  /// (`collision_env_distance_field.cpp:775`, the pushes at `:801-802`), so a
  /// null ACM leaves `attached_body_names_` empty no matter what is attached
  /// to the state. Measured, not inferred: the same request with `use_acm`
  /// false returns `attached_body_names: []` and with it true returns
  /// `["payload"]`.
  ///
  /// `request["contacts"]` (optional, defaults to `false`) selects upstream's
  /// contact-enumerating path and, when true, adds `collision` and `contacts`
  /// to the response. It is a request field rather than always-on because
  /// `req.contacts` is not an output-only switch upstream: the `true` branch
  /// of `getSelfCollisions` (`collision_env_distance_field.cpp:298-338`)
  /// writes `gsr->gradients_[i].types[col] = SELF` and
  /// `gradients_[i].collision = true` and keeps scanning, while the `false`
  /// branch (`:341-349`) returns on the first colliding link and writes
  /// neither. Those two fields are already dumped by this op, so flipping the
  /// switch would silently change what every existing fixture asserts.
  /// Making it a request field keeps old fixtures byte-identical and puts the
  /// mode in the fixture that asked for it. `max_contacts` /
  /// `max_contacts_per_pair` are exposed for the same reason -- the branch's
  /// scan bound is `std::min(req.max_contacts_per_pair, req.max_contacts -
  /// res.contact_count)`, so a fixture that cannot set them cannot reach the
  /// multi-contact shape at all.
  ///
  /// The difference is measurable rather than argued: one `contacts`-true
  /// request against pr2's `right_arm` reports `collision` on 5 of 22 links'
  /// gradients and `SELF` in one link's `types`, and the identical request
  /// with `contacts` absent reports 0 and 0.
  ///
  /// Two caveats for whoever writes the fixtures. Contact `depth` is always
  /// `0.0` on these paths -- upstream sets only `pos` and the two body
  /// identities (`:308-327`, `:1600-1616`), and `Contact::depth`'s `= 0.0`
  /// default member initializer (`collision_common.hpp:84`) is what supplies
  /// the value, so it is reproducible but carries no penetration
  /// measurement. And `body_name_2` is frequently not a body: see
  /// `shapeKindsFor`'s doc for the `"self"`/`"environment"` sentinels and the
  /// `null` `shape_kinds` they produce.
  ///
  /// `mode` (optional, `"collision"` by default, `"robot_only"` the only
  /// other value) selects `checkRobotCollision` instead of `checkCollision`.
  /// Requested by `crates/cspace-collision/doc/oracle-request-hybrid-collision-env-distance-field.md`:
  /// `checkRobotCollision` had never been called through any op, so
  /// `getEnvironmentCollisions`' distance/contact-recording logic had no
  /// oracle evidence behind it at all, in either direction. Absent, the
  /// dispatch is byte-for-byte the one every pre-existing fixture ran, which
  /// is what `verify-fixture-replay.sh` checks after the rebuild this field
  /// forced.
  ///
  /// `mode` is not orthogonal to `use_acm` on this path and a fixture author
  /// has to know that. `checkRobotCollision`'s no-acm overload calls
  /// `generateCollisionCheckingStructures(..., gsr, false)` and the acm
  /// overload passes `true` (`collision_env_distance_field.cpp:1462` vs
  /// `:1489`) -- the last argument is `generate_distance_field`, so dropping
  /// the acm also drops a structure, not just a filter. `checkCollision`'s own
  /// two overloads do not differ that way.
  json groupStateRepresentation(const json& request)
  {
    applyJointValues(request);
    applyAttachedBodies(*state_, request);
    requireBodySupportForAttachedBodies(*state_, "group_state_representation");

    const std::string group_name = request.at("group").get<std::string>();
    if (!model_->hasJointModelGroup(group_name))
      throw std::runtime_error("unknown group: " + group_name);

    const bool use_acm = request.value("use_acm", true);
    collision_detection::AllowedCollisionMatrix acm = buildAcm();

    collision_detection::WorldPtr world = std::make_shared<collision_detection::World>();
    addRequestObjects(*world, request);
    requireBodySupportForWorld(*world, "group_state_representation");
    collision_detection::CollisionEnvDistanceField env(model_, world);

    collision_detection::CollisionRequest req;
    req.group_name = group_name;
    const bool want_contacts = request.value("contacts", false);
    if (want_contacts)
    {
      req.contacts = true;
      req.max_contacts = request.value("max_contacts", static_cast<std::size_t>(100));
      req.max_contacts_per_pair = request.value("max_contacts_per_pair", static_cast<std::size_t>(1));
    }
    const bool want_gradients = request.value("gradients", false);
    if (want_gradients && want_contacts)
      throw std::runtime_error("gradients and contacts are mutually exclusive: getCollisionGradients "
                               "discards its CollisionResult (collision_env_distance_field.cpp:1517)");

    // `mode` selects which of `CollisionEnvDistanceField`'s entry points this
    // op drives. Absent (the default) is `checkCollision`, i.e. every fixture
    // committed before this field existed; `"robot_only"` is
    // `checkRobotCollision`, upstream's own name for "against the environment
    // only". Named after the method rather than given as a boolean so a third
    // entry point does not have to break the shape later.
    //
    // Mutually exclusive with `gradients` for the same reason that field is
    // already exclusive with `contacts`: `getCollisionGradients` and
    // `checkRobotCollision` are different upstream calls, not two flags on one
    // call. Rejected rather than silently ordered, so a request that asks for
    // both gets an error instead of whichever branch happens to be tested
    // first.
    //
    // Both `checkRobotCollision` overloads set `last_gsr_`
    // (`collision_env_distance_field.cpp:1468,1497`), so the response-dumping
    // code below needs no change to serve this branch --
    // `getLastGroupStateRepresentation()` is populated either way. They are
    // not otherwise symmetric and this op cannot hide that: the no-acm form
    // reaches `generateCollisionCheckingStructures(..., gsr, false)` while the
    // acm form passes `true` for that last argument (`:1462` vs `:1489`), so
    // `use_acm` selects more than whether the matrix is consulted.
    const std::string mode = request.value("mode", std::string("collision"));
    if (mode != "collision" && mode != "robot_only")
      throw std::runtime_error("unknown mode: " + mode + " (expected \"collision\" or \"robot_only\")");
    const bool robot_only = mode == "robot_only";
    if (robot_only && want_gradients)
      throw std::runtime_error("mode=robot_only and gradients are mutually exclusive: checkRobotCollision "
                               "and getCollisionGradients are different upstream calls "
                               "(collision_env_distance_field.cpp:1454,1527)");

    collision_detection::CollisionResult res;
    if (want_gradients)
    {
      collision_detection::GroupStateRepresentationPtr fresh;
      env.getCollisionGradients(req, res, *state_, use_acm ? &acm : nullptr, fresh);
    }
    else if (robot_only && use_acm)
    {
      env.checkRobotCollision(req, res, *state_, acm);
    }
    else if (robot_only)
    {
      env.checkRobotCollision(req, res, *state_);
    }
    else if (use_acm)
    {
      env.checkCollision(req, res, *state_, acm);
    }
    else
    {
      env.checkCollision(req, res, *state_);
    }

    collision_detection::GroupStateRepresentationConstPtr gsr = env.getLastGroupStateRepresentation();
    if (!gsr)
      throw std::runtime_error("no GroupStateRepresentation produced for group " + group_name);

    json links_out = json::array();
    for (std::size_t i = 0; i < gsr->dfce_->link_names_.size(); ++i)
    {
      json link_out;
      link_out["link_name"] = gsr->dfce_->link_names_[i];
      const collision_detection::PosedBodySphereDecompositionPtr& bd = gsr->link_body_decompositions_[i];
      link_out["has_link_decomposition"] = static_cast<bool>(bd);
      if (bd)
      {
        link_out["bounding_sphere_center"] = json::array(
            { bd->getBoundingSphereCenter().x(), bd->getBoundingSphereCenter().y(), bd->getBoundingSphereCenter().z() });
        link_out["bounding_sphere_radius"] = bd->getBoundingSphereRadius();

        json sphere_centers_out = json::array();
        for (const Eigen::Vector3d& c : bd->getSphereCenters())
          sphere_centers_out.push_back(json::array({ c.x(), c.y(), c.z() }));
        link_out["sphere_centers"] = sphere_centers_out;
        link_out["sphere_radii"] = bd->getSphereRadii();

        // Not the full point set: `BodyDecomposition::from_shapes`'s interior
        // sampling is independently oracle-verified elsewhere
        // (`collision_distance_field_types_parity.rs`,
        // `link_body_decomposition_parity.rs`), and every link's point count
        // here can run into the thousands -- dumping all of them would bloat
        // this fixture by orders of magnitude for no additional coverage.
        // The count alone still catches a wrong link's points being wired in
        // (`add_points_to_field` called with the wrong decomposition).
        link_out["collision_points_count"] = bd->getCollisionPoints().size();

        const collision_detection::PosedDistanceField& field = *gsr->link_distance_fields_[i];
        const Eigen::Isometry3d& pose = field.getPose();
        link_out["field_pose"] = json::array({ pose.translation().x(), pose.translation().y(),
                                               pose.translation().z(), Eigen::Quaterniond(pose.linear()).w(),
                                               Eigen::Quaterniond(pose.linear()).x(), Eigen::Quaterniond(pose.linear()).y(),
                                               Eigen::Quaterniond(pose.linear()).z() });

        link_out["gradient"] = gradientInfoToJson(gsr->gradients_[i]);
      }
      links_out.push_back(link_out);
    }

    json out{ { "group_name", group_name }, { "links", links_out } };
    if (want_gradients)
    {
      json attached_out = json::array();
      for (std::size_t i = gsr->dfce_->link_names_.size(); i < gsr->gradients_.size(); ++i)
      {
        attached_out.push_back(
            json{ { "name", gsr->dfce_->attached_body_names_.at(i - gsr->dfce_->link_names_.size()) },
                  { "gradient", gradientInfoToJson(gsr->gradients_[i]) } });
      }
      out["attached_body_gradients"] = attached_out;
    }
    if (want_contacts)
    {
      out["collision"] = res.collision;
      out["contacts"] = allContactsToJson(res.contacts, *env.getWorld());
    }
    return out;
  }

  /// Ground truth for `collision_common_distance_field.{hpp,cpp}`'s
  /// `getCollisionObjectPointDecomposition`: builds a one-object `World`
  /// exactly as the `world` op does, then dumps the flattened, posed
  /// interior points `getCollisionObjectPointDecomposition` produces --
  /// which routes through the same identity-keyed
  /// `getBodyDecompositionCacheEntry` this port's
  /// `collision_object_point_decomposition` also calls.
  json collisionObjectPointDecomposition(const json& request) const
  {
    const double resolution = request.at("resolution").get<double>();
    const json& object_json = request.at("object");
    const std::string id = object_json.at("id").get<std::string>();
    const Eigen::Isometry3d pose = fromRowMajor4x4(object_json.at("pose"));

    std::vector<shapes::ShapeConstPtr> shape_ptrs;
    EigenSTL::vector_Isometry3d shape_poses;
    const json& shapes_json = object_json.at("shapes");
    const json& shape_poses_json = object_json.at("shape_poses");
    for (std::size_t i = 0; i < shapes_json.size(); ++i)
    {
      const json& shape_json = shapes_json[i];
      shapes::ShapeConstPtr shape = parseShape(shape_json.at("type").get<std::string>(), shape_json);
      requireBodySupport(shape->type, "collision_object_point_decomposition");
      shape_ptrs.push_back(shape);
      shape_poses.push_back(fromRowMajor4x4(shape_poses_json[i]));
    }

    collision_detection::World w;
    w.addToObject(id, pose, shape_ptrs, shape_poses);

    collision_detection::PosedBodyPointDecompositionVectorPtr decomp =
        collision_detection::getCollisionObjectPointDecomposition(*w.getObject(id), resolution);

    json points_out = json::array();
    for (const Eigen::Vector3d& p : decomp->getCollisionPoints())
      points_out.push_back(json::array({ p.x(), p.y(), p.z() }));

    return json{ { "points", points_out } };
  }

  /// Ground truth for the central verification ask on this task: the posed
  /// sphere centres and radii `BodyDecomposition::from_shapes` +
  /// `PosedBodySphereDecomposition` should produce for a real robot link
  /// across a group state, built the same way upstream's (unported, out of
  /// scope this round) `addLinkBodyDecompositions` does --
  /// `BodyDecomposition(link->getShapes(), link->getCollisionOriginTransforms(),
  /// resolution, padding)` -- built once, then re-posed via
  /// `RobotState::getGlobalLinkTransform` per case. `request["cases"]`
  /// carries whole joint-value maps (not just this link's own joint) so the
  /// same request shape as `fk`/`jacobian` can encode "the same state
  /// queried twice" (two identical cases) and "one joint moves by less than
  /// the field resolution" (two cases differing by a sub-resolution delta
  /// on the joint that moves this link) without any special-casing here --
  /// those are properties of what the *runner* puts in `cases`, not this op.
  json linkBodyDecomposition(const json& request)
  {
    const std::string link_name = request.at("link").get<std::string>();
    const double resolution = request.at("resolution").get<double>();
    const double padding = request.at("padding").get<double>();

    if (!model_->hasLinkModel(link_name))
      throw std::runtime_error("unknown link: " + link_name);
    const moveit::core::LinkModel* link = model_->getLinkModel(link_name);

    collision_detection::BodyDecompositionConstPtr body_decomp =
        std::make_shared<const collision_detection::BodyDecomposition>(
            link->getShapes(), link->getCollisionOriginTransforms(), resolution, padding);

    json unposed_spheres = json::array();
    for (const collision_detection::CollisionSphere& cs : body_decomp->getCollisionSpheres())
    {
      unposed_spheres.push_back(
          json{ { "relative_vec", json::array({ cs.relative_vec_.x(), cs.relative_vec_.y(), cs.relative_vec_.z() }) },
                { "radius", cs.radius_ } });
    }

    json cases_out = json::array();
    for (const auto& case_json : request.at("cases"))
    {
      applyJointValues(json{ { "joint_values", case_json.at("joint_values") } });

      const Eigen::Isometry3d link_transform = state_->getGlobalLinkTransform(link_name);

      collision_detection::PosedBodySphereDecomposition posed(body_decomp);
      posed.updatePose(link_transform);

      json sphere_centers = json::array();
      for (const Eigen::Vector3d& c : posed.getSphereCenters())
        sphere_centers.push_back(json::array({ c.x(), c.y(), c.z() }));

      cases_out.push_back(json{ { "link_transform", toRowMajor4x4(link_transform) },
                                 { "sphere_centers", sphere_centers },
                                 { "bounding_sphere_center",
                                   json::array({ posed.getBoundingSphereCenter().x(),
                                                  posed.getBoundingSphereCenter().y(),
                                                  posed.getBoundingSphereCenter().z() }) },
                                 { "bounding_sphere_radius", posed.getBoundingSphereRadius() } });
    }

    return json{
      { "collision_spheres", unposed_spheres },
      { "relative_bounding_sphere",
        json{ { "center", json::array({ body_decomp->getRelativeBoundingSphere().center.x(),
                                         body_decomp->getRelativeBoundingSphere().center.y(),
                                         body_decomp->getRelativeBoundingSphere().center.z() }) },
              { "radius", body_decomp->getRelativeBoundingSphere().radius } } },
      { "cases", cases_out }
    };
  }

  /// Ground truth for `addLinkBodyDecompositions`' robot-wide link
  /// selection: `RobotModel::getLinkModelsWithCollisionGeometry()`'s names,
  /// in that method's own order (construction order -- the same order
  /// `getLinkModelNames()` in `model_info` reports, just filtered to links
  /// with `!getShapes().empty()`). Per-link `BodyDecomposition` geometry
  /// itself is already ground-truthed by the `link_body_decomposition` op
  /// (its `padding: 0.0` fixture case matches
  /// `LinkPaddingScale::link_padding`'s untracked-link default exactly, so
  /// that response's `collision_spheres`/`relative_bounding_sphere` double
  /// as ground truth for `addLinkBodyDecompositions`' per-link construction
  /// too) -- this op only needs to cover the link *set*, not re-dump every
  /// link's geometry.
  json linkModelsWithCollisionGeometry() const
  {
    json names = json::array();
    for (const moveit::core::LinkModel* link : model_->getLinkModelsWithCollisionGeometry())
      names.push_back(link->getName());
    return json{ { "links", names } };
  }

  /// Ground truth for the `cspace-trajectory` `ruckig_smoothing` port. Builds
  /// a `robot_trajectory::RobotTrajectory` from each case's waypoints and
  /// runs `trajectory_processing::RuckigSmoothing::applySmoothing` on it.
  ///
  /// A case that omits "velocity_limits" exercises the scaling-factor-only
  /// overload (RobotModel bounds, scaled by max_velocity_scaling_factor /
  /// max_acceleration_scaling_factor); a case that includes it (even as
  /// `{}`) exercises the explicit-limits overload instead, with
  /// "acceleration_limits" / "jerk_limits" defaulting to `{}` when absent --
  /// joints missing from a supplied map still fall back to RobotModel bounds
  /// (or DEFAULT_MAX_JERK), same as upstream. The
  /// `moveit_msgs::msg::JointLimits` overload is not exercised: it is a thin
  /// wrapper that unpacks into the explicit-limits overload's three maps,
  /// and cspace-trajectory does not port it (D1).
  json ruckig(const json& request)
  {
    const std::string group_name = request.at("group").get<std::string>();
    if (!model_->hasJointModelGroup(group_name))
      throw std::runtime_error("unknown group: " + group_name);

    json cases_out = json::array();
    for (const json& c : request.at("cases"))
      cases_out.push_back(ruckigCase(group_name, c));
    return json{ { "cases", cases_out } };
  }

  static std::unordered_map<std::string, double> readLimitMap(const json& c, const char* field)
  {
    std::unordered_map<std::string, double> out;
    if (!c.contains(field))
      return out;
    for (auto it = c.at(field).begin(); it != c.at(field).end(); ++it)
      out[it.key()] = it.value().get<double>();
    return out;
  }

  json ruckigCase(const std::string& group_name, const json& c)
  {
    robot_trajectory::RobotTrajectory trajectory(model_, group_name);

    const json& waypoints = c.at("waypoints");
    const json& durations = c.at("durations_from_previous");
    if (waypoints.size() != durations.size())
      throw std::runtime_error("waypoints/durations_from_previous length mismatch");

    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
      moveit::core::RobotState waypoint_state(model_);
      waypoint_state.setToDefaultValues();
      const json& values = waypoints.at(i);
      for (auto it = values.begin(); it != values.end(); ++it)
      {
        if (!hasVariable(it.key()))
          throw std::runtime_error("unknown joint variable: " + it.key());
        waypoint_state.setVariablePosition(it.key(), it.value().get<double>());
      }
      waypoint_state.update();
      trajectory.addSuffixWayPoint(waypoint_state, durations.at(i).get<double>());
    }

    const double max_velocity_scaling_factor = c.value("max_velocity_scaling_factor", 1.0);
    const double max_acceleration_scaling_factor = c.value("max_acceleration_scaling_factor", 1.0);
    const bool mitigate_overshoot = c.value("mitigate_overshoot", false);
    const double overshoot_threshold = c.value("overshoot_threshold", 0.01);

    bool ok = false;
    if (c.contains("velocity_limits"))
    {
      const std::unordered_map<std::string, double> velocity_limits = readLimitMap(c, "velocity_limits");
      const std::unordered_map<std::string, double> acceleration_limits = readLimitMap(c, "acceleration_limits");
      const std::unordered_map<std::string, double> jerk_limits = readLimitMap(c, "jerk_limits");
      ok = trajectory_processing::RuckigSmoothing::applySmoothing(
          trajectory, velocity_limits, acceleration_limits, jerk_limits, max_velocity_scaling_factor,
          max_acceleration_scaling_factor, mitigate_overshoot, overshoot_threshold);
    }
    else
    {
      ok = trajectory_processing::RuckigSmoothing::applySmoothing(
          trajectory, max_velocity_scaling_factor, max_acceleration_scaling_factor, mitigate_overshoot,
          overshoot_threshold);
    }

    json result;
    result["ok"] = ok;
    if (!ok)
      return result;

    const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_name);
    const std::vector<std::string>& variable_names = group->getVariableNames();

    json out_waypoints = json::array();
    json out_durations = json::array();
    for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i)
    {
      const moveit::core::RobotState& wp = trajectory.getWayPoint(i);
      json positions = json::object();
      json velocities = json::object();
      json accelerations = json::object();
      for (const std::string& name : variable_names)
      {
        positions[name] = wp.getVariablePosition(name);
        velocities[name] = wp.getVariableVelocity(name);
        accelerations[name] = wp.getVariableAcceleration(name);
      }
      out_waypoints.push_back(json{ { "positions", positions },
                                     { "velocities", velocities },
                                     { "accelerations", accelerations } });
      out_durations.push_back(trajectory.getWayPointDurationFromPrevious(i));
    }
    result["waypoints"] = out_waypoints;
    result["durations_from_previous"] = out_durations;
    return result;
  }

  /// Ground truth for the `cspace-trajectory` `Path`/`Trajectory` port (the
  /// model-independent numeric core of
  /// `trajectory_processing::time_optimal_trajectory_generation.hpp` lines
  /// 62-192) *and*, when a request names a `"group"`, for the
  /// `TimeOptimalTrajectoryGeneration` adapter (header line 193 on) --
  /// see `totgRobotTrajectoryCase` below. A request with no `"group"` key
  /// runs the original core-only path (`totgCase`), unchanged, so
  /// `totg_request.json`/`totg_response.json` (captured before the adapter
  /// was in scope) keep working verbatim. Each core-only case runs
  /// `Path::create` then, if that succeeds, `Trajectory::create`, and
  /// reports which stage failed when either returns `std::nullopt`.
  /// `sample_times` are request-supplied rather than derived from the
  /// computed duration, so both sides evaluate at identically the same
  /// instants regardless of any duration disagreement -- the numbers
  /// nlohmann::json can't represent (NaN, on a zero-length path) serialize
  /// as JSON `null` via `dump_float`'s own NaN/Inf guard.
  static Eigen::VectorXd totgReadVector(const json& arr)
  {
    Eigen::VectorXd v(static_cast<Eigen::Index>(arr.size()));
    for (std::size_t i = 0; i < arr.size(); ++i)
      v[static_cast<Eigen::Index>(i)] = arr.at(i).get<double>();
    return v;
  }

  static json totgWriteVector(const Eigen::VectorXd& v)
  {
    json out = json::array();
    for (Eigen::Index i = 0; i < v.size(); ++i)
      out.push_back(v[i]);
    return out;
  }

  json totg(const json& request)
  {
    json cases_out = json::array();
    if (request.contains("group"))
    {
      const std::string group_name = request.at("group").get<std::string>();
      if (!model_->hasJointModelGroup(group_name))
        throw std::runtime_error("unknown group: " + group_name);
      for (const json& c : request.at("cases"))
        cases_out.push_back(totgRobotTrajectoryCase(group_name, c));
    }
    else
    {
      for (const json& c : request.at("cases"))
        cases_out.push_back(totgCase(c));
    }
    return json{ { "cases", cases_out } };
  }

  /// Re-implementation of `TimeOptimalTrajectoryGeneration::hasMixedJointTypes`
  /// (cpp:1273-1288), which the oracle cannot call directly (it is a
  /// private member of that class): identical logic over the same public
  /// `JointModelGroup::getActiveJointModels()`/`JointModel::getType()`
  /// this crate's own `has_mixed_joint_types` also uses. Exposed on every
  /// `totgRobotTrajectoryCase` response (not just as a stderr `RCLCPP_WARN`
  /// side effect, which upstream's own real call site at cpp:1176 is
  /// limited to) so a mixed-joint-type group is a wire-checkable parity
  /// case rather than a log line a test would have to scrape.
  static bool hasMixedJointTypesForGroup(const moveit::core::JointModelGroup* group)
  {
    const std::vector<const moveit::core::JointModel*>& joint_models = group->getActiveJointModels();
    const bool have_prismatic =
        std::any_of(joint_models.cbegin(), joint_models.cend(), [](const moveit::core::JointModel* joint_model) {
          return joint_model->getType() == moveit::core::JointModel::JointType::PRISMATIC;
        });
    const bool have_revolute =
        std::any_of(joint_models.cbegin(), joint_models.cend(), [](const moveit::core::JointModel* joint_model) {
          return joint_model->getType() == moveit::core::JointModel::JointType::REVOLUTE;
        });
    return have_prismatic && have_revolute;
  }

  /// Ground truth for the `cspace-trajectory`
  /// `time_optimal_trajectory_generation` module -- the
  /// `TimeOptimalTrajectoryGeneration` adapter around `Path`/`Trajectory`.
  /// Shaped exactly like `ruckigCase` above (same waypoint/duration/
  /// limit-map wire format, same result shape), since both build a
  /// `robot_trajectory::RobotTrajectory` from named-variable waypoints and
  /// report it back the same way. A case that omits "velocity_limits"
  /// exercises the scaling-factor-only `computeTimeStamps` overload
  /// (`RobotModel` bounds, scaled by `max_velocity_scaling_factor`/
  /// `max_acceleration_scaling_factor`); a case that includes it (even as
  /// `{}`) exercises the explicit-limits overload instead, with
  /// "acceleration_limits" defaulting to `{}` when absent. The
  /// `moveit_msgs::msg::JointLimits` overload is not exercised, for the same
  /// reason `ruckig` does not exercise its analogous overload: a thin
  /// wrapper the port doesn't carry (D1).
  /// A case may also carry "acceleration_bounds" (joint name -> max
  /// acceleration, same shape as "velocity_limits"/"acceleration_limits"):
  /// applied to `model_` itself, via the same read
  /// (`JointModel::getVariableBoundsMsg`) / mutate / write
  /// (`JointModel::setVariableBounds`) pattern `joint_limits.yaml` loaders
  /// use upstream, *before* the scaling-only overload runs. `panda.urdf`
  /// (like every URDF) has no `<limit>` acceleration field, so `model_`
  /// never has `acceleration_bounded` set on any joint without this --
  /// which is exactly why the scaling-only overload had no reachable test
  /// case anywhere in this workspace before `RobotModel::joint_model_mut`
  /// landed on the Rust side (see `time_optimal_trajectory_generation.rs`'s
  /// former "Known gap" section, now closed). The mutation goes through
  /// `applyJointBounds`, so it lasts to the end of this request and no
  /// further; see that function for what it used to cost. It stays in its own
  /// `totg_robot_trajectory_scaling_only_request.json` because the cases in
  /// one request still share the override, not because the process does.
  json totgRobotTrajectoryCase(const std::string& group_name, const json& c)
  {
    if (c.contains("acceleration_bounds"))
    {
      for (const auto& [joint_name, max_acceleration] : readLimitMap(c, "acceleration_bounds"))
      {
        moveit::core::JointModel* joint_model = model_->getJointModel(joint_name);
        if (joint_model == nullptr)
          throw std::runtime_error("unknown joint: " + joint_name);
        std::vector<moveit_msgs::msg::JointLimits> limits = joint_model->getVariableBoundsMsg();
        for (moveit_msgs::msg::JointLimits& limit : limits)
        {
          limit.has_acceleration_limits = true;
          limit.max_acceleration = max_acceleration;
        }
        applyJointBounds(joint_model, limits);
      }
    }

    const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_name);
    robot_trajectory::RobotTrajectory trajectory(model_, group_name);

    const json& waypoints = c.at("waypoints");
    const json& durations = c.at("durations_from_previous");
    if (waypoints.size() != durations.size())
      throw std::runtime_error("waypoints/durations_from_previous length mismatch");

    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
      moveit::core::RobotState waypoint_state(model_);
      waypoint_state.setToDefaultValues();
      const json& values = waypoints.at(i);
      for (auto it = values.begin(); it != values.end(); ++it)
      {
        if (!hasVariable(it.key()))
          throw std::runtime_error("unknown joint variable: " + it.key());
        waypoint_state.setVariablePosition(it.key(), it.value().get<double>());
      }
      waypoint_state.update();
      trajectory.addSuffixWayPoint(waypoint_state, durations.at(i).get<double>());
    }

    const double max_velocity_scaling_factor = c.value("max_velocity_scaling_factor", 1.0);
    const double max_acceleration_scaling_factor = c.value("max_acceleration_scaling_factor", 1.0);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    bool ok = false;
    if (c.contains("velocity_limits"))
    {
      const std::unordered_map<std::string, double> velocity_limits = readLimitMap(c, "velocity_limits");
      const std::unordered_map<std::string, double> acceleration_limits = readLimitMap(c, "acceleration_limits");
      ok = totg.computeTimeStamps(trajectory, velocity_limits, acceleration_limits, max_velocity_scaling_factor,
                                   max_acceleration_scaling_factor);
    }
    else
    {
      ok = totg.computeTimeStamps(trajectory, max_velocity_scaling_factor, max_acceleration_scaling_factor);
    }

    json result;
    result["ok"] = ok;
    result["has_mixed_joint_types"] = hasMixedJointTypesForGroup(group);
    if (!ok)
      return result;

    const std::vector<std::string>& variable_names = group->getVariableNames();

    json out_waypoints = json::array();
    json out_durations = json::array();
    for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i)
    {
      const moveit::core::RobotState& wp = trajectory.getWayPoint(i);
      json positions = json::object();
      json velocities = json::object();
      json accelerations = json::object();
      for (const std::string& name : variable_names)
      {
        positions[name] = wp.getVariablePosition(name);
        velocities[name] = wp.getVariableVelocity(name);
        accelerations[name] = wp.getVariableAcceleration(name);
      }
      out_waypoints.push_back(json{ { "positions", positions },
                                     { "velocities", velocities },
                                     { "accelerations", accelerations } });
      out_durations.push_back(trajectory.getWayPointDurationFromPrevious(i));
    }
    result["waypoints"] = out_waypoints;
    result["durations_from_previous"] = out_durations;
    return result;
  }

  json totgCase(const json& c) const
  {
    std::vector<Eigen::VectorXd> waypoints;
    for (const json& wp : c.at("waypoints"))
      waypoints.push_back(totgReadVector(wp));
    const double max_deviation = c.at("max_deviation").get<double>();

    std::optional<trajectory_processing::Path> path =
        trajectory_processing::Path::create(waypoints, max_deviation);
    if (!path)
      return json{ { "ok", false }, { "stage", "path_create" } };

    const Eigen::VectorXd max_velocity = totgReadVector(c.at("max_velocity"));
    const Eigen::VectorXd max_acceleration = totgReadVector(c.at("max_acceleration"));
    const double time_step = c.at("time_step").get<double>();

    std::optional<trajectory_processing::Trajectory> trajectory =
        trajectory_processing::Trajectory::create(*path, max_velocity, max_acceleration, time_step);
    if (!trajectory)
      return json{ { "ok", false }, { "stage", "trajectory_create" } };

    json samples = json::array();
    for (const json& t_json : c.at("sample_times"))
    {
      const double t = t_json.get<double>();
      samples.push_back(json{ { "time", t },
                               { "position", totgWriteVector(trajectory->getPosition(t)) },
                               { "velocity", totgWriteVector(trajectory->getVelocity(t)) },
                               { "acceleration", totgWriteVector(trajectory->getAcceleration(t)) } });
    }

    return json{ { "ok", true }, { "duration", trajectory->getDuration() }, { "samples", samples } };
  }

  /// Ground truth for the *geometry* half of TOTG, separated from `totg`'s
  /// time-parameterisation half: builds only the `Path` and dumps what it
  /// exposes, with no `Trajectory` and therefore no integration, no time
  /// step, and no velocity/acceleration limits in play. A divergence that
  /// shows up here is in the path construction; one that shows up in `totg`
  /// but not here is in the integration.
  ///
  /// This is deliberately *not* the per-`CircularPathSegment` dump of
  /// `start_dot_end`/`angle`/`radius`/`center`/`x`/`y` that would be the
  /// direct analogue of this port's own internals. `CircularPathSegment` is
  /// declared inside `time_optimal_trajectory_generation.cpp` (line 103), not
  /// in the installed header, and `Path::getPathSegment` is `private`, so
  /// neither the type nor the per-segment handle is nameable from outside the
  /// translation unit -- unlike `DistanceField::getOcTreePoints`, which is
  /// merely `protected` and so was reachable by deriving (see `octreePoints`).
  /// Reaching those fields would mean patching upstream, which would make the
  /// oracle stop being a record of unmodified upstream behaviour.
  ///
  /// The public surface still pins the same geometry, indirectly but at full
  /// precision:
  ///
  /// - `getSwitchingPoints` returns every segment boundary as an arc length,
  ///   so the *lengths* of the three circular blends and of the straight runs
  ///   between them are differences of adjacent entries. A blend's arc length
  ///   is `angle * radius`.
  /// - `getCurvature(s)` inside a circular segment is the inward normal
  ///   scaled by `1/radius` (`CircularPathSegment::getCurvature` returns
  ///   `-(x*cos + y*sin)/radius` with `x`,`y` unit and orthogonal), so
  ///   `radius = 1 / ||getCurvature(s)||` recovers the radius directly, and
  ///   `angle` then follows from the arc length. Inside a linear segment the
  ///   curvature is the zero vector.
  /// - `getConfig(s)`/`getTangent(s)` at the two ends of a blend pin where the
  ///   blend starts and leaves, which with the radius pins the centre up to
  ///   the sign convention.
  ///
  /// So `angle`, `radius`, and the blend endpoints are all recoverable from
  /// this response; only `center`, `x`, and `y` as stored are not, and those
  /// are a basis choice rather than an observable of the path.
  ///
  /// A case gives "waypoints" and "max_deviation" (same meaning and same
  /// reader as `totgCase`). Sampling is by explicit arc length:
  /// "sample_arc_lengths" is taken verbatim, and "sample_count", if present
  /// and greater than one, adds a uniform grid of that many points over
  /// `[0, length]` inclusive. Explicit arc lengths are the useful ones here --
  /// a uniform grid will generally straddle a switching point rather than
  /// land inside a blend, and it is the samples *strictly inside* one blend
  /// that carry its radius.
  json totgPath(const json& request) const
  {
    json cases_out = json::array();
    for (const json& c : request.at("cases"))
      cases_out.push_back(totgPathCase(c));
    return json{ { "cases", cases_out } };
  }

  json totgPathCase(const json& c) const
  {
    std::vector<Eigen::VectorXd> waypoints;
    for (const json& wp : c.at("waypoints"))
      waypoints.push_back(totgReadVector(wp));
    const double max_deviation = c.at("max_deviation").get<double>();

    std::optional<trajectory_processing::Path> path =
        trajectory_processing::Path::create(waypoints, max_deviation);
    if (!path)
      return json{ { "ok", false }, { "stage", "path_create" } };

    const double length = path->getLength();

    json switching_out = json::array();
    for (const std::pair<double, bool>& sp : path->getSwitchingPoints())
      switching_out.push_back(json{ { "s", sp.first }, { "discontinuity", sp.second } });

    std::vector<double> arc_lengths;
    if (c.contains("sample_arc_lengths"))
      for (const json& s_json : c.at("sample_arc_lengths"))
        arc_lengths.push_back(s_json.get<double>());
    const std::size_t sample_count = c.value("sample_count", std::size_t{ 0 });
    for (std::size_t i = 0; i < sample_count && sample_count > 1; ++i)
      arc_lengths.push_back(length * static_cast<double>(i) / static_cast<double>(sample_count - 1));

    json samples = json::array();
    for (const double s : arc_lengths)
    {
      const Eigen::VectorXd curvature = path->getCurvature(s);
      samples.push_back(json{ { "s", s },
                              { "config", totgWriteVector(path->getConfig(s)) },
                              { "tangent", totgWriteVector(path->getTangent(s)) },
                              { "curvature", totgWriteVector(curvature) },
                              { "curvature_norm", curvature.norm() } });
    }

    return json{ { "ok", true },
                 { "length", length },
                 { "switching_points", switching_out },
                 { "samples", samples } };
  }

  /// Ground truth for the `cspace-smoothing` `AccelerationLimitedPlugin` port
  /// -- the online, one-step-lookahead acceleration-limiting QP in
  /// `online_signal_smoothing/acceleration_filter.cpp`. This is the only op
  /// in this file that starts a real `rclcpp::Node`:
  /// `AccelerationLimitedPlugin::initialize` takes an `rclcpp::Node::SharedPtr`
  /// and reads its own declared parameters (`update_period`,
  /// `planning_group_name`) through a `generate_parameter_library`
  /// `ParamListener`, so there is no way to drive the real upstream class
  /// without one. The node is never spun and nothing calls `rclcpp::spin` --
  /// it exists only to hold two parameter values and a clock, so this starts
  /// no discovery loop beyond the one-time participant `rclcpp::init` creates
  /// for the process. `rclcpp::init` itself is called at most once per
  /// process (guarded by `rclcpp::ok()`), since a second call would throw.
  ///
  /// The plugin is loaded through `pluginlib::ClassLoader`, not a direct
  /// `#include` of `acceleration_filter.hpp` plus a CMake target link:
  /// `moveit_acceleration_filter` is installed under
  /// `moveit_core_pluginTargets`, a separate export set
  /// `moveit_core/CMakeLists.txt` never passes to `ament_export_targets` (only
  /// `moveit_coreTargets` is exported) -- upstream's own intended consumption
  /// path for this plugin family is pluginlib's runtime `dlopen`, resolved
  /// from the `filter_plugin_acceleration.xml` description
  /// `pluginlib_export_plugin_description_file(moveit_core ...)` registers
  /// into the ament index, not a compile-time link. Going through
  /// `SmoothingBaseClass`'s pure-virtual interface this way also means this
  /// op never touches `osqp` or `MOVEIT_OSQP_V1` directly -- that stays fully
  /// inside the plugin's own `.so`.
  ///
  /// A case names a "group" (used both as the plugin's own
  /// `planning_group_name` parameter and as the joint-name space for
  /// "reset"/"commands" -- restricted, like every group this file's fixtures
  /// use for smoothing ops, to groups whose active joints each own exactly
  /// one variable, since `AccelerationLimitedPlugin::initialize`'s own
  /// bounds-extraction loop advances its index once per *joint*, not once
  /// per *variable*, and silently keeps only the last variable's bound for
  /// any joint with more than one -- transcribed as-is on the Rust side
  /// rather than worked around here), an "update_period", and an
  /// "acceleration_bounds" map (joint name -> symmetric max |accel|, same
  /// convention and the same `JointModel::setVariableBounds` mutation-of-
  /// `model_` pattern as `totgRobotTrajectoryCase`'s own "acceleration_bounds"
  /// above, routed through `applyJointBounds` so it ends with the request and
  /// still shared by the cases within it). `AccelerationLimitedPlugin::initialize` fails outright if any
  /// active joint in the group lacks an acceleration bound, so a case that
  /// omits a joint from "acceleration_bounds" exercises that failure path
  /// (`"initialize_ok": false`), not an unconstrained-bound case.
  ///
  /// "reset" seeds `last_positions_`/`last_velocities_`
  /// (`AccelerationLimitedPlugin::reset`); each entry of "commands" is one
  /// `doSmoothing` call in sequence, threaded through the plugin's own
  /// internal state exactly as a real streaming caller would be. The
  /// `accelerations` parameter `doSmoothing`/`reset` both take is read by
  /// neither (see the unnamed/`/* unused */` parameter in the upstream
  /// signatures) -- this op passes a scratch zero vector on every call and
  /// the wire format has no `accelerations` field anywhere.
  json accelerationFilter(const json& request)
  {
    json cases_out = json::array();
    for (const json& c : request.at("cases"))
      cases_out.push_back(accelerationFilterCase(c));
    return json{ { "cases", cases_out } };
  }

  static Eigen::VectorXd readNamedVector(const json& map, const std::vector<std::string>& names)
  {
    Eigen::VectorXd v(static_cast<Eigen::Index>(names.size()));
    for (std::size_t i = 0; i < names.size(); ++i)
      v[static_cast<Eigen::Index>(i)] = map.at(names[i]).get<double>();
    return v;
  }

  static json writeNamedVector(const Eigen::VectorXd& v, const std::vector<std::string>& names)
  {
    json out = json::object();
    for (std::size_t i = 0; i < names.size(); ++i)
      out[names[i]] = v[static_cast<Eigen::Index>(i)];
    return out;
  }

  json accelerationFilterCase(const json& c)
  {
    const std::string group_name = c.at("group").get<std::string>();
    if (!model_->hasJointModelGroup(group_name))
      throw std::runtime_error("unknown group: " + group_name);

    for (const auto& [joint_name, max_acceleration] : readLimitMap(c, "acceleration_bounds"))
    {
      moveit::core::JointModel* joint_model = model_->getJointModel(joint_name);
      if (joint_model == nullptr)
        throw std::runtime_error("unknown joint: " + joint_name);
      std::vector<moveit_msgs::msg::JointLimits> limits = joint_model->getVariableBoundsMsg();
      for (moveit_msgs::msg::JointLimits& limit : limits)
      {
        limit.has_acceleration_limits = true;
        limit.max_acceleration = max_acceleration;
      }
      applyJointBounds(joint_model, limits);
    }

    const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_name);
    std::vector<std::string> joint_names;
    for (const moveit::core::JointModel* joint_model : group->getActiveJointModels())
      joint_names.push_back(joint_model->getName());
    const auto num_joints = static_cast<std::size_t>(joint_names.size());

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(
        { rclcpp::Parameter("update_period", c.at("update_period").get<double>()),
          rclcpp::Parameter("planning_group_name", group_name) });
    static int node_counter = 0;
    auto node = std::make_shared<rclcpp::Node>("acceleration_filter_oracle_" + std::to_string(node_counter++),
                                               node_options);

    pluginlib::ClassLoader<online_signal_smoothing::SmoothingBaseClass> loader(
        "moveit_core", "online_signal_smoothing::SmoothingBaseClass");
    std::shared_ptr<online_signal_smoothing::SmoothingBaseClass> plugin =
        loader.createSharedInstance("online_signal_smoothing::AccelerationLimitedPlugin");

    json result;
    const bool initialize_ok = plugin->initialize(node, model_, num_joints);
    result["initialize_ok"] = initialize_ok;
    if (!initialize_ok)
      return result;

    const json& reset = c.at("reset");
    const Eigen::VectorXd reset_positions = readNamedVector(reset.at("positions"), joint_names);
    const Eigen::VectorXd reset_velocities = readNamedVector(reset.at("velocities"), joint_names);
    Eigen::VectorXd scratch_accelerations = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(num_joints));
    result["reset_ok"] = plugin->reset(reset_positions, reset_velocities, scratch_accelerations);

    json steps = json::array();
    for (const json& command : c.at("commands"))
    {
      Eigen::VectorXd positions = readNamedVector(command.at("positions"), joint_names);
      Eigen::VectorXd velocities = readNamedVector(command.at("velocities"), joint_names);
      const bool ok = plugin->doSmoothing(positions, velocities, scratch_accelerations);
      steps.push_back(json{ { "ok", ok },
                            { "positions", writeNamedVector(positions, joint_names) },
                            { "velocities", writeNamedVector(velocities, joint_names) } });
    }
    result["steps"] = steps;
    return result;
  }

  /// Ground truth for the `cspace-smoothing` `RuckigFilter` port -- the
  /// streaming, jerk-limited command smoother in
  /// `online_signal_smoothing/ruckig_filter.cpp`. Loaded through the same
  /// `pluginlib::ClassLoader<online_signal_smoothing::SmoothingBaseClass>` +
  /// one-off, never-spun `rclcpp::Node` as `accelerationFilter` above, for
  /// the identical reason: `RuckigFilterPlugin::initialize` has the same
  /// `SmoothingBaseClass` signature (`rclcpp::Node::SharedPtr`), and
  /// `moveit_ruckig_filter` sits under the same non-exported
  /// `moveit_core_pluginTargets` set as `moveit_acceleration_filter` --
  /// `filter_plugin_ruckig.xml` registers it in the ament plugin index the
  /// same way. See `accelerationFilter`'s own comment for the full node/
  /// pluginlib rationale, not repeated here.
  ///
  /// A case names a "group" (also the plugin's `planning_group_name`
  /// parameter and joint-name space), an "update_period", and
  /// "velocity_bounds"/"acceleration_bounds"/"jerk_bounds" maps (joint name
  /// -> symmetric max magnitude), applied to `model_` via
  /// `setJointAccelerationVelocityJerkBounds` -- same `JointModel::
  /// setVariableBounds` mutation-of-`model_` pattern, same single-DOF-joint-
  /// group restriction, and the same request-scoped lifetime as
  /// `accelerationFilter`'s own "acceleration_bounds". Missing any one of
  /// the three bound kinds for an active joint fails
  /// `RuckigFilterPlugin::initialize` outright (`getVelAccelJerkBounds`
  /// checks velocity, then acceleration, then jerk, in that order) --
  /// `"initialize_ok": false` in that case, no "reset"/"commands" evaluated.
  ///
  /// "reset" seeds `current_position_`/`current_velocity_`/
  /// `current_acceleration_` (`RuckigFilterPlugin::reset`, which -- unlike
  /// `AccelerationLimitedPlugin::reset` -- reads all three); each entry of
  /// "commands" is one `doSmoothing` call in sequence, threaded through the
  /// plugin's own internal Ruckig state exactly as a real streaming caller
  /// would be. A command names only "positions": `doSmoothing`'s own
  /// `velocities`/`accelerations` parameters are pure outputs upstream --
  /// never read, only overwritten -- so this op passes zero-filled scratch
  /// vectors of the right size for them on every call, matching
  /// `cspace-smoothing::ruckig_filter::RuckigFilter::do_smoothing`'s
  /// identical treatment on the Rust side (see that function's own doc
  /// comment on why the incoming `positions` is the sole per-call input).
  json ruckigFilter(const json& request)
  {
    json cases_out = json::array();
    for (const json& c : request.at("cases"))
      cases_out.push_back(ruckigFilterCase(c));
    return json{ { "cases", cases_out } };
  }

  // Not static: the bound override has to be recorded on `this` so the request
  // that made it can put it back (see `applyJointBounds`).
  void setJointAccelerationVelocityJerkBounds(const moveit::core::RobotModelPtr& model, const json& c)
  {
    auto apply = [&](const char* field, void (*setter)(moveit_msgs::msg::JointLimits&, double)) {
      for (const auto& [joint_name, value] : readLimitMap(c, field))
      {
        moveit::core::JointModel* joint_model = model->getJointModel(joint_name);
        if (joint_model == nullptr)
          throw std::runtime_error("unknown joint: " + joint_name);
        std::vector<moveit_msgs::msg::JointLimits> limits = joint_model->getVariableBoundsMsg();
        for (moveit_msgs::msg::JointLimits& limit : limits)
          setter(limit, value);
        applyJointBounds(joint_model, limits);
      }
    };
    apply("velocity_bounds", [](moveit_msgs::msg::JointLimits& limit, double value) {
      limit.has_velocity_limits = true;
      limit.max_velocity = value;
    });
    apply("acceleration_bounds", [](moveit_msgs::msg::JointLimits& limit, double value) {
      limit.has_acceleration_limits = true;
      limit.max_acceleration = value;
    });
    apply("jerk_bounds", [](moveit_msgs::msg::JointLimits& limit, double value) {
      limit.has_jerk_limits = true;
      limit.max_jerk = value;
    });
  }

  json ruckigFilterCase(const json& c)
  {
    const std::string group_name = c.at("group").get<std::string>();
    if (!model_->hasJointModelGroup(group_name))
      throw std::runtime_error("unknown group: " + group_name);

    setJointAccelerationVelocityJerkBounds(model_, c);

    const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_name);
    std::vector<std::string> joint_names;
    for (const moveit::core::JointModel* joint_model : group->getActiveJointModels())
      joint_names.push_back(joint_model->getName());
    const auto num_joints = static_cast<std::size_t>(joint_names.size());

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(
        { rclcpp::Parameter("update_period", c.at("update_period").get<double>()),
          rclcpp::Parameter("planning_group_name", group_name) });
    static int node_counter = 0;
    auto node =
        std::make_shared<rclcpp::Node>("ruckig_filter_oracle_" + std::to_string(node_counter++), node_options);

    pluginlib::ClassLoader<online_signal_smoothing::SmoothingBaseClass> loader(
        "moveit_core", "online_signal_smoothing::SmoothingBaseClass");
    std::shared_ptr<online_signal_smoothing::SmoothingBaseClass> plugin =
        loader.createSharedInstance("online_signal_smoothing::RuckigFilterPlugin");

    json result;
    const bool initialize_ok = plugin->initialize(node, model_, num_joints);
    result["initialize_ok"] = initialize_ok;
    if (!initialize_ok)
      return result;

    const json& reset = c.at("reset");
    const Eigen::VectorXd reset_positions = readNamedVector(reset.at("positions"), joint_names);
    const Eigen::VectorXd reset_velocities = readNamedVector(reset.at("velocities"), joint_names);
    const Eigen::VectorXd reset_accelerations = readNamedVector(reset.at("accelerations"), joint_names);
    result["reset_ok"] = plugin->reset(reset_positions, reset_velocities, reset_accelerations);

    json steps = json::array();
    for (const json& command : c.at("commands"))
    {
      Eigen::VectorXd positions = readNamedVector(command.at("positions"), joint_names);
      Eigen::VectorXd velocities = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(num_joints));
      Eigen::VectorXd accelerations = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(num_joints));
      const bool ok = plugin->doSmoothing(positions, velocities, accelerations);
      steps.push_back(json{ { "ok", ok },
                            { "positions", writeNamedVector(positions, joint_names) },
                            { "velocities", writeNamedVector(velocities, joint_names) },
                            { "accelerations", writeNamedVector(accelerations, joint_names) } });
    }
    result["steps"] = steps;
    return result;
  }

  /// Ground truth for the `cspace-constraints` `KinematicConstraintSet` port.
  /// Applies `joint_values` on top of the model defaults the same way
  /// `fk`/`jacobian` do, builds a `moveit_msgs::msg::Constraints` from
  /// `request["constraints"]` (see the free-function `*FromJson` builders
  /// above `Oracle` for the per-kind message shapes, matching
  /// `protocol.rs`'s `ConstraintsSpec`), builds a
  /// `moveit::core::Transforms(model_->getModelFrame())` (identity-only, no
  /// TF listener), and calls `KinematicConstraintSet::add(msg, tf)` then
  /// `decide(state, results)`. `add` returning `false` means at least one
  /// constraint failed to `configure()` -- this differential test's own case
  /// generator never produces such a constraint, so that is treated as a
  /// hard error rather than silently evaluating a partially-configured set
  /// (see `cspace-constraints`' own `KinematicConstraintSet::decide`
  /// deviation doc for why a partially-decidable set should never be
  /// reported as if it were fully decided).
  json constraints(const json& request)
  {
    applyJointValues(request);

    moveit_msgs::msg::Constraints msg;
    const json& spec = request.at("constraints");
    if (spec.contains("joint_constraints"))
      for (const auto& jc_json : spec.at("joint_constraints"))
        msg.joint_constraints.push_back(jointConstraintFromJson(jc_json));
    if (spec.contains("position_constraints"))
      for (const auto& pc_json : spec.at("position_constraints"))
        msg.position_constraints.push_back(positionConstraintFromJson(pc_json));
    if (spec.contains("orientation_constraints"))
      for (const auto& oc_json : spec.at("orientation_constraints"))
        msg.orientation_constraints.push_back(orientationConstraintFromJson(oc_json));
    if (spec.contains("visibility_constraints"))
      for (const auto& vc_json : spec.at("visibility_constraints"))
        msg.visibility_constraints.push_back(visibilityConstraintFromJson(vc_json));

    const moveit::core::Transforms transforms(model_->getModelFrame());
    kinematic_constraints::KinematicConstraintSet set(model_);
    if (!set.add(msg, transforms))
      throw std::runtime_error("KinematicConstraintSet::add failed to configure one or more constraints");

    std::vector<kinematic_constraints::ConstraintEvaluationResult> results;
    set.decide(*state_, results);

    json results_out = json::array();
    for (const kinematic_constraints::ConstraintEvaluationResult& r : results)
      results_out.push_back(json{ { "satisfied", r.satisfied }, { "distance", r.distance } });

    return json{ { "results", results_out } };
  }

  /// Ground truth for cspace-octomap's port of octomap 1.9.7 (see
  /// crates/cspace-core/src/octomap/*.rs's provenance comments): builds a
  /// throwaway `octomap::OcTree` local to this call (ignoring model_/state_,
  /// same pattern as shapePoints above), replays a request-described
  /// sequence of updates, and reports whatever the request asks to query
  /// afterward. Deliberately generic rather than one C++ method per
  /// scenario -- the scenario itself (repeated hits to the clamp, a miss
  /// sequence, a to-be-pruned cube of siblings, a ray leaving the tree) is
  /// constructed entirely on the Rust test side from these primitives.
  /// Shared by octomapOp and octreeInWorld so the two ops agree on what "the
  /// same actions" means: update_point, update_key, insert_ray, prune,
  /// update_inner_occupancy, set_occupancy_thres, set_prob_hit,
  /// set_prob_miss, set_clamping_thres_min, set_clamping_thres_max (the
  /// five `{"prob": <double>}` sensor-model setters cspace-octomap's round
  /// 15 ports -- ground truth for their effect on later hits/misses and on
  /// `isNodeOccupied`'s boundary, not merely on this port's own arithmetic).
  static void applyOctomapActions(octomap::OcTree& tree, const json& actions)
  {
    for (const auto& action : actions)
    {
      const std::string type = action.at("type").get<std::string>();
      const bool lazy_eval = action.value("lazy_eval", false);
      if (type == "update_point")
      {
        const auto p = action.at("point").get<std::array<double, 3>>();
        tree.updateNode(octomap::point3d(p[0], p[1], p[2]), action.at("occupied").get<bool>(), lazy_eval);
      }
      else if (type == "update_key")
      {
        const auto k = action.at("key").get<std::array<int, 3>>();
        const octomap::OcTreeKey key(static_cast<octomap::key_type>(k[0]), static_cast<octomap::key_type>(k[1]),
                                     static_cast<octomap::key_type>(k[2]));
        tree.updateNode(key, action.at("occupied").get<bool>(), lazy_eval);
      }
      else if (type == "insert_ray")
      {
        const auto o = action.at("origin").get<std::array<double, 3>>();
        const auto e = action.at("end").get<std::array<double, 3>>();
        const double max_range = action.value("max_range", -1.0);
        tree.insertRay(octomap::point3d(o[0], o[1], o[2]), octomap::point3d(e[0], e[1], e[2]), max_range, lazy_eval);
      }
      else if (type == "prune")
      {
        tree.prune();
      }
      else if (type == "update_inner_occupancy")
      {
        tree.updateInnerOccupancy();
      }
      else if (type == "set_occupancy_thres")
      {
        tree.setOccupancyThres(action.at("prob").get<double>());
      }
      else if (type == "set_prob_hit")
      {
        tree.setProbHit(action.at("prob").get<double>());
      }
      else if (type == "set_prob_miss")
      {
        tree.setProbMiss(action.at("prob").get<double>());
      }
      else if (type == "set_clamping_thres_min")
      {
        tree.setClampingThresMin(action.at("prob").get<double>());
      }
      else if (type == "set_clamping_thres_max")
      {
        tree.setClampingThresMax(action.at("prob").get<double>());
      }
      else
      {
        throw std::runtime_error("octomap: unsupported action type " + type);
      }
    }
  }

  json octomapOp(const json& request) const
  {
    const double resolution = request.at("resolution").get<double>();
    octomap::OcTree tree(resolution);
    applyOctomapActions(tree, request.at("actions"));

    json results = json::array();
    for (const auto& query : request.at("queries"))
    {
      const std::string type = query.at("type").get<std::string>();
      if (type == "occupancy")
      {
        const auto p = query.at("point").get<std::array<double, 3>>();
        const octomap::OcTreeNode* node = tree.search(p[0], p[1], p[2]);
        if (node == nullptr)
          results.push_back(json{ { "mapped", false } });
        else
          results.push_back(json{ { "mapped", true },
                                   { "log_odds", node->getLogOdds() },
                                   { "occupancy", node->getOccupancy() },
                                   { "occupied", tree.isNodeOccupied(node) } });
      }
      else if (type == "occupancy_by_key")
      {
        // Symmetric to the "update_key" action: queries by OcTreeKey
        // directly, so a caller comparing pruned/collapsed nodes does not
        // have to re-derive a float coordinate that round-trips through
        // coordToKeyChecked back to the same key.
        const auto k = query.at("key").get<std::array<int, 3>>();
        const octomap::OcTreeKey key(static_cast<octomap::key_type>(k[0]), static_cast<octomap::key_type>(k[1]),
                                     static_cast<octomap::key_type>(k[2]));
        const octomap::OcTreeNode* node = tree.search(key);
        if (node == nullptr)
          results.push_back(json{ { "mapped", false } });
        else
          results.push_back(json{ { "mapped", true },
                                   { "log_odds", node->getLogOdds() },
                                   { "occupancy", node->getOccupancy() },
                                   { "occupied", tree.isNodeOccupied(node) } });
      }
      else if (type == "ray_keys")
      {
        const auto o = query.at("origin").get<std::array<double, 3>>();
        const auto e = query.at("end").get<std::array<double, 3>>();
        octomap::KeyRay ray;
        const bool ok = tree.computeRayKeys(octomap::point3d(o[0], o[1], o[2]), octomap::point3d(e[0], e[1], e[2]), ray);
        json keys = json::array();
        for (const octomap::OcTreeKey& k : ray)
          keys.push_back(json::array({ k[0], k[1], k[2] }));
        results.push_back(json{ { "ok", ok }, { "keys", keys } });
      }
      else if (type == "node_count")
      {
        results.push_back(json{ { "count", static_cast<std::uint64_t>(tree.calcNumNodes()) } });
      }
      else if (type == "tree_walk")
      {
        // Direct, field-by-field pin for `begin_tree()`/`end_tree()`
        // (inner nodes and leaves, pre-order): every node's position,
        // size, depth, leaf-ness and occupancy, in traversal order. Unlike
        // `octreeInWorld`, which only checks a tree's *effect* on
        // downstream collision/distance results, this exposes the
        // iterator's own per-node output directly, so a Rust-side
        // `tree_iterator` port can be compared node-for-node instead of
        // merely self-consistently.
        json nodes = json::array();
        for (auto it = tree.begin_tree(), end = tree.end_tree(); it != end; ++it)
        {
          nodes.push_back(json{ { "x", it.getX() },
                                 { "y", it.getY() },
                                 { "z", it.getZ() },
                                 { "size", it.getSize() },
                                 { "depth", it.getDepth() },
                                 { "is_leaf", it.isLeaf() },
                                 { "log_odds", it->getLogOdds() },
                                 { "occupancy", it->getOccupancy() } });
        }
        results.push_back(json{ { "nodes", nodes } });
      }
      else if (type == "serialize")
      {
        // The two byte streams `octomap_msgs::msg::Octomap.data` can carry,
        // which is what a Rust decoder has to consume.
        //
        // `conversions.h`'s `readTree`/`binaryMsgToMap` feed `msg.data`
        // straight to `readBinaryData`, and `fullMsgToMap` feeds it to
        // `readData` -- neither writes or expects a `.bt`/`.ot` file header,
        // so `writeBinaryData`/`writeData` are the exact inverses and the
        // exact bytes that arrive on the wire. Emitting `writeBinary` here
        // instead would prepend a header the decoder must never see.
        //
        // Hex rather than base64: the fixtures are compared byte-for-byte by
        // a human as often as by a test, and `check-serde-float-roundtrip.sh`
        // has no float to protect here.
        //
        // `prune()` first, matching what upstream ships: `binaryMsgFromMap`'s
        // callers hand it a tree that has been through `updateNode`, and an
        // unpruned tree serialises a different node set for the same
        // occupancy. Pruning here makes the emitted bytes the ones a decoder
        // will actually meet.
        tree.prune();
        std::ostringstream binary_stream;
        tree.writeBinaryData(binary_stream);
        std::ostringstream full_stream;
        tree.writeData(full_stream);

        const auto to_hex = [](const std::string& raw) {
          std::ostringstream out;
          out << std::hex << std::setfill('0');
          for (const unsigned char byte : raw)
            out << std::setw(2) << static_cast<unsigned>(byte);
          return out.str();
        };

        results.push_back(json{ { "id", tree.getTreeType() },
                                 { "resolution", tree.getResolution() },
                                 { "node_count", static_cast<std::uint64_t>(tree.calcNumNodes()) },
                                 { "binary", to_hex(binary_stream.str()) },
                                 { "full", to_hex(full_stream.str()) } });
      }
      else
      {
        throw std::runtime_error("octomap: unsupported query type " + type);
      }
    }

    return json{ { "results", results } };
  }

  /// Ground truth for wiring `shapes::OcTree` into
  /// `collision_detection::World` -- the round-2 gap this task flagged:
  /// MoveIt represents sensor-derived obstacles as an octomap *inside the
  /// collision world*, and nothing prior to this op exercised that path.
  /// Builds an `octomap::OcTree` from `request["actions"]` (identical
  /// vocabulary to octomapOp, replayed through the same applyOctomapActions
  /// helper), wraps it in a `shapes::OcTree` exactly as
  /// `collision_detection::World::addToObject` expects any shape, adds it to
  /// a `World` object at `request["object_pose"]` (with an optional
  /// per-shape `request["shape_pose"]`, identity if absent), then reports:
  ///  - the shape's local pose and its world-composed global pose (the same
  ///    fields the `world` op above reports for any shape) -- this confirms
  ///    `World` composes an OcTree shape's pose the same generic way it
  ///    composes any other shape, i.e. nothing octree-specific happens at
  ///    this layer, matching this port's finding that neither
  ///    collision_detection_fcl nor collision_env_distance_field special-case
  ///    *pose* handling, only the geometry conversion;
  ///  - for each `request["queries"]` world-frame point, whether it is
  ///    occupied, computed the way a collision backend must: invert the
  ///    shape's global pose to map the world-frame point into the octree's
  ///    own local frame, then query that local point. This is the
  ///    computation collision_common.cpp's `fcl::OcTreed` wrap and
  ///    collision_env_distance_field.cpp's
  ///    `PosedBodyPointDecomposition(octree)` each rely on their own backend
  ///    performing before an octree query means anything in world
  ///    coordinates -- see crates/cspace-core/src/geometry/shapes.rs's module docs
  ///    for the full FCL/parry3d-f64/distance-field consumer analysis.
  ///
  /// Round 5's gap: everything above stops at pose composition and point
  /// occupancy, never a real collision query -- and `octreeShapeQuery` below
  /// deliberately bypasses `CollisionEnvFCL`/`RobotState`/ACM entirely (see
  /// its own doc). Neither answers "does a real robot collide with a World
  /// whose only object is this octree", which is the question a
  /// `cspace_collision::ParryCollisionEnv` wiring `Shape::OcTree` through
  /// `convert_shape` actually needs ground truth for. When the request
  /// carries a `"robot"` key (`{"joint_values": {...}}`, `collision`'s own
  /// vocabulary via `applyJointValues`), this additionally builds a real
  /// `collision_detection::CollisionEnvFCL` over the exact same `World`
  /// object already built above and runs `checkRobotCollision`/
  /// `distanceRobot`, adding `robot_collision`/`robot_distance` to the
  /// response. Requests without `"robot"` (round 2's own fixtures) see no
  /// change to the existing fields.
  json octreeInWorld(const json& request)
  {
    const double resolution = request.at("resolution").get<double>();
    auto tree = std::make_shared<octomap::OcTree>(resolution);
    applyOctomapActions(*tree, request.at("actions"));

    auto shape = std::make_shared<shapes::OcTree>(tree);

    auto w = std::make_shared<collision_detection::World>();
    const Eigen::Isometry3d object_pose = fromRowMajor4x4(request.at("object_pose"));
    const Eigen::Isometry3d shape_pose =
        request.contains("shape_pose") ? fromRowMajor4x4(request.at("shape_pose")) : Eigen::Isometry3d::Identity();

    const std::vector<shapes::ShapeConstPtr> shape_ptrs{ shape };
    const EigenSTL::vector_Isometry3d shape_poses{ shape_pose };
    w->addToObject("octree_object", object_pose, shape_ptrs, shape_poses);

    const EigenSTL::vector_Isometry3d& global_shape_poses = w->getGlobalShapeTransforms("octree_object");
    const Eigen::Isometry3d& global_pose = global_shape_poses.at(0);

    json queries_out = json::array();
    for (const auto& query : request.at("queries"))
    {
      const auto p = query.at("point").get<std::array<double, 3>>();
      const Eigen::Vector3d world_point(p[0], p[1], p[2]);
      const Eigen::Vector3d local_point = global_pose.inverse() * world_point;

      const octomap::OcTreeNode* node = tree->search(local_point.x(), local_point.y(), local_point.z());
      if (node == nullptr)
        queries_out.push_back(json{ { "mapped", false } });
      else
        queries_out.push_back(
            json{ { "mapped", true }, { "log_odds", node->getLogOdds() }, { "occupancy", node->getOccupancy() } });
    }

    json result = json{ { "shape_pose", toRowMajor4x4(shape_pose) },
                         { "global_pose", toRowMajor4x4(global_pose) },
                         { "queries", queries_out } };

    if (request.contains("robot"))
    {
      applyJointValues(request.at("robot"));
      collision_detection::AllowedCollisionMatrix acm = buildAcm();
      collision_detection::CollisionEnvFCL env(model_, w);

      collision_detection::CollisionRequest robot_req;
      collision_detection::CollisionResult robot_res;
      env.checkRobotCollision(robot_req, robot_res, *state_, acm);

      collision_detection::DistanceRequest robot_dreq;
      robot_dreq.enable_signed_distance = true;
      robot_dreq.acm = &acm;
      collision_detection::DistanceResult robot_dres;
      env.distanceRobot(robot_dreq, robot_dres, *state_);

      result["robot_collision"] = robot_res.collision;
      result["robot_distance"] = robot_dres.minimum_distance.distance;
    }

    return result;
  }

  /// Ground truth for an actual octree-vs-shape collision/distance query --
  /// `octreeInWorld` above only checks point occupancy through pose
  /// composition, it never runs a real query against a second shape. This is
  /// what PORTING-PLAN.md's octree decision (search that file for "결정
  /// 완료" -- the leaf-`Cuboid` `parry3d_f64::shape::Compound`
  /// approximation of `shapes::OcTree`) needs to be checked against: does
  /// the `Compound` agree with what FCL's own `fcl::OcTreed` narrow-phase
  /// actually reports for the same octree and the same query shape.
  ///
  /// Builds an `octomap::OcTree` from `request["actions"]` (the same
  /// `applyOctomapActions` vocabulary `octomapOp`/`octreeInWorld` use),
  /// wraps it as `shapes::OcTree` and the request's `shape` (any type
  /// `parseShape` supports) as a second `World::Object`, each posed
  /// independently (`octree_pose`, `shape_pose`). `World::Object::shapes_`
  /// and `getGlobalShapeTransforms` are the same fields `octreeInWorld` and
  /// the `world` op already rely on.
  ///
  /// Deliberately does not go through `CollisionEnvFCL`/`RobotState`/ACM:
  /// that machinery answers "does the robot collide with the world", not
  /// "do these two arbitrary objects collide", and an octree is not a robot
  /// link. Instead this builds the two `fcl::CollisionObjectd`s directly --
  /// `collision_detection::createCollisionGeometry(shape, World::Object*)`
  /// is the same public conversion `CollisionEnvFCL` itself calls internally
  /// (see moveit2's `collision_common.cpp`), so this is not a shortcut
  /// around FCL, just around the robot-centric wrapper -- and queries them
  /// with FCL's own free functions, `fcl::collide`/`fcl::distance`, exactly
  /// as `collision_common.cpp`'s own collision/distance callbacks do.
  json octreeShapeQuery(const json& request) const
  {
    const double resolution = request.at("resolution").get<double>();
    auto tree = std::make_shared<octomap::OcTree>(resolution);
    applyOctomapActions(*tree, request.at("actions"));
    auto octree_shape = std::make_shared<shapes::OcTree>(tree);
    const Eigen::Isometry3d octree_pose = fromRowMajor4x4(request.at("octree_pose"));

    const json& shape_json = request.at("shape");
    std::shared_ptr<shapes::Shape> query_shape = parseShape(shape_json.at("type").get<std::string>(), shape_json);
    const Eigen::Isometry3d shape_pose = fromRowMajor4x4(request.at("shape_pose"));

    auto world = std::make_shared<collision_detection::World>();
    world->addToObject("octree", octree_pose, { octree_shape }, { Eigen::Isometry3d::Identity() });
    world->addToObject("query", shape_pose, { query_shape }, { Eigen::Isometry3d::Identity() });

    collision_detection::World::ObjectConstPtr octree_obj = world->getObject("octree");
    collision_detection::World::ObjectConstPtr query_obj = world->getObject("query");

    collision_detection::FCLGeometryConstPtr octree_geom =
        collision_detection::createCollisionGeometry(octree_obj->shapes_[0], octree_obj.get());
    collision_detection::FCLGeometryConstPtr query_geom =
        collision_detection::createCollisionGeometry(query_obj->shapes_[0], query_obj.get());

    const Eigen::Isometry3d& octree_global = world->getGlobalShapeTransforms("octree").at(0);
    const Eigen::Isometry3d& query_global = world->getGlobalShapeTransforms("query").at(0);

    fcl::CollisionObjectd fcl_octree(octree_geom->collision_geometry_, octree_global);
    fcl::CollisionObjectd fcl_query(query_geom->collision_geometry_, query_global);

    fcl::CollisionRequestd creq;
    creq.enable_contact = true;
    creq.num_max_contacts = 200;
    fcl::CollisionResultd cres;
    const std::size_t num_contacts = fcl::collide(&fcl_octree, &fcl_query, creq, cres);

    fcl::DistanceRequestd dreq;
    dreq.enable_signed_distance = true;
    fcl::DistanceResultd dres;
    fcl::distance(&fcl_octree, &fcl_query, dreq, dres);

    return json{
      { "collision", num_contacts > 0 },
      { "num_contacts", static_cast<std::uint64_t>(num_contacts) },
      { "distance", dres.min_distance },
    };
  }

  /// Ground truth for `bodies::Body`'s posed algorithms --
  /// `containsPoint`/`intersectsRay`/`computeBoundingBox`/
  /// `computeBoundingSphere`/`computeBoundingCylinder` -- which
  /// `cspace-geometry`'s `Body` enum ported some rounds ago but which, until
  /// now, only `tests/probe_parity.rs`'s standalone `libgeometric_shapes.so`
  /// probe exercised, never the oracle's own JSON-line protocol. Builds a
  /// `bodies::Body` exactly the way `shapePoints` above does
  /// (`createEmptyBodyFromShapeType` + `setDimensionsDirty` +
  /// `setScaleDirty`/`setPaddingDirty` + `setPoseDirty` +
  /// `updateInternalData`, the batch form, so scale/padding/pose are all
  /// applied before the one `updateInternalData()` call rather than
  /// recomputing internal state after each setter). `request["points"]`
  /// is answered with `containsPoint`; `request["rays"]` (each an
  /// `origin`/`dir`/optional `count`, `count` omitted or `0` meaning
  /// upstream's "unlimited") is answered with `intersectsRay`'s real
  /// `intersections` out-param, not just its boolean return, so a caller can
  /// check hit points and hit *count* -- the exact contract a fixture that
  /// "skips rays and reports success" would never catch, because it would
  /// never look past a bare boolean.
  json bodyQuery(const json& request) const
  {
    const json& shape_json = request.at("shape");
    const std::string type = shape_json.at("type").get<std::string>();
    const Eigen::Isometry3d pose = fromRowMajor4x4(request.at("pose"));
    const double scale = request.value("scale", 1.0);
    const double padding = request.value("padding", 0.0);

    std::shared_ptr<shapes::Shape> shape = parseShape(type, shape_json);
    requireBodySupport(shape->type, "body_query");

    bodies::Body* body = bodies::createEmptyBodyFromShapeType(shape->type);
    body->setDimensionsDirty(shape.get());
    body->setScaleDirty(scale);
    body->setPaddingDirty(padding);
    body->setPoseDirty(pose);
    body->updateInternalData();

    json contains_out = json::array();
    for (const auto& p : request.value("points", json::array()))
    {
      const auto pt = p.get<std::array<double, 3>>();
      contains_out.push_back(body->containsPoint(pt[0], pt[1], pt[2]));
    }

    json rays_out = json::array();
    for (const auto& r : request.value("rays", json::array()))
    {
      const auto o = r.at("origin").get<std::array<double, 3>>();
      auto d = r.at("dir").get<std::array<double, 3>>();
      const unsigned int count = r.value("count", 0u);

      const Eigen::Vector3d origin(o[0], o[1], o[2]);
      Eigen::Vector3d dir(d[0], d[1], d[2]);
      dir.normalize();

      EigenSTL::vector_Vector3d intersections;
      const bool hit = body->intersectsRay(origin, dir, &intersections, count);

      json pts = json::array();
      for (const Eigen::Vector3d& ip : intersections)
        pts.push_back(json::array({ ip.x(), ip.y(), ip.z() }));
      rays_out.push_back(json{ { "hit", hit }, { "points", pts } });
    }

    bodies::BoundingSphere bsphere;
    body->computeBoundingSphere(bsphere);
    bodies::BoundingCylinder bcyl;
    body->computeBoundingCylinder(bcyl);
    bodies::AABB aabb;
    body->computeBoundingBox(aabb);
    bodies::OBB obb;
    body->computeBoundingBox(obb);
    const double volume = body->computeVolume();
    delete body;

    return json{
      { "contains", contains_out },
      { "rays", rays_out },
      { "volume", volume },
      { "bsphere",
        { { "center", json::array({ bsphere.center.x(), bsphere.center.y(), bsphere.center.z() }) },
          { "radius", bsphere.radius } } },
      { "bcyl",
        { { "radius", bcyl.radius },
          { "length", bcyl.length },
          { "origin", json::array({ bcyl.pose.translation().x(), bcyl.pose.translation().y(),
                                     bcyl.pose.translation().z() }) } } },
      { "aabb", { { "min", json::array({ aabb.min().x(), aabb.min().y(), aabb.min().z() }) },
                  { "max", json::array({ aabb.max().x(), aabb.max().y(), aabb.max().z() }) } } },
      { "obb", { { "extents", json::array({ obb.getExtents().x(), obb.getExtents().y(), obb.getExtents().z() }) },
                 { "origin", json::array({ obb.getPose().translation().x(), obb.getPose().translation().y(),
                                            obb.getPose().translation().z() }) } } },
    };
  }

  /// Phase 7's C++ baseline: OMPL RRTConnect over a state space built to
  /// mirror `cspace-planners-sbp`'s own `JointModelGroupSpace`, with validity
  /// answered by a real `planning_scene::PlanningScene`.
  ///
  /// # What this baseline is, and what it is not
  ///
  /// It is plain `ompl::geometric::RRTConnect` -- the reference
  /// implementation of the algorithm the port reimplements -- driven over a
  /// space this op constructs joint by joint to match `JointModelGroupSpace`
  /// exactly (see `buildPlanSpace`), so `PathGeometric::length()` and the
  /// port's own path length are measured in the *same metric* and the
  /// completion condition's "median path length within 1.3x" compares two
  /// numbers that mean the same thing.
  ///
  /// It is **not** `moveit_planners_ompl`'s `ModelBasedPlanningContext`. That
  /// class adds a projection evaluator, path simplification, a
  /// `ModelBasedStateSpace` whose distance delegates to
  /// `JointModelGroup::distance` over a flat array (not a weighted
  /// `CompoundStateSpace` -- the deviation `joint_model_group_space.rs`
  /// already documents), and the request-adapter chain. None of that is here.
  /// A success rate or path length from this op is a statement about
  /// RRTConnect on this space, not about what `move_group` would return.
  ///
  /// # Determinism
  ///
  /// `ompl::RNG::setSeed` is process-global and OMPL rejects it once any
  /// `RNG` instance exists, so the request's `seed` is applied at most once
  /// per oracle process -- the response says whether this call is the one
  /// that applied it (`seed_applied`). A whole request stream replayed in
  /// order into a fresh process is therefore reproducible, which is exactly
  /// what `verify-fixture-replay.sh` does; a single problem re-run in
  /// isolation is not.
  ///
  /// The termination budget is a counting `PlannerTerminationCondition`
  /// rather than a wall clock, so the answer does not depend on machine
  /// speed. OMPL 1.7 ships no iteration-based condition (checked:
  /// `PlannerTerminationCondition.h` declares only the timed, non/always,
  /// and/or, and exact-solution forms), so this counts *evaluations of the
  /// condition*.
  ///
  /// That count is the same unit as the port's `Termination::Iterations`,
  /// measured rather than assumed: `og::RRTConnect::solve` evaluates the
  /// condition exactly once at the top of each grow-and-connect iteration
  /// and nowhere else, so a budget of `n` permits `n` iterations and the
  /// terminating evaluation is the `n+1`th. A `max_iterations` sweep of
  /// `0,1,2,3,5,10,50,200,5000` on panda_arm against a box obstacle returns
  /// `ptc_evaluations = 1` at budget `0` with status `Timeout` and no path
  /// -- zero iterations run, exactly as the port's `for _ in
  /// 0..max_iterations` does at the same budget. `ptc_evaluations` is
  /// returned on every problem so this correspondence stays checkable
  /// rather than becoming a claim in a comment.
  ///
  /// Note what that same sweep also shows: every non-zero budget solved in
  /// one iteration. RRTConnect's `connect` step is greedy and unbounded --
  /// it walks toward the other tree until it arrives or is blocked -- so on
  /// a 7-DoF arm a single iteration routinely closes the whole gap, and an
  /// iteration budget is a far weaker lever here than its name suggests.
  /// Sizing a benchmark by this budget alone would mostly measure nothing.
  ///
  /// # Comparability knobs
  ///
  /// `range` maps to `RRTConnect::setRange`, the port's
  /// `RrtConnectParams::step_size`. `motion_resolution` is an *absolute*
  /// spacing in space-distance units, matching the port's
  /// `DiscreteMotionValidator::new(checker, resolution)`; OMPL states the
  /// same quantity as a fraction of the space's maximum extent, so it is
  /// converted here and both the extent and the resolved segment length are
  /// returned. The port has no counterpart to OMPL's `goal_bias`-free
  /// bidirectional growth: `RrtConnectParams::goal_bias` biases each tree
  /// toward the other tree's root, and `og::RRTConnect` has no such
  /// parameter at all. That is a real algorithmic difference between the two
  /// implementations, recorded here rather than papered over with a knob
  /// that does not exist on this side.
  ///
  /// `distance_probes` is the deterministic part of this op: each entry's
  /// `a`/`b` joint maps are converted to states and the *space's own*
  /// `distance` is returned. Planner output is seed-dependent, so this is the
  /// only surface on which the Rust side can assert bit-level agreement --
  /// and agreement here is what makes a length comparison meaningful at all.
  json plan(const json& request)
  {
    // OMPL's default console output handler writes INFO and WARN to stdout,
    // which is this process's NDJSON response channel. Silencing it is not
    // cosmetic: one OMPL log line mid-stream desynchronizes every subsequent
    // response from its request.
    ompl::msg::noOutputHandler();

    const std::string group_name = request.at("group").get<std::string>();
    const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_name);
    if (!group)
      throw std::runtime_error("plan: unknown joint model group: " + group_name);

    bool seed_applied = false;
    if (request.contains("seed") && !ompl_seeded_)
    {
      ompl::RNG::setSeed(request.at("seed").get<std::uint_fast32_t>());
      ompl_seeded_ = true;
      seed_applied = true;
    }

    PlanSpace plan_space = buildPlanSpace(group);
    plan_space.space->setup();
    const double maximum_extent = plan_space.space->getMaximumExtent();

    planning_scene::PlanningScene scene(model_);
    scene.getCurrentStateNonConst() = *state_;
    addRequestObjects(*scene.getWorldNonConst(), request);

    auto si = std::make_shared<ob::SpaceInformation>(plan_space.space);
    si->setStateValidityChecker(std::make_shared<PlanValidityChecker>(si, scene, plan_space.layout));
    const double motion_resolution = request.value("motion_resolution", 0.0);
    if (motion_resolution > 0.0)
      si->setStateValidityCheckingResolution(motion_resolution / maximum_extent);
    si->setup();

    // Reads a request's joint-name -> value map into a full RobotState,
    // starting from the default state so joints outside `group` hold the same
    // values on both sides.
    const auto read_state = [&](const json& joints) {
      moveit::core::RobotState robot_state(*state_);
      for (auto it = joints.begin(); it != joints.end(); ++it)
      {
        if (!hasVariable(it.key()))
          throw std::runtime_error("plan: unknown joint variable: " + it.key());
        robot_state.setVariablePosition(it.key(), it.value().get<double>());
      }
      robot_state.update();
      return robot_state;
    };

    json probes_out = json::array();
    for (const json& probe : request.value("distance_probes", json::array()))
    {
      ob::ScopedState<> a(plan_space.space);
      ob::ScopedState<> b(plan_space.space);
      robotStateToPlanState(read_state(probe.at("a")), plan_space.layout, a.get());
      robotStateToPlanState(read_state(probe.at("b")), plan_space.layout, b.get());
      probes_out.push_back(json{ { "distance", plan_space.space->distance(a.get(), b.get()) } });
    }

    const double range = request.value("range", 0.0);
    const std::size_t default_max_iterations = request.value("max_iterations", std::size_t{ 100000 });

    json problems_out = json::array();
    for (const json& problem : request.value("problems", json::array()))
    {
      ob::ScopedState<> start(plan_space.space);
      ob::ScopedState<> goal(plan_space.space);
      robotStateToPlanState(read_state(problem.at("start")), plan_space.layout, start.get());
      robotStateToPlanState(read_state(problem.at("goal")), plan_space.layout, goal.get());

      auto pdef = std::make_shared<ob::ProblemDefinition>(si);
      pdef->setStartAndGoalStates(start, goal);

      auto planner = std::make_shared<og::RRTConnect>(si);
      if (range > 0.0)
        planner->setRange(range);
      planner->setProblemDefinition(pdef);
      planner->setup();

      const std::size_t max_iterations = problem.value("max_iterations", default_max_iterations);
      std::size_t evaluations = 0;
      const ob::PlannerTerminationCondition ptc(
          [&evaluations, max_iterations]() { return ++evaluations > max_iterations; });

      const ob::PlannerStatus status = planner->solve(ptc);

      // No wall-clock field here either -- see `pilzTrajectory`'s comment on
      // the same point. This one was latent rather than live: the single
      // committed `plan` fixture has an empty `problems` array, so the
      // timing never reached a response file and replay stayed clean by
      // accident. `ptc_evaluations` is the deterministic stand-in, and it
      // is what actually bounds the search.
      json problem_out{ { "id", problem.at("id") },
                        { "solved", static_cast<bool>(status) },
                        { "exact", pdef->hasExactSolution() },
                        { "status", status.asString() },
                        { "ptc_evaluations", evaluations } };

      // Only an exact solution is reported as a path: RRTConnect can return
      // an approximate one that ends short of the goal, and counting that
      // toward a success rate would inflate this baseline against a port
      // whose `rrt_connect` has no approximate-solution path at all.
      if (pdef->hasExactSolution())
      {
        const auto path = std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        problem_out["length"] = path->length();

        json waypoints = json::array();
        moveit::core::RobotState waypoint_state(*state_);
        for (std::size_t i = 0; i < path->getStateCount(); ++i)
        {
          planStateToRobotState(path->getState(i), plan_space.layout, waypoint_state);
          json waypoint = json::object();
          for (const std::string& variable : group->getVariableNames())
            waypoint[variable] = waypoint_state.getVariablePosition(variable);
          waypoints.push_back(waypoint);
        }
        problem_out["path"] = waypoints;
      }

      problems_out.push_back(problem_out);
    }

    return json{ { "space",
                   json{ { "dimension", plan_space.space->getDimension() },
                         { "maximum_extent", maximum_extent },
                         { "longest_valid_segment_length", plan_space.space->getLongestValidSegmentLength() },
                         { "so3_quarter_turn_distance", plan_space.so3_quarter_turn_distance },
                         { "subspaces", plan_space.description } } },
                 { "seed_applied", seed_applied },
                 { "distance_probes", probes_out },
                 { "problems", problems_out } };
  }

  /// Ground truth for `ChompCost::getQuadraticCostInverse()`, requested by
  /// p6-totg in `crates/cspace-planners-chomp/doc/oracle-request-quad-cost-
  /// inv.md`.
  ///
  /// The full inverse matrix, element by element, not a residual. Round 16
  /// already checked residuals (`0.0` on the cofactor path, `1.78e-15` on
  /// the LU path) and that is precisely what cannot settle the question:
  /// Eigen's `MatrixXd` is `Dynamic`, so `.inverse()` is always
  /// `PartialPivLU`, while nalgebra's `try_inverse_mut` switches to a
  /// closed-form cofactor expansion for dimensions `1..=4` and only reaches
  /// `lu::try_invert_to` at `>= 5` (p6-totg verified that boundary against
  /// the pinned nalgebra 0.35.0 source, not its public docs). Two different
  /// decompositions can each produce a small residual and still disagree in
  /// the last bits, which is the whole comparison.
  ///
  /// `ChompCost`'s constructor reads only `getNumPoints()` and
  /// `getDiscretization()` off the trajectory -- no robot model, group, or
  /// joint values -- so the model this oracle already carries serves, and
  /// `joint_number` is passed as 0 because upstream never reads it.
  json chompQuadCostInverse(const json& request) const
  {
    const auto num_points = request.at("num_points").get<std::size_t>();
    const double discretization = request.at("discretization").get<double>();
    const auto derivative_costs = request.at("derivative_costs").get<std::vector<double>>();
    const double ridge_factor = request.at("ridge_factor").get<double>();

    const std::string group_name = request.value("group_name", std::string("panda_arm"));
    const chomp::ChompTrajectory trajectory(model_, num_points, discretization, group_name);
    const chomp::ChompCost cost(trajectory, /*joint_number=*/0, derivative_costs, ridge_factor);

    const Eigen::MatrixXd& inverse = cost.getQuadraticCostInverse();
    json rows = json::array();
    for (Eigen::Index r = 0; r < inverse.rows(); ++r)
    {
      json row = json::array();
      for (Eigen::Index c = 0; c < inverse.cols(); ++c)
        row.push_back(inverse(r, c));
      rows.push_back(row);
    }

    // Echoed rather than left implicit: it is derivable from `num_points`,
    // but a mismatch would mean DIFF_RULE_LENGTH or the boundary formula
    // diverged, which is itself the more interesting finding.
    return json{ { "num_vars_free", static_cast<std::int64_t>(inverse.rows()) }, { "quad_cost_inverse", rows } };
  }

  /// The name of a `moveit_msgs::msg::MoveItErrorCodes` value.
  ///
  /// Switched over the generated named constants rather than over integer
  /// literals: the numeric values are a property of the message definition
  /// this image was built against, and a table of hand-copied integers here
  /// would be a second, silently-diverging definition of them. Only the
  /// codes `ChompPlanner::solve` and `StompPlanningContext::solve` can
  /// actually set are named; anything else is reported as its number so an
  /// unexpected code is visible rather than collapsed into a wrong name.
  static std::string moveItErrorCodeName(std::int32_t val)
  {
    using Codes = moveit_msgs::msg::MoveItErrorCodes;
    switch (val)
    {
      case Codes::SUCCESS:
        return "SUCCESS";
      case Codes::FAILURE:
        return "FAILURE";
      case Codes::PLANNING_FAILED:
        return "PLANNING_FAILED";
      case Codes::INVALID_MOTION_PLAN:
        return "INVALID_MOTION_PLAN";
      case Codes::TIMED_OUT:
        return "TIMED_OUT";
      case Codes::START_STATE_IN_COLLISION:
        return "START_STATE_IN_COLLISION";
      case Codes::GOAL_IN_COLLISION:
        return "GOAL_IN_COLLISION";
      case Codes::GOAL_CONSTRAINTS_VIOLATED:
        return "GOAL_CONSTRAINTS_VIOLATED";
      case Codes::INVALID_GROUP_NAME:
        return "INVALID_GROUP_NAME";
      case Codes::INVALID_GOAL_CONSTRAINTS:
        return "INVALID_GOAL_CONSTRAINTS";
      case Codes::INVALID_ROBOT_STATE:
        return "INVALID_ROBOT_STATE";
      default:
        return "UNNAMED(" + std::to_string(val) + ")";
    }
  }

  /// Densifies `waypoints` to at most `resolution` spacing in `space`'s own
  /// metric -- the same rule, step for step, as the port harnesses'
  /// `densify` (`chomp_benchmark_port.rs`'s `fn densify`, whose body reads
  /// `let steps = ((dist / resolution).ceil() as u64).max(1);` and pushes
  /// `interpolate(from, to, i / steps)` for `i` in `1..=steps`).
  ///
  /// Condition 2 asks whether a returned path is collision-free at a
  /// resolution finer than the planner's own waypoint spacing, so the two
  /// sides have to densify identically or the comparison measures the
  /// densifier. `buildPlanSpace` is what makes that possible: it is the same
  /// weighted compound space the port's `JointModelGroupSpace` is, so
  /// `distance` and `interpolate` agree term by term.
  std::vector<moveit::core::RobotState> densifyPath(const PlanSpace& plan_space,
                                                    const std::vector<moveit::core::RobotState>& waypoints,
                                                    double resolution) const
  {
    std::vector<moveit::core::RobotState> out;
    if (waypoints.empty())
      return out;

    ob::ScopedState<> from(plan_space.space);
    ob::ScopedState<> to(plan_space.space);
    ob::ScopedState<> mid(plan_space.space);

    out.push_back(waypoints.front());
    for (std::size_t i = 0; i + 1 < waypoints.size(); ++i)
    {
      robotStateToPlanState(waypoints[i], plan_space.layout, from.get());
      robotStateToPlanState(waypoints[i + 1], plan_space.layout, to.get());
      const double dist = plan_space.space->distance(from.get(), to.get());
      const auto steps = std::max<std::uint64_t>(1, static_cast<std::uint64_t>(std::ceil(dist / resolution)));
      for (std::uint64_t step = 1; step <= steps; ++step)
      {
        const double t = static_cast<double>(step) / static_cast<double>(steps);
        plan_space.space->interpolate(from.get(), to.get(), t, mid.get());
        moveit::core::RobotState densified(*state_);
        planStateToRobotState(mid.get(), plan_space.layout, densified);
        densified.update();
        out.push_back(densified);
      }
    }
    return out;
  }

  /// Path length in `plan_space`'s metric -- the sum of `space->distance`
  /// over consecutive waypoints, which is what `plan()` reports for the OMPL
  /// baseline (`PathGeometric::length()`) and what the port harnesses sum.
  double planSpacePathLength(const PlanSpace& plan_space,
                             const std::vector<moveit::core::RobotState>& waypoints) const
  {
    ob::ScopedState<> a(plan_space.space);
    ob::ScopedState<> b(plan_space.space);
    double length = 0.0;
    for (std::size_t i = 0; i + 1 < waypoints.size(); ++i)
    {
      robotStateToPlanState(waypoints[i], plan_space.layout, a.get());
      robotStateToPlanState(waypoints[i + 1], plan_space.layout, b.get());
      length += plan_space.space->distance(a.get(), b.get());
    }
    return length;
  }

  /// Condition 2 for one returned path, in both of the port harnesses'
  /// two forms: densified at `resolution`, and at the planner's own returned
  /// waypoints only.
  ///
  /// Validity is `!scene.isStateColliding(state, "", false)` per waypoint --
  /// the upstream method the port's `PlanningScene::is_state_valid` with
  /// `constraints: None` reduces to (its body is `if
  /// self.is_state_colliding(env, request) { return false; }` then `None =>
  /// true`), and the empty group name is what makes it a whole-robot check
  /// on both sides rather than a group-restricted one.
  json condition2(planning_scene::PlanningScene& scene, const PlanSpace& plan_space,
                  const std::vector<moveit::core::RobotState>& waypoints, double resolution,
                  const std::vector<double>& extra_resolutions) const
  {
    const auto invalid_count = [&scene](const std::vector<moveit::core::RobotState>& states) {
      std::size_t invalid = 0;
      for (const moveit::core::RobotState& state : states)
      {
        if (scene.isStateColliding(state, "", false))
          ++invalid;
      }
      return invalid;
    };

    const std::vector<moveit::core::RobotState> densified = densifyPath(plan_space, waypoints, resolution);
    const std::size_t densified_invalid = invalid_count(densified);
    const std::size_t returned_invalid = invalid_count(waypoints);

    json out{
      { "condition2_valid", densified_invalid == 0 },
      { "invalid_waypoint_count", densified_invalid },
      { "condition2_valid_at_returned_waypoints", returned_invalid == 0 },
      { "densified_waypoint_count", densified.size() },
      { "returned_waypoint_count", waypoints.size() },
    };

    // One condition-2 verdict per requested resolution over the *same* path.
    // The plan does not depend on the densification resolution -- it is read
    // only here, after the planner returned -- so this is several verdicts
    // about one path rather than several runs, which is what makes an
    // operating-point sweep affordable on both sides. Absent the request
    // field the loop runs zero times and the object above is unchanged.
    if (!extra_resolutions.empty())
    {
      json by_resolution = json::array();
      for (const double extra : extra_resolutions)
      {
        const std::vector<moveit::core::RobotState> at = densifyPath(plan_space, waypoints, extra);
        const std::size_t at_invalid = invalid_count(at);
        by_resolution.push_back(json{
          { "resolution", extra },
          { "invalid_count", at_invalid },
          { "densified_waypoint_count", at.size() },
          { "valid", at_invalid == 0 },
        });
      }
      out["condition2_by_resolution"] = by_resolution;
    }

    return out;
  }

  /// A request's optional `condition2_resolutions`, the operating-point grid
  /// `condition2` walks in addition to `motion_resolution`.
  ///
  /// Rejected rather than defaulted when malformed, mirroring the port
  /// harnesses' `parse_condition2_resolutions`: a silently dropped grid would
  /// make a sweep report one resolution's verdict under every resolution's
  /// name.
  static std::vector<double> condition2Resolutions(const json& request)
  {
    std::vector<double> out;
    const auto it = request.find("condition2_resolutions");
    if (it == request.end() || it->is_null())
      return out;
    if (!it->is_array())
      throw std::runtime_error("condition2_resolutions must be an array");
    for (const json& entry : *it)
    {
      if (!entry.is_number())
        throw std::runtime_error("condition2_resolutions entries must be numbers");
      const double resolution = entry.get<double>();
      if (!(resolution > 0.0))
        throw std::runtime_error("condition2_resolutions entries must be positive");
      out.push_back(resolution);
    }
    return out;
  }

  /// Reads a request's joint-name -> value map into a full `RobotState`,
  /// starting from the default state so joints outside the planning group
  /// hold the same values here as they do on the port side.
  ///
  /// Shared by `chompPlan` and `stompPlan`; `plan()` keeps its own lambda of
  /// the same shape because it closes over that op's own error prefix.
  moveit::core::RobotState readBenchmarkState(const json& joints, const char* op_name) const
  {
    moveit::core::RobotState robot_state(*state_);
    for (auto it = joints.begin(); it != joints.end(); ++it)
    {
      if (!hasVariable(it.key()))
        throw std::runtime_error(std::string(op_name) + ": unknown joint variable: " + it.key());
      robot_state.setVariablePosition(it.key(), it.value().get<double>());
    }
    robot_state.update();
    return robot_state;
  }

  /// Phase 8's C++ CHOMP baseline: `chomp::ChompPlanner::solve` over the same
  /// request shape the `plan` op consumes, so the identical 500-problem set
  /// emitted by `plan_benchmark_problem_set` drives both baselines and the
  /// port with only the `op` field changed.
  ///
  /// # Why `chomp_motion_planner` and not `chomp_interface`
  ///
  /// `cspace_planners_chomp` (the `chomp_interface` package, which carries
  /// the pluginlib entry point) is not in this image. It does not need to
  /// be: `CHOMPPlanningContext::solve` is one forwarding call --
  /// `chomp_interface_->solve(planning_scene_, request_, chomp_interface_->getParams(), res);`
  /// (`chomp_planning_context.cpp:50-53`) -- into `chomp::ChompPlanner::solve`,
  /// and `class CHOMPInterface : public chomp::ChompPlanner`
  /// (`chomp_interface.hpp:48`) adds exactly one thing over its base, a
  /// `loadParams()` that reads `ChompParameters` off the ROS parameter
  /// server. All of the planning -- the 3.0s/0.03s `ChompTrajectory`, the
  /// joint-space-goal validation, the trajectory initialization, the
  /// `ChompOptimizer` and its recovery loop -- is in the base class, in
  /// `chomp_motion_planner`, which this image does build and this oracle
  /// already links for the `chomp_quad_cost_inverse` op.
  ///
  /// So this op runs upstream's real CHOMP. What it does not reproduce is
  /// the ROS parameter *plumbing*, and that turns out to matter: the two
  /// upstream sources of a default disagree. `ChompParameters`'s own
  /// constructor (`chomp_parameters.cpp`) sets `max_iterations_ = 50` and
  /// `planning_time_limit_ = 6.0`, while `CHOMPInterface::loadParams`
  /// (`chomp_interface.cpp`) declares its fallbacks as
  /// `get_parameter_or("chomp.max_iterations", ..., 200)` and
  /// `get_parameter_or("chomp.planning_time_limit", ..., 10.0)`. A
  /// parameter-free `move_group` therefore runs CHOMP at 200 iterations,
  /// not the 50 a bare `ChompParameters` gives. Both are requestable here
  /// (`chomp.max_iterations`), and the resolved values are echoed back in
  /// `config` so a run's own output says which it was.
  ///
  /// # The clock
  ///
  /// `planning_time_limit_` is wall-clock: `ChompOptimizer::optimize` breaks
  /// out when it elapses, so leaving it at either default makes the success
  /// rate a measurement of how loaded this machine was. It is a request
  /// field here for exactly that reason, and the sweep sets it far beyond
  /// reach so that `max_iterations_` -- a work bound -- is what terminates.
  json chompPlan(const json& request)
  {
    const std::string group_name = request.at("group").get<std::string>();
    const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_name);
    if (!group)
      throw std::runtime_error("chomp_plan: unknown joint model group: " + group_name);

    PlanSpace plan_space = buildPlanSpace(group);
    plan_space.space->setup();
    const double motion_resolution = request.at("motion_resolution").get<double>();
    const std::vector<double> condition2_resolutions = condition2Resolutions(request);

    const json cfg = request.value("chomp", json::object());
    chomp::ChompParameters params;
    params.max_iterations_ = cfg.value("max_iterations", params.max_iterations_);
    params.planning_time_limit_ = cfg.value("planning_time_limit", params.planning_time_limit_);

    auto scene = std::make_shared<planning_scene::PlanningScene>(model_);
    addRequestObjects(*scene->getWorldNonConst(), request);

    chomp::ChompPlanner planner;

    json problems_out = json::array();
    for (const json& problem : request.value("problems", json::array()))
    {
      const moveit::core::RobotState start_state = readBenchmarkState(problem.at("start"), "chomp_plan");
      const moveit::core::RobotState goal_state = readBenchmarkState(problem.at("goal"), "chomp_plan");
      scene->getCurrentStateNonConst() = start_state;

      planning_interface::MotionPlanRequest req;
      req.group_name = group_name;
      moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);
      req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, group));
      req.allowed_planning_time = params.planning_time_limit_;

      // `CHOMPPlannerManager::getPlanningContext` hands the context a
      // hybrid-collision *diff* of the scene, not the scene itself
      // (`chomp_plugin.cpp:94-95`: `ps = planning_scene->diff();` then
      // `ps->allocateCollisionDetector(CollisionDetectorAllocatorHybrid::create());`),
      // and that is load-bearing rather than incidental: `ChompOptimizer`'s
      // constructor casts the scene's *current* collision environment,
      // `planning_scene->getCollisionEnv(planning_scene->getCollisionDetectorName())`,
      // down to `CollisionEnvHybrid` and gives up when the cast fails
      // (`chomp_optimizer.cpp:76-78`) -- so it is the allocation above, which
      // is what makes that name "HYBRID", that decides the cast. A scene left
      // on the default detector makes `isInitialized()` false and
      // `ChompPlanner::solve` return PLANNING_FAILED without ever optimizing.
      // Measured: without this line all 3 problems of a floor_wall smoke set
      // returned PLANNING_FAILED in 1.6s total.
      //
      // The diff is per problem because the plugin builds one per request. The
      // undiffed `scene` keeps its default detector and is what `condition2`
      // checks against, so the validity verdict stays the FCL one both the
      // port and every other op in this file use, rather than silently
      // becoming a distance-field verdict for this planner alone.
      planning_scene::PlanningScenePtr hybrid_scene = scene->diff();
      hybrid_scene->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorHybrid::create());

      planning_interface::MotionPlanDetailedResponse res;
      planner.solve(hybrid_scene, req, params, res);

      const bool solved = res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      json problem_out{ { "id", problem.at("id") },
                        { "solved", solved },
                        { "error_code", res.error_code.val },
                        { "failure", moveItErrorCodeName(res.error_code.val) } };

      if (solved && !res.trajectory.empty() && res.trajectory[0])
      {
        std::vector<moveit::core::RobotState> waypoints;
        waypoints.reserve(res.trajectory[0]->getWayPointCount());
        for (std::size_t i = 0; i < res.trajectory[0]->getWayPointCount(); ++i)
          waypoints.push_back(res.trajectory[0]->getWayPoint(i));

        problem_out["length"] = planSpacePathLength(plan_space, waypoints);
        problem_out.update(
            condition2(*scene, plan_space, waypoints, motion_resolution, condition2_resolutions));
      }
      problems_out.push_back(problem_out);
    }

    return json{ { "planner_rng_seed", plannerRngSeedJson() },
                 { "config",
                   { { "max_iterations", params.max_iterations_ },
                     { "planning_time_limit", params.planning_time_limit_ },
                     { "trajectory_initialization_method", params.trajectory_initialization_method_ },
                     { "use_stochastic_descent", params.use_stochastic_descent_ },
                     { "enable_failure_recovery", params.enable_failure_recovery_ },
                     { "max_recovery_attempts", params.max_recovery_attempts_ },
                     { "learning_rate", params.learning_rate_ },
                     { "smoothness_cost_weight", params.smoothness_cost_weight_ },
                     { "obstacle_cost_weight", params.obstacle_cost_weight_ },
                     { "collision_threshold", params.collision_threshold_ },
                     { "min_clearance", params.min_clearance_ },
                     { "joint_update_limit", params.joint_update_limit_ } } },
                 { "problems", problems_out } };
  }

  /// Phase 8's C++ STOMP baseline: `stomp_moveit::StompPlanningContext::solve`,
  /// the same class the `stomp_moveit/StompPlanner` plugin instantiates,
  /// over the same request shape as `chompPlan`.
  ///
  /// # Where this code comes from
  ///
  /// `moveit_planners_stomp` is not one of this image's built packages, but
  /// its sources ship inside it (the whole moveit2 tree is copied to
  /// `/ws/src/moveit2` at the pinned SHA and left there), so the oracle
  /// compiles `stomp_moveit_planning_context.cpp` directly and generates
  /// `stomp_moveit_parameters.hpp` from upstream's own
  /// `res/stomp_moveit.yaml` -- see this project's `CMakeLists.txt`. The
  /// planning code is upstream's, unmodified, at the same SHA as everything
  /// else the oracle answers from; only the build system is this repo's.
  ///
  /// # This is the plugin's own path, including the null publisher
  ///
  /// `StompPlanningContext` takes its `Params` from
  /// `stomp_moveit::ParamListener`, which is exactly how
  /// `stomp_moveit_planner_plugin.cpp` builds it. That plugin then sets a
  /// path publisher *only* when `path_marker_topic` is non-empty, and the
  /// yaml's default for it is `""` -- so a default-configured plugin leaves
  /// the publisher null, `visualization::getIterationPathPublisher` returns
  /// a no-op for a null publisher by its own explicit `if (marker_publisher
  /// == nullptr)` branch, and this op's null publisher is not a deviation
  /// but the default configuration.
  ///
  /// # The clock
  ///
  /// `StompPlanningContext::solve` arms `cv.wait_for(lock,
  /// duration<double>(req.allowed_planning_time))` on a separate thread and
  /// calls `stomp->cancel()` when it fires, so `allowed_planning_time` is a
  /// wall clock and leaving it at `move_group`'s 5.0s default would make the
  /// success rate a property of this machine. It is taken from the request
  /// and the sweep sets it far beyond reach, leaving `num_iterations`
  /// (upstream's own default, 1000) as the terminating bound. The distinction
  /// survives into the output: upstream reports `TIMED_OUT` when the
  /// timeout future has fired and `PLANNING_FAILED` otherwise, and this op
  /// reports that code verbatim, so a run that silently became
  /// clock-bounded says so in its own failure counts.
  json stompPlan(const json& request)
  {
    const std::string group_name = request.at("group").get<std::string>();
    const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_name);
    if (!group)
      throw std::runtime_error("stomp_plan: unknown joint model group: " + group_name);

    PlanSpace plan_space = buildPlanSpace(group);
    plan_space.space->setup();
    const double motion_resolution = request.at("motion_resolution").get<double>();
    const std::vector<double> condition2_resolutions = condition2Resolutions(request);

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    static int node_counter = 0;
    auto node = std::make_shared<rclcpp::Node>("stomp_plan_oracle_" + std::to_string(node_counter++));
    stomp_moveit::ParamListener param_listener(node, "");
    const stomp_moveit::Params params = param_listener.get_params();

    auto scene = std::make_shared<planning_scene::PlanningScene>(model_);
    addRequestObjects(*scene->getWorldNonConst(), request);

    const double allowed_planning_time = request.at("stomp").at("allowed_planning_time").get<double>();

    json problems_out = json::array();
    for (const json& problem : request.value("problems", json::array()))
    {
      const moveit::core::RobotState start_state = readBenchmarkState(problem.at("start"), "stomp_plan");
      const moveit::core::RobotState goal_state = readBenchmarkState(problem.at("goal"), "stomp_plan");
      scene->getCurrentStateNonConst() = start_state;

      planning_interface::MotionPlanRequest req;
      req.group_name = group_name;
      moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);
      req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, group));
      req.allowed_planning_time = allowed_planning_time;

      stomp_moveit::StompPlanningContext context("stomp", group_name, params);
      context.setPlanningScene(scene);
      context.setMotionPlanRequest(req);

      planning_interface::MotionPlanResponse res;
      context.solve(res);

      const bool solved = res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      json problem_out{ { "id", problem.at("id") },
                        { "solved", solved },
                        { "error_code", res.error_code.val },
                        { "failure", moveItErrorCodeName(res.error_code.val) } };

      if (solved && res.trajectory)
      {
        std::vector<moveit::core::RobotState> waypoints;
        waypoints.reserve(res.trajectory->getWayPointCount());
        for (std::size_t i = 0; i < res.trajectory->getWayPointCount(); ++i)
          waypoints.push_back(res.trajectory->getWayPoint(i));

        problem_out["length"] = planSpacePathLength(plan_space, waypoints);
        problem_out.update(
            condition2(*scene, plan_space, waypoints, motion_resolution, condition2_resolutions));
      }
      problems_out.push_back(problem_out);
    }

    return json{ { "planner_rng_seed", plannerRngSeedJson() },
                 { "config",
                   { { "num_iterations", params.num_iterations },
                     { "num_iterations_after_valid", params.num_iterations_after_valid },
                     { "num_rollouts", params.num_rollouts },
                     { "max_rollouts", params.max_rollouts },
                     { "num_timesteps", params.num_timesteps },
                     { "exponentiated_cost_sensitivity", params.exponentiated_cost_sensitivity },
                     { "control_cost_weight", params.control_cost_weight },
                     { "delta_t", params.delta_t },
                     { "path_marker_topic", params.path_marker_topic },
                     { "allowed_planning_time", allowed_planning_time } } },
                 { "problems", problems_out } };
  }

  /// Attaches `kdl_kinematics_plugin/KDLKinematicsPlugin` to `group_name`,
  /// once per group, and reports whether the group now has a solver.
  ///
  /// PTP never needs this -- its goal is joint-space. LIN and CIRC do:
  /// `extractMotionPlanInfo` runs IK on the Cartesian goal, and a model
  /// built from URDF+SRDF alone carries no solver, so both returned
  /// `NO_IK_SOLUTION` (`-31`) before this existed. That is worth stating
  /// plainly because PORTING-PLAN.md §122 concluded "no pluginlib
  /// workaround needed" -- true of the three *generators*, which this file
  /// constructs directly, and measured as such in §130. LIN/CIRC's IK
  /// dependency is a separate requirement that conclusion never covered,
  /// and it surfaced only because PTP passing was not accepted as evidence
  /// for the other two.
  ///
  /// A consequence to carry into any LIN/CIRC comparison: the oracle's
  /// waypoints now depend on *this* KDL solver, while the port's
  /// `compute_pose_ik` depends on `cspace-kinematics`. Whatever those two
  /// disagree by is inside the `1e-6` budget before the trajectory code is
  /// reached at all, so a LIN/CIRC divergence is not attributable to the
  /// trajectory port until Phase 4's IK parity is subtracted out.
  ///
  /// The `ParamListener` reads `robot_description_kinematics.<group>` off
  /// the node; generate_parameter_library declares every one of those with
  /// a default, so an otherwise-empty node yields the plugin's documented
  /// defaults rather than failing.
  bool ensureKinematicsSolver(const std::string& group_name)
  {
    const moveit::core::RobotModelPtr& pilz_model = ensurePilzModel();
    const moveit::core::JointModelGroup* group = pilz_model->getJointModelGroup(group_name);
    if (group == nullptr)
      return false;
    if (group->getSolverInstance() != nullptr)
      return true;

    const std::vector<const moveit::core::LinkModel*>& tips = group->getLinkModels();
    if (tips.empty())
      return false;

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    static int node_counter = 0;
    auto node = std::make_shared<rclcpp::Node>("pilz_ik_oracle_" + std::to_string(node_counter++));

    auto loader = std::make_shared<pluginlib::ClassLoader<kinematics::KinematicsBase>>("moveit_core", "kinematics::"
                                                                                                      "KinematicsBase");
    std::shared_ptr<kinematics::KinematicsBase> solver =
        loader->createSharedInstance("kdl_kinematics_plugin/KDLKinematicsPlugin");
    if (!solver->initialize(node, *pilz_model, group_name, pilz_model->getRootLinkName(),
                            { tips.back()->getName() },
                            /*search_discretization=*/0.005))
      return false;

    // The loader must outlive every instance it created, and the node must
    // outlive the plugin that captured it -- both are kept alive here for
    // the process's lifetime rather than at block scope.
    kinematics_loaders_.push_back(loader);
    kinematics_nodes_.push_back(node);

    // Keyed by group *name*: `setKinematicsAllocators` takes
    // `std::map<std::string, SolverAllocatorFn>`, not a pointer-keyed map.
    std::map<std::string, moveit::core::SolverAllocatorFn> allocators;
    allocators[group_name] = [solver](const moveit::core::JointModelGroup*) { return solver; };
    pilz_model->setKinematicsAllocators(allocators);
    return group->getSolverInstance() != nullptr;
  }

  /// The `pilz_trajectory` op's own `RobotModel`, parsed from the same URDF
  /// and SRDF as `model_` but never shared with any other op.
  ///
  /// `ensureKinematicsSolver` attaches a solver by calling
  /// `setKinematicsAllocators`, which mutates the model in place and has no
  /// inverse -- once a group has a solver, every later op in the same
  /// process sees a model that answers `getSolverInstance()` differently
  /// than it did before. Ops that branch on that (constraint-sampler
  /// selection is the clearest) would then depend on whether a LIN or CIRC
  /// request happened to precede them in the same NDJSON stream, which is
  /// an ordering dependence no fixture records and no replay would
  /// reproduce.
  ///
  /// A private model removes the question instead of answering it: `model_`
  /// is never mutated, so there is no order-dependent state to measure. The
  /// earlier version excluded PTP from attaching a solver for exactly this
  /// reason; that exclusion is now about not doing pointless work rather
  /// than about protecting shared state.
  ///
  /// Built on first use, not in the constructor: most oracle runs never
  /// issue a pilz request, and a second `RobotModel` costs a full URDF/SRDF
  /// walk.
  const moveit::core::RobotModelPtr& ensurePilzModel()
  {
    if (!pilz_model_)
    {
      pilz_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model_, srdf_model_);
      pilz_state_ = std::make_unique<moveit::core::RobotState>(pilz_model_);
      pilz_state_->setToDefaultValues();
      pilz_state_->update();
    }
    return pilz_model_;
  }

  /// Ground truth for Phase 8's pilz completion condition: LIN/PTP/CIRC
  /// trajectories matching this port within `1e-6`. Requested by p1-joints
  /// after they ported `trajectory_functions` and `trajectory_generator`.
  ///
  /// One op for all three generators, selected by `generator`
  /// (`"ptp"`/`"lin"`/`"circ"`), because the three share a constructor shape
  /// and a single inherited entry point -- `TrajectoryGenerator::generate`,
  /// which each subclass specializes only through the private
  /// `extractMotionPlanInfo`/`plan` overrides. Three ops would have been
  /// three copies of the same request assembly.
  ///
  /// # Why the limits ride in the request
  ///
  /// p1-joints' request spec asked for the opposite: leave
  /// `LimitsContainer` out of the JSON and have both sides read the same
  /// `joint_limits.yaml`/`pilz_cartesian_limits.yaml`. That rests on a
  /// premise that does not hold here -- those files exist only under
  /// `third_party/moveit_resources/`, which is gitignored. A fixture whose
  /// meaning depends on a gitignored external checkout is precisely what
  /// `tools/ci/verify-clean-checkout.sh` exists to catch, and neither side
  /// has a YAML reader to begin with (`cspace-planners-pilz`'s `limits.rs`
  /// builds `JointLimitsContainer` programmatically; this file has
  /// nlohmann_json and nothing else).
  ///
  /// Carrying them in the request is also the stronger of the two against
  /// the divergence p1-joints was guarding against. Their worry was a
  /// typo'd limit; but one request drives *both* sides, so a typo lands
  /// identically on each. It can make a case less interesting than
  /// intended -- it cannot manufacture a disagreement. Two independent
  /// readers of one YAML file is the arrangement that can.
  ///
  /// `joint_limits` mirrors `cspace_planners_pilz`'s own `JointLimit`
  /// field-for-field (the union of `joint_limits::JointLimits` and pilz's
  /// `joint_limits_interface::JointLimits` extension), so the port's
  /// `limits::JointLimit` deserializes it without a translation table.
  ///
  /// # Response shape
  ///
  /// Per waypoint: `positions`, `velocities`, `accelerations` (all
  /// joint-name keyed) and `time_from_start` -- not positions alone.
  /// p1-joints' round 18 found two real bugs (`is_state_colliding`'s
  /// inverted return, `push_way_point`'s missing reference-state seed) that
  /// a positions-only trace cannot see: the second corrupts a floating
  /// joint's *state* rather than its interpolated position, and the
  /// backward-difference velocity/acceleration chain is arithmetic no
  /// positions-only fixture exercises at all.
  ///
  /// `sampling_time` is a required request field rather than defaulted.
  /// `generate()`'s C++ default is `0.1` and this port's
  /// `generate_joint_trajectory` runs the same `t += sampling_time`
  /// accumulation, so a `1e-6` comparison is only meaningful if both sides
  /// used one value; requiring it here means neither side's default can
  /// drift the two apart silently.
  json pilzTrajectory(const json& request)
  {
    namespace pilz = pilz_industrial_motion_planner;

    const std::string generator = request.at("generator").get<std::string>();
    const std::string group_name = request.at("group_name").get<std::string>();
    const double sampling_time = request.at("sampling_time").get<double>();

    // Every model/state reference below is the op's private pair, never the
    // shared `model_`/`state_` -- see `ensurePilzModel`'s doc comment.
    const moveit::core::RobotModelPtr& pilz_model = ensurePilzModel();
    const moveit::core::JointModelGroup* group = pilz_model->getJointModelGroup(group_name);
    if (group == nullptr)
      throw std::runtime_error("unknown joint model group: " + group_name);

    pilz::LimitsContainer limits = buildPilzLimits(request);

    // The scene's current state IS the start state: `generate()` reads it
    // back out through `scene->getCurrentState()` and `checkStartState`
    // validates it against `group_name`'s active joints, so seeding the
    // model default here instead would validate a state the request never
    // asked for.
    auto scene = std::make_shared<planning_scene::PlanningScene>(pilz_model);
    moveit::core::RobotState start_state(*pilz_state_);
    for (const auto& [variable, value] : request.at("start_state").items())
      start_state.setVariablePosition(variable, value.get<double>());
    start_state.update();
    scene->setCurrentState(start_state);

    planning_interface::MotionPlanRequest req;
    req.group_name = group_name;
    req.max_velocity_scaling_factor = request.at("max_velocity_scaling_factor").get<double>();
    req.max_acceleration_scaling_factor = request.at("max_acceleration_scaling_factor").get<double>();
    moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

    req.goal_constraints.push_back(buildPilzGoalConstraints(request.at("goal"), group, pilz_model));

    // CIRC alone needs a path constraint to name its interim/centre point;
    // the other two never read `path_constraints`.
    if (request.contains("path_constraint"))
    {
      const json& path = request.at("path_constraint");
      moveit_msgs::msg::Constraints constraint;
      constraint.name = path.at("name").get<std::string>();  // "interim" or "center"
      moveit_msgs::msg::PositionConstraint position_constraint;
      position_constraint.link_name = path.at("link_name").get<std::string>();
      const auto point = path.at("position").get<std::array<double, 3>>();
      geometry_msgs::msg::Pose primitive_pose;
      primitive_pose.position.x = point[0];
      primitive_pose.position.y = point[1];
      primitive_pose.position.z = point[2];
      primitive_pose.orientation.w = 1.0;
      position_constraint.constraint_region.primitive_poses.push_back(primitive_pose);
      constraint.position_constraints.push_back(position_constraint);
      req.path_constraints = constraint;
    }

    // POLYLINE reads the same `path_constraints` field, but as a *list* of
    // via poses rather than CIRC's single interim/centre point -- one
    // `position_constraints` entry per waypoint, each carrying a full pose
    // (`TrajectoryGeneratorPolyline::extractMotionPlanInfo` composes
    // `primitive_poses[0]`'s position and orientation with
    // `target_point_offset`). `smoothness_level` is a top-level
    // `MotionPlanRequest` field upstream, not part of the constraint.
    //
    // `header.frame_id` is left empty on both the waypoints and the goal, so
    // upstream falls back to the model frame and its
    // `scene->getFrameTransform(frame_id)` left-multiplication is the
    // identity -- which is what makes these waypoints directly comparable to
    // the port's already-frame-resolved ones.
    if (request.contains("path_waypoints"))
    {
      moveit_msgs::msg::Constraints constraint;
      for (const json& waypoint : request.at("path_waypoints"))
      {
        moveit_msgs::msg::PositionConstraint position_constraint;
        const auto point = waypoint.at("position").get<std::array<double, 3>>();
        const auto quat = waypoint.at("orientation").get<std::array<double, 4>>();
        geometry_msgs::msg::Pose primitive_pose;
        primitive_pose.position.x = point[0];
        primitive_pose.position.y = point[1];
        primitive_pose.position.z = point[2];
        primitive_pose.orientation.x = quat[0];
        primitive_pose.orientation.y = quat[1];
        primitive_pose.orientation.z = quat[2];
        primitive_pose.orientation.w = quat[3];
        position_constraint.constraint_region.primitive_poses.push_back(primitive_pose);
        constraint.position_constraints.push_back(position_constraint);
      }
      req.path_constraints = constraint;
    }
    if (request.contains("smoothness_level"))
      req.smoothness_level = request.at("smoothness_level").get<double>();

    // PTP is excluded because it never consults a solver, not because
    // attaching one would leak: `ensurePilzModel` gives this op a private
    // model, so nothing outside it can observe the attachment.
    if ((generator == "lin" || generator == "circ" || generator == "polyline") &&
        !ensureKinematicsSolver(group_name))
      throw std::runtime_error("no kinematics solver could be attached to group " + group_name +
                               ", which " + generator + " needs for its Cartesian goal");

    std::unique_ptr<pilz::TrajectoryGenerator> trajectory_generator;
    if (generator == "ptp")
      trajectory_generator = std::make_unique<pilz::TrajectoryGeneratorPTP>(pilz_model, limits, group_name);
    else if (generator == "lin")
      trajectory_generator = std::make_unique<pilz::TrajectoryGeneratorLIN>(pilz_model, limits, group_name);
    else if (generator == "circ")
      trajectory_generator = std::make_unique<pilz::TrajectoryGeneratorCIRC>(pilz_model, limits, group_name);
    else if (generator == "polyline")
      trajectory_generator = moveit_oracle::makePilzPolylineGenerator(pilz_model, limits, group_name);
    else
      throw std::runtime_error("unsupported generator: " + generator);

    planning_interface::MotionPlanResponse res;
    trajectory_generator->generate(scene, req, res, sampling_time);

    // `res.planning_time` is deliberately not reported. It is a wall-clock
    // reading, so it differs on every run of the same request -- measured
    // here as 0.001528053 then 0.001346184 for an otherwise byte-identical
    // LIN response. A fixture that carries it drifts on every
    // `verify-fixture-replay.sh` run, and the only way to keep it would be
    // an `ignore_result_fields_by_id` entry on every id of every pilz
    // fixture, now and forever. That is precisely the "trusting every
    // future fixture author to remember" failure that script's own header
    // rejects, so the field does not exist rather than being excluded
    // per-fixture. Nothing consumes it: no parity test can assert a C++
    // stopwatch against a Rust one.
    json out{ { "error_code", res.error_code.val } };

    // A non-SUCCESS `generate()` clears the trajectory rather than throwing
    // (`unittest_trajectory_generator_ptp.cpp:207-214`), so a null
    // `waypoints` here is a real, comparable outcome -- the port must
    // reject the same requests -- not a capture failure.
    if (res.trajectory == nullptr || res.trajectory->empty())
    {
      out["waypoints"] = nullptr;
      return out;
    }

    out["waypoints"] = serializePilzWaypoints(*res.trajectory, group);
    out["group_variable_names"] = group->getVariableNames();
    return out;
  }

  /// `joint_limits`/`cartesian_limits` -> `LimitsContainer`, shared by
  /// `pilz_trajectory` and `pilz_blend`.
  ///
  /// Extracted rather than copied when `pilz_blend` landed: the blender
  /// takes the same `LimitsContainer` its generators do
  /// (`TrajectoryBlenderTransitionWindow`'s only constructor argument), and
  /// two independent readers of one JSON shape is exactly the arrangement
  /// `pilzTrajectory`'s own "Why the limits ride in the request" note
  /// rejects for the two *sides* of a comparison -- the same argument
  /// applies to two ops on this side.
  static pilz_industrial_motion_planner::LimitsContainer buildPilzLimits(const json& request)
  {
    namespace pilz = pilz_industrial_motion_planner;

    pilz::JointLimitsContainer joint_limits;
    for (const auto& [joint_name, limit_json] : request.at("joint_limits").items())
    {
      pilz::JointLimit limit;
      limit.has_position_limits = limit_json.value("has_position_limits", false);
      limit.min_position = limit_json.value("min_position", 0.0);
      limit.max_position = limit_json.value("max_position", 0.0);
      limit.has_velocity_limits = limit_json.value("has_velocity_limits", false);
      limit.max_velocity = limit_json.value("max_velocity", 0.0);
      limit.has_acceleration_limits = limit_json.value("has_acceleration_limits", false);
      limit.max_acceleration = limit_json.value("max_acceleration", 0.0);
      limit.has_deceleration_limits = limit_json.value("has_deceleration_limits", false);
      limit.max_deceleration = limit_json.value("max_deceleration", 0.0);
      limit.has_jerk_limits = limit_json.value("has_jerk_limits", false);
      limit.max_jerk = limit_json.value("max_jerk", 0.0);
      limit.has_effort_limits = limit_json.value("has_effort_limits", false);
      limit.max_effort = limit_json.value("max_effort", 0.0);
      limit.angle_wraparound = limit_json.value("angle_wraparound", false);
      if (!joint_limits.addLimit(joint_name, limit))
        throw std::runtime_error("JointLimitsContainer::addLimit rejected joint " + joint_name);
    }

    pilz::LimitsContainer limits;
    limits.setJointLimits(joint_limits);
    if (request.contains("cartesian_limits"))
    {
      const json& cart = request.at("cartesian_limits");
      cartesian_limits::Params params;
      params.max_trans_vel = cart.at("max_trans_vel").get<double>();
      params.max_trans_acc = cart.at("max_trans_acc").get<double>();
      params.max_trans_dec = cart.at("max_trans_dec").get<double>();
      params.max_rot_vel = cart.at("max_rot_vel").get<double>();
      limits.setCartesianLimits(params);
    }
    return limits;
  }

  /// A discriminated goal, not upstream's `Constraints` list.
  /// `checkGoalConstraints` (`trajectory_generator.cpp:221-254`) accepts
  /// exactly one constraint of exactly one kind, so this shape produces the
  /// identical accepted set without a JSON encoding of the ambiguous
  /// multi-list message.
  moveit_msgs::msg::Constraints buildPilzGoalConstraints(const json& goal,
                                                         const moveit::core::JointModelGroup* group,
                                                         const moveit::core::RobotModelPtr& pilz_model) const
  {
    const std::string goal_kind = goal.at("kind").get<std::string>();
    if (goal_kind == "joint")
    {
      moveit::core::RobotState goal_state(*pilz_state_);
      for (const auto& [variable, value] : goal.at("joints").items())
        goal_state.setVariablePosition(variable, value.get<double>());
      goal_state.update();
      return kinematic_constraints::constructGoalConstraints(goal_state, group);
    }
    if (goal_kind == "cartesian")
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = pilz_model->getModelFrame();
      const auto position = goal.at("position").get<std::array<double, 3>>();
      const auto orientation = goal.at("orientation").get<std::array<double, 4>>();
      pose.pose.position.x = position[0];
      pose.pose.position.y = position[1];
      pose.pose.position.z = position[2];
      pose.pose.orientation.x = orientation[0];
      pose.pose.orientation.y = orientation[1];
      pose.pose.orientation.z = orientation[2];
      pose.pose.orientation.w = orientation[3];
      return kinematic_constraints::constructGoalConstraints(goal.at("link_name").get<std::string>(), pose,
                                                             goal.value("tolerance_position", 1e-4),
                                                             goal.value("tolerance_angle", 1e-4));
    }
    throw std::runtime_error("unsupported goal kind: " + goal_kind);
  }

  /// One waypoint array in `pilz_trajectory`'s established per-waypoint
  /// shape. `pilz_blend` emits three of these, so the shape is defined once.
  static json serializePilzWaypoints(const robot_trajectory::RobotTrajectory& trajectory,
                                     const moveit::core::JointModelGroup* group)
  {
    json waypoints_out = json::array();
    for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i)
    {
      const moveit::core::RobotState& waypoint = trajectory.getWayPoint(i);
      json positions = json::object();
      json velocities = json::object();
      json accelerations = json::object();
      for (const std::string& variable : group->getVariableNames())
      {
        positions[variable] = waypoint.getVariablePosition(variable);
        velocities[variable] = waypoint.getVariableVelocity(variable);
        accelerations[variable] = waypoint.getVariableAcceleration(variable);
      }
      waypoints_out.push_back(json{ { "positions", positions },
                                    { "velocities", velocities },
                                    { "accelerations", accelerations },
                                    { "time_from_start", trajectory.getWayPointDurationFromStart(i) } });
    }
    return waypoints_out;
  }

  /// One `pilz_blend` segment: build the generator named by
  /// `segment["generator"]` and run it from whatever state `scene` currently
  /// holds.
  ///
  /// The start state is the scene's, never a field of `segment` -- that is
  /// what makes chaining possible (see `pilzBlend`'s doc comment for why
  /// segment 2 must start from segment 1's own last waypoint rather than
  /// from an independently supplied pose).
  planning_interface::MotionPlanResponse
  generatePilzSegment(const planning_scene::PlanningScenePtr& scene, const moveit::core::RobotModelPtr& pilz_model,
                      const moveit::core::JointModelGroup* group, const std::string& group_name,
                      const pilz_industrial_motion_planner::LimitsContainer& limits, const json& segment,
                      double sampling_time)
  {
    namespace pilz = pilz_industrial_motion_planner;

    const std::string generator = segment.at("generator").get<std::string>();

    planning_interface::MotionPlanRequest req;
    req.group_name = group_name;
    req.max_velocity_scaling_factor = segment.at("max_velocity_scaling_factor").get<double>();
    req.max_acceleration_scaling_factor = segment.at("max_acceleration_scaling_factor").get<double>();
    moveit::core::robotStateToRobotStateMsg(scene->getCurrentState(), req.start_state);
    req.goal_constraints.push_back(buildPilzGoalConstraints(segment.at("goal"), group, pilz_model));

    if ((generator == "lin" || generator == "circ") && !ensureKinematicsSolver(group_name))
      throw std::runtime_error("no kinematics solver could be attached to group " + group_name + ", which " +
                               generator + " needs for its Cartesian goal");

    std::unique_ptr<pilz::TrajectoryGenerator> trajectory_generator;
    if (generator == "ptp")
      trajectory_generator = std::make_unique<pilz::TrajectoryGeneratorPTP>(pilz_model, limits, group_name);
    else if (generator == "lin")
      trajectory_generator = std::make_unique<pilz::TrajectoryGeneratorLIN>(pilz_model, limits, group_name);
    else if (generator == "circ")
      trajectory_generator = std::make_unique<pilz::TrajectoryGeneratorCIRC>(pilz_model, limits, group_name);
    else
      throw std::runtime_error("unsupported generator: " + generator);

    planning_interface::MotionPlanResponse res;
    trajectory_generator->generate(scene, req, res, sampling_time);
    return res;
  }

  /// `TrajectoryBlenderTransitionWindow::blend` -- the transition-window
  /// blender's only external measurement.
  ///
  /// Requested by p1-joints in
  /// `crates/cspace-planners-pilz/doc/oracle-request-pilz-blend.md` after
  /// they ported the 966-line blender with twelve tests, none of which
  /// compares a single number against upstream (PORTING-PLAN.md §185).
  ///
  /// # Segment 2 starts from segment 1's own last waypoint
  ///
  /// The request carries one `start_state` and a two-entry `segments` array,
  /// not a start state per segment, because upstream's real call sequence
  /// chains: `plan_components_builder.cpp:75-105` sets
  /// `blend_request.first_trajectory = traj_tail_`, the literal trajectory a
  /// prior planning step produced. Two independently-solved IK results for
  /// the same Cartesian corner can differ by more than `validateRequest`'s
  /// `isRobotStateEqual` boundary check tolerates on `panda_arm`'s redundant
  /// kinematics -- the request document measured `5.7e-6` per joint between
  /// two solvers converging on one pose. Chaining removes that failure mode
  /// by construction instead of by choosing parameters careful enough to
  /// dodge it.
  ///
  /// # Why there is no `blend_align_index` field
  ///
  /// The request document asked for three integer fields:
  /// `first_intersection_index`, `second_intersection_index`, and
  /// `blend_align_index`. Two of the three are emitted; the third is not,
  /// and the difference is not an oversight.
  ///
  /// `searchIntersectionPoints` and `determineTrajectoryAlignment` are both
  /// **private** members of `TrajectoryBlenderTransitionWindow`, so no
  /// caller can read their outputs directly. But the first two indices are
  /// still genuine upstream products, because `blend()` truncates its
  /// response trajectories at exactly those indices and the truncation is
  /// exact, not approximate (`trajectory_blender_transition_window.cpp`):
  /// the `[0, first_intersection_index)` copy loop makes
  /// `res.first_trajectory`'s waypoint count *equal*
  /// `first_intersection_index`, and the
  /// `[second_intersection_index + 1, count)` copy loop makes
  /// `res.second_trajectory`'s count equal the input's count minus
  /// `second_intersection_index + 1`. Inverting two exact copy loops
  /// recovers what upstream computed; it does not recompute it.
  ///
  /// `blend_align_index` has no such witness. It reaches only
  /// `blendTrajectoryCartesian`'s sampling arithmetic and never determines a
  /// boundary that survives into the response shape. Emitting it would mean
  /// running `determineTrajectoryAlignment`'s six lines *here*, in this
  /// file, from the two indices and a waypoint count -- and then the port
  /// would be compared against this file's re-implementation rather than
  /// against upstream's execution. That is a fixture measuring its own
  /// helper, which is the one thing an oracle exists not to do. The Rust
  /// side derives `blend_align_index` with its own
  /// `determine_trajectory_alignment` from the two indices emitted here, as
  /// the request document's own stated alternative allows, and the branch it
  /// chose is checked where it actually has consequences: through
  /// `blend_trajectory`'s waypoints, which are compared in full. The request
  /// document's case B exists precisely to make that branch change the
  /// output.
  ///
  /// `first_trajectory_input_waypoint_count` and
  /// `second_trajectory_input_waypoint_count` are emitted so the Rust side
  /// can perform that derivation, and so a segment-generation divergence
  /// (different waypoint counts before blending) reads as its own finding
  /// rather than as a blend mismatch.
  json pilzBlend(const json& request)
  {
    namespace pilz = pilz_industrial_motion_planner;

    const std::string group_name = request.at("group_name").get<std::string>();
    const std::string link_name = request.at("link_name").get<std::string>();
    const double sampling_time = request.at("sampling_time").get<double>();
    const double blend_radius = request.at("blend_radius").get<double>();

    const moveit::core::RobotModelPtr& pilz_model = ensurePilzModel();
    const moveit::core::JointModelGroup* group = pilz_model->getJointModelGroup(group_name);
    if (group == nullptr)
      throw std::runtime_error("unknown joint model group: " + group_name);

    const json& segments = request.at("segments");
    if (segments.size() != 2)
      throw std::runtime_error("pilz_blend needs exactly 2 segments, got " + std::to_string(segments.size()));

    const pilz::LimitsContainer limits = buildPilzLimits(request);

    auto scene = std::make_shared<planning_scene::PlanningScene>(pilz_model);
    moveit::core::RobotState start_state(*pilz_state_);
    for (const auto& [variable, value] : request.at("start_state").items())
      start_state.setVariablePosition(variable, value.get<double>());
    start_state.update();
    scene->setCurrentState(start_state);

    // A failure before the blend is not a finding about the blender, so the
    // response says which stage failed rather than reporting a bare
    // non-SUCCESS code the port would have to guess the origin of.
    planning_interface::MotionPlanResponse first =
        generatePilzSegment(scene, pilz_model, group, group_name, limits, segments.at(0), sampling_time);
    if (first.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS || first.trajectory == nullptr ||
        first.trajectory->empty())
      return json{ { "error_code", first.error_code.val }, { "stage", "segment1" } };

    // The chaining step: segment 2 plans from segment 1's own last waypoint,
    // not from a re-derived pose. See this function's doc comment.
    scene->setCurrentState(first.trajectory->getLastWayPoint());
    planning_interface::MotionPlanResponse second =
        generatePilzSegment(scene, pilz_model, group, group_name, limits, segments.at(1), sampling_time);
    if (second.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS || second.trajectory == nullptr ||
        second.trajectory->empty())
      return json{ { "error_code", second.error_code.val }, { "stage", "segment2" } };

    const std::size_t first_input_count = first.trajectory->getWayPointCount();
    const std::size_t second_input_count = second.trajectory->getWayPointCount();

    pilz::TrajectoryBlendRequest blend_request;
    blend_request.group_name = group_name;
    blend_request.link_name = link_name;
    blend_request.first_trajectory = first.trajectory;
    blend_request.second_trajectory = second.trajectory;
    blend_request.blend_radius = blend_radius;

    pilz::TrajectoryBlenderTransitionWindow blender(limits);
    pilz::TrajectoryBlendResponse blend_response;
    const bool blended = blender.blend(scene, blend_request, blend_response);

    json out{ { "first_trajectory_input_waypoint_count", first_input_count },
              { "second_trajectory_input_waypoint_count", second_input_count } };

    // `blend()` sets `res.error_code.val` on every path it takes -- SUCCESS
    // at `trajectory_blender_transition_window.cpp:141`, and a specific
    // rejection code in `validateRequest`, the blend-radius-too-large
    // branch, and `generateJointTrajectory`'s failure -- so the field is
    // read directly rather than reconstructed from the bool. The bool is
    // still checked, because agreement between the two is upstream's
    // invariant, not this file's assumption.
    out["error_code"] = blend_response.error_code.val;
    out["stage"] = "blend";
    if (!blended)
      return out;
    out["sampling_time"] = sampling_time;
    out["group_variable_names"] = group->getVariableNames();

    // Recovered by inverting `blend()`'s two exact copy loops, not
    // recomputed -- see this function's doc comment for why that is a
    // genuine upstream measurement while `blend_align_index` would not be.
    const std::size_t first_intersection_index = blend_response.first_trajectory->getWayPointCount();
    const std::size_t second_kept = blend_response.second_trajectory->getWayPointCount();
    if (second_kept > second_input_count)
      throw std::runtime_error("blend kept more second-trajectory waypoints than it was given");
    out["first_intersection_index"] = first_intersection_index;
    out["second_intersection_index"] = second_input_count - second_kept - 1;

    out["first_trajectory"] = serializePilzWaypoints(*blend_response.first_trajectory, group);
    out["blend_trajectory"] = serializePilzWaypoints(*blend_response.blend_trajectory, group);
    out["second_trajectory"] = serializePilzWaypoints(*blend_response.second_trajectory, group);
    return out;
  }

  moveit::core::RobotModelPtr model_;
  std::unique_ptr<moveit::core::RobotState> state_;

  // Joint bounds the current request replaced, and what they were before, in
  // the order they were replaced. `handle`'s `ScopedJointBounds` empties this
  // at the end of every request; it is never non-empty across two of them.
  std::vector<std::pair<moveit::core::JointModel*, std::vector<moveit_msgs::msg::JointLimits>>> replaced_bounds_;

  // The `pilz_trajectory` op's private pair, built on first use by
  // `ensurePilzModel` -- see that function's doc comment for why this op
  // does not share `model_`.
  moveit::core::RobotModelPtr pilz_model_;
  std::unique_ptr<moveit::core::RobotState> pilz_state_;

  // `urdf_model_` is the `ik` op's parsed URDF and `ensurePilzModel`'s
  // input; both are kept so a second `RobotModel` can be built without
  // re-reading and re-parsing the files the constructor already read.
  urdf::ModelInterfaceSharedPtr urdf_model_;
  srdf::ModelSharedPtr srdf_model_;
  // For the `ik` op only.
  KDL::Tree kdl_tree_;
  // Reseed draws between `ik` restart attempts (see `ik()`'s own doc
  // comment). Fixed-seeded for a reproducible oracle run; Phase 4's
  // completion condition never compares a solution's exact value, only
  // whether one was found and whether FK(solution) lands on target, so
  // this need not (and structurally cannot, since it is a wholly separate
  // RNG stream) match cspace-kinematics's own reseed draws.
  //
  // The seed is a startup argument (`--ik-rng-seed`, default 42) rather than
  // a hard-coded constant, because "this port's IK success rate is at least
  // the C++ plugin's" is a comparison of two random variables once
  // `max_restarts > 0`: each side's count is one draw from its own reseed
  // lottery. With 42 hard-coded, the oracle contributes exactly one sample
  // and the comparison cannot distinguish a real deficit from a draw. It is
  // a startup argument and not a per-request field on purpose -- the stream
  // must stay continuous across the cases of one run, exactly as it was when
  // it was a member initializer, so `--ik-rng-seed 42` reproduces every
  // number taken before this knob existed.
  random_numbers::RandomNumberGenerator ik_rng_;

  // For the `plan` op only. `ompl::RNG::setSeed` errors out once any RNG
  // instance exists, so the seed can be applied at most once per process;
  // this records whether that has already happened. See `plan()`'s
  // Determinism section.
  bool ompl_seeded_ = false;



  // For `ensureKinematicsSolver` only: a pluginlib loader unloads its
  // library on destruction, which would leave the group's cached solver
  // pointing into unmapped code, and KDLKinematicsPlugin holds the node it
  // was initialized with. Both outlive the call that made them.
  std::vector<std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>>> kinematics_loaders_;
  std::vector<rclcpp::Node::SharedPtr> kinematics_nodes_;
};

}  // namespace

int main(int argc, char** argv)
{
  std::string urdf_path;
  std::string srdf_path;
  // Default 42 -- the value this was hard-coded to before it became an
  // argument, so omitting the flag reproduces every earlier measurement.
  std::uint32_t ik_rng_seed = 42;
  // Unset means "leave rsl::rng to seed itself from std::random_device",
  // which is upstream's own behaviour and what every op other than
  // chomp_plan/stomp_plan gets. See the seeding block below.
  bool have_planner_rng_seed = false;
  std::uint32_t planner_rng_seed = 0;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--urdf" && i + 1 < argc)
      urdf_path = argv[++i];
    else if (arg == "--srdf" && i + 1 < argc)
      srdf_path = argv[++i];
    else if (arg == "--ik-rng-seed" && i + 1 < argc)
    {
      const std::string value = argv[++i];
      try
      {
        ik_rng_seed = static_cast<std::uint32_t>(std::stoul(value));
      }
      catch (const std::exception&)
      {
        std::cerr << "oracle: --ik-rng-seed expects an unsigned integer, got " << value << '\n';
        return 2;
      }
    }
    else if (arg == "--planner-rng-seed" && i + 1 < argc)
    {
      const std::string value = argv[++i];
      try
      {
        planner_rng_seed = static_cast<std::uint32_t>(std::stoul(value));
        have_planner_rng_seed = true;
      }
      catch (const std::exception&)
      {
        std::cerr << "oracle: --planner-rng-seed expects an unsigned integer, got " << value << '\n';
        return 2;
      }
    }
    else
    {
      std::cerr << "oracle: unrecognized argument " << arg << '\n';
      return 2;
    }
  }
  if (urdf_path.empty() || srdf_path.empty())
  {
    std::cerr << "usage: moveit_oracle --urdf <path> --srdf <path> [--ik-rng-seed <n>] "
                 "[--planner-rng-seed <n>]\n";
    return 2;
  }

  // Seeds `rsl::rng()` -- the single generator both C++ planners draw from
  // (`chomp_plan`'s stochastic descent through `rsl::uniform_real`,
  // `stomp_plan`'s noise through `MultivariateGaussian::sample`) -- and it
  // has to happen here, before anything else in the process runs.
  //
  // `rsl::rng` builds its thread_local `std::mt19937` on the first call and
  // throws on any later call that carries a seed sequence. MoveIt takes that
  // first call for itself: `moveit::getLogger()`'s fallback names the default
  // logger `fmt::format("moveit_{}", rsl::rng()())`
  // (`moveit_core/utils/src/logger.cpp:55`), and the model load logs through
  // it -- the `[moveit_2154718666]` in this oracle's own stderr is that draw.
  // Constructing the `Oracle` first therefore burns the seeding opportunity
  // and leaves the planners running off `std::random_device`, which is a
  // nondeterministic baseline; a per-request seed field cannot fix that
  // because by then it is already too late.
  //
  // So this is a startup argument rather than a request field, and it is
  // applied before the `Oracle` (and so before any `RobotModel`) exists. The
  // logger's own draw then comes out of the seeded stream, which costs one
  // value and makes even the generated logger name reproducible -- a free
  // check that nothing consumed the generator ahead of this line.
  if (have_planner_rng_seed)
  {
    try
    {
      rsl::rng(std::seed_seq{ planner_rng_seed });
      g_planner_rng_seed = planner_rng_seed;
    }
    catch (const std::exception& e)
    {
      std::cerr << "oracle: --planner-rng-seed could not be applied: " << e.what() << '\n';
      return 2;
    }
  }

  std::unique_ptr<Oracle> oracle;
  try
  {
    oracle = std::make_unique<Oracle>(urdf_path, srdf_path, ik_rng_seed);
  }
  catch (const std::exception& e)
  {
    std::cerr << "oracle: startup failed: " << e.what() << '\n';
    return 1;
  }

  // nlohmann::json::dump() emits the shortest round-trippable representation
  // of each double, so the 1e-9 FK comparison in Phase 2 loses nothing on the
  // wire. std::cout's own precision does not apply to dump() output.
  std::string line;
  while (std::getline(std::cin, line))
  {
    if (line.empty())
      continue;

    json response;
    std::uint64_t id = 0;
    try
    {
      const json request = json::parse(line);
      id = request.at("id").get<std::uint64_t>();
      response = json{ { "id", id }, { "ok", true }, { "result", oracle->handle(request) } };
    }
    catch (const std::exception& e)
    {
      response = json{ { "id", id }, { "ok", false }, { "error", std::string(e.what()) } };
    }
    std::cout << response.dump() << std::endl;
  }
  return 0;
}
