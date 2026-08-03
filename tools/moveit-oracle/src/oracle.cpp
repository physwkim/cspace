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
// This binary deliberately links only moveit_core: it never starts a ROS node,
// so a run needs no discovery, no DDS and no /clock.

#include <algorithm>
#include <cmath>
#include <fstream>
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
#include <moveit/distance_field/find_internal_points.hpp>
#include <moveit/distance_field/propagation_distance_field.hpp>
#include <moveit/dynamics_solver/dynamics_solver.hpp>
#include <moveit/kinematic_constraints/kinematic_constraint.hpp>
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

/// Matches the `kind` strings `moveit_model::Diagnostic::UnsupportedLinkGeometry`
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

/// Builds the `shapes::Shape` named by `request["shape"]["type"]` --
/// `"sphere"`, `"box"`, `"cylinder"` or `"mesh"`, the four `bodies::` has a
/// case for. Shared by `shapePoints` and `collisionDistanceFieldTypes`,
/// which both need a shape built from the same wire format.
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

class Oracle
{
public:
  Oracle(const std::string& urdf_path, const std::string& srdf_path)
  {
    const std::string urdf_xml = readFile(urdf_path);
    const std::string srdf_xml = readFile(srdf_path);

    urdf_model_ = urdf::parseURDF(urdf_xml);
    if (!urdf_model_)
      throw std::runtime_error("failed to parse URDF at " + urdf_path);

    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initString(*urdf_model_, srdf_xml))
      throw std::runtime_error("failed to parse SRDF at " + srdf_path);

    model_ = std::make_shared<moveit::core::RobotModel>(urdf_model_, srdf_model);
    state_ = std::make_unique<moveit::core::RobotState>(model_);
    state_->setToDefaultValues();
    state_->update();

    // For the `ik` op: one KDL::Tree built once, `getChain`-sliced per
    // request into whatever group's own base/tip the request names.
    if (!kdl_parser::treeFromUrdfModel(*urdf_model_, kdl_tree_))
      throw std::runtime_error("failed to build a KDL::Tree from the URDF at " + urdf_path);
  }

  json handle(const json& request)
  {
    const std::string op = request.at("op").get<std::string>();
    if (op == "model_info")
      return modelInfo();
    if (op == "fk")
      return fk(request);
    if (op == "jacobian")
      return jacobian(request);
    if (op == "random_states")
      return randomStates(request);
    if (op == "acm")
      return acm();
    if (op == "collision")
      return collision(request);
    if (op == "world")
      return world(request);
    if (op == "frame_transform")
      return frameTransform(request);
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

  /// Ground truth for `moveit-model`'s `LinkModel` collision/visual geometry
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

  /// Ground truth for `moveit-model`'s `JointModelGroup` end-effector fields
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

  /// Ground truth for `moveit-model`'s `JointModelGroup::is_chain`, keyed by
  /// group name.
  json groupIsChain() const
  {
    json out = json::object();
    for (const moveit::core::JointModelGroup* group : model_->getJointModelGroups())
      out[group->getName()] = group->isChain();
    return out;
  }

  /// Ground truth for `moveit-model`'s `JointModelGroup::joint_roots`, keyed
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

  /// Ground truth for `moveit-model`'s `RobotModel::descendant_link_indices`
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

  /// Ground truth for `moveit-model`'s `JointModelGroup::updated_link_names`/
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

  /// Ground truth for `moveit-model`'s `RobotModel::get_common_root`:
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

  /// Apply the request's joint values on top of the model defaults.
  ///
  /// Reset first: leaving the previous case's values in place would make a
  /// result depend on request order, which would quietly hide a disagreement
  /// on any variable the request omits.
  void applyJointValues(const json& request)
  {
    state_->setToDefaultValues();
    const json& values = request.at("joint_values");
    for (auto it = values.begin(); it != values.end(); ++it)
    {
      const std::string& variable = it.key();
      if (!hasVariable(variable))
        throw std::runtime_error("unknown joint variable: " + variable);
      state_->setVariablePosition(variable, it.value().get<double>());
    }
    state_->update();
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

  /// The `AllowedCollisionMatrix` `PlanningScene` itself builds: straight from
  /// the loaded SRDF's `disable_collisions`/`enable_collisions`/
  /// `disable_default_collisions`. Shared by `acm()` (which dumps it) and
  /// `collision()` (which filters both `CollisionEnvFCL` checks with it), so
  /// the two never risk building it two different ways.
  collision_detection::AllowedCollisionMatrix buildAcm() const
  {
    return collision_detection::AllowedCollisionMatrix(*model_->getSRDF());
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
  /// `moveit_kinematics::SolverParams::max_restarts`'s own identical
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
    // `moveit_kinematics::chain::ChainInfo::root_pose_world`).
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
    // `multiplier`/`offset`, matching `moveit_state::RobotState`'s own
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
          const double limit = consistency_limits_mimic[k];
          if (active_continuous[k])
          {
            // `RevoluteJointModel::getVariableRandomPositionsNearBy`
            // (revolute_joint_model.cpp:122-136): a continuous joint samples
            // `near ± distance` unclamped, then wraps into `(-pi, pi]`
            // instead of clamping to the joint's (here: reporting-only,
            // [-pi, pi]) bounds. Clamping here -- as this branch did before
            // this fix -- is the same defect this loop's non-continuous case
            // was fixed for, pointing the other way: it silently narrows the
            // reseed window near the wrap boundary instead of matching
            // upstream's wrap-around sampling. Matches this port's own fix
            // in `near_by_configuration` (crates/moveit-kinematics/src/cart_to_jnt.rs).
            double value = ik_rng_.uniformReal(seed_active[k] - limit, seed_active[k] + limit);
            if (value <= -M_PI || value > M_PI)
            {
              value = std::fmod(value, 2.0 * M_PI);
              if (value <= -M_PI)
                value += 2.0 * M_PI;
              else if (value > M_PI)
                value -= 2.0 * M_PI;
            }
            reseed_active[k] = value;
          }
          else
          {
            reseed_active[k] = ik_rng_.uniformReal(std::max(joint_min[full_i], seed_active[k] - limit),
                                                    std::min(joint_max[full_i], seed_active[k] + limit));
          }
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

  /// Ground truth for the `moveit-collision` differential test: builds an
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

  /// Ground truth for `moveit-collision::ParryCollisionEnv` (PORTING-PLAN.md
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
  /// `moveit_scene::AttachedBody`/`moveit_collision::AttachedBodyGeometry`.
  /// `state_` is a long-lived member reused across requests, so previously
  /// attached bodies are cleared first -- `applyJointValues`'s
  /// `setToDefaultValues()` resets joint positions but, verified by reading
  /// `RobotState::setToDefaultValues()`, never touches attached-body state,
  /// so a stale attachment from an earlier request would otherwise leak into
  /// this one. `attachBody`'s own `pose` parameter (the transform from the
  /// link to the attached body's own frame, upstream's separate level
  /// between the link and its shapes) is always identity here, matching
  /// `moveit_scene::AttachedBody`'s one-level design where `shape_poses` are
  /// already relative to the link frame directly -- see `protocol.rs`'s
  /// `AttachedBodySpec` doc.
  ///
  /// Both `CollisionRequest`s are default-constructed (`contacts = false`):
  /// only the boolean `collision` flag from each call is reported, and
  /// [`Contact`] coordinates are excluded from comparison per §4.5's recorded
  /// verification limit -- see `crates/moveit-collision/tests/collision_parity.rs`
  /// and `tools/moveit-diff`'s own collision comparison for why. Both
  /// `DistanceRequest`s set `enable_signed_distance = true`: a request that
  /// left it `false` could never surface deviation 6 (this port's
  /// single-`query::contact()`-call signed distance versus FCL's `distance`
  /// then, only for a penetrating pair, a second up-to-200-contact `collide`
  /// pass taking the deepest penetration) at all, since an unsigned distance
  /// is clamped to `>= 0` on both sides regardless of that deviation.
  json collision(const json& request)
  {
    applyJointValues(request);

    state_->clearAttachedBodies();
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
      state_->attachBody(id, Eigen::Isometry3d::Identity(), shapes, shape_poses, touch_links, link_name);
    }
    state_->update();

    collision_detection::AllowedCollisionMatrix acm = buildAcm();

    auto world = std::make_shared<collision_detection::World>();
    for (const auto& object_json : request.at("objects"))
    {
      const std::string id = object_json.at("id").get<std::string>();
      const Eigen::Isometry3d pose = fromRowMajor4x4(object_json.at("pose"));
      const json& shape_json = object_json.at("shape");
      const std::string shape_type = shape_json.at("type").get<std::string>();
      std::shared_ptr<shapes::Shape> shape = parseShape(shape_type, shape_json);
      world->addToObject(id, pose, { shape }, { Eigen::Isometry3d::Identity() });
    }

    collision_detection::CollisionEnvFCL env(model_, world);

    collision_detection::CollisionRequest self_req;
    collision_detection::CollisionResult self_res;
    env.checkSelfCollision(self_req, self_res, *state_, acm);

    collision_detection::CollisionRequest robot_req;
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
      { "robot_collision", robot_res.collision },
      { "robot_distance", robot_dres.minimum_distance.distance },
    };
  }

  /// Ground truth for `PlanningScene::frame_transform`/`knows_frame_transform`
  /// (`moveit-scene`). Builds a real `planning_scene::PlanningScene` from
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

    state_->clearAttachedBodies();
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
      state_->attachBody(id, Eigen::Isometry3d::Identity(), shapes, shape_poses, touch_links, link_name,
                          trajectory_msgs::msg::JointTrajectory(), subframes);
    }
    state_->update();

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

  /// Ground truth for the `moveit-collision` World port. Builds a
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

  /// Ground truth for the `moveit-distance-field` `PropagationDistanceField`
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

  /// Ground truth for `find_internal_points_convex`
  /// (`distance_field::findInternalPointsConvex`), the piece of
  /// `moveit-distance-field`'s shape-to-obstacle-points path that the
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

  /// Ground truth for the `moveit-geometry` STL loader
  /// (`crates/moveit-geometry/src/stl.rs`). Calls
  /// `shapes::createMeshFromResource` exactly as `RobotModel::constructShape`
  /// does for a URDF `<mesh>` element -- real Assimp, real
  /// `aiProcess_JoinIdenticalVertices` merging, real `package://` resolution
  /// against this image's colcon-built `moveit_resources` workspace -- so a
  /// resource path and scale go in and the vertex/triangle counts and
  /// vertex positions Assimp actually produced come out, with no
  /// `RobotModel` construction involved on either side of this op.
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

    return json{ { "vertex_count", mesh->vertex_count },
                 { "triangle_count", mesh->triangle_count },
                 { "vertices", vertices_out } };
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

  /// Ground truth for `moveit-distance-field`'s
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
  /// `attached_body_names_`/`attached_body_link_state_indices_` are always
  /// empty on the oracle side too, for the same reason `moveit-state`'s own
  /// `frame_transform` doc gives: this workspace has no `AttachedBody`
  /// fixture to attach, so both ports observe the same "no attached bodies"
  /// state rather than one port faking a nonexistent case the other cannot
  /// produce.
  json distanceFieldCacheEntry(const json& request)
  {
    applyJointValues(request);

    const std::string group_name = request.at("group").get<std::string>();
    if (!model_->hasJointModelGroup(group_name))
      throw std::runtime_error("unknown group: " + group_name);

    const bool use_acm = request.value("use_acm", true);
    collision_detection::AllowedCollisionMatrix acm = buildAcm();

    collision_detection::CollisionEnvDistanceField env(model_);

    collision_detection::CollisionRequest req;
    req.group_name = group_name;
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

    return out;
  }

  /// Ground truth for `moveit-distance-field`'s `group_state_representation`
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
  /// branch the way `moveit-distance-field`'s `group_state_representation`
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
  /// `moveit-distance-field`'s `group_state_representation` ports (that
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
  ///   count here, never `0`, while `group_state_representation`'s Rust port
  ///   always leaves it empty. Not comparable field-by-field; see the parity
  ///   test's own doc comment for how it handles this.
  /// - `checkCollision` runs the full self/intra-group/environment collision
  ///   pipeline *after* `getGroupStateRepresentation` returns, and that
  ///   pipeline can mutate `closest_distance`/`collision`/`types`/`distances`
  ///   in place for any sphere actually found in collision -- construction
  ///   alone (what the Rust port's own scope covers) never does. Every
  ///   fixture case this op currently drives happens to report `collision:
  ///   false` on every link (nothing overlaps at these joint configurations),
  ///   so those four fields still equal what a from-scratch construction
  ///   would report for this op's committed fixtures specifically -- but
  ///   that is a property of the chosen joint values, not a guarantee this
  ///   op's contract makes in general.
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
  json groupStateRepresentation(const json& request)
  {
    applyJointValues(request);

    const std::string group_name = request.at("group").get<std::string>();
    if (!model_->hasJointModelGroup(group_name))
      throw std::runtime_error("unknown group: " + group_name);

    const bool use_acm = request.value("use_acm", true);
    collision_detection::AllowedCollisionMatrix acm = buildAcm();

    collision_detection::CollisionEnvDistanceField env(model_);

    collision_detection::CollisionRequest req;
    req.group_name = group_name;
    collision_detection::CollisionResult res;
    if (use_acm)
      env.checkCollision(req, res, *state_, acm);
    else
      env.checkCollision(req, res, *state_);

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

        const collision_detection::GradientInfo& g = gsr->gradients_[i];
        json types_out = json::array();
        for (collision_detection::CollisionType t : g.types)
          types_out.push_back(static_cast<int>(t));
        link_out["gradient"] = json{ { "closest_distance", g.closest_distance },
                                      { "collision", g.collision },
                                      { "types", types_out },
                                      { "distances", g.distances },
                                      { "sphere_radii", g.sphere_radii },
                                      { "joint_name", g.joint_name },
                                      { "sphere_locations_count", g.sphere_locations.size() } };
      }
      links_out.push_back(link_out);
    }

    return json{ { "group_name", group_name }, { "links", links_out } };
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
      shape_ptrs.push_back(parseShape(shape_json.at("type").get<std::string>(), shape_json));
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

  /// Ground truth for the `moveit-trajectory` `ruckig_smoothing` port. Builds
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
  /// and moveit-trajectory does not port it (D1).
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

  /// Ground truth for the `moveit-trajectory` `Path`/`Trajectory` port (the
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

  /// Ground truth for the `moveit-trajectory`
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

  json totgRobotTrajectoryCase(const std::string& group_name, const json& c)
  {
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

  /// Ground truth for the `moveit-constraints` `KinematicConstraintSet` port.
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
  /// (see `moveit-constraints`' own `KinematicConstraintSet::decide`
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

  /// Ground truth for moveit-octomap's port of octomap 1.9.7 (see
  /// crates/moveit-octomap/src/*.rs's provenance comments): builds a
  /// throwaway `octomap::OcTree` local to this call (ignoring model_/state_,
  /// same pattern as shapePoints above), replays a request-described
  /// sequence of updates, and reports whatever the request asks to query
  /// afterward. Deliberately generic rather than one C++ method per
  /// scenario -- the scenario itself (repeated hits to the clamp, a miss
  /// sequence, a to-be-pruned cube of siblings, a ray leaving the tree) is
  /// constructed entirely on the Rust test side from these primitives.
  /// Shared by octomapOp and octreeInWorld so the two ops agree on what "the
  /// same actions" means: update_point, update_key, insert_ray, prune,
  /// update_inner_occupancy.
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
          results.push_back(
              json{ { "mapped", true }, { "log_odds", node->getLogOdds() }, { "occupancy", node->getOccupancy() } });
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
          results.push_back(
              json{ { "mapped", true }, { "log_odds", node->getLogOdds() }, { "occupancy", node->getOccupancy() } });
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
  ///    coordinates -- see crates/moveit-geometry/src/shapes.rs's module docs
  ///    for the full FCL/parry3d-f64/distance-field consumer analysis.
  ///
  /// Round 5's gap: everything above stops at pose composition and point
  /// occupancy, never a real collision query -- and `octreeShapeQuery` below
  /// deliberately bypasses `CollisionEnvFCL`/`RobotState`/ACM entirely (see
  /// its own doc). Neither answers "does a real robot collide with a World
  /// whose only object is this octree", which is the question a
  /// `moveit_collision::ParryCollisionEnv` wiring `Shape::OcTree` through
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
  /// `moveit-geometry`'s `Body` enum ported some rounds ago but which, until
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

  moveit::core::RobotModelPtr model_;
  std::unique_ptr<moveit::core::RobotState> state_;

  // For the `ik` op only.
  urdf::ModelInterfaceSharedPtr urdf_model_;
  KDL::Tree kdl_tree_;
  // Reseed draws between `ik` restart attempts (see `ik()`'s own doc
  // comment). Fixed-seeded for a reproducible oracle run; Phase 4's
  // completion condition never compares a solution's exact value, only
  // whether one was found and whether FK(solution) lands on target, so
  // this need not (and structurally cannot, since it is a wholly separate
  // RNG stream) match moveit-kinematics's own reseed draws.
  random_numbers::RandomNumberGenerator ik_rng_{ 42 };
};

}  // namespace

int main(int argc, char** argv)
{
  std::string urdf_path;
  std::string srdf_path;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--urdf" && i + 1 < argc)
      urdf_path = argv[++i];
    else if (arg == "--srdf" && i + 1 < argc)
      srdf_path = argv[++i];
    else
    {
      std::cerr << "oracle: unrecognized argument " << arg << '\n';
      return 2;
    }
  }
  if (urdf_path.empty() || srdf_path.empty())
  {
    std::cerr << "usage: moveit_oracle --urdf <path> --srdf <path>\n";
    return 2;
  }

  std::unique_ptr<Oracle> oracle;
  try
  {
    oracle = std::make_unique<Oracle>(urdf_path, srdf_path);
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
