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
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shapes.h>
#include <nlohmann/json.hpp>

#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/collision_detection/world.hpp>
#include <moveit/distance_field/find_internal_points.hpp>
#include <moveit/distance_field/propagation_distance_field.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <random_numbers/random_numbers.h>
#include <moveit/robot_state/robot_state.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

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
    if (op == "world")
      return world(request);
    if (op == "distance_field")
      return distanceField(request);
    if (op == "shape_points")
      return shapePoints(request);
    if (op == "common_root")
      return commonRoot(request);
    if (op == "ik")
      return ik(request);
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
  /// deviation on the Rust side, using the same numeric value
  /// (`kMaxRestarts`) so the two sides' success rates are a fair comparison
  /// rather than one side simply being given more attempts.
  json ik(const json& request)
  {
    constexpr unsigned int kMaxSolverIterations = 500;   // SolverParams::max_solver_iterations default
    constexpr double kEpsilon = 0.00001;                 // SolverParams::epsilon default
    constexpr double kSvdThreshold = 0.001;               // SolverParams::svd_threshold default
    constexpr unsigned int kMaxRestarts = 20;             // SolverParams::max_restarts default

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

    KDL::JntArray q_out(mimic_joints.size());
    bool success = cartToJnt(buildQFull(seed_active), q_out);
    for (unsigned int attempt = 0; attempt < kMaxRestarts && !success; ++attempt)
    {
      std::vector<double> reseed_active(active_joint_names.size());
      for (std::size_t k = 0; k < active_joint_names.size(); ++k)
      {
        reseed_active[k] =
            ik_rng_.uniformReal(joint_min[active_full_index[k]], joint_max[active_full_index[k]]);
      }
      success = cartToJnt(buildQFull(reseed_active), q_out);
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
    collision_detection::AllowedCollisionMatrix matrix(*model_->getSRDF());

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

    std::unique_ptr<shapes::Shape> shape;
    if (type == "sphere")
    {
      shape = std::make_unique<shapes::Sphere>(shape_json.at("radius").get<double>());
    }
    else if (type == "box")
    {
      const auto size = shape_json.at("size").get<std::array<double, 3>>();
      shape = std::make_unique<shapes::Box>(size[0], size[1], size[2]);
    }
    else if (type == "cylinder")
    {
      shape = std::make_unique<shapes::Cylinder>(shape_json.at("radius").get<double>(),
                                                  shape_json.at("length").get<double>());
    }
    else if (type == "mesh")
    {
      const auto vertices_json = shape_json.at("vertices");
      const auto triangles_json = shape_json.at("triangles");
      auto* mesh = new shapes::Mesh(vertices_json.size(), triangles_json.size());
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
      shape.reset(mesh);
    }
    else
    {
      throw std::runtime_error("shape_points: unsupported shape type " + type);
    }

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
