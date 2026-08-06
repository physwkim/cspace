// Drives upstream's own `moveit::planning_interface::MoveGroupInterface`
// against whatever `move_group`-shaped node is on the ROS graph, and prints
// what came back in a form `ros/verify-move-action-interop.sh` can assert on.
//
// The class under test is upstream moveit2 at the pinned sha, compiled from
// unmodified source -- no patch, no shim, no subclass, no reimplementation of
// its request-building. Everything in this file is harness around it: read two
// files, put them on the node as the parameters upstream's own
// `RobotModelLoader` reads, construct the interface, call `plan()`, grade what
// came back, print. That is the point: PORTING-PLAN.md Phase 9's completion
// condition is about the *unmodified* client, so anything this file did to help
// the request along would be measuring something else.
//
// The grading is upstream's too, and that is not incidental: Phase 9's
// condition says *valid* trajectory, and the only graders that can settle it
// without assuming the answer are ones the port did not write. See the block
// above `all_in_bounds` for which upstream entry points do it and why the
// node's own `/check_state_validity` is not one of them.
//
// This prints and does not assert. The assertions live in the shell gate, in
// one place, next to the `/plan_kinematic_path` leg's, so that what the gate
// would catch is readable without also reading C++.
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
std::string slurp(const char* path)
{
  std::ifstream file(path);
  if (!file)
  {
    std::cerr << "PROBE cannot read " << path << std::endl;
    std::exit(2);
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cerr << "usage: move_group_interface_probe <urdf> <srdf> <group> [explicit-start]\n";
    return 2;
  }
  rclcpp::init(argc, argv);

  // `robot_description` / `robot_description_semantic` as node parameters is
  // how `MoveGroupInterfaceImpl`'s constructor gets a model: it calls
  // `getSharedRobotModel(node_, opt.robot_description)`, and that reads the
  // parameter off this node. Publishing them on a latched topic instead would
  // exercise a different upstream path than the one under test.
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("robot_description", slurp(argv[1])),
      rclcpp::Parameter("robot_description_semantic", slurp(argv[2])),
  });
  auto node = rclcpp::Node::make_shared("move_group_interface_probe", options);

  // The four-argument constructor, with an explicit wait: the two-argument one
  // spins up a `tf2_ros::Buffer` this probe has no transforms for, and the
  // default wait is long enough that a genuinely absent action server reads as
  // a hung gate rather than a failed one. 20s is well over the ~3s the node
  // needs to come up and well under the shell's own `timeout`.
  std::cout << "PROBE constructing MoveGroupInterface group=" << argv[3] << std::endl;
  moveit::planning_interface::MoveGroupInterface group(node, argv[3], std::shared_ptr<tf2_ros::Buffer>(),
                                                       rclcpp::Duration::from_seconds(20.0));
  std::cout << "PROBE constructed" << std::endl;

  // The second spelling of the start state, and the only other one an
  // unmodified client can produce. `plan()` ships whatever
  // `considered_start_state_` holds, and the constructor leaves that as
  // `setStartStateToCurrentState()`'s empty diff (`is_diff = true`,
  // `move_group_interface.cpp:434-439`). The public
  // `setStartState(const moveit::core::RobotState&)` overload replaces it with
  // a fully-specified state instead -- `is_diff = false`, but
  // `joint_state.name` populated. Both are non-default `RobotState` messages,
  // and they land in *different* variants of `moveit_planning::StartState` --
  // the empty diff in `CurrentState`, the fully-specified one in `Overriding`
  // -- which is the invariant boundary keeping both modes on the gate: one run
  // cannot cover both. This comment used to say the port's
  // `robot_state_msg_is_default` accepted neither; that predicate is what
  // answered -16 to both, and it no longer exists anywhere in the tree.
  const bool explicit_start = argc > 4 && std::string(argv[4]) == "explicit-start";
  if (explicit_start)
  {
    moveit::core::RobotState start(group.getRobotModel());
    start.setToDefaultValues();
    group.setStartState(start);
  }
  std::cout << "PROBE mode=" << (explicit_start ? "explicit-start" : "default-start") << std::endl;

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const moveit::core::MoveItErrorCode code = group.plan(plan);

  // `source` is the discriminator the gate keys on, not `val`. A client that
  // never reached a server returns `MoveItErrorCode::FAILURE` built in-process
  // at `move_group_interface.cpp:659-663`, with `message` and `source` empty;
  // a reply that crossed DDS carries the strings the responding node set. `val`
  // alone cannot tell those apart, because a node is free to answer `FAILURE`.
  std::cout << "PROBE plan val=" << code.val << " source='" << code.source << "'" << std::endl;
  std::cout << "PROBE message='" << code.message << "'" << std::endl;
  std::cout << "PROBE points=" << plan.trajectory.joint_trajectory.points.size() << " multi_dof_points="
            << plan.trajectory.multi_dof_joint_trajectory.points.size() << std::endl;
  // `plan.planning_time` is deliberately not printed: upstream's `Plan` leaves
  // it an uninitialized `double` and `plan()` assigns it only on the success
  // path, so on every failing call it reads whatever was on the stack.

  // Everything below grades the trajectory that arrived, because `SUCCESS` and
  // a non-empty point list are not the completion condition: Phase 9 asks for a
  // *valid* trajectory, and a node that answered SUCCESS with three waypoints
  // outside the joint limits, or three that stop short of the goal, would be
  // indistinguishable from a correct one by the two facts above alone.
  //
  // The grader is upstream's own `moveit_core`, built from the same pinned sha
  // as the client -- `RobotState::satisfiesBounds` and
  // `PlanningScene::isStateConstrained`, over a `PlanningScene` this process
  // builds from the same URDF/SRDF. It is deliberately not the node's own
  // `/check_state_validity`: that would be the port grading its own plan with
  // the code that produced it, and would pass by construction for a whole class
  // of shared errors (a joint-limit reader that is wrong the same way in both
  // places). The goal it is graded against is the client's, not the grader's --
  // `constructMotionPlanRequest` returns the very request `plan()` just sent.
  const auto& traj = plan.trajectory.joint_trajectory;
  const moveit::core::RobotModelConstPtr model = group.getRobotModel();
  const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(argv[3]);
  planning_scene::PlanningScene scene(model);

  moveit_msgs::msg::MotionPlanRequest sent;
  group.constructMotionPlanRequest(sent);

  const auto waypoint_state = [&](const trajectory_msgs::msg::JointTrajectoryPoint& pt) {
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    for (size_t j = 0; j < traj.joint_names.size() && j < pt.positions.size(); ++j)
    {
      state.setVariablePosition(traj.joint_names[j], pt.positions[j]);
    }
    state.update();
    return state;
  };

  // Joint limits, every waypoint. `one_joint.urdf` bounds `j1` to [-1, 1], so
  // this is a real rejection and not a formality: a planner that sampled in an
  // unbounded space, or one that wrote the goal through without clamping, lands
  // outside it. Counted rather than short-circuited so the printed line says
  // how many waypoints were checked -- `0/0 in bounds` is vacuously true and
  // has to be readable as such.
  size_t in_bounds = 0;
  for (const auto& pt : traj.points)
  {
    if (waypoint_state(pt).satisfiesBounds(jmg))
    {
      ++in_bounds;
    }
  }
  const bool all_in_bounds = !traj.points.empty() && in_bounds == traj.points.size();
  std::cout << "PROBE all_in_bounds=" << (all_in_bounds ? "true" : "false") << " (" << in_bounds << '/'
            << traj.points.size() << " waypoints, upstream RobotState::satisfiesBounds)" << std::endl;

  // Does the trajectory end where the client asked it to? `goal_constraints[0]`
  // is what `constructGoal` put on the wire, tolerances included, and
  // `isStateConstrained` is upstream's own evaluator for it. This is the clause
  // that separates "a trajectory came back" from "the query was solved": a node
  // that returns its start state twice satisfies SUCCESS, non-empty and
  // in-bounds, and fails only here.
  bool goal_satisfied = false;
  if (!traj.points.empty() && !sent.goal_constraints.empty())
  {
    goal_satisfied = scene.isStateConstrained(waypoint_state(traj.points.back()), sent.goal_constraints[0]);
  }
  std::cout << "PROBE goal_satisfied=" << (goal_satisfied ? "true" : "false") << " (upstream "
            << "PlanningScene::isStateConstrained on the client's own goal_constraints["
            << (sent.goal_constraints.empty() ? "none" : "0") << "])" << std::endl;

  // Collision is checked and reported, not asserted on by the gate, and the
  // count of collision objects is printed next to it so the reason is visible
  // rather than inferred: `one_joint.urdf` declares no `<collision>` element on
  // either link and the node under test is started with no world, so nothing in
  // this configuration can collide and `0 colliding` is true of every possible
  // trajectory. Naming it a passing clause would be claiming a check that
  // cannot fail. What would make it real is a fixture with collision geometry
  // plus a `/planning_scene` carrying an obstacle, which is the shape the
  // scene-topic leg already runs against `/check_state_validity`.
  size_t colliding = 0;
  for (const auto& pt : traj.points)
  {
    moveit::core::RobotState state = waypoint_state(pt);
    if (scene.isStateColliding(state, argv[3]))
    {
      ++colliding;
    }
  }
  std::cout << "PROBE colliding=" << colliding << '/' << traj.points.size() << " (world objects="
            << scene.getWorld()->size() << ", links with collision geometry="
            << model->getLinkModelsWithCollisionGeometry().size() << " -- reported, not asserted)" << std::endl;

  // The verdict is the conjunction of the clauses above, so that the one string
  // the gate keys on cannot be true of a trajectory that fails any of them.
  // Before this it read `SUCCESS && !points.empty()`, which is the name
  // "received a response" and not the name it carries.
  std::cout << "PROBE verdict="
            << (code == moveit::core::MoveItErrorCode::SUCCESS && !traj.points.empty() && all_in_bounds &&
                        goal_satisfied
                    ? "VALID_TRAJECTORY_RECEIVED"
                    : "NO_VALID_TRAJECTORY")
            << std::endl;

  rclcpp::shutdown();
  return 0;
}
