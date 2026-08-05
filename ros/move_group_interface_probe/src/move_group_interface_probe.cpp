// Drives upstream's own `moveit::planning_interface::MoveGroupInterface`
// against whatever `move_group`-shaped node is on the ROS graph, and prints
// what came back in a form `ros/verify-move-action-interop.sh` can assert on.
//
// The class under test is upstream moveit2 at the pinned sha, compiled from
// unmodified source -- no patch, no shim, no subclass, no reimplementation of
// its request-building. Everything in this file is harness around it: read two
// files, put them on the node as the parameters upstream's own
// `RobotModelLoader` reads, construct the interface, call `plan()`, print. That
// is the point: PORTING-PLAN.md Phase 9's completion condition is about the
// *unmodified* client, so anything this file did to help the request along
// would be measuring something else.
//
// This prints and does not assert. The assertions live in the shell gate, in
// one place, next to the `/plan_kinematic_path` leg's, so that what the gate
// would catch is readable without also reading C++.
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <moveit/move_group_interface/move_group_interface.hpp>
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
  // which is the invariant boundary the gate asserts on: the port's
  // `robot_state_msg_is_default` accepts neither.
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
  std::cout << "PROBE verdict="
            << (code == moveit::core::MoveItErrorCode::SUCCESS && !plan.trajectory.joint_trajectory.points.empty()
                    ? "VALID_TRAJECTORY_RECEIVED"
                    : "NO_VALID_TRAJECTORY")
            << std::endl;

  rclcpp::shutdown();
  return 0;
}
