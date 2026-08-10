// Drives upstream's own `moveit::planning_interface::MoveGroupInterface`
// against whatever `move_group`-shaped node is on the ROS graph, and prints
// what came back in a form `ros/verify-move-action-interop.sh` can assert on.
//
// The class under test is upstream moveit2 at the pinned sha, compiled from
// unmodified source -- no patch, no shim, no subclass, no reimplementation of
// its request-building. Everything in this file is harness around it: give the
// node its model source (parameters, or nothing at all so the client goes to
// the graph for it), construct the interface, call `plan()` or
// `computeCartesianPath()`, grade what came back, print. That is the point:
// PORTING-PLAN.md Phase 9's completion condition is about the *unmodified*
// client, so anything this file did to help the request along would be
// measuring something else.
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
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

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
    std::cerr << "usage: move_group_interface_probe <urdf> <srdf> <group> [mode...]\n"
              << "  start state:  default-start | explicit-start\n"
              << "  model source: description-from-parameters | description-from-topic\n"
              << "  request:      plan | cartesian\n"
              << "  extra:        current-state\n";
    return 2;
  }

  // Every trailing argument is a mode word, and an unrecognised one is a
  // usage error rather than a default. It used to be
  // `argc > 4 && argv[4] == "explicit-start"`, under which a misspelled mode
  // silently ran the other one and the gate asserting `PROBE mode=` would
  // then be asserting against the probe's own echo of the same typo.
  bool explicit_start = false;
  bool description_from_topic = false;
  bool read_current_state = false;
  bool run_cartesian = false;
  for (int i = 4; i < argc; ++i)
  {
    const std::string mode(argv[i]);
    if (mode == "default-start")
      explicit_start = false;
    else if (mode == "explicit-start")
      explicit_start = true;
    else if (mode == "description-from-parameters")
      description_from_topic = false;
    else if (mode == "description-from-topic")
      description_from_topic = true;
    else if (mode == "current-state")
      read_current_state = true;
    else if (mode == "plan")
      run_cartesian = false;
    else if (mode == "cartesian")
      run_cartesian = true;
    else
    {
      std::cerr << "PROBE unknown mode '" << mode << "'" << std::endl;
      return 2;
    }
  }

  rclcpp::init(argc, argv);

  // The two ways upstream's own `SynchronizedStringParameter::loadInitialValue`
  // can obtain a description, made selectable because they are different code
  // paths with different failure modes and only one of them can be exercised
  // per run.
  //
  //   `description-from-parameters` sets them as node parameters, which
  //   `getMainParameter` reads and returns early on -- the topic is never
  //   touched.
  //
  //   `description-from-topic` sets neither, which is what a bare client looks
  //   like: `getMainParameter` returns false, `loadInitialValue` falls through
  //   to a transient-local `std_msgs/String` subscription on the same name and
  //   blocks up to `robot_description_timeout` (10 s by default), twice. This
  //   is the mode that measures whether something on the graph is publishing
  //   what the client needs, and the only one whose success says anything
  //   about a publisher.
  rclcpp::NodeOptions options;
  if (!description_from_topic)
  {
    options.parameter_overrides({
        rclcpp::Parameter("robot_description", slurp(argv[1])),
        rclcpp::Parameter("robot_description_semantic", slurp(argv[2])),
    });
  }
  auto node = rclcpp::Node::make_shared("move_group_interface_probe", options);
  std::cout << "PROBE description=" << (description_from_topic ? "topic" : "parameters") << std::endl;

  // Spin the probe's own node, because `MoveGroupInterface` deliberately does
  // not. Its constructor creates its callback group with
  // `false /* don't spin with node executor */` and adds *only that group* to
  // its private executor (`move_group_interface.cpp:129-133`), so everything it
  // puts on the node's default group is the application's to service. The
  // `CurrentStateMonitor`'s `joint_states` subscription is exactly that --
  // `createJointStateSubscription` calls `node_->create_subscription` with no
  // callback group (`current_state_monitor_middleware_handle.cpp:69-74`).
  // Without this thread the subscription exists and never fires, and
  // `getCurrentState` reports `latest received state has time 0.000000`: not a
  // stale stamp but no message at all, because
  // `CurrentStateMonitor::jointStateCallback` assigns `current_state_time_`
  // unconditionally (`current_state_monitor.cpp:341`) and had never run.
  //
  // Upstream's own ROS 2 test does the same thing before the same call --
  // `executor.add_node(move_group_node)` on a spin thread, then
  // `getCurrentState(60)` (`test_trajectory_cache.cpp:1047-1064`) -- so this is
  // the documented way to drive the unmodified class, not a modification of it.
  //
  // It does not disturb the description path: `SynchronizedStringParameter::`
  // `waitForMessage` subscribes on a *temporary node* of its own and drains it
  // with its own `rclcpp::WaitSet` (`synchronized_string_parameter.cpp:120-133`),
  // never touching this node's executor.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // The four-argument constructor, with an explicit wait: the two-argument one
  // spins up a `tf2_ros::Buffer` this probe has no transforms for, and the
  // default wait is long enough that a genuinely absent action server reads as
  // a hung gate rather than a failed one. 20s is well over the ~3s the node
  // needs to come up and well under the shell's own `timeout`.
  std::cout << "PROBE constructing MoveGroupInterface group=" << argv[3] << std::endl;
  moveit::planning_interface::MoveGroupInterface group(node, argv[3], std::shared_ptr<tf2_ros::Buffer>(),
                                                       rclcpp::Duration::from_seconds(20.0));
  std::cout << "PROBE constructed" << std::endl;

  // `getCurrentState`, opt-in so the two runs that do not ask for it behave
  // exactly as before. This is the only client call that needs `joint_states`:
  // it starts the `CurrentStateMonitor` and then waits for a state stamped no
  // earlier than the call itself, so a null return here is a topic that is
  // absent, stale-stamped, or carrying joints this model does not have.
  // `plan()` needs none of that -- it ships the constructor's empty diff.
  //
  // The variables are printed, not summarised: the gate sets a distinctive
  // joint value on the node through `/planning_scene` before this runs, so a
  // publisher sending zeros, or the model's own defaults, reads differently
  // here from one relaying the node's monitored state.
  if (read_current_state)
  {
    const moveit::core::RobotStatePtr current = group.getCurrentState(5.0);
    std::cout << "PROBE current_state=" << (current ? "received" : "timeout") << std::endl;
    if (current)
    {
      for (const std::string& variable : current->getRobotModel()->getVariableNames())
      {
        std::cout << "PROBE current_state_variable name=" << variable
                  << " position=" << current->getVariablePosition(variable) << std::endl;
      }
    }
  }

  // The second spelling of the start state, and the only other one an
  // unmodified client can produce. `plan()` ships whatever
  // `considered_start_state_` holds, and the constructor leaves that as
  // `setStartStateToCurrentState()`'s empty diff (`is_diff = true`,
  // `move_group_interface.cpp:434-439`). The public
  // `setStartState(const moveit::core::RobotState&)` overload replaces it with
  // a fully-specified state instead -- `is_diff = false`, but
  // `joint_state.name` populated. Both are non-default `RobotState` messages,
  // and they land in *different* variants of `cspace_planning::StartState` --
  // the empty diff in `CurrentState`, the fully-specified one in `Overriding`
  // -- which is the invariant boundary keeping both modes on the gate: one run
  // cannot cover both. This comment used to say the port's
  // `robot_state_msg_is_default` accepted neither; that predicate is what
  // answered -16 to both, and it no longer exists anywhere in the tree.
  if (explicit_start)
  {
    moveit::core::RobotState start(group.getRobotModel());
    start.setToDefaultValues();
    group.setStartState(start);
  }
  // Two lines because they are two axes: which start state the client ships,
  // and which endpoint it calls. A single `mode=cartesian` would have to stand
  // for both, and a gate asserting it would be asserting the start-state
  // spelling without saying so.
  std::cout << "PROBE mode=" << (explicit_start ? "explicit-start" : "default-start") << std::endl;
  std::cout << "PROBE request=" << (run_cartesian ? "cartesian" : "plan") << std::endl;

  // `/compute_cartesian_path`, through the same unmodified client. It is a
  // service and not the action, so it shares nothing with `plan()` below but
  // the interface object: a node that serves `/move_action` correctly can
  // still be missing this endpoint entirely, and until this branch existed
  // that is what the gate could not see.
  //
  // The call is the four-argument, non-deprecated overload
  // (`move_group_interface.hpp:778-780`). The three jump thresholds are not
  // arguments of it -- `dae612696` removed the parameter -- so this probe
  // cannot set them and neither can any other unmodified client, which is
  // what makes the node's refusal of them unreachable from here and checked
  // in-process instead.
  if (run_cartesian)
  {
    // `tip`'s pose at `j1 = 0.5`, in `getPoseReferenceFrame()` (the model
    // frame, `move_group_interface.cpp:175`). `one_joint.urdf`'s `j1` has no
    // `<origin>`, so `tip`'s translation is identically zero and its
    // orientation is a rotation of `j1` about z: the waypoint is exactly
    // reachable, and every pose on the straight line to it is too, so a
    // correct answer is `fraction = 1` rather than some fixture-dependent
    // fragment.
    const double angle = 0.5;
    geometry_msgs::msg::Pose target;
    target.orientation.z = std::sin(angle / 2.0);
    target.orientation.w = std::cos(angle / 2.0);
    const std::vector<geometry_msgs::msg::Pose> waypoints{ target };

    moveit_msgs::msg::RobotTrajectory result;
    moveit_msgs::msg::MoveItErrorCodes cartesian_code;
    const double fraction = group.computeCartesianPath(waypoints, 0.1, result, true, &cartesian_code);

    // `-1.0` is what the client returns for *any* non-SUCCESS reply and also
    // for a call that never reached a server
    // (`move_group_interface.cpp:899-911`), so it is printed beside `source`
    // rather than on its own, for the reason the `plan()` leg gives below.
    std::cout << "PROBE cartesian val=" << cartesian_code.val << " source='" << cartesian_code.source << "'"
              << std::endl;
    std::cout << "PROBE cartesian message='" << cartesian_code.message << "'" << std::endl;
    std::cout << "PROBE cartesian fraction=" << fraction << std::endl;
    std::cout << "PROBE cartesian points=" << result.joint_trajectory.points.size() << std::endl;

    const moveit::core::RobotModelConstPtr cartesian_model = group.getRobotModel();
    const moveit::core::JointModelGroup* cartesian_jmg = cartesian_model->getJointModelGroup(argv[3]);
    const auto& cartesian_traj = result.joint_trajectory;

    const auto cartesian_state = [&](const trajectory_msgs::msg::JointTrajectoryPoint& pt) {
      moveit::core::RobotState state(cartesian_model);
      state.setToDefaultValues();
      for (size_t j = 0; j < cartesian_traj.joint_names.size() && j < pt.positions.size(); ++j)
      {
        state.setVariablePosition(cartesian_traj.joint_names[j], pt.positions[j]);
      }
      state.update();
      return state;
    };

    size_t cartesian_in_bounds = 0;
    for (const auto& pt : cartesian_traj.points)
    {
      if (cartesian_state(pt).satisfiesBounds(cartesian_jmg))
      {
        ++cartesian_in_bounds;
      }
    }
    const bool cartesian_all_in_bounds = !cartesian_traj.points.empty() &&
                                         cartesian_in_bounds == cartesian_traj.points.size();
    std::cout << "PROBE cartesian all_in_bounds=" << (cartesian_all_in_bounds ? "true" : "false") << " ("
              << cartesian_in_bounds << '/' << cartesian_traj.points.size()
              << " waypoints, upstream RobotState::satisfiesBounds)" << std::endl;

    // The clause that separates "a path came back" from "the path went where
    // it was asked to". Graded through upstream's own forward kinematics on
    // the link the client itself named, against the pose the client itself
    // sent -- the node's `fraction` is not consulted, so a node that reported
    // `1.0` for a path ending anywhere else fails here.
    const std::string eef = group.getEndEffectorLink().empty() ? cartesian_jmg->getLinkModelNames().back() :
                                                                 group.getEndEffectorLink();
    double reached_error = std::numeric_limits<double>::infinity();
    if (!cartesian_traj.points.empty())
    {
      const moveit::core::RobotState end = cartesian_state(cartesian_traj.points.back());
      const Eigen::Isometry3d& achieved = end.getGlobalLinkTransform(eef);
      const Eigen::Quaterniond target_q(target.orientation.w, target.orientation.x, target.orientation.y,
                                        target.orientation.z);
      const Eigen::Vector3d target_p(target.position.x, target.position.y, target.position.z);
      reached_error = Eigen::Quaterniond(achieved.linear()).angularDistance(target_q) +
                      (achieved.translation() - target_p).norm();
    }
    const bool cartesian_reached = reached_error < 1e-6;
    std::cout << "PROBE cartesian reached=" << (cartesian_reached ? "true" : "false") << " (link=" << eef
              << ", pose error=" << reached_error << " rad+m, upstream RobotState::getGlobalLinkTransform)"
              << std::endl;

    std::cout << "PROBE cartesian verdict="
              << (cartesian_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS && fraction == 1.0 &&
                          cartesian_all_in_bounds && cartesian_reached ?
                      "FULL_CARTESIAN_PATH_RECEIVED" :
                      "NO_FULL_CARTESIAN_PATH")
              << std::endl;

    rclcpp::shutdown();
    return 0;
  }

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

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
