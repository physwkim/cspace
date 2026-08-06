# `MoveGroupInterface`'s public surface and the endpoint each declaration reaches

Generated. Regenerate with

    tools/ci/measure-client-endpoint-surface.py \
        --upstream ~/work/moveit2 --emit-doc > doc/client-endpoint-surface.md

and check it with `tools/ci/verify-client-endpoint-surface.sh`, which owns
the pinned-revision precondition. Every `hpp:` line number is relative to
that revision; every port path is relative to this repository and is read
from the working tree, so a rename moves it. `-- ` in the last column of
the declaration table means the declaration puts nothing on the wire.

Endpoint names are **relative** -- the client resolves each one through
`rclcpp::names::append(opt_.move_group_namespace, ...)`, so a leading slash
would name a different endpoint. `robot_description` is the model load: a
parameter read that falls back to a latched topic, not a `move_group`
endpoint, but the client's constructor cannot complete without it.

    public function declarations   126
      special members              7
      named operations             119
    non-function declarations      4
    count-public-declarations.sh   130
    reach the wire                 38
    client-local                   88

    port side, absent             8
    port side, bound              2
    port side, surplus            3

## What the port binds

`bound` means this workspace opens that endpoint in the direction the
client needs. `absent` means nothing here opens it. `role-mismatch` means
something here opens the name in the wrong direction. `surplus` means the
port opens an endpoint no `MoveGroupInterface` declaration asks for.

`bound` is a static fact about the socket and nothing more. Whether the
handler behind it then answers or rejects is a separate question this
table cannot reach -- it is Phase 9's (a)-versus-(b) split, and it needs
a read of the handler or a run of the node. `absent` is the whole of (c).

| endpoint | the client | the port must provide | opened at | verdict |
|---|---|---|---|---|
| `attached_collision_object` | publishes | subscriber | -- | absent |
| `check_state_validity` | -- | -- | `ros/moveit-ros/src/bin/move_group.rs:666` | surplus |
| `compute_cartesian_path` | calls | service server | `ros/moveit-ros/src/bin/move_group.rs:683` | bound |
| `execute_trajectory` | calls | action server | -- | absent |
| `get_planner_params` | calls | service server | -- | absent |
| `joint_states` | subscribes | publisher | -- | absent |
| `move_action` | calls | action server | `ros/moveit-ros/src/bin/move_group.rs:631` | bound |
| `plan_kinematic_path` | -- | -- | `ros/moveit-ros/src/bin/move_group.rs:615` | surplus |
| `planning_scene` | -- | -- | `ros/moveit-ros/src/bin/move_group.rs:654` | surplus |
| `query_planner_interface` | calls | service server | -- | absent |
| `robot_description` | reads | parameter or latched publisher | -- | absent |
| `set_planner_params` | calls | service server | -- | absent |
| `trajectory_execution_event` | publishes | subscriber | -- | absent |

## Every public declaration

| declaration | name | endpoints |
|---|---|---|
| `hpp:136` | `MoveGroupInterface` | `move_action` `execute_trajectory` `robot_description` |
| `hpp:147` | `MoveGroupInterface` | `move_action` `execute_trajectory` `robot_description` |
| `hpp:151` | `~MoveGroupInterface` | -- |
| `hpp:158` | `MoveGroupInterface` | -- |
| `hpp:159` | `operator=` | -- |
| `hpp:161` | `MoveGroupInterface` | -- |
| `hpp:162` | `operator=` | -- |
| `hpp:164` | `getName` | -- |
| `hpp:168` | `getNamedTargets` | -- |
| `hpp:171` | `getTF` | -- |
| `hpp:174` | `getRobotModel` | -- |
| `hpp:177` | `getNode` | -- |
| `hpp:180` | `getPlanningFrame` | -- |
| `hpp:183` | `getJointModelGroupNames` | -- |
| `hpp:186` | `getJointNames` | -- |
| `hpp:189` | `getLinkNames` | -- |
| `hpp:192` | `getNamedTargetValues` | -- |
| `hpp:195` | `getActiveJoints` | -- |
| `hpp:198` | `getJoints` | -- |
| `hpp:202` | `getVariableCount` | -- |
| `hpp:205` | `getInterfaceDescriptions` | `query_planner_interface` |
| `hpp:208` | `getInterfaceDescription` | `query_planner_interface` |
| `hpp:211` | `getPlannerParams` | `get_planner_params` |
| `hpp:215` | `setPlannerParams` | `set_planner_params` |
| `hpp:218` | `getDefaultPlanningPipelineId` | -- |
| `hpp:221` | `setPlanningPipelineId` | -- |
| `hpp:224` | `getPlanningPipelineId` | -- |
| `hpp:227` | `getDefaultPlannerId` | -- |
| `hpp:230` | `setPlannerId` | -- |
| `hpp:233` | `getPlannerId` | -- |
| `hpp:236` | `setPlanningTime` | -- |
| `hpp:240` | `setNumPlanningAttempts` | -- |
| `hpp:247` | `setMaxVelocityScalingFactor` | -- |
| `hpp:250` | `getMaxVelocityScalingFactor` | -- |
| `hpp:257` | `setMaxAccelerationScalingFactor` | -- |
| `hpp:260` | `getMaxAccelerationScalingFactor` | -- |
| `hpp:263` | `getPlanningTime` | -- |
| `hpp:267` | `getGoalJointTolerance` | -- |
| `hpp:271` | `getGoalPositionTolerance` | -- |
| `hpp:275` | `getGoalOrientationTolerance` | -- |
| `hpp:283` | `setGoalTolerance` | -- |
| `hpp:287` | `setGoalJointTolerance` | -- |
| `hpp:290` | `setGoalPositionTolerance` | -- |
| `hpp:293` | `setGoalOrientationTolerance` | -- |
| `hpp:299` | `setWorkspace` | -- |
| `hpp:303` | `setStartState` | -- |
| `hpp:307` | `setStartState` | -- |
| `hpp:310` | `setStartStateToCurrentState` | -- |
| `hpp:341` | `setJointValueTarget` | -- |
| `hpp:358` | `setJointValueTarget` | -- |
| `hpp:375` | `setJointValueTarget` | -- |
| `hpp:386` | `setJointValueTarget` | -- |
| `hpp:399` | `setJointValueTarget` | -- |
| `hpp:412` | `setJointValueTarget` | -- |
| `hpp:424` | `setJointValueTarget` | -- |
| `hpp:437` | `setJointValueTarget` | `joint_states` |
| `hpp:450` | `setJointValueTarget` | `joint_states` |
| `hpp:463` | `setJointValueTarget` | `joint_states` |
| `hpp:475` | `setApproximateJointValueTarget` | `joint_states` |
| `hpp:488` | `setApproximateJointValueTarget` | `joint_states` |
| `hpp:501` | `setApproximateJointValueTarget` | `joint_states` |
| `hpp:507` | `setRandomTarget` | -- |
| `hpp:511` | `setNamedTarget` | -- |
| `hpp:514` | `getJointValueTarget` | -- |
| `hpp:537` | `setPositionTarget` | -- |
| `hpp:546` | `setRPYTarget` | -- |
| `hpp:556` | `setOrientationTarget` | -- |
| `hpp:565` | `setPoseTarget` | -- |
| `hpp:574` | `setPoseTarget` | -- |
| `hpp:583` | `setPoseTarget` | -- |
| `hpp:603` | `setPoseTargets` | -- |
| `hpp:623` | `setPoseTargets` | -- |
| `hpp:643` | `setPoseTargets` | -- |
| `hpp:647` | `setPoseReferenceFrame` | -- |
| `hpp:652` | `setEndEffectorLink` | -- |
| `hpp:656` | `setEndEffector` | -- |
| `hpp:659` | `clearPoseTarget` | -- |
| `hpp:662` | `clearPoseTargets` | -- |
| `hpp:670` | `getPoseTarget` | -- |
| `hpp:677` | `getPoseTargets` | -- |
| `hpp:684` | `getEndEffectorLink` | -- |
| `hpp:691` | `getEndEffector` | -- |
| `hpp:695` | `getPoseReferenceFrame` | -- |
| `hpp:707` | `asyncMove` | `move_action` |
| `hpp:713` | `getMoveGroupClient` | `move_action` |
| `hpp:719` | `move` | `move_action` |
| `hpp:724` | `plan` | `move_action` |
| `hpp:732` | `asyncExecute` | `execute_trajectory` |
| `hpp:741` | `asyncExecute` | `execute_trajectory` |
| `hpp:750` | `execute` | `execute_trajectory` |
| `hpp:759` | `execute` | `execute_trajectory` |
| `hpp:771` | `computeCartesianPath` | `compute_cartesian_path` |
| `hpp:778` | `computeCartesianPath` | `compute_cartesian_path` |
| `hpp:794` | `computeCartesianPath` | `compute_cartesian_path` |
| `hpp:802` | `computeCartesianPath` | `compute_cartesian_path` |
| `hpp:808` | `stop` | `trajectory_execution_event` |
| `hpp:811` | `allowReplanning` | -- |
| `hpp:814` | `setReplanAttempts` | -- |
| `hpp:817` | `setReplanDelay` | -- |
| `hpp:821` | `allowLooking` | -- |
| `hpp:824` | `setLookAroundAttempts` | -- |
| `hpp:832` | `constructRobotState` | -- |
| `hpp:836` | `constructMotionPlanRequest` | -- |
| `hpp:851` | `attachObject` | `attached_collision_object` |
| `hpp:861` | `attachObject` | `attached_collision_object` |
| `hpp:868` | `detachObject` | `attached_collision_object` |
| `hpp:882` | `startStateMonitor` | `joint_states` |
| `hpp:885` | `getCurrentJointValues` | `joint_states` |
| `hpp:888` | `getCurrentState` | `joint_states` |
| `hpp:893` | `getCurrentPose` | `joint_states` |
| `hpp:898` | `getCurrentRPY` | `joint_states` |
| `hpp:901` | `getRandomJointValues` | -- |
| `hpp:906` | `getRandomPose` | `joint_states` |
| `hpp:919` | `rememberJointValues` | `joint_states` |
| `hpp:925` | `rememberJointValues` | -- |
| `hpp:928` | `getRememberedJointValues` | -- |
| `hpp:934` | `forgetJointValues` | -- |
| `hpp:944` | `setConstraintsDatabase` | `warehouse` |
| `hpp:947` | `getKnownConstraints` | `warehouse` |
| `hpp:952` | `getPathConstraints` | -- |
| `hpp:957` | `setPathConstraints` | `warehouse` |
| `hpp:962` | `setPathConstraints` | -- |
| `hpp:966` | `clearPathConstraints` | -- |
| `hpp:968` | `getTrajectoryConstraints` | -- |
| `hpp:969` | `setTrajectoryConstraints` | -- |
| `hpp:970` | `clearTrajectoryConstraints` | -- |
