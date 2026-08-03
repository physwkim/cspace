#!/bin/bash
# Sources the ROS and moveit2 overlays, then execs the oracle so it inherits
# the caller's stdin/stdout unchanged — moveit-diff speaks JSON lines over them.
set -euo pipefail
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /ws/install/setup.bash
exec /usr/local/bin/moveit_oracle "$@"
