#!/bin/bash
# Sources the ROS and moveit2 overlays, then execs the oracle so it inherits
# the caller's stdin/stdout unchanged — moveit-diff speaks JSON lines over them.
set -eo pipefail

# `set -u` stays off across the sourcing: ROS 2's setup.bash reads
# AMENT_TRACE_SETUP_FILES and other optional variables without a default, so an
# unbound-variable check aborts the entrypoint before the oracle ever starts.
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /ws/install/setup.bash

set -u
exec /usr/local/bin/moveit_oracle "$@"
