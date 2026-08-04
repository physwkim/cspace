#!/bin/bash
# Sources the ROS overlay, then execs the given command so it inherits the
# caller's stdio unchanged. Written independently of
# tools/moveit-oracle/entrypoint.sh (not copied -- this image has no
# /ws/install overlay to source, only /opt/ros/${ROS_DISTRO}).
set -eo pipefail

# `set -u` stays off across the sourcing: ROS 2's setup.bash reads optional
# variables (e.g. AMENT_TRACE_SETUP_FILES) without a default, and an
# unbound-variable check would abort here before anything runs.
source "/opt/ros/${ROS_DISTRO}/setup.bash"

set -u
exec "$@"
