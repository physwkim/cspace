#!/bin/bash
# Sources the ROS underlay, then the /ws/install overlay (moveit_msgs, built
# at image-build time -- see ../Dockerfile), then execs the given command so
# it inherits the caller's stdio unchanged. Written independently of
# tools/moveit-oracle/entrypoint.sh (not copied), but the same two-layer
# source order: the overlay must come after the underlay so its
# AMENT_PREFIX_PATH entries are prepended, not the other way round.
set -eo pipefail

# `set -u` stays off across the sourcing: ROS 2's setup.bash reads optional
# variables (e.g. AMENT_TRACE_SETUP_FILES) without a default, and an
# unbound-variable check would abort here before anything runs.
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/ws/install/setup.bash"

set -u
exec "$@"
