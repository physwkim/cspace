# The container that holds upstream's C++ `MoveGroupInterface`, for the
# `/move_action` leg of ros/verify-ros-interop.sh.
#
# Why a third image rather than adding `moveit_ros_planning_interface` to
# ros/Dockerfile: that image is deliberately the base image plus moveit_msgs
# and Rust, nothing else (see its own header). `moveit_ros_planning_interface`
# pulls in moveit_core, moveit_ros_planning, moveit_ros_occupancy_map_monitor
# and their transitive C++ dependencies -- an order of magnitude more build
# than everything ros/Dockerfile does today, imposed on every `cargo fmt`
# invocation of the gate. Keeping it separate means the four cheap legs (fmt,
# clippy, test, doc) and the `/plan_kinematic_path` leg do not pay for it.
#
# Why `FROM` the oracle image rather than `ORACLE_BASE_IMAGE`: the base image
# ships zero moveit packages (`ls /opt/ros/rolling/share | grep -c moveit` is
# 0 on it), so building `moveit_ros_planning_interface` from the base means
# fetching and building moveit2 from scratch. The oracle image already carries
# the pinned moveit2 checkout at /ws/src/moveit2 and a built /ws/install, so
# `--packages-up-to` here is an incremental build on top of it. That does tie
# this image to the oracle's per-round stamp, which ros/Dockerfile explicitly
# avoids for itself -- the difference is that ros/Dockerfile's content has
# nothing to do with the oracle, whereas this image's whole content is moveit2
# C++ at the same pinned sha the oracle builds. Tying them is the accurate
# relationship, and ros/verify-move-action-interop.sh resolves ORACLE_IMAGE
# through `oracle_image_tag` so a restamped oracle rebuilds this too.
#
# There is no stamp of this image's own sources, unlike the oracle's: the
# `colcon` layer keys on nothing but `ORACLE_IMAGE`, and the probe layer keys
# on the `COPY`ed files, so docker's own layer cache already rebuilds exactly
# what changed. The gate runs `docker build` unconditionally for that reason
# -- a hand-rolled staleness check here would be a second, weaker copy of it.
ARG ORACLE_IMAGE
FROM ${ORACLE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# `moveit_ros_planning_interface` on top of what the oracle image already
# built. `--packages-up-to` and not `--packages-select`: the oracle builds
# `moveit_core` and four planner packages, not the moveit_ros stack this
# depends on.
WORKDIR /ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /ws/install/setup.bash \
    && colcon build \
       --packages-up-to moveit_ros_planning_interface \
       --cmake-args -DCMAKE_BUILD_TYPE=Release \
       --event-handlers console_direct+

COPY move_group_interface_probe /ws/src/move_group_interface_probe
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /ws/install/setup.bash \
    && colcon build \
       --packages-select move_group_interface_probe \
       --cmake-args -DCMAKE_BUILD_TYPE=Release \
       --event-handlers console_direct+

# No ENTRYPOINT: the oracle image's own entrypoint runs the oracle binary, and
# this image is not an oracle. ros/verify-move-action-interop.sh passes
# `--entrypoint bash` and sources the two setup files itself.
