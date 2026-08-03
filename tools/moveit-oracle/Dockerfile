# Differential-test oracle image for moveit-rs.
#
# Layer 1 (this file, stage `moveit`): moveit_core built from the pinned
# upstream SHA. Only `--packages-up-to moveit_core` is built — the oracle links
# moveit_robot_model and moveit_robot_state and nothing else, so building the
# planners, moveit_ros and the setup assistant would cost most of the build
# time for code the oracle never calls. Later phases widen this list.
#
# Layer 2 (stage `oracle`): the oracle binary itself, rebuilt on every source
# change without re-running the moveit_core build.

ARG ROS_DISTRO=rolling
FROM moveit/moveit2:${ROS_DISTRO}-ci AS moveit

# `moveit_resources_fanuc_description` is named explicitly because nothing
# pulls it in: `--packages-up-to moveit_core` builds the panda and pr2
# description packages only because moveit_core test-depends on them, and
# fanuc is not among those. Without it, every `package://
# moveit_resources_fanuc_description/...` mesh in fixtures/fanuc.urdf fails
# to resolve inside the container, RobotModel logs "No geometry is
# associated to any robot links", and the oracle answers every fanuc
# collision query against a robot with no collision geometry at all -- a
# fixture captured that way agrees with anything.
ARG MOVEIT2_PACKAGES="moveit_core moveit_resources_fanuc_description"

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# nlohmann/json, plus orocos_kdl and kdl_parser for the ik op's vendored
# KDL::ChainIkSolverVelMimicSVD (see src/third_party/kdl_kinematics_plugin/)
# -- installed as prebuilt packages rather than added to
# --packages-up-to, since neither is part of the moveit2 source tree this
# image builds from.
RUN apt-get -q update \
    && apt-get -q install --no-install-recommends -y \
       nlohmann-json3-dev \
       ros-${ROS_DISTRO}-orocos-kdl-vendor \
       ros-${ROS_DISTRO}-kdl-parser \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src

# moveit2 at the pinned SHA, plus the two repos moveit2.repos pulls from source.
COPY moveit2 /ws/src/moveit2
COPY moveit_msgs /ws/src/moveit_msgs
COPY moveit_resources /ws/src/moveit_resources

# The SHA is verified by build.sh before the context is exported; .git is
# deliberately absent here so it cannot be re-checked inside the build.
WORKDIR /ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build \
       --packages-up-to ${MOVEIT2_PACKAGES} \
       --cmake-args -DCMAKE_BUILD_TYPE=Release \
       --event-handlers console_direct+

FROM moveit AS oracle

COPY moveit-oracle /ws/oracle-src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /ws/install/setup.bash \
    && cmake -S /ws/oracle-src -B /ws/oracle-build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /ws/oracle-build --parallel \
    && cmake --install /ws/oracle-build --prefix /usr/local

# Stamp the sources this image was built from. run-oracle.sh compares it
# against the working tree and refuses to answer from a stale image: a new
# op fails loudly ("unsupported op"), but a *changed* op does not -- it would
# quietly answer from old code, and a fixture captured that way looks exactly
# like a good one.
#
# The digest comes from src-digest.sh rather than a copy of its find
# expression: the stamp written here and the value run-oracle.sh checks it
# against have to be the same function, or a widened file set on one side
# reads as a stale image on the other.
RUN source /ws/oracle-src/src-digest.sh \
    && oracle_src_digest /ws/oracle-src > /usr/local/share/oracle-src.sha256

COPY moveit-oracle/entrypoint.sh /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
