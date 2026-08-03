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

ARG MOVEIT2_PACKAGES=moveit_core

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

# Stamp the oracle sources this image was built from. run-oracle.sh compares
# it against the working tree and refuses to answer from a stale image: a new
# op fails loudly ("unsupported op"), but a *changed* op does not -- it would
# quietly answer from old code, and a fixture captured that way looks exactly
# like a good one.
RUN cd /ws/oracle-src \
    && find . -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name 'CMakeLists.txt' \) \
       -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -d' ' -f1 \
       > /usr/local/share/oracle-src.sha256

COPY moveit-oracle/entrypoint.sh /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
