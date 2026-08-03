# Differential-test oracle image for moveit-rs.
#
# Layer 1 (this file, stage `moveit`): moveit_core built from the pinned
# upstream SHA. Only `--packages-up-to moveit_core` is built — the oracle links
# moveit_robot_model and moveit_robot_state and nothing else, so building the
# planners, moveit_ros and the setup assistant would cost most of the build
# time for code the oracle never calls. Later phases widen this list.
#
# `moveit_resources_fanuc_description` is listed explicitly alongside
# `moveit_core`: colcon's `--packages-up-to` only builds a package's actual
# dependency closure, and `moveit_core` does not depend on any of the
# `moveit_resources` description packages, so none of them would be built (and
# none would be on `AMENT_PREFIX_PATH`, so `package://` resolution would fail
# for all of them) if left to dependency inference alone.
# `moveit_resources_panda_description` and `moveit_resources_pr2_description`
# already build anyway, transitively -- panda's own `moveit_resources`
# metapackage or a colcon workspace scan pulls those two in as siblings under
# `/ws/src`, verified by `ls /ws/install` on a built image -- but the same is
# not true of fanuc's, which is why it needs to be named here for the `mesh`
# op's fanuc parity coverage (see `oracle.cpp`'s `meshOp`) to resolve
# `package://moveit_resources_fanuc_description/...` at all.
#
# Layer 2 (stage `oracle`): the oracle binary itself, rebuilt on every source
# change without re-running the moveit_core build.

# No defaults for BASE_IMAGE / MOVEIT2_PACKAGES / MOVEIT2_SHA: src-digest.sh
# holds the canonical values and build.sh passes all three explicitly. A
# default here would be a second definition of the same fact, and the stamp
# would then be a claim about whichever one happened to win.
#
# The base is pinned by manifest digest rather than the mutable
# `moveit/moveit2:rolling-ci` tag. That also removes the old
# `ARG ROS_DISTRO`, which never did what it looked like it did: it selected
# the base tag, while every `RUN` below read the base image's own
# `ROS_DISTRO` ENV -- an ENV overrides a same-named ARG -- so a
# `--build-arg ROS_DISTRO=humble` produced a humble base running rolling
# paths. With the base pinned, the distro is a property of the image and
# there is no argument left to disagree with it.
ARG BASE_IMAGE
FROM ${BASE_IMAGE} AS moveit

ARG MOVEIT2_PACKAGES

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
# The stamp comes from src-digest.sh rather than a copy of its find
# expression: the stamp written here and the value run-oracle.sh checks it
# against have to be the same function, or a widened file set on one side
# reads as a stale image on the other.
#
# The three ARGs are exported before sourcing so `oracle_stamp` hashes the
# values this build actually resolved, not src-digest.sh's own defaults.
# That is the whole point of stamping them: a hand-passed
# `--build-arg MOVEIT2_PACKAGES=...` or a re-pinned base now changes the
# stamp, and run-oracle.sh rejects the image against an unchanged tree.
# Reading the defaults here instead would stamp the canonical values onto a
# non-canonical image -- worse than not stamping them at all.
ARG BASE_IMAGE
ARG MOVEIT2_PACKAGES
ARG MOVEIT2_SHA
RUN export ORACLE_BASE_IMAGE="$BASE_IMAGE" \
           ORACLE_MOVEIT2_PACKAGES="$MOVEIT2_PACKAGES" \
           ORACLE_MOVEIT2_SHA="$MOVEIT2_SHA" \
    && source /ws/oracle-src/src-digest.sh \
    && oracle_stamp /ws/oracle-src > /usr/local/share/oracle-src.sha256

COPY moveit-oracle/entrypoint.sh /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
