# ROS 2 Rolling + moveit_msgs + Rust image for building/testing
# `ros/cspace-ros`.
#
# Deliberately separate from `tools/moveit-oracle`'s image: adding Rust (or
# moveit_msgs, which the oracle's own moveit_core build never needed) there
# would change that image's stamp, and every one of the seven other panels'
# fixture verification keys off that stamp for reasons that have nothing to
# do with ROS interop. See PORTING-PLAN.md §129.4.
#
# `BASE_IMAGE` is the literal base moveit2's own oracle build starts FROM
# (`ORACLE_BASE_IMAGE` in tools/ci/../moveit-oracle/src-digest.sh), pinned by
# manifest digest -- not the oracle's own derived, per-round-stamped tag
# (`moveit-rs/oracle:<stamp>`). The derived tag changes every time any of the
# seven other panels touches tools/moveit-oracle/src/**; pinning to it would
# make this image go stale for reasons that have nothing to do with ROS
# interop. The literal base only moves when someone deliberately repins
# ORACLE_BASE_IMAGE, which is the same event that invalidates the oracle
# image too -- so this and the oracle rebuild together, not on unrelated
# C++ changes.
#
# moveit_msgs, built from the third_party/moveit_msgs reference checkout
# (round 2, PORTING-PLAN.md Phase 9). Its own dependencies
# (octomap_msgs/action_msgs/sensor_msgs/geometry_msgs/trajectory_msgs/
# shape_msgs/object_recognition_msgs/std_msgs) all ship in the base image
# already (verified round 1: `docker run --rm --entrypoint bash "$BASE_IMAGE"
# -lc 'source /opt/ros/rolling/setup.bash; ls /opt/ros/rolling/share'`), so
# this is a standalone `colcon build --packages-select moveit_msgs` -- no
# moveit_core, no C++ planning stack, none of the oracle's ~5GB build. build.sh
# exports third_party/moveit_msgs at its committed HEAD into this image's
# build context (git-archive, same pattern as tools/moveit-oracle/build.sh,
# same reason: no dirty tree, no .git) rather than using the oracle's own
# already-built /ws/install -- FROM-ing the oracle's per-round-stamped image
# would tie this image's build to whichever worktree happened to build the
# oracle last, exactly the staleness problem BASE_IMAGE's manifest-digest pin
# (below) already avoids for the base layer.
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /ws/src
COPY moveit_msgs /ws/src/moveit_msgs
WORKDIR /ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build \
       --packages-select moveit_msgs \
       --cmake-args -DCMAKE_BUILD_TYPE=Release \
       --event-handlers console_direct+

# libclang-dev: r2r's build.rs (via bindgen) needs libclang.so at a
# predictable path. The base image already carries libclang-common-18-dev/
# libclang1-18 (clang tooling used elsewhere in the image), but not the
# `libclang-dev` metapackage that provides the unversioned `libclang.so`
# symlink bindgen's `clang-sys` probes for by default.
RUN apt-get -q update \
    && apt-get -q install --no-install-recommends -y \
       libclang-dev \
       ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# rustup, stable channel -- matches .github/workflows/ci.yml's
# `dtolnay/rust-toolchain@stable`, not a separate pin. `ros/cspace-ros` is its
# own workspace (D5) with its own toolchain resolution, but there is no
# reason for it to disagree with the root workspace's channel choice.
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
    bash -s -- -y --profile minimal --default-toolchain stable \
      --component rustfmt --component clippy

WORKDIR /ros

COPY entrypoint.sh /usr/local/bin/ros-entrypoint.sh
RUN chmod +x /usr/local/bin/ros-entrypoint.sh
ENTRYPOINT ["/usr/local/bin/ros-entrypoint.sh"]
