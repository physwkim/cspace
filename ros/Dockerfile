# ROS 2 Rolling + Rust image for building/testing `ros/moveit-ros`.
#
# Deliberately separate from `tools/moveit-oracle`'s image: adding Rust there
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
# No moveit_msgs in this image (yet): building it needs the pinned
# third_party/moveit_msgs checkout, and round 1 of Phase 9 scopes to the
# type-conversion *design* plus proving r2r builds at all -- see
# ros/moveit-ros/doc/message-mapping.md and PORTING-PLAN.md's Phase 9 round-1
# brief. std_msgs/geometry_msgs/sensor_msgs/trajectory_msgs/shape_msgs/
# object_recognition_msgs/octomap_msgs/action_msgs/builtin_interfaces all
# ship in the base image already (verified: `docker run --rm --entrypoint
# bash "$BASE_IMAGE" -lc 'source /opt/ros/rolling/setup.bash; ls
# /opt/ros/rolling/share'`), which is enough surface for this round's
# geometry_msgs conversions.
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

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
# `dtolnay/rust-toolchain@stable`, not a separate pin. `ros/moveit-ros` is its
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
