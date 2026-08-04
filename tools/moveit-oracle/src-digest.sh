#!/bin/bash
# Prints the digest of everything the oracle image is built from, and the
# image tag derived from it.
#
# One definition, used by build.sh (which tags with it), the Dockerfile
# (which stamps the image with it) and run-oracle.sh (which resolves the tag
# and verifies the stamp). Computing it separately in any of them would let
# them drift into disagreeing about which image is current -- and the
# disagreement would look exactly like a stale image.
#
# Every regular file under the directory is hashed, with no extension
# allowlist. build.sh `cp -a`s this whole directory into the build context
# and the Dockerfile `COPY`s all of it to /ws/oracle-src, so "hashed" and
# "went into the image" are the same set by construction; an allowlist makes
# them different sets, and every file kind outside it (Dockerfile, build.sh,
# the pinned MOVEIT2_SHA it carries, entrypoint.sh, a future .cmake or
# vendored data file) changes the image while leaving the stamp intact --
# which is exactly the stale-image failure the stamp exists to catch.
#
# The cost is the other direction: editing a host-side-only file
# (run-oracle.sh, capture-dynamics-fixtures.py) invalidates every built image
# and forces a rebuild that changes nothing inside it. That is the safe
# direction of the trade -- a spurious rebuild costs build time, a missed one
# costs a wrong fixture that looks exactly like a good one.
#
# The file digest alone was never the whole story: a `docker build
# --build-arg`, an env-var override, or upstream moving a mutable base tag
# all change the image without touching a file here. `oracle_stamp` closes
# that by hashing the *resolved* non-file build inputs alongside the files;
# see its comment for which bypass each one closes.
#
# Paths are relative on purpose: sha256sum hashes the name alongside the
# bytes, and these sources sit at /ws/oracle-src inside the image but under
# the repo out here.
#
# `sort` runs under LC_ALL=C so the digest is a function of the bytes alone.
# Collation is locale-dependent: this host sorts case-insensitively
# (`build.sh` before `CMakeLists.txt`), the container's C locale sorts by
# byte (`CMakeLists.txt` first). Same files, same contents, different
# concatenation order, different digest -- which run-oracle.sh reports as a
# stale image that rebuilding cannot fix.
set -euo pipefail

# Files only -- deliberately NOT the tag input. `oracle_stamp` is. Two
# functions that both return a hex digest, only one of which names a real
# image, is the same footgun shape as the bypasses above: passing this one
# to `oracle_image_tag` yields a tag that has never been built, and the
# failure reads as a stale image. The name is the guard; there is no way to
# make it a type here.
oracle_file_digest() {  # <tools/moveit-oracle dir>
  cd "$1" && find . -type f -print0 |
    LC_ALL=C sort -z | xargs -0 sha256sum | sha256sum | cut -d' ' -f1
}

# The canonical non-file build inputs. These live here, not in build.sh and
# not as Dockerfile defaults, because `oracle_stamp` has to hash the same
# values the build actually used -- two definitions is how they drift.
#
# ORACLE_BASE_IMAGE is pinned by manifest digest, not by the `rolling-ci`
# tag. A tag is mutable: upstream can move it and every local file digest
# stays identical while the image underneath changes. Repinning is a
# deliberate edit to this line, which changes the stamp, which changes the
# tag -- so a new base cannot arrive silently.
#
# There is deliberately no ROS_DISTRO here. It used to be a build arg, and
# it was one of the bypasses: `--build-arg ROS_DISTRO=...` selected the base
# tag while every `RUN` in the image kept reading the base image's own
# `ROS_DISTRO` ENV, which overrides a same-named ARG. With the base pinned
# by digest the distro is a property of that image and nothing else, so the
# argument that could disagree with it no longer exists rather than being
# checked for.
#
# Each takes an already-exported value if there is one. That is the single
# override mechanism: export before sourcing this file, and every consumer
# -- build.sh's `--build-arg`s, the Dockerfile's stamp, run-oracle.sh's
# check -- sees the same value, because they all read it from here. The
# Dockerfile depends on this: it exports the resolved ARGs before sourcing,
# so the stamp records what the build actually used and not these defaults.
ORACLE_BASE_IMAGE="${ORACLE_BASE_IMAGE:-moveit/moveit2@sha256:7c394edd1faac3eb2dda1519df0cda58e9d870c42aaa7a7678934f76e3d1acc0}"
# pilz_industrial_motion_planner joins moveit_core here for the
# `pilz_trajectory` op (Phase 8's LIN/PTP/CIRC completion condition). It is
# the one planner this file builds from source rather than taking prebuilt
# from the base image the way `plan` takes OMPL: pilz is a moveit2 package,
# so it tracks MOVEIT2_SHA, and a prebuilt copy would silently pair oracle
# trajectories with a different revision than every other op reports.
#
# moveit_kinematics comes with it, not for a generator but for LIN and CIRC:
# both run IK on their Cartesian goal, and a URDF+SRDF-only RobotModel has no
# solver, so both returned NO_IK_SOLUTION until `ensureKinematicsSolver`
# could load `kdl_kinematics_plugin/KDLKinematicsPlugin` by name.
ORACLE_MOVEIT2_PACKAGES="${ORACLE_MOVEIT2_PACKAGES:-moveit_core moveit_resources_fanuc_description pilz_industrial_motion_planner moveit_kinematics}"
ORACLE_MOVEIT2_SHA="${ORACLE_MOVEIT2_SHA:-e017c91ee12984393a28ba246075c65f69cde3bf}"

# Serialized build inputs, one `NAME=value` per line. A deliberately
# re-pinned build produces a distinguishable image instead of impersonating
# the canonical one, because the override lands in the stamp.
oracle_build_inputs() {
  printf 'BASE_IMAGE=%s\nMOVEIT2_PACKAGES=%s\nMOVEIT2_SHA=%s\n' \
    "$ORACLE_BASE_IMAGE" "$ORACLE_MOVEIT2_PACKAGES" "$ORACLE_MOVEIT2_SHA"
}

# What build.sh tags with, the Dockerfile stamps, and run-oracle.sh checks:
# the file digest and the resolved build inputs, together.
#
# `oracle_file_digest` covers everything under this directory. This adds the
# inputs that are not files, closing the three bypasses that used to be
# documented as known gaps: a hand-passed `--build-arg MOVEIT2_PACKAGES`, an
# env-overridden `MOVEIT2_SHA` (build.sh's pin check compares the env value
# against the checkout, so moving both passed it), and upstream repushing
# the mutable base tag.
#
# The intermediate `--target moveit` stage carries no stamp file at all, so
# run-oracle.sh's `<missing or unstamped>` branch rejects it. That is relied
# on deliberately here: the stamp is written only in the `oracle` stage.
oracle_stamp() {  # <tools/moveit-oracle dir>
  { oracle_file_digest "$1"; oracle_build_inputs; } | sha256sum | cut -d' ' -f1
}

# The tag carries the stamp so that two worktrees with different oracle
# sources build two different images instead of overwriting one shared
# `:latest`. Seven concurrent worktrees share this docker daemon; a single
# mutable tag makes whoever builds last silently the oracle for everyone.
oracle_image_tag() {  # <stamp>
  echo "moveit-rs/oracle:${1:0:16}"
}
