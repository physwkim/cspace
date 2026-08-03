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
# Not covered: `docker build --build-arg` overrides of ROS_DISTRO /
# MOVEIT2_PACKAGES, which change the image without touching a file. Nothing
# in this repo passes them.
#
# Paths are relative on purpose: sha256sum hashes the name alongside the
# bytes, and these sources sit at /ws/oracle-src inside the image but under
# the repo out here.
set -euo pipefail

oracle_src_digest() {  # <tools/moveit-oracle dir>
  cd "$1" && find . -type f -print0 |
    sort -z | xargs -0 sha256sum | sha256sum | cut -d' ' -f1
}

# The tag carries the digest so that two worktrees with different oracle
# sources build two different images instead of overwriting one shared
# `:latest`. Seven concurrent worktrees share this docker daemon; a single
# mutable tag makes whoever builds last silently the oracle for everyone.
oracle_image_tag() {  # <digest>
  echo "moveit-rs/oracle:${1:0:16}"
}
