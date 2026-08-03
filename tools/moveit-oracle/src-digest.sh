#!/bin/bash
# Prints the digest of the oracle's C++ sources, and the image tag derived
# from it.
#
# One definition, used by both build.sh (which stamps and tags with it) and
# run-oracle.sh (which resolves the tag and verifies the stamp). Computing it
# separately in each would let the two drift into disagreeing about which
# image is current -- and the disagreement would look exactly like a stale
# image.
#
# Paths are relative on purpose: sha256sum hashes the name alongside the
# bytes, and these sources sit at /ws/oracle-src inside the image but under
# the repo out here.
set -euo pipefail

oracle_src_digest() {  # <tools/moveit-oracle dir>
  cd "$1" && find . -type f \
    \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name 'CMakeLists.txt' \) \
    -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -d' ' -f1
}

# The tag carries the digest so that two worktrees with different oracle
# sources build two different images instead of overwriting one shared
# `:latest`. Seven concurrent worktrees share this docker daemon; a single
# mutable tag makes whoever builds last silently the oracle for everyone.
oracle_image_tag() {  # <digest>
  echo "moveit-rs/oracle:${1:0:16}"
}
