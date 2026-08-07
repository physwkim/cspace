#!/usr/bin/env bash
set -euo pipefail

# Builds libccd (pinned to tag v2.1, CCD_DOUBLE -- same pin
# tools/mpr-vs-epa/build.sh uses) from source and links
# cone_orientation_witness.c against it. See that file's own header
# comment for what the resulting binary computes and why.
#
# libccd has no system package here (`pkg-config --exists ccd` fails) --
# LIBCCD_SRC must point at a git checkout of
# https://github.com/danfis/libccd, checked out exactly at tag v2.1.
# Defaults to the machine-local checkout tools/mpr-vs-epa already uses.

LIBCCD_SRC="${LIBCCD_SRC:-/home/stevek/work/libccd}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/../ci/gate-lib.sh"
require_caller_tree "$(cd "$HERE/../.." && pwd)"
BUILD_DIR="$HERE/build"

if [ ! -d "$LIBCCD_SRC" ]; then
    echo "error: LIBCCD_SRC=$LIBCCD_SRC does not exist -- set LIBCCD_SRC to a libccd git checkout" >&2
    exit 1
fi

tag=$(git -C "$LIBCCD_SRC" describe --tags --exact-match 2>/dev/null || true)
if [ "$tag" != "v2.1" ]; then
    echo "error: $LIBCCD_SRC is not checked out exactly at tag v2.1 (got: '${tag:-<none>}')" \
         "-- this harness is pinned to the same build tools/mpr-vs-epa uses" >&2
    exit 1
fi

mkdir -p "$BUILD_DIR/libccd"
cmake -S "$LIBCCD_SRC" -B "$BUILD_DIR/libccd" -DCCD_DOUBLE=ON -DCMAKE_BUILD_TYPE=Release >/dev/null
# `ccd` only -- libccd's own CMakeLists also defines its testsuite/benchmark
# executables under the default target; this harness needs just the library.
cmake --build "$BUILD_DIR/libccd" --target ccd >/dev/null

gcc -O2 -Wall -Wextra \
    -I"$LIBCCD_SRC/src" \
    -I"$BUILD_DIR/libccd/src" \
    -o "$BUILD_DIR/cone_orientation_witness" \
    "$HERE/cone_orientation_witness.c" \
    "$BUILD_DIR/libccd/src/libccd.so" \
    -lm \
    -Wl,-rpath,"$BUILD_DIR/libccd/src"

echo "built: $BUILD_DIR/cone_orientation_witness"
