#!/usr/bin/env bash
set -euo pipefail

# Builds libccd (pinned to tag v2.1, CCD_DOUBLE -- the same build round
# 16/17/21's own investigation measured against) from source and links
# mpr_case104.c against it. See that file's own header comment for what
# the resulting binary computes and why.
#
# libccd has no system package on this machine (`pkg-config --exists ccd`
# fails) -- LIBCCD_SRC must point at a git checkout of
# https://github.com/danfis/libccd, checked out exactly at tag v2.1.
# Defaults to the machine-local checkout this port's own investigation
# already used.

LIBCCD_SRC="${LIBCCD_SRC:-/home/stevek/work/libccd}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$HERE/build"

if [ ! -d "$LIBCCD_SRC" ]; then
    echo "error: LIBCCD_SRC=$LIBCCD_SRC does not exist -- set LIBCCD_SRC to a libccd git checkout" >&2
    exit 1
fi

tag=$(git -C "$LIBCCD_SRC" describe --tags --exact-match 2>/dev/null || true)
if [ "$tag" != "v2.1" ]; then
    echo "error: $LIBCCD_SRC is not checked out exactly at tag v2.1 (got: '${tag:-<none>}')" \
         "-- this harness is pinned to the same build round 16/17/21 measured against" >&2
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
    -o "$BUILD_DIR/mpr_case104" \
    "$HERE/mpr_case104.c" \
    "$LIBCCD_SRC/src/testsuites/support.c" \
    "$BUILD_DIR/libccd/src/libccd.so" \
    -lm \
    -Wl,-rpath,"$BUILD_DIR/libccd/src"

echo "built: $BUILD_DIR/mpr_case104"
