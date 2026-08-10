#!/usr/bin/env bash
set -euo pipefail

# Builds probe.cpp against the pinned bullet3 checkout in third_party/ and
# runs it, printing the fixture lines `crates/cspace-bullet/src/epa.rs`'s
# tests assert. See probe.cpp's own header for what it computes and why the
# fixtures cannot be derived by hand.
#
# The compile flags are the ones that make the C++ and the Rust comparable
# at all: no -march, so GCC has no FMA to contract into, and no
# BT_USE_DOUBLE_PRECISION, so btScalar stays float. btScalar.h:216-244
# leaves BT_USE_SSE/BT_USE_SIMD_VECTOR3/BT_USE_SSE_IN_API undefined on
# non-Apple Linux, so btVector3 is the scalar struct linear_math.rs ports.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
. "$HERE/../ci/gate-lib.sh"
require_caller_tree "$REPO_ROOT"

BULLET_SRC="${BULLET_SRC:-$REPO_ROOT/third_party/bullet3}"
BULLET_PIN=7dee3436e747958e7088dfdcea0e4ae031ce619e
BUILD_DIR="$HERE/build"

if [ ! -d "$BULLET_SRC" ]; then
    echo "error: BULLET_SRC=$BULLET_SRC does not exist -- point it at a bullet3 checkout" >&2
    exit 1
fi

head=$(git -C "$BULLET_SRC" rev-parse HEAD 2>/dev/null || true)
if [ "$head" != "$BULLET_PIN" ]; then
    echo "error: $BULLET_SRC is at '${head:-<none>}', not the pin $BULLET_PIN (tag 3.24)" \
         "-- cspace-bullet is a port of that revision and these fixtures are its output" >&2
    exit 1
fi

mkdir -p "$BUILD_DIR"

# btGjkEpa2.cpp reaches btConvexShape, which reaches the whole shape
# hierarchy; btPolyhedralConvexShape reaches btConvexPolyhedron and the hull
# computer. This is the transitive closure of that, not a chosen subset.
g++ -O2 -Wall -Wextra -std=c++11 \
    -I"$BULLET_SRC/src" \
    -o "$BUILD_DIR/probe" \
    "$HERE/probe.cpp" \
    "$BULLET_SRC/src/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp" \
    "$BULLET_SRC/src/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp" \
    "$BULLET_SRC/src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp" \
    "$BULLET_SRC/src/BulletCollision/CollisionShapes/btBoxShape.cpp" \
    "$BULLET_SRC/src/BulletCollision/CollisionShapes/btCollisionShape.cpp" \
    "$BULLET_SRC/src/BulletCollision/CollisionShapes/btConeShape.cpp" \
    "$BULLET_SRC/src/BulletCollision/CollisionShapes/btConvexHullShape.cpp" \
    "$BULLET_SRC/src/BulletCollision/CollisionShapes/btConvexInternalShape.cpp" \
    "$BULLET_SRC/src/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp" \
    "$BULLET_SRC/src/BulletCollision/CollisionShapes/btConvexShape.cpp" \
    "$BULLET_SRC/src/BulletCollision/CollisionShapes/btCylinderShape.cpp" \
    "$BULLET_SRC/src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp" \
    "$BULLET_SRC/src/BulletCollision/CollisionShapes/btSphereShape.cpp" \
    "$BULLET_SRC/src/LinearMath/btAlignedAllocator.cpp" \
    "$BULLET_SRC/src/LinearMath/btConvexHull.cpp" \
    "$BULLET_SRC/src/LinearMath/btConvexHullComputer.cpp" \
    "$BULLET_SRC/src/LinearMath/btGeometryUtil.cpp" \
    "$BULLET_SRC/src/LinearMath/btSerializer.cpp" \
    "$BULLET_SRC/src/LinearMath/btVector3.cpp"

exec "$BUILD_DIR/probe"
