// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Prints `btGjkEpaSolver2::Penetration`, `::Distance`,
// `btGjkEpaPenetrationDepthSolver::calcPenDepth` and
// `btGjkPairDetector::getClosestPointsNonVirtual` results for the shape pairs
// `crates/cspace-bullet/src/{epa,pen_depth,gjk}.rs`'s tests assert on, so
// those assertions carry Bullet's own answer rather than a hand-derived one.
//
// This exists because EPA's answer is not the geometric one. On a
// configuration where the enclosing tetrahedron puts the origin on one of
// its edges, upstream's silhouette walk reaches an already-passed face,
// `expand` returns false, and `Evaluate` breaks out with `InvalidHull`
// leaving `outer` at whatever face was best at the time -- a defensible
// answer for a degenerate input, but not the depth a human would compute.
// `box_box_deep_x` below is exactly that case: two unit boxes overlapping
// 0.5 m along x, for which the geometric depth is 0.5 along x and Bullet
// reports 0.288675129 along a corner diagonal. A port asserting 0.5 there
// would be asserting against Bullet, so the fixtures come from here.
//
// Every transform basis is written as literal float arithmetic that is
// identical in C++ and in Rust (`2.0/3.0`, not `cos(theta)`), so the two
// sides are fed bit-identical inputs and any difference in the output is
// the algorithm's, not the setup's.

#include <cstdio>

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"
#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"

// `%.9g` round-trips a `float` exactly, so a fixture transcribed from this
// output and parsed back as `f32` is the same bit pattern the C++ held.
static void emit(const char* name, bool ok, const btGjkEpaSolver2::sResults& r)
{
	printf("%s|%d|%d|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g\n",
	       name, (int)ok, (int)r.status, (double)r.distance,
	       (double)r.normal[0], (double)r.normal[1], (double)r.normal[2],
	       (double)r.witnesses[0][0], (double)r.witnesses[0][1], (double)r.witnesses[0][2],
	       (double)r.witnesses[1][0], (double)r.witnesses[1][1], (double)r.witnesses[1][2]);
}

// `= {}` is deliberate. `Initialize` zeroes `witnesses` and `status` and
// leaves `normal`/`distance` alone, so on any path that does not reach EPA
// those two are whatever the caller's stack held -- not a fixture. Zeroing
// them here is what `Results::default()` does on the Rust side, so the two
// sides start from the same defined state and every printed field is one
// the algorithm either wrote or provably did not touch.
static void pen(const char* name, const btConvexShape* a, const btTransform& ta,
                const btConvexShape* b, const btTransform& tb, const btVector3& guess,
                bool usemargins)
{
	btGjkEpaSolver2::sResults r = {};
	const bool ok = btGjkEpaSolver2::Penetration(a, ta, b, tb, guess, r, usemargins);
	emit(name, ok, r);
}

static void dist(const char* name, const btConvexShape* a, const btTransform& ta,
                 const btConvexShape* b, const btTransform& tb, const btVector3& guess)
{
	btGjkEpaSolver2::sResults r = {};
	const bool ok = btGjkEpaSolver2::Distance(a, ta, b, tb, guess, r);
	emit(name, ok, r);
}

// `calcPenDepth`'s three out-parameters. `v` is only meaningful together
// with the return value, so all four are printed; the guess list's first
// entry is the only part of that list a working pair can observe, since the
// loop stops at the first guess that answers.
static void pendepth(const char* name, const btConvexShape* a, const btTransform& ta,
                     const btConvexShape* b, const btTransform& tb)
{
	btVoronoiSimplexSolver simplex;
	btGjkEpaPenetrationDepthSolver solver;
	btVector3 v(0, 0, 0), wa(0, 0, 0), wb(0, 0, 0);
	const bool ok = solver.calcPenDepth(simplex, a, b, ta, tb, v, wa, wb, 0);
	printf("%s|%d|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g\n",
	       name, (int)ok,
	       (double)v[0], (double)v[1], (double)v[2],
	       (double)wa[0], (double)wa[1], (double)wa[2],
	       (double)wb[0], (double)wb[1], (double)wb[2]);
}

// A `btManifoldResult` that replaces `addContactPoint` instead of extending
// it, exactly as MoveIt's `TesseractBroadphaseBridgedManifoldResult` does
// (`bullet_utils.hpp:571-630`). Nothing reaches the manifold's point cache, so
// the count printed below is this counter and not `getNumContacts()`.
struct RecordingResult : public btManifoldResult
{
	int count;
	btVector3 normal;
	btVector3 point;
	btScalar depth;

	RecordingResult(const btCollisionObjectWrapper* a, const btCollisionObjectWrapper* b)
		: btManifoldResult(a, b), count(0), normal(0, 0, 0), point(0, 0, 0), depth(0)
	{
	}

	void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld,
	                     btScalar d) override
	{
		++count;
		normal = normalOnBInWorld;
		point = pointInWorld;
		depth = d;
	}
};

// One `processCollision` through the dispatcher MoveIt configures, driven
// exactly as `TesseractCollisionPairCallback::processOverlap` drives it
// (`bullet_utils.cpp:517-533`): both wrappers built from the world
// transforms, `findAlgorithm` asked once, the threshold assigned onto the
// result before the call.
//
// `closestPointDistanceThreshold` is that callback's `contact_distance_`.
// On the *continuous* path it is always zero: `BulletBVHManager`'s
// constructor seeds `contact_distance_` to `BULLET_DEFAULT_CONTACT_DISTANCE`
// (`bullet_bvh_manager.cpp:55`, `= 0.00f`), and
// `checkRobotCollisionHelperCCD` never calls `setContactDistanceThreshold`
// -- the two `MAX_DISTANCE_MARGIN` assignments are both on the discrete
// `manager_` (`collision_env_bullet.cpp:127,187`). The non-zero rows below
// are therefore not reachable configurations; they are there so the term
// can be seen entering the sum at all, which a row that always passes zero
// cannot show.
static void cc(const char* name, btConvexShape* a, const btTransform& ta, btConvexShape* b,
               const btTransform& tb, btScalar closestPointDistanceThreshold)
{
	btDefaultCollisionConfiguration config;
	btCollisionDispatcher dispatcher(&config);
	dispatcher.setDispatcherFlags(dispatcher.getDispatcherFlags() &
	                              ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);

	btCollisionObject obj_a, obj_b;
	obj_a.setCollisionShape(a);
	obj_a.setWorldTransform(ta);
	obj_b.setCollisionShape(b);
	obj_b.setWorldTransform(tb);

	btCollisionObjectWrapper wrap_a(nullptr, a, &obj_a, ta, -1, -1);
	btCollisionObjectWrapper wrap_b(nullptr, b, &obj_b, tb, -1, -1);

	RecordingResult result(&wrap_a, &wrap_b);
	result.m_closestPointDistanceThreshold = closestPointDistanceThreshold;

	btCollisionAlgorithm* algo =
	    dispatcher.findAlgorithm(&wrap_a, &wrap_b, nullptr, BT_CLOSEST_POINT_ALGORITHMS);

	btDispatcherInfo info;
	algo->processCollision(&wrap_a, &wrap_b, info, &result);

	// The cut-off `processCollision` built for its GJK query, recomputed here
	// from the same four terms so the row records it directly rather than
	// leaving it inferable only through the contact.
	btScalar sum = a->getMargin() + b->getMargin() + gContactBreakingThreshold +
	               closestPointDistanceThreshold;

	printf("%s|%d|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g\n", name, result.count,
	       (double)result.normal[0], (double)result.normal[1], (double)result.normal[2],
	       (double)result.point[0], (double)result.point[1], (double)result.point[2],
	       (double)result.depth, (double)(sum * sum));

	algo->~btCollisionAlgorithm();
	dispatcher.freeCollisionAlgorithm(algo);
}

// `btVec3PointTriDist2` is btGjkPairDetector.cpp's own -- part of the libCCD
// GJK it carries -- and has external linkage but no header, so it is declared
// here rather than included. It is probed directly because the pre-pass that
// uses it feeds a single bit (`status`) into the detector, and a sweep of
// 230,400 shape/transform cells found only 22 in which that bit changes the
// answer: driving its arithmetic through the detector alone would leave the
// double-precision barycentric solve inside it effectively unpinned.
btScalar btVec3PointTriDist2(const btVector3* P, const btVector3* x0, const btVector3* B,
                             const btVector3* C, btVector3* witness);

// Both arms, because they are not the same arithmetic: with a witness the
// distance is recomputed from the witness point, without one it is
// accumulated in `double` (the in-triangle branch) or taken from the
// recycling path of `btVec3PointSegmentDist2` (the nearest-edge branch).
static void tri(const char* name, const btVector3& p, const btVector3& x0, const btVector3& b,
                const btVector3& c)
{
	btVector3 w(0, 0, 0);
	const btScalar with = btVec3PointTriDist2(&p, &x0, &b, &c, &w);
	const btScalar without = btVec3PointTriDist2(&p, &x0, &b, &c, 0);
	printf("%s|%.9g|%.9g|%.9g|%.9g|%.9g\n", name, (double)with, (double)without, (double)w[0],
	       (double)w[1], (double)w[2]);
}

// `btStorageResult` is abstract -- it leaves `setShapeIdentifiersA/B` pure --
// and its constructor initializes only `m_distance`, leaving the two vectors
// as stack garbage on any query that emits no contact. Zeroing them is what
// `StorageResult::new()` does on the Rust side, so a "no contact" row is a
// fixture rather than a reading of this process's stack.
struct ProbeResult : public btStorageResult
{
	ProbeResult()
	{
		m_normalOnSurfaceB.setValue(0, 0, 0);
		m_closestPointInB.setValue(0, 0, 0);
	}
	virtual void setShapeIdentifiersA(int, int) {}
	virtual void setShapeIdentifiersB(int, int) {}
};

// The detector's whole observable output: what it wrote into the sink, the
// three debug counters that record which of the exits produced it, and the
// cached axis/distance it leaves behind for the next query to seed from.
//
// `usePenSolver=false` covers the branch upstream guards with
// `if (m_penetrationDepthSolver)`; `ignoreMargin=true` is the configuration
// MoveIt's continuous check runs, where the cast hull carries the sweep and
// the margins are zeroed.
static void gjk(const char* name, const btConvexShape* a, const btTransform& ta,
                const btConvexShape* b, const btTransform& tb, bool ignoreMargin,
                btScalar maxDistanceSquared, bool usePenSolver)
{
	btVoronoiSimplexSolver simplex;
	btGjkEpaPenetrationDepthSolver penSolver;
	btGjkPairDetector detector(a, b, &simplex, usePenSolver ? &penSolver : 0);
	detector.setIgnoreMargin(ignoreMargin);

	btGjkPairDetector::ClosestPointInput input;
	input.m_transformA = ta;
	input.m_transformB = tb;
	input.m_maximumDistanceSquared = maxDistanceSquared;

	ProbeResult out;
	detector.getClosestPointsNonVirtual(input, out, 0);

	const btVector3 axis = detector.getCachedSeparatingAxis();
	printf("%s|%d|%d|%d|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g\n",
	       name, detector.m_lastUsedMethod, detector.m_degenerateSimplex, detector.m_curIter,
	       (double)out.m_distance,
	       (double)out.m_normalOnSurfaceB[0], (double)out.m_normalOnSurfaceB[1],
	       (double)out.m_normalOnSurfaceB[2],
	       (double)out.m_closestPointInB[0], (double)out.m_closestPointInB[1],
	       (double)out.m_closestPointInB[2],
	       (double)detector.getCachedSeparatingDistance(),
	       (double)axis[0], (double)axis[1], (double)axis[2]);
}

static btTransform at(btScalar x, btScalar y, btScalar z)
{
	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(x, y, z));
	return t;
}

// 60 degrees about (1,1,1)/sqrt(3). Every entry is exactly representable as
// the quotient of two small integers, so the Rust fixture spells the same
// basis without going through a quaternion.
static btTransform rot60_at(btScalar x, btScalar y, btScalar z)
{
	const btScalar p = btScalar(2.0) / btScalar(3.0);
	const btScalar m = btScalar(-1.0) / btScalar(3.0);
	btTransform t;
	t.setBasis(btMatrix3x3(p, m, p,
	                       p, p, m,
	                       m, p, p));
	t.setOrigin(btVector3(x, y, z));
	return t;
}

// `btDbvt`'s leaf-visit order. A `btCompoundShape` built with
// `BULLET_COMPOUND_USE_DYNAMIC_AABB` culls its children through this tree,
// and `collideTVNoStackAlloc` yields them in tree order rather than child
// order -- which MoveIt's `pair_done`/`done` early-exit can observe. The
// order is decided entirely by `Select`, so these rows pin the tree's shape
// through the only output it has.
struct RecordingCollide : btDbvt::ICollide
{
	int seen[64];
	int count;

	RecordingCollide() : count(0) {}

	void Process(const btDbvtNode* n)
	{
		if (count < 64)
			seen[count++] = n->dataAsInt;
	}
};

static btDbvtVolume cube(btScalar x, btScalar y, btScalar z)
{
	return btDbvtVolume::FromMM(btVector3(x - 0.5f, y - 0.5f, z - 0.5f),
	                            btVector3(x + 0.5f, y + 0.5f, z + 0.5f));
}

// `op` drives the one edit each row exercises after the inserts:
//   0 = none, 1 = update child 0 to `cube(100,0,0)`, 2 = remove child 1.
static void dbvt(const char* name, const btVector3* centres, int n, const btDbvtVolume& query,
                 int op)
{
	btDbvt tree;
	btDbvtNode* leaves[64];
	for (int i = 0; i < n; ++i)
	{
		leaves[i] = tree.insert(cube(centres[i].x(), centres[i].y(), centres[i].z()),
		                        reinterpret_cast<void*>(static_cast<size_t>(i)));
	}
	if (op == 1)
	{
		btDbvtVolume moved = cube(100.f, 0.f, 0.f);
		tree.update(leaves[0], moved);
	}
	else if (op == 2)
	{
		tree.remove(leaves[1]);
	}

	RecordingCollide collide;
	btNodeStack stack;
	tree.collideTVNoStackAlloc(tree.m_root, query, stack, collide);

	printf("%s|%d|%d", name, collide.count, tree.m_leaves);
	for (int i = 0; i < collide.count; ++i)
		printf("|%d", collide.seen[i]);
	printf("\n");
}

int main()
{
	const btTransform id = at(0, 0, 0);
	const btVector3 gx(1, 0, 0);

	btBoxShape unit_box(btVector3(0.5f, 0.5f, 0.5f));
	unit_box.setMargin(0.f);
	btBoxShape flat_box(btVector3(0.4f, 0.7f, 0.25f));
	flat_box.setMargin(0.f);
	btBoxShape margin_box(btVector3(0.5f, 0.5f, 0.5f));  // keeps the 0.04 default
	btSphereShape sphere(0.5f);
	btSphereShape small_sphere(0.3f);
	btCylinderShapeZ cyl(btVector3(0.3f, 0.3f, 0.5f));
	cyl.setMargin(0.f);
	btConeShapeZ cone(0.25f, 0.8f);
	cone.setMargin(0.f);

	// MoveIt's hull order: every vertex added first, `setMargin(0)` after
	// (`bullet_utils.cpp:145-152,577`). Reproduced, because `addPoint`
	// order decides which of several equally-extreme vertices `maxDot`
	// returns and the margin at `addPoint` time is what the cached AABB
	// keeps.
	btConvexHullShape hull;
	const btScalar hp[8][3] = {
	    {0.3f, 0.2f, 0.1f}, {-0.3f, 0.2f, 0.1f}, {0.3f, -0.2f, 0.1f}, {-0.3f, -0.2f, 0.1f},
	    {0.3f, 0.2f, -0.1f}, {-0.3f, 0.2f, -0.1f}, {0.3f, -0.2f, -0.1f}, {-0.3f, -0.2f, -0.1f}};
	for (int i = 0; i < 8; ++i) hull.addPoint(btVector3(hp[i][0], hp[i][1], hp[i][2]));
	hull.setMargin(0.f);

	// Penetrating. `box_box_deep_x` is the degenerate case described above.
	//
	// The `usemargins=true` cases are not decoration: with margins off,
	// `Initialize` shrinks every shape to its core, and a sphere's core is a
	// point (`btSphereShape::getMargin` returns the radius), so every pair
	// involving a sphere separates unless the centres nearly coincide. That
	// is the same `enableMargin` flag `btGjkPairDetector` flips between its
	// two passes, so both settings have to be covered.
	pen("box_box_deep_x", &unit_box, id, &unit_box, at(0.5f, 0.f, 0.f), gx, false);
	pen("box_box_shallow_x", &unit_box, id, &unit_box, at(0.9f, 0.f, 0.f), gx, false);
	pen("box_box_offset", &unit_box, id, &unit_box, at(0.6f, 0.35f, -0.2f), gx, false);
	pen("box_box_rot60", &unit_box, id, &flat_box, rot60_at(0.7f, 0.2f, 0.1f), gx, false);
	pen("box_box_margins", &margin_box, id, &margin_box, at(0.95f, 0.f, 0.f), gx, true);
	pen("sphere_sphere", &sphere, id, &sphere, at(0.7f, 0.f, 0.f), gx, true);
	pen("sphere_box", &small_sphere, id, &unit_box, at(0.6f, 0.1f, 0.f), gx, true);
	pen("cyl_box", &cyl, id, &flat_box, at(0.5f, 0.1f, 0.2f), gx, false);
	pen("cyl_cyl_rot60", &cyl, id, &cyl, rot60_at(0.4f, 0.1f, 0.f), gx, false);
	pen("cone_box", &cone, id, &unit_box, at(0.55f, 0.1f, 0.3f), gx, false);
	pen("cone_sphere", &cone, id, &small_sphere, at(0.2f, 0.f, 0.45f), gx, true);
	pen("hull_box", &hull, id, &unit_box, at(0.7f, 0.05f, 0.f), gx, false);
	pen("hull_sphere_rot60", &hull, id, &small_sphere, rot60_at(0.4f, 0.1f, 0.05f), gx, true);

	// `Penetration` on a separated pair -- GJK returns Valid, so the EPA
	// branch is never entered and the result is the GJK distance.
	pen("box_box_separated", &unit_box, id, &unit_box, at(1.5f, 0.f, 0.f), gx, false);

	// Separated.
	dist("d_box_box_far", &unit_box, id, &unit_box, at(3.f, 0.f, 0.f), gx);
	dist("d_box_box_diag", &unit_box, id, &flat_box, at(2.f, 1.5f, 0.5f), gx);
	dist("d_sphere_box", &small_sphere, id, &unit_box, at(2.f, 0.4f, 0.f), gx);
	dist("d_cyl_cone", &cyl, id, &cone, at(1.6f, 0.3f, 0.2f), gx);
	dist("d_hull_sphere", &hull, id, &sphere, rot60_at(1.4f, 0.6f, 0.f), gx);
	dist("d_box_box_touching", &unit_box, id, &unit_box, at(1.f, 0.f, 0.f), gx);

	// `calcPenDepth`. `p_*_coincident` is the row that pins `safeNormalize`'s
	// fallback: with the centres on top of each other the first two guesses
	// are zero-length, and what the loop actually tries is `(1, 0, 0)`.
	pendepth("p_box_box_overlap", &margin_box, id, &margin_box, at(0.95f, 0.f, 0.f));
	pendepth("p_box_box_diagonal", &margin_box, id, &flat_box, at(0.6f, 0.35f, -0.2f));
	pendepth("p_box_box_coincident", &margin_box, id, &flat_box, id);
	pendepth("p_box_box_separated", &unit_box, id, &unit_box, at(3.f, 0.f, 0.f));
	pendepth("p_cone_cyl_rot60", &cone, id, &cyl, rot60_at(0.3f, 0.1f, 0.2f));

	// A sphere sunk into a cylinder's rim: EPA spends 126 support vertices on
	// this pair before the hull goes invalid, which is the only row here that
	// comes within two of `EPA_MAX_VERTICES`. `m_nextsv` counts the expansion's
	// vertices alone -- the initial tetrahedron's faces index GJK's store, not
	// `m_sv_store` -- so anything that folds those four into the same budget
	// stops at 124 and answers off the face it had two iterations earlier.
	pendepth("p_sphere_cyl_deep", &sphere, at(-0.15f, 0.f, -0.25f), &cyl, at(0.15f, 0.f, 0.25f));

	// `btGjkPairDetector::getClosestPointsNonVirtual`. `BT_LARGE_FLOAT` as
	// `m_maximumDistanceSquared` is what `ClosestPointInput()` defaults to, so
	// a row passing it is the unclipped query; `g_maxdist_clipped` is the same
	// pair with a cut-off tight enough that the contact is dropped, which is
	// the only way to see the sink's untouched state.
	const btScalar unclipped = btScalar(BT_LARGE_FLOAT);

	gjk("g_box_box_deep", &unit_box, id, &unit_box, at(0.5f, 0.f, 0.f), false, unclipped, true);
	gjk("g_box_box_shallow", &unit_box, id, &unit_box, at(0.9f, 0.f, 0.f), false, unclipped, true);
	gjk("g_box_box_touching", &unit_box, id, &unit_box, at(1.f, 0.f, 0.f), false, unclipped, true);
	gjk("g_box_box_separated", &unit_box, id, &unit_box, at(1.5f, 0.f, 0.f), false, unclipped, true);
	gjk("g_box_box_offset", &unit_box, id, &unit_box, at(0.6f, 0.35f, -0.2f), false, unclipped, true);
	gjk("g_box_box_rot60", &unit_box, id, &flat_box, rot60_at(0.7f, 0.2f, 0.1f), false, unclipped, true);

	// Non-zero margins: the shapes are shrunk by their margin before GJK and
	// the witness points pushed back out by it afterwards.
	gjk("g_margin_overlap", &margin_box, id, &margin_box, at(0.95f, 0.f, 0.f), false, unclipped, true);
	gjk("g_margin_separated", &margin_box, id, &margin_box, at(1.2f, 0.f, 0.f), false, unclipped, true);

	gjk("g_sphere_sphere", &sphere, id, &sphere, at(0.7f, 0.f, 0.f), false, unclipped, true);
	gjk("g_sphere_box", &small_sphere, id, &unit_box, at(0.6f, 0.1f, 0.f), false, unclipped, true);
	gjk("g_cyl_box", &cyl, id, &flat_box, at(0.5f, 0.1f, 0.2f), false, unclipped, true);
	gjk("g_cyl_cyl_rot60", &cyl, id, &cyl, rot60_at(0.4f, 0.1f, 0.f), false, unclipped, true);
	gjk("g_cone_box", &cone, id, &unit_box, at(0.55f, 0.1f, 0.3f), false, unclipped, true);
	gjk("g_cone_sphere", &cone, id, &small_sphere, at(0.2f, 0.f, 0.45f), false, unclipped, true);
	gjk("g_hull_box", &hull, id, &unit_box, at(0.7f, 0.05f, 0.f), false, unclipped, true);
	gjk("g_hull_sphere_rot60", &hull, id, &small_sphere, rot60_at(0.4f, 0.1f, 0.05f), false, unclipped, true);

	// Coincident centres -- the case that drives the simplex degenerate.
	gjk("g_coincident", &margin_box, id, &flat_box, id, false, unclipped, true);

	// `setIgnoreMargin(true)`: MoveIt's continuous configuration.
	gjk("g_ccd_margin_boxes", &margin_box, id, &margin_box, at(0.95f, 0.f, 0.f), true, unclipped, true);
	gjk("g_ccd_sphere_box", &sphere, id, &unit_box, at(0.9f, 0.f, 0.f), true, unclipped, true);

	// The `m_maximumDistanceSquared` cut-off, and the null-solver branch on a
	// pair that would otherwise reach EPA.
	gjk("g_maxdist_clipped", &unit_box, id, &unit_box, at(3.f, 0.f, 0.f), false, btScalar(1.f), true);
	gjk("g_no_pen_solver", &unit_box, id, &unit_box, at(0.5f, 0.f, 0.f), false, unclipped, false);

	// Far from the world origin: everything the detector computes runs on the
	// pair recentred about `positionOffset`, and only the emitted point is
	// shifted back.
	gjk("g_far_from_origin", &unit_box, at(100.f, 100.f, 100.f), &unit_box,
	    at(100.9f, 100.f, 100.f), false, unclipped, true);

	// The rows above reach `m_lastUsedMethod` 1, 2 and 3 only, and none of
	// them changes its answer when the libCCD pre-pass's verdict is
	// discarded. These were found by sweeping 230,400 shape/transform cells
	// and keeping the ones that do reach a further exit, so that every branch
	// the port transcribes is pinned by at least one row rather than by the
	// common case alone.
	//
	// `g_prepass_*` are cells whose result changes if `status = 0` never
	// reaches the penetration condition -- the only evidence that the whole
	// libCCD pass has to be ported at all.
	gjk("g_prepass_flip", &flat_box, id, &cone, rot60_at(0.f, 0.8f, 0.6f), false, unclipped, true);
	gjk("g_prepass_rescue", &flat_box, id, &cone, rot60_at(0.1f, 0.8f, 0.6f), false, unclipped, true);
	gjk("g_prepass_margins", &sphere, id, &unit_box, rot60_at(0.7f, 0.1f, 0.1f), false, unclipped, true);
	gjk("g_prepass_degen12", &sphere, id, &cyl, at(0.3f, 0.f, 0.5f), false, unclipped, true);

	// `m_lastUsedMethod` 10 (normal reverted), 8 (EPA no deeper than GJK),
	// 6 (second-GJK rescue) and 5 (that rescue rejected).
	gjk("g_normal_reverted", &unit_box, id, &unit_box, at(0.f, 0.1f, 0.1f), false, unclipped, true);
	gjk("g_epa_not_deeper", &sphere, id, &cyl, at(0.3f, 0.f, 0.f), false, unclipped, true);
	gjk("g_second_gjk_rescue", &cyl, id, &unit_box, at(0.8f, 0.5f, 0.7f), false, unclipped, true);
	gjk("g_rescue_rejected", &unit_box, id, &cyl, at(0.8f, 0.5f, 0.7f), false, unclipped, true);
	gjk("g_rescue_rot60", &cyl, id, &cone, rot60_at(0.6f, 0.3f, 0.f), false, unclipped, true);

	// `m_degenerateSimplex` 11 and 12 -- the two "not getting any closer"
	// exits, which the rows above never take.
	gjk("g_degen11", &unit_box, id, &unit_box, at(1.1f, 0.f, 0.1f), false, unclipped, true);
	gjk("g_degen12", &unit_box, id, &unit_box, rot60_at(1.1f, 0.9f, 0.7f), false, unclipped, true);
	gjk("g_degen3", &unit_box, id, &unit_box, at(1.1f, 0.f, 0.2f), false, unclipped, true);

	// `btVec3PointTriDist2` inside the libCCD pass: `name|withWitness|
	// withoutWitness|witness xyz`.
	tri("t_inside", btVector3(0.f, 0.f, 1.f), btVector3(0.f, 0.f, 0.f), btVector3(1.f, 0.f, 0.f),
	    btVector3(0.f, 1.f, 0.f));
	tri("t_beyond_b", btVector3(2.f, -1.f, 0.5f), btVector3(0.f, 0.f, 0.f),
	    btVector3(1.f, 0.f, 0.f), btVector3(0.f, 1.f, 0.f));
	tri("t_beyond_c", btVector3(-1.f, 2.f, -0.5f), btVector3(0.f, 0.f, 0.f),
	    btVector3(1.f, 0.f, 0.f), btVector3(0.f, 1.f, 0.f));
	tri("t_behind_x0", btVector3(-1.f, -1.f, 0.25f), btVector3(0.f, 0.f, 0.f),
	    btVector3(1.f, 0.f, 0.f), btVector3(0.f, 1.f, 0.f));
	tri("t_past_bc", btVector3(1.f, 1.f, 0.1f), btVector3(0.f, 0.f, 0.f), btVector3(1.f, 0.f, 0.f),
	    btVector3(0.f, 1.f, 0.f));
	tri("t_on_face", btVector3(0.25f, 0.25f, 0.f), btVector3(0.f, 0.f, 0.f),
	    btVector3(1.f, 0.f, 0.f), btVector3(0.f, 1.f, 0.f));
	tri("t_origin_offset", btVector3(0.f, 0.f, 0.f), btVector3(-0.3f, 0.4f, 0.2f),
	    btVector3(0.7f, -0.2f, 0.9f), btVector3(0.1f, 0.6f, -0.8f));
	tri("t_sliver", btVector3(0.f, 0.f, 0.f), btVector3(1.f, 0.f, 0.f),
	    btVector3(1.0001f, 1e-4f, 0.f), btVector3(1.0002f, 2e-4f, 1e-5f));
	tri("t_large", btVector3(100.f, 100.f, 100.f), btVector3(-50.f, 0.f, 0.f),
	    btVector3(50.f, 0.f, 0.f), btVector3(0.f, 70.f, 30.f));

	// `btConvexConvexAlgorithm::processCollision`, driven the way MoveIt
	// drives it: a real `btCollisionDispatcher` with the relative
	// contact-breaking threshold cleared, `findAlgorithm` asked for
	// `BT_CLOSEST_POINT_ALGORITHMS`, and a `btManifoldResult` subclass that
	// overrides `addContactPoint` outright rather than letting the base cache
	// into the manifold -- which is what makes the manifold's point array
	// unreachable and its breaking threshold the only part that is read.
	// Fields: `name|contacts|normalOnB xyz|pointOnB xyz|depth|maxDistSq`.
	//
	// Every pair below has a non-polyhedral shape on at least one side, so
	// `min0->isPolyhedral() && min1->isPolyhedral()` is false and the query
	// takes the GJK branch -- the only one the continuous path can reach,
	// since a `CastHullShape` is `CUSTOM_CONVEX_SHAPE_TYPE` and sits on one
	// side of every CCD pair. Two boxes would take the SAT/clipping branch
	// instead and pin arithmetic the port deliberately does not carry.
	//
	// The cone/cylinder rows straddle the cut-off with both margins at zero,
	// which puts it at `gContactBreakingThreshold` alone: their apex-to-face
	// gap is 0.015 and 0.03 against a 0.02 bound, and the third row brings
	// the 0.03 one back with `m_closestPointDistanceThreshold`.
	cc("cc_sphere_cyl_overlap", &sphere, id, &cyl, at(0.6f, 0.1f, 0.2f), 0.f);
	cc("cc_sphere_cyl_far", &sphere, id, &cyl, at(3.f, 0.f, 0.f), 0.f);
	cc("cc_cone_cyl_inside_cutoff", &cone, id, &cyl, at(0.f, 0.f, 0.915f), 0.f);
	cc("cc_cone_cyl_past_cutoff", &cone, id, &cyl, at(0.f, 0.f, 0.93f), 0.f);
	cc("cc_cone_cyl_threshold_widens", &cone, id, &cyl, at(0.f, 0.f, 0.93f), 0.25f);
	cc("cc_cone_cyl_deep", &cone, id, &cyl, at(0.1f, 0.f, 0.3f), 0.f);
	cc("cc_hull_cone_rot60", &hull, id, &cone, rot60_at(0.4f, 0.1f, 0.05f), 0.05f);
	cc("cc_margin_box_sphere", &margin_box, id, &small_sphere, at(0.85f, 0.05f, 0.f), 0.f);

	// `btDbvt` leaf order. Fields: `name|visited|leaves|data...`.
	//
	// `dbvt_line4` and `dbvt_line8` are the plain shape test: a query volume
	// containing everything, so the sequence is the tree's in-order walk and
	// nothing else. `dbvt_cull` narrows the query to one cube. `dbvt_update`
	// and `dbvt_remove` re-run `dbvt_line4`'s tree after the one edit
	// `btCompoundShape` performs on a built tree -- `updateChildTransform`
	// and `removeChildShapeByIndex` -- because both go through
	// `removeleaf`/`insertleaf` and can reshape the tree above the leaf they
	// name. `dbvt_grid` puts nine cubes on a plane so `Select`'s ties are
	// reached rather than assumed. The remaining rows each carry their own
	// comment below, and each exists for one comparison the rows here leave
	// away from its boundary.
	{
		const btVector3 line4[] = {
		    btVector3(0.f, 0.f, 0.f), btVector3(2.f, 0.f, 0.f),
		    btVector3(4.f, 0.f, 0.f), btVector3(6.f, 0.f, 0.f)};
		const btVector3 line8[] = {
		    btVector3(0.f, 0.f, 0.f), btVector3(2.f, 0.f, 0.f),
		    btVector3(4.f, 0.f, 0.f), btVector3(6.f, 0.f, 0.f),
		    btVector3(8.f, 0.f, 0.f), btVector3(10.f, 0.f, 0.f),
		    btVector3(12.f, 0.f, 0.f), btVector3(14.f, 0.f, 0.f)};
		btVector3 grid[9];
		for (int gx_i = 0; gx_i < 3; ++gx_i)
			for (int gy_i = 0; gy_i < 3; ++gy_i)
				grid[gx_i * 3 + gy_i] =
				    btVector3(btScalar(gx_i) * 2.f, btScalar(gy_i) * 2.f, 0.f);

		const btDbvtVolume all = btDbvtVolume::FromMM(btVector3(-1e6f, -1e6f, -1e6f),
		                                              btVector3(1e6f, 1e6f, 1e6f));
		// Eight cubes at the corners of a lattice, so `Proximity` sees a
		// non-zero difference on all three axes at once -- every row above
		// is planar in z, where a dropped `btFabs` on that component cannot
		// change a comparison.
		btVector3 cube8[8];
		for (int cx = 0; cx < 2; ++cx)
			for (int cy = 0; cy < 2; ++cy)
				for (int cz = 0; cz < 2; ++cz)
					cube8[cx * 4 + cy * 2 + cz] = btVector3(
					    btScalar(cx) * 3.f, btScalar(cy) * 3.f, btScalar(cz) * 3.f);

		dbvt("dbvt_line4", line4, 4, all, 0);
		dbvt("dbvt_line8", line8, 8, all, 0);
		dbvt("dbvt_cull", line4, 4, cube(2.f, 0.f, 0.f), 0);
		// The far end of the same tree. `insertleaf`'s ascent is what grows
		// the ancestors' volumes to reach it, so a tree that skipped the
		// ascent culls this query and visits nothing.
		dbvt("dbvt_cull_far", line4, 4, cube(6.f, 0.f, 0.f), 0);
		// Queries that abut a leaf exactly, one on each side, so `Intersect`
		// is asked its six comparisons at equality rather than only away
		// from the boundary.
		dbvt("dbvt_touch_lo", line4, 4, cube(-1.f, -1.f, -1.f), 0);
		dbvt("dbvt_touch_hi", line4, 4, cube(7.f, 1.f, 1.f), 0);
		dbvt("dbvt_update", line4, 4, all, 1);
		dbvt("dbvt_remove", line4, 4, all, 2);
		dbvt("dbvt_grid", grid, 9, all, 0);
		dbvt("dbvt_cube8", cube8, 8, all, 0);
	}

	// The row that pins the solve's precision. `u,v,w,p,q,r,s,t` are C `double`
	// inside this float build, and on the triangles above that costs nothing --
	// their barycentrics are exactly representable, so evaluating the same
	// expression in `btScalar` lands on the same floats. This triangle was
	// searched for: it is 40 ulps apart between the two, in the witness's y
	// most of all. The literals are the shortest decimals that round-trip to
	// these floats, so `gjk.rs` can spell the same triangle with the same
	// digits rather than a second decimal for the same value.
	tri("t_wide_solve", btVector3(-0.12572217f, 0.13450241f, -0.36173308f),
	    btVector3(-0.19358504f, -0.5220996f, 0.3660835f),
	    btVector3(-0.40353602f, 0.53909004f, 0.7409283f),
	    btVector3(0.7509048f, -0.42147017f, -0.16159362f));

	return 0;
}
