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
#include <cstring>

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"
#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "LinearMath/btAabbUtil2.h"

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

// `BroadphaseNativeTypes` and the three ordering predicates built on it
// (`btBroadphaseProxy.h:27-80,164-186`). The values are positions in an
// unnumbered C enum and the predicates are `<`/`>` against its marker
// entries, so a port that retypes them by hand is one insertion away from
// wrong -- these rows are the enum reading itself out.
static void proxytype(const char* name, int value)
{
	printf("proxytype_%s|%d|%d|%d|%d\n", name, value, btBroadphaseProxy::isConvex(value) ? 1 : 0,
	       btBroadphaseProxy::isConcave(value) ? 1 : 0, btBroadphaseProxy::isCompound(value) ? 1 : 0);
}

// The type a built shape actually reports, which is what the dispatcher
// switches on -- separate from the enum above, because a shape reporting the
// wrong one of those values is a defect the enum rows cannot see.
static void shapetype(const char* name, const btCollisionShape* shape)
{
	printf("shapetype_%s|%d\n", name, shape->getShapeType());
}

// `getAverageSupport`'s `dynamic_cast<const btPolyhedralConvexShape*>`
// (`bullet_utils.hpp:351`) and the `getNumVertices`/`getVertex` pair it reads
// when that cast succeeds. The cast decides the whole branch -- a shape that
// fails it is asked for one support point instead of an average over the
// vertices tied for maximum support -- so a port that answers it wrongly for
// one shape silently runs different arithmetic, not a different number.
//
// Fields: `polycast_<name>|<0|1>|<n>`, then one `polyvert_<name>_<i>` row per
// vertex. Per vertex, not a summary: `getVertex` on a box synthesises corners
// from the half extents *with* margin, so which vertex sits at which index is
// itself the claim being pinned.
static void polyverts(const char* name, const btConvexShape* shape)
{
	const btPolyhedralConvexShape* pshape = dynamic_cast<const btPolyhedralConvexShape*>(shape);
	printf("polycast_%s|%d|%d\n", name, pshape ? 1 : 0, pshape ? pshape->getNumVertices() : 0);
	if (!pshape) return;

	for (int i = 0; i < pshape->getNumVertices(); ++i)
	{
		btVector3 v;
		pshape->getVertex(i, v);
		printf("polyvert_%s_%d|%.9g|%.9g|%.9g\n", name, i, (double)v[0], (double)v[1], (double)v[2]);
	}
}

// `btCompoundShape::getAabb` plus the leaf order of the tree it built while
// the children were added. The AABB is the accumulated `m_localAabb*` taken
// through `trans` -- so a row under a rotated transform is the only thing
// that separates "margin added to the half extents" from "margin added to
// the world box", and a row after `updateChildTransform(..., false)` is the
// only thing that shows the *stale* local AABB MoveIt deliberately keeps
// (`bullet_cast_bvh_manager.cpp:102`, `:115` both pass `false`).
//
// Fields: `name|min.xyz|max.xyz|visited|data...`.
static void compound(const char* name, const btCompoundShape* c, const btTransform& t)
{
	btVector3 mn, mx;
	c->getAabb(t, mn, mx);
	printf("%s|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g", name, (double)mn[0], (double)mn[1], (double)mn[2],
	       (double)mx[0], (double)mx[1], (double)mx[2]);

	RecordingCollide collide;
	const btDbvt* tree = c->getDynamicAabbTree();
	if (tree)
	{
		btNodeStack stack;
		const btDbvtVolume all =
		    btDbvtVolume::FromMM(btVector3(-1e6f, -1e6f, -1e6f), btVector3(1e6f, 1e6f, 1e6f));
		tree->collideTVNoStackAlloc(tree->m_root, all, stack, collide);
	}

	printf("|%d", collide.count);
	for (int i = 0; i < collide.count; ++i) printf("|%d", collide.seen[i]);
	printf("\n");
}

// `TestAabbAgainstAabb2` on its own, at each boundary. Both compound
// algorithms cull child pairs with it, and a cull is only visible in a
// traversal row as a pair that did not appear -- indistinguishable there from
// a pair the tree never reached. These rows separate the two.
static void aabbtest(const char* name, const btVector3& min1, const btVector3& max1,
                     const btVector3& min2, const btVector3& max2)
{
	printf("aabb_%s|%d\n", name, TestAabbAgainstAabb2(min1, max1, min2, max2) ? 1 : 0);
}

// A `btManifoldResult` that traces what the compound traversals do to it.
//
// `setShapeIdentifiersA`/`B` are virtual (`btManifoldResult.h:90-99`, pure in
// `btDiscreteCollisionDetectorInterface::Result`), and both compound
// algorithms call them immediately before dispatching a child pair -- after
// the tree cull and after `TestAabbAgainstAabb2`. Tracing them is therefore
// the dispatch order itself, with the real child indices, rather than the
// pre-cull leaf order the two `gCompound*ChildShapePairCallback` hooks would
// give. Which of A and B a compound-vs-convex row sets is the swap detection's
// answer, so it is recorded rather than normalised away.
//
// `addContactPoint` is replaced outright, exactly as MoveIt's bridge does
// (`bullet_utils.hpp:571-630`): nothing reaches the manifold's point cache, so
// `getNumContacts()` is zero at every level and the refresh loops both
// algorithms run over their child caches stay no-ops.
struct TraceResult : public btManifoldResult
{
	char dispatches[1024];
	int num_dispatches;
	char contacts[1024];
	int num_contacts;
	// Every contact's geometry, not just the last: a child whose transform is
	// wrong changes only the contacts *that child* reported, and with a single
	// slot the last child written vouches for all of them.
	enum { MAX_CONTACTS = 64 };
	btVector3 normal[MAX_CONTACTS];
	btVector3 point[MAX_CONTACTS];
	btScalar depth[MAX_CONTACTS];

	TraceResult(const btCollisionObjectWrapper* a, const btCollisionObjectWrapper* b)
		: btManifoldResult(a, b), num_dispatches(0), num_contacts(0)
	{
		dispatches[0] = 0;
		contacts[0] = 0;
	}

	static void note(char* buf, size_t cap, const char* tag, int a, int b)
	{
		size_t n = strlen(buf);
		snprintf(buf + n, cap - n, "|%s%d:%d", tag, a, b);
	}

	void setShapeIdentifiersA(int partId0, int index0) override
	{
		btManifoldResult::setShapeIdentifiersA(partId0, index0);
		note(dispatches, sizeof(dispatches), "A", partId0, index0);
		++num_dispatches;
	}

	void setShapeIdentifiersB(int partId1, int index1) override
	{
		btManifoldResult::setShapeIdentifiersB(partId1, index1);
		note(dispatches, sizeof(dispatches), "B", partId1, index1);
		++num_dispatches;
	}

	void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld,
	                     btScalar d) override
	{
		note(contacts, sizeof(contacts), "", m_index0, m_index1);
		if (num_contacts >= MAX_CONTACTS)
		{
			printf("ccoverflow|%d\n", num_contacts);
			abort();
		}
		normal[num_contacts] = normalOnBInWorld;
		point[num_contacts] = pointInWorld;
		depth[num_contacts] = d;
		++num_contacts;
	}
};

// `btCompoundCollisionAlgorithm::processCollision` and
// `btCompoundCompoundCollisionAlgorithm::processCollision`, driven exactly as
// `cc` above drives the convex-convex one -- same dispatcher, same cleared
// relative-breaking flag, same `BT_CLOSEST_POINT_ALGORITHMS` at the top level.
// Which table the *children* are then looked up in is not this call's choice
// but the threshold's, so a row with `closestPointDistanceThreshold > 0` is
// the only thing that exercises the fork at all.
//
// Two rows per case plus one per contact:
//   `ccdispatch_<name>|n|A|B<partId>:<index>...`  -- the dispatch trace
//   `cccontact_<name>|n|<index0>:<index1>...`     -- the identifiers each
//                                                    contact was tagged with
//   `ccpoint_<name>_<k>|normal xyz|point xyz|depth` -- contact k, which is what
//                                                    pins the composed child
//                                                    world transform of the
//                                                    child that reported it
static void compound_algo(const char* name, btCollisionShape* a, const btTransform& ta,
                          btCollisionShape* b, const btTransform& tb,
                          btScalar closestPointDistanceThreshold)
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

	TraceResult result(&wrap_a, &wrap_b);
	result.m_closestPointDistanceThreshold = closestPointDistanceThreshold;

	btCollisionAlgorithm* algo =
	    dispatcher.findAlgorithm(&wrap_a, &wrap_b, nullptr, BT_CLOSEST_POINT_ALGORITHMS);

	btDispatcherInfo info;
	algo->processCollision(&wrap_a, &wrap_b, info, &result);

	printf("ccdispatch_%s|%d%s\n", name, result.num_dispatches, result.dispatches);
	printf("cccontact_%s|%d%s\n", name, result.num_contacts, result.contacts);
	for (int k = 0; k < result.num_contacts; ++k)
		printf("ccpoint_%s_%d|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g\n", name, k,
		       (double)result.normal[k][0], (double)result.normal[k][1],
		       (double)result.normal[k][2], (double)result.point[k][0],
		       (double)result.point[k][1], (double)result.point[k][2], (double)result.depth[k]);

	algo->~btCollisionAlgorithm();
	dispatcher.freeCollisionAlgorithm(algo);
}

// `btDefaultCollisionConfiguration`'s two create-func tables, as a matrix.
//
// The tables are chains of `if`s over the proxy types
// (`btDefaultCollisionConfiguration.cpp:193-266` for closest points,
// `:267-343` for contact points), and every entry returns one of thirteen
// create-func members. Those members are `protected`, so a row cannot print
// the name directly; instead ANCHORS below fixes one pair per create-func by
// reading the table top-down, and every other pair is then labelled by
// pointer identity against those anchors. A pointer that matches no anchor
// prints `??`, which is a failure of this labelling and not a Bullet result.
struct Anchor
{
	int t0;
	int t1;
	const char* code;
};

static const Anchor ANCHORS[] = {
    {SPHERE_SHAPE_PROXYTYPE, SPHERE_SHAPE_PROXYTYPE, "ss"},
    {SPHERE_SHAPE_PROXYTYPE, TRIANGLE_SHAPE_PROXYTYPE, "st"},
    {TRIANGLE_SHAPE_PROXYTYPE, SPHERE_SHAPE_PROXYTYPE, "ts"},
    {BOX_SHAPE_PROXYTYPE, STATIC_PLANE_PROXYTYPE, "cp"},
    {STATIC_PLANE_PROXYTYPE, BOX_SHAPE_PROXYTYPE, "pc"},
    {BOX_SHAPE_PROXYTYPE, SPHERE_SHAPE_PROXYTYPE, "cx"},
    {BOX_SHAPE_PROXYTYPE, TRIANGLE_MESH_SHAPE_PROXYTYPE, "cv"},
    {TRIANGLE_MESH_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE, "vc"},
    {COMPOUND_SHAPE_PROXYTYPE, COMPOUND_SHAPE_PROXYTYPE, "kk"},
    {COMPOUND_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE, "kx"},
    {BOX_SHAPE_PROXYTYPE, COMPOUND_SHAPE_PROXYTYPE, "xk"},
    {TRIANGLE_MESH_SHAPE_PROXYTYPE, TRIANGLE_MESH_SHAPE_PROXYTYPE, "--"},
};

// The box-box entry exists only in the contact-point table, so its anchor is
// resolved there and reused as a label when scanning the closest-points one.
static const Anchor BOX_BOX_ANCHOR = {BOX_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE, "bb"};

static const int DISPATCH_TYPES[] = {
    BOX_SHAPE_PROXYTYPE,       TRIANGLE_SHAPE_PROXYTYPE, CONVEX_HULL_SHAPE_PROXYTYPE,
    SPHERE_SHAPE_PROXYTYPE,    CONE_SHAPE_PROXYTYPE,     CYLINDER_SHAPE_PROXYTYPE,
    CUSTOM_CONVEX_SHAPE_TYPE,  TRIANGLE_MESH_SHAPE_PROXYTYPE, STATIC_PLANE_PROXYTYPE,
    EMPTY_SHAPE_PROXYTYPE,     COMPOUND_SHAPE_PROXYTYPE};
static const int NUM_DISPATCH_TYPES = 11;

static void dispatch_table(btDefaultCollisionConfiguration& config, bool closest, const char* tag)
{
	btCollisionAlgorithmCreateFunc* known[16];
	const char* codes[16];
	int num_known = 0;

	for (int i = 0; i < 12; ++i)
	{
		known[num_known] = closest ? config.getClosestPointsAlgorithmCreateFunc(ANCHORS[i].t0, ANCHORS[i].t1)
		                           : config.getCollisionAlgorithmCreateFunc(ANCHORS[i].t0, ANCHORS[i].t1);
		codes[num_known] = ANCHORS[i].code;
		++num_known;
	}
	known[num_known] = config.getCollisionAlgorithmCreateFunc(BOX_BOX_ANCHOR.t0, BOX_BOX_ANCHOR.t1);
	codes[num_known] = BOX_BOX_ANCHOR.code;
	++num_known;

	for (int a = 0; a < NUM_DISPATCH_TYPES; ++a)
	{
		printf("dispatch_%s_%d", tag, DISPATCH_TYPES[a]);
		for (int b = 0; b < NUM_DISPATCH_TYPES; ++b)
		{
			btCollisionAlgorithmCreateFunc* got =
			    closest ? config.getClosestPointsAlgorithmCreateFunc(DISPATCH_TYPES[a], DISPATCH_TYPES[b])
			            : config.getCollisionAlgorithmCreateFunc(DISPATCH_TYPES[a], DISPATCH_TYPES[b]);
			const char* code = "??";
			for (int k = 0; k < num_known; ++k)
			{
				if (known[k] == got)
				{
					code = codes[k];
					break;
				}
			}
			printf("|%s", code);
		}
		printf("\n");
	}
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
	// side of every CCD pair.
	//
	// Two boxes would enter that conjunction, but not the SAT/clipping
	// arithmetic behind it. Nothing calls `initializePolyhedralFeatures` in
	// this build: every call site in `BulletCollision` is commented out bar
	// `btConvexConvexAlgorithm.cpp:565`, which needs both
	// `dispatchInfo.m_enableSatConvex` -- false by default -- and a
	// `TRIANGLE_SHAPE_PROXYTYPE` on side B. So `getConvexPolyhedron()` is null
	// on both sides, the inner `if` and its `else` both fail, and the branch
	// falls through to the same `gjkPairDetector.getClosestPoints` these rows
	// take.
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

	// `btCompoundShape`. Every row uses the same three children in the same
	// order, so the differences between rows are only the query transform,
	// the margin, and the one `updateChildTransform` some of them perform.
	{
		btCompoundShape three(true, 3);
		three.addChildShape(at(0.f, 0.f, 0.f), &unit_box);
		three.addChildShape(at(2.f, 0.f, 0.f), &sphere);
		three.addChildShape(at(0.f, 3.f, 0.f), &cyl);

		compound("comp_aabb_id", &three, id);
		compound("comp_aabb_rot60", &three, rot60_at(1.f, 2.f, 3.f));

		three.setMargin(0.25f);
		compound("comp_aabb_margin_rot60", &three, rot60_at(1.f, 2.f, 3.f));
		three.setMargin(0.f);

		// `false` is what MoveIt passes: the tree moves, the local AABB does
		// not. The following row is the same edit with `true`, so the pair
		// isolates the recalculation from the tree update.
		three.updateChildTransform(1, at(5.f, 0.f, 0.f), false);
		compound("comp_update_no_recalc", &three, id);
		three.recalculateLocalAabb();
		compound("comp_update_recalc", &three, id);

		btCompoundShape empty(true, 0);
		compound("comp_aabb_empty", &empty, rot60_at(1.f, 2.f, 3.f));

		btCompoundShape no_tree(false, 3);
		no_tree.addChildShape(at(0.f, 0.f, 0.f), &unit_box);
		no_tree.addChildShape(at(2.f, 0.f, 0.f), &sphere);
		no_tree.addChildShape(at(0.f, 3.f, 0.f), &cyl);
		compound("comp_no_tree", &no_tree, id);

		// Four children in a line, so the leaf order is the one `dbvt_line4`
		// pins -- reached here through `addChildShape` rather than through
		// `insert` directly.
		btCompoundShape line(true, 4);
		for (int i = 0; i < 4; ++i) line.addChildShape(at(btScalar(i) * 2.f, 0.f, 0.f), &unit_box);
		compound("comp_line4", &line, id);
	}

	// `TestAabbAgainstAabb2` at each boundary. Fields: `aabb_<name>|overlap`.
	{
		const btVector3 lo(0.f, 0.f, 0.f), hi(1.f, 1.f, 1.f);
		aabbtest("overlap", lo, hi, btVector3(0.5f, 0.5f, 0.5f), btVector3(1.5f, 1.5f, 1.5f));
		aabbtest("touch_x", lo, hi, btVector3(1.f, 0.f, 0.f), btVector3(2.f, 1.f, 1.f));
		aabbtest("gap_x", lo, hi, btVector3(1.0001f, 0.f, 0.f), btVector3(2.f, 1.f, 1.f));
		aabbtest("touch_y", lo, hi, btVector3(0.f, 1.f, 0.f), btVector3(1.f, 2.f, 1.f));
		aabbtest("gap_y", lo, hi, btVector3(0.f, 1.0001f, 0.f), btVector3(1.f, 2.f, 1.f));
		aabbtest("touch_z", lo, hi, btVector3(0.f, 0.f, 1.f), btVector3(1.f, 1.f, 2.f));
		aabbtest("gap_z", lo, hi, btVector3(0.f, 0.f, 1.0001f), btVector3(1.f, 1.f, 2.f));
		aabbtest("touch_neg_x", lo, hi, btVector3(-1.f, 0.f, 0.f), btVector3(0.f, 1.f, 1.f));
		aabbtest("gap_neg_x", lo, hi, btVector3(-1.f, 0.f, 0.f), btVector3(-0.0001f, 1.f, 1.f));
		aabbtest("contained", lo, hi, btVector3(0.25f, 0.25f, 0.25f), btVector3(0.75f, .75f, .75f));
		aabbtest("degenerate", lo, hi, btVector3(1.f, 1.f, 1.f), btVector3(1.f, 1.f, 1.f));
		aabbtest("gap_in_one_axis_only", lo, hi, btVector3(0.5f, 0.5f, 1.0001f),
		         btVector3(1.5f, 1.5f, 2.f));
	}

	// The two compound algorithms, driven through the same dispatcher.
	//
	// `line3`'s three children have exactly touching AABBs and the query
	// shape sits over the middle one, so every child clears both culls and
	// the order in the trace is the tree's own. `sphere_line3` is that row
	// with the operands exchanged: the create-func table answers
	// `SwappedCreateFunc`, the swap detection then tags the children through
	// `B` instead of `A`, and the child dispatch still puts the *child*
	// first -- so the contact normal is reported against the other operand
	// than at the top level.
	//
	// Every child pair has a sphere, cylinder or compound on one side, which
	// is what a `CastHullShape` guarantees on the real continuous path: two
	// polyhedral shapes would reach `btConvexConvexAlgorithm`'s SAT branch
	// condition, and `line3_box*` below is the only row that does.
	//
	// `no_tree_*` are the same shapes with the dynamic tree switched off,
	// which is the only configuration where `TestAabbAgainstAabb2` is the
	// sole cull: with a tree, an identity compound transform makes the leaf
	// volume and the child's world AABB the same box, and the per-child test
	// can only re-reject what the tree already rejected.
	{
		btCompoundShape line3(true, 3);
		for (int i = 0; i < 3; ++i) line3.addChildShape(at(btScalar(i), 0.f, 0.f), &unit_box);

		btCompoundShape no_tree3(false, 3);
		for (int i = 0; i < 3; ++i) no_tree3.addChildShape(at(btScalar(i), 0.f, 0.f), &unit_box);

		// The `gContactBreakingThreshold` window: `margin_box`'s AABB already
		// carries its 0.04 margin and `sphere`'s carries its whole radius,
		// but the GJK cut-off is both margins *plus* that 0.02. At 1.01 apart
		// the AABBs miss by 0.01 while the cores are 0.55 apart against a
		// 0.56 bound -- so this pair contacts if and only if the AABB cull is
		// skipped, which no tree row can show.
		btCompoundShape no_tree_pair(false, 2);
		no_tree_pair.addChildShape(at(0.f, 0.f, 0.f), &margin_box);
		no_tree_pair.addChildShape(at(3.f, 0.f, 0.f), &margin_box);

		btCompoundShape no_tree_cyl2(false, 2);
		no_tree_cyl2.addChildShape(at(0.f, 0.f, 0.f), &cyl);
		no_tree_cyl2.addChildShape(at(2.f, 0.f, 0.f), &cyl);

		btCompoundShape empty(true, 0);

		btCompoundShape inner(true, 2);
		inner.addChildShape(at(0.f, 0.f, 0.f), &unit_box);
		inner.addChildShape(at(1.f, 0.f, 0.f), &cyl);

		btCompoundShape nested(true, 2);
		nested.addChildShape(at(0.f, 0.f, 0.f), &unit_box);
		nested.addChildShape(at(1.5f, 0.f, 0.f), &inner);

		btCompoundShape pair2(true, 2);
		pair2.addChildShape(at(0.f, 0.f, 0.f), &cyl);
		pair2.addChildShape(at(1.f, 0.f, 0.f), &sphere);

		btCompoundShape sph3(true, 3);
		for (int i = 0; i < 3; ++i) sph3.addChildShape(at(btScalar(i), 0.f, 0.f), &sphere);

		// Four children at a different spacing, so this tree is not `line3`'s
		// shape translated: against a symmetric pair of trees `MycollideTT`'s
		// internal/internal push order is its own mirror image and the order it
		// produces cannot show which of the two middle pairs was pushed first.
		btCompoundShape sph4(true, 4);
		for (int i = 0; i < 4; ++i)
			sph4.addChildShape(at(btScalar(i) * 0.7f, 0.f, 0.f), &sphere);

		compound_algo("line3_sphere", &line3, id, &sphere, at(1.f, 0.f, 0.f), 0.f);
		compound_algo("sphere_line3", &sphere, at(1.f, 0.f, 0.f), &line3, id, 0.f);
		compound_algo("line3_sphere_off", &line3, id, &sphere, at(1.f, 0.9f, 0.3f), 0.f);
		compound_algo("line3_rot60_sphere", &line3, rot60_at(0.2f, 0.1f, 0.f), &sphere,
		              at(1.f, 0.f, 0.f), 0.f);
		compound_algo("line3_sphere_far", &line3, id, &sphere, at(9.f, 0.f, 0.f), 0.f);

		compound_algo("no_tree3_sphere", &no_tree3, id, &sphere, at(1.f, 0.f, 0.f), 0.f);
		compound_algo("no_tree_window", &no_tree_pair, id, &sphere, at(1.01f, 0.f, 0.f), 0.f);
		compound_algo("empty_sphere", &empty, id, &sphere, id, 0.f);

		// `m_closestPointDistanceThreshold > 0` -- the only setting under
		// which `extendAabb`, `extraExtends` and `thresholdVec` are not
		// additions of zero, and the only one that sends the child lookup to
		// the closest-points table instead of the contact-points one.
		compound_algo("line3_sphere_threshold", &line3, id, &sphere, at(1.f, 0.f, 0.f), 0.25f);
		compound_algo("no_tree_window_threshold", &no_tree_pair, id, &sphere, at(1.01f, 0.f, 0.f),
		              0.25f);
		compound_algo("line3_sphere_far_threshold", &line3, id, &sphere, at(3.9f, 0.f, 0.f), 1.f);

		// The one pair of rows where the two create-func tables disagree: two
		// boxes resolve to `btBoxBoxCollisionAlgorithm` in the contact-points
		// table and to `btConvexConvexAlgorithm` in the closest-points one,
		// and which table a child pair is looked up in is decided by the
		// threshold alone. The contact counts are the difference: four points
		// per child from the box-box detector, one from GJK.
		compound_algo("line3_box", &line3, id, &margin_box, at(1.f, 0.f, 0.f), 0.f);
		compound_algo("line3_box_threshold", &line3, id, &margin_box, at(1.f, 0.f, 0.f), 0.25f);

		// A compound as a compound's child: the outer identifiers are
		// overwritten by the inner ones before any contact is reported.
		compound_algo("nested_sphere", &nested, id, &sphere, at(1.75f, 0.f, 0.f), 0.f);
		compound_algo("sphere_nested", &sphere, at(1.75f, 0.f, 0.f), &nested, id, 0.f);

		// Both sides compound -- `MycollideTT`, which is a different
		// traversal from `collideTVNoStackAlloc` and has its own push order.
		compound_algo("line3_pair2", &line3, id, &pair2, at(1.f, 0.f, 0.f), 0.f);
		compound_algo("line3_pair2_rot60", &line3, id, &pair2, rot60_at(1.f, 0.1f, 0.f), 0.f);
		compound_algo("line3_pair2_threshold", &line3, id, &pair2, at(1.f, 0.f, 0.f), 0.25f);
		compound_algo("line3_pair2_far", &line3, id, &pair2, at(9.f, 0.f, 0.f), 0.f);

		// Three children on each side, which is what puts `MycollideTT` in its
		// internal/internal arm at the root and in both internal/leaf arms
		// below it with enough surviving pairs for the push order to show.
		compound_algo("line3_sph3", &line3, id, &sph3, at(1.f, 0.f, 0.f), 0.f);
		compound_algo("line3_rot60_sph3", &line3, rot60_at(0.2f, 0.1f, 0.f), &sph3,
		              at(1.f, 0.f, 0.f), 0.f);

		// `MyIntersect` re-boxes tree 1's leaf volume through `xform`, so a
		// rotated compound 1 inflates each sphere's local cube from +/-0.5 to
		// +/-0.8333 for the tree test while `Process` still measures the
		// sphere's own +/-0.5 world box. Here child pair (2,2) is 1.13 apart
		// along x -- inside the inflated bound and outside the true one -- so it
		// is the one pair in the whole fixture that reaches `Process` and is
		// rejected by its `TestAabbAgainstAabb2`.
		compound_algo("line3_sph3_rot60", &line3, id, &sph3, rot60_at(1.8f, -0.3f, 0.f), 0.f);

		// The same pair moved to 1.20 apart with the threshold at 0.15:
		// `thresholdVec` grows box 0 only, which leaves the pair separated by
		// 0.05, and growing both sides would close it.
		compound_algo("line3_sph3_rot60_threshold", &line3, id, &sph3,
		              rot60_at(1.87f, -0.3f, 0.f), 0.15f);

		// Three against four at 0.7 spacing, offset so that seven of the twelve
		// pairs survive and no pair sits on an exact AABB tie: two trees of
		// different shape, which is what makes the internal/internal and
		// leaf/internal push orders observable at all.
		compound_algo("line3_sph4", &line3, id, &sph4, at(0.35f, 0.f, 0.f), 0.f);

		// One side without a tree: `btCompoundCompoundCollisionAlgorithm`
		// hands the whole query back to `btCompoundCollisionAlgorithm`, whose
		// `m_isSwapped` is false either way -- so which operand is treated as
		// "the compound" is decided by position, not by which one still has a
		// tree.
		compound_algo("line3_notree_cyl2", &line3, id, &no_tree_cyl2, at(1.f, 0.f, 0.f), 0.f);
		compound_algo("notree_cyl2_line3", &no_tree_cyl2, id, &line3, at(1.f, 0.f, 0.f), 0.f);
	}

	// The two create-func tables. Fields:
	// `dispatch_<table>_<type0>|code(type0, t)` for each `t` in the type list.
	{
		btDefaultCollisionConfiguration dispatch_config;
		dispatch_table(dispatch_config, true, "closest");
		dispatch_table(dispatch_config, false, "contact");
	}

	// `BroadphaseNativeTypes`. Fields: `name|value|isConvex|isConcave|isCompound`.
	// Every entry the port carries a constant for, plus the four markers the
	// predicates compare against, plus the neighbours on each side of a
	// marker so an off-by-one in the port's numbering cannot sit between two
	// emitted rows.
	proxytype("BOX_SHAPE", BOX_SHAPE_PROXYTYPE);
	proxytype("TRIANGLE_SHAPE", TRIANGLE_SHAPE_PROXYTYPE);
	proxytype("CONVEX_HULL_SHAPE", CONVEX_HULL_SHAPE_PROXYTYPE);
	proxytype("CUSTOM_POLYHEDRAL_SHAPE", CUSTOM_POLYHEDRAL_SHAPE_TYPE);
	proxytype("IMPLICIT_CONVEX_SHAPES_START_HERE", IMPLICIT_CONVEX_SHAPES_START_HERE);
	proxytype("SPHERE_SHAPE", SPHERE_SHAPE_PROXYTYPE);
	proxytype("CAPSULE_SHAPE", CAPSULE_SHAPE_PROXYTYPE);
	proxytype("CONE_SHAPE", CONE_SHAPE_PROXYTYPE);
	proxytype("CYLINDER_SHAPE", CYLINDER_SHAPE_PROXYTYPE);
	proxytype("CONVEX_2D_SHAPE", CONVEX_2D_SHAPE_PROXYTYPE);
	proxytype("CUSTOM_CONVEX_SHAPE", CUSTOM_CONVEX_SHAPE_TYPE);
	proxytype("CONCAVE_SHAPES_START_HERE", CONCAVE_SHAPES_START_HERE);
	proxytype("TRIANGLE_MESH_SHAPE", TRIANGLE_MESH_SHAPE_PROXYTYPE);
	proxytype("EMPTY_SHAPE", EMPTY_SHAPE_PROXYTYPE);
	proxytype("STATIC_PLANE", STATIC_PLANE_PROXYTYPE);
	proxytype("CUSTOM_CONCAVE_SHAPE", CUSTOM_CONCAVE_SHAPE_TYPE);
	proxytype("CONCAVE_SHAPES_END_HERE", CONCAVE_SHAPES_END_HERE);
	proxytype("COMPOUND_SHAPE", COMPOUND_SHAPE_PROXYTYPE);
	proxytype("SOFTBODY_SHAPE", SOFTBODY_SHAPE_PROXYTYPE);
	proxytype("INVALID_SHAPE", INVALID_SHAPE_PROXYTYPE);

	// What each built shape reports. Fields: `name|shapeType`.
	shapetype("unit_box", &unit_box);
	shapetype("sphere", &sphere);
	shapetype("cyl", &cyl);
	shapetype("cone", &cone);
	shapetype("hull", &hull);

	// Which shapes `getAverageSupport` treats as polyhedral, and the vertices
	// it then averages. `margin_box` is here as well as `unit_box` because
	// `getVertex` uses the half extents *with* margin: it is the only row that
	// separates "the corners of the box" from "the corners the support
	// function would return".
	polyverts("unit_box", &unit_box);
	polyverts("flat_box", &flat_box);
	polyverts("margin_box", &margin_box);
	polyverts("sphere", &sphere);
	polyverts("cyl", &cyl);
	polyverts("cone", &cone);
	polyverts("hull", &hull);

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
