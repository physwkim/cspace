// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Prints `btGjkEpaSolver2::Penetration` and `::Distance` results for the
// shape pairs `crates/cspace-bullet/src/epa.rs`'s tests assert on, so those
// assertions carry Bullet's own answer rather than a hand-derived one.
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

#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"

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

	return 0;
}
