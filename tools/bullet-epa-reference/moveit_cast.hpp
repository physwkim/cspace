// Copyright (c) 2017, Southwest Research Institute
// Copyright (c) 2013, John Schulman
// SPDX-License-Identifier: BSD-2-Clause
//
// Transcribed verbatim from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection_bullet/include/moveit/collision_detection_bullet/bullet_integration/bullet_utils.hpp
// -- the constants at :51-54, `CastHullShape` at :240-334 and
// `getAverageSupport` at :344-383, with the doc comments dropped and nothing
// else changed.
//
// Why transcribed rather than included: the upstream header pulls in rclcpp,
// geometric_shapes and the rest of moveit_core, so `#include`-ing it would
// make this probe need a ROS 2 installation. `tools/moveit-oracle` is the
// instrument that does have one, and its `bullet_cast_pair` op reports the
// end-to-end swept answer -- but end-to-end is exactly what cannot localize a
// disagreement to the support function. These two functions depend on nothing
// but bullet3, so copying them here buys a unit-level pin at the cost of a
// copy that this file's own header attributes.
//
// The copy is what makes the fixture worth anything: rows derived from a
// hand-reading of the upstream would be asserting a reading, not the code.

#ifndef CSPACE_BULLET_EPA_REFERENCE_MOVEIT_CAST_HPP
#define CSPACE_BULLET_EPA_REFERENCE_MOVEIT_CAST_HPP

#include <stdexcept>

#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"

#define METERS

const btScalar BULLET_MARGIN = 0.0f;
const btScalar BULLET_SUPPORT_FUNC_TOLERANCE = 0.01f METERS;
const btScalar BULLET_LENGTH_TOLERANCE = 0.001f METERS;
const btScalar BULLET_EPSILON = 1e-3f;

struct CastHullShape : public btConvexShape
{
public:
	btConvexShape* m_shape;

	btTransform shape_transform;

	CastHullShape(btConvexShape* shape, const btTransform& t01) : m_shape(shape), shape_transform(t01)
	{
		m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;
	}

	void updateCastTransform(const btTransform& cast_transform)
	{
		shape_transform = cast_transform;
	}

	btVector3 localGetSupportingVertex(const btVector3& vec) const override
	{
		btVector3 support_vector_0 = m_shape->localGetSupportingVertex(vec);
		btVector3 support_vector_1 =
		    shape_transform * m_shape->localGetSupportingVertex(vec * shape_transform.getBasis());
		return (vec.dot(support_vector_0) > vec.dot(support_vector_1)) ? support_vector_0 : support_vector_1;
	}

	void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* /*vectors*/,
	                                                       btVector3* /*supportVerticesOut*/,
	                                                       int /*numVectors*/) const override
	{
		throw std::runtime_error("not implemented");
	}

	void getAabb(const btTransform& transform_world, btVector3& aabbMin, btVector3& aabbMax) const override
	{
		m_shape->getAabb(transform_world, aabbMin, aabbMax);
		btVector3 min1, max1;
		m_shape->getAabb(transform_world * shape_transform, min1, max1);
		aabbMin.setMin(min1);
		aabbMax.setMax(max1);
	}

	void getAabbSlow(const btTransform& /*t*/, btVector3& /*aabbMin*/, btVector3& /*aabbMax*/) const override
	{
		throw std::runtime_error("shouldn't happen");
	}

	void setLocalScaling(const btVector3& /*scaling*/) override
	{
	}

	const btVector3& getLocalScaling() const override
	{
		static btVector3 out(1, 1, 1);
		return out;
	}

	void setMargin(btScalar /*margin*/) override
	{
	}

	btScalar getMargin() const override
	{
		return 0;
	}

	int getNumPreferredPenetrationDirections() const override
	{
		return 0;
	}

	void getPreferredPenetrationDirection(int /*index*/, btVector3& /*penetrationVector*/) const override
	{
		throw std::runtime_error("not implemented");
	}

	void calculateLocalInertia(btScalar /*mass*/, btVector3& /*inertia*/) const override
	{
		throw std::runtime_error("not implemented");
	}

	const char* getName() const override
	{
		return "CastHull";
	}

	btVector3 localGetSupportingVertexWithoutMargin(const btVector3& v) const override
	{
		return localGetSupportingVertex(v);
	}
};

inline void getAverageSupport(const btConvexShape* shape, const btVector3& localNormal, float& outsupport,
                              btVector3& outpt)
{
	btVector3 pt_sum(0, 0, 0);
	float pt_count = 0;
	float max_support = -1000;

	const btPolyhedralConvexShape* pshape = dynamic_cast<const btPolyhedralConvexShape*>(shape);
	if (pshape)
	{
		int n_pts = pshape->getNumVertices();

		for (int i = 0; i < n_pts; ++i)
		{
			btVector3 pt;
			pshape->getVertex(i, pt);

			float sup = pt.dot(localNormal);
			if (sup > max_support + BULLET_EPSILON)
			{
				pt_count = 1;
				pt_sum = pt;
				max_support = sup;
			}
			else if (sup < max_support - BULLET_EPSILON) {}
			else
			{
				pt_count += 1;
				pt_sum += pt;
			}
		}
		outsupport = max_support;
		outpt = pt_sum / pt_count;
	}
	else
	{
		outpt = shape->localGetSupportingVertexWithoutMargin(localNormal);
		outsupport = localNormal.dot(outpt);
	}
}

// The tail of `addCastSingleResult` (`bullet_utils.hpp:451-514`), from the
// `cast_shape_is_first` derivation to the last `percent_interpolation`
// assignment, with three things removed and nothing else changed:
//
//   - the two `CollisionObjectWrapper` derefs the flag comes from, replaced by
//     the flag itself as an argument;
//   - the `col->` writes into `ContactTestData`'s stored contact -- the two
//     `std::swap`s, the `col->normal *= -1`, and the dead
//     `contact.pos = ...m_positionWorldOnB` at `:463`, which assigns to the
//     *local* `contact` that `processResult` has already copied out and so
//     never reaches the result;
//   - the `assert` at `:451`.
//
// What is left is the arithmetic: `normal_world_from_cast`, the two world
// transforms, the two `getAverageSupport` calls, and the three-way choice of
// `percent_interpolation`. `localsup0`/`localsup1` are kept even though nothing
// upstream reads them -- deleting them would be a judgement about which of
// `getAverageSupport`'s two outputs matters, and the point of a transcription
// is not to make judgements.
inline float castPercentInterpolation(const btManifoldPoint& cp, bool cast_shape_is_first, const CastHullShape* shape,
                                      const btTransform& first_world_transform)
{
	btVector3 normal_world_from_cast = (cast_shape_is_first ? -1 : 1) * cp.m_normalWorldOnB;

	btTransform tf_world0, tf_world1;
	tf_world0 = first_world_transform;
	tf_world1 = first_world_transform * shape->shape_transform;

	btVector3 normal_local0 = normal_world_from_cast * tf_world0.getBasis();
	btVector3 normal_local1 = normal_world_from_cast * tf_world1.getBasis();

	btVector3 pt_local0;
	float localsup0;
	getAverageSupport(shape->m_shape, normal_local0, localsup0, pt_local0);
	btVector3 pt_world0 = tf_world0 * pt_local0;
	btVector3 pt_local1;
	float localsup1;
	getAverageSupport(shape->m_shape, normal_local1, localsup1, pt_local1);
	btVector3 pt_world1 = tf_world1 * pt_local1;

	float sup0 = normal_world_from_cast.dot(pt_world0);
	float sup1 = normal_world_from_cast.dot(pt_world1);

	if (sup0 - sup1 > BULLET_SUPPORT_FUNC_TOLERANCE)
	{
		return 0;
	}
	else if (sup1 - sup0 > BULLET_SUPPORT_FUNC_TOLERANCE)
	{
		return 1;
	}
	else
	{
		const btVector3& pt_on_cast = cast_shape_is_first ? cp.m_positionWorldOnA : cp.m_positionWorldOnB;
		float l0c = (pt_on_cast - pt_world0).length();
		float l1c = (pt_on_cast - pt_world1).length();

		if (l0c + l1c < BULLET_LENGTH_TOLERANCE)
		{
			return .5;
		}
		else
		{
			return static_cast<float>(l0c / (l0c + l1c));
		}
	}
}

#endif  // CSPACE_BULLET_EPA_REFERENCE_MOVEIT_CAST_HPP
