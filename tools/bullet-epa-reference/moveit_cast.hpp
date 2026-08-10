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

#endif  // CSPACE_BULLET_EPA_REFERENCE_MOVEIT_CAST_HPP
