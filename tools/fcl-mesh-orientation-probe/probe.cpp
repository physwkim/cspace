// fcl-side counterpart of
// crates/moveit-collision/examples/mesh_orientation_probe.rs: the same 497
// systematic orientations (7 axes x 71 angles, 5-degree resolution from 5 to
// 355 degrees), the same 5 other kinds, the same 2 argument-order roles, the
// same exact-zero-gap-by-construction geometry, run against
// fcl::BVHModel<fcl::OBBRSSd> instead of parry's query::contact -- so the
// two CSVs join row-for-row into a confusion matrix.
//
// Construction: for a rotation (axis, angle), the mesh's own 8 vertices
// (bit-for-bit crates/moveit-collision/tests/exact_tangency_is_decided_per_shape_pair.rs's
// unit_cube_mesh) are rotated by an Eigen::AngleAxis<S>, and the mesh is
// translated so its own extremal rotated vertex -- the lowest for
// `mesh=upper/attached`, the highest for `mesh=lower/world` -- lands exactly
// on TOUCH = (5, 0, 0). The other (unrotated) kind is placed with its own
// HALF-extent touching feature (a face centre, a pole, a disc centre -- all
// coincide with TOUCH for a HALF-extent shape centred HALF away) on the same
// point from the opposite side. This is the identical algorithm the Rust
// probe uses, not a restatement -- see that file's own module doc for why
// the construction is exact for any rotation. It is not bit-identical
// (Eigen's AngleAxis and nalgebra's UnitQuaternion are different
// implementations of the same standard formula), which is named rather than
// elided in this tool's own README.
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/cone.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"

using S = double;
using Vec3 = fcl::Vector3<S>;
using MeshS = fcl::BVHModel<fcl::OBBRSS<S>>;

constexpr S HALF = 0.5;
const Vec3 TOUCH(5.0, 0.0, 0.0);

// Offsets tried, smallest magnitude first, once a delta == 0.0 case comes
// back false -- the smallest one that flips the result to true is reported
// as this pose's own miss depth. Mirrors
// mesh_orientation_probe.rs::MISS_DEPTH_PROBES exactly, for a directly
// comparable number.
const std::vector<S> MISS_DEPTH_PROBES = {-1e-17, -1e-16, -1e-15, -1e-14, -1e-13,
                                            -1e-12, -1e-9,  -1e-6,  -1e-3,  -1e-1, -1.0};

static std::array<Vec3, 8> cubeVertices() {
  std::array<Vec3, 8> v;
  int i = 0;
  for (S z : {-HALF, HALF})
    for (S y : {-HALF, HALF})
      for (S x : {-HALF, HALF}) v[i++] = Vec3(x, y, z);
  return v;
}

static std::shared_ptr<MeshS> unitCubeMesh() {
  auto m = std::make_shared<MeshS>();
  auto verts = cubeVertices();
  std::vector<Vec3> pts(verts.begin(), verts.end());
  std::vector<fcl::Triangle> tris = {
      {0, 2, 1}, {1, 2, 3},  // z = -HALF
      {4, 5, 6}, {5, 7, 6},  // z = +HALF
      {0, 1, 4}, {1, 5, 4},  // y = -HALF
      {2, 6, 3}, {3, 6, 7},  // y = +HALF
      {0, 4, 2}, {2, 4, 6},  // x = -HALF
      {1, 3, 5}, {3, 7, 5},  // x = +HALF
  };
  m->beginModel();
  m->addSubModel(pts, tris);
  m->endModel();
  return m;
}

// Rotates the mesh by (axis, angleRad) and translates it so its own extremal
// rotated vertex (lowest for wantMin, highest otherwise) sits exactly on
// TOUCH, offset by delta along z. Negative delta moves toward more overlap --
// same sign convention as the Rust probe's rotated_mesh_pose.
static fcl::Transform3<S> meshPose(const Vec3& axisUnit, S angleRad, bool wantMin, S delta) {
  Eigen::AngleAxis<S> aa(angleRad, axisUnit);
  fcl::Matrix3<S> R = aa.toRotationMatrix();
  auto verts = cubeVertices();
  Vec3 best = R * verts[0];
  for (int i = 1; i < 8; i++) {
    Vec3 r = R * verts[i];
    if ((wantMin && r.z() < best.z()) || (!wantMin && r.z() > best.z())) best = r;
  }
  S sign = wantMin ? 1.0 : -1.0;
  fcl::Transform3<S> tf = fcl::Transform3<S>::Identity();
  tf.linear() = R;
  tf.translation() = Vec3(TOUCH.x() - best.x(), TOUCH.y() - best.y(), TOUCH.z() - best.z() + sign * delta);
  return tf;
}

// The fixed (unrotated) shape's own pose: its own HALF-extent touching
// feature lands on TOUCH from the side opposite the rotated mesh.
static fcl::Transform3<S> fixedPose(bool meshIsUp) {
  S z = meshIsUp ? (TOUCH.z() - HALF) : (TOUCH.z() + HALF);
  fcl::Transform3<S> tf = fcl::Transform3<S>::Identity();
  tf.translation() = Vec3(TOUCH.x(), TOUCH.y(), z);
  return tf;
}

static bool collides(const std::shared_ptr<fcl::CollisionGeometry<S>>& upGeom,
                      const fcl::Transform3<S>& upTf,
                      const std::shared_ptr<fcl::CollisionGeometry<S>>& loGeom,
                      const fcl::Transform3<S>& loTf) {
  fcl::CollisionObject<S> o_up(upGeom, upTf);
  fcl::CollisionObject<S> o_lo(loGeom, loTf);
  fcl::CollisionRequest<S> req(200, true);
  fcl::CollisionResult<S> res;
  fcl::collide(&o_up, &o_lo, req, res);
  return res.isCollision();
}

struct OtherKind {
  std::string name;  // matches the Rust probe's `{:?}` Debug spelling
  std::shared_ptr<fcl::CollisionGeometry<S>> geom;
};

struct Axis {
  std::string name;
  Vec3 v;
};

int main() {
  auto meshGeom = std::static_pointer_cast<fcl::CollisionGeometry<S>>(unitCubeMesh());

  std::vector<OtherKind> others = {
      {"Box", std::make_shared<fcl::Box<S>>(2.0 * HALF, 2.0 * HALF, 2.0 * HALF)},
      {"Sphere", std::make_shared<fcl::Sphere<S>>(HALF)},
      {"Cylinder", std::make_shared<fcl::Cylinder<S>>(HALF, 2.0 * HALF)},
      {"Cone", std::make_shared<fcl::Cone<S>>(HALF, 2.0 * HALF)},
      {"Mesh", meshGeom},
  };

  std::vector<Axis> axes = {
      {"x", Vec3(1, 0, 0).normalized()},       {"y", Vec3(0, 1, 0).normalized()},
      {"z", Vec3(0, 0, 1).normalized()},       {"xy", Vec3(1, 1, 0).normalized()},
      {"xz", Vec3(1, 0, 1).normalized()},      {"yz", Vec3(0, 1, 1).normalized()},
      {"xyz", Vec3(1, 1, 1).normalized()},
  };

  struct Role {
    std::string name;
    bool meshIsUp;
  };
  std::vector<Role> roles = {{"mesh=upper/attached", true}, {"mesh=lower/world", false}};

  long total = 0, falseCount = 0;
  for (const auto& other : others) {
    for (const auto& role : roles) {
      for (const auto& axis : axes) {
        for (int deg = 5; deg < 360; deg += 5) {
          total++;
          S angleRad = deg * M_PI / 180.0;
          fcl::Transform3<S> meshTf = meshPose(axis.v, angleRad, role.meshIsUp, 0.0);
          fcl::Transform3<S> otherTf = fixedPose(role.meshIsUp);

          bool hit;
          if (role.meshIsUp) {
            hit = collides(meshGeom, meshTf, other.geom, otherTf);
          } else {
            hit = collides(other.geom, otherTf, meshGeom, meshTf);
          }

          std::string depthStr = "NA";
          if (!hit) {
            falseCount++;
            for (S d : MISS_DEPTH_PROBES) {
              fcl::Transform3<S> meshTfD = meshPose(axis.v, angleRad, role.meshIsUp, d);
              bool flips = role.meshIsUp ? collides(meshGeom, meshTfD, other.geom, otherTf)
                                          : collides(other.geom, otherTf, meshGeom, meshTfD);
              if (flips) {
                char buf[32];
                snprintf(buf, sizeof(buf), "%g", d);
                depthStr = buf;
                break;
              }
            }
          }

          printf("CSV,%s,%s,axis=%s,angle=%ddeg,%s,%s\n", other.name.c_str(), role.name.c_str(),
                 axis.name.c_str(), deg, hit ? "true" : "false", depthStr.c_str());
        }
      }
    }
  }
  fprintf(stderr, "total=%ld false=%ld (%.1f%%)\n", total, falseCount, 100.0 * falseCount / total);
  return 0;
}
