// Direct narrowphase probe of the fcl the oracle image ships (0.7.0-3build2).
// Each pair is placed in EXACT tangency along z: the lower shape's top face and
// the upper shape's bottom face are both at z = 0, using only values that are
// exact in binary floating point.
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/cone.h"
#include "fcl/geometry/shape/capsule.h"
#include "fcl/geometry/shape/ellipsoid.h"
#include "fcl/geometry/shape/convex.h"

using S = double;

struct Entry {
  std::string name;
  std::shared_ptr<fcl::CollisionGeometry<S>> geom;
  S half_z;  // distance from the shape's own origin to its lowest/highest point
};

static std::shared_ptr<fcl::Convex<S>> unitCubeConvex() {
  auto verts = std::make_shared<std::vector<fcl::Vector3<S>>>();
  for (int sx : {-1, 1}) for (int sy : {-1, 1}) for (int sz : {-1, 1})
    verts->push_back(fcl::Vector3<S>(sx * 0.5, sy * 0.5, sz * 0.5));
  // 6 quad faces, each as (count, i0, i1, i2, i3); index = 4*ix + 2*iy + iz
  auto faces = std::make_shared<std::vector<int>>();
  auto quad = [&](int a, int b, int c, int d) {
    faces->push_back(4); faces->push_back(a); faces->push_back(b);
    faces->push_back(c); faces->push_back(d);
  };
  quad(0,1,3,2); quad(4,6,7,5); quad(0,4,5,1);
  quad(2,3,7,6); quad(0,2,6,4); quad(1,5,7,3);
  return std::make_shared<fcl::Convex<S>>(verts, 6, faces, true);
}

int main() {
  std::vector<Entry> shapes = {
    {"box",       std::make_shared<fcl::Box<S>>(1.0, 1.0, 1.0),          0.5},
    {"sphere",    std::make_shared<fcl::Sphere<S>>(0.5),                 0.5},
    {"cylinder",  std::make_shared<fcl::Cylinder<S>>(0.5, 1.0),          0.5},
    {"cone",      std::make_shared<fcl::Cone<S>>(0.5, 1.0),              0.5},
    {"capsule",   std::make_shared<fcl::Capsule<S>>(0.5, 1.0),           1.0},
    {"ellipsoid", std::make_shared<fcl::Ellipsoid<S>>(0.5, 0.5, 0.5),    0.5},
    {"convex",    unitCubeConvex(),                                      0.5},
  };

  printf("upper,lower,delta,collision,num_contacts,depth\n");
  for (const auto& up : shapes) {
    for (const auto& lo : shapes) {
      for (S delta : {1e-9, 0.0, -1e-9}) {
        fcl::Transform3<S> tf_lo = fcl::Transform3<S>::Identity();
        tf_lo.translation() = fcl::Vector3<S>(0, 0, -lo.half_z);
        fcl::Transform3<S> tf_up = fcl::Transform3<S>::Identity();
        tf_up.translation() = fcl::Vector3<S>(0, 0, up.half_z + delta);

        fcl::CollisionObject<S> o_up(up.geom, tf_up);
        fcl::CollisionObject<S> o_lo(lo.geom, tf_lo);
        fcl::CollisionRequest<S> req(200, true);
        fcl::CollisionResult<S> res;
        fcl::collide(&o_up, &o_lo, req, res);
        S depth = res.numContacts() > 0 ? res.getContact(0).penetration_depth : 0.0 / 0.0;
        printf("%s,%s,%+.0e,%d,%zu,%.17g\n", up.name.c_str(), lo.name.c_str(), delta,
               res.isCollision() ? 1 : 0, res.numContacts(), depth);
      }
    }
  }
  return 0;
}
