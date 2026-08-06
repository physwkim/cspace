// How far `fcl::distance`'s `cylinder x box` answer sits from the *exact*
// answer, and how much of that gap is decided by nothing but which shape is
// passed first.
//
// `tools/fcl-distance-tolerance-probe` established that fcl's own answer on a
// box/cylinder pair moves by up to `4.418002e-04` when only the GJK stopping
// threshold changes. That says the reference is imprecise; it cannot say which
// of two answers is right, because both of its columns are fcl's. This probe
// removes that limit by choosing a configuration whose correct separation
// distance is known without any narrowphase at all, and then asks fcl the same
// question twice with the operands swapped.
//
// The configuration is `tools/moveit-diff`'s own lowered-floor scene
// (`--floor-top-z -0.5`): the floor is `Box(4, 4, 0.1)` centred at
// `z = -0.55`, so its top face is the plane `z = -0.5`, and the moving shape is
// `fixtures/prbt.urdf`'s `prbt_flange` collision cylinder (`length 0.02`,
// `radius 0.0331`). While the cylinder sits above that face and its whole
// silhouette projects inside the 4x4 footprint -- both enforced by
// `face_is_nearest` below -- the nearest feature of the box is that face, i.e.
// the plane, and for a cylinder of half-length `h`, radius `r`, centre `c` and
// unit axis `a` against the plane `z = z0` the distance is exactly
//
//     c_z - z0 - h * |a_z| - r * sqrt(1 - a_z^2)
//
// (the lowest point of the cylinder is the lowest point of the lower rim: drop
// `h * |a_z|` along the axis, then `r * sin(angle to z)` around the rim). It is
// evaluated in `double` from the same pose fcl is handed, so the numbers are
// directly comparable.
//
// Pose 0 is not random: it is `prbt_flange`'s world pose at case 8148 of the
// seed-1 10,000-state prbt sweep, composed from the *oracle's own* `fk` answer
// for that state (`Op::Fk`, row-major 4x4) rather than from this port's
// kinematics, so the row that pins the divergence owes nothing to the port. The
// remaining poses are an xorshift64-drawn band around it.
//
// Emits one CSV row per pose: the closed form, then fcl at MoveIt's default
// `distance_tolerance` (`1e-6`) in each operand order, then the cylinder-first
// order at a tightened `1e-12`, then `1e-12` with `GST_INDEP`. MoveIt's own
// call is `d_box_first` -- see `tools/ci/verify-fcl-cylinder-box-distance.sh`
// for why that ordering is the one `distanceCallback` reaches.
#include <fcl/fcl.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>

using S = double;

namespace {

// `fixtures/prbt.urdf`'s `prbt_flange` <collision>, verbatim.
constexpr double kCylRadius = 0.0331;
constexpr double kCylLength = 0.02;
// `tools/moveit-diff`'s `collision_scene`, at `--floor-top-z -0.5`.
constexpr double kFloorSize = 4.0;
constexpr double kFloorThickness = 0.1;
constexpr double kFloorTopZ = -0.5;

std::uint64_t rng_state = 88172645463325252ULL;

// xorshift64, for the same reason `fcl-distance-tolerance-probe` uses one: a
// probe whose poses move between standard libraries cannot pin a number.
double next_unit()
{
  rng_state ^= rng_state << 13;
  rng_state ^= rng_state >> 7;
  rng_state ^= rng_state << 17;
  return static_cast<double>(rng_state >> 11) / 9007199254740992.0;
}

// Shoemake's uniform random rotation.
Eigen::Quaternion<S> next_rotation()
{
  const double u1 = next_unit();
  const double u2 = next_unit();
  const double u3 = next_unit();
  const double a = std::sqrt(1.0 - u1);
  const double b = std::sqrt(u1);
  return Eigen::Quaternion<S>(a * std::sin(2.0 * M_PI * u2), a * std::cos(2.0 * M_PI * u2),
                              b * std::sin(2.0 * M_PI * u3), b * std::cos(2.0 * M_PI * u3));
}

double distance_with(const std::shared_ptr<fcl::CollisionGeometry<S>>& g1,
                     const fcl::Transform3<S>& t1,
                     const std::shared_ptr<fcl::CollisionGeometry<S>>& g2,
                     const fcl::Transform3<S>& t2, S tolerance, fcl::GJKSolverType solver)
{
  fcl::CollisionObject<S> o1(g1, t1);
  fcl::CollisionObject<S> o2(g2, t2);
  fcl::DistanceRequest<S> req(false, false, 0.0, 0.0, tolerance, solver);
  fcl::DistanceResult<S> res;
  return fcl::distance(&o1, &o2, req, res);
}

// The exact separation between the cylinder at `t` and the plane
// `z = kFloorTopZ`. Only valid while the box's top face is the nearest
// feature, which is what `face_is_nearest` guards.
double closed_form(const fcl::Transform3<S>& t)
{
  const Eigen::Matrix<S, 3, 1> axis = t.linear().col(2);
  const double az = axis.z();
  const double h = kCylLength * 0.5;
  return t.translation().z() - kFloorTopZ - h * std::fabs(az) -
         kCylRadius * std::sqrt(std::fmax(0.0, 1.0 - az * az));
}

// True when no point of the cylinder can be closer to a box *edge* than to the
// top face: the whole shape is above the face and its silhouette is strictly
// inside the footprint. `h * |a_xy| + r` bounds the horizontal offset of any
// cylinder point from the centre and `h + r` bounds the vertical one.
bool face_is_nearest(const fcl::Transform3<S>& t)
{
  const Eigen::Matrix<S, 3, 1> axis = t.linear().col(2);
  const double h = kCylLength * 0.5;
  const double reach = h * std::hypot(axis.x(), axis.y()) + kCylRadius;
  const double half = kFloorSize * 0.5;
  return std::fabs(t.translation().x()) + reach < half &&
         std::fabs(t.translation().y()) + reach < half &&
         t.translation().z() - (h + kCylRadius) > kFloorTopZ;
}

// `prbt_flange`'s world transform at case 8148, row-major, exactly as the
// oracle's `fk` op answered it for that state's joint values
// (`prbt_joint_1..6` = -0.681765976663856, -1.9643143747676026,
// 1.9502553948473182, -0.2497111438317039, -0.47123744163653836,
// -1.186060955153117). The cylinder's own `<origin xyz="0 0 -0.0035">` is
// applied on top of it below.
constexpr double kCase8148Flange[16] = {
  -0.7632298994630945,  -0.6239369787320012,  0.16787723829136986,  -0.07043886091334942,
  -0.6437259598814009,  0.7119010575781851,   -0.2807379076537945,  0.04502997236882393,
  0.05565077843412082,  -0.32233450139543596, -0.9449886031428274,  -0.17294436805834873,
  0.0,                  0.0,                  0.0,                  1.0
};

fcl::Transform3<S> case_8148_cylinder_pose()
{
  fcl::Transform3<S> flange = fcl::Transform3<S>::Identity();
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      flange.linear()(row, col) = kCase8148Flange[row * 4 + col];
    }
    flange.translation()(row) = kCase8148Flange[row * 4 + 3];
  }
  fcl::Transform3<S> local = fcl::Transform3<S>::Identity();
  local.translation() = Eigen::Matrix<S, 3, 1>(0.0, 0.0, -0.0035);
  return flange * local;
}

}  // namespace

int main()
{
  auto cyl = std::make_shared<fcl::Cylinder<S>>(kCylRadius, kCylLength);
  auto box = std::make_shared<fcl::Box<S>>(kFloorSize, kFloorSize, kFloorThickness);
  fcl::Transform3<S> box_pose = fcl::Transform3<S>::Identity();
  box_pose.translation() = Eigen::Matrix<S, 3, 1>(0.0, 0.0, kFloorTopZ - kFloorThickness * 0.5);

  std::printf("idx,closed,d_cyl_first,d_box_first,d_tight,d_indep\n");
  int emitted = 0;
  // 2000 poses, matching `fcl-distance-tolerance-probe`'s sample size. The
  // draw ceiling is a stop, not a target: a run that cannot fill the sample
  // says so instead of spinning.
  for (int draw = 0; draw < 20000 && emitted < 2000; ++draw)
  {
    fcl::Transform3<S> t = fcl::Transform3<S>::Identity();
    if (emitted == 0)
    {
      t = case_8148_cylinder_pose();
    }
    else
    {
      t.linear() = next_rotation().toRotationMatrix();
      // The band case 8148 sits in: the sweep's separated `floor`/link values
      // on this fixture run from about `0.27` to `0.45`, and the horizontal
      // offsets stay inside a 30cm square around the base, far from the 4x4
      // footprint's edges.
      t.translation() = Eigen::Matrix<S, 3, 1>(next_unit() * 0.3 - 0.15, next_unit() * 0.3 - 0.15,
                                               kFloorTopZ + 0.05 + next_unit() * 0.4);
    }
    if (!face_is_nearest(t))
    {
      continue;
    }
    const double exact = closed_form(t);
    const double d_cyl_first = distance_with(cyl, t, box, box_pose, 1e-6, fcl::GST_LIBCCD);
    const double d_box_first = distance_with(box, box_pose, cyl, t, 1e-6, fcl::GST_LIBCCD);
    const double d_tight = distance_with(cyl, t, box, box_pose, 1e-12, fcl::GST_LIBCCD);
    const double d_indep = distance_with(cyl, t, box, box_pose, 1e-12, fcl::GST_INDEP);
    std::printf("%d,%.17e,%.17e,%.17e,%.17e,%.17e\n", emitted, exact, d_cyl_first, d_box_first,
                d_tight, d_indep);
    ++emitted;
  }
  if (emitted < 2000)
  {
    std::fprintf(stderr, "only %d admissible poses in 20000 draws\n", emitted);
    return 2;
  }
  return 0;
}
