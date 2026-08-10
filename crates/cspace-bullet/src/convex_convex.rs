// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h
//   bullet3/src/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp

//! `btConvexConvexAlgorithm::processCollision` -- one GJK query per convex
//! pair, with the query's cut-off assembled from both margins and both
//! thresholds.
//!
//! # Why so little of a 500-line function is here
//!
//! `processCollision` is mostly branches this path cannot enter. Each is
//! excluded by a fact about the shapes MoveIt's continuous check builds, not
//! by a judgment that it is unimportant:
//!
//! - **The three capsule branches** (`btConvexConvexAlgorithm.cpp:292-356`)
//!   need `CAPSULE_SHAPE_PROXYTYPE` on at least one side.
//!   `createShapePrimitive` builds a box, sphere, cylinder, cone, convex hull
//!   or compound of those (`bullet_utils.cpp:60-210`) and never a capsule.
//!
//! - **The polyhedral SAT/clipping branch** (`:411-697`), and with it
//!   `btPolyhedralContactClipping`, `btConvexPolyhedron` and
//!   `btConvexHullComputer`, is gated on `min0->isPolyhedral() &&
//!   min1->isPolyhedral()`. Every pair that reaches here in the continuous
//!   path has a `CastHullShape` on one side, whose type is
//!   `CUSTOM_CONVEX_SHAPE_TYPE` (`bullet_utils.hpp:251`);
//!   `isPolyhedral` is `proxyType < IMPLICIT_CONVEX_SHAPES_START_HERE`
//!   (`btBroadphaseProxy.h:130-133`) and `CUSTOM_CONVEX_SHAPE_TYPE` sorts
//!   after it, so the conjunction is false. That a cast shape is always on one
//!   side follows from three rules together: `addCollisionObject` casts
//!   exactly the `KinematicFilter` group (`bullet_cast_bvh_manager.cpp:151-162`),
//!   `needsCollision` rejects kinematic-versus-kinematic in cast mode
//!   (`bullet_utils.hpp:553-564`), and the filter masks cull
//!   static-versus-static (`bullet_utils.cpp:270-291`).
//!
//! - **The perturbation loop** (`:704-775`) runs `m_numPerturbationIterations`
//!   extra queries at rotated poses. That count is `0` from
//!   `CreateFunc::CreateFunc` (`:173-177`) and MoveIt never calls
//!   `setConvexConvexMultipointIterations`.
//!
//! - **`USE_SEPDISTANCE_UTIL2`** is not defined in any build.
//!
//! What is left is the manifold hand-off, the `ClosestPointInput`, and one
//! call to [`crate::gjk`].

use crate::discrete_detector::ClosestPointInput;
use crate::gjk::{GjkPairDetector, PenetrationDepthSolver};
use crate::linear_math::{Scalar, Transform};
use crate::manifold::{ManifoldResult, PersistentManifold};
use crate::shapes::ConvexShape;
use crate::simplex::VoronoiSimplexSolver;

/// `btConvexConvexAlgorithm::processCollision`
/// (`btConvexConvexAlgorithm.cpp:273-789`), reduced to the reachable path.
///
/// `min0`/`min1` are `body0Wrap`'s and `body1Wrap`'s shapes and the two
/// transforms are their world transforms; upstream reads both out of the
/// wrappers it is handed.
///
/// The manifold is created by the caller rather than here. Upstream allocates
/// it from the dispatcher's pool on first use and remembers it in
/// `m_manifoldPtr`, and `m_ownManifold` then gates the closing
/// `refreshContactPoints`; both are bookkeeping for a pool this port does not
/// have, and the threshold that allocation carries is the only part the query
/// reads.
pub fn process_collision(
    min0: &dyn ConvexShape,
    transform_a: &Transform,
    min1: &dyn ConvexShape,
    transform_b: &Transform,
    manifold: PersistentManifold,
    result_out: &mut dyn ManifoldResult,
) {
    result_out.set_persistent_manifold(Some(manifold));

    let mut simplex_solver = VoronoiSimplexSolver::new();
    // `m_pdSolver` is the dispatcher's shared `btGjkEpaPenetrationDepthSolver`
    // (`btDefaultCollisionConfiguration.cpp:35`), never null here.
    let mut gjk_pair_detector = GjkPairDetector::new(
        min0,
        min1,
        &mut simplex_solver,
        Some(PenetrationDepthSolver::GjkEpa),
    );

    let input = ClosestPointInput {
        transform_a: *transform_a,
        transform_b: *transform_b,
        maximum_distance_squared: maximum_distance_squared(
            min0.margin(),
            min1.margin(),
            manifold.contact_breaking_threshold,
            result_out.state().closest_point_distance_threshold,
        ),
    };

    gjk_pair_detector.get_closest_points(&input, result_out);

    // `if (m_ownManifold) resultOut->refreshContactPoints();` (`:785-788`).
    result_out.refresh_contact_points();
}

/// `input.m_maximumDistanceSquared = min0->getMargin() + min1->getMargin() +
/// m_manifoldPtr->getContactBreakingThreshold() +
/// resultOut->m_closestPointDistanceThreshold;` then squared
/// (`btConvexConvexAlgorithm.cpp:389-392`).
///
/// Both margins enter the sum even though the query is run against the shapes
/// *with* their margins, so the cut-off is deliberately looser than the
/// distance it is compared against.
///
/// `closest_point_distance_threshold` is zero on every continuous query:
/// `BulletBVHManager`'s constructor seeds `contact_distance_` to
/// `BULLET_DEFAULT_CONTACT_DISTANCE` (`bullet_bvh_manager.cpp:55`, `0.00f`) and
/// `checkRobotCollisionHelperCCD` never raises it -- the two
/// `MAX_DISTANCE_MARGIN` assignments are both on the discrete manager
/// (`collision_env_bullet.cpp:127,187`). It is a parameter rather than a
/// constant because that is what makes the term visible to a test at all.
///
/// Upstream writes this inline; it is a function here so that the one number
/// this module contributes to the result can be asserted directly, rather than
/// only through a contact -- which would be pinning GJK a second time instead
/// of pinning this.
#[must_use]
pub fn maximum_distance_squared(
    margin_a: Scalar,
    margin_b: Scalar,
    contact_breaking_threshold: Scalar,
    closest_point_distance_threshold: Scalar,
) -> Scalar {
    let sum = margin_a + margin_b + contact_breaking_threshold + closest_point_distance_threshold;
    sum * sum
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::discrete_detector::Result as DetectorResult;
    use crate::linear_math::{BT_LARGE_FLOAT, Vec3};
    use crate::manifold::{CONTACT_BREAKING_THRESHOLD, ManifoldResultState};
    use crate::probe_fixture::{IDENTITY, at, diff, diff_vec3, probe_shapes, rot60_at, row};

    /// `probe.cpp`'s `RecordingResult`, and MoveIt's
    /// `TesseractBroadphaseBridgedManifoldResult` in miniature: an
    /// `add_contact_point` that replaces the base's rather than extending it,
    /// so no contact ever reaches a manifold. The count below is this
    /// counter, not `num_contacts`.
    struct RecordingResult {
        state: ManifoldResultState,
        count: usize,
        normal: Vec3,
        point: Vec3,
        depth: Scalar,
    }

    impl RecordingResult {
        fn new(ta: Transform, tb: Transform, closest_point_distance_threshold: Scalar) -> Self {
            let mut state = ManifoldResultState::new(0, ta, 1, tb);
            state.closest_point_distance_threshold = closest_point_distance_threshold;
            Self {
                state,
                count: 0,
                normal: Vec3::zero(),
                point: Vec3::zero(),
                depth: 0.0,
            }
        }
    }

    impl DetectorResult for RecordingResult {
        fn add_contact_point(
            &mut self,
            normal_on_b_in_world: Vec3,
            point_in_world: Vec3,
            depth: Scalar,
        ) {
            self.count += 1;
            self.normal = normal_on_b_in_world;
            self.point = point_in_world;
            self.depth = depth;
        }
    }

    impl ManifoldResult for RecordingResult {
        fn state(&mut self) -> &mut ManifoldResultState {
            &mut self.state
        }
    }

    /// The `cc_*` rows of `tools/bullet-epa-reference/build.sh`'s stdout,
    /// verbatim: the real `btConvexConvexAlgorithm::processCollision` from
    /// bullet3 @ `7dee3436`, reached through a `btCollisionDispatcher` with
    /// `CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD` cleared exactly as
    /// `BulletBVHManager` clears it, asked for `BT_CLOSEST_POINT_ALGORITHMS`.
    ///
    /// Every pair has a non-polyhedral shape on at least one side, so each
    /// row is on the branch the module docs argue is the only reachable one.
    /// A box-versus-box row would take the SAT/clipping branch and assert
    /// against arithmetic this port does not carry.
    ///
    /// The three `cc_cone_cyl_*cutoff*` rows are what make the cut-off
    /// observable rather than merely computed: both shapes carry a zero
    /// margin, so the bound is `gContactBreakingThreshold` alone, and their
    /// apex-to-face gaps of 0.015 and 0.03 fall either side of it. The third
    /// re-runs the 0.03 pair with `m_closestPointDistanceThreshold = 0.25`
    /// and gets its contact back, which is the only row here that could fail
    /// if that term were dropped from the sum. That threshold is zero on
    /// every real continuous query -- see [`maximum_distance_squared`] -- so
    /// the two non-zero rows pin the formula rather than a reachable pose.
    ///
    /// Fields: `name|contacts|normalOnB xyz|pointOnB xyz|depth|maxDistSq`.
    const BULLET_REFERENCE: &str = "\
cc_sphere_cyl_overlap|1|-0.988073647|-0.153982207|0.000172737826|0.304717928|0.0474874936|-5.32716513e-05|-0.191604018|0.270399988
cc_sphere_cyl_far|0|0|0|0|0|0|0|0|0.270399988
cc_cone_cyl_inside_cutoff|1|0|9.93409685e-07|-0.99999994|0|-1.49011612e-08|0.415000021|0.0150000164|0.00039999999
cc_cone_cyl_past_cutoff|0|0|0|0|0|0|0|0|0.00039999999
cc_cone_cyl_threshold_widens|1|0|-9.93410708e-07|-1|0|2.98023224e-08|0.430000007|0.0300000012|0.0729000047
cc_cone_cyl_deep|1|-0.954728544|0.00417782972|-0.297449321|-0.199743375|0.000841829111|-0.199999988|-0.369415283|0.00039999999
cc_hull_cone_rot60|1|-0.447253227|-0.000218240413|0.894407332|0.206645817|0.154420465|0.0866877064|-0.208727837|0.00490000006
cc_margin_box_sphere|1|-1|4.77601292e-09|4.29841158e-08|0.550000012|0.0500000007|1.2895236e-08|0.0500000119|0.129600003
";

    /// Every `processCollision` row, against the port.
    #[test]
    fn bullet_reference_process_collision() {
        let (_, _, margin_box, sphere, small_sphere, cyl, cone, hull) = probe_shapes();
        let mut bad = Vec::new();

        let mut case = |name: &str,
                        a: &dyn ConvexShape,
                        ta: &Transform,
                        b: &dyn ConvexShape,
                        tb: &Transform,
                        closest_point_distance_threshold: Scalar| {
            let f = row(BULLET_REFERENCE, name, 10);
            let n = |k: usize| -> Scalar {
                f[k].parse()
                    .unwrap_or_else(|e| panic!("{name}: field {k} ({:?}): {e}", f[k]))
            };

            let mut out = RecordingResult::new(*ta, *tb, closest_point_distance_threshold);
            // `getNewManifold` takes the smaller of the two bodies' contact
            // processing thresholds; `btCollisionObject` seeds that to
            // `BT_LARGE_FLOAT` and nothing in this path lowers it.
            let manifold = PersistentManifold::new(BT_LARGE_FLOAT);
            process_collision(a, ta, b, tb, manifold, &mut out);

            let want_count: usize = f[1]
                .parse()
                .unwrap_or_else(|e| panic!("{name}: field 1 ({:?}): {e}", f[1]));
            if out.count != want_count {
                bad.push(format!(
                    "{name}.contacts: port {}, bullet {want_count}",
                    out.count
                ));
            }
            if out.count > 0 {
                diff_vec3(
                    &mut bad,
                    name,
                    "normal",
                    out.normal,
                    Vec3::new(n(2), n(3), n(4)),
                );
                diff_vec3(
                    &mut bad,
                    name,
                    "point",
                    out.point,
                    Vec3::new(n(5), n(6), n(7)),
                );
                diff(&mut bad, name, "depth", out.depth, n(8));
            }
            diff(
                &mut bad,
                name,
                "maxDistSq",
                maximum_distance_squared(
                    a.margin(),
                    b.margin(),
                    CONTACT_BREAKING_THRESHOLD,
                    closest_point_distance_threshold,
                ),
                n(9),
            );
        };

        case(
            "cc_sphere_cyl_overlap",
            &sphere,
            &IDENTITY,
            &cyl,
            &at(0.6, 0.1, 0.2),
            0.0,
        );
        case(
            "cc_sphere_cyl_far",
            &sphere,
            &IDENTITY,
            &cyl,
            &at(3.0, 0.0, 0.0),
            0.0,
        );
        case(
            "cc_cone_cyl_inside_cutoff",
            &cone,
            &IDENTITY,
            &cyl,
            &at(0.0, 0.0, 0.915),
            0.0,
        );
        case(
            "cc_cone_cyl_past_cutoff",
            &cone,
            &IDENTITY,
            &cyl,
            &at(0.0, 0.0, 0.93),
            0.0,
        );
        case(
            "cc_cone_cyl_threshold_widens",
            &cone,
            &IDENTITY,
            &cyl,
            &at(0.0, 0.0, 0.93),
            0.25,
        );
        case(
            "cc_cone_cyl_deep",
            &cone,
            &IDENTITY,
            &cyl,
            &at(0.1, 0.0, 0.3),
            0.0,
        );
        case(
            "cc_hull_cone_rot60",
            &hull,
            &IDENTITY,
            &cone,
            &rot60_at(0.4, 0.1, 0.05),
            0.05,
        );
        case(
            "cc_margin_box_sphere",
            &margin_box,
            &IDENTITY,
            &small_sphere,
            &at(0.85, 0.05, 0.0),
            0.0,
        );

        assert!(
            bad.is_empty(),
            "{} deviations:\n{}",
            bad.len(),
            bad.join("\n")
        );
    }
}
