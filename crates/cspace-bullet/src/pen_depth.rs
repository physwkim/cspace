// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// EPA Copyright (c) Ricardo Padrela 2006
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h
//   bullet3/src/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp

//! `btGjkEpaPenetrationDepthSolver` -- the adapter `btGjkPairDetector` reaches
//! for once its own GJK reports the shapes overlap.
//!
//! It is nine tries at [`crate::epa`], not one. EPA's answer depends on the
//! tetrahedron `EncloseOrigin` happens to build, which depends on the first
//! support direction, which is the guess; a guess that produces a degenerate
//! tetrahedron makes `Evaluate` give up with `InvalidHull`. So `calcPenDepth`
//! walks a fixed list of nine directions and returns the first that answers,
//! which makes the *order* of that list part of the result rather than a
//! detail -- the two centre-to-centre directions come first, then the three
//! axes, then the four diagonals.

use crate::epa::{Results, distance, penetration};
use crate::linear_math::{Transform, Vec3};
use crate::shapes::ConvexShape;
use crate::simplex::VoronoiSimplexSolver;

/// The three values `calcPenDepth` writes through out-parameters: `v`,
/// `wWitnessOnA` and `wWitnessOnB`.
///
/// One struct rather than three `&mut btVector3`, because upstream writes all
/// three together on every one of its exit paths and never reads any of them
/// on the way in -- the incoming `v` is cast to `void` on the first line.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PenDepth {
    /// `v` -- the penetration normal on the `true` path, the separation
    /// direction on the `Distance` path, and zero when every guess failed.
    pub v: Vec3,
    /// `wWitnessOnA`, in world space.
    pub witness_on_a: Vec3,
    /// `wWitnessOnB`, in world space.
    pub witness_on_b: Vec3,
}

/// `btGjkEpaPenetrationDepthSolver::calcPenDepth`
/// (`btGjkEpaPenetrationDepthSolver.cpp:23-80`).
///
/// The return value alone does not say what happened. `true` means a guess
/// reached EPA and `out.v` is the penetration normal; `false` means either
/// that some guess found the shapes *separated* -- in which case `out` holds
/// that separation -- or that all nine guesses failed, in which case `out` is
/// zeroed. `btGjkPairDetector` tells the two apart by the distance it computes
/// from the witnesses, not by the flag.
///
/// `simplex_solver` is reset once per guess and is otherwise unused here.
/// Upstream casts it to `void`, yet still calls `reset()` on it nine times,
/// and the caller goes on using that same solver afterwards.
pub fn calc_pen_depth(
    simplex_solver: &mut VoronoiSimplexSolver,
    convex_a: &dyn ConvexShape,
    convex_b: &dyn ConvexShape,
    transform_a: &Transform,
    transform_b: &Transform,
    out: &mut PenDepth,
) -> bool {
    let guess_vectors = [
        (transform_b.origin - transform_a.origin).safe_normalize(),
        (transform_a.origin - transform_b.origin).safe_normalize(),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(0.0, 1.0, 1.0),
        Vec3::new(1.0, 0.0, 1.0),
    ];

    for guess_vector in guess_vectors {
        simplex_solver.reset();

        let mut results = Results::default();

        // `usemargins` defaults to `true` here (`btGjkEpa2.h:54-58`), so this
        // is the full shape and not the core one `Distance` uses below.
        if penetration(
            convex_a,
            transform_a,
            convex_b,
            transform_b,
            guess_vector,
            &mut results,
            true,
        ) {
            out.witness_on_a = results.witnesses[0];
            out.witness_on_b = results.witnesses[1];
            out.v = results.normal;
            return true;
        } else if distance(
            convex_a,
            transform_a,
            convex_b,
            transform_b,
            guess_vector,
            &mut results,
        ) {
            out.witness_on_a = results.witnesses[0];
            out.witness_on_b = results.witnesses[1];
            out.v = results.normal;
            return false;
        }
    }

    // Failed to find a distance or a penetration.
    *out = PenDepth::default();
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::linear_math::Scalar;
    use crate::probe_fixture::{IDENTITY, at, diff_vec3 as diff, probe_shapes, rot60_at, row};
    use crate::shapes::BoxShape;

    /// Two boxes overlapping along x answer on the *first* guess, which for
    /// this pair normalizes to `(1, 0, 0)`. Asserted against `Penetration`
    /// called directly with that guess rather than against transcribed
    /// numbers: what this module adds is the guess list and the field copy,
    /// and `epa`'s own fixtures already pin the arithmetic.
    #[test]
    fn the_first_guess_answers_for_boxes_overlapping_along_x() {
        let a = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
        let b = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
        let tb = at(0.95, 0.0, 0.0);
        let mut solver = VoronoiSimplexSolver::new();
        let mut got = PenDepth::default();

        assert!(calc_pen_depth(
            &mut solver,
            &a,
            &b,
            &IDENTITY,
            &tb,
            &mut got
        ));

        let mut want = Results::default();
        assert!(penetration(
            &a,
            &IDENTITY,
            &b,
            &tb,
            Vec3::new(1.0, 0.0, 0.0),
            &mut want,
            true,
        ));
        assert_eq!(
            got,
            PenDepth {
                v: want.normal,
                witness_on_a: want.witnesses[0],
                witness_on_b: want.witnesses[1],
            }
        );
    }

    /// Separated shapes take the `Distance` arm: `false`, but with the
    /// separation written rather than zeros. The caller cannot read the flag
    /// alone.
    #[test]
    fn separated_shapes_return_false_with_the_separation_written() {
        let mut a = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
        let mut b = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
        a.set_margin(0.0);
        b.set_margin(0.0);
        let tb = at(3.0, 0.0, 0.0);
        let mut solver = VoronoiSimplexSolver::new();
        let mut got = PenDepth::default();

        assert!(!calc_pen_depth(
            &mut solver,
            &a,
            &b,
            &IDENTITY,
            &tb,
            &mut got
        ));

        let mut want = Results::default();
        assert!(distance(
            &a,
            &IDENTITY,
            &b,
            &tb,
            Vec3::new(1.0, 0.0, 0.0),
            &mut want,
        ));
        assert_eq!(
            got,
            PenDepth {
                v: want.normal,
                witness_on_a: want.witnesses[0],
                witness_on_b: want.witnesses[1],
            }
        );
        assert_ne!(got, PenDepth::default(), "the all-failed path zeroes out");
    }

    /// The `p_*` rows of `tools/bullet-epa-reference/build.sh`'s stdout,
    /// verbatim: the real `btGjkEpaPenetrationDepthSolver::calcPenDepth` on
    /// five pairs. Fields: `name|ok|v xyz|witnessA xyz|witnessB xyz`.
    ///
    /// `p_box_box_coincident` is the row that pins the guess list's head.
    /// With the centres on top of each other the first two entries are
    /// `safeNormalize` of a zero vector, so what the loop actually tries is
    /// `(1, 0, 0)` -- and the answer comes back along z, which is why the
    /// guess cannot be inferred from the result and has to be pinned here.
    const BULLET_REFERENCE: &str = "\
p_box_box_overlap|1|-1|-5.30293946e-05|5.30293946e-05|0.5|0.460000008|0.459997386|0.449999988|0.459997356|0.460000038
p_box_box_diagonal|1|-1|6.20902201e-13|-5.03537251e-07|0.499999136|0.138829023|-0.129585564|0.200000018|0.138829023|-0.129585713
p_box_box_coincident|1|1.87058913e-06|-1.69519637e-06|1|-0.000133827329|6.63101673e-05|-0.499996066|-0.000132424393|6.50387738e-05|0.25
p_box_box_separated|0|-1|0|0|0.5|0.5|0.5|2.5|0.5|0.5
p_cone_cyl_rot60|1|-0.171172246|-0.938990176|-0.298324406|0.0274838991|0.149303183|-0.0857977271|-0.00921033695|-0.0519883633|-0.149749607
";

    fn reference(name: &str) -> (bool, PenDepth) {
        let f = row(BULLET_REFERENCE, name, 11);
        let n = |i: usize| -> Scalar {
            f[i].parse()
                .unwrap_or_else(|e| panic!("{name}: field {i} ({:?}): {e}", f[i]))
        };
        (
            f[1] == "1",
            PenDepth {
                v: Vec3::new(n(2), n(3), n(4)),
                witness_on_a: Vec3::new(n(5), n(6), n(7)),
                witness_on_b: Vec3::new(n(8), n(9), n(10)),
            },
        )
    }

    /// Every `calcPenDepth` row, against the port.
    #[test]
    fn bullet_reference_calc_pen_depth() {
        let (unit_box, flat_box, margin_box, _, _, cyl, cone, _) = probe_shapes();
        let rot60 = rot60_at(0.3, 0.1, 0.2);

        let mut bad = Vec::new();
        let mut case = |name: &str,
                        a: &dyn ConvexShape,
                        ta: &Transform,
                        b: &dyn ConvexShape,
                        tb: &Transform| {
            let mut solver = VoronoiSimplexSolver::new();
            let mut got = PenDepth::default();
            let ok = calc_pen_depth(&mut solver, a, b, ta, tb, &mut got);
            let (want_ok, want) = reference(name);
            if ok != want_ok {
                bad.push(format!("{name}.ok: port {ok}, bullet {want_ok}"));
            }
            diff(&mut bad, name, "v", got.v, want.v);
            diff(
                &mut bad,
                name,
                "witnessA",
                got.witness_on_a,
                want.witness_on_a,
            );
            diff(
                &mut bad,
                name,
                "witnessB",
                got.witness_on_b,
                want.witness_on_b,
            );
        };

        case(
            "p_box_box_overlap",
            &margin_box,
            &IDENTITY,
            &margin_box,
            &at(0.95, 0.0, 0.0),
        );
        case(
            "p_box_box_diagonal",
            &margin_box,
            &IDENTITY,
            &flat_box,
            &at(0.6, 0.35, -0.2),
        );
        case(
            "p_box_box_coincident",
            &margin_box,
            &IDENTITY,
            &flat_box,
            &IDENTITY,
        );
        case(
            "p_box_box_separated",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(3.0, 0.0, 0.0),
        );
        case("p_cone_cyl_rot60", &cone, &IDENTITY, &cyl, &rot60);

        assert!(
            bad.is_empty(),
            "{} deviations:\n{}",
            bad.len(),
            bad.join("\n")
        );
    }
}
