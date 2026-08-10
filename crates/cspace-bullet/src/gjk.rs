// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h
//   bullet3/src/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp
//
// The boolean pre-pass in this file is Bullet's own transcription of libCCD's
// GJK, carried in btGjkPairDetector.cpp under Bullet's zlib notice with the
// comment it still has upstream:
//
//   we add a separate implementation to check if the convex shapes intersect
//   See also "Real-time Collision Detection with Implicit Objects" by Leif
//   Olvang
//   Todo: integrate the simplex penetration check directly inside the Bullet
//   btVoronoiSimplexSolver and remove this temporary code from libCCD

//! `btGjkPairDetector` -- the entry point `btConvexConvexAlgorithm` calls to
//! turn two convex shapes and their transforms into one contact.
//!
//! # Two GJKs, run one after the other
//!
//! `getClosestPointsNonVirtual` runs *two* independent algorithms on the same
//! pair before it decides anything.
//!
//! The first is `do_simplex` and friends: a boolean intersect/no-intersect
//! GJK transcribed from libCCD, keeping its own four-point simplex, added
//! upstream to fix issue #1703. It never produces a distance; its whole output
//! is `status`, which is `0` for "intersecting", `-1` for "not intersecting"
//! and `-2` when the loop ran out of iterations without deciding.
//!
//! The second is the classic loop over [`crate::simplex`]'s
//! `btVoronoiSimplexSolver`, which produces the separating axis, the two
//! witness points and the distance.
//!
//! The two are not alternatives. The libCCD pass runs unconditionally, and its
//! `status == 0` is *or*-ed into the condition that sends the pair to
//! [`crate::pen_depth`] -- so a pair the second GJK thinks is separated still
//! reaches EPA if the first one saw an intersection. That is the whole point
//! of the pre-pass, and it is why porting only the classic loop would change
//! results.
//!
//! # `m_lastUsedMethod`
//!
//! Upstream calls this "some debugging to fix degeneracy problems", but it is
//! the only record of which of the several exits produced the answer, and the
//! port's tests read it. `1` is the plain GJK distance, `2` the degenerate
//! zero-length axis, `3` EPA, `5`/`8` an EPA result rejected for being
//! shallower than what GJK already had, `6` the second-GJK rescue,
//! `9` an EPA normal too short to normalize, and `10` a normal flipped by the
//! direction check at the end.

use crate::discrete_detector::{ClosestPointInput, Result};
use crate::linear_math::{BT_LARGE_FLOAT, SIMD_EPSILON, Scalar, Transform, Vec3, bt_fuzzy_zero};
use crate::pen_depth::{PenDepth, calc_pen_depth};
use crate::shapes::ConvexShape;
use crate::simplex::VoronoiSimplexSolver;

/// `REL_ERROR2` (`btGjkPairDetector.cpp:35`) -- "must be above the machine
/// epsilon", and it is: `1e-6` against an `f32` epsilon near `1.19e-7`.
pub const REL_ERROR2: Scalar = 1.0e-6;

/// `gGjkEpaPenetrationTolerance` (`btGjkPairDetector.cpp:36`).
///
/// A mutable global upstream, so that an application can retune it. Nothing on
/// MoveIt's collision path writes it, so it is a constant here; if that ever
/// stops being true this has to become state on the detector.
pub const GJK_EPA_PENETRATION_TOLERANCE: Scalar = 0.001;

/// `gGjkMaxIter` (`btGjkPairDetector.cpp:717`) -- "this is to catch invalid
/// input, perhaps check for #NaN?". It bounds both loops.
const GJK_MAX_ITER: i32 = 1000;

/// `btComputeSupport` (`btGjkPairDetector.cpp:78-99`).
///
/// `check2d` is gone: it zeroes the z component of both support points, and it
/// is `isConvex2d() && isConvex2d()`, true only for `BOX_2D_SHAPE_PROXYTYPE`
/// and `CONVEX_2D_SHAPE_PROXYTYPE`. Neither is among the five shapes
/// [`crate::shapes`] ports, so the flag is false on every pair this crate can
/// construct.
///
/// Returns `(supAworld, supBworld, aMinb)`.
fn compute_support(
    convex_a: &dyn ConvexShape,
    local_trans_a: &Transform,
    convex_b: &dyn ConvexShape,
    local_trans_b: &Transform,
    dir: Vec3,
) -> (Vec3, Vec3, Vec3) {
    let separating_axis_in_a = local_trans_a.basis.transposed_mul_vec(dir);
    let separating_axis_in_b = local_trans_b.basis.transposed_mul_vec(-dir);

    let p_in_a = convex_a.local_get_supporting_vertex_without_margin(separating_axis_in_a);
    let q_in_b = convex_b.local_get_supporting_vertex_without_margin(separating_axis_in_b);

    let sup_a_world = local_trans_a.transform_point(p_in_a);
    let sup_b_world = local_trans_b.transform_point(q_in_b);

    (sup_a_world, sup_b_world, sup_a_world - sup_b_world)
}

/// `btSupportVector` -- a Minkowski-difference point and the two shape points
/// it came from.
#[derive(Clone, Copy, Debug, Default)]
struct SupportVector {
    /// `v` -- support point in the Minkowski sum.
    v: Vec3,
    /// `v1` -- support point on object 1.
    v1: Vec3,
    /// `v2` -- support point on object 2.
    v2: Vec3,
}

/// `btSimplex` -- libCCD's own simplex, unrelated to
/// [`crate::simplex::VoronoiSimplexSolver`].
#[derive(Clone, Copy, Debug)]
struct CcdSimplex {
    ps: [SupportVector; 4],
    /// `last` -- index of the last added point, `-1` when empty. Kept signed
    /// rather than as a count, because `btSimplexSetSize(s, n)` writes
    /// `last = n - 1` and `btSimplexSize(s)` reads `last + 1`.
    last: i32,
}

impl CcdSimplex {
    /// `btSimplexInit` -- which only writes `last`, leaving `ps` whatever the
    /// stack held. Zeroing it here is unobservable: `size()` is `last + 1` and
    /// every index up to `last` was written by [`Self::add`] or
    /// [`Self::set`] before any read.
    const fn new() -> Self {
        Self {
            ps: [SupportVector {
                v: Vec3::zero(),
                v1: Vec3::zero(),
                v2: Vec3::zero(),
            }; 4],
            last: -1,
        }
    }

    /// `btSimplexSize`.
    const fn size(&self) -> i32 {
        self.last + 1
    }

    /// `btSimplexPoint`. Upstream notes "here is no check on boundaries"; the
    /// index arithmetic below is the same, so an out-of-range index panics
    /// here rather than reading past the array.
    fn point(&self, idx: usize) -> SupportVector {
        self.ps[idx]
    }

    /// `ccdSimplexLast`.
    fn last_point(&self) -> SupportVector {
        self.ps[usize::try_from(self.last).expect("simplex is non-empty")]
    }

    /// `btSimplexAdd` -- "here is no check on boundaries in sake of speed".
    fn add(&mut self, v: &SupportVector) {
        self.last += 1;
        self.ps[usize::try_from(self.last).expect("simplex holds at most 4")] = *v;
    }

    /// `btSimplexSet`.
    ///
    /// Upstream's `A`, `B`, `C`, `D` are pointers *into* `ps`, so a `set` can
    /// alias its own source; the ports of `btDoSimplex3`/`4` take those four
    /// by value up front instead. That is equivalent because upstream never
    /// reads a slot after overwriting it -- and where it would have,
    /// `btDoSimplex3`'s B/C swap, it makes the same copy itself.
    fn set(&mut self, pos: usize, a: &SupportVector) {
        self.ps[pos] = *a;
    }

    /// `btSimplexSetSize`.
    const fn set_size(&mut self, size: i32) {
        self.last = size - 1;
    }
}

/// `ccdEq` -- equal to within `SIMD_EPSILON`, absolute *or* relative to the
/// larger operand.
fn ccd_eq(a: Scalar, b: Scalar) -> bool {
    let ab = (a - b).abs();
    if ab.abs() < SIMD_EPSILON {
        return true;
    }

    let (fa, fb) = (a.abs(), b.abs());
    if fb > fa {
        ab < SIMD_EPSILON * fb
    } else {
        ab < SIMD_EPSILON * fa
    }
}

/// `btVec3Eq` -- [`ccd_eq`] on all three components.
fn vec3_eq(a: Vec3, b: Vec3) -> bool {
    ccd_eq(a.x, b.x) && ccd_eq(a.y, b.y) && ccd_eq(a.z, b.z)
}

/// `ccdSign` -- zero inside `btFuzzyZero`'s band, not just at exactly zero.
fn ccd_sign(val: Scalar) -> i32 {
    if bt_fuzzy_zero(val) {
        0
    } else if val < 0.0 {
        -1
    } else {
        1
    }
}

/// `btTripleCross(a, b, c, d)` -- `(a x b) x c`.
fn triple_cross(a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    a.cross(b).cross(c)
}

/// `btVec3PointSegmentDist2` (`btGjkPairDetector.cpp:265-322`) -- squared
/// distance from `p` to segment `x0..b`, and the witness point on it.
///
/// The clamps are not `t < 0` and `t > 1` but `t < 0 || btFuzzyZero(t)` and
/// `t > 1 || ccdEq(t, 1)`, so a `t` inside the epsilon band snaps to the
/// endpoint and the witness is the endpoint exactly.
///
/// The two arms of the `else` are not the same arithmetic: with a witness the
/// distance is `|(d*t + x0) - P|^2`, without one it is `|d*t + (x0 - P)|^2`.
/// Same value in exact arithmetic, different rounding in `f32`, so callers
/// must pass the witness through rather than always asking for one.
fn vec3_point_segment_dist2(p: Vec3, x0: Vec3, b: Vec3, witness: Option<&mut Vec3>) -> Scalar {
    // direction of segment
    let mut d = b - x0;
    // precompute vector from P to x0
    let a = x0 - p;

    // `t = -btScalar(1.) * btVec3Dot(&a, &d);` -- negation and multiply by -1
    // agree on every IEEE-754 value including the zeroes and the NaN sign.
    let mut t = -a.dot(d);
    t /= d.dot(d);

    if t < 0.0 || bt_fuzzy_zero(t) {
        if let Some(w) = witness {
            *w = x0;
        }
        (x0 - p).length2()
    } else if t > 1.0 || ccd_eq(t, 1.0) {
        if let Some(w) = witness {
            *w = b;
        }
        (b - p).length2()
    } else if let Some(w) = witness {
        *w = d * t + x0;
        (*w - p).length2()
    } else {
        // recycling variables
        d = d * t + a;
        d.dot(d)
    }
}

/// `btVec3PointTriDist2` (`btGjkPairDetector.cpp:324-412`) -- squared distance
/// from `p` to triangle `x0, b, c`.
///
/// The six dot products are `btScalar` (`f32`) but `u, v, w, p, q, r, s, t` and
/// the distance accumulator are all declared `double` upstream, so the
/// barycentric solve runs in the wider type inside a float build. Where `s` and
/// `t` are then *used* decides which type each operation gets, and upstream is
/// not uniform about it: `btFuzzyZero`, `ccdEq` and `btVec3Scale` all take
/// `btScalar`, so those narrow, while `s > btScalar(0)` and `s < btScalar(1)`
/// promote the literal instead and compare in `double`. Both spellings are
/// reproduced as written rather than normalized to one type.
///
/// The two spellings of the *comparison* cannot in fact disagree, which is why
/// no fixture row pins that half. Each is guarded by `ccdEq`/`btFuzzyZero`
/// against the same bound, and those admit a whole `SIMD_EPSILON`: a `double`
/// below `1` can only narrow to `>= 1` by landing on exactly `1`, which is
/// inside `ccdEq(_, 1)`, and one at or above `1` cannot narrow below it at all.
/// The solve above is a different matter -- `s` and `t` themselves move -- and
/// `t_wide_solve` pins it.
fn vec3_point_tri_dist2(
    p: Vec3,
    x0: Vec3,
    b: Vec3,
    c: Vec3,
    mut witness: Option<&mut Vec3>,
) -> Scalar {
    let mut d1 = b - x0;
    let mut d2 = c - x0;
    let a = x0 - p;

    let u = f64::from(a.dot(a));
    let v = f64::from(d1.dot(d1));
    let w = f64::from(d2.dot(d2));
    let pp = f64::from(a.dot(d1));
    let q = f64::from(a.dot(d2));
    let r = f64::from(d1.dot(d2));

    let s = (q * r - w * pp) / (w * v - r * r);
    let t = (-s * r - q) / w;

    // The narrowing upstream performs at every `btScalar`-typed parameter:
    // `btFuzzyZero`, `ccdEq` and `btVec3Scale`.
    let (sf, tf) = (s as Scalar, t as Scalar);
    // `ccdEq(t + s, btScalar(1))` sums in `double` and narrows at the call --
    // not the sum of the two halves already narrowed.
    let ts = (t + s) as Scalar;

    if (bt_fuzzy_zero(sf) || s > 0.0)
        && (ccd_eq(sf, 1.0) || s < 1.0)
        && (bt_fuzzy_zero(tf) || t > 0.0)
        && (ccd_eq(tf, 1.0) || t < 1.0)
        && (ccd_eq(ts, 1.0) || t + s < 1.0)
    {
        if let Some(wit) = witness {
            d1 = d1 * sf;
            d2 = d2 * tf;
            *wit = x0 + d1 + d2;
            (*wit - p).length2()
        } else {
            let dist = s * s * v + t * t * w + 2.0 * s * t * r + 2.0 * s * pp + 2.0 * t * q + u;
            // `btScalar btVec3PointTriDist2(...)` narrows its `double`
            // accumulator at the return.
            dist as Scalar
        }
    } else {
        // `witness` is forwarded, not replaced by a local: see
        // `vec3_point_segment_dist2` on why asking for one changes the value.
        let mut dist = vec3_point_segment_dist2(p, x0, b, witness.as_deref_mut());

        let mut witness2 = Vec3::zero();
        let dist2 = vec3_point_segment_dist2(p, x0, c, Some(&mut witness2));
        if dist2 < dist {
            dist = dist2;
            if let Some(wit) = witness.as_deref_mut() {
                *wit = witness2;
            }
        }

        let dist2 = vec3_point_segment_dist2(p, b, c, Some(&mut witness2));
        if dist2 < dist {
            dist = dist2;
            if let Some(wit) = witness {
                *wit = witness2;
            }
        }

        dist
    }
}

/// `btDoSimplex2` (`btGjkPairDetector.cpp:414-456`).
///
/// Returns `1` when the origin lies on the segment (the shapes touch), `0`
/// otherwise, having narrowed the simplex and written the next search
/// direction into `dir`.
fn do_simplex2(simplex: &mut CcdSimplex, dir: &mut Vec3) -> i32 {
    // get last added as A, and the other point as B
    let a = simplex.last_point();
    let b = simplex.point(0);
    let ab = b.v - a.v;
    let ao = a.v * -1.0;

    let dot = ab.dot(ao);

    // check if origin doesn't lie on AB segment
    let tmp = ab.cross(ao);
    if bt_fuzzy_zero(tmp.dot(tmp)) && dot > 0.0 {
        return 1;
    }

    if bt_fuzzy_zero(dot) || dot < 0.0 {
        // origin is in the outside area of A
        simplex.set(0, &a);
        simplex.set_size(1);
        *dir = ao;
    } else {
        // origin is in the area where the AB segment is: keep the simplex and
        // set the direction to AB x AO x AB
        *dir = triple_cross(ab, ao, ab);
    }

    0
}

/// `btDoSimplex3` (`btGjkPairDetector.cpp:458-561`).
///
/// `-1` means the triangle has no area, so the simplex cannot be expanded and
/// no intersection can be found; `1` means the origin is on the triangle.
fn do_simplex3(simplex: &mut CcdSimplex, dir: &mut Vec3) -> i32 {
    let a = simplex.last_point();
    let b = simplex.point(1);
    let c = simplex.point(0);

    // check touching contact
    let dist = vec3_point_tri_dist2(Vec3::zero(), a.v, b.v, c.v, None);
    if bt_fuzzy_zero(dist) {
        return 1;
    }

    // check if the triangle is really a triangle (has area > 0)
    if vec3_eq(a.v, b.v) || vec3_eq(a.v, c.v) {
        return -1;
    }

    let ao = a.v * -1.0;
    let ab = b.v - a.v;
    let ac = c.v - a.v;
    let abc = ab.cross(ac);

    let tmp = abc.cross(ac);
    let dot = tmp.dot(ao);
    if bt_fuzzy_zero(dot) || dot > 0.0 {
        let dot = ac.dot(ao);
        if bt_fuzzy_zero(dot) || dot > 0.0 {
            // C is already in place
            simplex.set(1, &a);
            simplex.set_size(2);
            *dir = triple_cross(ac, ao, ac);
        } else {
            let dot = ab.dot(ao);
            if bt_fuzzy_zero(dot) || dot > 0.0 {
                simplex.set(0, &b);
                simplex.set(1, &a);
                simplex.set_size(2);
                *dir = triple_cross(ab, ao, ab);
            } else {
                simplex.set(0, &a);
                simplex.set_size(1);
                *dir = ao;
            }
        }
    } else {
        let tmp = ab.cross(abc);
        let dot = tmp.dot(ao);
        if bt_fuzzy_zero(dot) || dot > 0.0 {
            let dot = ab.dot(ao);
            if bt_fuzzy_zero(dot) || dot > 0.0 {
                simplex.set(0, &b);
                simplex.set(1, &a);
                simplex.set_size(2);
                *dir = triple_cross(ab, ao, ab);
            } else {
                simplex.set(0, &a);
                simplex.set_size(1);
                *dir = ao;
            }
        } else {
            let dot = abc.dot(ao);
            if bt_fuzzy_zero(dot) || dot > 0.0 {
                *dir = abc;
            } else {
                // swap B and C, and search the other way along the normal
                simplex.set(0, &b);
                simplex.set(1, &c);
                *dir = abc * -1.0;
            }
        }
    }

    0
}

/// `btDoSimplex4` (`btGjkPairDetector.cpp:563-676`).
///
/// The tetrahedron is the only simplex that can enclose the origin, so this is
/// also where the intersection test lives: `-1` for a flat tetrahedron, `1`
/// for the origin on a face or inside, and otherwise the farthest vertex is
/// dropped and [`do_simplex3`] finishes the job.
fn do_simplex4(simplex: &mut CcdSimplex, dir: &mut Vec3) -> i32 {
    let a = simplex.last_point();
    let b = simplex.point(2);
    let c = simplex.point(1);
    let d = simplex.point(0);

    // check if the tetrahedron is really a tetrahedron (has volume > 0)
    let dist = vec3_point_tri_dist2(a.v, b.v, c.v, d.v, None);
    if bt_fuzzy_zero(dist) {
        return -1;
    }

    // check if the origin lies on one of the tetrahedron's faces
    for (p, q, r) in [
        (a.v, b.v, c.v),
        (a.v, c.v, d.v),
        (a.v, b.v, d.v),
        (b.v, c.v, d.v),
    ] {
        let dist = vec3_point_tri_dist2(Vec3::zero(), p, q, r, None);
        if bt_fuzzy_zero(dist) {
            return 1;
        }
    }

    let ao = a.v * -1.0;
    let ab = b.v - a.v;
    let ac = c.v - a.v;
    let ad = d.v - a.v;
    let abc = ab.cross(ac);
    let acd = ac.cross(ad);
    let adb = ad.cross(ab);

    // side of B, C, D relative to the planes ACD, ADB and ABC respectively
    let b_on_acd = ccd_sign(acd.dot(ab));
    let c_on_adb = ccd_sign(adb.dot(ac));
    let d_on_abc = ccd_sign(abc.dot(ad));

    // whether the origin is on the same side of each as B, C, D
    let ab_o = ccd_sign(acd.dot(ao)) == b_on_acd;
    let ac_o = ccd_sign(adb.dot(ao)) == c_on_adb;
    let ad_o = ccd_sign(abc.dot(ao)) == d_on_abc;

    if ab_o && ac_o && ad_o {
        // origin is in the tetrahedron
        return 1;
    } else if !ab_o {
        // B is the farthest from the origin: drop it, D and C are in place
        simplex.set(2, &a);
        simplex.set_size(3);
    } else if !ac_o {
        // C is the farthest
        simplex.set(1, &d);
        simplex.set(0, &b);
        simplex.set(2, &a);
        simplex.set_size(3);
    } else {
        // (!ad_o)
        simplex.set(0, &c);
        simplex.set(1, &b);
        simplex.set(2, &a);
        simplex.set_size(3);
    }

    do_simplex3(simplex, dir)
}

/// `btDoSimplex` (`btGjkPairDetector.cpp:678-695`).
fn do_simplex(simplex: &mut CcdSimplex, dir: &mut Vec3) -> i32 {
    match simplex.size() {
        2 => do_simplex2(simplex, dir),
        3 => do_simplex3(simplex, dir),
        // 4 -- the tetrahedron, which is the only shape that can encapsulate
        // the origin, so btDoSimplex4 also contains the test on it
        _ => do_simplex4(simplex, dir),
    }
}

/// `btConvexPenetrationDepthSolver*`.
///
/// Upstream this is a pointer to an abstract base with two implementations;
/// only `btGjkEpaPenetrationDepthSolver` is ported, and the detector branches
/// on the pointer being null three times, so what has to survive is the
/// presence or absence rather than the dispatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PenetrationDepthSolver {
    /// `btGjkEpaPenetrationDepthSolver` -- see [`crate::pen_depth`].
    GjkEpa,
}

/// `btGjkPairDetector`.
pub struct GjkPairDetector<'a> {
    /// `m_cachedSeparatingAxis`, seeded to `(0, 1, 0)` by both constructors
    /// and reset to the same value at the top of every query.
    cached_separating_axis: Vec3,
    penetration_depth_solver: Option<PenetrationDepthSolver>,
    simplex_solver: &'a mut VoronoiSimplexSolver,
    minkowski_a: &'a dyn ConvexShape,
    minkowski_b: &'a dyn ConvexShape,
    margin_a: Scalar,
    margin_b: Scalar,
    ignore_margin: bool,
    cached_separating_distance: Scalar,

    /// `m_lastUsedMethod` -- see the module docs.
    pub last_used_method: i32,
    /// `m_curIter`.
    pub cur_iter: i32,
    /// `m_degenerateSimplex` -- which exit the classic GJK loop took, `0` if
    /// it did not take a degenerate one.
    pub degenerate_simplex: i32,
    /// `m_catchDegeneracies`, `1` from both constructors. Zeroing it stops the
    /// detector sending a degenerate-but-shallow result to EPA.
    pub catch_degeneracies: i32,
    /// `m_fixContactNormalDirection`, `1` from both constructors.
    ///
    /// Upstream never reads it. The direction-fixing block at the end of
    /// `getClosestPointsNonVirtual` is guarded by a literal `if (1)`, not by
    /// this field, so setting it to `0` changes nothing -- reproduced rather
    /// than tidied, because a caller that sets it is expressing an intent the
    /// code does not honour and hiding the field would hide that.
    pub fix_contact_normal_direction: i32,
}

impl<'a> GjkPairDetector<'a> {
    /// `btGjkPairDetector(objectA, objectB, simplexSolver, penetrationDepthSolver)`
    /// -- the four-argument constructor, which reads the margins off the
    /// shapes.
    ///
    /// The six-argument overload exists for `btConvexConvexAlgorithm`'s
    /// perturbation loop, which passes shape types and margins captured before
    /// the shapes were swapped; it is not ported until that caller is.
    pub fn new(
        object_a: &'a dyn ConvexShape,
        object_b: &'a dyn ConvexShape,
        simplex_solver: &'a mut VoronoiSimplexSolver,
        penetration_depth_solver: Option<PenetrationDepthSolver>,
    ) -> Self {
        Self {
            cached_separating_axis: Vec3::new(0.0, 1.0, 0.0),
            penetration_depth_solver,
            simplex_solver,
            minkowski_a: object_a,
            minkowski_b: object_b,
            margin_a: object_a.margin(),
            margin_b: object_b.margin(),
            ignore_margin: false,
            cached_separating_distance: 0.0,
            last_used_method: -1,
            cur_iter: 0,
            degenerate_simplex: 0,
            catch_degeneracies: 1,
            fix_contact_normal_direction: 1,
        }
    }

    /// `setCachedSeparatingAxis`.
    pub const fn set_cached_separating_axis(&mut self, separating_axis: Vec3) {
        self.cached_separating_axis = separating_axis;
    }

    /// `getCachedSeparatingAxis`.
    #[must_use]
    pub const fn cached_separating_axis(&self) -> Vec3 {
        self.cached_separating_axis
    }

    /// `getCachedSeparatingDistance`.
    #[must_use]
    pub const fn cached_separating_distance(&self) -> Scalar {
        self.cached_separating_distance
    }

    /// `setIgnoreMargin` -- "don't use setIgnoreMargin, it's for Bullet's
    /// internal use". The continuous path is that internal use: the cast hulls
    /// carry their sweep in the shape, so the margins are zeroed here.
    pub const fn set_ignore_margin(&mut self, ignore_margin: bool) {
        self.ignore_margin = ignore_margin;
    }

    /// `getClosestPoints` -- forwards, ignoring `swapResults`, exactly as
    /// upstream does.
    pub fn get_closest_points(&mut self, input: &ClosestPointInput, output: &mut dyn Result) {
        self.get_closest_points_non_virtual(input, output);
    }

    /// `getClosestPointsNonVirtual` (`btGjkPairDetector.cpp:686-1183`).
    ///
    /// Both shapes are moved so that the midpoint of their origins sits at the
    /// world origin before anything is computed, and the offset is added back
    /// only to the contact point. That is not a tidy-up: GJK's termination
    /// tests are absolute, so a pair a long way from the origin loses the
    /// precision the tests need.
    /// Kept as one function, as upstream has it: its exits are only meaningful
    /// against the state the ones above them left behind.
    pub fn get_closest_points_non_virtual(
        &mut self,
        input: &ClosestPointInput,
        output: &mut dyn Result,
    ) {
        self.cached_separating_distance = 0.0;

        let mut distance: Scalar = 0.0;
        let mut normal_in_b = Vec3::zero();

        // `btVector3 pointOnA, pointOnB;` -- default-constructed, which for
        // btVector3 means uninitialized. Every path that reads `pointOnB`
        // either writes it first or is guarded by `isValid`, so zeroing is
        // only a deviation where upstream would have been reading its own
        // stack.
        //
        // `pointOnA` is not carried at all. Upstream writes it five times
        // (`compute_points`, the `-= m_cachedSeparatingAxis * (marginA / s)`
        // at :991, and three assignments in the EPA branch at :1050, :1081 and
        // :1083) and never reads it: only `pointOnB` reaches `addContactPoint`
        // at :1176, and the distance the EPA branch computes uses
        // `tmpPointOnA`, not `pointOnA`. All five writes are pure, so dropping
        // the variable removes no arithmetic that any caller can observe.
        // `compute_points` is still called for its effect on the simplex
        // solver; only its first output is discarded.
        let mut point_on_b = Vec3::zero();
        let mut local_trans_a = input.transform_a;
        let mut local_trans_b = input.transform_b;
        let position_offset = (local_trans_a.origin + local_trans_b.origin) * 0.5;
        local_trans_a.origin -= position_offset;
        local_trans_b.origin -= position_offset;

        let mut margin_a = self.margin_a;
        let mut margin_b = self.margin_b;

        // for CCD we don't use margins
        if self.ignore_margin {
            margin_a = 0.0;
            margin_b = 0.0;
        }

        self.cur_iter = 0;
        self.cached_separating_axis = Vec3::new(0.0, 1.0, 0.0);

        let mut is_valid = false;
        let mut check_simplex = false;
        let check_penetration = true;
        self.degenerate_simplex = 0;

        self.last_used_method = -1;
        let mut status = -2;
        let mut org_normal_in_b = Vec3::zero();
        let margin = margin_a + margin_b;

        let mut squared_distance = BT_LARGE_FLOAT;

        // The libCCD pre-pass. See the module docs: its only output is
        // `status`, and `status == 0` alone can send the pair to EPA below.
        {
            let mut simplex = CcdSimplex::new();
            let mut dir = Vec3::new(1.0, 0.0, 0.0);

            let (mut sup_a_world, mut sup_b_world, mut last_sup_v) = compute_support(
                self.minkowski_a,
                &local_trans_a,
                self.minkowski_b,
                &local_trans_b,
                dir,
            );

            let mut last = SupportVector {
                v: last_sup_v,
                v1: sup_a_world,
                v2: sup_b_world,
            };
            simplex.add(&last);

            dir = -last_sup_v;

            for _ in 0..GJK_MAX_ITER {
                (sup_a_world, sup_b_world, last_sup_v) = compute_support(
                    self.minkowski_a,
                    &local_trans_a,
                    self.minkowski_b,
                    &local_trans_b,
                    dir,
                );

                // If the farthest point of the Minkowski difference along
                // `dir` is before the origin the objects cannot intersect.
                let delta = last_sup_v.dot(dir);
                if delta < 0.0 {
                    // no intersection, besides margin
                    status = -1;
                    break;
                }

                last.v = last_sup_v;
                last.v1 = sup_a_world;
                last.v2 = sup_b_world;
                simplex.add(&last);

                let do_simplex_res = do_simplex(&mut simplex, &mut dir);
                if do_simplex_res == 1 {
                    status = 0; // intersection found
                    break;
                } else if do_simplex_res == -1 {
                    status = -1; // intersection not found
                    break;
                }

                // Three separate zero tests on `dir`, and only the last two
                // break. The first sets `status = -1` and falls through, so a
                // `dir` inside btFuzzyZero's band but with length2 above
                // SIMD_EPSILON keeps iterating with the status already set --
                // reproduced, since a later iteration can overwrite it.
                if bt_fuzzy_zero(dir.dot(dir)) {
                    status = -1;
                }

                if dir.length2() < SIMD_EPSILON {
                    // no intersection, besides margin
                    status = -1;
                    break;
                }

                if dir.fuzzy_zero() {
                    status = -1;
                    break;
                }
            }
        }

        self.simplex_solver.reset();

        // The classic GJK loop over btVoronoiSimplexSolver.
        loop {
            let separating_axis_in_a = local_trans_a
                .basis
                .transposed_mul_vec(-self.cached_separating_axis);
            let separating_axis_in_b = local_trans_b
                .basis
                .transposed_mul_vec(self.cached_separating_axis);

            let p_in_a = self
                .minkowski_a
                .local_get_supporting_vertex_without_margin(separating_axis_in_a);
            let q_in_b = self
                .minkowski_b
                .local_get_supporting_vertex_without_margin(separating_axis_in_b);

            let p_world = local_trans_a.transform_point(p_in_a);
            let q_world = local_trans_b.transform_point(q_in_b);

            let w = p_world - q_world;
            let delta = self.cached_separating_axis.dot(w);

            // potential exit, they don't overlap
            if (delta > 0.0) && (delta * delta > squared_distance * input.maximum_distance_squared)
            {
                self.degenerate_simplex = 10;
                check_simplex = true;
                break;
            }

            // exit 0: the new point is already in the simplex, or we didn't
            // come any closer
            if self.simplex_solver.in_simplex(w) {
                self.degenerate_simplex = 1;
                check_simplex = true;
                break;
            }

            // are we getting any closer?
            let f0 = squared_distance - delta;
            let f1 = squared_distance * REL_ERROR2;

            if f0 <= f1 {
                self.degenerate_simplex = if f0 <= 0.0 { 2 } else { 11 };
                check_simplex = true;
                break;
            }

            self.simplex_solver.add_vertex(w, p_world, q_world);

            // calculate the closest point to the origin (update vector v)
            let (new_cached_separating_axis, ok) = self.simplex_solver.closest();
            if !ok {
                self.degenerate_simplex = 3;
                check_simplex = true;
                break;
            }

            if new_cached_separating_axis.length2() < REL_ERROR2 {
                self.cached_separating_axis = new_cached_separating_axis;
                self.degenerate_simplex = 6;
                check_simplex = true;
                break;
            }

            let previous_squared_distance = squared_distance;
            squared_distance = new_cached_separating_axis.length2();

            // are we getting any closer?
            if previous_squared_distance - squared_distance
                <= SIMD_EPSILON * previous_squared_distance
            {
                check_simplex = true;
                self.degenerate_simplex = 12;
                break;
            }

            self.cached_separating_axis = new_cached_separating_axis;

            // degeneracy, this is typically due to invalid/uninitialized world
            // transforms for a btCollisionObject
            //
            // `if (m_curIter++ > gGjkMaxIter)` -- post-increment, so the
            // comparison sees the count *before* this iteration and the loop
            // runs one more time than a pre-increment would.
            let iters_before = self.cur_iter;
            self.cur_iter += 1;
            if iters_before > GJK_MAX_ITER {
                break;
            }

            if self.simplex_solver.full_simplex() {
                self.degenerate_simplex = 13;
                break;
            }
        }

        if check_simplex {
            (_, point_on_b) = self.simplex_solver.compute_points();
            normal_in_b = self.cached_separating_axis;

            let len_sqr = self.cached_separating_axis.length2();

            // valid normal
            if len_sqr < REL_ERROR2 {
                self.degenerate_simplex = 5;
            }
            if len_sqr > SIMD_EPSILON * SIMD_EPSILON {
                let rlen = 1.0 / len_sqr.sqrt();
                normal_in_b = normal_in_b * rlen; // normalize

                let s = squared_distance.sqrt();

                // upstream also does `pointOnA -= m_cachedSeparatingAxis *
                // (marginA / s)` here; see the declaration above on why the
                // A-side point is not carried
                point_on_b += self.cached_separating_axis * (margin_b / s);
                distance = (1.0 / rlen) - margin;
                is_valid = true;
                org_normal_in_b = normal_in_b;

                self.last_used_method = 1;
            } else {
                self.last_used_method = 2;
            }
        }

        let catch_degenerate_penetration_case = self.catch_degeneracies != 0
            && self.penetration_depth_solver.is_some()
            && self.degenerate_simplex != 0
            && ((distance + margin) < GJK_EPA_PENETRATION_TOLERANCE);

        if (check_penetration && (!is_valid || catch_degenerate_penetration_case)) || (status == 0)
        {
            // penetration case -- if there is no way to handle penetrations,
            // bail out
            if self.penetration_depth_solver.is_some() {
                self.cached_separating_axis = Vec3::zero();

                // Upstream passes `m_cachedSeparatingAxis` in by reference as
                // `v`; `calcPenDepth` opens with `(void)v` and only ever
                // writes it, so a value round-trip is the same program.
                let mut pd = PenDepth::default();
                let is_valid2 = calc_pen_depth(
                    self.simplex_solver,
                    self.minkowski_a,
                    self.minkowski_b,
                    &local_trans_a,
                    &local_trans_b,
                    &mut pd,
                );
                self.cached_separating_axis = pd.v;
                let (tmp_point_on_a, tmp_point_on_b) = (pd.witness_on_a, pd.witness_on_b);

                if self.cached_separating_axis.length2() != 0.0 {
                    if is_valid2 {
                        let mut tmp_normal_in_b = tmp_point_on_b - tmp_point_on_a;
                        let mut len_sqr = tmp_normal_in_b.length2();
                        if len_sqr <= (SIMD_EPSILON * SIMD_EPSILON) {
                            tmp_normal_in_b = self.cached_separating_axis;
                            len_sqr = self.cached_separating_axis.length2();
                        }

                        if len_sqr > (SIMD_EPSILON * SIMD_EPSILON) {
                            tmp_normal_in_b /= len_sqr.sqrt();
                            let distance2 = -(tmp_point_on_a - tmp_point_on_b).length();
                            self.last_used_method = 3;
                            // only replace valid penetrations when the result
                            // is deeper
                            if !is_valid || (distance2 < distance) {
                                distance = distance2;
                                point_on_b = tmp_point_on_b;
                                normal_in_b = tmp_normal_in_b;
                                is_valid = true;
                            } else {
                                self.last_used_method = 8;
                            }
                        } else {
                            self.last_used_method = 9;
                        }
                    } else if self.cached_separating_axis.length2() > 0.0 {
                        // The other degenerate case: the initial GJK reports a
                        // degeneracy, EPA reports no penetration, and the
                        // second GJK (on the supporting vector without margin)
                        // reports a valid positive distance. Use the second
                        // GJK's result instead of failing.
                        let distance2 = (tmp_point_on_a - tmp_point_on_b).length() - margin;
                        // only replace valid distances when the distance is
                        // less
                        if !is_valid || (distance2 < distance) {
                            distance = distance2;
                            point_on_b = tmp_point_on_b;
                            point_on_b += self.cached_separating_axis * margin_b;
                            normal_in_b = self.cached_separating_axis;
                            normal_in_b = normal_in_b.normalize();

                            is_valid = true;
                            self.last_used_method = 6;
                        } else {
                            self.last_used_method = 5;
                        }
                    }
                }
            }
        }

        if is_valid && ((distance < 0.0) || (distance * distance < input.maximum_distance_squared))
        {
            self.cached_separating_axis = normal_in_b;
            self.cached_separating_distance = distance;

            // The EPA penetration solver can report a penetration whose normal
            // points the opposite way from the segment joining the two contact
            // points. Upstream detects that and reverts the normal; its own
            // comment calls it a degeneracy still to be tracked down.
            //
            // The three probes differ in more than their axis -- `d1` scores
            // along `-normalInB` where `d0` and `d2` score along the axis they
            // probed, and `d0` takes its separating axes from the *input*
            // bases rather than the recentred ones -- so they are written out
            // one by one as upstream has them rather than folded into a helper
            // that would have to carry those differences as flags.
            let d2 = {
                let separating_axis_in_a = local_trans_a.basis.transposed_mul_vec(-org_normal_in_b);
                let separating_axis_in_b = local_trans_b.basis.transposed_mul_vec(org_normal_in_b);

                let p_in_a = self
                    .minkowski_a
                    .local_get_supporting_vertex_without_margin(separating_axis_in_a);
                let q_in_b = self
                    .minkowski_b
                    .local_get_supporting_vertex_without_margin(separating_axis_in_b);

                let w =
                    local_trans_a.transform_point(p_in_a) - local_trans_b.transform_point(q_in_b);
                org_normal_in_b.dot(w) - margin
            };

            let d1 = {
                let separating_axis_in_a = local_trans_a.basis.transposed_mul_vec(normal_in_b);
                let separating_axis_in_b = local_trans_b.basis.transposed_mul_vec(-normal_in_b);

                let p_in_a = self
                    .minkowski_a
                    .local_get_supporting_vertex_without_margin(separating_axis_in_a);
                let q_in_b = self
                    .minkowski_b
                    .local_get_supporting_vertex_without_margin(separating_axis_in_b);

                let w =
                    local_trans_a.transform_point(p_in_a) - local_trans_b.transform_point(q_in_b);
                (-normal_in_b).dot(w) - margin
            };

            // `input.m_transformA.getBasis()`, not `localTransA`'s -- the two
            // hold the same rotation here, since only the origins were moved,
            // but the line is transcribed as upstream wrote it.
            let d0 = {
                let separating_axis_in_a = input.transform_a.basis.transposed_mul_vec(-normal_in_b);
                let separating_axis_in_b = input.transform_b.basis.transposed_mul_vec(normal_in_b);

                let p_in_a = self
                    .minkowski_a
                    .local_get_supporting_vertex_without_margin(separating_axis_in_a);
                let q_in_b = self
                    .minkowski_b
                    .local_get_supporting_vertex_without_margin(separating_axis_in_b);

                let w =
                    local_trans_a.transform_point(p_in_a) - local_trans_b.transform_point(q_in_b);
                normal_in_b.dot(w) - margin
            };

            if d1 > d0 {
                self.last_used_method = 10;
                normal_in_b = normal_in_b * -1.0;
            }

            if org_normal_in_b.length2() != 0.0 && d2 > d0 && d2 > d1 && d2 > distance {
                normal_in_b = org_normal_in_b;
                distance = d2;
            }

            output.add_contact_point(normal_in_b, point_on_b + position_offset, distance);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::discrete_detector::StorageResult;
    use crate::probe_fixture::{IDENTITY, at, diff, diff_vec3, probe_shapes, rot60_at, row};
    use crate::shapes::BoxShape;

    /// A separated pair takes the plain GJK exit -- `m_lastUsedMethod == 1`,
    /// no degeneracy -- and reports the face gap, not the centre distance.
    #[test]
    fn separated_boxes_take_the_plain_gjk_exit() {
        let mut a = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
        let mut b = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
        a.set_margin(0.0);
        b.set_margin(0.0);

        let mut solver = VoronoiSimplexSolver::new();
        let mut detector =
            GjkPairDetector::new(&a, &b, &mut solver, Some(PenetrationDepthSolver::GjkEpa));
        let input = ClosestPointInput {
            transform_a: IDENTITY,
            transform_b: at(1.5, 0.0, 0.0),
            ..ClosestPointInput::new()
        };
        let mut out = StorageResult::new();
        detector.get_closest_points(&input, &mut out);

        assert_eq!(detector.last_used_method, 1);
        assert!((out.distance - 0.5).abs() < 1e-6, "{out:?}");
    }

    /// With no penetration-depth solver the detector cannot answer a
    /// penetrating pair at all: it emits nothing, leaving the sink at its
    /// `BT_LARGE_FLOAT` seed. That is the branch upstream guards with
    /// `if (m_penetrationDepthSolver)`, and it is reachable from the port's
    /// public API because the field is an `Option`.
    #[test]
    fn without_a_penetration_solver_an_overlap_emits_no_contact() {
        let mut a = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
        let mut b = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
        a.set_margin(0.0);
        b.set_margin(0.0);

        let mut solver = VoronoiSimplexSolver::new();
        let mut detector = GjkPairDetector::new(&a, &b, &mut solver, None);
        let input = ClosestPointInput {
            transform_a: IDENTITY,
            transform_b: at(0.5, 0.0, 0.0),
            ..ClosestPointInput::new()
        };
        let mut out = StorageResult::new();
        detector.get_closest_points(&input, &mut out);

        assert_eq!(out.distance, BT_LARGE_FLOAT);
    }

    /// The `g_*` rows of `tools/bullet-epa-reference/build.sh`'s stdout,
    /// verbatim: the real `btGjkPairDetector::getClosestPointsNonVirtual` from
    /// bullet3 @ `7dee3436` on the pairs below.
    ///
    /// The three counters are in the fixture because they are what separates
    /// exits that agree on the number. `g_box_box_deep` and `g_margin_overlap`
    /// both report a penetration, but the first reached it through EPA
    /// (`method 3`, `degenerate 5`) and the second through the plain GJK exit
    /// with a degenerate simplex (`method 1`, `degenerate 2`); a port that
    /// took the other route on either would still match every float.
    ///
    /// Fields: `name|lastUsedMethod|degenerateSimplex|curIter|distance|
    /// normalOnSurfaceB xyz|closestPointInB xyz|cachedSeparatingDistance|
    /// cachedSeparatingAxis xyz`.
    const BULLET_REFERENCE: &str = "\
g_box_box_deep|3|5|1|-0.2886751|-0.577350318|0.577350259|0.577350318|0.333333343|0.333333313|0.49999997|-0.2886751|-0.577350318|0.577350259|0.577350318
g_box_box_shallow|3|5|2|-0.0577350222|-0.577350259|0.577350259|0.577350259|0.466666669|0.466666669|0.49999997|-0.0577350222|-0.577350259|0.577350259|0.577350259
g_box_box_touching|3|5|2|-0|-1|-0|-0|0.5|0.5|0.5|-0|-1|-0|-0
g_box_box_separated|1|2|3|0.5|-1|0|0|1|0|0.5|0.5|-1|0|0
g_box_box_offset|3|5|3|-0.399999976|-1|0|0|0.100000024|0.175000042|-0.100000009|-0.399999976|-1|0|0
g_box_box_rot60|3|5|3|-0.402492225|-0.89442724|0|-0.44721362|0.139999986|0.269999981|0.319999993|-0.402492225|-0.89442724|0|-0.44721362
g_margin_overlap|1|2|3|-0.0500000268|-1|0|0|0.449999988|0|0.460000008|-0.0500000268|-1|0|0
g_margin_separated|1|2|3|0.200000033|-0.99999994|0|0|0.700000048|0|0.460000008|0.200000033|-0.99999994|0|0
g_sphere_sphere|1|1|1|-0.300000012|-1|0|0|0.199999988|0|0|-0.300000012|-1|0|0
g_sphere_box|1|1|4|-0.199999988|-1|0|0|0.100000024|-3.7252903e-09|0|-0.199999988|-1|0|0
g_cyl_box|3|5|3|-0.199999467|-1|5.58795037e-08|0|0.0999999791|5.2344054e-05|0.133333325|-0.199999467|-1|5.58795037e-08|0
g_cyl_cyl_rot60|3|5|3|-0.331661344|-0.447215855|-0.894426048|1.16814999e-05|-0.0131956488|-0.0288075171|-0.277646482|-0.331661344|-0.447215855|-0.894426048|1.16814999e-05
g_cone_box|3|5|3|-0.131240785|-0.954480231|-3.6900704e-07|-0.298274398|0.0500000119|5.99995255e-05|-0.199999988|-0.131240785|-0.954480231|-3.6900704e-07|-0.298274398
g_cone_sphere|1|1|3|-0.0941902846|-0.954479992|0|-0.298274994|-0.0863439962|0|0.360517502|-0.0941902846|-0.954479992|0|-0.298274994
g_hull_box|3|5|3|-0.100000024|-1|0|0|0.199999988|0.0142857209|0.0892857164|-0.100000024|-1|0|0
g_hull_sphere_rot60|1|1|3|-0.200000003|-1|-1.86264501e-07|-3.72528994e-08|0.100000009|0.0999999493|0.0499999896|-0.200000003|-1|-1.86264501e-07|-3.72528994e-08
g_coincident|3|5|3|-0.749996066|1.87059049e-06|-1.69520024e-06|0.99999994|-0.000132424393|6.50387738e-05|0.25|-0.749996066|1.87059049e-06|-1.69520024e-06|0.99999994
g_ccd_margin_boxes|1|2|3|0.0299999695|-1|0|0|0.48999998|0|0.460000008|0.0299999695|-1|0|0
g_ccd_sphere_box|1|1|3|0.399999976|-1|0|0|0.399999976|0|0|0.399999976|-1|0|0
g_maxdist_clipped|1|10|1|9.99999984e+17|0|0|0|0|0|0|0|-3|-1|0
g_no_pen_solver|2|5|1|9.99999984e+17|0|0|0|0|0|0|0|0|0|0
g_far_from_origin|3|5|2|-0.057734143|-0.577350318|0.577350318|0.577350318|100.466667|100.466667|100.5|-0.057734143|-0.577350318|0.577350318|0.577350318
g_prepass_flip|10|3|7|-0|0|-0.800000012|-0.600000024|-0.400000006|0.699999988|0.25|-0|-0|0.800000012|0.600000024
g_prepass_rescue|6|3|7|0.000100681136|0|-0.98037529|-0.197140679|-0.199970514|0.700098753|0.250019848|0.000100681136|0|-0.98037529|-0.197140679
g_prepass_margins|3|3|2|-0.499237269|-0.941131949|-0.187180549|-0.281485468|-0.00013011694|0.000260055065|0.000260040164|-0.499237269|-0.941131949|-0.187180549|-0.281485468
g_prepass_degen12|3|12|9|-0.499593139|-0.16625002|-0.00247252872|-0.986080587|0.000305563211|-0.000475483481|1.1920929e-07|-0.499593139|-0.16625002|-0.00247252872|-0.986080587
g_normal_reverted|10|5|2|-0.777817488|-0.707106829|-0.707106829|-0|0.5|0.600000024|0.0500000119|-0.777817488|0.707106829|0.707106829|0
g_epa_not_deeper|8|5|7|-0.499790788|-0.99766922|0.0682332888|-0.000641038292|0.000208720565|-1.42749632e-05|1.34110451e-07|-0.499790788|-0.99766922|0.0682332888|-0.000641038292
g_second_gjk_rescue|6|5|5|0.00031802064|-0.999999821|-0.000357276673|0.000562271453|0.300000042|0.0126247406|0.349999905|0.00031802064|-0.999999821|-0.000357276673|0.000562271453
g_rescue_rejected|5|5|6|1.88552679e-06|-0.999718964|-0.0237087496|0|0.500001907|0.499424517|0.349999994|1.88552679e-06|-0.999718964|-0.0237087496|0
g_rescue_rot60|6|5|9|0.000131919485|-0.656503081|-0.754323423|-0.000112956484|0.197666347|0.225801736|-0.2347655|0.000131919485|-0.656503081|-0.754323423|-0.000112956484
g_degen11|1|11|4|0.100000016|-1|-7.45057918e-08|0|0.600000024|3.7252903e-09|0.0500000007|0.100000016|-1|-7.45057918e-08|0
g_degen12|1|12|5|0.044721365|-0.894427419|-0.447213292|-1.66600032e-07|0.540000021|0.519999981|0.320000023|0.044721365|-0.894427419|-0.447213292|-1.66600032e-07
g_degen3|1|3|4|0.100000016|-1|0|-1.49011584e-07|0.600000024|0|0.100000009|0.100000016|-1|0|-1.49011584e-07
";

    /// One parsed row of [`BULLET_REFERENCE`].
    struct Reference {
        last_used_method: i32,
        degenerate_simplex: i32,
        cur_iter: i32,
        distance: Scalar,
        normal_on_surface_b: Vec3,
        closest_point_in_b: Vec3,
        cached_separating_distance: Scalar,
        cached_separating_axis: Vec3,
    }

    fn reference(name: &str) -> Reference {
        let f = row(BULLET_REFERENCE, name, 15);
        let i = |k: usize| -> i32 {
            f[k].parse()
                .unwrap_or_else(|e| panic!("{name}: field {k} ({:?}): {e}", f[k]))
        };
        let n = |k: usize| -> Scalar {
            f[k].parse()
                .unwrap_or_else(|e| panic!("{name}: field {k} ({:?}): {e}", f[k]))
        };
        Reference {
            last_used_method: i(1),
            degenerate_simplex: i(2),
            cur_iter: i(3),
            distance: n(4),
            normal_on_surface_b: Vec3::new(n(5), n(6), n(7)),
            closest_point_in_b: Vec3::new(n(8), n(9), n(10)),
            cached_separating_distance: n(11),
            cached_separating_axis: Vec3::new(n(12), n(13), n(14)),
        }
    }

    /// Every `getClosestPointsNonVirtual` row, against the port.
    #[test]
    fn bullet_reference_closest_points() {
        let (unit_box, flat_box, margin_box, sphere, small_sphere, cyl, cone, hull) =
            probe_shapes();
        let mut bad = Vec::new();

        let mut case = |name: &str,
                        a: &dyn ConvexShape,
                        ta: &Transform,
                        b: &dyn ConvexShape,
                        tb: &Transform,
                        ignore_margin: bool,
                        maximum_distance_squared: Scalar,
                        use_pen_solver: bool| {
            let mut solver = VoronoiSimplexSolver::new();
            let mut detector = GjkPairDetector::new(
                a,
                b,
                &mut solver,
                use_pen_solver.then_some(PenetrationDepthSolver::GjkEpa),
            );
            detector.set_ignore_margin(ignore_margin);

            let input = ClosestPointInput {
                transform_a: *ta,
                transform_b: *tb,
                maximum_distance_squared,
            };
            let mut out = StorageResult::new();
            detector.get_closest_points_non_virtual(&input, &mut out);

            let want = reference(name);
            for (field, got, expected) in [
                (
                    "lastUsedMethod",
                    detector.last_used_method,
                    want.last_used_method,
                ),
                (
                    "degenerateSimplex",
                    detector.degenerate_simplex,
                    want.degenerate_simplex,
                ),
                ("curIter", detector.cur_iter, want.cur_iter),
            ] {
                if got != expected {
                    bad.push(format!("{name}.{field}: port {got}, bullet {expected}"));
                }
            }
            diff(&mut bad, name, "distance", out.distance, want.distance);
            diff_vec3(
                &mut bad,
                name,
                "normalOnSurfaceB",
                out.normal_on_surface_b,
                want.normal_on_surface_b,
            );
            diff_vec3(
                &mut bad,
                name,
                "closestPointInB",
                out.closest_point_in_b,
                want.closest_point_in_b,
            );
            diff(
                &mut bad,
                name,
                "cachedSeparatingDistance",
                detector.cached_separating_distance(),
                want.cached_separating_distance,
            );
            diff_vec3(
                &mut bad,
                name,
                "cachedSeparatingAxis",
                detector.cached_separating_axis(),
                want.cached_separating_axis,
            );
        };

        let big = BT_LARGE_FLOAT;
        case(
            "g_box_box_deep",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(0.5, 0.0, 0.0),
            false,
            big,
            true,
        );
        case(
            "g_box_box_shallow",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(0.9, 0.0, 0.0),
            false,
            big,
            true,
        );
        case(
            "g_box_box_touching",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(1.0, 0.0, 0.0),
            false,
            big,
            true,
        );
        case(
            "g_box_box_separated",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(1.5, 0.0, 0.0),
            false,
            big,
            true,
        );
        case(
            "g_box_box_offset",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(0.6, 0.35, -0.2),
            false,
            big,
            true,
        );
        case(
            "g_box_box_rot60",
            &unit_box,
            &IDENTITY,
            &flat_box,
            &rot60_at(0.7, 0.2, 0.1),
            false,
            big,
            true,
        );
        case(
            "g_margin_overlap",
            &margin_box,
            &IDENTITY,
            &margin_box,
            &at(0.95, 0.0, 0.0),
            false,
            big,
            true,
        );
        case(
            "g_margin_separated",
            &margin_box,
            &IDENTITY,
            &margin_box,
            &at(1.2, 0.0, 0.0),
            false,
            big,
            true,
        );
        case(
            "g_sphere_sphere",
            &sphere,
            &IDENTITY,
            &sphere,
            &at(0.7, 0.0, 0.0),
            false,
            big,
            true,
        );
        case(
            "g_sphere_box",
            &small_sphere,
            &IDENTITY,
            &unit_box,
            &at(0.6, 0.1, 0.0),
            false,
            big,
            true,
        );
        case(
            "g_cyl_box",
            &cyl,
            &IDENTITY,
            &flat_box,
            &at(0.5, 0.1, 0.2),
            false,
            big,
            true,
        );
        case(
            "g_cyl_cyl_rot60",
            &cyl,
            &IDENTITY,
            &cyl,
            &rot60_at(0.4, 0.1, 0.0),
            false,
            big,
            true,
        );
        case(
            "g_cone_box",
            &cone,
            &IDENTITY,
            &unit_box,
            &at(0.55, 0.1, 0.3),
            false,
            big,
            true,
        );
        case(
            "g_cone_sphere",
            &cone,
            &IDENTITY,
            &small_sphere,
            &at(0.2, 0.0, 0.45),
            false,
            big,
            true,
        );
        case(
            "g_hull_box",
            &hull,
            &IDENTITY,
            &unit_box,
            &at(0.7, 0.05, 0.0),
            false,
            big,
            true,
        );
        case(
            "g_hull_sphere_rot60",
            &hull,
            &IDENTITY,
            &small_sphere,
            &rot60_at(0.4, 0.1, 0.05),
            false,
            big,
            true,
        );
        case(
            "g_coincident",
            &margin_box,
            &IDENTITY,
            &flat_box,
            &IDENTITY,
            false,
            big,
            true,
        );
        case(
            "g_ccd_margin_boxes",
            &margin_box,
            &IDENTITY,
            &margin_box,
            &at(0.95, 0.0, 0.0),
            true,
            big,
            true,
        );
        case(
            "g_ccd_sphere_box",
            &sphere,
            &IDENTITY,
            &unit_box,
            &at(0.9, 0.0, 0.0),
            true,
            big,
            true,
        );
        case(
            "g_maxdist_clipped",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(3.0, 0.0, 0.0),
            false,
            1.0,
            true,
        );
        case(
            "g_no_pen_solver",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(0.5, 0.0, 0.0),
            false,
            big,
            false,
        );
        case(
            "g_far_from_origin",
            &unit_box,
            &at(100.0, 100.0, 100.0),
            &unit_box,
            &at(100.9, 100.0, 100.0),
            false,
            big,
            true,
        );

        case(
            "g_prepass_flip",
            &flat_box,
            &IDENTITY,
            &cone,
            &rot60_at(0.0, 0.8, 0.6),
            false,
            big,
            true,
        );
        case(
            "g_prepass_rescue",
            &flat_box,
            &IDENTITY,
            &cone,
            &rot60_at(0.1, 0.8, 0.6),
            false,
            big,
            true,
        );
        case(
            "g_prepass_margins",
            &sphere,
            &IDENTITY,
            &unit_box,
            &rot60_at(0.7, 0.1, 0.1),
            false,
            big,
            true,
        );
        case(
            "g_prepass_degen12",
            &sphere,
            &IDENTITY,
            &cyl,
            &at(0.3, 0.0, 0.5),
            false,
            big,
            true,
        );
        case(
            "g_normal_reverted",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(0.0, 0.1, 0.1),
            false,
            big,
            true,
        );
        case(
            "g_epa_not_deeper",
            &sphere,
            &IDENTITY,
            &cyl,
            &at(0.3, 0.0, 0.0),
            false,
            big,
            true,
        );
        case(
            "g_second_gjk_rescue",
            &cyl,
            &IDENTITY,
            &unit_box,
            &at(0.8, 0.5, 0.7),
            false,
            big,
            true,
        );
        case(
            "g_rescue_rejected",
            &unit_box,
            &IDENTITY,
            &cyl,
            &at(0.8, 0.5, 0.7),
            false,
            big,
            true,
        );
        case(
            "g_rescue_rot60",
            &cyl,
            &IDENTITY,
            &cone,
            &rot60_at(0.6, 0.3, 0.0),
            false,
            big,
            true,
        );
        case(
            "g_degen11",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(1.1, 0.0, 0.1),
            false,
            big,
            true,
        );
        case(
            "g_degen12",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &rot60_at(1.1, 0.9, 0.7),
            false,
            big,
            true,
        );
        case(
            "g_degen3",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(1.1, 0.0, 0.2),
            false,
            big,
            true,
        );

        assert!(
            bad.is_empty(),
            "{} deviations:\n{}",
            bad.len(),
            bad.join("\n")
        );
    }

    /// `btVec3PointTriDist2`, probed directly rather than through the detector.
    /// Fields: `name|withWitness|withoutWitness|witness xyz`.
    ///
    /// Both arms are recorded because they are not the same arithmetic. With a
    /// witness the distance is recomputed from the witness point in `Scalar`;
    /// without one the in-triangle branch accumulates it in `double` and the
    /// nearest-edge branch takes whatever `vec3_point_segment_dist2` returned.
    /// `t_origin_offset` is the row where the two disagree, which is what makes
    /// forwarding the caller's witness -- rather than substituting a local --
    /// observable at all.
    ///
    /// `t_wide_solve` pins the solve's precision. `u,v,w,p,q,r,s,t` are C
    /// `double` inside this float build, and on the triangles above that costs
    /// nothing -- their barycentrics are exactly representable, so the same
    /// expression in `Scalar` lands on the same floats. This one was searched
    /// for: the two are 40 ulps apart, in the witness's y most of all.
    const TRI_REFERENCE: &str = "\
t_inside|1|1|0|0|0
t_beyond_b|2.25|2.25|1|0|0
t_beyond_c|2.25|2.25|0|1|0
t_behind_x0|2.0625|2.0625|0|0|0
t_past_bc|0.50999999|0.50999999|0.5|0.5|0
t_on_face|0|0|0.25|0.25|0
t_origin_offset|0.104445107|0.10444507|0.103992961|0.289371699|0.0994715393
t_sliver|1|1|1|0|0
t_large|15722.8906|15722.8906|4.81927872|63.2530136|27.1084347
t_wide_solve|0.509997964|0.509998083|0.227071851|-0.0090803951|0.242351681
";

    /// Every `btVec3PointTriDist2` row, against the port.
    ///
    /// The pre-pass this function serves feeds a single bit into the detector,
    /// and a sweep of 230,400 shape/transform cells found only 22 in which that
    /// bit changes the answer -- so the detector's own rows leave the
    /// double-precision barycentric solve inside here effectively unpinned.
    #[test]
    fn bullet_reference_point_tri_dist2() {
        let mut bad = Vec::new();
        let mut case = |name: &str, p: Vec3, x0: Vec3, b: Vec3, c: Vec3| {
            let f = row(TRI_REFERENCE, name, 6);
            let n = |i: usize| -> Scalar {
                f[i].parse()
                    .unwrap_or_else(|e| panic!("{name}: field {i} ({:?}): {e}", f[i]))
            };

            let mut witness = Vec3::zero();
            let with = vec3_point_tri_dist2(p, x0, b, c, Some(&mut witness));
            let without = vec3_point_tri_dist2(p, x0, b, c, None);

            diff(&mut bad, name, "withWitness", with, n(1));
            diff(&mut bad, name, "withoutWitness", without, n(2));
            diff_vec3(
                &mut bad,
                name,
                "witness",
                witness,
                Vec3::new(n(3), n(4), n(5)),
            );
        };

        let (o, ex, ey) = (
            Vec3::zero(),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );
        case("t_inside", Vec3::new(0.0, 0.0, 1.0), o, ex, ey);
        case("t_beyond_b", Vec3::new(2.0, -1.0, 0.5), o, ex, ey);
        case("t_beyond_c", Vec3::new(-1.0, 2.0, -0.5), o, ex, ey);
        case("t_behind_x0", Vec3::new(-1.0, -1.0, 0.25), o, ex, ey);
        case("t_past_bc", Vec3::new(1.0, 1.0, 0.1), o, ex, ey);
        case("t_on_face", Vec3::new(0.25, 0.25, 0.0), o, ex, ey);
        case(
            "t_origin_offset",
            o,
            Vec3::new(-0.3, 0.4, 0.2),
            Vec3::new(0.7, -0.2, 0.9),
            Vec3::new(0.1, 0.6, -0.8),
        );
        case(
            "t_sliver",
            o,
            ex,
            Vec3::new(1.0001, 1e-4, 0.0),
            Vec3::new(1.0002, 2e-4, 1e-5),
        );
        case(
            "t_large",
            Vec3::new(100.0, 100.0, 100.0),
            Vec3::new(-50.0, 0.0, 0.0),
            Vec3::new(50.0, 0.0, 0.0),
            Vec3::new(0.0, 70.0, 30.0),
        );
        case(
            "t_wide_solve",
            Vec3::new(-0.125_722_17, 0.134_502_41, -0.361_733_08),
            Vec3::new(-0.193_585_04, -0.522_099_6, 0.366_083_5),
            Vec3::new(-0.403_536_02, 0.539_090_04, 0.740_928_3),
            Vec3::new(0.750_904_8, -0.421_470_17, -0.161_593_62),
        );

        assert!(
            bad.is_empty(),
            "{} deviations:\n{}",
            bad.len(),
            bad.join("\n")
        );
    }
}
