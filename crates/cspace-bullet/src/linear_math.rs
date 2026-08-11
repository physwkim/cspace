// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2009 Erwin Coumans  http://bullet.googlecode.com
// Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/LinearMath/btScalar.h
//   bullet3/src/LinearMath/btVector3.h
//   bullet3/src/LinearMath/btMatrix3x3.h
//   bullet3/src/LinearMath/btTransform.h
//   bullet3/src/LinearMath/btAabbUtil2.h

//! `btScalar`, `btVector3`, `btMatrix3x3` and `btTransform`, in the
//! single-precision scalar configuration the oracle image's Bullet is built
//! in.
//!
//! # Why this is a port and not a `nalgebra` type alias
//!
//! Every value this crate produces is compared against a C++ Bullet linked
//! into the differential oracle, and the comparison is meant to be *exact*,
//! not "within a tolerance nobody measured". That is only reachable if the
//! arithmetic is the same arithmetic, operation for operation, so the
//! primitives are written out rather than delegated:
//!
//! - [`Vec3::normalize`] multiplies by the reciprocal rather than dividing.
//!   `btVector3::normalize()` is `*this /= length()` and `operator/=` is
//!   `*this *= btScalar(1.0) / s` (`btVector3.h:209-223`), which differs from
//!   a component-wise divide in the last bit.
//! - [`bt_fsel`] treats `0.0` as the *positive* branch, so a support direction
//!   with a zero component still selects the `+` face of a box.
//! - [`SIMD_INFINITY`] is `FLT_MAX`, not an infinity (`btScalar.h:544`). Code
//!   that seeds a running maximum with `-SIMD_INFINITY` is seeding it with
//!   `-f32::MAX`, and the distinction is observable the moment a dot product
//!   is itself infinite.
//!
//! # Which Bullet configuration this reproduces
//!
//! Bullet's headers carry SIMD variants of nearly every operation here, and
//! several of them are not bit-equal to the scalar ones -- `normalize()` under
//! `BT_USE_SSE_IN_API` is an `rsqrt` estimate plus one Newton-Raphson step
//! (`btVector3.h:307-336`), which is not correctly rounded. None of them are
//! live in the build this port is measured against: `btScalar.h:216-244` gates
//! `BT_USE_SSE`/`BT_USE_SIMD_VECTOR3`/`BT_USE_SSE_IN_API` on `__APPLE__` for
//! every non-Windows target, so on Linux/GCC `btVector3` is a plain
//! `btScalar m_floats[4]` and every path here is the scalar one.
//! `btConvexHullShape.cpp` is the one translation unit that defines
//! `BT_USE_SSE_IN_API` itself, and its guard is `_WIN32 || __i386__`
//! (`btConvexHullShape.cpp:16-18`) -- also dead on x86-64 Linux.
//!
//! Scalar `float` arithmetic on x86-64 compiles to SSE2 single-precision
//! instructions, which are IEEE-754 correctly rounded, and GCC's default
//! `-march=x86-64` has no FMA for `-ffp-contract=fast` to contract into. Rust
//! never contracts. So the two languages agree bit for bit on `+ - * / sqrt`,
//! which is the whole vocabulary below.
//!
//! # Not ported
//!
//! `btVector3`'s fourth component. Bullet carries `m_floats[3]` for alignment
//! and uses it as scratch in exactly one place this crate touches --
//! `batchedUnitVectorGetSupportingVertexWithoutMargin` stashes each dot
//! product in `supportVerticesOut[j][3]` (`btConvexHullShape.cpp:75-96`) --
//! and no caller in the CCD path reads it back. Quaternions, Euler
//! conversions, serialization, `rotate`, `angle`, `lerp` and the `btMatrix3x3`
//! eigen/diagonalize helpers are unused by the narrow phase and are absent
//! rather than stubbed.

use core::ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, Neg, Sub, SubAssign};

/// `btScalar`, in the single-precision configuration (`btScalar.h:314`).
///
/// The oracle image's Bullet is built without `BT_USE_DOUBLE_PRECISION`;
/// `sizeof(btScalar) == 4` there, and every constant below takes its
/// single-precision spelling.
pub type Scalar = f32;

/// `SIMD_EPSILON` == `FLT_EPSILON` (`btScalar.h:543`).
pub const SIMD_EPSILON: Scalar = Scalar::EPSILON;

/// `SIMD_INFINITY` == `FLT_MAX` (`btScalar.h:544`) -- **not** an infinity.
///
/// Bullet spells the name as if it were `inf` and defines it as the largest
/// finite float. Every `-SIMD_INFINITY` seed in the narrow phase is therefore
/// `-f32::MAX`, and a comparison against a genuinely infinite dot product goes
/// the other way than the name suggests.
pub const SIMD_INFINITY: Scalar = Scalar::MAX;

/// `BT_LARGE_FLOAT` == `1e18f` (`btScalar.h:316`).
pub const BT_LARGE_FLOAT: Scalar = 1e18;

/// `btFsel(a, b, c)` -- `a >= 0 ? b : c` (`btScalar.h:597-603`).
///
/// The boundary is the point: `a == 0.0` selects `b`, so a support direction
/// with a zero component picks the positive face rather than either one
/// arbitrarily. `-0.0 >= 0.0` is also true in IEEE-754, matching the C++.
#[inline]
#[must_use]
pub fn bt_fsel(a: Scalar, b: Scalar, c: Scalar) -> Scalar {
    if a >= 0.0 { b } else { c }
}

/// `btFuzzyZero(x)` -- `|x| < SIMD_EPSILON` (`btScalar.h:572`). Unlike
/// [`Vec3::fuzzy_zero`] this is not squared, so the two accept different
/// magnitudes; `getClosestPointsNonVirtual` uses both, four lines apart.
#[inline]
#[must_use]
pub fn bt_fuzzy_zero(x: Scalar) -> bool {
    x.abs() < SIMD_EPSILON
}

/// `btVector3`, less the fourth component (see the module docs).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    /// `m_floats[0]`.
    pub x: Scalar,
    /// `m_floats[1]`.
    pub y: Scalar,
    /// `m_floats[2]`.
    pub z: Scalar,
}

impl Vec3 {
    /// `btVector3(x, y, z)`.
    #[inline]
    #[must_use]
    pub const fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self { x, y, z }
    }

    /// `btVector3(0, 0, 0)`.
    #[inline]
    #[must_use]
    pub const fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// `btVector3::dot`.
    #[inline]
    #[must_use]
    pub fn dot(self, other: Self) -> Scalar {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// `btVector3::length2`.
    #[inline]
    #[must_use]
    pub fn length2(self) -> Scalar {
        self.dot(self)
    }

    /// `btVector3::length`.
    #[inline]
    #[must_use]
    pub fn length(self) -> Scalar {
        self.length2().sqrt()
    }

    /// `btVector3::distance2`.
    #[inline]
    #[must_use]
    pub fn distance2(self, other: Self) -> Scalar {
        (other - self).length2()
    }

    /// `btVector3::cross`.
    #[inline]
    #[must_use]
    pub fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    /// `btVector3::normalize`, as the scalar build spells it: `*this /=
    /// length()`, and `operator/=` is `*this *= btScalar(1.0) / s`
    /// (`btVector3.h:209-223`, `:341`).
    ///
    /// Reciprocal-then-multiply, not three divides. The two differ in the last
    /// bit, and this is the version every `getClosestPoints` call in the CCD
    /// path takes.
    #[inline]
    #[must_use]
    pub fn normalize(self) -> Self {
        self * (1.0 / self.length())
    }

    /// `btVector3::safeNormalize` (`btVector3.h:286-299`) -- `(1, 0, 0)` rather
    /// than a division when the vector is too short to normalize. Note the
    /// guard is on `length2()` against `SIMD_EPSILON * SIMD_EPSILON`, so the
    /// shortest vector it will normalize has length `SIMD_EPSILON`.
    #[inline]
    #[must_use]
    pub fn safe_normalize(self) -> Self {
        let l2 = self.length2();
        if l2 >= SIMD_EPSILON * SIMD_EPSILON {
            self / l2.sqrt()
        } else {
            Self::new(1.0, 0.0, 0.0)
        }
    }

    /// `btVector3::fuzzyZero` (`btVector3.h:688-691`) -- a *squared* length
    /// test, so the radius it accepts is `SIMD_EPSILON`, not `SIMD_EPSILON`
    /// squared.
    #[inline]
    #[must_use]
    pub fn fuzzy_zero(self) -> bool {
        self.length2() < SIMD_EPSILON * SIMD_EPSILON
    }

    /// `btVector3::absolute`.
    #[inline]
    #[must_use]
    pub fn absolute(self) -> Self {
        Self::new(self.x.abs(), self.y.abs(), self.z.abs())
    }

    /// `btVector3::minAxis` -- the index of the smallest component, ties going
    /// to the highest index (`btVector3.h:470-473`). Note that this is not
    /// [`Vec3::max_axis`]'s mirror image: the two disagree on how ties break.
    #[inline]
    #[must_use]
    pub fn min_axis(self) -> usize {
        if self.x < self.y {
            if self.x < self.z { 0 } else { 2 }
        } else if self.y < self.z {
            1
        } else {
            2
        }
    }

    /// `btVector3::maxAxis` -- the index of the largest component, ties going
    /// to the lowest index (`btVector3.h:487-490`).
    #[inline]
    #[must_use]
    pub fn max_axis(self) -> usize {
        if self.x < self.y {
            if self.y < self.z { 2 } else { 1 }
        } else if self.x < self.z {
            2
        } else {
            0
        }
    }

    /// `btVector3::dot3(v0, v1, v2)` -- the three dot products as a vector
    /// (`btVector3.h:720`). `btTransform::operator()` and `btTransformAabb`
    /// are both written in terms of it.
    #[inline]
    #[must_use]
    pub fn dot3(self, v0: Self, v1: Self, v2: Self) -> Self {
        Self::new(self.dot(v0), self.dot(v1), self.dot(v2))
    }

    /// `btVector3::maxDot` over an array, returning `(index, dot)`
    /// (`btVector3.h:998-1027`, scalar branch).
    ///
    /// Seeded with `-SIMD_INFINITY` and compared with a strict `>`, so ties go
    /// to the *first* vertex -- which is what makes a hull's support vertex a
    /// function of the order `addPoint` was called in, not only of the point
    /// set. An empty array returns `None` where Bullet returns its
    /// `ptIndex = -1` seed; the one caller that can see it
    /// (`convexHullSupport`) guards on a non-empty point list first.
    #[must_use]
    pub fn max_dot(self, points: &[Self]) -> Option<(usize, Scalar)> {
        let mut max_dot1 = -SIMD_INFINITY;
        let mut pt_index = None;
        for (i, point) in points.iter().enumerate() {
            let dot = point.dot(self);
            if dot > max_dot1 {
                max_dot1 = dot;
                pt_index = Some(i);
            }
        }
        pt_index.map(|i| (i, max_dot1))
    }
}

impl Index<usize> for Vec3 {
    type Output = Scalar;

    /// `btVector3::operator[]`. Bullet indexes `m_floats` with no bound check;
    /// this panics on `>= 3` rather than reading the alignment padding.
    #[inline]
    fn index(&self, index: usize) -> &Scalar {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("btVector3 component index {index} out of range"),
        }
    }
}

impl IndexMut<usize> for Vec3 {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut Scalar {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("btVector3 component index {index} out of range"),
        }
    }
}

impl Add for Vec3 {
    type Output = Self;

    #[inline]
    fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl Sub for Vec3 {
    type Output = Self;

    #[inline]
    fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}

impl Neg for Vec3 {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<Scalar> for Vec3 {
    type Output = Self;

    #[inline]
    fn mul(self, s: Scalar) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}

impl Mul<Vec3> for Scalar {
    type Output = Vec3;

    #[inline]
    fn mul(self, v: Vec3) -> Vec3 {
        v * self
    }
}

impl Mul for Vec3 {
    type Output = Self;

    /// `operator*(const btVector3&, const btVector3&)` -- component-wise, not
    /// a dot or a cross (`btVector3.h:869-880`). It is how local scaling is
    /// applied throughout the shape layer.
    #[inline]
    fn mul(self, other: Self) -> Self {
        Self::new(self.x * other.x, self.y * other.y, self.z * other.z)
    }
}

impl Div<Scalar> for Vec3 {
    type Output = Self;

    /// `operator/(const btVector3&, const btScalar&)` (`btVector3.h:843-856`)
    /// and `operator/=` (`:209-224`) are both `v * (1 / s)` in this build, not
    /// three divides. GJK and EPA divide by a length on nearly every
    /// iteration, so the distinction is not decorative: `1/s` rounds once and
    /// each component then rounds again, which is not the same value three
    /// correctly-rounded divides produce.
    #[inline]
    fn div(self, s: Scalar) -> Self {
        self * (1.0 / s)
    }
}

impl DivAssign<Scalar> for Vec3 {
    #[inline]
    fn div_assign(&mut self, s: Scalar) {
        *self = *self / s;
    }
}

impl AddAssign for Vec3 {
    #[inline]
    fn add_assign(&mut self, other: Self) {
        *self = *self + other;
    }
}

impl SubAssign for Vec3 {
    #[inline]
    fn sub_assign(&mut self, other: Self) {
        *self = *self - other;
    }
}

/// `btMatrix3x3`, stored as Bullet stores it: three **row** vectors
/// (`btMatrix3x3.h:38-43`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Matrix3 {
    /// `m_el` -- `rows[i]` is row `i`, so `rows[i][j]` is the element at row
    /// `i`, column `j`.
    pub rows: [Vec3; 3],
}

impl Matrix3 {
    /// `btMatrix3x3` from three rows.
    #[inline]
    #[must_use]
    pub const fn from_rows(r0: Vec3, r1: Vec3, r2: Vec3) -> Self {
        Self { rows: [r0, r1, r2] }
    }

    /// `btMatrix3x3::getIdentity`.
    #[inline]
    #[must_use]
    pub const fn identity() -> Self {
        Self::from_rows(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
    }

    /// `btMatrix3x3::transpose`.
    #[inline]
    #[must_use]
    pub fn transpose(&self) -> Self {
        let [a, b, c] = self.rows;
        Self::from_rows(
            Vec3::new(a.x, b.x, c.x),
            Vec3::new(a.y, b.y, c.y),
            Vec3::new(a.z, b.z, c.z),
        )
    }

    /// `btMatrix3x3::absolute`.
    #[inline]
    #[must_use]
    pub fn absolute(&self) -> Self {
        Self::from_rows(
            self.rows[0].absolute(),
            self.rows[1].absolute(),
            self.rows[2].absolute(),
        )
    }

    /// `btMatrix3x3::tdotx`/`tdoty`/`tdotz` as one call: `self^T * v`, which
    /// is also what `operator*(const btVector3&, const btMatrix3x3&)` computes
    /// (`btMatrix3x3.h:669-680`, `:1225`).
    ///
    /// The narrow phase uses this to take a world-space direction into a
    /// shape's local frame without materializing an inverse.
    #[inline]
    #[must_use]
    pub fn transposed_mul_vec(&self, v: Vec3) -> Vec3 {
        Vec3::new(
            self.rows[0].x * v.x + self.rows[1].x * v.y + self.rows[2].x * v.z,
            self.rows[0].y * v.x + self.rows[1].y * v.y + self.rows[2].y * v.z,
            self.rows[0].z * v.x + self.rows[1].z * v.y + self.rows[2].z * v.z,
        )
    }

    /// `btMatrix3x3::transposeTimes(m)` -- `self^T * m`
    /// (`btMatrix3x3.h:1147-1157`, scalar branch).
    ///
    /// Written out element by element rather than as three `transposed_mul_vec`
    /// calls, because `self^T * m` and `self^T * m^T` differ by exactly which
    /// index of `m` the sum runs over and the compact spelling makes them look
    /// alike.
    #[inline]
    #[must_use]
    pub fn transpose_times(&self, m: &Self) -> Self {
        let e = &self.rows;
        let element = |i: usize, j: usize| {
            e[0][i] * m.rows[0][j] + e[1][i] * m.rows[1][j] + e[2][i] * m.rows[2][j]
        };
        Self::from_rows(
            Vec3::new(element(0, 0), element(0, 1), element(0, 2)),
            Vec3::new(element(1, 0), element(1, 1), element(1, 2)),
            Vec3::new(element(2, 0), element(2, 1), element(2, 2)),
        )
    }
}

impl Index<usize> for Matrix3 {
    type Output = Vec3;

    /// `btMatrix3x3::operator[]` -- row `i`.
    #[inline]
    fn index(&self, index: usize) -> &Vec3 {
        &self.rows[index]
    }
}

impl Mul<Vec3> for Matrix3 {
    type Output = Vec3;

    /// `operator*(const btMatrix3x3&, const btVector3&)`.
    #[inline]
    fn mul(self, v: Vec3) -> Vec3 {
        Vec3::new(
            self.rows[0].dot(v),
            self.rows[1].dot(v),
            self.rows[2].dot(v),
        )
    }
}

impl Mul for Matrix3 {
    type Output = Self;

    /// `operator*(const btMatrix3x3& m1, const btMatrix3x3& m2)` --
    /// `btMatrix3x3(m2.tdotx(m1[i]), m2.tdoty(m1[i]), m2.tdotz(m1[i]))` per
    /// row (`btMatrix3x3.h:1342-1346`, scalar branch).
    #[inline]
    fn mul(self, m: Self) -> Self {
        Self::from_rows(
            m.transposed_mul_vec(self.rows[0]),
            m.transposed_mul_vec(self.rows[1]),
            m.transposed_mul_vec(self.rows[2]),
        )
    }
}

/// `btTransform` -- a rotation basis and a translation (`btTransform.h:32-46`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    /// `m_basis` -- the rotation, as three rows.
    pub basis: Matrix3,
    /// `m_origin` -- the translation.
    pub origin: Vec3,
}

impl Transform {
    /// `btTransform(basis, origin)`.
    #[inline]
    #[must_use]
    pub const fn new(basis: Matrix3, origin: Vec3) -> Self {
        Self { basis, origin }
    }

    /// `btTransform::setIdentity`.
    #[inline]
    #[must_use]
    pub const fn identity() -> Self {
        Self::new(Matrix3::identity(), Vec3::zero())
    }

    /// `btTransform::operator()` / `operator*(const btVector3&)`
    /// (`btTransform.h:91-100`).
    #[inline]
    #[must_use]
    pub fn transform_point(&self, x: Vec3) -> Vec3 {
        x.dot3(self.basis[0], self.basis[1], self.basis[2]) + self.origin
    }

    /// `btTransform::inverse` (`btTransform.h:183-187`).
    #[inline]
    #[must_use]
    pub fn inverse(&self) -> Self {
        let inv = self.basis.transpose();
        Self::new(inv, inv * -self.origin)
    }

    /// `btTransform::invXform` (`btTransform.h:215-220`) -- a point brought
    /// into this frame.
    ///
    /// Not `self.inverse().transform_point(x)`: upstream subtracts the origin
    /// first and applies the transposed basis to the difference, which is one
    /// rotation of a translated point rather than a rotation of the origin
    /// followed by an addition.
    #[inline]
    #[must_use]
    pub fn inv_xform(&self, in_vec: Vec3) -> Vec3 {
        self.basis.transposed_mul_vec(in_vec - self.origin)
    }

    /// `btTransform::inverseTimes` -- `self.inverse() * t`
    /// (`btTransform.h:222-228`).
    #[inline]
    #[must_use]
    pub fn inverse_times(&self, t: &Self) -> Self {
        let v = t.origin - self.origin;
        Self::new(
            self.basis.transpose_times(&t.basis),
            self.basis.transposed_mul_vec(v),
        )
    }
}

impl Mul for Transform {
    type Output = Self;

    /// `btTransform::operator*(const btTransform&)` (`btTransform.h:230-235`).
    #[inline]
    fn mul(self, t: Self) -> Self {
        Self::new(self.basis * t.basis, self.transform_point(t.origin))
    }
}

/// `btTransformAabb(halfExtents, margin, t, ...)`
/// (`btAabbUtil2.h:172-180`).
#[must_use]
pub fn transform_aabb_half_extents(
    half_extents: Vec3,
    margin: Scalar,
    t: &Transform,
) -> (Vec3, Vec3) {
    let half_extents_with_margin = half_extents + Vec3::new(margin, margin, margin);
    let abs_b = t.basis.absolute();
    let center = t.origin;
    let extent = half_extents_with_margin.dot3(abs_b[0], abs_b[1], abs_b[2]);
    (center - extent, center + extent)
}

/// `btTransformAabb(localAabbMin, localAabbMax, margin, trans, ...)`
/// (`btAabbUtil2.h:182-196`).
#[must_use]
pub fn transform_aabb(
    local_aabb_min: Vec3,
    local_aabb_max: Vec3,
    margin: Scalar,
    trans: &Transform,
) -> (Vec3, Vec3) {
    let mut local_half_extents = 0.5 * (local_aabb_max - local_aabb_min);
    local_half_extents += Vec3::new(margin, margin, margin);

    let local_center = 0.5 * (local_aabb_max + local_aabb_min);
    let abs_b = trans.basis.absolute();
    let center = trans.transform_point(local_center);
    let extent = local_half_extents.dot3(abs_b[0], abs_b[1], abs_b[2]);
    (center - extent, center + extent)
}

/// `TestAabbAgainstAabb2(aabbMin1, aabbMax1, aabbMin2, aabbMax2)`
/// (`btAabbUtil2.h:43-51`).
///
/// Touching counts as overlapping: the comparisons that reject are strict, so
/// two boxes that share a face plane pass. Both compound algorithms cull child
/// pairs with this after growing box 1 by the closest-point distance
/// threshold, which is why the sense of each comparison is worth pinning
/// rather than re-deriving -- an inclusive/exclusive slip only shows on the
/// exact-touch row.
///
/// Upstream tests x, then z, then y. The order is unobservable in the result
/// -- three `&&`-equivalent terms over a `bool` with no short-circuit -- so it
/// is not reproduced.
#[must_use]
pub fn test_aabb_against_aabb2(
    aabb_min1: Vec3,
    aabb_max1: Vec3,
    aabb_min2: Vec3,
    aabb_max2: Vec3,
) -> bool {
    aabb_min1.x <= aabb_max2.x
        && aabb_max1.x >= aabb_min2.x
        && aabb_min1.y <= aabb_max2.y
        && aabb_max1.y >= aabb_min2.y
        && aabb_min1.z <= aabb_max2.z
        && aabb_max1.z >= aabb_min2.z
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `SIMD_INFINITY` is `FLT_MAX`, not an infinity -- the one constant whose
    /// name says the opposite of its value, and the seed every `maxDot` and
    /// GJK distance loop starts from.
    #[test]
    fn simd_infinity_is_the_largest_finite_float() {
        assert!(SIMD_INFINITY.is_finite());
        assert_eq!(SIMD_INFINITY, f32::MAX);
        const { assert!(-SIMD_INFINITY > f32::NEG_INFINITY) };
    }

    /// `btFsel`'s `a >= 0` boundary: a zero direction component picks the
    /// positive face. Reproduced against the `bullet_support` oracle op, which
    /// reports all-positive box components for `d = [0, 1, 0]`.
    #[test]
    fn bt_fsel_sends_zero_to_the_positive_branch() {
        assert_eq!(bt_fsel(0.0, 1.0, -1.0), 1.0);
        assert_eq!(bt_fsel(-0.0, 1.0, -1.0), 1.0);
        assert_eq!(bt_fsel(-1.0e-45, 1.0, -1.0), -1.0);
    }

    /// `normalize` is reciprocal-then-multiply.
    ///
    /// `(1, 1, 3)` is a vector where the two spellings actually disagree --
    /// the obvious `(1, 2, 3)` does not, and a test written on it passes
    /// against either implementation. Found by enumerating small integer
    /// triples for a mismatch rather than assumed.
    #[test]
    fn normalize_multiplies_by_the_reciprocal() {
        let v = Vec3::new(1.0, 1.0, 3.0);
        let len = v.length();
        let recip = 1.0 / len;
        assert_eq!(
            v.normalize(),
            Vec3::new(1.0 * recip, 1.0 * recip, 3.0 * recip)
        );
        assert_ne!(
            v.normalize(),
            Vec3::new(1.0 / len, 1.0 / len, 3.0 / len),
            "the component-wise divide must not pass this test"
        );
    }

    /// Ties in `maxDot` go to the first vertex, which is what makes a convex
    /// hull's support point depend on `addPoint` order.
    #[test]
    fn max_dot_breaks_ties_toward_the_first_vertex() {
        let points = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(1.0, -1.0, 0.0),
        ];
        let (index, dot) = Vec3::new(1.0, 0.0, 0.0)
            .max_dot(&points)
            .expect("non-empty");
        assert_eq!(index, 0);
        assert_eq!(dot, 1.0);
    }

    #[test]
    fn max_dot_of_an_empty_slice_has_no_index() {
        assert_eq!(Vec3::new(1.0, 0.0, 0.0).max_dot(&[]), None);
    }

    /// `v * m` is `m^T * v`, not `m * v` -- the narrow phase relies on the
    /// difference to push a world direction into a local frame.
    #[test]
    fn vector_times_matrix_transposes() {
        let m = Matrix3::from_rows(
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        );
        let v = Vec3::new(1.0, 0.0, 0.0);
        assert_eq!(m * v, Vec3::new(0.0, 1.0, 0.0));
        assert_eq!(m.transposed_mul_vec(v), Vec3::new(0.0, -1.0, 0.0));
    }

    /// `A^T * B` is not `A^T * B^T`, and the two agree on every symmetric `B`
    /// -- including the identity, which is what a composition test would
    /// otherwise reach for. Both operands here are asymmetric so the wrong
    /// spelling cannot pass.
    #[test]
    fn transpose_times_transposes_only_the_left_operand() {
        let a = Matrix3::from_rows(
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(4.0, 5.0, 6.0),
            Vec3::new(7.0, 8.0, 10.0),
        );
        let b = Matrix3::from_rows(
            Vec3::new(0.0, 1.0, 2.0),
            Vec3::new(3.0, -1.0, 0.0),
            Vec3::new(1.0, 0.0, 4.0),
        );
        let expected = a.transpose() * b;
        assert_eq!(a.transpose_times(&b), expected);
        assert_ne!(a.transpose_times(&b), a.transpose() * b.transpose());
    }

    #[test]
    fn inverse_times_equals_inverse_then_multiply() {
        let a = Transform::new(
            Matrix3::from_rows(
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            Vec3::new(1.0, 2.0, 3.0),
        );
        // An asymmetric basis on both sides: with `b` a pure translation the
        // wrong `transpose_times` spelling is indistinguishable.
        let s = core::f32::consts::FRAC_1_SQRT_2;
        let b = Transform::new(
            Matrix3::from_rows(
                Vec3::new(s, 0.0, -s),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(s, 0.0, s),
            ),
            Vec3::new(-4.0, 5.0, 0.5),
        );
        assert_eq!(a.inverse_times(&b), a.inverse() * b);
    }

    #[test]
    fn transform_point_applies_basis_then_origin() {
        let t = Transform::new(
            Matrix3::from_rows(
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            Vec3::new(10.0, 0.0, 0.0),
        );
        assert_eq!(
            t.transform_point(Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(10.0, 1.0, 0.0)
        );
    }

    /// A rotated box's world AABB grows by the projection of its half extents
    /// onto the absolute basis, and the margin is added *before* that
    /// projection -- so a margin on a 45-degree-rotated box inflates the AABB
    /// by more than the margin.
    #[test]
    fn transform_aabb_adds_the_margin_before_projecting() {
        let s = core::f32::consts::FRAC_1_SQRT_2;
        let t = Transform::new(
            Matrix3::from_rows(
                Vec3::new(s, -s, 0.0),
                Vec3::new(s, s, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            Vec3::zero(),
        );
        let (min, max) = transform_aabb_half_extents(Vec3::new(1.0, 1.0, 1.0), 0.5, &t);
        assert_eq!(max.x, 1.5 * s + 1.5 * s);
        assert_eq!(min, -max);
    }

    #[test]
    fn transform_aabb_from_a_local_box_recentres_it() {
        let t = Transform::new(Matrix3::identity(), Vec3::new(1.0, 0.0, 0.0));
        let (min, max) = transform_aabb(
            Vec3::new(-1.0, -2.0, -3.0),
            Vec3::new(3.0, 2.0, 1.0),
            0.0,
            &t,
        );
        assert_eq!(min, Vec3::new(0.0, -2.0, -3.0));
        assert_eq!(max, Vec3::new(4.0, 2.0, 1.0));
    }

    /// The `aabb_*` rows of `tools/bullet-epa-reference/build.sh`'s stdout.
    ///
    /// The unit cube against a second box placed at each boundary in turn:
    /// touching on a face, and clear of it by one part in ten thousand. The
    /// `touch_*` rows are the ones that matter -- they are what says the
    /// comparisons that reject are strict -- and `gap_in_one_axis_only` is
    /// what says all three axes are tested rather than the first that
    /// separates.
    const BULLET_REFERENCE: &str = "\
aabb_overlap|1
aabb_touch_x|1
aabb_gap_x|0
aabb_touch_y|1
aabb_gap_y|0
aabb_touch_z|1
aabb_gap_z|0
aabb_touch_neg_x|1
aabb_gap_neg_x|0
aabb_contained|1
aabb_degenerate|1
aabb_gap_in_one_axis_only|0
";

    #[test]
    fn bullet_reference_test_aabb_against_aabb2() {
        let lo = Vec3::zero();
        let hi = Vec3::new(1.0, 1.0, 1.0);
        let cases: [(&str, Vec3, Vec3); 12] = [
            (
                "overlap",
                Vec3::new(0.5, 0.5, 0.5),
                Vec3::new(1.5, 1.5, 1.5),
            ),
            (
                "touch_x",
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(2.0, 1.0, 1.0),
            ),
            (
                "gap_x",
                Vec3::new(1.0001, 0.0, 0.0),
                Vec3::new(2.0, 1.0, 1.0),
            ),
            (
                "touch_y",
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 2.0, 1.0),
            ),
            (
                "gap_y",
                Vec3::new(0.0, 1.0001, 0.0),
                Vec3::new(1.0, 2.0, 1.0),
            ),
            (
                "touch_z",
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 1.0, 2.0),
            ),
            (
                "gap_z",
                Vec3::new(0.0, 0.0, 1.0001),
                Vec3::new(1.0, 1.0, 2.0),
            ),
            (
                "touch_neg_x",
                Vec3::new(-1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 1.0),
            ),
            (
                "gap_neg_x",
                Vec3::new(-1.0, 0.0, 0.0),
                Vec3::new(-0.0001, 1.0, 1.0),
            ),
            (
                "contained",
                Vec3::new(0.25, 0.25, 0.25),
                Vec3::new(0.75, 0.75, 0.75),
            ),
            (
                "degenerate",
                Vec3::new(1.0, 1.0, 1.0),
                Vec3::new(1.0, 1.0, 1.0),
            ),
            (
                "gap_in_one_axis_only",
                Vec3::new(0.5, 0.5, 1.0001),
                Vec3::new(1.5, 1.5, 2.0),
            ),
        ];

        let mut bad = Vec::new();
        for (name, min2, max2) in cases {
            let want = BULLET_REFERENCE
                .lines()
                .find(|l| l.starts_with(&format!("aabb_{name}|")))
                .unwrap_or_else(|| panic!("aabb_{name}: no such row in BULLET_REFERENCE"))
                .ends_with('1');
            let got = test_aabb_against_aabb2(lo, hi, min2, max2);
            if got != want {
                bad.push(format!("aabb_{name}: port {got}, bullet {want}"));
            }
        }
        assert_eq!(
            BULLET_REFERENCE.lines().count(),
            cases.len(),
            "BULLET_REFERENCE lost or gained a row"
        );
        assert!(bad.is_empty(), "{}", bad.join("\n"));
    }

    /// The `invxform_*` rows of `tools/bullet-epa-reference/build.sh`'s stdout.
    ///
    /// Each row carries both routes: `invXform` and `inverse()` then
    /// `operator*`. They are the same map, so what the row pins is that
    /// bullet's own two spellings disagree in `f32` and by how much -- which
    /// is why the port cannot substitute one for the other.
    ///
    /// The identity row is the control: with no rotation and no translation
    /// the two agree exactly, so a fixture built only from rotated frames
    /// would not say which route the port took if it ever gained a tolerance.
    ///
    /// Fields: `name|invXform xyz|inverseThenMul xyz`.
    const BULLET_REFERENCE_INV_XFORM: &str = "\
invxform_id|-10|-20|7|-10|-20|7
invxform_rot60|-22.2000008|-5.10000038|4.20000076|-22.2000008|-5.0999999|4.20000029
invxform_rot60_near|0.010000011|0.010000011|0.00999998022|0.0100000128|0.00999999046|0.00999999046
";

    #[test]
    fn bullet_reference_inv_xform() {
        let rot60 = crate::probe_fixture::rot60_at(0.3, -0.4, 0.2);
        let far = Vec3::new(-10.0, -20.0, 7.0);
        let cases: [(&str, Transform, Vec3); 3] = [
            ("id", crate::probe_fixture::IDENTITY, far),
            ("rot60", rot60, far),
            ("rot60_near", rot60, Vec3::new(0.31, -0.39, 0.21)),
        ];

        let mut bad = Vec::new();
        for (name, t, p) in cases {
            let f: Vec<&str> = BULLET_REFERENCE_INV_XFORM
                .lines()
                .find(|l| l.starts_with(&format!("invxform_{name}|")))
                .unwrap_or_else(|| panic!("invxform_{name}: no such row"))
                .split('|')
                .collect();
            let n = |k: usize| -> Scalar { f[k].parse().expect("a float field") };

            let got = t.inv_xform(p);
            let want = Vec3::new(n(1), n(2), n(3));
            if got != want {
                bad.push(format!("invxform_{name}: port {got:?}, bullet {want:?}"));
            }
            let other = Vec3::new(n(4), n(5), n(6));
            if name == "id" {
                assert_eq!(want, other, "the control row's two routes agree");
            } else {
                assert_ne!(
                    want, other,
                    "invxform_{name} was chosen because the two routes disagree"
                );
            }
        }
        assert_eq!(
            BULLET_REFERENCE_INV_XFORM.lines().count(),
            cases.len(),
            "BULLET_REFERENCE_INV_XFORM lost or gained a row"
        );
        assert!(bad.is_empty(), "{}", bad.join("\n"));
    }
}
