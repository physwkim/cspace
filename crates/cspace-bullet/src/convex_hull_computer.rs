// Copyright (c) 2011 Ole Kniemeyer, MAXON, www.maxon.net
// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/LinearMath/btConvexHullComputer.h
//   bullet3/src/LinearMath/btConvexHullComputer.cpp
//   bullet3/src/LinearMath/btAlignedObjectArray.h (quickSortInternal only)

//! `btConvexHullComputer` -- the convex hull of a point cloud, by the
//! Preparata-Hong divide-and-conquer construction, with exact integer
//! orientation predicates.
//!
//! MoveIt turns every collision mesh into a `btConvexHullShape` through this:
//! `createConvexHull` (`contact_checker_common.cpp:123-184`) copies the mesh
//! vertices into `btVector3`s, calls `compute`, and reads back
//! [`ConvexHullComputer::vertices`] and the face loops walked out of
//! [`ConvexHullComputer::faces`] and [`ConvexHullComputer::edges`]. Without it
//! the continuous-collision port cannot accept a single robot link mesh.
//!
//! # Why the output *order* is part of the answer
//!
//! `btConvexHullShape::localGetSupportingVertexWithoutMargin` breaks ties
//! toward the first vertex it saw (`maxDot`), so two hulls with the same
//! vertex *set* in a different order return different support points, and from
//! there different witnesses, normals and contact points. A port that computed
//! the right eight cube corners in the wrong order would pass every set-wise
//! check and still move contacts. So the fixture in this file's tests pins one
//! row per output vertex in emission order, not a summary.
//!
//! # Exact integer arithmetic, not floating point
//!
//! The input is quantized onto an integer grid before anything else happens.
//! `compute` measures the point cloud's extent, divides it by `10216` per axis
//! and rounds each coordinate to an `int32` (`btConvexHullComputer.cpp:1983-2044`),
//! so every coordinate afterwards is a small integer and every geometric
//! predicate -- "is this point above that plane", "which of these two edges
//! turns further" -- is an exact integer sign test rather than a float
//! comparison against a tolerance. That is the whole design: the hull is
//! decided by `Int128`, `Rational64` and `Rational128`, and floats
//! reappear only in `HullInternal::get_coordinates`, which maps the surviving
//! grid points back. Approximating those predicates does not merely perturb
//! coordinates -- it changes which points are on the hull.
//!
//! `10216` is the grid resolution and nothing else; it makes each quantized
//! coordinate fit in `[-5108, 5108]`, which keeps the cross products in
//! `Point64` and the dot products against them inside `i64` with room to
//! spare, so the exact predicates need 128 bits only where upstream reaches
//! for them.
//!
//! # Pointers, pools and identity
//!
//! Upstream is built out of intrusive doubly-linked lists of pool-allocated
//! `Vertex`/`Edge`/`Face` objects, and it compares those pointers for identity
//! constantly (`e != startEdge`, `c0 == first0`). This port replaces the
//! pointers with `u32` indices into three `Vec` arenas on `HullInternal`,
//! with `NIL` for `NULL`.
//!
//! Identity comparison then becomes index comparison, which is only faithful
//! if the two allocators hand out the same identities in the same order. The
//! edge pool is therefore reproduced exactly, free list included: upstream's
//! `Pool<T>` pushes a freed object onto the head of a singly-linked free list
//! and pops from that head first, so a `removeEdgePair` followed by a
//! `newEdgePair` *reuses* the just-freed slot. `merge` interleaves those two
//! calls and keeps `Edge*` values across them (`toPrev0`, `firstNew0`), so an
//! allocator that handed out a fresh slot where upstream recycled one would
//! compare a stale handle against a different index and take a different
//! branch. See `HullInternal::new_edge_object`.
//!
//! # `btAssert` is compiled out
//!
//! `tools/bullet-epa-reference/build.sh` compiles without `DEBUG`/`_DEBUG`, so
//! `btScalar.h:295` expands `btAssert(x)` to nothing and every `btAssert` in
//! the C++ is dead text in the build this port is measured against. They are
//! carried here as doc comments on the branch they describe rather than as
//! `assert!`, because a `debug_assert!` that upstream does not evaluate would
//! be this port's own new failure mode on inputs upstream accepts.
//!
//! # `shrink`
//!
//! `HullInternal::shrink` and `HullInternal::shift_face` move every face
//! inward along its normal. MoveIt never asks for it: `createConvexHull`'s
//! `shrink`/`shrinkClamp` default to `-1` (`contact_checker_common.hpp:73-74`)
//! and its one caller (`bullet_utils.cpp:144`) passes neither, so
//! `compute`'s `shrink > 0` gate is false and the whole path -- with it
//! `Rational128`, `PointR128`, `Face`, and the `point.index < 0` branch of
//! every `Vertex` coordinate accessor -- is unreachable from MoveIt. It is
//! ported anyway, and the fixture calls `compute` with a positive `shrink` on
//! two of its rows so the path is measured rather than merely written.
//!
//! # Deviations
//!
//! - **No `stride`.** Upstream takes `const void* coords` plus a byte stride so
//!   it can walk a padded or interleaved array; MoveIt passes
//!   `sizeof(btVector3)` over an array of `btVector3`. [`ConvexHullComputer::compute`]
//!   takes `&[Vec3]`, whose stride is its element size by construction. The
//!   `double` overload goes with it: MoveIt's is the `float` one, and `Vec3`
//!   is already `f32`.
//! - **`Int128` is a `u128`.** Upstream carries a `{low, high}` pair with
//!   hand-written add/negate/compare because C++ has no 128-bit integer; Rust
//!   does. Every operation below is the same modular arithmetic on the same
//!   128 bits, so the pair representation is unobservable.
//! - **`DMul` is exact multiplication.** Upstream's `DMul<UWord, UHWord>` is
//!   schoolbook long multiplication that computes the *exact* double-width
//!   product with no rounding or truncation, in both instantiations. Any exact
//!   implementation of the same product is therefore bit-identical, and
//!   `dmul_u128` is written as the exact 256-bit product.
//! - **Overflow.** Where upstream relies on `uint64_t`/`unsigned` wraparound
//!   the port spells `wrapping_*`; where the C++ is signed (and so would be
//!   undefined on overflow) the port uses plain arithmetic, which panics under
//!   this workspace's `overflow-checks`. Those sites are bounded by the
//!   `+/-5108` grid, and a panic is a better failure than a silent wrap.
//! - **Names.** Rust has no overloading, so `Point32::dot` splits into
//!   `Point32::dot` and `Point32::dot_p64` and `cross` into
//!   `Point32::cross` and `Point32::cross_p64`; `btConvexHullInternal::Edge`
//!   becomes `InternalEdge` so the public [`Edge`] keeps upstream's name.
//!
//! Unqualified citations in this file are lines in
//! `btConvexHullComputer.cpp`; a citation of any other file names that file.

use crate::linear_math::{SIMD_INFINITY, Scalar, Vec3};
use core::cmp::Ordering;
use core::ops::{Add, AddAssign, Neg, Sub};

/// The port's `NULL`: no vertex, edge or face.
///
/// `u32::MAX` rather than `Option<u32>` because upstream walks these links in
/// tight `do { e = e->next } while (e != start)` loops where every step would
/// otherwise unwrap, and because the arenas cannot reach `u32::MAX` entries --
/// the edge pool is sized `6 * count` and `count` is a mesh vertex count.
const NIL: u32 = u32::MAX;

/// Index into [`HullInternal::verts`], or [`NIL`].
type VId = u32;

/// Index into [`HullInternal::edges`], or [`NIL`].
type EId = u32;

/// Index into [`HullInternal::faces`], or [`NIL`].
type FId = u32;

// ---------------------------------------------------------------------------
// Exact integer arithmetic
// ---------------------------------------------------------------------------

/// `btConvexHullInternal::Int128` (`btConvexHullComputer.cpp:137-269`) -- a
/// 128-bit integer that is unsigned for comparison and two's-complement signed
/// for sign tests and conversion to float.
///
/// The dual reading is upstream's, not an ambiguity introduced here:
/// [`Int128::ucmp`] and [`Int128::get_sign`] are both called on the same values,
/// because `Rational64`/`Rational128` keep the sign in a separate field and
/// compare magnitudes unsigned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Int128(u128);

impl Int128 {
    /// `Int128(uint64_t low)` -- zero-extended.
    const fn from_u64(low: u64) -> Self {
        Self(low as u128)
    }

    /// `Int128(int64_t value)` -- sign-extended, which is what the `{low, high}`
    /// constructor spells as `high = (value >= 0) ? 0 : -1`.
    const fn from_i64(value: i64) -> Self {
        Self(value as i128 as u128)
    }

    /// The low 64 bits, upstream's `low` member.
    const fn low(self) -> u64 {
        self.0 as u64
    }

    /// The high 64 bits, upstream's `high` member.
    const fn high(self) -> u64 {
        (self.0 >> 64) as u64
    }

    /// `Int128::mul(int64_t, int64_t)` -- the exact 128-bit signed product
    /// (`:853-878`).
    ///
    /// Upstream takes magnitudes, multiplies them with `DMul`, and negates when
    /// the signs disagree. `|a| * |b| <= 2^126`, so no product is truncated and
    /// the result is exact; `i128` multiplication is the same value. The one
    /// input where upstream's `a = -a` does not produce a magnitude is
    /// `i64::MIN`, whose negation wraps back to itself -- and `(uint64_t)a` is
    /// then `2^63`, which *is* the magnitude, so that path is right by
    /// accident and agrees with this one.
    fn mul_i64(a: i64, b: i64) -> Self {
        Self((i128::from(a) * i128::from(b)) as u128)
    }

    /// `Int128::mul(uint64_t, uint64_t)` -- the exact 128-bit unsigned product
    /// (`:880-895`).
    fn mul_u64(a: u64, b: u64) -> Self {
        Self(u128::from(a) * u128::from(b))
    }

    /// `Int128::operator*(int64_t b)` (`:839-851`) -- `self * b` truncated to
    /// 128 bits.
    ///
    /// Not an exact product: upstream adds `a.high * b` into the high word with
    /// `uint64_t` arithmetic, discarding everything above bit 127. Its one
    /// caller is [`Rational128::compare_i64`], where the operands are a
    /// denominator and a coordinate small enough that nothing is discarded.
    fn mul_by_i64(self, b: i64) -> Self {
        let negative = (self.high() as i64) < 0;
        let a = if negative { -self } else { self };
        let (negative, b) = if b < 0 {
            (!negative, b.wrapping_neg())
        } else {
            (negative, b)
        };
        let result = Self(a.0.wrapping_mul(u128::from(b as u64)));
        if negative { -result } else { result }
    }

    /// `Int128::toScalar()` (`:233-237`).
    ///
    /// `btScalar(0x100000000LL) * btScalar(0x100000000LL)` is `2^64`, exact in
    /// `f32`; the two `u64 -> f32` conversions each round to nearest even in
    /// both languages, so the sum rounds identically. A negative value is
    /// negated first and the *result* negated, so the rounding happens on the
    /// magnitude either way.
    fn to_scalar(self) -> Scalar {
        if (self.high() as i64) >= 0 {
            self.high() as Scalar * (4_294_967_296.0 as Scalar * 4_294_967_296.0 as Scalar)
                + self.low() as Scalar
        } else {
            -(-self).to_scalar()
        }
    }

    /// `Int128::getSign()` (`:239-242`) -- the sign of the two's-complement
    /// reading: `-1`, `0` or `1`.
    fn get_sign(self) -> i32 {
        if (self.high() as i64) < 0 {
            -1
        } else if self.0 != 0 {
            1
        } else {
            0
        }
    }

    /// `Int128::ucmp` (`:249-268`) -- three-way *unsigned* comparison.
    ///
    /// Upstream compares `high` then `low`, which is exactly unsigned 128-bit
    /// ordering.
    fn ucmp(self, b: Self) -> i32 {
        match self.0.cmp(&b.0) {
            Ordering::Less => -1,
            Ordering::Equal => 0,
            Ordering::Greater => 1,
        }
    }
}

impl Neg for Int128 {
    type Output = Self;

    /// `Int128::operator-()` (`:163-166`) -- two's-complement negation, spelled
    /// upstream as `(-low, ~high + (low == 0))` so that the borrow out of the
    /// low word lands in the high one.
    fn neg(self) -> Self {
        Self(self.0.wrapping_neg())
    }
}

impl Add for Int128 {
    type Output = Self;

    /// `Int128::operator+` (`:168-183`) -- add with carry, modulo `2^128`.
    fn add(self, b: Self) -> Self {
        Self(self.0.wrapping_add(b.0))
    }
}

impl Sub for Int128 {
    type Output = Self;

    /// `Int128::operator-` (`:185-199`), which upstream defines as
    /// `*this + -b`.
    fn sub(self, b: Self) -> Self {
        self + -b
    }
}

impl AddAssign for Int128 {
    /// `Int128::operator+=` (`:201-220`).
    fn add_assign(&mut self, b: Self) {
        *self = *self + b;
    }
}

/// `DMul<Int128, uint64_t>::mul` (`:575-640`) -- the exact 256-bit product of
/// two [`Int128`] values, low half first.
///
/// Upstream splits both operands into 64-bit halves, forms the four partial
/// products with `Int128::mul(uint64_t, uint64_t)`, and folds the carries by
/// hand. Every step is exact -- `p0110` is the sum of two 64-bit values held in
/// 128 bits, and the only truncation is the deliberate `shlHalf`, whose
/// discarded bits were added into the high word one line earlier -- so this
/// transcription is bit-identical to it and to any other exact 256-bit
/// multiply.
///
/// Its only caller is [`Rational128::compare`], where it is what lets two
/// 128-bit rationals be compared by cross-multiplication without overflow.
fn dmul_u128(a: Int128, b: Int128) -> (Int128, Int128) {
    let p00 = Int128::mul_u64(a.low(), b.low());
    let p01 = Int128::mul_u64(a.low(), b.high());
    let p10 = Int128::mul_u64(a.high(), b.low());
    let mut p11 = Int128::mul_u64(a.high(), b.high());

    let p0110 = Int128::from_u64(p01.low()) + Int128::from_u64(p10.low());
    p11 += Int128::from_u64(p01.high());
    p11 += Int128::from_u64(p10.high());
    p11 += Int128::from_u64(p0110.high());

    // `shlHalf(p0110)`: `high = low; low = 0`, i.e. a 64-bit shift that drops
    // the bits already accounted for in `p11` above.
    let p0110 = Int128(u128::from(p0110.low()) << 64);
    let (sum, carry) = p00.0.overflowing_add(p0110.0);
    if carry {
        p11 += Int128::from_u64(1);
    }
    (Int128(sum), p11)
}

/// `btConvexHullInternal::Rational64` (`:271-327`) -- a rational held as an
/// unsigned numerator and denominator plus a separate sign, so that a zero
/// denominator is representable.
///
/// The zero denominator is the point of the class rather than an error case:
/// [`Rational64::is_nan`] (`0/0`) marks an edge pointing straight back along
/// the merge direction, which `findMaxAngle` skips, and
/// [`Rational64::is_negative_infinity`] (`-x/0`) is how `merge` recognises that
/// one side has run out of candidate edges.
#[derive(Clone, Copy, Debug)]
struct Rational64 {
    numerator: u64,
    denominator: u64,
    sign: i32,
}

impl Rational64 {
    /// `Rational64(int64_t numerator, int64_t denominator)` (`:279-309`).
    ///
    /// A negative denominator flips `sign` rather than being stored, so the
    /// stored pair is always the magnitude. `i64::MIN` negates to itself, and
    /// its `u64` reading is then its own magnitude, so `wrapping_neg` is both
    /// what the C++ does and the right answer.
    fn new(numerator: i64, denominator: i64) -> Self {
        let (sign, num) = match numerator.cmp(&0) {
            Ordering::Greater => (1, numerator as u64),
            Ordering::Less => (-1, numerator.wrapping_neg() as u64),
            Ordering::Equal => (0, 0),
        };
        let (sign, den) = match denominator.cmp(&0) {
            Ordering::Greater => (sign, denominator as u64),
            Ordering::Less => (-sign, denominator.wrapping_neg() as u64),
            Ordering::Equal => (sign, 0),
        };
        Self {
            numerator: num,
            denominator: den,
            sign,
        }
    }

    /// `isNegativeInfinity()` (`:311-314`).
    fn is_negative_infinity(self) -> bool {
        (self.sign < 0) && (self.denominator == 0)
    }

    /// `isNaN()` (`:316-319`).
    fn is_nan(self) -> bool {
        (self.sign == 0) && (self.denominator == 0)
    }

    /// `Rational64::compare` (`:897-940`), the non-assembly branch.
    ///
    /// Cross-multiplication in 128 bits rather than a division: two `u64`
    /// products cannot overflow `Int128`, so the comparison is exact for every
    /// input including the infinities above.
    fn compare(self, b: Self) -> i32 {
        if self.sign != b.sign {
            return self.sign - b.sign;
        } else if self.sign == 0 {
            return 0;
        }
        self.sign
            * Int128::mul_u64(self.numerator, b.denominator)
                .ucmp(Int128::mul_u64(self.denominator, b.numerator))
    }
}

/// `btConvexHullInternal::Rational128` (`:329-391`) -- [`Rational64`] widened
/// to 128-bit numerator and denominator, with an `is_int64` fast path.
///
/// `is_int64` is not an optimisation flag that can be recomputed: it records
/// that the value came from the `int64_t` constructor, where the numerator is a
/// plain magnitude and the denominator is `1`. [`Rational128::compare`] and
/// [`Rational128::compare_i64`] both branch on it to reconstruct the signed
/// `i64`, so a port that dropped it would have to divide.
#[derive(Clone, Copy, Debug)]
struct Rational128 {
    numerator: Int128,
    denominator: Int128,
    sign: i32,
    is_int64: bool,
}

impl Rational128 {
    /// `Rational128(int64_t value)` (`:338-357`).
    fn from_i64(value: i64) -> Self {
        let (sign, numerator) = match value.cmp(&0) {
            Ordering::Greater => (1, Int128::from_i64(value)),
            Ordering::Less => (-1, Int128::from_i64(value.wrapping_neg())),
            Ordering::Equal => (0, Int128::from_u64(0)),
        };
        Self {
            numerator,
            denominator: Int128::from_u64(1),
            sign,
            is_int64: true,
        }
    }

    /// `Rational128(const Int128& numerator, const Int128& denominator)`
    /// (`:359-381`).
    fn from_ratio(numerator: Int128, denominator: Int128) -> Self {
        let sign = numerator.get_sign();
        let num = if sign >= 0 { numerator } else { -numerator };
        let dsign = denominator.get_sign();
        let (sign, den) = if dsign >= 0 {
            (sign, denominator)
        } else {
            (-sign, -denominator)
        };
        Self {
            numerator: num,
            denominator: den,
            sign,
            is_int64: false,
        }
    }

    /// `Rational128::compare(const Rational128&)` (`:942-967`).
    ///
    /// The cross products are 256 bits wide, compared high half first; only
    /// then is the sign applied, because both stored pairs are magnitudes.
    fn compare(self, b: Self) -> i32 {
        if self.sign != b.sign {
            return self.sign - b.sign;
        } else if self.sign == 0 {
            return 0;
        }
        if self.is_int64 {
            return -b.compare_i64(i64::from(self.sign).wrapping_mul(self.numerator.low() as i64));
        }

        let (nbd_low, nbd_high) = dmul_u128(self.numerator, b.denominator);
        let (dbn_low, dbn_high) = dmul_u128(self.denominator, b.numerator);

        let cmp = nbd_high.ucmp(dbn_high);
        if cmp != 0 {
            return cmp * self.sign;
        }
        nbd_low.ucmp(dbn_low) * self.sign
    }

    /// `Rational128::compare(int64_t b)` (`:969-997`).
    ///
    /// `b == 0` returns `sign` rather than falling through to the
    /// multiplication: comparing against zero is a sign test, and the
    /// multiplication below would be by zero.
    fn compare_i64(self, b: i64) -> i32 {
        if self.is_int64 {
            let a = i64::from(self.sign).wrapping_mul(self.numerator.low() as i64);
            return match a.cmp(&b) {
                Ordering::Greater => 1,
                Ordering::Less => -1,
                Ordering::Equal => 0,
            };
        }
        let b = match b.cmp(&0) {
            Ordering::Greater => {
                if self.sign <= 0 {
                    return -1;
                }
                b
            }
            Ordering::Less => {
                if self.sign >= 0 {
                    return 1;
                }
                b.wrapping_neg()
            }
            Ordering::Equal => return self.sign,
        };

        self.numerator.ucmp(self.denominator.mul_by_i64(b)) * self.sign
    }
}

/// `btConvexHullInternal::PointR128` (`:393-423`) -- a point with 128-bit
/// rational coordinates over a shared denominator.
///
/// Only [`HullInternal::shift_face`] builds one: a shrunken face meets two
/// neighbouring faces at a point that is not on the integer grid, and this is
/// that point held exactly rather than rounded. The rounded copy goes into
/// `Vertex::point` beside it, which is why a vertex carries both.
#[derive(Clone, Copy, Debug)]
struct PointR128 {
    x: Int128,
    y: Int128,
    z: Int128,
    denominator: Int128,
}

impl PointR128 {
    /// The zero point. Upstream's default constructor leaves all four members
    /// uninitialised; every read is preceded by a write, so the value here is
    /// arbitrary and zero is the one that cannot be mistaken for a computed
    /// coordinate.
    const ZERO: Self = Self {
        x: Int128(0),
        y: Int128(0),
        z: Int128(0),
        denominator: Int128(0),
    };

    /// `xvalue()` (`:409-412`) -- a true `f32` division, not a
    /// reciprocal-multiply.
    fn xvalue(self) -> Scalar {
        self.x.to_scalar() / self.denominator.to_scalar()
    }

    /// `yvalue()` (`:414-417`).
    fn yvalue(self) -> Scalar {
        self.y.to_scalar() / self.denominator.to_scalar()
    }

    /// `zvalue()` (`:419-422`).
    fn zvalue(self) -> Scalar {
        self.z.to_scalar() / self.denominator.to_scalar()
    }
}

/// `btConvexHullInternal::Point64` (`:53-73`) -- an `i64` lattice vector.
///
/// Cross products of [`Point32`]s land here: with coordinates bounded by
/// `+/-5108` a cross product is at most about `5.2e7` and a further dot at most
/// about `1.6e12`, so the exact predicates fit in `i64` and reach for
/// [`Int128`] only in the rational comparisons.
#[derive(Clone, Copy, Debug)]
struct Point64 {
    x: i64,
    y: i64,
    z: i64,
}

impl Point64 {
    /// `Point64(x, y, z)`.
    const fn new(x: i64, y: i64, z: i64) -> Self {
        Self { x, y, z }
    }

    /// `Point64::dot(const Point64&)` (`:69-72`).
    fn dot(self, b: Self) -> i64 {
        self.x * b.x + self.y * b.y + self.z * b.z
    }
}

/// `btConvexHullInternal::Point32` (`:75-135`) -- a quantized input point plus
/// the index it had in the caller's array.
///
/// `index` is carried through the whole computation so that
/// [`ConvexHullComputer::original_vertex_index`] can report, for each output
/// vertex, which input point it was. It is `-1` for a point the algorithm
/// synthesised (arithmetic results, and the intersection vertices
/// [`HullInternal::shift_face`] creates), which is also the flag every
/// coordinate accessor reads to decide between `point` and `point128`.
#[derive(Clone, Copy, Debug)]
struct Point32 {
    x: i32,
    y: i32,
    z: i32,
    index: i32,
}

impl Point32 {
    /// `Point32(int32_t x, int32_t y, int32_t z)` -- note `index` is `-1`, so
    /// every point produced by arithmetic is marked synthetic.
    const fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z, index: -1 }
    }

    /// `isZero()` (`:101-104`).
    fn is_zero(self) -> bool {
        (self.x == 0) && (self.y == 0) && (self.z == 0)
    }

    /// `Point64 cross(const Point32&)` (`:106-109`) -- the `int64` widening is
    /// upstream's and is what keeps the product exact.
    fn cross(self, b: Self) -> Point64 {
        Point64::new(
            i64::from(self.y) * i64::from(b.z) - i64::from(self.z) * i64::from(b.y),
            i64::from(self.z) * i64::from(b.x) - i64::from(self.x) * i64::from(b.z),
            i64::from(self.x) * i64::from(b.y) - i64::from(self.y) * i64::from(b.x),
        )
    }

    /// `Point64 cross(const Point64&)` (`:111-114`).
    fn cross_p64(self, b: Point64) -> Point64 {
        Point64::new(
            i64::from(self.y) * b.z - i64::from(self.z) * b.y,
            i64::from(self.z) * b.x - i64::from(self.x) * b.z,
            i64::from(self.x) * b.y - i64::from(self.y) * b.x,
        )
    }

    /// `int64_t dot(const Point32&)` (`:116-119`).
    fn dot(self, b: Self) -> i64 {
        i64::from(self.x) * i64::from(b.x)
            + i64::from(self.y) * i64::from(b.y)
            + i64::from(self.z) * i64::from(b.z)
    }

    /// `int64_t dot(const Point64&)` (`:121-124`).
    fn dot_p64(self, b: Point64) -> i64 {
        i64::from(self.x) * b.x + i64::from(self.y) * b.y + i64::from(self.z) * b.z
    }
}

impl PartialEq for Point32 {
    /// `operator==` (`:91-94`) compares the three coordinates and **not**
    /// `index`, which is why this is written out rather than derived: two
    /// distinct input points that quantize onto the same grid cell must compare
    /// equal, and that is exactly how `computeInternal` recognises duplicates.
    fn eq(&self, b: &Self) -> bool {
        (self.x == b.x) && (self.y == b.y) && (self.z == b.z)
    }
}

impl Add for Point32 {
    type Output = Self;

    /// `operator+` (`:126-129`).
    fn add(self, b: Self) -> Self {
        Self::new(self.x + b.x, self.y + b.y, self.z + b.z)
    }
}

impl Sub for Point32 {
    type Output = Self;

    /// `operator-` (`:131-134`).
    fn sub(self, b: Self) -> Self {
        Self::new(self.x - b.x, self.y - b.y, self.z - b.z)
    }
}

// ---------------------------------------------------------------------------
// Arena objects
// ---------------------------------------------------------------------------

/// `btConvexHullInternal::Vertex` (`:428-501`).
#[derive(Clone, Copy, Debug)]
struct Vertex {
    /// Successor in the 2D convex polygon `computeInternal` maintains while
    /// merging, not a general list link.
    next: VId,
    /// Predecessor in that polygon.
    prev: VId,
    /// Head of this vertex's circular, clockwise edge ring.
    edges: EId,
    first_nearby_face: FId,
    last_nearby_face: FId,
    point128: PointR128,
    point: Point32,
    /// Doubles as a merge stamp during `shrink` and as the output index in
    /// `get_vertex_copy`, which is why it starts at `-1` and why the output
    /// stage tests `< 0`.
    copy: i32,
}

impl Vertex {
    /// `Vertex()` (`:440-442`). `point` and `point128` are left uninitialised
    /// upstream; both are written before any read.
    const NEW: Self = Self {
        next: NIL,
        prev: NIL,
        edges: NIL,
        first_nearby_face: NIL,
        last_nearby_face: NIL,
        point128: PointR128::ZERO,
        point: Point32::new(0, 0, 0),
        copy: -1,
    };

    /// `btScalar xvalue() const` (`:464-467`).
    fn xvalue(&self) -> Scalar {
        if self.point.index >= 0 {
            self.point.x as Scalar
        } else {
            self.point128.xvalue()
        }
    }

    /// `btScalar yvalue() const` (`:469-472`).
    fn yvalue(&self) -> Scalar {
        if self.point.index >= 0 {
            self.point.y as Scalar
        } else {
            self.point128.yvalue()
        }
    }

    /// `btScalar zvalue() const` (`:474-477`).
    fn zvalue(&self) -> Scalar {
        if self.point.index >= 0 {
            self.point.z as Scalar
        } else {
            self.point128.zvalue()
        }
    }

    /// `Rational128 dot(const Point64& b) const` (`:458-462`).
    ///
    /// An original input point is on the integer grid, so its plane distance is
    /// an exact `i64` and the rational is trivial; a point synthesised by
    /// [`HullInternal::shift_face`] is not, and its distance is the 128-bit
    /// rational built from `point128`. Both readings have to be exact, because
    /// the caller compares them against each other.
    fn dot(&self, b: Point64) -> Rational128 {
        if self.point.index >= 0 {
            Rational128::from_i64(self.point.dot_p64(b))
        } else {
            Rational128::from_ratio(
                self.point128.x.mul_by_i64(b.x)
                    + self.point128.y.mul_by_i64(b.y)
                    + self.point128.z.mul_by_i64(b.z),
                self.point128.denominator,
            )
        }
    }
}

/// `btConvexHullInternal::Edge` (`:503-536`) -- one half-edge.
///
/// `next`/`prev` walk the *source vertex's* edge ring, not the face; the face
/// loop is `e->reverse->prev` (see [`ConvexHullComputer::next_edge_of_face`]).
#[derive(Clone, Copy, Debug)]
struct InternalEdge {
    next: EId,
    prev: EId,
    reverse: EId,
    target: VId,
    face: FId,
    /// The merge stamp the pair was created under. `findMaxAngle` skips edges
    /// with `copy == merge_stamp` -- the ones this merge round created -- and
    /// the output stage reuses the field as the emitted edge index.
    copy: i32,
}

impl InternalEdge {
    /// The state `new (o) Edge()` leaves behind. `Edge` has no user-provided
    /// constructor, so that placement-new *value*-initialises: every pointer
    /// becomes `NULL` and `copy` becomes `0`, which is not the `-1` a
    /// default-constructed `Vertex` gets.
    const ZERO: Self = Self {
        next: NIL,
        prev: NIL,
        reverse: NIL,
        target: NIL,
        face: NIL,
        copy: 0,
    };
}

/// `btConvexHullInternal::Face` (`:538-573`) -- a face as an origin and two
/// edge directions, built only by [`HullInternal::shrink`].
#[derive(Clone, Copy, Debug)]
struct Face {
    nearby_vertex: VId,
    next_with_same_nearby_vertex: FId,
    origin: Point32,
    dir0: Point32,
    dir1: Point32,
}

impl Face {
    /// `Face()` (`:548-550`). `origin`/`dir0`/`dir1` are uninitialised upstream
    /// and are written by `init` before use.
    const NEW: Self = Self {
        nearby_vertex: NIL,
        next_with_same_nearby_vertex: NIL,
        origin: Point32::new(0, 0, 0),
        dir0: Point32::new(0, 0, 0),
        dir1: Point32::new(0, 0, 0),
    };

    /// `Point64 getNormal()` (`:569-572`) -- unnormalised, and its length
    /// carries the face's area, which is why `shiftFace` can use it directly in
    /// integer dot products.
    fn get_normal(&self) -> Point64 {
        self.dir0.cross(self.dir1)
    }
}

/// `btConvexHullInternal::Orientation` (`:658-663`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Orientation {
    /// The two edges are not adjacent in the ring.
    None,
    /// `next` follows `prev` clockwise.
    Clockwise,
    /// `next` follows `prev` counter-clockwise.
    CounterClockwise,
}

/// `btConvexHullInternal::IntermediateHull` (`:643-656`) -- the four extreme
/// vertices of a partial hull's 2D projection, which is all `merge` needs to
/// find where two partial hulls touch.
#[derive(Clone, Copy, Debug)]
struct IntermediateHull {
    min_xy: VId,
    max_xy: VId,
    min_yx: VId,
    max_yx: VId,
}

impl IntermediateHull {
    /// `IntermediateHull()` (`:651-653`).
    const EMPTY: Self = Self {
        min_xy: NIL,
        max_xy: NIL,
        min_yx: NIL,
        max_yx: NIL,
    };
}

// ---------------------------------------------------------------------------
// btConvexHullInternal
// ---------------------------------------------------------------------------

/// `btConvexHullInternal` (`:50-837`) -- the hull itself, on the integer grid.
///
/// Everything public about this port is [`ConvexHullComputer`]; this is the
/// working structure it drives and then reads out.
///
/// # Not carried
///
/// `usedEdgePairs` and `maxUsedEdgePairs` (`:772-773`) exist only to feed the
/// `printf` under `DEBUG_CONVEX_HULL` at the end of `compute` (`:2072-2074`);
/// nothing reads them otherwise, and a field written but never read is a lint
/// error here. `Face::next` and `Vertex::next`'s pool role likewise: upstream's
/// `Pool<T>` reuses each object's `next` as the free-list link, but only the
/// edge pool ever frees anything, so only [`InternalEdge`] needs it.
struct HullInternal {
    /// `scaling` -- grid units back to metres, per axis, and negated on one
    /// axis when the axis permutation below is odd.
    scaling: Vec3,
    /// `center` -- the input AABB's centre, subtracted before quantizing.
    center: Vec3,
    /// The vertex pool. Nothing is ever freed, so index `i` is the `i`-th
    /// `newObject()` -- which is what makes upstream's `Vertex* w = v + 1`
    /// (`:1219`) meaningful as `original_vertices[start + 1]`.
    verts: Vec<Vertex>,
    /// The edge pool's backing store. Slots are recycled; see
    /// [`HullInternal::new_edge_object`].
    edges: Vec<InternalEdge>,
    /// `Pool<Edge>::freeObjects` -- head of the LIFO free list.
    edge_free: EId,
    /// `Pool<Edge>::arraySize` -- `6 * count`, the block size a pool exhaustion
    /// allocates.
    edge_array_size: usize,
    /// The face pool. Only `shrink` allocates from it, and never frees.
    faces: Vec<Face>,
    /// `originalVertices` -- the quantized input in sorted order.
    original_vertices: Vec<VId>,
    /// `mergeStamp`, counting down from `-3`. Negative so that it can share
    /// `Edge::copy` with the output stage's non-negative indices.
    merge_stamp: i32,
    /// Which input axis the grid's `z` came from -- the *shortest* extent.
    min_axis: usize,
    /// Which input axis the grid's `x` came from.
    med_axis: usize,
    /// Which input axis the grid's `y` came from -- the *longest* extent.
    max_axis: usize,
    /// `vertexList` -- the vertex the output walk starts from.
    vertex_list: VId,
}

/// `btSetMin` (`btMinMax.h:38-45`) applied component-wise, which is
/// `btVector3::setMin` in the scalar build (`btVector3.h:626-638`).
///
/// Not in `linear_math` because nothing else in this crate reaches it.
fn set_min(a: &mut Vec3, b: Vec3) {
    if b.x < a.x {
        a.x = b.x;
    }
    if b.y < a.y {
        a.y = b.y;
    }
    if b.z < a.z {
        a.z = b.z;
    }
}

/// `btSetMax` / `btVector3::setMax` (`btVector3.h:609-621`).
fn set_max(a: &mut Vec3, b: Vec3) {
    if a.x < b.x {
        a.x = b.x;
    }
    if a.y < b.y {
        a.y = b.y;
    }
    if a.z < b.z {
        a.z = b.z;
    }
}

/// `btAlignedObjectArray<Point32>::quickSort(pointCmp())`
/// (`btAlignedObjectArray.h:311-348`, comparator at
/// `btConvexHullComputer.cpp:1947-1954`).
///
/// Ported rather than replaced by `sort_by`/`sort_unstable_by` because it is
/// not a stable sort and its comparator ignores `Point32::index`: points that
/// quantize onto the same grid cell compare equal, and *which* of them ends up
/// first decides which input index the surviving output vertex reports and
/// which duplicates `computeInternal`'s split skips. Two different unstable
/// sorts agree on the sorted sequence and disagree on that.
///
/// The comparator is lexicographic on `(y, x, z)` -- `y` first, because `y` is
/// the longest extent and the divide-and-conquer splits along it.
fn quick_sort(data: &mut [Point32]) {
    fn less(p: Point32, q: Point32) -> bool {
        (p.y < q.y) || ((p.y == q.y) && ((p.x < q.x) || ((p.x == q.x) && (p.z < q.z))))
    }

    fn internal(data: &mut [Point32], lo: i32, hi: i32) {
        let mut i = lo;
        let mut j = hi;
        // A *copy* of the middle element, so the swaps below cannot move the
        // pivot value out from under the partition loop.
        let x = data[((lo + hi) / 2) as usize];

        loop {
            while less(data[i as usize], x) {
                i += 1;
            }
            while less(x, data[j as usize]) {
                j -= 1;
            }
            if i <= j {
                data.swap(i as usize, j as usize);
                i += 1;
                j -= 1;
            }
            if i > j {
                break;
            }
        }

        if lo < j {
            internal(data, lo, j);
        }
        if i < hi {
            internal(data, i, hi);
        }
    }

    if data.len() > 1 {
        let hi = (data.len() - 1) as i32;
        internal(data, 0, hi);
    }
}

impl HullInternal {
    /// A pool-empty hull. `mergeStamp` and the axis fields are set by
    /// [`HullInternal::compute`] before anything reads them.
    fn new() -> Self {
        Self {
            scaling: Vec3::zero(),
            center: Vec3::zero(),
            verts: Vec::new(),
            edges: Vec::new(),
            edge_free: NIL,
            edge_array_size: 256,
            faces: Vec::new(),
            original_vertices: Vec::new(),
            merge_stamp: -3,
            min_axis: 0,
            med_axis: 0,
            max_axis: 0,
            vertex_list: NIL,
        }
    }

    fn vert(&self, v: VId) -> &Vertex {
        &self.verts[v as usize]
    }

    fn vert_mut(&mut self, v: VId) -> &mut Vertex {
        &mut self.verts[v as usize]
    }

    fn edge(&self, e: EId) -> &InternalEdge {
        &self.edges[e as usize]
    }

    fn edge_mut(&mut self, e: EId) -> &mut InternalEdge {
        &mut self.edges[e as usize]
    }

    fn face(&self, f: FId) -> &Face {
        &self.faces[f as usize]
    }

    fn face_mut(&mut self, f: FId) -> &mut Face {
        &mut self.faces[f as usize]
    }

    /// `Pool<Vertex>::newObject()` (`:732-752`).
    fn new_vertex_object(&mut self) -> VId {
        let id = self.verts.len() as VId;
        self.verts.push(Vertex::NEW);
        id
    }

    /// `Pool<Face>::newObject()`.
    fn new_face_object(&mut self) -> FId {
        let id = self.faces.len() as FId;
        self.faces.push(Face::NEW);
        id
    }

    /// `Pool<Edge>::newObject()` (`:732-752`) -- pop the free list, or carve a
    /// fresh block of `arraySize` when it is empty.
    ///
    /// The recycling is load-bearing, not an allocation detail. `merge` frees
    /// edge pairs and allocates new ones in the same loop while holding raw
    /// `Edge*` values (`toPrev0`, `firstNew0`, `min0`) across both, and every
    /// one of its exits is an identity comparison against one of them. Upstream
    /// pops the most recently freed slot; a port that appended a fresh index
    /// instead would give those comparisons different answers.
    ///
    /// `p->init()` links the fresh block in ascending order and hands back its
    /// first element, so the free list after a block allocation is
    /// `base+1, base+2, ...` -- reproduced here rather than reversed.
    fn new_edge_object(&mut self) -> EId {
        if self.edge_free == NIL {
            let base = self.edges.len() as EId;
            let size = self.edge_array_size as EId;
            for k in 0..size {
                let next = if k + 1 < size { base + k + 1 } else { NIL };
                self.edges.push(InternalEdge {
                    next,
                    prev: NIL,
                    reverse: NIL,
                    target: NIL,
                    face: NIL,
                    copy: 0,
                });
            }
            self.edge_free = base;
        }
        let o = self.edge_free;
        self.edge_free = self.edge(o).next;
        // `new (o) Edge()` -- value-initialisation, so a recycled slot cannot
        // carry a field over from its previous life.
        self.edges[o as usize] = InternalEdge::ZERO;
        o
    }

    /// `Pool<Edge>::freeObject()` (`:754-759`), including `~Edge()`
    /// (`:513-520`), which nulls every link before the slot joins the free
    /// list. `copy` is deliberately *not* cleared -- upstream's destructor
    /// leaves it, and the value-initialisation on reallocation is what resets
    /// it.
    fn free_edge_object(&mut self, e: EId) {
        let head = self.edge_free;
        let edge = self.edge_mut(e);
        edge.prev = NIL;
        edge.reverse = NIL;
        edge.target = NIL;
        edge.face = NIL;
        edge.next = head;
        self.edge_free = e;
    }

    /// `Edge::link(Edge* n)` (`:522-527`).
    fn link(&mut self, e: EId, n: EId) {
        self.edge_mut(e).next = n;
        self.edge_mut(n).prev = e;
    }

    /// `Face::init(Vertex* a, Vertex* b, Vertex* c)` (`:552-567`) -- the face
    /// through `a` spanned by `b - a` and `c - a`, appended to `a`'s nearby-face
    /// list.
    fn face_init(&mut self, f: FId, a: VId, b: VId, c: VId) {
        let origin = self.vert(a).point;
        let dir0 = self.vert(b).point - origin;
        let dir1 = self.vert(c).point - origin;
        let face = self.face_mut(f);
        face.nearby_vertex = a;
        face.origin = origin;
        face.dir0 = dir0;
        face.dir1 = dir1;
        let last = self.vert(a).last_nearby_face;
        if last != NIL {
            self.face_mut(last).next_with_same_nearby_vertex = f;
        } else {
            self.vert_mut(a).first_nearby_face = f;
        }
        self.vert_mut(a).last_nearby_face = f;
    }

    /// `Vertex::receiveNearbyFaces(Vertex* src)` (`:479-500`) -- splice `src`'s
    /// nearby-face list onto `dst`'s and repoint every face at `dst`.
    fn receive_nearby_faces(&mut self, dst: VId, src: VId) {
        let src_first = self.vert(src).first_nearby_face;
        let last = self.vert(dst).last_nearby_face;
        if last != NIL {
            self.face_mut(last).next_with_same_nearby_vertex = src_first;
        } else {
            self.vert_mut(dst).first_nearby_face = src_first;
        }
        let src_last = self.vert(src).last_nearby_face;
        if src_last != NIL {
            self.vert_mut(dst).last_nearby_face = src_last;
        }
        let mut f = src_first;
        while f != NIL {
            self.face_mut(f).nearby_vertex = dst;
            f = self.face(f).next_with_same_nearby_vertex;
        }
        self.vert_mut(src).first_nearby_face = NIL;
        self.vert_mut(src).last_nearby_face = NIL;
    }

    /// `newEdgePair(Vertex* from, Vertex* to)` (`:999-1018`) -- a half-edge and
    /// its reverse, stamped with the current merge round.
    fn new_edge_pair(&mut self, from: VId, to: VId) -> EId {
        let e = self.new_edge_object();
        let r = self.new_edge_object();
        let stamp = self.merge_stamp;
        {
            let edge = self.edge_mut(e);
            edge.reverse = r;
            edge.copy = stamp;
            edge.target = to;
            edge.face = NIL;
        }
        {
            let edge = self.edge_mut(r);
            edge.reverse = e;
            edge.copy = stamp;
            edge.target = from;
            edge.face = NIL;
        }
        e
    }

    /// `removeEdgePair(Edge* edge)` (`:781-815`) -- unlink both halves from
    /// their source vertices' rings and return them to the pool.
    fn remove_edge_pair(&mut self, e: EId) {
        let r = self.edge(e).reverse;

        let n = self.edge(e).next;
        if n != e {
            let p = self.edge(e).prev;
            self.edge_mut(n).prev = p;
            self.edge_mut(p).next = n;
            let target = self.edge(r).target;
            self.vert_mut(target).edges = n;
        } else {
            let target = self.edge(r).target;
            self.vert_mut(target).edges = NIL;
        }

        let n = self.edge(r).next;
        if n != r {
            let p = self.edge(r).prev;
            self.edge_mut(n).prev = p;
            self.edge_mut(p).next = n;
            let target = self.edge(e).target;
            self.vert_mut(target).edges = n;
        } else {
            let target = self.edge(e).target;
            self.vert_mut(target).edges = NIL;
        }

        self.free_edge_object(e);
        self.free_edge_object(r);
    }

    /// `getOrientation(prev, next, s, t)` (`:1399-1423`).
    ///
    /// `prev` and `next` share a source vertex. If they are adjacent in that
    /// vertex's ring in one direction the answer is that direction; if they are
    /// adjacent in *both* -- a two-edge ring, where "next" and "previous" are
    /// the same neighbour -- the tie is broken by the sign of `(t x s)` against
    /// the ring's own plane normal, which is the only case that needs
    /// arithmetic at all.
    fn get_orientation(&self, prev: EId, next: EId, s: Point32, t: Point32) -> Orientation {
        if self.edge(prev).next == next {
            if self.edge(prev).prev == next {
                let n = t.cross(s);
                // `next->reverse->target` is the vertex `prev` and `next` share.
                let origin = self.vert(self.edge(self.edge(next).reverse).target).point;
                let m = (self.vert(self.edge(prev).target).point - origin)
                    .cross(self.vert(self.edge(next).target).point - origin);
                let dot = n.dot(m);
                return if dot > 0 {
                    Orientation::CounterClockwise
                } else {
                    Orientation::Clockwise
                };
            }
            Orientation::CounterClockwise
        } else if self.edge(prev).prev == next {
            Orientation::Clockwise
        } else {
            Orientation::None
        }
    }

    /// `findMaxAngle(ccw, start, s, rxs, sxrxs, minCot)` (`:1425-1475`) -- the
    /// edge of `start` that turns least around the current bridge direction.
    ///
    /// The "angle" is a cotangent held as an exact [`Rational64`], never an
    /// `atan2`: `t.dot(sxrxs) / t.dot(rxs)` compared by cross-multiplication.
    /// A zero denominator with a zero numerator ([`Rational64::is_nan`]) is an
    /// edge pointing straight back along `s`, which is skipped -- upstream
    /// asserts it points the way the walk came from and otherwise ignores it.
    ///
    /// Edges created during the current merge round (`copy == merge_stamp`) are
    /// skipped, so the walk cannot follow a bridge it just built.
    fn find_max_angle(
        &self,
        ccw: bool,
        start: VId,
        s: Point32,
        rxs: Point64,
        sxrxs: Point64,
        min_cot: &mut Rational64,
    ) -> EId {
        let mut min_edge = NIL;
        let first = self.vert(start).edges;
        if first != NIL {
            let mut e = first;
            loop {
                if self.edge(e).copy > self.merge_stamp {
                    let t = self.vert(self.edge(e).target).point - self.vert(start).point;
                    let cot = Rational64::new(t.dot_p64(sxrxs), t.dot_p64(rxs));
                    if !cot.is_nan() {
                        if min_edge == NIL {
                            *min_cot = cot;
                            min_edge = e;
                        } else {
                            let cmp = cot.compare(*min_cot);
                            if cmp < 0 {
                                *min_cot = cot;
                                min_edge = e;
                            } else if (cmp == 0)
                                && (ccw
                                    == (self.get_orientation(min_edge, e, s, t)
                                        == Orientation::CounterClockwise))
                            {
                                min_edge = e;
                            }
                        }
                    }
                }
                e = self.edge(e).next;
                if e == first {
                    break;
                }
            }
        }
        min_edge
    }

    /// `mergeProjection(h0, h1, c0, c1)` (`:1020-1203`) -- merge the two
    /// partial hulls' 2D `(x, y)` projections and report the bridge's endpoints.
    ///
    /// Returns `false` for the one degenerate case it handles specially: the
    /// two hulls' touching vertices project onto the same `(x, y)` point, so
    /// there is no bridge in the projection at all and `merge` starts from a
    /// vertical direction instead. That is the return value's whole meaning --
    /// it is not a failure.
    fn merge_projection(
        &mut self,
        h0: &mut IntermediateHull,
        h1: &mut IntermediateHull,
        c0: &mut VId,
        c1: &mut VId,
    ) -> bool {
        let mut v0 = h0.max_yx;
        let mut v1 = h1.min_yx;
        if (self.vert(v0).point.x == self.vert(v1).point.x)
            && (self.vert(v0).point.y == self.vert(v1).point.y)
        {
            // Upstream asserts `v0->point.z < v1->point.z` here.
            let v1p = self.vert(v1).prev;
            if v1p == v1 {
                *c0 = v0;
                if self.vert(v1).edges != NIL {
                    // A lone vertex stacked directly above `v0`: it has exactly
                    // one edge, and the far end of it is the vertex to bridge
                    // from.
                    v1 = self.edge(self.vert(v1).edges).target;
                }
                *c1 = v1;
                return false;
            }
            let v1n = self.vert(v1).next;
            self.vert_mut(v1p).next = v1n;
            self.vert_mut(v1n).prev = v1p;
            if v1 == h1.min_xy {
                if (self.vert(v1n).point.x < self.vert(v1p).point.x)
                    || ((self.vert(v1n).point.x == self.vert(v1p).point.x)
                        && (self.vert(v1n).point.y < self.vert(v1p).point.y))
                {
                    h1.min_xy = v1n;
                } else {
                    h1.min_xy = v1p;
                }
            }
            if v1 == h1.max_xy {
                if (self.vert(v1n).point.x > self.vert(v1p).point.x)
                    || ((self.vert(v1n).point.x == self.vert(v1p).point.x)
                        && (self.vert(v1n).point.y > self.vert(v1p).point.y))
                {
                    h1.max_xy = v1n;
                } else {
                    h1.max_xy = v1p;
                }
            }
        }

        v0 = h0.max_xy;
        v1 = h1.max_xy;
        let mut v00 = NIL;
        let mut v10 = NIL;
        let mut sign: i32 = 1;

        for side in 0..=1 {
            let mut dx = (self.vert(v1).point.x - self.vert(v0).point.x) * sign;
            if dx > 0 {
                loop {
                    let dy = self.vert(v1).point.y - self.vert(v0).point.y;

                    let w0 = if side != 0 {
                        self.vert(v0).next
                    } else {
                        self.vert(v0).prev
                    };
                    if w0 != v0 {
                        let dx0 = (self.vert(w0).point.x - self.vert(v0).point.x) * sign;
                        let dy0 = self.vert(w0).point.y - self.vert(v0).point.y;
                        if (dy0 <= 0) && ((dx0 == 0) || ((dx0 < 0) && (dy0 * dx <= dy * dx0))) {
                            v0 = w0;
                            dx = (self.vert(v1).point.x - self.vert(v0).point.x) * sign;
                            continue;
                        }
                    }

                    let w1 = if side != 0 {
                        self.vert(v1).next
                    } else {
                        self.vert(v1).prev
                    };
                    if w1 != v1 {
                        let dx1 = (self.vert(w1).point.x - self.vert(v1).point.x) * sign;
                        let dy1 = self.vert(w1).point.y - self.vert(v1).point.y;
                        let dxn = (self.vert(w1).point.x - self.vert(v0).point.x) * sign;
                        if (dxn > 0)
                            && (dy1 < 0)
                            && ((dx1 == 0) || ((dx1 < 0) && (dy1 * dx < dy * dx1)))
                        {
                            v1 = w1;
                            dx = dxn;
                            continue;
                        }
                    }

                    break;
                }
            } else if dx < 0 {
                loop {
                    let dy = self.vert(v1).point.y - self.vert(v0).point.y;

                    let w1 = if side != 0 {
                        self.vert(v1).prev
                    } else {
                        self.vert(v1).next
                    };
                    if w1 != v1 {
                        let dx1 = (self.vert(w1).point.x - self.vert(v1).point.x) * sign;
                        let dy1 = self.vert(w1).point.y - self.vert(v1).point.y;
                        if (dy1 >= 0) && ((dx1 == 0) || ((dx1 < 0) && (dy1 * dx <= dy * dx1))) {
                            v1 = w1;
                            dx = (self.vert(v1).point.x - self.vert(v0).point.x) * sign;
                            continue;
                        }
                    }

                    let w0 = if side != 0 {
                        self.vert(v0).prev
                    } else {
                        self.vert(v0).next
                    };
                    if w0 != v0 {
                        let dx0 = (self.vert(w0).point.x - self.vert(v0).point.x) * sign;
                        let dy0 = self.vert(w0).point.y - self.vert(v0).point.y;
                        let dxn = (self.vert(v1).point.x - self.vert(w0).point.x) * sign;
                        if (dxn < 0)
                            && (dy0 > 0)
                            && ((dx0 == 0) || ((dx0 < 0) && (dy0 * dx < dy * dx0)))
                        {
                            v0 = w0;
                            dx = dxn;
                            continue;
                        }
                    }

                    break;
                }
            } else {
                // Same projected `x`: slide each side along its vertical run to
                // the extreme `y`, which is the tangent point when the tangent
                // is vertical.
                let x = self.vert(v0).point.x;
                let mut y0 = self.vert(v0).point.y;
                let mut w0 = v0;
                loop {
                    let t = if side != 0 {
                        self.vert(w0).next
                    } else {
                        self.vert(w0).prev
                    };
                    if (t == v0) || (self.vert(t).point.x != x) || (self.vert(t).point.y > y0) {
                        break;
                    }
                    w0 = t;
                    y0 = self.vert(t).point.y;
                }
                v0 = w0;

                let mut y1 = self.vert(v1).point.y;
                let mut w1 = v1;
                loop {
                    let t = if side != 0 {
                        self.vert(w1).prev
                    } else {
                        self.vert(w1).next
                    };
                    if (t == v1) || (self.vert(t).point.x != x) || (self.vert(t).point.y < y1) {
                        break;
                    }
                    w1 = t;
                    y1 = self.vert(t).point.y;
                }
                v1 = w1;
            }

            if side == 0 {
                v00 = v0;
                v10 = v1;

                v0 = h0.min_xy;
                v1 = h1.min_xy;
                sign = -1;
            }
        }

        self.vert_mut(v0).prev = v1;
        self.vert_mut(v1).next = v0;

        self.vert_mut(v00).next = v10;
        self.vert_mut(v10).prev = v00;

        if self.vert(h1.min_xy).point.x < self.vert(h0.min_xy).point.x {
            h0.min_xy = h1.min_xy;
        }
        if self.vert(h1.max_xy).point.x >= self.vert(h0.max_xy).point.x {
            h0.max_xy = h1.max_xy;
        }

        h0.max_yx = h1.max_yx;

        *c0 = v00;
        *c1 = v10;

        true
    }

    /// `findEdgeForCoplanarFaces(c0, c1, e0, e1, stop0, stop1)` (`:1477-1658`).
    ///
    /// When the bridging plane contains faces of *both* partial hulls, the
    /// bridge is not determined by the turn angle -- every candidate ties. This
    /// walks both coplanar face boundaries in the plane and advances `e0`/`e1`
    /// to the pair that actually forms the hull edge there. Without it, a merge
    /// across two coplanar faces picks an arbitrary tied edge and produces a
    /// non-convex result; with it, the tie is resolved by an exact
    /// [`Rational64`] slope comparison inside the plane.
    ///
    /// `e0`/`e1` are in-out: upstream takes them by reference and the caller
    /// reads the advanced values back.
    fn find_edge_for_coplanar_faces(
        &self,
        c0: VId,
        c1: VId,
        e0: &mut EId,
        e1: &mut EId,
        stop0: VId,
        stop1: VId,
    ) {
        let start0 = *e0;
        let start1 = *e1;
        let mut et0 = if start0 != NIL {
            self.vert(self.edge(start0).target).point
        } else {
            self.vert(c0).point
        };
        let mut et1 = if start1 != NIL {
            self.vert(self.edge(start1).target).point
        } else {
            self.vert(c1).point
        };
        let s = self.vert(c1).point - self.vert(c0).point;
        let seed = if start0 != NIL { start0 } else { start1 };
        let normal = (self.vert(self.edge(seed).target).point - self.vert(c0).point).cross(s);
        let dist = self.vert(c0).point.dot_p64(normal);
        let perp = s.cross_p64(normal);

        let mut max_dot0 = et0.dot_p64(perp);
        if *e0 != NIL {
            while self.edge(*e0).target != stop0 {
                let e = self.edge(self.edge(*e0).reverse).prev;
                if self.vert(self.edge(e).target).point.dot_p64(normal) < dist {
                    break;
                }
                if self.edge(e).copy == self.merge_stamp {
                    break;
                }
                let dot = self.vert(self.edge(e).target).point.dot_p64(perp);
                if dot <= max_dot0 {
                    break;
                }
                max_dot0 = dot;
                *e0 = e;
                et0 = self.vert(self.edge(e).target).point;
            }
        }

        let mut max_dot1 = et1.dot_p64(perp);
        if *e1 != NIL {
            while self.edge(*e1).target != stop1 {
                let e = self.edge(self.edge(*e1).reverse).next;
                if self.vert(self.edge(e).target).point.dot_p64(normal) < dist {
                    break;
                }
                if self.edge(e).copy == self.merge_stamp {
                    break;
                }
                let dot = self.vert(self.edge(e).target).point.dot_p64(perp);
                if dot <= max_dot1 {
                    break;
                }
                max_dot1 = dot;
                *e1 = e;
                et1 = self.vert(self.edge(e).target).point;
            }
        }

        let mut dx = max_dot1 - max_dot0;
        if dx > 0 {
            loop {
                let dy = (et1 - et0).dot(s);

                if (*e0 != NIL) && (self.edge(*e0).target != stop0) {
                    let f0 = self.edge(self.edge(*e0).next).reverse;
                    if self.edge(f0).copy > self.merge_stamp {
                        let d0 = self.vert(self.edge(f0).target).point - et0;
                        let dx0 = d0.dot_p64(perp);
                        let dy0 = d0.dot(s);
                        if if dx0 == 0 {
                            dy0 < 0
                        } else {
                            (dx0 < 0)
                                && (Rational64::new(dy0, dx0).compare(Rational64::new(dy, dx)) >= 0)
                        } {
                            et0 = self.vert(self.edge(f0).target).point;
                            dx = (et1 - et0).dot_p64(perp);
                            *e0 = if *e0 == start0 { NIL } else { f0 };
                            continue;
                        }
                    }
                }

                if (*e1 != NIL) && (self.edge(*e1).target != stop1) {
                    let f1 = self.edge(self.edge(*e1).reverse).next;
                    if self.edge(f1).copy > self.merge_stamp {
                        let d1 = self.vert(self.edge(f1).target).point - et1;
                        if d1.dot_p64(normal) == 0 {
                            let dx1 = d1.dot_p64(perp);
                            let dy1 = d1.dot(s);
                            let dxn = (self.vert(self.edge(f1).target).point - et0).dot_p64(perp);
                            if (dxn > 0)
                                && (if dx1 == 0 {
                                    dy1 < 0
                                } else {
                                    (dx1 < 0)
                                        && (Rational64::new(dy1, dx1)
                                            .compare(Rational64::new(dy, dx))
                                            > 0)
                                })
                            {
                                *e1 = f1;
                                et1 = self.vert(self.edge(*e1).target).point;
                                dx = dxn;
                                continue;
                            }
                        }
                        // else: upstream asserts `e1 == start1` and that the
                        // step leaves the plane downwards.
                    }
                }

                break;
            }
        } else if dx < 0 {
            loop {
                let dy = (et1 - et0).dot(s);

                if (*e1 != NIL) && (self.edge(*e1).target != stop1) {
                    let f1 = self.edge(self.edge(*e1).prev).reverse;
                    if self.edge(f1).copy > self.merge_stamp {
                        let d1 = self.vert(self.edge(f1).target).point - et1;
                        let dx1 = d1.dot_p64(perp);
                        let dy1 = d1.dot(s);
                        if if dx1 == 0 {
                            dy1 > 0
                        } else {
                            (dx1 < 0)
                                && (Rational64::new(dy1, dx1).compare(Rational64::new(dy, dx)) <= 0)
                        } {
                            et1 = self.vert(self.edge(f1).target).point;
                            dx = (et1 - et0).dot_p64(perp);
                            *e1 = if *e1 == start1 { NIL } else { f1 };
                            continue;
                        }
                    }
                }

                if (*e0 != NIL) && (self.edge(*e0).target != stop0) {
                    let f0 = self.edge(self.edge(*e0).reverse).prev;
                    if self.edge(f0).copy > self.merge_stamp {
                        let d0 = self.vert(self.edge(f0).target).point - et0;
                        if d0.dot_p64(normal) == 0 {
                            let dx0 = d0.dot_p64(perp);
                            let dy0 = d0.dot(s);
                            let dxn = (et1 - self.vert(self.edge(f0).target).point).dot_p64(perp);
                            if (dxn < 0)
                                && (if dx0 == 0 {
                                    dy0 > 0
                                } else {
                                    (dx0 < 0)
                                        && (Rational64::new(dy0, dx0)
                                            .compare(Rational64::new(dy, dx))
                                            < 0)
                                })
                            {
                                *e0 = f0;
                                et0 = self.vert(self.edge(*e0).target).point;
                                dx = dxn;
                                continue;
                            }
                        }
                    }
                }

                break;
            }
        }
    }

    /// `computeInternal(start, end, result)` (`:1205-1332`) -- the
    /// divide-and-conquer recursion, on the sorted point range `[start, end)`.
    fn compute_internal(&mut self, start: usize, end: usize, result: &mut IntermediateHull) {
        let n = end - start;
        match n {
            0 => {
                *result = IntermediateHull::EMPTY;
                return;
            }
            2 => {
                let mut v = self.original_vertices[start];
                // `Vertex* w = v + 1` (`:1219`). The vertex pool hands out one
                // contiguous block in `newObject()` order and `compute` fills
                // `original_vertices` in that same order, so `v + 1` is
                // `original_vertices[start + 1]`.
                let mut w = v + 1;
                if self.vert(v).point != self.vert(w).point {
                    let dx = self.vert(v).point.x - self.vert(w).point.x;
                    let dy = self.vert(v).point.y - self.vert(w).point.y;

                    if (dx == 0) && (dy == 0) {
                        // Two points differing only in `z`. The swap is
                        // unreachable in practice: the sort is lexicographic on
                        // `(y, x, z)`, so equal `x` and `y` already implies
                        // ascending `z` across the pair.
                        if self.vert(v).point.z > self.vert(w).point.z {
                            core::mem::swap(&mut v, &mut w);
                        }
                        self.vert_mut(v).next = v;
                        self.vert_mut(v).prev = v;
                        result.min_xy = v;
                        result.max_xy = v;
                        result.min_yx = v;
                        result.max_yx = v;
                    } else {
                        self.vert_mut(v).next = w;
                        self.vert_mut(v).prev = w;
                        self.vert_mut(w).next = v;
                        self.vert_mut(w).prev = v;

                        if (dx < 0) || ((dx == 0) && (dy < 0)) {
                            result.min_xy = v;
                            result.max_xy = w;
                        } else {
                            result.min_xy = w;
                            result.max_xy = v;
                        }

                        if (dy < 0) || ((dy == 0) && (dx < 0)) {
                            result.min_yx = v;
                            result.max_yx = w;
                        } else {
                            result.min_yx = w;
                            result.max_yx = v;
                        }
                    }

                    let e = self.new_edge_pair(v, w);
                    self.link(e, e);
                    self.vert_mut(v).edges = e;

                    let e = self.edge(e).reverse;
                    self.link(e, e);
                    self.vert_mut(w).edges = e;

                    return;
                }
                // The two points quantized onto the same cell: keep the first
                // and drop the second entirely -- it never gets an edge, and
                // never reaches the output.
                let v = self.original_vertices[start];
                self.vert_mut(v).edges = NIL;
                self.vert_mut(v).next = v;
                self.vert_mut(v).prev = v;

                result.min_xy = v;
                result.max_xy = v;
                result.min_yx = v;
                result.max_yx = v;
                return;
            }
            1 => {
                let v = self.original_vertices[start];
                self.vert_mut(v).edges = NIL;
                self.vert_mut(v).next = v;
                self.vert_mut(v).prev = v;

                result.min_xy = v;
                result.max_xy = v;
                result.min_yx = v;
                result.max_yx = v;
                return;
            }
            _ => {}
        }

        let split0 = start + n / 2;
        let p = self.vert(self.original_vertices[split0 - 1]).point;
        let mut split1 = split0;
        // Duplicates of the split point go to neither half: the left half keeps
        // the first copy, and `[split0, split1)` is dropped outright.
        while (split1 < end) && (self.vert(self.original_vertices[split1]).point == p) {
            split1 += 1;
        }
        self.compute_internal(start, split0, result);
        let mut hull1 = IntermediateHull::EMPTY;
        self.compute_internal(split1, end, &mut hull1);
        self.merge(result, &mut hull1);
    }

    /// `merge(h0, h1)` (`:1660-1945`) -- the gift-wrapping walk that stitches
    /// two partial hulls into one.
    ///
    /// It walks a bridge around the two hulls, at each step asking both sides
    /// (via [`HullInternal::find_max_angle`]) for the edge that turns least,
    /// and advancing whichever wins -- or both, when they tie exactly. New
    /// bridge edges are accumulated in the `pending` lists and spliced into the
    /// vertices' edge rings only once the walk has passed them, because the
    /// walk itself must not see them; edges the bridge cut off are freed on the
    /// way past.
    fn merge(&mut self, h0: &mut IntermediateHull, h1: &mut IntermediateHull) {
        if h1.max_xy == NIL {
            return;
        }
        if h0.max_xy == NIL {
            *h0 = *h1;
            return;
        }

        self.merge_stamp -= 1;

        let mut c0 = NIL;
        let mut to_prev0 = NIL;
        let mut first_new0 = NIL;
        let mut pending_head0 = NIL;
        let mut pending_tail0 = NIL;
        let mut c1 = NIL;
        let mut to_prev1 = NIL;
        let mut first_new1 = NIL;
        let mut pending_head1 = NIL;
        let mut pending_tail1 = NIL;
        let mut prev_point;

        if self.merge_projection(h0, h1, &mut c0, &mut c1) {
            let s = self.vert(c1).point - self.vert(c0).point;
            let down = Point32::new(0, 0, -1);
            let normal = down.cross(s);
            let t = s.cross_p64(normal);

            let mut start0 = NIL;
            let first = self.vert(c0).edges;
            if first != NIL {
                let mut e = first;
                loop {
                    let d = self.vert(self.edge(e).target).point - self.vert(c0).point;
                    // Upstream nests these; `&&` is the same short-circuit, and
                    // `get_orientation` is still only reached once the edge is
                    // known to lie in the bridging plane on the `t` side.
                    if (d.dot_p64(normal) == 0)
                        && (d.dot_p64(t) > 0)
                        && ((start0 == NIL)
                            || (self.get_orientation(start0, e, s, down) == Orientation::Clockwise))
                    {
                        start0 = e;
                    }
                    e = self.edge(e).next;
                    if e == first {
                        break;
                    }
                }
            }

            let mut start1 = NIL;
            let first = self.vert(c1).edges;
            if first != NIL {
                let mut e = first;
                loop {
                    let d = self.vert(self.edge(e).target).point - self.vert(c1).point;
                    if (d.dot_p64(normal) == 0)
                        && (d.dot_p64(t) > 0)
                        && ((start1 == NIL)
                            || (self.get_orientation(start1, e, s, down)
                                == Orientation::CounterClockwise))
                    {
                        start1 = e;
                    }
                    e = self.edge(e).next;
                    if e == first {
                        break;
                    }
                }
            }

            if (start0 != NIL) || (start1 != NIL) {
                self.find_edge_for_coplanar_faces(c0, c1, &mut start0, &mut start1, NIL, NIL);
                if start0 != NIL {
                    c0 = self.edge(start0).target;
                }
                if start1 != NIL {
                    c1 = self.edge(start1).target;
                }
            }

            prev_point = self.vert(c1).point;
            prev_point.z += 1;
        } else {
            prev_point = self.vert(c1).point;
            prev_point.x += 1;
        }

        let first0 = c0;
        let first1 = c1;
        let mut first_run = true;

        loop {
            let s = self.vert(c1).point - self.vert(c0).point;
            let r = prev_point - self.vert(c0).point;
            let rxs = r.cross(s);
            let sxrxs = s.cross_p64(rxs);

            let mut min_cot0 = Rational64::new(0, 0);
            let min0 = self.find_max_angle(false, c0, s, rxs, sxrxs, &mut min_cot0);
            let mut min_cot1 = Rational64::new(0, 0);
            let min1 = self.find_max_angle(true, c1, s, rxs, sxrxs, &mut min_cot1);
            if (min0 == NIL) && (min1 == NIL) {
                // Both hulls are single vertices (or degenerate to one edge):
                // the merged hull is the single bridge edge.
                let e = self.new_edge_pair(c0, c1);
                self.link(e, e);
                self.vert_mut(c0).edges = e;

                let e = self.edge(e).reverse;
                self.link(e, e);
                self.vert_mut(c1).edges = e;
                return;
            }

            let cmp = if min0 == NIL {
                1
            } else if min1 == NIL {
                -1
            } else {
                min_cot0.compare(min_cot1)
            };

            if first_run
                || (if cmp >= 0 {
                    !min_cot1.is_negative_infinity()
                } else {
                    !min_cot0.is_negative_infinity()
                })
            {
                let e = self.new_edge_pair(c0, c1);
                if pending_tail0 != NIL {
                    self.edge_mut(pending_tail0).prev = e;
                } else {
                    pending_head0 = e;
                }
                self.edge_mut(e).next = pending_tail0;
                pending_tail0 = e;

                let e = self.edge(e).reverse;
                if pending_tail1 != NIL {
                    self.edge_mut(pending_tail1).next = e;
                } else {
                    pending_head1 = e;
                }
                self.edge_mut(e).prev = pending_tail1;
                pending_tail1 = e;
            }

            let mut e0 = min0;
            let mut e1 = min1;

            if cmp == 0 {
                self.find_edge_for_coplanar_faces(c0, c1, &mut e0, &mut e1, NIL, NIL);
            }

            if (cmp >= 0) && (e1 != NIL) {
                if to_prev1 != NIL {
                    let mut e = self.edge(to_prev1).next;
                    while e != min1 {
                        let n = self.edge(e).next;
                        self.remove_edge_pair(e);
                        e = n;
                    }
                }

                if pending_tail1 != NIL {
                    if to_prev1 != NIL {
                        self.link(to_prev1, pending_head1);
                    } else {
                        let p = self.edge(min1).prev;
                        self.link(p, pending_head1);
                        first_new1 = pending_head1;
                    }
                    self.link(pending_tail1, min1);
                    pending_head1 = NIL;
                    pending_tail1 = NIL;
                } else if to_prev1 == NIL {
                    first_new1 = min1;
                }

                prev_point = self.vert(c1).point;
                c1 = self.edge(e1).target;
                to_prev1 = self.edge(e1).reverse;
            }

            if (cmp <= 0) && (e0 != NIL) {
                if to_prev0 != NIL {
                    let mut e = self.edge(to_prev0).prev;
                    while e != min0 {
                        let n = self.edge(e).prev;
                        self.remove_edge_pair(e);
                        e = n;
                    }
                }

                if pending_tail0 != NIL {
                    if to_prev0 != NIL {
                        self.link(pending_head0, to_prev0);
                    } else {
                        let n = self.edge(min0).next;
                        self.link(pending_head0, n);
                        first_new0 = pending_head0;
                    }
                    self.link(min0, pending_tail0);
                    pending_head0 = NIL;
                    pending_tail0 = NIL;
                } else if to_prev0 == NIL {
                    first_new0 = min0;
                }

                prev_point = self.vert(c0).point;
                c0 = self.edge(e0).target;
                to_prev0 = self.edge(e0).reverse;
            }

            if (c0 == first0) && (c1 == first1) {
                if to_prev0 == NIL {
                    self.link(pending_head0, pending_tail0);
                    self.vert_mut(c0).edges = pending_tail0;
                } else {
                    let mut e = self.edge(to_prev0).prev;
                    while e != first_new0 {
                        let n = self.edge(e).prev;
                        self.remove_edge_pair(e);
                        e = n;
                    }
                    if pending_tail0 != NIL {
                        self.link(pending_head0, to_prev0);
                        self.link(first_new0, pending_tail0);
                    }
                }

                if to_prev1 == NIL {
                    self.link(pending_tail1, pending_head1);
                    self.vert_mut(c1).edges = pending_tail1;
                } else {
                    let mut e = self.edge(to_prev1).next;
                    while e != first_new1 {
                        let n = self.edge(e).next;
                        self.remove_edge_pair(e);
                        e = n;
                    }
                    if pending_tail1 != NIL {
                        self.link(to_prev1, pending_head1);
                        self.link(pending_tail1, first_new1);
                    }
                }

                return;
            }

            first_run = false;
        }
    }

    /// `btConvexHullInternal::compute(coords, doubleCoords, stride, count)`
    /// (`:1956-2075`) -- quantize, sort, and run the recursion.
    ///
    /// The quantization is the algorithm's foundation: the AABB's longest
    /// extent becomes the grid's `y`, the shortest its `z`, and each axis is
    /// divided into `10216` steps. The axis permutation is chosen so that the
    /// divide-and-conquer splits along the longest extent, and `scaling` is
    /// negated when the permutation `(med, max, min)` is odd so that the grid
    /// stays right-handed -- without that a hull would come out with its faces
    /// wound backwards.
    fn compute(&mut self, coords: &[Vec3]) {
        let count = coords.len();
        let mut min = Vec3::new(1e30, 1e30, 1e30);
        let mut max = Vec3::new(-1e30, -1e30, -1e30);
        for &v in coords {
            set_min(&mut min, v);
            set_max(&mut max, v);
        }

        let mut s = max - min;
        self.max_axis = s.max_axis();
        self.min_axis = s.min_axis();
        if self.min_axis == self.max_axis {
            self.min_axis = (self.max_axis + 1) % 3;
        }
        self.med_axis = 3 - self.max_axis - self.min_axis;

        s /= 10216.0;
        if ((self.med_axis + 1) % 3) != self.max_axis {
            // `s *= -1`. Multiplying an `f32` by `-1.0` and negating it are the
            // same bit operation on every input, zeros and NaNs included.
            s = -s;
        }
        self.scaling = s;

        if s[0] != 0.0 {
            s[0] = 1.0 / s[0];
        }
        if s[1] != 0.0 {
            s[1] = 1.0 / s[1];
        }
        if s[2] != 0.0 {
            s[2] = 1.0 / s[2];
        }

        self.center = (min + max) * 0.5;

        let mut points: Vec<Point32> = Vec::with_capacity(count);
        for (i, &v) in coords.iter().enumerate() {
            let p = (v - self.center) * s;
            // `(int32_t)` -- truncation toward zero, and out of range it is
            // undefined in C++ where Rust saturates. Unreachable either way:
            // the scaling above puts every component in `[-5108, 5108]`, and a
            // zero-extent axis leaves its reciprocal at zero, so the product is
            // zero rather than infinite.
            points.push(Point32 {
                x: p[self.med_axis] as i32,
                y: p[self.max_axis] as i32,
                z: p[self.min_axis] as i32,
                index: i as i32,
            });
        }
        quick_sort(&mut points);

        self.edge_array_size = 6 * count;
        self.original_vertices.reserve(count);
        for &point in &points {
            let v = self.new_vertex_object();
            let vertex = self.vert_mut(v);
            vertex.edges = NIL;
            vertex.point = point;
            vertex.copy = -1;
            self.original_vertices.push(v);
        }

        self.merge_stamp = -3;

        let mut hull = IntermediateHull::EMPTY;
        self.compute_internal(0, count, &mut hull);
        self.vertex_list = hull.min_xy;
    }

    /// `toBtVector(const Point32& v)` (`:2077-2084`) -- a grid point back to
    /// the input frame's *directions*, without `center`; used for face normals
    /// and origins, where the translation would be wrong.
    fn to_bt_vector(&self, v: Point32) -> Vec3 {
        let mut p = Vec3::zero();
        p[self.med_axis] = v.x as Scalar;
        p[self.max_axis] = v.y as Scalar;
        p[self.min_axis] = v.z as Scalar;
        p * self.scaling
    }

    /// `getBtNormal(Face* face)` (`:2086-2089`).
    fn get_bt_normal(&self, face: FId) -> Vec3 {
        self.to_bt_vector(self.face(face).dir0)
            .cross(self.to_bt_vector(self.face(face).dir1))
            .normalize()
    }

    /// `getCoordinates(const Vertex* v)` (`:2091-2098`) -- the output
    /// coordinate of a hull vertex.
    ///
    /// This is where the integers become floats again, and it is not the
    /// identity on the input: an input point is quantized to the grid and then
    /// mapped back, so a hull vertex is the *grid* point, not the mesh vertex
    /// it came from. [`ConvexHullComputer::original_vertex_index`] is what
    /// recovers the original.
    fn get_coordinates(&self, v: VId) -> Vec3 {
        let vertex = self.vert(v);
        let mut p = Vec3::zero();
        p[self.med_axis] = vertex.xvalue();
        p[self.max_axis] = vertex.yvalue();
        p[self.min_axis] = vertex.zvalue();
        p * self.scaling + self.center
    }

    /// `shrink(amount, clampAmount)` (`:2100-2219`) -- move every face inward
    /// along its normal by `amount`, returning how far it actually moved, or a
    /// negative value if the hull collapsed.
    ///
    /// Unreachable from MoveIt (see the module docs). The traversal at the top
    /// does double duty: it enumerates the faces -- which nothing before this
    /// point built, because the hull is stored purely as vertices and edges --
    /// and it accumulates the exact integer volume and centroid of the
    /// tetrahedral fan from `ref`, which `clampAmount` needs to know how far in
    /// the nearest face is.
    fn shrink(&mut self, amount: Scalar, clamp_amount: Scalar) -> Scalar {
        if self.vertex_list == NIL {
            return 0.0;
        }
        let mut amount = amount;
        self.merge_stamp -= 1;
        let stamp = self.merge_stamp;
        let mut stack: Vec<VId> = Vec::new();
        self.vert_mut(self.vertex_list).copy = stamp;
        stack.push(self.vertex_list);
        let mut faces: Vec<FId> = Vec::new();

        let reference = self.vert(self.vertex_list).point;
        let mut hull_center_x = Int128::from_u64(0);
        let mut hull_center_y = Int128::from_u64(0);
        let mut hull_center_z = Int128::from_u64(0);
        let mut volume = Int128::from_u64(0);

        while let Some(v) = stack.pop() {
            let first = self.vert(v).edges;
            if first != NIL {
                let mut e = first;
                loop {
                    let target = self.edge(e).target;
                    if self.vert(target).copy != stamp {
                        self.vert_mut(target).copy = stamp;
                        stack.push(target);
                    }
                    if self.edge(e).copy != stamp {
                        let face = self.new_face_object();
                        let across = self.edge(self.edge(self.edge(e).reverse).prev).target;
                        self.face_init(face, target, across, v);
                        faces.push(face);
                        let mut f = e;

                        let mut a = NIL;
                        let mut b = NIL;
                        loop {
                            if (a != NIL) && (b != NIL) {
                                let vol = (self.vert(v).point - reference).dot_p64(
                                    (self.vert(a).point - reference)
                                        .cross(self.vert(b).point - reference),
                                );
                                let c = self.vert(v).point
                                    + self.vert(a).point
                                    + self.vert(b).point
                                    + reference;
                                hull_center_x += Int128::from_i64(vol * i64::from(c.x));
                                hull_center_y += Int128::from_i64(vol * i64::from(c.y));
                                hull_center_z += Int128::from_i64(vol * i64::from(c.z));
                                volume += Int128::from_i64(vol);
                            }

                            self.edge_mut(f).copy = stamp;
                            self.edge_mut(f).face = face;

                            a = b;
                            b = self.edge(f).target;

                            f = self.edge(self.edge(f).reverse).prev;
                            if f == e {
                                break;
                            }
                        }
                    }
                    e = self.edge(e).next;
                    if e == first {
                        break;
                    }
                }
            }
        }

        if volume.get_sign() <= 0 {
            return 0.0;
        }

        let mut hull_center = Vec3::zero();
        hull_center[self.med_axis] = hull_center_x.to_scalar();
        hull_center[self.max_axis] = hull_center_y.to_scalar();
        hull_center[self.min_axis] = hull_center_z.to_scalar();
        hull_center /= 4.0 * volume.to_scalar();
        hull_center = hull_center * self.scaling;

        let face_count = faces.len();

        if clamp_amount > 0.0 {
            let mut min_dist = SIMD_INFINITY;
            for &f in &faces {
                let normal = self.get_bt_normal(f);
                let dist = normal.dot(self.to_bt_vector(self.face(f).origin) - hull_center);
                if dist < min_dist {
                    min_dist = dist;
                }
            }

            if min_dist <= 0.0 {
                return 0.0;
            }

            // `btMin`, which is `a < b ? a : b` -- written out because Rust's
            // `f32::min` disagrees with it on a NaN operand.
            let clamped = min_dist * clamp_amount;
            amount = if amount < clamped { amount } else { clamped };
        }

        // A fixed-seed LCG shuffle, not randomness: `shiftFace` costs more when
        // consecutive faces are adjacent, and the order is pinned so that the
        // result is reproducible. `unsigned int` wraps, hence `wrapping_*`.
        let mut seed: u32 = 243_703;
        for i in 0..face_count {
            faces.swap(i, (seed as usize) % face_count);
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        }

        for &f in &faces {
            // `stack` is passed *by value* upstream (`:2221`), so `shiftFace`
            // gets a copy and the caller's vector is untouched. It is empty
            // here -- the traversal above drained it -- but the copy is what
            // upstream does.
            if !self.shift_face(f, amount, stack.clone()) {
                return -amount;
            }
        }

        amount
    }

    /// `shiftFace(face, amount, stack)` (`:2221-2636`) -- move one face inward
    /// and repair the hull around it.
    ///
    /// Returns `false` when the shift would push the face past the opposite
    /// side of the hull, which is how [`HullInternal::shrink`] detects that the
    /// requested amount empties the hull.
    ///
    /// The shape of it: find one edge that crosses the shifted plane, walk the
    /// crossing edges around the face creating a new polygon, splice that
    /// polygon in, and then delete everything that ended up outside. Vertices
    /// created at a crossing are not on the integer grid, so they carry an
    /// exact [`PointR128`] beside a rounded `Point32` -- and every subsequent
    /// plane test on them goes through [`Rational128`] rather than the rounded
    /// copy, which is why that class exists.
    fn shift_face(&mut self, face: FId, amount: Scalar, mut stack: Vec<VId>) -> bool {
        let mut orig_shift = self.get_bt_normal(face) * -amount;
        // A true `btScalar` division per component, not `btVector3::operator/=`
        // and so not a reciprocal-multiply.
        if self.scaling[0] != 0.0 {
            orig_shift[0] /= self.scaling[0];
        }
        if self.scaling[1] != 0.0 {
            orig_shift[1] /= self.scaling[1];
        }
        if self.scaling[2] != 0.0 {
            orig_shift[2] /= self.scaling[2];
        }
        let shift = Point32::new(
            orig_shift[self.med_axis] as i32,
            orig_shift[self.max_axis] as i32,
            orig_shift[self.min_axis] as i32,
        );
        if shift.is_zero() {
            // Less than one grid unit: nothing to do, and reporting success is
            // what keeps a sub-grid `shrink` from failing the whole hull.
            return true;
        }
        let normal = self.face(face).get_normal();
        let orig_dot = self.face(face).origin.dot_p64(normal);
        let shifted_origin = self.face(face).origin + shift;
        let shifted_dot = shifted_origin.dot_p64(normal);
        if shifted_dot >= orig_dot {
            return false;
        }

        let mut intersection = NIL;

        let mut start_edge = self.vert(self.face(face).nearby_vertex).edges;
        let mut opt_dot = self.vert(self.face(face).nearby_vertex).dot(normal);
        let mut cmp = opt_dot.compare_i64(shifted_dot);
        if cmp >= 0 {
            let mut e = start_edge;
            loop {
                let dot = self.vert(self.edge(e).target).dot(normal);
                if dot.compare(opt_dot) < 0 {
                    let c = dot.compare_i64(shifted_dot);
                    opt_dot = dot;
                    e = self.edge(e).reverse;
                    start_edge = e;
                    if c < 0 {
                        intersection = e;
                        break;
                    }
                    cmp = c;
                }
                e = self.edge(e).prev;
                if e == start_edge {
                    break;
                }
            }

            if intersection == NIL {
                return false;
            }
        } else {
            let mut e = start_edge;
            loop {
                let dot = self.vert(self.edge(e).target).dot(normal);
                if dot.compare(opt_dot) > 0 {
                    cmp = dot.compare_i64(shifted_dot);
                    if cmp >= 0 {
                        intersection = e;
                        break;
                    }
                    opt_dot = dot;
                    e = self.edge(e).reverse;
                    start_edge = e;
                }
                e = self.edge(e).prev;
                if e == start_edge {
                    break;
                }
            }

            if intersection == NIL {
                // The whole hull is already on the inner side of the shifted
                // plane, so the shift changes nothing.
                return true;
            }
        }

        if cmp == 0 {
            let mut e = self.edge(self.edge(intersection).reverse).next;
            while self
                .vert(self.edge(e).target)
                .dot(normal)
                .compare_i64(shifted_dot)
                <= 0
            {
                e = self.edge(e).next;
                if e == self.edge(intersection).reverse {
                    return true;
                }
            }
        }

        let mut first_intersection = NIL;
        let mut face_edge = NIL;
        let mut first_face_edge = NIL;

        loop {
            if cmp == 0 {
                let mut e = self.edge(self.edge(intersection).reverse).next;
                start_edge = e;
                loop {
                    if self
                        .vert(self.edge(e).target)
                        .dot(normal)
                        .compare_i64(shifted_dot)
                        >= 0
                    {
                        break;
                    }
                    intersection = self.edge(e).reverse;
                    e = self.edge(e).next;
                    if e == start_edge {
                        return true;
                    }
                }
            }

            if first_intersection == NIL {
                first_intersection = intersection;
            } else if intersection == first_intersection {
                break;
            }

            let prev_cmp = cmp;
            let prev_intersection = intersection;
            let prev_face_edge = face_edge;

            let mut e = self.edge(intersection).reverse;
            loop {
                e = self.edge(self.edge(e).reverse).prev;
                cmp = self
                    .vert(self.edge(e).target)
                    .dot(normal)
                    .compare_i64(shifted_dot);
                if cmp >= 0 {
                    intersection = e;
                    break;
                }
            }

            if cmp > 0 {
                let removed = self.edge(intersection).target;
                e = self.edge(intersection).reverse;
                if self.edge(e).prev == e {
                    self.vert_mut(removed).edges = NIL;
                } else {
                    let p = self.edge(e).prev;
                    let n = self.edge(e).next;
                    self.vert_mut(removed).edges = p;
                    self.link(p, n);
                    self.link(e, e);
                }

                // The new vertex is where the shifted plane meets the two faces
                // the cut edge separates: a 2x2 solve, done exactly in 128-bit
                // integers with `det` kept as the denominator rather than
                // dividing.
                let n0 = self.face(self.edge(intersection).face).get_normal();
                let n1 = self
                    .face(self.edge(self.edge(intersection).reverse).face)
                    .get_normal();
                let dir0 = self.face(face).dir0;
                let dir1 = self.face(face).dir1;
                let m00 = dir0.dot_p64(n0);
                let m01 = dir1.dot_p64(n0);
                let m10 = dir0.dot_p64(n1);
                let m11 = dir1.dot_p64(n1);
                let r0 =
                    (self.face(self.edge(intersection).face).origin - shifted_origin).dot_p64(n0);
                let r1 = (self
                    .face(self.edge(self.edge(intersection).reverse).face)
                    .origin
                    - shifted_origin)
                    .dot_p64(n1);
                let det = Int128::mul_i64(m00, m11) - Int128::mul_i64(m01, m10);
                let v = self.new_vertex_object();
                {
                    let vertex = self.vert_mut(v);
                    vertex.point.index = -1;
                    vertex.copy = -1;
                    vertex.point128 = PointR128 {
                        x: Int128::mul_i64(i64::from(dir0.x) * r0, m11)
                            - Int128::mul_i64(i64::from(dir0.x) * r1, m01)
                            + Int128::mul_i64(i64::from(dir1.x) * r1, m00)
                            - Int128::mul_i64(i64::from(dir1.x) * r0, m10)
                            + det.mul_by_i64(i64::from(shifted_origin.x)),
                        y: Int128::mul_i64(i64::from(dir0.y) * r0, m11)
                            - Int128::mul_i64(i64::from(dir0.y) * r1, m01)
                            + Int128::mul_i64(i64::from(dir1.y) * r1, m00)
                            - Int128::mul_i64(i64::from(dir1.y) * r0, m10)
                            + det.mul_by_i64(i64::from(shifted_origin.y)),
                        z: Int128::mul_i64(i64::from(dir0.z) * r0, m11)
                            - Int128::mul_i64(i64::from(dir0.z) * r1, m01)
                            + Int128::mul_i64(i64::from(dir1.z) * r1, m00)
                            - Int128::mul_i64(i64::from(dir1.z) * r0, m10)
                            + det.mul_by_i64(i64::from(shifted_origin.z)),
                        denominator: det,
                    };
                    vertex.point.x = vertex.point128.xvalue() as i32;
                    vertex.point.y = vertex.point128.yvalue() as i32;
                    vertex.point.z = vertex.point128.zvalue() as i32;
                }
                self.edge_mut(intersection).target = v;
                self.vert_mut(v).edges = e;

                stack.push(v);
                stack.push(removed);
                stack.push(NIL);
            }

            if (cmp != 0)
                || (prev_cmp != 0)
                || (self
                    .edge(self.edge(self.edge(prev_intersection).reverse).next)
                    .target
                    != self.edge(intersection).target)
            {
                let from = self.edge(prev_intersection).target;
                let to = self.edge(intersection).target;
                face_edge = self.new_edge_pair(from, to);
                if prev_cmp == 0 {
                    let n = self.edge(self.edge(prev_intersection).reverse).next;
                    self.link(face_edge, n);
                }
                if (prev_cmp == 0) || (prev_face_edge != NIL) {
                    let r = self.edge(prev_intersection).reverse;
                    self.link(r, face_edge);
                }
                if cmp == 0 {
                    let p = self.edge(self.edge(intersection).reverse).prev;
                    let fr = self.edge(face_edge).reverse;
                    self.link(p, fr);
                }
                let fr = self.edge(face_edge).reverse;
                let r = self.edge(intersection).reverse;
                self.link(fr, r);
            } else {
                face_edge = self.edge(self.edge(prev_intersection).reverse).next;
            }

            if prev_face_edge != NIL {
                if prev_cmp > 0 {
                    let pr = self.edge(prev_face_edge).reverse;
                    self.link(face_edge, pr);
                } else if face_edge != self.edge(prev_face_edge).reverse {
                    stack.push(self.edge(prev_face_edge).target);
                    while self.edge(face_edge).next != self.edge(prev_face_edge).reverse {
                        let n = self.edge(face_edge).next;
                        let removed = self.edge(n).target;
                        self.remove_edge_pair(n);
                        stack.push(removed);
                    }
                    stack.push(NIL);
                }
            }
            self.edge_mut(face_edge).face = face;
            let fr = self.edge(face_edge).reverse;
            let iface = self.edge(intersection).face;
            self.edge_mut(fr).face = iface;

            if first_face_edge == NIL {
                first_face_edge = face_edge;
            }
        }

        if cmp > 0 {
            let t = self.edge(face_edge).target;
            let fr = self.edge(first_face_edge).reverse;
            self.edge_mut(fr).target = t;
            let r = self.edge(first_intersection).reverse;
            self.link(r, first_face_edge);
            let fer = self.edge(face_edge).reverse;
            self.link(first_face_edge, fer);
        } else if first_face_edge != self.edge(face_edge).reverse {
            stack.push(self.edge(face_edge).target);
            while self.edge(first_face_edge).next != self.edge(face_edge).reverse {
                let n = self.edge(first_face_edge).next;
                let removed = self.edge(n).target;
                self.remove_edge_pair(n);
                stack.push(removed);
            }
            stack.push(NIL);
        }

        self.vertex_list = stack[0];

        // Breadth-first deletion of everything the shift cut off. The stack is
        // read as `kept, removed..., NULL` groups, and detaching a removed
        // vertex can expose more, which are appended as a further group -- so
        // the outer loop re-reads `stack.len()` each round rather than
        // iterating a snapshot.
        let mut pos = 0;
        while pos < stack.len() {
            let end = stack.len();
            while pos < end {
                let kept = stack[pos];
                pos += 1;
                let mut deeper = false;
                loop {
                    let removed = stack[pos];
                    pos += 1;
                    if removed == NIL {
                        break;
                    }
                    self.receive_nearby_faces(kept, removed);
                    while self.vert(removed).edges != NIL {
                        if !deeper {
                            deeper = true;
                            stack.push(kept);
                        }
                        let e = self.vert(removed).edges;
                        stack.push(self.edge(e).target);
                        self.remove_edge_pair(e);
                    }
                }
                if deeper {
                    stack.push(NIL);
                }
            }
        }

        self.face_mut(face).origin = shifted_origin;

        true
    }
}

// ---------------------------------------------------------------------------
// btConvexHullComputer
// ---------------------------------------------------------------------------

/// `btConvexHullComputer::Edge` (`btConvexHullComputer.h:30-64`) -- one output
/// half-edge.
///
/// The two link fields are *relative* offsets into
/// [`ConvexHullComputer::edges`], not indices: upstream navigates with
/// `this + next` and `this + reverse` so that the whole array can be copied
/// without fixing up references. The offsets are preserved rather than
/// converted to absolute indices, because they are what the array literally
/// contains and a port that stored indices would silently accept a fixture
/// whose offsets were wrong.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Edge {
    /// Offset to the next edge around the *source vertex*, clockwise.
    next: i32,
    /// Offset to this edge's opposite half.
    reverse: i32,
    /// Index into [`ConvexHullComputer::vertices`].
    target_vertex: i32,
}

impl Edge {
    /// `getTargetVertex()` (`btConvexHullComputer.h:45-48`).
    #[must_use]
    pub fn target_vertex(&self) -> i32 {
        self.target_vertex
    }
}

/// `btConvexHullComputer` (`btConvexHullComputer.h:24-100`) -- the convex hull
/// of a point cloud.
///
/// See the module docs for what the class is and why its output order matters.
///
/// ```
/// use cspace_bullet::convex_hull_computer::ConvexHullComputer;
/// use cspace_bullet::linear_math::Vec3;
///
/// // A cube with a point buried inside it.
/// let mut points: Vec<Vec3> = Vec::new();
/// for x in [-1.0, 1.0] {
///     for y in [-1.0, 1.0] {
///         for z in [-1.0, 1.0] {
///             points.push(Vec3::new(x, y, z));
///         }
///     }
/// }
/// points.push(Vec3::new(0.25, -0.5, 0.125));
///
/// let mut hull = ConvexHullComputer::new();
/// assert_eq!(hull.compute(&points, 0.0, 0.0), 0.0);
/// assert_eq!(hull.vertices.len(), 8);
/// assert_eq!(hull.faces.len(), 6);
/// ```
#[derive(Clone, Debug, Default)]
pub struct ConvexHullComputer {
    /// `vertices` -- the hull's vertices, in the order the output walk found
    /// them. Not a set: `btConvexHullShape::localGetSupportingVertexWithoutMargin`
    /// breaks ties toward the lowest index, so this order is observable
    /// downstream.
    pub vertices: Vec<Vec3>,
    /// `original_vertex_index` -- for each entry of [`Self::vertices`], the
    /// index it had in `compute`'s input, or `-1` for a vertex `shrink`
    /// synthesised.
    pub original_vertex_index: Vec<i32>,
    /// `edges` -- both halves of every hull edge, adjacent in pairs.
    pub edges: Vec<Edge>,
    /// `faces` -- one edge index per face; the face is the loop reached from it
    /// through [`Self::next_edge_of_face`]. Faces are planar n-gons, so the
    /// loop length varies.
    pub faces: Vec<i32>,
}

/// `getVertexCopy(vertex, vertices)` (`:2638-2651`) -- the output index of a
/// hull vertex, assigning one on first sight.
///
/// `Vertex::copy` is `-1` for every vertex the construction produced, so `< 0`
/// means "not yet emitted"; the same field then holds the emitted index. The
/// order vertices are discovered in is therefore the output order, and it is
/// decided by `vertexList` plus each vertex's edge ring.
fn get_vertex_copy(hull: &mut HullInternal, vertex: VId, vertices: &mut Vec<VId>) -> i32 {
    let mut index = hull.vert(vertex).copy;
    if index < 0 {
        index = vertices.len() as i32;
        hull.vert_mut(vertex).copy = index;
        vertices.push(vertex);
    }
    index
}

impl ConvexHullComputer {
    /// An empty computer. [`Self::compute`] fills it.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// `getSourceVertex()` (`btConvexHullComputer.h:40-43`) -- the vertex
    /// `edge` leaves, which is the *reverse* edge's target.
    ///
    /// # Panics
    ///
    /// If `edge` is out of range, or the stored offsets do not land inside
    /// [`Self::edges`]. Neither is reachable from a hull this type built.
    #[must_use]
    pub fn edge_source_vertex(&self, edge: usize) -> i32 {
        self.edges[self.reverse_edge(edge)].target_vertex
    }

    /// `getNextEdgeOfVertex()` (`btConvexHullComputer.h:50-53`) -- clockwise
    /// around the source vertex.
    ///
    /// # Panics
    ///
    /// If `edge` is out of range, or the offset leaves [`Self::edges`].
    #[must_use]
    pub fn next_edge_of_vertex(&self, edge: usize) -> usize {
        (edge as i32 + self.edges[edge].next) as usize
    }

    /// `getNextEdgeOfFace()` (`btConvexHullComputer.h:55-58`) --
    /// counter-clockwise around the face, which upstream builds as "reverse,
    /// then next around *that* vertex". Walking it from `faces[i]` until it
    /// returns to the start enumerates one face's vertices, which is exactly
    /// what MoveIt's `createConvexHull` does.
    ///
    /// # Panics
    ///
    /// If `edge` is out of range, or either offset leaves [`Self::edges`].
    #[must_use]
    pub fn next_edge_of_face(&self, edge: usize) -> usize {
        self.next_edge_of_vertex(self.reverse_edge(edge))
    }

    /// `getReverseEdge()` (`btConvexHullComputer.h:60-63`).
    ///
    /// # Panics
    ///
    /// If `edge` is out of range, or the offset leaves [`Self::edges`].
    #[must_use]
    pub fn reverse_edge(&self, edge: usize) -> usize {
        (edge as i32 + self.edges[edge].reverse) as usize
    }

    /// `btConvexHullComputer::compute(coords, stride, count, shrink, shrinkClamp)`
    /// (`btConvexHullComputer.h:90-93`), the public wrapper, which forwards to
    /// the private `compute(const void*, bool, ...)` this ports
    /// (`:2653-2760`, declared `btConvexHullComputer.h:27`).
    ///
    /// Returns the distance the hull was actually shrunk by, which is `0` for
    /// the `shrink <= 0` case MoveIt uses, and negative when `shrink` was so
    /// large that the hull came out empty -- MoveIt's `createConvexHull` treats
    /// that as a hard failure (`contact_checker_common.cpp:138-142`).
    ///
    /// `stride` is absent; see the module docs. `count <= 0` is
    /// `coords.is_empty()`, and note that upstream's early return clears
    /// `vertices`, `edges` and `faces` but *not* `original_vertex_index`, so an
    /// empty call after a non-empty one leaves that array stale. Reproduced,
    /// because a caller reading it by index would see the same thing.
    pub fn compute(&mut self, coords: &[Vec3], shrink: Scalar, shrink_clamp: Scalar) -> Scalar {
        if coords.is_empty() {
            self.vertices.clear();
            self.edges.clear();
            self.faces.clear();
            return 0.0;
        }

        let mut hull = HullInternal::new();
        hull.compute(coords);

        let mut shift = 0.0;
        if shrink > 0.0 {
            shift = hull.shrink(shrink, shrink_clamp);
            if shift < 0.0 {
                self.vertices.clear();
                self.edges.clear();
                self.faces.clear();
                return shift;
            }
        }

        self.vertices.clear();
        self.original_vertex_index.clear();
        self.edges.clear();
        self.faces.clear();

        let mut old_vertices: Vec<VId> = Vec::new();
        let vertex_list = hull.vertex_list;
        get_vertex_copy(&mut hull, vertex_list, &mut old_vertices);
        let mut copied = 0usize;
        while copied < old_vertices.len() {
            let v = old_vertices[copied];
            self.vertices.push(hull.get_coordinates(v));
            self.original_vertex_index.push(hull.vert(v).point.index);
            let first_edge = hull.vert(v).edges;
            if first_edge != NIL {
                let mut first_copy: i32 = -1;
                let mut prev_copy: i32 = -1;
                let mut e = first_edge;
                loop {
                    if hull.edge(e).copy < 0 {
                        let s = self.edges.len() as i32;
                        self.edges.push(Edge::default());
                        self.edges.push(Edge::default());
                        hull.edge_mut(e).copy = s;
                        let r = hull.edge(e).reverse;
                        hull.edge_mut(r).copy = s + 1;
                        self.edges[s as usize].reverse = 1;
                        self.edges[s as usize + 1].reverse = -1;
                        let target = hull.edge(e).target;
                        self.edges[s as usize].target_vertex =
                            get_vertex_copy(&mut hull, target, &mut old_vertices);
                        self.edges[s as usize + 1].target_vertex = copied as i32;
                    }
                    if prev_copy >= 0 {
                        let ec = hull.edge(e).copy;
                        self.edges[ec as usize].next = prev_copy - ec;
                    } else {
                        first_copy = hull.edge(e).copy;
                    }
                    prev_copy = hull.edge(e).copy;
                    e = hull.edge(e).next;
                    if e == first_edge {
                        break;
                    }
                }
                self.edges[first_copy as usize].next = prev_copy - first_copy;
            }
            copied += 1;
        }

        // Second pass for the faces. It reuses `Edge::copy` as a visited mark,
        // walking each face loop and setting every edge on it back to `-1`, so
        // a face is emitted from whichever of its edges is reached first and
        // never again.
        // `copied` equals `old_vertices.len()` by the time the loop above
        // exits; `take` keeps upstream's `i < copied` bound literal.
        for &v in old_vertices.iter().take(copied) {
            let first_edge = hull.vert(v).edges;
            if first_edge != NIL {
                let mut e = first_edge;
                loop {
                    if hull.edge(e).copy >= 0 {
                        self.faces.push(hull.edge(e).copy);
                        let mut f = e;
                        loop {
                            hull.edge_mut(f).copy = -1;
                            f = hull.edge(hull.edge(f).reverse).prev;
                            if f == e {
                                break;
                            }
                        }
                    }
                    e = hull.edge(e).next;
                    if e == first_edge {
                        break;
                    }
                }
            }
        }

        shift
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::probe_fixture::{diff, diff_vec3, row};

    /// The `chc_*` rows of `tools/bullet-epa-reference/build.sh`'s stdout --
    /// `btConvexHullComputer::compute` on seventeen point sets, one row per
    /// input point, output vertex, output half-edge and face.
    ///
    /// Pasted whole rather than edited row by row: a row transcribed field by
    /// field picks up the transcriber's idea of the field order, and the parser
    /// below checks only that each line has the arity its prefix implies.
    ///
    /// The input points are in here too, and the test reads them back out
    /// rather than spelling the point sets a second time in Rust. A point set
    /// written in both languages is a premise that can drift, and it would
    /// drift as a coordinate mismatch blamed on the hull algorithm rather than
    /// on the setup.
    ///
    /// What each case is for:
    ///
    /// - `cube8` -- the eight corners of a cube, nothing to discard.
    /// - `interior` -- the same plus five points strictly inside, which must
    ///   not appear in the output.
    /// - `dup` -- four square corners twice each and an apex three times.
    ///   Which copy survives is what the `origIndex` column pins.
    /// - `flat` -- nine coplanar points, so the shortest AABB extent is zero
    ///   and the grid's z axis collapses. The result is a two-faced degenerate
    ///   polyhedron, which is what upstream produces and not an error.
    /// - `collinear` -- cube corners plus six points strictly *on* hull edges.
    ///   They are on the boundary but are not vertices, and only the exact
    ///   integer orientation predicates separate the two -- any float predicate
    ///   with a tolerance keeps some of them.
    /// - `vpairs` -- points in vertical pairs sharing both projected
    ///   coordinates, arranged so the recursion's leaves are exactly those
    ///   pairs. That is `compute_internal`'s `dx == 0 && dy == 0` case.
    /// - `cloud` -- thirty-two pseudo-random points, most of them interior.
    /// - `shell` -- twenty-six points on the unit sphere, so every one is a
    ///   hull vertex: the shape of a real mesh's vertex list, where nothing is
    ///   discarded and every merge decides an edge.
    /// - `one`, `two`, `three`, `four`, `same` -- `compute_internal`'s base
    ///   cases, including five identical points collapsing to one vertex.
    /// - `shrunk`, `clamped`, `collapse`, `shrunk_shell` -- the `shrink > 0`
    ///   path MoveIt never takes, including the `shrinkClamp` branch that needs
    ///   the hull's exact integer volume and the collapse that returns a
    ///   negative shift and an empty hull.
    const BULLET_REFERENCE: &str = "\
chc_cube8|8|0|0|0|8|24|6
chcin_cube8_0|-1|-1|-1
chcin_cube8_1|-1|-1|1
chcin_cube8_2|-1|1|-1
chcin_cube8_3|-1|1|1
chcin_cube8_4|1|-1|-1
chcin_cube8_5|1|-1|1
chcin_cube8_6|1|1|-1
chcin_cube8_7|1|1|1
chcv_cube8_0|1|1|1|7
chcv_cube8_1|1|1|-1|6
chcv_cube8_2|-1|1|1|3
chcv_cube8_3|1|-1|1|5
chcv_cube8_4|1|-1|-1|4
chcv_cube8_5|-1|1|-1|2
chcv_cube8_6|-1|-1|1|1
chcv_cube8_7|-1|-1|-1|0
chce_cube8_0|0|1|4|8|1
chce_cube8_1|1|0|8|4|0
chce_cube8_2|0|2|0|12|3
chce_cube8_3|2|0|12|0|2
chce_cube8_4|0|3|2|14|5
chce_cube8_5|3|0|14|2|4
chce_cube8_6|1|4|1|18|7
chce_cube8_7|4|1|18|1|6
chce_cube8_8|1|5|6|11|9
chce_cube8_9|5|1|11|6|8
chce_cube8_10|2|5|3|20|11
chce_cube8_11|5|2|20|3|10
chce_cube8_12|2|6|10|17|13
chce_cube8_13|6|2|17|10|12
chce_cube8_14|3|4|16|7|15
chce_cube8_15|4|3|7|16|14
chce_cube8_16|3|6|5|22|17
chce_cube8_17|6|3|22|5|16
chce_cube8_18|4|7|15|21|19
chce_cube8_19|7|4|21|15|18
chce_cube8_20|5|7|9|23|21
chce_cube8_21|7|5|23|9|20
chce_cube8_22|6|7|13|19|23
chce_cube8_23|7|6|19|13|22
chcf_cube8_0|0
chcf_cube8_1|2
chcf_cube8_2|4
chcf_cube8_3|6
chcf_cube8_4|10
chcf_cube8_5|16
chc_interior|13|0|0|0|8|24|6
chcin_interior_0|-1|-1|-1
chcin_interior_1|-1|-1|1
chcin_interior_2|-1|1|-1
chcin_interior_3|-1|1|1
chcin_interior_4|1|-1|-1
chcin_interior_5|1|-1|1
chcin_interior_6|1|1|-1
chcin_interior_7|1|1|1
chcin_interior_8|0|0|0
chcin_interior_9|0.25|-0.5|0.125
chcin_interior_10|-0.75|0.375|-0.625
chcin_interior_11|0.875|0.875|0.5
chcin_interior_12|-0.125|-0.9375|0.9375
chcv_interior_0|1|1|1|7
chcv_interior_1|1|-1|1|5
chcv_interior_2|1|1|-1|6
chcv_interior_3|-1|1|1|3
chcv_interior_4|1|-1|-1|4
chcv_interior_5|-1|-1|1|1
chcv_interior_6|-1|1|-1|2
chcv_interior_7|-1|-1|-1|0
chce_interior_0|0|1|4|6|1
chce_interior_1|1|0|6|4|0
chce_interior_2|0|2|0|12|3
chce_interior_3|2|0|12|0|2
chce_interior_4|0|3|2|16|5
chce_interior_5|3|0|16|2|4
chce_interior_6|1|4|8|11|7
chce_interior_7|4|1|11|8|6
chce_interior_8|1|5|1|20|9
chce_interior_9|5|1|20|1|8
chce_interior_10|2|4|3|18|11
chce_interior_11|4|2|18|3|10
chce_interior_12|2|6|10|15|13
chce_interior_13|6|2|15|10|12
chce_interior_14|3|6|5|22|15
chce_interior_15|6|3|22|5|14
chce_interior_16|3|5|14|9|17
chce_interior_17|5|3|9|14|16
chce_interior_18|4|7|7|23|19
chce_interior_19|7|4|23|7|18
chce_interior_20|5|7|17|19|21
chce_interior_21|7|5|19|17|20
chce_interior_22|6|7|13|21|23
chce_interior_23|7|6|21|13|22
chcf_interior_0|0
chcf_interior_1|2
chcf_interior_2|4
chcf_interior_3|8
chcf_interior_4|10
chcf_interior_5|14
chc_dup|11|0|0|0|5|16|5
chcin_dup_0|-1|-1|0
chcin_dup_1|-1|-1|0
chcin_dup_2|1|-1|0
chcin_dup_3|1|-1|0
chcin_dup_4|1|1|0
chcin_dup_5|1|1|0
chcin_dup_6|-1|1|0
chcin_dup_7|-1|1|0
chcin_dup_8|0|0|1.5
chcin_dup_9|0|0|1.5
chcin_dup_10|0|0|1.5
chcv_dup_0|1|1|0|4
chcv_dup_1|0|0|1.5|8
chcv_dup_2|1|-1|0|3
chcv_dup_3|-1|1|0|7
chcv_dup_4|-1|-1|0|1
chce_dup_0|0|1|4|6|1
chce_dup_1|1|0|6|4|0
chce_dup_2|0|2|0|12|3
chce_dup_3|2|0|12|0|2
chce_dup_4|0|3|2|9|5
chce_dup_5|3|0|9|2|4
chce_dup_6|1|2|10|3|7
chce_dup_7|2|1|3|10|6
chce_dup_8|1|3|1|14|9
chce_dup_9|3|1|14|1|8
chce_dup_10|1|4|8|13|11
chce_dup_11|4|1|13|8|10
chce_dup_12|2|4|7|15|13
chce_dup_13|4|2|15|7|12
chce_dup_14|3|4|5|11|15
chce_dup_15|4|3|11|5|14
chcf_dup_0|0
chcf_dup_1|2
chcf_dup_2|4
chcf_dup_3|8
chcf_dup_4|10
chc_flat|9|0|0|0|4|8|2
chcin_flat_0|-1|-1|0
chcin_flat_1|-1|0|0
chcin_flat_2|-1|1|0
chcin_flat_3|0|-1|0
chcin_flat_4|0|0|0
chcin_flat_5|0|1|0
chcin_flat_6|1|-1|0
chcin_flat_7|1|0|0
chcin_flat_8|1|1|0
chcv_flat_0|1|1|0|8
chcv_flat_1|1|-1|0|6
chcv_flat_2|-1|1|0|2
chcv_flat_3|-1|-1|0|0
chce_flat_0|0|1|2|4|1
chce_flat_1|1|0|4|2|0
chce_flat_2|0|2|0|6|3
chce_flat_3|2|0|6|0|2
chce_flat_4|1|3|1|7|5
chce_flat_5|3|1|7|1|4
chce_flat_6|2|3|3|5|7
chce_flat_7|3|2|5|3|6
chcf_flat_0|0
chcf_flat_1|2
chc_collinear|14|0|0|0|8|24|6
chcin_collinear_0|-1|-1|-1
chcin_collinear_1|-1|-1|1
chcin_collinear_2|-1|1|-1
chcin_collinear_3|-1|1|1
chcin_collinear_4|1|-1|-1
chcin_collinear_5|1|-1|1
chcin_collinear_6|1|1|-1
chcin_collinear_7|1|1|1
chcin_collinear_8|-1|-1|-0.5
chcin_collinear_9|-1|-1|0
chcin_collinear_10|-1|-1|0.5
chcin_collinear_11|1|-0.25|1
chcin_collinear_12|1|0.5|1
chcin_collinear_13|0.375|1|1
chcv_collinear_0|1|1|1|7
chcv_collinear_1|1|-1|1|5
chcv_collinear_2|1|1|-1|6
chcv_collinear_3|-1|1|1|3
chcv_collinear_4|1|-1|-1|4
chcv_collinear_5|-1|-1|1|1
chcv_collinear_6|-1|1|-1|2
chcv_collinear_7|-1|-1|-1|0
chce_collinear_0|0|1|4|6|1
chce_collinear_1|1|0|6|4|0
chce_collinear_2|0|2|0|12|3
chce_collinear_3|2|0|12|0|2
chce_collinear_4|0|3|2|16|5
chce_collinear_5|3|0|16|2|4
chce_collinear_6|1|4|8|11|7
chce_collinear_7|4|1|11|8|6
chce_collinear_8|1|5|1|20|9
chce_collinear_9|5|1|20|1|8
chce_collinear_10|2|4|3|18|11
chce_collinear_11|4|2|18|3|10
chce_collinear_12|2|6|10|15|13
chce_collinear_13|6|2|15|10|12
chce_collinear_14|3|6|5|22|15
chce_collinear_15|6|3|22|5|14
chce_collinear_16|3|5|14|9|17
chce_collinear_17|5|3|9|14|16
chce_collinear_18|4|7|7|23|19
chce_collinear_19|7|4|23|7|18
chce_collinear_20|5|7|17|19|21
chce_collinear_21|7|5|19|17|20
chce_collinear_22|6|7|13|21|23
chce_collinear_23|7|6|21|13|22
chcf_collinear_0|0
chcf_collinear_1|2
chcf_collinear_2|4
chcf_collinear_3|8
chcf_collinear_4|10
chcf_collinear_5|14
chc_vpairs|16|0|0|0|8|24|6
chcin_vpairs_0|-1.5|-1|-0.125
chcin_vpairs_1|-1.5|-1|0.125
chcin_vpairs_2|-1.5|1|-0.125
chcin_vpairs_3|-1.5|1|0.125
chcin_vpairs_4|-0.5|-1|-0.125
chcin_vpairs_5|-0.5|-1|0.125
chcin_vpairs_6|-0.5|1|-0.125
chcin_vpairs_7|-0.5|1|0.125
chcin_vpairs_8|0.5|-1|-0.125
chcin_vpairs_9|0.5|-1|0.125
chcin_vpairs_10|0.5|1|-0.125
chcin_vpairs_11|0.5|1|0.125
chcin_vpairs_12|1.5|-1|-0.125
chcin_vpairs_13|1.5|-1|0.125
chcin_vpairs_14|1.5|1|-0.125
chcin_vpairs_15|1.5|1|0.125
chcv_vpairs_0|1.5|1|0.125|15
chcv_vpairs_1|1.5|-1|0.125|13
chcv_vpairs_2|1.5|1|-0.125|14
chcv_vpairs_3|-1.5|1|0.125|3
chcv_vpairs_4|1.5|-1|-0.125|12
chcv_vpairs_5|-1.5|-1|0.125|1
chcv_vpairs_6|-1.5|1|-0.125|2
chcv_vpairs_7|-1.5|-1|-0.125|0
chce_vpairs_0|0|1|4|6|1
chce_vpairs_1|1|0|6|4|0
chce_vpairs_2|0|2|0|12|3
chce_vpairs_3|2|0|12|0|2
chce_vpairs_4|0|3|2|16|5
chce_vpairs_5|3|0|16|2|4
chce_vpairs_6|1|4|8|11|7
chce_vpairs_7|4|1|11|8|6
chce_vpairs_8|1|5|1|20|9
chce_vpairs_9|5|1|20|1|8
chce_vpairs_10|2|4|3|18|11
chce_vpairs_11|4|2|18|3|10
chce_vpairs_12|2|6|10|15|13
chce_vpairs_13|6|2|15|10|12
chce_vpairs_14|3|6|5|22|15
chce_vpairs_15|6|3|22|5|14
chce_vpairs_16|3|5|14|9|17
chce_vpairs_17|5|3|9|14|16
chce_vpairs_18|4|7|7|23|19
chce_vpairs_19|7|4|23|7|18
chce_vpairs_20|5|7|17|19|21
chce_vpairs_21|7|5|19|17|20
chce_vpairs_22|6|7|13|21|23
chce_vpairs_23|7|6|21|13|22
chcf_vpairs_0|0
chcf_vpairs_1|2
chcf_vpairs_2|4
chcf_vpairs_3|8
chcf_vpairs_4|10
chcf_vpairs_5|14
chc_cloud|32|0|0|0|20|108|36
chcin_cloud_0|-0.9453125|-0.6171875|0.8515625
chcin_cloud_1|0.7734375|0.1875|-0.9609375
chcin_cloud_2|-0.4375|-0.5|0.7265625
chcin_cloud_3|0.6796875|0.1796875|0.03125
chcin_cloud_4|-0.6171875|-0.078125|-0.21875
chcin_cloud_5|-0.5390625|0.9453125|-0.0234375
chcin_cloud_6|0.3671875|-0.9765625|-0.4765625
chcin_cloud_7|-0.296875|-0.9453125|-0.078125
chcin_cloud_8|-0.3046875|0.8125|0.625
chcin_cloud_9|-0.109375|-0.4375|0.171875
chcin_cloud_10|0.2421875|0.8125|0.375
chcin_cloud_11|0.2734375|-0.6796875|-0.8125
chcin_cloud_12|-0.03125|0.9609375|0.9921875
chcin_cloud_13|-0.2734375|-0.6015625|-0.5703125
chcin_cloud_14|0.2890625|-0.5703125|-0.96875
chcin_cloud_15|-0.328125|0.453125|0.2578125
chcin_cloud_16|-0.5546875|-0.4453125|-0.2578125
chcin_cloud_17|0.890625|-0.921875|-0.2109375
chcin_cloud_18|0.875|-0.328125|0.4609375
chcin_cloud_19|-0.546875|-0.8046875|-0.2109375
chcin_cloud_20|0.609375|0.0078125|0.9765625
chcin_cloud_21|-0.4375|0.4375|-0.453125
chcin_cloud_22|-0.390625|0.5703125|0.8984375
chcin_cloud_23|-0.578125|-0.4765625|0.71875
chcin_cloud_24|-0.8125|-0.0078125|0.0546875
chcin_cloud_25|0.1484375|0.609375|-0.7578125
chcin_cloud_26|-0.65625|-0.5703125|-0.1640625
chcin_cloud_27|-0.3828125|0.6796875|0.2109375
chcin_cloud_28|0.171875|-0.5703125|-0.40625
chcin_cloud_29|-0.6640625|-0.0625|-0.3046875
chcin_cloud_30|0.1953125|-0.1953125|0.0078125
chcin_cloud_31|0.3359375|-0.8515625|-0.3203125
chcv_cloud_0|0.367124051|-0.9765625|-0.47640419|6
chcv_cloud_1|0.288949341|-0.570135057|-0.96875|14
chcv_cloud_2|0.890625|-0.921752632|-0.210748613|17
chcv_cloud_3|-0.296732008|-0.945269644|-0.0781127661|7
chcv_cloud_4|-0.546711385|-0.804546773|-0.210748613|19
chcv_cloud_5|0.148414567|0.609319925|-0.757799506|25
chcv_cloud_6|0.77327311|0.18734093|-0.96088016|1
chcv_cloud_7|-0.273369431|-0.601427913|-0.570266604|13
chcv_cloud_8|-0.663883567|-0.0624327026|-0.304611027|29
chcv_cloud_9|-0.437446475|0.437493861|-0.452986568|21
chcv_cloud_10|0.874990106|-0.327947587|0.460876316|18
chcv_cloud_11|0.609196067|0.0077390857|0.976447761|20
chcv_cloud_12|-0.9453125|-0.617169142|0.851489842|0
chcv_cloud_13|-0.656155944|-0.570135057|-0.163913369|26
chcv_cloud_14|-0.538983762|0.945196271|-0.0234076753|5
chcv_cloud_15|0.242044508|0.812438786|0.374883771|10
chcv_cloud_16|0.679643154|0.179565147|0.0311054662|3
chcv_cloud_17|-0.812325656|-0.0078125|0.0545230806|24
chcv_cloud_18|-0.0311177019|0.9609375|0.9921875|12
chcv_cloud_19|-0.390541643|0.570251286|0.898325086|22
chce_cloud_0|0|1|6|12|1
chce_cloud_1|1|0|12|6|0
chce_cloud_2|0|2|0|22|3
chce_cloud_3|2|0|22|0|2
chce_cloud_4|0|3|2|32|5
chce_cloud_5|3|0|32|2|4
chce_cloud_6|0|4|4|15|7
chce_cloud_7|4|0|15|4|6
chce_cloud_8|1|5|20|42|9
chce_cloud_9|5|1|42|20|8
chce_cloud_10|1|6|8|25|11
chce_cloud_11|6|1|25|8|10
chce_cloud_12|1|2|10|3|13
chce_cloud_13|2|1|3|10|12
chce_cloud_14|1|4|1|40|15
chce_cloud_15|4|1|40|1|14
chce_cloud_16|1|7|14|58|17
chce_cloud_17|7|1|58|14|16
chce_cloud_18|1|8|16|60|19
chce_cloud_19|8|1|60|16|18
chce_cloud_20|1|9|18|45|21
chce_cloud_21|9|1|45|18|20
chce_cloud_22|2|3|30|5|23
chce_cloud_23|3|2|5|30|22
chce_cloud_24|2|6|13|54|25
chce_cloud_25|6|2|54|13|24
chce_cloud_26|2|10|24|72|27
chce_cloud_27|10|2|72|24|26
chce_cloud_28|2|11|26|76|29
chce_cloud_29|11|2|76|26|28
chce_cloud_30|2|12|28|35|31
chce_cloud_31|12|2|35|28|30
chce_cloud_32|3|4|34|7|33
chce_cloud_33|4|3|7|34|32
chce_cloud_34|3|12|23|37|35
chce_cloud_35|12|3|37|23|34
chce_cloud_36|4|12|33|90|37
chce_cloud_37|12|4|90|33|36
chce_cloud_38|4|13|36|57|39
chce_cloud_39|13|4|57|36|38
chce_cloud_40|4|7|38|17|41
chce_cloud_41|7|4|17|38|40
chce_cloud_42|5|6|48|11|43
chce_cloud_43|6|5|11|48|42
chce_cloud_44|5|9|9|68|45
chce_cloud_45|9|5|68|9|44
chce_cloud_46|5|14|44|100|47
chce_cloud_47|14|5|100|44|46
chce_cloud_48|5|15|46|51|49
chce_cloud_49|15|5|51|46|48
chce_cloud_50|6|15|43|102|51
chce_cloud_51|15|6|102|43|50
chce_cloud_52|6|16|50|71|53
chce_cloud_53|16|6|71|50|52
chce_cloud_54|6|10|52|27|55
chce_cloud_55|10|6|27|52|54
chce_cloud_56|7|13|41|63|57
chce_cloud_57|13|7|63|41|56
chce_cloud_58|7|8|56|19|59
chce_cloud_59|8|7|19|56|58
chce_cloud_60|8|9|66|21|61
chce_cloud_61|9|8|21|66|60
chce_cloud_62|8|13|59|92|63
chce_cloud_63|13|8|92|59|62
chce_cloud_64|8|17|62|95|65
chce_cloud_65|17|8|95|62|64
chce_cloud_66|8|14|64|69|67
chce_cloud_67|14|8|69|64|66
chce_cloud_68|9|14|61|47|69
chce_cloud_69|14|9|47|61|68
chce_cloud_70|10|16|55|79|71
chce_cloud_71|16|10|79|55|70
chce_cloud_72|10|11|70|29|73
chce_cloud_73|11|10|29|70|72
chce_cloud_74|11|18|80|83|75
chce_cloud_75|18|11|83|80|74
chce_cloud_76|11|12|74|31|77
chce_cloud_77|12|11|31|74|76
chce_cloud_78|11|16|73|103|79
chce_cloud_79|16|11|103|73|78
chce_cloud_80|11|15|78|104|81
chce_cloud_81|15|11|104|78|80
chce_cloud_82|12|18|77|106|83
chce_cloud_83|18|12|106|77|82
chce_cloud_84|12|19|82|97|85
chce_cloud_85|19|12|97|82|84
chce_cloud_86|12|14|84|94|87
chce_cloud_87|14|12|94|84|86
chce_cloud_88|12|17|86|93|89
chce_cloud_89|17|12|93|86|88
chce_cloud_90|12|13|88|39|91
chce_cloud_91|13|12|39|88|90
chce_cloud_92|13|17|91|65|93
chce_cloud_93|17|13|65|91|92
chce_cloud_94|14|17|67|89|95
chce_cloud_95|17|14|89|67|94
chce_cloud_96|14|19|87|107|97
chce_cloud_97|19|14|107|87|96
chce_cloud_98|14|18|96|105|99
chce_cloud_99|18|14|105|96|98
chce_cloud_100|14|15|98|49|101
chce_cloud_101|15|14|49|98|100
chce_cloud_102|15|16|81|53|103
chce_cloud_103|16|15|53|81|102
chce_cloud_104|15|18|101|75|105
chce_cloud_105|18|15|75|101|104
chce_cloud_106|18|19|99|85|107
chce_cloud_107|19|18|85|99|106
chcf_cloud_0|0
chcf_cloud_1|2
chcf_cloud_2|4
chcf_cloud_3|6
chcf_cloud_4|8
chcf_cloud_5|10
chcf_cloud_6|14
chcf_cloud_7|16
chcf_cloud_8|18
chcf_cloud_9|20
chcf_cloud_10|24
chcf_cloud_11|26
chcf_cloud_12|28
chcf_cloud_13|30
chcf_cloud_14|34
chcf_cloud_15|36
chcf_cloud_16|38
chcf_cloud_17|44
chcf_cloud_18|46
chcf_cloud_19|48
chcf_cloud_20|50
chcf_cloud_21|52
chcf_cloud_22|56
chcf_cloud_23|62
chcf_cloud_24|64
chcf_cloud_25|66
chcf_cloud_26|70
chcf_cloud_27|74
chcf_cloud_28|78
chcf_cloud_29|80
chcf_cloud_30|82
chcf_cloud_31|84
chcf_cloud_32|86
chcf_cloud_33|88
chcf_cloud_34|96
chcf_cloud_35|98
chc_shell|26|0|0|0|26|144|48
chcin_shell_0|-0.877380371|-0.365575135|-0.310738862
chcin_shell_1|-0.468856454|0.771659553|-0.429785073
chcin_shell_2|0.714781821|-0.539457977|0.445052832
chcin_shell_3|-0.764458418|0.601541042|0.231843948
chcin_shell_4|0.951296747|0.293610096|-0.0939552337
chcin_shell_5|0.144645944|0.964306295|-0.221790448
chcin_shell_6|-0.518605113|-0.845397413|0.127875239
chcin_shell_7|0.395685554|0.402507722|-0.825481892
chcin_shell_8|0.0561828315|0.346460789|-0.936380506
chcin_shell_9|-0.675678492|0.722277045|-0.147561967
chcin_shell_10|-0.257323265|-0.674364388|0.692110837
chcin_shell_11|-0.479326606|-0.874961257|-0.0684752315
chcin_shell_12|-0.43338874|0.198120564|-0.879159987
chcin_shell_13|0.210057974|0.0859328061|0.973905146
chcin_shell_14|-0.974879324|0.204724655|-0.0877391398
chcin_shell_15|0.724001527|0.403052419|0.559795022
chcin_shell_16|-0.941792369|0.146885037|-0.302410394
chcin_shell_17|-0.435426056|-0.891586721|-0.124407448
chcin_shell_18|0.376857221|-0.609386146|-0.697586775
chcin_shell_19|-0.664462328|-0.40490672|0.628124535
chcin_shell_20|-0.660315454|0.693058372|0.289229095
chcin_shell_21|-0.268759996|-0.379425883|0.885327041
chcin_shell_22|-0.701542735|-0.443710804|-0.557636559
chcin_shell_23|-0.643340886|0.216777921|-0.734247804
chcin_shell_24|-0.513777375|-0.347146869|-0.784551978
chcin_shell_25|-0.436367154|-0.65085268|-0.621268511
chcv_shell_0|0.0560849234|0.346280843|-0.936380506|8
chcv_shell_1|-0.433377981|0.198041931|-0.878974676|12
chcv_shell_2|-0.468824446|0.771559358|-0.429638714|1
chcv_shell_3|0.144512549|0.964306235|-0.221706301|5
chcv_shell_4|0.395654529|0.402415425|-0.825308681|7
chcv_shell_5|0.37680003|-0.609278798|-0.697407842|18
chcv_shell_6|-0.513698161|-0.347135723|-0.784544945|24
chcv_shell_7|-0.64322859|0.216753453|-0.734244823|23
chcv_shell_8|-0.675658345|0.722146392|-0.147471428|9
chcv_shell_9|0.723911405|0.40296042|0.559723258|15
chcv_shell_10|0.951296747|0.293597877|-0.0938054174|4
chcv_shell_11|-0.660197675|0.692898273|0.289149284|20
chcv_shell_12|0.714672744|-0.539337635|0.444911599|2
chcv_shell_13|-0.435263425|-0.891586661|-0.124284714|17
chcv_shell_14|-0.436206162|-0.650698483|-0.621116042|25
chcv_shell_15|-0.701489031|-0.443599999|-0.557539582|22
chcv_shell_16|-0.941695392|0.146812305|-0.302298814|16
chcv_shell_17|-0.974879324|0.204581872|-0.0876347572|14
chcv_shell_18|-0.764274538|0.601520598|0.23174347|3
chcv_shell_19|0.209937677|0.0857727528|0.973905206|13
chcv_shell_20|-0.664345622|-0.404905289|0.627974391|19
chcv_shell_21|-0.268589616|-0.379290462|0.885272145|21
chcv_shell_22|-0.257276922|-0.674314976|0.69192481|10
chcv_shell_23|-0.518600345|-0.845262051|0.127777249|6
chcv_shell_24|-0.479194432|-0.874873459|-0.0683748275|11
chcv_shell_25|-0.877213001|-0.36548391|-0.310713351|0
chce_shell_0|0|1|10|16|1
chce_shell_1|1|0|16|10|0
chce_shell_2|0|2|0|22|3
chce_shell_3|2|0|22|0|2
chce_shell_4|0|3|2|28|5
chce_shell_5|3|0|28|2|4
chce_shell_6|0|4|4|36|7
chce_shell_7|4|0|36|4|6
chce_shell_8|0|5|6|46|9
chce_shell_9|5|0|46|6|8
chce_shell_10|0|6|8|13|11
chce_shell_11|6|0|13|8|10
chce_shell_12|1|6|1|50|13
chce_shell_13|6|1|50|1|12
chce_shell_14|1|7|12|19|15
chce_shell_15|7|1|19|12|14
chce_shell_16|1|2|14|3|17
chce_shell_17|2|1|3|14|16
chce_shell_18|2|7|17|58|19
chce_shell_19|7|2|58|17|18
chce_shell_20|2|8|18|31|21
chce_shell_21|8|2|31|18|20
chce_shell_22|2|3|20|5|23
chce_shell_23|3|2|5|20|22
chce_shell_24|3|9|32|72|25
chce_shell_25|9|3|72|32|24
chce_shell_26|3|10|24|35|27
chce_shell_27|10|3|35|24|26
chce_shell_28|3|4|26|7|29
chce_shell_29|4|3|7|26|28
chce_shell_30|3|8|23|66|31
chce_shell_31|8|3|66|23|30
chce_shell_32|3|11|30|75|33
chce_shell_33|11|3|75|30|32
chce_shell_34|4|10|29|39|35
chce_shell_35|10|4|39|29|34
chce_shell_36|4|5|34|9|37
chce_shell_37|5|4|9|34|36
chce_shell_38|5|10|37|76|39
chce_shell_39|10|5|76|37|38
chce_shell_40|5|12|38|92|41
chce_shell_41|12|5|92|38|40
chce_shell_42|5|13|40|94|43
chce_shell_43|13|5|94|40|42
chce_shell_44|5|14|42|53|45
chce_shell_45|14|5|53|42|44
chce_shell_46|5|6|44|11|47
chce_shell_47|6|5|11|44|46
chce_shell_48|6|15|52|55|49
chce_shell_49|15|6|55|52|48
chce_shell_50|6|7|48|15|51
chce_shell_51|7|6|15|48|50
chce_shell_52|6|14|47|104|53
chce_shell_53|14|6|104|47|52
chce_shell_54|7|15|51|108|55
chce_shell_55|15|7|108|51|54
chce_shell_56|7|16|54|61|57
chce_shell_57|16|7|61|54|56
chce_shell_58|7|8|56|21|59
chce_shell_59|8|7|21|56|58
chce_shell_60|8|16|59|114|61
chce_shell_61|16|8|114|59|60
chce_shell_62|8|17|60|120|63
chce_shell_63|17|8|120|60|62
chce_shell_64|8|18|62|79|65
chce_shell_65|18|8|79|62|64
chce_shell_66|8|11|64|33|67
chce_shell_67|11|8|33|64|66
chce_shell_68|9|19|74|87|69
chce_shell_69|19|9|87|74|68
chce_shell_70|9|12|68|77|71
chce_shell_71|12|9|77|68|70
chce_shell_72|9|10|70|27|73
chce_shell_73|10|9|27|70|72
chce_shell_74|9|11|25|84|75
chce_shell_75|11|9|84|25|74
chce_shell_76|10|12|73|41|77
chce_shell_77|12|10|41|73|76
chce_shell_78|11|18|67|122|79
chce_shell_79|18|11|122|67|78
chce_shell_80|11|20|78|132|81
chce_shell_81|20|11|132|78|80
chce_shell_82|11|21|80|125|83
chce_shell_83|21|11|125|80|82
chce_shell_84|11|19|82|69|85
chce_shell_85|19|11|69|82|84
chce_shell_86|12|19|71|124|87
chce_shell_87|19|12|124|71|86
chce_shell_88|12|21|86|134|89
chce_shell_89|21|12|134|86|88
chce_shell_90|12|22|88|97|91
chce_shell_91|22|12|97|88|90
chce_shell_92|12|13|90|43|93
chce_shell_93|13|12|43|90|92
chce_shell_94|13|14|102|45|95
chce_shell_95|14|13|45|102|94
chce_shell_96|13|22|93|136|97
chce_shell_97|22|13|136|93|96
chce_shell_98|13|23|96|140|99
chce_shell_99|23|13|140|96|98
chce_shell_100|13|24|98|111|101
chce_shell_101|24|13|111|98|100
chce_shell_102|13|15|100|105|103
chce_shell_103|15|13|105|100|102
chce_shell_104|14|15|95|49|105
chce_shell_105|15|14|49|95|104
chce_shell_106|15|25|110|113|107
chce_shell_107|25|15|113|110|106
chce_shell_108|15|16|106|57|109
chce_shell_109|16|15|57|106|108
chce_shell_110|15|24|103|142|111
chce_shell_111|24|15|142|103|110
chce_shell_112|16|25|109|117|113
chce_shell_113|25|16|117|109|112
chce_shell_114|16|17|112|63|115
chce_shell_115|17|16|63|112|114
chce_shell_116|17|25|115|127|117
chce_shell_117|25|17|127|115|116
chce_shell_118|17|20|116|123|119
chce_shell_119|20|17|123|116|118
chce_shell_120|17|18|118|65|121
chce_shell_121|18|17|65|118|120
chce_shell_122|18|20|121|81|123
chce_shell_123|20|18|81|121|122
chce_shell_124|19|21|85|89|125
chce_shell_125|21|19|89|85|124
chce_shell_126|20|25|119|139|127
chce_shell_127|25|20|139|119|126
chce_shell_128|20|23|126|137|129
chce_shell_129|23|20|137|126|128
chce_shell_130|20|22|128|135|131
chce_shell_131|22|20|135|128|130
chce_shell_132|20|21|130|83|133
chce_shell_133|21|20|83|130|132
chce_shell_134|21|22|133|91|135
chce_shell_135|22|21|91|133|134
chce_shell_136|22|23|131|99|137
chce_shell_137|23|22|99|131|136
chce_shell_138|23|25|129|143|139
chce_shell_139|25|23|143|129|138
chce_shell_140|23|24|138|101|141
chce_shell_141|24|23|101|138|140
chce_shell_142|24|25|141|107|143
chce_shell_143|25|24|107|141|142
chcf_shell_0|0
chcf_shell_1|2
chcf_shell_2|4
chcf_shell_3|6
chcf_shell_4|8
chcf_shell_5|10
chcf_shell_6|12
chcf_shell_7|14
chcf_shell_8|18
chcf_shell_9|20
chcf_shell_10|24
chcf_shell_11|26
chcf_shell_12|30
chcf_shell_13|32
chcf_shell_14|34
chcf_shell_15|38
chcf_shell_16|40
chcf_shell_17|42
chcf_shell_18|44
chcf_shell_19|48
chcf_shell_20|52
chcf_shell_21|54
chcf_shell_22|56
chcf_shell_23|60
chcf_shell_24|62
chcf_shell_25|64
chcf_shell_26|68
chcf_shell_27|70
chcf_shell_28|74
chcf_shell_29|78
chcf_shell_30|80
chcf_shell_31|82
chcf_shell_32|86
chcf_shell_33|88
chcf_shell_34|90
chcf_shell_35|96
chcf_shell_36|98
chcf_shell_37|100
chcf_shell_38|102
chcf_shell_39|106
chcf_shell_40|110
chcf_shell_41|112
chcf_shell_42|116
chcf_shell_43|118
chcf_shell_44|126
chcf_shell_45|128
chcf_shell_46|130
chcf_shell_47|138
chc_one|1|0|0|0|1|0|0
chcin_one_0|0.25|-0.5|0.75
chcv_one_0|0.25|-0.5|0.75|0
chc_two|2|0|0|0|2|2|1
chcin_two_0|-1|0|0
chcin_two_1|1|0.25|-0.5
chcv_two_0|1|0.25|-0.5|1
chcv_two_1|-1|0|0|0
chce_two_0|0|1|0|1|1
chce_two_1|1|0|1|0|0
chcf_two_0|0
chc_three|3|0|0|0|3|6|2
chcin_three_0|-1|-1|0
chcin_three_1|1|-0.5|0
chcin_three_2|0|1|0
chcv_three_0|0|1|0|2
chcv_three_1|-1|-1|0|0
chcv_three_2|1|-0.5|0|1
chce_three_0|0|1|2|4|1
chce_three_1|1|0|4|2|0
chce_three_2|0|2|0|5|3
chce_three_3|2|0|5|0|2
chce_three_4|1|2|1|3|5
chce_three_5|2|1|3|1|4
chcf_three_0|0
chcf_three_1|2
chc_four|4|0|0|0|4|12|4
chcin_four_0|-1|-1|-1
chcin_four_1|1|-1|-1
chcin_four_2|0|1|-1
chcin_four_3|0|0|1
chcv_four_0|0|1|-1|2
chcv_four_1|1|-1|-1|1
chcv_four_2|-1|-1|-1|0
chcv_four_3|0|0|1|3
chce_four_0|0|1|4|8|1
chce_four_1|1|0|8|4|0
chce_four_2|0|2|0|10|3
chce_four_3|2|0|10|0|2
chce_four_4|0|3|2|7|5
chce_four_5|3|0|7|2|4
chce_four_6|1|3|1|11|7
chce_four_7|3|1|11|1|6
chce_four_8|1|2|6|3|9
chce_four_9|2|1|3|6|8
chce_four_10|2|3|9|5|11
chce_four_11|3|2|5|9|10
chcf_four_0|0
chcf_four_1|2
chcf_four_2|4
chcf_four_3|6
chc_same|5|0|0|0|1|0|0
chcin_same_0|0.5|-0.25|0.125
chcin_same_1|0.5|-0.25|0.125
chcin_same_2|0.5|-0.25|0.125
chcin_same_3|0.5|-0.25|0.125
chcin_same_4|0.5|-0.25|0.125
chcv_same_0|0.5|-0.25|0.125|3
chc_shrunk|8|0.100000001|0|0.100000001|8|24|6
chcin_shrunk_0|-1|-1|-1
chcin_shrunk_1|-1|-1|1
chcin_shrunk_2|-1|1|-1
chcin_shrunk_3|-1|1|1
chcin_shrunk_4|1|-1|-1
chcin_shrunk_5|1|-1|1
chcin_shrunk_6|1|1|-1
chcin_shrunk_7|1|1|1
chcv_shrunk_0|0.900156736|0.900156736|-0.900156736|-1
chcv_shrunk_1|0.900156736|0.900156736|0.900156736|-1
chcv_shrunk_2|0.900156736|-0.900156736|-0.900156736|-1
chcv_shrunk_3|-0.900156736|0.900156736|-0.900156736|-1
chcv_shrunk_4|-0.900156736|0.900156736|0.900156736|-1
chcv_shrunk_5|0.900156736|-0.900156736|0.900156736|-1
chcv_shrunk_6|-0.900156736|-0.900156736|-0.900156736|-1
chcv_shrunk_7|-0.900156736|-0.900156736|0.900156736|-1
chce_shrunk_0|0|1|4|8|1
chce_shrunk_1|1|0|8|4|0
chce_shrunk_2|0|2|0|12|3
chce_shrunk_3|2|0|12|0|2
chce_shrunk_4|0|3|2|14|5
chce_shrunk_5|3|0|14|2|4
chce_shrunk_6|1|4|1|18|7
chce_shrunk_7|4|1|18|1|6
chce_shrunk_8|1|5|6|11|9
chce_shrunk_9|5|1|11|6|8
chce_shrunk_10|2|5|3|20|11
chce_shrunk_11|5|2|20|3|10
chce_shrunk_12|2|6|10|17|13
chce_shrunk_13|6|2|17|10|12
chce_shrunk_14|3|4|16|7|15
chce_shrunk_15|4|3|7|16|14
chce_shrunk_16|3|6|5|22|17
chce_shrunk_17|6|3|22|5|16
chce_shrunk_18|4|7|15|21|19
chce_shrunk_19|7|4|21|15|18
chce_shrunk_20|5|7|9|23|21
chce_shrunk_21|7|5|23|9|20
chce_shrunk_22|6|7|13|19|23
chce_shrunk_23|7|6|19|13|22
chcf_shrunk_0|0
chcf_shrunk_1|2
chcf_shrunk_2|4
chcf_shrunk_3|6
chcf_shrunk_4|10
chcf_shrunk_5|16
chc_clamped|8|0.5|0.25|0.25|8|24|6
chcin_clamped_0|-1|-1|-1
chcin_clamped_1|-1|-1|1
chcin_clamped_2|-1|1|-1
chcin_clamped_3|-1|1|1
chcin_clamped_4|1|-1|-1
chcin_clamped_5|1|-1|1
chcin_clamped_6|1|1|-1
chcin_clamped_7|1|1|1
chcv_clamped_0|0.75|0.75|-0.75|-1
chcv_clamped_1|0.75|0.75|0.75|-1
chcv_clamped_2|0.75|-0.75|-0.75|-1
chcv_clamped_3|-0.75|0.75|-0.75|-1
chcv_clamped_4|-0.75|0.75|0.75|-1
chcv_clamped_5|0.75|-0.75|0.75|-1
chcv_clamped_6|-0.75|-0.75|-0.75|-1
chcv_clamped_7|-0.75|-0.75|0.75|-1
chce_clamped_0|0|1|4|8|1
chce_clamped_1|1|0|8|4|0
chce_clamped_2|0|2|0|12|3
chce_clamped_3|2|0|12|0|2
chce_clamped_4|0|3|2|14|5
chce_clamped_5|3|0|14|2|4
chce_clamped_6|1|4|1|18|7
chce_clamped_7|4|1|18|1|6
chce_clamped_8|1|5|6|11|9
chce_clamped_9|5|1|11|6|8
chce_clamped_10|2|5|3|20|11
chce_clamped_11|5|2|20|3|10
chce_clamped_12|2|6|10|17|13
chce_clamped_13|6|2|17|10|12
chce_clamped_14|3|4|16|7|15
chce_clamped_15|4|3|7|16|14
chce_clamped_16|3|6|5|22|17
chce_clamped_17|6|3|22|5|16
chce_clamped_18|4|7|15|21|19
chce_clamped_19|7|4|21|15|18
chce_clamped_20|5|7|9|23|21
chce_clamped_21|7|5|23|9|20
chce_clamped_22|6|7|13|19|23
chce_clamped_23|7|6|19|13|22
chcf_clamped_0|0
chcf_clamped_1|2
chcf_clamped_2|4
chcf_clamped_3|6
chcf_clamped_4|10
chcf_clamped_5|16
chc_collapse|8|2|0|-2|0|0|0
chcin_collapse_0|-1|-1|-1
chcin_collapse_1|-1|-1|1
chcin_collapse_2|-1|1|-1
chcin_collapse_3|-1|1|1
chcin_collapse_4|1|-1|-1
chcin_collapse_5|1|-1|1
chcin_collapse_6|1|1|-1
chcin_collapse_7|1|1|1
chc_shrunk_shell|26|0.0500000007|0|0.0500000007|90|270|47
chcin_shrunk_shell_0|-0.877380371|-0.365575135|-0.310738862
chcin_shrunk_shell_1|-0.468856454|0.771659553|-0.429785073
chcin_shrunk_shell_2|0.714781821|-0.539457977|0.445052832
chcin_shrunk_shell_3|-0.764458418|0.601541042|0.231843948
chcin_shrunk_shell_4|0.951296747|0.293610096|-0.0939552337
chcin_shrunk_shell_5|0.144645944|0.964306295|-0.221790448
chcin_shrunk_shell_6|-0.518605113|-0.845397413|0.127875239
chcin_shrunk_shell_7|0.395685554|0.402507722|-0.825481892
chcin_shrunk_shell_8|0.0561828315|0.346460789|-0.936380506
chcin_shrunk_shell_9|-0.675678492|0.722277045|-0.147561967
chcin_shrunk_shell_10|-0.257323265|-0.674364388|0.692110837
chcin_shrunk_shell_11|-0.479326606|-0.874961257|-0.0684752315
chcin_shrunk_shell_12|-0.43338874|0.198120564|-0.879159987
chcin_shrunk_shell_13|0.210057974|0.0859328061|0.973905146
chcin_shrunk_shell_14|-0.974879324|0.204724655|-0.0877391398
chcin_shrunk_shell_15|0.724001527|0.403052419|0.559795022
chcin_shrunk_shell_16|-0.941792369|0.146885037|-0.302410394
chcin_shrunk_shell_17|-0.435426056|-0.891586721|-0.124407448
chcin_shrunk_shell_18|0.376857221|-0.609386146|-0.697586775
chcin_shrunk_shell_19|-0.664462328|-0.40490672|0.628124535
chcin_shrunk_shell_20|-0.660315454|0.693058372|0.289229095
chcin_shrunk_shell_21|-0.268759996|-0.379425883|0.885327041
chcin_shrunk_shell_22|-0.701542735|-0.443710804|-0.557636559
chcin_shrunk_shell_23|-0.643340886|0.216777921|-0.734247804
chcin_shrunk_shell_24|-0.513777375|-0.347146869|-0.784551978
chcin_shrunk_shell_25|-0.436367154|-0.65085268|-0.621268511
chcv_shrunk_shell_0|0.669640601|-0.500733256|0.422792494|-1
chcv_shrunk_shell_1|0.667365789|-0.501482427|0.423186749|-1
chcv_shrunk_shell_2|0.669891536|-0.500644922|0.422581792|-1
chcv_shrunk_shell_3|0.205290884|0.0743602663|0.909459829|-1
chcv_shrunk_shell_4|0.643421292|-0.504652321|0.429967195|-1
chcv_shrunk_shell_5|-0.422318637|-0.835275233|-0.116186127|-1
chcv_shrunk_shell_6|0.35108301|-0.566639721|-0.655283093|-1
chcv_shrunk_shell_7|0.672589362|-0.493862987|0.421075583|-1
chcv_shrunk_shell_8|0.196157217|0.0728179589|0.911860466|-1
chcv_shrunk_shell_9|0.681039095|0.36795783|0.526081681|-1
chcv_shrunk_shell_10|-0.248627514|-0.359452128|0.829477191|-1
chcv_shrunk_shell_11|-0.253053099|-0.629148185|0.657799006|-1
chcv_shrunk_shell_12|-0.432576388|-0.833370149|-0.0986833051|-1
chcv_shrunk_shell_13|-0.417842478|-0.833353996|-0.129953176|-1
chcv_shrunk_shell_14|0.349658459|-0.567039251|-0.657056451|-1
chcv_shrunk_shell_15|0.88924545|0.279134214|-0.0898557603|-1
chcv_shrunk_shell_16|0.89128685|0.275969446|-0.0768282861|-1
chcv_shrunk_shell_17|-0.622673392|0.644146323|0.267478913|-1
chcv_shrunk_shell_18|0.68277061|0.37144959|0.521751046|-1
chcv_shrunk_shell_19|-0.254301071|-0.364425153|0.828231335|-1
chcv_shrunk_shell_20|-0.262855142|-0.373519748|0.825327635|-1
chcv_shrunk_shell_21|-0.253339827|-0.629260182|0.657549262|-1
chcv_shrunk_shell_22|-0.281047672|-0.648395956|0.596196294|-1
chcv_shrunk_shell_23|-0.450972199|-0.82919842|-0.0689765066|-1
chcv_shrunk_shell_24|-0.417892307|-0.832709134|-0.13128379|-1
chcv_shrunk_shell_25|0.327390581|-0.574279189|-0.655910015|-1
chcv_shrunk_shell_26|0.347785503|-0.541004121|-0.663383007|-1
chcv_shrunk_shell_27|0.888491273|0.279872447|-0.0909375846|-1
chcv_shrunk_shell_28|0.685073376|0.37514922|0.515850246|-1
chcv_shrunk_shell_29|-0.623944879|0.645175338|0.26630643|-1
chcv_shrunk_shell_30|-0.606179297|-0.395740956|0.602118134|-1
chcv_shrunk_shell_31|-0.490932822|-0.785694242|0.143094078|-1
chcv_shrunk_shell_32|-0.452785134|-0.828702033|-0.0656837076|-1
chcv_shrunk_shell_33|-0.653617263|-0.436046779|-0.514902115|-1
chcv_shrunk_shell_34|-0.418756217|-0.611947|-0.586604953|-1
chcv_shrunk_shell_35|0.321586579|-0.57422936|-0.6560781|-1
chcv_shrunk_shell_36|0.0560052767|0.32834509|-0.880795598|-1
chcv_shrunk_shell_37|0.364943445|0.379654408|-0.779774785|-1
chcv_shrunk_shell_38|0.14813982|0.8953529|-0.208306774|-1
chcv_shrunk_shell_39|0.367471665|0.38190943|-0.776860118|-1
chcv_shrunk_shell_40|-0.631012022|0.649816155|0.260125339|-1
chcv_shrunk_shell_41|-0.621348381|-0.391655236|0.590679288|-1
chcv_shrunk_shell_42|-0.493488878|-0.795059621|0.104619898|-1
chcv_shrunk_shell_43|-0.811281979|-0.369892925|-0.283958882|-1
chcv_shrunk_shell_44|-0.661735058|-0.421260804|-0.53044343|-1
chcv_shrunk_shell_45|-0.424987644|-0.604583323|-0.590284109|-1
chcv_shrunk_shell_46|-0.491997629|-0.334728032|-0.735689044|-1
chcv_shrunk_shell_47|0.0431529321|0.3278642|-0.881025314|-1
chcv_shrunk_shell_48|0.0579219982|0.328900397|-0.880200267|-1
chcv_shrunk_shell_49|0.135412812|0.902194798|-0.217136458|-1
chcv_shrunk_shell_50|0.138731211|0.893680394|-0.227098271|-1
chcv_shrunk_shell_51|-0.632743478|0.651176035|0.258272439|-1
chcv_shrunk_shell_52|-0.634677768|-0.32033512|0.559552073|-1
chcv_shrunk_shell_53|-0.625925124|-0.385329425|0.585643053|-1
chcv_shrunk_shell_54|-0.826463819|-0.348191231|-0.298678637|-1
chcv_shrunk_shell_55|-0.666029871|-0.419248551|-0.524659157|-1
chcv_shrunk_shell_56|-0.66114831|-0.421395302|-0.531253695|-1
chcv_shrunk_shell_57|-0.493103296|-0.337750763|-0.733938575|-1
chcv_shrunk_shell_58|-0.658172071|-0.422543198|-0.534400225|-1
chcv_shrunk_shell_59|-0.490546912|-0.311727852|-0.741194487|-1
chcv_shrunk_shell_60|0.0313975997|0.323469937|-0.880282044|-1
chcv_shrunk_shell_61|-0.445620507|0.723865628|-0.409168601|-1
chcv_shrunk_shell_62|0.113832995|0.90297699|-0.215678006|-1
chcv_shrunk_shell_63|-0.636044264|0.650416553|0.255785108|-1
chcv_shrunk_shell_64|-0.723171115|0.570919573|0.20866403|-1
chcv_shrunk_shell_65|-0.629997671|-0.374549389|0.579558849|-1
chcv_shrunk_shell_66|-0.826634765|-0.347853541|-0.299069613|-1
chcv_shrunk_shell_67|-0.891777813|0.13562578|-0.284781724|-1
chcv_shrunk_shell_68|-0.606160343|0.20186618|-0.698033333|-1
chcv_shrunk_shell_69|-0.490600467|-0.301204324|-0.742908478|-1
chcv_shrunk_shell_70|-0.416082114|0.187946066|-0.827800155|-1
chcv_shrunk_shell_71|-0.448452801|0.711697638|-0.417454422|-1
chcv_shrunk_shell_72|-0.445850253|0.727422297|-0.405036896|-1
chcv_shrunk_shell_73|0.106919952|0.901135504|-0.217637718|-1
chcv_shrunk_shell_74|-0.719815791|0.576866567|0.20957911|-1
chcv_shrunk_shell_75|-0.648488581|0.673958421|-0.0956512764|-1
chcv_shrunk_shell_76|-0.915271282|0.208857656|-0.0826522335|-1
chcv_shrunk_shell_77|-0.920422375|0.195469871|-0.0897103697|-1
chcv_shrunk_shell_78|-0.836555839|-0.303098649|-0.291987836|-1
chcv_shrunk_shell_79|-0.892063558|0.137847736|-0.284695208|-1
chcv_shrunk_shell_80|-0.606275976|0.203016073|-0.698034167|-1
chcv_shrunk_shell_81|-0.418777615|0.18629761|-0.827348232|-1
chcv_shrunk_shell_82|-0.607545972|0.20559825|-0.695319176|-1
chcv_shrunk_shell_83|-0.632259369|0.682888865|-0.150733516|-1
chcv_shrunk_shell_84|-0.643824935|0.67843926|-0.136281431|-1
chcv_shrunk_shell_85|-0.645636082|0.675247431|-0.13657248|-1
chcv_shrunk_shell_86|-0.920908272|0.189256012|-0.0993192196|-1
chcv_shrunk_shell_87|-0.892904103|0.140503615|-0.280476689|-1
chcv_shrunk_shell_88|-0.60787791|0.204442322|-0.695973277|-1
chcv_shrunk_shell_89|-0.638031006|0.680684626|-0.143732384|-1
chce_shrunk_shell_0|0|1|4|8|1
chce_shrunk_shell_1|1|0|8|4|0
chce_shrunk_shell_2|0|2|0|12|3
chce_shrunk_shell_3|2|0|12|0|2
chce_shrunk_shell_4|0|3|2|14|5
chce_shrunk_shell_5|3|0|14|2|4
chce_shrunk_shell_6|1|4|1|20|7
chce_shrunk_shell_7|4|1|20|1|6
chce_shrunk_shell_8|1|5|6|24|9
chce_shrunk_shell_9|5|1|24|6|8
chce_shrunk_shell_10|2|6|3|28|11
chce_shrunk_shell_11|6|2|28|3|10
chce_shrunk_shell_12|2|7|10|32|13
chce_shrunk_shell_13|7|2|32|10|12
chce_shrunk_shell_14|3|8|16|34|15
chce_shrunk_shell_15|8|3|34|16|14
chce_shrunk_shell_16|3|9|5|38|17
chce_shrunk_shell_17|9|3|38|5|16
chce_shrunk_shell_18|4|10|7|40|19
chce_shrunk_shell_19|10|4|40|7|18
chce_shrunk_shell_20|4|11|18|44|21
chce_shrunk_shell_21|11|4|44|18|20
chce_shrunk_shell_22|5|12|9|48|23
chce_shrunk_shell_23|12|5|48|9|22
chce_shrunk_shell_24|5|13|22|52|25
chce_shrunk_shell_25|13|5|52|22|24
chce_shrunk_shell_26|6|14|11|56|27
chce_shrunk_shell_27|14|6|56|11|26
chce_shrunk_shell_28|6|15|26|58|29
chce_shrunk_shell_29|15|6|58|26|28
chce_shrunk_shell_30|7|16|13|62|31
chce_shrunk_shell_31|16|7|62|13|30
chce_shrunk_shell_32|7|9|30|17|33
chce_shrunk_shell_33|9|7|17|30|32
chce_shrunk_shell_34|8|10|36|19|35
chce_shrunk_shell_35|10|8|19|36|34
chce_shrunk_shell_36|8|17|15|64|37
chce_shrunk_shell_37|17|8|64|15|36
chce_shrunk_shell_38|9|18|33|70|39
chce_shrunk_shell_39|18|9|70|33|38
chce_shrunk_shell_40|10|19|35|72|41
chce_shrunk_shell_41|19|10|72|35|40
chce_shrunk_shell_42|11|20|21|74|43
chce_shrunk_shell_43|20|11|74|21|42
chce_shrunk_shell_44|11|21|42|78|45
chce_shrunk_shell_45|21|11|78|42|44
chce_shrunk_shell_46|12|22|23|80|47
chce_shrunk_shell_47|22|12|80|23|46
chce_shrunk_shell_48|12|23|46|84|49
chce_shrunk_shell_49|23|12|84|46|48
chce_shrunk_shell_50|13|24|25|88|51
chce_shrunk_shell_51|24|13|88|25|50
chce_shrunk_shell_52|13|25|50|55|53
chce_shrunk_shell_53|25|13|55|50|52
chce_shrunk_shell_54|14|25|27|90|55
chce_shrunk_shell_55|25|14|90|27|54
chce_shrunk_shell_56|14|26|54|94|57
chce_shrunk_shell_57|26|14|94|54|56
chce_shrunk_shell_58|15|16|60|31|59
chce_shrunk_shell_59|16|15|31|60|58
chce_shrunk_shell_60|15|27|29|96|61
chce_shrunk_shell_61|27|15|96|29|60
chce_shrunk_shell_62|16|28|59|69|63
chce_shrunk_shell_63|28|16|69|59|62
chce_shrunk_shell_64|17|19|66|41|65
chce_shrunk_shell_65|19|17|41|66|64
chce_shrunk_shell_66|17|29|37|102|67
chce_shrunk_shell_67|29|17|102|37|66
chce_shrunk_shell_68|18|28|39|100|69
chce_shrunk_shell_69|28|18|100|39|68
chce_shrunk_shell_70|18|29|68|67|71
chce_shrunk_shell_71|29|18|67|68|70
chce_shrunk_shell_72|19|20|65|43|73
chce_shrunk_shell_73|20|19|43|65|72
chce_shrunk_shell_74|20|30|73|77|75
chce_shrunk_shell_75|30|20|77|73|74
chce_shrunk_shell_76|21|30|45|104|77
chce_shrunk_shell_77|30|21|104|45|76
chce_shrunk_shell_78|21|22|76|47|79
chce_shrunk_shell_79|22|21|47|76|78
chce_shrunk_shell_80|22|31|79|108|81
chce_shrunk_shell_81|31|22|108|79|80
chce_shrunk_shell_82|23|32|49|112|83
chce_shrunk_shell_83|32|23|112|49|82
chce_shrunk_shell_84|23|33|82|87|85
chce_shrunk_shell_85|33|23|87|82|84
chce_shrunk_shell_86|24|33|51|114|87
chce_shrunk_shell_87|33|24|114|51|86
chce_shrunk_shell_88|24|34|86|116|89
chce_shrunk_shell_89|34|24|116|86|88
chce_shrunk_shell_90|25|35|53|120|91
chce_shrunk_shell_91|35|25|120|53|90
chce_shrunk_shell_92|26|36|57|124|93
chce_shrunk_shell_93|36|26|124|57|92
chce_shrunk_shell_94|26|37|92|128|95
chce_shrunk_shell_95|37|26|128|92|94
chce_shrunk_shell_96|27|38|98|101|97
chce_shrunk_shell_97|38|27|101|98|96
chce_shrunk_shell_98|27|39|61|132|99
chce_shrunk_shell_99|39|27|132|61|98
chce_shrunk_shell_100|28|38|63|130|101
chce_shrunk_shell_101|38|28|130|63|100
chce_shrunk_shell_102|29|40|71|136|103
chce_shrunk_shell_103|40|29|136|71|102
chce_shrunk_shell_104|30|41|75|107|105
chce_shrunk_shell_105|41|30|107|75|104
chce_shrunk_shell_106|31|41|81|138|107
chce_shrunk_shell_107|41|31|138|81|106
chce_shrunk_shell_108|31|42|106|111|109
chce_shrunk_shell_109|42|31|111|106|108
chce_shrunk_shell_110|32|42|83|140|111
chce_shrunk_shell_111|42|32|140|83|110
chce_shrunk_shell_112|32|43|110|142|113
chce_shrunk_shell_113|43|32|142|110|112
chce_shrunk_shell_114|33|44|85|146|115
chce_shrunk_shell_115|44|33|146|85|114
chce_shrunk_shell_116|34|35|118|91|117
chce_shrunk_shell_117|35|34|91|118|116
chce_shrunk_shell_118|34|45|89|148|119
chce_shrunk_shell_119|45|34|148|89|118
chce_shrunk_shell_120|35|46|117|152|121
chce_shrunk_shell_121|46|35|152|117|120
chce_shrunk_shell_122|36|47|93|158|123
chce_shrunk_shell_123|47|36|158|93|122
chce_shrunk_shell_124|36|48|122|127|125
chce_shrunk_shell_125|48|36|127|122|124
chce_shrunk_shell_126|37|48|95|160|127
chce_shrunk_shell_127|48|37|160|95|126
chce_shrunk_shell_128|37|39|126|99|129
chce_shrunk_shell_129|39|37|99|126|128
chce_shrunk_shell_130|38|49|97|164|131
chce_shrunk_shell_131|49|38|164|97|130
chce_shrunk_shell_132|39|50|129|163|133
chce_shrunk_shell_133|50|39|163|129|132
chce_shrunk_shell_134|40|51|103|168|135
chce_shrunk_shell_135|51|40|168|103|134
chce_shrunk_shell_136|40|52|134|172|137
chce_shrunk_shell_137|52|40|172|134|136
chce_shrunk_shell_138|41|53|105|174|139
chce_shrunk_shell_139|53|41|174|105|138
chce_shrunk_shell_140|42|43|109|113|141
chce_shrunk_shell_141|43|42|113|109|140
chce_shrunk_shell_142|43|54|141|178|143
chce_shrunk_shell_143|54|43|178|141|142
chce_shrunk_shell_144|44|55|115|180|145
chce_shrunk_shell_145|55|44|180|115|144
chce_shrunk_shell_146|44|56|144|186|147
chce_shrunk_shell_147|56|44|186|144|146
chce_shrunk_shell_148|45|57|150|155|149
chce_shrunk_shell_149|57|45|155|150|148
chce_shrunk_shell_150|45|58|119|189|151
chce_shrunk_shell_151|58|45|189|119|150
chce_shrunk_shell_152|46|59|154|192|153
chce_shrunk_shell_153|59|46|192|154|152
chce_shrunk_shell_154|46|57|121|188|155
chce_shrunk_shell_155|57|46|188|121|154
chce_shrunk_shell_156|47|60|123|194|157
chce_shrunk_shell_157|60|47|194|123|156
chce_shrunk_shell_158|47|61|156|198|159
chce_shrunk_shell_159|61|47|198|156|158
chce_shrunk_shell_160|48|50|125|133|161
chce_shrunk_shell_161|50|48|133|125|160
chce_shrunk_shell_162|49|50|131|161|163
chce_shrunk_shell_163|50|49|161|131|162
chce_shrunk_shell_164|49|62|162|167|165
chce_shrunk_shell_165|62|49|167|162|164
chce_shrunk_shell_166|51|62|135|200|167
chce_shrunk_shell_167|62|51|200|135|166
chce_shrunk_shell_168|51|63|166|202|169
chce_shrunk_shell_169|63|51|202|166|168
chce_shrunk_shell_170|52|64|137|208|171
chce_shrunk_shell_171|64|52|208|137|170
chce_shrunk_shell_172|52|65|170|177|173
chce_shrunk_shell_173|65|52|177|170|172
chce_shrunk_shell_174|53|54|176|143|175
chce_shrunk_shell_175|54|53|143|176|174
chce_shrunk_shell_176|53|65|139|210|177
chce_shrunk_shell_177|65|53|210|139|176
chce_shrunk_shell_178|54|66|175|183|179
chce_shrunk_shell_179|66|54|183|175|178
chce_shrunk_shell_180|55|67|182|216|181
chce_shrunk_shell_181|67|55|216|182|180
chce_shrunk_shell_182|55|66|145|212|183
chce_shrunk_shell_183|66|55|212|145|182
chce_shrunk_shell_184|56|68|147|218|185
chce_shrunk_shell_185|68|56|218|147|184
chce_shrunk_shell_186|56|58|184|151|187
chce_shrunk_shell_187|58|56|151|184|186
chce_shrunk_shell_188|57|58|149|187|189
chce_shrunk_shell_189|58|57|187|149|188
chce_shrunk_shell_190|59|69|153|222|191
chce_shrunk_shell_191|69|59|222|153|190
chce_shrunk_shell_192|59|60|190|157|193
chce_shrunk_shell_193|60|59|157|190|192
chce_shrunk_shell_194|60|70|193|224|195
chce_shrunk_shell_195|70|60|224|193|194
chce_shrunk_shell_196|61|71|159|228|197
chce_shrunk_shell_197|71|61|228|159|196
chce_shrunk_shell_198|61|72|196|232|199
chce_shrunk_shell_199|72|61|232|196|198
chce_shrunk_shell_200|62|73|165|234|201
chce_shrunk_shell_201|73|62|234|165|200
chce_shrunk_shell_202|63|74|204|207|203
chce_shrunk_shell_203|74|63|207|204|202
chce_shrunk_shell_204|63|75|169|237|205
chce_shrunk_shell_205|75|63|237|169|204
chce_shrunk_shell_206|64|74|171|236|207
chce_shrunk_shell_207|74|64|236|171|206
chce_shrunk_shell_208|64|76|206|242|209
chce_shrunk_shell_209|76|64|242|206|208
chce_shrunk_shell_210|65|77|173|244|211
chce_shrunk_shell_211|77|65|244|173|210
chce_shrunk_shell_212|66|78|179|215|213
chce_shrunk_shell_213|78|66|215|179|212
chce_shrunk_shell_214|67|78|181|246|215
chce_shrunk_shell_215|78|67|246|181|214
chce_shrunk_shell_216|67|79|214|250|217
chce_shrunk_shell_217|79|67|250|214|216
chce_shrunk_shell_218|68|69|220|191|219
chce_shrunk_shell_219|69|68|191|220|218
chce_shrunk_shell_220|68|80|185|252|221
chce_shrunk_shell_221|80|68|252|185|220
chce_shrunk_shell_222|69|81|219|227|223
chce_shrunk_shell_223|81|69|227|219|222
chce_shrunk_shell_224|70|71|226|197|225
chce_shrunk_shell_225|71|70|197|226|224
chce_shrunk_shell_226|70|81|195|253|227
chce_shrunk_shell_227|81|70|253|195|226
chce_shrunk_shell_228|71|82|225|258|229
chce_shrunk_shell_229|82|71|258|225|228
chce_shrunk_shell_230|72|83|199|235|231
chce_shrunk_shell_231|83|72|235|199|230
chce_shrunk_shell_232|72|73|230|201|233
chce_shrunk_shell_233|73|72|201|230|232
chce_shrunk_shell_234|73|83|233|260|235
chce_shrunk_shell_235|83|73|260|233|234
chce_shrunk_shell_236|74|75|203|238|237
chce_shrunk_shell_237|75|74|238|203|236
chce_shrunk_shell_238|75|84|205|264|239
chce_shrunk_shell_239|84|75|264|205|238
chce_shrunk_shell_240|76|85|209|266|241
chce_shrunk_shell_241|85|76|266|209|240
chce_shrunk_shell_242|76|77|240|211|243
chce_shrunk_shell_243|77|76|211|240|242
chce_shrunk_shell_244|77|86|243|247|245
chce_shrunk_shell_245|86|77|247|243|244
chce_shrunk_shell_246|78|86|213|268|247
chce_shrunk_shell_247|86|78|268|213|246
chce_shrunk_shell_248|79|87|217|267|249
chce_shrunk_shell_249|87|79|267|217|248
chce_shrunk_shell_250|79|88|248|255|251
chce_shrunk_shell_251|88|79|255|248|250
chce_shrunk_shell_252|80|81|254|223|253
chce_shrunk_shell_253|81|80|223|254|252
chce_shrunk_shell_254|80|88|221|257|255
chce_shrunk_shell_255|88|80|257|221|254
chce_shrunk_shell_256|82|88|229|251|257
chce_shrunk_shell_257|88|82|251|229|256
chce_shrunk_shell_258|82|89|256|261|259
chce_shrunk_shell_259|89|82|261|256|258
chce_shrunk_shell_260|83|89|231|263|261
chce_shrunk_shell_261|89|83|263|231|260
chce_shrunk_shell_262|84|89|239|259|263
chce_shrunk_shell_263|89|84|259|239|262
chce_shrunk_shell_264|84|85|262|241|265
chce_shrunk_shell_265|85|84|241|262|264
chce_shrunk_shell_266|85|87|265|269|267
chce_shrunk_shell_267|87|85|269|265|266
chce_shrunk_shell_268|86|87|245|249|269
chce_shrunk_shell_269|87|86|249|245|268
chcf_shrunk_shell_0|0
chcf_shrunk_shell_1|2
chcf_shrunk_shell_2|4
chcf_shrunk_shell_3|6
chcf_shrunk_shell_4|10
chcf_shrunk_shell_5|16
chcf_shrunk_shell_6|18
chcf_shrunk_shell_7|22
chcf_shrunk_shell_8|26
chcf_shrunk_shell_9|30
chcf_shrunk_shell_10|36
chcf_shrunk_shell_11|42
chcf_shrunk_shell_12|46
chcf_shrunk_shell_13|50
chcf_shrunk_shell_14|54
chcf_shrunk_shell_15|60
chcf_shrunk_shell_16|66
chcf_shrunk_shell_17|68
chcf_shrunk_shell_18|76
chcf_shrunk_shell_19|82
chcf_shrunk_shell_20|86
chcf_shrunk_shell_21|92
chcf_shrunk_shell_22|98
chcf_shrunk_shell_23|106
chcf_shrunk_shell_24|110
chcf_shrunk_shell_25|118
chcf_shrunk_shell_26|122
chcf_shrunk_shell_27|126
chcf_shrunk_shell_28|134
chcf_shrunk_shell_29|144
chcf_shrunk_shell_30|150
chcf_shrunk_shell_31|154
chcf_shrunk_shell_32|156
chcf_shrunk_shell_33|166
chcf_shrunk_shell_34|170
chcf_shrunk_shell_35|176
chcf_shrunk_shell_36|182
chcf_shrunk_shell_37|190
chcf_shrunk_shell_38|196
chcf_shrunk_shell_39|204
chcf_shrunk_shell_40|206
chcf_shrunk_shell_41|214
chcf_shrunk_shell_42|220
chcf_shrunk_shell_43|226
chcf_shrunk_shell_44|230
chcf_shrunk_shell_45|240
chcf_shrunk_shell_46|248
";

    /// Every `chc_*` case in [`BULLET_REFERENCE`], field for field.
    ///
    /// Vertices, edges and faces are compared in *emission order*, not as sets:
    /// `btConvexHullShape`'s `maxDot` breaks ties toward the lowest index, so a
    /// permutation of the right vertices is a different shape downstream.
    /// Coordinates go through [`diff`], which compares `f32` bit patterns, so a
    /// single rounding difference in the grid mapping fails a row rather than
    /// vanishing into a tolerance.
    #[test]
    fn bullet_reference_convex_hull() {
        let mut bad: Vec<String> = Vec::new();
        let mut covered: Vec<String> = Vec::new();

        for line in BULLET_REFERENCE.lines() {
            let header = line.split('|').next().unwrap();
            let Some(name) = header.strip_prefix("chc_") else {
                continue;
            };
            covered.push(header.to_string());

            let f = row(BULLET_REFERENCE, header, 8);
            let count: usize = f[1].parse().unwrap();
            let shrink: Scalar = f[2].parse().unwrap();
            let shrink_clamp: Scalar = f[3].parse().unwrap();
            let want_ret: Scalar = f[4].parse().unwrap();
            let want_vertices: usize = f[5].parse().unwrap();
            let want_edges: usize = f[6].parse().unwrap();
            let want_faces: usize = f[7].parse().unwrap();

            let mut points: Vec<Vec3> = Vec::with_capacity(count);
            for i in 0..count {
                let input_row = format!("chcin_{name}_{i}");
                covered.push(input_row.clone());
                let g = row(BULLET_REFERENCE, &input_row, 4);
                points.push(Vec3::new(
                    g[1].parse().unwrap(),
                    g[2].parse().unwrap(),
                    g[3].parse().unwrap(),
                ));
            }

            let mut hull = ConvexHullComputer::new();
            let ret = hull.compute(&points, shrink, shrink_clamp);
            diff(&mut bad, name, "ret", ret, want_ret);
            if (hull.vertices.len() != want_vertices)
                || (hull.edges.len() != want_edges)
                || (hull.faces.len() != want_faces)
            {
                bad.push(format!(
                    "{name}: port {}/{}/{} vertices/edges/faces, bullet \
                     {want_vertices}/{want_edges}/{want_faces}",
                    hull.vertices.len(),
                    hull.edges.len(),
                    hull.faces.len()
                ));
            }
            if hull.vertices.len() != hull.original_vertex_index.len() {
                bad.push(format!(
                    "{name}: {} vertices but {} original indices",
                    hull.vertices.len(),
                    hull.original_vertex_index.len()
                ));
            }

            // The row names come from the *reference* counts, so the coverage
            // check below stays meaningful even when the port emitted a
            // different number of anything.
            for i in 0..want_vertices {
                let vertex_row = format!("chcv_{name}_{i}");
                covered.push(vertex_row.clone());
                let g = row(BULLET_REFERENCE, &vertex_row, 5);
                let Some(&got) = hull.vertices.get(i) else {
                    continue;
                };
                diff_vec3(
                    &mut bad,
                    &format!("{name}[{i}]"),
                    "vertex",
                    got,
                    Vec3::new(
                        g[1].parse().unwrap(),
                        g[2].parse().unwrap(),
                        g[3].parse().unwrap(),
                    ),
                );
                let want_index: i32 = g[4].parse().unwrap();
                if hull.original_vertex_index[i] != want_index {
                    bad.push(format!(
                        "{name}[{i}].origIndex: port {}, bullet {want_index}",
                        hull.original_vertex_index[i]
                    ));
                }
            }

            for i in 0..want_edges {
                let edge_row = format!("chce_{name}_{i}");
                covered.push(edge_row.clone());
                let g = row(BULLET_REFERENCE, &edge_row, 6);
                if i >= hull.edges.len() {
                    continue;
                }
                let got: [i64; 5] = [
                    i64::from(hull.edge_source_vertex(i)),
                    i64::from(hull.edges[i].target_vertex()),
                    hull.next_edge_of_vertex(i) as i64,
                    hull.next_edge_of_face(i) as i64,
                    hull.reverse_edge(i) as i64,
                ];
                for (k, field) in ["source", "target", "nextOfVertex", "nextOfFace", "reverse"]
                    .into_iter()
                    .enumerate()
                {
                    let want: i64 = g[k + 1].parse().unwrap();
                    if got[k] != want {
                        bad.push(format!(
                            "{name}.edge[{i}].{field}: port {}, bullet {want}",
                            got[k]
                        ));
                    }
                }
            }

            for i in 0..want_faces {
                let face_row = format!("chcf_{name}_{i}");
                covered.push(face_row.clone());
                let g = row(BULLET_REFERENCE, &face_row, 2);
                let Some(&got) = hull.faces.get(i) else {
                    continue;
                };
                let want: i32 = g[1].parse().unwrap();
                if got != want {
                    bad.push(format!("{name}.face[{i}]: port {got}, bullet {want}"));
                }
            }
        }

        assert!(bad.is_empty(), "{}", bad.join("\n"));

        let mut want: Vec<String> = BULLET_REFERENCE
            .lines()
            .filter_map(|l| l.split('|').next())
            .map(str::to_string)
            .collect();
        want.sort();
        covered.sort();
        assert_eq!(
            covered, want,
            "the rows checked and BULLET_REFERENCE disagree on which rows exist"
        );
    }

    /// `compute` with `count <= 0` (`btConvexHullComputer.cpp:2655-2661`).
    ///
    /// Not a probe row because there is nothing for the C++ to print: the
    /// claim is about what the early return does *not* touch.
    /// `original_vertex_index` is left as it was, so a caller that indexes it
    /// against the now-empty `vertices` reads stale data -- upstream's
    /// behaviour, reproduced rather than tidied up.
    #[test]
    fn empty_input_clears_all_but_original_vertex_index() {
        let mut hull = ConvexHullComputer::new();
        let cube: Vec<Vec3> = (0..8)
            .map(|i| {
                Vec3::new(
                    if i & 1 == 0 { -1.0 } else { 1.0 },
                    if i & 2 == 0 { -1.0 } else { 1.0 },
                    if i & 4 == 0 { -1.0 } else { 1.0 },
                )
            })
            .collect();
        assert_eq!(hull.compute(&cube, 0.0, 0.0), 0.0);
        assert_eq!(hull.original_vertex_index.len(), 8);

        assert_eq!(hull.compute(&[], 0.0, 0.0), 0.0);
        assert!(hull.vertices.is_empty());
        assert!(hull.edges.is_empty());
        assert!(hull.faces.is_empty());
        assert_eq!(hull.original_vertex_index.len(), 8);
    }
}
