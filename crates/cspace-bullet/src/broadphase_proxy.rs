// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.h

//! `BroadphaseNativeTypes` and the ordering predicates built on it -- the
//! value `btCollisionDispatcher::findAlgorithm` switches on.
//!
//! # Why a newtype and not an enum
//!
//! Upstream this is one unnumbered C enum whose entries are grouped by range,
//! with four marker entries (`IMPLICIT_CONVEX_SHAPES_START_HERE`,
//! `CONCAVE_SHAPES_START_HERE`, `CONCAVE_SHAPES_END_HERE` and the implicit
//! zero start) that no shape ever reports. `isConvex`, `isConcave` and
//! `isPolyhedral` are `<`/`>` comparisons against those markers, so the
//! *numbering* is the contract and the names are a convenience over it. A
//! Rust `enum` would either have to spell all 37 entries -- 29 of which no
//! shape in this crate can produce -- or drop the ones between the ones it
//! keeps and silently renumber the comparisons.
//!
//! So the type is a newtype over the `i32` the C++ passes around, with one
//! associated constant per entry the port needs and per marker the predicates
//! read. `tools/bullet-epa-reference/probe.cpp` prints every one of those
//! values out of the real enum, and this module's tests assert against that
//! output -- the numbering is measured, not transcribed.
//!
//! # What is deliberately absent
//!
//! `isInfinite`, `isConvex2d`, `isSoftBody` and `isNonMoving` are not ported.
//! The first three are read by `btCollisionWorld`'s ray casts and by the soft
//! body pipeline, neither of which exists here; `isNonMoving` is a
//! broadphase-side filter. None is consulted by either create-func table.

/// `BroadphaseNativeTypes` (`btBroadphaseProxy.h:27-80`).
///
/// The wrapped value is the enum's own, so the predicates below are the same
/// integer comparisons upstream makes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BroadphaseNativeType(pub i32);

impl BroadphaseNativeType {
    /// `BOX_SHAPE_PROXYTYPE`.
    pub const BOX_SHAPE: Self = Self(0);
    /// `TRIANGLE_SHAPE_PROXYTYPE`.
    pub const TRIANGLE_SHAPE: Self = Self(1);
    /// `CONVEX_HULL_SHAPE_PROXYTYPE`.
    pub const CONVEX_HULL_SHAPE: Self = Self(4);
    /// `IMPLICIT_CONVEX_SHAPES_START_HERE` -- a marker, never reported.
    pub const IMPLICIT_CONVEX_SHAPES_START_HERE: Self = Self(7);
    /// `SPHERE_SHAPE_PROXYTYPE`.
    pub const SPHERE_SHAPE: Self = Self(8);
    /// `CONE_SHAPE_PROXYTYPE`.
    pub const CONE_SHAPE: Self = Self(11);
    /// `CYLINDER_SHAPE_PROXYTYPE`.
    pub const CYLINDER_SHAPE: Self = Self(13);
    /// `CUSTOM_CONVEX_SHAPE_TYPE` -- what MoveIt's `CastHullShape` reports
    /// (`bullet_utils.hpp:251`), and the last convex entry.
    pub const CUSTOM_CONVEX_SHAPE: Self = Self(19);
    /// `CONCAVE_SHAPES_START_HERE` -- a marker, never reported.
    pub const CONCAVE_SHAPES_START_HERE: Self = Self(20);
    /// `STATIC_PLANE_PROXYTYPE`.
    pub const STATIC_PLANE: Self = Self(28);
    /// `CONCAVE_SHAPES_END_HERE` -- a marker, never reported.
    pub const CONCAVE_SHAPES_END_HERE: Self = Self(30);
    /// `COMPOUND_SHAPE_PROXYTYPE`.
    pub const COMPOUND_SHAPE: Self = Self(31);

    /// `btBroadphaseProxy::isPolyhedral` (`btBroadphaseProxy.h:130-133`).
    ///
    /// Read by `btConvexConvexAlgorithm::processCollision` to decide between
    /// the SAT/clipping branch and GJK; `crate::convex_convex` carries only
    /// the latter, and its module docs say why the former is unreachable on
    /// the continuous path.
    #[must_use]
    pub fn is_polyhedral(self) -> bool {
        self < Self::IMPLICIT_CONVEX_SHAPES_START_HERE
    }

    /// `btBroadphaseProxy::isConvex` (`btBroadphaseProxy.h:135-138`).
    #[must_use]
    pub fn is_convex(self) -> bool {
        self < Self::CONCAVE_SHAPES_START_HERE
    }

    /// `btBroadphaseProxy::isConcave` (`btBroadphaseProxy.h:145-149`).
    ///
    /// Strict on both ends, which is why the marker itself is neither convex
    /// nor concave.
    #[must_use]
    pub fn is_concave(self) -> bool {
        self > Self::CONCAVE_SHAPES_START_HERE && self < Self::CONCAVE_SHAPES_END_HERE
    }

    /// `btBroadphaseProxy::isCompound` (`btBroadphaseProxy.h:150-153`).
    #[must_use]
    pub fn is_compound(self) -> bool {
        self == Self::COMPOUND_SHAPE
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The `proxytype_*` rows of `tools/bullet-epa-reference/build.sh`'s
    /// stdout: every entry this module names a constant for, the four
    /// markers, and the entries either side of a marker so an off-by-one in
    /// the numbering cannot hide between two rows.
    ///
    /// Fields: `name|value|isConvex|isConcave|isCompound`.
    const BULLET_REFERENCE: &str = "\
proxytype_BOX_SHAPE|0|1|0|0
proxytype_TRIANGLE_SHAPE|1|1|0|0
proxytype_CONVEX_HULL_SHAPE|4|1|0|0
proxytype_CUSTOM_POLYHEDRAL_SHAPE|6|1|0|0
proxytype_IMPLICIT_CONVEX_SHAPES_START_HERE|7|1|0|0
proxytype_SPHERE_SHAPE|8|1|0|0
proxytype_CAPSULE_SHAPE|10|1|0|0
proxytype_CONE_SHAPE|11|1|0|0
proxytype_CYLINDER_SHAPE|13|1|0|0
proxytype_CONVEX_2D_SHAPE|18|1|0|0
proxytype_CUSTOM_CONVEX_SHAPE|19|1|0|0
proxytype_CONCAVE_SHAPES_START_HERE|20|0|0|0
proxytype_TRIANGLE_MESH_SHAPE|21|0|1|0
proxytype_EMPTY_SHAPE|27|0|1|0
proxytype_STATIC_PLANE|28|0|1|0
proxytype_CUSTOM_CONCAVE_SHAPE|29|0|1|0
proxytype_CONCAVE_SHAPES_END_HERE|30|0|0|0
proxytype_COMPOUND_SHAPE|31|0|0|1
proxytype_SOFTBODY_SHAPE|32|0|0|0
proxytype_INVALID_SHAPE|35|0|0|0
";

    /// The value each constant this module declares carries, against the enum.
    ///
    /// Only the named ones can be checked this way; the rows with no constant
    /// here are for `predicates_agree_on_every_emitted_value`, which reads
    /// every row.
    #[test]
    fn every_named_constant_has_bullets_value() {
        let named: [(&str, BroadphaseNativeType); 12] = [
            ("BOX_SHAPE", BroadphaseNativeType::BOX_SHAPE),
            ("TRIANGLE_SHAPE", BroadphaseNativeType::TRIANGLE_SHAPE),
            ("CONVEX_HULL_SHAPE", BroadphaseNativeType::CONVEX_HULL_SHAPE),
            (
                "IMPLICIT_CONVEX_SHAPES_START_HERE",
                BroadphaseNativeType::IMPLICIT_CONVEX_SHAPES_START_HERE,
            ),
            ("SPHERE_SHAPE", BroadphaseNativeType::SPHERE_SHAPE),
            ("CONE_SHAPE", BroadphaseNativeType::CONE_SHAPE),
            ("CYLINDER_SHAPE", BroadphaseNativeType::CYLINDER_SHAPE),
            (
                "CUSTOM_CONVEX_SHAPE",
                BroadphaseNativeType::CUSTOM_CONVEX_SHAPE,
            ),
            (
                "CONCAVE_SHAPES_START_HERE",
                BroadphaseNativeType::CONCAVE_SHAPES_START_HERE,
            ),
            ("STATIC_PLANE", BroadphaseNativeType::STATIC_PLANE),
            (
                "CONCAVE_SHAPES_END_HERE",
                BroadphaseNativeType::CONCAVE_SHAPES_END_HERE,
            ),
            ("COMPOUND_SHAPE", BroadphaseNativeType::COMPOUND_SHAPE),
        ];

        let mut bad = Vec::new();
        for (name, port) in named {
            let row = format!("proxytype_{name}|");
            let line = BULLET_REFERENCE
                .lines()
                .find(|l| l.starts_with(&row))
                .unwrap_or_else(|| panic!("{name}: no such row in BULLET_REFERENCE"));
            let want: i32 = line.split('|').nth(1).unwrap().parse().unwrap();
            if port.0 != want {
                bad.push(format!("{name}: port {}, bullet {want}", port.0));
            }
        }
        assert!(bad.is_empty(), "{}", bad.join("\n"));
    }

    /// The three predicates, on every value the probe emitted -- including the
    /// ones no constant here names, which is the point: they are where an
    /// off-by-one in a boundary would show.
    #[test]
    fn predicates_agree_on_every_emitted_value() {
        let mut bad = Vec::new();
        let mut rows = 0;

        for line in BULLET_REFERENCE.lines() {
            let f: Vec<&str> = line.split('|').collect();
            assert_eq!(f.len(), 5, "{line}: expected 5 fields");
            rows += 1;

            let name = f[0];
            let t = BroadphaseNativeType(f[1].parse().unwrap());
            for (what, port, want) in [
                ("isConvex", t.is_convex(), f[2] == "1"),
                ("isConcave", t.is_concave(), f[3] == "1"),
                ("isCompound", t.is_compound(), f[4] == "1"),
            ] {
                if port != want {
                    bad.push(format!("{name}.{what}: port {port}, bullet {want}"));
                }
            }
        }

        assert_eq!(rows, 20, "BULLET_REFERENCE lost or gained a row");
        assert!(
            bad.is_empty(),
            "{} deviations:\n{}",
            bad.len(),
            bad.join("\n")
        );
    }

    /// `isPolyhedral`, which the probe does not print because
    /// `btBroadphaseProxy` exposes it the same way and it is one comparison
    /// against a value the rows above already pin.
    ///
    /// The boundary is what matters: the marker is polyhedral and the first
    /// implicit shape after it is not.
    #[test]
    fn is_polyhedral_ends_at_the_implicit_marker() {
        assert!(BroadphaseNativeType::BOX_SHAPE.is_polyhedral());
        assert!(BroadphaseNativeType::CONVEX_HULL_SHAPE.is_polyhedral());
        assert!(!BroadphaseNativeType::IMPLICIT_CONVEX_SHAPES_START_HERE.is_polyhedral());
        assert!(!BroadphaseNativeType::SPHERE_SHAPE.is_polyhedral());
        assert!(!BroadphaseNativeType::CUSTOM_CONVEX_SHAPE.is_polyhedral());
    }
}
