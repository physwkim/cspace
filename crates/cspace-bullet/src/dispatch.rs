// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp

//! `btDefaultCollisionConfiguration`'s two create-func tables -- which
//! algorithm a pair of proxy types resolves to.
//!
//! # One function, one documented divergence
//!
//! `getClosestPointsAlgorithmCreateFunc` and
//! `getCollisionAlgorithmCreateFunc` are two near-identical `if` chains
//! (`btDefaultCollisionConfiguration.cpp:193-266` and `:267-343`). Across the
//! whole enum they differ in exactly one entry: the contact-point table sends
//! box-vs-box to `btBoxBoxCollisionAlgorithm`, and the closest-points table
//! has no such row, so box-vs-box falls through to convex-convex. Writing
//! them as one function with one `table ==` guard keeps that the single
//! difference rather than something a reader has to diff two copies to find.
//!
//! Which table applies is decided per call site by
//! `m_closestPointDistanceThreshold`, and on the continuous path that
//! threshold is always zero -- see [`crate::convex_convex`]. So the top-level
//! query uses [`DispatchTable::ClosestPoints`] (MoveIt asks for it by name,
//! `bullet_utils.cpp:523`) and every compound child below it uses
//! [`DispatchTable::ContactPoints`].
//!
//! # Why the unported rows are named rather than folded away
//!
//! Only four of the thirteen create-funcs are reachable on the continuous
//! path, and it is tempting to write the dispatch as a match over this
//! crate's own shape variants. That would silently reroute the two rows the
//! variants cannot see -- sphere-vs-sphere and box-vs-box both come back as
//! "two convex shapes" -- into convex-convex, which is a different answer
//! with a different contact normal. So [`Algorithm`] names every entry the
//! tables can return, [`Algorithm::is_ported`] says which four this crate
//! implements, and a caller that reaches one of the others gets told rather
//! than served a plausible wrong result.

use crate::broadphase_proxy::BroadphaseNativeType;

/// Which of `btDefaultCollisionConfiguration`'s two tables to read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DispatchTable {
    /// `BT_CLOSEST_POINT_ALGORITHMS` --
    /// `getClosestPointsAlgorithmCreateFunc`.
    ClosestPoints,
    /// `BT_CONTACT_POINT_ALGORITHMS` -- `getCollisionAlgorithmCreateFunc`.
    ContactPoints,
}

/// The create-func a pair of proxy types resolves to.
///
/// Named after `btDefaultCollisionConfiguration`'s member rather than after
/// the algorithm class, because two members can build the same class with the
/// arguments swapped and it is the member the tables select.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Algorithm {
    /// `m_sphereSphereCF`.
    SphereSphere,
    /// `m_sphereTriangleCF`.
    SphereTriangle,
    /// `m_triangleSphereCF`.
    TriangleSphere,
    /// `m_boxBoxCF` -- contact-point table only.
    BoxBox,
    /// `m_convexPlaneCF`.
    ConvexPlane,
    /// `m_planeConvexCF`.
    PlaneConvex,
    /// `m_convexConvexCreateFunc`.
    ConvexConvex,
    /// `m_convexConcaveCreateFunc`.
    ConvexConcave,
    /// `m_swappedConvexConcaveCreateFunc`.
    SwappedConvexConcave,
    /// `m_compoundCompoundCreateFunc`.
    CompoundCompound,
    /// `m_compoundCreateFunc` -- the compound is body 0.
    Compound,
    /// `m_swappedCompoundCreateFunc` -- the compound is body 1.
    SwappedCompound,
    /// `m_emptyCreateFunc` -- "failed to find an algorithm".
    Empty,
}

impl Algorithm {
    /// Whether this crate carries the algorithm.
    ///
    /// The four that are true here are the ones the continuous path can
    /// reach: every pair it dispatches has a `CastHullShape`
    /// (`CUSTOM_CONVEX_SHAPE_TYPE`) or a compound on at least one side, so
    /// the sphere-sphere, box-box and triangle rows -- which need *both*
    /// sides to be that one type -- cannot come up, and MoveIt builds no
    /// plane and no concave shape at all.
    ///
    /// [`Algorithm::Empty`] is deliberately not "ported": upstream's empty
    /// algorithm is what a pair with no entry falls to, and reporting it as
    /// handled would turn a missing row into a silent no-collision.
    #[must_use]
    pub fn is_ported(self) -> bool {
        matches!(
            self,
            Self::ConvexConvex | Self::CompoundCompound | Self::Compound | Self::SwappedCompound
        )
    }
}

/// `getClosestPointsAlgorithmCreateFunc` / `getCollisionAlgorithmCreateFunc`.
///
/// `USE_BUGGY_SPHERE_BOX_ALGORITHM` is undefined in both tables, so the two
/// sphere-box rows it guards are not compiled and are not ported.
#[must_use]
pub fn find_algorithm(
    table: DispatchTable,
    proxy_type0: BroadphaseNativeType,
    proxy_type1: BroadphaseNativeType,
) -> Algorithm {
    use BroadphaseNativeType as T;

    if proxy_type0 == T::SPHERE_SHAPE && proxy_type1 == T::SPHERE_SHAPE {
        return Algorithm::SphereSphere;
    }

    if proxy_type0 == T::SPHERE_SHAPE && proxy_type1 == T::TRIANGLE_SHAPE {
        return Algorithm::SphereTriangle;
    }

    if proxy_type0 == T::TRIANGLE_SHAPE && proxy_type1 == T::SPHERE_SHAPE {
        return Algorithm::TriangleSphere;
    }

    // The one row the two tables disagree on.
    if table == DispatchTable::ContactPoints
        && proxy_type0 == T::BOX_SHAPE
        && proxy_type1 == T::BOX_SHAPE
    {
        return Algorithm::BoxBox;
    }

    if proxy_type0.is_convex() && proxy_type1 == T::STATIC_PLANE {
        return Algorithm::ConvexPlane;
    }

    if proxy_type1.is_convex() && proxy_type0 == T::STATIC_PLANE {
        return Algorithm::PlaneConvex;
    }

    if proxy_type0.is_convex() && proxy_type1.is_convex() {
        return Algorithm::ConvexConvex;
    }

    if proxy_type0.is_convex() && proxy_type1.is_concave() {
        return Algorithm::ConvexConcave;
    }

    if proxy_type1.is_convex() && proxy_type0.is_concave() {
        return Algorithm::SwappedConvexConcave;
    }

    if proxy_type0.is_compound() && proxy_type1.is_compound() {
        return Algorithm::CompoundCompound;
    }

    if proxy_type0.is_compound() {
        return Algorithm::Compound;
    }

    if proxy_type1.is_compound() {
        return Algorithm::SwappedCompound;
    }

    Algorithm::Empty
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The `dispatch_*` rows of `tools/bullet-epa-reference/build.sh`'s
    /// stdout: both tables, as a matrix over the eleven proxy types below.
    ///
    /// The probe cannot print a create-func's name -- the members are
    /// `protected` -- so it fixes one pair per create-func by reading the
    /// table top-down and labels every other pair by pointer identity against
    /// those anchors. A pair matching no anchor would print `??`, which does
    /// not appear.
    ///
    /// Row key is `dispatch_<table>_<type0>`; the columns are
    /// [`TYPES`] in order.
    const BULLET_REFERENCE: &str = "\
dispatch_closest_0|cx|cx|cx|cx|cx|cx|cx|cv|cp|cv|xk
dispatch_closest_1|cx|cx|cx|ts|cx|cx|cx|cv|cp|cv|xk
dispatch_closest_4|cx|cx|cx|cx|cx|cx|cx|cv|cp|cv|xk
dispatch_closest_8|cx|st|cx|ss|cx|cx|cx|cv|cp|cv|xk
dispatch_closest_11|cx|cx|cx|cx|cx|cx|cx|cv|cp|cv|xk
dispatch_closest_13|cx|cx|cx|cx|cx|cx|cx|cv|cp|cv|xk
dispatch_closest_19|cx|cx|cx|cx|cx|cx|cx|cv|cp|cv|xk
dispatch_closest_21|vc|vc|vc|vc|vc|vc|vc|--|--|--|xk
dispatch_closest_28|pc|pc|pc|pc|pc|pc|pc|--|--|--|xk
dispatch_closest_27|vc|vc|vc|vc|vc|vc|vc|--|--|--|xk
dispatch_closest_31|kx|kx|kx|kx|kx|kx|kx|kx|kx|kx|kk
dispatch_contact_0|bb|cx|cx|cx|cx|cx|cx|cv|cp|cv|xk
dispatch_contact_1|cx|cx|cx|ts|cx|cx|cx|cv|cp|cv|xk
dispatch_contact_4|cx|cx|cx|cx|cx|cx|cx|cv|cp|cv|xk
dispatch_contact_8|cx|st|cx|ss|cx|cx|cx|cv|cp|cv|xk
dispatch_contact_11|cx|cx|cx|cx|cx|cx|cx|cv|cp|cv|xk
dispatch_contact_13|cx|cx|cx|cx|cx|cx|cx|cv|cp|cv|xk
dispatch_contact_19|cx|cx|cx|cx|cx|cx|cx|cv|cp|cv|xk
dispatch_contact_21|vc|vc|vc|vc|vc|vc|vc|--|--|--|xk
dispatch_contact_28|pc|pc|pc|pc|pc|pc|pc|--|--|--|xk
dispatch_contact_27|vc|vc|vc|vc|vc|vc|vc|--|--|--|xk
dispatch_contact_31|kx|kx|kx|kx|kx|kx|kx|kx|kx|kx|kk
";

    /// `probe.cpp`'s `DISPATCH_TYPES`, in its order -- which is the column
    /// order of every row above.
    ///
    /// Chosen to cover both sides of all four range markers: `BOX`/`TRIANGLE`
    /// polyhedral, `CONVEX_HULL` the last polyhedral one, `SPHERE`/`CONE`/
    /// `CYLINDER` implicit, `CUSTOM_CONVEX` the last convex entry,
    /// `TRIANGLE_MESH` the first concave, `STATIC_PLANE` and `EMPTY` concave,
    /// and `COMPOUND` past the concave end.
    const TYPES: [BroadphaseNativeType; 11] = [
        BroadphaseNativeType::BOX_SHAPE,
        BroadphaseNativeType::TRIANGLE_SHAPE,
        BroadphaseNativeType::CONVEX_HULL_SHAPE,
        BroadphaseNativeType::SPHERE_SHAPE,
        BroadphaseNativeType::CONE_SHAPE,
        BroadphaseNativeType::CYLINDER_SHAPE,
        BroadphaseNativeType::CUSTOM_CONVEX_SHAPE,
        BroadphaseNativeType(21),
        BroadphaseNativeType::STATIC_PLANE,
        BroadphaseNativeType(27),
        BroadphaseNativeType::COMPOUND_SHAPE,
    ];

    /// The probe's two-letter code for each create-func.
    fn code(a: Algorithm) -> &'static str {
        match a {
            Algorithm::SphereSphere => "ss",
            Algorithm::SphereTriangle => "st",
            Algorithm::TriangleSphere => "ts",
            Algorithm::BoxBox => "bb",
            Algorithm::ConvexPlane => "cp",
            Algorithm::PlaneConvex => "pc",
            Algorithm::ConvexConvex => "cx",
            Algorithm::ConvexConcave => "cv",
            Algorithm::SwappedConvexConcave => "vc",
            Algorithm::CompoundCompound => "kk",
            Algorithm::Compound => "kx",
            Algorithm::SwappedCompound => "xk",
            Algorithm::Empty => "--",
        }
    }

    #[test]
    fn bullet_reference_dispatch_tables() {
        let mut bad = Vec::new();
        let mut cells = 0;

        for (table, tag) in [
            (DispatchTable::ClosestPoints, "closest"),
            (DispatchTable::ContactPoints, "contact"),
        ] {
            for t0 in TYPES {
                let name = format!("dispatch_{tag}_{}", t0.0);
                let line = BULLET_REFERENCE
                    .lines()
                    .find(|l| l.split('|').next() == Some(name.as_str()))
                    .unwrap_or_else(|| panic!("{name}: no such row in BULLET_REFERENCE"));
                let want: Vec<&str> = line.split('|').skip(1).collect();
                assert_eq!(
                    want.len(),
                    TYPES.len(),
                    "{name}: {} columns for {} types",
                    want.len(),
                    TYPES.len()
                );

                for (t1, want) in TYPES.into_iter().zip(want) {
                    cells += 1;
                    assert_ne!(want, "??", "{name} vs {}: probe could not label it", t1.0);
                    let got = code(find_algorithm(table, t0, t1));
                    if got != want {
                        bad.push(format!(
                            "{tag} {} vs {}: port {got}, bullet {want}",
                            t0.0, t1.0
                        ));
                    }
                }
            }
        }

        assert_eq!(cells, 2 * TYPES.len() * TYPES.len());
        assert!(
            bad.is_empty(),
            "{} of {cells} cells deviate:\n{}",
            bad.len(),
            bad.join("\n")
        );
    }

    /// The tables disagree on box-vs-box and on nothing else in this type set
    /// -- read off the rows rather than asserted about the code, so it stays
    /// a statement about Bullet.
    #[test]
    fn the_two_tables_differ_only_on_box_box() {
        let mut differ = Vec::new();
        for t0 in TYPES {
            for t1 in TYPES {
                let closest = find_algorithm(DispatchTable::ClosestPoints, t0, t1);
                let contact = find_algorithm(DispatchTable::ContactPoints, t0, t1);
                if closest != contact {
                    differ.push((t0.0, t1.0, closest, contact));
                }
            }
        }
        assert_eq!(
            differ,
            vec![(0, 0, Algorithm::ConvexConvex, Algorithm::BoxBox)],
            "unexpected divergence between the two tables"
        );
    }

    /// Every pair the continuous path can dispatch resolves to one of the
    /// four algorithms this crate carries.
    ///
    /// The precondition is that at least one side is a `CastHullShape`
    /// (`CUSTOM_CONVEX_SHAPE_TYPE`) or a compound, which
    /// `makeCastCollisionObject` guarantees: it replaces every convex child
    /// with a `CastHullShape` and every compound child with a compound of
    /// them (`bullet_utils.cpp:315-361`), and MoveIt builds no plane and no
    /// concave shape.
    #[test]
    fn every_cast_side_pair_is_ported() {
        let cast_side = [
            BroadphaseNativeType::CUSTOM_CONVEX_SHAPE,
            BroadphaseNativeType::COMPOUND_SHAPE,
        ];
        let other_side = [
            BroadphaseNativeType::BOX_SHAPE,
            BroadphaseNativeType::CONVEX_HULL_SHAPE,
            BroadphaseNativeType::SPHERE_SHAPE,
            BroadphaseNativeType::CONE_SHAPE,
            BroadphaseNativeType::CYLINDER_SHAPE,
            BroadphaseNativeType::CUSTOM_CONVEX_SHAPE,
            BroadphaseNativeType::COMPOUND_SHAPE,
        ];

        for table in [DispatchTable::ClosestPoints, DispatchTable::ContactPoints] {
            for cast in cast_side {
                for other in other_side {
                    for (a, b) in [(cast, other), (other, cast)] {
                        let algorithm = find_algorithm(table, a, b);
                        assert!(
                            algorithm.is_ported(),
                            "{table:?} {} vs {}: {algorithm:?} is not ported",
                            a.0,
                            b.0
                        );
                    }
                }
            }
        }
    }

    /// Three of the chain's arms can be reordered without changing any
    /// answer, and this is why: at the point each is reached its guard is
    /// disjoint from the guard of the arm it would swap with.
    ///
    /// The arms are convex-convex against the two plane rows,
    /// compound-compound against the two concave rows, and compound against
    /// swapped-compound. Reordering any of those pairs leaves every cell of
    /// [`BULLET_REFERENCE`] unchanged, which is a fact about the guards, not
    /// about the fixture -- so it is checked as one.
    ///
    /// [`TYPES`] carries a representative of every class the guards can tell
    /// apart -- `BOX`, `TRIANGLE`, `SPHERE` and `STATIC_PLANE` by identity,
    /// plus a plain convex, two plain concaves and `COMPOUND` -- so a pair
    /// outside it cannot make two of these guards co-hold either.
    #[test]
    fn the_reorderable_arms_have_disjoint_guards() {
        for t0 in TYPES {
            for t1 in TYPES {
                let convex_convex = t0.is_convex() && t1.is_convex();
                let convex_plane = t0.is_convex() && t1 == BroadphaseNativeType::STATIC_PLANE;
                let plane_convex = t1.is_convex() && t0 == BroadphaseNativeType::STATIC_PLANE;
                let compound_compound = t0.is_compound() && t1.is_compound();
                let convex_concave = t0.is_convex() && t1.is_concave();
                let swapped_convex_concave = t1.is_convex() && t0.is_concave();

                assert!(
                    !(convex_convex && (convex_plane || plane_convex)),
                    "{} vs {}: convex-convex co-holds with a plane row",
                    t0.0,
                    t1.0
                );
                assert!(
                    !(compound_compound && (convex_concave || swapped_convex_concave)),
                    "{} vs {}: compound-compound co-holds with a concave row",
                    t0.0,
                    t1.0
                );
                // The compound/swapped-compound pair is the one case whose
                // guards *do* co-hold -- when both sides are compound. That
                // pair never reaches them, because the compound-compound arm
                // above returns first, so the fact to check is on the
                // answer rather than on the guards.
                for table in [DispatchTable::ClosestPoints, DispatchTable::ContactPoints] {
                    if matches!(
                        find_algorithm(table, t0, t1),
                        Algorithm::Compound | Algorithm::SwappedCompound
                    ) {
                        assert!(
                            t0.is_compound() != t1.is_compound(),
                            "{table:?} {} vs {}: a one-sided compound answer with {} compound sides",
                            t0.0,
                            t1.0,
                            usize::from(t0.is_compound()) + usize::from(t1.is_compound())
                        );
                    }
                }
            }
        }
    }

    /// Two spheres are the pair the "just match on the shape variants"
    /// shortcut would get wrong, so it is worth one case of its own: this
    /// crate defines `SphereShape`, both tables send the pair to
    /// `btSphereSphereCollisionAlgorithm`, and nothing here implements it.
    #[test]
    fn sphere_sphere_is_named_and_unported() {
        for table in [DispatchTable::ClosestPoints, DispatchTable::ContactPoints] {
            let algorithm = find_algorithm(
                table,
                BroadphaseNativeType::SPHERE_SHAPE,
                BroadphaseNativeType::SPHERE_SHAPE,
            );
            assert_eq!(algorithm, Algorithm::SphereSphere);
            assert!(!algorithm.is_ported());
        }
    }
}
