// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Mesh's own per-paired-kind exact-tangency rule -- MEASURED, kept
//! deliberately separate from `fcl_tangency_table::SPECIALISED`.
//!
//! `SPECIALISED` is GENERATED: `tools/ci/verify-fcl-tangency-dispatch.sh
//! --emit` parses the pinned oracle image's own
//! `FCL_GJK_LIBCCD_SHAPE_INTERSECT`/`FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT`
//! macro registrations straight out of `gjk_solver_libccd-inl.h`, so a fresh
//! image re-derives the same table or the gate reddens. There is no such
//! macro for `Mesh` to parse -- fcl maps it to a `BVHModel` traversal, whose
//! leaf test falls to `shapeTriangleIntersect`'s own closed-form
//! specialisations (`Sphere`/`Halfspace`/`Plane` only) or generic libccd MPR
//! (`exact_tangency_is_decided_per_shape_pair.rs`'s module doc has the full
//! grep). A table with no registration to parse cannot be generated, so
//! unlike `SPECIALISED` this one is hand-written, and every entry below is
//! sourced from a probe run, not an oracle-image parse: the CSVs
//! `crates/moveit-collision/examples/mesh_orientation_probe.rs` (this port's
//! own `query::contact`, run with `cargo run -p moveit-collision --release
//! --example mesh_orientation_probe`) and `tools/fcl-mesh-orientation-probe`
//! (the same construction against `fcl::BVHModel<fcl::OBBRSSd>`, moveit2's
//! own mesh instantiation, run inside the pinned oracle image) both emit,
//! joined pose-by-pose over 497 systematic tilted orientations (7 axes, 71
//! angles at 5-degree resolution) x 5 other kinds x 2 argument orders.
//! Mixing the two provenances into one table would either break
//! `verify-fcl-tangency-dispatch.sh`'s gate or make it vouch for cells it
//! never checked -- this module exists so that cannot happen.

/// The other shape kind paired against a mesh at an exact-tangency dispatch
/// decision -- the same five kinds `mesh_orientation_probe.rs`'s
/// `OTHER_KINDS` sweeps, in the same order [`MESH_TANGENCY`] is indexed by.
/// `Mesh` covers `mesh x mesh`: [`crate::parry::mesh_other_kind`] classifies
/// the non-self side even when both shapes are `TriMesh`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum MeshOtherKind {
    Box = 0,
    Sphere = 1,
    Cylinder = 2,
    Cone = 3,
    Mesh = 4,
}

/// What [`MESH_TANGENCY`] records about a `(mesh, other)` pair's exact
/// tangency behaviour, measured for every one of 497 tilted orientations x 2
/// argument orders (`tools/fcl-mesh-orientation-probe/README.md` has the
/// full confusion matrix this is transcribed from).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum MeshVerdict {
    /// fcl reports touching at every one of the 497 tilted orientations
    /// measured, stable across argument order every time (145/497 of them a
    /// real `query::contact` miss). Matches the closed-form
    /// `Sphere`-triangle specialisation (`gjk_solver_libccd-inl.h:403-419,
    /// 480-498`), whose boundary padding (`sphere_triangle-inl.h:146,156`)
    /// is orientation-independent by construction -- there is nothing
    /// pose-dependent left to chase, so this is the one verdict with an
    /// unambiguous target. It is the only variant [`Self::as_tangency_bool`]
    /// answers `Some(true)` for, but that alone does not close the 145 --
    /// measured on 10 sampled miss poses, `query::intersection_test`
    /// (`accumulate_collision`'s existing confirmation call for a `None`
    /// `query::contact`) answers `false` at all 10, the same near-degenerate
    /// rounding this table exists to route around, one geometric query
    /// deeper (`Ball`-vs-`Triangle`'s `PointQuery::project_local_point`, not
    /// the GJK `contact` path). A widened-prediction second
    /// `query::contact` call (`2.0 * TIE_ROUNDING_MARGIN * f64::EPSILON *
    /// tie_scale(..)`) finds `Some` at all 10 of the same
    /// poses instead, feeding a real `dist` into `touches_at_tie` the same
    /// way every non-mesh tie already does -- but wiring that in touches
    /// `accumulate_collision`'s branch body, outside this table's own
    /// confinement. This verdict records the *target*; closing the 145 is a
    /// separate, not-yet-made change -- `Some(true)` only means "this table
    /// knows the answer", not "the answer is already delivered".
    AlwaysTouching,
    /// fcl itself has no single answer at a majority of the 497 orientations
    /// measured (408/497, 82.1%, argument-order-unstable) -- generalises
    /// `evidence-retention-1e69c0a3-1`'s `c04d5640` cone-under-tilt
    /// measurement from sampled poses to the majority. No stable target
    /// exists for this port to converge to, the same shape of open tie
    /// `exact_tangency_is_decided_per_shape_pair.rs` already leaves
    /// `mesh x cone`'s untilted case in.
    NoStableTarget,
    /// A real, stable divergence is measured in *both* directions at once,
    /// so a single boolean cannot represent fcl's own answer across every
    /// orientation without introducing the opposite error -- the same risk
    /// this session's `cylinder x box` uniformisation measured concretely.
    /// Left unrescued until diagnosed to an orientation-dependent rule
    /// rather than a shape-pair-wide one; picking a value now to shrink a
    /// residual count is exactly what this variant refuses to do.
    Undiagnosed,
}

impl MeshVerdict {
    /// `crate::parry::fcl_tangency_verdict`'s return value for this pair:
    /// `Some(true)` only for [`Self::AlwaysTouching`], the one verdict this
    /// table has an unambiguous answer for. `None` for the other two, which
    /// leaves `touches_at_tie`/`accumulate_collision` exactly where they
    /// were before this table existed (`unwrap_or(true)` in the tie band,
    /// the rescue branch's `== Some(true)` gate staying closed). Note
    /// `Some(true)` alone changes no *observable* behaviour yet, for any
    /// pair -- see [`Self::AlwaysTouching`]'s own doc for why `Sphere` still
    /// needs a further change this table does not make.
    pub(crate) const fn as_tangency_bool(self) -> Option<bool> {
        match self {
            Self::AlwaysTouching => Some(true),
            Self::NoStableTarget | Self::Undiagnosed => None,
        }
    }
}

/// Measured 2026-08-08 (`libfcl-dev 0.7.0-3build2` in the pinned oracle
/// image, `moveit-rs/oracle:ccc22ff0287a603f`; both probes' READMEs/module
/// docs have the build recipe and the raw counts each row below cites), rows
/// in [`MeshOtherKind`] order:
///
/// - `Box`: 481/497 agree, but a real residual splits both directions (3
///   `fcl=true,port=false`, 8 the other way) plus 5 fcl-unstable poses --
///   undiagnosed, not fixed blind.
/// - `Sphere`: [`MeshVerdict::AlwaysTouching`], see that variant's own doc.
/// - `Cylinder`: 460/497 agree; residual is one-directional (0 miss, 11
///   over-report) plus 13 fcl-unstable and 13 port-side-unstable poses --
///   undiagnosed, not fixed blind.
/// - `Cone`: [`MeshVerdict::NoStableTarget`], see that variant's own doc.
/// - `Mesh`: the most mixed cell -- 265/497 agree, 94/497 fcl-unstable, a
///   smaller stable residual both directions (21 miss, 8 over-report), and
///   151/497 poses where the port's own answer splits by role, of which 42
///   coincide with fcl's own instability (this file's own git history has
///   the correction: an earlier reading called the port's role-split
///   independent of fcl, which holds for 109 of the 151 but not the other
///   42). Undiagnosed, not fixed blind.
pub(crate) const MESH_TANGENCY: [MeshVerdict; 5] = [
    MeshVerdict::Undiagnosed,    // Box
    MeshVerdict::AlwaysTouching, // Sphere
    MeshVerdict::Undiagnosed,    // Cylinder
    MeshVerdict::NoStableTarget, // Cone
    MeshVerdict::Undiagnosed,    // Mesh
];
