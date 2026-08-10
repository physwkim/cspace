// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h
//   bullet3/src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp
//
// btVoronoiSimplexSolver.cpp carries a credit upstream is obliged to pass on,
// and so is this port:
//
//   Elsevier CDROM license agreements grants nonexclusive license to use the
//   software for any purpose, commercial or non-commercial as long as the
//   following credit is included identifying the original source of the
//   software:
//
//   Parts of the source are "from the book Real-Time Collision Detection by
//   Christer Ericson, published by Morgan Kaufmann Publishers,
//   (c) 2005 Elsevier Inc."

//! `btVoronoiSimplexSolver` -- the closest point from a 1-to-4-point simplex to
//! the origin, and the sub-simplex it lies on.
//!
//! This is the half of GJK that decides which vertices survive each iteration.
//! It is a faithful port, quirks included, because the quirks are load-bearing:
//!
//! - [`VoronoiSimplexSolver::point_outside_of_plane`] returns three values, not
//!   two. `-1` means "this tetrahedron is affinely degenerate", detected by a
//!   *fixed* threshold on the scalar triple product -- `1e-4` in single
//!   precision, unscaled by the simplex's size. A tetrahedron of millimetre
//!   edges is degenerate by that test whether or not it is flat.
//! - [`VoronoiSimplexSolver::in_simplex`] compares squared distances against
//!   `m_equalVertexThreshold`, whose default is `1e-4` -- also a squared
//!   distance, so the effective vertex-merging radius is 1e-2 metres.
//! - the barycentric weights are cached and reused to rebuild the witness
//!   points on both bodies, so an error in the sub-simplex classification shows
//!   up as a displaced contact point rather than as a wrong distance.
//!
//! # Not ported
//!
//! `btSimplexSolverInterface`, the abstract base. Bullet has exactly one
//! implementation of it and `btGjkPairDetector` holds a pointer to that base
//! only so the type can be swapped at construction; nothing in MoveIt's Bullet
//! integration swaps it.

use crate::linear_math::{BT_LARGE_FLOAT, Scalar, Vec3};

/// `VORONOI_SIMPLEX_MAX_VERTS` (`btVoronoiSimplexSolver.h:21`).
pub const VORONOI_SIMPLEX_MAX_VERTS: usize = 5;

/// `VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD` in single precision
/// (`btVoronoiSimplexSolver.h:29`).
///
/// Compared against a *squared* distance
/// ([`VoronoiSimplexSolver::in_simplex`]), so two support points within 1e-2
/// of each other count as the same vertex.
pub const VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD: Scalar = 0.0001;

/// `btUsageBitfield` (`btVoronoiSimplexSolver.h:32-54`) -- which of the up-to-
/// four simplex vertices the closest point actually uses.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UsageBitfield {
    /// `usedVertexA` -- simplex vertex 0 contributes to the closest point.
    pub used_vertex_a: bool,
    /// `usedVertexB` -- simplex vertex 1 contributes.
    pub used_vertex_b: bool,
    /// `usedVertexC` -- simplex vertex 2 contributes.
    pub used_vertex_c: bool,
    /// `usedVertexD` -- simplex vertex 3 contributes.
    pub used_vertex_d: bool,
}

impl UsageBitfield {
    /// `btUsageBitfield::reset`.
    pub const fn reset(&mut self) {
        *self = Self {
            used_vertex_a: false,
            used_vertex_b: false,
            used_vertex_c: false,
            used_vertex_d: false,
        };
    }
}

/// `btSubSimplexClosestResult` (`btVoronoiSimplexSolver.h:56-88`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SubSimplexClosestResult {
    /// `m_closestPointOnSimplex`. Note that `reset` does *not* clear it.
    pub closest_point_on_simplex: Vec3,
    /// `m_usedVertices` -- which simplex vertices the closest point uses.
    pub used_vertices: UsageBitfield,
    /// `m_barycentricCoords` -- the weights that rebuild the closest point,
    /// and with it the witness point on each body.
    pub barycentric_coords: [Scalar; 4],
    /// `m_degenerate` -- set when the tetrahedron test found an affinely
    /// dependent simplex, which is a different `false` from "the origin is
    /// inside".
    pub degenerate: bool,
}

impl Default for SubSimplexClosestResult {
    fn default() -> Self {
        Self {
            closest_point_on_simplex: Vec3::zero(),
            used_vertices: UsageBitfield::default(),
            barycentric_coords: [0.0; 4],
            degenerate: false,
        }
    }
}

impl SubSimplexClosestResult {
    /// `btSubSimplexClosestResult::reset` -- clears the flags and the weights
    /// but leaves `m_closestPointOnSimplex` alone, exactly as upstream does.
    pub const fn reset(&mut self) {
        self.degenerate = false;
        self.barycentric_coords = [0.0; 4];
        self.used_vertices.reset();
    }

    /// `btSubSimplexClosestResult::isValid`.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.barycentric_coords.iter().all(|&c| c >= 0.0)
    }

    /// `btSubSimplexClosestResult::setBarycentricCoordinates`, whose C++
    /// default arguments are zero.
    pub const fn set_barycentric_coordinates(
        &mut self,
        a: Scalar,
        b: Scalar,
        c: Scalar,
        d: Scalar,
    ) {
        self.barycentric_coords = [a, b, c, d];
    }
}

/// `btVoronoiSimplexSolver` (`btVoronoiSimplexSolver.h:92-171`).
#[derive(Clone, Debug, PartialEq)]
pub struct VoronoiSimplexSolver {
    num_vertices: usize,

    /// `m_simplexVectorW` -- the Minkowski-difference points.
    simplex_vector_w: [Vec3; VORONOI_SIMPLEX_MAX_VERTS],
    /// `m_simplexPointsP` -- the witness point on body A for each `W`.
    simplex_points_p: [Vec3; VORONOI_SIMPLEX_MAX_VERTS],
    /// `m_simplexPointsQ` -- the witness point on body B for each `W`.
    simplex_points_q: [Vec3; VORONOI_SIMPLEX_MAX_VERTS],

    cached_p1: Vec3,
    cached_p2: Vec3,
    cached_v: Vec3,
    last_w: Vec3,

    equal_vertex_threshold: Scalar,
    cached_valid_closest: bool,

    cached_bc: SubSimplexClosestResult,

    needs_update: bool,
}

impl Default for VoronoiSimplexSolver {
    fn default() -> Self {
        Self::new()
    }
}

impl VoronoiSimplexSolver {
    /// `btVoronoiSimplexSolver()` followed by `reset()`.
    ///
    /// Upstream's constructor sets `m_equalVertexThreshold` and nothing else
    /// (`btVoronoiSimplexSolver.h:130-133`); every other member is
    /// indeterminate until a caller invokes `reset()`, which
    /// `btGjkPairDetector::getClosestPoints` does before its first iteration.
    /// Rust has no such state, so this constructor is the composition of the
    /// two -- which is the only sequence upstream's own callers produce.
    #[must_use]
    pub fn new() -> Self {
        let mut solver = Self {
            num_vertices: 0,
            simplex_vector_w: [Vec3::zero(); VORONOI_SIMPLEX_MAX_VERTS],
            simplex_points_p: [Vec3::zero(); VORONOI_SIMPLEX_MAX_VERTS],
            simplex_points_q: [Vec3::zero(); VORONOI_SIMPLEX_MAX_VERTS],
            cached_p1: Vec3::zero(),
            cached_p2: Vec3::zero(),
            cached_v: Vec3::zero(),
            last_w: Vec3::zero(),
            equal_vertex_threshold: VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD,
            cached_valid_closest: false,
            cached_bc: SubSimplexClosestResult::default(),
            needs_update: true,
        };
        solver.reset();
        solver
    }

    /// `btVoronoiSimplexSolver::reset` (`btVoronoiSimplexSolver.cpp:59-66`).
    ///
    /// `m_lastW` is seeded at `BT_LARGE_FLOAT` in all three components so the
    /// first [`VoronoiSimplexSolver::in_simplex`] query cannot match it.
    pub const fn reset(&mut self) {
        self.cached_valid_closest = false;
        self.num_vertices = 0;
        self.needs_update = true;
        self.last_w = Vec3::new(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
        self.cached_bc.reset();
    }

    /// `btVoronoiSimplexSolver::addVertex` (`.cpp:69-79`).
    pub const fn add_vertex(&mut self, w: Vec3, p: Vec3, q: Vec3) {
        self.last_w = w;
        self.needs_update = true;

        self.simplex_vector_w[self.num_vertices] = w;
        self.simplex_points_p[self.num_vertices] = p;
        self.simplex_points_q[self.num_vertices] = q;

        self.num_vertices += 1;
    }

    /// `btVoronoiSimplexSolver::setEqualVertexThreshold`.
    pub const fn set_equal_vertex_threshold(&mut self, threshold: Scalar) {
        self.equal_vertex_threshold = threshold;
    }

    /// `btVoronoiSimplexSolver::getEqualVertexThreshold`.
    #[must_use]
    pub const fn equal_vertex_threshold(&self) -> Scalar {
        self.equal_vertex_threshold
    }

    /// `btVoronoiSimplexSolver::numVertices`.
    #[must_use]
    pub const fn num_vertices(&self) -> usize {
        self.num_vertices
    }

    /// `btVoronoiSimplexSolver::fullSimplex`.
    #[must_use]
    pub const fn full_simplex(&self) -> bool {
        self.num_vertices == 4
    }

    /// `btVoronoiSimplexSolver::emptySimplex`.
    #[must_use]
    pub const fn empty_simplex(&self) -> bool {
        self.num_vertices == 0
    }

    /// `btVoronoiSimplexSolver::maxVertex` -- the largest **squared** length in
    /// the simplex, despite the name (`.cpp:243-254`).
    #[must_use]
    pub fn max_vertex(&self) -> Scalar {
        let mut max_v = 0.0;
        for w in &self.simplex_vector_w[..self.num_vertices] {
            let cur_len2 = w.length2();
            if max_v < cur_len2 {
                max_v = cur_len2;
            }
        }
        max_v
    }

    /// `btVoronoiSimplexSolver::getSimplex` -- the live `(p, q, y)` triples.
    #[must_use]
    pub fn simplex(&self) -> (&[Vec3], &[Vec3], &[Vec3]) {
        (
            &self.simplex_points_p[..self.num_vertices],
            &self.simplex_points_q[..self.num_vertices],
            &self.simplex_vector_w[..self.num_vertices],
        )
    }

    /// `btVoronoiSimplexSolver::inSimplex` (`.cpp:269-294`).
    ///
    /// Two separate tests, and the second is the one that terminates GJK on a
    /// repeated support point: a `w` equal to `m_lastW` counts as present even
    /// after the reduction step removed it from the simplex. The first test is
    /// a squared-distance comparison against
    /// [`VoronoiSimplexSolver::equal_vertex_threshold`], not an equality.
    #[must_use]
    pub fn in_simplex(&self, w: Vec3) -> bool {
        let found = self.simplex_vector_w[..self.num_vertices]
            .iter()
            .any(|v| v.distance2(w) <= self.equal_vertex_threshold);

        // Checked after the loop, and unconditionally: upstream returns true
        // here even when the loop already found a match.
        if w == self.last_w {
            return true;
        }

        found
    }

    /// `btVoronoiSimplexSolver::backup_closest`.
    pub const fn backup_closest(&self) -> Vec3 {
        self.cached_v
    }

    /// `btVoronoiSimplexSolver::closest` -- the closest point to the origin,
    /// and whether the cached result is usable.
    pub fn closest(&mut self) -> (Vec3, bool) {
        let succes = self.update_closest_vector_and_points();
        (self.cached_v, succes)
    }

    /// `btVoronoiSimplexSolver::compute_points` -- the witness point on each
    /// body, rebuilt from the cached barycentric weights.
    pub fn compute_points(&mut self) -> (Vec3, Vec3) {
        self.update_closest_vector_and_points();
        (self.cached_p1, self.cached_p2)
    }

    /// `btVoronoiSimplexSolver::removeVertex` -- swap with the last and shrink
    /// (`.cpp:34-41`), which is why the simplex order is not the insertion
    /// order after any reduction.
    pub fn remove_vertex(&mut self, index: usize) {
        self.num_vertices -= 1;
        self.simplex_vector_w[index] = self.simplex_vector_w[self.num_vertices];
        self.simplex_points_p[index] = self.simplex_points_p[self.num_vertices];
        self.simplex_points_q[index] = self.simplex_points_q[self.num_vertices];
    }

    /// `btVoronoiSimplexSolver::reduceVertices` (`.cpp:43-56`) -- drop the
    /// unused vertices, highest index first so the lower ones stay addressable.
    pub fn reduce_vertices(&mut self, used_verts: UsageBitfield) {
        if self.num_vertices >= 4 && !used_verts.used_vertex_d {
            self.remove_vertex(3);
        }
        if self.num_vertices >= 3 && !used_verts.used_vertex_c {
            self.remove_vertex(2);
        }
        if self.num_vertices >= 2 && !used_verts.used_vertex_b {
            self.remove_vertex(1);
        }
        if self.num_vertices >= 1 && !used_verts.used_vertex_a {
            self.remove_vertex(0);
        }
    }

    /// `btVoronoiSimplexSolver::updateClosestVectorAndPoints` (`.cpp:81-233`).
    pub fn update_closest_vector_and_points(&mut self) -> bool {
        if !self.needs_update {
            return self.cached_valid_closest;
        }

        self.cached_bc.reset();
        self.needs_update = false;

        match self.num_vertices {
            0 => self.cached_valid_closest = false,
            1 => {
                self.cached_p1 = self.simplex_points_p[0];
                self.cached_p2 = self.simplex_points_q[0];
                self.cached_v = self.cached_p1 - self.cached_p2; // == m_simplexVectorW[0]
                self.cached_bc.reset();
                self.cached_bc
                    .set_barycentric_coordinates(1.0, 0.0, 0.0, 0.0);
                self.cached_valid_closest = self.cached_bc.is_valid();
            }
            2 => {
                // Closest point to the origin on the segment W0..W1.
                let from = self.simplex_vector_w[0];
                let to = self.simplex_vector_w[1];

                let p = Vec3::zero();
                let mut diff = p - from;
                let v = to - from;
                let mut t = v.dot(diff);

                if t > 0.0 {
                    let dot_vv = v.dot(v);
                    if t < dot_vv {
                        t /= dot_vv;
                        diff -= t * v;
                        self.cached_bc.used_vertices.used_vertex_a = true;
                        self.cached_bc.used_vertices.used_vertex_b = true;
                    } else {
                        t = 1.0;
                        diff -= v;
                        // Reduce to one point.
                        self.cached_bc.used_vertices.used_vertex_b = true;
                    }
                } else {
                    t = 0.0;
                    // Reduce to one point.
                    self.cached_bc.used_vertices.used_vertex_a = true;
                }
                self.cached_bc
                    .set_barycentric_coordinates(1.0 - t, t, 0.0, 0.0);

                self.cached_p1 = self.simplex_points_p[0]
                    + t * (self.simplex_points_p[1] - self.simplex_points_p[0]);
                self.cached_p2 = self.simplex_points_q[0]
                    + t * (self.simplex_points_q[1] - self.simplex_points_q[0]);
                self.cached_v = self.cached_p1 - self.cached_p2;

                self.reduce_vertices(self.cached_bc.used_vertices);

                self.cached_valid_closest = self.cached_bc.is_valid();
            }
            3 => {
                let p = Vec3::zero();

                let a = self.simplex_vector_w[0];
                let b = self.simplex_vector_w[1];
                let c = self.simplex_vector_w[2];

                let mut bc = self.cached_bc;
                Self::closest_pt_point_triangle(p, a, b, c, &mut bc);
                self.cached_bc = bc;

                let w = self.cached_bc.barycentric_coords;
                self.cached_p1 = self.simplex_points_p[0] * w[0]
                    + self.simplex_points_p[1] * w[1]
                    + self.simplex_points_p[2] * w[2];
                self.cached_p2 = self.simplex_points_q[0] * w[0]
                    + self.simplex_points_q[1] * w[1]
                    + self.simplex_points_q[2] * w[2];

                self.cached_v = self.cached_p1 - self.cached_p2;

                self.reduce_vertices(self.cached_bc.used_vertices);
                self.cached_valid_closest = self.cached_bc.is_valid();
            }
            4 => {
                let p = Vec3::zero();

                let a = self.simplex_vector_w[0];
                let b = self.simplex_vector_w[1];
                let c = self.simplex_vector_w[2];
                let d = self.simplex_vector_w[3];

                let mut bc = self.cached_bc;
                let has_separation = Self::closest_pt_point_tetrahedron(p, a, b, c, d, &mut bc);
                self.cached_bc = bc;

                if has_separation {
                    let w = self.cached_bc.barycentric_coords;
                    self.cached_p1 = self.simplex_points_p[0] * w[0]
                        + self.simplex_points_p[1] * w[1]
                        + self.simplex_points_p[2] * w[2]
                        + self.simplex_points_p[3] * w[3];
                    self.cached_p2 = self.simplex_points_q[0] * w[0]
                        + self.simplex_points_q[1] * w[1]
                        + self.simplex_points_q[2] * w[2]
                        + self.simplex_points_q[3] * w[3];

                    self.cached_v = self.cached_p1 - self.cached_p2;
                    self.reduce_vertices(self.cached_bc.used_vertices);

                    self.cached_valid_closest = self.cached_bc.is_valid();
                } else if self.cached_bc.degenerate {
                    self.cached_valid_closest = false;
                } else {
                    // Not degenerate and no separating sub-simplex: the origin
                    // is inside the tetrahedron. Upstream `break`s here, so the
                    // `isValid()` assignment below the branch is skipped and
                    // the witness points keep their previous values.
                    self.cached_valid_closest = true;
                    self.cached_v = Vec3::zero();
                }
            }
            _ => self.cached_valid_closest = false,
        }

        self.cached_valid_closest
    }

    /// `btVoronoiSimplexSolver::closestPtPointTriangle` (`.cpp:313-408`) --
    /// Ericson's seven-region test, verbatim including the comparison
    /// directions, which decide the tie cases.
    pub fn closest_pt_point_triangle(
        p: Vec3,
        a: Vec3,
        b: Vec3,
        c: Vec3,
        result: &mut SubSimplexClosestResult,
    ) -> bool {
        result.used_vertices.reset();

        // Vertex region outside A.
        let ab = b - a;
        let ac = c - a;
        let ap = p - a;
        let d1 = ab.dot(ap);
        let d2 = ac.dot(ap);
        if d1 <= 0.0 && d2 <= 0.0 {
            result.closest_point_on_simplex = a;
            result.used_vertices.used_vertex_a = true;
            result.set_barycentric_coordinates(1.0, 0.0, 0.0, 0.0);
            return true;
        }

        // Vertex region outside B.
        let bp = p - b;
        let d3 = ab.dot(bp);
        let d4 = ac.dot(bp);
        if d3 >= 0.0 && d4 <= d3 {
            result.closest_point_on_simplex = b;
            result.used_vertices.used_vertex_b = true;
            result.set_barycentric_coordinates(0.0, 1.0, 0.0, 0.0);
            return true;
        }

        // Edge region AB.
        let vc = d1 * d4 - d3 * d2;
        if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            let v = d1 / (d1 - d3);
            result.closest_point_on_simplex = a + v * ab;
            result.used_vertices.used_vertex_a = true;
            result.used_vertices.used_vertex_b = true;
            result.set_barycentric_coordinates(1.0 - v, v, 0.0, 0.0);
            return true;
        }

        // Vertex region outside C.
        let cp = p - c;
        let d5 = ab.dot(cp);
        let d6 = ac.dot(cp);
        if d6 >= 0.0 && d5 <= d6 {
            result.closest_point_on_simplex = c;
            result.used_vertices.used_vertex_c = true;
            result.set_barycentric_coordinates(0.0, 0.0, 1.0, 0.0);
            return true;
        }

        // Edge region AC.
        let vb = d5 * d2 - d1 * d6;
        if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            let w = d2 / (d2 - d6);
            result.closest_point_on_simplex = a + w * ac;
            result.used_vertices.used_vertex_a = true;
            result.used_vertices.used_vertex_c = true;
            result.set_barycentric_coordinates(1.0 - w, 0.0, w, 0.0);
            return true;
        }

        // Edge region BC.
        let va = d3 * d6 - d5 * d4;
        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

            result.closest_point_on_simplex = b + w * (c - b);
            result.used_vertices.used_vertex_b = true;
            result.used_vertices.used_vertex_c = true;
            result.set_barycentric_coordinates(0.0, 1.0 - w, w, 0.0);
            return true;
        }

        // Inside the face region.
        let denom = 1.0 / (va + vb + vc);
        let v = vb * denom;
        let w = vc * denom;

        result.closest_point_on_simplex = a + ab * v + ac * w;
        result.used_vertices.used_vertex_a = true;
        result.used_vertices.used_vertex_b = true;
        result.used_vertices.used_vertex_c = true;
        result.set_barycentric_coordinates(1.0 - v - w, v, w, 0.0);

        true
    }

    /// `btVoronoiSimplexSolver::pointOutsideOfPlane` (`.cpp:411-435`).
    ///
    /// Three-valued: `-1` for an affinely degenerate tetrahedron, else the
    /// `signp * signd < 0` test as `0`/`1`. The degeneracy threshold is a bare
    /// `1e-4` on the scalar triple product `[AD AB AC]`, with no normalization
    /// by the simplex's own scale (`CATCH_DEGENERATE_TETRAHEDRON`,
    /// single-precision branch), so whether a tetrahedron reads as degenerate
    /// depends on the units the geometry is expressed in.
    pub fn point_outside_of_plane(p: Vec3, a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> i32 {
        let normal = (b - a).cross(c - a);

        let signp = (p - a).dot(normal); // [AP AB AC]
        let signd = (d - a).dot(normal); // [AD AB AC]

        if signd * signd < (1e-4 * 1e-4) {
            return -1;
        }
        i32::from(signp * signd < 0.0)
    }

    /// `btVoronoiSimplexSolver::closestPtPointTetrahedron` (`.cpp:437-577`).
    ///
    /// Returns `false` in two distinct situations the caller must tell apart by
    /// reading `degenerate`: an affinely degenerate tetrahedron, and the origin
    /// being strictly inside all four half-spaces.
    pub fn closest_pt_point_tetrahedron(
        p: Vec3,
        a: Vec3,
        b: Vec3,
        c: Vec3,
        d: Vec3,
        final_result: &mut SubSimplexClosestResult,
    ) -> bool {
        let mut temp_result = SubSimplexClosestResult::default();

        // Start out assuming the point is inside all half-spaces, so closest to
        // itself.
        final_result.closest_point_on_simplex = p;
        final_result.used_vertices.reset();
        final_result.used_vertices.used_vertex_a = true;
        final_result.used_vertices.used_vertex_b = true;
        final_result.used_vertices.used_vertex_c = true;
        final_result.used_vertices.used_vertex_d = true;

        let point_outside_abc = Self::point_outside_of_plane(p, a, b, c, d);
        let point_outside_acd = Self::point_outside_of_plane(p, a, c, d, b);
        let point_outside_adb = Self::point_outside_of_plane(p, a, d, b, c);
        let point_outside_bdc = Self::point_outside_of_plane(p, b, d, c, a);

        if point_outside_abc < 0
            || point_outside_acd < 0
            || point_outside_adb < 0
            || point_outside_bdc < 0
        {
            final_result.degenerate = true;
            return false;
        }

        if point_outside_abc == 0
            && point_outside_acd == 0
            && point_outside_adb == 0
            && point_outside_bdc == 0
        {
            return false;
        }

        let mut best_sq_dist = Scalar::MAX;

        if point_outside_abc != 0 {
            Self::closest_pt_point_triangle(p, a, b, c, &mut temp_result);
            let q = temp_result.closest_point_on_simplex;

            let sq_dist = (q - p).dot(q - p);
            if sq_dist < best_sq_dist {
                best_sq_dist = sq_dist;
                final_result.closest_point_on_simplex = q;
                final_result.used_vertices.reset();
                final_result.used_vertices.used_vertex_a = temp_result.used_vertices.used_vertex_a;
                final_result.used_vertices.used_vertex_b = temp_result.used_vertices.used_vertex_b;
                final_result.used_vertices.used_vertex_c = temp_result.used_vertices.used_vertex_c;
                final_result.set_barycentric_coordinates(
                    temp_result.barycentric_coords[VERTA],
                    temp_result.barycentric_coords[VERTB],
                    temp_result.barycentric_coords[VERTC],
                    0.0,
                );
            }
        }

        if point_outside_acd != 0 {
            Self::closest_pt_point_triangle(p, a, c, d, &mut temp_result);
            let q = temp_result.closest_point_on_simplex;

            let sq_dist = (q - p).dot(q - p);
            if sq_dist < best_sq_dist {
                best_sq_dist = sq_dist;
                final_result.closest_point_on_simplex = q;
                final_result.used_vertices.reset();
                final_result.used_vertices.used_vertex_a = temp_result.used_vertices.used_vertex_a;
                final_result.used_vertices.used_vertex_c = temp_result.used_vertices.used_vertex_b;
                final_result.used_vertices.used_vertex_d = temp_result.used_vertices.used_vertex_c;
                final_result.set_barycentric_coordinates(
                    temp_result.barycentric_coords[VERTA],
                    0.0,
                    temp_result.barycentric_coords[VERTB],
                    temp_result.barycentric_coords[VERTC],
                );
            }
        }

        if point_outside_adb != 0 {
            Self::closest_pt_point_triangle(p, a, d, b, &mut temp_result);
            let q = temp_result.closest_point_on_simplex;

            let sq_dist = (q - p).dot(q - p);
            if sq_dist < best_sq_dist {
                best_sq_dist = sq_dist;
                final_result.closest_point_on_simplex = q;
                final_result.used_vertices.reset();
                final_result.used_vertices.used_vertex_a = temp_result.used_vertices.used_vertex_a;
                final_result.used_vertices.used_vertex_b = temp_result.used_vertices.used_vertex_c;
                final_result.used_vertices.used_vertex_d = temp_result.used_vertices.used_vertex_b;
                final_result.set_barycentric_coordinates(
                    temp_result.barycentric_coords[VERTA],
                    temp_result.barycentric_coords[VERTC],
                    0.0,
                    temp_result.barycentric_coords[VERTB],
                );
            }
        }

        if point_outside_bdc != 0 {
            Self::closest_pt_point_triangle(p, b, d, c, &mut temp_result);
            let q = temp_result.closest_point_on_simplex;

            let sq_dist = (q - p).dot(q - p);
            if sq_dist < best_sq_dist {
                final_result.closest_point_on_simplex = q;
                final_result.used_vertices.reset();
                final_result.used_vertices.used_vertex_b = temp_result.used_vertices.used_vertex_a;
                final_result.used_vertices.used_vertex_c = temp_result.used_vertices.used_vertex_c;
                final_result.used_vertices.used_vertex_d = temp_result.used_vertices.used_vertex_b;

                final_result.set_barycentric_coordinates(
                    0.0,
                    temp_result.barycentric_coords[VERTA],
                    temp_result.barycentric_coords[VERTC],
                    temp_result.barycentric_coords[VERTB],
                );
            }
        }

        // Upstream's tail is `if (all four used) return true; return true;` --
        // the same answer either way, so the branch is dropped rather than
        // reproduced as dead code.
        true
    }
}

/// `VERTA`/`VERTB`/`VERTC`/`VERTD` (`btVoronoiSimplexSolver.cpp:28-31`) -- the
/// weight slots a face's three barycentric coordinates are read out of before
/// being scattered into the tetrahedron's four.
const VERTA: usize = 0;
const VERTB: usize = 1;
const VERTC: usize = 2;

#[cfg(test)]
mod tests {
    use super::*;

    fn triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> SubSimplexClosestResult {
        let mut result = SubSimplexClosestResult::default();
        VoronoiSimplexSolver::closest_pt_point_triangle(p, a, b, c, &mut result);
        result
    }

    /// One case per Voronoi region of the triangle, which is what the seven
    /// branches are: three vertex regions, three edge regions, the face.
    #[test]
    fn closest_pt_point_triangle_covers_every_region() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(1.0, 0.0, 0.0);
        let c = Vec3::new(0.0, 1.0, 0.0);

        let outside_a = triangle(Vec3::new(-1.0, -1.0, 0.0), a, b, c);
        assert_eq!(outside_a.closest_point_on_simplex, a);
        assert_eq!(outside_a.barycentric_coords, [1.0, 0.0, 0.0, 0.0]);

        let outside_b = triangle(Vec3::new(2.0, -1.0, 0.0), a, b, c);
        assert_eq!(outside_b.closest_point_on_simplex, b);
        assert_eq!(outside_b.barycentric_coords, [0.0, 1.0, 0.0, 0.0]);

        let outside_c = triangle(Vec3::new(-1.0, 2.0, 0.0), a, b, c);
        assert_eq!(outside_c.closest_point_on_simplex, c);
        assert_eq!(outside_c.barycentric_coords, [0.0, 0.0, 1.0, 0.0]);

        let edge_ab = triangle(Vec3::new(0.5, -1.0, 0.0), a, b, c);
        assert_eq!(edge_ab.closest_point_on_simplex, Vec3::new(0.5, 0.0, 0.0));
        assert_eq!(
            edge_ab.used_vertices,
            UsageBitfield {
                used_vertex_a: true,
                used_vertex_b: true,
                used_vertex_c: false,
                used_vertex_d: false,
            }
        );

        let edge_ac = triangle(Vec3::new(-1.0, 0.5, 0.0), a, b, c);
        assert_eq!(edge_ac.closest_point_on_simplex, Vec3::new(0.0, 0.5, 0.0));

        let edge_bc = triangle(Vec3::new(1.0, 1.0, 0.0), a, b, c);
        assert_eq!(edge_bc.closest_point_on_simplex, Vec3::new(0.5, 0.5, 0.0));

        let face = triangle(Vec3::new(0.25, 0.25, 1.0), a, b, c);
        assert_eq!(face.closest_point_on_simplex, Vec3::new(0.25, 0.25, 0.0));
        assert_eq!(face.barycentric_coords, [0.5, 0.25, 0.25, 0.0]);
    }

    /// The degeneracy test is absolute, not relative: this tetrahedron is a
    /// perfectly good regular one, scaled down. Its triple product falls under
    /// `1e-4` and it reads as degenerate anyway -- a real Bullet property, and
    /// the reason a millimetre-scale robot behaves differently from a
    /// metre-scale one.
    #[test]
    fn tetrahedron_degeneracy_threshold_is_absolute_not_relative() {
        let unit = [
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, -1.0, -1.0),
            Vec3::new(-1.0, 1.0, -1.0),
            Vec3::new(-1.0, -1.0, 1.0),
        ];
        let at = |scale: Scalar, i: usize| unit[i] * scale;

        let big = VoronoiSimplexSolver::point_outside_of_plane(
            Vec3::new(5.0, 0.0, 0.0),
            at(1.0, 0),
            at(1.0, 1),
            at(1.0, 2),
            at(1.0, 3),
        );
        assert_eq!(big, 1, "a unit-scale tetrahedron is not degenerate");

        let small = VoronoiSimplexSolver::point_outside_of_plane(
            Vec3::new(0.005, 0.0, 0.0),
            at(0.001, 0),
            at(0.001, 1),
            at(0.001, 2),
            at(0.001, 3),
        );
        assert_eq!(small, -1, "the same shape at 1 mm reads as degenerate");
    }

    /// The origin strictly inside the tetrahedron is `false` **without**
    /// `degenerate`, and the solver turns that into "valid, zero separation" --
    /// which is how GJK learns to hand off to EPA.
    #[test]
    fn an_enclosed_origin_is_valid_with_zero_separation() {
        let mut solver = VoronoiSimplexSolver::new();
        for w in [
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, -1.0, -1.0),
            Vec3::new(-1.0, 1.0, -1.0),
            Vec3::new(-1.0, -1.0, 1.0),
        ] {
            solver.add_vertex(w, w, Vec3::zero());
        }
        let (v, valid) = solver.closest();
        assert!(valid);
        assert_eq!(v, Vec3::zero());
        assert_eq!(solver.num_vertices(), 4, "no reduction happened");
    }

    /// A degenerate (flat) full simplex is the other `false`, and it must come
    /// back invalid rather than as a zero separation.
    #[test]
    fn a_flat_tetrahedron_is_invalid_not_penetrating() {
        let mut solver = VoronoiSimplexSolver::new();
        for w in [
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(1.0, -1.0, 0.0),
            Vec3::new(-1.0, 1.0, 0.0),
            Vec3::new(-1.0, -1.0, 0.0),
        ] {
            solver.add_vertex(w, w, Vec3::zero());
        }
        let (_, valid) = solver.closest();
        assert!(!valid);
    }

    /// A segment whose closest point is an endpoint reduces to one vertex, and
    /// `removeVertex` swaps the survivor down from the end rather than shifting.
    #[test]
    fn a_segment_reduces_to_the_nearer_endpoint() {
        let mut solver = VoronoiSimplexSolver::new();
        solver.add_vertex(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::zero(),
        );
        solver.add_vertex(
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::zero(),
        );

        let (v, valid) = solver.closest();
        assert!(valid);
        assert_eq!(v, Vec3::new(1.0, 0.0, 0.0));
        assert_eq!(solver.num_vertices(), 1);
        let (_, _, w) = solver.simplex();
        assert_eq!(w, [Vec3::new(1.0, 0.0, 0.0)]);
    }

    /// A segment straddling the origin keeps both vertices and interpolates the
    /// witness points with the same `t`.
    #[test]
    fn a_straddling_segment_keeps_both_vertices() {
        let mut solver = VoronoiSimplexSolver::new();
        solver.add_vertex(
            Vec3::new(-1.0, 1.0, 0.0),
            Vec3::new(-1.0, 1.0, 0.0),
            Vec3::zero(),
        );
        solver.add_vertex(
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::zero(),
        );

        let (v, valid) = solver.closest();
        assert!(valid);
        assert_eq!(v, Vec3::new(0.0, 1.0, 0.0));
        assert_eq!(solver.num_vertices(), 2);

        let (p1, p2) = solver.compute_points();
        assert_eq!(p1, Vec3::new(0.0, 1.0, 0.0));
        assert_eq!(p2, Vec3::zero());
    }

    /// `inSimplex` answers on two grounds, and the `m_lastW` one survives the
    /// reduction that removed the vertex. Both are exercised: the reduced-away
    /// endpoint is still "in" because it was the last added, while a point at
    /// the surviving vertex is "in" by the distance test.
    #[test]
    fn in_simplex_remembers_the_last_added_vertex_after_reduction() {
        let mut solver = VoronoiSimplexSolver::new();
        let kept = Vec3::new(1.0, 0.0, 0.0);
        let dropped = Vec3::new(2.0, 0.0, 0.0);
        solver.add_vertex(kept, kept, Vec3::zero());
        solver.add_vertex(dropped, dropped, Vec3::zero());
        solver.closest();

        assert_eq!(solver.num_vertices(), 1);
        assert!(solver.in_simplex(kept), "still in the reduced simplex");
        assert!(
            solver.in_simplex(dropped),
            "gone from the simplex, but is m_lastW"
        );
        assert!(!solver.in_simplex(Vec3::new(5.0, 0.0, 0.0)));
    }

    /// The equal-vertex threshold is a squared distance, so its default of
    /// `1e-4` merges points up to `1e-2` apart.
    #[test]
    fn in_simplex_threshold_is_a_squared_distance() {
        let mut solver = VoronoiSimplexSolver::new();
        let w = Vec3::new(1.0, 0.0, 0.0);
        solver.add_vertex(w, w, Vec3::zero());

        assert_eq!(solver.equal_vertex_threshold(), 1e-4);
        assert!(solver.in_simplex(w + Vec3::new(0.0, 0.009, 0.0)));
        assert!(!solver.in_simplex(w + Vec3::new(0.0, 0.011, 0.0)));
    }

    /// `maxVertex` returns a squared length despite its name.
    #[test]
    fn max_vertex_is_a_squared_length() {
        let mut solver = VoronoiSimplexSolver::new();
        solver.add_vertex(Vec3::new(3.0, 4.0, 0.0), Vec3::zero(), Vec3::zero());
        solver.add_vertex(Vec3::new(1.0, 0.0, 0.0), Vec3::zero(), Vec3::zero());
        assert_eq!(solver.max_vertex(), 25.0);
    }

    /// `reset` seeds `m_lastW` far enough away that no real support point
    /// matches it.
    #[test]
    fn reset_seeds_last_w_out_of_reach() {
        let mut solver = VoronoiSimplexSolver::new();
        solver.add_vertex(Vec3::new(1.0, 0.0, 0.0), Vec3::zero(), Vec3::zero());
        solver.reset();
        assert!(solver.empty_simplex());
        assert!(!solver.in_simplex(Vec3::new(BT_LARGE_FLOAT, 0.0, 0.0)));
        assert!(solver.in_simplex(Vec3::new(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT)));
    }
}
