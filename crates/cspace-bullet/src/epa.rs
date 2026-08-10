// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2008 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/NarrowPhaseCollision/btGjkEpa2.h
//   bullet3/src/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp
//
// btGjkEpa2.cpp names its author in a line upstream carries in the file body
// rather than in the licence block, and so does this port:
//
//   GJK-EPA collision solver by Nathanael Presson, 2008
//   btGjkEpaSolver contributed under zlib by Nathanael Presson

//! `btGjkEpaSolver2` -- Presson's GJK for the separated case and EPA for the
//! penetrating one.
//!
//! This is the *second* GJK in Bullet's convex narrow phase and it is not a
//! duplicate of the first. [`crate::simplex`] is the incremental
//! `btVoronoiSimplexSolver` that `btGjkPairDetector`'s outer loop drives; this
//! module's [`Gjk`] is self-contained, keeps its own four-vertex store, and
//! exists to give EPA a simplex that already encloses the origin. They reach
//! the same answers by different arithmetic, and both are reachable from a
//! single `getClosestPoints` call, so porting one and reusing it for the other
//! would change results.
//!
//! # Arenas instead of pointers
//!
//! Upstream stores support vertices in fixed arrays (`m_store[4]`,
//! `m_sv_store[128]`) and threads raw pointers to them through the simplices
//! and the face list, with `m_fc_store[256]` faces on two intrusive doubly
//! linked lists (`m_hull`, `m_stock`). The port keeps the arrays at their
//! upstream sizes and replaces every pointer with an index into the array it
//! pointed at; the lists keep their `l[0]`/`l[1]` links as `Option<usize>`.
//! Face and vertex identity is therefore array position, which is what
//! upstream's pointer comparisons already meant.
//!
//! One place needs saying because it is not a mechanical substitution:
//! `EPA::Evaluate` copies the best face by value (`sFace outer = *best;`)
//! before continuing to expand the hull, and reads `outer.c[]` after the face
//! it was copied from has been recycled onto the stock list. The port copies
//! `Face` the same way, and that stays correct for the same reason it does
//! upstream -- `m_nextsv` only ever grows, so a support vertex is never
//! overwritten once a face has referred to it.
//!
//! [`EpaResultSimplex`] is the one type that holds vertices *by value* rather
//! than by index. Upstream's `m_result` is a `GJK::sSimplex`, so its `c[]` are
//! pointers, and on the fallback path they point into the *GJK* store while on
//! the normal path they point into EPA's. Nothing ever writes through them --
//! `Penetration` reads `->d` and `Distance` reads `->d` -- so the port copies
//! the two-vector vertex instead of modelling a pointer that can address
//! either arena.
//!
//! # Not ported
//!
//! `btGjkEpaSolver2::SignedDistance`, both overloads. They are the sphere-cast
//! entry points, reached from `btSoftBody` and `btGjkEpaSolver2`'s own tests;
//! `btGjkEpaPenetrationDepthSolver::calcPenDepth` -- the only caller MoveIt's
//! collision path has -- calls `Penetration` and `Distance` and nothing else.
//! `StackSizeRequirement` reports `sizeof(GJK) + sizeof(EPA)` for the SPU
//! scratch allocator, which has no meaning here.

use crate::linear_math::{Matrix3, Scalar, Transform, Vec3};
use crate::shapes::ConvexShape;

/// `GJK_MAX_ITERATIONS`.
const GJK_MAX_ITERATIONS: u32 = 128;
/// `GJK_ACCURACY`, single-precision branch.
const GJK_ACCURACY: Scalar = 0.0001;
/// `GJK_MIN_DISTANCE`, single-precision branch.
const GJK_MIN_DISTANCE: Scalar = 0.0001;
/// `GJK_DUPLICATED_EPS`, single-precision branch. A *squared* distance, so two
/// support points a centimetre apart count as the same vertex.
const GJK_DUPLICATED_EPS: Scalar = 0.0001;
/// `GJK_SIMPLEX2_EPS`, `GJK_SIMPLEX3_EPS`, `GJK_SIMPLEX4_EPS` -- all three are
/// literally `0.0` in every build, so `projectorigin`'s degeneracy tests are
/// strict `> 0` comparisons rather than tolerances.
const GJK_SIMPLEX_EPS: Scalar = 0.0;

/// `EPA_MAX_VERTICES`.
const EPA_MAX_VERTICES: usize = 128;
/// `EPA_MAX_ITERATIONS`.
const EPA_MAX_ITERATIONS: u32 = 255;
/// `EPA_ACCURACY`, single-precision branch.
const EPA_ACCURACY: Scalar = 0.0001;
/// `EPA_PLANE_EPS`, single-precision branch.
const EPA_PLANE_EPS: Scalar = 0.00001;
/// `EPA_MAX_FACES` -- `EPA_MAX_VERTICES * 2`.
const EPA_MAX_FACES: usize = EPA_MAX_VERTICES * 2;

/// `imd3` -- the "next index mod 3" table `projectorigin` and `expand` index
/// by hand rather than computing.
const IMD3: [usize; 3] = [1, 2, 0];

/// `GJK::sSV` -- a support vertex: the direction it was queried with, and the
/// resulting point of the Minkowski difference.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Sv {
    /// `d` -- the normalized query direction.
    pub d: Vec3,
    /// `w` -- `Support(d)` in the Minkowski difference.
    pub w: Vec3,
}

/// `MinkowskiDiff` -- the difference `A - B` expressed entirely in A's local
/// frame, which is why only one of the two transforms survives into the
/// results.
#[derive(Clone, Copy)]
pub struct MinkowskiDiff<'a> {
    shape0: &'a dyn ConvexShape,
    shape1: &'a dyn ConvexShape,
    /// `m_toshape1` -- `wtrs1.basisᵀ * wtrs0.basis`, rotating a direction from
    /// A's frame into B's.
    to_shape1: Matrix3,
    /// `m_toshape0` -- `wtrs0.inverseTimes(wtrs1)`, mapping a point from B's
    /// frame into A's.
    to_shape0: Transform,
    /// Upstream selects between the two support functions with a pointer to
    /// member (`Ls`), and its own `__SPU__` build selects with exactly this
    /// bool instead (`btGjkEpa2.cpp:118-142`).
    enable_margin: bool,
}

impl<'a> MinkowskiDiff<'a> {
    /// `Initialize` (`btGjkEpa2.cpp:904-918`), less the `sResults` fields it
    /// also clears -- those belong to the caller here.
    #[must_use]
    pub fn new(
        shape0: &'a dyn ConvexShape,
        wtrs0: &Transform,
        shape1: &'a dyn ConvexShape,
        wtrs1: &Transform,
        with_margins: bool,
    ) -> Self {
        Self {
            shape0,
            shape1,
            to_shape1: wtrs1.basis.transpose_times(&wtrs0.basis),
            to_shape0: wtrs0.inverse_times(wtrs1),
            enable_margin: with_margins,
        }
    }

    /// `MinkowskiDiff::Support0`.
    #[must_use]
    fn support0(&self, d: Vec3) -> Vec3 {
        if self.enable_margin {
            self.shape0.local_get_support_vertex_non_virtual(d)
        } else {
            self.shape0.local_get_supporting_vertex_without_margin(d)
        }
    }

    /// `MinkowskiDiff::Support1`.
    #[must_use]
    fn support1(&self, d: Vec3) -> Vec3 {
        let local = self.to_shape1 * d;
        let p = if self.enable_margin {
            self.shape1.local_get_support_vertex_non_virtual(local)
        } else {
            self.shape1
                .local_get_supporting_vertex_without_margin(local)
        };
        self.to_shape0.transform_point(p)
    }

    /// `MinkowskiDiff::Support(d)` -- the support of the difference.
    #[must_use]
    fn support(&self, d: Vec3) -> Vec3 {
        self.support0(d) - self.support1(-d)
    }

    /// `MinkowskiDiff::Support(d, index)`.
    #[must_use]
    fn support_of(&self, d: Vec3, index: usize) -> Vec3 {
        if index == 0 {
            self.support0(d)
        } else {
            self.support1(d)
        }
    }
}

/// `GJK::eStatus`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GjkStatus {
    /// `Valid` -- the simplex converged without containing the origin.
    Valid,
    /// `Inside` -- the origin is in the simplex, so the shapes overlap.
    Inside,
    /// `Failed` -- iteration limit reached.
    Failed,
}

/// `GJK::sSimplex` -- up to four vertices of [`Gjk`]'s own store, with the
/// barycentric weight of each.
#[derive(Clone, Copy, Debug)]
struct Simplex {
    /// `c` -- indices into [`Gjk::store`].
    c: [usize; 4],
    /// `p` -- barycentric weights.
    p: [Scalar; 4],
    /// `rank` -- how many of `c`/`p` are live.
    rank: usize,
}

impl Default for Simplex {
    fn default() -> Self {
        Self {
            c: [0; 4],
            p: [0.0; 4],
            rank: 0,
        }
    }
}

/// `GJK` (`btGjkEpa2.cpp:155-553`) -- the distance sub-solver EPA is built on.
pub struct Gjk<'a> {
    shape: MinkowskiDiff<'a>,
    ray: Vec3,
    distance: Scalar,
    simplices: [Simplex; 2],
    store: [Sv; 4],
    free: [usize; 4],
    nfree: usize,
    current: usize,
    status: GjkStatus,
}

impl<'a> Gjk<'a> {
    /// `GJK::GJK` followed by `GJK::Initialize`.
    #[must_use]
    pub fn new(shape: MinkowskiDiff<'a>) -> Self {
        Self {
            shape,
            ray: Vec3::zero(),
            distance: 0.0,
            simplices: [Simplex::default(); 2],
            store: [Sv::default(); 4],
            free: [0; 4],
            nfree: 0,
            current: 0,
            status: GjkStatus::Failed,
        }
    }

    /// `m_distance` -- the separation, once [`Gjk::evaluate`] has run.
    #[must_use]
    pub fn distance(&self) -> Scalar {
        self.distance
    }

    /// `m_simplex` -- the surviving simplex, as `(vertex, weight)` pairs.
    pub fn simplex(&self) -> impl Iterator<Item = (Sv, Scalar)> + '_ {
        let s = &self.simplices[self.current];
        (0..s.rank).map(move |i| (self.store[s.c[i]], s.p[i]))
    }

    /// `GJK::Evaluate` (`btGjkEpa2.cpp:201-336`).
    ///
    /// The duplicate-vertex guard is worth naming: `lastw` is a *four*-entry
    /// ring the loop writes with `(clastw + 1) & 3`, so it remembers the last
    /// four support points and stops when a new one lands within
    /// `GJK_DUPLICATED_EPS` of any of them. That is what terminates the loop on
    /// a flat contact, where the strict accuracy test alone would cycle.
    pub fn evaluate(&mut self, guess: Vec3) -> GjkStatus {
        let mut iterations: u32 = 0;
        let mut sqdist;
        let mut alpha: Scalar = 0.0;
        let mut clastw: usize = 0;

        self.free = [0, 1, 2, 3];
        self.nfree = 4;
        self.current = 0;
        self.status = GjkStatus::Valid;
        self.distance = 0.0;

        self.simplices[0].rank = 0;
        self.ray = guess;
        let sqrl = self.ray.length2();
        let seed = if sqrl > 0.0 {
            -self.ray
        } else {
            Vec3::new(1.0, 0.0, 0.0)
        };
        self.append_vertice(0, seed);
        self.simplices[0].p[0] = 1.0;
        self.ray = self.store[self.simplices[0].c[0]].w;
        sqdist = sqrl;
        let mut lastw = [self.ray; 4];

        loop {
            let next = 1 - self.current;
            // Check zero.
            let rl = self.ray.length();
            if rl < GJK_MIN_DISTANCE {
                self.status = GjkStatus::Inside;
                break;
            }

            // Append a new vertex in the -ray direction.
            let dir = -self.ray;
            self.append_vertice(self.current, dir);
            let cs_rank = self.simplices[self.current].rank;
            let w = self.store[self.simplices[self.current].c[cs_rank - 1]].w;

            if lastw
                .iter()
                .any(|&last| (w - last).length2() < GJK_DUPLICATED_EPS)
            {
                self.remove_vertice(self.current);
                break;
            }
            clastw = (clastw + 1) & 3;
            lastw[clastw] = w;

            // Check for termination.
            let omega = self.ray.dot(w) / rl;
            alpha = alpha.max(omega);
            if ((rl - alpha) - (GJK_ACCURACY * rl)) <= 0.0 {
                self.remove_vertice(self.current);
                break;
            }

            // Reduce the simplex.
            let mut weights = [0.0 as Scalar; 4];
            let mut mask: u32 = 0;
            let cs = self.simplices[self.current];
            let w0 = self.store[cs.c[0]].w;
            let w1 = self.store[cs.c[1]].w;
            sqdist = match cs.rank {
                2 => project_origin2(w0, w1, &mut weights, &mut mask),
                3 => project_origin3(w0, w1, self.store[cs.c[2]].w, &mut weights, &mut mask),
                4 => project_origin4(
                    w0,
                    w1,
                    self.store[cs.c[2]].w,
                    self.store[cs.c[3]].w,
                    &mut weights,
                    &mut mask,
                ),
                _ => sqdist,
            };

            if sqdist >= 0.0 {
                self.simplices[next].rank = 0;
                self.ray = Vec3::zero();
                self.current = next;
                for (i, (&slot, &weight)) in
                    cs.c.iter().zip(weights.iter()).take(cs.rank).enumerate()
                {
                    if mask & (1 << i) != 0 {
                        let rank = self.simplices[next].rank;
                        self.simplices[next].c[rank] = slot;
                        self.simplices[next].p[rank] = weight;
                        self.simplices[next].rank = rank + 1;
                        self.ray += self.store[slot].w * weight;
                    } else {
                        self.free[self.nfree] = slot;
                        self.nfree += 1;
                    }
                }
                if mask == 15 {
                    self.status = GjkStatus::Inside;
                }
            } else {
                self.remove_vertice(self.current);
                break;
            }

            iterations += 1;
            if iterations >= GJK_MAX_ITERATIONS {
                self.status = GjkStatus::Failed;
            }
            if self.status != GjkStatus::Valid {
                break;
            }
        }

        match self.status {
            GjkStatus::Valid => self.distance = self.ray.length(),
            GjkStatus::Inside => self.distance = 0.0,
            GjkStatus::Failed => {}
        }
        self.status
    }

    /// `GJK::EncloseOrigin` (`btGjkEpa2.cpp:338-402`) -- grow the surviving
    /// simplex to a non-degenerate tetrahedron, which is EPA's precondition.
    ///
    /// The rank-1 and rank-2 arms try both signs of each candidate direction
    /// and recurse, so this can add and drop several vertices before answering.
    pub fn enclose_origin(&mut self) -> bool {
        let current = self.current;
        match self.simplices[current].rank {
            1 => {
                for i in 0..3 {
                    let mut axis = Vec3::zero();
                    axis[i] = 1.0;
                    if self.try_enclose_with(axis) {
                        return true;
                    }
                    if self.try_enclose_with(-axis) {
                        return true;
                    }
                }
            }
            2 => {
                let s = self.simplices[current];
                let d = self.store[s.c[1]].w - self.store[s.c[0]].w;
                for i in 0..3 {
                    let mut axis = Vec3::zero();
                    axis[i] = 1.0;
                    let p = d.cross(axis);
                    if p.length2() > 0.0 {
                        if self.try_enclose_with(p) {
                            return true;
                        }
                        if self.try_enclose_with(-p) {
                            return true;
                        }
                    }
                }
            }
            3 => {
                let s = self.simplices[current];
                let n = (self.store[s.c[1]].w - self.store[s.c[0]].w)
                    .cross(self.store[s.c[2]].w - self.store[s.c[0]].w);
                if n.length2() > 0.0 {
                    if self.try_enclose_with(n) {
                        return true;
                    }
                    if self.try_enclose_with(-n) {
                        return true;
                    }
                }
            }
            4 => {
                let s = self.simplices[current];
                let d3 = self.store[s.c[3]].w;
                if det(
                    self.store[s.c[0]].w - d3,
                    self.store[s.c[1]].w - d3,
                    self.store[s.c[2]].w - d3,
                )
                .abs()
                    > 0.0
                {
                    return true;
                }
            }
            _ => {}
        }
        false
    }

    /// The `appendvertice` / recurse / `removevertice` triple every
    /// `EncloseOrigin` arm repeats.
    fn try_enclose_with(&mut self, v: Vec3) -> bool {
        let current = self.current;
        self.append_vertice(current, v);
        if self.enclose_origin() {
            return true;
        }
        self.remove_vertice(current);
        false
    }

    /// `GJK::getsupport` (`btGjkEpa2.cpp:404-408`).
    ///
    /// `d / d.length()`, not `d.normalized()` -- upstream never guards this, so
    /// a zero direction reaching here yields a NaN vertex rather than a
    /// fallback.
    #[must_use]
    pub fn get_support(&self, d: Vec3) -> Sv {
        let dir = d / d.length();
        Sv {
            d: dir,
            w: self.shape.support(dir),
        }
    }

    /// `GJK::removevertice`.
    fn remove_vertice(&mut self, simplex: usize) {
        self.simplices[simplex].rank -= 1;
        let rank = self.simplices[simplex].rank;
        self.free[self.nfree] = self.simplices[simplex].c[rank];
        self.nfree += 1;
    }

    /// `GJK::appendvertice`.
    fn append_vertice(&mut self, simplex: usize, v: Vec3) {
        let rank = self.simplices[simplex].rank;
        self.simplices[simplex].p[rank] = 0.0;
        self.nfree -= 1;
        let slot = self.free[self.nfree];
        self.simplices[simplex].c[rank] = slot;
        self.simplices[simplex].rank = rank + 1;
        self.store[slot] = self.get_support(v);
    }
}

/// `GJK::det` -- the scalar triple product, written out in upstream's own term
/// order. Re-associating it would change the last bits.
fn det(a: Vec3, b: Vec3, c: Vec3) -> Scalar {
    a.y * b.z * c.x + a.z * b.x * c.y - a.x * b.z * c.y - a.y * b.x * c.z + a.x * b.y * c.z
        - a.z * b.y * c.x
}

/// `GJK::projectorigin(a, b, ...)` -- the origin's closest point on a segment,
/// as a bitmask of which endpoints carry weight.
fn project_origin2(a: Vec3, b: Vec3, w: &mut [Scalar; 4], m: &mut u32) -> Scalar {
    let d = b - a;
    let l = d.length2();
    if l > GJK_SIMPLEX_EPS {
        let t = if l > 0.0 { -a.dot(d) / l } else { 0.0 };
        if t >= 1.0 {
            w[0] = 0.0;
            w[1] = 1.0;
            *m = 2;
            return b.length2();
        } else if t <= 0.0 {
            w[0] = 1.0;
            w[1] = 0.0;
            *m = 1;
            return a.length2();
        }
        w[1] = t;
        w[0] = 1.0 - t;
        *m = 3;
        return (a + d * t).length2();
    }
    -1.0
}

/// `GJK::projectorigin(a, b, c, ...)` -- the triangle case.
fn project_origin3(a: Vec3, b: Vec3, c: Vec3, w: &mut [Scalar; 4], m: &mut u32) -> Scalar {
    let vt = [a, b, c];
    let dl = [a - b, b - c, c - a];
    let n = dl[0].cross(dl[1]);
    let l = n.length2();
    if l > GJK_SIMPLEX_EPS {
        let mut mindist: Scalar = -1.0;
        let mut subw = [0.0 as Scalar; 4];
        let mut subm: u32 = 0;
        for i in 0..3 {
            if vt[i].dot(dl[i].cross(n)) > 0.0 {
                let j = IMD3[i];
                let subd = project_origin2(vt[i], vt[j], &mut subw, &mut subm);
                if (mindist < 0.0) || (subd < mindist) {
                    mindist = subd;
                    *m = (if subm & 1 != 0 { 1 << i } else { 0 })
                        + (if subm & 2 != 0 { 1 << j } else { 0 });
                    w[i] = subw[0];
                    w[j] = subw[1];
                    w[IMD3[j]] = 0.0;
                }
            }
        }
        if mindist < 0.0 {
            let d = a.dot(n);
            let s = l.sqrt();
            let p = n * (d / l);
            mindist = p.length2();
            *m = 7;
            w[0] = dl[1].cross(b - p).length() / s;
            w[1] = dl[2].cross(c - p).length() / s;
            w[2] = 1.0 - (w[0] + w[1]);
        }
        return mindist;
    }
    -1.0
}

/// `GJK::projectorigin(a, b, c, d, ...)` -- the tetrahedron case. Mask `15`
/// means the origin is strictly inside, which is what promotes GJK's status to
/// `Inside` and hands the problem to EPA.
fn project_origin4(a: Vec3, b: Vec3, c: Vec3, d: Vec3, w: &mut [Scalar; 4], m: &mut u32) -> Scalar {
    let vt = [a, b, c, d];
    let dl = [a - d, b - d, c - d];
    let vl = det(dl[0], dl[1], dl[2]);
    let ng = (vl * a.dot((b - c).cross(a - b))) <= 0.0;
    if ng && (vl.abs() > GJK_SIMPLEX_EPS) {
        let mut mindist: Scalar = -1.0;
        let mut subw = [0.0 as Scalar; 4];
        let mut subm: u32 = 0;
        for i in 0..3 {
            let j = IMD3[i];
            let s = vl * d.dot(dl[i].cross(dl[j]));
            if s > 0.0 {
                let subd = project_origin3(vt[i], vt[j], d, &mut subw, &mut subm);
                if (mindist < 0.0) || (subd < mindist) {
                    mindist = subd;
                    *m = (if subm & 1 != 0 { 1 << i } else { 0 })
                        + (if subm & 2 != 0 { 1 << j } else { 0 })
                        + (if subm & 4 != 0 { 8 } else { 0 });
                    w[i] = subw[0];
                    w[j] = subw[1];
                    w[IMD3[j]] = 0.0;
                    w[3] = subw[2];
                }
            }
        }
        if mindist < 0.0 {
            mindist = 0.0;
            *m = 15;
            w[0] = det(c, b, d) / vl;
            w[1] = det(a, c, d) / vl;
            w[2] = det(b, a, d) / vl;
            w[3] = 1.0 - (w[0] + w[1] + w[2]);
        }
        return mindist;
    }
    -1.0
}

/// `EPA::eStatus`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EpaStatus {
    /// `Valid`.
    Valid,
    /// `Degenerated` -- `newface` saw a zero-area triangle.
    Degenerated,
    /// `NonConvex` -- a face's supporting plane put the origin outside.
    NonConvex,
    /// `InvalidHull` -- `expand` could not close the horizon.
    InvalidHull,
    /// `OutOfFaces`.
    OutOfFaces,
    /// `OutOfVertices`.
    OutOfVertices,
    /// `AccuraryReached` (upstream's spelling) -- the normal converged.
    AccuraryReached,
    /// `FallBack` -- the simplex could not be grown to enclose the origin, so
    /// the answer is the caller's guess direction with zero depth.
    FallBack,
    /// `Failed`.
    Failed,
}

/// `EPA::sFace`. Copied by value in one place; see the module docs.
#[derive(Clone, Copy, Debug)]
struct Face {
    /// `n` -- the outward unit normal.
    n: Vec3,
    /// `d` -- the origin's distance to the face.
    d: Scalar,
    /// `c` -- indices into [`Epa::sv_store`].
    c: [usize; 3],
    /// `f` -- the adjacent face across each edge.
    f: [Option<usize>; 3],
    /// `l` -- previous/next in whichever list this face is on.
    l: [Option<usize>; 2],
    /// `e` -- which edge of `f[i]` this face is bound to.
    e: [u8; 3],
    /// `pass` -- the expansion pass that last visited this face.
    pass: u8,
}

impl Default for Face {
    fn default() -> Self {
        Self {
            n: Vec3::zero(),
            d: 0.0,
            c: [0; 3],
            f: [None; 3],
            l: [None; 2],
            e: [0; 3],
            pass: 0,
        }
    }
}

/// `EPA::sList`.
#[derive(Clone, Copy, Debug, Default)]
struct List {
    root: Option<usize>,
    count: usize,
}

/// `EPA::sHorizon`.
#[derive(Clone, Copy, Debug, Default)]
struct Horizon {
    cf: Option<usize>,
    ff: Option<usize>,
    nf: u32,
}

/// The result simplex, holding its vertices by value; see the module docs for
/// why this is not `Simplex`.
#[derive(Clone, Copy, Debug)]
pub struct EpaResultSimplex {
    /// `c` -- the witness vertices.
    pub c: [Sv; 4],
    /// `p` -- their barycentric weights.
    pub p: [Scalar; 4],
    /// `rank` -- how many are live.
    pub rank: usize,
}

impl Default for EpaResultSimplex {
    fn default() -> Self {
        Self {
            c: [Sv::default(); 4],
            p: [0.0; 4],
            rank: 0,
        }
    }
}

/// `EPA` (`btGjkEpa2.cpp:555-901`) -- expand the enclosing simplex outward
/// until the nearest face of the Minkowski difference's hull is found.
pub struct Epa {
    status: EpaStatus,
    result: EpaResultSimplex,
    normal: Vec3,
    depth: Scalar,
    sv_store: Vec<Sv>,
    faces: Vec<Face>,
    hull: List,
    stock: List,
}

impl Default for Epa {
    fn default() -> Self {
        Self::new()
    }
}

impl Epa {
    /// `EPA::EPA` followed by `EPA::Initialize` (`btGjkEpa2.cpp:637-647`).
    ///
    /// The stock list is seeded back to front so that its root is face `0`,
    /// which is the order `newface` then hands faces out in.
    #[must_use]
    pub fn new() -> Self {
        let mut epa = Self {
            status: EpaStatus::Failed,
            result: EpaResultSimplex::default(),
            normal: Vec3::zero(),
            depth: 0.0,
            sv_store: Vec::with_capacity(EPA_MAX_VERTICES),
            faces: vec![Face::default(); EPA_MAX_FACES],
            hull: List::default(),
            stock: List::default(),
        };
        for i in 0..EPA_MAX_FACES {
            epa.append_to_stock(EPA_MAX_FACES - i - 1);
        }
        epa
    }

    /// `m_normal` -- the separating normal, pointing from B towards A.
    #[must_use]
    pub fn normal(&self) -> Vec3 {
        self.normal
    }

    /// `m_depth` -- the penetration depth.
    #[must_use]
    pub fn depth(&self) -> Scalar {
        self.depth
    }

    /// `m_result` -- the face the depth was read off, as `(vertex, weight)`
    /// pairs.
    pub fn result(&self) -> impl Iterator<Item = (Sv, Scalar)> + '_ {
        (0..self.result.rank).map(move |i| (self.result.c[i], self.result.p[i]))
    }

    /// `EPA::Evaluate` (`btGjkEpa2.cpp:648-767`).
    pub fn evaluate(&mut self, gjk: &mut Gjk<'_>, guess: Vec3) -> EpaStatus {
        if (gjk.simplices[gjk.current].rank > 1) && gjk.enclose_origin() {
            // Clean up.
            while let Some(f) = self.hull.root {
                self.remove_from_hull(f);
                self.append_to_stock(f);
            }
            self.status = EpaStatus::Valid;
            self.sv_store.clear();

            // Orient the simplex.
            let current = gjk.current;
            {
                let s = gjk.simplices[current];
                let d3 = gjk.store[s.c[3]].w;
                if det(
                    gjk.store[s.c[0]].w - d3,
                    gjk.store[s.c[1]].w - d3,
                    gjk.store[s.c[2]].w - d3,
                ) < 0.0
                {
                    gjk.simplices[current].c.swap(0, 1);
                    gjk.simplices[current].p.swap(0, 1);
                }
            }

            // The simplex vertices move into EPA's own store, which is what
            // the faces then index; upstream's faces point straight at the GJK
            // store instead, and never write through those pointers.
            let s = gjk.simplices[current];
            let sv: Vec<Sv> = (0..4).map(|i| gjk.store[s.c[i]]).collect();
            self.sv_store.extend_from_slice(&sv);

            // Build the initial hull.
            let tetra = [
                self.new_face(0, 1, 2, true),
                self.new_face(1, 0, 3, true),
                self.new_face(2, 1, 3, true),
                self.new_face(0, 2, 3, true),
            ];
            if self.hull.count == 4 {
                let tetra = tetra.map(|f| f.expect("hull.count == 4 means all four faces landed"));
                let mut best = self.find_best();
                let mut outer = self.faces[best];
                let mut pass: u8 = 0;
                self.bind(tetra[0], 0, tetra[1], 0);
                self.bind(tetra[0], 1, tetra[2], 0);
                self.bind(tetra[0], 2, tetra[3], 0);
                self.bind(tetra[1], 1, tetra[3], 2);
                self.bind(tetra[1], 2, tetra[2], 1);
                self.bind(tetra[2], 2, tetra[3], 1);
                self.status = EpaStatus::Valid;

                for _ in 0..EPA_MAX_ITERATIONS {
                    if self.sv_store.len() >= EPA_MAX_VERTICES {
                        self.status = EpaStatus::OutOfVertices;
                        break;
                    }
                    let mut horizon = Horizon::default();
                    pass = pass.wrapping_add(1);
                    self.faces[best].pass = pass;
                    let w = gjk.get_support(self.faces[best].n);
                    self.sv_store.push(w);
                    let w_index = self.sv_store.len() - 1;
                    let wdist = self.faces[best].n.dot(w.w) - self.faces[best].d;
                    if wdist <= EPA_ACCURACY {
                        self.status = EpaStatus::AccuraryReached;
                        break;
                    }
                    let mut valid = true;
                    for j in 0..3 {
                        if !valid {
                            break;
                        }
                        let (f, e) = (self.faces[best].f[j], self.faces[best].e[j]);
                        valid &= self.expand(pass, w_index, f, usize::from(e), &mut horizon);
                    }
                    if valid && (horizon.nf >= 3) {
                        let (cf, ff) = (
                            horizon.cf.expect("nf >= 3 means the horizon has a face"),
                            horizon.ff.expect("nf >= 3 means the horizon has a face"),
                        );
                        self.bind(cf, 1, ff, 2);
                        self.remove_from_hull(best);
                        self.append_to_stock(best);
                        best = self.find_best();
                        outer = self.faces[best];
                    } else {
                        self.status = EpaStatus::InvalidHull;
                        break;
                    }
                }

                let projection = outer.n * outer.d;
                self.normal = outer.n;
                self.depth = outer.d;
                self.result.rank = 3;
                let c = [
                    self.sv_store[outer.c[0]],
                    self.sv_store[outer.c[1]],
                    self.sv_store[outer.c[2]],
                ];
                self.result.c[0] = c[0];
                self.result.c[1] = c[1];
                self.result.c[2] = c[2];
                self.result.p[0] = (c[1].w - projection).cross(c[2].w - projection).length();
                self.result.p[1] = (c[2].w - projection).cross(c[0].w - projection).length();
                self.result.p[2] = (c[0].w - projection).cross(c[1].w - projection).length();
                let sum = self.result.p[0] + self.result.p[1] + self.result.p[2];
                self.result.p[0] /= sum;
                self.result.p[1] /= sum;
                self.result.p[2] /= sum;
                return self.status;
            }
        }

        // Fallback.
        self.status = EpaStatus::FallBack;
        self.normal = -guess;
        let nl = self.normal.length();
        if nl > 0.0 {
            self.normal /= nl;
        } else {
            self.normal = Vec3::new(1.0, 0.0, 0.0);
        }
        self.depth = 0.0;
        self.result.rank = 1;
        let s = gjk.simplices[gjk.current];
        self.result.c[0] = gjk.store[s.c[0]];
        self.result.p[0] = 1.0;
        self.status
    }

    /// `EPA::bind`.
    fn bind(&mut self, fa: usize, ea: usize, fb: usize, eb: usize) {
        self.faces[fa].e[ea] = u8::try_from(eb).expect("edge index is 0..3");
        self.faces[fa].f[ea] = Some(fb);
        self.faces[fb].e[eb] = u8::try_from(ea).expect("edge index is 0..3");
        self.faces[fb].f[eb] = Some(fa);
    }

    /// `EPA::append(m_hull, face)`.
    fn append_to_hull(&mut self, face: usize) {
        self.faces[face].l[0] = None;
        self.faces[face].l[1] = self.hull.root;
        if let Some(root) = self.hull.root {
            self.faces[root].l[0] = Some(face);
        }
        self.hull.root = Some(face);
        self.hull.count += 1;
    }

    /// `EPA::append(m_stock, face)`.
    fn append_to_stock(&mut self, face: usize) {
        self.faces[face].l[0] = None;
        self.faces[face].l[1] = self.stock.root;
        if let Some(root) = self.stock.root {
            self.faces[root].l[0] = Some(face);
        }
        self.stock.root = Some(face);
        self.stock.count += 1;
    }

    /// `EPA::remove(m_hull, face)`.
    fn remove_from_hull(&mut self, face: usize) {
        let [prev, next] = self.faces[face].l;
        if let Some(next) = next {
            self.faces[next].l[0] = prev;
        }
        if let Some(prev) = prev {
            self.faces[prev].l[1] = next;
        }
        if self.hull.root == Some(face) {
            self.hull.root = next;
        }
        self.hull.count -= 1;
    }

    /// `EPA::remove(m_stock, face)`.
    fn remove_from_stock(&mut self, face: usize) {
        let [prev, next] = self.faces[face].l;
        if let Some(next) = next {
            self.faces[next].l[0] = prev;
        }
        if let Some(prev) = prev {
            self.faces[prev].l[1] = next;
        }
        if self.stock.root == Some(face) {
            self.stock.root = next;
        }
        self.stock.count -= 1;
    }

    /// `EPA::getedgedist` (`btGjkEpa2.cpp:769-803`) -- the origin's distance to
    /// an edge, and whether it falls outside that edge at all.
    fn get_edge_dist(&self, face: usize, a: usize, b: usize, dist: &mut Scalar) -> bool {
        let (aw, bw) = (self.sv_store[a].w, self.sv_store[b].w);
        let ba = bw - aw;
        let n_ab = ba.cross(self.faces[face].n);
        let a_dot_nab = aw.dot(n_ab);

        if a_dot_nab < 0.0 {
            let ba_l2 = ba.length2();
            let a_dot_ba = aw.dot(ba);
            let b_dot_ba = bw.dot(ba);

            if a_dot_ba > 0.0 {
                *dist = aw.length();
            } else if b_dot_ba < 0.0 {
                *dist = bw.length();
            } else {
                let a_dot_b = aw.dot(bw);
                *dist = ((aw.length2() * bw.length2() - a_dot_b * a_dot_b) / ba_l2)
                    .max(0.0)
                    .sqrt();
            }
            return true;
        }
        false
    }

    /// `EPA::newface` (`btGjkEpa2.cpp:805-847`).
    ///
    /// Note upstream's dead arm at the end: the `OutOfVertices` branch is
    /// guarded by `m_stock.root`, which is already known to be null there, so
    /// only `OutOfFaces` is reachable. Reproduced rather than tidied, since the
    /// status it does not set is one `Evaluate` can observe.
    fn new_face(&mut self, a: usize, b: usize, c: usize, forced: bool) -> Option<usize> {
        let Some(face) = self.stock.root else {
            self.status = EpaStatus::OutOfFaces;
            return None;
        };
        self.remove_from_stock(face);
        self.append_to_hull(face);
        self.faces[face].pass = 0;
        self.faces[face].c = [a, b, c];
        let (aw, bw, cw) = (self.sv_store[a].w, self.sv_store[b].w, self.sv_store[c].w);
        self.faces[face].n = (bw - aw).cross(cw - aw);
        let l = self.faces[face].n.length();

        if l > EPA_ACCURACY {
            let mut d = 0.0;
            if !(self.get_edge_dist(face, a, b, &mut d)
                || self.get_edge_dist(face, b, c, &mut d)
                || self.get_edge_dist(face, c, a, &mut d))
            {
                // The origin projects into the triangle's interior, so the
                // plane distance is the answer.
                d = aw.dot(self.faces[face].n) / l;
            }
            self.faces[face].d = d;
            self.faces[face].n /= l;
            if forced || (self.faces[face].d >= -EPA_PLANE_EPS) {
                return Some(face);
            }
            self.status = EpaStatus::NonConvex;
        } else {
            self.status = EpaStatus::Degenerated;
        }

        self.remove_from_hull(face);
        self.append_to_stock(face);
        None
    }

    /// `EPA::findbest` -- the hull face whose *squared* distance to the origin
    /// is smallest.
    fn find_best(&self) -> usize {
        let mut minf = self.hull.root.expect("findbest is only called on a hull");
        let mut mind = self.faces[minf].d * self.faces[minf].d;
        let mut f = self.faces[minf].l[1];
        while let Some(face) = f {
            let sqd = self.faces[face].d * self.faces[face].d;
            if sqd < mind {
                minf = face;
                mind = sqd;
            }
            f = self.faces[face].l[1];
        }
        minf
    }

    /// `EPA::expand` (`btGjkEpa2.cpp:864-898`) -- walk the silhouette of `w`
    /// across the hull, replacing every face it can see with a new one.
    fn expand(
        &mut self,
        pass: u8,
        w: usize,
        f: Option<usize>,
        e: usize,
        horizon: &mut Horizon,
    ) -> bool {
        const I1M3: [usize; 3] = [1, 2, 0];
        const I2M3: [usize; 3] = [2, 0, 1];

        let Some(f) = f else { return false };
        if self.faces[f].pass == pass {
            return false;
        }
        let e1 = I1M3[e];
        if (self.faces[f].n.dot(self.sv_store[w].w) - self.faces[f].d) < -EPA_PLANE_EPS {
            let (c_e1, c_e) = (self.faces[f].c[e1], self.faces[f].c[e]);
            if let Some(nf) = self.new_face(c_e1, c_e, w, false) {
                self.bind(nf, 0, f, e);
                if let Some(cf) = horizon.cf {
                    self.bind(cf, 1, nf, 2);
                } else {
                    horizon.ff = Some(nf);
                }
                horizon.cf = Some(nf);
                horizon.nf += 1;
                return true;
            }
        } else {
            let e2 = I2M3[e];
            self.faces[f].pass = pass;
            // `f`'s own links are re-read between the two calls, not hoisted:
            // upstream's `&&` sequences the left `expand` fully before it
            // loads `f->f[e2]`/`f->e[e2]`, and the recursion binds new faces
            // onto whichever face it walked into.
            let (f1, ee1) = (self.faces[f].f[e1], self.faces[f].e[e1]);
            if self.expand(pass, w, f1, usize::from(ee1), horizon) {
                let (f2, ee2) = (self.faces[f].f[e2], self.faces[f].e[e2]);
                if self.expand(pass, w, f2, usize::from(ee2), horizon) {
                    self.remove_from_hull(f);
                    self.append_to_stock(f);
                    return true;
                }
            }
        }
        false
    }
}

/// `btGjkEpaSolver2::sResults::eStatus`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResultStatus {
    /// `Separated`.
    Separated,
    /// `Penetrating`.
    Penetrating,
    /// `GJK_Failed`.
    GjkFailed,
    /// `EPA_Failed`.
    EpaFailed,
}

/// `btGjkEpaSolver2::sResults`.
#[derive(Clone, Copy, Debug)]
pub struct Results {
    /// `status`.
    pub status: ResultStatus,
    /// `witnesses` -- the closest (or deepest) point on each shape, in world
    /// space. Both are produced by applying `wtrs0`, because the whole
    /// computation lives in A's frame.
    pub witnesses: [Vec3; 2],
    /// `normal`.
    pub normal: Vec3,
    /// `distance` -- negative when penetrating.
    pub distance: Scalar,
}

impl Default for Results {
    fn default() -> Self {
        Self {
            status: ResultStatus::Separated,
            witnesses: [Vec3::zero(); 2],
            normal: Vec3::zero(),
            distance: 0.0,
        }
    }
}

/// `btGjkEpaSolver2::Distance` (`btGjkEpa2.cpp:936-971`) -- margin-free.
#[must_use]
pub fn distance(
    shape0: &dyn ConvexShape,
    wtrs0: &Transform,
    shape1: &dyn ConvexShape,
    wtrs1: &Transform,
    guess: Vec3,
    results: &mut Results,
) -> bool {
    *results = Results::default();
    let shape = MinkowskiDiff::new(shape0, wtrs0, shape1, wtrs1, false);
    let mut gjk = Gjk::new(shape);
    if gjk.evaluate(guess) == GjkStatus::Valid {
        let mut w0 = Vec3::zero();
        let mut w1 = Vec3::zero();
        for (sv, p) in gjk.simplex() {
            w0 += shape.support_of(sv.d, 0) * p;
            w1 += shape.support_of(-sv.d, 1) * p;
        }
        results.witnesses[0] = wtrs0.transform_point(w0);
        results.witnesses[1] = wtrs0.transform_point(w1);
        results.normal = w0 - w1;
        results.distance = results.normal.length();
        results.normal /= if results.distance > GJK_MIN_DISTANCE {
            results.distance
        } else {
            1.0
        };
        true
    } else {
        results.status = if gjk.status == GjkStatus::Inside {
            ResultStatus::Penetrating
        } else {
            ResultStatus::GjkFailed
        };
        false
    }
}

/// `btGjkEpaSolver2::Penetration` (`btGjkEpa2.cpp:974-1017`).
///
/// Both GJK and EPA are run on `-guess`, not `guess`: the caller's guess points
/// from A towards B and the Minkowski difference is `A - B`.
#[must_use]
pub fn penetration(
    shape0: &dyn ConvexShape,
    wtrs0: &Transform,
    shape1: &dyn ConvexShape,
    wtrs1: &Transform,
    guess: Vec3,
    results: &mut Results,
    use_margins: bool,
) -> bool {
    *results = Results::default();
    let shape = MinkowskiDiff::new(shape0, wtrs0, shape1, wtrs1, use_margins);
    let mut gjk = Gjk::new(shape);
    match gjk.evaluate(-guess) {
        GjkStatus::Inside => {
            let mut epa = Epa::new();
            if epa.evaluate(&mut gjk, -guess) != EpaStatus::Failed {
                let mut w0 = Vec3::zero();
                for (sv, p) in epa.result() {
                    w0 += shape.support_of(sv.d, 0) * p;
                }
                results.status = ResultStatus::Penetrating;
                results.witnesses[0] = wtrs0.transform_point(w0);
                results.witnesses[1] = wtrs0.transform_point(w0 - epa.normal() * epa.depth());
                results.normal = -epa.normal();
                results.distance = -epa.depth();
                return true;
            }
            results.status = ResultStatus::EpaFailed;
        }
        GjkStatus::Failed => results.status = ResultStatus::GjkFailed,
        GjkStatus::Valid => {}
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::probe_fixture::{IDENTITY, at, diff, diff_vec3, probe_shapes, rot60_at, row};
    use crate::shapes::{BoxShape, SphereShape};

    /// Two unit boxes 3 m apart: the separation is the gap between the faces,
    /// not between the centres, and the witnesses sit on the facing faces.
    #[test]
    fn distance_between_separated_boxes_is_the_face_gap() {
        let mut a = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
        let mut b = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
        a.set_margin(0.0);
        b.set_margin(0.0);

        let mut results = Results::default();
        assert!(distance(
            &a,
            &IDENTITY,
            &b,
            &at(3.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            &mut results,
        ));
        assert!((results.distance - 2.0).abs() < 1e-5, "{results:?}");
        assert!((results.witnesses[0].x - 0.5).abs() < 1e-5, "{results:?}");
        assert!((results.witnesses[1].x - 2.5).abs() < 1e-5, "{results:?}");
    }

    /// `Distance` returns false for overlapping shapes, with the status saying
    /// which way it failed -- that is the signal `calcPenDepth` reads to decide
    /// between a penetration and a separation result.
    #[test]
    fn distance_reports_penetrating_rather_than_a_negative_number() {
        let mut a = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
        let mut b = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
        a.set_margin(0.0);
        b.set_margin(0.0);

        let mut results = Results::default();
        assert!(!distance(
            &a,
            &IDENTITY,
            &b,
            &at(0.5, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            &mut results,
        ));
        assert_eq!(results.status, ResultStatus::Penetrating);
    }

    /// Two unit boxes overlapping by 0.5 m along x. The geometric depth is
    /// 0.5 along x; Bullet answers 0.288675129 along a corner diagonal, and
    /// so does this port -- see `bullet_reference_penetration`'s
    /// `box_box_deep_x` row and the note above `BULLET_REFERENCE` for why
    /// that is upstream's answer and not a defect. Asserted here only for
    /// the property the depth cannot change: EPA ran and said penetrating.
    #[test]
    fn penetration_of_overlapping_boxes_reports_penetrating() {
        let mut a = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
        let mut b = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
        a.set_margin(0.0);
        b.set_margin(0.0);

        let mut results = Results::default();
        assert!(penetration(
            &a,
            &IDENTITY,
            &b,
            &at(0.5, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            &mut results,
            false,
        ));
        assert_eq!(results.status, ResultStatus::Penetrating);
        assert!(results.distance < 0.0, "{results:?}");
    }

    /// A sphere is *all* margin, so the margin-free Minkowski difference of two
    /// spheres is a single point and `Distance` measures centre to centre.
    /// This is the case the margin flag actually changes.
    #[test]
    fn margins_change_what_the_sphere_pair_measures() {
        let a = SphereShape::new(0.5);
        let b = SphereShape::new(0.5);
        let wtrs1 = at(3.0, 0.0, 0.0);

        let mut without = Results::default();
        assert!(distance(
            &a,
            &IDENTITY,
            &b,
            &wtrs1,
            Vec3::new(1.0, 0.0, 0.0),
            &mut without,
        ));
        assert!((without.distance - 3.0).abs() < 1e-5, "{without:?}");

        // With margins the same pair penetrates nothing but is 2 m apart.
        let mut with = Results::default();
        assert!(penetration(
            &a,
            &IDENTITY,
            &b,
            &at(0.5, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            &mut with,
            true,
        ));
        assert!((with.distance + 0.5).abs() < 1e-4, "{with:?}");
    }

    /// `projectorigin`'s segment case, on both sides of each clamp and in the
    /// interior. The mask says which endpoints carry weight, and it is what
    /// decides which vertices survive the reduction.
    #[test]
    fn project_origin_on_a_segment_clamps_to_the_endpoints() {
        let mut w = [0.0 as Scalar; 4];
        let mut m = 0;

        // Origin beyond `a`: only `a` survives.
        let d = project_origin2(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            &mut w,
            &mut m,
        );
        assert_eq!((d, m, w[0], w[1]), (1.0, 1, 1.0, 0.0));

        // Origin beyond `b`: only `b` survives.
        let d = project_origin2(
            Vec3::new(-2.0, 0.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
            &mut w,
            &mut m,
        );
        assert_eq!((d, m, w[0], w[1]), (1.0, 2, 0.0, 1.0));

        // Origin between them: both survive, weighted by the split.
        let d = project_origin2(
            Vec3::new(-1.0, 1.0, 0.0),
            Vec3::new(3.0, 1.0, 0.0),
            &mut w,
            &mut m,
        );
        assert_eq!((d, m), (1.0, 3));
        assert_eq!((w[0], w[1]), (0.75, 0.25));
    }

    /// The tetrahedron case answers `15` -- and distance zero -- exactly when
    /// the origin is inside, which is the condition that sends the pair to EPA.
    #[test]
    fn project_origin_in_a_tetrahedron_answers_the_full_mask() {
        let mut w = [0.0 as Scalar; 4];
        let mut m = 0;
        let d = project_origin4(
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, -1.0, -1.0),
            Vec3::new(-1.0, 1.0, -1.0),
            Vec3::new(-1.0, -1.0, 1.0),
            &mut w,
            &mut m,
        );
        assert_eq!((d, m), (0.0, 15));
        assert!((w[0] + w[1] + w[2] + w[3] - 1.0).abs() < 1e-6);
    }

    /// Verbatim stdout of `tools/bullet-epa-reference/build.sh`, which runs
    /// the real `btGjkEpaSolver2` from bullet3 @ `7dee3436` on the pairs
    /// below. Pasted whole rather than transcribed field by field, so no row
    /// can pick up this file's idea of the field order; `probe.cpp` prints
    /// with `%.9g`, which round-trips a `float` exactly, so parsing a field
    /// back as `f32` recovers the bit pattern the C++ held.
    ///
    /// These are fixtures because EPA's answer is not the geometric one.
    /// `box_box_deep_x` is two unit boxes overlapping 0.5 m along x, for
    /// which the depth is plainly 0.5 along x -- and Bullet says 0.288675129
    /// along a corner diagonal. The enclosing tetrahedron `EncloseOrigin`
    /// builds there puts the origin *on* one of its edges; the silhouette
    /// walk then reaches a face already marked with the current pass,
    /// `expand` returns false, and `Evaluate` breaks out with `InvalidHull`
    /// leaving `outer` at whichever face was best when it gave up. Any
    /// hand-derived expectation for that row would be an assertion against
    /// Bullet rather than about it.
    ///
    /// Fields: `name|ok|status|distance|normal xyz|witness0 xyz|witness1 xyz`.
    const BULLET_REFERENCE: &str = "\
box_box_deep_x|1|1|-0.288675129|-0.577350259|0.577350259|0.577350259|0.5|0.166666672|0.333333313|0.333333343|0.333333313|0.49999997
box_box_shallow_x|1|1|-0.0577350408|-0.577350259|0.577350259|0.577350259|0.5|0.433333337|0.466666639|0.466666669|0.466666669|0.49999997
box_box_offset|1|1|-0.38783586|-0.816496551|0.408248276|0.408248276|0.49999997|0.416666657|0.141666681|0.183333337|0.574999988|0.300000012
box_box_rot60|1|1|-0.402492255|-0.89442718|-0|-0.44721359|0.5|0.269999981|0.5|0.139999986|0.269999981|0.319999993
box_box_margins|1|1|-0.0500000119|-1|-5.30293946e-05|5.30293946e-05|0.5|0.460000008|0.459997386|0.449999988|0.459997356|0.460000038
sphere_sphere|1|1|-0.299792975|-0.999309897|-0.037142314|-0|0.499793053|0.00556750409|0|0.200206965|-0.00556750037|0
sphere_box|1|1|-0.199999928|-1|-4.46721515e-07|4.16849105e-07|0.299999952|4.48888786e-05|5.7036661e-05|0.100000024|4.47995335e-05|5.71200326e-05
cyl_box|1|1|-0.199999705|-1|-3.33416779e-07|9.33621379e-07|0.299999714|0.000232525199|0.276245713|0.100000009|0.000232458522|0.276245892
cyl_cyl_rot60|1|1|-0.331670612|-0.44721067|-0.89442873|1.26854638e-06|0.134466961|0.268176138|-0.27908951|-0.0138596743|-0.0284795761|-0.279089093
cone_box|1|1|-0.131240785|-0.954480171|-6.21894969e-07|-0.298274249|0.175266743|0.000171859676|-0.160854235|0.0500000119|0.000171778054|-0.199999988
cone_sphere|1|1|-0.0941364989|-0.954357207|0.0135602225|-0.298359334|0.00361738726|-5.14130224e-05|0.388423175|-0.0862224549|0.00122509873|0.360336661
hull_box|1|1|-0.100000024|-1|-0|-0|0.300000012|0.0142857134|0|0.199999988|0.0142857134|0
hull_sphere_rot60|1|1|-0.19999674|-1|1.43533998e-05|3.71899732e-05|0.300000042|0.10026519|0.0506323725|0.100003302|0.100268058|0.0506398119
box_box_separated|0|0|0|0|0|0|0|0|0|0|0|0
d_box_box_far|1|0|2|-1|0|0|0.5|0.5|0.5|2.5|0.5|0.5
d_box_box_diag|1|0|1.14017546|-0.964763761|-0.263117403|0|0.5|0.5|0.333333313|1.60000002|0.800000012|0.333333313
d_sphere_box|1|0|1.5|-1|2.48352694e-09|9.93410776e-09|0|0|0|1.5|-3.7252903e-09|-1.49011612e-08
d_cyl_cone|1|0|1.07789433|-0.983065188|-0.18325603|3.05517574e-06|0.294741571|0.055892393|-0.19999671|1.35438204|0.253423035|-0.200000003
d_hull_sphere|1|0|1.17047|-0.939793348|-0.341743082|0|0.300000012|0.200000003|0|1.39999998|0.600000024|0
d_box_box_touching|0|1|0|0|0|0|0|0|0|0|0|0
";

    /// One parsed row of [`BULLET_REFERENCE`].
    struct Reference {
        ok: bool,
        status: ResultStatus,
        distance: Scalar,
        normal: Vec3,
        witnesses: [Vec3; 2],
    }

    fn reference(name: &str) -> Reference {
        let f = row(BULLET_REFERENCE, name, 13);
        let n = |i: usize| -> Scalar {
            f[i].parse()
                .unwrap_or_else(|e| panic!("{name}: field {i} ({:?}): {e}", f[i]))
        };
        Reference {
            ok: f[1] == "1",
            status: match f[2] {
                "0" => ResultStatus::Separated,
                "1" => ResultStatus::Penetrating,
                "2" => ResultStatus::GjkFailed,
                "3" => ResultStatus::EpaFailed,
                other => panic!("{name}: unknown status {other:?}"),
            },
            distance: n(3),
            normal: Vec3::new(n(4), n(5), n(6)),
            witnesses: [Vec3::new(n(7), n(8), n(9)), Vec3::new(n(10), n(11), n(12))],
        }
    }

    fn compare(into: &mut Vec<String>, name: &str, ok: bool, got: &Results) {
        let want = reference(name);
        if ok != want.ok {
            into.push(format!("{name}.ok: port {ok}, bullet {}", want.ok));
        }
        if got.status != want.status {
            into.push(format!(
                "{name}.status: port {:?}, bullet {:?}",
                got.status, want.status
            ));
        }
        diff(into, name, "distance", got.distance, want.distance);
        diff_vec3(into, name, "normal", got.normal, want.normal);
        for (i, (g, w)) in got.witnesses.iter().zip(want.witnesses.iter()).enumerate() {
            diff_vec3(into, name, &format!("witness{i}"), *g, *w);
        }
    }

    /// Every `Penetration` row of [`BULLET_REFERENCE`], against the port.
    #[test]
    fn bullet_reference_penetration() {
        let (unit_box, flat_box, margin_box, sphere, small_sphere, cyl, cone, hull) =
            probe_shapes();
        let gx = Vec3::new(1.0, 0.0, 0.0);
        let mut bad = Vec::new();

        let mut case = |name: &str,
                        a: &dyn ConvexShape,
                        ta: &Transform,
                        b: &dyn ConvexShape,
                        tb: &Transform,
                        usemargins: bool| {
            let mut r = Results::default();
            let ok = penetration(a, ta, b, tb, gx, &mut r, usemargins);
            compare(&mut bad, name, ok, &r);
        };

        case(
            "box_box_deep_x",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(0.5, 0.0, 0.0),
            false,
        );
        case(
            "box_box_shallow_x",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(0.9, 0.0, 0.0),
            false,
        );
        case(
            "box_box_offset",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(0.6, 0.35, -0.2),
            false,
        );
        case(
            "box_box_rot60",
            &unit_box,
            &IDENTITY,
            &flat_box,
            &rot60_at(0.7, 0.2, 0.1),
            false,
        );
        case(
            "box_box_margins",
            &margin_box,
            &IDENTITY,
            &margin_box,
            &at(0.95, 0.0, 0.0),
            true,
        );
        case(
            "sphere_sphere",
            &sphere,
            &IDENTITY,
            &sphere,
            &at(0.7, 0.0, 0.0),
            true,
        );
        case(
            "sphere_box",
            &small_sphere,
            &IDENTITY,
            &unit_box,
            &at(0.6, 0.1, 0.0),
            true,
        );
        case(
            "cyl_box",
            &cyl,
            &IDENTITY,
            &flat_box,
            &at(0.5, 0.1, 0.2),
            false,
        );
        case(
            "cyl_cyl_rot60",
            &cyl,
            &IDENTITY,
            &cyl,
            &rot60_at(0.4, 0.1, 0.0),
            false,
        );
        case(
            "cone_box",
            &cone,
            &IDENTITY,
            &unit_box,
            &at(0.55, 0.1, 0.3),
            false,
        );
        case(
            "cone_sphere",
            &cone,
            &IDENTITY,
            &small_sphere,
            &at(0.2, 0.0, 0.45),
            true,
        );
        case(
            "hull_box",
            &hull,
            &IDENTITY,
            &unit_box,
            &at(0.7, 0.05, 0.0),
            false,
        );
        case(
            "hull_sphere_rot60",
            &hull,
            &IDENTITY,
            &small_sphere,
            &rot60_at(0.4, 0.1, 0.05),
            true,
        );
        case(
            "box_box_separated",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(1.5, 0.0, 0.0),
            false,
        );

        assert!(
            bad.is_empty(),
            "{} deviations:\n{}",
            bad.len(),
            bad.join("\n")
        );
    }

    /// Every `Distance` row of [`BULLET_REFERENCE`], against the port.
    #[test]
    fn bullet_reference_distance() {
        let (unit_box, flat_box, _margin_box, sphere, small_sphere, cyl, cone, hull) =
            probe_shapes();
        let gx = Vec3::new(1.0, 0.0, 0.0);
        let mut bad = Vec::new();

        let mut case = |name: &str,
                        a: &dyn ConvexShape,
                        ta: &Transform,
                        b: &dyn ConvexShape,
                        tb: &Transform| {
            let mut r = Results::default();
            let ok = distance(a, ta, b, tb, gx, &mut r);
            compare(&mut bad, name, ok, &r);
        };

        case(
            "d_box_box_far",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(3.0, 0.0, 0.0),
        );
        case(
            "d_box_box_diag",
            &unit_box,
            &IDENTITY,
            &flat_box,
            &at(2.0, 1.5, 0.5),
        );
        case(
            "d_sphere_box",
            &small_sphere,
            &IDENTITY,
            &unit_box,
            &at(2.0, 0.4, 0.0),
        );
        case("d_cyl_cone", &cyl, &IDENTITY, &cone, &at(1.6, 0.3, 0.2));
        case(
            "d_hull_sphere",
            &hull,
            &IDENTITY,
            &sphere,
            &rot60_at(1.4, 0.6, 0.0),
        );
        case(
            "d_box_box_touching",
            &unit_box,
            &IDENTITY,
            &unit_box,
            &at(1.0, 0.0, 0.0),
        );

        assert!(
            bad.is_empty(),
            "{} deviations:\n{}",
            bad.len(),
            bad.join("\n")
        );
    }

    /// `det` is the scalar triple product, and its sign is what orients EPA's
    /// initial tetrahedron.
    #[test]
    fn det_is_the_signed_triple_product() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let z = Vec3::new(0.0, 0.0, 1.0);
        assert_eq!(det(x, y, z), 1.0);
        assert_eq!(det(y, x, z), -1.0);
        assert_eq!(det(x, y, x), 0.0);
    }
}
