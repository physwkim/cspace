// Copyright (c) 2011-2014, Willow Garage, Inc.
// Copyright (c) 2014-2016, Open Source Robotics Foundation
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from fcl @ e5efcc41b57b2d0da3bf183480f1298a6d531f44 (0.7.0-17-ge5efcc4):
//   include/fcl/geometry/bvh/detail/BV_fitter-inl.h (FitImpl<S, OBB<S>>::run)
//   include/fcl/math/geometry-inl.h (getCovariance, eigen_old, axisFromEigen,
//     getExtentAndCenter, getExtentAndCenter_pointcloud)

//! The oriented-bounding-box fit `fcl::detail::BVFitter<OBB<double>>::fit`
//! performs for one BVH node's primitives: a covariance-matrix
//! eigendecomposition sets the box's axes, then a projection pass over the
//! same points sets its center and half-extent. See [`fit_obb`]'s doc for
//! exactly which upstream fitter this is (and is not) -- fcl has two
//! independently selectable OBB fitters, and only one is on the path this
//! crate calls.

use parry3d_f64::math::{Matrix, Vector};

/// FCL `detail::FitImpl<S, OBB<S>>::run` (`BV_fitter-inl.h:301-327`), the
/// body of `fcl::detail::BVFitter<OBB<double>>::fit`: covariance ->
/// `eigen_old` -> `axisFromEigen` -> `getExtentAndCenter`, with no special
/// case on the point count. Every `BVFitter<OBB<double>>` call site --
/// point-cloud and triangle-mesh leaves alike -- reaches this same pipeline;
/// see [`covariance`]'s doc for why the triangle-mesh branch needs no
/// separate signature here.
///
/// Deliberately NOT ported: `detail::Fitter<S, OBB<S>>::fit`'s `fit1`/
/// `fit2`/`fit3`/`fit6` switch on point count (`utility-inl.h:554-578`), and
/// `generateCoordinateSystem` (`geometry-inl.h:643-650`) that `fit2`/`fit3`
/// call. `Fitter<S, OBB<S>>` and `BVFitter<OBB<S>>` are two separate
/// upstream fitters; only the latter (this function) is on the path
/// `ObbTree::build_node` calls.
///
/// `axis`'s columns are the box's three principal axes (`bv.axis.col(i)`
/// upstream, longest-extent axis first); `center`/`extent` are `OBB::To`/
/// `OBB::extent`.
pub(crate) fn fit_obb(points: &[Vector]) -> (Matrix, Vector, Vector) {
    let (eigenvalues, eigenvectors) = eigen_old(covariance(points));
    let axis = axis_from_eigen(eigenvectors, eigenvalues);
    let (center, extent) = extent_and_center(points, axis);
    (axis, center, extent)
}

/// FCL `getCovariance`, point-cloud branch (`geometry-inl.h:1333-1421`
/// overall; the `ts == nullptr`/`indices == nullptr`/`ps2 == nullptr` path
/// accumulates at `:1385-1408`, then `:1410-1421` divides by the point
/// count). The triangle-mesh branch (`:1349-1382`) accumulates the same
/// `S1`/`S2` sums over each triangle's three vertices in turn, so summed
/// over a flattened triangle-vertex list it computes exactly this branch's
/// arithmetic -- a plain point slice is enough for [`fit_obb`] to cover both
/// upstream primitive kinds.
fn covariance(points: &[Vector]) -> [[f64; 3]; 3] {
    let mut sum = Vector::ZERO;
    let mut sum_sq = [[0.0_f64; 3]; 3]; // sum_sq[i][j] accumulates sum(p[i] * p[j])

    for p in points {
        sum += *p;
        sum_sq[0][0] += p.x * p.x;
        sum_sq[1][1] += p.y * p.y;
        sum_sq[2][2] += p.z * p.z;
        sum_sq[0][1] += p.x * p.y;
        sum_sq[0][2] += p.x * p.z;
        sum_sq[1][2] += p.y * p.z;
    }

    let n = points.len() as f64;
    let m00 = sum_sq[0][0] - sum.x * sum.x / n;
    let m11 = sum_sq[1][1] - sum.y * sum.y / n;
    let m22 = sum_sq[2][2] - sum.z * sum.z / n;
    let m01 = sum_sq[0][1] - sum.x * sum.y / n;
    let m12 = sum_sq[1][2] - sum.y * sum.z / n;
    let m02 = sum_sq[0][2] - sum.x * sum.z / n;

    [[m00, m01, m02], [m01, m11, m12], [m02, m12, m22]]
}

/// FCL `eigen_old` (`geometry-inl.h:475-558`): classical cyclic Jacobi
/// eigenvalue iteration for a symmetric 3x3 matrix -- upstream's own hand-
/// rolled routine (as opposed to `eigen()`'s `Eigen::SelfAdjointEigenSolver`
/// a few lines above it, which `fitn`/`FitImpl` deliberately do not call).
/// Ported with the same `mat[row][col]`/`eigenvectors[row][col]` indexing
/// upstream's plain C arrays use (`R(ip, iq)`, `v[j][ip]`) rather than
/// through `Matrix`'s column-major addressing, since the algorithm only
/// ever touches the strict upper triangle (`row < col`).
///
/// Returns `(eigenvalues, eigenvectors)` where `eigenvectors[component][k]`
/// is the `component`-th coordinate of the eigenvector for `eigenvalues[k]`
/// -- upstream's accumulator `v` (`:485,543`), whose column `k` (over `j`)
/// is that eigenvector. Upstream then copies `v` into its output
/// `Eigen::Matrix3` `vout` with `vout.row(k)` set to that same eigenvector
/// (`:502-504`), i.e. eigenvectors as ROWS of `vout` -- an intentional
/// transpose of `v`'s column-per-eigenvector layout. [`axis_from_eigen`]
/// (`axisFromEigen`, the only caller) reads exactly `vout.row(k)`
/// (`:595-596`), so this port has it read `eigenvectors[..][k]` from `v`
/// directly instead of materializing that transposed matrix.
///
/// Deviation: upstream leaves `vout`/`dout` untouched if the sweep does not
/// converge within 50 iterations -- undefined behaviour in C++, since
/// `Eigen::Matrix3d`'s default constructor leaves its storage uninitialized
/// (`:555-557`). Rust has no such escape hatch, so this port falls through
/// to returning the last sweep's estimate instead. A real symmetric 3x3 has
/// only 3 off-diagonal pairs and converges to bit-exact `off_diag_sum ==
/// 0.0` in a handful of sweeps in practice, so this path is not expected to
/// be exercised.
fn eigen_old(m: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut mat = m;
    let mut eigenvectors = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut eigenvalues = [mat[0][0], mat[1][1], mat[2][2]];
    let mut base = eigenvalues;
    let mut shift = [0.0_f64; 3];

    for sweep in 0..50 {
        let mut off_diag_sum = 0.0;
        for (p, row) in mat.iter().enumerate() {
            for value in row.iter().skip(p + 1) {
                off_diag_sum += value.abs();
            }
        }
        if off_diag_sum == 0.0 {
            return (eigenvalues, eigenvectors);
        }

        let threshold = if sweep < 3 {
            0.2 * off_diag_sum / 9.0
        } else {
            0.0
        };

        for p in 0..3 {
            for q in (p + 1)..3 {
                let scaled = 100.0 * mat[p][q].abs();
                if sweep > 3
                    && eigenvalues[p].abs() + scaled == eigenvalues[p].abs()
                    && eigenvalues[q].abs() + scaled == eigenvalues[q].abs()
                {
                    mat[p][q] = 0.0;
                    continue;
                }
                if mat[p][q].abs() <= threshold {
                    continue;
                }

                let gap = eigenvalues[q] - eigenvalues[p];
                let tan_theta = if gap.abs() + scaled == gap.abs() {
                    mat[p][q] / gap
                } else {
                    let theta = 0.5 * gap / mat[p][q];
                    let mut tan_theta = 1.0 / (theta.abs() + (1.0 + theta * theta).sqrt());
                    if theta < 0.0 {
                        tan_theta = -tan_theta;
                    }
                    tan_theta
                };
                let cos_theta = 1.0 / (1.0 + tan_theta * tan_theta).sqrt();
                let sin_theta = tan_theta * cos_theta;
                let tau = sin_theta / (1.0 + cos_theta);
                let delta = tan_theta * mat[p][q];

                shift[p] -= delta;
                shift[q] += delta;
                eigenvalues[p] -= delta;
                eigenvalues[q] += delta;
                mat[p][q] = 0.0;

                // The four cell-pair sweeps, `:541-544`. Upstream indexes
                // `R`/`v` freely because C lets it; here each pair is taken
                // as two disjoint borrows, which is the same access spelled
                // so the compiler can see the two cells are never the same
                // one. `p < q` throughout, which is what makes every split
                // below valid.

                // `for(j = 0; j < ip; ++j) { R(j, ip), R(j, iq) }`
                for row in mat.iter_mut().take(p) {
                    let (left, right) = row.split_at_mut(q);
                    jacobi_rotate(&mut left[p], &mut right[0], sin_theta, tau);
                }
                // `for(j = ip + 1; j < iq; ++j) { R(ip, j), R(j, iq) }` --
                // the only one of the four that touches two different rows
                // at two different columns.
                let (through_p, past_p) = mat.split_at_mut(p + 1);
                let row_p = &mut through_p[p];
                for (offset, row) in past_p.iter_mut().take(q - p - 1).enumerate() {
                    jacobi_rotate(&mut row_p[p + 1 + offset], &mut row[q], sin_theta, tau);
                }
                // `for(j = iq + 1; j < n; ++j) { R(ip, j), R(iq, j) }`
                let (through_q, past_q) = mat.split_at_mut(q);
                let (row_p, row_q) = (&mut through_q[p], &mut past_q[0]);
                for (at_p, at_q) in row_p.iter_mut().zip(row_q.iter_mut()).skip(q + 1) {
                    jacobi_rotate(at_p, at_q, sin_theta, tau);
                }
                // `for(j = 0; j < n; ++j) { v[j][ip], v[j][iq] }`
                for row in &mut eigenvectors {
                    let (left, right) = row.split_at_mut(q);
                    jacobi_rotate(&mut left[p], &mut right[0], sin_theta, tau);
                }
            }
        }

        for p in 0..3 {
            base[p] += shift[p];
            eigenvalues[p] = base[p];
            shift[p] = 0.0;
        }
    }

    (eigenvalues, eigenvectors)
}

/// One Jacobi rotation applied to a pair of cells: upstream's
/// `g = X; h = Y; X = g - s*(h + g*tau); Y = h + s*(g - h*tau);`
/// (`geometry-inl.h:541-544`), which appears verbatim four times per
/// rotation over four different pairs of cells.
///
/// The two cells are always distinct -- that is what `eigen_old`'s splits
/// establish before calling this -- so taking them as two `&mut` is the
/// same access upstream performs, not a restriction on it.
fn jacobi_rotate(x: &mut f64, y: &mut f64, sin_theta: f64, tau: f64) {
    let (g, h) = (*x, *y);
    *x = g - sin_theta * (h + g * tau);
    *y = h + sin_theta * (g - h * tau);
}

/// FCL `axisFromEigen` (`geometry-inl.h:561-598`): order `eigen_old`'s three
/// eigenpairs largest-to-smallest and assign the axis columns -- the largest
/// and middle eigenvector directly (`axis.col(0)`/`axis.col(1)`,
/// `:595-596`), the smallest by cross product rather than trusting its sign
/// (`axis.col(2)`, `:597`), matching upstream exactly.
fn axis_from_eigen(eigenvectors: [[f64; 3]; 3], eigenvalues: [f64; 3]) -> Matrix {
    let eigenvector =
        |k: usize| Vector::new(eigenvectors[0][k], eigenvectors[1][k], eigenvectors[2][k]);

    let (mid, max) = mid_and_max_eigenvalue_indices(eigenvalues);
    let col0 = eigenvector(max);
    let col1 = eigenvector(mid);
    let col2 = col0.cross(col1);
    Matrix::from_cols(col0, col1, col2)
}

/// The largest- and middle-magnitude eigenvalue's indices, upstream's
/// `max`/`mid` (`geometry-inl.h:567-593`). The third (`min`) index upstream
/// derives along the way is never read past that point either -- `axis.col(2)`
/// comes from a cross product, not `eigenV.row(min)` -- so this port only
/// returns what [`axis_from_eigen`] actually uses.
fn mid_and_max_eigenvalue_indices(eigenvalues: [f64; 3]) -> (usize, usize) {
    let (min, max) = if eigenvalues[0] > eigenvalues[1] {
        (1, 0)
    } else {
        (0, 1)
    };

    if eigenvalues[2] < eigenvalues[min] {
        (min, max)
    } else if eigenvalues[2] > eigenvalues[max] {
        (max, 2)
    } else {
        (2, max)
    }
}

/// FCL `getExtentAndCenter` -> `detail::getExtentAndCenter_pointcloud`
/// (`geometry-inl.h:1314-1330` dispatch, `:227-287` the `ps2 == nullptr`/
/// `indices == nullptr` point-cloud implementation this port needs): project
/// every point onto the box's three axes, take the per-axis min/max of that
/// projection (`:246-263`), and report the half-extent and world-space
/// center those bounds imply (`:284-286`).
fn extent_and_center(points: &[Vector], axis: Matrix) -> (Vector, Vector) {
    let mut min_coord = [f64::MAX; 3];
    let mut max_coord = [-f64::MAX; 3];

    for p in points {
        let proj = [
            axis.col(0).dot(*p),
            axis.col(1).dot(*p),
            axis.col(2).dot(*p),
        ];
        for j in 0..3 {
            if proj[j] > max_coord[j] {
                max_coord[j] = proj[j];
            }
            if proj[j] < min_coord[j] {
                min_coord[j] = proj[j];
            }
        }
    }

    let mid = Vector::new(
        (max_coord[0] + min_coord[0]) * 0.5,
        (max_coord[1] + min_coord[1]) * 0.5,
        (max_coord[2] + min_coord[2]) * 0.5,
    );
    let extent = Vector::new(
        (max_coord[0] - min_coord[0]) * 0.5,
        (max_coord[1] - min_coord[1]) * 0.5,
        (max_coord[2] - min_coord[2]) * 0.5,
    );
    (axis * mid, extent)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn assert_orthonormal(axis: Matrix) {
        for i in 0..3 {
            assert_relative_eq!(axis.col(i).length(), 1.0, epsilon = 1e-8);
        }
        assert_relative_eq!(axis.col(0).dot(axis.col(1)), 0.0, epsilon = 1e-8);
        assert_relative_eq!(axis.col(0).dot(axis.col(2)), 0.0, epsilon = 1e-8);
        assert_relative_eq!(axis.col(1).dot(axis.col(2)), 0.0, epsilon = 1e-8);
        assert_relative_eq!(axis.determinant(), 1.0, epsilon = 1e-8);
    }

    fn contains(axis: Matrix, center: Vector, extent: Vector, p: Vector) -> bool {
        let local = p - center;
        let proj = [
            axis.col(0).dot(local),
            axis.col(1).dot(local),
            axis.col(2).dot(local),
        ];
        let extent = [extent.x, extent.y, extent.z];
        (0..3).all(|i| proj[i].abs() <= extent[i] + 1e-9)
    }

    fn random_point(rng: &mut impl rand::RngExt, span: f64) -> Vector {
        Vector::new(
            rng.random_range(-span..span),
            rng.random_range(-span..span),
            rng.random_range(-span..span),
        )
    }

    #[test]
    fn unit_cube_corners_fit_the_axis_aligned_box() {
        let points: Vec<Vector> = (0u8..8)
            .map(|i| {
                Vector::new(
                    if i & 1 != 0 { 1.0 } else { 0.0 },
                    if i & 2 != 0 { 1.0 } else { 0.0 },
                    if i & 4 != 0 { 1.0 } else { 0.0 },
                )
            })
            .collect();

        let (axis, center, extent) = fit_obb(&points);

        assert_orthonormal(axis);
        assert_relative_eq!(center.x, 0.5, epsilon = 1e-9);
        assert_relative_eq!(center.y, 0.5, epsilon = 1e-9);
        assert_relative_eq!(center.z, 0.5, epsilon = 1e-9);
        assert_relative_eq!(extent.x, 0.5, epsilon = 1e-9);
        assert_relative_eq!(extent.y, 0.5, epsilon = 1e-9);
        assert_relative_eq!(extent.z, 0.5, epsilon = 1e-9);

        for p in points {
            assert!(contains(axis, center, extent, p));
        }
    }

    /// The case this port exists for: a box fitted to a long, thin, exactly
    /// collinear point cloud must itself be long and thin, not merely a
    /// loose enclosure. Checked scale-invariantly (the short/long extent
    /// ratio, not just an absolute epsilon) since "thin" is a relative claim.
    #[test]
    fn points_on_a_long_thin_diagonal_segment_fit_a_thin_box() {
        let direction = Vector::new(1.0, 2.0, -2.0).normalize();
        let points: Vec<Vector> = (-10..=10)
            .map(|k| direction * (100.0 * f64::from(k)))
            .collect();

        let (axis, center, extent) = fit_obb(&points);

        assert_orthonormal(axis);
        assert_relative_eq!(extent.x, 1000.0, epsilon = 1e-6);
        assert!(extent.y / extent.x < 1e-9, "extent = {extent:?}");
        assert!(extent.z / extent.x < 1e-9, "extent = {extent:?}");
        assert_relative_eq!(axis.col(0).dot(direction).abs(), 1.0, epsilon = 1e-9);
        assert_relative_eq!(center.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(center.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(center.z, 0.0, epsilon = 1e-6);

        for p in points {
            assert!(contains(axis, center, extent, p));
        }
    }

    #[test]
    fn a_triangle_fits_a_degenerate_box_in_its_plane() {
        let points = [
            Vector::new(0.0, 0.0, 5.0),
            Vector::new(4.0, 0.0, 5.0),
            Vector::new(0.0, 3.0, 5.0),
        ];

        let (axis, center, extent) = fit_obb(&points);

        assert_orthonormal(axis);
        let extents = [extent.x, extent.y, extent.z];
        let degenerate = extents.iter().filter(|e| e.abs() < 1e-9).count();
        assert_eq!(
            degenerate, 1,
            "expected exactly one ~0 extent, got {extents:?}"
        );

        for p in points {
            assert!(contains(axis, center, extent, p));
        }
    }

    #[test]
    fn a_collinear_triangle_fits_a_degenerate_box_along_the_line() {
        let points = [
            Vector::new(0.0, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(2.0, 0.0, 0.0),
        ];

        let (axis, center, extent) = fit_obb(&points);

        assert_orthonormal(axis);
        assert_relative_eq!(extent.x, 1.0, epsilon = 1e-9);
        assert!(extent.y.abs() < 1e-9);
        assert!(extent.z.abs() < 1e-9);
        assert_relative_eq!(center.x, 1.0, epsilon = 1e-9);
        assert_relative_eq!(center.y, 0.0, epsilon = 1e-9);
        assert_relative_eq!(center.z, 0.0, epsilon = 1e-9);

        for p in points {
            assert!(contains(axis, center, extent, p));
        }
    }

    #[test]
    fn a_single_point_fits_a_zero_extent_box_centered_on_it() {
        let point = Vector::new(3.0, -2.0, 7.5);
        let (axis, center, extent) = fit_obb(&[point]);

        assert_orthonormal(axis);
        assert_relative_eq!(center.x, point.x, epsilon = 1e-9);
        assert_relative_eq!(center.y, point.y, epsilon = 1e-9);
        assert_relative_eq!(center.z, point.z, epsilon = 1e-9);
        assert!(extent.x.abs() < 1e-9);
        assert!(extent.y.abs() < 1e-9);
        assert!(extent.z.abs() < 1e-9);
        assert!(contains(axis, center, extent, point));
    }

    #[test]
    fn two_points_fit_a_thin_box_along_their_difference() {
        let p1 = Vector::new(1.0, 2.0, 3.0);
        let p2 = Vector::new(5.0, -1.0, 9.0);
        let (axis, center, extent) = fit_obb(&[p1, p2]);

        assert_orthonormal(axis);

        let expected_center = (p1 + p2) * 0.5;
        assert_relative_eq!(center.x, expected_center.x, epsilon = 1e-9);
        assert_relative_eq!(center.y, expected_center.y, epsilon = 1e-9);
        assert_relative_eq!(center.z, expected_center.z, epsilon = 1e-9);

        let half_dist = (p1 - p2).length() * 0.5;
        assert_relative_eq!(extent.x, half_dist, epsilon = 1e-9);
        assert!(extent.y.abs() < 1e-9);
        assert!(extent.z.abs() < 1e-9);

        assert!(contains(axis, center, extent, p1));
        assert!(contains(axis, center, extent, p2));
    }

    #[test]
    fn axis_is_orthonormal_with_positive_determinant_over_random_point_clouds() {
        use rand::RngExt as _;
        use rand::SeedableRng as _;

        let mut rng = rand_chacha::ChaCha8Rng::seed_from_u64(20260809);

        for _ in 0..500 {
            let count = rng.random_range(1usize..30);
            let points: Vec<Vector> = (0..count).map(|_| random_point(&mut rng, 10.0)).collect();
            let (axis, _, _) = fit_obb(&points);
            assert_orthonormal(axis);
        }
    }

    /// The invariant the rest of the OBB tree depends on: every fitted box
    /// must contain every point it was fitted from.
    #[test]
    fn every_fitted_box_contains_all_its_input_points() {
        use rand::RngExt as _;
        use rand::SeedableRng as _;

        let mut rng = rand_chacha::ChaCha8Rng::seed_from_u64(20260809);

        for _ in 0..500 {
            let count = rng.random_range(1usize..30);
            let points: Vec<Vector> = (0..count).map(|_| random_point(&mut rng, 10.0)).collect();
            let (axis, center, extent) = fit_obb(&points);
            for p in points {
                assert!(contains(axis, center, extent, p));
            }
        }
    }
    /// The three axes must be *the covariance's eigenvectors, largest
    /// eigenvalue first* -- not merely some orthonormal frame.
    ///
    /// This is the one property with no cheaper proxy. Reading `eigen_old`'s
    /// output as column-major eigenvectors instead of row-major
    /// (`geometry-inl.h:503-505` writes `vout.col(k)` from `v[k][*]`, so
    /// `vout`'s *rows* are the eigenvectors and `axisFromEigen` takes
    /// `eigenV.row(max)`) yields a frame that is still orthonormal and still
    /// right-handed, and `getExtentAndCenter` fits the extents to whatever
    /// axes it is handed -- so orthonormality, containment and tightness all
    /// pass on the transposed read. The only thing it loses is the one thing
    /// the fit exists for.
    ///
    /// Checked as an eigenvector relation against the scatter matrix rather
    /// than by re-deriving the axes: `M v = lambda v` for each axis, with the
    /// Rayleigh quotients in descending order. `axis.col(2)` is
    /// `col(0).cross(col(1))` and so may be the negation of the third
    /// eigenvector; that is still an eigenvector, and the relation still
    /// holds.
    #[test]
    fn the_axes_are_the_scatter_matrixs_eigenvectors_largest_eigenvalue_first() {
        use rand::RngExt as _;
        use rand::SeedableRng as _;

        let mut rng = rand_chacha::ChaCha8Rng::seed_from_u64(20260809);
        let mut anisotropic = 0;

        for _ in 0..500 {
            // Deliberately anisotropic: an isotropic cloud has no
            // well-separated eigenvectors, so it cannot tell a correct fit
            // from a transposed one.
            let stretch = Vector::new(
                rng.random_range(0.5..4.0),
                rng.random_range(0.1..1.0),
                rng.random_range(0.02..0.3),
            );
            let n = rng.random_range(6..40);
            let points: Vec<Vector> = (0..n)
                .map(|_| {
                    Vector::new(
                        rng.random_range(-1.0..1.0) * stretch.x,
                        rng.random_range(-1.0..1.0) * stretch.y,
                        rng.random_range(-1.0..1.0) * stretch.z,
                    )
                })
                .collect();

            let (axis, center, extent) = fit_obb(&points);

            // `getCovariance` (`geometry-inl.h:1411-1421`): the *scatter*
            // matrix, `S2 - S1 S1^T / n`, undivided. Scale does not move
            // eigenvectors, and this is what upstream decomposes.
            let s1 = points.iter().fold(Vector::ZERO, |a, p| a + *p);
            let mut m = [[0.0f64; 3]; 3];
            for p in &points {
                for i in 0..3 {
                    for j in 0..3 {
                        m[i][j] += p[i] * p[j];
                    }
                }
            }
            let np = points.len() as f64;
            for i in 0..3 {
                for j in 0..3 {
                    m[i][j] -= s1[i] * s1[j] / np;
                }
            }
            let mul = |v: Vector| {
                Vector::new(
                    m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                    m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                    m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
                )
            };

            let scale = (0..3).fold(0.0f64, |a, i| a.max(m[i][i].abs())).max(1.0);
            let mut lambdas = [0.0; 3];
            for (k, lambda_k) in lambdas.iter_mut().enumerate() {
                let v = axis.col(k);
                assert!(
                    (v.length() - 1.0).abs() < 1e-9,
                    "axis {k} is not a unit vector"
                );
                let mv = mul(v);
                let lambda = v.dot(mv);
                *lambda_k = lambda;
                assert!(
                    (mv - v * lambda).length() < 1e-7 * scale,
                    "axis {k} is not an eigenvector of the scatter matrix"
                );
            }
            assert!(
                lambdas[0] >= lambdas[1] - 1e-9 * scale && lambdas[1] >= lambdas[2] - 1e-9 * scale,
                "eigenvalues are not in descending order: {lambdas:?}"
            );
            if lambdas[0] > 4.0 * lambdas[2] + 1e-12 {
                anisotropic += 1;
            }

            // The extents have to be exact, not merely sufficient: a fit that
            // padded them would pass containment while giving away the
            // tightness this whole hierarchy is for.
            for k in 0..3 {
                let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
                for p in &points {
                    let proj = axis.col(k).dot(*p - center);
                    lo = lo.min(proj);
                    hi = hi.max(proj);
                }
                assert!(
                    (hi - extent[k]).abs() < 1e-9 && (lo + extent[k]).abs() < 1e-9,
                    "axis {k}: points span [{lo}, {hi}] but the extent is {}",
                    extent[k]
                );
            }
        }

        // Otherwise the ordering assertion held over clouds too round for the
        // order to mean anything.
        assert!(
            anisotropic > 300,
            "only {anisotropic} clouds had a well-separated principal axis"
        );
    }
}
