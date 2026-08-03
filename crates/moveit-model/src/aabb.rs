// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/aabb.hpp
//   moveit_core/robot_model/include/moveit/robot_model/aabb.h
//   moveit_core/robot_model/src/aabb.cpp
//
// This is `moveit::core::AABB`, not part of `geometric_shapes` — it lives
// here rather than in `moveit-geometry` for the same reason it lives in
// `moveit_core/robot_model` upstream: it exists solely to compute
// `LinkModel::centered_bounding_box_offset_`.

use moveit_geometry::{Isometry3, Vector3};

/// An axis-aligned bounding box, accumulated by repeatedly extending it with
/// points or transformed boxes. Upstream `moveit::core::AABB`, a thin
/// subclass of `Eigen::AlignedBox3d`.
///
/// # Matching upstream's empty-box constant exactly
///
/// A never-[`extend`](Aabb::extend)ed [`Aabb`] must match
/// `Eigen::AlignedBox3d`'s default-constructed "empty" state exactly:
/// `min = f64::MAX`, `max = f64::MIN` (Eigen's `setEmpty()` —
/// `m_min.setConstant(ScalarTraits::highest())`,
/// `m_max.setConstant(ScalarTraits::lowest())`, verified against Eigen's own
/// `AlignedBox.h`), **not** `+inf`/`-inf`. The two read the same in every
/// other respect ([`Aabb::extend`] immediately overwrites both to any real
/// point), but they disagree on [`Aabb::center`]: `(MAX + MIN) / 2` is an
/// exact `0.0` (the two huge magnitudes cancel), while `(inf - inf) / 2`
/// would be `NaN`. The oracle confirms zero: every `pr2` link with no
/// `<collision>` element at all reports
/// `centered_bounding_box_offset: [0.0, 0.0, 0.0]`, not `null`/`NaN` — see
/// [`crate::link_model::LinkModel`]'s doc comment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    min: Vector3,
    max: Vector3,
}

impl Default for Aabb {
    fn default() -> Self {
        Self {
            min: Vector3::from_element(f64::MAX),
            max: Vector3::from_element(f64::MIN),
        }
    }
}

impl Aabb {
    /// Extend the box to include `point`. Upstream `Eigen::AlignedBox3d::extend`.
    pub fn extend(&mut self, point: Vector3) {
        self.min = self.min.zip_map(&point, f64::min);
        self.max = self.max.zip_map(&point, f64::max);
    }

    /// Extend the box with a box of size `extents` (centered at its own
    /// local origin) placed at `transform`. Upstream
    /// `AABB::extendWithTransformedBox`; the derivation of the two corner
    /// points below is in that function's own comment (a component-wise
    /// bound on a rotated box, adapted from FCL's `computeBV<AABB, Box>`).
    pub fn extend_with_transformed_box(&mut self, transform: &Isometry3, extents: Vector3) {
        let r = transform.rotation.to_rotation_matrix();
        let r = r.matrix();
        let t = transform.translation.vector;

        let x_range = 0.5
            * ((r[(0, 0)] * extents[0]).abs()
                + (r[(0, 1)] * extents[1]).abs()
                + (r[(0, 2)] * extents[2]).abs());
        let y_range = 0.5
            * ((r[(1, 0)] * extents[0]).abs()
                + (r[(1, 1)] * extents[1]).abs()
                + (r[(1, 2)] * extents[2]).abs());
        let z_range = 0.5
            * ((r[(2, 0)] * extents[0]).abs()
                + (r[(2, 1)] * extents[1]).abs()
                + (r[(2, 2)] * extents[2]).abs());

        let delta = Vector3::new(x_range, y_range, z_range);
        self.extend(t + delta);
        self.extend(t - delta);
    }

    /// The center of the box. Exactly `0.0` in every component if never
    /// [`extend`](Aabb::extend)ed — see the doc comment on [`Aabb`]. Upstream
    /// `Eigen::AlignedBox3d::center`.
    pub fn center(&self) -> Vector3 {
        (self.min + self.max) / 2.0
    }

    /// The box's size along each axis. Upstream `Eigen::AlignedBox3d::sizes`.
    /// Only [`Aabb::center`] is needed outside tests (it is what
    /// [`crate::link_model::LinkModel::set_geometry`] consumes); this stays
    /// test-only rather than a speculative public accessor.
    #[cfg(test)]
    pub(crate) fn sizes(&self) -> Vector3 {
        self.max - self.min
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The exact-cancellation boundary this type exists to reproduce: Eigen's
    /// empty-box constant is `highest()`/`lowest()` (huge finite values), not
    /// `+inf`/`-inf`, so their average is `0.0`, not `NaN`.
    #[test]
    fn never_extended_box_has_zero_center() {
        let aabb = Aabb::default();
        assert_eq!(aabb.center(), Vector3::zeros());
    }

    #[test]
    fn extend_with_single_point_centers_on_that_point_with_zero_size() {
        let mut aabb = Aabb::default();
        aabb.extend(Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(aabb.center(), Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(aabb.sizes(), Vector3::zeros());
    }

    #[test]
    fn extend_with_identity_transformed_box_matches_the_box_itself() {
        let mut aabb = Aabb::default();
        aabb.extend_with_transformed_box(&Isometry3::identity(), Vector3::new(2.0, 4.0, 6.0));
        assert_eq!(aabb.center(), Vector3::zeros());
        assert_eq!(aabb.sizes(), Vector3::new(2.0, 4.0, 6.0));
    }

    #[test]
    fn extend_with_translated_box_offsets_the_center_by_the_translation() {
        let mut aabb = Aabb::default();
        let transform = Isometry3::translation(1.0, 0.0, 0.0);
        aabb.extend_with_transformed_box(&transform, Vector3::new(2.0, 2.0, 2.0));
        assert_eq!(aabb.center(), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(aabb.sizes(), Vector3::new(2.0, 2.0, 2.0));
    }

    #[test]
    fn extend_with_a_quarter_turn_swaps_the_footprint_axes() {
        // A 2x4x1 box rotated 90 degrees about Z should occupy a 4x2x1
        // footprint -- this is the case the "rotated OBB -> AABB" derivation
        // exists for; a naive axis-aligned extend would get it wrong.
        let mut aabb = Aabb::default();
        let transform = Isometry3::rotation(nalgebra::Vector3::new(
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_2,
        ));
        aabb.extend_with_transformed_box(&transform, Vector3::new(2.0, 4.0, 1.0));
        let sizes = aabb.sizes();
        assert!((sizes.x - 4.0).abs() < 1e-9, "{sizes:?}");
        assert!((sizes.y - 2.0).abs() < 1e-9, "{sizes:?}");
        assert!((sizes.z - 1.0).abs() < 1e-9, "{sizes:?}");
    }
}
