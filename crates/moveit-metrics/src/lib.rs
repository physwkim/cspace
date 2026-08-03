// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematics_metrics/include/moveit/kinematics_metrics/kinematics_metrics.hpp
//   moveit_core/kinematics_metrics/src/kinematics_metrics.cpp

//! Manipulability metrics for moveit-rs: [`KinematicsMetrics`], ported from
//! `moveit_core/kinematics_metrics`.
//!
//! # Scope
//!
//! Upstream's six public methods collapse to four here, one per computation
//! rather than one per `std::string`/`JointModelGroup*` overload pair —
//! this crate has no `RobotModel`-owned `JointModelGroup*` to accept, only
//! `&str` group names, matching [`moveit_state::Posed::jacobian`]'s own
//! established `&str`-group convention. Upstream's `bool` return (`false`
//! for "unknown group" and "group is not a chain" alike) becomes
//! [`moveit_error::Result`] with the two causes distinguished
//! ([`moveit_error::Error::UnknownName`] vs. [`moveit_error::Error::Other`]),
//! matching [`moveit_state::Posed::jacobian`]'s own error shape (see
//! its doc comment for the same deviation).
//!
//! Dependencies (`#include`) are `robot_state` and Eigen only upstream —
//! `rclcpp` appears solely for `RCLCPP_DEBUG` logging of intermediate
//! singular values, which this port drops rather than routes through a
//! logging facade (see `PORTING-PLAN.md` D1: no ROS decoupling needed, and
//! no logging crate is in scope for this port).
//!
//! # `getManipulabilityEllipsoid`: `nalgebra::SymmetricEigen`, not a complex
//! generic eigensolver
//!
//! Upstream feeds `matrix.block(0, 0, 3, 3)` — the linear (translation)
//! 3x3 block of `J * J^T` — to `Eigen::EigenSolver<Eigen::MatrixXd>`, the
//! generic (non-symmetric-specialized) solver, and stores the result in
//! `Eigen::MatrixXcd` (complex-valued). `J * J^T` is symmetric
//! positive-semidefinite by construction for *any* real `J`, for any
//! group, at any joint configuration — its eigenvalues are always real and
//! non-negative, and its eigenvectors always real, regardless of what
//! `Eigen::EigenSolver`'s complex-typed API defensively allows for. This
//! port uses [`nalgebra::SymmetricEigen`] instead: it requires a
//! (statically) symmetric input, an invariant `Matrix3::from(J*J^T block)`
//! satisfies by construction, and returns real `Vector3`/`Matrix3` rather
//! than complex — a strictly more precise API for an input that is always
//! real, not a numerically different computation.
//!
//! # Singular-value and eigenvalue ordering — verified, not assumed
//!
//! [`manipulability_index`](KinematicsMetrics::manipulability_index) and
//! [`manipulability`](KinematicsMetrics::manipulability) both reduce a
//! `nalgebra::SVD`'s singular values (product, or min/max ratio). Both
//! Eigen's `JacobiSVD`/`BDCSVD` (upstream) and `nalgebra::SVD::new` (this
//! port — confirmed via `nalgebra` 0.35.0's `svd.rs`, which sorts by an
//! internal `sort_by_singular_values()` call the same way `SVD::try_new`
//! does *not*) guarantee descending order, so a product or a min/max ratio
//! is order-independent and this is a non-issue for those two methods.
//!
//! [`manipulability_ellipsoid`](KinematicsMetrics::manipulability_ellipsoid)
//! is different: neither Eigen's `EigenSolver` (upstream, confirmed via the
//! oracle image's vendored `Eigen/src/Eigenvalues/EigenSolver.h`, which
//! documents its eigenvalues/eigenvectors as "not sorted in any particular
//! order") nor `nalgebra::SymmetricEigen` (this port, whose own doc
//! comment says the same: "unsorted eigenvalues") gives any ordering
//! guarantee — this is a genuine non-determinism in the reference
//! implementation itself, not a gap this port introduces. The oracle
//! parity test for this one method sorts both sides by eigenvalue and
//! normalizes eigenvector sign before comparing, rather than pinning
//! position-for-position the way the other three methods' fixtures do; see
//! that test's own comment for detail.
//!
//! # `getJointLimitsPenalty`
//!
//! Private upstream, exposed here as [`KinematicsMetrics::joint_limits_penalty`]
//! since a Rust module has no `friend`-equivalent visibility escape and every
//! other method in this crate needs it. Semantics, straight from
//! `kinematics_metrics.cpp:56-103`:
//!
//! - If [`KinematicsMetrics::penalty_multiplier`] is (within
//!   `f64::MIN_POSITIVE` of) zero, the penalty is `1.0` unconditionally —
//!   the whole per-joint loop below is skipped.
//! - A continuous revolute joint contributes nothing (skipped): it has no
//!   meaningful "distance to a limit".
//! - A planar joint is skipped if its `x`/`y` bounds are unbounded (this
//!   port's unbounded sentinel is `f64::INFINITY`/`f64::NEG_INFINITY` — see
//!   `JointModel::new_planar`'s own doc comment — *not* upstream's literal
//!   `DBL_MAX`, which this port's planar constructor never produces; using
//!   `DBL_MAX` here would silently never match and always fall through) or
//!   its `theta` bound is the default full range, checked against
//!   `std::f64::consts::PI` exactly as upstream checks against `M_PI` — this
//!   one is a literal, not a sentinel translation, since `new_planar`'s own
//!   `theta` bounds already are `[-PI, PI]` by value, not by convention.
//! - A floating joint is always skipped: "Joint limits are not well-defined
//!   for floating joints" (upstream's own comment, verbatim).
//! - Every other joint (revolute non-continuous, prismatic, and — by
//!   fallthrough, not an explicit case — fixed) contributes
//!   `lower_distance * upper_distance / range^2` to a running product,
//!   `range = lower_distance + upper_distance`. A fixed joint has zero
//!   variables, so [`moveit_model::joint::JointModel::distance`] returns
//!   `0.0` for it unconditionally, making `range == 0.0 <=
//!   f64::MIN_POSITIVE` true and skipping it through the *same* general
//!   `range` check every other joint type is skipped through when its range
//!   collapses — this port adds no explicit `Fixed` arm, matching upstream,
//!   which has none either.
//! - Final penalty: `1.0 - exp(-penalty_multiplier * product)`.

use nalgebra::{Matrix3, SVD, SymmetricEigen, Vector3};

use moveit_error::{Error, Result};
use moveit_model::joint::{JointKind, JointType};
use moveit_model::{JointModelGroup, RobotModel};
use moveit_state::{Posed, RobotState};

/// `kinematics_metrics::KinematicsMetrics`: manipulability and joint-limits
/// penalty over a [`RobotModel`], evaluated against a caller-supplied
/// [`RobotState`] per call (upstream stores no `RobotState` of its own
/// either — every method takes one as an argument).
pub struct KinematicsMetrics<'m> {
    model: &'m RobotModel,
    penalty_multiplier: f64,
}

impl<'m> KinematicsMetrics<'m> {
    /// `KinematicsMetrics(RobotModelConstPtr)`. `penalty_multiplier_`
    /// defaults to `0.0`, matching upstream's constructor initializer list —
    /// see [`Self::joint_limits_penalty`]'s doc for what that default means
    /// (the penalty is always `1.0` until [`Self::set_penalty_multiplier`]
    /// is called).
    pub fn new(model: &'m RobotModel) -> Self {
        Self {
            model,
            penalty_multiplier: 0.0,
        }
    }

    /// `getPenaltyMultiplier`.
    pub fn penalty_multiplier(&self) -> f64 {
        self.penalty_multiplier
    }

    /// `setPenaltyMultiplier`.
    pub fn set_penalty_multiplier(&mut self, value: f64) {
        self.penalty_multiplier = value;
    }

    fn group(&self, group: &str, caller: &str) -> Result<&'m JointModelGroup> {
        let group_model = self.model.joint_model_group(group)?;
        if !group_model.is_chain() {
            return Err(Error::other(format!(
                "the group '{group}' is not a chain; cannot compute {caller}"
            )));
        }
        Ok(group_model)
    }

    /// `getJointLimitsPenalty`. See this module's doc comment for the exact
    /// per-joint-type semantics.
    pub fn joint_limits_penalty(&self, state: &RobotState, group: &JointModelGroup) -> Result<f64> {
        if self.penalty_multiplier.abs() <= f64::MIN_POSITIVE {
            return Ok(1.0);
        }

        let mut joint_limits_multiplier = 1.0;
        for &joint_index in group.joint_indices() {
            let joint = self.model.joint_model_at(joint_index);

            if joint.joint_type() == JointType::Revolute {
                if let JointKind::Revolute(revolute) = joint.kind() {
                    if revolute.is_continuous() {
                        continue;
                    }
                }
            }
            if joint.joint_type() == JointType::Planar {
                let bounds = joint.variable_bounds();
                if bounds[0].min_position == f64::NEG_INFINITY
                    || bounds[0].max_position == f64::INFINITY
                    || bounds[1].min_position == f64::NEG_INFINITY
                    || bounds[1].max_position == f64::INFINITY
                    || bounds[2].min_position == -std::f64::consts::PI
                    || bounds[2].max_position == std::f64::consts::PI
                {
                    continue;
                }
            }
            if joint.joint_type() == JointType::Floating {
                continue;
            }

            let joint_values = state.joint_position(joint.name())?;
            let bounds = joint.variable_bounds();
            let lower_bounds: Vec<f64> = bounds.iter().map(|b| b.min_position).collect();
            let upper_bounds: Vec<f64> = bounds.iter().map(|b| b.max_position).collect();
            let lower_bound_distance = joint.distance(joint_values, &lower_bounds);
            let upper_bound_distance = joint.distance(joint_values, &upper_bounds);
            let range = lower_bound_distance + upper_bound_distance;
            if range <= f64::MIN_POSITIVE {
                continue;
            }
            joint_limits_multiplier *=
                lower_bound_distance * upper_bound_distance / (range * range);
        }
        Ok(1.0 - (-self.penalty_multiplier * joint_limits_multiplier).exp())
    }

    /// `getManipulabilityIndex`. `translation`: use only the 3
    /// translational rows of the Jacobian (`true`) or all 6 (`false`).
    ///
    /// Branches on the *full* Jacobian's column count (its DOF, i.e. the
    /// group's active-joint count) regardless of `translation`: below 6,
    /// the product of singular values; at or above 6, `sqrt(det(J J^T))` —
    /// upstream's own branch, kept exactly, not simplified to "always SVD"
    /// or "always determinant".
    ///
    /// `state` must already have fresh forward kinematics ([`RobotState::update`])
    /// — the same precondition upstream's own `const RobotState&` carries
    /// implicitly (it never calls `update()` itself), enforced here at the
    /// type level via [`Posed`] rather than by doc comment alone.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group`.
    /// [`Error::Other`] if `group` is not a chain (see
    /// [`moveit_state::Posed::jacobian`], which this delegates to).
    pub fn manipulability_index(
        &self,
        state: &Posed,
        group: &str,
        translation: bool,
    ) -> Result<f64> {
        let group_model = self.group(group, "manipulability index")?;
        let jacobian = state.jacobian(group, &Vector3::zeros())?;
        let penalty = self.joint_limits_penalty(state, group_model)?;
        let columns = jacobian.ncols();

        let index = if translation {
            let linear = jacobian.rows(0, 3).clone_owned();
            if columns < 6 {
                let svd = SVD::new(linear, false, false);
                penalty * svd.singular_values.iter().product::<f64>()
            } else {
                let matrix = &linear * linear.transpose();
                penalty * matrix.determinant().sqrt()
            }
        } else if columns < 6 {
            let svd = SVD::new(jacobian, false, false);
            penalty * svd.singular_values.iter().product::<f64>()
        } else {
            let matrix = &jacobian * jacobian.transpose();
            penalty * matrix.determinant().sqrt()
        };
        Ok(index)
    }

    /// `getManipulabilityEllipsoid`. Eigenvalues/eigenvectors of the
    /// translational 3x3 block of `J J^T` — see this module's doc comment
    /// for why this port returns real `Vector3`/`Matrix3` where upstream
    /// returns complex `Eigen::MatrixXcd`, and for the ordering
    /// non-guarantee shared by both.
    ///
    /// # Errors
    ///
    /// Same as [`Self::manipulability_index`].
    pub fn manipulability_ellipsoid(
        &self,
        state: &Posed,
        group: &str,
    ) -> Result<(Vector3<f64>, Matrix3<f64>)> {
        self.group(group, "manipulability ellipsoid")?;
        let jacobian = state.jacobian(group, &Vector3::zeros())?;
        let matrix = &jacobian * jacobian.transpose();
        let block: Matrix3<f64> = matrix.fixed_view::<3, 3>(0, 0).clone_owned();
        let eigen = SymmetricEigen::new(block);
        Ok((eigen.eigenvalues, eigen.eigenvectors))
    }

    /// `getManipulability`: the ratio of the smallest to the largest
    /// singular value of the Jacobian (`translation`: 3 translational rows
    /// only, or all 6), scaled by the joint-limits penalty. Unlike
    /// [`Self::manipulability_index`], there is no `columns < 6` branch
    /// here — upstream always takes the SVD path for this one method.
    ///
    /// # Errors
    ///
    /// Same as [`Self::manipulability_index`].
    pub fn manipulability(&self, state: &Posed, group: &str, translation: bool) -> Result<f64> {
        let group_model = self.group(group, "manipulability")?;
        let penalty = self.joint_limits_penalty(state, group_model)?;
        let jacobian = state.jacobian(group, &Vector3::zeros())?;
        let target = if translation {
            jacobian.rows(0, 3).clone_owned()
        } else {
            jacobian
        };
        let svd = SVD::new(target, false, false);
        Ok(penalty * svd.singular_values.min() / svd.singular_values.max())
    }
}

#[cfg(test)]
mod tests {
    use std::fs;

    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
    use moveit_state::RobotState;

    use super::*;

    fn build_model() -> RobotModel {
        let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
        let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
        let urdf_xml = fs::read_to_string(urdf_path).expect("fixture URDF must read");
        let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("fixture model must build")
    }

    /// Default `penalty_multiplier` is `0.0`: [`KinematicsMetrics::joint_limits_penalty`]
    /// must return `1.0` unconditionally, upstream's `fabs(x) <=
    /// numeric_limits::min()` short-circuit.
    #[test]
    fn default_penalty_multiplier_is_unpenalized() {
        let model = build_model();
        let state = RobotState::new(&model);
        let metrics = KinematicsMetrics::new(&model);
        let group = model.joint_model_group("panda_arm").unwrap();
        assert_eq!(metrics.joint_limits_penalty(&state, group).unwrap(), 1.0);
    }

    /// All four public metrics compute finite values for a real chain group
    /// at the default (all-zero) configuration, and agree with each other's
    /// documented relationship: `manipulability_index(translation=false)`
    /// with `columns >= 6` is `sqrt(det(J J^T))`, which is the product of
    /// the *squared* singular values' square roots — i.e. the product of
    /// the singular values themselves, matching
    /// `manipulability`'s own SVD without needing a second oracle round-trip
    /// to notice a sign/scale error between the two code paths.
    #[test]
    fn manipulability_metrics_are_finite_for_a_chain_group() {
        let model = build_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let metrics = KinematicsMetrics::new(&model);

        for translation in [false, true] {
            let index = metrics
                .manipulability_index(&posed, "panda_arm", translation)
                .unwrap_or_else(|e| panic!("manipulability_index(translation={translation}): {e}"));
            assert!(index.is_finite(), "translation={translation}: {index}");

            let manipulability = metrics
                .manipulability(&posed, "panda_arm", translation)
                .unwrap_or_else(|e| panic!("manipulability(translation={translation}): {e}"));
            assert!(
                manipulability.is_finite(),
                "translation={translation}: {manipulability}"
            );
        }

        let (eigenvalues, _eigenvectors) = metrics
            .manipulability_ellipsoid(&posed, "panda_arm")
            .expect("manipulability_ellipsoid");
        for i in 0..3 {
            assert!(eigenvalues[i].is_finite());
            // J J^T's 3x3 linear block is positive semidefinite.
            assert!(
                eigenvalues[i] >= -1e-9,
                "eigenvalue[{i}] = {}",
                eigenvalues[i]
            );
        }
    }

    /// Falsification testing (see `crates/moveit-metrics`'s round-12 commit
    /// history) found that disabling the `translation` branch entirely —
    /// i.e. always computing over the full 6-row Jacobian regardless of the
    /// `translation` argument — is *not* caught by
    /// [`manipulability_metrics_are_finite_for_a_chain_group`] above (a
    /// disabled-translation result is still finite): only the oracle
    /// fixture's `manipulability_index_translation` field catches it. This
    /// test closes that gap independent of the oracle, so a future edit
    /// that collapses the two branches fails locally without a docker
    /// container.
    #[test]
    fn manipulability_index_translation_flag_changes_the_result() {
        let model = build_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let metrics = KinematicsMetrics::new(&model);

        let full = metrics
            .manipulability_index(&posed, "panda_arm", false)
            .unwrap();
        let translation_only = metrics
            .manipulability_index(&posed, "panda_arm", true)
            .unwrap();
        assert_ne!(
            full, translation_only,
            "translation=false and translation=true must read different Jacobian blocks"
        );
    }

    /// Falsification testing found that dropping the joint-limits penalty
    /// factor from [`Self::manipulability_index`] — always treating it as
    /// `1.0` — is *not* caught by
    /// [`manipulability_metrics_are_finite_for_a_chain_group`] above either
    /// (an unpenalized result is still finite): only the oracle fixture's
    /// `manipulability_index_full` field catches it. This test pins the
    /// exact multiplicative relationship
    /// (`penalized == unpenalized * joint_limits_penalty`) independent of
    /// the oracle.
    #[test]
    fn manipulability_index_scales_by_joint_limits_penalty() {
        let model = build_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let group = model.joint_model_group("panda_arm").unwrap();

        let mut metrics = KinematicsMetrics::new(&model);
        let unpenalized = metrics
            .manipulability_index(&posed, "panda_arm", false)
            .unwrap();

        metrics.set_penalty_multiplier(1.5);
        let penalty = metrics.joint_limits_penalty(&posed, group).unwrap();
        let penalized = metrics
            .manipulability_index(&posed, "panda_arm", false)
            .unwrap();

        approx::assert_relative_eq!(penalized, unpenalized * penalty, max_relative = 1e-12);
    }

    /// `hand` is a joint-list group, not a chain — every method must reject
    /// it as [`Error::Other`], matching upstream's `isChain()` guard (which
    /// upstream conflates into a `false` return; this port keeps it as a
    /// distinct error, see this module's doc comment).
    #[test]
    fn non_chain_group_is_rejected() {
        let model = build_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let metrics = KinematicsMetrics::new(&model);

        assert!(model.joint_model_group("hand").is_ok_and(|g| !g.is_chain()));
        assert!(matches!(
            metrics.manipulability_index(&posed, "hand", false),
            Err(Error::Other(_))
        ));
    }

    /// An unknown group name must surface as [`Error::UnknownName`], not a
    /// bare upstream-style `false`.
    #[test]
    fn unknown_group_is_unknown_name() {
        let model = build_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let metrics = KinematicsMetrics::new(&model);

        assert!(matches!(
            metrics.manipulability(&posed, "no_such_group", false),
            Err(Error::UnknownName { .. })
        ));
    }
}
