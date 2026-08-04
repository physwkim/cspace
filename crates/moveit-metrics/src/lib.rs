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
//! Full audit of every `public:` member of upstream
//! `kinematics_metrics::KinematicsMetrics`, read from
//! `kinematics_metrics.hpp` itself. One line each: `ported as <symbol>` /
//! `distinct (<reason>)` / `D1 excludes it (<message type>)` / `unported
//! (<reason>)`. `getJointLimitsPenalty` is declared `private` upstream, not
//! `public`, so it is not one of the bullets below — see the
//! `getJointLimitsPenalty` section further down for its own port.
//!
//! 9 audit bullets below (verify: `rg -c '^//! - \`' crates/moveit-metrics/src/lib.rs`
//! gives `9`), zero `unported, in scope` gaps and zero D1 exclusions: this
//! header has no ROS message dependencies at all (`#include
//! <moveit/robot_state/robot_state.hpp>` and Eigen only), so nothing here
//! needed either bucket.
//!
//! - `KinematicsMetrics(RobotModelConstPtr)` — ported as [`KinematicsMetrics::new`].
//! - `getManipulabilityIndex(state, group_name: string, index&, translation=false)`
//!   — ported as [`KinematicsMetrics::manipulability_index`].
//! - `getManipulabilityIndex(state, joint_model_group: JointModelGroup*, index&, translation=false)`
//!   — distinct (collapses into the same [`KinematicsMetrics::manipulability_index`]:
//!   this crate has no `RobotModel`-owned `JointModelGroup*` to accept, only
//!   `&str` group names, matching [`moveit_state::Posed::jacobian`]'s own
//!   established `&str`-group convention).
//! - `getManipulabilityEllipsoid(state, group_name: string, eigen_values&, eigen_vectors&)`
//!   — ported as [`KinematicsMetrics::manipulability_ellipsoid`].
//! - `getManipulabilityEllipsoid(state, joint_model_group: JointModelGroup*, eigen_values&, eigen_vectors&)`
//!   — distinct (collapses into the same [`KinematicsMetrics::manipulability_ellipsoid`],
//!   same reason as the `getManipulabilityIndex` overload above).
//! - `getManipulability(state, group_name: string, condition_number&, translation=false)`
//!   — ported as [`KinematicsMetrics::manipulability`].
//! - `getManipulability(state, joint_model_group: JointModelGroup*, condition_number&, translation=false)`
//!   — distinct (collapses into the same [`KinematicsMetrics::manipulability`], same reason).
//! - `setPenaltyMultiplier(double)` — ported as [`KinematicsMetrics::set_penalty_multiplier`].
//! - `getPenaltyMultiplier() const` — ported as [`KinematicsMetrics::penalty_multiplier`].
//!
//! Three of the nine bullets are `distinct`: each is one half of an
//! overload pair whose `string`/`JointModelGroup*` variants this port
//! collapses into a single `&str`-taking method. That leaves six upstream
//! `get*` declarations (three computations, `getManipulabilityIndex`/
//! `getManipulabilityEllipsoid`/`getManipulability`, each declared twice)
//! folding into three port methods, plus
//! [`KinematicsMetrics::joint_limits_penalty`] — private upstream, exposed
//! here since a Rust module has no `friend`-equivalent visibility escape
//! and every other method in this crate needs it (see below) — as a fourth
//! public "does a computation" method: upstream's six `get*` declarations
//! collapsing to four port methods.
//!
//! Upstream's `bool` return (`false` for "unknown group" and "group is not
//! a chain" alike) becomes [`moveit_error::Result`] with the two causes
//! distinguished ([`moveit_error::Error::UnknownName`] vs.
//! [`moveit_error::Error::Other`]), matching
//! [`moveit_state::Posed::jacobian`]'s own error shape (see its doc
//! comment for the same deviation).
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
//!   `DBL_MAX` here would silently never match and always fall through —
//!   measured, not just argued: swapping the `x`/`y` sentinel comparisons
//!   for `-f64::MAX`/`f64::MAX` and rerunning this crate's tests
//!   (`--no-fail-fast`) leaves 13/14 passing and fails exactly
//!   `planar_xy_infinite_bounds_still_skip_despite_finite_theta` with
//!   `left: NaN, right: 0.7768698398515702` — not merely a wrong penalty
//!   but `NaN`, because the skip no longer fires, `PlanarJoint::distance`
//!   is evaluated against an `x`/`y` bound of `f64::INFINITY`/
//!   `f64::NEG_INFINITY` on each side (`(0.0 - f64::INFINITY).powi(2)` is
//!   `f64::INFINITY`, so `lower_bound_distance`/`upper_bound_distance` are
//!   each `f64::INFINITY`), and the per-joint term becomes
//!   `f64::INFINITY * f64::INFINITY / f64::INFINITY.powi(2)` —
//!   `∞ / ∞`, IEEE 754's textbook indeterminate form) or
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

    /// `joint_limits_penalty` is public and, unlike `manipulability*`, does
    /// not require a chain group -- pr2's `right_arm` (continuous joints)
    /// and `base` (the planar virtual joint) groups, both unedited, are
    /// enough to pin the per-joint-type skip logic without an oracle round
    /// trip.
    fn build_pr2_model() -> RobotModel {
        let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.urdf");
        let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.srdf");
        let urdf_xml = fs::read_to_string(urdf_path).expect("fixture URDF must read");
        let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("fixture model must build")
    }

    /// `../../fixtures/panda.srdf` must stay a byte-for-byte copy of the
    /// vendored upstream SRDF (`tools/ci/verify-fixture-provenance.sh`), so
    /// it cannot gain a group no upstream panda group has. This crate-local
    /// copy is deliberately allowed to diverge from it -- see the comment
    /// at the bottom of `tests/fixtures/panda.srdf` for the added
    /// `panda_base` group and what it's for.
    fn build_model_with_panda_base_group() -> RobotModel {
        let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
        let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/panda.srdf");
        let urdf_xml = fs::read_to_string(urdf_path).expect("fixture URDF must read");
        let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(srdf_path).expect("divergent fixture SRDF must parse");
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

    /// The exact boundary of the `penalty_multiplier` short-circuit:
    /// `penalty_multiplier == f64::MIN_POSITIVE` must still take the
    /// `<=` early return, matching upstream's `fabs(x) <=
    /// numeric_limits::min()`. `0.0` (the default, above) can't isolate
    /// `<=` from `<` -- `0.0` is strictly less than `f64::MIN_POSITIVE`
    /// either way -- only the boundary value itself can. Measured this
    /// round: weakening the comparison to `<` makes this exact input skip
    /// the short-circuit and fall into the real per-joint loop with a
    /// `penalty_multiplier` of `2.2250738585072014e-308`; `left: 0.0,
    /// right: 1.0` -- the result is finite (the per-joint product is
    /// bounded), but `1.0 - exp(-tiny * product)` underflows to `0.0` for
    /// any `tiny` this far below `f64::EPSILON`, so the two operators
    /// disagree by the full width of the penalty's range at this input,
    /// not by a rounding-sized amount.
    #[test]
    fn penalty_multiplier_at_the_min_positive_boundary_still_short_circuits() {
        let model = build_model();
        let state = RobotState::new(&model);
        let mut metrics = KinematicsMetrics::new(&model);
        metrics.set_penalty_multiplier(f64::MIN_POSITIVE);
        let group = model.joint_model_group("panda_arm").unwrap();
        assert_eq!(metrics.joint_limits_penalty(&state, group).unwrap(), 1.0);
    }

    /// A continuous revolute joint does not contribute to the product: pr2's
    /// `right_arm` chain includes two continuous joints
    /// (`r_forearm_roll_joint`, `r_wrist_roll_joint`, `type="continuous"`
    /// in `pr2.urdf`).
    ///
    /// A test that only moves a continuous joint's *position* cannot tell
    /// "excluded from the product" apart from "included but coincidentally
    /// invariant": `RevoluteJoint::distance` wraps at `2*PI`
    /// (`joint/revolute.rs`), and a continuous joint's bounds are always
    /// exactly `[-PI, PI]` -- two antipodal points on that wrap, so the
    /// wrapped distance to each is identical for *every* position, making
    /// the would-be product term a constant `0.25` regardless of where the
    /// joint sits. So this pins the exclusion directly: recompute the
    /// expected product from the same public primitives
    /// (`JointModel::variable_bounds`/`distance`) with continuous joints
    /// left out -- exactly the production loop's stated rule -- and compare
    /// against the real `joint_limits_penalty` output. Deleting the
    /// `is_continuous()` skip would fold both continuous joints' constant
    /// `0.25` factor into the actual product, moving it off this golden
    /// value while leaving it just as position-invariant as before -- the
    /// failure this test exists to catch would be invisible to a
    /// move-the-position-and-compare test at any position.
    #[test]
    fn continuous_revolute_joint_does_not_contribute_to_joint_limits_penalty() {
        let model = build_pr2_model();
        let mut metrics = KinematicsMetrics::new(&model);
        metrics.set_penalty_multiplier(1.5);
        let group = model.joint_model_group("right_arm").unwrap();
        let state = RobotState::new(&model);

        let mut expected_product = 1.0;
        for &joint_index in group.joint_indices() {
            let joint = model.joint_model_at(joint_index);
            if let JointKind::Revolute(revolute) = joint.kind() {
                if revolute.is_continuous() {
                    continue;
                }
            }
            let values = state.joint_position(joint.name()).unwrap();
            let bounds = joint.variable_bounds();
            let lower: Vec<f64> = bounds.iter().map(|b| b.min_position).collect();
            let upper: Vec<f64> = bounds.iter().map(|b| b.max_position).collect();
            let lower_distance = joint.distance(values, &lower);
            let upper_distance = joint.distance(values, &upper);
            let range = lower_distance + upper_distance;
            if range <= f64::MIN_POSITIVE {
                continue;
            }
            expected_product *= lower_distance * upper_distance / (range * range);
        }
        let expected_penalty = 1.0 - (-1.5_f64 * expected_product).exp();

        let actual_penalty = metrics.joint_limits_penalty(&state, group).unwrap();
        assert_eq!(actual_penalty, expected_penalty);
    }

    /// The exact boundary of the `range` skip, isolated the same way as
    /// `penalty_multiplier_at_the_min_positive_boundary_still_short_circuits`
    /// above: `panda_joint7`'s bounds are mutated so its distance to the
    /// lower bound is exactly `0.0` and its distance to the upper bound is
    /// exactly `f64::MIN_POSITIVE` (a revolute, non-continuous joint's
    /// `distance` is a plain `(value1 - value2).abs()`, so this is exact,
    /// not approximate), making `range == f64::MIN_POSITIVE` on the nose.
    /// `<=` must still skip this joint's term. Measured this round:
    /// weakening the comparison to `<` does not skip it, and
    /// `lower_bound_distance * upper_bound_distance / (range * range)`
    /// becomes `0.0 * f64::MIN_POSITIVE / (f64::MIN_POSITIVE *
    /// f64::MIN_POSITIVE)` -- `f64::MIN_POSITIVE * f64::MIN_POSITIVE`'s
    /// true value (`~4.95e-616`) is smaller than the smallest positive
    /// `f64` representable at all (the smallest subnormal, `~4.94e-324`),
    /// so it underflows to exactly `0.0`, giving `0.0 / 0.0 == NaN`,
    /// which then poisons the whole product -- the same failure
    /// shape as the planar `DBL_MAX` perturbation measured in this
    /// module's own doc comment, not a coincidence: both sentinels guard
    /// exactly this class of near-zero-denominator division.
    #[test]
    fn range_at_the_min_positive_boundary_still_skips() {
        let mut model = build_model();
        let joint = model.joint_model_mut("panda_joint7").unwrap();
        joint
            .set_variable_bounds(
                "panda_joint7",
                moveit_model::joint::VariableBounds {
                    min_position: 0.0,
                    max_position: f64::MIN_POSITIVE,
                    position_bounded: true,
                    ..Default::default()
                },
            )
            .unwrap();

        let mut metrics = KinematicsMetrics::new(&model);
        metrics.set_penalty_multiplier(1.5);
        let group = model.joint_model_group("panda_arm").unwrap();

        let state = RobotState::new(&model);
        let actual_penalty = metrics.joint_limits_penalty(&state, group).unwrap();

        assert!(actual_penalty.is_finite(), "{actual_penalty}");
    }

    /// A floating joint is skipped unconditionally (no bound check at all):
    /// `panda_base` is the divergent group over `virtual_joint`
    /// (`tests/fixtures/panda.srdf`, `type="floating"` -- see
    /// `build_model_with_panda_base_group`'s doc for why this reads a
    /// crate-local copy, not the shared root fixture). Its translation
    /// variables are bound to `±INFINITY`, so if the skip were
    /// removed, `distance()` to an infinite bound would make `range`
    /// infinite too, and `lower*upper/(range*range)` would be an
    /// `inf*inf/(inf*inf)` indeterminate form -- `NaN`, which fails this
    /// equality outright (`NaN != NaN`) rather than merely drifting. Measured
    /// this round, not just argued: neutralizing the `JointType::Floating`
    /// branch in `joint_limits_penalty` (`if false &&` guard, compiling but
    /// never taking the skip) makes this exact test fail with `left: NaN,
    /// right: NaN` -- this test already is the regression test for that
    /// claim, it just hadn't been fired at the production branch before.
    #[test]
    fn floating_joint_is_skipped_in_joint_limits_penalty() {
        let model = build_model_with_panda_base_group();
        let mut metrics = KinematicsMetrics::new(&model);
        metrics.set_penalty_multiplier(1.5);
        let group = model.joint_model_group("panda_base").unwrap();

        let baseline = RobotState::new(&model);
        let baseline_penalty = metrics.joint_limits_penalty(&baseline, group).unwrap();

        let mut moved = RobotState::new(&model);
        moved
            .set_joint_positions("virtual_joint", &[1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])
            .unwrap();
        let moved_penalty = metrics.joint_limits_penalty(&moved, group).unwrap();

        assert_eq!(baseline_penalty, moved_penalty);
    }

    /// A planar joint with its default (unconfigured) bounds is skipped:
    /// pr2's `base` group is exactly `world_joint` (`type="planar"` in
    /// `pr2.srdf`), left at `PlanarJoint`'s defaults --
    /// `x`/`y` bounded `±INFINITY`, `theta` bounded `∓PI`. The
    /// sentinel check in `joint_limits_penalty` reads only the joint's
    /// *bounds*, never the state's position, so this must hold for every
    /// position, not just the all-zero default -- moving `theta` off `0.0`
    /// must not change the penalty.
    ///
    /// This alone does not isolate the `x`/`y` infinite-bound check from
    /// the `theta` `±PI` literal check below it: both are true here at
    /// once (the OR only needs one to fire), so this test would keep
    /// passing even if the `x`/`y` half of the sentinel silently stopped
    /// matching -- `planar_xy_infinite_bounds_still_skip_despite_finite_theta`
    /// isolates that half specifically, the same way
    /// `planar_theta_bound_at_pi_literal_still_skips_despite_finite_translation`
    /// isolates `theta`'s.
    #[test]
    fn planar_joint_with_default_bounds_is_skipped_in_joint_limits_penalty() {
        let model = build_pr2_model();
        let mut metrics = KinematicsMetrics::new(&model);
        metrics.set_penalty_multiplier(1.5);
        let group = model.joint_model_group("base").unwrap();

        let baseline = RobotState::new(&model);
        let baseline_penalty = metrics.joint_limits_penalty(&baseline, group).unwrap();

        let mut moved = RobotState::new(&model);
        moved
            .set_joint_positions("world_joint", &[1.0, -2.0, 1.0])
            .unwrap();
        let moved_penalty = metrics.joint_limits_penalty(&moved, group).unwrap();

        assert_eq!(baseline_penalty, moved_penalty);
    }

    /// The `x`/`y` infinite-bound half of the same sentinel, isolated from
    /// `theta`: `theta` is given a finite, non-default range via
    /// `RobotModel::joint_model_mut`/`JointModel::set_variable_bounds`
    /// (`world_joint/theta` alone), while `x`/`y` are left at
    /// `PlanarJoint`'s default `±INFINITY`. The joint must still be
    /// skipped, on the `x`/`y` check alone -- measured this round, not just
    /// argued: swapping the `f64::NEG_INFINITY`/`f64::INFINITY` comparisons
    /// in `joint_limits_penalty` for `-f64::MAX`/`f64::MAX` (upstream's
    /// literal, finite `-DBL_MAX`/`DBL_MAX`, which this port's
    /// `new_planar` never actually produces -- see this module's own doc
    /// comment) and rerunning `--no-fail-fast` gives 14 tests: 13 passed, 1
    /// failed -- this test, and only this test, with `left: NaN, right:
    /// 0.7768698398515702` (see the module doc's own note on this same
    /// perturbation for why the failure is `NaN` and not merely a wrong
    /// finite value). Every other test in this file still passes (the
    /// `theta` check in `planar_joint_with_default_bounds_is_skipped_...` covers
    /// for it), and only this test catches the regression.
    #[test]
    fn planar_xy_infinite_bounds_still_skip_despite_finite_theta() {
        let mut model = build_pr2_model();
        let joint = model.joint_model_mut("world_joint").unwrap();
        joint
            .set_variable_bounds(
                "world_joint/theta",
                moveit_model::joint::VariableBounds {
                    min_position: -1.0,
                    max_position: 1.0,
                    position_bounded: true,
                    ..Default::default()
                },
            )
            .unwrap();
        // `x`/`y` left at `new_planar`'s default `±INFINITY` bounds.

        let mut metrics = KinematicsMetrics::new(&model);
        metrics.set_penalty_multiplier(1.5);
        let group = model.joint_model_group("base").unwrap();

        let state = RobotState::new(&model);
        let actual_penalty = metrics.joint_limits_penalty(&state, group).unwrap();
        let expected_penalty = 1.0 - (-1.5_f64).exp();

        assert_eq!(actual_penalty, expected_penalty);
    }

    /// The other direction of the same sentinel: give `world_joint` finite
    /// `x`/`y` bounds and a `theta` range that does not touch `±PI`,
    /// via the public `RobotModel::joint_model_mut`/
    /// `JointModel::set_variable_bounds` API (planar joints built from URDF
    /// never carry non-default bounds -- `joint/urdf.rs`'s
    /// `UrdfJointType::Planar` arm always calls `new_planar` with no limit
    /// data, so this direction cannot be reached from any URDF/SRDF
    /// fixture, only by mutating an already-built model). None of the six
    /// sentinel conditions hold, so the joint's distance-to-bound term is
    /// actually computed -- moving `theta` must now change the penalty.
    /// This is the test that pins the *other* end of the same design
    /// deviation `planar_xy_infinite_bounds_still_skip_despite_finite_theta`
    /// pins: that one confirms genuinely infinite bounds still match the
    /// port's `f64::NEG_INFINITY`/`f64::INFINITY` sentinel; this one
    /// confirms genuinely finite bounds do *not* spuriously match it (the
    /// failure mode upstream's own comment about `DBL_MAX` warns against,
    /// documented in this module's doc comment but, before these two tests,
    /// never exercised).
    #[test]
    fn planar_joint_with_finite_bounds_is_not_skipped_in_joint_limits_penalty() {
        let mut model = build_pr2_model();
        let joint = model.joint_model_mut("world_joint").unwrap();
        joint
            .set_variable_bounds(
                "world_joint/x",
                moveit_model::joint::VariableBounds {
                    min_position: -1.0,
                    max_position: 1.0,
                    position_bounded: true,
                    ..Default::default()
                },
            )
            .unwrap();
        joint
            .set_variable_bounds(
                "world_joint/y",
                moveit_model::joint::VariableBounds {
                    min_position: -1.0,
                    max_position: 1.0,
                    position_bounded: true,
                    ..Default::default()
                },
            )
            .unwrap();
        joint
            .set_variable_bounds(
                "world_joint/theta",
                moveit_model::joint::VariableBounds {
                    min_position: -1.0,
                    max_position: 1.0,
                    position_bounded: true,
                    ..Default::default()
                },
            )
            .unwrap();

        let mut metrics = KinematicsMetrics::new(&model);
        metrics.set_penalty_multiplier(1.5);
        let group = model.joint_model_group("base").unwrap();

        let baseline = RobotState::new(&model);
        let baseline_penalty = metrics.joint_limits_penalty(&baseline, group).unwrap();

        let mut moved = RobotState::new(&model);
        moved
            .set_joint_positions("world_joint", &[0.0, 0.0, 0.5])
            .unwrap();
        let moved_penalty = metrics.joint_limits_penalty(&moved, group).unwrap();

        assert_ne!(baseline_penalty, moved_penalty);
    }

    /// The `theta` bound's `±PI` literal comparison, isolated from
    /// the `x`/`y` infinite-bound checks: `x`/`y` are given the same finite
    /// bounds as the previous test, but `theta` is left at its default
    /// `∓PI` range. The joint must still be skipped on `theta` alone.
    ///
    /// This cannot be pinned by comparing two states the way the other
    /// three skip tests are: `PlanarJoint::distance` (`joint/planar.rs`)
    /// wraps its angular term at `2*PI`, and `-PI`/`PI` are the same point
    /// on that wrap, so *whether or not the skip fires*, the distance to
    /// the lower bound's `theta` component always equals the distance to
    /// the upper bound's -- for every position, not just the default. With
    /// `x`/`y` also symmetric around the all-zero default, that makes
    /// `lower_bound_distance == upper_bound_distance` regardless of `theta`,
    /// which forces the per-joint term to the constant `0.25` (`L*L /
    /// (2L)^2`) independent of whether the joint is skipped -- the same
    /// blind spot the `continuous_revolute_joint_does_not_contribute` test
    /// above works around, here on `theta` instead of a whole joint.
    ///
    /// So this pins the closed-form value directly instead: `base` is a
    /// single-joint group, so if `world_joint` is skipped,
    /// `joint_limits_multiplier` never leaves its initial `1.0` and the
    /// whole penalty collapses to `1.0 - exp(-penalty_multiplier)` --
    /// exactly, not approximately. If the `-std::f64::consts::PI`/
    /// `std::f64::consts::PI` literals in `joint_limits_penalty` ever
    /// changed (e.g. to a near-PI approximation), this default `theta`
    /// range would stop matching either literal, the joint would stop
    /// being skipped, and the penalty would move off this closed form --
    /// as confirmed by deliberately breaking the literal and re-running
    /// this test.
    #[test]
    fn planar_theta_bound_at_pi_literal_still_skips_despite_finite_translation() {
        let mut model = build_pr2_model();
        let joint = model.joint_model_mut("world_joint").unwrap();
        joint
            .set_variable_bounds(
                "world_joint/x",
                moveit_model::joint::VariableBounds {
                    min_position: -1.0,
                    max_position: 1.0,
                    position_bounded: true,
                    ..Default::default()
                },
            )
            .unwrap();
        joint
            .set_variable_bounds(
                "world_joint/y",
                moveit_model::joint::VariableBounds {
                    min_position: -1.0,
                    max_position: 1.0,
                    position_bounded: true,
                    ..Default::default()
                },
            )
            .unwrap();
        // `theta` left at `new_planar`'s default `∓PI` bounds.

        let mut metrics = KinematicsMetrics::new(&model);
        metrics.set_penalty_multiplier(1.5);
        let group = model.joint_model_group("base").unwrap();

        let state = RobotState::new(&model);
        let actual_penalty = metrics.joint_limits_penalty(&state, group).unwrap();
        let expected_penalty = 1.0 - (-1.5_f64).exp();

        assert_eq!(actual_penalty, expected_penalty);
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
