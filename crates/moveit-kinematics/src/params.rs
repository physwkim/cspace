// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_parameters.yaml

use std::collections::HashMap;

use moveit_error::{Error, Result};

/// Search parameters for a [`crate::KinematicsSolver`].
///
/// Upstream reads these from a ROS parameter server, one
/// `kdl_kinematics.<field>` entry per field of
/// `kdl_kinematics_parameters.yaml`, loaded through a generated
/// `ParamListener` inside `initialize(node, ...)`. This port has no node to
/// read from (see this crate's "do not port the ROS surface" doc), so the
/// same defaults become a plain [`Default`] impl the caller can override
/// before constructing a solver.
#[derive(Debug, Clone, PartialEq)]
pub struct SolverParams {
    /// `max_solver_iterations`: the iteration cap inside one `CartToJnt`
    /// call, not a wall-clock timeout — see this crate's doc comment on
    /// why a wall-clock `searchPositionIK` timeout is not ported.
    pub max_solver_iterations: usize,
    /// `epsilon`: `CartToJnt` accepts a step once
    /// `max(position_error, orientation_error)` drops to this or below.
    pub epsilon: f64,
    /// `orientation_vs_position`: relative weight of the three orientation
    /// error rows against the three position error rows.
    /// [`SolverParams::position_only`] overrides this to `0.0` exactly as
    /// upstream's `position_only_ik` overrides `orientation_vs_position`.
    pub orientation_vs_position: f64,
    /// `position_only_ik`.
    pub position_only: bool,
    /// `joints`/`__map_joints.weight`: per-active-joint weight, by joint
    /// name. A name absent here defaults to `1.0`, matching
    /// `KDLKinematicsPlugin::getJointWeights`'s own default-then-override.
    pub joint_weights: HashMap<String, f64>,
    /// `ChainIkSolverVelMimicSVD`'s `threshold` constructor argument: a
    /// singular value below `threshold * (largest singular value)` is
    /// treated as zero. Used by [`crate::NewtonRaphsonSolver`], the solver
    /// that ports `ChainIkSolverVelMimicSVD` as-is.
    pub svd_threshold: f64,
    /// Tikhonov damping constant for [`crate::LevenbergMarquardtSolver`]'s
    /// velocity step: `s / (s^2 + lambda^2)` in place of
    /// [`SolverParams::svd_threshold`]'s hard singular-value cutoff. Not an
    /// upstream field — `kdl_kinematics_plugin` only ever ships the
    /// truncated-SVD solver; see [`crate::LevenbergMarquardtSolver`]'s doc
    /// comment for why a fixed damping constant is still a faithful
    /// Levenberg-Marquardt / damped-least-squares IK step.
    pub lma_lambda: f64,
    /// Not an upstream field: replaces `searchPositionIK`'s wall-clock
    /// `timeout` with a bounded retry count, so a run is reproducible from
    /// a seed rather than from wall-clock timing. Upstream re-seeds
    /// randomly and retries `CartToJnt` until `timedOut`; this port retries
    /// up to this many times instead. `0` matches `getPositionIK`'s own
    /// "single attempt, no re-seeding" behaviour (it calls
    /// `searchPositionIK` with `timeout = 0.0`).
    pub max_restarts: usize,
}

impl Default for SolverParams {
    fn default() -> Self {
        Self {
            max_solver_iterations: 500,
            epsilon: 0.00001,
            orientation_vs_position: 1.0,
            position_only: false,
            joint_weights: HashMap::new(),
            svd_threshold: 0.001,
            lma_lambda: 0.01,
            max_restarts: 20,
        }
    }
}

impl SolverParams {
    /// `orientation_vs_position_weight` at `searchPositionIK`'s call site:
    /// `position_only_ik ? 0.0 : orientation_vs_position`.
    pub(crate) fn orientation_weight(&self) -> f64 {
        if self.position_only {
            0.0
        } else {
            self.orientation_vs_position
        }
    }

    /// Reject the three numeric fields whose only guard against dividing by
    /// an exact `0.0` (or being silently defeated by a NaN, which fails
    /// every comparison including the one meant to catch it) is that they
    /// stay strictly positive:
    ///
    /// - [`SolverParams::epsilon`] gates every convergence check in
    ///   `cart_to_jnt::cart_to_jnt` (`delta_twist_norm <= epsilon`,
    ///   `delta_q_norm < epsilon`, `step_size < epsilon`). Upstream itself
    ///   requires `epsilon > 0.0` (`kdl_kinematics_parameters.yaml`'s
    ///   `gt<>: [0.0]`); a non-positive or NaN `epsilon` makes every one of
    ///   those checks either trivially true or -- for NaN -- permanently
    ///   false, and `step_size`'s guard failing lets it underflow to `0.0`
    ///   and feed `newton_raphson::NewtonRaphsonSolver`'s
    ///   `step_size / old_step_size`.
    /// - [`SolverParams::svd_threshold`] gates
    ///   `NewtonRaphsonSolver`'s truncated-SVD pseudo-inverse
    ///   (`s > svd_threshold * smax`, else `1.0 / s`). Upstream hardcodes
    ///   this at a fixed `0.001` and never exposes it; a non-positive or
    ///   NaN threshold lets an exact-zero singular value -- reachable on
    ///   any chain with a kinematic redundancy, not a contrived input --
    ///   through to `1.0 / s`.
    /// - [`SolverParams::lma_lambda`] is this crate's own addition (see
    ///   `LevenbergMarquardtSolver`'s doc comment), and a non-positive
    ///   value defeats the Tikhonov damping it exists to provide: at
    ///   `lambda <= 0.0`, the same exact-zero singular value divides
    ///   `s / (s * s + lambda * lambda)` as `0.0 / 0.0`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::other`] naming the first field found to be
    /// non-finite or not strictly positive.
    pub(crate) fn validate(&self) -> Result<()> {
        for (name, value) in [
            ("epsilon", self.epsilon),
            ("svd_threshold", self.svd_threshold),
            ("lma_lambda", self.lma_lambda),
        ] {
            if !(value.is_finite() && value > 0.0) {
                return Err(Error::other(format!(
                    "SolverParams::{name} must be finite and > 0.0, got {value}"
                )));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orientation_weight_is_zero_exactly_when_position_only_is_set() {
        let mut params = SolverParams {
            orientation_vs_position: 3.5,
            ..Default::default()
        };
        assert_eq!(params.orientation_weight(), 3.5);

        params.position_only = true;
        assert_eq!(params.orientation_weight(), 0.0);
    }
}
