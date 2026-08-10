// Copyright (c) 2009, 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_parameters.hpp
//   moveit_planners/chomp/chomp_motion_planner/src/chomp_parameters.cpp

//! `ChompParameters`, upstream's plain-data tuning-knob bundle for the CHOMP
//! optimizer. No behavior beyond the two setters below; every other field is
//! read and written directly by callers, exactly as upstream's are (all
//! public, no accessor methods).

/// The trajectory-initialization methods
/// [`ChompParameters::set_trajectory_initialization_method`] accepts.
///
/// Ported from `ChompParameters::VALID_INITIALIZATION_METHODS`
/// (`chomp_parameters.cpp`).
pub const VALID_INITIALIZATION_METHODS: [&str; 4] =
    ["quintic-spline", "linear", "cubic", "fillTrajectory"];

/// Tuning parameters for the CHOMP optimizer.
///
/// Ported from `chomp::ChompParameters`. Every field keeps upstream's name
/// with the trailing underscore dropped, and upstream's default-constructor
/// value, transcribed from `ChompParameters::ChompParameters()`
/// (`chomp_parameters.cpp`).
#[derive(Debug, Clone, PartialEq)]
pub struct ChompParameters {
    /// Maximum time the optimizer can take to find a solution before
    /// terminating.
    pub planning_time_limit: f64,
    /// Maximum number of iterations the planner can take to find a good
    /// solution while optimizing.
    pub max_iterations: i32,
    /// Maximum iterations to be performed after a collision-free path is
    /// found.
    pub max_iterations_after_collision_free: i32,
    /// `smoothness_cost_weight`'s weight in the final cost CHOMP optimizes
    /// over.
    pub smoothness_cost_weight: f64,
    /// The weight given to obstacles towards the final cost CHOMP optimizes
    /// over.
    pub obstacle_cost_weight: f64,
    /// Learning rate used by the optimizer to find the local/global minima
    /// while reducing the total cost.
    pub learning_rate: f64,
    /// Variable associated with the cost in velocity.
    pub smoothness_cost_velocity: f64,
    /// Variable associated with the cost in acceleration.
    pub smoothness_cost_acceleration: f64,
    /// Variable associated with the cost in jerk.
    pub smoothness_cost_jerk: f64,
    /// Whether to use stochastic descent while optimizing the cost.
    pub use_stochastic_descent: bool,
    /// The noise added to the diagonal of the total quadratic cost matrix in
    /// the objective function.
    pub ridge_factor: f64,
    /// Whether pseudo-inverse calculations are enabled.
    pub use_pseudo_inverse: bool,
    /// The ridge factor used if pseudo-inverse is enabled.
    pub pseudo_inverse_ridge_factor: f64,
    /// The update limit for the robot joints.
    pub joint_update_limit: f64,
    /// The minimum distance that needs to be maintained to avoid obstacles.
    pub min_clearance: f64,
    /// The collision threshold cost that needs to be maintained to avoid
    /// collisions.
    pub collision_threshold: f64,
    /// Upstream's `filter_mode_`. Upstream never documents this field beyond
    /// its name; transcribed as-is.
    pub filter_mode: bool,
    /// The trajectory initialization method to use.
    ///
    /// Upstream stores this as a `std::string`, validated only through
    /// [`Self::set_trajectory_initialization_method`] against
    /// [`VALID_INITIALIZATION_METHODS`] (assigning the field directly skips
    /// validation, matching upstream's `public: std::string
    /// trajectory_initialization_method_` — the setter is a convenience, not
    /// an enforced invariant). Kept a `String` rather than ported as an enum:
    /// the field's only consumer, `chomp_planner.cpp`'s
    /// `ChompPlanner::solve` (`if
    /// (params.trajectory_initialization_method_.compare("quintic-spline")
    /// == 0)` / `"linear"` / `"cubic"` / `"fillTrajectory"`, plus a
    /// `.c_str()` diagnostic), is now ported as [`crate::planner::solve`]
    /// (round 20) — matched there with a plain `match ... .as_str()` over
    /// the same four string literals. An enum with illegal states
    /// unrepresentable would still be the better shape, but redesigning
    /// this field is a change to every existing caller of this crate's
    /// already-ported round-15 API, not something this port makes silently
    /// alongside an unrelated round's finding; left as `String` pending a
    /// dedicated decision.
    pub trajectory_initialization_method: String,
    /// If `true`, CHOMP tries to vary certain parameters to try and find a
    /// path if an initial path is not found with the specified CHOMP
    /// parameters.
    pub enable_failure_recovery: bool,
    /// The maximum recovery attempts to find a collision-free path after an
    /// initial failure to find a solution.
    pub max_recovery_attempts: i32,
}

impl Default for ChompParameters {
    fn default() -> Self {
        Self {
            planning_time_limit: 6.0,
            max_iterations: 50,
            max_iterations_after_collision_free: 5,
            smoothness_cost_weight: 0.1,
            obstacle_cost_weight: 1.0,
            learning_rate: 0.01,
            smoothness_cost_velocity: 0.0,
            smoothness_cost_acceleration: 1.0,
            smoothness_cost_jerk: 0.0,
            ridge_factor: 0.0,
            use_pseudo_inverse: false,
            pseudo_inverse_ridge_factor: 1e-4,
            joint_update_limit: 0.1,
            min_clearance: 0.2,
            collision_threshold: 0.07,
            use_stochastic_descent: true,
            filter_mode: false,
            trajectory_initialization_method: String::from("quintic-spline"),
            enable_failure_recovery: false,
            max_recovery_attempts: 5,
        }
    }
}

impl ChompParameters {
    /// Sets the recovery parameters, which can be changed in case CHOMP is
    /// not able to find a solution with the parameters originally set.
    ///
    /// Ported from `ChompParameters::setRecoveryParams`.
    pub fn set_recovery_params(
        &mut self,
        learning_rate: f64,
        ridge_factor: f64,
        planning_time_limit: i32,
        max_iterations: i32,
    ) {
        self.learning_rate = learning_rate;
        self.ridge_factor = ridge_factor;
        self.planning_time_limit = planning_time_limit as f64;
        self.max_iterations = max_iterations;
    }

    /// Sets a valid trajectory initialization method.
    ///
    /// Returns `true` and updates [`Self::trajectory_initialization_method`]
    /// if `method` is one of [`VALID_INITIALIZATION_METHODS`]; otherwise
    /// returns `false` and leaves the field unchanged, matching upstream's
    /// `setTrajectoryInitializationMethod` exactly (a silent no-op on an
    /// invalid method, not an error).
    ///
    /// Ported from `ChompParameters::setTrajectoryInitializationMethod`.
    pub fn set_trajectory_initialization_method(&mut self, method: impl Into<String>) -> bool {
        let method = method.into();
        if VALID_INITIALIZATION_METHODS.contains(&method.as_str()) {
            self.trajectory_initialization_method = method;
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_upstream_constructor() {
        let p = ChompParameters::default();
        assert_eq!(p.planning_time_limit, 6.0);
        assert_eq!(p.max_iterations, 50);
        assert_eq!(p.max_iterations_after_collision_free, 5);
        assert_eq!(p.smoothness_cost_weight, 0.1);
        assert_eq!(p.obstacle_cost_weight, 1.0);
        assert_eq!(p.learning_rate, 0.01);
        assert_eq!(p.smoothness_cost_velocity, 0.0);
        assert_eq!(p.smoothness_cost_acceleration, 1.0);
        assert_eq!(p.smoothness_cost_jerk, 0.0);
        assert_eq!(p.ridge_factor, 0.0);
        assert!(!p.use_pseudo_inverse);
        assert_eq!(p.pseudo_inverse_ridge_factor, 1e-4);
        assert_eq!(p.joint_update_limit, 0.1);
        assert_eq!(p.min_clearance, 0.2);
        assert_eq!(p.collision_threshold, 0.07);
        assert!(p.use_stochastic_descent);
        assert!(!p.filter_mode);
        assert_eq!(p.trajectory_initialization_method, "quintic-spline");
        assert!(!p.enable_failure_recovery);
        assert_eq!(p.max_recovery_attempts, 5);
    }

    #[test]
    fn set_trajectory_initialization_method_accepts_all_valid_methods() {
        let mut p = ChompParameters::default();
        for method in VALID_INITIALIZATION_METHODS {
            assert!(p.set_trajectory_initialization_method(method));
            assert_eq!(p.trajectory_initialization_method, method);
        }
    }

    #[test]
    fn set_trajectory_initialization_method_rejects_invalid_and_leaves_field_unchanged() {
        let mut p = ChompParameters::default();
        p.set_trajectory_initialization_method("linear");
        assert!(!p.set_trajectory_initialization_method("not-a-real-method"));
        assert_eq!(p.trajectory_initialization_method, "linear");
    }

    #[test]
    fn set_recovery_params_updates_exactly_four_fields() {
        let mut p = ChompParameters::default();
        let before = p.clone();
        p.set_recovery_params(0.5, 0.25, 10, 100);
        assert_eq!(p.learning_rate, 0.5);
        assert_eq!(p.ridge_factor, 0.25);
        assert_eq!(p.planning_time_limit, 10.0);
        assert_eq!(p.max_iterations, 100);

        assert_eq!(
            p.max_iterations_after_collision_free,
            before.max_iterations_after_collision_free
        );
        assert_eq!(p.smoothness_cost_weight, before.smoothness_cost_weight);
        assert_eq!(p.obstacle_cost_weight, before.obstacle_cost_weight);
    }
}
