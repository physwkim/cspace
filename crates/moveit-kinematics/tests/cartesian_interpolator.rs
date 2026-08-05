// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Invariant-boundary tests for [`moveit_kinematics::CartesianInterpolator`]
//! and the two jump-detection entry points it shares with upstream.
//!
//! Every boundary here is one the port can land on either side of, not a
//! narrative "walk the arm somewhere" scenario:
//!
//! - the waypoint count either side of the `floor(distance / max_step)`
//!   step at which it changes by one,
//! - a path that reaches its target (fraction exactly `1.0`) against one
//!   that runs out of workspace (fraction `< 1.0`, and the last waypoint at
//!   exactly the reported fraction),
//! - jump detection off against on over the *same* path, whose sixth
//!   waypoint is a genuine second IK branch for the identical tip pose,
//! - relative against absolute, in both directions: a path the relative
//!   rule rejects and the absolute one accepts, and the reverse,
//! - revolute against prismatic within the absolute rule, both driven by
//!   the *same* `JumpThreshold`, so only the joint-type dispatch can tell
//!   them apart.
//!
//! # Tolerances
//!
//! Every constant below was measured on this fixture before it was
//! written down; none is copied from another test. The margin over the
//! measurement is stated at each constant. All of them were also measured
//! with `SolverParams { max_restarts: 0, .. }` and came out bit-identical,
//! so no number here depends on the solver's random restarts firing --
//! `NewtonRaphsonSolver`'s first attempt converges at every waypoint these
//! tests reach, and no restart rescues the ones they do not.

use std::fs;

use moveit_geometry::{Isometry3, Vector3};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

use moveit_kinematics::{
    CartesianInterpolator, JumpThreshold, KinematicsSolver, MaxEefStep, NewtonRaphsonSolver,
    SolveOptions, SolverParams, check_joint_space_jump,
};

/// Distance from the last waypoint's tip to the requested target on the
/// fully reachable path. Measured `3.4426046318141434e-8` m; this is ~29x
/// that. It is well inside `SolverParams::epsilon` (`1e-5`), which is the
/// loosest the solver's own convergence contract permits, so a failure
/// here is the interpolation or the seed chain changing, not the solver
/// being allowed to stop earlier.
const TARGET_TRANSLATION_TOL: f64 = 1e-6;

/// Angle between the last waypoint's tip and the requested target on the
/// fully reachable path. Measured `4.371537578442642e-16` rad; this is
/// ~23x that. The path is a pure translation, so the orientation is
/// carried unchanged and the residual is rounding only.
const TARGET_ROTATION_TOL: f64 = 1e-14;

/// Distance from the last waypoint's tip to the straight-line pose at the
/// *returned* fraction on the out-of-workspace path. Measured
/// `1.1903178002181627e-6` m. The constant is `SolverParams::epsilon`
/// itself -- the solver's own convergence bound on the pose residual, and
/// therefore the tightest value that cannot fail for a legal solve -- at
/// ~8.4x the measurement.
const STOP_TRANSLATION_TOL: f64 = 1e-5;

/// Angle for the same comparison. Measured `5.082016314950136e-16` rad;
/// ~20x that.
const STOP_ROTATION_TOL: f64 = 1e-14;

/// How far the substituted second IK branch's tip pose sits from the
/// waypoint pose it replaces. Measured `2.38214130848478e-8` m and
/// `5.897333237770083e-8` rad; this is ~17x the larger. It is asserted so
/// that "branch flip" means what it says: the *same* Cartesian pose
/// reached through a different joint-space solution, not a different pose.
const BRANCH_FLIP_TOL: f64 = 1e-6;

/// Tip error against the hand-computed endpoint of a translation given in
/// the link's own frame. Measured `1.849548768641516e-8` m and
/// `5.082679126893948e-8` rad; ~20x the larger.
const LOCAL_FRAME_TOL: f64 = 1e-6;

/// The seed configuration every test starts from. Chosen only for being
/// well away from `panda_arm`'s joint limits and singularities, so the
/// first Newton-Raphson attempt converges at every waypoint.
const START: [f64; 7] = [0.0, -0.4, 0.0, -1.9, 0.0, 1.6, 0.75];

/// A configuration far enough from [`START`] that solving the sixth
/// waypoint's pose from it lands on a different IK branch. Not a solution
/// of anything by itself -- only a seed.
const FAR_SEED: [f64; 7] = [-2.5, 1.2, 2.5, -2.5, -2.0, 2.0, 2.0];

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
}

fn build_model(urdf_file: &str, srdf_file: &str) -> RobotModel {
    let urdf_path = fixture_path(urdf_file);
    let srdf_path = fixture_path(srdf_file);
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

/// A fresh solver every time. `NewtonRaphsonSolver` owns its RNG and
/// advances it on every restart, so reusing one instance would make a
/// probe's result depend on how many restarts an earlier call burned.
fn solver(model: &RobotModel) -> NewtonRaphsonSolver {
    NewtonRaphsonSolver::new(model, "panda_arm", &SolverParams::default())
        .expect("panda_arm is a chain")
}

fn start_state<'m>(model: &'m RobotModel, joint_names: &[String]) -> RobotState<'m> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, value) in joint_names.iter().zip(START) {
        state
            .set_variable_position(name, value)
            .expect("panda_arm joint");
    }
    state
}

fn tip_pose(state: &RobotState<'_>, tip: &str) -> Isometry3 {
    state
        .clone()
        .update()
        .global_link_transform(tip)
        .expect("tip link")
}

fn translated(pose: &Isometry3, dx: f64) -> Isometry3 {
    let mut moved = *pose;
    moved.translation.vector += Vector3::new(dx, 0.0, 0.0);
    moved
}

/// The straight-line pose `t` of the way from `from` to `to`. Both poses
/// in every test here share an orientation, so only the translation
/// interpolates; writing it out keeps the expectation independent of the
/// port's own private `interpolate_pose`.
fn lerp_translation(from: &Isometry3, to: &Isometry3, t: f64) -> Isometry3 {
    let mut pose = *from;
    pose.translation.vector = from.translation.vector * (1.0 - t) + to.translation.vector * t;
    pose
}

fn errors(actual: &Isometry3, expected: &Isometry3) -> (f64, f64) {
    (
        (actual.translation.vector - expected.translation.vector).norm(),
        actual.rotation.angle_to(&expected.rotation),
    )
}

/// The fully reachable path, plus the pieces every caller of it needs.
struct Fixture {
    model: RobotModel,
    tip: String,
    joint_names: Vec<String>,
}

impl Fixture {
    fn new() -> Self {
        let model = build_model("panda.urdf", "panda.srdf");
        let probe = solver(&model);
        let tip = probe.tip_frame().to_owned();
        let joint_names = probe.joint_names().to_vec();
        Self {
            model,
            tip,
            joint_names,
        }
    }

    fn start(&self) -> RobotState<'_> {
        start_state(&self.model, &self.joint_names)
    }

    /// 0.10 m of +x at a 0.01 m max step: 12 waypoints, fraction 1.0.
    fn reachable_path(&self) -> Vec<RobotState<'_>> {
        let start = self.start();
        let target = translated(&tip_pose(&start, &self.tip), 0.10);
        let config =
            CartesianInterpolator::new("panda_arm", &self.tip, MaxEefStep::from_step_size(0.01));
        let (path, fraction) = config
            .to_pose(
                &start,
                &mut solver(&self.model),
                &target,
                &mut SolveOptions::default(),
            )
            .expect("reachable path");
        assert_eq!(fraction.value(), 1.0, "fixture path must be fully solved");
        path
    }

    /// [`Fixture::reachable_path`] with waypoint 6 replaced by a second IK
    /// solution for that same waypoint's tip pose. Consecutive joint-space
    /// increments around index 6 jump from ~0.055 to ~5.69 (group) and
    /// ~0.027 to ~2.31 (largest single joint) without the Cartesian path
    /// changing at all.
    fn flipped_path(&self) -> Vec<RobotState<'_>> {
        const FLIP_AT: usize = 6;
        let mut path = self.reachable_path();
        let pose = tip_pose(&path[FLIP_AT], &self.tip);
        let base = tip_pose(&path[FLIP_AT], solver(&self.model).base_frame());
        let alternative = solver(&self.model)
            .solve(&FAR_SEED, &(base.inverse() * pose))
            .expect("a second IK branch exists for waypoint 6");
        for (name, value) in self.joint_names.iter().zip(&alternative) {
            path[FLIP_AT]
                .set_variable_position(name, *value)
                .expect("panda_arm joint");
        }

        let (translation, rotation) = errors(&tip_pose(&path[FLIP_AT], &self.tip), &pose);
        assert!(
            translation < BRANCH_FLIP_TOL && rotation < BRANCH_FLIP_TOL,
            "the substitute must be the same tip pose on another branch, \
             not another pose: translation {translation:e} m, rotation {rotation:e} rad"
        );
        path
    }
}

/// A path of `panda_arm` states whose only moving joint steps by `step`
/// each time. Every increment is identical, so the relative rule -- which
/// measures against the path's own average -- can never fire on it however
/// large `step` is.
fn uniform_path(model: &RobotModel, step: f64, count: usize) -> Vec<RobotState<'_>> {
    (0..count)
        .map(|i| {
            let mut state = RobotState::new(model);
            state.set_to_default_values();
            state
                .set_variable_position("panda_joint1", i as f64 * step)
                .expect("panda_joint1");
            state
        })
        .collect()
}

/// A path of `hand` states. `hand`'s only active joint is the prismatic
/// `panda_finger_joint1`, so the absolute rule can only reach it through
/// its prismatic threshold.
fn prismatic_path<'m>(model: &'m RobotModel, positions: &[f64]) -> Vec<RobotState<'m>> {
    positions
        .iter()
        .map(|&p| {
            let mut state = RobotState::new(model);
            state.set_to_default_values();
            state
                .set_variable_position("panda_finger_joint1", p)
                .expect("panda_finger_joint1");
            state
        })
        .collect()
}

#[test]
fn reachable_path_reports_a_full_fraction_and_lands_on_the_target() {
    let fixture = Fixture::new();
    let start = fixture.start();
    let target = translated(&tip_pose(&start, &fixture.tip), 0.10);
    let config =
        CartesianInterpolator::new("panda_arm", &fixture.tip, MaxEefStep::from_step_size(0.01));

    let (path, fraction) = config
        .to_pose(
            &start,
            &mut solver(&fixture.model),
            &target,
            &mut SolveOptions::default(),
        )
        .expect("reachable path");

    assert_eq!(
        fraction.value(),
        1.0,
        "a reachable target must report exactly 1.0, not merely close to it"
    );
    let (translation, rotation) = errors(
        &tip_pose(path.last().expect("waypoints"), &fixture.tip),
        &target,
    );
    assert!(
        translation < TARGET_TRANSLATION_TOL,
        "final tip translation error {translation:e} m exceeds {TARGET_TRANSLATION_TOL:e}"
    );
    assert!(
        rotation < TARGET_ROTATION_TOL,
        "final tip rotation error {rotation:e} rad exceeds {TARGET_ROTATION_TOL:e}"
    );
}

#[test]
fn waypoint_count_steps_by_one_across_the_max_step_floor() {
    // 0.10 / 0.010    == 10.000000000000000 -> floor 10 -> 11 steps -> 12 states
    // 0.10 / 0.010001 ==  9.999000099990003 -> floor  9 -> 10 steps -> 11 states
    // The two step sizes bracket the only discontinuity in
    // `floor(distance / max_step)` anywhere near 0.01.
    const BELOW: f64 = 0.010;
    const ABOVE: f64 = 0.010001;
    assert_eq!(
        (0.10_f64 / BELOW).floor(),
        10.0,
        "the fixture distance must sit exactly on the floor boundary"
    );
    assert_eq!(
        (0.10_f64 / ABOVE).floor(),
        9.0,
        "the larger step must fall on the other side of it"
    );

    let fixture = Fixture::new();
    let start = fixture.start();
    let target = translated(&tip_pose(&start, &fixture.tip), 0.10);

    let mut counts = Vec::new();
    for step in [BELOW, ABOVE] {
        let config =
            CartesianInterpolator::new("panda_arm", &fixture.tip, MaxEefStep::from_step_size(step));
        let (path, fraction) = config
            .to_pose(
                &start,
                &mut solver(&fixture.model),
                &target,
                &mut SolveOptions::default(),
            )
            .expect("reachable path");
        // Both sides are fully solved, so the count difference is the step
        // count and not one of them stopping early.
        assert_eq!(fraction.value(), 1.0, "step {step} must reach the target");
        counts.push(path.len());
    }

    assert_eq!(
        counts,
        vec![12, 11],
        "crossing the floor boundary must move the waypoint count by exactly one"
    );
}

#[test]
fn unreachable_path_stops_at_its_last_reachable_waypoint() {
    let fixture = Fixture::new();
    let start = fixture.start();
    let start_pose = tip_pose(&start, &fixture.tip);
    let target = translated(&start_pose, 2.0);
    let config =
        CartesianInterpolator::new("panda_arm", &fixture.tip, MaxEefStep::from_step_size(0.05));

    let (path, fraction) = config
        .to_pose(
            &start,
            &mut solver(&fixture.model),
            &target,
            &mut SolveOptions::default(),
        )
        .expect("partial path");

    assert!(
        fraction.value() < 1.0,
        "2 m of +x leaves panda's workspace, so the fraction must be short of 1.0"
    );

    // Not before: the last waypoint sits at exactly the reported fraction.
    // This is the whole content of the port's split of upstream's
    // `double& percentage` -- a trajectory whose tail runs past what the
    // return value claims would satisfy `fraction < 1.0` just as well.
    let last = path.last().expect("at least the start state");
    let (translation, rotation) = errors(
        &tip_pose(last, &fixture.tip),
        &lerp_translation(&start_pose, &target, fraction.value()),
    );
    assert!(
        translation < STOP_TRANSLATION_TOL,
        "last waypoint is {translation:e} m from the pose at the reported \
         fraction {}, so the two disagree",
        fraction.value()
    );
    assert!(
        rotation < STOP_ROTATION_TOL,
        "last waypoint is {rotation:e} rad from the pose at the reported fraction"
    );

    // Not after: one more max step along the same line has no IK solution,
    // from the last waypoint's own configuration or from the start's. The
    // step count is recomputed here rather than read out of the port, so
    // this stays a statement about the workspace and not about the port's
    // arithmetic.
    let steps = (2.0_f64 / 0.05).floor() as usize + 1;
    let beyond = lerp_translation(&start_pose, &target, fraction.value() + 1.0 / steps as f64);
    for (label, from) in [("last waypoint", last), ("start", &start)] {
        let mut probe = solver(&fixture.model);
        let base = tip_pose(from, probe.base_frame());
        let seed: Vec<f64> = fixture
            .joint_names
            .iter()
            .map(|name| from.variable_position(name).expect("panda_arm joint"))
            .collect();
        let solution = probe.solve(&seed, &(base.inverse() * beyond));
        assert!(
            solution.is_none(),
            "one step past the stop point is reachable from the {label}, \
             so stopping at {} was early: {solution:?}",
            fraction.value()
        );
    }
}

#[test]
fn a_disabled_threshold_leaves_the_branch_flip_in_place() {
    let fixture = Fixture::new();
    let group = fixture
        .model
        .joint_model_group("panda_arm")
        .expect("panda_arm");
    let mut path = fixture.flipped_path();
    let before = path.len();

    let kept = check_joint_space_jump(&mut path, group, &JumpThreshold::disabled());

    assert_eq!(
        kept.value(),
        1.0,
        "a disabled threshold must report the whole path solved"
    );
    assert_eq!(path.len(), before, "and must not drop a waypoint");
}

#[test]
fn an_absolute_threshold_truncates_at_the_revolute_branch_flip() {
    let fixture = Fixture::new();
    let group = fixture
        .model
        .joint_model_group("panda_arm")
        .expect("panda_arm");
    let mut path = fixture.flipped_path();
    let before = path.len();

    // 0.5 rad sits between the flip's largest single-joint increment
    // (2.3148 rad) and its neighbours' (0.0332 rad at most).
    let kept = check_joint_space_jump(&mut path, group, &JumpThreshold::absolute(0.5, 0.018));

    assert_eq!(
        kept.value(),
        6.0 / before as f64,
        "the surviving fraction is the jump index over the length before truncation"
    );
    assert_eq!(
        path.len(),
        6,
        "truncation keeps the waypoints strictly before the jump"
    );
}

#[test]
fn a_relative_threshold_truncates_at_the_branch_flip() {
    let fixture = Fixture::new();
    let group = fixture
        .model
        .joint_model_group("panda_arm")
        .expect("panda_arm");
    let mut path = fixture.flipped_path();
    let before = path.len();

    // The flip drags the path average up to 1.0755, so 2x it is 2.1509 --
    // above every unflipped increment (0.0664 at most) and below the
    // flip's 5.6913.
    let kept = check_joint_space_jump(&mut path, group, &JumpThreshold::relative(2.0));

    assert_eq!(
        kept.value(),
        6.0 / before as f64,
        "the relative rule must find the same jump the absolute one does"
    );
}

#[test]
fn the_relative_rule_fires_where_the_absolute_rule_does_not() {
    let fixture = Fixture::new();
    let group = fixture
        .model
        .joint_model_group("panda_arm")
        .expect("panda_arm");
    let before = fixture.flipped_path().len();

    // No *single* joint moves 3 rad at the flip -- the largest is 2.3148 --
    // so the per-joint rule sees nothing.
    let mut absolute = fixture.flipped_path();
    let by_absolute =
        check_joint_space_jump(&mut absolute, group, &JumpThreshold::absolute(3.0, 0.018));

    // The seven joints *together* move 5.6913, against a path average of
    // 1.0755, so the group-sum rule does.
    let mut relative = fixture.flipped_path();
    let by_relative = check_joint_space_jump(&mut relative, group, &JumpThreshold::relative(2.0));

    assert_eq!(
        by_absolute.value(),
        1.0,
        "the absolute rule is per joint, so a 2.3148 rad joint clears a 3.0 rad bound"
    );
    assert_eq!(
        by_relative.value(),
        6.0 / before as f64,
        "the relative rule sums the group, so the same waypoint is a jump"
    );
}

#[test]
fn the_absolute_rule_fires_where_the_relative_rule_does_not() {
    let fixture = Fixture::new();
    let group = fixture
        .model
        .joint_model_group("panda_arm")
        .expect("panda_arm");
    let before = 3;

    // Every increment is 1.0 rad, so the average is 1.0 and no increment
    // can exceed 1.5x it however large the steps are.
    let mut relative = uniform_path(&fixture.model, 1.0, before);
    let by_relative = check_joint_space_jump(&mut relative, group, &JumpThreshold::relative(1.5));

    let mut absolute = uniform_path(&fixture.model, 1.0, before);
    let by_absolute =
        check_joint_space_jump(&mut absolute, group, &JumpThreshold::absolute(0.5, 0.018));

    assert_eq!(
        by_relative.value(),
        1.0,
        "a uniform path has no increment above its own average, at any scale"
    );
    assert_eq!(
        by_absolute.value(),
        1.0 / before as f64,
        "the absolute rule is scale-aware, so the very first 1.0 rad step is a jump"
    );
}

#[test]
fn the_absolute_rule_measures_prismatic_joints_against_its_prismatic_bound() {
    let fixture = Fixture::new();
    let arm = fixture
        .model
        .joint_model_group("panda_arm")
        .expect("panda_arm");
    let hand = fixture.model.joint_model_group("hand").expect("hand");

    // One threshold, two paths. `panda_arm` is seven revolute joints and
    // `hand` is one prismatic joint, so nothing but the joint-type
    // dispatch decides which of 0.5 rad and 0.018 m each path is measured
    // against. Swap the two arms and both assertions below invert.
    let threshold = JumpThreshold::absolute(0.5, 0.018);

    // 0.020 m > 0.018 m at index 1; 0.015 m < 0.018 m at index 2.
    let mut prismatic = prismatic_path(&fixture.model, &[0.0, 0.020, 0.035]);
    let prismatic_len = prismatic.len();
    let by_prismatic = check_joint_space_jump(&mut prismatic, hand, &threshold);

    // 2.3148 rad > 0.5 rad at index 6; every other increment is under
    // 0.0332 rad -- and every one of them is over 0.018 m, which is what
    // the prismatic bound would have caught had it been consulted here.
    let mut revolute = fixture.flipped_path();
    let revolute_len = revolute.len();
    let by_revolute = check_joint_space_jump(&mut revolute, arm, &threshold);

    assert_eq!(
        by_prismatic.value(),
        1.0 / prismatic_len as f64,
        "0.020 m must be measured against the 0.018 m prismatic bound, not 0.5"
    );
    assert_eq!(
        by_revolute.value(),
        6.0 / revolute_len as f64,
        "0.027 rad must be measured against the 0.5 rad revolute bound, not 0.018"
    );
}

#[test]
fn an_empty_path_is_not_measured_for_jumps() {
    let fixture = Fixture::new();
    let group = fixture
        .model
        .joint_model_group("panda_arm")
        .expect("panda_arm");

    // The absolute rule reads `waypoints[0]` to get at the robot model, so
    // without the short-path guard this indexes an empty slice.
    let mut empty: Vec<RobotState<'_>> = Vec::new();
    let kept = check_joint_space_jump(&mut empty, group, &JumpThreshold::absolute(0.5, 0.018));

    assert_eq!(kept.value(), 1.0, "nothing to measure is nothing to reject");
}

#[test]
fn along_translation_reports_the_distance_actually_travelled() {
    let fixture = Fixture::new();
    let start = fixture.start();
    let start_pose = tip_pose(&start, &fixture.tip);
    let config =
        CartesianInterpolator::new("panda_arm", &fixture.tip, MaxEefStep::from_step_size(0.05));
    let translation = Vector3::new(2.0, 0.0, 0.0);

    let (_, travelled) = config
        .along_translation(
            &start,
            &mut solver(&fixture.model),
            &translation,
            &mut SolveOptions::default(),
        )
        .expect("partial path");
    let (_, fraction) = config
        .to_pose(
            &start,
            &mut solver(&fixture.model),
            &translated(&start_pose, 2.0),
            &mut SolveOptions::default(),
        )
        .expect("partial path");

    // The return value is a distance, not a fraction: upstream's
    // `Distance(distance) * Percentage(...)`. Deriving the expectation
    // from the equivalent `to_pose` call keeps this independent of how
    // many steps the path was cut into.
    let expected = translation.norm() * fraction.value();
    assert!(
        (travelled - expected).abs() < 1e-15,
        "reported {travelled} m against {expected} m for fraction {}",
        fraction.value()
    );
    assert!(
        travelled < translation.norm(),
        "the path does not finish, so it cannot have travelled the whole 2 m"
    );
}

#[test]
fn a_local_frame_translation_is_rotated_into_the_start_pose() {
    let fixture = Fixture::new();
    let start = fixture.start();
    let start_pose = tip_pose(&start, &fixture.tip);
    let translation = Vector3::new(0.10, 0.0, 0.0);
    let mut config =
        CartesianInterpolator::new("panda_arm", &fixture.tip, MaxEefStep::from_step_size(0.01));
    config.global_reference_frame = false;

    let (path, travelled) = config
        .along_translation(
            &start,
            &mut solver(&fixture.model),
            &translation,
            &mut SolveOptions::default(),
        )
        .expect("reachable path");

    let mut expected = start_pose;
    expected.translation.vector += start_pose.rotation * translation;
    let (error, rotation) = errors(
        &tip_pose(path.last().expect("waypoints"), &fixture.tip),
        &expected,
    );
    assert!(
        error < LOCAL_FRAME_TOL && rotation < LOCAL_FRAME_TOL,
        "local-frame endpoint is off by {error:e} m and {rotation:e} rad"
    );
    // The two frames genuinely disagree on this fixture, so the assertion
    // above is not satisfied by the global reading as well.
    let global_endpoint = translated(&start_pose, 0.10);
    assert!(
        (expected.translation.vector - global_endpoint.translation.vector).norm() > 0.05,
        "the fixture's start orientation must make the two frames differ"
    );
    assert!(
        (travelled - translation.norm()).abs() < 1e-15,
        "the local-frame path is reachable in full"
    );
}

#[test]
fn through_waypoints_accumulates_per_segment_and_drops_the_seam() {
    let fixture = Fixture::new();
    let start = fixture.start();
    let start_pose = tip_pose(&start, &fixture.tip);
    let near = translated(&start_pose, 0.05);
    let far = translated(&start_pose, 2.0);
    let config =
        CartesianInterpolator::new("panda_arm", &fixture.tip, MaxEefStep::from_step_size(0.05));

    let (path, fraction) = config
        .through_waypoints(
            &start,
            &mut solver(&fixture.model),
            &[near, far],
            &mut SolveOptions::default(),
        )
        .expect("partial path");

    // The same two segments run on their own. The second starts from the
    // first's end state, exactly as the accumulating loop does.
    let (first, first_fraction) = config
        .to_pose(
            &start,
            &mut solver(&fixture.model),
            &near,
            &mut SolveOptions::default(),
        )
        .expect("reachable segment");
    let (second, second_fraction) = config
        .to_pose(
            first.last().expect("waypoints"),
            &mut solver(&fixture.model),
            &far,
            &mut SolveOptions::default(),
        )
        .expect("partial segment");

    assert_eq!(
        first_fraction.value(),
        1.0,
        "the fixture's first waypoint must be reachable for the branch under test"
    );
    assert!(second_fraction.value() < 1.0, "and the second must not be");
    // A completed segment contributes `(i + 1) / n` outright; the first
    // incomplete one contributes its own fraction over `n` and ends the
    // loop.
    assert_eq!(
        fraction.value(),
        0.5 + second_fraction.value() / 2.0,
        "segment fractions must accumulate, not overwrite"
    );
    assert_eq!(
        path.len(),
        first.len() + second.len() - 1,
        "the seam state belongs to both segments and must appear once"
    );
}

/// The path parameter of the first interval on the 0.10 m / 0.01-step
/// fixture: `floor(0.10 / 0.01) + 1 == 11` steps, so `width == 1/11`. It
/// is the exact value `max_resolution` is compared against on the first
/// bisection decision, which is why both tests below straddle it rather
/// than picking a round number near it.
const FIRST_INTERVAL_WIDTH: f64 = 1.0 / 11.0;

/// A translational precision the fixture's first interval cannot meet at
/// full width but does meet at half width. Measured by scanning: at
/// `1e-4` the full-width interval is already accepted (12 states, no
/// bisection at all), and at `1e-6` one bisection is not enough either.
/// `1e-5` is the only decade that separates the two sides of the
/// `max_resolution` gate on this fixture.
const ONE_BISECTION_TRANSLATIONAL: f64 = 1e-5;

#[test]
fn an_interval_at_max_resolution_is_rejected_rather_than_bisected() {
    // Both sides run the identical path, identical solver and identical
    // `translational` precision; the *only* difference is whether
    // `max_resolution` sits above or below `FIRST_INTERVAL_WIDTH`. So
    // nothing but the `width < max_resolution` gate can explain the
    // difference in outcome.
    let fixture = Fixture::new();
    let start = fixture.start();
    let target = translated(&tip_pose(&start, &fixture.tip), 0.10);

    let run = |max_resolution: f64| {
        let mut config =
            CartesianInterpolator::new("panda_arm", &fixture.tip, MaxEefStep::from_step_size(0.01));
        config.precision = moveit_kinematics::CartesianPrecision {
            translational: ONE_BISECTION_TRANSLATIONAL,
            // Rotation is taken out of the decision: this is a pure
            // translation, and a rotational bound that can also fail would
            // make the failing side ambiguous about which check rejected.
            rotational: 1.0,
            max_resolution,
        };
        let (path, fraction) = config
            .to_pose(
                &start,
                &mut solver(&fixture.model),
                &target,
                &mut SolveOptions::default(),
            )
            .expect("the walk itself must succeed; only the interval is rejected");
        (path.len(), fraction.value())
    };

    // Above the width: the interval fails the deviation check and is not
    // allowed to bisect, so the walk stops before its first waypoint.
    assert_eq!(
        run(FIRST_INTERVAL_WIDTH * 1.0001),
        (1, 0.0),
        "a `max_resolution` above the interval width must reject at the first interval, \
         leaving only the start state and a zero fraction"
    );

    // Below the width: the same interval bisects once, both halves are
    // accepted, and every one of the 11 intervals does the same -- 12
    // waypoints plus their 11 midpoints.
    assert_eq!(
        run(FIRST_INTERVAL_WIDTH * 0.9999),
        (23, 1.0),
        "a `max_resolution` below the interval width must bisect and reach the target"
    );
}

/// The fraction the deep-bisection case stops at: the port halves the
/// interval twelve times before its leftmost leaf is accepted, and
/// `1/11 / 4096` is that leaf's path parameter. The equality below is
/// exact, not approximate: `half_width` is only ever produced by halving,
/// which is exact in binary, and `percentage - half_width` on the leftmost
/// branch is a Sterbenz subtraction of a value from twice itself, also
/// exact. Measured `2.21946022727272733e-5` against a computed
/// `2.21946022727272733e-5`.
const DEEPEST_ACCEPTED_LEAF: f64 = FIRST_INTERVAL_WIDTH / 4096.0;

#[test]
fn a_rejected_path_keeps_the_fraction_its_deepest_accepted_leaf_reached() {
    // This number is NOT a parity value: upstream reports `0.0` here.
    // Its `last_valid_percentage = percentage` sits after the `break`
    // (`cartesian_interpolator.cpp:268`, read at the pinned `e017c91e`), so
    // a walk that fails on its first interval reports zero while still
    // returning the waypoints its accepted sub-intervals pushed. Declining
    // that is `validate-and-improve-interval-percentage-discarded` in
    // `doc/upstream-bugs.md`, whose stated invariant is what this test
    // pins: the returned fraction is the path parameter of the last
    // waypoint in the returned trajectory, on success and failure alike.
    // That entry called this case constructible but unconstructed; this is
    // the construction.
    //
    // `translational` here is tight enough that no interval survives to
    // full width, but the recursion still accepts one leaf twelve levels
    // down before its sibling runs out of resolution. That makes this the
    // case that separates the two candidate owners of the reported
    // fraction: the *accepted leaf* writes `1/11 / 4096`, while
    // any write made on entry to an interval -- upstream's in/out
    // `double& percentage` -- would report the enclosing interval's `1/11`
    // instead, four thousand times larger.
    let fixture = Fixture::new();
    let start = fixture.start();
    let target = translated(&tip_pose(&start, &fixture.tip), 0.10);

    let mut config =
        CartesianInterpolator::new("panda_arm", &fixture.tip, MaxEefStep::from_step_size(0.01));
    config.precision = moveit_kinematics::CartesianPrecision {
        translational: 1e-8,
        rotational: 1.0,
        max_resolution: 1e-5,
    };

    let (path, fraction) = config
        .to_pose(
            &start,
            &mut solver(&fixture.model),
            &target,
            &mut SolveOptions::default(),
        )
        .expect("the walk itself must succeed; only the interval is rejected");

    assert_eq!(
        fraction.value(),
        DEEPEST_ACCEPTED_LEAF,
        "the reported fraction must be the accepted leaf's path parameter, \
         not the rejected interval's"
    );
    assert_eq!(
        path.len(),
        2,
        "the accepted leaf's state must be kept even though the path as a whole failed"
    );
}
