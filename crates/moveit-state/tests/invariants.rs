// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Invariant-boundary tests: bounded vs unbounded, wrapped vs unwrapped
//! continuous revolute, mimic vs independent, quaternion normalization at
//! and away from identity, and the PR2 planar joint's per-variable bound
//! shape. These are boundary conditions the FK-parity fixtures in
//! `tests/fk_parity.rs` do not target directly (a randomly sampled state is
//! already bounds-respecting and mimic-consistent by construction, so it
//! never exercises the *correction* paths tested here).

use std::fs;

use approx::assert_relative_eq;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use moveit_model::RobotModel;
use moveit_srdf::SrdfModel;
use moveit_state::{Posed, RobotState};

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
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf).expect("fixture model must build")
}

fn panda() -> RobotModel {
    build_model("panda.urdf", "panda.srdf")
}

fn pr2() -> RobotModel {
    build_model("pr2.urdf", "pr2.srdf")
}

// ---- Bounded vs unbounded -------------------------------------------------

/// A bounded revolute joint (panda's `panda_joint1`, bounds roughly
/// `[-2.897, 2.897]`): `enforce_bounds` clamps an out-of-range value, and
/// `satisfies_bounds` reports the violation before it is clamped.
#[test]
fn enforce_bounds_clamps_a_bounded_revolute_joint() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    state.set_variable_position("panda_joint1", 100.0).unwrap();

    assert!(
        !state.satisfies_bounds(0.0),
        "100.0 rad must violate bounds"
    );
    state.enforce_bounds();
    assert!(
        state.satisfies_bounds(0.0),
        "post-clamp state must satisfy bounds"
    );
    let clamped = state.variable_position("panda_joint1").unwrap();
    assert!(
        clamped < 100.0,
        "value must have been pulled back to the bound"
    );
}

/// An unbounded continuous revolute joint (PR2's `bl_caster_rotation_joint`,
/// `position_bounded == false`): `satisfies_bounds` accepts any value
/// unconditionally (`RevoluteJoint::satisfies_position_bounds` returns
/// `true` without even inspecting `value` when `continuous`), but
/// `enforce_bounds` is *not* therefore a no-op — it still wraps the stored
/// angle into `[-pi, pi]` (`RevoluteJoint::enforce_position_bounds`'s
/// `continuous` branch), the same canonicalization `harmonize_positions`
/// does. "Unbounded" means "no value is rejected", not "the value is
/// never touched".
#[test]
fn enforce_bounds_wraps_an_unbounded_continuous_joint_into_pi_range() {
    let model = pr2();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    state
        .set_variable_position("bl_caster_rotation_joint", 1000.0)
        .unwrap();

    assert!(
        state.satisfies_bounds(0.0),
        "an unbounded continuous joint satisfies bounds at any value"
    );
    state.enforce_bounds();
    let wrapped = state.variable_position("bl_caster_rotation_joint").unwrap();
    assert!(
        (-std::f64::consts::PI..=std::f64::consts::PI).contains(&wrapped),
        "enforce_bounds must wrap a continuous joint's value into [-pi, pi], got {wrapped}"
    );
    assert_relative_eq!(wrapped.sin(), 1000.0_f64.sin(), epsilon = 1e-9);
    assert_relative_eq!(wrapped.cos(), 1000.0_f64.cos(), epsilon = 1e-9);
}

// ---- Wrapped vs unwrapped continuous revolute -----------------------------

/// `harmonize_positions` rewraps a continuous joint's stored angle into
/// `(-pi, pi]` (`normalize_angle`'s range, see `moveit-model`'s
/// `planar.rs`) but must not change the geometry it produces: the global
/// link transform before and after harmonizing must match, because
/// harmonizing is documented (`RobotState::harmonizePosition`, upstream)
/// to never mark link transforms dirty.
#[test]
fn harmonize_positions_rewraps_without_changing_the_transform() {
    let model = pr2();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    let unwrapped = 4.0 * std::f64::consts::PI + 0.3; // several full turns past zero
    state
        .set_variable_position("bl_caster_rotation_joint", unwrapped)
        .unwrap();
    let before = state
        .update()
        .global_link_transform("bl_caster_rotation_link")
        .unwrap();

    state.harmonize_positions();
    let wrapped = state.variable_position("bl_caster_rotation_joint").unwrap();
    assert!(
        (-std::f64::consts::PI..=std::f64::consts::PI).contains(&wrapped),
        "harmonized angle {wrapped} must land in [-pi, pi]"
    );
    assert_ne!(
        wrapped, unwrapped,
        "harmonizing must actually change the stored value here"
    );

    let after = state
        .update()
        .global_link_transform("bl_caster_rotation_link")
        .unwrap();
    assert_relative_eq!(before, after, epsilon = 1e-9);
}

// ---- Mimic vs independent --------------------------------------------------

/// Setting a mimic joint's master propagates to the follower
/// (`l_gripper_r_finger_joint` mimics `l_gripper_l_finger_joint`,
/// multiplier 1.0, offset 0.0 — oracle-verified in `PORTING-PLAN.md` §8.4)
/// but leaves an unrelated independent joint untouched.
#[test]
fn setting_a_mimic_master_propagates_only_to_its_followers() {
    let model = pr2();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let independent_before = state.variable_position("bl_caster_rotation_joint").unwrap();

    state
        .set_variable_position("l_gripper_l_finger_joint", 0.4)
        .unwrap();

    assert_eq!(
        state.variable_position("l_gripper_r_finger_joint").unwrap(),
        0.4,
        "follower must track master * 1.0 + 0.0"
    );
    assert_eq!(
        state.variable_position("bl_caster_rotation_joint").unwrap(),
        independent_before,
        "an independent joint must not move when an unrelated master changes"
    );
}

/// `set_variable_positions` (the bulk, whole-array overload) does *not*
/// propagate mimic — the caller's array is trusted to already be
/// mimic-consistent (upstream: "the full state includes mimic joint
/// values"). A follower's slot must come back exactly as given, not
/// re-derived from its master.
#[test]
fn bulk_set_variable_positions_does_not_propagate_mimic() {
    let model = pr2();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    let mut positions = state.positions().to_vec();
    let master_index = model.variable_index("l_gripper_l_finger_joint").unwrap();
    let follower_index = model.variable_index("l_gripper_r_finger_joint").unwrap();
    positions[master_index] = 0.4;
    positions[follower_index] = 0.1; // deliberately inconsistent with the master
    state.set_variable_positions(&positions);

    assert_eq!(
        state.variable_position("l_gripper_r_finger_joint").unwrap(),
        0.1,
        "bulk overload must not silently re-derive a mimic follower"
    );
}

/// `set_to_default_values` re-derives every mimic joint's value from its
/// master's *new* default, even when the mimic's own slot held a stale
/// value from a previous write. This was verified against a live oracle
/// (two `fk` requests in the same session: one that set the master
/// off-default, one back to the default case) before writing this test —
/// `RobotModel::getVariableDefaultPositions` calls
/// `RobotModel::updateMimicJoints` internally upstream, easy to miss on a
/// first read since it is a private two-line tail call.
#[test]
fn set_to_default_values_rederives_mimic_from_a_previously_randomized_state() {
    let model = pr2();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let default_follower = state.variable_position("l_gripper_r_finger_joint").unwrap();

    state
        .set_variable_position("l_gripper_l_finger_joint", 0.4)
        .unwrap();
    assert_eq!(
        state.variable_position("l_gripper_r_finger_joint").unwrap(),
        0.4
    );

    state.set_to_default_values();
    assert_eq!(
        state.variable_position("l_gripper_r_finger_joint").unwrap(),
        default_follower,
        "mimic follower must track the master's default, not stay stuck at 0.4"
    );
}

// ---- Quaternion normalization at and away from identity -------------------

/// The default floating-joint quaternion (identity: `rot_w = 1`, others
/// `0`) is already normalized, so `satisfies_bounds` accepts it without
/// `enforce_bounds` changing anything.
#[test]
fn default_floating_quaternion_is_already_normalized() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    assert!(state.satisfies_bounds(0.0));
    let before = state.positions().to_vec();
    state.enforce_bounds();
    assert_eq!(
        state.positions(),
        before.as_slice(),
        "enforce_bounds must be a no-op on an already-normalized identity quaternion"
    );
}

/// Setting the four quaternion variables independently (as
/// `set_variable_position` allows, one at a time — there is no API that
/// forces them to move together) breaks normalization; `enforce_bounds`
/// must repair it, because `FloatingJoint::enforce_position_bounds` always
/// normalizes first (see that type's doc comment on why a per-variable
/// write cannot be trusted to preserve the unit-quaternion invariant).
#[test]
fn enforce_bounds_renormalizes_a_quaternion_broken_by_independent_writes() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    state
        .set_variable_position("virtual_joint/rot_x", 2.0)
        .unwrap();
    state
        .set_variable_position("virtual_joint/rot_y", 3.0)
        .unwrap();
    state
        .set_variable_position("virtual_joint/rot_z", 0.0)
        .unwrap();
    state
        .set_variable_position("virtual_joint/rot_w", 1.0)
        .unwrap();

    let norm_sqr_before: f64 = ["rot_x", "rot_y", "rot_z", "rot_w"]
        .iter()
        .map(|v| {
            let value = state
                .variable_position(&format!("virtual_joint/{v}"))
                .unwrap();
            value * value
        })
        .sum();
    assert!(
        (norm_sqr_before - 1.0).abs() > 1e-6,
        "the four independent writes must actually have broken normalization"
    );

    state.enforce_bounds();

    let norm_sqr_after: f64 = ["rot_x", "rot_y", "rot_z", "rot_w"]
        .iter()
        .map(|v| {
            let value = state
                .variable_position(&format!("virtual_joint/{v}"))
                .unwrap();
            value * value
        })
        .sum();
    assert_relative_eq!(norm_sqr_after, 1.0, epsilon = 1e-9);
}

// ---- PR2 planar joint: mixed bounded/unbounded within one joint -----------

/// `world_joint` (PR2's planar virtual joint) reports `x`/`y` as
/// `position_bounded == true` with infinite numeric range, and `theta` as
/// `position_bounded == false` with a finite `[-pi, pi]` range
/// (oracle-verified, `PORTING-PLAN.md` §8.4) — `position_bounded` and the
/// numeric range it might be expected to gate are independent axes, the
/// same independence the task brief calls out for the floating joint's
/// translation, just inverted: here the flag reads `true` over an infinite
/// range, and `false` over a finite one. `PlanarJoint::satisfies_position_bounds`
/// checks all three variables against their numeric bounds unconditionally
/// and never consults the flag, so x/y truly accept anything while theta
/// does not, regardless of what their respective `position_bounded` values
/// might suggest.
#[test]
fn pr2_planar_joint_x_y_and_theta_are_unbounded_for_different_reasons() {
    let model = pr2();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    state
        .set_joint_positions("world_joint", &[1.0e6, -1.0e6, 0.0])
        .unwrap();
    assert!(
        state.satisfies_bounds(0.0),
        "x/y must accept huge values: their numeric range is truly infinite"
    );

    state
        .set_variable_position("world_joint/theta", 3.0 * std::f64::consts::PI)
        .unwrap();
    assert!(
        !state.satisfies_bounds(0.0),
        "theta must reject a value outside [-pi, pi] even though position_bounded == false"
    );
    state.enforce_bounds();
    let wrapped = state.variable_position("world_joint/theta").unwrap();
    assert!(
        (-std::f64::consts::PI..=std::f64::consts::PI).contains(&wrapped),
        "enforce_bounds must wrap theta back into [-pi, pi], got {wrapped}"
    );
}

// ---- PR2 whole-model random positions: bounds + mimic together -----------

/// Sampling the whole PR2 model (95 links/joints, 19 unbounded continuous
/// revolute joints, 6 mimic joints) must produce a state that satisfies
/// every bound and keeps every one of the 6 known mimic pairs consistent —
/// exercising both invariants together, at model scale, the way a real
/// planner's `setToRandomPositions()` call would.
#[test]
fn pr2_random_positions_satisfy_bounds_and_mimic_consistency() {
    let model = pr2();
    let mut state = RobotState::new(&model);

    let mimic_pairs = [
        ("l_gripper_l_finger_joint", "l_gripper_l_finger_tip_joint"),
        ("l_gripper_l_finger_joint", "l_gripper_r_finger_joint"),
        ("l_gripper_l_finger_joint", "l_gripper_r_finger_tip_joint"),
        ("r_gripper_l_finger_joint", "r_gripper_l_finger_tip_joint"),
        ("r_gripper_l_finger_joint", "r_gripper_r_finger_joint"),
        ("r_gripper_l_finger_joint", "r_gripper_r_finger_tip_joint"),
    ];

    let mut rng = ChaCha8Rng::seed_from_u64(1);
    for round in 0..20 {
        state.set_to_random_positions_with(&mut rng);
        assert!(
            state.satisfies_bounds(0.0),
            "round {round}: bounds violated"
        );
        for (master, follower) in mimic_pairs {
            assert_eq!(
                state.variable_position(follower).unwrap(),
                state.variable_position(master).unwrap(),
                "round {round}: '{follower}' must equal '{master}' (multiplier 1.0, offset 0.0)"
            );
        }
    }
}

/// Structural check that a [`Posed`] view is `Send + Sync` — the whole
/// point of the `Posed` split (`PORTING-PLAN.md` §8.2.1, item 5): a
/// collision checker can fan a read-only view out across threads.
#[test]
fn posed_is_send_and_sync() {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<Posed<'static, 'static>>();
}
