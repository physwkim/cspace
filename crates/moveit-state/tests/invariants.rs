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

use moveit_model::{MeshSearchPaths, RobotModel};
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
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
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
    // Bisected per-constant (§85.3): dropping this to 1e-12 still passes;
    // 1e-15 fails at a real diff of ~2.22e-14
    // (0.8268795405320245 vs 0.8268795405320025) -- genuine `sin` rounding
    // noise from the wrap, not a hidden proportional tolerance (§79). 1e-9
    // sits ~4.5 decades above that noise floor; no change needed.
    assert_relative_eq!(wrapped.sin(), 1000.0_f64.sin(), epsilon = 1e-9);
    // Same bisection, independently: 1e-12 passes, 1e-15 fails at a real
    // diff of ~3.22e-14 (0.5623790762906707 vs 0.5623790762907029).
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
    // Bisected per-constant (§85.3) down through 1e-12, 1e-15, and 0.0: all
    // passed at `assert_relative_eq!`, `before`/`after` bit-for-bit
    // identical. Not a coincidence of this input -- this doc comment's own
    // opening paragraph is the reason: harmonizing never marks link
    // transforms dirty, so `update()` after `harmonize_positions()` returns
    // the same cached transform rather than recomputing one through sin/cos
    // of the rewrapped angle. `assert_eq!` says so directly, rather than
    // leaving a `1e-9` epsilon for someone to later read as "close enough"
    // and loosen.
    assert_eq!(before, after);
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

/// Same `variable_count <= positions.size()` boundary as
/// `set_variable_velocities_accepts_a_longer_than_needed_slice_and_ignores_the_tail`
/// -- upstream's `assert(getVariableCount() <= position.size())` tolerates a
/// longer buffer, only ever reading the leading `variable_count` entries.
/// Before this was fixed, `copy_from_slice` required an exact-length match
/// and panicked here instead of truncating.
#[test]
fn set_variable_positions_accepts_a_longer_than_needed_slice_and_ignores_the_tail() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let n = model.variable_count();
    let expected = vec![0.3; n];
    let mut positions = expected.clone();
    positions.extend([f64::NAN, f64::NAN]);

    state.set_variable_positions(&positions);

    assert_eq!(state.positions(), expected.as_slice());
}

/// The other side of the boundary: a `positions` shorter than
/// `variable_count` panics deterministically rather than reproducing
/// upstream's release-mode out-of-bounds `memcpy` read -- see
/// `set_variable_velocities_panics_on_a_shorter_than_needed_slice` for the
/// same pin on the sibling setter.
#[test]
#[should_panic]
fn set_variable_positions_panics_on_a_shorter_than_needed_slice() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let n = model.variable_count();
    assert!(n > 1, "fixture must have more than one variable");

    state.set_variable_positions(&vec![0.3; n - 1]);
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
    // Bisected per-constant (§85.3) down through 1e-12, 1e-15, and 0.0: all
    // passed at `assert_relative_eq!` -- `norm_sqr_after` is exactly `1.0`,
    // bit-for-bit, for this measured input (`(2.0, 3.0, 0.0, 1.0)`
    // renormalized). Unlike the transform site above, this is not claimed
    // structural for every possible input, only measured exact for this
    // one; `assert_eq!` records what was actually measured rather than
    // leaving an epsilon that reads as "approximately".
    assert_eq!(norm_sqr_after, 1.0);
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

// ---- Velocity / acceleration / effort --------------------------------

/// Before anything is ever set, `has_velocities`/`has_accelerations`/
/// `has_effort` all report `false` — the boundary a freshly constructed
/// `RobotState` must start at, matching upstream's default-constructed
/// `has_velocity_ = has_acceleration_ = has_effort_ = false`.
#[test]
fn fresh_state_has_no_velocity_acceleration_or_effort() {
    let model = panda();
    let state = RobotState::new(&model);
    assert!(!state.has_velocities());
    assert!(!state.has_accelerations());
    assert!(!state.has_effort());
}

/// Setting a single variable's velocity by name flips `has_velocities` and
/// is readable back both by name and by the model's global variable index,
/// without disturbing any other variable's velocity (still `0.0`).
#[test]
fn set_variable_velocity_is_readable_by_name_and_by_index() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let index = model.variable_index("panda_joint1").unwrap();

    state.set_variable_velocity("panda_joint1", 1.5).unwrap();

    assert!(state.has_velocities());
    assert_eq!(state.variable_velocity("panda_joint1").unwrap(), 1.5);
    assert_eq!(state.variable_velocity_at(index), 1.5);
    let other_index = model.variable_index("panda_joint2").unwrap();
    assert_eq!(
        state.variable_velocity_at(other_index),
        0.0,
        "an unrelated variable's velocity must stay at its zero default"
    );
}

/// `set_variable_velocities` (whole-array) is the bulk counterpart to the
/// per-variable setter above: one call replaces every velocity and flips
/// `has_velocities`.
#[test]
fn set_variable_velocities_replaces_the_whole_array() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let n = model.variable_count();

    state.set_variable_velocities(&vec![0.5; n]);

    assert!(state.has_velocities());
    assert!(state.velocities().iter().all(|&v| v == 0.5));
}

/// Upstream's precondition for this family is `variable_count <=
/// values.size()`, not equality (`assert(getVariableCount() <=
/// velocity.size())`, `robot_state.hpp`) -- a caller-supplied buffer longer
/// than `variable_count` is valid there, and only the leading
/// `variable_count` entries are ever read. Before this was fixed,
/// `copy_from_slice` required an exact-length match and panicked on this
/// input instead of truncating it.
#[test]
fn set_variable_velocities_accepts_a_longer_than_needed_slice_and_ignores_the_tail() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let n = model.variable_count();
    let mut values = vec![0.5; n];
    values.extend([f64::NAN, f64::NAN, f64::NAN]);

    state.set_variable_velocities(&values);

    assert!(state.velocities().iter().all(|&v| v == 0.5));
}

/// The other side of the same boundary: a `values` *shorter* than
/// `variable_count` is upstream UB in a release build (its debug-only
/// `assert` is compiled out, and the underlying `memcpy` over-reads). This
/// port panics deterministically instead, in every build profile -- pinning
/// that this fix's move to `&values[..len]` still rejects a short input
/// rather than silently accepting it (which a naive `values.len().min(len)`
/// truncation would have done).
#[test]
#[should_panic]
fn set_variable_velocities_panics_on_a_shorter_than_needed_slice() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let n = model.variable_count();
    assert!(n > 1, "fixture must have more than one variable");

    state.set_variable_velocities(&vec![0.5; n - 1]);
}

/// Same `variable_count <= values.size()` boundary as
/// `set_variable_velocities_accepts_a_longer_than_needed_slice_and_ignores_the_tail`,
/// on `set_variable_accelerations`'s own buffer -- its doc points at
/// `set_variable_velocities`'s `# Panics` section for the same upstream
/// `assert(getVariableCount() <= acceleration.size())` precondition.
#[test]
fn set_variable_accelerations_accepts_a_longer_than_needed_slice_and_ignores_the_tail() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let n = model.variable_count();
    let mut values = vec![0.6; n];
    values.push(f64::NAN);

    state.set_variable_accelerations(&values);

    assert!(state.accelerations().iter().all(|&v| v == 0.6));
}

/// Same boundary again, on `set_variable_efforts`'s own buffer -- its doc
/// also points at `set_variable_velocities`'s `# Panics` section for the
/// identical upstream `assert(getVariableCount() <= effort.size())`
/// precondition.
#[test]
fn set_variable_efforts_accepts_a_longer_than_needed_slice_and_ignores_the_tail() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let n = model.variable_count();
    let mut values = vec![0.7; n];
    values.push(f64::NAN);

    state.set_variable_efforts(&values);

    assert!(state.effort().iter().all(|&v| v == 0.7));
}

/// `invertVelocity` negates every velocity in place without disturbing
/// `has_velocities` or acceleration -- see `RobotState::invert_velocity`'s
/// doc comment on why acceleration is untouched even though this exists for
/// `RobotTrajectory::reverse`'s sake.
#[test]
fn invert_velocity_negates_every_velocity_and_leaves_acceleration_alone() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_variable_velocity("panda_joint1", 1.5).unwrap();
    state.set_variable_velocity("panda_joint2", -2.0).unwrap();
    state
        .set_variable_acceleration("panda_joint1", 3.0)
        .unwrap();

    state.invert_velocity();

    assert!(state.has_velocities());
    assert_eq!(state.variable_velocity("panda_joint1").unwrap(), -1.5);
    assert_eq!(state.variable_velocity("panda_joint2").unwrap(), 2.0);
    assert_eq!(state.variable_acceleration("panda_joint1").unwrap(), 3.0);
}

/// The boundary `invert_velocity`'s `has_velocity` guard exists for: a
/// freshly constructed state has never had velocity set, and inverting it
/// must stay a no-op rather than materializing an all-`-0.0` velocity array
/// and flipping `has_velocities` to `true`.
#[test]
fn invert_velocity_on_a_state_with_no_velocity_set_is_a_no_op() {
    let model = panda();
    let mut state = RobotState::new(&model);

    state.invert_velocity();

    assert!(!state.has_velocities());
    assert!(state.velocities().iter().all(|&v| v == 0.0));
}

/// The invariant this task exists to close: upstream aliases acceleration
/// and effort onto one buffer, so setting one clobbers the other
/// (`hasAccelerations() == true` implies `hasEffort() == false`, always).
/// This port gives them independent storage instead — setting acceleration
/// then effort (or the reverse order) must leave *both* set, with *both*
/// values intact, not just the most recently written one.
#[test]
fn acceleration_and_effort_do_not_alias() {
    let model = panda();
    let mut state = RobotState::new(&model);

    state
        .set_variable_acceleration("panda_joint1", 3.0)
        .unwrap();
    state.set_variable_effort("panda_joint1", 7.0).unwrap();

    assert!(
        state.has_accelerations(),
        "upstream's aliasing would have cleared this when effort was set"
    );
    assert!(state.has_effort());
    assert_eq!(state.variable_acceleration("panda_joint1").unwrap(), 3.0);
    assert_eq!(state.variable_effort("panda_joint1").unwrap(), 7.0);
}

/// Same invariant, opposite write order — the aliasing upstream implements
/// is order-dependent (whichever was set most recently wins), so both
/// orders must be checked to confirm this port has no order dependence at
/// all.
#[test]
fn effort_then_acceleration_also_do_not_alias() {
    let model = panda();
    let mut state = RobotState::new(&model);

    state.set_variable_effort("panda_joint1", 7.0).unwrap();
    state
        .set_variable_acceleration("panda_joint1", 3.0)
        .unwrap();

    assert!(state.has_accelerations());
    assert!(
        state.has_effort(),
        "upstream's aliasing would have cleared this when acceleration was set"
    );
    assert_eq!(state.variable_acceleration("panda_joint1").unwrap(), 3.0);
    assert_eq!(state.variable_effort("panda_joint1").unwrap(), 7.0);
}

/// `joint_velocity`/`joint_acceleration`/`joint_effort` return a joint's
/// own slice, in the same order `joint_position` already does — the
/// per-joint read path this task's accessor list calls for alongside the
/// per-variable one.
#[test]
fn joint_scoped_accessors_return_the_joints_own_slice() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_variable_velocity("panda_joint1", 1.0).unwrap();
    state
        .set_variable_acceleration("panda_joint1", 2.0)
        .unwrap();
    state.set_variable_effort("panda_joint1", 3.0).unwrap();

    assert_eq!(state.joint_velocity("panda_joint1").unwrap(), &[1.0]);
    assert_eq!(state.joint_acceleration("panda_joint1").unwrap(), &[2.0]);
    assert_eq!(state.joint_effort("panda_joint1").unwrap(), &[3.0]);
}

/// An unknown joint/variable name is `Error::UnknownName`, not a panic —
/// checked across every new accessor family, matching how the existing
/// position accessors already behave.
#[test]
fn unknown_name_is_an_error_not_a_panic_for_every_new_accessor() {
    let model = panda();
    let mut state = RobotState::new(&model);

    assert!(state.variable_velocity("no_such_joint").is_err());
    assert!(state.variable_acceleration("no_such_joint").is_err());
    assert!(state.variable_effort("no_such_joint").is_err());
    assert!(state.set_variable_velocity("no_such_joint", 1.0).is_err());
    assert!(
        state
            .set_variable_acceleration("no_such_joint", 1.0)
            .is_err()
    );
    assert!(state.set_variable_effort("no_such_joint", 1.0).is_err());
    assert!(state.joint_velocity("no_such_joint").is_err());
    assert!(state.joint_acceleration("no_such_joint").is_err());
    assert!(state.joint_effort("no_such_joint").is_err());
}

/// The boundary `enforce_bounds`/`satisfies_bounds` must now respect:
/// velocity bounds are checked *only* once `has_velocities()` is true.
/// Before any velocity is ever set, an out-of-bounds *default* velocity
/// cannot occur (defaults are `0.0`, always in-bounds), so this drives the
/// boundary explicitly by setting an out-of-range velocity and checking
/// both sides of the `has_velocities` gate.
#[test]
fn satisfies_bounds_checks_velocity_only_once_velocity_is_set() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    // panda_joint1's velocity limit is 2.3925 rad/s (fixture URDF).
    assert!(
        state.satisfies_bounds(0.0),
        "a state with no velocity ever set must satisfy bounds regardless of what a raw \
         out-of-range velocity write would look like, since has_velocities() is still false"
    );

    state.set_variable_velocity("panda_joint1", 100.0).unwrap();
    assert!(
        !state.satisfies_bounds(0.0),
        "once has_velocities() is true, an out-of-range velocity must be caught"
    );

    state.enforce_bounds();
    assert!(
        state.satisfies_bounds(0.0),
        "post-clamp state must satisfy both position and velocity bounds"
    );
    let clamped = state.variable_velocity("panda_joint1").unwrap();
    assert!(
        clamped <= 2.3925,
        "velocity must have been pulled back to its bound, got {clamped}"
    );
}

/// `enforce_bounds`'s velocity clamp must not dirty the transform cache —
/// upstream's `enforceVelocityBounds` never calls
/// `markDirtyJointTransforms`, unlike the position clamp right next to it.
/// A position that already satisfies its bounds must produce the exact
/// same global link transform before and after an out-of-bounds velocity
/// is clamped.
#[test]
fn enforce_bounds_velocity_clamp_does_not_perturb_the_transform() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let before = state.update().global_link_transform("panda_link8").unwrap();

    state.set_variable_velocity("panda_joint1", 100.0).unwrap();
    state.enforce_bounds();
    let after = state.update().global_link_transform("panda_link8").unwrap();

    assert_eq!(
        before, after,
        "clamping an out-of-bounds velocity must not move the link transform"
    );
}
