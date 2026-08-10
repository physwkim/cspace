// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! What a broad-phase data structure would have to beat, measured per robot.
//!
//! `crates/cspace-collision/src/parry.rs`'s module doc, deviation 7:
//!
//! > there is no broadphase here at all; every ACM-permitted pair is
//! > evaluated in link/object order every time
//!
//! `pair_can_touch` (`e06dfa8e`, `0fc74a40`) did not change that. It skips
//! the *narrow phase* for a pair whose bounds are apart; the pair is still
//! enumerated, and its `Aabb` tests are still paid. What a
//! `fcl::DynamicAABBTreeCollisionManager` would remove is the enumeration
//! itself -- the `n^2` term -- in exchange for refitting and traversing a
//! tree on every check, since every link moves on every check.
//!
//! Whether that trade wins is a question about `n`, and this repository's `n`
//! runs from 6 collision links to 54. The fanuc/cage measurement that
//! motivated the item is at the bottom of that range, where a tree is least
//! likely to pay, so this walks the whole range instead of one point:
//!
//! ```text
//! cargo run --release -p cspace-collision --example broadphase_profile
//! ```
//!
//! # What it reports
//!
//! Per robot, per state class, over states drawn with a fixed seed and run
//! through the same public `check_self_collision` every planner calls: the
//! number of states in the class and the median microseconds per call.
//!
//! Three classes, because they answer different questions:
//!
//! - `free` -- states with no self-collision. The planner-relevant one: a
//!   free state has nothing to stop the sweep early, so it enumerates every
//!   permitted pair. This is the class a broad phase would act on.
//! - `hit` -- states that self-collide. `sweep_is_done` stops the sweep at
//!   the first collision, so these enumerate only a prefix.
//! - `exhaustive` -- every sampled state under a cost-enabled request.
//!   `sweep_is_done`'s first clause is `collision && ... && !request.cost`,
//!   so this one never stops early, and it gives a full `n^2` sweep even for
//!   a fixture whose free class is empty.
//!
//! # The split behind the conclusion
//!
//! Splitting a check into per-check setup, pair enumeration and narrow phase
//! needs counters inside `accumulate_collision`, which are not in the shipped
//! code -- the same way `pair_can_touch`'s own quoted `0.039us` came from
//! instrumentation that is not in the tree. Measured that way on 2026-08-10,
//! free class, mean microseconds per check:
//!
//! ```text
//! robot            pairs   setup    enum  narrow   enum%   pairs reaching
//! fanuc               11    0.66    0.71    1.22     28%        0
//! prbt                36    1.10    2.21    0.01     67%        0
//! panda               19    1.05    2.22    0.11     66%        0
//! prbt_pg70           89    1.61    4.65    0.04     74%        0
//! dual_arm_panda     159    1.94    7.79    0.14     79%        0
//! pr2               1383    5.91   65.91  673.60      9%       45
//! ```
//!
//! `enum` is the whole of what a broad phase can remove, and no more: a tree
//! hands the narrow phase the same AABB-overlapping candidate set
//! `pair_can_touch` already selects, so it changes how that set is *found*,
//! never how big it is. Read down the `enum%` column and the trend runs the
//! wrong way for a tree. Where enumeration is most of the check it is also
//! only 2-8us of absolute cost over 19-159 pairs -- the regime where
//! refitting and traversing a tree over 7-22 moving links costs the same
//! order as the linear scan it would replace. Where the pair count is finally
//! large enough for the asymptotics to bite (pr2, 1383 pairs) the enumeration
//! is 9% of the check, and the other 91% is narrow phase on the 45 pairs
//! whose bounds overlap in a state that has no collision in it at all.
//!
//! So the item's own precondition -- is pair enumeration still dominant? --
//! answers yes on the small robots and no on the large one, which is the
//! opposite of the shape that would justify the port. See
//! `doc/handoff-2026-08-10.md` §2.1.
//!
//! The world leg is one box placed clear of the robot rather than a planning
//! scene: a broad phase's `n` is links plus world objects, and adding
//! obstacles moves every robot's row the same way, while the robot's own link
//! count is the variable the question turns on.

use std::collections::BTreeMap;
use std::fs;
use std::sync::Arc;
use std::time::Instant;

use cspace_collision::{
    AllowedCollision, AllowedCollisionMatrix, CollisionEnv, CollisionRequest, LinkPaddingScale,
    ParryCollisionEnv, World,
};
use cspace_geometry::{Cuboid, Isometry3, Shape};
use cspace_model::{MeshSearchPaths, RobotModel};
use cspace_srdf::SrdfModel;
use cspace_state::RobotState;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

/// Enough that a class median is not one outlier, and few enough that pr2 --
/// the slowest row by two orders of magnitude -- still finishes in seconds.
const STATES: usize = 200;

/// Fixed, so two runs of this file compare to each other.
const SEED: u64 = 0x8_2026;

/// Draws used to find the always-colliding pairs, and the share of them a
/// pair must contact in to count as always colliding. `0.98` rather than
/// `1.0` so one grazing draw does not keep a genuinely-always pair enabled.
const ALWAYS_SAMPLE: usize = 200;
const ALWAYS_FRACTION: f64 = 0.98;

/// Checks discarded before timing, so lazy mesh decode and `ObbTree`
/// construction are not charged to the first measured call.
const WARMUP: usize = 20;

/// Every fixture in `fixtures/`, ordered by link count: the trend across that
/// column is this file's whole point, so leaving one out would be leaving out
/// a data point about `n`.
const ROBOTS: &[&str] = &[
    "one_robot",
    "fanuc",
    "prbt",
    "panda",
    "prbt_pg70",
    "dual_arm_panda",
    "pr2",
];

/// The three `moveit_resources_*_description` packages committed under
/// `fixtures/meshes/` -- the same set `tests/collision_parity.rs`'s own
/// helper maps, which is what makes pr2's `<mesh>` links resolve rather than
/// silently drop.
fn fixture_mesh_search_paths() -> MeshSearchPaths {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    MeshSearchPaths::new([
        (
            "moveit_resources_panda_description",
            format!("{meshes_root}/panda_description"),
        ),
        (
            "moveit_resources_fanuc_description",
            format!("{meshes_root}/fanuc_description"),
        ),
        (
            "moveit_resources_pr2_description",
            format!("{meshes_root}/pr2_description"),
        ),
    ])
}

fn build(name: &str) -> (RobotModel, AllowedCollisionMatrix) {
    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
    let urdf_path = format!("{root}/{name}.urdf");
    let srdf_path = format!("{root}/{name}.srdf");
    let urdf_xml = fs::read_to_string(&urdf_path).expect("fixture URDF must be readable");
    let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    let model =
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
            .expect("fixture model must build");
    (model, AllowedCollisionMatrix::from_srdf(&srdf))
}

/// The population a broad phase would prune: unordered pairs of
/// collision-geometry links the ACM does not already suppress.
///
/// [`AllowedCollision::Always`] is upstream's own "skip the pair, no query at
/// all", so those pairs never reach the enumeration a tree would replace and
/// must not be counted as if they did.
fn permitted_pairs(model: &RobotModel, acm: &AllowedCollisionMatrix) -> usize {
    let links: Vec<&str> = model
        .link_models()
        .iter()
        .filter(|l| !l.shapes().is_empty())
        .map(|l| l.name())
        .collect();
    let mut pairs = 0;
    for (i, a) in links.iter().enumerate() {
        for b in &links[i + 1..] {
            if !matches!(acm.entry(a, b), Some(AllowedCollision::Always)) {
                pairs += 1;
            }
        }
    }
    pairs
}

/// Every link pair that contacts in the default state, or in at least
/// [`ALWAYS_FRACTION`] of a uniform sample, added to `base` as
/// [`AllowedCollision::Always`].
///
/// This stands in for the MoveIt setup assistant's `reason="Default"` and
/// `reason="Always"` passes, which is where a shipped SRDF's
/// `<disable_collisions>` entries come from. It is here because
/// `fixtures/pr2.srdf` carries exactly one such entry, against 68 for
/// dual_arm_panda and 34 for panda (`rg -c '<disable_collisions'
/// fixtures/*.srdf`). With nothing disabled, pr2's adjacent links overlap in
/// every configuration, its default state included, so the fixture has no
/// free class at all -- and the one robot whose link count makes the
/// broad-phase question interesting would contribute no free-state row. That
/// is a gap in the fixture, not a property of the robot.
///
/// A harness device, then, not a claim about what any real SRDF contains.
/// Adjacent pairs fall out of it for free: a parent and child sharing a joint
/// origin contact in every draw.
fn augment_acm_with_always_colliding(
    model: &RobotModel,
    base: &AllowedCollisionMatrix,
    env: &ParryCollisionEnv,
) -> AllowedCollisionMatrix {
    let request = CollisionRequest {
        contacts: true,
        max_contacts: usize::MAX,
        max_contacts_per_pair: 1,
        ..CollisionRequest::default()
    };
    let contacting = |state: &RobotState<'_>| {
        env.check_self_collision(&request, &state.clone().update(), &[], Some(base))
            .contacts
            .map(|c| c.by_pair.into_keys().collect::<Vec<_>>())
            .unwrap_or_default()
    };

    let mut state = RobotState::new(model);
    // The `Default` pass, kept separate from the count below because a pair
    // can overlap at home and separate under most random joint values --
    // which is the case that otherwise leaves pr2 with no reachable free
    // state, since every draw contracted toward home inherits the overlap.
    state.set_to_default_values();
    let at_home = contacting(&state);

    let mut rng = ChaCha8Rng::seed_from_u64(SEED);
    let mut seen: BTreeMap<(String, String), usize> = BTreeMap::new();
    for _ in 0..ALWAYS_SAMPLE {
        state.set_to_random_positions_with(&mut rng);
        for pair in contacting(&state) {
            *seen.entry(pair).or_default() += 1;
        }
    }

    let mut acm = base.clone();
    for (a, b) in at_home {
        acm.set_entry(&a, &b, true);
    }
    let threshold = (ALWAYS_FRACTION * ALWAYS_SAMPLE as f64).ceil() as usize;
    for ((a, b), count) in seen {
        if count >= threshold {
            acm.set_entry(&a, &b, true);
        }
    }
    acm
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn main() {
    for name in ROBOTS {
        let (model, srdf_acm) = build(name);

        // One box well clear of every fixture's reach, so the world leg
        // measures the same enumeration this file is about rather than a
        // scene-specific contact pattern.
        let mut world = World::new();
        world.add_shape(
            "probe_box",
            Arc::new(Shape::Cuboid(
                Cuboid::new(0.4, 0.4, 0.4).expect("positive cuboid"),
            )),
            Isometry3::translation(4.0, 0.0, 0.0),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let acm = augment_acm_with_always_colliding(&model, &srdf_acm, &env);

        let pairs = permitted_pairs(&model, &acm);
        let links = model
            .link_models()
            .iter()
            .filter(|l| !l.shapes().is_empty())
            .count();
        let request = CollisionRequest::default();

        // Sample once, classify once. Timing then runs per class, so a
        // class's median is not the other class's outlier.
        //
        // Uniform draws alone do not reach the free class on a robot with
        // many links, so each draw is also contracted toward the default
        // state by a shrinking `t` until the free class fills. Contraction is
        // per-variable and the default is in-bounds, so every contracted draw
        // is in-bounds too. `t` is reported: a row whose free states appear
        // only at small `t` is a robot whose free configurations are a narrow
        // set, not a robot whose checks are cheap.
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let home: Vec<f64> = state.positions().to_vec();

        let mut free: Vec<Vec<f64>> = Vec::new();
        let mut hit: Vec<Vec<f64>> = Vec::new();
        let mut free_t = 1.0;
        for t in [1.0, 0.5, 0.25, 0.1, 0.04] {
            let mut rng = ChaCha8Rng::seed_from_u64(SEED);
            let (mut f, mut h) = (Vec::new(), Vec::new());
            for _ in 0..STATES {
                state.set_to_random_positions_with(&mut rng);
                let positions: Vec<f64> = state
                    .positions()
                    .iter()
                    .zip(&home)
                    .map(|(q, h)| h + t * (q - h))
                    .collect();
                state.set_variable_positions(&positions);
                let posed = state.update();
                if env
                    .check_self_collision(&request, &posed, &[], Some(&acm))
                    .collision
                {
                    h.push(positions);
                } else {
                    f.push(positions);
                }
            }
            // The uniform draw owns the `hit` class: contracting toward home
            // is a device for reaching free states, and its colliding
            // leftovers are not the collision distribution a planner meets.
            if t == 1.0 {
                hit = h;
            }
            (free, free_t) = (f, t);
            if free.len() >= 30 {
                break;
            }
        }

        // The first check on a fixture pays lazy mesh decode and `ObbTree`
        // construction; without this the first measured call carries all of
        // it.
        let mut rng = ChaCha8Rng::seed_from_u64(SEED);
        for _ in 0..WARMUP {
            state.set_to_random_positions_with(&mut rng);
            let posed = state.update();
            let _ = env.check_self_collision(&request, &posed, &[], Some(&acm));
            let _ = env.check_robot_collision(&request, &posed, &[], Some(&acm));
        }

        let time = |states: &[Vec<f64>], state: &mut RobotState<'_>, req: &CollisionRequest| {
            let mut us = Vec::new();
            for positions in states {
                state.set_variable_positions(positions);
                let posed = state.update();
                let t = Instant::now();
                let _ = env.check_self_collision(req, &posed, &[], Some(&acm));
                us.push(t.elapsed().as_secs_f64() * 1e6);
            }
            us
        };

        let exhaustive_request = CollisionRequest {
            cost: true,
            ..CollisionRequest::default()
        };
        let all: Vec<Vec<f64>> = free.iter().chain(&hit).cloned().collect();

        let free_us = time(&free, &mut state, &request);
        let hit_us = time(&hit, &mut state, &request);
        let all_us = time(&all, &mut state, &exhaustive_request);

        let mut world_us = Vec::new();
        let mut rng = ChaCha8Rng::seed_from_u64(SEED);
        for _ in 0..STATES {
            state.set_to_random_positions_with(&mut rng);
            let posed = state.update();
            let t = Instant::now();
            let _ = env.check_robot_collision(&request, &posed, &[], Some(&acm));
            world_us.push(t.elapsed().as_secs_f64() * 1e6);
        }

        println!(
            "{name:<16} links {links:>3}  permitted pairs {pairs:>5}  world med {:>7.2}us",
            median(world_us)
        );
        for (label, us) in [
            (format!("free t={free_t:<4}"), free_us),
            ("hit       ".to_string(), hit_us),
            ("exhaustive".to_string(), all_us),
        ] {
            if us.is_empty() {
                println!("  {label}  (no states in this class)");
                continue;
            }
            println!("  {label}  n={:<4}  med {:>10.2}us", us.len(), median(us));
        }
    }

    println!();
    println!(
        "{STATES} states per robot, seed {SEED:#x}, {} build",
        if cfg!(debug_assertions) {
            "debug"
        } else {
            "release"
        }
    );
}
