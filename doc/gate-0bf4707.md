=== gate at 0bf4707 ===
--- cargo fmt --all -- --check ---
rc=0
--- cargo clippy --workspace --all-targets -- -D warnings ---
    Checking moveit-stomp-core v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-stomp-core)
    Checking moveit-geometry v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-geometry)
    Checking moveit-model v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-model)
    Checking moveit-state v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-state)
    Checking moveit-test-support v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-test-support)
    Checking moveit-smoothing v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-smoothing)
    Checking moveit-collision v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-collision)
    Checking moveit-kinematics v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-kinematics)
    Checking moveit-trajectory v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-trajectory)
    Checking moveit-metrics v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-metrics)
    Checking moveit-constraints v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-constraints)
    Checking moveit-distance-field v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-distance-field)
    Checking moveit-scene v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-scene)
    Checking moveit-diff v0.1.0 (/home/stevek/work/moveit-rs/tools/moveit-diff)
    Checking moveit-planners-chomp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-chomp)
    Checking moveit-planners-sbp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-sbp)
    Checking moveit-planners-pilz v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-pilz)
    Checking moveit-planners-stomp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-stomp)
    Checking moveit-planning v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planning)
    Finished `dev` profile [optimized + debuginfo] target(s) in 2.55s
rc=0
--- cargo nextest run --workspace --no-fail-fast ---
[32;1m        PASS[0m [   0.756s] (1629/1646) [35;1mmoveit-collision::collision_parity[0m [34;1mpr2_torso_lift_bellow_pair_crossover_confirms_min_of_two_candidates[0m
[32;1m        PASS[0m [   0.358s] (1630/1646) [35;1mmoveit-trajectory[0m [36mtrajectory::tests[0m[36m::[0m[34;1mupstream_test2[0m
[32;1m        PASS[0m [   0.815s] (1631/1646) [35;1mmoveit-collision::collision_parity[0m [34;1mpr2_case_7552_depth_disagreement_ranks_a_different_pair[0m
[32;1m        PASS[0m [   0.792s] (1632/1646) [35;1mmoveit-distance-field::collision_env_distance_field_parity[0m [34;1mget_collision_gradients_ignores_the_contacts_request_field[0m
[32;1m        PASS[0m [   0.827s] (1633/1646) [35;1mmoveit-planners-chomp[0m [36moptimizer::tests[0m[36m::[0m[34;1mperform_forward_kinematics_flags_the_point_an_obstacle_sits_on[0m
[32;1m        PASS[0m [   0.768s] (1634/1646) [35;1mmoveit-planners-stomp[0m [36mnoise_generators::tests[0m[36m::[0m[34;1mnum_timesteps_never_produces_a_covariance_multivariate_gaussian_new_rejects[0m
[32;1m        PASS[0m [   1.049s] (1635/1646) [35;1mmoveit-collision::collision_parity[0m [34;1mgripper_pair_contact_is_prediction_invariant[0m
[32;1m        PASS[0m [   1.021s] (1636/1646) [35;1mmoveit-distance-field::collision_env_distance_field_parity[0m [34;1mgenerate_distance_field_cache_entry_matches_the_oracle[0m
[32;1m        PASS[0m [   1.040s] (1637/1646) [35;1mmoveit-distance-field::collision_env_distance_field_parity[0m [34;1mcheck_self_collision_reuses_the_distance_field_cache_entry_fixture[0m
[32;1m        PASS[0m [   1.014s] (1638/1646) [35;1mmoveit-planners-chomp[0m [36moptimizer::tests[0m[36m::[0m[34;1moptimize_collision_threshold_break_is_a_strict_less_than[0m
[32;1m        PASS[0m [   1.007s] (1639/1646) [35;1mmoveit-planners-sbp[0m [36mregistry::tests[0m[36m::[0m[34;1mscenario3_orientation_only_corridor_sample_level_satisfaction_rate[0m
[32;1m        PASS[0m [   1.214s] (1640/1646) [35;1mmoveit-distance-field::collision_env_hybrid_parity[0m [34;1mcheck_collision_distance_field_environment_branch_paired_control[0m
[32;1m        PASS[0m [   1.292s] (1641/1646) [35;1mmoveit-collision::collision_parity[0m [34;1mfanuc_collision_matches_the_oracle[0m
[32;1m        PASS[0m [   1.234s] (1642/1646) [35;1mmoveit-distance-field::collision_env_distance_field_parity[0m [34;1mcheck_collision_matches_the_oracle_with_contacts_and_attached_bodies[0m
[32;1m        PASS[0m [   1.392s] (1643/1646) [35;1mmoveit-distance-field::collision_env_distance_field_parity[0m [34;1mcheck_self_collision_matches_the_oracle_with_contacts_and_attached_bodies[0m
[32;1m        PASS[0m [   1.532s] (1644/1646) [35;1mmoveit-distance-field::collision_env_hybrid_parity[0m [34;1mcheck_robot_collision_distance_field_matches_the_oracle_robot_only_mode[0m
[32;1m        PASS[0m [   1.442s] (1645/1646) [35;1mmoveit-trajectory[0m [36mtrajectory::tests[0m[36m::[0m[34;1mupstream_test3[0m
[32;1m        PASS[0m [   1.865s] (1646/1646) [35;1mmoveit-distance-field::collision_env_distance_field_parity[0m [34;1mgroup_state_representation_gradients_matches_the_oracle[0m
────────────
[32;1m     Summary[0m [   1.954s] [1m1646[0m tests run: [1m1646[0m [32;1mpassed[0m, [1m2[0m [33;1mskipped[0m
rc=0
--- cargo test --doc --workspace ---
   Doc-tests moveit_stomp_core

running 1 test
test crates/moveit-stomp-core/src/utils.rs - utils::generate_smoothing_matrix (line 351) ... ok

test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s

all doctests ran in 0.27s; merged doctests compilation took 0.26s
   Doc-tests moveit_test_support

running 0 tests

test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s

   Doc-tests moveit_trajectory

running 0 tests

test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s

rc=0
--- cargo doc --workspace --no-deps ---
    Checking moveit-kinematics v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-kinematics)
    Checking moveit-trajectory v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-trajectory)
 Documenting moveit-collision v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-collision)
 Documenting moveit-kinematics v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-kinematics)
 Documenting moveit-trajectory v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-trajectory)
 Documenting moveit-metrics v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-metrics)
    Checking moveit-constraints v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-constraints)
    Checking moveit-distance-field v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-distance-field)
 Documenting moveit-constraints v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-constraints)
 Documenting moveit-distance-field v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-distance-field)
    Checking moveit-scene v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-scene)
 Documenting moveit-planners-chomp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-chomp)
 Documenting moveit-scene v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-scene)
 Documenting moveit-diff v0.1.0 (/home/stevek/work/moveit-rs/tools/moveit-diff)
 Documenting moveit-planners-pilz v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-pilz)
 Documenting moveit-planners-sbp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-sbp)
 Documenting moveit-planning v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planning)
 Documenting moveit-planners-stomp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-stomp)
    Finished `dev` profile [optimized + debuginfo] target(s) in 1m 52s
   Generated /home/stevek/work/moveit-rs/target/doc/moveit_collision/index.html and 22 other files
rc=0
--- ./tools/ci/verify-private-doc-links.sh ---
 Documenting moveit-model v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-model)
 Documenting moveit-test-support v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-test-support)
 Documenting moveit-smoothing v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-smoothing)
 Documenting moveit-state v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-state)
 Documenting moveit-trajectory v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-trajectory)
 Documenting moveit-collision v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-collision)
 Documenting moveit-kinematics v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-kinematics)
 Documenting moveit-metrics v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-metrics)
 Documenting moveit-distance-field v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-distance-field)
 Documenting moveit-constraints v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-constraints)
 Documenting moveit-scene v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-scene)
 Documenting moveit-planners-chomp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-chomp)
 Documenting moveit-diff v0.1.0 (/home/stevek/work/moveit-rs/tools/moveit-diff)
 Documenting moveit-planning v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planning)
 Documenting moveit-planners-sbp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-sbp)
 Documenting moveit-planners-stomp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-stomp)
 Documenting moveit-planners-pilz v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-pilz)
    Finished `dev` profile [optimized + debuginfo] target(s) in 1m 44s
   Generated /home/stevek/work/moveit-rs/target/doc/moveit_collision/index.html and 22 other files
OK private-item doc links resolve
rc=0
--- ./tools/ci/check-audit-scripts-not-copied.sh ---
OK: no audit command copied into a crate (canonical copies in tools/ci/)
rc=0
--- ./tools/ci/check-dep-direction.sh ---
OK: no workspace member depends on a ROS client library
rc=0
--- ./tools/ci/check-fixture-format.sh ---
OK: 162 oracle fixtures are in canonical form
rc=0
--- ./tools/ci/check-license-matches-upstream.sh ---
OK: every crate's license matches the SPDX identifier in its own sources
rc=0
--- ./tools/ci/check-lints-not-silently-dropped.sh ---
OK: no crate silently drops a workspace lint
rc=0
--- ./tools/ci/check-no-dead-status-capture.sh ---
OK: no bare $? capture immediately follows an unguarded command-substitution close
rc=0
--- ./tools/ci/check-no-lint-suppression.sh ---
OK: no lint suppression in crates/ or tools/
rc=0
--- ./tools/ci/check-pilz-tolerance-overrides.sh ---
OK: all 10 per-case tolerance overrides in crates/moveit-planners-pilz/tests/pilz_blend_parity.rs have a #[should_panic] necessity test
rc=0
--- ./tools/ci/check-serde-float-roundtrip.sh ---
OK: serde_json resolves with "float_roundtrip"
rc=0
--- ./tools/ci/check-workspace-dep-inheritance.sh ---
OK: every inter-member dependency goes through [workspace.dependencies]
rc=0
=== root docker verify-all ===
    Checking moveit-scene v0.1.0 (/repo/crates/moveit-scene)
    Checking moveit-planning v0.1.0 (/repo/crates/moveit-planning)
 Documenting moveit-ros v0.1.0 (/repo/ros/moveit-ros)
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 5.76s
   Generated /repo/ros/moveit-ros/target/doc/moveit_ros/index.html
all gates passed
=== tools/ci/verify-upstream-license-provenance.sh
checked 348 upstream file(s) cited by 251 tracked source file(s)
OK: no permissive-SPDX file cites a copyleft upstream file, every
    asserted upstream copyright is reproduced by a file that file cites,
    and every ported-from file's notice is retained
=== tools/ci/verify-vendored-fixture-tests.sh
running 2 vendored-fixture test(s) against third_party/moveit_resources
   Compiling moveit-diff v0.1.0 (/home/stevek/work/moveit-rs/tools/moveit-diff)
    Finished `release` profile [optimized] target(s) in 2.13s
────────────
[32;1m Nextest run[0m ID [1m6a84aeae-317a-4bd0-9940-8bb366202076[0m with nextest profile: [1mdefault[0m
[32;1m    Starting[0m [1m2[0m tests across [1m2[0m binaries ([1m22[0m tests [33;1mskipped[0m)
[32;1m        PASS[0m [   0.107s] (1/2) [35;1mmoveit-diff::bin/moveit-diff[0m [36mvisibility_cone_ambiguity_diagnostic[0m[36m::[0m[34;1ma_real_mismatching_case_touches_exactly_one_link[0m
[32;1m        PASS[0m [   0.259s] (2/2) [35;1mmoveit-diff::bin/moveit-diff[0m [36mvisibility_cone_ambiguity_diagnostic[0m[36m::[0m[34;1mnear_placement_never_touches_more_than_one_link_at_once[0m
────────────
[32;1m     Summary[0m [   0.260s] [1m2[0m tests run: [1m2[0m [32;1mpassed[0m, [1m22[0m [33;1mskipped[0m
OK 2 vendored-fixture test(s) passed

OK all 10 verify script(s) passed
=== tree after: 0bf4707 3 dirty ===
WORKSPACE_GATE_FAIL=0
