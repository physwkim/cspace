=== gate at 4a8eaab ===
--- cargo fmt --all -- --check ---
rc=0
--- cargo clippy --workspace --all-targets -- -D warnings ---
    Checking moveit-collision v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-collision)
    Checking moveit-constraints v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-constraints)
    Checking moveit-distance-field v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-distance-field)
    Checking moveit-scene v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-scene)
    Checking moveit-diff v0.1.0 (/home/stevek/work/moveit-rs/tools/moveit-diff)
    Checking moveit-planners-chomp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-chomp)
    Checking moveit-planners-sbp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-sbp)
    Checking moveit-planners-pilz v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-pilz)
    Checking moveit-planning v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planning)
    Checking moveit-planners-stomp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-stomp)
    Finished `dev` profile [optimized + debuginfo] target(s) in 1.53s
rc=0
--- cargo nextest run --workspace --no-fail-fast ---
[32;1m        PASS[0m [   0.450s] (1624/1646) [35;1mmoveit-planners-chomp[0m [36moptimizer::tests[0m[36m::[0m[34;1moptimize_runs_exactly_max_iterations_when_filter_mode_and_mesh_to_mesh_never_break_out[0m
[32;1m        PASS[0m [   0.448s] (1625/1646) [35;1mmoveit-planners-chomp[0m [36mplanner::tests[0m[36m::[0m[34;1msolve_succeeds_with_no_obstacles_and_produces_a_101_point_trajectory[0m
[32;1m        PASS[0m [   0.459s] (1626/1646) [35;1mmoveit-planners-chomp[0m [36mplanner::tests[0m[36m::[0m[34;1msolve_returns_invalid_motion_plan_when_the_path_cannot_escape_collision[0m
[32;1m        PASS[0m [   0.304s] (1627/1646) [35;1mmoveit-trajectory[0m [36mtrajectory::tests[0m[36m::[0m[34;1mupstream_test2[0m
[32;1m        PASS[0m [   0.298s] (1628/1646) [35;1mmoveit-trajectory::totg_parity[0m [34;1mtotg_matches_the_oracle[0m
[32;1m        PASS[0m [   0.453s] (1629/1646) [35;1mmoveit-planners-sbp[0m [36mnn::tests[0m[36m::[0m[34;1mnearest_agrees_with_brute_force[0m
[32;1m        PASS[0m [   0.677s] (1630/1646) [35;1mmoveit-collision::collision_parity[0m [34;1mpr2_torso_lift_bellow_pair_crossover_confirms_min_of_two_candidates[0m
[32;1m        PASS[0m [   0.662s] (1631/1646) [35;1mmoveit-distance-field::collision_env_distance_field_parity[0m [34;1mget_collision_gradients_ignores_the_contacts_request_field[0m
[32;1m        PASS[0m [   0.728s] (1632/1646) [35;1mmoveit-collision::collision_parity[0m [34;1mpr2_case_7552_depth_disagreement_ranks_a_different_pair[0m
[32;1m        PASS[0m [   0.683s] (1633/1646) [35;1mmoveit-planners-chomp[0m [36moptimizer::tests[0m[36m::[0m[34;1mperform_forward_kinematics_flags_the_point_an_obstacle_sits_on[0m
[32;1m        PASS[0m [   0.726s] (1634/1646) [35;1mmoveit-planners-stomp[0m [36mnoise_generators::tests[0m[36m::[0m[34;1mnum_timesteps_never_produces_a_covariance_multivariate_gaussian_new_rejects[0m
[32;1m        PASS[0m [   0.917s] (1635/1646) [35;1mmoveit-distance-field::collision_env_distance_field_parity[0m [34;1mgenerate_distance_field_cache_entry_matches_the_oracle[0m
[32;1m        PASS[0m [   0.928s] (1636/1646) [35;1mmoveit-distance-field::collision_env_distance_field_parity[0m [34;1mcheck_self_collision_reuses_the_distance_field_cache_entry_fixture[0m
[32;1m        PASS[0m [   1.042s] (1637/1646) [35;1mmoveit-collision::collision_parity[0m [34;1mgripper_pair_contact_is_prediction_invariant[0m
[32;1m        PASS[0m [   0.925s] (1638/1646) [35;1mmoveit-planners-chomp[0m [36moptimizer::tests[0m[36m::[0m[34;1moptimize_collision_threshold_break_is_a_strict_less_than[0m
[32;1m        PASS[0m [   1.001s] (1639/1646) [35;1mmoveit-distance-field::collision_env_hybrid_parity[0m [34;1mcheck_collision_distance_field_environment_branch_paired_control[0m
[32;1m        PASS[0m [   0.910s] (1640/1646) [35;1mmoveit-planners-sbp[0m [36mregistry::tests[0m[36m::[0m[34;1mscenario3_orientation_only_corridor_sample_level_satisfaction_rate[0m
[32;1m        PASS[0m [   1.130s] (1641/1646) [35;1mmoveit-distance-field::collision_env_distance_field_parity[0m [34;1mcheck_collision_matches_the_oracle_with_contacts_and_attached_bodies[0m
[32;1m        PASS[0m [   1.209s] (1642/1646) [35;1mmoveit-collision::collision_parity[0m [34;1mfanuc_collision_matches_the_oracle[0m
[32;1m        PASS[0m [   1.379s] (1643/1646) [35;1mmoveit-distance-field::collision_env_distance_field_parity[0m [34;1mcheck_self_collision_matches_the_oracle_with_contacts_and_attached_bodies[0m
[32;1m        PASS[0m [   1.394s] (1644/1646) [35;1mmoveit-distance-field::collision_env_hybrid_parity[0m [34;1mcheck_robot_collision_distance_field_matches_the_oracle_robot_only_mode[0m
[32;1m        PASS[0m [   1.386s] (1645/1646) [35;1mmoveit-trajectory[0m [36mtrajectory::tests[0m[36m::[0m[34;1mupstream_test3[0m
[32;1m        PASS[0m [   1.844s] (1646/1646) [35;1mmoveit-distance-field::collision_env_distance_field_parity[0m [34;1mgroup_state_representation_gradients_matches_the_oracle[0m
────────────
[32;1m     Summary[0m [   1.931s] [1m1646[0m tests run: [1m1646[0m [32;1mpassed[0m, [1m2[0m [33;1mskipped[0m
rc=0
--- cargo test --doc --workspace ---

running 0 tests

test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s

   Doc-tests moveit_stomp_core

running 1 test
test crates/moveit-stomp-core/src/utils.rs - utils::generate_smoothing_matrix (line 351) ... ok

test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s

all doctests ran in 0.28s; merged doctests compilation took 0.27s
   Doc-tests moveit_test_support

running 0 tests

test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s

   Doc-tests moveit_trajectory

running 0 tests

test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s

rc=0
--- cargo doc --workspace --no-deps ---
 Documenting moveit-stomp-core v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-stomp-core)
 Documenting moveit-geometry v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-geometry)
 Documenting moveit-model v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-model)
 Documenting moveit-test-support v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-test-support)
 Documenting moveit-state v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-state)
 Documenting moveit-smoothing v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-smoothing)
    Checking moveit-collision v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-collision)
 Documenting moveit-collision v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-collision)
 Documenting moveit-kinematics v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-kinematics)
 Documenting moveit-metrics v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-metrics)
 Documenting moveit-trajectory v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-trajectory)
    Checking moveit-constraints v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-constraints)
    Checking moveit-distance-field v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-distance-field)
 Documenting moveit-constraints v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-constraints)
 Documenting moveit-distance-field v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-distance-field)
    Checking moveit-scene v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-scene)
 Documenting moveit-scene v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-scene)
 Documenting moveit-diff v0.1.0 (/home/stevek/work/moveit-rs/tools/moveit-diff)
 Documenting moveit-planners-chomp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-chomp)
 Documenting moveit-planners-pilz v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-pilz)
 Documenting moveit-planners-sbp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-sbp)
 Documenting moveit-planning v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planning)
 Documenting moveit-planners-stomp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-stomp)
    Finished `dev` profile [optimized + debuginfo] target(s) in 1m 49s
   Generated /home/stevek/work/moveit-rs/target/doc/moveit_collision/index.html and 22 other files
rc=0
--- ./tools/ci/verify-private-doc-links.sh ---
 Documenting moveit-srdf v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-srdf)
 Documenting moveit-stomp-core v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-stomp-core)
 Documenting moveit-octomap v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-octomap)
 Documenting moveit-sampling v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-sampling)
 Documenting moveit-geometry v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-geometry)
 Documenting moveit-model v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-model)
 Documenting moveit-test-support v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-test-support)
 Documenting moveit-smoothing v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-smoothing)
 Documenting moveit-state v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-state)
 Documenting moveit-collision v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-collision)
 Documenting moveit-trajectory v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-trajectory)
 Documenting moveit-metrics v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-metrics)
 Documenting moveit-kinematics v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-kinematics)
 Documenting moveit-constraints v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-constraints)
 Documenting moveit-distance-field v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-distance-field)
 Documenting moveit-diff v0.1.0 (/home/stevek/work/moveit-rs/tools/moveit-diff)
 Documenting moveit-planners-chomp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-chomp)
 Documenting moveit-scene v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-scene)
 Documenting moveit-planners-sbp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-sbp)
 Documenting moveit-planning v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planning)
 Documenting moveit-planners-stomp v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-stomp)
 Documenting moveit-planners-pilz v0.1.0 (/home/stevek/work/moveit-rs/crates/moveit-planners-pilz)
    Finished `dev` profile [optimized + debuginfo] target(s) in 1m 53s
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
=== ros/moveit-ros docker gate ===
test scene::planning_scene::tests::robot_model_name_matches_empty_and_exact ... ok
test scene::tests::unresolvable_non_empty_frame_id_is_still_rejected ... ok
test state::tests::is_diff_is_rejected_not_silently_dropped ... ok
test state::tests::attached_collision_objects_is_rejected_not_silently_dropped ... ok
test scene::tests::resolvable_frame_id_resolves_the_same_as_frame_transform ... ok
test state::tests::joint_state_positions_convert_by_name ... ok
test state::tests::mismatched_position_length_is_rejected ... ok
test trajectory::tests::seconds_to_duration_accepts_zero ... ok
test trajectory::tests::seconds_to_duration_carries_a_rounding_tie_into_seconds ... ok
test trajectory::tests::seconds_to_duration_rejects_infinity ... ok
test trajectory::tests::seconds_to_duration_rejects_just_above_i32_max_seconds ... ok
test scene::planning_scene::tests::octomap_origin_is_composed_with_the_header_frame_transform ... ok
test trajectory::tests::seconds_to_duration_rejects_nan ... ok
test trajectory::tests::positions_length_mismatch_is_rejected ... ok
test trajectory::tests::seconds_to_duration_rejects_negative ... ok
test trajectory::tests::negative_cumulative_duration_from_an_unvalidated_trajectory_is_rejected ... ok
test trajectory::tests::nonzero_start_time_is_rejected ... ok
test scene::shapes::tests::mesh_triangle_with_wrong_vertex_count_is_rejected ... ok
test trajectory::tests::velocities_length_mismatch_is_rejected ... ok
test state::tests::multi_dof_joint_state_is_rejected_not_silently_dropped ... ok
test scene::shapes::tests::plane_round_trips_through_msg ... ok
test state::tests::round_trip_through_msg ... ok
test state::tests::mismatched_velocity_length_is_rejected ... ok
test scene::tests::empty_frame_id_resolves_to_identity ... ok
test state::tests::unknown_joint_name_is_rejected ... ok
test trajectory::tests::seconds_to_duration_accepts_i32_max_seconds ... ok
test trajectory::tests::round_trip_through_msg ... ok
test trajectory::tests::add_suffix_way_point_rejects_a_nonzero_first_dt ... ok
test trajectory::tests::converts_and_computes_deltas_from_cumulative_time ... ok
test trajectory::tests::decreasing_time_from_start_is_rejected ... ok
test conversion_coverage::every_bidirectional_pair_has_a_round_trip_test ... ok

test result: ok. 154 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.02s

   Doc-tests moveit_ros

running 0 tests

test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s

=== root docker verify-all ===
  Downloaded syn v3.0.3
  Downloaded syn v2.0.119
  Downloaded spade v2.15.1
    Checking moveit-collision v0.1.0 (/repo/crates/moveit-collision)
    Checking moveit-constraints v0.1.0 (/repo/crates/moveit-constraints)
    Checking moveit-scene v0.1.0 (/repo/crates/moveit-scene)
    Checking moveit-planning v0.1.0 (/repo/crates/moveit-planning)
 Documenting moveit-ros v0.1.0 (/repo/ros/moveit-ros)
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 5.32s
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
    Finished `release` profile [optimized] target(s) in 2.14s
────────────
[32;1m Nextest run[0m ID [1m30ce8c01-4ca5-4c6f-8c6e-0fc1ea6e1b1b[0m with nextest profile: [1mdefault[0m
[32;1m    Starting[0m [1m2[0m tests across [1m2[0m binaries ([1m22[0m tests [33;1mskipped[0m)
[32;1m        PASS[0m [   0.103s] (1/2) [35;1mmoveit-diff::bin/moveit-diff[0m [36mvisibility_cone_ambiguity_diagnostic[0m[36m::[0m[34;1ma_real_mismatching_case_touches_exactly_one_link[0m
[32;1m        PASS[0m [   0.266s] (2/2) [35;1mmoveit-diff::bin/moveit-diff[0m [36mvisibility_cone_ambiguity_diagnostic[0m[36m::[0m[34;1mnear_placement_never_touches_more_than_one_link_at_once[0m
────────────
[32;1m     Summary[0m [   0.267s] [1m2[0m tests run: [1m2[0m [32;1mpassed[0m, [1m22[0m [33;1mskipped[0m
OK 2 vendored-fixture test(s) passed

OK all 10 verify script(s) passed
=== tree after: 4a8eaab 0 dirty ===
WORKSPACE_GATE_FAIL=0
