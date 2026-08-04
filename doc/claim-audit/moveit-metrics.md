# Claim audit — moveit-metrics

Prose-citation audit against upstream MoveIt2 pinned at `e017c91e`. Scope:
every citation in `crates/moveit-metrics/src/*.rs`. Same round/method as
`moveit-scene.md` — see that file for the full methodology note.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-metrics/src/lib.rs` (citation `kinematics_metrics.cpp:56-103`) | (not itemized individually by subagent — reported as part of the "fine" bucket for the response.rs/adapters/metrics/attached_body.rs batch) | CONFIRMED | subagent-reported only, not independently re-opened by me | |

## Summary

- 1 site total, this crate's only prose citation
- CONFIRMED (aggregate, not individually itemized): 1
- No EXPIRED findings in this crate

## §172 narrowing sweep (separate exercise, same round)

Not a citation audit, but recorded here since it also swept this crate
exhaustively and is worth keeping off the compaction-risk conversation
context per §175's same reasoning.

- Upstream-first direction: Round 17 made this convention a command,
  `tools/ci/count-narrowing-sweep.sh` (see `moveit-scene.md`'s sibling
  section for the full writeup and the discrepancy it found there).
  Run against `moveit_core/kinematics_metrics/src/kinematics_metrics.cpp`
  (the only upstream file this crate ports) it reports **4** hits,
  reproducing this section's existing figure exactly — all
  `for (unsigned int i = 0; i < singular_values.rows(); ++i)` /
  `for (int i = 0; i < singular_values.rows(); ++i)` style loop counters —
  `.rows()` on an Eigen matrix returns an integer row count (`Eigen::Index`),
  not a float. **0 real narrowing sites** (all 4 hits are `distinct`: true
  integer loop counters).
- Port-side direction: `rg '\bas\s+(i8|i16|i32|i64|i128|isize|u8|u16|u32|u64|u128|usize)\b'`
  across `crates/moveit-metrics` (src + tests) — **0 hits**.
- Both directions swept, both zero, no fix needed.

## Round 16 item 3, closed out: every output-diverging branch in `lib.rs`

Round 16's brief asked for every `if`/`match`/`?`/early-return in
`crates/moveit-metrics/src/lib.rs` whose output diverges, each marked
`pinned by <test name>` or `unpinned`. Round 16 pinned the continuous/
floating/planar skips and the `columns < 6` gate; this was left partial.
Full enumeration this round, by function:

**`group` (helper used by all 3 public metrics)**
- unknown group name (`joint_model_group(group)?` errors) — `pinned by unknown_group_is_unknown_name`
- `!group_model.is_chain()` — `pinned by non_chain_group_is_rejected`, `manipulability_rejects_a_non_chain_group`, `manipulability_ellipsoid_rejects_the_same_bad_groups`

**`joint_limits_penalty`**
- `penalty_multiplier.abs() <= f64::MIN_POSITIVE` early return — `pinned by default_penalty_multiplier_is_unpenalized` (0.0), `penalty_multiplier_at_the_min_positive_boundary_still_short_circuits` (exact boundary, `<=` isolated from `<`)
- continuous-revolute skip — `pinned by continuous_revolute_joint_does_not_contribute_to_joint_limits_penalty`
  - the nested `if let JointKind::Revolute(revolute) = joint.kind()` inside that skip is `unpinnable (not a real branch)`: `JointModel::joint_type()` (`moveit-model/src/joint/model.rs:288-295`) is computed *by* matching on `kind()`, so `joint_type() == Revolute` and `kind()` matching `JointKind::Revolute(_)` cannot disagree by construction — there is no state reachable through the public API that takes one arm without the other.
- planar 6-term sentinel — `pinned by each_planar_sentinel_condition_independently_skips_the_joint` (each of the 6 terms individually), plus `planar_joint_with_default_bounds_is_skipped_in_joint_limits_penalty`, `planar_xy_infinite_bounds_still_skip_despite_finite_theta`, `planar_theta_bound_at_pi_literal_still_skips_despite_finite_translation`, and the negative case `planar_joint_with_finite_bounds_is_not_skipped_in_joint_limits_penalty`
- floating-joint unconditional skip — `pinned by floating_joint_is_skipped_in_joint_limits_penalty`
- `state.joint_position(joint.name())?` early return — `unpinnable (not a real branch)`: `joint` is always drawn from the same `model` that built `state` (both come from the caller's own `RobotModel`/`RobotState` pair), so this cannot error through any state reachable via the public API without first breaking that same-model invariant, which nothing in this crate or its callers can do.
- `range <= f64::MIN_POSITIVE` skip, two distinct sub-cases:
  - fixed joint (`range` is exactly `0.0`) — **pinned this round**, not previously named by any test despite already being exercised by several. `panda_arm` (used by `manipulability_index_scales_by_joint_limits_penalty`, `range_at_the_min_positive_boundary_still_skips`, and the oracle parity test `panda_kinematics_metrics_matches_the_oracle`) and pr2's `right_arm` (used by `continuous_revolute_joint_does_not_contribute_to_joint_limits_penalty`) each contain a `Fixed` joint (`panda_joint8`; `r_upper_arm_joint`/`r_forearm_joint` — confirmed via a temporary probe over `JointModelGroup::joint_indices()`, not committed). Measured, not assumed: narrowing the skip to exclude `range == 0.0` specifically (`if range <= f64::MIN_POSITIVE && range != 0.0`, leaving the `f64::MIN_POSITIVE` boundary itself untouched) and rerunning `-p moveit-metrics --no-fail-fast` fails all four of those tests with `NaN`. `lib.rs`'s own module doc now cites this measurement and these four test names directly.
  - contrived non-zero boundary (`range == f64::MIN_POSITIVE` exactly, from a hand-mutated bound) — `pinned by range_at_the_min_positive_boundary_still_skips`

**`manipulability_index`**
- `self.group(...)?`, `state.jacobian(...)?`, `self.joint_limits_penalty(...)?` error propagation — same sites as `group`'s rows above
- `translation` (bool) × `columns < 6` (bool), 4 combinations — `pinned by panda_kinematics_metrics_matches_the_oracle` (`panda_arm`, 7 columns, both `translation` values, always the `columns >= 6` path) and `panda_arm_5dof_kinematics_metrics_matches_the_oracle` (`panda_arm_5dof`, 5 columns, both `translation` values, always the `columns < 6` path); the `< 6` threshold literal itself has a recorded bidirectional falsification in `kinematics_metrics_parity.rs`'s own module doc (narrowing to `< 5`/`< 4` breaks the 5dof case; widening is unobservable through any oracle fixture, with the reason why recorded there)

**`manipulability_ellipsoid`**
- `self.group(...)?` — `pinned by manipulability_ellipsoid_rejects_the_same_bad_groups`, though that same test's own doc comment records this call site as redundant with `state.jacobian(...)?`'s own check for this method specifically (asserted regardless)
- `state.jacobian(...)?` — same test, the actually load-bearing error path here

**`manipulability`**
- `self.group(...)?`, `state.jacobian(...)?`, `self.joint_limits_penalty(...)?` — `pinned by manipulability_rejects_a_non_chain_group`, `unknown_group_is_unknown_name`
- `translation` (bool) target selection (`jacobian.rows(0, 3)` vs. the full Jacobian) — `pinned by panda_kinematics_metrics_matches_the_oracle` (asserts the fixture's `manipulability_translation` field at `translation = true` separately from the full value at `false`)

Result: every genuine branch is pinned; the two `unpinnable` entries are
both structural invariants enforced by this crate's own types, not gaps.
One branch (`range == 0.0`, the fixed-joint fallthrough) was pinned in
substance by four already-existing tests but had never been named as
such — closed this round by citing them directly in `lib.rs`'s doc
comment (see commit).
