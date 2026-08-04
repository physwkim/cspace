// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream C++ file to port: PORTING-PLAN.md §2 records that the Rust
// ecosystem has no OMPL equivalent, and §6.3 lists that as the top risk
// (D3: native Rust planners first, an OMPL FFI bridge only as fallback).
// This crate is original design work against that gap, not a transcription.
// RRT-Connect follows the published algorithm (Kuffner & LaValle, ICRA
// 2000); the nearest-neighbour index is GNAT-family (Brin, 1995) for the
// reason recorded in `nn`'s doc comment, not a port of OMPL's C++ GNAT.

//! Sampling-based motion planning for moveit-rs.
//!
//! # Scope
//!
//! This is the abstract foundation plus one planner, built against four
//! [`StateSpace`] implementations covering MoveIt's actual joint types:
//! [`RealVectorSpace`] (plain bounded `R^n` — prismatic and bounded revolute
//! joints), [`so2::So2Space`] (a continuous joint's wraparound),
//! [`se3::Se3Space`] (a floating joint's `R^3 x SO(3)`), and
//! [`compound::CompoundSpace`] (a `JointModelGroup`'s heterogeneous product
//! of any of the above, weighted). All four were first tested standalone
//! with no dependency on `moveit-model` or `moveit-state`;
//! [`joint_model_group_space::JointModelGroupSpace`] is the bridge from an
//! actual `RobotModel`/`JointModelGroup` to a [`StateSpace`], and is what
//! brings those two crates in as dependencies.
//!
//! - [`space`] — the [`StateSpace`] trait and [`RealVectorSpace`].
//! - [`so2`] — [`so2::So2Space`], a wraparound revolute joint.
//! - [`se3`] — [`se3::Se3Space`], a floating joint.
//! - [`compound`] — [`compound::CompoundSpace`], a weighted product of
//!   subspaces of any of the above kinds.
//! - [`joint_model_group_space`] —
//!   [`joint_model_group_space::JointModelGroupSpace`], a `RobotModel` joint
//!   model group as a [`StateSpace`].
//! - [`validity`] — [`StateValidityChecker`] and [`MotionValidator`], kept
//!   separate on purpose (see [`validity`]'s doc comment).
//! - [`planning_scene_validity`] —
//!   [`planning_scene_validity::PlanningSceneValidityChecker`], the bridge
//!   from a [`joint_model_group_space::JointModelGroupSpace`] sample to a
//!   real `moveit_scene::PlanningScene` collision/constraint check.
//! - [`nn`] — [`Gnat`], the nearest-neighbour index.
//! - [`rrt_connect`] — bidirectional RRT-Connect.
//! - [`registry`] — [`registry::PlannerManager`]/[`registry::PlanningContext`]
//!   and the [`registry::PLANNER_MANAGERS`] compile-time registry (D4),
//!   plus [`registry::RrtConnectManager`], the one registered planner.
//!
//! # Why properties, not an oracle
//!
//! Every other crate in this workspace is checked against `tools/moveit-oracle`,
//! a C++ binary linking the real moveit2. There is nothing to link here: no
//! C++ RRT-Connect or GNAT exists in this workspace to compare against, and
//! a sampling planner's *specific* output path is not a meaningful thing to
//! match bit-for-bit against a different implementation's RNG draws anyway.
//! Correctness here is established by properties that would fail if the
//! implementation were wrong — path endpoints are exact, every returned
//! segment is independently re-checked against the same
//! [`MotionValidator`] used to build it, nearest-neighbour results are
//! checked against brute force over thousands of queries, and a closed
//! passage is checked to fail rather than hang or return an invalid path.
//! See each module's `tests` for the specific claims and the crate's commit
//! history / report for which parts of this design are least certain.
//!
//! # Completion statement
//!
//! Every number below is a command someone can re-run.
//!
//! **`planning_interface.hpp`** (this crate's only upstream C++ header —
//! see "Round 6 symbol audit" immediately below for why the other six
//! modules have none to audit against):
//! `sed -n '126,203p' crates/moveit-planners-sbp/src/lib.rs | rg -c '^//! - '`
//! is **18** — one bullet per upstream declaration, or per matched sibling
//! overload/accessor pair audited together (e.g. `getGroupName()`/`getName()`
//! on one line). Those 18 bullets account for all **25** public
//! declarations in
//! `/home/stevek/work/moveit2/moveit_core/planning_interface/include/moveit/planning_interface/planning_interface.hpp`:
//! `PlannerConfigurationSettings`/`PlannerConfigurationMap` (2, audited as
//! one struct-level unit — this port's D4 exclusion is of the whole
//! stringly-typed config bag, not field-by-field), `PlanningContext`'s 12
//! public members (ctor, dtor, `getGroupName`, `getName`,
//! `getPlanningScene`, `getMotionPlanRequest`, `setPlanningScene`,
//! `setMotionPlanRequest`, `solve` x2, `terminate`, `clear`), and
//! `PlannerManager`'s 11 (ctor, dtor, `initialize`, `getDescription`,
//! `getPlanningAlgorithms`, `getPlanningContext` x2, `canServiceRequest`,
//! `setPlannerConfigurations`, `getPlannerConfigurations`, `terminate`).
//! **22** of the 25 are `unported` (D1/D2/D4/structural, one reason per
//! bullet); the remaining **3** are ported — `solve`'s two overloads
//! collapsed into one [`registry::PlanningContext::solve`], and
//! `getPlanningContext(scene, req, error_code)` ported as
//! [`registry::PlannerManager::get_planning_context`] (its
//! error-code-ignoring sibling overload stays unported, Rust's `Result`
//! already spelling "ignore the error" at the call site). Re-derive the 25
//! by reading the header at that path directly; nothing here parses it
//! automatically.
//!
//! **Tests.** `cargo nextest run -p moveit-planners-sbp --no-fail-fast`:
//! **94** tests, 94 passed (round 18: was 93 when this section was written,
//! stale by one test the moment `4f870fe` added `plan_space_parity.rs` a few
//! commits later the same round — re-verified rather than left wrong).
//! There is no oracle comparison for this crate at
//! all (see "Why properties, not an oracle" above) — every test is a
//! property or boundary check instead. The one property common to every
//! [`StateSpace`] this crate ships (`space::RealVectorSpace`, `so2::So2Space`,
//! `se3::Se3Space`, `compound::CompoundSpace`,
//! `joint_model_group_space::JointModelGroupSpace`) is exercised through one
//! shared helper: `rg -c 'assert_metric_and_interpolation_axioms\(' <file>`
//! finds exactly one call site in each of `so2.rs`, `space.rs`,
//! `compound.rs` and `se3.rs`, and two in `joint_model_group_space.rs` (one
//! per fixture group it drives the axioms against) — six call sites total
//! against the one `pub(crate) fn assert_metric_and_interpolation_axioms`
//! defined in `test_support.rs`, not six independent reimplementations of
//! the same check.
//!
//! Round 14: `rg -c 'assert_relative_eq!' crates/moveit-planners-sbp/` is
//! **0** — this crate has no site using that macro at all, so there is
//! nothing here to bisect for a default-`max_relative`-masking regression
//! the way other crates in this workspace round needed to.
//!
//! # Round 18: Phase 7's C++ baseline, not yet a port-side verdict
//!
//! Phase 7's three completion conditions (`PORTING-PLAN.md` §5) each compare
//! this port's [`rrt_connect::rrt_connect`] against C++ OMPL RRTConnect on
//! the same 500 problems. Building the problem set and measuring the C++
//! side of that comparison is this round's whole scope — no port-side
//! measurement exists yet (deferred to the next round; see that round's own
//! report for why: a wrong problem set would have invalidated every port
//! number measured against it, so the set is built and its C++ baseline
//! checked on its own first).
//!
//! `examples/plan_benchmark_problem_set.rs` samples `(start, goal)` pairs
//! for one named obstacle configuration and filters out any pair invalid on
//! either end (self-collision or obstacle penetration, via this crate's own
//! [`planning_scene_validity::PlanningSceneValidityChecker`]) — an
//! unfiltered set would measure the sampler, not the planner.
//! `benches/sweep_baseline.sh` drives it against the live oracle's `plan` op
//! (§118) and is the reproducible command behind every number below; see
//! that script's own doc comment for the full obstacle-difficulty sweep
//! table, the reasoning for building the final set from exactly two of the
//! six configs (`floor_wall`, `cage` — the only two where median
//! `ptc_evaluations` rises above RRTConnect's single-iteration floor), and
//! why that selection is *not* the same band another panel's
//! untransferable-geometry measurement named.
//!
//! **C++ baseline, 500 problems (250 `floor_wall` + 250 `cage`, seeds
//! `900001`/`900002`):** `498/500` solved (`99.6%`), median path length
//! `2.6598` (space-distance units, [`joint_model_group_space::JointModelGroupSpace::distance`]'s
//! own metric — see `tests/plan_space_parity.rs` for why that metric is the
//! one both sides can be compared in at all), median `ptc_evaluations` `2`.
//! This is the number condition 1 (success rate ≥90% of this) and condition
//! 3 (port median length within 1.3x of this `2.6598`) will be compared
//! against once a port-side measurement exists. Condition 2 (100% of
//! produced paths pass `moveit-scene`'s collision/constraint checks) needs
//! no C++ baseline at all — it is scored purely against the port's own
//! output — and is untouched by this round.
//!
//! ## Round 18: tolerance-floor survey
//!
//! `70a6b31` fixed the workspace's `serde_json` float parser because 8.1% of
//! committed fixture literals came back one ULP off, contaminating any
//! tolerance bisected against a value read that way. `rg -c
//! '1e-6|1e-9|1e-12' crates/moveit-planners-sbp/src --glob '!lib.rs'` (this
//! doc comment itself quotes that pattern, so `lib.rs` is excluded rather
//! than left to self-match) finds every such literal in `space.rs`,
//! `so2.rs`, `se3.rs`, `compound.rs`, `nn.rs`, `validity.rs`,
//! `joint_model_group_space.rs` and `sampling.rs` — none in `benches/` or
//! `examples/`. Every one of them compares two Rust-computed `f64`s (a
//! metric-axiom check, a norm-stays-near-1 check, a sample-stays-in-radius
//! check, or a formula cross-check against a hand-written Rust literal) —
//! none reads a `serde_json`-parsed value from a committed fixture and
//! compares it with a tolerance. The one site in this crate that *does*
//! compare against a live-oracle-derived value,
//! `tests/plan_space_parity.rs`'s `distance_probes` check, uses bit-exact
//! `assert_eq!`, not a tolerance, so the noisy-parser floor `70a6b31` fixed
//! was never a floor this crate's tolerances could have been bisected
//! against in the first place. Matching `moveit-constraints`'s own survey
//! (`a4c6fc6`) and `moveit-geometry`/`moveit-octomap`'s (`d239666`): none
//! affected here either.
//!
//! # Round 6 symbol audit
//!
//! This crate has two upstream relationships, not one, and they get audited
//! separately:
//!
//! - The state-space/algorithm modules ([`space`], [`so2`], [`se3`],
//!   [`compound`], [`nn`], [`rrt_connect`]) have **no upstream C++ file at
//!   all** (D3, see the top-of-file comment) — there is no OMPL header in
//!   this workspace to audit them against, so they are out of scope for a
//!   symbol-closure audit by construction, not by omission.
//! - [`registry`]'s [`registry::PlannerManager`]/[`registry::PlanningContext`]
//!   *do* have an upstream counterpart —
//!   `moveit_core/planning_interface/include/moveit/planning_interface/planning_interface.hpp`
//!   — read directly for this audit. Every symbol below:
//!
//! ## `PlannerConfigurationSettings` / `PlannerConfigurationMap`
//!
//! - Both -> unported: a stringly-typed `HashMap<String, String>` plugin
//!   config bag for a runtime plugin-loading boundary this crate doesn't
//!   have (D4: [`registry::PLANNER_MANAGERS`] is a compile-time registry).
//!   [`registry::PlanningRequest::params`]/[`registry::PlanningRequest::resolution`]
//!   are concretely-typed fields instead — see `registry`'s own doc comment,
//!   "Planner-specific tuning" paragraph.
//!
//! ## `PlanningContext`
//!
//! - `ctor(name, group)` -> unported: no persistent named/grouped identity
//!   object; `registry`'s private `RrtConnectContext` borrows exactly what
//!   `solve()` needs and nothing more.
//! - `getGroupName()`/`getName()` -> unported: the group name already lives
//!   on the [`registry::PlanningRequest`] the caller holds; no second
//!   accessor is needed since this port keeps no separate identity struct.
//! - `getPlanningScene()`/`getMotionPlanRequest()` -> unported: the caller
//!   already owns the `&mut PlanningScene` and [`registry::PlanningRequest`]
//!   it passed to [`registry::PlannerManager::get_planning_context`]; nothing
//!   needs them handed back.
//! - `setPlanningScene()`/`setMotionPlanRequest()` -> unported: every
//!   [`registry::PlanningContext`] here is single-use, built fresh per
//!   `solve()` — see [`registry::PlanningContext`]'s own "Deviation from
//!   upstream: no `terminate`/`clear`" doc section, which this shares the
//!   same reasoning with.
//! - `solve(MotionPlanResponse&)` and `solve(MotionPlanDetailedResponse&)` ->
//!   collapsed and ported as one [`registry::PlanningContext::solve`]
//!   returning `Result<`[`registry::PlanningResponse`]`, `[`registry::PlanError`]`>`;
//!   no detailed-response variant exists because nothing in this workspace
//!   consumes upstream's extra per-stage timing/trajectory detail.
//! - `terminate()`/`clear()` -> unported — see [`registry::PlanningContext`]'s
//!   own "Deviation from upstream" doc: no concurrency model here for
//!   cross-thread cancellation, and no context reuse to clear.
//! - `~PlanningContext()` (dtor) -> unported: no Rust equivalent to audit —
//!   every `registry::PlanningContext` implementor is dropped by ordinary
//!   Rust ownership, not an explicit virtual destructor.
//!
//! ## `PlannerManager`
//!
//! - `PlannerManager()` (ctor) / `~PlannerManager()` (dtor) -> unported: both
//!   are trivial no-ops upstream (`{}`); [`registry::RrtConnectManager`] and
//!   every other [`registry::PlannerManager`] implementor is an ordinary
//!   Rust value built by a struct literal or `#[derive(Default)]` and
//!   dropped automatically, with no constructor/destructor body to port.
//! - `initialize(model, node, parameter_namespace)` -> unported: no
//!   `rclcpp::Node`/ROS parameter namespace exists anywhere in this
//!   workspace (D1/D2); [`registry::RrtConnectManager`] needs no
//!   initialization step (`#[derive(Default)]`, a unit struct).
//! - `getDescription()` -> unported: no caller needs a human-readable
//!   description string; [`registry::PlannerManager::name`] (below) already
//!   identifies the manager uniquely for the registry lookup.
//! - `getPlanningAlgorithms(algs)` -> unported: this crate registers one
//!   algorithm per [`registry::PlannerManager`] impl (1:1, not 1:many like
//!   upstream's plugin-hosts-multiple-algorithms model), so there is no
//!   "algorithm names within one manager" list to enumerate;
//!   [`registry::PLANNER_MANAGERS`] itself is the cross-manager list.
//! - `getPlanningContext(scene, req, error_code)` -> ported as
//!   [`registry::PlannerManager::get_planning_context`], collapsed: the
//!   `moveit_msgs::msg::MoveItErrorCodes` out-parameter becomes the ordinary
//!   `Result<_, `[`registry::PlanError`]`>` return.
//! - `getPlanningContext(scene, req)` (the error-code-ignoring overload) ->
//!   unported: Rust already makes ignoring a `Result` an explicit
//!   `.unwrap()`/`let _ =` at the call site; no second overload is needed to
//!   spell that.
//! - `canServiceRequest(req)` -> unported: `get_planning_context` itself is
//!   the only admission check (it fails with e.g. `SbpError::UnknownGroup`);
//!   no separate dry-run query exists to ask "would you accept this" without
//!   also building the context.
//! - `setPlannerConfigurations(pcs)`/`getPlannerConfigurations()` -> unported:
//!   no `PlannerConfigurationMap` exists here (see above).
//! - `terminate()` (non-virtual, base-class async-cancel) -> unported, same
//!   reasoning as `PlanningContext::terminate()` above.
//! - Not upstream: [`registry::PlannerManager::name`] — new API this port's
//!   registry lookup needs (matches `moveit_kinematics::registry`'s
//!   `SolverRegistration` D4 precedent, per `registry.rs`'s own top-of-file
//!   comment).

pub mod compound;
mod error;
pub mod joint_model_group_space;
pub mod nn;
pub mod planning_scene_validity;
pub mod registry;
mod rrt_connect;
mod sampling;
pub mod se3;
pub mod so2;
pub mod space;
#[cfg(test)]
mod test_support;
pub mod validity;

pub use compound::{CompoundSpace, CompoundValue};
pub use error::SbpError;
pub use joint_model_group_space::JointModelGroupSpace;
pub use nn::Gnat;
pub use planning_scene_validity::PlanningSceneValidityChecker;
pub use registry::{
    PLANNER_MANAGERS, PlanError, PlannerManager, PlannerRegistration, PlanningContext,
    PlanningRequest, PlanningResponse, RrtConnectManager,
};
pub use rrt_connect::{PlanningFailure, RrtConnectParams, Termination, rrt_connect};
pub use se3::{Se3Space, Se3State};
pub use so2::So2Space;
pub use space::{RealVectorSpace, StateSpace};
pub use validity::{DiscreteMotionValidator, MotionValidator, StateValidityChecker};
