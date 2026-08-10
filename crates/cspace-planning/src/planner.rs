// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/planning_interface/include/moveit/planning_interface/planning_interface.hpp

//! [`PlannerManager`] and [`PlanningContext`]: the two-step
//! "build a context, then solve it" interface every concrete planner
//! implements, ported from `planning_interface::PlannerManager`
//! (`planning_interface.hpp:148-211`) and
//! `planning_interface::PlanningContext` (`:78-143`).
//!
//! # Why these live here and not in a planner crate
//!
//! Upstream puts all four of `MotionPlanRequest`, `MotionPlanResponse`,
//! `PlanningContext` and `PlannerManager` in one package
//! (`moveit_core/planning_interface`): `planning_interface.hpp` includes
//! `planning_request.hpp` and `planning_response.hpp` on its first two
//! lines (`:40-41`), because the two traits are *defined* by the request
//! and response types they move between. This module is that unit's Rust
//! half, and [`crate::request::PlanningRequest`]/
//! [`crate::response::PlanningResponse`] are the other. Splitting them
//! across crates is what produced the defect this module exists to close —
//! `cspace_planners::sbp::registry` used to declare its own
//! `PlanningRequest`/`PlanningResponse`/`PlanningContext`/`PlannerManager`
//! set that shared only *names* with this crate's, so
//! [`crate::pipeline::generate_plan`] could not call the workspace's only
//! concrete planner (PORTING-PLAN.md D8/§140.2).
//!
//! Registration — going from a planner-id *string* to a
//! [`PlannerManager`] instance — is deliberately *not* here: it is
//! `cspace_planning::planner_registry`'s job, one crate up, so that the
//! `linkme::distributed_slice` it needs keeps its `unsafe_code = "allow"`
//! confined to a crate with no other code in it (PORTING-PLAN.md §140.1).
//! Nothing in this module or in [`crate::pipeline`] resolves a name.
//!
//! # Scope — full declaration audit of `planning_interface.hpp`
//!
//! Every *public* declaration the header makes, one bullet each, with a
//! disposition: `ported as <symbol>` / `unported (<reason>)` / `D<n>
//! excludes it` / `no Rust counterpart exists`. The header is 214 lines and
//! declares two classes and two namespace-level items; nothing else. The
//! per-class counts below are `tools/ci/count-public-declarations.sh`'s, run
//! against the pinned checkout, not hand-tallied:
//! `PlanningContext` **12**, `PlannerManager` **11** — 23 bullets, and the
//! four namespace-level ones bring the walk to 27. Protected data
//! (`name_`/`group_`/`planning_scene_`/`request_`, `config_settings_`) is
//! implementation detail and is not audited, matching
//! `crates/cspace-planning/src/scene/scene.rs`'s convention for the same reason.
//!
//! Three of the 23 are ported. That ratio is the point of the file rather
//! than an omission: upstream's two classes carry a plugin lifecycle (ROS
//! parameters, re-settable contexts, asynchronous termination) that D1 and
//! this port's single-use contexts remove outright, and what survives is
//! the two-step "build a context, then solve it" shape.
//!
//! ## Namespace level (4)
//!
//! - `struct PlannerConfigurationSettings` (`:56-69`) — **ported as**
//!   [`PlannerConfigurationSettings`]. Was "D1 excludes it" until
//!   PORTING-PLAN.md §285: D1 drops the ROS *parameter server* that fills
//!   this struct upstream, not the struct, and `/set_planner_params` fills
//!   it over a service D1 says nothing about.
//! - `typedef PlannerConfigurationMap` (`:72`) — **ported as**
//!   [`PlannerConfigurationMap`], same.
//! - `MOVEIT_CLASS_FORWARD(PlanningContext)` (`:74`) — **no Rust
//!   counterpart exists.** It defines `PlanningContextPtr`/`ConstPtr`/
//!   `WeakPtr` `std::shared_ptr` aliases; this port returns
//!   `Box<dyn PlanningContext<'m> + 'a>` and has no shared ownership to
//!   alias.
//! - `MOVEIT_CLASS_FORWARD(PlannerManager)` (`:145`) — same; the registry
//!   holds `&'static dyn PlannerManager`.
//!
//! ## `PlanningContext` (12)
//!
//! - `PlanningContext(const std::string& name, const std::string& group)`
//!   (`:82`) — **unported.** Upstream stores the pair so the context can be
//!   re-pointed later; here the group is
//!   [`crate::PlanningRequest::group_name`] at the one call site that builds
//!   the context, and the name is [`PlannerManager::name`].
//! - `virtual ~PlanningContext()` (`:84`) — **no Rust counterpart exists**
//!   (`Drop`, and nothing here needs a custom one).
//! - `getGroupName()` (`:87-90`) — **unported (no reader).** See the
//!   constructor bullet.
//! - `getName()` (`:93-96`) — **unported (no reader)**, same.
//! - `getPlanningScene()` (`:99-102`) — **unported (no reader).** The scene
//!   is the caller's; [`PlannerManager::get_planning_context`] borrows it in.
//! - `getMotionPlanRequest()` (`:105-108`) — **unported (no reader).**
//!   [`crate::pipeline::generate_plan`] keeps the request itself, which is
//!   why the trait borrows rather than moves it.
//! - `setPlanningScene(...)` (`:111`) — **unported.** Upstream's context is
//!   re-settable so one object can serve a second query; every context here
//!   is built for exactly one query and dropped, so there is no second query
//!   to re-point it at.
//! - `setMotionPlanRequest(...)` (`:114`) — **unported**, same.
//! - `virtual void solve(MotionPlanResponse& res) = 0` (`:118`) — **ported
//!   as** [`PlanningContext::solve`], with the mutated-in-place output
//!   parameter and its `error_code` replaced by one `Result`.
//! - `virtual void solve(MotionPlanDetailedResponse& res) = 0` (`:122`) —
//!   **unported.** The detailed response has no port on this side
//!   (`doc/port-coverage.md`'s `planning_response.hpp` row: its only
//!   counterpart is `cspace_planners::chomp::ChompSolution`, narrowed to what
//!   chomp fills).
//! - `virtual bool terminate() = 0` (`:126`) — **unported.** Asynchronous
//!   cancellation needs a caller on another thread holding the same context;
//!   [`PlanningContext::solve`] takes `&mut self` and runs to completion
//!   synchronously, so no such caller can exist.
//! - `virtual void clear() = 0` (`:129`) — **unported**, same: reuse-after-
//!   clear has no reuse to serve.
//!
//! ## `PlannerManager` (11)
//!
//! - `PlannerManager()` (`:151-153`) — **no Rust counterpart exists.** A
//!   trait has no constructor; each concrete manager brings its own.
//! - `virtual ~PlannerManager()` (`:155-157`) — **no Rust counterpart
//!   exists** (`Drop`).
//! - `virtual bool initialize(model, node, parameter_namespace)`
//!   (`:164-165`) — **D1 excludes it.** It takes an `rclcpp::Node` and a ROS
//!   parameter namespace; the per-planner configuration it reads is
//!   constructor arguments on the concrete manager here.
//! - `virtual std::string getDescription() const = 0` (`:168`) — **ported
//!   as** [`PlannerManager::name`], which additionally serves as the
//!   registry key; see that method's own comment for why this port
//!   deliberately merges the two roles upstream keeps apart.
//! - `virtual void getPlanningAlgorithms(std::vector<std::string>&) const`
//!   (`:172`) — **unported (no caller).**
//!   `PlanningPipeline::generatePlan` does not call it
//!   (`planning_pipeline.cpp:294-330` calls `getPlanningContext`, `solve`
//!   and `getDescription` only); its upstream caller is `move_group`'s
//!   `query_planners` service capability
//!   (`query_planners_service_capability.cpp:98,102`).
//! - `virtual PlanningContextPtr getPlanningContext(planning_scene, req,
//!   error_code) const = 0` (`:181-183`) — **ported as**
//!   [`PlannerManager::get_planning_context`], whose "empty ptr is returned
//!   and error code is set" contract is its `Result`.
//! - `PlanningContextPtr getPlanningContext(planning_scene, req) const`
//!   (`:186-187`) — **unported.** This overload exists upstream only to drop
//!   the `error_code` out-parameter; with a `Result` return the same
//!   spelling is `.ok()` at the call site, so a second method would be one
//!   name for a combinator.
//! - `virtual bool canServiceRequest(const MotionPlanRequest&) const = 0`
//!   (`:190`) — **unported (no caller)**, same citation as
//!   `getPlanningAlgorithms`; its other upstream callers are the concrete
//!   plugins' own early-outs.
//! - `virtual void setPlannerConfigurations(const PlannerConfigurationMap&)`
//!   (`:193`) — **unported, replaced by construction.** Upstream needs a
//!   setter because pluginlib hands it an already-built plugin;
//!   `crate::planner_registry::PlannerRegistration::construct` takes the
//!   [`PlannerConfigurationMap`] as its argument instead, so a manager
//!   cannot exist without the configuration it plans under and no caller
//!   can forget to install one (PORTING-PLAN.md §285).
//! - `const PlannerConfigurationMap& getPlannerConfigurations() const`
//!   (`:196-199`) — **unported (no reader).** Its upstream caller is
//!   `getParams` (`query_planners_service_capability.cpp:132`), reading
//!   back through the manager because the manager owns the store; here the
//!   store is the node's and `get_planner_params` reads it directly
//!   (`ros/cspace-ros/src/planner_params.rs:237`), so nothing asks a
//!   manager what it was configured with. Upstream's round trip is not
//!   lossless either — OMPL's override rewrites what it is handed before
//!   storing it (`ompl_interface.cpp:85-100`) — so the two are not the
//!   same observation and one is not a stand-in for the other.
//! - `void terminate() const` (`:202`) — **unported**, for
//!   `PlanningContext::terminate`'s reason: it forwards to the contexts this
//!   manager built, and nothing here outlives its `solve`.
//!
//! The two traits' one concrete implementation is
//! `cspace_planners::sbp::registry` (`RrtConnectManager` and its context),
//! which cites this same header as the "plugin half" it stands in for; the
//! dispositions above are what that implementation has to satisfy and are
//! audited here rather than there, because they are properties of the
//! interface and not of any one planner.

use std::collections::BTreeMap;

use crate::scene::PlanningScene;
use cspace_collision::ParryCollisionEnv;

use crate::request::PlanningRequest;
use crate::response::PlanningResponse;

/// Upstream's `planning_interface::PlannerConfigurationSettings`
/// (`planning_interface.hpp:56-69`): one planning algorithm's settings, for
/// one group.
///
/// `config` is a [`BTreeMap`] and not a `HashMap` because upstream's is a
/// `std::map<std::string, std::string>` whose ordering is observable on the
/// wire: `get_planner_params` fills its key/value arrays by iterating it
/// (`query_planners_service_capability.cpp:150-154`), so they come back
/// sorted by key.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct PlannerConfigurationSettings {
    /// The group (as defined in the SRDF) this configuration is meant for,
    /// empty for a global one.
    pub group: String,
    /// This configuration's own key in a [`PlannerConfigurationMap`] —
    /// [`configuration_name`]'s output. Upstream states the same two forms
    /// on the field itself (`planning_interface.hpp:60-64`).
    pub name: String,
    /// "Key-value pairs of settings that get passed to the planning
    /// algorithm" (`planning_interface.hpp:66`). Upstream's own note on the
    /// struct is that "settings with unknown keys are ignored"
    /// (`planning_interface.hpp:54`), which is why a manager reading one key
    /// out of this map is not obliged to reject the others.
    pub config: BTreeMap<String, String>,
}

/// Upstream's `planning_interface::PlannerConfigurationMap`
/// (`planning_interface.hpp:72`): map from
/// [`PlannerConfigurationSettings::name`] to the settings.
///
/// This is the type `crate::planner_registry::PlannerRegistration::construct`
/// takes, so every [`PlannerManager`] the registry builds is built from one.
pub type PlannerConfigurationMap = BTreeMap<String, PlannerConfigurationSettings>;

/// The key a configuration is stored under: a bare `planner_config` when no
/// group is named, `group[planner_config]` otherwise.
///
/// Upstream spells this twice in the writer alone — `getParams` builds the
/// group form inline (`query_planners_service_capability.cpp:142`) and
/// `setParams` builds it again in a ternary (`:190-191`) — and a third time
/// in the reader (`planning_context_manager.cpp:508-510`). One function here
/// instead, so the service that writes a configuration and the planner that
/// reads it cannot disagree about the spelling across the crate boundary.
pub fn configuration_name(group: &str, planner_config: &str) -> String {
    if group.is_empty() {
        planner_config.to_string()
    } else {
        format!("{group}[{planner_config}]")
    }
}

/// The configuration a query runs under, or `None` if the map holds neither
/// a configuration for this `(group_name, planner_id)` pair nor a default
/// for the group.
///
/// Ports `PlanningContextManager::getPlanningContext`'s configuration
/// selection (`planning_context_manager.cpp:504-526`): a non-empty
/// `planner_id` is looked up first — verbatim when it already contains the
/// group name, as `group[planner_id]` otherwise — and a miss falls back to
/// the entry keyed by the bare group name, which is upstream's "group's
/// default configuration" form (`planning_interface.hpp:60-64`).
///
/// # Deviation from upstream: a total miss is not fatal here
///
/// Upstream returns *no context* when neither key is present
/// (`planning_context_manager.cpp:519-526`), because every OMPL tuning
/// parameter comes from `ompl_planning.yaml` and a manager with no
/// configuration has nothing to plan with. Every [`PlannerManager`] in this
/// workspace instead carries compiled-in defaults (see
/// `cspace_planners::sbp::RrtConnectManager::default`), so a configuration is
/// an *overlay* on those and `None` means "plan with your own defaults"
/// rather than "refuse". Making the miss fatal would mean a node that plans
/// nothing at all until some client has called `/set_planner_params`.
pub fn configuration_for<'c>(
    configs: &'c PlannerConfigurationMap,
    group_name: &str,
    planner_id: &str,
) -> Option<&'c PlannerConfigurationSettings> {
    if !planner_id.is_empty() {
        // `req.planner_id.find(req.group_name) == std::string::npos ?
        //  group + "[" + planner_id + "]" : planner_id` (`:508-510`): a
        // planner_id that already names the group is a whole key already.
        let key = if planner_id.contains(group_name) {
            planner_id.to_string()
        } else {
            configuration_name(group_name, planner_id)
        };
        if let Some(settings) = configs.get(&key) {
            return Some(settings);
        }
    }
    configs.get(group_name)
}

/// Opaque planner failure: a [`PlannerManager`] implementation boxes
/// whatever error its own concrete planner produced (e.g.
/// `cspace_planners::sbp::PlanError`) into this. This crate cannot name a
/// concrete planner error type — it
/// does not, and must not, depend on any concrete planner crate; the
/// dependency runs the other way (see this module's doc, "Why these live
/// here and not in a planner crate").
///
/// Replaces the `moveit_msgs::msg::MoveItErrorCodes& error_code` out
/// parameter of `getPlanningContext` (`planning_interface.hpp:183`) and the
/// `res.error_code` a `solve()` sets (`planning_response.hpp:63`) alike.
pub type PlanError = Box<dyn std::error::Error + Send + Sync + 'static>;

/// A planning query bound to a scene, ready to run. Ports
/// `planning_interface::PlanningContext` (`planning_interface.hpp:78-143`);
/// see this module's doc for the members and methods that have no
/// counterpart here and why.
pub trait PlanningContext<'m> {
    /// Runs the query to completion. Ports
    /// `virtual void solve(MotionPlanResponse& res)` (`:118`), with
    /// upstream's mutated-in-place output parameter and its `error_code`
    /// replaced by one `Result`.
    fn solve(&mut self) -> Result<PlanningResponse<'m>, PlanError>;
}

/// Builds a [`PlanningContext`] for a `(scene, request)` pair. Ports
/// `planning_interface::PlannerManager` (`planning_interface.hpp:148-211`);
/// see this module's doc for the methods that have no counterpart here and
/// why.
///
/// # Deviation from upstream: specialized to [`ParryCollisionEnv`]
///
/// Upstream's `PlannerManager` is not itself generic over the collision
/// checker — the scene it is given already owns one.
/// [`crate::scene::PlanningScene`] is generic over `E: CollisionEnv<..>`
/// instead of owning one (see that type's own doc comment), which would
/// force [`PlannerManager::get_planning_context`] to be generic over `E`
/// too — and a generic *type* parameter on a trait method breaks `dyn`
/// object-safety (a generic *lifetime* parameter, like this method's
/// `'a`/`'m`, does not). [`ParryCollisionEnv`] is the only
/// [`cspace_collision::CollisionEnv`] implementation anywhere in this
/// workspace (PORTING-PLAN.md D4.5: parry3d-f64 replaces FCL+Bullet
/// outright, not as one plugin among several), so specializing directly to
/// it costs nothing today and keeps this trait usable as
/// `dyn PlannerManager` — which `cspace_planning::planner_registry`'s slice and
/// [`crate::pipeline::generate_plan`]'s `planners` parameter both require.
pub trait PlannerManager {
    /// This manager's name. Plays the role of upstream's
    /// `getDescription()` (`planning_interface.hpp:168`, the string
    /// `generatePlan` logs at `planning_pipeline.cpp:311,318,324` and that
    /// [`crate::pipeline::PipelineError::Planner`] carries here), and is
    /// simultaneously the key a caller matches on when scanning
    /// `cspace_planning::planner_registry`'s `PLANNER_MANAGERS` — upstream keeps
    /// those two apart (a pluginlib class name vs. a free-form
    /// description), this port deliberately does not, so that a name found
    /// in the registry and a name reported in an error are the same string.
    fn name(&self) -> &'static str;

    /// Builds a [`PlanningContext`] that will plan `request` against
    /// `scene`, using `env` for collision checking. Ports
    /// `getPlanningContext(planning_scene, req, error_code)`
    /// (`planning_interface.hpp:181-183`), whose "empty ptr is returned and
    /// error code is set" contract is this `Result`.
    ///
    /// Fails only if `request` cannot be resolved against
    /// `scene.robot_model()` (e.g. an unknown
    /// [`PlanningRequest::group_name`]); planning failure itself surfaces
    /// from [`PlanningContext::solve`], matching upstream's own split
    /// between context construction (`planning_pipeline.cpp:306-315`) and
    /// solving (`:318-329`).
    ///
    /// `request` is borrowed, not moved: upstream takes
    /// `const MotionPlanRequest&` and copies what the context keeps into
    /// its own `request_` member (`:142`), and
    /// [`crate::pipeline::generate_plan`] needs the request back afterwards
    /// to feed the response-adapter chain and the next planner in the
    /// chain.
    fn get_planning_context<'a, 'm>(
        &self,
        scene: &'a mut PlanningScene<'m>,
        env: &'a ParryCollisionEnv,
        request: &PlanningRequest,
    ) -> Result<Box<dyn PlanningContext<'m> + 'a>, PlanError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settings(group: &str, planner_config: &str) -> PlannerConfigurationSettings {
        let name = configuration_name(group, planner_config);
        PlannerConfigurationSettings {
            group: group.to_string(),
            name,
            config: BTreeMap::new(),
        }
    }

    fn map(entries: &[PlannerConfigurationSettings]) -> PlannerConfigurationMap {
        entries
            .iter()
            .map(|s| (s.name.clone(), s.clone()))
            .collect()
    }

    /// The writer's two forms, which the reader's key has to match:
    /// `planner_config` alone for a global configuration,
    /// `group[planner_config]` for a group's
    /// (`query_planners_service_capability.cpp:190-191`).
    #[test]
    fn a_configuration_name_is_bracketed_only_when_a_group_is_named() {
        assert_eq!(configuration_name("", "RRTConnect"), "RRTConnect");
        assert_eq!(configuration_name("arm", "RRTConnect"), "arm[RRTConnect]");
    }

    /// `planner_id` non-empty and not containing the group: upstream looks
    /// up `group[planner_id]` and nothing else first (`:508-510`).
    #[test]
    fn a_named_planner_selects_the_group_bracketed_entry() {
        let configs = map(&[settings("arm", "RRTConnect"), settings("", "arm")]);
        assert_eq!(
            configuration_for(&configs, "arm", "RRTConnect").map(|s| s.name.as_str()),
            Some("arm[RRTConnect]")
        );
    }

    /// The `planner_id.find(group_name) != npos` arm (`:506-508`): an id
    /// that already names the group is used verbatim, *not* wrapped a
    /// second time into `arm[arm[RRTConnect]]`.
    #[test]
    fn a_planner_id_that_already_names_the_group_is_a_whole_key() {
        let configs = map(&[settings("arm", "RRTConnect")]);
        assert_eq!(
            configuration_for(&configs, "arm", "arm[RRTConnect]").map(|s| s.name.as_str()),
            Some("arm[RRTConnect]")
        );
    }

    /// The fallback arm (`:518-526`): a `planner_id` the map does not hold
    /// falls through to the group's own default entry rather than to
    /// nothing, so a configuration set with an empty `planner_config`-as-
    /// group-name still governs the query.
    #[test]
    fn an_unknown_planner_id_falls_back_to_the_group_default_entry() {
        let configs = map(&[settings("", "arm")]);
        assert_eq!(
            configuration_for(&configs, "arm", "PRM").map(|s| s.name.as_str()),
            Some("arm")
        );
        // And with no `planner_id` at all, which is what an unmodified
        // client sends: the first lookup is skipped entirely.
        assert_eq!(
            configuration_for(&configs, "arm", "").map(|s| s.name.as_str()),
            Some("arm")
        );
    }

    /// A global configuration -- what `/set_planner_params` writes when the
    /// request names no group -- is keyed by the planner config alone, and
    /// upstream's reader never looks there. Asserted rather than left
    /// implicit because it is the case a client is most likely to send and
    /// be surprised by, and because a lookup rule that fell back to a bare
    /// `planner_id` key would pass every other test in this module.
    #[test]
    fn a_global_configuration_does_not_govern_a_grouped_query() {
        let configs = map(&[settings("", "RRTConnect")]);
        assert!(configuration_for(&configs, "arm", "RRTConnect").is_none());
        // The same entry does govern when the query names no group, which
        // is what makes the miss above a property of the key rule rather
        // than of the entry being unreachable.
        assert_eq!(
            configuration_for(&configs, "", "RRTConnect").map(|s| s.name.as_str()),
            Some("RRTConnect")
        );
    }

    #[test]
    fn an_empty_map_selects_nothing() {
        assert!(configuration_for(&PlannerConfigurationMap::new(), "arm", "RRTConnect").is_none());
    }
}
