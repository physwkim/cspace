// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_ros/move_group/src/default_capabilities/query_planners_service_capability.cpp
//   moveit_ros/move_group/src/default_capabilities/query_planners_service_capability.hpp

//! `MoveGroupQueryPlannersService`, upstream's one capability that serves all
//! three planner-parameter services.
//!
//! Upstream creates the three in a single `initialize()`
//! (`query_planners_service_capability.cpp:58`, `:65`, `:70`), and they share
//! one piece of state -- the planner manager's configuration map -- so they
//! are ported together, behind one [`spawn`]:
//!
//! * `query_planner_interface` (`moveit_msgs/srv/QueryPlannerInterfaces`)
//! * `get_planner_params` (`moveit_msgs/srv/GetPlannerParams`)
//! * `set_planner_params` (`moveit_msgs/srv/SetPlannerParams`)
//!
//! The names are `move_group::QUERY_PLANNERS_SERVICE_NAME`,
//! `GET_PLANNER_PARAMS_SERVICE_NAME` and `SET_PLANNER_PARAMS_SERVICE_NAME`
//! (`move_group/capability_names.hpp:46`, `:48`, `:50`). They are advertised
//! unqualified, for the same reason `move_action` is: `MoveGroupInterface`
//! resolves them against the node's namespace.
//!
//! # None of the three has an error channel
//!
//! `QueryPlannerInterfaces` replies with `planner_interfaces` alone,
//! `GetPlannerParams` with `params` alone, and `SetPlannerParams`'s response
//! is *empty* -- no field at all. Upstream reports every failure in these
//! handlers (unknown pipeline, no planner plugin, mismatched key/value
//! counts) by logging and returning, so a client sees an empty answer and
//! cannot distinguish "nothing configured" from "your request was rejected".
//! That is upstream's wire contract, and this port keeps it rather than
//! inventing a code field the message has nowhere to carry.
//!
//! # What this port resolves a `pipeline_id` against
//!
//! Upstream resolves one through `resolvePlanningPipeline`
//! (`move_group_capability.cpp:223-246`): an empty id means "the default
//! pipeline", and any other id is looked up in
//! `moveit_cpp_->getPlanningPipelines()`, with a miss logged and answered as
//! a null pipeline.
//!
//! This workspace has no planning-pipeline registry at all -- there is one
//! *planner manager* registry,
//! [`static@moveit_planner_registry::PLANNER_MANAGERS`],
//! which corresponds to upstream's planner-plugin level (`pluginlib`'s
//! `planning_interface::PlannerManager`), not to its pipeline level. So the
//! rule here is upstream's with an empty pipeline map: the empty id resolves
//! to this node's one default surface, and **every** non-empty id is a miss.
//! A manager's name is deliberately not accepted as a pipeline id -- a
//! manager is not a pipeline, and letting one string mean both is the dual
//! meaning that would have to be untangled once real pipelines exist.
//!
//! That rule is also what an unmodified client actually exercises:
//! `MoveGroupInterface::getPlannerParams`/`setPlannerParams`
//! (`move_group_interface.cpp:280`, `:298`) set `planner_config`, `group` and
//! `replace`, and never set `pipeline_id` at all, so it arrives
//! default-constructed and empty.
//!
//! # What a description can honestly carry here
//!
//! Upstream fills a `PlannerInterfaceDescription` from three planner-manager
//! calls (`query_planners_service_capability.cpp:99-102`): `getDescription()`
//! into `name`, the pipeline's map key into `pipeline_id`, and
//! `getPlanningAlgorithms()` into `planner_ids`.
//!
//! This port's [`moveit_planning::planner::PlannerManager`] has none of
//! those three. It has `name()`, and `PlannerRegistration` carries that
//! same name plus a constructor. So:
//!
//! * `name` is the registration's name. Upstream's `getDescription` is "a
//!   short string that identifies the planning interface"
//!   (`planning_interface.hpp:168`) -- OMPL answers `"OMPL"` -- and the
//!   registration name is the only string of that kind this port has.
//! * `pipeline_id` is empty, which is exactly the id this capability accepts
//!   as "the default": the value advertised is a value the other two services
//!   take back.
//! * `planner_ids` is empty. That is not a stub: upstream's
//!   `PlannerManager::getPlanningAlgorithms` base implementation is
//!   `algs.clear()` (`planning_interface.cpp:124-128`), so any manager that
//!   does not override it reports no algorithms, and no manager in this
//!   workspace has an override to port.
//!
//! # Registration order is sorted, and that is load-bearing
//!
//! `MoveGroupInterface::getInterfaceDescription`
//! (`move_group_interface.cpp:247-262`) takes `planner_interfaces.front()` --
//! the client reads *one* description off the front of this list. The
//! registry it is derived from is a `linkme::distributed_slice`, whose
//! iteration order is not a contract in this workspace: it has already
//! flipped once from an unrelated dependency change (PORTING-PLAN.md §219).
//! [`registered_interface_names`] therefore sorts by name, so which
//! description a client gets is a property of the names and not of link
//! order.
//!
//! # What a `set` reaches, and what it does not
//!
//! Upstream's store is `PlannerManager::config_settings_`
//! (`planning_interface.hpp:210`), written by `setPlannerConfigurations` and
//! read back by `getPlannerConfigurations` -- the same instance the pipeline
//! plans with, so a `set` persists for every subsequent plan on that
//! pipeline, for the life of the node.
//!
//! Here the map is owned by the node ([`PlannerConfigurations`]) rather than
//! by a manager instance, because `PlannerRegistration::construct` builds a
//! *fresh* `Box<dyn PlannerManager>` on each call: a config written into a
//! constructed instance would be dropped with it. The node owning the store
//! is what makes the write survive at all.
//!
//! Upstream's `setParams` ends at
//! `planner_interface->setPlannerConfigurations(configs)`
//! (`query_planners_service_capability.cpp:205`), handing the map to the
//! instance the pipeline plans with. The equivalent here is not a call at
//! all: [`moveit_planner_registry::resolve_planner`] takes the store as an
//! argument, so the manager the next plan runs on is *built from* whatever
//! `set` last wrote (PORTING-PLAN.md §285). [`spawn`] therefore returns the
//! store rather than keeping it private -- the binary hands the same
//! [`PlannerConfigurations`] to `moveit_ros::move_group::plan_only`, and
//! that sharing is what makes a `set` reach a plan.
//!
//! Which entry of the map a given plan runs under is the planner's lookup,
//! not this module's: [`moveit_planning::configuration_for`] ports
//! `PlanningContextManager::getPlanningContext`'s selection
//! (`planning_context_manager.cpp:504-526`), which keys off the *request's*
//! group and `planner_id`. One consequence is worth stating because a client
//! will hit it: a `set` with no `group` is stored under the bare
//! `planner_config`, and that key is one the reader never looks for -- it
//! searches `group[planner_id]` and then the bare group name. That is
//! upstream's behaviour too, not a gap introduced here.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::rc::Rc;

use futures::executor::LocalSpawner;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
// The slice lives in `moveit-planner-registry`; `moveit-planners-sbp` reaches
// it only through a private `use` and re-exports it nowhere, so importing it
// from there compiles on a branch whose base predates that crate and stops
// resolving once the branch merges. This file was written before the merge and
// did import it from `moveit-planners-sbp`; because the file is new, git merged
// it clean and the breakage surfaced only as a compile error. Two intra-doc
// links above pointed the same way and fail the doc build rather than the
// compile, so `cargo build` alone would not have found them.
use moveit_planner_registry::PLANNER_MANAGERS;
use moveit_planning::{PlannerConfigurationMap, configuration_name};
use r2r::QosProfile;
use r2r::moveit_msgs::msg::{PlannerInterfaceDescription, PlannerParams};
use r2r::moveit_msgs::srv::{GetPlannerParams, QueryPlannerInterfaces, SetPlannerParams};

/// The node's live configuration store, shared by the `get` and `set` tasks
/// and by every plan the node runs.
///
/// `Rc<RefCell<_>>` and not a lock, for the reason this binary's module doc
/// gives for the monitored scene: one `LocalPool` on one thread, and no
/// borrow is held across an `.await`.
///
/// The map itself is [`moveit_planning::PlannerConfigurationMap`], defined
/// where upstream defines it (`planning_interface.hpp:72`, next to the
/// `PlannerManager` that plans under it) rather than here: it used to live
/// in this file, which meant the planner-registry crate could not name the
/// type it now takes as a constructor argument (PORTING-PLAN.md §285).
pub type PlannerConfigurations = Rc<RefCell<PlannerConfigurationMap>>;

/// Whether `pipeline_id` names a pipeline this node serves.
///
/// Upstream's `resolvePlanningPipeline` (`move_group_capability.cpp:223-246`)
/// over an empty pipeline map: the empty id is the default and resolves,
/// every other id misses. See this module's doc for why a planner-manager
/// name is not accepted here.
fn resolves_pipeline(pipeline_id: &str) -> bool {
    pipeline_id.is_empty()
}

/// Every registered planner manager's name, sorted.
///
/// Sorted rather than in slice order because a client reads
/// `planner_interfaces.front()` and `linkme`'s order is not a contract -- see
/// this module's doc.
pub fn registered_interface_names() -> Vec<&'static str> {
    sorted_names(
        PLANNER_MANAGERS
            .iter()
            .map(|registration| registration.name)
            .collect(),
    )
}

/// The ordering half of [`registered_interface_names`], split out because the
/// live registry has exactly one entry and so cannot exhibit an ordering at
/// all -- a test driving `registered_interface_names` directly would pass
/// with the sort deleted.
fn sorted_names(mut names: Vec<&'static str>) -> Vec<&'static str> {
    names.sort_unstable();
    names
}

/// `MoveGroupQueryPlannersService::queryInterface`
/// (`query_planners_service_capability.cpp:76-105`).
///
/// Takes the names rather than reading the registry itself so the boundary
/// tests below can drive it with an empty registry and with several
/// registrations, neither of which this workspace's own registry can produce
/// today (it has exactly one entry).
fn query_response(interface_names: &[&str]) -> QueryPlannerInterfaces::Response {
    QueryPlannerInterfaces::Response {
        planner_interfaces: interface_names
            .iter()
            .map(|name| PlannerInterfaceDescription {
                name: (*name).to_string(),
                pipeline_id: String::new(),
                planner_ids: Vec::new(),
            })
            .collect(),
    }
}

/// `MoveGroupQueryPlannersService::getParams`
/// (`query_planners_service_capability.cpp:107-155`).
///
/// `interface_count` stands in for upstream's two preconditions between the
/// pipeline and the config map -- `planner_plugin_names.empty()` and a null
/// `getPlannerManager` (`:120-133`) -- both of which answer an empty reply.
/// With no planner manager registered there is no configuration surface to
/// read, which is the same branch.
fn get_response(
    configs: &PlannerConfigurationMap,
    interface_count: usize,
    req: &GetPlannerParams::Request,
) -> GetPlannerParams::Response {
    let mut merged: BTreeMap<String, String> = BTreeMap::new();
    if resolves_pipeline(&req.pipeline_id) && interface_count > 0 {
        // "fetch default params first" (`:139`).
        if let Some(settings) = configs.get(&req.planner_config) {
            merged.extend(settings.config.iter().map(|(k, v)| (k.clone(), v.clone())));
        }
        if !req.group.is_empty() {
            // "merge in group-specific params" (`:146`) -- but the call is
            // `std::map::insert`, which does *not* overwrite an existing key,
            // so a parameter already supplied by the global configuration
            // keeps the global value and the group-specific one is dropped.
            // `or_insert` is that semantic; `extend`/`insert` would be the
            // opposite one, and the difference is observable to any client
            // that sets a key both globally and per group.
            if let Some(settings) =
                configs.get(&configuration_name(&req.group, &req.planner_config))
            {
                for (key, value) in &settings.config {
                    merged.entry(key.clone()).or_insert_with(|| value.clone());
                }
            }
        }
    }

    let (keys, values): (Vec<String>, Vec<String>) = merged.into_iter().unzip();
    GetPlannerParams::Response {
        params: PlannerParams {
            keys,
            values,
            // Upstream never fills `descriptions` here, and never reads it in
            // `setParams` either -- the field is inert on both legs.
            descriptions: Vec::new(),
        },
    }
}

/// `MoveGroupQueryPlannersService::setParams`
/// (`query_planners_service_capability.cpp:157-206`).
///
/// Mutates `configs` in place where upstream copies the map, edits the copy
/// and installs it with `setPlannerConfigurations` (`:205`). That copy exists
/// because upstream's store lives behind an accessor returning a `const&`;
/// the observable result -- the map after the call -- is the same, and the
/// store here is owned outright.
///
/// Returns whether anything was written, which the reply cannot carry (see
/// this module's doc): it is what the boundary tests assert on, and what the
/// caller logs.
fn apply_set(
    configs: &mut PlannerConfigurationMap,
    interface_count: usize,
    req: &SetPlannerParams::Request,
) -> bool {
    // `:160-164`: a mismatched pair of arrays is rejected before anything is
    // resolved, so a malformed request cannot half-write a configuration.
    if req.params.keys.len() != req.params.values.len() {
        return false;
    }
    if !resolves_pipeline(&req.pipeline_id) || interface_count == 0 {
        return false;
    }

    let name = configuration_name(&req.group, &req.planner_config);
    let settings = configs.entry(name.clone()).or_default();
    settings.group.clone_from(&req.group);
    settings.name = name;
    // `:190-193`: `replace` clears the existing parameters first; without it
    // the request augments them.
    if req.replace {
        settings.config.clear();
    }
    for (key, value) in req.params.keys.iter().zip(req.params.values.iter()) {
        // `config.config[key] = value` (`:196`) -- assignment through
        // `operator[]`, which *does* overwrite, unlike the `insert` on the
        // read leg above.
        settings.config.insert(key.clone(), value.clone());
    }
    true
}

/// Creates the three services on `node`, spawns their handler tasks, and
/// returns the store they share.
///
/// One entry point, mirroring upstream's single
/// `MoveGroupQueryPlannersService::initialize()`
/// (`query_planners_service_capability.cpp:56-74`), so the node binary
/// registers this capability the way upstream loads it: as one thing.
///
/// The store is *returned* rather than kept private because a planner has to
/// be built from it -- that is this port's whole equivalent of upstream's
/// closing `setPlannerConfigurations` call (`:205`), see this module's doc.
/// Returning it also means the binary cannot accidentally plan against a
/// second, empty map: there is one `PlannerConfigurationMap::new()` in this
/// module and every reader holds an `Rc` onto it.
pub fn spawn(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
) -> Result<PlannerConfigurations, String> {
    let query = node
        .create_service::<QueryPlannerInterfaces::Service>(
            "query_planner_interface",
            QosProfile::default(),
        )
        .map_err(|e| format!("create_service(query_planner_interface): {e}"))?;
    let get = node
        .create_service::<GetPlannerParams::Service>("get_planner_params", QosProfile::default())
        .map_err(|e| format!("create_service(get_planner_params): {e}"))?;
    let set = node
        .create_service::<SetPlannerParams::Service>("set_planner_params", QosProfile::default())
        .map_err(|e| format!("create_service(set_planner_params): {e}"))?;

    let configs: PlannerConfigurations = Rc::new(RefCell::new(PlannerConfigurationMap::new()));
    let configs_for_plans = Rc::clone(&configs);

    spawner
        .spawn_local(async move {
            let mut query = query;
            while let Some(req) = query.next().await {
                let response = query_response(&registered_interface_names());
                if let Err(e) = req.respond(response) {
                    eprintln!("responding to query_planner_interface request: {e}");
                }
            }
        })
        .map_err(|e| format!("spawning query_planner_interface task: {e}"))?;

    let configs_for_get = Rc::clone(&configs);
    spawner
        .spawn_local(async move {
            let mut get = get;
            while let Some(req) = get.next().await {
                let response = {
                    let configs = configs_for_get.borrow();
                    get_response(&configs, registered_interface_names().len(), &req.message)
                };
                if let Err(e) = req.respond(response) {
                    eprintln!("responding to get_planner_params request: {e}");
                }
            }
        })
        .map_err(|e| format!("spawning get_planner_params task: {e}"))?;

    spawner
        .spawn_local(async move {
            let mut set = set;
            while let Some(req) = set.next().await {
                {
                    let mut configs = configs.borrow_mut();
                    if !apply_set(
                        &mut configs,
                        registered_interface_names().len(),
                        &req.message,
                    ) {
                        // Upstream's three rejection paths each log and return
                        // (`:162`, `:169`, `:175`). The reply carries no field
                        // to say so, so the log is the only signal either side
                        // has -- upstream included.
                        eprintln!(
                            "set_planner_params rejected: pipeline_id={:?} keys={} values={}",
                            req.message.pipeline_id,
                            req.message.params.keys.len(),
                            req.message.params.values.len()
                        );
                    }
                }
                if let Err(e) = req.respond(SetPlannerParams::Response {}) {
                    eprintln!("responding to set_planner_params request: {e}");
                }
            }
        })
        .map_err(|e| format!("spawning set_planner_params task: {e}"))?;

    Ok(configs_for_plans)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn get_request(
        pipeline_id: &str,
        planner_config: &str,
        group: &str,
    ) -> GetPlannerParams::Request {
        GetPlannerParams::Request {
            pipeline_id: pipeline_id.to_string(),
            planner_config: planner_config.to_string(),
            group: group.to_string(),
        }
    }

    fn set_request(
        pipeline_id: &str,
        planner_config: &str,
        group: &str,
        pairs: &[(&str, &str)],
        replace: bool,
    ) -> SetPlannerParams::Request {
        SetPlannerParams::Request {
            pipeline_id: pipeline_id.to_string(),
            planner_config: planner_config.to_string(),
            group: group.to_string(),
            params: PlannerParams {
                keys: pairs.iter().map(|(k, _)| (*k).to_string()).collect(),
                values: pairs.iter().map(|(_, v)| (*v).to_string()).collect(),
                descriptions: Vec::new(),
            },
            replace,
        }
    }

    fn pairs(response: &GetPlannerParams::Response) -> Vec<(String, String)> {
        response
            .params
            .keys
            .iter()
            .cloned()
            .zip(response.params.values.iter().cloned())
            .collect()
    }

    /// The registry this node actually links, not a fixture: if
    /// `moveit-planners-sbp`'s `distributed_slice` failed to reach this
    /// binary, every description would silently vanish and
    /// `query_planner_interface` would answer an empty list that reads
    /// exactly like "no planners configured".
    #[test]
    fn the_linked_registry_is_not_empty() {
        assert!(
            !registered_interface_names().is_empty(),
            "PLANNER_MANAGERS reached this crate with no registrations"
        );
        assert!(registered_interface_names().contains(&"rrt_connect"));
    }

    /// Drives the exact function `registered_interface_names` routes the
    /// registry through, with a slice order the live registry cannot produce.
    /// Deleting the `sort_unstable` inside it fails this.
    #[test]
    fn interface_names_come_back_sorted_whatever_the_slice_order() {
        assert_eq!(
            sorted_names(vec!["stomp", "rrt_connect", "chomp"]),
            ["chomp", "rrt_connect", "stomp"]
        );
        // The client reads `planner_interfaces.front()`, so the first
        // description is the one a rename or a link-order flip would change.
        assert_eq!(
            query_response(&sorted_names(vec!["stomp", "chomp"])).planner_interfaces[0].name,
            "chomp"
        );
    }

    #[test]
    fn a_description_carries_the_name_an_empty_pipeline_id_and_no_planner_ids() {
        let response = query_response(&["rrt_connect"]);
        assert_eq!(response.planner_interfaces.len(), 1);
        let desc = &response.planner_interfaces[0];
        assert_eq!(desc.name, "rrt_connect");
        // The value advertised has to be one `get`/`set` accept back.
        assert_eq!(desc.pipeline_id, "");
        assert!(resolves_pipeline(&desc.pipeline_id));
        // Upstream's own base-class default, not a stub.
        assert!(desc.planner_ids.is_empty());
    }

    #[test]
    fn an_empty_registry_answers_an_empty_interface_list() {
        assert!(query_response(&[]).planner_interfaces.is_empty());
    }

    #[test]
    fn a_set_round_trips_through_a_get() {
        let mut configs = PlannerConfigurationMap::new();
        assert!(apply_set(
            &mut configs,
            1,
            &set_request("", "RRTConnect", "", &[("range", "0.1")], false)
        ));
        assert_eq!(
            pairs(&get_response(
                &configs,
                1,
                &get_request("", "RRTConnect", "")
            )),
            [("range".to_string(), "0.1".to_string())]
        );
    }

    #[test]
    fn a_group_set_is_keyed_as_group_bracket_config() {
        let mut configs = PlannerConfigurationMap::new();
        apply_set(
            &mut configs,
            1,
            &set_request("", "RRTConnect", "arm", &[("range", "0.2")], false),
        );
        assert!(configs.contains_key("arm[RRTConnect]"));
        assert_eq!(configs["arm[RRTConnect]"].group, "arm");
        assert_eq!(configs["arm[RRTConnect]"].name, "arm[RRTConnect]");
        // A global read must not see a group-specific write.
        assert!(
            pairs(&get_response(
                &configs,
                1,
                &get_request("", "RRTConnect", "")
            ))
            .is_empty()
        );
    }

    /// The `std::map::insert` boundary: upstream's "merge in group-specific
    /// params" does not overwrite a key the global configuration already
    /// supplied. Two keys, one shared and one group-only, separate that from
    /// "the group config is ignored entirely".
    #[test]
    fn a_group_read_keeps_the_global_value_for_a_shared_key() {
        let mut configs = PlannerConfigurationMap::new();
        apply_set(
            &mut configs,
            1,
            &set_request("", "RRTConnect", "", &[("range", "global")], false),
        );
        apply_set(
            &mut configs,
            1,
            &set_request(
                "",
                "RRTConnect",
                "arm",
                &[("range", "group"), ("goal_bias", "0.05")],
                false,
            ),
        );
        assert_eq!(
            pairs(&get_response(
                &configs,
                1,
                &get_request("", "RRTConnect", "arm")
            )),
            [
                ("goal_bias".to_string(), "0.05".to_string()),
                ("range".to_string(), "global".to_string()),
            ]
        );
    }

    #[test]
    fn replace_clears_the_existing_parameters_and_augment_keeps_them() {
        let mut configs = PlannerConfigurationMap::new();
        apply_set(
            &mut configs,
            1,
            &set_request("", "RRTConnect", "", &[("range", "0.1")], false),
        );
        apply_set(
            &mut configs,
            1,
            &set_request("", "RRTConnect", "", &[("goal_bias", "0.05")], false),
        );
        assert_eq!(
            pairs(&get_response(
                &configs,
                1,
                &get_request("", "RRTConnect", "")
            ))
            .len(),
            2
        );
        apply_set(
            &mut configs,
            1,
            &set_request("", "RRTConnect", "", &[("range", "0.3")], true),
        );
        assert_eq!(
            pairs(&get_response(
                &configs,
                1,
                &get_request("", "RRTConnect", "")
            )),
            [("range".to_string(), "0.3".to_string())]
        );
    }

    #[test]
    fn a_later_set_overwrites_a_key_the_earlier_one_wrote() {
        let mut configs = PlannerConfigurationMap::new();
        apply_set(
            &mut configs,
            1,
            &set_request("", "RRTConnect", "", &[("range", "0.1")], false),
        );
        apply_set(
            &mut configs,
            1,
            &set_request("", "RRTConnect", "", &[("range", "0.9")], false),
        );
        assert_eq!(
            pairs(&get_response(
                &configs,
                1,
                &get_request("", "RRTConnect", "")
            )),
            [("range".to_string(), "0.9".to_string())]
        );
    }

    #[test]
    fn mismatched_key_and_value_counts_write_nothing() {
        let mut configs = PlannerConfigurationMap::new();
        let mut req = set_request("", "RRTConnect", "", &[("range", "0.1")], false);
        req.params.values.clear();
        assert!(!apply_set(&mut configs, 1, &req));
        assert!(configs.is_empty());
    }

    #[test]
    fn a_nonempty_pipeline_id_resolves_to_nothing_on_both_legs() {
        let mut configs = PlannerConfigurationMap::new();
        assert!(!apply_set(
            &mut configs,
            1,
            &set_request("ompl", "RRTConnect", "", &[("range", "0.1")], false)
        ));
        assert!(configs.is_empty());

        // And a read for a named pipeline sees nothing even when the default
        // pipeline does hold that configuration.
        apply_set(
            &mut configs,
            1,
            &set_request("", "RRTConnect", "", &[("range", "0.1")], false),
        );
        assert!(
            pairs(&get_response(
                &configs,
                1,
                &get_request("ompl", "RRTConnect", "")
            ))
            .is_empty()
        );
        assert_eq!(
            pairs(&get_response(
                &configs,
                1,
                &get_request("", "RRTConnect", "")
            ))
            .len(),
            1
        );
    }

    /// Upstream's `planner_plugin_names.empty()` / null-manager arms
    /// (`:120-133`, `:169-180`): no planner surface, no configuration
    /// surface, on both legs.
    #[test]
    fn with_no_registered_interface_neither_leg_does_anything() {
        let mut configs = PlannerConfigurationMap::new();
        assert!(!apply_set(
            &mut configs,
            0,
            &set_request("", "RRTConnect", "", &[("range", "0.1")], false)
        ));
        assert!(configs.is_empty());

        apply_set(
            &mut configs,
            1,
            &set_request("", "RRTConnect", "", &[("range", "0.1")], false),
        );
        assert!(
            pairs(&get_response(
                &configs,
                0,
                &get_request("", "RRTConnect", "")
            ))
            .is_empty()
        );
    }

    #[test]
    fn an_unknown_planner_config_reads_empty_rather_than_failing() {
        let mut configs = PlannerConfigurationMap::new();
        apply_set(
            &mut configs,
            1,
            &set_request("", "RRTConnect", "", &[("range", "0.1")], false),
        );
        assert!(pairs(&get_response(&configs, 1, &get_request("", "PRM", ""))).is_empty());
    }

    /// The reply's arrays are index-paired, and upstream fills them from a
    /// `std::map`, so they arrive sorted by key. A client rebuilds a map from
    /// them by index (`move_group_interface.cpp:292-293`), so a pairing slip
    /// is silent.
    #[test]
    fn get_returns_key_sorted_index_paired_arrays() {
        let mut configs = PlannerConfigurationMap::new();
        apply_set(
            &mut configs,
            1,
            &set_request(
                "",
                "RRTConnect",
                "",
                &[("range", "0.1"), ("goal_bias", "0.05"), ("attempts", "7")],
                false,
            ),
        );
        let response = get_response(&configs, 1, &get_request("", "RRTConnect", ""));
        assert_eq!(response.params.keys, ["attempts", "goal_bias", "range"]);
        assert_eq!(response.params.values, ["7", "0.05", "0.1"]);
        assert!(response.params.descriptions.is_empty());
    }
}
