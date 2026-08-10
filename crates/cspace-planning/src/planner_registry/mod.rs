// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file ported line-for-line. This is the D4 compile-time
// replacement for the `pluginlib` half of
//   moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp
// (`planner_plugin_loader_`/`planner_map_`, `planning_pipeline.hpp:262-263`)
// and for the `CLASS_LOADER_REGISTER_CLASS(_, planning_interface::PlannerManager)`
// macro every upstream planner plugin ends with.

//! Name-to-implementation lookup for [`PlannerManager`]s: the D4
//! compile-time stand-in for upstream's `pluginlib`-populated
//! `PlanningPipeline::planner_map_` (`planning_pipeline.hpp:263`).
//!
//! # Why a crate of its own
//!
//! Everything here could have been a module of `cspace-planning`, next to
//! the [`PlannerManager`] trait it hands out. It is not, for one measured
//! reason: [`PLANNER_MANAGERS`] needs `linkme::distributed_slice`, which
//! forces `unsafe_code = "allow"` on whatever crate hosts it, and that
//! relaxation is necessarily crate-wide. Hosting it here keeps the blanket
//! allow over one struct, one static and one function instead of over all
//! of `cspace-planning`'s planning logic (PORTING-PLAN.md §140.1; see this
//! crate's `Cargo.toml` for the full argument).
//!
//! The layering that falls out of that is deliberate and worth stating:
//! `cspace-planning` defines the vocabulary (`PlanningRequest`,
//! `PlanningResponse`, [`PlannerManager`], `PlanningContext`) and knows
//! nothing about which planners exist; planner crates depend on that
//! vocabulary and register themselves here; a *caller* that wants to go
//! from a planner-id string to something it can call depends on this crate.
//! `crate::pipeline::generate_plan` does not — it takes already
//! resolved planners, because name resolution is orthogonal to what a
//! pipeline does.

use crate::{PlannerConfigurationMap, PlannerManager};
use cspace_core::error::{Error, Result};

/// One [`PlannerManager`] implementation's compile-time registration.
///
/// Replaces upstream's `CLASS_LOADER_REGISTER_CLASS(ConcreteType,
/// planning_interface::PlannerManager)` — a pluginlib macro that lets a
/// class be looked up by name from a `.so` pluginlib never has to link
/// against at build time. Per PORTING-PLAN.md D4 that runtime, string-keyed
/// `dlopen` lookup is not ported: every planner this workspace ships is
/// linked in at compile time and simply appears in [`PLANNER_MANAGERS`], a
/// `linkme::distributed_slice` scanned once rather than resolved per plugin
/// request. Mirrors `cspace_core::kinematics::registry::SolverRegistration`
/// exactly, for the same reason.
pub struct PlannerRegistration {
    /// The name a caller scanning [`PLANNER_MANAGERS`] matches on, equal to
    /// the [`PlannerManager::name`] of what [`PlannerRegistration::construct`]
    /// builds — `"rrt_connect"` for `cspace_planners::sbp::RrtConnectManager`.
    /// `tests/registered_planners.rs`'
    /// `registration_names_match_the_managers_they_build` is what keeps the
    /// two from drifting apart.
    pub name: &'static str,
    /// Builds one instance of this registration's [`PlannerManager`], to
    /// plan under `configs`.
    ///
    /// Taking the [`PlannerConfigurationMap`] here rather than offering a
    /// setter is the whole shape of upstream's
    /// `setPlannerConfigurations` (`planning_interface.hpp:193`) on this
    /// side: pluginlib hands `move_group` an already-constructed plugin, so
    /// upstream has no choice but to configure it afterwards — and
    /// `MoveGroupQueryPlannersService::setParams` closing with
    /// `planner_interface->setPlannerConfigurations(configs)`
    /// (`query_planners_service_capability.cpp:205`) is a call any other
    /// caller can forget to make. There is no plugin loader here, so the
    /// configuration is an *argument* and a manager that plans under no
    /// configuration at all cannot be constructed through this registry
    /// (PORTING-PLAN.md §285).
    ///
    /// A manager reads whichever keys it documents and ignores the rest —
    /// upstream's own rule for this map, stated on the struct
    /// (`planning_interface.hpp:54`, "Settings with unknown keys are
    /// ignored"). An empty map is the ordinary case, not an error: it means
    /// the manager plans in whatever default configuration it documents.
    pub construct: fn(&PlannerConfigurationMap) -> Box<dyn PlannerManager>,
}

/// Every [`PlannerManager`] any crate linked into the same binary
/// registers. See [`PlannerRegistration`]'s doc comment for why this
/// replaces pluginlib rather than reproducing it, and `Cargo.toml`'s
/// `[lints.rust]` comment for why this crate — and only this crate — sets
/// `unsafe_code = "allow"` to host it.
///
/// A registering crate needs the same relaxation (the
/// `#[distributed_slice(PLANNER_MANAGERS)]` attribute expands to a
/// `#[link_section]` static on the *registration* side too, not just here);
/// `cspace_planners::sbp` already sets it for exactly this reason.
///
/// # A registering crate must be *linked*, not merely depended on
///
/// "Any crate linked into the same binary" is literal. A registration lives
/// in an object file the linker keeps only if some symbol in the binary
/// pulls it in; a crate that is a dependency but whose items are never named
/// contributes nothing here, and the failure is silent — [`resolve_planner`]
/// returns [`Error::UnknownName`] as if the planner had never been written.
/// Measured, not reasoned about: `cspace-planning`'s crate-level doctest and
/// this crate's `tests/registered_planners.rs` both failed exactly that way
/// until each grew a `use cspace_planners as _;`. Any binary that
/// expects to find a planner here needs that line (see
/// `ros/cspace-ros/src/lib.rs` for the production one).
#[linkme::distributed_slice]
pub static PLANNER_MANAGERS: [PlannerRegistration];

/// Build the [`PLANNER_MANAGERS`] entry registered under `name`, to plan
/// under `configs`.
///
/// Selection is by [`PlannerRegistration::name`], never by
/// [`PLANNER_MANAGERS`]'s iteration order. That order is `linkme`'s
/// link-section placement order, a function of the whole workspace's
/// dependency graph — any crate anywhere adding an unrelated dependency can
/// silently reorder it, which is not hypothetical here: PORTING-PLAN.md
/// §177 records adding `thiserror` to an unrelated crate flipping the
/// sibling `KINEMATICS_SOLVERS` slice's order and silently switching which
/// IK solver pilz used. A caller that picks "the first registration that
/// constructs" is not applying a selection rule; it is letting the linker
/// apply one it never wrote down.
///
/// # Errors
///
/// [`Error::UnknownName`] if no [`PLANNER_MANAGERS`] entry is registered
/// under `name`. Construction itself is infallible — unlike
/// `cspace_core::kinematics::registry::resolve_solver`, whose registrations are
/// per-`(model, group)` and can legitimately fail to build, a
/// [`PlannerManager`] is model-independent: everything that can fail about
/// a particular query surfaces from
/// [`PlannerManager::get_planning_context`] instead. `configs` cannot fail
/// either: an entry a manager does not understand is ignored, per
/// [`PlannerRegistration::construct`].
pub fn resolve_planner(
    name: &str,
    configs: &PlannerConfigurationMap,
) -> Result<Box<dyn PlannerManager>> {
    let registration = PLANNER_MANAGERS
        .iter()
        .find(|registration| registration.name == name)
        .ok_or_else(|| Error::unknown_name("planner manager", name))?;
    Ok((registration.construct)(configs))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The `Err` half of [`resolve_planner`]: an unregistered name must be
    /// [`Error::UnknownName`], not a silent fallback to whatever
    /// registration happens to sit first in the slice.
    ///
    /// This is the only test that can live in this crate's *unit* test
    /// binary. Anything asserting on a real registration needs a planner
    /// crate linked in, and a planner crate depends on this one, so the
    /// unit-test build would hold two copies of this crate — the
    /// `cfg(test)` one and the one the planner links — and `linkme` aborts
    /// at startup with `duplicate #[distributed_slice] with name
    /// "PLANNER_MANAGERS"`. `tests/registered_planners.rs` is those
    /// assertions, in an integration test, where only the non-`cfg(test)`
    /// build of this crate exists.
    #[test]
    fn an_unregistered_name_is_rejected_rather_than_defaulted() {
        // Destructured rather than `expect_err`: that would require `Debug`
        // on the `Ok` type, and `Box<dyn PlannerManager>` deliberately has
        // none -- a planner is not a value this crate can print.
        let Err(err) = resolve_planner("no_such_planner", &PlannerConfigurationMap::new()) else {
            panic!("an unregistered planner name must not resolve");
        };
        assert!(
            matches!(err, Error::UnknownName { .. }),
            "expected Error::UnknownName, got {err:?}"
        );
    }
}
