// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The half of `cspace_planning::planner_registry`'s coverage that needs a real
//! registration in the slice.
//!
//! An integration test rather than a `#[cfg(test)] mod tests`, for a reason
//! that is structural and not a preference: a planner crate depends on
//! `cspace_planning::planner_registry`, so a *unit*-test build of this crate links two
//! copies of it — the `cfg(test)` one under test and the plain one the
//! planner was compiled against — and `linkme` refuses that at startup with
//! `duplicate #[distributed_slice] with name "PLANNER_MANAGERS"` (measured;
//! every test here failed that way before the split). An integration test
//! links only the plain build, so there is exactly one slice.

// Linked for its registration alone: `RrtConnectManager` registers itself
// into `PLANNER_MANAGERS` through a `linkme::distributed_slice` static and
// nothing below names a `cspace_planners::sbp` item, so without this line the
// linker drops the object file the registration sits in and every assertion
// here sees an empty slice.
use cspace_planners as _;

use cspace_core::error::Error;
use cspace_planning::PlannerConfigurationMap;
use cspace_planning::planner_registry::{PLANNER_MANAGERS, resolve_planner};

/// [`PLANNER_MANAGERS`]' membership must not depend on where `linkme`
/// happened to place each registration in the link section
/// (PORTING-PLAN.md §177) — checked as a set, never indexed or compared as
/// an ordered sequence, so this test cannot itself become order-dependent.
#[test]
fn every_expected_registration_exists_regardless_of_slice_order() {
    let names: std::collections::HashSet<&str> = PLANNER_MANAGERS.iter().map(|r| r.name).collect();
    assert!(
        names.contains("rrt_connect"),
        "missing registration: rrt_connect"
    );
}

/// The registry's one cross-crate invariant: a registration's key and the
/// `PlannerManager::name` of what it builds are the same string.
/// [`resolve_planner`] returns a boxed manager and callers report failures
/// through that name (`cspace_planning::pipeline::PipelineError::Planner`),
/// so a registration whose key disagreed with its manager would make an
/// error name a planner the caller never asked for.
#[test]
fn registration_names_match_the_managers_they_build() {
    for registration in PLANNER_MANAGERS {
        let manager = (registration.construct)(&PlannerConfigurationMap::new());
        assert_eq!(
            registration.name,
            manager.name(),
            "registration key and PlannerManager::name must agree"
        );
    }
}

/// The `Ok` half of [`resolve_planner`], on the one registration this
/// workspace has.
#[test]
fn a_registered_name_resolves_to_that_manager() {
    let manager = resolve_planner("rrt_connect", &PlannerConfigurationMap::new())
        .expect("rrt_connect is registered");
    assert_eq!(manager.name(), "rrt_connect");
}

/// The `Err` half, re-asserted here rather than only in the unit test: with
/// a registration actually present, "unregistered name is rejected" stops
/// being trivially true. The unit-test copy runs against an empty slice,
/// where any lookup misses; this one runs against a populated slice, which
/// is where a fallback-to-first-entry bug would show.
#[test]
fn an_unregistered_name_is_rejected_even_with_registrations_present() {
    assert!(
        !PLANNER_MANAGERS.is_empty(),
        "this test is only meaningful against a populated slice"
    );
    let Err(err) = resolve_planner("no_such_planner", &PlannerConfigurationMap::new()) else {
        panic!("an unregistered planner name must not resolve");
    };
    assert!(
        matches!(err, Error::UnknownName { .. }),
        "expected Error::UnknownName, got {err:?}"
    );
}
