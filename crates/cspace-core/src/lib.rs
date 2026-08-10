// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// This file is the merged crate's root only. It ports nothing itself -- every
// module below carries its own upstream citation header.

//! Configuration-space robot core.
//!
//! Twelve former crates, one module each. The split into separate packages was
//! a build-time boundary, not a design one: nothing outside this workspace ever
//! depended on a subset, and the dependency graph between them was a DAG with
//! no cycles to break. Merging them changes no visibility that mattered --
//! every `pub(crate)` widens to this crate, and none of it was load-bearing --
//! and it cuts what `crates.io` has to carry from twelve packages to one.
//!
//! Module order below is the dependency order: [`error`] depends on nothing,
//! and `test_support` (feature-gated, so not linkable from a default doc
//! build) sits on the model layer.
//!
//! `#[forbid(unsafe_code)]` sits on each module rather than on the crate,
//! because [`kinematics`] cannot carry it -- see this crate's `Cargo.toml` for
//! why linkme forces `unsafe_code = "allow"` at the package level, and why
//! that exemption is confined here instead of covering all twelve.

#[forbid(unsafe_code)]
pub mod error;

#[forbid(unsafe_code)]
pub mod geometry;

#[forbid(unsafe_code)]
pub mod octomap;

#[forbid(unsafe_code)]
pub mod srdf;

#[forbid(unsafe_code)]
pub mod model;

#[forbid(unsafe_code)]
pub mod state;

// The one module the crate-level `unsafe_code = "allow"` exists for: its
// `registry` uses `linkme::distributed_slice`, which expands to
// `#[link_section]` statics.
pub mod kinematics;

#[forbid(unsafe_code)]
pub mod sampling;

#[forbid(unsafe_code)]
pub mod trajectory;

#[forbid(unsafe_code)]
pub mod smoothing;

#[forbid(unsafe_code)]
pub mod metrics;

// Fixture loaders. `cfg(test)` covers this crate's own unit tests; the
// `test-support` feature is what integration tests and downstream crates
// enable, since they link the library built without `cfg(test)`.
#[cfg(any(test, feature = "test-support"))]
#[forbid(unsafe_code)]
pub mod test_support;
