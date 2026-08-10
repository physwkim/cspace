// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// This file is the merged crate's root only. It ports nothing itself -- every
// module below carries its own upstream citation header, and its own
// copyright line where the port is not original work.

//! The workspace's motion planners, one module per former crate.
//!
//! [`sbp`] is original design work against the gap PORTING-PLAN.md D3 records
//! (no OMPL equivalent exists in the Rust ecosystem); [`chomp`], [`pilz`] and
//! [`stomp`] are ports of upstream's `moveit_planners/` packages of the same
//! names. They share no code and never call each other -- what they share is
//! the planner-side interface they are reached through,
//! `cspace_planning::planner_registry`, and the [`sbp::registry`] entry is what
//! links a `PlannerManager` implementation into a binary at all.
//!
//! `#[forbid(unsafe_code)]` sits on each module rather than on the crate,
//! because [`sbp`] cannot carry it -- see this crate's `Cargo.toml` for why
//! linkme forces `unsafe_code = "allow"` at the package level, and why that
//! exemption is confined to one module instead of covering all four.

// The one module the crate-level `unsafe_code = "allow"` exists for: its
// `registry` uses `linkme::distributed_slice`, which expands to
// `#[link_section]` statics.
pub mod sbp;

#[forbid(unsafe_code)]
pub mod chomp;

#[forbid(unsafe_code)]
pub mod pilz;

#[forbid(unsafe_code)]
pub mod stomp;
