// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The moveit-rs side of the differential comparison.
//!
//! Every function here is a stub until the crate it needs lands. Phase 1 wires
//! [`model_info`] to `moveit-model`; Phase 2 wires [`fk`] to `moveit-state`.
//! Until then a run reports every case as failed, which is exactly the Phase 0
//! completion criterion in `PORTING-PLAN.md` — it proves the harness executes
//! end to end before there is anything to compare.

use std::collections::BTreeMap;

use crate::protocol::{FkResult, ModelInfo};

/// Phase 1: `moveit-model` is not implemented yet.
pub fn model_info() -> Result<ModelInfo, String> {
    Err("moveit-model not implemented (PORTING-PLAN.md Phase 1)".to_owned())
}

/// Phase 2: `moveit-state` is not implemented yet.
pub fn fk(_model: &ModelInfo, _joint_values: &BTreeMap<String, f64>) -> Result<FkResult, String> {
    Err("moveit-state not implemented (PORTING-PLAN.md Phase 2)".to_owned())
}
