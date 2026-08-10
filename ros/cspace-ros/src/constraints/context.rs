// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Shared `&RobotModel`+[`Transforms`] context for `PositionConstraint`/
//! `OrientationConstraint`/`VisibilityConstraint` conversions (§5/§6/§7).
//!
//! `cspace_planning::constraints`' own constructors decide fixed-vs-mobile per
//! `Transforms::can_transform(frame_id)`: true iff `frame_id` is already a
//! key in the `Transforms` map (confirmed from `transforms.rs`'s own doc
//! comment -- a plain map-membership check, not "is this a robot link").
//! `RobotModel` alone carries no poses to seed that map with (poses are a
//! `RobotState`/`Posed` concept, not a `RobotModel` one), so the only
//! context a bare-message `TryFrom` can honestly build is the minimal one
//! below: the model's own root frame registered as the single fixed entry.
//!
//! This means every `frame_id` on the wire ends up one of exactly three
//! ways once passed through the affected constructors: (1) `frame_id ==
//! model.model_frame()` resolves **fixed** (registered at
//! `Transforms::new`); (2) any other name that is a real link
//! (`model.has_link_model`) resolves **mobile** (re-decided against the
//! robot's current pose every time `decide()` runs, matching what upstream
//! does for any frame that isn't the fixed world frame); (3) anything else
//! is `Error::UnknownName { kind: "frame", .. }`. No wire message carries enough
//! information to register additional *fixed* frames (that needs a live
//! `RobotState`/TF tree, not just a message) -- this is a deliberate,
//! documented scope boundary for a message-only conversion, not an
//! oversight; see `doc/message-mapping.md` §5/§6/§7.
use cspace_core::error::Error;
use cspace_core::geometry::Transforms;
use cspace_core::model::RobotModel;

pub(crate) fn minimal_transforms(model: &RobotModel) -> Result<Transforms, Error> {
    Transforms::new(model.model_frame())
}
