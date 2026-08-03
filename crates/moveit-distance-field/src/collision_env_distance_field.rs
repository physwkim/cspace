// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_distance_field/include/moveit/collision_distance_field/collision_env_distance_field.hpp
//   moveit_core/collision_distance_field/src/collision_env_distance_field.cpp

//! The construction-only slice of `CollisionEnvDistanceField`:
//! [`add_link_body_decompositions`] (upstream's two `addLinkBodyDecompositions`
//! overloads), which builds one [`BodyDecomposition`] per robot link with
//! collision geometry, unposed, ready for a `RobotState` to pose later; and
//! [`generate_distance_field_cache_entry`] (upstream
//! `generateDistanceFieldCacheEntry`), which builds a
//! [`crate::DistanceFieldCacheEntry`] for one group.
//!
//! # Scope: what unblocked this round, and what is still blocked
//!
//! A previous round of this file's own doc comment recorded
//! `DistanceFieldCacheEntry::link_names_` (upstream
//! `getUpdatedLinkModelNames`) as a blocking dependency gap in
//! `moveit-model`. That gap is closed --
//! [`moveit_model::JointModelGroup::updated_link_names`]/
//! [`moveit_model::JointModelGroup::updated_link_with_geometry_names`] now
//! exist, oracle-verified -- which unblocks
//! [`generate_distance_field_cache_entry`] and the
//! [`crate::DistanceFieldCacheEntry`] struct itself (in
//! `collision_common_distance_field.rs`; see its module doc for the type).
//!
//! Still blocked, and why:
//!
//! - **`generateCollisionCheckingStructures`/`getDistanceFieldCacheEntry`/
//!   `getGroupStateRepresentation`/`compareCacheEntryToState`/
//!   `compareCacheEntryToAllowedCollisionMatrix`/
//!   `updateGroupStateRepresentationState`.** Every one of these reads or
//!   writes `CollisionEnvDistanceField`'s own `distance_field_cache_entry_`
//!   cache member, or consumes/produces a `GroupStateRepresentation`
//!   (deferred; see `collision_common_distance_field.rs`'s module doc for
//!   why). Porting any of them needs either the not-yet-ported
//!   `CollisionEnvDistanceField` type itself or the not-yet-ported
//!   `GroupStateRepresentation` construction path
//!   (`getGroupStateRepresentation` poses every group link's sphere
//!   decomposition at its current global transform -- real work, not a
//!   trivial wrapper). Both are out of this round's scope.
//! - **`getPosedLinkBodySphereDecomposition`/`getPosedLinkBodyPointDecomposition`.**
//!   Trivial one-line wrappers, but every upstream caller of either is
//!   itself inside the still-blocked `getGroupStateRepresentation`; porting
//!   a wrapper with no real caller to exercise it stays deferred alongside
//!   it rather than added speculatively.
//! - **`AttachedBody`-dependent methods** (`getAttachedBodySphereDecomposition`/
//!   `getAttachedBodyPointDecomposition`, in `collision_common_distance_field.cpp`).
//!   `AttachedBody` does not exist anywhere in this workspace this round
//!   either (p1-fixtures owns it).
//!
//! The rest of `CollisionEnvDistanceField` -- `checkSelfCollision`,
//! `checkCollision`, `checkRobotCollision`, the `distance*` methods,
//! `createCollisionModelMarker` (draws `visualization_msgs::msg::MarkerArray`,
//! out of scope under PORTING-PLAN.md D1 same as
//! `getBodySphereVisualizationMarkers`), `getEnvironmentCollisions`/
//! `getEnvironmentProximityGradients`, `updateDistanceObject`,
//! `generateDistanceFieldCacheEntryWorld`, `notifyObjectChange`, and the
//! `CollisionEnvDistanceField` type itself (a `CollisionEnv` implementor
//! wrapping a `World` observer and a `planning_scene::PlanningScene` this
//! workspace does not have yet either) -- remains out of scope regardless
//! ("do not try to land all of it in one round") and is not addressed here.
//!
//! Upstream's own `test_collision_distance_field.cpp` -- read in full to
//! look for a ground truth for this round's new function too -- does not
//! help narrow this further: every one of its `TEST_F` cases calls
//! `checkSelfCollision` or `checkRobotCollision`, none exercise
//! `addLinkBodyDecompositions` or `DistanceFieldCacheEntry` construction in
//! isolation. The oracle op `distance_field_cache_entry` drives
//! `generateDistanceFieldCacheEntry` indirectly instead, through the same
//! public `checkSelfCollision` -> `getLastDistanceFieldEntry()` path those
//! upstream tests use (see the oracle's own doc comment on that op for
//! why); the tests below are oracle-driven only, matching this crate's
//! practice for the other two files in this sub-package
//! (`collision_distance_field_types`, `collision_common_distance_field`)
//! where upstream's own suite does not reach the ported slice directly.
//!
//! # `addLinkBodyDecompositions`'s two overloads, as one function
//!
//! Upstream declares two independent, near-duplicate overloads:
//! `addLinkBodyDecompositions(resolution)` and
//! `addLinkBodyDecompositions(resolution, link_spheres)`, the second
//! additionally calling `BodyDecomposition::replaceCollisionSpheres` per
//! link found in `link_spheres`. Rust has no overloading; this port
//! collapses them into [`add_link_body_decompositions`] with
//! `link_spheres_override: Option<&HashMap<String, Vec<CollisionSphere>>>`
//! -- `None` behaves exactly like the first overload, `Some` exactly like
//! the second. This changes nothing observable, matching how this crate
//! already collapsed `BodyDecomposition`'s own two constructors behind one
//! `padding` parameter.
//!
//! `getLinkModelsWithCollisionGeometry()` (the robot-wide link set both
//! overloads iterate) is not a `RobotModel` method in this workspace either
//! -- `moveit-collision`'s `LinkPaddingScale` documents the same absence and
//! takes that link list as a caller-supplied argument for the same reason
//! (see its own deviation note). Unlike `getUpdatedLinkModelNames`, this is
//! a one-line filter over already-fully-public data
//! (`RobotModel::link_models()` where `!LinkModel::shapes().is_empty()`,
//! confirmed against upstream's own construction-time filter in
//! `robot_model.cpp`: `if (!link->getShapes().empty()) { ... }`), not a
//! nontrivial graph traversal another crate's own doc comment claims
//! ownership of -- so this port computes it locally rather than reporting
//! it as a blocker.
//!
//! That local filter cannot reach byte-exact parity with the live oracle on
//! `pr2.urdf`, and this is expected rather than a bug: `moveit-model`
//! deliberately never loads `<mesh>` collision geometry (its `LinkModel`
//! doc comment, deviation 4), while the oracle links against the real mesh
//! files and so reports every mesh-only-collision link (`base_link`, the
//! caster rotation links, `torso_lift_link`, every arm link, ...) as having
//! collision geometry. The parity test below checks the weaker, still
//! load-bearing property this port can actually satisfy: the computed set
//! equals the oracle's set minus exactly the links `moveit-model` recorded
//! a `Diagnostic::UnsupportedLinkGeometry { kind: "mesh", .. }` for -- i.e.
//! every disagreement is accounted for, none is silent.

use std::collections::{HashMap, HashSet};
use std::sync::Arc;

use moveit_collision::{AllowedCollisionMatrix, AllowedCollisionType, LinkPaddingScale};
use moveit_error::Result;
use moveit_geometry::Isometry3;
use moveit_model::RobotModel;
use moveit_state::Posed;
use nalgebra::Vector3;

use crate::collision_common_distance_field::DistanceFieldCacheEntry;
use crate::collision_distance_field_types::{
    BodyDecomposition, CollisionSphere, PosedBodyPointDecomposition,
};
use crate::{DistanceField, GridGeometry, PropagationDistanceField};

/// Upstream's `link_body_decomposition_vector_` paired with
/// `link_body_decomposition_index_map_`: every link's [`BodyDecomposition`],
/// in `RobotModel::link_models()` order, and a name-to-index lookup into it.
type LinkBodyDecompositions = (Vec<Arc<BodyDecomposition>>, HashMap<String, usize>);

/// Upstream's two `CollisionEnvDistanceField::addLinkBodyDecompositions`
/// overloads (see this module's doc comment for why they are one function
/// here). Builds one [`BodyDecomposition`] per link in `robot_model` that
/// has collision geometry, from *all* of that link's shapes together (not
/// per-shape, unlike [`crate::get_body_decomposition_cache_entry`]), padded
/// per [`LinkPaddingScale::link_padding`] (`0.0` for an untracked link,
/// matching upstream's `getLinkPadding` default) rather than
/// [`BodyDecomposition::DEFAULT_PADDING`].
///
/// Returns the decompositions in the same order as `robot_model.link_models()`
/// (matching upstream's `link_body_decomposition_vector_`, itself built in
/// `RobotModel`'s own construction-time link order -- see this module's doc
/// comment) alongside a name-to-index map (upstream's
/// `link_body_decomposition_index_map_`).
///
/// # Errors
///
/// See [`BodyDecomposition::from_shapes`].
pub fn add_link_body_decompositions(
    robot_model: &RobotModel,
    resolution: f64,
    link_padding: &LinkPaddingScale,
    link_spheres_override: Option<&HashMap<String, Vec<CollisionSphere>>>,
) -> Result<LinkBodyDecompositions> {
    let mut link_body_decomposition_vector = Vec::new();
    let mut link_body_decomposition_index_map = HashMap::new();

    for link_model in robot_model.link_models() {
        if link_model.shapes().is_empty() {
            continue;
        }

        let shapes: Vec<_> = link_model
            .shapes()
            .iter()
            .map(|link_shape| link_shape.shape.clone())
            .collect();
        let poses: Vec<_> = link_model
            .shapes()
            .iter()
            .map(|link_shape| link_shape.origin_transform)
            .collect();
        let mut decomposition = BodyDecomposition::from_shapes(
            &shapes,
            &poses,
            resolution,
            link_padding.link_padding(link_model.name()),
        )?;

        if let Some(overrides) = link_spheres_override {
            if let Some(spheres) = overrides.get(link_model.name()) {
                decomposition.replace_collision_spheres(spheres.clone(), Isometry3::identity());
            }
        }

        link_body_decomposition_vector.push(Arc::new(decomposition));
        link_body_decomposition_index_map.insert(
            link_model.name().to_string(),
            link_body_decomposition_vector.len() - 1,
        );
    }

    Ok((
        link_body_decomposition_vector,
        link_body_decomposition_index_map,
    ))
}

/// The distance-field-construction parameters
/// [`generate_distance_field_cache_entry`] needs only when its caller wants
/// a real [`PropagationDistanceField`] built -- upstream's
/// `generate_distance_field` `bool` plus the `size_`/`origin_`/`resolution_`/
/// `max_propogation_distance_`/`use_signed_distance_field_` members
/// `CollisionEnvDistanceField` would otherwise already carry from its own
/// construction. Bundled into one struct (matching how
/// [`PropagationDistanceField::new`] itself already bundles `size`/`origin`/
/// `resolution` into [`GridGeometry`]) both to stay under this crate's
/// `clippy::too_many_arguments` budget and because these five values are
/// only ever meaningful together. `None` in
/// [`generate_distance_field_cache_entry`]'s own `generate_distance_field`
/// parameter plays the role of upstream's `generate_distance_field = false`.
pub struct DistanceFieldConfig {
    /// Corner-origin grid geometry, matching [`PropagationDistanceField::new`].
    /// Upstream performs a center-to-corner shift (`origin_.x() - 0.5 *
    /// size_.x()`, and the same for `y`/`z`) inline at this exact call site,
    /// sourced from `CollisionEnvDistanceField::origin_`/`size_` (which are
    /// center-origin). Since that struct is not ported (see this module's
    /// doc comment), the shift becomes this field's caller's responsibility
    /// instead of this function's.
    pub geometry: GridGeometry,
    /// `max_propogation_distance_`.
    pub max_propagation_distance: f64,
    /// `use_signed_distance_field_`.
    pub use_signed_distance_field: bool,
}

/// Upstream `CollisionEnvDistanceField::generateDistanceFieldCacheEntry`.
/// Builds a [`DistanceFieldCacheEntry`] for `group_name`: which of the
/// group's updated links have collision geometry, which pairs are exempt
/// from self/intra-group collision checking per `acm`, the joint-variable
/// cache key (`state_check_indices`/`state_values`), and -- when
/// `generate_distance_field` is `Some` -- a real distance field of every
/// *other* link's collision geometry, posed at `state`'s current transforms.
///
/// Upstream reads `robot_model_`/`resolution_`/`link_body_decomposition_index_map_`/
/// `in_group_update_map_` from `CollisionEnvDistanceField`'s own
/// construction-time state; since that type is not ported (see this
/// module's doc comment), every one of those becomes an explicit parameter
/// here instead: `state.model()` for `robot_model_`, `link_body_decompositions`
/// for `link_body_decomposition_index_map_` (see [`add_link_body_decompositions`]),
/// and `state.model().joint_model_group(group_name)?.updated_link_with_geometry_names()`
/// computed inline for `in_group_update_map_.find(group_name)->second` (the
/// two are the exact same set -- `in_group_update_map_` is built once per
/// group, at `CollisionEnvDistanceField` construction, from precisely that
/// query; see `collision_env_distance_field.cpp:141-155`).
///
/// # Deviations from upstream
///
/// - **`link_state_indices` computed directly, not searched.** Upstream
///   searches `state.getJointModelGroup(group_name)->getUpdatedLinkModels()`
///   for each `link_names_[i]` and stores the matching position. Both
///   `getUpdatedLinkModels()` and `getUpdatedLinkModelNames()` are built
///   from the *same* sorted `updated_link_model_vector_`
///   (`joint_model_group.cpp:261-278`), so that search always finds
///   `link_name` at position `i` itself -- there is no input on which the
///   search and a direct `i` could disagree. This port computes
///   `link_state_indices[i] = i` directly instead of re-deriving an
///   invariant result via a linear search.
/// - **The "already has a field" skip is not ported.** Upstream's
///   `generate_distance_field` branch checks `if (dfce->distance_field_) {
///   skip } else { build }`, but `dfce` is a freshly-`make_shared`d struct a
///   few lines above with `distance_field_` still null -- that check can
///   never take the "skip" branch from within this function. This port
///   builds the field unconditionally when `generate_distance_field` is
///   `Some`, matching the only behavior upstream's own dead branch allows.
/// - **The "no link state found" skip is not ported**, for the same reason:
///   it guards against `getUpdatedLinkModels()` not containing a name from
///   `link_names_`, which -- per the same-sorted-vector fact above -- cannot
///   happen.
///
/// # Errors
///
/// [`moveit_error::Error::UnknownName`] if `group_name` does not name a
/// group in `state`'s model. See [`PropagationDistanceField::new`] for
/// errors from building the distance field.
pub fn generate_distance_field_cache_entry<'m>(
    group_name: &str,
    state: &Posed<'_, 'm>,
    acm: Option<&AllowedCollisionMatrix>,
    link_body_decompositions: &LinkBodyDecompositions,
    generate_distance_field: Option<DistanceFieldConfig>,
) -> Result<DistanceFieldCacheEntry<'m>> {
    let model = state.model();
    let group = model.joint_model_group(group_name)?;
    let (link_body_decomposition_vector, link_body_decomposition_index_map) =
        link_body_decompositions;

    let link_indices = group.updated_link_indices();
    let link_names: Vec<String> = group.updated_link_names().to_vec();
    let total = link_names.len(); // + attached_body_names.len(), always 0 here.

    let mut link_has_geometry = Vec::with_capacity(link_names.len());
    let mut link_body_indices = Vec::with_capacity(link_names.len());
    let mut link_state_indices = Vec::with_capacity(link_names.len());
    let mut self_collision_enabled = vec![true; total];
    let mut intra_group_collision_enabled = vec![Vec::new(); total];

    for (i, (&link_index, link_name)) in link_indices.iter().zip(link_names.iter()).enumerate() {
        let link = model.link_model_at(link_index);
        // See this function's "Deviations from upstream": always `i` itself.
        link_state_indices.push(i);

        if !link.shapes().is_empty() {
            link_has_geometry.push(true);
            let body_index = *link_body_decomposition_index_map
                .get(link_name)
                .expect("an updated link with geometry always has a body decomposition");
            link_body_indices.push(body_index);

            let mut row = vec![true; total];
            if let Some(acm) = acm {
                if is_always_allowed(acm, link_name, link_name) {
                    self_collision_enabled[i] = false;
                }
                for (j, other) in link_names.iter().enumerate().skip(i + 1) {
                    if other == link_name {
                        row[j] = false;
                        continue;
                    }
                    if is_always_allowed(acm, link_name, other) {
                        row[j] = false;
                    }
                }
                // No attached bodies exist in this workspace (see
                // `DistanceFieldCacheEntry`'s "Deviations from upstream"),
                // so upstream's per-attached-body loop here is a no-op.
            }
            intra_group_collision_enabled[i] = row;
        } else {
            link_has_geometry.push(false);
            link_body_indices.push(0);
            self_collision_enabled[i] = false;
            intra_group_collision_enabled[i] = vec![false; total];
        }
    }
    // Upstream's second loop, over `attached_body_names_`, is a no-op here
    // for the same reason: this port never populates that vector.

    let active_variable_names: HashSet<&str> = group
        .active_joint_indices()
        .iter()
        .flat_map(|&joint_index| model.joint_model_at(joint_index).variable_names())
        .map(String::as_str)
        .collect();

    let mut state_values = Vec::with_capacity(model.variable_names().len());
    let mut state_check_indices = Vec::new();
    for name in model.variable_names() {
        state_values.push(state.variable_position(name)?);
        if !active_variable_names.contains(name.as_str()) {
            state_check_indices.push(state_values.len() - 1);
        }
    }

    let distance_field = generate_distance_field
        .map(|config| {
            build_non_group_distance_field(
                model,
                state,
                link_names.iter().map(String::as_str),
                link_body_decomposition_vector,
                link_body_decomposition_index_map,
                config,
            )
        })
        .transpose()?;

    Ok(DistanceFieldCacheEntry {
        group_name: group_name.to_string(),
        state: (**state).clone(),
        state_check_indices,
        state_values,
        acm: acm.cloned().unwrap_or_default(),
        distance_field,
        link_names,
        link_has_geometry,
        link_body_indices,
        link_state_indices,
        attached_body_names: Vec::new(),
        attached_body_link_state_indices: Vec::new(),
        self_collision_enabled,
        intra_group_collision_enabled,
    })
}

/// `acm.getEntry(a, b, type) && type == AllowedCollision::ALWAYS`.
fn is_always_allowed(acm: &AllowedCollisionMatrix, a: &str, b: &str) -> bool {
    matches!(
        acm.allowed_collision(a, b).map(|entry| entry.kind()),
        Some(AllowedCollisionType::Always)
    )
}

/// The `generate_distance_field: true` branch of upstream
/// `generateDistanceFieldCacheEntry`: a fresh [`PropagationDistanceField`]
/// seeded with every collision-bearing link's points *outside* the group
/// (`in_group_names`), posed at `state`'s current global link transforms.
/// Attached-body points are always skipped -- this workspace has no
/// `AttachedBody` to enumerate (see [`DistanceFieldCacheEntry`]'s
/// "Deviations from upstream").
fn build_non_group_distance_field<'a>(
    robot_model: &RobotModel,
    state: &Posed<'_, '_>,
    in_group_names: impl Iterator<Item = &'a str>,
    link_body_decomposition_vector: &[Arc<BodyDecomposition>],
    link_body_decomposition_index_map: &HashMap<String, usize>,
    config: DistanceFieldConfig,
) -> Result<PropagationDistanceField> {
    let in_group: HashSet<&str> = in_group_names.collect();

    let mut all_points: Vec<Vector3<f64>> = Vec::new();
    for link in robot_model.link_models() {
        if link.shapes().is_empty() || in_group.contains(link.name()) {
            continue;
        }
        let body_index = link_body_decomposition_index_map[link.name()];
        let pose = state.global_link_transform_at(link.link_index());
        let posed = PosedBodyPointDecomposition::with_pose(
            Arc::clone(&link_body_decomposition_vector[body_index]),
            pose,
        );
        all_points.extend_from_slice(posed.collision_points());
        // Attached bodies on this link: always none, see this function's
        // doc comment.
    }

    let mut field = PropagationDistanceField::new(
        config.geometry,
        config.max_propagation_distance,
        config.use_signed_distance_field,
    )?;
    field.add_points_to_field(&all_points);
    Ok(field)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pr2_model() -> RobotModel {
        let urdf_path = format!("{}/tests/fixtures/pr2.urdf", env!("CARGO_MANIFEST_DIR"));
        let srdf_path = format!("{}/tests/fixtures/pr2.srdf", env!("CARGO_MANIFEST_DIR"));
        let urdf_xml =
            std::fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
        let urdf = urdf_rs::read_file(&urdf_path).expect("pr2.urdf must parse");
        let srdf = moveit_srdf::SrdfModel::parse_file(&srdf_path).expect("pr2.srdf must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf).expect("pr2 model must build")
    }

    #[test]
    fn only_links_with_shapes_get_a_decomposition() {
        let model = pr2_model();
        let padding = LinkPaddingScale::new();
        let (vector, index_map) =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();

        assert_eq!(vector.len(), index_map.len());
        for link_model in model.link_models() {
            assert_eq!(
                index_map.contains_key(link_model.name()),
                !link_model.shapes().is_empty(),
                "link {} disagreed on whether it has a decomposition",
                link_model.name()
            );
        }
    }

    #[test]
    fn link_spheres_override_replaces_the_computed_spheres_for_that_link_only() {
        let model = pr2_model();
        let padding = LinkPaddingScale::new();
        let (baseline, index_map) =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();
        let target_link = model
            .link_models()
            .iter()
            .find(|l| !l.shapes().is_empty())
            .unwrap()
            .name()
            .to_string();

        let overridden_spheres = vec![CollisionSphere::new(
            nalgebra::Vector3::new(1.0, 2.0, 3.0),
            9.9,
        )];
        let mut overrides = HashMap::new();
        overrides.insert(target_link.clone(), overridden_spheres.clone());

        let (with_override, _) =
            add_link_body_decompositions(&model, 0.05, &padding, Some(&overrides)).unwrap();

        let target_index = index_map[&target_link];
        assert_eq!(
            with_override[target_index].collision_spheres(),
            overridden_spheres.as_slice()
        );
        // Every other link's decomposition must be untouched.
        for (name, &index) in &index_map {
            if name == &target_link {
                continue;
            }
            assert_eq!(
                with_override[index].collision_spheres(),
                baseline[index].collision_spheres(),
                "link {name} was not overridden but its spheres changed anyway"
            );
        }
    }
}
