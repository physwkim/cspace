// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_common.hpp
//   moveit_core/collision_detection/src/collision_common.cpp

use std::collections::BTreeMap;
use std::fmt;
use std::sync::Arc;

use moveit_geometry::Vector3;

/// Upstream `collision_detection::BodyTypes::Type` (aliased there as `BodyType`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BodyType {
    /// A link on the robot.
    #[default]
    RobotLink,
    /// A body attached to a robot link.
    RobotAttached,
    /// A body in the environment.
    WorldObject,
}

/// One point of contact between two bodies.
///
/// Upstream `collision_detection::Contact`.
///
/// # Deviation from upstream
///
/// Upstream's `body_type_1`/`body_type_2` have no in-class initializer, so a
/// default-constructed `Contact` (as opposed to a value-initialized one)
/// technically leaves them indeterminate. A collision backend always fills in
/// every field before a `Contact` is read, so this never matters in practice;
/// [`Default`] here always produces a fully-defined value
/// ([`BodyType::RobotLink`], enum value `0`, the value zero-initialization
/// would produce) rather than reproducing an indeterminate value Rust has no
/// way to express anyway.
#[derive(Debug, Clone, Default)]
pub struct Contact {
    /// Contact position.
    pub pos: Vector3,
    /// Normal unit vector at the contact.
    pub normal: Vector3,
    /// Penetration depth between the two bodies.
    pub depth: f64,
    /// Name of the first body in the contact.
    pub body_name_1: String,
    /// Type of the first body in the contact.
    pub body_type_1: BodyType,
    /// Name of the second body in the contact.
    pub body_name_2: String,
    /// Type of the second body in the contact.
    pub body_type_2: BodyType,
    /// Distance fraction between two casted poses at which the contact
    /// occurred: `0` at the start pose, `1` at the end pose.
    pub percent_interpolation: f64,
    /// The two nearest points connecting the two bodies.
    pub nearest_points: [Vector3; 2],
}

/// Partial cost attributed to one volume of space, when collision costs are
/// computed.
///
/// Upstream `collision_detection::CostSource`.
///
/// # Deviation from upstream
///
/// Upstream's `operator<` orders a `std::set<CostSource>` most-costly-first,
/// so consumers just iterate the set. Nothing in this crate populates or
/// orders `CostSource`s — that is the collision backend's job, and every
/// backend (FCL/Bullet/parry) needs a `RobotModel` to run, so it is out of
/// scope here (see this crate's module doc). Ordering is therefore left to
/// whichever later crate actually builds a collection of these, rather than
/// carrying an `Ord` impl with no caller to exercise it.
#[derive(Debug, Clone, Copy, Default)]
pub struct CostSource {
    /// Minimum corner of the axis-aligned bounding box for this cost source.
    pub aabb_min: [f64; 3],
    /// Maximum corner of the axis-aligned bounding box for this cost source.
    pub aabb_max: [f64; 3],
    /// The partial cost: the probability of a collision existing in this
    /// volume.
    pub cost: f64,
}

impl CostSource {
    /// `getVolume`: the volume of the AABB around this cost source.
    pub fn volume(&self) -> f64 {
        (self.aabb_max[0] - self.aabb_min[0])
            * (self.aabb_max[1] - self.aabb_min[1])
            * (self.aabb_max[2] - self.aabb_min[2])
    }
}

/// Upstream `CollisionRequest`'s `is_done`,
/// `std::function<bool(const CollisionResult&)>`.
///
/// An `Arc` (not a plain closure type) for the same reason as
/// [`crate::DecideContactFn`]: a shared, cloneable, thread-safe callback,
/// matching upstream's copyable `std::function`.
pub type IsDoneFn = Arc<dyn Fn(&CollisionResult) -> bool + Send + Sync>;

/// Representation of a collision-checking request.
///
/// Upstream `collision_detection::CollisionRequest`.
///
/// # Deviation from upstream
///
/// Trait objects have no [`fmt::Debug`]/[`PartialEq`], so those are
/// implemented by hand below, printing a placeholder for
/// [`CollisionRequest::is_done`] instead of deriving.
#[derive(Clone)]
pub struct CollisionRequest {
    /// Group to check collisions for; `None` means the whole robot
    /// (descendant links included).
    ///
    /// Upstream represents "whole robot" as `group_name == ""`; [`Option`]
    /// keeps that distinct from a group literally named `""`.
    pub group_name: Option<String>,
    /// Use a padded collision environment.
    pub pad_environment_collisions: bool,
    /// Do self-collision checks with padded robot links.
    pub pad_self_collisions: bool,
    /// Compute proximity distance.
    pub distance: bool,
    /// Return detailed distance information. Only meaningful when
    /// [`CollisionRequest::distance`] is set.
    pub detailed_distance: bool,
    /// Compute a collision cost.
    pub cost: bool,
    /// Compute contacts; otherwise only a binary collision yes/no is
    /// reported.
    pub contacts: bool,
    /// Overall maximum number of contacts to compute.
    pub max_contacts: usize,
    /// Maximum number of contacts to compute per pair of bodies.
    pub max_contacts_per_pair: usize,
    /// How many top cost sources to return, when costs are computed.
    pub max_cost_sources: usize,
    /// Decides whether collision detection should stop early. `None` matches
    /// upstream's default-constructed `nullptr` `std::function`.
    pub is_done: Option<IsDoneFn>,
    /// Report information about detected collisions.
    pub verbose: bool,
}

impl fmt::Debug for CollisionRequest {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CollisionRequest")
            .field("group_name", &self.group_name)
            .field(
                "pad_environment_collisions",
                &self.pad_environment_collisions,
            )
            .field("pad_self_collisions", &self.pad_self_collisions)
            .field("distance", &self.distance)
            .field("detailed_distance", &self.detailed_distance)
            .field("cost", &self.cost)
            .field("contacts", &self.contacts)
            .field("max_contacts", &self.max_contacts)
            .field("max_contacts_per_pair", &self.max_contacts_per_pair)
            .field("max_cost_sources", &self.max_cost_sources)
            .field("is_done", &self.is_done.as_ref().map(|_| ".."))
            .field("verbose", &self.verbose)
            .finish()
    }
}

impl Default for CollisionRequest {
    fn default() -> Self {
        Self {
            group_name: None,
            pad_environment_collisions: true,
            pad_self_collisions: false,
            distance: false,
            detailed_distance: false,
            cost: false,
            contacts: false,
            max_contacts: 1,
            max_contacts_per_pair: 1,
            max_cost_sources: 1,
            is_done: None,
            verbose: false,
        }
    }
}

/// Upstream `collision_detection::DistanceRequestTypes::DistanceRequestType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DistanceRequestType {
    /// Find the global minimum.
    #[default]
    Global,
    /// Find the global minimum for each pair.
    Single,
    /// Find a limited (`max_contacts_per_body`) set of contacts for a given
    /// pair.
    Limited,
    /// Find all the contacts for a given pair.
    All,
}

/// Representation of a distance-reporting request.
///
/// Upstream `collision_detection::DistanceRequest`.
///
/// # Deviation from upstream
///
/// `enableGroup` and `active_components_only` (a `const std::set<const
/// moveit::core::LinkModel*>*`, derived from a `RobotModelConstPtr`'s
/// `JointModelGroup`) are not ported: both need a `RobotModel`, which is out
/// of scope for this crate (see the module doc). `acm` becomes a borrowed
/// reference rather than upstream's raw `const AllowedCollisionMatrix*`,
/// since this workspace forbids `unsafe_code`.
#[derive(Debug, Clone, Copy)]
pub struct DistanceRequest<'a> {
    /// Whether nearest-point information should be calculated.
    pub enable_nearest_points: bool,
    /// Whether a signed distance should be calculated in a collision.
    pub enable_signed_distance: bool,
    /// The kind of distance request.
    pub request_type: DistanceRequestType,
    /// Maximum number of contacts to store per body.
    pub max_contacts_per_body: usize,
    /// Group name, or `None` for the whole robot; see
    /// [`CollisionRequest::group_name`].
    pub group_name: Option<&'a str>,
    /// The allowed-collision matrix used to filter checks.
    pub acm: Option<&'a crate::AllowedCollisionMatrix>,
    /// Only calculate distances for objects within this threshold of each
    /// other.
    pub distance_threshold: f64,
    /// Log debug information.
    pub verbose: bool,
    /// Whether to calculate the normalized gradient vector connecting the
    /// closest points on the two objects.
    pub compute_gradient: bool,
}

impl Default for DistanceRequest<'_> {
    fn default() -> Self {
        Self {
            enable_nearest_points: false,
            enable_signed_distance: false,
            request_type: DistanceRequestType::default(),
            max_contacts_per_body: 1,
            group_name: None,
            acm: None,
            distance_threshold: f64::MAX,
            verbose: false,
            compute_gradient: false,
        }
    }
}

/// Distance information for one pair of objects.
///
/// Upstream `collision_detection::DistanceResultsData`.
#[derive(Debug, Clone)]
pub struct DistanceResultsData {
    /// Distance between the two objects. `<= 0` means they are in collision.
    pub distance: f64,
    /// The nearest points, one per object.
    pub nearest_points: [Vector3; 2],
    /// Object link names, one per object. Upstream stores `""` for an unset
    /// name; kept as `String` rather than `Option` to match that upstream
    /// `clear()` never distinguishes "unset" from "named the empty string".
    pub link_names: [String; 2],
    /// Object body types, one per object.
    pub body_types: [BodyType; 2],
    /// Normalized vector connecting the closest points, from `link_names[0]`
    /// to `link_names[1]`.
    pub normal: Vector3,
}

impl Default for DistanceResultsData {
    /// Upstream `DistanceResultsData::clear()`, which the constructor also
    /// calls.
    fn default() -> Self {
        Self {
            distance: f64::MAX,
            nearest_points: [Vector3::zeros(); 2],
            link_names: [String::new(), String::new()],
            body_types: [BodyType::WorldObject; 2],
            normal: Vector3::zeros(),
        }
    }
}

/// Upstream `collision_detection::DistanceMap`.
pub type DistanceMap = BTreeMap<(String, String), Vec<DistanceResultsData>>;

/// Result of a distance request.
///
/// Upstream `collision_detection::DistanceResult`.
#[derive(Debug, Clone, Default)]
pub struct DistanceResult {
    /// Whether the two objects were found to be in collision.
    pub collision: bool,
    /// Result data for the two objects with the minimum distance.
    pub minimum_distance: DistanceResultsData,
    /// Distance data for each link in the request's active components.
    pub distances: DistanceMap,
}

/// Result of a collision-checking request.
///
/// Upstream `collision_detection::CollisionResult`.
///
/// # Deviation from upstream
///
/// `print()` is not ported: it logs through `rclcpp`'s throttled logger
/// (`RCLCPP_WARN_STREAM_THROTTLE`), unavailable in a ROS-independent core
/// crate (PORTING-PLAN.md D1) and with no behavior to preserve once the
/// logging call itself is gone.
#[derive(Debug, Clone)]
pub struct CollisionResult {
    /// Whether a collision was found.
    pub collision: bool,
    /// Closest distance between two bodies.
    pub distance: f64,
    /// Distance data, when [`CollisionRequest::distance`] was set.
    pub distance_result: DistanceResult,
    /// Number of contacts returned.
    pub contact_count: usize,
    /// Pairs of body ids in contact, plus their contact details.
    pub contacts: BTreeMap<(String, String), Vec<Contact>>,
    /// Individual cost sources, when [`CollisionRequest::cost`] was set.
    pub cost_sources: Vec<CostSource>,
}

impl Default for CollisionResult {
    fn default() -> Self {
        Self {
            collision: false,
            distance: f64::MAX,
            distance_result: DistanceResult::default(),
            contact_count: 0,
            contacts: BTreeMap::new(),
            cost_sources: Vec::new(),
        }
    }
}

impl CollisionResult {
    /// Reset to the same state a fresh [`CollisionResult`] starts in.
    ///
    /// Upstream `CollisionResult::clear()`.
    pub fn clear(&mut self) {
        *self = Self::default();
    }
}
