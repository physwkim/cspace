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
/// Upstream orders a `std::set<CostSource>` most-costly-first via
/// `operator<`; `collision_tools.cpp`'s set-based utilities (this crate's
/// private `tools` module) rely on that ordering (and on set dedup) directly, so
/// [`Ord`]/[`Eq`] are implemented below to match `operator<` exactly,
/// including its tie-break chain. Upstream compares `double`s with a bare
/// `<`/`>`, which is silently blind to `NaN` (every comparison involving
/// `NaN` is `false`, so a `NaN` cost or AABB bound sorts as neither greater
/// nor less than anything — `std::set` would treat it as size-1-equivalent
/// to whatever it's compared against first, an upstream latent bug). This
/// port uses [`f64::total_cmp`] instead, which never panics and gives a
/// total order for every bit pattern including `NaN` — well-formed geometry
/// never produces `NaN` here, so this only changes behavior in the already-
/// buggy case, never in the documented one.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct CostSource {
    /// Minimum corner of the axis-aligned bounding box for this cost source.
    pub aabb_min: [f64; 3],
    /// Maximum corner of the axis-aligned bounding box for this cost source.
    pub aabb_max: [f64; 3],
    /// The partial cost: the probability of a collision existing in this
    /// volume.
    pub cost: f64,
}

/// See the [`CostSource`] deviation note: this asserts reflexivity holds,
/// which is true for every value real geometry produces (no `NaN`).
impl Eq for CostSource {}

impl PartialOrd for CostSource {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for CostSource {
    /// `operator<`, negated to match `std::set`'s most-costly-first
    /// iteration order: this returns `Less` exactly when upstream's
    /// `*this < other` is `true`.
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        use std::cmp::Ordering;

        let c1 = self.cost * self.volume();
        let c2 = other.cost * other.volume();
        match c2.total_cmp(&c1) {
            Ordering::Equal => match other.cost.total_cmp(&self.cost) {
                Ordering::Equal => total_cmp_aabb(&self.aabb_min, &other.aabb_min),
                ord => ord,
            },
            ord => ord,
        }
    }
}

/// `std::array<double, 3>::operator<`: plain lexicographic order, using
/// [`f64::total_cmp`] per element for the same reason as [`CostSource::cmp`].
fn total_cmp_aabb(a: &[f64; 3], b: &[f64; 3]) -> std::cmp::Ordering {
    a.iter()
        .zip(b)
        .map(|(x, y)| x.total_cmp(y))
        .find(|ord| *ord != std::cmp::Ordering::Equal)
        .unwrap_or(std::cmp::Ordering::Equal)
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

/// How close two bodies were found to be, when [`CollisionRequest::distance`]
/// was set.
///
/// Upstream folds this into `CollisionResult::distance` (a bare `f64`) plus
/// `CollisionResult::distance_result` (a full [`DistanceResult`]), with the
/// doc comment on `CollisionRequest::detailed_distance` noting as prose that
/// the latter is only meaningful when *both* `distance` and
/// `detailed_distance` were requested — an implicit AND between two separate
/// bools, checked nowhere. Here `Detailed` can only exist together with the
/// full [`DistanceResult`] it requires, and there is exactly one place
/// ([`CollisionDistance::distance`]) to read the scalar back out of either
/// variant, instead of a second `f64` field a caller could read out of sync
/// with `distance_result`.
#[derive(Debug, Clone)]
pub enum CollisionDistance {
    /// `detailed_distance` was not set: only the closest distance itself.
    Closest(f64),
    /// `detailed_distance` was set: the full per-link distance breakdown.
    Detailed(DistanceResult),
}

impl CollisionDistance {
    /// The closest distance between two bodies, regardless of which variant
    /// this is.
    pub fn distance(&self) -> f64 {
        match self {
            Self::Closest(distance) => *distance,
            Self::Detailed(result) => result.minimum_distance.distance,
        }
    }

    /// Combine two distance computations covering disjoint parts of the same
    /// request (self-collision and robot-collision, in
    /// [`crate::env::CollisionEnv::check_collision`]'s default merge) into
    /// the overall closest distance.
    fn combine_closest(self, other: Self) -> Self {
        match (self, other) {
            (Self::Detailed(mut a), Self::Detailed(b)) => {
                if b.minimum_distance.distance < a.minimum_distance.distance {
                    a.minimum_distance = b.minimum_distance;
                }
                a.collision |= b.collision;
                a.distances.extend(b.distances);
                Self::Detailed(a)
            }
            (a, b) => {
                if b.distance() < a.distance() {
                    b
                } else {
                    a
                }
            }
        }
    }
}

/// Contacts found, when [`CollisionRequest::contacts`] was set.
///
/// Upstream splits this across two `CollisionResult` fields, `contact_count`
/// (a running total) and `contacts` (the pairwise map); every backend
/// increments the former exactly once per `Contact` pushed into the latter
/// (see `collision_detection_fcl/src/collision_common.cpp` and
/// `collision_detection_bullet/.../contact_checker_common.cpp`), so the two
/// never actually carry independent information. [`ContactData::count`]
/// derives the total from the map instead of caching a second copy that
/// could drift from it.
#[derive(Debug, Clone, Default)]
pub struct ContactData {
    /// Pairs of body ids in contact, plus their contact details.
    pub by_pair: BTreeMap<(String, String), Vec<Contact>>,
}

impl ContactData {
    /// `contact_count`: the total number of contacts across every pair.
    pub fn count(&self) -> usize {
        self.by_pair.values().map(Vec::len).sum()
    }

    /// Combine contacts found for disjoint parts of the same request
    /// (self-collision and robot-collision) into one set.
    fn merge(&mut self, other: Self) {
        for (pair, mut contacts) in other.by_pair {
            self.by_pair.entry(pair).or_default().append(&mut contacts);
        }
    }
}

/// Result of a collision-checking request.
///
/// Upstream `collision_detection::CollisionResult`.
///
/// # Deviation from upstream
///
/// Upstream's `distance`, `contact_count`/`contacts` and `cost_sources` are
/// each meaningful only when the matching [`CollisionRequest`] flag
/// (`distance`, `contacts`, `cost`) was set — otherwise they sit at their
/// default-constructed value (`f64::MAX`, `0`/empty map, empty set). That
/// default value is not unique to "not requested": `f64::MAX` is also the
/// literal answer for "no valid distance was found", and an empty
/// map/set is also the literal answer for "requested, and none exist" — the
/// exact same field value means two different things depending on a flag
/// stored on a different object entirely, which is the dual-meaning defect
/// PORTING-PLAN.md §4.1/§4.3 name (see [`crate::AllowedCollision`] for the
/// same fix applied to the allowed-collision matrix). Here each of the three
/// becomes `Option`-wrapped ([`CollisionDistance`] / [`ContactData`] /
/// `Vec<CostSource>`): `None` means "not requested", full stop, and
/// `Some` — even wrapping an empty `Vec` — always means "requested, and this
/// is what was found." Nothing upstream ever reads `contact_count`/`contacts`
/// without first checking `req.contacts` (see the `checkCollision` default
/// merge below), so no caller loses information by this change.
///
/// `print()` is not ported: it logs through `rclcpp`'s throttled logger
/// (`RCLCPP_WARN_STREAM_THROTTLE`), unavailable in a ROS-independent core
/// crate (PORTING-PLAN.md D1) and with no behavior to preserve once the
/// logging call itself is gone.
#[derive(Debug, Clone, Default)]
pub struct CollisionResult {
    /// Whether a collision was found.
    pub collision: bool,
    /// Distance information, present exactly when
    /// [`CollisionRequest::distance`] was set.
    pub distance: Option<CollisionDistance>,
    /// Contacts found, present exactly when [`CollisionRequest::contacts`]
    /// was set.
    pub contacts: Option<ContactData>,
    /// Individual cost sources, present exactly when
    /// [`CollisionRequest::cost`] was set.
    pub cost_sources: Option<Vec<CostSource>>,
}

impl CollisionResult {
    /// Reset to the same state a fresh [`CollisionResult`] starts in.
    ///
    /// Upstream `CollisionResult::clear()`.
    pub fn clear(&mut self) {
        *self = Self::default();
    }

    /// Combine a result covering a disjoint part of the same request into
    /// this one.
    ///
    /// Upstream's `CollisionEnv::checkCollision` passes one `CollisionResult&`
    /// by reference into both `checkSelfCollision` and `checkRobotCollision`,
    /// letting each backend accumulate into the same object in place. This
    /// port's `CollisionEnv` trait methods each return their own owned
    /// `CollisionResult` instead (this crate's established structured-return
    /// idiom — see `world`'s module docs), so `check_collision`'s default
    /// implementation calls this explicitly to fold the robot-collision
    /// result into the self-collision one.
    pub fn merge(&mut self, other: Self) {
        self.collision |= other.collision;
        self.distance = match (self.distance.take(), other.distance) {
            (None, None) => None,
            (a, None) => a,
            (None, b) => b,
            (Some(a), Some(b)) => Some(a.combine_closest(b)),
        };
        self.contacts = match (self.contacts.take(), other.contacts) {
            (None, None) => None,
            (a @ Some(_), None) => a,
            (None, b @ Some(_)) => b,
            (Some(mut a), Some(b)) => {
                a.merge(b);
                Some(a)
            }
        };
        self.cost_sources = match (self.cost_sources.take(), other.cost_sources) {
            (None, None) => None,
            (a @ Some(_), None) => a,
            (None, b @ Some(_)) => b,
            (Some(mut a), Some(b)) => {
                a.extend(b);
                Some(a)
            }
        };
    }
}
