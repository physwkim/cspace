// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection/include/moveit/collision_detection/world.hpp
//   moveit_core/collision_detection/src/world.cpp

//! The collision world: a container of named, posed [`Object`]s. Upstream
//! `collision_detection::World`.
//!
//! `RobotState` and the `bodies::` posed-geometry layer are out of scope —
//! see the crate docs — so this module ports the container itself: adding,
//! moving and removing objects and their shapes, object subframes, and the
//! change-notification mechanism other code (`PlanningScene`, a collision
//! backend) uses to stay in sync with a `World` it does not own.
//!
//! # Deviations from upstream
//!
//! 1. **One `Vec<ShapeEntry>`, not three parallel vectors.** Upstream
//!    `Object::shapes_`, `shape_poses_` and `global_shape_poses_` are three
//!    `std::vector`s a caller must index in lockstep; nothing stops them
//!    drifting to different lengths. [`Object::shapes`] is a single
//!    `Vec<`[`ShapeEntry`]`>`, so "shape `i`'s pose" is one fact, not three
//!    facts a caller must keep aligned.
//! 2. **One `BTreeMap`, not two parallel maps, for subframes.** The same
//!    defect exists one level down: `Object::subframe_poses_` and
//!    `global_subframe_poses_` are two `std::map`s upstream's own
//!    `updateGlobalPosesInternal` walks in lockstep, assuming matching
//!    iteration order. [`Object`]'s private `subframes` field is one
//!    `BTreeMap<String, Subframe>`; there is no second map that can disagree
//!    with it on keys or order.
//! 3. **`Object` is never observably pose-less.** Upstream's
//!    `Object(const std::string&)` constructor leaves `pose_` default
//!    constructed — for `Eigen::Isometry3d` this is uninitialized memory, not
//!    identity — and every call site (`World::addToObject`,
//!    `World::setObjectPose`) assigns `pose_` immediately afterward, so the
//!    gap is never externally observed. This port's private `Object::new`
//!    takes the pose as a parameter, so there is no gap to reason about.
//! 4. **No `addObserver`/`removeObserver`/`notify`.** Upstream registers
//!    `std::function` callbacks behind raw `Observer*` pointers compared by
//!    identity (`ObserverHandle`) — a callback-plus-context pair that this
//!    crate's own `PORTING-PLAN.md` philosophy replaces with "return a
//!    structured delta, let the owner apply it" wherever it appears. Storing
//!    closures in `World` instead (`Vec<Box<dyn Fn(&Object, Action)>>`) would
//!    dodge the raw-pointer identity problem but not the deeper one: a
//!    callback that needs to mutate its own captured state while `World`
//!    holds `&mut self` mid-mutation needs `RefCell`-style interior
//!    mutability to even compile, which is itself a runtime-checked patch
//!    over the same aliasing question the delta already answers for free.
//!    Every mutator here returns [`Option<Notification>`] (or
//!    `Vec<Notification>` for [`World::clear_objects`]) describing what
//!    changed; the caller decides how — and to whom — to dispatch it, with
//!    no lifetime, closure or interior-mutability concerns on this crate's
//!    side at all. [`World::all_objects_as_notifications`] replaces the one
//!    use upstream's `notifyObserverAllObjects` had that a plain return value
//!    cannot: replaying every current object to a newly attached observer.
//! 5. **No `ASSERT_ISOMETRY`.** Upstream asserts (debug builds only) that
//!    caller-supplied poses are valid isometries, because `Eigen::Isometry3d`
//!    is really just a 4x4 affine matrix that can be corrupted into a
//!    shear/scale by direct manipulation. `nalgebra::Isometry3` is a
//!    translation paired with a unit rotation and cannot represent a
//!    non-isometry at all — the runtime check upstream needs has no Rust
//!    counterpart because the illegal state is unrepresentable by
//!    construction, the same move `crate::transforms` — see
//!    `cspace_core::geometry::Transforms` — already made for the same reason.
//! 6. **`getTransform`'s two overloads become [`World::get_transform`] /
//!    [`World::try_get_transform`].** Same split as
//!    `cspace_core::geometry::Transforms::transform`/`try_transform`: the
//!    `Result`-returning form uses [`cspace_core::error::Error::UnknownName`] where
//!    upstream throws `std::runtime_error`, and the `Option`-returning form
//!    replaces the `bool& frame_found` out-parameter.
//! 7. **Unknown object/out-of-range shape index return `None`, not identity
//!    plus a log.** [`World::global_shape_transform`] and
//!    [`World::global_shape_transforms`] return `Option` where upstream logs
//!    an error and returns a static identity transform (single-shape case)
//!    or a static empty vector (all-shapes case) for an unknown object —
//!    and, for the single-shape case, does not check `shape_index` at all,
//!    so an out-of-range index is `std::vector::operator[]` undefined
//!    behavior upstream, not a defined case this port could match even if it
//!    wanted to.
//! 8. **`knows_transform` and `get_transform`/`try_get_transform` preserve a
//!    genuine upstream inconsistency, on purpose.** Both walk `objects_` for
//!    the first name that prefixes the query with a following `/`, but
//!    upstream's two loops are not the same loop: `knowsTransform` returns as
//!    soon as it finds the first such candidate object, using *that* object's
//!    subframe presence as the whole answer; `getTransform` keeps trying
//!    later candidates if the first one's subframe lookup misses. Two
//!    objects can only both be candidates when one's name is itself a
//!    strict, `/`-terminated prefix of the other's (e.g. `"a"` and
//!    `"a/b"`) — a real but narrow setup — and in exactly that setup the two
//!    methods can disagree: `knows_transform` can say `false` for a name
//!    `get_transform`/`try_get_transform` successfully resolves via the
//!    second candidate. This is verified straight from `World::knowsTransform`
//!    and `World::getTransform` in `world.cpp`, not inferred from the header
//!    comment (which only documents the intended, unambiguous case) — see
//!    `subframe_name_colliding_with_a_sibling_object_name_is_the_documented_ambiguity`
//!    below. Unifying the two into one shared loop would silently change
//!    which of them is "the buggy one," so neither is patched here.
//! 9. **`add_to_object` reproduces upstream's "existing object keeps its
//!    pose" quirk.** `World::addToObject(id, pose, shapes, shape_poses)`
//!    only assigns `pose` when it creates a fresh `Object`; calling it again
//!    on an existing object silently ignores the `pose` argument; and
//!    `shape_poses` is always relative to whatever the object's pose already
//!    is, not the just-ignored argument. Read directly from `world.cpp`
//!    (`if (!obj) { ...; obj->pose_ = pose; } else ensureUnique(obj);` —
//!    there is no assignment to `obj->pose_` in the `else` branch), not
//!    guessed from the doc comment, which does not mention this at all.
//! 10. **`move_object`'s identity short-circuit ports Eigen's
//!     `Transform::isApprox`/`NumTraits<double>::dummy_precision()` exactly**
//!     (`(a - b).cwiseAbs2().sum() <= prec² · min(‖a‖², ‖b‖²)`, `prec =
//!     1e-12`), verified against `Eigen/src/Core/Fuzzy.h` and
//!     `Eigen/src/Core/NumTraits.h`, rather than substituted with
//!     `approx::relative_eq!`'s own (differently shaped) default tolerance.
//! 11. **`removeObject`, not `removeObjectFromWorld`.** The upstream symbol
//!     is `bool World::removeObject(const std::string&)`; this port's
//!     [`World::remove_object`] names the method upstream actually has.
//!
//! # Declaration audit — `world.hpp` / `world.cpp`
//!
//! Every public declaration in `world.hpp`, each with a disposition. The
//! deviations above say *how* pieces differ; this says *whether each one is
//! here at all*, which is a different question and the one the crate-doc
//! prose above could not answer. It exists because "ported" is otherwise a
//! file-level claim — `doc/declaration-audit-coverage.md` is the tree-wide
//! measurement of how far that claim reaches.
//!
//! `tools/ci/count-public-declarations.sh world.hpp World` prints **37**, the
//! `public:` declarations at brace depth 1 of `class World`. The 37 bullets
//! below are that list, enumerated independently and then checked against the
//! script's count rather than assumed from it.
//!
//! ## `class World`, 37 declarations
//!
//! 1. `World()` → [`World::new`].
//! 2. `World(const World& other)` → `#[derive(Clone)]`; the copy-constructor
//!    body is `objects_ = other.objects_;` and nothing else, which is what
//!    deriving gives (see [`World`]'s own doc comment).
//! 3. `virtual ~World()` → **decided-non-port.** Its body drains
//!    `observers_` (`while (!observers_.empty()) removeObserver(...)`), and
//!    deviation 4 removed observers entirely, so there is nothing left to
//!    release; `BTreeMap`/`Arc` free themselves. `virtual` exists so a
//!    subclass destructor runs — this port has no subclass, `World` being
//!    concrete with no trait to implement.
//! 4. `MOVEIT_STRUCT_FORWARD(Object)` → `Arc<`[`Object`]`>`. The macro's
//!    `ObjectPtr`/`ObjectConstPtr` pair collapses to one type: `Arc<T>` hands
//!    out `&T`, so the const/non-const distinction upstream needs two aliases
//!    for is the default here. `ObjectWeakPtr` is unused upstream — nothing
//!    in `world.{hpp,cpp}` names it.
//! 5. `struct Object` → [`Object`], audited member-by-member below.
//! 6. `getObjectIds()` → [`World::object_ids`].
//! 7. `getObject()` → [`World::get_object`].
//! 8. `using const_iterator` → **decided-non-port.** It names
//!    `std::map<std::string, ObjectPtr>::const_iterator`, i.e. the container
//!    upstream happens to use; [`World::iter`] returns `impl Iterator`, which
//!    is the same access without publishing the representation.
//! 9. `begin()` → [`World::iter`].
//! 10. `end()` → [`World::iter`]. Upstream's begin/end pair is one Rust
//!     iterator; there is no separate end sentinel to expose.
//! 11. `size()` → [`World::len`] (and [`World::is_empty`], which Rust
//!     convention requires alongside `len` and upstream has no counterpart
//!     for).
//! 12. `find(object_id)` → [`World::get_object`]. Upstream returns an
//!     iterator the caller compares against `end()`; the `Option` return
//!     covers both the found and not-found halves of that, so `find` and
//!     `getObject` are one method here.
//! 13. `hasObject()` → [`World::has_object`].
//! 14. `knowsTransform()` → [`World::knows_transform`] (deviation 8).
//! 15. `getTransform(name)` → [`World::get_transform`] (deviation 6).
//! 16. `getTransform(name, bool& frame_found)` → [`World::try_get_transform`].
//! 17. `getGlobalShapeTransform()` → [`World::global_shape_transform`]
//!     (deviation 7).
//! 18. `getGlobalShapeTransforms()` → [`World::global_shape_transforms`].
//! 19. `addToObject(id, pose, shapes, shape_poses)` → [`World::add_to_object`]
//!     (deviation 9).
//! 20. `addToObject(id, shapes, shape_poses)` →
//!     [`World::add_shapes_to_object`].
//! 21. `addToObject(id, pose, shape, shape_pose)` →
//!     [`World::add_shape_to_object`].
//! 22. `addToObject(id, shape, shape_pose)` → [`World::add_shape`]. Upstream's
//!     four overloads are four names here because Rust has no overloading;
//!     the three convenience forms forward to the first exactly as upstream's
//!     inline bodies do.
//! 23. `moveShapeInObject()` → [`World::move_shape_in_object`].
//! 24. `moveShapesInObject()` → [`World::move_shapes_in_object`].
//! 25. `moveObject()` → [`World::move_object`] (deviation 10).
//! 26. `setObjectPose()` → [`World::set_object_pose`].
//! 27. `removeShapeFromObject()` → [`World::remove_shape_from_object`].
//! 28. `removeObject()` → [`World::remove_object`] (deviation 11).
//! 29. `setSubframesOfObject()` → [`World::set_subframes_of_object`].
//! 30. `clearObjects()` → [`World::clear_objects`].
//! 31. `enum ActionBits` → [`Action`]'s associated constants
//!     ([`Action::UNINITIALIZED`] … [`Action::REMOVE_SHAPE`]), same values.
//! 32. `class Action` → [`Action`]. Upstream's `Action` is a one-`int`
//!     wrapper whose only members are two constructors and
//!     `operator ActionBits()`; here the wrapper and the bits are one type,
//!     so the implicit conversion has nothing to convert between.
//! 33. `class ObserverHandle` → **decided-non-port** (deviation 4). It is the
//!     identity token for a registered callback, and there are no registered
//!     callbacks.
//! 34. `using ObserverCallbackFn` → **decided-non-port** (deviation 4). Its
//!     `(const ObjectConstPtr&, Action)` payload is exactly
//!     [`Notification`]'s two fields, which is where that signature went.
//! 35. `addObserver()` → **decided-non-port** (deviation 4). Every mutator
//!     returns the [`Notification`] the callback would have received.
//! 36. `removeObserver()` → **decided-non-port** (deviation 4). Nothing is
//!     registered, so nothing is unregistered.
//! 37. `notifyObserverAllObjects()` → [`World::all_objects_as_notifications`].
//!     The one observer operation with no return-value equivalent — replaying
//!     the current world to a newly attached observer — so it is the one that
//!     survives deviation 4.
//!
//! Expiry for 33–36 as a group: an in-tree caller that must be notified
//! *without* holding the `&mut World` that produced the change. Every caller
//! in this tree today has the return value in hand at the point of the
//! change, so the callback registry has nothing to do that a returned
//! [`Notification`] does not already do.
//!
//! ## `struct Object`, 8 declarations
//!
//! `count-public-declarations.sh` cannot count these: it matches
//! `class <name>` and `Object` is a `struct`, and it excludes members nested
//! below depth 1 by design. Enumerated by hand from `world.hpp:78-117`
//! instead — a `struct` has no access specifiers to track, so the whole body
//! is public.
//!
//! 1. `Object(const std::string& object_id)` → `Object::new(id, pose)`,
//!    **private on purpose** (deviation 3): an `Object` outside a [`World`]
//!    has no way to keep its global poses current, so the only constructor is
//!    the one [`World`] calls.
//! 2. `id_` → [`Object::id`].
//! 3. `pose_` → [`Object::pose`].
//! 4. `shapes_` → [`ShapeEntry::shape`], through [`Object::shapes`].
//! 5. `shape_poses_` → [`ShapeEntry::pose`] (deviation 1).
//! 6. `global_shape_poses_` → [`ShapeEntry::global_pose`].
//! 7. `subframe_poses_` → [`Object::subframe_pose`], plus
//!    [`Object::subframe_names`] for the key set `std::map` exposed directly
//!    (deviation 2).
//! 8. `global_subframe_poses_` → [`Object::global_subframe_pose`].
//!
//! All eight are read-only accessors here where upstream has public fields.
//! That is deliberate and is what deviations 1 and 2 buy: a writable
//! `shape_poses_` is precisely how the three vectors drift apart upstream.
//!
//! `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` (`world.hpp:84`) is not counted as a
//! declaration — it is an allocator override for over-aligned Eigen members,
//! the same case `count-public-declarations.sh` documents skipping. No Rust
//! counterpart exists or is needed.
//!
//! ## Private declarations and `world.cpp`
//!
//! Not owed by a *public* declaration audit, recorded because the
//! substitutions are not one-to-one: `ensureUnique` → [`Arc::make_mut`] via
//! this module's `ensure_unique`; `addToObjectInternal` → `Object::push_shape`
//! (not `virtual` here — nothing subclasses `World`);
//! `updateGlobalPosesInternal` → `Object::recompute_global_poses`, which has
//! no `update_shape_poses`/`update_subframe_poses` flags because it always
//! does both; `notify`/`notifyAll` → gone with deviation 4; `objects_` →
//! `World::objects`; `observers_` → gone.
//!
//! `world.cpp` adds one file-local declaration to the header's list: an
//! anonymous-namespace `getLogger()` returning an `rclcpp::Logger`
//! (`world.cpp:47-50`), excluded by D1 — this crate references no ROS type.
//! Every other definition in the file implements a declaration listed above.

use std::collections::BTreeMap;
use std::sync::Arc;

use cspace_core::error::{Error, Result};
use cspace_core::geometry::{Isometry3, Shape};

/// Bits describing what happened to an [`Object`] in one [`Notification`].
/// Upstream `World::ActionBits`/`World::Action`.
///
/// Several bits may be set (e.g. `CREATE | ADD_SHAPE`, upstream's own
/// example). If [`Action::DESTROY`] is set, upstream guarantees no other bit
/// is — this port does not enforce that as an invariant (nothing prevents
/// constructing `Action::DESTROY | Action::ADD_SHAPE` by hand), it just never
/// produces such a value itself, matching every call site in `world.cpp`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Action(u8);

impl Action {
    /// Upstream `World::UNINITIALIZED`. [`Action::default`].
    pub const UNINITIALIZED: Self = Self(0);
    /// Upstream `World::CREATE`: the object was created.
    pub const CREATE: Self = Self(1);
    /// Upstream `World::DESTROY`: the object was destroyed.
    pub const DESTROY: Self = Self(2);
    /// Upstream `World::MOVE_SHAPE`: one or more shapes in the object moved.
    pub const MOVE_SHAPE: Self = Self(4);
    /// Upstream `World::ADD_SHAPE`: shape(s) were added to the object.
    pub const ADD_SHAPE: Self = Self(8);
    /// Upstream `World::REMOVE_SHAPE`: shape(s) were removed from the object.
    pub const REMOVE_SHAPE: Self = Self(16);

    /// Whether every bit set in `other` is also set in `self`.
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// The raw bitmask, for display or wire encoding.
    pub const fn bits(self) -> u8 {
        self.0
    }
}

impl Default for Action {
    fn default() -> Self {
        Self::UNINITIALIZED
    }
}

impl std::ops::BitOr for Action {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

impl std::ops::BitOrAssign for Action {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

/// A change that happened to an [`Object`] in a [`World`], for the caller to
/// dispatch however it needs to. Replaces upstream's
/// `ObserverCallbackFn = std::function<void(const ObjectConstPtr&, Action)>`
/// — see the module docs, deviation 4.
#[derive(Debug, Clone)]
pub struct Notification {
    /// The object's state right after the change.
    pub object: Arc<Object>,
    /// What happened.
    pub action: Action,
}

/// One shape making up an [`Object`], with its pose cached in both the
/// object's own frame and the world frame. Unifies upstream's
/// `Object::shapes_`/`shape_poses_`/`global_shape_poses_` triple — see the
/// module docs, deviation 1.
///
/// Both fields are private: mutating a shape's pose without recomputing its
/// global pose together is exactly the desync this type exists to make
/// impossible, so the only way to change one is through a [`World`] method
/// that changes both in the same step.
#[derive(Debug, Clone)]
pub struct ShapeEntry {
    shape: Arc<Shape>,
    pose: Isometry3,
    global_pose: Isometry3,
}

impl ShapeEntry {
    /// The shape itself, shared with any other object or `World` that also
    /// holds this same `Arc`. Upstream `shapes::ShapeConstPtr` in `shapes_`.
    pub fn shape(&self) -> &Arc<Shape> {
        &self.shape
    }

    /// This shape's pose relative to its object's own pose. Upstream
    /// `shape_poses_[i]`.
    pub fn pose(&self) -> Isometry3 {
        self.pose
    }

    /// This shape's pose in the world frame: the object's pose composed with
    /// [`ShapeEntry::pose`]. Upstream `global_shape_poses_[i]`.
    pub fn global_pose(&self) -> Isometry3 {
        self.global_pose
    }
}

/// One subframe on an [`Object`], with its pose cached in both the object's
/// own frame and the world frame. Unifies upstream's
/// `subframe_poses_`/`global_subframe_poses_` pair — see the module docs,
/// deviation 2.
#[derive(Debug, Clone, Copy)]
struct Subframe {
    pose: Isometry3,
    global_pose: Isometry3,
}

/// A named object in a [`World`]: a pose, a set of posed shapes, and a set of
/// named, posed subframes. Upstream `World::Object`.
///
/// Every field is private and reachable only through methods; [`World`]'s own
/// methods are the sole owners of when an `Object`'s shapes, subframes or
/// pose change (see the module docs, deviations 1 and 2 for why the internal
/// representation needs that ownership to hold its own invariants).
#[derive(Debug, Clone)]
pub struct Object {
    id: String,
    pose: Isometry3,
    shapes: Vec<ShapeEntry>,
    subframes: BTreeMap<String, Subframe>,
}

impl Object {
    fn new(id: String, pose: Isometry3) -> Self {
        Self {
            id,
            pose,
            shapes: Vec::new(),
            subframes: BTreeMap::new(),
        }
    }

    fn push_shape(&mut self, shape: Arc<Shape>, pose: Isometry3) {
        let global_pose = self.pose * pose;
        self.shapes.push(ShapeEntry {
            shape,
            pose,
            global_pose,
        });
    }

    /// Recompute every shape's and subframe's global pose from the object's
    /// current pose. Upstream `updateGlobalPosesInternal(obj, true, true)` —
    /// always both, since (per deviations 1 and 2) there is only one shape
    /// list and one subframe map here, not a pair upstream lets the caller
    /// update independently.
    fn recompute_global_poses(&mut self) {
        let pose = self.pose;
        for entry in &mut self.shapes {
            entry.global_pose = pose * entry.pose;
        }
        for subframe in self.subframes.values_mut() {
            subframe.global_pose = pose * subframe.pose;
        }
    }

    /// This object's id. Upstream `id_`.
    pub fn id(&self) -> &str {
        &self.id
    }

    /// This object's pose. All shapes and subframes are relative to it.
    /// Upstream `pose_`.
    pub fn pose(&self) -> Isometry3 {
        self.pose
    }

    /// This object's shapes. Upstream `shapes_`/`shape_poses_`/
    /// `global_shape_poses_`, unified — see [`ShapeEntry`].
    pub fn shapes(&self) -> &[ShapeEntry] {
        &self.shapes
    }

    /// This object's subframe pose relative to the object's own pose, if
    /// `name` names one. Upstream `subframe_poses_.find(name)`.
    pub fn subframe_pose(&self, name: &str) -> Option<Isometry3> {
        self.subframes.get(name).map(|s| s.pose)
    }

    /// This object's subframe pose in the world frame, if `name` names one.
    /// Upstream `global_subframe_poses_.find(name)`.
    pub fn global_subframe_pose(&self, name: &str) -> Option<Isometry3> {
        self.subframes.get(name).map(|s| s.global_pose)
    }

    /// Every subframe name on this object.
    pub fn subframe_names(&self) -> impl Iterator<Item = &str> {
        self.subframes.keys().map(String::as_str)
    }
}

/// Whether the CREATE bit should be reported.
const fn created_bit(created: bool) -> Action {
    if created {
        Action::CREATE
    } else {
        Action::UNINITIALIZED
    }
}

/// If `name` is `object_name` followed by `/`, the suffix after the slash.
/// Upstream's repeated `name.rfind(object.first, 0) == 0 && name[object.first.length()] == '/'`
/// check in `World::knowsTransform` and `World::getTransform`.
fn subframe_suffix<'a>(name: &'a str, object_name: &str) -> Option<&'a str> {
    name.strip_prefix(object_name)?.strip_prefix('/')
}

/// Eigen's `NumTraits<double>::dummy_precision()`, verified against
/// `Eigen/src/Core/NumTraits.h` — see the module docs, deviation 10.
const EIGEN_DUMMY_PRECISION: f64 = 1e-12;

/// Eigen's `Transform::isApprox`/`MatrixBase::isApprox` on the two
/// transforms' full 4x4 homogeneous matrices, verified against
/// `Eigen/src/Geometry/Transform.h` and `Eigen/src/Core/Fuzzy.h` — see the
/// module docs, deviation 10. Not `approx::relative_eq!`: Eigen's formula
/// compares squared Frobenius norms with a single shared precision, which is
/// a different shape of tolerance than `relative_eq!`'s per-component
/// epsilon/max-relative pair.
fn eigen_is_approx(a: &Isometry3, b: &Isometry3) -> bool {
    let ma = a.to_homogeneous();
    let mb = b.to_homogeneous();
    let diff_sq: f64 = (ma - mb).iter().map(|x| x * x).sum();
    let norm_a: f64 = ma.iter().map(|x| x * x).sum();
    let norm_b: f64 = mb.iter().map(|x| x * x).sum();
    diff_sq <= EIGEN_DUMMY_PRECISION * EIGEN_DUMMY_PRECISION * norm_a.min(norm_b)
}

/// The outcome of [`World::move_object`]. Upstream returns a plain `bool`
/// (`false` only when the object does not exist; `true` covers both "moved"
/// and "already there, did nothing"), which is enough for upstream's direct
/// caller because the DESTROY/CREATE/MOVE_SHAPE distinction upstream's
/// `bool` doesn't carry travels separately, through the observer callback.
/// This port has no separate channel (see the module docs, deviation 4), so
/// the three outcomes upstream's caller could previously only reconstruct by
/// combining the `bool` return with what its observer saw are named here
/// directly instead.
#[derive(Debug, Clone)]
pub enum MoveObjectOutcome {
    /// No object with that id exists.
    NotFound,
    /// The object exists but `transform` was the identity (Eigen
    /// `isApprox`), so nothing changed and no notification was produced.
    /// Upstream's early `if (transform.isApprox(Identity)) return true;`.
    NoChange,
    /// The object moved.
    Moved(Notification),
}

/// A container of named, posed objects. Upstream `collision_detection::World`.
///
/// `#[derive(Clone)]` reproduces upstream's copy constructor
/// (`objects_ = other.objects_;`) exactly: cloning a `BTreeMap<String,
/// Arc<Object>>` clones the map's keys and bumps each `Object`'s refcount, the
/// same shallow, copy-on-write-backed copy `std::map<std::string, ObjectPtr>`
/// gives upstream for free from `shared_ptr`'s own copy semantics.
#[derive(Debug, Clone, Default)]
pub struct World {
    objects: BTreeMap<String, Arc<Object>>,
}

impl World {
    /// An empty world.
    pub fn new() -> Self {
        Self::default()
    }

    /// Every object id. Upstream `getObjectIds`.
    pub fn object_ids(&self) -> Vec<String> {
        self.objects.keys().cloned().collect()
    }

    /// A snapshot of the named object, if it exists. Upstream `getObject`.
    ///
    /// The returned `Arc` shares the `Object` this `World` currently holds —
    /// if a later mutation needs to change that same object, [`World`]
    /// clones a fresh copy for itself rather than mutating through this
    /// handle, so a snapshot taken before a mutation keeps reading the
    /// pre-mutation state. Upstream `ensureUnique`'s `use_count() != 1` copy-
    /// on-write, ported directly onto `Arc::strong_count`.
    pub fn get_object(&self, id: &str) -> Option<Arc<Object>> {
        self.objects.get(id).cloned()
    }

    /// Whether an object with this id exists. Upstream `hasObject`.
    pub fn has_object(&self, id: &str) -> bool {
        self.objects.contains_key(id)
    }

    /// The number of objects. Upstream `size`.
    pub fn len(&self) -> usize {
        self.objects.len()
    }

    /// Whether there are no objects. Upstream has no equivalent; added for
    /// the `len`/`is_empty` pair clippy expects.
    pub fn is_empty(&self) -> bool {
        self.objects.is_empty()
    }

    /// Every object, in id order. Upstream `begin`/`end`.
    pub fn iter(&self) -> impl Iterator<Item = (&String, &Arc<Object>)> {
        self.objects.iter()
    }

    /// Ensure the map entry `id` is the sole owner of its `Object`, cloning
    /// a fresh copy if some [`World::get_object`] snapshot is still holding
    /// the old one. Upstream `ensureUnique`.
    fn ensure_unique(arc: &mut Arc<Object>) {
        if Arc::strong_count(arc) != 1 {
            *arc = Arc::new((**arc).clone());
        }
    }

    /// The object `id`, uniquely owned by this `World`, creating it at
    /// `default_pose` if it did not exist. The single funnel every mutator
    /// below that can create an object goes through.
    fn object_mut_or_create(&mut self, id: &str, default_pose: Isometry3) -> &mut Object {
        let arc = self
            .objects
            .entry(id.to_owned())
            .or_insert_with(|| Arc::new(Object::new(id.to_owned(), default_pose)));
        Self::ensure_unique(arc);
        Arc::get_mut(arc).expect("ensure_unique just made this the sole owner")
    }

    /// The object `id`, uniquely owned by this `World`, if it exists. The
    /// single funnel every mutator below that never creates an object goes
    /// through.
    fn object_mut(&mut self, id: &str) -> Option<&mut Object> {
        let arc = self.objects.get_mut(id)?;
        Self::ensure_unique(arc);
        Some(Arc::get_mut(arc).expect("ensure_unique just made this the sole owner"))
    }

    /// Add posed shapes to an object at `pose`, creating the object if it
    /// does not exist. The one worker every `add_*` method below funnels
    /// through. Upstream `addToObject(id, pose, shapes, shape_poses)`.
    ///
    /// `None`, with no mutation, when `shapes.len() != shape_poses.len()`
    /// (upstream logs and returns) or `shapes` is empty (upstream returns
    /// silently). See the module docs, deviation 9, for why `pose` is
    /// ignored — not just unused for computing shape poses, genuinely
    /// discarded — when the object already exists.
    pub fn add_to_object(
        &mut self,
        id: &str,
        pose: Isometry3,
        shapes: &[Arc<Shape>],
        shape_poses: &[Isometry3],
    ) -> Option<Notification> {
        if shapes.len() != shape_poses.len() || shapes.is_empty() {
            return None;
        }
        let created = !self.objects.contains_key(id);
        let obj = self.object_mut_or_create(id, pose);
        for (shape, shape_pose) in shapes.iter().zip(shape_poses) {
            obj.push_shape(Arc::clone(shape), *shape_pose);
        }
        Some(Notification {
            object: Arc::clone(&self.objects[id]),
            action: Action::ADD_SHAPE | created_bit(created),
        })
    }

    /// [`World::add_to_object`] at the identity pose. Upstream
    /// `addToObject(id, shapes, shape_poses)`.
    pub fn add_shapes_to_object(
        &mut self,
        id: &str,
        shapes: &[Arc<Shape>],
        shape_poses: &[Isometry3],
    ) -> Option<Notification> {
        self.add_to_object(id, Isometry3::identity(), shapes, shape_poses)
    }

    /// [`World::add_to_object`] with a single shape. Upstream
    /// `addToObject(id, pose, shape, shape_pose)`.
    pub fn add_shape_to_object(
        &mut self,
        id: &str,
        pose: Isometry3,
        shape: Arc<Shape>,
        shape_pose: Isometry3,
    ) -> Option<Notification> {
        self.add_to_object(id, pose, &[shape], &[shape_pose])
    }

    /// [`World::add_to_object`] with a single shape at the identity pose.
    /// Upstream `addToObject(id, shape, shape_pose)`.
    pub fn add_shape(
        &mut self,
        id: &str,
        shape: Arc<Shape>,
        shape_pose: Isometry3,
    ) -> Option<Notification> {
        self.add_to_object(id, Isometry3::identity(), &[shape], &[shape_pose])
    }

    /// Move one shape (matched by `Arc` identity, upstream's pointer
    /// equality) to a new pose relative to its object. Upstream
    /// `moveShapeInObject`.
    ///
    /// `None` if the object does not exist or does not hold this shape.
    pub fn move_shape_in_object(
        &mut self,
        id: &str,
        shape: &Arc<Shape>,
        shape_pose: Isometry3,
    ) -> Option<Notification> {
        let obj = self.object_mut(id)?;
        let idx = obj
            .shapes
            .iter()
            .position(|e| Arc::ptr_eq(&e.shape, shape))?;
        let global_pose = obj.pose * shape_pose;
        obj.shapes[idx].pose = shape_pose;
        obj.shapes[idx].global_pose = global_pose;
        Some(Notification {
            object: Arc::clone(&self.objects[id]),
            action: Action::MOVE_SHAPE,
        })
    }

    /// Move every shape in an object to new poses, in the object's own
    /// shape order. Upstream `moveShapesInObject`.
    ///
    /// `None`, with no mutation, if the object does not exist or
    /// `shape_poses.len()` does not match its current shape count — upstream
    /// requires the exact count too, and touches nothing if it disagrees.
    pub fn move_shapes_in_object(
        &mut self,
        id: &str,
        shape_poses: &[Isometry3],
    ) -> Option<Notification> {
        let obj = self.object_mut(id)?;
        if shape_poses.len() != obj.shapes.len() {
            return None;
        }
        let object_pose = obj.pose;
        for (entry, &pose) in obj.shapes.iter_mut().zip(shape_poses) {
            entry.pose = pose;
            entry.global_pose = object_pose * pose;
        }
        Some(Notification {
            object: Arc::clone(&self.objects[id]),
            action: Action::MOVE_SHAPE,
        })
    }

    /// Move an object's pose by `transform`, applied in the world frame
    /// (`new_pose = transform * old_pose`). Upstream `moveObject`.
    ///
    /// See [`MoveObjectOutcome`] for how this reproduces upstream's
    /// early-exit on a transform that `eigen_is_approx`s to the identity —
    /// see the module docs, deviation 10, for the exact formula.
    pub fn move_object(&mut self, id: &str, transform: Isometry3) -> MoveObjectOutcome {
        let Some(old_pose) = self.objects.get(id).map(|o| o.pose) else {
            return MoveObjectOutcome::NotFound;
        };
        if eigen_is_approx(&transform, &Isometry3::identity()) {
            return MoveObjectOutcome::NoChange;
        }
        let notification = self.set_object_pose(id, transform * old_pose);
        MoveObjectOutcome::Moved(notification)
    }

    /// Set an object's pose in the world frame, creating the object (with no
    /// shapes) if it does not exist. Upstream `setObjectPose`.
    ///
    /// Always produces a notification: `CREATE` for a fresh object,
    /// `MOVE_SHAPE` for an existing one with at least one shape, or
    /// [`Action::UNINITIALIZED`] (no bits set) for an existing, shapeless
    /// one — matching `action = obj->shapes_.empty() ? 0 : MOVE_SHAPE;`
    /// exactly; upstream still calls `notify` with that all-zero action, so
    /// this still returns a `Notification`, not `None`.
    pub fn set_object_pose(&mut self, id: &str, pose: Isometry3) -> Notification {
        let created = !self.objects.contains_key(id);
        let obj = self.object_mut_or_create(id, pose);
        let action = if created {
            Action::CREATE
        } else if obj.shapes.is_empty() {
            Action::UNINITIALIZED
        } else {
            Action::MOVE_SHAPE
        };
        obj.pose = pose;
        obj.recompute_global_poses();
        Notification {
            object: Arc::clone(&self.objects[id]),
            action,
        }
    }

    /// Remove one shape (matched by `Arc` identity) from an object,
    /// destroying the object entirely if that was its last shape. Upstream
    /// `removeShapeFromObject`.
    ///
    /// `None` if the object does not exist or does not hold this shape.
    pub fn remove_shape_from_object(
        &mut self,
        id: &str,
        shape: &Arc<Shape>,
    ) -> Option<Notification> {
        let obj = self.object_mut(id)?;
        let idx = obj
            .shapes
            .iter()
            .position(|e| Arc::ptr_eq(&e.shape, shape))?;
        obj.shapes.remove(idx);
        let now_empty = obj.shapes.is_empty();
        if now_empty {
            let object = self
                .objects
                .remove(id)
                .expect("object_mut just confirmed this id exists");
            Some(Notification {
                object,
                action: Action::DESTROY,
            })
        } else {
            Some(Notification {
                object: Arc::clone(&self.objects[id]),
                action: Action::REMOVE_SHAPE,
            })
        }
    }

    /// Remove an object entirely. Upstream `removeObject` — see the module
    /// docs, deviation 11, for the name.
    ///
    /// `None` if no such object exists.
    pub fn remove_object(&mut self, id: &str) -> Option<Notification> {
        let object = self.objects.remove(id)?;
        Some(Notification {
            object,
            action: Action::DESTROY,
        })
    }

    /// Remove every object. Upstream `clearObjects`.
    ///
    /// Returns one `DESTROY` notification per object that existed, in id
    /// order — upstream's `notifyAll(DESTROY)` runs before `objects_.clear()`,
    /// so every observer sees every object that existed at the time of the
    /// call.
    pub fn clear_objects(&mut self) -> Vec<Notification> {
        let notifications = self.all_objects_as_notifications(Action::DESTROY);
        self.objects.clear();
        notifications
    }

    /// Set an object's subframes, replacing whatever it had. Upstream
    /// `setSubframesOfObject`.
    ///
    /// `false`, with no mutation — this method never creates an object — if
    /// no object named `id` exists. Produces no [`Notification`]: upstream's
    /// `setSubframesOfObject` does not call `notify` at all, unlike every
    /// other mutator here; this is a real upstream asymmetry, not an
    /// oversight in this port.
    pub fn set_subframes_of_object(
        &mut self,
        id: &str,
        subframe_poses: BTreeMap<String, Isometry3>,
    ) -> bool {
        let Some(obj) = self.object_mut(id) else {
            return false;
        };
        let object_pose = obj.pose;
        obj.subframes = subframe_poses
            .into_iter()
            .map(|(name, pose)| {
                let global_pose = object_pose * pose;
                (name, Subframe { pose, global_pose })
            })
            .collect();
        true
    }

    /// The global transform to a single shape of an object. Upstream
    /// `getGlobalShapeTransform`.
    ///
    /// `None` for an unknown object or an out-of-range `shape_index` — see
    /// the module docs, deviation 7.
    pub fn global_shape_transform(&self, object_id: &str, shape_index: usize) -> Option<Isometry3> {
        self.objects
            .get(object_id)?
            .shapes
            .get(shape_index)
            .map(ShapeEntry::global_pose)
    }

    /// The global transforms to every shape of an object, in shape order.
    /// Upstream `getGlobalShapeTransforms`.
    ///
    /// `None` for an unknown object — see the module docs, deviation 7.
    pub fn global_shape_transforms(&self, object_id: &str) -> Option<Vec<Isometry3>> {
        Some(
            self.objects
                .get(object_id)?
                .shapes
                .iter()
                .map(ShapeEntry::global_pose)
                .collect(),
        )
    }

    /// Whether an object or subframe named `name` exists. Upstream
    /// `knowsTransform`.
    ///
    /// A subframe name is `"<object id>/<subframe name>"`. See the module
    /// docs, deviation 8: this does **not** always agree with
    /// [`World::get_transform`]/[`World::try_get_transform`] on that case.
    pub fn knows_transform(&self, name: &str) -> bool {
        if self.objects.contains_key(name) {
            return true;
        }
        for (object_name, object) in &self.objects {
            if let Some(suffix) = subframe_suffix(name, object_name) {
                return object.subframes.contains_key(suffix);
            }
        }
        false
    }

    /// The global transform to an object or subframe named `name`, or
    /// [`Error::UnknownName`] if none resolves. Upstream the throwing
    /// `getTransform(const std::string&)` overload.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] when `name` does not resolve — see
    /// [`World::try_get_transform`].
    pub fn get_transform(&self, name: &str) -> Result<Isometry3> {
        self.try_get_transform(name)
            .ok_or_else(|| Error::unknown_name("transform", name))
    }

    /// The global transform to an object or subframe named `name`, or
    /// `None`. Upstream `getTransform(name, bool& frame_found)`.
    ///
    /// A subframe name is `"<object id>/<subframe name>"`. An exact object
    /// name always wins over any subframe interpretation, checked first —
    /// upstream checks `objects_.find(name)` before ever looking at
    /// subframes. See the module docs, deviation 8, for how this can
    /// disagree with [`World::knows_transform`] when more than one object
    /// name could prefix-match `name`.
    pub fn try_get_transform(&self, name: &str) -> Option<Isometry3> {
        if let Some(object) = self.objects.get(name) {
            return Some(object.pose);
        }
        for (object_name, object) in &self.objects {
            if let Some(suffix) = subframe_suffix(name, object_name)
                && let Some(subframe) = object.subframes.get(suffix)
            {
                return Some(subframe.global_pose);
            }
        }
        None
    }

    /// One notification per current object, all carrying `action`. Replaces
    /// upstream `notifyObserverAllObjects` — see the module docs, deviation
    /// 4: the caller dispatches these to whichever newly attached observer
    /// needs to catch up, instead of `World` doing it through a stored
    /// handle.
    pub fn all_objects_as_notifications(&self, action: Action) -> Vec<Notification> {
        self.objects
            .values()
            .map(|object| Notification {
                object: Arc::clone(object),
                action,
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sphere(radius: f64) -> Arc<Shape> {
        Arc::new(Shape::Sphere(
            cspace_core::geometry::Sphere::new(radius).unwrap(),
        ))
    }

    fn translation(x: f64, y: f64, z: f64) -> Isometry3 {
        Isometry3::translation(x, y, z)
    }

    // --- eigen_is_approx: the exact boundary, not a narrative case ---
    // identity: 4 (3 rotation-diagonal ones + the homogeneous corner 1).
    // threshold = prec^2 * 4 = 4e-24; diff_sq = dx^2.
    #[test]
    fn eigen_is_approx_below_threshold_is_approx() {
        let a = Isometry3::identity();
        let b = translation(1e-13, 0.0, 0.0); // diff_sq = 1e-26 < 4e-24
        assert!(eigen_is_approx(&a, &b));
    }

    #[test]
    fn eigen_is_approx_above_threshold_is_not_approx() {
        let a = Isometry3::identity();
        let b = translation(1e-11, 0.0, 0.0); // diff_sq = 1e-22 > 4e-24
        assert!(!eigen_is_approx(&a, &b));
    }

    // --- add_to_object ---

    #[test]
    fn add_to_object_creates_with_given_pose_and_shape_global_pose() {
        let mut world = World::new();
        let pose = translation(1.0, 0.0, 0.0);
        let shape_pose = translation(0.0, 2.0, 0.0);
        let notification = world
            .add_to_object("box", pose, &[sphere(1.0)], &[shape_pose])
            .expect("fresh object with one shape must notify");
        assert!(notification.action.contains(Action::CREATE));
        assert!(notification.action.contains(Action::ADD_SHAPE));
        let obj = world.get_object("box").unwrap();
        assert_eq!(obj.pose(), pose);
        assert_eq!(obj.shapes().len(), 1);
        assert_eq!(obj.shapes()[0].pose(), shape_pose);
        assert_eq!(obj.shapes()[0].global_pose(), pose * shape_pose);
    }

    #[test]
    fn add_to_object_on_existing_object_ignores_pose_argument() {
        let mut world = World::new();
        let original_pose = translation(1.0, 0.0, 0.0);
        world
            .add_to_object(
                "box",
                original_pose,
                &[sphere(1.0)],
                &[Isometry3::identity()],
            )
            .unwrap();
        let notification = world
            .add_to_object(
                "box",
                translation(99.0, 99.0, 99.0),
                &[sphere(1.0)],
                &[Isometry3::identity()],
            )
            .expect("existing object with a new shape must notify");
        assert!(!notification.action.contains(Action::CREATE));
        assert!(notification.action.contains(Action::ADD_SHAPE));
        // The pose argument on the second call is discarded entirely: the
        // object keeps its original pose (module docs, deviation 9).
        assert_eq!(world.get_object("box").unwrap().pose(), original_pose);
    }

    #[test]
    fn add_to_object_mismatched_lengths_is_a_no_op() {
        let mut world = World::new();
        let result = world.add_to_object(
            "box",
            Isometry3::identity(),
            &[sphere(1.0), sphere(1.0)],
            &[Isometry3::identity()],
        );
        assert!(result.is_none());
        assert!(!world.has_object("box"));
    }

    #[test]
    fn add_to_object_empty_shapes_is_a_no_op() {
        let mut world = World::new();
        let result = world.add_to_object("box", Isometry3::identity(), &[], &[]);
        assert!(result.is_none());
        assert!(!world.has_object("box"));
    }

    #[test]
    fn add_shape_convenience_overloads_use_identity_pose() {
        let mut world = World::new();
        world
            .add_shape("box", sphere(1.0), translation(1.0, 0.0, 0.0))
            .unwrap();
        assert_eq!(
            world.get_object("box").unwrap().pose(),
            Isometry3::identity()
        );
    }

    // --- move_shape_in_object / move_shapes_in_object ---

    #[test]
    fn move_shape_in_object_updates_pose_and_global_pose() {
        let mut world = World::new();
        let shape = sphere(1.0);
        let object_pose = translation(1.0, 0.0, 0.0);
        world
            .add_to_object(
                "box",
                object_pose,
                &[Arc::clone(&shape)],
                &[Isometry3::identity()],
            )
            .unwrap();
        let new_shape_pose = translation(0.0, 5.0, 0.0);
        let notification = world
            .move_shape_in_object("box", &shape, new_shape_pose)
            .unwrap();
        assert_eq!(notification.action, Action::MOVE_SHAPE);
        let obj = world.get_object("box").unwrap();
        assert_eq!(obj.shapes()[0].pose(), new_shape_pose);
        assert_eq!(obj.shapes()[0].global_pose(), object_pose * new_shape_pose);
    }

    #[test]
    fn move_shape_in_object_unknown_shape_is_none() {
        let mut world = World::new();
        world
            .add_to_object(
                "box",
                Isometry3::identity(),
                &[sphere(1.0)],
                &[Isometry3::identity()],
            )
            .unwrap();
        let other_shape = sphere(1.0);
        assert!(
            world
                .move_shape_in_object("box", &other_shape, Isometry3::identity())
                .is_none()
        );
    }

    #[test]
    fn move_shape_in_object_unknown_object_is_none() {
        let mut world = World::new();
        assert!(
            world
                .move_shape_in_object("box", &sphere(1.0), Isometry3::identity())
                .is_none()
        );
    }

    #[test]
    fn move_shapes_in_object_count_mismatch_is_a_no_op() {
        let mut world = World::new();
        world
            .add_to_object(
                "box",
                Isometry3::identity(),
                &[sphere(1.0)],
                &[Isometry3::identity()],
            )
            .unwrap();
        let before = world.get_object("box").unwrap().shapes()[0].pose();
        assert!(
            world
                .move_shapes_in_object("box", &[Isometry3::identity(), Isometry3::identity()])
                .is_none()
        );
        assert_eq!(world.get_object("box").unwrap().shapes()[0].pose(), before);
    }

    #[test]
    fn move_shapes_in_object_unknown_object_is_none() {
        // Zero poses: an auto-vivified empty object would also have zero
        // shapes, so this can only return `None` via the object-missing
        // guard, not incidentally via the count-mismatch guard too.
        let mut world = World::new();
        assert!(world.move_shapes_in_object("box", &[]).is_none());
    }

    // --- move_object ---

    #[test]
    fn move_object_not_found() {
        let mut world = World::new();
        assert!(matches!(
            world.move_object("nope", translation(1.0, 0.0, 0.0)),
            MoveObjectOutcome::NotFound
        ));
    }

    #[test]
    fn move_object_identity_transform_is_no_change() {
        let mut world = World::new();
        world.set_object_pose("box", translation(1.0, 2.0, 3.0));
        assert!(matches!(
            world.move_object("box", Isometry3::identity()),
            MoveObjectOutcome::NoChange
        ));
    }

    #[test]
    fn move_object_applies_transform_in_world_frame() {
        let mut world = World::new();
        let original = translation(1.0, 0.0, 0.0);
        world.set_object_pose("box", original);
        let delta = translation(0.0, 1.0, 0.0);
        let MoveObjectOutcome::Moved(notification) = world.move_object("box", delta) else {
            panic!("expected Moved");
        };
        assert_eq!(notification.object.pose(), delta * original);
        assert_eq!(world.get_object("box").unwrap().pose(), delta * original);
    }

    // --- set_object_pose ---

    #[test]
    fn set_object_pose_on_new_id_is_create() {
        let mut world = World::new();
        let notification = world.set_object_pose("box", Isometry3::identity());
        assert!(notification.action.contains(Action::CREATE));
    }

    #[test]
    fn set_object_pose_on_shapeless_existing_object_is_uninitialized_action() {
        let mut world = World::new();
        world.set_object_pose("box", Isometry3::identity());
        let notification = world.set_object_pose("box", translation(1.0, 0.0, 0.0));
        assert_eq!(notification.action, Action::UNINITIALIZED);
    }

    #[test]
    fn set_object_pose_on_populated_existing_object_is_move_shape() {
        let mut world = World::new();
        world
            .add_to_object(
                "box",
                Isometry3::identity(),
                &[sphere(1.0)],
                &[Isometry3::identity()],
            )
            .unwrap();
        let new_pose = translation(1.0, 0.0, 0.0);
        let notification = world.set_object_pose("box", new_pose);
        assert_eq!(notification.action, Action::MOVE_SHAPE);
        let obj = world.get_object("box").unwrap();
        assert_eq!(obj.shapes()[0].global_pose(), new_pose);
    }

    // --- remove_shape_from_object / remove_object / clear_objects ---

    #[test]
    fn remove_shape_from_object_last_shape_destroys_object() {
        let mut world = World::new();
        let shape = sphere(1.0);
        world
            .add_to_object(
                "box",
                Isometry3::identity(),
                &[Arc::clone(&shape)],
                &[Isometry3::identity()],
            )
            .unwrap();
        let notification = world.remove_shape_from_object("box", &shape).unwrap();
        assert_eq!(notification.action, Action::DESTROY);
        assert!(!world.has_object("box"));
    }

    #[test]
    fn remove_shape_from_object_non_last_shape_keeps_object() {
        let mut world = World::new();
        let shape_a = sphere(1.0);
        let shape_b = sphere(2.0);
        world
            .add_to_object(
                "box",
                Isometry3::identity(),
                &[Arc::clone(&shape_a), Arc::clone(&shape_b)],
                &[Isometry3::identity(), Isometry3::identity()],
            )
            .unwrap();
        let notification = world.remove_shape_from_object("box", &shape_a).unwrap();
        assert_eq!(notification.action, Action::REMOVE_SHAPE);
        assert!(world.has_object("box"));
        assert_eq!(world.get_object("box").unwrap().shapes().len(), 1);
    }

    #[test]
    fn remove_shape_from_object_unknown_is_none() {
        let mut world = World::new();
        assert!(
            world
                .remove_shape_from_object("box", &sphere(1.0))
                .is_none()
        );
    }

    #[test]
    fn remove_shape_from_object_unknown_shape_is_none() {
        let mut world = World::new();
        world
            .add_to_object(
                "box",
                Isometry3::identity(),
                &[sphere(1.0)],
                &[Isometry3::identity()],
            )
            .unwrap();
        // Value-equal but a distinct `Arc`: `remove_shape_from_object` matches
        // by `Arc` identity, so this must miss even though it looks the same.
        let other_shape = sphere(1.0);
        assert!(
            world
                .remove_shape_from_object("box", &other_shape)
                .is_none()
        );
        assert!(world.has_object("box"));
        assert_eq!(world.get_object("box").unwrap().shapes().len(), 1);
    }

    #[test]
    fn remove_object_missing_is_none() {
        let mut world = World::new();
        assert!(world.remove_object("box").is_none());
    }

    #[test]
    fn clear_objects_notifies_every_object_in_id_order_and_empties_world() {
        let mut world = World::new();
        world.set_object_pose("b", Isometry3::identity());
        world.set_object_pose("a", Isometry3::identity());
        let notifications = world.clear_objects();
        let ids: Vec<&str> = notifications.iter().map(|n| n.object.id()).collect();
        assert_eq!(ids, vec!["a", "b"]);
        assert!(notifications.iter().all(|n| n.action == Action::DESTROY));
        assert!(world.is_empty());
    }

    // --- set_subframes_of_object ---

    #[test]
    fn set_subframes_of_object_unknown_object_is_false() {
        let mut world = World::new();
        assert!(!world.set_subframes_of_object("box", BTreeMap::new()));
    }

    #[test]
    fn set_subframes_of_object_computes_global_pose_and_replaces_old_ones() {
        let mut world = World::new();
        let object_pose = translation(1.0, 0.0, 0.0);
        world.set_object_pose("box", object_pose);
        let mut first = BTreeMap::new();
        first.insert("old".to_owned(), Isometry3::identity());
        assert!(world.set_subframes_of_object("box", first));

        let mut second = BTreeMap::new();
        let subframe_pose = translation(0.0, 1.0, 0.0);
        second.insert("tip".to_owned(), subframe_pose);
        assert!(world.set_subframes_of_object("box", second));

        let obj = world.get_object("box").unwrap();
        assert_eq!(
            obj.subframe_pose("old"),
            None,
            "old subframes are replaced, not merged"
        );
        assert_eq!(obj.subframe_pose("tip"), Some(subframe_pose));
        assert_eq!(
            obj.global_subframe_pose("tip"),
            Some(object_pose * subframe_pose)
        );
    }

    // --- global_shape_transform(s) ---

    #[test]
    fn global_shape_transform_unknown_object_is_none() {
        let world = World::new();
        assert!(world.global_shape_transform("box", 0).is_none());
    }

    #[test]
    fn global_shape_transform_out_of_range_index_is_none() {
        let mut world = World::new();
        world
            .add_to_object(
                "box",
                Isometry3::identity(),
                &[sphere(1.0)],
                &[Isometry3::identity()],
            )
            .unwrap();
        assert!(world.global_shape_transform("box", 1).is_none());
    }

    #[test]
    fn global_shape_transforms_unknown_object_is_none() {
        let world = World::new();
        assert!(world.global_shape_transforms("box").is_none());
    }

    // --- knows_transform / get_transform / try_get_transform ---

    #[test]
    fn transform_lookup_exact_object_name() {
        let mut world = World::new();
        let pose = translation(1.0, 2.0, 3.0);
        world.set_object_pose("box", pose);
        assert!(world.knows_transform("box"));
        assert_eq!(world.try_get_transform("box"), Some(pose));
        assert_eq!(world.get_transform("box").unwrap(), pose);
    }

    #[test]
    fn transform_lookup_subframe_name() {
        let mut world = World::new();
        let object_pose = translation(1.0, 0.0, 0.0);
        world.set_object_pose("box", object_pose);
        let subframe_pose = translation(0.0, 1.0, 0.0);
        let mut subframes = BTreeMap::new();
        subframes.insert("tip".to_owned(), subframe_pose);
        world.set_subframes_of_object("box", subframes);

        assert!(world.knows_transform("box/tip"));
        let expected = object_pose * subframe_pose;
        assert_eq!(world.try_get_transform("box/tip"), Some(expected));
        assert_eq!(world.get_transform("box/tip").unwrap(), expected);
    }

    #[test]
    fn transform_lookup_unknown_name_errors() {
        let world = World::new();
        assert!(!world.knows_transform("nothing"));
        assert_eq!(world.try_get_transform("nothing"), None);
        assert!(world.get_transform("nothing").is_err());
    }

    /// The documented upstream ambiguity (module docs, deviation 8), pinned
    /// concretely: object `"a"` has no subframe `"b/c"`; object `"a/b"` has a
    /// subframe `"c"`. Querying `"a/b/c"` makes `"a"` the first
    /// prefix-matching candidate for both methods, but only `getTransform`
    /// keeps looking past it.
    #[test]
    fn subframe_name_colliding_with_a_sibling_object_name_is_the_documented_ambiguity() {
        let mut world = World::new();
        world.set_object_pose("a", Isometry3::identity());
        let object_pose = translation(1.0, 0.0, 0.0);
        world.set_object_pose("a/b", object_pose);
        let subframe_pose = translation(0.0, 1.0, 0.0);
        let mut subframes = BTreeMap::new();
        subframes.insert("c".to_owned(), subframe_pose);
        world.set_subframes_of_object("a/b", subframes);

        // knows_transform stops at the first candidate ("a") and reports its
        // (missing) subframe "b/c" — never reaching "a/b".
        assert!(!world.knows_transform("a/b/c"));

        // get_transform/try_get_transform keep going past "a" and resolve
        // through "a/b"'s subframe "c".
        let expected = object_pose * subframe_pose;
        assert_eq!(world.try_get_transform("a/b/c"), Some(expected));
        assert_eq!(world.get_transform("a/b/c").unwrap(), expected);
    }

    // --- World::clone is a COW snapshot, not a deep-independent copy ---

    #[test]
    fn clone_diverges_after_mutation_copy_on_write() {
        let mut world = World::new();
        world.set_object_pose("box", Isometry3::identity());
        let snapshot = world.clone();

        world.set_object_pose("box", translation(1.0, 0.0, 0.0));

        assert_eq!(
            snapshot.get_object("box").unwrap().pose(),
            Isometry3::identity()
        );
        assert_eq!(
            world.get_object("box").unwrap().pose(),
            translation(1.0, 0.0, 0.0)
        );
    }

    #[test]
    fn get_object_snapshot_survives_a_later_mutation() {
        let mut world = World::new();
        world.set_object_pose("box", Isometry3::identity());
        let snapshot = world.get_object("box").unwrap();

        world.set_object_pose("box", translation(1.0, 0.0, 0.0));

        assert_eq!(snapshot.pose(), Isometry3::identity());
        assert_eq!(
            world.get_object("box").unwrap().pose(),
            translation(1.0, 0.0, 0.0)
        );
    }

    // --- all_objects_as_notifications / object_ids ---

    #[test]
    fn all_objects_as_notifications_covers_every_object_in_id_order() {
        let mut world = World::new();
        world.set_object_pose("b", Isometry3::identity());
        world.set_object_pose("a", Isometry3::identity());
        let notifications = world.all_objects_as_notifications(Action::CREATE);
        let ids: Vec<&str> = notifications.iter().map(|n| n.object.id()).collect();
        assert_eq!(ids, vec!["a", "b"]);
        assert_eq!(world.object_ids(), vec!["a".to_owned(), "b".to_owned()]);
    }

    // --- Action bit operations ---

    #[test]
    fn action_default_is_uninitialized() {
        assert_eq!(Action::default(), Action::UNINITIALIZED);
        assert_eq!(Action::default().bits(), 0);
    }

    #[test]
    fn action_contains_and_bitor() {
        let combined = Action::CREATE | Action::ADD_SHAPE;
        assert!(combined.contains(Action::CREATE));
        assert!(combined.contains(Action::ADD_SHAPE));
        assert!(!combined.contains(Action::DESTROY));
    }
}
