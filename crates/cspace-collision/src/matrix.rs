// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_matrix.hpp
//   moveit_core/collision_detection/src/collision_matrix.cpp

//! Which pairs of named bodies may touch: upstream
//! `collision_detection::AllowedCollisionMatrix` and the
//! `AllowedCollision::Type`/`DecideContactFn` vocabulary around it.
//!
//! # Declaration audit — `collision_matrix.hpp` / `collision_matrix.cpp`
//!
//! Every public declaration in `collision_matrix.hpp`, each with a
//! disposition. Written for the reason `world`'s equivalent section states:
//! "ported" is otherwise a file-level claim, and
//! `doc/declaration-audit-coverage.md` measures how far that claim reaches
//! across the tree.
//!
//! `tools/ci/count-public-declarations.sh collision_matrix.hpp
//! AllowedCollisionMatrix` prints **29**. The 29 bullets below were
//! enumerated by hand from `collision_matrix.hpp:82-247` and then checked
//! against that count.
//!
//! ## The collapse this audit is easiest to read against
//!
//! Upstream stores an entry's *type* and its *predicate* in two parallel
//! maps (`entries_`/`allowed_contacts_`, and `default_entries_`/
//! `default_allowed_contacts_` for defaults), so nearly every accessor comes
//! in two overloads: one filling an `AllowedCollision::Type&`, one filling a
//! `DecideContactFn&`. [`AllowedCollision`] is one sum type carrying the
//! predicate only in the variant that has one, so each upstream *pair* of
//! overloads is one method here plus [`AllowedCollision::kind`] or
//! [`AllowedCollision::predicate`] at the call site. That is why the 29
//! below map onto fewer Rust methods without anything being dropped.
//!
//! ## `class AllowedCollisionMatrix`, 29 declarations
//!
//! 1. `AllowedCollisionMatrix()` → [`AllowedCollisionMatrix::new`].
//! 2. `AllowedCollisionMatrix(names, allowed = false)` →
//!    [`AllowedCollisionMatrix::from_names`]; the C++ default argument
//!    becomes a required parameter, Rust having no default arguments.
//! 3. `AllowedCollisionMatrix(const srdf::Model&)` →
//!    [`AllowedCollisionMatrix::from_srdf`].
//! 4. `AllowedCollisionMatrix(const moveit_msgs::msg::AllowedCollisionMatrix&)`
//!    → **unported, in scope, assigned elsewhere.** D6/§4.3 put every
//!    `moveit_msgs` conversion in `cspace-ros` as a `TryFrom`, and it is not
//!    written yet: `ros/cspace-ros/src/scene/planning_scene.rs:19-24` names
//!    `allowed_collision_matrix` in its own list of `PlanningScene` message
//!    fields still unconverted. Expires when that conversion lands.
//! 5. `AllowedCollisionMatrix(const AllowedCollisionMatrix&) = default` →
//!    `#[derive(Clone)]`.
//! 6. `operator=(const AllowedCollisionMatrix&) = default` → `Clone` plus
//!    ordinary assignment; Rust has no separate copy-assignment operator to
//!    port.
//! 7. `getEntry(name1, name2, AllowedCollision::Type&)` →
//!    [`AllowedCollisionMatrix::entry`] + [`AllowedCollision::kind`].
//! 8. `getEntry(name1, name2, DecideContactFn&)` →
//!    [`AllowedCollisionMatrix::entry`] + [`AllowedCollision::predicate`].
//! 9. `hasEntry(name)` → [`AllowedCollisionMatrix::has_entry`].
//! 10. `hasEntry(name1, name2)` →
//!     [`AllowedCollisionMatrix::has_pair_entry`].
//! 11. `removeEntry(name1, name2)` →
//!     [`AllowedCollisionMatrix::remove_entry`].
//! 12. `removeEntry(name)` →
//!     [`AllowedCollisionMatrix::remove_entries_for`].
//! 13. `setEntry(name1, name2, bool)` →
//!     [`AllowedCollisionMatrix::set_entry`].
//! 14. `setEntry(name1, name2, DecideContactFn&)` →
//!     [`AllowedCollisionMatrix::set_conditional_entry`].
//! 15. `setEntry(name, bool)` →
//!     [`AllowedCollisionMatrix::set_entry_for_known`]. Named for what it
//!     does — pair `name` with the names the matrix already knows — because
//!     upstream's own doc comment warns that the known set changes under the
//!     caller and recommends `setDefaultEntry` instead.
//! 16. `setEntry(name, other_names, bool)` →
//!     [`AllowedCollisionMatrix::set_entry_with`].
//! 17. `setEntry(names1, names2, bool)` →
//!     [`AllowedCollisionMatrix::set_entry_between`].
//! 18. `setEntry(bool)` → [`AllowedCollisionMatrix::set_all_entries`].
//! 19. `getAllEntryNames(std::vector<std::string>&)` →
//!     [`AllowedCollisionMatrix::all_entry_names`], returning the vector
//!     instead of filling an out-parameter.
//! 20. `getMessage(moveit_msgs::msg::AllowedCollisionMatrix&)` →
//!     **unported, in scope, assigned elsewhere** — the same D6/§4.3 routing
//!     and the same expiry as 4.
//! 21. `clear()` → [`AllowedCollisionMatrix::clear`].
//! 22. `getSize()` → [`AllowedCollisionMatrix::len`], plus
//!     [`AllowedCollisionMatrix::is_empty`] which Rust convention requires
//!     alongside `len` and upstream has no counterpart for.
//! 23. `setDefaultEntry(name, bool)` →
//!     [`AllowedCollisionMatrix::set_default_entry`].
//! 24. `setDefaultEntry(name, DecideContactFn&)` →
//!     [`AllowedCollisionMatrix::set_default_conditional_entry`].
//! 25. `getDefaultEntry(name, AllowedCollision::Type&)` →
//!     [`AllowedCollisionMatrix::default_entry`] +
//!     [`AllowedCollision::kind`].
//! 26. `getDefaultEntry(name, DecideContactFn&)` →
//!     [`AllowedCollisionMatrix::default_entry`] +
//!     [`AllowedCollision::predicate`].
//! 27. `getAllowedCollision(name1, name2, DecideContactFn&)` →
//!     [`AllowedCollisionMatrix::allowed_collision`] +
//!     [`AllowedCollision::predicate`].
//! 28. `getAllowedCollision(name1, name2, AllowedCollision::Type&)` →
//!     [`AllowedCollisionMatrix::allowed_collision`] +
//!     [`AllowedCollision::kind`]. Upstream's two `getAllowedCollision`
//!     overloads can disagree with each other; see
//!     [`AllowedCollision::combine_defaults`]'s doc comment for the case and
//!     why collapsing them makes it unrepresentable.
//! 29. `print(std::ostream&)` → **decided-non-port**; see
//!     [`AllowedCollisionMatrix`]'s own doc comment for the measurement
//!     behind that and the condition that re-opens it.
//!
//! ## File-level declarations outside the class, 3
//!
//! 1. `namespace AllowedCollision { enum Type }` →
//!    [`AllowedCollisionType`]. The namespace-wrapped enum is a pre-`enum
//!    class` idiom for scoping the three names; a Rust enum scopes its
//!    variants already.
//! 2. `using DecideContactFn` → [`DecideContactFn`], `Arc<dyn Fn>` where
//!    upstream has `std::function`. `Send + Sync` are required because an
//!    [`AllowedCollisionMatrix`] travels between threads in this port;
//!    `std::function` carries no such bound and upstream relies on
//!    convention.
//! 3. `MOVEIT_CLASS_FORWARD(AllowedCollisionMatrix)` → **decided-non-port.**
//!    The macro's `Ptr`/`ConstPtr`/`WeakPtr` aliases exist so callers can
//!    share one matrix; a caller here owns the value or wraps it at the use
//!    site, and a fixed alias in this module would decide that for them.
//!
//! ## Private declarations and `collision_matrix.cpp`
//!
//! One private declaration, `getDefaultEntry(name1, name2,
//! AllowedCollision::Type&)`, is
//! [`AllowedCollision::combine_defaults`] here.
//!
//! `collision_matrix.cpp` adds two file-local declarations to the header's
//! list: an anonymous-namespace `getLogger()` returning an `rclcpp::Logger`
//! (`:46-52`), excluded by D1; and `static bool andDecideContact(f1, f2,
//! contact)` (`:296`), which is not excluded — it is the AND of two
//! predicates, ported inside [`AllowedCollision::combine_defaults`] as the
//! closure over both `Arc`s. Every other definition in the file implements a
//! declaration listed above.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::sync::Arc;

use cspace_core::srdf::SrdfModel;

use crate::common::Contact;

/// Upstream `collision_detection::AllowedCollision::Type`: which of the three
/// outcomes an [`AllowedCollision`] entry represents, without its predicate.
///
/// This is the return value of [`AllowedCollision::kind`], not a second
/// storage location for it — unlike upstream, there is nowhere a `kind()` and
/// the actual predicate can disagree, because [`AllowedCollision::Conditional`]
/// is the only variant that carries a predicate at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AllowedCollisionType {
    /// The pair is never allowed to collide.
    Never,
    /// The pair is always allowed to collide.
    Always,
    /// Whether the pair is allowed depends on the produced [`Contact`]; see
    /// [`AllowedCollision::predicate`].
    Conditional,
}

/// Upstream `collision_detection::DecideContactFn`, i.e.
/// `std::function<bool(Contact&)>`.
///
/// `Arc` (not `Box`) because [`AllowedCollisionMatrix::set_entry`] mirrors one
/// call into two map cells (`name1`→`name2` and `name2`→`name1`) that must
/// share the same predicate, and because
/// [`AllowedCollisionMatrix::allowed_collision`] combines two defaults into a
/// fresh predicate that closes over both without re-cloning the closures
/// themselves. `Send + Sync` so an `AllowedCollisionMatrix` can be shared
/// across a multi-threaded planner the way upstream's copyable `std::function`
/// can.
pub type DecideContactFn = Arc<dyn Fn(&mut Contact) -> bool + Send + Sync>;

/// One collision-matrix entry: whether a pair (or a name's default) may
/// collide, and — only when that answer is conditional — the predicate that
/// decides it.
///
/// # Deviation from upstream
///
/// Upstream splits this across two parallel maps, `entries_: map<string,
/// map<string, AllowedCollision::Type>>` and `allowed_contacts_: map<string,
/// map<string, DecideContactFn>>`, keyed identically. A cell's predicate is
/// only meaningful when the parallel `entries_` cell reads `CONDITIONAL`; nothing
/// stops the two maps from disagreeing (a stale `allowed_contacts_` entry left
/// behind after `entries_` is overwritten with `NEVER`/`ALWAYS` by a bare
/// `setEntry(name1, name2, bool)`), and PORTING-PLAN.md §4.1/§4.3 name exactly
/// this class of defect. Here the predicate lives *inside* the `Conditional`
/// variant, so [`AllowedCollisionMatrix::set_entry`] overwriting a cell with
/// `Never`/`Always` structurally cannot leave a stale predicate — there is no
/// second map for it to survive in.
#[derive(Clone)]
pub enum AllowedCollision {
    /// `AllowedCollision::Type::NEVER`.
    Never,
    /// `AllowedCollision::Type::ALWAYS`.
    Always,
    /// `AllowedCollision::Type::CONDITIONAL`, with the predicate that decides
    /// it. Upstream's `CONDITIONAL` can only be constructed by
    /// `setEntry`/`setDefaultEntry`'s `DecideContactFn` overload, which always
    /// supplies a predicate at the same time; this variant makes that
    /// pairing hold by construction instead of by discipline.
    Conditional(DecideContactFn),
}

impl fmt::Debug for AllowedCollision {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Never => f.write_str("Never"),
            Self::Always => f.write_str("Always"),
            Self::Conditional(_) => f.write_str("Conditional(..)"),
        }
    }
}

impl AllowedCollision {
    /// `allowed ? AllowedCollision::ALWAYS : AllowedCollision::NEVER` — the
    /// conversion every `bool`-taking upstream `setEntry`/`setDefaultEntry`
    /// overload performs on its way into storage.
    pub fn from_bool(allowed: bool) -> Self {
        if allowed { Self::Always } else { Self::Never }
    }

    /// The classification alone, with no predicate attached.
    pub fn kind(&self) -> AllowedCollisionType {
        match self {
            Self::Never => AllowedCollisionType::Never,
            Self::Always => AllowedCollisionType::Always,
            Self::Conditional(_) => AllowedCollisionType::Conditional,
        }
    }

    /// The predicate, if this is [`AllowedCollision::Conditional`].
    pub fn predicate(&self) -> Option<&DecideContactFn> {
        match self {
            Self::Conditional(f) => Some(f),
            Self::Never | Self::Always => None,
        }
    }

    /// `andDecideContact` combined with the two-default merge rule from
    /// `AllowedCollisionMatrix::getDefaultEntry(name1, name2, Type&)`: `NEVER`
    /// wins outright: if either side is `NEVER`, the pair is `NEVER` and
    /// nothing needs a predicate, since [`AllowedCollision::Never`] carries
    /// none. Otherwise, if either side is `CONDITIONAL`, the result is
    /// `CONDITIONAL`; when *both* sides are, upstream's
    /// `getAllowedCollision(name1, name2, DecideContactFn&)` ANDs the two
    /// predicates (`andDecideContact`), which this reproduces by closing over
    /// both `Arc`s. When only one side is `CONDITIONAL`, its predicate is
    /// used as-is — upstream's fn-overload only ever finds a default predicate
    /// for a name whose type is `CONDITIONAL`, so "the other side has no
    /// predicate to AND with" here matches upstream exactly. Otherwise both
    /// sides are `ALWAYS`.
    ///
    /// # Deviation from upstream
    ///
    /// Upstream's `getDefaultEntry(name1, name2, Type&)` and
    /// `getAllowedCollision(name1, name2, DecideContactFn&)` are two
    /// independent algorithms that can disagree: if `name1`'s default is
    /// `NEVER` and `name2`'s is `CONDITIONAL(f)`, the `Type` overload reports
    /// `NEVER` (its "`NEVER` wins" rule looks only at `Type`) while the
    /// `DecideContactFn` overload still reports `f` as a usable predicate
    /// (`getDefaultEntry(name1, fn1)` fails since `name1` is not
    /// `CONDITIONAL`, but `getDefaultEntry(name2, fn2)` succeeds, so the
    /// overload returns `found = true, fn = f` — a predicate for a pair its
    /// own `Type` query calls unconditionally disallowed). A caller that
    /// queried `Type` first and only fetched the predicate when `Type ==
    /// CONDITIONAL`, as every real upstream caller does, never observes this.
    /// The unified [`AllowedCollision`] here cannot represent "`Never` but
    /// also here is a predicate", so this combinator returns [`Self::Never`]
    /// in that case and the inconsistent predicate is unreachable — which is
    /// the point of collapsing the two upstream maps into one sum type.
    fn combine_defaults(a: &Self, b: &Self) -> Self {
        match (a, b) {
            (Self::Never, _) | (_, Self::Never) => Self::Never,
            (Self::Conditional(f1), Self::Conditional(f2)) => {
                let f1 = Arc::clone(f1);
                let f2 = Arc::clone(f2);
                Self::Conditional(Arc::new(move |contact: &mut Contact| {
                    f1(contact) && f2(contact)
                }))
            }
            (Self::Conditional(f), _) | (_, Self::Conditional(f)) => {
                Self::Conditional(Arc::clone(f))
            }
            (Self::Always, Self::Always) => Self::Always,
        }
    }
}

/// Which pairs of named bodies are allowed to collide.
///
/// Upstream `collision_detection::AllowedCollisionMatrix`. All elements are
/// referred to by name, exactly as upstream documents; nothing here checks
/// that a name actually exists in any particular robot or world.
///
/// # Deviation from upstream
///
/// The message-based constructor (`AllowedCollisionMatrix(const
/// moveit_msgs::msg::AllowedCollisionMatrix&)`) and `getMessage()` are not
/// ported: both round-trip a ROS message type, which PORTING-PLAN.md §4.3
/// confines to `cspace-ros`'s `TryFrom` layer, not the core crate.
///
/// `print()` is not ported either, but for its own reason and not that one.
/// It does no logging at all: `collision_matrix.cpp:428-491` writes an ASCII
/// table — index header rows, then one row per name with a `01?`/`-`
/// indicator per pair — to the `std::ostream&` its caller supplies, and
/// touches nothing else. It has zero callers in the pinned upstream checkout
/// (`rg '\.print\(|->print\('` finds six call sites, none of them on an
/// `AllowedCollisionMatrix`). A Rust equivalent would be a `Display` impl on
/// this type; nothing in this port has asked for one. This re-opens the day
/// a caller here wants to dump a matrix for a human to read.
#[derive(Debug, Clone, Default)]
pub struct AllowedCollisionMatrix {
    entries: BTreeMap<String, BTreeMap<String, AllowedCollision>>,
    defaults: BTreeMap<String, AllowedCollision>,
}

impl AllowedCollisionMatrix {
    /// An empty matrix. Upstream's default constructor.
    pub fn new() -> Self {
        Self::default()
    }

    /// Every pair among `names` (including a name with itself, since upstream
    /// loops `j` from `i`, not `i + 1`) set to `allowed`.
    ///
    /// Upstream `AllowedCollisionMatrix(const std::vector<std::string>&, bool)`.
    /// Upstream's `allowed` parameter defaults to `false`; Rust has no default
    /// arguments, so it is required here.
    pub fn from_names(names: &[String], allowed: bool) -> Self {
        let mut acm = Self::default();
        for i in 0..names.len() {
            for j in i..names.len() {
                acm.set_entry(&names[i], &names[j], allowed);
            }
        }
        acm
    }

    /// Build from an SRDF's collision-default sections, in the exact order
    /// upstream applies them: defaults from `disable_default_collisions`
    /// first, then `enable_collisions` re-enabling specific pairs, then
    /// `disable_collisions` disabling specific pairs *last* — so a pair named
    /// in both wins as disabled, matching upstream's comment ("*finally*
    /// disable selected collision pairs").
    ///
    /// Upstream `AllowedCollisionMatrix(const srdf::Model&)`.
    pub fn from_srdf(srdf: &SrdfModel) -> Self {
        let mut acm = Self::default();
        for name in srdf.no_default_collision_links() {
            acm.set_default_entry(name, true);
        }
        for pair in srdf.enabled_collision_pairs() {
            acm.set_entry(&pair.link1, &pair.link2, false);
        }
        for pair in srdf.disabled_collision_pairs() {
            acm.set_entry(&pair.link1, &pair.link2, true);
        }
        acm
    }

    /// The single owner of every write to `entries`: mirrors `value` into
    /// both `name1`→`name2` and `name2`→`name1`. Every `set_*` method funnels
    /// through this — including [`AllowedCollisionMatrix::set_entry`] and
    /// [`AllowedCollisionMatrix::set_conditional_entry`], the two upstream
    /// overloads that write a pair directly — so the matrix can never end up
    /// with a pair recorded in one direction and not the other.
    fn set_pair(&mut self, name1: &str, name2: &str, value: AllowedCollision) {
        self.entries
            .entry(name1.to_owned())
            .or_default()
            .insert(name2.to_owned(), value.clone());
        self.entries
            .entry(name2.to_owned())
            .or_default()
            .insert(name1.to_owned(), value);
    }

    /// Set the entry for a pair to `AllowedCollision::Never`/`Always`.
    ///
    /// Upstream `setEntry(const std::string&, const std::string&, bool)`. Also
    /// covers upstream's "remove function pointers, if any" step: overwriting
    /// the cell replaces any previous [`AllowedCollision::Conditional`]
    /// wholesale, so there is no separate predicate map to clean up.
    pub fn set_entry(&mut self, name1: &str, name2: &str, allowed: bool) {
        self.set_pair(name1, name2, AllowedCollision::from_bool(allowed));
    }

    /// Set the entry for a pair to `AllowedCollision::Conditional(f)`.
    ///
    /// Upstream `setEntry(const std::string&, const std::string&,
    /// DecideContactFn&)`.
    pub fn set_conditional_entry(&mut self, name1: &str, name2: &str, f: DecideContactFn) {
        self.set_pair(name1, name2, AllowedCollision::Conditional(f));
    }

    /// Pair `name` with every element of `other_names` (skipping `name`
    /// itself, if present in the list).
    ///
    /// Upstream `setEntry(const std::string&, const std::vector<std::string>&,
    /// bool)`.
    pub fn set_entry_with(&mut self, name: &str, other_names: &[String], allowed: bool) {
        for other in other_names {
            if other != name {
                self.set_entry(other, name, allowed);
            }
        }
    }

    /// Pair every element of `names1` with every element of `names2`.
    ///
    /// Upstream `setEntry(const std::vector<std::string>&, const
    /// std::vector<std::string>&, bool)`.
    pub fn set_entry_between(&mut self, names1: &[String], names2: &[String], allowed: bool) {
        for name1 in names1 {
            self.set_entry_with(name1, names2, allowed);
        }
    }

    /// Pair `name` with every other name already known to the matrix (i.e.
    /// every existing row of `entries`).
    ///
    /// Upstream `setEntry(const std::string&, bool)`.
    ///
    /// # Deviation from upstream
    ///
    /// Upstream loops `entries_` directly with a `last`-name guard alongside
    /// the `name != entry.first` check, defending against `setEntry(name,
    /// entry.first, allowed)` inserting a brand-new `entries_[name]` row
    /// mid-iteration and that row later being revisited by the same
    /// `std::map` range-for (since `entries_` is keyed by name and a fresh
    /// `name` key can land ahead of the iterator). `entries_`'s keys are
    /// unique, so once `entry.first != name` is excluded, `last` never excludes
    /// anything further — it is a vestigial guard. Rust's borrow checker
    /// forbids mutating `entries` while iterating it at all, so this takes a
    /// snapshot of the current keys first; that snapshot cannot contain a row
    /// inserted by this same call, which is a strict superset of what upstream's
    /// two guards already excluded, so the set of pairs written is identical.
    pub fn set_entry_for_known(&mut self, name: &str, allowed: bool) {
        let known: Vec<String> = self
            .entries
            .keys()
            .filter(|k| k.as_str() != name)
            .cloned()
            .collect();
        for other in known {
            self.set_entry(name, &other, allowed);
        }
    }

    /// Set every already-known pair to `AllowedCollision::Never`/`Always`.
    ///
    /// Upstream `setEntry(bool)`.
    pub fn set_all_entries(&mut self, allowed: bool) {
        let value = AllowedCollision::from_bool(allowed);
        for row in self.entries.values_mut() {
            for cell in row.values_mut() {
                *cell = value.clone();
            }
        }
    }

    /// The explicit entry for a pair, ignoring defaults.
    ///
    /// Upstream `getEntry(const std::string&, const std::string&,
    /// AllowedCollision::Type&)` and the `DecideContactFn&` overload, unified:
    /// both upstream overloads look up the same cell, one reading its `Type`
    /// and the other its paired predicate, so one lookup returning the whole
    /// [`AllowedCollision`] covers both.
    pub fn entry(&self, name1: &str, name2: &str) -> Option<&AllowedCollision> {
        self.entries.get(name1)?.get(name2)
    }

    /// Whether `name` has at least one explicit pair entry.
    ///
    /// Upstream `hasEntry(const std::string&)`.
    pub fn has_entry(&self, name: &str) -> bool {
        self.entries.contains_key(name)
    }

    /// Whether the pair `(name1, name2)` has an explicit entry.
    ///
    /// Upstream `hasEntry(const std::string&, const std::string&)`.
    pub fn has_pair_entry(&self, name1: &str, name2: &str) -> bool {
        self.entries
            .get(name1)
            .is_some_and(|row| row.contains_key(name2))
    }

    /// Remove the entry for a pair, in both directions. A no-op if it is not
    /// present.
    ///
    /// Upstream `removeEntry(const std::string&, const std::string&)`.
    pub fn remove_entry(&mut self, name1: &str, name2: &str) {
        if let Some(row) = self.entries.get_mut(name1) {
            row.remove(name2);
        }
        if let Some(row) = self.entries.get_mut(name2) {
            row.remove(name1);
        }
    }

    /// Remove every entry that mentions `name`: its own row, and its cell in
    /// every other row.
    ///
    /// Upstream `removeEntry(const std::string&)`.
    pub fn remove_entries_for(&mut self, name: &str) {
        self.entries.remove(name);
        for row in self.entries.values_mut() {
            row.remove(name);
        }
    }

    /// Set the default for `name` to `AllowedCollision::Never`/`Always`.
    ///
    /// Upstream `setDefaultEntry(const std::string&, bool)`. Overwriting the
    /// default replaces any previous [`AllowedCollision::Conditional`]
    /// wholesale, covering upstream's `default_allowed_contacts_.erase(name)`.
    pub fn set_default_entry(&mut self, name: &str, allowed: bool) {
        self.defaults
            .insert(name.to_owned(), AllowedCollision::from_bool(allowed));
    }

    /// Set the default for `name` to `AllowedCollision::Conditional(f)`.
    ///
    /// Upstream `setDefaultEntry(const std::string&, DecideContactFn&)`.
    pub fn set_default_conditional_entry(&mut self, name: &str, f: DecideContactFn) {
        self.defaults
            .insert(name.to_owned(), AllowedCollision::Conditional(f));
    }

    /// The default entry for a single name, if one was set.
    ///
    /// Upstream `getDefaultEntry(const std::string&, AllowedCollision::Type&)`
    /// and the `DecideContactFn&` overload, unified for the same reason as
    /// [`AllowedCollisionMatrix::entry`].
    pub fn default_entry(&self, name: &str) -> Option<&AllowedCollision> {
        self.defaults.get(name)
    }

    /// The default entry for a name, combined for a specific `(name1,
    /// name2)` pair per [`AllowedCollision::combine_defaults`].
    fn default_for_pair(&self, name1: &str, name2: &str) -> Option<AllowedCollision> {
        match (self.default_entry(name1), self.default_entry(name2)) {
            (None, None) => None,
            (Some(a), None) => Some(a.clone()),
            (None, Some(b)) => Some(b.clone()),
            (Some(a), Some(b)) => Some(AllowedCollision::combine_defaults(a, b)),
        }
    }

    /// The allowed-collision answer for a pair: the explicit entry if one
    /// exists, [`AllowedCollisionMatrix::entry`], else the combined default
    /// for the two names (`default_for_pair`/`combine_defaults`, both
    /// private: they are internal helpers with no reason to be exposed on
    /// their own).
    ///
    /// Upstream `getAllowedCollision(const std::string&, const std::string&,
    /// AllowedCollision::Type&)` and the `DecideContactFn&` overload, unified.
    /// This is the precedence order upstream's `bool found = getEntry(...) ||
    /// getDefaultEntry(...)` encodes: explicit entry first, per-element
    /// default second — there is no third, "global" default; a query naming a
    /// pair with no explicit entry and no default on either side returns
    /// [`None`], exactly as upstream's `getAllowedCollision` returns `false`.
    pub fn allowed_collision(&self, name1: &str, name2: &str) -> Option<AllowedCollision> {
        if let Some(entry) = self.entry(name1, name2) {
            return Some(entry.clone());
        }
        self.default_for_pair(name1, name2)
    }

    /// Every name known to the matrix: the union of names with at least one
    /// explicit entry and names with a default entry, sorted.
    ///
    /// Upstream `getAllEntryNames`, which builds the same union by starting
    /// from `entries_`'s keys (already sorted, `std::map`) and inserting each
    /// `default_entries_` key via `lower_bound` if not already present —
    /// equivalent to a sorted set union, which [`BTreeSet`] gives directly.
    pub fn all_entry_names(&self) -> Vec<String> {
        let mut names: BTreeSet<String> = self.entries.keys().cloned().collect();
        names.extend(self.defaults.keys().cloned());
        names.into_iter().collect()
    }

    /// The number of rows in the explicit-entry table (i.e. the number of
    /// distinct names with at least one explicit pair entry — *not* the
    /// number of pairs).
    ///
    /// Upstream `getSize`, which returns `entries_.size()`: `entries_` is
    /// keyed one row per name, so this matches upstream exactly, including
    /// the "rows, not pairs" counting.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether [`AllowedCollisionMatrix::len`] is zero.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Remove every entry and every default.
    ///
    /// Upstream `clear`.
    pub fn clear(&mut self) {
        self.entries.clear();
        self.defaults.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn always_true() -> DecideContactFn {
        Arc::new(|_: &mut Contact| true)
    }

    fn always_false() -> DecideContactFn {
        Arc::new(|_: &mut Contact| false)
    }

    // ---- explicit entry vs default vs "not found at all" ------------------

    #[test]
    fn explicit_entry_takes_precedence_over_default() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_default_entry("a", true); // default: Always
        acm.set_entry("a", "b", false); // explicit: Never
        assert_eq!(
            acm.allowed_collision("a", "b").unwrap().kind(),
            AllowedCollisionType::Never
        );
    }

    #[test]
    fn falls_back_to_per_element_default_when_no_explicit_entry() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_default_entry("a", true);
        assert_eq!(
            acm.allowed_collision("a", "b").unwrap().kind(),
            AllowedCollisionType::Always
        );
    }

    #[test]
    fn neither_explicit_nor_default_is_not_found() {
        let acm = AllowedCollisionMatrix::new();
        assert!(acm.allowed_collision("a", "b").is_none());
        assert!(acm.entry("a", "b").is_none());
        assert!(acm.default_entry("a").is_none());
    }

    #[test]
    fn default_never_wins_over_default_always_on_the_other_name() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_default_entry("a", false); // Never
        acm.set_default_entry("b", true); // Always
        assert_eq!(
            acm.allowed_collision("a", "b").unwrap().kind(),
            AllowedCollisionType::Never
        );
    }

    // ---- CONDITIONAL, with and without a predicate -------------------------

    #[test]
    fn conditional_entry_carries_its_predicate() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_conditional_entry("a", "b", always_true());
        let entry = acm.entry("a", "b").unwrap();
        assert_eq!(entry.kind(), AllowedCollisionType::Conditional);
        let mut contact = test_contact();
        assert!(entry.predicate().unwrap()(&mut contact));
    }

    #[test]
    fn never_and_always_carry_no_predicate() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("a", "b", true);
        acm.set_entry("a", "c", false);
        assert!(acm.entry("a", "b").unwrap().predicate().is_none());
        assert!(acm.entry("a", "c").unwrap().predicate().is_none());
    }

    #[test]
    fn overwriting_a_conditional_entry_with_bool_drops_the_predicate() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_conditional_entry("a", "b", always_true());
        acm.set_entry("a", "b", true);
        let entry = acm.entry("a", "b").unwrap();
        assert_eq!(entry.kind(), AllowedCollisionType::Always);
        assert!(entry.predicate().is_none());
    }

    #[test]
    fn combining_two_conditional_defaults_ands_their_predicates() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_default_conditional_entry("a", always_true());
        acm.set_default_conditional_entry("b", always_false());
        let combined = acm.allowed_collision("a", "b").unwrap();
        assert_eq!(combined.kind(), AllowedCollisionType::Conditional);
        let mut contact = test_contact();
        assert!(!combined.predicate().unwrap()(&mut contact));
    }

    #[test]
    fn combining_one_conditional_default_with_an_always_default_uses_it_unchanged() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_default_conditional_entry("a", always_false());
        acm.set_default_entry("b", true); // Always
        let combined = acm.allowed_collision("a", "b").unwrap();
        assert_eq!(combined.kind(), AllowedCollisionType::Conditional);
        let mut contact = test_contact();
        assert!(!combined.predicate().unwrap()(&mut contact));
    }

    // ---- remove-then-lookup -------------------------------------------------

    #[test]
    fn remove_entry_then_lookup_falls_back_to_default() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_default_entry("a", true);
        acm.set_entry("a", "b", false);
        acm.remove_entry("a", "b");
        assert!(acm.entry("a", "b").is_none());
        assert_eq!(
            acm.allowed_collision("a", "b").unwrap().kind(),
            AllowedCollisionType::Always
        );
    }

    #[test]
    fn remove_entry_is_symmetric() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("a", "b", true);
        acm.remove_entry("b", "a");
        assert!(acm.entry("a", "b").is_none());
        assert!(acm.entry("b", "a").is_none());
    }

    #[test]
    fn remove_entries_for_name_clears_its_row_and_every_cell_naming_it() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("a", "b", true);
        acm.set_entry("a", "c", true);
        acm.set_entry("b", "c", true);
        acm.remove_entries_for("a");
        assert!(!acm.has_entry("a"));
        assert!(acm.entry("b", "a").is_none());
        assert!(acm.entry("c", "a").is_none());
        // Unrelated pair survives.
        assert!(acm.has_pair_entry("b", "c"));
    }

    #[test]
    fn removing_a_pair_that_does_not_exist_is_a_silent_no_op() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("a", "b", true);
        acm.remove_entry("x", "y");
        assert!(acm.has_pair_entry("a", "b"));
    }

    // ---- all-vs-all overwrite ------------------------------------------------

    #[test]
    fn set_entry_between_pairs_every_combination() {
        let mut acm = AllowedCollisionMatrix::new();
        let names1 = vec!["a".to_owned(), "b".to_owned()];
        let names2 = vec!["c".to_owned(), "d".to_owned()];
        acm.set_entry_between(&names1, &names2, true);
        for n1 in &names1 {
            for n2 in &names2 {
                assert_eq!(
                    acm.entry(n1, n2).unwrap().kind(),
                    AllowedCollisionType::Always,
                    "{n1}-{n2}"
                );
            }
        }
        // Only the cross product was set, not within a group.
        assert!(acm.entry("a", "b").is_none());
        assert!(acm.entry("c", "d").is_none());
    }

    #[test]
    fn set_all_entries_overwrites_every_existing_pair_but_adds_none() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("a", "b", true);
        acm.set_entry("a", "c", false);
        acm.set_all_entries(false);
        assert_eq!(
            acm.entry("a", "b").unwrap().kind(),
            AllowedCollisionType::Never
        );
        assert_eq!(
            acm.entry("a", "c").unwrap().kind(),
            AllowedCollisionType::Never
        );
        assert!(acm.entry("a", "d").is_none());
    }

    #[test]
    fn set_entry_for_known_pairs_name_with_every_other_existing_row_but_not_itself() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("b", "c", true);
        acm.set_entry_for_known("a", true);
        assert!(acm.has_pair_entry("a", "b"));
        assert!(acm.has_pair_entry("a", "c"));
        // "a" itself was not a known row before the call, so it cannot be
        // paired with itself.
        assert!(acm.entry("a", "a").is_none());
    }

    #[test]
    fn set_entry_for_known_excludes_the_name_even_when_it_is_already_a_known_row() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("b", "c", true);
        // Give "a" its own row before the call, unlike the sibling test above:
        // the snapshot `set_entry_for_known` takes of existing rows now
        // contains "a" itself, so this exercises the `!= name` exclusion
        // rather than vacuously passing because "a" was never in the
        // snapshot to begin with.
        acm.set_entry("a", "z", true);
        acm.set_entry_for_known("a", true);
        assert!(acm.entry("a", "a").is_none());
    }

    #[test]
    fn from_names_pairs_every_name_with_itself_and_every_other_name() {
        let names = vec!["a".to_owned(), "b".to_owned()];
        let acm = AllowedCollisionMatrix::from_names(&names, true);
        assert_eq!(
            acm.entry("a", "a").unwrap().kind(),
            AllowedCollisionType::Always
        );
        assert_eq!(
            acm.entry("a", "b").unwrap().kind(),
            AllowedCollisionType::Always
        );
    }

    // ---- getAllEntryNames / getSize -----------------------------------------

    #[test]
    fn all_entry_names_is_the_sorted_union_of_explicit_and_default_names() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("z", "a", true);
        acm.set_default_entry("m", true);
        assert_eq!(acm.all_entry_names(), vec!["a", "m", "z"]);
    }

    #[test]
    fn len_counts_rows_not_pairs() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("a", "b", true);
        acm.set_entry("a", "c", true);
        // Two rows ("a" and "b" and "c" each got a row via set_pair), not
        // "one pair".
        assert_eq!(acm.len(), 3);
        assert!(!acm.is_empty());
    }

    #[test]
    fn clear_removes_entries_and_defaults() {
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("a", "b", true);
        acm.set_default_entry("a", true);
        acm.clear();
        assert!(acm.is_empty());
        assert!(acm.default_entry("a").is_none());
    }

    fn test_contact() -> Contact {
        Contact::default()
    }
}
