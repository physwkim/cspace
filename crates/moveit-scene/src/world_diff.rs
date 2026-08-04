// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection/include/moveit/collision_detection/world_diff.hpp
//   moveit_core/collision_detection/src/world_diff.cpp

//! [`WorldDiff`]: an accumulated record of what changed in a
//! [`moveit_collision::World`] ([`Action`] bits per object id).
//!
//! # Deviation from upstream
//!
//! Upstream `WorldDiff` is a live observer: it subscribes to a `World` via
//! `World::addObserver` and accumulates whatever `Action` the world reports
//! through that callback for as long as it stays subscribed
//! (`setWorld`/the constructor taking a `WorldPtr` both call
//! `world->addObserver(...)`; the destructor and `reset()` unsubscribe).
//! `moveit_collision::World` deliberately has no observer mechanism at all —
//! every mutator returns the [`Notification`] it produced instead of pushing
//! it through a callback (see that crate's `world` module docs, "deviation
//! 4"). `WorldDiff` here is the same idea one level up: a pure accumulator
//! with [`WorldDiff::record`] as its one write path, fed explicit
//! [`Notification`]s by its caller rather than a live subscription. It holds
//! no reference to any `World` at all, so there is nothing to subscribe or
//! unsubscribe, and no lifetime to manage.

use std::collections::BTreeMap;

use moveit_collision::{Action, Notification, World};

/// A change record over a [`World`]: which objects changed and how, since
/// whatever baseline the owner chooses — see [`WorldDiff::record`] and
/// [`WorldDiff::set_world`].
///
/// Upstream `collision_detection::WorldDiff`.
#[derive(Debug, Clone, Default)]
pub struct WorldDiff {
    changes: BTreeMap<String, Action>,
}

impl WorldDiff {
    /// An empty diff. Upstream's default constructor (no world).
    pub fn new() -> Self {
        Self::default()
    }

    /// Fold one [`Notification`] into the diff. Upstream's private `notify`:
    /// [`Action::DESTROY`] unconditionally *replaces* whatever was recorded
    /// for this id (a destroyed object's earlier CREATE/MOVE_SHAPE/...
    /// history stops mattering); any other action is OR'd into the existing
    /// bits. Because of this asymmetry, an id that is destroyed and then
    /// recreated within the same diff (as [`WorldDiff::set_world`] can
    /// produce, for an id present in both the old and new world) ends up
    /// with a combined `DESTROY | CREATE | ...` mask, not a fresh `CREATE`
    /// alone — matching upstream's own `notify` exactly, not a
    /// simplification of it.
    pub fn record(&mut self, notification: &Notification) {
        let id = notification.object.id();
        if notification.action == Action::DESTROY {
            self.changes.insert(id.to_owned(), notification.action);
        } else {
            *self.changes.entry(id.to_owned()).or_default() |= notification.action;
        }
    }

    /// [`WorldDiff::record`] for every notification, in order.
    pub fn record_all<'a>(&mut self, notifications: impl IntoIterator<Item = &'a Notification>) {
        for notification in notifications {
            self.record(notification);
        }
    }

    /// Upstream `setWorld`: as if this diff had been watching `old` (if any)
    /// up to now, and switches to watching `new` from this point on —
    /// *without* clearing whatever was already recorded. Synthesizes a
    /// `DESTROY` for every object `old` currently holds, then a `CREATE |
    /// ADD_SHAPE` for every object `new` currently holds, each folded
    /// through [`WorldDiff::record`] exactly as a live observer would have
    /// seen them via upstream's `notifyObserverAllObjects`.
    ///
    /// An id present in both `old` and `new` therefore ends up
    /// `DESTROY | CREATE | ADD_SHAPE` in [`WorldDiff::changes`], not
    /// "unchanged" — upstream's real, if surprising, behavior (see
    /// [`WorldDiff::record`]'s doc), reproduced deliberately rather than
    /// smoothed over.
    pub fn set_world(&mut self, old: Option<&World>, new: &World) {
        if let Some(old) = old {
            self.record_all(&old.all_objects_as_notifications(Action::DESTROY));
        }
        self.record_all(&new.all_objects_as_notifications(Action::CREATE | Action::ADD_SHAPE));
    }

    /// Discard every recorded change. Upstream `reset()`/`clearChanges()`,
    /// unified: this port has no live subscription for `reset()` to also
    /// unsubscribe from ([`WorldDiff::set_world`] is the closest analogue),
    /// so the two upstream methods collapse to the same operation here.
    pub fn clear_changes(&mut self) {
        self.changes.clear();
    }

    /// The accumulated change for `id`, if any. Upstream `find`.
    pub fn get(&self, id: &str) -> Option<Action> {
        self.changes.get(id).copied()
    }

    /// Every recorded change, by object id, in id order. Upstream
    /// `getChanges`/`begin`/`end`.
    pub fn changes(&self) -> &BTreeMap<String, Action> {
        &self.changes
    }

    /// The number of objects with a recorded change. Upstream `size`.
    pub fn len(&self) -> usize {
        self.changes.len()
    }

    /// Whether [`WorldDiff::len`] is zero.
    pub fn is_empty(&self) -> bool {
        self.changes.is_empty()
    }

    /// Set (replacing, not OR-ing) the recorded change for `id`; removes the
    /// entry entirely if `action` is [`Action::UNINITIALIZED`] (no bits
    /// set). Upstream `set`.
    pub fn set(&mut self, id: &str, action: Action) {
        if action == Action::UNINITIALIZED {
            self.changes.remove(id);
        } else {
            self.changes.insert(id.to_owned(), action);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use moveit_geometry::{Isometry3, Shape, Sphere};

    use super::*;

    fn sphere_shape() -> Arc<Shape> {
        Arc::new(Shape::Sphere(Sphere::new(1.0).unwrap()))
    }

    // ---- record: add vs remove vs move-only --------------------------------

    #[test]
    fn recording_an_add_notification_sets_the_add_shape_bit() {
        let mut world = World::new();
        let notification = world
            .add_shape("a", sphere_shape(), Isometry3::identity())
            .unwrap();
        let mut diff = WorldDiff::new();
        diff.record(&notification);
        assert!(diff.get("a").unwrap().contains(Action::CREATE));
        assert!(diff.get("a").unwrap().contains(Action::ADD_SHAPE));
    }

    #[test]
    fn recording_a_remove_notification_sets_only_the_destroy_bit() {
        let mut world = World::new();
        world
            .add_shape("a", sphere_shape(), Isometry3::identity())
            .unwrap();
        let removal = world.remove_object("a").unwrap();
        let mut diff = WorldDiff::new();
        diff.record(&removal);
        assert_eq!(diff.get("a").unwrap(), Action::DESTROY);
    }

    #[test]
    fn recording_a_move_only_notification_sets_only_move_shape() {
        let mut world = World::new();
        world
            .add_shape("a", sphere_shape(), Isometry3::identity())
            .unwrap();
        let mut diff = WorldDiff::new();
        // A fresh diff started only after the object already existed: only
        // the subsequent move is recorded, not the earlier creation.
        let moved = world.set_object_pose("a", Isometry3::translation(1.0, 0.0, 0.0));
        diff.record(&moved);
        assert_eq!(diff.get("a").unwrap(), Action::MOVE_SHAPE);
    }

    // ---- record: DESTROY overwrites, everything else ORs -------------------

    #[test]
    fn destroy_after_add_shape_overwrites_rather_than_ors() {
        let mut world = World::new();
        let add = world
            .add_shape("a", sphere_shape(), Isometry3::identity())
            .unwrap();
        let mut diff = WorldDiff::new();
        diff.record(&add);
        assert!(diff.get("a").unwrap().contains(Action::CREATE));

        let destroy = world.remove_object("a").unwrap();
        diff.record(&destroy);
        // Pure DESTROY, not DESTROY | CREATE | ADD_SHAPE: the overwrite rule.
        assert_eq!(diff.get("a").unwrap(), Action::DESTROY);
    }

    #[test]
    fn add_shape_after_move_shape_ors_the_bits_together() {
        let mut world = World::new();
        world
            .add_shape("a", sphere_shape(), Isometry3::identity())
            .unwrap();
        let mut diff = WorldDiff::new();
        let moved = world.set_object_pose("a", Isometry3::translation(1.0, 0.0, 0.0));
        diff.record(&moved);
        let added = world
            .add_shape_to_object(
                "a",
                Isometry3::identity(),
                sphere_shape(),
                Isometry3::identity(),
            )
            .unwrap();
        diff.record(&added);
        let combined = diff.get("a").unwrap();
        assert!(combined.contains(Action::MOVE_SHAPE));
        assert!(combined.contains(Action::ADD_SHAPE));
    }

    // ---- set_world -----------------------------------------------------------

    #[test]
    fn set_world_from_none_synthesizes_create_add_shape_for_every_new_object() {
        let mut new_world = World::new();
        new_world
            .add_shape("a", sphere_shape(), Isometry3::identity())
            .unwrap();
        new_world
            .add_shape("b", sphere_shape(), Isometry3::identity())
            .unwrap();

        let mut diff = WorldDiff::new();
        diff.set_world(None, &new_world);

        assert_eq!(diff.len(), 2);
        for id in ["a", "b"] {
            let action = diff.get(id).unwrap();
            assert!(action.contains(Action::CREATE));
            assert!(action.contains(Action::ADD_SHAPE));
        }
    }

    #[test]
    fn set_world_synthesizes_destroy_for_every_object_only_in_the_old_world() {
        let mut old_world = World::new();
        old_world
            .add_shape("gone", sphere_shape(), Isometry3::identity())
            .unwrap();
        let new_world = World::new();

        let mut diff = WorldDiff::new();
        diff.set_world(Some(&old_world), &new_world);

        assert_eq!(diff.get("gone").unwrap(), Action::DESTROY);
    }

    #[test]
    fn set_world_for_an_id_in_both_worlds_combines_destroy_with_create() {
        // The subtle upstream case this module's doc calls out: an id
        // present in both old and new worlds is not "unchanged" — it is
        // DESTROY (synthesized for the old world) OR'd with CREATE |
        // ADD_SHAPE (synthesized for the new world), because `record`'s
        // DESTROY-overwrite rule only applies to the single notification
        // being folded in, not retroactively to bits already present.
        let mut old_world = World::new();
        old_world
            .add_shape("stays", sphere_shape(), Isometry3::identity())
            .unwrap();
        let mut new_world = World::new();
        new_world
            .add_shape("stays", sphere_shape(), Isometry3::identity())
            .unwrap();

        let mut diff = WorldDiff::new();
        diff.set_world(Some(&old_world), &new_world);

        let combined = diff.get("stays").unwrap();
        assert!(combined.contains(Action::DESTROY));
        assert!(combined.contains(Action::CREATE));
        assert!(combined.contains(Action::ADD_SHAPE));
    }

    #[test]
    fn set_world_does_not_clear_changes_already_recorded() {
        let mut diff = WorldDiff::new();
        diff.set("preexisting", Action::CREATE);
        diff.set_world(None, &World::new());
        assert!(diff.get("preexisting").is_some());
    }

    // ---- set / clear_changes --------------------------------------------------

    #[test]
    fn set_replaces_rather_than_ors_an_existing_entry() {
        let mut diff = WorldDiff::new();
        diff.set("a", Action::CREATE | Action::ADD_SHAPE);
        diff.set("a", Action::MOVE_SHAPE);
        assert_eq!(diff.get("a").unwrap(), Action::MOVE_SHAPE);
    }

    #[test]
    fn set_with_uninitialized_action_erases_the_entry() {
        let mut diff = WorldDiff::new();
        diff.set("a", Action::CREATE);
        diff.set("a", Action::UNINITIALIZED);
        assert!(diff.get("a").is_none());
    }

    #[test]
    fn clear_changes_empties_the_diff() {
        let mut diff = WorldDiff::new();
        diff.set("a", Action::CREATE);
        diff.clear_changes();
        assert!(diff.is_empty());
        assert_eq!(diff.len(), 0);
    }

    #[test]
    fn a_fresh_diff_is_empty() {
        let diff = WorldDiff::new();
        assert!(diff.is_empty());
        assert!(diff.changes().is_empty());
    }
}
