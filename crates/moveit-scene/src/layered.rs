// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! [`Layered`]: the single mechanism [`crate::PlanningScene`] uses to answer
//! "is this mine, or my parent's?" — see the crate-level doc for why this
//! exists instead of upstream's `std::optional<T>` fields.

/// A value a [`crate::PlanningScene`] either owns locally, or defers to its
/// parent for.
///
/// # Deviation from upstream
///
/// Upstream stores each of `robot_state_`, `acm_` (and others this port does
/// not carry yet, e.g. `scene_transforms_`) as `std::optional<T>`, and
/// duplicates the same ternary at every accessor: `field_.has_value() ?
/// field_.value() : parent_->getField()`. That is the "implicit value plus a
/// flag, re-derived ad hoc at every read site" shape this project treats as
/// a defect source elsewhere: nothing stops one accessor's ternary from
/// getting the polarity backwards, or a new accessor from forgetting the
/// parent fallthrough entirely, and the mistake would surface only as a
/// child scene silently reading its own (absent) default instead of the
/// parent's real value.
///
/// `Layered<T>` does not remove the fallthrough — a child scene's whole
/// reason to exist is to defer to its parent until it diverges, and per
/// upstream's own doc comment on [`crate::PlanningScene::diff`] that really
/// is resolved live, at read time: "if changes to these are made in the
/// parent they will be visible in the child" (until the child's own copy is
/// materialized). What `Layered<T>` removes is the fallthrough's
/// *multiplicity*: it is a generic method, [`Layered::resolve`], written and
/// reasoned about exactly once, and every [`crate::PlanningScene`] accessor
/// is a one-line call through it rather than its own hand-rolled ternary.
/// "Mine or inherited?" is answered by matching this type's two variants —
/// not by a caller remembering to check a `bool`/`Option` correctly every
/// time it reads a field.
#[derive(Debug, Clone)]
pub(crate) enum Layered<T> {
    /// This scene owns `T` locally; the parent is not consulted.
    Own(T),
    /// This scene defers to its parent for this value.
    Inherited,
}

impl<T> Layered<T> {
    /// The resolved value: `self`'s own value if owned, else whatever
    /// `parent` produces. `parent` is only invoked for
    /// [`Layered::Inherited`], so an owned value never pays for (or
    /// requires) a parent closure at all.
    pub(crate) fn resolve<'a>(&'a self, parent: impl FnOnce() -> &'a T) -> &'a T {
        match self {
            Layered::Own(value) => value,
            Layered::Inherited => parent(),
        }
    }

    /// Whether this scene owns the value locally (vs. deferring to its
    /// parent).
    pub(crate) fn is_own(&self) -> bool {
        matches!(self, Layered::Own(_))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn own_resolves_to_itself_without_calling_the_parent_closure() {
        let layered = Layered::Own(5);
        let resolved = layered.resolve(|| panic!("parent must not be consulted for Own"));
        assert_eq!(*resolved, 5);
    }

    #[test]
    fn inherited_resolves_through_the_parent_closure() {
        let layered: Layered<i32> = Layered::Inherited;
        let parent_value = 7;
        let resolved = layered.resolve(|| &parent_value);
        assert_eq!(*resolved, 7);
    }

    #[test]
    fn is_own_distinguishes_the_two_variants() {
        assert!(Layered::Own(1).is_own());
        assert!(!Layered::<i32>::Inherited.is_own());
    }
}
