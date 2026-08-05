# Assertion-discrimination ledger — p10-attached-body (`set_scale`/`set_padding`)

The eight tests added with `AttachedBody::set_scale`/`set_padding`
(`crates/moveit-scene/src/attached_body.rs`, upstream `attached_body.cpp:86`
and `:120`), and the six mutations that say what each one discriminates.

None of these tests adds a site to the coarse-assertion sweep's corpus:
`tools/ci/count-coarse-assertions.py crates/moveit-scene/src/attached_body.rs`
prints nothing. That is deliberate rather than lucky — the error case first
read `assert!(err.to_string().contains("Sphere radius must be non-negative"))`,
which the scanner classes coarse, and it was replaced with an `assert_eq!`
on the whole rendered message. The whole message is also the *more*
discriminating assertion: `moveit_error::Error::Construct` is a shared
catch-all (that type's own doc counts 121 sites) and only the message says
which check fired. So this ledger has no rows in
`doc/assertion-discrimination-orphans.txt`'s namespace and no `file:line`
first columns; every row below is a mutation.

## Method

Baseline is the whole crate, not just the new file, so a mutation that
takes down something outside `attached_body.rs` cannot hide:

    cargo nextest run -p moveit-scene        # 100 tests, 100 pass at baseline

Each mutation is applied to `crates/moveit-scene/src/attached_body.rs`,
the suite is run, and the file is restored by `cp` from a pristine copy
taken before the first mutation and confirmed byte-for-byte with `cmp`
(`[restored]` before the next mutation is applied). `git status
--porcelain` at the end shows the one intended modification and nothing
else. M2's first run was cancelled by nextest's fail-fast with one test
unrun, so it was re-run with `--no-fail-fast`; the numbers below are that
complete run.

Where a mutation takes down more than one test, the row says so and names
the family. No row claims exclusivity that a mutation did not show.

## 1. The two branches of upstream's `use_count() == 1`

`Arc::make_mut` is the whole of `apply_to_shapes`' per-shape policy, so
the two branches cannot both be neutralized independently in safe Rust —
there is no safe way to mutate through an `Arc` that another owner holds.
The two mutations below therefore each remove *one* branch and keep the
other, which is what isolates the two claims from each other.

| # | mutation | fails | stays green | verdict |
|---|---|---|---|---|
| M1 | insert `*shape = Arc::new((**shape).clone());` before the `make_mut`, so the in-place branch is never taken | 3 — `an_unshared_shape_is_mutated_in_place_without_a_clone`, `a_scale_of_one_changes_no_dimension_and_no_allocation`, `a_padding_of_zero_changes_no_dimension_and_no_allocation` | the other 97, including both sharing tests | family-of-3 |
| M2 | `Arc::make_mut(shape)` → `Arc::get_mut(shape).expect("M2: the clone branch is gone")`, so the clone branch panics instead of cloning | 2 — `a_shape_shared_with_another_owner_is_cloned_rather_than_mutated_in_place`, `an_outstanding_weak_forces_a_clone_upstreams_use_count_would_not` | the other 98 | family-of-2 |

M1's family is exactly the three tests that assert allocation identity
(`Arc::as_ptr` before == after). That it is a family and not one test is
the point: the two identity boundaries the port must hold — "a real update
of a sole-owned shape reuses the allocation" and "a no-op update does not
churn allocations" — are the same guard seen twice, and M1 shows both move
together. It leaves both sharing tests green, so those are not testing
allocation identity by accident.

M2's family is exactly the two cases that must clone. `Arc::get_mut`
returns `None` for both of them (strong > 1, and weak > 0), which is why
one mutation cannot separate them; the separation comes from the tests'
own assertions instead — the shared test reads the other owner's radius,
the weak test reads `Weak::strong_count`. Both panics quote `M2: the clone
branch is gone` at `attached_body.rs:215`, so the attribution is the
runtime's.

M1 and M2 are each other's negative: neither fails a test the other fails.
That is the evidence that the in-place claim and the must-clone claim are
carried by disjoint tests, which is what "`Arc::make_mut` is upstream's
`use_count() == 1` branch plus its `else`" needs in order to be a tested
claim rather than a comment.

## 2. `set_padding` is not `set_scale`

| # | mutation | fails | stays green | verdict |
|---|---|---|---|---|
| M4 | in `set_padding`, `shape.padd(padding)` → `shape.scale(padding)` | 3 — `a_padding_of_zero_changes_no_dimension_and_no_allocation`, `negative_padding_shrinks_a_shape_that_can_absorb_it`, `negative_padding_larger_than_a_shape_is_rejected_after_updating_its_predecessors` | the other 97, including every `set_scale` test | family-of-3 |

The family is every padding test and no scale test, which is the useful
split: it says the two public methods reach different `moveit_geometry`
entry points, not merely that both do *something*. The three failures also
fail differently — `a_padding_of_zero` gets `left: 0.0, right: 0.25`
(multiplying by zero instead of adding it), `negative_padding_shrinks`
fails at its `expect` (scaling by `-0.375` is rejected outright), and
`negative_padding_larger_...` gets `left: 0.5, right: 0.125` (the failure
moves to the *first* shape, so nothing is applied before it).

## 3. The error path

| # | mutation | fails | stays green | verdict |
|---|---|---|---|---|
| M3 | `update(Arc::make_mut(shape))?` → `let _ = update(Arc::make_mut(shape));`, swallowing the rejection | 1 — `negative_padding_larger_than_a_shape_is_rejected_after_updating_its_predecessors`, at its `expect_err` (`attached_body.rs:393`) | the other 99 | discriminating |
| M5 | make application transactional: snapshot the `Shape` values first (by value, so no strong count is bumped and M1's family stays green), and on error restore them | 1 — the same test, at its `radius(&body.shapes()[0]) == 0.125` assertion (`attached_body.rs:407`), `left: 0.5, right: 0.125` | the other 99 | discriminating |
| M6 | `.map_err(\|_\| moveit_error::Error::other("M6"))` on the per-shape result | 1 — the same test, at its whole-message assertion (`attached_body.rs:394`), `left: "M6"` | the other 99 | discriminating |

M3, M5 and M6 all fail one test and are still three separate rows because
they fail it at three different assertions, which is the only evidence
that the test's three claims are three claims. M3 says the rejection is
propagated at all; M5 says it is propagated *without* a rollback, i.e.
that partial application is the documented contract and not an accident
of nobody having looked; M6 says the propagated error is the shape
layer's own dimension rejection rather than any `Err` at all.

M5 is deliberately written so that it does not disturb M1's family: a
naive rollback that snapshots `Vec<Arc<Shape>>` would bump every strong
count and make `make_mut` clone everywhere, failing the allocation-identity
tests too and collapsing the signal. Snapshotting `Shape` by value keeps
the counts untouched, so the one test that moves is the one that asserts
partial application.

## What is *not* covered

- `a_body_with_no_shapes_accepts_both_and_stays_empty` is moved by none of
  the six. It is a boundary case with no guard of its own: the loop body
  never runs, so every mutation above is unreachable from it. Its job is
  to fail if some future edit gives the empty case a special path — an
  early return, a `debug_assert!(!self.shapes.is_empty())` — and that is a
  guard which does not exist yet, so no mutation of today's code can
  demonstrate it. Recorded here as uncovered rather than claimed.
- The `Weak`-forces-a-clone deviation is *shown* by
  `an_outstanding_weak_forces_a_clone_upstreams_use_count_would_not` and
  its `Weak::strong_count() == 0` was measured, not predicted from std's
  source. But no mutation isolates the `strong_count` assertion from the
  `Arc::as_ptr` one in that same test: both are consequences of the single
  `make_mut` call, and M2 kills the test before either runs.
- Nothing here tests `moveit_geometry::Shape::scale`/`padd` themselves.
  Their own rejection thresholds are that crate's, tested there; this
  ledger only claims which of them each method reaches (M4) and that the
  rejection is propagated whole (M3, M6).
