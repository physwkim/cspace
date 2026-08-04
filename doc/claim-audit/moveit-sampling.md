# Claim audit: moveit-sampling

Per PORTING-PLAN.md §175. One row per item, appended as found, not
batched at report time. `evidence` is the upstream `file:line` actually
opened this round -- inference from the port is not evidence.

Upstream root for this crate: `/home/stevek/work/moveit2` (pinned
`e017c91ee12984393a28ba246075c65f69cde3bf`), the two
`multivariate_gaussian.hpp` files (stomp's and chomp's).

## §172 sweep (round 33), upstream-first per `0ad8c67`

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `moveit2 moveit_planners/stomp/include/stomp_moveit/math/multivariate_gaussian.hpp`, `moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/multivariate_gaussian.{h,hpp}` | No float-derived `int`/`unsigned`/`size_t` narrowing anywhere in either upstream `multivariate_gaussian` header this crate ports (nor its stray `.h` twin, checked though not cited) | CONFIRMED, 0 hits | Full-file grep of all three files, read in this tree; zero matches for `static_cast<int-family>`, C-style int-family casts, or float-initialized int-family declarations | (none) |
| `crates/moveit-sampling/src/*.rs` (port-side anchor: `as i8..u128/usize` receiving `f64`) | Zero occurrences of the anchor pattern anywhere in this crate | CONFIRMED, 0 hits (run, not skipped) | Read in this tree only | (none) |

## §167.6 bare-directory-citation sweep (this round)

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| Every `Ported from` header in this crate (`lib.rs:5`, `multivariate_gaussian.rs:5`) | Neither cites a bare package/directory line with no filenames indented beneath it -- both list the two `multivariate_gaussian.hpp` files explicitly | CONFIRMED, 0 hits of the shape the parser now closes | Read both headers in full in this tree; `tools/ci/verify-upstream-license-provenance.sh` also run over the whole workspace this round: `checked 334 upstream file(s) cited by 242 tracked source file(s)`, 0 findings | (none) |

## §194 port-only API sweep (this round): `moveit-sampling`

This crate's share of the cross-crate sweep triggered by `a682f63`
(full rationale in `moveit-stomp-core`'s own claim-audit doc). This
crate has exactly one public type.

**How "no port-only API exists at all" (row below, prior round) was
established, and why it was checked again this round.** Absence claims
in this project have a bad record -- one fell to a single `rg` earlier
this session ([[absence-claims-need-the-source-opened]]-shaped) -- so
per this round's ask, the exact method, not just the conclusion:

- **Anchor, port side:** `` ^\s*pub fn|^\s*pub struct `` (`rg`,
  `crates/moveit-sampling/src/*.rs`). Five hits: `pub struct
  MultivariateGaussian` (`multivariate_gaussian.rs:69`), `pub fn
  new` (`:79`), `pub fn size` (`:92`), `pub fn
  sample_with_covariance` (`:102`), `pub fn
  sample_without_covariance` (`:113`); plus the `pub use
  multivariate_gaussian::MultivariateGaussian` re-export in
  `lib.rs:61`, which is the same type, not a sixth item.
- **This crate has no single upstream counterpart file** -- it ports
  two independently-maintained classes in two different upstream
  packages (`stomp_moveit::math::MultivariateGaussian` and
  `chomp::MultivariateGaussian`, see the crate's own module doc, "One
  class, two upstream files"). Every port-side hit above was checked
  against **both**, not one:
  - `/home/stevek/work/moveit2/moveit_planners/stomp/include/stomp_moveit/math/multivariate_gaussian.hpp`
    (pinned `e017c91ee12984393a28ba246075c65f69cde3bf`), full `public:`
    section read this round (lines 58-71): a templated constructor and
    one templated `sample(output, use_covariance = true)`. Nothing
    else public; `size_` is a `private:` `int` (line 77).
  - `/home/stevek/work/moveit2/moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/multivariate_gaussian.hpp`,
    same pin, full `public:` section read this round (lines 50-58): a
    templated constructor and one templated `sample(output)` (no
    `use_covariance` parameter). Same layout, `size_` private (line
    64).
  - Both packages' stray `.h` twins (`multivariate_gaussian.h` next to
    each `.hpp` above) opened and confirmed this round to be pure
    `create_deprecated_headers.py`-generated forwarding stubs (`#pragma
    message` + one `#include` of the real `.hpp`, no declarations of
    their own) -- contribute nothing beyond the `.hpp` files already
    checked, consistent with `moveit-stomp-core`'s own `§172` row
    noting the same shape for a different `.h`/`.hpp` pair.
  - `rg -i size` against both `.hpp` files: the only hits are the
    private `size_` member and its two internal uses -- confirms no
    public `size()`/`getSize()` accessor exists in either class, under
    any name, not just the exact spelling this port chose.

**Correction to the prior round's row 33 classification, found by this
re-check:** `size()` was classified `port-only? no` on the claim that
it is an "upstream-faithful ported method." That is wrong -- neither
upstream class has a public size accessor at all (`size_` is
`private:` in both, confirmed above), so `size()` **is** port-only,
the same shape as every other row this crate's sweep is looking for.
It carries no invariant risk for a different reason than "it has an
upstream counterpart": `size()` returns `self.mean.len()`, a read-only
view over the `mean` this port's own caller already passed into
`new(mean, covariance)` -- it exposes no state the caller did not
already possess by construction, so there is no upstream-private state
newly reachable through it, unlike `Stomp::with_cancel_handle`'s
pre-construction `proceed` window.

| API | port-only? | upstream state newly reachable | invariant at risk? |
|---|---|---|---|
| `MultivariateGaussian::new(mean, covariance)` | no -- direct port of `stomp_moveit::math::MultivariateGaussian`/`chomp::MultivariateGaussian`'s constructor, with a stated deviation (see this crate's own "Deviation: construction can fail") | none -- the deviation moves in the *safe* direction: upstream's constructor always succeeds and silently produces `NaN`-sampling state for a non-positive-definite `covariance`; this port's `Option`-returning constructor makes that state *unconstructable* instead of opening anything new | no -- the opposite pattern from `with_cancel_handle`: this narrows what was reachable upstream |
| `sample_with_covariance`, `sample_without_covariance` | no -- upstream-faithful ported methods (`sample(output, true)`/`sample(output, false)` split into two names, see this crate's own "Deviation: two named methods" doc) over already-validated state | none | no |
| `size()` | **yes -- corrected this round, was misclassified "no" in the prior pass.** No public `size()`/`size_` accessor exists in either upstream class; `size_` is `private:` in both | a read of `mean.len()` -- state the caller already supplied to `new` and therefore already possessed | no -- read-only, over caller-already-known state, not upstream-private state |

**Conclusion (corrected):** one port-only API exists in this crate
(`size()`, not zero), found by this re-check to have been
misclassified previously. It carries no invariant risk: it is a
read-only accessor over state the caller already owns, not a new path
to anything upstream kept structurally unreachable. The prior round's
headline -- "no port-only API exists in this crate at all" -- was
false as stated; "no port-only API in this crate carries invariant
risk" is the claim that actually holds, and is the corrected headline
here.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-sampling/src/multivariate_gaussian.rs`, every `pub fn`/`pub struct` (re-check) | `size()` is port-only (prior round's "no" was wrong); it carries no invariant risk. `new`'s deviation still narrows reachable state rather than expanding it | CONFIRMED, 5 items re-enumerated and classified above, 1 corrected | Read every public item in this tree; read the full `public:` section of both upstream `.hpp` files and confirmed both `.h` twins are deprecation stubs, this round | (pending, see report) |
