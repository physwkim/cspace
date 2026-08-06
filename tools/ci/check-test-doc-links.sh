#!/usr/bin/env bash
# Fails if a bracket-style intra-doc link (`` [`Ident`] `` / `` [`Ident`](path) ``)
# inside a `crates/*/tests/*.rs` integration test file names an identifier
# this check cannot find anywhere it can plausibly be defined.
#
# `verify-private-doc-links.sh` closed half of PORTING-PLAN.md §198's gap:
# `cargo doc --document-private-items` reaches every private item, but
# `cargo doc` has no target-selection flag for a `tests/*.rs` integration
# target at all (`cargo doc --help` lists `--lib`/`--bins`/`--examples`, no
# `--test`/`--tests`), and `RUSTDOCFLAGS="--cfg test"` on the lib target
# fails to link dev-dependencies, as that script's own comment already
# records. So every test file's own `//!`/`///` doc comments -- 337 bracket
# links across this workspace's `tests/*.rs` files at the time this was
# written -- are checked by nothing: not the public-item doc gate, not
# `verify-private-doc-links.sh`, not `cargo test` (a bracket link is not a
# doctest), not `cargo clippy` (link resolution is a rustdoc lint, not a
# clippy one).
#
# This is not rustdoc. It cannot resolve a path the way rustdoc does --
# disambiguators, glob imports, trait-method inheritance, prelude items. It
# resolves exactly three things, deliberately narrow so a gap it cannot
# check reads as a gap, not as a pass:
#
#   1. A link with no `::` (`` [`EPSILON`] ``) or one rooted at `crate`/
#      `Self`/`self`/`super`/the crate's own name is checked against that
#      one crate's own tracked `.rs` files (src/ and tests/ together, so a
#      link from one test file to a helper or sibling test function in
#      another test file of the same crate still resolves).
#   2. A `Type::member` link whose `Type` the same file imports by name
#      (`use other_crate::Type;` or `use other_crate::{Type, ...};`) is
#      checked against THAT crate instead -- `` [`MeshSearchPaths::none`] ``
#      appears in over 30 test files across 9 crates that import it from
#      moveit-model, and treating every one of those as a same-crate link
#      would fail on all of them. This parser is line-based and flat: a
#      multi-line `use a::{B, C};` group is read as one statement, but a
#      nested group (`use a::{b::C, D}`) is not descended into, and `as`
#      aliases are not tracked, so an aliased or nested-group import falls
#      through to rule 3 instead.
#   3. A `Type::member` link whose `Type` is not imported at all (true for
#      most: something defined in the same file needs no `use`) is checked
#      against the current crate, same as rule 1.
#   4. Anything else -- a path rooted at a lowercase identifier that is
#      neither the crate's own name nor an import target -- is assumed to
#      name the standard library or an unrecognized shape and is SKIPPED,
#      counted, and printed, never silently. This trades missing some real
#      breakage for not flagging a legitimate `` [`std::sync::Arc`] `` as
#      broken, which would make the gate noisy enough to stop being read.
#
# A target crate is "found in" by any of three tiers: a definition-shaped
# occurrence (`fn`/`struct`/`enum`/`trait`/`type`/`const`/`static`/`mod` NAME)
# anywhere in that crate; an enum-variant-shaped line (NAME at the start of a
# line, other than whitespace, followed by `(`, `{`, or `,` -- `Code(...)`,
# `Code {`, `Code,`) -- because the first regex only matches the `enum Foo`
# line, never the variants inside its body, and a variant referenced solely
# from another crate's doc comment (e.g. `` [`moveit_error::Error::Code`] ``
# from a pilz test, never restated anywhere in moveit-error's own src/tests)
# has no second occurrence for the next tier to catch; or -- because neither
# regex can see associated consts or method call sites -- the bare identifier
# appearing at least once more in that crate beyond the link's own text.
# None of the three is exact; together they are still strictly narrower than
# "the identifier exists as *something*", which is what makes a renamed or
# removed item's stale link fail rather than coincidentally re-match
# unrelated text.
#
# Named `check-*` so `ci.yml`'s glob runs it: this needs nothing but python3
# and the file itself -- no docker, no cargo, no ROS. The `#[cfg(test)] mod
# tests` blocks embedded in `src/*.rs` (as opposed to separate `tests/*.rs`
# files) are NOT covered by this script -- those are still only reachable by
# building the *lib* target with `--cfg test`, which is exactly the
# dev-dependency-linking failure `verify-private-doc-links.sh` documents and
# neither script works around. Do not read a green run here as covering them.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$(dirname "${BASH_SOURCE[0]}")/gate-lib.sh"
require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

mapfile -t test_files < <(git ls-files --deduplicate -- 'crates/*/tests/*.rs' | sort)
require_nonempty "${#test_files[@]}" "crates/*/tests/*.rs file"

mapfile -t all_crate_files < <(git ls-files --deduplicate -- 'crates/*/*.rs' 'crates/*/**/*.rs' | sort)
require_nonempty "${#all_crate_files[@]}" "crates/*/**/*.rs file"

python3 - "$REPO_ROOT" "${#test_files[@]}" "${test_files[@]}" "${#all_crate_files[@]}" "${all_crate_files[@]}" <<'PY'
import re
import sys

repo_root = sys.argv[1]
idx = 2
n_test = int(sys.argv[idx]); idx += 1
test_files = sys.argv[idx:idx + n_test]; idx += n_test
n_all = int(sys.argv[idx]); idx += 1
all_crate_files = sys.argv[idx:idx + n_all]; idx += n_all

LINK_RE = re.compile(r'\[`([^`]+)`\](?:\(([^)]+)\))?')
DEF_RE_TEMPLATE = r'\b(?:pub(?:\([^)]*\))?\s+)?(?:fn|struct|enum|trait|type|const|static|mod)\s+{}\b'
# An enum variant's own line: `Code(MoveItErrorCode),` / `Code {` / `Code,`.
# `enum Foo { ... }`'s DEF_RE_TEMPLATE hit only ever matches the `enum Foo`
# line, never a variant inside its body -- this is what makes a variant
# resolvable from a single cross-crate doc-link occurrence instead of
# needing a second, coincidental mention of its name elsewhere in the crate.
VARIANT_RE_TEMPLATE = r'(?m)^[ \t]*{}[ \t]*[(,{{]'
SELF_ROOTS = {"crate", "Self", "self", "super"}
# Flat `use a::b::{X, Y as Z};` or `use a::b::X;`. Deliberately does not
# match a nested group (`{x::Y, Z}`) -- those items are left unmapped and
# fall back to same-crate resolution, per this script's own header comment.
USE_GROUP_RE = re.compile(r'use\s+((?:crate|self|super|[A-Za-z_][A-Za-z0-9_]*)(?:::[A-Za-z0-9_]+)*)::\{([^{}]*)\};')
USE_SINGLE_RE = re.compile(r'use\s+((?:crate|self|super|[A-Za-z_][A-Za-z0-9_]*)(?:::[A-Za-z0-9_]+)*)::([A-Za-z_][A-Za-z0-9_]*)\s*(?:as\s+[A-Za-z_][A-Za-z0-9_]*)?;')

def crate_dir_of(path):
    parts = path.split("/")
    assert parts[0] == "crates", path
    return parts[1]

def crate_ident_of(crate_dir):
    return crate_dir.replace("-", "_")

crate_dirs = sorted({crate_dir_of(f) for f in all_crate_files})
dir_by_ident = {crate_ident_of(d): d for d in crate_dirs}

crate_corpus = {}
for crate_dir in crate_dirs:
    parts = []
    for f in all_crate_files:
        if crate_dir_of(f) == crate_dir:
            with open(f"{repo_root}/{f}", encoding="utf-8") as handle:
                parts.append(handle.read())
    crate_corpus[crate_dir] = "\n".join(parts)

def resolved_in(crate_dir, tail):
    corpus = crate_corpus.get(crate_dir)
    if corpus is None:
        return None  # unknown crate dir (external to this workspace) -- can't check
    def_re = re.compile(DEF_RE_TEMPLATE.format(re.escape(tail)))
    if def_re.search(corpus):
        return True
    variant_re = re.compile(VARIANT_RE_TEMPLATE.format(re.escape(tail)))
    if variant_re.search(corpus):
        return True
    occurrences = len(re.findall(r'\b' + re.escape(tail) + r'\b', corpus))
    return occurrences >= 2

def parse_imports(text):
    """Map local type name -> defining crate_dir, for names imported from
    another crate (`crate`/`self`/`super`-rooted imports map to no entry,
    since those already resolve same-crate by default)."""
    imports = {}

    def record(mod_path, name):
        head = mod_path.split("::")[0]
        if head in SELF_ROOTS:
            return
        crate_dir = dir_by_ident.get(head)
        if crate_dir is not None:
            imports[name] = crate_dir

    for m in USE_GROUP_RE.finditer(text):
        mod_path, body = m.group(1), m.group(2)
        for item in body.split(","):
            item = item.strip()
            if not item or "::" in item:
                continue  # nested group item -- not descended into, see header
            name = item.split(" as ")[0].strip()
            if re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', name):
                record(mod_path, name)

    for m in USE_SINGLE_RE.finditer(text):
        mod_path, name = m.group(1), m.group(2)
        record(mod_path, name)

    return imports

failures = []
skipped = 0
checked = 0

for path in test_files:
    crate_dir = crate_dir_of(path)
    crate_ident = crate_ident_of(crate_dir)

    with open(f"{repo_root}/{path}", encoding="utf-8") as handle:
        text = handle.read()
    lines = text.split("\n")
    imports = parse_imports(text)

    for lineno, line in enumerate(lines, start=1):
        for m in LINK_RE.finditer(line):
            display, explicit_path = m.group(1), m.group(2)
            target = explicit_path if explicit_path else display
            target = re.sub(r'(\(\))?!?$', '', target).strip()
            if not target or not re.match(r'^[A-Za-z_][A-Za-z0-9_:]*$', target):
                continue

            segments = target.split("::")
            head = segments[0] if len(segments) > 1 else None
            tail = segments[-1]

            if head is None:
                target_crate = crate_dir
            elif head in SELF_ROOTS or head == crate_ident:
                target_crate = crate_dir
            elif head in imports:
                target_crate = imports[head]
            elif head in dir_by_ident:
                # A fully-qualified path into another workspace crate, not
                # necessarily `use`-imported by name (the doc comment just
                # spelled out the whole path), e.g.
                # `` [`moveit_geometry::compound_from_octree`] ``.
                target_crate = dir_by_ident[head]
            elif head[:1].isupper():
                # Not imported from elsewhere -- assume locally defined.
                target_crate = crate_dir
            else:
                skipped += 1
                continue

            checked += 1
            found = resolved_in(target_crate, tail)
            if found is False:
                failures.append((path, lineno, target, target_crate))
            # found is None (unknown crate_dir) never happens here: every
            # target_crate value above comes from dir_by_ident or crate_dir,
            # both already keys of crate_corpus.

print(f"checked {checked} link(s), skipped {skipped} unresolved-prefix link(s), "
      f"across {len(test_files)} test file(s) in {len(crate_dirs)} crate(s)")

if failures:
    print(f"FAIL {len(failures)} link(s) name an identifier not found in the crate they resolve to:", file=sys.stderr)
    for path, lineno, target, target_crate in failures:
        print(f"  {path}:{lineno}: [`{target}`] -- not found anywhere in crates/{target_crate}", file=sys.stderr)
    sys.exit(1)

print("OK every resolvable bracket link in crates/*/tests/*.rs resolves to something in its target crate")
PY
