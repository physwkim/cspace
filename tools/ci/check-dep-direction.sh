#!/bin/bash
# Enforces PORTING-PLAN.md §3: no crate except `moveit-ros` may depend on a ROS
# client library. D1 makes the core a ROS-independent library; without a
# mechanical check, a single convenience dependency quietly re-couples it and
# the breakage is only noticed when someone tries to build without ROS 2.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

BANNED_RE='^(r2r|r2r_.*|rclrs|ros2-client|rustdds|rosidl_.*)$'
ALLOWED_PACKAGE='moveit-ros'

# Capture the package list up front. A process-substitution feeding `while
# read` swallows the producer's exit status, so a failing `cargo metadata` would
# drive zero iterations and the script would report OK — a false pass that only
# shows up as "CI was green while checking nothing".
if ! metadata="$(cargo metadata --no-deps --format-version 1)"; then
  echo "FAIL cargo metadata failed" >&2
  exit 2
fi
packages="$(python3 -c 'import json,sys; [print(p["name"]) for p in json.load(sys.stdin)["packages"]]' <<<"$metadata")"
if [[ -z "$packages" ]]; then
  echo "FAIL cargo metadata listed no workspace packages" >&2
  exit 2
fi

status=0
while read -r pkg; do
  [[ "$pkg" == "$ALLOWED_PACKAGE" ]] && continue
  # `cargo tree -e normal` omits dev- and build-dependencies: a ROS dep in a
  # dev-dependency would still make `cargo test` need ROS, so include those too.
  # `cargo tree` runs on its own rather than at the head of the pipe, and its
  # stderr is kept: piped, the `|| true` at the tail would turn "cargo tree
  # could not resolve this package" into an empty `hits`, i.e. into "no ROS
  # dependency found" -- a pass for a crate that was never inspected.
  tree_status=0
  tree="$(cargo tree -p "$pkg" -e normal,dev,build --prefix none --no-dedupe)" \
    || tree_status=$?
  if [[ $tree_status -ne 0 ]]; then
    echo "FAIL cargo tree -p $pkg exited $tree_status -- $pkg was not checked" >&2
    status=1
    continue
  fi
  hits="$(awk '{print $1}' <<<"$tree" | sort -u | grep -E "$BANNED_RE" || true)"
  if [[ -n "$hits" ]]; then
    echo "FAIL $pkg depends on a ROS client library:" >&2
    sed 's/^/    /' <<<"$hits" >&2
    status=1
  fi
done <<<"$packages"

if [[ $status -eq 0 ]]; then
  echo "OK: no crate outside $ALLOWED_PACKAGE depends on a ROS client library"
fi
exit $status
