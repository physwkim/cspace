#!/bin/bash
# Builds the moveit-rs oracle image.
#
# The build context is assembled from `git archive` exports rather than copies:
# it pins each tree to its committed HEAD (a dirty working tree cannot leak into
# an oracle that is supposed to be the reference), and it keeps .git out of the
# context so the daemon is not handed hundreds of megabytes of history.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MOVEIT2_SRC="${MOVEIT2_SRC:-$HOME/work/moveit2}"
MOVEIT2_SHA="${MOVEIT2_SHA:-e017c91ee12984393a28ba246075c65f69cde3bf}"

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"

# Tagged by source digest, not `:latest`. Several worktrees share this docker
# daemon, and more than one of them edits the oracle's C++ at a time; a single
# mutable tag means whoever built last is silently everyone's oracle, and a
# sweep running across robots can have the image swapped under it between two
# of them.
SRC_DIGEST="$(oracle_src_digest "$REPO_ROOT/tools/moveit-oracle")"
IMAGE="${IMAGE:-$(oracle_image_tag "$SRC_DIGEST")}"

have_sha="$(git -C "$MOVEIT2_SRC" rev-parse HEAD)"
if [[ "$have_sha" != "$MOVEIT2_SHA" ]]; then
  echo "moveit2 at $MOVEIT2_SRC is $have_sha, expected $MOVEIT2_SHA" >&2
  echo "check it out, or override with MOVEIT2_SHA=<sha> to re-pin deliberately" >&2
  exit 1
fi

# Same filesystem as the repo so the export is a plain local write.
CTX="$(mktemp -d "$REPO_ROOT/.oracle-ctx.XXXXXX")"
trap 'rm -rf "$CTX"' EXIT
# Ctrl-C during the (long) build would otherwise kill the shell without
# running the EXIT trap; exiting from the INT handler routes it through.
trap 'exit 130' INT

export_tree() {  # <src repo> <dest name>
  mkdir -p "$CTX/$2"
  git -C "$1" archive HEAD | tar -x -C "$CTX/$2"
}

export_tree "$MOVEIT2_SRC"                        moveit2
export_tree "$REPO_ROOT/third_party/moveit_msgs"  moveit_msgs
export_tree "$REPO_ROOT/third_party/moveit_resources" moveit_resources
cp -a "$REPO_ROOT/tools/moveit-oracle" "$CTX/moveit-oracle"

echo "context: $(du -sh "$CTX" | cut -f1)"
# No pipe here: piping docker build into tail/tee masks its exit status behind
# the last stage of the pipeline, which turns a failed build into a silent
# success for any caller that checks $?.
#
# Not `exec` either: exec replaces this shell, so the EXIT trap above never
# runs and $CTX -- a full moveit2 + moveit_resources export, ~100 MB each --
# is left behind by every build, successful ones included. `set -e` already
# propagates a failed build's status without exec's help.
docker build ${TARGET:+--target "$TARGET"} -t "$IMAGE" -f "$CTX/moveit-oracle/Dockerfile" "$CTX"
