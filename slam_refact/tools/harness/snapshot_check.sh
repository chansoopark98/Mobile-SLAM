#!/bin/bash
# RM-0 snapshot rule check: does an extracted `git archive <commit>` snapshot directory hold exactly the
# tracked content of that commit? Compares git tree shas, so no bytes are copied and nothing in the repo or in
# the snapshot is modified (a throwaway index file under $TMPDIR is the only thing written).
#
# Usage:
#   snapshot_check.sh SNAPSHOT_DIR [COMMIT]          check SNAPSHOT_DIR against COMMIT (default HEAD)
#   snapshot_check.sh --find N SNAPSHOT_DIR          list which of the last N commits (first-parent walk
#                                                    from HEAD) has a tree equal to the snapshot
# Env: REPO (default: the repo containing this script).
#
# Method: read the commit's tree into a temporary index, `git add -u` from the snapshot directory as work tree
# (updates every tracked path from the snapshot's bytes, drops paths missing there), `git write-tree`, and
# compare with `git rev-parse COMMIT^{tree}`. Files in the snapshot that the commit does not track (for
# example the `build` symlink that the build scripts add) are listed as EXTRA and do not change the verdict.
#
# What a MATCH proves: the snapshot's tracked files are byte-identical (git blob sha1, file mode) to COMMIT.
# What it does not prove: when the snapshot was made, who made it, that EXTRA files are harmless, or that the
# builds that used it were run on it. The commit sha is the only time anchor (a commit can be rewritten).
# Exit: 0 MATCH, 1 MISMATCH, 2 usage/environment error.
set -u
REPO="${REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
GIT=(git --git-dir="$REPO/.git")

snap_tree() {  # $1 = snapshot dir, $2 = commit; prints snapshot tree sha
  local idx; idx="$(mktemp "${TMPDIR:-/tmp}/snapshot_check_idx.XXXXXX")" || return 2
  rm -f "$idx"
  GIT_INDEX_FILE="$idx" "${GIT[@]}" read-tree "$2^{tree}" || { rm -f "$idx"; return 2; }
  GIT_INDEX_FILE="$idx" "${GIT[@]}" --work-tree="$1" add -u >/dev/null 2>&1 || true
  if [ -n "${SNAP_DIFF:-}" ]; then
    GIT_INDEX_FILE="$idx" "${GIT[@]}" diff-index --cached --name-status "$2^{tree}" > "$SNAP_DIFF"
  fi
  GIT_INDEX_FILE="$idx" "${GIT[@]}" write-tree
  local rc=$?
  rm -f "$idx"
  return $rc
}

if [ "${1:-}" = "--find" ]; then
  n="${2:-}"; dir="${3:-}"
  [ -n "$n" ] && [ -d "$dir" ] || { echo "usage: $0 --find N SNAPSHOT_DIR" >&2; exit 2; }
  dir="$(cd "$dir" && pwd)"
  found=1
  for c in $("${GIT[@]}" rev-list --first-parent -n "$n" HEAD); do
    t="$(snap_tree "$dir" "$c")" || exit 2
    if [ "$t" = "$("${GIT[@]}" rev-parse "$c^{tree}")" ]; then
      echo "MATCH commit=$c tree=$t"; found=0
    fi
  done
  [ $found -eq 0 ] || echo "NO MATCH in last $n first-parent commits"
  exit $found
fi

dir="${1:-}"; commit="${2:-HEAD}"
[ -d "$dir" ] || { echo "usage: $0 SNAPSHOT_DIR [COMMIT] | --find N SNAPSHOT_DIR" >&2; exit 2; }
dir="$(cd "$dir" && pwd)"
want="$("${GIT[@]}" rev-parse "$commit^{tree}")" || exit 2
full="$("${GIT[@]}" rev-parse "$commit^{commit}")" || exit 2
difflist="$(mktemp "${TMPDIR:-/tmp}/snapshot_check_diff.XXXXXX")" || exit 2
trap 'rm -f "$difflist"' EXIT
got="$(SNAP_DIFF="$difflist" snap_tree "$dir" "$commit")" || exit 2

echo "commit   $full"
echo "expected $want   (git rev-parse COMMIT^{tree})"
echo "snapshot $got   (tracked paths re-hashed from $dir)"

# EXTRA = files in the snapshot the commit does not track (informational).
extra="$(cd "$dir" && find . \( -type f -o -type l \) -not -path './.git/*' | sed 's|^\./||' | LC_ALL=C sort \
  | LC_ALL=C comm -23 - <("${GIT[@]}" ls-tree -r --name-only "$full" | LC_ALL=C sort))"
n_extra=0; [ -n "$extra" ] && n_extra="$(printf '%s\n' "$extra" | wc -l)"
echo "extra    $n_extra untracked path(s) in snapshot (not part of the verdict)"
[ "$n_extra" -gt 0 ] && printf '%s\n' "$extra" | head -20 | sed 's/^/           /'

if [ "$want" = "$got" ]; then echo "RESULT   MATCH"; exit 0; fi
echo "differing tracked paths (M = bytes differ, D = missing in snapshot), with the snapshot file's blob sha:"
while IFS=$'\t' read -r st path; do
  if [ "$st" = "D" ]; then echo "  D $path"; else echo "  $st $path  blob=$("${GIT[@]}" hash-object "$dir/$path")"; fi
done < "$difflist"
echo "RESULT   MISMATCH"
exit 1
