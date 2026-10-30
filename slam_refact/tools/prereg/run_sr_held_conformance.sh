#!/usr/bin/env bash
# Conformance of the shared HELD device: C++ (sr_held.h) and node (sr_held.mjs) against the one case table
# sr_held_cases.json, and a diff of the decisions both made.
#
#   bash slam_refact/tools/prereg/run_sr_held_conformance.sh [BUILD_DIR] [JOBS]
#
# BUILD_DIR defaults to build/slam-refact-full-c1-held (a NEW directory; existing build/slam-refact-full-* directories are
# pinned and are never touched). JOBS defaults to 2 and must be 1..4. Steps:
#   1. regenerate the case table into a temp file and compare it with the checked-in one (the table is reproducible)
#   2. configure + build the standalone C++ test (cmake -S engine/tests/support), run it through ctest
#   3. run the node test
#   4. diff the DECISION lines of both
# The mutation check is separate and slower: python3 -I slam_refact/tools/prereg/run_sr_held_mutants.py --out BUILD_DIR/mutants
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../../.." && pwd)"
build="${1:-$repo/build/slam-refact-full-c1-held}"
jobs="${2:-2}"
case "$jobs" in 1|2|3|4) ;; *) echo "JOBS must be 1..4 (got $jobs)" >&2; exit 2;; esac
case "$(basename "$build")" in
  slam-refact-full-c1-held|slam-refact-full-c1-held-*|sr-held-*) ;;
  *) echo "refusing BUILD_DIR $build: use a new directory named slam-refact-full-c1-held* (existing slam-refact-full-* builds are pinned)" >&2; exit 2;;
esac
cases="$here/sr_held_cases.json"
mkdir -p "$build"

echo "== 1. case table is reproducible"
python3 -I "$here/gen_sr_held_cases.py" --out "$build/cases.regenerated.json" >/dev/null
cmp "$build/cases.regenerated.json" "$cases"

echo "== 2. C++ conformance (standalone cmake, jobs $jobs)"
cmake -S "$repo/slam_refact/engine/tests/support" -B "$build" -G Ninja >/dev/null
cmake --build "$build" --parallel "$jobs"
ctest --test-dir "$build" --output-on-failure
SR_HELD_TMP="$build/work" SR_HELD_DECISIONS="$build/cpp-decisions.txt" "$build/sr_held_conformance" "$cases" | tail -1

echo "== 3. node conformance"
SR_HELD_TMP="$build/work" SR_HELD_DECISIONS="$build/node-decisions.txt" node --test "$here/sr_held_conformance.test.mjs" | grep -E "^# (tests|pass|fail)"

echo "== 4. decisions of the two implementations"
diff "$build/cpp-decisions.txt" "$build/node-decisions.txt"
echo "IDENTICAL: $(wc -l < "$build/node-decisions.txt") decision lines"
echo "CONFORMANCE OK"
