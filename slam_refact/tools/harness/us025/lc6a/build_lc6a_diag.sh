#!/usr/bin/env bash
# US-025 LC-6a DIAG (NOW-2, 2026-10-29 engine review): builds slam_refact_replay_lc6a_diag = lc6a_diag_main.cc (lc6a_replay_main.cc + a read-only S2 / shared-count
# dump) against a fresh build of the PINNED engine commit. Nothing in slam_refact/engine and none of the pinned binaries / build dirs is touched.
#   1. git archive of the pin (slam_refact/engine + slam_refact/tools/harness/replay, the only trees the engine CMake reads) -> $OUT/src
#   2. cmake (same cache values as the pin's native-build: Release, SLAM_REFACT_LOOP_CLOSE=ON, WERROR=ON, BUILD_HARNESS=ON, BUILD_TESTS=OFF, offline Ceres 2.2) -> $OUT/native-build
#   3. ninja -j<jobs> slam_refact_replay_maphash (builds every static library; the maphash target is the one build_lc6a.sh takes its flags and link line from)
#   4. compile lc6a_diag_main.cc with the compile flags CMake used for maphash_replay_main.cc, link with the same line + the same --wrap as build_lc6a.sh
#   bash slam_refact/tools/harness/us025/lc6a/build_lc6a_diag.sh [jobs, 1..2, default 2]
set -euo pipefail
cd "$(dirname "$0")/../../../../.."
ROOT=$PWD
JOBS=${1:-2}
case "$JOBS" in 1|2) ;; *) echo "jobs must be 1 or 2" >&2; exit 2;; esac
PIN=d11d3e0abca62604de28b1db5863e39d198877eb
OUT=build/slam-refact-review-20261029/lc6a-diag
SRC=$OUT/src
ENG=$OUT/native-build
BIN=$OUT/bin
CERES_SRC=$ROOT/build/_deps/ceres-solver-src
[ -f "$CERES_SRC/CMakeLists.txt" ] || { echo "Ceres source missing: $CERES_SRC" >&2; exit 2; }
git cat-file -e "$PIN^{commit}" || { echo "pin commit $PIN not in this repository" >&2; exit 2; }
mkdir -p "$SRC" "$BIN"
if [ ! -f "$SRC/slam_refact/engine/CMakeLists.txt" ]; then
  git archive "$PIN" slam_refact/engine slam_refact/tools/harness/replay | tar -x -C "$SRC"
fi
echo "$PIN" > "$OUT/pin-commit.txt"
if [ ! -f "$ENG/build.ninja" ]; then
  cmake -S "$SRC/slam_refact/engine" -B "$ENG" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
    -DFETCHCONTENT_SOURCE_DIR_CERES_SOLVER="$CERES_SRC" -DSLAM_REFACT_CERES_SOURCE_DIR="$CERES_SRC" \
    -DSLAM_REFACT_CERES_VERSION=2.2 -DSLAM_REFACT_LOOP_CLOSE=ON -DSLAM_REFACT_WERROR=ON \
    -DSLAM_REFACT_BUILD_HARNESS=ON -DSLAM_REFACT_BUILD_TESTS=OFF
fi
grep -q '^SLAM_REFACT_LOOP_CLOSE:BOOL=ON' "$ENG/CMakeCache.txt" || { echo "$ENG was not configured with SLAM_REFACT_LOOP_CLOSE=ON" >&2; exit 2; }
ninja -C "$ENG" -j"$JOBS" slam_refact_replay_maphash
CC=$(python3 - "$ENG" <<'PY'
import json, sys
for e in json.load(open(sys.argv[1] + '/compile_commands.json')):
    if e['file'].endswith('harness/replay/maphash_replay_main.cc'):
        c = e['command']
        print(c[:c.index(' -o ')])
        break
else:
    raise SystemExit('maphash_replay_main.cc not in compile_commands.json')
PY
)
echo "$CC" > "$BIN/compile-flags.txt"
(cd "$ENG" && $CC -c "$ROOT/slam_refact/tools/harness/us025/lc6a/lc6a_diag_main.cc" -o "$ROOT/$BIN/lc6a_diag_main.o")
LINK=$(cd "$ENG" && ninja -t commands slam_refact_replay_maphash | tail -1)
LIBS=$(echo "$LINK" | tr ' ' '\n' | grep -E '\.a$|libz\.so' | tr '\n' ' ')
WRAP="-Wl,--wrap=_ZN11slam_refact11map_builder9LoopCloseERNS_3map8WorldMapEPNS0_14LoopCloseStateEPNS_16MapBuilderOutputEPNS0_15LoopCloseReportE"
(cd "$ENG" && /usr/bin/c++ -O3 -DNDEBUG $WRAP "$ROOT/$BIN/lc6a_diag_main.o" -o "$ROOT/$BIN/slam_refact_replay_lc6a_diag" $LIBS)
( cd "$ENG" && sha256sum $LIBS 2>/dev/null | grep -v libz.so ) > "$BIN/input-libs.sha256"
sha256sum "$BIN/slam_refact_replay_lc6a_diag" > "$BIN/binaries.sha256"
echo "built $BIN/slam_refact_replay_lc6a_diag"
