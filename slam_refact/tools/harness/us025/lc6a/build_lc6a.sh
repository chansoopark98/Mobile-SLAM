#!/usr/bin/env bash
# US-025 LC-6a: builds slam_refact_replay_lc6a (lc6a_replay_main.cc = replay_main.cc + a linker --wrap capture of map_builder::LoopClose)
# against the static libraries of an existing engine build (default build/slam-refact-final-20261025/lc3-native, SLAM_REFACT_LOOP_CLOSE=ON). Nothing in
# slam_refact/engine is changed and the engine build directory is only read; the output goes to build/slam-refact-final-20261025/lc6a/bin.
# The compile flags are those CMake used for tools/harness/replay/maphash_replay_main.cc in that build (compile_commands.json); the link line is
# `ninja -t commands slam_refact_replay_maphash` with our object and wrap option substituted. sha256 of every input library is written next to the binary.
#   bash slam_refact/tools/harness/us025/lc6a/build_lc6a.sh [engine build dir]
set -euo pipefail
cd "$(dirname "$0")/../../../../.."
ENG=${1:-build/slam-refact-final-20261025/lc3-native}
OUT=build/slam-refact-final-20261025/lc6a/bin
mkdir -p "$OUT"
[ -f "$ENG/build.ninja" ] || { echo "not a ninja build dir: $ENG" >&2; exit 2; }
grep -q '^SLAM_REFACT_LOOP_CLOSE:BOOL=ON' "$ENG/CMakeCache.txt" || { echo "$ENG was not configured with SLAM_REFACT_LOOP_CLOSE=ON" >&2; exit 2; }
ROOT=$PWD
CC=$(python3 - "$ENG" <<'PY'
import json, sys
for e in json.load(open(sys.argv[1] + '/compile_commands.json')):
    if e['file'].endswith('harness/replay/maphash_replay_main.cc'):
        c = e['command']
        c = c[:c.index(' -o ')]
        print(c)
        break
else:
    raise SystemExit('maphash_replay_main.cc not in compile_commands.json')
PY
)
echo "$CC" > "$OUT/compile-flags.txt"
(cd "$ENG" && $CC -c "$ROOT/slam_refact/tools/harness/us025/lc6a/lc6a_replay_main.cc" -o "$ROOT/$OUT/lc6a_replay_main.o")
LINK=$(cd "$ENG" && ninja -t commands slam_refact_replay_maphash | tail -1)
# libraries from the link line (relative to the engine build dir), without the maphash objects
LIBS=$(echo "$LINK" | tr ' ' '\n' | grep -E '\.a$|libz\.so' | tr '\n' ' ')
WRAP="-Wl,--wrap=_ZN11slam_refact11map_builder9LoopCloseERNS_3map8WorldMapEPNS0_14LoopCloseStateEPNS_16MapBuilderOutputEPNS0_15LoopCloseReportE"
(cd "$ENG" && /usr/bin/c++ -O3 -DNDEBUG $WRAP "$ROOT/$OUT/lc6a_replay_main.o" -o "$ROOT/$OUT/slam_refact_replay_lc6a" $LIBS)
( cd "$ENG" && sha256sum $LIBS 2>/dev/null | grep -v libz.so ) > "$OUT/input-libs.sha256"
sha256sum "$OUT/slam_refact_replay_lc6a" "$ENG/harness/slam_refact_replay" > "$OUT/binaries.sha256"
echo "built $OUT/slam_refact_replay_lc6a"
