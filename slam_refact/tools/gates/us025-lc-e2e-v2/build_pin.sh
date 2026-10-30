#!/usr/bin/env bash
# us025-lc-e2e-gate.v2 (UNCHANGED copy of the v1 script; v2 reuses the v1 pin directory, the engine commit and the binary are the same, D-V2-3, so this script is NOT run for v2): builds the PINNED engine replay binary slam_refact_replay_lc6a from a git archive of <commit> (never from the working tree), with
# SLAM_REFACT_LOOP_CLOSE=ON, Release, -Werror, jobs 2, under build/c1-build.lock. Procedure = ba-e2e-v2-pin/build-native.sh (configure + build of a snapshot) plus
# the link step of slam_refact/tools/harness/us025/lc6a/build_lc6a.sh (lc6a_replay_main.cc = replay_main.cc + a --wrap capture of map_builder::LoopClose), both
# taken from the SNAPSHOT so the binary is a function of <commit> alone. Output: build/slam-refact-pkgh-20261022/us025/lc-e2e-v1-pin (refuses to overwrite it).
#   bash slam_refact/tools/gates/us025-lc-e2e-v2/build_pin.sh <40-hex commit>
# Run by the coordinator after the engine version change (D-G10) is committed; not run by the lane that wrote it (UNVERIFIED: only `bash -n` was checked).
# Then: node slam_refact/tools/gates/us025-lc-e2e-v2/make_lc_manifest.mjs --prereg-dir <dir> --static <static.json> --engine-commit <commit> [--check]
set -euo pipefail
cd "$(dirname "$0")/../../../.."
R=$PWD
C=${1:-}
[[ "$C" =~ ^[0-9a-f]{40}$ ]] || { echo "usage: build_pin.sh <40-hex commit>" >&2; exit 2; }
git cat-file -e "$C^{commit}" || { echo "not a commit: $C" >&2; exit 2; }
git merge-base --is-ancestor "$C" HEAD || { echo "$C is not an ancestor of HEAD" >&2; exit 2; }
P=build/slam-refact-pkgh-20261022/us025/lc-e2e-v1-pin
[ ! -e "$P" ] || { echo "refusing to overwrite $P" >&2; exit 2; }
S=$P/src
mkdir -p "$S" "$P/native"
git archive "$C" | tar -x -C "$S"
MAIN=$S/slam_refact/tools/harness/us025/lc6a/lc6a_replay_main.cc
[ -f "$MAIN" ] || { echo "$C does not contain slam_refact/tools/harness/us025/lc6a/lc6a_replay_main.cc: commit the lc6a tools before the pin commit" >&2; exit 2; }
B=$P/native-build
flock build/c1-build.lock cmake -S "$S/slam_refact/engine" -B "$B" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DSLAM_REFACT_CERES_SOURCE_DIR="$R/build/_deps/ceres-solver-src" \
  -DSLAM_REFACT_GTEST_SOURCE_DIR="$R/build/_deps/googletest-src" \
  -DSLAM_REFACT_EVIDENCE_ROOT="$R/build/slam-refact-20261015" \
  -DSLAM_REFACT_THIRD_PARTY_DIR="$R/$S/slam_refact/engine/third_party" \
  -DSLAM_REFACT_WERROR=ON -DSLAM_REFACT_BUILD_TESTS=OFF -DSLAM_REFACT_BUILD_HARNESS=ON \
  -DSLAM_REFACT_LOOP_CLOSE=ON -DCMAKE_CXX_COMPILER=/usr/bin/c++ -DFETCHCONTENT_FULLY_DISCONNECTED=ON
flock build/c1-build.lock cmake --build "$B" -j 2 --target slam_refact_replay_maphash
CC=$(python3 - "$B" <<'PY'
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
(cd "$B" && $CC -c "$R/$MAIN" -o "$R/$P/lc6a_replay_main.o")
LINK=$(cd "$B" && ninja -t commands slam_refact_replay_maphash | tail -1)
LIBS=$(echo "$LINK" | tr ' ' '\n' | grep -E '\.a$|libz\.so' | tr '\n' ' ')
WRAP="-Wl,--wrap=_ZN11slam_refact11map_builder9LoopCloseERNS_3map8WorldMapEPNS0_14LoopCloseStateEPNS_16MapBuilderOutputEPNS0_15LoopCloseReportE"
(cd "$B" && /usr/bin/c++ -O3 -DNDEBUG $WRAP "$R/$P/lc6a_replay_main.o" -o "$R/$P/native/slam_refact_replay_lc6a" $LIBS)
chmod 555 "$P/native/slam_refact_replay_lc6a"
( cd "$B" && sha256sum $LIBS 2>/dev/null | grep -v libz.so ) > "$P/native/input-libs.sha256"
python3 -I - "$C" "$P" "$B" "$MAIN" <<'PY'
import hashlib, json, subprocess, sys, pathlib, datetime
c, p, b, main = sys.argv[1:5]
sh = lambda f: hashlib.sha256(pathlib.Path(f).read_bytes()).hexdigest()
run = lambda *a: subprocess.run(a, capture_output=True, text=True).stdout.strip()
cache = pathlib.Path(b, 'CMakeCache.txt').read_text().splitlines()
pick = lambda k: next((x for x in cache if x.startswith(k + ':')), None)
doc = {'schema': 'us025-lc-e2e-v1-pin-v1', 'commit': c, 'commit_tree': run('git', 'rev-parse', c + '^{tree}'),
       'built_at': datetime.datetime.now().astimezone().isoformat(timespec='seconds'), 'jobs': 2,
       'binary': {'path': p + '/native/slam_refact_replay_lc6a', 'sha256': sh(p + '/native/slam_refact_replay_lc6a')},
       'lc6a_main': {'path': main, 'sha256': sh(main)}, 'input_libs_sha256_file': sh(p + '/native/input-libs.sha256'),
       'cmake_cache': {k: pick(k) for k in ('CMAKE_BUILD_TYPE', 'SLAM_REFACT_LOOP_CLOSE', 'SLAM_REFACT_WERROR', 'CMAKE_CXX_COMPILER')},
       'compiler': run('/usr/bin/c++', '--version').splitlines()[0],
       'note': 'Built from a git archive of the commit, not from the working tree. Do not rebuild or modify this directory: the gate manifest pins the binary sha256.'}
pathlib.Path(p, 'pin.json').write_text(json.dumps(doc, indent=1) + '\n')
print(json.dumps(doc, indent=1))
PY
echo "PIN_BUILD_DONE $P/native/slam_refact_replay_lc6a"
