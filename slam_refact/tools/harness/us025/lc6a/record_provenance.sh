#!/usr/bin/env bash
# US-025 LC-6a: provenance of an LC-6a evidence run (what was compared with what). Writes build/slam-refact-final-20261025/lc6a/provenance.json (never overwritten).
set -euo pipefail
cd "$(dirname "$0")/../../../../.."
OUT=build/slam-refact-final-20261025/lc6a/provenance.json
[ ! -e "$OUT" ] || { echo "refusing to overwrite $OUT" >&2; exit 2; }
ENG=build/slam-refact-final-20261025/lc3-native
python3 - "$OUT" "$ENG" <<'PY'
import hashlib, json, subprocess, sys, pathlib
out, eng = sys.argv[1:3]
sh = lambda p: hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
run = lambda *a: subprocess.run(a, capture_output=True, text=True).stdout.strip()
files = ['slam_refact/tools/harness/us025/compare_us025.py', 'slam_refact/tools/harness/us025/lc6a/lc6a_replay_main.cc', 'slam_refact/tools/harness/us025/lc6a/build_lc6a.sh',
         'slam_refact/tools/harness/us025/lc6a/run_engine_arms.sh', 'slam_refact/tools/harness/us025/lc6a/run_original_lc0.sh', 'slam_refact/tools/harness/us025/lc6a/make_run_lc6a.py',
         'slam_refact/tools/harness/us025/lc6a/run_lc6a.mjs', 'slam_refact/tools/harness/us025/lc6a/make_null_bundles.sh', 'slam_refact/tools/harness/us025/lc6a/make_spec.py',
         'slam_refact/tools/harness/us025/lc6a/mutants_lc6a.py', 'slam_refact/tools/harness/us025/lc6a/test_compare_us025.py',
         'slam_refact/tools/harness/us026/compare_us026.py', 'slam_refact/engine/src/map_builder/loop_close.cc', 'slam_refact/engine/src/map_builder/map_builder.cc',
         'slam_refact/engine/src/map_builder/agate_descriptor.cc', 'slam_refact/engine/src/map_tracker/map_tracker.cc']
doc = {'gitHead': run('git', 'rev-parse', 'HEAD'), 'gitBranch': run('git', 'rev-parse', '--abbrev-ref', 'HEAD'),
       'engineSrcStatusSha256': hashlib.sha256(run('git', 'status', '--porcelain', '--', 'slam_refact/engine').encode()).hexdigest(),
       'engineBuild': eng, 'engineBuildCache': {k: run('grep', '-m1', '^' + k, eng + '/CMakeCache.txt') for k in ('CMAKE_BUILD_TYPE', 'SLAM_REFACT_LOOP_CLOSE', 'SLAM_REFACT_WERROR')},
       'files': {f: sh(f) for f in files}, 'binaries': {f: sh(f) for f in ('build/slam-refact-final-20261025/lc6a/bin/slam_refact_replay_lc6a', eng + '/harness/slam_refact_replay')},
       'wasmOriginal': '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422', 'goldenManifestSha256': sh('build/slam-refact-pkgh-20261022/us025/golden/MANIFEST.sha256'),
       'node': run('node', '--version'), 'python': run('python3', '--version'), 'uname': run('uname', '-a')}
pathlib.Path(out).write_text(json.dumps(doc, indent=1) + '\n')
print('wrote', out)
PY
