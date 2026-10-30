#!/usr/bin/env bash
# Spec lane S1 (frontend): reproduce every piece of evidence cited in slam_refact/docs/spec/01-frontend.md.
# Requires: lane B public build (slam_refact/tools/re/public_build/build_probe.sh + build_probe_wasm.sh),
# lane C TUM fixture (build/slam-refact-20261015/reference/fixtures/tum-room1-portrait-full), Flip6 replay
# build/flip6-20261015/L-S2-loop-replay, /usr/bin/google-chrome, ~/emsdk.
# Capture labels must be new per attempt (existing directories are never overwritten).
# Usage: run_all.sh <capture_label>        e.g. run_all.sh c3
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../../.." && pwd)"
T="${ROOT}/slam_refact/tools/re/spec_frontend"
E="${ROOT}/build/slam-refact-20261015/spec/frontend"
L="${1:?capture label}"
cd "${ROOT}"
bash "${T}/build.sh"
bash "${T}/build_wasm.sh"
"${E}/bin/frontend_probe" selftest | tee "${E}/selftest.json"
for s in "240 320" "480 640" "360 640" "720 1280" "480 480" "640 480 90"; do
  "${E}/bin/frontend_probe" layout ${s}
done | tee "${E}/layouts.jsonl"
python3 -I "${T}/binary_tables.py" --out "${E}/binary-tables.json"
node "${T}/gl_semantics_probe.mjs" --out "build/slam-refact-20261015/spec/frontend/gl-semantics/swiftshader-${L}.json"
node "${T}/capture_pyramid.mjs" --scenario tum-room1-full --label "${L}" --max-frames 40 --max-captures 24 --rows 660
node "${T}/capture_pyramid.mjs" --scenario flip6-s2 --label "${L}" --max-frames 36 --max-captures 24 --rows 660
python3 -I "${T}/run_gates.py" --capture-dir "${E}/capture/tum-room1-full/${L}" --src-w 240 --src-h 320 \
  --fx-level0 521.8225 --out "${E}/gates/tum-room1-full-${L}.json"
python3 -I "${T}/run_gates.py" --capture-dir "${E}/capture/flip6-s2/${L}" --src-w 480 --src-h 640 \
  --fx-level0 506.4104 --out "${E}/gates/flip6-s2-${L}.json"
# v1 (pre-registered) verdicts, normative IEEE model and SwiftShader-approximate model.
for ss in 0 1; do
  python3 -I "${T}/compare_captures.py" --capture-dir "${E}/capture/tum-room1-full/${L}" --fx-level0 521.8225 \
    --swiftshader-approx "${ss}" --out "${E}/compare/tum-${L}-ss${ss}-bf1.json"
done
