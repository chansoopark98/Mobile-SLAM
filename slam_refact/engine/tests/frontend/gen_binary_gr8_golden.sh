#!/usr/bin/env bash
# F-H2 evidence oracle (architecture.md §6.6.1): run the ORIGINAL xr-slam.wasm func 917
# (keypointsForCropJs) + func 918 (Gr8Gl::retainBest) in isolation on every captured GPU pyramid
# of a capture directory and store the kept keypoints per level, in output order.
# Reuses the spec-lane probe slam_refact/tools/re/spec_frontend/binary_gr8_probe.mjs unchanged
# (export-section patch only; the probe checks the pinned wasm sha256 and the code-section hash).
#
# Usage (cwd = repo root):
#   bash slam_refact/engine/tests/frontend/gen_binary_gr8_golden.sh <capture_dir> <out_dir>
#   e.g. build/slam-refact-20261015/spec/frontend/capture/tum-room1-full/c2
#        build/slam-refact-20261015/wp1/binary-gr8/tum-room1-full-c2
# The output directory must not exist yet (evidence is never overwritten).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../../.." && pwd)"
CAP="${1:?capture dir}"
OUT="${2:?output dir}"
PROBE="${ROOT}/slam_refact/tools/re/spec_frontend/binary_gr8_probe.mjs"
cd "${ROOT}"
if [[ -e "${OUT}" ]]; then
  echo "refusing to overwrite ${OUT}" >&2
  exit 2
fi
mkdir -p "${OUT}"
# S1 §2.2 layout of 240x320 / 480x640 cameras (layouts.jsonl, F-LAY).
LEVELS='[{"r":3,"c":3,"w":480,"h":640},{"r":3,"c":486,"w":444,"h":333},{"r":339,"c":486,"w":231,"h":308},{"r":339,"c":720,"w":160,"h":214}]'
ROWS="$(python3 -I -c 'import json,sys; print(json.load(open(sys.argv[1]))["rows"])' "${CAP}/meta.json")"
CAPS="$(python3 -I -c 'import json,sys; print(" ".join(str(c["capture"]) for c in json.load(open(sys.argv[1]))["captures"] if c.get("inputFrame") is not None))' "${CAP}/meta.json")"
{
  echo "# $(date -Iseconds) gen_binary_gr8_golden.sh ${CAP} ${OUT}"
  echo "# node $(node --version), rows ${ROWS}, levels ${LEVELS}, budgets 199,138,96,67, edge 21"
} > "${OUT}/commands.txt"
for c in ${CAPS}; do
  f="$(printf 'cap-%03d' "${c}")"
  cmd=(node "${PROBE}" --capture "${CAP}/${f}.rgba.gz" --rows "${ROWS}" --levels "${LEVELS}"
       --budgets 199,138,96,67 --edge 21 --out "${OUT}/${f}.json")
  echo "${cmd[*]}" >> "${OUT}/commands.txt"
  "${cmd[@]}" > /dev/null
done
(cd "${OUT}" && sha256sum cap-*.json > SHA256SUMS)
echo "wrote $(ls "${OUT}" | grep -c '^cap-') files to ${OUT}"
