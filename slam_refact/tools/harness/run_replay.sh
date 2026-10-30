#!/usr/bin/env bash
# Replay one scenario through the C++ engine (architecture.md §6.1-6.3, WP5): bundle -> GlueRunner ->
# trace in the original reference schema. A new label is required; existing bundles/runs are never
# overwritten.
#
#   bash slam_refact/tools/harness/run_replay.sh <scenario> <label> <mode>
#
# modes (§6.2):
#   parity    drop mask = reference r1, pyramid model swiftshader-approx (gate runs)
#   ideal     every frame processed (report only)
#   ieee      drop mask r1, IEEE pyramid model (report only, D-9)
#   injected  captured pyramids injected: flip6-s2 frames 0-71 (c2-73, V-H7), tum-room1-full frames 0-22 (c2, V-H7b)
#   absolute  <scenario>-absolute with the DERIVED-LINACC-OS-EQUIVALENT acceleration (MS-RT), drop mask r1
# The runner binary defaults to build/slam-refact-native/harness/slam_refact_replay (override with
# SLAM_REFACT_REPLAY).
set -euo pipefail
cd "$(dirname "$0")/../../.."
[ $# -eq 3 ] || { echo "usage: $0 <scenario> <label> <parity|ideal|ieee|injected|absolute>" >&2; exit 2; }
scenario=$1 label=$2 mode=$3
[[ $label =~ ^[a-z0-9-]+$ ]] || { echo "label must match [a-z0-9-]+" >&2; exit 2; }
H=slam_refact/tools/harness
E=build/slam-refact-20261015
CAP=$E/spec/frontend/capture
case $mode in
  parity) args=(--drop-mask-from r1) ;;
  ideal) args=() ;;
  ieee) args=(--drop-mask-from r1 --pyramid-model ieee) ;;
  injected)
    case $scenario in
      flip6-s2) args=(--drop-mask-from r1 --pyramid-capture "$CAP/flip6-s2/c2-73" --max-frames 72) ;;
      tum-room1-full) args=(--drop-mask-from r1 --pyramid-capture "$CAP/tum-room1-full/c2" --max-frames 23) ;;
      *) echo "no captured pyramids for $scenario" >&2; exit 2 ;;
    esac ;;
  absolute)
    [[ $scenario == *-absolute ]] || { echo "absolute mode needs a *-absolute scenario" >&2; exit 2; }
    args=(--drop-mask-from r1 --linear-accel "$E/reference/runs/$scenario/derived-linear-acceleration.tsv.gz") ;;
  *) echo "unknown mode $mode" >&2; exit 2 ;;
esac
bin=${SLAM_REFACT_REPLAY:-build/slam-refact-native/harness/slam_refact_replay}
if [ ! -x "$bin" ]; then
  echo "BLOCKED: $bin not built" >&2
  exit 3
fi
bundle=$E/harness/bundles/$scenario/$label
out=$E/engine-runs/$scenario/$label
[ ! -e "$out" ] || { echo "refusing to overwrite $out" >&2; exit 2; }
prep=(python3 -I $H/prepare_inputs.py --scenario "$scenario" --label "$label" "${args[@]}")
"${prep[@]}"
mkdir -p "$(dirname "$out")"
"$bin" --bundle "$bundle" --out "$out"
{
  echo "# $(date -u +%Y-%m-%dT%H:%M:%SZ) cwd=$PWD"
  echo "bash slam_refact/tools/harness/run_replay.sh $scenario $label $mode"
  printf '%q ' "${prep[@]}"; echo
  printf '%q ' "$bin" --bundle "$bundle" --out "$out"; echo
} > "$out/commands.txt"
python3 - "$out" "$bundle" "$bin" "$mode" <<'EOF'
import hashlib, json, pathlib, subprocess, sys
out, bundle, binary = (pathlib.Path(a) for a in sys.argv[1:4])
mode = sys.argv[4]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
git = lambda *a: subprocess.run(['git', *a], capture_output=True, text=True).stdout.strip()
prov = {'mode': mode, 'gitHead': git('rev-parse', 'HEAD'), 'gitDirtyFiles': len(git('status', '--short').splitlines()),
        'binary': str(binary), 'binarySha256': sha(binary), 'bundle': str(bundle), 'bundleJsonSha256': sha(bundle / 'bundle.json'),
        'thresholdsSha256': sha(pathlib.Path('slam_refact/tools/harness/thresholds-v1.json'))}
(out / 'provenance.json').write_text(json.dumps(prov, indent=1) + '\n')
print(json.dumps(prov))
EOF
(cd "$out" && sha256sum -- * > SHA256SUMS)
