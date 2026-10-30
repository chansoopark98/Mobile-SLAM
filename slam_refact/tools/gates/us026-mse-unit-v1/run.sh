#!/usr/bin/env bash
# us026-mse-unit-gate.v1: the registered first-run command (gate file section 5).
#
#   bash slam_refact/tools/gates/us026-mse-unit-v1/run.sh --designation-sha256 SHA256 --out NEWDIR
#        [--gate us026-mse-unit-gate.v1] [--prereg DIR] [--binary HELD_BIN] [--precheck] [--judge-only GOLDEN_DIR]
#
#   --designation-sha256  sha256 of the coordinator's designation file (exported as SR_US026_DESIGNATION)
#   --out                 a NEW directory: run.log (this script's whole output), goldens/ (step 1), judge.gtest.json,
#                         SHA256SUMS
#   --gate                the designated gate "<gate_id>.v<N>" (default: the gate this directory is named for)
#   --prereg              preregistration directory (default build/slam-refact-full-20261019/preregistration)
#   --binary              the held judge (default <prereg>/us026-mse-unit-held-v1/sr_tests_us026_method_precision_held);
#                         its own sha256 is a held.pins entry, so any other binary is refused before it compares anything
#   --precheck            side-effect-free check of both kinds (golden, judge) with the shared device: no marker, nothing
#                         generated, no comparison; --out is not needed
#   --audit               after the coordinator recorded first_real_run_at: sr_held audit of both kinds (no env value)
#   --judge-only DIR      step 2 only, on the goldens/ of an earlier run whose judge step was REFUSED before its marker
#                         (a refusal is not a run; gate file section 6 rule R3)
#
# Steps of a real run: (0) lane-precheck (blocked -> exit 6, nothing created), (1) golden step = golden.mjs (sr_held kind
# "golden": marker, then the original-side goldens), (2) judge = the held C++ test (sr_held kind "judge": golden marker,
# golden provenance, marker, then the comparison), (3) refuse an output with a US026-MUTATION line.
# Exit codes: 0 judge PASS; 1 judge FAIL (or judge HELD, see run.log "[US026-INTEGRITY] FAIL"); 2 usage / precondition;
#   3 golden step HELD (no marker: not a run); 4 golden step failed after its marker (INVALID); 5 mutation line (INVALID);
#   6 lane-precheck blocked (wait; do not change its thresholds).
# This script never creates, edits or removes a marker, manifest, designation or seed file (the devices create markers).
# It is an accident guard, not an authentication (protocol 12.6).
set -uo pipefail
orig_args=("$@")
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../../../.." && pwd)"

gate="us026-mse-unit-gate.v1"
prereg="$repo/build/slam-refact-full-20261019/preregistration"
binary="" sha="" out="" precheck=0 judge_only="" audit=0
usage() { echo "usage: $0 --designation-sha256 SHA256 --out NEWDIR [--gate ID.vN] [--prereg DIR] [--binary BIN] [--precheck] [--judge-only GOLDEN_DIR] | --audit [--gate ID.vN] [--prereg DIR]" >&2; exit 2; }
while [ $# -gt 0 ]; do
  case "$1" in
    --designation-sha256) [ $# -ge 2 ] || usage; sha="$2"; shift 2;;
    --out) [ $# -ge 2 ] || usage; out="$2"; shift 2;;
    --gate) [ $# -ge 2 ] || usage; gate="$2"; shift 2;;
    --prereg) [ $# -ge 2 ] || usage; prereg="$2"; shift 2;;
    --binary) [ $# -ge 2 ] || usage; binary="$2"; shift 2;;
    --precheck) precheck=1; shift;;
    --judge-only) [ $# -ge 2 ] || usage; judge_only="$2"; shift 2;;
    --audit) audit=1; shift;;
    *) usage;;
  esac
done
[ "$audit" -eq 1 ] && sha="${sha:-$(printf "%064d" 0)}"
case "$sha" in *[!0-9a-f]*|"") echo "precondition: --designation-sha256 is not lower-case hex" >&2; exit 2;; esac
[ "${#sha}" -eq 64 ] || { echo "precondition: --designation-sha256 is not 64 hex digits" >&2; exit 2; }
case "$gate" in
  *.v[1-9]|*.v[1-9][0-9]*) ;;
  *) echo "precondition: --gate must be <gate_id>.v<N>" >&2; exit 2;;
esac
[ -d "$prereg" ] || { echo "precondition: --prereg $prereg is not a directory" >&2; exit 2; }
prereg="$(cd "$prereg" && pwd)"
[ -n "$binary" ] || binary="$prereg/us026-mse-unit-held-v1/sr_tests_us026_method_precision_held"
[ -f "$binary" ] || { echo "precondition: binary $binary is not a file" >&2; exit 2; }
binary="$(readlink -f "$binary")"
node_bin="$(readlink -f "$(command -v node)")" || { echo "precondition: node not found" >&2; exit 2; }

# The pinned files (manifest held.pins, by base name). Both steps present exactly this set.
mmg="$repo/slam_refact/tools/harness/us026/mse_method_golden.mjs"
kf6="$repo/slam_refact/tools/re/spec_metric_scale/mse-probe-kf6.mjs"
wasm="$repo/build/slam-refact-20261015/inputs/xr-slam.wasm"
golden="$here/golden.mjs"
runner="$here/run.sh"
srheld="$repo/slam_refact/tools/prereg/sr_held.mjs"
others=("$mmg" "$kf6" "$wasm" "$golden" "$runner" "$srheld")
golden_pins=("$binary" "${others[@]}")                       # + process.execPath inside golden.mjs
judge_pins="$(IFS=:; echo "$node_bin:${others[*]}")"            # + the judge's own executable
pin_args=(); for p in "$binary" "$node_bin" "${others[@]}"; do pin_args+=(--pin "$p"); done
gate_args=(--prereg "$prereg" --gate "$gate" --env-var SR_US026_DESIGNATION --mmg "$mmg" --kf6 "$kf6" --wasm "$wasm")

if [ "$audit" -eq 1 ]; then
  # post-run audit (after the coordinator recorded first_real_run_at): both kinds, sr_held audit (order, pins, markers)
  "$node_bin" --input-type=module -e "
    import { audit } from '$srheld';
    const [prereg, gate] = process.argv.slice(1);
    const m = /^(.+)\.v([0-9]+)\$/.exec(gate);
    let rc = 0;
    for (const kind of ['golden', 'judge']) {
      const o = audit({ preregDir: prereg, gateId: m[1], version: Number(m[2]), kind });
      console.log(o.log.join('\n'));
      console.log(\`audit \${kind}: \${o.ok ? 'OK' : 'FAIL [' + o.code + ']: ' + o.why}\`);
      if (!o.ok) rc = 1;
    }
    process.exit(rc);" "$prereg" "$gate"
  exit $?
fi

if [ "$precheck" -eq 1 ]; then
  id="${gate%.v*}" version="${gate##*.v}" rc=0
  echo "# precheck $(date -Is) gate=$gate prereg=$prereg (no marker, nothing generated, no comparison)"
  echo "## kind golden (golden.mjs --check)"
  gp=(); for p in "${golden_pins[@]}"; do gp+=(--pin "$p"); done
  SR_US026_DESIGNATION="$sha" "$node_bin" "$golden" "${gate_args[@]}" "${gp[@]}" --check || rc=$?
  echo "## kind judge (sr_held.mjs check)"
  SR_US026_DESIGNATION="$sha" "$node_bin" "$srheld" check --prereg "$prereg" --gate "$id" --version "$version" --kind judge \
    --env-var SR_US026_DESIGNATION --seed "${pin_args[@]}" || rc=$?
  for k in golden judge; do
    mk="$prereg/$gate.first-run.$k.marker"
    if [ -e "$mk" ]; then echo "marker $k: EXISTS ($mk)"; rc=3; else echo "marker $k: absent"; fi
  done
  echo "precheck exit=$rc"
  exit "$rc"
fi

[ -n "$out" ] || usage
[ ! -e "$out" ] || { echo "precondition: $out exists (a new directory is required)" >&2; exit 2; }
if [ -n "$judge_only" ]; then
  [ -f "$judge_only/GOLDEN-SHA256SUMS" ] || { echo "precondition: --judge-only $judge_only has no GOLDEN-SHA256SUMS" >&2; exit 2; }
  judge_only="$(cd "$judge_only" && pwd)"
fi
if ! PRECHECK_MAX_HEADLESS=1 PRECHECK_MAX_BUILDS=0 bash "$repo/slam_refact/docs/design/tools/lane-precheck.sh"; then
  echo "lane-precheck blocked: nothing was created; wait and run again (do not change the thresholds)" >&2
  exit 6
fi
mkdir -p "$(dirname "$out")" && mkdir "$out" || { echo "precondition: cannot create $out" >&2; exit 2; }
out="$(cd "$out" && pwd)"
log="$out/run.log"
goldens="${judge_only:-$out/goldens}"

{
  echo "# RUN $gate $(date -Is) reviewer=$sha"
  printf 'command: bash %q' "$0"; printf ' %q' "${orig_args[@]}"; echo
  echo "cwd: $PWD host: $(hostname) loadavg: $(cut -d' ' -f1-3 /proc/loadavg) node: $("$node_bin" --version)"
  echo "git HEAD: $(git -C "$repo" rev-parse HEAD 2>/dev/null) dirty F paths: $(git -C "$repo" status --porcelain -- slam_refact/engine slam_refact/web-chunk slam_refact/web-smoke slam_refact/tools wasm src 2>/dev/null | wc -l)"
  sha256sum "$binary" "$node_bin" "${others[@]}"
  rc=0
  if [ -z "$judge_only" ]; then
    echo "## step 1: golden step (kind golden) $(date -Is)"
    gp=(); for p in "${golden_pins[@]}"; do gp+=(--pin "$p"); done
    SR_US026_DESIGNATION="$sha" "$node_bin" "$golden" "${gate_args[@]}" "${gp[@]}" --out "$goldens"
    rc=$?
    echo "golden step exit=$rc"
    if [ "$rc" -ne 0 ]; then echo "exit=$rc"; echo "# done $(date -Is)"; exit "$rc"; fi
  else
    echo "## step 1 skipped (--judge-only $goldens)"
  fi
  echo "## step 2: judge (kind judge) $(date -Is)"
  env SR_US026_DESIGNATION="$sha" SR_US026_GATE="$gate" SR_US026_PREREG_DIR="$prereg" SR_US026_DATA_DIR="$goldens" \
    SR_US026_PINS="$judge_pins" "$binary" "--gtest_output=json:$out/judge.gtest.json" > "$out/judge.log" 2>&1
  rc=$?
  cat "$out/judge.log"
  echo "judge exit=$rc"
  if grep -q '^US026-MUTATION' "$out/judge.log"; then
    echo "INVALID: the output contains a US026-MUTATION line (a mutation copy was judged)"
    rc=5
  fi
  echo "exit=$rc"
  echo "# done $(date -Is)"
  exit "$rc"
} 2>&1 | tee -a "$log"
rc=${PIPESTATUS[0]}

(cd "$out" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS)
exit "$rc"
