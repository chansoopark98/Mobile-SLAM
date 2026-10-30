#!/usr/bin/env bash
# it-scale-gate.v5 pre-designation SMOKE (requirement added after the v4 first run was INVALID for an unexercised output path).
#
#   bash slam_refact/tools/gates/it-scale-v5/smoke.sh --smoke-seed N --out NEWDIR [--prereg DIR] [--gate it-scale-gate.v5]
#
# Runs ONE sequence derived from the DEV seed N (must be in the manifest held.dev_seeds) through the very code path of the registered run
# (scene.py -> 5 original SDK runs in headless Chromium -> exact-input bundles -> engine gate arm -> REPORT runs -> controls -> premises -> judge)
# with the same directory layout (--out must be new and under build/slam-refact-pkgh-20261022; the driver writes to NEWDIR/gate).
# NO marker, NO sr_held call, NO coordinator seed read, NO designation needed.  The judged status of the dev sequence is information only.
# Exit codes: 0 every stage up to the rule's judge ran; 4 a stage failed (INCOMPLETE); 2 usage/precondition; 6 lane-precheck wait exceeded.
# This script is part of the pinned tool set; it changes nothing of the registered run.  Uses headless Chromium (flock build/c1-chromium.lock).
set -uo pipefail
orig_args=("$@")
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../../../.." && pwd)"
gate="it-scale-gate.v5"; prereg="$repo/build/slam-refact-full-20261019/preregistration"; seed="" out=""
usage() { echo "usage: $0 --smoke-seed N --out NEWDIR [--gate ID.vN] [--prereg DIR]" >&2; exit 2; }
while [ $# -gt 0 ]; do
  case "$1" in
    --smoke-seed) [ $# -ge 2 ] || usage; seed="$2"; shift 2;;
    --out) [ $# -ge 2 ] || usage; out="$2"; shift 2;;
    --gate) [ $# -ge 2 ] || usage; gate="$2"; shift 2;;
    --prereg) [ $# -ge 2 ] || usage; prereg="$2"; shift 2;;
    *) usage;;
  esac
done
case "$seed" in *[!0-9]*|"") echo "precondition: --smoke-seed is not a decimal integer" >&2; exit 2;; esac
[ -n "$out" ] || usage
[ ! -e "$out" ] || { echo "precondition: $out exists (a new directory is required)" >&2; exit 2; }
case "$(realpath -m "$out")" in "$repo"/build/slam-refact-pkgh-20261022/?*) ;; *) echo "precondition: --out must be a new directory under $repo/build/slam-refact-pkgh-20261022" >&2; exit 2;; esac
[ -d "$prereg" ] || { echo "precondition: --prereg $prereg is not a directory" >&2; exit 2; }
prereg="$(cd "$prereg" && pwd)"
engine="$prereg/${gate}-held-v1/sr_e5_replay"; prefix="$prereg/${gate}-held-v1/sr_e5_replay_prefix"
for f in "$engine" "$prefix"; do [ -f "$f" ] || { echo "precondition: $f is not a file" >&2; exit 2; }; done
mkdir -p "$(dirname "$out")"
waited=0
until PRECHECK_MAX_HEADLESS=0 PRECHECK_MAX_BUILDS=0 bash "$repo/slam_refact/docs/design/tools/lane-precheck.sh" >/dev/null 2>&1 \
      && [ "$(cut -d' ' -f1 /proc/loadavg | cut -d. -f1)" -lt 12 ]; do
  [ "$waited" -lt 21600 ] || { echo "lane-precheck still blocked after 6 h (nothing created)" >&2; exit 6; }
  sleep 30; waited=$((waited + 30))
done
mkdir "$out" || exit 2
out="$(cd "$out" && pwd)"
{
  echo "# it-scale-gate.v5 SMOKE $(date -Is) gate=$gate dev seed=$seed"
  printf 'command: bash %q' "$0"; printf ' %q' "${orig_args[@]}"; echo
  echo "cwd: $PWD host: $(hostname) loadavg: $(cut -d' ' -f1-3 /proc/loadavg) lane-precheck waited ${waited}s"
  echo "node: $(command -v node) $(node --version); python3: $(command -v python3) $(python3 -I -c 'import sys; print(sys.version.split()[0])')"
  echo "chrome: $(/usr/bin/google-chrome --version 2>&1 | head -1)"
  echo "engine: $(sha256sum "$engine")"; echo "prefix engine: $(sha256sum "$prefix")"
  node "$here/run.mjs" --prereg "$prereg" --gate "$gate" --engine "$engine" --prefix-engine "$prefix" --smoke --smoke-seed "$seed" --out "$out/gate"
  rc=$?
  echo "exit=$rc"; echo "# done $(date -Is)"
  exit "$rc"
} 2>&1 | tee -a "$out/run.log"
rc=${PIPESTATUS[0]}
(cd "$out" && find . -type f ! -name SHA256SUMS -printf '%P\0' | sort -z | xargs -0 sha256sum > SHA256SUMS)
exit "$rc"
