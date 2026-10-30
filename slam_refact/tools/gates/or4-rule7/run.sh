#!/usr/bin/env bash
# OR-4 v6 rule 7 (succession condition) as ONE registered command (preregistration or4-it-world-regression.v6.md section 0.2 rule 7;
# change-closure audit B "OR-4": rule 7 on the frozen tree as a registered command, compared by sha256 only, exit code inside the tee).
#
#   bash slam_refact/tools/gates/or4-rule7/run.sh --binary FILE --binary-sha256 SHA256 --out NEWDIR [--workers 1|2]
#
#   --binary         the frozen sr_or4_replay of the tree under test (a held copy, preregistration/<gate>-held-v<N>/ or the freeze build)
#   --binary-sha256  the designated sha256 of that binary (FREEZE.json); any other binary is refused before anything runs
#   --out            a NEW directory: N/, D/, T/ with the 42 runs, run.log (this script's whole output, including "exit=<code>"),
#                    compare-rule7.json (per trace: sha256 of the timing-stripped trace of the new run and of the pinned binary b1f4f92a),
#                    SHA256SUMS (of run.log and compare-rule7.json)
#
# What it does (rule7.py): runs N s0 and D force (14 bundles each) with the given binary, compares their stripped-trace sha256 with the
# pinned binary's; only if all 28 are identical runs T s0 (14, with the target) and compares again. Exit code 0 only for 42/42 identical;
# 1 = a difference or a run that did not finish (the gate's result is NOT succeeded to this binary; register the gate again);
# 2 = a precondition failed (nothing was run). It does not write a first-run marker or a ledger and does not call analyze-null/real-*.
# Not to be run before the freeze (there is no frozen binary yet): this file was only checked with `bash -n`.
set -uo pipefail
orig_args=("$@")

usage() { echo "usage: $0 --binary FILE --binary-sha256 SHA256 --out NEWDIR [--workers 1|2]" >&2; exit 2; }

binary="" sha="" out="" workers=2
while [ $# -gt 0 ]; do
  case "$1" in
    --binary) [ $# -ge 2 ] || usage; binary="$2"; shift 2;;
    --binary-sha256) [ $# -ge 2 ] || usage; sha="$2"; shift 2;;
    --out) [ $# -ge 2 ] || usage; out="$2"; shift 2;;
    --workers) [ $# -ge 2 ] || usage; workers="$2"; shift 2;;
    *) usage;;
  esac
done
[ -n "$binary" ] && [ -n "$sha" ] && [ -n "$out" ] || usage
[ -f "$binary" ] || { echo "precondition: binary $binary is not a file" >&2; exit 2; }
case "$sha" in *[!0-9a-f]*|"") echo "precondition: --binary-sha256 is not lower-case hex" >&2; exit 2;; esac
[ "${#sha}" -eq 64 ] || { echo "precondition: --binary-sha256 is not 64 hex digits" >&2; exit 2; }
case "$workers" in 1|2) ;; *) echo "precondition: --workers must be 1 or 2" >&2; exit 2;; esac
[ ! -e "$out" ] || { echo "precondition: $out exists (a new directory is required)" >&2; exit 2; }

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
actual="$(sha256sum "$binary" | cut -d' ' -f1)"
[ "$actual" = "$sha" ] || { echo "precondition: binary sha256 $actual is not the designated $sha" >&2; exit 2; }
mkdir -p "$(dirname "$out")" && mkdir "$out" || { echo "precondition: cannot create $out" >&2; exit 2; }
out="$(cd "$out" && pwd)"
log="$out/run.log"

{
  echo "# or4-rule7 $(date -Is)"
  printf 'command: bash %q' "$0"; printf ' %q' "${orig_args[@]}"; echo
  echo "cwd: $PWD host: $(hostname) loadavg: $(cut -d' ' -f1-3 /proc/loadavg)"
  echo "binary: $binary sha256 $actual (designated $sha)"
  echo "python3: $(python3 -I -c 'import sys; print(sys.version.split()[0])') $(command -v python3)"
  echo "SLAM_REFACT_* in the environment: $(env | grep -c '^SLAM_REFACT_' || true)"
  python3 -I "$here/rule7.py" run --binary "$binary" --binary-sha256 "$sha" --out "$out" --workers "$workers"
  rc=$?
  echo "exit=$rc"
  echo "# done $(date -Is)"
  exit "$rc"
} 2>&1 | tee -a "$log"
rc=${PIPESTATUS[0]}

sums=(run.log)
[ ! -f "$out/compare-rule7.json" ] || sums+=(compare-rule7.json)
(cd "$out" && sha256sum "${sums[@]}" > SHA256SUMS)
exit "$rc"
