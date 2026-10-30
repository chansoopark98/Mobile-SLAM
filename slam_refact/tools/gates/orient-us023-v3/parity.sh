#!/usr/bin/env bash
# US-023 criterion 3 (the portrait default path is unchanged), review N-3 / C-U23-1: the comparison of two exact-input replay runs of the
# 14 bundles is written to a FILE (it used to be shown on the console only).
#
#   bash slam_refact/tools/gates/orient-us023-v3/parity.sh --a DIR_A --b DIR_B --out NEWDIR
#
#   DIR_A / DIR_B   one directory per bundle with trace.jsonl and worldpoints-sampled.jsonl (slam_refact_replay output of the earlier
#                   and of the frozen build; binary.sha256 in the directory is quoted in the report)
#   NEWDIR          created here: compare.txt (the per-file comparison, ends with "ALL IDENTICAL rows N" or "DIFFERENCES"), run.log (this
#                   script's output including "exit=<code>"), SHA256SUMS
# Exit code 0 only when all 28 files are identical (timing fields excluded). It replays nothing; the two directories come from
# tools/harness/run_replay.sh or regress/run_exact.sh. This is a REPORT/verification item of the gate (gate md section 5), not a gate.
set -uo pipefail
orig_args=("$@")

usage() { echo "usage: $0 --a DIR_A --b DIR_B --out NEWDIR" >&2; exit 2; }
a="" b="" out=""
while [ $# -gt 0 ]; do
  case "$1" in
    --a) [ $# -ge 2 ] || usage; a="$2"; shift 2;;
    --b) [ $# -ge 2 ] || usage; b="$2"; shift 2;;
    --out) [ $# -ge 2 ] || usage; out="$2"; shift 2;;
    *) usage;;
  esac
done
[ -n "$a" ] && [ -n "$b" ] && [ -n "$out" ] || usage
[ -d "$a" ] && [ -d "$b" ] || { echo "precondition: --a and --b must be directories" >&2; exit 2; }
[ ! -e "$out" ] || { echo "precondition: $out exists (a new directory is required)" >&2; exit 2; }
mkdir -p "$(dirname "$out")" && mkdir "$out" || { echo "precondition: cannot create $out" >&2; exit 2; }
out="$(cd "$out" && pwd)"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

{
  echo "# orient parity $(date -Is)"
  printf 'command: bash %q' "$0"; printf ' %q' "${orig_args[@]}"; echo
  python3 -I "$here/../../re/runtime/orient/compare_regress.py" --a "$a" --b "$b" --out "$out/compare.txt"
  rc=$?
  echo "exit=$rc"
  exit "$rc"
} 2>&1 | tee -a "$out/run.log"
rc=${PIPESTATUS[0]}

(cd "$out" && sha256sum run.log compare.txt > SHA256SUMS)
exit "$rc"
