#!/usr/bin/env bash
# Seals hand gate data sets and prints the values a registration pins. It never touches a set that is already sealed.
#
#   seal_gate_set.sh <dir>
#       Writes <dir>/SHA256SUMS (sha256 of every other file in the directory, sorted by name) and prints the sha256 of
#       SHA256SUMS itself: the value that the sealed.json of the v1 registration pins (US-021 HAND-G1..G3).
#
#   seal_gate_set.sh --held-values <gate data root>
#       Read-only. For registrations on the shared HELD device (sr_held, hand-us021.v2 and later): the root holds the sealed
#       data sets (g1, g2, g3, g3-tensors-1) as immediate subdirectories, each already sealed with the form above. Verifies
#       every set against its SHA256SUMS, then prints the JSON fragment of the manifest `held` object that depends on the data:
#         inputs_tree_sha256       -> held.inputs[<kind>]       (tree sha256 of the whole root, tools/prereg/sr_held.mjs treeSha)
#         sealed_file_sha256       -> held.sealed_file_sha256   (the SHA256SUMS of every set: report mode refuses a directory
#                                                               that contains one of them)
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "${1:-}" = "--held-values" ]; then
  root="${2:-}"
  root="${root%/}"
  [ -n "$root" ] && [ -d "$root" ] || { echo "usage: seal_gate_set.sh --held-values <gate data root>" >&2; exit 2; }
  sets=0
  for d in "$root"/*/; do
    [ -d "$d" ] || continue
    d="${d%/}"
    [ -f "$d/SHA256SUMS" ] || { echo "not sealed (no SHA256SUMS): $d" >&2; exit 2; }
    (cd "$d" && sha256sum --check --quiet SHA256SUMS) || { echo "set differs from its SHA256SUMS: $d" >&2; exit 2; }
    # the files listed are exactly the files present: the checksum list of the directory contents equals SHA256SUMS
    (cd "$d" && find . -maxdepth 1 -type f ! -name SHA256SUMS -printf '%f\n' | LC_ALL=C sort | xargs sha256sum) | cmp -s - "$d/SHA256SUMS" \
      || { echo "SHA256SUMS does not list exactly the files of $d" >&2; exit 2; }
    sets=$((sets + 1))
  done
  [ "$sets" -gt 0 ] || { echo "no data set below $root" >&2; exit 2; }
  exec env SR_HELD_MJS="$here/../prereg/sr_held.mjs" SR_HELD_ROOT="$root" node --input-type=module -e '
    import fs from "node:fs";
    import { pathToFileURL } from "node:url";
    const { treeSha, sha256 } = await import(pathToFileURL(process.env.SR_HELD_MJS).href);
    const root = process.env.SR_HELD_ROOT;
    const t = treeSha(root);
    if (t.sha === undefined) { console.error("tree sha256 failed: " + t.code); process.exit(2); }
    const names = fs.readdirSync(root, { withFileTypes: true }).filter((e) => e.isDirectory()).map((e) => e.name).sort();
    const files = new Set(names.map((n) => sha256(fs.readFileSync(`${root}/${n}/SHA256SUMS`))));
    console.log(JSON.stringify({ sets: names, inputs_tree_sha256: t.sha, sealed_file_sha256: [...files].sort() }, null, 1));
  '
fi

dir="${1:-}"
[ -d "$dir" ] || { echo "no such directory: $dir" >&2; exit 2; }
[ ! -e "$dir/SHA256SUMS" ] || { echo "already sealed: $dir/SHA256SUMS" >&2; exit 2; }
(cd "$dir" && find . -maxdepth 1 -type f ! -name SHA256SUMS -printf '%f\n' | LC_ALL=C sort | xargs sha256sum > SHA256SUMS)
sha256sum "$dir/SHA256SUMS" | cut -d' ' -f1
