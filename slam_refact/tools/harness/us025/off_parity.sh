#!/bin/bash
# US-025 LC-5: loop close OFF exact-input invariance (before = no loop close, after = loop close compiled in
# but runtime-disabled). Compares two builds of the same engine family on the same inputs; it does not run the
# original SDK and proves nothing about ON behaviour.
#
# Usage:
#   off_parity.sh --out DIR --before-replay BIN --after-replay BIN [options]
#
# Stages (--stages a,b,c; default = every stage whose inputs were given):
#   replay24  the 24 exact-input bundles (list files, one bundle dir per line): run both binaries, compare
#             per bundle: trace sha (srStageMs removed), points-by-kind/keyframe/world-point-count sha,
#             worldpoints-sampled sha, any other output file sha, meta (wallSeconds removed), stage timing
#   parity9   the 9 F2 parity bundles (--parity-bundles DIR/<scenario>/<label>): same comparison; with
#             --frozen DIR/<scenario>/<label> the frozen-binary output is compared to "after" as well
#   wasm5     5 bundles through two WASM builds (--before-wasm DIR --after-wasm DIR --src TREE [--frozen-wasm DIR])
#   ctest     test name/status sets: --before-ctest-logs DIR (ctest-list.log ctest-unit.log ctest-evidence.log)
#             against --after-build DIR (runs ctest there unless --no-run-ctest); --expect FILE with lines
#             "+regex" (name allowed to be new) / "-regex" (name allowed to disappear)
#   mutate    detection power: --after-build DIR --after-src TREE patch map_builder.cc with each mutant,
#             rebuild slam_refact_replay (+ all tests when --mutant-ctest), run replay24 and compare against a
#             fresh run of the unmutated binary, restore the source and rebuild.
#
# Exit status: 0 only if every requested stage found no unexplained difference. wallSeconds/timing fields are
# reported, never used as a pass criterion.
set -u
PUBLIC_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"

OUT=""; BEFORE=""; AFTER=""; JOBS=2; STAGES=""
LISTS=""; PARITY_B=""; FROZEN=""; BWASM=""; AWASM=""; FWASM=""; SRC=""
BCTEST=""; ABUILD=""; ASRC=""; EXPECT=""; RUNCTEST=1; MUTCTEST=0; MUTANTS="A B C"
while [ $# -gt 0 ]; do
  case "$1" in
    --out) OUT=$2; shift 2;; --before-replay) BEFORE=$2; shift 2;; --after-replay) AFTER=$2; shift 2;;
    --jobs) JOBS=$2; shift 2;; --stages) STAGES=$2; shift 2;; --lists) LISTS=$2; shift 2;;
    --parity-bundles) PARITY_B=$2; shift 2;; --frozen) FROZEN=$2; shift 2;;
    --before-wasm) BWASM=$2; shift 2;; --after-wasm) AWASM=$2; shift 2;; --frozen-wasm) FWASM=$2; shift 2;;
    --src) SRC=$2; shift 2;; --before-ctest-logs) BCTEST=$2; shift 2;; --after-build) ABUILD=$2; shift 2;;
    --after-src) ASRC=$2; shift 2;; --expect) EXPECT=$2; shift 2;; --no-run-ctest) RUNCTEST=0; shift;;
    --mutant-ctest) MUTCTEST=1; shift;; --mutants) MUTANTS=$2; shift 2;;
    -h|--help) sed -n 2,30p "$0"; exit 0;;
    *) echo "unknown option $1" >&2; exit 2;;
  esac
done
case "$JOBS" in 1|2|3|4) ;; *) echo "--jobs must be 1..4" >&2; exit 2;; esac
[ -n "$OUT" ] || { echo "--out required" >&2; exit 2; }
mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)
# absolute paths (the stages cd around)
for v in BEFORE AFTER PARITY_B FROZEN BWASM AWASM FWASM SRC BCTEST ABUILD ASRC EXPECT; do
  [ -n "${!v}" ] && printf -v "$v" '%s' "$(realpath "${!v}")"
done
ABS_LISTS=""; for l in $LISTS; do ABS_LISTS="$ABS_LISTS $(realpath "$l")"; done; LISTS=$ABS_LISTS
RC=0
if [ -z "$STAGES" ]; then
  [ -n "$BEFORE" ] && [ -n "$AFTER" ] && [ -n "$LISTS" ] && STAGES="$STAGES,replay24"
  [ -n "$BEFORE" ] && [ -n "$AFTER" ] && [ -n "$PARITY_B" ] && STAGES="$STAGES,parity9"
  [ -n "$BWASM" ] && [ -n "$AWASM" ] && STAGES="$STAGES,wasm5"
  [ -n "$BCTEST" ] && [ -n "$ABUILD" ] && STAGES="$STAGES,ctest"
fi
want() { case ",$STAGES," in *",$1,"*) return 0;; esac; return 1; }

# ---------------------------------------------------------------- comparer
cat > "$OUT/_cmp.py" <<'PY'
import hashlib, json, pathlib, sys
a_dir, b_dir, name, strict_meta = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2]), sys.argv[3], sys.argv[4] == '1'
POINT_KEYS = ('i', 'worldPoints', 'wpAdded', 'wpRemoved', 'srPointsBySource', 'srKeyframes', 'srGroundMatches', 'srKeyframeCreated')

def sha(b): return hashlib.sha256(b).hexdigest()

def load(d):
    lines = (d / 'trace.jsonl').read_text().splitlines()
    rows = [json.loads(x) for x in lines]
    h = hashlib.sha256(); hp = hashlib.sha256(); stage = []
    for r in rows:
        st = r.pop('srStageMs', None)
        if st is not None: stage.append((st.get('mapBuilder', 0.0), bool(r.get('srKeyframeCreated'))))
        h.update(json.dumps(r, sort_keys=True).encode() + b'\n')
        hp.update(json.dumps({k: r.get(k) for k in POINT_KEYS}, sort_keys=True).encode() + b'\n')
    kinds = [0] * 5
    for r in rows:
        for i, v in enumerate(r.get('srPointsBySource') or []): kinds[i] += v
    last = rows[-1].get('srPointsBySource') if rows else None
    others = {p.name: sha(p.read_bytes()) for p in sorted(d.iterdir()) if p.is_file() and p.name not in ('trace.jsonl', 'meta.json')}
    meta = json.loads((d / 'meta.json').read_text()) if (d / 'meta.json').exists() else {}
    wall = meta.pop('wallSeconds', None)
    mb = sorted(x for x, _ in stage)
    kf = sorted(x for x, c in stage if c)
    def pct(v, q): return v[min(len(v) - 1, int(q * len(v)))] if v else None
    return dict(rows=len(rows), trace_sha=h.hexdigest(), points_sha=hp.hexdigest(), kinds_sum=kinds, kinds_last=last, others=others, meta=meta,
                wall=wall, mb_total_ms=sum(mb), mb_kf_p50=pct(kf, .5), mb_kf_p99=pct(kf, .99), mb_n_kf=len(kf))

res = []; bad = 0
keys = sorted(p.name for p in a_dir.iterdir() if p.is_dir())
for k in keys:
    if not (b_dir / k).is_dir(): res.append(dict(key=k, error='missing in B')); bad += 1; continue
    try:
        x, y = load(a_dir / k), load(b_dir / k)
    except Exception as e:
        res.append(dict(key=k, error=repr(e))); bad += 1; continue
    ok_files = x['others'] == y['others']
    meta_keys = sorted(q for q in set(x['meta']) | set(y['meta']) if x['meta'].get(q) != y['meta'].get(q))
    same = (x['trace_sha'] == y['trace_sha'] and x['points_sha'] == y['points_sha'] and ok_files and x['kinds_sum'] == y['kinds_sum']
            and (not strict_meta or not meta_keys))
    bad += 0 if same else 1
    res.append(dict(key=k, rows=x['rows'], identical=same, trace_sha=x['trace_sha'][:16], trace_sha_B=y['trace_sha'][:16],
                    points_identical=x['points_sha'] == y['points_sha'], kinds_last=x['kinds_last'], kinds_last_B=y['kinds_last'],
                    other_files_identical=ok_files, other_files=sorted(x['others']), meta_keys_differing=meta_keys,
                    wall_A=x['wall'], wall_B=y['wall'], mb_total_ms_A=round(x['mb_total_ms'], 1), mb_total_ms_B=round(y['mb_total_ms'], 1),
                    mb_kf_p50_A=x['mb_kf_p50'], mb_kf_p50_B=y['mb_kf_p50'], mb_kf_p99_A=x['mb_kf_p99'], mb_kf_p99_B=y['mb_kf_p99'], n_kf=x['mb_n_kf']))
n_same = sum(1 for r in res if r.get('identical'))
print(json.dumps(dict(name=name, compared=len(res), identical=n_same, differing=len(res) - n_same, rows=res), indent=1))
for r in res:
    if 'error' in r: print('%-48s ERROR %s' % (r['key'], r['error']), file=sys.stderr); continue
    print('%-48s %5d fr  %s trace %s / %s  points=%s files=%s  mb_total %8.1f -> %8.1f ms  wall %s -> %s' % (
        r['key'], r['rows'], 'SAME' if r['identical'] else 'DIFF', r['trace_sha'], r['trace_sha_B'], r['points_identical'],
        r['other_files_identical'], r['mb_total_ms_A'], r['mb_total_ms_B'], r['wall_A'] and round(r['wall_A'], 1), r['wall_B'] and round(r['wall_B'], 1)), file=sys.stderr)
print('%s: %d/%d identical' % (name, n_same, len(res)), file=sys.stderr)
sys.exit(0 if bad == 0 else 1)
PY

# ---------------------------------------------------------------- helpers
key_of() { echo "$(basename "$(dirname "$1")")__$(basename "$1")"; }

# run_set BIN DEST BUNDLE...   (jobs-way parallel; every output dir gets <key>.log next to it)
run_set() {
  local bin=$1 dest=$2; shift 2
  mkdir -p "$dest"
  printf '%s\n' "$@" | xargs -P "$JOBS" -I{} bash -c '
    b="{}"; k="$(basename "$(dirname "$b")")__$(basename "$b")"
    rm -rf "'"$dest"'/$k"
    "'"$bin"'" --bundle "$b" --out "'"$dest"'/$k" > "'"$dest"'/$k.log" 2>&1; echo "rc=$?" >> "'"$dest"'/$k.log"'
}

bundles_of_lists() { for l in $LISTS; do grep -v '^[[:space:]]*$' "$l"; done; }

compare_dirs() { # A B NAME STRICT -> json in $OUT/NAME.json, table in $OUT/NAME.txt
  python3 -I "$OUT/_cmp.py" "$1" "$2" "$3" "${4:-1}" > "$OUT/$3.json" 2> "$OUT/$3.txt"; local r=$?
  cat "$OUT/$3.txt"; return $r
}

sha_of() { sha256sum "$1" | cut -d' ' -f1; }

# ---------------------------------------------------------------- replay24
if want replay24; then
  echo "== replay24: before=$BEFORE after=$AFTER"
  mapfile -t BUNDLES < <(bundles_of_lists)
  echo "bundles: ${#BUNDLES[@]}"; echo "before sha256 $(sha_of "$BEFORE")  after sha256 $(sha_of "$AFTER")" | tee "$OUT/replay24.binaries.txt"
  run_set "$BEFORE" "$OUT/replay24/before" "${BUNDLES[@]}"
  run_set "$AFTER" "$OUT/replay24/after" "${BUNDLES[@]}"
  compare_dirs "$OUT/replay24/before" "$OUT/replay24/after" replay24-before-vs-after 1 || RC=1
fi

# ---------------------------------------------------------------- parity9
if want parity9; then
  echo "== parity9"
  P9="tum-room1-full:par tum-room1-lowtex:par flip6-s2:par flip6-s3-lowtex:par flip6-n1:par flip6-s2-absolute:par tum-room1-full-absolute:par flip6-s2:inj tum-room1-full:inj"
  PB=(); for it in $P9; do PB+=("$PARITY_B/${it%%:*}/${it##*:}"); done
  run_set "$BEFORE" "$OUT/parity9/before" "${PB[@]}"
  run_set "$AFTER" "$OUT/parity9/after" "${PB[@]}"
  compare_dirs "$OUT/parity9/before" "$OUT/parity9/after" parity9-before-vs-after 1 || RC=1
  if [ -n "$FROZEN" ]; then
    mkdir -p "$OUT/parity9/frozen"
    for it in $P9; do
      k="${it%%:*}__${it##*:}"; rm -rf "$OUT/parity9/frozen/$k"
      cp -r "$FROZEN/${it%%:*}/${it##*:}" "$OUT/parity9/frozen/$k"
    done
    compare_dirs "$OUT/parity9/frozen" "$OUT/parity9/after" parity9-frozen-vs-after 0 || RC=1
  fi
fi

# ---------------------------------------------------------------- wasm5
if want wasm5; then
  echo "== wasm5"
  cat > "$OUT/_wasm.mjs" <<'MJS'
import path from 'node:path';
import { createHash } from 'node:crypto';
import { createRequire } from 'node:module';
const [src, parity, ...dirsArg] = process.argv.slice(2);
const { readBundle, readPgm } = await import(path.join(src, 'slam_refact/engine/tests/wasm/bundle_io.mjs'));
const { replay } = await import(path.join(src, 'slam_refact/engine/tests/wasm/bundle_replay.mjs'));
const require = createRequire(import.meta.url);
const dirs = Object.fromEntries(dirsArg.filter(Boolean).map((a) => a.split('=')));
const items = [['tum-room1-full', 'par', 400], ['flip6-s2', 'par', 400], ['flip6-s2-absolute', 'par', 400], ['tum-room1-full', 'inj', 22], ['flip6-s3-lowtex', 'par', 380]];
const out = [];
for (const [s, l, n] of items) {
  const h = {};
  for (const [tag, dir] of Object.entries(dirs)) {
    const Module = await require(path.join(dir, 'slam_refact_engine.js'))();
    const t0 = performance.now();
    const rows = replay(Module, readBundle(`${parity}/${s}/${l}`), n, (f) => readPgm(f.image));
    const wall = performance.now() - t0;
    const x = createHash('sha256');
    for (const r of rows) { const pose = r.pose ? { ...r.pose } : r.pose; if (pose) delete pose.stageMs; x.update(JSON.stringify({ i: r.i, fresh: r.fresh, pose }) + '\n'); }
    h[tag] = { sha256: x.digest('hex'), rows: rows.length, fresh: rows.filter((r) => r.fresh).length, wallMs: Math.round(wall) };
  }
  out.push({ scenario: s, label: l, maxFrames: n, ...h, before_eq_after: h.before.sha256 === h.after.sha256, frozen_eq_after: h.frozen ? h.frozen.sha256 === h.after.sha256 : null });
}
console.log(JSON.stringify(out, null, 1));
MJS
  PARB="${PARITY_B:-${PUBLIC_REPO_ROOT}/build/slam-refact-f2-20261023/parity/bundles}"
  node "$OUT/_wasm.mjs" "$SRC" "$PARB" "before=$BWASM" "after=$AWASM" ${FWASM:+"frozen=$FWASM"} > "$OUT/wasm5.json" 2> "$OUT/wasm5.err"
  echo "node rc=$?"
  python3 -I - "$OUT/wasm5.json" <<'PY' || RC=1
import json, sys
r = json.load(open(sys.argv[1])); bad = 0
for x in r:
    ok = x['before_eq_after'] and x['frozen_eq_after'] is not False; bad += 0 if ok else 1
    print('%-20s %-4s %4d fr  before==after %s  frozen==after %s  sha %s' % (x['scenario'], x['label'], x['after']['rows'], x['before_eq_after'], x['frozen_eq_after'], x['after']['sha256'][:16]))
print('wasm5: %d/%d identical' % (len(r) - bad, len(r))); sys.exit(1 if bad else 0)
PY
fi

# ---------------------------------------------------------------- ctest
if want ctest; then
  echo "== ctest"
  CT="$OUT/ctest"; mkdir -p "$CT"
  if [ "$RUNCTEST" = 1 ]; then
    (cd "$ABUILD" && ctest -N > "$CT/ctest-list.log" 2>&1
     ctest -L unit --parallel "$JOBS" --output-on-failure > "$CT/ctest-unit.log" 2>&1; echo "CTEST_UNIT_DONE rc=$?" >> "$CT/ctest-unit.log"
     ctest -L '^evidence$' --parallel "$JOBS" --output-on-failure > "$CT/ctest-evidence.log" 2>&1; echo "CTEST_EVIDENCE_DONE rc=$?" >> "$CT/ctest-evidence.log")
  fi
  cat > "$OUT/_ctest.py" <<'PY'
import json, re, sys
before, after, expect = sys.argv[1], sys.argv[2], sys.argv[3]
def norm(n): return re.sub(r'#GetParam\(\)=.*$', '#GetParam()', re.sub(r'0x[0-9a-fA-F]+', '0xPTR', n))  # gtest prints raw object bytes (heap pointers) for non-printable params
def listing(p):
    return {norm(m.group(1)) for l in open(p) for m in [re.match(r'\s*Test\s+#\d+: (\S+)', l)] if m}
def statuses(p):
    d = {}
    for l in open(p):
        m = re.match(r'\s*\d+/\d+ Test\s+#\d+: (\S+) \.+\s*(\*\*\*)?(\w[\w ]*?)\s+[\d.]+ sec', l)
        if m: d[norm(m.group(1))] = m.group(3)
    return d
exp = [l.rstrip('\n') for l in open(expect)] if expect != '-' else []
plus = [re.compile(l[1:]) for l in exp if l.startswith('+')]; minus = [re.compile(l[1:]) for l in exp if l.startswith('-')]
rc = 0; rep = {}
for tag in ('list', 'unit', 'evidence'):
    b, a = '%s/ctest-%s.log' % (before, tag), '%s/ctest-%s.log' % (after, tag)
    if tag == 'list': B, A = listing(b), listing(a); SB = SA = {}
    else: SB, SA = statuses(b), statuses(a); B, A = set(SB), set(SA)
    added, removed = sorted(A - B), sorted(B - A)
    changed = sorted(n for n in A & B if SB.get(n) != SA.get(n))
    un_add = [n for n in added if not any(p.search(n) for p in plus)]
    un_rm = [n for n in removed if not any(p.search(n) for p in minus)]
    cnt = lambda S: {s: sum(1 for v in S.values() if v == s) for s in sorted(set(S.values()))}
    rep[tag] = dict(before=len(B), after=len(A), added=added, removed=removed, status_changed=changed, unexplained_added=un_add,
                    unexplained_removed=un_rm, before_status=cnt(SB), after_status=cnt(SA))
    print('%-8s before %d after %d added %d removed %d status-changed %d unexplained +%d -%d  %s -> %s' % (
        tag, len(B), len(A), len(added), len(removed), len(changed), len(un_add), len(un_rm), cnt(SB), cnt(SA)))
    for n in added: print('   +', n, '(explained)' if n not in un_add else '(UNEXPLAINED)')
    for n in removed: print('   -', n, '(explained)' if n not in un_rm else '(UNEXPLAINED)')
    for n in changed: print('   ~', n, SB.get(n), '->', SA.get(n))
    if un_add or un_rm or changed: rc = 1
json.dump(rep, open('ctest-compare.json', 'w'), indent=1)
sys.exit(rc)
PY
  (set -o pipefail; cd "$CT" && python3 -I "$OUT/_ctest.py" "$BCTEST" "$CT" "${EXPECT:--}" | tee "$CT/ctest-compare.txt") || RC=1
fi

# ---------------------------------------------------------------- mutate
if want mutate; then
  echo "== mutate (source $ASRC, build $ABUILD)"
  M="$OUT/mut"; mkdir -p "$M"
  F="$ASRC/slam_refact/engine/src/map_builder/map_builder.cc"
  cp "$F" "$M/map_builder.cc.orig"; ORIG_SHA=$(sha_of "$F")
  mapfile -t BUNDLES < <(bundles_of_lists)
  cat > "$M/_patch.py" <<'PY'
import sys
f, mut = sys.argv[1], sys.argv[2]
s = open(f).read()
def sub(old, new):
    global s
    assert s.count(old) == 1, (old, s.count(old)); s = s.replace(old, new)
A = ('#ifdef SLAM_REFACT_LOOP_CLOSE\n  if (!s.loop_close_enabled) {  // MUTANT A: descriptor computed although OFF\n'
     '    if (!loop_close_) loop_close_ = std::make_unique<LoopCloseState>();\n'
     '    ComputeKeyframeGlobalDescriptor(loop_close_.get(), &wm.MutableKeyframe(nk));\n  }\n#endif\n')
if mut in 'AB': sub('  wm.SetKeyframePose(nk, trk.pose);', A + '  wm.SetKeyframePose(nk, trk.pose);')
if mut == 'B':
    sub('  wm.AdjustDirtyPoints();  // 936.c:5758',
        '#ifdef SLAM_REFACT_LOOP_CLOSE\n  if (!s.loop_close_enabled && loop_close_) {  // MUTANT B: ring/cursor updated (full LoopClose, result discarded) although OFF\n'
        '    MapBuilderOutput discarded;\n    LoopClose(wm, loop_close_.get(), &discarded);\n  }\n#endif\n  wm.AdjustDirtyPoints();  // 936.c:5758')
if mut == 'C':
    sub('  wm.AdjustDirtyPoints();  // 936.c:5758',
        '  out->world_transform_updated = true;  // MUTANT C (positive control): identity world correction on every keyframe although OFF\n'
        '  out->world_correction = c8::HMatrixGen::i();\n  wm.AdjustDirtyPoints();  // 936.c:5758')
open(f, 'w').write(s)
PY
  restore() { cp "$M/map_builder.cc.orig" "$F"; }
  trap restore EXIT
  build() { (cd "$ABUILD" && flock "$PUBLIC_REPO_ROOT/build/c1-build.lock" cmake --build . -j "$JOBS" "$@") ; }
  # control: the unmutated "after" binary, run again in this stage (noise floor)
  mkdir -p "$M/ctrl"; cp "$ABUILD/harness/slam_refact_replay" "$M/ctrl/slam_refact_replay"
  run_set "$M/ctrl/slam_refact_replay" "$M/ctrl/replays" "${BUNDLES[@]}"
  for m in $MUTANTS; do
    echo "-- mutant $m"; mkdir -p "$M/$m"; restore; python3 -I "$M/_patch.py" "$F" "$m" || { echo "patch $m failed"; RC=1; continue; }
    diff -u "$M/map_builder.cc.orig" "$F" > "$M/$m/patch.diff"
    if [ "$MUTCTEST" = 1 ]; then
      build > "$M/$m/build.log" 2>&1 || { echo "build $m failed"; RC=1; restore; continue; }
      (cd "$ABUILD" && ctest -L unit --parallel "$JOBS" --output-on-failure > "$M/$m/ctest-unit.log" 2>&1; echo "CTEST_UNIT_DONE rc=$?" >> "$M/$m/ctest-unit.log")
    else
      build slam_refact_replay > "$M/$m/build.log" 2>&1 || { echo "build $m failed"; RC=1; restore; continue; }
    fi
    cp "$ABUILD/harness/slam_refact_replay" "$M/$m/slam_refact_replay"
    restore
    run_set "$M/$m/slam_refact_replay" "$M/$m/replays" "${BUNDLES[@]}"
    compare_dirs "$M/ctrl/replays" "$M/$m/replays" "mutant-$m-vs-ctrl" 1 || echo "mutant $m: DIFFERENCE detected (exit 1 from comparer)"
  done
  restore; trap - EXIT
  [ "$(sha_of "$F")" = "$ORIG_SHA" ] || { echo "source not restored"; RC=1; }
  build > "$M/rebuild-restored.log" 2>&1 || RC=1
fi

echo "off_parity: rc=$RC"
exit $RC
