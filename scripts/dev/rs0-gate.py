#!/usr/bin/env python3
"""RS-0 parity gate with parallel replays and a content-addressed replay cache.

Same items, compared fields and exclusions as build/audit-run1-20261115/rs0/scripts/run-rs0.sh:
  RS-0b native parity  frozen binary vs new binary per bundle: trace rows without srStageMs, worldpoints-sampled
                       bytes, meta keys (only contractVersion / wallSeconds may differ).
  RS-0d OFF determinism two independent runs (rep a, rep b) of the new binary: trace rows without srStageMs, meta
                       without wallSeconds, every other output file byte-identical.
  RS-0c WASM parity    frozen vs new WASM engine, pose rows hashed without wallMs / pose.stageMs.
  RS-0a exact-input    slam_refact/tools/harness/divergence/exact_input_report.py on the two exact-input lists;
                       verdict: no blocked case and every fraction_within == 1.

Speedups (nothing above is relaxed):
  * every replay is an independent process; all of them (native and WASM) run in one pool, longest first;
  * a replay's outputs are stored under <cache>/runs/<key>, key = sha256 of (tool sha256 incl. linked shared
    libraries, bundle content hash incl. every referenced image / pyramid file, frame cap, rep tag, scrubbed env).
    A key that already has a successful run is reused instead of re-run: frozen references are computed once,
    unchanged new binaries are not re-run (pass --fresh to re-run every new-binary replay);
  * --tier fast cuts every native bundle to its first N input frames (frozen and new replay the same cut bundle)
    and caps WASM items at M frames. The fast tier is for everyday iterations; --tier full is the merge gate.

  python3 scripts/dev/rs0-gate.py --tier fast|full --new-native <slam_refact_replay> [--new-wasm <dir>] --out <new dir>
      [--root <repo with build/>] [--jobs N] [--fresh] [--sections parity,off,wasm,exact] [--plan-only]
Replays run with every SLAM_REFACT_* / SR_* / TRKPROBE_* environment variable removed (accuracy mode OFF).
"""
import argparse
import concurrent.futures
import datetime
import hashlib
import json
import os
import pathlib
import resource
import shutil
import subprocess
import sys
import threading
import time
import collections

CODE_ROOT = pathlib.Path(__file__).resolve().parents[2]
EXACT_TOOL = CODE_ROOT / 'slam_refact/tools/harness/divergence/exact_input_report.py'
WASM_HASH = CODE_ROOT / 'scripts/dev/rs0-wasm-hash.mjs'
WASM_HELPERS = [CODE_ROOT / 'slam_refact/engine/tests/wasm/bundle_io.mjs',
                CODE_ROOT / 'slam_refact/engine/tests/wasm/bundle_replay.mjs']
CACHE_SCHEMA = 'rs0-cache-v2'  # v2: stalled runs are never cached (v1 entries are not reused)
SCRUB_PREFIXES = ('SLAM_REFACT_', 'SR_', 'TRKPROBE_')
ENV_TAG = 'scrub:' + ','.join(SCRUB_PREFIXES)
META_MAY_DIFFER = ('contractVersion', 'wallSeconds')
SECTIONS = ('parity', 'off', 'wasm', 'exact')
# A frame whose stage time reached a solver's wall-clock cap in that stage may have a time-cut solve (the vendored
# pose estimation in mapTracker stops at 1.0 s, iterative BA in mapBuilder at 0.2 s). Unloaded frames stay below 0.45 s.
# Such runs are reported (stalled_replays) and never cached; verdicts are not changed.
STALL_CAPS_MS = {'mapTracker': 1000.0, 'mapBuilder': 200.0}
WASM_COST_PER_FRAME = 1.6  # LPT estimate relative to one native frame (RS-0 2026-11-15 timings)


def now():
    return datetime.datetime.now().astimezone().isoformat(timespec='seconds')


def scrubbed_env():
    return {k: v for k, v in os.environ.items() if not k.startswith(SCRUB_PREFIXES)}


def stall_frames_in_trace(trace_path):
    n = 0
    with open(trace_path) as f:
        for line in f:
            st = json.loads(line).get('srStageMs') or {}
            if isinstance(st, dict) and any((st.get(k) or 0) >= c for k, c in STALL_CAPS_MS.items()):
                n += 1
    return n


def wait_for_memory(min_free_gb, log=print):
    """Blocks while MemAvailable is below min_free_gb (replays stalled by memory pressure can hit solver time caps)."""
    warned = False
    while min_free_gb > 0:
        try:
            info = dict(l.split(':', 1) for l in open('/proc/meminfo'))
            avail_gb = int(info['MemAvailable'].split()[0]) / (1 << 20)
        except (OSError, KeyError, ValueError):
            return
        if avail_gb >= min_free_gb:
            return
        if not warned:
            log('[rs0] waiting: MemAvailable %.1f GB < %.1f GB' % (avail_gb, min_free_gb))
            warned = True
        time.sleep(2)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def sha256_json(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True).encode()).hexdigest()


class FileHasher:
    """sha256 of files, memoized on (path, size, mtime_ns, inode) in a JSON file."""

    def __init__(self, memo_path=None):
        self.memo_path = memo_path
        self.memo = {}
        self.lock = threading.Lock()
        if memo_path and memo_path.exists():
            try:
                self.memo = json.loads(memo_path.read_text())
            except ValueError:
                self.memo = {}

    def sha(self, path):
        p = os.path.abspath(path)
        st = os.stat(p)
        stamp = [st.st_size, st.st_mtime_ns, st.st_ino]
        with self.lock:
            hit = self.memo.get(p)
        if hit and hit[:3] == stamp:
            return hit[3]
        digest = sha256_file(p)
        with self.lock:
            self.memo[p] = stamp + [digest]
        return digest

    def save(self):
        if not self.memo_path:
            return
        self.memo_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.memo_path.with_suffix('.tmp%d' % os.getpid())
        with self.lock:
            tmp.write_text(json.dumps(self.memo))
        os.replace(tmp, self.memo_path)


def frame_rows(bundle):
    return [l.split('\t') for l in (pathlib.Path(bundle) / 'frames.tsv').read_text().splitlines() if l]


def referenced_files(bundle, cwd):
    """Files named by frames.tsv (images, injected pyramids), in first-use order."""
    seen, out = set(), []
    for row in frame_rows(bundle):
        for field in row[1:]:
            if '/' in field and field not in seen:
                p = pathlib.Path(field) if os.path.isabs(field) else pathlib.Path(cwd) / field
                if p.is_file():
                    seen.add(field)
                    out.append(p)
    return out


def bundle_hash(bundle, hasher, cwd, pool=None):
    bundle = pathlib.Path(bundle)
    h = hashlib.sha256()
    for name in sorted(os.listdir(bundle)):
        p = bundle / name
        if p.is_file():
            h.update(('%s\t%s\n' % (name, hasher.sha(p))).encode())
    refs = referenced_files(bundle, cwd)
    digests = list(pool.map(hasher.sha, refs)) if pool else [hasher.sha(p) for p in refs]
    for d in digests:
        h.update(('ref\t%s\n' % d).encode())
    return h.hexdigest()


def processed_frames(bundle, cap=None):
    rows = [r for r in frame_rows(bundle) if r[0] == 'frame']
    if cap is not None:
        rows = rows[:cap]
    return sum(1 for r in rows if len(r) > 5 and r[5] == '1')


def truncate_bundle(src, n_frames, dest_root, src_hash):
    """Copy of `src` whose frames.tsv keeps the bootstrap row and the first n_frames input frames.

    Sensor rows are kept whole: replay_main.cc pushes a sensor row only before a frame with a later time, so the
    replay of the cut bundle is the replay of the first n_frames input frames. The copy keeps <scenario>/<label>.
    """
    src = pathlib.Path(src)
    dest = pathlib.Path(dest_root) / ('%s-f%d' % (src_hash[:20], n_frames)) / src.parent.name / src.name
    if (dest / 'frames.tsv').exists():
        return dest
    tmp = dest.parent / ('.tmp-%s-%d-%d' % (src.name, os.getpid(), threading.get_ident()))
    if tmp.exists():
        shutil.rmtree(tmp)
    tmp.mkdir(parents=True)
    for name in sorted(os.listdir(src)):
        if name != 'frames.tsv' and (src / name).is_file():
            shutil.copyfile(src / name, tmp / name)
    kept, frames = [], 0
    for line in (src / 'frames.tsv').read_text().splitlines(keepends=True):
        if line.startswith('frame\t'):
            if frames == n_frames:
                break
            frames += 1
        kept.append(line)
    (tmp / 'frames.tsv').write_text(''.join(kept))
    try:
        os.rename(tmp, dest)
    except OSError:  # another runner made it first
        shutil.rmtree(tmp)
    return dest


def linked_libraries(binary):
    try:
        out = subprocess.run(['ldd', str(binary)], capture_output=True, text=True, timeout=60).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    libs = []
    for line in out.splitlines():
        parts = line.split('=>')
        target = (parts[1] if len(parts) > 1 else parts[0]).strip().split(' (')[0].strip()
        if target.startswith('/') and os.path.isfile(target):
            libs.append(target)
    return sorted(set(libs))


def native_tool_id(binary, hasher):
    libs = linked_libraries(binary)
    return {'binary': str(binary), 'sha256': hasher.sha(binary),
            'libs_sha256': sha256_json([[l, hasher.sha(l)] for l in libs]), 'libs': len(libs)}


def wasm_tool_id(engine_dir, hasher):
    d = pathlib.Path(engine_dir)
    parts = {'js': hasher.sha(d / 'slam_refact_engine.js'),
             'wasm': hasher.sha(d / 'slam_refact_engine.wasm') if (d / 'slam_refact_engine.wasm').exists() else None,
             'hash_script': hasher.sha(WASM_HASH), 'helpers': [hasher.sha(p) for p in WASM_HELPERS],
             'node': subprocess.run(['node', '--version'], capture_output=True, text=True).stdout.strip()}
    return {'dir': str(d), 'sha256': sha256_json(parts), 'parts': parts}


def run_key(kind, tool_sha, bundle_h, extra):
    return sha256_json([CACHE_SCHEMA, kind, tool_sha, bundle_h, ENV_TAG, extra])


class Cache:
    def __init__(self, root):
        self.root = pathlib.Path(root)
        self.runs = self.root / 'runs'
        self.tmp = self.root / 'tmp'
        self.failed = self.root / 'failed'

    def get(self, key):
        p = self.runs / key / 'result.json'
        if not p.exists():
            return None
        r = json.loads(p.read_text())
        return r if r.get('rc') == 0 else None

    def store(self, key, tmp_dir, result, replace=False):
        """Moves a finished run into the cache. replace=True (forced re-run): the run just made becomes the cached
        one and the previous entry moves to superseded/ (never compared again)."""
        (tmp_dir / 'result.json').write_text(json.dumps(result, indent=1) + '\n')
        if result['rc'] != 0:
            dest = self.failed / ('%s-%d' % (key, time.time_ns()))
        elif result.get('stall_frames'):
            dest = self.root / 'unstable' / ('%s-%d' % (key, time.time_ns()))
        else:
            dest = self.runs / key
            if replace and dest.exists():
                old = self.root / 'superseded' / ('%s-%d' % (key, time.time_ns()))
                old.parent.mkdir(parents=True, exist_ok=True)
                os.rename(dest, old)
        dest.parent.mkdir(parents=True, exist_ok=True)
        try:
            os.rename(tmp_dir, dest)
        except OSError:  # same key finished by another runner meanwhile: compare that run, keep this one aside
            if replace:
                raise
            aside = self.root / 'superseded' / ('%s-%d' % (key, time.time_ns()))
            aside.parent.mkdir(parents=True, exist_ok=True)
            os.rename(tmp_dir, aside)
        return dest


def execute_task(task, cache, cwd, min_free_gb=0):
    wait_for_memory(min_free_gb)
    tmp = cache.tmp / ('%s.%d.%d' % (task['key'], os.getpid(), threading.get_ident()))
    if tmp.exists():
        shutil.rmtree(tmp)
    tmp.mkdir(parents=True)
    t0 = time.monotonic()
    if task['kind'] == 'native':
        with open(tmp / 'replay.log', 'w') as log:
            rc = subprocess.call([task['tool'], '--bundle', task['bundle'], '--out', str(tmp / 'out')], cwd=cwd,
                                 env=scrubbed_env(), stdout=log, stderr=subprocess.STDOUT)
        payload = {'stall_frames': stall_frames_in_trace(tmp / 'out/trace.jsonl')} if rc == 0 else {}
    else:
        with open(tmp / 'hash.out', 'w') as out, open(tmp / 'replay.log', 'w') as log:
            rc = subprocess.call(['node', str(WASM_HASH), task['tool'], task['bundle'], str(task['max_frames'])],
                                 cwd=cwd, env=scrubbed_env(), stdout=out, stderr=log)
        payload = {}
        if rc == 0:
            lines = [l for l in (tmp / 'hash.out').read_text().splitlines() if l.strip()]
            try:
                h = json.loads(lines[-1])
                payload = {'stall_frames': h.pop('stallFrames', 0), 'hash': h}
            except (IndexError, ValueError):
                rc = 97
    result = dict(schema=CACHE_SCHEMA, key=task['key'], kind=task['kind'], role=task['role'], rc=rc,
                  seconds=round(time.monotonic() - t0, 3), finished=now(), tool=task['tool'], tool_sha256=task['tool_sha'],
                  bundle=task['bundle'], bundle_hash=task['bundle_hash'], extra=task['extra'], env=ENV_TAG, **payload)
    final = cache.store(task['key'], tmp, result, replace=task.get('replace', False))
    return dict(result, dir=str(final))


def run_tasks(tasks, cache, jobs, cwd, fresh_roles=(), log=print, min_free_gb=0):
    """Runs every task whose key has no cached successful run; returns {task id: result}. Longest first."""
    results, pending = {}, {}
    for t in tasks:
        forced = t['role'] in fresh_roles
        hit = None if forced else cache.get(t['key'])
        if hit is not None:
            results[t['id']] = dict(hit, dir=str(cache.runs / t['key']), cached=True)
        else:
            pending.setdefault(t['key'], []).append(dict(t, replace=forced))
    order = sorted(pending.values(), key=lambda g: -g[0]['est'])
    log('[rs0] %d replays: %d cached, %d to run (%d jobs)' % (len(tasks), len(results), len(order), jobs))
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = {ex.submit(execute_task, g[0], cache, cwd, min_free_gb): g for g in order}
        for f in concurrent.futures.as_completed(futs):
            r = f.result()
            for t in futs[f]:
                results[t['id']] = dict(r, cached=False)
            log('[rs0] %s rc=%d %.1fs%s %s' % (futs[f][0]['id'], r['rc'], r['seconds'],
                                             ' STALL=%d' % r['stall_frames'] if r.get('stall_frames') else '', r['bundle']))
    return results


# ---- comparisons: same fields and exclusions as rs0/scripts/{parity,off}-compare.py and wasm-parity.mjs ----

def parity_compare(pairs):
    """pairs: [(scenario, label, frozen_dir, new_dir)] -> rows of rs0/scripts/parity-compare.py."""
    rows = []
    for s, l, fz, d in sorted(pairs, key=lambda x: (x[0], x[1])):
        fz, d = pathlib.Path(fz), pathlib.Path(d)
        diffkeys = collections.Counter()
        n = 0
        a_lines = (fz / 'trace.jsonl').read_text().splitlines()
        b_lines = (d / 'trace.jsonl').read_text().splitlines()
        if len(a_lines) != len(b_lines):  # the old script stopped on an assert here: never identical
            rows.append(dict(scenario=s, label=l, frames_frozen=len(a_lines), frames_new=len(b_lines),
                             error='trace row count differs', identical=False, worldpoints_sha_identical=False,
                             meta_keys_differing=[]))
            continue
        ha, hb = hashlib.sha256(), hashlib.sha256()
        for x, y in zip(a_lines, b_lines):
            jx, jy = json.loads(x), json.loads(y)
            n += 1
            for k in set(jx) | set(jy):
                if jx.get(k) != jy.get(k):
                    diffkeys[k] += 1
            for j, h in ((jx, ha), (jy, hb)):
                j = dict(j)
                j.pop('srStageMs', None)
                h.update(json.dumps(j, sort_keys=True).encode() + b'\n')
        wp_same = sha256_file(fz / 'worldpoints-sampled.jsonl') == sha256_file(d / 'worldpoints-sampled.jsonl')
        ma, mb = json.loads((fz / 'meta.json').read_text()), json.loads((d / 'meta.json').read_text())
        mdiff = sorted(k for k in set(ma) | set(mb) if ma.get(k) != mb.get(k))
        rows.append(dict(scenario=s, label=l, frames=n, trace_sha_without_srStageMs_frozen=ha.hexdigest(),
                         trace_sha_without_srStageMs_new=hb.hexdigest(), identical=ha.hexdigest() == hb.hexdigest(),
                         fields_differing=dict(diffkeys), worldpoints_sha_identical=wp_same, meta_keys_differing=mdiff))
    return rows


def off_compare(pairs):
    """pairs: [(fixture, rep_a_dir, rep_b_dir)] -> rows of rs0/scripts/off-compare.py."""
    rows = []
    for name, a, d in sorted(pairs, key=lambda x: x[0]):
        a, d = pathlib.Path(a), pathlib.Path(d)
        la = (a / 'trace.jsonl').read_text().splitlines()
        lb = (d / 'trace.jsonl').read_text().splitlines()
        diff_rows = 0
        keys = {}
        for x, y in zip(la, lb):
            jx, jy = json.loads(x), json.loads(y)
            ks = [k for k in set(jx) | set(jy) if jx.get(k) != jy.get(k)]
            for k in ks:
                keys[k] = keys.get(k, 0) + 1
            if [k for k in ks if k != 'srStageMs']:
                diff_rows += 1
        diff_rows += abs(len(la) - len(lb))
        ma, mb = json.loads((a / 'meta.json').read_text()), json.loads((d / 'meta.json').read_text())
        mdiff = sorted(k for k in set(ma) | set(mb) if ma.get(k) != mb.get(k))
        files = sorted({p.name for p in a.iterdir()} | {p.name for p in d.iterdir()})
        other = {f: (a / f).exists() and (d / f).exists() and sha256_file(a / f) == sha256_file(d / f)
                 for f in files if f not in ('trace.jsonl', 'meta.json')}
        rows.append(dict(fixture=name, rows_a=len(la), rows_b=len(lb), trace_diff_rows_excl_srStageMs=diff_rows,
                         fields_differing=keys, meta_keys_differing=mdiff,
                         meta_diff_excl_wallSeconds=[k for k in mdiff if k != 'wallSeconds'], other_files_identical=other,
                         zero_diff=diff_rows == 0 and not [k for k in mdiff if k != 'wallSeconds'] and all(other.values())))
    return rows


def wasm_rows(items, hashes):
    """items: [(scenario, label, maxFrames)], hashes: {(s, l, n, tag): {sha256, rows, fresh}} -> wasm-parity rows."""
    out = []
    for s, l, n in items:
        fz, nw = hashes[(s, l, n, 'frozen')], hashes[(s, l, n, 'new')]
        out.append({'scenario': s, 'label': l, 'maxFrames': n, 'frozen': fz, 'new': nw,
                    'identical': fz['sha256'] == nw['sha256']})
    return out


def verdict_parity(rows):
    ok = bool(rows) and all(r['identical'] and r['worldpoints_sha_identical']
                            and set(r['meta_keys_differing']) <= set(META_MAY_DIFFER) for r in rows)
    return 'VERIFIED' if ok else 'FAIL'


def verdict_off(rows):
    return 'VERIFIED' if rows and all(r['zero_diff'] for r in rows) else 'FAIL'


def verdict_wasm(rows):
    return 'VERIFIED' if rows and all(r['identical'] for r in rows) else 'FAIL'


def verdict_exact(reports):
    cases = [c for rep in reports for c in rep['cases']]
    ok = bool(cases) and all(c.get('result') is not None and all(v == 1 for v in c['result']['fraction_within'].values())
                             for c in cases)
    return 'VERIFIED' if ok else 'FAIL'


def quarantine_touches(root, links, marker):
    touched = []
    for link in links:
        p = pathlib.Path(root) / 'build' / link
        if p.is_symlink():
            out = subprocess.run(['find', '-P', str(p) + '/', '-newer', str(marker), '-type', 'f'],
                                 capture_output=True, text=True).stdout
            touched += [l for l in out.splitlines() if l]
    return touched


def link(src, dst):
    """Hard-linked copy of a run's output files: the gate's evidence stays fixed even if the cache entry is later
    replaced (--fresh, RS-0d reps) and moved to superseded/."""
    dst = pathlib.Path(dst)
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(src, dst, copy_function=os.link)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tier', choices=('fast', 'full'), required=True)
    ap.add_argument('--new-native', type=pathlib.Path)
    ap.add_argument('--new-wasm', type=pathlib.Path, help='dir with slam_refact_engine.js/.wasm')
    ap.add_argument('--frozen-native', type=pathlib.Path)
    ap.add_argument('--frozen-wasm', type=pathlib.Path)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    ap.add_argument('--root', type=pathlib.Path, default=CODE_ROOT, help='repository root holding build/ (data)')
    ap.add_argument('--manifest', type=pathlib.Path, default=CODE_ROOT / 'scripts/dev/rs0-gate.json')
    ap.add_argument('--cache', type=pathlib.Path, help='default <root>/build/rs0-cache')
    ap.add_argument('--jobs', type=int, default=min(24, max(1, (os.cpu_count() or 2) * 3 // 4)))
    ap.add_argument('--sections', default=','.join(SECTIONS))
    ap.add_argument('--fresh', action='store_true', help='re-run every new-binary / new-WASM replay')
    ap.add_argument('--plan-only', action='store_true', help='print the plan (cached / to run) and exit')
    ap.add_argument('--label', default='')
    ap.add_argument('--min-free-gb', type=float, default=4.0, help='start a replay only while MemAvailable >= this')
    ap.add_argument('--allow-unpinned-frozen', action='store_true', help='frozen reference differs from the pin (never a merge gate)')
    a = ap.parse_args(argv)
    cpus = os.cpu_count() or 1
    if not 1 <= a.jobs <= cpus:
        raise SystemExit('--jobs must be 1..%d' % cpus)
    sections = [s for s in a.sections.split(',') if s]
    if not sections or set(sections) - set(SECTIONS):
        raise SystemExit('--sections: subset of ' + ','.join(SECTIONS))
    root = a.root.resolve()
    man = json.loads(a.manifest.read_text())
    rp = lambda p: pathlib.Path(p) if os.path.isabs(str(p)) else root / p
    frozen_native = (a.frozen_native or rp(man['defaults']['frozen_native'])).resolve()
    frozen_wasm = (a.frozen_wasm or rp(man['defaults']['frozen_wasm'])).resolve()
    needs_native = bool(set(sections) & {'parity', 'off', 'exact'})
    if needs_native and not a.new_native:
        raise SystemExit('--new-native is required for sections parity/off/exact')
    if 'wasm' in sections and not a.new_wasm:
        raise SystemExit('--new-wasm is required for section wasm')
    if not a.plan_only:
        if a.out.exists():
            raise SystemExit('refusing to reuse existing --out ' + str(a.out))
        a.out.mkdir(parents=True)
    out = a.out.resolve()
    cache = Cache((a.cache or root / 'build/rs0-cache').resolve())
    fast = a.tier == 'fast'
    nf, wf = man['fast']['native_frames'], man['fast']['wasm_frames']
    t_start = time.monotonic()
    started = now()
    marker = out / 'started.marker' if not a.plan_only else None
    if marker:
        marker.write_text(started + '\n')
    hasher = FileHasher(cache.root / 'filehash.json')
    stages = {}

    # ---- plan
    t0 = time.monotonic()
    hpool = concurrent.futures.ThreadPoolExecutor(8)
    bundle_memo = {}

    def bundle_for(src):
        """(--bundle argument, content hash, processed frames) for the tier.

        The full tier passes the bundle path as the manifest / list spells it (relative to --root, as run-rs0.sh
        did), so meta.json "bundle" is byte-identical to the old evidence; the argument is part of the cache key.
        """
        key = str(src)
        if key not in bundle_memo:
            p = rp(src)
            h = bundle_hash(p, hasher, root, hpool)
            if fast:
                b = truncate_bundle(p, nf, cache.root / 'bundles', h)
                bundle_memo[key] = (str(b), bundle_hash(b, hasher, root, hpool), processed_frames(b))
            else:
                bundle_memo[key] = (key, h, processed_frames(p))
        return bundle_memo[key]

    tools = {}
    if needs_native:
        tools['frozen-native'] = native_tool_id(frozen_native, hasher)
        tools['new-native'] = native_tool_id(a.new_native.resolve(), hasher)
    if 'wasm' in sections:
        tools['frozen-wasm'] = wasm_tool_id(frozen_wasm, hasher)
        tools['new-wasm'] = wasm_tool_id(a.new_wasm.resolve(), hasher)
    # The frozen reference is pinned (manifest defaults.frozen_pins), and may never be the build under test.
    pins = man['defaults'].get('frozen_pins', {})
    unpinned = []
    if 'frozen-native' in tools:
        if pins.get('native') not in (None, tools['frozen-native']['sha256']):
            unpinned.append('frozen native sha256 %s != pin %s' % (tools['frozen-native']['sha256'], pins['native']))
        if tools['frozen-native']['sha256'] == tools['new-native']['sha256']:
            raise SystemExit('the new native binary is the frozen reference (same sha256)')
    if 'frozen-wasm' in tools:
        fp, np_ = tools['frozen-wasm']['parts'], tools['new-wasm']['parts']
        for part in ('js', 'wasm'):
            if pins.get('wasm_' + part) not in (None, fp[part]):
                unpinned.append('frozen WASM %s sha256 %s != pin %s' % (part, fp[part], pins['wasm_' + part]))
        if (fp['js'], fp['wasm']) == (np_['js'], np_['wasm']):
            raise SystemExit('the new WASM engine is the frozen reference (same sha256)')
    if unpinned and not a.allow_unpinned_frozen:
        raise SystemExit('; '.join(unpinned) + ' (pass --allow-unpinned-frozen for a non-gate comparison)')
    tasks = []

    def add(tid, kind, role, tool, bundle, extra, est, max_frames=None):
        b, bh, _ = bundle
        t = tools[tool]
        tool_path = t['binary'] if kind == 'native' else t['dir']
        tool_sha = sha256_json([t['sha256'], t.get('libs_sha256')])
        tasks.append(dict(id=tid, kind=kind, role=role, tool=tool_path, tool_sha=tool_sha, bundle=str(b), bundle_hash=bh,
                          extra=extra, est=est, max_frames=max_frames,
                          key=run_key(kind, tool_sha, bh, [extra, str(b)])))

    if 'parity' in sections:
        for s, l in man['parity']['items']:
            b = bundle_for(pathlib.Path(man['parity']['bundle_root']) / s / l)
            for tag in ('frozen', 'new'):
                add('parity/%s/%s/%s' % (tag, s, l), 'native', tag, '%s-native' % tag, b, {}, b[2])
    if 'off' in sections:
        for rep in man['off']['reps']:
            for name, src in man['off']['items']:
                b = bundle_for(src)
                add('off/%s/%s' % (rep, name), 'native', 'det', 'new-native', b, {'rep': rep}, b[2])
    exact_lists = []
    if 'exact' in sections:
        for n, lst in enumerate(man['exact']['lists'], 1):
            entries = [l.strip() for l in rp(lst).read_text().splitlines() if l.strip() and not l.startswith('#')]
            exact_lists.append((n, lst, entries))
            for e in entries:
                b = bundle_for(e)
                add('exact/%d/%s/%s' % (n, pathlib.Path(e).parent.name, pathlib.Path(e).name), 'native', 'new',
                    'new-native', b, {}, b[2])
    wasm_items = []
    if 'wasm' in sections:
        for s, l, n in man['wasm']['items']:
            n = min(n, wf) if fast else n
            wasm_items.append((s, l, n))
            src = rp(pathlib.Path(man['wasm']['bundle_root']) / s / l)
            bh = bundle_hash(src, hasher, root, hpool)
            for tag in ('frozen', 'new'):
                add('wasm/%s/%s/%s/%d' % (tag, s, l, n), 'wasm', tag, '%s-wasm' % tag, (src, bh, None),
                    {'maxFrames': n}, n * WASM_COST_PER_FRAME, max_frames=n)
    hpool.shutdown()
    hasher.save()
    stages['plan_hash_s'] = round(time.monotonic() - t0, 2)
    # RS-0d reps are two real runs on every gate run (as in run-rs0.sh); --fresh also re-runs every new replay.
    fresh_roles = ('det', 'new') if a.fresh else ('det',)
    if a.plan_only:
        todo = [t for t in tasks if t['role'] in fresh_roles or cache.get(t['key']) is None]
        print(json.dumps({'tier': a.tier, 'replays': len(tasks), 'to_run': len(todo), 'cached': len(tasks) - len(todo),
                          'to_run_ids': [t['id'] for t in todo], 'plan_s': stages['plan_hash_s']}, indent=1))
        return 0

    # ---- run
    t0 = time.monotonic()
    cpu0 = resource.getrusage(resource.RUSAGE_CHILDREN)
    res = run_tasks(tasks, cache, a.jobs, root, fresh_roles, min_free_gb=a.min_free_gb)
    cpu1 = resource.getrusage(resource.RUSAGE_CHILDREN)
    run_wall = time.monotonic() - t0
    child_cpu = (cpu1.ru_utime - cpu0.ru_utime) + (cpu1.ru_stime - cpu0.ru_stime)
    stages['replays_s'] = round(run_wall, 2)
    (out / 'runs.json').write_text(json.dumps([dict(id=t['id'], key=t['key'], cached=res[t['id']]['cached'],
                                                    rc=res[t['id']]['rc'], seconds=res[t['id']]['seconds'],
                                                    stall_frames=res[t['id']].get('stall_frames'),
                                                    dir=res[t['id']]['dir']) for t in tasks], indent=1) + '\n')
    failed = [t['id'] for t in tasks if res[t['id']]['rc'] != 0]
    summary = {'gate': 'RS-0 (%s tier)' % a.tier, 'label': a.label, 'started': started, 'tier': a.tier,
               'merge_gate': a.tier == 'full' and set(sections) == set(SECTIONS) and not unpinned, 'root': str(root), 'code_root': str(CODE_ROOT), 'cache': str(cache.root),
               'tools': tools, 'jobs': a.jobs, 'sections': sections, 'failed_replays': failed,
               'fast_frames': {'native': nf, 'wasm': wf} if fast else None}

    def rdir(tid):
        return pathlib.Path(res[tid]['dir']) / 'out'

    # ---- compare
    t0 = time.monotonic()
    if 'parity' in sections:
        pairs = []
        for s, l in man['parity']['items']:
            for tag in ('frozen', 'new'):
                link(rdir('parity/%s/%s/%s' % (tag, s, l)), out / 'parity' / tag / s / l)
            pairs.append((s, l, out / 'parity/frozen' / s / l, out / 'parity/new' / s / l))
        ok = not [f for f in failed if f.startswith('parity/')]
        rows = parity_compare(pairs) if ok else []
        (out / 'parity-compare.json').write_text(json.dumps(rows, indent=1) + '\n')
        summary['RS-0b'] = {'combos': len(rows), 'trace_identical': sum(r['identical'] for r in rows),
                            'worldpoints_identical': sum(r['worldpoints_sha_identical'] for r in rows),
                            'meta_keys_differing': sorted({k for r in rows for k in r['meta_keys_differing']}),
                            'verdict': verdict_parity(rows) if ok else 'FAIL'}
    if 'off' in sections:
        for rep in man['off']['reps']:
            for name, _ in man['off']['items']:
                link(rdir('off/%s/%s' % (rep, name)), out / 'off' / rep / name)
        ok = not [f for f in failed if f.startswith('off/')]
        ra, rb = man['off']['reps']
        rows = off_compare([(name, out / 'off' / ra / name, out / 'off' / rb / name) for name, _ in man['off']['items']]) if ok else []
        (out / 'off-compare.json').write_text(json.dumps(rows, indent=1) + '\n')
        summary['RS-0d'] = {'fixtures': len(rows), 'zero_diff': sum(r['zero_diff'] for r in rows),
                            'trace_diff_rows': sum(r['trace_diff_rows_excl_srStageMs'] for r in rows), 'reps': [ra, rb],
                            'verdict': verdict_off(rows) if ok else 'FAIL'}
    if 'wasm' in sections:
        ok = not [f for f in failed if f.startswith('wasm/')]
        rows = wasm_rows(wasm_items, {(s, l, n, tag): res['wasm/%s/%s/%s/%d' % (tag, s, l, n)]['hash']
                                      for s, l, n in wasm_items for tag in ('frozen', 'new')}) if ok else []
        (out / 'wasm-parity.json').write_text(json.dumps(rows, indent=1) + '\n')
        summary['RS-0c'] = {'items': len(rows), 'identical': sum(r['identical'] for r in rows),
                            'verdict': verdict_wasm(rows) if ok else 'FAIL'}
    stages['compare_s'] = round(time.monotonic() - t0, 2)
    if 'exact' in sections:
        t0 = time.monotonic()
        reports, rcs = [], []
        ok = not [f for f in failed if f.startswith('exact/')]
        for n, lst, entries in exact_lists:
            rep_dir = out / 'exact' / ('replays%d' % n)
            list_file = rp(lst)
            if fast:
                list_file = out / 'exact' / ('list%d.txt' % n)
                list_file.parent.mkdir(parents=True, exist_ok=True)
                list_file.write_text(''.join(str(bundle_for(e)[0]) + '\n' for e in entries))
            for e in entries:
                p = pathlib.Path(e)
                link(rdir('exact/%d/%s/%s' % (n, p.parent.name, p.name)), rep_dir / p.parent.name / p.name)
            cmd = [sys.executable, '-I', str(EXACT_TOOL), '--bundles', str(list_file), '--replays', str(rep_dir),
                   '--out', str(out / 'exact' / ('report%d' % n)), '--jobs', '1', '--label', '%s-list%d' % (a.label or a.tier, n)]
            if ok:  # every replay is in place: --run only records the binary (nothing is re-run)
                cmd += ['--run', '--binary', str(a.new_native.resolve())]
            with open(out / 'exact' / ('run%d.log' % n), 'w') as log:
                rc = subprocess.call(cmd, cwd=root, stdout=log, stderr=subprocess.STDOUT)
            rcs.append(rc)
            rp_json = out / 'exact' / ('report%d' % n) / 'report.json'
            if rc == 0 and rp_json.exists():
                reports.append(json.loads(rp_json.read_text()))
        cases = [c for r in reports for c in r['cases']]
        summary['RS-0a'] = {'cases': len(cases),
                            'all_fraction_within_1': sum(1 for c in cases if c.get('result') and
                                                         all(v == 1 for v in c['result']['fraction_within'].values())),
                            'blocked': sum(1 for c in cases if c.get('result') is None),
                            'ref_fresh_missing_in_engine': sum(c['result']['ref_fresh_missing_in_engine'] for c in cases
                                                               if c.get('result')),
                            'rc': rcs,
                            'verdict': verdict_exact(reports) if ok and rcs and all(r == 0 for r in rcs)
                            and len(reports) == len(exact_lists) else 'FAIL'}
        stages['exact_reports_s'] = round(time.monotonic() - t0, 2)

    # ---- quarantine (read-only evidence links) and summary
    t0 = time.monotonic()
    touched = quarantine_touches(root, man['defaults'].get('quarantine_links', []), marker)
    (out / 'quarantine-touch.txt').write_text(''.join(t + '\n' for t in touched))
    stages['quarantine_check_s'] = round(time.monotonic() - t0, 2)
    verdict_keys = {'parity': 'RS-0b', 'off': 'RS-0d', 'wasm': 'RS-0c', 'exact': 'RS-0a'}
    verdicts = [summary[verdict_keys[s]]['verdict'] for s in sections]
    summary['quarantine_touched'] = len(touched)
    stalled = [dict(id=t['id'], stall_frames=res[t['id']]['stall_frames']) for t in tasks if res[t['id']].get('stall_frames')]
    summary['stalled_replays'] = stalled
    if stalled:
        summary['stall_note'] = ('%d replays had frames at a solver wall-clock cap %s (process stalls: CPU / memory '
                                 'pressure); their outputs may differ from an unloaded run and were not cached. Verdicts '
                                 'are unchanged; a FAIL here needs a rerun on an unloaded machine before it is read as an '
                                 'engine change.' % (len(stalled), STALL_CAPS_MS))
    summary['overall'] = 'PASS' if all(v == 'VERIFIED' for v in verdicts) and not touched else 'FAIL'
    ran = [res[t['id']] for t in tasks if not res[t['id']]['cached']]
    summary['timing'] = dict(stages, total_s=round(time.monotonic() - t_start, 2), replays_total=len(tasks),
                             replays_run=len(ran), replays_cached=len(tasks) - len(ran),
                             replay_seconds_sum=round(sum(r['seconds'] for r in ran), 1),
                             child_cpu_s=round(child_cpu, 1),
                             mean_busy_cores=round(child_cpu / run_wall, 2) if run_wall > 0 else None,
                             longest_replay=max(((r['seconds'], r['bundle']) for r in ran), default=None))
    summary['finished'] = now()
    (out / 'SUMMARY.json').write_text(json.dumps(summary, indent=1) + '\n')
    ledger = cache.root / 'ledger.jsonl'
    with open(ledger, 'a') as f:
        f.write(json.dumps({'time': summary['finished'], 'tier': a.tier, 'merge_gate': summary['merge_gate'], 'out': str(out), 'overall': summary['overall'],
                            'verdicts': dict(zip(sections, verdicts)), 'total_s': summary['timing']['total_s'],
                            'new_native': tools.get('new-native', {}).get('sha256'),
                            'new_wasm': tools.get('new-wasm', {}).get('sha256')}) + '\n')
    print(json.dumps({'overall': summary['overall'], 'merge_gate': summary['merge_gate'], 'verdicts': dict(zip(sections, verdicts)),
                      'timing': summary['timing'], 'summary': str(out / 'SUMMARY.json')}, indent=1))
    return 0 if summary['overall'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
