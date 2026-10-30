#!/usr/bin/env python3
"""RM-4e / v5 mutation run of the first-real-run hold (gate rm4c-chunk-gate.v4.md section 5; v5: the generic conditions are the shared device, mutated by tools/prereg/run_sr_held_mutants.py, so these mutants are the checks chunk-hold.mjs adds around it): chunk-hold.mjs and chunk-hold.test.mjs are copied under
--out (under build/) into a tree of the same shape (<out>/<name>/slam_refact/web-chunk/tests/), one single-site edit of chunk-hold.mjs per copy, then
`node --test chunk-hold.test.mjs` runs in the copy. A mutant is DETECTED when at least one test fails.

  python3 -I slam_refact/web-chunk/tests/mutants/run-hold-mutants.py --out build/.../rm4e-20261020/mutants-hold [--only HM1,HM9]
The v3 reviewer's mutants HM1 (pin form without `pinned`), HM9 (gate file location) and HM18 (second run by manifest) are conditions of the shared device now: sr_held pins the manifest
before the review, requires a bare gate file name and refuses first_real_run_at; the tests chunk-hold.test.mjs RM4E-H3 / H4 and the ALREADY_RUN case keep them as behaviour tests.
Two records of the same fact (manifest.pinned groups vs the live files, which the device also compares) are defence in depth: a mutant that removes only the group loop is
not listed because the device refuses the same cases first.
"""
import argparse
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[4]
TESTS = ROOT / 'slam_refact/web-chunk/tests'
H = 'chunk-hold.mjs'

M = {
    'H0-unmutated-copy': [],
    # what is gate data
    'HX1-roi-gate-kind-not-held': [("  if (mode === 'roi') return kind === 'gate';", "  if (mode === 'roi') return false;")],
    'HX2-registered-seed-not-gate-data': [("  return registered?.seed != null && !noTargets && parseSeed(seed) === registered.seed;", "  return false;")],
    'HX3-it-gate-kind-not-held': [("  if (kind === 'gate') return !noTargets;", "  if (kind === 'gate') return false;")],
    'HP5-parse-seed-accepts-anything': [("!/^\\s*[+-]?\\d+\\s*$/.test(text)", "false")],
    # arguments
    'HA1-mutate-allowed': [("  if (values.mutate) throw held(", "  if (false) throw held(")],
    'HA2-chunk-dir-allowed': [("  if (values.chunkDir !== undefined) throw held(", "  if (false) throw held(")],
    'HA3-frames-not-checked': [("if (Number(values.frames) !== registered.frames ||", "if (false ||")],
    'HA4-seed-not-compared': [("if (registered.seed != null && parseSeed(values.seed) !== registered.seed) throw", "if (false) throw")],
    'HA5-engine-dir-not-compared': [("if (!wantEngine || !gotEngine || wantEngine !== gotEngine) throw", "if (false) throw")],
    'HA6-expected-chunk-bytes-not-compared': [("    if (live !== m[2]) throw", "    if (false) throw")],
    'HA7-expected-chunk-name-unchecked': [("if (!m || !CHUNK_FILES.some((f) => path.basename(f) === m[1])) throw", "if (!m) throw")],
    'HA8-registered-frames-missing-allowed': [("if (!Number.isInteger(registered?.frames)) throw held(", "if (false) throw held(")],
    # the pins around the device
    'HP1-incomplete-record-allowed': [("throw held('pins', 'manifest.pinned is incomplete')", "void 0")],
    'HP2-held-pins-vs-pinned-not-compared': [("if (JSON.stringify(sortKeys(flat)) !== JSON.stringify(sortKeys(manifest.held?.pins ?? null))) throw held('pins',", "if (false) throw held('pins',")],
    'HE1-environment-compares-pinned-keys-only': [("for (const key of new Set([...Object.keys(pinned.environment), ...Object.keys(live.environment)])) {", "for (const key of Object.keys(pinned.environment)) {")],
    'HE2-environment-compares-the-six-version-keys-only': [("for (const key of new Set([...Object.keys(pinned.environment), ...Object.keys(live.environment)])) {", "for (const key of ['node', 'python', 'numpy', 'pillow', 'playwright', 'browser']) {")],
    'HB1-binary-sha-not-computed': [("node: process.version, node_sha256: fileSha256Sync(realpathSync(process.execPath)),", "node: process.version, node_sha256: 'x',")],
    'HB2-node-executable-not-presented': [("  if (pinned.environment?.node_sha256) files.push([process.execPath, pinned.environment.node_sha256]);", "")],
    'HB3-device-not-presented': [("  for (const [name, sha] of Object.entries(pinned.held_device ?? {})) files.push([name === 'sr_held.mjs' ? SR_HELD_FILE : path.join(path.dirname(SR_HELD_FILE), name), sha]);", "")],
    # the marker and the claim
    'HF1-marker-not-prechecked': [("  if (existsSync(path.join(root, markerRel))) throw held(", "  if (false) throw held(")],
    'HF2-claim-skips-the-device': [("  const outcome = claim(hold.rq);", "  const outcome = { marker: path.join(root, 'none'), seed: '', log: [] };")],
    'HF3-null-runs-claim-too': [("  if (!hold.gateData) return null;\n  const outcome", "  const outcome")],
    'HF4-one-marker-for-both-kinds': [("export const gateDataKind = (values) => (values.mode === 'roi' ? 'roi' : 'it');", "export const gateDataKind = (values) => 'roi';")],
    'HF5-seed-returned-for-every-registration': [("seed: hold.seedHeld ? outcome.seed : null", "seed: outcome.seed")],
    # the seed policy
    'HS1-held-and-registered-seed-allowed': [("  if (wantSeed && registered.seed != null) throw held('seed',", "  if (false) throw held('seed',")],
    'HS2-seed-never-held': [("  const wantSeed = manifest.held?.seed === true;", "  const wantSeed = false;")],
    'HS3-no-seed-at-all-allowed': [("  if (!wantSeed && values.mode === 'it' && registered.seed == null) throw held('seed',", "  if (false) throw held('seed',")],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--only')
    a = ap.parse_args()
    out = pathlib.Path(a.out).resolve()
    if not str(out).startswith(str(ROOT / 'build') + os.sep):
        sys.exit('--out must be under build/')
    out.mkdir(parents=True, exist_ok=True)
    only = set(a.only.split(',')) if a.only else None
    results = {}
    for name, edits in M.items():
        if only and name.split('-')[0] not in only and name not in only and not name.startswith('H0'):
            continue
        d = out / name
        if d.exists():
            shutil.rmtree(d)
        t = d / 'slam_refact/web-chunk/tests'
        t.mkdir(parents=True)
        for f in (H, 'chunk-hold.test.mjs'):
            shutil.copy(TESTS / f, t / f)
        # everything else the tests read from the repository (live pins, the environment) is linked, so the unmutated copy passes all tests
        for src in sorted((ROOT / 'slam_refact/web-chunk').iterdir()):
            if src.name not in ('tests', '__pycache__'):
                (d / 'slam_refact/web-chunk' / src.name).symlink_to(src)
        for src in sorted(TESTS.iterdir()):
            if src.name not in (H, 'chunk-hold.test.mjs', '__pycache__'):
                (t / src.name).symlink_to(src)
        (d / 'slam_refact/tools').symlink_to(ROOT / 'slam_refact/tools')
        (d / 'scripts').symlink_to(ROOT / 'scripts')
        engine = d / 'build/slam-refact-full-20261019/chunk/rm4d-20261020'
        engine.mkdir(parents=True)
        (engine / 'engine-pinned').symlink_to(ROOT / 'build/slam-refact-full-20261019/chunk/rm4d-20261020/engine-pinned')
        text = (t / H).read_text()
        for old, new in edits:  # the H0 baseline has no edit: it must pass (the copy is a faithful tree)
            if text.count(old) != 1:
                sys.exit(f'{name}: pattern found {text.count(old)} times')
            text = text.replace(old, new)
        (t / H).write_text(text)
        r = subprocess.run(['node', '--test', str(t / 'chunk-hold.test.mjs')], cwd=d, capture_output=True, text=True)
        (d / 'test.log').write_text(r.stdout + r.stderr)
        failed = re.findall(r'^not ok \d+ - (.*)$', r.stdout, re.M)
        if name.startswith('H0'):
            if r.returncode != 0:
                sys.exit(f'the unmutated copy fails its own tests: {failed}')
            results[name] = {'detected': None, 'failed': []}
            print('BASELINE   ', name, '| passes all tests in the copy tree', flush=True)
            continue
        results[name] = {'detected': r.returncode != 0, 'failed': [x[:70] for x in failed]}
        print(('DETECTED  ' if r.returncode else 'UNDETECTED'), name, '|', len(failed), [x[:12] for x in failed], flush=True)
    (out / 'results.json').write_text(json.dumps(results, indent=1) + '\n')
    undetected = [k for k, v in results.items() if v['detected'] is False]
    print(json.dumps({'mutants': len(results), 'undetected': undetected}))
    sys.exit(1 if undetected else 0)


if __name__ == '__main__':
    main()
