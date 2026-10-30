#!/usr/bin/env python3
"""rm3a-lighting-gate.v3 manifest writer.

  python3 -I make_manifest.py draft  --prereg DIR --out MANIFEST [--created-at ISO]
  python3 -I make_manifest.py review --manifest M --review-file R --designation D --reviewer TEXT --at ISO --out M2

draft : reads DIR/rm3a-lighting-gate.v3.md, the v1/v2 files in DIR, the held pin set (run.mjs --print-pins), the staged input tree (sr_held tree sha256),
        the null-run/mutation evidence of the dry run and writes the manifest with review = {} and first_real_run_at = null.  Refuses to overwrite.
review: adds the review object (reviewer, at, file, sha256, designation_file, designation_file_sha256); everything else unchanged.  Refuses to overwrite.
Never reads a coordinator designation or review text other than for its sha256.
"""
import argparse, datetime, hashlib, json, os, pathlib, subprocess, sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SNAP = ROOT / 'build/slam-refact-rm3a-v3-20261023'
sha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
rel = lambda p: str(pathlib.Path(p).resolve().relative_to(ROOT))


def draft(a):
    prereg = pathlib.Path(a.prereg).resolve()
    md = prereg / 'rm3a-lighting-gate.v3.md'
    v1, v2 = prereg / 'rm3a-lighting-gate.v1.md', prereg / 'rm3a-lighting-gate.v2.md'
    v1m, v2m, v2a = prereg / 'rm3a-lighting-gate.v1.manifest.json', prereg / 'rm3a-lighting-gate.v2.manifest.json', prereg / 'rm3a-lighting-gate.v2.addendum.json'
    binary = SNAP / 'held/sr_tests_xr_api_evidence'
    pins = json.loads(subprocess.run(['node', str(HERE / 'run.mjs'), '--bin', str(binary), '--print-pins'], capture_output=True, text=True, check=True).stdout)
    tree = subprocess.run(['node', '-e', f"import('{ROOT}/slam_refact/tools/prereg/sr_held.mjs').then(m=>{{const t=m.treeSha('{SNAP}/src/build');if(!t.sha)process.exit(1);console.log(t.sha)}})"],
                          capture_output=True, text=True, check=True).stdout.strip()
    head = (SNAP / 'head-sha.txt').read_text().strip()
    nm = SNAP / 'runs/nullmut-dry1'
    nmv = json.load(open(nm / 'nullmut-verdict.json'))
    assert nmv['verdict'] == 'PASS' and nmv['binary_sha256'] == sha(binary), 'dry-run evidence is not a PASS of the pinned binary'
    if sha(binary) != pins['sr_tests_xr_api_evidence']:
        sys.exit('binary sha mismatch')
    m = {
        'gate_id': 'rm3a-lighting-gate', 'version': 3,
        'supersedes': {'file': 'rm3a-lighting-gate.v2.md', 'sha256': sha(v2), 'manifest': 'rm3a-lighting-gate.v2.manifest.json', 'manifest_sha256': sha(v2m),
                       'addendum': 'rm3a-lighting-gate.v2.addendum.json', 'addendum_sha256': sha(v2a),
                       'v1_file': 'rm3a-lighting-gate.v1.md', 'v1_sha256': sha(v1), 'v1_manifest_sha256': sha(v1m)},
        'supersedes_note': 'v3 re-registers the v2 gate (definition, numbers, tolerance unchanged) so that it can reach GATE class: coordinator-designated independent pre-review, full pin set, shared sr_held one-run marker. '
                           'v2 stays on record as REPORT (non-independent pre-reviewer, --no-pin only, manifest edited after the run, no measurer/binary pin). The results of v2 are known (gate file section 0).',
        'file': 'rm3a-lighting-gate.v3.md', 'sha256': sha(md),
        'created_at': a.created_at or datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
        'registered_by': 'w3-rm3a-v3 worker (Sonnet 5.5)',
        'git_head': head, 'tree_sha': None,
        'snapshot': 'build/slam-refact-rm3a-v3-20261023/src = git archive HEAD (head-sha.txt); the 3262 tracked files equal the blob ids of git ls-tree -r HEAD; src/build is the staged input tree (real file copies); no write-tree (A3 not granted)',
        'classification': ['pre-registration defect (procedure): v2 evidence class REPORT, see gate file header', 'registered after the v2 results were known (gate file section 0)', 'not a requirement change'],
        'known_before_registration': 'v2 first run 2026-10-19T21:50:52+09:00 (hand-port estimator): G-L1a 136/137 {6}, G-L1b 119/120 {6}, G-L2 1009/1009, G-L3 600/600; vendored estimator re-run 2026-10-22 PASS (rm3a-lighting-vendoring-results.md); '
                                     'F2 snapshot d7629f6 evidence ctest 52/52; the v3 binary has run only the 7 null-run/mutation tests, never the 4 gate tests',
        'resolution': {'metric': 'float32 representation resolution (1 ulp) of the compared exposure = spacing of the float32 value, maximum over the compared data', 'value': 2.9802322e-08,
                       'unit': 'exposure units: api3 max |exposure| 0.4978 (spacing 2^-25 = 2.98e-8), api1 max |exposure| 0.0886 (spacing 7.45e-9)',
                       'method': 'numpy float32 spacing of the largest |exposure| in oracle/tsv-api1/lighting.tsv and oracle/tsv-api3/lighting.tsv; protocol section 3: max(null-run value, representation resolution); null-runs N1, N2, N2b measured 0'},
        'tolerance': {'value': 0, 'unit': 'exact (0 ulp)', 'exact_class_gate': True,
                      'exact_class_justification': 'The compared quantity is float32(int score) / float32(cols*rows) on both sides (lighting.cc and the original, DCMP 64809-64810): the same IEEE operation on the same integers gives the same bits, and two different integer scores differ by at least '
                                                   '1/34240 = 2.92e-5, i.e. about 980 ulp at the largest exposure (api3) and about 3900 ulp (api1). A bit-exact comparison therefore has no noise to tolerate. The exception works only toward a false FAIL (1 ulp noise would FAIL; noise never makes a PASS). Protocol section 11 conditions 1-6, gate file section 2.'},
        'null_run': {'evidence': rel(nm / 'gtest-nullmut.log'), 'result': 'PASS', 'run_at': nmv.get('run_at', None) or datetime.datetime.fromtimestamp(os.path.getmtime(nm / 'gtest-nullmut.log')).astimezone().isoformat(timespec='seconds'),
                     'detail': 'N1 api1 1009 / api3 600 rows 0 mismatches; N2 api3 and N2b api1 first 60 callbacks 0 differing between two pipeline runs; verdict file ' + rel(nm / 'nullmut-verdict.json') + ' (judge.mjs judgeNullMutation, binary sha256 ' + sha(binary) + ')'},
        'mutation': {'description': 'M1 frame+1 input (api3), M2 one-ulp expected value (api1), M3/M3b wrong association rule (gate file section 5)', 'evidence': rel(nm / 'gtest-nullmut.log'), 'detected': True,
                     'sub_resolution_detected': True, 'sub_resolution_mutation': 'M2: one expected value moved by one ulp (nextafterf) is pinpointed at exactly one callback (501)',
                     'larger_mutations_detected': 'M1 input frame+1: 58 of 60 callbacks mismatch; M3/M3b wrong association rule: 132 of 137 (api1) and 118 of 120 (api3) mismatch; the same numbers as the v2 log',
                     'detail': 'M1 58 of 60; M2 callback 501 only; M3 132 of 137; M3b 118 of 120 (both != {6})'},
        'independent_samples': {'n': 2, 'derivation': 'The independent unit is the input RECORD: Flip6 L-S2 loop replay (api1, 480x640) and TUM VI room1 full (api3, 240x320). The 1009 and 600 callbacks inside one record, the two association rules (G-L1 vs G-L2/G-L3) on one record, '
                                                      'and any rerun of one record are not independent samples. These are the same two records as v1/v2/the vendoring check/the F2 re-run, so n = 2 counts records, not new independent inputs.'},
        'inputs': {'build/slam-refact-full-20261019/oracle/tsv-api1/lighting.tsv': 'd052e350265c00a81a31a0c0c1e4aec81213e8703f44a8ea11ab5de7e8e0d738',
                   'build/slam-refact-full-20261019/oracle/tsv-api3/lighting.tsv': 'a1e4c7e1a62fe8a8af229eb7c6cc9a58895bec7ed95e4d922e72d46f770ec9b8',
                   'build/slam-refact-full-20261019/oracle/flip6-s2/api1/api-oracle.json.gz': 'ec0772accd7ab5dc3f64378baf8d212a42351326fbb4ce599c94009a6a6d6838',
                   'build/slam-refact-full-20261019/oracle/tum-room1-full/api3/api-oracle.json.gz': 'b1d80b4d00f043fecc4d3b3a24142b7af454b9092043809702cb0d4c2804c01e',
                   'slam_refact/tools/harness/oracle/extract_lighting_oracle.py': 'd0a4fb29bf9d0fd68e80b9d042f8e016dca02826315bd9d9b061fbe2f29e1d97',
                   'staged_tree': rel(SNAP / 'src/build') + ' (sr_held tree sha256 ' + tree + '; 1612 real files: 2 lighting.tsv, hits.tsv, 1609 pgm)'},
        'build': {'script': rel(SNAP / 'build-native.sh'), 'script_sha256': sha(SNAP / 'build-native.sh'), 'log': rel(SNAP / 'logs/build-native.log'), 'head': head,
                  'engine_tree_git': subprocess.run(['git', '-C', str(ROOT), 'rev-parse', f'{head}:slam_refact/engine'], capture_output=True, text=True, check=True).stdout.strip(),
                  'engine_tree_git_at_f2_d7629f6': subprocess.run(['git', '-C', str(ROOT), 'rev-parse', 'd7629f6:slam_refact/engine'], capture_output=True, text=True, check=True).stdout.strip(),
                  'binary': 'build/slam-refact-rm3a-v3-20261023/held/sr_tests_xr_api_evidence', 'binary_sha256': sha(binary), 'data_dir': str(SNAP / 'src/build'),
                  'f2_engine_pin_reused': False, 'f2_engine_pin_note': 'engine-pin has no gtest binary; the F2 gtest binary (sha256 070c4380bdda2c7ce8687f21eae198838803b19edddc1c446c4bc07d683eeb27) reads its inputs through a symlink to the live build dir'},
        'runner': {'driver': rel(HERE / 'run.mjs'), 'judge': rel(HERE / 'judge.mjs'), 'wrapper': rel(HERE / 'run.sh'), 'tests': rel(HERE / 'judge.test.mjs'), 'held': 'slam_refact/tools/prereg/sr_held.mjs (shared, protocol 12.5)'},
        'tool_sha256': {rel(HERE / n): sha(HERE / n) for n in ('run.mjs', 'run.sh', 'judge.mjs', 'judge.test.mjs', 'make_manifest.py')},
        'held': {'kinds': ['main'], 'pins': pins, 'inputs': {'main': tree}, 'seed': False},
        'first_run_marker': 'rm3a-lighting-gate.v3.first-run.main.marker',
        'review_request': 'rm3a-lighting-gate.v3.review-request.md',
        'review': {}, 'first_real_run_at': None,
    }
    with open(a.out, 'x') as f:
        json.dump(m, f, indent=1); f.write('\n')
    print(f'wrote {a.out} sha256={sha(a.out)}')


def review(a):
    m = json.load(open(a.manifest))
    if m.get('review'):
        sys.exit('manifest already has a review object')
    m['review'] = {'reviewer': a.reviewer, 'at': a.at, 'file': os.path.basename(a.review_file), 'sha256': sha(a.review_file),
                   'designation_file': os.path.basename(a.designation), 'designation_file_sha256': sha(a.designation)}
    with open(a.out, 'x') as f:
        json.dump(m, f, indent=1); f.write('\n')
    print(f'wrote {a.out} sha256={sha(a.out)}')


ap = argparse.ArgumentParser()
sub = ap.add_subparsers(dest='cmd', required=True)
d = sub.add_parser('draft'); d.add_argument('--prereg', required=True); d.add_argument('--out', required=True); d.add_argument('--created-at')
r = sub.add_parser('review')
for k in ('manifest', 'review-file', 'designation', 'reviewer', 'at', 'out'):
    r.add_argument('--' + k, required=True)
args = ap.parse_args()
{'draft': draft, 'review': review}[args.cmd](args)
