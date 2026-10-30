#!/usr/bin/env python3
"""Writes / pins the manifests of orient-us024-gate.v2 and orient-us023-gate.v2 (protocol §6).
  make_manifest_v2.py create   at registration: gate file sha256, created_at, tiers, null-run / mutation evidence, supersedes (v1)
  make_manifest_v2.py pin      after the recordings and the final build, before the review: held binary copy (read-only, in the
                               preregistration directory), link input list, recording SHA256SUMS. Refuses once review.at or
                               first_real_run_at is set. Existing manifests are never recreated."""
import datetime, hashlib, json, os, pathlib, shutil, stat, subprocess, sys

P = pathlib.Path('build/slam-refact-full-20261019/preregistration')
EV = 'build/slam-refact-full-20261019/orient/evidence/'
REC = pathlib.Path('build/slam-refact-full-20261019/orient/original/flip6-s2')
HELD = P / 'orient-held-v2'
sha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
now = lambda: datetime.datetime.now().astimezone().replace(microsecond=0).isoformat()
DIRS = {
    'orient-us024-gate': ['v2-a-ios-12promax', 'v2-a-ios-14pro-nz', 'v2-a-android-pixel7', 'v2-a-android-reduced-nz', 'v2-k-ios-se1', 'v2-k-ios-8',
                          'v2-k-ios-xr', 'v2-k-ios-12', 'v2-k-ios-12promax', 'v2-k-ios-14pro', 'v2-k-ios-15plus'],
    'orient-us023-gate': ['v2-b-o0', 'v2-b-o90', 'v2-b-o180', 'v2-b-o270', 'v2-b-px90', 'v2-b-px270', 'v2-b-s90', 'v2-b-s180', 'v2-b-s270'],
}
ulp32 = {'metric': 'float32 1 ulp', 'value': 1.1920929e-07, 'unit': 'relative', 'method': 'representation resolution; the channel carries bit patterns (channel null-run)'}
ex = lambda j: {'value': 0, 'unit': 'bits', 'exact_class_gate': True, 'exact_class_justification': j}
TIERS24 = [
    {'id': 'S-int', 'resolution': {'metric': 'integer field', 'value': 1, 'unit': 'ns / kind', 'method': 'int64 / int32'},
     'tolerance': ex('integer fields: identical integer operations (x1000 wrap, 16000000 -> 16666667 mapping, copy); any different value differs by at least 1')},
    {'id': 'S-float', 'resolution': ulp32,
     'tolerance': ex('float32 x/y/z compared as bit patterns including the sign of a zero: f32.neg and copy after an identity multiply-add; a different value differs by at least 1 ulp, 0x00000000 vs 0x80000000 differ in one bit; the channel loses no bit (channel null-run) so there is no false-PASS path')},
    {'id': 'K', 'resolution': ulp32,
     'tolerance': ex('reality.intrinsics float32 bit patterns including the sign of a zero: same float operations and order as the public source; any different value differs by at least 1 ulp')},
]
TIERS23 = TIERS24 + [
    {'id': 'P', 'resolution': {'metric': '8-bit level', 'value': 1, 'unit': 'level', 'method': 'RGBA8 atlas'},
     'tolerance': ex('8-bit output of the same shader operations (CPU port in the swiftshader-approx model vs SwiftShader GPU, F-H5 bit-exact for portrait); any different value differs by at least 1 level')},
    {'id': 'R-position', 'resolution': {'metric': 'float32 1 ulp at |p| <= 4', 'value': 4.76837e-07, 'unit': 'responsive units', 'method': 'representation resolution'},
     'tolerance': {'value': 1e-5, 'unit': 'responsive units'}},
    {'id': 'R-rotation', 'resolution': {'metric': 'float32 1 ulp at |q| <= 1', 'value': 1.1920929e-07, 'unit': 'quaternion component', 'method': 'representation resolution'},
     'tolerance': {'value': 1e-5, 'unit': 'quaternion component'}},
]


def create():
    head = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
    v1 = {g: P / f'{g}.v1.md' for g in DIRS}
    v1m = {g: P / f'{g}.v1.manifest.json' for g in DIRS}
    review1 = P / 'orient-gates.v1.review.md'
    for g, tiers in (('orient-us024-gate', TIERS24), ('orient-us023-gate', TIERS23)):
        f = P / f'{g}.v2.md'
        out = P / f'{g}.v2.manifest.json'
        assert not out.exists(), out
        m = {
            'gate_id': g, 'version': 2,
            'supersedes': {'file': str(v1[g]), 'sha256': sha(v1[g]), 'manifest': str(v1m[g]), 'manifest_sha256': sha(v1m[g]),
                           'v1_review_file': str(review1), 'v1_review_sha256': sha(review1),
                           'reason': 'v1 REJECTed by the independent pre-run review (no first real run, no designation file): B-1 the original-side channel lost the sign of a zero, M-1 engine zero rules fitted to that loss; v1 has no engine-vs-original result'},
            'classification': ['pre-registration defect (measurement instrument): bit channel + channel null-run, engine zero rules withdrawn, zero-sign prediction registered from the wat (B-1, M-1)',
                               'procedure: O_EXCL first-run marker, ordering and re-run rules, read-only held binary copy with pin history, link input list (m-1..m-4)',
                               'documentation / low: signed-overflow UB removed, probe wording, FindValue scoping, unrecorded geometry combinations, K 180 degree statement'],
            'file': str(f), 'sha256': sha(f), 'created_at': now(), 'registered_by': 'US-023/024 orientation executor',
            'git_head': head, 'tree_sha': None,
            'snapshot': 'build/slam-refact-full-20261019/preregistration/orient-snapshot-pre-v2/ (sources, harness scripts, probe wasm, checked-in channel null-run data + SHA256SUMS); no write-tree (A3 not granted)',
            'resolution': tiers[0]['resolution'], 'tolerance': tiers[0]['tolerance'], 'tiers': tiers,
            'null_run': {'evidence': EV + 'instrument-null-runs-and-mutations-v2.log', 'channel_null_run_evidence': EV + 'channel-null-run.log', 'result': 'PASS'},
            'mutation': {'description': 'comparator mutations on the synthetic recording (sensors x12 incl. one zero-sign bit, intrinsics x4 incl. zero-sign bit, pyramid x2); R input mutations on exploration recordings (REPORT)',
                         'evidence': EV + 'instrument-null-runs-and-mutations-v2.log (and ' + EV + 'explore-report-pose-and-mutations.log for R)',
                         'detected': True, 'sub_resolution_detected': True, 'larger_mutations_detected': True,
                         'sub_resolution_mutation': 'one ulp on x of one event (S, S\'); one ulp on m[0] (K); one level on one pixel (P); the sign bit of one zero (S, S\', K)',
                         'not_detected_by_design': ['IMU receive time +1 ms and one dropped DeviceMotion packet by R (the tracker does not use the IMU stream at responsive scale; S covers the sensor path)']},
            'review': {'reviewer': None, 'at': None, 'review_file': None, 'review_file_sha256': None, 'designation_file': None, 'designation_file_sha256': None},
            'first_real_run_at': None,
            'held_run': {'gate_file': f.name, 'gate_file_sha256': sha(f), 'held_binary_sha256': None, 'input_sums': {}, 'pinned_at': None,
                         'note': 'held_binary_sha256, held_binary, link_inputs_sha256 and input_sums are pinned after the recordings and the final build and before the review; no comparison of the engine with a registered recording exists'},
        }
        out.write_text(json.dumps(m, indent=1) + '\n')
        print('created', out, m['created_at'])


def pin():
    held_src = pathlib.Path('build/slam-refact-full-orient/tests/engine/sr_tests_orient_gate_held')
    HELD.mkdir(exist_ok=True)
    dst = HELD / 'sr_tests_orient_gate_held'
    if dst.exists():
        os.chmod(dst, 0o755)
        dst.unlink()
    shutil.copy2(held_src, dst)
    os.chmod(dst, 0o555)
    inputs = subprocess.run([sys.executable, 'slam_refact/tools/re/runtime/orient/link_inputs.py', 'build/slam-refact-full-orient', 'tests/engine/sr_tests_orient_gate_held'],
                            capture_output=True, text=True, check=True).stdout
    inputs_file = HELD / 'inputs.sha256'
    if inputs_file.exists():
        os.chmod(inputs_file, 0o644)
    inputs_file.write_text(inputs)
    os.chmod(inputs_file, 0o444)
    for g, dirs in DIRS.items():
        f = P / f'{g}.v2.manifest.json'
        m = json.load(open(f))
        assert m['first_real_run_at'] is None and m['review']['at'] is None, 'already reviewed or run: pin refused'
        h = m['held_run']
        old = h.get('held_binary_sha256')
        hist = h.setdefault('pin_history', [])
        if old:
            hist.append({'held_binary_sha256': old, 'pinned_at': h['pinned_at'], 'superseded_at': now()})
        h['held_binary_sha256'] = sha(dst)
        h['held_binary'] = str(dst)
        h['link_inputs_file'] = str(inputs_file)
        h['link_inputs_sha256'] = sha(inputs_file)
        h['recording_root'] = str(REC)
        h['input_sums'] = {d: sha(REC / d / 'SHA256SUMS') for d in dirs}
        h['probe_wasm_sha256'] = '6518245142fba3ee43c60e8a7b6ec935e10fcdab0d2440a720fcf2918097d7f2'
        h['original_wasm_sha256'] = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422'
        h['pinned_at'] = now()
        h['v1_pin_history'] = [{'file': 'orient-us02x-gate.v1.manifest.json', 'held_binary_sha256': 'f0ff025ebaab76d4e02ed98470688a503182738787ab5b9a5baa2b847cff1e68',
                                'note': 'first v1 pin; invalidated by a rebuild before the review (sources unchanged apart from the release-check test)'},
                               {'held_binary_sha256': json.load(open(P / f'{g}.v1.manifest.json'))['held_run']['held_binary_sha256'], 'note': 'v1 final pin (build directory binary); v1 REJECTed, never ran'}]
        f.write_text(json.dumps(m, indent=1) + '\n')
        print('pinned', g, h['held_binary_sha256'][:16], len(h['input_sums']))


if __name__ == '__main__':
    {'create': create, 'pin': pin}[sys.argv[1]]()
