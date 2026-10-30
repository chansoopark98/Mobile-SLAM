#!/usr/bin/env python3
"""Registration helper of the image-target scale gate `it-scale-gate` v1 (US-015; preregistration/it-scale-gate.v1.md).

Not part of the measurement and not pinned. It writes the manifest from the evidence files (so the numbers are the log's, not
typed), pins the test binary, and, once the coordinator has written the seed file, derives the registered table (rendering
only: no engine runs) and records its tree sha256. It never runs an engine, never writes a review field and never writes
first_real_run_at.

  its_finalize.py pin-binary --binary <sr_tests_image_target_scale> --held-dir <dir>
        copy the binary read-only (mode 0555) into <dir>; print the sha256 (held_pins)
  its_finalize.py init --gate-md <gate.md> --manifest <out.json> --evidence <it4 dir> --held-dir <dir>
        phase A: the manifest without the seed-dependent fields (inputs_tree_sha256.registered, coordinator_seed.value)
  its_finalize.py seed --seed-file <seed.txt> --registered-dir <dir> --manifest <json>
        phase B (needs the coordinator seed): derive the registered table into <dir>, record its tree sha256, refresh created_at
  its_finalize.py sources --manifest <json>
        verify that the pinned sources and the binary are unchanged since phase A
"""
import argparse
import hashlib
import json
import os
import pathlib
import re
import shutil
import stat
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
import its_gate_data as data  # noqa: E402
import curvy_held_tool as ht  # noqa: E402

PREREG = ROOT / 'build/slam-refact-full-20261019/preregistration'
GATE_ID, VERSION = 'it-scale-gate', 1
SOURCES = [
    'slam_refact/tools/image_target/its_gate_data.py',
    'slam_refact/tools/image_target/its_finalize.py',
    'slam_refact/tools/image_target/curvy_held_tool.py',
    'slam_refact/tools/image_target/tests/test_its_gate_data.py',
    'slam_refact/tools/image_target/tests/test_its_finalize.py',
    'slam_refact/engine/tests/image_target/image_target_scale_test.cc',
    'slam_refact/engine/tests/image_target/its_held.h',
    'slam_refact/engine/tests/image_target/its_held_test.cc',
    'slam_refact/engine/tests/image_target/curvy_held.h',
    'slam_refact/engine/tests/image_target/test_util.h',
    'slam_refact/engine/tests/image_target/CMakeLists.txt',
    'slam_refact/engine/src/vio_tracker/vio_tracker.cc',
    'slam_refact/engine/third_party/xr_source/reality/engine/imagedetection/image-tracking-state.cc',
    'slam_refact/engine/third_party/xr_source/reality/engine/imagedetection/image-tracking-state.h',
    'slam_refact/engine/third_party/xr_source/reality/engine/imagedetection/detection-image-tracker.cc',
    'slam_refact/engine/third_party/xr_source/reality/engine/imagedetection/tracked-image.h',
]
BINARY = 'sr_tests_image_target_scale'


def sha_file(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def now():
    return subprocess.run(['date', '-Is'], capture_output=True, text=True, check=True).stdout.strip()


def rel(p):
    try:
        return str(pathlib.Path(p).resolve().relative_to(ROOT))
    except ValueError:
        return str(p)


def write_manifest(path, m):
    pathlib.Path(path).write_text(json.dumps(m, indent=1, ensure_ascii=False) + '\n')


def pin_binary(binary, held_dir):
    held = pathlib.Path(held_dir)
    held.mkdir(parents=True, exist_ok=True)
    dst = held / pathlib.Path(binary).name
    if dst.exists():
        dst.chmod(0o755)
    shutil.copyfile(binary, dst)
    dst.chmod(0o555)
    held.chmod(0o555)
    return sha_file(dst)


def parse_null(log):
    tiers = []
    for line in pathlib.Path(log).read_text().splitlines():
        m = re.match(r'\[ITS-NULL\] tier "(.+)": resolution (\S+) tolerance (\S+) ratio (\S+) (OK|VIOLATES.*)', line)
        if m:
            tiers.append({'name': m.group(1), 'resolution': float(m.group(2)), 'tolerance': float(m.group(3)),
                          'ratio': float(m.group(4)), 'ok': m.group(5) == 'OK'})
    return tiers


def passed(log, expect_tests):
    text = pathlib.Path(log).read_text()
    return ('[  PASSED  ] %d test' % expect_tests) in text and '[  FAILED  ]' not in text


def init(a):
    ev = pathlib.Path(a.evidence)
    gate = pathlib.Path(a.gate_md)
    tiers = parse_null(ev / 'null-run.log')
    if len(tiers) != 8 or not all(t['ok'] for t in tiers):
        raise SystemExit(f'null-run log: expected 8 tiers all OK, got {len(tiers)}')
    checks = {'null-run.log': (2, 'RunsAreBitIdentical + NullRunResolutionRule'), 'mutation-input.log': (1, 'InputMutationsAreDetected'),
              'dev-control.log': (1, 'ControlPassesOnDevData'), 'mutation-code-IS1.log': (1, 'IS1'), 'mutation-code-IS2.log': (1, 'IS2'),
              'mutation-code-IV1.log': (1, 'IV1')}
    for f, (n, what) in checks.items():
        if not passed(ev / f, n):
            raise SystemExit(f'{f} ({what}) did not pass')
    held_bin = pathlib.Path(a.held_dir) / BINARY
    m = {
        'gate_id': GATE_ID, 'version': VERSION, 'sha256': sha_file(gate), 'supersedes': None,
        'replaces': {'gate': 'IT-P2 (scale exactly 0.5; image-target.md section 7.2, public-source judgement)',
                     'rule': 'IT-P2 counts as replaced when S0 passes in the first real run of this gate; until then its recorded status stands'},
        'file': rel(gate), 'created_at': now(), 'registered_by': 'US-015 executor',
        'git_head': subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True, cwd=ROOT).stdout.strip(),
        'tree_sha': None,
        'snapshot': 'none (grade middle needs a tar snapshot; this gate adds only test code and tools, no engine source was changed): gate file sha256 + date -Is + tool_sha256 of every source below',
        'coordinator_seed': {'value': None, 'file': rel(PREREG / 'it-scale-gate.v1.coordinator-seed.txt'), 'generated_at': None,
                             'derivation': 'its_gate_data.derive_registered (gate file section 1)', 'pending': 'the coordinator has not written the seed file yet'},
        'data_variants': {
            'dev': 'its_gate_data.py --variant dev (fixed numbers) and --variant dev-seed with the executor seeds 9100..9199; the only variants an engine ran on',
            'registered': 'derived from the coordinator seed; rendered (a table of numbers, no engine) and never an engine input before the first real run'},
        'seen_before_registration': 'all dev data (dev rows, executor seeds 9101..9112); see gate file section 0 for the order of events',
        'resolution': {'metric': 'S0 scale relative to 0.5; S1 scale relative; S2 position / k, rotation deg, size relative; S3 stationarity / k; S4 S5 relative and degrees',
                       'value': {t['name']: t['resolution'] for t in tiers}, 'unit': 'per tier, see tiers',
                       'method': 'gate file section 4; ' + rel(ev / 'null-run.log') + '; resolution = max(null-run value, float32 representation floor)'},
        'tolerance': {'value': None, 'unit': 'per tier, see tiers'},
        'tiers': [{'id': t['name'], 'resolution': {'value': t['resolution'], 'unit': 'null-run value or float32 floor'},
                   'tolerance': {'value': t['tolerance'], 'unit': 'as in the tier name'}} for t in tiers],
        'classification': {
            'GATE': ['S0 (zero and sub-threshold baseline keep the public prior 0.5)', 'S1 scale', 'S2 world pose and size', 'S3 world stationarity',
                     'S4 world scaled by c', 'S5 world rotated about the vertical axis'],
            'PREMISE': ['P1 camera steps', 'P1s slow steps', 'P2 k away from the prior', 'P3 sample count', 'P4 photo size', 'P5 table kinds', 'P6 pairs'],
            'REPORT': ['S6 loss of the target', 'S8 feature noise 0.5 and 1.0 px', 'cost of the prior', 'convergence profile', 'dev seed sweep']},
        'null_run': {'evidence': rel(ev / 'null-run.log'), 'result': 'PASS',
                     'detail': 'N1 truth = prior (k 0.5); N2 pairs at c 1.6 / psi 75 and the identity pairs c 1 / psi 0; N3 two identical runs are bit-identical; N4 zero and slow baseline; every tier tolerance >= 3 x resolution (ratios in the log)'},
        'mutation': {'description': 'input mutations of the scripted tam (x1.10, x0.90, x1.05, rotation only, one-frame lag, x translation negated; negative control x1.005 must pass) and three code mutants (computeScale inverted, baseline threshold 3e-4, yaw offset not applied); dev data only; sensitivity report x1.005 .. x1.05',
                     'evidence': rel(ev / 'mutation-input.log'), 'detected': True,
                     'code_mutant_evidence': [rel(ev / f'mutation-code-{x}.log') for x in ('IS1', 'IS2', 'IV1')],
                     'sensitivity_evidence': rel(ev / 'sensitivity.log'),
                     'note': 'not an exact-class gate: no sub-resolution mutation is claimed; the smallest tam mis-scaling the gate detects is between 1 and 2 percent (sensitivity.log)'},
        'held_run': {'gate_file': gate.name, 'gate_file_sha256': sha_file(gate),
                     'device': 'its_held.h (CheckRelease, ClaimFirstRun, HeldGate); unit test its_held_test.cc'},
        'held_pins': {BINARY: sha_file(held_bin)} if held_bin.exists() else {},
        'inputs_tree_sha256': {'registered': None},
        'review': {'reviewer': None, 'at': None, 'file': None, 'sha256': None,
                   'designation_file': 'it-scale-gate.v1.review-designation.json', 'designation_file_sha256': None},
        'first_real_run_at': None,
        'tool_sha256': {s: sha_file(ROOT / s) for s in SOURCES},
        'verify_manifest_sha256': sha_file(ev / 'verify_manifest.py') if (ev / 'verify_manifest.py').exists() else None,
        'timeline_honesty': 'the test code, the tools and the dev runs preceded this registration; the tolerances were written into the first draft of the test before any engine run, except S4 / S5 which were tightened (not loosened) after the first dev pair run; see gate file section 0.',
        'revisions': [],
        'pending': ['phase B: the coordinator seed file, then its_finalize.py seed (inputs_tree_sha256.registered, coordinator_seed.value, created_at)',
                    'independent reviewer designation (protocol 12); review.* and the designation file are the coordinator\'s',
                    'first real run: not done'],
    }
    write_manifest(a.manifest, m)
    print('manifest written', a.manifest)


def sources_ok(m):
    bad = [s for s, h in m['tool_sha256'].items() if sha_file(ROOT / s) != h]
    pinned = m.get('held_pins', {}).get(BINARY)
    held = PREREG / 'it-scale-held-v1' / BINARY
    if pinned and (not held.exists() or sha_file(held) != pinned):
        bad.append('held binary ' + rel(held))
    return bad


def seed(a):
    mp = pathlib.Path(a.manifest)
    m = json.loads(mp.read_text())
    if m.get('review', {}).get('at') or m.get('first_real_run_at'):
        raise SystemExit('a review or a first real run is recorded: the manifest is not edited any more')
    bad = sources_ok(m)
    if bad:
        raise SystemExit('sources changed since phase A (a new phase A is needed): ' + ', '.join(bad))
    sf = pathlib.Path(a.seed_file)
    if not sf.exists():
        raise SystemExit(f'{sf}: the coordinator seed file does not exist (BLOCKED: the coordinator writes it)')
    value = data.read_seed(sf)
    if value in data.EXECUTOR_SEEDS:
        raise SystemExit(f'seed {value} is one of the executor seeds {data.EXECUTOR_SEEDS.start}..{data.EXECUTOR_SEEDS.stop - 1}')
    toks = sf.read_text().split()
    out = pathlib.Path(a.registered_dir)
    data.write_dataset(data.derive_registered(value), 'registered', out, seed=value, force=True)
    tree = ht.tree_sha(str(out))
    if not tree:
        raise SystemExit('empty registered directory')
    stamp = now()
    m['coordinator_seed'].update({'value': value, 'generated_at': toks[1] if len(toks) > 1 else None, 'file_sha256': sha_file(sf)})
    m['coordinator_seed'].pop('pending', None)
    m['inputs_tree_sha256']['registered'] = tree
    m['created_at'] = stamp
    m['revisions'].append({'at': stamp, 'what': 'phase B: coordinator seed, registered table rendered (no engine), inputs_tree_sha256.registered, created_at refreshed'})
    m['pending'] = [p for p in m['pending'] if not p.startswith('phase B')]
    write_manifest(mp, m)
    print('registered table', out, 'tree sha256', tree)


def main(argv):
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('pin-binary')
    p.add_argument('--binary', required=True)
    p.add_argument('--held-dir', required=True)
    p = sub.add_parser('init')
    p.add_argument('--gate-md', required=True)
    p.add_argument('--manifest', required=True)
    p.add_argument('--evidence', required=True)
    p.add_argument('--held-dir', required=True)
    p = sub.add_parser('seed')
    p.add_argument('--seed-file', required=True)
    p.add_argument('--registered-dir', required=True)
    p.add_argument('--manifest', required=True)
    p = sub.add_parser('sources')
    p.add_argument('--manifest', required=True)
    a = ap.parse_args(argv)
    if a.cmd == 'pin-binary':
        print(pin_binary(a.binary, a.held_dir))
    elif a.cmd == 'init':
        init(a)
    elif a.cmd == 'seed':
        seed(a)
    else:
        bad = sources_ok(json.loads(pathlib.Path(a.manifest).read_text()))
        print('unchanged' if not bad else 'CHANGED: ' + ', '.join(bad))
        return 1 if bad else 0
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
