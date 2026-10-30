#!/usr/bin/env python3
"""FORK (ba only) for us026-ba-e2e-gate.v2 of slam_refact/tools/harness/us026/make_e2e_static.py.

Static (author) part of the v2 manifest and the null-run / mutation summary files it cites (written as null-run-summary-ba-v2.json and
mutation-summary-ba-v2.json). Reads the v2 dev refresh evidence (dev scenario flip6-n1 only, never registration data) and writes NEW files only.
Differences from the v1 script: version 2, `supersedes`, the v2 command file / snapshot / lane text, the new replay binary and freeze record
(from the pin directory's pin.json), `dev` naming the v1 dev runs AND the v2 refresh, and the independent_samples derivation with the
record-reuse disclosure. Tiers, tolerances, resolutions, arms, mutants, poke table and the other field texts are the v1 texts.

  python3 -I make_e2e_static.py --evidence <v2 evidence dir> --e2e <v2 dev refresh out-root> --mutants <dir>[,<dir>] --pin <pin.json> --out <static.json>
"""
import argparse
import hashlib
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[4]
DEV = 'flip6-n1'
GID = 'us026-ba-e2e-gate'
PRE = 'build/slam-refact-full-20261019/preregistration'
V1_MD = '%s/us026-ba-e2e-gate.v1.md' % PRE
V1_MANIFEST = '%s/us026-ba-e2e-gate.v1.manifest.json' % PRE
REG = ['tum-room1-full', 'flip6-s2', 'flip6-s3-lowtex', 'tum-room2-full', 'tum-room4-full']
ULP_RES = {'value': 1, 'unit': 'float32 ulp of max(|a|,|b|)', 'metric': 'per component |a-b| / ulp32(max(|a|,|b|))'}
INT_RES = {'value': 1, 'unit': 'integer unit (count, flag, enum, id)', 'metric': 'equality of integer-class fields'}


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def rel(p):
    return str(pathlib.Path(p).resolve().relative_to(ROOT))


def write_new(p, obj):
    p = pathlib.Path(p)
    with open(p, 'x') as f:
        f.write(json.dumps(obj, indent=1, ensure_ascii=False) + '\n')
    return rel(p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--evidence', type=pathlib.Path, required=True)
    ap.add_argument('--e2e', type=pathlib.Path, required=True)
    ap.add_argument('--mutants', required=True)
    ap.add_argument('--pin', type=pathlib.Path, required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    E = a.evidence
    dev = a.e2e / GID / DEV
    comp = json.loads((dev / 'compare.json').read_text())
    rep = json.loads((dev / 'report.json').read_text())
    if rep['kind'] != 'dev':
        raise SystemExit('not a dev report')
    pin = json.loads(a.pin.read_text())
    null = comp['compare']['null']
    nr = {'gate': GID + '.v2', 'kind': 'null-run summary (dev scenario %s only; v2 refresh: the v1 dev e2e3 original runs and bundles re-played with the v2 binary)' % DEV,
          'NR-2': {'what': 'dev null-poke original run (e2e3) vs its exact-input replay by the v2 binary, comparator compare_us026.py unchanged',
                   'report': rel(dev / 'report.json'), 'reportSha256': sha(dev / 'report.json'), 'compare': rel(dev / 'compare.json'),
                   'compareSha256': sha(dev / 'compare.json'), 'framesCompared': null['framesCompared'], 'bitIdentical': null['bitIdentical'],
                   'poseMaxUlps': null['S']['poseMaxUlps'], 'integerMismatchFrames': null['D']['mismatchFrames'],
                   'latch': null.get('latch'), 'wpPositionPrintedMismatchFrames_REPORT': null['S']['wpPositionPrintedMismatchFrames']},
          'comparatorUnitTests': {'log': rel(E / 'test_compare_us026.final.log'), 'sha256': sha(E / 'test_compare_us026.final.log')},
          'runnerTests': {'log': rel(E / 'gate_e2e.test.final.log'), 'sha256': sha(E / 'gate_e2e.test.final.log')}}
    nr['NR-1'] = [{'file': rel(E / f), 'sha256': sha(E / f), 'zero': json.loads((E / f).read_text())['zero'],
                   'note': 'comparator self-vs-self on the v1 dev original runs; engine-independent, copied unchanged from gate-evidence'} for f in ('nr-ba-1-self.json', 'nr-ba-1-self-null.json')]
    nr['NR-3_REPORT'] = [json.loads((E / f).read_text()) | {'file': rel(E / f)} for f in ('nr-ba-3-vs-us026-n1-ba2.json', 'nr-ba-3-vs-us026-n1-ba3.json')]
    nr['result'] = 'PASS' if (null['bitIdentical'] and all(x['zero'] for x in nr['NR-1'])) else 'FAIL'
    nr_file = write_new(E / 'null-run-summary-ba-v2.json', nr)
    muts = []
    for d in a.mutants.split(','):
        m = json.loads((pathlib.Path(d) / 'mutants.json').read_text())
        for x in m['mutants']:
            muts.append({'mutant': x['mutant'], 'decision': x['decision'], 'detected': x['detected'], 'dev': m['dev'], 'file': rel(pathlib.Path(d) / 'mutants.json'),
                         'record': x.get('record'), 'firstIntegerMismatch': x.get('firstIntegerMismatch'), 'poseFirstViolation': x.get('poseFirstViolation'),
                         'latch': x.get('latch')})
    unit = ['CMP-INT: one integer field +1 on one frame (each of fresh, status, reason, keyframeCreated, mapReset, points, world point id list) -> G-*-D FAIL',
            'CMP-ULP: pose component +3 ulp -> PASS, +4 ulp -> FAIL', 'null control 1 ulp -> INVALID', 'R-ND: original-only early difference -> INVALID; engine-only early difference -> FAIL',
            'US026B-M1 (runner, gate_e2e.test.mjs): engine BA-on replay abnormal while off and null replays exit 0 -> counted FAIL']
    big = [x for x in muts if x['mutant'] in ('M-BA-1', 'M-BA-2', 'M-BA-3')]
    sub = [x for x in muts if x['mutant'] == 'M-BA-4']
    ms = {'gate': GID + '.v2', 'kind': 'mutation summary (dev scenario %s only; input variants and copies, decision code unchanged; v2 binary)' % DEV,
          'e2e': muts, 'comparatorUnitMutants': unit, 'comparatorUnitTestLog': rel(E / 'test_compare_us026.final.log'),
          'largerDetected': '%d/%d' % (sum(x['detected'] for x in big), len(big)), 'subResolutionInputDetected': '%d/%d' % (sum(x['detected'] for x in sub), len(sub))}
    ms_file = write_new(E / 'mutation-summary-ba-v2.json', ms)
    larger = 'larger mutations: ' + ', '.join('%s %s' % (x['mutant'], x['decision']) for x in big) + ' (dev e2e, v2 binary; see %s)' % ms_file
    subtxt = ('integer tier: one-unit change of each integer field detected by the comparator (CMP-INT, unit tests); continuous tier: 3 ulp boundary '
              '(+3 PASS, +4 FAIL). End-to-end input one-float32-ulp mutants: ' + ', '.join('%s %s' % (x['mutant'], x['decision']) for x in sub) +
              ' (an undetected one is reported as undetected, not as a detection)')
    tiers = [{'id': 'G-BA-D', 'resolution': INT_RES, 'tolerance': {'value': 0, 'unit': 'exact', 'exact_class_gate': True,
              'exact_class_justification': 'Integer counts, bool flags, enum strings and id lists (fresh, status, reason, worldPoints, wpAdded, wpRemoved, map reset, keyframe created, live keyframes, live points per type, world point id list); two different values differ by at least 1. The same quantities exist on both sides (engine trace_writer.cc:211-258, original memprobe fields). Any upstream float difference can only create mismatches (false FAIL direction, protocol 11 condition 5). Mutation power: CMP-INT unit mutants and M-BA-1/2/3.'}},
             {'id': 'G-BA-S', 'resolution': ULP_RES, 'tolerance': {'value': 3, 'unit': 'float32 ulp of max(|a|,|b|)'}}]
    nind = {'n': 5, 'derivation': 'The independent unit is the input RECORD: TUM VI room1, TUM VI room2, TUM VI room4, Flip6 L-S2, Flip6 L-S3 (one registration scenario each). The BA on/off arms, the null-poke run and any rerun of one record are not independent samples. The dev record Flip6 N1 is not counted. v2 disclosure: these are the SAME five records as ba-e2e v1, whose v1 originals the engine fix US026-CC-1 was diagnosed and checked against; the v2 originals are fresh runs (original runs differ from run to run), so n = 5 counts records, not fresh independent inputs unrelated to the fix.'}
    desc = 'BA: M-BA-1 (BA-off replay as the engine arm), M-BA-2 (numGlobalIterativeIterations=i:9), M-BA-3 (useFastPointAdjustmentInGlobalIterative=b:1), M-BA-4 (one accelerometer value one float32 ulp, sub-resolution input); comparator unit mutants (integer +1, pose 3/4 ulp, null 1 ulp, R-ND); runner unit test of the US026B-M1 rule. Re-run for v2 on the dev scenario with the v2 binary.'
    fields = {
        'classification': ['new version v2 of us026-ba-e2e-gate (engine fix US026-CC-1; criteria, tiers, tolerances, scenarios, arms unchanged)',
                           'original reference runtime (poke) vs our engine exact-input replay', 'desktop headless Chromium + native replay only'],
        'seen_before_registration': 'gate document section 0.1: the v1 gate result (FAIL, PASS 3 / FAIL 2) is known; the registration records were replayed by the fixed engine against their v1 originals (REPORT, known); the v2 original runs (fresh, 5 records x BA-poke and null) are unobserved',
        'resolution': ULP_RES, 'tolerance': {'value': 3, 'unit': 'float32 ulp of max(|a|,|b|) (continuous tiers; integer tiers exact-class, see tiers)'},
        'tiers': tiers,
        'null_run': {'evidence': nr_file, 'result': nr['result']},
        'mutation': {'description': desc, 'evidence': ms_file, 'detected': True, 'sub_resolution_detected': True, 'sub_resolution_mutation': subtxt,
                     'larger_mutations_detected': larger, 'summary': {'largerDetected': ms['largerDetected'], 'subResolutionInputDetected': ms['subResolutionInputDetected']}},
        'independent_samples': nind,
        'original': {'wasm_sha256': '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422',
                     'poke_table': [{'key': 'ba', 'name': 'MapBuilder.bundleAdjustmentIterativeEnabled', 'addr': 606188, 'type': 'u8', 'default': 0, 'values': [0, 1], 'cache_addr': 527080},
                                    {'key': 'mse', 'name': 'MetricScaleEstimator.scaleEstimationMethod', 'addr': 606476, 'type': 'i32', 'default': 1, 'values': [1, 2, 3, 4, 5], 'cache_addr': 527084}],
                     'read_only': {'params_version': 613472, 'ba_latch': 608328}},
        'dev': {'scenarios': [DEV], 'run_log': 'build/slam-refact-full-20261019/us026-draft/dev-run-log.tsv',
                'e2e_outputs': ['build/slam-refact-full-20261019/us026/gate-dev/e2e1', 'build/slam-refact-full-20261019/us026/gate-dev/e2e2', 'build/slam-refact-full-20261019/us026/gate-dev/e2e3'],
                'v2_dev_refresh': rel(a.e2e),
                'v2_dev_refresh_what': 'offline: the e2e3 original runs and bundles re-played with the v2 binary, comparator unchanged (refresh_dev_evidence.py)',
                'replay_pin_check_v1': 'build/slam-refact-full-20261019/us026/pin-check-24fdba2a/pin-check.json'},
        'pre_declared_rules': 'gate document sections 5.2, 5.3 and 7 (R-L load <= 12 inside the lock right before every original run, never relaxed; R-W 4 h wait, before the marker no run used; R-I failure after the marker uses the run, coordinator decision + new version, run.sh stops at the first non-zero exit; R-V validity; R-N null control bit-identical else INVALID; R-ND original difference before the setting can act -> INVALID; R-B BLOCKED only for a PASS without evidence of effect, a FAIL always counts; R-K verdict sentence; R-T constants unchanged) - the v1 rules, unchanged; US026B-M1 is code in v2 (gate document 2.3 C-4)',
        'first_real_run_at_rule': 'After run.sh ends, the coordinator writes first_real_run_at = the birth time (stat --format=%W) of the EARLIEST first-run marker of this gate, floored to whole seconds, ISO with time zone; review.at < first_real_run_at is checked as instants; only the "first_real_run_at": null text is replaced (gate document section 11.3).',
        'coordinator_seed': 'none (no held seed; inputs are recorded registration datasets)',
        'lane': 'US026 (E-1 lane order ended with the v1 runs; v2 is a re-registration after the v1 post-run review and the root-cause fix)',
        'freeze': {'record': pin['freeze_record']['file'], 'record_sha256': pin['freeze_record']['sha256'], 'head': pin['head'], 'head_tree_sha': pin['head_tree_sha'],
                   'note': 'the replay binary is built from git archive of this head; the v2 gate sources under slam_refact/tools/gates/us026-ba-e2e-v2/ are created after this freeze and are pinned by sha256 in pinned.files'},
    }
    st = {'registered_by': 'US-026 ba-e2e v2 worker w1-ba-e2e-v2 (Sonnet 5.5; make_e2e_manifest.mjs + make_e2e_static.py forks); not run on any registration scenario',
          'snapshot': '%s/us026-ba-e2e-gate.v2.snapshot/ (copies of every pinned repo file, SHA256SUMS)' % PRE,
          'command_file': 'slam_refact/tools/gates/us026-ba-e2e-v2/run.sh',
          'supersedes': {'file': 'us026-ba-e2e-gate.v1.md', 'sha256': sha(ROOT / V1_MD), 'manifest': 'us026-ba-e2e-gate.v1.manifest.json', 'manifest_sha256': sha(ROOT / V1_MANIFEST),
                         'v1_result': 'FAIL (registration records: PASS 3 [tum-room1-full, flip6-s3-lowtex, tum-room4-full], FAIL 2 [flip6-s2, tum-room2-full]); the v1 FAIL is not erased by v2 (protocol 7.4)',
                         'change_class': 'engine-fix re-registration: the engine changed (US026-CC-1), the criteria did not; neither a registration defect nor a requirement change (to be confirmed by the pre-reviewer)'},
          'replay_binary': {'path': pin['binary']['path'], 'sha256': pin['binary']['sha256'], 'pin_record': rel(a.pin), 'pin_record_sha256': sha(a.pin),
                            'freeze_records': [pin['freeze_record']['file']], 'built_from': 'git archive %s (FREEZE.r8, engine subtree b732dfc490a0d0258dafae9aaa3a9964d0415e29)' % pin['head'],
                            'v1_binary': pin['v1_binary'],
                            'rule': 'the binary is the build of the frozen snapshot recorded in pin.json; v1 pin rule (dev bundles identical to the lane-A binary) is replaced by the v2 dev refresh (gate document 2)'},
          'registered_scenarios': REG, 'fields': fields}
    write_new(a.out, st)
    print(json.dumps({'static': rel(a.out), 'null_run': nr_file, 'null_result': nr['result'], 'mutation': ms_file, 'larger': ms['largerDetected'], 'subInput': ms['subResolutionInputDetected']}))


if __name__ == '__main__':
    sys.exit(main())
