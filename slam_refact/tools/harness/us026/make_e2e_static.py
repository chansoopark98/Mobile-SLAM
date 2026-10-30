#!/usr/bin/env python3
"""Static (author) part of the us026-ba-e2e-gate.v1 / us026-mse-e2e-gate.v1 manifests, and the null-run / mutation summary
files they cite. Reads the dev evidence (never registration data) and writes NEW files only.

  python3 -I make_e2e_static.py --gate ba|mse --evidence <gate-evidence dir> --e2e <final dev e2e out-root> --mutants <dir>[,<dir>] --out <static.json>
"""
import argparse
import hashlib
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[4]
DEV = {'ba': 'flip6-n1', 'mse': 'flip6-arcore-s3-absolute'}
GID = {'ba': 'us026-ba-e2e-gate', 'mse': 'us026-mse-e2e-gate'}
REG = {'ba': ['tum-room1-full', 'flip6-s2', 'flip6-s3-lowtex', 'tum-room2-full', 'tum-room4-full'],
       'mse': ['tum-room1-full-absolute', 'flip6-s2-absolute', 'flip6-arcore-s2-absolute', 'flip6-arcore-l1-absolute']}
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
    ap.add_argument('--gate', choices=('ba', 'mse'), required=True)
    ap.add_argument('--evidence', type=pathlib.Path, required=True)
    ap.add_argument('--e2e', type=pathlib.Path, required=True)
    ap.add_argument('--mutants', required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    g = a.gate
    E = a.evidence
    dev = a.e2e / GID[g] / DEV[g]
    comp = json.loads((dev / 'compare.json').read_text())
    rep = json.loads((dev / 'report.json').read_text())
    if rep['kind'] != 'dev':
        raise SystemExit('not a dev report')
    null = comp['compare']['null']
    nr = {'gate': GID[g], 'kind': 'null-run summary (dev scenario %s only)' % DEV[g],
          'NR-2': {'what': 'dev null-poke original run vs its exact-input replay, final runner (gate_e2e.mjs run --kind dev)',
                   'report': rel(dev / 'report.json'), 'reportSha256': sha(dev / 'report.json'), 'compare': rel(dev / 'compare.json'),
                   'compareSha256': sha(dev / 'compare.json'), 'framesCompared': null['framesCompared'], 'bitIdentical': null['bitIdentical'],
                   'poseMaxUlps': null['S']['poseMaxUlps'], 'integerMismatchFrames': null['D']['mismatchFrames'],
                   'latch': null.get('latch'), 'wpPositionPrintedMismatchFrames_REPORT': null['S']['wpPositionPrintedMismatchFrames']},
          'comparatorUnitTests': {'log': rel(E / 'test_compare_us026.final.log'), 'sha256': sha(E / 'test_compare_us026.final.log')},
          'runnerTests': {'log': rel(E / 'gate_e2e.test.final.log'), 'sha256': sha(E / 'gate_e2e.test.final.log')}}
    if g == 'ba':
        nr['NR-1'] = [{'file': rel(E / f), 'sha256': sha(E / f), 'zero': json.loads((E / f).read_text())['zero']} for f in ('nr-ba-1-self.json', 'nr-ba-1-self-null.json')]
        nr['NR-3_REPORT'] = [json.loads((E / f).read_text()) | {'file': rel(E / f)} for f in ('nr-ba-3-vs-us026-n1-ba2.json', 'nr-ba-3-vs-us026-n1-ba3.json')]
    else:
        nr['NR-1'] = [{'file': rel(E / f), 'sha256': sha(E / f), 'zero': json.loads((E / f).read_text())['zero']} for f in ('nr-mse-1-self-null.json', 'nr-mse-1-self-m2.json')]
    nr['result'] = 'PASS' if (null['bitIdentical'] and all(x['zero'] for x in nr['NR-1'])) else 'FAIL'
    nr_file = write_new(E / ('null-run-summary-%s.json' % g), nr)
    muts = []
    for d in a.mutants.split(','):
        m = json.loads((pathlib.Path(d) / 'mutants.json').read_text())
        for x in m['mutants']:
            muts.append({'mutant': x['mutant'], 'decision': x['decision'], 'detected': x['detected'], 'dev': m['dev'], 'file': rel(pathlib.Path(d) / 'mutants.json'),
                         'record': x.get('record'), 'firstIntegerMismatch': x.get('firstIntegerMismatch'), 'poseFirstViolation': x.get('poseFirstViolation'),
                         'latch': x.get('latch')})
    unit = ['CMP-INT: one integer field +1 on one frame (each of fresh, status, reason, keyframeCreated, mapReset, points, world point id list) -> G-*-D FAIL',
            'CMP-ULP: pose component +3 ulp -> PASS, +4 ulp -> FAIL', 'null control 1 ulp -> INVALID', 'R-ND: original-only early difference -> INVALID; engine-only early difference -> FAIL']
    if g == 'mse':
        unit += ['CMP-REL: method 3/4 latch scale relative 5e-7 -> PASS, 2e-6 -> FAIL; method 2/5 scale +3 ulp PASS, +4 ulp FAIL', 'latch frame shift -> G-MSE-D FAIL', 'M-ID-5 scale +10 ulp -> FAIL']
    big = [x for x in muts if x['mutant'] in ('M-BA-1', 'M-BA-2', 'M-BA-3') or x['mutant'].startswith('M-MSE-1')]
    sub = [x for x in muts if x['mutant'] in ('M-BA-4', 'M-MSE-2')]
    ms = {'gate': GID[g], 'kind': 'mutation summary (dev scenario %s only; input variants and copies, decision code unchanged)' % DEV[g],
          'e2e': muts, 'comparatorUnitMutants': unit, 'comparatorUnitTestLog': rel(E / 'test_compare_us026.final.log'),
          'largerDetected': '%d/%d' % (sum(x['detected'] for x in big), len(big)), 'subResolutionInputDetected': '%d/%d' % (sum(x['detected'] for x in sub), len(sub))}
    ms_file = write_new(E / ('mutation-summary-%s.json' % g), ms)
    larger = 'larger mutations: ' + ', '.join('%s %s' % (x['mutant'], x['decision']) for x in big) + ' (dev e2e; see %s)' % ms_file
    subtxt = ('integer tier: one-unit change of each integer field detected by the comparator (CMP-INT, unit tests); continuous tier: 3 ulp boundary '
              '(+3 PASS, +4 FAIL). End-to-end input one-float32-ulp mutants: ' + ', '.join('%s %s' % (x['mutant'], x['decision']) for x in sub) +
              ' (an undetected one is reported as undetected, not as a detection)')
    fields = {}
    if g == 'ba':
        tiers = [{'id': 'G-BA-D', 'resolution': INT_RES, 'tolerance': {'value': 0, 'unit': 'exact', 'exact_class_gate': True,
                  'exact_class_justification': 'Integer counts, bool flags, enum strings and id lists (fresh, status, reason, worldPoints, wpAdded, wpRemoved, map reset, keyframe created, live keyframes, live points per type, world point id list); two different values differ by at least 1. The same quantities exist on both sides (engine trace_writer.cc:211-258, original memprobe fields). Any upstream float difference can only create mismatches (false FAIL direction, protocol 11 condition 5). Mutation power: CMP-INT unit mutants and M-BA-1/2/3.'}},
                 {'id': 'G-BA-S', 'resolution': ULP_RES, 'tolerance': {'value': 3, 'unit': 'float32 ulp of max(|a|,|b|)'}}]
        nind = {'n': 5, 'derivation': 'The independent unit is the input RECORD: TUM VI room1, TUM VI room2, TUM VI room4, Flip6 L-S2, Flip6 L-S3 (one registration scenario each). The BA on/off arms, the null-poke run and any rerun of one record are not independent samples. The dev record Flip6 N1 is not counted.'}
        desc = 'BA: M-BA-1 (BA-off replay as the engine arm), M-BA-2 (numGlobalIterativeIterations=i:9), M-BA-3 (useFastPointAdjustmentInGlobalIterative=b:1), M-BA-4 (one accelerometer value one float32 ulp, sub-resolution input); comparator unit mutants (integer +1, pose 3/4 ulp, null 1 ulp, R-ND).'
    else:
        tiers = [{'id': 'G-MSE-D', 'resolution': INT_RES, 'tolerance': {'value': 0, 'unit': 'exact', 'exact_class_gate': True,
                  'exact_class_justification': 'The G-BA-D integer fields plus the per-frame latch flag (engine srScaleLatched vs original VioTracker latch u8 @b+2840) and the latch frame; bool/integer values differ by at least 1; false FAIL direction only. Mutation power: CMP-INT, latch-frame shift unit mutants, M-MSE-1.'}},
                 {'id': 'G-MSE-S-ulp', 'resolution': ULP_RES, 'tolerance': {'value': 3, 'unit': 'float32 ulp of max(|a|,|b|)', 'applies_to': 'latched scale, methods 1, 2, 5 (and M-ID-5)'}},
                 {'id': 'G-MSE-S-rel', 'resolution': {'value': 1.1920929e-07, 'unit': 'relative (float32 spacing 2^-23)', 'metric': '|a-b|/max(|a|,|b|) of the latched scale'},
                  'tolerance': {'value': 1e-06, 'unit': 'relative', 'applies_to': 'latched scale, methods 3, 4'}},
                 {'id': 'G-MSE-P', 'resolution': ULP_RES, 'tolerance': {'value': 3, 'unit': 'float32 ulp of max(|a|,|b|)', 'applies_to': 'published pose every frame; methods 3/4 frames from the original latch frame are REPORT when the two latched scales are not bit-identical'}}]
        nind = {'n': 4, 'derivation': 'The independent unit is the input RECORD: TUM VI room1, Flip6 L-S2, Flip6 ARCore A-S2, Flip6 ARCore physical-live-01 (one absolute registration scenario each). Methods 2-5, the method-1 arm, the null-poke run and reruns are not independent samples. The dev record Flip6 ARCore A-S3 is not counted.'}
        desc = 'MSE: M-MSE-1 (method-1 replay as the method-N engine arm, N = 2, 3, 4), M-MSE-2 (one linear-acceleration value one float32 ulp, sub-resolution input); comparator unit mutants (integer +1, latch frame, scale ulp/relative boundaries, M-ID-5, null 1 ulp, R-ND).'
    fields = {
        'classification': ['new gate (v1)', 'original reference runtime (poke) vs our engine exact-input replay', 'desktop headless Chromium + native replay only'],
        'seen_before_registration': 'gate document section 0.1: dev results known; the poked-setting results of the registration records unobserved',
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
        'dev': {'scenarios': [DEV[g]], 'run_log': 'build/slam-refact-full-20261019/us026-draft/dev-run-log.tsv',
                'e2e_outputs': ['build/slam-refact-full-20261019/us026/gate-dev/e2e1', 'build/slam-refact-full-20261019/us026/gate-dev/e2e2', rel(a.e2e)],
                'replay_pin_check': 'build/slam-refact-full-20261019/us026/pin-check-24fdba2a/pin-check.json'},
        'pre_declared_rules': 'gate document sections 5.2, 5.3 and 7 (R-L load <= 12 inside the lock right before every original run, never relaxed; R-W 4 h wait, before the marker no run used; R-I failure after the marker uses the run, coordinator decision + new version, run.sh stops at the first non-zero exit; R-V validity; R-N null control bit-identical else INVALID; R-ND original difference before the setting can act -> INVALID; R-B BLOCKED only for a PASS without evidence of effect, a FAIL always counts; R-K verdict sentence; R-T constants unchanged)',
        'first_real_run_at_rule': 'After run.sh ends, the coordinator writes first_real_run_at = the birth time (stat --format=%W) of the EARLIEST first-run marker of this gate, floored to whole seconds, ISO with time zone; review.at < first_real_run_at is checked as instants; only the "first_real_run_at": null text is replaced (gate document section 11.3).',
        'coordinator_seed': 'none (no held seed; inputs are recorded registration datasets)',
        'lane': 'US026 (E-1: us026-mse-unit v1 -> us026-ba-e2e v1 -> us026-mse-e2e v1)',
    }
    st = {'registered_by': 'US-026 lane C2 executor (make_e2e_manifest.mjs + make_e2e_static.py); not run on any registration scenario',
          'snapshot': 'build/slam-refact-full-20261019/preregistration/%s.v1.snapshot/ (copies of every pinned repo file, SHA256SUMS)' % GID[g],
          'command_file': 'slam_refact/tools/gates/%s/run.sh' % ('us026-ba-e2e-v1' if g == 'ba' else 'us026-mse-e2e-v1'),
          'replay_binary': {'path': 'build/slam-refact-full-c2-20261020-or1pair/native/harness/slam_refact_replay',
                            'sha256': '24fdba2a2b0da97d2f9c12770b19abf12a5f50d42fcc73359b699b47705000a2',
                            'freeze_records': ['build/slam-refact-full-c2-20261020/FREEZE.json', 'build/slam-refact-full-c2-20261020/FREEZE.r2.json', 'build/slam-refact-full-c2-20261020/FREEZE.r3.json'],
                            'rule': 'pinned because it replays the 22 lane-A dev bundles identically to the binary lane A used (pin-check.json); otherwise BLOCKED'},
          'registered_scenarios': REG[g], 'fields': fields}
    write_new(a.out, st)
    print(json.dumps({'static': rel(a.out), 'null_run': nr_file, 'null_result': nr['result'], 'mutation': ms_file, 'larger': ms['largerDetected'], 'subInput': ms['subResolutionInputDetected']}))


if __name__ == '__main__':
    sys.exit(main())
