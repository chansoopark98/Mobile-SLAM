#!/usr/bin/env python3
"""us025-lc-e2e-gate.v2: the static (author) part of the registration manifest and the null-run / mutation summary files it cites (written NEW, never overwritten).
FORK of ../us025-lc-e2e-v1/make_lc_static.py. Changed: v2 names, `supersedes` (v1 .md and manifest sha256), the v2 exposure record, the whole-run comparison's null and mutation
evidence (D-V2-2) and the re-judgement of the v1 data (D-V2-1/2). Reads DEV evidence only (the LC-6a/LC-G dev runs, the LC-6a comparator mutants, the v1 parameter mutants,
the READ-ONLY v1 gate output through the whole-run null / mutants / re-judgement files, unit-test logs); v2 registration scenarios are never run.

  python3 -I make_lc_static.py --evidence <new dir for the summaries> --lc6a build/slam-refact-final-20261025/lc6a --params-mutants <mutants_params.sh out dir (the v1 one)> \
      --unit-logs <dir with judge.test.log, gate_lc.test.log> --full-null <dir of compare_full_us025.py outputs on the v1 NO_TRIGGER runs> \
      --full-mutants <mutants_full.py out dir> --rejudge <rejudge_v1_data.mjs output> --v1-prereg <v1 preregistration dir> --out <new static.json>

The summaries (null-run-summary-lc-v2.json, mutation-summary-lc-v2.json) are what the manifest's null_run.evidence / mutation.evidence point at. Nothing here picks a
tolerance: tiers and tolerances are the ones declared in docs/spec/12-loop-close.md §8 and in compare_us025.py before any result (registration document §4).
"""
import argparse
import hashlib
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[4]
GID = 'us025-lc-e2e-gate'
ULP_RES = {'value': 1, 'unit': 'float32 ulp of max(|a|,|b|)', 'metric': 'per component |a-b| / ulp32(max(|a|,|b|))'}
INT_RES = {'value': 1, 'unit': 'integer unit (count, flag, enum, id, byte of a descriptor)', 'metric': 'equality of integer-class fields'}
REG = ['flip6-s3-lowtex', 'flip6-s2', 'tum-room1-full', 'tum-room2-full', 'tum-room4-full']


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
    ap.add_argument('--lc6a', type=pathlib.Path, required=True)
    ap.add_argument('--params-mutants', type=pathlib.Path, required=True)
    ap.add_argument('--unit-logs', type=pathlib.Path, required=True)
    ap.add_argument('--full-null', type=pathlib.Path, required=True)
    ap.add_argument('--full-mutants', type=pathlib.Path, required=True)
    ap.add_argument('--rejudge', type=pathlib.Path, required=True)
    ap.add_argument('--v1-prereg', type=pathlib.Path, required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    a.evidence.mkdir(parents=True, exist_ok=True)
    suite = json.loads((a.lc6a / 'suite-final.json').read_text())
    nulls = [r for r in suite['results'] if r['arm'] == 'null']
    nr = {'gate': GID + '.v2', 'kind': 'null-run summary (DEV runs of LC-6a; the registration scenarios were not run)',
          'NR-LC-1': {'what': 'original lc=0 run replayed by the engine with loop close OFF, compare_us025.py null, whole run',
                      'source': rel(a.lc6a / 'suite-final.json'), 'sourceSha256': sha(a.lc6a / 'suite-final.json'),
                      'runs': [{'name': r['name'], 'scenario': r.get('scenario'), 'verdict': r['verdict'], 'framesCompared': r['checks'][0]['compared'],
                                'bitIdentical': r['checks'][0].get('bitIdentical'), 'poseMaxUlps': (r['checks'][0].get('S') or {}).get('poseMaxUlps'),
                                'integerMismatchFrames': ((r['checks'][0].get('D') or {}).get('mismatchFrames'))} for r in nulls]},
          'NR-LC-2': {'what': 'comparator self-check: wrong pairs are refused or FAIL (negative controls of LC-6a)', 'dir': rel(a.lc6a / 'negctl'),
                      'files': sorted(p.name for p in (a.lc6a / 'negctl').glob('*.json'))},
          'NR-LC-3': {'what': 'D-V2-2: the whole-run ON comparison (compare_full_us025.py) of the engine ON replay of a real original lc=1 run WITHOUT a loop close success against its own original, '
                              'whole run (the measurement floor of that comparison; dev data = the stored v1 gate output, read-only)',
                      'dir': rel(a.full_null), 'runs': [{'file': f.name, 'verdict': json.loads(f.read_text())['verdict'],
                                                         'attempts': next(c['compared'] for c in json.loads(f.read_text())['checks'] if c['name'] == 'attemptFrameSet'),
                                                         'traceFrames': next(c['compared'] for c in json.loads(f.read_text())['checks'] if c['name'] == 'traceAndWorldPointsWholeRun'),
                                                         'declaredU2': sum(c.get('declaredU2', 0) for c in json.loads(f.read_text())['checks']),
                                                         'mismatch': sum(c.get('mismatch', 0) for c in json.loads(f.read_text())['checks'])} for f in sorted(a.full_null.glob('*.json'))]},
          'comparatorUnitTests': 'slam_refact/tools/harness/us025/lc6a/test_compare_us025.py (13 tests, LC-6a); slam_refact/tools/gates/us025-lc-e2e-v2/test_compare_full_us025.py (15 tests, v2)',
          'result': 'PASS' if nulls and all(r['verdict'] == 'PASS' for r in nulls) and a.full_null.is_dir() and all(json.loads(f.read_text())['verdict'] in ('PASS', 'PASS_DECLARED') for f in a.full_null.glob('*.json')) else 'FAIL',
          'resolution': 'measured floor 0 (bit identical over all frames and the memprobe integers); the representation resolution is 1 ulp (float32) / 1 integer unit'}
    nr_file = write_new(a.evidence / 'null-run-summary-lc-v2.json', nr)
    comp = []
    for d in ('mutants-rerun', 'mutants-n1'):
        m = json.loads((a.lc6a / d / 'mutants.json').read_text())
        for t in m['table']:
            comp.append({'dir': d, 'run': t['run'], 'mutant': t['mutant'], 'verdict': t['verdict'], 'expected': t['expected'], 'fired': t['fired'], 'ok': t['ok']})
    pm = []
    for line in (a.params_mutants / 'results.tsv').read_text().splitlines():
        run, m, rrc, crc, txt = (line.split('\t') + [''] * 5)[:5]
        pm.append({'run': run, 'mutant': m, 'replay': rrc, 'compare': crc, 'verdict': 'FAIL' if crc == 'compare_rc=10' else 'PASS*' if crc == 'compare_rc=0' else crc, 'text': txt.strip()})
    pm.sort(key=lambda x: (x['run'], x['mutant']))
    ctrl_ok = all(x['verdict'] == 'PASS*' for x in pm if x['mutant'] == 'CTRL')
    large = [x for x in pm if x['mutant'] in ('D120', 'D40', 'M30', 'C10')]
    bound = [x for x in pm if x['mutant'] in ('D79', 'M74', 'C34')]
    undetected_large = sorted({x['mutant'] for x in large if x['verdict'] != 'FAIL'})
    fm = json.loads((a.full_mutants / 'mutants.json').read_text())
    fmut = [t for t in fm['table'] if t['expected'] == 'FAIL']
    ms = {'gate': GID + '.v2', 'kind': 'mutation summary (DEV runs only; mutated COPIES of replay directories and input variants of dev bundles; no source touched)',
          'wholeRunMutants': {'what': 'D-V2-2: mutants_full.py, one quantity of a COPY of an engine ON replay of a real original lc=1 run without a success changed; compare_full_us025.py must FAIL; '
                              'the F-limited compare_us025.py on arm answers BLOCKED for the same copies', 'file': rel(a.full_mutants / 'mutants.json'),
                              'allExpectationsHold': fm['allExpectationsHold'], 'mutantsExpectedFail': len(fmut), 'detected': sum(t['verdict'] == 'FAIL' for t in fmut),
                              'v1ArmOnBlocked': sum(t.get('v1ArmOn') == 'BLOCKED' for t in fmut),
                              'documentedHoles': [{'run': t['run'], 'mutant': t['mutant'], 'verdict': t['verdict']} for t in fm['table'] if t['mutant'] == 'cursor-at-u2-attempt'],
                              'controls': [{'mutant': t['mutant'], 'verdict': t['verdict'], 'ok': t['ok']} for t in fm['table'] if t['mutant'].startswith('control')]},
          'comparatorMutants': {'what': 'LC-6a mutants_lc6a.py: one quantity of a real engine replay directory changed, ON comparison must FAIL (3 ulp PnP boundary PASS, 4 ulp FAIL)',
                                'files': [rel(a.lc6a / d / 'mutants.json') for d in ('mutants-rerun', 'mutants-n1')], 'rows': comp,
                                'allExpectationsHold': all(x['ok'] for x in comp)},
          'engineParameterMutants': {'what': 'mutants_params.sh: the dev ON bundle with ONE loop-close threshold changed (WorldMaps.loopClosing*), replayed by the dev slam_refact_replay_lc6a, compare_us025.py on. '
                                     'CTRL = unmutated ON bundle (must pass). Large = threshold moved far; boundary = one unit.',
                                     'dir': rel(a.params_mutants), 'resultsSha256': sha(a.params_mutants / 'results.tsv'), 'controlPasses': ctrl_ok, 'rows': pm,
                                     'largeDetected': '%d/%d' % (sum(x['verdict'] == 'FAIL' for x in large), len(large)),
                                     'boundaryDetected': '%d/%d' % (sum(x['verdict'] == 'FAIL' for x in bound), len(bound)),
                                     'largeNotDetected': undetected_large,
                                     'honesty': 'a mutant whose compare verdict is PASS* was NOT detected; the registration document lists these as undetected classes'},
          'unitTests': None}
    ul = a.unit_logs
    ms['unitTests'] = {'judge_test_log': rel(ul / 'judge.test.log'), 'judge_test_log_sha256': sha(ul / 'judge.test.log'),
                       'gate_lc_test_log': rel(ul / 'gate_lc.test.log'), 'gate_lc_test_log_sha256': sha(ul / 'gate_lc.test.log'),
                       'compare_full_test_log': rel(ul / 'test_compare_full_us025.log'), 'compare_full_test_log_sha256': sha(ul / 'test_compare_full_us025.log')}
    ms_file = write_new(a.evidence / 'mutation-summary-lc-v2.json', ms)
    detected = bool(ms['comparatorMutants']['allExpectationsHold'] and ctrl_ok and any(x['verdict'] == 'FAIL' for x in large) and fm['allExpectationsHold'])
    sub = ('1 ulp / 1 bit: trace pose component +1 ulp (trace-pose-1ulp), correction matrix one bit (C-one-bit), descriptor one byte (descriptor-byte), out+0x1c, trace integer +1 '
           '(LC-6a comparator mutants, all FAIL on 4 dev runs); PnP-after pose 3 ulp stays PASS and 4 ulp FAILs (the declared tolerance boundary).')
    larger = ('candidate id, ring cursor, slot count, live count, dropped attempt, stage class, missing engine success (comparator mutants, all FAIL); engine parameter mutants '
              + ', '.join('%s %s' % (x['run'] + '/' + x['mutant'], x['verdict']) for x in large) + ' (see %s)' % ms_file)
    tiers = [
        {'id': 'G-LC-D', 'resolution': INT_RES, 'tolerance': {'value': 0, 'unit': 'exact', 'exact_class_gate': True,
          'exact_class_justification': 'Counts, flags, enums, ids, ring words and descriptor bytes (attempt frame set, slot/live/free counts, newest handle, ring [index,id,ring0..2,cursor], candidate id, threshold stage, out+0x1c, trigger frame set, agate 32 B descriptors, trace integer tier, world point id lists). Two different values differ by at least 1 unit. The same quantities exist on both sides (engine LoopCloseReport / trace_writer, original lcgold.js + memprobe). Any upstream float difference can only create mismatches (false-FAIL direction, protocol 11 condition 5). Declared in docs/spec/12-loop-close.md section 8 (G-A, G-C, G-G, G-H) before any result. Power: comparator mutants and parameter mutants.'}},
        {'id': 'G-LC-B', 'resolution': ULP_RES, 'tolerance': {'value': 0, 'unit': 'exact (float32 bit pattern)', 'determinism_class_gate': True,
          'protocol_exception': {'category': 'pre-declared bit tolerance 0: an explicit EXCEPTION to the protocol section 3 three-times rule; NOT an exact-class gate (section 11: not the same integer operands) and NOT a same-path determinism comparison (section 11.1 condition 1: it compares a different implementation, the original wasm against the re-implementation)',
                                 'flag_note': 'the flag name determinism_class_gate is kept only because the section 6 verification snippet needs one of the two flags to accept tolerance 0; it is not a claim of section 11.1',
                                 'granted_by': 'Task #34 pre-review (reviews/prereview-us025-lc-e2e-v1.md section 1, Q1)',
                                 'direction': 'only the false-FAIL direction: any nonzero difference is a FAIL; noise can never make a PASS',
                                 'grounds': ['docs/spec/12-loop-close.md section 8 G-F: C compared bit for bit; the wording was tightened from "3 ulp or bit" to "bit" in 0a5c5c4, AFTER the LC-G golden showed 7/7 bit identity of the vendored multiplication with the original 660 (so the bit rule for C was NOT declared before every dev observation)',
                                             'the engine library is built with -fno-fast-math -ffp-contract=off -march=x86-64 (SlamRefactFlags.cmake) and wasm has no FMA: both sides run sequential IEEE operations; no known platform noise source for 3 ulp to absorb',
                                             'a 3 ulp rule would not loosen C in practice: a 1 ulp change of the PnP pose moves the mm-scale correction by hundreds of ulp (synthetic simulation, median about 780 ulp, 0.15 percent within 3 ulp; INFERRED)',
                                             'trace pose components before F: 0 ulp comes from compare_us026.compare and from observation (null arm floor 0 over 1448 dev frames; ba-e2e v2 maxima 0 ulp on 5 scenarios), NOT from spec 12 section 8; the same quantity had a 3 ulp tier in ba-e2e v2',
                                             'cost accepted: a single 1 ulp difference anywhere FAILs the gate and consumes the one-run marker (new version needed)'],
                                 'pnp_shadow': 'G-LC-P (3 ulp for the pose after PnP) is shadowed by the bit check of C in practice: a 1 ulp PnP difference fails C first'},
          'exact_class_justification': 'See protocol_exception. Correction matrix C (128 bytes), the pre-call newest pose (hook runs only) and the F-before trace pose components are compared as float32 bit patterns with tolerance 0 as a PRE-DECLARED EXCEPTION to the three-times rule (not exact-class, not section 11.1). The NULL arm measured the floor at exactly 0 on dev data (3 runs, 1448 frames).'}},
        {'id': 'G-LC-P', 'resolution': ULP_RES, 'tolerance': {'value': 3, 'unit': 'float32 ulp of max(|a|,|b|)'}},
    ]
    nind = {'n': 1, 'derivation': 'UNCHANGED from v1 (v2 records new runs of the same five datasets, so it adds no independent sample). The independent unit is the input RECORD (scenario dataset): flip6-s3-lowtex, flip6-s2, tum-room1-full, tum-room2-full, tum-room4-full are registered, but the dev trigger survey saw '
            'loop close successes only in flip6-s3-lowtex (and in the dev scenario flip6-n1, which is excluded), so the guaranteed lower bound on evaluated records is n = 1: PASS needs K_MIN = 3 evaluated '
            'original runs, which may all be repeated recordings of ONE dataset. Repeated original runs of one dataset (up to 16 lc=1 runs of flip6-s3-lowtex) differ in trajectory but share the dataset and the '
            'trigger mechanism and are NOT counted as independent samples; the ON, OFF and NULL arms of one run are not independent either. The verdict sentence is fixed by judge.mjs (claim) and says "ONE dataset '
            'only (thin sample)" when fewer than 2 scenarios were evaluated. The number actually evaluated is reported in the result document.'}
    desc = ('Comparator mutants (LC-6a mutants_lc6a.py, 4 dev runs) and engine parameter mutants (mutants_params.sh: D120, D40, M30, C10 large; D79, M74, C34 boundary) on dev data; '
            'judge.mjs and gate_lc.mjs unit tests (INVALID/INFRA/TOOL_ERROR/US025-M1/floor/K_MIN rules). Source-level engine mutants (ring update skipped, correction inverted) were NOT built in this pass.')
    fields = {
        'classification': ['us025-lc-e2e-gate v2 (US-025 loop close, LC-7): change class = RESULT-DEPENDENT REQUIREMENT CHANGE (D-V2-1 and the registered floors relaxed after the v1 INVALID, D-V2-2 tightened); not a registration defect', 'original reference runtime (poke lc=1, plain wasm) vs our engine exact-input replay, up to the first loop close success (N2)',
                           'desktop headless Chromium + native replay only', 'NOT a real-device or production accuracy statement'],
        'seen_before_registration': 'V2 ADDITION (the strongest exposure of this gate): the COMPLETE v1 first-run result is known: verdict INVALID (exit 8), s3-lowtex EVALUATED 6 / NO_TRIGGER 8 / INFRA 2, flip6-s2 EVALUATED 1 / NO_TRIGGER 2 / INFRA 3 / INVALID 2 (candidate picks compared 1 < 2), room1/2/4 NO_TRIGGER 12 of 12, INFRA 5 of 36, NULL_ZERO 10 of 10, k_runs 6, independent_samples.n 1; D-V2-1 (THIN) and its 25 percent bound are a relaxation chosen after seeing it (pre-review C1: denominator = runs that reached a success, scenario with THIN > EVALUATED not evaluated), and the v1 data re-judged with the v2 rules PASS (k_runs 6, k_scenarios 1) only with the carried-U2 rule of compare_full_us025.py, which was itself added after the v1 data showed 2 of 22 false FAILs and narrowed by the pre-review (C2) (registration v2 sections 2, 4). Then the v1 text: registration document 2: the LC-0 trigger survey (flip6-s2, flip6-s3-lowtex, tum-room1-full, tum-room4-full, flip6-n1), the LC-G goldens (flip6-s3-lowtex, flip6-n1), the LC-6b synthetic scene (plain lc=1 runs hung 4/4, one debugger run, non-self candidates reached the match stage with at most 40 matches) and the LC-6a dev A/B (6 ON runs: 4 PASS, 2 PASS_DECLARED, OFF 6/6, NULL 3/3) are known and were produced with the same comparator and the same declared tolerances. All gate original runs are NEW (recorded after the marker).',
        'resolution': ULP_RES, 'tolerance': {'value': 0, 'unit': 'exact for integer and bit tiers; 3 float32 ulp only for the PnP-after pose (tiers)'},
        'tiers': tiers,
        'null_run': {'evidence': nr_file, 'result': nr['result']},
        'mutation': {'description': desc, 'evidence': ms_file, 'detected': detected, 'sub_resolution_detected': bool(comp) and ms['comparatorMutants']['allExpectationsHold'], 'sub_resolution_mutation': sub,
                     'larger_mutations_detected': larger, 'summary': {'comparatorAllExpectationsHold': ms['comparatorMutants']['allExpectationsHold'],
                                                                  'parameterControlPasses': ctrl_ok, 'largeDetected': ms['engineParameterMutants']['largeDetected'],
                                                                  'boundaryDetected': ms['engineParameterMutants']['boundaryDetected'], 'largeNotDetected': undetected_large}},
        'independent_samples': nind,
        'rejudge_of_v1_data': {'file': rel(a.rejudge), 'sha256': sha(a.rejudge), 'note': 'exposure computation, not a v2 result'},
        'original': {'wasm_sha256': '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422', 'run_options_lc1': '--memprobe --memprobe-lc --memprobe-detail 12 --record-inputs --lcgold --poke {"lc":1}',
                     'run_options_lc0': '--memprobe --memprobe-lc --memprobe-detail 12 --record-inputs --poke {"lc":0}', 'instrumented_wasm': False,
                     'poke': {'key': 'lc', 'name': 'MapBuilder.loopCloseEnabled', 'addr': 606264, 'type': 'u8', 'default': 0, 'values': [0, 1], 'cache_addr': 527080}},
        'dev': {'scenarios': ['flip6-n1', 'flip6-arcore-s3-absolute'], 'lc6a_report': 'slam_refact/docs/design/us025-lc6a-ab.md', 'lc0_survey': 'slam_refact/docs/design/us025-lc0-trigger-survey.md',
                'lcg_golden': 'slam_refact/docs/design/us025-lcg-golden.md'},
        'pre_declared_rules': 'registration document sections 5, 6 and 7 (v2: THIN class and the 25 percent bound D-V2-1, the whole-run NO_TRIGGER comparison D-V2-2; run, scenario and gate classes; INFRA / TOOL_ERROR / INVALID / BLOCKED / INSUFFICIENT; K_MIN; floors; US025-M1 and M2 (any abnormal engine replay exit is a FAIL unless ENOSPC); load <= 8 inside the lock; 4 h wait; no re-run; watchdog 600 s + 120 s grace per original run; start condition disk <= 90 percent)',
        'first_real_run_at_rule': 'After run.sh ends, the coordinator writes first_real_run_at = the birth time (stat --format=%W) of the EARLIEST first-run marker of this gate, floored to whole seconds, ISO with time zone; review.at < first_real_run_at is checked as instants; only the "first_real_run_at": null text is replaced (registration document section 11).',
        'coordinator_seed': 'none (no held seed; inputs are recorded registration datasets)',
        'lane': 'US025 (LC-7); the lane order of preregistration-protocol 13.1 applies (no earlier first run of this lane is pending)',
        'exposure': {'v1_result_known_before_registration': True, 'd_v2_1_relaxation_chosen_after_results': True, 'thin_fraction_025_chosen_after_results': True, 'd_v2_2_carried_u2_rule_added_after_v1_data_showed_false_fail': True, 'v1_data_rejudged_with_v2_rules': 'PASS k_runs 6, k_scenarios 1 (C1 and C2 applied; flip6-s2 not evaluated: THIN 2 > EVALUATED 1); without a carry rule 2 of 22 whole-run comparisons FAIL (measured before the pre-review)', 'prereview_tightened_carry_rule_tested_by_reviewer_on_the_22_runs': True, 'null_22_runs_are_the_data_the_carry_rule_was_designed_on': True, 'v3_lock': 'registration v2 section 2.6', 'tolerances_chosen_after_results': False, 'g_f_bit_rule_tightened_after_lcg_golden': True, 'trace_pose_zero_ulp_source': 'compare_us026.compare and null-run observation, not spec 12 section 8', 'compare_us025_py_added_in_the_commit_of_the_lc6a_result': 'UNVERIFIED whether the constants predate the result (they are copied from spec 12 section 8)', 'floors_checked_against_dev_minima': True, 'run_counts_informed_by_dev_trigger_frequency': True,
                     'note': 'tolerances are the spec 12 section 8 / compare_us025.py declarations that predate the LC-6a result; SCENARIO_RUNS (16 for flip6-s3-lowtex, 8 for flip6-s2, 4 elsewhere) was chosen knowing the dev trigger frequency (registration document 3.2)'},
    }
    v1md, v1mf = a.v1_prereg / 'us025-lc-e2e-gate.v1.md', a.v1_prereg / 'us025-lc-e2e-gate.v1.manifest.json'
    supersedes = {'file': v1md.name, 'sha256': sha(v1md), 'manifest': v1mf.name, 'manifest_sha256': sha(v1mf),
                  'v1_result': 'INVALID (run.sh exit 8, 2026-10-26T19:08:47+09:00 to 2026-10-27T00:10:10+09:00): k_runs 6 of 1 scenario, flip6-s2 INVALID by the candidate-pick floor, room1/2/4 BLOCKED, INFRA 5; the v1 INVALID is not erased by v2 (protocol 7.4)',
                  'change_class': 'result-dependent requirement change (relaxation D-V2-1, tightening D-V2-2), made after the v1 result was known; to be confirmed by the pre-reviewer'}
    st = {'registered_by': 'US-025 LC-7 v2 worker (Sonnet 5.5; draft; make_lc_manifest.mjs + make_lc_static.py); not run on any registration scenario',
          'snapshot': '<prereg dir>/us025-lc-e2e-gate.v2.snapshot/ (copies of every pinned repo file + SHA256SUMS; make_snapshot.sh, run by the coordinator at registration)',
          'command_file': 'slam_refact/tools/gates/us025-lc-e2e-v2/run.sh', 'supersedes': supersedes, 'registered_scenarios': REG, 'fields': fields}
    write_new(a.out, st)
    print(json.dumps({'static': rel(a.out), 'null_run': nr_file, 'null_result': nr['result'], 'mutation': ms_file, 'large': ms['engineParameterMutants']['largeDetected'],
                      'boundary': ms['engineParameterMutants']['boundaryDetected'], 'largeNotDetected': undetected_large, 'controlPasses': ctrl_ok}))


if __name__ == '__main__':
    sys.exit(main())
