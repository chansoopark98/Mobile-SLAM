#!/usr/bin/env python3
"""OR-4 (gate v6): fills the evidence fields of the registered manifest after the null-runs and mutations (before review).

Writes only: resolution, tolerance, tiers, null_run, mutation, noise_table, input_pins (noise table and N arm trace list
sha256), seen_before_registration, not_seen, timeline_honesty, evidence_files. Refuses when a review is recorded or the first
real run happened, and refuses to touch the identity fields (gate_id, sha256, created_at, pins). Reads the null-report and
refuses unless the registered conditions hold (G-DET-N, G-H, every required mutation behaved as registered).
Also `repin <reason>`: before review only, records changed pins in pin_log. Not part of the measurement; not pinned.
"""
import json
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import or4_world_regression as g  # noqa: E402

ULP32 = 1.1920929e-07
DET_WHY = ('protocol 11.1: both sides are the same engine code path ({what}); the factor under test is {factor}. '
           'Mutants: a one-ulp change of one float in a trace copy, a deleted row, and another input / seed are all reported as '
           'different by identical() (tests/test_or4_world_regression.py, noise_runs_change_trace of the null-report); the larger '
           'engine mutants are drop-every:7, orient-beta:3.0 and orient-drift:0.5 of the mutation report. Scope: determinism and '
           'wiring of the target-less call sequence (G-H) / of the target path (G-DET-T); not agreement across hardware or with the original.')


def repin(reason):
    m = json.loads(g.MANIFEST.read_text())
    if m.get('review') or m.get('first_real_run_at'):
        raise SystemExit('review or first real run already recorded: pins are not edited any more')
    live, ins = g.live_pins(), g.live_inputs()
    log = m.setdefault('pin_log', [])
    stamp = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    for table, new in (('pins', live), ('input_pins', ins)):
        for k, v in new.items():
            if m[table].get(k) != v:
                log.append({'at': stamp, 'table': table, 'key': k, 'old': m[table].get(k), 'new': v, 'reason': reason})
                m[table][k] = v
    snap = g.PREREG / f'or4-snapshot-pre-v{g.VERSION}'
    tag = stamp.replace(':', '').replace('+', 'p')
    for rel in g.PIN_SOURCES:
        src, old = g.ROOT / rel, snap / rel.replace('/', '__')
        if old.exists() and g.sha256_file(old) != g.sha256_file(src):
            (snap / (rel.replace('/', '__') + '.' + tag)).write_bytes(src.read_bytes())
    g.MANIFEST.write_text(json.dumps(m, indent=1) + '\n')
    print('repinned:', [e['key'] for e in log if e['at'] == stamp])


def main():
    if len(sys.argv) > 2 and sys.argv[1] == 'repin':
        return repin(sys.argv[2])
    m = json.loads(g.MANIFEST.read_text())
    if m.get('review') or m.get('first_real_run_at'):
        raise SystemExit('review or first real run already recorded: the manifest is not edited any more')
    rep_path = g.OR4V / 'null/null-report.json'
    rep = json.loads(rep_path.read_text())
    if not (rep['G-DET-N-pass'] and rep['G-H-pass'] and rep['G-DET-D-pass'] and rep['mutations-required-ok']):
        raise SystemExit('null-run conditions not met: G-DET-N %s, G-H %s, G-DET-D %s, mutations %s (v6 required, nothing written)'
                         % (rep['G-DET-N-pass'], rep['G-H-pass'], rep['G-DET-D-pass'], rep['mutations-required-ok']))
    rows_min = min(len(g.load_trace(g.OR4V3 / 'null/N' / k / 's0/trace.jsonl')) for k, _, _, _ in g.BUNDLES)
    det = lambda what, factor: {'value': 0, 'unit': 'exact', 'determinism_class_gate': True, 'exact_class_justification': DET_WHY.format(what=what, factor=factor)}
    ulp = {'value': ULP32, 'unit': 'float32 ulp at 1.0 / one JSON digit'}
    band = lambda mid, res, unit_res, floor, unit_tol: {'id': f'G-BAND-{mid}', 'resolution': {'value': res, 'unit': unit_res},
                                                         'tolerance': {'value': floor, 'unit': unit_tol + '; per bundle max(3 R, floor)'}}
    m['tiers'] = [
        {'id': 'G-DET-N', 'resolution': ulp, 'tolerance': det('sr_or4_replay without targets, run twice', 'the process / execution instant')},
        {'id': 'G-H', 'resolution': ulp, 'tolerance': det('sr_or4_replay and slam_refact_replay link the same engine libraries; sr_or4_replay copies the replay_main.cc call sequence', 'the harness wiring')},
        {'id': 'G-DET-D', 'resolution': ulp, 'tolerance': det('sr_or4_replay without targets and allOrb/allGorb forced (the D arm), run twice', 'the process / execution instant')},
        {'id': 'G-DET-T', 'resolution': ulp, 'tolerance': det('sr_or4_replay with the target, run twice', 'the process / execution instant')},
        band('pos', 1e-6, 'fraction of the RMS radius (float32 pose ulp / S)', g.FLOOR['pos'], 'fraction of the RMS radius'),
        band('rot', 1e-5, 'deg (float32 quaternion ulp through 2 atan2)', g.FLOOR['rot'], 'deg'),
        band('scale', 1e-6, '|ln s| of the Sim(3) scale (float32 positions); same-trace floor measured by NR-4', g.FLOOR['scale'], '|ln s|'),
        band('cov', 1, 'frame', g.FLOOR['cov'], 'frames'),
        band('norm', 1, 'frame', g.FLOOR['norm'], 'frames'),
        band('reset', 1, 'count', g.FLOOR['reset'], 'count'),
        {'id': 'G-COV-cap', 'resolution': {'value': 1.0 / rows_min, 'unit': 'fraction of rows (one frame of the shortest bundle, %d rows)' % rows_min},
         'tolerance': {'value': g.COV_LOSS_CAP, 'unit': 'fraction of rows (CLAUDE.md section 3: 1 percentage point of tracked rows); per bundle max(1 %, 3 R_D[cov] / rows), vacuous when it reaches the largest possible loss'}},
    ]
    m['resolution'] = {'metric': 'see tiers', 'value': None, 'unit': 'see tiers',
                       'method': 'null-runs NR-1..NR-7 (gate section 6): identical reruns, harness equivalence, noise R (id and RNG seeds), same-trace floor, D1, original-side tool null'}
    m['tolerance'] = {'value': None, 'unit': 'see tiers; band tolerance per bundle and metric = max(3 R[b][m], floor[m]) with R from noise_table'}
    nt = g.sha256_file(g.NOISE_TABLE)
    nl, nn = g.trace_list_sha(g.OR4V3 / 'null/N')
    dl, dn = g.trace_list_sha(*g.d_roots())
    m['input_pins']['noise_table'] = nt
    m['input_pins']['n_arm_traces'] = nl
    m['input_pins']['null_report'] = g.sha256_file(rep_path)
    m['input_pins']['d_arm_traces'] = dl
    m['noise_table'] = {'file': str(g.NOISE_TABLE.relative_to(g.ROOT)), 'sha256': nt, 'n_arm_trace_list_sha256': nl, 'n_arm_traces': nn, 'd_arm_trace_list_sha256': dl, 'd_arm_traces': dn,
                        'note': 'R_N and R_D only from no-target runs (N arm, D arm): 4 id seeds + 3 RNG seeds each; the GATE tolerance uses R_D; fixed by this file before any T run'}
    m['null_run'] = {'evidence': str(rep_path.relative_to(g.ROOT)), 'result': 'PASS', 'run_at': rep['created'],
                     'detail': 'G-DET-N 14/14 identical, G-H 14/14 identical, G-DET-D 14/14 identical, noise tables R_N and R_D (7 runs each), same-trace metric floor (NR-4), D vs N inherited effect, required mutations on the D arm behaved as registered, G-EXERCISED negative control (N and D runs fail the positive indicator), G-COV cap table, leave-one-out false-FAIL estimate for the powered bundles; '
                               'original-side null: or4/v3/null/original-null.json (reused)',
                     'sha256': g.sha256_file(rep_path)}
    m['mutation'] = {'description': 'engine (D arm; bundles chosen by the registered rule: first three with zero R_D): drop-every:7, orient-beta:3.0, orient-drift:0.5 (required); '
                                    'trace-level mutants of the D s0 trace on 14 bundles (tracked rows only: scale x0.9/1.1/1.2/1.5, pos 10 %, rot 2 deg, norm every10, tracked-cov every20, reset 5) '
                                    'with negative controls (scale x1.02, scale x1.5 on INITIALIZING rows only, pos 3 %, rot 0.5 deg, norm 2, cov 2, reset 2); detection iff the moved metric exceeds the bundle tolerance; '
                                    'G-EXERCISED positive-indicator negative control on 28 real no-target runs; identical() mutants: 1 ulp, deleted row, other input (unit tests)',
                     'evidence': str(rep_path.relative_to(g.ROOT)), 'detected': bool(rep['mutations-required-ok']),
                     'sub_resolution_detected': True, 'sub_resolution_mutation': 'one float of a trace copy moved by one ulp (test_identical_ignores_timing_but_sees_one_ulp)',
                     'larger_mutations_detected': True,
                     'unit_test_log': str((g.OR4V / 'mutation/unit-tests.log').relative_to(g.ROOT)),
                     'summary': {k: {'detected': v['detected'], 'failing_metrics': v['failing_metrics'], 'required': v['required'],
                                     'expected_detected': v.get('expected_detected'), 'ok': v['ok']} for k, v in rep['mutations'].items()}}
    m['evidence_files'] = [str(p.relative_to(g.ROOT)) for p in (g.OR4V3 / 'null/original-null.json', g.OR4V / 'mutation/unit-tests.log') if p.exists()]
    m['tree_divergence'] = {'recorded_at_pin': str(g.SOURCE_STATE.relative_to(g.ROOT)), 'now': g.tree_divergence(),
                            'note': 'files changed or added in the engine source tree since the binaries were pinned (v3); comment-only edits of this lane: '
                                    'replay_diagnostics.h, or4_replay_main.cc (version labels); the results apply to the pinned binary sha256 only'}
    m['seen_before_registration'] = ('gate v6 section 0.4: everything up to and including the v5 null-report (R_N, R_D, 112 + 112 traces, D vs N, powered 6, mutations) and the v5 review calculations (null simulation, leave-one-out, tracked counts); '
                                     'the declared dev runs on tum-room1-lowtex__pmr3b; the original smoke runs and r1~r2 pair metrics')
    m['not_seen'] = 'the v6 null-report values (cov cap table, vacuous table with row-count maxima, leave-one-out estimate, G-EXERCISED negative control); the T arm of every bundle except the dev run; original target / no-target arms'
    m['timeline_honesty'] = ('v6 created_at precedes null-report.json (v6), the manifest evidence fields and unit-tests.log of v6. No new engine run: the N arm (v3), D arm (v3 force, v5 98 runs), D-arm mutation runs (v5) and the pinned binaries are reused. '
                             'A dry run of the v6 analyze-null into a scratch directory happened before the v6 md was written to check the code path; its output was not inspected.')
    m['post_registration_observation'] = {
        'powered_bundles_R_D': rep['powered_bundles'],
        'mutation_bundles': rep['mutation_bundles'],
        'expected_TN_exceed_consistent_with_D_vs_N': rep['expected_TN_exceed_consistent_with_D_vs_N'],
        'D_vs_N_exceeding_bundles_v5_metrics': sorted(k for k, v in rep['D_vs_N_inherited'].items() if v['exceeds_N_tolerance']),
        'note': 'REPORT-only comparison of the declared T-vs-N expectation (gate v6 section 0.3, 11 bundles) with the computed D-vs-N exceedance; R-US014 uses the powered bundles of the power table.',
        'loo_false_fail_rate_powered': rep['loo_false_fail_rate_powered'], 'loo_bundle_fail_probability_any_powered': rep['loo_bundle_fail_probability_any_powered'],
        'exercised_negative_control_ok': rep['exercised_negative_control_ok']}
    m['finalized_at'] = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    g.MANIFEST.write_text(json.dumps(m, indent=1) + '\n')
    print('manifest evidence fields written', m['finalized_at'])


if __name__ == '__main__':
    main()
