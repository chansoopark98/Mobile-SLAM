#!/usr/bin/env python3
"""A/B report of the original xr-slam.js chunk vs the slam_refact chunk (spec docs/spec/07-xr-slam-chunk.md §4).

Reads one run-chunk-ab.mjs label directory (<label>/original, <label>/refact) and evaluates the criteria that were
fixed before any run: AB-I (intrinsics), AB-T (first translation), AB-S (status agreement), AB-R (NORMAL ratio), and
per run C7-2 (load/start), C8-2 (b) (response lag, refact), C9-2 (lighting / hitTest rules, when api-oracle ran) and
C9-3 (trackingstatus events = status changes). Tolerances are constants here and in spec 07; they are not tuned.

  python3 -I slam_refact/tools/re/runtime/chunk_ab_report.py --run-dir <label dir> --scenario flip6-s2 \
      --out-json <new file> --out-md <new file> [--truncated]
"""
import argparse
import gzip
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'slam_refact/tools/harness/eval'))

import frame_parity  # noqa: E402
import trace_metrics as tm  # noqa: E402

INTRINSICS_TOL = 1e-6        # AB-I
FIRST_TRANSLATION_TOL = 5    # AB-T (V-EV1 window width)
NORMAL_RATIO_TOL = 0.05      # AB-R (REPORT)
THRESHOLDS = ROOT / 'slam_refact/tools/harness/thresholds-v1.json'


def engine_bytes_unreplaced(engine):
    """C7-2: the engine bytes equal the manifest AND the chunk reported that no test hook replaced the glue.
    Fail-closed (RM-4c m-F): a missing or non-boolean `glueReplaced` is not a pass."""
    engine = engine or {}
    return engine.get('matchesManifest') is True and engine.get('glueReplaced') is False


def load_run(d):
    rows = [json.loads(line) for line in (d / 'trace.jsonl').read_text().splitlines() if line]
    callbacks = [json.loads(line) for line in (d / 'callbacks.jsonl').read_text().splitlines() if line]
    events = json.loads((d / 'events.json').read_text())
    meta = json.loads((d / 'meta.json').read_text())
    oracle_path = d / 'api-oracle.json.gz'
    oracle = json.loads(gzip.decompress(oracle_path.read_bytes())) if oracle_path.exists() else None
    return {'rows': rows, 'callbacks': callbacks, 'events': events, 'meta': meta, 'oracle': oracle}


def tracking_events_check(run):
    """C9-3: reality.trackingstatus events equal the (status, reason) changes of the non-null callbacks."""
    changes, last = [], ('UNSPECIFIED', 'UNSPECIFIED')
    for c in run['callbacks']:
        if c['status'] is None:
            continue
        cur = (c['status'], c['reason'])
        if cur != last:
            changes.append(list(cur))
            last = cur
    events = [[e['detail']['status'], e['detail']['reason']] for e in run['events']['events'] if e['name'] == 'reality.trackingstatus']
    return {'pass': events == changes, 'events': len(events), 'changes': len(changes), 'sequence': events[:20]}


def is_zero(v):
    return v in ('-0', 0, 0.0)


def oracle_check(run):
    """C9-2: lighting temperature 6500 on fresh callbacks; hitTest type rules where the callback has world points."""
    o = run['oracle']
    if not o:
        return None
    temps = [r['lighting']['temperature'] if r['lighting'] else None for r in o['lighting']]
    lighting_ok = bool(temps) and all(t == 6500 for t in temps)
    violations, samples, fp_hits = [], 0, 0
    for h in o['hits']:
        if not h['worldPoints']:
            continue
        for q in h['queries']:
            samples += 1
            types, hits = q['types'], q['hits']
            surfaces_only = bool(types) and 'FEATURE_POINT' not in types
            want = 0 if surfaces_only else 1
            ok = len(hits) == want and all(x['type'] == 'FEATURE_POINT' for x in hits)
            ok = ok and all(all(is_zero(x['rotation'][k]) for k in 'wxyz') for x in hits)
            fp_hits += sum(x['type'] == 'FEATURE_POINT' for x in hits)
            if not ok and len(violations) < 10:
                violations.append({'fresh': h['fresh'], 'x': q['x'], 'y': q['y'], 'types': types, 'hits': hits})
    rot_signs = sorted({json.dumps(x['rotation'], sort_keys=True) for h in o['hits'] for q in h['queries'] for x in q['hits']})
    return {'lighting_rows': len(temps), 'lighting_temperature_6500': lighting_ok,
            'lighting_exposure_range': [min((r['lighting']['exposure'] for r in o['lighting'] if r['lighting']), default=None),
                                        max((r['lighting']['exposure'] for r in o['lighting'] if r['lighting']), default=None)],
            'hit_samples_with_points': samples, 'feature_point_hits': fp_hits, 'hit_rule_violations': violations,
            'hit_rules_pass': samples > 0 and not violations, 'hit_rotation_values': rot_signs[:4]}


def run_summary(run):
    rows = run['rows']
    fresh = tm.fresh_rows(rows) if hasattr(tm, 'fresh_rows') else [r for r in rows if r['fresh']]
    m = run['meta']
    s = m.get('summary', {})
    return {'status': m.get('status'), 'input_frames': len(rows), 'fresh_frames': len(fresh),
            'fresh_ratio': len(fresh) / len(rows), 'first_translation_i': tm.first_translation(rows),
            'normal_ratio': sum(r.get('status') == 'NORMAL' for r in rows) / len(rows),
            'limited_ratio': tm.limited_ratio(rows), 'world_points_p50': tm.world_points_p50(rows) if fresh else None,
            'publish_to_callback_ms': s.get('publishToCallbackMs'), 'xr_cpu_stage_ms': s.get('xrCpuStageMs'),
            'status_counts': s.get('statusCounts'),
            'load_start': {'chunk_requested': s.get('chunkRequested'), 'on_start': bool(s.get('onStart')),
                           'non_null_reality': s.get('nonNullReality'), 'page_errors': s.get('pageErrors'),
                           'probe_errors': s.get('errors'), 'blocked_external': s.get('blockedExternal'),
                           'served_xr_js_sha256': m.get('served', {}).get('xr.js', {}).get('sha256')},
            'trackingstatus_events': tracking_events_check(run), 'api': oracle_check(run)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--out-json', required=True)
    ap.add_argument('--out-md', required=True)
    ap.add_argument('--truncated', action='store_true', help='--max-frames run: AB-S is REPORT only')
    a = ap.parse_args()
    d = pathlib.Path(a.run_dir).resolve()
    for out in (a.out_json, a.out_md):
        if pathlib.Path(out).exists():
            sys.exit(f'{out} exists (new file per report)')
    orig, ref = load_run(d / 'original'), load_run(d / 'refact')
    so, sr = run_summary(orig), run_summary(ref)
    thresholds = json.loads(THRESHOLDS.read_text())
    gates = thresholds['gates'] if isinstance(thresholds, dict) else thresholds
    ev3 = next(g for g in gates if g['id'] == 'V-EV3')['tolerance']['min_agreement'].get(a.scenario)

    io = orig['events']['intrinsicsChanges']
    ir = ref['events']['intrinsicsChanges']
    max_diff = None
    if io and len(io) == len(ir):
        max_diff = max(abs(x - y) for co, cr in zip(io, ir) for x, y in zip(co['intrinsics'], cr['intrinsics']))
    ab_i = {'original_entries': len(io), 'refact_entries': len(ir), 'max_abs_diff': max_diff,
            'original_first': io[0]['intrinsics'] if io else None, 'refact_first': ir[0]['intrinsics'] if ir else None,
            'pass': max_diff is not None and max_diff <= INTRINSICS_TOL}
    fo, fr = so['first_translation_i'], sr['first_translation_i']
    ab_t = {'original': fo, 'refact': fr, 'pass': fo is not None and fr is not None and abs(fo - fr) <= FIRST_TRANSLATION_TOL}
    agree = frame_parity.status_agreement(ref['rows'], orig['rows'])
    ab_s = {**agree, 'min_agreement': ev3, 'class': 'REPORT' if a.truncated or ev3 is None else 'GATE',
            'pass': agree['agreement'] is not None and ev3 is not None and agree['agreement'] >= ev3}
    ab_r = {'original': so['normal_ratio'], 'refact': sr['normal_ratio'], 'abs_diff': abs(so['normal_ratio'] - sr['normal_ratio']),
            'pass': abs(so['normal_ratio'] - sr['normal_ratio']) <= NORMAL_RATIO_TOL, 'class': 'REPORT'}
    chunk_diag = ref['meta'].get('chunkDiag') or {}
    lag = chunk_diag.get('responseLag') or {}
    c8b = {'response_lag': lag, 'pass': bool(lag) and set(lag) == {'1'}}
    lo = so['load_start']
    lr = sr['load_start']
    c72 = {'pass': bool(lr['chunk_requested'] and lr['on_start'] and lr['non_null_reality'] and lr['page_errors'] == 0
                        and lr['probe_errors'] == 0 and lr['blocked_external'] == 0
                        and engine_bytes_unreplaced(chunk_diag.get('engine'))),
           'refact': lr, 'original': lo, 'engine': chunk_diag.get('engine')}
    report = {'schema': 'slam-refact-chunk-ab-report-v1', 'scenario': a.scenario, 'run_dir': str(d.relative_to(ROOT)),
              'tolerances': {'AB-I': INTRINSICS_TOL, 'AB-T': FIRST_TRANSLATION_TOL, 'AB-S': ev3, 'AB-R': NORMAL_RATIO_TOL},
              'AB-I': ab_i, 'AB-T': ab_t, 'AB-S': ab_s, 'AB-R': ab_r, 'C7-2': c72, 'C8-2b': c8b,
              'C9-2': {'original': so['api'], 'refact': sr['api']},
              'C9-3': {'original': so['trackingstatus_events'], 'refact': sr['trackingstatus_events']},
              'runs': {'original': so, 'refact': sr}, 'chunk_diag': chunk_diag}
    pathlib.Path(a.out_json).write_text(json.dumps(report, indent=1) + '\n')

    def v(x):
        return 'PASS' if x else 'FAIL'

    def f(x, n=4):
        return 'null' if x is None else (f'{x:.{n}f}' if isinstance(x, float) and not math.isnan(x) else str(x))

    c92o, c92r = so['api'] or {}, sr['api'] or {}
    lines = [
        f'# xr-slam chunk A/B: {a.scenario}', '',
        f'- run: `{report["run_dir"]}` (original = deployed `xr-slam.js`, refact = `slam_refact/web-chunk/xr-slam.js`; same `xr.js`, same `page-runtime.js` input, sequential headless SwiftShader runs)',
        '- criteria and tolerances: `slam_refact/docs/spec/07-xr-slam-chunk.md` §4 (fixed before the runs). Desktop headless numbers are not phone performance.', '',
        '| criterion | original | refact | tolerance | verdict |', '|---|---|---|---|---|',
        f'| AB-I intrinsics (entries, max abs diff) | {ab_i["original_entries"]} | {ab_i["refact_entries"]} | <= {INTRINSICS_TOL} (diff {f(max_diff, 9)}) | {v(ab_i["pass"])} |',
        f'| AB-T first translation frame | {fo} | {fr} | +-{FIRST_TRANSLATION_TOL} | {v(ab_t["pass"])} |',
        f'| AB-S status agreement (both fresh {agree["both_fresh"]}) | | {f(agree["agreement"])} | >= {ev3} | {v(ab_s["pass"])} ({ab_s["class"]}) |',
        f'| AB-R NORMAL ratio | {f(so["normal_ratio"])} | {f(sr["normal_ratio"])} | abs diff <= {NORMAL_RATIO_TOL} | {v(ab_r["pass"])} (REPORT) |',
        f'| fresh frame ratio | {f(so["fresh_ratio"])} | {f(sr["fresh_ratio"])} | record | - |',
        f'| limited ratio | {f(so["limited_ratio"])} | {f(sr["limited_ratio"])} | record | - |',
        f'| world points p50 | {so["world_points_p50"]} | {sr["world_points_p50"]} | record | - |',
        f'| publish->callback ms p50/p95/p99 | {f((so["publish_to_callback_ms"] or {}).get("p50"), 1)}/{f((so["publish_to_callback_ms"] or {}).get("p95"), 1)}/{f((so["publish_to_callback_ms"] or {}).get("p99"), 1)} | {f((sr["publish_to_callback_ms"] or {}).get("p50"), 1)}/{f((sr["publish_to_callback_ms"] or {}).get("p95"), 1)}/{f((sr["publish_to_callback_ms"] or {}).get("p99"), 1)} | record | - |',
        f'| C7-2 load + XR8.run start (refact) | | chunk {lr["chunk_requested"]}, onStart {lr["on_start"]}, reality {lr["non_null_reality"]}, errors {lr["page_errors"]}/{lr["probe_errors"]}, external {lr["blocked_external"]} | all | {v(c72["pass"])} |',
        f'| C8-2(b) response lag (stage - source) | | {json.dumps(lag)} | all 1 | {v(c8b["pass"])} |',
        f'| C9-2 lighting temperature 6500 | {c92o.get("lighting_temperature_6500")} ({c92o.get("lighting_rows")}) | {c92r.get("lighting_temperature_6500")} ({c92r.get("lighting_rows")}) | all fresh | {v(c92r.get("lighting_temperature_6500"))} |',
        f'| C9-2 hitTest rules (samples with points) | {c92o.get("hit_rules_pass")} ({c92o.get("hit_samples_with_points")}) | {c92r.get("hit_rules_pass")} ({c92r.get("hit_samples_with_points")}) | no violation | {v(c92r.get("hit_rules_pass"))} |',
        f'| C9-3 trackingstatus events = changes | {so["trackingstatus_events"]["pass"]} ({so["trackingstatus_events"]["events"]}) | {sr["trackingstatus_events"]["pass"]} ({sr["trackingstatus_events"]["events"]}) | equal | {v(sr["trackingstatus_events"]["pass"])} |',
        '', '## status counts', '', f'- original: `{json.dumps(so["status_counts"])}`', f'- refact: `{json.dumps(sr["status_counts"])}`',
        '', '## refact chunk stages (ms, desktop headless)', '',
        f'- stage (GPU draw + PBO readPixels submit): `{json.dumps(chunk_diag.get("stageMs"))}`',
        f'- read (fence + getBufferSubData): `{json.dumps(chunk_diag.get("readMs"))}`',
        f'- engine (processFramePyramid): `{json.dumps(chunk_diag.get("engineMs"))}`',
        f'- readback fence at read: `{json.dumps((chunk_diag.get("session") or {}).get("readback"))}`',
        f'- staged {chunk_diag.get("stagedFrames")}, processed {chunk_diag.get("processedFrames")}, fresh responses {chunk_diag.get("freshResponses")}, dropped staged {chunk_diag.get("droppedStaged")}, IMU events pushed {chunk_diag.get("motionEventsPushed")} / skipped {chunk_diag.get("motionEventsSkipped")}',
    ]
    pathlib.Path(a.out_md).write_text('\n'.join(lines) + '\n')
    print(json.dumps({k: report[k]['pass'] for k in ('AB-I', 'AB-T', 'AB-S', 'AB-R', 'C7-2', 'C8-2b')}))


if __name__ == '__main__':
    main()
