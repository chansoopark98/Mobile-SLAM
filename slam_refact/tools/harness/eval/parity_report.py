#!/usr/bin/env python3
"""Parity report generator (architecture.md §6.1 eval/parity_report.py, US-011; WP5).

Evaluates every gate of thresholds-v1.json and writes parity.json plus the markdown skeleton of
slam_refact/docs/reports/parity.md, grouped by gate class:
  * trace gates: computed here from engine runs written by tools/harness/replay (gates.py);
  * module gates (evaluator module-report / command / manual / isolation): verdicts come from
    --module-results, a JSON object {gate_id: {"verdict": ..., "evidence": ..., "command": ...}}
    assembled from the owning WP's GTest / command output; missing entries are NOT-RUN.
Class rules (D-16): BLOCKED-ALLOWED->X counts as X once executed; BLOCKED is only accepted with a
reason; MUST-PASS completion requires PASS (predeclared BLOCKED reasons only for V-H7, B-5).

The registered sha256 of thresholds-v1.json is checked first; a modified file stops the report.

Usage (repo root):
  python3 -I slam_refact/tools/harness/eval/parity_report.py \
      --run tum-room1-full=build/slam-refact-20261015/engine-runs/tum-room1-full/<label> ... \
      --injected flip6-s2=<run dir> --absolute flip6-s2-absolute=<run dir> \
      --module-results <json> --out-json <new file> --out-md <new file> \
      [--memprobe flip6-s2=<probe run 1>,<probe run 2> ...]

--memprobe SCENARIO=DIR1,DIR2 (two run-reference.mjs --memprobe runs of the original) lifts the registered blocked_if
of G-M3(b), G-M4, G-M5 (that scenario) and G-M7 (flip6-s2): those runs replace r1/r2 as the reference for these
gates only; the tolerance rules are unchanged (memprobe.py). Without --memprobe the output is as before.
"""
import argparse
import datetime
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import gates as gates_mod  # noqa: E402
import memprobe as mp  # noqa: E402
from common import THRESHOLDS_PATH, sha256_file  # noqa: E402

REGISTERED_SHA256 = '9a616652493e4b4d6105d8e5e3b91e6a0605cc003f94802683804cd14b337620'
CLASS_ORDER = ('MUST-PASS', 'BLOCKED-ALLOWED->MUST-PASS', 'REPORT', 'BLOCKED-ALLOWED->REPORT')


def load_thresholds(path=THRESHOLDS_PATH, expected=REGISTERED_SHA256):
    digest = sha256_file(path)
    if digest != expected:
        raise SystemExit('thresholds file %s sha256 %s != registered %s (see docs/reports/thresholds-registration.md)'
                         % (path, digest, expected))
    return json.loads(pathlib.Path(path).read_text()), digest


def parse_pairs(items):
    out = {}
    for item in items or []:
        key, _, value = item.partition('=')
        if not value:
            raise SystemExit('expected SCENARIO=DIR, got ' + item)
        out[key] = value
    return out


def module_results(thresholds, supplied, skip=()):
    out = []
    for gate in thresholds['gates']:
        if gate['id'] in gates_mod.TRACE_EVALUATORS or gate['id'] in skip:
            continue
        got = supplied.get(gate['id'])
        if got is None:
            out.append(gates_mod.result(gate, None, 'NOT-RUN', note='no verdict supplied (--module-results)'))
            continue
        verdict = got.get('verdict')
        if verdict not in ('PASS', 'FAIL', 'BLOCKED', 'REPORTED'):
            raise SystemExit('module result %s: verdict %r' % (gate['id'], verdict))
        if verdict == 'BLOCKED' and not got.get('reason'):
            raise SystemExit('module result %s: BLOCKED needs a reason' % gate['id'])
        r = gates_mod.result(gate, None, verdict, got.get('evidence'), gate['tolerance'], note=got.get('reason'))
        r['command'] = got.get('command')
        out.append(r)
    return out


def effective_class(r):
    """D-16: BLOCKED-ALLOWED->X is X once executed."""
    cls = r['class']
    if cls.startswith('BLOCKED-ALLOWED->') and r['verdict'] not in ('BLOCKED', 'NOT-RUN'):
        return cls.split('->', 1)[1]
    return cls


def completion(results, thresholds):
    by_id = {g['id']: g for g in thresholds['gates']}
    must = [r for r in results if effective_class(r) == 'MUST-PASS']
    blocking = []
    for r in must:
        if r['verdict'] == 'PASS':
            continue
        if r['verdict'] == 'BLOCKED' and by_id[r['id']].get('predeclared_blocked_reason'):
            continue
        blocking.append({'id': r['id'], 'scenario': r['scenario'], 'verdict': r['verdict']})
    unevaluated = [{'id': r['id'], 'scenario': r['scenario']} for r in results
                   if effective_class(r) == 'REPORT' and r['verdict'] == 'NOT-RUN']
    return {'must_pass_blocking': blocking, 'report_not_evaluated': unevaluated,
            'complete': not blocking and not unevaluated}


def fmt_measured(r):
    m = r.get('measured')
    if m is None:
        return r.get('note') or ''
    text = json.dumps(gates_mod.json_safe(m), separators=(',', ':'))
    return text if len(text) <= 160 else text[:157] + '...'


def markdown(report):
    lines = ['# slam_refact parity 보고서', '',
             '- 생성: `%s` (%s)' % (' '.join(report['argv']), report['generated_at']),
             '- 기준: `slam_refact/tools/harness/thresholds-v1.json` sha256 `%s` (등록값과 일치 확인)' % report['thresholds_sha256'],
             '- 표기: VERIFIED(실행 관측) / BLOCKED(사유) / NOT-RUN(입력 없음). trace gate는 GlueRunner 출력, module gate는 '
             '해당 WP의 GTest·명령 결과입니다. 둘을 섞어 판정하지 않습니다(P-4).',
             '- 한계: headless desktop 원본(SwiftShader) 대비 parity입니다. 실기기 성능·정확도가 아닙니다.', '',
             '## 완료 판정 (D-16)', '',
             '- MUST-PASS 미충족: %s' % (', '.join('%s/%s=%s' % (b['id'], b['scenario'], b['verdict'])
                                         for b in report['completion']['must_pass_blocking']) or '없음'),
             '- REPORT 미평가: %d건' % len(report['completion']['report_not_evaluated']),
             '- 완료: **%s**' % ('예' if report['completion']['complete'] else '아니오'), '']
    for cls in CLASS_ORDER:
        rows = [r for r in report['results'] if r['class'] == cls]
        if not rows:
            continue
        lines += ['## %s' % cls, '', '| gate | 시나리오 | 판정 | 측정 | 태그 |', '|---|---|---|---|---|']
        for r in rows:
            lines.append('| %s | %s | %s | `%s` | %s |' % (r['id'], r['scenario'] or '-', r['verdict'],
                                                         fmt_measured(r).replace('|', '\\|'), ', '.join(r.get('tags') or [])))
        lines.append('')
    lines += ['## 입력', '', '```json', json.dumps(report['inputs'], indent=1), '```', '']
    return '\n'.join(lines)


def build_report(thresholds, digest, runs, injected, absolute, supplied, argv, memprobe=None):
    memprobe = memprobe or {}
    ctx = gates_mod.Context(thresholds, runs, injected, absolute, memprobe)
    probe_results = gates_mod.evaluate_memprobe_gates(ctx)
    results = (gates_mod.evaluate_trace_gates(ctx) + probe_results
               + module_results(thresholds, supplied, skip={r['id'] for r in probe_results}))
    results = [gates_mod.json_safe(r) for r in results]
    inputs = {'runs': runs, 'injected': injected, 'absolute': absolute, 'module_results': sorted(supplied)}
    if memprobe:
        inputs['memprobe'] = {sc: [str(d) for d in p.dirs] for sc, p in memprobe.items()}
    return {'schema': 'slam-refact-parity-report-v1', 'generated_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'argv': argv, 'thresholds_sha256': digest,
            'inputs': inputs,
            'results': results, 'completion': completion(results, thresholds)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', action='append', help='SCENARIO=DIR engine parity run (drop-mask r1, SwiftShader model)')
    ap.add_argument('--injected', action='append', help='SCENARIO=DIR engine run with injected captured pyramids (V-H7, V-H7b)')
    ap.add_argument('--absolute', action='append', help='SCENARIO-absolute=DIR engine run with estimate_scale (MS-RT)')
    ap.add_argument('--module-results', type=pathlib.Path)
    ap.add_argument('--memprobe', action='append', help='SCENARIO=DIR1,DIR2: two original memory-probe runs (G-M3b/G-M4/G-M5/G-M7)')
    ap.add_argument('--out-json', type=pathlib.Path, required=True)
    ap.add_argument('--out-md', type=pathlib.Path)
    a = ap.parse_args()
    for p in (a.out_json, a.out_md):
        if p is not None and p.exists():
            raise SystemExit('refusing to overwrite ' + str(p))
    thresholds, digest = load_thresholds()
    supplied = json.loads(a.module_results.read_text()) if a.module_results else {}
    report = build_report(thresholds, digest, parse_pairs(a.run), parse_pairs(a.injected), parse_pairs(a.absolute), supplied, sys.argv,
                          mp.parse_memprobe_args(a.memprobe))
    a.out_json.write_text(json.dumps(report, indent=1) + '\n')
    if a.out_md:
        a.out_md.write_text(markdown(report))
    c = report['completion']
    print(json.dumps({'results': len(report['results']), 'must_pass_blocking': len(c['must_pass_blocking']),
                      'report_not_evaluated': len(c['report_not_evaluated']), 'complete': c['complete']}))


if __name__ == '__main__':
    main()
