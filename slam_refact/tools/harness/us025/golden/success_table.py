#!/usr/bin/env python3
"""US-025 LC-G: table of every loop-close success (outLcOk == 1) in the given runs, with what the original did next (G-H, U3).

One row per success event: run, seq, stage frame (recorded-inputs when the run has it), slots/live, newest id, self candidate?, correction translation / rotation angle,
whether the SAME processStagedFrame call left through an exception (header.throws seq == event seq: `abort()` from getPoint 634, or a `memory access out of bounds`
trap in func 936), the number of unlink-loop iterations seen (hook runs), and the node pointer the loop followed after the first erase (hook runs).
usage: success_table.py --run <name>=<run dir> [...] --out <new json>"""
import argparse
import gzip
import json
import math
import pathlib
import struct


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', action='append', required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    rows = []
    for spec in a.run:
        name, d = spec.split('=', 1)
        d = pathlib.Path(d)
        lg = d / 'lcgold'
        header = json.loads((lg / 'header.json').read_text())
        blobs = (lg / 'blobs.bin').read_bytes()
        rd = lambda r: blobs[r['off']:r['off'] + r['len']]
        events = [json.loads(l) for l in (lg / 'events.jsonl').read_text().splitlines() if l]
        hooks = [json.loads(l) for l in (lg / 'hooks.jsonl').read_text().splitlines() if l] if (lg / 'hooks.jsonl').exists() else []
        stage = {}
        if (d / 'recorded-inputs.jsonl.gz').exists():
            for x in (json.loads(l) for l in gzip.open(d / 'recorded-inputs.jsonl.gz', 'rt')):
                if x.get('fn') == 'processStagedFrame':
                    stage[x['probeSeq']] = x['i']
        throws = {t['seq']: t for t in (header.get('throws') or [])}
        for ev in events:
            if not ev.get('outLcOk'):
                continue
            C = struct.unpack('<16f', rd(ev['outLcMat'])[:64])
            tr = math.sqrt(sum(v * v for v in C[12:15]))
            cos = max(-1.0, min(1.0, (C[0] + C[5] + C[10] - 1.0) / 2.0))
            row = {'run': name, 'seq': ev['seq'], 'stageFrame': stage.get(ev['seq']), 'nSlots': ev['nSlots'], 'nLive': ev['nLive'], 'newest': ev['newest'],
                   'candidateId': ev['ringPost'][2], 'selfCandidate': ev['ringPost'][2] == ev['newest'][1],
                   'correctionTranslation_m': tr, 'correctionAngle_rad': math.acos(cos), 'wpiSizePostCall': ev['post']['newest']['wpiSize']}
            t = throws.get(ev['seq'])
            if 'throws' not in header:   # run made before the wrapper logged exceptions: use the run-level error records (events.json errors, meta.json console errors)
                errs = []
                if (d / 'events.json').exists():
                    errs += [e.get('message', '')[:100] for e in json.loads((d / 'events.json').read_text()).get('errors', [])]
                errs += [c['text'][:100] for c in json.loads((d / 'meta.json').read_text()).get('console', []) if c.get('type') == 'error']
                row['runLevelErrors(no per-call exception log)'] = errs
            row['exceptionInSameCall'] = None if t is None else {'message': t['message'], 'topWasmFrames': t['stack'][:3]}
            its = [h for h in hooks if h['seq'] == ev['seq'] and h['id'] == 'unlink-iter']
            pre = [h for h in hooks if h['seq'] == ev['seq'] and h['id'] == 'unlink-pre']
            post = [h for h in hooks if h['seq'] == ev['seq'] and h['id'] == 'unlink-post']
            if its or pre:
                row['unlinkIterationsSeen'] = len(its)
                row['erases'] = [{'node': p['node'], 'k': p['k'], 'rightBefore': p['nodeWords'][1], 'rightAfter': q['right'], 'firstWordAfter': q['nodeWords'][0]} for p, q in zip(pre, post)]
                row['lastIterationNodeInMemory'] = its[-1]['nodeInMem'] if its else None
            h1146 = [h for h in hooks if h['seq'] == ev['seq'] and h['id'] == '1146-out']
            if h1146:
                row['pnpReturn'] = h1146[0]['ret']
            h1128 = [h for h in hooks if h['seq'] == ev['seq'] and h['id'] == '1128-out']
            if h1128:
                row['matches'] = h1128[0]['nMatches']
            h1146i = [h for h in hooks if h['seq'] == ev['seq'] and h['id'] == '1146-in']
            if h1146i:
                row['correspondences'] = h1146i[0]['nObs']
            rows.append(row)
    summary = {'successes': len(rows), 'selfCandidate': sum(r['selfCandidate'] for r in rows),
               'exceptionInSameCall(logged per call)': sum(1 for r in rows if r['exceptionInSameCall']),
               'runs_without_per_call_log_but_with_run_level_error': sum(1 for r in rows if r.get('runLevelErrors(no per-call exception log)')),
               'exceptionKinds': {}}
    for r in rows:
        k = r['exceptionInSameCall']['message'][:40] if r['exceptionInSameCall'] else 'none'
        summary['exceptionKinds'][k] = summary['exceptionKinds'].get(k, 0) + 1
    a.out.write_text(json.dumps({'summary': summary, 'rows': rows}, indent=1) + '\n')
    print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main()
