#!/usr/bin/env python3
"""US-025 LC-G (item 5): frame correspondence of one original lc=1 run for the exact-input replay (LC-6a).

For a run made with --record-inputs --memprobe --memprobe-detail N --lcgold --poke '{"lc":1}' (plain original wasm), this writes one JSON mapping every
processStagedFrame call (memprobe seq) to the input frame it was staged for (recorded-inputs `processStagedFrame.i` = i of the preceding stageFrame,
which is the bundle frame index of make_bundles.sh/make_variant.py), the loop-close attempt data (events.jsonl of the same run) and the aborts (header.aborts).
A frame staged twice (the SDK re-stages a frame after an abort) appears in `restaged`.
usage: make_correspondence.py --run <run dir> --bundle <exact bundle dir> --out <new json>"""
import argparse
import gzip
import json
import pathlib


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', type=pathlib.Path, required=True)
    ap.add_argument('--bundle', type=pathlib.Path)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    rec = [json.loads(l) for l in gzip.open(a.run / 'recorded-inputs.jsonl.gz', 'rt')][1:]
    calls = [x for x in rec if x.get('fn') == 'processStagedFrame']
    seq2i = {x['probeSeq']: x['i'] for x in calls}
    mp = [json.loads(l) for l in gzip.open(a.run / 'memprobe.jsonl.gz', 'rt')]
    fields = mp[0]['fields']
    mp = mp[1:]
    lg = a.run / 'lcgold'
    header = json.loads((lg / 'header.json').read_text())
    events = [json.loads(l) for l in (lg / 'events.jsonl').read_text().splitlines() if l]
    by_frame = {}
    for s, i in seq2i.items():
        by_frame.setdefault(i, []).append(s)
    restaged = {str(i): v for i, v in by_frame.items() if i is not None and len(v) > 1}
    attempts = []
    for ev in events:
        if 'skipped' in ev or not ev.get('lcEnabled') or ev['nSlots'] < 5:
            continue
        ring = ev['ringPost']
        attempts.append({'seq': ev['seq'], 'stageFrame': seq2i.get(ev['seq']), 'memprobeFrame': ev.get('i'), 'nSlots': ev['nSlots'], 'nLive': ev['nLive'], 'newest': ev['newest'],
                         'pickId': ring[2] if ring[2] else None, 'selfCandidate': bool(ring[2]) and ring[2] == ev['newest'][1], 'outLcOk': ev['outLcOk']})
    out = {'schema': 'slam-refact-us025-lc-correspondence-v1', 'run': str(a.run), 'bundle': str(a.bundle) if a.bundle else None, 'wasmSha256': header['wasmSha256'], 'hookMode': header['hookMode'],
           'calls': len(calls), 'memprobeRecords': len(mp), 'aborts': header.get('aborts'), 'restagedFrames(stageFrame -> seqs)': restaged,
           'outLcOkSeqs': [e['seq'] for e in events if e.get('outLcOk')], 'attempts': attempts,
           'note': 'bundle frame index == stageFrame; memprobeFrame (trace i) is null for a call without a fresh callback (aborted or repeated call).'}
    a.out.write_text(json.dumps(out, indent=1) + '\n')
    print(json.dumps({k: out[k] for k in ('calls', 'outLcOkSeqs', 'restagedFrames(stageFrame -> seqs)')}))


if __name__ == '__main__':
    main()
