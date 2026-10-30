#!/usr/bin/env python3
"""Per-stage engine timing of replay runs (architecture.md §6.4 performance; repo CLAUDE.md §3).

Reads the `srStageMs` of every processed (non-repeated) row of engine traces written by
tools/harness/replay and reports p50/p95/p99 per stage, split into before / after map
initialization (first row with srKeyframes > 0), plus processed / dropped / repeated frame counts
and MAP_RESET count. Percentiles are nearest-rank. These are DESKTOP NATIVE single-thread wall
times of the replay process, not phone measurements; `glue` is the 733/704 post-processing only
(capture, copy and render stages of the browser do not exist in the replay).

Usage (repo root):
  python3 -I slam_refact/tools/harness/eval/stage_timing.py --run NAME=RUN_DIR ... --out-json <new file>
"""
import argparse
import json
import math
import pathlib

STAGES = ('frontend', 'mapTracker', 'mapBuilder', 'metricScale', 'glue', 'total')


def nearest_rank(values, p):
    if not values:
        return None
    s = sorted(values)
    return s[max(0, math.ceil(p / 100.0 * len(s)) - 1)]


def summarize(rows):
    out = {}
    for stage in STAGES:
        v = [r[stage] for r in rows]
        out[stage] = {'n': len(v), 'p50': nearest_rank(v, 50), 'p95': nearest_rank(v, 95),
                      'p99': nearest_rank(v, 99), 'max': max(v) if v else None}
    return out


def run_stats(run_dir):
    rows = [json.loads(line) for line in (pathlib.Path(run_dir) / 'trace.jsonl').read_text().splitlines() if line]
    processed = [r for r in rows if r['fresh'] and not r.get('srRepeated') and r.get('srStageMs')]
    init_i = next((r['i'] for r in processed if (r.get('srKeyframes') or 0) > 0), None)
    timed = []
    for r in processed:
        ms = dict(r['srStageMs'])
        ms['total'] = sum(ms[k] for k in STAGES[:-1])
        ms['post_init'] = init_i is not None and r['i'] >= init_i
        timed.append(ms)
    return {
        'rows': len(rows), 'processed_frames': len(processed),
        'dropped_frames': sum(1 for r in rows if not r['fresh']),
        'repeated_frames': sum(1 for r in rows if r.get('srRepeated')),
        'map_resets': sum(1 for r in processed if r.get('srMapReset')),
        'map_init_frame': init_i,
        'pre_init': summarize([t for t in timed if not t['post_init']]),
        'post_init': summarize([t for t in timed if t['post_init']]),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', action='append', required=True, help='NAME=RUN_DIR')
    ap.add_argument('--out-json', type=pathlib.Path, required=True)
    a = ap.parse_args()
    if a.out_json.exists():
        raise SystemExit('refusing to overwrite ' + str(a.out_json))
    report = {'schema': 'slam-refact-stage-timing-v1',
              'note': 'desktop native replay (single thread), not a phone; nearest-rank percentiles; ms',
              'runs': {}}
    for item in a.run:
        name, _, run_dir = item.partition('=')
        report['runs'][name] = dict(run_stats(run_dir), run=run_dir)
    a.out_json.write_text(json.dumps(report, indent=1) + '\n')
    for name, r in report['runs'].items():
        post = r['post_init']['total']
        pre = r['pre_init']['total']
        print('%-26s processed=%d dropped=%d resets=%d pre p50/p95/p99=%s/%s/%s post p50/p95/p99=%s/%s/%s' % (
            name, r['processed_frames'], r['dropped_frames'], r['map_resets'],
            *(('%.1f' % pre[k]) if pre[k] is not None else '-' for k in ('p50', 'p95', 'p99')),
            *(('%.1f' % post[k]) if post[k] is not None else '-' for k in ('p50', 'p95', 'p99'))))


if __name__ == '__main__':
    main()
