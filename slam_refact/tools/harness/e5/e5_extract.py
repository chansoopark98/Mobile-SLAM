#!/usr/bin/env python3
"""E5: per-frame image-target series of one run (original SDK or engine) in one common form.

series = {'n': N, 'processed': [bool], 'detected': [bool], 'pos': [[x,y,z]|None], 'quat': [[w,x,y,z]|None], 'scale': [float|None],
          'events': [(kind, frame)], 'pose': [[x,y,z]|None] (the camera position the run published), 'cam_q': the camera quaternion (w,x,y,z) it published}
Frames are the input frame indices of the sequence (= fixture frame index). A frame is 'processed' when the original delivered
a fresh callback for it / the engine processed it (the engine bundle uses the original's own fresh set as its drop mask).
Original: image-targets.json (events + detectedImagesRows, frame by videoTime) and trace.jsonl. Engine: images.jsonl and trace.jsonl.
The scale of the engine = height_in_meters (scaledHeight = 1 for a non-rotated target, web-chunk/image-targets.js); the original's
`scale` field is used as published.  Development tool, no product code.
"""
import json, math, sys

EVENT_NAMES = {'reality.imagefound': 'found', 'reality.imagelost': 'lost'}
ENGINE_KIND = {0: 'found', 1: 'updated', 2: 'lost'}   # ImageTargetEventKind order checked at run time against the header (see engine_kinds)


def _load_jsonl(path):
    with open(path) as f:
        return [json.loads(l) for l in f if l.strip()]


def original(run_dir):
    it = json.load(open(run_dir + '/image-targets.json'))
    trace = _load_jsonl(run_dir + '/trace.jsonl')
    n = max(r['i'] for r in trace) + 1
    s = {'n': n, 'processed': [False] * n, 'detected': [False] * n, 'pos': [None] * n, 'quat': [None] * n, 'scale': [None] * n,
         'pose': [None] * n, 'cam_q': [None] * n, 'events': []}
    for r in trace:
        s['processed'][r['i']] = bool(r['fresh'])
        if r['fresh'] and r.get('p'):
            s['pose'][r['i']] = r['p']
            s['cam_q'][r['i']] = r.get('q_wxyz')
    for row in it['detectedImagesRows']:
        i = row['frameIndex']
        if i is None or not row['detectedImages']:
            continue
        d = row['detectedImages'][0]
        s['detected'][i] = True
        s['pos'][i] = [d['position']['x'], d['position']['y'], d['position']['z']]
        s['quat'][i] = [d['rotation']['w'], d['rotation']['x'], d['rotation']['y'], d['rotation']['z']]
        s['scale'][i] = d['scale']
    for e in it['events']:
        k = EVENT_NAMES.get(e['name'])
        if k and e['frameIndex'] is not None:
            s['events'].append((k, e['frameIndex']))
    s['events'].sort(key=lambda x: x[1])
    return s


def engine(run_dir, kinds=None):
    kinds = kinds or ENGINE_KIND
    rows = _load_jsonl(run_dir + '/images.jsonl')
    trace = _load_jsonl(run_dir + '/trace.jsonl')
    n = max(r['i'] for r in trace) + 1 if trace else 0
    s = {'n': n, 'processed': [False] * n, 'detected': [False] * n, 'pos': [None] * n, 'quat': [None] * n, 'scale': [None] * n,
         'pose': [None] * n, 'cam_q': [None] * n, 'events': []}
    for r in trace:
        i = r['i']
        s['processed'][i] = bool(r.get('processed', r.get('fresh', True))) and not r.get('dropped', False)
        p = r.get('p')
        if p:
            s['pose'][i] = p
            s['cam_q'][i] = r.get('q_wxyz')
    for r in rows:
        i = r['i']
        s['processed'][i] = True
        if r['detected']:
            d = r['detected'][0]
            s['detected'][i] = True
            s['pos'][i] = d['p']; s['quat'][i] = d['q']; s['scale'][i] = d['h']
        for e in r['events']:
            k = kinds.get(e['kind'])
            if k in ('found', 'lost'):
                s['events'].append((k, i))
    s['events'].sort(key=lambda x: x[1])
    return s


if __name__ == '__main__':
    kind, path = sys.argv[1], sys.argv[2]
    s = original(path) if kind == 'orig' else engine(path)
    print(json.dumps({'n': s['n'], 'processed': sum(s['processed']), 'detected': sum(s['detected']), 'events': s['events']}))
