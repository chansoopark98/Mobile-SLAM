#!/usr/bin/env python3
"""Turn a run-sky-reference.mjs record (result.json + binary dumps) into <record>/replay.txt, the
line-based input of sr_sky_replay and of the sky gate tests (no JSON parser in C++).

replay.txt (space separated, one record per line, '#' comments):
  meta <video_w> <video_h> <orientation> <canvas_w> <canvas_h> <processed_frames> <mirrored 0|1>
  config <invert 0|1> <edge_smoothness> <near|-1> <far|-1>
  frame <k> <cam_w> <cam_h> <cam file> <alpha> <beta> <gamma>
  event <staged> <name>                       (layerloading | layerscanning | layerfound <percentage>)
  invoke <k> <sent_at_staged> <answered_at_staged> <rx> <ry> <rz> <rw> <percentage> <img file> <data file>
  resized <k> <cols> <rows> <file> <rx> <ry> <rz> <rw>   (invokeData the layers module computed at processed frame k)
  update <k> <idx> <sky file> <texture_w> <texture_h> <percentage> <i0..i15> <rx> <ry> <rz> <rw> <px> <py> <pz>
Floats are written with repr() (round-trip). Run with python3 -I.
"""
import argparse
import hashlib
import json
import os
import sys


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('record')
  a = ap.parse_args()
  rec = os.path.abspath(a.record)
  r = json.load(open(os.path.join(rec, 'result.json')))
  st = r['state']
  if r.get('status') != 'RECORDED':
    sys.exit('record status %s' % r.get('status'))
  out = []
  on = st['onStart']
  cfg = r['skyConfig']
  out.append('meta %d %d %d %d %d %d %d' % (on['videoWidth'], on['videoHeight'], on['orientation'],
                                            on['canvasWidth'], on['canvasHeight'], st['staged'],
                                            1 if r['mirrored'] else 0))
  out.append('config %d %r %r %r' % (1 if cfg.get('invertLayerMask') else 0,
                                     float(cfg.get('edgeSmoothness', 0.0)),
                                     -1.0 if r['near'] is None else r['near'],
                                     -1.0 if r['far'] is None else r['far']))
  ol = {o['k']: o for o in st['orientationLog']}
  for c in st['cams']:
    k = c['k']
    o = ol[k]
    out.append('frame %d %d %d cam/cam-%04d.rgba %r %r %r' % (k, c['w'], c['h'], k, o['alpha'], o['beta'], o['gamma']))
  for e in st['events']:
    name = e['name'].split('.')[-1]
    d = json.loads(e['detail'])
    if name == 'layerfound':
      out.append('event %d %s %r' % (e['staged'], name, d['percentage']))
    else:
      out.append('event %d %s' % (e['staged'], name))
  for iv in st['invokes']:
    k = iv['k']
    rot = iv['rotation']
    if 'answeredAtStaged' not in iv:
      continue
    out.append('invoke %d %d %d %r %r %r %r %r invoke/invoke-%04d.img.rgba invoke/invoke-%04d.data.f32' % (
        k, iv['sentAtStaged'], iv['answeredAtStaged'], rot['x'], rot['y'], rot['z'], rot['w'],
        iv['percentage'], k, k))
  for k, m in sorted((st.get('resizedMeta') or {}).items(), key=lambda kv: int(kv[0])):
    q = m['rotation']
    out.append('resized %d %d %d resized/resized-%04d.rgba %r %r %r %r' % (int(k), m['cols'], m['rows'], int(k), q['x'], q['y'], q['z'], q['w']))
  for u in st['updates']:
    if 'idx' not in u:
      continue
    k = u['staged'] - 1
    rot = u['rotation'] or {'x': 0, 'y': 0, 'z': 0, 'w': 1}
    pos = u['position'] or {'x': 0, 'y': 0, 'z': 0}
    intr = ' '.join(repr(x) for x in (u['intrinsics'] + [0.0] * 16)[:16])
    out.append('update %d %d sky/sky-%04d.rgba %d %d %r %s %r %r %r %r %r %r %r' % (
        k, u['idx'], u['idx'], u['textureWidth'], u['textureHeight'], u['percentage'], intr,
        rot['x'], rot['y'], rot['z'], rot['w'], pos['x'], pos['y'], pos['z']))
  text = '\n'.join(out) + '\n'
  open(os.path.join(rec, 'replay.txt'), 'w').write(text)
  print('replay.txt', hashlib.sha256(text.encode()).hexdigest(), len(out), 'lines')


if __name__ == '__main__':
  main()
