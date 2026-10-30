"""Extract the loop close golden vectors (US-025 LC-4) from the lc-g dump into engine/tests/data/loop_close/.

Sources (original xr-slam.wasm, produced by the LC-G worker, read only; format: docs/design/us025-lcg-golden.md):
  G-CFGH  <golden>/G-CFGH/<run>/lcgold/{events.jsonl,hooks.jsonl,blobs.bin}   candidate selection state per attempt
  G-DE    <golden>/G-DE/<run>/lcgold/{hooks.jsonl,blobs.bin}                   func 1128 / 1146 / 660 input and output

Outputs (compact text/binary; the C++ test needs no JSON parser):
  select-attempts.tsv  one line per loop close attempt that passed S1 (slot count >= 5): the slot/id/covisibility
                       state, the original's pick (ring) and, for hook runs, the original's u2-result and the
                       uninitialised tail of its distance list.
  pairs.tsv, pairs.bin keyframe frames (ray x/y/scale + ORB of the ORB-bearing points) and the original's local match
                       results, correspondences and (successful calls) PnP/660 records.

Usage (cwd = repo root):
  python3 -B -I slam_refact/tools/re/loop_close/gen_loop_close_test_data.py \
      --golden build/slam-refact-pkgh-20261022/us025/golden --out slam_refact/engine/tests/data/loop_close
Every extracted value is cross-checked against a second record of the dump (see the assert messages); nothing is
derived from the engine.
"""

import argparse
import glob
import hashlib
import json
import os
import struct
import sys


def rd(blobs, ref):
  return blobs[ref['off']:ref['off'] + ref['len']]


def load_run(run_dir):
  lg = os.path.join(run_dir, 'lcgold')
  blobs = open(os.path.join(lg, 'blobs.bin'), 'rb').read()
  events = [json.loads(l) for l in open(os.path.join(lg, 'events.jsonl')) if l.strip()]
  hooks = []
  hp = os.path.join(lg, 'hooks.jsonl')
  if os.path.exists(hp):
    hooks = [json.loads(l) for l in open(hp) if l.strip()]
  return blobs, events, hooks


# ---------------------------------------------------------------------------------------------
# select-attempts.tsv
# ---------------------------------------------------------------------------------------------


def gen_select(golden, out):
  runs = sorted(d for d in os.listdir(os.path.join(golden, 'G-CFGH')) if os.path.isdir(os.path.join(golden, 'G-CFGH', d)))
  desc_lines = []
  att_lines = []
  n_att = n_hook = 0
  for run in runs:
    blobs, events, hooks = load_run(os.path.join(golden, 'G-CFGH', run))
    by_seq = {}
    for h in hooks:
      by_seq.setdefault(h['seq'], []).append(h)
    table = {}  # kf id -> descriptor (a (slot, id) descriptor never changes while the keyframe lives)
    for ev in events:
      if 'skipped' in ev or not ev.get('lcEnabled') or ev['nSlots'] < 5:
        continue
      n_slots = ev['nSlots']
      assert ev['nDesc'] == n_slots == ev['nHandles'], 'vector sizes differ'
      vec = rd(blobs, ev['desc'])
      ids = [ev['handles'][2 * s + 1] for s in range(n_slots)]
      for s in range(n_slots):
        assert ev['handles'][2 * s] == s, 'handle index != slot'
        if ids[s] == 0:
          continue
        d = vec[32 * s:32 * s + 32].hex()
        assert table.setdefault(ids[s], d) == d, 'descriptor of a live keyframe changed'
      ni, nid = ev['newest']
      assert ids[ni] == nid, 'newest handle != slot id'
      ring = ev['ringPost']
      assert ring[0:2] == [ni, nid], 'ring was not reset to the newest handle'
      pick = ring[2] if (ring[5] != 0 or ring[2] != 0) else 0
      covis = {s: c for s, c in ev['covis']}
      edges = {}
      for s, lst in covis.items():
        for (i, idn, sh) in lst:
          assert ids[i] == idn and idn != 0 and ids[s] != 0, 'covisibility entry of a dead keyframe'
          other = {x[0]: x[2] for x in covis[i]}
          assert other.get(s) == sh, 'covisibility is not symmetric'
          if sh >= 9 and s < i:
            edges[(s, i)] = 10 if sh >= 10 else 9
      edge_txt = ';'.join('%d-%d-%d' % (a, b, c) for (a, b), c in sorted(edges.items())) or '-'
      hook = '-'
      tail = '-'
      recs = by_seq.get(ev['seq'], [])
      res = [h for h in recs if h['id'] == 'u2-result']
      dist = [h for h in recs if h['id'] == 'u2-dist']
      if res:
        assert len(res) == 1 and len(dist) == 1
        r = res[0]
        hook = '%08x:%d:%d:%d' % (r['bestBits'], r['candIdx'], r['candId'], r['found'])
        raw = rd(blobs, dist[0]['dist'])
        assert len(raw) == 12 * n_slots
        live = sum(1 for x in ids if x != 0)
        ent = []
        for m in range(n_slots):
          bits, idx, ident = struct.unpack_from('<III', raw, 12 * m)
          if m < live:
            # the compressed live prefix must be the S3 model (checked again in the test)
            continue
          ent.append('%d:%08x:%d:%d' % (m, bits, idx, ident))
        tail = ';'.join(ent) or '-'
        n_hook += 1
      att_lines.append('A\t%s\t%d\t%d\t%d\t%d\t%s\t%d\t%s\t%s\t%s' % (
          run, ev['seq'], 1 if res else 0, n_slots, ni, ','.join(str(x) for x in ids), pick, edge_txt, hook, tail))
      n_att += 1
    for kid, d in sorted(table.items()):
      desc_lines.append('D\t%s\t%d\t%s' % (run, kid, d))
  with open(os.path.join(out, 'select-attempts.tsv'), 'w') as f:
    f.write('# US-025 LC-4 G-C: loop close attempts of the original with >= 5 keyframe slots (S1 passed), %d attempts (%d with the hook record)\n' % (n_att, n_hook))
    f.write('# D run kfid descriptor_hex32B\n')
    f.write('# A run seq hook nSlots newestIdx ids(0=free) pickId(0=none) edges(a-b-shared,shared 9 or 10) u2result(bestBits:candIdx:candId:found) tail(m:bits:idx:id for m>=live)\n')
    f.write('\n'.join(desc_lines + att_lines) + '\n')
  print('select-attempts: %d attempts (%d hook), %d descriptors' % (n_att, n_hook, len(desc_lines)))


# ---------------------------------------------------------------------------------------------
# pairs.tsv / pairs.bin
# ---------------------------------------------------------------------------------------------


def frame_blob(blobs, kf):
  """ORB-bearing points of a keyframe: (point idx u32, x f32, y f32, scale u8, orb 32 B)."""
  n_pts = kf['nPts']
  pts = rd(blobs, kf['pts'])
  assert len(pts) == 0x50 * n_pts
  orb = rd(blobs, kf['orb'])
  kp = struct.unpack('<%dI' % kf['nOrb'], rd(blobs, kf['kp']))
  assert len(orb) == 32 * kf['nOrb'] and len(kp) == kf['nOrb']
  assert all(kp[i] < kp[i + 1] for i in range(len(kp) - 1)), 'keypoint indices are not ascending'
  out = bytearray()
  for d, p in enumerate(kp):
    scale = pts[0x50 * p]
    x, y = struct.unpack_from('<2I', pts, 0x50 * p + 0x18)
    out += struct.pack('<IIIB', p, x, y, scale) + orb[32 * d:32 * d + 32]
  return bytes(out)


def pts_xy_scale(blobs, kf, p):
  pts = rd(blobs, kf['pts'])
  x, y = struct.unpack_from('<2I', pts, 0x50 * p + 0x18)
  return x, y, pts[0x50 * p]


def gen_pairs(golden, out):
  runs = sorted(d for d in os.listdir(os.path.join(golden, 'G-DE')) if os.path.isdir(os.path.join(golden, 'G-DE', d)))
  frames = {}      # key -> (bytes, kf dict)
  cases = []       # tuples
  zero_pool = []   # zero-match calls with ORB on both sides
  chosen_blob = bytearray()
  pnp_count = 0
  for run in runs:
    blobs, _events, hooks = load_run(os.path.join(golden, 'G-DE', run))
    by = {}
    for h in hooks:
      by.setdefault(h['seq'], []).append(h)
    for seq, recs in sorted(by.items()):
      i128 = [h for h in recs if h['id'] == '1128-in']
      if not i128:
        continue
      assert len(i128) == 1
      a = i128[0]
      o = [h for h in recs if h['id'] == '1128-out'][0]
      ls = [h for h in recs if h['id'] == 'lc-start'][0]
      new, cand = a['newest'], a['cand']
      self_case = new['id'] == cand['id']
      nm = o['nMatches']
      matches = rd(blobs, o['matches']) if nm else b''
      assert len(matches) == 20 * nm
      p146 = [h for h in recs if h['id'] == '1146-in']
      select = False
      if nm > 0 and (not self_case or nm >= 20):  # keep every non-self call and the self calls with >= 20 matches
        select = True
      elif new['nOrb'] > 0 and cand['nOrb'] > 0:
        zero_pool.append((new['nOrb'] * cand['nOrb'], run, seq))
      cases.append(dict(run=run, seq=seq, blobs=blobs, a=a, o=o, ls=ls, new=new, cand=cand, self=self_case, nm=nm,
                        matches=matches, p146=p146, recs=recs, select=select))
  zero_pool.sort(reverse=True)
  zero_keep = {(r, s) for _, r, s in zero_pool[:8]}

  def key_of(run, kf):
    return '%s:%d' % (run, kf['id'])

  lines = []
  bin_out = bytearray()
  frame_off = {}
  n_cases = 0
  case_lines = []
  for c in cases:
    if not (c['select'] or (c['run'], c['seq']) in zero_keep):
      continue
    blobs = c['blobs']
    new, cand = c['new'], c['cand']
    for kf in (new, cand):
      k = key_of(c['run'], kf)
      fb = frame_blob(blobs, kf)
      if k in frames:
        assert frames[k][0] == fb, 'keyframe %s differs between calls' % k
      else:
        frames[k] = (fb, kf)
    # layout cross-check: the dictionary scale of every match is the candidate point's scale byte
    for i in range(c['nm']):
      w, q, rot, dist, scale = struct.unpack_from('<IIffB', c['matches'], 20 * i)
      assert pts_xy_scale(blobs, cand, q)[2] == scale, 'scale byte layout'
      assert q < cand['nPts'] and w < new['nPts']
    wpi = c['ls']['newestDetail']['wpi'] if c['self'] else None
    corr = '-'
    if c['self'] and c['nm'] >= 75:
      wset = set(wpi)
      assert len(wpi) == c['ls']['newestDetail']['wpiSize']
      qs = [struct.unpack_from('<II', c['matches'], 20 * i) for i in range(c['nm'])]
      ncorr = sum(1 for (w, q) in qs if q in wset)
      corr = str(ncorr)
      if c['p146']:
        p = c['p146'][0]
        assert p['nObs'] == ncorr == p['nWp'], 'correspondence count != 1146 nObs'
        obs = rd(blobs, p['obs'])
        wp = rd(blobs, p['wp'])
        j = 0
        for (w, q) in qs:
          if q not in wset:
            continue
          x, y, _ = pts_xy_scale(blobs, new, w)
          ox, oy, ow, osc, odist, owt = struct.unpack_from('<IIIiff', obs, 24 * j)
          assert (ox, oy) == (x, y) and ow == 0x3f800000 and osc == 1 and odist == 0.0 and owt == 1.0, 'ray %d' % j
          j += 1
        o146 = [h for h in c['recs'] if h['id'] == '1146-out'][0]
        pose_in = rd(blobs, p['pose'])
        pose_out = rd(blobs, o146['pose'])
        assert len(pose_in) == 0x84
        ret = o146['ret']
        if ret:
          i660 = [h for h in c['recs'] if h['id'] == '660-in'][0]
          o660 = [h for h in c['recs'] if h['id'] == '660-out'][0]
          A, B, C = rd(blobs, i660['A']), rd(blobs, i660['B']), rd(blobs, o660['C'])
          assert pose_in[:0x82] == A[:0x82] and pose_out[:0x82] == B[:0x82], '1146 pose in/out != 660 A/B (first 0x82 B)'
        else:
          assert not [h for h in c['recs'] if h['id'] in ('660-in', '655-in')], 'a failed 1146 was followed by 660/655'
          A = B = C = bytes(0x84)
        c['pnp_block'] = struct.pack('<II', ncorr, ret) + wp + pose_in + pose_out + A + B + C
        pnp_count += 1
      else:
        c['pnp_block'] = None
    else:
      c['pnp_block'] = None
    c['corr'] = corr
    c['wpi'] = wpi
    n_cases += 1
  # layout: frames, then matches, then pnp blocks
  keys = sorted(frames)
  for k in keys:
    frame_off[k] = len(bin_out)
    bin_out += frames[k][0]
  for k in keys:
    kf = frames[k][1]
    w, h, cx, cy, fx, fy = kf['intrinsicsWords']
    lines.append('F\t%s\t%d\t%d\t%d\t%d\t%s\t%d' % (k, frame_off[k], len(frames[k][0]), kf['nPts'], kf['nOrb'],
                                                   ','.join('%08x' % v if i >= 2 else '%d' % v for i, v in enumerate(kf['intrinsicsWords'])),
                                                   kf['excludedEdgeWord']))
  for c in cases:
    if not (c['select'] or (c['run'], c['seq']) in zero_keep):
      continue
    m_off = len(bin_out)
    bin_out += c['matches']
    pnp_off = '-'
    if c['pnp_block'] is not None:
      pnp_off = str(len(bin_out))
      bin_out += c['pnp_block']
    wpi = ','.join(str(x) for x in c['wpi']) if c['wpi'] is not None else '-'
    case_lines.append('C\t%s:%d\t%s\t%s\t%s\t%d\t%d\t%s\t%s\t%s' % (
        c['run'], c['seq'], 'self' if c['self'] else 'pair', key_of(c['run'], c['new']), key_of(c['run'], c['cand']),
        c['nm'], m_off, wpi if c['self'] and c['nm'] >= 74 else '-', c['corr'], pnp_off))
  with open(os.path.join(out, 'pairs.tsv'), 'w') as f:
    f.write('# US-025 LC-4 G-D/G-E: original func 1128 (local match) calls of loop close attempts, %d cases, %d frames, %d PnP calls\n' % (len(case_lines), len(keys), pnp_count))
    f.write('# F key off len nPts nOrb intrinsics(width,height,cx,cy,fx,fy; float words in hex) excludedEdgePixels\n')
    f.write('#   frame blob = nOrb x (u32 point idx, u32 x bits, u32 y bits, u8 scale, 32 B orb), ascending point idx; other points have no ORB\n')
    f.write('# C id kind newKey candKey nMatches matchesOff wpi(cand obs idx with a world point; self cases with >= 74 matches) corr pnpOff\n')
    f.write('#   matches = nMatches x (u32 wordsIdx, u32 dictionaryIdx, f32 rotation, f32 descriptorDist, u8 scale + 3 pad)\n')
    f.write('#   pnp block = u32 n, u32 ret (1146 return value), n x HPoint3 (4 f32), pose_in 0x84, pose_out 0x84, 660 A 0x84, 660 B 0x84, 660 C 0x84 (A/B/C zero when ret = 0)\n')
    f.write('\n'.join(lines + case_lines) + '\n')
  with open(os.path.join(out, 'pairs.bin'), 'wb') as f:
    f.write(bytes(bin_out))
  print('pairs: %d cases, %d frames, %d pnp, %d bytes' % (len(case_lines), len(keys), pnp_count, len(bin_out)))


def main(argv=None):
  ap = argparse.ArgumentParser()
  ap.add_argument('--golden', required=True)
  ap.add_argument('--out', required=True)
  a = ap.parse_args(argv)
  os.makedirs(a.out, exist_ok=True)
  gen_select(a.golden, a.out)
  gen_pairs(a.golden, a.out)
  return 0


if __name__ == '__main__':
  sys.exit(main())
