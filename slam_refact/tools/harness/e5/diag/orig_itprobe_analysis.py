#!/usr/bin/env python3
"""Analysis of an instrumented ORIGINAL run (itprobe.jsonl.gz from run_original_itprobe.mjs) against the exact-input ENGINE replay of the same run
(sr_e5_diag with ITPROBE_OUT=<engine itprobe.jsonl> ITPROBE_RNG_OUT=<engine rng file>).  No engine, SDK or browser needed.

  orig_itprobe_analysis.py compare --orig itprobe.jsonl.gz --engine-probe eng.itprobe.jsonl --engine-rng eng.rng [--frames 1-150] [--json out.json]
      first frame where the original's shared-RNG state differs from the engine's (end of frame; signature = next 8 raw outputs, representation
      independent), first frame where the sequence of local-homography calls (return, n rays, inliers) differs, first frame where `found` differs;
      also tries frame shifts -1/+1 to flag an index misalignment.
  orig_itprobe_analysis.py seed --orig itprobe.jsonl.gz --frame F --out state.txt
      libstdc++ text state of the original's stream at the START of frame F (tag-3 slot of frame F = func 784 entry, before any RNG use of the frame;
      falls back to the post state of the previous record), for the engine run  ITPROBE_RNG_SET=F:state.txt  (itprobe.cc).
  orig_itprobe_analysis.py summary --orig itprobe.jsonl.gz [--frames 100-130]
      per frame: RNG signature changes, the original's hom calls and found flags (human readable)."""
import argparse, gzip, json, re, sys, pathlib

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mt_tools as mt  # noqa: E402


def load_orig(path):
    rows = [json.loads(l) for l in gzip.open(path, 'rt')]
    return rows[0], rows[1:]


def frame_range(spec):
    if not spec:
        return None
    a, b = spec.split('-')
    return int(a), int(b)


def sig_of_state(b64):
    x, i = mt.parse_state(b64)
    return ' '.join(mt.sig8_hex(x, i))


def orig_frames(recs):
    """frame i -> record (first fresh record per frame), plus the ordered list of all records"""
    by = {}
    for r in recs:
        if r.get('i') is not None and r['i'] not in by:
            by[r['i']] = r
    return by


def orig_hom(rec):
    return [(s['a'], s['b'], s['c']) for s in rec['slots'] if s['tag'] == 1]


def orig_found(rec):
    f = [s['a'] for s in rec['slots'] if s['tag'] == 2]
    return f[0] if f else None


def load_engine_probe(path):
    out = {}
    for l in open(path):
        l = re.sub(r'-?nan', 'null', l)
        l = re.sub(r'-?inf', 'null', l)
        r = json.loads(l)
        out[r['i']] = r
    return out


def engine_hom(rec):
    return [(1 if s['ok'] else 0, s['n'], s['inl']) for s in rec['stages'] if s['fn'] == 'hom']


def engine_found(rec):
    return 1 if rec['postSt'] in (2, 3) else 0


def load_engine_rng(path):
    return {json.loads(l)['i']: json.loads(l)['sig'] for l in open(path)}


def cmd_compare(a):
    hdr, recs = load_orig(a.orig)
    O = orig_frames(recs)
    E = load_engine_probe(a.engine_probe)
    R = load_engine_rng(a.engine_rng)
    rng = frame_range(a.frames) or (min(O), max(O))
    frames = [i for i in sorted(O) if rng[0] <= i <= rng[1] and O[i].get('post')]
    res = {'orig': a.orig, 'frames': rng, 'wrapped': hdr.get('wrapped'), 'error': hdr.get('error')}
    sig = {i: sig_of_state(O[i]['post']) for i in frames}
    for shift in (0, -1, 1):
        eq = sum(1 for i in frames if R.get(i + shift) == sig[i])
        res[f'rng_equal_frames_shift{shift:+d}'] = f'{eq}/{len(frames)}'
    first = next((i for i in frames if i in R and R[i] != sig[i]), None)
    res['first_rng_state_difference_frame'] = first
    if first is not None:
        res['rng_at_first_difference'] = {'orig': sig[first], 'engine': R[first], 'orig_previous_frame': sig.get(first - 1), 'engine_previous_frame': R.get(first - 1)}
    first_h = None
    for i in frames:
        if i in E and orig_hom(O[i]) != engine_hom(E[i]):
            first_h = i
            break
    res['first_hom_call_difference_frame'] = first_h
    if first_h is not None:
        res['hom_at_first_difference'] = {'orig(ret,n,inliers)': orig_hom(O[first_h]), 'engine(ok,n,inliers)': engine_hom(E[first_h])}
    first_f = next((i for i in frames if i in E and orig_found(O[i]) is not None and orig_found(O[i]) != engine_found(E[i])), None)
    res['first_found_difference_frame'] = first_f
    # the draws: frames where the stream changed
    ch_o = [i for k, i in enumerate(frames) if k == 0 or sig[i] != sig[frames[k - 1]]]
    ch_e = [i for k, i in enumerate(frames) if i in R and (k == 0 or R.get(frames[k - 1]) != R[i])]
    res['frames_with_draws_original'] = ch_o
    res['frames_with_draws_engine'] = ch_e
    print(json.dumps(res, indent=1))
    if a.json:
        pathlib.Path(a.json).write_text(json.dumps(res, indent=1))


def seed_positions(max_draws=400000):
    """signature (next 8 raw outputs, hex string) -> number of draws consumed since a fresh seed-5489 state; one stream, built once"""
    x, i = mt.seed_state()
    out = []
    ws = []
    for _ in range(max_draws + 8):
        o, i = mt.step(x, i)
        out.append(o)
    pos = {}
    for n in range(max_draws):
        pos.setdefault(' '.join('%08x' % v for v in out[n:n + 8]), n)
    return pos


def cmd_draws(a):
    """draws consumed since the last re-seed (engine re-creation at the start recenter), original vs engine, per frame"""
    hdr, recs = load_orig(a.orig)
    O = orig_frames(recs)
    R = load_engine_rng(a.engine_rng)
    pos = seed_positions(a.max_draws)
    rng = frame_range(a.frames) or (min(O), max(O))
    rows = []
    for f in sorted(O):
        if not (rng[0] <= f <= rng[1]) or not O[f].get('post') or f not in R:
            continue
        rows.append((f, pos.get(sig_of_state(O[f]['post'])), pos.get(R[f])))
    first = next((r for r in rows if r[1] != r[2]), None)
    # draws only count after the last frame where both are on the fresh seed stream (position 0 or unknown before the re-seed)
    res = {'rows(frame,orig_draws,engine_draws)': rows if a.verbose else None, 'first_frame_with_different_draw_count': first,
           'unlocatable_frames(not on the seed stream within max_draws)': [r[0] for r in rows if r[1] is None or r[2] is None][:20]}
    if first:
        f = first[0]
        prev = [r for r in rows if r[0] < f][-1:] or None
        res['previous_frame_rows'] = prev
    print(json.dumps(res, indent=1))
    if a.json:
        pathlib.Path(a.json).write_text(json.dumps(res, indent=1))


def cmd_seed(a):
    hdr, recs = load_orig(a.orig)
    by_seq = {r['seq']: r for r in recs}
    O = orig_frames(recs)
    rec = O[a.frame]
    state = None
    for s in rec['slots']:
        if s['tag'] == 3:
            state = s['state']
            src = f'tag-3 slot of frame {a.frame} (func 784 entry)'
            break
    if state is None:
        prev = by_seq.get(rec['seq'] - 1)
        assert prev and prev.get('post'), 'no tag-3 slot and no previous post state'
        state = prev['post']
        src = f'post state of record seq {prev["seq"]}'
    x, i = mt.parse_state(state)
    pathlib.Path(a.out).write_text(mt.to_libstdcxx_text(x, i))
    print(f'wrote {a.out} from {src}; libc++ index {i}; sig8 {mt.sig8_hex(x, i)}')


def cmd_summary(a):
    hdr, recs = load_orig(a.orig)
    O = orig_frames(recs)
    rng = frame_range(a.frames) or (min(O), max(O))
    prev = None
    for i in sorted(O):
        if not (rng[0] <= i <= rng[1]):
            continue
        r = O[i]
        sig = sig_of_state(r['post']) if r.get('post') else None
        print(i, 'rng-changed' if sig != prev else 'rng-same', 'hom', orig_hom(r), 'found', orig_found(r))
        prev = sig


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('compare'); c.add_argument('--orig', required=True); c.add_argument('--engine-probe', required=True); c.add_argument('--engine-rng', required=True)
    c.add_argument('--frames'); c.add_argument('--json'); c.set_defaults(fn=cmd_compare)
    s = sub.add_parser('seed'); s.add_argument('--orig', required=True); s.add_argument('--frame', type=int, required=True); s.add_argument('--out', required=True); s.set_defaults(fn=cmd_seed)
    d = sub.add_parser('draws'); d.add_argument('--orig', required=True); d.add_argument('--engine-rng', required=True); d.add_argument('--frames'); d.add_argument('--json')
    d.add_argument('--max-draws', type=int, default=400000); d.add_argument('--verbose', action='store_true'); d.set_defaults(fn=cmd_draws)
    m = sub.add_parser('summary'); m.add_argument('--orig', required=True); m.add_argument('--frames'); m.set_defaults(fn=cmd_summary)
    a = ap.parse_args()
    a.fn(a)


if __name__ == '__main__':
    main()
