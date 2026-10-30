#!/usr/bin/env python3
"""Synthetic test of orig_itprobe_analysis.py (no SDK, no engine): fabricates an original itprobe.jsonl.gz from the libc++ emulation of mt_tools and
an engine rng/probe file whose stream equals it up to frame 9 and then differs; checks the reported first differences and the seed text round trip
(the seed text is loaded by a real libstdc++ mt19937 through mt_check)."""
import base64, gzip, json, os, pathlib, struct, subprocess, sys, tempfile, unittest
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mt_tools as mt  # noqa: E402

DRAWS = {3: 40, 6: 700, 9: 5, 12: 624}   # frame -> draws consumed in that frame (original); the engine consumes a different number at frame 9


def b64(x, i):
    return base64.b64encode(struct.pack('<625I', *x, i)).decode()


def build(tmp):
    x, i = mt.seed_state()
    recs = []
    eng_rng = {}
    ex, ei = mt.seed_state()
    for f in range(1, 16):
        st_in = (list(x), i)
        n = DRAWS.get(f, 0)
        x, i = mt.advance(x, i, n)
        ne = n if f != 9 else n + 3
        ex, ei = mt.advance(ex, ei, ne)
        slots = [{'tag': 3, 'a': 0, 'b': 0, 'c': 0, 'state': b64(*st_in)}, {'tag': 1, 'a': 1 if f % 2 else 0, 'b': 30, 'c': f}]
        recs.append({'i': f, 'seq': f + 1, 'count': len(slots), 'rngPtr': 100, 'slots': slots, 'post': b64(x, i)})
        eng_rng[f] = ' '.join(mt.sig8_hex(ex, ei))
    p = pathlib.Path(tmp)
    with gzip.open(p / 'itprobe.jsonl.gz', 'wt') as g:
        g.write(json.dumps({'schema': 'x', 'wrapped': True, 'error': None}) + '\n')
        for r in recs:
            g.write(json.dumps(r) + '\n')
    (p / 'eng.rng').write_text(''.join(json.dumps({'i': f, 'sig': s}) + '\n' for f, s in eng_rng.items()))
    probe = []
    for f in range(1, 16):
        hom = [{'fn': 'hom', 'ok': bool(f % 2), 'n': 30, 'inl': f if f != 11 else 99}]
        probe.append({'i': f, 'stages': hom, 'postSt': 3})
    (p / 'eng.probe.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in probe))
    return p


class T(unittest.TestCase):
    def test_compare_and_seed(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = build(tmp)
            out = subprocess.run([sys.executable, '-I', str(HERE / 'orig_itprobe_analysis.py'), 'compare', '--orig', str(p / 'itprobe.jsonl.gz'),
                                  '--engine-probe', str(p / 'eng.probe.jsonl'), '--engine-rng', str(p / 'eng.rng')], capture_output=True, text=True, check=True).stdout
            r = json.loads(out)
            self.assertEqual(r['first_rng_state_difference_frame'], 9)
            self.assertEqual(r['first_hom_call_difference_frame'], 11)
            self.assertEqual(r['frames_with_draws_original'][:5], [1, 3, 6, 9, 12])
            # seed: the state at the start of frame 12 as libstdc++ text must continue like the original stream
            subprocess.run([sys.executable, '-I', str(HERE / 'orig_itprobe_analysis.py'), 'seed', '--orig', str(p / 'itprobe.jsonl.gz'), '--frame', '12', '--out', str(p / 's.txt')], check=True, capture_output=True)
            n12 = 40 + 700 + 5   # draws consumed before frame 12
            chk = p / 'mt_check'
            if subprocess.run(['g++', '-O1', '-std=c++17', '-o', str(chk), str(HERE / 'mt_check.cc')], capture_output=True).returncode == 0:
                res = subprocess.run([str(chk), str(n12), str(p / 's.txt')], capture_output=True, text=True)
                self.assertEqual(res.returncode, 0, res.stdout)


if __name__ == '__main__':
    unittest.main()
