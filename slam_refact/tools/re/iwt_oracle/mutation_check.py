#!/usr/bin/env python3
"""Mutation check of the func994 oracle comparison (US-015): planted bugs in a copy of the engine's
image_world_tracker.cc must make compare.py fail on the same cases.

For each mutant the copy is compiled with the exact compile command ninja uses for the real file,
linked in front of libsr_image_target.a (the copy defines every symbol of the original object, so the
archive member is not pulled) together with the replay main object, run on the cases, and compared
with the oracle. The shared source is not modified.

  python3 -I mutation_check.py BUILD_DIR CASES.txt ORACLE.json OUT_DIR
Stdlib only. Needs the build dir's ninja (the replay target must be built).
"""
import json
import os
import pathlib
import shlex
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parents[4]
SRC = REPO / 'slam_refact/engine/src/image_target/image_world_tracker.cc'
HERE = pathlib.Path(__file__).resolve().parent

# (name, original text, mutated text, what it breaks)
MUTANTS = [
    ('M1-min-pairs-5', 'constexpr size_t kMinPairs = 6;', 'constexpr size_t kMinPairs = 5;',
     'fallback threshold (5 pairs solve instead of using the scale guess)'),
    ('M2-any-source', 'mp.source != kSourceTriangulated', 'mp.source < 0',
     'TRIANGULATED filter dropped (GROUND points paired)'),
    ('M3-offset-sign', '(mp.position.x() - xf.offset[0]) * xf.scale',
     '(mp.position.x() + xf.offset[0]) * xf.scale', 'map transform offset sign on x'),
    # (A first M4 that built the fallback pose from st->scale was an equivalent mutant: st->scale was
    # already set to the guess one line above. Replaced.)
    ('M4-guess-not-stored', '    st->scale = scale_guess;\n', '',
     'fallback keeps the previous scale instead of storing the guess'),
    ('M5-no-tam-inv', 'tam.inv() * st->worldPts', 'tam * st->worldPts',
     'world points not moved into the camera frame before the solve'),
]


def ninja_commands(build, target):
  out = subprocess.run(['ninja', '-C', str(build), '-t', 'commands', target], check=True,
                       capture_output=True, text=True).stdout.splitlines()
  return out


def main():
  build, cases, oracle, out_dir = map(pathlib.Path, sys.argv[1:5])
  out_dir.mkdir(parents=True, exist_ok=True)
  cmds = ninja_commands(build, 'sr_iwt_replay')
  compile_cmd = next(c for c in cmds if c.endswith(str(SRC)) or str(SRC) in c and ' -c ' in c)
  link_cmd = next(c for c in cmds if '-o tests/image_target/sr_iwt_replay ' in c)
  text = SRC.read_text()
  results = {}
  for name, old, new, what in MUTANTS:
    assert text.count(old) == 1, (name, 'source no longer contains the mutated text')
    mdir = out_dir / name
    mdir.mkdir(exist_ok=True)
    msrc = mdir / 'image_world_tracker_mutant.cc'
    msrc.write_text(text.replace(old, new))
    mobj = mdir / 'mutant.o'
    argv = shlex.split(compile_cmd)
    o = argv.index('-o')
    argv[o + 1] = str(mobj)
    argv[argv.index('-c') + 1] = str(msrc)
    for flag in ('-MT', '-MF'):  # dependency-file options with an argument
      while flag in argv:
        i = argv.index(flag)
        del argv[i:i + 2]
    argv = [a for a in argv if a != '-MD']
    subprocess.run(argv, check=True, cwd=build)
    largv = shlex.split(link_cmd.strip(': &').replace('&& :', ''))
    lo = largv.index('-o')
    largv[lo + 1] = str(mdir / 'sr_iwt_replay_mutant')
    largv.insert(lo, str(mobj))
    subprocess.run(largv, check=True, cwd=build)
    eng = mdir / 'engine.txt'
    with open(eng, 'w') as f:
      subprocess.run([str(mdir / 'sr_iwt_replay_mutant'), str(cases)], check=True, stdout=f,
                     stderr=subprocess.DEVNULL)
    cmp = subprocess.run([sys.executable, '-I', str(HERE / 'compare.py'), str(oracle), str(eng)],
                         capture_output=True, text=True)
    (mdir / 'compare.json').write_text(cmp.stdout)
    rep = json.loads(cmp.stdout)
    failing = sorted(n for n, r in rep['cases'].items()
                     if r.get('error') or not r.get('visible_equal', False) or not r.get('ids_equal', False)
                     or r['scale']['abs_diff'] > rep['tol'] or r['out']['max_abs_diff'] > rep['tol']
                     or any(isinstance(v, dict) and isinstance(v.get('count'), list) for v in r.values()))
    results[name] = {'breaks': what, 'compare_exit': cmp.returncode, 'detected': cmp.returncode != 0,
                     'failing_cases': failing}
  ok = all(r['detected'] for r in results.values())
  print(json.dumps({'all_detected': ok, 'mutants': results}, indent=1))
  sys.exit(0 if ok else 1)


if __name__ == '__main__':
  main()
