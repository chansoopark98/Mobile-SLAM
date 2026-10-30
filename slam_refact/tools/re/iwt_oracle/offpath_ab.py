#!/usr/bin/env python3
"""US-015 review F-1: default-off output of the patched image target tracker against the unpatched public
files on a sequence where the target is detected, tracked, lost and found again.

A = sr_iwt_offpath_dump as built (patched vendored headers and sources, no world tracker installed).
B = the same main source compiled against the four imagedetection files with
    patches/us015-image-world-tracker.patch reverse-applied (copies under OUT/unpatched; an -isystem in front of the
    vendored -isystem makes them win), with the two unpatched .cc linked in front of the static libraries.
Every compile uses the exact ninja command of the corresponding real object. The shared sources and the build
tree are not written. Exit 0 when A and B print byte-identical output.

  python3 -I offpath_ab.py BUILD_DIR OUT_DIR [--plant]
--plant changes the public EMA (0.8 / 0.2 -> 0.7 / 0.3) in the unpatched copy: the outputs must then differ
(evidence that B really runs the copies and that the dump sees the scale path).
Stdlib only.
"""
import hashlib
import json
import pathlib
import shlex
import shutil
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parents[4]
VEND = REPO / 'slam_refact/engine/third_party/xr_source'
PATCH = VEND / 'patches/us015-image-world-tracker.patch'
FILES = ['image-tracking-state.h', 'image-tracking-state.cc', 'detection-image-tracker.h', 'detection-image-tracker.cc']
MAIN = REPO / 'slam_refact/engine/tests/image_target/iwt_offpath_dump_main.cc'


def strip_dep(argv):
  for flag in ('-MT', '-MF'):
    while flag in argv:
      i = argv.index(flag)
      del argv[i:i + 2]
  return [a for a in argv if a != '-MD']


def main():
  build, out = pathlib.Path(sys.argv[1]).resolve(), pathlib.Path(sys.argv[2]).resolve()
  out.mkdir(parents=True, exist_ok=False)
  cmds = subprocess.run(['ninja', '-C', str(build), '-t', 'commands', 'sr_iwt_offpath_dump'], check=True,
                        capture_output=True, text=True).stdout.splitlines()
  tree = out / 'unpatched'
  d = tree / 'reality/engine/imagedetection'
  d.mkdir(parents=True)
  for f in FILES:
    shutil.copy(VEND / 'reality/engine/imagedetection' / f, d / f)
  subprocess.run(['patch', '-s', '-R', '-p1', '-d', str(tree), '-i', str(PATCH)], check=True)
  if '--plant' in sys.argv:
    its = d / 'image-tracking-state.cc'
    text = its.read_text()
    old = 'scaleGuess_ = scaleGuess_ * .8 + guessedScale * .2;'
    assert text.count(old) == 1
    its.write_text(text.replace(old, 'scaleGuess_ = scaleGuess_ * .7 + guessedScale * .3;'))
  objs = []
  for src, real in (('main.o', MAIN), ('its.o', d / 'image-tracking-state.cc'), ('dit.o', d / 'detection-image-tracker.cc')):
    key = str(MAIN) if src == 'main.o' else '/reality/engine/imagedetection/' + real.name
    cmd = [c for c in cmds if ' -c ' in c and c.split(' -c ')[1].strip().endswith(key if src != 'main.o' else MAIN.name)]
    assert len(cmd) == 1, (src, len(cmd))
    argv = strip_dep(shlex.split(cmd[0]))
    argv[argv.index('-o') + 1] = str(out / src)
    argv[argv.index('-c') + 1] = str(real)
    argv[1:1] = ['-isystem', str(tree)]  # system dir like the vendored tree (same warning policy), searched first
    subprocess.run(argv, check=True, cwd=build)
    objs.append(str(out / src))
  link = next(c for c in cmds if '-o tests/image_target/sr_iwt_offpath_dump ' in c).strip()
  link = link[link.index('/usr/bin/c++'):].split(' && ')[0]
  largv = shlex.split(link)
  largv = [a for a in largv if not a.endswith('iwt_offpath_dump_main.cc.o')]
  lo = largv.index('-o')
  largv[lo + 1] = str(out / 'dump_unpatched')
  largv[lo:lo] = objs
  subprocess.run(largv, check=True, cwd=build)
  a = subprocess.run([str(build / 'tests/image_target/sr_iwt_offpath_dump')], check=True, capture_output=True).stdout
  b = subprocess.run([str(out / 'dump_unpatched')], check=True, capture_output=True).stdout
  (out / 'patched.txt').write_bytes(a)
  (out / 'unpatched.txt').write_bytes(b)
  lines = a.decode().splitlines()
  frames = [l for l in lines if l.startswith('frame ')]
  rep = {
      'identical': a == b, 'planted': '--plant' in sys.argv,
      'sha256_patched': hashlib.sha256(a).hexdigest(), 'sha256_unpatched': hashlib.sha256(b).hexdigest(),
      'frames': len(frames),
      'frames_with_located': sum(1 for l in frames if not l.endswith('located 0')),
      'located_records': sum(1 for l in lines if l.startswith(' li ')),
      'status_counts': {s: sum(1 for l in lines if l.startswith(' li ') and (' status %s ' % s) in l) for s in '123'},
      'max_lastSeen': max([int(l.split(' lastSeen ')[1].split()[0]) for l in lines if l.startswith(' li ')] or [-1]),
      'unpatched_differs_from_vendored': {f: (VEND / 'reality/engine/imagedetection' / f).read_bytes() != (d / f).read_bytes()
                                          for f in FILES},
  }
  (out / 'result.json').write_text(json.dumps(rep, indent=1) + '\n')
  print(json.dumps(rep, indent=1))
  sys.exit(0 if rep['identical'] != rep['planted'] else 1)


if __name__ == '__main__':
  main()
