#!/usr/bin/env python3
"""Builds a gate-test binary that contains ONE mutated copy of a sky source file (protocol §4: mutants are copies, the shared
sources are never edited). US-020 / US-021 preregistration evidence.

  build_source_mutant.py --build <cmake build dir> --target <gate test target> --src <path of the original source>
                         --replace '<old>' '<new>' [--replace ...] --out <new scratch dir> [--only-first]

Copies the source to <out>/mutated.cc, applies the replacements (each must match exactly once unless --only-first), compiles it
with the compile flags ninja uses for the original translation unit, and links <out>/gate: the mutated object comes BEFORE the
project library on the link line, so the unmodified archive member of that translation unit is not pulled in. The original
build tree is not changed. Python 3, standard library only.
"""
import argparse
import os
import shlex
import subprocess
import sys


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--build', required=True)
  ap.add_argument('--target', required=True)
  ap.add_argument('--src', required=True)
  ap.add_argument('--replace', nargs=2, action='append', required=True)
  ap.add_argument('--out', required=True)
  ap.add_argument('--only-first', action='store_true')
  a = ap.parse_args()
  src = os.path.abspath(a.src)
  out = os.path.abspath(a.out)
  os.makedirs(out, exist_ok=False)
  text = open(src).read()
  for old, new in a.replace:
    n = text.count(old)
    if n == 0 or (n > 1 and not a.only_first):
      sys.exit('replacement %r matches %d times' % (old, n))
    text = text.replace(old, new, 1)
  if text == open(src).read():
    sys.exit('mutation changed nothing')
  mutated = os.path.join(out, 'mutated.cc')
  open(mutated, 'w').write(text)
  cmds = subprocess.run(['ninja', '-C', a.build, '-t', 'commands', a.target], check=True, capture_output=True, text=True).stdout.splitlines()
  compile_cmd = [c for c in cmds if ' -c ' in c and src in c]
  if len(compile_cmd) != 1:
    sys.exit('found %d compile commands for %s' % (len(compile_cmd), src))
  argv = shlex.split(compile_cmd[0])
  clean, i = [], 0
  while i < len(argv):
    t = argv[i]
    if t in ('-MD',):
      i += 1
    elif t in ('-MT', '-MF'):
      i += 2
    elif t == '-o':
      clean += ['-o', os.path.join(out, 'mutated.o')]
      i += 2
    elif t == '-c':
      clean += ['-c', mutated]
      i += 2
    else:
      clean.append(t)
      i += 1
  subprocess.run(clean, check=True, cwd=a.build)
  link = [c for c in cmds if ' -o ' in c and a.target in c and ' -c ' not in c and 'ar qc' not in c]
  link = [c for c in link if c.startswith(': && /usr/bin/c++')]
  if len(link) != 1:
    sys.exit('found %d link commands' % len(link))
  seg = link[0].split('&&')[1].strip()
  largv = shlex.split(seg)
  o = largv.index('-o')
  largv[o + 1] = os.path.join(out, 'gate')
  # the mutated object goes first among the inputs (before every archive)
  first_input = next(i for i, t in enumerate(largv) if t.endswith('.o'))
  largv.insert(first_input, os.path.join(out, 'mutated.o'))
  subprocess.run(largv, check=True, cwd=a.build)
  print(os.path.join(out, 'gate'))


if __name__ == '__main__':
  main()
