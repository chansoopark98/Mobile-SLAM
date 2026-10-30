#!/usr/bin/env python3
"""Compute the #include closure of public xr_reference sources (header graph, not Bazel graph).

For each seed .cc/.h, follow quoted includes resolved against --root (and optional --shim dirs
that take precedence, mirroring -I order). Reports repo headers reached, the paired .cc files
(header foo.h -> foo.cc in the same dir, the Bazel convention), and non-repo includes grouped
as external (Eigen, ceres, capnp, GL, ...).

Usage:
  include_closure.py --root R [--shim DIR ...] --out OUT.json SEED [SEED ...]
"""

import argparse
import json
import os
import re
import sys

INCLUDE_RE = re.compile(r'^\s*#\s*include\s*([<"])([^>"]+)[>"]', re.M)

EXTERNAL_PREFIXES = {
    'Eigen/': 'eigen', 'unsupported/Eigen': 'eigen', 'ceres/': 'ceres', 'glog/': 'glog',
    'capnp/': 'capnp', 'kj/': 'capnp', 'GLES': 'opengl', 'EGL/': 'opengl', 'OpenGL/': 'opengl',
    'GL/': 'opengl', 'emscripten': 'emscripten', 'nlohmann/': 'json', 'zlib.h': 'zlib',
    'kiss_fft': 'kissfft', 'opencv2/': 'opencv', 'arm_neon.h': 'neon', 'benchmark/': 'benchmark',
    'gtest/': 'gtest', 'gmock/': 'gtest', 'spdlog/': 'spdlog', 'absl/': 'abseil',
}


def strip_comments(text):
  text = re.sub(r'/\*.*?\*/', '', text, flags=re.S)
  return re.sub(r'//[^\n]*', '', text)


def resolve(inc, cur_dir, root, shims):
  for base in shims + [root]:
    p = os.path.join(base, inc)
    if os.path.isfile(p):
      return p
  p = os.path.join(cur_dir, inc)
  if os.path.isfile(p):
    return p
  return None


def closure(seeds, root, shims):
  seen = set()
  external = {}
  unresolved = {}
  stack = [os.path.join(root, s) if not os.path.isabs(s) else s for s in seeds]
  while stack:
    path = os.path.normpath(stack.pop())
    if path in seen:
      continue
    seen.add(path)
    with open(path, errors='replace') as f:
      text = strip_comments(f.read())
    for kind, inc in INCLUDE_RE.findall(text):
      hit = resolve(inc, os.path.dirname(path), root, shims)
      if hit and (kind == '"' or inc.startswith(('c8/', 'reality/', 'third_party/'))):
        stack.append(hit)
        continue
      group = None
      for pref, g in EXTERNAL_PREFIXES.items():
        if inc.startswith(pref):
          group = g
          break
      rel = os.path.relpath(path, root)
      if group:
        external.setdefault(group, set()).add(rel)
      elif kind == '"' or '/' in inc and not inc.startswith(('sys/', 'bits/')):
        unresolved.setdefault(inc, set()).add(rel)
  repo = sorted(os.path.relpath(p, root) for p in seen if p.startswith(os.path.abspath(root)))
  shim_hits = sorted(p for p in seen if not p.startswith(os.path.abspath(root)))
  paired = []
  for h in repo:
    if h.endswith('.h'):
      cc = h[:-2] + '.cc'
      if os.path.isfile(os.path.join(root, cc)):
        paired.append(cc)
  return {
      'repo_files': repo,
      'paired_cc': sorted(set(paired)),
      'shim_files': shim_hits,
      'external': {k: sorted(v) for k, v in sorted(external.items())},
      'unresolved': {k: sorted(v) for k, v in sorted(unresolved.items())},
  }


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--root', required=True)
  ap.add_argument('--shim', action='append', default=[])
  ap.add_argument('--out', required=True)
  ap.add_argument('--per-seed', action='store_true', help='one closure per seed')
  ap.add_argument('seeds', nargs='+')
  a = ap.parse_args()
  root = os.path.abspath(a.root)
  shims = [os.path.abspath(s) for s in a.shim]
  if a.per_seed:
    res = {s: closure([s], root, shims) for s in a.seeds}
  else:
    res = {'+'.join(a.seeds): closure(a.seeds, root, shims)}
  with open(a.out, 'w') as f:
    json.dump(res, f, indent=1)
  for k, r in res.items():
    print(f"{k}: headers={len(r['repo_files'])} paired_cc={len(r['paired_cc'])} "
          f"external=[{','.join(r['external'])}] unresolved=[{','.join(r['unresolved'])}]")
  return 0


if __name__ == '__main__':
  sys.exit(main())
