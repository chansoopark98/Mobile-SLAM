#!/usr/bin/env python3
"""Compute Bazel cc_library dependency closures for the public xr_reference/xr_reference tree.

The BUILD files are Starlark; we evaluate them with permissive Python stubs (cc_library etc.
record their arguments, everything else is a no-op). select() branches are unioned, so the
closure is an upper bound across platforms. Only //c8/... and //reality/... and //third_party
labels are followed; '@repo' labels are reported as external deps.

Usage:
  bazel_closure.py --root <xr_reference repo> --out <json> //pkg:target [//pkg:target ...]
"""

import argparse
import json
import os
import sys


class _Anything:
  """Stub object: any attribute is an empty list (e.g. SIMD.NEON)."""

  def __getattr__(self, _name):
    return []

  def __call__(self, *args, **kwargs):
    return None


class _UnionList(list):
  pass


def _select(branches, **_kwargs):
  out = _UnionList()
  for value in branches.values():
    if isinstance(value, list):
      out.extend(value)
  return out


def load_package(root, pkg, cache):
  if pkg in cache:
    return cache[pkg]
  rules = {}
  path = os.path.join(root, pkg, 'BUILD')
  if not os.path.exists(path):
    path = os.path.join(root, pkg, 'BUILD.bazel')
  if not os.path.exists(path):
    cache[pkg] = None
    return None

  def make_rule(kind):
    def rule(**kwargs):
      name = kwargs.get('name')
      if name:
        rules[name] = {'kind': kind, **{k: v for k, v in kwargs.items() if k != 'name'}}
    return rule

  class Namespace(dict):
    def __missing__(self, key):
      if key in ('cc_library', 'cc_test', 'cc_binary', 'objc_library'):
        return make_rule(key)
      if key.endswith(('_library', '_binary', '_test', 'filegroup', '_data', '_include', 'genrule')):
        return make_rule(key)
      return _Anything()

  ns = Namespace()
  ns.update({
      'load': lambda *a, **k: None,
      'package': lambda *a, **k: None,
      'licenses': lambda *a, **k: None,
      'exports_files': lambda *a, **k: None,
      'select': _select,
      'glob': lambda *a, **k: [],
      'True': True, 'False': False, 'None': None,
  })
  with open(path) as f:
    src = f.read()
  try:
    exec(compile(src, path, 'exec'), ns)  # noqa: S102 (trusted local BUILD read as data model)
  except Exception as e:  # pragma: no cover - reported in output
    rules['__error__'] = {'kind': 'error', 'error': repr(e)}
  cache[pkg] = rules
  return rules


def parse_label(label, cur_pkg):
  if label.startswith('@'):
    return ('external', label)
  if label.startswith(':'):
    return (cur_pkg, label[1:])
  if label.startswith('//'):
    body = label[2:]
    if ':' in body:
      pkg, name = body.split(':', 1)
    else:
      pkg, name = body, body.rsplit('/', 1)[-1]
    return (pkg, name)
  return (cur_pkg, label)


def closure(root, seeds):
  cache = {}
  seen = set()
  externals = {}
  files = []
  missing = []
  order = []
  stack = [parse_label(s, '') for s in seeds]
  parents = {}
  while stack:
    pkg, name = stack.pop()
    key = f'//{pkg}:{name}'
    if key in seen:
      continue
    seen.add(key)
    if pkg == 'external':
      continue
    if pkg.startswith('bzl/inliner'):
      continue
    rules = load_package(root, pkg, cache)
    if rules is None or name not in rules:
      missing.append(key)
      continue
    rule = rules[name]
    order.append(key)
    for attr in ('srcs', 'hdrs', 'textual_hdrs'):
      for f in rule.get(attr, []) or []:
        if isinstance(f, str) and not f.startswith((':', '//', '@')):
          files.append(os.path.join(pkg, f))
    for dep in rule.get('deps', []) or []:
      if not isinstance(dep, str):
        continue
      dpkg, dname = parse_label(dep, pkg)
      if dpkg == 'external':
        externals.setdefault(dname, set()).add(key)
        continue
      dkey = f'//{dpkg}:{dname}'
      parents.setdefault(dkey, key)
      stack.append((dpkg, dname))
  return {
      'seeds': seeds,
      'targets': sorted(order),
      'files': sorted(set(files)),
      'external': {k: sorted(v) for k, v in sorted(externals.items())},
      'unresolved': sorted(missing),
  }


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--root', required=True)
  ap.add_argument('--out', required=True)
  ap.add_argument('labels', nargs='+')
  args = ap.parse_args()
  result = {}
  for label in args.labels:
    result[label] = closure(args.root, [label])
  with open(args.out, 'w') as f:
    json.dump(result, f, indent=1)
  for label, r in result.items():
    ext = ','.join(r['external'].keys())
    print(f"{label}: targets={len(r['targets'])} files={len(r['files'])} external=[{ext}] "
          f"unresolved={len(r['unresolved'])}")
  return 0


if __name__ == '__main__':
  sys.exit(main())
