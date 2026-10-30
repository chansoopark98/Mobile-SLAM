#!/usr/bin/env python3
"""Compare public-source config defaults (globalParams().getOrSet("Key", default)) with the
compiled defaults recovered from the xr-slam WASM (config-defaults.json from the REA lane).

Usage: config_parity.py <xr_reference repo root> <config-defaults.json> <out.json>
Prints a TSV summary: key, public default, binary default(s), verdict.
"""
import json
import os
import re
import sys

CALL_RE = re.compile(r'getOrSet(?:<[^>]*>)?\(\s*"([A-Z][A-Za-z0-9]*\.[A-Za-z0-9]+)"\s*,\s*([^)]*?)\)',
                     re.S)


def parse_literal(text):
  t = text.strip()
  if t in ('true', 'false'):
    return 1 if t == 'true' else 0
  enum = {'DescriptorType::ORB': 1, 'DescriptorType::GORB': 2}
  if t in enum:
    return enum[t]
  t2 = t.rstrip('fF')
  try:
    return int(t2, 0)
  except ValueError:
    pass
  try:
    return float(t2)
  except ValueError:
    return None


def main():
  root, defaults_path, out_path = sys.argv[1:4]
  public = {}
  for sub in ('reality/engine', 'c8'):
    for dp, _, files in os.walk(os.path.join(root, sub)):
      for fn in files:
        if not fn.endswith(('.cc', '.h')) or fn.endswith('-test.cc'):
          continue
        p = os.path.join(dp, fn)
        src = open(p, errors='replace').read()
        for m in CALL_RE.finditer(src):
          line = src[:m.start()].count('\n') + 1
          public.setdefault(m.group(1), []).append(
            {'file': os.path.relpath(p, root), 'line': line, 'raw': m.group(2).strip(),
             'value': parse_literal(m.group(2))})
  binary = {}
  for e in json.load(open(defaults_path))['entries']:
    vals = [a[1] for a in e['args'] if a[0] in ('i32', 'i64', 'f32', 'f64')]
    binary.setdefault(e['key'], []).append({'func': e['func'], 'value': vals[0] if vals else None})
  rows = []
  for key in sorted(public):
    pv = public[key][0]['value']
    bvs = [b['value'] for b in binary.get(key, [])]
    if not bvs:
      verdict = 'NOT_IN_BINARY_TABLE'
    elif pv is None:
      verdict = 'PUBLIC_NON_LITERAL'
    elif all(abs(float(b) - float(pv)) <= 1e-6 * max(1.0, abs(float(pv))) for b in bvs):
      verdict = 'MATCH'
    else:
      verdict = 'DIFFER'
    rows.append({'key': key, 'public': public[key], 'binary': binary.get(key, []),
                 'verdict': verdict})
  json.dump(rows, open(out_path, 'w'), indent=1)
  for r in rows:
    print('%s\t%s\t%s\t%s' % (r['key'], r['public'][0]['raw'],
                              ','.join(str(b['value']) for b in r['binary']) or '-', r['verdict']))
  counts = {}
  for r in rows:
    counts[r['verdict']] = counts.get(r['verdict'], 0) + 1
  print('summary', counts, 'binary_keys', len(binary))


if __name__ == '__main__':
  main()
