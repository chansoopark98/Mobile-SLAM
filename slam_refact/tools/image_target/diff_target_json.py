#!/usr/bin/env python3
"""Compare two image target JSON files field by field (US-012).

Typical use: the JSON of image_target_cli.py against the JSON the public Node CLI writes for the
same answers (`cat test/<case>.txt | node src/index.js`). Key order, value types and values are
compared recursively; `created` and `updated` are ignored unless --strict-time is given.
Numbers are equal when |a - b| <= --abs-tol (default 0: exact).

The default comparison parses both files, so `1` and `1.0`, or `1e-7` and `1e-07`, are the same
number. --text additionally compares the files as text, line by line, with the values of
`created` / `updated` blanked (unless --strict-time): the number notation, indentation and key
spelling must match what JSON.stringify(data, null, 2) writes.

Exit status: 0 = equal, 1 = differences (listed on stdout), 2 = usage or read error.
"""
import argparse
import json
import re
import sys

IGNORED_BY_DEFAULT = ('created', 'updated')


def diff(a, b, path, abs_tol, out):
  if isinstance(a, dict) and isinstance(b, dict):
    if list(a.keys()) != list(b.keys()):
      out.append('%s: keys %s != %s' % (path or '/', list(a.keys()), list(b.keys())))
    for k in a:
      if k in b:
        diff(a[k], b[k], '%s/%s' % (path, k), abs_tol, out)
    return
  if isinstance(a, list) and isinstance(b, list):
    if len(a) != len(b):
      out.append('%s: length %d != %d' % (path, len(a), len(b)))
    for i, (x, y) in enumerate(zip(a, b)):
      diff(x, y, '%s[%d]' % (path, i), abs_tol, out)
    return
  num = (int, float)
  if (isinstance(a, num) and isinstance(b, num) and not isinstance(a, bool)
      and not isinstance(b, bool)):
    if abs(a - b) > abs_tol:
      out.append('%s: %r != %r' % (path, a, b))
    return
  if type(a) is not type(b) or a != b:
    out.append('%s: %r != %r' % (path, a, b))


def compare(a, b, abs_tol=0.0, strict_time=False):
  """Returns the list of differences between two parsed target JSON objects."""
  if not strict_time and isinstance(a, dict) and isinstance(b, dict):
    a = {k: v for k, v in a.items() if k not in IGNORED_BY_DEFAULT}
    b = {k: v for k, v in b.items() if k not in IGNORED_BY_DEFAULT}
  out = []
  diff(a, b, '', abs_tol, out)
  return out


_TIME_LINE = re.compile(r'^(\s*"(?:created|updated)": )[^,\n]*(,?)$', re.M)


def compare_text(a_text, b_text, strict_time=False):
  """Line-by-line text differences of two target JSON files (number notation included)."""
  def normalize(text):
    text = text.replace('\r\n', '\n')
    return text if strict_time else _TIME_LINE.sub(r'\1<time>\2', text)

  a_lines = normalize(a_text).split('\n')
  b_lines = normalize(b_text).split('\n')
  out = []
  if len(a_lines) != len(b_lines):
    out.append('text: %d lines != %d lines' % (len(a_lines), len(b_lines)))
  for i, (x, y) in enumerate(zip(a_lines, b_lines)):
    if x != y:
      out.append('text line %d: %s != %s' % (i + 1, x.strip(), y.strip()))
  return out


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
  ap.add_argument('a')
  ap.add_argument('b')
  ap.add_argument('--abs-tol', type=float, default=0.0)
  ap.add_argument('--strict-time', action='store_true')
  ap.add_argument('--text', action='store_true',
                  help='also compare the files as text (number notation, layout)')
  args = ap.parse_args(argv)
  try:
    with open(args.a, encoding='utf-8') as f:
      a_text = f.read()
    with open(args.b, encoding='utf-8') as f:
      b_text = f.read()
    a = json.loads(a_text)
    b = json.loads(b_text)
  except (OSError, ValueError) as e:
    print('error: %s' % e, file=sys.stderr)
    return 2
  differences = compare(a, b, args.abs_tol, args.strict_time)
  if args.text:
    differences += compare_text(a_text, b_text, args.strict_time)
  for d in differences:
    print(d)
  print('diff_target_json: %s' % ('EQUAL' if not differences else '%d differences' % len(differences)))
  return 0 if not differences else 1


if __name__ == '__main__':
  sys.exit(main())
