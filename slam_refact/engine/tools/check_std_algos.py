#!/usr/bin/env python3
"""Find standard-library constructs whose results differ between libc++ and other libraries
(architecture.md §5.3 "감시").

The original binary was built with emscripten libc++. Unstable sorts, selection, heaps,
shuffles, distributions and unordered-container iteration can give different orders or values
on another standard library, so every use must either go through slam_refact::libcxx or be
listed in tools/std_algos_allowlist.tsv with a reason.

Scanned: engine src/, include/, wasm/ and third_party/ (xr_reference, xr_shim). Not scanned:
third_party/libcxx_compat (the implementation itself) and tests/ (tests may use std:: to build
expectations; pass --include-tests to scan them too).

Allowlist (tab-separated; '#' comments):
  path:line   construct   reason   reviewer   [note]
  reason in: value-only | lookup-only | order-irrelevant | ported | unreachable
  ('unreachable' = code that is never instantiated or called in the engine build; see note.)
Every finding must have an entry and every entry must match a finding (stale entries fail, so
line numbers stay honest).

Exit 0 = clean, 1 = unlisted findings or stale entries, 2 = usage error.
"""
import argparse
import os
import re
import sys

ENGINE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
DEFAULT_ROOTS = ['src', 'include', 'wasm', 'third_party/xr_source', 'third_party/xr_shim']
EXTS = ('.h', '.hpp', '.hh', '.cc', '.cpp', '.cxx', '.inc')
REASONS = {'value-only', 'lookup-only', 'order-irrelevant', 'ported', 'unreachable'}

PATTERNS = [
    ('std::sort', r'\bstd::sort\s*\('),
    ('std::nth_element', r'\bstd::nth_element\s*\('),
    ('std::partial_sort', r'\bstd::partial_sort(_copy)?\s*\('),
    ('std::shuffle', r'\bstd::(random_)?shuffle\s*\('),
    ('std::*_heap', r'\bstd::(make|push|pop|sort)_heap\s*\('),
    ('std::priority_queue', r'\bstd::priority_queue\b'),
    ('*_distribution<', r'(?<!libcxx::)\b\w+_distribution\s*<'),
    ('generate_canonical', r'(?<!libcxx::)\bgenerate_canonical\s*<'),
    ('std::unordered_*', r'\bstd::unordered_(map|set|multimap|multiset)\b'),
    ('HashMap/HashSet', r'\bHash(Map|Set)\s*<'),
    ('rand()', r'(\bstd::rand\s*\(|(?<![\w:.>])rand\s*\(|\bsrand\s*\()'),
    ('std::random_device', r'\bstd::random_device\b'),
]
COMPILED = [(name, re.compile(rx)) for name, rx in PATTERNS]


def strip_comments(text):
  """Blank out comments and string literals, keeping line structure."""
  out = []
  i, n = 0, len(text)
  while i < n:
    c = text[i]
    if text.startswith('//', i):
      j = text.find('\n', i)
      j = n if j < 0 else j
      i = j
    elif text.startswith('/*', i):
      j = text.find('*/', i + 2)
      j = n if j < 0 else j + 2
      out.append(''.join('\n' if ch == '\n' else ' ' for ch in text[i:j]))
      i = j
    elif c == '"':
      j = i + 1
      while j < n and text[j] != '"':
        j += 2 if text[j] == '\\' else 1
      out.append('""')
      i = j + 1
    elif c == "'" and i + 2 < n and (text[i + 1] != '\\' and text[i + 2] == "'"):
      out.append("' '")
      i += 3
    else:
      out.append(c)
      i += 1
  return ''.join(out)


def scan(roots):
  findings = []
  for root in roots:
    base = os.path.join(ENGINE, root)
    if not os.path.isdir(base):
      continue
    for d, _, files in os.walk(base):
      for f in sorted(files):
        if not f.endswith(EXTS):
          continue
        path = os.path.join(d, f)
        rel = os.path.relpath(path, ENGINE)
        with open(path, errors='replace') as fh:
          lines = strip_comments(fh.read()).split('\n')
        for ln, line in enumerate(lines, 1):
          for name, rx in COMPILED:
            if rx.search(line):
              findings.append(('%s:%d' % (rel, ln), name))
  return findings


def load_allowlist(path):
  entries = {}
  errors = []
  with open(path) as f:
    for no, line in enumerate(f, 1):
      line = line.rstrip('\n')
      if not line.strip() or line.startswith('#'):
        continue
      cols = line.split('\t')
      if len(cols) < 4:
        errors.append('allowlist line %d: need path:line, construct, reason, reviewer' % no)
        continue
      loc, construct, reason, reviewer = cols[:4]
      if reason not in REASONS:
        errors.append('allowlist line %d: reason %r not in %s' % (no, reason, sorted(REASONS)))
      if not reviewer.strip():
        errors.append('allowlist line %d: empty reviewer' % no)
      entries[(loc, construct)] = no
  return entries, errors


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--allowlist', default=os.path.join(ENGINE, 'tools', 'std_algos_allowlist.tsv'))
  ap.add_argument('--include-tests', action='store_true')
  ap.add_argument('--list', action='store_true', help='print every finding (allowlisted or not)')
  a = ap.parse_args()
  roots = DEFAULT_ROOTS + (['tests'] if a.include_tests else [])
  findings = scan(roots)
  allow, errors = load_allowlist(a.allowlist)
  seen = set()
  unlisted = []
  for loc, name in findings:
    key = (loc, name)
    if key in allow:
      seen.add(key)
    else:
      unlisted.append(key)
    if a.list:
      print('%s\t%s\t%s' % (loc, name, 'allowlisted' if key in allow else 'UNLISTED'))
  scanned = tuple(r.rstrip('/') + '/' for r in roots)
  stale = [k for k in allow if k not in seen and k[0].startswith(scanned)]
  for e in errors:
    print('ERROR ' + e)
  for loc, name in unlisted:
    print('UNLISTED %s\t%s' % (loc, name))
  for loc, name in sorted(stale):
    print('STALE    %s\t%s (allowlist line %d)' % (loc, name, allow[(loc, name)]))
  ok = not (errors or unlisted or stale)
  print('check_std_algos: %d findings, %d allowlisted, %d unlisted, %d stale -> %s' %
        (len(findings), len(seen), len(unlisted), len(stale), 'PASS' if ok else 'FAIL'))
  return 0 if ok else 1


if __name__ == '__main__':
  sys.exit(main())
