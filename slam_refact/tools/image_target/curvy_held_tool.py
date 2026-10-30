#!/usr/bin/env python3
"""Helper of the curved target HELD device (curvy-it-gate.v3.md section 6; the C++ side is
slam_refact/engine/tests/image_target/curvy_held.h). Same algorithms, so that the coordinator can compute the values of the
designation file and the registration can pin them:

  core-sha  <manifest.json>     sha256 of the manifest with the review object and the first_real_run_at value blanked
  tree-sha  <dir>               sha256 of the sorted "<relative path>\\t<file sha256>\\n" lines of every regular file
  file-sha  <file>...           sha256 of each file (held_pins, gate file, review file)
  designation-values <manifest.json> <gate.md> <review.md>
                                the three sha256 values the designation file must carry

Only the standard library is used (development tool).
"""
import hashlib
import os
import re
import sys


def sha(b):
    return hashlib.sha256(b).hexdigest()


def _find_key(s, key, start=0):
    for m in re.finditer(r'"%s"\s*:' % re.escape(key), s[start:]):
        return start + m.start()
    return -1


def manifest_core_text(m):
    r = _find_key(m, 'review')
    if r >= 0:
        e = m.index(':', r) + 1
        while e < len(m) and m[e].isspace():
            e += 1
        if m[e] == '{':
            depth = 0
            for i in range(e, len(m)):
                if m[i] == '{':
                    depth += 1
                if m[i] == '}':
                    depth -= 1
                    if depth == 0:
                        m = m[:e] + '{}' + m[i + 1:]
                        break
    f = _find_key(m, 'first_real_run_at')
    if f >= 0:
        e = m.index(':', f) + 1
        while e < len(m) and m[e].isspace():
            e += 1
        q = e
        if m[q] == '"':
            q = m.find('"', q + 1)
            q = len(m) if q < 0 else q + 1
        else:
            while q < len(m) and m[q] not in ',}\n':
                q += 1
        m = m[:e] + 'null' + m[q:]
    return m


def tree_sha(d):
    files = []
    for root, _, names in os.walk(d):
        for n in names:
            p = os.path.join(root, n)
            if os.path.isfile(p):
                files.append(os.path.relpath(p, d))
    files.sort()
    if not files:
        return ''
    lines = ''.join('%s\t%s\n' % (f, sha(open(os.path.join(d, f), 'rb').read())) for f in files)
    return sha(lines.encode())


def main(argv):
    cmd = argv[0]
    if cmd == 'core-sha':
        print(sha(manifest_core_text(open(argv[1], encoding='utf-8').read()).encode()))
    elif cmd == 'tree-sha':
        print(tree_sha(argv[1]))
    elif cmd == 'file-sha':
        for f in argv[1:]:
            print(sha(open(f, 'rb').read()), f)
    elif cmd == 'designation-values':
        print('manifest_core_sha256', sha(manifest_core_text(open(argv[1], encoding='utf-8').read()).encode()))
        print('gate_file_sha256', sha(open(argv[2], 'rb').read()))
        print('review_file_sha256', sha(open(argv[3], 'rb').read()))
    else:
        raise SystemExit(__doc__)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
