#!/usr/bin/env python3
"""List every source and header that went into a ninja target, with its sha256 (orient gate v2, review m-4: the held binary links
uncommitted code of other lanes, so a result proves that binary only; this list is its provenance).
  link_inputs.py <build dir> <target> > inputs.sha256
Uses `ninja -t commands` (the compile commands of the target's closure: the -c source of each) and `ninja -t deps <object>` (the headers
the compiler read). Paths are made relative to the repo root when they are inside it; system headers (/usr) are listed too."""
import hashlib, os, re, shlex, subprocess, sys
build, target = sys.argv[1], sys.argv[2]
root = os.getcwd()
cmds = subprocess.run(['ninja', '-C', build, '-t', 'commands', target], capture_output=True, text=True, check=True).stdout.splitlines()
files = set()
objs = []
for c in cmds:
    t = shlex.split(c)
    if '-c' in t and '-o' in t:
        src = t[t.index('-c') + 1]
        files.add(os.path.normpath(os.path.join(build, src)) if not os.path.isabs(src) else src)
        objs.append(t[t.index('-o') + 1])
for o in objs:
    out = subprocess.run(['ninja', '-C', build, '-t', 'deps', o], capture_output=True, text=True).stdout.splitlines()
    for l in out[1:]:
        l = l.strip()
        if l:
            files.add(os.path.normpath(l if os.path.isabs(l) else os.path.join(build, l)))
rows = []
for f in sorted(files):
    if os.path.isfile(f):
        rows.append('%s  %s' % (hashlib.sha256(open(f, 'rb').read()).hexdigest(), os.path.relpath(f, root) if f.startswith(root) else f))
print('\n'.join(rows))
print('# files=%d compile_commands=%d' % (len(rows), len(objs)), file=sys.stderr)
