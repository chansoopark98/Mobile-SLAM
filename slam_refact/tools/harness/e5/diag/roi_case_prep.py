#!/usr/bin/env python3
"""Cases for roi_model_test: python3 -I roi_case_prep.py <run dir with runs/e5-seq-a/o1/glcap> <frame> <out dir>
Writes <out>/f<frame>-slot<i>.raw (the original's R plane of each non-clear ROI slot of capture <frame>) and <out>/case-f<frame>.txt (viewport c r w h, the 16 floats of the
`mvp` uniform of that slot's quad draw in the stage call S<frame+1>, raw file).  The camera image of capture <frame> is PGM <frame>-1 (glue deferral)."""
import gzip, json, sys, os
import numpy as np
run, frame, out = sys.argv[1], int(sys.argv[2]), sys.argv[3]
os.makedirs(out, exist_ok=True)
ops = json.load(open(f'{run}/runs/e5-seq-a/o1/glcap/glcap.json'))['ops']
S, cur, found = f'S{frame + 1}', None, []
for o in ops:
    if o[0] != S:
        continue
    if o[1] == 'viewport':
        cur = o[2:6]
    if o[1] == 'uniformMatrix4fv' and o[4] != [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1] and cur and cur[1] == 650:
        found.append((cur, o[4]))
cap = np.frombuffer(gzip.open(f'{run}/runs/e5-seq-a/o1/glcap/cap-{frame:03d}.rgba.gz').read(), dtype=np.uint8).reshape(1024, 1024, 4)
lines = []
for si, (vp, m) in enumerate(found):
    c, r, w, h = vp
    open(f'{out}/f{frame}-slot{si}.raw', 'wb').write(cap[r:r + h, c:c + w, 0].tobytes())
    lines.append(' '.join([str(v) for v in [c, r, w, h]] + ['%.9g' % v for v in m] + [f'{out}/f{frame}-slot{si}.raw']))
open(f'{out}/case-f{frame}.txt', 'w').write('\n'.join(lines) + '\n')
