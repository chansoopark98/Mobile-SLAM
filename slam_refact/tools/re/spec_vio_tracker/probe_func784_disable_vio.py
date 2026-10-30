#!/usr/bin/env python3
"""Decode the disableVio (tracker+4) branch of func784: pose written to tamExtrinsic (tracker+6464).

Emulates WAT 0x08bb7d..0x08bedb (dcmp 82978-83062) with tracker+4 = 1 and a random device
quaternion; checks the pre-registered hypothesis (tolerance 1e-5):
  rotation(tamExtrinsic) = R(xrRotationFromDeviceRotation(devicePose)) (and its inverse block),
  translation entries reported verbatim.
The emulation also reports the translation entries actually written.
Usage: python3 -I probe_func784_disable_vio.py --wat build/rea-analysis-20261015/xr_reference/func-784.wat.txt
"""
import argparse
import json
import os
import random
import sys

import sympy as sp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wat_emu import Emu, parse, quat_norm, camera_motion, f32  # noqa: E402
from probe_func784 import xr_from_device, rand_quat  # noqa: E402

B, C = sp.Symbol('b'), sp.Symbol('c')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wat', required=True)
    ap.add_argument('--trials', type=int, default=50)
    a = ap.parse_args()
    lines = parse(a.wat)
    rng = random.Random(7)
    worst_rot = 0.0
    written = None
    for _ in range(a.trials):
        q = rand_quat(rng)
        init = {('b + 4', 'i32.load8_u'): 1}
        for k in range(4):
            init[(f'c + {104 + 4 * k}', 'f32')] = f32(q[k])
        emu = Emu(lines, ['a', 'b', 'c', 'd', 'e', 'f'], init)
        emu.locals[173] = (0.0, 0.0, 0.0, 0.0)
        emu.run(0x08BB7D, 0x08BEDB)
        m = []
        for k in range(32):
            try:
                m.append(emu.lf(B, 6464 + 4 * k))
            except KeyError:
                m.append(None)
        exp, exp_inv = camera_motion((0.0, 0.0, 0.0), xr_from_device(q))
        rot_idx = [0, 1, 2, 4, 5, 6, 8, 9, 10]
        worst_rot = max(worst_rot, max(abs(m[k] - exp[k]) for k in rot_idx),
                        max(abs(m[16 + k] - exp_inv[k]) for k in rot_idx))
        written = [k for k in range(32) if m[k] is not None]
        trans = [m[12], m[13], m[14], m[28], m[29], m[30], m[15], m[31]]
    res = dict(max_rot_err_vs_R_xr=worst_rot, n_written_floats=len(written),
               last_translation_and_inverse_translation_and_w=trans,
               verdict='tamExtrinsic = cameraMotion((0,0,0), xrRotationFromDeviceRotation(devicePose))'
               if worst_rot <= 1e-5 and all(abs(v) == 0.0 for v in trans[:6]) else 'hypothesis rejected')
    print(json.dumps(res))


if __name__ == '__main__':
    main()
