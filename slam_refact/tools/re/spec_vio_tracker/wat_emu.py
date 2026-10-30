#!/usr/bin/env python3
"""Partial emulator for a slice of a `wasm-objdump -d` disassembly of ONE function.

Purpose: decode inlined float math (quaternion / HMatrix algebra) in very large functions such as
func784 (VioTracker::track) by running the real instructions on chosen inputs and comparing the
outputs with candidate formulas (see probe_func784.py). It is not a full wasm VM.

Model (documented so results are not over-read):
  * Addresses are symbolic linear expressions (sympy) in the function's parameters and the
    stack frame pointer, e.g. `b + 7720`, `G0 - 3168 + 2080`. Memory is a dict keyed by
    (canonical address string, kind). Loads of never-stored addresses must come from `init`
    (same keys) or raise KeyError, so every input is explicit.
  * Float values are Python floats; f32 results are rounded with struct round-trips (float32
    parity), f64 is native.
  * v128 is a tuple of 4 f32 lanes (f32x4) or 2 f64 lanes (f64x2); i8x16.shuffle supports
    4-byte aligned lane moves only.
  * Control flow: block/loop/end are structural; br_if/br/select evaluate concrete conditions.
    `jumps={A: B}` skips code (e.g. heap/vector management).
  * Calls: a fixed table of pure helpers implemented natively with the public c8 semantics
    (sin/cos/atan2, the config getter returns its default or an override,
    groundPlaneRotationRads, Quaternion::fromHMatrix, cameraMotion, memcpy). Anything else aborts.
Stdlib + sympy. Run with python3 -I.
"""
import math
import re
import struct

import sympy as sp

LINE_RE = re.compile(r'^\s*([0-9a-f]+):\s+(?:[0-9a-f]{2} )+\s*\|\s*(\S.*)$')


def _i(x):
    return int(x)


INT_BINOPS = {
    'i32.and': lambda a, b: _i(a) & _i(b), 'i32.or': lambda a, b: _i(a) | _i(b),
    'i32.xor': lambda a, b: _i(a) ^ _i(b), 'i32.shl': lambda a, b: (_i(a) << (_i(b) & 31)) & 0xFFFFFFFF,
    'i32.shr_s': lambda a, b: _i(a) >> (_i(b) & 31), 'i32.shr_u': lambda a, b: (_i(a) & 0xFFFFFFFF) >> (_i(b) & 31),
    'i32.eq': lambda a, b: int(a == b), 'i32.ne': lambda a, b: int(a != b),
    'i64.eq': lambda a, b: int(a == b), 'i64.ne': lambda a, b: int(a != b),
    'i32.lt_s': lambda a, b: int(a < b), 'i32.lt_u': lambda a, b: int(a < b),
    'i32.gt_s': lambda a, b: int(a > b), 'i32.gt_u': lambda a, b: int(a > b),
    'i32.le_s': lambda a, b: int(a <= b), 'i32.ge_s': lambda a, b: int(a >= b),
    'i32.ge_u': lambda a, b: int(a >= b), 'i64.lt_s': lambda a, b: int(a < b),
    'i32.div_s': lambda a, b: int(a / b), 'i32.rem_u': lambda a, b: a % b,
}


def f32(x):
    if isinstance(x, float) and (math.isinf(x) or math.isnan(x)):
        return x
    return struct.unpack('<f', struct.pack('<f', x))[0]


def parse(path):
    out = []
    for ln in open(path):
        m = LINE_RE.match(ln)
        if m:
            out.append((int(m.group(1), 16), m.group(2).strip()))
    return out


class Emu:
    def __init__(self, lines, params, init=None, param_overrides=None):
        self.lines = lines
        self.addr_index = {a: i for i, (a, _) in enumerate(lines)}
        self.locals = {i: sp.Symbol(n) for i, n in enumerate(params)}
        self.stack = []
        self.bytes = {}
        self.objs = {}
        for (a_, kind_), v_ in (init or {}).items():
            self.store(sp.sympify(a_), kind_, v_)
        self.param_overrides = param_overrides or {}
        self.block_end = {}
        st = []
        for i, (_, ins) in enumerate(lines):
            op = ins.split()[0]
            if op in ('block', 'loop', 'if'):
                st.append(i)
            elif op == 'end' and st:
                self.block_end[st.pop()] = i
        self.calls = []

    # ---------- memory (byte-addressed per symbolic base) ----------
    FMT = {'f32': ('<f', 4), 'f64.load': ('<d', 8), 'i32.load': ('<i', 4), 'i64.load': ('<q', 8),
           'i32.load8_u': ('<B', 1), 'i32.load8_s': ('<b', 1), 'i32.load16_u': ('<H', 2),
           'i32.load16_s': ('<h', 2), 'i64.load32_u': ('<I', 4), 'i64.load32_s': ('<i', 4)}
    STORE_KIND = {'i32.load8': 'i32.load8_u', 'i32.load16': 'i32.load16_u', 'i64.load8': 'i32.load8_u',
                  'i64.load16': 'i32.load16_u', 'i64.load32': 'i64.load32_u'}

    @staticmethod
    def split(addr):
        e = sp.expand(sp.sympify(addr))
        const = int(e.as_coeff_Add()[0]) if e.as_coeff_Add()[0].is_number else 0
        base = sp.expand(e - const)
        return str(base), const

    @staticmethod
    def key(addr):
        return str(sp.expand(addr))

    def load(self, addr, kind):
        base, off = self.split(addr)
        fmt, size = self.FMT[kind]
        obj = self.objs.get((base, off))
        if obj is not None and obj[1] == size:
            return obj[0]
        raw = bytearray()
        for k in range(size):
            b = self.bytes.get((base, off + k))
            if b is None:
                raise KeyError(f'uninitialised load {(base, off, kind)}')
            raw.append(b)
        v = struct.unpack(fmt, bytes(raw))[0]
        return f32(v) if kind == 'f32' else v

    def store(self, addr, kind, val):
        kind = self.STORE_KIND.get(kind, kind)
        base, off = self.split(addr)
        fmt, size = self.FMT[kind]
        for k in range(size):
            self.objs.pop((base, off + k), None)
            self.bytes.pop((base, off + k), None)
        if isinstance(val, (int, float)) and not isinstance(val, bool) or isinstance(val, bool):
            v = val
            if fmt in ('<i', '<I', '<B', '<b', '<H', '<h', '<q'):
                v = int(v)
                bits = 8 * size
                v &= (1 << bits) - 1
                ufmt = {1: '<B', 2: '<H', 4: '<I', 8: '<Q'}[size]
                raw = struct.pack(ufmt, v)
            else:
                raw = struct.pack(fmt, v)
            for k in range(size):
                self.bytes[(base, off + k)] = raw[k]
        else:
            self.objs[(base, off)] = (val, size)

    def lf(self, base, off):
        return self.load(base + off, 'f32')

    # ---------- helpers ----------
    def pop(self):
        return self.stack.pop()

    def popn(self, n):
        if n == 0:
            return []
        v = self.stack[-n:]
        del self.stack[-n:]
        return v

    def enclosing(self, i, depth):
        st = 0
        for j in range(i - 1, -1, -1):
            op = self.lines[j][1].split()[0]
            if op == 'end':
                st += 1
            elif op in ('block', 'loop', 'if'):
                if st:
                    st -= 1
                elif depth == 0:
                    return j
                else:
                    depth -= 1
        raise RuntimeError('no enclosing block')

    def branch_target(self, i, depth):
        tgt = self.enclosing(i, depth)
        if self.lines[tgt][1].split()[0] == 'loop':
            return tgt + 1
        return self.block_end[tgt] + 1

    # ---------- native helper calls ----------
    def call(self, fn, addr):
        self.calls.append((hex(addr), fn))
        if fn == 850:
            return None
        if fn == 458:
            key_addr, default = self.popn(2)
            return self.param_overrides.get(int(key_addr), default)
        if fn in (461, 462):
            x = self.pop()
            return f32(math.sin(x) if fn == 461 else math.cos(x))
        if fn == 897:
            y, x = self.popn(2)
            return math.atan2(y, x)
        if fn in (898, 899):
            x = self.pop()
            return math.sin(x) if fn == 898 else math.cos(x)
        if fn == 279:  # memcpy(dst, src, n): byte copy (objects copied when aligned)
            dst, src, n = self.popn(3)
            db, do = self.split(dst)
            sb, so = self.split(src)
            for k in range(int(n)):
                self.bytes.pop((db, do + k), None)
                self.objs.pop((db, do + k), None)
                b = self.bytes.get((sb, so + k))
                if b is not None:
                    self.bytes[(db, do + k)] = b
                o = self.objs.get((sb, so + k))
                if o is not None:
                    self.objs[(db, do + k)] = o
            return dst
        if fn == 519:  # groundPlaneRotationRads(Quaternion*) -- public egomotion.cc:194
            p = self.pop()
            q = [self.lf(p, 4 * k) for k in range(4)]  # x y z w
            return f32(ground_plane_rotation_rads(q))
        if fn == 520:  # Quaternion::fromHMatrix -> out (x,y,z,w)
            out, m = self.popn(2)
            mat = [self.lf(m, 4 * k) for k in range(16)]
            q = quat_from_colmajor(mat)
            for k in range(4):
                self.store(out + 4 * k, 'f32', f32(q[k]))
            return None
        if fn == 521:  # cameraMotion(HVector3*, Quaternion*) -> HMatrix out (col-major + inverse)
            out, t, qp = self.popn(3)
            tv = [self.lf(t, 4 * k) for k in range(3)]
            q = [self.lf(qp, 4 * k) for k in range(4)]
            m, inv = camera_motion(tv, q)
            for k in range(16):
                self.store(out + 4 * k, 'f32', f32(m[k]))
                self.store(out + 64 + 4 * k, 'f32', f32(inv[k]))
            return None
        raise RuntimeError(f'unsupported call {fn} at {hex(addr)}')

    # ---------- interpreter ----------
    def run(self, start, end, jumps=None, max_steps=500000):
        jumps = jumps or {}
        i = self.addr_index[start]
        end_i = self.addr_index[end]
        steps = 0
        while i < end_i:
            steps += 1
            if steps > max_steps:
                raise RuntimeError('step limit')
            addr, ins = self.lines[i]
            if addr in jumps:
                i = self.addr_index[jumps[addr]]
                continue
            parts = ins.split()
            op, args = parts[0], parts[1:]
            nxt = i + 1
            s = self.stack
            if op in ('block', 'loop', 'end', 'nop'):
                pass
            elif op == 'local.get':
                k = int(args[0])
                s.append(self.locals[k] if k in self.locals else sp.Symbol(f'L{k}'))
            elif op == 'local.set':
                self.locals[int(args[0])] = self.pop()
            elif op == 'local.tee':
                self.locals[int(args[0])] = s[-1]
            elif op in ('i32.const', 'i64.const'):
                s.append(int(args[0]))
            elif op == 'f32.const':
                s.append(f32(float.fromhex(args[0])))
            elif op == 'f64.const':
                s.append(float.fromhex(args[0]))
            elif op == 'v128.const':
                s.append(tuple(struct.unpack('<f', struct.pack('<I', int(x, 16)))[0] for x in args[:4]))
            elif op in ('i32.add', 'i64.add'):
                b, a = self.pop(), self.pop()
                s.append(a + b)
            elif op in ('i32.sub', 'i64.sub'):
                b, a = self.pop(), self.pop()
                s.append(a - b)
            elif op == 'i32.mul':
                b, a = self.pop(), self.pop()
                s.append(a * b)
            elif op in ('f32.add', 'f32.sub', 'f32.mul', 'f32.div', 'f64.add', 'f64.sub', 'f64.mul', 'f64.div'):
                b, a = self.pop(), self.pop()
                kind = op[4:]
                if kind == 'div':
                    r = a / b if b != 0 else (math.copysign(math.inf, a) if a != 0 else math.nan)
                else:
                    r = {'add': a + b, 'sub': a - b, 'mul': a * b}[kind]
                s.append(f32(r) if op.startswith('f32') else r)
            elif op in ('f32.neg', 'f64.neg'):
                s.append(-self.pop())
            elif op in ('f32.sqrt', 'f64.sqrt'):
                x = self.pop()
                r = math.sqrt(x) if x >= 0 else math.nan
                s.append(f32(r) if op.startswith('f32') else r)
            elif op in ('f32.abs', 'f64.abs'):
                s.append(abs(self.pop()))
            elif op in ('f32.min', 'f32.max', 'f64.min', 'f64.max'):
                b, a = self.pop(), self.pop()
                s.append(min(a, b) if op.endswith('min') else max(a, b))
            elif op == 'f32.demote_f64':
                s.append(f32(self.pop()))
            elif op in ('f64.promote_f32', 'f64.convert_i64_s', 'f64.convert_i32_s'):
                s.append(float(self.pop()))
            elif op in ('f32.convert_i32_s', 'f32.convert_i32_u', 'f32.convert_i64_s'):
                s.append(f32(float(self.pop())))
            elif op in ('f32.lt', 'f64.lt', 'f32.gt', 'f64.gt', 'f32.le', 'f64.le', 'f32.ge', 'f64.ge',
                        'f32.eq', 'f64.eq', 'f32.ne', 'f64.ne'):
                b, a = self.pop(), self.pop()
                r = {'lt': a < b, 'gt': a > b, 'le': a <= b, 'ge': a >= b, 'eq': a == b, 'ne': a != b}[op[4:]]
                s.append(int(r))
            elif op == 'select':
                c, b, a = self.pop(), self.pop(), self.pop()
                s.append(a if c else b)
            elif op in ('i32.eqz', 'i64.eqz'):
                s.append(int(self.pop() == 0))
            elif op in INT_BINOPS:
                b, a = self.pop(), self.pop()
                s.append(INT_BINOPS[op](a, b))
            elif op.endswith('.load') or op.startswith('i32.load') or op.startswith('i64.load'):
                off = int(args[1])
                base = self.pop()
                if op == 'v128.load':
                    s.append(tuple(self.lf(base, off + 4 * k) for k in range(4)))
                else:
                    kind = 'f32' if op == 'f32.load' else op
                    s.append(self.load(base + off, kind))
            elif op.endswith('.store') or op in ('i32.store8', 'i32.store16'):
                off = int(args[1])
                val = self.pop()
                base = self.pop()
                if op == 'v128.store':
                    for k in range(4):
                        self.store(base + off + 4 * k, 'f32', val[k])
                elif op == 'f32.store':
                    self.store(base + off, 'f32', val)
                else:
                    self.store(base + off, op.replace('store', 'load'), val)
            elif op == 'f32x4.splat':
                v = self.pop()
                s.append((v, v, v, v))
            elif op == 'f64x2.splat':
                v = self.pop()
                s.append((v, v))
            elif op == 'f32x4.replace_lane':
                v = self.pop()
                vec = list(self.pop())
                vec[int(args[0])] = v
                s.append(tuple(vec))
            elif op in ('f32x4.extract_lane', 'f64x2.extract_lane', 'i32x4.extract_lane'):
                s.append(self.pop()[int(args[0])])
            elif op in ('f32x4.add', 'f32x4.sub', 'f32x4.mul', 'f32x4.div', 'f64x2.add', 'f64x2.sub', 'f64x2.mul'):
                b, a = self.pop(), self.pop()
                fn = {'add': lambda x, y: x + y, 'sub': lambda x, y: x - y, 'mul': lambda x, y: x * y,
                      'div': lambda x, y: x / y}[op.split('.')[1]]
                r = tuple(fn(x, y) for x, y in zip(a, b))
                s.append(tuple(f32(x) for x in r) if op.startswith('f32') else r)
            elif op == 'f32x4.neg':
                s.append(tuple(-x for x in self.pop()))
            elif op == 'f32x4.sqrt':
                s.append(tuple(f32(math.sqrt(x)) if x >= 0 else math.nan for x in self.pop()))
            elif op == 'f32x4.gt':
                b, a = self.pop(), self.pop()
                s.append(tuple(-1 if x > y else 0 for x, y in zip(a, b)))
            elif op == 'f64x2.promote_low_f32x4':
                a = self.pop()
                s.append((float(a[0]), float(a[1])))
            elif op == 'i8x16.shuffle':
                words = [int(x, 16) for x in args[:4]]
                idx = [(w >> (8 * k)) & 0xFF for w in words for k in range(4)]
                b, a = self.pop(), self.pop()
                both = list(a) + list(b)
                lanes = []
                for k in range(4):
                    q = idx[4 * k:4 * k + 4]
                    if q[0] % 4 or q != list(range(q[0], q[0] + 4)):
                        raise RuntimeError(f'unaligned shuffle at {hex(addr)}')
                    lanes.append(both[q[0] // 4])
                s.append(tuple(lanes))
            elif op == 'call':
                r = self.call(int(args[0]), addr)
                if r is not None:
                    s.append(r)
            elif op == 'br_if':
                c = self.pop()
                if c:
                    nxt = self.branch_target(i, int(args[0]))
            elif op == 'br':
                nxt = self.branch_target(i, int(args[0]))
            elif op == 'br_table':
                idx = int(self.pop())
                depths = [int(x) for x in args]
                d = depths[idx] if 0 <= idx < len(depths) - 1 else depths[-1]
                nxt = self.branch_target(i, d)
            elif op == 'global.get':
                s.append(sp.Symbol(f'G{args[0]}'))
            elif op in ('global.set', 'drop'):
                self.pop()
            else:
                raise RuntimeError(f'unsupported op {op} at {hex(addr)}')
            i = nxt
        return self


# ---------------- reference math (public c8 semantics) ----------------

def quat_to_rotmat(q):
    """c8 Quaternion::toRotationMat (quaternion.cc:174), q = (x, y, z, w)."""
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def quat_mul(a, b):
    """c8 Quaternion::times without the trailing normalize; (x, y, z, w) order."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_norm(q):
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q) if n > 0 else q


def ground_plane_rotation_rads(q):
    """c8 groundPlaneRotationRads (egomotion.cc:194), FLOAT_EPS = 1e-5."""
    r = quat_to_rotmat(q)
    fx, fz = r[0][2], r[2][2]
    if math.hypot(fx, fz) <= 1e-5:
        q2 = quat_norm(quat_mul(q, (-0.70710678118, 0.0, 0.0, 0.70710678118)))
        return ground_plane_rotation_rads(q2)
    return math.atan2(fx, fz)


def quat_from_colmajor(m):
    """c8 Quaternion::fromHMatrix (quaternion.cc:38) on a column-major array; returns (x,y,z,w)."""
    def R(r, c):
        return m[4 * c + r]
    tr = R(0, 0) + R(1, 1) + R(2, 2)
    if tr >= 0:
        t = math.sqrt(tr + 1.0)
        w = 0.5 * t
        t = 0.5 / t
        return ((R(2, 1) - R(1, 2)) * t, (R(0, 2) - R(2, 0)) * t, (R(1, 0) - R(0, 1)) * t, w)
    i = (0 if R(0, 0) > R(2, 2) else 2) if R(0, 0) > R(1, 1) else (1 if R(1, 1) > R(2, 2) else 2)
    j, k = (i + 1) % 3, (i + 2) % 3
    t = math.sqrt(1.0 + R(i, i) - R(j, j) - R(k, k))
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.5 * t
    t = 0.5 / t
    q[j] = (R(j, i) + R(i, j)) * t
    q[k] = (R(k, i) + R(i, k)) * t
    q[3] = (R(k, j) - R(j, k)) * t
    return tuple(q)


def camera_motion(t, q):
    """c8 cameraMotion(t, q) = T(t) * R(q), column-major matrix and its inverse."""
    r = quat_to_rotmat(q)
    m = [0.0] * 16
    inv = [0.0] * 16
    for c in range(3):
        for rr in range(3):
            m[4 * c + rr] = r[rr][c]
            inv[4 * c + rr] = r[c][rr]
    m[12], m[13], m[14], m[15] = t[0], t[1], t[2], 1.0
    for rr in range(3):
        inv[12 + rr] = -sum(r[c][rr] * t[c] for c in range(3))
    inv[15] = 1.0
    return m, inv
