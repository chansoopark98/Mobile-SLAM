#!/usr/bin/env python3
"""D-G5 independent SOURCE check (G6, record only): does the deployed original web path always produce a centered principal point?

  python3 -I dg5_check.py --out <dir>

Reads only assets/references/xr_reference/analysis-runtime/standalone/{xr.js,xr-slam.js}. Writes xr-slam-embedded.wasm (decoded from the
base64 literal in xr-slam.js; sha256 must equal the registered xr-slam wasm sha) and dg5-source-check.json. Checks:
  1. wasm sha256 equals 780731fe...c422;
  2. the f32.const 239.5 / 319.5 sites of the whole module and the functions that contain them; which functions reference the focal table
     (data address 296576, i32.const) ; no f64.const 239.5/319.5; no 239.5/319.5 float pair in any data segment byte run;
  3. JS side: the public XrController API object (names), and the call sites of capnp *Intrinsics setters outside their class definitions
     (none => the JS never writes pixel/camera intrinsics); the graphics intrinsics fields the JS does write (texture size, clip planes, zoom).
"""
import argparse, base64, hashlib, json, pathlib, re, struct

ROOT = pathlib.Path(__file__).resolve().parents[5]
SDK = ROOT / 'assets/references/xr_reference/analysis-runtime/standalone'
WASM_SHA = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422'


def leb(b, i):
    r = sh = 0
    while True:
        x = b[i]; i += 1; r |= (x & 0x7f) << sh; sh += 7
        if x < 0x80:
            return r, i


def sleb(v):
    out = bytearray()
    while True:
        b = v & 0x7f; v >>= 7
        if (v == 0 and not b & 0x40) or (v == -1 and b & 0x40):
            out.append(b); return bytes(out)
        out.append(b | 0x80)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); a = ap.parse_args()
    out = pathlib.Path(a.out); out.mkdir(parents=True, exist_ok=True)
    xr = (SDK / 'xr.js').read_bytes(); xs = (SDK / 'xr-slam.js').read_bytes()
    s = xs.decode('latin1')
    res = {'schema': 'rm3b-dg5-source-check-v1', 'files': {'xr.js': hashlib.sha256(xr).hexdigest(), 'xr-slam.js': hashlib.sha256(xs).hexdigest()}}
    hits = [m.start() for m in re.finditer('AGFzbQ', s)]
    i = hits[0]; j = i
    while j < len(s) and re.match(r'[A-Za-z0-9+/=]', s[j]):
        j += 1
    w = base64.b64decode(s[i:j] + '=' * ((4 - (j - i) % 4) % 4))
    (out / 'xr-slam-embedded.wasm').write_bytes(w)
    res['wasm'] = {'b64_magic_offsets_in_xr_slam_js': hits, 'bytes': len(w), 'sha256': hashlib.sha256(w).hexdigest(), 'equals_registered': hashlib.sha256(w).hexdigest() == WASM_SHA}
    # sections, imports, function bodies
    k = 8; secs = {}
    while k < len(w):
        sid = w[k]; k += 1; size, k = leb(w, k); secs[sid] = (k, size); k += size
    p, _ = secs[2]; ni, p = leb(w, p); nimp = 0
    for _ in range(ni):
        l, p = leb(w, p); p += l; l, p = leb(w, p); p += l; kind = w[p]; p += 1
        if kind == 0:
            _, p = leb(w, p); nimp += 1
        elif kind == 1:
            p += 1; fl = w[p]; p += 1; _, p = leb(w, p)
            if fl & 1: _, p = leb(w, p)
        elif kind == 2:
            fl = w[p]; p += 1; _, p = leb(w, p)
            if fl & 1: _, p = leb(w, p)
        elif kind == 3:
            p += 2
    c0, _ = secs[10]; n, q = leb(w, c0); funcs = []
    for f in range(n):
        sz, q2 = leb(w, q); funcs.append((nimp + f, q2, q2 + sz)); q = q2 + sz

    def fn_of(off):
        for idx, lo, hi in funcs:
            if lo <= off < hi: return idx
        return None
    f32 = lambda v: b'\x43' + struct.pack('<f', v)
    sites = {str(v): [(m.start(), fn_of(m.start())) for m in re.finditer(re.escape(f32(v)), w)] for v in (239.5, 319.5)}
    tab = b'\x41' + sleb(296576)
    res['wasm'].update({'imported_functions': nimp, 'defined_functions': n,
        'f32_const_sites_offset_function': sites,
        'functions_containing_239.5': sorted({f for _, f in sites['239.5']}), 'functions_containing_319.5': sorted({f for _, f in sites['319.5']}),
        'focal_table_address': 296576, 'focal_table_i32_const_sites_offset_function': [(m.start(), fn_of(m.start())) for m in re.finditer(re.escape(tab), w)],
        'f64_const_239.5_319.5': [m.start() for v in (239.5, 319.5) for m in re.finditer(re.escape(b'\x44' + struct.pack('<d', v)), w)],
        'float_pair_239.5_319.5_in_module_bytes': [m.start() for m in re.finditer(re.escape(struct.pack('<ff', 239.5, 319.5)), w)],
        'note': 'function indices count the imported functions first (the Ghidra func509 of spec 01 section 5.12 is this numbering)'})
    api = re.search(r'return G\.then\(\(\(\)=>\(\{pipelineModule:WA,[^}]*\}\)', s)
    res['js'] = {'public_xrcontroller_api': api.group(0)[:400] if api else None}
    calls = [(m.start(), s[max(0, m.start() - 50):m.end() + 60].replace('\n', ' ')) for m in re.finditer(r'(?<![A-Za-z_$])[A-Za-z_$][\w$]*\.(init|set|adopt)(Pixel|Camera|Graphics|InverseCamera)?Intrinsics?\(', s)]
    res['js']['intrinsics_setter_call_sites_outside_class_definitions'] = calls
    g = s.index('getGraphicsIntrinsics();B.setTextureWidth')
    res['js']['graphics_intrinsics_fields_written_by_configureXr'] = s[g:g + 220]
    res['js']['xr_js_intrinsics_names'] = sorted(set(re.findall(r'[A-Za-z_]*[Ii]ntrinsic[A-Za-z_]*', xr.decode('latin1'))))
    (out / 'dg5-source-check.json').write_text(json.dumps(res, indent=1) + '\n')
    print(json.dumps({k2: res['wasm'][k2] for k2 in ('sha256', 'equals_registered', 'functions_containing_239.5', 'functions_containing_319.5')}),
          '| setter call sites:', len(calls))


if __name__ == '__main__':
    main()
