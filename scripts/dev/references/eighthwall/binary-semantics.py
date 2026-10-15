import argparse
import collections
import hashlib
import json
import pathlib
import re
import struct
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[4]
parser = argparse.ArgumentParser(description="Pinned 8th Wall string/address/code reachability survey")
parser.add_argument('--survey', required=True, type=pathlib.Path)
parser.add_argument('--output', required=True, type=pathlib.Path)
args = parser.parse_args()
SURVEY = args.survey.resolve()
OUT = args.output.resolve()
if OUT.is_relative_to(pathlib.Path('/mnt/backup/SLAM')):
    parser.error('Canonical dataset is read-only')
OUT.mkdir(parents=True, exist_ok=False)
(OUT / pathlib.Path(__file__).name).write_bytes(pathlib.Path(__file__).read_bytes())
sys.path.insert(0, str(ROOT / 'scripts/dev/references/re-lab'))
import analyze as graph_tools


def leb(data, pos, signed=False):
    value, shift = 0, 0
    while True:
        byte = data[pos]
        pos += 1
        value |= (byte & 127) << shift
        shift += 7
        if not byte & 128:
            if signed and byte & 64:
                value -= 1 << shift
            return value, pos


def data_segments(data):
    pos, segments = 8, []
    while pos < len(data):
        sid = data[pos]
        length, start = leb(data, pos + 1)
        pos = start + length
        if sid != 11:
            continue
        count, p = leb(data, start)
        for _ in range(count):
            flag, p = leb(data, p)
            if flag != 0 or data[p] != 0x41:
                raise ValueError('Expected active memory0 i32.const data segment')
            address, p = leb(data, p + 1, signed=True)
            assert data[p] == 0x0b
            size, p = leb(data, p + 1)
            segments.append({'file_start': p, 'size': size, 'memory_start': address})
            p += size
        assert p == pos
    return segments


records = []
for module in ['xr-slam', 'xr-face']:
    folder = SURVEY / module
    wasm = (folder / 'target.wasm').read_bytes()
    expected = {'xr-slam': '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422', 'xr-face': '52900bd20428641cb575027c2bf66dc270eca855e2e2676fac4c823a8235141d'}[module]
    if hashlib.sha256(wasm).hexdigest() != expected:
        raise SystemExit('Input differs from pinned standalone module')
    segments = data_segments(wasm)
    funcs = graph_tools.parse_disassembly((folder / 'disassembly.stdout').read_text())
    refs = collections.defaultdict(set)
    for index, function in funcs.items():
        for line in function['lines']:
            match = re.search(r'\|\s+i32.const\s+(-?\d+)', line)
            if match:
                refs[int(match[1]) & 0xffffffff].add(index)
    summary = json.loads((folder / 'summary.json').read_text())
    reaches = {f['name']: graph_tools.reachable(funcs, f['index']) for f in summary['selected']}
    strings = json.loads((folder / 'selected-strings.json').read_text())
    picked = [s for s in strings if re.search(r'ceres|AutoDiffCostFunction|BundleAdjustment|PosePnP|MapTracker|MapBuilder', s['text'])]
    results = []
    for s in picked:
        seg = next((seg for seg in segments if seg['file_start'] <= s['offset'] < seg['file_start'] + seg['size']), None)
        if seg is None:
            continue
        address = seg['memory_start'] + s['offset'] - seg['file_start']
        references = sorted(refs[address])
        pointers = []
        for segment in segments:
            start = segment['file_start']
            blob = wasm[start:start + segment['size']]
            for offset in range((-segment['memory_start']) % 4, len(blob) - 3, 4):
                if struct.unpack_from('<I', blob, offset)[0] == address:
                    ptr = segment['memory_start'] + offset
                    pointers.append({'memory_address': ptr, 'exact_const_refs': sorted(refs[ptr])})
        demangled = None
        if s['text'].startswith('N5ceres'):
            proc = subprocess.run(['c++filt', '-t', s['text']], capture_output=True, text=True, check=True)
            demangled = proc.stdout.strip()
        results.append(s | {'memory_address': address, 'demangled_type': demangled, 'exact_const_refs': references,
                            'direct_reachable_from': {name: sorted(set(references) & reached) for name, reached in reaches.items() if set(references) & reached},
                            'aligned_data_pointers': pointers})
    candidates = sorted({f for r in results for f in r['exact_const_refs']})
    for f in candidates:
        (OUT / f'{module}-func-{f}.wasm.txt').write_text('\n'.join(funcs[f]['lines']) + '\n')
    records.append({'module': module, 'wasm_sha256': hashlib.sha256(wasm).hexdigest(), 'data_segments': segments,
                    'results': results, 'candidate_function_count': len(candidates)})
out = OUT / 'results.json'
out.write_text(json.dumps({'script_sha256': hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),
                            'modules': records,
                            'limits': ['Exact i32.const references only; optimizer-derived addresses not resolved',
                                       'Data pointer references do not resolve indirect call targets',
                                       'Direct reachability is static and does not prove execution on any input']}, indent=2) + '\n')
print(json.dumps([{'module': r['module'], 'strings': len(r['results']), 'candidates': r['candidate_function_count'],
                   'exact_reference_strings': sum(bool(x['exact_const_refs']) for x in r['results']),
                   'reachable_reference_strings': sum(bool(x['direct_reachable_from']) for x in r['results'])} for r in records], indent=2))
