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
parser = argparse.ArgumentParser(description="Pinned 8th Wall RTTI/vtable/function candidate analysis")
parser.add_argument('--survey', required=True, type=pathlib.Path)
parser.add_argument('--xrefs', required=True, type=pathlib.Path)
parser.add_argument('--output', required=True, type=pathlib.Path)
args = parser.parse_args()
OUT = args.output.resolve()
if OUT.is_relative_to(pathlib.Path('/mnt/backup/SLAM')):
    parser.error('Canonical dataset is read-only')
OUT.mkdir(parents=True, exist_ok=False)
(OUT / pathlib.Path(__file__).name).write_bytes(pathlib.Path(__file__).read_bytes())
sys.path.insert(0, str(ROOT / 'scripts/dev/references/re-lab'))
import analyze as tools

source = json.loads((args.xrefs / 'results.json').read_text())['modules'][0]
folder = args.survey / 'xr-slam'
data = (folder / 'target.wasm').read_bytes()
if hashlib.sha256(data).hexdigest() != '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422':
    raise SystemExit('Input differs from pinned standalone module')
functions = tools.parse_disassembly((folder / 'disassembly.stdout').read_text())
table = {int(a): int(b) for a, b in re.findall(r'elem\[(\d+)\] = ref.func:(\d+)', (folder / 'sections.stdout').read_text())}
segments = source['data_segments']
refs = collections.defaultdict(set)
for i, f in functions.items():
    for line in f['lines']:
        match = re.search(r'\|\s+i32.const\s+(-?\d+)', line)
        if match:
            refs[int(match[1]) & 0xffffffff].add(i)


def memory_words(addr, count):
    seg = next(s for s in segments if s['memory_start'] <= addr < s['memory_start'] + s['size'])
    return struct.unpack_from('<' + str(count) + 'I', data, seg['file_start'] + addr - seg['memory_start'])


def shortest(start, target):
    paths = collections.deque([[start]])
    seen = {start}
    while paths:
        path = paths.popleft()
        if path[-1] == target:
            return path
        for child in functions.get(path[-1], {}).get('calls', {}):
            if child not in seen:
                seen.add(child)
                paths.append(path + [child])
    return None


residuals, commands = [], []
for entry in source['results']:
    if not entry['demangled_type'] or 'AutoDiffCostFunction' not in entry['demangled_type']:
        continue
    ptr = entry['aligned_data_pointers'][0]['memory_address']
    type_info = ptr - 4
    candidates = []
    for seg in segments:
        for offset in range((-seg['memory_start']) % 4, seg['size'] - 19, 4):
            addr = seg['memory_start'] + offset
            words = struct.unpack_from('<5I', data, seg['file_start'] + offset)
            if words[:2] == (0, type_info) and all(v in table for v in words[2:]):
                targets = [table[v] for v in words[2:]]
                candidates.append({'header_address': addr, 'address_point': addr + 8, 'slots': list(words[2:]),
                                   'target_functions': targets, 'address_point_const_refs': sorted(refs[addr + 8]),
                                   'target_const_refs_paths': {str(f): shortest(508, f) for f in refs[addr + 8]}})
                for f in targets:
                    (OUT / f'xr-slam-func-{f}.wasm.txt').write_text('\n'.join(functions[f]['lines']) + '\n')
    residuals.append({'type': entry['demangled_type'], 'string_address': entry['memory_address'], 'type_info_candidate': type_info,
                      'vtable_candidates': candidates})

symbols = json.loads((folder / 'symbols.stdout').read_text())
selected = sorted({1051, 1145, 1563} | {c['target_functions'][-1] for r in residuals for c in r['vtable_candidates']})
for index in selected:
    f = functions[index]
    address = next(s['vaddr'] for s in symbols if s.get('ordinal') == index and not s.get('is_imported'))
    for op in ['agfj', 'pdc']:
        argv = [str(tools.R2), '-q', '-e', 'scr.color=false', '-c', f"s {address};af;{op}", str(folder / 'target.wasm')]
        p = subprocess.run(argv, env=tools.tool_environment(), capture_output=True, timeout=120)
        prefix = OUT / f'symbol-func-{index}-{op}'
        prefix.with_suffix('.stdout').write_bytes(p.stdout)
        prefix.with_suffix('.stderr').write_bytes(p.stderr)
        commands.append({'argv': argv, 'exit': p.returncode, 'stdout_sha256': hashlib.sha256(p.stdout).hexdigest(), 'stderr_sha256': hashlib.sha256(p.stderr).hexdigest()})
    graph = json.loads((OUT / f'symbol-func-{index}-agfj.stdout').read_text())
    f['radare_instructions'] = sum(len(b.get('ops', [])) for b in graph[0]['blocks'])
result = {'script_sha256': hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(), 'residuals': residuals,
          'selected': [{'index': i, 'instructions': functions[i]['instructions'], 'radare_instructions': functions[i]['radare_instructions'],
                        'direct_path_from_process': shortest(508, i), 'calls': functions[i]['calls']} for i in selected],
          'commands': commands, 'limits': ['Vtable classification assumes observed Itanium-style offset/typeinfo/function-table layout',
                                          'Last slot is an Evaluate candidate, not recovered debug symbol',
                                          'Address-point constructors statically reachable does not prove runtime dispatch']}
(OUT / 'residuals-symbol.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
