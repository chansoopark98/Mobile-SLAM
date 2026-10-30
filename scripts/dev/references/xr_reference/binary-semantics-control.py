#!/usr/bin/env python3
"""Known WAT control for string-address mapping and direct/indirect boundaries."""
import argparse
import ast
import hashlib
import json
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts/dev/references/re-lab'))
import analyze

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', required=True, type=pathlib.Path)
args = parser.parse_args()
out = args.output.resolve()
if out.is_relative_to(pathlib.Path('/mnt/backup/SLAM')):
    parser.error('Canonical dataset is read-only')
out.mkdir(parents=True, exist_ok=False)
(out / pathlib.Path(__file__).name).write_bytes(pathlib.Path(__file__).read_bytes())
wat = '''(module (type (func)) (memory 1) (table 1 funcref)
 (elem (i32.const 0) $decoy) (data (i32.const 1234) "ceres-known\\00")
 (func $root (export "root") call $child)
 (func $child i32.const 1234 drop)
 (func $decoy i32.const 1234 drop)
 (func (export "indirect") i32.const 0 call_indirect (type 0)))'''
(out / 'control.wat').write_text(wat)
commands = []
for name, argv in [
    ('compile', [analyze.WABT / 'wat2wasm', out / 'control.wat', '-o', out / 'control.wasm']),
    ('sections', [analyze.WABT / 'wasm-objdump', '-x', out / 'control.wasm']),
    ('disassembly', [analyze.WABT / 'wasm-objdump', '-d', out / 'control.wasm']),
]:
    p = subprocess.run(list(map(str, argv)), capture_output=True)
    (out / (name + '.stdout')).write_bytes(p.stdout)
    (out / (name + '.stderr')).write_bytes(p.stderr)
    commands.append({'argv': list(map(str, argv)), 'exit': p.returncode,
                     'stdout_sha256': hashlib.sha256(p.stdout).hexdigest(),
                     'stderr_sha256': hashlib.sha256(p.stderr).hexdigest()})
    if p.returncode:
        (out / 'failed-commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        raise SystemExit('Control tool failure; preserved logs')
script = pathlib.Path(__file__).with_name('binary-semantics.py')
# Load only the two parser definitions, without executing the survey CLI.
tree = ast.parse(script.read_text())
namespace = {}
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)
                             and n.name in ['leb', 'data_segments']], type_ignores=[]), str(script), 'exec'), namespace)
b = (out / 'control.wasm').read_bytes()
segments = namespace['data_segments'](b)
s = segments[0]
offset = b.index(b'ceres-known')
address = s['memory_start'] + offset - s['file_start']
funcs = analyze.parse_disassembly((out / 'disassembly.stdout').read_text())
reached = analyze.reachable(funcs, 0)
checks = {'memory_address': address == 1234,
          'wabtx_confirms_offset': 'init i32=1234' in (out / 'sections.stdout').read_text(),
          'direct_reach': reached == {0, 1}, 'decoy_excluded': 2 not in reached,
          'indirect_unresolved': funcs[3]['indirect_sites'] == 1 and 2 not in analyze.reachable(funcs, 3),
          'signed_leb_negative': namespace['leb'](bytes([0x7f]), 0, True) == (-1, 1)}
result = {'checks': checks, 'pass': all(checks.values()), 'data_file_offset': offset,
          'calculated_memory_address': address, 'segments': segments,
          'analyzer_sha256': hashlib.sha256(script.read_bytes()).hexdigest(),
          'script_sha256': hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(), 'commands': commands}
(out / 'verification.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
if not result['pass']:
    raise SystemExit(1)
