#!/usr/bin/env python3
"""Locate pinned ARM64 ARCore runtime boundaries through exact code/string xrefs."""
import argparse
import bisect
import collections
import datetime
import hashlib
import json
import pathlib
import re
import struct
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts/dev/references/re-lab'))
from bootstrap import TOOLS, digest, tool_environment

PIN = '7c32d45c2ef1ebaf92deae86a7ba31b0dd00d39bde09ab0890535ff396d62050'
EXPORTS = ['ArSession_update', 'ArSession_pause', 'ArSession_resume', 'ArSession_destroy',
           'ArFrame_create', 'ArFrame_destroy', 'ArFrame_getTimestamp', 'ArFrame_acquireCamera',
           'ArFrame_acquireCameraImage', 'ArFrame_getAndroidSensorPose', 'ArCamera_getPose',
           'ArCamera_getDisplayOrientedPose', 'ArCamera_getTrackingState', 'ArCamera_getTrackingFailureReason',
           'ArCamera_getImageIntrinsics', 'ArCamera_getTextureIntrinsics', 'ArCamera_release',
           'ArCameraIntrinsics_getFocalLength', 'ArCameraIntrinsics_getPrincipalPoint',
           'ArCameraIntrinsics_getImageDimensions', 'ArImage_getTimestamp', 'ArImage_getWidth',
           'ArImage_getHeight', 'ArImage_getPlaneData', 'ArImage_release']
NEEDLES = [
    'Requested timestamp after latest received imu sample.',
    'Image timestamp appears ahead of CLOCK_BOOTTIME',
    'Attempts to add data at timestamp',
    'The query timestamp %.*g is too new.',
    'Cannot set the initial state with a timestamp',
    'ASensorManager_createEventQueue failed',
    'NdkSensorDataProvider : no event queue.',
    'Failed to create ASensorEventQueue.',
    'ASensorEventQueue_setEventRate error with status:',
    'sensor_event_queue_ is null before calling',
    'Failed to register sensor to queue',
    'Could not disable sensor from event queue',
]


def elf_sections(data):
    if data[:6] != b'\x7fELF\x02\x01':
        raise ValueError('Require little-endian ELF64')
    offset = struct.unpack_from('<Q', data, 40)[0]
    entry_size, count, names_index = struct.unpack_from('<HHH', data, 58)
    sections = [struct.unpack_from('<IIQQQQIIQQ', data, offset + i * entry_size) for i in range(count)]
    names = data[sections[names_index][4]:sections[names_index][4] + sections[names_index][5]]
    return {names[s[0]:].split(b'\0', 1)[0].decode(): {'address': s[3], 'offset': s[4], 'size': s[5]}
            for s in sections}


def adjacent_adrp_add(data, section):
    """Only adjacent ADRP + 64-bit ADD-immediate with matching source register."""
    refs = []
    for offset in range(section['offset'], section['offset'] + section['size'] - 7, 4):
        first, second = struct.unpack_from('<II', data, offset)
        if first & 0x9f000000 != 0x90000000 or second & 0xff000000 != 0x91000000:
            continue
        reg = first & 31
        if (second >> 5) & 31 != reg or reg == 31:
            continue
        immediate = ((first >> 29) & 3) | (((first >> 5) & 0x7ffff) << 2)
        if immediate & (1 << 20):
            immediate -= 1 << 21
        pc = section['address'] + offset - section['offset']
        page = (pc & ~4095) + (immediate << 12)
        add = ((second >> 10) & 4095) << (12 if (second >> 22) & 1 else 0)
        refs.append({'instruction_address': pc, 'target_address': page + add,
                     'adrp_register': reg, 'add_destination': second & 31})
    return refs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=pathlib.Path)
    parser.add_argument('--output', required=True, type=pathlib.Path)
    parser.add_argument('--follow', action='append', default=[], type=lambda value: int(value, 0), help='Additional exact FDE function entry address')
    parser.add_argument('--follow-only', action='store_true', help='Analyze only the explicitly supplied function addresses')
    args = parser.parse_args()
    original = args.input.resolve()
    data = original.read_bytes()
    if hashlib.sha256(data).hexdigest() != PIN:
        parser.error('Different binary: redo address/signature review before changing the pin')
    out = args.output.resolve()
    if out.is_relative_to(pathlib.Path('/mnt/backup/SLAM')):
        parser.error('Canonical dataset is read-only')
    out.mkdir(parents=True, exist_ok=False)
    target = out / 'target.so'
    target.write_bytes(data)
    (out / pathlib.Path(__file__).name).write_bytes(pathlib.Path(__file__).read_bytes())
    commands = []

    def run(name, argv, timeout=120):
        start = time.monotonic()
        try:
            p = subprocess.run(list(map(str, argv)), env=tool_environment(), capture_output=True, timeout=timeout)
            code, stdout, stderr = p.returncode, p.stdout, p.stderr
        except subprocess.TimeoutExpired as exc:
            code, stdout, stderr = None, exc.stdout or b'', exc.stderr or b''
        (out / (name + '.stdout')).write_bytes(stdout)
        (out / (name + '.stderr')).write_bytes(stderr)
        commands.append({'name': name, 'argv': list(map(str, argv)), 'exit': code, 'seconds': time.monotonic() - start,
                         'stdout_sha256': hashlib.sha256(stdout).hexdigest(), 'stderr_sha256': hashlib.sha256(stderr).hexdigest()})
        (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        if code != 0:
            raise SystemExit('Tool failed; outputs preserved: ' + name)
        return stdout.decode(errors='replace')

    r2 = TOOLS / 'radare2/usr/bin/radare2'
    for name, argv in [('r2-version', [r2, '-v']), ('objdump-version', ['aarch64-linux-gnu-objdump', '--version']),
                       ('readelf-version', ['readelf', '--version']),
                       ('layout', ['readelf', '-h', '-l', '-S', '-d', target]),
                       ('symbols', ['readelf', '--dyn-syms', '--wide', target]),
                       ('relocations', ['readelf', '-r', '--wide', target])]:
        run(name, argv)
    unwind = run('unwind', ['readelf', '--debug-dump=frames-interp', target])
    symbols = json.loads(run('radare-symbols', [r2, '-q', '-c', 'isj', target]))
    ranges = sorted(set((int(a, 16), int(b, 16)) for a, b in re.findall(r'FDE .* pc=([0-9a-f]+)\.\.([0-9a-f]+)', unwind)))
    starts = [a for a, b in ranges]
    sections = elf_sections(data)
    strings = []
    for m in re.finditer(rb'[ -~]{9,}', data):
        text = m[0].decode()
        if any(needle in text for needle in NEEDLES):
            section = next((s for s in sections.values() if s['offset'] <= m.start() < s['offset'] + s['size']), None)
            if section:
                strings.append({'file_offset': m.start(), 'address': section['address'] + m.start() - section['offset'], 'text': text})
    by_target = collections.defaultdict(list)
    for ref in adjacent_adrp_add(data, sections['.text']):
        by_target[ref['target_address']].append(ref)
    selected = {}
    for string in strings:
        string['xrefs'] = []
        for ref in by_target[string['address']]:
            i = bisect.bisect_right(starts, ref['instruction_address']) - 1
            owner = ranges[i] if i >= 0 and ref['instruction_address'] < ranges[i][1] else None
            string['xrefs'].append(ref | {'fde_range': owner})
            if owner:
                selected.setdefault(owner, []).append(string['text'])
    results = []
    if args.follow_only:
        selected = {}
    for address in args.follow:
        i = bisect.bisect_left(starts, address)
        if i >= len(ranges) or ranges[i][0] != address:
            raise SystemExit('Follow address must be an exact unwind function entry')
        selected.setdefault(ranges[i], []).append(f'explicit direct callee {address:#x}')
    public = {s['name']: s for s in symbols if s['name'] in EXPORTS and not s.get('is_imported')}
    if set(public) != set(EXPORTS):
        raise SystemExit('Selected export missing')
    if args.follow_only:
        public = {}
    entries = [(f'internal-{a:x}', a, b, labels, 'unwind') for (a, b), labels in selected.items()]
    entries += [(name, s['vaddr'], s['vaddr'] + s['size'], [name], 'dynamic_symbol') for name, s in public.items()]
    for name, start, end, labels, bounds_source in entries:
        # Unwind-derived function boundaries avoid beginning analysis at the middle of a function.
        for operation in ['agfj', 'pdc']:
            run(name + '-' + operation, [r2, '-q', '-e', 'scr.color=false', '-c', f's {start};af;{operation}', target])
        gnu = run(name + '-objdump', ['aarch64-linux-gnu-objdump', '-d', f'--start-address={start}', f'--stop-address={end}', target])
        graph = json.loads((out / (name + '-agfj.stdout')).read_text())
        ops = [op for f in graph for b in f.get('blocks', []) for op in b.get('ops', [])]
        raw = json.loads(run(name + '-bounded-ops', [r2, '-q', '-e', 'scr.color=false', '-c', f'pDj {end-start} @ {start}', target]))
        gnu_count = len(re.findall(r'^\s*[0-9a-f]+:\s+[0-9a-f]{8}\s+', gnu, re.M))
        results.append({'name': name, 'start': start, 'end': end, 'bound_bytes': end - start, 'labels': labels, 'bounds_source': bounds_source,
                        'radare_instructions': len(ops), 'gnu_instructions': gnu_count,
                        'radare_bounded_instructions': len(raw), 'radare_bounded_matches_gnu': len(raw) == gnu_count,
                        'radare_cfg_ops_outside_bounds': sum(not start <= op.get('addr', -1) < end for op in ops),
                        'radare_blocks': sum(len(f.get('blocks', [])) for f in graph),
                        'calls': [op for op in raw if op.get('type') in ['call', 'ucall', 'icall', 'rcall']],
                        'radare_matches_bound_bytes': len(ops) * 4 == end - start,
                        'gnu_matches_bound_bytes': gnu_count * 4 == end - start})
    summary = {'observed_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
               'input': {'path': str(original), 'sha256': PIN, 'bytes': len(data)},
               'script_sha256': digest(pathlib.Path(__file__)), 'fde_count': len(ranges), 'sections': sections,
               'strings': strings, 'internal_functions': [f for f in results if f['bounds_source'] == 'unwind'],
               'public_functions': [f for f in results if f['bounds_source'] == 'dynamic_symbol'],
               'limits': ['Only adjacent ADRP+ADD exact string starts; missed refs are not evidence of absence',
                          'FDE ranges establish code bounds, not original C++ function names',
                          'Radare CFG may follow shared tails/outside FDE bounds; use bounded raw operations for inventory',
                          'Diagnostic strings identify branch purpose; they do not prove actual runtime execution',
                          'No target ELF execution, device collection, optimizer identification or accuracy measurement']}
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({'fde_count': len(ranges), 'strings': len(strings), 'internal_functions': len(results),
                      'commands': len(commands)}, indent=2))


if __name__ == '__main__':
    main()
