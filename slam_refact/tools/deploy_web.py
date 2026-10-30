#!/usr/bin/env python3
"""Deploy the slam_refact WASM engine to the measurement page directory web/slam-refact/.

Copies <build>/slam_refact_engine.{js,wasm} into web/slam-refact/ and writes
web/slam-refact/manifest.json (runtime schema and artifact sha256/size). Detailed build
and authorization records are written only to --evidence-dir. The page (web/slam-refact.html) shows the manifest and its
worker refuses to run bytes that do not match it.

Every write needs --authorization <json> with {"authorized": true, "candidate_sha256": <wasm sha>}
(optional "candidate_js_sha256" is checked too). The historical VINS-Mono approval
build/refactor-orchestration/deploy-authorized.json is refused. web/vio_engine.* is never touched:
the destination must be a directory named slam-refact directly under the web root.

  python3 slam_refact/tools/deploy_web.py --authorization build/<lane>/deploy-authorized.json \
      [--build-dir build/slam-refact-wasm/wasm] [--evidence-dir build/<lane>/deploy]
"""
import argparse
import datetime
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS = {'js': 'slam_refact_engine.js', 'wasm': 'slam_refact_engine.wasm'}
LEGACY_AUTHORIZATION = ROOT / 'build/refactor-orchestration/deploy-authorized.json'
SCHEMA = 'slam-refact-web-deploy-v1'


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    with open(path, 'rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def git(*args):
    return subprocess.run(['git', '-C', str(ROOT), *args], check=True, capture_output=True,
                          text=True).stdout.strip()


def source_provenance():
    """HEAD, dirty flags and a digest of the working-tree engine sources (tracked files)."""
    files = sorted(git('ls-files', '-z', '--', 'slam_refact/engine').split('\0'))
    files = [f for f in files if f]
    h = hashlib.sha256()
    for name in files:
        h.update(name.encode() + b'\0' + sha256_file(ROOT / name).encode() + b'\n')
    return {
        'head': git('rev-parse', 'HEAD'),
        'branch': git('rev-parse', '--abbrev-ref', 'HEAD'),
        'engine_tree': git('rev-parse', 'HEAD:slam_refact/engine'),
        'engine_dirty': bool(git('status', '--porcelain', '--', 'slam_refact/engine')),
        'worktree_dirty': bool(git('status', '--porcelain')),
    }, {'engine_files': len(files), 'engine_sources_sha256': h.hexdigest(),
        'digest_rule': 'sha256 over "path\\0sha256(content)\\n" of git ls-files slam_refact/engine'}


def build_info(build_dir):
    info = {'dir': os.path.relpath(build_dir, ROOT)}
    cache = build_dir.parent / 'CMakeCache.txt'
    if cache.is_file():
        for line in cache.read_text(errors='replace').splitlines():
            for key in ('CMAKE_BUILD_TYPE', 'CMAKE_CXX_COMPILER_VERSION', 'CMAKE_HOME_DIRECTORY'):
                if line.startswith(key + ':'):
                    info[key] = line.split('=', 1)[1]
    return info


def check_destination(dest, web_root):
    dest = dest.resolve()
    if dest.parent != web_root.resolve() or dest.name != 'slam-refact':
        raise SystemExit(f'refusing destination {dest}: must be <web root>/slam-refact')
    return dest


def check_authorization(path, digests):
    path = path.resolve()
    if path == LEGACY_AUTHORIZATION.resolve():
        raise SystemExit('refusing the historical VINS-Mono authorization file; use a new lane path')
    data = json.loads(path.read_text())
    if data.get('authorized') is not True:
        raise SystemExit('authorization: "authorized" is not true')
    if data.get('candidate_sha256') != digests['wasm']:
        raise SystemExit('authorization: candidate_sha256 does not match the wasm artifact')
    if 'candidate_js_sha256' in data and data['candidate_js_sha256'] != digests['js']:
        raise SystemExit('authorization: candidate_js_sha256 does not match the js artifact')
    return {'path': os.path.relpath(path, ROOT), 'sha256': sha256_file(path),
            'approved_by': data.get('approved_by'), 'reason': data.get('reason')}


def atomic_write(dest_dir, name, data):
    fd, tmp = tempfile.mkstemp(prefix=f'.{name}.', dir=dest_dir)
    try:
        with os.fdopen(fd, 'wb') as out:
            out.write(data)
            out.flush()
            os.fsync(out.fileno())
        if sha256_file(tmp) != sha256_bytes(data):
            raise SystemExit(f'copy checksum mismatch for {name}')
        os.chmod(tmp, 0o644)
        os.replace(tmp, dest_dir / name)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--authorization', type=Path, required=True)
    p.add_argument('--build-dir', type=Path, default=ROOT / 'build/slam-refact-wasm/wasm')
    p.add_argument('--web-root', type=Path, default=ROOT / 'web')
    p.add_argument('--evidence-dir', type=Path, help='new directory: copy of the manifest + backup of the replaced files')
    args = p.parse_args(argv)

    build_dir = args.build_dir.resolve()
    dest = check_destination(args.web_root / 'slam-refact', args.web_root)
    blobs = {k: (build_dir / name).read_bytes() for k, name in ARTIFACTS.items()}
    digests = {k: sha256_bytes(v) for k, v in blobs.items()}
    authorization = check_authorization(args.authorization, digests)
    git_info, source = source_provenance()
    if args.evidence_dir and args.evidence_dir.exists():
        raise SystemExit(f'refusing to reuse evidence dir {args.evidence_dir}')

    private_record = {
        'schema': SCHEMA, 'engine': 'slam_refact', 'git': git_info, 'source': source,
        'artifacts': {k: {'file': ARTIFACTS[k], 'sha256': digests[k], 'bytes': len(blobs[k])} for k in ARTIFACTS},
        'build': build_info(build_dir),
        'deployed_at': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'),
        'authorization': authorization,
        'page': 'web/slam-refact.html',
        'note': 'measurement page engine; independent of web/vio_engine.* (VINS-Mono)',
    }

    manifest = {'schema': SCHEMA, 'record_kind': 'source-artifact-index', 'engine': 'slam_refact',
                'artifacts': private_record['artifacts']}
    dest.mkdir(exist_ok=True)
    if args.evidence_dir:
        backup = args.evidence_dir / 'replaced'
        backup.mkdir(parents=True)
        for name in [*ARTIFACTS.values(), 'manifest.json']:
            if (dest / name).is_file():
                shutil.copy2(dest / name, backup / name)
    for k, name in ARTIFACTS.items():
        atomic_write(dest, name, blobs[k])
    for k, name in ARTIFACTS.items():
        if sha256_file(dest / name) != digests[k]:
            raise SystemExit(f'deployed {name} does not match the build artifact')
    text = (json.dumps(manifest, indent=1) + '\n').encode()
    atomic_write(dest, 'manifest.json', text)
    if args.evidence_dir:
        (args.evidence_dir / 'manifest.json').write_bytes(text)
        (args.evidence_dir / 'deployment-record.private.json').write_text(json.dumps(private_record, indent=2) + '\n')
    print(json.dumps({'deployed': os.path.relpath(dest, ROOT), 'head': git_info['head'],
                      'engine_dirty': git_info['engine_dirty'], 'wasm_sha256': digests['wasm'],
                      'js_sha256': digests['js']}, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
