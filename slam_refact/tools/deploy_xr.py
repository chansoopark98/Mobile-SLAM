#!/usr/bin/env python3
"""Stage (dry run) and deploy the slam_refact XR entry page to web/slam-refact-xr.html + web/slam-refact-xr/.

The tree that is built (a mirror of the web/ layout; the file list is closed and matches the allowlist of web/server.js):
  slam-refact-xr.html                                 entry page = phone page (web-chunk/samples/phone/index.html) + <base> + sample links
  slam-refact-xr/gate.js                              start gate for the sample pages (slam_refact/web-xr/gate.js)
  slam-refact-xr/deploy.json                          provenance record of this build
  slam-refact-xr/engine/xr.js                         deployed reference runtime SDK (--sdk-dir; only xr.js is taken)
  slam-refact-xr/engine/<7 chunk files>               slam_refact/web-chunk/*.js
  slam-refact-xr/engine/slam-refact-engine/{manifest.json,slam_refact_engine.js,slam_refact_engine.wasm}   from --engine-dir
  slam-refact-xr/three/*, slam-refact-xr/aframe/*     local vendor builds (no CDN)
  slam-refact-xr/samples/{phone,threejs,aframe,common}/...   sample sources (threejs/aframe: root-absolute paths made relative + gate.js)

The engine/chunk artifacts are PARAMETERS: --engine-dir (a dir with manifest.json + slam_refact_engine.{js,wasm}, e.g. a later engine pin),
--chunk-dir, --sdk-dir. The engine files are verified against their manifest (the chunk verifies the same hashes at run time).

  # dry run (default): stage under build/, print the plan against web/, write nothing into web/
  python3 slam_refact/tools/deploy_xr.py --label <new-lowercase-dash-label> [--engine-dir <dir>] [--web-root web]
  # execute: needs the user's chat request, an authorization JSON on a NEW lane path, and an evidence dir
  python3 slam_refact/tools/deploy_xr.py --label <new label> --execute --authorization build/<lane>/deploy-authorized.json --evidence-dir build/<lane>/deploy-<label>

Authorization JSON: {"authorized": true, "candidate_sha256": <wasm sha>, "bundle_sha256": <tree digest printed by the dry run>,
"approved_by": <user, chat date, verbatim quote>, "reason": ...}. The historical VINS-Mono approval is refused. web/vio_engine.* is never touched.
Rollback: git (web/slam-refact-xr* tracked after the deploy commit) or the backup in <evidence-dir>/replaced/.
"""
import argparse
import datetime
import hashlib
import importlib.util
import json
import os
import posixpath
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = 'slam-refact-xr-deploy-v1'
ENGINE_MANIFEST_SCHEMA = 'slam-refact-chunk-engine-v1'
LEGACY_AUTHORIZATION = ROOT / 'build/refactor-orchestration/deploy-authorized.json'
CHUNK_FILES = ['xr-slam.js', 'engine-loader.js', 'gr8-compute-pyramid.js', 'gr8-shaders.js', 'image-events.js', 'image-targets.js', 'sensors.js']
ENGINE_FILES = ['manifest.json', 'slam_refact_engine.js', 'slam_refact_engine.wasm']
THREE_FILES = ['three.module.min.js', 'three.core.min.js']
AFRAME_FILE = 'aframe-v1.5.0.min.js'
PHONE_FILES = ['app.js', 'metrics.js', 'targets.js', 'style.css']
SAMPLE_PAGES = {'threejs': ['index.html', 'app.js'], 'aframe': ['index.html', 'app.js'], 'common': ['probe.js']}
PRINT_EXT = ('.pdf', '.png')
# Mirror of the allowlist in web/server.js (SLAM_REFACT_XR_FILES / SLAM_REFACT_XR_SAMPLE): every served path must be in this set or match this pattern.
SERVED_FIXED = frozenset(
    ['slam-refact-xr.html', 'slam-refact-xr/deploy.json', 'slam-refact-xr/gate.js']
    + [f'slam-refact-xr/engine/{n}' for n in ['xr.js'] + CHUNK_FILES]
    + [f'slam-refact-xr/engine/slam-refact-engine/{n}' for n in ENGINE_FILES]
    + [f'slam-refact-xr/three/{n}' for n in THREE_FILES] + [f'slam-refact-xr/aframe/{AFRAME_FILE}'])
SERVED_SAMPLE = re.compile(r'slam-refact-xr/samples/(?:(?:phone|threejs|aframe|common)/[A-Za-z0-9_-]+\.(?:html|js|css)'
                           r'|phone/targets/(?:print/)?[A-Za-z0-9_-][A-Za-z0-9_.-]*\.(?:json|png|pdf))')
FORBIDDEN_BASENAMES = ('vio_engine',)  # web/vio_engine.js, vio_engine.wasm, vio_engine.manifest.json are never a write target
LABEL_RE = re.compile(r'^[a-z0-9][a-z0-9-]*$')
ROOT_ABSOLUTE = re.compile(r'''(["'`])/(engine|three|aframe|samples)/''')
ANY_ROOT_ABSOLUTE = re.compile(r'''(?:src|href)\s*=\s*["']/|from\s+["']/|import\s*\(\s*["']/|import\s+["']/''')
DEFAULT_ENGINE_DIR = ROOT / 'web/slam-refact-xr/engine/slam-refact-engine'
DEFAULT_SDK_DIR = ROOT / 'web/slam-refact-xr/engine'
DEFAULT_THREE_DIR = ROOT / 'web/slam-refact-xr/three'
DEFAULT_AFRAME_DIR = ROOT / 'web/slam-refact-xr/aframe'
STAGE_BASE = ROOT / 'build/slam-refact-w-20261023/stage'
LICENSE_NOTE = ('xr.js is the reference runtime engine binary. Redistribution from web/ rests on the user\'s statement that the legal review is done '
                '(HANDOFF section 1; orchestrator decision J-1). The engine-binary licence text is not available locally: UNVERIFIED. '
                'three.js / A-Frame carry their own licence headers; not independently verified.')


def fail(msg):
    raise SystemExit(f'deploy_xr: {msg}')


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    with open(path, 'rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def git(*args, check=True):
    r = subprocess.run(['git', '-C', str(ROOT), *args], capture_output=True, text=True)
    if check and r.returncode:
        fail(f'git {" ".join(args)}: {r.stderr.strip()}')
    return r.stdout.strip()


def rel(path):
    try:
        return os.path.relpath(Path(path).resolve(), ROOT)
    except ValueError:
        return str(path)


def check_served(served):
    """Refuse any served path that is not a normalized, relative path inside slam-refact-xr.html | slam-refact-xr/ and in the web/server.js allowlist."""
    if not isinstance(served, str) or not served:
        fail(f'served path is empty or not a string: {served!r}')
    if served.startswith('/') or '\\' in served or any(ord(c) < 32 or c == '%' for c in served):
        fail(f'served path is absolute or has a backslash, control character or %: {served!r}')
    parts = served.split('/')
    if any(part in ('', '.', '..') for part in parts) or posixpath.normpath(served) != served:
        fail(f'served path is not normalized (empty component, "." or ".."): {served!r}')
    if any(part.startswith('.') for part in parts):
        fail(f'served path has a hidden component: {served!r}')
    if served != 'slam-refact-xr.html' and not served.startswith('slam-refact-xr/'):
        fail(f'served path is outside slam-refact-xr.html / slam-refact-xr/: {served!r}')
    if parts[-1].lower().startswith(FORBIDDEN_BASENAMES):
        fail(f'served path names a vio_engine file, which is never deployed: {served!r}')
    if served not in SERVED_FIXED and not SERVED_SAMPLE.fullmatch(served):
        fail(f'served path is not in the web/server.js allowlist: {served!r}')
    return served


class Bundle:
    """Ordered map of served path -> (bytes, source, transform)."""

    def __init__(self):
        self.files = {}

    def add(self, served, data, source, transform=None):
        check_served(served)
        if served in self.files:
            fail(f'duplicate served path {served}')
        if not isinstance(data, bytes):
            data = data.encode()
        self.files[served] = (data, source, transform)

    def add_file(self, served, src, transform=None):
        check_served(served)
        src = Path(src)
        if src.is_symlink() or not src.is_file():
            fail(f'input is not a regular file: {src}')
        self.add(served, src.read_bytes(), rel(src), transform)

    def digest(self):
        h = hashlib.sha256()
        for name in sorted(n for n in self.files if n != 'slam-refact-xr/deploy.json'):
            h.update(name.encode() + b'\0' + sha256_bytes(self.files[name][0]).encode() + b'\n')
        return h.hexdigest()


def read_engine(engine_dir):
    engine_dir = engine_dir.resolve()
    blobs = {}
    for name in ENGINE_FILES:
        p = engine_dir / name
        if p.is_symlink() or not p.is_file():
            fail(f'engine dir {rel(engine_dir)} lacks regular file {name}')
        blobs[name] = p.read_bytes()
    manifest = json.loads(blobs['manifest.json'])
    if manifest.get('schema') != ENGINE_MANIFEST_SCHEMA:
        fail(f'engine manifest schema {manifest.get("schema")} != {ENGINE_MANIFEST_SCHEMA}')
    for key, name in (('js', 'slam_refact_engine.js'), ('wasm', 'slam_refact_engine.wasm')):
        entry = manifest['artifacts'][key]
        if entry['file'] != name or entry['sha256'] != sha256_bytes(blobs[name]) or entry['bytes'] != len(blobs[name]):
            fail(f'engine {name} does not match its manifest (sha256/bytes)')
    public_manifest = {'schema': ENGINE_MANIFEST_SCHEMA, 'record_kind': 'source-artifact-index',
                       'engine': 'slam_refact',
                       'artifacts': {key: {field: manifest['artifacts'][key][field] for field in ('file', 'sha256', 'bytes')}
                                     for key in ('js', 'wasm')}}
    blobs['manifest.json'] = (json.dumps(public_manifest, indent=2) + '\n').encode()
    return engine_dir, blobs, manifest


def engine_provenance(engine_dir, blobs, manifest):
    build = manifest.get('build') or {}
    pin_head = build.get('gitHead')
    pin_tree = git('rev-parse', f'{pin_head}:slam_refact/engine', check=False) if pin_head else ''
    cur_tree = git('rev-parse', 'HEAD:slam_refact/engine')
    prov = build.get('provenance') or {}
    prov_ok = None
    if prov.get('file'):
        p = ROOT / prov['file']
        prov_ok = p.is_file() and sha256_file(p) == prov.get('sha256')
    return {
        'dir': rel(engine_dir), 'manifest_sha256': sha256_bytes(blobs['manifest.json']),
        'js_sha256': sha256_bytes(blobs['slam_refact_engine.js']), 'wasm_sha256': sha256_bytes(blobs['slam_refact_engine.wasm']),
        'wasm_bytes': len(blobs['slam_refact_engine.wasm']), 'built_from_git_head': pin_head, 'built_engine_dirty': build.get('engineDirty'),
        'built_engine_tree': pin_tree or None, 'current_engine_tree': cur_tree,
        'engine_tree_equals_current_head': bool(pin_tree) and pin_tree == cur_tree,
        'build_provenance': {'file': prov.get('file'), 'sha256': prov.get('sha256'), 'file_matches': prov_ok},
        'created_at': manifest.get('createdAt'),
    }


def make_entry_page(phone_html, links_html):
    """slam-refact-xr.html = the phone page with <base> (so its relative paths resolve inside slam-refact-xr/samples/phone/) and the sample links."""
    text = phone_html.decode()
    meta = '<meta charset="utf-8">'
    marker = '<p id="start-note" class="note"></p>'
    if text.count(meta) != 1 or text.count(marker) != 1 or '<base' in text:
        fail('phone/index.html no longer has the anchors the entry page transform needs (meta charset, start-note) or already has <base>')
    text = text.replace(meta, meta + '\n  <base href="slam-refact-xr/samples/phone/">', 1)
    text = text.replace(marker, marker + '\n' + links_html.rstrip('\n'), 1)
    return text


def rewrite_sample(name, data):
    text = data.decode()
    new, n = ROOT_ABSOLUTE.subn(lambda m: f'{m.group(1)}../../{m.group(2)}/', text)
    if name.endswith('.html'):
        if text.count('<script') < 1:
            fail(f'{name}: no <script> to put gate.js before')
        new = new.replace('<script', '<script src="../../gate.js"></script>\n  <script', 1)
        n += 1
    return new, n


def build_bundle(args):
    chunk_dir, samples_dir, xr_dir = args.chunk_dir.resolve(), args.samples_dir.resolve(), args.xr_src_dir.resolve()
    b = Bundle()
    engine_dir, blobs, manifest = read_engine(args.engine_dir)
    sdk = args.sdk_dir.resolve() / 'xr.js'
    # engine + SDK + chunk
    for name in ENGINE_FILES:
        b.add(f'slam-refact-xr/engine/slam-refact-engine/{name}', blobs[name], rel(engine_dir / name))
    b.add_file('slam-refact-xr/engine/xr.js', sdk)
    for name in CHUNK_FILES:
        b.add_file(f'slam-refact-xr/engine/{name}', chunk_dir / name)
    for name in THREE_FILES:
        b.add_file(f'slam-refact-xr/three/{name}', args.three_dir / name)
    b.add_file(f'slam-refact-xr/aframe/{AFRAME_FILE}', args.aframe_dir / AFRAME_FILE)
    b.add_file('slam-refact-xr/gate.js', xr_dir / 'gate.js')
    # phone page = entry page
    phone = samples_dir / 'phone'
    for name in PHONE_FILES:
        b.add_file(f'slam-refact-xr/samples/phone/{name}', phone / name)
    links = (xr_dir / 'entry-links.html').read_text()
    b.add('slam-refact-xr.html', make_entry_page((phone / 'index.html').read_bytes(), links), f'{rel(phone / "index.html")} + {rel(xr_dir / "entry-links.html")}',
          'insert <base href="slam-refact-xr/samples/phone/"> after meta charset; insert sample links after #start-note')
    # bundled image targets: only what targets/manifest.json lists + the print sheets; checked against targets/MANIFEST.sha256
    tdir = phone / 'targets'
    sums = {}
    for line in (tdir / 'MANIFEST.sha256').read_text().splitlines():
        m = re.fullmatch(r'([0-9a-f]{64})\s+(\S.*)', line)
        if m and not m.group(2).startswith('.omc/'):
            sums[m.group(2)] = m.group(1)
    tmanifest = json.loads((tdir / 'manifest.json').read_text())
    names = ['manifest.json']
    for t in tmanifest['targets']:
        names += [t['json'], t['image']]
    names += sorted(f'print/{p.name}' for p in (tdir / 'print').iterdir() if p.suffix in PRINT_EXT and p.is_file())
    for name in names:
        check_served(f'slam-refact-xr/samples/phone/targets/{name}')
        if name not in sums or sums[name] != sha256_file(tdir / name):
            fail(f'targets/{name} does not match targets/MANIFEST.sha256')
        b.add_file(f'slam-refact-xr/samples/phone/targets/{name}', tdir / name)
    # three.js / A-Frame samples: root-absolute paths of the dev server become relative, gate.js goes first
    for sample, files in SAMPLE_PAGES.items():
        for name in files:
            src = samples_dir / sample / name
            if sample == 'common':
                b.add_file(f'slam-refact-xr/samples/{sample}/{name}', src)
                continue
            text, n = rewrite_sample(f'{sample}/{name}', src.read_bytes())
            b.add(f'slam-refact-xr/samples/{sample}/{name}', text, rel(src), f'{n} root-absolute path(s) made relative' + ('; gate.js inserted' if name.endswith('.html') else ''))
    # no root-absolute reference may remain in any shipped page or sample script
    for served, (data, _, _) in b.files.items():
        if served.endswith(('.html', '.js')) and served.startswith(('slam-refact-xr/samples/', 'slam-refact-xr.html')):
            text = data.decode(errors='replace')
            if ROOT_ABSOLUTE.search(text) or ANY_ROOT_ABSOLUTE.search(text):
                fail(f'{served} still has a root-absolute URL (it would not resolve under /slam-refact-xr/)')
    return b, engine_dir, blobs, manifest


def input_paths_dirty():
    return git('status', '--porcelain', '--', 'slam_refact/web-chunk', 'slam_refact/web-xr', 'slam_refact/engine', 'slam_refact/tools/deploy_xr.py').splitlines()


def provenance(args, b, engine_dir, blobs, manifest, mode, authorization):
    inputs_dirty = input_paths_dirty()
    xr_entry = b.files['slam-refact-xr/engine/xr.js']
    return {
        'schema': SCHEMA, 'mode': mode, 'generated_at': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'),
        'git': {'head': git('rev-parse', 'HEAD'), 'branch': git('rev-parse', '--abbrev-ref', 'HEAD'), 'worktree_dirty': bool(git('status', '--porcelain')),
                'input_paths_dirty': inputs_dirty,
                'trees': {p: git('rev-parse', f'HEAD:{p}', check=False) or None for p in ('slam_refact/web-chunk', 'slam_refact/engine', 'slam_refact/web-xr')}},
        'engine': engine_provenance(engine_dir, blobs, manifest),
        'xr_js': {'sha256': sha256_bytes(xr_entry[0]), 'bytes': len(xr_entry[0]), 'source': xr_entry[1], 'licence_note': LICENSE_NOTE},
        'bundle_sha256': b.digest(), 'bundle_digest_rule': 'sha256 over "served path\\0sha256(content)\\n", sorted, excluding slam-refact-xr/deploy.json',
        'files': [{'path': n, 'sha256': sha256_bytes(d), 'bytes': len(d), 'source': s, 'transform': t} for n, (d, s, t) in sorted(b.files.items())],
        'tool': {'file': rel(Path(__file__)), 'sha256': sha256_file(__file__)},
        'authorization': authorization,
        'note': 'XR8 chunk entry page; independent of web/vio_engine.* (VINS-Mono) and of web/slam-refact.html (measurement page)',
    }


def public_artifact_record(record):
    """The served index verifies bytes; operator/build details remain local evidence."""
    engine = record['engine']
    return {
        'schema': SCHEMA, 'record_kind': 'source-artifact-index',
        'engine': {k: engine[k] for k in ('js_sha256', 'wasm_sha256', 'wasm_bytes')},
        'xr_js': {k: record['xr_js'][k] for k in ('sha256', 'bytes')},
        'bundle_sha256': record['bundle_sha256'], 'bundle_digest_rule': record['bundle_digest_rule'],
        'files': [{k: row[k] for k in ('path', 'sha256', 'bytes')} for row in record['files']],
    }


def write_tree(b, root):
    root.mkdir(parents=True, exist_ok=False)
    for served, (data, _, _) in b.files.items():
        check_served(served if served == 'slam-refact-xr.html' or served.startswith('slam-refact-xr/') else 'slam-refact-xr/' + served)
        p = root / served
        if not p.resolve().is_relative_to(root.resolve()):
            fail(f'refusing to write outside {root}: {served!r}')
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)
        os.chmod(p, 0o644)


def plan_against(web_root, b):
    new, changed, same = [], [], []
    for served, (data, _, _) in sorted(b.files.items()):
        p = web_root / served
        if not p.is_file():
            new.append(served)
        elif sha256_file(p) == sha256_bytes(data):
            same.append(served)
        else:
            changed.append(served)
    stale = []
    d = web_root / 'slam-refact-xr'
    if d.is_dir():
        stale = sorted(str(p.relative_to(web_root)) for p in d.rglob('*') if p.is_file() and str(p.relative_to(web_root)) not in b.files)
    return {'new': len(new), 'changed': changed, 'unchanged': len(same), 'stale_in_destination': stale}


def check_authorization(path, wasm_sha, js_sha, bundle_sha):
    path = Path(path).resolve()
    if path == LEGACY_AUTHORIZATION.resolve():
        fail('refusing the historical VINS-Mono authorization file; use a new lane path')
    data = json.loads(path.read_text())
    if data.get('authorized') is not True:
        fail('authorization: "authorized" is not true')
    if data.get('candidate_sha256') != wasm_sha:
        fail('authorization: candidate_sha256 does not match the engine wasm')
    if 'candidate_js_sha256' in data and data['candidate_js_sha256'] != js_sha:
        fail('authorization: candidate_js_sha256 does not match the engine js')
    if data.get('bundle_sha256') != bundle_sha:
        fail(f'authorization: bundle_sha256 does not match this build ({bundle_sha}); re-run the dry run, review, and authorize that digest')
    if not data.get('approved_by') or str(data['approved_by']).startswith('FILL'):
        fail('authorization: approved_by must carry the user\'s chat instruction')
    return {'path': rel(path), 'sha256': sha256_file(path), 'approved_by': data.get('approved_by'), 'reason': data.get('reason')}


def execute(args, b, prov_fn, wasm_sha, js_sha):
    web_root = args.web_root.resolve()
    html_dest, dir_dest = web_root / 'slam-refact-xr.html', web_root / 'slam-refact-xr'
    if not web_root.is_dir():
        fail(f'web root {web_root} is not a directory')
    for served in b.files:
        check_served(served)
    for dest in (html_dest, dir_dest):
        if dest.is_symlink():
            fail(f'refusing to deploy over a symlink: {dest}')
    dirty = input_paths_dirty()
    if dirty:
        fail('refusing --execute: input paths are git-dirty, so deploy.json would not describe the bundle (commit or stash first):\n  ' + '\n  '.join(dirty))
    if not args.authorization or not args.evidence_dir:
        fail('--execute needs --authorization and --evidence-dir')
    if args.evidence_dir.exists():
        fail(f'refusing to reuse evidence dir {args.evidence_dir}')
    auth = check_authorization(args.authorization, wasm_sha, js_sha, b.digest())
    private_record = prov_fn('execute', auth)
    record = (json.dumps(public_artifact_record(private_record), indent=1) + '\n').encode()
    b.add('slam-refact-xr/deploy.json', record, 'generated')
    pid = os.getpid()
    new_dir = web_root / f'.slam-refact-xr.new-{pid}'
    new_html = web_root / f'.slam-refact-xr.html.new-{pid}'
    ev = args.evidence_dir
    (ev / 'replaced').mkdir(parents=True)
    try:
        write_tree(below_prefix(b), new_dir)
        new_html.write_bytes(b.files['slam-refact-xr.html'][0])
        os.chmod(new_html, 0o644)
    except BaseException:
        shutil.rmtree(new_dir, ignore_errors=True)
        new_html.unlink(missing_ok=True)
        raise
    # verify the staged copy byte for byte before touching the destination
    for served, (data, _, _) in b.files.items():
        p = new_html if served == 'slam-refact-xr.html' else new_dir / served.removeprefix('slam-refact-xr/')
        if sha256_file(p) != sha256_bytes(data):
            fail(f'staged copy of {served} differs from the build')
    # move the previous deploy output into the evidence dir (a move, nothing is deleted), then the new tree into place
    if dir_dest.exists():
        os.rename(dir_dest, ev / 'replaced/slam-refact-xr')
    if html_dest.exists():
        os.rename(html_dest, ev / 'replaced/slam-refact-xr.html')
    os.rename(new_dir, dir_dest)
    os.rename(new_html, html_dest)
    for served, (data, _, _) in b.files.items():
        if sha256_file(web_root / served) != sha256_bytes(data):
            fail(f'deployed {served} differs from the build')
    (ev / 'deploy.json').write_bytes(record)
    (ev / 'deployment-record.private.json').write_text(json.dumps(private_record, indent=2) + '\n')
    return auth


def below_prefix(b):
    """The files below slam-refact-xr/ with that prefix stripped (the tree that becomes <web>/slam-refact-xr/)."""
    out = Bundle()
    for served, (data, src, tr) in b.files.items():
        if served != 'slam-refact-xr.html':
            out.files[served.removeprefix('slam-refact-xr/')] = (data, src, tr)
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--label', required=True, help='new lowercase-dash label; the stage dir build/slam-refact-w-20261023/stage/<label> is never reused')
    p.add_argument('--engine-dir', type=Path, default=DEFAULT_ENGINE_DIR)
    p.add_argument('--sdk-dir', type=Path, default=DEFAULT_SDK_DIR)
    p.add_argument('--chunk-dir', type=Path, default=ROOT / 'slam_refact/web-chunk')
    p.add_argument('--samples-dir', type=Path, default=None, help='default: <chunk-dir>/samples')
    p.add_argument('--xr-src-dir', type=Path, default=ROOT / 'slam_refact/web-xr')
    p.add_argument('--three-dir', type=Path, default=DEFAULT_THREE_DIR)
    p.add_argument('--aframe-dir', type=Path, default=DEFAULT_AFRAME_DIR)
    p.add_argument('--web-root', type=Path, default=ROOT / 'web')
    p.add_argument('--stage-base', type=Path, default=STAGE_BASE)
    p.add_argument('--execute', action='store_true', help='write into --web-root (default is a dry run that only stages under build/)')
    p.add_argument('--authorization', type=Path)
    p.add_argument('--evidence-dir', type=Path)
    args = p.parse_args(argv)
    if not LABEL_RE.match(args.label):
        fail('--label must match [a-z0-9][a-z0-9-]*')
    args.samples_dir = args.samples_dir or args.chunk_dir / 'samples'
    stage = args.stage_base.resolve() / args.label
    if not stage.is_relative_to(ROOT / 'build'):
        fail('stage must be under build/')
    if stage.exists():
        fail(f'refusing to reuse stage dir {rel(stage)}')

    b, engine_dir, blobs, manifest = build_bundle(args)
    wasm_sha, js_sha = sha256_bytes(blobs['slam_refact_engine.wasm']), sha256_bytes(blobs['slam_refact_engine.js'])
    prov = lambda mode, auth: provenance(args, b, engine_dir, blobs, manifest, mode, auth)
    if not args.execute:
        dirty = input_paths_dirty()
        if dirty:
            print('deploy_xr: WARNING input paths are git-dirty (a later --execute will refuse):\n  ' + '\n  '.join(dirty), file=sys.stderr)
        record = json.dumps(public_artifact_record(prov('dry-run', None)), indent=1) + '\n'
        b.add('slam-refact-xr/deploy.json', record, 'generated')
        write_tree(b, stage)
        plan = plan_against(args.web_root.resolve(), b)
        print(json.dumps({'mode': 'dry-run', 'stage': rel(stage), 'files': len(b.files), 'bundle_sha256': b.digest(), 'engine_wasm_sha256': wasm_sha,
                          'engine_js_sha256': js_sha, 'engine': prov('dry-run', None)['engine'], 'plan_against_web_root': {'web_root': rel(args.web_root.resolve()), **plan},
                          'wrote_into_web_root': False}, indent=1))
        return 0
    auth = execute(args, b, prov, wasm_sha, js_sha)
    print(json.dumps({'mode': 'execute', 'deployed': [rel(args.web_root.resolve() / 'slam-refact-xr.html'), rel(args.web_root.resolve() / 'slam-refact-xr')],
                      'bundle_sha256': b.digest(), 'engine_wasm_sha256': wasm_sha, 'authorization': auth, 'evidence': rel(args.evidence_dir)}, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
