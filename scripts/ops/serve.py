#!/usr/bin/env python3
"""Manual HTTPS lifecycle: exact owned PID identity, trusted health, bounded logs."""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import http.client
import json
import os
from pathlib import Path
import selectors
import shutil
import signal
import socket
import ssl
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
SERVER = ROOT / 'web/server.js'
STATE = Path(os.environ.get('MOBILE_SLAM_SERVER_STATE_DIR', str(ROOT / 'build/server'))).absolute()
RECORD = STATE / 'service.json'
SYSTEM_CA = '/etc/ssl/certs/ca-certificates.crt'
LOG_LIMIT = 1024 * 1024


def append_bounded_log(log, chunk, written, capped):
    if capped:
        return written, capped
    marker = b'\n[log limit reached; following output discarded]\n'
    clean = bytes(value if (value >= 32 and value != 127) or value in (9, 10) else 32 for value in chunk)
    remaining = LOG_LIMIT - len(marker) - written
    kept = clean[:max(0, remaining)]
    log.write(kept)
    written += len(kept)
    if len(clean) > remaining:
        log.write(marker)
        capped = True
    log.flush()
    return written, capped


def atomic_json(path, data):
    temporary = path.with_name(path.name + f'.{os.getpid()}.tmp')
    with temporary.open('x') as out:
        os.chmod(temporary, 0o600)
        json.dump(data, out, indent=2)
        out.write('\n')
    temporary.replace(path)


def proc_identity(pid):
    try:
        directory = Path('/proc') / str(pid)
        stat = (directory / 'stat').read_text().split(') ', 1)[1].split()
        if stat[0] == 'Z':
            return None
        return {'start_ticks': stat[19], 'uid': directory.stat().st_uid,
                'argv': (directory / 'cmdline').read_bytes().rstrip(b'\0').decode().split('\0'),
                'cwd': str((directory / 'cwd').resolve()), 'exe': str((directory / 'exe').resolve())}
    except (OSError, IndexError, UnicodeError):
        return None


def owned(record):
    expected = {key: record[key] for key in ('start_ticks', 'uid', 'argv', 'cwd', 'exe')}
    command = [expected['exe'], str(SERVER), '--port', str(record.get('port')), '--host', record.get('host'), '--hostname', 'dev.serdic.com']
    if (record.get('schema') != 'mobile-slam-owned-https-v1' or record.get('repo') != str(ROOT)
            or expected['uid'] != os.getuid() or expected['cwd'] != str(ROOT) or expected['argv'] != command):
        raise RuntimeError('PID identity mismatch; refusing to control a foreign process')
    actual = proc_identity(record['pid'])
    if actual is None:
        return False
    if actual != expected:
        raise RuntimeError('PID identity mismatch; refusing to control a foreign process')
    return True


def read_record():
    if not RECORD.exists():
        return None
    if RECORD.is_symlink() or RECORD.stat().st_uid != os.getuid():
        raise RuntimeError('Unsafe PID record ownership')
    return json.loads(RECORD.read_text())


def trusted_health(record):
    target = '127.0.0.1' if record['host'] == '0.0.0.0' else record['host']
    context = ssl.create_default_context(cafile=SYSTEM_CA)
    with socket.create_connection((target, record['port']), timeout=2) as raw:
        with context.wrap_socket(raw, server_hostname='dev.serdic.com') as connection:
            connection.settimeout(2)
            connection.sendall(f'GET /__health__ HTTP/1.1\r\nHost: dev.serdic.com:{record["port"]}\r\nConnection: close\r\n\r\n'.encode())
            response = http.client.HTTPResponse(connection)
            response.begin()
            payload = response.read(16385)
            if response.status != 200 or len(payload) > 16384:
                raise RuntimeError('Health response rejected')
            health = json.loads(payload)
    if health.get('pid') != record['pid'] or health.get('instanceId') != record['instance_id']:
        raise RuntimeError('Health identity mismatch')
    return health


def authorization(port):
    if port != 7002:
        return 'ephemeral-test', None, False
    flag = ROOT / 'build/refactor-orchestration/server-start-authorized.json'
    if not flag.is_file():
        raise RuntimeError('Candidate is not ready: parent server-start-authorized.json is absent')
    data = json.loads(flag.read_text())
    if data.get('authorized') is not True:
        raise RuntimeError('Candidate start is not authorized')
    mode = data.get('single_file_embedded_wasm', False)
    if not isinstance(mode, bool):
        raise RuntimeError('Embedded WASM mode must be boolean')
    single_file = mode
    if single_file and data.get('wasm_sha256') is not None:
        raise RuntimeError('Embedded WASM authorization must not approve a legacy external file')
    artifacts = [('vio_engine.js', 'js_sha256')]
    if not single_file:
        artifacts.append(('vio_engine.wasm', 'wasm_sha256'))
    for name, key in artifacts:
        with (ROOT / 'web' / name).open('rb') as source:
            actual = hashlib.file_digest(source, 'sha256').hexdigest()
        if not data.get(key) or data[key] != actual:
            raise RuntimeError(f'Deployed candidate hash mismatch: {name}')
    label = data.get('deployment_id', 'validated-candidate')
    if not isinstance(label, str) or not 1 <= len(label) <= 128 or any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._-' for c in label):
        raise RuntimeError('Invalid deployment identifier')
    manifest = data.get('source_manifest_sha256')
    if manifest is not None and (not isinstance(manifest, str) or len(manifest) != 64 or any(c not in '0123456789abcdef' for c in manifest)):
        raise RuntimeError('Invalid source manifest digest')
    return label, manifest, single_file


def monitor(config_path):
    config = json.loads(Path(config_path).read_text())
    authorized_label, authorized_manifest, single_file = authorization(config['port'])
    if (config['deployment_id'] != authorized_label or config.get('source_manifest_sha256') != authorized_manifest
            or config.get('single_file_embedded_wasm') != single_file):
        raise RuntimeError('Candidate authorization changed before launch')
    environment = os.environ.copy()
    environment['MOBILE_SLAM_SERVER_INSTANCE'] = config['instance_id']
    environment['MOBILE_SLAM_DEPLOYMENT_ID'] = config['deployment_id']
    environment['MOBILE_SLAM_SINGLE_FILE_WASM'] = '1' if single_file else '0'
    environment.pop('MOBILE_SLAM_SOURCE_MANIFEST_SHA256', None)
    if config.get('source_manifest_sha256'):
        environment['MOBILE_SLAM_SOURCE_MANIFEST_SHA256'] = config['source_manifest_sha256']
    with Path(config['log_path']).open('xb') as log:
        os.chmod(config['log_path'], 0o600)
        child = subprocess.Popen(config['argv'], cwd=ROOT, env=environment,
                                 stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        selector = selectors.DefaultSelector()
        try:
            identity = None
            previous_identity = None
            for _ in range(200):
                identity = proc_identity(child.pid)
                expected = {'argv': config['argv'], 'cwd': str(ROOT), 'exe': str(Path(config['argv'][0]).resolve()), 'uid': os.getuid()}
                if identity and all(identity[key] == value for key, value in expected.items()) and identity == previous_identity:
                    break
                previous_identity = identity
                identity = None
                if child.poll() is not None:
                    break
                time.sleep(0.01)
            if identity is None:
                raise RuntimeError('Child did not reach a stable post-exec PID identity')
            record = {'schema': 'mobile-slam-owned-https-v1', 'repo': str(ROOT),
                      'pid': child.pid, **identity, 'instance_id': config['instance_id'],
                      'port': config['port'], 'host': config['host'], 'log_path': config['log_path'],
                      'started_at': config['started_at'], 'monitor_pid': os.getpid()}
            atomic_json(RECORD, record)
            selector.register(child.stdout, selectors.EVENT_READ)
            written, capped = 0, False
            while True:
                if not selector.select(1) and child.poll() is None:
                    continue
                chunk = os.read(child.stdout.fileno(), 65536)
                if not chunk:
                    break
                written, capped = append_bounded_log(log, chunk, written, capped)
            child.wait()
        finally:
            selector.close()
            child.stdout.close()
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=5)


def archive_record(record):
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    RECORD.replace(STATE / f'stopped-{stamp}-{record["pid"]}.json')


def stop_record(record):
    if owned(record):
        os.kill(record['pid'], signal.SIGTERM)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and owned(record):
            time.sleep(0.1)
        if owned(record):
            os.kill(record['pid'], signal.SIGKILL)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and owned(record):
                time.sleep(0.1)
            if owned(record):
                raise RuntimeError('Owned server did not stop')
    archive_record(record)
    print(json.dumps({'state': 'stopped', 'pid': record['pid'], 'log': record['log_path']}))


def start(port, host):
    previous = read_record()
    if previous:
        if owned(previous):
            if previous['port'] != port or previous['host'] != host:
                raise RuntimeError('Owned service uses another port/host; stop it explicitly first')
            health = trusted_health(previous)
            print(json.dumps({'state': 'already-running', 'pid': previous['pid'], 'health': health}))
            return
        raise RuntimeError('Stale PID record: use stop to archive it before a new start')
    deployment, manifest, single_file = authorization(port)
    with socket.socket() as collision:
        try:
            collision.bind((host, port))
        except OSError as error:
            raise RuntimeError('Port collision: refusing to touch the existing listener') from error
    node = shutil.which('node')
    if not node:
        raise RuntimeError('Node.js missing')
    node = str(Path(node).resolve())
    instance = uuid.uuid4().hex
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    log_dir = ROOT / 'logs/https'
    if log_dir.is_symlink():
        raise RuntimeError('Log directory must not be a symlink')
    log_dir.mkdir(parents=True, exist_ok=True)
    if log_dir.resolve().parent != (ROOT / 'logs').resolve() or log_dir.stat().st_uid != os.getuid():
        raise RuntimeError('Unsafe log directory ownership')
    os.chmod(log_dir, 0o700)
    config = {'port': port, 'host': host, 'instance_id': instance,
              'deployment_id': deployment, 'started_at': stamp,
              'source_manifest_sha256': manifest,
              'single_file_embedded_wasm': single_file,
              'argv': [node, str(SERVER), '--port', str(port), '--host', host, '--hostname', 'dev.serdic.com'],
              'log_path': str(log_dir / f'{stamp}-{port}-{instance}.log')}
    config_path = STATE / f'launch-{instance}.json'
    atomic_json(config_path, config)
    # Double-fork leaves the monitor independent of the caller/terminal, and
    # the intermediate child is explicitly reaped. Conda's posix_spawn build
    # does not provide setsid on this host.
    launcher = os.fork()
    if launcher == 0:
        try:
            os.setsid()
            if os.fork() != 0:
                os._exit(0)
            null_fd = os.open(os.devnull, os.O_RDWR)
            for descriptor in (0, 1, 2):
                os.dup2(null_fd, descriptor)
            if null_fd > 2:
                os.close(null_fd)
            os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve()), '_monitor', str(config_path)], dict(os.environ))
        except BaseException:
            os._exit(1)
    _, status = os.waitpid(launcher, 0)
    if not os.WIFEXITED(status) or os.WEXITSTATUS(status) != 0:
        raise RuntimeError('Detached monitor launcher failed')
    deadline = time.monotonic() + 8
    failure = None
    while time.monotonic() < deadline:
        record = read_record()
        if record:
            try:
                if not owned(record):
                    raise RuntimeError('New owned server is not ready during startup')
                health = trusted_health(record)
                print(json.dumps({'state': 'running', 'pid': record['pid'], 'url': f'https://dev.serdic.com:{port}/', 'log': record['log_path'], 'health': health}))
                return
            except (OSError, RuntimeError, ValueError) as error:
                failure = error
        time.sleep(0.1)
    record = read_record()
    if record and record.get('instance_id') == instance:
        stop_record(record)
    raise RuntimeError(f'Startup did not become healthy: {failure or "no child PID record"}')


def main():
    if len(sys.argv) == 3 and sys.argv[1] == '_monitor':
        monitor(sys.argv[2])
        return
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'status', 'stop'])
    parser.add_argument('--port', type=int, default=7002)
    parser.add_argument('--host', choices=['0.0.0.0', '127.0.0.1'], default='0.0.0.0')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('Port must be 1..65535')
    if STATE.is_symlink():
        raise RuntimeError('Lifecycle state must not be a symlink')
    STATE.mkdir(parents=True, exist_ok=True)
    if STATE.is_symlink() or STATE.stat().st_uid != os.getuid():
        raise RuntimeError('Unsafe lifecycle state ownership')
    os.chmod(STATE, 0o700)
    with (STATE / 'lifecycle.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.action == 'start':
            start(args.port, args.host)
            return
        record = read_record()
        if not record:
            print(json.dumps({'state': 'stopped'}))
            return
        if args.action == 'stop':
            stop_record(record)
            return
        if not owned(record):
            print(json.dumps({'state': 'stale', 'pid': record['pid']}))
            raise SystemExit(1)
        print(json.dumps({'state': 'running', 'pid': record['pid'], 'log': record['log_path'], 'health': trusted_health(record)}))


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        print(json.dumps({'state': 'error', 'reason': str(error)}), file=sys.stderr)
        raise SystemExit(1)
