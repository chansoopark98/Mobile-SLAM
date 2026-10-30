import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import socket
import ssl
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch
from contextlib import redirect_stdout

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/ops/serve.sh'


def free_port():
    with socket.socket() as connection:
        connection.bind(('127.0.0.1', 0))
        return connection.getsockname()[1]


class LifecycleTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='mobile-slam-lifecycle-')
        self.state = Path(self.temporary.name) / 'state'
        self.environment = {**os.environ, 'MOBILE_SLAM_SERVER_STATE_DIR': str(self.state)}
        self.port = free_port()

    def command(self, action, port=None):
        result = subprocess.run(['bash', str(SCRIPT), action, '--port', str(port or self.port), '--host', '127.0.0.1'],
                                cwd=ROOT, env=self.environment, capture_output=True, text=True, timeout=20)
        lines = (result.stdout if result.returncode == 0 else result.stderr).strip().splitlines()
        payload = json.loads(lines[-1]) if lines else {}
        return result.returncode, payload

    def tearDown(self):
        if (self.state / 'service.json').exists():
            self.command('stop')
        self.temporary.cleanup()

    def test_start_status_stop_uses_trusted_tls_and_preserves_existing_log(self):
        old_log = ROOT / 'logs/test-tumvi.log'
        before = hashlib.sha256(old_log.read_bytes()).hexdigest() if old_log.exists() else None
        code, started = self.command('start')
        self.assertEqual(code, 0, started)
        self.assertEqual(started['health']['tls']['intermediateCount'], 2)
        self.assertEqual(started['health']['remoteLogs'], 'disabled')
        pid = started['pid']
        self.assertEqual(self.command('start')[1]['pid'], pid)
        self.assertEqual(self.command('status')[1]['pid'], pid)
        self.assertLessEqual(Path(started['log']).stat().st_size, 1024 * 1024)
        context = ssl.create_default_context(cafile='/etc/ssl/certs/ca-certificates.crt')
        with socket.create_connection(('127.0.0.1', self.port), timeout=3) as raw:
            with context.wrap_socket(raw, server_hostname='dev.serdic.com') as connection:
                connection.sendall(f'HEAD / HTTP/1.1\r\nHost: dev.serdic.com:{self.port}\r\nConnection: close\r\n\r\n'.encode())
                chunks = []
                while chunk := connection.recv(16384):
                    chunks.append(chunk)
                headers, body = b''.join(chunks).split(b'\r\n\r\n', 1)
                self.assertTrue(headers.startswith(b'HTTP/1.1 200'))
                self.assertEqual(body, b'')
        self.assertEqual(self.command('stop')[0], 0)
        self.assertEqual(self.command('status')[1]['state'], 'stopped')
        if before is not None:
            self.assertEqual(hashlib.sha256(old_log.read_bytes()).hexdigest(), before)

    def test_collision_does_not_control_existing_listener(self):
        with socket.socket() as collision:
            collision.bind(('127.0.0.1', self.port))
            collision.listen()
            code, payload = self.command('start')
            self.assertNotEqual(code, 0)
            self.assertIn('collision', payload['reason'])
            self.assertEqual(collision.getsockname()[1], self.port)
            self.assertFalse((self.state / 'service.json').exists())

    def test_strict_curl_uses_hostname_chain_and_system_trust(self):
        self.assertEqual(self.command('start')[0], 0)
        result = subprocess.run(['curl', '-q', '--noproxy', '*', '--cacert', '/etc/ssl/certs/ca-certificates.crt',
                                 '--fail', '--silent', '--show-error', '--max-time', '5',
                                 '--resolve', f'dev.serdic.com:{self.port}:127.0.0.1',
                                 f'https://dev.serdic.com:{self.port}/__health__'],
                                cwd=ROOT, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        health = json.loads(result.stdout)
        self.assertEqual(health['tls']['hostname'], 'dev.serdic.com')
        self.assertEqual(health['tls']['intermediateCount'], 2)
        self.assertEqual(health['source']['serverSha256'], hashlib.sha256((ROOT / 'web/server.js').read_bytes()).hexdigest())

    def test_foreign_pid_record_never_signals_other_process(self):
        child = subprocess.Popen(['sleep', '30'])
        try:
            self.state.mkdir()
            stat = Path(f'/proc/{child.pid}/stat').read_text().split(') ', 1)[1].split()
            record = {'schema': 'mobile-slam-owned-https-v1', 'repo': str(ROOT), 'pid': child.pid,
                      'start_ticks': stat[19], 'uid': os.getuid(), 'argv': ['wrong-command'],
                      'cwd': str(ROOT), 'exe': '/usr/bin/node'}
            (self.state / 'service.json').write_text(json.dumps(record))
            for action in ['start', 'stop', 'status']:
                code, payload = self.command(action)
                self.assertNotEqual(code, 0)
                self.assertIn('identity mismatch', payload['reason'])
                self.assertIsNone(child.poll())
            (self.state / 'service.json').unlink()
        finally:
            child.terminate()
            child.wait(timeout=5)

    def test_stale_record_blocks_start_until_explicit_stop(self):
        code, payload = self.command('start')
        self.assertEqual(code, 0, payload)
        record_path = self.state / 'service.json'
        record = json.loads(record_path.read_text())
        self.assertEqual(self.command('stop')[0], 0)
        record_path.write_text(json.dumps(record))
        code, payload = self.command('start')
        self.assertNotEqual(code, 0)
        self.assertIn('Stale PID record', payload['reason'])
        self.assertEqual(self.command('stop')[0], 0)

    def test_log_sink_caps_bytes_and_strips_terminal_controls(self):
        spec = importlib.util.spec_from_file_location('serve', ROOT / 'scripts/ops/serve.py')
        ops = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ops)
        stream = io.BytesIO()
        written, capped = ops.append_bounded_log(stream, b'hello\x1b[2J\x00\r\x7fworld\n', 0, False)
        self.assertNotIn(b'\x1b', stream.getvalue())
        self.assertNotIn(b'\x00', stream.getvalue())
        self.assertNotIn(b'\r', stream.getvalue())
        self.assertNotIn(b'\x7f', stream.getvalue())
        written, capped = ops.append_bounded_log(stream, b'x' * (2 * 1024 * 1024), written, capped)
        self.assertTrue(capped)
        self.assertLessEqual(len(stream.getvalue()), 1024 * 1024)
        before = stream.getvalue()
        ops.append_bounded_log(stream, b'not-written', written, capped)
        self.assertEqual(stream.getvalue(), before)

    def test_candidate_authorization_requires_current_paired_hashes(self):
        spec = importlib.util.spec_from_file_location('serve', ROOT / 'scripts/ops/serve.py')
        ops = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ops)
        fixture = Path(self.temporary.name) / 'candidate'
        ops.ROOT = fixture
        with self.assertRaisesRegex(RuntimeError, 'absent'):
            ops.authorization(7002)
        (fixture / 'web').mkdir(parents=True)
        (fixture / 'web/vio_engine.js').write_bytes(b'known-loader')
        (fixture / 'web/vio_engine.wasm').write_bytes(b'known-wasm')
        directory = fixture / 'build/refactor-orchestration'
        directory.mkdir(parents=True)
        record = {'authorized': True, 'js_sha256': hashlib.sha256(b'known-loader').hexdigest(),
                  'wasm_sha256': hashlib.sha256(b'known-wasm').hexdigest(), 'deployment_id': 'test'}
        (directory / 'server-start-authorized.json').write_text(json.dumps(record))
        self.assertEqual(ops.authorization(7002), ('test', None, False))
        (fixture / 'web/vio_engine.wasm').write_bytes(b'wrong-pair')
        with self.assertRaisesRegex(RuntimeError, 'hash mismatch'):
            ops.authorization(7002)

    def test_single_file_authorization_ignores_missing_or_legacy_external_wasm(self):
        spec = importlib.util.spec_from_file_location('serve', ROOT / 'scripts/ops/serve.py')
        ops = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ops)
        fixture = Path(self.temporary.name) / 'single-file'
        ops.ROOT = fixture
        (fixture / 'web').mkdir(parents=True)
        (fixture / 'web/vio_engine.js').write_bytes(b'known-embedded-loader')
        directory = fixture / 'build/refactor-orchestration'
        directory.mkdir(parents=True)
        record = {'authorized': True, 'single_file_embedded_wasm': True,
                  'js_sha256': hashlib.sha256(b'known-embedded-loader').hexdigest(),
                  'wasm_sha256': None, 'deployment_id': 'embedded'}
        flag = directory / 'server-start-authorized.json'
        flag.write_text(json.dumps(record))
        self.assertEqual(ops.authorization(7002), ('embedded', None, True))
        (fixture / 'web/vio_engine.wasm').write_bytes(b'old-unused-artifact')
        self.assertEqual(ops.authorization(7002), ('embedded', None, True))
        record['wasm_sha256'] = hashlib.sha256(b'old-unused-artifact').hexdigest()
        flag.write_text(json.dumps(record))
        with self.assertRaisesRegex(RuntimeError, 'legacy external'):
            ops.authorization(7002)
        record['wasm_sha256'] = None
        flag.write_text(json.dumps(record))
        (fixture / 'web/vio_engine.js').write_bytes(b'wrong-loader')
        with self.assertRaisesRegex(RuntimeError, 'hash mismatch'):
            ops.authorization(7002)

    def test_start_retries_transient_pid_metadata_without_signalling_foreign_process(self):
        spec = importlib.util.spec_from_file_location('serve', ROOT / 'scripts/ops/serve.py')
        ops = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ops)
        self.state.mkdir()
        ops.STATE, ops.RECORD = self.state, self.state / 'service.json'
        original_owned = ops.owned
        calls = 0

        def transition(record):
            nonlocal calls
            calls += 1
            if calls == 1:
                raise RuntimeError('Fixture: post-exec PID metadata is transitioning')
            return original_owned(record)

        with patch.dict(os.environ, self.environment), patch.object(ops, 'owned', transition), redirect_stdout(io.StringIO()):
            ops.start(self.port, '127.0.0.1')
        record = json.loads(ops.RECORD.read_text())
        self.assertGreaterEqual(calls, 2)
        self.assertTrue(original_owned(record))
        self.assertEqual(ops.trusted_health(record)['pid'], record['pid'])
        self.assertEqual(self.command('stop')[0], 0)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if ops.proc_identity(record['monitor_pid']) is None:
                break
            time.sleep(0.01)
        self.assertIsNone(ops.proc_identity(record['monitor_pid']))


if __name__ == '__main__':
    unittest.main()
