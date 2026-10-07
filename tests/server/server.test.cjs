const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const http = require('node:http');
const vm = require('node:vm');
const { once } = require('node:events');
const { execFileSync } = require('node:child_process');

const serverFile = path.resolve(__dirname, '../../web/server.js');
const source = fs.readFileSync(serverFile, 'utf8');

function fixture() {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), 'mobile-slam-server-'));
  const web = path.join(root, 'web');
  for (const folder of ['web/js', 'web/public', 'web/.certs', 'web/.private', 'assets/keys', 'logs', 'room1/cam0/data', 'room4/imu0']) fs.mkdirSync(path.join(root, folder), { recursive: true });
  for (const [name, body] of Object.entries({ 'web/index.html': '<h1>fixture</h1>', 'web/js/app.js': 'export const fixture = 1;', 'web/vio_engine.js': 'var fixture=1;', 'web/vio_engine.wasm': 'wasm-fixture', 'web/public/profile.json': '{"camera":"fixture"}', 'web/server.js': 'PRIVATE_SERVER_SENTINEL', 'web/.certs/key.pem': 'SECRET_SENTINEL', 'web/.private/hidden.js': 'SECRET_SENTINEL', 'web/package.json': '{"secret":"SECRET_SENTINEL"}', 'assets/keys/serdic_com_cert.crt': 'FAKE_CERT', 'assets/keys/serdic_com.key': 'FAKE_KEY', 'logs/test-tumvi.log': 'PRESERVE_OLD_LOG', 'room1/cam0/data/1.png': 'image-fixture', 'room1/cam0/data.csv': '#time,file\n1,1.png', 'room4/imu0/data.csv': '#imu\n1,0,0,0,0,0,0' })) fs.writeFileSync(path.join(root, name), body);
  fs.symlinkSync(path.join(root, 'assets/keys/serdic_com.key'), path.join(web, 'js/outside.js'));
  fs.symlinkSync(path.join(web, '.private/hidden.js'), path.join(web, 'js/hidden.js'));
  fs.symlinkSync(path.join(web, 'server.js'), path.join(web, 'js/server-alias.js'));
  return { root, web, cleanup: () => fs.rmSync(root, { recursive: true, force: true }) };
}

function loadFixture(f, options = {}) {
  let captured;
  const module = { exports: {} };
  const context = { module, exports: module.exports, __dirname: f.web, __filename: path.join(f.web, 'server.js'), Buffer, URL, console: { log() {}, error() {} }, process: { pid: process.pid, uptime: process.uptime, getuid: process.getuid, argv: ['node', 'server.js', '0'], env: {}, stdout: { write() {} } }, setTimeout, clearTimeout };
  context.require = (name) => name === 'https' || name === 'node:https' ? { createServer(_options, handler) { captured = handler; return { listen() {} }; } } : require(name);
  vm.runInNewContext(source, context, { filename: serverFile });
  const app = typeof module.exports.createApp === 'function' ? module.exports.createApp({ webRoot: f.web, datasetRoots: { room1: path.join(f.root, 'room1'), room4: path.join(f.root, 'room4') }, hostname: 'dev.serdic.com', metadata: { fixture: true }, ...options }) : captured;
  return { exports: module.exports, app };
}

async function running(f, action, options = {}) {
  const { app } = loadFixture(f, options);
  assert.equal(typeof app, 'function');
  const server = http.createServer(app).listen(0, '127.0.0.1');
  await once(server, 'listening');
  try { return await action(server.address().port); }
  finally { await new Promise(resolve => server.close(resolve)); }
}

function request(port, pathname, method = 'GET', body = '') {
  return new Promise((resolve, reject) => {
    const req = http.request({ host: '127.0.0.1', port, path: pathname, method, headers: { Host: `dev.serdic.com:${port}`, Connection: 'close', 'Content-Length': Buffer.byteLength(body) } }, response => {
      const chunks = [];
      response.on('data', chunk => chunks.push(chunk));
      response.on('end', () => resolve({ status: response.statusCode, headers: response.headers, body: Buffer.concat(chunks).toString() }));
    });
    req.on('error', reject); req.end(body);
  });
}

test('Import has no log/TLS/listener startup side effects and exports factories', () => {
  const f = fixture();
  try {
    const loaded = loadFixture(f);
    assert.equal(fs.readFileSync(path.join(f.root, 'logs/test-tumvi.log'), 'utf8'), 'PRESERVE_OLD_LOG');
    assert.equal(typeof loaded.exports.createApp, 'function');
    assert.equal(typeof loaded.exports.createServer, 'function');
  } finally { f.cleanup(); }
});

test('Public JS/profile and HEAD have MIME/security/cache headers and no HEAD body', async () => {
  const f = fixture();
  try { await running(f, async port => {
    const script = await request(port, '/js/app.js?v=11');
    assert.equal(script.status, 200);
    assert.match(script.headers['content-type'], /javascript/);
    assert.equal(script.headers['cross-origin-opener-policy'], 'same-origin');
    assert.equal(script.headers['cross-origin-embedder-policy'], 'credentialless');
    assert.match(script.headers['permissions-policy'], /camera=\(self\)/);
    assert.match(script.headers['cache-control'], /no-store/);
    const profile = await request(port, '/public/profile.json'); assert.equal(profile.status, 200);
    const head = await request(port, '/js/app.js', 'HEAD');
    assert.equal(head.status, 200); assert.equal(head.body, '');
    assert.equal(Number(head.headers['content-length']), Buffer.byteLength('export const fixture = 1;'));
  }); } finally { f.cleanup(); }
});

test('GET and HEAD reject private, encoded traversal, NUL and hidden/outside/server symlink targets', async () => {
  const f = fixture();
  try { await running(f, async port => {
    for (const pathname of ['/server.js', '/package.json', '/.certs/key.pem', '/%2ecerts/key.pem', '/js/../server.js', '/js/%2e%2e/server.js', '/js/%252e%252e/server.js', '/js/%00app.js', '/js/outside.js', '/js/hidden.js', '/js/server-alias.js', '/public/a.key', '/public/a.crt', '/datasets/tum/dataset-room1_512_16/mav0/../archive.tar']) {
      for (const method of ['GET', 'HEAD']) {
        const response = await request(port, pathname, method);
        assert.ok([400, 403].includes(response.status), `${method} ${pathname}: ${response.status}`);
        assert.ok(!response.body.includes('SECRET_SENTINEL'));
        assert.ok(!response.body.includes('PRIVATE_SERVER_SENTINEL'));
        if (method === 'HEAD') assert.equal(response.body, '');
      }
    }
  }); } finally { f.cleanup(); }
});

test('Only explicitly mapped room1/room4 datasets serve sensor files; archives and other roots denied', async () => {
  const f = fixture();
  try { await running(f, async port => {
    assert.equal((await request(port, '/datasets/tum/dataset-room1_512_16/mav0/cam0/data.csv')).status, 200);
    assert.equal((await request(port, '/datasets/tum/dataset-room4_512_16/mav0/imu0/data.csv')).status, 200);
    assert.equal((await request(port, '/datasets/tum/dataset-room2_512_16/mav0/cam0/data.csv')).status, 403);
    assert.equal((await request(port, '/datasets/tum/dataset-room1_512_16.tar')).status, 403);
  }); } finally { f.cleanup(); }
});

test('Remote writes are disabled and existing logs preserved; health omits private paths', async () => {
  const f = fixture();
  try { await running(f, async port => {
    for (const method of ['POST', 'PUT', 'PATCH', 'DELETE']) {
      const response = await request(port, '/log', method, '{"msg":"\\u001b[2JSECRET_SENTINEL"}');
      assert.equal(response.status, 405); assert.equal(response.headers.allow, 'GET, HEAD');
    }
    assert.equal(fs.readFileSync(path.join(f.root, 'logs/test-tumvi.log'), 'utf8'), 'PRESERVE_OLD_LOG');
    const health = await request(port, '/__health__');
    assert.equal(health.status, 200);
    assert.ok(!health.body.includes(f.root)); assert.ok(!health.body.includes('FAKE_KEY'));
  }); } finally { f.cleanup(); }
});

test('Strict TLS loads the real leaf and two intermediates; missing chain and wrong SAN fail', async () => {
  const f = fixture();
  try {
    const exported = loadFixture(f).exports;
    assert.equal(typeof exported.loadTlsOptions, 'function');
    const realRoot = path.resolve(__dirname, '../..');
    const keyDir = path.join(realRoot, 'assets/keys');
    const tls = exported.loadTlsOptions({ keyPath: path.join(keyDir, 'serdic_com.key'), certPath: path.join(keyDir, 'serdic_com_cert.crt'), chainPath: path.join(keyDir, 'serdic_com_chain_cert.crt'), hostname: 'dev.serdic.com' });
    assert.equal((tls.cert.toString().match(/BEGIN CERTIFICATE/g) || []).length, 3);
    assert.throws(() => exported.loadTlsOptions({ keyPath: path.join(keyDir, 'serdic_com.key'), certPath: path.join(keyDir, 'serdic_com_cert.crt'), chainPath: path.join(f.root, 'missing.crt'), hostname: 'dev.serdic.com' }));
    assert.throws(() => exported.loadTlsOptions({ keyPath: path.join(keyDir, 'serdic_com.key'), certPath: path.join(keyDir, 'serdic_com_cert.crt'), chainPath: path.join(keyDir, 'serdic_com_chain_cert.crt'), hostname: 'localhost' }));
    const invalidChain = path.join(f.root, 'chain-with-root.crt');
    fs.writeFileSync(invalidChain, fs.readFileSync(path.join(keyDir, 'serdic_com_chain_cert.crt')) + '\n' + fs.readFileSync(path.join(keyDir, 'serdic_com_root_cert.crt')));
    fs.chmodSync(invalidChain, 0o644);
    assert.throws(() => exported.loadTlsOptions({ keyPath: path.join(keyDir, 'serdic_com.key'), certPath: path.join(keyDir, 'serdic_com_cert.crt'), chainPath: invalidChain, hostname: 'dev.serdic.com' }), /exclude self-signed root/);
  } finally { f.cleanup(); }
});

test('Embedded single-file mode never advertises or serves an unused legacy external WASM', async () => {
  const f = fixture();
  fs.symlinkSync(path.join(f.web, 'vio_engine.wasm'), path.join(f.web, 'public/legacy.json'));
  try { await running(f, async port => {
    const health = JSON.parse((await request(port, '/__health__')).body);
    assert.equal(health.wasmMode, 'single-file-embedded');
    assert.equal(health.artifactSnapshot['vio_engine.wasm'].active, false);
    assert.equal(health.artifactSnapshot['vio_engine.wasm'].available, false);
    assert.equal(health.artifactSnapshot['vio_engine.wasm'].sha256, undefined);
    assert.equal((await request(port, '/vio_engine.js')).status, 200);
    assert.equal((await request(port, '/vio_engine.wasm')).status, 403);
    assert.equal((await request(port, '/public/legacy.json')).status, 403);
  }, { singleFileEmbeddedWasm: true }); } finally { f.cleanup(); }
});

test('Absent optional favicon returns empty 204 for GET and HEAD', async () => {
  const f = fixture();
  try { await running(f, async port => {
    for (const method of ['GET', 'HEAD']) {
      const response = await request(port, '/favicon.ico', method);
      assert.equal(response.status, 204);
      assert.equal(response.body, '');
      assert.equal(response.headers['content-length'], undefined);
    }
  }); } finally { f.cleanup(); }
});
