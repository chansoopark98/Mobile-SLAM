'use strict';

const https = require('node:https');
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const { execFileSync } = require('node:child_process');

const ROOT = path.resolve(__dirname, '..');
const HOSTNAME = 'dev.serdic.com';
const SYSTEM_CA = '/etc/ssl/certs/ca-certificates.crt';
const MIME = {
  '.html': 'text/html; charset=utf-8', '.js': 'application/javascript; charset=utf-8',
  '.mjs': 'application/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8',
  '.json': 'application/json', '.wasm': 'application/wasm', '.png': 'image/png',
  '.jpg': 'image/jpeg', '.jpeg': 'image/jpeg', '.webp': 'image/webp', '.svg': 'image/svg+xml',
  '.ico': 'image/x-icon', '.csv': 'text/csv; charset=utf-8', '.yaml': 'text/yaml; charset=utf-8',
};
const PRIVATE_NAMES = new Set(['server.js', 'package.json', 'package-lock.json']);

function allowedFile(relative, dataset = false) {
  const parts = relative.split(path.sep);
  if (parts.some(part => !part || part.startsWith('.') || PRIVATE_NAMES.has(part.toLowerCase()))) return false;
  const route = parts.join('/');
  if (dataset) return /^(?:cam[01]|imu[01]|mocap0)\/(?:data\.csv|sensor\.yaml|data\/\d+\.png)$/.test(route);
  if (['index.html', 'test-tumvi.html', 'audit-mobile.html', 'vio_engine.js', 'vio_engine.wasm', 'favicon.ico'].includes(route)) return true;
  if (/^js\/.+\.(?:js|mjs)$/.test(route)) return true;
  if (/^css\/.+\.css$/.test(route)) return true;
  if (/^(?:images|public)\/.+\.(?:json|css|png|jpg|jpeg|webp|svg|ico)$/.test(route)) return true;
  return false;
}

function decodeRoute(raw) {
  const encoded = String(raw || '').split('?', 1)[0];
  if (!encoded.startsWith('/') || encoded.startsWith('//')) throw Object.assign(new Error('Bad Request'), { status: 400 });
  let route;
  try { route = decodeURIComponent(encoded); }
  catch { throw Object.assign(new Error('Bad Request'), { status: 400 }); }
  if (/[\x00-\x1f\x7f%\\]/.test(route)) throw Object.assign(new Error('Bad Request'), { status: 400 });
  if (route.split('/').some(part => part.startsWith('.'))) throw Object.assign(new Error('Forbidden'), { status: 403 });
  return route === '/' ? '/index.html' : route;
}

function hashArtifact(filename, root) {
  try {
    const physical = fs.realpathSync(filename);
    const relative = path.relative(fs.realpathSync(root), physical);
    if (relative.startsWith('..') || path.isAbsolute(relative) || !allowedFile(relative)) return { available: false };
    const data = fs.readFileSync(physical);
    return { bytes: data.length, sha256: crypto.createHash('sha256').update(data).digest('hex') };
  } catch (error) {
    if (error.code === 'ENOENT') return { available: false };
    throw error;
  }
}

function createApp(options = {}) {
  const webRoot = path.resolve(options.webRoot || path.join(ROOT, 'web'));
  const hostname = options.hostname || HOSTNAME;
  const datasetRoots = options.datasetRoots || {
    room1: path.join(ROOT, 'assets/datasets/tum/dataset-room1_512_16/mav0'),
    room4: path.resolve(process.env.MOBILE_SLAM_ROOM4_ROOT || path.join(ROOT, 'build/refactor-data/tum/dataset-room4_512_16/mav0')),
  };
  const deploymentId = options.deploymentId || process.env.MOBILE_SLAM_DEPLOYMENT_ID || 'unlabelled';
  if (!/^[A-Za-z0-9._-]{1,128}$/.test(deploymentId)) throw new Error('Invalid deployment identifier');
  const startedAt = new Date().toISOString();
  const singleFileEmbeddedWasm = options.singleFileEmbeddedWasm ?? (process.env.MOBILE_SLAM_SINGLE_FILE_WASM === '1');
  const sourceManifestSha256 = process.env.MOBILE_SLAM_SOURCE_MANIFEST_SHA256 || null;
  if (sourceManifestSha256 && !/^[a-f0-9]{64}$/.test(sourceManifestSha256)) throw new Error('Invalid source manifest digest');
  const serverSourceSha256 = crypto.createHash('sha256').update(fs.readFileSync(__filename)).digest('hex');
  const artifactSnapshot = { 'vio_engine.js': hashArtifact(path.join(webRoot, 'vio_engine.js'), webRoot),
    'vio_engine.wasm': singleFileEmbeddedWasm ? { available: false, active: false, reason: 'embedded-in-loader' }
      : hashArtifact(path.join(webRoot, 'vio_engine.wasm'), webRoot) };
  const headers = {
    'Cross-Origin-Opener-Policy': 'same-origin', 'Cross-Origin-Embedder-Policy': 'credentialless',
    'Permissions-Policy': 'camera=(self), accelerometer=(self), gyroscope=(self)',
    'X-Content-Type-Options': 'nosniff', 'Cache-Control': 'no-store, no-cache, must-revalidate',
    Pragma: 'no-cache', Expires: '0', 'Strict-Transport-Security': 'max-age=86400',
  };

  function send(req, res, status, body, type = 'text/plain; charset=utf-8') {
    const bytes = Buffer.from(body);
    res.writeHead(status, { ...headers, 'Content-Type': type, 'Content-Length': bytes.length });
    res.end(req.method === 'HEAD' ? undefined : bytes);
  }

  return async function handleRequest(req, res) {
    if (!['GET', 'HEAD'].includes(req.method)) {
      res.setHeader('Allow', 'GET, HEAD');
      res.setHeader('Connection', 'close');
      send(req, res, 405, 'Read-only server');
      return;
    }
    const host = String(req.headers.host || '').toLowerCase();
    if (!(host === hostname || new RegExp(`^${hostname.replaceAll('.', '\\.')}:[0-9]{1,5}$`).test(host))) {
      send(req, res, 421, 'Misdirected Request'); return;
    }
    if (req.headers['transfer-encoding'] || Number(req.headers['content-length'] || 0) !== 0) {
      res.setHeader('Connection', 'close'); send(req, res, 400, 'Bad Request'); return;
    }
    let handle, route;
    try {
      route = decodeRoute(req.url);
      if (route === '/__health__' || route === '/__audit-health__') {
        send(req, res, 200, JSON.stringify({
          schema: 'mobile-slam-https-health-v1', service: 'mobile-slam-https', pid: process.pid,
          instanceId: process.env.MOBILE_SLAM_SERVER_INSTANCE || 'direct', startedAt,
          uptimeSeconds: Math.round(process.uptime()), deploymentId, artifactSnapshot,
          wasmMode: singleFileEmbeddedWasm ? 'single-file-embedded' : 'external-paired',
          source: { serverSha256: serverSourceSha256, manifestSha256: sourceManifestSha256 },
          tls: options.tlsMetadata || null, remoteLogs: 'disabled',
        }), 'application/json');
        return;
      }
      let requestedRoot = webRoot;
      let relative = route.slice(1);
      let dataset = false;
      if (route.startsWith('/datasets/')) {
        const match = /^\/datasets\/tum\/dataset-(room[14])_512_16\/mav0\/(.+)$/.exec(route);
        if (!match || !datasetRoots[match[1]]) throw Object.assign(new Error('Forbidden'), { status: 403 });
        requestedRoot = path.resolve(datasetRoots[match[1]]); relative = match[2]; dataset = true;
      }
      if (!allowedFile(relative, dataset)) throw Object.assign(new Error('Forbidden'), { status: 403 });
      if (!dataset && relative === 'vio_engine.wasm' && singleFileEmbeddedWasm) throw Object.assign(new Error('Forbidden'), { status: 403 });
      const realRoot = await fs.promises.realpath(requestedRoot);
      if (dataset && options.datasetBoundary) {
        const local = path.relative(options.datasetBoundary, realRoot);
        if (local.startsWith('..') || path.isAbsolute(local)) throw Object.assign(new Error('Forbidden'), { status: 403 });
      }
      const filename = await fs.promises.realpath(path.join(realRoot, relative));
      const actual = path.relative(realRoot, filename);
      if (actual.startsWith('..') || path.isAbsolute(actual) || !allowedFile(actual, dataset)
          || (!dataset && singleFileEmbeddedWasm && actual === 'vio_engine.wasm')) {
        throw Object.assign(new Error('Forbidden'), { status: 403 });
      }
      handle = await fs.promises.open(filename, fs.constants.O_RDONLY | fs.constants.O_NOFOLLOW);
      // Recheck the opened descriptor, so a path swap cannot bypass the realpath boundary.
      const opened = await fs.promises.realpath(`/proc/self/fd/${handle.fd}`);
      const openedRelative = path.relative(realRoot, opened);
      if (openedRelative.startsWith('..') || path.isAbsolute(openedRelative) || !allowedFile(openedRelative, dataset)
          || (!dataset && singleFileEmbeddedWasm && openedRelative === 'vio_engine.wasm')) {
        throw Object.assign(new Error('Forbidden'), { status: 403 });
      }
      const stat = await handle.stat();
      if (!stat.isFile()) throw Object.assign(new Error('Forbidden'), { status: 403 });
      res.writeHead(200, { ...headers, 'Content-Type': MIME[path.extname(filename).toLowerCase()], 'Content-Length': stat.size });
      if (req.method === 'HEAD') { await handle.close(); handle = null; res.end(); return; }
      const stream = handle.createReadStream(); handle = null;
      stream.on('error', () => res.destroy());
      res.on('close', () => stream.destroy());
      stream.pipe(res);
    } catch (error) {
      if (handle) await handle.close().catch(() => {});
      if (res.headersSent) { res.destroy(); return; }
      if (route === '/favicon.ico' && error.code === 'ENOENT') {
        try { await fs.promises.lstat(path.join(webRoot, 'favicon.ico')); }
        catch (missing) {
          if (missing.code === 'ENOENT') { res.writeHead(204, headers); res.end(); return; }
        }
      }
      const status = error.status || (['ENOENT', 'ENOTDIR'].includes(error.code) ? 404 : ['EACCES', 'ELOOP'].includes(error.code) ? 403 : 500);
      send(req, res, status, ({ 400: 'Bad Request', 403: 'Forbidden', 404: 'Not Found' })[status] || 'Internal Server Error');
    }
  };
}

function loadTlsOptions(options = {}) {
  const keyPath = options.keyPath || path.join(ROOT, 'assets/keys/serdic_com.key');
  const certPath = options.certPath || path.join(ROOT, 'assets/keys/serdic_com_cert.crt');
  const chainPath = options.chainPath || path.join(ROOT, 'assets/keys/serdic_com_chain_cert.crt');
  const hostname = options.hostname || HOSTNAME;
  for (const filename of [keyPath, certPath, chainPath]) {
    const parent = fs.lstatSync(path.dirname(filename));
    if (!parent.isDirectory() || (parent.mode & 0o022)) throw new Error('TLS directory must not be group/world writable');
    const stat = fs.lstatSync(filename);
    if (!stat.isFile() || (stat.mode & 0o022)) throw new Error('TLS files must be regular and not group/world writable');
    if (filename === keyPath && ((stat.mode & 0o077) || stat.uid !== process.getuid())) throw new Error('TLS key must be owner-only');
  }
  const leafBytes = fs.readFileSync(certPath);
  const chainBytes = fs.readFileSync(chainPath);
  const pem = /-----BEGIN CERTIFICATE-----[\s\S]+?-----END CERTIFICATE-----/g;
  const leafBlocks = leafBytes.toString().match(pem) || [];
  const chainBlocks = chainBytes.toString().match(pem) || [];
  if (leafBlocks.length !== 1 || chainBlocks.length < 1) throw new Error('TLS requires one leaf and ordered intermediates');
  const certificates = [...leafBlocks, ...chainBlocks].map(block => new crypto.X509Certificate(block));
  const leaf = certificates[0];
  if (leaf.ca || !leaf.checkHost(hostname, { subject: 'never' })) throw new Error('TLS leaf SAN mismatch');
  const key = fs.readFileSync(keyPath);
  if (!leaf.checkPrivateKey(crypto.createPrivateKey(key))) throw new Error('TLS key mismatch');
  for (let i = 0; i < certificates.length; i++) {
    const current = certificates[i];
    if (!(Date.parse(current.validFrom) <= Date.now() && Date.now() <= Date.parse(current.validTo))) throw new Error('TLS certificate not currently valid');
    if (i > 0 && (!current.ca || (current.subject === current.issuer && current.verify(current.publicKey)))) throw new Error('TLS chain must exclude self-signed root');
    if (i + 1 < certificates.length && (!current.checkIssued(certificates[i + 1]) || !current.verify(certificates[i + 1].publicKey))) throw new Error('TLS intermediate chain order/signature invalid');
  }
  try {
    execFileSync('openssl', ['verify', '-purpose', 'sslserver', '-verify_hostname', hostname, '-CAfile', SYSTEM_CA, '-untrusted', chainPath, certPath], { timeout: 5000, stdio: ['ignore', 'pipe', 'pipe'] });
  } catch { throw new Error('TLS system trust verification failed'); }
  return { key, cert: Buffer.concat([leafBytes, Buffer.from('\n'), chainBytes]), minVersion: 'TLSv1.2',
    metadata: { hostname, leafSha256: leaf.fingerprint256.replaceAll(':', '').toLowerCase(), intermediateCount: chainBlocks.length, validUntil: leaf.validTo } };
}

function createServer(options = {}) {
  const { metadata, ...tls } = loadTlsOptions(options);
  const server = https.createServer({ ...tls, maxHeaderSize: 8192 }, createApp({ ...options, datasetBoundary: ROOT, tlsMetadata: metadata }));
  server.requestTimeout = 15000; server.headersTimeout = 10000; server.keepAliveTimeout = 5000;
  server.maxHeadersCount = 64; server.maxRequestsPerSocket = 100;
  return server;
}

function parseArguments(argv) {
  const options = { port: 7002, host: '0.0.0.0', hostname: HOSTNAME };
  for (let i = 0; i < argv.length; i++) {
    if (i === 0 && /^\d+$/.test(argv[i])) { options.port = Number(argv[i]); continue; }
    if (!['--port', '--host', '--hostname'].includes(argv[i]) || !argv[i + 1]) throw new Error('Use [port] or --port/--host/--hostname');
    const key = argv[i].slice(2); options[key] = key === 'port' ? Number(argv[++i]) : argv[++i];
  }
  if (!Number.isInteger(options.port) || options.port < 0 || options.port > 65535) throw new Error('Invalid port');
  if (!['0.0.0.0', '127.0.0.1'].includes(options.host)) throw new Error('Use an explicit IPv4 local host');
  if (options.hostname !== HOSTNAME) throw new Error('Public hostname is dev.serdic.com');
  return options;
}

module.exports = { createApp, createServer, loadTlsOptions, parseArguments };

if (require.main === module) {
  try {
    const options = parseArguments(process.argv.slice(2));
    const server = createServer(options);
    server.on('error', error => { console.error(JSON.stringify({ event: 'startup_error', code: error.code || 'SERVER_ERROR' })); process.exitCode = 1; });
    server.listen(options.port, options.host, () => console.log(JSON.stringify({ event: 'listening', pid: process.pid, host: options.host, port: server.address().port, url: `https://${HOSTNAME}:${server.address().port}/`, remoteLogs: 'disabled' })));
    let closing = false;
    for (const signal of ['SIGTERM', 'SIGINT']) process.on(signal, () => {
      if (closing) return; closing = true;
      server.close(() => { console.log(JSON.stringify({ event: 'stopped' })); });
      setTimeout(() => server.closeAllConnections(), 3000).unref();
    });
  } catch (error) { console.error(JSON.stringify({ event: 'startup_error', reason: error.message })); process.exitCode = 1; }
}
