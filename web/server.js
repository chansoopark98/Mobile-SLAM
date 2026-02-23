/**
 * HTTPS development server for Mobile VIO.
 * HTTPS is required for camera/IMU access on mobile devices.
 *
 * Usage:
 *   node server.js [port]
 *
 * Uses real SSL certificates from assets/keys/ if available,
 * otherwise falls back to self-signed certificates.
 */
const https = require('https');
const http = require('http');
const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

const PORT = parseInt(process.argv[2]) || 2224;

// SSL certificate paths - prefer real certs from assets/keys/
const PROJECT_ROOT = path.resolve(__dirname, '..');
const REAL_CERT_FILE = path.join(PROJECT_ROOT, 'assets/keys/serdic_com_cert.crt');
const REAL_KEY_FILE = path.join(PROJECT_ROOT, 'assets/keys/serdic_com.key');
const SELF_CERT_DIR = path.join(__dirname, '.certs');
const SELF_CERT_FILE = path.join(SELF_CERT_DIR, 'cert.pem');
const SELF_KEY_FILE = path.join(SELF_CERT_DIR, 'key.pem');

// Determine which certs to use
const USE_REAL_CERTS = fs.existsSync(REAL_CERT_FILE) && fs.existsSync(REAL_KEY_FILE);
const CERT_FILE = USE_REAL_CERTS ? REAL_CERT_FILE : SELF_CERT_FILE;
const KEY_FILE = USE_REAL_CERTS ? REAL_KEY_FILE : SELF_KEY_FILE;

// MIME types
const MIME = {
    '.html': 'text/html',
    '.js': 'application/javascript',
    '.mjs': 'application/javascript',
    '.css': 'text/css',
    '.json': 'application/json',
    '.wasm': 'application/wasm',
    '.png': 'image/png',
    '.jpg': 'image/jpeg',
    '.svg': 'image/svg+xml',
    '.ico': 'image/x-icon',
    '.csv': 'text/csv',
    '.yaml': 'text/yaml',
};

// Generate self-signed certificates if no real certs and no self-signed exist
function ensureCerts() {
    if (USE_REAL_CERTS) {
        console.log('Using real SSL certificate from assets/keys/');
        return;
    }

    if (fs.existsSync(SELF_CERT_FILE) && fs.existsSync(SELF_KEY_FILE)) return;

    console.log('No real certs found. Generating self-signed SSL certificate...');
    fs.mkdirSync(SELF_CERT_DIR, { recursive: true });

    execSync(`openssl req -x509 -newkey rsa:2048 -keyout "${SELF_KEY_FILE}" -out "${SELF_CERT_FILE}" \
        -days 365 -nodes -subj "/CN=localhost"`, { stdio: 'inherit' });

    console.log('Self-signed certificate generated.');
}

// Remote log file (browser console → server file)
const LOG_FILE = path.join(PROJECT_ROOT, 'logs', 'test-tumvi.log');
fs.mkdirSync(path.dirname(LOG_FILE), { recursive: true });
// Clear log on server start
fs.writeFileSync(LOG_FILE, `--- TUM VI Test Log (${new Date().toISOString()}) ---\n`);

// Static file handler
function handleRequest(req, res) {
    // POST /log — receive browser console logs and write to file
    if (req.method === 'POST' && req.url === '/log') {
        let body = '';
        req.on('data', chunk => { body += chunk; });
        req.on('end', () => {
            try {
                const { level, msg } = JSON.parse(body);
                const line = `[${new Date().toISOString()}] [${level || 'info'}] ${msg}\n`;
                fs.appendFileSync(LOG_FILE, line);
                process.stdout.write(`  [client] ${line}`);
            } catch (e) { /* ignore malformed */ }
            res.writeHead(200);
            res.end('ok');
        });
        return;
    }
    // Decode percent-encoding; reject malformed URLs
    let reqPath;
    try {
        reqPath = decodeURIComponent(req.url);
    } catch (e) {
        res.writeHead(400);
        res.end('Bad Request');
        return;
    }

    // Reject null bytes
    if (reqPath.includes('\x00')) {
        res.writeHead(400);
        res.end('Bad Request');
        return;
    }

    let filePath;
    const webDir = path.resolve(__dirname) + path.sep;
    const datasetsDir = path.resolve(PROJECT_ROOT, 'assets', 'datasets') + path.sep;

    // Route /datasets/ to PROJECT_ROOT/assets/datasets/ (for TUM VI test harness)
    if (reqPath.startsWith('/datasets/')) {
        filePath = path.join(PROJECT_ROOT, 'assets', reqPath);
        // Security: restrict to assets/datasets/ only (not assets/keys/ etc.)
        if (!path.resolve(filePath).startsWith(datasetsDir)) {
            res.writeHead(403);
            res.end('Forbidden');
            return;
        }
    } else {
        filePath = path.join(__dirname, reqPath === '/' ? '/index.html' : reqPath);
        // Security: restrict to web/ directory only
        if (!path.resolve(filePath).startsWith(webDir)) {
            res.writeHead(403);
            res.end('Forbidden');
            return;
        }
    }

    const ext = path.extname(filePath).toLowerCase();
    const contentType = MIME[ext] || 'application/octet-stream';

    fs.readFile(filePath, (err, data) => {
        if (err) {
            if (err.code === 'ENOENT') {
                res.writeHead(404);
                res.end('Not Found');
            } else {
                res.writeHead(500);
                res.end('Internal Server Error');
            }
            return;
        }

        // CORS headers
        // Note: COEP 'require-corp' would block CDN resources (Three.js).
        // Using 'credentialless' allows cross-origin loads while keeping COOP.
        // SharedArrayBuffer not needed since WASM is single-threaded.
        res.setHeader('Cross-Origin-Opener-Policy', 'same-origin');
        res.setHeader('Cross-Origin-Embedder-Policy', 'credentialless');

        res.writeHead(200, { 'Content-Type': contentType });
        res.end(data);
    });
}

// Start server
ensureCerts();

const options = {
    key: fs.readFileSync(KEY_FILE),
    cert: fs.readFileSync(CERT_FILE),
};

const server = https.createServer(options, handleRequest);
server.listen(PORT, '0.0.0.0', () => {
    console.log(`\n  Mobile VIO HTTPS Server`);
    console.log(`  ─────────────────────────`);
    console.log(`  Local:   https://localhost:${PORT}`);

    // Show network addresses for mobile access
    const nets = require('os').networkInterfaces();
    for (const name of Object.keys(nets)) {
        for (const net of nets[name]) {
            if (net.family === 'IPv4' && !net.internal) {
                console.log(`  Network: https://${net.address}:${PORT}`);
            }
        }
    }
    if (USE_REAL_CERTS) {
        console.log(`\n  SSL: Real certificate (serdic_com)\n`);
    } else {
        console.log(`\n  Note: Accept the self-signed certificate warning in your browser.\n`);
    }
});
