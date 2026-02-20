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

// Static file handler
function handleRequest(req, res) {
    let filePath = path.join(__dirname, req.url === '/' ? '/index.html' : req.url);

    // Security: prevent directory traversal
    if (!filePath.startsWith(__dirname)) {
        res.writeHead(403);
        res.end('Forbidden');
        return;
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
