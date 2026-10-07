/** Bounded check of the TUM app's createImageBitmap/canvas/R-channel decode path. */
import http from 'node:http';
import fs from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createRequire } from 'node:module';
import { createHash } from 'node:crypto';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const { chromium } = createRequire(path.join(root, 'scripts/dev/package.json'))('playwright');
const dataset = path.join(root, 'assets/datasets/tum/dataset-room1_512_16/mav0/cam0');
const entries = (await fs.readFile(path.join(dataset, 'data.csv'), 'utf8')).split('\n').filter(line => line.trim() && !line.startsWith('#'));
const selected = [];
for (const index of [0, 45, 600]) {
    const filename = entries[index].split(',')[1].trim();
    const source = path.join(dataset, 'data', filename);
    const png = await fs.readFile(source);
    selected.push({ index, source, png, pngSha256: createHash('sha256').update(png).digest('hex') });
}
const server = http.createServer((req, res) => {
    const match = /^\/image\/(\d+)$/.exec(req.url);
    if (match && selected[Number(match[1])]) { res.setHeader('Content-Type', 'image/png'); res.end(selected[Number(match[1])].png); }
    else { res.setHeader('Content-Type', 'text/html'); res.end('<!doctype html><title>PNG decode audit</title>'); }
});
await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
let browser;
try {
    browser = await chromium.launch({ headless: true, executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome' });
    const page = await browser.newPage();
    await page.goto(`http://127.0.0.1:${server.address().port}`);
    const output = path.join(root, 'build/audit-browser/image-decode');
    await fs.mkdir(output, { recursive: true });
    const rows = [];
    for (let i = 0; i < selected.length; i++) {
        const decoded = await page.evaluate(async index => {
            const blob = await (await fetch('/image/' + index)).blob();
            const bitmap = await createImageBitmap(blob);
            const canvas = new OffscreenCanvas(bitmap.width, bitmap.height);
            const ctx = canvas.getContext('2d');
            ctx.drawImage(bitmap, 0, 0);
            bitmap.close();
            const rgba = ctx.getImageData(0, 0, canvas.width, canvas.height).data;
            const gray = new Uint8Array(canvas.width * canvas.height);
            for (let p = 0; p < gray.length; p++) gray[p] = rgba[p * 4];
            return { width: canvas.width, height: canvas.height, gray: Array.from(gray) };
        }, i);
        const bytes = Buffer.from(decoded.gray);
        const binary = path.join(output, `frame-${selected[i].index}-gray-u8.bin`);
        await fs.writeFile(binary, bytes);
        rows.push({ index: selected[i].index, source: selected[i].source, pngSha256: selected[i].pngSha256, width: decoded.width, height: decoded.height, binary, decodedSha256: createHash('sha256').update(bytes).digest('hex') });
    }
    const report = { browser: browser.version(), decode: 'createImageBitmap default options; OffscreenCanvas 2d; getImageData R channel', frames: rows, limits: 'Three sampled PNGs only; not a camera photometric calibration or all-frame parity guarantee.' };
    await fs.writeFile(path.join(output, 'report.json'), JSON.stringify(report, null, 2) + '\n');
    console.log(JSON.stringify(report, null, 2));
} finally {
    if (browser) await browser.close();
    await new Promise(resolve => server.close(resolve));
}
