/** Offline replay of preserved Android grayscale/IMU through the current browser Worker. */
import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import { createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { parseArgs } from 'node:util';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const MAX_REPLAY_FRAMES = 2000;
const MAX_RAW_IMAGE_BYTES = 512 * 1024 * 1024;
const sha256 = bytes => createHash('sha256').update(bytes).digest('hex');
const inside = (directory, file) => file === directory || file.startsWith(directory + path.sep);

/** P5 only; consume the header delimiter without stripping whitespace-valued pixels. */
export function parsePGM(bytes, width, height) {
    let offset = 0;
    const whitespace = value => [9, 10, 13, 32].includes(value);
    const token = () => {
        while (offset < bytes.length) {
            if (whitespace(bytes[offset])) { offset++; continue; }
            if (bytes[offset] !== 35) break;
            while (offset < bytes.length && bytes[offset] !== 10) offset++;
        }
        const start = offset;
        while (offset < bytes.length && !whitespace(bytes[offset]) && bytes[offset] !== 35) offset++;
        return bytes.subarray(start, offset).toString('ascii');
    };
    const magic = token(), columns = token(), rows = token(), maximum = token();
    if (magic !== 'P5' || columns !== String(width) || rows !== String(height) || maximum !== '255') {
        throw new Error('Expected P5 8-bit grayscale with recorded dimensions');
    }
    if (!whitespace(bytes[offset])) throw new Error('Missing PGM raster delimiter');
    if (bytes[offset] === 13 && bytes[offset + 1] === 10) offset++;
    offset++;
    if (bytes.length - offset !== width * height) throw new Error('PGM payload size mismatch');
    return bytes.subarray(offset);
}

export function validateRecording(document, frameLimit = 300) {
    const recording = document?.recording;
    if (document?.schema !== 'mobile-slam-android-input-v1' || document.completed !== true || !recording) throw new Error('Completed Android input record required');
    if (!Number.isInteger(frameLimit) || frameLimit < 1 || frameLimit > MAX_REPLAY_FRAMES) throw new Error(`frames must be 1..${MAX_REPLAY_FRAMES}`);
    const { width, height, config, frames, imu } = recording;
    if (![width, height].every(value => Number.isInteger(value) && value > 0) || width * height > 16777216 || config?.width !== width || config?.height !== height) throw new Error('Invalid recording dimensions/config');
    if (recording.profile !== 'mobile_default') throw new Error('Only preserved mobile_default parameter profile is supported');
    if (!Array.isArray(frames) || !frames.length || !Array.isArray(imu) || !imu.length) throw new Error('Image and IMU input required');
    if (recording.counters?.imuOverflow || recording.counters?.resultOverflow) throw new Error('Recording overflow prevents complete prefix replay');
    let previous = -Infinity;
    for (const frame of frames) {
        if (!Number.isFinite(frame.timestamp) || frame.timestamp < 0 || frame.timestamp <= previous ||
            typeof frame.file !== 'string' || path.basename(frame.file) !== frame.file || !/^[a-f0-9]{64}$/.test(frame.sha256)) throw new Error('Invalid frame metadata or ordering');
        previous = frame.timestamp;
    }
    previous = -Infinity;
    for (const row of imu) {
        if (!Array.isArray(row) || row.length !== 7 || !row.every(Number.isFinite) || row[0] < 0 || row[0] <= previous) throw new Error('Invalid IMU sample or ordering');
        previous = row[0];
    }
    const prefix = frames.slice(0, frameLimit);
    if (width * height * prefix.length > MAX_RAW_IMAGE_BYTES) throw new Error('Selected raw image prefix exceeds 512 MiB budget');
    for (const [index, frame] of prefix.entries()) {
        if (frame.index !== index) throw new Error('Saved frames are not a contiguous accepted prefix');
    }
    if (recording.acceptedFrameMetadata !== undefined) {
        const accepted = recording.acceptedFrameMetadata;
        if (!Array.isArray(accepted) || accepted.length !== recording.counters?.acceptedFrames || accepted.length !== frames.length) throw new Error('Streamed record has missing accepted-frame payloads');
        for (const [index, frame] of frames.entries()) {
            if (accepted[index]?.index !== index || frame.index !== index || accepted[index].timestamp !== frame.timestamp || accepted[index].payloadStatus !== 'saved') throw new Error('Streamed record has noncontiguous or unmatched saved payloads');
        }
    }
    if (imu[0][0] > prefix[0].timestamp || imu.at(-1)[0] < prefix.at(-1).timestamp) throw new Error('Input prefix lacks past/future IMU bracket');
    return { recording, frames: prefix };
}

export function summarizeResults(rows) {
    const fresh = rows.filter(row => row.poseFresh && row.poseValid && Number.isFinite(row.poseTimestamp) &&
        row.pose?.length === 16 && row.pose.every(Number.isFinite));
    const reasons = {};
    for (const row of rows) reasons[row.reason] = (reasons[row.reason] || 0) + 1;
    const timing = key => {
        const values = rows.map(row => row[key]).filter(Number.isFinite).sort((a, b) => a - b);
        const percentile = p => values[Math.max(0, Math.ceil(values.length * p) - 1)] ?? null;
        return { count: values.length, p50: percentile(0.5), p95: percentile(0.95), p99: percentile(0.99), max: values.at(-1) ?? null };
    };
    const jumps = [];
    for (let i = 1; i < fresh.length; i++) if (fresh[i].engineEpoch === fresh[i - 1].engineEpoch) {
        jumps.push(Math.hypot(...[3, 7, 11].map(index => fresh[i].pose[index] - fresh[i - 1].pose[index])));
    }
    return { completedFrames: rows.length, initializedFrames: rows.filter(row => row.initialized).length,
        freshPoseFrames: fresh.length, freshPoseCoverage: rows.length ? fresh.length / rows.length : 0,
        firstFreshPoseFrame: rows.findIndex(row => row === fresh[0]),
        maxPositionNormM: fresh.length ? Math.max(...fresh.map(row => Math.hypot(row.pose[3], row.pose[7], row.pose[11]))) : null,
        maxConsecutivePublishedTranslationM: jumps.length ? Math.max(...jumps) : null,
        observedEpochTransitions: rows.slice(1).filter((row, index) => row.engineEpoch !== rows[index].engineEpoch).length,
        motionGuardResets: rows.filter(row => row.reason === 'pose_translation_discontinuity').length,
        reasons, engineProcessingMs: timing('engineProcessingMs'), roundTripMs: timing('roundTripMs') };
}

/** Each sample is submitted once; an already submitted future bracket covers later frames. */
export function imuBatchRanges(frames, imu) {
    let cursor = 0;
    return frames.map(frame => {
        const start = cursor;
        while (cursor < imu.length && imu[cursor][0] <= frame.timestamp) cursor++;
        if (cursor < imu.length && (cursor === 0 || imu[cursor - 1][0] <= frame.timestamp)) cursor++;
        return { start, count: cursor - start };
    });
}

export async function main(argv = process.argv.slice(2)) {
    const { values } = parseArgs({ args: argv, options: {
        record: { type: 'string' }, 'input-dir': { type: 'string' }, loader: { type: 'string' }, output: { type: 'string' },
        frames: { type: 'string', default: '300' }, 'timeout-ms': { type: 'string', default: '300000' },
    } });
    if (!values.record || !values['input-dir'] || !values.loader || !values.output) throw new Error('Required: --record record.json --input-dir mav0/cam0/data --loader candidate/vio_engine.js --output new-directory [--frames 300]');
    const timeoutMs = Number(values['timeout-ms']);
    if (!Number.isInteger(timeoutMs) || timeoutMs < 1000 || timeoutMs > 600000) throw new Error('timeout-ms must be 1000..600000');
    const recordPath = await fs.realpath(values.record), inputDirectory = await fs.realpath(values['input-dir']);
    const loaderPath = await fs.realpath(values.loader);
    const outputRequested = path.resolve(values.output);
    // Resolve the existing ancestor before creating anything in a requested path.
    let ancestor = path.dirname(outputRequested);
    let canonicalAncestor;
    for (;;) {
        try { canonicalAncestor = await fs.realpath(ancestor); break; }
        catch (error) { if (error.code !== 'ENOENT') throw error; ancestor = path.dirname(ancestor); }
    }
    const output = path.resolve(canonicalAncestor, path.relative(ancestor, outputRequested));
    if (inside(path.dirname(recordPath), output) || inside(inputDirectory, output)) throw new Error('Output must be outside preserved input directories');
    const recordBytes = await fs.readFile(recordPath);
    const { recording, frames } = validateRecording(JSON.parse(recordBytes), Number(values.frames));
    const imuBatches = imuBatchRanges(frames, recording.imu);
    const report = { schema: 'mobile-slam-android-replay-v1', createdAt: new Date().toISOString(), completed: false,
        completionMeaning: 'All requested saved frames returned; initialization, lossless IMU consumption and accuracy are separate outcomes.',
        input: { recordPath, recordSha256: sha256(recordBytes), storedFrames: recording.frames.length,
            requestedFrames: Number(values.frames), replayFrames: frames.length,
            rawImageBytes: recording.width * recording.height * frames.length,
            rawImageBudgetBytes: MAX_RAW_IMAGE_BYTES,
            acceptedFramesInRecording: recording.counters?.acceptedFrames ?? null,
            payloadScope: recording.acceptedFrameMetadata ? 'streamed_record_saved_prefix' : 'legacy_first_saved_prefix',
            estimatorLifecycle: recording.estimatorLifecycle ?? null,
            firstTimestampS: frames[0].timestamp, lastTimestampS: frames.at(-1).timestamp,
            durationS: frames.at(-1).timestamp - frames[0].timestamp, recordingDurationS: (recording.stoppedAtMs - recording.startedAtMs) / 1000 },
        config: recording.config, parameters: { mobile: [0.06, 10, 120], tracking: [21, 3, 30, 0], pnp: [false, 3] },
        calibration: recording.calibration, fingerprints: {}, images: [], requests: [], console: [], pageErrors: [],
        limits: ['Saved image prefix only, not the full recording or unseen later failure interval.',
            'Fresh engine lifecycle; pre-record estimator state is absent. Recorded double timestamps and body IMU are unchanged.',
            'Serial desktop replay with a pre-submitted future IMU bracket; live arrival timing, drops and phone performance are not reproduced.',
            'mobile_default solver/tracking parameters reconstructed from app defaults; recorder did not persist parameter RPCs.',
            'Heuristic camera calibration, physical exposure timing, sensor synchronization and independent trajectory accuracy remain unverified.'] };
    const routes = new Map([['/', { mime: 'text/html', bytes: Buffer.from('<!doctype html><title>Preserved Android input replay</title>') }]]);
    for (const [url, file] of [['/candidate/vio_engine.js', loaderPath], ['/js/vio-worker.js', path.join(root, 'web/js/vio-worker.js')], ['/js/vio-wrapper.js', path.join(root, 'web/js/vio-wrapper.js')]]) {
        const bytes = await fs.readFile(file);
        report.fingerprints[url] = { path: file, bytes: bytes.length, sha256: sha256(bytes) };
        routes.set(url, { bytes, mime: 'application/javascript' });
    }
    for (const file of [fileURLToPath(import.meta.url), path.join(root, 'web/js/app.js')]) {
        const bytes = await fs.readFile(file);
        report.fingerprints[path.relative(root, file)] = { path: file, bytes: bytes.length, sha256: sha256(bytes) };
    }
    for (const [index, frame] of frames.entries()) {
        const file = await fs.realpath(path.join(inputDirectory, frame.file));
        if (!inside(inputDirectory, file)) throw new Error('Image link escapes input directory');
        const pgm = await fs.readFile(file), gray = parsePGM(pgm, recording.width, recording.height);
        const graySha256 = sha256(gray);
        if (graySha256 !== frame.sha256) throw new Error(`Recorded grayscale hash mismatch: ${frame.file}`);
        report.images.push({ index, recordIndex: frame.index, file: frame.file, timestamp: frame.timestamp,
            sourceTiming: frame.sourceTiming, pgmSha256: sha256(pgm), graySha256 });
        routes.set(`/images/${index}`, { bytes: gray, mime: 'application/octet-stream' });
    }
    await fs.mkdir(path.dirname(output), { recursive: true });
    await fs.mkdir(output, { recursive: false });
    const server = http.createServer((request, response) => {
        response.setHeader('Cross-Origin-Opener-Policy', 'same-origin');
        response.setHeader('Cross-Origin-Embedder-Policy', 'credentialless');
        response.setHeader('Cache-Control', 'no-store');
        if (request.url === '/favicon.ico') { response.writeHead(204).end(); return; }
        const item = routes.get(request.url);
        report.requests.push({ path: request.url, status: item ? 200 : 404 });
        if (!item) { response.writeHead(404).end(); return; }
        response.setHeader('Content-Type', item.mime); response.end(item.bytes);
    });
    let browser, timeout;
    try {
        await new Promise((resolve, reject) => { server.once('error', reject); server.listen(0, '127.0.0.1', resolve); });
        const { chromium } = await import('playwright');
        browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true, args: ['--enable-unsafe-swiftshader'] });
        report.browser = browser.version(); report.node = process.version;
        const page = await browser.newPage();
        page.on('console', message => report.console.push({ type: message.type(), text: message.text() }));
        page.on('pageerror', error => report.pageErrors.push(error.message));
        await page.goto(`http://127.0.0.1:${server.address().port}/`);
        report.replay = await Promise.race([page.evaluate(async ({ frames, imu, imuBatches, config, parameters }) => {
            const { VIOWrapper } = await import('/js/vio-wrapper.js');
            const vio = new VIOWrapper({ frameTimeoutMs: 60000 });
            const rows = [];
            let imuCursor = 0;
            try {
                await vio.load('/candidate/vio_engine.js');
                if (!await vio.configure(config)) throw new Error('Recorded config rejected');
                const applied = [await vio.setMobileParams(...parameters.mobile), await vio.setTrackingParams(...parameters.tracking), await vio.setPnPParams(...parameters.pnp)];
                if (!applied.every(Boolean)) throw new Error('Mobile replay parameters rejected');
                for (const [index, frame] of frames.entries()) {
                    const { start, count } = imuBatches[index];
                    imuCursor = start + count;
                    if (count > 0) {
                        const batch = new Float64Array(imu.slice(start, imuCursor).flat());
                        if (!vio.sendIMU(batch, count)) throw new Error(`IMU submission rejected at frame ${index}`);
                    }
                    const response = await fetch(`/images/${index}`);
                    if (!response.ok) throw new Error(`Image fetch failed at frame ${index}`);
                    const gray = new Uint8Array(await response.arrayBuffer());
                    const started = performance.now();
                    if (!vio.sendFrame(gray, frame.timestamp)) throw new Error(`Frame ${index} rejected`);
                    await vio.waitForFree(60000);
                    const result = vio.getLatestResult();
                    if (!result || result.inputTimestamp !== frame.timestamp) throw new Error(`Missing matching result at frame ${index}`);
                    const { mapPoints, ...fields } = result;
                    rows.push({ ...fields, index, submittedIMUStart: start, submittedIMUCount: count, roundTripMs: performance.now() - started,
                        pose: result.pose ? Array.from(result.pose) : null });
                }
                return { crossOriginIsolated, secureContext: isSecureContext, rows, imuSamplesSubmitted: imuCursor, transport: vio.getMetrics() };
            } finally { vio.dispose(); }
        }, { frames, imu: recording.imu, imuBatches, config: recording.config, parameters: report.parameters }),
        new Promise((_, reject) => { timeout = setTimeout(() => reject(new Error('Bounded replay timeout')), timeoutMs); })]);
        report.summary = summarizeResults(report.replay.rows);
        const drops = report.replay.transport.imuDrops || {};
        report.inputDelivery = {
            workerDrops: Object.fromEntries(['overflow', 'stale', 'capacity', 'invalid', 'order', 'invalidBatches'].map(key => [key, drops[key] ?? null])),
            missingBracketFrames: report.replay.rows.filter(row => row.reason === 'missing_bracket').length,
            engineLifecycleDiscardedSamples: drops.reset ?? null,
        };
        report.inputDelivery.noTransportLossObserved = Object.values(report.inputDelivery.workerDrops).every(value => value === 0) && report.inputDelivery.missingBracketFrames === 0;
        if (report.pageErrors.length || report.requests.some(request => request.status !== 200) ||
            report.replay.transport.framesCompleted !== frames.length || report.replay.transport.framesTimedOut ||
            report.replay.transport.workerErrors || report.replay.transport.imuReadingsRejected) throw new Error('Replay completeness/transport checks failed');
        report.completed = true;
    } catch (error) { report.error = error.stack; process.exitCode = 1; }
    finally {
        clearTimeout(timeout);
        if (browser) await browser.close();
        if (server.listening) await new Promise(resolve => server.close(resolve));
        await fs.writeFile(path.join(output, 'report.json'), JSON.stringify(report, null, 2) + '\n', { flag: 'wx' });
    }
    console.log(JSON.stringify({ output, completed: report.completed, summary: report.summary, error: report.error }, null, 2));
    return report;
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
    main().catch(error => { console.error(error.stack); process.exitCode = 1; });
}
