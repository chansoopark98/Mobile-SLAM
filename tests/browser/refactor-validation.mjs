/** Final isolated browser/native validation. No build, deployment, or service lifecycle mutations. */
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createRequire } from 'node:module';
import { createHash } from 'node:crypto';
import { spawn } from 'node:child_process';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const { chromium } = createRequire(path.join(root, 'scripts/dev/package.json'))('playwright');
const options = { phase: 'preflight', origin: 'https://dev.serdic.com:7002', output: 'build/refactor-validation',
    nativeBuild: 'build/refactor-native', wasmProvenance: 'build/refactor-wasm/build-provenance.json',
    nativeProvenance: 'build/refactor-native/native-replay-provenance.json', contract: null, pacedFrames: 300,
    benchmarkZeroTolerances: process.env.MOBILE_SLAM_BENCHMARK_ZERO_TOLERANCES ?? '0',
    room1: 'assets/datasets/tum/dataset-room1_512_16', room4: 'build/refactor-data/tum/dataset-room4_512_16',
    config: 'config/tum_vi_room1.yaml' };
const keys = { '--phase': 'phase', '--origin': 'origin', '--output': 'output', '--native-build': 'nativeBuild',
    '--wasm-provenance': 'wasmProvenance', '--native-provenance': 'nativeProvenance', '--contract': 'contract',
    '--paced-frames': 'pacedFrames', '--benchmark-zero-tolerances': 'benchmarkZeroTolerances', '--room1': 'room1', '--room4': 'room4', '--config': 'config' };
for (let i = 2; i < process.argv.length; i += 2) {
    if (!(process.argv[i] in keys) || !process.argv[i + 1]) throw new Error(`Unknown or incomplete argument: ${process.argv[i]}`);
    options[keys[process.argv[i]]] = process.argv[i + 1];
}
assert.ok(['self-test', 'preflight', 'pilot', 'room4', 'paced', 'loss', 'mock', 'all'].includes(options.phase));
assert.equal(new URL(options.origin).protocol, 'https:');
assert.equal(new URL(options.origin).hostname, 'dev.serdic.com');
assert.equal(new URL(options.origin).port, '7002');
options.pacedFrames = Number(options.pacedFrames);
assert.ok(Number.isInteger(options.pacedFrames) && options.pacedFrames >= 100 && options.pacedFrames <= 2821);
assert.ok(['0', '1'].includes(options.benchmarkZeroTolerances), 'benchmark-zero-tolerances must explicitly be 0 or 1');
options.benchmarkZeroTolerances = options.benchmarkZeroTolerances === '1';
for (const name of ['output', 'nativeBuild', 'wasmProvenance', 'nativeProvenance', 'room1', 'room4', 'config']) options[name] = path.resolve(root, options[name]);
options.contract = path.resolve(root, options.contract || path.join(options.output, 'parity-contract.json'));
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const json = filename => fs.readFile(filename, 'utf8').then(JSON.parse);
const save = async (filename, value) => { await fs.mkdir(path.dirname(filename), { recursive: true }); await fs.writeFile(filename, JSON.stringify(value, null, 2) + '\n'); };
const expected = { room1: 2821, room4: 2228 };
const notice = (phase, fields = {}) => console.log(JSON.stringify({ phase, at: new Date().toISOString(), ...fields }));
let browser;
const report = { schema: 'mobile-slam-final-virtual-validation-v1', startedAt: new Date().toISOString(), phase: options.phase,
    origin: options.origin, node: process.version, playwright: createRequire(path.join(root, 'scripts/dev/package.json'))('playwright/package.json').version,
    tlsVerification: 'enabled; no ignoreHTTPSErrors or curl -k', results: {},
    solverProfiles: { pairedNumericalRequested: options.benchmarkZeroTolerances,
        paired: { profile: options.benchmarkZeroTolerances ? 'max10_zero_positive_tolerances' : 'default', maximumIterations: 10, timeCapSeconds: 10, scope: 'Opt-in numerical parity; actual Summary iterations/termination remain unchanged.' },
        pacedAndLoss: { profile: 'default', maximumIterations: 10, timeCapSeconds: 0.1, scope: 'TUM calibrated desktop replay budget; physical phone runtime/thermal unverified.' },
        actualOptionsEvidence: 'Bounded engine diagnostic capture; ordinary replay does not infer tolerance values.' },
    limits: ['Desktop calibrated replay and fake camera/sensors; physical phone sensor/camera/exposure/thermal and field accuracy unverified.',
        'Workstation public DNS access is distinct from independent external-network reachability.', 'Dataset scheduling/roundtrip are proxies, not physical capture-to-display latency.'] };

async function command(executable, args, filename, env = {}) {
    await fs.mkdir(path.dirname(filename), { recursive: true });
    const file = await fs.open(filename, 'wx');
    const stream = file.createWriteStream();
    const child = spawn(executable, args, { cwd: root, env: { ...process.env, ...env }, stdio: ['ignore', 'pipe', 'pipe'] });
    let tail = '';
    for (const output of [child.stdout, child.stderr]) output.on('data', bytes => { stream.write(bytes); tail = (tail + bytes).slice(-4000); });
    const timer = setTimeout(() => child.kill('SIGTERM'), 1200000);
    const code = await new Promise((resolve, reject) => { child.once('error', reject); child.once('close', resolve); });
    clearTimeout(timer);
    await new Promise(resolve => stream.end(resolve));
    if (code !== 0) throw new Error(`${executable} exited ${code}: ${tail}`);
    return tail;
}

function finitePose(pose) {
    assert.equal(pose?.length, 16, 'Pose must be row-major 4×4');
    assert.ok(pose.every(Number.isFinite));
    assert.ok(pose.slice(12, 16).every((value, i) => Math.abs(value - [0, 0, 0, 1][i]) < 1e-6));
    const r = [[pose[0], pose[1], pose[2]], [pose[4], pose[5], pose[6]], [pose[8], pose[9], pose[10]]];
    let orthogonality = 0;
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
        const error = r.reduce((sum, row) => sum + row[i] * row[j], 0) - Number(i === j);
        orthogonality += error * error;
    }
    const determinant = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
    assert.ok(Math.sqrt(orthogonality) < 1e-4 && Math.abs(determinant - 1) < 1e-4, 'Pose rotation must satisfy SO(3)');
}

function quaternion(pose) {
    finitePose(pose);
    const m00 = pose[0], m01 = pose[1], m02 = pose[2], m10 = pose[4], m11 = pose[5], m12 = pose[6], m20 = pose[8], m21 = pose[9], m22 = pose[10];
    const trace = m00 + m11 + m22;
    let x, y, z, w, s;
    if (trace > 0) { s = Math.sqrt(trace + 1) * 2; w = s / 4; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s; }
    else if (m00 > m11 && m00 > m22) { s = Math.sqrt(1 + m00 - m11 - m22) * 2; w = (m21 - m12) / s; x = s / 4; y = (m01 + m10) / s; z = (m02 + m20) / s; }
    else if (m11 > m22) { s = Math.sqrt(1 + m11 - m00 - m22) * 2; w = (m02 - m20) / s; x = (m01 + m10) / s; y = s / 4; z = (m12 + m21) / s; }
    else { s = Math.sqrt(1 + m22 - m00 - m11) * 2; w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = s / 4; }
    const norm = Math.hypot(x, y, z, w);
    return [x / norm, y / norm, z / norm, w / norm];
}

async function exportTrajectory(replay, directory) {
    let lastTimestamp = -Infinity;
    const poses = replay.rows.filter(row => row.poseFresh && row.poseValid && row.pose);
    assert.ok(poses.length > 0, 'Zero fresh poses cannot establish replay/accuracy acceptance');
    const lines = poses.map(row => {
        assert.ok(Number.isFinite(row.poseTimestamp) && row.poseTimestamp > lastTimestamp, 'Actual pose timestamps must be strictly ordered');
        lastTimestamp = row.poseTimestamp;
        return [row.poseTimestamp, row.pose[3], row.pose[7], row.pose[11], ...quaternion(row.pose)].map(value => value.toFixed(12)).join(' ');
    });
    const filename = path.join(directory, 'trajectory-camera.txt');
    await fs.writeFile(filename, '# actual engine poseTimestamp seconds; camera T_W_C; tx ty tz qx qy qz qw\n' + lines.join('\n') + '\n');
    await save(filename + '.json', { poses: poses.length, expectedFrames: replay.requestedFrames, estimateFrame: 'camera',
        timestampSource: 'actual engine poseTimestamp; fresh+valid only', sha256: sha(await fs.readFile(filename)) });
    return filename;
}

function percentiles(values) {
    const sorted = values.filter(Number.isFinite).sort((a, b) => a - b);
    const value = p => sorted[Math.max(0, Math.ceil(sorted.length * p) - 1)] ?? null;
    return { count: sorted.length, p50: value(.5), p95: value(.95), p99: value(.99), max: sorted.at(-1) ?? null };
}

function summary(replay) {
    const names = ['decodeMs', 'inputHashMs', 'frameCopyMs', 'workerQueueMs', 'imuWaitMs', 'engineProcessingMs', 'workerProcessingMs', 'roundtripMs', 'totalProcessingMs', 'schedulingLatenessMs', 'deadlineToResultMs', 'poseAgeSeconds'];
    const stages = {};
    const firstInitialization = replay.rows.find(row => row.initialized)?.frame ?? Infinity;
    for (const phase of ['all', 'beforeInitialization', 'afterInitialization']) {
        const rows = replay.rows.filter(row => phase === 'all' || (phase === 'afterInitialization' ? row.frame >= firstInitialization : row.frame < firstInitialization));
        stages[phase] = Object.fromEntries(names.map(name => [name, percentiles(rows.map(row => row[name]))]));
    }
    return { counts: replay.counts, solver: replay.solver, benchmarkSolverProfile: replay.benchmarkSolverProfile,
        firstFreshFrame: replay.rows.find(row => row.poseFresh && row.poseValid)?.frame ?? null,
        wallSeconds: (replay.finishedAtMs - replay.startedAtMs) / 1000,
        datasetSeconds: replay.rows.at(-1).inputTimestamp - replay.rows[0].inputTimestamp,
        engineEpochChanges: replay.rows.filter(row => Number.isFinite(row.engineEpoch)).filter((row, i, rows) => i && row.engineEpoch !== rows[i - 1].engineEpoch).length,
        reasonCounts: replay.rows.reduce((acc, row) => { acc[row.reason] = (acc[row.reason] || 0) + 1; return acc; }, {}), stages,
        scope: replay.mode === 'max_speed_serial' ? 'Serial max-speed with hash instrumentation; throughput is not live latency.' : 'Actual source-time deadline pacing; dataset proxy, not physical camera latency.' };
}

function watch(page) {
    const events = { consoleErrors: [], pageErrors: [], failedRequests: [], badResponses: [], expectedNegative: [] };
    const listeners = {
        console: message => { if (message.type() === 'error' && events.consoleErrors.length < 200) events.consoleErrors.push({ text: message.text(), location: message.location() }); },
        pageerror: error => events.pageErrors.push(error.message),
        requestfailed: request => events.failedRequests.push({ url: request.url(), error: request.failure()?.errorText }),
        response: response => {
            if (response.status() < 400) return;
            const url = new URL(response.url());
            const entry = { url: response.url(), status: response.status(), method: response.request().method() };
            if ((url.pathname === '/favicon.ico' && response.status() === 404) || (url.pathname === '/log' && response.status() === 405)) events.expectedNegative.push(entry);
            else events.badResponses.push(entry);
        },
    };
    for (const [event, listener] of Object.entries(listeners)) page.on(event, listener);
    return { events, stop() { for (const [event, listener] of Object.entries(listeners)) page.off(event, listener); } };
}

function assertNetwork(events) {
    assert.deepEqual(events.pageErrors, [], 'New page errors');
    assert.deepEqual(events.failedRequests, [], 'Failed network requests');
    assert.deepEqual(events.badResponses, [], 'Unexpected HTTP errors');
    const allowedErrors = events.consoleErrors.filter(entry => {
        const pathname = entry.location.url ? new URL(entry.location.url, options.origin).pathname : null;
        // Chromium favicon console errors do not consistently produce Playwright response events.
        return !((pathname === '/favicon.ico' && /status of 404/.test(entry.text)) ||
            (pathname === '/log' && /status of 405/.test(entry.text)));
    });
    assert.deepEqual(allowedErrors, [], 'Unexpected console errors');
}

async function verifyProvenance(filename) {
    const value = await json(filename);
    assert.equal(value.schema, 'mobile-slam-build-v1');
    assert.equal(sha(await fs.readFile(value.artifact.path)), value.artifact.sha256, 'Built artifact hash changed');
    for (const [file, hash] of Object.entries(value.source_files)) assert.equal(sha(await fs.readFile(path.join(root, file))), hash, `Source changed after build: ${file}`);
    return value;
}

async function preflight() {
    const native = await verifyProvenance(options.nativeProvenance);
    const wasm = await verifyProvenance(options.wasmProvenance);
    assert.equal(native.source_manifest_sha256, wasm.source_manifest_sha256, 'Native/WASM source manifests differ');
    assert.equal(wasm.single_file_embedded_wasm, true);
    const context = await browser.newContext(); // Default strict certificate validation.
    const page = await context.newPage();
    const watched = watch(page);
    try {
        const response = await page.goto(options.origin + '/__health__');
        assert.equal(response.status(), 200);
        const health = await response.json();
        assert.equal(health.schema, 'mobile-slam-https-health-v1');
        assert.equal(health.artifactSnapshot['vio_engine.js'].sha256, wasm.artifact.sha256, 'Running service does not serve fresh candidate');
        const served = await page.evaluate(async paths => {
            const records = [];
            for (const pathname of paths) {
                const response = await fetch(pathname);
                const bytes = await response.arrayBuffer();
                const digest = Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', bytes)), value => value.toString(16).padStart(2, '0')).join('');
                const head = await fetch(pathname, { method: 'HEAD' });
                records.push({ pathname, status: response.status, bytes: bytes.byteLength, sha256: digest,
                    headers: Object.fromEntries(response.headers), headStatus: head.status, headBytes: (await head.arrayBuffer()).byteLength,
                    secureContext: isSecureContext, crossOriginIsolated });
            }
            return records;
        }, ['/vio_engine.js', '/js/vio-worker.js', '/js/vio-wrapper.js', '/js/app.js', '/js/test-tumvi-app.js', '/js/imu.js', '/js/camera.js', '/js/orientation.js']);
        for (const item of served) {
            assert.equal(item.status, 200); assert.equal(item.headStatus, 200); assert.equal(item.headBytes, 0);
            assert.equal(item.secureContext, true); assert.equal(item.crossOriginIsolated, true);
            assert.equal(item.headers['cross-origin-opener-policy'], 'same-origin');
            assert.equal(item.headers['cross-origin-embedder-policy'], 'credentialless');
            assert.match(item.headers['cache-control'], /no-store/);
            assert.match(item.headers['content-type'], /javascript/);
            assert.equal(item.sha256, item.pathname === '/vio_engine.js' ? wasm.artifact.sha256 : sha(await fs.readFile(path.join(root, 'web', item.pathname.slice(1)))));
        }
        const pages = [];
        for (const pathname of ['/', '/test-tumvi.html', '/audit-mobile.html']) {
            const response = await page.goto(options.origin + pathname);
            assert.equal(response.status(), 200);
            if (pathname !== '/audit-mobile.html') await page.waitForFunction(() => /WASM loaded/.test(document.getElementById('status')?.textContent) && !document.getElementById('btn-start')?.disabled, null, { timeout: 30000 });
            pages.push({ pathname, status: response.status(), secureContext: await page.evaluate(() => isSecureContext), security: await response.securityDetails() });
        }
        assertNetwork(watched.events);
        let publicDNS;
        try {
            const stdout = await command('curl', ['--fail', '--silent', '--show-error', '--connect-timeout', '5', '--max-time', '12', options.origin + '/__health__'], path.join(options.output, `public-dns-${options.phase}.log`));
            const publicHealth = JSON.parse(stdout);
            assert.equal(publicHealth.schema, health.schema);
            assert.equal(publicHealth.pid, health.pid);
            assert.equal(publicHealth.artifactSnapshot['vio_engine.js'].sha256, wasm.artifact.sha256);
            publicDNS = { verified: true, health: publicHealth, scope: 'Public DNS from this workstation; not an independent external-network observer' };
        } catch (error) { publicDNS = { verified: false, error: error.message }; }
        const result = { health, served, pages, publicDNS, events: watched.events, nativeProvenance: options.nativeProvenance, wasmProvenance: options.wasmProvenance,
            nativeArtifact: native.artifact, wasmArtifact: wasm.artifact, sourceManifestSha256: native.source_manifest_sha256,
            solverProfileSelection: report.solverProfiles };
        await save(path.join(options.output, `preflight-${options.phase}.json`), result);
        notice('preflight', { verified: true, pid: health.pid, candidateSha256: wasm.artifact.sha256, publicDNS: publicDNS.verified });
        return result;
    } catch (error) {
        await save(path.join(options.output, `preflight-failure-${options.phase}.json`), { error: error.message, pageURL: page.url(), events: watched.events });
        throw error;
    } finally { watched.stop(); await context.close(); }
}

async function browserReplay(dataset, frames, directory, { paced = false, loss = false } = {}) {
    await fs.mkdir(directory, { recursive: true });
    const context = await browser.newContext();
    const page = await context.newPage();
    const watched = watch(page);
    let replay;
    try {
        const paired = !paced && !loss;
        const benchmarkProfile = paired && options.benchmarkZeroTolerances;
        const parameters = new URLSearchParams({ dataset, frames: String(frames), speed: paced ? '1' : '-1',
            solverTime: paired ? '10' : '0.1', iterations: '10', features: '150',
            benchmarkZeroTolerances: benchmarkProfile ? '1' : '0', inputHash: paced ? '0' : '1' });
        await page.goto(`${options.origin}/test-tumvi.html?${parameters}`);
        await page.waitForFunction(() => window.__tumviReplay && !document.getElementById('btn-start').disabled, null, { timeout: 30000 });
        if (loss) await page.evaluate(() => {
            const app = window.__tumviReplay.app;
            const start = app.start.bind(app);
            app.start = async function () {
                // Pause scheduling until its actual prefetcher exists, then install only the deliberate blank-image fixture.
                const originalTimer = this.startPlaybackTimer;
                this.startPlaybackTimer = () => {};
                await start();
                this.startPlaybackTimer = originalTimer;
                const get = this.prefetcher.get.bind(this.prefetcher);
                this.prefetcher.get = index => index >= 100 && index < 112 ? Promise.resolve(new Uint8Array(512 * 512)) : get(index);
                this.startPlaybackTimer();
            };
        });
        await page.locator('#btn-start').click();
        await page.waitForFunction(() => {
            const value = window.__tumviReplay.export();
            return value.completed || value.failed || /^Error:|^Replay failed|FAILED/.test(document.getElementById('status').textContent);
        }, null, { timeout: 900000 });
        replay = await page.evaluate(async hashIMU => {
            const app = window.__tumviReplay.app;
            const value = window.__tumviReplay.export();
            if (hashIMU) for (const row of value.rows) {
                const bytes = app.imuData.slice(row.imuStartIndex * 7, row.imuEndIndexExclusive * 7);
                const digest = await crypto.subtle.digest('SHA-256', bytes);
                row.imuSha256 = Array.from(new Uint8Array(digest), byte => byte.toString(16).padStart(2, '0')).join('');
            }
            return value;
        }, !paced);
        await save(path.join(directory, 'results.json'), replay);
        await save(path.join(directory, 'browser-events.json'), watched.events);
        assert.equal(replay.schema, 'mobile-slam-replay-v2');
        assert.equal(replay.completed, true, 'Replay did not complete');
        assert.equal(replay.failed, false);
        assert.equal(replay.counts.input, frames);
        assert.equal(replay.benchmarkSolverProfile.requested, benchmarkProfile);
        assert.equal(replay.benchmarkSolverProfile.actual, benchmarkProfile, 'Replay did not confirm the actual engine profile');
        assert.equal(replay.counts.errors, 0);
        assert.equal(replay.counts.accepted, replay.counts.completed);
        assert.equal(replay.counts.input, replay.counts.accepted + replay.counts.dropped);
        assert.deepEqual(replay.rows.map(row => row.frame), Array.from({ length: frames }, (_, i) => i));
        if (!paced) assert.equal(replay.counts.dropped, 0);
        assert.ok(replay.counts.freshPoses > 0, 'Zero initialization/fresh pose coverage');
        for (const row of replay.rows) {
            assert.ok(Number.isFinite(row.inputTimestamp));
            if (row.poseFresh && row.poseValid) { finitePose(row.pose); assert.ok(Math.abs(row.poseTimestamp - row.inputTimestamp) < 1e-6); }
            else { assert.equal(row.pose, null); assert.equal(row.poseTimestamp, null); }
            if (row.completed) { assert.equal(row.executionSeed, 0); assert.equal(row.cvThreads, 1);
                assert.equal(row.benchmarkSolverProfile, benchmarkProfile, 'Actual engine profile changed during replay'); }
        }
        assertNetwork(watched.events);
        const trajectory = await exportTrajectory(replay, directory);
        if (!loss) await command('python3', ['scripts/evaluation/audit_metrics.py', trajectory, path.join(options[dataset], 'mav0/mocap0/data.csv'), '--estimate-frame', 'camera', '--config', options.config,
            '--expected-frames', String(frames), '--max-dt', '0.01', '--rpe-delta', '1', '--rpe-tolerance', '0.05', '--output', path.join(directory, 'metrics.json')], path.join(directory, 'metrics.log'));
        else await save(path.join(directory, 'metrics-context.json'), { status: 'not_computed',
            reason: 'Deliberate blank-image reset fixture contains distinct engine world epochs; validate loss/recovery/freshness rather than an aggregate trajectory accuracy claim.',
            engineEpochs: [...new Set(replay.rows.map(row => row.engineEpoch))], timestampSource: 'actual engine poseTimestamp', poseFrame: 'camera per engine world epoch' });
        const summarized = summary(replay);
        await save(path.join(directory, 'summary.json'), summarized);
        await page.screenshot({ path: path.join(directory, 'replay.png') });
        if (loss) {
            const before = replay.rows.slice(0, 100), blank = replay.rows.slice(100, 112), recovered = replay.rows.slice(112);
            assert.ok(before.some(row => row.poseFresh && row.poseValid), 'Texture warmup did not initialize actual estimator');
            assert.ok(blank.every(row => !row.poseFresh && !row.poseValid && !row.pose && row.poseTimestamp === null && row.mapPointCount === 0), 'Blank frame exposed stale pose/map');
            assert.ok(recovered.some(row => row.poseFresh && row.poseValid), 'Actual estimator did not recover after blank image fixture');
            const gap = await page.evaluate(async () => {
                const app = window.__tumviReplay.app;
                const vio = app.vio;
                const last = vio.getLatestResult();
                const timestamp = last.inputTimestamp + 2;
                // A real future bracket allows engine's frame-gap guard to run; no fabricated pose is injected.
                vio.sendIMU(new Float64Array([timestamp - .005,0,0,9.81007,0,0,0,timestamp + .005,0,0,9.81007,0,0,0]), 2);
                const accepted = vio.sendFrame(new Uint8Array(512*512), timestamp);
                const droppedBusy = !vio.sendFrame(new Uint8Array(512*512), timestamp);
                await vio.waitForFree(10000);
                const { pose, mapPoints, ...result } = vio.getLatestResult();
                return { accepted, droppedBusy, result, pose: pose ? Array.from(pose) : null, mapCount: vio.getMapPoints().count, transport: vio.getMetrics() };
            });
            await save(path.join(directory, 'gap-drop.json'), gap);
            assert.equal(gap.accepted, true); assert.equal(gap.droppedBusy, true); assert.equal(gap.pose, null);
            assert.equal(gap.result.poseTimestamp, null); assert.equal(gap.mapCount, 0); assert.equal(gap.result.reason, 'frame_gap');
        }
        notice(dataset + (paced ? '-paced' : loss ? '-loss' : '-browser'), { frames, freshPoses: replay.counts.freshPoses, dropped: replay.counts.dropped, directory });
        return replay;
    } catch (error) {
        const partial = await page.evaluate(() => window.__tumviReplay?.export()).catch(() => null);
        await save(path.join(directory, 'failure.json'), { error: error.message, pageURL: page.url(), partial, events: watched.events });
        await page.screenshot({ path: path.join(directory, 'failure.png') }).catch(() => {});
        throw error;
    } finally { watched.stop(); await context.close(); }
}

async function nativeReplay(dataset, directory) {
    const binary = path.join(options.nativeBuild, 'native_replay');
    await command(binary, [options[dataset], options.config, directory, String(expected[dataset]), 'bracket', '0', '1', '10', '10', '1'], directory + '.log',
        { MOBILE_SLAM_BENCHMARK_ZERO_TOLERANCES: options.benchmarkZeroTolerances ? '1' : '0' });
    const value = await json(path.join(directory, 'summary.json'));
    assert.equal(value.frames_processed, expected[dataset]); assert.ok(value.pose_count > 0); assert.equal(value.invalid_pose_count, 0);
    assert.equal(value.cv_threads, 1); assert.equal(value.rng_seed, 0); assert.equal(value.solver_iteration_limit, 10); assert.equal(value.solver_time_limit_s, 10);
    assert.equal(value.benchmark_solver_profile.requested, options.benchmarkZeroTolerances);
    assert.equal(value.benchmark_solver_profile.actual, options.benchmarkZeroTolerances);
    await command('python3', ['scripts/dev/hash-inputs.py', '--raw', path.join(directory, 'input-oracle.bin'), '--dataset', options[dataset], '--config', options.config, '--output', path.join(directory, 'input-manifest.json')], path.join(directory, 'hash-inputs.log'));
    await command('python3', ['scripts/evaluation/audit_metrics.py', path.join(directory, 'trajectory-camera.txt'), path.join(options[dataset], 'mav0/mocap0/data.csv'), '--estimate-frame', 'camera', '--config', options.config,
        '--expected-frames', String(expected[dataset]), '--max-dt', '0.01', '--rpe-delta', '1', '--rpe-tolerance', '0.05', '--output', path.join(directory, 'metrics.json')], path.join(directory, 'metrics.log'));
    notice(dataset + '-native', { frames: value.frames_processed, freshPoses: value.pose_count, directory });
    return value;
}

async function compareInputs(nativeDirectory, replay, outputFile) {
    const native = await json(path.join(nativeDirectory, 'input-manifest.json'));
    const profile = await json(path.join(nativeDirectory, 'summary.json'));
    const close = (actual, expected, label) => assert.ok(Number.isFinite(actual) && Math.abs(actual - expected) <= 1e-12, `Native/browser setting differs: ${label}`);
    for (const [field, actual, expected] of [
        ['cameraModel', replay.calibration.modelType, profile.camera_model_enum],
        ['width', replay.calibration.width, profile.image_width_height[0]], ['height', replay.calibration.height, profile.image_width_height[1]],
        ['iterations', replay.solver.num_iterations, profile.solver_iteration_limit], ['timeCap', replay.solver.solver_time, profile.solver_time_limit_s],
        ['featureLimit', replay.solver.max_features, profile.feature_limit], ['LKwindow', replay.tracking.lk_window, profile.lk_window_size],
        ['LKpyramid', replay.tracking.lk_pyramid, profile.lk_pyramid_levels], ['LKiterations', replay.tracking.lk_criteria_count, profile.lk_iterations],
        ['LKepsilon', replay.tracking.lk_criteria_eps, profile.lk_epsilon], ['minDist', replay.tracking.min_dist, profile.min_dist],
        ['Fthreshold', replay.tracking.f_threshold, profile.f_threshold], ['edgeFactor', replay.tracking.f_edge_factor, profile.f_edge_factor],
    ]) close(actual, expected, field);
    assert.equal(replay.tracking.pnp, profile.pnp_enabled === 1);
    assert.equal(replay.benchmarkSolverProfile.requested, profile.benchmark_solver_profile.requested);
    assert.equal(replay.benchmarkSolverProfile.actual, profile.benchmark_solver_profile.actual);
    assert.equal(replay.benchmarkSolverProfile.profileName, profile.benchmark_solver_profile.name);
    for (const [i, key] of ['fx', 'fy', 'cx', 'cy'].entries()) close(replay.calibration[key], profile.camera_intrinsics_fx_fy_cx_cy[i], key);
    for (const [i, key] of ['k2', 'k3', 'k4', 'k5'].entries()) close(replay.calibration[key], profile.camera_distortion_k2k3k4k5[i], key);
    for (const key of ['acc_n', 'acc_w', 'gyr_n', 'gyr_w', 'g_norm']) close(replay.calibration[key], profile.imu_noise[key], key);
    for (const [i, value] of replay.calibration.r_ic.entries()) close(value, profile.camera_to_imu_rotation_rowmajor[i], `R_ic[${i}]`);
    for (const [i, value] of replay.calibration.t_ic.entries()) close(value, profile.camera_to_imu_translation[i], `t_ic[${i}]`);
    assert.equal(native.frames.length, replay.rows.length);
    const failures = [];
    for (const [i, row] of replay.rows.entries()) {
        const reference = native.frames[i];
        for (const [name, actual, wanted] of [['grayHash', row.decodedGraySha256, reference.imageSha256], ['grayBytes', row.grayBytes, reference.imageBytes],
            ['IMUHash', row.imuSha256, reference.imuSha256], ['IMUCount', row.submittedIMUCount, reference.imuCount], ['processedIMUCount', row.imuCount, reference.imuCount]]) {
            if (actual !== wanted) failures.push({ frame: i, field: name, actual, expected: wanted });
        }
    }
    const result = { matched: failures.length === 0, frames: replay.rows.length, failures, nativeSourceSha256: native.source_sha256, inputPolicy: replay.inputPolicy,
        settings: { native: profile, browser: { calibration: replay.calibration, solver: replay.solver, tracking: replay.tracking }, numericComparisonTolerance: 1e-12 } };
    await save(outputFile, result);
    assert.deepEqual(failures, [], 'Decoded image / exact Float64 IMU packet differs across paths');
}

async function pilot() {
    const pairs = [];
    for (let run = 1; run <= 3; run++) {
        const directory = path.join(options.output, `pilot-${run}`);
        const native = path.join(directory, 'native');
        await nativeReplay('room1', native);
        const worker = path.join(directory, 'browser');
        const replay = await browserReplay('room1', expected.room1, worker);
        await compareInputs(native, replay, path.join(directory, 'input-comparison.json'));
        pairs.push([path.join(native, 'results.json'), path.join(worker, 'results.json')]);
    }
    const args = ['scripts/dev/compare-replay.py', '--solver-iterations', '10', '--solver-time', '10', '--output', options.contract];
    for (const pair of pairs) args.push('--freeze-pilot', ...pair);
    await command('python3', args, path.join(options.output, 'freeze-contract.log'));
    const contract = await json(options.contract);
    assert.equal(contract.status, 'frozen_before_room4');
    await save(path.join(options.output, 'frozen-contract-sha256.json'), { file: options.contract, sha256: sha(await fs.readFile(options.contract)), frozenAt: new Date().toISOString() });
    return { pairs, contract: options.contract };
}

async function room4() {
    const frozen = await json(path.join(options.output, 'frozen-contract-sha256.json'));
    assert.equal(sha(await fs.readFile(options.contract)), frozen.sha256, 'Frozen contract changed before room4');
    const directory = path.join(options.output, 'room4');
    const native = path.join(directory, 'native');
    await nativeReplay('room4', native);
    const worker = path.join(directory, 'browser');
    const replay = await browserReplay('room4', expected.room4, worker);
    await compareInputs(native, replay, path.join(directory, 'input-comparison.json'));
    await command('python3', ['scripts/dev/compare-replay.py', '--native', path.join(native, 'results.json'), '--browser', path.join(worker, 'results.json'), '--expected-frames', '2228', '--contract', options.contract, '--output', path.join(directory, 'parity.json')], path.join(directory, 'compare.log'));
    return json(path.join(directory, 'parity.json'));
}

async function mock() {
    const wrapper = path.join(options.output, 'chrome-strict-resolver.sh');
    const realChrome = process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome';
    assert.ok(!realChrome.includes("'"));
    await fs.writeFile(wrapper, `#!/usr/bin/env bash\nexec '${realChrome}' '--host-resolver-rules=MAP dev.serdic.com 127.0.0.1' "$@"\n`, { mode: 0o755 });
    const directory = path.join(options.output, 'mock');
    await command('node', ['tests/browser/mobile-source-contracts-browser.mjs'], path.join(options.output, 'mock.log'), { BROWSER_EXECUTABLE: wrapper,
        MOBILE_SOURCE_TEST_ORIGIN: options.origin, MOBILE_SOURCE_TEST_OUTPUT: directory });
    const value = await json(path.join(directory, 'browser-contracts.json'));
    assert.equal(value.error, undefined); assert.equal(value.scenarios.length, 5);
    assert.deepEqual(value.pixelRotationGolden, { none: [10,20,30,40,50,60], cw: [50,30,10,60,40,20], ccw: [20,40,60,10,30,50], half: [60,50,40,30,20,10] });
    value.androidGenericDenial = await androidGenericDenial();
    return value;
}

async function androidGenericDenial() {
    const browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
        args: ['--enable-unsafe-swiftshader', '--host-resolver-rules=MAP dev.serdic.com 127.0.0.1', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'] });
    const context = await browser.newContext({ viewport: { width: 412, height: 915 }, userAgent: 'Android Chrome Generic Sensor permission fixture' });
    await context.addInitScript(() => {
        window.__validationPermissionQueries = [];
        Object.defineProperty(window, 'DeviceMotionEvent', { configurable: true, value: class extends Event {} });
        Object.defineProperty(window, 'Accelerometer', { configurable: true, value: class extends EventTarget { start() { throw new Error('Denied sensor must not start'); } stop() {} } });
        Object.defineProperty(window, 'Gyroscope', { configurable: true, value: class extends EventTarget { start() { throw new Error('Denied sensor must not start'); } stop() {} } });
        Object.defineProperty(navigator, 'permissions', { configurable: true, value: { query: async ({ name }) => {
            window.__validationPermissionQueries.push({ name, activation: navigator.userActivation.isActive, atMs: performance.now() });
            return { state: 'denied' };
        } } });
        class Orientation extends EventTarget {
            constructor() { super(); this.type = 'portrait-primary'; this.angle = 0; }
            async lock() {} unlock() {}
            change(type) { this.type = type; this.angle = 90; this.dispatchEvent(new Event('change')); }
        }
        Object.defineProperty(screen, 'orientation', { configurable: true, value: new Orientation() });
    });
    const page = await context.newPage();
    const watched = watch(page);
    const directory = path.join(options.output, 'mock-android-generic-denial');
    await fs.mkdir(directory, { recursive: true });
    try {
        await page.goto(options.origin + '/');
        await page.waitForFunction(() => window.__mobileSLAM && !document.getElementById('btn-start').disabled, null, { timeout: 30000 });
        await page.locator('#btn-start').click();
        await page.waitForFunction(() => window.__mobileSLAM.diagnostics().transport.framesCompleted >= 3, null, { timeout: 10000 });
        const before = await page.evaluate(() => ({ diagnostics: window.__mobileSLAM.diagnostics(), permissionQueries: window.__validationPermissionQueries,
            status: document.getElementById('status').textContent }));
        assert.equal(before.diagnostics.imu.permission.api, 'Permissions.query');
        assert.equal(before.diagnostics.imu.permission.state, 'denied');
        assert.equal(before.diagnostics.imu.emitted, 0);
        assert.deepEqual(before.permissionQueries.map(row => row.name), ['accelerometer', 'gyroscope']);
        assert.ok(before.permissionQueries.every(row => row.activation === true));
        assert.match(before.status, /permission denied/);
        const rotationInvalidation = await page.evaluate(() => {
            const app = window.__mobileSLAM.app;
            const epoch = app.vio.getMetrics().clientEpoch;
            screen.orientation.change('landscape-primary');
            return { beforeEpoch: epoch, afterEpoch: app.vio.getMetrics().clientEpoch, cachedResult: app.vio.getLatestResult(), mapCount: app.vio.getMapPoints().count };
        });
        assert.ok(rotationInvalidation.afterEpoch > rotationInvalidation.beforeEpoch);
        assert.equal(rotationInvalidation.cachedResult, null); assert.equal(rotationInvalidation.mapCount, 0);
        await page.waitForFunction(() => window.__mobileSLAM.app.running && !window.__mobileSLAM.app._reconfiguring && window.__mobileSLAM.app.vio.configured, null, { timeout: 10000 });
        const after = await page.evaluate(() => ({ diagnostics: window.__mobileSLAM.diagnostics(), configuredCamera: window.__mobileSLAM.app._lastConfigParams,
            screenOrientation: screen.orientation.type }));
        assert.equal(after.screenOrientation, 'landscape-primary');
        assert.equal(after.diagnostics.imu.emitted, 0);
        assertNetwork(watched.events);
        const result = { before, rotationInvalidation, after, events: watched.events,
            scope: 'Actual HTTPS app/Worker with mocked Android permission denial and screen orientation; no physical phone evidence' };
        await save(path.join(directory, 'result.json'), result);
        await page.screenshot({ path: path.join(directory, 'app.png') });
        return result;
    } finally { watched.stop(); await context.close(); await browser.close(); }
}

async function main() {
    if (options.phase === 'self-test') {
        const pose = [0,-1,0,1,1,0,0,2,0,0,1,3,0,0,0,1];
        const q = quaternion(pose);
        assert.ok(Math.abs(q[2] - Math.SQRT1_2) < 1e-12 && Math.abs(q[3] - Math.SQRT1_2) < 1e-12);
        assert.throws(() => finitePose(Array(16).fill(0)));
        assert.deepEqual(percentiles([1,2,3,4,NaN]), { count: 4, p50: 2, p95: 4, p99: 4, max: 4 });
        notice('self-test', { pass: 3, scope: 'Runner serialization / SO(3) rejection / percentile helpers; no browser or estimator validation' });
        return;
    }
    assert.equal(process.env.MOBILE_SLAM_PERFORMANCE_READY, '1', 'Leader performance_ready required; do not profile during builds/replays');
    if (['pilot', 'room4', 'all'].includes(options.phase)) assert.equal(options.benchmarkZeroTolerances, true,
        'Full numerical parity requires explicit --benchmark-zero-tolerances 1 (or matching environment flag)');
    await fs.mkdir(options.output, { recursive: true });
    browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
        args: ['--enable-unsafe-swiftshader', '--host-resolver-rules=MAP dev.serdic.com 127.0.0.1', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'] });
    report.browser = browser.version();
    report.results.preflight = await preflight();
    if (['pilot', 'all'].includes(options.phase)) report.results.pilot = await pilot();
    if (['room4', 'all'].includes(options.phase)) report.results.room4 = await room4();
    if (['paced', 'all'].includes(options.phase)) report.results.paced = summary(await browserReplay('room1', options.pacedFrames, path.join(options.output, 'paced-20hz'), { paced: true }));
    if (['loss', 'all'].includes(options.phase)) report.results.loss = summary(await browserReplay('room1', 350, path.join(options.output, 'textured-blank-recover'), { loss: true }));
    if (['mock', 'all'].includes(options.phase)) { await browser.close(); browser = null; report.results.mock = await mock(); }
    report.status = 'verified';
}

try { await main(); }
catch (error) { report.status = 'failed'; report.error = { message: error.message, stack: error.stack }; process.exitCode = 1; notice('failed', { error: error.message }); }
finally {
    if (browser) await browser.close();
    report.finishedAt = new Date().toISOString();
    if (options.phase !== 'self-test') await save(path.join(options.output, `validation-${options.phase}.json`), report);
}
