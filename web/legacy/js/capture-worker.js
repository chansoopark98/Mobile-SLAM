/** Optional VideoFrame capture experiment. The default App path stays synchronous. */
export class CaptureWorker {
    constructor({ onFrame, onError = () => {}, timeoutMs = 2000,
        createWorker = () => new Worker(new URL('./capture-worker.js', import.meta.url), { type: 'module' }),
        createFrame = (video, init) => new VideoFrame(video, init) } = {}) {
        this.onFrame = onFrame;
        this.onError = onError;
        this.createWorker = createWorker;
        this.createFrame = createFrame;
        this.timeoutMs = timeoutMs;
        this.worker = null;
        this.inFlight = null;
        this.pending = null;
        this.state = 'idle';
        this.seq = 0;
        this.timer = null;
        this.stopPromise = null;
        this.stats = { scheduled: 0, posted: 0, completed: 0, accepted: 0, dropped: 0,
            coalesced: 0, closedLocal: 0, closedRemote: 0, releasedByTermination: 0,
            staleReplies: 0, constructorFailures: 0, errors: 0 };
    }

    start() {
        if (this.state === 'stopping') { this.restartRequested = true; return; }
        if (this.state !== 'idle') return;
        try {
            this.state = 'starting';
            this.remoteBase = this.stats.closedRemote;
            const worker = this.worker = this.createWorker();
            worker.onmessage = event => { if (this.worker === worker) this._receive(event.data); };
            worker.onerror = event => { if (this.worker === worker) { event.preventDefault?.(); this._fail(new Error(event.message || 'Capture Worker error')); } };
            worker.onmessageerror = () => { if (this.worker === worker) this._fail(new Error('Capture Worker message could not be decoded')); };
            this.timer = setTimeout(() => this._fail(new Error('Capture Worker startup timeout')), this.timeoutMs);
            this.worker.postMessage({ type: 'init' });
        } catch (error) { this._fail(error); }
    }

    /** True consumes this callback; after a failed transfer it must never be reread. */
    capture(video, packet) {
        if (!['ready', 'probing', 'active'].includes(this.state)) return false;
        // Close before allocating: inFlight + old pending + replacement would peak at three.
        if (this.pending) { this._closePending(); this.stats.coalesced++; }
        const started = performance.now();
        let frame;
        try {
            frame = this.createFrame(video, { timestamp: Math.round(packet.timing.timestampS * 1e6) });
        } catch (error) {
            this.stats.constructorFailures++;
            this._fail(error);
            return false; // No frame was transferred; this callback can still use synchronous capture.
        }
        const ended = performance.now();
        if (this.configuration !== packet.configuration) {
            this.configuration = packet.configuration;
            this.configurationId = (this.configurationId ?? 0) + 1;
        }
        const identity = { seq: ++this.seq, generation: packet.captureGeneration,
            configurationId: this.configurationId,
            clientEpoch: packet.geometry.clientEpoch, timestampUs: Math.round(packet.timing.timestampS * 1e6),
            presentedFrames: packet.timing.presentedFrames, geometry: { ...packet.geometry } };
        const entry = { frame, packet, identity, cancelled: false };
        Object.assign(packet.timing, { capturePath: 'video_frame_worker_experiment', physicalTargetVerified: false,
            mainSnapshotStartMs: started, mainSnapshotEndMs: ended,
            pixelReadStartMs: null, pixelReadEndMs: null, snapshotCopyMs: 0 });
        this.stats.scheduled++;
        if (this.state === 'ready') this.state = 'probing';
        if (this.inFlight) {
            this.pending = entry;
        } else this._dispatch(entry);
        return true;
    }

    _dispatch(entry) {
        this.inFlight = entry;
        try {
            this.worker.postMessage({ type: 'frame', identity: entry.identity, frame: entry.frame }, [entry.frame]);
            entry.frame = null; // Ownership is now solely in the Worker realm.
            this.stats.posted++;
            this.timer = setTimeout(() => this._fail(new Error('Capture Worker frame timeout')), this.timeoutMs);
        } catch (error) {
            entry.frame?.close();
            this.stats.closedLocal++;
            entry.frame = null;
            this.inFlight = null;
            this.stats.dropped++;
            this._fail(error);
        }
    }

    _receive(data) {
        if (data?.type === 'ready' && this.state === 'starting') {
            clearTimeout(this.timer); this.timer = null;
            this.state = 'ready'; return;
        }
        if (!data?.identity) {
            if (data?.type === 'error') this._fail(new Error(data.message || 'Capture Worker initialization failed'));
            return;
        }
        const entry = this.inFlight;
        if (!entry || data.identity.seq !== entry.identity.seq) { this.stats.staleReplies++; return; }
        clearTimeout(this.timer); this.timer = null;
        const closed = Number.isSafeInteger(data.closedFrames) && data.closedFrames > 0;
        if (closed) this.stats.closedRemote = Math.max(this.stats.closedRemote, this.remoteBase + data.closedFrames);
        this.inFlight = null;
        this.stats.completed++;
        const exactIdentity = JSON.stringify(data.identity) === JSON.stringify(entry.identity);
        const validGray = data.gray instanceof ArrayBuffer &&
            data.gray.byteLength === entry.identity.geometry.cropWidth * entry.identity.geometry.cropHeight;
        const validTiming = ['workerDrawReadMs', 'workerGrayMs'].every(key => Number.isFinite(data.timing?.[key]) && data.timing[key] >= 0);
        if (data.type !== 'frame' || !exactIdentity || !validGray || !validTiming || !closed) {
            if (!closed) this.stats.releasedByTermination++;
            this.stats.dropped++;
            this._fail(new Error(data.message || 'Invalid capture Worker response')); return;
        }
        if (entry.cancelled || this.state === 'stopping') this.stats.dropped++;
        else {
            this.state = 'active'; // The actual VideoFrame -> transfer -> draw/read probe succeeded.
            entry.packet.gray = new Uint8Array(data.gray);
            Object.assign(entry.packet.timing, data.timing, { captureWorkerReceiveMs: performance.now() });
            try {
                if (this.onFrame(entry.packet) === false) this.stats.dropped++;
                else this.stats.accepted++;
            } catch (error) { this.stats.dropped++; this._fail(error); return; }
        }
        if (this.state === 'stopping') this._finishStop();
        else if (!this.inFlight && this.pending) { const next = this.pending; this.pending = null; this._dispatch(next); }
    }

    _closePending() {
        if (!this.pending) return;
        this.pending.frame.close();
        this.pending = null;
        this.stats.closedLocal++;
        this.stats.dropped++;
    }

    cancel() {
        this._closePending();
        if (this.inFlight) this.inFlight.cancelled = true;
    }

    /** Drain before termination; the existing deadline also bounds a broken Worker. */
    stop() {
        if (this.stopPromise) return this.stopPromise;
        this.cancel();
        if (!this.inFlight) { this._finishStop(); return Promise.resolve(); }
        this.state = 'stopping';
        this.stopPromise = new Promise(resolve => { this.resolveStop = resolve; });
        return this.stopPromise;
    }

    _finishStop() {
        clearTimeout(this.timer); this.timer = null;
        this.worker?.terminate(); this.worker = null;
        this.state = 'idle';
        this.resolveStop?.(); this.resolveStop = null; this.stopPromise = null;
        if (this.restartRequested) { this.restartRequested = false; this.start(); }
    }

    _fail(error) {
        clearTimeout(this.timer); this.timer = null;
        this._closePending();
        if (this.inFlight) {
            this.stats.dropped++;
            this.stats.releasedByTermination++; // Realm release is not an explicit close ACK.
            this.inFlight = null;
        }
        this.worker?.terminate(); this.worker = null;
        this.state = 'failed';
        this.lastError = String(error.message || error);
        this.stats.errors++;
        this.resolveStop?.(); this.resolveStop = null; this.stopPromise = null;
        this.onError(error);
        if (this.restartRequested) { this.restartRequested = false; this.state = 'idle'; this.start(); }
    }

    getMetrics() {
        return { ...this.stats, state: this.state, inFlight: this.inFlight ? 1 : 0, pending: this.pending ? 1 : 0,
            lastError: this.lastError ?? null, physicalTargetVerified: false };
    }
}

/** Shared entry point for the real Worker and its fault/lifetime probes. */
export function createCaptureWorkerHandler(send, Canvas = OffscreenCanvas) {
    let canvas, context, closedFrames = 0;
    return message => {
        if (message?.type === 'init') {
            try {
                canvas = new Canvas(2, 2);
                context = canvas.getContext('2d', { willReadFrequently: true });
                if (!context) throw new Error('Capture Worker OffscreenCanvas2D unavailable');
                context.getImageData(0, 0, 1, 1);
                send({ type: 'ready' });
            } catch (error) { send({ type: 'error', message: String(error.message || error) }); }
            return;
        }
        if (message?.type !== 'frame') return;
        const { frame, identity } = message;
        let reply;
        try {
            const g = identity.geometry;
            const swap = g.rotation === 'cw' || g.rotation === 'ccw';
            const width = swap ? g.sourceHeight : g.sourceWidth;
            const height = swap ? g.sourceWidth : g.sourceHeight;
            if (!context || !['none', 'cw', 'ccw', 'half'].includes(g.rotation) ||
                ![g.sourceWidth, g.sourceHeight, g.cropX, g.cropY, g.cropWidth, g.cropHeight].every(Number.isSafeInteger) ||
                Math.min(g.sourceWidth, g.sourceHeight, g.cropWidth, g.cropHeight) <= 0 || g.cropX < 0 || g.cropY < 0 ||
                g.cropX + g.cropWidth > width || g.cropY + g.cropHeight > height ||
                frame.displayWidth !== g.sourceWidth || frame.displayHeight !== g.sourceHeight || frame.timestamp !== identity.timestampUs) {
                throw new Error('Capture Worker frame geometry/timestamp mismatch');
            }
            if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
            const started = performance.now();
            context.save();
            try {
                if (g.rotation === 'cw') { context.translate(width, 0); context.rotate(Math.PI / 2); }
                else if (g.rotation === 'ccw') { context.translate(0, height); context.rotate(-Math.PI / 2); }
                else if (g.rotation === 'half') { context.translate(width, height); context.rotate(Math.PI); }
                context.drawImage(frame, 0, 0, g.sourceWidth, g.sourceHeight);
            } finally { context.restore(); }
            const rgba = context.getImageData(g.cropX, g.cropY, g.cropWidth, g.cropHeight).data;
            const readEnd = performance.now();
            const gray = new Uint8Array(g.cropWidth * g.cropHeight);
            for (let i = 0, j = 0; i < rgba.length; i += 4, j++) gray[j] = (rgba[i] * 77 + rgba[i + 1] * 150 + rgba[i + 2] * 29) >> 8;
            reply = { type: 'frame', identity, gray: gray.buffer,
                timing: { workerDrawReadMs: readEnd - started, workerGrayMs: performance.now() - readEnd } };
        } catch (error) { reply = { type: 'error', identity, message: String(error.message || error) }; }
        finally { frame?.close(); closedFrames++; }
        send({ ...reply, closedFrames }, reply.gray ? [reply.gray] : []);
    };
}

if (typeof WorkerGlobalScope !== 'undefined' && globalThis instanceof WorkerGlobalScope) {
    const handle = createCaptureWorkerHandler((message, transfer = []) => postMessage(message, transfer));
    globalThis.onmessage = event => handle(event.data);
}
