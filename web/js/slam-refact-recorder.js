// Opt-in input recorder of the slam_refact measurement page (only with ?record=1; default off).
//
// Purpose: replay a real-device run on the PC with exactly the inputs the engine got. For every frame
// POSTED to the engine worker it stores the 480x640 gray engine-input frame and one JSON row
// {id, timeNs, videoTimeNs, readMs, sensors: <the exact sensor rows posted with that frame>}.
// Frames dropped as busy are not stored (their sensor rows travel with the next posted frame).
//
// Gray = round(0.299 R + 0.587 G + 0.114 B) of the same 480x640 capture pixels the engine receives.
// The engine's GPU frontend converts colour itself (the original shaders), so this gray is an
// APPROXIMATION of what the GPU path saw: a CPU-pyramid replay of it is not bit-identical to the
// device run (sensors, timing, frame order and config are exact; pixel intensities are not).
//
// Storage: OPFS only (origin-private file system on this device). Cost split (the engine must not
// pay for the recording):
//   main thread   GPU capture: one createImageBitmap(canvas) snapshot of the engine-input canvas,
//                 taken synchronously BEFORE the canvas is transferred to the engine worker (the
//                 copy is GPU-side; no readback here). CPU capture: one memcpy of the RGBA bytes the
//                 engine gets. Then a transfer to the writer Worker.
//   writer Worker (slam-refact-rec-writer.js) GPU readback (drawImage + getImageData), gray
//                 conversion and the OPFS write at the frame's index (createSyncAccessHandle).
//   engine worker untouched.
// The writer backlog is bounded (REC_MAX_BACKLOG frames): when the writer falls that far behind the
// recording stops (recorded frames stay valid, the summary says why) instead of slowing the engine.
// ?rec-verify=N (diagnostic) also computes the gray the old synchronous way on the main thread for
// every N-th frame and the writer compares it byte for byte with its own (summary.verify). Only the
// GPU capture path is tested by it (it checks the snapshot); on the CPU path both sides come from
// the same bytes and it can only confirm the gray formula.
// Per-frame metadata stays in memory and is written as JSON to OPFS on Stop. No network access:
// this module never fetches or sends anything, and downloads happen only from the user's click.
import { percentiles } from './slam-refact-core.js';
import { grayFromRgba } from './slam-refact-gray.js';

export { grayFromRgba };
export const REC_PREFIX = 'slam-refact-rec-';
export const REC_WIDTH = 480;
export const REC_HEIGHT = 640;
export const REC_MAX_FRAMES = 6000;   // ~1.8 GB on the device (307200 B per frame), ~3.3 min at 30 fps

export const REC_MAX_BACKLOG = 45;   // frames queued in the writer (~1.5 s; ~55 MB of snapshots)

export function recordingStamp(date = new Date()) { return date.toISOString().replace(/[:.]/g, '-'); }

export async function sha256Hex(bytes) {
  const d = new Uint8Array(await crypto.subtle.digest('SHA-256', bytes));
  return Array.from(d, (b) => b.toString(16).padStart(2, '0')).join('');
}

export async function opfsRoot() { return navigator.storage.getDirectory(); }

export class InputRecorder {
  constructor({ width = REC_WIDTH, height = REC_HEIGHT, maxFrames = REC_MAX_FRAMES, maxBacklog = REC_MAX_BACKLOG,
    verifyEvery = 0 } = {}) {
    this.width = width; this.height = height; this.frameBytes = width * height; this.maxFrames = maxFrames;
    this.maxBacklog = maxBacklog; this.verifyEvery = Math.max(0, verifyEvery | 0);
    this.active = false; this.finalizing = false;
    this.stamp = null; this.names = null; this.rows = []; this.resets = [];
    this.writer = null; this.posted = 0; this.written = 0; this.maxQueued = 0; this.error = null;
    this.limitReached = false; this.stopReason = null;
    this.costsMs = [];       // main-thread cost per recorded frame: snapshot/copy + transfer
    this.pending = new Set(); this.summary = null; this.waiters = [];
    this.refCanvas = null; this.refCtx = null;
    this.onStopped = null;   // (stopReason) => void, called once when the recording stops early
  }

  get count() { return this.rows.length; }

  // Opens the writer and the (truncated) frames file. Rejects when OPFS is unavailable: the page then
  // refuses to start a recording run instead of silently running without one.
  async begin() {
    if (!navigator.storage || typeof navigator.storage.getDirectory !== 'function') throw new Error('OPFS (navigator.storage.getDirectory) is unavailable');
    this.stamp = recordingStamp();
    this.names = { frames: `${REC_PREFIX}${this.stamp}.frames.bin`, meta: `${REC_PREFIX}${this.stamp}.meta.json` };
    this.rows = []; this.resets = []; this.posted = 0; this.written = 0; this.maxQueued = 0; this.error = null;
    this.limitReached = false; this.stopReason = null; this.costsMs = []; this.pending = new Set(); this.summary = null;
    // Free space: refuse a recording that cannot hold even 30 s; report the rest (page note).
    let space = null;
    try {
      const est = await navigator.storage.estimate?.();
      if (est && Number.isFinite(est.quota) && Number.isFinite(est.usage)) {
        const freeBytes = Math.max(0, est.quota - est.usage);
        space = { freeBytes, frames: Math.floor(freeBytes / this.frameBytes), persisted: false };
        if (space.frames < 900) throw new Error(`only ${(freeBytes / 1e6).toFixed(0)} MB free for OPFS recordings (needs ~${(900 * this.frameBytes / 1e6).toFixed(0)} MB for 30 s); delete old recordings first`);
        this.maxFrames = Math.min(this.maxFrames, space.frames - 60);
      }
      if (space) space.persisted = Boolean(await navigator.storage.persist?.());
    } catch (e) { if (/MB free/.test(String(e && e.message))) throw e; }
    this.writer = new Worker('js/slam-refact-rec-writer.js', { type: 'module' });
    this.writer.onmessage = (ev) => this.onWriter(ev.data);
    this.writer.onerror = (ev) => { this.fail(`writer worker error: ${ev.message}`); };
    const opened = this.wait((m) => m.t === 'opened');
    this.writer.postMessage({ t: 'open', name: this.names.frames, width: this.width, height: this.height });
    await opened;
    this.active = true;
    return space;
  }

  wait(pred) {
    return new Promise((resolve, reject) => {
      if (this.error) reject(new Error(this.error));
      else this.waiters.push({ pred, resolve, reject });
    });
  }

  fail(message) {
    if (!this.error) this.error = message;
    if (this.active) this.stop('error', this.error);
    this.active = false;
    for (const w of this.waiters.splice(0)) w.reject(new Error(this.error));
  }

  onWriter(m) {
    if (m.t === 'error') { this.fail(m.message); return; }
    if (m.t === 'wrote') this.written = m.count;
    const rest = [];
    for (const w of this.waiters) { if (w.pred(m)) w.resolve(m); else rest.push(w); }
    this.waiters = rest;
  }

  // Frame source of the next add(), taken from the 480x640 engine-input pixels BEFORE they go to the
  // engine worker: canvas = the capture canvas (call before transferToImageBitmap clears it), or
  // rgba = the bytes about to be transferred. Returns null when the recording does not take it.
  take({ canvas = null, rgba = null }) {
    if (!this.active) return null;
    if (this.rows.length >= this.maxFrames) { this.stop('limit', `recording limit reached (${this.maxFrames} frames)`); return null; }
    if (this.posted - this.written >= this.maxBacklog) {
      this.stop('backlog', `recording writer fell ${this.maxBacklog} frames behind`);
      return null;
    }
    const verify = this.verifyEvery > 0 && this.rows.length % this.verifyEvery === 0;
    const ref = verify ? this.referenceGray(canvas, rgba) : null;
    if (rgba) return { rgba: rgba.slice(), ref };
    return { bitmap: createImageBitmap(canvas), ref };
  }

  // Old synchronous path (diagnostic ?rec-verify only): main-thread readback + gray.
  referenceGray(canvas, rgba) {
    if (rgba) return grayFromRgba(rgba);
    if (!this.refCtx) {
      this.refCanvas = new OffscreenCanvas(this.width, this.height);
      this.refCtx = this.refCanvas.getContext('2d', { willReadFrequently: true, alpha: false });
    }
    this.refCtx.drawImage(canvas, 0, 0);
    return grayFromRgba(this.refCtx.getImageData(0, 0, this.width, this.height).data);
  }

  stop(reason, message) {
    if (!this.active) return;
    this.active = false; this.stopReason = { reason, message, atFrame: this.rows.length };
    if (reason === 'limit') this.limitReached = true;
    try { this.onStopped?.(this.stopReason); } catch { /* display only */ }
  }

  // Releases a take() result that will not be added (the engine post failed).
  discard(src) {
    if (src && src.bitmap) src.bitmap.then((b) => b.close(), () => {});
  }

  // src: the result of take(); row: the frame's metadata row (the values posted to the engine).
  add(src, row, costStartMs) {
    if (!src) return false;
    const index = this.rows.length;
    this.rows.push({ ...row, index });
    this.posted++;
    const post = (payload, transfer) => {
      if (!this.writer) { payload.bitmap?.close(); return; }
      this.writer.postMessage({ t: 'f', i: index, ...payload }, transfer);
    };
    const ref = src.ref ? { ref: src.ref.buffer } : {};
    const refT = src.ref ? [src.ref.buffer] : [];
    if (src.rgba) {
      post({ rgba: src.rgba.buffer, ...ref }, [src.rgba.buffer, ...refT]);
    } else {
      const p = src.bitmap.then((bitmap) => post({ bitmap, ...ref }, [bitmap, ...refT]),
        (e) => this.fail(`frame snapshot failed: ${e && e.message || e}`));
      this.pending.add(p);
      p.finally(() => this.pending.delete(p));
    }
    this.maxQueued = Math.max(this.maxQueued, this.posted - this.written);
    this.costsMs.push(performance.now() - costStartMs);
    return true;
  }

  noteReset(atMs) { if (this.active) this.resets.push({ beforeFrameIndex: this.rows.length, atMs }); }

  // Stops accepting frames, drains the snapshots and the writer, writes the meta JSON
  // (buildMeta(stats) -> object) to OPFS and returns the recording summary (file names, frame count,
  // bytes, sha256 of the JSON, writer costs).
  async finish(buildMeta) {
    if (this.summary || !this.writer) return this.summary;
    if (this.active) this.active = false;
    this.finalizing = true;
    try {
      await Promise.allSettled([...this.pending]);
      if (this.error) throw new Error(this.error);
      const closed = this.wait((m) => m.t === 'closed');
      this.writer.postMessage({ t: 'close' });
      const c = await closed;
      if (this.error) throw new Error(this.error);
      const writerCostMs = { convert: percentiles(c.costMs.convert), write: percentiles(c.costMs.write) };
      const verify = this.verifyEvery > 0 ? { every: this.verifyEvery, ...c.verify } : null;
      const stats = { frames: this.rows.length, framesWritten: c.count, framesBytes: c.bytes,
        maxQueuedFrames: this.maxQueued, limitReached: this.limitReached, maxFrames: this.maxFrames,
        stopReason: this.stopReason, writerCostMs, verify, resets: this.resets };
      if (c.count !== this.rows.length || c.bytes !== this.rows.length * this.frameBytes) {
        throw new Error(`OPFS frame count/bytes mismatch: rows ${this.rows.length}, written ${c.count}, bytes ${c.bytes}`);
      }
      const json = JSON.stringify(buildMeta(stats));
      const jsonBytes = new TextEncoder().encode(json);
      const metaSha256 = await sha256Hex(jsonBytes);
      const written = this.wait((m) => m.t === 'metaWritten');
      this.writer.postMessage({ t: 'meta', name: this.names.meta, json });
      await written;
      this.summary = { ok: true, stamp: this.stamp, files: { frames: this.names.frames, meta: this.names.meta },
        frames: this.rows.length, width: this.width, height: this.height, framesBytes: c.bytes,
        metaBytes: jsonBytes.byteLength, metaSha256, limitReached: this.limitReached,
        stopReason: this.stopReason, maxQueuedFrames: this.maxQueued, writerCostMs, verify, error: null };
    } catch (e) {
      this.summary = { ok: false, stamp: this.stamp, files: this.names, frames: this.rows.length,
        stopReason: this.stopReason, error: String(e && e.message || e) };
    } finally {
      this.finalizing = false;
      this.writer.terminate(); this.writer = null;
    }
    return this.summary;
  }
}

// Read access to the OPFS recording files (CDP pulling alternative to the download button).
export function recordingApi() {
  const entries = async () => {
    const root = await opfsRoot();
    const out = [];
    for await (const [name, h] of root.entries()) if (h.kind === 'file' && name.startsWith(REC_PREFIX)) out.push(name);
    return out.sort();
  };
  const fileOf = async (name) => {
    if (!String(name).startsWith(REC_PREFIX) || /[\/\\]/.test(name)) throw new Error(`not a recording file: ${name}`);
    return (await (await opfsRoot()).getFileHandle(name)).getFile();
  };
  return {
    // async: names of the recording files in OPFS
    files: entries,
    // async: size in bytes
    size: async (name) => (await fileOf(name)).size,
    // async: base64 of bytes [offset, offset + length) (length is capped at 8 MiB per call)
    readChunk: async (name, offset, length) => {
      const f = await fileOf(name);
      const buf = new Uint8Array(await f.slice(offset, offset + Math.min(length, 8 << 20)).arrayBuffer());
      let s = '';
      for (let i = 0; i < buf.length; i += 0x8000) s += String.fromCharCode.apply(null, buf.subarray(i, i + 0x8000));
      return btoa(s);
    },
    // async: File objects of one recording (used by the page's download button)
    file: fileOf,
    // async: deletes one recording file from OPFS
    remove: async (name) => { await fileOf(name); await (await opfsRoot()).removeEntry(name); },
  };
}
