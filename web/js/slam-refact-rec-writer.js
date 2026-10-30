// Dedicated writer Worker of the opt-in input recorder (?record=1, web/js/slam-refact-recorder.js).
// Takes each recorded frame as an ImageBitmap snapshot (GPU capture) or an RGBA copy (CPU capture),
// does the GPU readback and the gray conversion HERE (off the main thread and off the engine
// worker), and writes the gray frame at its index into one OPFS file (origin-private, on this
// device only) through a sync access handle. No network access of any kind.
// Messages in:  {t:'open', name, width, height} | {t:'f', i, bitmap|rgba, ref?} (ref: ?rec-verify gray)
//               | {t:'meta', name, json} | {t:'close'}
// Messages out: {t:'opened'} | {t:'wrote', count} | {t:'metaWritten', name, bytes}
//               | {t:'closed', count, bytes, costMs: {convert: [...], write: [...]}, verify}
//               | {t:'error', message}
import { grayFromRgba } from './slam-refact-gray.js';

let handle = null;
let width = 0, height = 0, frameBytes = 0;
let count = 0;
let canvas = null, ctx = null, gray = null;
let convertMs = [], writeMs = [];
let verify = { frames: 0, mismatchFrames: 0, mismatchBytes: 0 };   // ?rec-verify reference comparisons
let failed = null;
let chain = Promise.resolve();

function writeAll(h, bytes, at) {
  const n = h.write(bytes, { at });
  if (n !== bytes.byteLength) throw new Error(`short OPFS write ${n}/${bytes.byteLength} at ${at}`);
}

function rgbaOf(m) {
  if (m.rgba) return new Uint8Array(m.rgba);
  if (!ctx) {
    canvas = new OffscreenCanvas(width, height);
    ctx = canvas.getContext('2d', { willReadFrequently: true, alpha: false });
  }
  try { ctx.drawImage(m.bitmap, 0, 0); } finally { m.bitmap.close(); }
  return ctx.getImageData(0, 0, width, height).data;
}

self.onmessage = (ev) => {
  const m = ev.data;
  // After a failure every later frame is only released (its bitmap closed); close still answers.
  if (failed && m.t === 'f') { m.bitmap?.close(); return; }
  chain = chain.then(async () => {
    if (m.t === 'open') {
      const root = await navigator.storage.getDirectory();
      const fh = await root.getFileHandle(m.name, { create: true });
      handle = await fh.createSyncAccessHandle();
      handle.truncate(0);
      width = m.width; height = m.height; frameBytes = width * height;
      gray = new Uint8Array(frameBytes);
      count = 0; convertMs = []; writeMs = []; failed = null;
      verify = { frames: 0, mismatchFrames: 0, mismatchBytes: 0 };
      self.postMessage({ t: 'opened' });
    } else if (m.t === 'f') {
      if (failed) { m.bitmap?.close(); return; }   // queued before the failure
      const t0 = performance.now();
      grayFromRgba(rgbaOf(m), gray);
      const t1 = performance.now();
      if (m.ref) {
        const ref = new Uint8Array(m.ref);
        let bad = 0;
        for (let k = 0; k < frameBytes; k++) if (ref[k] !== gray[k]) bad++;
        verify.frames++; if (bad) { verify.mismatchFrames++; verify.mismatchBytes += bad; }
      }
      writeAll(handle, gray, m.i * frameBytes);
      count++;
      convertMs.push(t1 - t0); writeMs.push(performance.now() - t1);
      self.postMessage({ t: 'wrote', count });
    } else if (m.t === 'meta') {
      const root = await navigator.storage.getDirectory();
      const mh = await (await root.getFileHandle(m.name, { create: true })).createSyncAccessHandle();
      const bytes = new TextEncoder().encode(m.json);
      try {
        mh.truncate(0);
        writeAll(mh, bytes, 0);
        mh.flush();
      } finally { mh.close(); }
      self.postMessage({ t: 'metaWritten', name: m.name, bytes: bytes.byteLength });
    } else if (m.t === 'close') {
      let bytes = 0;
      if (handle) { handle.flush(); bytes = handle.getSize(); handle.close(); handle = null; }
      self.postMessage({ t: 'closed', count, bytes, costMs: { convert: convertMs, write: writeMs }, verify });
    }
  }).catch((e) => {
    m.bitmap?.close();
    if (m.t === 'f') {
      if (failed) return;   // one error message per failure
      failed = String(e && e.message || e);
    }
    self.postMessage({ t: 'error', message: String(e && e.message || e) });
  });
};
