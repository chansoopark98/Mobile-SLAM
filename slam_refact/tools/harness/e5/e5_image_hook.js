/* E5 copy of tools/re/runtime/image-target-hook.js: only change = the detectedImagesRows cap 400 -> 100000 (the whole sequence is recorded). */
/* Image-target hook for the original-SDK reference runs (full-parity plan Rev B, OR-4;
 * build/slam-refact-full-20261019/preregistration/or4-it-world-regression.v5.md; used through run-reference-it.mjs).
 * Installed with page.addInitScript before page-runtime.js (same mechanism as api-oracle.js). On xrloaded (before
 * page-runtime's own listener) it
 *   - merges `imageTargetData` into page-runtime's XrController.configure call (the public API: the SDK loads
 *     imagePath through an <img>, builds the DetectionImage in the engine and dispatches imageloading /
 *     imagescanning; the page serves the files from the harness origin),
 *   - appends a pipeline module that records reality.imagefound / imageupdated / imagelost / imageloading /
 *     imagescanning events (the frame is the next onUpdate: events are dispatched during processCpu) and, for fresh
 *     callbacks, the reality keys and reality.detectedImages if the SDK exposes it,
 *   - decodes each served target image once in the page and records its size and sha256 of the red channel plus
 *     whether R = G = B, so the run can prove the browser decoded the same luminance as the engine's PGM.
 * Read-only with respect to the SDK: no SDK bytes, exports or memory are changed. Results: window.__imageTargetHook.
 * Without --image-targets this file is not loaded and a run is byte-identical to before. */
window.__imageTargetHookInit = options => {
  const state = window.__imageTargetHook = {options: {imageTargetNames: options.imageTargetData.map(t => t.name)}, installed: false, error: null,
    configureCalls: [], events: [], decode: [], realityKeys: null, detectedImagesSeen: 0, detectedImagesRows: []};
  const sha256Hex = async bytes => Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', bytes)), b => b.toString(16).padStart(2, '0')).join('');
  window.addEventListener('xrloaded', () => {
    try {
      const xc = XR8.XrController;
      const configure = xc.configure.bind(xc);
      xc.configure = args => {
        const merged = {...(args || {}), imageTargetData: options.imageTargetData};
        state.configureCalls.push({keys: Object.keys(merged), targets: merged.imageTargetData.length});
        return configure(merged);
      };
      const pending = [];
      const names = ['reality.imageloading', 'reality.imagescanning', 'reality.imagefound', 'reality.imageupdated', 'reality.imagelost'];
      const add = XR8.addCameraPipelineModules.bind(XR8);
      XR8.addCameraPipelineModules = modules => add([...modules, {
        name: 'image-target-hook',
        listeners: names.map(event => ({event, process: args => pending.push({name: event, at: performance.now(), detail: JSON.parse(JSON.stringify(args.detail ?? null))})})),
        onUpdate: ({frameStartResult: f, processCpuResult: p}) => {
          const r = p && p.reality;
          for (const e of pending.splice(0)) state.events.push({...e, videoTime: f.videoTime, repeat: f.repeatFrame ? 1 : 0});
          if (!r || f.repeatFrame) return;
          if (!state.realityKeys) state.realityKeys = Object.keys(r);
          if (r.detectedImages && r.detectedImages.length) {
            state.detectedImagesSeen++;
            if (state.detectedImagesRows.length < 100000) state.detectedImagesRows.push({videoTime: f.videoTime, detectedImages: JSON.parse(JSON.stringify(r.detectedImages))});
          }
        },
      }]);
      state.installed = true;
      (async () => {
        for (const t of options.imageTargetData) {
          try {
            const img = new Image();
            await new Promise((res, rej) => { img.onload = res; img.onerror = () => rej(new Error('image load failed ' + t.imagePath)); img.src = t.imagePath; });
            const c = document.createElement('canvas'); c.width = img.naturalWidth; c.height = img.naturalHeight;
            const ctx = c.getContext('2d', {willReadFrequently: true}); ctx.drawImage(img, 0, 0);
            const d = ctx.getImageData(0, 0, c.width, c.height).data;
            const red = new Uint8Array(c.width * c.height); let grey = true;
            for (let i = 0; i < red.length; i++) { red[i] = d[4 * i]; if (d[4 * i] !== d[4 * i + 1] || d[4 * i] !== d[4 * i + 2] || d[4 * i + 3] !== 255) grey = false; }
            state.decode.push({name: t.name, width: c.width, height: c.height, greyRgbEqual: grey, redSha256: await sha256Hex(red)});
          } catch (error) { state.decode.push({name: t.name, error: String(error)}); }
        }
      })();
    } catch (error) { state.error = String(error && error.stack || error); }
  }, {once: true});
};
