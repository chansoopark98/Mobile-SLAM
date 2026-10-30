// Gray conversion of the opt-in input recorder (?record=1), shared by the main-thread recorder module
// and its writer Worker (no DOM, no network).
// RGBA (Uint8ClampedArray / Uint8Array, 4 bytes per pixel) -> gray bytes,
// gray = round(0.299 R + 0.587 G + 0.114 B) in double precision ((x + 0.5) | 0 equals Math.round(x)
// for x >= 0), the recording format since schema slam-refact-input-recording-v1.
export function grayFromRgba(rgba, out) {
  const n = rgba.length >> 2;
  const g = out ?? new Uint8Array(n);
  for (let i = 0, j = 0; i < n; i++, j += 4) g[i] = (0.299 * rgba[j] + 0.587 * rgba[j + 1] + 0.114 * rgba[j + 2] + 0.5) | 0;
  return g;
}
