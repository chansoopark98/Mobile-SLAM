// Spec lane S1: measure the WebGL2 (ANGLE + SwiftShader, same Chrome flags as lane C) behaviour that the
// GPU pyramid depends on, so the CPU port can reproduce the reference environment bit for bit:
//   A. LINEAR filtering of an RGBA8 texture: value returned by texture() for texel pair (0, 255) and
//      (a, a+1) at fractional positions t = k/4096 (read through an RGBA32F render target).
//   B. float -> RGBA8 unorm conversion on write: values (k + d)/255 for d in [0.49, 0.51].
//   C. NEAREST lookup at exact texel boundaries (u = l / W).
//
//   node slam_refact/tools/re/spec_frontend/gl_semantics_probe.mjs --out <json>
import fs from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {parseArgs} from 'node:util';
import {chromium} from '../../../../scripts/dev/node_modules/playwright/index.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, '../../../..');
const {values} = parseArgs({options: {out: {type: 'string'}}});
if (!values.out) throw new Error('--out required');

const pageFn = () => {
  const canvas = document.createElement('canvas');
  const gl = canvas.getContext('webgl2');
  const res = {renderer: null, colorBufferFloat: !!gl.getExtension('EXT_color_buffer_float')};
  const dbg = gl.getExtension('WEBGL_debug_renderer_info');
  res.renderer = dbg ? gl.getParameter(dbg.UNMASKED_RENDERER_WEBGL) : null;
  const vs = `#version 300 es
in vec2 p; out vec2 uv; void main(){ uv = p * 0.5 + 0.5; gl_Position = vec4(p, 0.0, 1.0); }`;
  const prog = fs => {
    const mk = (type, src) => { const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s)); return s; };
    const p = gl.createProgram(); gl.attachShader(p, mk(gl.VERTEX_SHADER, vs)); gl.attachShader(p, mk(gl.FRAGMENT_SHADER, fs));
    gl.bindAttribLocation(p, 0, 'p'); gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p)); return p;
  };
  const vb = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, vb);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]), gl.STATIC_DRAW);
  gl.enableVertexAttribArray(0); gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0);
  const tex = (w, h, data, filter) => { const t = gl.createTexture(); gl.bindTexture(gl.TEXTURE_2D, t);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, data);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, filter); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, filter);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE); return t; };
  const target = (w, h, f32) => { const t = gl.createTexture(); gl.bindTexture(gl.TEXTURE_2D, t);
    if (f32) gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA32F, w, h, 0, gl.RGBA, gl.FLOAT, null);
    else gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
    const fb = gl.createFramebuffer(); gl.bindFramebuffer(gl.FRAMEBUFFER, fb);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, t, 0); gl.viewport(0, 0, w, h); return fb; };
  const read = (w, h, f32) => { const a = f32 ? new Float32Array(w * h * 4) : new Uint8Array(w * h * 4);
    gl.readPixels(0, 0, w, h, gl.RGBA, f32 ? gl.FLOAT : gl.UNSIGNED_BYTE, a); return a; };

  // A. filtering. Texture 4x1: texels (a, b, a, b). Fragment k samples u = (0.5 + k/4096)/4 (between texel 0 and 1).
  const N = 4096;
  const pA = prog(`#version 300 es
precision highp float; uniform sampler2D s; in vec2 uv; out vec4 o;
void main(){ float k = floor(gl_FragCoord.x); float u = (0.5 + k / 4096.0) / 4.0; o = texture(s, vec2(u, 0.5)); }`);
  const pairs = [[0, 255], [1, 2], [100, 101], [37, 38], [254, 255]];
  res.filter = [];
  for (const [a, b] of pairs) {
    const data = new Uint8Array([a, a, a, 255, b, b, b, 255, a, a, a, 255, b, b, b, 255]);
    const t = tex(4, 1, data, gl.LINEAR);
    target(N, 1, true); gl.useProgram(pA); gl.activeTexture(gl.TEXTURE0); gl.bindTexture(gl.TEXTURE_2D, t);
    gl.uniform1i(gl.getUniformLocation(pA, 's'), 0); gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    const out = read(N, 1, true);
    const r = []; for (let k = 0; k < N; k++) r.push(out[4 * k]);
    res.filter.push({a, b, r});
  }
  // B. unorm write rounding: fragment k writes (base + d_k)/255, d_k = 0.49 + 0.02*k/1024, base = k % 200 + 20.
  const pB = prog(`#version 300 es
precision highp float; in vec2 uv; out vec4 o;
void main(){ float k = floor(gl_FragCoord.x); float base = mod(k, 200.0) + 20.0; float d = 0.49 + 0.02 * k / 1024.0;
  o = vec4((base + d) / 255.0, (base + 0.5) / 255.0, base / 255.0 + 0.5 / 255.0, 1.0); }`);
  target(1024, 1, false); gl.useProgram(pB); gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
  const ub = read(1024, 1, false);
  res.unorm = []; for (let k = 0; k < 1024; k++) res.unorm.push([ub[4 * k], ub[4 * k + 1], ub[4 * k + 2]]);
  // C. NEAREST at exact boundaries: 1024x1 texture R = x & 255; fragment k samples u = k / 1024 exactly.
  const d1024 = new Uint8Array(1024 * 4); for (let x = 0; x < 1024; x++) { d1024[4 * x] = x & 255; d1024[4 * x + 3] = 255; }
  const tN = tex(1024, 1, d1024, gl.NEAREST);
  const pC = prog(`#version 300 es
precision highp float; uniform sampler2D s; in vec2 uv; out vec4 o;
void main(){ float k = floor(gl_FragCoord.x); o = texture(s, vec2(k * (1.0 / 1024.0), 0.5)); }`);
  target(1024, 1, true); gl.useProgram(pC); gl.bindTexture(gl.TEXTURE_2D, tN); gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
  const nc = read(1024, 1, true);
  res.nearestBoundary = []; for (let k = 0; k < 1024; k++) res.nearestBoundary.push(Math.round(nc[4 * k] * 255));
  // D. texel -> float conversion: 256x1 texture R = x; NEAREST and LINEAR (at texel centres) fetch.
  const d256 = new Uint8Array(256 * 4); for (let x = 0; x < 256; x++) { d256[4 * x] = x; d256[4 * x + 3] = 255; }
  const pD = prog(`#version 300 es
precision highp float; uniform sampler2D s; in vec2 uv; out vec4 o;
void main(){ float k = floor(gl_FragCoord.x); o = texture(s, vec2((k + 0.5) / 256.0, 0.5)); }`);
  res.texelToFloat = {};
  for (const [name, filter] of [['nearest', gl.NEAREST], ['linear', gl.LINEAR]]) {
    const t = tex(256, 1, d256, filter);
    target(256, 1, true); gl.useProgram(pD); gl.bindTexture(gl.TEXTURE_2D, t); gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    const o = read(256, 1, true); res.texelToFloat[name] = []; for (let k = 0; k < 256; k++) res.texelToFloat[name].push(o[4 * k]);
  }
  // E. LINEAR 2D: 2x2 texture (a b / c d), sample at (fx, fy) from a grid of 64x64 fractions; several texel sets.
  const pE = prog(`#version 300 es
precision highp float; uniform sampler2D s; in vec2 uv; out vec4 o;
void main(){ vec2 k = floor(gl_FragCoord.xy); vec2 f = k / 64.0; o = texture(s, (vec2(0.5) + f) / 2.0); }`);
  const sets = [[0, 255, 0, 255], [0, 0, 255, 255], [10, 20, 30, 40], [57, 56, 58, 57], [200, 13, 77, 140]];
  res.linear2d = [];
  for (const q of sets) {
    const data = new Uint8Array([q[0], q[0], q[0], 255, q[1], q[1], q[1], 255, q[2], q[2], q[2], 255, q[3], q[3], q[3], 255]);
    const t = tex(2, 2, data, gl.LINEAR);
    target(64, 64, true); gl.useProgram(pE); gl.bindTexture(gl.TEXTURE_2D, t); gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    const o = read(64, 64, true); const r = []; for (let k = 0; k < 64 * 64; k++) r.push(o[4 * k]);
    res.linear2d.push({texels: q, r});
  }
  return res;
};

const browser = await chromium.launch({executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
  args: ['--enable-unsafe-swiftshader']});
try {
  const page = await browser.newPage();
  await page.setContent('<!doctype html><html><body></body></html>');
  const res = await page.evaluate(pageFn);
  res.browserVersion = browser.version();
  // Summaries.
  const f0 = res.filter[0];
  const weights = f0.r.map(v => Math.round(v * 255 * 1e6) / 1e6);  // value in units of 1/255 => weight * 255
  const distinct = [...new Set(f0.r)].length;
  res.summary = {
    filterDistinctLevelsFor0_255: distinct,
    filterWeightAt: Object.fromEntries([0, 1, 2, 3, 4, 8, 16, 341, 1024, 1365, 2048, 2730, 3072, 4095].map(k => [k, f0.r[k]])),
    unormRoundingTies: res.unorm.slice(500, 524),
    nearestBoundaryMismatches: res.nearestBoundary.filter((v, k) => v !== (k & 255)).length,
  };
  await fs.mkdir(path.dirname(path.resolve(root, values.out)), {recursive: true});
  await fs.writeFile(path.resolve(root, values.out), JSON.stringify(res) + '\n');
  console.log(JSON.stringify({renderer: res.renderer, colorBufferFloat: res.colorBufferFloat, summary: res.summary}, null, 1));
} finally {
  await browser.close();
}
