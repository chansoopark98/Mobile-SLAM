// Spec lane S1 / U1: read back the exact interpolated `texUv` varying that ANGLE + SwiftShader hands to the
// Gr8 fragment shaders, for the reference runtime quad (c8/pixels/opengl/gl-quad.cc makeVertexArrayRect: 4 vertices,
// indices {0,1,2},{2,1,3}, uv (0,1),(0,0),(1,1),(1,0)) drawn into the pyramid viewports (glViewport(c, r, w, h)).
// The fragment shader writes texUv into an RGBA32F target, so the readback holds the varying bits.
//
//   node slam_refact/tools/re/spec_frontend/varying_probe.mjs --out <dir>
// Output: <dir>/varying.json (cases + browser) and <dir>/case-<i>.f32 (w*h*4 float32, row 0 = viewport row r).
import fs from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath, pathToFileURL} from 'node:url';
import {parseArgs} from 'node:util';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, '../../../..');
const pwModule = process.env.PLAYWRIGHT_MODULE ||
  path.resolve(root, 'scripts/dev/node_modules/playwright/index.mjs');
const {chromium} = await import(pathToFileURL(pwModule).href);
const {values} = parseArgs({options: {out: {type: 'string'}}});
if (!values.out) throw new Error('--out required');

// Viewports (c, r, w, h) of the 1024 atlas layouts (S1 §2.2) and the filtered input, with the vertex rotation
// each stage uses (gr8-rotate*.vert): input 0, stage1 0, decimate -90/+90/0.
const CASES = [
  {name: 'filtered-720x960', vp: [0, 0, 720, 960], rot: 0},
  {name: 'L0-480x640', vp: [3, 3, 480, 640], rot: 0},
  {name: 'L1-444x333-rot-90', vp: [486, 3, 444, 333], rot: -90},
  {name: 'L2-231x308-rot90', vp: [486, 339, 231, 308], rot: 90},
  {name: 'L3-160x214', vp: [720, 339, 160, 214], rot: 0},
  {name: 'L0-360x640', vp: [3, 3, 360, 640], rot: 0},
  {name: 'L1-444x249-rot-90', vp: [366, 3, 444, 249], rot: -90},
  {name: 'L2-173x308-rot90', vp: [366, 255, 173, 308], rot: 90},
  {name: 'L3-120x214', vp: [542, 255, 120, 214], rot: 0},
  {name: 'full-1024', vp: [0, 0, 1024, 1024], rot: 0},
];

const pageFn = (cases) => {
  const canvas = document.createElement('canvas');
  const gl = canvas.getContext('webgl2');
  if (!gl.getExtension('EXT_color_buffer_float')) throw new Error('EXT_color_buffer_float missing');
  const dbg = gl.getExtension('WEBGL_debug_renderer_info');
  const renderer = dbg ? gl.getParameter(dbg.UNMASKED_RENDERER_WEBGL) : null;
  const vs = `#version 300 es
in vec3 position;
in vec2 uv;
out vec2 texUv;
uniform int rotation;
void main() {
  gl_Position = vec4(position, 1.0);
  vec2 fixUv = uv;
  if (rotation == -90 || rotation == 270) {
    texUv = vec2(1.0 - fixUv.y, fixUv.x);
  } else if (rotation == 180 || rotation == -180) {
    texUv = vec2(1.0 - fixUv.x, 1.0 - fixUv.y);
  } else if (rotation == -270 || rotation == 90) {
    texUv = vec2(fixUv.y, 1.0 - fixUv.x);
  } else {
    texUv = fixUv;
  }
}`;
  const fs = `#version 300 es
precision mediump float;
in vec2 texUv;
out vec4 fragColor;
void main() { fragColor = vec4(texUv, 0.0, 1.0); }`;
  const mk = (type, src) => { const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s);
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s)); return s; };
  const prog = gl.createProgram();
  gl.attachShader(prog, mk(gl.VERTEX_SHADER, vs)); gl.attachShader(prog, mk(gl.FRAGMENT_SHADER, fs));
  gl.bindAttribLocation(prog, 0, 'position'); gl.bindAttribLocation(prog, 2, 'uv');
  gl.linkProgram(prog);
  if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
  // makeVertexArrayRect() with the unit sub-rect.
  const vao = gl.createVertexArray(); gl.bindVertexArray(vao);
  const pos = new Float32Array([-1, 1, 0, -1, -1, 0, 1, 1, 0, 1, -1, 0]);
  const uvs = new Float32Array([0, 1, 0, 0, 1, 1, 1, 0]);
  const pb = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, pb); gl.bufferData(gl.ARRAY_BUFFER, pos, gl.STATIC_DRAW);
  gl.enableVertexAttribArray(0); gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
  const ub = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, ub); gl.bufferData(gl.ARRAY_BUFFER, uvs, gl.STATIC_DRAW);
  gl.enableVertexAttribArray(2); gl.vertexAttribPointer(2, 2, gl.FLOAT, false, 0, 0);
  const ib = gl.createBuffer(); gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ib);
  gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Uint8Array([0, 1, 2, 2, 1, 3]), gl.STATIC_DRAW);
  const T = 1024;
  const t = gl.createTexture(); gl.bindTexture(gl.TEXTURE_2D, t);
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA32F, T, T, 0, gl.RGBA, gl.FLOAT, null);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
  const fb = gl.createFramebuffer(); gl.bindFramebuffer(gl.FRAMEBUFFER, fb);
  gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, t, 0);
  gl.useProgram(prog); gl.frontFace(gl.CCW);
  const out = [];
  for (const c of cases) {
    const [x, y, w, h] = c.vp;
    gl.clearColor(-7, -7, -7, -7); gl.viewport(0, 0, T, T); gl.clear(gl.COLOR_BUFFER_BIT);
    gl.viewport(x, y, w, h);
    gl.uniform1i(gl.getUniformLocation(prog, 'rotation'), c.rot);
    gl.drawElements(gl.TRIANGLES, 6, gl.UNSIGNED_BYTE, 0);
    const a = new Float32Array(w * h * 4);
    gl.readPixels(x, y, w, h, gl.RGBA, gl.FLOAT, a);
    const u8 = new Uint8Array(a.buffer);
    let bin = '';
    for (let i = 0; i < u8.length; i += 0x8000) bin += String.fromCharCode.apply(null, u8.subarray(i, i + 0x8000));
    out.push(btoa(bin));
  }
  return {renderer, data: out};
};

const browser = await chromium.launch({executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
  args: ['--enable-unsafe-swiftshader']});
try {
  const page = await browser.newPage();
  await page.setContent('<!doctype html><html><body></body></html>');
  const res = await page.evaluate(pageFn, CASES);
  const outDir = path.resolve(root, values.out);
  await fs.mkdir(outDir, {recursive: true});
  for (let i = 0; i < CASES.length; i++) {
    await fs.writeFile(path.join(outDir, `case-${i}.f32`), Buffer.from(res.data[i], 'base64'));
  }
  const meta = {browserVersion: browser.version(), renderer: res.renderer, cases: CASES,
    layout: 'float32 RGBA, w*h pixels, row 0 = viewport row r (GL window y), R = texUv.x, G = texUv.y'};
  await fs.writeFile(path.join(outDir, 'varying.json'), JSON.stringify(meta, null, 1) + '\n');
  console.log(JSON.stringify({renderer: res.renderer, browser: meta.browserVersion, cases: CASES.length}));
} finally {
  await browser.close();
}
