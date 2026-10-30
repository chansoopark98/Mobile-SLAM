// WebGL2 gr8 pyramid of the slam_refact measurement page: the original reference runtime GPU frontend.
//
// Runs the shaders embedded in the original xr-slam.wasm (web/js/slam-refact-gr8-shaders.js,
// extracted verbatim by slam_refact/tools/re/spec_frontend/extract_gr8_shaders.py) in the draw
// order of the public GlRealityFrame (reality/engine/features/gl-reality-frame.cc:
// drawTexture = drawInputStage, drawInitialStage (+ ROI slot clears), 3 x (blur, decimate),
// stage 2, stage 3; readPixels of dest3). The readback atlas (texture_size x rows RGBA, row 0 =
// GL row 0) is what the engine takes as an injected pyramid (H7, SlamRefactGlue.processFramePyramid)
// instead of running its CPU port of the same shaders. Layout and shader parameters come from the
// engine (SlamRefactGlue.pyramidLayout) so both sides agree on every rectangle.
//
// No DOM: works on a WebGL2 context of an OffscreenCanvas in the engine Worker.
import { inputFrag, noopVert, pyramidBlurFrag, pyramidDecimateFrag, rotateOnlyVert, rotateVert,
  stage1Frag, stage2Frag, stage3Frag } from './slam-refact-gr8-shaders.js';

// gr8-feature-shader.cc:111-140 createSubpixelParams (double math, then float).
export function subpixelParams() {
  const p0 = 0.975316, p1 = 0.006133, p2 = 0.000039;
  const k0 = p0 * p1, k1 = p0 * p2, k2 = p1 * p2;
  const k3 = k1 + 2.0 * k2, k4 = k0 + 2.0 * k1, k5 = k0 + 4.0 * (k1 + k2), k6 = 2.0 * p1 + 4.0 * p2;
  return new Float32Array([0.5 * k3 / k5, 0.5 * k4 / k5, p1 / k6, p2 / k6, k2 / k5, (k4 + 2.0 * k3 - 4.0 * k2) / k5]);
}

const roundToPowerOfTwo = (v) => { let x = v - 1; x |= x >> 1; x |= x >> 2; x |= x >> 4; x |= x >> 8; x |= x >> 16; return x + 1; };

// GlVertexAttrib slots used by Gr8FeatureShader::initialize (position SLOT_0, uv SLOT_2).
const POSITION = 0, UV = 2;

export class Gr8GlPyramid {
  // gl: WebGL2RenderingContext. layout: SlamRefactGlue.pyramidLayout(). camera: engineInit size.
  constructor(gl, layout, { cameraWidth, cameraHeight, rotation = 0 }) {
    if (!(gl && typeof gl.createVertexArray === 'function')) throw new Error('WebGL2 context required');
    if (rotation !== 0) throw new Error('only rotation 0 (portrait) is supported (R16)');
    this.gl = gl;
    this.layout = layout;
    this.cameraWidth = cameraWidth;
    this.cameraHeight = cameraHeight;
    this.rotation = rotation;
    // gl-reality-frame.cc:198-233 initialize: input = camera size (portrait, no input rotation).
    if (layout.rotateInput) throw new Error('landscape engineInit is not supported (R16)');
    this.size = layout.textureSize;
    this.rows = layout.rows;
    const p = layout.shaderParams;

    this.programs = {
      input: this._program(noopVert, inputFrag),
      stage1: this._program(rotateVert, stage1Frag),
      blur: this._program(noopVert, pyramidBlurFrag),
      decimate: this._program(rotateOnlyVert, pyramidDecimateFrag),
      stage2: this._program(noopVert, stage2Frag),
      stage3: this._program(noopVert, stage3Frag),
    };
    // makeVertexArrayRect() (c8/pixels/opengl/gl-quad.cc:26-95): the full-viewport quad; subQuad_
    // over the whole texture has the same vertices and uvs.
    this.quad = gl.createVertexArray();
    gl.bindVertexArray(this.quad);
    this._buffer(gl.ARRAY_BUFFER, new Float32Array([-1, 1, 0, -1, -1, 0, 1, 1, 0, 1, -1, 0]));
    gl.enableVertexAttribArray(POSITION);
    gl.vertexAttribPointer(POSITION, 3, gl.FLOAT, false, 0, 0);
    this._buffer(gl.ARRAY_BUFFER, new Float32Array([0, 1, 0, 0, 1, 1, 1, 0]));
    gl.enableVertexAttribArray(UV);
    gl.vertexAttribPointer(UV, 2, gl.FLOAT, false, 0, 0);
    this._buffer(gl.ELEMENT_ARRAY_BUFFER, new Uint8Array([0, 1, 2, 2, 1, 3]));
    gl.bindVertexArray(null);

    // Camera texture (LINEAR, CLAMP_TO_EDGE), filtered input (pow2, LINEAR), dest1-3 (NEAREST).
    this.camera = this._texture(cameraWidth, cameraHeight, gl.LINEAR);
    this.filtered = this._framebuffer(roundToPowerOfTwo(layout.filteredWidth), roundToPowerOfTwo(layout.filteredHeight), gl.LINEAR);
    this.dest1 = this._framebuffer(this.size, this.size, gl.NEAREST);
    this.dest2 = this._framebuffer(this.size, this.size, gl.NEAREST);
    this.dest3 = this._framebuffer(this.size, this.size, gl.NEAREST);

    // Gr8FeatureShader::initialize sets subpixelQ once; bindAndSetParams sets the stage-3
    // parameters (gain 1, scoreThresh 4, nmsTolerance 1 from c8EmAsm_engineInit, S1 §4).
    const s3 = this.programs.stage3;
    gl.useProgram(s3.id);
    gl.uniform1fv(s3.loc('subpixelQ'), subpixelParams());
    gl.uniform1f(s3.loc('nmsTolerance'), p.nmsTolerance);
    gl.uniform1f(s3.loc('scoreThresh'), p.scoreThresh);
    gl.uniform1f(s3.loc('gain'), p.gain);
    gl.uniform1i(s3.loc('skipSubpixel'), p.skipSubpixel ? 1 : 0);
    gl.useProgram(null);
    this.readback = new Uint8Array(this.size * this.rows * 4);
    this._check('init');
  }

  _buffer(target, data) {
    const gl = this.gl, b = gl.createBuffer();
    gl.bindBuffer(target, b);
    gl.bufferData(target, data, gl.STATIC_DRAW);
    return b;
  }

  _program(vs, fs) {
    const gl = this.gl;
    const sh = (type, src) => {
      const s = gl.createShader(type);
      gl.shaderSource(s, src);
      gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(`gr8 shader compile: ${gl.getShaderInfoLog(s)}`);
      return s;
    };
    const id = gl.createProgram();
    gl.attachShader(id, sh(gl.VERTEX_SHADER, vs));
    gl.attachShader(id, sh(gl.FRAGMENT_SHADER, fs));
    gl.bindAttribLocation(id, POSITION, 'position');
    gl.bindAttribLocation(id, UV, 'uv');
    gl.linkProgram(id);
    if (!gl.getProgramParameter(id, gl.LINK_STATUS)) throw new Error(`gr8 program link: ${gl.getProgramInfoLog(id)}`);
    const cache = new Map();
    return { id, loc: (name) => { if (!cache.has(name)) cache.set(name, gl.getUniformLocation(id, name)); return cache.get(name); } };
  }

  _texture(w, h, filter) {
    const gl = this.gl, t = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, t);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, filter);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, filter);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    return { tex: t, width: w, height: h };
  }

  _framebuffer(w, h, filter) {
    const gl = this.gl, t = this._texture(w, h, filter), fb = gl.createFramebuffer();
    gl.bindFramebuffer(gl.FRAMEBUFFER, fb);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, t.tex, 0);
    if (gl.checkFramebufferStatus(gl.FRAMEBUFFER) !== gl.FRAMEBUFFER_COMPLETE) throw new Error('gr8 framebuffer incomplete');
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    return { ...t, fb };
  }

  _check(what) {
    const e = this.gl.getError();
    if (e !== this.gl.NO_ERROR) throw new Error(`[gr8gl] ${what}: GL error 0x${e.toString(16)}`);
  }

  // source: ImageBitmap / VideoFrame / ImageData / OffscreenCanvas (cameraWidth x cameraHeight), or
  // { rgba: Uint8Array, width, height } (row 0 = image top).
  upload(source) {
    const gl = this.gl;
    gl.bindTexture(gl.TEXTURE_2D, this.camera.tex);
    gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);
    gl.pixelStorei(gl.UNPACK_PREMULTIPLY_ALPHA_WEBGL, false);
    gl.pixelStorei(gl.UNPACK_ALIGNMENT, 4);
    if (source && source.rgba) {
      if (source.width !== this.cameraWidth || source.height !== this.cameraHeight) throw new Error('camera frame size differs from engineInit');
      gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, source.width, source.height, gl.RGBA, gl.UNSIGNED_BYTE, source.rgba);
    } else {
      const w = source.displayWidth ?? source.width, h = source.displayHeight ?? source.height;
      if (w !== this.cameraWidth || h !== this.cameraHeight) throw new Error(`camera frame ${w}x${h} differs from engineInit`);
      gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, gl.RGBA, gl.UNSIGNED_BYTE, source);
    }
  }

  _bindSrcDst(src, dst) {
    const gl = this.gl;
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, src.tex);
    gl.bindFramebuffer(gl.FRAMEBUFFER, dst.fb);
  }

  // GlRealityFrame::drawTexture (gl-reality-frame.cc:532-569) on the uploaded camera texture.
  draw() {
    const gl = this.gl, L = this.layout.levels, P = this.programs, size = this.size;
    gl.disable(gl.BLEND);
    gl.bindVertexArray(this.quad);
    const quad = () => gl.drawElements(gl.TRIANGLES, 6, gl.UNSIGNED_BYTE, 0);
    const shaderRotation = (l) => (l.rotated ? this.rotation - 90 : this.rotation);

    // drawInputStage: filtered input = 3x3 blur of the camera texture, viewport 720 x 960.
    gl.useProgram(P.input.id);
    this._bindSrcDst(this.camera, this.filtered);
    gl.uniform2f(P.input.loc('scale'), 1 / this.cameraWidth, 1 / this.cameraHeight);
    gl.viewport(0, 0, this.layout.filteredWidth, this.layout.filteredHeight);
    quad();

    // drawInitialStage: level 0 from the filtered input, then the four ROI slots cleared.
    const s1 = P.stage1;
    gl.useProgram(s1.id);
    this._bindSrcDst(this.filtered, this.dest1);
    const identity = new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
    gl.uniformMatrix4fv(s1.loc('mvp'), false, identity);
    gl.uniform1i(s1.loc('clear'), 0);
    gl.uniform1i(s1.loc('flipY'), 0);
    gl.uniform2f(s1.loc('scale'), 1 / this.filtered.width, 1 / this.filtered.height);
    gl.uniform2f(s1.loc('pixScale'), this.layout.filteredWidth, this.layout.filteredHeight);
    gl.uniform1i(s1.loc('rotation'), shaderRotation(L[0]));
    gl.viewport(L[0].c, L[0].r, L[0].w, L[0].h);
    quad();
    gl.bindTexture(gl.TEXTURE_2D, this.camera.tex);
    gl.uniform2f(s1.loc('scale'), 1 / this.cameraWidth, 1 / this.cameraHeight);
    gl.uniform2f(s1.loc('pixScale'), this.cameraWidth, this.cameraHeight);
    for (const l of this.layout.roiSlots) {
      gl.uniform1i(s1.loc('rotation'), shaderRotation(l));
      gl.viewport(l.c, l.r, l.w, l.h);
      gl.uniformMatrix4fv(s1.loc('mvp'), false, identity);
      gl.uniform1i(s1.loc('clear'), 1);
      quad();
    }

    // 3 x (drawPyramidBlurStage(dest1 -> dest2, i), drawPyramidDecimateStage(dest2 -> dest1, i)).
    for (let i = 0; i < 3; i++) {
      const b = P.blur, l = L[i];
      gl.useProgram(b.id);
      this._bindSrcDst(this.dest1, this.dest2);
      gl.viewport(l.c, l.r, l.w, l.h);
      gl.uniform4f(b.loc('roi'), l.c, l.r, l.w, l.h);
      gl.uniform2f(b.loc('scale'), 1 / size, 1 / size);
      quad();
      const d = P.decimate, n = L[i + 1];
      gl.useProgram(d.id);
      this._bindSrcDst(this.dest2, this.dest1);
      gl.uniform4f(d.loc('roi'), l.c, l.r, l.w, l.h);
      gl.viewport(n.c, n.r, n.w, n.h);
      gl.uniform2f(d.loc('scale'), 1 / size, 1 / size);
      gl.uniform1i(d.loc('rotation'), shaderRotation(n) - shaderRotation(l));
      quad();
    }

    // drawStage(dest1 -> dest2, stage 2), drawStage(dest2 -> dest3, stage 3), full texture.
    for (const [prog, src, dst] of [[P.stage2, this.dest1, this.dest2], [P.stage3, this.dest2, this.dest3]]) {
      gl.useProgram(prog.id);
      this._bindSrcDst(src, dst);
      gl.viewport(0, 0, size, size);
      gl.uniform2f(prog.loc('scale'), 1 / size, 1 / size);
      gl.uniform1i(prog.loc('skipSubpixel'), this.layout.shaderParams.skipSubpixel ? 1 : 0);
      quad();
    }
    gl.bindVertexArray(null);
    this._check('draw');
  }

  // glReadPixels of dest3 rows [0, rows): the engine's atlas (the CPU half reads only inside the
  // level rectangles, all above `rows`; the original reads the whole texture through a PBO).
  read() {
    const gl = this.gl;
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.dest3.fb);
    gl.pixelStorei(gl.PACK_ALIGNMENT, 4);
    gl.readPixels(0, 0, this.size, this.rows, gl.RGBA, gl.UNSIGNED_BYTE, this.readback);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    this._check('readPixels');
    return this.readback;
  }

  dispose() {
    const gl = this.gl;
    for (const p of Object.values(this.programs)) gl.deleteProgram(p.id);
    for (const t of [this.camera, this.filtered, this.dest1, this.dest2, this.dest3]) {
      gl.deleteTexture(t.tex);
      if (t.fb) gl.deleteFramebuffer(t.fb);
    }
    gl.deleteVertexArray(this.quad);
  }
}
