/**
 * Camera capture module using navigator.mediaDevices.getUserMedia.
 * Captures frames from the environment-facing camera and extracts grayscale data.
 *
 * v9: Landscape 4:3 crop + zoom=1.0 standard lens selection.
 *
 * Pipeline:
 *   1. Capture portrait from rear camera (480x640) with zoom=1.0 (standard lens)
 *   2. Rotate if needed (CCW for raw landscape sensors)
 *   3. Center-crop to landscape 4:3 (480x360) for VIO
 *
 * Camera coordinate system (same regardless of crop):
 *   X_c = right in image (phone right edge)
 *   Y_c = down in image (phone bottom edge)
 *   Z_c = forward (into scene)
 *
 * Rotation logic:
 *   - If videoWidth < videoHeight: browser already auto-rotated → no rotation ('none')
 *   - If videoWidth > videoHeight: raw sensor dims → apply CCW 90° rotation
 *   - URL ?rotate=none|cw|ccw overrides all detection
 */

const CAMERA_VERSION = 'v9';

export class Camera {
    constructor() {
        this.video = null;
        this.canvas = null;
        this.ctx = null;
        this.stream = null;
        this.width = 0;
        this.height = 0;
        this.running = false;

        // Rotation mode: 'none' | 'cw' | 'ccw'
        this._rotateMode = 'none';
        // Native video dimensions reported by videoWidth/videoHeight
        this._nativeWidth = 0;
        this._nativeHeight = 0;
        // Whether output dims are swapped from native
        this._dimsSwapped = false;

        // Cached output buffer — reused each frame to avoid GC pressure
        this._grayBuffer = null;
        // Capture timing counter
        this._captureCount = 0;

        // Landscape crop mode
        this._cropMode = 'none'; // 'none' | 'landscape_4_3'
        this._portraitWidth = 0;
        this._portraitHeight = 0;
        this._cropOffsetY = 0;

        // WebGL grayscale path (optional, enabled via ?grayscale=webgl)
        this._useWebGL = new URLSearchParams(window.location.search).get('grayscale') === 'webgl';
        this._glCanvas = null;
        this._gl = null;
        this._glProgram = null;
        this._glTexture = null;
        this._glReadBuffer = null; // RGBA readPixels buffer
        this._glContextLost = false;
        this._glTimingFrames = 0; // frames logged for timing comparison
    }

    /**
     * Initialize camera capture.
     * Output is always portrait-oriented to match the phone's screen and
     * VINS-Mono camera coordinate convention.
     *
     * @param {number} targetWidth - Desired capture width (larger dim)
     * @param {number} targetHeight - Desired capture height (smaller dim)
     * @returns {Promise<{width: number, height: number, rotated: boolean}>}
     */
    async initialize(targetWidth = 640, targetHeight = 480) {
        console.log(`[Camera] ${CAMERA_VERSION} — VINS-Mono portrait capture`);

        // Request landscape dimensions (camera sensor is naturally landscape)
        const reqW = Math.max(targetWidth, targetHeight);
        const reqH = Math.min(targetWidth, targetHeight);

        // Force rear camera
        let constraints = {
            video: {
                facingMode: { exact: 'environment' },
                width: { ideal: reqW },
                height: { ideal: reqH },
                frameRate: { ideal: 30, min: 20 },
            },
            audio: false,
        };

        try {
            this.stream = await navigator.mediaDevices.getUserMedia(constraints);
        } catch (e) {
            console.warn(`[Camera] exact 'environment' failed (${e.message}), trying ideal`);
            constraints.video.facingMode = 'environment';
            this.stream = await navigator.mediaDevices.getUserMedia(constraints);
        }
        this.video = document.createElement('video');
        this.video.setAttribute('playsinline', '');
        this.video.setAttribute('autoplay', '');
        this.video.srcObject = this.stream;

        await new Promise((resolve) => {
            this.video.onloadedmetadata = () => {
                this.video.play();
                resolve();
            };
        });

        while (this.video.readyState < 2) {
            await new Promise(r => setTimeout(r, 50));
        }

        this._nativeWidth = this.video.videoWidth;
        this._nativeHeight = this.video.videoHeight;

        // Log camera track info
        const activeTrack = this.stream.getVideoTracks()[0];
        if (activeTrack) {
            const settings = activeTrack.getSettings();
            this._facingMode = settings.facingMode || 'unknown';
            console.log(`[Camera] ${CAMERA_VERSION} facingMode: ${this._facingMode}`);
            console.log(`[Camera] ${CAMERA_VERSION} track settings: ${settings.width}x${settings.height}`);
            console.log(`[Camera] ${CAMERA_VERSION} videoWidth/Height: ${this._nativeWidth}x${this._nativeHeight}`);

            // Prefer standard (1x) lens over ultrawide by setting zoom=1.0.
            // Many phones default to ultrawide (~100° FOV) for 'environment' camera,
            // which breaks VIO focal length estimation (assumes ~69° standard lens).
            try {
                const caps = activeTrack.getCapabilities();
                if (caps && caps.zoom) {
                    const zoomRange = caps.zoom;
                    console.log(`[Camera] ${CAMERA_VERSION} zoom range: ${zoomRange.min}-${zoomRange.max} (current: ${settings.zoom || 'N/A'})`);
                    if (zoomRange.min <= 1.0 && zoomRange.max >= 1.0) {
                        await activeTrack.applyConstraints({ advanced: [{ zoom: 1.0 }] });
                        const updatedSettings = activeTrack.getSettings();
                        console.log(`[Camera] ${CAMERA_VERSION} zoom set to ${updatedSettings.zoom || 1.0} (standard lens)`);
                    }
                }
            } catch (zoomErr) {
                console.log(`[Camera] ${CAMERA_VERSION} zoom constraint not supported: ${zoomErr.message}`);
            }
        }

        // Determine rotation mode
        const nativeIsLandscape = this._nativeWidth > this._nativeHeight;

        // URL override takes highest priority
        const urlRotate = new URLSearchParams(window.location.search).get('rotate');
        if (urlRotate && ['none', 'cw', 'ccw'].includes(urlRotate)) {
            this._rotateMode = urlRotate;
            console.log(`[Camera] ${CAMERA_VERSION} Rotation override from URL: ${urlRotate}`);
        } else if (nativeIsLandscape) {
            // Browser reports raw sensor dimensions (e.g., 640x480).
            // But drawImage() may or may not auto-rotate the pixel content.
            // Use a pixel test to detect: draw video to a small canvas and check
            // whether the actual pixel aspect ratio matches landscape or portrait.
            const needsRotation = await this._detectDrawImageOrientation();
            if (needsRotation) {
                this._rotateMode = 'ccw';
                console.log(`[Camera] ${CAMERA_VERSION} drawImage returns raw landscape → applying CCW rotation`);
            } else {
                // drawImage already auto-rotates to portrait despite landscape videoWidth/Height
                this._rotateMode = 'none';
                console.log(`[Camera] ${CAMERA_VERSION} drawImage auto-rotates to portrait → no rotation needed`);
            }
        } else {
            // Browser reports portrait dimensions (e.g., 480x640).
            // This means browser auto-rotated both videoWidth AND drawImage content.
            // No manual rotation needed.
            this._rotateMode = 'none';
            console.log(`[Camera] ${CAMERA_VERSION} Browser auto-rotated to portrait ${this._nativeWidth}x${this._nativeHeight} → no rotation needed`);
        }

        // Set output dimensions — always portrait
        if (this._rotateMode === 'cw' || this._rotateMode === 'ccw') {
            // Manual rotation: swap native dims for portrait output
            this.width = this._nativeHeight;   // e.g., 480
            this.height = this._nativeWidth;   // e.g., 640
            this._dimsSwapped = true;
        } else if (nativeIsLandscape) {
            // 'none' mode with landscape native (URL override ?rotate=none):
            // Still swap dims for portrait canvas, rely on browser auto-rotation
            this.width = this._nativeHeight;
            this.height = this._nativeWidth;
            this._dimsSwapped = true;
        } else {
            // Native is already portrait
            this.width = this._nativeWidth;
            this.height = this._nativeHeight;
            this._dimsSwapped = false;
        }

        console.log(`[Camera] ${CAMERA_VERSION} Output: ${this.width}x${this.height} (${this.width < this.height ? 'portrait' : 'landscape'}), ` +
            `Rotation: ${this._rotateMode}, DimsSwapped: ${this._dimsSwapped}`);

        // Store full portrait dimensions (before any crop)
        this._portraitWidth = this.width;
        this._portraitHeight = this.height;

        // Create offscreen canvas at full portrait size (crop is done at getImageData level)
        this.canvas = document.createElement('canvas');
        this.canvas.width = this._portraitWidth;
        this.canvas.height = this._portraitHeight;
        this.ctx = this.canvas.getContext('2d', { willReadFrequently: true });

        this.running = true;

        // Initialize WebGL grayscale path if requested
        if (this._useWebGL) {
            this._initWebGL();
        }

        return { width: this.width, height: this.height, rotated: this._rotateMode !== 'none' };
    }

    /**
     * Initialize WebGL resources for GPU-accelerated grayscale conversion.
     * Called only when ?grayscale=webgl is set.
     * Uses a hidden offscreen canvas with WebGL1 for maximum compatibility.
     */
    _initWebGL() {
        try {
            this._glCanvas = document.createElement('canvas');
            this._glCanvas.style.display = 'none';
            document.body.appendChild(this._glCanvas);

            const gl = this._glCanvas.getContext('webgl', {
                antialias: false,
                depth: false,
                stencil: false,
                alpha: false,
                preserveDrawingBuffer: true,
            });
            if (!gl) {
                console.warn('[Camera] WebGL not available, falling back to CPU grayscale');
                this._useWebGL = false;
                return;
            }
            this._gl = gl;

            // Handle context loss
            this._glCanvas.addEventListener('webglcontextlost', (e) => {
                e.preventDefault();
                this._glContextLost = true;
                console.warn('[Camera] WebGL context lost, falling back to CPU grayscale');
            });
            this._glCanvas.addEventListener('webglcontextrestored', () => {
                this._glContextLost = false;
                console.log('[Camera] WebGL context restored');
                this._initWebGLResources(gl);
            });

            this._initWebGLResources(gl);
            console.log(`[Camera] WebGL grayscale enabled (WebGL1)`);
        } catch (e) {
            console.warn('[Camera] WebGL init failed, falling back to CPU:', e.message);
            this._useWebGL = false;
        }
    }

    /**
     * Create WebGL shader program, texture, and geometry.
     * Called on init and after context restore.
     */
    _initWebGLResources(gl) {
        const vsSource = `
            attribute vec2 a_pos;
            attribute vec2 a_uv;
            varying vec2 v_uv;
            void main() {
                gl_Position = vec4(a_pos, 0.0, 1.0);
                v_uv = a_uv;
            }
        `;
        const fsSource = `
            precision mediump float;
            uniform sampler2D u_tex;
            varying vec2 v_uv;
            void main() {
                vec4 c = texture2D(u_tex, v_uv);
                float y = dot(c.rgb, vec3(0.299, 0.587, 0.114));
                gl_FragColor = vec4(y, y, y, 1.0);
            }
        `;

        const vs = this._compileShader(gl, gl.VERTEX_SHADER, vsSource);
        const fs = this._compileShader(gl, gl.FRAGMENT_SHADER, fsSource);
        if (!vs || !fs) { this._useWebGL = false; return; }

        const prog = gl.createProgram();
        gl.attachShader(prog, vs);
        gl.attachShader(prog, fs);
        gl.linkProgram(prog);
        if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) {
            console.warn('[Camera] WebGL link error:', gl.getProgramInfoLog(prog));
            this._useWebGL = false;
            return;
        }
        this._glProgram = prog;

        // Full-screen quad: positions in NDC, UVs flipped-Y so (0,0) = top-left
        // prettier-ignore
        const verts = new Float32Array([
            // x,    y,   u,   v
            -1.0, -1.0,  0.0, 1.0,
             1.0, -1.0,  1.0, 1.0,
            -1.0,  1.0,  0.0, 0.0,
             1.0,  1.0,  1.0, 0.0,
        ]);
        const buf = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buf);
        gl.bufferData(gl.ARRAY_BUFFER, verts, gl.STATIC_DRAW);

        const posLoc = gl.getAttribLocation(prog, 'a_pos');
        const uvLoc  = gl.getAttribLocation(prog, 'a_uv');
        gl.enableVertexAttribArray(posLoc);
        gl.enableVertexAttribArray(uvLoc);
        gl.vertexAttribPointer(posLoc, 2, gl.FLOAT, false, 16, 0);
        gl.vertexAttribPointer(uvLoc,  2, gl.FLOAT, false, 16, 8);

        // Texture for video frames
        const tex = gl.createTexture();
        gl.bindTexture(gl.TEXTURE_2D, tex);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
        this._glTexture = tex;
    }

    _compileShader(gl, type, src) {
        const s = gl.createShader(type);
        gl.shaderSource(s, src);
        gl.compileShader(s);
        if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) {
            console.warn('[Camera] WebGL shader compile error:', gl.getShaderInfoLog(s));
            return null;
        }
        return s;
    }

    /**
     * Capture grayscale using WebGL shader.
     * The video is drawn via a portrait canvas first (same rotation logic as CPU path),
     * then the portrait canvas is used as the WebGL texture source.
     * The crop region is rendered by adjusting UV coordinates.
     *
     * Returns Uint8Array of width*height grayscale pixels (R channel from RGBA readPixels).
     */
    _captureGrayscaleWebGL() {
        const gl = this._gl;
        if (!gl || this._glContextLost) return null;

        // 1. Draw video to portrait canvas (same rotation logic as CPU path)
        const cw = this.canvas.width;
        const ch = this.canvas.height;

        if (this._rotateMode === 'cw') {
            this.ctx.save();
            this.ctx.translate(cw, 0);
            this.ctx.rotate(Math.PI / 2);
            this.ctx.drawImage(this.video, 0, 0, this._nativeWidth, this._nativeHeight);
            this.ctx.restore();
        } else if (this._rotateMode === 'ccw') {
            this.ctx.save();
            this.ctx.translate(0, ch);
            this.ctx.rotate(-Math.PI / 2);
            this.ctx.drawImage(this.video, 0, 0, this._nativeWidth, this._nativeHeight);
            this.ctx.restore();
        } else {
            this.ctx.drawImage(this.video, 0, 0, cw, ch);
        }

        // 2. Resize WebGL canvas to output dimensions
        if (this._glCanvas.width !== this.width || this._glCanvas.height !== this.height) {
            this._glCanvas.width = this.width;
            this._glCanvas.height = this.height;
            gl.viewport(0, 0, this.width, this.height);
            // Reallocate readPixels buffer
            this._glReadBuffer = new Uint8Array(this.width * this.height * 4);
        }

        // 3. Upload portrait canvas as texture
        gl.bindTexture(gl.TEXTURE_2D, this._glTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, this.canvas);

        // 4. Render with crop via UV coordinates
        //    Crop: x in [0, portraitW], y in [cropOffsetY, cropOffsetY + outputH]
        //    UV: u in [0,1], v in [cropOffsetY/portraitH, (cropOffsetY+outputH)/portraitH]
        const pH = this._portraitHeight;
        const vTop    = this._cropMode === 'landscape_4_3' ? this._cropOffsetY / pH : 0.0;
        const vBottom = this._cropMode === 'landscape_4_3' ? (this._cropOffsetY + this.height) / pH : 1.0;

        // Update UV coords in vertex buffer for the crop region
        // prettier-ignore
        const verts = new Float32Array([
            // x,    y,    u,    v
            -1.0, -1.0,  0.0,  vBottom,
             1.0, -1.0,  1.0,  vBottom,
            -1.0,  1.0,  0.0,  vTop,
             1.0,  1.0,  1.0,  vTop,
        ]);
        gl.bufferData(gl.ARRAY_BUFFER, verts, gl.DYNAMIC_DRAW);

        gl.useProgram(this._glProgram);
        gl.uniform1i(gl.getUniformLocation(this._glProgram, 'u_tex'), 0);
        gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);

        // 5. Read back RGBA pixels, extract R channel as grayscale
        if (!this._glReadBuffer || this._glReadBuffer.length !== this.width * this.height * 4) {
            this._glReadBuffer = new Uint8Array(this.width * this.height * 4);
        }
        gl.readPixels(0, 0, this.width, this.height, gl.RGBA, gl.UNSIGNED_BYTE, this._glReadBuffer);

        // 6. Extract R channel into grayscale buffer (R=G=B from shader)
        const pixelCount = this.width * this.height;
        if (!this._grayBuffer || this._grayBuffer.length !== pixelCount) {
            this._grayBuffer = new Uint8Array(pixelCount);
        }
        const gray = this._grayBuffer;
        const rgba = this._glReadBuffer;

        // readPixels returns bottom-to-top rows; flip vertically to match CPU path (top-to-bottom)
        const rowBytes = this.width * 4;
        for (let row = 0; row < this.height; row++) {
            const srcRow = this.height - 1 - row;
            const srcBase = srcRow * rowBytes;
            const dstBase = row * this.width;
            for (let col = 0; col < this.width; col++) {
                gray[dstBase + col] = rgba[srcBase + col * 4]; // R channel
            }
        }

        return gray;
    }

    /**
     * Enable landscape 4:3 crop from portrait image.
     * Crops the center vertical region: full width, height = width * 3/4.
     * Output becomes landscape (width > height) with 4:3 aspect ratio.
     * Must be called after initialize().
     *
     * This improves VIO by:
     * - Matching standard SLAM landscape image assumption
     * - Reducing vertical extent (floor/ceiling) while keeping useful horizontal features
     * - Reducing total pixels → faster processing
     */
    enableLandscapeCrop() {
        if (this._portraitWidth === 0) return;
        this._cropMode = 'landscape_4_3';
        // Landscape 4:3: width = portrait width (shorter physical side), height = width * 3/4
        const cropH = Math.floor(this._portraitWidth * 3 / 4) & ~1; // ensure even
        this._cropOffsetY = Math.floor((this._portraitHeight - cropH) / 2);
        this.width = this._portraitWidth;
        this.height = cropH;
        this._grayBuffer = null; // reallocate for new size
        console.log(`[Camera] ${CAMERA_VERSION} Landscape 4:3 crop: ` +
            `${this._portraitWidth}x${this._portraitHeight} → ${this.width}x${this.height} ` +
            `(offsetY=${this._cropOffsetY}, aspect=${(this.width/this.height).toFixed(2)})`);
    }

    /** Get original portrait dimensions (before crop) */
    getPortraitDimensions() {
        return { width: this._portraitWidth, height: this._portraitHeight };
    }

    /**
     * Capture a grayscale frame.
     *
     * Steps:
     *   1. Draw video to full portrait canvas (with rotation if needed)
     *   2. Extract pixel region (full portrait or landscape 4:3 center crop)
     *   3. Convert RGBA → grayscale
     *
     * @returns {Uint8Array|null} Grayscale pixel data (width * height bytes)
     */
    captureGrayscale() {
        if (!this.running || !this.video || this.video.readyState < 2) {
            return null;
        }

        const n = this._captureCount;
        const logTiming = n < 10 || n % 100 === 0;

        // WebGL path (optional, ?grayscale=webgl)
        if (this._useWebGL && !this._glContextLost) {
            const t0 = performance.now();
            const result = this._captureGrayscaleWebGL();
            if (result) {
                if (logTiming) {
                    const dtGL = performance.now() - t0;
                    // For the first few frames measure CPU timing on a separate buffer
                    if (this._glTimingFrames < 5) {
                        // Temporarily swap _grayBuffer so CPU path doesn't overwrite WebGL result
                        const savedBuf = this._grayBuffer;
                        this._grayBuffer = null;
                        const tCPU = performance.now();
                        this._captureGrayscaleCPU();
                        const dtCPU = performance.now() - tCPU;
                        this._grayBuffer = savedBuf; // restore WebGL result buffer
                        console.log(`[Camera] grayscale timing #${n}: WebGL=${dtGL.toFixed(1)}ms  CPU=${dtCPU.toFixed(1)}ms (${this.width}x${this.height})`);
                        this._glTimingFrames++;
                    } else {
                        console.log(`[Camera] captureGrayscale (WebGL): ${dtGL.toFixed(1)}ms (${this.width}x${this.height})`);
                    }
                }
                this._captureCount = n + 1;
                return result;
            }
            // WebGL returned null — fall through to CPU
            console.warn('[Camera] WebGL captureGrayscale returned null, falling back to CPU');
            this._useWebGL = false;
        }

        // CPU path (default)
        const t0 = performance.now();
        const gray = this._captureGrayscaleCPU();
        if (logTiming) {
            console.log(`[Camera] captureGrayscale (CPU): ${(performance.now() - t0).toFixed(1)}ms (${this.width}x${this.height})`);
        }
        this._captureCount = n + 1;
        return gray;
    }

    /**
     * CPU-based grayscale conversion (original path, always available as fallback).
     * Draws video to portrait canvas, extracts crop region, converts RGBA → grayscale.
     * Returns the shared _grayBuffer (Uint8Array).
     */
    _captureGrayscaleCPU() {
        // Draw video to full portrait canvas (canvas always at portrait dimensions)
        const cw = this.canvas.width;
        const ch = this.canvas.height;

        if (this._rotateMode === 'cw') {
            this.ctx.save();
            this.ctx.translate(cw, 0);
            this.ctx.rotate(Math.PI / 2);
            this.ctx.drawImage(this.video, 0, 0, this._nativeWidth, this._nativeHeight);
            this.ctx.restore();
        } else if (this._rotateMode === 'ccw') {
            this.ctx.save();
            this.ctx.translate(0, ch);
            this.ctx.rotate(-Math.PI / 2);
            this.ctx.drawImage(this.video, 0, 0, this._nativeWidth, this._nativeHeight);
            this.ctx.restore();
        } else {
            // No manual rotation — browser handles orientation
            this.ctx.drawImage(this.video, 0, 0, cw, ch);
        }

        // Extract pixels: full portrait or center crop for landscape 4:3
        const cropY = this._cropMode === 'landscape_4_3' ? this._cropOffsetY : 0;
        const imageData = this.ctx.getImageData(0, cropY, this.width, this.height);
        const rgba = imageData.data;

        // Reuse output buffer to avoid per-frame allocation / GC pressure
        const pixelCount = this.width * this.height;
        if (!this._grayBuffer || this._grayBuffer.length !== pixelCount) {
            this._grayBuffer = new Uint8Array(pixelCount);
        }
        const gray = this._grayBuffer;

        for (let i = 0, j = 0; i < rgba.length; i += 4, j++) {
            gray[j] = (rgba[i] * 77 + rgba[i + 1] * 150 + rgba[i + 2] * 29) >> 8;
        }

        return gray;
    }

    /**
     * Detect whether drawImage() returns raw (landscape) or auto-rotated (portrait) pixels.
     * Draws video to a small test canvas at native dims and checks the actual content
     * aspect ratio by comparing edge brightness distributions.
     *
     * Returns true if drawImage returns raw landscape data (needs manual rotation).
     * Returns false if drawImage auto-rotates to portrait (no rotation needed).
     *
     * Fallback: if detection is inconclusive, assumes raw landscape (needs rotation),
     * since that's the more common case on Android Chrome.
     */
    async _detectDrawImageOrientation() {
        try {
            // Draw video at native dimensions to a test canvas
            const testW = this._nativeWidth;
            const testH = this._nativeHeight;
            const testCanvas = document.createElement('canvas');
            testCanvas.width = testW;
            testCanvas.height = testH;
            const testCtx = testCanvas.getContext('2d');
            testCtx.drawImage(this.video, 0, 0, testW, testH);
            const testData = testCtx.getImageData(0, 0, testW, testH).data;

            // Check if drawn pixels fill the canvas in landscape or portrait pattern.
            // If drawImage auto-rotated to portrait but canvas is landscape (W>H),
            // the content will have black/empty bars on the sides.
            // Sample the rightmost column — if mostly black, content is narrower than canvas.

            // Count non-black pixels in the rightmost 5% of columns
            const sampleStartX = Math.floor(testW * 0.95);
            let rightEdgeBrightness = 0;
            let sampleCount = 0;
            for (let y = 0; y < testH; y += 4) {
                for (let x = sampleStartX; x < testW; x += 2) {
                    const idx = (y * testW + x) * 4;
                    rightEdgeBrightness += testData[idx] + testData[idx + 1] + testData[idx + 2];
                    sampleCount++;
                }
            }
            const avgRightBrightness = sampleCount > 0 ? rightEdgeBrightness / (sampleCount * 3) : 128;

            // Count non-black pixels in the center
            const centerStartX = Math.floor(testW * 0.4);
            const centerEndX = Math.floor(testW * 0.6);
            const centerStartY = Math.floor(testH * 0.4);
            const centerEndY = Math.floor(testH * 0.6);
            let centerBrightness = 0;
            let centerCount = 0;
            for (let y = centerStartY; y < centerEndY; y += 4) {
                for (let x = centerStartX; x < centerEndX; x += 2) {
                    const idx = (y * testW + x) * 4;
                    centerBrightness += testData[idx] + testData[idx + 1] + testData[idx + 2];
                    centerCount++;
                }
            }
            const avgCenterBrightness = centerCount > 0 ? centerBrightness / (centerCount * 3) : 128;

            // If right edge is very dark compared to center, drawImage auto-rotated
            // and the content doesn't fill the landscape canvas → portrait content in landscape canvas
            const ratio = avgCenterBrightness > 1 ? avgRightBrightness / avgCenterBrightness : 1;

            console.log(`[Camera] ${CAMERA_VERSION} drawImage orientation test: ` +
                `rightEdge=${avgRightBrightness.toFixed(1)}, center=${avgCenterBrightness.toFixed(1)}, ` +
                `ratio=${ratio.toFixed(3)}`);

            if (ratio < 0.15 && avgCenterBrightness > 10) {
                // Right edge is black, center has content → auto-rotated portrait in landscape canvas
                return false;  // no manual rotation needed
            }

            // Content fills the landscape canvas → raw landscape data
            return true;  // needs manual rotation
        } catch (e) {
            console.warn(`[Camera] ${CAMERA_VERSION} drawImage orientation detection failed:`, e.message);
            // Fallback: assume raw landscape (most common on Android)
            return true;
        }
    }

    /**
     * Re-detect orientation after screen rotation.
     * The video stream stays the same, but drawImage behavior may change.
     * Updates rotation mode, output dimensions, and offscreen canvas.
     */
    async redetectOrientation() {
        if (!this.running || !this.video) return;

        // URL override takes priority — never re-detect
        const urlRotate = new URLSearchParams(window.location.search).get('rotate');
        if (urlRotate && ['none', 'cw', 'ccw'].includes(urlRotate)) return;

        // Re-read native dimensions (may have swapped after orientation change)
        this._nativeWidth = this.video.videoWidth;
        this._nativeHeight = this.video.videoHeight;
        const nativeIsLandscape = this._nativeWidth > this._nativeHeight;

        const oldMode = this._rotateMode;
        if (nativeIsLandscape) {
            const needsRotation = await this._detectDrawImageOrientation();
            this._rotateMode = needsRotation ? 'ccw' : 'none';
        } else {
            this._rotateMode = 'none';
        }

        // Update portrait dimensions
        if (this._rotateMode === 'cw' || this._rotateMode === 'ccw' || nativeIsLandscape) {
            this._portraitWidth = this._nativeHeight;
            this._portraitHeight = this._nativeWidth;
            this._dimsSwapped = true;
        } else {
            this._portraitWidth = this._nativeWidth;
            this._portraitHeight = this._nativeHeight;
            this._dimsSwapped = false;
        }

        // Re-apply crop if active, otherwise use portrait dims
        if (this._cropMode === 'landscape_4_3') {
            const cropH = Math.floor(this._portraitWidth * 3 / 4) & ~1;
            this._cropOffsetY = Math.floor((this._portraitHeight - cropH) / 2);
            this.width = this._portraitWidth;
            this.height = cropH;
        } else {
            this.width = this._portraitWidth;
            this.height = this._portraitHeight;
        }

        // Resize offscreen canvas to portrait (canvas always full portrait for drawing)
        if (this.canvas && (this.canvas.width !== this._portraitWidth || this.canvas.height !== this._portraitHeight)) {
            this.canvas.width = this._portraitWidth;
            this.canvas.height = this._portraitHeight;
        }

        if (oldMode !== this._rotateMode) {
            console.log(`[Camera] ${CAMERA_VERSION} Orientation re-detected: ${oldMode}→${this._rotateMode}, output=${this.width}x${this.height}`);
        }
    }

    /** Whether the camera frame is being rotated */
    isRotated() {
        return this._rotateMode !== 'none';
    }

    /** Get rotation mode */
    getRotateMode() {
        return this._rotateMode;
    }

    /** Whether output dimensions are swapped from native */
    isDimsSwapped() {
        return this._dimsSwapped;
    }

    /** Get the video element for display */
    getVideoElement() {
        return this.video;
    }

    /** Get the active video track */
    getVideoTrack() {
        if (this.stream) {
            const tracks = this.stream.getVideoTracks();
            return tracks.length > 0 ? tracks[0] : null;
        }
        return null;
    }

    /** Stop camera capture */
    stop() {
        this.running = false;
        if (this.stream) {
            this.stream.getTracks().forEach((track) => track.stop());
            this.stream = null;
        }
        if (this.video) {
            this.video.srcObject = null;
        }
        // Clean up WebGL resources
        if (this._gl) {
            const gl = this._gl;
            if (this._glTexture) { gl.deleteTexture(this._glTexture); this._glTexture = null; }
            if (this._glProgram) { gl.deleteProgram(this._glProgram); this._glProgram = null; }
            this._gl = null;
        }
        if (this._glCanvas) {
            this._glCanvas.remove();
            this._glCanvas = null;
        }
        this._glReadBuffer = null;
    }
}
