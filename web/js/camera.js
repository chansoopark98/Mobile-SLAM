/**
 * Capture natural HTMLVideoElement pixels, then apply explicit rotation and crop.
 * videoWidth/videoHeight define the drawImage source domain. Scene brightness
 * never determines orientation. Camera/body calibration must describe this domain.
 */

const CAMERA_VERSION = 'v10';

// Profiles describe drawImage pixels before this module's explicit rotation.
// A browser's implicit orientation is not recoverable from videoWidth alone;
// profile dimensions and orientation must match the actual drawImage domain.
export function validateCameraProfile(profile) {
    if (!profile || profile.schema !== 'mobile-slam-camera-profile-v1' || profile.pixelFrame !== 'drawImage') throw new Error('Invalid camera profile schema/pixelFrame');
    if (typeof profile.provenance !== 'string' || !profile.provenance.trim()) throw new Error('Camera profile requires calibration provenance');
    if (!['portrait-primary', 'portrait-secondary', 'landscape-primary', 'landscape-secondary'].includes(profile.orientationType)) throw new Error('Camera profile requires orientationType');
    if (![profile.width, profile.height].every(n => Number.isInteger(n) && n > 0) ||
        ![profile.fx, profile.fy, profile.cx, profile.cy].every(Number.isFinite) || profile.fx <= 0 || profile.fy <= 0 ||
        profile.cx < -0.5 || profile.cx > profile.width - 0.5 || profile.cy < -0.5 || profile.cy > profile.height - 0.5) throw new Error('Invalid camera intrinsics');
    if (![0, 2].includes(profile.modelType)) throw new Error('Unsupported camera model: use KANNALA_BRANDT=0 or PINHOLE=2');
    if (!Array.isArray(profile.distortion) || profile.distortion.length !== 4 || !profile.distortion.every(Number.isFinite)) throw new Error('Invalid camera distortion');
    const r = profile.r_ic;
    if (!Array.isArray(r) || r.length !== 9 || !r.every(Number.isFinite)) throw new Error('Invalid camera/body rotation');
    for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) {
        let dot = 0; for (let k = 0; k < 3; k++) dot += r[k * 3 + a] * r[k * 3 + b];
        if (Math.abs(dot - (a === b ? 1 : 0)) > 1e-6) throw new Error('Camera/body rotation must be orthogonal');
    }
    const determinant = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) + r[2] * (r[3] * r[7] - r[4] * r[6]);
    if (Math.abs(determinant - 1) > 1e-6) throw new Error('Camera/body rotation must have determinant +1');
    if (!Array.isArray(profile.t_ic) || profile.t_ic.length !== 3 || !profile.t_ic.every(Number.isFinite)) throw new Error('Invalid camera/body translation');
    return profile;
}

export function processingDimensions(width, height, scale = 1) {
    if (![width, height, scale].every(Number.isFinite) || width <= 0 || height <= 0 || scale <= 0 || scale > 1) throw new Error('Invalid processing dimensions');
    return { width: scale === 1 ? width : Math.max(2, Math.round(width * scale) & ~1),
        height: scale === 1 ? height : Math.max(2, Math.round(height * scale) & ~1) };
}

export function transformCameraProfile(profile, transform) {
    validateCameraProfile(profile);
    const rotations = {
        none: [1,0,0,0,1,0,0,0,1], cw: [0,-1,0,1,0,0,0,0,1],
        ccw: [0,1,0,-1,0,0,0,0,1], half: [-1,0,0,0,-1,0,0,0,1],
    };
    const q = rotations[transform.rotation];
    if (!q) throw new Error('Invalid pixel rotation');
    const swap = transform.rotation === 'cw' || transform.rotation === 'ccw';
    const rotatedW = swap ? profile.height : profile.width;
    const rotatedH = swap ? profile.width : profile.height;
    const { cropX, cropY, cropWidth, cropHeight, width, height } = transform;
    if (![cropX,cropY,cropWidth,cropHeight,width,height].every(Number.isInteger) || cropX < 0 || cropY < 0 ||
        Math.min(cropWidth,cropHeight,width,height) <= 0 || cropX + cropWidth > rotatedW || cropY + cropHeight > rotatedH) throw new Error('Invalid camera crop/resize');
    let cx = profile.cx, cy = profile.cy;
    if (transform.rotation === 'cw') { cx = profile.height - 1 - profile.cy; cy = profile.cx; }
    else if (transform.rotation === 'ccw') { cx = profile.cy; cy = profile.width - 1 - profile.cx; }
    else if (transform.rotation === 'half') { cx = profile.width - 1 - profile.cx; cy = profile.height - 1 - profile.cy; }
    const sx = width / cropWidth, sy = height / cropHeight;
    const r_ic = new Array(9).fill(0);
    // C_new = Q C_old => R_B_Cnew = R_B_Cold Q^T; body lever arm unchanged.
    for (let row = 0; row < 3; row++) for (let col = 0; col < 3; col++) for (let k = 0; k < 3; k++) r_ic[row * 3 + col] += profile.r_ic[row * 3 + k] * q[col * 3 + k];
    let [k2,k3,k4,k5] = profile.distortion;
    if (profile.modelType === 2) {
        // PINHOLE API slots k4/k5 are tangential p1/p2. Rotate [p2,p1].
        const p1 = k4, p2 = k5;
        k4 = q[3] * p2 + q[4] * p1;
        k5 = q[0] * p2 + q[1] * p1;
    }
    return { width, height, fx: (swap ? profile.fy : profile.fx) * sx, fy: (swap ? profile.fx : profile.fy) * sy,
        cx: (cx - cropX + 0.5) * sx - 0.5, cy: (cy - cropY + 0.5) * sy - 0.5,
        modelType: profile.modelType, k2,k3,k4,k5,r_ic,t_ic: [...profile.t_ic] };
}

export class Camera {
    constructor() {
        this.video = null;
        this.canvas = null;
        this.ctx = null;
        this.stream = null;
        this.width = 0;
        this.height = 0;
        this.running = false;

        // Explicit pixel rotation; never inferred from image brightness.
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
        this._cropOffsetX = 0;
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
        console.log(`[Camera] ${CAMERA_VERSION} — natural video capture`);
        if (this.stream) throw new Error('Camera capture already exists; stop before initializing again');
        const cameraParams = new URLSearchParams(window.location.search);
        const cameraNumbers = cameraParams.getAll('camera');
        if (cameraNumbers.length > 1 || (cameraNumbers.length === 1 && !/^(0|[1-9]\d*)$/.test(cameraNumbers[0]))) {
            throw new Error('Invalid camera selection: use one rear camera number');
        }
        const cameraNumber = cameraNumbers.length ? cameraNumbers[0] : null;

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

        const openDefault = async () => {
            try {
                return await navigator.mediaDevices.getUserMedia(constraints);
            } catch (e) {
                console.warn(`[Camera] exact 'environment' failed (${e.message}), trying ideal`);
                constraints.video.facingMode = 'environment';
                return navigator.mediaDevices.getUserMedia(constraints);
            }
        };
        if (cameraNumber !== null) {
            // A number identifies an observed label, not a universal lens type.
            let devices = await navigator.mediaDevices.enumerateDevices();
            const videoDevices = devices.filter(device => device.kind === 'videoinput');
            if (videoDevices.length && videoDevices.every(device => !device.label)) {
                // Only bootstrap permission if labels are hidden, then release our
                // temporary capture before opening exclusive rear-camera hardware.
                const bootstrap = await openDefault();
                try {
                    devices = await navigator.mediaDevices.enumerateDevices();
                } finally {
                    bootstrap.getTracks().forEach(track => track.stop());
                }
            }
            const matches = devices.filter(device => device.kind === 'videoinput' &&
                device.label === `camera ${cameraNumber}, facing back`);
            if (matches.length !== 1 || !matches[0].deviceId) {
                throw new Error(`Rear camera ${cameraNumber} is missing or ambiguous`);
            }
            const selected = matches[0];
            const isSelectedRear = stream => {
                const tracks = stream.getVideoTracks();
                if (tracks.length !== 1 || tracks[0].readyState !== 'live') return false;
                const settings = tracks[0].getSettings();
                return settings.deviceId === selected.deviceId && settings.facingMode === 'environment';
            };
            let capture = null;
            try {
                capture = await navigator.mediaDevices.getUserMedia({
                    ...constraints,
                    video: { ...constraints.video, facingMode: { exact: 'environment' },
                        deviceId: { exact: selected.deviceId } },
                });
                if (!isSelectedRear(capture)) throw new Error(`Rear camera ${cameraNumber} returned an unexpected device or facing`);
                this.stream = capture;
            } catch (error) {
                if (capture) capture.getTracks().forEach(track => track.stop());
                throw error;
            }
            console.log(`[Camera] ${CAMERA_VERSION} explicit rear camera ${cameraNumber}`);
        } else {
            this.stream = await openDefault();
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

            // Request 1x on the selected camera; this does not choose a main lens.
            try {
                const caps = activeTrack.getCapabilities();
                if (caps && caps.zoom) {
                    const zoomRange = caps.zoom;
                    console.log(`[Camera] ${CAMERA_VERSION} zoom range: ${zoomRange.min}-${zoomRange.max} (current: ${settings.zoom || 'N/A'})`);
                    if (zoomRange.min <= 1.0 && zoomRange.max >= 1.0) {
                        await activeTrack.applyConstraints({ advanced: [{ zoom: 1.0 }] });
                        const updatedSettings = activeTrack.getSettings();
                        console.log(`[Camera] ${CAMERA_VERSION} zoom set to ${updatedSettings.zoom || 1.0} on selected camera`);
                    }
                }
            } catch (zoomErr) {
                console.log(`[Camera] ${CAMERA_VERSION} zoom constraint not supported: ${zoomErr.message}`);
            }
        }

        const urlRotate = new URLSearchParams(window.location.search).get('rotate');
        this._rotateMode = ['none', 'cw', 'ccw', 'half'].includes(urlRotate) ? urlRotate : 'none';
        this._refreshPixelGeometry();
        console.log(`[Camera] ${CAMERA_VERSION} Source: ${this._nativeWidth}x${this._nativeHeight}, output: ${this.width}x${this.height}, rotation: ${this._rotateMode}`);

        // The canvas holds the full rotated source; crop is applied on readback.
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
        } else if (this._rotateMode === 'half') {
            this.ctx.save();
            this.ctx.translate(cw, ch);
            this.ctx.rotate(Math.PI);
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

        // 4. Use the same crop rectangle as the CPU readback.
        const uLeft = this._cropOffsetX / this._portraitWidth;
        const uRight = (this._cropOffsetX + this.width) / this._portraitWidth;
        const vTop = this._cropOffsetY / this._portraitHeight;
        const vBottom = (this._cropOffsetY + this.height) / this._portraitHeight;

        // Update UV coords in vertex buffer for the crop region
        // prettier-ignore
        const verts = new Float32Array([
            // x,    y,    u,    v
            -1.0, -1.0,  uLeft,  vBottom,
             1.0, -1.0,  uRight, vBottom,
            -1.0,  1.0,  uLeft,  vTop,
             1.0,  1.0,  uRight, vTop,
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
     * Enable centered 4:3 crop within the rotated image, using even dimensions.
     * Must be called after initialize().
     */
    enableLandscapeCrop() {
        if (!this.running) return;
        this._cropMode = 'landscape_4_3';
        this._refreshPixelGeometry();
    }

    /** Uncropped rotated dimensions; retained name for existing callers. */
    getPortraitDimensions() {
        return { width: this._portraitWidth, height: this._portraitHeight };
    }

    getPixelTransform(width = this.width, height = this.height) {
        return { sourceWidth: this._nativeWidth, sourceHeight: this._nativeHeight,
            rotation: this._rotateMode, cropX: this._cropOffsetX,
            cropY: this._cropOffsetY, cropWidth: this.width, cropHeight: this.height, width, height };
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
        } else if (this._rotateMode === 'half') {
            this.ctx.save();
            this.ctx.translate(cw, ch);
            this.ctx.rotate(Math.PI);
            this.ctx.drawImage(this.video, 0, 0, this._nativeWidth, this._nativeHeight);
            this.ctx.restore();
        } else {
            // No manual rotation — browser handles orientation
            this.ctx.drawImage(this.video, 0, 0, cw, ch);
        }

        // Extract pixels: full portrait or center crop for landscape 4:3
        const imageData = this.ctx.getImageData(this._cropOffsetX, this._cropOffsetY, this.width, this.height);
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

    _refreshPixelGeometry() {
        const width = this.video.videoWidth, height = this.video.videoHeight;
        if (![width, height].every(value => Number.isInteger(value) && value > 0)) throw new Error('Camera natural dimensions unavailable');
        this._nativeWidth = width;
        this._nativeHeight = height;
        this._dimsSwapped = this._rotateMode === 'cw' || this._rotateMode === 'ccw';
        this._portraitWidth = this._dimsSwapped ? height : width;
        this._portraitHeight = this._dimsSwapped ? width : height;
        this.width = this._portraitWidth;
        this.height = this._portraitHeight;
        if (this._cropMode === 'landscape_4_3') {
            this.width = Math.min(this._portraitWidth, Math.floor(this._portraitHeight * 4 / 3) & ~1);
            this.height = Math.min(this._portraitHeight, Math.floor(this._portraitWidth * 3 / 4) & ~1);
            if (this.width < 2 || this.height < 2) throw new Error('Camera dimensions too small for crop');
        }
        this._cropOffsetX = Math.floor((this._portraitWidth - this.width) / 2);
        this._cropOffsetY = Math.floor((this._portraitHeight - this.height) / 2);
        if (this.canvas && (this.canvas.width !== this._portraitWidth || this.canvas.height !== this._portraitHeight)) {
            this.canvas.width = this._portraitWidth;
            this.canvas.height = this._portraitHeight;
        }
        this._grayBuffer = null;
    }

    /** Preserve the requested pixel rotation while refreshing natural video size. */
    async redetectOrientation() {
        if (this.running && this.video) this._refreshPixelGeometry();
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
