/**
 * Camera capture module using navigator.mediaDevices.getUserMedia.
 * Captures frames from the environment-facing camera and extracts grayscale data.
 *
 * v7: Robust rotation handling for VINS-Mono coordinate system.
 *
 * The grayscale image MUST be portrait-oriented for the VIO engine:
 *   X_c = right in image (phone right edge)
 *   Y_c = down in image (phone bottom edge)
 *   Z_c = forward (into scene)
 *
 * Rotation logic:
 *   - If videoWidth < videoHeight: browser already auto-rotated dimensions
 *     → drawImage also auto-rotates → no manual rotation needed ('none')
 *   - If videoWidth > videoHeight: browser reports raw sensor dims
 *     → drawImage does NOT auto-rotate → apply CCW 90° rotation
 *   - URL ?rotate=none|cw|ccw overrides all detection
 *
 * Output is always portrait: width < height (e.g., 480x640, 3:4 aspect ratio).
 * fx, fy, cx, cy are computed for these portrait dimensions.
 */

const CAMERA_VERSION = 'v8';

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

        // Create offscreen canvas matching output dimensions
        this.canvas = document.createElement('canvas');
        this.canvas.width = this.width;
        this.canvas.height = this.height;
        this.ctx = this.canvas.getContext('2d', { willReadFrequently: true });

        this.running = true;
        return { width: this.width, height: this.height, rotated: this._rotateMode !== 'none' };
    }

    /**
     * Capture a grayscale frame in portrait orientation.
     *
     * CCW rotation transform (landscape 640x480 → portrait 480x640):
     *   translate(0, outH) + rotate(-PI/2) + drawImage(video, 0, 0, nW, nH)
     *   Maps sensor pixels so that scene-up → image-up in portrait canvas.
     *
     * @returns {Uint8Array|null} Grayscale pixel data (width * height bytes), portrait-oriented
     */
    captureGrayscale() {
        if (!this.running || !this.video || this.video.readyState < 2) {
            return null;
        }

        const t0 = performance.now();

        if (this._rotateMode === 'cw') {
            this.ctx.save();
            this.ctx.translate(this.width, 0);
            this.ctx.rotate(Math.PI / 2);
            this.ctx.drawImage(this.video, 0, 0, this._nativeWidth, this._nativeHeight);
            this.ctx.restore();
        } else if (this._rotateMode === 'ccw') {
            this.ctx.save();
            this.ctx.translate(0, this.height);
            this.ctx.rotate(-Math.PI / 2);
            this.ctx.drawImage(this.video, 0, 0, this._nativeWidth, this._nativeHeight);
            this.ctx.restore();
        } else {
            // No manual rotation — browser handles orientation
            this.ctx.drawImage(this.video, 0, 0, this.width, this.height);
        }

        const imageData = this.ctx.getImageData(0, 0, this.width, this.height);
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

        const dt = performance.now() - t0;
        const n = this._captureCount;
        if (n < 5 || n % 100 === 0) {
            console.log(`[Camera] captureGrayscale: ${dt.toFixed(1)}ms (${this.width}x${this.height})`);
        }
        this._captureCount = n + 1;

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

        // Update output dimensions
        if (this._rotateMode === 'cw' || this._rotateMode === 'ccw' || nativeIsLandscape) {
            this.width = this._nativeHeight;
            this.height = this._nativeWidth;
            this._dimsSwapped = true;
        } else {
            this.width = this._nativeWidth;
            this.height = this._nativeHeight;
            this._dimsSwapped = false;
        }

        // Resize offscreen canvas if dimensions changed
        if (this.canvas && (this.canvas.width !== this.width || this.canvas.height !== this.height)) {
            this.canvas.width = this.width;
            this.canvas.height = this.height;
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
    }
}
