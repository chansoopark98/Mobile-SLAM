/**
 * Camera capture module using navigator.mediaDevices.getUserMedia.
 * Captures frames from the environment-facing camera and extracts grayscale data.
 *
 * Handles video frame orientation:
 * Mobile camera sensors are physically landscape (e.g., 640x480).
 * Modern browsers (Chrome 93+, Safari 15+) auto-rotate canvas.drawImage()
 * content to match the display orientation. This means drawImage() output
 * is already portrait-correct — no manual rotation is needed.
 *
 * For older browsers that do NOT auto-rotate drawImage, use URL override:
 *   ?rotate=ccw  — for Android rear cameras (sensor orientation 90°)
 *   ?rotate=cw   — for some Samsung/front cameras (sensor orientation 270°)
 *   ?rotate=none  — explicit no rotation (default)
 */

const CAMERA_VERSION = 'v5';

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
        // Native (un-rotated) video dimensions reported by videoWidth/videoHeight
        this._nativeWidth = 0;
        this._nativeHeight = 0;
        // Whether output dims are swapped from native (for portrait output from landscape sensor)
        this._dimsSwapped = false;
    }

    /**
     * Initialize camera capture.
     * @param {number} targetWidth - Desired capture width
     * @param {number} targetHeight - Desired capture height
     * @param {boolean} [expectPortrait=null] - If true, expect portrait output.
     * @returns {Promise<{width: number, height: number, rotated: boolean}>}
     */
    async initialize(targetWidth = 640, targetHeight = 480, expectPortrait = null) {
        console.log(`[Camera] ${CAMERA_VERSION} — browser auto-rotate default`);

        // Infer expected orientation if not specified
        if (expectPortrait === null) {
            expectPortrait = window.innerHeight > window.innerWidth;
        }

        // Request dimensions matching the expected orientation.
        let reqW = targetWidth;
        let reqH = targetHeight;
        if (expectPortrait && targetWidth > targetHeight) {
            reqW = targetHeight;
            reqH = targetWidth;
        } else if (!expectPortrait && targetHeight > targetWidth) {
            reqW = targetHeight;
            reqH = targetWidth;
        }

        // Force rear camera. Try exact first, fall back to ideal.
        let constraints = {
            video: {
                facingMode: { exact: 'environment' },
                width: { ideal: reqW },
                height: { ideal: reqH },
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

        // Wait for actual frame data to be available
        while (this.video.readyState < 2) {
            await new Promise(r => setTimeout(r, 50));
        }

        this._nativeWidth = this.video.videoWidth;
        this._nativeHeight = this.video.videoHeight;

        // Log camera track info for diagnostics
        const activeTrack = this.stream.getVideoTracks()[0];
        if (activeTrack) {
            const settings = activeTrack.getSettings();
            this._facingMode = settings.facingMode || 'unknown';
            console.log(`[Camera] ${CAMERA_VERSION} facingMode: ${this._facingMode}`);
            console.log(`[Camera] ${CAMERA_VERSION} track settings: ${settings.width}x${settings.height}`);
            console.log(`[Camera] ${CAMERA_VERSION} videoWidth/Height: ${this._nativeWidth}x${this._nativeHeight}`);
            if (this._facingMode !== 'environment' && this._facingMode !== 'unknown') {
                console.warn(`[Camera] WARNING: Not using rear camera (facingMode=${this._facingMode})`);
            }
        }

        // Determine if native dimensions match expected orientation
        const nativePortrait = this._nativeHeight > this._nativeWidth;
        const needsSwap = (expectPortrait !== nativePortrait);

        // Check URL override: ?rotate=none|cw|ccw
        const urlRotate = new URLSearchParams(window.location.search).get('rotate');
        if (urlRotate && ['none', 'cw', 'ccw'].includes(urlRotate)) {
            this._rotateMode = urlRotate;
            console.log(`[Camera] ${CAMERA_VERSION} Rotation override from URL: ${urlRotate}`);
        } else if (!needsSwap) {
            // Native dimensions already match expected orientation.
            this._rotateMode = 'none';
            console.log(`[Camera] ${CAMERA_VERSION} Native dims match expected → no rotation`);
        } else {
            // Native is landscape but we expect portrait (or vice versa).
            //
            // Modern Chrome Android (93+) and Safari (15+) auto-rotate
            // canvas.drawImage(video) content to match the display orientation.
            // The video element's <video> display is auto-rotated by the compositor,
            // AND drawImage() output is also auto-rotated.
            //
            // So we just need portrait-sized canvas and draw without rotation.
            // The browser handles the rotation internally.
            //
            // If the preview looks DISTORTED (squished, not rotated):
            //   Your browser does NOT auto-rotate drawImage.
            //   Use ?rotate=ccw (most Android) or ?rotate=cw (some Samsung).
            this._rotateMode = 'none';
            console.log(`[Camera] ${CAMERA_VERSION} Native ${this._nativeWidth}x${this._nativeHeight} ` +
                `doesn't match expected ${expectPortrait ? 'portrait' : 'landscape'}`);
            console.log(`[Camera] ${CAMERA_VERSION} Using browser auto-rotation (no manual rotation)`);
            console.log(`[Camera] ${CAMERA_VERSION} If preview looks distorted, try ?rotate=ccw or ?rotate=cw`);
        }

        // Set output dimensions
        // CW/CCW: always swap (manual rotation handles orientation)
        // none with needsSwap: swap dims, rely on browser auto-rotation
        // none without needsSwap: use native dims
        if (this._rotateMode === 'cw' || this._rotateMode === 'ccw' || needsSwap) {
            this.width = this._nativeHeight;
            this.height = this._nativeWidth;
            this._dimsSwapped = true;
        } else {
            this.width = this._nativeWidth;
            this.height = this._nativeHeight;
            this._dimsSwapped = false;
        }

        console.log(`[Camera] ${CAMERA_VERSION} Output: ${this.width}x${this.height}, ` +
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
     * Capture a grayscale frame.
     * Applies rotation based on mode to ensure output matches screen orientation.
     *
     * Rotation transforms (for landscape 640x480 → portrait 480x640):
     *   CW:  translate(outW, 0) + rotate(+PI/2)
     *        Correct for sensor orientation 270° (scene top → sensor left)
     *   CCW: translate(0, outH) + rotate(-PI/2)
     *        Correct for sensor orientation 90° (scene top → sensor right)
     *   none (with swapped dims): drawImage scales to portrait canvas
     *        Correct when browser auto-rotates drawImage content
     *
     * @returns {Uint8Array|null} Grayscale pixel data (width * height bytes)
     */
    captureGrayscale() {
        if (!this.running || !this.video || this.video.readyState < 2) {
            return null;
        }

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
            // No manual rotation — draw at output dimensions.
            // If dims are swapped (portrait from landscape sensor), the browser's
            // auto-rotation handles orientation. drawImage scales content to fit.
            this.ctx.drawImage(this.video, 0, 0, this.width, this.height);
        }

        const imageData = this.ctx.getImageData(0, 0, this.width, this.height);
        const rgba = imageData.data;
        const gray = new Uint8Array(this.width * this.height);

        // RGBA to grayscale: Y = 0.299R + 0.587G + 0.114B (BT.601 integer approx)
        for (let i = 0, j = 0; i < rgba.length; i += 4, j++) {
            gray[j] = (rgba[i] * 77 + rgba[i + 1] * 150 + rgba[i + 2] * 29) >> 8;
        }

        return gray;
    }

    /** Whether the camera frame is being rotated */
    isRotated() {
        return this._rotateMode !== 'none';
    }

    /** Get rotation mode: 'none' | 'cw' | 'ccw' */
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

    /** Get the active video track for settings/capabilities queries */
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
