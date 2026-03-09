/**
 * Camera capture module using navigator.mediaDevices.getUserMedia.
 * Captures frames from the environment-facing camera and extracts grayscale data.
 *
 * Handles video frame orientation mismatch:
 * Some mobile devices return landscape-oriented video even when the screen is
 * in portrait mode. This module detects the mismatch and applies a canvas
 * rotation so the output grayscale image always matches the screen orientation.
 */
export class Camera {
    constructor() {
        this.video = null;
        this.canvas = null;
        this.ctx = null;
        this.stream = null;
        this.width = 0;
        this.height = 0;
        this.running = false;

        // Video frame rotation: true if canvas rotation is applied
        this._rotateFrame = false;
        // Native (un-rotated) video dimensions
        this._nativeWidth = 0;
        this._nativeHeight = 0;
    }

    /**
     * Initialize camera capture.
     * Requests dimensions matching the expected screen orientation to encourage
     * the browser to auto-rotate. If the browser returns mismatched orientation,
     * a canvas rotation is applied during capture.
     *
     * @param {number} targetWidth - Desired capture width
     * @param {number} targetHeight - Desired capture height
     * @param {boolean} [expectPortrait=null] - If true, expect portrait output.
     *   If null, inferred from window dimensions.
     * @returns {Promise<{width: number, height: number, rotated: boolean}>}
     *   Effective capture dimensions (after any rotation) and rotation flag.
     */
    async initialize(targetWidth = 640, targetHeight = 480, expectPortrait = null) {
        // Infer expected orientation if not specified
        if (expectPortrait === null) {
            expectPortrait = window.innerHeight > window.innerWidth;
        }

        // Request dimensions matching the expected orientation.
        // This hints to the browser that we want portrait/landscape-oriented video.
        let reqW = targetWidth;
        let reqH = targetHeight;
        if (expectPortrait && targetWidth > targetHeight) {
            // Swap to request portrait
            reqW = targetHeight;
            reqH = targetWidth;
        } else if (!expectPortrait && targetHeight > targetWidth) {
            // Swap to request landscape
            reqW = targetHeight;
            reqH = targetWidth;
        }

        const constraints = {
            video: {
                facingMode: 'environment',
                width: { ideal: reqW },
                height: { ideal: reqH },
            },
            audio: false,
        };

        this.stream = await navigator.mediaDevices.getUserMedia(constraints);
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

        this._nativeWidth = this.video.videoWidth;
        this._nativeHeight = this.video.videoHeight;

        // Detect orientation mismatch:
        // If we expected portrait but got landscape (or vice versa), we need rotation.
        const nativePortrait = this._nativeHeight > this._nativeWidth;
        this._rotateFrame = (expectPortrait !== nativePortrait);

        if (this._rotateFrame) {
            // After rotation: swap width and height
            this.width = this._nativeHeight;
            this.height = this._nativeWidth;
            console.warn(`[Camera] Orientation mismatch: native ${this._nativeWidth}x${this._nativeHeight} ` +
                `(${nativePortrait ? 'portrait' : 'landscape'}) but expected ` +
                `${expectPortrait ? 'portrait' : 'landscape'} → rotating 90° CW`);
        } else {
            this.width = this._nativeWidth;
            this.height = this._nativeHeight;
        }

        console.log(`[Camera] Native: ${this._nativeWidth}x${this._nativeHeight}, ` +
            `Output: ${this.width}x${this.height}, Rotated: ${this._rotateFrame}`);

        // Create offscreen canvas matching output dimensions
        this.canvas = document.createElement('canvas');
        this.canvas.width = this.width;
        this.canvas.height = this.height;
        this.ctx = this.canvas.getContext('2d', { willReadFrequently: true });
        this.running = true;

        return { width: this.width, height: this.height, rotated: this._rotateFrame };
    }

    /**
     * Capture a grayscale frame.
     * If the video frame orientation doesn't match the screen, a 90° CW
     * canvas rotation is applied before extracting grayscale pixels.
     * @returns {Uint8Array|null} Grayscale pixel data (width * height bytes)
     */
    captureGrayscale() {
        if (!this.running || !this.video || this.video.readyState < 2) {
            return null;
        }

        if (this._rotateFrame) {
            // Apply 90° CW rotation via canvas transform:
            //   translate to top-right → rotate 90° CW → draw at native size
            // Result: native landscape → output portrait (or vice versa)
            this.ctx.save();
            this.ctx.translate(this.width, 0);
            this.ctx.rotate(Math.PI / 2);
            this.ctx.drawImage(this.video, 0, 0, this._nativeWidth, this._nativeHeight);
            this.ctx.restore();
        } else {
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
        return this._rotateFrame;
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
