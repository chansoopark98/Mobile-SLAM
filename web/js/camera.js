/**
 * Camera capture module using navigator.mediaDevices.getUserMedia.
 * Captures frames from the environment-facing camera and extracts grayscale data.
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
    }

    /**
     * Initialize camera capture.
     * @param {number} targetWidth - Desired capture width
     * @param {number} targetHeight - Desired capture height
     * @param {HTMLVideoElement} [videoElement] - Optional existing video element
     * @returns {Promise<{width: number, height: number}>} Actual capture dimensions
     */
    async initialize(targetWidth = 640, targetHeight = 480, videoElement = null) {
        const constraints = {
            video: {
                facingMode: 'environment',
                width: { ideal: targetWidth },
                height: { ideal: targetHeight },
            },
            audio: false,
        };

        this.stream = await navigator.mediaDevices.getUserMedia(constraints);
        this.video = videoElement || document.createElement('video');
        this.video.setAttribute('playsinline', '');
        this.video.setAttribute('autoplay', '');
        this.video.srcObject = this.stream;

        await new Promise((resolve) => {
            this.video.onloadedmetadata = () => {
                this.video.play();
                resolve();
            };
        });

        this.width = this.video.videoWidth;
        this.height = this.video.videoHeight;

        // Create offscreen canvas for frame extraction
        this.canvas = document.createElement('canvas');
        this.canvas.width = this.width;
        this.canvas.height = this.height;
        this.ctx = this.canvas.getContext('2d', { willReadFrequently: true });
        this.running = true;

        return { width: this.width, height: this.height };
    }

    /**
     * Capture a grayscale frame.
     * @returns {Uint8Array|null} Grayscale pixel data (width * height bytes)
     */
    captureGrayscale() {
        if (!this.running || !this.video || this.video.readyState < 2) {
            return null;
        }

        this.ctx.drawImage(this.video, 0, 0, this.width, this.height);
        const imageData = this.ctx.getImageData(0, 0, this.width, this.height);
        const rgba = imageData.data;
        const gray = new Uint8Array(this.width * this.height);

        // RGBA to grayscale: Y = 0.299R + 0.587G + 0.114B
        for (let i = 0, j = 0; i < rgba.length; i += 4, j++) {
            gray[j] = (rgba[i] * 77 + rgba[i + 1] * 150 + rgba[i + 2] * 29) >> 8;
        }

        return gray;
    }

    /** Get the video element for display */
    getVideoElement() {
        return this.video;
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
