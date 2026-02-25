/**
 * Screen orientation handler for mobile VIO coordinate system adaptation.
 *
 * Three coordinate frames are involved:
 *
 * 1. W3C Device Frame (DeviceMotion API, fixed to phone body):
 *      X_d = right edge,  Y_d = top edge,  Z_d = out of screen
 *      Phone upright portrait → acc_y ≈ +9.8 (gravity on Y)
 *
 * 2. VIO Body Frame (what the VIO engine receives as "IMU body frame"):
 *      X_v = right,  Y_v = forward (into scene),  Z_v = up
 *      IMU data is rotated from device frame → VIO body frame in app.js
 *      so gravity appears on Z_v, matching TUM VI dataset convention.
 *      Rotation: +90° around X → x_v=x_d, y_v=-z_d, z_v=y_d
 *
 * 3. Camera Frame (OpenCV, rotates with screen orientation):
 *      X_c = right in image,  Y_c = down in image,  Z_c = forward
 *
 * R_ic transforms camera frame → VIO body frame: v_vio = R_ic * v_cam
 * This is R_device_to_vio × R_camera_to_device for each orientation.
 */

/**
 * R_ic matrices for each screen orientation (row-major 3×3).
 *
 * Two-step derivation:
 *   Step 1 — Camera→Device (R_cd) per orientation:
 *     portrait:  X_c=X_d, Y_c=-Y_d, Z_c=-Z_d  →  R_cd = [1,0,0; 0,-1,0; 0,0,-1]
 *     landscape: X_c=Y_d, Y_c=X_d,  Z_c=-Z_d   →  R_cd = [0,1,0; 1,0,0; 0,0,-1]
 *     etc.
 *
 *   Step 2 — Device→VIO body (R_dv), same for all orientations:
 *     x_v = x_d, y_v = -z_d, z_v = y_d  →  R_dv = [1,0,0; 0,0,-1; 0,1,0]
 *
 *   R_ic = R_dv × R_cd
 *
 * @type {Object<string, number[]>}
 */
const R_IC = {
    // Portrait (0°): natural phone orientation
    //   Camera: X_c=right, Y_c=down, Z_c=forward
    //   VIO:    X_c→X_v,   Y_c→-Z_v,  Z_c→Y_v
    'portrait-primary': [
         1,  0,  0,
         0,  0,  1,
         0, -1,  0,
    ],

    // Landscape-primary (90°): phone rotated 90° CCW, right edge at top
    //   Camera: X_c→Z_v(up), Y_c→X_v(right), Z_c→Y_v(forward)
    'landscape-primary': [
         0,  1,  0,
         0,  0,  1,
         1,  0,  0,
    ],

    // Portrait upside-down (180°): phone flipped 180°
    //   Camera: X_c→-X_v(left), Y_c→Z_v(up), Z_c→Y_v(forward)
    'portrait-secondary': [
        -1,  0,  0,
         0,  0,  1,
         0,  1,  0,
    ],

    // Landscape-secondary (270°): phone rotated 90° CW, left edge at top
    //   Camera: X_c→-Z_v(down), Y_c→-X_v(left), Z_c→Y_v(forward)
    'landscape-secondary': [
         0, -1,  0,
         0,  0,  1,
        -1,  0,  0,
    ],
};

export class OrientationHandler {
    constructor() {
        /** @type {string} */
        this._currentType = 'portrait-primary';
        /** @type {function|null} */
        this._onChangeCallback = null;
        /** @type {function|null} */
        this._handler = null;
    }

    /**
     * Get current screen orientation type.
     * @returns {string} portrait-primary | landscape-primary |
     *                   portrait-secondary | landscape-secondary
     */
    getType() {
        if (screen.orientation && screen.orientation.type) {
            return screen.orientation.type;
        }
        // Fallback: infer from window dimensions
        return window.innerWidth > window.innerHeight
            ? 'landscape-primary'
            : 'portrait-primary';
    }

    /**
     * Check if current orientation is landscape.
     * @returns {boolean}
     */
    isLandscape() {
        return this.getType().startsWith('landscape');
    }

    /**
     * Get R_ic (camera-to-IMU rotation) for the current screen orientation.
     * @returns {number[]} 9-element row-major 3×3 rotation matrix
     */
    getRIC() {
        return R_IC[this.getType()] || R_IC['portrait-primary'];
    }

    /**
     * Get R_ic for a specific orientation type.
     * @param {string} type - Screen orientation type
     * @returns {number[]} 9-element row-major 3×3 rotation matrix
     */
    static getRICForType(type) {
        return R_IC[type] || R_IC['portrait-primary'];
    }

    /**
     * Try to lock screen orientation to portrait-primary.
     * Portrait is preferred for VIO: simpler extrinsics, consistent gravity axis.
     * @returns {Promise<boolean>} true if lock succeeded
     */
    async tryLockPortrait() {
        if (!screen.orientation || !screen.orientation.lock) {
            return false;
        }
        try {
            await screen.orientation.lock('portrait-primary');
            console.log('[Orientation] Locked to portrait-primary');
            return true;
        } catch (e) {
            console.warn('[Orientation] Could not lock portrait:', e.message);
            return false;
        }
    }

    /**
     * Start listening for orientation changes.
     * Fires callback when orientation type changes (not just angle).
     * @param {function} callback - Called with { type, r_ic, isLandscape }
     */
    startListening(callback) {
        this._onChangeCallback = callback;
        this._currentType = this.getType();

        if (screen.orientation) {
            this._handler = () => {
                const newType = screen.orientation.type;
                if (newType !== this._currentType) {
                    const oldType = this._currentType;
                    this._currentType = newType;
                    console.log(`[Orientation] Changed: ${oldType} → ${newType}`);
                    if (this._onChangeCallback) {
                        this._onChangeCallback({
                            type: newType,
                            r_ic: R_IC[newType] || R_IC['portrait-primary'],
                            isLandscape: newType.startsWith('landscape'),
                        });
                    }
                }
            };
            screen.orientation.addEventListener('change', this._handler);
        }
    }

    /** Stop listening for orientation changes. */
    stopListening() {
        if (this._handler && screen.orientation) {
            screen.orientation.removeEventListener('change', this._handler);
            this._handler = null;
        }
        this._onChangeCallback = null;
    }
}
