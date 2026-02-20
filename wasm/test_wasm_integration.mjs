/**
 * WASM integration test - simulates multi-frame VIO processing.
 * Tests: module load, configure, multi-frame processFrame, getMapPoints, reset.
 * Run with: node test_wasm_integration.mjs
 */

let passed = 0;
let failed = 0;

function assert(condition, msg) {
    if (condition) {
        console.log(`  PASS: ${msg}`);
        passed++;
    } else {
        console.log(`  FAIL: ${msg}`);
        failed++;
    }
}

async function main() {
    console.log('=== WASM Integration Test ===\n');

    // 1. Load module
    console.log('[1] Module Loading');
    const modulePath = new URL('./build/vio_engine.js', import.meta.url).href;
    const VIOWasm = (await import(modulePath)).default;
    assert(typeof VIOWasm === 'function', 'VIOWasm is a function');

    const wasm = await VIOWasm();
    assert(!!wasm, 'WASM instance created');
    assert(typeof wasm._malloc === 'function', '_malloc available');
    assert(typeof wasm._free === 'function', '_free available');
    assert(!!wasm.HEAPU8, 'HEAPU8 available');
    assert(!!wasm.HEAPF64, 'HEAPF64 available');

    // 2. Create engine
    console.log('\n[2] Engine Creation');
    const engine = new wasm.VIOEngine();
    assert(!!engine, 'VIOEngine created');
    assert(engine.isInitialized() === false, 'Not initialized initially');
    assert(engine.getFeaturePointCount() === 0, 'Zero features initially');

    // 3. Configure with typical mobile camera params
    console.log('\n[3] Configuration');
    const WIDTH = 320;
    const HEIGHT = 240;

    const r_ic_ptr = wasm._malloc(9 * 8);
    const t_ic_ptr = wasm._malloc(3 * 8);
    new Float64Array(wasm.HEAPF64.buffer, r_ic_ptr, 9).set([1, 0, 0, 0, 1, 0, 0, 0, 1]);
    new Float64Array(wasm.HEAPF64.buffer, t_ic_ptr, 3).set([0, 0, 0]);

    const configured = engine.configure(
        WIDTH, HEIGHT,
        260.0, 260.0,    // fx, fy
        160.0, 120.0,    // cx, cy
        0,               // PINHOLE
        0, 0, 0, 0,      // distortion
        r_ic_ptr, t_ic_ptr,
        0.08, 0.004,     // acc_n, acc_w
        0.004, 0.0002,   // gyr_n, gyr_w
        9.81             // g_norm
    );
    assert(configured === true, 'Configure succeeded');

    // 4. Multi-frame processing with synthetic data
    console.log('\n[4] Multi-frame Processing');
    const imgSize = WIDTH * HEIGHT;
    const img_ptr = wasm._malloc(imgSize);
    const pose_ptr = wasm._malloc(16 * 8);
    const imu_ptr = wasm._malloc(7 * 8 * 10); // 10 IMU readings max
    const mapPts_ptr = wasm._malloc(2000 * 3 * 8);

    const NUM_FRAMES = 20;
    let totalTime = 0;

    for (let frame = 0; frame < NUM_FRAMES; frame++) {
        // Generate synthetic image with some texture
        const imgData = new Uint8Array(wasm.HEAPU8.buffer, img_ptr, imgSize);
        for (let y = 0; y < HEIGHT; y++) {
            for (let x = 0; x < WIDTH; x++) {
                // Checkerboard + gradient pattern for feature detection
                const checker = ((Math.floor(x / 20) + Math.floor(y / 20)) % 2) * 200;
                const gradient = Math.floor((x + frame * 2) % 256);
                imgData[y * WIDTH + x] = Math.floor((checker + gradient) / 2);
            }
        }

        // Generate synthetic IMU readings (stationary with gravity)
        const numIMU = 5;
        const imuData = new Float64Array(wasm.HEAPF64.buffer, imu_ptr, numIMU * 7);
        for (let i = 0; i < numIMU; i++) {
            const t = frame * 0.033 + i * 0.007; // ~30fps, ~140Hz IMU
            imuData[i * 7 + 0] = t;        // timestamp
            imuData[i * 7 + 1] = 0.0;      // acc_x
            imuData[i * 7 + 2] = 0.0;      // acc_y
            imuData[i * 7 + 3] = 9.81;     // acc_z (gravity)
            imuData[i * 7 + 4] = 0.0;      // gyro_x
            imuData[i * 7 + 5] = 0.0;      // gyro_y
            imuData[i * 7 + 6] = 0.0;      // gyro_z
        }

        const t0 = performance.now();
        const hasPose = engine.processFrame(img_ptr, WIDTH, HEIGHT, imu_ptr, numIMU, pose_ptr);
        totalTime += performance.now() - t0;

        if (frame === 0) {
            assert(hasPose === false, `Frame ${frame}: no pose yet (expected)`);
        }
    }

    const avgMs = (totalTime / NUM_FRAMES).toFixed(1);
    console.log(`  INFO: ${NUM_FRAMES} frames processed, avg ${avgMs}ms/frame`);
    assert(totalTime > 0, `Processing took ${totalTime.toFixed(0)}ms total`);

    // 5. Test getMapPoints
    console.log('\n[5] Map Points');
    const numPts = engine.getMapPoints(mapPts_ptr, 2000);
    assert(typeof numPts === 'number', `getMapPoints returned ${numPts} points`);

    // 6. Test reset
    console.log('\n[6] Reset');
    engine.reset();
    assert(engine.isInitialized() === false, 'Not initialized after reset');

    // 7. Re-configure after reset
    console.log('\n[7] Re-configure After Reset');
    const reconfigured = engine.configure(
        WIDTH, HEIGHT,
        260.0, 260.0,
        160.0, 120.0,
        0, 0, 0, 0, 0,
        r_ic_ptr, t_ic_ptr,
        0.08, 0.004, 0.004, 0.0002, 9.81
    );
    assert(reconfigured === true, 'Re-configure after reset succeeded');

    // Cleanup
    wasm._free(r_ic_ptr);
    wasm._free(t_ic_ptr);
    wasm._free(img_ptr);
    wasm._free(pose_ptr);
    wasm._free(imu_ptr);
    wasm._free(mapPts_ptr);
    engine.delete();

    // Summary
    console.log(`\n=== Results: ${passed} passed, ${failed} failed ===`);
    if (failed > 0) process.exit(1);
}

main().catch(err => {
    console.error('Integration test error:', err);
    process.exit(1);
});
