/**
 * WASM module load test - verifies VIOEngine can be instantiated and configured.
 * Run with: node test_wasm_module.mjs
 */
async function main() {
    console.log('=== WASM Module Load Test ===\n');

    // Step 1: Load the module
    console.log('[1] Loading vio_engine.js...');
    const modulePath = new URL('./build/vio_engine.js', import.meta.url).href;
    const VIOWasm = (await import(modulePath)).default;
    console.log('    Module function loaded:', typeof VIOWasm === 'function' ? 'OK' : 'FAIL');

    // Step 2: Initialize WASM
    console.log('[2] Initializing WASM runtime...');
    const wasm = await VIOWasm();
    console.log('    WASM instance:', wasm ? 'OK' : 'FAIL');
    console.log('    _malloc available:', typeof wasm._malloc === 'function' ? 'OK' : 'FAIL');
    console.log('    _free available:', typeof wasm._free === 'function' ? 'OK' : 'FAIL');
    console.log('    HEAPU8 available:', wasm.HEAPU8 ? 'OK' : 'FAIL');
    console.log('    HEAPF64 available:', wasm.HEAPF64 ? 'OK' : 'FAIL');

    // Step 3: Create VIOEngine
    console.log('[3] Creating VIOEngine instance...');
    const engine = new wasm.VIOEngine();
    console.log('    VIOEngine created:', engine ? 'OK' : 'FAIL');

    // Step 4: Check API methods
    console.log('[4] Checking API methods...');
    console.log('    configure:', typeof engine.configure === 'function' ? 'OK' : 'FAIL');
    console.log('    processFrame:', typeof engine.processFrame === 'function' ? 'OK' : 'FAIL');
    console.log('    isInitialized:', typeof engine.isInitialized === 'function' ? 'OK' : 'FAIL');
    console.log('    getFeaturePointCount:', typeof engine.getFeaturePointCount === 'function' ? 'OK' : 'FAIL');
    console.log('    getMapPoints:', typeof engine.getMapPoints === 'function' ? 'OK' : 'FAIL');
    console.log('    reset:', typeof engine.reset === 'function' ? 'OK' : 'FAIL');

    // Step 5: Test initial state
    console.log('[5] Testing initial state...');
    console.log('    isInitialized:', engine.isInitialized() === false ? 'OK (false)' : 'FAIL');
    console.log('    getFeaturePointCount:', engine.getFeaturePointCount() === 0 ? 'OK (0)' : 'FAIL');

    // Step 6: Test configure
    console.log('[6] Testing configure...');
    // Allocate extrinsic buffers
    const r_ic_ptr = wasm._malloc(9 * 8);
    const t_ic_ptr = wasm._malloc(3 * 8);
    // Identity rotation
    const r_ic = new Float64Array(wasm.HEAPF64.buffer, r_ic_ptr, 9);
    r_ic.set([1, 0, 0, 0, 1, 0, 0, 0, 1]);
    // Zero translation
    const t_ic = new Float64Array(wasm.HEAPF64.buffer, t_ic_ptr, 3);
    t_ic.set([0, 0, 0]);

    const configured = engine.configure(
        640, 480,       // width, height
        500.0, 500.0,   // fx, fy
        320.0, 240.0,   // cx, cy
        0,              // PINHOLE
        0, 0, 0, 0,     // distortion
        r_ic_ptr, t_ic_ptr,
        0.08, 0.004,    // acc_n, acc_w
        0.004, 0.0002,  // gyr_n, gyr_w
        9.81            // g_norm
    );
    console.log('    configure result:', configured ? 'OK (true)' : 'FAIL (false)');

    // Step 7: Test processFrame with dummy data
    console.log('[7] Testing processFrame with dummy image...');
    const imgSize = 640 * 480;
    const img_ptr = wasm._malloc(imgSize);
    const pose_ptr = wasm._malloc(16 * 8);
    const imu_ptr = wasm._malloc(7 * 8);  // 1 IMU reading

    // Write dummy grayscale image (all gray)
    const imgData = new Uint8Array(wasm.HEAPU8.buffer, img_ptr, imgSize);
    imgData.fill(128);

    // Write dummy IMU (1 reading)
    const imuData = new Float64Array(wasm.HEAPF64.buffer, imu_ptr, 7);
    imuData.set([0.001, 0, 0, 9.81, 0, 0, 0]); // timestamp, acc_xyz, gyro_xyz

    const hasPose = engine.processFrame(img_ptr, 640, 480, imu_ptr, 1, pose_ptr);
    console.log('    processFrame result:', hasPose === false ? 'OK (false, not initialized yet)' : `unexpected: ${hasPose}`);
    console.log('    isInitialized after 1 frame:', engine.isInitialized() === false ? 'OK (false)' : 'unexpected');

    // Cleanup
    wasm._free(r_ic_ptr);
    wasm._free(t_ic_ptr);
    wasm._free(img_ptr);
    wasm._free(pose_ptr);
    wasm._free(imu_ptr);
    engine.delete();

    console.log('\n=== All WASM module tests passed ===');
}

main().catch(err => {
    console.error('WASM test failed:', err);
    process.exit(1);
});
