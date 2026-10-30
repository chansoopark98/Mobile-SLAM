/**
 * SharedMemory - Allocates and manages memory on the WASM heap.
 * Provides typed array views into WASM linear memory for zero-copy data transfer.
 */
export class SharedMemory {
    /**
     * @param {Object} wasm - The WASM module instance
     * @param {TypedArray} heapType - e.g. wasm.HEAPU8, wasm.HEAPF64
     * @param {number} count - Number of elements (not bytes)
     */
    constructor(wasm, heapType, count) {
        this.wasm = wasm;
        this.count = count;

        // Determine bytes per element from heap type
        if (heapType === wasm.HEAPU8 || heapType === wasm.HEAP8) {
            this.bytesPerElement = 1;
            this.TypedArrayClass = Uint8Array;
        } else if (heapType === wasm.HEAPF32) {
            this.bytesPerElement = 4;
            this.TypedArrayClass = Float32Array;
        } else if (heapType === wasm.HEAPF64) {
            this.bytesPerElement = 8;
            this.TypedArrayClass = Float64Array;
        } else if (heapType === wasm.HEAP32 || heapType === wasm.HEAPU32) {
            this.bytesPerElement = 4;
            this.TypedArrayClass = Int32Array;
        } else {
            this.bytesPerElement = 8;
            this.TypedArrayClass = Float64Array;
        }

        const numBytes = count * this.bytesPerElement;
        this.ptr = wasm._malloc(numBytes);
        if (!this.ptr) {
            throw new Error(`SharedMemory: failed to allocate ${numBytes} bytes`);
        }

        this._updateView();
    }

    /** Refresh typed array view (needed after memory growth) */
    _updateView() {
        this.view = new this.TypedArrayClass(
            this.wasm.HEAPU8.buffer,
            this.ptr,
            this.count
        );
    }

    /** Write data into WASM heap */
    write(data) {
        // Check if buffer was detached (memory growth)
        if (this.view.buffer !== this.wasm.HEAPU8.buffer) {
            this._updateView();
        }
        this.view.set(data);
    }

    /** Read data from WASM heap */
    read(length) {
        if (this.view.buffer !== this.wasm.HEAPU8.buffer) {
            this._updateView();
        }
        if (length !== undefined && length < this.count) {
            return this.view.subarray(0, length);
        }
        return this.view;
    }

    /** Get byte offset (pointer) for passing to C++ */
    get byteOffset() {
        return this.ptr;
    }

    /** Free WASM heap memory */
    dispose() {
        if (this.ptr) {
            this.wasm._free(this.ptr);
            this.ptr = 0;
            this.view = null;
        }
    }
}
