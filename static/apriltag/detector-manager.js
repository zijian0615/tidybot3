/**
 * AprilTagDetectorManager
 *
 * This class manages the communication with the detector worker, providing
 * a simple API that mimics the original AprilTagDetector class, but with
 * all processing offloaded to a Web Worker thread.
 */

// Resolve this script's own directory so the worker (and, transitively, the
// wasm module) can be located regardless of what path this page is served
// from - e.g. plain `/`, a GitHub Pages project path like `/apriltag-js/`,
// or a subfolder of a larger site.
const _SCRIPT_URL = document.currentScript && document.currentScript.src;
const _JS_BASE = _SCRIPT_URL
    ? _SCRIPT_URL.slice(0, _SCRIPT_URL.lastIndexOf('/') + 1)
    : 'js/';

class AprilTagDetectorManager {
    constructor() {
        this.worker = null;
        this.initialized = false;
        this.initializing = false;
        this.pendingRequests = [];
        this.callbacks = {
            onInitialized: null,
            onDetectionResult: null,
            onError: null
        };

        // Performance tracking
        this.lastDetectionTime = 0;
        this.processingTime = 0;
        this.workerBusy = false;
    }

    /**
     * Initialize the detector worker
     * @param {Object} callbacks - Callback functions
     * @returns {Promise} - Resolves when initialization is complete
     */
    async initialize(callbacks = {}) {
        if (this.initialized) {
            return Promise.resolve();
        }

        if (this.initializing) {
            return new Promise((resolve, reject) => {
                this.pendingRequests.push({ type: 'initialize', resolve, reject });
            });
        }

        this.initializing = true;

        // Set up callbacks
        if (callbacks.onInitialized) this.callbacks.onInitialized = callbacks.onInitialized;
        if (callbacks.onDetectionResult) this.callbacks.onDetectionResult = callbacks.onDetectionResult;
        if (callbacks.onError) this.callbacks.onError = callbacks.onError;

        return new Promise((resolve, reject) => {
            try {
                console.log("Creating worker...");

                // Resolve the worker script relative to this file's own URL,
                // so it works no matter what path the page is served from.
                this.worker = new Worker(_JS_BASE + 'detector-worker.js');
                console.log("Worker created successfully");

                // Set up message handler with logging
                this.worker.onmessage = (event) => {
                    console.log("Worker message received:", event.data.type);
                    this._handleWorkerMessage(event);
                };

                // Set up error handler with better error reporting
                this.worker.onerror = (error) => {
                    this.initializing = false;
                    const errorMsg = `Worker error: ${error.message || 'unknown error'} at ${error.filename}:${error.lineno}`;
                    console.error(errorMsg, error);
                    if (this.callbacks.onError) {
                        this.callbacks.onError(errorMsg);
                    }
                    reject(new Error(errorMsg));
                };

                // Store this promise's resolve/reject functions to call when initialization is complete
                this.pendingRequests.push({ type: 'initialize', resolve, reject });
            } catch (error) {
                this.initializing = false;
                console.error('Failed to initialize detector worker:', error);
                reject(error);
            }
        });
    }

    /**
     * Handle messages from the worker
     * @private
     * @param {MessageEvent} event - Message event from worker
     */
    _handleWorkerMessage(event) {
        const message = event.data;

        switch (message.type) {
            case 'initialized':
                this._handleInitialized();
                break;

            case 'setup-complete':
                this._resolveRequest('setup', message);
                break;

            case 'family-added':
                this._resolveRequest('addFamily', message);
                break;

            case 'parameters-set':
                this._resolveRequest('setParameters', message);
                break;

            case 'detection-result':
                this.workerBusy = false;
                this.processingTime = message.processingTime;
                this._resolveRequest('detect', message);
                // Also call the detection callback
                if (this.callbacks.onDetectionResult) {
                    this.callbacks.onDetectionResult(message.detections, message.processingTime);
                }
                break;

            case 'cleanup-complete':
                this._resolveRequest('cleanup', message);
                break;

            case 'busy':
                // Worker is busy, just log it
                console.log('Detector worker is busy, skipping this detection');
                this.workerBusy = true;
                this._resolveRequest('detect', { detections: [] });
                break;

            case 'error':
                console.error(`Worker error: ${message.message}`, message.error);
                if (this.callbacks.onError) {
                    this.callbacks.onError(message.message);
                }
                // Reject any pending request for the same command
                this._rejectRequest(null, new Error(message.message));
                break;

            default:
                console.warn('Unknown message from worker:', message);
                break;
        }
    }

    /**
     * Handle successful initialization
     * @private
     */
    _handleInitialized() {
        this.initialized = true;
        this.initializing = false;

        console.log('AprilTag detector worker initialized');

        // Call the initialization callback
        if (this.callbacks.onInitialized) {
            this.callbacks.onInitialized();
        }

        // Resolve any pending initialize requests
        this._resolveRequest('initialize');
    }

    /**
     * Resolve a pending request
     * @private
     * @param {string} type - Request type
     * @param {Object} data - Data to pass to resolve function
     */
    _resolveRequest(type, data) {
        const index = this.pendingRequests.findIndex(req => req.type === type);
        if (index !== -1) {
            const request = this.pendingRequests.splice(index, 1)[0];
            request.resolve(data);
        }
    }

    /**
     * Reject a pending request
     * @private
     * @param {string} type - Request type (or null for all types)
     * @param {Error} error - Error to pass to reject function
     */
    _rejectRequest(type, error) {
        const requests = type
            ? this.pendingRequests.filter(req => req.type === type)
            : this.pendingRequests;

        for (const request of requests) {
            const index = this.pendingRequests.indexOf(request);
            if (index !== -1) {
                this.pendingRequests.splice(index, 1);
                request.reject(error);
            }
        }
    }

    /**
     * Set up the detector with a specific tag family
     * @param {string} family - Tag family to use
     * @param {number} hammingDist - Hamming distance for this family
     * @param {Object} params - Detection parameters
     * @returns {Promise} - Resolves when setup is complete
     */
    setupDetector(family = 'tag36h11', hammingDist = 0, params = null) {
        if (!this.initialized) {
            return Promise.reject(new Error('Detector not initialized. Call initialize() first.'));
        }

        return new Promise((resolve, reject) => {
            this.pendingRequests.push({ type: 'setup', resolve, reject });

            this.worker.postMessage({
                type: 'setup',
                family,
                hammingDist,
                params
            });
        });
    }

    /**
     * Add a tag family to the detector
     * @param {string} family - Tag family to add
     * @param {number} hammingDist - Hamming distance
     * @returns {Promise} - Resolves when family is added
     */
    addFamily(family, hammingDist = 0) {
        if (!this.initialized) {
            return Promise.reject(new Error('Detector not initialized. Call initialize() first.'));
        }

        return new Promise((resolve, reject) => {
            this.pendingRequests.push({ type: 'addFamily', resolve, reject });

            this.worker.postMessage({
                type: 'addFamily',
                family,
                hammingDist
            });
        });
    }

    /**
     * Set detector parameters
     * @param {Object} params - Detection parameters
     * @returns {Promise} - Resolves when parameters are set
     */
    setParameters(params = {}) {
        if (!this.initialized) {
            return Promise.reject(new Error('Detector not initialized. Call initialize() first.'));
        }

        return new Promise((resolve, reject) => {
            this.pendingRequests.push({ type: 'setParameters', resolve, reject });

            this.worker.postMessage({
                type: 'setParameters',
                params
            });
        });
    }

    /**
     * Detect AprilTags in an image
     * @param {Uint8Array|ImageData} imageData - Image data (RGBA or grayscale)
     * @param {number} width - Image width
     * @param {number} height - Image height
     * @returns {Promise} - Resolves with an array of detections
     */
    detect(imageData, width, height) {
        // Defensive check to prevent race conditions, especially on mobile
        if (!this.initialized || !this.worker) {
            console.warn('Detect called when detector is not fully initialized');
            return Promise.resolve([]); // Return empty result instead of rejecting
        }

        if (this.workerBusy) {
            // If worker is busy, return empty array immediately
            return Promise.resolve([]);
        }

        this.workerBusy = true;
        this.lastDetectionTime = performance.now();

        return new Promise((resolve, reject) => {
            try {
                // Additional defensive check before proceeding
                if (!this.initialized || !this.worker) {
                    this.workerBusy = false;
                    resolve([]);
                    return;
                }

                this.pendingRequests.push({ type: 'detect', resolve, reject });

                // Get the buffer from imageData (handle different input types)
                let buffer;
                if (imageData instanceof ImageData) {
                    buffer = imageData.data.buffer;
                } else if (imageData instanceof Uint8Array || imageData instanceof Uint8ClampedArray) {
                    buffer = imageData.buffer;
                } else {
                    this.workerBusy = false;
                    reject(new Error('Invalid imageData type. Expected ImageData, Uint8Array, or Uint8ClampedArray.'));
                    return;
                }

                // Create a copy of the buffer so we can transfer it
                // (this is important so we don't detach the buffer from the original imageData)
                const bufferCopy = buffer.slice(0);

                // Send the message, transferring the buffer copy to avoid copying
                this.worker.postMessage({
                    type: 'detect',
                    imageData: bufferCopy,
                    width,
                    height
                }, [bufferCopy]);
            } catch (error) {
                // Handle any unexpected errors
                console.error("Unexpected error in detect:", error);
                this.workerBusy = false;
                resolve([]); // Resolve with empty array instead of rejecting
            }
        });
    }

    /**
     * Clean up resources
     * @returns {Promise} - Resolves when cleanup is complete
     */
    cleanup() {
        if (!this.initialized) {
            return Promise.resolve();
        }

        return new Promise((resolve, reject) => {
            this.pendingRequests.push({ type: 'cleanup', resolve, reject });

            this.worker.postMessage({ type: 'cleanup' });

            // Reset state
            this.initialized = false;
            this.initializing = false;
            this.workerBusy = false;
        });
    }

    /**
     * Clean up detector but keep worker running
     * Used for reconfiguring detector without restarting the worker
     * @returns {Promise} - Resolves when cleanup is complete
     */
    cleanupDetector() {
        if (!this.initialized) {
            return Promise.resolve();
        }

        return new Promise((resolve, reject) => {
            const originalResolve = resolve;

            // Create a wrapper resolve function that sets the initialized flag to false
            // This matches the worker state after cleanup
            const wrappedResolve = (data) => {
                // After cleanup, the detector is no longer initialized in the worker
                // So we need to set our state to match
                this.initialized = false;
                this.workerBusy = false;

                // Now resolve the original promise
                originalResolve(data);
            };

            this.pendingRequests.push({ type: 'cleanup', resolve: wrappedResolve, reject });

            this.worker.postMessage({ type: 'cleanup' });
        });
    }

    /**
     * Get detector status information
     * @returns {Object} - Status information
     */
    getStatus() {
        return {
            initialized: this.initialized,
            initializing: this.initializing,
            busy: this.workerBusy,
            lastDetectionTime: this.lastDetectionTime,
            processingTime: this.processingTime
        };
    }

    /**
     * Terminate the worker
     * Call this when completely done with the detector
     */
    terminate() {
        if (this.worker) {
            this.worker.terminate();
            this.worker = null;
        }

        this.initialized = false;
        this.initializing = false;
        this.workerBusy = false;
        this.pendingRequests = [];
    }
}
