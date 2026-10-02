/**
 * AprilTag Detector Web Worker
 *
 * This worker handles AprilTag detection in a separate thread, keeping the main UI responsive.
 * It loads the AprilTag WebAssembly module and handles all detector operations.
 */

// Import required scripts. importScripts() resolves relative to this
// worker's own URL, so this works no matter what path the page itself is
// served from (root, a GitHub Pages project path, a subfolder, etc.) -
// as long as these files sit next to this one.
importScripts('tag-families.js', 'apriltag_wasm.js', 'apriltag-wasm-wrapper.js');

// Global detector instance
let detector = new AprilTagDetector();
let initialized = false;
let processingInProgress = false;

// Initialize the detector
async function initializeDetector() {
    try {
        await detector.initialize();
        initialized = true;
        postMessage({ type: 'initialized' });
    } catch (error) {
        postMessage({
            type: 'error',
            message: 'Failed to initialize detector',
            error: error.toString()
        });
    }
}
// Start initialization immediately when the worker is created
initializeDetector();

// Handle messages from main thread
onmessage = function(e) {
    const message = e.data;

    switch (message.type) {
        case 'setup':
            setupDetector(message.family, message.hammingDist, message.params);
            break;

        case 'detect':
            if (!initialized) {
                postMessage({
                    type: 'error',
                    message: 'Detector not initialized yet'
                });
                return;
            }

            if (processingInProgress) {
                postMessage({
                    type: 'busy',
                    message: 'Detection already in progress'
                });
                return;
            }

            detectTags(message.imageData, message.width, message.height);
            break;

        case 'addFamily':
            addTagFamily(message.family, message.hammingDist);
            break;

        case 'setParameters':
            setDetectorParams(message.params);
            break;

        case 'cleanup':
            cleanup();
            break;
    }
};

// Setup detector with specified family
function setupDetector(family, hammingDist, params) {
    if (!initialized) {
        postMessage({
            type: 'error',
            message: 'Cannot setup: detector not initialized'
        });
        return;
    }

    try {
        const result = detector.setupDetector(family || 'tag36h11', hammingDist || 0);

        if (result === 0) {
            if (params) {
                detector.setParameters(params);
            }
            postMessage({ type: 'setup-complete', family });
        } else {
            postMessage({
                type: 'error',
                message: `Failed to setup detector with family ${family}`,
                code: result
            });
        }
    } catch (error) {
        postMessage({
            type: 'error',
            message: 'Error setting up detector',
            error: error.toString()
        });
    }
}

// Add a tag family to the detector
function addTagFamily(family, hammingDist) {
    if (!initialized) {
        postMessage({
            type: 'error',
            message: 'Cannot add family: detector not initialized'
        });
        return;
    }

    try {
        const result = detector.addFamily(family, hammingDist || 0);

        if (result === 0) {
            postMessage({ type: 'family-added', family });
        } else {
            postMessage({
                type: 'error',
                message: `Failed to add family ${family}`,
                code: result
            });
        }
    } catch (error) {
        postMessage({
            type: 'error',
            message: 'Error adding tag family',
            error: error.toString()
        });
    }
}

// Set detector parameters
function setDetectorParams(params) {
    if (!initialized) {
        postMessage({
            type: 'error',
            message: 'Cannot set parameters: detector not initialized'
        });
        return;
    }

    try {
        detector.setParameters(params);
        postMessage({ type: 'parameters-set' });
    } catch (error) {
        postMessage({
            type: 'error',
            message: 'Error setting parameters',
            error: error.toString()
        });
    }
}

// Detect tags in the provided image
function detectTags(imageDataBuffer, width, height) {
    processingInProgress = true;

    try {
        // Create start time for performance measurement
        const startTime = performance.now();

        // Create a view over the transferred buffer
        const imageData = new Uint8Array(imageDataBuffer);

        // Convert to grayscale if needed (detect based on array length)
        let grayscaleData;
        if (imageData.length === width * height * 4) {
            // Input is RGBA, convert to grayscale
            grayscaleData = AprilTagDetector.convertToGrayscale(imageData, width, height);
        } else {
            // Input is already grayscale
            grayscaleData = imageData;
        }

        // Detect tags
        const detections = detector.detect(grayscaleData, width, height);

        // Calculate processing time
        const processingTime = performance.now() - startTime;

        // Send result back to main thread
        postMessage({
            type: 'detection-result',
            detections: detections,
            processingTime: processingTime
        });
    } catch (error) {
        postMessage({
            type: 'error',
            message: 'Error during detection',
            error: error.toString()
        });
    } finally {
        processingInProgress = false;
    }
}

// Clean up resources
function cleanup() {
    if (initialized) {
        detector.cleanup();
        initialized = false;
        postMessage({ type: 'cleanup-complete' });
    }
}
