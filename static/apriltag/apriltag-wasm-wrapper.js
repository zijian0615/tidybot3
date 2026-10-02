/**
 * AprilTag WebAssembly Wrapper
 *
 * This wrapper makes it easier to use the compiled apriltag_wasm.js module
 * by providing a simple JavaScript interface to the WebAssembly functions.
 *
 * Family name/index lookups come from js/tag-families.js (loaded before this
 * file - see detector-worker.js), so the family table exists in exactly one
 * place instead of being duplicated here.
 */
class AprilTagDetector {
    constructor() {
        this.module = null;
        this.initialized = false;
        this.detecting = false;

        // Function pointers once initialized
        this._initialize_detector = null;
        this._set_detector_parameters = null;
        this._detect_tags = null;
        this._cleanup = null;
        this._malloc = null;
        this._free = null;
        this._add_family = null;

        // Detection buffers
        this._detectionBuffer = null;
        this._imageBuffer = null;
        this._maxDetections = 100; // Maximum number of detections to process
    }

    /**
     * Initialize the detector with the AprilTag WebAssembly module
     * @returns {Promise} - Resolves when initialization is complete
     */
    async initialize() {
        if (this.initialized) {
            console.log("Detector already initialized");
            return Promise.resolve();
        }

        try {
            // Load the AprilTag WebAssembly module
            this.module = await AprilTagWasm();

            // Get function pointers
            this._initialize_detector = this.module.cwrap('initialize_detector', 'number', ['string', 'number']);
            this._set_detector_parameters = this.module.cwrap('set_detector_parameters', 'void',
                ['number', 'number', 'number', 'number', 'number']);
            this._detect_tags = this.module.cwrap('detect_tags', 'number',
                ['number', 'number', 'number', 'number', 'number']);
            this._cleanup = this.module.cwrap('cleanup', 'void', []);
            this._malloc = this.module._malloc;
            this._free = this.module._free;
            this._add_family = this.module.cwrap('add_family', 'number', ['number', 'number']);

            console.log("AprilTag WebAssembly module loaded");
            this.initialized = true;

            // Create a detection buffer for results
            const detectionStructSize = 60; // size of detection_info_t in bytes
            this._detectionBuffer = this._malloc(detectionStructSize * this._maxDetections);

            return Promise.resolve();
        } catch (error) {
            console.error("Failed to initialize AprilTag detector:", error);
            return Promise.reject(error);
        }
    }

    /**
     * Set up the detector with a specific tag family
     * @param {string} family - Tag family to use (e.g., "tag36h11", "tagAll")
     * @param {number} hammingDist - Hamming distance threshold for this family
     * @returns {number} - Result code (0 = success)
     */
    setupDetector(family = "tag36h11", hammingDist = 0) {
        if (!this.initialized) {
            throw new Error("Detector not initialized. Call initialize() first.");
        }

        // Some families require a minimum hamming distance to keep the
        // false-positive rate acceptable - see js/tag-families.js. addFamily()
        // enforces this too, but initialize_detector() adds the first family
        // directly (not via addFamily()), so it needs the same clamp here.
        const minHamming = tagFamilyMinHamming(family);
        let effectiveHammingDist = hammingDist;
        if (effectiveHammingDist < minHamming) {
            effectiveHammingDist = minHamming;
        }

        const result = this._initialize_detector(family, effectiveHammingDist);

        if (result !== 0) {
            console.error(`Failed to initialize detector with family ${family}, error code: ${result}`);
        } else {
            console.log(`Detector initialized with family ${family} at hamming ${effectiveHammingDist}`);
        }

        return result;
    }

    /**
     * Add an additional tag family to the detector
     * @param {string} family - Tag family to add (e.g., "tag36h11")
     * @param {number} hammingDist - Hamming distance threshold (0-3)
     * @returns {number} - Result code (0 = success)
     */
    addFamily(family, hammingDist = 0) {
        if (!this.initialized) {
            throw new Error("Detector not initialized. Call initialize() first.");
        }

        // Get family index
        const familyIndex = tagFamilyIndex(family);
        if (familyIndex < 0) {
            console.error(`Unknown tag family: ${family}`);
            return -1;
        }

        // Some families require a minimum hamming distance to keep the
        // false-positive rate acceptable - see js/tag-families.js.
        const minHamming = tagFamilyMinHamming(family);
        let effectiveHammingDist = hammingDist;
        if (effectiveHammingDist < minHamming) {
            console.log(`Setting minimum hamming distance of ${minHamming} for ${family}`);
            effectiveHammingDist = minHamming;
        }

        // Add the family with specified hamming distance
        const result = this._add_family(familyIndex, effectiveHammingDist);

        if (result !== 0) {
            console.error(`Failed to add family ${family} with hamming ${effectiveHammingDist}, error: ${result}`);
        } else {
            console.log(`Added family ${family} with hamming distance ${effectiveHammingDist}`);
        }

        return result;
    }

    /**
     * Set detector parameters
     * @param {Object} params - Detection parameters
     * @param {number} params.quadDecimate - How much to decimate input image (1.0 = full resolution)
     * @param {number} params.quadSigma - Gaussian blur for quad detection
     * @param {boolean} params.refineEdges - Whether to refine edges
     * @param {number} params.decodeSharpening - How much to sharpen decoded images
     * @param {boolean} params.debug - Enable debug mode
     */
    setParameters(params = {}) {
        if (!this.initialized) {
            throw new Error("Detector not initialized. Call initialize() first.");
        }

        const {
            quadDecimate = 2.0,
            quadSigma = 0.0,
            refineEdges = true,
            decodeSharpening = 0.25,
            debug = false
        } = params;

        this._set_detector_parameters(
            quadDecimate,
            quadSigma,
            refineEdges ? 1 : 0,
            decodeSharpening,
            debug ? 1 : 0
        );

        console.log("Set detector parameters:", params);
    }

    /**
     * Detect AprilTags in an image
     * @param {Uint8Array} imageData - Grayscale image data (1 byte per pixel)
     * @param {number} width - Image width
     * @param {number} height - Image height
     * @returns {Array} - Array of detection objects
     */
    detect(imageData, width, height) {
        if (!this.initialized) {
            throw new Error("Detector not initialized. Call initialize() first.");
        }

        if (this.detecting) {
            console.warn("Detection already in progress. Ignoring request.");
            return [];
        }

        this.detecting = true;

        try {
            // Allocate memory for the image
            const imageSize = width * height;
            if (!this._imageBuffer || this._imageBufferSize < imageSize) {
                if (this._imageBuffer) {
                    this._free(this._imageBuffer);
                }
                this._imageBuffer = this._malloc(imageSize);
                this._imageBufferSize = imageSize;
            }

            // Copy image data to WebAssembly memory
            this.module.HEAPU8.set(imageData, this._imageBuffer);

            // Perform detection
            const numDetections = this._detect_tags(
                this._imageBuffer,
                width,
                height,
                this._detectionBuffer,
                this._maxDetections
            );

            if (numDetections < 0) {
                console.error("Detection failed with error code:", numDetections);
                return [];
            }

            if (numDetections > 0) {
                console.log(`Detected ${numDetections} tags`);
            }

            // Read detection results
            const detections = [];
            // Structure layout in memory:
            // - id: int (4 bytes)
            // - family_id: int (4 bytes)
            // - center_x, center_y: float (4 bytes each)
            // - corners[4][2]: float (4 bytes each, 32 bytes total)
            // - decision_margin: float (4 bytes)
            // - hamming: float (4 bytes)
            // - rotation: float (4 bytes)
            // Total: 60 bytes
            const detectionStructSize = 60;

            for (let i = 0; i < numDetections; i++) {
                const baseOffset = this._detectionBuffer + (i * detectionStructSize);

                // Read detection info from memory
                const id = this.module.getValue(baseOffset, 'i32');
                const familyId = this.module.getValue(baseOffset + 4, 'i32');
                const centerX = this.module.getValue(baseOffset + 8, 'float');
                const centerY = this.module.getValue(baseOffset + 12, 'float');

                // Validate data before continuing
                if (isNaN(centerX) || isNaN(centerY) ||
                    centerX < 0 || centerX > width * 2 ||
                    centerY < 0 || centerY > height * 2) {
                    console.warn(`Skipping invalid detection ${i} with center (${centerX}, ${centerY})`);
                    continue;
                }

                // Read corners (4 corners, 2 coordinates each)
                const corners = [];
                let cornersOffset = baseOffset + 16;
                let validCorners = true;

                for (let j = 0; j < 4; j++) {
                    const x = this.module.getValue(cornersOffset, 'float');
                    const y = this.module.getValue(cornersOffset + 4, 'float');

                    // Validate corner coordinates
                    if (isNaN(x) || isNaN(y) ||
                        x < 0 || x > width * 2 ||
                        y < 0 || y > height * 2) {
                        console.warn(`Skipping invalid corner ${j} (${x}, ${y}) in detection ${i}`);
                        validCorners = false;
                        break;
                    }

                    corners.push({x, y});
                    cornersOffset += 8;
                }

                if (!validCorners) continue;

                // Read the remaining values with correct offsets
                const decisionMargin = this.module.getValue(baseOffset + 48, 'float');
                const hamming = this.module.getValue(baseOffset + 52, 'float');
                const rotation = this.module.getValue(baseOffset + 56, 'float');

                detections.push({
                    id,
                    family: tagFamilyName(familyId),
                    familyId,
                    center: {x: centerX, y: centerY},
                    corners,
                    decisionMargin,
                    hamming,
                    rotation
                });
            }

            return detections;
        } finally {
            this.detecting = false;
        }
    }

    /**
     * Clean up resources when done
     */
    cleanup() {
        if (!this.initialized) {
            return;
        }

        // Free allocated memory
        if (this._imageBuffer) {
            this._free(this._imageBuffer);
            this._imageBuffer = null;
        }

        if (this._detectionBuffer) {
            this._free(this._detectionBuffer);
            this._detectionBuffer = null;
        }

        // Clean up detector
        this._cleanup();
        this.initialized = false;

        console.log("AprilTag detector cleaned up");
    }

    /**
     * Convert an RGBA image to grayscale
     * @param {Uint8ClampedArray} rgba - RGBA image data
     * @param {number} width - Image width
     * @param {number} height - Image height
     * @returns {Uint8Array} - Grayscale image data
     */
    static convertToGrayscale(rgba, width, height) {
        const grayscale = new Uint8Array(width * height);

        // Calculate min and max values for dynamic range adjustment
        let min = 255;
        let max = 0;

        // First pass: find min/max and convert to grayscale
        for (let i = 0, j = 0; i < width * height; i++, j += 4) {
            // Use proper luminance weights (BT.709)
            const value = Math.round(rgba[j] * 0.2126 + rgba[j + 1] * 0.7152 + rgba[j + 2] * 0.0722);
            grayscale[i] = value;

            if (value < min) min = value;
            if (value > max) max = value;
        }

        // Apply contrast enhancement if needed
        if (max - min < 100) { // If contrast is low, enhance it
            const range = max - min;
            const scale = range > 0 ? 255 / range : 1;

            // Second pass: apply contrast enhancement
            for (let i = 0; i < width * height; i++) {
                grayscale[i] = Math.min(255, Math.max(0, Math.round((grayscale[i] - min) * scale)));
            }
        }

        return grayscale;
    }
}
