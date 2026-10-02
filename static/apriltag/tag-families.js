/**
 * Tag family table
 *
 * Single source of truth for the tag families this build exposes. The
 * `index` values are not arbitrary - they must match the tag_family_t enum
 * and tag_creators[] array in wasm-build/apriltag_wasm.c exactly, because
 * that index is what actually crosses the WebAssembly boundary in
 * add_family(family_idx, hamming) and comes back out of detect_tags() as
 * detection_info_t.family_id.
 *
 * minHamming reflects a rule enforced on both sides of the wasm boundary:
 * the C shim (add_family) and the JS wrapper both raise the requested
 * hamming distance up to this floor for tagCustom48h12, tagStandard52h13,
 * and tagCircle49h12 - their code spaces are dense enough that hamming 0
 * produces an unacceptable false-positive rate.
 *
 * tagCircle21h7 and tagCircle49h12 additionally pin maxHamming down to
 * their minHamming, locking the UI to a single fixed value each, for two
 * different reasons - see the comment on each entry below.
 *
 * Used by both the wrapper (js/apriltag-wasm-wrapper.js) and the
 * playground UI (js/playground.js) - previously this table was
 * duplicated as a name->index map and a separate index->name switch
 * inside the wrapper alone.
 */
const TAG_FAMILIES = [
    { index: 0, name: 'tag36h11',         label: '36h11',         minHamming: 0 },
    { index: 1, name: 'tag25h9',          label: '25h9',          minHamming: 0 },
    { index: 2, name: 'tag16h5',          label: '16h5',          minHamming: 0 },
    // maxHamming pins this at exactly hamming 0: its code space is small
    // and dense enough that hamming >= 1 widens the match tolerance far
    // enough to misdetect other active families' tags as corrupted
    // tagCircle21h7 codes (reproduced in testing). Hamming 0 (exact match
    // only) doesn't have that problem, and is cheap - the earlier wasm
    // build's bug that force-substituted a much larger hamming distance
    // for any hamming-0 request is what made this family unsafe to ship
    // before; see the README's "Known limitations".
    { index: 3, name: 'tagCircle21h7',    label: 'Circle21h7',    minHamming: 0, maxHamming: 0 },
    // maxHamming caps it at exactly minHamming: with 65535 codes at 49 bits,
    // even hamming 2 needs several GB for the decode table - genuinely too
    // large to offer as a choice, independent of any wasm bug.
    { index: 4, name: 'tagCircle49h12',   label: 'Circle49h12',   minHamming: 1, maxHamming: 1 },
    { index: 5, name: 'tagCustom48h12',   label: 'Custom48h12',   minHamming: 1 },
    { index: 6, name: 'tagStandard41h12', label: 'Standard41h12', minHamming: 0 },
    { index: 7, name: 'tagStandard52h13', label: 'Standard52h13', minHamming: 1 },
];

const TAG_FAMILY_BY_NAME = Object.fromEntries(TAG_FAMILIES.map(f => [f.name, f]));
const TAG_FAMILY_BY_INDEX = Object.fromEntries(TAG_FAMILIES.map(f => [f.index, f]));

function tagFamilyIndex(name) {
    const f = TAG_FAMILY_BY_NAME[name];
    return f ? f.index : -1;
}

function tagFamilyName(index) {
    const f = TAG_FAMILY_BY_INDEX[index];
    return f ? f.name : 'unknown';
}

function tagFamilyMinHamming(name) {
    const f = TAG_FAMILY_BY_NAME[name];
    return f ? f.minHamming : 0;
}
