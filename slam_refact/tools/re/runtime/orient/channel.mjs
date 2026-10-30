// Host side of the bit-exact channel (US-023/024 v2): the files run-orient.mjs writes for the func 647 probe and for
// reality.intrinsics contain only integers and hex strings of float32 bit patterns, so the sign of a zero (and a denormal)
// cannot be lost on the way (orient-us02x-gate.v1 review B-1). Shared by run-orient.mjs and channel_null_run.mjs.
export function intrinsicsFile(changes) {
  return JSON.stringify(changes.map(c => ({textureWidth: c.textureWidth, textureHeight: c.textureHeight, orientation: c.orientation, videoTime: c.videoTime, bits: c.bits})));
}
export function probeFile(probe647) {
  return JSON.stringify(probe647);
}
