// B6 / RM-2: feed synthetic RGB pixels to the real sharp 0.34.5 .grayscale() and dump the 8-bit result,
// so the grayscale function can be identified without any resampling or JPEG in the way.
// Usage: node sharp_gray_probe.mjs <orig-cli-dir> <out-dir>
// Writes: pure_channels.gray.raw  (4 rows x 256: R ramp, G ramp, B ramp, gray ramp; other channels 0)
//         grid.rgb.raw / grid.gray.raw (17^3 grid of RGB triples, levels 0,16,..,255)
import fs from 'fs'
import path from 'path'
import {createRequire} from 'module'

const [origDir, outDir] = process.argv.slice(2)
const require = createRequire(path.join(origDir, 'x.js'))
const sharp = require('sharp')
fs.mkdirSync(outDir, {recursive: true})

const w = 256, h = 4
const buf = Buffer.alloc(w * h * 3)
for (let x = 0; x < w; x++) {
  buf[(0 * w + x) * 3 + 0] = x
  buf[(1 * w + x) * 3 + 1] = x
  buf[(2 * w + x) * 3 + 2] = x
  buf[(3 * w + x) * 3 + 0] = x; buf[(3 * w + x) * 3 + 1] = x; buf[(3 * w + x) * 3 + 2] = x
}
const a = await sharp(buf, {raw: {width: w, height: h, channels: 3}}).grayscale().raw()
  .toBuffer({resolveWithObject: true})
fs.writeFileSync(path.join(outDir, 'pure_channels.gray.raw'), a.data)
console.log('pure channels', JSON.stringify(a.info))

const n = 17 ** 3
const rgb = Buffer.alloc(n * 3)
let i = 0
for (let r = 0; r < 17; r++) for (let g = 0; g < 17; g++) for (let b = 0; b < 17; b++) {
  rgb[i * 3] = Math.min(255, r * 16); rgb[i * 3 + 1] = Math.min(255, g * 16); rgb[i * 3 + 2] = Math.min(255, b * 16); i++
}
const g2 = await sharp(rgb, {raw: {width: n, height: 1, channels: 3}}).grayscale().raw()
  .toBuffer({resolveWithObject: true})
fs.writeFileSync(path.join(outDir, 'grid.rgb.raw'), rgb)
fs.writeFileSync(path.join(outDir, 'grid.gray.raw'), g2.data)
console.log('grid', JSON.stringify(g2.info))
