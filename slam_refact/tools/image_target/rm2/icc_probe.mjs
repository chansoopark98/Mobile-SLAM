// B6 / RM-2: dump what sharp(path).autoOrient().raw() returns for every LFS image (RGB(A)/gray, 8 bit), so
// icc_probe.py can compare it with PIL's decode (PIL ignores embedded ICC profiles; libvips/sharp applies them).
// Usage: node icc_probe.mjs <orig-cli-dir> <lfs-files-root> <out-dir>
import fs from 'fs/promises'
import path from 'path'
import {createRequire} from 'module'

const [origDir, root, outDir] = process.argv.slice(2)
const require = createRequire(path.join(origDir, 'x.js'))
const sharp = require('sharp')
await fs.mkdir(outDir, {recursive: true})
const walk = async (dir) => (await Promise.all((await fs.readdir(dir, {withFileTypes: true}))
  .map(async e => e.isDirectory() ? walk(path.join(dir, e.name)) : [path.join(dir, e.name)]))).flat()
const meta = []
for (const p of (await walk(root)).sort()) {
  if (!/\.(jpe?g|png)$/i.test(p)) continue
  const img = sharp(p).autoOrient()
  const m = await sharp(p).metadata()
  const {data, info} = await img.raw().toBuffer({resolveWithObject: true})
  const id = path.relative(root, p).replace(/\//g, '__')
  await fs.writeFile(path.join(outDir, id + '.raw'), data)
  meta.push({path: path.relative(root, p), id, width: info.width, height: info.height, channels: info.channels,
    hasProfile: !!m.hasProfile, iccBytes: m.icc ? m.icc.length : 0, space: m.space})
}
await fs.writeFile(path.join(outDir, 'meta.json'), JSON.stringify(meta, null, 1))
console.log('wrote', meta.length)
