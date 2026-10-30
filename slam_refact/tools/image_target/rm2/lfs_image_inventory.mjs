// B6 / RM-2: what the LFS test images really are (sharp metadata, EXIF orientation applied).
// Usage: node lfs_image_inventory.mjs <orig-cli-dir> <lfs-files-root> > inventory.tsv
// Columns: path, real container format (sharp), file bytes, stored WxH, EXIF orientation, WxH after
// autoOrient (what the image-target CLI and reference runtime see), channels, bit depth, hasAlpha, colour space.
import fs from 'fs/promises'
import path from 'path'
import {createRequire} from 'module'

const [origDir, root] = process.argv.slice(2)
const require = createRequire(path.join(origDir, 'x.js'))
const sharp = require('sharp')

const walk = async (dir) => {
  const out = []
  for (const e of await fs.readdir(dir, {withFileTypes: true})) {
    const p = path.join(dir, e.name)
    if (e.isDirectory()) out.push(...await walk(p))
    else out.push(p)
  }
  return out
}
console.log(['path', 'format', 'bytes', 'stored_wh', 'exif_orientation', 'oriented_wh', 'channels', 'depth',
  'hasAlpha', 'space'].join('\t'))
for (const p of (await walk(root)).sort()) {
  const rel = path.relative(root, p)
  const bytes = (await fs.stat(p)).size
  if (!/\.(jpe?g|png)$/i.test(p)) {
    console.log([rel, 'NOT-AN-IMAGE-FOR-SHARP', bytes, '-', '-', '-', '-', '-', '-', '-'].join('\t'))
    continue
  }
  const m = await sharp(p).metadata()
  console.log([rel, m.format, bytes, `${m.width}x${m.height}`, m.orientation ?? '-',
    `${m.autoOrient.width}x${m.autoOrient.height}`, m.channels, m.depth, m.hasAlpha, m.space].join('\t'))
}
