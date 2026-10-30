// B6 / RM-2: reproduce the luminance step of the public image-target-cli (apply.js) with the real
// sharp (0.34.5) and dump the intermediate buffers, so the grayscale / resize behaviour of
// sharp+libvips can be compared with our PIL implementation without guessing.
//
// Usage: node sharp_gray_stages.mjs <orig-cli-dir> <out-dir> <image> [<image> ...]
//   <orig-cli-dir> = rm2-results/orig-cli (unmodified copy of the public src with node_modules)
// Per image <id> (path with / replaced by __) writes <out-dir>/<id>/:
//   meta.json                   sharp/libvips versions, sizes, crop, format
//   cropped_rgb.raw             the cropped image exactly as apply.js makes it (pre-resize), RGB(A)
//   A_final_gray.raw            apply.js luminance: croppedImage.resize({height:640}).grayscale()  (1 ch)
//   C_resized_rgb.raw           same without .grayscale(): croppedImage.resize({height:640})
//   D_gray_fullres.raw          croppedImage.grayscale() at full resolution (no resize)
//   B_gray_then_resize.raw      D re-fed to sharp as raw 1 channel, then resize({height:640})
//   A_file.<ext>.raw            A re-encoded as the CLI does (toFile) and decoded back by sharp
// Everything is 8-bit, row-major, channels as in meta.json. No network.
import fs from 'fs/promises'
import path from 'path'
import {createRequire} from 'module'
import {pathToFileURL} from 'url'

const [origDir, outDir, ...images] = process.argv.slice(2)
if (!origDir || !outDir || !images.length) {
  console.error('usage: node sharp_gray_stages.mjs <orig-cli-dir> <out-dir> <image>...')
  process.exit(2)
}
const require = createRequire(path.join(origDir, 'x.js'))
const sharp = require('sharp')
const {getDefaultCrop} = await import(pathToFileURL(path.join(origDir, 'src/crop.js')).href)
const CONSTANTS = JSON.parse(await fs.readFile(path.join(origDir, 'src/constants.json'), 'utf8'))

const rawOf = async (img) => {
  const {data, info} = await img.clone().raw().toBuffer({resolveWithObject: true})
  return {data, info}
}

for (const imagePath of images) {
  const id = imagePath.replace(/^\/+/, '').replace(/[\/]/g, '__')
  const dir = path.join(outDir, id)
  await fs.mkdir(dir, {recursive: true})

  // ---- the same steps as interactive.js (flat, default crop) + apply.js ----
  const rawImage = sharp(imagePath)
  rawImage.autoOrient()
  const {autoOrient: baseMetadata, format} = await rawImage.metadata()
  const sourceIsLandscape = baseMetadata.width >= baseMetadata.height
  const crop = getDefaultCrop(baseMetadata, sourceIsLandscape)
  const originalImage = crop.isRotated ? rawImage.clone().rotate(90) : rawImage
  const croppedImage = originalImage.clone().extract(crop)
  const finalLum = croppedImage.clone().resize({height: CONSTANTS.luminanceHeight}).grayscale()

  const cropped = await rawOf(croppedImage)
  const A = await rawOf(finalLum)
  const C = await rawOf(croppedImage.clone().resize({height: CONSTANTS.luminanceHeight}))
  const D = await rawOf(croppedImage.clone().grayscale())
  const B = await rawOf(
    sharp(D.data, {raw: {width: D.info.width, height: D.info.height, channels: D.info.channels}})
      .resize({height: CONSTANTS.luminanceHeight}))
  const ext = format === 'jpeg' ? 'jpg' : format
  const filePath = path.join(dir, `A_file.${ext}`)
  await finalLum.clone().toFile(filePath)
  const F = await rawOf(sharp(filePath))

  const write = (name, o) => fs.writeFile(path.join(dir, name), o.data)
  await Promise.all([write('cropped_rgb.raw', cropped), write('A_final_gray.raw', A),
    write('C_resized_rgb.raw', C), write('D_gray_fullres.raw', D), write('B_gray_then_resize.raw', B),
    write(`A_file.${ext}.raw`, F)])
  await fs.writeFile(path.join(dir, 'meta.json'), JSON.stringify({
    image: imagePath, format, ext, exifOrientation: (await sharp(imagePath).metadata()).orientation ?? null,
    baseMetadata: {width: baseMetadata.width, height: baseMetadata.height}, crop,
    cropped: cropped.info, A: A.info, C: C.info, D: D.info, B: B.info, file: F.info,
    sharp: sharp.versions, simd: sharp.simd(),
  }, null, 1))
  console.log(id, 'crop', JSON.stringify(crop), 'A', A.info.width + 'x' + A.info.height + 'x' + A.info.channels,
    'C ch', C.info.channels)
}
