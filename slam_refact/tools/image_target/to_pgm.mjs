#!/usr/bin/env node
// R3 (RM-2 / US-012): convert the luminance image written by the public image-target CLI
// (<name>_luminance.jpg|png) to the binary PGM that the native tests read
// (slam_refact/engine/src/image_target/target_files.h: P5, maxval 255, raster = width*height bytes).
//
// Usage: node to_pgm.mjs <in.jpg|in.png> <out.pgm> [--max-channel-diff N]
// Prints one JSON line: input/output sha256, width, height, file/raw channels, observed channel difference.
//
// Pixel rule (the same one the browser path uses: it draws the image and reads the R channel only):
//   sharp's raw() output expands a gray file to R=G=B(+A), so for every input the converter takes the R
//   channel of the decoded RGB(A) bytes, and only when R, G and B agree within --max-channel-diff
//   (default 0). An image that is really colour must not be silently turned into a luminance target.
//   `file_channels` in the output is the channel count of the FILE (sharp metadata: 1 gray, 2 gray+alpha,
//   3 RGB, 4 RGBA); the public CLI's `.grayscale()` output is not necessarily a 1-channel file.
// sharp is asked for the raw decoded bytes (no resize, no colour conversion, no EXIF rotation: the CLI has
// already oriented the image and sharp does not write EXIF by default). Needs sharp (sharp_path.mjs).
import crypto from 'crypto'
import fs from 'fs'
import {fileURLToPath} from 'url'

import {loadSharp} from './sharp_path.mjs'

const sha256 = (buf) => crypto.createHash('sha256').update(buf).digest('hex')

/**
 * @param {string} inputPath
 * @param {{maxChannelDiff?: number}} [options]
 * @returns {Promise<{pgm: Buffer, width: number, height: number, channels: number,
 *   fileChannels: number, maxChannelDiff: number}>}
 */
const toPgm = async (inputPath, options = {}) => {
  const maxChannelDiff = options.maxChannelDiff || 0
  const sharp = loadSharp()
  const fileChannels = (await sharp(inputPath).metadata()).channels
  const {data, info} = await sharp(inputPath).raw().toBuffer({resolveWithObject: true})
  const {width, height, channels} = info
  if (width <= 0 || height <= 0 || data.length !== width * height * channels) {
    throw new Error(`unexpected raw size ${data.length} for ${width}x${height}x${channels}`)
  }
  let observedDiff = 0
  const gray = Buffer.alloc(width * height)
  for (let i = 0; i < width * height; ++i) {
    const o = i * channels
    gray[i] = data[o]
    if (channels >= 3) {
      observedDiff = Math.max(
        observedDiff, Math.abs(data[o] - data[o + 1]), Math.abs(data[o] - data[o + 2])
      )
    }
  }
  if (observedDiff > maxChannelDiff) {
    throw new Error(
      `not a luminance image: R/G/B differ by up to ${observedDiff} (> ${maxChannelDiff}) in ${inputPath}`
    )
  }
  const header = Buffer.from(`P5\n${width} ${height}\n255\n`, 'latin1')
  return {
    pgm: Buffer.concat([header, gray]), width, height, channels, fileChannels, maxChannelDiff: observedDiff,
  }
}

const main = async () => {
  const argv = process.argv.slice(2)
  const positional = []
  let maxChannelDiff = 0
  for (let i = 0; i < argv.length; ++i) {
    if (argv[i] === '--max-channel-diff') {
      maxChannelDiff = Number(argv[++i])
    } else {
      positional.push(argv[i])
    }
  }
  if (positional.length !== 2 || !Number.isInteger(maxChannelDiff) || maxChannelDiff < 0) {
    throw new Error('usage: to_pgm.mjs <in.jpg|in.png> <out.pgm> [--max-channel-diff N]')
  }
  const [input, output] = positional
  const r = await toPgm(input, {maxChannelDiff})
  fs.writeFileSync(output, r.pgm)
  console.log(JSON.stringify({
    input,
    input_sha256: sha256(fs.readFileSync(input)),
    output,
    output_sha256: sha256(r.pgm),
    width: r.width,
    height: r.height,
    file_channels: r.fileChannels,
    raw_channels: r.channels,
    observed_max_channel_diff: r.maxChannelDiff,
  }))
}

if (process.argv[1] && fileURLToPath(import.meta.url) === fs.realpathSync(process.argv[1])) {
  main().catch((err) => {
    console.error('to_pgm:', err.message)
    process.exit(2)
  })
}

export {toPgm}
