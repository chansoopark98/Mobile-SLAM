// R3: unit tests of to_pgm.mjs (development tool, not a product test).
// Run: SHARP_NODE_MODULES=<dir with sharp/> node --test slam_refact/tools/image_target/to_pgm.test.mjs
//   LFS_TEST_IMAGES  directory with the real git-lfs test images (default
//                    <repo>/build/slam-refact-full-c1-net/lfs-files/apps/image-target-cli/test).
// The synthetic cases always run. The cases on the real public test images (love.jpg, cone-cup.png) are
// skipped, with the reason printed, only when those git-lfs files are not present (build/ is not in git).
import assert from 'node:assert/strict'
import crypto from 'crypto'
import fs from 'fs'
import os from 'os'
import path from 'path'
import {spawnSync} from 'child_process'
import {fileURLToPath} from 'url'
import {test} from 'node:test'

import {loadSharp, repoRoot} from './sharp_path.mjs'
import {toPgm} from './to_pgm.mjs'

const here = path.dirname(fileURLToPath(import.meta.url))
const sharp = loadSharp()
const lfsDir = process.env.LFS_TEST_IMAGES ||
  path.join(repoRoot, 'build/slam-refact-full-c1-net/lfs-files/apps/image-target-cli/test')

const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'to_pgm_test_'))

// The PGM rules of slam_refact/engine/src/image_target/target_files.cc ParsePgm.
const parsePgm = (buf) => {
  assert.equal(buf.subarray(0, 2).toString('latin1'), 'P5')
  const m = /^P5\s+(\d+)\s+(\d+)\s+(\d+)\s/.exec(buf.subarray(0, 64).toString('latin1'))
  assert.ok(m, 'header')
  const [width, height, maxval] = [Number(m[1]), Number(m[2]), Number(m[3])]
  assert.equal(maxval, 255)
  const raster = buf.subarray(m[0].length)
  assert.equal(raster.length, width * height)
  return {width, height, raster, headerBytes: m[0].length}
}

const gradient = (w, h) => {
  const g = Buffer.alloc(w * h)
  for (let i = 0; i < g.length; ++i) {
    g[i] = (i * 7 + (i >> 5)) & 255
  }
  return g
}

test('gray PNG: header and raster equal the decoded bytes', async () => {
  const w = 37
  const h = 23
  const g = gradient(w, h)
  const file = path.join(tmp, 'gray1.png')
  await sharp(g, {raw: {width: w, height: h, channels: 1}}).toColourspace('b-w').png().toFile(file)
  const r = await toPgm(file)
  assert.equal(r.fileChannels, 1)
  assert.equal(r.pgm.subarray(0, 13).toString('latin1'), 'P5\n37 23\n255\n')
  const p = parsePgm(r.pgm)
  assert.deepEqual([p.width, p.height], [w, h])
  assert.ok(p.raster.equals(g))
})

test('gray+alpha PNG: gray channel, alpha ignored', async () => {
  const w = 16
  const h = 9
  const g = gradient(w, h)
  const raw = Buffer.alloc(w * h * 2)
  for (let i = 0; i < g.length; ++i) {
    raw[2 * i] = g[i]
    raw[2 * i + 1] = 255 - g[i]
  }
  const file = path.join(tmp, 'gray_alpha.png')
  await sharp(raw, {raw: {width: w, height: h, channels: 2}}).toColourspace('b-w').png().toFile(file)
  const r = await toPgm(file)
  assert.equal(r.fileChannels, 2)
  assert.ok(parsePgm(r.pgm).raster.equals(g))
})

test('RGBA PNG with R=G=B (what the public CLI writes for PNG sources): R channel', async () => {
  const w = 20
  const h = 11
  const g = gradient(w, h)
  const rgba = Buffer.alloc(w * h * 4)
  for (let i = 0; i < g.length; ++i) {
    rgba.fill(g[i], 4 * i, 4 * i + 3)
    rgba[4 * i + 3] = 255
  }
  const file = path.join(tmp, 'rgba_gray.png')
  await sharp(rgba, {raw: {width: w, height: h, channels: 4}}).png().toFile(file)
  const r = await toPgm(file)
  assert.equal(r.fileChannels, 4)
  assert.equal(r.maxChannelDiff, 0)
  assert.ok(parsePgm(r.pgm).raster.equals(g))
})

test('colour image is refused, and accepted only within an explicit tolerance', async () => {
  const w = 8
  const h = 8
  const rgb = Buffer.alloc(w * h * 3)
  for (let i = 0; i < w * h; ++i) {
    rgb[3 * i] = 100
    rgb[3 * i + 1] = 90
    rgb[3 * i + 2] = 100
  }
  const file = path.join(tmp, 'colour.png')
  await sharp(rgb, {raw: {width: w, height: h, channels: 3}}).png().toFile(file)
  await assert.rejects(() => toPgm(file), /not a luminance image: R\/G\/B differ by up to 10/)
  await assert.rejects(() => toPgm(file, {maxChannelDiff: 9}), /not a luminance image/)
  const r = await toPgm(file, {maxChannelDiff: 10})
  assert.equal(r.maxChannelDiff, 10)
  assert.ok(parsePgm(r.pgm).raster.every((v) => v === 100))
})

test('command line: writes the PGM and prints matching sha256 and size', async () => {
  const w = 12
  const h = 5
  const g = gradient(w, h)
  const input = path.join(tmp, 'cli_in.png')
  const output = path.join(tmp, 'cli_out.pgm')
  await sharp(g, {raw: {width: w, height: h, channels: 1}}).png().toFile(input)
  const r = spawnSync(process.execPath, [path.join(here, 'to_pgm.mjs'), input, output], {encoding: 'utf8'})
  assert.equal(r.status, 0, r.stderr)
  const info = JSON.parse(r.stdout)
  const bytes = fs.readFileSync(output)
  assert.equal(info.width, w)
  assert.equal(info.height, h)
  assert.equal(info.output_sha256, crypto.createHash('sha256').update(bytes).digest('hex'))
  assert.ok(parsePgm(bytes).raster.equals(g))
  const bad = spawnSync(process.execPath, [path.join(here, 'to_pgm.mjs'), input], {encoding: 'utf8'})
  assert.equal(bad.status, 2)
})

const realImage = (name) => {
  const file = path.join(lfsDir, name)
  if (!fs.existsSync(file) || fs.statSync(file).size < 1000) {
    return null  // missing, or still a git-lfs pointer (131 bytes)
  }
  return file
}

// CLI-like luminance files from a real public test image: resize to height 640 and .grayscale(), the calls
// of the public apply.js, written as JPEG (3 channels expected) and as PNG (4 channels expected).
for (const [name, ext] of [['love.jpg', 'jpg'], ['cone-cup.png', 'png']]) {
  test(`real public test image ${name}: PGM bytes equal sharp.raw() of the same file`, async (t) => {
    const src = realImage(name)
    if (!src) {
      t.skip(`${name} not found or an LFS pointer under ${lfsDir}`)
      return
    }
    const lum = path.join(tmp, `lum_${name}.${ext}`)
    await sharp(src).resize({height: 640}).grayscale().toFile(lum)
    const r = await toPgm(lum)
    const raw = await sharp(lum).raw().toBuffer({resolveWithObject: true})
    const {width, height, channels} = raw.info
    const expected = Buffer.alloc(width * height)
    for (let i = 0; i < expected.length; ++i) {
      expected[i] = raw.data[i * channels]
    }
    const p = parsePgm(r.pgm)
    assert.deepEqual([p.width, p.height], [width, height])
    assert.equal(p.height, 640)
    assert.equal(r.channels, channels)
    assert.equal(r.fileChannels, ext === 'jpg' ? 3 : 4)
    assert.equal(r.maxChannelDiff, 0)
    assert.ok(p.raster.equals(expected))
    assert.equal(
      r.pgm.subarray(0, p.headerBytes).toString('latin1'), `P5\n${width} ${height}\n255\n`
    )
  })
}

test('real colour public test image is refused as a luminance image', async (t) => {
  const src = realImage('love.jpg')
  if (!src) {
    t.skip(`love.jpg not found or an LFS pointer under ${lfsDir}`)
    return
  }
  await assert.rejects(() => toPgm(src), /not a luminance image/)
})
