#!/usr/bin/env node
// R3 (RM-2 / US-012): run the PUBLIC reference runtime image-target CLI, unmodified and not copied, with its
// interactive prompts answered from a script (the public `test/*.txt` method: `cat case.txt | node src/index.js`).
//
// Usage:
//   node run_public_cli.mjs --script answers.txt [--overwrite]      (or answers on stdin)
//
// Environment:
//   xr_reference_ROOT      root of the reference runtime clone (default <repo>/assets/references/xr_reference/official/xr_reference)
//   SHARP_NODE_MODULES   directory that contains sharp/ (see sharp_path.mjs)
//
// Answer order of the public prompts (src/interactive.js): image path, type (flat|cylinder|cone), geometry
// answers, output folder, target name. Use ABSOLUTE paths for the image and the folder: the CLI resolves them
// against the working directory, and the public test scripts' `test/love.jpg` is a git-lfs pointer in the clone
// (the real images are under build/slam-refact-full-c1-net/lfs-files, checked by lfs-files.SHA256SUMS).
// --overwrite sets OVERWRITE_FILES=true like the public test/all.sh.
//
// The CLI writes <name>.json, <name>_{original,cropped,thumbnail,luminance}.<ext> (+ _geometry for cones).
// Convert <name>_luminance.<ext> to the PGM the native tests read with to_pgm.mjs.
import fs from 'fs'
import path from 'path'
import {spawnSync} from 'child_process'
import {fileURLToPath} from 'url'

import {repoRoot} from './sharp_path.mjs'

const here = path.dirname(fileURLToPath(import.meta.url))

const parseArgs = (argv) => {
  const out = {script: null, overwrite: false}
  for (let i = 0; i < argv.length; ++i) {
    if (argv[i] === '--script') {
      out.script = argv[++i]
    } else if (argv[i] === '--overwrite') {
      out.overwrite = true
    } else {
      throw new Error(`unknown argument: ${argv[i]}`)
    }
  }
  return out
}

const main = () => {
  const args = parseArgs(process.argv.slice(2))
  const root = path.resolve(
    process.env.xr_reference_ROOT || path.join(repoRoot, 'assets/references/xr_reference/official/xr_reference')
  )
  const cliEntry = path.join(root, 'apps/image-target-cli/src/index.js')
  if (!fs.existsSync(cliEntry)) {
    throw new Error(`public CLI not found: ${cliEntry} (set xr_reference_ROOT)`)
  }
  const hook = path.join(here, 'sharp_resolve_hook.mjs')
  const env = {...process.env}
  if (args.overwrite) {
    env.OVERWRITE_FILES = 'true'
  }
  const stdio = [args.script ? fs.openSync(args.script, 'r') : 'inherit', 'inherit', 'inherit']
  console.error(`run_public_cli: node --import ${hook} ${cliEntry} < ${args.script || 'stdin'}`)
  const r = spawnSync(process.execPath, ['--import', hook, cliEntry], {stdio, env})
  if (r.error) {
    throw r.error
  }
  process.exit(r.status === null ? 1 : r.status)
}

try {
  main()
} catch (err) {
  console.error('run_public_cli:', err.message)
  process.exit(2)
}
