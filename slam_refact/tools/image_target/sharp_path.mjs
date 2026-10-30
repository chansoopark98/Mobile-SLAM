// R3: where the development-only sharp 0.34.5 lives. Shared by run_public_cli.mjs and to_pgm.mjs.
//
// sharp is a dev-only dependency of the public reference runtime image-target CLI (MIT, `sharp ^0.34.5`); it is
// installed once under build/ (no new npm install, not a product runtime dependency). ES modules do not
// honour NODE_PATH, so the location is passed explicitly:
//   SHARP_NODE_MODULES  directory that contains sharp/ (default: <repo>/build/slam-refact-full-c1-net/npm/node_modules)
import path from 'path'
import {createRequire} from 'module'
import {fileURLToPath} from 'url'

const here = path.dirname(fileURLToPath(import.meta.url))
const repoRoot = path.resolve(here, '..', '..', '..')

const sharpModulesDir = () => path.resolve(
  process.env.SHARP_NODE_MODULES ||
  path.join(repoRoot, 'build', 'slam-refact-full-c1-net', 'npm', 'node_modules')
)

// Absolute path of sharp's CommonJS entry point.
const sharpEntry = () => createRequire(path.join(sharpModulesDir(), 'x.js')).resolve('sharp')

const loadSharp = () => createRequire(path.join(sharpModulesDir(), 'x.js'))('sharp')

export {repoRoot, sharpModulesDir, sharpEntry, loadSharp}
