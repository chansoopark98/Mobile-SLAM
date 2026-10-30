// R3: preload (node --import) that resolves the bare specifier 'sharp' of the UNMODIFIED public CLI
// sources to the dev-only install named by SHARP_NODE_MODULES (see sharp_path.mjs). Nothing is copied
// into, or installed in, the reference runtime clone.
import {registerHooks} from 'module'
import {pathToFileURL} from 'url'

import {sharpEntry} from './sharp_path.mjs'

const entryUrl = pathToFileURL(sharpEntry()).href

registerHooks({
  resolve(specifier, context, nextResolve) {
    if (specifier === 'sharp') {
      return nextResolve(entryUrl, context)
    }
    return nextResolve(specifier, context)
  },
})
