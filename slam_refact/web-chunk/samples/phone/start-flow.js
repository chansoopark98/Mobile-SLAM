// The Start tap, as an ordered sequence. Kept out of app.js so the order is testable:
//   1. preflight (secure context, camera API, WASM SIMD) - synchronous, nothing is asked yet
//   2. motion permission requests - STARTED SYNCHRONOUSLY in the tap, before the first await (iOS accepts them only inside the user gesture)
//   3. engine script (xr.js) is loaded - only after the permission answer, because the chunk registers sensor listeners when it loads
//   4. the pipeline modules are added and XR8.run is called (this is where the camera permission prompt appears)
import { detectPlatform, preflight, requestMotionPermissions, describeMotionFailure } from '../common/platform.js';

export class StartError extends Error {
  constructor(info) { super(info.title); this.name = 'StartError'; this.info = info; }
}

// env: { win, nav, wa }   steps: { loadXr8(), run(), onStep?(name) }
export async function runStartFlow(env, steps) {
  const platform = detectPlatform(env.nav);
  const blocked = preflight(env);
  if (blocked) throw new StartError(blocked);
  // No await above this line and none between preflight and the request: the calls below happen in the same task as the click.
  const permission = requestMotionPermissions(env.win);
  steps.onStep && steps.onStep('permission');
  const result = await permission;
  if (result.motion !== 'granted' && result.motion !== 'unsupported') throw new StartError(describeMotionFailure(result, platform));
  if (result.orientation === 'denied') throw new StartError(describeMotionFailure(result, platform));
  steps.onStep && steps.onStep('engine');
  await steps.loadXr8();
  steps.onStep && steps.onStep('camera');
  steps.run();
  return { platform, permission: result };
}
