// DeviceMotion queue of the slam_refact xr-slam chunk: the original sensors.ts logic
// (ARC:reality/app/xr/js/src/sensors.ts:15-85; deployed SDK/xr-slam.js offset ~5,494,950 `k=function(A){...}`)
// without capnp.
//
// The original pushes one queue entry per sensor kind, and setEventQueue() serialises the queue
// for c8EmAsm_stageFrame and empties it. Here one entry is one DeviceMotion event (the engine's
// GlueRunner::PushDeviceMotion splits it into LINEAR_ACCELERATION, ACCELEROMETER, GYROSCOPE in the
// same order, frontend/sensor_intake.cc:126-146), and takeForStage() hands the entries staged with a
// frame to the caller, which pushes them into the engine right before that frame is processed.
// The 5e9 ns clear rule is applied here exactly as in the original; GlueRunner's SensorIntake
// applies the same rule to the same push sequence, so it never fires a second time.
export function createSensors(isInvalid, diagnosticOptions = null) {
  const eventQueue = [];
  const delivery = diagnosticOptions?.enabled === true ? createDeliveryDiagnostics(diagnosticOptions.maxEntries) : null;

  // updateDeviceMotion: timestampNanos = 1e3 * performance.now() * 1e3 (same float operation order).
  const updateDeviceMotion = ({ acceleration, accelerationIncludingGravity, rotationRate, timeStamp, interval }) => {
    const seq = delivery?.receive();
    if (isInvalid()) { delivery?.reject(seq, 'invalid-context'); return; }
    const timestampNanos = 1e3 * window.performance.now() * 1e3;
    const eventTimestampNanos = 1e3 * timeStamp * 1e3;
    if (eventQueue.length && timestampNanos - eventQueue[0].timestampNanos > 5e9) {
      delivery?.drop(eventQueue, 'queue-age-clear');
      eventQueue.length = 0;  // clear the queue for large lag times
    }
    const entry = { timestampNanos, eventTimestampNanos, interval, acceleration, accelerationIncludingGravity, rotationRate };
    delivery?.enqueue(entry, seq);
    eventQueue.push(entry);
  };

  // setEventQueue: returns the staged entries and empties the queue (nothing when invalid, like the original).
  const takeForStage = () => {
    if (isInvalid()) return null;
    const staged = eventQueue.slice();
    eventQueue.length = 0;
    delivery?.stage(staged);
    return staged;
  };

  const carryForStage = (events) => {
    if (!events?.length) return;
    delivery?.carry(events);
    eventQueue.unshift(...events);
  };

  return { updateDeviceMotion, takeForStage, carryForStage, queueLength: () => eventQueue.length, ...(delivery ? { delivery } : {}) };
}

// M0: event-unit ownership accounting only. WeakMap metadata never changes the event/payload,
// and detailed timing/drop histories are fixed-size rings; no raw acceleration/gyro is exported.
function createDeliveryDiagnostics(limit = 256) {
  const maxEntries = Number.isInteger(limit) && limit >= 1 && limit <= 1024 ? limit : 256;
  const ring = () => {
    const entries = []; let next = 0, total = 0;
    return { push(value) { entries[next] = value; next = (next + 1) % maxEntries; total++; },
      snapshot: () => ({ total, overwritten: Math.max(0, total - maxEntries),
        entries: (total < maxEntries ? entries : entries.slice(next).concat(entries.slice(0, next))).map((e) => ({ ...e })) }) };
  };
  const timings = ring(), drops = ring(), transitions = ring(), states = new WeakMap();
  const pending = { queued: 0, staged: 0, inFlight: 0 };
  const droppedByReason = {};
  let received = 0, pushed = 0, dropped = 0, uncertain = 0, duplicateSettlements = 0;
  let previousArrival = null, previousEvent = null;
  const finite = (n) => Number.isFinite(n) ? n : null;
  const context = () => {
    try { return { visibility: globalThis.document?.visibilityState ?? 'unknown',
      focused: typeof globalThis.document?.hasFocus === 'function' ? globalThis.document.hasFocus() : null }; }
    catch (_) { return { visibility: 'unknown', focused: null }; }
  };
  const transition = (e, phase) => {
    const state = states.get(e);
    if (!state || !(state.phase in pending)) { duplicateSettlements++; return null; }
    pending[state.phase]--;
    transitions.push({ seq: state.seq, from: state.phase, to: phase });
    state.phase = phase;
    if (phase in pending) pending[phase]++;
    return state.seq;
  };
  const recordDrops = (seqs, reason) => {
    if (!seqs.length) return;
    dropped += seqs.length;
    droppedByReason[reason] = (droppedByReason[reason] || 0) + seqs.length;
    let first = seqs[0], last = first;
    for (const seq of seqs.slice(1)) {
      if (seq === last + 1) last = seq;
      else { drops.push({ reason, firstSeq: first, lastSeq: last, count: last - first + 1 }); first = last = seq; }
    }
    drops.push({ reason, firstSeq: first, lastSeq: last, count: last - first + 1 });
  };
  return {
    receive: () => ++received,
    seqOf: (event) => states.get(event)?.seq ?? null,
    reject: (seq, reason) => recordDrops([seq], reason),
    enqueue(e, seq) {
      states.set(e, { seq, phase: 'queued' }); pending.queued++;
      timings.push({ seq, arrivalTimeNanos: finite(e.timestampNanos), eventTimeNanos: finite(e.eventTimestampNanos),
        // Raw DeviceMotionEvent.interval as delivered (spec: ms; WebKit unit UNVERIFIED). pushMotion uses interval * 1e6 as ns.
        rawInterval: finite(e.interval),
        arrivalDtMs: previousArrival === null ? null : finite((e.timestampNanos - previousArrival) / 1e6),
        eventDtMs: previousEvent === null ? null : finite((e.eventTimestampNanos - previousEvent) / 1e6), ...context() });
      previousArrival = e.timestampNanos; previousEvent = e.eventTimestampNanos;
    },
    stage: (events) => { for (const e of events) transition(e, 'staged'); },
    carry: (events) => { for (const e of events) transition(e, 'queued'); },
    process: (events) => { for (const e of events) transition(e, 'inFlight'); },
    pushed(e) { if (transition(e, 'pushed') !== null) pushed++; },
    uncertain(e) { if (transition(e, 'uncertain') !== null) uncertain++; },
    drop(events, reason) { recordDrops(events.filter((e) => states.get(e)?.phase in pending)
      .map((e) => transition(e, 'dropped')), reason); },
    snapshot: () => ({ unit: 'DeviceMotion-event', received, pushed, dropped, uncertainPushes: uncertain,
      pending: { ...pending }, droppedByReason: { ...droppedByReason }, duplicateSettlements,
      accountingError: received - pushed - dropped - uncertain - pending.queued - pending.staged - pending.inFlight,
      preReceiveLoss: 'unknown', pushedMeaning: 'successful-glue-push-call-not-backend-fusion', maxEntries,
      timings: timings.snapshot(), dropRanges: drops.snapshot(), transitions: transitions.snapshot(), context: context() }),
  };
}

// Pushes one staged DeviceMotion entry into SlamRefactGlue.pushDeviceMotionEvent (FULL-7, spec 06 section 6.1): the
// binding does what the original sensors.ts updateDeviceMotion does - each of acceleration, accelerationIncludingGravity
// and rotationRate that is truthy becomes its own sensor event (LINEAR_ACCELERATION, ACCELEROMETER, GYROSCOPE) and the
// numbers go through Number(...) like capnp's float setters. The sub-objects are handed over as they are, so a device
// without a gyroscope (rotationRate null) still delivers its accelerometer (RM-4c; before this the whole event was
// skipped).
//
// Returns false for the one event the engine does not take: `interval * 1e6` outside the capnp Int64 range (or infinite) on
// an event that has a sensor. The engine binding then throws a JS error ("interval is outside the Int64 nanosecond range",
// VERIFIED on the engine pinned for RM-4d, probe: sensor + 1e14 ms / Infinity throw, the same interval on an event without
// any sensor is a no-op, NaN is taken as 0 ns like capnp-ts); an engine built before RM-3c aborted the WASM module instead
// (RM-4c review m4: the older comment and spec 07 section 3 said "abort for events with or without sensors"). The original
// throws a RangeError from setEventQueue for such an event (its whole stage fails and the queue is retried until the 5e9 ns rule
// clears it); the chunk drops that one event and counts it, so a bad event cannot fail the frame. The check is the same
// for events without a sensor (dropped here, a no-op there). The order of the fields the binding reads inside each sensor
// object (alpha/beta/gamma, x/y/z) cannot be seen from JS: the objects are handed over untouched; the engine side owns that
// evidence (RM-3c), this module only proves that nothing is swapped on the way in.
const TWO_POW_32 = 4294967296;
export function pushMotion(glue, e) {
  const ns = e.interval * 1000 * 1000;  // sensors.ts:56, evaluated left to right
  if (!Number.isNaN(ns) && !(Math.abs(ns) / TWO_POW_32 <= TWO_POW_32)) return false;
  glue.pushDeviceMotionEvent(e.timestampNanos, {
    acceleration: e.acceleration,
    accelerationIncludingGravity: e.accelerationIncludingGravity,
    rotationRate: e.rotationRate,
    interval: e.interval,
  });
  return true;
}
