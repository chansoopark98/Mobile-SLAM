// US-025 LC-G hang diagnosis, armed variant: attach to the page of a run started with chrome-debug-wrapper.sh WHILE it still responds and enable the
// debugger (Debugger.enable is not interruptible, so it cannot be sent once the renderer spins in an endless loop); after `hangAfterS` seconds without the
// run finishing, send Debugger.pause (interruptible: V8 stops even WebAssembly loops) and print the call stack, then resume.
// usage: node cdp_arm.mjs <port> <hangAfterS> <runDir-meta.json path to watch> [pauseTimeoutMs]
const [port = '9333', hangAfterS = '150', metaPath = '', pauseTimeoutMs = '30000'] = process.argv.slice(2);
import fs from 'node:fs';
const sleep = ms => new Promise(r => setTimeout(r, ms));
let target = null;
for (let i = 0; i < 300 && !target; i++) {
  try { target = (await (await fetch(`http://127.0.0.1:${port}/json/list`)).json()).find(t => t.type === 'page' && /index\.html/.test(t.url)); } catch (e) { /* browser not up yet */ }
  if (!target) await sleep(500);
}
if (!target) { console.log(JSON.stringify({error: 'no page target'})); process.exit(0); }
const ws = new WebSocket(target.webSocketDebuggerUrl);
await new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
let id = 0, pausedParams = null;
const pending = new Map(), waiters = [];
ws.onmessage = m => {
  const msg = JSON.parse(m.data);
  if (msg.id && pending.has(msg.id)) { pending.get(msg.id)(msg); pending.delete(msg.id); }
  if (msg.method === 'Debugger.paused') { pausedParams = msg.params; waiters.splice(0).forEach(w => w(msg.params)); }
};
ws.onclose = () => process.exit(0);
const send = (method, params = {}) => new Promise(res => { const i = ++id; pending.set(i, res); ws.send(JSON.stringify({id: i, method, params})); });
await send('Debugger.enable');
const start = Date.now();
while (Date.now() - start < Number(hangAfterS) * 1000) {
  if (metaPath && fs.existsSync(metaPath)) { console.log(JSON.stringify({finishedBeforeHangTimeout: true})); process.exit(0); }
  await sleep(1000);
}
await send('Debugger.pause').catch(() => {});
const ev = pausedParams ?? await Promise.race([new Promise(r => waiters.push(r)), sleep(Number(pauseTimeoutMs)).then(() => null)]);
console.log(JSON.stringify({paused: !!ev, reason: ev?.reason ?? null, url: target.url,
  frames: (ev?.callFrames ?? []).slice(0, 30).map(f => ({fn: f.functionName, url: f.url.slice(0, 100), line: f.location.lineNumber, col: f.location.columnNumber}))}, null, 1));
process.exit(0);
