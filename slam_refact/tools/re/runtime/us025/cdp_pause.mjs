// US-025 LC-G hang diagnosis: talk raw CDP over the loopback DevTools port of a run started with chrome-debug-wrapper.sh (no Playwright: connectOverCDP
// blocks on a hung renderer), pause the page's JS thread (Debugger.pause also stops WebAssembly code) and print the call stack.
// usage: node cdp_pause.mjs [port] [timeoutMs]
const port = process.argv[2] || '9333', timeoutMs = Number(process.argv[3] || 20000);
const list = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
const pages = list.filter(t => t.type === 'page');
const results = [];
for (const t of pages) {
  const ws = new WebSocket(t.webSocketDebuggerUrl);
  await new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
  let id = 0;
  const pending = new Map();
  let pausedEvent = null;
  const waiters = [];
  ws.onmessage = m => {
    const msg = JSON.parse(m.data);
    if (msg.id && pending.has(msg.id)) { pending.get(msg.id)(msg); pending.delete(msg.id); }
    if (msg.method === 'Debugger.paused') { pausedEvent = msg.params; waiters.splice(0).forEach(w => w(msg.params)); }
  };
  const send = (method, params = {}) => new Promise(res => { const i = ++id; pending.set(i, res); ws.send(JSON.stringify({id: i, method, params})); });
  await send('Debugger.enable');
  await send('Debugger.pause');
  const ev = pausedEvent ?? await Promise.race([new Promise(r => waiters.push(r)), new Promise(r => setTimeout(() => r(null), timeoutMs))]);
  results.push({url: t.url.slice(0, 80), paused: !!ev, reason: ev?.reason ?? null,
    frames: (ev?.callFrames ?? []).slice(0, 25).map(f => ({fn: f.functionName, url: f.url.slice(0, 90), line: f.location.lineNumber, col: f.location.columnNumber}))});
  try { await send('Debugger.resume'); } catch (e) { /* page may be gone */ }
  ws.close();
}
console.log(JSON.stringify(results, null, 1));
