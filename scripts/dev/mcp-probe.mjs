import { Client } from '@modelcontextprotocol/sdk/client/index.js';
import { StdioClientTransport } from '@modelcontextprotocol/sdk/client/stdio.js';
import { createServer } from 'node:http';
import { mkdir, writeFile } from 'node:fs/promises';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const devDir = dirname(fileURLToPath(import.meta.url));
const root = resolve(devDir, '../..');
const output = process.argv[2] || resolve(root, 'build/dev-tools/mcp-probe.json');
const report = { startedAt: new Date().toISOString(), servers: [] };

function textOf(result) {
  return (result.content || []).filter((item) => item.type === 'text')
    .map((item) => item.text).join('\n');
}

async function probe(name, calls) {
  const entry = { name, status: 'BLOCKED', calls: [] };
  report.servers.push(entry);
  const transport = new StdioClientTransport({
    command: 'bash', args: [resolve(devDir, 'mcp-server.sh'), name], cwd: root,
    stderr: 'pipe',
  });
  const client = new Client({ name: 'mobile-slam-tool-probe', version: '0.1.0' });
  try {
    await client.connect(transport, { timeout: 20_000 });
    entry.server = client.getServerVersion();
    const tools = await client.listTools({}, { timeout: 20_000 });
    entry.tools = tools.tools.map((tool) => tool.name);
    for (const call of calls) {
      const result = await client.callTool(call, undefined, { timeout: 45_000 });
      const content = textOf(result);
      entry.calls.push({ name: call.name, isError: !!result.isError,
        text: content.slice(0, 2400), characters: content.length });
      if (result.isError || /rate.limit|unauthorized|API key is required|failed to fetch/i.test(content)) {
        throw new Error(`${call.name} returned an error; see recorded tool result`);
      }
    }
    entry.status = 'VERIFIED';
  } catch (error) {
    entry.error = String(error);
    process.exitCode = 1;
  } finally {
    await client.close();
  }
}

const fixture = createServer((request, response) => {
  response.writeHead(200, { 'Content-Type': 'text/html' });
  response.end('<!doctype html><title>Mobile SLAM MCP probe</title><main><h1>Mobile SLAM MCP probe</h1><p>Local read-only fixture</p></main>');
});
await new Promise((ok) => fixture.listen(0, '127.0.0.1', ok));
try {
  await probe('playwright', [
    { name: 'browser_navigate', arguments: { url: `http://127.0.0.1:${fixture.address().port}/` } },
    { name: 'browser_snapshot', arguments: {} },
    { name: 'browser_close', arguments: {} },
  ]);
} finally {
  await new Promise((ok) => fixture.close(ok));
}
await probe('context7', [
  { name: 'resolve-library-id', arguments: {
    libraryName: 'emscripten', query: 'Emscripten WebAssembly memory growth and SIMD official documentation',
  } },
  { name: 'query-docs', arguments: {
    libraryId: '/emscripten-core/emscripten', query: 'ALLOW_MEMORY_GROWTH and SIMD compiler options',
  } },
]);
report.finishedAt = new Date().toISOString();
await mkdir(dirname(output), { recursive: true });
await writeFile(output, JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify({ output, servers: report.servers.map(({ name, status, error, tools }) => ({ name, status, error, toolCount: tools?.length })) }, null, 2));
