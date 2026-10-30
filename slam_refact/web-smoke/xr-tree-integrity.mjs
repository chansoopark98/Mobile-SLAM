import crypto from 'node:crypto';

export const CURRENT_INTEGRITY_PATH = 'config/runtime-integrity/xr-tree.json';
const DEPLOY_PATH = 'slam-refact-xr/deploy.json';
const DIGEST_RULE = 'sha256 over "served path\\0sha256(content)\\n", sorted, excluding slam-refact-xr/deploy.json';
const isSha = (value) => typeof value === 'string' && /^[0-9a-f]{64}$/.test(value);
const requireValue = (condition, message) => { if (!condition) throw new Error(message); };

function fileMap(rows, prefix, label) {
  requireValue(Array.isArray(rows) && rows.length > 0, `${label}: files must be a nonempty array`);
  const result = new Map();
  for (const row of rows) {
    requireValue(row && typeof row.path === 'string' && row.path.startsWith(prefix), `${label}: invalid path`);
    const name = row.path.slice(prefix.length);
    requireValue((name === 'slam-refact-xr.html' || name.startsWith('slam-refact-xr/'))
      && !/[\\%\x00-\x20\x7f]/.test(name) && name.split('/').every((p) => p && !p.startsWith('.'))
      && name !== DEPLOY_PATH, `${label}: invalid content path ${name}`);
    requireValue(!result.has(name), `${label}: duplicate path ${name}`);
    requireValue(Number.isSafeInteger(row.bytes) && row.bytes >= 0, `${label}: invalid bytes for ${name}`);
    requireValue(isSha(row.sha256), `${label}: invalid sha256 for ${name}`);
    result.set(name, row);
  }
  return result;
}

function sameFiles(expected, actual, label) {
  requireValue(expected.size === actual.size && [...expected.keys()].every((name) => actual.has(name)), `${label}: file set differs`);
  for (const [name, row] of expected) {
    requireValue(row.bytes === actual.get(name).bytes, `${label}: bytes differ for ${name}`);
    requireValue(row.sha256 === actual.get(name).sha256, `${label}: sha256 differs for ${name}`);
  }
}

// Inputs contain already measured file hashes, never paths to read from metadata.
export function verifyXrTreeIntegrity({ files, deploy, currentIntegrity }) {
  const actual = fileMap(files, '', 'served tree');
  requireValue(deploy?.schema === 'slam-refact-xr-deploy-v1', 'deployment receipt: invalid schema');
  requireValue(deploy.bundle_digest_rule === DIGEST_RULE && isSha(deploy.bundle_sha256), 'deployment receipt: invalid digest rule or digest');
  const deploymentFiles = fileMap(deploy.files, '', 'deployment receipt');
  const hash = crypto.createHash('sha256');
  for (const name of [...actual.keys()].sort()) hash.update(`${name}\0${actual.get(name).sha256}\n`);
  const bundleSha256 = hash.digest('hex');
  let metadataKind = 'deployment-receipt', metadataPath = DEPLOY_PATH, expectedDigest = deploy.bundle_sha256;
  let expected = deploymentFiles;
  if (currentIntegrity !== undefined) {
    requireValue(currentIntegrity?.schema === 'slam-refact-runtime-integrity-v1', 'current integrity: invalid schema');
    requireValue(currentIntegrity.record_kind === 'source-artifact-index', 'current integrity: invalid record kind');
    requireValue(currentIntegrity.bundle_digest_rule === DIGEST_RULE && isSha(currentIntegrity.bundle_sha256), 'current integrity: invalid digest rule or digest');
    sameFiles(deploymentFiles, actual, 'artifact index');
    expected = fileMap(currentIntegrity.files, '', 'current integrity');
    expectedDigest = currentIntegrity.bundle_sha256;
    requireValue(expectedDigest === deploy.bundle_sha256, 'current integrity: artifact index digest differs');
    metadataKind = 'runtime-integrity';
    metadataPath = CURRENT_INTEGRITY_PATH;
  }
  sameFiles(expected, actual, metadataKind);
  requireValue(expectedDigest === bundleSha256, `${metadataKind}: bundle digest differs`);
  return { verified: true, metadataKind, metadataPath, files: actual.size, bundleSha256 };
}
