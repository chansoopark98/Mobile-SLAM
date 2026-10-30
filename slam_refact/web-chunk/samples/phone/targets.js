// Image target loading for the phone page. Two sources, both same-origin or user-picked local files (no network beyond this origin):
//   1. bundled demo targets listed in ./targets/manifest.json (package C fills the directory: <name>.json + <name>.png/.jpg),
//   2. a JSON file and an image file the user picks on the phone.
// The result is an `imageTargetData` array for XR8.XrController.configure. The page never sends a target anywhere.
export const MAX_TARGETS = 32;  // the chunk rejects more than MAX_ACTIVE_IMAGE_TARGETS (image-targets.js)
const FILE_RE = /^[A-Za-z0-9][A-Za-z0-9_.-]*$/;

export const isSafeFileName = (s) => typeof s === 'string' && FILE_RE.test(s) && !s.includes('..');

export function parseManifest(obj) {
  if (!obj || obj.schema !== 'phone-targets-v1' || !Array.isArray(obj.targets)) throw new Error('targets/manifest.json 형식이 맞지 않습니다');
  return obj.targets.map((t, i) => {
    if (!t || typeof t.name !== 'string' || !t.name || !isSafeFileName(t.json) || !isSafeFileName(t.image)) throw new Error(`targets/manifest.json ${i}번째 항목이 올바르지 않습니다`);
    return { name: t.name, label: typeof t.label === 'string' ? t.label : t.name, json: t.json, image: t.image };
  });
}

// target JSON (reference runtime image target file) + the URL the image can be loaded from -> one imageTargetData entry.
export function buildTargetEntry(json, imageUrl, fallbackName) {
  if (!json || typeof json !== 'object' || Array.isArray(json)) throw new Error('타깃 JSON이 객체가 아닙니다');
  const name = typeof json.name === 'string' && json.name ? json.name : fallbackName;
  if (!name) throw new Error('타깃 JSON에 name이 없습니다');
  const type = json.type ?? 'PLANAR';
  // Curved targets are passed on: whether the pinned engine accepts them is the engine's answer, shown as a chunk error (US-014 / package F).
  if (!['PLANAR', 'CYLINDER', 'CONICAL'].includes(String(type).toUpperCase())) throw new Error(`"${name}": ${type} 타깃은 이 앱에서 지원하지 않습니다(PLANAR/CYLINDER/CONICAL)`);
  return { ...json, name, type, imagePath: imageUrl };
}

export async function loadBundledManifest(fetchFn = fetch, base = new URL('./targets/', document.baseURI)) {
  const res = await fetchFn(new URL('manifest.json', base), { cache: 'no-store' });
  if (!res.ok) throw new Error(`targets/manifest.json: HTTP ${res.status}`);
  return parseManifest(await res.json());
}

export async function loadBundledTarget(item, fetchFn = fetch, base = new URL('./targets/', document.baseURI)) {
  const jsonUrl = new URL(item.json, base), imageUrl = new URL(item.image, base);
  if (jsonUrl.origin !== location.origin || imageUrl.origin !== location.origin) throw new Error('외부 주소의 타깃은 읽지 않습니다');
  const res = await fetchFn(jsonUrl, { cache: 'no-store' });
  if (!res.ok) throw new Error(`${item.json}: HTTP ${res.status}`);
  return buildTargetEntry(await res.json(), imageUrl.href, item.name);
}

// Files the user picked: the image becomes a blob: URL (kept alive while the target is active; revoke() frees it).
export async function loadPickedTarget(jsonFile, imageFile) {
  if (!jsonFile || !imageFile) throw new Error('JSON 파일과 이미지 파일을 모두 선택하세요');
  if (!/^image\//.test(imageFile.type)) throw new Error('이미지 파일이 아닙니다');
  let json;
  try { json = JSON.parse(await jsonFile.text()); } catch (e) { throw new Error('JSON을 읽지 못했습니다: ' + e.message); }
  const url = URL.createObjectURL(imageFile);
  try { return { entry: buildTargetEntry(json, url, jsonFile.name.replace(/\.json$/i, '')), revoke: () => URL.revokeObjectURL(url) }; }
  catch (e) { URL.revokeObjectURL(url); throw e; }
}
