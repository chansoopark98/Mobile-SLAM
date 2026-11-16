// Experimental engine configurations of the phone page (opt-in query ?keepmap=1|fast). Pure: no DOM, so tests can check every case.
// BOTH configurations were REJECTED by the frozen gate of 2026-11-16 (build/xr-commercial-20261116/verdict-K1-final.json, verdict-K1F-final.json:
// G1 (primary improvement) passed; the no-regression gates G2, G3 and G4 failed; G5 was not measured). They are experimental opt-ins for on-device comparison and must not be made the default
// without a passing gate and the user's approval (CLAUDE.md 절대 규칙 2). The engine's own default stays OFF for every accuracy flag.
// This is the one place that names the flags. K1 = keep the map after tracking loss; K1F = K1 + fast extend.
export const EXPERIMENTAL_ENGINE_CONFIGS = Object.freeze({
  1: Object.freeze({ name: 'K1', accuracy: Object.freeze({ enabled: true, k1KeepMap: true }) }),
  fast: Object.freeze({ name: 'K1F', accuracy: Object.freeze({ enabled: true, k1KeepMap: true, k1FastExtend: true }) }),
});

// search = window.location.search -> { mode: 'default' | 'K1' | 'K1F', accuracy, error }. accuracy is a fresh copy to pass as engineConfig.accuracy,
// null in default mode. An unknown keepmap value sends nothing and returns an error text for the page's error display (never a silent fallback).
export function resolveEngineOptions(search = '') {
  const all = new URLSearchParams(search).getAll('keepmap');
  if (!all.length) return { mode: 'default', accuracy: null, error: null };
  // A repeated key is ambiguous: an error, never "first wins".
  const v = all.length === 1 ? all[0] : all.join(',');
  const c = all.length === 1 && Object.hasOwn(EXPERIMENTAL_ENGINE_CONFIGS, v) ? EXPERIMENTAL_ENGINE_CONFIGS[v] : null;
  if (!c) return { mode: 'default', accuracy: null, error: `keepmap 값 '${v.slice(0, 20)}'은(는) 알 수 없습니다 (1 또는 fast). 엔진 옵션 없이 실행합니다.` };
  return { mode: c.name, accuracy: { ...c.accuracy }, error: null };
}
