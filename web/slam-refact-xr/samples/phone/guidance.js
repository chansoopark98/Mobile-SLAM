// Tracking state -> what to tell the user and what to do next. Pure: no DOM, so tests can check every case.
// Inputs are what the page really observes (reality.trackingStatus / trackingReason, world point count, time since start); the progress bar
// shows the point count against a display scale only. It is a guide for the user, not a statement about how many points the engine needs.
export const POINTS_FULL_BAR = 40;      // display scale of the progress bar
export const SLOW_INIT_MS = 12000;      // after this long without tracking, suggest the recenter tool

export const STATUS_LABEL = Object.freeze({ normal: '추적 중', searching: '주변을 찾는 중', idle: '준비 중', error: '시작할 수 없음', stopped: '종료됨' });

// -> { tone: 'wait' | 'warn' | 'ok' | 'error', pill: keyof STATUS_LABEL, title, body, progress: number | null }
export function trackingGuide({ phase, status, reason, points = 0, objectCount = 0, landscape = false, notTrackingMs = 0 }) {
  if (phase === 'error') return { tone: 'error', pill: 'error', title: '시작하지 못했어요', body: '권한과 연결을 확인한 뒤 다시 시작해 주세요.', progress: null };
  if (phase === 'stopped') return { tone: 'wait', pill: 'stopped', title: '카메라를 껐어요', body: '도구에서 다시 시작할 수 있어요.', progress: null };
  if (status === 'NORMAL') {
    return { tone: 'ok', pill: 'normal', progress: 1,
      title: objectCount ? `물체 ${objectCount}개를 놓았어요` : '준비됐어요',
      body: objectCount ? '다시 탭하면 하나 더 놓아요. 지우려면 도구를 열어 주세요.' : '놓고 싶은 곳을 탭해 보세요. 바닥이나 책상처럼 무늬가 있는 면이 잘 잡혀요.' };
  }
  const pill = status === 'LIMITED' ? 'searching' : 'idle';
  const progress = status === 'LIMITED' && points > 0 ? Math.min(0.95, points / POINTS_FULL_BAR) : null;
  const slow = notTrackingMs >= SLOW_INIT_MS ? ' 오래 걸리면 도구에서 원점 다시 잡기를 눌러 보세요.' : '';
  if (landscape) return { tone: 'warn', pill, progress, title: '휴대폰을 세로로 돌려 주세요', body: '가로 화면은 아직 실기기에서 검증되지 않아 추적이 멈출 수 있어요.' };
  if (reason === 'EXCESSIVE_MOTION') return { tone: 'warn', pill, progress, title: '너무 빨리 움직였어요', body: `천천히 움직이면 다시 잡혀요. 잠시 멈췄다가 천천히 옮겨 보세요.${slow}` };
  if (reason === 'INSUFFICIENT_FEATURES') return { tone: 'warn', pill, progress, title: '인식할 무늬가 부족해요', body: `무늬나 모서리가 있는 면을 비추고, 더 밝은 곳으로 가 보세요. 흰 벽이나 유리는 어려워요.${slow}` };
  return { tone: 'wait', pill, progress, title: '주변을 살펴보는 중', body: `한 걸음 정도 천천히 옮기면서 바닥이나 책상을 비춰 주세요. 제자리에서 돌기만 하면 시작이 늦어질 수 있어요.${slow}` };
}

// Result of a tap -> hit test.
export function tapOutcome(ok, { objectCount = 0 } = {}) {
  if (ok) return { tone: 'ok', title: `물체를 놓았어요 (${objectCount}개)`, body: '다시 탭하면 하나 더 놓아요.' };
  return { tone: 'warn', title: '여기엔 놓을 수 없어요', body: '탭한 곳에서 기준점을 찾지 못했어요. 무늬가 있는 면을 비추고 휴대폰을 천천히 움직인 뒤 다시 탭해 주세요.' };
}
