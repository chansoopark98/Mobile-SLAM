// DOM side of the phone page's guidance: status pill, guidance card, reticle, tap ring, start progress, recovery sheet.
// What to say comes from guidance.js; this file only writes it to the page (and skips writes when nothing changed).
import { STATUS_LABEL } from './guidance.js';

const START_STEPS = [['permission', '모션 센서 허용'], ['engine', '엔진 불러오기'], ['camera', '카메라 켜기']];

export function createHud(doc = document) {
  const $ = (id) => doc.getElementById(id);
  const el = {
    pill: $('h-status').parentElement, status: $('h-status'), reason: $('h-reason'), title: $('h-title'), msg: $('h-msg'), meter: $('h-meter'),
    card: doc.querySelector('.guidance'), reticle: $('reticle'), count: $('h-count'), countText: $('h-count-text'), fx: $('tap-fx'),
    startAction: doc.querySelector('.start-action'), start: $('start'), alert: $('start-alert'), alertTitle: $('start-alert-title'), alertBody: $('start-alert-body'),
    recovery: $('recovery'), recTitle: $('rec-title'), recBody: $('rec-body'), recDetail: $('rec-detail'), recPrimary: $('rec-primary'),
  };
  let shown = '', holdUntil = 0, holdTimer = null, lastGuide = null, progressList = null;

  const paint = (g) => {
    const key = `${g.tone}|${g.title}|${g.body}|${g.pill}|${g.progress === null ? 'i' : Math.round(g.progress * 50)}`;
    if (key === shown) return;
    shown = key;
    el.status.textContent = STATUS_LABEL[g.pill] || STATUS_LABEL.idle;
    el.pill.dataset.state = g.pill === 'normal' ? 'normal' : g.pill === 'error' ? 'error' : 'pending';
    el.title.textContent = g.title;
    el.msg.textContent = g.body;
    el.card.dataset.tone = g.tone;
    el.reticle.dataset.tone = g.tone;
    el.meter.hidden = g.tone === 'ok' || g.tone === 'error' || g.pill === 'stopped';
    if (g.progress === null) { el.meter.dataset.indeterminate = 'true'; el.meter.removeAttribute('aria-valuenow'); }
    else { el.meter.dataset.indeterminate = 'false'; el.meter.setAttribute('aria-valuenow', String(Math.round(g.progress * 100))); el.meter.firstElementChild.style.setProperty('--p', String(g.progress)); }
  };

  return {
    el,
    // Tracking guide; a transient message (tap result) stays until its hold ends, then the latest guide comes back.
    render(g) { lastGuide = g; if (Date.now() >= holdUntil) paint(g); },
    flash(outcome, ms = 3500) {
      holdUntil = Date.now() + ms;
      paint({ ...outcome, pill: outcome.pill || (lastGuide ? lastGuide.pill : 'normal'), progress: 1 });
      el.meter.hidden = true;
      clearTimeout(holdTimer);
      holdTimer = setTimeout(() => { holdUntil = 0; if (lastGuide) paint(lastGuide); }, ms);
    },
    setCount(n) { el.count.hidden = n === 0; el.countText.textContent = `물체 ${n}`; },
    setReason(reason) { el.reason.textContent = reason && reason !== 'UNSPECIFIED' ? `(${reason})` : ''; },
    tapRing(x, y, ok) {
      const ring = doc.createElement('div');
      ring.className = 'tap-ring'; ring.dataset.ok = String(ok);
      ring.style.left = `${x}px`; ring.style.top = `${y}px`;
      el.fx.append(ring);
      ring.addEventListener('animationend', () => ring.remove(), { once: true });
      setTimeout(() => ring.remove(), 1200);  // animations can be switched off (reduced motion): do not leave rings behind
      try { if (typeof navigator.vibrate === 'function') navigator.vibrate(ok ? 14 : [10, 40, 10]); } catch (_) { /* vibration is optional (not on iOS) */ }
    },
    // Start screen: which step the Start tap is on.
    startStep(name) {
      if (!progressList) { progressList = doc.createElement('ul'); progressList.className = 'start-progress'; progressList.setAttribute('aria-label', '시작 진행'); el.startAction.insertBefore(progressList, el.start); }
      const at = START_STEPS.findIndex(([n]) => n === name);
      progressList.replaceChildren(...START_STEPS.map(([, label], i) => { const li = doc.createElement('li'); li.textContent = label; li.dataset.state = i < at ? 'done' : i === at ? 'active' : 'todo'; return li; }));
    },
    clearStartSteps() { if (progressList) { progressList.remove(); progressList = null; } },
    startAlert(info, tone) {
      el.alert.hidden = !info;
      if (!info) return;
      el.alert.dataset.tone = tone; el.alertTitle.textContent = info.title; el.alertBody.textContent = info.body;
    },
    showRecovery(info, detail) {
      el.recTitle.textContent = info.title; el.recBody.textContent = info.body;
      el.recDetail.textContent = detail || '';
      el.recDetail.parentElement.hidden = !detail;
      el.recovery.hidden = false;
      el.recPrimary.focus();
    },
    hideRecovery() { el.recovery.hidden = true; },
  };
}
