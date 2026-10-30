/* Start gate of the slam_refact XR sample pages (three.js, A-Frame). Classic script, must be the first script of the page.
 *
 * The samples (slam_refact/web-chunk/samples/{threejs,aframe}) start XR8 as soon as they load. For the deployed tree this gate makes them
 * wait for a tap, without editing the samples:
 *  - getUserMedia is wrapped: the call (made by XR8.run) is held until the "start" button is tapped, then forwarded unchanged;
 *  - window devicemotion / deviceorientation listener registrations (the chunk makes them when xr.js loads) are held and registered on the tap.
 * Nothing is requested, stored or sent by this file. The phone page (slam-refact-xr.html) has its own start button and does not use it. */
(function () {
  'use strict';
  var SENSOR = { devicemotion: true, deviceorientation: true };
  var released = false, held = [], overlay = null, resolveTap;
  var tapped = new Promise(function (resolve) { resolveTap = resolve; });
  var add = window.addEventListener, remove = window.removeEventListener;
  window.addEventListener = function (type, fn, opts) {
    if (!released && SENSOR[type] === true) { held.push([type, fn, opts]); return; }
    return add.call(this, type, fn, opts);
  };
  window.removeEventListener = function (type, fn, opts) {
    if (!released && SENSOR[type] === true) { held = held.filter(function (h) { return !(h[0] === type && h[1] === fn); }); return; }
    return remove.call(this, type, fn, opts);
  };
  var md = navigator.mediaDevices;
  if (md && typeof md.getUserMedia === 'function') {
    var gum = md.getUserMedia.bind(md);
    md.getUserMedia = function () { var args = arguments; return tapped.then(function () { return gum.apply(null, args); }); };
  }
  function release() {
    if (released) return;
    released = true;
    var pending = held; held = [];
    pending.forEach(function (h) { add.call(window, h[0], h[1], h[2]); });
    if (overlay) overlay.remove();
    resolveTap();
  }
  function mount() {
    overlay = document.createElement('div');
    overlay.id = 'xr-gate';
    overlay.setAttribute('style', 'position:fixed;inset:0;z-index:2147483000;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:16px;padding:24px;box-sizing:border-box;background:rgba(0,0,0,.88);color:#fff;font:15px/1.4 system-ui,sans-serif;text-align:center');
    var p = document.createElement('p');
    p.textContent = '시작을 누르기 전에는 카메라와 모션 센서를 사용하지 않습니다. 시작하면 브라우저가 카메라 권한을 물어볼 수 있습니다. 이 페이지는 영상·센서 값을 저장하거나 서버로 보내지 않습니다.';
    p.style.maxWidth = '28em';
    var b = document.createElement('button');
    b.id = 'xr-gate-start'; b.type = 'button'; b.textContent = '시작';
    b.setAttribute('style', 'font:inherit;font-size:18px;font-weight:700;min-height:48px;min-width:160px;border:0;border-radius:8px;background:#4fc3f7;color:#00202f');
    b.addEventListener('click', release);
    var a = document.createElement('a');
    a.href = '../../../slam-refact-xr.html'; a.textContent = '← 진입 페이지로'; a.style.color = '#b8c0c8';
    overlay.append(p, b, a);
    document.body.append(overlay);
  }
  if (document.body) mount(); else document.addEventListener('DOMContentLoaded', mount, { once: true });
  window.__xrGate = { isReleased: function () { return released; } };
})();
