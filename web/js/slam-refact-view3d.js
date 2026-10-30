// 3D trajectory view of the slam_refact measurement page (web/slam-refact.html).
//
// Dependency-free (Canvas 2D + a small perspective camera), so the page needs no CDN and adds no
// runtime dependency. The engine publishes world_from_camera poses in a right-handed, y-up world
// with a GL camera (-Z forward, +Y up; PublishedFrame in
// slam_refact/engine/include/slam_refact/glue_runner.h). The ground of the responsive scale is
// y = 0 and the camera starts at the configured origin (0, 2, 0 by default; absolute scale
// publishes origin y = 1 and metres), so the grid is drawn at y = 0.
//
// Accuracy mode (engine ACC-3): the detected floor and further horizontal surfaces are drawn as
// filled hulls at their heights, and the last hit test result as a marker.
//
// Display only: nothing here feeds the engine or the metrics.
import { STATUS_COLORS, isPaddingPoint, quatRotate, trajectoryStats } from './slam-refact-core.js';

const MAX_POSES = 60000;
const MAX_POINTS = 4096;
const hex = (c) => `#${c.toString(16).padStart(6, '0')}`;

// Portrait (3:4) frustum in camera coordinates: apex, 4 image-plane corners, up-marker apex.
function frustumCorners(d) {
  const w = 0.375 * d, h = 0.5 * d;
  return [[0, 0, 0], [-w, -h, -d], [w, -h, -d], [w, h, -d], [-w, h, -d], [0, h * 1.45, -d]];
}
const FRUSTUM_EDGES = [[0, 1], [0, 2], [0, 3], [0, 4], [1, 2], [2, 3], [3, 4], [4, 1], [4, 5], [5, 3]];

export class TrajectoryView3D {
  constructor(canvas, { origin = [0, 2, 0] } = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.origin = origin;
    this.mode = 'orbit';
    // Orbit camera around target: yaw about +Y, pitch above the ground plane, distance.
    this.target = [origin[0], origin[1] * 0.5, origin[2]];
    this.yaw = 0.7;
    this.pitch = 0.55;
    this.distance = 6;
    this.fovY = 55 * Math.PI / 180;
    this.poses = [];            // { p:[x,y,z], q:[w,x,y,z], status, brk } fresh poses
    this.points = new Float32Array(MAX_POINTS * 4);   // x, y, z, confidence
    this.pointCount = 0;
    this._brk = true;
    this.surfaces = null;       // SlamRefactGlue.getSurfaces()
    this.hit = null;            // { type, position, distance }
    this._dirty = true;
    this._installInput();
    this.resize();
  }

  _installInput() {
    const c = this.canvas;
    const ptrs = new Map();
    let pinch0 = null;
    c.addEventListener('pointerdown', (e) => { c.setPointerCapture(e.pointerId); ptrs.set(e.pointerId, [e.clientX, e.clientY]); });
    const up = (e) => { ptrs.delete(e.pointerId); pinch0 = null; };
    c.addEventListener('pointerup', up);
    c.addEventListener('pointercancel', up);
    c.addEventListener('pointermove', (e) => {
      if (!ptrs.has(e.pointerId)) return;
      const [px, py] = ptrs.get(e.pointerId);
      ptrs.set(e.pointerId, [e.clientX, e.clientY]);
      if (ptrs.size === 1) {
        if (this.mode === 'follow') this.setMode('orbit');
        this.yaw -= (e.clientX - px) * 0.008;
        this.pitch = Math.max(-1.45, Math.min(1.55, this.pitch + (e.clientY - py) * 0.008));
      } else if (ptrs.size === 2) {
        const [a, b] = [...ptrs.values()];
        const d = Math.hypot(a[0] - b[0], a[1] - b[1]);
        if (pinch0) this.distance = Math.max(0.2, Math.min(500, this.distance * pinch0 / d));
        pinch0 = d;
      }
      this._dirty = true;
    });
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.distance = Math.max(0.2, Math.min(500, this.distance * Math.exp(e.deltaY * 0.001)));
      this._dirty = true;
    }, { passive: false });
  }

  resize() {
    const dpr = Math.min(globalThis.devicePixelRatio || 1, 2);
    const w = Math.max(1, Math.round((this.canvas.clientWidth || this.canvas.width) * dpr));
    const h = Math.max(1, Math.round((this.canvas.clientHeight || this.canvas.height) * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) { this.canvas.width = w; this.canvas.height = h; }
    this.dpr = dpr;
    this._dirty = true;
  }

  setMode(mode) {
    this.mode = mode;
    if (mode === 'top') { this.pitch = 1.55; this.fit(); }
    this._dirty = true;
  }

  // Frame the trajectory, the origin and the ground point under it, plus the map points near the
  // trajectory (within 3x its radius; far triangulations would shrink the path to a dot).
  fit() {
    const ps = [this.origin, [this.origin[0], 0, this.origin[2]], ...this.poses.map((r) => r.p)];
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    const grow = (p) => { for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], p[k]); hi[k] = Math.max(hi[k], p[k]); } };
    ps.forEach(grow);
    const c = [0, 1, 2].map((k) => (lo[k] + hi[k]) / 2);
    const reach = 3 * Math.max(0.5 * Math.hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]), 1.0);
    for (let i = 0; i < this.pointCount; i++) {
      const p = [this.points[i * 4], this.points[i * 4 + 1], this.points[i * 4 + 2]];
      if (Math.hypot(p[0] - c[0], p[1] - c[1], p[2] - c[2]) <= reach) grow(p);
    }
    this.target = [0, 1, 2].map((k) => (lo[k] + hi[k]) / 2);
    const r = Math.max(0.5 * Math.hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]), 1.0);
    // The narrower of the vertical and horizontal field of view (a phone canvas is portrait).
    const aspect = this.canvas.width / Math.max(1, this.canvas.height);
    const half = Math.min(this.fovY / 2, Math.atan(Math.tan(this.fovY / 2) * aspect));
    this.distance = 1.15 * r / Math.tan(half);
    this._dirty = true;
  }

  // r: page record { fresh, position:[x,y,z], rotationWxyz:[w,x,y,z], status }.
  addPose(r) {
    if (!r.fresh || !r.position || !r.rotationWxyz) return;
    if (![...r.position, ...r.rotationWxyz].every(Number.isFinite)) return;
    if (this.poses.length >= MAX_POSES) this.poses.shift();
    this.poses.push({ p: r.position.slice(), q: r.rotationWxyz.slice(), status: r.status, brk: this._brk });
    this._brk = false;
    this._dirty = true;
  }

  // Engine reset: keep the drawn history, but do not connect across the reset.
  breakTrajectory() { this._brk = true; }

  // flat: Float64Array [id, confidence, x, y, z] * n (SlamRefactGlue.getWorldPoints). Returns the
  // number of map points kept (padding entries are skipped).
  setWorldPoints(flat) {
    let n = 0;
    for (let k = 0; flat && k + 4 < flat.length && n < MAX_POINTS; k += 5) {
      if (isPaddingPoint(flat[k], flat[k + 1])) continue;
      this.points.set([flat[k + 2], flat[k + 3], flat[k + 4], flat[k + 1]], n * 4);
      n++;
    }
    this.pointCount = n;
    this._dirty = true;
    return n;
  }

  setSurfaces(s) { this.surfaces = s; this._dirty = true; }

  setHit(h) { this.hit = h; this._dirty = true; }

  clear() {
    this.poses = [];
    this.pointCount = 0;
    this.surfaces = null;
    this.hit = null;
    this._brk = true;
    this.target = [this.origin[0], this.origin[1] * 0.5, this.origin[2]];
    this.distance = 6;
    this._dirty = true;
  }

  stats() {
    const s = trajectoryStats(this.poses.map((r) => r.p), this.origin);
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < this.pointCount; i++) {
      for (let k = 0; k < 3; k++) { const v = this.points[i * 4 + k]; lo[k] = Math.min(lo[k], v); hi[k] = Math.max(hi[k], v); }
    }
    s.points = this.pointCount;
    s.pointBounds = this.pointCount ? [lo, hi] : null;
    return s;
  }

  // View basis of the orbit (or chase) camera; returns a projector world -> [x, y, depth].
  _camera() {
    let eye, look;
    const last = this.poses[this.poses.length - 1];
    if (this.mode === 'follow' && last) {
      // Behind (+Z of the device camera) and above the current pose, looking at it.
      const back = quatRotate(last.q, [0, 0, 1]);
      const bl = Math.hypot(back[0], back[2]) || 1;
      eye = [last.p[0] + 1.6 * back[0] / bl, last.p[1] + 0.9, last.p[2] + 1.6 * back[2] / bl];
      look = last.p;
    } else {
      const cp = Math.cos(this.pitch), sp = Math.sin(this.pitch);
      eye = [this.target[0] + this.distance * cp * Math.sin(this.yaw), this.target[1] + this.distance * sp,
        this.target[2] + this.distance * cp * Math.cos(this.yaw)];
      look = this.target;
    }
    const f = norm(sub(look, eye));
    let r = cross(f, [0, 1, 0]);
    if (Math.hypot(...r) < 1e-6) r = [Math.cos(this.yaw), 0, -Math.sin(this.yaw)];
    r = norm(r);
    const u = cross(r, f);
    const W = this.canvas.width, H = this.canvas.height;
    const fpx = (H / 2) / Math.tan(this.fovY / 2);
    return (p) => {
      const d = sub(p, eye);
      const z = dot(d, f);
      return [W / 2 + fpx * dot(d, r) / z, H / 2 - fpx * dot(d, u) / z, z];
    };
  }

  render() {
    if (!this._dirty && this.mode !== 'follow') return;
    this._dirty = false;
    const g = this.ctx, W = this.canvas.width, H = this.canvas.height, dpr = this.dpr;
    g.fillStyle = '#10131a';
    g.fillRect(0, 0, W, H);
    const P = this._camera();
    const near = 0.02;
    const seg = (a, b, style, width = 1) => {
      let pa = P(a), pb = P(b);
      if (pa[2] < near && pb[2] < near) return;
      if (pa[2] < near || pb[2] < near) {           // clip against the near plane
        const t = (near - pa[2]) / (pb[2] - pa[2]);
        const m = [0, 1, 2].map((k) => a[k] + t * (b[k] - a[k]));
        if (pa[2] < near) pa = P(m); else pb = P(m);
      }
      g.strokeStyle = style; g.lineWidth = width * dpr;
      g.beginPath(); g.moveTo(pa[0], pa[1]); g.lineTo(pb[0], pb[1]); g.stroke();
    };

    // Ground grid at y = 0 around the origin, spaced to the view distance.
    const step = gridStep(this.distance);
    const half = step * 10;
    const cx = Math.round(this.target[0] / step) * step, cz = Math.round(this.target[2] / step) * step;
    for (let i = -10; i <= 10; i++) {
      const s = i === 0 ? '#34414f' : '#222a33';
      seg([cx + i * step, 0, cz - half], [cx + i * step, 0, cz + half], s);
      seg([cx - half, 0, cz + i * step], [cx + half, 0, cz + i * step], s);
    }
    // World axes at the origin (x red, y green, z blue) and the drop line to the ground.
    const o = this.origin, a = step;
    seg(o, [o[0] + a, o[1], o[2]], '#e05555', 2);
    seg(o, [o[0], o[1] + a, o[2]], '#55c060', 2);
    seg(o, [o[0], o[1], o[2] + a], '#5580e0', 2);
    g.setLineDash([4 * dpr, 4 * dpr]);
    seg([o[0], 0, o[2]], o, '#5a6a7a');
    g.setLineDash([]);

    // Detected floor (teal) and further horizontal surfaces (violet): filled hull at the height.
    const hull = (sf, fill, stroke) => {
      const xz = sf.hullXz;
      if (!xz || xz.length < 6) return;
      const pts = [];
      for (let k = 0; k + 1 < xz.length; k += 2) {
        const q = P([xz[k], sf.height, xz[k + 1]]);
        if (q[2] < near) return;
        pts.push(q);
      }
      g.fillStyle = fill; g.strokeStyle = stroke; g.lineWidth = 1.5 * dpr;
      g.beginPath(); g.moveTo(pts[0][0], pts[0][1]);
      for (const q of pts.slice(1)) g.lineTo(q[0], q[1]);
      g.closePath(); g.fill(); g.stroke();
    };
    if (this.surfaces) {
      for (const sf of this.surfaces.others ?? []) hull(sf, 'rgba(171,71,188,0.16)', '#ab47bc');
      if (this.surfaces.floorDetected) hull(this.surfaces.floor, 'rgba(38,198,218,0.18)', '#26c6da');
    }

    // Map points, coloured by confidence (blue low -> yellow high).
    for (let i = 0; i < this.pointCount; i++) {
      const k = i * 4;
      const q = P([this.points[k], this.points[k + 1], this.points[k + 2]]);
      if (q[2] < near) continue;
      const c = Math.max(0, Math.min(1, this.points[k + 3]));
      g.fillStyle = `rgb(${Math.round(64 + 191 * c)},${Math.round(115 + 115 * c)},${Math.round(255 - 204 * c)})`;
      g.fillRect(q[0] - 1.5 * dpr, q[1] - 1.5 * dpr, 3 * dpr, 3 * dpr);
    }

    // Ground shadow, then the trajectory coloured by status: one stroked path per run of segments
    // of the same style (a long run has tens of thousands of segments).
    const polyline = (point, styleOf, width) => {
      let style = null;
      for (let i = 1; i < this.poses.length; i++) {
        const b = this.poses[i];
        if (b.brk) continue;
        let pa = P(point(this.poses[i - 1].p)), pb = P(point(b.p));
        if (pa[2] < near || pb[2] < near) {      // rare: clip this segment on its own
          if (style !== null) { g.stroke(); style = null; }
          seg(point(this.poses[i - 1].p), point(b.p), styleOf(b), width);
          continue;
        }
        const st = styleOf(b);
        if (st !== style) {
          if (style !== null) g.stroke();
          style = st; g.strokeStyle = st; g.lineWidth = width * dpr; g.beginPath();
        }
        g.moveTo(pa[0], pa[1]); g.lineTo(pb[0], pb[1]);
      }
      if (style !== null) g.stroke();
    };
    polyline((p) => [p[0], 0, p[2]], () => '#3a4b5c', 1);
    polyline((p) => p, (b) => hex(STATUS_COLORS[b.status] ?? STATUS_COLORS.UNSPECIFIED), 2);
    // Start marker and current camera frustum.
    const s0 = P(o);
    if (s0[2] > near) { g.fillStyle = '#ffffff'; g.beginPath(); g.arc(s0[0], s0[1], 3.5 * dpr, 0, 2 * Math.PI); g.fill(); }
    const last = this.poses[this.poses.length - 1];
    if (last) {
      const corners = frustumCorners(Math.max(0.12, step * 0.6)).map((c) => add(last.p, quatRotate(last.q, c)));
      const style = hex(STATUS_COLORS[last.status] ?? STATUS_COLORS.UNSPECIFIED);
      for (const [i, j] of FRUSTUM_EDGES) seg(corners[i], corners[j], style, 1.5);
    }
    // Hit test result: marker cross and the ray from the current camera.
    if (this.hit && Array.isArray(this.hit.position)) {
      const h = this.hit.position, r = Math.max(0.05, step * 0.25);
      seg([h[0] - r, h[1], h[2]], [h[0] + r, h[1], h[2]], '#ff4081', 2);
      seg([h[0], h[1], h[2] - r], [h[0], h[1], h[2] + r], '#ff4081', 2);
      seg([h[0], h[1], h[2]], [h[0], h[1] + r, h[2]], '#ff4081', 2);
      if (last) { g.setLineDash([3 * dpr, 3 * dpr]); seg(last.p, h, 'rgba(255,64,129,0.6)'); g.setLineDash([]); }
    }
    g.fillStyle = '#8a9aa8';
    g.font = `${11 * dpr}px ui-monospace, monospace`;
    const floorText = this.surfaces && this.surfaces.floorDetected ? ` · floor y ${this.surfaces.floor.height.toFixed(2)}` : '';
    g.fillText(`grid ${fmtStep(step)} · ${this.mode}${floorText}`, 8 * dpr, H - 8 * dpr - 30 * dpr);
  }
}

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => { const l = Math.hypot(...a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };

// 1-2-5 grid spacing so about 10 cells span the view.
export function gridStep(distance) {
  const raw = Math.max(distance / 6, 1e-3);
  const e = Math.pow(10, Math.floor(Math.log10(raw)));
  const m = raw / e;
  return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * e;
}
const fmtStep = (s) => (s >= 1 ? s.toFixed(0) : s.toPrecision(1));
