// The in-page push simulation: a side view of the UR5e, the controls and a live chart.
// Physics: core.js (500 Hz). Drawing: canvas 2D, nothing else to load.
import { PushSim, SETTINGS, LIMITS, gripImpedance, DT } from "./core.js";

const C = {
  ink: "#0b0b0b", ink2: "#52514e", muted: "#898781", grid: "#e1e0d9", axis: "#c3c2b7",
  blue: "#2a78d6", orange: "#eb6834", critical: "#d03b3b",
  housing: "#3a3c3f", housingHi: "#5b5e63", cap: "#6c9dd1", capHi: "#a9c9ea",
  tubeLo: "#9da1a7", tubeHi: "#f1f2f4", floor: "#f1f0ec",
};
const START = [0.525, 0.207];          // tool flange at pose A, in the arm plane (reach, height) [m]
const SHOULDER = [0, 0.1625], L1 = 0.425, L2 = 0.3922;
const WRIST1 = [-0.0997, 0.0996];      // wrist 1 axis relative to the flange (tool pointing down)
const VIEW = { u0: -0.17, u1: 0.80, z0: -0.035, z1: 0.70 };
const FORCE_M_PER_N = 0.008;          // force arrow: metres per newton in the scene (capped at 30 N)
const KEY_FORCE = 12;                  // N, arrow keys

function lerp(a, b, t) { return a + (b - a) * t; }

export function mount(root) {
  const stage = root.querySelector("[data-stage]");
  const cv = root.querySelector("canvas[data-robot]");
  const ch = root.querySelector("canvas[data-chart]");
  const chip = root.querySelector("[data-chip]");
  const hint = root.querySelector("[data-hint]");
  const out = {
    force: root.querySelector("[data-out=force]"), speed: root.querySelector("[data-out=speed]"),
    dev: root.querySelector("[data-out=dev]"), params: root.querySelector("[data-params]"),
    grip: root.querySelector("[data-out=grip]"),
  };
  const gripIn = root.querySelector("input[data-grip]");
  const sim = new PushSim();
  const ctx = cv.getContext("2d"), cctx = ch.getContext("2d");
  let s = 1, dpr = 1, W = 0, H = 0, CW = 0, CH = 0;
  let drag = null, keys = new Set(), oscOn = false, oscT = 0, oscHz = 0, lastChip = "";
  const trail = [];

  // ---- settings ---------------------------------------------------------------------------
  function setSetting(key) {
    sim.setSetting(key);
    for (const b of root.querySelectorAll("[data-setting]")) b.setAttribute("aria-pressed", String(b.dataset.setting === key));
    const c = SETTINGS[key];
    out.params.textContent = `M ${c.M} kg · D ${c.D.toFixed(1)} N·s/m · K ${LIMITS.K} N/m · F ${c.F} N` +
      (c.notch ? ` · notch ${c.notch[0]} Hz` : "");
  }
  function setGrip() {
    const g = gripIn.value / 100, h = gripImpedance(g);
    sim.setGrip(g);
    const word = g < 0.25 ? "light" : g < 0.7 ? "medium" : "firm";
    out.grip.textContent = `${word} · ${h.c.toFixed(0)} N·s/m, ${(h.k / 1000).toFixed(1)} kN/m`;
  }
  for (const b of root.querySelectorAll("[data-setting]")) b.addEventListener("click", () => setSetting(b.dataset.setting));
  gripIn.addEventListener("input", setGrip);
  root.querySelector("[data-reset]").addEventListener("click", () => { sim.reset(); trail.length = 0; oscOn = false; });
  setSetting("m2");
  setGrip();

  // ---- geometry ---------------------------------------------------------------------------
  const px = (u, z) => [(u - VIEW.u0) * s, (VIEW.z1 - z) * s];
  function resize() {
    dpr = Math.min(2, window.devicePixelRatio || 1);
    W = cv.clientWidth; H = Math.round(W * (VIEW.z1 - VIEW.z0) / (VIEW.u1 - VIEW.u0));
    cv.style.height = H + "px";
    cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr);
    s = W / (VIEW.u1 - VIEW.u0);
    CW = ch.clientWidth; CH = ch.clientHeight;
    ch.width = Math.round(CW * dpr); ch.height = Math.round(CH * dpr);
  }
  function placeHint() {
    const [x, y] = px(START[0], START[1] - 0.09);
    hint.style.left = `${x}px`; hint.style.top = `${y + 18}px`;
  }
  new ResizeObserver(() => { resize(); placeHint(); }).observe(stage);
  resize();
  placeHint();

  // ---- input ------------------------------------------------------------------------------
  cv.addEventListener("pointerdown", (e) => {
    if (e.button !== 0 && e.pointerType === "mouse") return;
    try { cv.setPointerCapture(e.pointerId); } catch { /* synthetic events */ }
    sim.grab();
    drag = { id: e.pointerId, x0: e.clientX, y0: e.clientY, start: sim.x.slice(), x: e.clientX, y: e.clientY };
    hint.hidden = true;
    cv.classList.add("grabbing");
    e.preventDefault();
  });
  cv.addEventListener("pointermove", (e) => { if (drag && e.pointerId === drag.id) { drag.x = e.clientX; drag.y = e.clientY; } });
  const end = (e) => {
    if (!drag || e.pointerId !== drag.id) return;
    drag = null; sim.release(); cv.classList.remove("grabbing");
  };
  cv.addEventListener("pointerup", end);
  cv.addEventListener("pointercancel", end);
  const DIRS = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, 1], ArrowDown: [0, -1] };
  cv.addEventListener("keydown", (e) => { if (DIRS[e.key]) { keys.add(e.key); hint.hidden = true; e.preventDefault(); } });
  cv.addEventListener("keyup", (e) => { keys.delete(e.key); });
  cv.addEventListener("blur", () => keys.clear());
  function keyForce() {
    if (!keys.size) return null;
    const v = [0, 0];
    for (const k of keys) { v[0] += DIRS[k][0]; v[1] += DIRS[k][1]; }
    const n = Math.hypot(...v);
    return n ? v.map((c) => c * KEY_FORCE / n) : null;
  }

  // ---- drawing: robot -----------------------------------------------------------------------
  function tube(a, b, width, lo = C.tubeLo, hi = C.tubeHi) {
    const [ax, ay] = px(...a), [bx, by] = px(...b), w = width * s;
    const ang = Math.atan2(by - ay, bx - ax), len = Math.hypot(bx - ax, by - ay);
    ctx.save();
    ctx.translate(ax, ay); ctx.rotate(ang);
    const g = ctx.createLinearGradient(0, -w / 2, 0, w / 2);
    g.addColorStop(0, lo); g.addColorStop(0.35, hi); g.addColorStop(1, lo);
    ctx.fillStyle = g;
    ctx.fillRect(0, -w / 2, len, w);
    ctx.restore();
  }
  function housingRect(u0, z0, u1, z1, dir = "h") {
    const [x0, y0] = px(u0, z1), [x1, y1] = px(u1, z0);
    const g = dir === "h" ? ctx.createLinearGradient(0, y0, 0, y1) : ctx.createLinearGradient(x0, 0, x1, 0);
    g.addColorStop(0, C.housingHi); g.addColorStop(0.45, C.housing); g.addColorStop(1, "#2b2d30");
    ctx.fillStyle = g;
    roundRect(x0, y0, x1 - x0, y1 - y0, Math.min(6, (x1 - x0) / 4, (y1 - y0) / 4));
    ctx.fill();
  }
  function roundRect(x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath();
  }
  function joint(c, r, capR) {
    const [x, y] = px(...c);
    let g = ctx.createRadialGradient(x - r * s * 0.3, y - r * s * 0.35, r * s * 0.1, x, y, r * s);
    g.addColorStop(0, C.housingHi); g.addColorStop(1, C.housing);
    ctx.fillStyle = g; ctx.beginPath(); ctx.arc(x, y, r * s, 0, 2 * Math.PI); ctx.fill();
    g = ctx.createRadialGradient(x - capR * s * 0.35, y - capR * s * 0.4, capR * s * 0.1, x, y, capR * s);
    g.addColorStop(0, C.capHi); g.addColorStop(1, C.cap);
    ctx.fillStyle = g; ctx.beginPath(); ctx.arc(x, y, capR * s, 0, 2 * Math.PI); ctx.fill();
    ctx.strokeStyle = "rgba(0,0,0,.18)"; ctx.lineWidth = 1; ctx.stroke();
  }
  function elbowFor(w) {  // elbow-up planar IK, shoulder -> wrist 1
    const d = Math.hypot(w[0] - SHOULDER[0], w[1] - SHOULDER[1]);
    const dd = Math.min(L1 + L2 - 1e-6, Math.max(Math.abs(L1 - L2) + 1e-6, d));
    const th = Math.atan2(w[1] - SHOULDER[1], w[0] - SHOULDER[0]);
    const b = Math.acos((L1 * L1 + dd * dd - L2 * L2) / (2 * L1 * dd));
    return [SHOULDER[0] + L1 * Math.cos(th + b), SHOULDER[1] + L1 * Math.sin(th + b)];
  }
  function drawRobot(T) {
    const W1 = [T[0] + WRIST1[0], T[1] + WRIST1[1]];
    const E = elbowFor(W1);
    // base and shoulder column
    housingRect(-0.08, 0, 0.08, 0.035, "v");
    ctx.fillStyle = C.cap; { const [x0, y0] = px(-0.076, 0.041), [x1, y1] = px(0.076, 0.035); ctx.fillRect(x0, y0, x1 - x0, y1 - y0); }
    housingRect(-0.066, 0.041, 0.066, 0.17, "v");
    tube(SHOULDER, E, 0.094);
    joint(SHOULDER, 0.074, 0.055);
    tube(E, W1, 0.076);
    joint(E, 0.062, 0.046);
    // wrist 2 housing (axis along the reach) and wrist 3 housing (axis vertical) with the flange
    const W5 = [T[0], W1[1]];
    housingRect(W1[0], W5[1] - 0.046, W5[0] + 0.046, W5[1] + 0.046, "h");
    { const [x0, y0] = px(W5[0] + 0.034, W5[1] + 0.044), [x1, y1] = px(W5[0] + 0.048, W5[1] - 0.044);
      const g = ctx.createLinearGradient(0, y0, 0, y1); g.addColorStop(0, C.capHi); g.addColorStop(1, C.cap);
      ctx.fillStyle = g; roundRect(x0, y0, x1 - x0, y1 - y0, 3); ctx.fill(); }
    housingRect(T[0] - 0.045, T[1] + 0.009, T[0] + 0.045, W5[1] - 0.02, "v");
    joint(W1, 0.047, 0.035);
    // flange and a short handle
    { const [x0, y0] = px(T[0] - 0.032, T[1] + 0.009), [x1, y1] = px(T[0] + 0.032, T[1]);
      ctx.fillStyle = "#c9ccd0"; ctx.fillRect(x0, y0, x1 - x0, y1 - y0); }
    tube([T[0], T[1]], [T[0], T[1] - 0.07], 0.034, "#b8bbbf", "#eceef0");
    { const [x0, y0] = px(T[0] - 0.017, T[1] - 0.07);
      ctx.fillStyle = "#9da1a7"; roundRect(x0, y0 - 2, 0.034 * s, 4, 2); ctx.fill(); }
  }

  function arrow(from, vec, color, width) {
    const len = Math.hypot(...vec);
    if (len < 2) return;
    const [x, y] = from, ux = vec[0] / len, uy = vec[1] / len, head = Math.min(12, len * 0.6) + 2;
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width; ctx.lineCap = "round";
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + ux * (len - head * 0.8), y + uy * (len - head * 0.8)); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x + ux * len, y + uy * len);
    ctx.lineTo(x + ux * (len - head) - uy * head * 0.45, y + uy * (len - head) + ux * head * 0.45);
    ctx.lineTo(x + ux * (len - head) + uy * head * 0.45, y + uy * (len - head) - ux * head * 0.45);
    ctx.closePath(); ctx.fill();
  }

  function drawScene() {
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    // floor
    const [, yf] = px(0, 0);
    ctx.fillStyle = C.floor; ctx.fillRect(0, yf, W, H - yf);
    ctx.strokeStyle = C.axis; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(0, yf + 0.5); ctx.lineTo(W, yf + 0.5); ctx.stroke();
    // workspace box
    const B = LIMITS.BOX;
    const [bx0, by0] = px(START[0] + B.x[0], START[1] + B.z[1]), [bx1, by1] = px(START[0] + B.x[1], START[1] + B.z[0]);
    ctx.setLineDash([4, 4]); ctx.strokeStyle = C.axis; ctx.lineWidth = 1;
    ctx.strokeRect(bx0 + 0.5, by0 + 0.5, bx1 - bx0, by1 - by0); ctx.setLineDash([]);
    // trail
    const T = [START[0] + sim.x[0], START[1] + sim.x[1]];
    if (trail.length > 1) {
      ctx.lineWidth = 2; ctx.lineCap = "round"; ctx.lineJoin = "round";
      for (let i = 1; i < trail.length; i++) {
        ctx.strokeStyle = `rgba(42,120,214,${0.05 + 0.25 * i / trail.length})`;
        const [x0, y0] = px(...trail[i - 1]), [x1, y1] = px(...trail[i]);
        ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
      }
    }
    drawRobot(T);
    ctx.fillStyle = C.muted; ctx.font = `12px ${FONT}`; ctx.textBaseline = "top"; ctx.textAlign = "right";
    ctx.fillText("workspace", bx1, by1 + 4); ctx.textAlign = "left";
    // reference and the virtual spring (to the commanded position)
    const [rx, ry] = px(START[0], START[1] - 0.04), [cx, cy] = px(START[0] + sim.dx[0], START[1] + sim.dx[1] - 0.04);
    if (Math.hypot(cx - rx, cy - ry) > 3) {
      ctx.strokeStyle = C.blue; ctx.lineWidth = 1.5; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(cx, cy); ctx.stroke(); ctx.setLineDash([]);
    }
    ctx.fillStyle = C.blue; ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(rx, ry, 5, 0, 2 * Math.PI); ctx.stroke(); ctx.fill();
    // hand force on the handle
    const grip = px(T[0], T[1] - 0.04);
    const F = sim.F;
    const nF = Math.hypot(...F);
    if (nF > 0.3) {
      const k = FORCE_M_PER_N * s * Math.min(1, 30 / nF);
      arrow(grip, [F[0] * k, -F[1] * k], C.orange, 3);
      ctx.fillStyle = C.orange; ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(...grip, 6, 0, 2 * Math.PI); ctx.stroke(); ctx.fill();
    }
    // the loop is oscillating: vibration marks around the handle
    if (oscOn) {
      ctx.strokeStyle = C.critical; ctx.lineWidth = 2; ctx.lineCap = "round";
      for (const side of [-1, 1]) for (let i = 0; i < 3; i++) {
        const r = 0.03 * s + i * 6;
        ctx.beginPath();
        ctx.arc(grip[0], grip[1], r, side > 0 ? -0.5 : Math.PI - 0.5, side > 0 ? 0.5 : Math.PI + 0.5);
        ctx.stroke();
      }
    }
  }

  // ---- drawing: chart -----------------------------------------------------------------------
  function panel(y0, h, data, k, max, color, title, limit, limitLabel) {
    const x0 = 34, x1 = CW - 8, w = x1 - x0, hist = sim.hist, N = data.length;
    cctx.font = `12px ${FONT}`; cctx.textBaseline = "alphabetic";
    cctx.fillStyle = C.ink2; cctx.fillText(title, x0, y0 + 10);
    const top = y0 + 24, bot = y0 + h, Y = (v) => bot - Math.min(1, v / max) * (bot - top);
    cctx.strokeStyle = C.grid; cctx.lineWidth = 1; cctx.fillStyle = C.muted; cctx.textAlign = "right"; cctx.textBaseline = "middle";
    for (const t of [0, max / 2, max]) {
      const y = Math.round(Y(t)) + 0.5;
      cctx.beginPath(); cctx.moveTo(x0, y); cctx.lineTo(x1, y); cctx.stroke();
      cctx.fillText(String(t), x0 - 6, y);
    }
    cctx.textAlign = "left";
    if (limit != null) {
      const y = Math.round(Y(limit)) + 0.5;
      cctx.strokeStyle = C.muted; cctx.setLineDash([4, 3]);
      cctx.beginPath(); cctx.moveTo(x0, y); cctx.lineTo(x1, y); cctx.stroke();
      // key in the title row, right-aligned: the trace never covers it
      cctx.textAlign = "right"; cctx.textBaseline = "alphabetic"; cctx.fillStyle = C.muted;
      const tw = cctx.measureText(limitLabel).width;
      cctx.fillText(limitLabel, x1, y0 + 10);
      cctx.beginPath(); cctx.moveTo(x1 - tw - 26, y0 + 6.5); cctx.lineTo(x1 - tw - 6, y0 + 6.5); cctx.stroke();
      cctx.setLineDash([]); cctx.textAlign = "left";
    }
    // min / max per pixel column, so an oscillation shows as a band
    const n = hist.n, cols = Math.max(1, Math.floor(w));
    cctx.strokeStyle = color; cctx.lineWidth = 1.6; cctx.lineJoin = "round";
    cctx.beginPath();
    let first = true;
    for (let c = 0; c < cols; c++) {
      const a = Math.floor(c * N / cols), b = Math.floor((c + 1) * N / cols);
      if (b <= N - n) continue;
      let lo = Infinity, hi = -Infinity;
      for (let j = Math.max(a, N - n); j < b; j++) { const v = k * data[(hist.i + j) % N]; if (v < lo) lo = v; if (v > hi) hi = v; }
      const x = x0 + c + 0.5;
      if (first) { cctx.moveTo(x, Y(lo)); first = false; } else cctx.lineTo(x, Y(lo));
      if (hi - lo > 0) cctx.lineTo(x, Y(hi));
    }
    cctx.stroke();
  }
  function drawChart() {
    cctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    cctx.clearRect(0, 0, CW, CH);
    const h = (CH - 22) / 2;
    const c = SETTINGS[sim.key];
    panel(0, h - 8, sim.hist.force, 1, 24, C.orange, "Hand force [N]", c.F, `spring force limit ${c.F} N`);
    panel(h + 8, h - 8, sim.hist.speed, 1000, 120, C.blue, "Tool speed [mm/s]", 1000 * LIMITS.V_MAX, "speed limit");
    cctx.fillStyle = C.muted; cctx.font = `12px ${FONT}`; cctx.textBaseline = "bottom"; cctx.textAlign = "right";
    cctx.fillText("last 5 s → now", CW - 8, CH);
    cctx.textAlign = "left";
  }

  // ---- status -------------------------------------------------------------------------------
  function mean(arr, k) {
    const h = sim.hist, N = arr.length, m = Math.min(k, h.n);
    let sum = 0;
    for (let j = 1; j <= m; j++) sum += arr[(h.i - j + N) % N];
    return m ? sum / m : 0;
  }
  function status(dt) {
    const r = sim.oscRms;
    if (!oscOn && r > 0.45) { oscT += dt; if (oscT > 0.25) { oscOn = true; oscT = 0; } }
    else if (oscOn && r < 0.25) { oscT += dt; if (oscT > 0.6) { oscOn = false; oscT = 0; } }
    else oscT = 0;
    if (oscOn) oscHz = oscHz ? lerp(oscHz, sim.oscHz, 0.05) : sim.oscHz;
    const f = sim.flags, dev = Math.hypot(...sim.x), c = SETTINGS[sim.key];
    if (oscOn) return ["bad", `Oscillating at about ${Math.round(oscHz)} Hz: this grip and the link mode close an unstable loop`];
    if (sim.holding || keys.size) {
      if (f.box) return ["", "At the workspace limit"];
      if (f.speed) return ["", "At the speed limit (100 mm/s)"];
      if (f.limit) return ["", `Pushing: the spring is capped at ${c.F} N, the rest is D·v + M·a`];
      return ["", "Pushing: the spring is still in its linear range"];
    }
    if (dev > 0.001) return f.free ? ["", "Hands off for 0.3 s: free return at 100 mm/s"]
      : ["", `Hands off: the capped spring pulls back at F/D = ${(1000 * c.F / c.D).toFixed(0)} mm/s`];
    return ["", "Drag the tool (or anywhere on the picture) to push it"];
  }

  // ---- loop ---------------------------------------------------------------------------------
  let running = false, last = 0, acc = 0, uiT = 0, trailT = 0;
  function frame(now) {
    if (!running) return;
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now; acc += dt;
    if (drag) {
      const tx = drag.start[0] + (drag.x - drag.x0) / s, tz = drag.start[1] - (drag.y - drag.y0) / s;
      sim.aim(tx, tz);
    }
    sim.keyForce = keyForce();
    while (acc >= DT) { sim.step(); acc -= DT; }
    trailT += dt;
    if (trailT > 0.03) {
      trailT = 0;
      trail.push([START[0] + sim.x[0], START[1] + sim.x[1]]);
      if (trail.length > 90) trail.shift();
    }
    drawScene();
    drawChart();
    const [cls, text] = status(dt);
    uiT += dt;
    if (uiT > 0.08) {
      uiT = 0;
      out.force.textContent = mean(sim.hist.force, 50).toFixed(1);
      out.speed.textContent = (1000 * mean(sim.hist.speed, 50)).toFixed(0);
      out.dev.textContent = (1000 * Math.hypot(...sim.x)).toFixed(0);
    }
    if (text !== lastChip) { chip.textContent = text; chip.className = "sim-chip " + cls; lastChip = text; }
    requestAnimationFrame(frame);
  }
  new IntersectionObserver((es) => {
    const vis = es.some((e) => e.isIntersecting);
    if (vis && !running) { running = true; last = performance.now(); acc = 0; requestAnimationFrame(frame); }
    if (!vis) { running = false; if (drag) { drag = null; sim.release(); } }
  }).observe(stage);
}

const FONT = '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif';

for (const el of document.querySelectorAll("[data-pushsim]")) mount(el);
