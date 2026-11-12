// The in-page push simulation: controls, live chart and status around a robot view.
// Physics: core.js (500 Hz). View: view3d.js (three.js + the UR5e meshes, loaded when the
// section comes near), or view2d.js (canvas side view) if WebGL or the 3D files are missing.
import { PushSim, SETTINGS, LIMITS, gripImpedance, DT } from "./core.js";

const C = { ink2: "#52514e", muted: "#898781", grid: "#e1e0d9", blue: "#2a78d6", orange: "#eb6834" };
const FONT = '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif';
const KEY_FORCE = 12;  // N, arrow keys
const IDLE = {
  "3d": "Drag the robot to push its tool. Drag the background to turn the view.",
  "2d": "Drag the tool (or anywhere on the picture) to push it",
};

function lerp(a, b, t) { return a + (b - a) * t; }

export function mount(root) {
  const stage = root.querySelector("[data-stage]");
  const ch = root.querySelector("canvas[data-chart]");
  const chip = root.querySelector("[data-chip]");
  const hint = root.querySelector("[data-hint]");
  const loading = root.querySelector("[data-loading]");
  const viewsSlot = root.querySelector("[data-views]");
  const out = {
    force: root.querySelector("[data-out=force]"), speed: root.querySelector("[data-out=speed]"),
    dev: root.querySelector("[data-out=dev]"), params: root.querySelector("[data-params]"),
    grip: root.querySelector("[data-out=grip]"),
  };
  const gripIn = root.querySelector("input[data-grip]");
  const sim = new PushSim();
  const cctx = ch.getContext("2d");
  let view = null, dpr = 1, CW = 0, CH = 0;
  let keys = new Set(), oscOn = false, oscT = 0, oscHz = 0, lastChip = "";

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
  root.querySelector("[data-reset]").addEventListener("click", () => { sim.reset(); view?.reset(); oscOn = false; });
  setSetting("m2");
  setGrip();

  // ---- view ---------------------------------------------------------------------------------
  const ui = { touched() { hint.hidden = true; } };
  function placeHint() {
    const [x, y] = view.hintPos();
    hint.style.left = `${x}px`; hint.style.top = `${y + 14}px`;
  }
  function resize() {
    dpr = Math.min(2, window.devicePixelRatio || 1);
    CW = ch.clientWidth; CH = ch.clientHeight;
    ch.width = Math.round(CW * dpr); ch.height = Math.round(CH * dpr);
    if (view) { view.resize(); placeHint(); }
  }
  new ResizeObserver(resize).observe(stage);
  resize();

  const DIRS = { ArrowLeft: ["right", -1], ArrowRight: ["right", 1], ArrowUp: ["up", 1], ArrowDown: ["up", -1] };
  function attach(v) {
    view = v;
    stage.prepend(v.canvas);
    if (v.bar) viewsSlot.replaceChildren(v.bar);
    loading.hidden = true;
    hint.hidden = false;
    hint.textContent = v.kind === "3d" ? "Drag the robot to push it" : "Drag to push the tool";
    v.canvas.addEventListener("keydown", (e) => { if (DIRS[e.key]) { keys.add(e.key); ui.touched(); e.preventDefault(); } });
    v.canvas.addEventListener("keyup", (e) => keys.delete(e.key));
    v.canvas.addEventListener("blur", () => keys.clear());
    resize();
  }
  async function loadView() {
    try {
      const { createView3D } = await import("./view3d.js");
      attach(await createView3D(stage, sim, ui));
    } catch (err) {
      console.warn("3D view unavailable, using the 2D view:", err);
      const { createView2D } = await import("./view2d.js");
      attach(createView2D(stage, sim, ui));
    }
  }
  function keyForce() {
    if (!keys.size || !view) return null;
    const ax = view.keyAxes(), v = [0, 0, 0];
    for (const k of keys) { const [name, sgn] = DIRS[k]; for (let a = 0; a < 3; a++) v[a] += sgn * ax[name][a]; }
    const n = Math.hypot(...v);
    return n > 1e-9 ? v.map((c) => c * KEY_FORCE / n) : null;
  }

  // ---- chart --------------------------------------------------------------------------------
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
    if (!oscOn && r > 0.45 && sim.oscHz >= 8) { oscT += dt; if (oscT > 0.25) { oscOn = true; oscT = 0; } }
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
    return ["", IDLE[view ? view.kind : "3d"]];
  }

  // ---- loop ---------------------------------------------------------------------------------
  let running = false, last = 0, acc = 0, uiT = 0;
  function frame(now) {
    if (!running) return;
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now; acc += dt;
    sim.keyForce = keyForce();
    while (acc >= DT) { sim.step(); acc -= DT; }
    const [cls, text] = status(dt);
    if (view) { view.render(dt, { oscOn }); if (!hint.hidden) placeHint(); }
    drawChart();
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
  let loadStarted = false;
  new IntersectionObserver((es) => {
    if (!loadStarted && es.some((e) => e.isIntersecting)) { loadStarted = true; loadView(); }
  }, { rootMargin: "900px 0px" }).observe(stage);
  new IntersectionObserver((es) => {
    const vis = es.some((e) => e.isIntersecting);
    if (vis && !running) { running = true; last = performance.now(); acc = 0; requestAnimationFrame(frame); }
    if (!vis) { running = false; if (sim.holding) sim.release(); }
  }).observe(stage);
}

for (const el of document.querySelectorAll("[data-pushsim]")) mount(el);
