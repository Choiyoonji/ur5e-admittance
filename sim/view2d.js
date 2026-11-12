// 2D side view of the push simulation (fallback when WebGL or the 3D files are not available).
// The arm plane at pose A: reach direction E in base x-y, and z.
import { LIMITS } from "./core.js";

const C = {
  ink2: "#52514e", muted: "#898781", axis: "#c3c2b7", blue: "#2a78d6", orange: "#eb6834", critical: "#d03b3b",
  housing: "#3a3c3f", housingHi: "#5b5e63", cap: "#6c9dd1", capHi: "#a9c9ea",
  tubeLo: "#9da1a7", tubeHi: "#f1f2f4", floor: "#f1f0ec",
};
const E = [0.98, 0.199];               // reach direction at pose A (base x-y)
const START = [0.525, 0.207];          // tool flange at pose A, in the arm plane (reach, height) [m]
const SHOULDER = [0, 0.1625], L1 = 0.425, L2 = 0.3922;
const WRIST1 = [-0.0997, 0.0996];      // wrist 1 axis relative to the flange (tool pointing down)
const VIEW = { u0: -0.17, u1: 0.80, z0: -0.035, z1: 0.70 };
const FORCE_M_PER_N = 0.008;           // force arrow: metres per newton in the scene (capped at 30 N)
const FONT = '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif';
const inPlane = (v) => [E[0] * v[0] + E[1] * v[1], v[2]];

export function createView2D(stage, sim, ui) {
  const cv = document.createElement("canvas");
  cv.className = "sim-2d";
  cv.tabIndex = 0;
  cv.setAttribute("role", "application");
  cv.setAttribute("aria-label", "Side view of the robot. Drag to push the tool; with the picture focused, arrow keys push with 12 N.");
  const ctx = cv.getContext("2d");
  let s = 1, dpr = 1, W = 0, H = 0, drag = null, oscOn = false;
  const trail = [];
  let trailT = 0;
  const px = (u, z) => [(u - VIEW.u0) * s, (VIEW.z1 - z) * s];

  function resize() {
    dpr = Math.min(2, window.devicePixelRatio || 1);
    W = stage.clientWidth; H = Math.round(W * (VIEW.z1 - VIEW.z0) / (VIEW.u1 - VIEW.u0));
    cv.style.width = W + "px"; cv.style.height = H + "px";
    cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr);
    s = W / (VIEW.u1 - VIEW.u0);
  }

  cv.addEventListener("pointerdown", (e) => {
    if (e.button !== 0 && e.pointerType === "mouse") return;
    try { cv.setPointerCapture(e.pointerId); } catch { /* synthetic events */ }
    sim.grab();
    drag = { id: e.pointerId, x0: e.clientX, y0: e.clientY, start: sim.x.slice(), x: e.clientX, y: e.clientY };
    cv.classList.add("grabbing");
    ui.touched();
    e.preventDefault();
  });
  cv.addEventListener("pointermove", (e) => { if (drag && e.pointerId === drag.id) { drag.x = e.clientX; drag.y = e.clientY; } });
  const end = (e) => {
    if (!drag || e.pointerId !== drag.id) return;
    drag = null; sim.release(); cv.classList.remove("grabbing");
  };
  cv.addEventListener("pointerup", end);
  cv.addEventListener("pointercancel", end);

  // ---- drawing ------------------------------------------------------------------------------
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
    const [bx0, by0] = px(START[0] + B[0][0], START[1] + B[2][1]), [bx1, by1] = px(START[0] + B[0][1], START[1] + B[2][0]);
    ctx.setLineDash([4, 4]); ctx.strokeStyle = C.axis; ctx.lineWidth = 1;
    ctx.strokeRect(bx0 + 0.5, by0 + 0.5, bx1 - bx0, by1 - by0); ctx.setLineDash([]);
    // trail
    const xp = inPlane(sim.x), dp = inPlane(sim.dx);
    const T = [START[0] + xp[0], START[1] + xp[1]];
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
    const [rx, ry] = px(START[0], START[1] - 0.04), [cx, cy] = px(START[0] + dp[0], START[1] + dp[1] - 0.04);
    if (Math.hypot(cx - rx, cy - ry) > 3) {
      ctx.strokeStyle = C.blue; ctx.lineWidth = 1.5; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(cx, cy); ctx.stroke(); ctx.setLineDash([]);
    }
    ctx.fillStyle = C.blue; ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(rx, ry, 5, 0, 2 * Math.PI); ctx.stroke(); ctx.fill();
    // hand force on the handle
    const grip = px(T[0], T[1] - 0.04);
    const F = inPlane(sim.F);
    const nF = Math.hypot(...sim.F);
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

  function render(dt, state) {
    oscOn = state.oscOn;
    if (drag) {
      const du = (drag.x - drag.x0) / s, dz = -(drag.y - drag.y0) / s;
      sim.aim([drag.start[0] + E[0] * du, drag.start[1] + E[1] * du, drag.start[2] + dz]);
    }
    trailT += dt;
    if (trailT > 0.03) {
      trailT = 0;
      const p = inPlane(sim.x);
      trail.push([START[0] + p[0], START[1] + p[1]]);
      if (trail.length > 90) trail.shift();
    }
    drawScene();
  }
  function hintPos() { return px(START[0], START[1] - 0.09).map((v, i) => v + (i ? 18 : 0)); }
  function keyAxes() { return { right: [E[0], E[1], 0], up: [0, 0, 1] }; }
  function reset() { trail.length = 0; }
  return { canvas: cv, bar: null, resize, render, keyAxes, hintPos, reset, kind: "2d" };
}
