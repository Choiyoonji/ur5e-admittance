// Push simulation: the hand-guiding controller in a loop with a model of the robot.
//
// The controller is the real one's cycle for the tool position (orientation held), in base
// axes: 15 Hz first-order wrench filter (+ optional notch), ellipsoidal deadband, exact
// zero-order-hold admittance M dv' + D dv + sat(K dx) = F, force-limited spring with free
// return, speed and acceleration limits, workspace box, v + Kp (x_cmd - x_actual).
// The robot is the per-axis link response fitted to the open-loop identification (plant.js).
// The hand is a spring-damper around a smoothed cursor; its stiffness and damping are the grip.
// No DOM here, so node can run it (sim/test_core.mjs).
import { PLANT } from "./plant.js";

export const DT = PLANT.dt;
const N = 3;                 // x, y, z (base axes; at pose A the arm reaches along x)
const K = 2000;              // N/m
const FILTER_HZ = 15;
const DEADBAND = [0.75, 0.75, 1.75];  // N (z is the sensor's tool axis)
const TOUCH = 1.0;           // N past the deadband = touched
const FREE_DELAY = 0.3, FREE_RAMP = 0.2, V_MAX = 0.10;  // s, s, m/s
const ACC_MAX = 1.2;         // m/s^2: 0.8 x 4 rad/s^2 through the Jacobian at this pose
const KP = 3.0;              // 1/s
const BOX = [[-0.15, 0.15], [-0.15, 0.15], [-0.10, 0.15]];  // m around the start
const NOISE = [0.2, 0.2, 0.5];  // N rms per raw sample (sensor noise, z is 2.5x x)
const HAND_HZ = 5;           // the arm behind the hand: cursor -> hand equilibrium
const HAND_MAX = 60;         // N

const zeta1 = (m) => 2 * Math.sqrt(K * m);
export const SETTINGS = {
  m8: { M: 8, D: zeta1(8), F: 5, notch: null },
  m2: { M: 2, D: zeta1(2), F: 3.5, notch: null },
  m1n: { M: 1, D: zeta1(1), F: 3.5, notch: [36, 1.5] },
  m05: { M: 0.5, D: zeta1(0.5), F: 3.5, notch: null },
};

// grip 0..1 -> hand stiffness [N/m] and damping [N s/m]. A firm grip measured on the link is
// 40-60 N s/m; near 15-20 Hz the grip is mostly damping, so the spring is kept soft (0.8-2.5 kN/m).
export function gripImpedance(g) {
  return { k: 800 + 1700 * g, c: 5 + 55 * g };
}

const norm = (v) => Math.hypot(...v);
const zeros = () => new Array(N).fill(0);

function expm3(A) {  // scaling and squaring, 12-term Taylor (as expm_small in the controller)
  const mul = (P, Q) => P.map((r, i) => Q[0].map((_, j) => r.reduce((s, _v, k) => s + P[i][k] * Q[k][j], 0)));
  const nA = Math.max(...A.map((r) => r.reduce((s, v) => s + Math.abs(v), 0)));
  const j = nA > 0.25 ? Math.ceil(Math.log2(nA / 0.25)) : 0;
  const X = A.map((r) => r.map((v) => v / 2 ** j));
  let E = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], T = E;
  for (let k = 1; k <= 12; k++) { T = mul(T, X).map((r) => r.map((v) => v / k)); E = E.map((r, a) => r.map((v, b) => v + T[a][b])); }
  for (let i = 0; i < j; i++) E = mul(E, E);
  return E;
}

function zoh(m, d, k) {  // [dx; v](t+dt) = Phi [dx; v] + Gam u
  const E = expm3([[0, 1, 0], [-k / m, -d / m, 1 / m], [0, 0, 0]].map((r) => r.map((v) => v * DT)));
  return { p00: E[0][0], p01: E[0][1], p10: E[1][0], p11: E[1][1], g0: E[0][2], g1: E[1][2] };
}

function notchCoeffs(f0, q) {  // Tustin, prewarped at f0
  const k = Math.tan(Math.PI * f0 * DT), n = 1 / (1 + k / q + k * k);
  const b0 = (1 + k * k) * n, b1 = 2 * (k * k - 1) * n;
  return { b0, b1, b2: b0, a1: b1, a2: (1 - k / q + k * k) * n };
}

class Plant {  // one axis: link velocity from the commanded velocity
  constructor(p) { this.p = p; this.s = new Float64Array(p.B.length); this.t = new Float64Array(p.B.length); }
  step(u) {
    const { A, B, C, D } = this.p, s = this.s, t = this.t, n = s.length;
    let y = D * u;
    for (let i = 0; i < n; i++) y += C[i] * s[i];
    for (let i = 0; i < n; i++) { let a = B[i] * u; for (let j = 0; j < n; j++) a += A[i][j] * s[j]; t[i] = a; }
    this.s = t; this.t = s;
    return y;
  }
}

let spare = null;
function gauss() {
  if (spare !== null) { const v = spare; spare = null; return v; }
  let u = 0;
  while (u === 0) u = Math.random();
  const v = Math.random(), r = Math.sqrt(-2 * Math.log(u));
  spare = r * Math.sin(2 * Math.PI * v);
  return r * Math.cos(2 * Math.PI * v);
}

const HIST = 2500;  // 5 s at 500 Hz

export class PushSim {
  constructor() {
    this.alpha = 1 - Math.exp(-2 * Math.PI * FILTER_HZ * DT);
    this.hist = { force: new Float32Array(HIST), speed: new Float32Array(HIST), n: 0, i: 0 };
    this.setSetting("m2");
    this.setGrip(0.6);
    this.reset();
  }

  setSetting(key) {
    this.key = key;
    this.cfg = SETTINGS[key];
    const { M, D } = this.cfg;
    this.zK = zoh(M, D, K);
    this.z0 = zoh(M, D, 0);
    this.nc = this.cfg.notch ? notchCoeffs(...this.cfg.notch) : null;
    this.ns = Array.from({ length: N }, () => [0, 0]);
  }

  setGrip(g) { this.grip = g; this.hand = gripImpedance(g); }

  reset() {
    this.t = 0;
    this.x = zeros(); this.v = zeros();        // link position / velocity, relative to the start
    this.dx = zeros(); this.vd = zeros();      // admittance deviation state (x_cmd - x_ref)
    this.vcPrev = zeros();
    this.wf = zeros(); this.ns = Array.from({ length: N }, () => [0, 0]);
    this.plant = [new Plant(PLANT.x), new Plant(PLANT.y), new Plant(PLANT.z)];
    this.delay = Array.from({ length: PLANT.delay }, zeros);
    this.untouched = 1; this.free = 1;
    this.target = null; this.he = zeros(); this.hv = zeros();
    this.keyForce = null;
    this.F = zeros(); this.fext = zeros();
    this.flags = {};
    this.hp = zeros(); this.hpLp = zeros(); this.osc = 0; this.crossings = []; this.lastSign = 0; this.oscHz = 0;
    this.hist.n = 0; this.hist.i = 0; this.hist.force.fill(0); this.hist.speed.fill(0);
  }

  // hand: grab(), then aim([x, y, z]) with the hand's target (m, relative to the start), release()
  grab() { this.he = this.x.slice(); this.hv = this.v.slice(); this.target = this.x.slice(); }
  aim(p) { if (this.target) this.target = p.slice(); }
  release() { this.target = null; }
  get holding() { return this.target !== null; }

  handForce() {
    if (this.target) {
      const w = 2 * Math.PI * HAND_HZ, F = zeros();
      for (let a = 0; a < N; a++) {
        const acc = w * w * (this.target[a] - this.he[a]) - 2 * w * this.hv[a];
        this.hv[a] += acc * DT;
        this.he[a] += this.hv[a] * DT;
        F[a] = this.hand.k * (this.he[a] - this.x[a]) + this.hand.c * (this.hv[a] - this.v[a]);
      }
      const n = norm(F);
      return n > HAND_MAX ? F.map((f) => f * HAND_MAX / n) : F;
    }
    return this.keyForce ? this.keyForce.slice() : zeros();
  }

  step() {
    const c = this.cfg, flags = {};
    // hand, sensor, filter, notch
    const F = this.handForce();
    this.F = F;
    const wn = zeros();
    for (let a = 0; a < N; a++) {
      const w = F[a] + NOISE[a] * gauss();
      this.wf[a] += this.alpha * (w - this.wf[a]);
      if (this.nc) {
        const q = this.nc, s = this.ns[a], y = q.b0 * this.wf[a] + s[0];
        s[0] = q.b1 * this.wf[a] - q.a1 * y + s[1];
        s[1] = q.b2 * this.wf[a] - q.a2 * y;
        wn[a] = y;
      } else wn[a] = this.wf[a];
    }
    // ellipsoidal deadband
    const r = norm(wn.map((f, a) => f / DEADBAND[a]));
    const fext = r > 1 ? wn.map((f) => f * (1 - 1 / r)) : zeros();
    this.fext = fext;
    // free return
    if (norm(fext) > TOUCH) { this.untouched = 0; this.free = 0; }
    else {
      this.untouched += DT;
      if (this.untouched > FREE_DELAY) this.free = Math.min(1, this.free + DT / FREE_RAMP);
    }
    if (this.free > 0) flags.free = true;
    // force-limited spring
    const limit = c.F + (c.D * V_MAX - c.F) * this.free;
    let fs = this.dx.map((d) => K * d);
    const nfs = norm(fs), sat = nfs > limit;
    if (sat) { fs = fs.map((f) => f * limit / nfs); flags.limit = true; }
    // exact ZOH admittance over the coming cycle
    const Z = sat ? this.z0 : this.zK;
    const vNew = zeros(), step = zeros();
    for (let a = 0; a < N; a++) {
      const u = fext[a] - (sat ? fs[a] : 0);
      vNew[a] = Z.p10 * this.dx[a] + Z.p11 * this.vd[a] + Z.g1 * u;
      step[a] = (Z.p00 - 1) * this.dx[a] + Z.p01 * this.vd[a] + Z.g0 * u;
    }
    // workspace box around the start: the blocked part of the step is removed before the speed
    // limit, so a hand pushing into a face does not eat the speed along it
    for (let a = 0; a < N; a++) {
      const [lo, hi] = BOX[a], to = this.dx[a] + step[a];
      if (to < lo || to > hi) {
        step[a] = Math.min(hi, Math.max(lo, to)) - this.dx[a];
        vNew[a] = 0; flags.box = true;
      }
    }
    let vc = step.map((s) => s / DT);
    let limited = false;
    const sp = norm(vc);
    if (sp > V_MAX) { vc = vc.map((v) => v * V_MAX / sp); limited = true; flags.speed = true; }
    const dv = vc.map((v, a) => v - this.vcPrev[a]), ndv = norm(dv);
    if (ndv > ACC_MAX * DT) { vc = this.vcPrev.map((p, a) => p + dv[a] * ACC_MAX * DT / ndv); limited = true; flags.acc = true; }
    if (limited) { this.vd = vc.slice(); for (let a = 0; a < N; a++) step[a] = vc[a] * DT; }
    else this.vd = vNew;
    for (let a = 0; a < N; a++) this.dx[a] += step[a];
    // (rounding) keep the deviation inside the box
    for (let a = 0; a < N; a++) {
      const [lo, hi] = BOX[a];
      if (this.dx[a] < lo || this.dx[a] > hi) {
        this.dx[a] = Math.min(hi, Math.max(lo, this.dx[a]));
        this.vd[a] = 0; vc[a] = 0; flags.box = true;
      }
    }
    this.vcPrev = vc.slice();
    // command -> delay -> robot
    this.delay.push(vc.map((v, a) => v + KP * (this.dx[a] - this.x[a])));
    const u = this.delay.shift();
    for (let a = 0; a < N; a++) {
      const vl = this.plant[a].step(u[a]);
      this.x[a] += 0.5 * (this.v[a] + vl) * DT;
      this.v[a] = vl;
    }
    this.t += DT;
    this.flags = flags;
    this.watchOscillation(F);
    const h = this.hist;
    h.force[h.i] = norm(F);
    h.speed[h.i] = norm(this.v);
    h.i = (h.i + 1) % HIST;
    h.n = Math.min(HIST, h.n + 1);
  }

  // 12-60 Hz part of the hand force: rms (0.25 s) and the frequency from zero crossings (0.5 s)
  // of its largest component
  watchOscillation(F) {
    const aLo = 1 - Math.exp(-2 * Math.PI * 12 * DT), aHi = 1 - Math.exp(-2 * Math.PI * 60 * DT);
    let e = 0, dom = 0;
    for (let a = 0; a < N; a++) {
      this.hpLp[a] += aLo * (F[a] - this.hpLp[a]);
      this.hp[a] += aHi * (F[a] - this.hpLp[a] - this.hp[a]);
      e += this.hp[a] ** 2;
      if (Math.abs(this.hp[a]) > Math.abs(this.hp[dom])) dom = a;
    }
    this.osc += (DT / 0.25) * (e - this.osc);
    this.domAxis = this.domAxis === undefined ? dom : this.domAxis;
    if (Math.abs(this.hp[dom]) > 2 * Math.abs(this.hp[this.domAxis])) this.domAxis = dom;
    const sgn = Math.sign(this.hp[this.domAxis]);
    if (sgn !== 0 && sgn !== this.lastSign) { if (this.lastSign !== 0) this.crossings.push(this.t); this.lastSign = sgn; }
    while (this.crossings.length && this.crossings[0] < this.t - 0.5) this.crossings.shift();
    this.oscHz = this.crossings.length / (2 * 0.5);
  }

  get oscRms() { return Math.sqrt(Math.max(0, this.osc)); }

  run(seconds) { const n = Math.round(seconds / DT); for (let i = 0; i < n; i++) this.step(); }
}

export const LIMITS = { BOX, V_MAX, K, DEADBAND };
