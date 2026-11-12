// 3D view of the push simulation: the UR5e meshes posed by IK from the simulated tool position.
// Drag the robot to push (in the screen plane through the tool), drag the background to orbit.
import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { toCreasedNormals } from "three/addons/utils/BufferGeometryUtils.js";
import { LIMITS } from "./core.js";

const COL = { ref: 0x2a78d6, force: 0xeb6834, trail: 0x2a78d6, halo: 0xd03b3b, box: 0xa9a89e, grid: 0xd9d8d2 };
const Q_A = [191.5, -71.9, 115.1, -137.9, -90.7, -155.1].map((d) => d * Math.PI / 180);  // pose A
// URDF (ur_description, UR5e): joint origins and visual mesh offsets, xyz rpy
const JOINTS = [
  [0, 0, 0.1625, 0, 0, 0], [0, 0, 0, Math.PI / 2, 0, 0], [-0.425, 0, 0, 0, 0, 0],
  [-0.3922, 0, 0.1333, 0, 0, 0], [0, -0.0997, 0, Math.PI / 2, 0, 0], [0, 0.0996, 0, Math.PI / 2, Math.PI, Math.PI],
];
const LINKS = ["base", "shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3"];
const MESH_OFFSETS = {
  base: [0, 0, 0, 0, 0, Math.PI], shoulder: [0, 0, 0, 0, 0, Math.PI],
  upper_arm: [0, 0, 0.138, Math.PI / 2, 0, -Math.PI / 2], forearm: [0, 0, 0.007, Math.PI / 2, 0, -Math.PI / 2],
  wrist_1: [0, 0, -0.127, Math.PI / 2, 0, 0], wrist_2: [0, 0, -0.0997, 0, 0, 0], wrist_3: [0, -0.0005, -0.0989, Math.PI / 2, 0, 0],
};
const VIEWS = {  // camera position, target
  "3D": [[1.45, -1.22, 0.92], [0.27, 0.10, 0.31]],
  Side: [[0.30, -1.8, 0.34], [0.30, 0.12, 0.33]],
  Front: [[2.0, 0.18, 0.40], [0.30, 0.12, 0.33]],
  Top: [[0.36, 0.16, 2.3], [0.36, 0.17, 0.2]],
};
const HANDLE = 0.07, GRIP = 0.04;          // handle length below the flange, grip point on it [m]
const FORCE_M_PER_N = 0.008, FORCE_CAP = 30;

function origin(x, y, z, r, p, yw) {  // URDF <origin>: R = Rz(yaw) Ry(pitch) Rx(roll)
  const m = new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(r, p, yw, "ZYX"));
  m.setPosition(x, y, z);
  return m;
}
const J = JOINTS.map((o) => origin(...o));
const OFF = LINKS.map((l) => origin(...MESH_OFFSETS[l]));
const rz = new THREE.Matrix4();

function fk(q, out) {  // out: 7 link frames; returns tool0 (= the last frame)
  const T = new THREE.Matrix4();
  out[0].copy(T);
  for (let i = 0; i < 6; i++) { T.multiply(J[i]).multiply(rz.makeRotationZ(q[i])); out[i + 1].copy(T); }
  return out[6];
}

// Damped least squares on position + orientation (numeric Jacobian), orientation held at pose A
class IK {
  constructor() {
    this.q = Q_A.slice();
    this.F = Array.from({ length: 7 }, () => new THREE.Matrix4());
    this.tmp = Array.from({ length: 7 }, () => new THREE.Matrix4());
    const T0 = fk(this.q, this.F);
    this.p0 = new THREE.Vector3().setFromMatrixPosition(T0);
    this.R0 = new THREE.Quaternion().setFromRotationMatrix(T0);
    this.zTool = new THREE.Vector3(0, 0, 1).applyQuaternion(this.R0);
  }
  err(T, target) {
    const p = new THREE.Vector3().setFromMatrixPosition(T), qn = new THREE.Quaternion().setFromRotationMatrix(T);
    const dq = this.R0.clone().multiply(qn.invert());
    if (dq.w < 0) { dq.x = -dq.x; dq.y = -dq.y; dq.z = -dq.z; dq.w = -dq.w; }
    return [target.x - p.x, target.y - p.y, target.z - p.z, 2 * dq.x, 2 * dq.y, 2 * dq.z];
  }
  solve(target, iters = 3) {
    for (let it = 0; it < iters; it++) {
      const e = this.err(fk(this.q, this.F), target);
      if (Math.hypot(...e) < 1e-6) break;
      const h = 1e-6, Jm = [];
      for (let j = 0; j < 6; j++) {
        const qq = this.q.slice(); qq[j] += h;
        const ej = this.err(fk(qq, this.tmp), target);
        Jm.push(ej.map((v, i) => (e[i] - v) / h));  // column j: d(pose)/dq_j
      }
      // dq = J^T (J J^T + l^2 I)^-1 e
      const A = Array.from({ length: 6 }, (_, r) => Array.from({ length: 6 }, (_, c) => {
        let s = r === c ? 1e-6 : 0;
        for (let j = 0; j < 6; j++) s += Jm[j][r] * Jm[j][c];
        return s;
      }));
      const y = gauss6(A, e.slice());
      for (let j = 0; j < 6; j++) { let s = 0; for (let r = 0; r < 6; r++) s += Jm[j][r] * y[r]; this.q[j] += s; }
    }
    fk(this.q, this.F);
    return this.F;
  }
}
function gauss6(A, b) {  // solve A y = b (6x6, partial pivoting)
  const n = 6;
  for (let i = 0; i < n; i++) {
    let p = i;
    for (let r = i + 1; r < n; r++) if (Math.abs(A[r][i]) > Math.abs(A[p][i])) p = r;
    [A[i], A[p]] = [A[p], A[i]]; [b[i], b[p]] = [b[p], b[i]];
    for (let r = i + 1; r < n; r++) {
      const f = A[r][i] / A[i][i];
      for (let c = i; c < n; c++) A[r][c] -= f * A[i][c];
      b[r] -= f * b[i];
    }
  }
  const y = new Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) { let s = b[i]; for (let c = i + 1; c < n; c++) s -= A[i][c] * y[c]; y[i] = s / A[i][i]; }
  return y;
}

const ON_TOP = { depthTest: false, depthWrite: false, transparent: true };
const UP = new THREE.Vector3(0, 1, 0);
function rod(radius, color, opacity = 1) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 1, 20),
    new THREE.MeshStandardMaterial({ color, roughness: 0.5, opacity, ...ON_TOP }));
  m.renderOrder = 10;
  return m;
}
function placeBetween(mesh, a, b) {
  const len = a.distanceTo(b);
  mesh.visible = len > 1e-4;
  if (!mesh.visible) return;
  mesh.position.copy(a).add(b).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(UP, b.clone().sub(a).normalize());
  mesh.scale.set(1, len, 1);
}
function makeArrow(color) {
  const g = new THREE.Group(), shaft = rod(0.0055, color);
  const head = new THREE.Mesh(new THREE.ConeGeometry(0.016, 0.04, 24),
    new THREE.MeshStandardMaterial({ color, roughness: 0.5, ...ON_TOP }));
  head.renderOrder = 10;
  g.add(shaft, head);
  g.set = (from, vec) => {
    const len = vec.length();
    g.visible = len > 0.004;
    if (!g.visible) return;
    const dir = vec.clone().normalize(), tip = from.clone().addScaledVector(dir, len);
    placeBetween(shaft, from, tip.clone().addScaledVector(dir, -0.035));
    head.position.copy(tip).addScaledVector(dir, -0.02);
    head.quaternion.setFromUnitVectors(UP, dir);
  };
  return g;
}

export async function createView3D(stage, sim, ui) {
  const canvas = document.createElement("canvas");
  canvas.className = "sim-gl";
  canvas.tabIndex = 0;
  canvas.setAttribute("role", "application");
  canvas.setAttribute("aria-label", "3D view of the robot. Drag the robot to push its tool, drag the background to " +
    "turn the view. With the view focused, arrow keys push with 12 N.");
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });  // throws without WebGL
  renderer.setClearColor(0xfbfbfa);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const gltf = await new GLTFLoader().loadAsync(new URL("./ur5e.glb", import.meta.url).href);
  const scene = new THREE.Scene();
  scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f96, 1.8));
  const sun = new THREE.DirectionalLight(0xffffff, 2.2);
  sun.position.set(1.5, -1.2, 3.0);
  sun.target.position.set(0.3, 0.1, 0);
  sun.castShadow = true;
  sun.shadow.mapSize.set(1024, 1024);
  Object.assign(sun.shadow.camera, { left: -0.8, right: 0.8, top: 0.8, bottom: -0.8, near: 0.5, far: 6 });
  scene.add(sun, sun.target);
  const fill = new THREE.DirectionalLight(0xffffff, 0.7);
  fill.position.set(-2, 2, 1.5);
  scene.add(fill);
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(4, 4), new THREE.ShadowMaterial({ opacity: 0.16 }));
  ground.receiveShadow = true;
  scene.add(ground);
  const grid = new THREE.GridHelper(2, 20, COL.grid, COL.grid);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = 0.0005;
  scene.add(grid);

  // robot: one holder per link, posed from the IK frames
  const holders = LINKS.map((name) => {
    const src = gltf.scene.getObjectByName(name);
    const h = new THREE.Group();
    h.matrixAutoUpdate = false;
    src.traverse((o) => {
      if (!o.isMesh) return;
      o.geometry = toCreasedNormals(o.geometry, Math.PI / 6);
      const old = o.material;
      o.material = new THREE.MeshStandardMaterial({ color: old.color, roughness: 0.55, metalness: 0.05 });
      o.castShadow = true; o.receiveShadow = true;
    });
    h.add(src);
    scene.add(h);
    return h;
  });
  const robotMeshes = [];
  holders.forEach((h) => h.traverse((o) => { if (o.isMesh) robotMeshes.push(o); }));
  // a short handle under the flange: what the hand holds
  const handle = new THREE.Mesh(new THREE.CylinderGeometry(0.017, 0.017, HANDLE, 28),
    new THREE.MeshStandardMaterial({ color: 0xd4d6d9, roughness: 0.45, metalness: 0.1 }));
  handle.castShadow = true;
  scene.add(handle);
  robotMeshes.push(handle);

  // overlays
  const ik = new IK();
  const start = ik.p0.clone();
  const box = (() => {
    const B = LIMITS.BOX, size = B.map(([lo, hi]) => hi - lo), mid = B.map(([lo, hi]) => (lo + hi) / 2);
    const g = new THREE.EdgesGeometry(new THREE.BoxGeometry(...size));
    const l = new THREE.LineSegments(g, new THREE.LineDashedMaterial({ color: COL.box, dashSize: 0.012, gapSize: 0.01,
      transparent: true, opacity: 0.85 }));
    l.computeLineDistances();
    l.position.set(start.x + mid[0], start.y + mid[1], start.z + mid[2]);
    return l;
  })();
  scene.add(box);
  const gripOff = ik.zTool.clone().multiplyScalar(GRIP);
  const refBall = new THREE.Mesh(new THREE.SphereGeometry(0.011, 24, 16),
    new THREE.MeshStandardMaterial({ color: COL.ref, roughness: 0.4, ...ON_TOP }));
  refBall.renderOrder = 10;
  refBall.position.copy(start).add(gripOff);
  scene.add(refBall);
  const spring = rod(0.0025, COL.ref, 0.85);
  scene.add(spring);
  const arrow = makeArrow(COL.force);
  scene.add(arrow);
  const handBall = new THREE.Mesh(new THREE.SphereGeometry(0.012, 24, 16),
    new THREE.MeshStandardMaterial({ color: COL.force, roughness: 0.4, ...ON_TOP }));
  handBall.renderOrder = 11;
  scene.add(handBall);
  const TRAIL = 90, trailPts = new Float32Array(TRAIL * 3);
  const trailGeo = new THREE.BufferGeometry();
  trailGeo.setAttribute("position", new THREE.BufferAttribute(trailPts, 3));
  const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: COL.trail, opacity: 0.6, ...ON_TOP }));
  trail.renderOrder = 9;
  trail.frustumCulled = false;
  scene.add(trail);
  let trailN = 0, trailT = 0;
  const halo = new THREE.Mesh(new THREE.SphereGeometry(1, 32, 20),
    new THREE.MeshBasicMaterial({ color: COL.halo, transparent: true, opacity: 0, depthWrite: false }));
  halo.renderOrder = 5;
  scene.add(halo);

  // camera and orbit
  const cam = new THREE.PerspectiveCamera(30, 4 / 3, 0.02, 20);
  cam.up.set(0, 0, 1);
  cam.position.set(...VIEWS["3D"][0]);
  const controls = new OrbitControls(cam, canvas);
  controls.target.set(...VIEWS["3D"][1]);
  Object.assign(controls, { enablePan: false, enableZoom: false, enableDamping: true, dampingFactor: 0.12,
    rotateSpeed: 0.8, maxPolarAngle: Math.PI / 2 - 0.03 });
  controls.update();
  let anim = null;
  function goTo(name) {
    const [p, t] = VIEWS[name];
    anim = { t: 0, p0: cam.position.clone(), t0: controls.target.clone(), p1: new THREE.Vector3(...p), t1: new THREE.Vector3(...t) };
  }

  // input: push when the pointer lands on the robot or near the grip point, otherwise orbit
  const ray = new THREE.Raycaster(), ndc = new THREE.Vector2(), plane = new THREE.Plane();
  let drag = null;
  const tool = () => start.clone().add(new THREE.Vector3(...sim.x));
  const gripPoint = () => tool().add(gripOff);
  function rayAt(e) {
    const r = canvas.getBoundingClientRect();
    ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ndc, cam);
    return r;
  }
  function onRobot(e) {
    const r = rayAt(e);
    if (ray.intersectObjects(robotMeshes, false).length) return true;
    const g = gripPoint().project(cam);
    const dx = (g.x - ndc.x) * r.width / 2, dy = (g.y - ndc.y) * r.height / 2;
    return Math.hypot(dx, dy) < 40;
  }
  canvas.addEventListener("pointerdown", (e) => {
    if (e.pointerType === "mouse" && e.button !== 0) return;
    if (!onRobot(e)) return;  // OrbitControls takes it
    e.stopImmediatePropagation();
    e.preventDefault();
    try { canvas.setPointerCapture(e.pointerId); } catch { /* synthetic events */ }
    const n = new THREE.Vector3();
    cam.getWorldDirection(n);
    plane.setFromNormalAndCoplanarPoint(n, gripPoint());
    rayAt(e);
    const p0 = new THREE.Vector3();
    if (!ray.ray.intersectPlane(plane, p0)) return;
    sim.grab();
    drag = { id: e.pointerId, p0, x0: sim.x.slice(), last: e };
    canvas.style.cursor = "grabbing";
    ui.touched();
  }, { capture: true });
  canvas.addEventListener("pointermove", (e) => {
    if (drag && e.pointerId === drag.id) { drag.last = e; return; }
    if (e.pointerType === "mouse" && !e.buttons) canvas.style.cursor = onRobot(e) ? "grab" : "move";
  });
  const end = (e) => {
    if (!drag || e.pointerId !== drag.id) return;
    drag = null; sim.release(); canvas.style.cursor = "grab";
  };
  canvas.addEventListener("pointerup", end);
  canvas.addEventListener("pointercancel", end);
  controls.addEventListener("start", () => ui.touched());
  function keyAxes() {  // arrow keys push along the screen's right and up
    const right = new THREE.Vector3().setFromMatrixColumn(cam.matrixWorld, 0);
    const up = new THREE.Vector3().setFromMatrixColumn(cam.matrixWorld, 1);
    return { right: right.toArray(), up: up.toArray() };
  }

  // view buttons
  const bar = document.createElement("div");
  bar.className = "sim-views";
  for (const name of Object.keys(VIEWS)) {
    const b = document.createElement("button");
    b.type = "button"; b.textContent = name;
    b.addEventListener("click", () => { goTo(name); ui.touched(); });
    bar.append(b);
  }

  let W = 0, H = 0;
  function resize() {
    W = stage.clientWidth;
    H = Math.round(W * (W < 520 ? 0.95 : 0.75));
    renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    renderer.setSize(W, H, false);
    canvas.style.width = W + "px"; canvas.style.height = H + "px";
    cam.aspect = W / H;
    cam.fov = W < 520 ? 36 : 30;
    cam.updateProjectionMatrix();
  }

  const v3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);
  function render(dt, state) {
    if (anim) {
      anim.t = Math.min(1, anim.t + dt / 0.45);
      const s = anim.t * anim.t * (3 - 2 * anim.t);
      cam.position.lerpVectors(anim.p0, anim.p1, s);
      controls.target.lerpVectors(anim.t0, anim.t1, s);
      if (anim.t >= 1) anim = null;
    }
    controls.update();
    if (drag) {
      rayAt(drag.last);
      const p = new THREE.Vector3();
      if (ray.ray.intersectPlane(plane, p)) {
        const d = p.sub(drag.p0);
        sim.aim([drag.x0[0] + d.x, drag.x0[1] + d.y, drag.x0[2] + d.z]);
      }
    }
    const T = tool();
    const F = ik.solve(T);
    holders.forEach((h, i) => { h.matrix.multiplyMatrices(F[i], OFF[i]); h.matrixWorldNeedsUpdate = true; });
    handle.position.copy(T).addScaledVector(ik.zTool, HANDLE / 2);
    handle.quaternion.setFromUnitVectors(UP, ik.zTool);
    const grip = T.clone().add(gripOff);
    placeBetween(spring, refBall.position, start.clone().add(v3(sim.dx)).add(gripOff));
    const f = v3(sim.F), nF = f.length();
    arrow.set(grip, f.multiplyScalar(FORCE_M_PER_N * Math.min(1, FORCE_CAP / Math.max(nF, 1e-9))));
    handBall.visible = nF > 0.3;
    handBall.position.copy(grip);
    trailT += dt;
    if (trailT > 0.03) {
      trailT = 0;
      trailPts.copyWithin(0, 3);
      trailPts.set([grip.x, grip.y, grip.z], (TRAIL - 1) * 3);
      trailN = Math.min(TRAIL, trailN + 1);
      trailGeo.setDrawRange(TRAIL - trailN, trailN);
      trailGeo.attributes.position.needsUpdate = true;
    }
    const a = state.oscOn ? Math.min(1, sim.oscRms / 2) : 0;
    halo.visible = a > 0;
    halo.position.copy(grip);
    halo.scale.setScalar(0.04 + 0.05 * a * (0.85 + 0.15 * Math.sin(performance.now() / 40)));
    halo.material.opacity = 0.18 + 0.2 * a;
    renderer.render(scene, cam);
  }
  function hintPos() {
    cam.updateMatrixWorld();
    const g = gripPoint().project(cam);
    return [(g.x + 1) / 2 * W, (1 - g.y) / 2 * H];
  }
  function reset() { trailN = 0; trailGeo.setDrawRange(0, 0); ik.q = Q_A.slice(); }

  return { canvas, bar, resize, render, keyAxes, hintPos, reset, kind: "3d" };
}
