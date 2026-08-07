(function () {
"use strict";

/* ------------------------------------------------------------------ *
 *  Constants: a small single-seat sailplane                          *
 * ------------------------------------------------------------------ */
const G = 9.81;
const RHO = 1.225;

const MASS = 300;          // kg
const WINGSPAN = 12;       // m
const S_WING = 9;          // m^2
const CHORD = S_WING / WINGSPAN;
const AR = (WINGSPAN * WINGSPAN) / S_WING;

const I_PITCH = 900;       // kg*m^2, about local X (right) axis
const I_YAW = 1100;        // kg*m^2, about local Y (up) axis
const I_ROLL = 700;        // kg*m^2, about local Z (aft) axis

const CL0 = 0.2;
const CLALPHA = 5.5;       // per rad
const STALL_ALPHA = 14 * Math.PI / 180;
const CD0 = 0.02;
const K_INDUCED = 1 / (Math.PI * 0.85 * AR);
const AIRBRAKE_CD = 0.9;

const CY_BETA = 0.6;

const CL_AILERON = 0.09;
const CL_P = 0.5;
const CL_BETA = 0.06;       // dihedral effect (roll restoring)

const CM0 = 0.02;
const CM_ALPHA = -0.6;      // pitch static stability
const CM_Q = 12;
const CM_ELEVATOR = 0.16;

const CN_RUDDER = 0.06;
const CN_R = 0.35;
const CN_BETA = 0.09;

const STALL_SPEED = Math.sqrt((2 * MASS * G) / (RHO * S_WING * (CL0 + CLALPHA * STALL_ALPHA)));
const LAUNCH_ALT = 400;
const LAUNCH_SPEED = STALL_SPEED * 1.6;

const GROUND_LEVEL = 0;

/* ------------------------------------------------------------------ *
 *  Simulation state                                                  *
 * ------------------------------------------------------------------ */
const V3 = THREE.Vector3;

const state = {
  pos: new V3(0, LAUNCH_ALT, 0),
  vel: new V3(0, -1, -LAUNCH_SPEED),
  quat: new THREE.Quaternion(),
  angVel: new V3(0, 0, 0),  // body frame: x=pitch-axis rate, y=yaw-axis rate, z=roll-axis rate
  launched: false,
  crashed: false,
  landed: false,
  airspeed: LAUNCH_SPEED,
  alpha: 0,
  beta: 0,
};

const controls = { aileron: 0, elevator: 0, rudder: 0, airbrake: 0 };

const LOCAL_FWD = new V3(0, 0, -1);
const LOCAL_RIGHT = new V3(1, 0, 0);
const LOCAL_UP = new V3(0, 1, 0);

function resetState() {
  state.pos.set(0, LAUNCH_ALT, 0);
  const q = new THREE.Quaternion().setFromAxisAngle(new V3(0, 1, 0), 0);
  state.quat.copy(q);
  state.vel.set(0, -0.5, -LAUNCH_SPEED);
  state.angVel.set(0, 0, 0);
  state.launched = true;
  state.crashed = false;
  state.landed = false;
  hideMessage();
}

function physicsStep(dt) {
  if (!state.launched || state.crashed || state.landed) return;

  const q = state.quat;
  const qInv = q.clone().invert();
  const vBody = state.vel.clone().applyQuaternion(qInv);

  const u = -vBody.z;              // forward speed
  const w = -vBody.y;              // "downward" component in body frame
  const v = vBody.x;               // sideways speed
  const V_air = Math.max(state.vel.length(), 0.01);

  const alpha = Math.atan2(w, Math.max(u, 0.01));
  const beta = Math.atan2(v, Math.max(u, 0.01));
  state.alpha = alpha;
  state.beta = beta;
  state.airspeed = V_air;

  // --- Lift / drag coefficients with a simple stall model ---
  let CL;
  const absAlpha = Math.abs(alpha);
  if (absAlpha <= STALL_ALPHA) {
    CL = CL0 + CLALPHA * alpha;
  } else {
    const over = absAlpha - STALL_ALPHA;
    const peak = CL0 + CLALPHA * STALL_ALPHA * Math.sign(alpha || 1);
    const falloff = Math.max(0.15, 1 - over * 3.2);
    CL = peak * falloff;
  }
  let CD = CD0 + K_INDUCED * CL * CL;
  if (absAlpha > STALL_ALPHA) CD += (absAlpha - STALL_ALPHA) * 0.6;
  CD += AIRBRAKE_CD * controls.airbrake * 0.15;

  const CY = -CY_BETA * beta;

  const qDyn = 0.5 * RHO * V_air * V_air;
  const L = qDyn * S_WING * CL;
  const D = qDyn * S_WING * CD;
  const Y = qDyn * S_WING * CY;

  const dir = vBody.lengthSq() > 1e-6 ? vBody.clone().normalize() : new V3(0, 0, -1);
  let liftDir = LOCAL_UP.clone().sub(dir.clone().multiplyScalar(dir.dot(LOCAL_UP)));
  liftDir = liftDir.lengthSq() > 1e-6 ? liftDir.normalize() : new V3(0, 1, 0);
  let sideDir = LOCAL_RIGHT.clone().sub(dir.clone().multiplyScalar(dir.dot(LOCAL_RIGHT)));
  sideDir = sideDir.lengthSq() > 1e-6 ? sideDir.normalize() : new V3(1, 0, 0);

  const aeroForceBody = dir.clone().multiplyScalar(-D)
    .add(liftDir.multiplyScalar(L))
    .add(sideDir.multiplyScalar(Y));

  const aeroForceWorld = aeroForceBody.applyQuaternion(q);
  const gravityForce = new V3(0, -MASS * G, 0);
  const totalForce = aeroForceWorld.add(gravityForce);
  const accel = totalForce.multiplyScalar(1 / MASS);
  state.vel.addScaledVector(accel, dt);

  // --- Moments (control + damping + stability) ---
  const V_safe = Math.max(V_air, 3);
  const pHat = state.angVel.z * WINGSPAN / (2 * V_safe);
  const qHat = state.angVel.x * CHORD / (2 * V_safe);
  const rHat = state.angVel.y * WINGSPAN / (2 * V_safe);

  // CL_P/CM_Q/CN_R are stored as positive magnitudes, so damping subtracts.
  const rollMoment = qDyn * S_WING * WINGSPAN *
    (CL_BETA * beta - CL_P * pHat - CL_AILERON * controls.aileron);
  const pitchMoment = qDyn * S_WING * CHORD *
    (CM0 + CM_ALPHA * alpha - CM_Q * qHat + CM_ELEVATOR * controls.elevator);
  const yawMoment = qDyn * S_WING * WINGSPAN *
    (-CN_BETA * beta - CN_R * rHat - CN_RUDDER * controls.rudder);

  state.angVel.x += (pitchMoment / I_PITCH) * dt;
  state.angVel.y += (yawMoment / I_YAW) * dt;
  state.angVel.z += (rollMoment / I_ROLL) * dt;

  // --- Quaternion integration: dq/dt = 0.5 * q * omega ---
  const omega = state.angVel;
  const qOmega = new THREE.Quaternion(omega.x, omega.y, omega.z, 0);
  const qDot = q.clone().multiply(qOmega);
  q.x += 0.5 * qDot.x * dt;
  q.y += 0.5 * qDot.y * dt;
  q.z += 0.5 * qDot.z * dt;
  q.w += 0.5 * qDot.w * dt;
  q.normalize();

  state.pos.addScaledVector(state.vel, dt);

  checkGround();
}

function extractAttitude() {
  const q = state.quat;
  const worldFwd = LOCAL_FWD.clone().applyQuaternion(q);
  const worldRight = LOCAL_RIGHT.clone().applyQuaternion(q);
  const pitch = Math.asin(THREE.MathUtils.clamp(worldFwd.y, -1, 1));
  const bank = Math.asin(THREE.MathUtils.clamp(-worldRight.y, -1, 1));
  let heading = Math.atan2(worldFwd.x, -worldFwd.z);
  if (heading < 0) heading += Math.PI * 2;
  return { pitch, bank, heading };
}

function checkGround() {
  if (state.pos.y > GROUND_LEVEL + 0.05) return;
  const { pitch, bank } = extractAttitude();
  const vs = state.vel.y;
  const gentle = vs > -4.5 && Math.abs(bank) < 0.35 && Math.abs(pitch) < 0.35;
  state.pos.y = GROUND_LEVEL;
  state.vel.set(0, 0, 0);
  state.angVel.set(0, 0, 0);
  if (gentle) {
    state.landed = true;
    showMessage("LANDED", "#7fffb0");
  } else {
    state.crashed = true;
    showMessage("CRASHED", "#ff6b6b");
  }
}

/* ------------------------------------------------------------------ *
 *  Controls: touch joysticks + optional device tilt                  *
 * ------------------------------------------------------------------ */
const input = { left: { x: 0, y: 0 }, right: { x: 0, y: 0 } };
let tiltEnabled = false;
let tiltBaseline = null;

function setupJoystick(rootEl, target) {
  const knob = rootEl.querySelector(".joystick-knob");
  const base = rootEl.querySelector(".joystick-base");
  let active = false;
  let pointerId = null;
  const radius = 65;

  function move(clientX, clientY) {
    const rect = base.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    let dx = clientX - cx;
    let dy = clientY - cy;
    const dist = Math.hypot(dx, dy);
    if (dist > radius) {
      dx = (dx / dist) * radius;
      dy = (dy / dist) * radius;
    }
    knob.style.transform = `translate(${dx - 28}px, ${dy - 28}px)`;
    target.x = dx / radius;
    target.y = dy / radius;
  }

  function reset() {
    knob.style.transform = "translate(-50%, -50%)";
    target.x = 0;
    target.y = 0;
  }

  rootEl.addEventListener("pointerdown", (e) => {
    active = true;
    pointerId = e.pointerId;
    rootEl.setPointerCapture(e.pointerId);
    move(e.clientX, e.clientY);
    e.preventDefault();
  });
  rootEl.addEventListener("pointermove", (e) => {
    if (!active || e.pointerId !== pointerId) return;
    move(e.clientX, e.clientY);
    e.preventDefault();
  });
  function end(e) {
    if (e.pointerId !== pointerId) return;
    active = false;
    pointerId = null;
    reset();
  }
  rootEl.addEventListener("pointerup", end);
  rootEl.addEventListener("pointercancel", end);
}

function setupTilt() {
  const btn = document.getElementById("btn-tilt");
  btn.addEventListener("click", async () => {
    if (typeof DeviceOrientationEvent !== "undefined" &&
        typeof DeviceOrientationEvent.requestPermission === "function") {
      try {
        const res = await DeviceOrientationEvent.requestPermission();
        if (res !== "granted") return;
      } catch (e) { return; }
    }
    tiltEnabled = !tiltEnabled;
    tiltBaseline = null;
    btn.classList.toggle("active", tiltEnabled);
  });

  window.addEventListener("deviceorientation", (e) => {
    if (!tiltEnabled) return;
    if (e.beta === null || e.gamma === null) return;
    if (!tiltBaseline) tiltBaseline = { beta: e.beta, gamma: e.gamma };
    const dPitch = THREE.MathUtils.clamp((e.beta - tiltBaseline.beta) / 30, -1, 1);
    const dRoll = THREE.MathUtils.clamp((e.gamma - tiltBaseline.gamma) / 30, -1, 1);
    input.left.y = dPitch;
    input.left.x = dRoll;
  });
}

function updateControlsFromInput() {
  controls.aileron = THREE.MathUtils.clamp(input.left.x, -1, 1);
  controls.elevator = THREE.MathUtils.clamp(-input.left.y, -1, 1);
  controls.rudder = THREE.MathUtils.clamp(input.right.x, -1, 1);
  controls.airbrake = THREE.MathUtils.clamp(input.right.y, 0, 1);
}

/* ------------------------------------------------------------------ *
 *  Three.js scene                                                    *
 * ------------------------------------------------------------------ */
const canvas = document.getElementById("scene");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: "high-performance" });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));

const scene = new THREE.Scene();
const skyColor = new THREE.Color(0x8fc7ea);
scene.background = skyColor;
scene.fog = new THREE.Fog(0xbfe0f5, 800, 6000);

const camera = new THREE.PerspectiveCamera(62, window.innerWidth / window.innerHeight, 0.1, 8000);

const hemi = new THREE.HemisphereLight(0xffffff, 0x3a2c1a, 0.9);
scene.add(hemi);
const sun = new THREE.DirectionalLight(0xfff4dd, 1.1);
sun.position.set(400, 600, 200);
scene.add(sun);

function createSkyDome() {
  const size = 4096;
  const c = document.createElement("canvas");
  c.width = 8; c.height = size;
  const ctx = c.getContext("2d");
  const grad = ctx.createLinearGradient(0, 0, 0, size);
  grad.addColorStop(0, "#2f6fb0");
  grad.addColorStop(0.55, "#8fc7ea");
  grad.addColorStop(0.78, "#cfe9f7");
  grad.addColorStop(1, "#eef6f0");
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, 8, size);
  const tex = new THREE.CanvasTexture(c);
  const geo = new THREE.SphereGeometry(5000, 24, 16);
  const mat = new THREE.MeshBasicMaterial({ map: tex, side: THREE.BackSide, fog: false });
  return new THREE.Mesh(geo, mat);
}
scene.add(createSkyDome());

function createGroundTexture() {
  const c = document.createElement("canvas");
  c.width = c.height = 512;
  const ctx = c.getContext("2d");
  ctx.fillStyle = "#4f7a3d";
  ctx.fillRect(0, 0, 512, 512);
  ctx.strokeStyle = "rgba(255,255,255,0.12)";
  ctx.lineWidth = 2;
  const step = 512 / 8;
  for (let i = 0; i <= 8; i++) {
    ctx.beginPath(); ctx.moveTo(i * step, 0); ctx.lineTo(i * step, 512); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, i * step); ctx.lineTo(512, i * step); ctx.stroke();
  }
  for (let i = 0; i < 400; i++) {
    ctx.fillStyle = `rgba(${20 + Math.random() * 40 | 0}, ${60 + Math.random() * 40 | 0}, ${20 + Math.random() * 30 | 0}, 0.25)`;
    ctx.beginPath();
    ctx.arc(Math.random() * 512, Math.random() * 512, 4 + Math.random() * 10, 0, Math.PI * 2);
    ctx.fill();
  }
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.repeat.set(300, 300);
  tex.anisotropy = 4;
  return tex;
}

const groundGeo = new THREE.PlaneGeometry(30000, 30000);
const groundMat = new THREE.MeshLambertMaterial({ map: createGroundTexture() });
const ground = new THREE.Mesh(groundGeo, groundMat);
ground.rotation.x = -Math.PI / 2;
scene.add(ground);

// Runway marker near the origin
const runway = new THREE.Mesh(
  new THREE.PlaneGeometry(18, 400),
  new THREE.MeshLambertMaterial({ color: 0x555555 })
);
runway.rotation.x = -Math.PI / 2;
runway.position.set(0, 0.05, -100);
scene.add(runway);
for (let i = 0; i < 6; i++) {
  const stripe = new THREE.Mesh(new THREE.PlaneGeometry(2, 20), new THREE.MeshLambertMaterial({ color: 0xffffff }));
  stripe.rotation.x = -Math.PI / 2;
  stripe.position.set(0, 0.06, -20 - i * 40);
  scene.add(stripe);
}

// Scattered markers for visual speed/motion reference
const markerGeo = new THREE.ConeGeometry(3, 10, 5);
const markerMat = new THREE.MeshLambertMaterial({ color: 0x2f4d2a });
const markerMesh = new THREE.InstancedMesh(markerGeo, markerMat, 260);
const dummy = new THREE.Object3D();
for (let i = 0; i < 260; i++) {
  const angle = Math.random() * Math.PI * 2;
  const r = 150 + Math.random() * 2800;
  dummy.position.set(Math.cos(angle) * r, 5, Math.sin(angle) * r);
  dummy.scale.setScalar(0.6 + Math.random() * 1.6);
  dummy.rotation.y = Math.random() * Math.PI * 2;
  dummy.updateMatrix();
  markerMesh.setMatrixAt(i, dummy.matrix);
}
scene.add(markerMesh);

/* --- Glider model, built from primitives, local -Z = forward --- */
function buildGlider() {
  const group = new THREE.Group();
  const bodyMat = new THREE.MeshStandardMaterial({ color: 0xf2f2f2, roughness: 0.5 });
  const accentMat = new THREE.MeshStandardMaterial({ color: 0xd23c3c, roughness: 0.5 });
  const glassMat = new THREE.MeshStandardMaterial({ color: 0x335566, roughness: 0.2, metalness: 0.3 });

  const fuselage = new THREE.Mesh(new THREE.CapsuleGeometry(0.35, 4.6, 4, 8), bodyMat);
  fuselage.rotation.x = Math.PI / 2;
  fuselage.position.z = -0.3;
  group.add(fuselage);

  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.32, 0.9, 8), accentMat);
  nose.rotation.x = -Math.PI / 2;
  nose.position.z = -2.9;
  group.add(nose);

  const canopy = new THREE.Mesh(new THREE.SphereGeometry(0.32, 10, 8), glassMat);
  canopy.scale.set(1, 0.8, 1.6);
  canopy.position.set(0, 0.28, -1.6);
  group.add(canopy);

  const wing = new THREE.Mesh(new THREE.BoxGeometry(WINGSPAN, 0.09, 0.9), bodyMat);
  wing.position.set(0, 0.05, -0.2);
  group.add(wing);

  const wingTipL = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.09, 0.5), accentMat);
  wingTipL.position.set(-WINGSPAN / 2 + 0.4, 0.05, -0.2);
  group.add(wingTipL);
  const wingTipR = wingTipL.clone();
  wingTipR.position.x = WINGSPAN / 2 - 0.4;
  group.add(wingTipR);

  const stab = new THREE.Mesh(new THREE.BoxGeometry(2.6, 0.06, 0.55), bodyMat);
  stab.position.set(0, 0.15, 2.55);
  group.add(stab);

  const fin = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.75, 0.65), accentMat);
  fin.position.set(0, 0.5, 2.6);
  group.add(fin);

  group.traverse((o) => { if (o.isMesh) { o.castShadow = false; o.receiveShadow = false; } });
  return group;
}
const glider = buildGlider();
scene.add(glider);

/* --- Camera rig --- */
let cameraMode = "chase"; // chase | cockpit
const chaseOffset = new V3(0, 3.2, 11);
const cockpitOffset = new V3(0, 0.55, -1.1);

function updateCamera(dt) {
  const q = glider.quaternion;
  if (cameraMode === "chase") {
    const desired = chaseOffset.clone().applyQuaternion(q).add(glider.position);
    camera.position.lerp(desired, 1 - Math.pow(0.001, dt));
    const lookTarget = glider.position.clone().add(new V3(0, 1, 0));
    camera.up.set(0, 1, 0);
    camera.lookAt(lookTarget);
  } else {
    const desired = cockpitOffset.clone().applyQuaternion(q).add(glider.position);
    camera.position.copy(desired);
    const fwd = LOCAL_FWD.clone().applyQuaternion(q);
    camera.up.copy(LOCAL_UP.clone().applyQuaternion(q));
    camera.lookAt(desired.clone().add(fwd));
  }
}

/* ------------------------------------------------------------------ *
 *  HUD                                                                *
 * ------------------------------------------------------------------ */
const hudSpeed = document.getElementById("hud-speed");
const hudAlt = document.getElementById("hud-alt");
const hudVs = document.getElementById("hud-vs");
const hudHdg = document.getElementById("hud-hdg");
const hudBank = document.getElementById("hud-bank");
const hudPitch = document.getElementById("hud-pitch");
const aiHorizon = document.getElementById("ai-horizon");
const messageBanner = document.getElementById("message-banner");

let messageTimer = null;
function showMessage(text, color) {
  messageBanner.textContent = text;
  messageBanner.style.color = color || "#ffdd55";
  messageBanner.classList.add("show");
  if (messageTimer) clearTimeout(messageTimer);
}
function hideMessage() {
  messageBanner.classList.remove("show");
}

function updateHud() {
  const { pitch, bank, heading } = extractAttitude();
  hudSpeed.textContent = Math.round(state.airspeed * 3.6);
  hudAlt.textContent = Math.max(0, Math.round(state.pos.y));
  hudVs.textContent = state.vel.y.toFixed(1);
  hudHdg.textContent = String(Math.round((heading * 180 / Math.PI)) % 360).padStart(3, "0");
  hudBank.textContent = Math.round(bank * 180 / Math.PI);
  hudPitch.textContent = Math.round(pitch * 180 / Math.PI);

  const pitchPx = THREE.MathUtils.clamp(pitch, -1.3, 1.3) * 70;
  aiHorizon.style.transform = `translate(-50%, -50%) rotate(${bank}rad) translateY(${pitchPx}px)`;
  aiHorizon.style.top = "50%";
  aiHorizon.style.left = "50%";

  if (state.alpha > STALL_ALPHA && state.launched && !state.crashed && !state.landed) {
    showMessage("STALL", "#ff9d3d");
  } else if (!state.crashed && !state.landed && messageBanner.textContent === "STALL") {
    hideMessage();
  }
}

/* ------------------------------------------------------------------ *
 *  Camera toggle / launch buttons                                    *
 * ------------------------------------------------------------------ */
document.getElementById("btn-camera").addEventListener("click", () => {
  cameraMode = cameraMode === "chase" ? "cockpit" : "chase";
});
document.getElementById("btn-launch").addEventListener("click", resetState);

setupJoystick(document.getElementById("stick-left"), input.left);
setupJoystick(document.getElementById("stick-right"), input.right);
setupTilt();

/* ------------------------------------------------------------------ *
 *  Start overlay                                                     *
 * ------------------------------------------------------------------ */
document.getElementById("btn-start").addEventListener("click", async () => {
  const overlay = document.getElementById("start-overlay");
  overlay.classList.add("hidden");
  try {
    if (document.documentElement.requestFullscreen) {
      await document.documentElement.requestFullscreen({ navigationUI: "hide" }).catch(() => {});
    }
  } catch (e) { /* ignore */ }
  resetState();
});

/* ------------------------------------------------------------------ *
 *  Resize                                                            *
 * ------------------------------------------------------------------ */
function onResize() {
  const w = window.innerWidth, h = window.innerHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", onResize);
window.addEventListener("orientationchange", onResize);
onResize();

/* ------------------------------------------------------------------ *
 *  Main loop (fixed-step physics, variable-rate render)               *
 * ------------------------------------------------------------------ */
const FIXED_DT = 1 / 60;
let lastTime = performance.now();
let accumulator = 0;

function frame(now) {
  requestAnimationFrame(frame);
  let dt = (now - lastTime) / 1000;
  lastTime = now;
  dt = Math.min(dt, 0.05);
  accumulator += dt;

  updateControlsFromInput();

  while (accumulator >= FIXED_DT) {
    physicsStep(FIXED_DT);
    accumulator -= FIXED_DT;
  }

  glider.position.copy(state.pos);
  glider.quaternion.copy(state.quat);

  updateCamera(dt);
  updateHud();

  renderer.render(scene, camera);
}
requestAnimationFrame(frame);

// Exposed for debugging / automated checks.
window.__sim = { state, controls, input, extractAttitude, resetState, physicsStep };
})();
