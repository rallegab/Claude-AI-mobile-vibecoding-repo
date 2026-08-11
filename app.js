(function () {
"use strict";

/* ------------------------------------------------------------------ *
 *  Constants: a small single-seat motor glider                       *
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
const CD0 = 0.032;         // higher than a pure sailplane: cowling/prop drag
const K_INDUCED = 1 / (Math.PI * 0.85 * AR);
const AIRBRAKE_CD = 0.9;

const CY_BETA = 0.6;

const CL_AILERON = 0.09;
const CL_P = 0.65;
const CL_BETA = 0.06;       // dihedral effect (roll restoring)

const CM0 = 0.02;
const CM_ALPHA = -0.95;     // pitch static stability
const CM_Q = 17;
const CM_ELEVATOR = 0.16;

const CN_RUDDER = 0.06;
const CN_R = 0.45;
const CN_BETA = 0.13;

// Small engine: at full throttle, roughly offsets cruise drag so the
// aircraft can sustain level flight instead of always sinking, without
// being strong enough to power through a stall or a steep climb.
const MAX_THRUST = 310; // N, at throttle = 1

const STALL_SPEED = Math.sqrt((2 * MASS * G) / (RHO * S_WING * (CL0 + CLALPHA * STALL_ALPHA)));
const LAUNCH_ALT = 400;
const LAUNCH_SPEED = STALL_SPEED * 2.2; // close to the powered trim speed, to minimize the release transient

// Ground collision now samples the active level's terrain height field
// directly (see terrainHeight / checkGround below) rather than a flat plane.

/* ------------------------------------------------------------------ *
 *  Ring course                                                       *
 * ------------------------------------------------------------------ */
const RING_MESSAGES = [
  "CHAVY,\nMAKE ME MORE MONEY",
  "NICK,\nMAKE ME MORE MONEY TOO",
  "YOU TOO -\nMAKE ME MORE MONEY",
  "LUIS, YOU'RE TOO SAD.\nGET HAPPIER!",
  "NICE FLYING!",
  "KEEP IT UP!",
  "ALMOST THERE!",
  "LAST RING -\nMAKE IT RAIN!",
];
const WIN_TITLE = "CONGRATS, PANJIA FOUNDER!";
const WIN_SUBTITLE = "YOU BEAT THE GAME";

const RING_HOLE_RADIUS = 14;
const RING_TUBE_RADIUS = 1.2;

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

// Snapshot of state.pos/quat as of the physics step *before* the most recent
// one. The render loop (which runs at a different, variable rate than the
// fixed 60Hz physics steps) lerps/slerps between this and the current state
// each frame instead of snapping straight to it - otherwise, on any frame
// where the accumulator happens to produce zero or two steps instead of
// one, the glider visibly holds still or jumps.
const renderPrevPos = new V3().copy(state.pos);
const renderPrevQuat = new THREE.Quaternion().copy(state.quat);

const controls = { aileron: 0, elevator: 0, rudder: 0, airbrake: 0, throttle: 1 };

const rings = []; // filled in by spawnRings() once the scene exists
let ringIndex = 0;
let ringsComplete = false; // all rings passed - still need to land to win
let courseWon = false; // landed safely after clearing every ring

/* ------------------------------------------------------------------ *
 *  Wind: a steady breeze plus layered, ever-changing gusts. Horizontal *
 *  only, world frame. Several sine waves at incommensurate frequencies *
 *  are summed per axis so direction and strength both wander smoothly, *
 *  irregularly, and without ever exactly repeating on a short cycle.   *
 * ------------------------------------------------------------------ */
// Both reassigned per level by buildLevel() below - Norway and Nepal blow
// harder than Germany. WIND_MEAN is mutated in place via .set() rather than
// reassigned, since updateWind()'s closure captures it by reference.
const WIND_MEAN = new V3(3, 0, -2);   // steady prevailing breeze, m/s
let WIND_GUST_AMPLITUDE = 4.5;        // m/s, layered on top of the mean

const wind = new V3();
let windTime = 0;

function updateWind(dt) {
  windTime += dt;
  const t = windTime;
  const gx = WIND_GUST_AMPLITUDE * (0.5 * Math.sin(t * 0.08 + 0.4) + 0.3 * Math.sin(t * 0.23 + 2.1) + 0.2 * Math.sin(t * 0.61 + 4.7));
  const gz = WIND_GUST_AMPLITUDE * (0.5 * Math.sin(t * 0.11 + 1.3) + 0.3 * Math.sin(t * 0.31 + 3.6) + 0.2 * Math.sin(t * 0.77 + 0.9));
  wind.set(WIND_MEAN.x + gx, 0, WIND_MEAN.z + gz);
}

const LOCAL_FWD = new V3(0, 0, -1);
const LOCAL_RIGHT = new V3(1, 0, 0);
const LOCAL_UP = new V3(0, 1, 0);
const GRAVITY_FORCE = new V3(0, -MASS * G, 0); // constant, read-only - never mutated

// Scratch objects reused every physicsStep call instead of allocating fresh
// Vector3/Quaternion instances each time. Physics runs at a fixed 60Hz, so
// the naive version was ~10 heap allocations per call, ~600/sec - enough
// GC churn to risk periodic frame hitches on weaker hardware. Safe as long
// as each is only ever "live" within a single synchronous call (true here).
const _prevPos = new V3();
const _qInv = new THREE.Quaternion();
const _relativeVel = new V3();
const _vBody = new V3();
const _dir = new V3();
const _liftDir = new V3();
const _sideDir = new V3();
const _aeroForce = new V3();
const _qOmega = new THREE.Quaternion();
const _qDot = new THREE.Quaternion();

// Swapped out by buildLevel() to the active level's height-field function,
// so everything that samples terrain (ground collision, ring placement,
// scenery) automatically tracks whichever level is currently loaded without
// needing to know about levels itself.
let terrainHeight = () => 0;

// Level progression: which level is loaded, and how far the player has
// unlocked. Persisted in localStorage so progress survives a reload.
let currentLevelIndex = 0;
const LEVEL_UNLOCK_KEY = "gliderSimMaxUnlockedLevel";
let maxUnlockedLevel = 0;
try {
  const stored = parseInt(localStorage.getItem(LEVEL_UNLOCK_KEY), 10);
  if (!isNaN(stored)) maxUnlockedLevel = THREE.MathUtils.clamp(stored, 0, 2);
} catch (e) { /* localStorage unavailable (e.g. private browsing) - default to 0 */ }

function unlockNextLevel() {
  const next = currentLevelIndex + 1;
  if (next > 2 || next <= maxUnlockedLevel) return;
  maxUnlockedLevel = next;
  try { localStorage.setItem(LEVEL_UNLOCK_KEY, String(maxUnlockedLevel)); } catch (e) { /* ignore */ }
  if (typeof refreshLevelSelectUI === "function") refreshLevelSelectUI();
}

function resetState() {
  const groundAtSpawn = terrainHeight(0, 0);
  state.pos.set(0, LAUNCH_ALT + groundAtSpawn, 0);
  const q = new THREE.Quaternion().setFromAxisAngle(new V3(0, 1, 0), 0);
  state.quat.copy(q);
  state.vel.set(0, -0.5, -LAUNCH_SPEED);
  state.angVel.set(0, 0, 0);
  state.launched = true;
  state.crashed = false;
  state.landed = false;
  renderPrevPos.copy(state.pos);
  renderPrevQuat.copy(state.quat);
  input.throttle = 1;
  if (throttleVisualUpdate) throttleVisualUpdate(1);
  hideMessage();
  spawnRings();
}

function physicsStep(dt) {
  if (!state.launched || state.crashed || state.landed) return;

  updateWind(dt);

  _prevPos.copy(state.pos);
  const prevPos = _prevPos;
  const q = state.quat;
  _qInv.copy(q).invert();
  _relativeVel.copy(state.vel).sub(wind); // airflow relative to the moving air mass, not the ground
  _vBody.copy(_relativeVel).applyQuaternion(_qInv);

  const u = -_vBody.z;              // forward speed
  const w = -_vBody.y;              // "downward" component in body frame
  const v = _vBody.x;               // sideways speed
  const V_air = Math.max(_relativeVel.length(), 0.01);

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

  if (_vBody.lengthSq() > 1e-6) { _dir.copy(_vBody).normalize(); } else { _dir.set(0, 0, -1); }
  _liftDir.copy(LOCAL_UP).addScaledVector(_dir, -_dir.dot(LOCAL_UP));
  if (_liftDir.lengthSq() > 1e-6) { _liftDir.normalize(); } else { _liftDir.set(0, 1, 0); }
  _sideDir.copy(LOCAL_RIGHT).addScaledVector(_dir, -_dir.dot(LOCAL_RIGHT));
  if (_sideDir.lengthSq() > 1e-6) { _sideDir.normalize(); } else { _sideDir.set(1, 0, 0); }

  _aeroForce.copy(_dir).multiplyScalar(-D);
  _aeroForce.addScaledVector(_liftDir, L);
  _aeroForce.addScaledVector(_sideDir, Y);
  _aeroForce.addScaledVector(LOCAL_FWD, MAX_THRUST * controls.throttle);
  _aeroForce.applyQuaternion(q);
  _aeroForce.add(GRAVITY_FORCE);
  _aeroForce.multiplyScalar(1 / MASS);
  state.vel.addScaledVector(_aeroForce, dt);

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
  _qOmega.set(omega.x, omega.y, omega.z, 0);
  _qDot.copy(q).multiply(_qOmega);
  q.x += 0.5 * _qDot.x * dt;
  q.y += 0.5 * _qDot.y * dt;
  q.z += 0.5 * _qDot.z * dt;
  q.w += 0.5 * _qDot.w * dt;
  q.normalize();

  state.pos.addScaledVector(state.vel, dt);

  checkRingCrossing(prevPos, state.pos);
  checkGround();
}

const _attFwd = new V3();
const _attRight = new V3();
function extractAttitude() {
  const q = state.quat;
  _attFwd.copy(LOCAL_FWD).applyQuaternion(q);
  _attRight.copy(LOCAL_RIGHT).applyQuaternion(q);
  const pitch = Math.asin(THREE.MathUtils.clamp(_attFwd.y, -1, 1));
  const bank = Math.asin(THREE.MathUtils.clamp(-_attRight.y, -1, 1));
  let heading = Math.atan2(_attFwd.x, -_attFwd.z);
  if (heading < 0) heading += Math.PI * 2;
  return { pitch, bank, heading };
}

// Steepest slope (rise/run) the terrain is allowed to have at the touchdown
// point for a landing to still count as "gentle" rather than a crash, even
// if the aircraft's own vertical speed/attitude were otherwise fine - a
// level touchdown on the side of a mountain still isn't a landing. Germany's
// rolling hills never get anywhere near this (well under 0.1); mountain
// flanks on Norway/Nepal are well past it.
const SAFE_LANDING_SLOPE = 0.18;

function terrainSlopeAt(x, z) {
  const e = 3;
  const hL = terrainHeight(x - e, z), hR = terrainHeight(x + e, z);
  const hD = terrainHeight(x, z - e), hU = terrainHeight(x, z + e);
  return Math.hypot((hR - hL) / (2 * e), (hU - hD) / (2 * e));
}

function checkGround() {
  const groundY = terrainHeight(state.pos.x, state.pos.z);
  if (state.pos.y > groundY + 0.05) return;
  const { pitch, bank } = extractAttitude();
  const vs = state.vel.y;
  const slope = terrainSlopeAt(state.pos.x, state.pos.z);
  const gentle = slope <= SAFE_LANDING_SLOPE && vs > -4.5 && Math.abs(bank) < 0.35 && Math.abs(pitch) < 0.35;
  state.pos.y = groundY;
  state.vel.set(0, 0, 0);
  state.angVel.set(0, 0, 0);
  if (gentle) {
    state.landed = true;
    if (ringsComplete) {
      courseWon = true;
      unlockNextLevel();
      showWinBanner();
      playVictoryFanfare();
    } else {
      showMessage("LANDED", "#7fffb0");
    }
  } else {
    state.crashed = true;
    showMessage("CRASHED", "#ff6b6b");
    playCrashSound();
  }
}

/* ------------------------------------------------------------------ *
 *  Audio: everything synthesized via Web Audio API, no sound assets   *
 * ------------------------------------------------------------------ */
let audioCtx = null;
let masterGain = null;
let engineOsc1 = null, engineOsc2 = null, engineGain = null;
let soundEnabled = true;

// Must be called from a user-gesture handler (mobile autoplay policy).
function initAudio() {
  if (audioCtx) return;
  const Ctx = window.AudioContext || window.webkitAudioContext;
  if (!Ctx) return;
  audioCtx = new Ctx();

  masterGain = audioCtx.createGain();
  masterGain.gain.value = soundEnabled ? 0.6 : 0;
  masterGain.connect(audioCtx.destination);

  // Two slightly detuned sawtooths through a lowpass filter for a buzzy,
  // idle-to-full-throttle engine drone.
  engineOsc1 = audioCtx.createOscillator();
  engineOsc2 = audioCtx.createOscillator();
  engineOsc1.type = "sawtooth";
  engineOsc2.type = "sawtooth";
  engineOsc2.detune.value = 14;
  const engineFilter = audioCtx.createBiquadFilter();
  engineFilter.type = "lowpass";
  engineFilter.frequency.value = 800;
  engineGain = audioCtx.createGain();
  engineGain.gain.value = 0;
  engineOsc1.connect(engineFilter);
  engineOsc2.connect(engineFilter);
  engineFilter.connect(engineGain);
  engineGain.connect(masterGain);
  engineOsc1.start();
  engineOsc2.start();

  audioCtx.resume();
}

function setSoundEnabled(enabled) {
  soundEnabled = enabled;
  if (masterGain) {
    masterGain.gain.setTargetAtTime(enabled ? 0.6 : 0, audioCtx.currentTime, 0.05);
  }
}

function updateEngineSound() {
  if (!audioCtx) return;
  const flying = state.launched && !state.crashed && !state.landed;
  const targetFreq = flying ? 55 + 70 * controls.throttle : 42;
  const targetGain = flying ? 0.05 + 0.09 * controls.throttle : 0;
  const t = audioCtx.currentTime;
  engineOsc1.frequency.setTargetAtTime(targetFreq, t, 0.15);
  engineOsc2.frequency.setTargetAtTime(targetFreq, t, 0.15);
  engineGain.gain.setTargetAtTime(targetGain, t, 0.2);
}

function playRingChime() {
  if (!audioCtx || !soundEnabled) return;
  const t = audioCtx.currentTime;
  const osc = audioCtx.createOscillator();
  const gain = audioCtx.createGain();
  osc.type = "sine";
  osc.frequency.setValueAtTime(1046.5, t);
  osc.frequency.exponentialRampToValueAtTime(1568, t + 0.12);
  gain.gain.setValueAtTime(0.0001, t);
  gain.gain.exponentialRampToValueAtTime(0.35, t + 0.02);
  gain.gain.exponentialRampToValueAtTime(0.0001, t + 0.35);
  osc.connect(gain);
  gain.connect(masterGain);
  osc.start(t);
  osc.stop(t + 0.4);
}

function playCrashSound() {
  if (!audioCtx || !soundEnabled) return;
  const t = audioCtx.currentTime;

  const bufferSize = Math.floor(audioCtx.sampleRate * 0.4);
  const buffer = audioCtx.createBuffer(1, bufferSize, audioCtx.sampleRate);
  const data = buffer.getChannelData(0);
  for (let i = 0; i < bufferSize; i++) data[i] = (Math.random() * 2 - 1) * (1 - i / bufferSize);
  const noise = audioCtx.createBufferSource();
  noise.buffer = buffer;
  const noiseFilter = audioCtx.createBiquadFilter();
  noiseFilter.type = "lowpass";
  noiseFilter.frequency.setValueAtTime(3000, t);
  noiseFilter.frequency.exponentialRampToValueAtTime(200, t + 0.4);
  const noiseGain = audioCtx.createGain();
  noiseGain.gain.setValueAtTime(0.9, t);
  noiseGain.gain.exponentialRampToValueAtTime(0.001, t + 0.4);
  noise.connect(noiseFilter);
  noiseFilter.connect(noiseGain);
  noiseGain.connect(masterGain);
  noise.start(t);

  const thud = audioCtx.createOscillator();
  thud.type = "sine";
  thud.frequency.setValueAtTime(130, t);
  thud.frequency.exponentialRampToValueAtTime(35, t + 0.3);
  const thudGain = audioCtx.createGain();
  thudGain.gain.setValueAtTime(0.7, t);
  thudGain.gain.exponentialRampToValueAtTime(0.001, t + 0.35);
  thud.connect(thudGain);
  thudGain.connect(masterGain);
  thud.start(t);
  thud.stop(t + 0.4);
}

function playVictoryFanfare() {
  if (!audioCtx || !soundEnabled) return;
  const t = audioCtx.currentTime;
  const notes = [523.25, 659.25, 783.99, 1046.5]; // C5 E5 G5 C6
  notes.forEach((freq, i) => {
    const start = t + i * 0.12;
    const osc = audioCtx.createOscillator();
    const gain = audioCtx.createGain();
    osc.type = "triangle";
    osc.frequency.setValueAtTime(freq, start);
    gain.gain.setValueAtTime(0.0001, start);
    gain.gain.exponentialRampToValueAtTime(0.3, start + 0.02);
    gain.gain.exponentialRampToValueAtTime(0.0001, start + 0.5);
    osc.connect(gain);
    gain.connect(masterGain);
    osc.start(start);
    osc.stop(start + 0.55);
  });
}

/* ------------------------------------------------------------------ *
 *  Controls: touch joysticks + optional device tilt                  *
 * ------------------------------------------------------------------ */
const input = { left: { x: 0, y: 0 }, right: { x: 0, y: 0 }, throttle: 1 };
let tiltEnabled = false;
let tiltBaseline = null;
let throttleVisualUpdate = null;

function setupJoystick(rootEl, target) {
  const knob = rootEl.querySelector(".joystick-knob");
  const base = rootEl.querySelector(".joystick-base");
  let active = false;
  let pointerId = null;
  let cachedRect = null; // re-queried once per drag, not on every pointermove
  const radius = 65;

  function move(clientX, clientY) {
    const rect = cachedRect;
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
    cachedRect = base.getBoundingClientRect();
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

// Unlike the joysticks, the throttle holds its position on release rather
// than springing back to center - it's a lever, not a stick.
function setupThrottleSlider(rootEl, target) {
  const fill = rootEl.querySelector(".slider-fill");
  const handle = rootEl.querySelector(".slider-handle");
  const track = rootEl.querySelector(".slider-track");
  let pointerId = null;
  let cachedRect = null; // re-queried once per drag, not on every pointermove

  function applyVisual(value) {
    const pct = THREE.MathUtils.clamp(value, 0, 1) * 100;
    fill.style.height = pct + "%";
    handle.style.bottom = pct + "%";
  }

  function move(clientY) {
    const rect = cachedRect;
    const value = 1 - THREE.MathUtils.clamp((clientY - rect.top) / rect.height, 0, 1);
    target.throttle = value;
    applyVisual(value);
  }

  rootEl.addEventListener("pointerdown", (e) => {
    pointerId = e.pointerId;
    cachedRect = track.getBoundingClientRect();
    rootEl.setPointerCapture(e.pointerId);
    move(e.clientY);
    e.preventDefault();
  });
  rootEl.addEventListener("pointermove", (e) => {
    if (e.pointerId !== pointerId) return;
    move(e.clientY);
    e.preventDefault();
  });
  function end(e) {
    if (e.pointerId !== pointerId) return;
    pointerId = null;
  }
  rootEl.addEventListener("pointerup", end);
  rootEl.addEventListener("pointercancel", end);

  applyVisual(target.throttle);
  return applyVisual;
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
  // Inverted: stick/tilt up = nose down, stick/tilt down = nose up.
  controls.elevator = THREE.MathUtils.clamp(input.left.y, -1, 1);
  controls.rudder = THREE.MathUtils.clamp(input.right.x, -1, 1);
  controls.airbrake = THREE.MathUtils.clamp(input.right.y, 0, 1);
  controls.throttle = THREE.MathUtils.clamp(input.throttle, 0, 1);
}

/* ------------------------------------------------------------------ *
 *  Three.js scene                                                    *
 * ------------------------------------------------------------------ */
const canvas = document.getElementById("scene");
// stencil:false - nothing in this scene uses stencil testing, so skip
// allocating that buffer.
const renderer = new THREE.WebGLRenderer({ canvas, antialias: false, powerPreference: "high-performance", stencil: false });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));

const scene = new THREE.Scene();
const skyColor = new THREE.Color(0x8fc7ea);
scene.background = skyColor;
// Closer, thicker haze than the render distance below actually needs, so the
// far edge of the terrain/backdrop is comfortably hidden well before it -
// masking it instead of just pushing it further away.
scene.fog = new THREE.Fog(0xbfe0f5, 600, 4200);

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
  const geo = new THREE.SphereGeometry(7000, 24, 16);
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
  return tex;
}

/* --- Near terrain: gentle hills, farmland, and villages --- */
function smoothstep(edge0, edge1, x) {
  const t = THREE.MathUtils.clamp((x - edge0) / (edge1 - edge0), 0, 1);
  return t * t * (3 - 2 * t);
}

// Large enough that flying the full 8-ring course (which can reach ~3100m
// from the airfield on its own) plus the return flight to land afterward
// never approaches the edge - the hill wavelengths are long (~1700m+), so
// this stays smooth despite the coarser per-segment size at this extent.
const TERRAIN_SIZE = 14000;
const TERRAIN_HALF = TERRAIN_SIZE / 2;

// Distant flat backdrop, built as four strips framing a hole the exact size
// of the near terrain patch below - rather than a single plane sitting
// underneath it. An overlapping plane plus a tiny offset still z-fights at
// long view distances (depth-buffer precision is poor way out here, with
// an 8000-unit far plane), so the two meshes just never share any ground.
const backdropTexBase = createGroundTexture();
function makeBackdropStrip(width, depth, x, z) {
  const tex = backdropTexBase.clone();
  tex.needsUpdate = true;
  tex.repeat.set(width / 100, depth / 100);
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(width, depth),
    new THREE.MeshLambertMaterial({ map: tex })
  );
  mesh.rotation.x = -Math.PI / 2;
  mesh.position.set(x, 0, z);
  return mesh;
}
const BACKDROP_OUTER_HALF = 15000;
scene.add(makeBackdropStrip(BACKDROP_OUTER_HALF * 2, BACKDROP_OUTER_HALF - TERRAIN_HALF, 0, TERRAIN_HALF + (BACKDROP_OUTER_HALF - TERRAIN_HALF) / 2));
scene.add(makeBackdropStrip(BACKDROP_OUTER_HALF * 2, BACKDROP_OUTER_HALF - TERRAIN_HALF, 0, -TERRAIN_HALF - (BACKDROP_OUTER_HALF - TERRAIN_HALF) / 2));
scene.add(makeBackdropStrip(BACKDROP_OUTER_HALF - TERRAIN_HALF, TERRAIN_SIZE, TERRAIN_HALF + (BACKDROP_OUTER_HALF - TERRAIN_HALF) / 2, 0));
scene.add(makeBackdropStrip(BACKDROP_OUTER_HALF - TERRAIN_HALF, TERRAIN_SIZE, -TERRAIN_HALF - (BACKDROP_OUTER_HALF - TERRAIN_HALF) / 2, 0));

// Farmland fields: rotated rectangles blended into the grass with a soft
// edge. Only populated by buildLevel() for levels with hasFarmland (Germany).
const FIELD_COLORS = [0xd8c15a, 0xc9a63f, 0xdac788, 0xb8935a];
const FIELDS = [];

function fieldBlendAt(x, z) {
  let best = null, bestT = 0;
  const EDGE = 16;
  for (const f of FIELDS) {
    const dx = x - f.cx, dz = z - f.cz;
    const cos = Math.cos(-f.rot), sin = Math.sin(-f.rot);
    const lx = dx * cos - dz * sin, lz = dx * sin + dz * cos;
    const tx = 1 - smoothstep(f.halfW - EDGE, f.halfW, Math.abs(lx));
    const tz = 1 - smoothstep(f.halfD - EDGE, f.halfD, Math.abs(lz));
    const t = Math.min(tx, tz);
    if (t > bestT) { bestT = t; best = f; }
  }
  return best ? { color: best.color, t: bestT } : null;
}

// Villages: cluster centers scattered around the terrain. Only populated by
// buildLevel() for levels with hasFarmland (Germany).
const VILLAGE_COUNT = 6;
const VILLAGES = [];

/* ------------------------------------------------------------------ *
 *  Levels: Germany (unchanged rolling farmland), Norway (higher hills +   *
 *  moderate snow-capped mountains + stronger wind), Nepal (very high,     *
 *  mountain-covered terrain + strongest, most turbulent wind). Each       *
 *  level's terrain height is the same layered-sine hill base as Germany's *
 *  original formula, just amplitude-scaled, plus a sum of dome-shaped     *
 *  mountain bumps for the two harder levels.                              *
 * ------------------------------------------------------------------ */
function hillsBase(x, z, ampScale) {
  return ampScale * (
    12 * Math.sin(x * 0.0011 + 1.3) * Math.cos(z * 0.0009 + 0.7)
    + 8 * Math.sin(x * 0.0023 - 0.6) * Math.sin(z * 0.0017 + 2.1)
    + 5 * Math.cos(x * 0.0037 + 2.8) * Math.cos(z * 0.0031 - 1.4)
  );
}

function mountainBump(x, z, m) {
  const dx = x - m.x, dz = z - m.z;
  const d = Math.hypot(dx, dz);
  if (d >= m.radius) return 0;
  const t = 1 - smoothstep(0, m.radius, d); // 1 at center, 0 at the rim
  return m.height * Math.pow(t, 1.5); // >1 exponent for steeper, more peak-like flanks
}

function makeTerrainHeightFn(level) {
  return function (x, z) {
    const dist = Math.hypot(x, z);
    let h = hillsBase(x, z, level.hillAmpScale);
    for (const m of level.mountains) h += mountainBump(x, z, m);
    const edgeFalloff = 1 - smoothstep(TERRAIN_HALF * 0.72, TERRAIN_HALF * 0.98, dist);
    const runwayFlatten = smoothstep(150, 400, dist);
    return h * edgeFalloff * runwayFlatten;
  };
}

// Scatters mountains from a set of {count, height range, radius range, zone}
// specs. "corridor" mountains sit within/near the ring course's typical
// footprint (real in-flight hazards); "backdrop" mountains are placed at a
// distance chosen to stay mostly inside the fog's full-opacity range (4200)
// so they're still visible as dramatic scenery rather than fogged out
// entirely. Both zones keep clear of the runway/launch area.
function generateMountains(specs) {
  const mountains = [];
  for (const spec of specs) {
    for (let i = 0; i < spec.count; i++) {
      let x, z, tries = 0;
      do {
        if (spec.zone === "corridor") {
          x = (Math.random() * 2 - 1) * 650;
          z = -50 - Math.random() * 3750;
        } else {
          const angle = Math.random() * Math.PI * 2;
          const r = 1400 + Math.random() * 1400;
          x = Math.sin(angle) * r;
          z = Math.cos(angle) * r;
        }
        tries++;
      } while (Math.hypot(x, z) < 480 && tries < 20);
      mountains.push({
        x, z,
        height: spec.heightMin + Math.random() * (spec.heightMax - spec.heightMin),
        radius: spec.radiusMin + Math.random() * (spec.radiusMax - spec.radiusMin),
      });
    }
  }
  return mountains;
}

const LEVELS = [
  {
    name: "Germany", subtitle: "Rolling farmland",
    hillAmpScale: 1, mountains: [],
    windMean: { x: 3, y: 0, z: -2 }, windGust: 4.5,
    hasFarmland: true, treeCount: 260, treeColor: 0x2f4d2a,
    hasSnow: false, groundColor: 0x4f7a3d,
  },
  {
    name: "Norway", subtitle: "High hills & snow peaks",
    hillAmpScale: 3,
    mountains: generateMountains([
      { count: 3, heightMin: 140, heightMax: 260, radiusMin: 240, radiusMax: 380, zone: "corridor" },
      { count: 3, heightMin: 500, heightMax: 820, radiusMin: 380, radiusMax: 600, zone: "backdrop" },
    ]),
    windMean: { x: 5, y: 0, z: -3.5 }, windGust: 7.5,
    hasFarmland: false, treeCount: 90, treeColor: 0x2c4a30,
    hasSnow: true, rockLine: 90, rockBand: 60, snowLine: 190, snowBand: 55,
    groundColor: 0x5a7259, rockColor: 0x847d6f, snowColor: 0xf3f7fb,
  },
  {
    name: "Nepal", subtitle: "Towering peaks, fierce wind",
    hillAmpScale: 5,
    mountains: generateMountains([
      { count: 6, heightMin: 220, heightMax: 460, radiusMin: 260, radiusMax: 420, zone: "corridor" },
      { count: 9, heightMin: 900, heightMax: 1700, radiusMin: 420, radiusMax: 700, zone: "backdrop" },
    ]),
    windMean: { x: 7, y: 0, z: -5 }, windGust: 11,
    hasFarmland: false, treeCount: 0, treeColor: 0x000000,
    hasSnow: true, rockLine: 110, rockBand: 70, snowLine: 220, snowBand: 60,
    groundColor: 0x8a7a5c, rockColor: 0x8f8577, snowColor: 0xf5f8fb,
  },
];

function buildTerrainMesh(level) {
  // Extra resolution only matters (and is only worth the vertex cost) once
  // there are mountains to actually shape - Germany stays at the original
  // 110 segments, unchanged.
  const segs = level.mountains.length > 0 ? 220 : 110;
  const geo = new THREE.PlaneGeometry(TERRAIN_SIZE, TERRAIN_SIZE, segs, segs);
  geo.rotateX(-Math.PI / 2);
  const posAttr = geo.attributes.position;
  const colors = new Float32Array(posAttr.count * 3);
  const baseColor = new THREE.Color(level.groundColor);
  const rockColor = level.hasSnow ? new THREE.Color(level.rockColor) : null;
  const snowColor = level.hasSnow ? new THREE.Color(level.snowColor) : null;
  const tmpColor = new THREE.Color();
  for (let i = 0; i < posAttr.count; i++) {
    const x = posAttr.getX(i);
    const z = posAttr.getZ(i);
    const y = terrainHeight(x, z);
    posAttr.setY(i, y);

    tmpColor.copy(baseColor);
    if (level.hasFarmland) {
      const field = fieldBlendAt(x, z);
      if (field) tmpColor.lerp(field.color, field.t);
    }
    if (level.hasSnow) {
      const tRock = smoothstep(level.rockLine, level.rockLine + level.rockBand, y);
      if (tRock > 0) tmpColor.lerp(rockColor, tRock);
      const tSnow = smoothstep(level.snowLine, level.snowLine + level.snowBand, y);
      if (tSnow > 0) tmpColor.lerp(snowColor, tSnow);
    }
    const jitter = 0.94 + Math.random() * 0.12;
    colors[i * 3] = tmpColor.r * jitter;
    colors[i * 3 + 1] = tmpColor.g * jitter;
    colors[i * 3 + 2] = tmpColor.b * jitter;
  }
  geo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  geo.computeVertexNormals();
  return new THREE.Mesh(geo, new THREE.MeshLambertMaterial({ vertexColors: true }));
}

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

// Everything buildLevel() rebuilds per level (terrain, villages, markers)
// lives under this group, so switching levels is just "dispose and clear
// this group's children, then build the new ones" - the runway, backdrop,
// sky dome and glider stay untouched since they're level-independent.
const levelGroup = new THREE.Group();
scene.add(levelGroup);

function disposeLevelObject(obj) {
  obj.traverse((o) => {
    if (!o.isMesh && !o.isInstancedMesh) return;
    o.geometry.dispose();
    const mats = Array.isArray(o.material) ? o.material : [o.material];
    for (const m of mats) {
      if (m.map) m.map.dispose();
      m.dispose();
    }
  });
}

const WALL_COLORS = [0xc9b28a, 0xb5673a, 0xd8d8d0, 0xcfa96b];
const ROOF_COLORS = [0x7a4632, 0x5a5f68, 0x8a3d30];

function buildVillages(level) {
  const buildingTotal = VILLAGES.reduce((sum, v) => sum + v.buildings, 0);
  if (buildingTotal === 0) return;
  const wallMesh = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshLambertMaterial(), buildingTotal);
  const roofMesh = new THREE.InstancedMesh(new THREE.ConeGeometry(0.8, 1, 4), new THREE.MeshLambertMaterial(), buildingTotal);
  const dummy = new THREE.Object3D();
  const color = new THREE.Color();
  let idx = 0;
  for (const village of VILLAGES) {
    for (let i = 0; i < village.buildings; i++) {
      const a = Math.random() * Math.PI * 2;
      const r = Math.random() * 110;
      const x = village.x + Math.cos(a) * r;
      const z = village.z + Math.sin(a) * r;
      const w = 5 + Math.random() * 7;
      const d = 5 + Math.random() * 7;
      const h = 4 + Math.random() * 10;
      const y = terrainHeight(x, z);
      const rotY = Math.random() * Math.PI * 2;

      dummy.position.set(x, y + h / 2, z);
      dummy.rotation.set(0, rotY, 0);
      dummy.scale.set(w, h, d);
      dummy.updateMatrix();
      wallMesh.setMatrixAt(idx, dummy.matrix);
      wallMesh.setColorAt(idx, color.setHex(WALL_COLORS[idx % WALL_COLORS.length]));

      const roofH = 2.5 + Math.random() * 2;
      dummy.position.set(x, y + h + roofH / 2, z);
      dummy.rotation.set(0, rotY + Math.PI / 4, 0);
      dummy.scale.set(Math.max(w, d) * 0.8, roofH, Math.max(w, d) * 0.8);
      dummy.updateMatrix();
      roofMesh.setMatrixAt(idx, dummy.matrix);
      roofMesh.setColorAt(idx, color.setHex(ROOF_COLORS[idx % ROOF_COLORS.length]));

      idx++;
    }
  }
  wallMesh.instanceColor.needsUpdate = true;
  roofMesh.instanceColor.needsUpdate = true;
  levelGroup.add(wallMesh, roofMesh);
}

// Scattered markers for visual speed/motion reference (trees). On
// snow-capped levels they're kept below the rock line, since trees don't
// grow out of bare rock or snow.
function buildMarkers(level) {
  if (level.treeCount <= 0) return;
  const markerGeo = new THREE.ConeGeometry(3, 10, 5);
  const markerMat = new THREE.MeshLambertMaterial({ color: level.treeColor });
  const markerMesh = new THREE.InstancedMesh(markerGeo, markerMat, level.treeCount);
  const dummy = new THREE.Object3D();
  const treeCeiling = level.hasSnow ? level.rockLine : Infinity;
  for (let i = 0; i < level.treeCount; i++) {
    let x, z, y, tries = 0;
    do {
      const angle = Math.random() * Math.PI * 2;
      const r = 150 + Math.random() * 2800;
      x = Math.cos(angle) * r;
      z = Math.sin(angle) * r;
      y = terrainHeight(x, z);
      tries++;
    } while (y > treeCeiling && tries < 6);
    const scale = 0.6 + Math.random() * 1.6;
    dummy.position.set(x, y + 5 * scale, z);
    dummy.scale.setScalar(scale);
    dummy.rotation.y = Math.random() * Math.PI * 2;
    dummy.updateMatrix();
    markerMesh.setMatrixAt(i, dummy.matrix);
  }
  levelGroup.add(markerMesh);
}

function buildLevel(idx) {
  currentLevelIndex = idx;
  const level = LEVELS[idx];

  while (levelGroup.children.length) {
    const child = levelGroup.children[levelGroup.children.length - 1];
    disposeLevelObject(child);
    levelGroup.remove(child);
  }

  terrainHeight = makeTerrainHeightFn(level);

  FIELDS.length = 0;
  VILLAGES.length = 0;
  if (level.hasFarmland) {
    for (let i = 0; i < 13; i++) {
      const angle = Math.random() * Math.PI * 2;
      const dist = 350 + Math.random() * 2300;
      FIELDS.push({
        cx: Math.sin(angle) * dist,
        cz: Math.cos(angle) * dist,
        halfW: 45 + Math.random() * 90,
        halfD: 40 + Math.random() * 80,
        rot: Math.random() * Math.PI,
        color: new THREE.Color(FIELD_COLORS[i % FIELD_COLORS.length]),
      });
    }
    for (let i = 0; i < VILLAGE_COUNT; i++) {
      const angle = (i / VILLAGE_COUNT) * Math.PI * 2 + (Math.random() - 0.5) * 0.7;
      const dist = 700 + Math.random() * 1900;
      VILLAGES.push({
        x: Math.sin(angle) * dist,
        z: Math.cos(angle) * dist,
        buildings: 6 + Math.floor(Math.random() * 8),
      });
    }
  }

  levelGroup.add(buildTerrainMesh(level));
  buildVillages(level);
  buildMarkers(level);

  WIND_MEAN.set(level.windMean.x, level.windMean.y, level.windMean.z);
  WIND_GUST_AMPLITUDE = level.windGust;

  const hudLevelName = document.getElementById("hud-level-name");
  if (hudLevelName) hudLevelName.textContent = level.name.toUpperCase();
}

buildLevel(0);

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

  const propeller = buildPropeller();
  propeller.position.set(0, 0, -3.5);
  group.add(propeller);
  group.userData.propeller = propeller;

  group.traverse((o) => { if (o.isMesh) { o.castShadow = false; o.receiveShadow = false; } });
  return group;
}

/* Simple 3-blade propeller, spinning about local Z (the fuselage/forward axis). */
function buildPropeller() {
  const group = new THREE.Group();
  const propMat = new THREE.MeshStandardMaterial({ color: 0x2a2a2a, roughness: 0.35, metalness: 0.4 });

  const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.12, 0.18, 10), propMat);
  hub.rotation.x = Math.PI / 2;
  group.add(hub);

  const bladeGeo = new THREE.BoxGeometry(0.11, 0.95, 0.02);
  for (let i = 0; i < 3; i++) {
    const blade = new THREE.Mesh(bladeGeo, propMat);
    blade.position.y = 0.5;
    const pivot = new THREE.Group();
    pivot.rotation.z = (i * Math.PI * 2) / 3;
    pivot.add(blade);
    group.add(pivot);
  }
  return group;
}

const glider = buildGlider();
scene.add(glider);
const propeller = glider.userData.propeller;

/* --- Ring course: five gates to fly through, in order --- */
const RING_COLORS = {
  upcoming: { color: 0x3a5a78, emissive: 0x0c1822, intensity: 0.4 },
  active: { color: 0xffb020, emissive: 0xffb020, intensity: 1 },
  passed: { color: 0x4ce88a, emissive: 0x1f6b3f, intensity: 0.5 },
};

function makeTextTexture(text, size) {
  const c = document.createElement("canvas");
  c.width = c.height = size;
  const ctx = c.getContext("2d");
  ctx.clearRect(0, 0, size, size);

  const forcedLines = text.split("\n");
  const fontSize = forcedLines.length > 1 ? size * 0.11 : size * 0.14;
  ctx.font = `900 ${fontSize}px system-ui, sans-serif`;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";

  const maxWidth = size * 0.82;
  const lines = [];
  for (const forced of forcedLines) {
    const words = forced.split(" ");
    let line = "";
    for (const word of words) {
      const test = line ? line + " " + word : word;
      if (ctx.measureText(test).width > maxWidth && line) {
        lines.push(line);
        line = word;
      } else {
        line = test;
      }
    }
    lines.push(line);
  }

  const lineHeight = fontSize * 1.2;
  const startY = size / 2 - ((lines.length - 1) * lineHeight) / 2;
  ctx.lineJoin = "round";
  ctx.lineWidth = fontSize * 0.16;
  ctx.strokeStyle = "rgba(10,15,20,0.85)";
  ctx.fillStyle = "#ffffff";
  lines.forEach((line, i) => {
    const y = startY + i * lineHeight;
    ctx.strokeText(line, size / 2, y);
    ctx.fillText(line, size / 2, y);
  });

  const tex = new THREE.CanvasTexture(c);
  return tex;
}

function buildRingVisual(message) {
  const group = new THREE.Group();

  const torusGeo = new THREE.TorusGeometry(RING_HOLE_RADIUS + RING_TUBE_RADIUS, RING_TUBE_RADIUS, 12, 32);
  const torusMat = new THREE.MeshStandardMaterial({ color: 0x3a5a78, emissive: 0x0c1822, roughness: 0.4 });
  const torus = new THREE.Mesh(torusGeo, torusMat);
  group.add(torus);

  const planeSize = (RING_HOLE_RADIUS - 1) * 2;
  const textTex = makeTextTexture(message, 512);
  const textMat = new THREE.MeshBasicMaterial({ map: textTex, transparent: true, side: THREE.DoubleSide, depthWrite: false });
  const textPlane = new THREE.Mesh(new THREE.PlaneGeometry(planeSize, planeSize), textMat);
  // Plane's front face defaults to local +Z, but players approach from the
  // -Z side (opposite the ring's direction-of-travel normal) - flip so the
  // text reads correctly on approach rather than mirrored.
  textPlane.rotation.y = Math.PI;
  group.add(textPlane);

  scene.add(group);
  return { group, torus, torusMat, center: new V3(), normal: new V3(0, 0, 1), passed: false };
}

for (const msg of RING_MESSAGES) rings.push(buildRingVisual(msg));

// setFromUnitVectors only constrains the forward axis, leaving roll about it
// arbitrary - build an up-preserving basis instead so rings don't tilt randomly.
function quaternionFromForward(forward) {
  const worldUp = Math.abs(forward.y) > 0.99 ? new V3(1, 0, 0) : new V3(0, 1, 0);
  const right = new V3().crossVectors(worldUp, forward).normalize();
  const up = new V3().crossVectors(forward, right).normalize();
  const m = new THREE.Matrix4().makeBasis(right, up, forward);
  return new THREE.Quaternion().setFromRotationMatrix(m);
}

function setRingVisualState(ring, stateName) {
  const c = RING_COLORS[stateName];
  ring.torusMat.color.setHex(c.color);
  ring.torusMat.emissive.setHex(c.emissive);
  ring.torusMat.emissiveIntensity = c.intensity;
}

// Rings must stay above the local terrain with a safety margin - on
// Germany's gentle hills this basically never engages (terrain there is
// only ever a couple meters at most within the course footprint), but on
// Norway/Nepal it pushes ring altitude up over any mountain that would
// otherwise poke through the course. Retries a fresh random point a few
// times first (rather than just clamping in place), so a ring doesn't end
// up awkwardly hovering right over a peak's summit; falls back to a clamp
// if nothing clearer turns up in time.
const RING_MIN_CLEARANCE = 50;
const RING_ALT_CEILING = LAUNCH_ALT + 220;

function pickRingPoint(prevCursor, groundAtSpawn) {
  let lateral, forward, altitude;
  for (let attempt = 0; attempt < 8; attempt++) {
    if (!prevCursor) {
      lateral = 0;
      forward = -(300 + Math.random() * 150);
      altitude = groundAtSpawn + LAUNCH_ALT - (60 + Math.random() * 70);
    } else {
      lateral = THREE.MathUtils.clamp(prevCursor.x + (Math.random() * 2 - 1) * 170, -420, 420);
      forward = prevCursor.z - (200 + Math.random() * 180);
      altitude = Math.max(groundAtSpawn + 70, prevCursor.y - (25 + Math.random() * 55));
    }
    const clearAlt = terrainHeight(lateral, forward) + RING_MIN_CLEARANCE;
    if (altitude < clearAlt) altitude = clearAlt;
    if (altitude <= RING_ALT_CEILING) return new V3(lateral, altitude, forward);
  }
  return new V3(lateral, Math.min(altitude, RING_ALT_CEILING), forward);
}

function spawnRings() {
  ringIndex = 0;
  ringsComplete = false;
  courseWon = false;
  hideWinBanner();

  const groundAtSpawn = terrainHeight(0, 0);
  const points = [];
  let cursor = pickRingPoint(null, groundAtSpawn);
  points.push(cursor.clone());
  for (let i = 1; i < rings.length; i++) {
    cursor = pickRingPoint(cursor, groundAtSpawn);
    points.push(cursor.clone());
  }

  for (let i = 0; i < rings.length; i++) {
    const ring = rings[i];
    const nextPoint = points[Math.min(i + 1, points.length - 1)];
    const dir = i < points.length - 1
      ? nextPoint.clone().sub(points[i]).normalize()
      : points[i].clone().sub(points[i - 1]).normalize();

    ring.center.copy(points[i]);
    ring.normal.copy(dir);
    ring.passed = false;
    ring.group.position.copy(points[i]);
    ring.group.quaternion.copy(quaternionFromForward(dir));
    setRingVisualState(ring, i === 0 ? "active" : "upcoming");
  }
}

const _ringD0 = new V3();
const _ringD1 = new V3();
const _ringHit = new V3();
function checkRingCrossing(prevPos, currPos) {
  if (ringsComplete || ringIndex >= rings.length) return;
  const ring = rings[ringIndex];
  const d0 = _ringD0.copy(prevPos).sub(ring.center).dot(ring.normal);
  const d1 = _ringD1.copy(currPos).sub(ring.center).dot(ring.normal);
  if ((d0 > 0) === (d1 > 0)) return;
  const t = d0 / (d0 - d1);
  _ringHit.copy(prevPos).lerp(currPos, t);
  if (_ringHit.distanceTo(ring.center) <= RING_HOLE_RADIUS) passRing();
}

function passRing() {
  const ring = rings[ringIndex];
  ring.passed = true;
  setRingVisualState(ring, "passed");
  playRingChime();
  ringIndex++;
  if (ringIndex < rings.length) {
    setRingVisualState(rings[ringIndex], "active");
  } else {
    ringsComplete = true;
    showMessage("ALL RINGS CLEARED - LAND TO WIN!", "#ffe066");
  }
}

/* --- Camera rig --- */
let cameraMode = "chase"; // chase | cockpit
const chaseOffset = new V3(0, 3.2, 11);
const cockpitOffset = new V3(0, 0.55, -1.1);

const _camDesired = new V3();
const _camLookTarget = new V3();
const _camFwd = new V3();
const UP_OFFSET = new V3(0, 1, 0); // constant, read-only - never mutated

function updateCamera(dt) {
  const q = glider.quaternion;
  if (cameraMode === "chase") {
    _camDesired.copy(chaseOffset).applyQuaternion(q).add(glider.position);
    camera.position.lerp(_camDesired, 1 - Math.pow(0.001, dt));
    _camLookTarget.copy(glider.position).add(UP_OFFSET);
    camera.up.set(0, 1, 0);
    camera.lookAt(_camLookTarget);
  } else {
    _camDesired.copy(cockpitOffset).applyQuaternion(q).add(glider.position);
    camera.position.copy(_camDesired);
    _camFwd.copy(LOCAL_FWD).applyQuaternion(q);
    camera.up.copy(LOCAL_UP).applyQuaternion(q);
    _camLookTarget.copy(_camDesired).add(_camFwd);
    camera.lookAt(_camLookTarget);
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
const winBanner = document.getElementById("win-banner");
const winTitleEl = document.getElementById("win-title");
const winSubtitleEl = document.getElementById("win-subtitle");
const hudRingCount = document.getElementById("hud-ring-count");
document.getElementById("hud-ring-total").textContent = rings.length;
const windArrowEl = document.getElementById("wind-arrow");
const hudWindSpeed = document.getElementById("hud-wind-speed");
const runwayIndicatorEl = document.getElementById("runway-indicator");
const runwayArrowEl = document.getElementById("runway-arrow");
const hudRunwayDist = document.getElementById("hud-runway-dist");
const RUNWAY_TARGET = { x: 0, z: -100 }; // center of the runway strip

function normalizeAngle(a) {
  a = a % (Math.PI * 2);
  if (a > Math.PI) a -= Math.PI * 2;
  if (a < -Math.PI) a += Math.PI * 2;
  return a;
}

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

function showWinBanner() {
  winTitleEl.textContent = WIN_TITLE;
  const isLastLevel = currentLevelIndex === LEVELS.length - 1;
  winSubtitleEl.textContent = isLastLevel
    ? WIN_SUBTITLE
    : `${LEVELS[currentLevelIndex + 1].name.toUpperCase()} UNLOCKED - TAP LEVEL`;
  winBanner.classList.add("show");
}
function hideWinBanner() {
  winBanner.classList.remove("show");
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

  // Arrow points in the compass direction the wind is blowing toward, same
  // 0=forward/-Z convention as heading, so it's directly comparable to HDG.
  const windAngle = Math.atan2(wind.x, -wind.z);
  windArrowEl.style.transform = `rotate(${windAngle}rad)`;
  hudWindSpeed.textContent = Math.round(wind.length() * 3.6);

  // Runway direction: shown only once all rings are cleared, since that's
  // when the player needs to find their way back to land. Arrow is relative
  // to the current heading (straight up = runway dead ahead) rather than an
  // absolute compass bearing, so it reads as a direct "turn this way" cue.
  const showRunwayNav = ringsComplete && !courseWon && !state.crashed && !state.landed;
  runwayIndicatorEl.classList.toggle("show", showRunwayNav);
  if (showRunwayNav) {
    const dx = RUNWAY_TARGET.x - state.pos.x;
    const dz = RUNWAY_TARGET.z - state.pos.z;
    const runwayBearing = Math.atan2(dx, -dz);
    runwayArrowEl.style.transform = `rotate(${normalizeAngle(runwayBearing - heading)}rad)`;
    hudRunwayDist.textContent = Math.round(Math.hypot(dx, dz));
  }

  if (state.alpha > STALL_ALPHA && state.launched && !state.crashed && !state.landed) {
    showMessage("STALL", "#ff9d3d");
  } else if (!state.crashed && !state.landed && messageBanner.textContent === "STALL") {
    if (ringsComplete && !courseWon) {
      showMessage("ALL RINGS CLEARED - LAND TO WIN!", "#ffe066");
    } else {
      hideMessage();
    }
  }

  hudRingCount.textContent = Math.min(ringIndex, rings.length);
}

/* ------------------------------------------------------------------ *
 *  Camera toggle / launch buttons                                    *
 * ------------------------------------------------------------------ */
document.getElementById("btn-camera").addEventListener("click", () => {
  cameraMode = cameraMode === "chase" ? "cockpit" : "chase";
});
document.getElementById("btn-launch").addEventListener("click", resetState);
document.getElementById("btn-sound").addEventListener("click", (e) => {
  setSoundEnabled(!soundEnabled);
  e.currentTarget.classList.toggle("active", soundEnabled);
});
document.getElementById("btn-levels").addEventListener("click", openLevelSelect);

setupJoystick(document.getElementById("stick-left"), input.left);
setupJoystick(document.getElementById("stick-right"), input.right);
throttleVisualUpdate = setupThrottleSlider(document.getElementById("throttle"), input);
setupTilt();

/* ------------------------------------------------------------------ *
 *  Start overlay / level select                                      *
 * ------------------------------------------------------------------ */
const levelButtons = Array.from(document.querySelectorAll(".level-btn"));

function refreshLevelSelectUI() {
  for (const btn of levelButtons) {
    const idx = parseInt(btn.dataset.level, 10);
    const locked = idx > maxUnlockedLevel;
    btn.classList.toggle("locked", locked);
    btn.disabled = locked;
  }
}
refreshLevelSelectUI();

// Reopens the level picker at any time, pausing the current flight (physics
// no-ops while state.launched is false, and the engine sound follows suit)
// rather than trying to build a separate pause/resume flow - picking a
// level always (re)starts that level fresh via resetState().
function openLevelSelect() {
  state.launched = false;
  refreshLevelSelectUI();
  document.getElementById("start-overlay").classList.remove("hidden");
}

for (const btn of levelButtons) {
  btn.addEventListener("click", async () => {
    const idx = parseInt(btn.dataset.level, 10);
    if (idx > maxUnlockedLevel) return;
    document.getElementById("start-overlay").classList.add("hidden");
    initAudio();
    try {
      if (document.documentElement.requestFullscreen) {
        await document.documentElement.requestFullscreen({ navigationUI: "hide" }).catch(() => {});
      }
    } catch (e) { /* ignore */ }
    buildLevel(idx);
    resetState();
  });
}

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
    renderPrevPos.copy(state.pos);
    renderPrevQuat.copy(state.quat);
    physicsStep(FIXED_DT);
    accumulator -= FIXED_DT;
  }

  // Interpolate between the last two physics steps by how far into the next
  // (not-yet-due) step the accumulator has drifted, so the glider's visual
  // position/orientation advances smoothly every render frame regardless of
  // how the variable render rate happens to align with the fixed 60Hz steps.
  const renderAlpha = Math.min(accumulator / FIXED_DT, 1);
  glider.position.lerpVectors(renderPrevPos, state.pos, renderAlpha);
  glider.quaternion.copy(renderPrevQuat).slerp(state.quat, renderAlpha);

  if (state.launched && !state.crashed && !state.landed) {
    propeller.rotation.z += dt * (8 + 24 * controls.throttle);
  }

  if (ringIndex < rings.length) {
    rings[ringIndex].torusMat.emissiveIntensity = 0.6 + 0.4 * Math.sin(now * 0.006);
  }

  updateCamera(dt);
  updateHud();
  updateEngineSound();

  renderer.render(scene, camera);
}
requestAnimationFrame(frame);

// Exposed for debugging / automated checks.
window.__sim = {
  state, controls, input, extractAttitude, resetState, physicsStep,
  rings, getRingIndex: () => ringIndex, isCourseWon: () => courseWon,
  isRingsComplete: () => ringsComplete,
  propeller, camera, glider, wind,
  initAudio, playRingChime, playCrashSound, playVictoryFanfare, setSoundEnabled,
  getAudioCtx: () => audioCtx, isSoundEnabled: () => soundEnabled,
  getEngineParams: () => ({ gain: engineGain.gain.value, freq: engineOsc1.frequency.value }),
  // terrainHeight is reassigned per level - expose a wrapper that always
  // delegates to the current one, rather than capturing today's value.
  VILLAGES, FIELDS, terrainHeight: (x, z) => terrainHeight(x, z), terrainSlopeAt, scene,
  LEVELS, buildLevel, getCurrentLevel: () => currentLevelIndex,
  getMaxUnlockedLevel: () => maxUnlockedLevel,
  spawnRings, getRingCenters: () => rings.map((r) => r.center.clone()),
};
})();
