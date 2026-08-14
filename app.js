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
  "KNOW THYSELF\n— DELPHIC MAXIM",
  "I THINK, THEREFORE I AM\n— DESCARTES",
  "THE UNEXAMINED LIFE IS\nNOT WORTH LIVING\n— SOCRATES",
  "MAN IS CONDEMNED\nTO BE FREE\n— SARTRE",
  "WHAT DOES NOT KILL ME\nMAKES ME STRONGER\n— NIETZSCHE",
  "HAPPINESS IS THE\nMEANING OF LIFE\n— ARISTOTLE",
  "I KNOW THAT\nI KNOW NOTHING\n— SOCRATES",
  "YOU HAVE POWER OVER YOUR\nMIND, NOT OUTSIDE EVENTS\n— MARCUS AURELIUS",
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

// thrustZ/strafeX/strafeY are space-level-only (see spacePhysicsStep) -
// aileron/elevator/rudder do double duty as pure RCS rotation there too.
const controls = { aileron: 0, elevator: 0, rudder: 0, airbrake: 0, throttle: 1, thrustZ: 0, strafeX: 0, strafeY: 0 };

const rings = []; // filled in by spawnRings() once the scene exists (always 8, regardless of debug mode - see activeRingCount)
let ringIndex = 0;
let ringsComplete = false; // all rings passed - still need to land to win
let courseWon = false; // landed safely after clearing every ring
// How many of the 8 rings actually need to be cleared this course - normally
// all of them, but debugMode collapses this to 1 on boss levels (see
// spawnRings) so the boss fight can be reached and iterated on quickly
// without re-flying the full ring course every time.
let activeRingCount = 8; // reassigned by spawnRings() before any gameplay logic can consult it

const DEBUG_MODE_KEY = "gliderSimDebugMode";
let debugMode = false;
try { debugMode = localStorage.getItem(DEBUG_MODE_KEY) === "1"; } catch (e) { /* localStorage unavailable - default to off */ }

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
  // 0-3: Germany, Norway, Nepal, and the hidden Deep Space level - LEVELS
  // itself isn't defined yet at this point in the module, so the ceiling is
  // hardcoded here same as it always has been, rather than LEVELS.length-1.
  if (!isNaN(stored)) maxUnlockedLevel = THREE.MathUtils.clamp(stored, 0, 3);
} catch (e) { /* localStorage unavailable (e.g. private browsing) - default to 0 */ }

function unlockNextLevel() {
  const next = currentLevelIndex + 1;
  if (next > LEVELS.length - 1 || next <= maxUnlockedLevel) return;
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
  if (messageTimer) { clearTimeout(messageTimer); messageTimer = null; }
  if (typeof resetBoss === "function") resetBoss();
  spawnRings();
}

function physicsStep(dt) {
  if (LEVELS[currentLevelIndex].isSpace) { spacePhysicsStep(dt); return; }
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
  _aeroForce.addScaledVector(_liftDir, L * boss.liftMultiplier);
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
  if (boss.active) updateBoss(dt);
}

/* ------------------------------------------------------------------ *
 *  Space physics: pure Newtonian 6DOF, RCS thrusters only. No gravity, *
 *  no lift, no drag, no wind, no ground - direct rotational/          *
 *  translational thrust response in body frame, then straight-line    *
 *  integration. Deliberately still gives residual angular velocity a  *
 *  light exponential damping (SPACE_ANGULAR_DAMPING) rather than true *
 *  frictionless drift - reads as the RCS quietly holding attitude     *
 *  when idle, which is far more flyable on a touchscreen than an      *
 *  un-piloted ship spinning forever from the last input.              *
 * ------------------------------------------------------------------ */
const SPACE_PITCH_ACCEL = 1.6;      // rad/s^2 at full elevator deflection
const SPACE_ROLL_ACCEL = 1.6;
const SPACE_YAW_ACCEL = 1.2;
const SPACE_ANGULAR_DAMPING = 0.6;  // 1/s
const SPACE_MAIN_THRUST_ACCEL = 14; // m/s^2, forward/back
const SPACE_STRAFE_ACCEL = 9;       // m/s^2, left/right and up/down RCS

const _spaceBodyAccel = new V3();

function spacePhysicsStep(dt) {
  if (!state.launched || state.crashed || state.landed) return;

  _prevPos.copy(state.pos);
  const prevPos = _prevPos;
  const q = state.quat;

  // --- Rotation: direct RCS torque, no aerodynamic coupling at all ---
  state.angVel.x += controls.elevator * SPACE_PITCH_ACCEL * dt;
  state.angVel.z -= controls.aileron * SPACE_ROLL_ACCEL * dt;
  state.angVel.y -= controls.rudder * SPACE_YAW_ACCEL * dt;
  state.angVel.multiplyScalar(Math.exp(-SPACE_ANGULAR_DAMPING * dt));

  const omega = state.angVel;
  _qOmega.set(omega.x, omega.y, omega.z, 0);
  _qDot.copy(q).multiply(_qOmega);
  q.x += 0.5 * _qDot.x * dt;
  q.y += 0.5 * _qDot.y * dt;
  q.z += 0.5 * _qDot.z * dt;
  q.w += 0.5 * _qDot.w * dt;
  q.normalize();

  // --- Translation: body-frame thrust only, no gravity/lift/drag ---
  _spaceBodyAccel.set(
    controls.strafeX * SPACE_STRAFE_ACCEL,
    controls.strafeY * SPACE_STRAFE_ACCEL,
    -controls.thrustZ * SPACE_MAIN_THRUST_ACCEL
  );
  _spaceBodyAccel.applyQuaternion(q);
  state.vel.addScaledVector(_spaceBodyAccel, dt);
  state.pos.addScaledVector(state.vel, dt);
  state.airspeed = state.vel.length(); // repurposed as plain speed for the space HUD readout

  checkRingCrossing(prevPos, state.pos);
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
  if (boss.active) { boss.active = false; stopBossMusic(); }
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
let rocketNoiseFilter = null, rocketNoiseGain = null, rocketRumbleGain = null;
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

  // Deep Space level: there's no propeller/engine out there, just RCS
  // thrusters - filtered looping noise (a rumbling roar) through a bandpass
  // filter, plus a low sawtooth underneath for body, rather than the
  // sawtooth engine drone above. Both chains always exist; updateEngineSound
  // silences whichever one the current level isn't using.
  const noiseBuffer = audioCtx.createBuffer(1, audioCtx.sampleRate * 2, audioCtx.sampleRate);
  const noiseData = noiseBuffer.getChannelData(0);
  for (let i = 0; i < noiseData.length; i++) noiseData[i] = Math.random() * 2 - 1;
  const rocketNoise = audioCtx.createBufferSource();
  rocketNoise.buffer = noiseBuffer;
  rocketNoise.loop = true;
  rocketNoiseFilter = audioCtx.createBiquadFilter();
  rocketNoiseFilter.type = "bandpass";
  rocketNoiseFilter.frequency.value = 260;
  rocketNoiseFilter.Q.value = 0.7;
  rocketNoiseGain = audioCtx.createGain();
  rocketNoiseGain.gain.value = 0;
  rocketNoise.connect(rocketNoiseFilter);
  rocketNoiseFilter.connect(rocketNoiseGain);
  rocketNoiseGain.connect(masterGain);
  rocketNoise.start();

  const rocketRumbleOsc = audioCtx.createOscillator();
  rocketRumbleOsc.type = "sawtooth";
  rocketRumbleOsc.frequency.value = 45;
  rocketRumbleGain = audioCtx.createGain();
  rocketRumbleGain.gain.value = 0;
  rocketRumbleOsc.connect(rocketRumbleGain);
  rocketRumbleGain.connect(masterGain);
  rocketRumbleOsc.start();

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
  const t = audioCtx.currentTime;

  if (LEVELS[currentLevelIndex].isSpace) {
    // Responds to any thruster firing, not just the main engine - a strafe
    // burst alone should still sound like something's happening.
    const thrustMag = flying ? Math.max(Math.abs(controls.thrustZ), Math.abs(controls.strafeX), Math.abs(controls.strafeY)) : 0;
    rocketNoiseFilter.frequency.setTargetAtTime(260 + 900 * thrustMag, t, 0.12);
    rocketNoiseGain.gain.setTargetAtTime(thrustMag > 0.02 ? 0.05 + 0.22 * thrustMag : 0, t, 0.08);
    rocketRumbleGain.gain.setTargetAtTime(thrustMag > 0.02 ? 0.03 + 0.05 * thrustMag : 0, t, 0.08);
    engineGain.gain.setTargetAtTime(0, t, 0.1);
    return;
  }
  rocketNoiseGain.gain.setTargetAtTime(0, t, 0.1);
  rocketRumbleGain.gain.setTargetAtTime(0, t, 0.1);

  const targetFreq = flying ? 55 + 70 * controls.throttle : 42;
  const targetGain = flying ? 0.05 + 0.09 * controls.throttle : 0;
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

// One-shot dramatic sting for the Red Baron's reveal: a low cinematic boom,
// a dissonant detuned-sawtooth chord stab through a bandpass filter for a
// brassy edge, and a filtered noise "impact" on top.
function playBossStinger() {
  if (!audioCtx || !soundEnabled) return;
  const t = audioCtx.currentTime;

  const boom = audioCtx.createOscillator();
  boom.type = "sine";
  boom.frequency.setValueAtTime(110, t);
  boom.frequency.exponentialRampToValueAtTime(38, t + 0.5);
  const boomGain = audioCtx.createGain();
  boomGain.gain.setValueAtTime(0.0001, t);
  boomGain.gain.exponentialRampToValueAtTime(0.9, t + 0.03);
  boomGain.gain.exponentialRampToValueAtTime(0.0001, t + 0.9);
  boom.connect(boomGain);
  boomGain.connect(masterGain);
  boom.start(t);
  boom.stop(t + 1);

  const chordFreqs = [110, 130.81, 164.81, 220]; // A2 minor-ish stack
  chordFreqs.forEach((f, i) => {
    const osc = audioCtx.createOscillator();
    osc.type = "sawtooth";
    osc.frequency.value = f;
    osc.detune.value = (i - 1.5) * 6;
    const filt = audioCtx.createBiquadFilter();
    filt.type = "bandpass";
    filt.frequency.value = f * 3;
    filt.Q.value = 1.2;
    const g = audioCtx.createGain();
    g.gain.setValueAtTime(0.0001, t);
    g.gain.exponentialRampToValueAtTime(0.22, t + 0.04);
    g.gain.exponentialRampToValueAtTime(0.0001, t + 1.4);
    osc.connect(filt);
    filt.connect(g);
    g.connect(masterGain);
    osc.start(t);
    osc.stop(t + 1.5);
  });

  const bufferSize = Math.floor(audioCtx.sampleRate * 0.5);
  const buffer = audioCtx.createBuffer(1, bufferSize, audioCtx.sampleRate);
  const data = buffer.getChannelData(0);
  for (let i = 0; i < bufferSize; i++) data[i] = (Math.random() * 2 - 1) * (1 - i / bufferSize);
  const noise = audioCtx.createBufferSource();
  noise.buffer = buffer;
  const noiseFilter = audioCtx.createBiquadFilter();
  noiseFilter.type = "highpass";
  noiseFilter.frequency.value = 800;
  const noiseGain = audioCtx.createGain();
  noiseGain.gain.setValueAtTime(0.5, t);
  noiseGain.gain.exponentialRampToValueAtTime(0.001, t + 0.5);
  noise.connect(noiseFilter);
  noiseFilter.connect(noiseGain);
  noiseGain.connect(masterGain);
  noise.start(t);
}

// Looping tense battle ostinato, driven by a simple fixed-tempo scheduler
// (setInterval, not sample-accurate lookahead scheduling - fine for a
// background loop rather than tightly synced music, consistent with the
// rest of this game's audio). Routed through its own gain node into
// masterGain so it still respects the mute toggle and can fade
// independently of the engine drone.
let bossMusicGain = null;
let bossMusicTimer = null;
let bossMusicStep = 0;

function startBossMusic() {
  if (!audioCtx || bossMusicTimer) return;
  bossMusicGain = audioCtx.createGain();
  bossMusicGain.gain.value = 0;
  bossMusicGain.connect(masterGain);
  bossMusicGain.gain.setTargetAtTime(soundEnabled ? 0.5 : 0, audioCtx.currentTime, 0.4);
  bossMusicStep = 0;

  const BEAT = 0.28;
  const bassNotes = [55, 55, 65.41, 55]; // A1, A1, C2, A1 - driving ostinato

  function playStep() {
    if (!audioCtx || !bossMusicGain) return;
    const t = audioCtx.currentTime;
    const note = bassNotes[bossMusicStep % bassNotes.length];
    const osc = audioCtx.createOscillator();
    osc.type = "sawtooth";
    osc.frequency.value = note;
    const filt = audioCtx.createBiquadFilter();
    filt.type = "lowpass";
    filt.frequency.value = 500;
    const g = audioCtx.createGain();
    g.gain.setValueAtTime(0.0001, t);
    g.gain.exponentialRampToValueAtTime(0.6, t + 0.02);
    g.gain.exponentialRampToValueAtTime(0.0001, t + BEAT * 0.9);
    osc.connect(filt);
    filt.connect(g);
    g.connect(bossMusicGain);
    osc.start(t);
    osc.stop(t + BEAT);

    if (bossMusicStep % 4 === 0) {
      const stab = audioCtx.createOscillator();
      stab.type = "sawtooth";
      stab.frequency.value = 220;
      const stabGain = audioCtx.createGain();
      stabGain.gain.setValueAtTime(0.0001, t);
      stabGain.gain.exponentialRampToValueAtTime(0.18, t + 0.02);
      stabGain.gain.exponentialRampToValueAtTime(0.0001, t + BEAT * 1.8);
      stab.connect(stabGain);
      stabGain.connect(bossMusicGain);
      stab.start(t);
      stab.stop(t + BEAT * 2);
    }

    bossMusicStep++;
  }
  playStep();
  bossMusicTimer = setInterval(playStep, BEAT * 1000);
}

function stopBossMusic() {
  if (bossMusicTimer) {
    clearInterval(bossMusicTimer);
    bossMusicTimer = null;
  }
  if (bossMusicGain) {
    const g = bossMusicGain;
    if (audioCtx) g.gain.setTargetAtTime(0, audioCtx.currentTime, 0.3);
    setTimeout(() => { try { g.disconnect(); } catch (e) { /* already gone */ } }, 800);
    bossMusicGain = null;
  }
}

function playBossCannonFire() {
  if (!audioCtx || !soundEnabled) return;
  const t = audioCtx.currentTime;
  const bufferSize = Math.floor(audioCtx.sampleRate * 0.12);
  const buffer = audioCtx.createBuffer(1, bufferSize, audioCtx.sampleRate);
  const data = buffer.getChannelData(0);
  for (let i = 0; i < bufferSize; i++) data[i] = (Math.random() * 2 - 1) * (1 - i / bufferSize);
  const noise = audioCtx.createBufferSource();
  noise.buffer = buffer;
  const filt = audioCtx.createBiquadFilter();
  filt.type = "bandpass";
  filt.frequency.value = 1800;
  filt.Q.value = 0.8;
  const g = audioCtx.createGain();
  g.gain.setValueAtTime(0.5, t);
  g.gain.exponentialRampToValueAtTime(0.001, t + 0.12);
  noise.connect(filt);
  filt.connect(g);
  g.connect(masterGain);
  noise.start(t);
}

function playBossHitSound() {
  if (!audioCtx || !soundEnabled) return;
  const t = audioCtx.currentTime;
  const osc = audioCtx.createOscillator();
  osc.type = "square";
  osc.frequency.setValueAtTime(220, t);
  osc.frequency.exponentialRampToValueAtTime(80, t + 0.18);
  const g = audioCtx.createGain();
  g.gain.setValueAtTime(0.35, t);
  g.gain.exponentialRampToValueAtTime(0.001, t + 0.2);
  osc.connect(g);
  g.connect(masterGain);
  osc.start(t);
  osc.stop(t + 0.25);
}

/* ------------------------------------------------------------------ *
 *  Controls: touch joysticks + optional device tilt                  *
 * ------------------------------------------------------------------ */
const input = { left: { x: 0, y: 0 }, right: { x: 0, y: 0 }, strafe: { x: 0, y: 0 }, throttle: 1 };
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

// beta/gamma from DeviceOrientationEvent are relative to the device's own
// physical (portrait-natural) frame, not the current visual orientation - so
// used raw, tilt controls end up rotated 90 degrees whenever the page is
// actually being viewed in landscape (which, per the manifest, is the only
// orientation this game runs in). Remap them into screen-relative pitch/roll
// using the current screen rotation angle.
function getScreenAngle() {
  if (screen.orientation && typeof screen.orientation.angle === "number") return screen.orientation.angle;
  if (typeof window.orientation === "number") return window.orientation;
  return 0;
}

function tiltAnglesForScreen(beta, gamma, angle) {
  switch (angle) {
    case 90: return { pitch: -gamma, roll: beta };
    case -90:
    case 270: return { pitch: gamma, roll: -beta };
    case 180: return { pitch: -beta, roll: -gamma };
    default: return { pitch: beta, roll: gamma };
  }
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

  // Rotating the phone mid-flight changes what beta/gamma mean relative to
  // the screen, so a stale baseline captured under the old angle would throw
  // the controls off - recalibrate to the new angle instead of carrying it
  // forward.
  window.addEventListener("orientationchange", () => { tiltBaseline = null; });

  window.addEventListener("deviceorientation", (e) => {
    if (!tiltEnabled) return;
    if (e.beta === null || e.gamma === null) return;
    const { pitch, roll } = tiltAnglesForScreen(e.beta, e.gamma, getScreenAngle());
    if (!tiltBaseline) tiltBaseline = { pitch, roll };
    const dPitch = THREE.MathUtils.clamp((pitch - tiltBaseline.pitch) / 30, -1, 1);
    const dRoll = THREE.MathUtils.clamp((roll - tiltBaseline.roll) / 30, -1, 1);
    input.left.y = dPitch;
    input.left.x = dRoll;
  });
}

function updateControlsFromInput() {
  controls.aileron = THREE.MathUtils.clamp(input.left.x, -1, 1);
  // Inverted: stick/tilt up = nose down, stick/tilt down = nose up.
  controls.elevator = THREE.MathUtils.clamp(input.left.y, -1, 1);
  controls.rudder = THREE.MathUtils.clamp(input.right.x, -1, 1);
  if (LEVELS[currentLevelIndex].isSpace) {
    // Space: no drag to bleed speed off with, so "airbrake" is meaningless -
    // the right stick's Y axis becomes bidirectional main thrust instead
    // (up = forward), and a third stick drives pure RCS strafe.
    controls.thrustZ = THREE.MathUtils.clamp(-input.right.y, -1, 1);
    controls.strafeX = THREE.MathUtils.clamp(input.strafe.x, -1, 1);
    controls.strafeY = THREE.MathUtils.clamp(-input.strafe.y, -1, 1);
    controls.throttle = THREE.MathUtils.clamp(controls.thrustZ, 0, 1); // cosmetic only - propeller spin/engine sound reuse this
  } else {
    controls.airbrake = THREE.MathUtils.clamp(input.right.y, 0, 1);
    controls.throttle = THREE.MathUtils.clamp(input.throttle, 0, 1);
  }
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
// masking it instead of just pushing it further away. Kept in its own
// variable (rather than only living on scene.fog) so buildLevel() can null
// it out for the space level, then restore the exact same object when
// switching back to an atmosphere level.
const atmosphereFog = new THREE.Fog(0xbfe0f5, 600, 4200);
scene.fog = atmosphereFog;

// Everything that only makes sense with sky/ground/air - the sky dome,
// distant ground backdrop, and runway - lives here so buildLevel() can hide
// the whole lot in one shot for the space level rather than tracking each
// mesh individually.
const atmosphereGroup = new THREE.Group();
scene.add(atmosphereGroup);

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
atmosphereGroup.add(createSkyDome());

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
atmosphereGroup.add(makeBackdropStrip(BACKDROP_OUTER_HALF * 2, BACKDROP_OUTER_HALF - TERRAIN_HALF, 0, TERRAIN_HALF + (BACKDROP_OUTER_HALF - TERRAIN_HALF) / 2));
atmosphereGroup.add(makeBackdropStrip(BACKDROP_OUTER_HALF * 2, BACKDROP_OUTER_HALF - TERRAIN_HALF, 0, -TERRAIN_HALF - (BACKDROP_OUTER_HALF - TERRAIN_HALF) / 2));
atmosphereGroup.add(makeBackdropStrip(BACKDROP_OUTER_HALF - TERRAIN_HALF, TERRAIN_SIZE, TERRAIN_HALF + (BACKDROP_OUTER_HALF - TERRAIN_HALF) / 2, 0));
atmosphereGroup.add(makeBackdropStrip(BACKDROP_OUTER_HALF - TERRAIN_HALF, TERRAIN_SIZE, -TERRAIN_HALF - (BACKDROP_OUTER_HALF - TERRAIN_HALF) / 2, 0));

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
// The ring course's own footprint (see pickRingPoint below) - backdrop
// mountains must stay clear of this box entirely (not just the runway),
// since their footprints can reach 700m+ radius and are much taller than
// the ring-clearance ceiling can accommodate. Corridor mountains are
// allowed inside it by design (that's the actual in-flight hazard) and stay
// short enough that ring placement can always clear them.
const RING_CORRIDOR_X = 650;
const RING_CORRIDOR_Z_MIN = -3800;
const RING_CORRIDOR_Z_MAX = -50;

function generateMountains(specs) {
  const mountains = [];
  for (const spec of specs) {
    for (let i = 0; i < spec.count; i++) {
      const height = spec.heightMin + Math.random() * (spec.heightMax - spec.heightMin);
      const radius = spec.radiusMin + Math.random() * (spec.radiusMax - spec.radiusMin);
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
        const clearOfRunway = Math.hypot(x, z) >= 480 + radius;
        const clearOfCorridor = spec.zone === "corridor" ||
          Math.abs(x) > RING_CORRIDOR_X + radius ||
          z < RING_CORRIDOR_Z_MIN - radius || z > RING_CORRIDOR_Z_MAX + radius;
        if (clearOfRunway && clearOfCorridor) break;
      } while (tries < 30);
      mountains.push({ x, z, height, radius });
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
    hasBoss: true, // final level - the Red Baron shows up once all rings are cleared
  },
  {
    // Hidden bonus level, unlocked only after beating Nepal (see
    // unlockNextLevel/maxUnlockedLevel) - no terrain, no atmosphere, no
    // aerodynamics at all. isSpace fully reroutes physics (spacePhysicsStep),
    // ring placement (pickRingPointSpace), environment (starGroup instead of
    // terrain), and the win condition (win immediately on the 8th ring,
    // no landing) - see the isSpace checks throughout this file.
    name: "Deep Space", subtitle: "Zero-G · RCS thrusters only",
    isSpace: true,
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
atmosphereGroup.add(runway);
for (let i = 0; i < 6; i++) {
  const stripe = new THREE.Mesh(new THREE.PlaneGeometry(2, 20), new THREE.MeshLambertMaterial({ color: 0xffffff }));
  stripe.rotation.x = -Math.PI / 2;
  stripe.position.set(0, 0.06, -20 - i * 40);
  atmosphereGroup.add(stripe);
}

/* --- Deep Space level: starfield + a distant planet, built once and just  *
 * toggled visible/hidden by buildLevel() rather than rebuilt per switch,   *
 * since neither depends on anything level-specific.                       */
function createStarfield() {
  const STAR_COUNT = 2400;
  const positions = new Float32Array(STAR_COUNT * 3);
  for (let i = 0; i < STAR_COUNT; i++) {
    // Uniform random point on a sphere shell (not a naive lat/long
    // random pair, which would bunch stars up near the poles).
    const u = Math.random(), v = Math.random();
    const theta = 2 * Math.PI * u;
    const phi = Math.acos(2 * v - 1);
    const r = 3800 + Math.random() * 800;
    positions[i * 3] = r * Math.sin(phi) * Math.cos(theta);
    positions[i * 3 + 1] = r * Math.sin(phi) * Math.sin(theta);
    positions[i * 3 + 2] = r * Math.cos(phi);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  const mat = new THREE.PointsMaterial({ color: 0xffffff, size: 2.5, sizeAttenuation: false, fog: false });
  return new THREE.Points(geo, mat);
}

function createSpacePlanet() {
  // Same canvas-texture approach as createSkyDome/createGroundTexture above
  // - a simple banded gradient, no image assets.
  const c = document.createElement("canvas");
  c.width = 4; c.height = 256;
  const ctx = c.getContext("2d");
  const grad = ctx.createLinearGradient(0, 0, 0, 256);
  grad.addColorStop(0, "#8a5fc0");
  grad.addColorStop(0.4, "#a877d0");
  grad.addColorStop(0.6, "#5d3d90");
  grad.addColorStop(1, "#2c1e52");
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, 4, 256);
  const tex = new THREE.CanvasTexture(c);
  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(900, 32, 24),
    new THREE.MeshStandardMaterial({ map: tex, roughness: 0.9, fog: false })
  );
  mesh.position.set(2600, 300, -5200);
  return mesh;
}

const spaceBackgroundColor = new THREE.Color(0x03050d);
const starGroup = new THREE.Group();
starGroup.add(createStarfield());
starGroup.add(createSpacePlanet());
starGroup.visible = false;
scene.add(starGroup);

// Scattered asteroids for visual speed/motion reference, playing the same
// role trees do on the atmosphere levels (see buildMarkers) - rebuilt into
// levelGroup by buildLevel() each time the space level (re)loads, so it's
// disposed automatically along with everything else when switching away.
function buildAsteroidField() {
  const COUNT = 140;
  const geo = new THREE.IcosahedronGeometry(1, 0);
  const mat = new THREE.MeshStandardMaterial({ color: 0x8a8378, roughness: 0.95, flatShading: true });
  const mesh = new THREE.InstancedMesh(geo, mat, COUNT);
  const dummy = new THREE.Object3D();
  for (let i = 0; i < COUNT; i++) {
    dummy.position.set(
      (Math.random() * 2 - 1) * 900,
      LAUNCH_ALT + (Math.random() * 2 - 1) * 700,
      -300 - Math.random() * 2600
    );
    const s = 3 + Math.random() * 9;
    dummy.scale.set(s, s * (0.7 + Math.random() * 0.6), s * (0.7 + Math.random() * 0.6));
    dummy.rotation.set(Math.random() * Math.PI * 2, Math.random() * Math.PI * 2, Math.random() * Math.PI * 2);
    dummy.updateMatrix();
    mesh.setMatrixAt(i, dummy.matrix);
  }
  levelGroup.add(mesh);
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

  // Switching levels (e.g. via the LEVEL button mid-flight) must never leave
  // a boss encounter, active projectiles, or a lift penalty carried over
  // into whatever gets loaded next.
  if (typeof resetBoss === "function") resetBoss();

  while (levelGroup.children.length) {
    const child = levelGroup.children[levelGroup.children.length - 1];
    disposeLevelObject(child);
    levelGroup.remove(child);
  }

  FIELDS.length = 0;
  VILLAGES.length = 0;

  if (level.isSpace) {
    terrainHeight = () => 0; // no ground at all - just a stable baseline for the spawn point
    buildAsteroidField();
  } else {
    terrainHeight = makeTerrainHeightFn(level);
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
  }

  atmosphereGroup.visible = !level.isSpace;
  starGroup.visible = !!level.isSpace;
  scene.background = level.isSpace ? spaceBackgroundColor : skyColor;
  scene.fog = level.isSpace ? null : atmosphereFog;

  const hud = document.getElementById("hud");
  if (hud) hud.classList.toggle("space-mode", !!level.isSpace);
  const hudSpeedLabel = document.getElementById("hud-speed-label");
  const hudSpeedUnit = document.getElementById("hud-speed-unit");
  if (hudSpeedLabel) hudSpeedLabel.textContent = level.isSpace ? "SPD" : "IAS";
  if (hudSpeedUnit) hudSpeedUnit.textContent = level.isSpace ? "m/s" : "km/h";
  const stickRightTag = document.getElementById("stick-right-tag");
  if (stickRightTag) stickRightTag.textContent = level.isSpace ? "YAW / THRUST" : "YAW / AIRBRAKE";

  const hudLevelName = document.getElementById("hud-level-name");
  if (hudLevelName) hudLevelName.textContent = level.name.toUpperCase();
}

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

/* --- Red Baron: final-level boss, a red WWI biplane --- */
function buildBossPlane() {
  const group = new THREE.Group();
  const bodyMat = new THREE.MeshStandardMaterial({ color: 0xaa1a1a, roughness: 0.45 });
  const trimMat = new THREE.MeshStandardMaterial({ color: 0x1a1a1a, roughness: 0.5 });
  const wingMat = new THREE.MeshStandardMaterial({ color: 0x8f1414, roughness: 0.5 });

  const fuselage = new THREE.Mesh(new THREE.CapsuleGeometry(0.4, 4.0, 4, 8), bodyMat);
  fuselage.rotation.x = Math.PI / 2;
  group.add(fuselage);

  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.36, 0.8, 8), trimMat);
  nose.rotation.x = -Math.PI / 2;
  nose.position.z = -2.6;
  group.add(nose);

  const upperWing = new THREE.Mesh(new THREE.BoxGeometry(9, 0.1, 1.1), wingMat);
  upperWing.position.set(0, 1.05, 0);
  group.add(upperWing);

  const lowerWing = new THREE.Mesh(new THREE.BoxGeometry(7.8, 0.1, 1.0), wingMat);
  lowerWing.position.set(0, -0.15, 0.15);
  group.add(lowerWing);

  for (const side of [-1, 1]) {
    const strut = new THREE.Mesh(new THREE.BoxGeometry(0.08, 1.2, 0.08), trimMat);
    strut.position.set(side * 3.1, 0.45, 0.05);
    group.add(strut);
  }

  const stab = new THREE.Mesh(new THREE.BoxGeometry(2.2, 0.06, 0.5), wingMat);
  stab.position.set(0, 0.15, 2.3);
  group.add(stab);

  const fin = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.7, 0.6), wingMat);
  fin.position.set(0, 0.5, 2.35);
  group.add(fin);

  const bossPropeller = buildPropeller();
  bossPropeller.position.set(0, 0, -3.0);
  group.add(bossPropeller);
  group.userData.propeller = bossPropeller;

  group.traverse((o) => { if (o.isMesh) { o.castShadow = false; o.receiveShadow = false; } });
  group.visible = false;
  return group;
}

const bossPlane = buildBossPlane();
scene.add(bossPlane);
const bossPropeller = bossPlane.userData.propeller;

// Picture-in-picture camera: a small chase view locked onto the Baron
// himself, rendered into a corner of the screen while he's active (after
// the reveal cutscene ends). The main camera stays at its normal close
// follow distance throughout - pulling it back to keep him in the main
// view (an earlier approach) made the player's own glider too small and
// distant to land precisely by, which is exactly when it matters most.
// Aspect is set to match the PIP DOM frame's own size in renderBossPip().
const bossCamera = new THREE.PerspectiveCamera(60, 130 / 95, 0.1, 8000);
const bossPipOffset = new V3(0, 4, 13);
const _bossPipDesired = new V3();
const _bossPipLookTarget = new V3();

function updateBossPipCamera() {
  _bossPipDesired.copy(bossPipOffset).applyQuaternion(bossPlane.quaternion).add(bossPlane.position);
  bossCamera.position.copy(_bossPipDesired);
  bossCamera.up.set(0, 1, 0);
  _bossPipLookTarget.copy(bossPlane.position).add(UP_OFFSET);
  bossCamera.lookAt(_bossPipLookTarget);
}

// Pooled cannon-fire projectiles - fixed-size, reused rather than
// allocated per shot. Geometry is pre-rotated so its long axis is local -Z,
// matching quaternionFromForward()'s convention, so orienting a shot each
// firing is just a quaternion assignment.
const BOSS_PROJECTILE_COUNT = 14;
const bossProjectileGeo = new THREE.CapsuleGeometry(0.16, 1.0, 3, 5);
bossProjectileGeo.rotateX(Math.PI / 2);
const bossProjectileMat = new THREE.MeshBasicMaterial({ color: 0xffdd55 });
const bossProjectiles = [];
for (let i = 0; i < BOSS_PROJECTILE_COUNT; i++) {
  const mesh = new THREE.Mesh(bossProjectileGeo, bossProjectileMat);
  mesh.visible = false;
  scene.add(mesh);
  bossProjectiles.push({ mesh, pos: new V3(), vel: new V3(), active: false, life: 0 });
}

// Final-boss encounter state. liftMultiplier is read directly by
// physicsStep's lift calculation (always, at a default of 1 - so this is
// zero-cost when no boss is active) rather than the game special-casing an
// instant "shot down" state: enough hits genuinely zeroes the aircraft's
// lift, and the existing ground-collision/crash logic takes it from there.
const BOSS_SPAWN_DISTANCE = 500;
const BOSS_CRUISE_SPEED = 46;
// How far behind the player the Baron settles once he's caught up - kept
// tighter than a real WWI tail-chase distance would be so the normal chase
// camera (which itself only trails the player by chaseOffset.z=11) can
// actually keep him in frame; see the dynamic camera pullback in
// updateCamera(), which also depends on this staying comfortably under its
// own clamp ceiling.
const BOSS_CHASE_DISTANCE = 35;
const BOSS_FIRE_RANGE = 420;
const BOSS_FIRE_INTERVAL_MIN = 1.8;
const BOSS_FIRE_INTERVAL_MAX = 3.4;
// Slow enough to visibly dodge at the close BOSS_CHASE_DISTANCE range
// (~35-55m out): at the original 190, travel time there was under 0.3s -
// essentially no reaction window regardless of aim. Combined with the aim
// lead/spread below, shots are now a genuine "see it coming and juke"
// threat instead of a near-unavoidable one.
const BOSS_PROJECTILE_SPEED = 90;
const BOSS_AIM_LEAD_FACTOR = 0.4; // how much of the player's current velocity to lead by - partial, not a perfect intercept
const BOSS_AIM_SPREAD = 0.02; // random aim error per shot (unit-vector component jitter) - small enough that a straight-flying target still mostly gets hit, but a few meters of active dodging at the slower projectile speed reliably clears it
const BOSS_HIT_RADIUS = 4.5;
const BOSS_MAX_HITS = 4;
const BOSS_REVEAL_DURATION = 1.5;

const boss = {
  active: false,
  pos: new V3(),
  vel: new V3(),
  quat: new THREE.Quaternion(),
  hits: 0,
  liftMultiplier: 1,
  nextFireTime: 0,
  revealTimer: 0,
};
let bossTime = 0; // sim-seconds accumulated while the boss is active - used for firing cadence instead of wall-clock time, so it stays correct under direct physicsStep-driven testing too

function resetBoss() {
  boss.active = false;
  boss.hits = 0;
  boss.liftMultiplier = 1;
  boss.nextFireTime = 0;
  boss.revealTimer = 0;
  bossTime = 0;
  bossPlane.visible = false;
  for (const p of bossProjectiles) { p.active = false; p.mesh.visible = false; }
  stopBossMusic();
  if (typeof updateBossHud === "function") updateBossHud();
}

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
  let best = null; // lowest-terrain candidate seen, in case nothing clears outright
  for (let attempt = 0; attempt < 16; attempt++) {
    let lateral, forward, altitude;
    if (!prevCursor) {
      lateral = 0;
      forward = -(300 + Math.random() * 150);
      altitude = groundAtSpawn + LAUNCH_ALT - (60 + Math.random() * 70);
    } else {
      lateral = THREE.MathUtils.clamp(prevCursor.x + (Math.random() * 2 - 1) * 170, -420, 420);
      forward = prevCursor.z - (200 + Math.random() * 180);
      altitude = Math.max(groundAtSpawn + 70, prevCursor.y - (25 + Math.random() * 55));
    }
    const groundHere = terrainHeight(lateral, forward);
    const clearAlt = groundHere + RING_MIN_CLEARANCE;
    if (altitude < clearAlt) altitude = clearAlt;
    if (altitude <= RING_ALT_CEILING) return new V3(lateral, altitude, forward);
    if (!best || groundHere < best.groundHere) best = { lateral, forward, altitude, groundHere };
  }
  // Every attempt needed more clearance than the ceiling allows (only
  // possible if a mountain still ended up dominating this whole area) -
  // fall back to whichever attempt had the lowest local terrain, clamped
  // just above *that* ground rather than to a flat altitude, so the ring
  // still never ends up buried below its own footing.
  return new V3(best.lateral, Math.min(best.altitude, Math.max(best.groundHere + RING_MIN_CLEARANCE, RING_ALT_CEILING)), best.forward);
}

// No terrain to keep clear of in space, so the course is just a free 3D
// random walk from the spawn point - clamped to a bounding volume around it
// so the 8-ring course never wanders absurdly far away.
const SPACE_RING_BOUND = 550;

function pickRingPointSpace(prevCursor) {
  if (!prevCursor) {
    return new V3(0, LAUNCH_ALT, -(300 + Math.random() * 150));
  }
  const lateral = THREE.MathUtils.clamp(prevCursor.x + (Math.random() * 2 - 1) * 220, -SPACE_RING_BOUND, SPACE_RING_BOUND);
  const vertical = THREE.MathUtils.clamp(prevCursor.y + (Math.random() * 2 - 1) * 180, LAUNCH_ALT - SPACE_RING_BOUND, LAUNCH_ALT + SPACE_RING_BOUND);
  const forward = prevCursor.z - (220 + Math.random() * 200);
  return new V3(lateral, vertical, forward);
}

function spawnRings() {
  ringIndex = 0;
  ringsComplete = false;
  courseWon = false;
  hideWinBanner();

  const level = LEVELS[currentLevelIndex];
  // Debug mode collapses the course down to just the first ring on boss
  // levels, so the Red Baron fight can be reached and iterated on directly
  // instead of re-flying all 8 rings every time.
  activeRingCount = (debugMode && level.hasBoss) ? 1 : rings.length;
  hudRingTotal.textContent = activeRingCount;

  const points = [];
  if (level.isSpace) {
    let cursor = pickRingPointSpace(null);
    points.push(cursor.clone());
    for (let i = 1; i < activeRingCount; i++) {
      cursor = pickRingPointSpace(cursor);
      points.push(cursor.clone());
    }
  } else {
    const groundAtSpawn = terrainHeight(0, 0);
    let cursor = pickRingPoint(null, groundAtSpawn);
    points.push(cursor.clone());
    for (let i = 1; i < activeRingCount; i++) {
      cursor = pickRingPoint(cursor, groundAtSpawn);
      points.push(cursor.clone());
    }
  }

  for (let i = 0; i < rings.length; i++) {
    const ring = rings[i];
    if (i >= activeRingCount) {
      ring.group.visible = false;
      continue;
    }
    const nextPoint = points[Math.min(i + 1, points.length - 1)];
    let dir;
    if (i < points.length - 1) {
      dir = nextPoint.clone().sub(points[i]).normalize();
    } else if (i > 0) {
      dir = points[i].clone().sub(points[i - 1]).normalize();
    } else {
      // Only one active ring (debug mode) - no adjacent point to derive a
      // direction from, so just face the way the player launches.
      dir = new V3(0, 0, -1);
    }

    ring.center.copy(points[i]);
    ring.normal.copy(dir);
    ring.passed = false;
    ring.group.position.copy(points[i]);
    ring.group.quaternion.copy(quaternionFromForward(dir));
    ring.group.visible = true;
    setRingVisualState(ring, i === 0 ? "active" : "upcoming");
  }
}

const _ringD0 = new V3();
const _ringD1 = new V3();
const _ringHit = new V3();
function checkRingCrossing(prevPos, currPos) {
  if (ringsComplete || ringIndex >= activeRingCount) return;
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
  if (ringIndex < activeRingCount) {
    setRingVisualState(rings[ringIndex], "active");
  } else {
    ringsComplete = true;
    const level = LEVELS[currentLevelIndex];
    if (level.isSpace) {
      // No landing out here - clearing the last ring wins the level outright.
      courseWon = true;
      showWinBanner();
      playVictoryFanfare();
    } else if (level.hasBoss) {
      // Hold off on the banner until the reveal cutscene is done, so it
      // doesn't cover the Baron's entrance.
      if (messageTimer) clearTimeout(messageTimer);
      messageTimer = setTimeout(() => {
        messageTimer = null;
        if (!state.crashed && !state.landed) showMessage("ALL RINGS CLEARED - LAND TO WIN!", "#ffe066");
      }, BOSS_REVEAL_DURATION * 1000);
    } else {
      showMessage("ALL RINGS CLEARED - LAND TO WIN!", "#ffe066");
    }
    spawnBoss();
  }
}

/* --- Red Baron boss encounter: pursuit AI + cannon fire --- */
const _bossFwd = new V3();          // player's forward direction, recomputed each call
const _bossChaseTarget = new V3();
const _bossDesiredDir = new V3();
const _bossWorldUp = new V3();
const _bossRight = new V3();
const _bossUpVec = new V3();
const _bossBasis = new THREE.Matrix4();
const _bossZAxis = new V3();
const _bossDesiredQuat = new THREE.Quaternion();
const _bossOwnFwd = new V3();
const _bossAimPoint = new V3();
const _bossAimDir = new V3();
const _bossMuzzlePos = new V3();

// Deliberately NOT quaternionFromForward() reused: that helper maps local
// +Z to the given direction (correct for the ring/projectile geometry,
// whose "pointy end" is baked in at +Z), but the boss plane is built like
// the glider - local -Z is the nose. Reusing it here would face (and fly)
// the boss backwards. This writes into pre-allocated scratch objects
// instead of allocating new ones, too - runs every physics step while the
// boss is active, so it needs to be allocation-free like the rest of the
// hot path (quaternionFromForward only ever runs on rare, one-off calls).
function bossQuatFromForward(forwardDir, outQuat) {
  _bossWorldUp.set(0, 1, 0);
  if (Math.abs(forwardDir.y) > 0.99) _bossWorldUp.set(1, 0, 0);
  _bossRight.crossVectors(forwardDir, _bossWorldUp).normalize();
  _bossZAxis.copy(forwardDir).negate(); // local +Z maps to -forwardDir, so local -Z (nose) maps to forwardDir
  _bossUpVec.crossVectors(_bossZAxis, _bossRight).normalize();
  _bossBasis.makeBasis(_bossRight, _bossUpVec, _bossZAxis);
  outQuat.setFromRotationMatrix(_bossBasis);
}

function spawnBoss() {
  const level = LEVELS[currentLevelIndex];
  if (!level.hasBoss) return;

  boss.active = true;
  boss.hits = 0;
  boss.liftMultiplier = 1;
  bossTime = 0;
  boss.nextFireTime = 2.2; // a beat of breathing room before the first shot
  boss.revealTimer = BOSS_REVEAL_DURATION;

  _bossFwd.copy(LOCAL_FWD).applyQuaternion(state.quat);
  boss.pos.copy(state.pos).addScaledVector(_bossFwd, -BOSS_SPAWN_DISTANCE);
  boss.pos.y += 20 + Math.random() * 30;
  boss.pos.x += (Math.random() * 2 - 1) * 80;
  bossQuatFromForward(_bossFwd, boss.quat);
  boss.vel.copy(_bossFwd).multiplyScalar(BOSS_CRUISE_SPEED);

  bossPlane.position.copy(boss.pos);
  bossPlane.quaternion.copy(boss.quat);
  bossPlane.visible = true;

  playBossStinger();
  startBossMusic();
  if (typeof updateBossHud === "function") updateBossHud();
}

function updateBoss(dt) {
  bossTime += dt;

  // Chase a point behind-and-above the player - a classic "on your six"
  // pursuit position - rather than homing straight at the player's own
  // position, so the Baron reads as flying a pursuit curve, not teleporting.
  _bossFwd.copy(LOCAL_FWD).applyQuaternion(state.quat);
  _bossChaseTarget.copy(state.pos).addScaledVector(_bossFwd, -BOSS_CHASE_DISTANCE);
  _bossChaseTarget.y += 12;

  // Simple terrain-avoidance: bias the chase target upward if the boss
  // itself is getting close to the ground beneath it, so it climbs instead
  // of flying into a mountainside on Nepal.
  const groundBelowBoss = terrainHeight(boss.pos.x, boss.pos.z);
  if (boss.pos.y < groundBelowBoss + 60) {
    _bossChaseTarget.y = Math.max(_bossChaseTarget.y, groundBelowBoss + 120);
  }

  _bossDesiredDir.copy(_bossChaseTarget).sub(boss.pos);
  const distToTarget = _bossDesiredDir.length();
  if (distToTarget > 1e-3) _bossDesiredDir.multiplyScalar(1 / distToTarget);
  else _bossDesiredDir.copy(_bossFwd);

  bossQuatFromForward(_bossDesiredDir, _bossDesiredQuat);
  // Exponential-decay turn rate, same framerate-independent pattern the
  // chase camera already uses - avoids the boss snapping onto a new heading
  // instantly while still turning briskly.
  boss.quat.slerp(_bossDesiredQuat, 1 - Math.pow(0.0006, dt));

  _bossOwnFwd.copy(LOCAL_FWD).applyQuaternion(boss.quat);
  // Two-phase speed: a fast intercept while still far from the chase target
  // (closing a 500m spawn gap at only a modest cruise-speed multiplier took
  // over 10 seconds in testing - most of the encounter would pass with the
  // Baron barely visible, let alone close enough to threaten), then easing
  // down to a speed that holds a stable trailing position once close rather
  // than continuing to close in and overshoot/oscillate around the player.
  let speed;
  if (distToTarget > 80) {
    speed = THREE.MathUtils.clamp(distToTarget * 0.9, 90, 180);
  } else {
    speed = THREE.MathUtils.clamp(BOSS_CRUISE_SPEED + (distToTarget - 20) * 0.6, BOSS_CRUISE_SPEED * 0.6, BOSS_CRUISE_SPEED * 1.4);
  }
  boss.vel.copy(_bossOwnFwd).multiplyScalar(speed);
  boss.pos.addScaledVector(boss.vel, dt);

  bossPlane.position.copy(boss.pos);
  bossPlane.quaternion.copy(boss.quat);
  bossPropeller.rotation.z += dt * 26;

  if (bossTime >= boss.nextFireTime) {
    if (boss.pos.distanceTo(state.pos) <= BOSS_FIRE_RANGE) fireBossProjectile();
    boss.nextFireTime = bossTime + BOSS_FIRE_INTERVAL_MIN + Math.random() * (BOSS_FIRE_INTERVAL_MAX - BOSS_FIRE_INTERVAL_MIN);
  }

  for (const p of bossProjectiles) {
    if (!p.active) continue;
    p.pos.addScaledVector(p.vel, dt);
    p.life += dt;
    p.mesh.position.copy(p.pos);
    if (p.life > 8) {
      p.active = false;
      p.mesh.visible = false;
      continue;
    }
    if (p.pos.distanceTo(state.pos) <= BOSS_HIT_RADIUS) {
      p.active = false;
      p.mesh.visible = false;
      registerBossHit();
    }
  }
}

function fireBossProjectile() {
  const slot = bossProjectiles.find((p) => !p.active);
  if (!slot) return; // pool exhausted (very unlikely at this fire rate) - just skip this shot

  _bossMuzzlePos.copy(_bossOwnFwd).multiplyScalar(3.2).add(boss.pos);

  // Partial lead on the player's current velocity, plus a bit of random aim
  // error, so shots feel aimed without being a perfect, unavoidable
  // intercept every time.
  const dist = _bossMuzzlePos.distanceTo(state.pos);
  const leadTime = (dist / BOSS_PROJECTILE_SPEED) * BOSS_AIM_LEAD_FACTOR;
  _bossAimPoint.copy(state.pos).addScaledVector(state.vel, leadTime);
  _bossAimDir.copy(_bossAimPoint).sub(_bossMuzzlePos).normalize();
  _bossAimDir.x += (Math.random() * 2 - 1) * BOSS_AIM_SPREAD;
  _bossAimDir.y += (Math.random() * 2 - 1) * BOSS_AIM_SPREAD;
  _bossAimDir.z += (Math.random() * 2 - 1) * BOSS_AIM_SPREAD;
  _bossAimDir.normalize();

  slot.pos.copy(_bossMuzzlePos);
  slot.vel.copy(_bossAimDir).multiplyScalar(BOSS_PROJECTILE_SPEED);
  slot.life = 0;
  slot.active = true;
  slot.mesh.position.copy(slot.pos);
  slot.mesh.quaternion.copy(quaternionFromForward(_bossAimDir));
  slot.mesh.visible = true;
  playBossCannonFire();
}

let bossHitFlashUntil = 0;
function registerBossHit() {
  boss.hits++;
  boss.liftMultiplier = Math.max(0, 1 - boss.hits / BOSS_MAX_HITS);
  bossHitFlashUntil = performance.now() + 700;
  playBossHitSound();
  if (typeof updateBossHud === "function") updateBossHud();
}

/* --- Camera rig --- */
let cameraMode = "chase"; // chase | cockpit
const chaseOffset = new V3(0, 3.2, 11);
const cockpitOffset = new V3(0, 0.55, -1.1);

const _camDesired = new V3();
const _camLookTarget = new V3();
const _camFwd = new V3();
const _camShipUp = new V3();
const UP_OFFSET = new V3(0, 1, 0); // constant, read-only - never mutated

const _bossRevealMid = new V3();
const _bossRevealOffset = new V3();

function updateCamera(dt) {
  // Brief cinematic cut when the boss first appears: a close chase-style
  // shot on the Baron himself, not a wide shot trying to frame both
  // aircraft together - he spawns BOSS_SPAWN_DISTANCE=500 away, so any
  // camera far enough back to fit both planes in frame shrinks each to a
  // handful of pixels against the terrain, effectively invisible despite
  // technically being in the view frustum. A tight shot on just the Baron
  // reads far better as a "here's your villain" reveal beat anyway.
  if (boss.active && boss.revealTimer > 0) {
    boss.revealTimer -= dt;
    _bossOwnFwd.copy(LOCAL_FWD).applyQuaternion(boss.quat);
    _bossRevealOffset.copy(_bossOwnFwd).multiplyScalar(-16);
    _bossRevealOffset.y += 4;
    camera.position.copy(boss.pos).add(_bossRevealOffset);
    camera.up.set(0, 1, 0);
    _bossRevealMid.copy(boss.pos).add(UP_OFFSET);
    camera.lookAt(_bossRevealMid);
    if (camera.fov !== 62) { camera.fov = 62; camera.updateProjectionMatrix(); }
    return;
  }
  if (camera.fov !== 62) { camera.fov = 62; camera.updateProjectionMatrix(); }
  const q = glider.quaternion;
  if (cameraMode === "chase" && LEVELS[currentLevelIndex].isSpace) {
    // A rigid mount bolted to the hull, not a world-up-stabilized rig - out
    // here the ship can point any which way, so "world up" isn't a stable
    // reference to smooth toward at all. Using it (like the atmosphere chase
    // cam below does) made the camera swing wildly as the ship's own
    // orientation drifted away from world up. Locking position/orientation
    // to the ship's own frame every frame, with no smoothing lag, reads as
    // "fixed behind the vehicle" instead - exactly what makes it flyable.
    _camDesired.copy(chaseOffset).applyQuaternion(q).add(glider.position);
    camera.position.copy(_camDesired);
    _camShipUp.copy(LOCAL_UP).applyQuaternion(q);
    camera.up.copy(_camShipUp);
    _camLookTarget.copy(glider.position).addScaledVector(_camShipUp, UP_OFFSET.y);
    camera.lookAt(_camLookTarget);
  } else if (cameraMode === "chase") {
    // Always the normal close offset, even with the boss active - pulling
    // the main camera back to keep him in frame (an earlier approach) made
    // the player's own glider too small and distant to fly precisely by,
    // which matters most exactly when it's hardest: lining up a landing
    // while getting shot at. The Baron is shown in a separate
    // picture-in-picture instead (see updateBossPipCamera/renderBossPip).
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

const bossPipFrameEl = document.getElementById("boss-pip-frame");

// Renders the small Red Baron chase view into a corner of the screen, while
// he's active and past the reveal cutscene (which already shows him
// full-screen). Reads the DOM frame's own on-screen rect each call and
// mirrors it exactly for the WebGL viewport/scissor, rather than
// duplicating the CSS positioning math in JS - guarantees the rendered
// inset and its visible border line up regardless of safe-area insets,
// orientation, or anything else the CSS handles.
function renderBossPip() {
  const active = boss.active && boss.revealTimer <= 0;
  bossPipFrameEl.classList.toggle("show", active);
  if (!active) return;

  updateBossPipCamera();

  const rect = bossPipFrameEl.getBoundingClientRect();
  const canvasWidth = renderer.domElement.clientWidth;
  const canvasHeight = renderer.domElement.clientHeight;
  const x = rect.left;
  const y = canvasHeight - rect.top - rect.height; // CSS top-down -> WebGL bottom-up
  const w = rect.width, h = rect.height;
  if (w <= 0 || h <= 0) return;

  const aspect = w / h;
  if (Math.abs(bossCamera.aspect - aspect) > 1e-3) {
    bossCamera.aspect = aspect;
    bossCamera.updateProjectionMatrix();
  }

  renderer.setScissorTest(true);
  renderer.setViewport(x, y, w, h);
  renderer.setScissor(x, y, w, h);
  renderer.render(scene, bossCamera);
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, canvasWidth, canvasHeight);
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
const hudRingTotal = document.getElementById("hud-ring-total");
hudRingTotal.textContent = rings.length; // overwritten by spawnRings() once a course is picked, in case debug mode shrinks it
const windArrowEl = document.getElementById("wind-arrow");
const hudWindSpeed = document.getElementById("hud-wind-speed");
const runwayIndicatorEl = document.getElementById("runway-indicator");
const runwayArrowEl = document.getElementById("runway-arrow");
const hudRunwayDist = document.getElementById("hud-runway-dist");
const hudBossRow = document.getElementById("hud-boss-row");
const hudBossHits = document.getElementById("hud-boss-hits");
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
  const nextLevel = LEVELS[currentLevelIndex + 1];
  // Beating Nepal unlocks the hidden Deep Space level, but naming it here
  // would spoil the surprise before the player even reopens the level
  // picker - show the same "you beat the game" text as an actual last
  // level would get, and let them find the new tile on their own.
  const revealsSecret = !!(nextLevel && nextLevel.isSpace);
  winSubtitleEl.textContent = (!nextLevel || revealsSecret)
    ? WIN_SUBTITLE
    : `${nextLevel.name.toUpperCase()} UNLOCKED - TAP LEVEL`;
  winBanner.classList.add("show");
}
function hideWinBanner() {
  winBanner.classList.remove("show");
}

function updateHud() {
  const { pitch, bank, heading } = extractAttitude();
  hudSpeed.textContent = LEVELS[currentLevelIndex].isSpace ? Math.round(state.airspeed) : Math.round(state.airspeed * 3.6);
  hudAlt.textContent = Math.max(0, Math.round(state.pos.y));
  hudVs.textContent = state.vel.y.toFixed(1);
  hudHdg.textContent = String(Math.round((heading * 180 / Math.PI)) % 360).padStart(3, "0");
  hudBank.textContent = Math.round(bank * 180 / Math.PI);
  hudPitch.textContent = Math.round(pitch * 180 / Math.PI);

  // The horizon disc rotates opposite to bank - the little aircraft/wings
  // symbol is the fixed reference, so as the real aircraft rolls right, the
  // horizon (relative to that fixed symbol) appears to rotate left, just
  // like a real attitude indicator.
  const pitchPx = THREE.MathUtils.clamp(pitch, -1.3, 1.3) * 70;
  aiHorizon.style.transform = `translate(-50%, -50%) rotate(${-bank}rad) translateY(${pitchPx}px)`;
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

  const flying = state.launched && !state.crashed && !state.landed;
  if (state.alpha > STALL_ALPHA && flying) {
    showMessage("STALL", "#ff9d3d");
  } else if (boss.active && flying && performance.now() < bossHitFlashUntil) {
    showMessage("HIT!", "#ff3b3b");
  } else if (!state.crashed && !state.landed &&
    (messageBanner.textContent === "STALL" || messageBanner.textContent === "HIT!")) {
    if (ringsComplete && !courseWon) {
      showMessage("ALL RINGS CLEARED - LAND TO WIN!", "#ffe066");
    } else {
      hideMessage();
    }
  }

  hudRingCount.textContent = Math.min(ringIndex, activeRingCount);
}

function updateBossHud() {
  hudBossRow.style.display = boss.active ? "" : "none";
  hudBossHits.textContent = boss.hits;
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
setupJoystick(document.getElementById("stick-strafe"), input.strafe);
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
    // The secret level doesn't just show locked like the others - it's
    // absent from the menu entirely until earned, so its existence is
    // itself the surprise.
    if (btn.classList.contains("secret-level")) btn.classList.toggle("hidden-secret", locked);
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

// Debug mode: collapses boss-level ring courses down to just the first ring
// (see spawnRings) so the Red Baron fight can be reached and re-tried
// quickly. Persisted across reloads since it's a testing convenience you'd
// otherwise have to re-enable constantly.
const debugToggleBtn = document.getElementById("btn-debug");
function setDebugMode(enabled) {
  debugMode = enabled;
  try { localStorage.setItem(DEBUG_MODE_KEY, enabled ? "1" : "0"); } catch (e) { /* ignore */ }
  if (debugToggleBtn) debugToggleBtn.classList.toggle("active", enabled);
}
setDebugMode(debugMode); // reflect the persisted value in the UI on load
debugToggleBtn.addEventListener("click", () => setDebugMode(!debugMode));

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

  if (ringIndex < activeRingCount) {
    rings[ringIndex].torusMat.emissiveIntensity = 0.6 + 0.4 * Math.sin(now * 0.006);
  }

  updateCamera(dt);
  updateHud();
  updateEngineSound();

  renderer.render(scene, camera);
  renderBossPip();
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
  getRocketParams: () => ({ noiseGain: rocketNoiseGain.gain.value, rumbleGain: rocketRumbleGain.gain.value, filterFreq: rocketNoiseFilter.frequency.value }),
  // terrainHeight is reassigned per level - expose a wrapper that always
  // delegates to the current one, rather than capturing today's value.
  VILLAGES, FIELDS, terrainHeight: (x, z) => terrainHeight(x, z), terrainSlopeAt, scene,
  LEVELS, buildLevel, getCurrentLevel: () => currentLevelIndex,
  getMaxUnlockedLevel: () => maxUnlockedLevel,
  spawnRings, getRingCenters: () => rings.slice(0, activeRingCount).map((r) => r.center.clone()),
  boss, bossPlane, fireBossProjectile, updateCamera, updateBoss,
  bossCamera, renderBossPip,
  getBossProjectiles: () => bossProjectiles.map((p) => ({ active: p.active, pos: p.pos.clone() })),
  getActiveRingCount: () => activeRingCount,
  isDebugMode: () => debugMode,
  setDebugMode: (v) => setDebugMode(v),
};

// Deferred until the very end of the module - buildLevel()'s resetBoss()
// call reaches into HUD DOM lookups and boss/scene objects declared
// throughout the file, so this needs everything above to have already run
// (requestAnimationFrame(frame) above is safe to call first regardless,
// since the frame callback itself never runs synchronously - only on the
// next paint, well after this line).
buildLevel(0);
})();
