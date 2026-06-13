// picar2 FPV client.
//
// Three loosely-coupled pieces:
//   1. WebRTC video receiver via WHEP (MediaMTX endpoint /cam/whep)
//   2. rosbridge_websocket client → publishes /pan_tilt_controller/commands
//   3. WebXR session for head pose; throttled to PUBLISH_HZ
//
// Coordinate convention:
//   WebXR pose.transform.orientation is a quaternion in headset-local "local"
//   reference space. We pull yaw + pitch out of it, subtract an origin
//   captured on session start (so the user can start facing wherever), then
//   mirror-flip and clamp into the URDF servo limits.
//
// Mirror sign rationale:
//   When the user yaws head LEFT (positive yaw in WebXR's right-handed
//   y-up frame), they expect the camera to pan LEFT too — i.e. the world
//   appears head-locked. The pan_joint axis in our URDF is +Z (up), so
//   pan_cmd = +yaw maps a head-left motion to a camera-left motion.
//   Sign will likely need flipping after first servo install — controlled
//   by PAN_SIGN / TILT_SIGN constants below.

const PUBLISH_HZ        = 30;
const PUBLISH_PERIOD_MS = 1000 / PUBLISH_HZ;
const SOFT_LIMIT        = 1.4;       // just inside URDF ±1.5708 to spare the servo
const DEADBAND          = 0.005;     // rad, suppress jitter publishes
const PAN_SIGN          = -1;        // flipped: head/mouse right → camera right
const TILT_SIGN         = -1;        // mouse/key up → camera up (servo polarity)
// The UI is served from a different port (8443) than MediaMTX (8889), so we
// build absolute URLs for WHEP. The cert is shared so HTTPS works on both.
const MEDIAMTX_PORT     = 8889;
const WHEP_PATH         = `https://${location.hostname}:${MEDIAMTX_PORT}/cam/whep`;
const ROSBRIDGE_PORT    = 5001;
const TOPIC             = '/pan_tilt_controller/commands';
const CMD_VEL_TOPIC     = '/cmd_vel';
const CMD_VEL_TYPE      = 'geometry_msgs/Twist';
const NAV_CANCEL_SVC    = '/navigate_to_pose/_action/cancel_goal';
const NAV_CANCEL_TYPE   = 'action_msgs/srv/CancelGoal';

// Drive limits — match controllers.yaml (desired_linear_vel: 0.40,
// max_reverse_vel: 0.25, max_angular_vel: 1.2). Slightly under angular so
// the controller never refuses a command.
const MAX_LIN_FWD       = 0.40;      // m/s forward
const MAX_LIN_REV       = 0.25;      // m/s reverse
const MAX_ANG           = 1.00;      // rad/s
const CMD_VEL_HZ        = 10;        // matches controller_frequency in nav2.yaml
const CMD_VEL_PERIOD_MS = 1000 / CMD_VEL_HZ;

// PC input sensitivities
const MOUSE_RAD_PER_PX  = 0.004;     // ~0.23°/px — feel-tuned for FPS-style look
const KEY_STEP_RAD      = 0.05;      // arrow / WASD key tap = ~2.9°
const KEY_REPEAT_HZ     = 30;        // when key held, apply step at this rate

// VR video quad size in clip space (1 = fills eye viewport). Below ~0.7
// gives a "monitor across the room" feel; above ~0.9 feels claustrophobic.
// In VR, right thumbstick Y adjusts this live within [VIDEO_SCALE_MIN, MAX].
const VIDEO_SCALE       = 0.6;
const VIDEO_SCALE_MIN   = 0.2;
const VIDEO_SCALE_MAX   = 1.0;
const VIDEO_SCALE_RATE  = 0.5;       // clip-space units per second at full stick
const STICK_DEADZONE    = 0.15;      // ignore drift near the centre

// Shared setpoint state. WebXR loop, mouse drag, and keyboard all write here;
// one publisher reads from here at PUBLISH_HZ and pushes through rosbridge.
const state = { pan: 0, tilt: 0, dirty: true, source: 'idle' };

// True while the PC deadman (Space) is held. While true, mouse drag and
// keyboard WASD don't move pan/tilt — they're repurposed for driving.
let pcDeadman = false;

const $ = id => document.getElementById(id);

// ── HUD helpers ─────────────────────────────────────────────────────────────
function setStatus(el, text, cls) {
  const e = $(el);
  e.textContent = text;
  e.className = 'value ' + (cls || '');
}

// ── 1. WebRTC / WHEP ────────────────────────────────────────────────────────
async function startStream() {
  setStatus('status', 'connecting…');
  const pc = new RTCPeerConnection({ iceServers: [] });
  pc.addTransceiver('video', { direction: 'recvonly' });

  pc.ontrack = (e) => {
    $('cam').srcObject = e.streams[0];
    setStatus('status', 'live', 'ok');
  };
  pc.onconnectionstatechange = () => {
    if (pc.connectionState === 'failed' || pc.connectionState === 'disconnected') {
      setStatus('status', pc.connectionState, 'err');
    }
  };

  const offer = await pc.createOffer();
  await pc.setLocalDescription(offer);

  // WHEP: POST SDP offer, receive SDP answer.
  let res;
  try {
    res = await fetch(WHEP_PATH, {
      method: 'POST',
      headers: { 'Content-Type': 'application/sdp' },
      body: offer.sdp,
    });
  } catch (e) {
    setStatus('status', 'whep fetch failed', 'err');
    throw e;
  }
  if (!res.ok) {
    setStatus('status', `whep ${res.status}`, 'err');
    throw new Error(`WHEP HTTP ${res.status}`);
  }
  const answer = await res.text();
  await pc.setRemoteDescription({ type: 'answer', sdp: answer });
  return pc;
}

// ── 2. rosbridge_websocket ──────────────────────────────────────────────────
// rosbridge listens on plain ws:// not wss://. From an HTTPS origin we still
// try wss:// first (so future TLS-enabled rosbridge "just works"), but with
// an aggressive timeout — otherwise the browser sits ~20 s waiting for the
// TLS handshake to finish before falling back.
const WSS_PROBE_TIMEOUT_MS = 2000;

class RosBridge {
  constructor(host) {
    this.host = host;
    // Skip the wss probe entirely if the page is on plain http (no point).
    this.scheme = location.protocol === 'https:' ? 'wss' : 'ws';
    this.ws = null;
    this.advertised = false;
    this.connect();
  }
  connect() {
    const url = `${this.scheme}://${this.host}:${ROSBRIDGE_PORT}`;
    setStatus('rosStatus', `connecting (${this.scheme})…`);
    this.ws = new WebSocket(url);

    // 2 s probe timer: if the socket hasn't opened by then, force-close so
    // onclose fires and we fail over to ws://.
    let timedOut = false;
    const probeTimer = setTimeout(() => {
      if (this.ws && this.ws.readyState !== WebSocket.OPEN) {
        timedOut = true;
        try { this.ws.close(); } catch {}
      }
    }, WSS_PROBE_TIMEOUT_MS);

    this.ws.onopen = () => {
      clearTimeout(probeTimer);
      this._onopen();
    };
    this.ws.onclose = () => {
      clearTimeout(probeTimer);
      this._onfailover(timedOut);
    };
    this.ws.onerror = () => {};
  }
  _onfailover(timedOut) {
    if (this.scheme === 'wss') {
      // wss probe failed or timed out — fall back to plain ws.
      this.scheme = 'ws';
      setStatus('rosStatus', timedOut ? 'wss timed out → ws://' : 'retry ws://', 'err');
      setTimeout(() => this.connect(), 100);
    } else {
      setStatus('rosStatus', 'closed', 'err');
      setTimeout(() => this.connect(), 2000);
    }
  }
  _onopen() {
    setStatus('rosStatus', 'connected', 'ok');
    this._send({
      op: 'advertise',
      topic: TOPIC,
      type: 'std_msgs/Float64MultiArray',
    });
    this._send({
      op: 'advertise',
      topic: CMD_VEL_TOPIC,
      type: CMD_VEL_TYPE,
    });
    this.advertised = true;
  }
  _send(obj) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(obj));
    }
  }
  publish(pan, tilt) {
    if (!this.advertised) return false;
    this._send({
      op: 'publish',
      topic: TOPIC,
      msg: { data: [pan, tilt] },
    });
    return true;
  }
  publishCmdVel(linear, angular) {
    if (!this.advertised) return false;
    this._send({
      op: 'publish',
      topic: CMD_VEL_TOPIC,
      msg: {
        linear:  { x: linear,  y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular },
      },
    });
    return true;
  }
  cancelNav() {
    // Empty UUID + zero stamp means "cancel all goals" per the
    // action_msgs/srv/CancelGoal contract used by Nav2.
    this._send({
      op: 'call_service',
      service: NAV_CANCEL_SVC,
      type: NAV_CANCEL_TYPE,
      args: {
        goal_info: {
          goal_id: { uuid: new Array(16).fill(0) },
          stamp: { sec: 0, nanosec: 0 },
        },
      },
    });
  }
}

function clamp(v, lo, hi) { return Math.min(hi, Math.max(lo, v)); }

function setSetpoint(pan, tilt, source) {
  const p = clamp(pan,  -SOFT_LIMIT, SOFT_LIMIT);
  const t = clamp(tilt, -SOFT_LIMIT, SOFT_LIMIT);
  if (p !== state.pan || t !== state.tilt) {
    state.pan = p;
    state.tilt = t;
    state.dirty = true;
  }
  state.source = source;
}

function addSetpoint(dpan, dtilt, source) {
  setSetpoint(state.pan + dpan, state.tilt + dtilt, source);
}

// ── 3. Publisher loop — single source of truth ──────────────────────────────
// All inputs (WebXR, mouse, keys) write to `state`; this loop reads it at
// PUBLISH_HZ. Avoids inputs racing each other and keeps STM32 serial sane.
function startPublisher(ros) {
  let publishesThisSecond = 0;
  let rateWindowStart = performance.now();
  let lastPublished = { pan: NaN, tilt: NaN };

  setInterval(() => {
    const now = performance.now();
    const moved = Math.abs(state.pan - lastPublished.pan) > DEADBAND ||
                  Math.abs(state.tilt - lastPublished.tilt) > DEADBAND;
    if (state.dirty && (moved || isNaN(lastPublished.pan))) {
      if (ros.publish(state.pan, state.tilt)) {
        lastPublished = { pan: state.pan, tilt: state.tilt };
        publishesThisSecond++;
      }
      state.dirty = false;
    }
    $('ptVal').textContent = `${state.pan.toFixed(2)} / ${state.tilt.toFixed(2)}`;
    if (now - rateWindowStart >= 1000) {
      $('rateVal').textContent = `${publishesThisSecond} Hz (${state.source})`;
      publishesThisSecond = 0;
      rateWindowStart = now;
    }
  }, PUBLISH_PERIOD_MS);
}

// ── 4. WebXR head pose loop ─────────────────────────────────────────────────
// Yaw / pitch extracted from a quaternion in the standard right-handed,
// y-up coordinate space WebXR uses.  Formulas from the Tait–Bryan ZYX
// decomposition; small-angle gimbal-lock pole at pitch=±π/2 is ignored —
// we clamp pitch well before that.
function quatToYawPitch(q) {
  const { x, y, z, w } = q;
  const yaw   = Math.atan2(2*(w*y + x*z), 1 - 2*(y*y + x*x));
  const sinp  = 2*(w*x - y*z);
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI/2 : Math.asin(sinp);
  return { yaw, pitch };
}

// Compile a fullscreen-quad video renderer. The vertex shader emits a quad
// covering normalized device space; the fragment shader samples the bound
// video texture. Each eye gets the same image (mono FPV) — true stereo would
// need two cameras.
function makeVideoRenderer(gl, videoEl) {
  const vsSrc = `
    attribute vec2 a_pos;
    attribute vec2 a_uv;
    uniform float u_scale;
    varying vec2 v_uv;
    void main() {
      v_uv = a_uv;
      gl_Position = vec4(a_pos * u_scale, 0.0, 1.0);
    }`;
  const fsSrc = `
    precision mediump float;
    varying vec2 v_uv;
    uniform sampler2D u_tex;
    void main() { gl_FragColor = texture2D(u_tex, v_uv); }`;

  const compile = (type, src) => {
    const s = gl.createShader(type);
    gl.shaderSource(s, src); gl.compileShader(s);
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) {
      throw new Error('shader compile: ' + gl.getShaderInfoLog(s));
    }
    return s;
  };
  const prog = gl.createProgram();
  gl.attachShader(prog, compile(gl.VERTEX_SHADER, vsSrc));
  gl.attachShader(prog, compile(gl.FRAGMENT_SHADER, fsSrc));
  gl.linkProgram(prog);
  if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) {
    throw new Error('link: ' + gl.getProgramInfoLog(prog));
  }
  const posLoc   = gl.getAttribLocation(prog, 'a_pos');
  const uvLoc    = gl.getAttribLocation(prog, 'a_uv');
  const texLoc   = gl.getUniformLocation(prog, 'u_tex');
  const scaleLoc = gl.getUniformLocation(prog, 'u_scale');

  // Unit-cube quad in clip space; final size is `a_pos * u_scale` in the
  // vertex shader, so we can change scale live without rebuilding the VBO.
  // UVs always cover the full 0..1 range so the video isn't cropped.
  const vbo = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
  gl.bufferData(gl.ARRAY_BUFFER,
    new Float32Array([
      // pos.xy   uv.xy
      -1, -1,    0, 1,
       1, -1,    1, 1,
      -1,  1,    0, 0,
      -1,  1,    0, 0,
       1, -1,    1, 1,
       1,  1,    1, 0,
    ]),
    gl.STATIC_DRAW);

  const tex = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, tex);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);

  return function drawEye(viewport, scale) {
    // Update texture from the current video frame.
    if (videoEl.readyState >= videoEl.HAVE_CURRENT_DATA) {
      gl.bindTexture(gl.TEXTURE_2D, tex);
      gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, gl.RGB, gl.UNSIGNED_BYTE, videoEl);
    }
    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
    gl.useProgram(prog);
    gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
    // Interleaved: pos.xy (offset 0) + uv.xy (offset 8) = stride 16 bytes.
    gl.enableVertexAttribArray(posLoc);
    gl.vertexAttribPointer(posLoc, 2, gl.FLOAT, false, 16, 0);
    gl.enableVertexAttribArray(uvLoc);
    gl.vertexAttribPointer(uvLoc,  2, gl.FLOAT, false, 16, 8);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, tex);
    gl.uniform1i(texLoc, 0);
    gl.uniform1f(scaleLoc, scale);
    gl.drawArrays(gl.TRIANGLES, 0, 6);
  };
}

async function enterVR(ros) {
  if (!('xr' in navigator)) {
    setStatus('status', 'no WebXR', 'err'); return;
  }
  const supported = await navigator.xr.isSessionSupported('immersive-vr');
  if (!supported) { setStatus('status', 'no immersive-vr', 'err'); return; }

  const session = await navigator.xr.requestSession('immersive-vr', {
    requiredFeatures: ['local'],
  });

  const canvas = document.createElement('canvas');
  const gl = canvas.getContext('webgl', { xrCompatible: true, antialias: false });
  await session.updateRenderState({ baseLayer: new XRWebGLLayer(session, gl) });
  const refSpace = await session.requestReferenceSpace('local');

  const drawEye = makeVideoRenderer(gl, $('cam'));

  // Controller bindings:
  //   LEFT  thumbstick Y  → live video panel scale (zoom-in/out)
  //   LEFT  X button      → end VR session
  //   LEFT  Y button      → cancel any active Nav2 goal (edge-trigger)
  //   RIGHT thumbstick Y  → forward / reverse velocity
  //   RIGHT thumbstick X  → angular velocity
  //   RIGHT grip          → deadman; cmd_vel is forced to zero unless held
  //   RIGHT A button      → recenter pan/tilt servos to (0, 0)
  // gamepad layout (Quest XR Standard):
  //   axes[2,3] = thumbstick X, Y (up = negative Y)
  //   buttons[1] = grip, [4] = X/A (lower face), [5] = Y/B (upper face)
  let scale = VIDEO_SCALE;
  let lastT = 0;
  let prevLX = false;        // left X (exit VR)
  let prevLY = false;        // left Y (nav cancel)
  let prevRA = false;        // right A (recenter)
  let lastCmdVelMs = 0;
  let cmdVelDirty = false;   // suppress zero-spam: only publish (0,0) once after stop
  let deadman = false;       // right-grip held — see head-pose loop below

  function pollControllers(dtSec) {
    let leftPad = null, rightPad = null;
    for (const src of session.inputSources) {
      if (src.handedness === 'left'  && src.gamepad) leftPad  = src.gamepad;
      if (src.handedness === 'right' && src.gamepad) rightPad = src.gamepad;
    }

    // ── Left controller ─────────────────────────────────────────────────
    if (leftPad) {
      // X — exit session.
      const x = leftPad.buttons[4]?.pressed ?? false;
      if (x && !prevLX) { session.end(); return; }
      prevLX = x;

      // Y — cancel nav goal.
      const y = leftPad.buttons[5]?.pressed ?? false;
      if (y && !prevLY) { ros.cancelNav(); }
      prevLY = y;

      // Stick Y — scale.
      const sy = leftPad.axes[3] ?? 0;
      if (Math.abs(sy) >= STICK_DEADZONE) {
        const delta = -sy * VIDEO_SCALE_RATE * dtSec;
        scale = Math.min(VIDEO_SCALE_MAX,
                         Math.max(VIDEO_SCALE_MIN, scale + delta));
      }
    }

    // ── Right controller — drive ────────────────────────────────────────
    if (rightPad) {
      const a    = rightPad.buttons[4]?.pressed ?? false;
      const grip = rightPad.buttons[1]?.pressed ?? false;

      // A — edge-trigger pan/tilt recenter.
      if (a && !prevRA) setSetpoint(0, 0, 'vr-A');
      prevRA = a;

      // Deadman edge transitions:
      //   pressed  → snap pan/tilt to center (forward) for driving;
      //   released → reset head-pose origin so wherever the user is now
      //              looking becomes the new neutral (no camera jump).
      if (grip && !deadman) {
        setSetpoint(0, 0, 'vr-drive');
        origin = null;
      }
      if (!grip && deadman) {
        origin = null;
      }
      deadman = grip;

      // Thumbstick → twist, gated by deadman grip.
      const sx = rightPad.axes[2] ?? 0;
      const sy = rightPad.axes[3] ?? 0;
      const ax = Math.abs(sx) >= STICK_DEADZONE ? sx : 0;
      const ay = Math.abs(sy) >= STICK_DEADZONE ? sy : 0;

      let linear = 0, angular = 0;
      if (grip) {
        // Stick up (negative axis) → forward. Apply asymmetric caps.
        linear  = -ay * (ay < 0 ? MAX_LIN_FWD : MAX_LIN_REV);
        // Stick right (positive axis) → turn right → negative yaw (ROS REP-103).
        angular = -ax * MAX_ANG;
        cmdVelDirty = true;
      }

      const now = performance.now();
      if (now - lastCmdVelMs >= CMD_VEL_PERIOD_MS) {
        if (grip) {
          ros.publishCmdVel(linear, angular);
          lastCmdVelMs = now;
        } else if (cmdVelDirty) {
          // Grip just released — publish one zero to ensure the controller's
          // 500 ms watchdog doesn't keep the last commanded velocity.
          ros.publishCmdVel(0, 0);
          lastCmdVelMs = now;
          cmdVelDirty = false;
        }
      }
    }
  }

  let origin = null;
  session.requestAnimationFrame(function loop(t, frame) {
    const dt = lastT ? (t - lastT) / 1000 : 0;
    lastT = t;

    // 1. Head pose → setpoint, suppressed while the driving deadman is held
    //    so the camera stays centered (facing forward) while moving.
    const pose = frame.getViewerPose(refSpace);
    if (pose && !deadman) {
      const { yaw, pitch } = quatToYawPitch(pose.transform.orientation);
      if (origin === null) origin = { yaw, pitch };
      // Negate yaw delta vs mouse/keyboard: WebXR head-right yields negative
      // yaw, but for VR FPV the user expects head-right → camera-right (true
      // FPS feel), which is opposite of the grab-the-world mouse convention.
      setSetpoint(
        -PAN_SIGN * (yaw   - origin.yaw),
        TILT_SIGN * (pitch - origin.pitch),
        'vr',
      );
    }

    // 2. Controller poll for scale.
    if (dt > 0) pollControllers(dt);

    // 3. Render the video to both eyes (mono — same image both sides).
    const baseLayer = session.renderState.baseLayer;
    gl.bindFramebuffer(gl.FRAMEBUFFER, baseLayer.framebuffer);
    gl.clearColor(0, 0, 0, 1);
    gl.clear(gl.COLOR_BUFFER_BIT);
    if (pose) {
      for (const view of pose.views) {
        drawEye(baseLayer.getViewport(view), scale);
      }
    }
    session.requestAnimationFrame(loop);
  });

  session.addEventListener('end', () => {
    setStatus('status', 'VR ended');
    setSetpoint(0, 0, 'idle');
  });
}

// ── 5. Mouse drag — FPS-style look ──────────────────────────────────────────
// Press-and-drag on the video to pan/tilt. Movement is incremental so the
// servo position persists between drags (like a mouselook camera). Right-
// click drag and pointer-lock could be added; left-drag covers 90% of use.
function installMouseControls() {
  const vid = $('cam');
  let dragging = false;
  let last = { x: 0, y: 0 };

  vid.addEventListener('mousedown', (e) => {
    if (e.button !== 0) return;  // left only
    dragging = true;
    last = { x: e.clientX, y: e.clientY };
    vid.style.cursor = 'grabbing';
    e.preventDefault();
  });
  window.addEventListener('mouseup', () => {
    dragging = false;
    vid.style.cursor = '';
  });
  window.addEventListener('mousemove', (e) => {
    if (!dragging) return;
    const dx = e.clientX - last.x;
    const dy = e.clientY - last.y;
    last = { x: e.clientX, y: e.clientY };
    // While the deadman is held the user is driving — don't smear the camera
    // around with mouse motion. Drag is queued via `last` updates so the
    // next non-deadman drag picks up from the current cursor smoothly.
    if (pcDeadman) return;
    // Direction is controlled by PAN_SIGN / TILT_SIGN at the top of this file
    // so a single sign flip applies to mouse, keyboard, and WebXR identically.
    // Browser dy convention: positive = mouse moved DOWN.
    addSetpoint(
      PAN_SIGN  *  dx * MOUSE_RAD_PER_PX,
      TILT_SIGN * -dy * MOUSE_RAD_PER_PX,
      'mouse',
    );
  });
  vid.style.cursor = 'grab';
}

// ── 6. Keyboard — arrow keys / WASD / Space deadman ─────────────────────────
// Held key applies KEY_STEP_RAD repeatedly at KEY_REPEAT_HZ so it feels like
// continuous motion rather than per-keystroke jumps. 'R' / Home recenters
// pan/tilt. Space is a deadman: while held, WASD drives the car (Twist to
// /cmd_vel at 10 Hz) and the camera is forced to centre; on release, pan/tilt
// control resumes and a single zero Twist is published so the STM32 watchdog
// stops the motors immediately.
function installKeyboardControls(ros) {
  const held = new Set();
  const tick = 1000 / KEY_REPEAT_HZ;
  let lastCmdVelMs = 0;

  // Skip only when typing in a real text field. BUTTONs grab focus after a
  // click, but we still want arrow keys / WASD to drive the camera — let
  // them through and preventDefault stops the browser's default focus nav.
  const isTextField = (el) =>
    el && (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA' || el.isContentEditable);

  function updateDeadman() {
    const next = held.has(' ');
    if (next && !pcDeadman) {
      // Entering drive mode: snap camera centre so the user can see straight.
      setSetpoint(0, 0, 'pc-drive');
    }
    if (!next && pcDeadman) {
      // Leaving drive mode: zero cmd_vel so the watchdog stops the motors.
      ros.publishCmdVel(0, 0);
    }
    pcDeadman = next;
  }

  window.addEventListener('keydown', (e) => {
    if (isTextField(e.target)) return;
    const k = e.key.toLowerCase();
    if (['arrowup','arrowdown','arrowleft','arrowright','w','a','s','d','r','home',' '].includes(k)) {
      e.preventDefault();
      if (document.activeElement && document.activeElement.tagName === 'BUTTON') {
        document.activeElement.blur();
      }
      held.add(k);
      if (k === 'r' || k === 'home') setSetpoint(0, 0, 'keyboard');
      if (k === ' ') updateDeadman();
    }
  });
  window.addEventListener('keyup', (e) => {
    const k = e.key.toLowerCase();
    held.delete(k);
    if (k === ' ') updateDeadman();
  });
  // Lose-focus safety: if the user alt-tabs away with Space held, we'd be
  // stuck commanding velocity forever. Treat blur as "all keys released".
  window.addEventListener('blur', () => {
    held.clear();
    updateDeadman();
  });

  setInterval(() => {
    if (pcDeadman) {
      // Drive mode: arrow keys / WASD → Twist.
      const fwd  = held.has('arrowup')    || held.has('w');
      const rev  = held.has('arrowdown')  || held.has('s');
      const left = held.has('arrowleft')  || held.has('a');
      const right= held.has('arrowright') || held.has('d');
      const linear  = (fwd ? MAX_LIN_FWD : 0) - (rev ? MAX_LIN_REV : 0);
      const angular = (left ? MAX_ANG : 0)    - (right ? MAX_ANG : 0);
      const now = performance.now();
      if (now - lastCmdVelMs >= CMD_VEL_PERIOD_MS) {
        ros.publishCmdVel(linear, angular);
        lastCmdVelMs = now;
      }
      return;
    }
    if (held.size === 0) return;
    // Pan/tilt mode: deltas in the user's frame (right/up positive),
    // signs applied at the boundary.
    let userPan = 0, userTilt = 0;
    if (held.has('arrowright') || held.has('d')) userPan  += KEY_STEP_RAD;
    if (held.has('arrowleft')  || held.has('a')) userPan  -= KEY_STEP_RAD;
    if (held.has('arrowup')    || held.has('w')) userTilt += KEY_STEP_RAD;
    if (held.has('arrowdown')  || held.has('s')) userTilt -= KEY_STEP_RAD;
    if (userPan || userTilt) {
      addSetpoint(PAN_SIGN * userPan, TILT_SIGN * userTilt, 'keyboard');
    }
  }, tick);
}

// ── 7. Center button — quick servo home ────────────────────────────────────
function installCenterButton() {
  const btn = $('center');
  if (btn) btn.onclick = () => setSetpoint(0, 0, 'button');
}

// ── Bootstrap ───────────────────────────────────────────────────────────────
(async function main() {
  const ros = new RosBridge(location.hostname);
  await startStream().catch(() => {});
  startPublisher(ros);

  installMouseControls();
  installKeyboardControls(ros);
  installCenterButton();

  if ('xr' in navigator) {
    const ok = await navigator.xr.isSessionSupported('immersive-vr').catch(() => false);
    $('enterVR').disabled = !ok;
    $('enterVR').onclick = () => enterVR(ros);
  } else {
    $('enterVR').textContent = 'no WebXR';
  }
})();
