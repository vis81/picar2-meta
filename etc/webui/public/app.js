'use strict';

// Phone UI for map building. Talks to server.py over plain fetch — status at
// 2 Hz, map only when its sequence number changes, drive at 10 Hz while the
// joystick is held.

const $ = (id) => document.getElementById(id);
const canvas = $('map');
const ctx = canvas.getContext('2d');

// The joystick's full deflection, reported by the server so it tracks the
// top speed set in Settings. These are only the values used before the
// first status arrives; the server clamps regardless, so they cannot
// command more than it allows.
let driveLimits = [0.40, 1.18];

let mapData = null;        // {w, h, res, ox, oy, img}
// Marked cells of the local costmap, as map-frame metres [x0,y0,x1,y1,...].
// What the robot currently believes is in its way, which the static map
// cannot show — a false obstacle looks like empty floor there.
let obstacles = null;
// Per-device, so it belongs in this browser rather than on the robot: two
// people looking at the same robot can reasonably want different overlays.
let showObstacles = localStorage.getItem('showObstacles') !== '0';
// Off by default: the plan is a debugging view, and on a small screen it
// competes with the route the person actually drew.
let showPlan = localStorage.getItem('showPlan') === '1';
let plan = null;
let slamBackend = 'cartographer';   // cartographer | slam_toolbox
let obstacleCell = 0.05;   // costmap resolution, replaced by the response
let mapSeq = -1;
let pose = null;
let modes = {};
let detail = {};
let phase = 'idle';
let exploreStatus = null;
let mode = 'idle';         // idle | mapping | localize — the server owns this
let mapName = null;        // localize: which saved map is loaded
let localized = false;
let maps = [];             // saved maps, for the picker
let lastSaved = null;      // preselect what you just saved when localizing
let armed = null;          // null | 'pose' — a map tap is being awaited
// The same UI serves a phone and a desktop browser, and "tap" is wrong on
// one of them. Decided once from the pointer, not from the user agent.
const HAS_MOUSE = window.matchMedia('(hover: hover) and (pointer: fine)').matches;
const TAP = HAS_MOUSE ? 'Click' : 'Tap';
let drag = null;           // {a, p} in grid cells while placing
let nav = { state: 'idle', goal: null, distance: null };
let navReady = false;      // nav2 up and able to take a goal
let maxSpeed = null;       // controller's top speed, null until nav2 reports
let speedRange = [0.1, 0.6];
let route = { waypoints: [], active: false, loop: false, flow: false, index: null,
              passed: 0, error: null };
let view = { scale: 1, tx: 0, ty: 0, fitted: false };

// ── map rendering ─────────────────────────────────────────────────────

function gridToImage(bytes, w, h) {
  const img = ctx.createImageData(w, h);
  const d = img.data;
  for (let i = 0; i < w * h; i++) {
    // ROS grids are row-major from the bottom-left; canvas rows go top-down.
    const row = h - 1 - Math.floor(i / w);
    const dst = (row * w + (i % w)) * 4;
    const v = bytes[i];
    let r, g, b;
    // Cartographer publishes occupancy probability, so free space is ~42 and
    // walls ~68 — not the 0/100 a map_server grid uses. Split at the 50
    // midpoint, or the whole map renders as undifferentiated grey.
    if (v === 255) { r = 42; g = 47; b = 55; }            // unknown
    else if (v >= 60) { r = 12; g = 15; b = 20; }         // occupied
    else if (v < 50) { r = 232; g = 234; b = 237; }       // free
    else { r = g = b = 120; }                             // uncertain
    d[dst] = r; d[dst + 1] = g; d[dst + 2] = b; d[dst + 3] = 255;
  }
  return img;
}

async function fetchMap() {
  const r = await fetch('/api/map');
  if (!r.ok) return;
  const h = r.headers;
  const w = +h.get('X-Map-Width'), ht = +h.get('X-Map-Height');
  const bytes = new Uint8Array(await r.arrayBuffer());
  if (!w || !ht || bytes.length < w * ht) return;

  const off = document.createElement('canvas');
  off.width = w; off.height = ht;
  off.getContext('2d').putImageData(gridToImage(bytes, w, ht), 0, 0);

  mapData = {
    w, h: ht,
    res: +h.get('X-Map-Resolution'),
    ox: +h.get('X-Map-Origin-X'),
    oy: +h.get('X-Map-Origin-Y'),
    img: off,
  };
  if (!view.fitted) fitView();
  $('hint').classList.add('hidden');
}

function fitView() {
  if (!mapData) return;
  const pad = 40;
  const s = Math.min(
    (canvas.width - pad) / mapData.w,
    (canvas.height - pad) / mapData.h);
  view.scale = s;
  view.tx = (canvas.width - mapData.w * s) / 2;
  view.ty = (canvas.height - mapData.h * s) / 2;
  view.fitted = true;
}

async function fetchObstacles() {
  if (!showObstacles) { obstacles = null; return; }
  try {
    const r = await fetch('/api/costmap');
    if (!r.ok) { obstacles = null; return; }
    obstacleCell = parseFloat(r.headers.get('X-Cell')) || obstacleCell;
    obstacles = new Float32Array(await r.arrayBuffer());
  } catch (e) {
    obstacles = null;            // a dropped poll should not freeze the view
  }
}

function renderBattery(b) {
  const el = $('batt');
  // Hidden rather than showing a dash: the reading goes stale when the link
  // to the STM32 drops, and a stale number is worse than an absent one.
  if (!b || b.volts == null) { el.classList.add('hidden'); return; }
  el.classList.remove('hidden');
  el.textContent = b.percent == null
    ? `${b.volts.toFixed(2)} V`
    : `${b.percent}% · ${b.volts.toFixed(2)} V`;
  el.classList.toggle('warn', b.percent != null && b.percent <= 30 && b.percent > 15);
  el.classList.toggle('crit', b.percent != null && b.percent <= 15);
}

async function fetchPlan() {
  if (!showPlan) { plan = null; return; }
  try {
    const r = await fetch('/api/plan');
    if (!r.ok) { plan = null; return; }
    plan = new Float32Array(await r.arrayBuffer());
  } catch (e) {
    plan = null;
  }
}

function drawPlan() {
  if (!plan || plan.length < 4 || !mapData) return;
  // Above the obstacles, below the robot and the route: it is the thing being
  // explained by one and explaining the other.
  ctx.save();
  ctx.strokeStyle = 'rgba(120, 210, 255, 0.95)';
  ctx.lineWidth = 2 / view.scale;      // constant on screen at any zoom
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  ctx.beginPath();
  for (let i = 0; i < plan.length; i += 2) {
    const gx = (plan[i] - mapData.ox) / mapData.res;
    const gy = mapData.h - (plan[i + 1] - mapData.oy) / mapData.res;
    if (i === 0) ctx.moveTo(gx, gy); else ctx.lineTo(gx, gy);
  }
  ctx.stroke();
  ctx.restore();
}

function drawObstacles() {
  if (!obstacles || !obstacles.length || !mapData) return;
  // One cell, in grid units. Drawn as squares rather than dots so the cell
  // grid stays legible when zoomed in — this view is read to judge whether
  // an obstacle is real, and a blob hides how many cells are actually set.
  const res = obstacleCell / mapData.res;
  ctx.fillStyle = 'rgba(255, 92, 92, 0.55)';
  for (let i = 0; i < obstacles.length; i += 2) {
    const gx = (obstacles[i] - mapData.ox) / mapData.res;
    const gy = mapData.h - (obstacles[i + 1] - mapData.oy) / mapData.res;
    ctx.fillRect(gx - res / 2, gy - res / 2, res, res);
  }
}

function draw() {
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = '#11141a';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  if (mapData) {
    ctx.imageSmoothingEnabled = false;
    ctx.setTransform(view.scale, 0, 0, view.scale, view.tx, view.ty);
    ctx.drawImage(mapData.img, 0, 0);

    // Under the robot and the route: those are what you are steering, and
    // the obstacles are context for them.
    drawObstacles();
    drawPlan();

    if (pose) {
      // map metres → grid cells → canvas (already inside the view transform)
      const px = (pose.x - mapData.ox) / mapData.res;
      const py = mapData.h - (pose.y - mapData.oy) / mapData.res;
      const r = Math.max(6 / view.scale, 4);
      ctx.translate(px, py);
      ctx.rotate(-pose.yaw);
      ctx.beginPath();
      ctx.moveTo(r * 1.6, 0);
      ctx.lineTo(-r, r * 0.9);
      ctx.lineTo(-r, -r * 0.9);
      ctx.closePath();
      ctx.fillStyle = '#4a9eff';
      ctx.fill();
      ctx.lineWidth = 1.5 / view.scale;
      ctx.strokeStyle = '#06101f';
      ctx.stroke();
      ctx.setTransform(view.scale, 0, 0, view.scale, view.tx, view.ty);
    }

    if (route.waypoints && route.waypoints.length) drawRoute();
    if (nav.goal && !route.active) drawGoal();
    if (drag) drawPlacement();
  }
  requestAnimationFrame(draw);
}

// The waypoint list, numbered, joined in order. The one being driven to is
// highlighted so it is obvious where in the lap the robot is.
function drawRoute() {
  const wp = route.waypoints;
  const pt = (w) => ({ x: (w.x - mapData.ox) / mapData.res,
                       y: mapData.h - (w.y - mapData.oy) / mapData.res });
  const pts = wp.map(pt);
  const r = Math.max(7 / view.scale, 4);

  // Joining line, closed into a ring when looping so the lap reads as one.
  if (pts.length > 1) {
    ctx.beginPath();
    ctx.moveTo(pts[0].x, pts[0].y);
    for (const p of pts.slice(1)) ctx.lineTo(p.x, p.y);
    if (route.loop) ctx.closePath();
    ctx.strokeStyle = route.active ? 'rgba(61,220,132,.75)' : 'rgba(139,149,165,.5)';
    ctx.lineWidth = 1.5 / view.scale;
    ctx.setLineDash(route.active ? [] : [6 / view.scale, 4 / view.scale]);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  pts.forEach((p, i) => {
    const next = route.active && i === route.index;
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.fillStyle = next ? '#3ddc84' : 'rgba(26,31,40,.85)';
    ctx.fill();
    ctx.strokeStyle = next ? '#3ddc84' : (route.active ? '#3ddc84' : '#8b95a5');
    ctx.lineWidth = 2 / view.scale;
    ctx.stroke();
    // heading tick
    ctx.beginPath();
    ctx.moveTo(p.x, p.y);
    ctx.lineTo(p.x + Math.cos(wp[i].yaw) * r * 2,
               p.y - Math.sin(wp[i].yaw) * r * 2);
    ctx.stroke();
    const fs = Math.max(10 / view.scale, 6);
    ctx.fillStyle = next ? '#06101f' : '#e8eaed';
    ctx.font = `${fs}px system-ui`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(String(i + 1), p.x, p.y);
  });
}

// Where Nav2 is heading. Greys out once the goal is over, rather than
// vanishing, so a failed trip leaves something to look at.
function drawGoal() {
  const live = nav.state === 'active' || nav.state === 'pending';
  const gx = (nav.goal.x - mapData.ox) / mapData.res;
  const gy = mapData.h - (nav.goal.y - mapData.oy) / mapData.res;
  const r = Math.max(7 / view.scale, 4);
  ctx.strokeStyle = live ? '#3ddc84' : '#6b7684';
  ctx.lineWidth = 2 / view.scale;
  ctx.beginPath();
  ctx.moveTo(gx - r, gy - r); ctx.lineTo(gx + r, gy + r);
  ctx.moveTo(gx + r, gy - r); ctx.lineTo(gx - r, gy + r);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(gx, gy);
  ctx.lineTo(gx + Math.cos(nav.goal.yaw) * r * 2.2,
             gy - Math.sin(nav.goal.yaw) * r * 2.2);
  ctx.stroke();
}

// Live preview of where the robot is being placed: a ring at the touch
// point and an arrow towards the drag, so the heading that gets sent is
// the one on screen.
function drawPlacement() {
  const r = Math.max(8 / view.scale, 5);
  const col = '#ffb74d';
  ctx.beginPath();
  ctx.arc(drag.a.gx, drag.a.gy, r, 0, Math.PI * 2);
  ctx.lineWidth = 2 / view.scale;
  ctx.strokeStyle = col;
  ctx.stroke();

  const yaw = dragYaw();
  const len = Math.max(r * 3, Math.hypot(drag.p.gx - drag.a.gx,
                                         drag.p.gy - drag.a.gy));
  const ex = drag.a.gx + Math.cos(yaw) * len;
  const ey = drag.a.gy - Math.sin(yaw) * len;
  ctx.beginPath();
  ctx.moveTo(drag.a.gx, drag.a.gy);
  ctx.lineTo(ex, ey);
  ctx.stroke();
  ctx.save();
  ctx.translate(ex, ey);
  ctx.rotate(-yaw);
  ctx.beginPath();
  ctx.moveTo(r, 0);
  ctx.lineTo(-r * 0.6, r * 0.6);
  ctx.lineTo(-r * 0.6, -r * 0.6);
  ctx.closePath();
  ctx.fillStyle = col;
  ctx.fill();
  ctx.restore();
}

function resize() {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.floor(window.innerWidth * dpr);
  canvas.height = Math.floor(window.innerHeight * dpr);
  canvas.style.width = window.innerWidth + 'px';
  canvas.style.height = window.innerHeight + 'px';
  view.fitted = false;
  fitView();
}

// ── status polling ────────────────────────────────────────────────────

async function poll() {
  try {
    const s = await (await fetch('/api/status')).json();
    modes = s.modes || {};
    detail = s.detail || {};
    phase = s.phase || 'idle';
    exploreStatus = s.explore_status;
    pose = s.pose;
    // The server owns the mode — deriving it from which processes happen to
    // be alive is what the old UI did, and it cannot tell "mapping without
    // nav2 yet" from "not mapping".
    mode = s.mode || 'idle';
    mapName = s.map_name;
    localized = !!s.localized;
    maps = s.maps || [];
    nav = s.nav || { state: 'idle', goal: null, distance: null };
    navReady = !!s.nav_ready;
    // Alongside the status poll rather than on its own timer: one cadence to
    // reason about, and an obstacle overlay older than the pose it sits
    // under would be worse than none.
    fetchObstacles();
    fetchPlan();
    // Not while a step is in flight, or the poll would snap the display
    // back to the old value between the tap and the server's reply.
    if (!speedBusy) maxSpeed = s.max_speed;
    if (s.speed_range) speedRange = s.speed_range;
    renderBattery(s.battery);
    if (s.slam_backend) slamBackend = s.slam_backend;
    if (s.drive_limits) driveLimits = s.drive_limits;
    renderSpeed();
    route = s.route || route;
    reportFailures();
    setLive(true);

    if (s.map_seq !== mapSeq) { mapSeq = s.map_seq; if (s.has_map) fetchMap(); }
    renderChips();
    renderControls();
  } catch (e) {
    setLive(false);
  }
}

function setLive(ok) {
  $('dot').className = 'dot ' + (ok ? 'live' : 'dead');
  if (!ok) { $('state').textContent = 'no connection'; return; }
  if (route.active) {
    const n = route.waypoints.length;
    const at = route.index == null ? '' : ` — heading for ${route.index + 1}/${n}`;
    $('state').textContent =
      (route.loop ? 'looping route' : 'driving route')
      + (route.flow ? ' (flowing)' : '') + at;
    return;
  }
  if (route.error) { $('state').textContent = route.error; return; }
  // Any phase the server invents is shown verbatim, so new ones need no
  // client change.
  if (mode === 'mapping' && phase === 'waiting for nav2') {
    // Do not let this read like the mode is blocked — you can drive now.
    $('state').textContent = 'mapping — drive now, navigation starting…';
  } else if (phase && phase !== 'idle' && phase !== 'mapping') {
    $('state').textContent = phase + '…';        // e.g. "waiting for nav2…"
  } else if (mode === 'mapping') {
    if (nav.state === 'active' && nav.distance != null) {
      $('state').textContent = `driving — ${nav.distance.toFixed(1)} m to go`;
    } else if (nav.state === 'pending') {
      $('state').textContent = 'sending goal…';
    } else if (modes.explore) {
      $('state').textContent =
        exploreStatus === 'exploration_complete' ? 'explored — nothing left'
                                                 : 'exploring';
    } else {
      $('state').textContent = 'mapping — drive with the stick';
    }
  } else if (mode === 'localize') {
    if (nav.state === 'active' && nav.distance != null) {
      $('state').textContent = `driving — ${nav.distance.toFixed(1)} m to go`;
    } else if (nav.state === 'pending') {
      $('state').textContent = 'sending goal…';
    } else if (nav.state === 'canceling') {
      $('state').textContent = 'stopping…';
    } else if (nav.state === 'aborted' || nav.state === 'rejected') {
      $('state').textContent = 'could not get there';
    } else if (nav.state === 'canceled') {
      $('state').textContent = 'stopped — you took over';
    } else if (localized && !navReady) {
      $('state').textContent = 'localized — starting navigation…';
    } else {
      $('state').textContent = localized ? `localized on ${mapName}`
                                         : 'position unknown';
    }
  } else {
    $('state').textContent = 'idle';
  }
}

function renderChips() {
  const chips = [
    ['map', modes.map],
    ['amcl', modes.amcl],
    ['nav', modes.nav2],
    ['explore', modes.explore],
  ];
  $('chips').innerHTML = chips
    .map(([n, on]) => `<span class="chip${on ? ' on' : ''}">${n}</span>`)
    .join('');
}

function renderControls() {
  const mapping = mode === 'mapping';
  const loc = mode === 'localize';

  // The pill is the only way in or out of a mode; disable it mid-switch,
  // because a teardown SIGINT waits up to 10 s per layer and a second tap
  // would just queue behind it.
  const switching = /^(switching|stopping|loading)/.test(phase || '');
  for (const [id, m] of [['m-idle', 'idle'], ['m-map', 'mapping'],
                         ['m-loc', 'localize']]) {
    $(id).classList.toggle('on', mode === m);
    $(id).disabled = switching;
  }

  $('explore').classList.toggle('hidden', !mapping);
  // When explore reports itself finished the main button becomes "Search
  // again" and resumes, so without this there is no way to stop exploring
  // short of tearing the mode down — and "finished" is a state it sits in
  // for minutes at a time while the supervisor nudges it.
  $('stopexplore').classList.toggle(
    'hidden', !(mapping && modes.explore
                && exploreStatus === 'exploration_complete'));
  $('save').classList.toggle('hidden', !mapping);
  $('setpose').classList.toggle('hidden', !loc);
  $('setpose').classList.toggle('armed', armed === 'pose');
  // Goals work while mapping too — SLAM supplies the pose, so the
  // only precondition is having one.
  $('goto').classList.toggle('hidden', mode === 'idle');
  $('goto').classList.toggle('armed', armed === 'goal');
  // Nav2 is brought up for you in both modes as soon as a pose exists, so
  // these can wait until it can actually be acted on rather than swallowing
  // a press for a minute.
  $('goto').disabled = !navReady;
  $('goto').textContent = (pose && !navReady) ? 'Go to… (starting)' : 'Go to…';
  $('cancelgoal').classList.toggle(
    'hidden', !(mode !== 'idle'
                && (nav.state === 'active' || nav.state === 'pending')));
  $('changemap').classList.toggle('hidden', !loc);
  // Routes work in both modes. While one is running the button becomes the
  // way to stop it, so stopping is always one tap from the main screen
  // rather than buried in the sheet.
  $('route').classList.toggle('hidden', mode === 'idle');
  $('route').classList.toggle('danger', route.active);
  $('route').textContent = route.active
    ? 'Stop route'
    : (route.waypoints.length ? `Route (${route.waypoints.length})` : 'Route');
  // Manual driving is available in both modes.
  $('stick').classList.toggle('hidden', mode === 'idle');

  $('explore').textContent = !modes.explore ? 'Explore automatically'
    : exploreStatus === 'exploration_complete' ? 'Search again'
    : 'Pause exploring';
  // Only the "start exploring" press needs Nav2. Pausing, resuming a
  // stalled explorer and stopping must stay live whatever Nav2 is doing —
  // greying out the way to stop a moving robot would be a poor trade.
  $('explore').disabled = !modes.explore && !navReady;
  if (!modes.explore && !navReady && mapping) {
    $('explore').textContent = 'Explore automatically (starting)';
  }
  if (mode !== 'idle') $('hint').classList.add('hidden');
}

// A layer that exited non-zero means the launch died — show why, since the
// alternative is a button that silently does nothing.
async function reportFailures() {
  const bad = Object.keys(detail).find((k) => detail[k].failed);
  if (!bad) return;
  const hint = $('hint');
  if (hint.dataset.showing === bad) return;
  hint.dataset.showing = bad;
  hint.classList.remove('hidden');
  let tail = '';
  try { tail = await (await fetch('/api/logs?layer=' + bad)).text(); } catch (e) {}
  hint.innerHTML = `<b>${bad} failed to start</b><pre>` +
    (tail.split('\n').slice(-8).join('\n') || 'no output') + '</pre>';
}

const post = (url, body) => fetch(url, {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify(body || {}),
});

// ── buttons ───────────────────────────────────────────────────────────

// ── mode pill ─────────────────────────────────────────────────────────

function setMode(m, map) {
  const hint = $('hint');
  hint.dataset.showing = '';
  // Otherwise an armed gesture outlives the mode: its button is hidden but
  // every single-finger tap still places instead of panning, with nothing
  // on screen to explain why the map stopped moving.
  armed = null; drag = null;
  if (m === 'mapping') {
    hint.classList.remove('hidden');
    hint.textContent = `Starting ${slamBackend === 'slam_toolbox' ? 'slam_toolbox' : 'cartographer'} — the stick works as soon as the map appears.`;
  }
  return post('/api/mode', map ? { mode: m, map } : { mode: m });
}

$('m-idle').onclick = () => {
  // Leaving mapping throws away an unsaved map; leaving localize costs
  // nothing, so only ask in the first case.
  if (mode === 'mapping' &&
      !confirm('Stop mapping? The map is discarded unless you saved it.')) return;
  setMode('idle');
  mapData = null; mapSeq = -1; view.fitted = false;
};

$('m-map').onclick = () => {
  if (mode === 'mapping') return;
  setMode('mapping');
  mapData = null; mapSeq = -1; view.fitted = false;
};

// Localize needs a map, so the pill opens the picker and the choice is what
// actually switches.
$('m-loc').onclick = () => openMapPicker();
$('changemap').onclick = () => openMapPicker();

$('goto').onclick = () => arm(armed === 'goal' ? null : 'goal');
$('cancelgoal').onclick = () => post('/api/goal/cancel');
$('stopexplore').onclick = () => post('/api/explore', { on: false });

function arm(what) {
  armed = what;
  // Mouse users get no equivalent of "the button is highlighted, now tap the
  // map" unless the cursor says so.
  canvas.style.cursor = what ? 'crosshair' : 'grab';
  const hint = $('hint');
  hint.dataset.showing = '';
  if (armed) {
    hint.classList.remove('hidden');
    hint.textContent = armed === 'goal'
      ? `${TAP} where to go — drag to set the direction to arrive facing.`
      : armed === 'waypoint'
      ? `${TAP} to drop a waypoint — drag to set the heading to arrive on.`
      : `${TAP} where the robot is — drag to point the way it faces.`;
  } else {
    hint.classList.add('hidden');
  }
  renderControls();
}

$('setpose').onclick = () => arm(armed === 'pose' ? null : 'pose');
$('cancel-load').onclick = () => $('mapsheet').classList.add('hidden');

// ── route ─────────────────────────────────────────────────────────────

$('route').onclick = () => {
  if (route.active) { post('/api/route/stop'); return; }
  openRoute();
};
$('cancel-route').onclick = () => $('routesheet').classList.add('hidden');
$('wpundo').onclick = async () => { await post('/api/waypoint/undo'); renderRoute(); };
$('wpclear').onclick = async () => { await post('/api/waypoints/clear'); renderRoute(); };
$('wpadd').onclick = () => {
  $('routesheet').classList.add('hidden');
  arm('waypoint');
};
$('routego').onclick = async () => {
  $('routeerr').textContent = '';
  const r = await (await post('/api/route/start',
                              { loop: $('loop').checked,
                                flow: $('flow').checked })).json();
  if (r.ok) $('routesheet').classList.add('hidden');
  else $('routeerr').textContent = r.error || 'Could not start the route.';
};

function openRoute() {
  $('routeerr').textContent = '';
  $('loop').checked = route.loop;
  $('flow').checked = route.flow;
  renderRoute();
  $('routesheet').classList.remove('hidden');
}

const SPEED_STEP = 0.05;
let speedBusy = false;

function renderSpeed() {
  const v = $('speedval');
  if (!v) return;
  v.textContent = maxSpeed == null ? '—' : maxSpeed.toFixed(2) + ' m/s';
  const [lo, hi] = speedRange;
  $('speeddown').disabled = speedBusy || maxSpeed == null || maxSpeed <= lo + 1e-6;
  $('speedup').disabled   = speedBusy || maxSpeed == null || maxSpeed >= hi - 1e-6;
  $('speednote').textContent = maxSpeed == null
    ? `Available once navigation is running. The joystick is limited to ` +
      `${driveLimits[0].toFixed(2)} m/s until then.`
    : `${lo.toFixed(2)}–${hi.toFixed(2)} m/s, and the joystick matches it. ` +
      'Not saved — returns to the configured speed when navigation restarts.';
}

async function stepSpeed(delta) {
  // maxSpeed is null only before the first status arrives; it no longer waits
  // on Nav2, because the speed can be set with the robot idle.
  if (maxSpeed == null || speedBusy) return;
  const [lo, hi] = speedRange;
  const want = Math.min(hi, Math.max(lo, Math.round((maxSpeed + delta) * 100) / 100));
  if (want === maxSpeed) return;
  speedBusy = true;
  maxSpeed = want;                 // optimistic, so the tap feels immediate
  renderSpeed();
  try {
    const r = await (await post('/api/speed', { value: want })).json();
    if (r.ok) maxSpeed = r.max_speed;
    else $('settingserr').textContent = r.error || 'could not change the speed';
  } catch (e) {
    $('settingserr').textContent = 'could not change the speed';
  } finally {
    speedBusy = false;
    renderSpeed();
  }
}

$('settings').onclick = () => {
  $('settingserr').textContent = '';
  $('showobs').checked = showObstacles;
  $('showplan').checked = showPlan;
  renderSlam();
  renderSpeed();
  $('settingssheet').classList.remove('hidden');
};
$('cancel-settings').onclick = () => $('settingssheet').classList.add('hidden');

function renderSlam() {
  document.querySelectorAll('#slampill .seg').forEach((b) => {
    b.classList.toggle('on', b.dataset.slam === slamBackend);
    // Switching under a live map would tear down what holds the pose graph.
    b.disabled = mode === 'mapping';
  });
  $('slamnote').textContent = mode === 'mapping'
    ? 'Stop mapping to switch backend.'
    : 'Used the next time you start Mapping.';
}

document.querySelectorAll('#slampill .seg').forEach((b) => {
  b.onclick = async () => {
    const r = await (await post('/api/slam', { backend: b.dataset.slam })).json();
    if (r.ok) { slamBackend = r.slam_backend; renderSlam(); }
    else $('settingserr').textContent = r.error || 'could not switch backend';
  };
});

$('showobs').onclick = (e) => {
  showObstacles = e.target.checked;
  try { localStorage.setItem('showObstacles', showObstacles ? '1' : '0'); }
  catch (_) {}                            // private mode: still works, just forgets
  if (!showObstacles) obstacles = null;   // clear immediately, not next poll
};

$('showplan').onclick = (e) => {
  showPlan = e.target.checked;
  try { localStorage.setItem('showPlan', showPlan ? '1' : '0'); } catch (_) {}
  if (!showPlan) plan = null;
};

$('speeddown').onclick = () => stepSpeed(-SPEED_STEP);
$('speedup').onclick   = () => stepSpeed(+SPEED_STEP);

function renderRoute() {
  // Same reason as the arm() hints: this sheet is read on both devices.
  const rp = document.querySelector('#routesheet p');
  if (rp) rp.textContent =
    `${TAP} the map to drop a waypoint, dragging to set the heading to arrive on.`;
  const l = $('wplist');
  const wp = route.waypoints || [];
  l.innerHTML = wp.length
    ? wp.map((w, i) => `<div class="maprow">${i + 1}. ` +
        `<small>${w.x.toFixed(2)}, ${w.y.toFixed(2)} m</small></div>`).join('')
    : '<p>No waypoints yet. <b>Add waypoint</b>, then tap the map.</p>';
  $('routego').disabled = wp.length < 2 || !navReady;
}

function openMapPicker() {
  const list = $('maplist');
  $('loaderr').textContent = '';
  const usable = maps.filter((m) => m.has_grid);
  if (!maps.length) {
    list.innerHTML = '<p>No saved maps yet — build one in Mapping mode and press ' +
                     '<b>Finish &amp; save</b>.</p>';
  } else {
    list.innerHTML = maps.map((m) => {
      const when = new Date(m.mtime * 1000).toLocaleString();
      const mb = (m.size / 1048576).toFixed(1);
      // A map with no grid cannot be localized on. Show it disabled with the
      // reason rather than hiding it — you just saved it and would wonder
      // where it went.
      const why = m.has_grid ? `<small>${when} · ${mb} MB</small>`
                             : '<small>no grid saved — cannot localize on this</small>';
      const sel = (m.name === lastSaved) ? ' primary' : '';
      return `<button class="btn maprow${sel}" data-map="${m.name}"` +
             `${m.has_grid ? '' : ' disabled'}>${m.name}${why}</button>`;
    }).join('');
    for (const b of list.querySelectorAll('[data-map]')) {
      b.onclick = () => {
        $('mapsheet').classList.add('hidden');
        mapData = null; mapSeq = -1; view.fitted = false;
        setMode('localize', b.dataset.map);
      };
    }
  }
  $('mapsheet').classList.remove('hidden');
}

$('explore').onclick = () => {
  if (modes.explore && exploreStatus === 'exploration_complete') {
    post('/api/explore/resume');   // stalled, not finished — kick it
  } else if (modes.explore) {
    post('/api/explore', { on: false });
  } else {
    // First press also brings Nav2 up, which is slow — say so, since the
    // phase does not update for a second or two.
    const hint = $('hint');
    hint.dataset.showing = '';
    hint.classList.remove('hidden');
    hint.textContent = 'Starting nav2 so the robot can plan — up to a minute on the Pi.';
    post('/api/explore', { on: true });
  }
};

$('save').onclick = () => {
  $('saveerr').textContent = '';
  $('sheet').classList.remove('hidden');
  $('mapname').focus();
};
$('cancel-save').onclick = () => $('sheet').classList.add('hidden');

$('confirm-save').onclick = async () => {
  const name = $('mapname').value.trim();
  if (!name) { $('saveerr').textContent = 'Give the map a name.'; return; }
  $('confirm-save').disabled = true;
  $('saveerr').textContent = 'Saving…';
  try {
    const r = await (await post('/api/mapping/save', { name })).json();
    if (r.ok) {
      $('sheet').classList.add('hidden');
      $('saveerr').textContent = '';
      // Offer this one first next time Localize is opened — it is almost
      // always the map you want, and it is the only one guaranteed to exist.
      lastSaved = name;
    } else {
      $('saveerr').textContent = r.detail || 'Save failed.';
    }
  } catch (e) {
    $('saveerr').textContent = 'Save failed: ' + e;
  }
  $('confirm-save').disabled = false;
};

// ── joystick ──────────────────────────────────────────────────────────

const stick = $('stick'), knob = $('knob');
let stickId = null, driveTimer = null, cmd = { linear: 0, angular: 0 };

function stickMove(t) {
  const r = stick.getBoundingClientRect();
  const cx = r.left + r.width / 2, cy = r.top + r.height / 2;
  const max = r.width / 2 - 27;
  let dx = t.clientX - cx, dy = t.clientY - cy;
  const d = Math.hypot(dx, dy);
  if (d > max) { dx *= max / d; dy *= max / d; }
  knob.style.transform = `translate(${dx}px, ${dy}px)`;
  cmd = {
    linear: (-dy / max) * driveLimits[0],
    angular: (-dx / max) * driveLimits[1],
  };
}

function stickEnd() {
  stickId = null;
  knob.style.transform = 'translate(0,0)';
  cmd = { linear: 0, angular: 0 };
  clearInterval(driveTimer); driveTimer = null;
  post('/api/drive', cmd);
}

stick.addEventListener('touchstart', (e) => {
  e.preventDefault();
  const t = e.changedTouches[0];
  stickId = t.identifier;
  stickMove(t);
  // Yield autonomy on manual input. One unconditional call: /cmd_vel has
  // no mux, so a live Nav2 goal would resume driving the instant you lift
  // your thumb, and deciding here would branch on state up to half a
  // second stale — exactly when it matters. The server knows what to stop.
  post('/api/takeover');
  armed = null; drag = null;
  driveTimer = setInterval(() => post('/api/drive', cmd), 100);
}, { passive: false });

stick.addEventListener('touchmove', (e) => {
  e.preventDefault();
  for (const t of e.changedTouches) if (t.identifier === stickId) stickMove(t);
}, { passive: false });

stick.addEventListener('touchend', stickEnd);
stick.addEventListener('touchcancel', stickEnd);

// ── map pan / pinch ───────────────────────────────────────────────────

let pan = null, pinch = null;
// Screen point → grid cell, the exact inverse of the pose overlay in draw().
function toGrid(t) {
  const dpr = canvas.width / window.innerWidth;
  return { gx: (t.clientX * dpr - view.tx) / view.scale,
           gy: (t.clientY * dpr - view.ty) / view.scale };
}

function gridToMetres(g) {
  return { x: g.gx * mapData.res + mapData.ox,
           y: (mapData.h - g.gy) * mapData.res + mapData.oy };
}

canvas.addEventListener('touchstart', (e) => {
  // While armed the map does not pan — that is what separates "place the
  // robot here" from an ordinary drag, with no timing or distance guess
  // that could fire a goal by accident.
  if (armed && e.touches.length === 1 && mapData) {
    const g = toGrid(e.touches[0]);
    drag = { a: g, p: g };
    pan = null;
    return;
  }
  if (e.touches.length === 1) {
    pan = { x: e.touches[0].clientX, y: e.touches[0].clientY };
  } else if (e.touches.length === 2) {
    pan = null;
    pinch = touchDist(e) / view.scale;
  }
}, { passive: true });

canvas.addEventListener('touchmove', (e) => {
  const dpr = canvas.width / window.innerWidth;
  if (drag) {
    // A second finger means they meant to zoom — abandon the placement
    // rather than sending somewhere they only half-chose.
    if (e.touches.length > 1) { drag = null; pinch = touchDist(e) / view.scale; return; }
    drag.p = toGrid(e.touches[0]);
    return;
  }
  if (e.touches.length === 1 && pan) {
    view.tx += (e.touches[0].clientX - pan.x) * dpr;
    view.ty += (e.touches[0].clientY - pan.y) * dpr;
    pan = { x: e.touches[0].clientX, y: e.touches[0].clientY };
  } else if (e.touches.length === 2 && pinch) {
    view.scale = Math.max(0.5, Math.min(40, touchDist(e) / pinch));
  }
}, { passive: true });

canvas.addEventListener('touchend', () => {
  if (drag) { commitDrag(); drag = null; armed = null; renderControls(); }
  pan = null; pinch = null;
});

// ── mouse ─────────────────────────────────────────────────────────────
// The phone is the primary device and its touch handlers are left exactly
// as they were; these reuse the same functions rather than replacing them,
// because stickMove() and toGrid() only ever wanted clientX/clientY and a
// MouseEvent has both.

// Left button only. Right and middle keep their usual meaning, and a
// context menu mid-drag would otherwise strand the robot driving.
const LEFT = 0;

// Drive: press on the stick, drag, release.
stick.addEventListener('mousedown', (e) => {
  if (e.button !== LEFT) return;
  e.preventDefault();
  stickMove(e);
  // Same unconditional yield as touchstart: the server decides what to stop.
  post('/api/takeover');
  armed = null; drag = null;
  mouseDriving = true;
  clearInterval(driveTimer);
  driveTimer = setInterval(() => post('/api/drive', cmd), 100);
});

let mouseDriving = false;

// Move and release are bound to the window, not the stick. Releasing the
// button outside a 132 px circle is easy to do, and on the stick alone that
// would leave the robot driving with no knob under the cursor — the one
// failure here that actually moves the robot.
window.addEventListener('mousemove', (e) => {
  if (mouseDriving) { stickMove(e); return; }
  if (mousePan) {
    const dpr = canvas.width / window.innerWidth;
    view.tx += (e.clientX - mousePan.x) * dpr;
    view.ty += (e.clientY - mousePan.y) * dpr;
    mousePan = { x: e.clientX, y: e.clientY };
  } else if (drag && mouseArmed) {
    drag.p = toGrid(e);
  }
});

function endMouse() {
  if (mouseDriving) { mouseDriving = false; stickEnd(); }
  if (drag && mouseArmed) { commitDrag(); drag = null; armed = null; renderControls(); }
  mouseArmed = false;
  mousePan = null;
  canvas.style.cursor = armed ? 'crosshair' : 'grab';
}
window.addEventListener('mouseup', (e) => { if (e.button === LEFT) endMouse(); });
// Leaving the window with the button down never delivers a mouseup, so the
// robot would keep the last command until the 0.4 s drive timeout. Stop on
// the way out instead.
window.addEventListener('blur', endMouse);
document.addEventListener('mouseleave', endMouse);

// Map: left-drag to pan, or to place a pose/goal/waypoint while armed.
let mousePan = null, mouseArmed = false;

canvas.addEventListener('mousedown', (e) => {
  if (e.button !== LEFT) return;
  e.preventDefault();
  if (armed && mapData) {
    const g = toGrid(e);
    drag = { a: g, p: g };
    mouseArmed = true; mousePan = null;
    return;
  }
  mousePan = { x: e.clientX, y: e.clientY };
  canvas.style.cursor = 'grabbing';
});

// Wheel to zoom, about the cursor rather than the centre: zooming toward a
// corner of a 20 m map is otherwise a pan-and-zoom chore. Same 0.5-40 clamp
// as pinch, so both gestures reach the same limits.
canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const dpr = canvas.width / window.innerWidth;
  const before = view.scale;
  // deltaMode 1 is lines rather than pixels, which trackpads and some mice
  // report; without the scale a single notch would jump the whole range.
  const step = e.deltaMode === 1 ? e.deltaY * 16 : e.deltaY;
  const next = Math.max(0.5, Math.min(40, before * Math.exp(-step * 0.0015)));
  if (next === before) return;
  // Hold the point under the cursor still: solve tx so that the grid
  // coordinate there is unchanged at the new scale.
  const px = e.clientX * dpr, py = e.clientY * dpr;
  view.tx = px - (px - view.tx) * (next / before);
  view.ty = py - (py - view.ty) * (next / before);
  view.scale = next;
}, { passive: false });

// True when the drag was too short to mean a direction.
function dragIsTap() {
  const dx = drag.p.gx - drag.a.gx, dy = drag.p.gy - drag.a.gy;
  return Math.hypot(dx, dy) * view.scale < 10;
}

function dragYaw() {
  const dx = drag.p.gx - drag.a.gx, dy = drag.p.gy - drag.a.gy;
  // Below about 10 screen pixels the direction is noise, so keep the
  // robot's current heading. The preview arrow shows whichever applies,
  // so what you see is what gets sent.
  if (Math.hypot(dx, dy) * view.scale < 10) return pose ? pose.yaw : 0;
  return Math.atan2(-dy, dx);          // screen y grows downward
}

async function commitDrag() {
  if (!mapData) return;
  const m = gridToMetres(drag.a);
  const body = { x: m.x, y: m.y, yaw: dragYaw() };
  // A plain tap on a waypoint means "just go through here" — let the route
  // point it at the next one rather than freezing whatever heading the
  // robot happened to have, which the planner often cannot achieve.
  if (armed === 'waypoint' && dragIsTap()) body.auto = true;
  const kind = armed;
  const url = kind === 'goal' ? '/api/goal'
            : kind === 'waypoint' ? '/api/waypoint'
            : '/api/initialpose';
  const hint = $('hint');
  hint.dataset.showing = '';
  try {
    const r = await (await post(url, body)).json();
    if (!r.ok) {
      hint.classList.remove('hidden');
      hint.textContent = r.error ||
        (kind === 'goal' ? 'Could not send the goal.'
         : kind === 'waypoint' ? 'Could not add the waypoint.'
         : 'Could not set the position.');
    } else {
      hint.classList.add('hidden');
      // Straight back to the sheet so a run of waypoints is quick to place.
      if (kind === 'waypoint') { route.waypoints.push(body); openRoute(); }
    }
  } catch (err) {
    hint.classList.remove('hidden');
    hint.textContent = 'Failed: ' + err;
  }
}

function touchDist(e) {
  return Math.hypot(
    e.touches[0].clientX - e.touches[1].clientX,
    e.touches[0].clientY - e.touches[1].clientY);
}

// ── go ────────────────────────────────────────────────────────────────

window.addEventListener('resize', resize);
resize();
draw();
poll();
setInterval(poll, 500);
