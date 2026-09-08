'use strict';

// Phone UI for map building. Talks to server.py over plain fetch — status at
// 2 Hz, map only when its sequence number changes, drive at 10 Hz while the
// joystick is held.

const $ = (id) => document.getElementById(id);
const canvas = $('map');
const ctx = canvas.getContext('2d');

const MAX_LINEAR = 0.25;   // must match server.py
const MAX_ANGULAR = 0.8;

let mapData = null;        // {w, h, res, ox, oy, img}
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
let drag = null;           // {a, p} in grid cells while placing
let nav = { state: 'idle', goal: null, distance: null };
let navReady = false;      // nav2 up and able to take a goal
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

function draw() {
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = '#11141a';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  if (mapData) {
    ctx.imageSmoothingEnabled = false;
    ctx.setTransform(view.scale, 0, 0, view.scale, view.tx, view.ty);
    ctx.drawImage(mapData.img, 0, 0);

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

    if (nav.goal) drawGoal();
    if (drag) drawPlacement();
  }
  requestAnimationFrame(draw);
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
    ['map', modes.cartographer],
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
  // Goals work while mapping too — cartographer supplies the pose, so the
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
    hint.textContent = 'Starting cartographer — the stick works as soon as the map appears.';
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
  const hint = $('hint');
  hint.dataset.showing = '';
  if (armed) {
    hint.classList.remove('hidden');
    hint.textContent = armed === 'goal'
      ? 'Tap where to go — drag to set the direction to arrive facing.'
      : 'Tap where the robot is — drag to point the way it faces.';
  } else {
    hint.classList.add('hidden');
  }
  renderControls();
}

$('setpose').onclick = () => arm(armed === 'pose' ? null : 'pose');
$('cancel-load').onclick = () => $('mapsheet').classList.add('hidden');

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
    linear: (-dy / max) * MAX_LINEAR,
    angular: (-dx / max) * MAX_ANGULAR,
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
  const isGoal = armed === 'goal';
  const url = isGoal ? '/api/goal' : '/api/initialpose';
  const hint = $('hint');
  hint.dataset.showing = '';
  try {
    const r = await (await post(url, body)).json();
    if (!r.ok) {
      hint.classList.remove('hidden');
      hint.textContent = r.error ||
        (isGoal ? 'Could not send the goal.'
                : 'Could not set the position.');
    } else {
      hint.classList.add('hidden');
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
