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
    if (v === 255) { r = 42; g = 47; b = 55; }            // unknown
    else if (v >= 65) { r = 12; g = 15; b = 20; }         // occupied
    else if (v <= 25) { r = 232; g = 234; b = 237; }      // free
    else { const t = 200 - v; r = g = b = t; }            // uncertain
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
    }
  }
  requestAnimationFrame(draw);
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
    pose = s.pose;
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
  if (phase && phase !== 'idle' && phase !== 'mapping') {
    $('state').textContent = phase + '…';        // e.g. "waiting for nav2…"
  } else {
    $('state').textContent = modes.cartographer
      ? (modes.explore ? 'exploring' : 'mapping — paused')
      : 'idle';
  }
}

function renderChips() {
  const chips = [
    ['map', modes.cartographer],
    ['nav', modes.nav2],
    ['explore', modes.explore],
  ];
  $('chips').innerHTML = chips
    .map(([n, on]) => `<span class="chip${on ? ' on' : ''}">${n}</span>`)
    .join('');
}

function renderControls() {
  const mapping = !!modes.cartographer;
  $('start').classList.toggle('hidden', mapping);
  $('explore').classList.toggle('hidden', !mapping);
  $('save').classList.toggle('hidden', !mapping);
  $('stop').classList.toggle('hidden', !mapping);
  $('stick').classList.toggle('hidden', !mapping);
  $('explore').textContent = modes.explore ? 'Pause exploring' : 'Resume exploring';
  if (mapping) $('hint').classList.add('hidden');
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

$('start').onclick = async () => {
  $('start').disabled = true;
  const hint = $('hint');
  hint.dataset.showing = '';
  hint.classList.remove('hidden');
  hint.textContent = 'Starting cartographer, then nav2 — nav2 takes up to a minute on the Pi.';
  await post('/api/mapping/start', { autonomous: true });
  setTimeout(() => { $('start').disabled = false; }, 4000);
};

$('explore').onclick = () => post('/api/explore', { on: !modes.explore });

$('stop').onclick = async () => {
  if (!confirm('Stop mapping? The map is discarded unless you saved it.')) return;
  await post('/api/mapping/stop');
  mapData = null; mapSeq = -1; view.fitted = false;
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
  // Pausing exploration on manual input avoids fighting the planner.
  if (modes.explore) post('/api/explore', { on: false });
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
canvas.addEventListener('touchstart', (e) => {
  if (e.touches.length === 1) {
    pan = { x: e.touches[0].clientX, y: e.touches[0].clientY };
  } else if (e.touches.length === 2) {
    pan = null;
    pinch = touchDist(e) / view.scale;
  }
}, { passive: true });

canvas.addEventListener('touchmove', (e) => {
  const dpr = canvas.width / window.innerWidth;
  if (e.touches.length === 1 && pan) {
    view.tx += (e.touches[0].clientX - pan.x) * dpr;
    view.ty += (e.touches[0].clientY - pan.y) * dpr;
    pan = { x: e.touches[0].clientX, y: e.touches[0].clientY };
  } else if (e.touches.length === 2 && pinch) {
    view.scale = Math.max(0.5, Math.min(40, touchDist(e) / pinch));
  }
}, { passive: true });

canvas.addEventListener('touchend', () => { pan = null; pinch = null; });

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
