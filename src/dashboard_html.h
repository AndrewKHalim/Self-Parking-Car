#pragma once

// The dashboard, as one self-contained page. No CDN, no framework, no webfont, so every byte comes from the ESP32. A websocket reconnects on its own, the page keeps the last 300 samples, and ten JSON messages a second is all the traffic.

#include <Arduino.h>

static const char DASHBOARD_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>autoparking</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin: 0; padding: 12px; background: #111; color: #e8e8e8;
         font: 16px/1.4 -apple-system, "Segoe UI", Roboto, sans-serif; }
  h1 { font-size: 1.05rem; font-weight: 600; letter-spacing: .05em;
       text-transform: uppercase; color: #93a1ad; margin: 0 0 6px; }
  #link { font-size: .8rem; color: #c9803a; margin-bottom: 12px; }
  #link.on { color: #5fa85f; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(94px, 1fr));
          gap: 8px; margin-bottom: 12px; }
  .cell { background: #1b1b1b; border: 1px solid #262626; border-radius: 8px;
          padding: 8px 10px; }
  .k { display: block; font-size: .68rem; text-transform: uppercase;
       letter-spacing: .07em; color: #767676; }
  .v { display: block; font-size: 1.25rem; font-variant-numeric: tabular-nums; }
  canvas { display: block; width: 100%; height: 240px; background: #1b1b1b;
           border: 1px solid #262626; border-radius: 8px; }
  .legend { display: flex; gap: 18px; margin: 8px 0; font-size: .8rem;
            color: #9a9a9a; }
  .legend i { display: inline-block; width: 10px; height: 10px; margin-right: 6px;
              border-radius: 2px; }
  button { width: 100%; margin-top: 12px; padding: 16px; font-size: 1.05rem;
           border: 0; border-radius: 10px; background: #2d6cdf; color: #fff; }
  button:disabled { background: #2a2a2a; color: #6a6a6a; }
</style>
</head>
<body>

<h1>autoparking</h1>
<div id="link">connecting</div>

<div class="grid">
  <div class="cell"><span class="k">t</span><span class="v" id="v-t">-</span></div>
  <div class="cell"><span class="k">x</span><span class="v" id="v-x">-</span></div>
  <div class="cell"><span class="k">psi</span><span class="v" id="v-psi">-</span></div>
  <div class="cell"><span class="k">rear</span><span class="v" id="v-rear">-</span></div>
  <div class="cell"><span class="k">servo</span><span class="v" id="v-servo">-</span></div>
  <div class="cell"><span class="k">phase</span><span class="v" id="v-phase">-</span></div>
  <div class="cell"><span class="k">strokes</span><span class="v" id="v-strokes">-</span></div>
  <div class="cell"><span class="k">calibrated</span><span class="v" id="v-cal">-</span></div>
</div>

<div class="legend">
  <span><i style="background:#4c9aff"></i>x</span>
  <span><i style="background:#ffb020"></i>psi</span>
  <span><i style="background:#5fd38b"></i>servo</span>
</div>
<canvas id="chart"></canvas>

<button id="start" disabled>start</button>

<script>
// The firmware sends integers. t in ms, x and gaps in mm, psi in milliradians, servo in degrees. The page converts for display only.
const MRAD_TO_DEG = 180 / Math.PI / 1000;
const REAR_OUT_OF_RANGE_MM = 8000;
const MAX_POINTS = 1200;
const T_SPAN_MS = 20000;

// Fixed ranges per axis. Peak normalisation made two runs incomparable and hid how large an excursion was, and the servo is on its own band because its degrees are not millimetres.
const AXES = [
  { key: 'x', caption: 'x mm', min: -80, max: 80, colour: '#4c9aff', pose: true },
  { key: 'psi', caption: 'psi deg', min: -30, max: 30, colour: '#ffb020', pose: true },
  { key: 'servo', caption: 'servo deg', min: 45, max: 135, colour: '#5fd38b', pose: false }
];

// One record per sample, in time order. The CSV seeds it with up to 34 s of run, and the socket appends only what is newer.
let pts = [];
let seededT = 0;
const byId = (id) => document.getElementById(id);
const link = byId('link');
const button = byId('start');
const canvas = byId('chart');
const ctx = canvas.getContext('2d');
let retry = 0;

// The websocket stays up on its own. The car is an access point that may reboot or drop the link, and the page has to survive both without a reload.
function connect() {
  const scheme = location.protocol === 'https:' ? 'wss://' : 'ws://';
  const sock = new WebSocket(scheme + location.host + '/ws');
  sock.onopen = () => {
    retry = 0;
    link.textContent = 'live';
    link.className = 'on';
    seed();
  };
  sock.onclose = () => {
    link.textContent = 'reconnecting';
    link.className = '';
    setTimeout(connect, Math.min(5000, 400 * ++retry));
  };
  sock.onerror = () => sock.close();
  sock.onmessage = (event) => {
    let msg;
    try { msg = JSON.parse(event.data); } catch (err) { return; }
    show(msg);
  };
}

// One message is one sample. The button follows what the control task will accept. A start needs the calibration and cannot arrive during a run.
function show(d) {
  byId('v-t').textContent = (d.t / 1000).toFixed(1) + ' s';
  byId('v-x').textContent = d.x + ' mm';
  byId('v-psi').textContent = (d.psi * MRAD_TO_DEG).toFixed(1) + '\u00b0';
  byId('v-rear').textContent = d.rear >= REAR_OUT_OF_RANGE_MM ? 'far' : d.rear + ' mm';
  byId('v-servo').textContent = d.servo + '\u00b0';
  byId('v-phase').textContent = d.phase;
  byId('v-strokes').textContent = d.strokes;
  byId('v-cal').textContent = d.cal ? 'yes' : 'no';
  button.disabled = !d.cal || d.phase === 'back_in' || d.phase === 'correct';

  if (d.t <= seededT) return;
  pts.push({ t: d.t, x: d.x, psi: d.psi * MRAD_TO_DEG, servo: d.servo, pose: !!d.valid });
  if (pts.length > MAX_POINTS) pts.shift();
  draw();
}

// Match the backing store to the CSS size and the screen density, so the lines stay sharp on a phone without layout maths elsewhere.
function fit() {
  const ratio = Math.min(2, window.devicePixelRatio || 1);
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * ratio) || canvas.height !== Math.round(h * ratio)) {
    canvas.width = Math.round(w * ratio);
    canvas.height = Math.round(h * ratio);
  }
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return [w, h];
}

// The board keeps the run, so a reload or a dropped socket repopulates from /log.csv. The snapshot replaces the series, so a reboot cannot mix the old run with the new one, and the socket fills the gap from the next sample on.
function seed() {
  fetch('/log.csv', { cache: 'no-store' })
    .then((r) => (r.ok ? r.text() : ''))
    .then((text) => {
      const lines = text.split('\n');
      const seeded = [];
      for (let i = 1; i < lines.length; i++) {
        const f = lines[i].split(',');
        if (f.length < 14) continue;
        const t = Number(f[0]);
        if (!Number.isFinite(t)) continue;
        seeded.push({
          t,
          x: Number(f[1]),
          psi: Number(f[2]) * MRAD_TO_DEG,
          servo: Number(f[8]),
          pose: (Number(f[12]) & 1) === 1
        });
      }
      if (!seeded.length) {
        pts = [];
        seededT = 0;
        draw();
        return;
      }
      pts = seeded.slice(-MAX_POINTS);
      seededT = seeded[seeded.length - 1].t;
      draw();
    })
    .catch(() => {});
}

// One band per axis, all against time in seconds. Fixed ranges and labelled ticks let two runs and the two channels be compared by eye.
function draw() {
  const [w, h] = fit();
  ctx.clearRect(0, 0, w, h);

  const left = 42;
  const right = 8;
  const top = 4;
  const bottom = 18;
  const pw = w - left - right;
  const bh = (h - top - bottom) / AXES.length;
  const last = pts.length ? pts[pts.length - 1].t : 0;
  const tMax = Math.max(last, T_SPAN_MS);
  const tMin = tMax - T_SPAN_MS;
  const px = (t) => left + ((t - tMin) / (tMax - tMin)) * pw;

  ctx.font = '10px -apple-system, "Segoe UI", Roboto, sans-serif';

  AXES.forEach((a, i) => {
    const y0 = top + i * bh;
    const py = (v) => y0 + ((a.max - v) / (a.max - a.min)) * bh;

    // Time grid first, shared by all three bands.
    ctx.strokeStyle = '#242424';
    ctx.lineWidth = 1;
    for (let t = Math.ceil(tMin / 5000) * 5000; t <= tMax; t += 5000) {
      ctx.beginPath();
      ctx.moveTo(px(t), y0);
      ctx.lineTo(px(t), y0 + bh);
      ctx.stroke();
    }

    // Ticks and their labels. The zero line is drawn heavier where the axis crosses it, because centred is what the controller is aiming at.
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (const v of [a.max, (a.min + a.max) / 2, a.min]) {
      const y = py(v);
      ctx.strokeStyle = v === 0 ? '#46525e' : '#242424';
      ctx.beginPath();
      ctx.moveTo(left, y);
      ctx.lineTo(left + pw, y);
      ctx.stroke();
      ctx.fillStyle = '#767676';
      ctx.fillText(String(v), left - 4, y);
    }

    ctx.textAlign = 'right';
    ctx.textBaseline = 'top';
    ctx.fillStyle = a.colour;
    ctx.fillText(a.caption, left + pw, y0 + 2);

    // The clip keeps an excursion past the fixed range inside its own band instead of crossing the next one.
    ctx.save();
    ctx.beginPath();
    ctx.rect(left, y0, pw, bh);
    ctx.clip();
    ctx.strokeStyle = a.colour;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    let started = false;
    for (const p of pts) {
      if (p.t < tMin || p.t > tMax) continue;
      if (a.pose && !p.pose) { started = false; continue; }
      const v = Math.min(a.max, Math.max(a.min, p[a.key]));
      if (!started) { ctx.moveTo(px(p.t), py(v)); started = true; }
      else ctx.lineTo(px(p.t), py(v));
    }
    ctx.stroke();
    ctx.restore();

    if (i === AXES.length - 1) {
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      ctx.fillStyle = '#767676';
      for (let t = Math.ceil(tMin / 5000) * 5000; t <= tMax; t += 5000) {
        ctx.fillText((t / 1000).toFixed(0) + ' s', px(t), y0 + bh + 4);
      }
    }
  });
}

window.addEventListener('resize', draw);

// The start request is a plain fetch. The control task decides, and it refuses one before the calibration and ignores one while a run is in progress.
button.addEventListener('click', () => {
  button.disabled = true;
  fetch('/start').catch(() => {});
});

connect();
draw();
</script>
</body>
</html>
)HTML";
