#pragma once

// the dashboard, as one self-contained page. no cdn, no framework, no webfont:
// the car may be serving this from its own access point with no route off it,
// so every byte has to come from the esp32.
//
// the page is fed by a websocket that reconnects on its own, and it keeps the
// last 300 samples itself. nothing is fetched per frame, so the only traffic is
// the ten json messages a second the firmware sends.

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
</div>
<canvas id="chart"></canvas>

<button id="start" disabled>start</button>

<script>
// the firmware sends integers: t in ms, x in mm, psi in milliradians, the gaps
// in mm, the servo in degrees. the page converts for display and keeps the
// car's own units in the history.
const WINDOW = 300;
const MRAD_TO_DEG = 180 / Math.PI / 1000;
const REAR_OUT_OF_RANGE_MM = 8000;

const xs = [];
const psis = [];
const byId = (id) => document.getElementById(id);
const link = byId('link');
const button = byId('start');
const canvas = byId('chart');
const ctx = canvas.getContext('2d');
let retry = 0;

// a websocket that stays up on its own. the car is an access point that may
// reboot or drop the link, and the page has to survive both without a reload.
function connect() {
  const scheme = location.protocol === 'https:' ? 'wss://' : 'ws://';
  const sock = new WebSocket(scheme + location.host + '/ws');
  sock.onopen = () => {
    retry = 0;
    link.textContent = 'live';
    link.className = 'on';
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

// one message is one sample. the button follows what the control task will
// accept: a start needs the calibration and cannot arrive during a run.
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

  xs.push(d.x);
  psis.push(d.psi * MRAD_TO_DEG);
  if (xs.length > WINDOW) { xs.shift(); psis.shift(); }
  draw();
}

// match the backing store to the css size and the screen density, so the lines
// stay sharp on a phone without any layout maths elsewhere.
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

// each series is scaled to its own peak over the window. x moves in millimetres
// and psi in degrees, so a shared scale would flatten one of them; the legend
// is what tells the reader that the two lines share a canvas, not a scale.
function series(values, w, h, colour) {
  if (values.length < 2) return;
  let peak = 1;
  for (const v of values) peak = Math.max(peak, Math.abs(v));
  const pad = 10;
  const span = h / 2 - pad;
  ctx.strokeStyle = colour;
  ctx.lineWidth = 2;
  ctx.beginPath();
  for (let i = 0; i < values.length; i++) {
    const px = w - (values.length - 1 - i) * (w / (WINDOW - 1));
    const py = h / 2 - (values[i] / peak) * span;
    if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
  }
  ctx.stroke();
}

function draw() {
  const [w, h] = fit();
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = '#2b2b2b';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, h / 2);
  ctx.lineTo(w, h / 2);
  ctx.stroke();
  series(xs, w, h, '#4c9aff');
  series(psis, w, h, '#ffb020');
}

window.addEventListener('resize', draw);

// the start request is a plain fetch. the control task decides: it refuses one
// before the calibration and ignores one while a run is in progress.
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
