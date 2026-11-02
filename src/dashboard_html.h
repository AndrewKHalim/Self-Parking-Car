#pragma once

// The dashboard, as one self-contained page. No CDN, no framework, no webfont,
// so every byte comes from the ESP32. The websocket reconnects on its own and
// carries ten samples a second. The page keeps the last 600 of them in fixed
// arrays and draws them on a canvas, so a run can be read back as a shape and
// not only as the newest number.
//
// The history lives in the page, not on the board. The socket already sends
// every sample the plots need, so plotting costs the control loop nothing and
// there is no second buffer to keep in step with the loop.

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
  canvas { display: block; width: 100%; height: 300px; background: #1b1b1b;
           border: 1px solid #262626; border-radius: 8px; }
  .legend { display: flex; flex-wrap: wrap; gap: 14px; margin: 8px 0;
            font-size: .8rem; color: #9a9a9a; }
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
  <div class="cell"><span class="k">state</span><span class="v" id="v-state">-</span></div>
  <div class="cell"><span class="k">x</span><span class="v" id="v-x">-</span></div>
  <div class="cell"><span class="k">psi</span><span class="v" id="v-psi">-</span></div>
  <div class="cell"><span class="k">servo</span><span class="v" id="v-servo">-</span></div>
  <div class="cell"><span class="k">gL</span><span class="v" id="v-gl">-</span></div>
  <div class="cell"><span class="k">gR</span><span class="v" id="v-gr">-</span></div>
  <div class="cell"><span class="k">rear</span><span class="v" id="v-rear">-</span></div>
  <div class="cell"><span class="k">ready</span><span class="v" id="v-ready">-</span></div>
  <div class="cell"><span class="k">fault</span><span class="v" id="v-fault">-</span></div>
</div>

<div class="legend">
  <span><i style="background:#5fd38b"></i>x</span>
  <span><i style="background:#7aa2f7"></i>psi</span>
  <span><i style="background:#e0af68"></i>rear</span>
  <span><i style="background:#bb9af7"></i>servo</span>
</div>
<canvas id="chart"></canvas>

<button id="start" disabled>start</button>

<script>
// The firmware sends integers. t in ms, x and the gaps in mm, psi in
// milliradians, the servo in degrees. 8190 is no target and 65535 is a timeout,
// and neither is a distance, so both are stored as a gap and the line breaks
// there instead of climbing to eight or sixty-five metres. 0 to 29 mm is the
// part's minimum range, which means the face is at the wall.
const NO_TARGET = 8190;
const TIMEOUT = 65535;
const WALL_MAX = 29;

// 600 samples is one minute at the 10 Hz socket. The ring is allocated once and
// every sample overwrites the oldest slot, so a long run never grows the heap
// and never copies the series.
const N = 600;
const SPAN_MS = 60000;

const T = new Float32Array(N);
const XV = new Float32Array(N);
const PV = new Float32Array(N);
const RV = new Float32Array(N);
const SV = new Float32Array(N);
let head = 0;
let count = 0;
let lastT = 0;

// One band per quantity, fixed ranges so two runs compare by eye. The rear band
// carries the stop line, because that is the number the brake is decided on.
const AXES = [
  { arr: XV, caption: 'x mm', min: -120, max: 120, colour: '#5fd38b', zero: true, digits: 1 },
  { arr: PV, caption: 'psi deg', min: -20, max: 20, colour: '#7aa2f7', zero: true, digits: 1 },
  { arr: RV, caption: 'rear mm', min: 0, max: 400, colour: '#e0af68', zero: false, digits: 0, stop: 90 },
  { arr: SV, caption: 'servo deg', min: 40, max: 140, colour: '#bb9af7', zero: false, digits: 0 }
];

const byId = (id) => document.getElementById(id);
const link = byId('link');
const button = byId('start');
const canvas = byId('chart');
const ctx = canvas.getContext('2d');
let retry = 0;

function mm(v) {
  if (v === NO_TARGET || v === TIMEOUT) return 'none';
  if (v <= WALL_MAX) return 'wall';
  return v + ' mm';
}

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
    let d;
    try { d = JSON.parse(event.data); } catch (err) { return; }
    show(d);
  };
}

// One message is one sample. ready is the firmware's own gate, so the button is
// only live where a start is accepted. A clock that went backwards is a reboot,
// and the old run's samples are dropped rather than mixed into the new one.
function show(d) {
  const t = Number(d.t);
  if (!Number.isFinite(t)) return;
  if (t < lastT) { head = 0; count = 0; }
  lastT = t;

  T[head] = t;
  XV[head] = Number(d.x);
  PV[head] = Number(d.psi) / 1000 * 57.2958;
  RV[head] = (d.rear === NO_TARGET || d.rear === TIMEOUT) ? NaN : Number(d.rear);
  SV[head] = Number(d.servo);
  head = (head + 1) % N;
  if (count < N) count++;

  byId('v-t').textContent = (t / 1000).toFixed(1) + ' s';
  byId('v-state').textContent = d.state;
  byId('v-x').textContent = Number(d.x).toFixed(1) + ' mm';
  byId('v-psi').textContent = PV[(head + N - 1) % N].toFixed(1) + '\u00b0';
  byId('v-servo').textContent = d.servo + '\u00b0';
  byId('v-gl').textContent = mm(d.gL);
  byId('v-gr').textContent = mm(d.gR);
  byId('v-rear').textContent = mm(d.rear);
  byId('v-ready').textContent = d.ready ? 'yes' : 'no';
  byId('v-fault').textContent = d.fault;
  button.disabled = !d.ready;

  draw();
}

// Match the backing store to the CSS size and the screen density, so the lines
// stay sharp on a phone. The canvas is only reallocated when the CSS size
// actually changed, so a redraw allocates nothing.
function fit() {
  const ratio = Math.min(2, window.devicePixelRatio || 1);
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  const bw = Math.round(w * ratio);
  const bh = Math.round(h * ratio);
  if (canvas.width !== bw || canvas.height !== bh) {
    canvas.width = bw;
    canvas.height = bh;
  }
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return [w, h];
}

// Four bands against time in seconds. The window is the last minute, the grid
// is ten seconds, and the newest value of each band is labelled at its right
// edge so the plot and the cells agree.
function draw() {
  const size = fit();
  const w = size[0];
  const h = size[1];
  ctx.clearRect(0, 0, w, h);

  const left = 46;
  const right = 10;
  const top = 4;
  const bottom = 18;
  const pw = w - left - right;
  const bh = (h - top - bottom) / AXES.length;
  const tMax = Math.max(lastT, SPAN_MS);
  const tMin = tMax - SPAN_MS;
  const px = (t) => left + ((t - tMin) / SPAN_MS) * pw;
  const first = (head - count + N) % N;

  ctx.font = '10px -apple-system, "Segoe UI", Roboto, sans-serif';

  AXES.forEach((a, i) => {
    const y0 = top + i * bh;
    const py = (v) => y0 + ((a.max - v) / (a.max - a.min)) * bh;

    // Time grid first, shared by every band.
    ctx.strokeStyle = '#242424';
    ctx.lineWidth = 1;
    for (let t = Math.ceil(tMin / 10000) * 10000; t <= tMax; t += 10000) {
      ctx.beginPath();
      ctx.moveTo(px(t), y0);
      ctx.lineTo(px(t), y0 + bh);
      ctx.stroke();
    }

    // Value ticks, with the zero line heavier where the axis crosses it.
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (const v of [a.max, (a.min + a.max) / 2, a.min]) {
      const y = py(v);
      ctx.strokeStyle = (a.zero && v === 0) ? '#46525e' : '#242424';
      ctx.beginPath();
      ctx.moveTo(left, y);
      ctx.lineTo(left + pw, y);
      ctx.stroke();
      ctx.fillStyle = '#767676';
      ctx.fillText(String(v), left - 4, y);
    }

    // The rear stop line is the threshold the brake fires on, so it belongs on
    // the plot and not only in the source.
    if (a.stop !== undefined) {
      const y = py(a.stop);
      ctx.strokeStyle = '#8a4b4b';
      ctx.setLineDash([4, 3]);
      ctx.beginPath();
      ctx.moveTo(left, y);
      ctx.lineTo(left + pw, y);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = '#8a4b4b';
      ctx.textAlign = 'left';
      ctx.fillText(a.stop + ' stop', left + 2, y - 6);
    }

    // The clip keeps an excursion past the fixed range inside its own band
    // instead of crossing the next one. A gap breaks the line rather than
    // joining across it.
    ctx.save();
    ctx.beginPath();
    ctx.rect(left, y0, pw, bh);
    ctx.clip();
    ctx.strokeStyle = a.colour;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    let started = false;
    for (let k = 0; k < count; k++) {
      const j = (first + k) % N;
      const t = T[j];
      const v = a.arr[j];
      if (t < tMin || t > tMax || !Number.isFinite(v)) { started = false; continue; }
      const c = Math.min(a.max, Math.max(a.min, v));
      if (!started) { ctx.moveTo(px(t), py(c)); started = true; }
      else ctx.lineTo(px(t), py(c));
    }
    ctx.stroke();
    ctx.restore();

    // The caption and the newest value, so the band names itself and its last
    // number matches the cells above.
    const newest = count ? a.arr[(head + N - 1) % N] : NaN;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillStyle = a.colour;
    ctx.fillText(a.caption, left + 4, y0 + 3);
    ctx.textAlign = 'right';
    ctx.fillStyle = '#c8c8c8';
    ctx.fillText(Number.isFinite(newest) ? newest.toFixed(a.digits) : 'none',
                 left + pw - 2, y0 + 3);

    if (i === AXES.length - 1) {
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      ctx.fillStyle = '#767676';
      for (let t = Math.ceil(tMin / 10000) * 10000; t <= tMax; t += 10000) {
        ctx.fillText((t / 1000).toFixed(0) + ' s', px(t), y0 + bh + 4);
      }
    }
  });
}

window.addEventListener('resize', draw);

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
