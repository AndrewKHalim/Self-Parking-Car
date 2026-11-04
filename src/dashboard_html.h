#pragma once

// The dashboard, as one self-contained page: no CDN, no framework, no webfont,
// so every byte comes from the ESP32. The websocket reconnects on its own and
// keeps the last 600 samples, which is a minute at the 10 Hz telemetry rate.

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
  /* Map beside the traces on a tablet or laptop, stacked on a phone. */
  .viz { display: grid; grid-template-columns: minmax(260px, 360px) 1fr;
         gap: 12px; align-items: start; margin-bottom: 4px; }
  @media (max-width: 720px) { .viz { grid-template-columns: 1fr; } }
  .legend { display: flex; flex-wrap: wrap; gap: 4px 18px; margin: 8px 0;
            font-size: .8rem; color: #9a9a9a; }
  .legend i { display: inline-block; width: 10px; height: 10px; margin-right: 6px;
              border-radius: 2px; }
  .row { display: flex; flex-wrap: wrap; gap: 8px; align-items: stretch;
         margin-top: 12px; }
  input { flex: 1; min-width: 0; padding: 14px; font-size: 1rem; color: #e8e8e8;
          background: #1b1b1b; border: 1px solid #262626; border-radius: 10px; }
  .lab { flex: 1 1 58px; min-width: 58px; }
  .lab input { width: 100%; padding: 12px 8px; font-size: .95rem; }
  /* Five limit fields plus the button do not share one phone line. */
  #set-target { flex: 1 1 100%; }
  button { padding: 14px 16px; font-size: 1rem; border: 0; border-radius: 10px;
           background: #2d6cdf; color: #fff; }
  button:disabled { background: #2a2a2a; color: #6a6a6a; }
  #stop { background: #b93838; }
</style>
</head>
<body>

<h1>autoparking</h1>
<div id="link">connecting</div>

<div class="grid">
  <div class="cell"><span class="k">t</span><span class="v" id="v-t">-</span></div>
  <div class="cell"><span class="k">state</span><span class="v" id="v-state">-</span></div>
  <div class="cell"><span class="k">gL</span><span class="v" id="v-gl">-</span></div>
  <div class="cell"><span class="k">gR</span><span class="v" id="v-gr">-</span></div>
  <div class="cell"><span class="k">diff</span><span class="v" id="v-diff">-</span></div>
  <div class="cell"><span class="k">yaw</span><span class="v" id="v-yaw">-</span></div>
  <div class="cell"><span class="k">Kyaw</span><span class="v" id="v-kyaw">-</span></div>
  <div class="cell"><span class="k">rear</span><span class="v" id="v-rear">-</span></div>
  <div class="cell"><span class="k">rear target</span><span class="v" id="v-target">-</span></div>
  <div class="cell"><span class="k">align tol</span><span class="v" id="v-align">-</span></div>
  <div class="cell"><span class="k">servo</span><span class="v" id="v-servo">-</span></div>
  <div class="cell"><span class="k">cycles</span><span class="v" id="v-cycles">-</span></div>
  <div class="cell"><span class="k">pull-backs</span><span class="v" id="v-backoffs">-</span></div>
  <div class="cell"><span class="k">duty</span><span class="v" id="v-duty">-</span></div>
  <div class="cell"><span class="k">fault</span><span class="v" id="v-fault">-</span></div>
  <div class="cell"><span class="k">Kp</span><span class="v" id="v-kp">-</span></div>
  <div class="cell"><span class="k">Ki</span><span class="v" id="v-ki">-</span></div>
  <div class="cell"><span class="k">Kd</span><span class="v" id="v-kd">-</span></div>
</div>

<div class="viz">
  <div>
    <div class="legend">
      <span><i style="background:#5c6b7a"></i>bay 400×400 mm</span>
      <span><i style="background:#2d6cdf"></i>car 170×234 mm</span>
      <span><i style="background:#5fd38b"></i>gL</span>
      <span><i style="background:#7aa2f7"></i>gR</span>
      <span><i style="background:#e0af68"></i>rear / target</span>
    </div>
    <canvas id="map"></canvas>
  </div>
  <div>
    <div class="legend">
      <span><i style="background:#5fd38b"></i>gL</span>
      <span><i style="background:#7aa2f7"></i>gR</span>
      <span><i style="background:#e0af68"></i>rear</span>
      <span><i style="background:#e0af68;opacity:.5"></i>target</span>
      <span><i style="background:#bb9af7"></i>servo</span>
    </div>
    <canvas id="chart"></canvas>
  </div>
</div>

<div class="row">
  <label class="lab"><span class="k">target mm</span>
    <input id="in-rear-target" type="number" min="30" max="500" step="5" value="100"></label>
  <label class="lab"><span class="k">tol mm</span>
    <input id="in-align-diff" type="number" min="1" max="200" step="1" value="10"></label>
  <label class="lab"><span class="k">pulls</span>
    <input id="in-backoff" type="number" min="0" max="20" step="1" value="3"></label>
  <label class="lab"><span class="k">duty</span>
    <input id="in-duty" type="number" min="0" max="255" step="1" value="200"></label>
  <label class="lab"><span class="k">steer ms</span>
    <input id="in-steer" type="number" min="0" max="2000" step="50" value="300"></label>
  <label class="lab"><span class="k">yaw deg</span>
    <input id="in-yaw-tol" type="number" min="0.5" max="25" step="0.5" value="5"></label>
  <button id="set-target">apply</button>
</div>
<div class="row">
  <label class="lab"><span class="k">Kp</span>
    <input id="in-kp" type="number" min="0" max="5" step="0.01" value="0.15"></label>
  <label class="lab"><span class="k">Ki</span>
    <input id="in-ki" type="number" min="0" max="1" step="0.001" value="0.01"></label>
  <label class="lab"><span class="k">Kd</span>
    <input id="in-kd" type="number" min="0" max="1" step="0.01" value="0.02"></label>
  <label class="lab"><span class="k">Kyaw</span>
    <input id="in-kyaw" type="number" min="0" max="3" step="0.05" value="0.60"></label>
  <button id="set-gains">set gains</button>
</div>
<div class="row">
  <button id="start" disabled>start</button>
  <button id="stop">stop</button>
</div>

<script>
// The firmware sends integers: t in ms, gaps and rear in mm, servo in degrees.
const OUT_OF_RANGE_MM = 8000;
const MAX_POINTS = 600;
const T_SPAN_MS = 20000;

// Body and sensor stations from config.h (chassis namespace), expressed from
// the geometric centre of the body: +z forward, +x right, both in millimetres.
const CAR_W_MM = 170;            // MAX_WIDTH_MM, outer width across the tires
const CAR_L_MM = 234;            // LENGTH_MM
const HALF_W_MM = CAR_W_MM / 2;
const HALF_L_MM = CAR_L_MM / 2;
const SENSOR_SPAN_HALF_MM = 47.5; // SIDE_SENSOR_SPAN_MM / 2
const LEFT_Z_MM = -12;           // 67 mm ahead of the axle - 79 mm centre offset
const RIGHT_Z_MM = 68;           // 147 mm ahead of the axle - 79 mm centre offset
const REAR_Z_MM = -HALF_L_MM;    // rear bumper face
const STAGGER_MM = LEFT_Z_MM + RIGHT_Z_MM;  // 56, the firmware's stagger coefficient
const NOMINAL_REAR_MM = 250;     // drawn when the rear sensor has no target
const BAY_W_MM = 400;            // fixed drawn bay, one scale for every frame
const BAY_DEPTH_MM = 400;
const BAY_COLOUR = '#5c6b7a';    // the fixed box, so green and blue stay readings
const FONT = '10px -apple-system, "Segoe UI", Roboto, sans-serif';

// One band per quantity, fixed ranges so two runs can be compared by eye. The
// two gaps share a band because their convergence is the thing being watched.
const AXES = [
  { caption: 'gL & gR mm', min: 0, max: 600,
    lines: [{ key: 'gL', colour: '#5fd38b' }, { key: 'gR', colour: '#7aa2f7' }] },
  { caption: 'rear mm', min: 0, max: 400, target: true,
    lines: [{ key: 'rear', colour: '#e0af68' }] },
  { caption: 'servo deg', min: 45, max: 135, centre: 90,
    lines: [{ key: 'servo', colour: '#bb9af7' }] }
];

const BUSY = ['reverse', 'settle_rear', 'steer', 'forward', 'steer_counter',
              'forward_counter', 'settle_fwd'];

let pts = [];
let rearTarget = 100;
let inputsSeeded = false;
let latest = null;

const byId = (id) => document.getElementById(id);
const link = byId('link');
const startButton = byId('start');
const canvas = byId('chart');
const ctx = canvas.getContext('2d');
const mapCanvas = byId('map');
const mapCtx = mapCanvas.getContext('2d');
let retry = 0;

// The websocket stays up on its own: the car is an access point that may reboot
// or drop the link, and the page has to survive both without a reload.
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

const mm = (v, whenMissing) => (v >= OUT_OF_RANGE_MM ? whenMissing : v + ' mm');
const plotted = (v) => (v >= OUT_OF_RANGE_MM ? NaN : v);

function show(d) {
  byId('v-t').textContent = (d.t / 1000).toFixed(1) + ' s';
  byId('v-state').textContent = d.state;
  byId('v-gl').textContent = mm(d.gL, 'none');
  byId('v-gr').textContent = mm(d.gR, 'none');
  byId('v-diff').textContent = mm(Math.abs(d.diff), 'none');
  byId('v-yaw').textContent = d.yaw.toFixed(1) + '\u00b0';
  byId('v-kyaw').textContent = d.kYaw;
  byId('v-rear').textContent = mm(d.rear, 'far');
  byId('v-target').textContent = d.rearTarget + ' mm';
  byId('v-align').textContent = d.alignDiff + ' mm';
  byId('v-servo').textContent = d.servo + '\u00b0';
  byId('v-cycles').textContent = d.cycles;
  byId('v-backoffs').textContent = d.backoffs;
  byId('v-duty').textContent = d.duty;
  byId('v-fault').textContent = d.fault;
  byId('v-kp').textContent = d.kp;
  byId('v-ki').textContent = d.ki;
  byId('v-kd').textContent = d.kd;

  // Seed the inputs once from what the board is running. They are never
  // overwritten again, so a value typed but not yet sent survives the next
  // telemetry frame.
  rearTarget = d.rearTarget;
  if (!inputsSeeded) {
    byId('in-rear-target').value = rearTarget;
    byId('in-align-diff').value = d.alignDiff;
    byId('in-backoff').value = d.backoffMax;
    byId('in-duty').value = d.duty;
    byId('in-steer').value = d.steerMs;
    byId('in-yaw-tol').value = d.yawTol;
    byId('in-kp').value = d.kp;
    byId('in-ki').value = d.ki;
    byId('in-kd').value = d.kd;
    byId('in-kyaw').value = d.kYaw;
    inputsSeeded = true;
  }
  startButton.disabled = BUSY.indexOf(d.state) !== -1;

  pts.push({
    t: d.t,
    gL: plotted(d.gL),
    gR: plotted(d.gR),
    rear: plotted(d.rear),
    servo: d.servo
  });
  if (pts.length > MAX_POINTS) pts.shift();
  latest = d;
  draw();
  drawMap();
}

// Match the backing store to the CSS size and the screen density, so the lines
// stay sharp on a phone without layout maths elsewhere.
function fit(el, g) {
  const ratio = Math.min(2, window.devicePixelRatio || 1);
  const w = el.clientWidth;
  const h = el.clientHeight;
  if (el.width !== Math.round(w * ratio) || el.height !== Math.round(h * ratio)) {
    el.width = Math.round(w * ratio);
    el.height = Math.round(h * ratio);
  }
  g.setTransform(ratio, 0, 0, ratio, 0, 0);
  return [w, h];
}

function draw() {
  const [w, h] = fit(canvas, ctx);
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

    // Time grid first, shared by every band.
    ctx.strokeStyle = '#242424';
    ctx.lineWidth = 1;
    for (let t = Math.ceil(tMin / 5000) * 5000; t <= tMax; t += 5000) {
      ctx.beginPath();
      ctx.moveTo(px(t), y0);
      ctx.lineTo(px(t), y0 + bh);
      ctx.stroke();
    }

    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (const v of [a.max, (a.min + a.max) / 2, a.min]) {
      const y = py(v);
      ctx.beginPath();
      ctx.moveTo(left, y);
      ctx.lineTo(left + pw, y);
      ctx.stroke();
      ctx.fillStyle = '#767676';
      ctx.fillText(String(v), left - 4, y);
    }

    // The rear band carries the stop distance the machine is aiming at, so a
    // change made in the input above is visible against the trace.
    if (a.target) {
      const y = py(Math.min(a.max, Math.max(a.min, rearTarget)));
      ctx.save();
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = '#e0af68';
      ctx.beginPath();
      ctx.moveTo(left, y);
      ctx.lineTo(left + pw, y);
      ctx.stroke();
      ctx.restore();
    }

    // Centred steering is the neutral the controller returns to.
    if (a.centre) {
      const y = py(a.centre);
      ctx.strokeStyle = '#46525e';
      ctx.beginPath();
      ctx.moveTo(left, y);
      ctx.lineTo(left + pw, y);
      ctx.stroke();
    }

    ctx.textAlign = 'right';
    ctx.textBaseline = 'top';
    ctx.fillStyle = '#9a9a9a';
    ctx.fillText(a.caption, left + pw, y0 + 2);

    // The clip keeps an excursion past the fixed range inside its own band
    // instead of drawing across the next one.
    ctx.save();
    ctx.beginPath();
    ctx.rect(left, y0, pw, bh);
    ctx.clip();
    for (const line of a.lines) {
      ctx.strokeStyle = line.colour;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      let started = false;
      for (const p of pts) {
        const v = p[line.key];
        if (p.t < tMin || p.t > tMax || !Number.isFinite(v)) {
          started = false;
          continue;
        }
        const y = py(Math.min(a.max, Math.max(a.min, v)));
        if (!started) { ctx.moveTo(px(p.t), y); started = true; }
        else ctx.lineTo(px(p.t), y);
      }
      ctx.stroke();
    }
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

// The drawn bay is fixed: +X right, +Z forward, origin at the middle of the rear
// wall, a U of BAY_W_MM by BAY_DEPTH_MM at one constant scale. Readings move the
// car inside it, and the dashed rays end at their measured length, so a ray only
// touches a wall when the reading says it should.
function drawMap() {
  const [w, h] = fit(mapCanvas, mapCtx);
  const g = mapCtx;
  g.clearRect(0, 0, w, h);

  const d = latest;
  const psi = ((d ? d.yaw : 0) * Math.PI) / 180;
  const c = Math.cos(psi);
  const s = Math.sin(psi);
  const hasLeft = !!d && d.gL < OUT_OF_RANGE_MM;
  const hasRight = !!d && d.gR < OUT_OF_RANGE_MM;
  const hasRear = !!d && d.rear < OUT_OF_RANGE_MM;
  const gL = hasLeft ? d.gL : 0;
  const gR = hasRight ? d.gR : 0;
  const rearMm = hasRear ? d.rear : NOMINAL_REAR_MM;

  // The rear ray has to land on Z = 0, so the centre sits (rear + L/2) cos away
  // from the wall. cos is floored so a car nearly parallel to the wall cannot
  // collapse the frame.
  const Zc = (rearMm + HALF_L_MM) * Math.max(0.35, c);

  // The gap pair fixes the lateral offset. The stagger term is the firmware's
  // own, so the map and the controller agree about which way is straight.
  let Xc = 0;
  if (hasLeft && hasRight) Xc = ((gL - gR) * c - STAGGER_MM * s) / 2;

  const carPt = (xc, zc) => [Xc + xc * c + zc * s, Zc - xc * s + zc * c];
  const corners = [carPt(-HALF_W_MM, REAR_Z_MM), carPt(HALF_W_MM, REAR_Z_MM),
                   carPt(HALF_W_MM, HALF_L_MM), carPt(-HALF_W_MM, HALF_L_MM)];

  // The box alone sets the scale, so it is identical in every frame. Anything
  // past its edges is clipped by the canvas.
  const xMin = -BAY_W_MM / 2;
  const xMax = BAY_W_MM / 2;
  const zMin = 0;
  const zMax = BAY_DEPTH_MM;
  const pad = 14;
  const scale = Math.min((w - 2 * pad) / (xMax - xMin), (h - 2 * pad) / (zMax - zMin));
  const ox = (w - (xMax - xMin) * scale) / 2;
  const oy = (h - (zMax - zMin) * scale) / 2;
  const mx = (v) => ox + (v - xMin) * scale;
  const my = (v) => oy + (zMax - v) * scale;

  g.lineWidth = 1;
  g.strokeStyle = '#222';
  g.beginPath();
  for (let v = Math.ceil(xMin / 100) * 100; v <= xMax; v += 100) {
    g.moveTo(mx(v), my(zMin));
    g.lineTo(mx(v), my(zMax));
  }
  for (let v = Math.ceil(zMin / 100) * 100; v <= zMax; v += 100) {
    g.moveTo(mx(xMin), my(v));
    g.lineTo(mx(xMax), my(v));
  }
  g.stroke();

  // The line the car is trying to sit on.
  g.save();
  g.setLineDash([5, 5]);
  g.strokeStyle = '#2c2c2c';
  g.beginPath();
  g.moveTo(mx(0), my(zMax));
  g.lineTo(mx(0), my(zMin));
  g.stroke();
  g.restore();

  // The bay itself: one U, open at the front, at a position that never changes.
  g.strokeStyle = BAY_COLOUR;
  g.lineWidth = 2.5;
  g.beginPath();
  g.moveTo(mx(xMin), my(zMax));
  g.lineTo(mx(xMin), my(0));
  g.lineTo(mx(xMax), my(0));
  g.lineTo(mx(xMax), my(zMax));
  g.stroke();
  if (d) {
    const ty = my(d.rearTarget);
    g.save();
    g.setLineDash([4, 4]);
    g.lineWidth = 1.5;
    g.beginPath();
    g.moveTo(mx(xMin), ty);
    g.lineTo(mx(xMax), ty);
    g.stroke();
    g.restore();
    g.fillStyle = '#e0af68';
    g.font = FONT;
    g.textAlign = 'right';
    g.textBaseline = 'bottom';
    g.fillText('target ' + d.rearTarget + ' mm', mx(xMax) - 2, ty - 2);
  }

  const tracePoly = (list) => {
    g.beginPath();
    list.forEach((p, i) => (i ? g.lineTo(mx(p[0]), my(p[1]))
                              : g.moveTo(mx(p[0]), my(p[1]))));
    g.closePath();
  };

  tracePoly(corners);
  g.fillStyle = 'rgba(45, 108, 223, 0.18)';
  g.fill();
  g.strokeStyle = '#2d6cdf';
  g.lineWidth = 2;
  g.stroke();

  // The 95 mm sensor span sits 37.5 mm inboard of each tire, which is the gap the
  // sensors cannot see.
  tracePoly([carPt(-SENSOR_SPAN_HALF_MM, REAR_Z_MM), carPt(SENSOR_SPAN_HALF_MM, REAR_Z_MM),
             carPt(SENSOR_SPAN_HALF_MM, HALF_L_MM), carPt(-SENSOR_SPAN_HALF_MM, HALF_L_MM)]);
  g.save();
  g.setLineDash([3, 3]);
  g.lineWidth = 1;
  g.strokeStyle = 'rgba(232, 232, 232, 0.25)';
  g.stroke();
  g.restore();

  // Nose to tail, so front and back are never in doubt.
  const tailPt = carPt(0, -30);
  const nosePt = carPt(0, 90);
  const tx = mx(tailPt[0]);
  const ty2 = my(tailPt[1]);
  const nx = mx(nosePt[0]);
  const ny = my(nosePt[1]);
  const dl = Math.hypot(nx - tx, ny - ty2) || 1;
  const ux = (nx - tx) / dl;
  const uy = (ny - ty2) / dl;
  g.strokeStyle = '#2d6cdf';
  g.lineWidth = 1.5;
  g.beginPath();
  g.moveTo(tx, ty2);
  g.lineTo(nx, ny);
  g.stroke();
  g.fillStyle = '#2d6cdf';
  g.beginPath();
  g.moveTo(nx, ny);
  g.lineTo(nx - ux * 8 - uy * 4, ny - uy * 8 + ux * 4);
  g.lineTo(nx - ux * 8 + uy * 4, ny - uy * 8 - ux * 4);
  g.closePath();
  g.fill();

  const ray = (mount, dir, len, colour, label, toLeft) => {
    const [fx, fz] = carPt(mount[0], mount[1]);
    const hx = fx + dir[0] * len;
    const hz = fz + dir[1] * len;
    g.save();
    g.setLineDash([4, 4]);
    g.strokeStyle = colour;
    g.lineWidth = 1.5;
    g.beginPath();
    g.moveTo(mx(fx), my(fz));
    g.lineTo(mx(hx), my(hz));
    g.stroke();
    g.restore();
    g.fillStyle = colour;
    for (const p of [[fx, fz], [hx, hz]]) {
      g.beginPath();
      g.arc(mx(p[0]), my(p[1]), 3.5, 0, Math.PI * 2);
      g.fill();
    }
    // The rear ray meets the target line, so its label goes on the far side of
    // the ray instead of on top of that caption.
    g.font = FONT;
    g.textAlign = toLeft ? 'right' : 'left';
    g.textBaseline = 'middle';
    const ax = mx((fx + hx) / 2) + (toLeft ? -6 : 6);
    g.fillText(label, ax, my((fz + hz) / 2) - 6);
  };

  if (hasLeft) ray([-SENSOR_SPAN_HALF_MM, LEFT_Z_MM], [-c, s], gL, '#5fd38b', gL + ' mm');
  if (hasRight) ray([SENSOR_SPAN_HALF_MM, RIGHT_Z_MM], [c, -s], gR, '#7aa2f7', gR + ' mm');
  if (hasRear) ray([0, REAR_Z_MM], [-s, -c], rearMm, '#e0af68', rearMm + ' mm', true);

  g.font = FONT;
  g.textAlign = 'left';
  g.textBaseline = 'top';
  g.fillStyle = '#9a9a9a';
  g.fillText(d ? 'yaw ' + d.yaw.toFixed(1) + '\u00b0  diff ' +
    (d.diff >= OUT_OF_RANGE_MM ? 'none' : d.diff + ' mm') : 'no telemetry', 6, 6);
}

window.addEventListener('resize', () => {
  draw();
  drawMap();
});

// Both limits are read by the control task on its next tick, so a run can be
// retargeted without a reflash. Out-of-range values are clamped on the board.
byId('set-target').addEventListener('click', () => {
  const q = [['rear_target_mm', 'in-rear-target'], ['align_diff_mm', 'in-align-diff'],
    ['backoff_max', 'in-backoff'], ['duty', 'in-duty'], ['steer_ms', 'in-steer'],
    ['yaw_tol_deg', 'in-yaw-tol']]
    .map(([k, id]) => {
      const v = Number(byId(id).value);
      return Number.isFinite(v) ? k + '=' + encodeURIComponent(v) : null;
    }).filter(Boolean);
  if (!q.length) return;
  fetch('/config?' + q.join('&'), { method: 'POST' }).catch(() => {});
});

byId('set-gains').addEventListener('click', () => {
  const q = ['kp', 'ki', 'kd'].map((k) => {
    const v = Number(byId('in-' + k).value);
    return Number.isFinite(v) ? k + '=' + encodeURIComponent(v) : null;
  });
  // The input id is in-kyaw but the board's query key is k_yaw, so it is not
  // part of the loop above.
  const kyaw = Number(byId('in-kyaw').value);
  if (Number.isFinite(kyaw)) q.push('k_yaw=' + encodeURIComponent(kyaw));
  const fields = q.filter(Boolean);
  if (!fields.length) return;
  fetch('/config?' + fields.join('&'), { method: 'POST' }).catch(() => {});
});

startButton.addEventListener('click', () => {
  startButton.disabled = true;
  fetch('/start', { method: 'POST' }).catch(() => {});
});

byId('stop').addEventListener('click', () => {
  fetch('/stop', { method: 'POST' }).catch(() => {});
});

connect();
draw();
drawMap();
</script>
</body>
</html>
)HTML";
