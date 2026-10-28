#include <Arduino.h>

// The dashboard is one flash-resident page: no CDN, no font, no image, so every
// byte comes from the ESP32 and the page still works on the car's own access
// point with no uplink. The charts are hand-drawn on canvas because a chart
// library would weigh more than the whole page.

extern const char DASHBOARD_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>autoparking</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin: 0; padding: 12px; background: #111; color: #e8e8e8;
         font: 15px/1.45 -apple-system, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; }
  .wrap { max-width: 1080px; margin: 0 auto; }
  header { display: flex; align-items: center; flex-wrap: wrap; gap: 10px; }
  h1 { font-size: 1rem; font-weight: 600; letter-spacing: .06em; text-transform: uppercase;
       color: #93a1ad; margin: 0; }
  .pill { font-size: .65rem; text-transform: uppercase; letter-spacing: .09em;
          padding: 2px 8px; border-radius: 99px; background: #262626; color: #9a9a9a;
          border: 1px solid #333; }
  .pill.live { background: #12291b; color: #61bf80; border-color: #1e4a2c; }
  .pill.polling { background: #2a2314; color: #c9a03a; border-color: #4a3d1d; }
  .pill.down { background: #2a1515; color: #e06c6c; border-color: #4a1d1d; }
  #meta { font-size: .76rem; color: #8a8a8a; margin: 3px 0 12px;
          font-variant-numeric: tabular-nums; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(94px, 1fr));
          gap: 6px; }
  .cell { background: #1b1b1b; border: 1px solid #262626; border-radius: 8px;
          padding: 6px 9px 7px; min-width: 0; }
  .cell.wide { grid-column: span 2; }
  .k { display: block; font-size: .63rem; text-transform: uppercase;
       letter-spacing: .08em; color: #747474; }
  .v { display: block; font-size: 1rem; font-variant-numeric: tabular-nums;
       white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  .s { display: block; font-size: .62rem; letter-spacing: .06em; color: #6b6b6b;
       text-transform: uppercase; }
  .na { color: #555; }
  .warn { color: #e06c6c; }
  .bits { display: flex; gap: 3px; margin-top: 3px; }
  .bit { font-size: .58rem; font-style: normal; padding: 0 3px; border-radius: 3px;
         border: 1px solid #333; color: #555; }
  .bit.on { background: #3ddc97; border-color: #3ddc97; color: #0b0b0b; }
  .card { background: #1b1b1b; border: 1px solid #262626; border-radius: 8px;
          padding: 6px 7px 7px; margin-top: 8px; }
  .chd { display: flex; flex-wrap: wrap; align-items: baseline; gap: 2px 12px;
         font-size: .65rem; color: #8f8f8f; text-transform: uppercase;
         letter-spacing: .06em; margin: 0 2px 5px; }
  .chd b { color: #ccd4dc; font-weight: 600; font-size: .7rem; }
  .chd .u { color: #5d5d5d; }
  .lg { display: flex; flex-wrap: wrap; gap: 2px 10px; text-transform: none;
        letter-spacing: 0; }
  .lg span { white-space: nowrap; }
  .sw { display: inline-block; width: 11px; height: 3px; border-radius: 2px;
        vertical-align: middle; margin-right: 4px; }
  .sw.thin { height: 1px; }
  canvas { display: block; width: 100%; background: #161616; border-radius: 5px; }
  .h-dist { height: 200px; }
  .h-mid { height: 180px; }
  .h-drive { height: 150px; }
  .h-tl { height: 46px; }
  .actions { display: flex; gap: 8px; margin: 12px 0 4px; }
  .btn { flex: 1; text-align: center; text-decoration: none; font: inherit;
         font-weight: 600; font-size: .95rem; padding: 13px; border: 0;
         border-radius: 10px; background: #2d6cdf; color: #fff; cursor: pointer; }
  .btn.ghost { background: #232323; color: #cfcfcf; border: 1px solid #333; }
  @media (max-width: 420px) {
    .h-dist { height: 180px; }
    .h-mid { height: 165px; }
    .h-drive { height: 140px; }
  }
</style>
</head>
<body>
<div class="wrap">

<header>
  <h1>autoparking</h1>
  <span class="pill" id="status">starting</span>
</header>
<div id="meta">waiting for /api/info</div>

<div class="grid">
  <div class="cell wide"><span class="k">state</span><span class="v" id="v-state">&mdash;</span></div>
  <div class="cell"><span class="k">t</span><span class="v" id="v-t">&mdash;</span></div>
  <div class="cell wide"><span class="k">ip</span><span class="v" id="v-ip">&mdash;</span><span class="s" id="v-wifi">&mdash;</span></div>
  <div class="cell"><span class="k">uptime</span><span class="v" id="v-up">&mdash;</span></div>
  <div class="cell"><span class="k">samples</span><span class="v" id="v-samples">&mdash;</span></div>

  <div class="cell"><span class="k">raw L</span><span class="v" id="v-rawL">&mdash;</span></div>
  <div class="cell"><span class="k">raw R</span><span class="v" id="v-rawR">&mdash;</span></div>
  <div class="cell"><span class="k">raw B</span><span class="v" id="v-rawB">&mdash;</span></div>

  <div class="cell"><span class="k">filt L</span><span class="v" id="v-fL">&mdash;</span></div>
  <div class="cell"><span class="k">filt R</span><span class="v" id="v-fR">&mdash;</span></div>
  <div class="cell"><span class="k">filt B</span><span class="v" id="v-fB">&mdash;</span></div>

  <div class="cell"><span class="k">lat L</span><span class="v" id="v-latL">&mdash;</span></div>
  <div class="cell"><span class="k">lat R</span><span class="v" id="v-latR">&mdash;</span></div>
  <div class="cell"><span class="k">err</span><span class="v" id="v-err">&mdash;</span></div>

  <div class="cell"><span class="k">p</span><span class="v" id="v-p">&mdash;</span></div>
  <div class="cell"><span class="k">i</span><span class="v" id="v-i">&mdash;</span></div>
  <div class="cell"><span class="k">d</span><span class="v" id="v-d">&mdash;</span></div>
  <div class="cell"><span class="k">out</span><span class="v" id="v-out">&mdash;</span></div>

  <div class="cell"><span class="k">servo</span><span class="v" id="v-sv">&mdash;</span></div>
  <div class="cell"><span class="k">wl</span><span class="v" id="v-wl">&mdash;</span></div>
  <div class="cell"><span class="k">wr</span><span class="v" id="v-wr">&mdash;</span></div>

  <div class="cell"><span class="k">dir</span><span class="v" id="v-dir">&mdash;</span></div>
  <div class="cell"><span class="k">pwm</span><span class="v" id="v-pwm">&mdash;</span></div>
  <div class="cell"><span class="k">valid</span><span class="v" id="v-valid">&mdash;</span>
    <span class="bits"><i class="bit" id="v-bitL">L</i><i class="bit" id="v-bitR">R</i><i class="bit" id="v-bitB">B</i></span>
  </div>
  <div class="cell"><span class="k">prox</span><span class="v" id="v-prox">&mdash;</span></div>
  <div class="cell"><span class="k">both</span><span class="v" id="v-both">&mdash;</span></div>
  <div class="cell"><span class="k">pull fwd</span><span class="v" id="v-pf">&mdash;</span></div>
</div>

<div class="card">
  <div class="chd"><b>distances</b><span class="u">mm</span>
    <span class="lg">
      <span><i class="sw" style="background:#4c9aff"></i>fL</span>
      <span><i class="sw" style="background:#ffb020"></i>fR</span>
      <span><i class="sw" style="background:#3ddc97"></i>fB</span>
      <span><i class="sw thin" style="background:#5f7fa0"></i>raw</span>
    </span>
  </div>
  <canvas id="ch-dist" class="h-dist"></canvas>
</div>

<div class="card">
  <div class="chd"><b>lateral error / servo</b><span class="u">mm / deg</span>
    <span class="lg">
      <span><i class="sw" style="background:#ffb020"></i>err</span>
      <span><i class="sw" style="background:#4c9aff"></i>servo</span>
    </span>
  </div>
  <canvas id="ch-err" class="h-mid"></canvas>
</div>

<div class="card">
  <div class="chd"><b>pid</b><span class="u">deg</span>
    <span class="lg">
      <span><i class="sw" style="background:#4c9aff"></i>p</span>
      <span><i class="sw" style="background:#3ddc97"></i>i</span>
      <span><i class="sw" style="background:#f472b6"></i>d</span>
      <span><i class="sw" style="background:#ffb020"></i>out</span>
      <span><i class="sw" style="background:#9aa7b4"></i>servo</span>
      <span class="u">shaded = loop off</span>
    </span>
  </div>
  <canvas id="ch-pid" class="h-mid"></canvas>
</div>

<div class="card">
  <div class="chd"><b>ackermann wheels</b><span class="u">deg</span>
    <span class="lg">
      <span><i class="sw" style="background:#4c9aff"></i>wl</span>
      <span><i class="sw" style="background:#f472b6"></i>wr</span>
    </span>
  </div>
  <canvas id="ch-ack" class="h-mid"></canvas>
</div>

<div class="card">
  <div class="chd"><b>drive</b><span class="u">pwm</span>
    <span class="lg">
      <span><i class="sw" style="background:#4c9aff"></i>pwm</span>
      <span><i class="sw" style="background:#c9803a"></i>REV</span>
      <span><i class="sw" style="background:#3ddc97"></i>FWD</span>
      <span><i class="sw" style="background:#3a3a3a"></i>STOP</span>
    </span>
  </div>
  <canvas id="ch-drive" class="h-drive"></canvas>
</div>

<div class="card">
  <div class="chd"><b>state</b>
    <span class="lg">
      <span><i class="sw" style="background:#6b7280"></i>boot</span>
      <span><i class="sw" style="background:#3b82f6"></i>idle</span>
      <span><i class="sw" style="background:#f59e0b"></i>reverse</span>
      <span><i class="sw" style="background:#a855f7"></i>pull</span>
      <span><i class="sw" style="background:#22c55e"></i>parked</span>
      <span><i class="sw" style="background:#ef4444"></i>e-stop</span>
    </span>
  </div>
  <canvas id="ch-time" class="h-tl"></canvas>
</div>

<div class="actions">
  <a class="btn" href="/log.csv" download>Download CSV</a>
  <button class="btn ghost" id="reset" type="button">Clear log</button>
</div>

</div>
<script>
// Everything here is in the firmware's own millisecond clock, so x needs no clock sync with the browser.
const CAP = 2000;        // the ring holds 2048; the page keeps almost all of it
const SPAN = 60000;      // rolling window; loop() has delay(25), so a minute is ~2000 samples
const FAR = 1500;        // SENSOR_MAX_VALID_CM: at or above it there was no return, so it is a gap, not a distance

const DASH = '\u2014';
const byId = (id) => document.getElementById(id);
const pill = byId('status');

const C = {
  fL: '#4c9aff', fR: '#ffb020', fB: '#3ddc97',
  rL: '#5f7fa0', rR: '#8f7a54', rB: '#5f9c82',
  err: '#ffb020', sv: '#4c9aff',
  p: '#4c9aff', i: '#3ddc97', d: '#f472b6', out: '#ffb020', servo: '#9aa7b4',
  wl: '#4c9aff', wr: '#f472b6',
  pwm: '#4c9aff',
  REV: '#c9803a', FWD: '#3ddc97', STOP: '#3a3a3a'
};

const STATE_C = {
  BOOT: '#6b7280', IDLE_READY: '#3b82f6', REVERSE_PARK: '#f59e0b',
  PULL_FORWARD: '#a855f7', PARKED_DONE: '#22c55e', EMERGENCY_STOP: '#ef4444'
};
const STATE_S = {
  BOOT: 'BOOT', IDLE_READY: 'IDLE', REVERSE_PARK: 'REVERSE',
  PULL_FORWARD: 'PULL', PARKED_DONE: 'PARKED', EMERGENCY_STOP: 'E-STOP'
};

const S = { samples: [], lastSeq: 0, seen: false, total: 0, first: 0, caps: 0,
            info: null, gen: 0, fails: 0, infoAt: 0 };
let dirty = true;
let timer = 0;
let infoBusy = false;

// The loop jitters, so x comes from t and not from the sample index. Index spacing would stretch slow stretches and squeeze fast ones.
function win() {
  const a = S.samples;
  if (!a.length) return { t0: 0, t1: SPAN };
  const t1 = a[a.length - 1].t;
  const t0 = Math.max(t1 - SPAN, a[0].t);
  return { t0: t0 >= t1 ? t1 - 1 : t0, t1 };
}

function X(f, s, t) {
  const k = (t - s.t0) / (s.t1 - s.t0);
  return f.L + (k < 0 ? 0 : k > 1 ? 1 : k) * f.pw;
}

// One backing store per canvas, DPR capped at 2: a 3x phone would otherwise pay 9x the pixel work for no visible gain.
function frame(cv, left, right) {
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const w = cv.clientWidth;
  const h = cv.clientHeight;
  if (!w || !h) return null;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
    cv.width = Math.round(w * dpr);
    cv.height = Math.round(h * dpr);
  }
  const g = cv.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  const T = 7;
  const B = 14;
  return { g: g, w: w, h: h, L: left, R: right, T: T, B: B,
           pw: w - left - right, ph: h - T - B };
}

function gridY(f, ticks) {
  const g = f.g;
  g.font = '10px ui-monospace, SFMono-Regular, Menlo, monospace';
  g.textAlign = 'right';
  g.textBaseline = 'middle';
  for (const t of ticks) {
    const y = Math.round(f.T + (1 - t.p) * f.ph) + .5;
    g.strokeStyle = t.zero ? '#454545' : '#242424';
    g.lineWidth = 1;
    g.setLineDash(t.zero ? [4, 4] : []);
    g.beginPath();
    g.moveTo(f.L, y);
    g.lineTo(f.L + f.pw, y);
    g.stroke();
    g.setLineDash([]);
    g.fillStyle = t.zero ? '#8a8a8a' : '#6b6b6b';
    g.fillText(t.v, f.L - 4, y);
  }
}

function rightTicks(f, ticks) {
  const g = f.g;
  g.font = '10px ui-monospace, SFMono-Regular, Menlo, monospace';
  g.textAlign = 'left';
  g.textBaseline = 'middle';
  for (const t of ticks) {
    g.fillStyle = t.zero ? '#8a8a8a' : '#6b6b6b';
    g.fillText(t.v, f.L + f.pw + 5, f.T + (1 - t.p) * f.ph);
  }
}

function gridX(f, s) {
  const g = f.g;
  g.font = '10px ui-monospace, SFMono-Regular, Menlo, monospace';
  g.textAlign = 'center';
  g.textBaseline = 'bottom';
  const step = 15000;
  for (let t = Math.ceil(s.t0 / step) * step; t < s.t1 - 700; t += step) {
    const x = Math.round(X(f, s, t)) + .5;
    g.strokeStyle = '#1c1c1c';
    g.beginPath();
    g.moveTo(x, f.T);
    g.lineTo(x, f.T + f.ph);
    g.stroke();
    g.fillStyle = '#5a5a5a';
    g.fillText('-' + Math.round((s.t1 - t) / 1000) + 's', x, f.h - 2);
  }
  g.textAlign = 'right';
  g.fillStyle = '#5a5a5a';
  g.fillText('now', f.L + f.pw, f.h - 2);
}

function note(f, txt) {
  if (!txt) return;
  const g = f.g;
  g.fillStyle = '#4e4e4e';
  g.font = '11px -apple-system, "Segoe UI", Roboto, sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText(txt, f.L + f.pw / 2, f.T + f.ph / 2);
}

function clipPlot(f) {
  f.g.save();
  f.g.beginPath();
  f.g.rect(f.L, f.T, f.pw, f.ph);
  f.g.clip();
}

function series(f, s, get, y, colour, width) {
  const g = f.g;
  g.strokeStyle = colour;
  g.lineWidth = width || 1.5;
  g.lineJoin = 'round';
  g.beginPath();
  let pen = false;
  for (const d of S.samples) {
    if (d.t < s.t0) continue;
    const v = get(d);
    if (v === null || v === undefined || !isFinite(v)) { pen = false; continue; }
    const x = X(f, s, d.t);
    const py = y(v);
    if (pen) g.lineTo(x, py);
    else { g.moveTo(x, py); pen = true; }
  }
  g.stroke();
}

// PWM is a duty, not a position, so the trace steps between values instead of sliding between them.
function stepSeries(f, s, get, y, colour) {
  const g = f.g;
  g.strokeStyle = colour;
  g.lineWidth = 2;
  g.beginPath();
  let px = null;
  let py = 0;
  for (const d of S.samples) {
    if (d.t < s.t0) continue;
    const x = X(f, s, d.t);
    const yy = y(get(d));
    if (px === null) g.moveTo(x, yy);
    else { g.lineTo(x, py); g.lineTo(x, yy); }
    px = x;
    py = yy;
  }
  g.stroke();
}

function runs(get) {
  const out = [];
  let cur = null;
  for (const d of S.samples) {
    const v = get(d);
    if (cur && cur.v === v) cur.t1 = d.t;
    else { cur = { v: v, t0: d.t, t1: d.t }; out.push(cur); }
  }
  return out;
}

function maxAbs(get) {
  let m = 0;
  for (const d of S.samples) m = Math.max(m, Math.abs(get(d)));
  return m;
}

function niceMax(v, floor) {
  v = Math.max(v, floor || 1);
  const p = Math.pow(10, Math.floor(Math.log10(v)));
  const n = v / p;
  return (n <= 1 ? 1 : n <= 2 ? 2 : n <= 2.5 ? 2.5 : n <= 5 ? 5 : 10) * p;
}

function drawDist() {
  const f = frame(byId('ch-dist'), 40, 8);
  if (!f) return;
  const s = win();
  gridY(f, [{ v: '1500', p: 1 }, { v: '1000', p: 2 / 3 },
            { v: '500', p: 1 / 3 }, { v: '0', p: 0 }]);
  gridX(f, s);
  if (!S.samples.length) return note(f, 'waiting for samples');
  const y = (v) => f.T + (1 - v / FAR) * f.ph;
  clipPlot(f);
  series(f, s, (d) => d.rawL < FAR ? d.rawL : null, y, C.rL, 1);
  series(f, s, (d) => d.rawR < FAR ? d.rawR : null, y, C.rR, 1);
  series(f, s, (d) => d.rawB < FAR ? d.rawB : null, y, C.rB, 1);
  series(f, s, (d) => d.fL < FAR ? d.fL : null, y, C.fL, 2);
  series(f, s, (d) => d.fR < FAR ? d.fR : null, y, C.fR, 2);
  series(f, s, (d) => d.fB < FAR ? d.fB : null, y, C.fB, 2);
  f.g.restore();
}

function drawErr() {
  const f = frame(byId('ch-err'), 40, 40);
  if (!f) return;
  const s = win();
  const lim = niceMax(maxAbs((d) => d.err), 20);
  const mid = f.T + f.ph / 2;
  gridY(f, [{ v: '+' + lim, p: 1 }, { v: '0', p: .5, zero: 1 }, { v: '-' + lim, p: 0 }]);
  rightTicks(f, [{ v: '45\u00b0', p: 0 }, { v: '90\u00b0', p: .5, zero: 1 }, { v: '135\u00b0', p: 1 }]);
  gridX(f, s);
  if (!S.samples.length) return note(f, 'waiting for samples');
  const yErr = (v) => mid - (v / lim) * (f.ph / 2);
  const ySv = (v) => f.T + ((135 - v) / 90) * f.ph;
  clipPlot(f);
  series(f, s, (d) => d.err, yErr, C.err, 2);
  series(f, s, (d) => d.sv, ySv, C.sv, 1.4);
  f.g.restore();
}

function drawPid() {
  const f = frame(byId('ch-pid'), 40, 40);
  if (!f) return;
  const s = win();
  let m = 0;
  for (const d of S.samples) m = Math.max(m, Math.abs(d.p), Math.abs(d.i), Math.abs(d.d), Math.abs(d.out));
  const lim = niceMax(m / 10, 3);
  const mid = f.T + f.ph / 2;
  gridY(f, [{ v: '+' + lim + '\u00b0', p: 1 }, { v: '0', p: .5, zero: 1 }, { v: '-' + lim + '\u00b0', p: 0 }]);
  rightTicks(f, [{ v: '45\u00b0', p: 0 }, { v: '90\u00b0', p: .5, zero: 1 }, { v: '135\u00b0', p: 1 }]);
  gridX(f, s);
  if (!S.samples.length) return note(f, 'waiting for samples');
  const yT = (v) => mid - (v / lim) * (f.ph / 2);
  const ySv = (v) => f.T + ((135 - v) / 90) * f.ph;
  clipPlot(f);
  series(f, s, (d) => d.p / 10, yT, C.p, 1.6);
  series(f, s, (d) => d.i / 10, yT, C.i, 1.6);
  series(f, s, (d) => d.d / 10, yT, C.d, 1.6);
  series(f, s, (d) => d.out / 10, yT, C.out, 2);
  series(f, s, (d) => d.sv, ySv, C.servo, 1.2);
  f.g.restore();
  // Dim the stretches where the loop was off: the terms are zero there, and a zero trace would read as a measurement.
  const rr = runs((d) => d.pa);
  for (let i = 0; i < rr.length; i++) {
    if (rr[i].v) continue;
    const a = Math.max(rr[i].t0, s.t0);
    const b = Math.min(i + 1 < rr.length ? rr[i + 1].t0 : s.t1, s.t1);
    if (b <= a) continue;
    const x0 = X(f, s, a);
    const x1 = X(f, s, b);
    f.g.fillStyle = 'rgba(17,17,17,.62)';
    f.g.fillRect(x0, f.T, x1 - x0, f.ph);
    if (x1 - x0 > 60) {
      f.g.fillStyle = '#5f5f5f';
      f.g.font = '10px -apple-system, "Segoe UI", Roboto, sans-serif';
      f.g.textAlign = 'center';
      f.g.textBaseline = 'middle';
      f.g.fillText('loop off', (x0 + x1) / 2, f.T + 10);
    }
  }
}

function drawAck() {
  const f = frame(byId('ch-ack'), 40, 8);
  if (!f) return;
  const s = win();
  let m = 0;
  for (const d of S.samples) m = Math.max(m, Math.abs(d.wl), Math.abs(d.wr));
  const lim = niceMax(m / 10, 5);
  const mid = f.T + f.ph / 2;
  gridY(f, [{ v: '+' + lim + '\u00b0', p: 1 }, { v: '0', p: .5, zero: 1 }, { v: '-' + lim + '\u00b0', p: 0 }]);
  gridX(f, s);
  if (!S.samples.length) return note(f, 'waiting for samples');
  const y = (v) => mid - (v / lim) * (f.ph / 2);
  clipPlot(f);
  series(f, s, (d) => d.wl / 10, y, C.wl, 2);
  series(f, s, (d) => d.wr / 10, y, C.wr, 2);
  f.g.restore();
}

function drawDrive() {
  const f = frame(byId('ch-drive'), 40, 8);
  if (!f) return;
  const s = win();
  const band = 8;
  const ph = f.ph - band;
  const pos = (v) => (v / 255) * (ph / f.ph);
  gridY(f, [{ v: '255', p: pos(255) }, { v: '170', p: pos(170) },
            { v: '85', p: pos(85) }, { v: '0', p: 0 }]);
  gridX(f, s);
  if (!S.samples.length) return note(f, 'waiting for samples');
  clipPlot(f);
  const rr = runs((d) => d.dir);
  for (let i = 0; i < rr.length; i++) {
    const a = Math.max(rr[i].t0, s.t0);
    const b = Math.min(i + 1 < rr.length ? rr[i + 1].t0 : s.t1, s.t1);
    if (b <= a) continue;
    f.g.fillStyle = C[rr[i].v] || C.STOP;
    f.g.fillRect(X(f, s, a), f.T + f.ph - band, Math.max(1, X(f, s, b) - X(f, s, a)), band);
  }
  stepSeries(f, s, (d) => d.pwm, (v) => f.T + (1 - v / 255) * ph, C.pwm);
  f.g.restore();
}

function drawTime() {
  const f = frame(byId('ch-time'), 0, 0);
  if (!f) return;
  f.T = 0;
  f.B = 0;
  f.ph = f.h;
  const s = win();
  const g = f.g;
  g.fillStyle = '#161616';
  g.fillRect(0, 0, f.w, f.h);
  if (!S.samples.length) return note(f, '');
  const rr = runs((d) => d.st);
  g.font = '600 9px -apple-system, "Segoe UI", Roboto, sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  for (let i = 0; i < rr.length; i++) {
    const a = Math.max(rr[i].t0, s.t0);
    const b = Math.min(i + 1 < rr.length ? rr[i + 1].t0 : s.t1, s.t1);
    if (b <= a) continue;
    const x0 = X(f, s, a);
    const x1 = X(f, s, b);
    g.fillStyle = STATE_C[rr[i].v] || '#555';
    g.fillRect(x0, 0, x1 - x0, f.h);
    if (x1 - x0 > 42) {
      g.fillStyle = 'rgba(0,0,0,.72)';
      g.fillText(STATE_S[rr[i].v] || rr[i].v, (x0 + x1) / 2, f.h / 2);
    }
  }
}

function drawAll() {
  drawDist();
  drawErr();
  drawPid();
  drawAck();
  drawDrive();
  drawTime();
}

// ---- cards ----------------------------------------------------------------

function put(id, txt, bad, warn) {
  const el = byId(id);
  el.textContent = txt;
  el.classList.toggle('na', !!bad);
  el.classList.toggle('warn', !!warn);
}

const mm = (v) => v >= FAR ? DASH : v + ' mm';
const deg = (v) => (v > 0 ? '+' : '') + (v / 10).toFixed(1) + '\u00b0';
const sgn = (v) => (v > 0 ? '+' : '') + v;

function fmtUp(ms) {
  const s = Math.floor(ms / 1000);
  const h = Math.floor(s / 3600);
  const m = Math.floor(s % 3600 / 60);
  if (h) return h + 'h ' + m + 'm';
  if (m) return m + 'm ' + (s % 60) + 's';
  return s + 's';
}

function paintSample() {
  const a = S.samples;
  const d = a.length ? a[a.length - 1] : null;
  put('v-state', d ? d.st : DASH);
  byId('v-state').style.color = d ? (STATE_C[d.st] || '#e8e8e8') : '#e8e8e8';
  put('v-t', d ? (d.t / 1000).toFixed(1) + ' s' : DASH);
  put('v-rawL', d ? mm(d.rawL) : DASH, !!d && d.rawL >= FAR);
  put('v-rawR', d ? mm(d.rawR) : DASH, !!d && d.rawR >= FAR);
  put('v-rawB', d ? mm(d.rawB) : DASH, !!d && d.rawB >= FAR);
  put('v-fL', d ? mm(d.fL) : DASH, !!d && d.fL >= FAR);
  put('v-fR', d ? mm(d.fR) : DASH, !!d && d.fR >= FAR);
  put('v-fB', d ? mm(d.fB) : DASH, !!d && d.fB >= FAR);
  put('v-latL', d && d.fL < FAR ? d.latL + ' mm' : DASH, !!d && d.fL >= FAR);
  put('v-latR', d && d.fR < FAR ? d.latR + ' mm' : DASH, !!d && d.fR >= FAR);
  // 12 mm is the alignment window from ParkingConfig; beyond it the car has to correct.
  put('v-err', d ? sgn(d.err) + ' mm' : DASH, false, !!d && Math.abs(d.err) > 12);
  put('v-p', d ? deg(d.p) : DASH);
  put('v-i', d ? deg(d.i) : DASH);
  put('v-d', d ? deg(d.d) : DASH);
  put('v-out', d ? deg(d.out) : DASH);
  put('v-sv', d ? d.sv + '\u00b0' : DASH);
  put('v-wl', d ? deg(d.wl) : DASH);
  put('v-wr', d ? deg(d.wr) : DASH);
  put('v-dir', d ? d.dir : DASH);
  byId('v-dir').style.color = d && d.dir === 'REV' ? C.REV
    : d && d.dir === 'FWD' ? C.FWD : '#9a9a9a';
  put('v-pwm', d ? String(d.pwm) : DASH);
  put('v-valid', d ? String(d.valid) : DASH);
  for (const b of [[1, 'v-bitL'], [2, 'v-bitR'], [4, 'v-bitB']]) {
    byId(b[1]).classList.toggle('on', !!d && !!(d.valid & b[0]));
  }
  put('v-prox', d ? (d.prox ? 'TRIPPED' : 'clear') : DASH, false, !!d && !!d.prox);
  put('v-both', d ? (d.both ? 'yes' : 'no') : DASH, !!d && !d.both);
  put('v-pf', d ? String(d.pf) : DASH);
}

function paintInfo() {
  const i = S.info;
  put('v-ip', i && i.ip ? i.ip : DASH);
  put('v-wifi', i && i.wifi ? String(i.wifi).toUpperCase() : DASH);
  put('v-up', i ? fmtUp(i.uptime_ms || 0) : DASH);
  put('v-samples', i ? String(i.total) : DASH);
  if (!i) { byId('meta').textContent = 'waiting for /api/info'; return; }
  const m = [String(i.wifi || '').toUpperCase(), i.ip || DASH, i.state,
             'up ' + fmtUp(i.uptime_ms || 0), i.total + ' samples'];
  if (i.caps) m.push('ring ' + i.caps + ' from ' + i.first);
  byId('meta').textContent = m.join(' \u00b7 ');
}

// ---- feed -----------------------------------------------------------------

async function getJSON(url) {
  const r = await fetch(url, { cache: 'no-store' });
  if (!r.ok) throw new Error('http ' + r.status);
  return r.json();
}

// sampleToJson sends flags & 0x07, so the firmware's pidActive bit never reaches the page. It zeros the terms outside REVERSE_PARK, so the state is the honest activity signal.
function pidOn(d) {
  return d.st === 'REVERSE_PARK' ? 1 : 0;
}

function ingest(r) {
  if (typeof r.total === 'number') S.total = r.total;
  if (typeof r.first === 'number') S.first = r.first;
  const list = r.samples || [];
  for (const d of list) {
    if (typeof d.t !== 'number' || typeof d.seq !== 'number') continue;
    // seq 0 means "from the oldest" on the server, so the first sample can be resent; drop what is already held.
    if (S.seen && d.seq <= S.lastSeq) continue;
    S.samples.push(Object.assign({ pa: pidOn(d) }, d)); // copy so pa is frozen at ingest
    S.seen = true;
    if (d.seq > S.lastSeq) S.lastSeq = d.seq;
  }
  const a = S.samples;
  const last = a.length ? a[a.length - 1].t : 0;
  while (a.length > CAP) a.shift();
  while (a.length && a[0].t < last - SPAN - 10000) a.shift();
  if (list.length) dirty = true;
}

function setStatus(txt) {
  pill.textContent = txt;
  pill.className = 'pill ' + txt;
}

function schedule(ms) {
  clearTimeout(timer);
  timer = setTimeout(poll, ms);
}

async function loadInfo() {
  if (infoBusy) return;
  infoBusy = true;
  S.infoAt = performance.now();
  try {
    const r = await getJSON('/api/info');
    S.info = r;
    if (typeof r.total === 'number') S.total = r.total;
    if (typeof r.first === 'number') S.first = r.first;
    if (typeof r.caps === 'number') S.caps = r.caps;
    paintInfo();
  } catch (e) {
    // poll() owns the link status; a failed info fetch rides the same poll backoff.
  } finally {
    infoBusy = false;
  }
}

async function poll() {
  const g = S.gen;
  let delay = 250;
  try {
    const r = await getJSON('/api/since?seq=' + S.lastSeq);
    if (g === S.gen) {
      ingest(r);
      S.fails = 0;
      setStatus('live');
      if (performance.now() - S.infoAt > 2000) loadInfo();
    }
  } catch (e) {
    if (g === S.gen) {
      S.fails++;
      setStatus('down');
      // One retry at 1 s, then hold 2 s: a rebooting AP needs a moment, but a dead link must not spin the radio.
      delay = S.fails > 1 ? 2000 : 1000;
    }
  }
  schedule(delay);
}

// The ring holds 2048 samples and the API returns at most 600, so a wrapped ring takes a few requests. Start at 0; the server answers from its oldest seq.
async function backfill() {
  let seq = 0;
  let hops = 0;
  while (hops++ < 60) {
    const r = await getJSON('/api/since?seq=' + seq);
    ingest(r);
    if (!(r.samples || []).length) break;
    if (S.lastSeq >= r.total - 1) break;
    if (S.lastSeq <= seq) break; // no progress: stop rather than loop on a ring that did not move
    seq = S.lastSeq;
    paintInfo();
  }
}

async function boot() {
  setStatus('polling');
  paintInfo();
  paintSample();
  try {
    await backfill();
    await loadInfo();
  } catch (e) {
    setStatus('down');
  }
  paintInfo();
  paintSample();
  dirty = true;
  schedule(0);
}

byId('reset').addEventListener('click', async () => {
  if (!confirm('Clear the sample ring on the device?')) return;
  try {
    await getJSON('/api/reset');
  } catch (e) {
    setStatus('down');
    return;
  }
  S.gen++; // any reply still in flight belongs to the old ring
  S.samples.length = 0;
  S.lastSeq = 0;
  S.seen = false;
  S.total = 0;
  S.first = 0;
  S.info = null;
  dirty = true;
  paintInfo();
  paintSample();
  setStatus('polling');
  await loadInfo();
});

// Card text is cheap but not free, so it settles at 5 Hz while the canvases follow new samples.
setInterval(() => {
  paintInfo();
  paintSample();
}, 200);

function loop() {
  if (dirty) { dirty = false; drawAll(); }
  requestAnimationFrame(loop);
}

window.addEventListener('resize', () => { dirty = true; });
requestAnimationFrame(loop);
boot();
</script>
</body>
</html>
)HTML";
