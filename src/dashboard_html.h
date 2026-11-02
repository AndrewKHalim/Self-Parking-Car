#pragma once

// The dashboard, as one self-contained page. No CDN, no framework, no webfont,
// so every byte comes from the ESP32. The websocket reconnects on its own and
// the page only shows the newest sample. There is no chart and no CSV, because
// the cells are the whole state of a car that only reverses in a straight line.

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
  button { width: 100%; margin-top: 12px; padding: 16px; font-size: 1.05rem;
           border: 0; border-radius: 10px; background: #2d6cdf; color: #fff; }
  button:disabled { background: #2a2a2a; color: #6a6a6a; }
</style>
</head>
<body>

<h1>autoparking</h1>
<div id="link">connecting</div>

<div class="grid">
  <div class="cell"><span class="k">state</span><span class="v" id="v-state">-</span></div>
  <div class="cell"><span class="k">gL</span><span class="v" id="v-gl">-</span></div>
  <div class="cell"><span class="k">gR</span><span class="v" id="v-gr">-</span></div>
  <div class="cell"><span class="k">rear</span><span class="v" id="v-rear">-</span></div>
  <div class="cell"><span class="k">servo</span><span class="v" id="v-servo">-</span></div>
  <div class="cell"><span class="k">ready</span><span class="v" id="v-ready">-</span></div>
  <div class="cell"><span class="k">fault</span><span class="v" id="v-fault">-</span></div>
</div>

<button id="start" disabled>start</button>

<script>
// The firmware sends raw millimetres. 8190 is no target, 65535 is a timeout,
// and 0 to 29 is the part's minimum range, which means the face is at the wall
// rather than 29 mm away.
const NO_TARGET = 8190;
const TIMEOUT = 65535;
const WALL_MAX = 29;

const byId = (id) => document.getElementById(id);
const link = byId('link');
const button = byId('start');
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
// only live where a start is accepted.
function show(d) {
  byId('v-state').textContent = d.state;
  byId('v-gl').textContent = mm(d.gL);
  byId('v-gr').textContent = mm(d.gR);
  byId('v-rear').textContent = mm(d.rear);
  byId('v-servo').textContent = d.servo + '\u00b0';
  byId('v-ready').textContent = d.ready ? 'yes' : 'no';
  byId('v-fault').textContent = d.fault;
  button.disabled = !d.ready;
}

button.addEventListener('click', () => {
  button.disabled = true;
  fetch('/start').catch(() => {});
});

connect();
</script>
</body>
</html>
)HTML";
