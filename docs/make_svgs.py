#!/usr/bin/env python3
"""Regenerate the diagrams under docs/.

Five figures, one file each, built from the node and edge tables below:

    docs/flow-webui.svg    runtime + dashboard: two tasks and the queue
    docs/flow-parking.svg  the phase machine, decision by decision
    docs/flow-guard.svg    the reverse-leg collision guard and unstick
    docs/block-pid.svg     the lateral PID as a block diagram with feedback
    docs/scurve.svg        what the S-curve does, and why it works

Run from the repo root:

    python3 docs/make_svgs.py

Style is plain TikZ: white paper, thin black edges, blue!10 processes, red!10
decisions, grey terminators. The drawing carries only labels and yes/no; every
sentence lives in the notes block under the figure. Node text is at most two
lines, and fit() warns when a label outgrows its shape.
"""

import os

SERIF = ("'Latin Modern Roman', 'CMU Serif', Georgia, 'Times New Roman', "
         "serif")

PAL = {
    "paper": "#ffffff",
    "ink": "#000000",
    "ink70": "#4d4d4d",
    "ink50": "#808080",
    "ink20": "#cccccc",
    "proc_fill": "#e6e6ff",   # blue!10
    "proc_line": "#6666ff",   # blue!60
    "dec_fill": "#ffe6e6",    # red!10
    "dec_line": "#e64545",    # red!70
    "term_fill": "#e6e6e6",   # black!10
    "term_line": "#4d4d4d",   # black!70
    "io_fill": "#fff0e0",     # orange!12
    "io_line": "#e6994d",     # orange!60
    "ok_fill": "#ccffcc",     # green!20
    "ok_line": "#4dcc4d",     # green!70
    "err_fill": "#ffcccc",    # red!20
    "err_line": "#cc0000",
    "c1": "#0000bf",
    "c2": "#bf0000",
}

S = 12.0
SB = 10.5
LAB = 9.5
CAP = 10.5

warnings = []
_prefix = ""
N = {}
OUT = []


def use(name):
    global _prefix
    _prefix = name
    N.clear()
    del OUT[:]


def esc(t):
    return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def width_of(t, size):
    wide = sum(1 for c in t if c.isupper() or c in "MW@m")
    return (wide * 0.62 + (len(t) - wide) * 0.50) * size


def fit(t, size, room, what):
    if width_of(t, size) > room:
        warnings.append(f"{what}: {t!r} needs {width_of(t, size):.0f}px, "
                        f"room {room:.0f}px")


def text(x, y, body, size=S, fill=None, weight="400", anchor="start"):
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" '
            f'fill="{fill or PAL["ink"]}" font-weight="{weight}" '
            f'text-anchor="{anchor}">{esc(body)}</text>')


def reg(name, x, y, w, h):
    N[name] = {"x": x, "y": y, "w": w, "h": h,
               "cx": x + w / 2, "cy": y + h / 2}
    return N[name]


def _label(x, cy, w, title, sub):
    if sub:
        fit(title, S, w, "node")
        fit(sub, SB, w, "node sub")
        return (text(x + w / 2, cy - 2, title, anchor="middle")
                + text(x + w / 2, cy + 14, sub, size=SB, fill=PAL["ink70"],
                       anchor="middle"))
    fit(title, S, w, "node")
    return text(x + w / 2, cy + 4, title, anchor="middle")


def proc(name, x, y, w, h, title, sub=None, fill=None, line=None):
    reg(name, x, y, w, h)
    OUT.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" '
               f'fill="{fill or PAL["proc_fill"]}" '
               f'stroke="{line or PAL["proc_line"]}" stroke-width="1"/>'
               + _label(x, N[name]["cy"], w, title, sub))


def term(name, x, y, w, h, title, sub=None, fill=None, line=None):
    reg(name, x, y, w, h)
    OUT.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{h / 2}" '
               f'fill="{fill or PAL["term_fill"]}" '
               f'stroke="{line or PAL["term_line"]}" stroke-width="1"/>'
               + _label(x, N[name]["cy"], w, title, sub))


def io(name, x, y, w, h, title, sub=None, skew=14):
    reg(name, x, y, w, h)
    d = (f'M {x + skew} {y} L {x + w} {y} L {x + w - skew} {y + h} '
         f'L {x} {y + h} Z')
    OUT.append(f'<path d="{d}" fill="{PAL["io_fill"]}" '
               f'stroke="{PAL["io_line"]}" stroke-width="1"/>'
               + _label(x + skew / 2, N[name]["cy"], w - skew, title, sub))


def dec(name, cx, cy, w, h, title, sub=None):
    x, y = cx - w / 2, cy - h / 2
    reg(name, x, y, w, h)
    d = f'M {cx} {y} L {cx + w / 2} {cy} L {cx} {y + h} L {cx - w / 2} {cy} Z'
    OUT.append(f'<path d="{d}" fill="{PAL["dec_fill"]}" '
               f'stroke="{PAL["dec_line"]}" stroke-width="1"/>'
               + _label(x + w * 0.2, cy, w * 0.6, title, sub))


def conn(name, cx, cy, letter):
    reg(name, cx - 15, cy - 15, 30, 30)
    OUT.append(f'<circle cx="{cx}" cy="{cy}" r="15" fill="{PAL["paper"]}" '
               f'stroke="{PAL["ink70"]}" stroke-width="1"/>'
               + text(cx, cy + 4, letter, anchor="middle"))


def marker():
    return (f'<defs><marker id="{_prefix}arw" viewBox="0 0 8 8" refX="7.6" '
            f'refY="4" markerWidth="7" markerHeight="7" '
            f'orient="auto-start-reverse"><path d="M 0 0.8 L 8 4 L 0 7.2 z" '
            f'fill="{PAL["ink70"]}"/></marker></defs>')


def anchor_of(name, side):
    n = N[name]
    if side == "top":
        return (n["cx"], n["y"])
    if side == "bottom":
        return (n["cx"], n["y"] + n["h"])
    if side == "left":
        return (n["x"], n["cy"])
    if side == "right":
        return (n["x"] + n["w"], n["cy"])
    raise ValueError(side)


def arrow(a, b, sa=None, sb=None, label=None, dx=0, dy=0):
    p = anchor_of(a, sa) if isinstance(a, str) else a
    q = anchor_of(b, sb) if isinstance(b, str) else b
    out = (f'<line x1="{p[0]:.1f}" y1="{p[1]:.1f}" x2="{q[0]:.1f}" '
           f'y2="{q[1]:.1f}" stroke="{PAL["ink70"]}" stroke-width="1" '
           f'marker-end="url(#{_prefix}arw)"/>')
    if label:
        out += text((p[0] + q[0]) / 2 + dx, (p[1] + q[1]) / 2 + dy, label,
                    size=LAB, fill=PAL["ink70"], anchor="middle")
    return out


def elbow(points, label=None, at=None):
    d = "M " + " L ".join(f"{x:.1f} {y:.1f}" for x, y in points)
    out = (f'<path d="{d}" fill="none" stroke="{PAL["ink70"]}" '
           f'stroke-width="1" marker-end="url(#{_prefix}arw)"/>')
    if label and at:
        out += text(at[0], at[1], label, size=LAB, fill=PAL["ink70"])
    return out


def wire(points):
    d = "M " + " L ".join(f"{x:.1f} {y:.1f}" for x, y in points)
    return (f'<path d="{d}" fill="none" stroke="{PAL["ink70"]}" '
            f'stroke-width="1"/>')


def polyline(points, colour, width=1.6, dash=None):
    d = "M " + " L ".join(f"{x:.1f} {y:.1f}" for x, y in points)
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return (f'<path d="{d}" fill="none" stroke="{colour}" '
            f'stroke-width="{width}" stroke-linejoin="round" '
            f'stroke-linecap="round"{dash_attr}/>')


def title(x, y, body):
    return text(x, y, body, size=CAP, fill=PAL["ink70"])


def frame(x, y, w, h, fill=None):
    return (f'<rect x="{x}" y="{y}" width="{w}" height="{h}" '
            f'fill="{fill or "none"}" stroke="{PAL["ink20"]}" '
            f'stroke-width="0.8"/>')


def notes(x, y, lines, width=300):
    """The figure's footnotes: a rule, then numbered roman sentences."""
    out = [f'<line x1="{x}" y1="{y}" x2="{x + width}" y2="{y}" '
           f'stroke="{PAL["ink20"]}" stroke-width="0.8"/>']
    for i, line in enumerate(lines, 1):
        out.append(text(x, y + 15 * i, f"{i}. {line}", size=LAB,
                        fill=PAL["ink70"]))
    return "".join(out)


def svg(width, height, body):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} '
            f'{height}" width="{width}" height="{height}" '
            f'font-family="{SERIF}" role="img">'
            f'<rect x="0" y="0" width="{width}" height="{height}" '
            f'fill="{PAL["paper"]}"/>' + marker() + "".join(body) + "</svg>")


# --------------------------------------------------------------------------
# 1. Runtime and web UI.
# --------------------------------------------------------------------------

def flow_webui():
    use("webui-")
    s = OUT
    s.append(title(40, 30, "runtime: one task owns the car, one owns the "
                           "network"))
    s.append(text(40, 56, "core 0", size=LAB, fill=PAL["ink50"]))
    s.append(text(380, 56, "shared", size=LAB, fill=PAL["ink50"]))
    s.append(text(660, 56, "core 1", size=LAB, fill=PAL["ink50"]))

    io("browser", 40, 96, 280, 44, "browser", "WebSocket, chart")
    proc("handlers", 40, 176, 280, 58, "HTTP handlers",
         "/ start stop config")
    proc("publish", 40, 336, 280, 58, "publishTelemetry", "JSON, 10 Hz")

    proc("flags", 380, 176, 220, 58, "atomic flags", "store only")
    proc("slot", 380, 336, 220, 58, "Telemetry slot", "one slot, newest")

    proc("exchange", 660, 80, 280, 44, "exchange()")
    proc("read", 660, 160, 280, 58, "sensorBank.read()", "3 sensors, 30 Hz")
    proc("apply", 660, 254, 280, 58, "apply requests", "stop wins, then start")
    proc("tick", 660, 348, 280, 44, "machine.tick()")
    proc("overwrite", 660, 428, 280, 58, "xQueueOverwrite", "slot, newest")
    proc("delay", 660, 522, 280, 44, "delay 33 ms")

    s.append(arrow("browser", "handlers", "bottom", "top"))
    s.append(arrow("handlers", "flags", "right", "left", "POST, GET", dy=-6))
    s.append(elbow([anchor_of("flags", "right"), (630, 205), (630, 102),
                    anchor_of("exchange", "left")], "store", (612, 130)))
    s.append(arrow("exchange", "read", "bottom", "top"))
    s.append(arrow("read", "apply", "bottom", "top"))
    s.append(arrow("apply", "tick", "bottom", "top"))
    s.append(arrow("tick", "overwrite", "bottom", "top"))
    s.append(arrow("overwrite", "delay", "bottom", "top"))
    s.append(elbow([anchor_of("overwrite", "left"), (630, 457), (630, 365),
                    anchor_of("slot", "right")], "tick result", (628, 476)))
    s.append(arrow("slot", "publish", "left", "right"))
    s.append(elbow([anchor_of("publish", "left"), (20, 365), (20, 118),
                    anchor_of("browser", "left")], "push", (24, 250)))
    s.append(elbow([anchor_of("delay", "right"), (966, 544), (966, 74),
                    (800, 74), anchor_of("exchange", "top")], "30 Hz",
                   (912, 508)))
    s.append(notes(40, 600, [
        "handlers only store a flag; the control task applies it next tick",
        "the slot carries a copy, never a half-written state",
        "stop is applied before start, so an abort is never swallowed",
    ], width=380))
    return s, 980, 700


# --------------------------------------------------------------------------
# 2. Parking logic.
# --------------------------------------------------------------------------

def flow_parking():
    use("park-")
    s = OUT
    s.append(title(40, 30, "parking logic: one phase per tick, decisions on "
                           "the settled sample"))
    CX, W = 470, 290

    term("start", CX - 75, 44, 150, 40, "start()")
    dec("rearok", CX, 140, 240, 84, "rear valid?")
    dec("inband", CX, 244, 240, 84, "rear ≤ target?")
    proc("rev", CX - W / 2, 318, W, 48, "REVERSE")
    dec("stopq", CX, 418, 250, 84, "stop? see guards")
    proc("settle", CX - W / 2, 488, W, 58, "SETTLE_REAR", "settleRear()")
    dec("gaps", CX, 604, 250, 84, "gaps readable?")
    dec("nudged", CX, 708, 250, 84, "nudge overshot?")
    dec("ok", CX, 824, 270, 84, "ok and not early?")
    dec("steep", CX, 942, 230, 84, "yaw ≥ 12°?")
    dec("latok", CX, 1060, 230, 84, "lat ok?")
    proc("scurve", CX - W / 2, 1128, W, 58, "S-curve, half + half",
         "STEER_WAIT first")
    proc("settlef", CX - W / 2, 1230, W, 58, "SETTLE_FORWARD",
         "brake, recentre servo")
    dec("band", CX, 1356, 240, 84, "rear in band?")

    proc("arc", 60, 918, 200, 48, "one arc, 1000 ms")
    proc("half", 60, 1036, 200, 48, "half stroke, yaw trim")

    proc("fault1", 700, 117, 210, 46, "rear_sensor", "FAULT",
         PAL["err_fill"], PAL["err_line"])
    proc("fsensor", 700, 581, 210, 46, "side_sensor", "FAULT",
         PAL["err_fill"], PAL["err_line"])
    term("parked", 725, 700, 150, 44, "PARKED", fill=PAL["ok_fill"],
         line=PAL["ok_line"])
    dec("close", 800, 824, 240, 84, "rear < target − tol?")
    dec("capq", 800, 942, 220, 84, "backoffs < cap?")
    proc("nudge", 700, 1010, 200, 58, "nudge 120–260 ms",
         "straight, centred")
    proc("fault2", 700, 1110, 210, 46, "backoff_limit", "FAULT",
         PAL["err_fill"], PAL["err_line"])

    conn("A", 290, 342, "A")
    conn("B", 290, 517, "B")
    conn("Bi", 640, 244, "B")
    conn("Aj", 640, 708, "A")
    conn("Bn", 640, 1039, "B")
    conn("A9", 250, 1356, "A")
    conn("B9", 690, 1356, "B")

    s.append(arrow("start", "rearok", "bottom", "top"))
    s.append(arrow("rearok", "inband", "bottom", "top", "yes", dx=26))
    s.append(arrow("inband", "rev", "bottom", "top", "no", dx=20))
    s.append(arrow("rev", "stopq", "bottom", "top"))
    s.append(arrow("stopq", "settle", "bottom", "top", "yes", dx=28))
    s.append(arrow("settle", "gaps", "bottom", "top"))
    s.append(arrow("gaps", "nudged", "bottom", "top", "yes", dx=24))
    s.append(arrow("nudged", "ok", "bottom", "top", "no", dx=20))
    s.append(arrow("ok", "steep", "bottom", "top", "no", dx=20))
    s.append(arrow("steep", "latok", "bottom", "top", "no", dx=20))
    s.append(arrow("latok", "scurve", "bottom", "top", "no", dx=20))
    s.append(arrow("scurve", "settlef", "bottom", "top"))
    s.append(arrow("settlef", "band", "bottom", "top"))

    s.append(arrow("rearok", "fault1", "right", "left", "no", dy=-6))
    s.append(arrow("inband", "Bi", "right", "left", "yes", dy=-8))
    s.append(elbow([anchor_of("stopq", "left"), (290, 418), (290, 357)],
                   "no", (296, 406)))
    s.append(arrow("A", "rev", "right", "left"))
    s.append(arrow("B", "settle", "right", "left"))
    s.append(arrow("gaps", "fsensor", "right", "left", "no", dy=-6))
    s.append(arrow("nudged", "Aj", "right", "left", "yes", dy=-8))

    s.append(arrow("ok", "close", "right", "left", "yes", dy=-8))
    s.append(arrow("close", "parked", "top", "bottom", "no", dx=18))
    s.append(arrow("close", "capq", "bottom", "top", "yes", dx=22))
    s.append(arrow("capq", "nudge", "bottom", "top", "yes", dx=22))
    s.append(arrow("nudge", "Bn", "left", "right"))
    s.append(elbow([anchor_of("capq", "right"), (940, 942), (940, 1133),
                    anchor_of("fault2", "right")], "no", (946, 952)))

    s.append(arrow("steep", "arc", "left", "right", "yes", dy=-8))
    s.append(arrow("latok", "half", "left", "right", "yes", dy=-8))
    # both strokes leave on the far left and join before SETTLE_FORWARD; the
    # dot is an OR, spelled out in the notes
    s.append(wire([anchor_of("arc", "left"), (36, 942), (36, 1060)]))
    s.append(wire([anchor_of("half", "left"), (36, 1060)]))
    s.append(f'<circle cx="36" cy="1060" r="3" fill="{PAL["ink70"]}"/>')
    s.append(elbow([(36, 1060), (36, 1259), anchor_of("settlef", "left")]))

    s.append(arrow("band", "B9", "right", "left", "yes", dy=-8))
    s.append(arrow("band", "A9", "left", "right", "no", dy=-8))

    s.append(notes(40, 1450, [
        "A and B are jump labels: the same letter joins the two ends",
        "the dot where two wires join is an OR: either one continues",
        "an early stop never parks: right gaps, wrong distance",
        "a nudge that pulls past target + 25 mm re-enters REVERSE",
        "the cap is a hard stop: backoff_limit faults out instead of nudging",
        "the strokes themselves are in scurve.svg",
    ], width=520))
    return s, 980, 1560


# --------------------------------------------------------------------------
# 3. Reverse-leg guard.
# --------------------------------------------------------------------------

def flow_guard():
    use("guard-")
    s = OUT
    s.append(title(40, 30, "reverse-leg guard: four checks, in this order"))

    term("tick", 325, 64, 150, 44, "tick()", "33 ms")
    proc("anchor", 250, 132, 300, 48, "latch the anchor",
         "recordReverseStart, once per leg")
    dec("lead", 400, 244, 250, 84, "rear ≤ target + 15?")
    dec("drive", 400, 376, 250, 84, "elapsed ≥ 350 ms?")
    proc("slope", 250, 432, 300, 58, "live tan psi", "once moved ≥ 15 mm")
    proc("clear", 250, 518, 300, 58, "project each tire",
         "gap − 37.5 − lever·psi")
    dec("closeq", 400, 646, 270, 84, "clear ≤ 17.5, closing?")
    dec("stalled", 400, 778, 270, 84, "8 mm in 700 ms?")
    dec("timeout", 400, 910, 230, 84, "6000 ms?")
    term("next", 325, 968, 150, 40, "next tick")

    proc("brake", 700, 220, 220, 48, "brake", "→ SETTLE_REAR")
    proc("early", 700, 622, 220, 48, "brake, early stop", "→ SETTLE_REAR")
    proc("unstick", 700, 754, 220, 48, "unstick ≥ 9.5°",
         "→ correction stroke")
    proc("fault", 700, 886, 220, 48, "rev_timeout", "FAULT",
         PAL["err_fill"], PAL["err_line"])

    s.append(arrow("tick", "anchor", "bottom", "top"))
    s.append(arrow("anchor", "lead", "bottom", "top"))
    s.append(arrow("lead", "brake", "right", "left", "yes", dy=-6))
    s.append(arrow("lead", "drive", "bottom", "top", "no", dx=20))
    s.append(arrow("drive", "slope", "bottom", "top", "yes", dx=26))
    s.append(arrow("slope", "clear", "bottom", "top"))
    s.append(arrow("clear", "closeq", "bottom", "top"))
    s.append(arrow("closeq", "early", "right", "left", "yes", dy=-6))
    s.append(arrow("closeq", "stalled", "bottom", "top", "no", dx=20))
    s.append(arrow("stalled", "unstick", "right", "left", "yes", dy=-6))
    s.append(arrow("stalled", "timeout", "bottom", "top", "no", dx=20))
    s.append(arrow("timeout", "fault", "right", "left", "yes", dy=-6))
    s.append(arrow("timeout", "next", "bottom", "top", "no", dx=20))
    s.append(elbow([anchor_of("drive", "left"), (180, 376), (180, 778),
                    anchor_of("stalled", "left")], "no", (186, 560)))
    s.append(elbow([anchor_of("next", "left"), (60, 988), (60, 86),
                    anchor_of("tick", "left")], "next tick", (66, 520)))
    s.append(notes(40, 1024, [
        "the lead check is first: the brake has to beat the 10-15 mm coast",
        "the side guard is skipped until the car has driven 15 mm",
        "17.5 mm at the tire is 55 mm at the sensor when psi is 0",
        "under 8 mm of travel in 700 ms means a wheel is hung on a wall",
    ], width=460))
    return s, 980, 1100


# --------------------------------------------------------------------------
# 4. PID block diagram with feedback.
# --------------------------------------------------------------------------

def block_pid():
    use("pid-")
    s = OUT
    s.append(title(40, 30, "the lateral PID as a loop: the offset goes in, the "
                           "servo angle comes out"))

    AX = 126
    term("sp", 40, AX - 22, 160, 44, "set point", "offset 0 mm")
    s.append(f'<circle cx="245" cy="{AX}" r="18" fill="{PAL["paper"]}" '
             f'stroke="{PAL["ink70"]}" stroke-width="1"/>')
    s.append(text(245, AX + 5, "Σ", size=14, anchor="middle"))
    s.append(text(259, AX + 14, "−", size=13, fill=PAL["ink70"]))

    proc("pid", 300, AX - 34, 230, 68, "LateralPID",
         "Kp·e + Ki∫e + Kd·de, ±15.4°")
    proc("servo", 550, AX - 34, 230, 68, "setWheelAngleDeg",
         "servo = 90 + 2.92·wheel")

    proc("car", 600, 190, 280, 64, "chassis", "servo held for 500 ms")
    proc("sensors", 330, 190, 250, 64, "VL53L0X", "gL, gR after the move")
    proc("est", 60, 190, 250, 64, "corr_diff_mm", "gL − gR − 56·tan psi")

    s.append(arrow("sp", (227, AX), "right"))
    s.append(text(262, AX - 12, "e", size=LAB, fill=PAL["ink70"],
                  anchor="middle"))
    s.append(arrow((263, AX), (300, AX)))
    s.append(arrow((530, AX), (550, AX)))
    s.append(arrow((780, AX), (900, AX)))
    s.append(text(846, AX - 12, "u: servo angle", size=CAP, anchor="middle"))
    s.append(text(878, AX + 22, "output", size=LAB, fill=PAL["ink70"],
                  anchor="end"))

    # the output is applied, the car moves, and the gaps are read again: the
    # feedback enters the chassis block on its right edge
    s.append(elbow([(900, AX + 2), (900, AX + 96), (882, AX + 96)]))
    s.append(arrow("car", "sensors", "left", "right"))
    s.append(arrow("sensors", "est", "left", "right"))
    s.append(elbow([anchor_of("est", "left"), (36, AX + 96), (36, AX + 34),
                    (245, AX + 34), (245, AX + 20)]))
    s.append(text(150, AX + 28, "y", size=LAB, fill=PAL["ink70"]))

    s.append(notes(40, 300, [
        "the set point is 0 mm: the tail belongs centred between the gaps",
        "e is 0 minus y, and y is the offset the sensors report after the move",
        "the block runs once per cycle, on a settled and braked chassis",
        "gains: Kp 0.15 deg/mm, Ki 0.01 with the integral capped at 5 deg",
        "the servo angle is applied for 500 ms, then gL and gR are read again",
    ], width=600))
    return s, 980, 420


# --------------------------------------------------------------------------
# 5. S-curve.
# --------------------------------------------------------------------------

def scurve():
    use("sc-")
    s = OUT
    s.append(title(40, 30, "correction stroke: two halves, opposite lock, "
                           "equal time"))

    x0, x1, yA, yB = 120, 320, 330, 150
    pts = []
    for i in range(41):
        t = i / 40.0
        pts.append((x0 + (x1 - x0) * (3 * t * t - 2 * t * t * t),
                    yA + (yB - yA) * t))
    mid = len(pts) // 2
    s.append(polyline(pts[:mid + 1], PAL["c1"], 2.2))
    s.append(polyline(pts[mid:], PAL["c2"], 2.2))
    s.append(arrow(pts[-4], pts[-1]))

    for cx, cy in ((x0, yA), (x1, yB)):
        s.append(f'<rect x="{cx - 20}" y="{cy - 30}" width="40" height="60" '
                 f'rx="7" fill="none" stroke="{PAL["ink50"]}" '
                 f'stroke-width="1"/>')
    s.append(text(x0 - 30, yA + 4, "in", size=LAB, fill=PAL["ink50"],
                  anchor="end"))
    s.append(text(x1 + 30, yB + 4, "out", size=LAB, fill=PAL["ink50"]))
    s.append(f'<line x1="{x0}" y1="366" x2="{x1}" y2="366" '
             f'stroke="{PAL["ink50"]}" stroke-width="0.8" '
             f'marker-start="url(#{_prefix}arw)" '
             f'marker-end="url(#{_prefix}arw)"/>')
    s.append(text((x0 + x1) / 2, 382, "net shift", size=LAB,
                  fill=PAL["ink50"], anchor="middle"))
    s.append(text(40, 308, "half 1: +d", size=LAB, fill=PAL["c1"]))
    s.append(text(258, 188, "half 2: −d", size=LAB, fill=PAL["c2"],
                  anchor="end"))

    # The phases as a ruler, then the three signals they produce.
    L, R = 470, 950
    span = R - L

    def at(ms):
        return L + span * (ms / 1900.0)

    phases = [("STEER_WAIT", 0, 300, None),
              ("FORWARD_CORRECT", 300, 800, "#e8e8ff"),
              ("STEER_COUNTER", 800, 1100, None),
              ("FORWARD_COUNTER", 1100, 1600, "#ffe8e8"),
              ("SETTLE", 1600, 1900, None)]
    for name, a, b, tint in phases:
        s.append(frame(at(a), 58, at(b) - at(a), 16, tint))
        s.append(text((at(a) + at(b)) / 2, 70, name, size=8.0,
                      fill=PAL["ink70"], anchor="middle"))
        if tint:
            for plot_y in (104, 202, 300):
                s.append(frame(at(a), plot_y, at(b) - at(a), 54, tint))

    rows = [("wheel angle", 104, "+d", "0", "−d"),
            ("heading", 202, "θ", "0", None),
            ("tail offset", 300, "Δ", "0", None)]
    for label, top, hi, zero, lo in rows:
        s.append(text(L, top - 6, label, size=CAP, fill=PAL["ink"]))
        s.append(frame(L, top, span, 54))
        s.append(f'<line x1="{L}" y1="{top + 27}" x2="{R}" y2="{top + 27}" '
                 f'stroke="{PAL["ink20"]}" stroke-dasharray="3 3"/>')
        s.append(text(L - 6, top + 10, hi, size=LAB, fill=PAL["ink50"],
                      anchor="end"))
        s.append(text(L - 6, top + 31, zero, size=LAB, fill=PAL["ink50"],
                      anchor="end"))
        if lo:
            s.append(text(L - 6, top + 52, lo, size=LAB, fill=PAL["ink50"],
                          anchor="end"))

    z = 131
    s.append(polyline([(L, z), (at(300), z - 22), (at(800), z - 22),
                       (at(1100), z + 22), (at(1600), z + 22),
                       (at(1900), z)], PAL["c1"], 1.8))
    s.append(polyline([(L, 229), (at(300), 229), (at(800), 207),
                       (at(1100), 207), (at(1600), 229), (at(1900), 229)],
                      PAL["c1"], 1.8))
    s.append(text(R, 194, "back to 0", size=LAB, fill=PAL["ink70"],
                  anchor="end"))
    offs = []
    for i in range(21):
        t = i / 20.0
        offs.append((L + span * t, 327 - 20 * (3 * t * t - 2 * t * t * t)))
    s.append(polyline(offs[:11], PAL["c1"], 1.8))
    s.append(polyline(offs[10:], PAL["c2"], 1.8))
    s.append(text(R, 292, "shifted by Δ", size=LAB, fill=PAL["ink70"],
                  anchor="end"))
    s.append(text(L, 372, "0 ms", size=LAB, fill=PAL["ink50"]))
    s.append(text(R, 372, "1900 ms", size=LAB, fill=PAL["ink50"],
                  anchor="end"))

    proc("lat", 470, 404, 230, 58, "lateral fix", "first_w = +lat + ½yaw")
    proc("trim", 720, 404, 240, 58, "yaw trim only", "wheel = Kyaw · yaw")
    proc("arc", 470, 486, 230, 58, "yaw ≥ 12 deg", "one arc, no mirror",
         PAL["dec_fill"], PAL["dec_line"])
    s.append(notes(40, 580, [
        "the first half steers toward the gap; the second mirrors it",
        "equal lock for equal time cancels the rotation: heading out = in",
        "the offset does not cancel, both halves push the tail the same way",
        "above 12 deg of yaw the S-curve is replaced by one straightening arc",
    ], width=460))
    return s, 980, 660


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    for name, build in (("flow-webui.svg", flow_webui),
                        ("flow-parking.svg", flow_parking),
                        ("flow-guard.svg", flow_guard),
                        ("block-pid.svg", block_pid),
                        ("scurve.svg", scurve)):
        body, w, h = build()
        out = svg(w, h, body)
        path = os.path.join(here, name)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(out + "\n")
        print(f"wrote {path} ({len(out)} bytes)")
    for w in warnings:
        print(f"warning: {w}")


if __name__ == "__main__":
    main()
