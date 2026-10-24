#!/usr/bin/env python3
"""plots the step-response evidence and computes its metrics.

reads the two CSVs written by build/step_rig: the time series and the run
info. computes rise time, overshoot, settling time, steady-state error and
peak effort from the measured column, prints them, writes metrics.txt, and
saves the step response as PNG and SVG.

    python3 -m venv .venv-plot
    .venv-plot/bin/pip install -r tools/plot-requirements.txt
    .venv-plot/bin/python tools/step_plot.py
"""

import argparse
import csv
import os


def read_info(path):
    info = {}
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or "=" not in line:
                continue
            key, value = line.split("=", 1)
            info[key] = value
    return info


def read_rows(path):
    with open(path) as fh:
        return list(csv.DictReader(fh))


# the first time the trace crosses a level, interpolated between samples.
def cross_time(rows, level, start):
    for i in range(start + 1, len(rows)):
        y0 = float(rows[i - 1]["x_meas_m"]) * 1000.0
        y1 = float(rows[i]["x_meas_m"]) * 1000.0
        if (y0 < level <= y1) or (y0 > level >= y1):
            if y1 == y0:
                return float(rows[i]["t_s"])
            frac = (level - y0) / (y1 - y0)
            return float(rows[i - 1]["t_s"]) + frac * (
                float(rows[i]["t_s"]) - float(rows[i - 1]["t_s"]))
    return float("nan")


# all metrics come from the measured column, the same data as the plot.
def metrics(rows, info):
    ref = float(info["ref_m"]) * 1000.0
    t_step = float(info["t_step_s"])
    start = next(i for i, r in enumerate(rows) if float(r["ref_m"]) > 0.0)
    pre = [float(r["x_meas_m"]) * 1000.0 for r in rows[:start]]
    base = sum(pre) / len(pre) if pre else 0.0
    amp = ref - base

    t10 = cross_time(rows, base + 0.1 * amp, start)
    t90 = cross_time(rows, base + 0.9 * amp, start)

    post = [float(r["x_meas_m"]) * 1000.0 for r in rows[start:]]
    peak = max(post)
    overshoot = max(0.0, (peak - ref) / abs(ref) * 100.0)

    band = 0.02 * abs(ref)
    outside = [i for i in range(start, len(rows))
               if abs(float(rows[i]["x_meas_m"]) * 1000.0 - ref) > band]
    settle = float(rows[outside[-1]]["t_s"]) - t_step if outside else 0.0
    settled = bool(outside) and outside[-1] < len(rows) - 2

    tail = [r for r in rows if float(r["t_s"]) >= float(rows[-1]["t_s"]) - 1.0]
    ss = sum(float(r["x_meas_m"]) * 1000.0 for r in tail) / len(tail) - ref

    effort = max(rows, key=lambda r: abs(float(r["steer_cmd_rad"])))
    peak_cmd = abs(float(effort["steer_cmd_rad"])) * 57.29578
    peak_psi = max(abs(float(r["psi_rad"])) for r in rows) * 57.29578

    return {
        "ref": ref, "base": base, "t_step": t_step, "t10": t10, "t90": t90,
        "rise": t90 - t10, "overshoot": overshoot, "settle": settle,
        "band": band, "ss": ss, "ss_pct": ss / abs(ref) * 100.0,
        "peak_cmd": peak_cmd, "peak_cmd_t": float(effort["t_s"]),
        "peak_psi": peak_psi, "final": post[-1], "settled": settled,
    }


def print_metrics(m, info):
    settle_txt = (f"{m['settle']:.2f} s" if m["settled"] else
                  f"not settled, last excursion {m['settle']:.2f} s after the step")
    lines = [
        "step response metrics",
        f"  reference step: {m['ref']:.2f} mm at t = {m['t_step']:.2f} s",
        f"  speed: {float(info['speed_mps']):.3f} m/s",
        f"  rise time 10-90%: {m['rise']:.2f} s "
        f"(t10 {m['t10']:.2f} s, t90 {m['t90']:.2f} s)",
        f"  overshoot: {m['overshoot']:.2f} %",
        f"  settling time 2% band: {settle_txt} "
        f"(band {m['band']:.2f} mm)",
        f"  steady-state error (last 1 s mean): "
        f"{m['ss']:.2f} mm ({m['ss_pct']:.2f} %)",
        f"  final value: {m['final']:.2f} mm",
        f"  peak steering command: {m['peak_cmd']:.2f} deg "
        f"at t = {m['peak_cmd_t']:.2f} s",
        f"  peak heading: {m['peak_psi']:.2f} deg",
        f"  seed {info['seed']}, measurement noise {float(info['noise_m']):.4f} m",
    ]
    text = "\n".join(lines)
    print(text)
    return text


def plot_step(rows, info, m, outdir, plt, suffix="", note="", window=8.0):
    plt.rcParams.update({
        "font.size": 11, "axes.titlesize": 13, "axes.labelsize": 12,
        "legend.fontsize": 10, "xtick.labelsize": 10, "ytick.labelsize": 10,
    })

    t = [float(r["t_s"]) for r in rows]
    ref = [float(r["ref_m"]) * 1000.0 for r in rows]
    x = [float(r["x_meas_m"]) * 1000.0 for r in rows]
    t_end = min(t[-1], m["t_step"] + window)
    speed = float(info["speed_mps"])
    tag = f" ({note})" if note else ""

    fig, ax = plt.subplots(figsize=(9.2, 5.4))
    ax.axhspan(m["ref"] - m["band"], m["ref"] + m["band"],
               color="#cfe8cf", alpha=0.75, label="2% band")
    ax.plot(t, x, color="#1f4e9c", linewidth=2.0, label="measured x")
    ax.plot(t, ref, color="#333333", linewidth=1.6, linestyle="--",
            label="reference")
    ax.axvline(m["t_step"], color="#777777", linewidth=1.2,
               linestyle=":")
    ax.text(m["t_step"] + 0.12, 3.0, "step", color="#555555", fontsize=10)

    ax.set_xlim(0, t_end)
    ax.set_ylim(-3.0, 48.0)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("lateral offset x (mm)")
    ax.set_title("lateral position step response, "
                 f"{m['ref']:.0f} mm reference at {speed:.2f} m/s{tag}")
    ax.grid(True, color="#d0d0d0", linewidth=0.7)
    ax.legend(loc="lower right", framealpha=0.95)

    settle_box = f"{m['settle']:.2f} s" if m["settled"] else "not settled"
    box = (f"rise 10-90%  {m['rise']:.2f} s     overshoot  {m['overshoot']:.1f} %\n"
           f"settle 2%    {settle_box}     ss error   {m['ss']:.2f} mm")
    ax.text(0.02, 0.97, box, transform=ax.transAxes, va="top", ha="left",
            family="monospace", fontsize=10,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.95,
                      edgecolor="#999999"))

    for ext in ("png", "svg"):
        fig.savefig(os.path.join(outdir, f"step_response{suffix}.{ext}"),
                    dpi=160, bbox_inches="tight")
    plt.close(fig)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.join(os.path.dirname(here), "docs")
    builddir = os.path.join(os.path.dirname(here), "build")
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.join(outdir, "step_response.csv"))
    ap.add_argument("--info", default=os.path.join(builddir,
                                                   "step_run_info.txt"))
    ap.add_argument("--outdir", default=outdir)
    ap.add_argument("--metrics-dir", default=builddir)
    ap.add_argument("--suffix", default="")
    ap.add_argument("--note", default="")
    ap.add_argument("--window", type=float, default=8.0)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = read_rows(args.csv)
    info = read_info(args.info)
    m = metrics(rows, info)

    text = print_metrics(m, info)
    os.makedirs(args.outdir, exist_ok=True)
    os.makedirs(args.metrics_dir, exist_ok=True)
    metrics_path = os.path.join(args.metrics_dir, f"metrics{args.suffix}.txt")
    with open(metrics_path, "w") as fh:
        fh.write(text + "\n")

    plot_step(rows, info, m, args.outdir, plt, args.suffix, args.note,
              args.window)
    print(f"\nwrote {metrics_path}")
    print(f"wrote {os.path.join(args.outdir, f'step_response{args.suffix}.png')}")
    print(f"wrote {os.path.join(args.outdir, f'step_response{args.suffix}.svg')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
