#!/usr/bin/env python3
"""Parameter sweep for the reverse-parking controller.

Drives the host simulator across a grid of assumed conditions and writes a
CSV. The point is to explore the envelope, not to certify anything: the plant
is a guess until the car is measured.

    python3 tools/sweep.py --out build/sweep.csv
    python3 tools/sweep.py --quick

No plotting: this host has no matplotlib and no pip, so the summary is text.
"""

import argparse
import csv
import io
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SIM = os.path.join(ROOT, "build", "sim")

COLUMNS = [
    "x0", "psi0", "W", "D", "v", "noise", "seed", "finalX", "finalPsi",
    "finalRear", "minClear", "minRear", "peakPsi", "strokes", "centred",
    "aligned", "rearOk", "hitWall", "gaveUp", "timedOut",
]


def run_batch(jobs):
    """Run the simulator once per job and return the parsed rows."""
    rows = []
    for job in jobs:
        cmd = [SIM, "--csv"]
        for key, value in job.items():
            cmd += ["--" + key, str(value)]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            print(f"sim failed: {' '.join(cmd)}\n{proc.stderr}", file=sys.stderr)
            continue
        lines = proc.stdout.strip().splitlines()
        if len(lines) < 2:
            continue
        reader = csv.DictReader(io.StringIO("\n".join(lines)))
        for row in reader:
            rows.append(row)
    return rows


def summarise(rows, label):
    n = len(rows)
    if n == 0:
        print(f"{label}: no rows")
        return
    ok = [r for r in rows if r["centred"] == "1" and r["aligned"] == "1"
          and r["rearOk"] == "1" and r["hitWall"] == "0" and r["gaveUp"] == "0"]
    strokes = sorted(int(r["strokes"]) for r in rows)
    clearance = sorted(float(r["minClear"]) * 1000 for r in rows)
    peak = sorted(float(r["peakPsi"]) for r in rows)
    print(f"{label}: {len(ok)}/{n} fully successful "
          f"({100.0 * len(ok) / n:.0f}%)")
    print(f"   strokes   min/med/max: {strokes[0]}/"
          f"{strokes[len(strokes) // 2]}/{strokes[-1]}")
    print(f"   clearance min/med: {clearance[0]:.0f}/"
          f"{clearance[len(clearance) // 2]:.0f} mm")
    print(f"   peak yaw  max: {peak[-1]:.1f} deg")
    for key in ("centred", "aligned", "rearOk"):
        bad = sum(1 for r in rows if r[key] != "1")
        print(f"   not {key}: {bad}")
    for key in ("hitWall", "gaveUp", "timedOut"):
        bad = sum(1 for r in rows if r[key] == "1")
        print(f"   {key}: {bad}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "build", "sweep.csv"))
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--seeds", type=int, default=1)
    args = ap.parse_args()

    if not os.path.exists(SIM):
        print(f"missing {SIM}; build it first:\n"
              f"  g++ -std=c++17 -O2 -Icontrol -o build/sim "
              f"tools/sim_main.cpp -lm", file=sys.stderr)
        return 1

    xs = [0.02, 0.04, 0.06] if not args.quick else [0.02, 0.04]
    psis = [0, 5, -5, 10] if not args.quick else [0, 5]
    widths = [0.40, 0.50, 0.60] if not args.quick else [0.50]
    speeds = [0.10, 0.15, 0.20] if not args.quick else [0.15]
    noises = [0.003, 0.006] if not args.quick else [0.003]

    jobs = []
    for x0 in xs:
        for psi0 in psis:
            for W in widths:
                for v in speeds:
                    for noise in noises:
                        for seed in range(1, args.seeds + 1):
                            jobs.append({
                                "x": x0, "psi": psi0, "W": W, "D": 0.60,
                                "v": v, "noise": noise, "seed": seed,
                            })

    print(f"running {len(jobs)} scenarios")
    rows = run_batch(jobs)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in COLUMNS})
    print(f"wrote {args.out}")

    summarise(rows, "overall")

    # Break the results down by the variable that the plan is unsure about.
    for key, label in (("x0", "start offset"), ("psi0", "start heading"),
                       ("W", "bay width"), ("v", "speed"), ("noise", "noise")):
        groups = {}
        for row in rows:
            groups.setdefault(row[key], []).append(row)
        print()
        for value in sorted(groups, key=float):
            summarise(groups[value], f"{label} = {value}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
