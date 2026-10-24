# step response evidence

the lateral position loop is driven with a 40 mm reference step while the car
reverses at a constant speed. the controller is the real `Control::step` from
`control/control.h`, and the plant is the same servo and rear axle bicycle
model the parking simulator uses. the data is logged once per 30 Hz control
tick and the figures are generated from that CSV.

## why lateral position

the controlled variable is the rear axle lateral offset `x`, the state the
parking supervisor regulates. a heading step would exercise only the inner
loop: it would show the derivative action, the servo dead band and the heading
filter, but not the position integration the manoeuvre is judged on. a lateral
step puts the outer loop, the inner loop and the plant in the loop.

## reference hook

`Control::step` had no reference input. it now takes a defaulted argument:

```c++
float step(float x, float psi, float vSigned, float dt, float psiCap,
           float xRef = 0.0f) {
...
float psiRef = -sgn * (kx * (lpfX - xRef) + c->ki * integ);
...
integ += (lpfX - xRef) * dt;
```

with `xRef = 0` every parking call site keeps its meaning and the parking
behaviour is unchanged.

## plant and loop

`tools/step_rig.cpp` uses the servo and bicycle model from
`tools/sim_main.cpp`:

- servo: move only when `|cmd - wheel| > servo_deadband`, subtract the dead
  band from the step, and clamp the step to `servo_rate * dt`;
- kinematics: `yaw = -v * tan(wheel) / L_wb`, `psi += yaw * dt`,
  `x += v * sin(psi) * dt`, with the Ackermann yaw branch and the parallel
  scrub term;
- control period `Config::dt` (30 Hz), plant sub-stepped at 1 ms;
- every servo and plant value comes from `Config` and `car`: dead band, slew
  rate, steer model, scrub gain, speed, gains and steering limit.

## assumptions

- simulation only. nothing here is a hardware measurement.
- the servo values are the SG90 datasheet numbers already in `Config`:
  `servo_deadband = 0.0314 rad` (10 us through the 1 to 2 ms angle range) and
  `servo_rate = 10.472 rad/s` (0.10 s/60 degrees at 4.8 V).
- constant speed. the drive ramp is not modelled because `accel` is a
  simulator field rather than a `Config` value, so the car starts at
  `driveSpeed(cfg)`.
- no measurement noise by default. the seed is fixed at 1 and `--noise`
  adds Gaussian noise on `x` for a repeatable noisy run.
- the 40 mm step makes the steering command reach the 25 degree limit at the
  start, so the early rise is actuator limited.
- the nominal 0.15 m/s case cannot hold the 2 percent band: with the 1.8
  degree dead band the position dead zone is 2.35 mm, against a band of
  0.80 mm. the second figure shows that limit cycle.
- the run starts at `x = 0`, the parking target, and steps away from it.
- the integral gain is zero, so there is no integral action to trim the offset
  the dead band leaves. the measured residual is 1.61 percent at 0.08 m/s and
  0.79 percent at 0.15 m/s.

## results

| metric | 0.08 m/s primary | 0.15 m/s nominal |
|---|---|---|
| rise time 10 to 90 % | 2.64 s | 2.11 s |
| overshoot | 0.00 % | 5.30 % |
| settling to 2 % band | 5.47 s | not settled |
| steady-state error, last 1 s | -0.65 mm (-1.61 %) | -0.31 mm (-0.79 %) |
| final value | 39.36 mm | 39.01 mm |
| peak steering command | 24.98 deg | 24.98 deg |
| peak heading | 12.68 deg | 7.97 deg |

the primary run settles inside the 0.80 mm band with no overshoot. the
residual 0.65 mm comes from the servo dead band: the final heading trim is
below the dead band, so the wheel stops a fraction short.

the nominal run rises to about 38.5 mm by 4 s, dips, climbs to 42.1 mm
around 16 s and turns back. the band is crossed repeatedly and the position
sweeps roughly 37.7 to 42.1 mm.

## files

```
docs/step_response.csv         primary time series, 0.08 m/s
docs/nominal_step_response.csv nominal time series, 0.15 m/s
docs/step_response.png         primary step response
docs/step_response.svg
docs/step_response_015.png     nominal step response
docs/step_response_015.svg
```

`build/step_run_info.txt` and `build/nominal_run_info.txt` record the
parameters used for each run. `build/metrics.txt` and
`build/metrics_015.txt` are written by the plot script with the printed
metrics.

## reproduce

```bash
mkdir -p build docs
g++ -std=c++17 -O2 -Wall -Wextra -Icontrol -o build/step_rig tools/step_rig.cpp -lm
./build/step_rig --speed 0.08 --ref 0.04 --duration 15
.venv-plot/bin/python tools/step_plot.py

./build/step_rig --speed 0.15 --ref 0.04 --duration 20 \
    --out docs/nominal_step_response.csv \
    --servo-out build/nominal_servo.csv \
    --info-out build/nominal_run_info.txt
.venv-plot/bin/python tools/step_plot.py \
    --csv docs/nominal_step_response.csv \
    --info build/nominal_run_info.txt \
    --suffix _015 --note "nominal speed" --window 19
```

the plot environment is throwaway and is not the repo venv:

```bash
python3 -m venv .venv-plot
.venv-plot/bin/pip install -r tools/plot-requirements.txt
```

the rig is deterministic: two runs of the same command produce a
byte-identical CSV.
