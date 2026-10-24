# Self-parking-car

A small RC car reverses into a perpendicular parking bay, lines itself up with the bay walls and stops before the rear obstacle. The steering is closed loop, the step response is logged by a host rig, and a web dashboard shows the live state and a graph.

The car uses VL53L0X time-of-flight sensors for autonomous reverse parking. The steering is described as Ackermann, and the control code runs either kind of steering until someone measures the chassis.

## The hardware

| part | spec | notes |
|---|---|---|
| MCU | ESP32-S3-DevkitC-1 N16R8 | 16MB flash, 8MB octal PSRAM, dual core at 240MHz |
| ToF sensors | 3x VL53L0X | must be VL53L0X, not VL53L1X. they are not drop-in compatible and use a different library. two face out sideways, one faces back |
| steering servo | SG90 or MG996R | the MG996R has less backlash, which matters here |
| motor driver | DRV8833 module | a TB6612FNG also works and gives a clean sleep between runs |
| drive motor | stock Tamiya yellow motor, 6V rated | |
| chassis | Tamiya RC chassis with servo bay | the stock knuckle is turned by the drive motor through a plastic arm, so it cannot take a commanded angle and the chassis is converted to servo steering |
| power | 2S LiPo plus buck converter | the motor rail needs 6V regulation |

The three sensors are addressed over one I2C bus. Each one is enabled in turn through its XSHUT line and given an address: `0x29` left, `0x2A` right, `0x2B` rear.

### Pin map

| GPIO | function |
|---|---|
| 8 | I2C SDA |
| 9 | I2C SCL |
| 15 | VL53L0X left XSHUT |
| 16 | VL53L0X right XSHUT |
| 10 | VL53L0X rear XSHUT |
| 17 | steering servo signal, 50Hz, 1000 to 2000µs |
| 18 | DRV8833 IN1 |
| 21 | DRV8833 IN2 |

Leave GPIO 26 to 32 alone: the octal PSRAM claims them. GPIO 19 and 20 are the USB pins, and GPIO 0, 3, 45 and 46 are strapping pins that are sampled at boot.

## Instructions

### 1. The toolchain

```bash
python3 -m venv --without-pip .venv
curl -fsSL https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
.venv/bin/python3 /tmp/get-pip.py
.venv/bin/pio run
```

The first build pulls the pinned platform and the toolchain, about 4GB and a few minutes. After that a build takes seconds. Each machine needs its own `~/.platformio`, because it lives under `$HOME`.

If `pio` stops before it compiles anything with

```
Error: Failed to install Python dependencies into penv
```

Then PlatformIO's own environment needs pip:

```bash
curl -fsSL https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
~/.platformio/penv/bin/python3 /tmp/get-pip.py
```

### 2. The firmware

```bash
.venv/bin/pio run            # build
.venv/bin/pio run -t upload  # flash it, with the board connected
```

`lib/Network/src` is committed source with a hand-written `library.json` beside it, because Arduino core 3.3.12 ships `Network` without one and PlatformIO will not link it. Leave it as it is: a symlink into `~/.platformio/packages/` resolves on the machine that made it and nowhere else.

### 3. The network and the dashboard

The firmware joins the network named in `src/wifi_config.h`: put the SSID and the password there, or leave the placeholders and let it fall back. The join is bounded at 15 s, after which the car brings up its own access point:

| | |
|---|---|
| SSID | `autoparking` |
| password | `autoparking` |
| page | `http://192.168.4.1/` |

The page shows the live state and a chart of the last 300 samples, and the button asks the control task to start a run. It refuses before the calibration and ignores the request while a run is in progress. Telemetry goes to the page over a WebSocket at `/ws`, about ten messages a second.

### 4. The host simulator

No board needed. The control core has no hardware calls in it, so the same code runs on the host.

```bash
mkdir -p build
g++ -std=c++17 -O2 -Wall -Wextra -Icontrol -o build/sim tools/sim_main.cpp -lm
g++ -std=c++17 -O2 -Wall -Wextra -Icontrol -o build/geom_checks tools/geom_checks.cpp -lm

./build/geom_checks                             # the sensor geometry
./build/sim --x 0.04 --psi 5 --trace            # one scenario, traced
./build/sim --x 0.04 --psi 5 --steer ackermann  # the other steering model
python3 tools/sweep.py --out build/sweep.csv    # the parameter sweep
```

## The step response

The controlled variable is the rear axle lateral offset `x`, the state the parking supervisor regulates. `tools/step_rig.cpp` steps the reference from 0 to 40 mm while the car reverses at a constant speed, through the real `Control::step` and the same servo and bicycle model the simulator uses. One sample per 30 Hz control tick, and the figures come from that CSV.

![lateral step response at 0.08 m/s](docs/step_response.png)

![lateral step response at the nominal 0.15 m/s](docs/step_response_015.png)

The 0.08 m/s run is the settling case. The 2 percent band is 0.80 mm around the 40 mm reference. At the nominal 0.15 m/s the dead zone from the 1.8 degree servo dead band is 2.35 mm, wider than that band, so the position limit-cycles between about 37.7 and 42.1 mm and does not settle.

| metric | 0.08 m/s | 0.15 m/s |
|---|---|---|
| rise time, 10 to 90 % | 2.64 s | 2.11 s |
| overshoot | 0.00 % | 5.30 % |
| settling, 2 % band | 5.47 s | not settled |
| steady-state error, last 1 s | -0.65 mm (-1.61 %) | -0.31 mm (-0.79 %) |
| peak steering command | 24.98 deg | 24.98 deg |
| peak heading | 12.68 deg | 7.97 deg |

What the numbers assume:

- Simulation, not hardware. Nothing here is a measurement.
- The rig measures `x` directly. It has no ToF geometry, cone, range failures, sensor latency, walls or clearance cap, and no parking supervisor in the loop.
- The servo numbers are the SG90 datasheet figures already in `Config`: 600 deg/s at no load (0.10 s per 60 degrees at 4.8 V), and a 1.8 degree dead band mapped through the 1000 to 2000 us position table.
- The servo model is dead band plus slew only: no load, no backlash, no compliance, no position feedback.
- The figures use the default parallel steering and the 25 degree mechanical limit. Ackermann would clamp to `deltaEqMax`, about 20 degrees.
- Constant speed. The drive ramp is not modelled, because `accel` is a simulator field and not a `Config` value, so the car starts at the configured speed.
- The speed is commanded constant, but the parallel scrub term bleeds it over the run, -0.080 to -0.07795 m/s and -0.150 to -0.14862 m/s. The controller is fed the nominal speed.
- No measurement noise. The seed is fixed at 1, and `--noise` adds Gaussian noise on `x` for a repeatable noisy run.
- 30 Hz control, with the plant substepped at 1 ms.
- The integral gain is zero (`ki = 0`), so there is no integral action to trim the offset the dead band leaves. The measured residual is 1.61 percent at 0.08 m/s and 0.79 percent at 0.15 m/s.
- The run starts at `x = 0`, the parking target, in free space with psi and delta at zero, and the step lands at 1.0 s.

Reproduce both runs and the plots:

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

The plot script needs matplotlib in a throwaway venv. This is not the repo venv, and `.venv-plot/` is ignored:

```bash
python3 -m venv .venv-plot
.venv-plot/bin/pip install -r tools/plot-requirements.txt
```

The SVG versions of the figures sit beside the PNGs. The longer note, with the plant, the reference hook and the assumptions, is [docs/step-response.md](docs/step-response.md).

## The dashboard and the web server

| file | what is in it |
|---|---|
| `src/main.cpp` | the runtime: the two FreeRTOS tasks and the handoff between them |
| `src/esp32_hal.h` | the ESP32 `Hal`: three VL53L0X on one bus, the servo and the DRV8833 |
| `src/dashboard_html.h` | the page, as one PROGMEM string |
| `src/wifi_config.h` | the SSID, the password, the fallback access point and the hostname |

The core map:

| what runs | core | where it is set |
|---|---|---|
| control task | 1 | `xTaskCreatePinnedToCore`, priority 5, 8 KB, 33 ms period |
| web task | 0 | `xTaskCreatePinnedToCore`, priority 2, 8 KB, 100 ms period |
| AsyncTCP task | 0 | `CONFIG_ASYNC_TCP_RUNNING_CORE=0` in `platformio.ini` |
| WiFi and lwIP tasks | 0 | the platform's sdkconfig: `CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0` and `CONFIG_LWIP_TCPIP_TASK_AFFINITY=0x0` |

The endpoints:

| route | method | what it does |
|---|---|---|
| `/` | GET | the page |
| `/ws` | WebSocket | telemetry, one JSON object per message |
| `/start` | GET | sets the start request |

The page is reachable on the configured network, or over the `autoparking` access point at `192.168.4.1`. The SSID, the password and the fallback are in `src/wifi_config.h`.

Telemetry crosses from the control task to the web task through a one-deep queue written with `xQueueOverwrite`, so a slow socket can never hold up the control loop: the web task either gets the newest tick or nothing at all. The JSON carries the numeric fields as integers, t in ms, distances in mm, angles in milliradians and the servo in whole degrees. Phase and stroke are strings. The web task drains the queue every 100 ms and sends, which is 10 Hz. The start request crosses the other way as a `std::atomic<bool>`, set by `/start` and cleared by the control task, which still refuses it before the calibration and ignores it mid-run.

This is compile-verified, not hardware-verified. The firmware builds and links for the target, but no board has run it, so no sensor has been read, the servo has never moved, the network join is untested and the page has not been opened on a phone.

## How the maneuver runs

The supervisor owns the maneuver and the inner loop owns the steering. They never fight over the servo: during a correction stroke the supervisor has it, and otherwise the inner loop does.

```mermaid
%%{init: {'flowchart': {'curve': 'linear'}}}%%
flowchart TD
    start([Power on]) --> cal["Park the car centred and squared up<br/>take 32 readings for S0 and the trim"]
    cal --> backin["BACK_IN<br/>reverse into the bay<br/>inner loop holds the heading, servo follows"]
    backin -->|"rear limit reached"| done["DONE<br/>motor off, brake on, steering centred<br/>the log says whether it landed in tolerance"]
    backin -->|"out of tolerance for 3 ticks"| correct{"CORRECT<br/>heading out of tolerance?"}
    backin -->|"rear reading unusable"| giveup["GIVEUP<br/>brake and flag it"]
    correct -->|yes| rotate["Rotate stroke<br/>hold one lock until the heading moves"]
    correct -->|no| translate["Translate stroke<br/>S-curve: one lock, then the opposite"]
    rotate -->|"stroke ends"| settle
    translate -->|"stroke ends"| settle
    settle["Settle<br/>brake, centre the wheels, wait for stillness"] --> correct
    correct -->|"in tolerance for 3 ticks"| backin
    correct -->|"stroke budget spent"| giveup
```

The heading is corrected before the offset, because a standing heading error makes the offset grow. A translate stroke is an S-curve rather than one held angle: the servo takes one lock for the first half and the opposite lock for the second, so the heading excursion cancels itself and the car ends up moved sideways with its heading where it started. The stroke stops on the measured excursion rather than on a timer, which is what makes it independent of the speed and of the battery.

A stroke that loses the side readings stops, because the car may be against a wall and holding the lock makes that worse. The rear sensor gets the same treatment: an unusable reading stops the car unless the wall is simply out of the sensor's range, which the distance reversed since the last usable reading tells apart.

## How the sensing and steering fit together

The state is referred to the rear axle centre, which is where the instant centre of rotation lives on any car with a fixed rear axle. That is what lets one equivalent steer angle describe the whole vehicle.

```mermaid
%%{init: {'flowchart': {'curve': 'stepBefore'}}}%%
flowchart LR
    ref["reference<br/>centred and square"]
    sum(("+<br/>-"))
    ctl["Controller<br/>control.h, delta_eq out"]
    steer["Steering<br/>ackermann.h, the two front wheel angles<br/>and the servo"]
    plant["Plant<br/>car body, bicycle model"]
    sense["Sensing<br/>3x VL53L0X, then geometry.h<br/>gaps to a pose"]
    pose["x lateral offset<br/>psi heading"]
    sup["Supervisor<br/>phase and correction strokes"]
    sw1["steer_model<br/>parallel or Ackermann"]
    sw2["motor_driver<br/>DRV8833 or LM298"]

    ref --> sum
    sum --> ctl
    ctl --> steer
    steer --> plant
    plant --> sense
    sense --> pose
    pose -->|"feedback"| sum

    pose -.-> sup
    sup -.->|"stroke commands"| steer
    sw1 -.-> steer
    sw2 -.-> plant
```

The solid arrows are the loop. The reference and the measured pose meet at the summing point, so the controller works on the difference. That one equivalent steer angle goes through the Ackermann split and the car, and the sensing chain measures where the car ended up. The dotted arrows carry everything outside the loop. The supervisor takes the servo during a correction stroke, and the two switches say which hardware and which steering geometry the blocks stand for.

The controller works in one angle, `delta_eq`. The bicycle model in the simulator is exact for it rather than an approximation, because the rear wheels are fixed and the whole vehicle state is already referenced to the rear axle centre.

Two things are switches rather than facts, because neither has been measured yet. The steering model decides how much lock the car may be given: with Ackermann steering the inner wheel reaches its stop before the equivalent angle does, and the equivalent angle is held below that. The motor driver decides the speed and the stopping distance: the DRV8833 and the LM298 have the same truth table, so the firmware does not care which one is fitted, but the LM298 drops several volts across itself and would run the car at about 71 percent of the speed.

## The layout

| path | what is in it |
|---|---|
| `control/` | the control core: geometry, the steering loop, the supervisor and the stroke model. no hardware calls, only the `Hal` interface |
| `control/ackermann.h` | the Ackermann relations, splitting the equivalent angle into the two front wheel angles |
| `tools/sim_main.cpp` | the host plant, the sensor forward model, the scenarios and the fault injection |
| `tools/step_rig.cpp` | the lateral step-response rig: a 40 mm reference step through the real control law |
| `tools/step_plot.py` | computes the step metrics, writes the step figures |
| `tools/sweep.py` | the parameter sweep, writes a CSV and prints a summary |
| `tools/geom_checks.cpp` | direct checks of the sensor geometry, including the clearance interlock |
| `docs/` | the step-response note, both time series and the PNG and SVG figures |
| `src/main.cpp` | the firmware runtime: the control task on core 1, the web task on core 0, and the handoff between them |
| `src/esp32_hal.h` | the ESP32 `Hal`: three VL53L0X on one bus, the servo and the DRV8833 |
| `src/dashboard_html.h` | the dashboard, as one PROGMEM string |
| `src/wifi_config.h` | the SSID, the password and the fallback access point, in one place |
| `src/control_build.cpp` | builds the control core with no hardware attached, as a compile check |
| `lib/Network/` | the committed `Network` source, with the manifest that makes PlatformIO build it |

The control core has no hardware calls in it, so the same code runs against the host simulator and against the ESP32 through `src/esp32_hal.h`. Both are built: the simulator on the host, the firmware for the target.

## Status

The firmware builds and links for the N16R8 at RAM 14.7 percent and Flash 15.6 percent of the 6,553,600 byte app partition (1,024,170 bytes). The control core, the HAL adapter and the dashboard are written and compile for the target, and the host simulator builds with them. The board has never been flashed: no sensor reading, servo position or WiFi join has been verified on real hardware. The maneuver itself remains unproven until the car runs, and the step response above is simulated.
