# Self-parking-car

A small RC car reverses into a perpendicular parking bay, steered by a cascade
PID, and stops before the rear wall. The two side sensors measure the lateral
offset and the heading, the rear sensor sets the stop, and a dashboard on the
ESP32 shows the pose and the steering command while the loop runs.

The branch is `simple`. One loop at 25 ms, one controller, no simulator and no
HAL. The control law is in `src/control.h`, the gains and the limits are in
`src/config.h`.

## The hardware

| part | spec | notes |
|---|---|---|
| MCU | ESP32-S3 Super Mini | 4MB flash, 240MHz dual core, native USB-C, module soldered to the board |
| ToF sensors | 3x VL53L0X | must be VL53L0X, not VL53L1X. they are not drop-in compatible and use a different library. two face straight out sideways, one faces back |
| steering servo | SG90 or MG996R | steered by the cascade during the reverse, held at centre otherwise. the centre, the direction and the linkage are still unmeasured |
| motor driver | DRV8833 module | driven at 20 kHz, with a real brake on both inputs high |
| drive motor | stock Tamiya yellow motor, 6V rated | |
| chassis | Tamiya RC chassis with servo bay | the stock knuckle is turned by the drive motor through a plastic arm, so it cannot take a commanded angle and the chassis is converted to servo steering |
| power | one 5 V rail for the whole car | the ESP32, the DRV8833 VM pin and the servo share it. The stock 6 V motor sees about 4.5 V at the driver's dropout, so the duty has to be high enough for it to break away from rest. The DRV8833 is rated to 10.8 V, so VM must never be fed from a 3S pack directly |

The three sensors are addressed over one I2C bus. Each one is enabled in turn
through its XSHUT line and given an address: `0x29` left, `0x2A` right, `0x2B`
rear. All three power up at `0x29`, so they are held in reset together and
released one at a time. The rear sensor comes up first, then the right, and the
left comes up last because it keeps `0x29`. A part that fails its init is put
back into reset before the next one is released, so a dead part cannot answer
for the next one.

### Pin map

| GPIO | function |
|---|---|
| 1 | VL53L0X left XSHUT |
| 2 | VL53L0X right XSHUT |
| 4 | DRV8833 IN1 (left motor) |
| 5 | DRV8833 IN2 (left motor) |
| 6 | DRV8833 IN3 (right motor) |
| 7 | DRV8833 IN4 (right motor) |
| 8 | I2C SDA |
| 9 | I2C SCL |
| 10 | VL53L0X rear XSHUT |
| 11 | steering servo signal, 50Hz, 544 to 2400 us |

The table is the harness as it stands. Every pin in it, the two inversion flags
and the three sensor addresses live in `src/config.h`.

The measured chassis and sensor stations are also in `src/config.h`: `L_wb`
0.183 m, `t_front` 0.0905 m, `L_car` 0.234 m, `d_L` 0.067 m, `d_R` -0.147 m
and a face separation of 0.095 m. Nothing here computes a heading, so they are kept
as measurements rather than used.

## The control law

The two side faces sit at different stations along the car, so the pair
separates lateral offset from heading once the bay width is known. Each gap is
the distance from that face to the bay wall.

```
gL = W/2 + x + d_L*psi
gR = W/2 - x + d_R*psi
```

Add them and the offset cancels, so the sum is a heading signal.

```
gL + gR = W + (d_L + d_R)*psi
psi = (gL + gR - S0) / (d_L + d_R)
```

Subtract them and the width cancels, so the difference is the offset plus a
heading term that comes out once the heading is known.

```
gL - gR = 2x + (d_L - d_R)*psi
x = 0.5*(gL - gR) - 0.5*(d_L - d_R)*psi
```

With `d_L = 0.067`, `d_R = -0.147`, `d_L + d_R = -0.080` and
`d_L - d_R = 0.214`. `S0` is the bay width, measured at the start of every run
with the motors off and the wheels centred, averaged over 32 valid pairs. It is
the bay width only if the car starts square. A car that starts yawed carries
that yaw as a constant bias in every `psi`, which shows as a `psi` that does not
sit at zero while the car stands straight.

The offset is not referenced to the start. At `psi = 0` the difference is
already absolute, so subtracting the starting difference would make the loop
hold whatever offset the car entered with instead of centring it. The starting
difference is printed as `x0` and is a diagnostic only.

The controller is a cascade, because reversing steering is unstable and the
heading loop has to be the fast one.

| loop | measures | output | gains |
|---|---|---|---|
| outer, offset | `x` in metres | heading reference `psiRef` in radians | `Kx` 4.0 rad/m, `Ki` 0 |
| inner, heading | `psi` in radians | road wheel angle `delta` in radians | `Kp` 1.0, `Kd` 0.15 s |

```
psiRef = +Kx*x + Ki*integral(x)              clamped to +/-0.17 rad
delta  =  Kp*(psiRef - psi) - Kd*psiDot      clamped to +/-0.20 rad
servo  =  90 + 1*2.9*delta*57.2958           clamped to 45..135 degrees
```

The outer sign is the reverse-drive sign, and it is not cosmetic. The pose
convention is `x > 0` to the right and `psi > 0` nose right, so reverse drive
gives `xdot = -V*psi` and `psidot = (V/L)*delta`. With `delta = Kp*(psiRef-psi)`
and `psiRef = +Kx*x` the offset loop closes as

```
xddot + (V*Kp/L)*xdot + (V^2*Kp*Kx/L)*x = 0
```

Every coefficient is positive for `V > 0`, `Kx > 0` and `Kp > 0`, so it is
stable. The negative form belongs to forward drive, where the same algebra
gives determinant `-V^2*Kp*Kx/L`: a saddle, and any offset grows. A future
forward maneuver must negate this law rather than copy it.

The scale is the justification for the gains. 20 mm of offset asks for 4.6
degrees of heading, and 4.6 degrees of heading asks for 4.6 degrees of road
wheel, which the 2.9 linkage turns into 13.3 servo degrees. That is the size of
correction `main` drove on this car. The gains are untuned guesses, not a
measured response.

`psi` passes through a first-order low pass with `tau = 0.05 s` before the
derivative, and `psiDot` is the difference of the filtered signal over the
measured `dt` from `millis()`, clamped to +/-3 rad/s. The integral freezes while
`psiRef` sits at its clamp and resets in `idle`.

Inside a deadband of 8 mm of offset and 1 degree of heading the servo is held at
90 and nothing is reset. That band is inside what the sensor and the linkage can
resolve, so a correction there is a correction on noise, and the loops resume
from where they were when the car drifts out.

Every cascade return, including the deadband hold and the `STEER_ENABLE` hold,
goes through a slew limit of 6 servo degrees per tick, 240 degrees per second at
25 ms. That is a rate limit, not a filter: it keeps the last command and nothing
else, so it adds no lag while the command tracks the loop. It exists to remove
the step at deadband entry and exit, where the target jumps between 90 and a
live angle inside one tick. A state change still writes centre directly, so that
step is untouched.

`STEER_ENABLE` in `src/config.h` is the one switch. Off, the loop holds the
wheels at centre and everything else runs, which is how to prove the drivetrain
without the steering.

`PER_WHEEL` is derived from the steering hardware, not guessed. The pinion is a
0.8 module, 12 tooth spur gear on the servo spline: one servo revolution moves
`pi*0.8*12 = 30.16` mm of rack, so 0.0838 mm per servo degree. The steering arm
is 14.00 mm from the kingpin bore to the tie-rod bore, so a servo degree turns
the wheel `atan(0.0838/14) = 0.343` degrees, and one wheel degree needs 2.92
servo degrees. `PER_WHEEL = 2.9` encodes that, and both clamps are proven to
stay off the servo limit at build time:

```
DELTA_MAX    = 0.20 rad = 11.5 wheel degrees = 33.2 servo degrees
command span = 90 +/- 33.2 = 56.8 .. 123.2 degrees, inside 45 .. 135
```

Two `static_assert`s in `src/config.h` hold that inequality, so a wider
`DELTA_MAX` or a different ratio fails the build instead of quietly flattening
the loop against the clamp. The ratio comes from the CAD dimensions. The centre
and the direction are still unproven on the car, so a wrong `DIR` makes both
loops positive feedback and the first powered run has to check which way the
wheels go.

## The platform

`platformio.ini` pins a pioarduino espressif32 release by URL rather than by a
registry version. The registry `espressif32` platform ships Arduino core 2.x,
which is not the core this car was brought up on. The pinned release is
55.03.312, Arduino core 3.3.12.

The board definition is `esp32-s3-devkitc-1`, with three overrides for the
Super Mini: `qio_qspi` memory type, 4 MB of flash and the `max_app_4MB.csv`
partition table. `-DBOARD_HAS_PSRAM` puts the 2 MB of in-package PSRAM on the
heap, and the two USB flags put `Serial` on the USB CDC port at boot.
`-DCONFIG_ASYNC_TCP_RUNNING_CORE=0` keeps the web server's task on core 0, away
from the loop.

Two things are not negotiable. Do not write into `~/.platformio/packages/`, and
do not remove `lib/Network/` or its `library.json`. Arduino core 3.3.12 ships
`Network` without a PlatformIO manifest, so the vendored copy is what makes the
library resolve.

## Instructions

### 1. The toolchain

```bash
python3 -m venv --without-pip .venv
curl -fsSL https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
.venv/bin/python3 /tmp/get-pip.py
.venv/bin/pio run
```

The first build downloads the pinned platform and the toolchain, about 4GB and a few
minutes. After that a build takes seconds. Each machine needs its own
`~/.platformio`, because it lives under `$HOME`.

### 2. The firmware

```bash
.venv/bin/pio run            # build
.venv/bin/pio run -t upload  # flash it, with the board connected
```

The application environment is `esp32-s3-devkitc-1`. A build writes
`.pio/build/esp32-s3-devkitc-1/firmware.factory.bin`, which is the bootloader,
the partition table, the boot app and the application merged into one image.
The tagged copy of that file is `releases/autoparking.factory.bin`.

#### Flash a prebuilt binary

Put the board in download mode: hold **BOOT**, tap **RESET**, release
**BOOT**. It then enumerates as `/dev/ttyACM0` on Linux, `/dev/cu.usbmodem*` on
macOS or `COMx` on Windows.

```bash
~/.platformio/penv/bin/esptool.py --chip esp32s3 --port /dev/ttyACM0 \
    --baud 460800 write_flash 0x0 autoparking.factory.bin
```

The factory image goes at `0x0` and nothing else is needed. Do not flash
`firmware.bin` at `0x0`: that is the application alone and belongs at `0x10000`.

#### The bench project

`bench/` is a separate PlatformIO project, so it does not touch the parking
firmware or its environment. It links no libraries and takes the pin map and the
inversion flags from `src/config.h`.

```bash
cd bench
../.venv/bin/pio run -e motor_test -e motor_reverse -e i2c_scan -e servo_sweep
```

`env:motor_test` drives both motors forward at duty 200 from `setup()` and keeps
driving until the power is cut. It is what to flash when the only question is
whether the drive works. If the car drives away backwards, the inversion flag for
that side is wrong.

`env:motor_reverse` is the same test with the direction flipped, and it uses the
firmware's own `Drive` class, so the PWM frequency, duty and inversion are the
ones the parking firmware sends. It drives both motors in reverse from `setup()`
and keeps driving until the power is cut. The car moves backwards the moment it
is powered: put the wheels off the ground or leave the car clear behind it. If
it drives forward instead, the inversion flags or the channel mapping are wrong.

`env:i2c_scan` drives the three XSHUT lines high, prints the idle voltage of
GPIO 8 and 9 on the ADC, then scans with SDA on GPIO 8 and SCL on GPIO 9, closes
the bus, prints the levels again, and rescans with the two swapped.

`env:servo_sweep` steps the servo through 45 to 135 degrees in 5 degree steps,
one second apart, and prints each angle with the pulse it sends.

## What the firmware does

One loop at `delay(25)`. The states are `idle`, `check`, `calibrate`,
`reverse`, `brake`, `parked` and `fault`.

| state | what it does |
|---|---|
| `idle` | brakes, wheels centred, resets the integral, accepts a start |
| `check` | takes five frames, medians each channel, and refuses the run unless the rear is fresh and 30 to 800 mm and both sides are fresh and at least 25 mm |
| `calibrate` | motors off, wheels centred, averages 32 valid side pairs into `S0` and prints `x0` |
| `reverse` | brakes released, both motors reverse at duty 200, cascade steers, stops on the raw rear frame at or under 90 mm |
| `brake` | both motor inputs high for 300 ms, so the car is held |
| `parked` | brake held, wheels centred, accepts a start |
| `fault` | brake held, latched until power cycle |

`check` runs once at boot and again on every start. A failed check names the
reason, `fault=rear` or `fault=side`, and latches. `calibrate` gives up after
3000 ms without 32 valid pairs and latches `fault=side`, so a dead side pair
cannot hold the run there.

`reverse` ends four ways:

- a fresh raw rear frame at or under 90 mm, which brakes and parks;
- a fresh side reading under 25 mm, which brakes and faults;
- two consecutive rear frames that are neither 8190 nor 65535, which brakes
  and faults as `rear`;
- 3000 ms without one of the above, which brakes and faults as `timeout`.

The stop test runs on the raw rear frame, not on the filtered value. The filter
trails the car by two or three samples and three blocking reads make a loop
about 100 ms, so at duty 200 the filtered number is still near 90 mm when the
gap is already at the wall. The filtered value is kept as a secondary test.

`calibrate` hands over to `reverse` only on a rear frame that came back fresh in
that same loop pass. A run with nothing to stop on faults as `rear` before the
motors move.

The cap is a give-up, not a wall. A run that ran out of time shows
`fault=timeout` on the serial line and the page, so it cannot be mistaken for a
run that reached the rear limit.

A fresh rear reading of 0 to 29 mm is the part's minimum range, so it means the
face is at the wall. It counts as a stop, not as a distance of 29 mm.

`/start` is accepted only from `idle` and `parked`, and the page disables the
button whenever the firmware's `ready` flag is false. `ready` is true only in
those two states, so the button is dark for the whole run and in a fault.

The servo is written by the cascade during `reverse` and held at 90 everywhere
else. Every command is clamped to 45 to 135 degrees inside the controller.

## The boot lines

The board prints these before anything can move. The raw values are the codes
the parts answered with, 8190 for no target and 65535 for a timeout or a part
that never came up.

```
autoparking-car: cascade PID reverse, rear stop, side guard
pins i2c=8/9 xshut=1/2/10 servo=11 motor=4/5/6/7
drive pwm=20000Hz/8bit duty=200/200 servo=45..135 centre=90
times loop=25ms check=5 frames reverseMax=3000ms brake=300ms stop=90mm guard=25mm
control steer=1 Kx=4.00 Ki=0.00 Kp=1.00 Kd=0.15 psiRefMax=0.17 deltaMax=0.20 deadband=8mm/0.017rad
geometry dL=0.067 dR=-0.147 sum=-0.080 diff=0.214 perWheel=2.9 dir=1 tau=0.050s
part rear  xshut=10 addr=0x2B init=ok raw=123 mm
part right xshut= 2 addr=0x2A init=ok raw=456 mm
part left  xshut= 1 addr=0x29 init=ok raw=789 mm
verdict rear=visible sides=usable
```

A failed part prints `init=fail` and `raw=65535`. The verdict line names
whether the rear is visible and whether the sides are usable, so a flash cannot
leave you guessing which sensor is dead.

## Serial and the web page

The serial line prints at 10 Hz:

```
t=1234 state=reverse x=-6.2mm psi=0.0131rad psiRef=-0.0248rad servo=86 gL=210.4 gR=198.1 rear=340.0 raw=352 ready=0 fault=none
```

`rear` is the filtered value the cascade sees and `raw` is the frame the stop
test compares, so a log from a run says which of the two crossed 90 mm first.

The ESP32 joins the network in `src/wifi_config.h`. If it cannot join within
15 seconds it starts its own access point, `autoparking` with password
`autoparking`, so the page is reachable on a bench with no configured network.

| route | what it serves |
|---|---|
| `/` | `src/dashboard_html.h`, the page |
| `/start` | sets the start flag and returns `{"start":true}` |
| `/ws` | one JSON snapshot at 10 Hz |

The page shows state, `x` in millimetres, `psi` in degrees, the servo command
in degrees, the filtered gaps, the rear, ready and fault, and has a start button
that is disabled while `ready` is false. 8190 and 65535 show as `none`, and 0 to
29 shows as `wall`. Under the cells it plots x, psi, the rear and the servo
against time, four bands over the last 600 samples, which is one minute at the
10 Hz socket. The history is a ring in the page, so the plots cost the control
loop nothing, and 8190 and 65535 are drawn as gaps instead of as eight or
sixty-five metres. The rear band carries the 90 mm stop line. There is no CSV.
The JSON carries `x` in millimetres, `psi` and `psiRef` in milliradians and the
servo in degrees.

## The layout

| path | what is in it |
|---|---|
| `src/config.h` | pins, addresses, measured chassis and stations, servo range, sensor timing and thresholds |
| `src/sensors.h` | the three VL53L0X, independent bring-up, raw reads and the freshness rules |
| `src/drive.h` | the DRV8833 on four pins at 20 kHz, forward, reverse and brake |
| `src/control.h` | the pose from the two gaps and the cascade, with the derivation |
| `src/main.cpp` | the one loop, the states and the web routes |
| `src/dashboard_html.h` | the page, as one PROGMEM string |
| `src/wifi_config.h` | the SSID, the password and the fallback access point |
| `bench/` | the separate bench project: forward and reverse motor tests, I2C scan, servo sweep |
| `releases/` | the factory image published by a tag |
| `lib/Network/` | the vendored `Network` source, kept because the platform expects it |

## Status

`hw-2026-11-03.2` was flashed and driven by the owner. The car reversed at duty
200 and stopped at the wall rather than at 90 mm, which is the defect this image
fixes, and the page showed the rear sensor's numbers live through the run. This
image, `hw-2026-11-03.3`, is build-verified only and has not been flashed.

The firmware builds and links for the ESP32-S3 Super Mini at 48,220 bytes of RAM
(14.7 percent of 327,680) and 1,004,482 bytes of flash. The application partition
is the 4,063,232-byte `app0` from `max_app_4MB.csv`, so the app uses 24.7
percent of it. PlatformIO prints 12.0 percent because it divides by the board's
8 MB `maximum_size`, which is not the figure to quote. The
`releases/autoparking.factory.bin` built from this tree is 1,086,144 bytes, sha256
`0813af8b8444969b654b0bda02fee52b3226457c60254321cefcd963089935b3`.

The four bench environments build. Duty 200 drives this car in reverse as well
as forward, which the `hw-2026-11-03.2` run showed. The steering linkage, the
servo centre and the 90 mm stopping distance are still unmeasured, and no run
has completed.
