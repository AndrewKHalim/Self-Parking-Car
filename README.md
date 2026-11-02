# Self-parking-car

A small RC car reverses into a perpendicular parking bay, centres itself
between the two side walls and stops before the rear wall. Three VL53L0X
time-of-flight sensors feed one loop, one PID on the lateral error and one
small state machine. A dashboard on the ESP32 shows the live reading and has a
start button.

The branch is `simple`. It is `main`'s maneuver and `main`'s PID with this
car's pins and sensors, plus the dashboard. There is no pose, no calibration,
no control core, no HAL and no host simulator.

## The hardware

| part | spec | notes |
|---|---|---|
| MCU | ESP32-S3 Super Mini | 4MB flash, 240MHz dual core, native USB-C, module soldered to the board |
| ToF sensors | 3x VL53L0X | must be VL53L0X, not VL53L1X. they are not drop-in compatible and use a different library. two face straight out sideways, one faces back |
| steering servo | SG90 or MG996R | the MG996R has less backlash, which matters here |
| motor driver | DRV8833 module | a TB6612FNG also works and gives a clean sleep between runs |
| drive motor | stock Tamiya yellow motor, 6V rated | |
| chassis | Tamiya RC chassis with servo bay | the stock knuckle is turned by the drive motor through a plastic arm, so it cannot take a commanded angle and the chassis is converted to servo steering |
| power | one 5 V rail for the whole car | the ESP32, the DRV8833 VM pin and the servo share it. The stock 6 V motor sees about 4.5 V at the driver's dropout, so the duty has to be high enough for it to break away from rest. The DRV8833 is rated to 10.8 V, so VM must never be fed from a 3S pack directly |

The three sensors are addressed over one I2C bus. Each one is enabled in turn
through its XSHUT line and given an address: `0x29` left, `0x2A` right, `0x2B`
rear. All three power up at `0x29`, so they are held in reset together and
released one at a time. The rear sensor comes up first, then the right, and the
left comes up last because it keeps `0x29`.

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
and the three sensor addresses live in `include/ParkingConfig.h` under
`namespace board`, so a measured change touches one file.

Leave GPIO 26 to 32 alone: the in-package flash claims them. GPIO 19 and 20 are
the USB pins, GPIO 43 and 44 are UART0, and GPIO 0, 3, 45 and 46 are strapping
pins that are sampled at boot. GPIO 9, 10 and 11 are FSPI IO MUX pins, safe here
only because nothing on this board drives SPI2.

The card also records the measured chassis and sensor stations in `namespace
car`: `L_wb` 0.183 m, `t_front` 0.0905 m, `L_car` 0.234 m, `d_L` 0.067 m and
`d_R` -0.147 m. Nothing in this firmware derives a pose, so they are kept as
measurements rather than used.

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
library resolve. It is committed source, not a symlink, because an absolute
symlink into one machine's package tree does not resolve anywhere else.

## Instructions

### 1. The toolchain

```bash
python3 -m venv --without-pip .venv
curl -fsSL https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
.venv/bin/python3 /tmp/get-pip.py
.venv/bin/pio run
```

The first build pulls the pinned platform and the toolchain, about 4GB and a few
minutes. After that a build takes seconds. Each machine needs its own
`~/.platformio`, because it lives under `$HOME`.

If `pio` stops before it compiles anything with

```
Error: Failed to install Python dependencies into penv
```

then PlatformIO's own environment needs pip:

```bash
curl -fsSL https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
~/.platformio/penv/bin/python3 /tmp/get-pip.py
```

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

Tagged builds carry a ready-to-flash image at
`https://github.com/AndrewKHalim/Self-Parking-Car/releases`, named
`autoparking.factory.bin`.

Put the board in download mode: hold **BOOT**, tap **RESET**, release
**BOOT**. It then enumerates as `/dev/ttyACM0` on Linux, `/dev/cu.usbmodem*` on
macOS or `COMx` on Windows.

```bash
~/.platformio/penv/bin/esptool.py --chip esp32s3 --port /dev/ttyACM0 \
    --baud 460800 write_flash 0x0 autoparking.factory.bin
```

The factory image goes at `0x0` and nothing else is needed. Do not flash
`firmware.bin` at `0x0`: that is the application alone and belongs at `0x10000`.

Check the write with `verify_flash 0x0 autoparking.factory.bin` in place of
`write_flash`, or open the USB CDC console at 115200 and watch for the
`autoparking-car: simple` line.

#### The bench project

`bench/` is a separate PlatformIO project, so it does not touch the parking
firmware or its environment. It links no libraries and takes the pin map and the
inversion flags from `include/ParkingConfig.h`.

```bash
cd bench
../.venv/bin/pio run -e motor_test -e i2c_scan -e servo_sweep
```

`env:motor_test` drives both motors forward at duty 200 from `setup()` and keeps
driving until the power is cut. It is what to flash when the only question is
whether the drive works. If the car pulls away backwards, the inversion flag for
that side is wrong. If it does not move at all, the fault is in the motor wiring,
the driver's power or its VM rail, and not the parking logic.

`env:i2c_scan` drives the three XSHUT lines high, prints the idle voltage of
GPIO 8 and 9 on the ADC, then scans with SDA on GPIO 8 and SCL on GPIO 9, closes
the bus, prints the levels again, and rescans with the two swapped. A device at
`0x29` with the pins as wired and nothing when swapped means the bus is healthy
and the wiring is right. The other way round means the two lines are crossed.
Nothing in either scan means no sensor is powered or out of reset.

`env:servo_sweep` steps the servo through 45 to 135 degrees in 5 degree steps,
one second apart, and prints each angle with the pulse it sends. The pulse map is
the library default, 544 to 2400 us over 0 to 180 degrees, so a pulse printed
here means the same thing on the car.

## How the maneuver runs

The loop runs every 25 ms and prints one serial line every 100 ms.

| phase | what it does |
|---|---|
| `idle` | motor off, wheels centred, waiting for the page's start button |
| `reverse` | PID steers on the gap difference, both motors reverse at duty 200, and the leg stops at 76 mm of rear clearance |
| `pull` | both motors forward at duty 200 with the wheels turned 25 degrees opposite the larger gap, for 1400 ms or until the rear reads over 220 mm, then a 150 ms pause |
| `parked` | motor off, wheels centred, maneuver over |

When the rear reading reaches 76 mm the car stops. If the gap difference is
over 12 mm, the rear is still under 130 mm and fewer than three corrections
have run, it pulls forward once and reverses again. After three pulls it parks
where it is. The rear sensor face is the rearmost point of the car, so the
reading is the tail clearance itself; the car rolls on about 20 mm after the
drive is cut, which leaves the tail about 50 mm off the wall.

Every reverse leg has a 4000 ms cap. If the rear sensor has no valid reading the
car still reverses, stops at the cap and prints `fault=rear` with the raw value
it saw. If the reading is valid but never reaches 76 mm it stops at the cap with
`fault=timeout`. Either way the car parks and the fault stays on the page.

## The sensors

The two side faces are perpendicular to the car, so their readings are the left
and right gaps themselves and no trigonometric projection is applied. The rear
face is the rearmost point of the car, so its reading is the tail clearance.

The lateral error is `gL - gR` in millimetres, converted to centimetres for the
PID. A positive error means the left gap is larger, so the car sits to the right
of the bay centre and the rear has to be steered toward the left wall.

Ranging runs continuously with a 33 ms measurement budget. A raw reading of 0 or
anything over 8000 mm is invalid. The library reports 8190 mm with no target and
65535 mm on an I2C timeout, so both are rejected. An invalid reading keeps the
last good value on that channel and clears its valid flag. The filter is main's
exponential moving average with alpha 0.35, and it snaps to the first valid
reading instead of creeping up from zero.

The two side faces sit ahead of the rear axle, 67 mm on the left and 147 mm on
the right. Because the stations enter the error with opposite signs, the gap
difference is not the offset:

```
gL - gR = 2*x + 0.214*sin(psi)
```

Here `x` is the lateral offset and `psi` is the yaw. At 5 degrees of yaw the
heading term alone is 18.6 mm of false error. This firmware keeps main's signal,
`gL - gR`, because there is no heading estimate.

## The steering

The servo is attached with the library defaults (`steering.attach(PIN_SERVO)`),
which gives the 544 to 2400 us pulse map over 0 to 180 degrees. Commands are
whole degrees, clamped to 45 to 135 around a centre of 90:

| constant | value | meaning |
|---|---|---|
| `CENTRE_DEG` | 90 | command that should point the wheels straight |
| `MIN_DEG` | 45 | lower clamp |
| `MAX_DEG` | 135 | upper clamp |
| `INVERTED` | false | sign between a positive lateral error and the command |

The PID is main's, unchanged: gains 1.8, 0.04 and 0.40 on an error in
centimetres, an 8 mm deadband, an integral clamp of 25 and a derivative low
pass of 0.6. The output is a servo angle offset, so the command is
`90 + round(output)` before the clamp. None of it has been tuned on this car.

A pull forward turns the wheels 25 degrees the other way: a positive gap
difference commands `90 - 25`, a negative one `90 + 25`. The pull's end is
either the 1400 ms timer or a rear reading over 220 mm, whichever comes first.

## The web page

The ESP32 joins the network in `src/wifi_config.h`. If it cannot join within
15 seconds it starts its own access point, `autoparking` with password
`autoparking`, so the page is reachable on a bench with no configured network.
Either way the page is at `/` on the printed address.

| route | what it serves |
|---|---|
| `/` | `src/dashboard_html.h`, the page |
| `/start` | sets the start flag and returns `{"start":true}` |
| `/ws` | one JSON snapshot at 10 Hz |

The page shows phase, `gL`, `gR`, rear, servo, corrections, fault and time, and
has a start button. There is no chart. The start button is disabled while the
car is reversing or pulling. The websocket reconnects on its own.

## The layout

| path | what is in it |
|---|---|
| `include/ParkingConfig.h` | pins, addresses, measured stations, servo range, gains, duties and times in one place |
| `include/DistanceSensorManager.h` | the three VL53L0X, their bring-up order and continuous ranging |
| `include/MotorController.h` | the DRV8833 on four pins at 20 kHz, forward, reverse and stop |
| `include/PIDController.h` | main's lateral PID and the servo angle mapping |
| `src/main.cpp` | the one loop, the maneuver and the web routes |
| `src/dashboard_html.h` | the page, as one PROGMEM string |
| `src/wifi_config.h` | the SSID, the password and the fallback access point |
| `bench/` | the separate bench project: motor test, I2C scan, servo sweep |
| `releases/` | the factory image published by a tag |
| `lib/Network/` | the vendored `Network` source, kept because the platform expects it |

## Status

Build-verified only. Nothing on this branch has been flashed or run on the car.

The firmware builds and links for the ESP32-S3 Super Mini at 48,092 bytes of RAM
(14.7 percent of 327,680) and 995,654 bytes of flash. The application partition
is the 4,063,232-byte `app0` from `max_app_4MB.csv`, so the app uses 24.5
percent of it. PlatformIO prints 11.9 percent because it divides by the board's
8 MB `maximum_size`, which is not the figure to quote. The
`releases/autoparking.factory.bin` built from this tree is 1,077,408 bytes, sha256
`8913dafd1f5ecd6bfc4ae007bdae4c9b002fcc57e1a5d611c38dd496445d44be`.

The three bench environments build. The servo, sensor and drive behaviour is not
observed on hardware in this branch. The servo centre and direction are the
assumed 90 degrees and not inverted, the two motor inversion flags are main's,
and the PID gains are main's, never tuned on this car. Duty 200 is the only
duty ever seen to move this car and only forward on the bench. No maneuver has
run to completion.
