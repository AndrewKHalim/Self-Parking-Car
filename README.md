# Self-parking-car

A small RC car reverses into a perpendicular parking bay, centres itself
between the two side walls and stops before the rear wall. Three VL53L0X
time-of-flight sensors feed one loop, one PID on the lateral error and a small
state machine. Telemetry is one serial line at 10 Hz. There are no tasks, no
web server and no dashboard on this branch.

The branch is `simple`. It rebuilds the firmware in the shape of `main` with
the pins, sensor geometry and thresholds of this car. `f4dca` keeps the older
control core, the dashboard and the host simulator.

## The hardware

| part | spec | notes |
|---|---|---|
| MCU | ESP32-S3 Super Mini | 4MB flash, 240MHz dual core, native USB-C, module soldered to the board |
| ToF sensors | 3x VL53L0X | must be VL53L0X, not VL53L1X. they are not drop-in compatible and use a different library. two face straight out sideways, one faces back |
| steering servo | SG90 or MG996R | the MG996R has less backlash, which matters here |
| motor driver | DRV8833 module | a TB6612FNG also works and gives a clean sleep between runs |
| drive motor | stock Tamiya yellow motor, 6V rated | |
| chassis | Tamiya RC chassis with servo bay | the stock knuckle is turned by the drive motor through a plastic arm, so it cannot take a commanded angle and the chassis is converted to servo steering |
| power | one 5 V rail for the whole car | the ESP32, the DRV8833 VM pin and the servo share it. The stock 6 V motor sees about 4.5 V at the driver's dropout, so the reverse duty has to be high enough for it to break away from rest. The DRV8833 is rated to 10.8 V, so VM must never be fed from a 3S pack directly |

The three sensors are addressed over one I2C bus. Each one is enabled in turn
through its XSHUT line and given an address: `0x29` left, `0x2A` right, `0x2B`
rear. All three power up at `0x29`, so they are released one at a time and the
left sensor, which keeps `0x29`, comes up last.

### Pin map

| GPIO | function |
|---|---|
| 1 | VL53L0X left XSHUT |
| 2 | VL53L0X right XSHUT |
| 6 | DRV8833 IN3 (right motor) |
| 7 | DRV8833 IN4 (right motor) |
| 4 | DRV8833 IN1 (left motor) |
| 5 | DRV8833 IN2 (left motor) |
| 8 | I2C SDA |
| 9 | I2C SCL |
| 10 | VL53L0X rear XSHUT |
| 11 | steering servo signal, 50Hz, library default pulse map |

The table is the harness as it stands. Every pin in it, the two inversion flags
and the three sensor addresses live in `include/ParkingConfig.h` under
`namespace board`, so a measured change touches one file.

Leave GPIO 26 to 32 alone: the in-package flash claims them. GPIO 19 and 20 are
the USB pins, GPIO 43 and 44 are UART0, and GPIO 0, 3, 45 and 46 are strapping
pins that are sampled at boot. GPIO 9, 10 and 11 are FSPI IO MUX pins, safe here
only because nothing on this board drives SPI2.

## The platform

`platformio.ini` pins a pioarduino espressif32 release by URL rather than by a
registry version. The registry `espressif32` platform ships Arduino core 2.x,
which is not the core this car was brought up on. The pinned release is
55.03.312, Arduino core 3.3.12.

The board definition is `esp32-s3-devkitc-1`, with three overrides for the
Super Mini: `qio_qspi` memory type, 4 MB of flash and the
`max_app_4MB.csv` partition table. `-DBOARD_HAS_PSRAM` puts the 2 MB of
in-package PSRAM on the heap, and the two USB flags put `Serial` on the USB CDC
port at boot.

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

#### Flash a prebuilt binary

Tagged builds carry a ready-to-flash image at
`https://github.com/AndrewKHalim/Self-Parking-Car/releases`, named
`autoparking.factory.bin`. The same file sits in the tagged tree under
`releases/`.

Put the board in download mode: hold **BOOT**, tap **RESET**, release
**BOOT**. It then enumerates as `/dev/ttyACM0` on Linux, `/dev/cu.usbmodem*` on
macOS or `COMx` on Windows.

```bash
~/.platformio/penv/bin/esptool.py --chip esp32s3 --port /dev/ttyACM0 \
    --baud 460800 write_flash 0x0 autoparking.factory.bin
```

`autoparking.factory.bin` is the bootloader, the partition table, the boot app
and the application merged into one image, so it goes at `0x0` and nothing else
is needed. Do not flash `firmware.bin` at `0x0`: that is the application alone
and belongs at `0x10000`.

The four separate images work too, if they are what you have:

```bash
~/.platformio/penv/bin/esptool.py --chip esp32s3 --port /dev/ttyACM0 \
    --baud 460800 write_flash 0x0 bootloader.bin \
    0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin
```

Check the write with `verify_flash 0x0 autoparking.factory.bin` in place of
`write_flash`, or open the USB CDC console at 115200 and watch for the
`autoparking-car: simple branch` line.

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
here means the same thing on the car. Use it to find the centre and the steering
direction.

## How the maneuver runs

The loop runs every 25 ms and prints one telemetry line every 100 ms.

| state | what it does |
|---|---|
| `IDLE` | wheels centred, motor off, waits 2 s |
| `REVERSE` | PID steers on the lateral error, motor reverses at duty 90, or duty 40 below 160 mm of rear reading |
| `PULL_FORWARD` | motor forward at duty 140 for 1.4 s or until the rear reading passes 220 mm, with counter-steer opposite the error sign |
| `PARKED` | motor off, wheels centred, maneuver over |
| `EMERGENCY_STOP` | motor off, wheels centred, latched |

The reverse stops when the rear reading reaches 70 mm. The rear sensor face is
the rearmost point of the car, so that reading is the tail clearance; the car
rolls on about 20 mm after the drive is cut, which leaves the tail about 50 mm
off the wall.

While the rear reading is inside 160 mm, a lateral error larger than 10 mm asks
for a pull forward. The car backs out, moves forward with the wheels turned
against the error, and returns to `REVERSE`. The pull-forward count is capped at
3, so a wall that cannot be reached does not keep the car hunting.

Any valid reading under 40 mm stops the car and latches `EMERGENCY_STOP`. Power
cycle the board to clear it.

## Sensor geometry

The two side faces are perpendicular to the car, not angled. Their readings are
the left and right gaps themselves, so there is no `sin(45)` factor anywhere.
`gL` and `gR` are the gaps in metres.

The lateral error is `gL - gR`. A positive error means the left gap is larger,
so the car sits to the right of the bay centre and the rear has to be steered
toward the left wall. The PID keeps main's sign behaviour.

The two side faces sit ahead of the rear axle: 67 mm on the left and 147 mm on
the right. A centring-only loop does not need those numbers, so they are
recorded as constants in `namespace geometry` and nothing derives a heading
from them. A heading estimate would need them.

The rear face is the rearmost point of the car, so the rear reading is the tail
clearance itself. Ranging runs continuously and the loop reads the latest
result. A reading of 8190 mm or 65535 mm means no target or a timeout, and is
rejected.

## The steering

The servo is attached with the library defaults (`servo.attach(PIN_SERVO)`),
driven at whole degrees after clamping to the range in `namespace servo`:

| constant | value | meaning |
|---|---|---|
| `CENTRE_DEG` | 90 | command that should point the wheels straight |
| `DIR` | 1 | sign between a positive lateral error and the servo command |
| `MIN_DEG` | 45 | lower clamp |
| `MAX_DEG` | 135 | upper clamp |

The lateral PID works in metres and its output is a servo angle offset in
degrees. The gains are 180, 4.0 and 40, which are `main`'s 1.8, 0.04 and 0.40
per centimetre rescaled to metres. They are inherited and unverified on this
car. The deadband is 8 mm, the integral clamp is 25 and the derivative low pass
is 0.6.

Two constants have to be calibrated on the car:

1. `servo::CENTRE_DEG`. Run the bench sweep and find the angle where the wheels
   point straight. Put that number here.
2. `servo::DIR`. Command a small turn and watch which way the wheels go. If a
   positive lateral error steers the wrong way, set `DIR` to `-1`.

Change one at a time. A wrong centre looks like an offset the PID cannot remove,
and a wrong sign looks like the car steering away from the wall on every
correction.

## The layout

| path | what is in it |
|---|---|
| `include/ParkingConfig.h` | pins, addresses, geometry, servo constants, PID gains and maneuver thresholds, in one place |
| `include/DistanceSensorManager.h` | the three VL53L0X, their bring-up order and continuous ranging |
| `include/MotorController.h` | the DRV8833 on four pins, forward, reverse and stop |
| `include/PIDController.h` | the lateral PID and the servo angle mapping |
| `src/main.cpp` | the one loop and the state machine |
| `bench/` | the separate bench project: motor test, I2C scan, servo sweep |
| `releases/` | the factory images published by a tag |
| `lib/Network/` | the vendored `Network` source, kept because the platform expects it |

## Status

Build-verified only. Nothing on this branch has been flashed or run on the car.

The firmware builds and links for the ESP32-S3 Super Mini at 24,740 bytes of RAM
(7.6 percent of 327,680) and 363,095 bytes of flash. The application partition
is the 4,063,232-byte `app0` from `max_app_4MB.csv`, so the app uses about 8.9
percent of it. PlatformIO prints 4.3 percent because it divides by the board's
8 MB `maximum_size`, which is not the figure to quote. The merged
`releases/autoparking.factory.bin` is 435,200 bytes.

The three bench environments build. The servo, sensor and drive behaviour is
not observed on hardware in this branch. The servo centre and direction are
still the assumed 90 degrees and `DIR = 1`, and the PID gains are main's,
rescaled but never tuned on this car. No maneuver has run to completion.
