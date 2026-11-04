# FAQ

Questions the bench and the WebUI keep raising, answered from the code. Every
number is the one in `src/config.h`, and where a value comes from the CAD
geometry the arithmetic is shown.

## Steering

### Why does the road wheel stop at 15.4 degrees?

The servo travels 45 to 135 degrees and sits at 90 when centred. The linkage
multiplies, so the wheel angle is `(servo - 90) / 2.92`. Half the travel is
`45 / 2.92 = 15.41` degrees, so 15.4 is the mechanical stop, not a tuning
choice. Two `static_assert`s in `config.h` keep the two ends of the range inside
the servo travel.

### Where does 2.92 servo degrees per wheel degree come from?

The pinion has 12 teeth on a rack of 2.6 mm pitch, so one pinion turn moves the
rack `12 * 2.6 = 31.2 mm`, which is `0.0867 mm` per servo degree. The servo horn
is 14.25 mm from the kingpin, so one degree of road wheel moves the rack
`14.25 * pi / 180 = 0.2487 mm`. The ratio is `0.2487 / 0.0867 = 2.87`, and the
header stores 2.92.

### What turning circle does the full lock give?

`R = wheelbase / tan(wheel) = 183 / tan(15.4) = 664 mm` at the rear axle, with
the wheel at the stop. Both numbers are CAD values, so the radius moves if the
chassis does.

### Why is the servo held for 300 ms before the car moves?

The servo swings through the rack while the motors are braked, and 300 ms is
what the bench needed for a full lock change. The same wait applies at the end
of a stroke, so the wheels are centred before the next reverse leg starts. The
page can raise it to 2000 ms.

## The reverse leg

### Why does the car stop 100 mm from the wall?

`rear_target_mm` is 100 mm with a 5 mm tolerance either side. The rear sensor
face sits 38 mm behind the rear axle, so the stop leaves the tail clear of the
wall. The target can be set from 30 to 500 mm on the page.

### Why is the brake commanded at 115 mm instead of 105?

At duty 200 the chassis coasts 10 to 15 mm after the motors brake. Braking at
`target + tolerance + lead = 100 + 5 + 10 = 115 mm` lets that coast finish
inside the 95 to 105 mm band. Braking at the top of the band instead would land
the car short of the target, which then reads as an overshoot on the other side.

### How is the yaw angle psi computed?

From the change in the gap difference over the reverse leg that just ended:

```
tan(psi) = (diff_start - diff_stop) / (2 * travel)
psi      = atan(tan(psi))
```

`diff` is `gL - gR`, and `travel` is `rev_start_rear_mm - rear_mm`. The 2 is
there because the two gaps move in opposite directions during a straight
reverse: one sensor swings toward its wall by `travel * sin(psi)` while the
other swings away by the same amount, so the difference changes twice as fast
as either gap. The result is clamped to `+/-0.84`, which is `tan(40 deg)`.

The estimate needs 25 mm of travel, or 15 mm if the leg was stopped early by
the wall guard or the stall detector. Below that the previous leg's angle is
used, and the guard runs on `0` until one exists.

### Why does the slope ignore the 80 mm sensor stagger?

The stagger term is constant along the leg, because both sensors keep their
longitudinal stations. `56 * tan(psi)` appears in both samples and drops out of
the difference. It only has to be removed from the standing measurement, which
is where the stagger correction sits.

### Why 56 mm for the stagger coefficient?

The side sensors sit at `z = 67` and `z = 147` mm ahead of the rear axle and the
body centre is at `234 / 2 - 38 = 79` mm. Their offsets from that centre are
`-12` and `+68`, and the sum is 56. At yaw, part of `gL - gR` is heading rather
than offset, so the offset the alignment test uses is
`diff = gL - gR - 56 * tan(psi)`.

### Why is a fixed 500 ms backoff replaced by a nudge?

A second reverse leg at duty 200 overshoots the same way, so the old fixed
stroke turned into a loop that ended at `backoff_limit`. The nudge is
`120 + 5 * overshoot` ms, clamped to 120..260, with the servo centred. It undoes
the overshoot and goes straight to `SETTLE_REAR`, so the car never drives at the
wall a second time.

### Why is the nudge time proportional to the overshoot?

A 5 mm overshoot and a 15 mm overshoot need different strokes. A fixed 500 ms
pulled far past the target, which is what forced the second reverse leg.

### What happens if the nudge pulls too far?

If the settled `rear_mm` is still more than `target + 25 mm`, the nudge failed
and the machine starts a reverse leg again. That limit is
`REAR_NUDGE_MAX_ABOVE_MM`.

### Why can a tire touch the wall while the gap sensor still reads 55 mm?

The tires are 170 mm across and the side sensors 95 mm apart, so each sidewall
sticks `(170 - 95) / 2 = 37.5 mm` outboard of its sensor face. A 17.5 mm tire
gap is therefore a 55 mm sensor reading, and at yaw the wheel furthest along the
car reaches the wall long before its sensor does.

### How does the wall guard project the tire position?

Each tire edge is projected through `tan(psi)` using its longitudinal distance
from the sensor that watches that side. The rear tire edges are 100.5 mm behind
the left sensor and 180.5 mm behind the right one, the front edges 149.5 mm and
69.5 mm ahead. A car at 25 degrees yaw with `gR = 130 mm` therefore has
`130 - 37.5 - 180.5 * 0.47 = 7.7 mm` of tire clearance, even though the gap
reading looks comfortable.

### Why does the guard need 350 ms and a 4 mm shrink?

The first 350 ms of a leg are too short for the slope to mean anything, so the
guard runs on the previous leg's angle. A gap also has to have shrunk by 4 mm
before it counts as closing, so one noisy pair while reversing straight does not
brake the car in open space.

### What is the stall detector for?

A rear wheel hung on a wall leaves `rear_mm` almost unchanged while the motors
keep pushing. If the reading has not moved 8 mm in 700 ms, the leg stops early
and the correction stroke runs. Waiting for the 6 s reverse timeout would grind
the tire against the wall for the rest of it.

### What stops a run from trying to unstick forever?

Each unstick attempt is counted. Once the count passes `2 * backoff_max`, the
next stall ends the run with `rev_timeout`; with the default of 3 that is 6
attempts.

### Why 9.5 degrees of lock to get off a wall?

A wheel pressed against a wall has to climb away from it, and a small steering
angle mostly pushes the tire along the wall. `UNSTICK_MIN_WHEEL_DEG` is the floor
the config sets for that lock, and the sign is chosen away from the closer side.

## The correction stroke

### Why does the lateral correction use two halves with opposite lock?

Each half rotates the car by `wheel * stroke / wheelbase`. With equal lock for
equal time, the second half removes the heading the first half added and the
lateral movement from both halves adds. The output is a sideways shift with no
net rotation.

### What does `k_yaw` do?

It converts degrees of yaw error into road-wheel degrees, 0.6 by default, and
the result is clamped to the 15.4 degree stop. The correction is split over the
two halves, so `0.6 * psi` is the full angle and each half gets half of it.

### Why does a heading-only error get one half stroke instead of two?

The car is already where it belongs laterally, so the counter half would move it
sideways. Half a stroke at the yaw angle rotates the car without translating it.
The same branch also resets the PID, because carrying an integral from a
lateral correction would leave a bias behind.

### Why is there a separate branch above 12 degrees of yaw?

An S-curve keeps the diagonal pose while it shifts the car sideways, so at 20 to
30 degrees the body sweeps one end into the wall. Above
`LARGE_YAW_STRAIGHTEN_DEG` the machine runs one unsplit stroke for the full
`forward_move_ms` instead, at `k_yaw * psi` clamped to the wheel stop. At
`psi = -25` degrees that is a full second at -15 degrees, which pulls the rear
tire off the right wall and turns the car about 20 degrees the other way.

### What are the PID gains?

`kp = 0.15` road-wheel degrees per mm of offset, `ki = 0.01` per mm-second, and
`kd = 0.02` per mm/s. The integral is capped at 5 degrees of wheel so it cannot
wind up while the car is stuck, and the sum is clamped to the 15.4 degree stop.
The block runs once per stationary sample, and the first sample of a run uses a
1 s interval because there is no previous timestamp yet.

### How long does one correction cycle take?

`SETTLE_REAR` 200 ms, `STEER_WAIT` 300 ms, `FORWARD_CORRECT` 500 ms,
`STEER_COUNTER_WAIT` 300 ms, `FORWARD_COUNTER` 500 ms, and `SETTLE_FORWARD`
300 ms, so 1900 ms from the stop to the next sample. The 500 ms halves are
`forward_move_ms / 2`, so the page's value moves both of them.

## Tuning the PID

### How do I change the gains while it runs?

`/config` takes `kp`, `ki` and `kd`. A handler stores the value and the control
task applies it on the next tick, so the change is live and needs no reflash. A
negative value is ignored rather than clamped, which is how the page sends only
the fields that were edited.

### What does each gain act on?

The error is `corr_diff_mm_`, the stagger-corrected `gL - gR` in millimetres.
The output is road-wheel degrees:

```
u = kp * e + ki * integral(e dt) + kd * (e - e_prev) / dt
```

`dt` is the time since the previous evaluation, which spans a whole correction
cycle including the reverse leg, and the first sample of a run uses 1 s. The
derivative is 0 on that first sample because there is no previous error to
subtract. The output is clamped to `+/-15.4` degrees, which is the wheel stop.

The integral is stored in mm-seconds and capped at `5 / ki`, so its share of the
output can never exceed 5 degrees. With `ki = 0.01` the cap sits at 500 mm-s.
Setting `ki = 0` clears the integral.

### Where should I start?

From the defaults: `kp = 0.15`, `ki = 0.01`, `kd = 0.02`. For a first pass set
`ki` and `kd` to 0 and tune `kp` alone, because a proportional term alone is
easy to read: a 20 mm offset asks for 3 degrees of wheel. Then bring `ki` up to
remove what `kp` leaves behind.

### What do I change when I see a given behaviour?

| what you see | what it means | what to change |
| --- | --- | --- |
| the tail crosses the bay centre and the next cycle pulls it back | the proportional term is over-driving the stroke | halve `kp` |
| the offset drops, then flattens out above `align_diff_mm` | the proportional term alone cannot close it | raise `ki` to 0.02, then 0.05 |
| the wheels sit at 15.4 degrees for the whole stroke | the offset is past 103 mm at `kp = 0.15`, so the stroke runs at full lock | if that over-corrects, the stroke is too long or too fast: reduce `duty`, since `forward_move_ms` needs a reflash |
| the offset gets worse every cycle | the loop is correcting in the wrong direction, which usually means `gL` and `gR` are swapped | stop the run and check the side sensors |
| the car centres but the heading swings back and forth | `k_yaw` is too high for the bay | lower `k_yaw` from 0.6 toward 0.3 |
| the car centres but parks skewed | `k_yaw` is too low | raise `k_yaw` |
| the correction never runs at all | the offset is already inside `align_diff_mm` | that is the tolerance working; lower `align_diff_mm` if the bay needs it |
| the tail gets closer and the run still ends at `backoff_limit` | the rear distance failed, not the lateral loop | see the brake lead and the nudge above |
| the stroke grows cycle after cycle while the offset stays put | the integral is winding up | lower `ki`, or set it to 0 |

### Which numbers are not on the page?

`/config` covers the target, the alignment tolerance, the steer wait, the duty,
the backoff cap and the four gains (`kp`, `ki`, `kd`, `k_yaw`). Everything else
in `defaults` needs a reflash, including `forward_move_ms` (1000), `settle_ms`
(200), the nudge window (120 to 260 ms at 5 ms per mm), the 10 mm brake lead,
the guard thresholds, the stall window and the 6 s reverse timeout. `duty` is
the runtime lever for how far any timed stroke travels.

### Why does one gain behave differently between runs?

The S-curve turns the wheel angle into heading and sideways movement through
the distance the car travels, and that distance is duty-dependent. The same
stroke time moves further at a higher duty, so gains tuned at 200 can
over-correct at 255.

### Why does the integral survive from one cycle to the next?

It is only cleared by `start`, `stop`, the heading-only trim and the steep-yaw
arc, because those two branches are rotations that should not carry a lateral
bias. An ordinary correction keeps its integral on purpose, so a small offset
that `kp` cannot close still accumulates help.

### Can I tune it with the car stopped?

No. The block only runs on a settled sample with an offset outside
`align_diff_mm`, so a stopped machine never evaluates it. Tuning means running
the maneuver and changing one gain per run.

## Sensors and timing

### Why three sensors with different addresses?

All three VL53L0X parts power up at `0x29`, so they share one bus only after
`XSHUT` bring-up: the firmware holds two of them off, releases one at a time, and
moves it to its own address (left `0x29`, right `0x2A`, rear `0x2B`).

### Why a 33 ms control period?

The timing budget is 33 ms and the loop period matches it, so each tick gets one
fresh sample per sensor at 30 Hz. The page is fed at 10 Hz, which is the 100 ms
telemetry period.

### What does a reading of 8000 mm or more mean?

The library returns 8190 when there is no target and 65535 on an I2C timeout.
`OUT_OF_RANGE_MM = 8000` treats anything above it as unusable rather than as a
very distant wall.

### Why does the machine compute nothing while the car moves?

The distance sensors are read on every tick but only sampled for geometry when
the chassis is braked, so a rolling car cannot produce a stale mix of readings.
The two exceptions are the reverse-leg anchor, needed to measure the slope
afterwards, and the live tangent the wall guard projects with.

## Tuning and faults

### Which values can I change without reflashing?

The page's `/config` endpoint takes these:

| field | default | range |
| --- | --- | --- |
| `rear_target_mm` | 100 | 30 to 500 |
| `align_diff_mm` | 10 | 1 to 200 |
| `steer_ms` | 300 | up to 2000 |
| `duty` | 200 | up to 255 |
| `backoff_max` | 3 | 0 to 20 |
| `yaw_tol_deg` | 5 | 0.5 to 25 |
| `k_yaw` | 0.6 | 0 to 3 |
| `kp` | 0.15 | 0 to 5 |
| `ki` | 0.01 | 0 to 1 |
| `kd` | 0.02 | 0 to 1 |

`backoff_max = 0` lifts the cap, so the car keeps pulling until Stop.

### What do the fault strings mean?

`rear_sensor`: the rear reading was unusable at start. `side_sensor`: a side
reading was unusable at a sample. `rev_timeout`: 6 s of reversing without a
stop, or the unstick budget ran out. `backoff_limit`: the run needed more
straight pull-aways than `backoff_max` allows.

### Why does an early stop never count as parked?

A leg stopped by the wall guard or the stall detector may still show gaps that
look aligned, while the car is short of the target or hung on a wall. The
`was_early` flag forces a correction stroke, so the run cannot declare success on
a pose it never reached.

### Why does the page show two differences?

`raw_diff_mm` is `gL - gR` as the two sensors returned it. `diff_mm` has
`56 * tan(psi)` removed, which is the offset the alignment test compares against
`align_diff_mm`. On a straight car the two are equal.

### Why does the first start sometimes enter `SETTLE_REAR` immediately?

`start()` checks `rear_mm` against the target before assigning a leg. If the car
is already at 100 mm or closer, it brakes and settles instead of reversing into
the wall.
