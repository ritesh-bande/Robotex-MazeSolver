# Robotex Maze Solver Bot

A rule-based, wall-following maze-solving robot built on an ESP32, using three VL53L1X time-of-flight distance sensors (left/front/right) and dual quadrature encoders for closed-loop turning and straight-line correction.

## Overview

The robot navigates a maze by tracking a wall (left or right, selectable at startup) and reacting to its surroundings using an explicit, prioritized rule set — rather than a flood-fill or full maze-mapping algorithm. It is designed for micromouse / line-maze style competitions such as Robotex.

Core capabilities:
- Non-blocking, continuous ToF distance sensing on three sides
- PID-based wall following at a fixed target distance
- Closed-loop in-place turns using quadrature encoder ticks (not fixed timing)
- Encoder-balanced straight driving when no wall is present on either side
- Dead-end detection and 180° recovery turn
- Adaptive stall-timeout guard so a jammed/slipping wheel can't hang the turn forever

## Hardware

| Component | Details |
|---|---|
| MCU | ESP32 |
| Distance sensors | 3x VL53L1X (Left, Front, Right), Short distance mode |
| Motor driver | Dual H-bridge (e.g. TB6612FNG-style: AIN1/AIN2/PWMA, BIN1/BIN2/PWMB, STBY) |
| Encoders | Quadrature, 2 channels per wheel (A/B) |
| Mode select | 1 push button (BTN1 = left-wall-follow mode) |

### Pin Map

```
Buttons
  BTN1        GPIO 34   (left-wall-follow select)
  BTN2        GPIO 35   (right-wall-follow — currently disabled in code)

Motor Driver
  AIN1        GPIO 17
  AIN2        GPIO 16
  PWMA        GPIO 4
  STBY        GPIO 5
  BIN1        GPIO 19
  BIN2        GPIO 18
  PWMB        GPIO 21

Encoders
  LEFT_ENC_A  GPIO 2
  LEFT_ENC_B  GPIO 15
  RIGHT_ENC_A GPIO 22
  RIGHT_ENC_B GPIO 23

ToF Sensors (I2C on GPIO 33 = SDA, GPIO 32 = SCL)
  XSHUT_L     GPIO 13   -> reassigned to I2C address 0x30
  XSHUT_F     GPIO 27   -> reassigned to I2C address 0x31
  XSHUT_R     GPIO 25   -> reassigned to I2C address 0x32
```

All three VL53L1X sensors boot at the same default I2C address, so each is held in reset via its XSHUT pin, brought up one at a time, and reassigned a unique address during `setupSensors()`.

## Dependencies

- [`VL53L1X` Arduino library](https://github.com/pololu/vl53l1x-arduino) (Pololu)
- `Wire.h` (bundled with Arduino core)
- ESP32 Arduino core (for `IRAM_ATTR`, `analogWrite`, hardware interrupts, etc.)

## Navigation Logic

On every loop iteration, `behaviorStep()` evaluates rules in strict priority order — the first match wins:

1. **Dead end** — walls detected on all three sides: back off briefly, then rotate 180°.
2. **Front blocked**
   - If the left side is open: curve left into the opening (smoother than a pivot).
   - Otherwise: back off briefly, then rotate 90° right.
3. **Tracked-side wall present, front clear** — follow it with PID at `targetDist`.
4. **No wall on either side, front clear** — genuine open space; drive straight using encoder-balanced correction instead of arcing toward a nonexistent wall.
5. **Tracked-side wall missing, opposite side walled** — arc toward where the tracked wall should be (fast outer wheel / slow inner wheel).

> **Note:** `behaviorStep()` implements the full rule set described above, but the current `loop()` runs a simplified test routine instead (see [Current State](#current-state-of-loop) below).

### Wall-Following PID

`runWallPIDSingle()` drives a standard PID loop on the error between the measured wall distance and `targetDist`. For right-wall following, the error is mirrored (`invertMirror`) so the same PID math and steering sign work for both left and right modes.

```
Kp = 0.56
Ki = 0
Kd = 0.085
```

Output is applied as a differential speed offset around `baseSpeed`, clamped to `[30, 240]` per wheel so neither steering direction is starved of headroom.

### Closed-Loop Turning

`rotate(degree)` turns in place using quadrature encoder ticks rather than a fixed delay:

- Target tick count = `abs(degree) * encoderCountToDegrees` (calibrate `encoderCountToDegrees` per-robot).
- A turn is only accepted as "done" after `ROT_DONE_HITS` (5) consecutive in-tolerance (`ROT_TOLERANCE` = 3 ticks) readings, to avoid stopping on a single noisy tick.
- Speed ramps down smoothly over the final 50 ticks of whichever wheel is furthest along.
- A stall-timeout guard (`ROT_BASE_TIMEOUT_MS` + `ROT_MS_PER_DEGREE * degree`) forces the turn to end if a wheel jams or slips, instead of spinning forever.

### Open-Space Straight Driving

When no wall is present on either side, `driveStraightUsingEncoders()` compares left/right tick counts each cycle and applies a small proportional correction (`straightKp = 0.6`) to cancel drift, since there's no wall to reference for steering.

### Sensor Filtering

Raw ToF readings are exponentially smoothed (`FILTER_ALPHA = 0.4`) into `filtL`/`filtF`/`filtR` before being used for wall/front detection and PID, to reduce single-sample mm jitter. Readings of `0` or `> 4000` (sensor error codes) are clamped to `4000` before filtering.

## Key Tunable Parameters

| Parameter | Value | Meaning |
|---|---|---|
| `baseSpeed` | 220 | Cruise motor speed while wall-following |
| `targetDist` | 45 mm | Desired distance to the tracked wall |
| `frontThresh` | 150 mm | Distance below which the front is considered blocked |
| `wallThresh` | 90 mm | Distance above which a side wall is considered missing |
| `outerSpeed` / `innerSpeed` | 255 / 70 | Wheel speeds while arcing to find a lost wall |
| `rotationSpeed` | 180 | Cruise speed during in-place rotation |
| `encoderCountToDegrees` | 0.945 | Ticks per degree of rotation — **must be calibrated per robot** |
| `timingBudget` | 20 ms | VL53L1X measurement budget, also sets loop cadence |

## Setup & Usage

1. Wire the hardware per the pin map above.
2. Install the Pololu `VL53L1X` library via the Arduino Library Manager.
3. Flash the sketch to the ESP32.
4. Open Serial Monitor at `115200` baud.
5. On boot, the robot waits for `BTN1` to be pressed to select left-wall-follow mode, then primes the sensor filters and starts running.

## Current State of `loop()`

The full rule-based `behaviorStep()` function is implemented but **not currently called**. The active `loop()` runs a minimal test routine instead: it stops if the front sensor reads under 150 mm, otherwise runs right-wall PID follow. To run the complete maze-solving behavior (dead-end handling, opening detection, open-space driving, etc.), swap the body of `loop()` back to call `behaviorStep()`.

## Known Limitations / TODO

- Right-wall-follow mode (`BTN2`) is wired in comments but disabled; only left-wall-follow is currently selectable at startup.
- `loop()` currently bypasses `behaviorStep()` for single-mode PID testing — needs to be switched back for full maze solving.
- `encoderCountToDegrees` and PID gains are robot-specific and must be recalibrated for a different chassis/wheel/motor combination.
