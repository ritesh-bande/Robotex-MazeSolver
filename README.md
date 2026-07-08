# 🤖 ESP32 Wall Following Robot

A PID-based autonomous wall-following robot using ESP32, IR distance sensors, quadrature encoders, and DC motors.

---

## Features

- Left wall following
- PID steering control
- Encoder-based precise turning
- Dead-end detection
- Junction handling
- Calibration mode
- Modular code

---

## Hardware

- ESP32 DevKit
- 2 DC Motors
- Motor Driver (TB6612FNG / L298N)
- 2 Quadrature Encoders
- IR Distance Sensors
- Li-ion Battery

---

## Project Structure

```
code/
    rotate_calibration_test/
    wall_follower_left_priority/
    wall_follower_no_filter/
    wallfollower_only_pid/

docs/
images/
hardware/
```

---

## Algorithm

1. Read sensors
2. Detect walls
3. Calculate error
4. Apply PID
5. Adjust motor speed
6. Detect intersections
7. Execute turns

---

## Images

(Add robot photos here)

---

## Demo

(Add YouTube link)

---

## Future Improvements

- Maze solving
- Flood Fill
- Mapping
- Bluetooth Monitoring
- Web Dashboard

---

## Author

Ritesh Bande
