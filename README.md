# Line Following Robot (LFT)

An Arduino Nano–based line-following robot using a TB6612FNG dual motor driver, two N20 DC motors, and a 7-channel analog IR sensor array. The firmware includes sensor calibration, PID-based line following, adjustable derivative gain (`Kd`), EEPROM persistence, and an in-place U-turn routine when the line is lost.

## Features

- Arduino Nano controller
- TB6612FNG dual motor driver with two N20 motors
- Seven analog IR sensors (`A0`–`A6`)
- Automatic sensor calibration by spinning for approximately 3 seconds
- Separate **Dry Run** and **Final Run** modes
- PID line-following control
- On-the-fly `Kd` adjustment using buttons
- Saves the tuned `Kd` value to EEPROM when stopping
- Stops when the line is lost, then attempts an in-place turn to reacquire it
- Serial diagnostics at `115200` baud

## Hardware

- Arduino Nano
- TB6612FNG motor driver
- 2 × N20 DC motors
- 7-channel analog IR sensor array
- Battery suitable for the motors and electronics
- 3 push buttons
- Robot chassis and wiring

> **Power note:** Use a suitable motor supply and ensure the Arduino, sensor array, and motor driver share a common ground. Check the motor driver's and battery's voltage/current limits before powering the robot.

## Pin Connections

### TB6612FNG motor driver

| TB6612FNG signal | Arduino Nano pin | Purpose |
|---|---:|---|
| AIN1 | D3 | Left motor direction |
| AIN2 | D2 | Left motor direction |
| PWMA | D9 | Left motor speed (PWM) |
| BIN1 | D5 | Right motor direction |
| BIN2 | D6 | Right motor direction |
| PWMB | D10 | Right motor speed (PWM) |
| STBY | D12 | Driver standby / button 3 shared pin |

### IR sensor array

| Sensor | Arduino Nano pin |
|---|---|
| Sensor 0 (leftmost) | A0 |
| Sensor 1 | A1 |
| Sensor 2 | A2 |
| Sensor 3 (centre) | A3 |
| Sensor 4 | A4 |
| Sensor 5 | A5 |
| Sensor 6 (rightmost) | A6 |

### Buttons and LED

| Control | Arduino Nano pin | Idle action | While running |
|---|---:|---|---|
| Button 1 | D7 | Calibrate sensors | Increase `Kd` |
| Button 2 | D11 | Start Dry Run | Decrease `Kd` |
| Button 3 | D12 | Start Final Run | Stop robot |

The built-in LED on D13 indicates calibration/run line detection status.

**Important D12 wiring:** D12 is shared by button 3 and the TB6612FNG `STBY` signal. The sketch configures D12 as `INPUT_PULLUP`, never as an output. The button should pull D12 to GND when pressed; this also pulls `STBY` LOW and disables the motor driver while held. Verify your physical wiring carefully before powering the robot.

## Default Firmware Settings

| Setting | Value in sketch |
|---|---:|
| `Kp` | `0.18` |
| `Kd` | `0.040` |
| `Ki` | `0` |
| Dry Run target speed | `170` |
| Final Run target speed | `220` |
| Initial run ramp speed | `30` |
| U-turn speed | `150` |
| U-turn direction | Right (`UTURN_DIR = 1`) |
| Line-lost delay before U-turn | `80 ms` |
| U-turn timeout | `3000 ms` |
| `Kd` adjustment per button press | `0.02` |
| `Kd` allowed range | `0.0`–`30.0` |
| Serial baud rate | `115200` |

Motor speeds use the Arduino PWM command range (`0`–`255`), with negative values commanding reverse. Actual speed depends on the battery, motors, driver, and robot mechanics; these numbers are not RPM values.

## Getting Started

1. Assemble the robot and connect the motors, driver, sensors, and buttons according to the tables above.
2. Open `Working/Working.ino` in the Arduino IDE.
3. Select **Arduino Nano** and the correct processor/bootloader option for your board.
4. Connect the board over USB and select its COM port.
5. Upload the sketch.
6. Open Serial Monitor at **115200 baud** to see the initial `Kp` and `Kd` values and calibration sensor ranges.
7. Place the robot over the intended line and press **Button 1 (D7)** to calibrate. During calibration, the robot spins for approximately 3 seconds so the sensors can sample the surface and line.
8. After calibration, press **Button 2 (D11)** for a Dry Run or **Button 3 (D12)** for a Final Run.

Calibration is stored only in RAM in this sketch, so recalibrate after a reset or power cycle.

## Button Operation

### While idle

- **D7 — Calibrate:** samples sensor minimum/maximum values while the robot spins for approximately 3 seconds, then returns to idle.
- **D11 — Dry Run:** starts line following at target speed `170`. Calibration must have been performed since power-up/reset.
- **D12 — Final Run:** starts line following at target speed `220`. Calibration must have been performed since power-up/reset.

If a run is requested before calibration, the LED flashes to indicate an error and the robot returns to idle.

### While running

- **D7 — `Kd +`:** increases `Kd` by `0.02`, briefly reverses both motors, then resumes the run.
- **D11 — `Kd -`:** decreases `Kd` by `0.02`, briefly reverses both motors, then resumes the run.
- **D12 — Stop:** stops the motors and returns to idle.

The adjusted `Kd` is saved to EEPROM when the run is stopped if it was changed. The saved `Kd` is loaded at startup. `Kp` and `Ki` are fixed in the sketch and are not saved to EEPROM.

## Line-Loss and U-Turn Behavior

When no sensor detects the line, the firmware stops both motors. If the line remains missing for longer than `LOST_BEFORE_UTURN_MS` (`80 ms`), it spins in place using opposite motor directions. It stops turning when the centre sensor detects the line, provided that detection occurs after the initial 100 ms spin interval.

If the centre sensor does not detect the line within `UTURN_TIMEOUT_MS` (`3000 ms`), the robot stops and returns to idle. This is a **line-reacquisition routine**, not a precise encoder-controlled 180-degree turn; the actual angle depends on traction, speed, and the track layout.

## PID and Tuning Notes

The sketch calculates a weighted sensor error using these weights:

`{3, 2, 1, 0, -2, -3, -4}`

It then calculates the PID correction and applies it differentially to the motors:

- Left motor command = current speed − PID correction
- Right motor command = current speed + PID correction

Current defaults are `Kp = 0.18`, `Kd = 0.040`, and `Ki = 0`. The code uses a scaled sensor error (up to roughly 1000 per sensor), so small gains may be appropriate. Tune on a secured test track, change one parameter at a time, and test at low speed before increasing speed.

If the robot steers away from the line, inspect sensor orientation, motor direction, and the `steerDir` setting. If it consistently pulls to one side, the `leftScale` and `rightScale` values can be used for motor balancing.

## Troubleshooting

- **Robot will not start:** confirm calibration was completed after the latest reset and check the battery, wiring, and motor-driver standby pin.
- **Robot turns the wrong way to correct the line:** check sensor order, motor direction, and `steerDir`.
- **Robot does not reacquire the line:** check the centre sensor, line contrast, calibration range, U-turn direction, and timeout.
- **One motor is weaker:** inspect mechanical friction and battery voltage; adjust `leftScale` or `rightScale` only after checking the hardware.
- **Sensor readings look incorrect:** check sensor height, lighting, wiring, and whether the calibration sweep passed over both line and background.
- **Kd changes do not persist:** ensure the robot is stopped through the normal stop flow after changing `Kd`, and verify EEPROM support on the board.

## Repository Structure

```text
LFT/
└── Working/
    └── Working.ino
```

## License

No license is specified yet. Add a license file if you want to define how others may use, modify, and distribute this project.
