# Arduino PID Line Follower Robot

A compact autonomous line-following robot built using an Arduino, TB6612FNG motor driver, 5-channel IR sensor array and ultrasonic sensors.

The robot uses PID control for smooth line following and includes basic obstacle detection and a servo-based grabber for picking up objects.

## Hardware

* Arduino Nano / compatible AVR board
* TB6612FNG motor driver
* 5-channel analog IR sensor array
* 2 × DC geared motors
* 2 × ultrasonic sensors
* Servo motor with grabber
* LiPo battery

## Main Features

* PID-based line following
* Automatic sensor calibration
* Lost-line recovery
* Obstacle detection and avoidance
* Package detection and pickup
* Adjustable PID and motor speed settings

## Pin Configuration

| Function         |       Pin |
| ---------------- | --------: |
| Motor L BIN1     |        D2 |
| Motor L BIN2     |        D3 |
| Motor L PWM      |        D5 |
| Motor R AIN1     |        D7 |
| Motor R AIN2     |        D8 |
| Motor R PWM      |        D6 |
| TB6612 STBY      |        D4 |
| Left Ultrasonic  | D11 / D12 |
| Right Ultrasonic |  D13 / A5 |
| Buttons          |  D9 / D10 |
| Line Sensors     |     A0–A4 |
| Grabber Servo    |       A6* |

* A6 is analog-only on a standard ATmega328P Nano, so the servo pin may need to be changed depending on the exact board being used.

## PID Settings

The current robot is tuned with:

```cpp
KP = 0.068
KD = 0.12
KI = 0.002
BASE_SPEED = 95
MAX_SPEED = 150
```

These values are specific to the current robot and track setup. They may need to be adjusted for a different build.

## How It Works

On startup, press the calibration button and move the robot across the line so the sensors can learn the track.

After calibration, press the start button. The robot follows the line using PID control and checks the ultrasonic sensors for objects or obstacles.

The code is intentionally kept simple and easy to tune for competition use.

## Project Status

This is a working robotics project developed and tuned for a specific robot build. Hardware, track conditions and sensor placement can affect performance.

## License

MIT License
