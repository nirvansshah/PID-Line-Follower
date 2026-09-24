# PID Line Follower

# Arduino Autonomous Line Following Robot

An Arduino-based autonomous line-following robot designed for robotics competitions and educational experimentation.

The robot combines **PID line following**, **ultrasonic object detection**, **obstacle avoidance**, and a **servo-operated grabber** to navigate a marked course and interact with objects placed on the track.

## Features

* PID-based line following
* 5-channel analog IR line sensor array
* Automatic sensor calibration
* Rear-facing sensor orientation compensation
* Adjustable PID parameters
* Adjustable motor speed and bias
* TB6612FNG dual motor driver
* Dual ultrasonic sensors for object detection
* Package detection and pickup using a servo grabber
* Static obstacle avoidance
* Moving-obstacle detection
* Button-controlled calibration and start sequence
* Serial debugging output

## Robot Architecture

The robot operates using a simple state machine:

```text
IDLE
  ↓
CALIBRATING
  ↓
WAITING_TO_START
  ↓
LINE_FOLLOWING
  ├── Package detected → HANDLE_PACKAGE
  └── Obstacle detected → HANDLE_OBSTACLE
                               ↓
                        LINE_FOLLOWING
```

The main states used by the program are:

```cpp
enum RobotState {
  IDLE,
  CALIBRATING,
  WAITING_TO_START,
  LINE_FOLLOWING,
  HANDLE_OBSTACLE,
  HANDLE_PACKAGE,
  COURSE_FINISHED
};
```

## Hardware

| Component                        | Description                   |
| -------------------------------- | ----------------------------- |
| Arduino                          | Main microcontroller          |
| TB6612FNG                        | Dual DC motor driver          |
| 5-channel analog IR sensor array | Line detection                |
| 2 × DC geared motors             | Left and right drive          |
| 2 × Ultrasonic sensors           | Object/obstacle detection     |
| Servo motor                      | Grabber mechanism             |
| Push buttons                     | Calibration and start control |
| Battery                          | Robot power source            |

## Pin Configuration

### TB6612FNG Motor Driver

| Function | Arduino Pin |
| -------- | ----------: |
| BIN1     |          D2 |
| BIN2     |          D3 |
| PWMB     |          D5 |
| AIN1     |          D7 |
| AIN2     |          D8 |
| PWMA     |          D6 |
| STBY     |          D4 |

The code maps the TB6612 channels as:

* **Motor L** → TB6612 Motor B channel
* **Motor R** → TB6612 Motor A channel

### Line Sensors

The robot uses five analog sensors:

| Sensor Index | Arduino Pin |
| ------------ | ----------: |
| 0            |          A4 |
| 1            |          A3 |
| 2            |          A2 |
| 3            |          A1 |
| 4            |          A0 |

The sensor order is intentionally reversed:

```cpp
int sensorPins[NUM_SENSORS] = { A4, A3, A2, A1, A0 };
```

This compensates for the sensor array orientation used by the robot.

### Ultrasonic Sensors

| Sensor | Trigger | Echo |
| ------ | ------: | ---: |
| Left   |     D11 |  D12 |
| Right  |     D13 |   A5 |

### Controls

| Function | Arduino Pin |
| -------- | ----------: |
| Button 1 |          D9 |
| Button 2 |         D10 |

Button 1 starts calibration.

Button 2 starts the autonomous run after calibration.

## Servo / Grabber

The grabber is controlled using the Arduino Servo library.

Current positions:

```cpp
const int GRABBER_OPEN = 90;
const int GRABBER_CLOSED = 0;
```

The robot starts with the grabber open.

When a package is detected, the robot:

1. Stops.
2. Moves forward slowly.
3. Stops closer to the package.
4. Closes the grabber.
5. Marks the package as collected.
6. Resumes line following.

> **Hardware note:** On a classic ATmega328P Arduino Nano, A6 is analog-input-only. The current `SERVO_PIN A6` assignment therefore needs to be changed to a usable digital pin for a standard Nano.

## PID Line Following

The robot calculates the position of the detected line and uses PID control to adjust the left and right motor speeds.

Current PID values:

```cpp
const float KP = 0.068;
const float KD = 0.12;
const float KI = 0.002;
```

Base and maximum speeds:

```cpp
const int BASE_SPEED = 95;
const int MAX_SPEED = 150;
```

Motor correction is calculated from:

* Proportional error
* Integral error
* Change in error

Conceptually:

```text
Left Motor  = Base Speed + PID Correction
Right Motor = Base Speed - PID Correction
```

The integral term is limited to prevent excessive accumulation.

## Line Detection

The robot uses five analog sensors to determine where the line is located.

For a black line:

```cpp
const bool IS_BLACK_LINE = true;
```

The code compares each sensor reading with its calibrated threshold.

The five sensor positions are mapped approximately as:

```text
0    1    2    3    4
|    |    |    |    |
---- line position ----
```

The center position corresponds to approximately:

```cpp
2000
```

PID error is calculated as:

```cpp
error = lastPosition - 2000;
```

## Sensor Calibration

Calibration starts when **Button 1** is pressed.

The robot rotates in both directions while recording the minimum and maximum values from every line sensor.

For each sensor:

```cpp
threshold = (minimum + maximum) / 2;
```

This allows the robot to adapt to the actual track and sensor readings instead of relying entirely on fixed thresholds.

During calibration, make sure the sensors pass over both the line and the surrounding track surface.

## Object Detection

Two ultrasonic sensors are used to detect objects in front of the robot.

The detection threshold is currently:

```cpp
const int OBSTACLE_DISTANCE_THRESHOLD = 15;
const int PACKAGE_DISTANCE_THRESHOLD = 15;
```

The currently active classification logic is:

### Both sensors detect an object

The object is treated as a **wide obstacle**.

```text
Left sensor   → Object
Right sensor  → Object

Result → OBSTACLE
```

### Only one sensor detects an object

The object is treated as a **narrow package**.

```text
Left sensor   → Object
Right sensor  → Clear

Result → PACKAGE
```

The same logic applies when only the right sensor detects the object.

## Obstacle Handling

When a wide obstacle is detected, the robot enters:

```cpp
HANDLE_OBSTACLE
```

The robot first determines whether the obstacle is moving by comparing ultrasonic measurements before and after a short waiting period.

If the obstacle appears to be moving, the robot waits until the path is clear.

For a static obstacle, the robot performs a predefined avoidance maneuver involving:

* Reverse movement
* Turning
* Forward movement
* Additional turning
* Searching for the line again

Once the line is detected, normal PID line following resumes.

## Lost-Line Recovery

When none of the five sensors detects the line, the robot enters a recovery behavior based on the previous error.

The robot pivots in the appropriate direction until the line is detected again.

This allows the robot to recover from temporary line loss during sharp turns or disturbances.

## Startup Sequence

The normal operating sequence is:

### 1. Power On

The robot starts in:

```text
IDLE
```

### 2. Press Button 1

Sensor calibration begins.

```text
IDLE
 ↓
CALIBRATING
```

### 3. Calibration Complete

The robot stops and waits for the second button.

```text
WAITING_TO_START
```

### 4. Press Button 2

Autonomous operation begins.

```text
LINE_FOLLOWING
```

## Serial Monitoring

Serial communication is initialized at:

```cpp
Serial.begin(9600);
```

The serial monitor provides useful debugging information including:

* Robot state
* Sensor calibration values
* PID values
* Error
* Integral value
* Derivative value
* PID correction
* Object detection
* Obstacle handling

Open the Arduino Serial Monitor at:

```text
9600 baud
```

## PID Tuning

PID values are highly dependent on the physical robot.

Factors affecting performance include:

* Motor speed
* Motor mismatch
* Sensor height
* Sensor spacing
* Robot weight
* Battery voltage
* Track surface
* Line width
* Robot geometry

A practical tuning process is:

```text
1. Start with low speed
2. Tune KP
3. Tune KD
4. Add a small KI if required
5. Increase speed gradually
```

Symptoms can help identify tuning problems:

| Behaviour                 | Possible Adjustment     |
| ------------------------- | ----------------------- |
| Robot responds too slowly | Increase KP             |
| Robot oscillates heavily  | Reduce KP / increase KD |
| Robot overshoo            |                         |
