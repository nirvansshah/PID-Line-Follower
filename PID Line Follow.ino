#include <Servo.h>

// ================================================================
//  Competition Line Follower
//  Arduino + TB6612FNG + 5 sensor line array + 2 ultrasonic sensors
// ================================================================

// ---------------- Motor driver ----------------
#define BIN1 2
#define BIN2 3
#define PWMB 5

#define AIN1 7
#define AIN2 8
#define PWMA 6

#define STBY 4

// ---------------- Servo ----------------
// Kept exactly as wired on this robot.
//
// IMPORTANT:
// On a classic ATmega328P Arduino Nano, A6 is analog-input-only
// and cannot be used as a normal digital output. The pin is kept
// here because it matches the current robot wiring.
#define SERVO_PIN A6

// ---------------- Buttons ----------------
#define BUTTON1 9
#define BUTTON2 10

// ---------------- Ultrasonic sensors ----------------
#define US_LEFT_TRIG 11
#define US_LEFT_ECHO 12
#define US_RIGHT_TRIG 13
#define US_RIGHT_ECHO A5

// ---------------- Line sensors ----------------
const byte NUM_SENSORS = 5;
const bool IS_BLACK_LINE = true;

// Sensor board is mounted in the reverse direction.
const byte sensorPins[NUM_SENSORS] = {A4, A3, A2, A1, A0};

// Position used by the PID controller.
// Centre sensor = 2000.
const int sensorPosition[NUM_SENSORS] = {
  0, 1000, 2000, 3000, 4000
};

// ---------------- PID tuning ----------------
const float KP = 0.068f;
const float KD = 0.120f;
const float KI = 0.002f;

const int BASE_SPEED = 95;
const int MAX_SPEED = 150;
const int MOTOR_BIAS = 0;
const float SMOOTHING_ALPHA = 0.40f;

// ---------------- Object detection ----------------
const int OBSTACLE_DISTANCE_THRESHOLD = 15;   // cm
const int PACKAGE_DISTANCE_THRESHOLD = 15;    // cm
const int MOVING_OBSTACLE_THRESHOLD = 3;      // cm

// Ultrasonic readings are updated periodically instead of on every
// PID loop. This keeps line following responsive.
const unsigned long ULTRASONIC_INTERVAL = 70;
const unsigned long ULTRASONIC_TIMEOUT = 8000UL;
const unsigned long OBSTACLE_WAIT_TIME = 1000;

// ---------------- Grabber ----------------
const int GRABBER_OPEN = 90;
const int GRABBER_CLOSED = 0;

// ---------------- Package handling ----------------
const int PACKAGE_APPROACH_SPEED = 60;
const unsigned long PACKAGE_APPROACH_TIME = 400;
const unsigned long GRABBER_CLOSE_TIME = 1000;

// ---------------- Obstacle avoidance ----------------
// These timings are for the current robot and track setup.
const unsigned long AVOID_REVERSE_TIME = 300;
const unsigned long AVOID_TURN1_TIME = 300;
const unsigned long AVOID_FORWARD1_TIME = 800;
const unsigned long AVOID_TURN2_TIME = 250;
const unsigned long AVOID_FORWARD2_TIME = 700;
const unsigned long AVOID_TURN3_TIME = 200;
const unsigned long AVOID_FORWARD3_TIME = 900;
const unsigned long LINE_SEARCH_TIMEOUT = 3000;

// ---------------- Debug ----------------
// Leave this false during an actual run.
const bool DEBUG = false;

// ================================================================
//  Robot state
// ================================================================

enum RobotState {
  IDLE,
  CALIBRATING,
  WAITING_TO_START,
  LINE_FOLLOWING,
  HANDLE_OBSTACLE,
  HANDLE_PACKAGE,
  COURSE_FINISHED
};

RobotState currentState = IDLE;

// ================================================================
//  Global variables
// ================================================================

Servo grabberServo;

int minReadings[NUM_SENSORS];
int maxReadings[NUM_SENSORS];
int thresholds[NUM_SENSORS];

// Values from the original robot setup.
// Used only when a sensor does not get enough range during calibration.
const int fallbackThresholds[NUM_SENSORS] = {
  974, 859, 944, 946, 971
};

float error = 0.0f;
float previousError = 0.0f;
float integral = 0.0f;

float smoothedPosition = 2000.0f;
bool haveSmoothedPosition = false;

bool packageCollected = false;

// Cached ultrasonic readings.
long leftDistance = 999;
long rightDistance = 999;
unsigned long lastUltrasonicRead = 0;

// ================================================================
//  Function prototypes
// ================================================================

void motorL(int speedValue);
void motorR(int speedValue);
void stopMotors();

void handleIdle();
void handleCalibration();
void handleWaitToStart();
void handleLineFollowing();
void handlePackagePickup();
void handleObstacle();

void followLine();
int readLinePosition();
void recordSensorMinMax();

void updateUltrasonicReadings();
long getUltrasonicDistance(byte trigPin, byte echoPin);

int checkForObject();
bool wideObstacleDetected();
bool narrowObjectDetected();

void avoidStaticObstacle();

void resetPID();
bool buttonPressed(byte pin);

void debugPrint(const char *msg);

// ================================================================
//  Setup
// ================================================================

void setup() {
  Serial.begin(9600);

  // Faster ADC conversion for the five line sensors.
  // This setting is intended for the 16 MHz AVR board used by this robot.
  ADCSRA = (ADCSRA & 0b11111000) | 0b010;

  // Motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Buttons
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);

  // Ultrasonic sensors
  pinMode(US_LEFT_TRIG, OUTPUT);
  pinMode(US_LEFT_ECHO, INPUT);

  pinMode(US_RIGHT_TRIG, OUTPUT);
  pinMode(US_RIGHT_ECHO, INPUT);

  digitalWrite(US_LEFT_TRIG, LOW);
  digitalWrite(US_RIGHT_TRIG, LOW);

  // Grabber
  grabberServo.attach(SERVO_PIN);
  grabberServo.write(GRABBER_OPEN);

  stopMotors();

  if (DEBUG) {
    Serial.println(F("Robot ready."));
    Serial.println(F("Press BUTTON1 for calibration."));
  }
}

// ================================================================
//  Main loop
// ================================================================

void loop() {

  // Ultrasonic sensors are only needed while the robot is running.
  if (currentState == LINE_FOLLOWING ||
      currentState == HANDLE_OBSTACLE ||
      currentState == HANDLE_PACKAGE) {
    updateUltrasonicReadings();
  }

  switch (currentState) {

    case IDLE:
      handleIdle();
      break;

    case CALIBRATING:
      handleCalibration();
      break;

    case WAITING_TO_START:
      handleWaitToStart();
      break;

    case LINE_FOLLOWING:
      handleLineFollowing();
      break;

    case HANDLE_OBSTACLE:
      handleObstacle();
      break;

    case HANDLE_PACKAGE:
      handlePackagePickup();
      break;

    case COURSE_FINISHED:
      stopMotors();
      digitalWrite(LED_BUILTIN, HIGH);
      break;
  }
}

// ================================================================
//  Button handling
// ================================================================

bool buttonPressed(byte pin) {
  if (digitalRead(pin) != LOW) {
    return false;
  }

  delay(30);

  if (digitalRead(pin) != LOW) {
    return false;
  }

  // Wait until the button is released.
  while (digitalRead(pin) == LOW) {
    delay(2);
  }

  return true;
}

// ================================================================
//  PID reset
// ================================================================

void resetPID() {
  error = 0.0f;
  previousError = 0.0f;
  integral = 0.0f;

  smoothedPosition = 2000.0f;
  haveSmoothedPosition = false;
}

// ================================================================
//  State: IDLE
// ================================================================

void handleIdle() {

  if (!buttonPressed(BUTTON1)) {
    return;
  }

  digitalWrite(LED_BUILTIN, LOW);

  currentState = CALIBRATING;
}

// ================================================================
//  Sensor calibration
// ================================================================

void handleCalibration() {

  digitalWrite(LED_BUILTIN, HIGH);

  for (byte i = 0; i < NUM_SENSORS; i++) {
    minReadings[i] = 1023;
    maxReadings[i] = 0;
  }

  // Rotate one way.
  for (int i = 0; i < 350; i++) {

    motorL(70);
    motorR(-60);

    recordSensorMinMax();

    delay(10);
  }

  // Rotate back the other way.
  for (int i = 0; i < 250; i++) {

    motorL(-60);
    motorR(70);

    recordSensorMinMax();

    delay(10);
  }

  stopMotors();
  delay(150);

  if (DEBUG) {
    Serial.println(F("Calibration values:"));
  }

  for (byte i = 0; i < NUM_SENSORS; i++) {

    int span = maxReadings[i] - minReadings[i];

    // If the sensor did not see enough contrast, use the known
    // values from the original robot setup.
    if (span < 30) {
      thresholds[i] = fallbackThresholds[i];
    }
    else {
      thresholds[i] = minReadings[i] + (span / 2);
    }

    if (DEBUG) {

      Serial.print(F("S"));
      Serial.print(i);

      Serial.print(F(" min="));
      Serial.print(minReadings[i]);

      Serial.print(F(" max="));
      Serial.print(maxReadings[i]);

      Serial.print(F(" thr="));
      Serial.println(thresholds[i]);
    }
  }

  resetPID();

  digitalWrite(LED_BUILTIN, LOW);

  currentState = WAITING_TO_START;
}

// ================================================================
//  Record calibration values
// ================================================================

void recordSensorMinMax() {

  for (byte i = 0; i < NUM_SENSORS; i++) {

    int reading = analogRead(sensorPins[i]);

    if (reading < minReadings[i]) {
      minReadings[i] = reading;
    }

    if (reading > maxReadings[i]) {
      maxReadings[i] = reading;
    }
  }
}

// ================================================================
//  State: waiting to start
// ================================================================

void handleWaitToStart() {

  if (!buttonPressed(BUTTON2)) {
    return;
  }

  resetPID();

  digitalWrite(LED_BUILTIN, LOW);

  currentState = LINE_FOLLOWING;
}

// ================================================================
//  State: line following
// ================================================================

void handleLineFollowing() {

  int objectType = checkForObject();

  // 2 = wide obstacle
  if (objectType == 2) {

    resetPID();

    currentState = HANDLE_OBSTACLE;
    return;
  }

  // 1 = narrow package
  if (objectType == 1 && !packageCollected) {

    resetPID();

    currentState = HANDLE_PACKAGE;
    return;
  }

  followLine();
}

// ================================================================
//  Package pickup
// ================================================================

void handlePackagePickup() {

  stopMotors();
  delay(250);

  // Move closer to the package.
  motorL(PACKAGE_APPROACH_SPEED);
  motorR(PACKAGE_APPROACH_SPEED);

  delay(PACKAGE_APPROACH_TIME);

  stopMotors();
  delay(250);

  // Close the grabber.
  grabberServo.write(GRABBER_CLOSED);

  delay(GRABBER_CLOSE_TIME);

  packageCollected = true;

  resetPID();

  currentState = LINE_FOLLOWING;
}

// ================================================================
//  Obstacle handling
// ================================================================

void handleObstacle() {

  stopMotors();
  delay(300);

  long initialDistance = min(leftDistance, rightDistance);

  // Give a moving obstacle a chance to move away.
  unsigned long startWait = millis();

  while (millis() - startWait < OBSTACLE_WAIT_TIME) {

    updateUltrasonicReadings();
    delay(10);
  }

  long finalDistance = min(leftDistance, rightDistance);

  bool validInitial =
    initialDistance < 900;

  bool validFinal =
    finalDistance < 900;

  bool objectMoved =
    validInitial &&
    validFinal &&
    abs(initialDistance - finalDistance) >
    MOVING_OBSTACLE_THRESHOLD;

  if (objectMoved) {

    if (DEBUG) {
      Serial.println(F("Moving obstacle"));
    }

    // Wait for the path to clear.
    unsigned long clearStart = millis();

    while (millis() - clearStart < 5000) {

      updateUltrasonicReadings();

      if (!wideObstacleDetected()) {
        break;
      }

      stopMotors();

      delay(20);
    }
  }

  else {

    if (DEBUG) {
      Serial.println(F("Static obstacle"));
    }

    avoidStaticObstacle();
  }

  resetPID();

  currentState = LINE_FOLLOWING;
}

// ================================================================
//  PID line following
// ================================================================

void followLine() {

  int position = readLinePosition();

  // Line lost.
  if (position < 0) {

    integral = 0.0f;

    // Turn toward the side where the line was last seen.
    if (error >= 0) {

      motorL(BASE_SPEED);
      motorR(-BASE_SPEED);
    }

    else {

      motorL(-BASE_SPEED);
      motorR(BASE_SPEED);
    }

    return;
  }

  // First valid reading after starting/recovering.
  if (!haveSmoothedPosition) {

    smoothedPosition = position;
    haveSmoothedPosition = true;
  }

  else {

    smoothedPosition =
      (SMOOTHING_ALPHA * position) +
      ((1.0f - SMOOTHING_ALPHA) * smoothedPosition);
  }

  // Centre of the five sensors.
  error = smoothedPosition - 2000.0f;

  float P = error;
  float D = error - previousError;

  // Only integrate near the centre. This stops the integral term
  // from becoming too large during sharp turns.
  if (fabs(error) < 400.0f) {

    integral += error;
  }

  else {

    integral = 0.0f;
  }

  integral = constrain(
    integral,
    -20000.0f,
    20000.0f
  );

  float correction =
    (KP * P) +
    (KI * integral) +
    (KD * D);

  previousError = error;

  int leftSpeed =
    BASE_SPEED +
    (int)correction;

  int rightSpeed =
    BASE_SPEED -
    (int)correction +
    MOTOR_BIAS;

  leftSpeed =
    constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);

  rightSpeed =
    constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  motorL(leftSpeed);
  motorR(rightSpeed);

  if (DEBUG) {

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint > 100) {

      lastPrint = millis();

      Serial.print(F("pos="));
      Serial.print(smoothedPosition);

      Serial.print(F(" err="));
      Serial.print(error);

      Serial.print(F(" corr="));
      Serial.println(correction);
    }
  }
}

// ================================================================
//  Read line position
// ================================================================

int readLinePosition() {

  long weightedSum = 0;
  int activeSensors = 0;

  for (byte i = 0; i < NUM_SENSORS; i++) {

    int reading =
      analogRead(sensorPins[i]);

    bool seesLine;

    if (IS_BLACK_LINE) {

      seesLine =
        reading > thresholds[i];
    }

    else {

      seesLine =
        reading < thresholds[i];
    }

    if (seesLine) {

      weightedSum += sensorPosition[i];

      activeSensors++;
    }
  }

  if (activeSensors == 0) {
    return -1;
  }

  return weightedSum / activeSensors;
}

// ================================================================
//  Ultrasonic updates
// ================================================================

void updateUltrasonicReadings() {

  if (millis() - lastUltrasonicRead <
      ULTRASONIC_INTERVAL) {
    return;
  }

  lastUltrasonicRead = millis();

  leftDistance =
    getUltrasonicDistance(
      US_LEFT_TRIG,
      US_LEFT_ECHO
    );

  // Give the first sensor's echo time to disappear.
  delay(2);

  rightDistance =
    getUltrasonicDistance(
      US_RIGHT_TRIG,
      US_RIGHT_ECHO
    );
}

// ================================================================
//  Ultrasonic distance
// ================================================================

long getUltrasonicDistance(
  byte trigPin,
  byte echoPin
) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long duration =
    pulseIn(
      echoPin,
      HIGH,
      ULTRASONIC_TIMEOUT
    );

  // No echo should mean "nothing detected", not zero distance.
  if (duration == 0) {
    return 999;
  }

  long distance =
    (long)(
      duration * 0.0343f / 2.0f
    );

  // Ignore impossible readings.
  if (distance <= 0 || distance > 400) {
    return 999;
  }

  return distance;
}

// ================================================================
//  Object classification
//
//  0 = clear
//  1 = package
//  2 = wide obstacle
// ================================================================

int checkForObject() {

  if (wideObstacleDetected()) {
    return 2;
  }

  if (narrowObjectDetected()) {
    return 1;
  }

  return 0;
}

// ================================================================
//  Wide object
// ================================================================

bool wideObstacleDetected() {

  bool leftSees =
    leftDistance <
    OBSTACLE_DISTANCE_THRESHOLD;

  bool rightSees =
    rightDistance <
    OBSTACLE_DISTANCE_THRESHOLD;

  return leftSees && rightSees;
}

// ================================================================
//  Narrow object
// ================================================================

bool narrowObjectDetected() {

  bool leftSees =
    leftDistance <
    PACKAGE_DISTANCE_THRESHOLD;

  bool rightSees =
    rightDistance <
    PACKAGE_DISTANCE_THRESHOLD;

  // Exactly one sensor sees it.
  return leftSees != rightSees;
}

// ================================================================
//  Static obstacle avoidance
// ================================================================

void avoidStaticObstacle() {

  // Back away first.
  motorL(-80);
  motorR(-80);
  delay(AVOID_REVERSE_TIME);

  // Turn around the obstacle.
  motorL(-BASE_SPEED);
  motorR(BASE_SPEED);
  delay(AVOID_TURN1_TIME);

  // Move forward.
  motorL(BASE_SPEED);
  motorR(BASE_SPEED);
  delay(AVOID_FORWARD1_TIME);

  // Turn back toward the track.
  motorL(BASE_SPEED);
  motorR(-BASE_SPEED);
  delay(AVOID_TURN2_TIME);

  // Move forward again.
  motorL(BASE_SPEED);
  motorR(BASE_SPEED);
  delay(AVOID_FORWARD2_TIME);

  // Final correction.
  motorL(BASE_SPEED);
  motorR(-BASE_SPEED);
  delay(AVOID_TURN3_TIME);

  // Search for the line while moving forward.
  motorL(BASE_SPEED);
  motorR(BASE_SPEED);

  unsigned long startSearch =
    millis();

  while (millis() - startSearch <
         LINE_SEARCH_TIMEOUT) {

    if (readLinePosition() >= 0) {
      break;
    }

    delay(2);
  }

  stopMotors();
  delay(100);
}

// ================================================================
//  Left motor
// ================================================================

void motorL(int speedValue) {

  speedValue =
    constrain(
      speedValue,
      -255,
      255
    );

  digitalWrite(STBY, HIGH);

  if (speedValue >= 0) {

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    analogWrite(
      PWMB,
      speedValue
    );
  }

  else {

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    analogWrite(
      PWMB,
      -speedValue
    );
  }
}

// ================================================================
//  Right motor
// ================================================================

void motorR(int speedValue) {

  speedValue =
    constrain(
      speedValue,
      -255,
      255
    );

  digitalWrite(STBY, HIGH);

  if (speedValue >= 0) {

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    analogWrite(
      PWMA,
      speedValue
    );
  }

  else {

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    analogWrite(
      PWMA,
      -speedValue
    );
  }
}

// ================================================================
//  Stop both motors
// ================================================================

void stopMotors() {

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// ================================================================
//  Optional debug helper
// ================================================================

void debugPrint(const char *msg) {

  if (DEBUG) {
    Serial.println(msg);
  }
}
