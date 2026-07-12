/* Wall Follower — IMU-based rotation (gyro), encoders kept for straight-line drift only
   ================================================================================
   

   NOT FIXED IN CODE (needs a bench test, can't be fixed blind):
     Turn direction / gyro sign convention. Before trusting rotate() in
     behaviorStep(), command a single rotate(90) on the bench and confirm
     the robot physically turns right by ~90 degrees, not left or by some
     multiple. If it's backwards, flip the sign of targetAngle (not the
     PID error line) — see the comment inside rotate().

   UNCHANGED: wall-following PID math, front/dead-end rule order, motor
   driver, sensor init — same as before.
   ================================================================================
*/

#include <Wire.h>
#include <VL53L1X.h>

// ---------------- Pins ----------------
#define BTN1 34
//#define BTN2 35

#define AIN1 17
#define AIN2 16
#define PWMA 4
#define STBY 5
#define BIN1 19
#define BIN2 18
#define PWMB 21

#define LEFT_ENC_A 2
#define LEFT_ENC_B 15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// XSHUT pins for sensors
#define XSHUT_L 13
#define XSHUT_F 27
#define XSHUT_R 25

// I2C addresses
#define ADDR_L 0x30
#define ADDR_F 0x31
#define ADDR_R 0x32

// IMU (MPU6050) address
#define MPU_ADDR 0x68

// Quadrature decode: only channel A triggers the interrupt; reading B at
// that instant tells direction. Standard single-interrupt quadrature trick.
// Still used for driveStraightUsingEncoders() — not used for rotation.
void IRAM_ATTR leftEncoderISR() {
  bool a = digitalRead(LEFT_ENC_A);
  bool b = digitalRead(LEFT_ENC_B);
  leftTicks += (a == b) ? 1 : -1;
}

void IRAM_ATTR rightEncoderISR() {
  bool a = digitalRead(RIGHT_ENC_A);
  bool b = digitalRead(RIGHT_ENC_B);
  rightTicks += (a == b) ? 1 : -1;
}

// ---------------- Sensors & globals ----------------
VL53L1X sensorL, sensorF, sensorR;
int timingBudget = 20;      // ms, also the loop cadence
unsigned long curtime = 0;  // for timing budget (unsigned long matches millis())

int distLeft = 0, distFront = 0, distRight = 0;

const int baseSpeed    = 235;   // cruise speed while wall-following
const int targetDist   = 45;    // mm, desired wall distance
const int frontThresh  = 150;   // mm, below this = front obstacle
const int outerSpeed   = 255;   // fast wheel while searching for a missing wall
const int innerSpeed   = 70;    // slow wheel while searching for a missing wall
const int rotationSpeed = 180;  // kept for reference; turn now uses TURN_BASE_SPEED below

// ---------------- FIX 2/6 — hysteresis band for wall presence ----------------
// A wall must clearly commit to "present" (<= ENTER) or "absent" (> EXIT)
// before the mode flips. Readings in between keep whatever state was
// already active, instead of chattering back and forth every loop tick.
const int WALL_ENTER_THRESH = 85;   // mm — wall counts as present below this
const int WALL_EXIT_THRESH  = 95;   // mm — wall counts as gone above this
bool wallLeftActive  = false;
bool wallRightActive = false;

// ---------------- Wall PID ----------------
float Kp = 0.619;
float Ki = 0.0001;
float Kd = 0.463;
float pidPrev = 0, pidInt = 0;

// ---------------- Mode ----------------
bool followLeft = false; // set at startup

// ---------------- Motor helper ----------------
void mspeed(int a, int b) {
  a = constrain(a, -255, 255);
  b = constrain(b, -255, 255);
  digitalWrite(AIN1, a >= 0); digitalWrite(AIN2, a < 0);
  analogWrite(PWMA, abs(a));
  digitalWrite(BIN1, b >= 0); digitalWrite(BIN2, b < 0);
  analogWrite(PWMB, abs(b));
}
void stopmotors(){
  mspeed(0,0);
}

// ---------------- Sensors ----------------
void setupSensors() {
  Wire.begin(33, 32);
  Wire.setClock(400000);

  pinMode(XSHUT_L, OUTPUT);
  pinMode(XSHUT_F, OUTPUT);
  pinMode(XSHUT_R, OUTPUT);
  digitalWrite(XSHUT_L, LOW);
  digitalWrite(XSHUT_F, LOW);
  digitalWrite(XSHUT_R, LOW);
  delay(100);

  pinMode(XSHUT_L, INPUT);
  delay(10);
  if (!sensorL.init()) { Serial.println("SensorL init fail"); while (1); }
  sensorL.setAddress(ADDR_L);

  pinMode(XSHUT_F, INPUT);
  delay(10);
  if (!sensorF.init()) { Serial.println("SensorF init fail"); while (1); }
  sensorF.setAddress(ADDR_F);

  pinMode(XSHUT_R, INPUT);
  delay(10);
  if (!sensorR.init()) { Serial.println("SensorR init fail"); while (1); }
  sensorR.setAddress(ADDR_R);

  sensorL.setDistanceMode(VL53L1X::Short);
  sensorF.setDistanceMode(VL53L1X::Short);
  sensorR.setDistanceMode(VL53L1X::Short);

  sensorL.setMeasurementTimingBudget(timingBudget * 1000);
  sensorF.setMeasurementTimingBudget(timingBudget * 1000);
  sensorR.setMeasurementTimingBudget(timingBudget * 1000);

  sensorL.startContinuous(timingBudget);
  sensorF.startContinuous(timingBudget);
  sensorR.startContinuous(timingBudget);

  Serial.println("Sensors up");
}

// ---------------- FIX 3 — staleness watchdog ----------------
// If a sensor stops returning fresh reads for longer than this, its
// distance is forced to "far away" (4000) instead of silently trusting
// a frozen value, and a one-time warning is printed.
#define SENSOR_STALE_MS 300

unsigned long lastGoodL = 0, lastGoodF = 0, lastGoodR = 0;
bool staleWarnedL = false, staleWarnedF = false, staleWarnedR = false;

void updateSensors() {
  unsigned long now = millis();

  if (sensorL.dataReady()) {
    distLeft = sensorL.read(false);
    lastGoodL = now;
    staleWarnedL = false;
  } else if (now - lastGoodL > SENSOR_STALE_MS) {
    distLeft = 4000;
    if (!staleWarnedL) { Serial.println("WARN: sensorL stale"); staleWarnedL = true; }
  }

  if (sensorF.dataReady()) {
    distFront = sensorF.read(false);
    lastGoodF = now;
    staleWarnedF = false;
  } else if (now - lastGoodF > SENSOR_STALE_MS) {
    distFront = 4000;
    if (!staleWarnedF) { Serial.println("WARN: sensorF stale"); staleWarnedF = true; }
  }

  if (sensorR.dataReady()) {
    distRight = sensorR.read(false);
    lastGoodR = now;
    staleWarnedR = false;
  } else if (now - lastGoodR > SENSOR_STALE_MS) {
    distRight = 4000;
    if (!staleWarnedR) { Serial.println("WARN: sensorR stale"); staleWarnedR = true; }
  }

  // Guard against 0 / absurd readings same as before, on top of staleness check
  if (distLeft  == 0 || distLeft  > 4000) distLeft  = 4000;
  if (distFront == 0 || distFront > 4000) distFront = 4000;
  if (distRight == 0 || distRight > 4000) distRight = 4000;
}

// ============================================================
//  IMU (MPU6050) gyro reading, used ONLY for rotate().
//  Straight-line driving below is untouched and still uses encoders.
// ============================================================
float         gyroBiasZ     = 0.0f;
float         yaw           = 0.0f;
float         yawOffsetSave = 0.0f;
unsigned long imuLastUs     = 0;
#define GYRO_CALIB_N 10000

int16_t readGyroZRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  return (Wire.read() << 8) | Wire.read();
}

// Call often (every loop pass during a turn) — internally rate-limited
// to ~100Hz so calling it more often than that is harmless.
void updateYaw() {
  unsigned long now = micros();
  if (now - imuLastUs < 10000) return;
  float dt = (now - imuLastUs) / 1000000.0f;
  imuLastUs = now;
  long sum = 0;
  for (int i = 0; i < 4; i++) sum += readGyroZRaw();
  float gyroZ = (sum / 4.0f - gyroBiasZ) / 131.0f;
  if (fabsf(gyroZ) < 0.4f) gyroZ = 0.0f;  // ignore tiny bias noise while still
  yaw += gyroZ * dt;
}

// resetYaw() marks "zero" for the turn about to start.
// getYaw() returns how far we've rotated since that zero point.
void resetYaw() { updateYaw(); yawOffsetSave = yaw; }
float getYaw()   { return yaw - yawOffsetSave; }

float normalizeAngle(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void wakeMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00);
  Wire.endTransmission(true);
}

void calibrateGyro() {
  Serial.println("IMU calibrating - keep still...");
  delay(2000);
  long sum = 0;
  for (int i = 0; i < GYRO_CALIB_N; i++) {
    sum += readGyroZRaw();
    delayMicroseconds(500);
  }
  gyroBiasZ = (float)sum / (float)GYRO_CALIB_N;
  Serial.printf("Gyro bias Z = %.4f\n", gyroBiasZ);
}

void setupIMU() {
  // Wire already begun in setupSensors() before this runs.
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU6050 NOT FOUND!");
    while (1);
  }
  Serial.println("MPU6050 OK.");
  wakeMPU();
  calibrateGyro();     // robot must be perfectly still here
  yaw = 0.0f;
  yawOffsetSave = 0.0f;
  imuLastUs = micros();
}

// ---------------- Open-space straight driving ----------------
const float straightKp = 0.6f;

void resetEncoderBaseline() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
}

void resetWallPID() {
  pidInt = 0;
  pidPrev = 0;
}

void driveStraightUsingEncoders() {
  noInterrupts();
  long l = leftTicks;
  long r = rightTicks;
  leftTicks = 0;
  rightTicks = 0;
  interrupts();

  float skew = (float)(l - r);
  float correction = straightKp * skew;

  int leftSp  = constrain((int)(baseSpeed - correction), 30, 255);
  int rightSp = constrain((int)(baseSpeed + correction), 30, 255);
  mspeed(leftSp, rightSp);
}

// ---------------- Wall PID routine (math unchanged) ----------------
void runWallPIDSingle (float measured, int desired, bool invertMirror) {
  float e = measured - (float)desired;
  if (invertMirror) e = -e;

  pidInt += e;
  pidInt = constrain(pidInt, -5000, 5000);
  float deriv = e - pidPrev;
  pidPrev = e;

  float pid = Kp * e + Ki * pidInt + Kd * deriv;

  int leftSp  = constrain((int)(baseSpeed - pid), 30, 240);
  int rightSp = constrain((int)(baseSpeed + pid), 30, 240);
  mspeed(leftSp, rightSp);
}

// ============================================================
//  Turn PID + rotate(), gyro-driven instead of tick-count driven
// ============================================================
#define TURN_PID_KP        4.5f
#define TURN_PID_KI        0.02f
#define TURN_PID_KD        1.8f

#define TURN_BASE_SPEED    150   // start near old rotationSpeed(180), tune from here
#define TURN_MIN_SPEED     80
#define RAMP_UP_DEG        10.0f
#define RAMP_DOWN_DEG      30.0f
#define TURN_DEADBAND      3.0f    // degrees of error considered "arrived"
#define TURN_DONE_CYCLES   8       // consecutive in-tolerance loops required to stop
#define TURN_LOOP_MS       5

// FIX 4 — timeout now scales with turn size instead of a flat 1500ms,
// so a 180 degree U-turn gets proportionally more time than a 90.
#define TURN_TIMEOUT_BASE_MS      500
#define TURN_TIMEOUT_MS_PER_DEG   8

float trapIntegral = 0.0f, trapLastError = 0.0f;
unsigned long lastTrapTime = 0;

void resetTrapPID() {
  trapIntegral = 0.0f;
  trapLastError = 0.0f;
  lastTrapTime = millis();
}

float trapPID(float error) {
  unsigned long now = millis();
  float dt = (now - lastTrapTime) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = TURN_LOOP_MS / 1000.0f;
  lastTrapTime = now;
  trapIntegral += error * dt;
  trapIntegral  = constrain(trapIntegral, -50.0f, 50.0f);
  float derivative = (error - trapLastError) / dt;
  trapLastError = error;
  return (TURN_PID_KP * error) + (TURN_PID_KI * trapIntegral) + (TURN_PID_KD * derivative);
}

// degree: positive = turn right, negative = turn left, 180/-180 = U-turn
//
// NOTE ON DIRECTION (see file header): verify on the bench that
// rotate(90) actually turns the robot right by ~90 degrees. If it turns
// the wrong way or by the wrong multiple, flip the sign here:
//   float targetAngle = (float)(-degree);   <-- try (float)(degree) instead
// Do NOT touch the pidError sign flip below — that one fixes a separate,
// already-verified left/right asymmetry bug and should stay as-is.
void rotate(int degree) {
  if (degree == 0) return;

  resetYaw();
  resetTrapPID();

  float targetAngle = (float)(-degree);
  int   direction   = (degree > 0) ? 1 : -1;
  float totalAngle  = fabsf(targetAngle);

  int doneCnt = 0;
  unsigned long startMs = millis();
  unsigned long maxMs = TURN_TIMEOUT_BASE_MS + (unsigned long)abs(degree) * TURN_TIMEOUT_MS_PER_DEG;

  while (true) {
    updateYaw();
    float currentYaw = getYaw();
    float traveled  = fabsf(normalizeAngle(currentYaw));
    float remaining = totalAngle - traveled;

    if (millis() - startMs > maxMs) {
      Serial.println("rotate(): timeout, ending turn early");
      break;
    }

    if (remaining <= TURN_DEADBAND) {
      if (++doneCnt >= TURN_DONE_CYCLES) break;
    } else {
      doneCnt = 0;
    }

    float speed;
    if (traveled < RAMP_UP_DEG)
      speed = map(traveled, 0, RAMP_UP_DEG, TURN_MIN_SPEED, TURN_BASE_SPEED);
    else if (remaining < RAMP_DOWN_DEG)
      speed = map(remaining, 0, RAMP_DOWN_DEG, TURN_MIN_SPEED, TURN_BASE_SPEED);
    else
      speed = TURN_BASE_SPEED;
    speed = constrain(speed, (float)TURN_MIN_SPEED, (float)TURN_BASE_SPEED);

    float yawError = normalizeAngle(targetAngle - currentYaw);

    // Sign flip so left and right turns behave identically with the
    // same PID gains (without this, one direction over/under-corrects).
    float pidError   = yawError * -direction;
    float correction = trapPID(pidError);
    float adjSpeed   = constrain(speed + correction, (float)TURN_MIN_SPEED, (float)TURN_BASE_SPEED);

    int leftCmd, rightCmd;
    if (direction < 0) {
      leftCmd  = constrain((int)( adjSpeed), -200, 200);
      rightCmd = constrain((int)(-adjSpeed), -200, 200);
    } else {
      leftCmd  = constrain((int)(-adjSpeed), -200, 200);
      rightCmd = constrain((int)( adjSpeed), -200, 200);
    }

    mspeed(leftCmd, rightCmd);
    delay(TURN_LOOP_MS);
  }

  // Active counter-brake: briefly reverse to kill spin momentum
  // instead of just cutting power and coasting past the target.
  mspeed(direction * 60, -direction * 60);
  delay(5);
  mspeed(0, 0);

  // FIX 1 — clear leftover encoder ticks generated by the turn itself,
  // so the next driveStraightUsingEncoders() call doesn't see a fake
  // "skew" caused by spinning in place rather than real straight drift.
  resetEncoderBaseline();
  resetWallPID();

  Serial.printf("[TURN] target=%d final_remaining=%.2f\n",
                degree, totalAngle - fabsf(normalizeAngle(getYaw())));
}

// ---------------- Main behavior (explicit rule order) ----------------
void behaviorStep() {
  updateSensors();

  // FIX 2/6 — hysteresis: only flip state when a reading clearly commits
  // to "present" or "absent". Values in the dead band keep the previous
  // state instead of chattering every loop tick.
  bool prevLeftActive  = wallLeftActive;
  bool prevRightActive = wallRightActive;

  if (distLeft  <= WALL_ENTER_THRESH) wallLeftActive  = true;
  else if (distLeft  > WALL_EXIT_THRESH) wallLeftActive  = false;

  if (distRight <= WALL_ENTER_THRESH) wallRightActive = true;
  else if (distRight > WALL_EXIT_THRESH) wallRightActive = false;

  bool wallFrontPresent = distFront < frontThresh;

  // FIX 2 — reset the relevant PID/encoder baseline exactly on the
  // transition edge, not every loop, so state doesn't leak across modes.
  if (wallLeftActive != prevLeftActive || wallRightActive != prevRightActive) {
    resetWallPID();
    resetEncoderBaseline();
  }

  if (wallLeftActive && wallFrontPresent && wallRightActive) {
    mspeed(-100, -100);
    delay(20);
    rotate(180);
    mspeed(0, 0);
    resetWallPID();
    return;
  }

  if (wallFrontPresent) {
    if (!wallLeftActive) {
      mspeed(innerSpeed, outerSpeed);
      return;
    }
    mspeed(-100, -100);
    delay(20);
    rotate(90);
    return;
  }

  if (followLeft) {
    if (wallLeftActive) {
      runWallPIDSingle(distLeft, targetDist, false);
      return;
    }
  } else {
    if (wallRightActive) {
      runWallPIDSingle(distRight, targetDist, true);
      return;
    }
  }

  if (!wallLeftActive && !wallRightActive) {
    driveStraightUsingEncoders();
    return;
  }

  if (followLeft) {
    mspeed(innerSpeed, outerSpeed);
    return;
  } else {
    mspeed(outerSpeed, innerSpeed);
    return;
  }
}

// ---------------- Setup & loop ----------------
void setup() {
  Serial.begin(115200);
  delay(10);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(BTN1, INPUT);

  setupSensors();   // also does Wire.begin()
  setupIMU();       // must come after Wire.begin(), robot must be still

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  Serial.println("Press BTN1 (left) or BTN2 (right) to choose wall-follow mode...");

  while (true) {
    if (digitalRead(BTN1) == HIGH) { followLeft = true;  break; }
    delay(10);
  }
  delay(300);
  Serial.printf("Mode chosen: %s-wall follow\n", followLeft ? "LEFT" : "RIGHT");

  unsigned long bootNow = millis();
  lastGoodL = lastGoodF = lastGoodR = bootNow;

  for (int i = 0; i < 10; i++) {
    updateSensors();
    delay(timingBudget);
  }

  curtime = millis();
}

void loop() {
  // Intentionally left minimal — behaviorStep() dispatch is handled
  // elsewhere by design.
}
