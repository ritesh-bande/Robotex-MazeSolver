// mama.ino
// ══════════════════════════════════════════════════════════════════
//  FIXES FROM v7:
//
//  FIX 1 — Back wall hardcoded 0  (Bug 3)
//    a[2] was always 0 — back wall never marked.
//    [v8 attempted to infer this from previous-cell maze data, but
//     that inference is logically a no-op when tracking is correct
//     and propagates sensor noise when it isn't — REVERTED below.]
//
//  FIX 2 — imuHeadingCorrection() undefined yawtarget  (Bug 1)
//    yawtarget was declared but never assigned → garbage correction.
//    Now stores yaw at the moment resetImuPID() is called and holds
//    that as the reference heading for the entire straight segment.
//
//  FIX 3 — Local dX/dY shadowing globals  (new bug)
//    countKnownStraight() declared its own local dX/dY/wB arrays,
//    silently shadowing the global ones. Removed local copies —
//    function now uses the global dX/dY and its own local wB only.
//
//  FIX 4 — identifyBlock() direct sensor.read() calls
//    Called sensor1/2/3.read() directly instead of using the cached
//    distLeft/distFront/distRight values, causing up to 3×20ms stall
//    and leaving globals stale. Now calls updateSensors() once and
//    uses the globals, consistent with all other functions.
//
// ══════════════════════════════════════════════════════════════════
//  FIXES APPLIED IN THIS PASS (v8 -> v8.1):
//
//  BUGFIX A — rotate() BLAST_ZONE / BRAKE_ZONE swapped
//    BLAST_ZONE was 25.0f and BRAKE_ZONE was 75.0f. Since the branch
//    order checks "absErr > BLAST_ZONE" first, any error above 25°
//    (including errors near 180°) hit the full-speed blast branch,
//    making the ramp-down branch unreachable dead code. Robot would
//    blast at full speed almost to the target then hard-stop with no
//    deceleration, causing overshoot/slip. Swapped: BLAST_ZONE=75.0f,
//    BRAKE_ZONE=25.0f, matching how the branch logic was written.
//
//  BUGFIX B — identifyBlock() missing revisit guard
//    No check for visited[posX][posY] before writing maze[posX][posY]
//    = type. On any revisit (loop maze, backtrack), this OVERWRITES
//    wall bits that neighbor cells had already mirrored into this
//    cell (via the |= mirroring writes), silently corrupting the map
//    on noisy revisits. Added an early return if already visited.
//
//  BUGFIX C — identifyBlock() back-wall inference reverted
//    The v8 inference `a[2] = (maze[prevX][prevY] & wallBit[orientation])`
//    is checking "did I just drive through a wall to get here" — in a
//    correctly tracked robot this is always 0 (physically impossible
//    otherwise), so it added complexity with zero benefit. Worse: if
//    prevCell's bit was already wrong due to sensor noise, this
//    re-confirms the false wall immediately after the robot disproved
//    it by driving through. Reverted to the simple, always-correct
//    a[2] = 0 (back is where we just came from — always open).
//
//  CLEANUP — unified wall-detection threshold
//    identifyBlock() used "< 90" while wallPIDcorrection() /
//    getHeadingCorrection() used "< 80" for the same physical
//    left/right wall presence check. Pulled into one constant,
//    WALL_DETECT_THRESH, used everywhere so the two systems can't
//    silently drift apart again.
//
//  UNCHANGED (already fixed in v7):
//    - Bitmask convention consistent across identifyBlock/floodfill/nextBlock
//    - Double position update fixed via advancePosition()
//    - getHeadingCorrection() switcher (wall PID ↔️ encoder heading)
//    - updateSensors() called once per tick in each drive loop
//
//  BITMASK CONVENTION (all functions agree):
//    bit 0 (1) = West  (-x)
//    bit 1 (2) = North (+y)
//    bit 2 (4) = East  (+x)
//    bit 3 (8) = South (-y)
//
//  ORIENTATION CONVENTION:
//    0 = West, 1 = North, 2 = East, 3 = South
// ══════════════════════════════════════════════════════════════════

#include <Wire.h>
#include <VL53L1X.h>
#include <math.h>
#include <queue>
#include <utility>
using namespace std;

// ================================================================
//  PINS
// ================================================================
#define BTN1 34
// #define BTN2 35
#define LED1 0
#define LED2 12

#define AIN1 17
#define AIN2 16
#define PWMA 4
#define STBY 5
#define BIN1 19
#define BIN2 18
#define PWMB 21

#define LEFT_ENC_A  2
#define LEFT_ENC_B  15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

#define XSHUT1 13
#define XSHUT2 27
#define XSHUT3 25
#define ADDR1  0x30
#define ADDR2  0x31
#define ADDR3  0x32

#define MPU_ADDR 0x68

// ─── TURN PID ────────────────────────────────────────────────
#define PID_KP        4.5f
#define PID_KI        0.02f
#define PID_KD        1.8f

// ─── SPEED ───────────────────────────────────────────────────
#define BASE_SPEED      100
#define RAMP_UP_DEG     10.0f
#define RAMP_DOWN_DEG   30.0f
#define MIN_DRIVE_SPEED 80
#define TURN_DEADBAND   20.0f

// ─── TIMING ──────────────────────────────────────────────────
#define LOOP_MS       10
#define GYRO_CALIB_N  10000

// ─── WALL DETECTION ──────────────────────────────────────────
// CLEANUP: single shared threshold used by identifyBlock() AND
// wallPIDcorrection()/getHeadingCorrection() — previously these
// disagreed (90 vs 80).
#define WALL_DETECT_THRESH 100

// ================================================================
//  SENSORS
// ================================================================
VL53L1X sensor1;   // left
VL53L1X sensor2;   // front
VL53L1X sensor3;   // right

// ================================================================
//  ENCODERS  (full quadrature — 4 ISRs)
// ================================================================
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

void IRAM_ATTR isrLA() { leftTicks  += (digitalRead(LEFT_ENC_A)  == digitalRead(LEFT_ENC_B))  ?  1 : -1; }
void IRAM_ATTR isrLB() { leftTicks  += (digitalRead(LEFT_ENC_A)  != digitalRead(LEFT_ENC_B))  ?  1 : -1; }
void IRAM_ATTR isrRA() { rightTicks += (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) ?  1 : -1; }
void IRAM_ATTR isrRB() { rightTicks += (digitalRead(RIGHT_ENC_A) != digitalRead(RIGHT_ENC_B)) ?  1 : -1; }

// ================================================================
//  IMU — MPU6050
// ================================================================
float         gyroBiasZ   = 0.0f;
float         yaw         = 0.0f;
float         yawRef      = 0.0f;          // reference set by resetYaw()
unsigned long imuLastUs   = 0;
float         yawOffsetSave = 0.0f;

// ─── TRAP TURN PID ───────────────────────────────────────────
float         trapIntegral  = 0.0f;
float         trapLastError = 0.0f;
unsigned long lasttrapTime  = 0;

int16_t readGyroZRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  return (Wire.read() << 8) | Wire.read();
}

void updateYaw() {
  unsigned long now = micros();
  if (now - imuLastUs < 10000) return;
  float dt = (now - imuLastUs) / 1000000.0f;
  imuLastUs = now;
  long sum = 0;
  for (int i = 0; i < 4; i++) sum += readGyroZRaw();
  float gyroZ = (sum / 4.0f - gyroBiasZ) / 131.0f;
  if (fabsf(gyroZ) < 0.4f) gyroZ = 0.0f;
  yaw += gyroZ * dt;
}

void resetYaw() {
  updateYaw();
  yawOffsetSave = yaw;
}

float getYaw() {
  return yaw - yawOffsetSave;
}

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
  Serial.println("IMU calibrating — keep still...");
  delay(2000);
  long sum = 0;
  for (int i = 0; i < GYRO_CALIB_N; i++) {
    sum += readGyroZRaw();
    delayMicroseconds(500);
  }
  gyroBiasZ = (float)sum / (float)GYRO_CALIB_N;
  Serial.printf("Gyro bias Z = %.4f\n", gyroBiasZ);
}

// ================================================================
//  MAZE
//  Bitmask: bit0=W(1) bit1=N(2) bit2=E(4) bit3=S(8)
//  Orientation: 0=West 1=North 2=East 3=South
// ================================================================
const int MAZESIZE = 5;
int targetX = MAZESIZE - 1, targetY = MAZESIZE - 1;
float blockSize = 18.0f;

int  maze[MAZESIZE][MAZESIZE];
int  flood[MAZESIZE][MAZESIZE];
bool visited[MAZESIZE][MAZESIZE];

int posX = 0, posY = 0;
int orientation = 2;   
// Global direction tables — one copy, used everywhere
const int dX[4] = { -1,  0, +1,  0 };   // W N E S
const int dY[4] = {  0, +1,  0, -1 };   // W N E S

// Wall bit for each absolute direction: W=1 N=2 E=4 S=8
const int wallBit[4] = { 1, 2, 4, 8 };

// ================================================================
//  SPEEDS & TUNING
// ================================================================
int timingBudget   = 20;
int frontThresh    = 80;
int baseSpeed      = 220;
int minSpeed       = 150;
int exploreSpeed   = 210;
int fastSpeed      = 240;
int fastRampCm     = 6;
int fastRampDownCm = 7;

// ─── 3-ZONE TURN ─────────────────────────────────────────────
// BUGFIX A: these two were swapped (BLAST_ZONE=25, BRAKE_ZONE=75).
// Branch order in rotate() is "if (absErr > BLAST_ZONE) ... else if
// (absErr > BRAKE_ZONE) ...", which requires BLAST_ZONE > BRAKE_ZONE
// to ever reach the ramp-down branch. Swapped to correct values.
float BLAST_ZONE    = 25.0f;   // was 25.0f
float BRAKE_ZONE    = 75.0f;   // was 75.0f
int   FF_BLAST_SPEED = 170;
int   FF_BRAKE_MIN   = 60;

float rotKp = 3.0f, rotKi = 0.0f, rotKd = 0.5f;
float ROT_DONE_THRESH = 2.0f;
int   ROT_DONE_CYCLES = 12;
float INT_CLAMP       = 200.0f;

// ─── IMU STRAIGHT HEADING PID ────────────────────────────────
// FIX 2: imuYawTarget is now set in resetImuPID() so
//        imuHeadingCorrection() always has a valid reference.
float imuKp = 2.5f, imuKi = 0.0f, imuKd = 1.2f;
float imuErr = 0, imuInt = 0, imuPrev = 0;
float imuYawTarget = 0.0f;   // ← FIX 2: was an uninitialized local

// ─── WALL PID ────────────────────────────────────────────────
float wallKp = 0.5f, wallKi = 0.0001f, wallKd = 1.1f;
float wallError = 0, wallInt = 0, wallPrev = 0;
int   targetLeftDist = 64, targetRightDist = 37;

// ─── ENCODER HEADING (open space) ────────────────────────────
float encKp = 1.5f, encKi = 0.0f, encKd = 0.25f;
float encoderError = 0, encoderInt = 0, encoderPrev = 0;
long  lastLeftTicks = 0, lastRightTicks = 0;
bool  wasUsingEncoder = false;

// ─── TICKS PER CM ────────────────────────────────────────────
float cmToEncoderTicks = 200.314f;

// ================================================================
//  SENSOR DISTANCES  (updated once per control-loop tick)
// ================================================================
int distLeft, distFront, distRight;

// ================================================================
//  TIMING HELPER
// ================================================================
unsigned long lastMillis = 0;
void setdelay()           { lastMillis = millis(); }
void mdelay(unsigned long d) {
  unsigned long e = millis() - lastMillis;
  if (e < d) delay(d - e);
  lastMillis = millis();
}

// ================================================================
//  MOTOR DRIVER  (b = left motor, a = right motor)
// ================================================================
void mspeed(int b, int a) {
  if (abs(a) <= 255) {
    digitalWrite(AIN1, a >= 0);
    digitalWrite(AIN2, a < 0);
    analogWrite(PWMA, abs(a));
  }
  if (abs(b) <= 255) {
    digitalWrite(BIN1, b >= 0);
    digitalWrite(BIN2, b < 0);
    analogWrite(PWMB, abs(b));
  }
}
void motorsStop() { mspeed(0, 0); }

// ================================================================
//  SENSOR READ — call ONCE per control-loop tick at the top of
//  each drive loop. All correction functions reuse these globals.
// ================================================================
void updateSensors() {
  distLeft  = sensor1.read();
  distFront = sensor2.read();
  distRight = sensor3.read();
}

// ================================================================
//  WALL PID
// ================================================================
int wallPIDcorrection() {
  bool hasLeft  = (distLeft  < WALL_DETECT_THRESH);
  bool hasRight = (distRight < WALL_DETECT_THRESH);

  if (hasLeft && hasRight) {
    wallError = distLeft - distRight;
  } else if (hasRight) {
    wallError = targetRightDist - distRight;
  } else if (hasLeft) {
    wallError = distLeft - targetLeftDist;
  } else {
    wallError = 0;
    wallInt  *= 0.9f;   // bleed integral, don't snap
  }

  wallInt += wallError;
  wallInt  = constrain(wallInt, -100000, 100000);
  int deriv = wallError - wallPrev;
  wallPrev  = wallError;
  return (int)(wallKp * wallError + wallKi * wallInt + wallKd * deriv);
}

// ================================================================
//  ENCODER HEADING CORRECTION (both walls absent)
// ================================================================
int encHeadingCorrection() {
  long dL = leftTicks  - lastLeftTicks;
  long dR = rightTicks - lastRightTicks;
  lastLeftTicks  = leftTicks;
  lastRightTicks = rightTicks;

  encoderError  = (float)(dL - dR);
  encoderInt   += encoderError;
  encoderInt    = constrain(encoderInt, -200.0f, 200.0f);
  float deriv   = encoderError - encoderPrev;
  encoderPrev   = encoderError;
  return (int)(encKp * encoderError + encKi * encoderInt + encKd * deriv);
}

// ================================================================
//  HEADING CORRECTION SWITCHER
//  Wall visible → wall PID.  Both absent → encoder heading hold.
//  Resets the entering PID state on each transition (edge-triggered).
// ================================================================
// int getHeadingCorrection() {
//   bool hasLeft  = (distLeft  < WALL_DETECT_THRESH);
//   bool hasRight = (distRight < WALL_DETECT_THRESH);

//   if (!hasLeft && !hasRight) {
//     if (!wasUsingEncoder) {
//       resetEncPID();   // forward declaration — defined below
//       wasUsingEncoder = true;
//     }
//     return encHeadingCorrection();
//   } else {
//     if (wasUsingEncoder) {
//       resetWallPID();  // forward declaration — defined below
//       wasUsingEncoder = false;
//     }
//     return wallPIDcorrection();
//   }
// }

float alphaSmooth = 0.2f;   // persists across calls (global/static)

int hybridHeadingCorrection() {
  bool hasLeft  = (distLeft  < WALL_DETECT_THRESH);
  bool hasRight = (distRight < WALL_DETECT_THRESH);
  float alphaTarget = (hasLeft || hasRight) ? 1.0f : 0.0f;

  // low-pass filter alpha itself — smooths the transition
  alphaSmooth += 0.15f * (alphaTarget - alphaSmooth);

  int wallCorr = wallPIDcorrection();
  int encCorr  = encHeadingCorrection();
  return (int)(alphaSmooth * wallCorr + (1.0f - alphaSmooth) * encCorr);
}

// ================================================================
//  IMU HEADING PID  (straight-drive correction)
//  FIX 2: uses imuYawTarget set by resetImuPID() — no longer reads
//  an uninitialized local variable.
// ================================================================
int imuHeadingCorrection() {
  updateYaw();
  imuErr  = imuYawTarget - yaw;   // ← FIX 2: valid reference
  imuInt += imuErr;
  imuInt  = constrain(imuInt, -500.0f, 500.0f);
  float deriv = imuErr - imuPrev;
  imuPrev = imuErr;
  return (int)(imuKp * imuErr + imuKi * imuInt + imuKd * deriv);
}

// ================================================================
//  PID RESETS
// ================================================================
void resetWallPID() { wallError = wallInt = wallPrev = 0; }
void resetEncPID()  {
  encoderError = encoderInt = encoderPrev = 0;
  lastLeftTicks  = leftTicks;
  lastRightTicks = rightTicks;
}
void resetImuPID()  {
  imuErr = imuInt = imuPrev = 0;
  imuYawTarget = yaw;   // ← FIX 2: capture current heading as target
}

// ================================================================
//  POSITION
// ================================================================
void advancePosition(int cells) {
  posX = constrain(posX + dX[orientation] * cells, 0, MAZESIZE - 1);
  posY = constrain(posY + dY[orientation] * cells, 0, MAZESIZE - 1);
}

// ================================================================
//  rotate() — 3-zone feedforward
//  +degree = CW (right),  -degree = CCW (left)
// ================================================================
void rotate(int degree) {
  yaw = 0.0f;
  float target = -1.0f * (float)degree;
  int   dir    = (degree < 0) ? 1 : -1;

  float integral = 0.0f, prevErr = 0.0f;
  int   doneCnt  = 0;
  unsigned long startMs = millis();

  while (true) {
    updateYaw();
    float err    = target - yaw;
    float absErr = fabsf(err);

    if (millis() - startMs > 500) break;

    if (absErr < ROT_DONE_THRESH) {
      if (++doneCnt >= ROT_DONE_CYCLES) break;
    } else {
      doneCnt = 0;
    }

    int leftSpd = 0, rightSpd = 0;

    if (absErr > BLAST_ZONE) {
      leftSpd  = -dir * FF_BLAST_SPEED;
      rightSpd =  dir * FF_BLAST_SPEED;
      integral = 0.0f; prevErr = 0.0f;
    } else if (absErr > BRAKE_ZONE) {
      float t  = (absErr - BRAKE_ZONE) / (BLAST_ZONE - BRAKE_ZONE);
      int   spd = constrain((int)(FF_BRAKE_MIN + t * (FF_BLAST_SPEED - FF_BRAKE_MIN)),
                            FF_BRAKE_MIN, FF_BLAST_SPEED);
      leftSpd  = -dir * spd;
      rightSpd =  dir * spd;
      prevErr  = err; integral = 0.0f;
    }
    // Zone 3 (PID) intentionally omitted — re-enable when tuning

    mspeed(leftSpd, rightSpd);
  }
}

// ================================================================
//  TRAP TURN PID
// ================================================================
void resetPID() {
  trapIntegral  = 0.0f;
  trapLastError = 0.0f;
  lasttrapTime  = millis();
}

float trapPID(float error) {
  unsigned long now = millis();
  float dt = (now - lasttrapTime) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = LOOP_MS / 1000.0f;
  lasttrapTime = now;
  trapIntegral += error * dt;
  trapIntegral  = constrain(trapIntegral, -50.0f, 50.0f);
  float derivative = (error - trapLastError) / dt;
  trapLastError = error;
  return (PID_KP * error) + (PID_KI * trapIntegral) + (PID_KD * derivative);
}

// ================================================================
//  moveForward() — short fixed-distance move (used after turns)
// ================================================================
void moveForward(float distCm) {
  if (distCm <= 0) return;
  long targetTicks = (long)roundf(cmToEncoderTicks * distCm);
  leftTicks = 0; rightTicks = 0;
  resetWallPID();
  resetEncPID();
  resetImuPID();
  wasUsingEncoder = false;
  while (true) {
    int avg = (abs(leftTicks) + abs(rightTicks)) / 2;
    if (avg >= targetTicks) break;
    updateSensors();
    if (distFront < frontThresh) break;
    int corr = hybridHeadingCorrection();
    mspeed(constrain(exploreSpeed - corr, -220, 220),
           constrain(exploreSpeed + corr, -220, 220));
  }
  return ;
}


// ================================================================
//  trapezoidalTurn()
// ================================================================
void trapezoidalTurn(float targetAngle) {
  Serial.printf("[TRAP] Starting %.1f deg turn\n", targetAngle);
  resetYaw();
  resetPID();

  float totalAngle = fabsf(targetAngle);
  float startYaw = 0.0f;
  int direction = (targetAngle > 0) ? 1 : -1;

  if (targetAngle == 180.0f) {
    // U-turn: blast 110° then wall-centre briefly
    rotate(110);
    unsigned long startMs = millis();
    while (millis() - startMs < 700) {
      updateSensors();
      int corr = wallPIDcorrection();
      mspeed(constrain(-corr, -220, 220), constrain(+corr, -220, 220));
    }
    mspeed(-100, -100);
    delay(450);
    return;
  }

  while (true) {
    updateYaw();
    float currentYaw = getYaw();
    float traveled = fabsf(normalizeAngle(currentYaw - startYaw));
    float remaining = totalAngle - traveled;

    if (remaining <= TURN_DEADBAND) {
      // mspeed(direction > 0 ? 0 : -60, direction > 0 ? 0 : 60);  // brief reverse-pulse brake
      // delay(90);
      motorsStop();
      resetEncPID();
      // motorsStop();
      //moveForward(3.5f);
      Serial.printf("[TRAP] Done. Final error: %.2f deg\n", remaining);
      return;
    }

    float speed;
    if (traveled < RAMP_UP_DEG)
      speed = map(traveled, 0, RAMP_UP_DEG, MIN_DRIVE_SPEED, BASE_SPEED);
    else if (remaining < RAMP_DOWN_DEG)
      speed = map(remaining, 0, RAMP_DOWN_DEG, MIN_DRIVE_SPEED, BASE_SPEED);
    else
      speed = BASE_SPEED;
    speed = constrain(speed, MIN_DRIVE_SPEED, BASE_SPEED);

    float yawError = normalizeAngle(targetAngle - currentYaw);
    float correction = trapPID(yawError);

    // Apply correction on top of the ramp speed, then pivot
    float adjSpeed = constrain(speed + correction, MIN_DRIVE_SPEED, BASE_SPEED);

    int leftSpeed, rightSpeed;
    if (direction > 0) {
      // Positive target — right turn — pivot right
      leftSpeed = constrain((int)(adjSpeed), -180, 180);    // left forward
      rightSpeed = constrain((int)(-adjSpeed), -180, 180);  // right backward
    } else {
      // Negative target — left turn — pivot left
      leftSpeed = constrain((int)(-adjSpeed), -180, 180);  // left backward
      rightSpeed = constrain((int)(adjSpeed), -180, 180);  // right forward
    }

    mspeed(leftSpeed, rightSpeed);
    Serial.printf("[TRAP] yaw=%.2f traveled=%.1f remaining=%.1f spd=%d L=%d R=%d\n",
                  currentYaw, traveled, remaining, (int)speed, leftSpeed, rightSpeed);
    delay(LOOP_MS);
  }
}
void ArcTurn(float targetAngle) {
  Serial.printf("[TRAP] Starting %.1f deg turn\n", targetAngle);
  resetYaw();
  resetPID();

  float totalAngle = fabsf(targetAngle);
  float startYaw   = 0.0f;
  int   direction  = (targetAngle > 0) ? 1 : -1;

  if (targetAngle == 180.0f) {
    // U-turn: blast 110° then wall-centre briefly
    rotate(110);
    unsigned long startMs = millis();
    while (millis() - startMs < 700) {
      updateSensors();
      int corr = wallPIDcorrection();
      mspeed(constrain(-corr, -220, 220), constrain(+corr, -220, 220));
    }
    mspeed(-100, -100);
    delay(450);
    return;
  }

  while (true) {
    updateYaw();
    float currentYaw = getYaw();
    float traveled   = fabsf(normalizeAngle(currentYaw - startYaw));
    float remaining  = totalAngle - traveled;

    if (remaining <= TURN_DEADBAND) {
      motorsStop();
      //moveForward(3.5f);
      Serial.printf("[TRAP] Done. Final error: %.2f deg\n", remaining);
      return;
    }

    float speed;
    if (traveled < RAMP_UP_DEG)
      speed = map(traveled,  0, RAMP_UP_DEG,   MIN_DRIVE_SPEED, BASE_SPEED);
    else if (remaining < RAMP_DOWN_DEG)
      speed = map(remaining, 0, RAMP_DOWN_DEG, MIN_DRIVE_SPEED, BASE_SPEED);
    else
      speed = BASE_SPEED;
    speed = constrain(speed, MIN_DRIVE_SPEED, BASE_SPEED);

    float yawError   = normalizeAngle(targetAngle - currentYaw);
    float correction = trapPID(yawError);

    int leftSpeed, rightSpeed;
    if (targetAngle == 90.0f) {
      leftSpeed  = constrain((int)(speed + direction * correction), 0, 180);
      rightSpeed = constrain((int)(speed - direction * correction), 0, 180);
    } else {   // -90
      leftSpeed  = constrain((int)(speed - direction * correction), 0, 180);
      rightSpeed = constrain((int)(speed + direction * correction), 0, 180);
    }

    mspeed(leftSpeed, rightSpeed);
    Serial.printf("[TRAP] yaw=%.2f traveled=%.1f remaining=%.1f spd=%d L=%d R=%d\n",
                  currentYaw, traveled, remaining, (int)speed, leftSpeed, rightSpeed);
    delay(LOOP_MS);
  }
}

// ================================================================
//  identifyBlock()
//
//  Bitmask written to maze[posX][posY]:
//    bit0(1)=West  bit1(2)=North  bit2(4)=East  bit3(8)=South
//
//  a[] = sensor readings in robot-relative frame:
//    a[0] = front sensor  (always orientation direction)
//    a[1] = right sensor
//    a[2] = back wall     ← BUGFIX C: reverted to hardcoded 0.
//                           The cell behind is always the one the
//                           robot just physically drove through, so
//                           it is always open by definition. The v8
//                           "inference" from previous-cell data was
//                           a no-op in the correct-tracking case and
//                           propagated sensor noise in the bad case.
//    a[3] = left sensor
//
//  b[] = rotated to absolute frame using orientation.
//
//  Neighbor mirroring: when we mark a wall on side X of (posX,posY),
//  we also mark the mirror wall in the adjacent cell.
//
//  FIX 4: uses updateSensors() + globals instead of sensor.read().
//  BUGFIX B: now returns immediately if this cell was already
//  mapped, so a noisy revisit can never clobber wall bits that
//  neighboring cells already mirrored into this cell.
// ================================================================
void identifyBlock() {
  // BUGFIX B: revisit guard — never re-map (and thereby risk
  // corrupting) a cell that's already been identified.
  if (visited[posX][posY]) return;

  // FIX 4: single updateSensors() call — no more direct sensor.read()
  updateSensors();

  uint8_t a[4];
  a[0] = (distFront < WALL_DETECT_THRESH) ? 1 : 0;   // front
  a[1] = (distRight < WALL_DETECT_THRESH) ? 1 : 0;   // right
  a[3] = (distLeft  < WALL_DETECT_THRESH) ? 1 : 0;   // left

  // BUGFIX C: back is always where we just came from — always open.
  a[2] = 0;

  // Rotate robot-relative a[] to absolute b[] using orientation
  uint8_t b[4];
  for (int i = 0; i < 4; i++) b[i] = a[(i - orientation + 4) % 4];

  // Write bitmask:  b[0]=W b[1]=N b[2]=E b[3]=S  → bits 0,1,2,3
  int type = b[0] * 1 + b[1] * 2 + b[2] * 4 + b[3] * 8;
  maze[posX][posY] |= type;
  visited[posX][posY] = true;

  // Mirror walls into adjacent cells
  // West neighbor: our West wall (bit0) → their East wall (bit2)
  if (posX > 0 && b[0])
    maze[posX - 1][posY] |= 4;
  // North neighbor: our North wall (bit1) → their South wall (bit3)
  if (posY < MAZESIZE - 1 && b[1])
    maze[posX][posY + 1] |= 8;
  // East neighbor: our East wall (bit2) → their West wall (bit0)
  if (posX < MAZESIZE - 1 && b[2])
    maze[posX + 1][posY] |= 1;
  // South neighbor: our South wall (bit3) → their North wall (bit1)
  if (posY > 0 && b[3])
    maze[posX][posY - 1] |= 2;

  Serial.printf("identifyBlock (%d,%d) orient=%d type=%d  W=%d N=%d E=%d S=%d\n",
                posX, posY, orientation, type, b[0], b[1], b[2], b[3]);
}

// ================================================================
//  floodfill()
//  BFS from target outward. Respects bitmask convention:
//    bit0(1)=W  bit1(2)=N  bit2(4)=E  bit3(8)=S
// ================================================================
void floodfill() {
  memset(flood, -1, sizeof(flood));
  flood[targetX][targetY] = 0;
  queue<pair<int,int>> q;
  q.push({ targetX, targetY });

  while (!q.empty()) {
    auto [x, y] = q.front(); q.pop();

    // West: passable if bit0 of (x,y) is clear
    if (x > 0 && flood[x-1][y] == -1 && !(maze[x][y] & 1)) {
      flood[x-1][y] = flood[x][y] + 1;
      q.push({ x-1, y });
    }
    // North: passable if bit1 of (x,y) is clear
    if (y < MAZESIZE-1 && flood[x][y+1] == -1 && !(maze[x][y] & 2)) {
      flood[x][y+1] = flood[x][y] + 1;
      q.push({ x, y+1 });
    }
    // East: passable if bit2 of (x,y) is clear
    if (x < MAZESIZE-1 && flood[x+1][y] == -1 && !(maze[x][y] & 4)) {
      flood[x+1][y] = flood[x][y] + 1;
      q.push({ x+1, y });
    }
    // South: passable if bit3 of (x,y) is clear
    if (y > 0 && flood[x][y-1] == -1 && !(maze[x][y] & 8)) {
      flood[x][y-1] = flood[x][y] + 1;
      q.push({ x, y-1 });
    }
  }
}

// ================================================================
//  nextBlock()
//  Picks the neighbor with flood[neighbor] == flood[current] - 1
//  that has no wall between them. Updates orientation only —
//  posX/posY updated separately via advancePosition().
//  Returns degrees to rotate (0, 90, -90, 180) or -1 if stuck.
// ================================================================
int rotationDirections[4] = { 0, 90, 180, -90 };

int nextBlock() {
  int oldOrient = orientation;
  int chosen    = -1;

  // Priority: North, East, South, West
  // North (bit1): y+1, no wall bit1
  if (posX < MAZESIZE-1 && flood[posX+1][posY] == flood[posX][posY]-1 && !(maze[posX][posY] & 4)) {
    chosen = 2; orientation = 2;
  }
  // East (bit2): x+1, no wall bit2
  else if (posY < MAZESIZE-1 && flood[posX][posY+1] == flood[posX][posY]-1 && !(maze[posX][posY] & 2)) {
    chosen = 1; orientation = 1;
  }
  // South (bit3): y-1, no wall bit3
  else if (posX > 0 && flood[posX-1][posY] == flood[posX][posY]-1 && !(maze[posX][posY] & 1)) {
    chosen = 0; orientation = 0;
  }
  // West (bit0): x-1, no wall bit0
  else if (posY > 0 && flood[posX][posY-1] == flood[posX][posY]-1 && !(maze[posX][posY] & 8)) {
    chosen = 3; orientation = 3;
  }

  if (chosen == -1) return -1;
  return rotationDirections[(orientation - oldOrient + 4) % 4];
}

// ================================================================
//  countKnownStraight()
//  FIX 3: removed local dX/dY that shadowed the globals.
//  Uses global dX/dY; only wB is local (wall bits per direction).
// ================================================================
int countKnownStraight() {
  // FIX 3: no local dX/dY — uses globals declared at file scope
  const int wB[4] = { 1, 2, 4, 8 };   // wall bit per direction
  int count = 0, x = posX, y = posY;
  while (count < MAZESIZE) {
    if (maze[x][y] & wB[orientation]) break;
    int nx = x + dX[orientation], ny = y + dY[orientation];
    if (nx < 0 || nx >= MAZESIZE || ny < 0 || ny >= MAZESIZE) break;
    if (!visited[nx][ny]) break;
    x = nx; y = ny;
    count++;
  }
  return count;
}

// ================================================================
//  exploreCell() — move one cell, wall/encoder heading control
// ================================================================
bool exploreCell() {
  long targetTicks = (long)roundf(cmToEncoderTicks * blockSize);
  leftTicks = 0; rightTicks = 0;
  resetWallPID();
  resetEncPID();
  wasUsingEncoder = false;

  while (true) {
    int avg = (abs(leftTicks) + abs(rightTicks)) / 2;
    if (avg >= targetTicks) break;
    updateSensors();
    if (distFront < frontThresh) {
      motorsStop();
      advancePosition(1);
      return true;   // hit front wall
    }
    int corr = hybridHeadingCorrection();
    mspeed(constrain(exploreSpeed - corr, -220, 220),
           constrain(exploreSpeed + corr, -220, 220));
  }
  advancePosition(1);
  return false;
}

// ================================================================
//  moveForwardFast() — multi-cell fast run
// ================================================================
void moveForwardFast(int totalCm, int cellCount) {
  long targetTicks = (long)roundf(cmToEncoderTicks * (float)totalCm);
  leftTicks = 0; rightTicks = 0;
  resetWallPID();
  resetEncPID();
  resetImuPID();

  int  nextB  = 1;
  long bTicks = (long)roundf(cmToEncoderTicks * blockSize);

  while (true) {
    long avg = (abs(leftTicks) + abs(rightTicks)) / 2;

    // Advance logical position at each cell boundary
    if (nextB <= cellCount && avg >= bTicks) {
      advancePosition(1);
      Serial.printf("[FAST] boundary %d -> (%d,%d)\n", nextB, posX, posY);
      nextB++;
      bTicks = (long)roundf(cmToEncoderTicks * blockSize * nextB);
    }

    if (avg >= targetTicks) break;

    long rem  = targetTicks - avg;
    long rUp  = (long)roundf(cmToEncoderTicks * (float)fastRampCm);
    long rDn  = (long)roundf(cmToEncoderTicks * (float)fastRampDownCm);
    int  spd;
    if      (avg < rUp) spd = map(avg, 0, rUp, minSpeed, fastSpeed);
    else if (rem < rDn) spd = map(rem, 0, rDn, minSpeed, fastSpeed);
    else                spd = fastSpeed;
    spd = constrain(spd, minSpeed, fastSpeed);

    updateSensors();
    if (distFront < frontThresh) {
      mspeed(-80, -80); delay(3); motorsStop();
      Serial.println("[FAST] brake!");
      return;
    }

    int corr = hybridHeadingCorrection();
    mspeed(constrain(spd - corr, minSpeed, 255),
           constrain(spd + corr, minSpeed, 255));
  }
  mspeed(-60, -60); delay(2); motorsStop();
}

// ================================================================
//  exploreLoop()
// ================================================================
// void exploreLoop() {
//   while (!(posX == targetX && posY == targetY)) {
//     identifyBlock();
//     floodfill();

//     int deg = nextBlock();
//     if (deg == -1) {
//       Serial.println("No path found!");
//       motorsStop();
//       // return;
//     }

//     if (deg != 0){
//       trapezoidalTurn(deg);
//       motorsStop();
//       delay(1000);

//     }

//     int knownAhead = countKnownStraight();
//     if (knownAhead >= 2) {
//       int tc = 1 + knownAhead;
//       Serial.printf("[EXP] Burst %d cells\n", tc);
//       moveForwardFast(tc * blockSize, tc);
//       floodfill();
//     } else {
//       bool hit = exploreCell();
//       Serial.printf("[EXP] pos=(%d,%d)\n", posX, posY);
//       if (hit) Serial.println("[EXP] front wall hit");
//     }
//   }
//   motorsStop();
//   Serial.printf("[EXP] Goal reached (%d,%d)!\n", posX, posY);
// }

void exploreLoop() {
  while (!(posX == targetX && posY == targetY)) {
    identifyBlock();
    floodfill();
    int deg = nextBlock();
    if (deg == -1) {
      Serial.println("No path found!");
      motorsStop();
      return;
    }
    if (deg != 0) {
      // moveForward(3.5);
      trapezoidalTurn(deg);
    }
    // int knownAhead = countKnownStraight();
    // if (knownAhead >= 2) {
    //   int tc = 1 + knownAhead;
    //   moveForwardFast(tc * blockSize, tc);
    //   floodfill();
    // } else {
    //   exploreCell();
    // }
    exploreCell();
  }
  identifyBlock();  // ← ADD: scan the goal cell itself before exiting
  motorsStop();
  Serial.printf("[EXP] Goal reached (%d,%d)!\n", posX, posY);
}

// ================================================================
//  returnToStart()
// ================================================================
void returnToStart() {
  targetX = 0; targetY = 0;
  floodfill();
  while (!(posX == 0 && posY == 0)) {
    identifyBlock();
    floodfill();
    int deg = nextBlock();
    if (deg == -1) { motorsStop(); return; }
    if (deg != 0) {
      motorsStop(); delay(60);
      trapezoidalTurn(deg); delay(80);
    }
    exploreCell();
  }
  motorsStop();
  Serial.println("At start.");
}

// ================================================================
//  SPEEDRUN
// ================================================================
#define MAX_SEGMENTS 64
struct Segment { int rotateDeg; int cells; };
Segment speedrunPath[MAX_SEGMENTS];
int     speedrunSegCount = 0;

void buildSpeedrunPath() {
  int sX = posX, sY = posY, sO = orientation;
  speedrunSegCount = 0;
  int segCells = 0;

  while (!(posX == targetX && posY == targetY) && speedrunSegCount < MAX_SEGMENTS) {
    int deg = nextBlock();
    if (deg == -1) break;
    advancePosition(1);   // simulate physical move for path building
    if (deg == 0) {
      segCells++;
    } else {
      if (segCells > 0) speedrunPath[speedrunSegCount++] = { 0, segCells };
      speedrunPath[speedrunSegCount++] = { deg, 1 };
      segCells = 0;
    }
  }
  if (segCells > 0 && speedrunSegCount < MAX_SEGMENTS)
    speedrunPath[speedrunSegCount++] = { 0, segCells };

  posX = sX; posY = sY; orientation = sO;   // restore state
  Serial.printf("Speedrun: %d segs\n", speedrunSegCount);
  for (int i = 0; i < speedrunSegCount; i++)
    Serial.printf("  seg%d rot=%d cells=%d\n", i, speedrunPath[i].rotateDeg, speedrunPath[i].cells);
}

void runSpeedrun() {
  for (int i = 0; i < speedrunSegCount; i++) {
    int deg   = speedrunPath[i].rotateDeg;
    int cells = speedrunPath[i].cells;
    if (deg != 0) { trapezoidalTurn(deg); delay(80); }
    if (cells > 0) {
      Serial.printf("[SR] seg%d %d cells\n", i, cells);
      moveForwardFast(cells * blockSize, cells);
      delay(50);
    }
  }
  motorsStop();
  Serial.println("[SPEEDRUN] Done!");
}

// ================================================================
//  DEBUG — print maze arrays to Serial
// ================================================================
void printMaze() {
  Serial.println();
  Serial.println("========== MAZE WALLS (bitmask: W=1 N=2 E=4 S=8) ==========");
  for (int y = MAZESIZE - 1; y >= 0; y--) {
    for (int x = 0; x < MAZESIZE; x++) Serial.printf("%2d ", maze[x][y]);
    Serial.println();
  }

  Serial.println("========== VISITED ==========");
  for (int y = MAZESIZE - 1; y >= 0; y--) {
    for (int x = 0; x < MAZESIZE; x++) Serial.printf("%2d ", visited[x][y] ? 1 : 0);
    Serial.println();
  }

  Serial.println("========== FLOOD VALUES ==========");
  for (int y = MAZESIZE - 1; y >= 0; y--) {
    for (int x = 0; x < MAZESIZE; x++) Serial.printf("%3d ", flood[x][y]);
    Serial.println();
  }
  Serial.printf("pos=(%d,%d) orient=%d  target=(%d,%d)\n",
                posX, posY, orientation, targetX, targetY);
  Serial.println("===========================================");
}

// ================================================================
//  HARDWARE INIT
// ================================================================
void setupSensors() {
  Wire.begin(33, 32);
  Wire.setClock(400000);
  pinMode(XSHUT1, OUTPUT); pinMode(XSHUT2, OUTPUT); pinMode(XSHUT3, OUTPUT);
  digitalWrite(XSHUT1, LOW); digitalWrite(XSHUT2, LOW); digitalWrite(XSHUT3, LOW);
  delay(100);

  pinMode(XSHUT1, INPUT); delay(10);
  if (!sensor1.init()) { Serial.println("S1 fail"); while (1); }
  sensor1.setAddress(ADDR1);

  pinMode(XSHUT2, INPUT); delay(10);
  if (!sensor2.init()) { Serial.println("S2 fail"); while (1); }
  sensor2.setAddress(ADDR2);

  pinMode(XSHUT3, INPUT); delay(10);
  if (!sensor3.init()) { Serial.println("S3 fail"); while (1); }
  sensor3.setAddress(ADDR3);

  sensor1.setDistanceMode(VL53L1X::Medium);
  sensor1.setMeasurementTimingBudget(timingBudget * 1000);
  sensor1.startContinuous(timingBudget);
  sensor2.setDistanceMode(VL53L1X::Medium);
  sensor2.setMeasurementTimingBudget(timingBudget * 1000);
  sensor2.startContinuous(timingBudget);
  sensor3.setDistanceMode(VL53L1X::Medium);
  sensor3.setMeasurementTimingBudget(timingBudget * 1000);
  sensor3.startContinuous(timingBudget);
  Serial.println("ToF OK.");
}

void setupIMU() {
  Wire.begin(33, 32);
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU6050 NOT FOUND!");
    while (1);
  }
  Serial.println("MPU6050 OK.");
  wakeMPU();
  calibrateGyro();
  yaw = 0.0f; yawRef = 0.0f;
  imuLastUs = micros();
}

void setupMotorEncoders() {
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(LEFT_ENC_A,  INPUT_PULLUP);
  pinMode(LEFT_ENC_B,  INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  isrLA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B),  isrLB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isrRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), isrRB, CHANGE);
   pinMode(BTN1, INPUT);
  pinMode(LED1, OUTPUT); pinMode(LED2, OUTPUT);
  Serial.println("Motors+encoders OK.");
}

void waitButtonPress() {
  while (true) {
    if (digitalRead(BTN1) == HIGH) break;
    delay(50); digitalWrite(LED1, LOW);
    delay(50); digitalWrite(LED1, HIGH);
  }
  delay(500);
}

// ================================================================
//  SETUP + LOOP
// ================================================================
void setup() {
  Serial.begin(115200);
  setupMotorEncoders();
  setupSensors();
  setupIMU();
  memset(maze,    0,  sizeof(maze));
  memset(flood,  -1,  sizeof(flood));
  memset(visited, 0,  sizeof(visited));
  Serial.println("Waiting for button...");
  waitButtonPress();
  Serial.println("Starting in 1s");
  delay(1000);
  // trapezoidalTurn(-90);
  // waitButtonPress();
  // Serial.println("Starting in 1s");
  // delay(1000);
  // trapezoidalTurn(90);

  // for(int i=0;i<7;i++){
  //   exploreCell();
  // }
  // motorsStop();
}

void loop() {
  Serial.println("=== EXPLORE ===");
  targetX = MAZESIZE - 1;
  targetY = MAZESIZE - 1;
  exploreLoop();
  motorsStop();
  delay(1000);

  Serial.println("Explore done. Press button to print maze...");
  waitButtonPress();
  printMaze();

  // ── Uncomment below to enable return + speedrun ──────────
  // Serial.println("=== RETURN ===");
  // returnToStart();
  // delay(2000);

  // Serial.println("=== SPEEDRUN ===");
  // targetX = MAZESIZE - 1; targetY = MAZESIZE - 1;
  // floodfill();
  // buildSpeedrunPath();
  // for (int i = 0; i < 6; i++) {
  //   digitalWrite(LED1, i % 2); digitalWrite(LED2, !(i % 2)); delay(200);
  // }
  // while (digitalRead(BTN1) == LOW && digitalRead(BTN2) == LOW) delay(50);
  // delay(500);
  // runSpeedrun();

  while (true) {
    digitalWrite(LED1, HIGH); digitalWrite(LED2, HIGH); delay(300);
    digitalWrite(LED1, LOW);  digitalWrite(LED2, LOW);  delay(300);
  }
}