#include <Wire.h>
#include <VL53L1X.h>



#define BTN1 34

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

#define XSHUT_L 13
#define XSHUT_F 27
#define XSHUT_R 25


#define ADDR_L 0x30
#define ADDR_F 0x31
#define ADDR_R 0x32

// Encoders
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

const int baseSpeed    = 150;   // cruise speed while wall-following
const int targetDist   = 45;    // mm, desired wall distance
const int frontThresh  = 150;   // mm, below this = front obstacle
const int wallThresh   = 90;   // mm, above this = wall considered missing
const int outerSpeed   = 255;   // fast wheel while searching for a missing wall
const int innerSpeed   = 70;    // slow wheel while searching for a missing wall
const int rotationSpeed = 180;  // in-place turn cruise speed




// ---------------- Wall PID ----------------
float Kp = 0.5;
float Ki = 0.0;
float Kd = 0.0;
float pidPrev = 0, pidInt = 0;



// Light smoothing on the raw sensor readings only (does not change how
// distLeft/distFront/distRight are used anywhere else in the code) —
// removes single-sample mm jitter so the PID isn't reacting to noise.
const float FILTER_ALPHA = 0.4f;
float filtL = -1, filtF = -1, filtR = -1;




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




// ---------------- Sensors ----------------
void setupSensors() {
  Wire.begin(33, 32);

  // Reset all sensors — they all boot at the same I2C address, so each
  // one has to be woken individually via XSHUT and reassigned.
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

  // Short mode: better accuracy/stability than Medium at the close ranges
  // this bot runs at (targetDist=80mm, frontThresh=150mm).
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



void updateSensors() {
  // Non-blocking: only pull a new sample if one is actually ready, instead
  // of stalling the loop waiting on the sensor. Keeps the last good value
  // otherwise, so nothing downstream ever sees a stale zero.
  if (sensorL.dataReady()) distLeft  = sensorL.read(false);
  if (sensorF.dataReady()) distFront = sensorF.read(false);
  if (sensorR.dataReady()) distRight = sensorR.read(false);

  // Error handling: VL53L1X reports 0 or very large values on a bad read
  if (distLeft  == 0 || distLeft  > 4000) distLeft  = 4000;
  if (distFront == 0 || distFront > 4000) distFront = 4000;
  if (distRight == 0 || distRight > 4000) distRight = 4000;

  // Smooth for the PID/decision logic to use
  if (filtL < 0) filtL = distLeft;
  if (filtF < 0) filtF = distFront;
  if (filtR < 0) filtR = distRight;
  filtL = FILTER_ALPHA * distLeft  + (1 - FILTER_ALPHA) * filtL;
  filtF = FILTER_ALPHA * distFront + (1 - FILTER_ALPHA) * filtF;
  filtR = FILTER_ALPHA * distRight + (1 - FILTER_ALPHA) * filtR;
}



const float straightKp = 0.6f;



void driveStraightUsingEncoders() {
  noInterrupts();
  long l = leftTicks;
  long r = rightTicks;
  leftTicks = 0;
  rightTicks = 0;
  interrupts();

  float skew = (float)(l - r); // positive = left wheel outpacing right wheel
  float correction = straightKp * skew;

  int leftSp  = constrain((int)(baseSpeed - correction), 30, 255);
  int rightSp = constrain((int)(baseSpeed + correction), 30, 255);
  mspeed(leftSp, rightSp);
}



// ---------------- PID routines ----------------
void runWallPIDSingle (float measured, int desired, bool invertMirror) {
  // invertMirror == true -> mirror error (for right-wall case) so the
  // same PID math and steering direction work for both follow modes
  float e = measured - (float)desired;
  if (invertMirror) e = -e;

  pidInt += e;
  pidInt = constrain(pidInt, -5000, 5000); // clamp so it can't wind up
  float deriv = e - pidPrev;
  pidPrev = e;

  float pid = Kp * e + Ki * pidInt + Kd * deriv;

  // NOTE: clamp fixed to 30-255 (was 30-240) — the old ceiling left the
  // "speed up" direction with far less headroom than "slow down", which
  // biased steering toward whichever side happened to need the smaller
  // correction. Now both directions have equal room to act.
  int leftSp  = constrain((int)(baseSpeed - pid), 30, 255);
  int rightSp = constrain((int)(baseSpeed + pid), 30, 255);
  mspeed(leftSp, rightSp);
}


// ---------------- Turning ----------------
float encoderCountToDegrees = 0.945; // ticks per degree of in-place rotation — calibrate on your bot

const int   ROT_DONE_HITS  = 5;   // consecutive in-tolerance checks required before calling a turn "done"
const int   ROT_TOLERANCE  = 3;   // ticks — how close to target counts as "arrived"
const unsigned long ROT_BASE_TIMEOUT_MS = 300; // stall-guard base
const unsigned long ROT_MS_PER_DEGREE   = 8;   // stall-guard scales with turn size

void rotate(int degree) {
  int target = (int)(encoderCountToDegrees * abs(degree));
  int dir = (degree > 0) ? 1 : -1;
  leftTicks = 0;
  rightTicks = 0;

  bool leftDone = false, rightDone = false;
  int leftHits = 0, rightHits = 0;

  unsigned long startMs = millis();
  unsigned long maxMs = ROT_BASE_TIMEOUT_MS + (unsigned long)abs(degree) * ROT_MS_PER_DEGREE;

  while (!leftDone || !rightDone) {
    long absL = abs(leftTicks);
    long absR = abs(rightTicks);

    // Require several consecutive in-tolerance readings, not just one
    // lucky tick count, before calling a wheel's turn complete.
    if (!leftDone) {
      if (absL >= target - ROT_TOLERANCE) {
        if (++leftHits >= ROT_DONE_HITS) leftDone = true;
      } else {
        leftHits = 0;
      }
    }
    if (!rightDone) {
      if (absR >= target - ROT_TOLERANCE) {
        if (++rightHits >= ROT_DONE_HITS) rightDone = true;
      } else {
        rightHits = 0;
      }
    }

    // Stall guard: don't spin forever waiting on a jammed/slipping wheel.
    if (millis() - startMs > maxMs) {
      Serial.println("rotate(): stall timeout, ending turn early");
      break;
    }

    // Smooth deceleration: ramp speed down over the last 50 ticks of
    // whichever wheel is furthest along, instead of running full speed
    // right up to the cutoff.
    long progressed = max(absL, absR);
    long remaining  = target - progressed;
    int rspeed = rotationSpeed;
    if (remaining < 50) {
      rspeed = max(80L, (long)rotationSpeed * remaining / 50);
    }

    int leftCmd  = leftDone  ? 0 :  dir * rspeed;
    int rightCmd = rightDone ? 0 : -dir * rspeed;
    mspeed(leftCmd, rightCmd);

    delay(5);
  }

  mspeed(0, 0);
  pidInt = 0;
  pidPrev = 0;
}











