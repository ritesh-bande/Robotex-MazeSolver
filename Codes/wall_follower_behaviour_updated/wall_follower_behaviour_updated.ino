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

VL53L1X sensorL, sensorF, sensorR;
int timingBudget = 20;
unsigned long curtime = 0;

int distLeft = 0, distFront = 0, distRight = 0;

const int baseSpeed    = 150;
const int targetDist   = 45;
const int frontThresh  = 150;
const int wallThresh   = 90;
const int rotationSpeed = 180;

const int wallHysteresis  = 10;
const int frontHysteresis = 10;

bool wallLeftPresent  = false;
bool wallRightPresent = false;
bool wallFrontPresent = false;

void updateWallFlags() {
  if (wallLeftPresent)  wallLeftPresent  = distLeft  <= (wallThresh + wallHysteresis);
  else                   wallLeftPresent  = distLeft  <= (wallThresh - wallHysteresis);

  if (wallRightPresent) wallRightPresent = distRight <= (wallThresh + wallHysteresis);
  else                   wallRightPresent = distRight <= (wallThresh - wallHysteresis);

  if (wallFrontPresent) wallFrontPresent = distFront <  (frontThresh + frontHysteresis);
  else                   wallFrontPresent = distFront <  (frontThresh - frontHysteresis);
}

float Kp = 1.35;
float Ki = 0.0002;
float Kd = 0.87;
float pidPrev = 0, pidInt = 0;

void mspeed(int a, int b) {
  a = constrain(a, -255, 255);
  b = constrain(b, -255, 255);
  digitalWrite(AIN1, a >= 0); digitalWrite(AIN2, a < 0);
  analogWrite(PWMA, abs(a));
  digitalWrite(BIN1, b >= 0); digitalWrite(BIN2, b < 0);
  analogWrite(PWMB, abs(b));
}

void setupSensors() {
  Wire.begin(33, 32);

  pinMode(XSHUT_L, OUTPUT);
  pinMode(XSHUT_F, OUTPUT);
  pinMode(XSHUT_R, OUTPUT);
  digitalWrite(XSHUT_L, LOW);
  digitalWrite(XSHUT_F, LOW);
  digitalWrite(XSHUT_R, LOW);
  delay(100);

  pinMode(XSHUT_L, INPUT);
  delay(10);
  if (!sensorL.init()) { while (1); }
  sensorL.setAddress(ADDR_L);

  pinMode(XSHUT_F, INPUT);
  delay(10);
  if (!sensorF.init()) { while (1); }
  sensorF.setAddress(ADDR_F);

  pinMode(XSHUT_R, INPUT);
  delay(10);
  if (!sensorR.init()) { while (1); }
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
}

void updateSensors() {
  if (sensorL.dataReady()) distLeft  = sensorL.read(false);
  if (sensorF.dataReady()) distFront = sensorF.read(false);
  if (sensorR.dataReady()) distRight = sensorR.read(false);

  if (distLeft  == 0 || distLeft  > 4000) distLeft  = 4000;
  if (distFront == 0 || distFront > 4000) distFront = 4000;
  if (distRight == 0 || distRight > 4000) distRight = 4000;
}

const float straightKp = 0.6f;

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

void runWallPIDSingle(float measured, int desired) {
  float e = measured - (float)desired;

  pidInt += e;
  pidInt = constrain(pidInt, -5000, 5000);
  float deriv = e - pidPrev;
  pidPrev = e;

  float pid = Kp * e + Ki * pidInt + Kd * deriv;

  int leftSp  = constrain((int)(baseSpeed - pid), 30, 255);
  int rightSp = constrain((int)(baseSpeed + pid), 30, 255);
  mspeed(leftSp, rightSp);
}

float encoderCountToDegrees = 0.945;

const int   ROT_DONE_HITS  = 5;
const int   ROT_TOLERANCE  = 3;
const unsigned long ROT_BASE_TIMEOUT_MS = 300;
const unsigned long ROT_MS_PER_DEGREE   = 8;

const int PRE_TURN_CREEP_TICKS = 20;

void rotate(int degree) {
  int target = (int)(encoderCountToDegrees * abs(degree));
  int dir = (degree >= 0) ? 1 : -1;
  leftTicks = 0;
  rightTicks = 0;

  bool leftDone = false, rightDone = false;
  int leftHits = 0, rightHits = 0;

  unsigned long startMs = millis();
  unsigned long maxMs = ROT_BASE_TIMEOUT_MS + (unsigned long)abs(degree) * ROT_MS_PER_DEGREE;

  while (!leftDone || !rightDone) {
    long absL = abs(leftTicks);
    long absR = abs(rightTicks);

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

    if (millis() - startMs > maxMs) break;

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

void creepForward(int ticks) {
  leftTicks = 0;
  rightTicks = 0;

  bool leftDone = false, rightDone = false;
  int leftHits = 0, rightHits = 0;

  unsigned long startMs = millis();
  unsigned long maxMs = ROT_BASE_TIMEOUT_MS + (unsigned long)ticks * 4;

  while (!leftDone || !rightDone) {
    long absL = abs(leftTicks);
    long absR = abs(rightTicks);

    if (!leftDone) {
      if (absL >= ticks - ROT_TOLERANCE) {
        if (++leftHits >= ROT_DONE_HITS) leftDone = true;
      } else {
        leftHits = 0;
      }
    }
    if (!rightDone) {
      if (absR >= ticks - ROT_TOLERANCE) {
        if (++rightHits >= ROT_DONE_HITS) rightDone = true;
      } else {
        rightHits = 0;
      }
    }

    if (millis() - startMs > maxMs) break;

    long progressed = max(absL, absR);
    long remaining  = ticks - progressed;
    int spd = baseSpeed;
    if (remaining < 30) {
      spd = max(60L, (long)baseSpeed * remaining / 30);
    }

    float skew = (float)(leftTicks - rightTicks);
    float correction = straightKp * skew;

    int leftCmd  = leftDone  ? 0 : constrain((int)(spd - correction), 0, 255);
    int rightCmd = rightDone ? 0 : constrain((int)(spd + correction), 0, 255);
    mspeed(leftCmd, rightCmd);

    delay(5);
  }

  mspeed(0, 0);
}

void behaviorStep() {
  updateSensors();
  updateWallFlags();

  if (wallFrontPresent) {
    if (!wallLeftPresent) {
      creepForward(PRE_TURN_CREEP_TICKS);
      rotate(90);
    } else if (!wallRightPresent) {
      creepForward(PRE_TURN_CREEP_TICKS);
      rotate(-90);
    } else {
      rotate(180);
    }
    pidInt = 0;
    pidPrev = 0;
    return;
  }

  if (wallLeftPresent) {
    runWallPIDSingle(distLeft, targetDist);
    return;
  }

  if (wallRightPresent) {
    creepForward(PRE_TURN_CREEP_TICKS);
    rotate(90);
    pidInt = 0;
    pidPrev = 0;
    return;
  }

  pidInt = 0;
  pidPrev = 0;
  driveStraightUsingEncoders();
}

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(BTN1, INPUT);

  setupSensors();

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  while (digitalRead(BTN1) != HIGH) { delay(10); }

  curtime = millis();
}

void loop() {
  behaviorStep();
  while (millis() - curtime < (unsigned long)timingBudget) { }
  curtime = millis();
}
