/* ISOLATED TURN TEST — measures encoderCountToDegrees accuracy
   Does ONE 90-degree turn, then stops forever. Nothing else runs —
   no wall following, no sensors used for decisions, no repeated turns.

   HOW TO USE:
   1. Flash this to the robot.
   2. Place it on open, flat ground with room to spin freely.
   3. Mark its starting direction (tape on the floor, or line it up
      against a straight edge).
   4. Power it on. It waits 3 seconds (time to place it and step back),
      then turns once, then stops.
   5. Compare where it ends up facing against a 90-degree reference
      (protractor, folded paper corner, book corner).
   6. Repeat 3-4 times to check consistency.
   7. Adjust encoderCountToDegrees below based on what you measure:
        new_value = old_value * (90 / actual_angle_you_measured)
*/

#include <Wire.h>
#include <VL53L1X.h>

// ---------------- Pins ----------------
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

// ---------------- Tuning value under test ----------------
float encoderCountToDegrees = 0.945;  // <-- adjust this based on what you measure in the arena

const int rotationSpeed = 180;

const int   ROT_DONE_HITS  = 5;
const int   ROT_TOLERANCE  = 3;
const unsigned long ROT_BASE_TIMEOUT_MS = 300;
const unsigned long ROT_MS_PER_DEGREE   = 8;

// ---------------- Motor helper ----------------
void mspeed(int a, int b) {
  a = constrain(a, -255, 255);
  b = constrain(b, -255, 255);
  digitalWrite(AIN1, a >= 0); digitalWrite(AIN2, a < 0);
  analogWrite(PWMA, abs(a));
  digitalWrite(BIN1, b >= 0); digitalWrite(BIN2, b < 0);
  analogWrite(PWMB, abs(b));
}

// ---------------- Turn function (unchanged from the main sketch) ----------------
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

    if (millis() - startMs > maxMs) {
      Serial.println("rotate(): stall timeout, ending turn early");
      break;
    }

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

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  Serial.println("Place the robot now and mark its starting direction.");
  Serial.println("Turning in 3 seconds...");
  delay(3000);

  Serial.println("Turning now (90 degrees right)...");
  rotate(90);

  Serial.println("Turn complete. Compare against your 90-degree reference now.");
  Serial.printf("Final tick counts -> left:%ld right:%ld (target was %d)\n",
                leftTicks, rightTicks, (int)(encoderCountToDegrees * 90));
}

void loop() {
  // Nothing here on purpose — the turn already happened once in setup().
  // Power-cycle the robot to run the test again.
}
