/*
  ============================================================
  TEST 3: OBJECT DETECTION ONLY
  ============================================================
  Isolates the servo-sweep + Bayes filter object finding logic.

  PIN ASSIGNMENTS:
    Servo Signal    -> A2
    Sweep TRIG      -> A1
    Sweep ECHO      -> A0

    ENA (Left  PWM) -> D5
    ENB (Right PWM) -> D6
    IN1 (Left  Fwd) -> D2
    IN2 (Left  Bck) -> D4
    IN3 (Right Fwd) -> D7
    IN4 (Right Bck) -> D8

  WHAT TO WATCH (Serial Monitor @ 9600 baud):
    - Prints the full depth map after each sweep
    - Prints which bin/angle the object candidate was found at
    - Prints belief array after Bayes updates
    - Prints front distance before each drive step
    - "Object touched!" means success

  TO TEST:
    1. Upload, open Serial Monitor, place an object ~40cm ahead
    2. Servo should sweep, find a dip, and print a candidate angle
    3. Robot should rotate toward it, drive forward, repeat
    4. Robot should stop when within OD_TOUCH_DIST_CM
    5. Try moving the object to different angles to test sweep coverage
  ============================================================
*/

#include <Arduino.h>
#include <pins_arduino.h>
#include <Servo.h>
#include <math.h>

// --- Motor Pins ---
const int L_PWM_PIN = 5;
const int R_PWM_PIN = 6;
const int PIN_LF    = 2;
const int PIN_LB    = 4;
const int PIN_RF    = 7;
const int PIN_RB    = 8;

// --- Object Detection Servo + Ultrasonic Pins ---
const int SERVO_PIN      = A2;
const int TRIG_FRONT_PIN = A1;
const int ECHO_FRONT_PIN = A0;

// --- Tunable Constants ---
#define OD_NUM_BINS 12

const int            OD_SERVO_CENTER    = 90;    // Servo angle for straight ahead
const int            OD_SERVO_MIN       = 20;    // Leftmost sweep angle
const int            OD_SERVO_MAX       = 160;   // Rightmost sweep angle
const int            OD_SERVO_SETTLE    = 180;   // ms to wait after moving servo
const unsigned long  OD_PULSE_TIMEOUT   = 30000UL;
const int            OD_ULTRA_GAP_MS    = 20;
const int            OD_MAX_DIST_CM     = 80;    // Readings above this are treated as empty
const int            OD_MIN_CONTRAST_CM = 8;     // Min depth dip to count as object
const float          OD_SIGMA_BINS      = 1.0f;  // Bayes Gaussian width — raise for fuzzier updates
const int            OD_BELIEF_SWEEPS   = 3;     // Sweeps per step before deciding direction
const int            OD_DRIVE_SPEED     = 120;
const int            OD_TURN_SPEED      = 120;
const float          OD_TURN_MS_PER_DEG = 5.3f;  // Tune this: ms of spin per degree of turn
const int            OD_TURN_DEADBAND   = 6;     // Degrees of error to ignore
const int            OD_FORWARD_STEP_MS = 350;   // ms of forward drive per step
const int            OD_MAX_STEPS       = 25;
const int            OD_TOUCH_DIST_CM   = 14;    // Stop when object is this close
const int            OD_FINAL_PUSH_MS   = 300;

// ============================================================
// GLOBALS
// ============================================================
Servo sonarServo;
int   odScanAngles[OD_NUM_BINS];
int   odDepthMap[OD_NUM_BINS];
float odBeliefs[OD_NUM_BINS];
bool  odSweepForward = true;
int   odSearchDir    = 1;

// ============================================================
// MOTOR HELPERS
// ============================================================
void setMotors(int leftCmd, int rightCmd) {
  leftCmd  = constrain(leftCmd,  -255, 255);
  rightCmd = constrain(rightCmd, -255, 255);
  digitalWrite(PIN_LF, leftCmd  >= 0 ? HIGH : LOW);
  digitalWrite(PIN_LB, leftCmd  >= 0 ? LOW  : HIGH);
  digitalWrite(PIN_RF, rightCmd >= 0 ? HIGH : LOW);
  digitalWrite(PIN_RB, rightCmd >= 0 ? LOW  : HIGH);
  analogWrite(L_PWM_PIN, abs(leftCmd));
  analogWrite(R_PWM_PIN, abs(rightCmd));
}

void stopMotors() {
  analogWrite(L_PWM_PIN, 0);
  analogWrite(R_PWM_PIN, 0);
  digitalWrite(PIN_LF, LOW); digitalWrite(PIN_LB, LOW);
  digitalWrite(PIN_RF, LOW); digitalWrite(PIN_RB, LOW);
}

void spinLeftForMs(int ms) {
  if (ms <= 0) return;
  setMotors(OD_TURN_SPEED, -OD_TURN_SPEED);
  delay(ms);
  stopMotors();
  delay(120);
}

void spinRightForMs(int ms) {
  if (ms <= 0) return;
  setMotors(-OD_TURN_SPEED, OD_TURN_SPEED);
  delay(ms);
  stopMotors();
  delay(120);
}

// ============================================================
// ULTRASONIC HELPERS
// ============================================================
float odReadRawCm() {
  digitalWrite(TRIG_FRONT_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_FRONT_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_FRONT_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_FRONT_PIN, HIGH, OD_PULSE_TIMEOUT);
  if (dur == 0) return (float)OD_MAX_DIST_CM;
  float cm = dur / 58.0f;
  return constrain(cm, 0.0f, (float)OD_MAX_DIST_CM);
}

float odMedian3(float a, float b, float c) {
  if (a > b) { float t=a; a=b; b=t; }
  if (b > c) { float t=b; b=c; c=t; }
  if (a > b) { float t=a; a=b; b=t; }
  return b;
}

float odReadFilteredCm() {
  float v0 = odReadRawCm(); delay(OD_ULTRA_GAP_MS);
  float v1 = odReadRawCm(); delay(OD_ULTRA_GAP_MS);
  float v2 = odReadRawCm(); delay(OD_ULTRA_GAP_MS);
  return odMedian3(v0, v1, v2);
}

// ============================================================
// OBJECT DETECTION CORE
// ============================================================
void odBuildScanAngles() {
  for (int i = 0; i < OD_NUM_BINS; i++) {
    float t = (OD_NUM_BINS == 1) ? 0.0f : ((float)i / (float)(OD_NUM_BINS - 1));
    odScanAngles[i] = (int)(OD_SERVO_MIN + t * (OD_SERVO_MAX - OD_SERVO_MIN) + 0.5f);
  }
}

void odSweepDepthMap(int outCm[]) {
  Serial.print("[OD] Depth map: ");
  for (int k = 0; k < OD_NUM_BINS; k++) {
    int i = odSweepForward ? k : (OD_NUM_BINS - 1 - k);
    sonarServo.write(odScanAngles[i]);
    delay(OD_SERVO_SETTLE);
    outCm[i] = (int)(odReadFilteredCm() + 0.5f);
    Serial.print(odScanAngles[i]); Serial.print("°=");
    Serial.print(outCm[i]); Serial.print("cm  ");
  }
  Serial.println();
  sonarServo.write(OD_SERVO_CENTER);
  delay(120);
  odSweepForward = !odSweepForward;
}

int odFindObjectCandidate(int cm[]) {
  int bestIdx = -1, bestContrast = 0;
  for (int i = 0; i < OD_NUM_BINS; i++) {
    if (cm[i] <= 0 || cm[i] >= OD_MAX_DIST_CM) continue;
    int contrast = 0;
    if      (i == 0)             contrast = (cm[1] - cm[0]) / 2;
    else if (i == OD_NUM_BINS-1) contrast = (cm[OD_NUM_BINS-2] - cm[OD_NUM_BINS-1]) / 2;
    else                         contrast = (cm[i-1] + cm[i+1]) / 2 - cm[i];
    if (contrast > bestContrast) { bestContrast = contrast; bestIdx = i; }
  }
  return (bestIdx >= 0 && bestContrast >= OD_MIN_CONTRAST_CM) ? bestIdx : -1;
}

void odInitBeliefs() {
  float u = 1.0f / OD_NUM_BINS;
  for (int i = 0; i < OD_NUM_BINS; i++) odBeliefs[i] = u;
}

void odUpdateBeliefs(int measIdx) {
  float total = 0;
  for (int i = 0; i < OD_NUM_BINS; i++) {
    float e = (float)(i - measIdx);
    odBeliefs[i] *= expf(-0.5f * e * e / (OD_SIGMA_BINS * OD_SIGMA_BINS));
    total += odBeliefs[i];
  }
  if (total <= 0) { odInitBeliefs(); return; }
  for (int i = 0; i < OD_NUM_BINS; i++) odBeliefs[i] /= total;
}

void odPrintBeliefs() {
  Serial.print("[OD] Beliefs: ");
  for (int i = 0; i < OD_NUM_BINS; i++) {
    Serial.print(odBeliefs[i], 2); Serial.print(" ");
  }
  Serial.println();
}

int odBestBeliefIdx() {
  int best = 0;
  for (int i = 1; i < OD_NUM_BINS; i++)
    if (odBeliefs[i] > odBeliefs[best]) best = i;
  return best;
}

float odMaxBelief() {
  float m = odBeliefs[0];
  for (int i = 1; i < OD_NUM_BINS; i++) if (odBeliefs[i] > m) m = odBeliefs[i];
  return m;
}

void odRotateToward(int targetDeg) {
  int delta = targetDeg - OD_SERVO_CENTER;
  Serial.print("[OD] Rotating toward "); Serial.print(targetDeg);
  Serial.print("° (delta="); Serial.print(delta); Serial.println("°)");
  if (abs(delta) <= OD_TURN_DEADBAND) { Serial.println("[OD] Within deadband, no turn."); return; }
  int ms = (int)(abs(delta) * OD_TURN_MS_PER_DEG);
  if (delta < 0) spinRightForMs(ms);
  else           spinLeftForMs(ms);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(9600);

  pinMode(PIN_LF, OUTPUT); pinMode(PIN_LB, OUTPUT);
  pinMode(PIN_RF, OUTPUT); pinMode(PIN_RB, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT); pinMode(R_PWM_PIN, OUTPUT);
  stopMotors();

  pinMode(TRIG_FRONT_PIN, OUTPUT);
  pinMode(ECHO_FRONT_PIN, INPUT);
  digitalWrite(TRIG_FRONT_PIN, LOW);

  sonarServo.attach(SERVO_PIN);
  sonarServo.write(OD_SERVO_CENTER);
  delay(500);

  odBuildScanAngles();
  odInitBeliefs();
  randomSeed(analogRead(A5));

  Serial.println("=== TEST 3: OBJECT DETECTION ===");
  Serial.println("Place an object ~40cm ahead. Starting in 3s...");
  delay(3000);
  Serial.println("GO!");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  for (int step = 0; step < OD_MAX_STEPS; step++) {
    Serial.print("\n--- Step "); Serial.print(step); Serial.println(" ---");

    odInitBeliefs();
    int lastCand = -1;

    for (int s = 0; s < OD_BELIEF_SWEEPS; s++) {
      Serial.print("[OD] Sweep "); Serial.println(s);
      odSweepDepthMap(odDepthMap);
      int cand = odFindObjectCandidate(odDepthMap);
      lastCand = cand;
      if (cand >= 0) {
        Serial.print("[OD] Candidate bin="); Serial.print(cand);
        Serial.print(" angle="); Serial.println(odScanAngles[cand]);
        odUpdateBeliefs(cand);
        odPrintBeliefs();
      } else {
        Serial.println("[OD] No dip found this sweep.");
      }
    }

    int bestIdx   = odBestBeliefIdx();
    int bestAngle = odScanAngles[bestIdx];
    Serial.print("[OD] Best belief: bin="); Serial.print(bestIdx);
    Serial.print(" angle="); Serial.println(bestAngle);

    // Low confidence → search turn
    if (lastCand < 0 && odMaxBelief() <= (1.0f / OD_NUM_BINS + 0.05f)) {
      Serial.println("[OD] Low confidence → search turn");
      int ms = (int)(25.0f * OD_TURN_MS_PER_DEG);
      if (odSearchDir > 0) spinLeftForMs(ms); else spinRightForMs(ms);
      odSearchDir = -odSearchDir;
      continue;
    }

    odRotateToward(bestAngle);

    // Check front distance
    sonarServo.write(OD_SERVO_CENTER);
    delay(OD_SERVO_SETTLE);
    float front = odReadFilteredCm();
    Serial.print("[OD] Front dist after rotate: "); Serial.print(front, 1); Serial.println(" cm");

    if (front <= OD_TOUCH_DIST_CM) {
      Serial.println("[OD] Object touched! Final push...");
      setMotors(OD_DRIVE_SPEED, OD_DRIVE_SPEED);
      delay(OD_FINAL_PUSH_MS);
      stopMotors();
      Serial.println("=== SUCCESS: Object reached! ===");
      while (true) delay(1000); // Halt
    }

    // Drive forward one step
    Serial.println("[OD] Driving forward...");
    setMotors(OD_DRIVE_SPEED, OD_DRIVE_SPEED);
    delay(OD_FORWARD_STEP_MS);
    stopMotors();
    delay(200);

    // Re-check after drive
    sonarServo.write(OD_SERVO_CENTER);
    delay(OD_SERVO_SETTLE);
    front = odReadFilteredCm();
    Serial.print("[OD] Front dist after drive: "); Serial.print(front, 1); Serial.println(" cm");

    if (front <= OD_TOUCH_DIST_CM) {
      Serial.println("[OD] Object touched! Final push...");
      setMotors(OD_DRIVE_SPEED, OD_DRIVE_SPEED);
      delay(OD_FINAL_PUSH_MS);
      stopMotors();
      Serial.println("=== SUCCESS: Object reached! ===");
      while (true) delay(1000); // Halt
    }
  }

  Serial.println("=== FAILED: Max steps reached. Halting. ===");
  while (true) delay(1000);
}
