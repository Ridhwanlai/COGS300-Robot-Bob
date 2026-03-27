/*
  Lab 07 - Object Detection & Tracking (FAST SWEEP variant)

  This is a drop-in variant of Lab07_ObjectDetection.ino that focuses on
  making the *servo + ultrasonic sweep* much faster.

  What changed vs the original:
    - Uses a dynamic servo settle delay (based on how far the servo moved)
    - Uses ULTRA_SAMPLES to choose 1/2/3 readings per angle (instead of always 3)
    - Uses a shorter ultrasonic timeout + optional min ping interval
    - Uses a faster serial baud rate (115200) to reduce print overhead

  IMPORTANT:
    The lab materials recommend ~60ms between ultrasonic pings for reliability.
    If you speed things up, expect noisier readings and compensate with
    multiple sweeps (BELIEF_SWEEPS) or a stronger signal-processing filter.

  Wiring is the same as the kit lessons:
    Servo: A2
    Ultrasonic: TRIG=A1, ECHO=A0
    L298N: ENA=5, ENB=6, IN1=2, IN2=4, IN3=7, IN4=8
*/

#include <Servo.h>
#include <math.h>

// -------------------
// Pin assignments (from the kit lesson code)
// -------------------
const int SERVO_PIN = A2;
const int TRIG_PIN  = A1;
const int ECHO_PIN  = A0;

const int L_PWM_PIN = 5;   // ENA
const int R_PWM_PIN = 6;   // ENB
const int PIN_LB    = 2;   // IN1
const int PIN_LF    = 4;   // IN2
const int PIN_RB    = 7;   // IN3
const int PIN_RF    = 8;   // IN4

// -------------------
// Speed / debug knobs
// -------------------
const long SERIAL_BAUD = 115200;

// Print less = sweep faster.
// Keep depth map printing on for Lab 07 visualization, but you can disable later.
const bool PRINT_DEPTH_EACH_SWEEP = true;
const bool PRINT_BELIEFS_EACH_SWEEP = false;

// -------------------
// Sweep / depth map settings
// -------------------
const int SERVO_CENTER_DEG = 90;

// If your mount allows full range, set to 0 and 180.
// Many sonar towers collide at extremes; 20..160 is often safer.
const int SERVO_MIN_DEG = 20;
const int SERVO_MAX_DEG = 160;

// Number of bins in your depth map (index -> angle).
// Fewer bins = faster sweep, but less angular resolution.
const int NUM_BINS = 9;

// SERVO SPEED MODEL (tune for your build)
// SG90-class servos are roughly ~0.10-0.15s per 60 degrees at 5V.
// That is ~2 ms/deg. This is *very* approximate.
const int   SERVO_BASE_SETTLE_MS = 25;   // fixed overhead
const float SERVO_MS_PER_DEG     = 2.0f; // additional per degree moved

// -------------------
// Ultrasonic timing / filtering
// -------------------

// If you only care about the bin at ~2m, you can reduce this timeout further.
// timeout_us ~ max_cm * 58
const unsigned long PULSE_TIMEOUT_US = 25000UL; // ~4.3m max, prevents hanging

// Recommended by many guides is ~60ms between pings for full-range reliability.
// For a shorter max distance, you can often reduce this.
const int MIN_PING_INTERVAL_MS = 35; // try 35–60 (60 is safer)

// Samples per angle:
//  - 1 = fastest (no filtering)
//  - 2 = average of 2
//  - 3 = median of 3 (robust, slower)
const int ULTRA_SAMPLES = 1;
const int ULTRA_SAMPLE_GAP_MS = 35; // gap between samples when ULTRA_SAMPLES > 1

const int MAX_DISTANCE_CM = 400; // clamp

// -------------------
// Object-vs-wall detection (signal processing)
// -------------------
const int MIN_CONTRAST_CM = 25; // how much lower than neighbors counts as a "dip"

// -------------------
// Bayes belief filter over angle bins
// -------------------
float beliefs[NUM_BINS];
const float SIGMA_BINS = 1.0f;    // gaussian width in bins
const int BELIEF_SWEEPS = 2;      // fewer sweeps = faster (original was 3)

// -------------------
// Motion / approach settings
// -------------------
const int DRIVE_SPEED = 140;       // PWM 0..255
const int TURN_SPEED  = 140;       // PWM 0..255

// Calibrate: if a 90° turn takes ~700ms, TURN_MS_PER_DEG ~ 7.8
const float TURN_MS_PER_DEG = 8.0f;

const int TURN_DEADBAND_DEG = 6;

const int FORWARD_STEP_MS = 350;
const int MAX_APPROACH_STEPS = 25;

const int TOUCH_DISTANCE_CM = 14;
const int FINAL_PUSH_MS = 500;

// -------------------
// Globals
// -------------------
Servo sonarServo;
int scanAngles[NUM_BINS];
int depthMap[NUM_BINS];
bool sweepForward = true;

int lastServoAngle = SERVO_CENTER_DEG;
unsigned long lastPingMs = 0;

// -------------------
// Motor helpers (copied/adapted from kit lesson code)
// -------------------
void Set_Speed(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(L_PWM_PIN, pwm);
  analogWrite(R_PWM_PIN, pwm);
}

void advance() {
  digitalWrite(PIN_RB, LOW);
  digitalWrite(PIN_RF, HIGH);
  digitalWrite(PIN_LB, LOW);
  digitalWrite(PIN_LF, HIGH);
}

void back() {
  digitalWrite(PIN_RB, HIGH);
  digitalWrite(PIN_RF, LOW);
  digitalWrite(PIN_LB, HIGH);
  digitalWrite(PIN_LF, LOW);
}

void turnR() {
  digitalWrite(PIN_RB, LOW);
  digitalWrite(PIN_RF, HIGH);
  digitalWrite(PIN_LB, HIGH);
  digitalWrite(PIN_LF, LOW);
}

void turnL() {
  digitalWrite(PIN_RB, HIGH);
  digitalWrite(PIN_RF, LOW);
  digitalWrite(PIN_LB, LOW);
  digitalWrite(PIN_LF, HIGH);
}

void stopp() {
  digitalWrite(PIN_RB, HIGH);
  digitalWrite(PIN_RF, HIGH);
  digitalWrite(PIN_LB, HIGH);
  digitalWrite(PIN_LF, HIGH);
}

void stopMotors() {
  Set_Speed(0);
  stopp();
}

void spinLeftForMs(int ms) {
  if (ms <= 0) return;
  turnL();
  Set_Speed(TURN_SPEED);
  delay(ms);
  stopMotors();
  delay(80);
}

void spinRightForMs(int ms) {
  if (ms <= 0) return;
  turnR();
  Set_Speed(TURN_SPEED);
  delay(ms);
  stopMotors();
  delay(80);
}

// -------------------
// Fast-ish servo positioning helper
// -------------------
void moveServoAndWait(int angle) {
  angle = constrain(angle, SERVO_MIN_DEG, SERVO_MAX_DEG);
  sonarServo.write(angle);

  int delta = abs(angle - lastServoAngle);
  int waitMs = SERVO_BASE_SETTLE_MS + (int)(SERVO_MS_PER_DEG * (float)delta);
  if (waitMs < 20) waitMs = 20; // at least one 50Hz servo frame
  delay(waitMs);

  lastServoAngle = angle;
}

// -------------------
// Ultrasonic reading (based on kit Lesson_7)
// -------------------
float readUltrasonicRawCm() {
  // Enforce a minimum interval between pings (helps reduce self-interference)
  unsigned long now = millis();
  unsigned long elapsed = now - lastPingMs;
  if (elapsed < (unsigned long)MIN_PING_INTERVAL_MS) {
    delay((int)((unsigned long)MIN_PING_INTERVAL_MS - elapsed));
  }
  lastPingMs = millis();

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT_US);
  if (duration == 0) {
    return (float)MAX_DISTANCE_CM;
  }
  float cm = duration / 58.0f;
  if (cm < 0) cm = 0;
  if (cm > MAX_DISTANCE_CM) cm = MAX_DISTANCE_CM;
  return cm;
}

float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

float readUltrasonicFilteredCm() {
  if (ULTRA_SAMPLES <= 1) {
    return readUltrasonicRawCm();
  }
  if (ULTRA_SAMPLES == 2) {
    float v0 = readUltrasonicRawCm();
    delay(ULTRA_SAMPLE_GAP_MS);
    float v1 = readUltrasonicRawCm();
    return 0.5f * (v0 + v1);
  }

  // Default: median-of-3
  float v0 = readUltrasonicRawCm();
  delay(ULTRA_SAMPLE_GAP_MS);
  float v1 = readUltrasonicRawCm();
  delay(ULTRA_SAMPLE_GAP_MS);
  float v2 = readUltrasonicRawCm();
  return median3(v0, v1, v2);
}

// -------------------
// Depth map sweep
// -------------------
void buildScanAngles() {
  for (int i = 0; i < NUM_BINS; i++) {
    float t = (NUM_BINS == 1) ? 0.0f : ((float)i / (float)(NUM_BINS - 1));
    float a = (float)SERVO_MIN_DEG + t * (float)(SERVO_MAX_DEG - SERVO_MIN_DEG);
    scanAngles[i] = (int)(a + 0.5f);
  }
}

void sweepDepthMap(int outCm[NUM_BINS]) {
  for (int k = 0; k < NUM_BINS; k++) {
    int i = sweepForward ? k : (NUM_BINS - 1 - k);
    int angle = scanAngles[i];
    moveServoAndWait(angle);

    float d = readUltrasonicFilteredCm();
    outCm[i] = (int)(d + 0.5f);
  }

  // Return to center so we can use "front" readings consistently
  moveServoAndWait(SERVO_CENTER_DEG);
  sweepForward = !sweepForward;
}

void printDepthMap(const int cm[NUM_BINS]) {
  for (int i = 0; i < NUM_BINS; i++) {
    Serial.print("d");
    Serial.print(i);
    Serial.print(":");
    Serial.print(cm[i]);
    if (i < NUM_BINS - 1) Serial.print(" ");
  }
  Serial.println();
}

// -------------------
// Signal-processing: find an "object-like" dip
// -------------------
int findObjectCandidateIndex(const int cm[NUM_BINS]) {
  int bestIdx = -1;
  int bestContrast = 0;

  for (int i = 0; i < NUM_BINS; i++) {
    int di = cm[i];
    if (di <= 0 || di >= MAX_DISTANCE_CM) continue;

    int contrast = 0;
    if (NUM_BINS == 1) {
      contrast = 0;
    } else if (i == 0) {
      contrast = (cm[1] - cm[0]) / 2;
    } else if (i == NUM_BINS - 1) {
      contrast = (cm[NUM_BINS - 2] - cm[NUM_BINS - 1]) / 2;
    } else {
      int neighborAvg = (cm[i - 1] + cm[i + 1]) / 2;
      contrast = neighborAvg - cm[i];
    }

    if (contrast > bestContrast) {
      bestContrast = contrast;
      bestIdx = i;
    }
  }

  if (bestIdx < 0) return -1;
  if (bestContrast < MIN_CONTRAST_CM) return -1;
  return bestIdx;
}

// -------------------
// Bayes-filter style belief update over angle bins
// -------------------
void initBeliefsUniform() {
  float u = 1.0f / (float)NUM_BINS;
  for (int i = 0; i < NUM_BINS; i++) beliefs[i] = u;
}

float sensorModel(int measuredIdx, int stateIdx) {
  float e = (float)(stateIdx - measuredIdx);
  float s2 = SIGMA_BINS * SIGMA_BINS;
  return expf(-0.5f * (e * e) / s2);
}

void updateBeliefs(int measuredIdx) {
  float total = 0.0f;
  for (int i = 0; i < NUM_BINS; i++) {
    beliefs[i] = beliefs[i] * sensorModel(measuredIdx, i);
    total += beliefs[i];
  }
  if (total <= 0.0f) {
    initBeliefsUniform();
    return;
  }
  for (int i = 0; i < NUM_BINS; i++) beliefs[i] = beliefs[i] / total;
}

int bestBeliefIndex() {
  int best = 0;
  float bestVal = beliefs[0];
  for (int i = 1; i < NUM_BINS; i++) {
    if (beliefs[i] > bestVal) {
      bestVal = beliefs[i];
      best = i;
    }
  }
  return best;
}

float maxBelief() {
  float m = beliefs[0];
  for (int i = 1; i < NUM_BINS; i++) if (beliefs[i] > m) m = beliefs[i];
  return m;
}

void printBeliefs() {
  for (int i = 0; i < NUM_BINS; i++) {
    Serial.print("b");
    Serial.print(i);
    Serial.print(":");
    Serial.print(beliefs[i], 3);
    if (i < NUM_BINS - 1) Serial.print(" ");
  }
  Serial.println();
}

// -------------------
// Turn toward a chosen servo angle
// -------------------
void rotateTowardServoAngle(int targetServoDeg) {
  int delta = targetServoDeg - SERVO_CENTER_DEG;
  if (abs(delta) <= TURN_DEADBAND_DEG) return;

  int ms = (int)(abs(delta) * TURN_MS_PER_DEG);

  // Assumes: smaller servo angles point more RIGHT, larger point more LEFT.
  if (delta < 0) spinRightForMs(ms);
  else          spinLeftForMs(ms);
}

// -------------------
// Main behavior
// -------------------
void turnAroundFromWall() {
  back();
  Set_Speed(DRIVE_SPEED);
  delay(350);
  stopMotors();
  delay(120);

  int ms180 = (int)(180.0f * TURN_MS_PER_DEG);
  spinLeftForMs(ms180);
}

bool approachObject() {
  for (int step = 0; step < MAX_APPROACH_STEPS; step++) {
    Serial.print("\n=== STEP ");
    Serial.print(step);
    Serial.println(" ===");

    initBeliefsUniform();
    int lastCandidate = -1;

    for (int s = 0; s < BELIEF_SWEEPS; s++) {
      sweepDepthMap(depthMap);

      if (PRINT_DEPTH_EACH_SWEEP) {
        Serial.print("Sweep ");
        Serial.print(s);
        Serial.print(" angles:");
        for (int i = 0; i < NUM_BINS; i++) {
          Serial.print(" ");
          Serial.print(scanAngles[i]);
        }
        Serial.println();

        printDepthMap(depthMap);
      }

      int cand = findObjectCandidateIndex(depthMap);
      lastCandidate = cand;

      if (cand >= 0) {
        Serial.print("Candidate dip at index ");
        Serial.print(cand);
        Serial.print(" (servo angle ");
        Serial.print(scanAngles[cand]);
        Serial.println(")");
        updateBeliefs(cand);
      } else {
        Serial.println("No strong dip candidate found this sweep.");
      }

      if (PRINT_BELIEFS_EACH_SWEEP) {
        printBeliefs();
      }
    }

    int bestIdx = bestBeliefIndex();
    int bestAngle = scanAngles[bestIdx];

    Serial.print("Chosen index = ");
    Serial.print(bestIdx);
    Serial.print(", servo angle = ");
    Serial.print(bestAngle);
    Serial.print(", max belief = ");
    Serial.println(maxBelief(), 3);

    if (lastCandidate < 0 && maxBelief() <= (1.0f / (float)NUM_BINS + 0.05f)) {
      Serial.println("Low confidence: doing a small search turn...");
      spinLeftForMs((int)(25.0f * TURN_MS_PER_DEG));
      continue;
    }

    rotateTowardServoAngle(bestAngle);

    moveServoAndWait(SERVO_CENTER_DEG);
    float front = readUltrasonicFilteredCm();
    Serial.print("Front distance after turn: ");
    Serial.print(front, 1);
    Serial.println(" cm");

    if (front <= TOUCH_DISTANCE_CM) {
      Serial.println("Touch range reached! Final push...");
      advance();
      Set_Speed(DRIVE_SPEED);
      delay(FINAL_PUSH_MS);
      stopMotors();
      return true;
    }

    Serial.println("Driving forward a step...");
    advance();
    Set_Speed(DRIVE_SPEED);
    delay(FORWARD_STEP_MS);
    stopMotors();
    delay(120);

    moveServoAndWait(SERVO_CENTER_DEG);
    front = readUltrasonicFilteredCm();
    Serial.print("Front distance after forward step: ");
    Serial.print(front, 1);
    Serial.println(" cm");

    if (front <= TOUCH_DISTANCE_CM) {
      Serial.println("Touch range reached! Final push...");
      advance();
      Set_Speed(DRIVE_SPEED);
      delay(FINAL_PUSH_MS);
      stopMotors();
      return true;
    }
  }

  Serial.println("Gave up after max steps (did not reach touch distance).");
  return false;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(PIN_LB, OUTPUT);
  pinMode(PIN_LF, OUTPUT);
  pinMode(PIN_RB, OUTPUT);
  pinMode(PIN_RF, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);

  stopMotors();

  sonarServo.attach(SERVO_PIN);
  sonarServo.write(SERVO_CENTER_DEG);
  lastServoAngle = SERVO_CENTER_DEG;
  delay(400);

  buildScanAngles();
  initBeliefsUniform();

  randomSeed(analogRead(A3));

  Serial.println("Lab 07 Object Detection (FAST SWEEP) starting...");
  Serial.print("Servo sweep range: ");
  Serial.print(SERVO_MIN_DEG);
  Serial.print("..");
  Serial.print(SERVO_MAX_DEG);
  Serial.print(" deg, bins=");
  Serial.println(NUM_BINS);

  Serial.print("ULTRA_SAMPLES=");
  Serial.print(ULTRA_SAMPLES);
  Serial.print(", MIN_PING_INTERVAL_MS=");
  Serial.println(MIN_PING_INTERVAL_MS);

  Serial.println("Place robot facing the wall. Starting in 3 seconds...");
  delay(3000);

  turnAroundFromWall();
  Serial.println("Turn-around complete. Beginning object search...\n");
}

void loop() {
  bool done = approachObject();
  if (done) {
    Serial.println("DONE: touched object (or very close). Stopping.");
  } else {
    Serial.println("FAILED: could not reach object. Stopping.");
  }
  stopMotors();
  while (true) delay(1000);
}
