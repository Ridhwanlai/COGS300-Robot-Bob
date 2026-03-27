/*
  Lab 07 - Object Detection & Tracking (Arduino UNO + 2WD L298N car)

  Implements the Lab 07 workflow:
    1) Sweep with ultrasonic + servo
    2) Build a depth map (angle bins -> distance)
    3) Differentiate wall vs object using the *shape* (local minimum / "dip")
    4) Use a simple Bayes-filter style belief update over angle bins to reduce noise
    5) Turn toward the detected object and drive forward in small steps until touching

  References (from your provided kit/lab materials):
    - Servo on pin A2, Ultrasonic TRIG=A1, ECHO=A0 (kit lessons)
    - L298N pin mapping: ENA=5, ENB=6, IN1=2, IN2=4, IN3=7, IN4=8 (kit lessons)
    - Lab 07: scan/map/approach, depth map, wall vs object (signal processing or Bayes)
    - Bayes Filter: multiply beliefs by a gaussian sensor model + normalize

  -----------------------
  WIRING (Arduino UNO)
  -----------------------
  Ultrasonic (HC-SR04):
    TRIG -> A1
    ECHO -> A0
    VCC  -> 5V
    GND  -> GND

  Servo (SG90):
    Signal -> A2
    +5V    -> 5V (or external 5V if servo browns out your UNO)
    GND    -> GND

  L298N Motor Driver:
    ENA -> D5 (PWM)
    ENB -> D6 (PWM)
    IN1 -> D2
    IN2 -> D4
    IN3 -> D7
    IN4 -> D8

  NOTE about your TCRT5000 sensors:
    This Lab 07 sketch does NOT require them.
    If you currently have TCRT5000 AO pins plugged into A0/A1/A2, you MUST move or disconnect them,
    because A0/A1/A2 are used here for ultrasonic+servo.

  -----------------------
  TUNING YOU WILL LIKELY NEED
  -----------------------
  - TURN_MS_PER_DEG: depends on your battery level, wheel friction, surface.
  - SERVO_MIN_DEG / SERVO_MAX_DEG: avoid mechanical collisions (common safe range: 20..160).
  - TOUCH_DISTANCE_CM: when to do the final "push".

  Serial Monitor / Plotter:
    - Open Serial Monitor at 9600 baud.
    - Use Serial Plotter to see "d0..dN" series.
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
// Sweep / depth map settings
// -------------------
int searchDir = 1;
const int SERVO_CENTER_DEG = 90;

// If your mount allows full range, set to 0 and 180.
// Many sonar towers collide at extremes; 20..160 is often safer.
const int SERVO_MIN_DEG = 20;
const int SERVO_MAX_DEG = 160;

// Number of bins in your depth map (index -> angle). 9 bins ~ about 22.5° if using 0..180.
const int NUM_BINS = 12;

const int SERVO_SETTLE_MS = 180;  // wait after moving servo before reading

// Ultrasonic timing / filtering
const unsigned long PULSE_TIMEOUT_US = 30000UL; // ~30ms ~ 5m max; prevents hanging
const int ULTRA_SAMPLES = 3;                   // median-of-3
const int ULTRA_SAMPLE_GAP_MS = 20;            // Lab slide tip: ~60ms between pings
const int MAX_DISTANCE_CM = 80;               // clamp

// -------------------
// Object-vs-wall detection (signal processing)
// -------------------
const int MIN_CONTRAST_CM = 8; // how much lower than neighbors counts as a "dip"

// -------------------
// Bayes belief filter over angle bins
// -------------------
float beliefs[NUM_BINS];
const float SIGMA_BINS = 1.0f;    // how many bins of sensor uncertainty (gaussian width)
const int BELIEF_SWEEPS = 3;      // do multiple sweeps before committing to a direction

// -------------------
// Motion / approach settings
// -------------------
const int DRIVE_SPEED = 120;       // PWM 0..255
const int TURN_SPEED  = 120;       // PWM 0..255

// This is the big calibration knob.
// Example: if a 90° turn takes ~700ms, TURN_MS_PER_DEG ~ 700/90 ≈ 7.8
const float TURN_MS_PER_DEG = 5.3f;

const int TURN_DEADBAND_DEG = 6;   // don't bother turning for tiny angle errors

const int FORWARD_STEP_MS = 350;   // drive a bit, then re-scan
const int MAX_APPROACH_STEPS = 25; // safety to prevent infinite running

const int TOUCH_DISTANCE_CM = 14;  // when to do the final push
const int FINAL_PUSH_MS = 300;

// -------------------
// Globals
// -------------------
Servo sonarServo;
int scanAngles[NUM_BINS];
int depthMap[NUM_BINS];
bool sweepForward = true;

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
  // in-place right turn (as named in kit code)
  digitalWrite(PIN_RB, LOW);
  digitalWrite(PIN_RF, HIGH);
  digitalWrite(PIN_LB, HIGH);
  digitalWrite(PIN_LF, LOW);
}

void turnL() {
  // in-place left turn (as named in kit code)
  digitalWrite(PIN_RB, HIGH);
  digitalWrite(PIN_RF, LOW);
  digitalWrite(PIN_LB, LOW);
  digitalWrite(PIN_LF, HIGH);
}

void stopp() {
  // "brake" stop (kit code sets all HIGH)
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
  delay(120);
}

void spinRightForMs(int ms) {
  if (ms <= 0) return;
  turnR();
  Set_Speed(TURN_SPEED);
  delay(ms);
  stopMotors();
  delay(120);
}

// -------------------
// Ultrasonic reading (based on kit Lesson_7)
// -------------------
float readUltrasonicRawCm() {
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
  // Median-of-3 with ~60ms gap between pings (helps reduce noise)
  float v0 = readUltrasonicRawCm();
  delay(ULTRA_SAMPLE_GAP_MS);
  float v1 = readUltrasonicRawCm();
  delay(ULTRA_SAMPLE_GAP_MS);
  float v2 = readUltrasonicRawCm();
  delay(ULTRA_SAMPLE_GAP_MS);
  float m = median3(v0, v1, v2);
  return m;
}

// -------------------
// Depth map sweep
// -------------------
void buildScanAngles() {
  // Evenly space NUM_BINS angles between SERVO_MIN_DEG and SERVO_MAX_DEG
  for (int i = 0; i < NUM_BINS; i++) {
    float t = (NUM_BINS == 1) ? 0.0f : ((float)i / (float)(NUM_BINS - 1));
    float a = (float)SERVO_MIN_DEG + t * (float)(SERVO_MAX_DEG - SERVO_MIN_DEG);
    // round to int
    scanAngles[i] = (int)(a + 0.5f);
  }
}

void sweepDepthMap(int outCm[NUM_BINS]) {
  for (int k = 0; k < NUM_BINS; k++) {
    int i = sweepForward ? k : (NUM_BINS - 1 - k);
    int angle = scanAngles[i];
    sonarServo.write(angle);
    delay(SERVO_SETTLE_MS);

    float d = readUltrasonicFilteredCm();
    outCm[i] = (int)(d + 0.5f);
  }

  // Return to center so we can use "front" readings consistently
  sonarServo.write(SERVO_CENTER_DEG);
  delay(120);

  sweepForward = !sweepForward;
}

void printDepthMap(const int cm[NUM_BINS]) {
  // Print in a Serial Plotter-friendly way: d0:123 d1:456 ...
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
// Returns index of best candidate, or -1 if none found.
// -------------------
int findObjectCandidateIndex(const int cm[NUM_BINS]) {
  int bestIdx = -1;
  int bestContrast = 0;

  for (int i = 0; i < NUM_BINS; i++) {
    int di = cm[i];
    if (di <= 0 || di >= MAX_DISTANCE_CM) continue; // invalid / no-echo

    int contrast = 0;

    if (NUM_BINS == 1) {
      contrast = 0;
    } else if (i == 0) {
      // endpoint: compare to one neighbor
      contrast = cm[1] - cm[0];
      // wall is often at edges; penalize endpoints a bit
      contrast = contrast / 2;
    } else if (i == NUM_BINS - 1) {
      contrast = cm[NUM_BINS - 2] - cm[NUM_BINS - 1];
      contrast = contrast / 2;
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
  // Gaussian around the measured index (models "sensor" uncertainty over bins)
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
  for (int i = 0; i < NUM_BINS; i++) {
    beliefs[i] = beliefs[i] / total;
  }
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

  if (abs(delta) <= TURN_DEADBAND_DEG) {
    return; // close enough
  }

  int ms = (int)(abs(delta) * TURN_MS_PER_DEG);

  // IMPORTANT: This assumes your servo mount matches the kit's typical orientation:
  //   smaller servo angles = ultrasonic points more to the RIGHT
  //   larger servo angles  = ultrasonic points more to the LEFT
  // So if targetServoDeg < 90, object is to the right -> turn right.
  if (delta < 0) {
    spinRightForMs(ms);
  } else {
    spinLeftForMs(ms);
  }
}

// -------------------
// Main behavior
// -------------------
void turnAroundFromWall() {
  // Give yourself space from the wall
  back();
  Set_Speed(DRIVE_SPEED);
  delay(350);
  stopMotors();
  delay(150);

  // Turn ~180 degrees
  int ms180 = (int)(180.0f * TURN_MS_PER_DEG);
  spinLeftForMs(ms180);
}

bool approachObject() {
  for (int step = 0; step < MAX_APPROACH_STEPS; step++) {
    Serial.print("\n=== STEP ");
    Serial.print(step);
    Serial.println(" ===");

    // 1) Belief-driven scanning
    initBeliefsUniform();

    int lastCandidate = -1;

    for (int s = 0; s < BELIEF_SWEEPS; s++) {
      sweepDepthMap(depthMap);

      Serial.print("Sweep ");
      Serial.print(s);
      Serial.print(" angles:");
      for (int i = 0; i < NUM_BINS; i++) {
        Serial.print(" ");
        Serial.print(scanAngles[i]);
      }
      Serial.println();

      printDepthMap(depthMap);

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
        // Keep beliefs as-is (uniform on first sweep). You could also slowly relax to uniform here.
      }

      printBeliefs();
      delay(200);
    }

    int bestIdx = bestBeliefIndex();
    int bestAngle = scanAngles[bestIdx];

    Serial.print("Chosen index = ");
    Serial.print(bestIdx);
    Serial.print(", servo angle = ");
    Serial.print(bestAngle);
    Serial.print(", max belief = ");
    Serial.println(maxBelief(), 3);

    // 2) If we didn't detect anything confidently, do a small search turn and try again
    if (lastCandidate < 0 && maxBelief() <= (1.0f / (float)NUM_BINS + 0.05f)) {
      Serial.println("Low confidence: doing a small search turn...");
      int searchMs = (int)(25.0f * TURN_MS_PER_DEG);
      if (searchDir > 0) spinLeftForMs(searchMs);
      else spinRightForMs(searchMs);
      searchDir = -searchDir;
      continue;
    }

    // 3) Rotate robot toward the detected direction
    rotateTowardServoAngle(bestAngle);

    // 4) Check front distance
    sonarServo.write(SERVO_CENTER_DEG);
    delay(SERVO_SETTLE_MS);
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

    // 5) Move forward a bit and repeat
    Serial.println("Driving forward a step...");
    advance();
    Set_Speed(DRIVE_SPEED);
    delay(FORWARD_STEP_MS);
    stopMotors();
    delay(200);

    // Re-check
    sonarServo.write(SERVO_CENTER_DEG);
    delay(SERVO_SETTLE_MS);
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
  Serial.begin(9600);
  delay(500);

  // Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(PIN_LB, OUTPUT);
  pinMode(PIN_LF, OUTPUT);
  pinMode(PIN_RB, OUTPUT);
  pinMode(PIN_RF, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);

  stopMotors();

  // Servo
  sonarServo.attach(SERVO_PIN);
  sonarServo.write(SERVO_CENTER_DEG);
  delay(500);

  // Angles + beliefs
  buildScanAngles();
  initBeliefsUniform();

  // Random seed (optional)
  randomSeed(analogRead(A3));

  Serial.println("Lab 07 Object Detection starting...");
  Serial.print("Servo sweep range: ");
  Serial.print(SERVO_MIN_DEG);
  Serial.print("..");
  Serial.print(SERVO_MAX_DEG);
  Serial.print(" deg, bins=");
  Serial.println(NUM_BINS);

  Serial.println("Place robot facing the wall. Starting in 3 seconds...");
  delay(3000);

  Serial.println("Beginning object search...\n");
}

void loop() {
  bool done = approachObject();
  if (done) {
    Serial.println("DONE: touched object (or very close). Stopping.");
  } else {
    Serial.println("FAILED: could not reach object. Stopping.");
  }

  stopMotors();
  while (true) {
    delay(1000);
  }
}