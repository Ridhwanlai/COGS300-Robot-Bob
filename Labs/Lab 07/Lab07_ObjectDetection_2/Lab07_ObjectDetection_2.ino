/*
  Lab 07 - Object Detection & Tracking (Arduino UNO + 2WD L298N car)
*/

#include <Servo.h>
#include <math.h>

// -------------------
// Pin assignments
// -------------------
const int SERVO_PIN = A2;
const int TRIG_PIN  = A1;
const int ECHO_PIN  = A0;

const int L_PWM_PIN = 5;
const int R_PWM_PIN = 6;
const int PIN_LB    = 2;
const int PIN_LF    = 4;
const int PIN_RB    = 7;
const int PIN_RF    = 8;

// -------------------
// Sweep / depth map settings
// -------------------
int searchDir = 1;
const int SERVO_CENTER_DEG = 90;
const int SERVO_MIN_DEG    = 20;
const int SERVO_MAX_DEG    = 160;
const int NUM_BINS         = 12;
const int SERVO_SETTLE_MS  = 80;  // reduced from 180

// -------------------
// Ultrasonic
// -------------------
const unsigned long PULSE_TIMEOUT_US  = 30000UL;
const int           ULTRA_SAMPLE_GAP_MS = 20;
const int           MAX_DISTANCE_CM     = 80;

// -------------------
// Object detection
// -------------------
const int MIN_CONTRAST_CM = 8;

// -------------------
// Bayes filter
// -------------------
float beliefs[NUM_BINS];
const float SIGMA_BINS   = 1.0f;
const int   BELIEF_SWEEPS = 3;  // reduced from 3

// -------------------
// Motion settings
// -------------------
const int   DRIVE_SPEED       = 150;  // increased from 120
const int   TURN_SPEED        = 120;
const float TURN_MS_PER_DEG   = 5.3f;
const int   TURN_DEADBAND_DEG = 6;
const int   FORWARD_STEP_MS   = 250;  // reduced from 350
const int   MAX_APPROACH_STEPS = 25;
const int   TOUCH_DISTANCE_CM = 14;
const int   FINAL_PUSH_MS     = 300;

// -------------------
// Globals
// -------------------
Servo sonarServo;
int scanAngles[NUM_BINS];
int depthMap[NUM_BINS];
bool sweepForward = true;

// -------------------
// Motor helpers
// -------------------
void Set_Speed(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(L_PWM_PIN, pwm);
  analogWrite(R_PWM_PIN, pwm);
}

void advance() {
  digitalWrite(PIN_RB, LOW);  digitalWrite(PIN_RF, HIGH);
  digitalWrite(PIN_LB, LOW);  digitalWrite(PIN_LF, HIGH);
}

void back() {
  digitalWrite(PIN_RB, HIGH); digitalWrite(PIN_RF, LOW);
  digitalWrite(PIN_LB, HIGH); digitalWrite(PIN_LF, LOW);
}

void turnR() {
  digitalWrite(PIN_RB, LOW);  digitalWrite(PIN_RF, HIGH);
  digitalWrite(PIN_LB, HIGH); digitalWrite(PIN_LF, LOW);
}

void turnL() {
  digitalWrite(PIN_RB, HIGH); digitalWrite(PIN_RF, LOW);
  digitalWrite(PIN_LB, LOW);  digitalWrite(PIN_LF, HIGH);
}

void stopp() {
  digitalWrite(PIN_RB, HIGH); digitalWrite(PIN_RF, HIGH);
  digitalWrite(PIN_LB, HIGH); digitalWrite(PIN_LF, HIGH);
}

void stopMotors() { Set_Speed(0); stopp(); }

void spinLeftForMs(int ms) {
  if (ms <= 0) return;
  turnL(); Set_Speed(TURN_SPEED); delay(ms); stopMotors(); delay(100);
}

void spinRightForMs(int ms) {
  if (ms <= 0) return;
  turnR(); Set_Speed(TURN_SPEED); delay(ms); stopMotors(); delay(100);
}

// -------------------
// Ultrasonic
// -------------------
float readUltrasonicRawCm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT_US);
  if (duration == 0) return (float)MAX_DISTANCE_CM;
  float cm = duration / 58.0f;
  return constrain(cm, 0, MAX_DISTANCE_CM);
}

float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

float readUltrasonicFilteredCm() {
  float v0 = readUltrasonicRawCm(); delay(ULTRA_SAMPLE_GAP_MS);
  float v1 = readUltrasonicRawCm(); delay(ULTRA_SAMPLE_GAP_MS);
  float v2 = readUltrasonicRawCm(); delay(ULTRA_SAMPLE_GAP_MS);
  return median3(v0, v1, v2);
}

// -------------------
// Depth map sweep
// -------------------
void buildScanAngles() {
  for (int i = 0; i < NUM_BINS; i++) {
    float t = (NUM_BINS == 1) ? 0.0f : ((float)i / (float)(NUM_BINS - 1));
    scanAngles[i] = (int)(SERVO_MIN_DEG + t * (SERVO_MAX_DEG - SERVO_MIN_DEG) + 0.5f);
  }
}

void sweepDepthMap(int outCm[NUM_BINS]) {
  for (int k = 0; k < NUM_BINS; k++) {
    int i = sweepForward ? k : (NUM_BINS - 1 - k);
    sonarServo.write(scanAngles[i]);
    delay(SERVO_SETTLE_MS);
    outCm[i] = (int)(readUltrasonicFilteredCm() + 0.5f);
  }
  sonarServo.write(SERVO_CENTER_DEG);
  delay(100);
  sweepForward = !sweepForward;
}

void printDepthMap(const int cm[NUM_BINS]) {
  for (int i = 0; i < NUM_BINS; i++) {
    Serial.print("d"); Serial.print(i); Serial.print(":"); Serial.print(cm[i]);
    if (i < NUM_BINS - 1) Serial.print(" ");
  }
  Serial.println();
}

// -------------------
// Wall baseline estimation
// Fits expected wall curve D/cos(theta) to the depth map
// Returns estimated wall distance at each bin
// -------------------
void estimateWallBaseline(const int cm[NUM_BINS], float baseline[NUM_BINS]) {
  // Use the outer 3 bins on each side to estimate the wall
  // These are most likely to be pure wall readings
  const int EDGE_BINS = 3;
  
  // Collect edge readings and their angles relative to center
  float sumCos = 0, sumD = 0, count = 0;
  
  for (int i = 0; i < EDGE_BINS && i < NUM_BINS; i++) {
    if (cm[i] > 0 && cm[i] < MAX_DISTANCE_CM) {
      float angleRad = (scanAngles[i] - SERVO_CENTER_DEG) * PI / 180.0f;
      float wallDist = cm[i] * cosf(angleRad);  // project onto wall normal
      sumD += wallDist;
      count++;
    }
  }
  for (int i = NUM_BINS - EDGE_BINS; i < NUM_BINS; i++) {
    if (cm[i] > 0 && cm[i] < MAX_DISTANCE_CM) {
      float angleRad = (scanAngles[i] - SERVO_CENTER_DEG) * PI / 180.0f;
      float wallDist = cm[i] * cosf(angleRad);
      sumD += wallDist;
      count++;
    }
  }
  
  float wallD = (count > 0) ? (sumD / count) : MAX_DISTANCE_CM;
  
  // Project wall distance back to each bin angle
  for (int i = 0; i < NUM_BINS; i++) {
    float angleRad = (scanAngles[i] - SERVO_CENTER_DEG) * PI / 180.0f;
    float cosA = cosf(angleRad);
    if (cosA < 0.1f) cosA = 0.1f;
    baseline[i] = wallD / cosA;
    if (baseline[i] > MAX_DISTANCE_CM) baseline[i] = MAX_DISTANCE_CM;
  }
}

// -------------------
// Compute residual map (how much closer than wall each bin is)
// Positive residual = closer than wall = possible object
// -------------------
void computeResidual(const int cm[NUM_BINS], const float baseline[NUM_BINS], float residual[NUM_BINS]) {
  for (int i = 0; i < NUM_BINS; i++) {
    residual[i] = baseline[i] - (float)cm[i];
    if (residual[i] < 0) residual[i] = 0;  // clamp — we only care about things closer than wall
  }
}

// -------------------
// Find object candidate using residual map + shape scoring
// Requires a localized bump in the residual (not spanning whole sweep)
// -------------------
int findObjectCandidateIndex(const int cm[NUM_BINS]) {
  float baseline[NUM_BINS];
  float residual[NUM_BINS];
  
  estimateWallBaseline(cm, baseline);
  computeResidual(cm, baseline, residual);

  // Print residual for debugging
  Serial.print("Residual: ");
  for (int i = 0; i < NUM_BINS; i++) {
    Serial.print(residual[i], 1);
    if (i < NUM_BINS - 1) Serial.print(" ");
  }
  Serial.println();

  // Find all peaks in the residual map
  int dipIndices[NUM_BINS];
  float dipScores[NUM_BINS];
  int dipCount = 0;

  for (int i = 1; i < NUM_BINS - 1; i++) {
    // Must be a local peak in residual
    if (residual[i] <= residual[i-1] || residual[i] <= residual[i+1]) continue;
    // Must be significantly above wall baseline
    if (residual[i] < MIN_CONTRAST_CM) continue;
    // Width check — object shouldn't span more than half the sweep
    // Count how many consecutive bins have residual > threshold around this peak
    int width = 1;
    for (int j = i-1; j >= 0 && residual[j] > MIN_CONTRAST_CM/2; j--) width++;
    for (int j = i+1; j < NUM_BINS && residual[j] > MIN_CONTRAST_CM/2; j++) width++;
    
    int maxObjectWidth = NUM_BINS / 2;  // objects shouldn't span more than half the view
    if (width > maxObjectWidth) continue;  // too wide — likely another wall or large flat surface

    dipIndices[dipCount] = i;
    dipScores[dipCount]  = residual[i];
    dipCount++;
  }

  if (dipCount == 0) return -1;

  // Single object peak
  if (dipCount == 1) return dipIndices[0];

  // Two peaks — find strongest two and aim for midpoint (two edges of same object)
  int best1 = 0, best2 = 1;
  if (dipScores[best2] > dipScores[best1]) { int t = best1; best1 = best2; best2 = t; }
  for (int i = 2; i < dipCount; i++) {
    if (dipScores[i] > dipScores[best1]) { best2 = best1; best1 = i; }
    else if (dipScores[i] > dipScores[best2]) { best2 = i; }
  }

  float midFloat = (dipIndices[best1] + dipIndices[best2]) / 2.0f;
  int midBin = (int)(midFloat + 0.5f);

  Serial.print("Two object peaks at bins ");
  Serial.print(dipIndices[best1]);
  Serial.print(" and ");
  Serial.print(dipIndices[best2]);
  Serial.print(" -> midpoint bin ");
  Serial.println(midBin);

  return constrain(midBin, 0, NUM_BINS - 1);
}

// -------------------
// Bayes filter
// -------------------
void initBeliefsUniform() {
  float u = 1.0f / (float)NUM_BINS;
  for (int i = 0; i < NUM_BINS; i++) beliefs[i] = u;
}

float sensorModel(int measuredIdx, int stateIdx) {
  float e  = (float)(stateIdx - measuredIdx);
  float s2 = SIGMA_BINS * SIGMA_BINS;
  return expf(-0.5f * (e * e) / s2);
}

void updateBeliefs(int measuredIdx) {
  float total = 0.0f;
  for (int i = 0; i < NUM_BINS; i++) {
    beliefs[i] *= sensorModel(measuredIdx, i);
    total += beliefs[i];
  }
  if (total <= 0.0f) { initBeliefsUniform(); return; }
  for (int i = 0; i < NUM_BINS; i++) beliefs[i] /= total;
}

int bestBeliefIndex() {
  int best = 0;
  for (int i = 1; i < NUM_BINS; i++)
    if (beliefs[i] > beliefs[best]) best = i;
  return best;
}

float maxBelief() {
  float m = beliefs[0];
  for (int i = 1; i < NUM_BINS; i++) if (beliefs[i] > m) m = beliefs[i];
  return m;
}

void printBeliefs() {
  for (int i = 0; i < NUM_BINS; i++) {
    Serial.print("b"); Serial.print(i); Serial.print(":"); Serial.print(beliefs[i], 3);
    if (i < NUM_BINS - 1) Serial.print(" ");
  }
  Serial.println();
}

// -------------------
// Turn toward servo angle
// -------------------
void rotateTowardServoAngle(int targetServoDeg) {
  int delta = targetServoDeg - SERVO_CENTER_DEG;
  if (abs(delta) <= TURN_DEADBAND_DEG) return;
  int ms = (int)(abs(delta) * TURN_MS_PER_DEG);
  if (delta < 0) spinRightForMs(ms);
  else           spinLeftForMs(ms);
}

// -------------------
// Main approach loop
// -------------------
bool approachObject() {
  for (int step = 0; step < MAX_APPROACH_STEPS; step++) {
    Serial.print("\n=== STEP "); Serial.print(step); Serial.println(" ===");

    initBeliefsUniform();
    int lastCandidate = -1;

    for (int s = 0; s < BELIEF_SWEEPS; s++) {
      sweepDepthMap(depthMap);
      printDepthMap(depthMap);

      int cand = findObjectCandidateIndex(depthMap);
      lastCandidate = cand;

      if (cand >= 0) {
        Serial.print("Dip at bin "); Serial.print(cand);
        Serial.print(" ("); Serial.print(scanAngles[cand]); Serial.println("deg)");
        updateBeliefs(cand);
      } else {
        Serial.println("No dip found.");
      }
      printBeliefs();
    }

    int bestIdx   = bestBeliefIndex();
    int bestAngle = scanAngles[bestIdx];
    Serial.print("Target: bin="); Serial.print(bestIdx);
    Serial.print(" angle="); Serial.print(bestAngle);
    Serial.print(" belief="); Serial.println(maxBelief(), 3);

    // Low confidence — search
    if (lastCandidate < 0 && maxBelief() <= (1.0f / (float)NUM_BINS + 0.05f)) {
      Serial.println("Low confidence, searching...");
      int searchMs = (int)(25.0f * TURN_MS_PER_DEG);
      if (searchDir > 0) spinLeftForMs(searchMs);
      else               spinRightForMs(searchMs);
      searchDir = -searchDir;
      continue;
    }

    // Turn toward object
    rotateTowardServoAngle(bestAngle);

    // Check front
    sonarServo.write(SERVO_CENTER_DEG);
    delay(SERVO_SETTLE_MS);
    float front = readUltrasonicFilteredCm();
    Serial.print("Front: "); Serial.print(front, 1); Serial.println(" cm");

    if (front <= TOUCH_DISTANCE_CM) {
      Serial.println("Touch! Final push...");
      advance(); Set_Speed(DRIVE_SPEED); delay(FINAL_PUSH_MS); stopMotors();
      return true;
    }

    // Step forward
    advance(); Set_Speed(DRIVE_SPEED); delay(FORWARD_STEP_MS); stopMotors(); delay(150);

    sonarServo.write(SERVO_CENTER_DEG);
    delay(SERVO_SETTLE_MS);
    front = readUltrasonicFilteredCm();
    Serial.print("After step: "); Serial.print(front, 1); Serial.println(" cm");

    if (front <= TOUCH_DISTANCE_CM) {
      Serial.println("Touch! Final push...");
      advance(); Set_Speed(DRIVE_SPEED); delay(FINAL_PUSH_MS); stopMotors();
      return true;
    }
  }

  Serial.println("Gave up.");
  return false;
}

// -------------------
// Setup / Loop
// -------------------
void setup() {
  Serial.begin(9600);
  delay(500);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PIN_LB, OUTPUT); pinMode(PIN_LF, OUTPUT);
  pinMode(PIN_RB, OUTPUT); pinMode(PIN_RF, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT); pinMode(R_PWM_PIN, OUTPUT);

  stopMotors();

  sonarServo.attach(SERVO_PIN);
  sonarServo.write(SERVO_CENTER_DEG);
  delay(500);

  buildScanAngles();
  initBeliefsUniform();
  randomSeed(analogRead(A3));

  Serial.println("Lab 07 ready. Starting in 3s...");
  delay(3000);
  Serial.println("Searching...\n");
}

void loop() {
  bool done = approachObject();
  Serial.println(done ? "DONE." : "FAILED.");
  stopMotors();
  while (true) delay(1000);
}