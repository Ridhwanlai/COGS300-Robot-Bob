/*
  ============================================================
  INTEGRATED ROBOT - Line Following → Wall Following → Object Detection
  ============================================================

  MODE SEQUENCE (sensor-based automatic switching):
    1. LINE_FOLLOW   → follows tape using 3 IR sensors
                     → switches to WALL_FOLLOW when all 3 IR sensors go LOW (off tape)
    2. WALL_FOLLOW   → follows right wall using P-controller
                     → switches to OBJECT_DETECT when front ultrasonic sees open space (>60cm)
    3. OBJECT_DETECT → sweeps servo, builds depth map, Bayes filter to find & approach object
                     → halts when object is touched (within OD_TOUCH_DIST_CM)

  ============================================================
  PIN ASSIGNMENTS
  ============================================================

  L298N Motor Driver (Lab 07 layout):
    ENA (Left  PWM) -> D5
    ENB (Right PWM) -> D6
    IN1 (Left  Fwd) -> D2
    IN2 (Left  Bck) -> D4
    IN3 (Right Fwd) -> D7
    IN4 (Right Bck) -> D8

  IR Sensors (Line Following):
    Left   IR -> D12
    Middle IR -> D13
    Right  IR -> D11

  Wall-Follow Ultrasonics (Lab 05 layout):
    Front HC-SR04: TRIG -> D1,  ECHO -> A3  (moved from D2 — conflicts with IN1)
    Right HC-SR04: TRIG -> D3,  ECHO -> A4  (moved from D4 — conflicts with IN2)

  Object Detection (servo-mounted ultrasonic):
    Servo Signal    -> A2
    Sweep TRIG      -> A1
    Sweep ECHO      -> A0

  ============================================================
*/

#include <Servo.h>
#include <math.h>

// ============================================================
// ALL CONSTANTS (defined before any functions to avoid forward-reference errors)
// ============================================================

// --- Motor Pins (Lab 07) ---
const int L_PWM_PIN = 5;
const int R_PWM_PIN = 6;
const int PIN_LF    = 2;   // IN1 Left Forward
const int PIN_LB    = 4;   // IN2 Left Backward
const int PIN_RF    = 7;   // IN3 Right Forward
const int PIN_RB    = 8;   // IN4 Right Backward

// --- IR Sensor Pins ---
const int IR_LEFT   = 12;
const int IR_MIDDLE = 13;
const int IR_RIGHT  = 11;

// --- Wall-Follow Ultrasonic Pins (Lab 05, ECHOs remapped to avoid motor conflicts) ---
const int TRIG_FRONT_PIN = A1;
const int ECHO_FRONT_PIN = A0;
const int TRIG_RIGHT_PIN = A3;
const int ECHO_RIGHT_PIN = A4;

// --- Object Detection Servo + Ultrasonic Pins ---
const int SERVO_PIN      = A2;
// const int TRIG_FRONT_PIN = A1;
// const int ECHO_FRONT_PIN = A0;

// --- Line Following ---
const int  LF_BASE_SPEED      = 90;
const int  LF_TURN_SPEED_FAST = 130;
const int  LF_SOFT_OUTER      = 97;
const int  LF_SOFT_INNER      = 93;
const int  LF_RIGHT_TRIM      = 0;
const int  LF_LEFT_TRIM       = 0;
const bool LF_USE_FILTER      = true;
const int  LF_FILTER_READS    = 2;
const int  LF_FILTER_DELAY    = 5;

// --- Wall Following ---
const float          WF_SET_CM          = 20.0f;
const float          WF_DEADBAND_CM     = 1.5f;
const float          WF_KP              = 6.0f;
const int            WF_BASE_PWM        = 170;
const int            WF_MAX_CORR        = 90;
const int            WF_STEER_SIGN      = +1;
const float          WF_FRONT_STOP_CM   = 14.0f;
const float          WF_WALL_LOST_CM    = 150.0f;
const int            WF_SEARCH_SPEED    = 140;
const unsigned long  WF_TURN_LEFT_MS    = 320;
const unsigned long  WF_SEARCH_RIGHT_MS = 220;
const float          WF_OPEN_SPACE_CM   = 60.0f;

// --- Shared Ultrasonic Filter ---
const unsigned long  US_PULSE_TIMEOUT = 30000UL;
const float          US_MIN_CM        = 2.0f;
const float          US_MAX_CM        = 400.0f;
const float          US_ALPHA         = 0.35f;
const unsigned long  US_HOLD_MS       = 200;
const unsigned long  US_PERIOD_MS     = 60;

// --- Object Detection ---
// Use #define for OD_NUM_BINS so it can size arrays at compile time
#define OD_NUM_BINS 12

const int            OD_SERVO_CENTER    = 90;
const int            OD_SERVO_MIN       = 20;
const int            OD_SERVO_MAX       = 160;
const int            OD_SERVO_SETTLE    = 180;
const unsigned long  OD_PULSE_TIMEOUT   = 30000UL;
const int            OD_ULTRA_GAP_MS    = 20;
const int            OD_MAX_DIST_CM     = 80;
const int            OD_MIN_CONTRAST_CM = 8;
const float          OD_SIGMA_BINS      = 1.0f;
const int            OD_BELIEF_SWEEPS   = 3;
const int            OD_DRIVE_SPEED     = 120;
const int            OD_TURN_SPEED      = 120;
const float          OD_TURN_MS_PER_DEG = 5.3f;
const int            OD_TURN_DEADBAND   = 6;
const int            OD_FORWARD_STEP_MS = 350;
const int            OD_MAX_STEPS       = 25;
const int            OD_TOUCH_DIST_CM   = 14;
const int            OD_FINAL_PUSH_MS   = 300;

// ============================================================
// GLOBALS
// ============================================================

enum RobotMode { LINE_FOLLOW, WALL_FOLLOW, OBJECT_DETECT };
RobotMode currentMode = LINE_FOLLOW;

// Wall-follow state machine
char          wfState        = 'N';
unsigned long wfStateUntilMs = 0;

// Shared ultrasonic filtered readings
float         frontCmFilt    = -1.0f;
float         rightCmFilt    = -1.0f;
bool          frontInit      = false;
bool          rightInit      = false;
unsigned long lastFrontOkMs  = 0;
unsigned long lastRightOkMs  = 0;
unsigned long lastUltraMs    = 0;

// Object Detection globals
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
// MODE 1: LINE FOLLOWING
// Returns true when all 3 sensors go LOW → switch to WALL_FOLLOW
// ============================================================
void lf_goStraight()    { setMotors(LF_BASE_SPEED + LF_LEFT_TRIM,  LF_BASE_SPEED + LF_RIGHT_TRIM); }
void lf_hardTurnRight() { setMotors(LF_TURN_SPEED_FAST, 0); }
void lf_hardTurnLeft()  { setMotors(0, LF_TURN_SPEED_FAST); }
void lf_softTurnRight() { setMotors(LF_SOFT_OUTER, LF_SOFT_INNER); delay(40); }
void lf_softTurnLeft()  { setMotors(LF_SOFT_INNER, LF_SOFT_OUTER); delay(40); }

bool runLineFollowing() {
  int R = digitalRead(IR_RIGHT);
  int M = digitalRead(IR_MIDDLE);
  int L = digitalRead(IR_LEFT);

  if (LF_USE_FILTER) {
    for (int i = 0; i < LF_FILTER_READS; i++) {
      delay(LF_FILTER_DELAY);
      if (digitalRead(IR_RIGHT)  != R ||
          digitalRead(IR_MIDDLE) != M ||
          digitalRead(IR_LEFT)   != L) {
        return false; // unstable reading, stay in line follow
      }
    }
  }

  // All sensors off tape → end of line → trigger mode switch
  if (L == LOW && M == LOW && R == LOW) {
    stopMotors();
    Serial.println("[LINE] All sensors off tape → switching to WALL_FOLLOW");
    return true;
  }

  // Truth table
  if      (L==HIGH && M==HIGH && R==HIGH) { lf_goStraight();    Serial.println("[LINE] Junction - straight"); }
  else if (L==HIGH && M==HIGH && R==LOW)  { lf_hardTurnLeft();  Serial.println("[LINE] Hard Left"); }
  else if (L==HIGH && M==LOW  && R==HIGH) { lf_goStraight();    Serial.println("[LINE] Cross - straight"); }
  else if (L==HIGH && M==LOW  && R==LOW)  { lf_softTurnLeft();  Serial.println("[LINE] Soft Left"); }
  else if (L==LOW  && M==HIGH && R==HIGH) { lf_hardTurnRight(); Serial.println("[LINE] Hard Right"); }
  else if (L==LOW  && M==HIGH && R==LOW)  { lf_goStraight();    Serial.println("[LINE] Straight"); }
  else if (L==LOW  && M==LOW  && R==HIGH) { lf_softTurnRight(); Serial.println("[LINE] Soft Right"); }

  return false;
}

// ============================================================
// SHARED ULTRASONIC (used by Wall Follow)
// ============================================================
float readUsCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long dur = pulseIn(echoPin, HIGH, US_PULSE_TIMEOUT);
  if (dur == 0) return -1.0f;
  float cm = (dur * 0.0343f) / 2.0f;
  if (cm < US_MIN_CM || cm > US_MAX_CM) return -1.0f;
  return cm;
}

void updateUltrasonics() {
  unsigned long now = millis();
  if (now - lastUltraMs < US_PERIOD_MS) return;
  lastUltraMs = now;

  float fr = readUsCm(TRIG_FRONT_PIN, ECHO_FRONT_PIN);
  if (fr > 0) {
    frontCmFilt  = frontInit ? (1.0f - US_ALPHA)*frontCmFilt + US_ALPHA*fr : fr;
    frontInit    = true;
    lastFrontOkMs = now;
  }

  float rr = readUsCm(TRIG_RIGHT_PIN, ECHO_RIGHT_PIN);
  if (rr > 0) {
    rightCmFilt  = rightInit ? (1.0f - US_ALPHA)*rightCmFilt + US_ALPHA*rr : rr;
    rightInit    = true;
    lastRightOkMs = now;
  }
}

bool  frontValid() { return frontInit && (millis() - lastFrontOkMs <= US_HOLD_MS); }
bool  rightValid() { return rightInit && (millis() - lastRightOkMs <= US_HOLD_MS); }
float getFrontCm() { return frontValid() ? frontCmFilt : -1.0f; }
float getRightCm() { return rightValid() ? rightCmFilt : -1.0f; }

// ============================================================
// MODE 2: WALL FOLLOWING
// Returns true when front sees open space → switch to OBJECT_DETECT
// ============================================================
bool runWallFollowing() {
  updateUltrasonics();
  unsigned long now = millis();
  float f = getFrontCm();
  float r = getRightCm();

  // Open space ahead → switch to Object Detection
  if (frontValid() && f > WF_OPEN_SPACE_CM) {
    stopMotors();
    Serial.print("[WALL] Open space detected (");
    Serial.print(f, 1);
    Serial.println(" cm) → switching to OBJECT_DETECT");
    return true;
  }

  // Timed states (corner handling)
  if (wfState == 'L') {
    if (now < wfStateUntilMs) { setMotors(180, -180); return false; }
    else wfState = 'N';
  } else if (wfState == 'R') {
    if (now < wfStateUntilMs) { setMotors(-WF_SEARCH_SPEED, WF_SEARCH_SPEED); return false; }
    else wfState = 'N';
  }

  // Obstacle too close ahead → pivot left
  if (frontValid() && f > 0 && f < WF_FRONT_STOP_CM) {
    wfState = 'L';
    wfStateUntilMs = now + WF_TURN_LEFT_MS;
    setMotors(180, -180);
    return false;
  }

  // Wall lost on right → search right
  if (!rightValid() || r < 0 || r > WF_WALL_LOST_CM) {
    wfState = 'R';
    wfStateUntilMs = now + WF_SEARCH_RIGHT_MS;
    setMotors(-WF_SEARCH_SPEED, WF_SEARCH_SPEED);
    return false;
  }

  // Normal P-control
  float err = r - WF_SET_CM;
  if (abs(err) < WF_DEADBAND_CM) err = 0;
  int corr = constrain((int)(WF_KP * err), -WF_MAX_CORR, WF_MAX_CORR);
  setMotors(WF_BASE_PWM + WF_STEER_SIGN * corr,
            WF_BASE_PWM - WF_STEER_SIGN * corr);
  return false;
}

// ============================================================
// MODE 3: OBJECT DETECTION (Lab 07)
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

void odBuildScanAngles() {
  for (int i = 0; i < OD_NUM_BINS; i++) {
    float t = (OD_NUM_BINS == 1) ? 0.0f : ((float)i / (float)(OD_NUM_BINS - 1));
    odScanAngles[i] = (int)(OD_SERVO_MIN + t * (OD_SERVO_MAX - OD_SERVO_MIN) + 0.5f);
  }
}

void odSweepDepthMap(int outCm[]) {
  for (int k = 0; k < OD_NUM_BINS; k++) {
    int i = odSweepForward ? k : (OD_NUM_BINS - 1 - k);
    sonarServo.write(odScanAngles[i]);
    delay(OD_SERVO_SETTLE);
    outCm[i] = (int)(odReadFilteredCm() + 0.5f);
  }
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
  if (abs(delta) <= OD_TURN_DEADBAND) return;
  int ms = (int)(abs(delta) * OD_TURN_MS_PER_DEG);
  if (delta < 0) spinRightForMs(ms);
  else           spinLeftForMs(ms);
}

bool runObjectDetection() {
  for (int step = 0; step < OD_MAX_STEPS; step++) {
    Serial.print("\n[OD] Step "); Serial.println(step);

    odInitBeliefs();
    int lastCand = -1;

    for (int s = 0; s < OD_BELIEF_SWEEPS; s++) {
      odSweepDepthMap(odDepthMap);
      int cand = odFindObjectCandidate(odDepthMap);
      lastCand = cand;
      if (cand >= 0) {
        Serial.print("[OD] Candidate bin="); Serial.print(cand);
        Serial.print(" angle="); Serial.println(odScanAngles[cand]);
        odUpdateBeliefs(cand);
      } else {
        Serial.println("[OD] No dip found this sweep.");
      }
    }

    int bestIdx   = odBestBeliefIdx();
    int bestAngle = odScanAngles[bestIdx];

    if (lastCand < 0 && odMaxBelief() <= (1.0f / OD_NUM_BINS + 0.05f)) {
      Serial.println("[OD] Low confidence, search turn...");
      int ms = (int)(25.0f * OD_TURN_MS_PER_DEG);
      if (odSearchDir > 0) spinLeftForMs(ms); else spinRightForMs(ms);
      odSearchDir = -odSearchDir;
      continue;
    }

    odRotateToward(bestAngle);

    sonarServo.write(OD_SERVO_CENTER);
    delay(OD_SERVO_SETTLE);
    float front = odReadFilteredCm();
    Serial.print("[OD] Front dist: "); Serial.print(front, 1); Serial.println(" cm");

    if (front <= OD_TOUCH_DIST_CM) {
      Serial.println("[OD] Object touched! Final push...");
      setMotors(OD_DRIVE_SPEED, OD_DRIVE_SPEED);
      delay(OD_FINAL_PUSH_MS);
      stopMotors();
      return true;
    }

    setMotors(OD_DRIVE_SPEED, OD_DRIVE_SPEED);
    delay(OD_FORWARD_STEP_MS);
    stopMotors();
    delay(200);

    sonarServo.write(OD_SERVO_CENTER);
    delay(OD_SERVO_SETTLE);
    front = odReadFilteredCm();
    if (front <= OD_TOUCH_DIST_CM) {
      Serial.println("[OD] Object touched! Final push...");
      setMotors(OD_DRIVE_SPEED, OD_DRIVE_SPEED);
      delay(OD_FINAL_PUSH_MS);
      stopMotors();
      return true;
    }
  }

  Serial.println("[OD] Max steps reached without touching object.");
  return false;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  delay(500);

  // Motor pins
  pinMode(PIN_LF, OUTPUT); pinMode(PIN_LB, OUTPUT);
  pinMode(PIN_RF, OUTPUT); pinMode(PIN_RB, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT); pinMode(R_PWM_PIN, OUTPUT);
  stopMotors();

  // IR sensor pins
  pinMode(IR_LEFT,   INPUT);
  pinMode(IR_MIDDLE, INPUT);
  pinMode(IR_RIGHT,  INPUT);

  // Wall-follow ultrasonic pins
  pinMode(TRIG_FRONT_PIN, OUTPUT); pinMode(ECHO_FRONT_PIN, INPUT);
  pinMode(TRIG_RIGHT_PIN, OUTPUT); pinMode(ECHO_RIGHT_PIN, INPUT);
  digitalWrite(TRIG_FRONT_PIN, LOW);
  digitalWrite(TRIG_RIGHT_PIN, LOW);

  // Object detection sweep ultrasonic
  pinMode(TRIG_FRONT_PIN, OUTPUT);
  pinMode(ECHO_FRONT_PIN, INPUT);

  // Servo
  sonarServo.attach(SERVO_PIN);
  sonarServo.write(OD_SERVO_CENTER);
  delay(500);

  odBuildScanAngles();
  odInitBeliefs();
  randomSeed(analogRead(A5));

  Serial.println("=== INTEGRATED ROBOT READY ===");
  Serial.println("Sequence: LINE_FOLLOW -> WALL_FOLLOW -> OBJECT_DETECT");
  Serial.println("Starting in 3 seconds...");
  delay(3000);
  Serial.println(">>> MODE: LINE_FOLLOW <<<");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  switch (currentMode) {

    case LINE_FOLLOW:
      if (runLineFollowing()) {
        currentMode = WALL_FOLLOW;
        wfState = 'N';
        delay(500);
        Serial.println(">>> MODE: WALL_FOLLOW <<<");
      }
      break;

    case WALL_FOLLOW:
      if (runWallFollowing()) {
        currentMode = OBJECT_DETECT;
        delay(500);
        Serial.println(">>> MODE: OBJECT_DETECT <<<");
      }
      break;

    case OBJECT_DETECT:
      stopMotors();
      if (runObjectDetection()) {
        Serial.println("=== MISSION COMPLETE: Object reached! ===");
      } else {
        Serial.println("=== OBJECT DETECT FAILED. Halting. ===");
      }
      while (true) delay(1000);
      break;
  }
}
