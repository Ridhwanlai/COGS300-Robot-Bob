/*
  ============================================================
  TEST 2: WALL FOLLOWING ONLY
  ============================================================

  PIN ASSIGNMENTS:
    Front HC-SR04: TRIG -> A1,  ECHO -> A0
    Right HC-SR04: TRIG -> A3,  ECHO -> A4

    ENA (Left  PWM) -> D5
    ENB (Right PWM) -> D6
    IN1 (Left  Fwd) -> D2
    IN2 (Left  Bck) -> D4
    IN3 (Right Fwd) -> D7
    IN4 (Right Bck) -> D8

  STATE LOGIC (in priority order):
    1. STUCK: front sensor unchanged for > US_STUCK_MS -> reverse then push (fires ONCE only)
    2. Front NEAR + Right FAR   -> turn right  (front blocked, wall gone)
    3. Front NEAR + Right NEAR  -> turn left   (corner)
    4. Front FAR  + Right NEAR  -> distance maintenance:
          too close (r < TARGET - TOLERANCE) -> steer left
          too far   (r > TARGET + TOLERANCE) -> steer right
          in band                            -> go straight
    5. Front FAR  + Right FAR   -> go straight (open space)

  DISTANCE MAINTENANCE:
    Target distance = WF_RIGHT_NEAR_CM (the near/far threshold).
    Tolerance band  = WF_RIGHT_TOLERANCE_CM on each side.
    Within the band: go straight.
    Outside the band: one wheel stops, other drives — gentle arc correction.
  ============================================================
*/

// --- Motor Pins ---
const int L_PWM_PIN = 5;
const int R_PWM_PIN = 6;
const int PIN_LF    = 2;
const int PIN_LB    = 4;
const int PIN_RF    = 7;
const int PIN_RB    = 8;

// --- Wall-Follow Ultrasonic Pins ---
const int TRIG_FRONT_PIN = A1;
const int ECHO_FRONT_PIN = A0;
const int TRIG_RIGHT_PIN = A3;
const int ECHO_RIGHT_PIN = A4;

// --- Tunable Constants ---
const float          WF_FRONT_NEAR_CM       = 15.0f;  // Front closer than this = NEAR
const float          WF_RIGHT_NEAR_CM       = 13.0f;  // Target wall distance (also NEAR threshold)
const float          WF_RIGHT_TOLERANCE_CM  = 3.0f;   // Acceptable band around target (+/- this value)
const int            WF_STRAIGHT_SPEED      = 110;    // Forward speed (both wheels)
const int            WF_TURN_SPEED          = 120;    // Speed for hard turns (corner/right)
const int            WF_CORRECT_OUTER       = 110;    // Outer wheel speed during correction arc
const int            WF_CORRECT_INNER       = 60;     // Inner wheel speed during correction arc

// --- SMA Filter ---
#define US_SMA_WINDOW 5

// --- Ultrasonic Timing ---
const unsigned long  US_PULSE_TIMEOUT    = 30000UL;
const float          US_MIN_CM           = 2.0f;
const float          US_MAX_CM           = 400.0f;
const unsigned long  US_HOLD_MS          = 200;
const unsigned long  US_PERIOD_MS        = 60;

// --- Stuck Detection (front sensor only) ---
const unsigned long  US_STUCK_MS         = 3000UL;
const float          US_STUCK_BAND_CM    = 1.5f;
const int            US_STUCK_PUSH_MS    = 2000;
const int            US_STUCK_REVERSE_MS = 500;
const int            US_STUCK_PUSH_PWM   = 255;

// ============================================================
// GLOBALS
// ============================================================

float frontBuf[US_SMA_WINDOW];
float rightBuf[US_SMA_WINDOW];
int   frontBufIdx   = 0;
int   rightBufIdx   = 0;
int   frontBufCount = 0;
int   rightBufCount = 0;

float         frontCmFilt   = -1.0f;
float         rightCmFilt   = -1.0f;
bool          frontInit     = false;
bool          rightInit     = false;
unsigned long lastFrontOkMs = 0;
unsigned long lastRightOkMs = 0;
unsigned long lastUltraMs   = 0;

float         stuckFrontRef    = -1.0f;
unsigned long stuckSinceMs     = 0;
bool          stuckTimerActive = false;
bool          stuckUsed        = false;

// ============================================================
// MOTOR HELPERS
// ============================================================
void setMotors(int leftCmd, int rightCmd) {
  leftCmd  = constrain(leftCmd,  -255, 255);
  rightCmd = constrain(rightCmd, -255, 255);
  digitalWrite(PIN_LF, leftCmd  >= 0 ? LOW  : HIGH);
  digitalWrite(PIN_LB, leftCmd  >= 0 ? HIGH : LOW);
  digitalWrite(PIN_RF, rightCmd >= 0 ? LOW  : HIGH);
  digitalWrite(PIN_RB, rightCmd >= 0 ? HIGH : LOW);
  analogWrite(L_PWM_PIN, abs(leftCmd));
  analogWrite(R_PWM_PIN, abs(rightCmd));
}

void stopMotors() {
  analogWrite(L_PWM_PIN, 0);
  analogWrite(R_PWM_PIN, 0);
  digitalWrite(PIN_LF, LOW); digitalWrite(PIN_LB, LOW);
  digitalWrite(PIN_RF, LOW); digitalWrite(PIN_RB, LOW);
}

// ============================================================
// SMA FILTER HELPERS
// ============================================================
void smaAdd(float* buf, int* idx, int* count, float val) {
  buf[*idx] = val;
  *idx = (*idx + 1) % US_SMA_WINDOW;
  if (*count < US_SMA_WINDOW) (*count)++;
}

float smaGet(float* buf, int count) {
  if (count == 0) return -1.0f;
  float sum = 0;
  for (int i = 0; i < count; i++) sum += buf[i];
  return sum / count;
}

// ============================================================
// ULTRASONIC HELPERS
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
    smaAdd(frontBuf, &frontBufIdx, &frontBufCount, fr);
    frontCmFilt   = smaGet(frontBuf, frontBufCount);
    frontInit     = true;
    lastFrontOkMs = now;
  }

  float rr = readUsCm(TRIG_RIGHT_PIN, ECHO_RIGHT_PIN);
  if (rr > 0) {
    smaAdd(rightBuf, &rightBufIdx, &rightBufCount, rr);
    rightCmFilt   = smaGet(rightBuf, rightBufCount);
    rightInit     = true;
    lastRightOkMs = now;
  }
}

bool  frontValid() { return frontInit && (millis() - lastFrontOkMs <= US_HOLD_MS); }
bool  rightValid() { return rightInit && (millis() - lastRightOkMs <= US_HOLD_MS); }
float getFrontCm() { return frontValid() ? frontCmFilt : -1.0f; }
float getRightCm() { return rightValid() ? rightCmFilt : -1.0f; }

// ============================================================
// STUCK DETECTION (front sensor only)
// ============================================================
bool checkStuck(float f) {
  if (stuckUsed) return false;

  unsigned long now = millis();

  if (!frontValid() || f <= 0) {
    stuckTimerActive = false;
    return false;
  }

  bool fStuck = stuckFrontRef >= 0 && abs(f - stuckFrontRef) < US_STUCK_BAND_CM;

  if (fStuck) {
    if (!stuckTimerActive) {
      stuckTimerActive = true;
      stuckSinceMs     = now;
    } else if (now - stuckSinceMs > US_STUCK_MS) {
      Serial.println("[STUCK] Detected! Reversing then full power push...");
      setMotors(-US_STUCK_PUSH_PWM, -US_STUCK_PUSH_PWM);
      delay(US_STUCK_REVERSE_MS);
      setMotors(US_STUCK_PUSH_PWM, US_STUCK_PUSH_PWM);
      delay(US_STUCK_PUSH_MS);
      stopMotors();
      Serial.println("[STUCK] Done. Stuck detection now permanently disabled.");
      stuckUsed = true;
      return true;
    }
  } else {
    stuckTimerActive = false;
    stuckFrontRef    = f;
  }

  return false;
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

  pinMode(TRIG_FRONT_PIN, OUTPUT); pinMode(ECHO_FRONT_PIN, INPUT);
  pinMode(TRIG_RIGHT_PIN, OUTPUT); pinMode(ECHO_RIGHT_PIN, INPUT);
  digitalWrite(TRIG_FRONT_PIN, LOW);
  digitalWrite(TRIG_RIGHT_PIN, LOW);

  for (int i = 0; i < US_SMA_WINDOW; i++) { frontBuf[i] = 0; rightBuf[i] = 0; }

  Serial.println("=== TEST 2: WALL FOLLOW ===");
  Serial.print("SMA window: "); Serial.print(US_SMA_WINDOW); Serial.println(" samples");
  Serial.print("Wall target: "); Serial.print(WF_RIGHT_NEAR_CM, 1);
  Serial.print("cm  +/- "); Serial.print(WF_RIGHT_TOLERANCE_CM, 1); Serial.println("cm");
  Serial.println("Place robot beside a right wall. Starting in 3s...");
  delay(3000);
  Serial.println("GO!");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  updateUltrasonics();

  float f = getFrontCm();
  float r = getRightCm();

  bool frontNear = frontValid() && f > 0 && f < WF_FRONT_NEAR_CM;
  bool rightNear = rightValid() && r > 0 && r < WF_RIGHT_NEAR_CM;

  // Distance maintenance zones (only relevant when front is FAR)
  float target    = WF_RIGHT_NEAR_CM;
  bool  tooClose  = rightValid() && r > 0 && r < (target - WF_RIGHT_TOLERANCE_CM);
  bool  tooFar    = rightValid() && r > 0 && r > (target + WF_RIGHT_TOLERANCE_CM);
  bool  inBand    = rightValid() && r > 0 && !tooClose && !tooFar;

  Serial.print("[WALL] Front=");
  if (frontValid()) { Serial.print(f, 1); Serial.print("cm ("); Serial.print(frontNear ? "NEAR" : "FAR"); Serial.print(")"); }
  else Serial.print("---");
  Serial.print("  Right=");
  if (rightValid()) {
    Serial.print(r, 1); Serial.print("cm (");
    if      (tooClose) Serial.print("TOO CLOSE");
    else if (tooFar)   Serial.print("TOO FAR");
    else if (inBand)   Serial.print("IN BAND");
    else               Serial.print("FAR");
    Serial.print(")");
  } else Serial.print("---");
  if (stuckUsed) Serial.print("  [STUCK DISABLED]");
  Serial.print("  ->  ");

  if (checkStuck(f)) return;

  if      (frontNear && !rightNear) {
    // Front blocked, wall gone — turn right
    setMotors(-WF_TURN_SPEED, WF_TURN_SPEED);
    Serial.println("TURN RIGHT (front blocked, wall gone)");
  }
  else if (frontNear && rightNear) {
    // Corner — turn left
    setMotors(WF_TURN_SPEED, -WF_TURN_SPEED);
    Serial.println("TURN LEFT (corner)");
  }
  else if (!frontNear && rightNear) {
    // Wall detected ahead is clear — maintain distance
    if (tooClose) {
      // Too close to wall — arc left (slow right wheel)
      setMotors(WF_CORRECT_OUTER, WF_CORRECT_INNER);
      Serial.println("CORRECT LEFT (too close)");
    } else if (tooFar) {
      // Too far from wall — arc right (slow left wheel)
      setMotors(WF_CORRECT_INNER, WF_CORRECT_OUTER);
      Serial.println("CORRECT RIGHT (too far)");
    } else {
      // In band — go straight
      setMotors(WF_STRAIGHT_SPEED, WF_STRAIGHT_SPEED);
      Serial.println("STRAIGHT (in band)");
    }
  }
  else {
    // Both far — open space, go straight
    setMotors(WF_STRAIGHT_SPEED, WF_STRAIGHT_SPEED);
    Serial.println("STRAIGHT (open space)");
  }
}