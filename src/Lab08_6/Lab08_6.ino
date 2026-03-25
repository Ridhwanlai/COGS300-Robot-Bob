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
    1. Front NEAR + Right NEAR  -> turn left   (corner)
    2. Front NEAR + Right FAR   -> turn right  (wall ended, front blocked)
    3. Front FAR  + Right NEAR  -> go straight (normal wall follow, maintain distance)
    4. Front FAR  + Right FAR   -> go straight (no wall, no obstacle — open space)

  THRESHOLDS:
    WF_FRONT_NEAR_CM  — front closer than this = NEAR
    WF_RIGHT_NEAR_CM  — right closer than this = NEAR
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
const float          WF_FRONT_NEAR_CM  = 25.0f;   // Front closer than this = NEAR
const float          WF_RIGHT_NEAR_CM  = 15.0f;   // Right closer than this = NEAR
const int            WF_STRAIGHT_SPEED = 160;      // Forward speed
const int            WF_TURN_SPEED     = 170;      // Speed of turning wheels during a turn

// --- Ultrasonic Filter ---
const unsigned long  US_PULSE_TIMEOUT  = 30000UL;
const float          US_MIN_CM         = 2.0f;
const float          US_MAX_CM         = 400.0f;
const float          US_ALPHA          = 0.35f;
const unsigned long  US_HOLD_MS        = 200;
const unsigned long  US_PERIOD_MS      = 60;

// ============================================================
// GLOBALS
// ============================================================
float         frontCmFilt   = -1.0f;
float         rightCmFilt   = -1.0f;
bool          frontInit     = false;
bool          rightInit     = false;
unsigned long lastFrontOkMs = 0;
unsigned long lastRightOkMs = 0;
unsigned long lastUltraMs   = 0;

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

  Serial.println("=== TEST 2: WALL FOLLOW ===");
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

  // Print sensor readings every cycle
  Serial.print("[WALL] Front=");
  if (frontValid()) { Serial.print(f, 1); Serial.print("cm ("); Serial.print(frontNear ? "NEAR" : "FAR"); Serial.print(")"); }
  else Serial.print("---");
  Serial.print("  Right=");
  if (rightValid()) { Serial.print(r, 1); Serial.print("cm ("); Serial.print(rightNear ? "NEAR" : "FAR"); Serial.print(")"); }
  else Serial.print("---");
  Serial.print("  ->  ");

  // State hierarchy
  if      (frontNear && rightNear) {
    // Corner: both sensors near → turn left
    setMotors(WF_TURN_SPEED, -WF_TURN_SPEED);
    Serial.println("TURN LEFT (corner)");
  }
  else if (frontNear && !rightNear) {
    // Wall ended but front blocked → turn right
    setMotors(-WF_TURN_SPEED, WF_TURN_SPEED);
    Serial.println("TURN RIGHT (front blocked, wall gone)");
  }
  else if (!frontNear && rightNear) {
    // Normal: wall on right, clear ahead → go straight
    setMotors(WF_STRAIGHT_SPEED, WF_STRAIGHT_SPEED);
    Serial.println("STRAIGHT (wall follow)");
  }
  else {
    // Both far: open space
    setMotors(WF_STRAIGHT_SPEED, WF_STRAIGHT_SPEED);
    Serial.println("STRAIGHT (open space)");
  }
}