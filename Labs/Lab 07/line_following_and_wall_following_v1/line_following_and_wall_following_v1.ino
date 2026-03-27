//   ============================================================
//   COMBINED v2: LINE FOLLOW → WALL FOLLOW (one-way)
//   Wall following logic sourced from TEST 2.
//   ============================================================

//   PIN ASSIGNMENTS:
//     Left   IR -> D9
//     Middle IR -> D12
//     Right  IR -> D11

//     Front HC-SR04: TRIG -> A1,  ECHO -> A0
//     Right HC-SR04: TRIG -> A3,  ECHO -> A4

//     ENA (Left  PWM) -> D5
//     ENB (Right PWM) -> D6
//     IN1 (Left  Fwd) -> D2
//     IN2 (Left  Bck) -> D4
//     IN3 (Right Fwd) -> D7
//     IN4 (Right Bck) -> D8

//   STATE MACHINE:
//     MODE_LINE_FOLLOW (default)
//       → Runs line following + stuck detection as in TEST 1 v3.
//       → Ultrasonics are polled every loop but ignored for steering.
//       → Transition to MODE_WALL_FOLLOW when IR all go off:
//           (a) Coast straight for TRANSITION_COAST_MS (robot enters corridor), then
//           (b) Right ultrasonic reads a wall closer than TRANSITION_RIGHT_WALL_CM.
//         If no wall is seen after the coast, normal LF recovery runs instead.

//     MODE_WALL_FOLLOW (permanent)
//       → Runs wall following as in TEST 2.
//       → IR sensors are still read (printed to Serial) but ignored for steering.
//       → No return to line follow.
//   ============================================================

// ============================================================
// MOTOR PINS
// ============================================================
const int L_PWM_PIN = 5;
const int R_PWM_PIN = 6;
const int PIN_LF    = 2;
const int PIN_LB    = 4;
const int PIN_RF    = 7;
const int PIN_RB    = 8;

// ============================================================
// IR SENSOR PINS
// ============================================================
const int IR_LEFT   = 9;
const int IR_MIDDLE = 12;
const int IR_RIGHT  = 11;

const bool IR_LEFT_INVERT   = true;
const bool IR_MIDDLE_INVERT = true;
const bool IR_RIGHT_INVERT  = true;

// ============================================================
// ULTRASONIC PINS
// ============================================================
const int TRIG_FRONT_PIN = A1;
const int ECHO_FRONT_PIN = A0;
const int TRIG_RIGHT_PIN = A3;
const int ECHO_RIGHT_PIN = A4;

// ============================================================
// STATE MACHINE
// ============================================================
enum RobotMode { MODE_LINE_FOLLOW, MODE_WALL_FOLLOW };
RobotMode currentMode = MODE_LINE_FOLLOW;

// --- Transition constants ---
// When all IR sensors go off-tape, the robot coasts straight for TRANSITION_COAST_MS,
// then checks the right sensor. If a wall is detected within TRANSITION_RIGHT_WALL_CM,
// it switches to wall-follow. Otherwise it falls through to normal LF recovery.
const unsigned long TRANSITION_COAST_MS       = 300;   // How long to drive straight after tape ends (ms).
                                                        // Tune so the robot is inside the corridor before
                                                        // the right sensor check fires.
const float         TRANSITION_RIGHT_WALL_CM  = 20.0f; // Right sensor threshold to confirm corridor wall.
                                                        // Measured: open air ~30cm+, inside corridor ~10-14cm.
                                                        // 20cm sits cleanly between both — reliable trigger.

// ============================================================
// LINE FOLLOWING CONSTANTS
// ============================================================
const int  LF_BASE_SPEED      = 110;
const int  LF_TURN_SPEED_FAST = 100;
const int  LF_SOFT_OUTER      = 97;
const int  LF_SOFT_INNER      = 93;
const int  LF_RIGHT_TRIM      = 0;
const int  LF_LEFT_TRIM       = 0;
const bool LF_USE_FILTER      = false;
const int  LF_FILTER_READS    = 2;
const int  LF_FILTER_DELAY    = 5;
const int  LF_REVERSE_SPEED   = 90;

// --- Recovery constants ---
const unsigned long RECOVERY_STOP_MS     = 200;   // Pause before reversing (ms)
const unsigned long RECOVERY_REVERSE_MS  = 300;   // How long to reverse (ms)

// ============================================================
// WALL FOLLOWING CONSTANTS  (from TEST 2)
// ============================================================
const float          WF_FRONT_NEAR_CM       = 11.0f;  // Front obstacle threshold (cm).
const float          WF_RIGHT_NEAR_CM       = 8.0f;   // Target distance to maintain from right wall (cm).
const float          WF_RIGHT_TOLERANCE_CM  = 1.0f;   // Dead band around target (+/- this value).
const int            WF_STRAIGHT_SPEED      = 110;
const int            WF_TURN_SPEED          = 120;
const int            WF_CORRECT_OUTER       = 110;
const int            WF_CORRECT_INNER       = 60;

// ============================================================
// WALL FOLLOW: STUCK DETECTION CONSTANTS  (from TEST 2, two-tier)
// ============================================================
const unsigned long  WF_STUCK_MS              = 1000UL; // Time robot must be motionless before stuck fires (ms).
const float          WF_STUCK_BAND_CM         = 1.5f;   // Front sensor change < this = considered not moving.

// Tier 1 — straight reverse + full-power push
const int            WF_STUCK_REVERSE_PWM_1   = 255;
const int            WF_STUCK_REVERSE_MS_1    = 500;
const int            WF_STUCK_PUSH_PWM_1      = 255;
const int            WF_STUCK_PUSH_MS_1       = 1500;

// Tier 2 — angled reverse (left wheel stronger → angles robot rightward)
const int            WF_STUCK_REVERSE_PWM_2_L = 200;   // Left wheel (stronger)
const int            WF_STUCK_REVERSE_PWM_2_R = 120;   // Right wheel (weaker)
const int            WF_STUCK_REVERSE_MS_2    = 250;

// ============================================================
// ULTRASONIC FILTER + TIMING
// ============================================================
#define US_SMA_WINDOW 4   // TEST 2 used window of 4

const unsigned long  US_PULSE_TIMEOUT    = 30000UL;
const float          US_MIN_CM           = 2.0f;
const float          US_MAX_CM           = 400.0f;
const unsigned long  US_HOLD_MS          = 200;
const unsigned long  US_PERIOD_MS        = 60;

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

// ============================================================
// WALL FOLLOW: STUCK DETECTION STATE
// ============================================================
float         wfStuckFrontRef    = -1.0f;
unsigned long wfStuckSinceMs     = 0;
bool          wfStuckTimerActive = false;
bool          wfStuckFirstFired  = false;  // false = tier 1 armed, true = tier 2+ active

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
// SENSOR HELPERS — IR
// ============================================================
void readSensors(int &L, int &M, int &R) {
  L = IR_LEFT_INVERT   ? !digitalRead(IR_LEFT)   : digitalRead(IR_LEFT);
  M = IR_MIDDLE_INVERT ? !digitalRead(IR_MIDDLE) : digitalRead(IR_MIDDLE);
  R = IR_RIGHT_INVERT  ? !digitalRead(IR_RIGHT)  : digitalRead(IR_RIGHT);
}

bool anySensorOnTape() {
  int L, M, R;
  readSensors(L, M, R);
  return (L == HIGH || M == HIGH || R == HIGH);
}

// ============================================================
// SENSOR HELPERS — ULTRASONIC
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

// Polls both ultrasonics on a fixed period — called every loop() in both modes.
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
// TRANSITION CHECK
// Called from recoverFromLostTape() the moment all IR go off.
// Sequence:
//   1. Coast straight for TRANSITION_COAST_MS — robot enters the corridor.
//   2. Force a fresh ultrasonic reading.
//   3. Right sensor < TRANSITION_RIGHT_WALL_CM → switch to wall-follow.
//   4. Otherwise → return false, normal LF recovery takes over.
//
// Front sensor plays no role in the transition decision.
// ============================================================
bool checkTransitionToWallFollow() {
  Serial.println("[TRANSITION] IR off — coasting straight...");
  setMotors(LF_BASE_SPEED + LF_LEFT_TRIM, LF_BASE_SPEED + LF_RIGHT_TRIM);
  delay(TRANSITION_COAST_MS);
  stopMotors();

  // Two update calls so the SMA gets at least one fresh sample
  updateUltrasonics();
  delay(US_PERIOD_MS + 10);
  updateUltrasonics();

  float r = getRightCm();
  Serial.print("[TRANSITION] Right sensor after coast: ");
  if (rightValid() && r > 0) { Serial.print(r, 1); Serial.println("cm"); }
  else                        { Serial.println("no reading"); }

  if (rightValid() && r > 0 && r < TRANSITION_RIGHT_WALL_CM) {
    Serial.println(">>> Wall on right confirmed — switching to WALL FOLLOW <<<");
    currentMode = MODE_WALL_FOLLOW;
    return true;
  }

  Serial.println("[TRANSITION] No wall detected — resuming LF recovery.");
  return false;
}

// ============================================================
// LINE FOLLOW ACTIONS
// ============================================================
void lf_goStraight()    { setMotors(LF_BASE_SPEED + LF_LEFT_TRIM, LF_BASE_SPEED + LF_RIGHT_TRIM); }
void lf_hardTurnRight() { setMotors(LF_TURN_SPEED_FAST, 0); }
void lf_hardTurnLeft()  { setMotors(0, LF_TURN_SPEED_FAST); }
void lf_softTurnRight() { setMotors(LF_SOFT_OUTER, LF_SOFT_INNER); delay(40); }
void lf_softTurnLeft()  { setMotors(LF_SOFT_INNER, LF_SOFT_OUTER); delay(40); }

// ============================================================
// LINE FOLLOW: RECOVERY — simple stop → reverse, no spinning
// Transition check runs first — if wall found, switches mode
// and returns immediately without reversing.
// ============================================================
void recoverFromLostTape() {
  if (checkTransitionToWallFollow()) return;

  Serial.println("All sensors OFF tape — reversing until tape found...");
  stopMotors();
  delay(RECOVERY_STOP_MS);

  // Keep reversing in short pulses until any sensor sees tape again.
  // No spinning — robot stays oriented and just backs up.
  while (!anySensorOnTape()) {
    setMotors(-LF_REVERSE_SPEED, -LF_REVERSE_SPEED);
    delay(RECOVERY_REVERSE_MS);
    stopMotors();
    delay(50);
    Serial.println("  Still off tape — reversing again...");
  }

  Serial.println("  Tape found. Resuming.");
}

// ============================================================
// WALL FOLLOW: STUCK DETECTION  (from TEST 2, two-tier)
//   Tier 1 (first trigger): straight reverse + full-power push forward.
//   Tier 2+ (subsequent):   angled reverse (left > right → angles rightward).
//   Timer resets after each recovery so the robot can trigger again if needed.
// ============================================================
bool checkWfStuck(float f) {
  unsigned long now = millis();

  if (!frontValid() || f <= 0) {
    wfStuckTimerActive = false;
    return false;
  }

  bool fStuck = wfStuckFrontRef >= 0 && abs(f - wfStuckFrontRef) < WF_STUCK_BAND_CM;

  if (fStuck) {
    if (!wfStuckTimerActive) {
      wfStuckTimerActive = true;
      wfStuckSinceMs     = now;
    } else if (now - wfStuckSinceMs > WF_STUCK_MS) {
      if (!wfStuckFirstFired) {
        Serial.println("[STUCK-1] First stuck! Reversing then full power push...");
        setMotors(-WF_STUCK_REVERSE_PWM_1, -WF_STUCK_REVERSE_PWM_1);
        delay(WF_STUCK_REVERSE_MS_1);
        setMotors(WF_STUCK_PUSH_PWM_1, WF_STUCK_PUSH_PWM_1);
        delay(WF_STUCK_PUSH_MS_1);
        stopMotors();
        Serial.println("[STUCK-1] Done.");
        wfStuckFirstFired = true;
      } else {
        Serial.println("[STUCK-N] Angled reverse right...");
        setMotors(-WF_STUCK_REVERSE_PWM_2_R, -WF_STUCK_REVERSE_PWM_2_L);
        delay(WF_STUCK_REVERSE_MS_2);
        stopMotors();
        Serial.println("[STUCK-N] Done.");
      }
      wfStuckTimerActive = false;
      wfStuckFrontRef    = -1.0f;
      return true;
    }
  } else {
    wfStuckTimerActive = false;
    wfStuckFrontRef    = f;
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

  pinMode(IR_LEFT,   INPUT);
  pinMode(IR_MIDDLE, INPUT);
  pinMode(IR_RIGHT,  INPUT);

  pinMode(TRIG_FRONT_PIN, OUTPUT); pinMode(ECHO_FRONT_PIN, INPUT);
  pinMode(TRIG_RIGHT_PIN, OUTPUT); pinMode(ECHO_RIGHT_PIN, INPUT);
  digitalWrite(TRIG_FRONT_PIN, LOW);
  digitalWrite(TRIG_RIGHT_PIN, LOW);

  for (int i = 0; i < US_SMA_WINDOW; i++) { frontBuf[i] = 0; rightBuf[i] = 0; }

  Serial.println("=== COMBINED v2: LINE FOLLOW -> WALL FOLLOW (TEST 2 WF) ===");
  Serial.println("Transition: IR off -> coast 300ms straight -> right wall < 20cm = wall follow");
  Serial.print("WF target: "); Serial.print(WF_RIGHT_NEAR_CM, 1);
  Serial.print("cm  +/- "); Serial.print(WF_RIGHT_TOLERANCE_CM, 1); Serial.println("cm");
  Serial.println("Starting in 3s...");
  delay(3000);
  Serial.println("GO!");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  // Always poll ultrasonics regardless of mode
  updateUltrasonics();

  // ---- MODE: LINE FOLLOW ----
  if (currentMode == MODE_LINE_FOLLOW) {
    int L, M, R;
    readSensors(L, M, R);

    Serial.print("[LF] L="); Serial.print(L);
    Serial.print(" M="); Serial.print(M);
    Serial.print(" R="); Serial.print(R);

    // Print ultrasonic readings passively (for monitoring only)
    float f = getFrontCm();
    Serial.print("  Front=");
    if (frontValid()) { Serial.print(f, 1); Serial.print("cm"); }
    else Serial.print("---");
    float r_diag = getRightCm();
    Serial.print("  Right=");
    if (rightValid()) { Serial.print(r_diag, 1); Serial.print("cm"); }
    else Serial.print("---");
    Serial.print(" -> ");

    if (LF_USE_FILTER) {
      for (int i = 0; i < LF_FILTER_READS; i++) {
        delay(LF_FILTER_DELAY);
        int nL, nM, nR;
        readSensors(nL, nM, nR);
        if (nL != L || nM != M || nR != R) {
          Serial.println("UNSTABLE");
          return;
        }
      }
    }

    if      (L==HIGH && M==HIGH && R==HIGH) { lf_goStraight();    Serial.println("Junction"); }
    else if (L==HIGH && M==HIGH && R==LOW)  { lf_softTurnLeft();  Serial.println("Soft L");   }
    else if (L==HIGH && M==LOW  && R==HIGH) { lf_goStraight();    Serial.println("Cross");    }
    else if (L==HIGH && M==LOW  && R==LOW)  { lf_hardTurnLeft();  Serial.println("Hard L");   }
    else if (L==LOW  && M==HIGH && R==HIGH) { lf_softTurnRight(); Serial.println("Soft R");   }
    else if (L==LOW  && M==HIGH && R==LOW)  { lf_goStraight();    Serial.println("Straight"); }
    else if (L==LOW  && M==LOW  && R==HIGH) { lf_hardTurnRight(); Serial.println("Hard R");   }
    else if (L==LOW  && M==LOW  && R==LOW)  {
      Serial.println("OFF TAPE");
      recoverFromLostTape();  // transition check is embedded here
    }

    return;  // done with line-follow iteration
  }

  // ---- MODE: WALL FOLLOW  (TEST 2 logic) ----
  float f = getFrontCm();
  float r = getRightCm();

  bool rightVisible = rightValid() && r > 0;  // Wall present if sensor has any valid reading
  bool frontNear    = frontValid() && f > 0 && f < WF_FRONT_NEAR_CM;

  float target   = WF_RIGHT_NEAR_CM;
  bool  tooClose = rightVisible && r < (target - WF_RIGHT_TOLERANCE_CM);
  bool  tooFar   = rightVisible && r > (target + WF_RIGHT_TOLERANCE_CM);
  bool  inBand   = rightVisible && !tooClose && !tooFar;

  Serial.print("[WF] Front=");
  if (frontValid()) { Serial.print(f, 1); Serial.print("cm ("); Serial.print(frontNear ? "NEAR" : "FAR"); Serial.print(")"); }
  else Serial.print("---");
  Serial.print("  Right=");
  if (rightValid()) {
    Serial.print(r, 1); Serial.print("cm (");
    if (!rightVisible) Serial.print("NO READING");
    else if (tooClose) Serial.print("TOO CLOSE");
    else if (tooFar)   Serial.print("TOO FAR");
    else               Serial.print("IN BAND");
    Serial.print(")");
  } else Serial.print("---");
  Serial.print(wfStuckFirstFired ? "  [STUCK-N]" : "  [STUCK-1 armed]");
  Serial.print("  ->  ");

  if (checkWfStuck(f)) return;

  if      (frontNear && rightVisible) {
    setMotors(WF_TURN_SPEED, -WF_TURN_SPEED);
    Serial.println("TURN LEFT (corner)");
  }
  else if (frontNear && !rightVisible) {
    setMotors(-WF_TURN_SPEED, WF_TURN_SPEED);
    Serial.println("TURN RIGHT (front blocked, no wall)");
  }
  else if (!frontNear && rightVisible) {
    if (tooClose) {
      setMotors(WF_CORRECT_OUTER, WF_CORRECT_INNER);
      Serial.println("CORRECT LEFT (too close)");
    } else if (tooFar) {
      setMotors(WF_CORRECT_INNER, WF_CORRECT_OUTER);
      Serial.println("CORRECT RIGHT (too far)");
    } else {
      setMotors(WF_STRAIGHT_SPEED, WF_STRAIGHT_SPEED);
      Serial.println("STRAIGHT (in band)");
    }
  }
  else {
    setMotors(WF_STRAIGHT_SPEED, WF_STRAIGHT_SPEED);
    Serial.println("STRAIGHT (no right reading)");
  }
}
