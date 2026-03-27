// ============================================================
// USED FOR LAB 11 TOURNAMENT
//COMBINED v3: LINE FOLLOW → WALL FOLLOW → EXIT → HALT
// ============================================================

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
enum RobotMode {
  MODE_LINE_FOLLOW,
  MODE_WALL_FOLLOW,
  MODE_HALT
};
RobotMode currentMode = MODE_LINE_FOLLOW;

// ============================================================
// TRANSITION CONSTANTS (LF → WF)
// ============================================================
const unsigned long TRANSITION_COAST_MS      = 300;
const float         TRANSITION_RIGHT_WALL_CM = 20.0f;

// ============================================================
// LINE FOLLOWING CONSTANTS (From Code 2)
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

// When line is lost: left wheel slightly faster than right → gentle rightward drift.
// Increase the gap between LF_SEEK_LEFT and LF_SEEK_RIGHT for a sharper arc,
// decrease it for a shallower arc. Keep both values moderate to avoid spinning.
const int  LF_SEEK_LEFT       = 105;   // left wheel speed while seeking line
const int  LF_SEEK_RIGHT      = 88;    // right wheel speed while seeking line

// ============================================================
// WALL FOLLOWING CONSTANTS (From Code 1)
// ============================================================
const float         WF_FRONT_NEAR_CM      = 11.0f;
const float         WF_RIGHT_NEAR_CM      = 8.0f;
const float         WF_RIGHT_TOLERANCE_CM = 1.0f;
const int           WF_STRAIGHT_SPEED     = 110;
const int           WF_TURN_SPEED         = 120;
const int           WF_CORRECT_OUTER      = 110;
const int           WF_CORRECT_INNER      = 60;

// ── EXIT CONDITION (WF → HALT) ──────────────────────────────
const float         WF_EXIT_RIGHT_CM      = 100.0f;  // right > this = wall ended
const unsigned long WF_EXIT_DEBOUNCE_MS   = 500UL;   // wall must be gone this long
const int           EXIT_STOP_MS          = 500;     // settle after stopping
const int           EXIT_PRE_TURN_MS      = 300;     // delay before turn fires
const int           EXIT_TURN_SPEED       = 150;     // dedicated turn speed
const int           EXIT_TURN_MS          = 500;     // tune for 90 degrees on your surface
const int           EXIT_STRAIGHT_MS      = 3000;    // straight after turn

unsigned long       rightGoneSinceMs      = 0;
bool                rightGoneTimerActive  = false;
bool                wfStraightDone        = false;

// ============================================================
// WALL FOLLOW STUCK DETECTION CONSTANTS
// ============================================================
const unsigned long WF_STUCK_MS              = 1000UL;
const float         WF_STUCK_BAND_CM         = 1.5f;
const int           WF_STUCK_REVERSE_PWM_1   = 255;
const int           WF_STUCK_REVERSE_MS_1    = 500;
const int           WF_STUCK_PUSH_PWM_1      = 255;
const int           WF_STUCK_PUSH_MS_1       = 1500;
const int           WF_STUCK_REVERSE_PWM_2_L = 200;
const int           WF_STUCK_REVERSE_PWM_2_R = 120;
const int           WF_STUCK_REVERSE_MS_2    = 250;

// ============================================================
// ULTRASONIC FILTER + TIMING
// ============================================================
#define US_SMA_WINDOW 4

const unsigned long US_PULSE_TIMEOUT = 30000UL;
const float         US_MIN_CM        = 2.0f;
const float         US_MAX_CM        = 400.0f;
const unsigned long US_HOLD_MS       = 200;
const unsigned long US_PERIOD_MS     = 60;

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
// STUCK DETECTION STATE
// ============================================================
float         wfStuckFrontRef    = -1.0f;
unsigned long wfStuckSinceMs     = 0;
bool          wfStuckTimerActive = false;
bool          wfStuckFirstFired  = false;

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
// IR HELPERS
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
// ULTRASONIC HELPERS
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
// TRANSITION CHECK: LF → WF
// ============================================================
bool checkTransitionToWallFollow() {
  Serial.println("[TRANSITION] IR off — coasting straight...");
  setMotors(LF_BASE_SPEED + LF_LEFT_TRIM, LF_BASE_SPEED + LF_RIGHT_TRIM);
  delay(TRANSITION_COAST_MS);
  stopMotors();

  updateUltrasonics();
  delay(US_PERIOD_MS + 10);
  updateUltrasonics();

  float r = getRightCm();
  Serial.print("[TRANSITION] Right sensor after coast: ");
  if (rightValid() && r > 0) { Serial.print(r, 1); Serial.println("cm"); }
  else                        { Serial.println("no reading"); }

  if (rightValid() && r > 0 && r < TRANSITION_RIGHT_WALL_CM) {
    Serial.println(">>> Wall confirmed — switching to WALL FOLLOW <<<");
    currentMode = MODE_WALL_FOLLOW;
    return true;
  }

  Serial.println("[TRANSITION] No wall — resuming LF recovery.");
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
// RECOVERY: drift gently right until line is found again.
// Left wheel (LF_SEEK_LEFT) runs faster than right (LF_SEEK_RIGHT),
// producing a shallow rightward arc. 
// ============================================================
void recoverFromLostTape() {
  if (checkTransitionToWallFollow()) return;

  Serial.println("All sensors OFF line — drifting right to seek line...");

  while (!anySensorOnTape()) {
    setMotors(LF_SEEK_LEFT, LF_SEEK_RIGHT);
    Serial.println("  Still off line — drifting right...");
  }
  stopMotors();
  Serial.println("  Line found. Resuming.");
}

// ============================================================
// STUCK DETECTION
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
        Serial.println("[STUCK-1] Reversing then full power push...");
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
// EXIT WALL FOLLOW
// 1. Stop + settle
// 2. Pre-turn pause
// 3. 90 degree right turn
// 4. Check front > 300cm
// 5. Drive straight 3000ms
// 6. HALT
// ============================================================
void exitWallFollow() {
  Serial.println(">>> EXITING WALL FOLLOW <<<");

  // STEP 1: Stop and settle
  Serial.println("[EXIT] Stopping...");
  stopMotors();
  delay(EXIT_STOP_MS);
  Serial.println("[EXIT] Stopped");

  // STEP 2: Pre-turn pause
  Serial.println("[EXIT] Pre-turn pause...");
  delay(EXIT_PRE_TURN_MS);

  // STEP 3: 90 degree right turn
  Serial.println("[EXIT] Turning right 90 degrees...");
  setMotors(EXIT_TURN_SPEED, -EXIT_TURN_SPEED);
  delay(EXIT_TURN_MS);
  stopMotors();
  delay(200);
  Serial.println("[EXIT] Turn complete");

  // STEP 4: Wait until front > 300cm
  Serial.println("[EXIT] Checking front sensor...");
  while (true) {
    float frontCheck = readUsCm(TRIG_FRONT_PIN, ECHO_FRONT_PIN);
    Serial.print("[EXIT] front="); Serial.println(frontCheck);
    if (frontCheck > 300.0f || frontCheck < 0) break;
  }
  Serial.println("[EXIT] Front clear (>300cm)");

  // STEP 5: Drive straight 3000ms
  Serial.print("[EXIT] Driving straight "); Serial.print(EXIT_STRAIGHT_MS); Serial.println("ms...");
  setMotors(WF_STRAIGHT_SPEED, WF_STRAIGHT_SPEED);
  delay(EXIT_STRAIGHT_MS);

  // STEP 6: Halt
  stopMotors();
  Serial.println("[EXIT] Done");
  Serial.println(">>> HALT <<<");
  currentMode = MODE_HALT;
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

  Serial.println("=== COMBINED v3: LINE FOLLOW -> WALL FOLLOW -> EXIT -> HALT ===");
  Serial.println("LF->WF:  IR all off + right wall < 20cm");
  Serial.print("WF->EXIT: right > "); Serial.print(WF_EXIT_RIGHT_CM); Serial.println("cm for 500ms");
  Serial.print("Exit turn:    "); Serial.print(EXIT_TURN_MS);      Serial.println("ms at speed 150");
  Serial.print("Exit straight:"); Serial.print(EXIT_STRAIGHT_MS);  Serial.println("ms");
  Serial.println("Starting in 3s...");
  delay(3000);
  Serial.println("GO!");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {

  updateUltrasonics();

  // ── HALT ──────────────────────────────────────────────────
  if (currentMode == MODE_HALT) {
    stopMotors();
    return;
  }

  // ── LINE FOLLOW ───────────────────────────────────────────
  if (currentMode == MODE_LINE_FOLLOW) {
    int L, M, R;
    readSensors(L, M, R);

    Serial.print("[LF] L="); Serial.print(L);
    Serial.print(" M=");     Serial.print(M);
    Serial.print(" R=");     Serial.print(R);

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
      Serial.println("OFF LINE");
      recoverFromLostTape();
    }

    return;
  }

  // ── WALL FOLLOW ───────────────────────────────────────────
  if (currentMode == MODE_WALL_FOLLOW) {

    float f = getFrontCm();
    float r = getRightCm();

    bool rightVisible = rightValid() && r > 0;
    bool frontNear    = frontValid() && f > 0 && f < WF_FRONT_NEAR_CM;

    float target   = WF_RIGHT_NEAR_CM;
    bool  tooClose = rightVisible && r < (target - WF_RIGHT_TOLERANCE_CM);
    bool  tooFar   = rightVisible && r > (target + WF_RIGHT_TOLERANCE_CM);

    // ── EXIT CHECK (debounced 500ms) ─────────────────────────
    bool rightWallGone = !rightValid() || r <= 0 || r > WF_EXIT_RIGHT_CM;

    if (rightWallGone) {
      if (!rightGoneTimerActive) {
        rightGoneTimerActive = true;
        rightGoneSinceMs     = millis();
        Serial.print("[EXIT CHECK] Right > ");
        Serial.print(WF_EXIT_RIGHT_CM);
        Serial.println("cm — debounce started");
      } else if (millis() - rightGoneSinceMs > WF_EXIT_DEBOUNCE_MS) {
        Serial.println("[EXIT CHECK] Debounce passed — exiting WF");
        wfStraightDone = true;
        stopMotors();
        delay(200);
        exitWallFollow();
        return;
      }
    } else {
      if (rightGoneTimerActive) {
        Serial.println("[EXIT CHECK] Wall back — resetting debounce");
      }
      rightGoneTimerActive = false;
    }

    Serial.print("[WF] Front=");
    if (frontValid()) { Serial.print(f, 1); Serial.print("cm ("); Serial.print(frontNear ? "NEAR" : "FAR"); Serial.print(")"); }
    else Serial.print("---");
    Serial.print("  Right=");
    if (rightValid()) {
      Serial.print(r, 1); Serial.print("cm (");
      if      (tooClose) Serial.print("TOO CLOSE");
      else if (tooFar)   Serial.print("TOO FAR");
      else               Serial.print("IN BAND");
      Serial.print(")");
    } else Serial.print("---");
    Serial.print(wfStuckFirstFired ? "  [STUCK-N]" : "  [STUCK-1 armed]");
    Serial.print("  -> ");

    if (checkWfStuck(f)) return;

    if      (frontNear && rightVisible)  { setMotors(WF_TURN_SPEED, -WF_TURN_SPEED);  Serial.println("TURN LEFT (corner)");          }
    else if (frontNear && !rightVisible) { setMotors(-WF_TURN_SPEED, WF_TURN_SPEED);  Serial.println("TURN RIGHT (front, no wall)"); }
    else if (!frontNear && rightVisible) {
      if      (tooClose) { setMotors(WF_CORRECT_OUTER, WF_CORRECT_INNER);   Serial.println("CORRECT LEFT (too close)"); }
      else if (tooFar)   { setMotors(WF_CORRECT_INNER, WF_CORRECT_OUTER);   Serial.println("CORRECT RIGHT (too far)");  }
      else               { setMotors(WF_STRAIGHT_SPEED, WF_STRAIGHT_SPEED); Serial.println("STRAIGHT (in band)");       }
    }
    else {
      setMotors(WF_STRAIGHT_SPEED, WF_STRAIGHT_SPEED);
      Serial.println("STRAIGHT (no wall — keep moving)");
    }
  }
}