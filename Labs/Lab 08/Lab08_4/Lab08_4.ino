/*
  ============================================================
  TEST 1 v3: LINE FOLLOWING WITH STUCK DETECTION
  ============================================================

  PIN ASSIGNMENTS:
    Left   IR -> D9
    Middle IR -> D12
    Right  IR -> D11

    ENA (Left  PWM) -> D5
    ENB (Right PWM) -> D6
    IN1 (Left  Fwd) -> D2
    IN2 (Left  Bck) -> D4
    IN3 (Right Fwd) -> D7
    IN4 (Right Bck) -> D8

  CHANGES FROM v2:
    - Stuck detection: counts how many times recovery triggers
      within a time window. If it exceeds a threshold, the car
      does a tank turn (spin in place) instead of the normal
      reverse-and-resume cycle, to break out of the corner.
  ============================================================
*/

// --- Motor Pins ---
const int L_PWM_PIN = 5;
const int R_PWM_PIN = 6;
const int PIN_LF    = 2;
const int PIN_LB    = 4;
const int PIN_RF    = 7;
const int PIN_RB    = 8;

// --- IR Sensor Pins ---
const int IR_LEFT   = 9;
const int IR_MIDDLE = 12;
const int IR_RIGHT  = 11;

// --- Sensor Invert Flags ---
const bool IR_LEFT_INVERT   = true;
const bool IR_MIDDLE_INVERT = true;
const bool IR_RIGHT_INVERT  = true;

// --- Line Following Constants ---
const int  LF_BASE_SPEED      = 90;
const int  LF_TURN_SPEED_FAST = 160;
const int  LF_SOFT_OUTER      = 97;
const int  LF_SOFT_INNER      = 93;
const int  LF_RIGHT_TRIM      = 0;
const int  LF_LEFT_TRIM       = 0;
const bool LF_USE_FILTER      = false;
const int  LF_FILTER_READS    = 2;
const int  LF_FILTER_DELAY    = 5;
const int  LF_REVERSE_SPEED   = 90;

// --- Recovery Constants ---
const int           RECOVERY_SPIN_SPEED  = 130;
const unsigned long RECOVERY_REVERSE_MS  = 250;
const unsigned long RECOVERY_SPIN_MAX_MS = 1500;
const unsigned long RECOVERY_STOP_MS     = 200;

// --- Stuck Detection Constants ---
const int           STUCK_COUNT_THRESHOLD = 3;      // After this many recoveries in the time window, do tank turn
const unsigned long STUCK_TIME_WINDOW_MS  = 5000;   // Time window to count recoveries (ms)
const int           TANK_TURN_SPEED       = 150;    // PWM speed for the tank turn
const unsigned long TANK_TURN_MS          = 500;    // How long to tank turn (ms) — tune this for ~90 degrees
const unsigned long TANK_TURN_SPIN_MAX_MS = 2000;   // After tank turn, max time to spin-search for tape

// --- State tracking ---
int           lastTurnDir       = 0;    // +1 = left, -1 = right, 0 = unknown
int           recoveryCount     = 0;    // How many recoveries in current window
unsigned long firstRecoveryTime = 0;    // When the current stuck window started

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
// SENSOR HELPERS
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
// LINE FOLLOW ACTIONS
// ============================================================
void lf_goStraight()    { setMotors(LF_BASE_SPEED + LF_LEFT_TRIM, LF_BASE_SPEED + LF_RIGHT_TRIM); }
void lf_hardTurnRight() { setMotors(LF_TURN_SPEED_FAST, 0); }
void lf_hardTurnLeft()  { setMotors(0, LF_TURN_SPEED_FAST); }
void lf_softTurnRight() { setMotors(LF_SOFT_OUTER, LF_SOFT_INNER); delay(40); }
void lf_softTurnLeft()  { setMotors(LF_SOFT_INNER, LF_SOFT_OUTER); delay(40); }

// ============================================================
// TANK TURN ESCAPE — aggressive turn when stuck is detected
// ============================================================
void tankTurnEscape() {
  Serial.println("=== STUCK DETECTED — TANK TURN ===");

  stopMotors();
  delay(RECOVERY_STOP_MS);

  // Reverse a bit first
  Serial.println("  Reversing...");
  setMotors(-LF_REVERSE_SPEED, -LF_REVERSE_SPEED);
  delay(RECOVERY_REVERSE_MS);
  stopMotors();
  delay(RECOVERY_STOP_MS);

  // Tank turn: spin in place in the last known turn direction
  int spinDir = (lastTurnDir >= 0) ? 1 : -1;

  if (spinDir > 0) {
    Serial.println("  Tank turn LEFT...");
    setMotors(-TANK_TURN_SPEED, TANK_TURN_SPEED);
  } else {
    Serial.println("  Tank turn RIGHT...");
    setMotors(TANK_TURN_SPEED, -TANK_TURN_SPEED);
  }

  // Spin for the fixed duration first (to get past the corner)
  delay(TANK_TURN_MS);

  // Then keep spinning until we find tape (or timeout)
  unsigned long t0 = millis();
  while (millis() - t0 < TANK_TURN_SPIN_MAX_MS) {
    if (anySensorOnTape()) {
      Serial.println("  Tape found after tank turn! Resuming.");
      stopMotors();
      delay(50);
      // Reset stuck counter
      recoveryCount = 0;
      firstRecoveryTime = 0;
      return;
    }
    delay(5);
  }

  // If first direction failed, try opposite
  Serial.println("  Trying opposite direction...");
  stopMotors();
  delay(RECOVERY_STOP_MS);

  if (spinDir > 0) {
    setMotors(TANK_TURN_SPEED, -TANK_TURN_SPEED);
  } else {
    setMotors(-TANK_TURN_SPEED, TANK_TURN_SPEED);
  }

  t0 = millis();
  while (millis() - t0 < TANK_TURN_SPIN_MAX_MS * 2) {
    if (anySensorOnTape()) {
      Serial.println("  Tape found on opposite tank turn! Resuming.");
      stopMotors();
      delay(50);
      recoveryCount = 0;
      firstRecoveryTime = 0;
      return;
    }
    delay(5);
  }

  Serial.println("  Tank turn failed. Stopping.");
  stopMotors();
  recoveryCount = 0;
  firstRecoveryTime = 0;
  delay(500);
}

// ============================================================
// NORMAL RECOVERY — reverse and resume (used before stuck threshold)
// ============================================================
void normalRecovery() {
  Serial.print("All sensors OFF tape — recovery #");
  Serial.println(recoveryCount);

  stopMotors();
  delay(RECOVERY_STOP_MS);

  Serial.println("  Reversing...");
  setMotors(-LF_REVERSE_SPEED, -LF_REVERSE_SPEED);
  delay(RECOVERY_REVERSE_MS);
  stopMotors();
  delay(RECOVERY_STOP_MS);
  Serial.println("  Done. Resuming...");
}

// ============================================================
// MAIN RECOVERY ENTRY — decides normal vs tank turn
// ============================================================
void recoverFromLostTape() {
  unsigned long now = millis();

  // If this is the first recovery, or the window has expired, reset the counter
  if (recoveryCount == 0 || (now - firstRecoveryTime) > STUCK_TIME_WINDOW_MS) {
    recoveryCount = 1;
    firstRecoveryTime = now;
  } else {
    recoveryCount++;
  }

  Serial.print("[Stuck counter: ");
  Serial.print(recoveryCount);
  Serial.print("/");
  Serial.print(STUCK_COUNT_THRESHOLD);
  Serial.print(" in ");
  Serial.print(now - firstRecoveryTime);
  Serial.println("ms]");

  if (recoveryCount >= STUCK_COUNT_THRESHOLD) {
    // We've been going back and forth too many times — do a tank turn
    tankTurnEscape();
  } else {
    // Normal recovery — just reverse and resume
    normalRecovery();
  }
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

  Serial.println("=== LINE FOLLOW v3 (stuck detection) ===");
  Serial.println("Starting in 3s...");
  delay(3000);
  Serial.println("GO!");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  int L, M, R;
  readSensors(L, M, R);

  Serial.print("L="); Serial.print(L);
  Serial.print(" M="); Serial.print(M);
  Serial.print(" R="); Serial.print(R);
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
  else if (L==HIGH && M==HIGH && R==LOW)  { lf_softTurnLeft();  Serial.println("Soft L");  lastTurnDir =  1; }
  else if (L==HIGH && M==LOW  && R==HIGH) { lf_goStraight();    Serial.println("Cross"); }
  else if (L==HIGH && M==LOW  && R==LOW)  { lf_hardTurnLeft();  Serial.println("Hard L");  lastTurnDir =  1; }
  else if (L==LOW  && M==HIGH && R==HIGH) { lf_softTurnRight(); Serial.println("Soft R");  lastTurnDir = -1; }
  else if (L==LOW  && M==HIGH && R==LOW)  { lf_goStraight();    Serial.println("Straight"); }
  else if (L==LOW  && M==LOW  && R==HIGH) { lf_hardTurnRight(); Serial.println("Hard R");  lastTurnDir = -1; }
  else if (L==LOW  && M==LOW  && R==LOW)  {
    recoverFromLostTape();
  }
}
