/*
  ============================================================
  TEST 1 v5: LINE FOLLOWING — REBALANCED TURNS
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

  CHANGES FROM v4:
    - Hard turns: slow wheel reduced from 40 to 15
      (tighter arc without fully stopping the wheel)
    - Soft turns: differential widened slightly
    - Base speed lowered to 75 for more reaction time
    - Stuck detection and tank turn retained from v3
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
const int           STUCK_COUNT_THRESHOLD = 2;
const unsigned long STUCK_TIME_WINDOW_MS  = 5000;
const int           TANK_TURN_SPEED       = 150;
const unsigned long TANK_TURN_MS          = 500;
const unsigned long TANK_TURN_SPIN_MAX_MS = 2000;

// --- State tracking ---
int           lastTurnDir       = 0;
int           recoveryCount     = 0;
unsigned long firstRecoveryTime = 0;

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
// TANK TURN ESCAPE
// ============================================================
void tankTurnEscape() {
  Serial.println("=== STUCK — TANK TURN ===");

  stopMotors();
  delay(RECOVERY_STOP_MS);

  Serial.println("  Reversing...");
  setMotors(-LF_REVERSE_SPEED, -LF_REVERSE_SPEED);
  delay(RECOVERY_REVERSE_MS);
  stopMotors();
  delay(RECOVERY_STOP_MS);

  int spinDir = (lastTurnDir >= 0) ? 1 : -1;

  if (spinDir > 0) {
    Serial.println("  Tank turn LEFT...");
    setMotors(-TANK_TURN_SPEED, TANK_TURN_SPEED);
  } else {
    Serial.println("  Tank turn RIGHT...");
    setMotors(TANK_TURN_SPEED, -TANK_TURN_SPEED);
  }

  delay(TANK_TURN_MS);

  unsigned long t0 = millis();
  while (millis() - t0 < TANK_TURN_SPIN_MAX_MS) {
    if (anySensorOnTape()) {
      Serial.println("  Tape found! Resuming.");
      stopMotors();
      delay(50);
      recoveryCount = 0;
      firstRecoveryTime = 0;
      return;
    }
    delay(5);
  }

  Serial.println("  Trying opposite...");
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
      Serial.println("  Tape found opposite! Resuming.");
      stopMotors();
      delay(50);
      recoveryCount = 0;
      firstRecoveryTime = 0;
      return;
    }
    delay(5);
  }

  Serial.println("  Tank turn failed.");
  stopMotors();
  recoveryCount = 0;
  firstRecoveryTime = 0;
  delay(500);
}

// ============================================================
// NORMAL RECOVERY
// ============================================================
void normalRecovery() {
  Serial.print("OFF tape — recovery #");
  Serial.println(recoveryCount);

  stopMotors();
  delay(RECOVERY_STOP_MS);

  Serial.println("  Reversing...");
  setMotors(-LF_REVERSE_SPEED, -LF_REVERSE_SPEED);
  delay(RECOVERY_REVERSE_MS);
  stopMotors();
  delay(RECOVERY_STOP_MS);
  Serial.println("  Resuming...");
}

// ============================================================
// RECOVERY ENTRY
// ============================================================
void recoverFromLostTape() {
  unsigned long now = millis();

  if (recoveryCount == 0 || (now - firstRecoveryTime) > STUCK_TIME_WINDOW_MS) {
    recoveryCount = 1;
    firstRecoveryTime = now;
  } else {
    recoveryCount++;
  }

  Serial.print("[Stuck: ");
  Serial.print(recoveryCount);
  Serial.print("/");
  Serial.print(STUCK_COUNT_THRESHOLD);
  Serial.println("]");

  if (recoveryCount >= STUCK_COUNT_THRESHOLD) {
    tankTurnEscape();
  } else {
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

  Serial.println("=== LINE FOLLOW v5 (rebalanced turns) ===");
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
