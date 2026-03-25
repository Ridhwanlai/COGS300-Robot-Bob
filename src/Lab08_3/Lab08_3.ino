/*
  ============================================================
  TEST 1 v2: LINE FOLLOWING WITH SHARP CORNER RECOVERY
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

  CHANGE FROM v1:
    When all sensors lose the tape, the car now:
      1. Stops
      2. Reverses briefly
      3. Spins in the direction of the LAST known turn
      4. Keeps spinning until a sensor finds tape again (with a timeout)
    This handles sharp zigzag corners where the tape exits
    at a steep angle.
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

// --- Tunable Constants ---
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
const int           RECOVERY_SPIN_SPEED  = 130;  // PWM speed during search spin
const unsigned long RECOVERY_REVERSE_MS  = 250;  // How far to reverse before spinning
const unsigned long RECOVERY_SPIN_MAX_MS = 1500; // Max time to spin searching for tape
const unsigned long RECOVERY_STOP_MS     = 200;  // Pause between reverse and spin

// --- State tracking ---
// +1 = last turn was left, -1 = last turn was right, 0 = unknown (default left)
int lastTurnDir = 0;

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
// SENSOR READ HELPER (applies invert flags)
// ============================================================
void readSensors(int &L, int &M, int &R) {
  int rawL = digitalRead(IR_LEFT);
  int rawM = digitalRead(IR_MIDDLE);
  int rawR = digitalRead(IR_RIGHT);
  L = IR_LEFT_INVERT   ? !rawL : rawL;
  M = IR_MIDDLE_INVERT ? !rawM : rawM;
  R = IR_RIGHT_INVERT  ? !rawR : rawR;
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
// RECOVERY ROUTINE — called when all sensors lose the tape
// ============================================================
void recoverFromLostTape() {
  Serial.println("All sensors OFF tape — starting recovery...");

  // Step 1: Stop
  stopMotors();
  delay(RECOVERY_STOP_MS);

  // Step 2: Reverse a bit to back away from the corner
  Serial.println("  Reversing...");
  setMotors(-LF_REVERSE_SPEED, -LF_REVERSE_SPEED);
  delay(RECOVERY_REVERSE_MS);
  stopMotors();
  delay(RECOVERY_STOP_MS);

  // Step 3: Spin in the direction of the last known turn
  // If we were turning left before losing tape, the tape went left → spin left
  // If unknown, default to left (arbitrary, but consistent)
  int spinDir = (lastTurnDir <= 0) ? 1 : -1;  // 1 = spin left, -1 = spin right

  if (spinDir > 0) {
    Serial.println("  Spinning LEFT to find tape...");
    setMotors(-RECOVERY_SPIN_SPEED, RECOVERY_SPIN_SPEED);  // spin left in place
  } else {
    Serial.println("  Spinning RIGHT to find tape...");
    setMotors(RECOVERY_SPIN_SPEED, -RECOVERY_SPIN_SPEED);  // spin right in place
  }

  // Step 4: Keep spinning until a sensor finds tape or timeout
  unsigned long spinStart = millis();
  while (millis() - spinStart < RECOVERY_SPIN_MAX_MS) {
    if (anySensorOnTape()) {
      Serial.println("  Tape found! Resuming line following.");
      stopMotors();
      delay(50);
      return;
    }
    delay(10);  // small delay to avoid hammering the sensors
  }

  // Step 5: If we didn't find tape spinning one way, try the other
  Serial.println("  Not found — trying opposite direction...");
  stopMotors();
  delay(RECOVERY_STOP_MS);

  if (spinDir > 0) {
    setMotors(RECOVERY_SPIN_SPEED, -RECOVERY_SPIN_SPEED);  // now spin right
  } else {
    setMotors(-RECOVERY_SPIN_SPEED, RECOVERY_SPIN_SPEED);  // now spin left
  }

  spinStart = millis();
  while (millis() - spinStart < RECOVERY_SPIN_MAX_MS * 2) {  // longer timeout for opposite
    if (anySensorOnTape()) {
      Serial.println("  Tape found on opposite spin! Resuming.");
      stopMotors();
      delay(50);
      return;
    }
    delay(10);
  }

  // Step 6: Still nothing — stop and let the main loop retry
  Serial.println("  Recovery failed — stopping. Will retry...");
  stopMotors();
  delay(500);
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

  Serial.println("=== LINE FOLLOW v2 (sharp corner recovery) ===");
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
  Serial.print("  ->  ");

  if (LF_USE_FILTER) {
    for (int i = 0; i < LF_FILTER_READS; i++) {
      delay(LF_FILTER_DELAY);
      int nL, nM, nR;
      readSensors(nL, nM, nR);
      if (nL != L || nM != M || nR != R) {
        Serial.println("UNSTABLE (skipped)");
        return;
      }
    }
  }

  // Normal line following truth table
  if      (L==HIGH && M==HIGH && R==HIGH) { lf_goStraight();    Serial.println("Junction - straight"); }
  else if (L==HIGH && M==HIGH && R==LOW)  { lf_softTurnLeft();  Serial.println("Soft Left");   lastTurnDir = 1; }
  else if (L==HIGH && M==LOW  && R==HIGH) { lf_goStraight();    Serial.println("Cross - straight"); }
  else if (L==HIGH && M==LOW  && R==LOW)  { lf_hardTurnLeft();  Serial.println("Hard Left");   lastTurnDir = 1; }
  else if (L==LOW  && M==HIGH && R==HIGH) { lf_softTurnRight(); Serial.println("Soft Right");  lastTurnDir = -1; }
  else if (L==LOW  && M==HIGH && R==LOW)  { lf_goStraight();    Serial.println("Straight"); }
  else if (L==LOW  && M==LOW  && R==HIGH) { lf_hardTurnRight(); Serial.println("Hard Right");  lastTurnDir = -1; }
  else if (L==LOW  && M==LOW  && R==LOW)  {
    recoverFromLostTape();
  }
}
