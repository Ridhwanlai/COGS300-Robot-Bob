/*
  ============================================================
  TEST 1: LINE FOLLOWING ONLY
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

  SENSOR INVERT FLAGS:
    Set to true if that sensor reads HIGH when off tape, LOW when on tape.

  TURN LOGIC:
    - Left/right directions are normal
    - Soft and hard are swapped from original:
        was hardTurnLeft  -> now softTurnLeft
        was softTurnLeft  -> now hardTurnLeft
        was hardTurnRight -> now softTurnRight
        was softTurnRight -> now hardTurnRight

  ON ALL SENSORS OFF TAPE (L L L):
    - Stop
    - Wait 500ms
    - Reverse for 200ms
    - Stop (and loop resumes normal line following)
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
const int  LF_REVERSE_SPEED   = 90;   // Speed for the reverse pulse

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
// LINE FOLLOW ACTIONS
// ============================================================
void lf_goStraight()    { setMotors(LF_BASE_SPEED + LF_LEFT_TRIM, LF_BASE_SPEED + LF_RIGHT_TRIM); }
void lf_hardTurnRight() { setMotors(LF_TURN_SPEED_FAST, 0); }
void lf_hardTurnLeft()  { setMotors(0, LF_TURN_SPEED_FAST); }
void lf_softTurnRight() { setMotors(LF_SOFT_OUTER, LF_SOFT_INNER); delay(40); }
void lf_softTurnLeft()  { setMotors(LF_SOFT_INNER, LF_SOFT_OUTER); delay(40); }

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

  Serial.println("=== TEST 1: LINE FOLLOW ===");
  Serial.println("Starting in 3s...");
  delay(3000);
  Serial.println("GO!");
  Serial.println("Format: RAW L M R  |  FINAL L M R  ->  action");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  int rawL = digitalRead(IR_LEFT);
  int rawM = digitalRead(IR_MIDDLE);
  int rawR = digitalRead(IR_RIGHT);

  int L = IR_LEFT_INVERT   ? !rawL : rawL;
  int M = IR_MIDDLE_INVERT ? !rawM : rawM;
  int R = IR_RIGHT_INVERT  ? !rawR : rawR;

  Serial.print("RAW L="); Serial.print(rawL);
  Serial.print(" M=");     Serial.print(rawM);
  Serial.print(" R=");     Serial.print(rawR);
  Serial.print("  |  FINAL L="); Serial.print(L);
  Serial.print(" M=");           Serial.print(M);
  Serial.print(" R=");           Serial.print(R);
  Serial.print("  ->  ");

  if (LF_USE_FILTER) {
    for (int i = 0; i < LF_FILTER_READS; i++) {
      delay(LF_FILTER_DELAY);
      int nL = IR_LEFT_INVERT   ? !digitalRead(IR_LEFT)   : digitalRead(IR_LEFT);
      int nM = IR_MIDDLE_INVERT ? !digitalRead(IR_MIDDLE) : digitalRead(IR_MIDDLE);
      int nR = IR_RIGHT_INVERT  ? !digitalRead(IR_RIGHT)  : digitalRead(IR_RIGHT);
      if (nL != L || nM != M || nR != R) {
        Serial.println("UNSTABLE (skipped)");
        return;
      }
    }
  }

  if      (L==HIGH && M==HIGH && R==HIGH) { lf_goStraight();    Serial.println("Junction - straight"); }
  else if (L==HIGH && M==HIGH && R==LOW)  { lf_softTurnLeft();  Serial.println("Soft Left"); }
  else if (L==HIGH && M==LOW  && R==HIGH) { lf_goStraight();    Serial.println("Cross - straight"); }
  else if (L==HIGH && M==LOW  && R==LOW)  { lf_hardTurnLeft();  Serial.println("Hard Left"); }
  else if (L==LOW  && M==HIGH && R==HIGH) { lf_softTurnRight(); Serial.println("Soft Right"); }
  else if (L==LOW  && M==HIGH && R==LOW)  { lf_goStraight();    Serial.println("Straight"); }
  else if (L==LOW  && M==LOW  && R==HIGH) { lf_hardTurnRight(); Serial.println("Hard Right"); }
  else if (L==LOW  && M==LOW  && R==LOW)  {
    Serial.println("All sensors OFF tape. Stopping...");
    stopMotors();
    delay(700);
    Serial.println("Reversing...");
    setMotors(-LF_REVERSE_SPEED, -LF_REVERSE_SPEED);
    delay(200);
    stopMotors();
    Serial.println("Done reversing. Resuming...");
  }
}