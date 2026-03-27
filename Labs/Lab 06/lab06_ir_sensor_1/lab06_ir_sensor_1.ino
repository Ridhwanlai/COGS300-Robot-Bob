// ============================
// Wall follower (Lab05) + IR line sensor support (Lab06)
// - Adds IR Infrared Obstacle Avoidance / Reflection Sensor Module as a DIGITAL line sensor
// - Lets you choose: wall-follow OR line-follow / line-detect using IR
//
// IR sensor details per Lab06 slides:
//   VCC -> 5V, GND -> GND, use D (digital) output, tune with potentiometer. (Digital is easiest)
// ============================

#include <math.h>

// ---------- CHOOSE WHAT YOU WANT THE ROBOT TO DO ----------
enum RunMode { MODE_WALL_FOLLOW, MODE_LINE_FOLLOW, MODE_LINE_DETECT_TWO_STRIPS };

// Pick ONE:
const RunMode RUN_MODE = MODE_LINE_FOLLOW;
// const RunMode RUN_MODE = MODE_LINE_DETECT_TWO_STRIPS;
// const RunMode RUN_MODE = MODE_WALL_FOLLOW;

// ---------- Ultrasonic Pins ----------
const uint8_t trigFront = 1;
const uint8_t echoFront = 2;

const uint8_t trigSide  = 3;
const uint8_t echoSide  = 4;

// ---------- L298N Motor Pins ----------
// From your notes: RIGHT motor is on OUT1/OUT2 => ENA is RIGHT motor PWM.
const uint8_t ENA = 9;   // RIGHT motor PWM
const uint8_t IN1 = 7;
const uint8_t IN2 = 8;

const uint8_t ENB = 10;  // LEFT motor PWM
const uint8_t IN3 = 6;
const uint8_t IN4 = 5;

// ---------- IR Line Sensor Pins (from your wiring notes) ----------
const uint8_t irRightPin = 11;   // Right IR sensor OUT -> D11
const uint8_t irLeftPin  = 12;   // Left IR sensor OUT  -> D12 (ONLY if D12 is NOT also used by encoder!)

// Start with ONE sensor (Lab06 recommendation): use the right IR on D11.
const bool USE_RIGHT_IR = true;
const bool USE_LEFT_IR  = true; // set true ONLY if D12 isn't shared with encoder

// Many LM393 IR modules pull output LOW when "detecting" (signal LED on).
// If your readings look inverted, flip this.
const bool IR_ACTIVE_LOW = true;

// Use internal pullup to be robust with LM393/open-collector outputs
const bool IR_USE_PULLUP = true;

// ---------- WALL FOLLOW CONFIG (your original) ----------
const bool FOLLOW_RIGHT_WALL = true;
const bool INSIDE_TURN_LEFT  = true;

const bool invertLeftMotor  = false;
const bool invertRightMotor = false;

const float stopDistanceCm   = 10.0f;
const float slowPointCm      = 35.0f;
const float KpFront          = 4.0f;

const float sideSetPointCm   = 20.0f;
const float sideDeadbandCm   = 2.0f;
const float KpSide           = 3.0f;
const int   maxSteerPWM      = 90;

const float insideTriggerCm  = 22.0f;
const float frontClearCm     = 25.0f;

const float sideCornerTooCloseCm = 9.9f;
const float insideApproachCm     = 30.0f;

const int maxSpeedPWM = 200;
const int minSpeedPWM = 60;

const int turnPWM = 170;
const unsigned long minTurnMs = 300;
const unsigned long maxTurnMs = 1800;
const unsigned long recoverMs = 250;

// ---------- LINE FOLLOW TUNING (Lab06) ----------
const int lineBasePWM   = 150;  // forward speed on tape
const int lineSteerPWM  = 70;   // steering strength (bigger = harder turns)
const int lineSearchPWM = 130;  // pivot speed when line is lost

// ---------- Ultrasonic timing ----------
const unsigned long ECHO_TIMEOUT_US = 24000UL;
const unsigned long PING_GAP_MS     = 40UL;

// ---------- Internal state (wall follower) ----------
enum Mode { FOLLOW, TURN_INSIDE, RECOVER };
Mode mode = FOLLOW;
unsigned long modeStartMs = 0;

// last good distances
float frontCm = 999.0f;
float sideCm  = 999.0f;
unsigned long lastFrontOkMs = 0;
unsigned long lastSideOkMs  = 0;

// ---------- Helpers ----------
int clampPWM(int v) { return constrain(v, 0, 255); }

void setRightMotorSigned(int cmd) {
  if (invertRightMotor) cmd = -cmd;
  cmd = constrain(cmd, -255, 255);

  if (cmd > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, cmd);
  } else if (cmd < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -cmd);
  } else {
    analogWrite(ENA, 0);
  }
}

void setLeftMotorSigned(int cmd) {
  if (invertLeftMotor) cmd = -cmd;
  cmd = constrain(cmd, -255, 255);

  if (cmd > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, cmd);
  } else if (cmd < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -cmd);
  } else {
    analogWrite(ENB, 0);
  }
}

void setMotorsSigned(int leftCmd, int rightCmd) {
  setLeftMotorSigned(leftCmd);
  setRightMotorSigned(rightCmd);
}

void stopMotors() {
  setMotorsSigned(0, 0);
}

void pivotLeft(int pwm) {  // left back, right forward
  pwm = clampPWM(pwm);
  setMotorsSigned(-pwm, +pwm);
}

void pivotRight(int pwm) { // left forward, right back
  pwm = clampPWM(pwm);
  setMotorsSigned(+pwm, -pwm);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max - in_min == 0) return out_min;
  float t = (x - in_min) / (in_max - in_min);
  t = constrain(t, 0.0f, 1.0f);
  return out_min + t * (out_max - out_min);
}

// ---------- Ultrasonic read (returns true if valid) ----------
bool readUltrasonicCM(uint8_t trigPin, uint8_t echoPin, float &outCm) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return false;

  outCm = (duration * 0.0343f) / 2.0f;
  if (outCm < 1.0f || outCm > 400.0f) return false;
  return true;
}

void updateSensors() {
  static unsigned long lastPingMs = 0;
  static bool pingFrontNext = true;

  unsigned long nowMs = millis();
  if (nowMs - lastPingMs < PING_GAP_MS) return;
  lastPingMs = nowMs;

  float d;
  if (pingFrontNext) {
    if (readUltrasonicCM(trigFront, echoFront, d)) {
      frontCm = d;
      lastFrontOkMs = nowMs;
    }
  } else {
    if (readUltrasonicCM(trigSide, echoSide, d)) {
      sideCm = d;
      lastSideOkMs = nowMs;
    }
  }
  pingFrontNext = !pingFrontNext;
}

bool frontFresh() { return (millis() - lastFrontOkMs) < 250; }
bool sideFresh()  { return (millis() - lastSideOkMs)  < 250; }

void enterMode(Mode m) {
  mode = m;
  modeStartMs = millis();
}

// ---------- IR helpers ----------
bool irDetected(uint8_t pin) {
  int v = digitalRead(pin);
  return IR_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

bool lineSeen(bool &leftOn, bool &rightOn) {
  leftOn  = (USE_LEFT_IR  ? irDetected(irLeftPin)  : false);
  rightOn = (USE_RIGHT_IR ? irDetected(irRightPin) : false);
  return leftOn || rightOn;
}

// ---------- LINE FOLLOW / LINE DETECT loops ----------
int lastTurnDir = +1; // +1 = last turn right, -1 = last turn left

void doLineFollow() {
  // Optional ultrasonic safety stop:
  updateSensors();
  if (frontFresh() && frontCm <= stopDistanceCm) {
    stopMotors();
    return;
  }

  bool leftOn, rightOn;
  bool onLine = lineSeen(leftOn, rightOn);

  // Debug ~10x/sec
  static unsigned long lastPrintMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastPrintMs > 100) {
    lastPrintMs = nowMs;
    Serial.print("IR L=");
    Serial.print(leftOn);
    Serial.print(" R=");
    Serial.print(rightOn);
    Serial.print("  front=");
    Serial.print(frontCm);
    Serial.print("  side=");
    Serial.println(sideCm);
  }

  // If you only have ONE IR sensor enabled, use emergent "bounce":
  if (!(USE_LEFT_IR && USE_RIGHT_IR)) {
    if (onLine) {
      // On tape: arc one way
      setMotorsSigned(lineBasePWM - lineSteerPWM, lineBasePWM + lineSteerPWM);
      lastTurnDir = -1;
    } else {
      // Off tape: arc back the other way
      setMotorsSigned(lineBasePWM + lineSteerPWM, lineBasePWM - lineSteerPWM);
      lastTurnDir = +1;
    }
    return;
  }

  // Two-sensor detection steering:
  if (leftOn && rightOn) {
    // centered / line wide
    setMotorsSigned(lineBasePWM, lineBasePWM);
  } else if (leftOn && !rightOn) {
    // line is under LEFT sensor -> steer LEFT (turn toward line)
    lastTurnDir = -1;
    setMotorsSigned(lineBasePWM - lineSteerPWM, lineBasePWM + lineSteerPWM);
  } else if (!leftOn && rightOn) {
    // line is under RIGHT sensor -> steer RIGHT
    lastTurnDir = +1;
    setMotorsSigned(lineBasePWM + lineSteerPWM, lineBasePWM - lineSteerPWM);
  } else {
    // Lost line: pivot in last known direction to reacquire
    if (lastTurnDir < 0) pivotLeft(lineSearchPWM);
    else pivotRight(lineSearchPWM);
  }
}

void doLineDetectTwoStrips() {
  // Behavior from Lab06 step 3:
  // Start on first strip, drive forward, stop on second strip.
  bool leftOn, rightOn;
  bool onLine = lineSeen(leftOn, rightOn);

  static bool armed = false;  // becomes true after we've LEFT the start strip

  // Debug ~10x/sec
  static unsigned long lastPrintMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastPrintMs > 100) {
    lastPrintMs = nowMs;
    Serial.print("LINE-DETECT onLine=");
    Serial.print(onLine);
    Serial.print(" armed=");
    Serial.println(armed);
  }

  if (!armed) {
    // Keep driving until we are OFF the starting tape once
    setMotorsSigned(lineBasePWM, lineBasePWM);
    if (!onLine) armed = true;
    return;
  }

  // Armed: stop at the next time we hit tape
  if (onLine) {
    stopMotors();
    return;
  }

  setMotorsSigned(lineBasePWM, lineBasePWM);
}

// ---------- WALL FOLLOW loop (your original logic, unchanged) ----------
void doWallFollow() {
  updateSensors();

  unsigned long nowMs = millis();

  // Debug (print ~5x/sec)
  static unsigned long lastPrintMs = 0;
  if (nowMs - lastPrintMs > 200) {
    lastPrintMs = nowMs;
    Serial.print("front=");
    Serial.print(frontCm);
    Serial.print(" side=");
    Serial.print(sideCm);
    Serial.print(" mode=");
    Serial.println(mode);
  }

  // SAFETY: if front reading is fresh and very close, stop.
  if (frontFresh() && frontCm <= stopDistanceCm) {
    stopMotors();
    return;
  }

  if (mode == FOLLOW) {
    int baseSpeed = minSpeedPWM;

    if (frontFresh()) {
      if (frontCm < slowPointCm) {
        baseSpeed = (int)mapFloat(frontCm, stopDistanceCm, slowPointCm, minSpeedPWM, maxSpeedPWM);
      } else {
        float frontError = frontCm - stopDistanceCm;
        baseSpeed = (int)(KpFront * frontError);
      }
    }
    baseSpeed = constrain(baseSpeed, minSpeedPWM, maxSpeedPWM);

    bool insideCorner = false;

    if (frontFresh() && frontCm <= insideTriggerCm) {
      insideCorner = true;
    }

    if (frontFresh() && sideFresh() &&
        frontCm <= insideApproachCm &&
        sideCm <= sideCornerTooCloseCm) {
      insideCorner = true;
    }

    if (insideCorner) {
      enterMode(TURN_INSIDE);
      return;
    }

    int leftPWM  = baseSpeed;
    int rightPWM = baseSpeed;

    if (sideFresh()) {
      float error = sideCm - sideSetPointCm;
      if (fabs(error) < sideDeadbandCm) error = 0.0f;

      int steer = (int)(KpSide * error);
      steer = constrain(steer, -maxSteerPWM, +maxSteerPWM);

      if (FOLLOW_RIGHT_WALL) {
        leftPWM  = baseSpeed + steer;
        rightPWM = baseSpeed - steer;
      } else {
        leftPWM  = baseSpeed - steer;
        rightPWM = baseSpeed + steer;
      }
    }

    leftPWM  = constrain(leftPWM,  0, maxSpeedPWM);
    rightPWM = constrain(rightPWM, 0, maxSpeedPWM);

    setMotorsSigned(leftPWM, rightPWM);
    return;
  }

  if (mode == TURN_INSIDE) {
    unsigned long t = nowMs - modeStartMs;

    if (t > maxTurnMs) {
      enterMode(RECOVER);
      return;
    }

    if (INSIDE_TURN_LEFT) pivotLeft(turnPWM);
    else pivotRight(turnPWM);

    if (t < minTurnMs) return;

    bool frontClear = (!frontFresh()) || (frontCm >= frontClearCm);

    bool sideOk = true;
    if (sideFresh()) {
      sideOk = (fabs(sideCm - sideSetPointCm) <= (sideDeadbandCm + 4.0f));
    }

    if (frontClear && sideOk) {
      enterMode(RECOVER);
      return;
    }
    return;
  }

  if (mode == RECOVER) {
    if (nowMs - modeStartMs > recoverMs) {
      enterMode(FOLLOW);
      return;
    }

    int pwm = 120;
    setMotorsSigned(pwm, pwm);
    return;
  }
}

// ---------- setup/loop ----------
void setup() {
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigSide, OUTPUT);
  pinMode(echoSide, INPUT);

  digitalWrite(trigFront, LOW);
  digitalWrite(trigSide, LOW);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  if (USE_RIGHT_IR) pinMode(irRightPin, IR_USE_PULLUP ? INPUT_PULLUP : INPUT);
  if (USE_LEFT_IR)  pinMode(irLeftPin,  IR_USE_PULLUP ? INPUT_PULLUP : INPUT);

  Serial.begin(9600);
  stopMotors();
}

void loop() {
  if (RUN_MODE == MODE_LINE_FOLLOW) {
    doLineFollow();
    return;
  }

  if (RUN_MODE == MODE_LINE_DETECT_TWO_STRIPS) {
    doLineDetectTwoStrips();
    return;
  }

  // default
  doWallFollow();
}