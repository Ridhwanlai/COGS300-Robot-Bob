// ============================
// Lab 05 Wall-following + Follow-me (based on your Lab 04 scaffold)
// Keeps your manual WASD driving + telemetry, and adds:
//   - Follow-me mode (front ultrasonic + P-controller)
//   - Wall-follow mode (right ultrasonic + P-controller + simple corner handling)
//
// UPDATED to match your newest ultrasonic wiring:
//   Front  HC-SR04: TRIG=D1, ECHO=D2
//   Right  HC-SR04: TRIG=D3, ECHO=D4
//
// Manual controls (same idea as before):
//   m = manual
//   w/s/a/d = drive
//   x or space = stop (and returns to manual)
//
// Auto controls:
//   f = follow-me (front sensor)
//   u = wall-follow (right sensor)
//
// Telemetry CSV (appended columns at end):
//   t_ms,photo,leftTicks,rightTicks,dLeft,dRight,mode,cmd,leftPWM,rightPWM,frontCm,rightCm,wallState
// ============================

// ---------- Photocell ----------
const int photocellPin = A0;

// simple smoothing (no delays)
float photoFilt = 0;
bool  photoInit = false;
const float photoAlpha = 0.20f;  // 0.1–0.3; higher = more responsive, less smooth

void updatePhotocell() {
  int raw = analogRead(photocellPin);
  if (!photoInit) {
    photoFilt = raw;
    photoInit = true;
  } else {
    photoFilt = (1.0f - photoAlpha) * photoFilt + photoAlpha * raw;
  }
}

int getPhotocell() {
  return (int)(photoFilt + 0.5f);
}

// ---------- Encoders ----------
const int encLeftPin  = 12;  // left encoder D0 -> D12
const int encRightPin = 13;  // right encoder D0 -> D13

volatile long leftTicks  = 0;
volatile long rightTicks = 0;

bool useInterrupts = true;
int lastEncLeft  = HIGH;   // for polling fallback
int lastEncRight = HIGH;

// Debounce / noise filter for encoder edges (microseconds)
const unsigned long minTickGapUs = 200;
unsigned long lastTickUsL = 0;
unsigned long lastTickUsR = 0;

void setupEncoders() {
  pinMode(encLeftPin, INPUT_PULLUP);
  pinMode(encRightPin, INPUT_PULLUP);

  lastEncLeft  = digitalRead(encLeftPin);
  lastEncRight = digitalRead(encRightPin);

  unsigned long nowUs = micros();
  lastTickUsL = nowUs;
  lastTickUsR = nowUs;

  // Force polling (works on any pins)
  useInterrupts = false;
}

void updateEncodersByPolling() {
  int a = digitalRead(encLeftPin);
  int b = digitalRead(encRightPin);
  unsigned long nowUs = micros();

  // COUNT ONLY FALLING EDGES (HIGH -> LOW)
  if (lastEncLeft == HIGH && a == LOW) {
    if (nowUs - lastTickUsL >= minTickGapUs) {
      leftTicks++;
      lastTickUsL = nowUs;
    }
  }

  if (lastEncRight == HIGH && b == LOW) {
    if (nowUs - lastTickUsR >= minTickGapUs) {
      rightTicks++;
      lastTickUsR = nowUs;
    }
  }

  lastEncLeft  = a;
  lastEncRight = b;
}

// ---------- Motors (L298N) ----------
// These pin numbers match your physical build wiring list.
const int enA = 9;   // ENA
const int in1 = 7;   // IN1
const int in2 = 8;   // IN2

const int enB = 10;  // ENB
const int in3 = 6;   // IN3
const int in4 = 5;   // IN4

// Your earlier code behavior is preserved:
// Signed motor setters: + = forward, - = backward
// (Direction chosen so your manual commands still behave the same)
void setRightMotor(int cmd) {
  cmd = constrain(cmd, -255, 255);

  if (cmd > 0) {
    // forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, cmd);
  } else if (cmd < 0) {
    // backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, -cmd);
  } else {
    analogWrite(enA, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void setLeftMotor(int cmd) {
  cmd = constrain(cmd, -255, 255);

  if (cmd > 0) {
    // forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, cmd);
  } else if (cmd < 0) {
    // backward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, -cmd);
  } else {
    analogWrite(enB, 0);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
}

// Track commanded motor values (signed)
int currentLeftCmd  = 0;   // -255..255
int currentRightCmd = 0;   // -255..255

void setMotors(int leftCmd, int rightCmd) {
  currentLeftCmd  = constrain(leftCmd, -255, 255);
  currentRightCmd = constrain(rightCmd, -255, 255);
  setLeftMotor(currentLeftCmd);
  setRightMotor(currentRightCmd);
}

void stopMotors() {
  setMotors(0, 0);
}

// ---------- Manual driving constants ----------
const int baseSpeed = 200;
const int rightTrim = -0;
const int leftTrim  = -0;
const int turnSpeed = 180;

// manual command state
int manualLeftCmd  = 0;
int manualRightCmd = 0;

// ---------- Ultrasonic sensors (HC-SR04) ----------
// UPDATED PINS per your latest build update:
const int trigFrontPin = 1;
const int echoFrontPin = 2;

const int trigRightPin = 3;
const int echoRightPin = 4;

// If you only have one ultrasonic sensor, set one of these false.
// (Kit sheet lists HC-SR04 (1), but your build describes two.) :contentReference[oaicite:2]{index=2}
const bool USE_FRONT_US = true;
const bool USE_RIGHT_US = true;

// Reading / filtering
const unsigned long PULSE_TIMEOUT_US = 30000UL; // prevents hanging
const float ULTRA_MIN_CM = 2.0f;
const float ULTRA_MAX_CM = 400.0f;

const unsigned long ultraPeriodMs = 60; // how often to refresh readings
unsigned long lastUltraMs = 0;

float frontCmRaw  = -1.0f, rightCmRaw  = -1.0f;
float frontCmFilt = -1.0f, rightCmFilt = -1.0f;
bool  frontInit = false, rightInit = false;

const float ultraAlpha = 0.35f;          // smoothing factor
const unsigned long ultraHoldMs = 200;   // consider last valid reading "fresh" for this long
unsigned long lastFrontOkMs = 0;
unsigned long lastRightOkMs = 0;

static float usToCm(unsigned long us) {
  return (us * 0.0343f) / 2.0f;
}

static float readUltrasonicCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, PULSE_TIMEOUT_US);
  if (duration == 0) return -1.0f;

  float cm = usToCm(duration);
  if (cm < ULTRA_MIN_CM || cm > ULTRA_MAX_CM) return -1.0f;
  return cm;
}

void updateUltrasonics() {
  unsigned long nowMs = millis();
  if (nowMs - lastUltraMs < ultraPeriodMs) return;
  lastUltraMs = nowMs;

  if (USE_FRONT_US) {
    frontCmRaw = readUltrasonicCm(trigFrontPin, echoFrontPin);
    if (frontCmRaw > 0) {
      if (!frontInit) { frontCmFilt = frontCmRaw; frontInit = true; }
      else { frontCmFilt = (1.0f - ultraAlpha) * frontCmFilt + ultraAlpha * frontCmRaw; }
      lastFrontOkMs = nowMs;
    }
  }

  if (USE_RIGHT_US) {
    rightCmRaw = readUltrasonicCm(trigRightPin, echoRightPin);
    if (rightCmRaw > 0) {
      if (!rightInit) { rightCmFilt = rightCmRaw; rightInit = true; }
      else { rightCmFilt = (1.0f - ultraAlpha) * rightCmFilt + ultraAlpha * rightCmRaw; }
      lastRightOkMs = nowMs;
    }
  }
}

bool frontValid() {
  return USE_FRONT_US && frontInit && (millis() - lastFrontOkMs <= ultraHoldMs);
}

bool rightValid() {
  return USE_RIGHT_US && rightInit && (millis() - lastRightOkMs <= ultraHoldMs);
}

float getFrontCm() {
  return frontValid() ? frontCmFilt : -1.0f;
}

float getRightCm() {
  return rightValid() ? rightCmFilt : -1.0f;
}

// ---------- Lab 05 control parameters ----------
// Follow-me (front sensor)
const float followSetCm      = 25.0f;
const float followDeadbandCm = 2.0f;
const float kP_follow        = 8.0f;   // PWM per cm error (tune as needed)
const int   followMaxPWM     = 200;

// Wall-follow (right sensor)
const float wallSetCm        = 20.0f;
const float wallDeadbandCm   = 1.5f;
const float kP_wall          = 6.0f;   // PWM per cm error (tune as needed)
const int   wallBasePWM      = 170;
const int   wallMaxCorr      = 90;

// If your robot steers the wrong way (goes away from the wall when it should go toward it),
// flip this between +1 and -1.
const int WALL_STEER_SIGN = +1;

// Simple corner-ish handling (optional but helpful)
const float frontStopCm      = 14.0f;  // if front sees obstacle closer than this, do a left pivot
const float wallLostCm       = 150.0f; // if right is invalid or huge -> treat as wall lost
const int   searchSpeed      = 140;
const unsigned long turnLeftMs  = 320;
const unsigned long searchRightMs = 220;

// Wall-follow state
char wallState = 'N'; // N=normal, L=turningLeft, R=searchingRight
unsigned long wallStateUntilMs = 0;

// ---------- Serial/Mode ----------
char mode = 'M';     // 'M' manual, 'F' follow-me, 'W' wall-follow
char lastCmd = '_';  // last received command

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') continue;

    // Normalize
    if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');

    lastCmd = c;

    if (c == 'm') {
      mode = 'M';
      manualLeftCmd = 0;
      manualRightCmd = 0;
      stopMotors();

    } else if (c == 'u') {
      // wall-follow
      mode = 'W';
      wallState = 'N';
      stopMotors();

    } else if (c == 'f') {
      // follow-me
      mode = 'F';
      stopMotors();

    } else if (c == 'x' || c == ' ') {
      // Safety stop: stop + return to manual
      mode = 'M';
      manualLeftCmd = 0;
      manualRightCmd = 0;
      stopMotors();

    } else if (mode == 'M') {
      // Manual driving commands:
      // w = forward, s = backward, a = pivot left, d = pivot right
      if (c == 'w') {
        manualLeftCmd  =  baseSpeed + leftTrim;
        manualRightCmd =  baseSpeed + rightTrim;

      } else if (c == 's') {
        manualLeftCmd  = -(baseSpeed + leftTrim);
        manualRightCmd = -(baseSpeed + rightTrim);

      } else if (c == 'd') { // pivot right (kept exactly as your prior behavior)
        manualLeftCmd  = -turnSpeed;
        manualRightCmd =  turnSpeed;

      } else if (c == 'a') { // pivot left (kept exactly as your prior behavior)
        manualLeftCmd  =  turnSpeed;
        manualRightCmd = -turnSpeed;
      }
    }
  }
}

void doFollowMe() {
  float f = getFrontCm();
  if (f < 0) {
    stopMotors();
    return;
  }

  float err = f - followSetCm; // Lab 05: error = measurement - set_point
  float errAbs = (err >= 0) ? err : -err;

  if (errAbs < followDeadbandCm) {
    stopMotors();
    return;
  }

  int cmd = (int)(kP_follow * err);
  cmd = constrain(cmd, -followMaxPWM, followMaxPWM);

  setMotors(cmd + leftTrim, cmd + rightTrim);
}

void doWallFollow() {
  unsigned long nowMs = millis();
  float f = getFrontCm();
  float r = getRightCm();

  // Timed states first
  if (wallState == 'L') {
    if (nowMs < wallStateUntilMs) {
      // pivot left
      setMotors(turnSpeed, -turnSpeed);
      return;
    } else {
      wallState = 'N';
    }
  } else if (wallState == 'R') {
    if (nowMs < wallStateUntilMs) {
      // pivot right (search for wall)
      setMotors(-searchSpeed, searchSpeed);
      return;
    } else {
      wallState = 'N';
    }
  }

  // Trigger corner/edge behaviors
  if (frontValid() && f > 0 && f < frontStopCm) {
    wallState = 'L';
    wallStateUntilMs = nowMs + turnLeftMs;
    setMotors(turnSpeed, -turnSpeed);
    return;
  }

  if (!rightValid() || r < 0 || r > wallLostCm) {
    wallState = 'R';
    wallStateUntilMs = nowMs + searchRightMs;
    setMotors(-searchSpeed, searchSpeed);
    return;
  }

  // Normal wall-follow P-control:
  // error = measurement - set_point
  float err = r - wallSetCm;
  float errAbs = (err >= 0) ? err : -err;
  if (errAbs < wallDeadbandCm) err = 0;

  int corr = (int)(kP_wall * err);
  corr = constrain(corr, -wallMaxCorr, wallMaxCorr);

  int leftCmd  = wallBasePWM + leftTrim  + WALL_STEER_SIGN * corr;
  int rightCmd = wallBasePWM + rightTrim - WALL_STEER_SIGN * corr;

  setMotors(leftCmd, rightCmd);
}

// ---------- Telemetry timing ----------
const unsigned long telemetryPeriodMs = 1000; // keep your original 1 Hz
unsigned long lastTelemetryMs = 0;
long prevLeftTicks  = 0;
long prevRightTicks = 0;

void setup() {
  Serial.begin(115200);
  setupEncoders();

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Ultrasonic pins
  if (USE_FRONT_US) {
    pinMode(trigFrontPin, OUTPUT);
    pinMode(echoFrontPin, INPUT);
    digitalWrite(trigFrontPin, LOW);
  }
  if (USE_RIGHT_US) {
    pinMode(trigRightPin, OUTPUT);
    pinMode(echoRightPin, INPUT);
    digitalWrite(trigRightPin, LOW);
  }

  stopMotors();
}

void loop() {
  handleSerialCommands();

  if (!useInterrupts) updateEncodersByPolling();

  updatePhotocell();
  updateUltrasonics();

  // ---------- Control ----------
  if (mode == 'M') {
    setMotors(manualLeftCmd, manualRightCmd);
  } else if (mode == 'F') {
    doFollowMe();
  } else if (mode == 'W') {
    doWallFollow();
  } else {
    stopMotors();
  }

  // ---------- Telemetry ----------
  unsigned long now = millis();
  if (now - lastTelemetryMs >= telemetryPeriodMs) {
    lastTelemetryMs = now;

    long lt = leftTicks;
    long rt = rightTicks;

    long dL = lt - prevLeftTicks;
    long dR = rt - prevRightTicks;
    prevLeftTicks  = lt;
    prevRightTicks = rt;

    int photo = getPhotocell();
    float fcm = getFrontCm();
    float rcm = getRightCm();

    // CSV: t_ms,photo,leftTicks,rightTicks,dLeft,dRight,mode,cmd,leftPWM,rightPWM,frontCm,rightCm,wallState
    Serial.print(now);             Serial.print(',');
    Serial.print(photo);           Serial.print(',');
    Serial.print(lt);              Serial.print(',');
    Serial.print(rt);              Serial.print(',');
    Serial.print(dL);              Serial.print(',');
    Serial.print(dR);              Serial.print(',');
    Serial.print(mode);            Serial.print(',');
    Serial.print(lastCmd);         Serial.print(',');
    Serial.print(currentLeftCmd);  Serial.print(',');
    Serial.print(currentRightCmd); Serial.print(',');

    Serial.print(fcm, 1);          Serial.print(',');
    Serial.print(rcm, 1);          Serial.print(',');
    Serial.println(wallState);
  }
}