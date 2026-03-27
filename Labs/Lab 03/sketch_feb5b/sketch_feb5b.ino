// ============================
// Lab 04 telemetry + control scaffold (FIXED)
// Changes implemented from my suggestions:
// 1) Encoder polling now counts ONLY one edge (FALLING: HIGH->LOW) to avoid ~2x counts
// 2) Added a tiny "min tick gap" debounce in microseconds to reject noise/glitches
// 3) Cleaned up a couple of minor typos/comments (e.g., telemetryPeriodMs comment, "255..255s")
// Everything else (your speed reduction + 1 Hz telemetry) is kept.
// ============================

// ---------- Photocell ----------
const int photocellPin = A0;
const int threshold    = 270;
const int deadband     = 8;
const int safetyHigh   = 1015;   // pegged high -> stop (wiring/noise safety)

// simple smoothing (no delays)
float photoFilt = 0;
bool  photoInit = false;
const float photoAlpha = 0.20f;  // 0.1–0.3; higher = more responsive, less smooth

// ---------- Encoders ----------
const int encLeftPin  = 12;  // left encoder D0 -> D12
const int encRightPin = 13;  // right encoder D0 -> D13

volatile long leftTicks  = 0;
volatile long rightTicks = 0;

bool useInterrupts = true;
int lastEncLeft  = HIGH;   // for polling fallback
int lastEncRight = HIGH;

// Debounce / noise filter for encoder edges (microseconds)
// If you still see inflated counts, increase this (e.g., 500, 1000).
const unsigned long minTickGapUs = 200;
unsigned long lastTickUsL = 0;
unsigned long lastTickUsR = 0;

// ---------- Motors (L298N) ----------
// const int enA = 9;   // ENA  (assumed RIGHT motor)
// const int in1 = 7;
// const int in2 = 8;

// const int enB = 10;  // ENB  (assumed LEFT motor)
// const int in3 = 6;
// const int in4 = 5;

// swapping orientation
// ---------- Motors (L298N) ----------
const int enA = 9;   // ENA  (assumed RIGHT motor)
const int in1 = 8;
const int in2 = 7;

const int enB = 10;  // ENB  (assumed LEFT motor)
const int in3 = 5;
const int in4 = 6;

const int baseSpeed = 200;

// You said it veers LEFT -> right wheel effectively faster -> slow RIGHT motor
const int rightTrim = -0;
const int leftTrim  = -0;

const int turnSpeed = 180; // manual pivot turn speed

// Track commanded motor values (signed)
int currentLeftCmd  = 0;   // -255..255
int currentRightCmd = 0;   // -255..255

// ---------- Control ----------
char mode = 'M';     // 'A' Auto, 'M' Manual
char lastCmd = '_';  // last received command
bool autoForwardState = false;  // hysteresis state for auto mode

// manual command state
int manualLeftCmd  = 0;
int manualRightCmd = 0;

// ---------- Telemetry timing ----------
const unsigned long telemetryPeriodMs = 1000; // 1 Hz (once per second)
unsigned long lastTelemetryMs = 0;
long prevLeftTicks  = 0;
long prevRightTicks = 0;

// ---------- Helpers ----------
int clampPWM(int v) { return constrain(v, 0, 255); }

void setupEncoders() {
  pinMode(encLeftPin, INPUT_PULLUP);
  pinMode(encRightPin, INPUT_PULLUP);

  // Initialize last states
  lastEncLeft  = digitalRead(encLeftPin);
  lastEncRight = digitalRead(encRightPin);

  // Initialize debounce timers
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

  // COUNT ONLY FALLING EDGES (HIGH -> LOW) to avoid ~2x counts
  // Also apply a minimum time gap to suppress noise/glitches.
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

// Signed motor setters: + = forward, - = backward
void setRightMotor(int cmd) {
  cmd = constrain(cmd, -255, 255);

  if (cmd > 0) {
    // forward (your corrected direction)
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, cmd);
  } else if (cmd < 0) {
    // backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, -cmd);
  } else {
    analogWrite(enA, 0);
  }
}

void setLeftMotor(int cmd) {
  cmd = constrain(cmd, -255, 255);

  if (cmd > 0) {
    // forward (your corrected direction)
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, cmd);
  } else if (cmd < 0) {
    // backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, -cmd);
  } else {
    analogWrite(enB, 0);
  }
}

void setMotors(int leftCmd, int rightCmd) {
  currentLeftCmd  = constrain(leftCmd, -255, 255);
  currentRightCmd = constrain(rightCmd, -255, 255);
  setLeftMotor(currentLeftCmd);
  setRightMotor(currentRightCmd);
}

void stopMotors() {
  setMotors(0, 0);
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') continue;

    lastCmd = c;

    if (c == 'm') {           // manual mode
      mode = 'M';
      manualLeftCmd = 0;
      manualRightCmd = 0;
      stopMotors();

    } else if (c == 'u') {    // auto mode
      mode = 'A';
      stopMotors();

    } else if (mode == 'M') {
      // Manual driving commands:
      // w = forward, s = backward, a = pivot left, d = pivot right, space/x = stop

      if (c == 'w') { // forward
        manualLeftCmd  =  baseSpeed + leftTrim;
        manualRightCmd =  baseSpeed + rightTrim;

      } else if (c == 's') { // backward
        manualLeftCmd  = -(baseSpeed + leftTrim);
        manualRightCmd = -(baseSpeed + rightTrim);

      } else if (c == 'd') { // pivot right
        manualLeftCmd  = -turnSpeed;
        manualRightCmd =  turnSpeed;

      } else if (c == 'a') { // pivot left
        manualLeftCmd  =  turnSpeed;
        manualRightCmd = -turnSpeed;

      } else if (c == ' ' || c == 'x') { // stop
        manualLeftCmd = 0;
        manualRightCmd = 0;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  setupEncoders();

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  stopMotors();
}

void loop() {
  handleSerialCommands();

  if (!useInterrupts) updateEncodersByPolling();

  updatePhotocell();
  int photo = getPhotocell();

  // ---------- Control ----------
  if (mode == 'A') {
    // Safety: if sensor is pegged high, stop
    if (photo >= safetyHigh) {
      autoForwardState = false;
      stopMotors();
    } else {
      // Hysteresis: only flip state when clearly past thresholds
      if (photo > (threshold + deadband)) {
        autoForwardState = true;
      } else if (photo < (threshold - deadband)) {
        autoForwardState = false;
      }

      if (autoForwardState) {
        setMotors(baseSpeed + leftTrim, baseSpeed + rightTrim);
      } else {
        stopMotors();
      }
    }
  } else {
    // Manual mode
    setMotors(manualLeftCmd, manualRightCmd);
  }

  // ---------- Telemetry ----------
  unsigned long now = millis();
  if (now - lastTelemetryMs >= telemetryPeriodMs) {
    lastTelemetryMs = now;

    // Since we're polling (no interrupts), this is safe—but leaving as-is is fine.
    long lt = leftTicks;
    long rt = rightTicks;

    long dL = lt - prevLeftTicks;
    long dR = rt - prevRightTicks;
    prevLeftTicks  = lt;
    prevRightTicks = rt;

    // CSV: t_ms,photo,leftTicks,rightTicks,dLeft,dRight,mode,cmd,leftPWM,rightPWM
    Serial.print(now);             Serial.print(',');
    Serial.print(photo);           Serial.print(',');
    Serial.print(lt);              Serial.print(',');
    Serial.print(rt);              Serial.print(',');
    Serial.print(dL);              Serial.print(',');
    Serial.print(dR);              Serial.print(',');
    Serial.print(mode);            Serial.print(',');
    Serial.print(lastCmd);         Serial.print(',');
    Serial.print(currentLeftCmd);  Serial.print(',');
    Serial.println(currentRightCmd);
  }
}