// ============================
// Photocell-driven corner turning (single sensor)
// - Follows tape by driving forward while "on tape"
// - When tape is lost, pivots left/right (sweeps) until tape is found again
// - Keeps your median filtering + pegged-high/low fault handling
// ============================

// Photocell settings
const int photocellPin = A0;
const int threshold    = 270;
const int deadband     = 8;     // hysteresis window

// "Pegged" thresholds for fault detection
const int safetyHigh   = 1015;  // ~5V / saturated
const int safetyLow    = 5;     // ~0V / shorted low

// Motor pins (L298N)
const int enA = 9;   // ENA  (RIGHT motor)
const int in1 = 7;
const int in2 = 8;

const int enB = 10;  // ENB  (LEFT motor)
const int in3 = 6;
const int in4 = 5;

// Base speed + trims (for going straight)
const int baseSpeed = 200;
const int rightTrim = -60;   // slow RIGHT motor
const int leftTrim  = 0;

// --- Cornering / search behavior tuning ---
const int searchPWM = 160;                 // pivot speed when searching for tape
const unsigned long searchMaxMs = 4000;    // give up after this long searching
const unsigned long sweepStartMs = 250;    // first sweep duration
const unsigned long sweepStepMs  = 150;    // increase sweep duration each flip
const unsigned long sweepMaxMs   = 900;    // max sweep duration per direction

// State variables for tape detection hysteresis
bool onTapeState = false;  // "true" means we believe we're on tape

// Fault handling
bool sensorFault = false;
int  faultStreak = 0;
int  recoverStreak = 0;
const int faultTripCount    = 3;
const int faultRecoverCount = 5;

// State machine for turning corners
enum RunState { FOLLOW, SEARCH };
RunState state = FOLLOW;

int preferredSearchDir = +1; // alternates each time we enter SEARCH
int searchDir = +1;          // +1 = pivot right, -1 = pivot left

unsigned long searchStartMs = 0;
unsigned long lastFlipMs = 0;
unsigned long sweepMs = sweepStartMs;

// ---------- Sensor filtering: median ----------
int readPhotocellMedian(int samplesOdd) {
  const int NMAX = 15;
  if (samplesOdd < 3) samplesOdd = 3;
  if (samplesOdd > NMAX) samplesOdd = NMAX;
  if (samplesOdd % 2 == 0) samplesOdd++;

  int vals[NMAX];

  for (int i = 0; i < samplesOdd; i++) {
    vals[i] = analogRead(photocellPin);
    delay(2);
  }

  for (int i = 0; i < samplesOdd - 1; i++) {
    for (int j = i + 1; j < samplesOdd; j++) {
      if (vals[j] < vals[i]) {
        int t = vals[i];
        vals[i] = vals[j];
        vals[j] = t;
      }
    }
  }

  return vals[samplesOdd / 2];
}

int clampPWM(int v) {
  return constrain(v, 0, 255);
}

// ---------- Motor helpers (signed control) ----------
void setRightMotorSigned(int cmd) {
  cmd = constrain(cmd, -255, 255);

  if (cmd > 0) {
    // forward
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

void setLeftMotorSigned(int cmd) {
  cmd = constrain(cmd, -255, 255);

  if (cmd > 0) {
    // forward
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

void setMotorsSigned(int leftCmd, int rightCmd) {
  setLeftMotorSigned(leftCmd);
  setRightMotorSigned(rightCmd);
}

void stopMotors() {
  setMotorsSigned(0, 0);
}

void moveForwardTrimmed() {
  int leftPWM  = clampPWM(baseSpeed + leftTrim);
  int rightPWM = clampPWM(baseSpeed + rightTrim);
  setMotorsSigned(leftPWM, rightPWM);
}

void pivotLeft(int pwm) {
  pwm = clampPWM(pwm);
  // left motor backward, right motor forward
  setMotorsSigned(-pwm, +pwm);
}

void pivotRight(int pwm) {
  pwm = clampPWM(pwm);
  // left motor forward, right motor backward
  setMotorsSigned(+pwm, -pwm);
}

// ---------- Search state entry ----------
void beginSearch(unsigned long nowMs) {
  state = SEARCH;
  searchStartMs = nowMs;
  lastFlipMs = nowMs;
  sweepMs = sweepStartMs;

  searchDir = preferredSearchDir;       // start with preferred direction
  preferredSearchDir = -preferredSearchDir; // alternate next time
}

// ============================

void setup() {
  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  stopMotors();
}

void loop() {
  unsigned long nowMs = millis();

  int measurement = readPhotocellMedian(11);
  int error = measurement - threshold;

  // Debug
  Serial.print("m=");
  Serial.print(measurement);
  Serial.print(" err=");
  Serial.print(error);
  Serial.print(" state=");
  Serial.print(state == FOLLOW ? "FOLLOW" : "SEARCH");
  Serial.print(" fault=");
  Serial.println(sensorFault ? 1 : 0);

  // -------- Fault detection --------
  bool bad = (measurement >= safetyHigh) || (measurement <= safetyLow);

  if (!sensorFault) {
    if (bad) faultStreak++;
    else faultStreak = 0;

    if (faultStreak >= faultTripCount) {
      sensorFault = true;
      recoverStreak = 0;
      stopMotors();
    }
  }

  if (sensorFault) {
    stopMotors();

    bool good = (measurement < (safetyHigh - 20)) && (measurement > (safetyLow + 20));
    if (good) recoverStreak++;
    else recoverStreak = 0;

    if (recoverStreak >= faultRecoverCount) {
      sensorFault = false;
      faultStreak = 0;
    }
    return;
  }

  // -------- Tape detection with hysteresis --------
  // Tape is higher than floor in your calibration.
  if (measurement > (threshold + deadband)) {
    onTapeState = true;
  } else if (measurement < (threshold - deadband)) {
    onTapeState = false;
  }
  // else: keep previous onTapeState

  // -------- State machine for corners --------
  if (state == FOLLOW) {
    if (onTapeState) {
      // Stay on tape: drive forward
      moveForwardTrimmed();
    } else {
      // Lost tape: start searching (turning) instead of stopping
      beginSearch(nowMs);
    }
  }

  if (state == SEARCH) {
    // If we found tape again, go back to FOLLOW
    if (onTapeState) {
      state = FOLLOW;
      return;
    }

    // Give up if searching too long (prevents infinite spinning)
    if (nowMs - searchStartMs > searchMaxMs) {
      stopMotors();
      return;
    }

    // Sweep left/right: flip direction after sweepMs, then widen the sweep
    if (nowMs - lastFlipMs > sweepMs) {
      searchDir = -searchDir;
      lastFlipMs = nowMs;
      sweepMs = min(sweepMs + sweepStepMs, sweepMaxMs);
    }

    // Pivot in the current search direction
    if (searchDir > 0) pivotRight(searchPWM);
    else pivotLeft(searchPWM);
  }
}