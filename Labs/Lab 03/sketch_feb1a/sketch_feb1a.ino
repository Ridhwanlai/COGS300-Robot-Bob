// ============================
// Photocell + motor code with:
//  - Median filtering (better than average when motor noise causes spikes)
//  - Pegged-high fault mode (1022/1023) -> STOP
//  - Auto-recovery when readings return to normal
//  - Motor trim to reduce veering
// ============================

// Photocell settings
const int photocellPin = A0;
const int threshold    = 270;
const int deadband     = 8;     // hysteresis window

// "Pegged" thresholds for fault detection
const int safetyHigh   = 1015;  // ~5V / saturated
const int safetyLow    = 5;     // ~0V / shorted low (optional safety)

// Motor pins (L298N)
const int enA = 9;   // ENA  (you assumed this is RIGHT motor)
const int in1 = 7;
const int in2 = 8;

const int enB = 10;  // ENB  (you assumed this is LEFT motor)
const int in3 = 6;
const int in4 = 5;

// Base speed + trims
const int baseSpeed = 200;

// Veers LEFT => right wheel too fast => slow RIGHT motor (more negative = slower)
const int rightTrim = -60;
const int leftTrim  = 0;

// State variables
bool lastWasForward = false;

// Fault handling (so one random spike doesn’t instantly disable)
bool sensorFault = false;
int  faultStreak = 0;
int  recoverStreak = 0;
const int faultTripCount   = 3;  // how many consecutive bad readings to enter fault
const int faultRecoverCount = 5; // how many consecutive good readings to exit fault

// ---------- Sensor filtering: median of N samples ----------
int readPhotocellMedian(int samplesOdd) {
  // samplesOdd must be odd (e.g., 9, 11, 13)
  const int NMAX = 15;
  if (samplesOdd < 3) samplesOdd = 3;
  if (samplesOdd > NMAX) samplesOdd = NMAX;
  if (samplesOdd % 2 == 0) samplesOdd++; // force odd

  int vals[NMAX];

  for (int i = 0; i < samplesOdd; i++) {
    vals[i] = analogRead(photocellPin);
    delay(2);
  }

  // Simple sort (N is small)
  for (int i = 0; i < samplesOdd - 1; i++) {
    for (int j = i + 1; j < samplesOdd; j++) {
      if (vals[j] < vals[i]) {
        int t = vals[i];
        vals[i] = vals[j];
        vals[j] = t;
      }
    }
  }

  return vals[samplesOdd / 2]; // median
}

int clampPWM(int v) {
  return constrain(v, 0, 255);
}

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
  // Use median filtering to reduce “1022 spikes” from noise
  int measurement = readPhotocellMedian(11);
  int error = measurement - threshold;

  // Debug
  Serial.print("m=");
  Serial.print(measurement);
  Serial.print(" err=");
  Serial.print(error);
  Serial.print(" fault=");
  Serial.println(sensorFault ? 1 : 0);

  // -------- Fault detection: pegged high or low for several consecutive loops --------
  bool bad = (measurement >= safetyHigh) || (measurement <= safetyLow);

  if (!sensorFault) {
    if (bad) faultStreak++;
    else faultStreak = 0;

    if (faultStreak >= faultTripCount) {
      sensorFault = true;
      lastWasForward = false;
      recoverStreak = 0;
      stopMotors();
      // Don’t return yet; we’ll handle fault below
    }
  }

  // -------- If in fault mode, STOP and wait until sensor looks sane again --------
  if (sensorFault) {
    stopMotors();

    // "Good" = clearly not pegged high/low
    bool good = (measurement < (safetyHigh - 20)) && (measurement > (safetyLow + 20));

    if (good) recoverStreak++;
    else recoverStreak = 0;

    if (recoverStreak >= faultRecoverCount) {
      sensorFault = false;
      faultStreak = 0;
      // lastWasForward stays false until we see tape again via hysteresis
    }

    return; // stay stopped in fault
  }

  // -------- Normal behavior with hysteresis --------
  if (measurement > (threshold + deadband)) {
    lastWasForward = true;
  } else if (measurement < (threshold - deadband)) {
    lastWasForward = false;
  }

  if (lastWasForward) moveForwardTrimmed();
  else stopMotors();
}

//////////////////////////////////////////////////
// MOTOR FUNCTIONS
//////////////////////////////////////////////////

void moveForwardTrimmed() {
  // Your corrected forward direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  int rightPWM = clampPWM(baseSpeed + rightTrim); // ENA
  int leftPWM  = clampPWM(baseSpeed + leftTrim);  // ENB

  analogWrite(enA, rightPWM);
  analogWrite(enB, leftPWM);
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}