// ============================
// Lab 05: Wall-following + "Turn a corner" (PWM differential drive)
// - Right ultrasonic keeps a set distance from the wall using a P-controller
// - Front ultrasonic detects inside corners / obstacles
// - Handles:
//    * gradual corners: continuous P-correction
//    * inside corners: pivot left until front clears + wall re-acquired
//    * outside corners: arc right until wall re-acquired
// ============================
//
// WIRING ASSUMPTION (matches your build notes):
//   Front HC-SR04:  VCC=5V, GND=GND, TRIG=D1, ECHO=D2
//   Right HC-SR04:  VCC=5V, GND=GND, TRIG=D3, ECHO=D4
//
// IMPORTANT: On Arduino UNO/Nano, D1 is hardware Serial TX.
// If FRONT_TRIG is on D1, keep DEBUG_SERIAL = 0, or move FRONT_TRIG to another pin.

#define DEBUG_SERIAL 0   // set to 1 only if FRONT_TRIG is NOT D1

// Ultrasonic pins
const uint8_t FRONT_TRIG = 1;
const uint8_t FRONT_ECHO = 2;
const uint8_t RIGHT_TRIG = 3;
const uint8_t RIGHT_ECHO = 4;

// Ultrasonic timing + filtering
const unsigned long ULTRA_TIMEOUT_US = 25000UL; // ~4.3m max range; smaller = faster loop
const uint8_t ULTRA_SAMPLES = 3;                // 1,3,5,7... (median filter)
const uint8_t ULTRA_INTER_SAMPLE_MS = 15;       // helps reduce ringing/crosstalk

const int INVALID_CM = 999;

// Motor pins (L298N) - same as your sketch
const int enA = 9;   // ENA  (RIGHT motor)
const int in1 = 7;
const int in2 = 8;

const int enB = 10;  // ENB  (LEFT motor)
const int in3 = 6;
const int in4 = 5;

// ====== Wall-following (P-controller) ======
const int wallSetCm = 20;        // desired distance from the right wall (cm)
const int wallDeadbandCm = 2;    // cm window where we don't steer
const float kP = 7.0;            // PWM per cm of error (tune!)
const int maxSteer = 120;        // cap for stability

const int baseFwdPWM = 170;      // forward cruising PWM
const int minFwdPWM  = 120;      // below this the TT motor may stall
const int maxFwdPWM  = 230;

// Optional trims if one side is stronger
const int leftTrim  = 0;
const int rightTrim = -60;

// ====== Corner logic thresholds ======
// Inside corner: wall appears in front
const int frontTurnCm  = 15;  // start turning left if front wall closer than this
const int frontClearCm = 25;  // consider "cleared" when front is farther than this

// Outside corner: wall disappears on the right
const int wallLostCm      = wallSetCm + 25; // if right distance gets big -> wall ended
const int wallReacquireCm = wallSetCm + 10; // wall found again

// Turn behaviors (PWM)
const int pivotPWM = 170;         // pivot-in-place speed for inside corners
const int cornerBasePWM = 150;    // forward speed while arcing right for outside corners
const int cornerBiasPWM = 90;     // steering bias while arcing right
const int cornerMinPWM  = 80;     // keep wheels moving

// Timeouts (ms) to avoid infinite turning
const unsigned long insideTurnTimeoutMs  = 2500;
const unsigned long outsideTurnTimeoutMs = 3000;

// ====== State machine ======
enum RunState { FOLLOW_WALL, TURN_INSIDE_LEFT, TURN_OUTSIDE_RIGHT, STOPPED };
RunState state = FOLLOW_WALL;
unsigned long stateStartMs = 0;

// ---------- Motor helpers (signed control) ----------
int clampPWM(int v) { return constrain(v, 0, 255); }

void setRightMotorSigned(int cmd) {
  cmd = constrain(cmd, -255, 255);
  if (cmd > 0) {           // forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, cmd);
  } else if (cmd < 0) {    // backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, -cmd);
  } else {
    analogWrite(enA, 0);
  }
}

void setLeftMotorSigned(int cmd) {
  cmd = constrain(cmd, -255, 255);
  if (cmd > 0) {           // forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, cmd);
  } else if (cmd < 0) {    // backward
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

void pivotLeft(int pwm) {
  pwm = clampPWM(pwm);
  setMotorsSigned(-pwm, +pwm); // left back, right forward
}

void pivotRight(int pwm) {
  pwm = clampPWM(pwm);
  setMotorsSigned(+pwm, -pwm); // left forward, right back
}

// Drive forward with steering.
// steer > 0 => turn RIGHT (left faster, right slower)
void driveForwardSteer(int steer) {
  steer = constrain(steer, -maxSteer, +maxSteer);

  int leftPWM  = baseFwdPWM + steer + leftTrim;
  int rightPWM = baseFwdPWM - steer + rightTrim;

  leftPWM  = constrain(leftPWM,  0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  // keep forward wheels from stalling (but don't force if already 0)
  if (leftPWM  > 0 && leftPWM  < minFwdPWM) leftPWM  = minFwdPWM;
  if (rightPWM > 0 && rightPWM < minFwdPWM) rightPWM = minFwdPWM;

  if (leftPWM  > maxFwdPWM) leftPWM  = maxFwdPWM;
  if (rightPWM > maxFwdPWM) rightPWM = maxFwdPWM;

  setMotorsSigned(leftPWM, rightPWM);
}

// Arc right (both forward), used for outside corners
void arcRight() {
  int leftPWM  = cornerBasePWM + cornerBiasPWM;
  int rightPWM = cornerBasePWM - cornerBiasPWM;

  leftPWM  = constrain(leftPWM,  0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  if (leftPWM  > 0 && leftPWM  < cornerMinPWM) leftPWM  = cornerMinPWM;
  if (rightPWM > 0 && rightPWM < cornerMinPWM) rightPWM = cornerMinPWM;

  setMotorsSigned(leftPWM, rightPWM);
}

// ---------- Ultrasonic helpers ----------
int readHCSR04_once_cm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long us = pulseIn(echoPin, HIGH, ULTRA_TIMEOUT_US);
  if (us == 0) return INVALID_CM;

  // HC-SR04 rule of thumb: cm ≈ us / 58
  int cm = (int)(us / 58UL);
  if (cm <= 0) cm = INVALID_CM;
  return cm;
}

int readHCSR04_median_cm(uint8_t trigPin, uint8_t echoPin, uint8_t samplesOdd) {
  const uint8_t NMAX = 7;
  if (samplesOdd < 1) samplesOdd = 1;
  if (samplesOdd > NMAX) samplesOdd = NMAX;
  if (samplesOdd % 2 == 0) samplesOdd++;

  int vals[NMAX];
  for (uint8_t i = 0; i < samplesOdd; i++) {
    vals[i] = readHCSR04_once_cm(trigPin, echoPin);
    delay(ULTRA_INTER_SAMPLE_MS);
  }

  // simple sort
  for (uint8_t i = 0; i < samplesOdd - 1; i++) {
    for (uint8_t j = i + 1; j < samplesOdd; j++) {
      if (vals[j] < vals[i]) {
        int t = vals[i];
        vals[i] = vals[j];
        vals[j] = t;
      }
    }
  }
  return vals[samplesOdd / 2];
}

void enterState(RunState newState, unsigned long nowMs) {
  state = newState;
  stateStartMs = nowMs;
}

// ============================

void setup() {
#if DEBUG_SERIAL
  Serial.begin(9600);
#endif

  // Ultrasonic pins
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);

  digitalWrite(FRONT_TRIG, LOW);
  digitalWrite(RIGHT_TRIG, LOW);

  // Motor pins
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

  // Read sensors (median-filtered)
  int rightCm = readHCSR04_median_cm(RIGHT_TRIG, RIGHT_ECHO, ULTRA_SAMPLES);
  int frontCm = readHCSR04_median_cm(FRONT_TRIG, FRONT_ECHO, ULTRA_SAMPLES);

#if DEBUG_SERIAL
  Serial.print("right=");
  Serial.print(rightCm);
  Serial.print("cm front=");
  Serial.print(frontCm);
  Serial.print("cm state=");
  Serial.println(state);
#endif

  // ----- State machine -----
  switch (state) {

    case FOLLOW_WALL: {
      // Inside corner / obstacle ahead?
      if (frontCm <= frontTurnCm) {
        enterState(TURN_INSIDE_LEFT, nowMs);
        break;
      }

      // Outside corner / wall ended?
      if (rightCm >= wallLostCm || rightCm == INVALID_CM) {
        enterState(TURN_OUTSIDE_RIGHT, nowMs);
        break;
      }

      // P-controller steering to keep a constant distance to the right wall:
      // error = measurement - set_point
      // output = p * error
      int errorCm = rightCm - wallSetCm;
      if (abs(errorCm) <= wallDeadbandCm) errorCm = 0;

      int steer = (int)(kP * (float)errorCm);
      steer = constrain(steer, -maxSteer, +maxSteer);

      driveForwardSteer(steer);
      break;
    }

    case TURN_INSIDE_LEFT: {
      // Safety: don't turn forever
      if (nowMs - stateStartMs > insideTurnTimeoutMs) {
        stopMotors();
        enterState(STOPPED, nowMs);
        break;
      }

      // Pivot left until we clear the wall in front AND see right-wall distance again
      pivotLeft(pivotPWM);

      bool frontClear = (frontCm >= frontClearCm) || (frontCm == INVALID_CM);
      bool wallFound  = (rightCm <= wallReacquireCm);

      if (frontClear && wallFound) {
        enterState(FOLLOW_WALL, nowMs);
      }
      break;
    }

    case TURN_OUTSIDE_RIGHT: {
      if (nowMs - stateStartMs > outsideTurnTimeoutMs) {
        stopMotors();
        enterState(STOPPED, nowMs);
        break;
      }

      // If suddenly there's a wall ahead, treat as inside corner
      if (frontCm <= frontTurnCm) {
        enterState(TURN_INSIDE_LEFT, nowMs);
        break;
      }

      // Arc right forward until the right wall comes back into range
      arcRight();

      if (rightCm <= wallReacquireCm) {
        enterState(FOLLOW_WALL, nowMs);
      }
      break;
    }

    case STOPPED:
    default:
      stopMotors();
      break;
  }
}