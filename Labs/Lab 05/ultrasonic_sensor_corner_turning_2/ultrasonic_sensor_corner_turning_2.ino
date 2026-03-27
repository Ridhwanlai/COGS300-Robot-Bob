// ============================
// Dual-ultrasonic wall follower with robust corner handling
// - Front ultrasonic: obstacle/inside-corner detection
// - Side ultrasonic: wall-follow distance
// - Inside corner: commits to a LEFT turn until clear, then recovers straight
// - Reduces crosstalk by alternating pings (front/side) with a gap
// ============================

// ---------- Ultrasonic Pins ----------
const uint8_t trigFront = 1;
const uint8_t echoFront = 2;

const uint8_t trigSide  = 3;
const uint8_t echoSide  = 4;

// ---------- L298N Motor Pins ----------
// IMPORTANT (typical L298N):
// OUT1/OUT2 = Motor A controlled by ENA/IN1/IN2
// OUT3/OUT4 = Motor B controlled by ENB/IN3/IN4
// From your wiring notes, RIGHT motor is on OUT1/OUT2 => ENA is RIGHT motor PWM.
const uint8_t ENA = 9;   // RIGHT motor PWM
const uint8_t IN1 = 7;
const uint8_t IN2 = 8;

const uint8_t ENB = 10;  // LEFT motor PWM
const uint8_t IN3 = 6;
const uint8_t IN4 = 5;

// ---------- KEY CONFIG (edit this) ----------
const bool FOLLOW_RIGHT_WALL = true;   // true = side sensor faces right wall, false = faces left wall
const bool INSIDE_TURN_LEFT  = true;   // you asked for inside LEFT turns

// If one motor’s “forward” is actually backward, flip one of these:
const bool invertLeftMotor  = false;
const bool invertRightMotor = false;

// ---------- Control Parameters ----------
const float stopDistanceCm   = 10.0f;  // full stop safety
const float slowPointCm      = 35.0f;  // start slowing as you approach something
const float KpFront          = 4.0f;   // speed vs front distance

const float sideSetPointCm   = 20.0f;  // desired wall distance
const float sideDeadbandCm   = 2.0f;   // ignore small errors
const float KpSide           = 3.0f;   // steering gain
const int   maxSteerPWM      = 90;     // cap steering

// INSIDE CORNER tuning (these are the ones most affected by moving the sensor)
const float insideTriggerCm  = 22.0f;  // start the inside turn BEFORE you reach stopDistanceCm
const float frontClearCm     = 25.0f;  // consider "clear" when front distance is bigger than this

// If the side sensor moved forward, it may "see" the front wall at corners.
// This extra trigger helps catch inside corners even if frontDist doesn't dip enough:
const float sideCornerTooCloseCm = 9.9f;  // if side distance suddenly drops below this near a corner, treat as inside corner
const float insideApproachCm     = 30.0f;  // only use the sideCornerTooClose trigger when front is within this range

// Speeds
const int maxSpeedPWM = 200;
const int minSpeedPWM = 60;

// Turn behavior
const int turnPWM = 170;                    // pivot speed during inside turn
const unsigned long minTurnMs = 300;        // don't exit turn too early
const unsigned long maxTurnMs = 1800;       // failsafe timeout
const unsigned long recoverMs = 250;        // short straight drive after turn

// ---------- Ultrasonic timing ----------
const unsigned long ECHO_TIMEOUT_US = 24000UL; // ~4m max; reduce if needed
const unsigned long PING_GAP_MS     = 40UL;    // gap between sensor pings to reduce crosstalk

// ---------- Internal state ----------
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

  if (cmd > 0) { // forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, cmd);
  } else if (cmd < 0) { // backward
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

  if (cmd > 0) { // forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, cmd);
  } else if (cmd < 0) { // backward
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

// Smooth float map
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
  if (duration == 0) return false; // timeout => invalid/no echo

  outCm = (duration * 0.0343f) / 2.0f; // cm
  if (outCm < 1.0f || outCm > 400.0f) return false;
  return true;
}

// Alternate pings to reduce crosstalk (important when sensors are close together)
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

  // Optional debug (OK on Arduino R4 USB Serial)
  Serial.begin(9600);

  stopMotors();
}

void loop() {
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

  // ---------- Mode logic ----------
  if (mode == FOLLOW) {
    // Compute base speed from front distance
    int baseSpeed = minSpeedPWM;

    if (frontFresh()) {
      if (frontCm < slowPointCm) {
        baseSpeed = (int)mapFloat(frontCm, stopDistanceCm, slowPointCm, minSpeedPWM, maxSpeedPWM);
      } else {
        // Far away => faster, via P
        float frontError = frontCm - stopDistanceCm;
        baseSpeed = (int)(KpFront * frontError);
      }
    }
    baseSpeed = constrain(baseSpeed, minSpeedPWM, maxSpeedPWM);

    // Decide inside-corner trigger
    bool insideCorner = false;

    if (frontFresh() && frontCm <= insideTriggerCm) {
      insideCorner = true;
    }

    // Extra corner trigger for when the side sensor is moved forward and "sees" the front wall
    if (frontFresh() && sideFresh() &&
        frontCm <= insideApproachCm &&
        sideCm <= sideCornerTooCloseCm) {
      insideCorner = true;
    }

    if (insideCorner) {
      enterMode(TURN_INSIDE);
      return;
    }

    // Wall-follow steering
    int leftPWM  = baseSpeed;
    int rightPWM = baseSpeed;

    if (sideFresh()) {
      float error = sideCm - sideSetPointCm;
      if (fabs(error) < sideDeadbandCm) error = 0.0f;

      int steer = (int)(KpSide * error);
      steer = constrain(steer, -maxSteerPWM, +maxSteerPWM);

      if (FOLLOW_RIGHT_WALL) {
        // Right wall: too far (error+) => turn RIGHT => left faster, right slower
        leftPWM  = baseSpeed + steer;
        rightPWM = baseSpeed - steer;
      } else {
        // Left wall: too far (error+) => turn LEFT => left slower, right faster
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

    // Turn failsafe
    if (t > maxTurnMs) {
      enterMode(RECOVER);
      return;
    }

    // Commit to an in-place turn so it actually corners even if side readings get weird
    if (INSIDE_TURN_LEFT) pivotLeft(turnPWM);
    else pivotRight(turnPWM);

    // if (INSIDE_TURN_LEFT) pivotRight(turnPWM);
    // else pivotLeft(turnPWM);

    // Don’t exit instantly (prevents jitter)
    if (t < minTurnMs) return;

    // Exit conditions:
    //  - front is clear (or front reading isn't fresh)
    //  - and side distance is reasonable again (if side is fresh)
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
    // Drive straight briefly so you don't immediately hook back into a circle
    if (nowMs - modeStartMs > recoverMs) {
      enterMode(FOLLOW);
      return;
    }

    int pwm = 120;
    setMotorsSigned(pwm, pwm);
    return;
  }
}