// ---------- Ultrasonic Sensors ----------
const uint8_t trigFront = 1;
const uint8_t echoFront = 2;

const uint8_t trigSide  = 3;
const uint8_t echoSide  = 4;

// If your SIDE sensor is wired with TRIG/ECHO swapped, swap these:
// const uint8_t trigSide  = 4;
// const uint8_t echoSide  = 3;

// ---------- L298N Motor Pins ----------
const uint8_t IN1 = 7;
const uint8_t IN2 = 8;
const uint8_t IN3 = 6;
const uint8_t IN4 = 5;

const uint8_t ENA = 9;   // RIGHT motor PWM
const uint8_t ENB = 10;  // LEFT  motor PWM

// ---------- Control Parameters ----------
const float stopDistance = 10.0f;   // full stop before wall (cm)
const float KpFront      = 4.0f;

const float sideSetPoint = 20.0f;   // desired distance to right wall (cm)
const float sideDeadband = 2.0f;    // ignore small errors
const float KpSide       = 5.0f;    // steering gain (PWM per cm error)
const int   maxSteer     = 90;      // cap steering correction

const int maxSpeed = 200;
const int minSpeed = 60;

// ---------- Ultrasonic timing ----------
const unsigned long ECHO_TIMEOUT_US = 22000UL; // max echo wait (~3.8m). Lower = faster/less interference
const unsigned long PING_GAP_MS     = 35UL;    // minimum gap between *any* pings (prevents crosstalk)

const float INVALID_DIST = -1.0f;

// Last good readings so one bad ping doesn't wreck control
float frontDistCM = INVALID_DIST;
float sideDistCM  = INVALID_DIST;

bool isValidDist(float d) {
  return (d > 1.0f && d < 400.0f);
}

// One ping. Returns INVALID_DIST on timeout/no-echo.
float pingCM(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);

  // pulseIn() returns 0 if it times out (meaning "no echo received")
  if (duration == 0) return INVALID_DIST;

  // distance (cm) = (duration_us * 0.0343 cm/us) / 2
  return (duration * 0.0343f) / 2.0f;
}

// Ping only ONE sensor per call, alternating front/side.
// This prevents “second sensor never works” caused by ultrasonic interference.
void updateSensors() {
  static unsigned long lastPingMs = 0;
  static bool pingFrontNext = true;

  unsigned long nowMs = millis();
  if (nowMs - lastPingMs < PING_GAP_MS) return;
  lastPingMs = nowMs;

  if (pingFrontNext) {
    float d = pingCM(trigFront, echoFront);
    if (isValidDist(d)) frontDistCM = d;
  } else {
    float d = pingCM(trigSide, echoSide);
    if (isValidDist(d)) sideDistCM = d;
  }

  pingFrontNext = !pingFrontNext;
}

void setup() {
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigSide, OUTPUT);
  pinMode(echoSide, INPUT);

  digitalWrite(trigFront, LOW);
  digitalWrite(trigSide, LOW);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // forward direction (your original)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // Optional debugging:
  // If you're on an UNO and keep trigFront on D1, Serial can conflict.
  // Serial.begin(115200);
}

void loop() {
  updateSensors();

  // If we haven't gotten a valid FRONT reading yet, don't move.
  if (!isValidDist(frontDistCM)) {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    return;
  }

  // ---------- FRONT SPEED CONTROL ----------
  if (frontDistCM <= stopDistance) {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    return;
  }

  float frontError = frontDistCM - stopDistance;
  int baseSpeed = (int)(KpFront * frontError);
  baseSpeed = constrain(baseSpeed, 0, maxSpeed);
  baseSpeed = max(baseSpeed, minSpeed);

  int leftSpeed  = baseSpeed;
  int rightSpeed = baseSpeed;

  // ---------- SIDE WALL FOLLOWING ----------
  // Only steer if we have a valid side reading
  if (isValidDist(sideDistCM)) {
    float sideError = sideDistCM - sideSetPoint;  // + too far, - too close
    if (abs(sideError) < sideDeadband) sideError = 0;

    int steer = (int)(KpSide * sideError);
    steer = constrain(steer, -maxSteer, +maxSteer);

    // Right-side wall sensor steering:
    //  +steer => too far from wall => turn RIGHT (left faster, right slower)
    //  -steer => too close         => turn LEFT  (left slower, right faster)
    leftSpeed  = baseSpeed + steer;
    rightSpeed = baseSpeed - steer;
  }

  leftSpeed  = constrain(leftSpeed,  0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  if (leftSpeed  > 0) leftSpeed  = max(leftSpeed,  minSpeed);
  if (rightSpeed > 0) rightSpeed = max(rightSpeed, minSpeed);

  analogWrite(ENA, rightSpeed);
  analogWrite(ENB, leftSpeed);

  // Optional debugging:
  // Serial.print("front="); Serial.print(frontDistCM);
  // Serial.print(" side=");  Serial.print(sideDistCM);
  // Serial.print(" L=");     Serial.print(leftSpeed);
  // Serial.print(" R=");     Serial.println(rightSpeed);
}