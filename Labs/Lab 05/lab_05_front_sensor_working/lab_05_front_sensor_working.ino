// ---------- Ultrasonic Sensors ----------
int trigFront = 1;
int echoFront = 2;

int trigSide = 3;
int echoSide = 4;

// ---------- L298N Motor Pins ----------
int IN1 = 7;
int IN2 = 8;
int IN3 = 6;
int IN4 = 5;

int ENA = 9;
int ENB = 10;

// ---------- Control Parameters ----------
float slowPoint = 35.0;     // where slowdown becomes noticeable
float stopDistance = 10.0;   // full stop before wall
float KpFront = 4.0;

// float sideSetPoint = 20.0;
// float sideTolerance = 2.0;

int maxSpeed = 200;
int minSpeed = 60;

void setup() {
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigSide, OUTPUT);
  pinMode(echoSide, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // forward direction
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// ---------- Ultrasonic ----------
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  return (duration / 2.0) / 29.1;
}

void loop() {
  float frontDist = readUltrasonic(trigFront, echoFront);
  // float sideDist  = readUltrasonic(trigSide, echoSide);

  // ---------- FRONT SPEED CONTROL ----------
  float frontError = frontDist - stopDistance;
  int baseSpeed = KpFront * frontError;
  baseSpeed = constrain(baseSpeed, 0, maxSpeed);

  if (frontDist <= stopDistance) {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    return;
  }

  baseSpeed = max(baseSpeed, minSpeed);

  int leftSpeed = baseSpeed;
  int rightSpeed = baseSpeed;

  // // ---------- SIDE WALL FOLLOWING ----------
  // if (sideDist < sideSetPoint - sideTolerance) {
  //   rightSpeed *= 0.7;
  // }
  // else if (sideDist > sideSetPoint + sideTolerance) {
  //   leftSpeed *= 0.7;
  // }

  analogWrite(ENA, rightSpeed);
  analogWrite(ENB, leftSpeed);

  delay(50);
}
