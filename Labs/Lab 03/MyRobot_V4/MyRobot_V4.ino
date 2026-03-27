// Photocell settings
const int photocellPin = A0;
const int threshold = 170;

// Motor pins (L298N)
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

const int enB = 10;
const int in3 = 5;
const int in4 = 6;


const int speedVal = 100;

int readPhotocellAverage(int samples) {

  long sum = 0;   // long prevents overflow

  for (int i = 0; i < samples; i++) {
    sum += analogRead(photocellPin);
    delay(2); // tiny delay improves stability
  }

  return sum / samples;
}

void setup() {

  Serial.begin(9600); // optional but VERY useful for calibration

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  stopMotors(); // start safely
}

void loop() {

  int measurement = readPhotocellAverage(10);
  Serial.println(measurement); // helps you see sensor values

  if (measurement < threshold) {
    stopMotors();
  }
  else {
    moveForward();
  }
}

//////////////////////////////////////////////////
// MOTOR FUNCTIONS
//////////////////////////////////////////////////

void moveForward() {

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}