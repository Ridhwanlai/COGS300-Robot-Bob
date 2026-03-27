/*
 * Line Following Robot - 3 Sensor
 * Right IR Sensor:  Pin 11
 * Middle IR Sensor: Pin 13
 * Left IR Sensor:   Pin 12
 *
 * Sensor Logic:
 * - HIGH = on tape (line detected)
 * - LOW  = off tape (no line)
 *
 * Motor Driver: L298N
 * Right Motor: ENA (pin 9), IN1 (pin 7), IN2 (pin 8)
 * Left Motor:  ENB (pin 10), IN3 (pin 6), IN4 (pin 5)
 *
 * 3-Sensor Truth Table:
 * L  M  R  | Action
 * 0  0  0  | All off tape → stop
 * 0  0  1  | Hard right turn
 * 0  1  0  | Middle only  → go straight
 * 0  1  1  | Slight right drift → turn right (soft)
 * 1  0  0  | Hard left turn
 * 1  0  1  | Both outer hit → go straight (rare crossing)
 * 1  1  0  | Slight left drift → turn left (soft)
 * 1  1  1  | All on tape (thick tape/junction) → go straight
 */

// IR Sensor Pins
const int IR_RIGHT  = 11;
const int IR_MIDDLE = 13;
const int IR_LEFT   = 12;

// Motor pins (L298N)
const int enA = 9;
const int in1 = 7;
const int in2 = 8;
const int enB = 10;
const int in3 = 6;
const int in4 = 5;

// Speed settings
const int BASE_SPEED       = 90;
const int TURN_SPEED_FAST  = 130;
const int SOFT_TURN_OUTER  = 97;  // Adjusted down from 112
const int SOFT_TURN_INNER  = 93;  // Adjusted up from 95
const int RIGHT_TRIM       = 0;
const int LEFT_TRIM        = 0;

// Debounce filter — set to false to disable, true to enable
const bool USE_FILTER = true;
const int FILTER_READS    = 2;   // Extra confirmations required
const int FILTER_DELAY_MS = 5;   // ms between each confirmation read

void setup() {
  Serial.begin(9600);
  pinMode(IR_RIGHT,  INPUT);
  pinMode(IR_MIDDLE, INPUT);
  pinMode(IR_LEFT,   INPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial.println("3-Sensor Line Follower Ready");
  delay(2000);
}

void loop() {
  int R = digitalRead(IR_RIGHT);
  int M = digitalRead(IR_MIDDLE);
  int L = digitalRead(IR_LEFT);

  // Debounce filter — only proceeds if readings are stable across FILTER_READS checks
  if (USE_FILTER) {
    for (int i = 0; i < FILTER_READS; i++) {
      delay(FILTER_DELAY_MS);
      if (digitalRead(IR_RIGHT)  != R ||
          digitalRead(IR_MIDDLE) != M ||
          digitalRead(IR_LEFT)   != L) {
        return; // Unstable reading, skip this cycle
      }
    }
  }

  if (L == HIGH && M == HIGH && R == HIGH) {
    stopMotors();
    Serial.println("STOP - All sensors off tape");
  }
  else if (L == HIGH && M == HIGH && R == LOW) {
    hardTurnLeft();
    Serial.println("Hard Left Turn");
  }
  else if (L == HIGH && M == LOW && R == HIGH) {
    goStraight();
    Serial.println("Straight - Junction/Cross");
  }
  else if (L == HIGH && M == LOW && R == LOW) {
    softTurnLeft();
    Serial.println("Soft Left Correction");
  }
  else if (L == LOW && M == HIGH && R == HIGH) {
    hardTurnRight();
    Serial.println("Hard Right Turn");
  }
  else if (L == LOW && M == HIGH && R == LOW) {
    goStraight();
    Serial.println("Straight - Junction/Cross");
  }
  else if (L == LOW && M == LOW && R == HIGH) {
    softTurnRight();
    Serial.println("Soft Right Correction");
  }
  else if (L == LOW && M == LOW && R == LOW) {
    goStraight();
    Serial.println("Straight - All on tape");
  }

}

void setMotors(int rightSpd, int leftSpd) {
  if (rightSpd >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    rightSpd = -rightSpd;
  }
  analogWrite(enA, constrain(rightSpd, 0, 255));

  if (leftSpd >= 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    leftSpd = -leftSpd;
  }
  analogWrite(enB, constrain(leftSpd, 0, 255));
}

void goStraight() {
  setMotors(BASE_SPEED + RIGHT_TRIM, BASE_SPEED + LEFT_TRIM);
}

void hardTurnRight() {
  setMotors(0, TURN_SPEED_FAST);
}

void hardTurnLeft() {
  setMotors(TURN_SPEED_FAST, 0);
}

void softTurnRight() {
  setMotors(SOFT_TURN_INNER, SOFT_TURN_OUTER);
  delay(40);
}

void softTurnLeft() {
  setMotors(SOFT_TURN_OUTER, SOFT_TURN_INNER);
  delay(40);
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
}