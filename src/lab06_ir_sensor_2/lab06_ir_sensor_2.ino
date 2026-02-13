/*
 * Line Following Robot
 * Right IR Sensor: Pin 11
 * Left IR Sensor: Pin 12
 * 
 * Sensor Logic:
 * - HIGH = on tape (line detected)
 * - LOW = off tape (no line)
 * 
 * Motor Driver: L298N
 * Right Motor: ENA (pin 9), IN1 (pin 7), IN2 (pin 8)
 * Left Motor: ENB (pin 10), IN3 (pin 6), IN4 (pin 5)
 */

// IR Sensor Pins
const int IR_SENSOR_RIGHT = 11;
const int IR_SENSOR_LEFT = 12;

// Motor pins (L298N)
const int enA = 9;   // ENA - RIGHT motor
const int in1 = 7;
const int in2 = 8;

const int enB = 10;  // ENB - LEFT motor
const int in3 = 6;
const int in4 = 5;

// Base speed + trims
const int baseSpeed = 130;
const int rightTrim = -40;  // Slow right motor to correct left veer
const int leftTrim = 0;

// Calculated motor speeds
int rightSpeed = baseSpeed + rightTrim;
int leftSpeed = baseSpeed + leftTrim;

// Timer for off-line condition
unsigned long offLineStartTime = 0;
bool isOffLine = false;
const unsigned long STOP_DELAY = 2000;  // 3 seconds in milliseconds

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Set IR sensor pins as inputs
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  
  // Set motor pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  Serial.println("Line Following Robot Started");
  delay(2000); // Wait 2 seconds before starting
}

void loop() {
  // Read both IR sensors
  int rightSensor = digitalRead(IR_SENSOR_RIGHT);
  int leftSensor = digitalRead(IR_SENSOR_LEFT);
  
  // Both sensors on tape - go straight
  if (rightSensor == LOW && leftSensor == LOW) {
    goStraight();
    Serial.println("Going Straight - Both on tape");
  }
  
  // Right sensor off tape - turn left to adjust
  else if (rightSensor == LOW && leftSensor == HIGH) {
    turnLeft();
    Serial.println("Adjusting Right - Right sensor off tape");
    delay(200);
  }
  
  // Left sensor off tape - turn right to adjust
  else if (rightSensor == HIGH && leftSensor == LOW) {
    turnRight();
    Serial.println("Adjusting Left - Left sensor off tape");
    delay(200);
  }
  
  // Both sensors off tape - stop
  else {
    // If just went off-line, start the timer
    if (!isOffLine) {
      offLineStartTime = millis();
      isOffLine = true;
      Serial.println("Both sensors off tape - Timer started");
    }
    
    // Check if off-line for more than 3 seconds
    unsigned long offLineTime = millis() - offLineStartTime;
    if (offLineTime >= STOP_DELAY) {
      stopMotors();
      Serial.print("STOP - Off tape for ");
      Serial.print(offLineTime / 1000.0);
      Serial.println(" seconds");
    } else {
      // Keep searching (you can choose to keep moving or slow down)
      goStraight();  // Or use turnLeft()/turnRight() to search
      Serial.print("Searching for line... ");
      Serial.print((STOP_DELAY - offLineTime) / 1000.0);
      Serial.println(" seconds remaining");
    }
  }
  delay(50); // Small delay for stability
}

// Move forward straight
void goStraight() {
  // Right motor forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, rightSpeed);
  
  // Left motor forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, leftSpeed);
}

// Turn right (slow down right motor, keep left motor normal)
void turnRight() {
  // Right motor slow or stopped
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, rightSpeed * 0.3);  // Reduced speed for right
  
  // Left motor normal speed
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, leftSpeed);
}

// Turn left (slow down left motor, keep right motor normal)
void turnLeft() {
  // Right motor normal speed
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, rightSpeed);
  
  // Left motor slow or stopped
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, leftSpeed * 0.3);  // Reduced speed for left
}

// Stop both motors
void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
}