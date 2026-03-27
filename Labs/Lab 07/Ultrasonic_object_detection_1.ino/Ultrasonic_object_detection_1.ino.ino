#include <Servo.h>
/**
  ============================================================
  LESSON 12 (MODIFIED): ULTRASOUND OBJECT CONTACT CAR
  ============================================================

  This sketch makes the robot drive forward automatically, search for objects using a servo-mounted
  ultrasonic sensor, drive toward the closest object, and stop when it makes contact.

  PIN ASSIGNMENTS:
    The ENA pin for left motor PWM speed control is connected to D5.
    The ENB pin for right motor PWM speed control is connected to D6.
    The IN1 pin for left motor forward direction is connected to D2.
    The IN2 pin for left motor backward direction is connected to D4.
    The IN3 pin for right motor forward direction is connected to D7.
    The IN4 pin for right motor backward direction is connected to D8.

    The servo signal pin is connected to A2.
    The ultrasonic sensor TRIG pin is connected to A1.
    The ultrasonic sensor ECHO pin is connected to A0.

  BEHAVIOR:
    The robot drives forward at a constant speed.
    When the front ultrasonic sensor detects an object closer than DETECT_CM, the robot stops and
    sweeps the servo left and right to determine which direction the object is closest.
    The robot then turns toward the closer side and drives forward at a slower approach speed.
    When the front sensor reads below CONTACT_CM, or stops returning valid readings after previously
    seeing a nearby object, the robot considers itself in contact with the object and stops permanently.
  ============================================================
*/


// ============================================================
// MOTOR PINS
// ============================================================

// The left motor PWM speed control pin is connected to D5.
#define Lpwm_pin  5
// The right motor PWM speed control pin is connected to D6.
#define Rpwm_pin  6
// The left motor forward direction pin is connected to D2.
int pinLF = 2;
// The left motor backward direction pin is connected to D4.
int pinLB = 4;
// The right motor forward direction pin is connected to D7.
int pinRF = 7;
// The right motor backward direction pin is connected to D8.
int pinRB = 8;

// ============================================================
// SERVO AND ULTRASONIC
// ============================================================

// This is the Servo object that controls the servo motor used to sweep the ultrasonic sensor.
Servo myservo;

// ============================================================
// TUNABLE CONSTANTS
// ============================================================

// The robot first notices an object ahead and begins the approach sequence when the front sensor reads below this value in centimeters.
const float DETECT_CM       = 50.0;
// The robot considers itself in contact with the object when the front sensor reads below this value in centimeters.
const float CONTACT_CM      = 4.0;
// This is the PWM speed used when no object is detected and the robot is driving forward normally.
const int   DRIVE_SPEED     = 100;
// This is the slower PWM speed used when the robot is approaching a detected object.
const int   APPROACH_SPEED  = 60;
// This is the PWM speed used when the robot turns toward the object.
const int   TURN_SPEED      = 100;
// This is how long in milliseconds the robot turns toward the object before driving forward again.
const int   TURN_MS         = 200;
// This is how long in milliseconds the robot pushes forward after detecting contact to ensure it touches the object.
const int   FINAL_PUSH_MS   = 300;

// ============================================================
// GLOBALS
// ============================================================

// This variable stores the distance measured when the servo is pointing left (160 degrees).
volatile int DL;
// This variable stores the distance measured when the servo is pointing straight ahead.
volatile int DM;
// This variable stores the distance measured when the servo is pointing right (20 degrees).
volatile int DR;
// This flag is set to true once the robot has made contact with an object and stopped permanently.
bool contactMade = false;
// This flag tracks whether the front sensor has recently detected an object within DETECT_CM.
bool objectSeen  = false;

// ============================================================
// ULTRASONIC HELPERS
// ============================================================

/**
 * CHECK DISTANCE
 * The checkdistance function triggers the ultrasonic sensor on A1, measures how long the echo
 * takes to return on A0, and converts the duration into a distance in centimeters.
 * The duration is divided by 58, which is an approximation of the speed-of-sound conversion
 * for a round-trip measurement in centimeters.
 */
float checkdistance() {
    // The TRIG pin is set LOW to ensure a clean start before sending the trigger pulse.
    digitalWrite(A1, LOW);
    // A 2-microsecond delay allows the TRIG pin to settle at LOW before the pulse begins.
    delayMicroseconds(2);
    // The TRIG pin is set HIGH to begin the trigger pulse that tells the sensor to emit an ultrasonic burst.
    digitalWrite(A1, HIGH);
    // The TRIG pin is held HIGH for 10 microseconds, which is the pulse duration required by the HC-SR04 sensor.
    delayMicroseconds(10);
    // The TRIG pin is set back to LOW to end the trigger pulse.
    digitalWrite(A1, LOW);
    // The duration of the echo pulse is measured and converted to centimeters by dividing by 58.
    float distance = pulseIn(A0, HIGH) / 58.00;
    // A short delay is added to let the sensor settle before the next reading.
    delay(10);
    // The measured distance in centimeters is returned.
    return distance;
}

// ============================================================
// OBSTACLE DETECTION
// ============================================================

/**
 * DETECT OBSTACLE DISTANCE
 * The Detect_obstacle_distance function sweeps the servo to the left and right and takes distance
 * measurements in both directions. The servo is moved to 160 degrees (left) and three readings are
 * taken, keeping only the last one. Then the servo is moved to 20 degrees (right) and three readings
 * are taken, again keeping only the last one. The results are stored in the global variables DL and DR.
 */
void Detect_obstacle_distance() {
    // The servo is moved to 160 degrees to point the ultrasonic sensor to the left.
    myservo.write(160);
    // Three readings are taken to the left, with a delay between each to let the sensor settle.
    for (int i = 1; i <= 3; i = i + (1)) {
        // The distance to the left is measured and stored in DL, overwriting any previous reading.
        DL = checkdistance();
        // A 100-millisecond delay is added between readings.
        delay(100);
    }
    // The servo is moved to 20 degrees to point the ultrasonic sensor to the right.
    myservo.write(20);
    // Three readings are taken to the right, with a delay between each to let the sensor settle.
    for (int i = 1; i <= 3; i = i + (1)) {
        // The distance to the right is measured and stored in DR, overwriting any previous reading.
        DR = checkdistance();
        // A 100-millisecond delay is added between readings.
        delay(100);
    }
    // The servo is returned to center after the sweep is complete.
    myservo.write(90);
}

// ============================================================
// MOTOR HELPERS
// ============================================================

/**
 * SET SPEED
 * The Set_Speed function sets the PWM duty cycle for both motors to the same value.
 * The pwm parameter is the desired speed from 0 (stopped) to 255 (full speed).
 */
void Set_Speed(unsigned char pwm) {
    // The left motor PWM pin is set to the specified speed.
    analogWrite(Lpwm_pin, pwm);
    // The right motor PWM pin is set to the specified speed.
    analogWrite(Rpwm_pin, pwm);
}

/**
 * ADVANCE
 * The advance function sets both motors to drive forward.
 * The forward direction pin is set LOW and the backward direction pin is set HIGH for each motor.
 */
void advance() {
    // The right motor forward pin is set LOW as part of the forward direction configuration.
    digitalWrite(pinRF, LOW);
    // The right motor backward pin is set HIGH to complete the forward direction configuration.
    digitalWrite(pinRB, HIGH);
    // The left motor forward pin is set LOW as part of the forward direction configuration.
    digitalWrite(pinLF, LOW);
    // The left motor backward pin is set HIGH to complete the forward direction configuration.
    digitalWrite(pinLB, HIGH);
}

/**
 * TURN RIGHT
 * The turnR function spins the robot to the right by driving the right motor forward
 * and the left motor backward.
 */
void turnR() {
    // The right motor forward pin is set LOW as part of the forward direction configuration.
    digitalWrite(pinRF, LOW);
    // The right motor backward pin is set HIGH to complete the forward direction configuration.
    digitalWrite(pinRB, HIGH);
    // The left motor forward pin is set HIGH as part of the backward direction configuration.
    digitalWrite(pinLF, HIGH);
    // The left motor backward pin is set LOW to complete the backward direction configuration.
    digitalWrite(pinLB, LOW);
}

/**
 * TURN LEFT
 * The turnL function spins the robot to the left by driving the left motor forward
 * and the right motor backward.
 */
void turnL() {
    // The right motor forward pin is set HIGH as part of the backward direction configuration.
    digitalWrite(pinRF, HIGH);
    // The right motor backward pin is set LOW to complete the backward direction configuration.
    digitalWrite(pinRB, LOW);
    // The left motor forward pin is set LOW as part of the forward direction configuration.
    digitalWrite(pinLF, LOW);
    // The left motor backward pin is set HIGH to complete the forward direction configuration.
    digitalWrite(pinLB, HIGH);
}

/**
 * STOP
 * The stopp function stops both motors by setting all four direction pins to HIGH.
 * When both direction pins for a motor are HIGH, the H-bridge shorts the motor terminals together,
 * which brakes the motor instead of letting it coast.
 */
void stopp() {
    // The right motor forward pin is set HIGH as part of the braking configuration.
    digitalWrite(pinRF, HIGH);
    // The right motor backward pin is set HIGH to complete the braking configuration.
    digitalWrite(pinRB, HIGH);
    // The left motor forward pin is set HIGH as part of the braking configuration.
    digitalWrite(pinLF, HIGH);
    // The left motor backward pin is set HIGH to complete the braking configuration.
    digitalWrite(pinLB, HIGH);
}

// ============================================================
// SETUP
// ============================================================

/**
 * SETUP
 * The setup function runs once when the Arduino is powered on or reset.
 * It attaches the servo, configures all motor and ultrasonic sensor pins,
 * initializes the distance variables to zero, centers the servo to face straight ahead,
 * and starts serial communication for debugging.
 */
void setup() {
    // Serial communication is started at 9600 baud for printing debug messages to the serial monitor.
    Serial.begin(9600);
    // The servo is attached to pin A2.
    myservo.attach(A2);
    // The ultrasonic sensor TRIG pin on A1 is configured as an output.
    pinMode(A1, OUTPUT);
    // The ultrasonic sensor ECHO pin on A0 is configured as an input.
    pinMode(A0, INPUT);
    // The left motor forward direction pin is configured as an output.
    pinMode(pinLF, OUTPUT);
    // The left motor backward direction pin is configured as an output.
    pinMode(pinLB, OUTPUT);
    // The right motor forward direction pin is configured as an output.
    pinMode(pinRF, OUTPUT);
    // The right motor backward direction pin is configured as an output.
    pinMode(pinRB, OUTPUT);
    // The left motor PWM speed control pin is configured as an output.
    pinMode(Lpwm_pin, OUTPUT);
    // The right motor PWM speed control pin is configured as an output.
    pinMode(Rpwm_pin, OUTPUT);
    // The left distance variable is initialized to zero.
    DL = 0;
    // The middle (front) distance variable is initialized to zero.
    DM = 0;
    // The right distance variable is initialized to zero.
    DR = 0;
    // The servo is centered to 90 degrees so the ultrasonic sensor faces straight ahead.
    myservo.write(90);

    // The program name is printed to the serial monitor to confirm the sketch is running.
    Serial.println("=== OBJECT CONTACT CAR ===");
    // The detection and contact distances are printed so the user can verify the current settings.
    Serial.print("Detect at: ");
    Serial.print(DETECT_CM, 1);
    Serial.print("cm  Contact at: ");
    Serial.print(CONTACT_CM, 1);
    Serial.println("cm");
    // A countdown message is printed to give the user time to place the robot in position.
    Serial.println("Starting in 3s...");
    // The program pauses for three seconds before the robot begins moving.
    delay(3000);
    // A final message is printed to indicate the robot is now active.
    Serial.println("GO!");
}

// ============================================================
// MAIN LOOP
// ============================================================

/**
 * MAIN LOOP
 * The loop function runs repeatedly after setup.
 * If the robot has already made contact with an object, the motors stay stopped and nothing else happens.
 * Otherwise, the robot measures the distance straight ahead each iteration.
 * If the distance is below CONTACT_CM, the robot performs a final push and stops permanently.
 * If the sensor previously detected a nearby object and then stops returning valid readings,
 * the object is likely too close for the sensor to measure, so contact is assumed.
 * If the distance is below DETECT_CM, the robot stops, sweeps the servo left and right to find
 * which direction the object is closer, turns toward the closer side, and then drives forward
 * at a slower approach speed.
 * If no object is detected, the robot drives forward at normal speed.
 */
void loop() {
    // If the robot has already made contact with an object, the motors stay stopped and nothing else happens.
    if (contactMade) {
        return;
    }

    // The distance straight ahead is measured and stored in DM.
    DM = checkdistance();

    // The front distance is printed to the serial monitor for debugging.
    Serial.print("[OBJ] Front=");
    Serial.print(DM);
    Serial.print("cm  ->  ");

    // If the front distance is below CONTACT_CM and above zero, the robot has reached the object.
    if (DM > 0 && DM < CONTACT_CM) {
        Serial.println("CONTACT! Final push...");
        // The motor direction pins are set to drive forward for the final push.
        advance();
        // The motor speed is set to the approach speed for the final push.
        Set_Speed(APPROACH_SPEED);
        // The robot pushes forward for FINAL_PUSH_MS milliseconds to ensure it touches the object.
        delay(FINAL_PUSH_MS);
        // The motors are braked after the final push.
        stopp();
        // The motor speed is set to zero.
        Set_Speed(0);
        // The contact flag is set so the robot does not move again.
        contactMade = true;
        Serial.println("=== SUCCESS: Object reached! ===");
        return;
    }

    // If the sensor previously saw a nearby object but now returns zero, the object is likely too close to measure.
    if (objectSeen && DM == 0) {
        Serial.println("CONTACT (sensor lost object at close range)! Final push...");
        // The motor direction pins are set to drive forward for the final push.
        advance();
        // The motor speed is set to the approach speed for the final push.
        Set_Speed(APPROACH_SPEED);
        // The robot pushes forward for FINAL_PUSH_MS milliseconds to ensure it touches the object.
        delay(FINAL_PUSH_MS);
        // The motors are braked after the final push.
        stopp();
        // The motor speed is set to zero.
        Set_Speed(0);
        // The contact flag is set so the robot does not move again.
        contactMade = true;
        Serial.println("=== SUCCESS: Object reached! ===");
        return;
    }

    // If the front sensor detects an object within DETECT_CM, the robot begins the approach sequence.
    if (DM > 0 && DM < DETECT_CM) {
        // The object has been seen, so the flag is set for the close-range contact check.
        objectSeen = true;
        // The motors are braked while the servo sweeps.
        stopp();
        // The motor speed is set to zero during the sweep.
        Set_Speed(0);
        // The servo sweeps left and right, storing the measured distances in DL and DR.
        Detect_obstacle_distance();

        // The left and right distances are printed to the serial monitor for debugging.
        Serial.print("APPROACHING — DL=");
        Serial.print(DL);
        Serial.print("cm  DR=");
        Serial.print(DR);

        // The robot turns toward whichever side has the closer object.
        if (DL < DR) {
            // The left side has a closer object, so the robot turns left toward it.
            Serial.println("  -> Turn LEFT toward object");
            turnL();
            Set_Speed(TURN_SPEED);
            delay(TURN_MS);
        } else {
            // The right side has a closer object, or both are equal, so the robot turns right toward it.
            Serial.println("  -> Turn RIGHT toward object");
            turnR();
            Set_Speed(TURN_SPEED);
            delay(TURN_MS);
        }

        // After turning, the robot drives forward at the slower approach speed.
        advance();
        Set_Speed(APPROACH_SPEED);
    } else {
        // If no object is detected within DETECT_CM, the object-seen flag is cleared.
        objectSeen = false;
        // The robot drives forward at normal speed.
        advance();
        Set_Speed(DRIVE_SPEED);
        Serial.println("DRIVING (no object)");
    }
}
