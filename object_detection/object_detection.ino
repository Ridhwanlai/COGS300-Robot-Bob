/**
  ============================================================
  OBJECT DETECTION: DRIVE FORWARD AND STOP ON CONTACT
  ============================================================

  PIN ASSIGNMENTS:
    The front HC-SR04 ultrasonic sensor has its TRIG pin connected to A1 and its ECHO pin connected to A0.
    The right HC-SR04 ultrasonic sensor has its TRIG pin connected to A3 and its ECHO pin connected to A4.

    The ENA pin for left motor PWM speed control is connected to D5.
    The ENB pin for right motor PWM speed control is connected to D6.
    The IN1 pin for left motor forward direction is connected to D2.
    The IN2 pin for left motor backward direction is connected to D4.
    The IN3 pin for right motor forward direction is connected to D7.
    The IN4 pin for right motor backward direction is connected to D8.

  BEHAVIOR:
    The robot drives forward automatically.
    When the front sensor detects an object within OD_DETECT_CM, the robot slows down to its approach speed.
    When the front sensor reading drops below OD_CONTACT_CM, or the front sensor stops returning valid
    readings after previously detecting a nearby object, the robot considers itself in contact and stops permanently.

  TUNING:
    OD_DETECT_CM is the distance at which the robot first notices an object ahead and slows down.
    OD_CONTACT_CM is the distance at which the robot considers itself close enough to be in contact with the object.
    OD_DRIVE_SPEED is the PWM speed used when no object is detected and the robot is driving forward normally.
    OD_APPROACH_SPEED is the slower PWM speed used when the robot is approaching a detected object.
  ============================================================
*/
#include <Arduino.h>
#include <pins_arduino.h>

// --- Motor Pins ---

// The left motor PWM speed control pin is connected to D5.
const int L_PWM_PIN = 5;
// The right motor PWM speed control pin is connected to D6.
const int R_PWM_PIN = 6;
// The left motor forward direction pin is connected to D2.
const int PIN_LF    = 2;
// The left motor backward direction pin is connected to D4.
const int PIN_LB    = 4;
// The right motor forward direction pin is connected to D7.
const int PIN_RF    = 7;
// The right motor backward direction pin is connected to D8.
const int PIN_RB    = 8;

// --- Ultrasonic Pins ---

// The front ultrasonic sensor TRIG pin is connected to A1.
const int TRIG_FRONT_PIN = A1;
// The front ultrasonic sensor ECHO pin is connected to A0.
const int ECHO_FRONT_PIN = A0;
// The right ultrasonic sensor TRIG pin is connected to A3.
const int TRIG_RIGHT_PIN = A3;
// The right ultrasonic sensor ECHO pin is connected to A4.
const int ECHO_RIGHT_PIN = A4;

// --- Object Detection Constants ---
// The robot first notices an object ahead and slows down when the front sensor reads below this value in centimeters.
const float OD_DETECT_CM       = 30.0f;
// The robot considers itself in contact with the object when the front sensor reads below this value in centimeters.
const float OD_CONTACT_CM      = 4.0f;
// This is the PWM speed used when no object is detected and the robot is driving forward normally.
const int   OD_DRIVE_SPEED     = 110;
// This is the slower PWM speed used when the robot is approaching a detected object.
const int   OD_APPROACH_SPEED  = 60;

/**
 * SMA FILTER
 * SMA stands for Simple Moving Average.
 * It is a smoothing filter that averages the last few ultrasonic sensor readings to reduce noise and jitter.
 * Individual HC-SR04 readings can spike or drop randomly, so instead of reacting to each raw reading,
 * the code keeps a buffer of recent readings and uses their average.
 * Without this filter, the robot would constantly jitter as the sensor produces noisy readings.
 * The SMA ensures the robot makes decisions based on a stable, representative distance rather than
 * whatever noise the sensor happened to produce on one particular cycle.
 * US_SMA_WINDOW defines how many recent readings are stored and averaged.
 */
#define US_SMA_WINDOW 4

// --- Ultrasonic Timing ---
// This is the maximum time in microseconds to wait for the ultrasonic sensor's ECHO pin to receive a reflected signal.
const unsigned long US_PULSE_TIMEOUT = 30000UL;
// Any ultrasonic reading below this value in centimeters is considered invalid and discarded
// because the sensor cannot reliably distinguish the outgoing sound burst from the returning echo at very short distances.
const float         US_MIN_CM        = 2.0f;
// Any ultrasonic reading above this value in centimeters is considered invalid and discarded
// because the reflected sound signal is too weak for the sensor to reliably detect at that distance.
const float         US_MAX_CM        = 400.0f;
// The last valid ultrasonic reading is held for this many milliseconds before being considered too old to reflect the robot's current position.
const unsigned long US_HOLD_MS       = 200;
// This is the minimum time in milliseconds between consecutive ultrasonic sensor readings.
const unsigned long US_PERIOD_MS     = 60;

// ============================================================
// GLOBALS
// ============================================================

// This buffer stores the most recent front ultrasonic sensor readings for the SMA filter.
float frontBuf[US_SMA_WINDOW];
// This is the current write index into the front sensor SMA buffer.
int   frontBufIdx   = 0;
// This is the number of valid readings currently stored in the front sensor SMA buffer.
int   frontBufCount = 0;

// This is the filtered front sensor distance in centimeters, computed by the SMA filter.
float         frontCmFilt   = -1.0f;
// This flag indicates whether the front sensor has produced at least one valid reading since startup.
bool          frontInit     = false;
// This is the timestamp in milliseconds of the last valid front sensor reading.
unsigned long lastFrontOkMs = 0;
// This is the timestamp in milliseconds of the last ultrasonic sensor read cycle.
unsigned long lastUltraMs   = 0;

// This flag is set to true once the robot has made contact with an object and stopped permanently.
bool          contactMade   = false;
// This flag tracks whether the front sensor has recently detected an object within OD_DETECT_CM.
bool          objectSeen    = false;

// ============================================================
// MOTOR HELPERS
// ============================================================

/**
 * MOTOR HELPERS
 * The leftCmd parameter is the desired speed and direction for the left motor,
 * where positive values mean forward and negative values mean backward.
 * The rightCmd parameter is the desired speed and direction for the right motor,
 * where positive values mean forward and negative values mean backward.
 */
void setMotors(int leftCmd, int rightCmd) {
    // The left motor command is clamped to the valid PWM range of -255 to 255.
    leftCmd  = constrain(leftCmd,  -255, 255);
    // The right motor command is clamped to the valid PWM range of -255 to 255.
    rightCmd = constrain(rightCmd, -255, 255);
    // If the left motor command is zero or positive, the left motor is set to the forward direction.
    if (leftCmd >= 0) {
        // The left motor forward pin is set LOW as part of the forward direction configuration.
        digitalWrite(PIN_LF, LOW);
        // The left motor backward pin is set HIGH to complete the forward direction configuration.
        digitalWrite(PIN_LB, HIGH);
    } else {
        // The left motor forward pin is set HIGH as part of the backward direction configuration.
        digitalWrite(PIN_LF, HIGH);
        // The left motor backward pin is set LOW to complete the backward direction configuration.
        digitalWrite(PIN_LB, LOW);
    }
    // If the right motor command is zero or positive, the right motor is set to the forward direction.
    if (rightCmd >= 0) {
        // The right motor forward pin is set LOW as part of the forward direction configuration.
        digitalWrite(PIN_RF, LOW);
        // The right motor backward pin is set HIGH to complete the forward direction configuration.
        digitalWrite(PIN_RB, HIGH);
    } else {
        // The right motor forward pin is set HIGH as part of the backward direction configuration.
        digitalWrite(PIN_RF, HIGH);
        // The right motor backward pin is set LOW to complete the backward direction configuration.
        digitalWrite(PIN_RB, LOW);
    }
    // The absolute value of leftCmd is written to the left motor PWM pin
    // because the direction pins already handle forward or backward,
    // so only the magnitude is needed here to control speed.
    analogWrite(L_PWM_PIN, abs(leftCmd));
    // The absolute value of rightCmd is written to the right motor PWM pin
    // because the direction pins already handle forward or backward,
    // so only the magnitude is needed here to control speed.
    analogWrite(R_PWM_PIN, abs(rightCmd));
}

void stopMotors() {
    // The left motor PWM duty cycle is set to 0%, which cuts power to the left motor and stops it from spinning.
    analogWrite(L_PWM_PIN, 0);
    // The right motor PWM duty cycle is set to 0%, which cuts power to the right motor and stops it from spinning.
    analogWrite(R_PWM_PIN, 0);
    // The left motor forward direction pin is set LOW so that both direction pins are at the same potential,
    // which prevents any unintended current from flowing through the left motor.
    digitalWrite(PIN_LF, LOW);
    // The left motor backward direction pin is also set LOW to match,
    // ensuring no voltage difference exists across the left motor terminals.
    digitalWrite(PIN_LB, LOW);
    // The right motor forward direction pin is set LOW so that both direction pins are at the same potential,
    // which prevents any unintended current from flowing through the right motor.
    digitalWrite(PIN_RF, LOW);
    // The right motor backward direction pin is also set LOW to match,
    // ensuring no voltage difference exists across the right motor terminals.
    digitalWrite(PIN_RB, LOW);
}

// ============================================================
// SMA FILTER HELPERS
// ============================================================

/**
 * SMA ADD
 * The smaAdd function adds a new sensor reading to a circular buffer used by the Simple Moving Average filter.
 * It stores the new value at the current write position in the buffer, replacing whatever was there before.
 * Once the buffer is full, new readings overwrite the oldest ones so the average always reflects the most recent data.
 */
void smaAdd(float* buf, int* idx, int* count, float val) {
    // The new reading is stored in the buffer at the current write index, overwriting the oldest value if the buffer is full.
    buf[*idx] = val;
    // The write index is advanced by one and wraps back to zero when it reaches the end of the buffer.
    *idx = (*idx + 1) % US_SMA_WINDOW;
    // If the buffer has not yet been completely filled, the count of valid readings is incremented by one.
    if (*count < US_SMA_WINDOW) {
        (*count)++;
    }
}

// The smaGet function returns the average of all valid readings in the buffer, or -1.0 if the buffer is empty.
float smaGet(float* buf, int count) {
    if (count == 0) {
        return -1.0f;
    }
    float sum = 0;
    for (int i = 0; i < count; i++) {
        sum += buf[i];
    }
    return sum / count;
}

// ============================================================
// ULTRASONIC HELPERS
// ============================================================

/**
 * READ ULTRASONIC CENTIMETERS
 * The readUsCm function triggers an ultrasonic sensor, measures the echo duration,
 * and converts it to a distance in centimeters.
 * If no echo is received or the distance is outside the valid range, it returns -1.0.
 */
float readUsCm(int trigPin, int echoPin) {
    // The TRIG pin is set LOW to ensure a clean start before sending the trigger pulse.
    digitalWrite(trigPin, LOW);
    // A 2-microsecond delay allows the TRIG pin to settle at LOW before the pulse begins.
    delayMicroseconds(2);
    // The TRIG pin is set HIGH to begin the trigger pulse that tells the sensor to emit an ultrasonic burst.
    digitalWrite(trigPin, HIGH);
    // The TRIG pin is held HIGH for 10 microseconds, which is the pulse duration required by the HC-SR04 sensor.
    delayMicroseconds(10);
    // The TRIG pin is set back to LOW to end the trigger pulse.
    digitalWrite(trigPin, LOW);
    // The duration in microseconds that the ECHO pin stays HIGH is measured,
    // which represents the round-trip travel time of the sound.
    unsigned long dur = pulseIn(echoPin, HIGH, US_PULSE_TIMEOUT);
    // If the duration is zero, no echo was received within the timeout, so the reading is invalid.
    if (dur == 0) {
        return -1.0f;
    }
    // The duration is converted to centimeters by multiplying by the speed of sound and dividing by two for the round trip.
    float cm = (dur * 0.0343f) / 2.0f;
    // If the computed distance is below the minimum or above the maximum valid range, the reading is discarded as unreliable.
    if (cm < US_MIN_CM || cm > US_MAX_CM) {
        return -1.0f;
    }
    // The valid distance in centimeters is returned.
    return cm;
}

/**
 * UPDATE ULTRASONICS
 * The updateUltrasonics function reads the front ultrasonic sensor and updates
 * its filtered distance value using the SMA filter.
 * It only performs a reading if enough time has passed since the last one.
 */
void updateUltrasonics() {
    // The current time in milliseconds is captured for use in timing checks and timestamps.
    unsigned long now = millis();
    // If not enough time has passed since the last reading cycle, the function returns early without doing anything.
    if (now - lastUltraMs < US_PERIOD_MS) {
        return;
    }
    // The timestamp of the last reading cycle is updated to the current time.
    lastUltraMs = now;
    // The front ultrasonic sensor is read and the result is stored in a temporary variable.
    float fr = readUsCm(TRIG_FRONT_PIN, ECHO_FRONT_PIN);
    // If the front sensor returned a valid reading, it is processed through the SMA filter.
    if (fr > 0) {
        // The valid front reading is added to the front sensor's SMA buffer.
        smaAdd(frontBuf, &frontBufIdx, &frontBufCount, fr);
        // The filtered front distance is recalculated as the average of all valid readings in the buffer.
        frontCmFilt   = smaGet(frontBuf, frontBufCount);
        // The front sensor is marked as initialized, meaning it has produced at least one valid reading.
        frontInit     = true;
        // The timestamp of the last valid front reading is updated to the current time.
        lastFrontOkMs = now;
    }
}

// The frontValid function returns true if the front sensor has been initialized
// and the last valid reading was received within US_HOLD_MS milliseconds of the current time.
bool frontValid() {
    return frontInit && (millis() - lastFrontOkMs <= US_HOLD_MS);
}

// The getFrontCm function returns the filtered front distance if valid, or -1.0 if unavailable.
float getFrontCm() {
    if (frontValid()) {
        return frontCmFilt;
    } else {
        return -1.0f;
    }
}

// ============================================================
// SETUP
// ============================================================

/**
 * SETUP
 * The setup function runs once when the Arduino is powered on or reset.
 * It initializes serial communication, configures all pins, stops the motors,
 * zeros out the SMA buffer, prints the configuration, and waits three seconds before starting.
 */
void setup() {
    // Serial communication is started at 9600 baud for printing debug messages to the serial monitor.
    Serial.begin(9600);
    // The left motor forward and backward direction pins are configured as outputs.
    pinMode(PIN_LF, OUTPUT); pinMode(PIN_LB, OUTPUT);
    // The right motor forward and backward direction pins are configured as outputs.
    pinMode(PIN_RF, OUTPUT); pinMode(PIN_RB, OUTPUT);
    // The left and right motor PWM speed control pins are configured as outputs.
    pinMode(L_PWM_PIN, OUTPUT); pinMode(R_PWM_PIN, OUTPUT);
    // The motors are stopped immediately to ensure the robot does not move during initialization.
    stopMotors();
    // The front ultrasonic sensor TRIG pin is configured as an output and the ECHO pin is configured as an input.
    pinMode(TRIG_FRONT_PIN, OUTPUT); pinMode(ECHO_FRONT_PIN, INPUT);
    // The right ultrasonic sensor TRIG pin is configured as an output and the ECHO pin is configured as an input.
    pinMode(TRIG_RIGHT_PIN, OUTPUT); pinMode(ECHO_RIGHT_PIN, INPUT);
    // The front TRIG pin is set LOW to ensure it is not sending a signal before the first reading.
    digitalWrite(TRIG_FRONT_PIN, LOW);
    // The right TRIG pin is set LOW to ensure it is not sending a signal before the first reading.
    digitalWrite(TRIG_RIGHT_PIN, LOW);

    // The SMA filter buffer is filled with zeros so that the averaging starts from a clean state.
    for (int i = 0; i < US_SMA_WINDOW; i++) {
        frontBuf[i] = 0;
    }

    // The program name is printed to the serial monitor to confirm the sketch is running.
    Serial.println("=== OBJECT DETECTION: DRIVE AND CONTACT ===");
    // The detection and contact distances are printed so the user can verify the current settings.
    Serial.print("Detect at: ");
    Serial.print(OD_DETECT_CM, 1);
    Serial.print("cm  Contact at: ");
    Serial.print(OD_CONTACT_CM, 1);
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
 * The robot drives forward at normal speed until the front sensor detects an object within OD_DETECT_CM.
 * Once an object is detected, the robot slows down to OD_APPROACH_SPEED to approach it carefully.
 * When the front sensor reading drops below OD_CONTACT_CM, the robot considers itself in contact
 * with the object and stops permanently.
 * If the front sensor previously detected a nearby object and then stops returning valid readings,
 * that also counts as contact because the object is likely too close for the sensor to measure.
 */
void loop() {
    // If the robot has already made contact with an object, the motors stay stopped and nothing else happens.
    if (contactMade) {
        return;
    }

    // The front ultrasonic sensor reading is updated if enough time has passed since the last reading.
    updateUltrasonics();

    // The current filtered front sensor distance is retrieved, or -1.0 if unavailable.
    float f = getFrontCm();

    // The front sensor status is printed to the serial monitor for debugging.
    Serial.print("[OBJ] Front=");
    if (frontValid()) {
        Serial.print(f, 1);
        Serial.print("cm");
    } else {
        Serial.print("---");
    }
    Serial.print("  ->  ");

    // If the front sensor has a valid reading and the object is closer than OD_CONTACT_CM, contact is made.
    if (frontValid() && f > 0 && f < OD_CONTACT_CM) {
        stopMotors();
        contactMade = true;
        Serial.println("CONTACT! Stopped.");
        return;
    }

    // If the front sensor previously saw an object nearby but now returns no valid reading,
    // the object is likely too close for the sensor to measure, so contact is assumed.
    if (objectSeen && !frontValid()) {
        stopMotors();
        contactMade = true;
        Serial.println("CONTACT (sensor lost object at close range)! Stopped.");
        return;
    }

    // If the front sensor detects an object within OD_DETECT_CM, the robot slows down to approach it.
    if (frontValid() && f > 0 && f < OD_DETECT_CM) {
        objectSeen = true;
        setMotors(OD_APPROACH_SPEED, OD_APPROACH_SPEED);
        Serial.println("APPROACHING (object detected)");
    } else {
        // If no object is detected within OD_DETECT_CM, the robot drives forward at normal speed.
        objectSeen = false;
        setMotors(OD_DRIVE_SPEED, OD_DRIVE_SPEED);
        Serial.println("DRIVING (no object)");
    }
}
