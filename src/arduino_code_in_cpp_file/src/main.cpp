/**
  ============================================================
  TEST 2: WALL FOLLOWING ONLY
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

  STATE LOGIC (in priority order):
    1. If the robot is stuck, a two-tier recovery sequence is triggered based on the front sensor only.
    2. If the front is near and the right wall is visible, the robot turns left because it is in a corner.
    3. If the front is near and the right wall is lost, the robot turns right
       because the front is blocked but no wall is on the right.
    4. If the front is far and the right wall is visible, the robot performs distance maintenance:
        If the distance is less than WF_RIGHT_NEAR_CM - WF_RIGHT_TOLERANCE_CM,
        the robot corrects left because it is too close.
        If the distance is greater than WF_RIGHT_NEAR_CM + WF_RIGHT_TOLERANCE_CM,
        the robot corrects right because it is too far.
        If the distance is within WF_RIGHT_NEAR_CM - WF_RIGHT_TOLERANCE_CM and WF_RIGHT_NEAR_CM + WF_RIGHT_TOLERANCE_CM,
        the robot drives straight.
    5. If the front is far and the right ultrasonic sensor can no longer detect a wall,
        the robot drives straight because it is in open space with no wall seen.

  The rightVisible flag is true when the distance measured by the right ultrasonic sensor is less than WF_RIGHT_LOST_CM,
  meaning the right ultrasonic sensor detects a wall within WF_RIGHT_LOST_CM.
    The tooClose flag is true when the distance measured by the right ultrasonic sensor is less than
    WF_RIGHT_NEAR_CM - WF_RIGHT_LOST_CM.
    The tooFar flag is true when the distance measured by the right ultrasonic sensor is greater than
    WF_RIGHT_NEAR_CM + WF_RIGHT_LOST_CM.
    inBand: The inBand flag is true when the distance measured by the right ultrasonic sensor
    is between WF_RIGHT_NEAR_CM - WF_RIGHT_TOLERANCE_CM and WF_RIGHT_NEAR_CM + WF_RIGHT_TOLERANCE_CM.

    This means even when the robot drifts far from the wall, as long as the wall is still visible,
    it will correct back toward it.

  TUNING:
    WF_RIGHT_NEAR_CM is the desired distance from the right wall that the robot tries to maintain.

    WF_RIGHT_TOLERANCE_CM is the acceptable range around WF_RIGHT_NEAR_CM within which
    the robot drives straight without correcting. It is measured as plus or minus this value in centimeters.

    WF_RIGHT_LOST_CM is the threshold above which the right sensor reading means
    the right ultrasonic sensor can no longer detect a wall.
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

// --- Wall-Follow Ultrasonic Pins ---

// The front ultrasonic sensor TRIG pin is connected to A1.
const int TRIG_FRONT_PIN = A1;
// The front ultrasonic sensor ECHO pin is connected to A0.
const int ECHO_FRONT_PIN = A0;
// The right ultrasonic sensor TRIG pin is connected to A3.
const int TRIG_RIGHT_PIN = A3;
// The right ultrasonic sensor ECHO pin is connected to A4.
const int ECHO_RIGHT_PIN = A4;

// --- Tunable Constants ---
// The front sensor considers the path blocked when the distance is closer than this value.
const float          WF_FRONT_NEAR_CM       = 11.0f;
// This is the desired distance from the right wall that the robot tries to maintain.
const float          WF_RIGHT_NEAR_CM       = 8.0f;
/**
 This is the acceptable range around WF_RIGHT_NEAR_CM where the robot drives straight without correcting,
 measured as plus or minus this value in centimeters.
 * */
const float          WF_RIGHT_TOLERANCE_CM  = 1.0f;
// The right ultrasonic sensor can no longer detect a wall when the reading is above this value.
const float          WF_RIGHT_LOST_CM       = 50.0f;
// This is the PWM speed used for both wheels when driving straight.
const int            WF_STRAIGHT_SPEED      = 110;
// This is the PWM speed used for hard pivot turns such as corners and right turns.
const int            WF_TURN_SPEED          = 120;
// This is the PWM speed for the faster wheel when the robot curves to adjust its distance from the wall.
const int            WF_CORRECT_OUTER       = 110;
// This is the PWM speed for the slower wheel when the robot curves to adjust its distance from the wall.
const int            WF_CORRECT_INNER       = 60;

/**
 * SMA FILTER
 * SMA stands for Simple Moving Average.
 * It is a smoothing filter that averages the last few ultrasonic sensor readings to reduce noise and jitter.
 * Individual HC-SR04 readings can spike or drop randomly, so instead of reacting to each raw reading,
 * the code keeps a buffer of recent readings and uses their average.
 * Without this filter, the robot would constantly jitter between "too close" and "too far" as the sensor
 * produces noisy readings, causing the motors to switch directions rapidly and the robot to wobble.
 * The SMA ensures the robot makes decisions based on a stable, representative distance rather than
 * whatever noise the sensor happened to produce on one particular cycle.
 * US_SMA_WINDOW defines how many recent readings are stored and averaged.
 */
#define US_SMA_WINDOW 4

// --- Ultrasonic Timing ---
// This is the maximum time in microseconds to wait for the ultrasonic sensor's ECHO pin to receive a reflected signal.
const unsigned long  US_PULSE_TIMEOUT    = 30000UL;
/**
Any ultrasonic reading below this value in centimeters is considered invalid and discarded
because the sensor cannot reliably distinguish the outgoing sound burst from the returning echo at very short distances.
 * */
const float          US_MIN_CM           = 2.0f;
/**
 Any ultrasonic reading above this value in centimeters is considered invalid and discarded
 because the reflected sound signal is too weak for the sensor to reliably detect at that distance.
 Readings above this value are most likely caused by noise, stray echoes,
 or the signal simply not returning in a meaningful way.
 * */
const float          US_MAX_CM           = 400.0f;
// The last valid ultrasonic reading is held for this many milliseconds before being considered stale.
const unsigned long  US_HOLD_MS          = 200;
// This is the minimum time in milliseconds between consecutive ultrasonic sensor readings.
const unsigned long  US_PERIOD_MS        = 60;


// The robot is considered stuck if the front sensor reading stays unchanged for this many milliseconds.
const unsigned long  US_STUCK_MS           = 2000UL;
// The front sensor reading is considered unchanged if it varies by less than this many centimeters.
const float          US_STUCK_BAND_CM      = 1.5f;
// This is the PWM speed used for reversing during the first stuck recovery attempt.
const int            US_STUCK_REVERSE_PWM_1  = 255;
// This is how long in milliseconds the robot reverses during the first stuck recovery attempt.
const int            US_STUCK_REVERSE_MS_1   = 500;
// This is the PWM speed used for the forward push during the first stuck recovery attempt.
const int            US_STUCK_PUSH_PWM_1     = 255;
// This is how long in milliseconds the robot pushes forward during the first stuck recovery attempt.
const int            US_STUCK_PUSH_MS_1      = 1500;
// This is the PWM speed used for reversing during every subsequent stuck recovery attempt.
const int            US_STUCK_REVERSE_PWM_2  = 200;
// This is how long in milliseconds the robot reverses during every subsequent stuck recovery attempt.
const int            US_STUCK_REVERSE_MS_2   = 250;

// This buffer stores the most recent front ultrasonic sensor readings for the SMA filter.
float frontBuf[US_SMA_WINDOW];
// This buffer stores the most recent right ultrasonic sensor readings for the SMA filter.
float rightBuf[US_SMA_WINDOW];
// This is the current write index into the front sensor SMA buffer.
int   frontBufIdx   = 0;
// This is the current write index into the right sensor SMA buffer.
int   rightBufIdx   = 0;
// This is the number of valid readings currently stored in the front sensor SMA buffer.
int   frontBufCount = 0;
// This is the number of valid readings currently stored in the right sensor SMA buffer.
int   rightBufCount = 0;
// This is the filtered front sensor distance in centimeters, computed by the SMA filter.
float         frontCmFilt   = -1.0f;
// This is the filtered right sensor distance in centimeters, computed by the SMA filter.
float         rightCmFilt   = -1.0f;
// This flag indicates whether the front sensor has produced at least one valid reading since startup.
bool          frontInit     = false;
// This flag indicates whether the right sensor has produced at least one valid reading since startup.
bool          rightInit     = false;
// This is the timestamp in milliseconds of the last valid front sensor reading.
unsigned long lastFrontOkMs = 0;
// This is the timestamp in milliseconds of the last valid right sensor reading.
unsigned long lastRightOkMs = 0;
// This is the timestamp in milliseconds of the last ultrasonic sensor read cycle.
unsigned long lastUltraMs   = 0;
// This stores the front sensor reference reading used to detect whether the robot is stuck.
float         stuckFrontRef    = -1.0f;
// This is the timestamp in milliseconds of when the front sensor reading first stopped changing.
unsigned long stuckSinceMs     = 0;
// This flag indicates whether the stuck detection timer is currently running.
bool          stuckTimerActive = false;
// This flag indicates whether the first full-power stuck recovery attempt has already been used.
bool          stuckFirstFired  = false;
/**
 * MOTOR HELPERS
 * The leftCmd parameter is the desired speed and direction for the left motor,
 * where positive values mean forward and negative values mean backward.
 * The rightCmd parameter is the desired speed and direction for the right motor,
 * where positive values mean forward and negative values mean backward.
 * */
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
    /**
     * The absolute value of leftCmd is written to the left motor PWM pin
     * because the direction pins already handle forward or backward,
     * so only the magnitude is needed here to control speed.
     * */
    analogWrite(L_PWM_PIN, abs(leftCmd));
    /**
     * The absolute value of rightCmd is written to the right motor PWM pin
     * because the direction pins already handle forward or backward,
     * so only the magnitude is needed here to control speed.
     * */
    analogWrite(R_PWM_PIN, abs(rightCmd));
}
void stopMotors() {
    // The left motor PWM duty cycle is set to 0%, which cuts power to the left motor and stops it from spinning.
    analogWrite(L_PWM_PIN, 0);
    // The right motor PWM duty cycle is set to 0%, which cuts power to the right motor and stops it from spinning.
    analogWrite(R_PWM_PIN, 0);
    /**
     * The left motor forward direction pin is set LOW so that both direction pins are at the same potential,
     * which prevents any unintended current from flowing through the left motor.
     * */
    digitalWrite(PIN_LF, LOW);
    /**
     * The left motor backward direction pin is also set LOW to match,
     * ensuring no voltage difference exists across the left motor terminals.
     * */
    digitalWrite(PIN_LB, LOW);
    /**
     * The right motor forward direction pin is set LOW so that both direction pins are at the same potential,
     * which prevents any unintended current from flowing through the right motor.
     * */
    digitalWrite(PIN_RF, LOW);
    /**
     * The right motor backward direction pin is also set LOW to match,
     * ensuring no voltage difference exists across the right motor terminals.
     * */
    digitalWrite(PIN_RB, LOW);
}
/**
 * The smaAdd function adds a new sensor reading to a circular buffer used by the Simple Moving Average filter.
 * It stores the new value at the current write position in the buffer, replacing whatever was there before.
 * After storing the value, it advances the write index by one and wraps it back to zero when it reaches
 * the end of the buffer, so the buffer is reused continuously without needing to shift any data.
 * If the buffer has not yet been completely filled with readings, the count of valid entries is incremented
 * so that the averaging function knows how many values to include.
 * Once the buffer is full, the count stops increasing and new readings simply overwrite the oldest ones,
 * so the average always reflects the most recent readings rather than being dragged by stale data.
 */
/**
 * The buf parameter is a pointer to the circular buffer array that stores the most recent sensor readings.
 *
 * The idx parameter is a pointer to the current write index in the buffer,
 * which tracks where the next reading should be stored.
 *
 * The count parameter is a pointer to the number of valid readings currently in the buffer,
 * which is needed to know when the buffer is full.
 *
 * The val parameter is the new sensor reading to be added to the buffer.
 * */
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
/**
 * The buf parameter is a pointer to the circular buffer array containing the sensor readings to be averaged.
 * The count parameter is the number of valid readings currently stored in the buffer,
 * which determines how many values to include in the average.
 * */
float smaGet(float* buf, int count) {
    /**
     * If there are no valid readings in the buffer,
     * the function returns -1.0 to indicate that no average can be computed.
     * */
    if (count == 0) {
        return -1.0f;
    }
    /**
     * The sum variable is initialized to zero and will accumulate the total of all valid readings in the buffer.
     * */
    float sum = 0;
    /**
     * Each valid reading in the buffer is added to the running sum one at a time.
     * */
    for (int i = 0; i < count; i++) {
        sum += buf[i];
    }
    /**
     * The sum is divided by the number of valid readings to produce the average,
     * which is returned as the filtered distance.
     * */
    return sum / count;
}
/**
 * READ ULTRASONIC CENTIMETERS
 * The readUsCm function triggers an ultrasonic sensor
 * and measures how long it takes for the sound to travel to an object and bounce back,
 * then converts that time into a distance in centimeters.
 * It first sends a short pulse on the TRIG pin to tell the sensor to emit a burst of ultrasonic sound.
 * It then measures how long the ECHO pin stays HIGH,
 * which corresponds to the round-trip travel time of the sound.
 * The travel time is converted to centimeters using the speed of sound (0.0343 cm per microsecond),
 * divided by two because the sound travels to the object and back.
 * If no echo is received within the timeout,
 * or if the computed distance falls outside the valid range,
 * the function returns -1.0 to indicate an invalid reading.
 */
 /**
  * The trigPin parameter is the Arduino pin connected to the TRIG pin of the ultrasonic sensor to be read.
  * The echoPin parameter is the Arduino pin connected to the ECHO pin of the ultrasonic sensor to be read.
  * */
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
     /**
      * The duration in microseconds that the ECHO pin stays HIGH is measured,
      * which represents the round-trip travel time of the sound.
      * */
    unsigned long dur = pulseIn(echoPin, HIGH, US_PULSE_TIMEOUT);
     // If the duration is zero, no echo was received within the timeout, so the reading is invalid.
    if (dur == 0) {
        return -1.0f;
    }
     // The duration is converted to centimeters
     // by multiplying by the speed of sound and dividing by two for the round trip.
    float cm = (dur * 0.0343f) / 2.0f;
     /**
      * If the computed distance is below the minimum or above the maximum valid range,
      * the reading is discarded as unreliable.
      * */
    if (cm < US_MIN_CM || cm > US_MAX_CM) {
        return -1.0f;
    }
     // The valid distance in centimeters is returned.
    return cm;
}
/**
 * UPDATE ULTRASONICS
 * The updateUltrasonics function reads both the front and right ultrasonic sensors
 * and updates their filtered distance values using the SMA filter.
 * It is called every loop iteration, but only performs a reading if enough time has passed since the last one,
 * as defined by US_PERIOD_MS, to avoid reading the sensors too frequently.
 * If a valid reading is obtained from a sensor, it is added to that sensor's SMA buffer,
 * the filtered average is recalculated, and the timestamp of the last valid reading is updated.
 * If the reading is invalid, the previous filtered value is kept and no update occurs for that sensor.
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
    // The right ultrasonic sensor is read and the result is stored in a temporary variable.
    float rr = readUsCm(TRIG_RIGHT_PIN, ECHO_RIGHT_PIN);
    // If the right sensor returned a valid reading, it is processed through the SMA filter.
    if (rr > 0) {
        // The valid right reading is added to the right sensor's SMA buffer.
        smaAdd(rightBuf, &rightBufIdx, &rightBufCount, rr);
        // The filtered right distance is recalculated as the average of all valid readings in the buffer.
        rightCmFilt   = smaGet(rightBuf, rightBufCount);
        // The right sensor is marked as initialized, meaning it has produced at least one valid reading.
        rightInit     = true;
        // The timestamp of the last valid right reading is updated to the current time.
        lastRightOkMs = now;
    }
}
/**
 * The frontValid function checks whether the front sensor's filtered reading can be trusted.
 * It returns true only if the front sensor has produced at least one valid reading since startup
 * and the most recent valid reading is not too old.
 * If the sensor has never returned a valid reading, or if the last valid reading was more than
 * US_HOLD_MS milliseconds ago, the function returns false to indicate the data is stale or unavailable.
 */
bool  frontValid() {
    /**
     * The function returns true only if the front sensor has been initialized
     * and the last valid reading was received within US_HOLD_MS milliseconds of the current time.
     * */
    return frontInit && (millis() - lastFrontOkMs <= US_HOLD_MS);
}
/**
 * The rightValid function checks whether the right sensor's filtered reading can be trusted.
 * It returns true only if the right sensor has produced at least one valid reading since startup
 * and the most recent valid reading is not too old.
 * If the sensor has never returned a valid reading, or if the last valid reading was more than
 * US_HOLD_MS milliseconds ago, the function returns false to indicate the data is stale or unavailable.
 */
bool  rightValid() {
    /**
     * The function returns true only if the right sensor has been initialized
     * and the last valid reading was received within US_HOLD_MS milliseconds of the current time.
     * */
    return rightInit && (millis() - lastRightOkMs <= US_HOLD_MS);
}
/**
 * The getFrontCm function returns the current filtered front sensor distance if the reading is valid,
 * or -1.0 if the front sensor data is unavailable or stale.
 * This provides a single safe way for the rest of the code to access the front sensor distance
 * without needing to check validity separately each time.
 */
float getFrontCm() {
    // If frontValid() returns true, the filtered distance in centimeters is returned.
    if (frontValid()) {
        return frontCmFilt;
    } else {
        /**
         * If the front sensor has no valid reading or the reading is stale,
         * -1.0 is returned to indicate unavailable data.
         * */
        return -1.0f;
    }
}
/**
 * The getRightCm function returns the current filtered right sensor distance if the reading is valid,
 * or -1.0 if the right sensor data is unavailable or stale.
 * This provides a single safe way for the rest of the code to access the right sensor distance
 * without needing to check validity separately each time.
 * */
float getRightCm() {
    // If rightValid() returns true, the filtered distance in centimeters is returned.
    if (rightValid()) {
        return rightCmFilt;
    } else {
        /**
         * If the right sensor has no valid reading or the reading is stale,
         * -1.0 is returned to indicate unavailable data.
         * */
        return -1.0f;
    }
}
/**
 * STUCK DETECTION
 * The stuck recovery system uses two tiers of power depending on how many times the robot has been stuck.
 * The first time the robot gets stuck, it performs a full-power reverse followed by a full-power forward push,
 * which is designed to overcome obstacles like ramps.
 * Every subsequent time the robot gets stuck, it performs a shorter and gentler reverse with no forward push,
 * which is enough to free it from a wall or minor snag without overshooting.
 * The stuckFirstFired boolean tracks whether the first tier has already been used.
 * */
/**
 * CHECK STUCK
 * The checkStuck function determines whether the robot is stuck by monitoring whether the front sensor
 * reading has stayed essentially the same for longer than US_STUCK_MS milliseconds.
 * Each time it is called, it compares the current front reading to a stored reference value.
 * If the reading has not changed by more than US_STUCK_BAND_CM, the stuck timer continues running.
 * If the reading has changed, the timer resets and the reference is updated to the new value.
 * When the timer exceeds US_STUCK_MS, the function triggers a recovery sequence.
 * The first time recovery is triggered, both motors reverse at full power and then push forward at full power
 * to overcome obstacles like ramps.
 * Every subsequent time, the motors perform a shorter and gentler reverse with no forward push,
 * which is enough to free the robot from a wall or minor snag without overshooting.
 * The function returns true if a recovery sequence was executed, or false otherwise.
 */
bool checkStuck(float f) {
    // The current time in milliseconds is captured for use in timing comparisons.
    unsigned long now = millis();
    // If the front sensor data is invalid or unavailable, stuck detection is paused and the timer is reset.
    if (!frontValid() || f <= 0) {
        stuckTimerActive = false;
        return false;
    }
    // The current front reading is compared to the stored reference to determine if it has changed by less than US_STUCK_BAND_CM.
    bool fStuck = stuckFrontRef >= 0 && abs(f - stuckFrontRef) < US_STUCK_BAND_CM;
    // If the front reading has not changed significantly, the robot may be stuck.
    if (fStuck) {
        // If the stuck timer is not already running, it is started and the current time is recorded.
        if (!stuckTimerActive) {
            stuckTimerActive = true;
            stuckSinceMs     = now;
            // If the stuck timer has been running for longer than US_STUCK_MS, a recovery sequence is triggered.
        } else if (now - stuckSinceMs > US_STUCK_MS) {
            // If this is the first time the robot has been stuck, the full-power recovery sequence is used.
            if (!stuckFirstFired) {
                // A message is printed to the serial monitor indicating the first stuck recovery has started.
                Serial.println("[STUCK-1] First stuck! Reversing then full power push...");
                // Both motors reverse at full power to back the robot away from the obstacle.
                setMotors(-US_STUCK_REVERSE_PWM_1, -US_STUCK_REVERSE_PWM_1);
                // The robot continues reversing for US_STUCK_REVERSE_MS_1 milliseconds.
                delay(US_STUCK_REVERSE_MS_1);
                // Both motors push forward at full power to drive the robot over or past the obstacle.
                setMotors(US_STUCK_PUSH_PWM_1, US_STUCK_PUSH_PWM_1);
                // The robot continues pushing forward for US_STUCK_PUSH_MS_1 milliseconds.
                delay(US_STUCK_PUSH_MS_1);
                // Both motors are stopped after the recovery sequence is complete.
                stopMotors();
                // A message is printed to the serial monitor indicating the first stuck recovery is done.
                Serial.println("[STUCK-1] Done.");
                // The flag is set to indicate that the first full-power recovery has been used.
                stuckFirstFired = true;
            } else {
                // A message is printed to the serial monitor indicating a subsequent soft recovery has started.
                Serial.println("[STUCK-N] Soft reverse...");
                // Both motors reverse at reduced power for a short duration.
                setMotors(-US_STUCK_REVERSE_PWM_2, -US_STUCK_REVERSE_PWM_2);
                // The robot continues reversing for US_STUCK_REVERSE_MS_2 milliseconds.
                delay(US_STUCK_REVERSE_MS_2);
                // Both motors are stopped after the soft reverse is complete.
                stopMotors();
                // A message is printed to the serial monitor indicating the soft recovery is done.
                Serial.println("[STUCK-N] Done.");
            }
            // The stuck timer is reset so it can detect a new stuck condition in the future.
            stuckTimerActive = false;
            // The reference reading is cleared so it will be set fresh on the next call.
            stuckFrontRef    = -1.0f;
            // The function returns true to indicate that a recovery sequence was executed.
            return true;
        }
    } else {
        // If the front reading has changed significantly, the robot is not stuck, so the timer is reset.
        stuckTimerActive = false;
        // The reference reading is updated to the current value for future comparisons.
        stuckFrontRef    = f;
    }
    // If no recovery sequence was triggered, the function returns false.
    return false;
}
/**
 * SETUP
 * The setup function runs once when the Arduino is powered on or reset.
 * It initializes serial communication for debugging, configures all motor and ultrasonic sensor pins,
 * stops the motors to ensure the robot does not move before it is ready, zeros out the SMA filter buffers,
 * prints the current configuration to the serial monitor, and waits three seconds before starting
 * to give the user time to place the robot in position.
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

    // Both SMA filter buffers are filled with zeros so that the averaging starts from a clean state.
    for (int i = 0; i < US_SMA_WINDOW; i++) {
        frontBuf[i] = 0; rightBuf[i] = 0;
    }

    // The program name is printed to the serial monitor to confirm the sketch is running.
    Serial.println("=== TEST 2: WALL FOLLOW ===");
    // The SMA filter window size is printed so the user can verify the current smoothing configuration.
    Serial.print("SMA window: ");
    Serial.print(US_SMA_WINDOW);
    Serial.println(" samples");
    // The desired wall distance and tolerance are printed so the user can verify the distance maintenance settings.
    Serial.print("Wall target: ");
    Serial.print(WF_RIGHT_NEAR_CM, 1);
    Serial.print("cm  +/- ");
    Serial.print(WF_RIGHT_TOLERANCE_CM, 1);
    // The wall-lost threshold is printed so the user can verify when the sensor considers the wall gone.
    Serial.print("cm  lost>");
    Serial.print(WF_RIGHT_LOST_CM, 1);
    Serial.println("cm");
    // A countdown message is printed to give the user time to place the robot in position.
    Serial.println("Starting in 3s...");
    // The program pauses for three seconds before the robot begins moving.
    delay(3000);
    // A final message is printed to indicate the robot is now active.
    Serial.println("GO!");
}

/**
 * MAIN LOOP
 * The loop function runs repeatedly after setup and contains the core wall-following logic.
 * Each iteration, it updates the front and right ultrasonic sensor readings by calling updateUltrasonics(),
 * determines whether the right wall is TOO CLOSE, TOO FAR, IN BAND, or LOST,
 * prints diagnostic information to the serial monitor, checks if the robot is stuck,
 * and then decides which motor action to take based on the state of the front and right sensors.
 * The decision tree follows a priority order: stuck recovery first, then turning decisions
 * based on whether the front is blocked and whether the right wall is visible, and finally
 * distance maintenance to keep the robot at the desired distance from the wall.
 */
void loop() {
    // The ultrasonic sensors are read and their filtered values are updated if enough time has passed.
    updateUltrasonics();
    // The current filtered front sensor distance is retrieved, or -1.0 if unavailable.
    float f = getFrontCm();
    // The current filtered right sensor distance is retrieved, or -1.0 if unavailable.
    float r = getRightCm();
    // The frontNear flag is true if the front sensor has a valid reading that is closer than WF_FRONT_NEAR_CM.
    bool frontNear     = frontValid() && f > 0 && f < WF_FRONT_NEAR_CM;
    /**
     * The rightVisible flag is true if the right sensor has a valid reading within WF_RIGHT_LOST_CM,
     * meaning a wall is detected.
     * */
    bool rightVisible  = rightValid() && r > 0 && r < WF_RIGHT_LOST_CM;
    // The target variable stores WF_RIGHT_NEAR_CM for use in the tooClose, tooFar, and inBand calculations below.
    float target   = WF_RIGHT_NEAR_CM;
    // The tooClose flag is true if the right wall is visible and closer than WF_RIGHT_NEAR_CM minus WF_RIGHT_TOLERANCE_CM.
    bool  tooClose = rightVisible && r < (target - WF_RIGHT_TOLERANCE_CM);
    // The tooFar flag is true if the right wall is visible and farther than WF_RIGHT_NEAR_CM plus WF_RIGHT_TOLERANCE_CM.
    bool  tooFar   = rightVisible && r > (target + WF_RIGHT_TOLERANCE_CM);
    // The inBand flag is true if the right wall is visible and the distance is between WF_RIGHT_NEAR_CM minus and plus WF_RIGHT_TOLERANCE_CM.
    bool  inBand   = rightVisible && !tooClose && !tooFar;
    // The front sensor status is printed to the serial monitor for debugging.
    Serial.print("[WALL] Front=");
    // If the front sensor has a valid reading, the distance and whether it is NEAR or FAR are printed.
    if (frontValid()) {
        Serial.print(f, 1);
        Serial.print("cm (");
        if (frontNear) {
            Serial.print("NEAR");
        } else {
            Serial.print("FAR");
        }
        Serial.print(")");
    } else {
        // If the front sensor has no valid reading, dashes are printed to indicate unavailable data.
        Serial.print("---");
    }
    // The right sensor status is printed to the serial monitor for debugging.
    Serial.print("  Right=");
    /**
     * If the right sensor has a valid reading,
     * the distance and whether it is TOO CLOSE, TOO FAR, IN BAND, or LOST are printed.
     * */
    if (rightValid()) {
        Serial.print(r, 1);
        Serial.print("cm (");
        if (!rightVisible) {
            // If the right sensor reading is beyond WF_RIGHT_LOST_CM, the wall is considered lost.
            Serial.print("LOST");
        } else if (tooClose) {
            // If the robot is closer to the wall than the lower bound of the acceptable range, TOO CLOSE is printed.
            Serial.print("TOO CLOSE");
        } else if (tooFar) {
            // If the robot is farther from the wall than the upper bound of the acceptable range, TOO FAR is printed.
            Serial.print("TOO FAR");
        } else {
            // If the robot is within the acceptable distance range, IN BAND is printed.
            Serial.print("IN BAND");
        }
        Serial.print(")");
    } else {
        // If the right sensor has no valid reading, dashes are printed to indicate unavailable data.
        Serial.print("---");
    }
    // The current stuck detection tier is printed so the user can see whether the first recovery is still available.
    if (stuckFirstFired) {
        Serial.print("  [STUCK-N]");
    } else {
        Serial.print("  [STUCK-1 armed]");
    }
    // An arrow is printed to separate the sensor status from the action that follows.
    Serial.print("  ->  ");
    // The stuck detection check runs first because it has the highest priority in the decision tree.
    if (checkStuck(f)) {
        // If a stuck recovery sequence was executed, the rest of the loop is skipped for this iteration.
        return;
    }
    // If the front is blocked and the right wall is visible, the robot is in a corner and pivots left.
    if      (frontNear && rightVisible) {
        setMotors(WF_TURN_SPEED, -WF_TURN_SPEED);
        Serial.println("TURN LEFT (corner)");
    }
        // If the front is blocked but the right wall is gone, the robot pivots right into the open space.
    else if (frontNear && !rightVisible) {
        setMotors(-WF_TURN_SPEED, WF_TURN_SPEED);
        Serial.println("TURN RIGHT (front blocked, wall gone)");
    }
        // If the front is clear and the right wall is visible, the robot performs distance maintenance.
    else if (!frontNear && rightVisible) {
        // If the robot is too close to the wall, it curves left to increase its distance.
        if (tooClose) {
            setMotors(WF_CORRECT_OUTER, WF_CORRECT_INNER);
            Serial.println("CORRECT LEFT (too close)");
            // If the robot is too far from the wall, it curves right to decrease its distance.
        } else if (tooFar) {
            setMotors(WF_CORRECT_INNER, WF_CORRECT_OUTER);
            Serial.println("CORRECT RIGHT (too far)");
            // If the robot is within the acceptable range, it drives straight.
        } else {
            setMotors(WF_STRAIGHT_SPEED, WF_STRAIGHT_SPEED);
            Serial.println("STRAIGHT (in band)");
        }
    }
        // If both the front is clear and the right wall is gone, the robot drives straight into open space.
    else {
        setMotors(WF_STRAIGHT_SPEED, WF_STRAIGHT_SPEED);
        Serial.println("STRAIGHT (open space, wall lost)");
    }
}