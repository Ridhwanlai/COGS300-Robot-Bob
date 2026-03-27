/**
  ============================================================
  COMBINED v2: LINE FOLLOW → WALL FOLLOW (one-way)
  The wall following logic is sourced from TEST 2.
  ============================================================

  PIN ASSIGNMENTS:
    The left IR sensor is connected to D9.
    The middle IR sensor is connected to D12.
    The right IR sensor is connected to D11.

    The front HC-SR04 ultrasonic sensor has its TRIG pin connected to A1 and its ECHO pin connected to A0.
    The right HC-SR04 ultrasonic sensor has its TRIG pin connected to A3 and its ECHO pin connected to A4.

    The ENA pin for left motor PWM speed control is connected to D5.
    The ENB pin for right motor PWM speed control is connected to D6.
    The IN1 pin for left motor forward direction is connected to D2.
    The IN2 pin for left motor backward direction is connected to D4.
    The IN3 pin for right motor forward direction is connected to D7.
    The IN4 pin for right motor backward direction is connected to D8.

  STATE MACHINE:
    The robot starts in MODE_LINE_FOLLOW by default.
    In this mode, the robot follows a tape line using three IR sensors and includes stuck detection.
    The ultrasonic sensors are polled every loop iteration but are not used for steering in this mode.
    The robot transitions to MODE_WALL_FOLLOW when all three IR sensors go off-tape.
    When that happens, the robot coasts straight for TRANSITION_COAST_MS to enter the corridor,
    then checks if the right ultrasonic sensor detects a wall closer than TRANSITION_RIGHT_WALL_CM.
    If a wall is detected, the robot switches to wall-follow mode permanently.
    If no wall is detected, normal line-follow recovery runs instead.

    In MODE_WALL_FOLLOW, the robot follows the right wall using ultrasonic sensors.
    The IR sensors are still read and printed to serial for monitoring, but they are not used for steering.
    There is no return to line-follow mode from wall-follow mode.
  ============================================================
*/

// ============================================================
// MOTOR PINS
// ============================================================

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

// ============================================================
// IR SENSOR PINS
// ============================================================

// The left IR sensor is connected to D9.
const int IR_LEFT   = 9;
// The middle IR sensor is connected to D12.
const int IR_MIDDLE = 12;
// The right IR sensor is connected to D11.
const int IR_RIGHT  = 11;

// When set to true, the reading from the left IR sensor is inverted so that HIGH means on-tape.
const bool IR_LEFT_INVERT   = true;
// When set to true, the reading from the middle IR sensor is inverted so that HIGH means on-tape.
const bool IR_MIDDLE_INVERT = true;
// When set to true, the reading from the right IR sensor is inverted so that HIGH means on-tape.
const bool IR_RIGHT_INVERT  = true;

// ============================================================
// ULTRASONIC PINS
// ============================================================

// The front ultrasonic sensor TRIG pin is connected to A1.
const int TRIG_FRONT_PIN = A1;
// The front ultrasonic sensor ECHO pin is connected to A0.
const int ECHO_FRONT_PIN = A0;
// The right ultrasonic sensor TRIG pin is connected to A3.
const int TRIG_RIGHT_PIN = A3;
// The right ultrasonic sensor ECHO pin is connected to A4.
const int ECHO_RIGHT_PIN = A4;

// ============================================================
// STATE MACHINE
// ============================================================

// The robot has two modes: MODE_LINE_FOLLOW and MODE_WALL_FOLLOW.
enum RobotMode { MODE_LINE_FOLLOW, MODE_WALL_FOLLOW };
// The robot starts in line-follow mode by default.
RobotMode currentMode = MODE_LINE_FOLLOW;

/**
 * TRANSITION CONSTANTS
 * When all three IR sensors go off-tape, the robot coasts straight for TRANSITION_COAST_MS milliseconds
 * so that it moves far enough into the corridor for the right ultrasonic sensor to detect a wall.
 * After coasting, if the right sensor detects a wall closer than TRANSITION_RIGHT_WALL_CM,
 * the robot switches to wall-follow mode permanently.
 * If no wall is detected, the robot resumes normal line-follow recovery instead.
 */
// This is how long in milliseconds the robot drives straight after losing the tape before checking for a wall.
const unsigned long TRANSITION_COAST_MS       = 300;
// If the right ultrasonic sensor reads below this value in centimeters after coasting, a corridor wall is confirmed.
const float         TRANSITION_RIGHT_WALL_CM  = 20.0f;

// ============================================================
// LINE FOLLOWING CONSTANTS
// ============================================================

// This is the PWM speed used for both wheels when the robot drives straight along the tape.
const int  LF_BASE_SPEED      = 110;
// This is the PWM speed used for the active wheel during a hard turn to stay on the tape.
const int  LF_TURN_SPEED_FAST = 100;
// This is the PWM speed for the outer (faster) wheel during a soft turn.
const int  LF_SOFT_OUTER      = 97;
// This is the PWM speed for the inner (slower) wheel during a soft turn.
const int  LF_SOFT_INNER      = 93;
// This value is added to the right motor speed to compensate for any drift toward the right.
const int  LF_RIGHT_TRIM      = 0;
// This value is added to the left motor speed to compensate for any drift toward the left.
const int  LF_LEFT_TRIM       = 0;
// When set to true, the IR sensor readings are checked multiple times to filter out noise.
const bool LF_USE_FILTER      = false;
// This is the number of additional times the IR sensors are read to confirm a stable reading.
const int  LF_FILTER_READS    = 2;
// This is the delay in milliseconds between each additional IR sensor read during filtering.
const int  LF_FILTER_DELAY    = 5;
// This is the PWM speed used when the robot reverses to find the tape again after losing it.
const int  LF_REVERSE_SPEED   = 90;

/**
 * RECOVERY CONSTANTS
 * When the robot loses the tape, it pauses briefly before reversing to avoid overcorrecting.
 * It then reverses in short pulses until any IR sensor sees the tape again.
 */
// This is how long in milliseconds the robot pauses before starting to reverse during recovery.
const unsigned long RECOVERY_STOP_MS     = 200;
// This is how long in milliseconds each reverse pulse lasts during recovery.
const unsigned long RECOVERY_REVERSE_MS  = 300;

// ============================================================
// WALL FOLLOWING CONSTANTS (from TEST 2)
// ============================================================

// The front sensor considers the path blocked when the distance is closer than this value in centimeters.
const float          WF_FRONT_NEAR_CM       = 11.0f;
// This is the desired distance from the right wall that the robot tries to maintain in centimeters.
const float          WF_RIGHT_NEAR_CM       = 8.0f;
// This is the acceptable range around WF_RIGHT_NEAR_CM where the robot drives straight without correcting, measured as plus or minus this value in centimeters.
const float          WF_RIGHT_TOLERANCE_CM  = 1.0f;
// This is the PWM speed used for both wheels when driving straight along the wall.
const int            WF_STRAIGHT_SPEED      = 110;
// This is the PWM speed used for hard pivot turns such as corners.
const int            WF_TURN_SPEED          = 120;
// This is the PWM speed for the faster wheel when the robot curves to adjust its distance from the wall.
const int            WF_CORRECT_OUTER       = 110;
// This is the PWM speed for the slower wheel when the robot curves to adjust its distance from the wall.
const int            WF_CORRECT_INNER       = 60;

// ============================================================
// WALL FOLLOW: STUCK DETECTION CONSTANTS (from TEST 2, two-tier)
// ============================================================

/**
 * STUCK DETECTION
 * The stuck detection system monitors the front sensor to determine if the robot is stuck during wall following.
 * If the front sensor reading stays within WF_STUCK_BAND_CM of a reference value for longer than
 * WF_STUCK_MS milliseconds, the robot is considered stuck.
 * The first time stuck is detected (tier 1), the robot reverses straight at full power then pushes forward
 * at full power to overcome obstacles like ramps.
 * Every subsequent time (tier 2+), the robot performs an angled reverse where the left wheel is stronger
 * than the right wheel, which angles the robot rightward during the reverse to help it clear corners.
 * The timer resets after each trigger so the robot can fire again if it gets stuck again.
 */
// The robot is considered stuck if the front sensor reading stays unchanged for this many milliseconds.
const unsigned long  WF_STUCK_MS              = 1000UL;
// The front sensor reading is considered unchanged if it varies by less than this many centimeters.
const float          WF_STUCK_BAND_CM         = 1.5f;

// This is the PWM speed used for reversing during the first stuck recovery attempt (tier 1).
const int            WF_STUCK_REVERSE_PWM_1   = 255;
// This is how long in milliseconds the robot reverses during the first stuck recovery attempt (tier 1).
const int            WF_STUCK_REVERSE_MS_1    = 500;
// This is the PWM speed used for the forward push during the first stuck recovery attempt (tier 1).
const int            WF_STUCK_PUSH_PWM_1      = 255;
// This is how long in milliseconds the robot pushes forward during the first stuck recovery attempt (tier 1).
const int            WF_STUCK_PUSH_MS_1       = 1500;

// This is the PWM speed for the left wheel during the angled reverse in subsequent stuck recovery attempts (tier 2+), which is stronger to angle the robot rightward.
const int            WF_STUCK_REVERSE_PWM_2_L = 200;
// This is the PWM speed for the right wheel during the angled reverse in subsequent stuck recovery attempts (tier 2+), which is weaker so the robot angles rightward.
const int            WF_STUCK_REVERSE_PWM_2_R = 120;
// This is how long in milliseconds the robot reverses during subsequent stuck recovery attempts (tier 2+).
const int            WF_STUCK_REVERSE_MS_2    = 250;

// ============================================================
// ULTRASONIC FILTER + TIMING
// ============================================================

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

// This is the maximum time in microseconds to wait for the ultrasonic sensor's ECHO pin to receive a reflected signal.
const unsigned long  US_PULSE_TIMEOUT    = 30000UL;
// Any ultrasonic reading below this value in centimeters is considered invalid and discarded
// because the sensor cannot reliably distinguish the outgoing sound burst from the returning echo at very short distances.
const float          US_MIN_CM           = 2.0f;
// Any ultrasonic reading above this value in centimeters is considered invalid and discarded
// because the reflected sound signal is too weak for the sensor to reliably detect at that distance.
const float          US_MAX_CM           = 400.0f;
// The last valid ultrasonic reading is held for this many milliseconds before being considered too old to reflect the robot's current position.
const unsigned long  US_HOLD_MS          = 200;
// This is the minimum time in milliseconds between consecutive ultrasonic sensor readings.
const unsigned long  US_PERIOD_MS        = 60;

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

// ============================================================
// WALL FOLLOW: STUCK DETECTION STATE
// ============================================================

// This stores the front sensor reference reading used to detect whether the robot is stuck.
float         wfStuckFrontRef    = -1.0f;
// This is the timestamp in milliseconds of when the front sensor reading first stopped changing.
unsigned long wfStuckSinceMs     = 0;
// This flag indicates whether the stuck detection timer is currently running.
bool          wfStuckTimerActive = false;
// This flag indicates whether the first full-power stuck recovery attempt (tier 1) has already been used.
bool          wfStuckFirstFired  = false;

// ============================================================
// MOTOR HELPERS
// ============================================================

/**
 * SET MOTORS
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

/**
 * STOP MOTORS
 * The stopMotors function cuts power to both motors and sets all direction pins to LOW
 * so that no current flows through either motor.
 */
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
// SENSOR HELPERS — IR
// ============================================================

/**
 * READ SENSORS
 * The readSensors function reads all three IR sensors and applies inversion if configured.
 * After this function returns, L, M, and R will be HIGH if the corresponding sensor sees the tape,
 * and LOW if it does not, regardless of whether the raw signal is inverted.
 */
// The L parameter receives the left IR sensor reading after inversion is applied.
// The M parameter receives the middle IR sensor reading after inversion is applied.
// The R parameter receives the right IR sensor reading after inversion is applied.
void readSensors(int &L, int &M, int &R) {
    // The left IR sensor is read and inverted if IR_LEFT_INVERT is true.
    if (IR_LEFT_INVERT) {
        L = !digitalRead(IR_LEFT);
    } else {
        L = digitalRead(IR_LEFT);
    }
    // The middle IR sensor is read and inverted if IR_MIDDLE_INVERT is true.
    if (IR_MIDDLE_INVERT) {
        M = !digitalRead(IR_MIDDLE);
    } else {
        M = digitalRead(IR_MIDDLE);
    }
    // The right IR sensor is read and inverted if IR_RIGHT_INVERT is true.
    if (IR_RIGHT_INVERT) {
        R = !digitalRead(IR_RIGHT);
    } else {
        R = digitalRead(IR_RIGHT);
    }
}

/**
 * ANY SENSOR ON TAPE
 * The anySensorOnTape function returns true if at least one of the three IR sensors currently sees the tape.
 * This is used during recovery to know when the robot has reversed far enough to find the tape again.
 */
bool anySensorOnTape() {
    int L, M, R;
    // All three IR sensors are read with inversion applied.
    readSensors(L, M, R);
    // The function returns true if any of the three sensors reads HIGH, meaning it sees the tape.
    return (L == HIGH || M == HIGH || R == HIGH);
}

// ============================================================
// SENSOR HELPERS — ULTRASONIC
// ============================================================

/**
 * SMA ADD
 * The smaAdd function adds a new sensor reading to a circular buffer used by the Simple Moving Average filter.
 * It stores the new value at the current write position in the buffer, replacing whatever was there before.
 * Once the buffer is full, new readings overwrite the oldest ones so the average always reflects the most recent data.
 */
// The buf parameter is a pointer to the circular buffer array that stores the most recent sensor readings.
// The idx parameter is a pointer to the current write index in the buffer, which tracks where the next reading should be stored.
// The count parameter is a pointer to the number of valid readings currently in the buffer, which is needed to know when the buffer is full.
// The val parameter is the new sensor reading to be added to the buffer.
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
 * SMA GET
 * The smaGet function returns the average of all valid readings in the buffer, or -1.0 if the buffer is empty.
 */
// The buf parameter is a pointer to the circular buffer array containing the sensor readings to be averaged.
// The count parameter is the number of valid readings currently stored in the buffer, which determines how many values to include in the average.
float smaGet(float* buf, int count) {
    // If there are no valid readings in the buffer, the function returns -1.0 to indicate that no average can be computed.
    if (count == 0) {
        return -1.0f;
    }
    // The sum variable is initialized to zero and will accumulate the total of all valid readings in the buffer.
    float sum = 0;
    // Each valid reading in the buffer is added to the running sum one at a time.
    for (int i = 0; i < count; i++) {
        sum += buf[i];
    }
    // The sum is divided by the number of valid readings to produce the average, which is returned as the filtered distance.
    return sum / count;
}

/**
 * READ ULTRASONIC CENTIMETERS
 * The readUsCm function triggers an ultrasonic sensor, measures the echo duration,
 * and converts it to a distance in centimeters.
 * If no echo is received or the distance is outside the valid range, it returns -1.0.
 */
// The trigPin parameter is the Arduino pin connected to the TRIG pin of the ultrasonic sensor to be read.
// The echoPin parameter is the Arduino pin connected to the ECHO pin of the ultrasonic sensor to be read.
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
 * The updateUltrasonics function reads both the front and right ultrasonic sensors
 * and updates their filtered distance values using the SMA filter.
 * It is called every loop iteration in both modes, but only performs a reading
 * if enough time has passed since the last one, as defined by US_PERIOD_MS.
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
 * FRONT VALID
 * The frontValid function returns true if the front sensor has been initialized
 * and the last valid reading was received within US_HOLD_MS milliseconds of the current time.
 */
bool frontValid() {
    return frontInit && (millis() - lastFrontOkMs <= US_HOLD_MS);
}

/**
 * RIGHT VALID
 * The rightValid function returns true if the right sensor has been initialized
 * and the last valid reading was received within US_HOLD_MS milliseconds of the current time.
 */
bool rightValid() {
    return rightInit && (millis() - lastRightOkMs <= US_HOLD_MS);
}

/**
 * GET FRONT CM
 * The getFrontCm function returns the filtered front distance if frontValid() returns true,
 * or -1.0 if the front sensor data is unavailable or too old to reflect the robot's current position.
 */
float getFrontCm() {
    if (frontValid()) {
        return frontCmFilt;
    } else {
        return -1.0f;
    }
}

/**
 * GET RIGHT CM
 * The getRightCm function returns the filtered right distance if rightValid() returns true,
 * or -1.0 if the right sensor data is unavailable or too old to reflect the robot's current position.
 */
float getRightCm() {
    if (rightValid()) {
        return rightCmFilt;
    } else {
        return -1.0f;
    }
}

// ============================================================
// TRANSITION CHECK
// ============================================================

/**
 * CHECK TRANSITION TO WALL FOLLOW
 * This function is called when all three IR sensors go off-tape.
 * It coasts the robot straight for TRANSITION_COAST_MS milliseconds so the robot enters the corridor,
 * then forces a fresh ultrasonic reading.
 * If the right sensor detects a wall closer than TRANSITION_RIGHT_WALL_CM, the robot switches to wall-follow mode.
 * If no wall is detected, the function returns false and normal line-follow recovery takes over.
 * The front sensor plays no role in the transition decision.
 */
bool checkTransitionToWallFollow() {
    // A message is printed to indicate the transition check has started.
    Serial.println("[TRANSITION] IR off — coasting straight...");
    // The robot drives straight at the line-follow base speed to coast into the corridor.
    setMotors(LF_BASE_SPEED + LF_LEFT_TRIM, LF_BASE_SPEED + LF_RIGHT_TRIM);
    // The robot coasts for TRANSITION_COAST_MS milliseconds to move far enough into the corridor.
    delay(TRANSITION_COAST_MS);
    // The robot stops after coasting so the ultrasonic reading is taken while stationary.
    stopMotors();

    // Two update calls are made so the SMA buffer gets at least one fresh sample after the coast.
    updateUltrasonics();
    // A short delay ensures enough time passes for the second ultrasonic reading cycle.
    delay(US_PERIOD_MS + 10);
    updateUltrasonics();

    // The right sensor distance is retrieved after the fresh readings.
    float r = getRightCm();
    // The right sensor value is printed to the serial monitor for debugging.
    Serial.print("[TRANSITION] Right sensor after coast: ");
    if (rightValid() && r > 0) {
        Serial.print(r, 1);
        Serial.println("cm");
    } else {
        Serial.println("no reading");
    }

    // If the right sensor detects a wall closer than TRANSITION_RIGHT_WALL_CM, wall-follow mode is activated.
    if (rightValid() && r > 0 && r < TRANSITION_RIGHT_WALL_CM) {
        Serial.println(">>> Wall on right confirmed — switching to WALL FOLLOW <<<");
        currentMode = MODE_WALL_FOLLOW;
        return true;
    }

    // If no wall is detected, the function returns false and normal line-follow recovery takes over.
    Serial.println("[TRANSITION] No wall detected — resuming LF recovery.");
    return false;
}

// ============================================================
// LINE FOLLOW ACTIONS
// ============================================================

// The robot drives straight at the base speed with any left and right trim applied.
void lf_goStraight()    { setMotors(LF_BASE_SPEED + LF_LEFT_TRIM, LF_BASE_SPEED + LF_RIGHT_TRIM); }
// The robot turns hard right by running only the left wheel at the fast turn speed.
void lf_hardTurnRight() { setMotors(LF_TURN_SPEED_FAST, 0); }
// The robot turns hard left by running only the right wheel at the fast turn speed.
void lf_hardTurnLeft()  { setMotors(0, LF_TURN_SPEED_FAST); }
// The robot turns soft right by running the left wheel faster than the right wheel, with a short delay for the turn to take effect.
void lf_softTurnRight() { setMotors(LF_SOFT_OUTER, LF_SOFT_INNER); delay(40); }
// The robot turns soft left by running the right wheel faster than the left wheel, with a short delay for the turn to take effect.
void lf_softTurnLeft()  { setMotors(LF_SOFT_INNER, LF_SOFT_OUTER); delay(40); }

// ============================================================
// LINE FOLLOW: RECOVERY
// ============================================================

/**
 * RECOVER FROM LOST TAPE
 * This function is called when all three IR sensors go off-tape.
 * It first checks if the robot should transition to wall-follow mode by calling checkTransitionToWallFollow().
 * If a wall is detected, the function returns immediately without reversing.
 * If no wall is detected, the robot stops, pauses for RECOVERY_STOP_MS milliseconds,
 * then reverses in short pulses until any IR sensor sees the tape again.
 * The robot does not spin during recovery — it stays oriented and just backs up.
 */
void recoverFromLostTape() {
    // The transition check runs first. If a wall is found, the mode switches and the function returns immediately.
    if (checkTransitionToWallFollow()) {
        return;
    }

    // A message is printed to indicate the robot has lost the tape and is starting recovery.
    Serial.println("All sensors OFF tape — reversing until tape found...");
    // The robot stops before starting to reverse.
    stopMotors();
    // The robot pauses for RECOVERY_STOP_MS milliseconds before reversing to avoid overcorrecting.
    delay(RECOVERY_STOP_MS);

    // The robot reverses in short pulses until any IR sensor detects the tape again.
    while (!anySensorOnTape()) {
        // The robot reverses at LF_REVERSE_SPEED for RECOVERY_REVERSE_MS milliseconds.
        setMotors(-LF_REVERSE_SPEED, -LF_REVERSE_SPEED);
        delay(RECOVERY_REVERSE_MS);
        // The robot pauses briefly between reverse pulses to allow the sensors to settle.
        stopMotors();
        delay(50);
        Serial.println("  Still off tape — reversing again...");
    }

    Serial.println("  Tape found. Resuming.");
}

// ============================================================
// WALL FOLLOW: STUCK DETECTION (two-tier, fires repeatedly)
// ============================================================

/**
 * CHECK WALL FOLLOW STUCK
 * The checkWfStuck function monitors the front sensor to determine if the robot is stuck during wall following.
 * If the front reading stays within WF_STUCK_BAND_CM of a reference value for longer than WF_STUCK_MS milliseconds,
 * a recovery sequence is triggered.
 * The first time (tier 1), both motors reverse straight at full power then push forward at full power.
 * Every subsequent time (tier 2+), the robot performs an angled reverse where the left wheel is stronger
 * than the right wheel, which angles the robot rightward during the reverse.
 * After each trigger, the timer resets so it can fire again if the robot gets stuck again.
 * The f parameter is the current filtered front sensor distance in centimeters.
 */
bool checkWfStuck(float f) {
    // The current time in milliseconds is captured for use in timing comparisons.
    unsigned long now = millis();

    // If the front sensor data is invalid or unavailable, stuck detection is paused and the timer is reset.
    if (!frontValid() || f <= 0) {
        wfStuckTimerActive = false;
        return false;
    }

    // The current front reading is compared to the stored reference to determine if it has changed by less than WF_STUCK_BAND_CM.
    bool fStuck = wfStuckFrontRef >= 0 && abs(f - wfStuckFrontRef) < WF_STUCK_BAND_CM;

    // If the front reading has not changed significantly, the robot may be stuck.
    if (fStuck) {
        // If the stuck timer is not already running, it is started and the current time is recorded.
        if (!wfStuckTimerActive) {
            wfStuckTimerActive = true;
            wfStuckSinceMs     = now;
        // If the stuck timer has been running for longer than WF_STUCK_MS, a recovery sequence is triggered.
        } else if (now - wfStuckSinceMs > WF_STUCK_MS) {
            // If this is the first time the robot has been stuck, the full-power tier 1 recovery is used.
            if (!wfStuckFirstFired) {
                // A message is printed to the serial monitor indicating the tier 1 stuck recovery has started.
                Serial.println("[STUCK-1] First stuck! Reversing then full power push...");
                // Both motors reverse straight at full power to back the robot away from the obstacle.
                setMotors(-WF_STUCK_REVERSE_PWM_1, -WF_STUCK_REVERSE_PWM_1);
                // The robot continues reversing for WF_STUCK_REVERSE_MS_1 milliseconds.
                delay(WF_STUCK_REVERSE_MS_1);
                // Both motors push forward at full power to drive the robot over or past the obstacle.
                setMotors(WF_STUCK_PUSH_PWM_1, WF_STUCK_PUSH_PWM_1);
                // The robot continues pushing forward for WF_STUCK_PUSH_MS_1 milliseconds.
                delay(WF_STUCK_PUSH_MS_1);
                // Both motors are stopped after the recovery sequence is complete.
                stopMotors();
                // A message is printed to the serial monitor indicating the tier 1 recovery is done.
                Serial.println("[STUCK-1] Done.");
                // The flag is set to indicate that tier 1 has been used, so future triggers use tier 2.
                wfStuckFirstFired = true;
            } else {
                // A message is printed to the serial monitor indicating a tier 2 angled reverse has started.
                Serial.println("[STUCK-N] Angled reverse right...");
                // The left wheel reverses at WF_STUCK_REVERSE_PWM_2_R and the right wheel reverses at WF_STUCK_REVERSE_PWM_2_L,
                // which makes the robot angle rightward during the reverse because the right wheel is stronger.
                setMotors(-WF_STUCK_REVERSE_PWM_2_R, -WF_STUCK_REVERSE_PWM_2_L);
                // The robot continues the angled reverse for WF_STUCK_REVERSE_MS_2 milliseconds.
                delay(WF_STUCK_REVERSE_MS_2);
                // Both motors are stopped after the angled reverse is complete.
                stopMotors();
                // A message is printed to the serial monitor indicating the tier 2 recovery is done.
                Serial.println("[STUCK-N] Done.");
            }
            // The stuck timer is reset so it can detect a new stuck condition in the future.
            wfStuckTimerActive = false;
            // The reference reading is cleared so it will be set fresh on the next call.
            wfStuckFrontRef    = -1.0f;
            // The function returns true to indicate that a recovery sequence was executed.
            return true;
        }
    } else {
        // If the front reading has changed significantly, the robot is not stuck, so the timer is reset.
        wfStuckTimerActive = false;
        // The reference reading is updated to the current value for future comparisons.
        wfStuckFrontRef    = f;
    }

    // If no recovery sequence was triggered, the function returns false.
    return false;
}

// ============================================================
// SETUP
// ============================================================

/**
 * SETUP
 * The setup function runs once when the Arduino is powered on or reset.
 * It initializes serial communication for debugging, configures all motor, IR sensor,
 * and ultrasonic sensor pins, stops the motors, zeros out the SMA filter buffers,
 * prints the configuration and transition settings to the serial monitor,
 * and waits three seconds before starting.
 */
void setup() {
    // Serial communication is started at 9600 baud for printing debug messages to the serial monitor.
    Serial.begin(9600);

    // The left motor forward and backward direction pins are configured as outputs.
    pinMode(PIN_LF, OUTPUT);
    pinMode(PIN_LB, OUTPUT);
    // The right motor forward and backward direction pins are configured as outputs.
    pinMode(PIN_RF, OUTPUT);
    pinMode(PIN_RB, OUTPUT);
    // The left and right motor PWM speed control pins are configured as outputs.
    pinMode(L_PWM_PIN, OUTPUT);
    pinMode(R_PWM_PIN, OUTPUT);
    // The motors are stopped immediately to ensure the robot does not move during initialization.
    stopMotors();

    // The three IR sensor pins are configured as inputs.
    pinMode(IR_LEFT,   INPUT);
    pinMode(IR_MIDDLE, INPUT);
    pinMode(IR_RIGHT,  INPUT);

    // The front ultrasonic sensor TRIG pin is configured as an output and the ECHO pin is configured as an input.
    pinMode(TRIG_FRONT_PIN, OUTPUT);
    pinMode(ECHO_FRONT_PIN, INPUT);
    // The right ultrasonic sensor TRIG pin is configured as an output and the ECHO pin is configured as an input.
    pinMode(TRIG_RIGHT_PIN, OUTPUT);
    pinMode(ECHO_RIGHT_PIN, INPUT);
    // The front TRIG pin is set LOW to ensure it is not sending a signal before the first reading.
    digitalWrite(TRIG_FRONT_PIN, LOW);
    // The right TRIG pin is set LOW to ensure it is not sending a signal before the first reading.
    digitalWrite(TRIG_RIGHT_PIN, LOW);

    // Both SMA filter buffers are filled with zeros so that the averaging starts from a clean state.
    for (int i = 0; i < US_SMA_WINDOW; i++) {
        frontBuf[i] = 0;
        rightBuf[i] = 0;
    }

    // The program name and transition settings are printed to the serial monitor.
    Serial.println("=== COMBINED v2: LINE FOLLOW -> WALL FOLLOW (TEST 2 WF) ===");
    Serial.println("Transition: IR off -> coast 300ms straight -> right wall < 20cm = wall follow");
    // The wall-follow distance settings are printed so the user can verify the configuration.
    Serial.print("WF target: ");
    Serial.print(WF_RIGHT_NEAR_CM, 1);
    Serial.print("cm  +/- ");
    Serial.print(WF_RIGHT_TOLERANCE_CM, 1);
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
 * The ultrasonic sensors are polled every iteration regardless of which mode the robot is in.
 * In MODE_LINE_FOLLOW, the robot reads the three IR sensors and steers to follow the tape.
 * If all three IR sensors go off-tape, the robot attempts to transition to wall-follow mode.
 * In MODE_WALL_FOLLOW, the robot uses the front and right ultrasonic sensors to follow the right wall.
 * The decision tree checks for stuck recovery first, then determines whether to turn, correct, or drive straight
 * based on whether the front is blocked and whether the right wall is visible.
 */
void loop() {
    // The ultrasonic sensors are updated every loop iteration regardless of the current mode.
    updateUltrasonics();

    // ---- MODE: LINE FOLLOW ----
    if (currentMode == MODE_LINE_FOLLOW) {
        int L, M, R;
        // All three IR sensors are read with inversion applied.
        readSensors(L, M, R);

        // The IR sensor values are printed to the serial monitor for debugging.
        Serial.print("[LF] L=");
        Serial.print(L);
        Serial.print(" M=");
        Serial.print(M);
        Serial.print(" R=");
        Serial.print(R);

        // The front ultrasonic distance is printed for passive monitoring during line following.
        float f = getFrontCm();
        Serial.print("  Front=");
        if (frontValid()) {
            Serial.print(f, 1);
            Serial.print("cm");
        } else {
            Serial.print("---");
        }
        // The right ultrasonic distance is printed for passive monitoring during line following.
        float r_diag = getRightCm();
        Serial.print("  Right=");
        if (rightValid()) {
            Serial.print(r_diag, 1);
            Serial.print("cm");
        } else {
            Serial.print("---");
        }
        Serial.print(" -> ");

        // If filtering is enabled, the IR sensors are re-read multiple times to confirm a stable reading.
        if (LF_USE_FILTER) {
            for (int i = 0; i < LF_FILTER_READS; i++) {
                // A short delay is added between each re-read to let the sensors settle.
                delay(LF_FILTER_DELAY);
                int nL, nM, nR;
                readSensors(nL, nM, nR);
                // If any sensor reading changed between reads, the reading is considered unstable and the loop iteration is skipped.
                if (nL != L || nM != M || nR != R) {
                    Serial.println("UNSTABLE");
                    return;
                }
            }
        }

        // The robot decides which action to take based on the combination of IR sensor readings.
        // If all three sensors see the tape, the robot drives straight through the junction.
        if      (L==HIGH && M==HIGH && R==HIGH) { lf_goStraight();    Serial.println("Junction"); }
        // If the left and middle sensors see the tape but the right does not, the robot turns soft left.
        else if (L==HIGH && M==HIGH && R==LOW)  { lf_softTurnLeft();  Serial.println("Soft L");   }
        // If the left and right sensors see the tape but the middle does not, the robot drives straight through the cross.
        else if (L==HIGH && M==LOW  && R==HIGH) { lf_goStraight();    Serial.println("Cross");    }
        // If only the left sensor sees the tape, the robot turns hard left.
        else if (L==HIGH && M==LOW  && R==LOW)  { lf_hardTurnLeft();  Serial.println("Hard L");   }
        // If the middle and right sensors see the tape but the left does not, the robot turns soft right.
        else if (L==LOW  && M==HIGH && R==HIGH) { lf_softTurnRight(); Serial.println("Soft R");   }
        // If only the middle sensor sees the tape, the robot drives straight.
        else if (L==LOW  && M==HIGH && R==LOW)  { lf_goStraight();    Serial.println("Straight"); }
        // If only the right sensor sees the tape, the robot turns hard right.
        else if (L==LOW  && M==LOW  && R==HIGH) { lf_hardTurnRight(); Serial.println("Hard R");   }
        // If no sensors see the tape, the robot has lost the line and recovery is attempted.
        else if (L==LOW  && M==LOW  && R==LOW)  {
            Serial.println("OFF TAPE");
            // The recovery function checks for a wall-follow transition first, then reverses if needed.
            recoverFromLostTape();
        }

        // The line-follow iteration is complete, so the function returns early.
        return;
    }

    // ---- MODE: WALL FOLLOW (TEST 2 logic) ----
    // The current filtered front sensor distance is retrieved.
    float f = getFrontCm();
    // The current filtered right sensor distance is retrieved.
    float r = getRightCm();

    // The rightVisible flag is true if the right sensor has any valid reading, meaning a wall is present.
    bool rightVisible = rightValid() && r > 0;
    // The frontNear flag is true if the front sensor has a valid reading that is closer than WF_FRONT_NEAR_CM.
    bool frontNear    = frontValid() && f > 0 && f < WF_FRONT_NEAR_CM;

    // The target variable stores WF_RIGHT_NEAR_CM for use in the tooClose, tooFar, and inBand calculations below.
    float target   = WF_RIGHT_NEAR_CM;
    // The tooClose flag is true if the right wall is visible and closer than WF_RIGHT_NEAR_CM minus WF_RIGHT_TOLERANCE_CM.
    bool  tooClose = rightVisible && r < (target - WF_RIGHT_TOLERANCE_CM);
    // The tooFar flag is true if the right wall is visible and farther than WF_RIGHT_NEAR_CM plus WF_RIGHT_TOLERANCE_CM.
    bool  tooFar   = rightVisible && r > (target + WF_RIGHT_TOLERANCE_CM);
    // The inBand flag is true if the right wall is visible and the distance is between WF_RIGHT_NEAR_CM minus and plus WF_RIGHT_TOLERANCE_CM.
    bool  inBand   = rightVisible && !tooClose && !tooFar;

    // The front sensor status is printed to the serial monitor for debugging.
    Serial.print("[WF] Front=");
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
        Serial.print("---");
    }
    // The right sensor status is printed to the serial monitor for debugging.
    Serial.print("  Right=");
    if (rightValid()) {
        Serial.print(r, 1);
        Serial.print("cm (");
        if (!rightVisible) {
            Serial.print("NO READING");
        } else if (tooClose) {
            Serial.print("TOO CLOSE");
        } else if (tooFar) {
            Serial.print("TOO FAR");
        } else {
            Serial.print("IN BAND");
        }
        Serial.print(")");
    } else {
        Serial.print("---");
    }
    // The stuck detection tier is printed so the user can see which recovery level is armed.
    if (wfStuckFirstFired) {
        Serial.print("  [STUCK-N]");
    } else {
        Serial.print("  [STUCK-1 armed]");
    }
    Serial.print("  ->  ");

    // The stuck detection check runs first because it has the highest priority in the wall-follow decision tree.
    if (checkWfStuck(f)) {
        return;
    }

    // If the front is blocked and the right wall is visible, the robot is in a corner and pivots left.
    if (frontNear && rightVisible) {
        setMotors(WF_TURN_SPEED, -WF_TURN_SPEED);
        Serial.println("TURN LEFT (corner)");
    }
    // If the front is blocked but the right wall is not visible, the robot pivots right into the open space.
    else if (frontNear && !rightVisible) {
        setMotors(-WF_TURN_SPEED, WF_TURN_SPEED);
        Serial.println("TURN RIGHT (front blocked, no wall)");
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
    // If both the front is clear and the right wall is not visible, the robot drives straight.
    else {
        setMotors(WF_STRAIGHT_SPEED, WF_STRAIGHT_SPEED);
        Serial.println("STRAIGHT (no right reading)");
    }
}
