#include <Arduino.h>
/*
  ============================================================
  TEST 3: OBJECT HUNT — Servo Sweep + Wall Follow (Maze)
  v2: Distance-scaled detection — works at any range
  ============================================================

  PIN ASSIGNMENTS:
    Front HC-SR04 (on servo): TRIG -> A1,  ECHO -> A0
    Right HC-SR04 (fixed):    TRIG -> A3,  ECHO -> A4
    Servo (pan left/right):   Signal -> A2

    ENA (Left  PWM) -> D5
    ENB (Right PWM) -> D6
    IN1 (Left  Fwd) -> D2
    IN2 (Left  Bck) -> D4
    IN3 (Right Fwd) -> D7
    IN4 (Right Bck) -> D8

  DISTANCE-SCALED DETECTION:
    The core problem with fixed thresholds is geometry.
    An object that is 8cm wide appears as approximately 15 degrees when it is 30cm away,
    but the same 8cm object appears as approximately 30 degrees when it is only 15cm away.
    A fixed OBJ_MAX_WIDTH_DEG threshold that works at 30cm will reject the same object at 15cm
    because it looks as wide as a wall at close range.

    The fix is to scale the allowed angular width inversely with distance.
    At the reference distance OBJ_REF_DIST_CM, the threshold is OBJ_MAX_WIDTH_DEG.
    At half the reference distance, the threshold doubles.
    At double the reference distance, the threshold halves.
    The formula is: scaledWidth = OBJ_MAX_WIDTH_DEG * (OBJ_REF_DIST_CM / measuredDistance).

    The same principle applies to OBJ_WALL_MARGIN_CM.
    At close range, the absolute difference between an object and its neighbors shrinks
    because even the background walls are relatively close.
    The formula is: scaledMargin = OBJ_WALL_MARGIN_CM * (measuredDistance / OBJ_REF_DIST_CM).
    This makes the margin requirement more lenient at close range and stricter at long range.

  CALIBRATION MODE:
    Set CALIBRATION_MODE to true in the constants below.
    The robot will not move. It will repeatedly sweep, print every reading,
    and print the computed width, margin, and scaled thresholds for each candidate.
    Place a known object at different distances and note the values.
    Use those values to tune OBJ_MAX_WIDTH_DEG, OBJ_WALL_MARGIN_CM, and OBJ_REF_DIST_CM.
    Set CALIBRATION_MODE back to false when done.

  BEHAVIOUR OVERVIEW:
    [SWEEP]
      The front servo pans across SWEEP_MIN_DEG to SWEEP_MAX_DEG in
      steps of SWEEP_STEP_DEG, taking a distance reading at each angle.
      Readings are stored in a small array (the depth profile).
      After the sweep, the profile is scanned for an object signature:
        A local minimum that is closer than OBJ_MAX_CM,
        narrower than the distance-scaled width threshold,
        and stands out from its neighbors by more than the distance-scaled margin.
      A wide close return is a wall. A narrow close return is an object.
      If an object is found, the robot steers toward it (CHARGE mode).
      Otherwise it does a short wall-follow burst then sweeps again.

    [WALL FOLLOW]
      Standard right-wall follow at wider clearances suited to an
      open or irregular maze.
      Front sensor is read at the servo's centre position (90 degrees).
      Right sensor is the fixed HC-SR04.
      Wall follow runs for WF_BURST_MS milliseconds between sweeps.

    [CHARGE]
      Servo centres, robot drives straight at CHARGE_SPEED.
      Runs until front sensor reads below CHARGE_IMPACT_CM (hit)
      or CHARGE_TIMEOUT_MS elapses (lost the object, resume wall follow).
      On impact: stop briefly, log the hit, reverse briefly, resume.

    [DEAD END]
      Detected when front sensor is near and right sensor has no wall.
      Action: pivot 180 degrees in place, resume.

  TUNING NOTES:
    OBJ_REF_DIST_CM     — the distance at which you tune OBJ_MAX_WIDTH_DEG.
                           Set this to where detection currently works well.
                           All scaling is relative to this distance.
    OBJ_MAX_WIDTH_DEG   — max angular span at OBJ_REF_DIST_CM.
                           Automatically widens for closer objects, narrows for farther ones.
    OBJ_WALL_MARGIN_CM  — contrast at OBJ_REF_DIST_CM.
                           Automatically relaxes at close range, tightens at long range.
    OBJ_MAX_CM          — absolute max range. Objects beyond this are ignored entirely.
    SWEEP_SETTLE_MS     — wait after each servo step. Too short and the servo is still vibrating.
                           80ms is a safe starting point; try 60ms if sweeps feel too slow.
  ============================================================
*/

#include <Servo.h>

// ============================================================
// CALIBRATION MODE
// ============================================================

// Set this to true to disable driving and only sweep with detailed serial output.
// The robot will print the raw depth profile, every candidate's measured width and margin,
// and the distance-scaled thresholds it would be compared against.
// Set back to false for normal operation.
const bool CALIBRATION_MODE = false;

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
// SERVO PIN + OBJECT
// ============================================================

// The servo signal pin is connected to A2.
const int SERVO_PIN        = A2;
// This is the servo angle in degrees that points straight ahead.
const int SERVO_CENTER_DEG = 90;
// This is the Servo object that controls the servo motor used to sweep the ultrasonic sensor.
Servo panServo;

// ============================================================
// SERVO SWEEP CONSTANTS
// ============================================================

// This is the leftmost servo angle in degrees during a sweep.
const int   SWEEP_MIN_DEG      = 40;
// This is the rightmost servo angle in degrees during a sweep.
const int   SWEEP_MAX_DEG      = 140;
// This is the angular step size in degrees between consecutive sweep positions.
const int   SWEEP_STEP_DEG     = 5;
// This is how long in milliseconds the code waits after moving the servo to let it settle before taking a reading.
const int   SWEEP_SETTLE_MS    = 120;

// This is the total number of measurement positions in one sweep, computed from the arc and step size.
const int   SWEEP_STEPS = (SWEEP_MAX_DEG - SWEEP_MIN_DEG) / SWEEP_STEP_DEG + 1;

// ============================================================
// OBJECT DETECTION CONSTANTS (distance-scaled)
// ============================================================

/**
 * DISTANCE-SCALED OBJECT DETECTION
 * The width and margin thresholds are defined at a reference distance and then scaled
 * based on the actual measured distance of each candidate.
 * OBJ_REF_DIST_CM is the distance at which OBJ_MAX_WIDTH_DEG and OBJ_WALL_MARGIN_CM
 * are tuned to work correctly. Set it to where detection currently works best.
 * At closer distances the width threshold increases and the margin threshold decreases.
 * At farther distances the width threshold decreases and the margin threshold increases.
 */
// This is the reference distance in centimeters at which the width and margin thresholds are tuned.
const float OBJ_REF_DIST_CM    = 25.0f;
// Any reading below this distance in centimeters is considered a candidate for object detection.
const float OBJ_MAX_CM         = 35.0f;
// At the reference distance, an object candidate must span fewer than this many degrees to be classified as an object rather than a wall.
const int   OBJ_MAX_WIDTH_DEG  = 20;
// At the reference distance, an object candidate must be this many centimeters closer than at least one of its neighbors to count as a protrusion rather than a flat wall.
const float OBJ_WALL_MARGIN_CM = 8.0f;
// This is the minimum allowed scaled margin in centimeters, which prevents the margin from shrinking too small at very close range.
const float OBJ_MIN_MARGIN_CM  = 3.0f;
// This is the maximum allowed scaled width in degrees, which prevents the width threshold from growing absurdly large at very close range.
const int   OBJ_MAX_SCALED_WIDTH_DEG = 60;

// ============================================================
// CHARGE (ram object) CONSTANTS
// ============================================================

// The robot considers itself to have hit the object when the front distance drops below this value in centimeters.
const float          CHARGE_IMPACT_CM   = 8.0f;
// If the robot has not hit the object within this many milliseconds, it gives up and resumes wall following.
const unsigned long  CHARGE_TIMEOUT_MS  = 2500UL;
// This is the PWM speed used when driving straight toward the detected object.
const int            CHARGE_SPEED       = 200;
// This is how long in milliseconds the robot pauses after hitting an object.
const unsigned long  CHARGE_STOP_MS     = 300;

// ============================================================
// WALL FOLLOWING CONSTANTS (wider than corridor version)
// ============================================================

// The front sensor considers the path blocked when the distance is closer than this value in centimeters.
const float          WF_FRONT_NEAR_CM      = 25.0f;
// This is the desired distance from the right wall that the robot tries to maintain in centimeters.
const float          WF_RIGHT_NEAR_CM      = 15.0f;
// This is the acceptable range around WF_RIGHT_NEAR_CM where the robot drives straight without correcting, measured as plus or minus this value in centimeters.
const float          WF_RIGHT_TOLERANCE_CM = 3.0f;
// This is the PWM speed used for both wheels when driving straight along the wall.
const int            WF_STRAIGHT_SPEED     = 110;
// This is the PWM speed used for hard pivot turns such as corners.
const int            WF_TURN_SPEED         = 120;
// This is the PWM speed for the faster wheel when the robot curves to adjust its distance from the wall.
const int            WF_CORRECT_OUTER      = 110;
// This is the PWM speed for the slower wheel when the robot curves to adjust its distance from the wall.
const int            WF_CORRECT_INNER      = 60;
// This is how long in milliseconds each wall-follow burst lasts between sweeps.
const unsigned long  WF_BURST_MS           = 600UL;

// ============================================================
// STUCK DETECTION CONSTANTS (two-tier, same as TEST 2)
// ============================================================

/**
 * STUCK DETECTION
 * The stuck detection system monitors the front sensor to determine if the robot is stuck during wall following.
 * If the front sensor reading stays within WF_STUCK_BAND_CM of a reference value for longer than
 * WF_STUCK_MS milliseconds, the robot is considered stuck.
 * The first time stuck is detected (tier 1), the robot reverses straight at full power then pushes forward.
 * Every subsequent time (tier 2+), the robot performs an angled reverse where the left wheel is stronger
 * than the right wheel, which angles the robot rightward during the reverse.
 */
// The robot is considered stuck if the front sensor reading stays unchanged for this many milliseconds.
const unsigned long  WF_STUCK_MS              = 1500UL;
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
// This is the PWM speed for the left wheel during the angled reverse in subsequent stuck recovery attempts (tier 2+).
const int            WF_STUCK_REVERSE_PWM_2_L = 200;
// This is the PWM speed for the right wheel during the angled reverse in subsequent stuck recovery attempts (tier 2+).
const int            WF_STUCK_REVERSE_PWM_2_R = 120;
// This is how long in milliseconds the robot reverses during subsequent stuck recovery attempts (tier 2+).
const int            WF_STUCK_REVERSE_MS_2    = 300;

// ============================================================
// ULTRASONIC FILTER + TIMING
// ============================================================

/**
 * SMA FILTER
 * SMA stands for Simple Moving Average.
 * It is a smoothing filter that averages the last few ultrasonic sensor readings to reduce noise and jitter.
 * US_SMA_WINDOW defines how many recent readings are stored and averaged.
 */
#define US_SMA_WINDOW 4

// This is the maximum time in microseconds to wait for the ultrasonic sensor's ECHO pin to receive a reflected signal.
const unsigned long  US_PULSE_TIMEOUT = 30000UL;
// Any ultrasonic reading below this value in centimeters is considered invalid.
const float          US_MIN_CM        = 2.0f;
// Any ultrasonic reading above this value in centimeters is considered invalid.
const float          US_MAX_CM        = 400.0f;
// The last valid ultrasonic reading is held for this many milliseconds before being considered too old to reflect the robot's current position.
const unsigned long  US_HOLD_MS       = 200;
// This is the minimum time in milliseconds between consecutive ultrasonic sensor readings.
const unsigned long  US_PERIOD_MS     = 60;

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
// STUCK DETECTION STATE
// ============================================================

// This stores the front sensor reference reading used to detect whether the robot is stuck.
float         stuckFrontRef    = -1.0f;
// This is the timestamp in milliseconds of when the front sensor reading first stopped changing.
unsigned long stuckSinceMs     = 0;
// This flag indicates whether the stuck detection timer is currently running.
bool          stuckTimerActive = false;
// This flag indicates whether the first full-power stuck recovery attempt (tier 1) has already been used.
bool          stuckFirstFired  = false;

// ============================================================
// OBJECT COUNTER
// ============================================================

// This variable tracks how many objects the robot has successfully hit so far.
int objectsHit = 0;

// ============================================================
// SWEEP PROFILE STORAGE
// ============================================================

// This array stores the distance reading in centimeters for each sweep position.
// The maximum number of steps is (130-50)/5+1 = 17, and the array is padded to 20.
float sweepProfile[20];

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
    // The left motor PWM duty cycle is set to 0%, which cuts power to the left motor.
    analogWrite(L_PWM_PIN, 0);
    // The right motor PWM duty cycle is set to 0%, which cuts power to the right motor.
    analogWrite(R_PWM_PIN, 0);
    // All direction pins are set LOW to prevent any unintended current flow.
    digitalWrite(PIN_LF, LOW);
    digitalWrite(PIN_LB, LOW);
    digitalWrite(PIN_RF, LOW);
    digitalWrite(PIN_RB, LOW);
}

// ============================================================
// ULTRASONIC HELPERS
// ============================================================

/**
 * SMA ADD
 * The smaAdd function adds a new sensor reading to a circular buffer used by the Simple Moving Average filter.
 */
void smaAdd(float* buf, int* idx, int* count, float val) {
    // The new reading is stored in the buffer at the current write index.
    buf[*idx] = val;
    // The write index is advanced by one and wraps back to zero when it reaches the end of the buffer.
    *idx = (*idx + 1) % US_SMA_WINDOW;
    // If the buffer has not yet been completely filled, the count of valid readings is incremented.
    if (*count < US_SMA_WINDOW) {
        (*count)++;
    }
}

/**
 * SMA GET
 * The smaGet function returns the average of all valid readings in the buffer, or -1.0 if the buffer is empty.
 */
float smaGet(float* buf, int count) {
    // If there are no valid readings, the function returns -1.0.
    if (count == 0) {
        return -1.0f;
    }
    // All valid readings are summed and divided by the count to produce the average.
    float sum = 0;
    for (int i = 0; i < count; i++) {
        sum += buf[i];
    }
    return sum / count;
}

/**
 * READ ULTRASONIC CENTIMETERS
 * The readUsCm function triggers an ultrasonic sensor, measures the echo duration,
 * and converts it to a distance in centimeters.
 * If no echo is received or the distance is outside the valid range, it returns -1.0.
 */
float readUsCm(int trigPin, int echoPin) {
    // The TRIG pin is set LOW to ensure a clean start before sending the trigger pulse.
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // The TRIG pin is set HIGH for 10 microseconds to trigger the ultrasonic burst.
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    // The TRIG pin is set back to LOW to end the trigger pulse.
    digitalWrite(trigPin, LOW);
    // The echo duration is measured in microseconds.
    unsigned long dur = pulseIn(echoPin, HIGH, US_PULSE_TIMEOUT);
    // If no echo was received, the reading is invalid.
    if (dur == 0) {
        return -1.0f;
    }
    // The duration is converted to centimeters using the speed of sound.
    float cm = (dur * 0.0343f) / 2.0f;
    // If the distance is outside the valid range, the reading is discarded.
    if (cm < US_MIN_CM || cm > US_MAX_CM) {
        return -1.0f;
    }
    return cm;
}

/**
 * UPDATE ULTRASONICS
 * The updateUltrasonics function reads both the front and right ultrasonic sensors
 * and updates their filtered distance values using the SMA filter.
 * It only performs a reading if enough time has passed since the last one.
 */
void updateUltrasonics() {
    unsigned long now = millis();
    // If not enough time has passed since the last reading cycle, the function returns early.
    if (now - lastUltraMs < US_PERIOD_MS) {
        return;
    }
    lastUltraMs = now;

    // The front ultrasonic sensor is read and processed through the SMA filter if valid.
    float fr = readUsCm(TRIG_FRONT_PIN, ECHO_FRONT_PIN);
    if (fr > 0) {
        smaAdd(frontBuf, &frontBufIdx, &frontBufCount, fr);
        frontCmFilt   = smaGet(frontBuf, frontBufCount);
        frontInit     = true;
        lastFrontOkMs = now;
    }

    // The right ultrasonic sensor is read and processed through the SMA filter if valid.
    float rr = readUsCm(TRIG_RIGHT_PIN, ECHO_RIGHT_PIN);
    if (rr > 0) {
        smaAdd(rightBuf, &rightBufIdx, &rightBufCount, rr);
        rightCmFilt   = smaGet(rightBuf, rightBufCount);
        rightInit     = true;
        lastRightOkMs = now;
    }
}

/**
 * FRONT VALID
 * Returns true if the front sensor has been initialized and the last valid reading
 * was received within US_HOLD_MS milliseconds of the current time.
 */
bool frontValid() {
    return frontInit && (millis() - lastFrontOkMs <= US_HOLD_MS);
}

/**
 * RIGHT VALID
 * Returns true if the right sensor has been initialized and the last valid reading
 * was received within US_HOLD_MS milliseconds of the current time.
 */
bool rightValid() {
    return rightInit && (millis() - lastRightOkMs <= US_HOLD_MS);
}

/**
 * GET FRONT CM
 * Returns the filtered front distance if valid, or -1.0 if unavailable.
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
 * Returns the filtered right distance if valid, or -1.0 if unavailable.
 */
float getRightCm() {
    if (rightValid()) {
        return rightCmFilt;
    } else {
        return -1.0f;
    }
}

// ============================================================
// SERVO HELPER
// ============================================================

/**
 * SERVO MOVE TO
 * The servoMoveTo function moves the servo to the specified angle and waits for it to settle.
 */
void servoMoveTo(int deg) {
    panServo.write(deg);
    delay(SWEEP_SETTLE_MS);
}

// ============================================================
// SWEEP — distance-scaled object detection
// ============================================================

/**
 * PERFORM SWEEP
 * The performSweep function pans the servo across the sweep arc, takes a distance reading
 * at each step, and scans the resulting depth profile for object candidates.
 *
 * An object candidate must satisfy three conditions:
 *   1. Its distance must be below OBJ_MAX_CM.
 *   2. It must be a local minimum — closer than at least one neighbor by more than
 *      the distance-scaled margin.
 *   3. The angular width of the dip below OBJ_MAX_CM must be less than the distance-scaled
 *      width threshold.
 *
 * The distance scaling works as follows:
 *   The width threshold is scaled up for closer objects because the same physical object
 *   subtends a larger angle when it is closer to the sensor.
 *   The formula is: scaledWidth = OBJ_MAX_WIDTH_DEG * (OBJ_REF_DIST_CM / candidateDistance).
 *   The margin threshold is scaled down for closer objects because at close range,
 *   the absolute difference between an object and background walls shrinks.
 *   The formula is: scaledMargin = OBJ_WALL_MARGIN_CM * (candidateDistance / OBJ_REF_DIST_CM).
 *
 * The function returns the index of the best (closest) candidate, or -1 if none is found.
 * The servo returns to center after the sweep regardless of the result.
 */
int performSweep() {
    Serial.println("[SWEEP] Starting...");

    // The servo sweeps across the arc and a distance reading is taken at each step.
    for (int i = 0; i < SWEEP_STEPS; i++) {
        // The servo angle for this step is calculated from the minimum angle plus the step offset.
        int deg = SWEEP_MIN_DEG + i * SWEEP_STEP_DEG;
        // The servo is moved to this angle.
        panServo.write(deg);
        // The code waits for the servo to settle before taking a reading.
        delay(SWEEP_SETTLE_MS);
        // A direct distance reading is taken from the front ultrasonic sensor.
        float d = readUsCm(TRIG_FRONT_PIN, ECHO_FRONT_PIN);
        // If the sensor returns an invalid reading, it is treated as maximum range (open space).
        if (d > 0) {
            sweepProfile[i] = d;
        } else {
            sweepProfile[i] = US_MAX_CM;
        }
        // The angle and distance are printed to the serial monitor for debugging.
        Serial.print("  ");
        Serial.print(deg);
        Serial.print("deg -> ");
        Serial.print(sweepProfile[i], 1);
        Serial.println("cm");
    }

    // The servo is returned to the center position after the sweep is complete.
    servoMoveTo(SERVO_CENTER_DEG);

    // The depth profile is scanned for narrow local minimums that indicate objects.
    int   bestIdx  = -1;
    float bestDist = OBJ_MAX_CM;

    for (int i = 1; i < SWEEP_STEPS - 1; i++) {
        float curr = sweepProfile[i];
        float prev = sweepProfile[i - 1];
        float next = sweepProfile[i + 1];

        // If this reading is at or beyond the maximum detection range, it is skipped.
        if (curr >= OBJ_MAX_CM) {
            continue;
        }

        // The distance-scaled margin is calculated for this candidate's distance.
        // At close range the margin is smaller because background walls are also close.
        // The margin is clamped to OBJ_MIN_MARGIN_CM to prevent it from becoming negligibly small.
        float scaledMargin = OBJ_WALL_MARGIN_CM * (curr / OBJ_REF_DIST_CM);
        if (scaledMargin < OBJ_MIN_MARGIN_CM) {
            scaledMargin = OBJ_MIN_MARGIN_CM;
        }

        // The candidate must be a local minimum — closer than at least one neighbor by the scaled margin.
        bool isMin = (curr < prev - scaledMargin) || (curr < next - scaledMargin);
        if (!isMin) {
            continue;
        }

        // The angular width of the dip below OBJ_MAX_CM is measured by expanding outward from this bin.
        int runStart = i;
        int runEnd = i;
        while (runStart > 0 && sweepProfile[runStart - 1] < OBJ_MAX_CM) {
            runStart--;
        }
        while (runEnd < SWEEP_STEPS - 1 && sweepProfile[runEnd + 1] < OBJ_MAX_CM) {
            runEnd++;
        }
        int widthDeg = (runEnd - runStart) * SWEEP_STEP_DEG;

        // The distance-scaled width threshold is calculated for this candidate's distance.
        // At close range the threshold is larger because the same physical object appears wider.
        int scaledMaxWidth = (int)(OBJ_MAX_WIDTH_DEG * (OBJ_REF_DIST_CM / curr));
        // The scaled width is clamped to OBJ_MAX_SCALED_WIDTH_DEG to prevent extreme values at very close range.
        if (scaledMaxWidth > OBJ_MAX_SCALED_WIDTH_DEG) {
            scaledMaxWidth = OBJ_MAX_SCALED_WIDTH_DEG;
        }

        // In calibration mode, all candidates and their scaled thresholds are printed regardless of pass or fail.
        if (CALIBRATION_MODE) {
            Serial.print("  [CAL] Bin ");
            Serial.print(i);
            Serial.print(" (");
            Serial.print(SWEEP_MIN_DEG + i * SWEEP_STEP_DEG);
            Serial.print("deg) dist=");
            Serial.print(curr, 1);
            Serial.print("cm  width=");
            Serial.print(widthDeg);
            Serial.print("deg (max=");
            Serial.print(scaledMaxWidth);
            Serial.print(")  margin=");
            Serial.print(scaledMargin, 1);
            Serial.print("cm  prev=");
            Serial.print(prev, 1);
            Serial.print("  next=");
            Serial.print(next, 1);
            if (widthDeg > scaledMaxWidth) {
                Serial.println("  -> REJECTED (too wide)");
            } else {
                Serial.println("  -> PASSED");
            }
        }

        // If the measured width exceeds the scaled width threshold, this candidate is too wide and is a wall.
        if (widthDeg > scaledMaxWidth) {
            if (!CALIBRATION_MODE) {
                Serial.print("  [SWEEP] Candidate at index ");
                Serial.print(i);
                Serial.print(" width=");
                Serial.print(widthDeg);
                Serial.print("deg (max=");
                Serial.print(scaledMaxWidth);
                Serial.println("deg) — TOO WIDE (wall), skipping.");
            }
            continue;
        }

        // If this candidate is closer than the current best, it becomes the new best.
        if (curr < bestDist) {
            bestDist = curr;
            bestIdx  = i;
        }
    }

    // The result is printed to the serial monitor.
    if (bestIdx >= 0) {
        int bestDeg = SWEEP_MIN_DEG + bestIdx * SWEEP_STEP_DEG;
        Serial.print("[SWEEP] Object candidate at ");
        Serial.print(bestDeg);
        Serial.print("deg, dist=");
        Serial.print(bestDist, 1);
        Serial.println("cm");
    } else {
        Serial.println("[SWEEP] No object found.");
    }

    return bestIdx;
}

// ============================================================
// CHARGE — drive toward detected object and ram it
// ============================================================

/**
 * CHARGE AT OBJECT
 * The chargeAtObject function steers the robot toward the detected object and drives straight at it.
 * It first performs a brief proportional turn to face the object's angle, then drives forward
 * while polling the front sensor until the distance drops below CHARGE_IMPACT_CM (hit)
 * or CHARGE_TIMEOUT_MS elapses (lost the object).
 * On impact, the object counter is incremented, the robot stops briefly, then reverses to unstick.
 */
// The sweepIdx parameter is the index into the sweep profile where the object was detected.
void chargeAtObject(int sweepIdx) {
    // The target angle in degrees is calculated from the sweep index.
    int targetDeg = SWEEP_MIN_DEG + sweepIdx * SWEEP_STEP_DEG;
    // The offset from center determines the direction and amount of the pre-turn.
    int offset    = targetDeg - SERVO_CENTER_DEG;

    Serial.print("[CHARGE] Steering toward ");
    Serial.print(targetDeg);
    Serial.print("deg (offset ");
    Serial.print(offset);
    Serial.println("deg)");

    // The turn duration is proportional to the offset — approximately 3 milliseconds per degree.
    int turnDuration = abs(offset) * 3;
    // If the object is to the left of center, the robot turns left.
    if (offset < 0) {
        setMotors(0, WF_TURN_SPEED);
        // If the object is to the right of center, the robot turns right.
    } else if (offset > 0) {
        setMotors(WF_TURN_SPEED, 0);
    }
    // The robot turns for the computed duration.
    if (turnDuration > 0) {
        delay(turnDuration);
    }

    // The servo is centered so the front sensor faces straight ahead during the charge.
    servoMoveTo(SERVO_CENTER_DEG);
    // The charge start time is recorded for timeout tracking.
    unsigned long chargeStart = millis();

    // The robot drives straight at the object until it hits or times out.
    while (millis() - chargeStart < CHARGE_TIMEOUT_MS) {
        // The robot drives forward at charge speed.
        setMotors(CHARGE_SPEED, CHARGE_SPEED);
        // A direct front sensor reading is taken to check the distance to the object.
        float f = readUsCm(TRIG_FRONT_PIN, ECHO_FRONT_PIN);
        Serial.print("[CHARGE] Front=");
        Serial.print(f, 1);
        Serial.println("cm");

        // If the front distance is below CHARGE_IMPACT_CM, the robot has hit the object.
        if (f > 0 && f < CHARGE_IMPACT_CM) {
            // The object hit counter is incremented.
            objectsHit++;
            Serial.print(">>> HIT! Objects hit so far: ");
            Serial.println(objectsHit);
            // The motors are stopped on impact.
            stopMotors();
            // The robot pauses briefly after impact.
            delay(CHARGE_STOP_MS);
            // The robot reverses briefly to unstick from the object.
            setMotors(-WF_STRAIGHT_SPEED, -WF_STRAIGHT_SPEED);
            delay(300);
            stopMotors();
            return;
        }
        // A short delay is added between front sensor polls during the charge.
        delay(20);
    }

    // If the timeout was reached without hitting anything, the object was lost.
    Serial.println("[CHARGE] Timeout — object lost. Resuming wall follow.");
    stopMotors();
}

// ============================================================
// STUCK DETECTION (two-tier)
// ============================================================

/**
 * CHECK STUCK
 * The checkStuck function monitors the front sensor to determine if the robot is stuck.
 * If the front reading stays within WF_STUCK_BAND_CM of a reference value for longer than
 * WF_STUCK_MS milliseconds, a recovery sequence is triggered.
 */
bool checkStuck(float f) {
    unsigned long now = millis();

    // If the front sensor data is invalid, stuck detection is paused.
    if (!frontValid() || f <= 0) {
        stuckTimerActive = false;
        return false;
    }

    // The current front reading is compared to the stored reference.
    bool fStuck = stuckFrontRef >= 0 && abs(f - stuckFrontRef) < WF_STUCK_BAND_CM;

    if (fStuck) {
        // If the stuck timer is not already running, it is started.
        if (!stuckTimerActive) {
            stuckTimerActive = true;
            stuckSinceMs     = now;
            // If the timer has been running longer than WF_STUCK_MS, recovery is triggered.
        } else if (now - stuckSinceMs > WF_STUCK_MS) {
            if (!stuckFirstFired) {
                // Tier 1: full-power reverse then push forward.
                Serial.println("[STUCK-1] Reversing then full power push...");
                setMotors(-WF_STUCK_REVERSE_PWM_1, -WF_STUCK_REVERSE_PWM_1);
                delay(WF_STUCK_REVERSE_MS_1);
                setMotors(WF_STUCK_PUSH_PWM_1, WF_STUCK_PUSH_PWM_1);
                delay(WF_STUCK_PUSH_MS_1);
                stopMotors();
                stuckFirstFired = true;
            } else {
                // Tier 2+: angled reverse to angle the robot rightward.
                Serial.println("[STUCK-N] Angled reverse right...");
                setMotors(-WF_STUCK_REVERSE_PWM_2_R, -WF_STUCK_REVERSE_PWM_2_L);
                delay(WF_STUCK_REVERSE_MS_2);
                stopMotors();
            }
            // The stuck timer is reset so it can detect a new stuck condition.
            stuckTimerActive = false;
            stuckFrontRef    = -1.0f;
            return true;
        }
    } else {
        // If the front reading has changed, the robot is not stuck, so the timer is reset.
        stuckTimerActive = false;
        stuckFrontRef    = f;
    }
    return false;
}

// ============================================================
// DEAD END CHECK + PIVOT 180
// ============================================================

/**
 * CHECK AND HANDLE DEAD END
 * This function detects dead ends by checking if the front is blocked while the right side is open.
 * This combination means the robot is pointing at a dead-end pocket with no usable right wall
 * to follow out of it.
 * The response is to pivot 180 degrees in place and resume.
 */
bool checkAndHandleDeadEnd(float f, float r) {
    // The front is considered blocked if the front sensor reads below WF_FRONT_NEAR_CM.
    bool frontBlocked = frontValid() && f > 0 && f < WF_FRONT_NEAR_CM;
    // The right is considered open if the right sensor has no valid reading or reads above 40cm.
    bool rightOpen    = !rightValid() || r <= 0 || r > 40.0f;

    // If the front is blocked and the right is open, the robot is in a dead end.
    if (frontBlocked && rightOpen) {
        Serial.println("[DEAD END] Front blocked, right open — pivoting 180...");
        // The robot pivots in place by running the left wheel forward and the right wheel backward.
        setMotors(WF_TURN_SPEED, -WF_TURN_SPEED);
        // The robot pivots for 800 milliseconds, which should be approximately 180 degrees.
        delay(825);
        stopMotors();
        delay(100);
        return true;
    }
    return false;
}

// ============================================================
// WALL FOLLOW BURST
// ============================================================

/**
 * WALL FOLLOW BURST
 * The wallFollowBurst function runs wall-follow logic for WF_BURST_MS milliseconds, then returns.
 * The front sensor reads from the SMA with the servo at center between sweeps.
 * The right sensor is the fixed HC-SR04.
 * Dead end detection and stuck detection are checked each iteration.
 */
void wallFollowBurst() {
    unsigned long burstStart = millis();
    Serial.println("[WF] Starting burst...");

    // The wall-follow logic runs until WF_BURST_MS milliseconds have elapsed.
    while (millis() - burstStart < WF_BURST_MS) {
        // The ultrasonic sensors are updated each iteration.
        updateUltrasonics();

        float f = getFrontCm();
        float r = getRightCm();

        // Dead end detection is checked first.
        if (checkAndHandleDeadEnd(f, r)) {
            continue;
        }

        // Stuck detection is checked next.
        if (checkStuck(f)) {
            continue;
        }

        // The rightVisible flag is true if the right sensor has any valid reading.
        bool rightVisible = rightValid() && r > 0;
        // The frontNear flag is true if the front sensor reads below WF_FRONT_NEAR_CM.
        bool frontNear    = frontValid() && f > 0 && f < WF_FRONT_NEAR_CM;

        float target   = WF_RIGHT_NEAR_CM;
        bool  tooClose = rightVisible && r < (target - WF_RIGHT_TOLERANCE_CM);
        bool  tooFar   = rightVisible && r > (target + WF_RIGHT_TOLERANCE_CM);

        // The sensor readings are printed for debugging.
        Serial.print("[WF] Front=");
        if (frontValid()) {
            Serial.print(f, 1);
            Serial.print("cm");
        } else {
            Serial.print("---");
        }
        Serial.print("  Right=");
        if (rightValid()) {
            Serial.print(r, 1);
            Serial.print("cm");
        } else {
            Serial.print("---");
        }
        Serial.print("  -> ");

        // If the front is blocked and the right wall is visible, the robot pivots left (corner).
        if (frontNear && rightVisible) {
            setMotors(WF_TURN_SPEED, -WF_TURN_SPEED);
            Serial.println("TURN LEFT (corner)");
        }
            // If the front is blocked but the right wall is gone, the robot pivots right.
        else if (frontNear && !rightVisible) {
            setMotors(-WF_TURN_SPEED, WF_TURN_SPEED);
            Serial.println("TURN RIGHT (front blocked, no wall)");
        }
            // If the front is clear and the right wall is visible, the robot performs distance maintenance.
        else if (!frontNear && rightVisible) {
            if (tooClose) {
                setMotors(WF_CORRECT_OUTER, WF_CORRECT_INNER);
                Serial.println("CORRECT LEFT (too close)");
            } else if (tooFar) {
                setMotors(WF_CORRECT_INNER, WF_CORRECT_OUTER);
                Serial.println("CORRECT RIGHT (too far)");
            } else {
                setMotors(WF_STRAIGHT_SPEED, WF_STRAIGHT_SPEED);
                Serial.println("STRAIGHT (in band)");
            }
        }
            // If both the front is clear and the right wall is gone, the robot drives straight.
        else {
            setMotors(WF_STRAIGHT_SPEED, WF_STRAIGHT_SPEED);
            Serial.println("STRAIGHT (open)");
        }
    }

    // The motors are stopped at the end of the burst.
    stopMotors();
}

// ============================================================
// SETUP
// ============================================================

/**
 * SETUP
 * The setup function runs once when the Arduino is powered on or reset.
 * It initializes serial communication, configures all pins, stops the motors,
 * attaches and centers the servo, prints the configuration, and waits three seconds.
 */
void setup() {
    // Serial communication is started at 9600 baud.
    Serial.begin(9600);

    // The motor direction and PWM pins are configured as outputs.
    pinMode(PIN_LF, OUTPUT);
    pinMode(PIN_LB, OUTPUT);
    pinMode(PIN_RF, OUTPUT);
    pinMode(PIN_RB, OUTPUT);
    pinMode(L_PWM_PIN, OUTPUT);
    pinMode(R_PWM_PIN, OUTPUT);
    // The motors are stopped immediately.
    stopMotors();

    // The ultrasonic sensor pins are configured.
    pinMode(TRIG_FRONT_PIN, OUTPUT);
    pinMode(ECHO_FRONT_PIN, INPUT);
    pinMode(TRIG_RIGHT_PIN, OUTPUT);
    pinMode(ECHO_RIGHT_PIN, INPUT);
    digitalWrite(TRIG_FRONT_PIN, LOW);
    digitalWrite(TRIG_RIGHT_PIN, LOW);

    // Both SMA filter buffers are filled with zeros.
    for (int i = 0; i < US_SMA_WINDOW; i++) {
        frontBuf[i] = 0;
        rightBuf[i] = 0;
    }

    // The servo is attached and centered.
    panServo.attach(SERVO_PIN);
    servoMoveTo(SERVO_CENTER_DEG);

    // The configuration is printed to the serial monitor.
    Serial.println("=== TEST 3: OBJECT HUNT v2 (distance-scaled) ===");
    if (CALIBRATION_MODE) {
        Serial.println("*** CALIBRATION MODE — robot will NOT move ***");
    }
    Serial.print("Sweep arc: ");
    Serial.print(SWEEP_MIN_DEG);
    Serial.print("deg - ");
    Serial.print(SWEEP_MAX_DEG);
    Serial.print("deg  step=");
    Serial.print(SWEEP_STEP_DEG);
    Serial.println("deg");
    Serial.print("Object detect: < ");
    Serial.print(OBJ_MAX_CM, 1);
    Serial.print("cm  width < ");
    Serial.print(OBJ_MAX_WIDTH_DEG);
    Serial.print("deg @ ");
    Serial.print(OBJ_REF_DIST_CM, 1);
    Serial.println("cm ref");
    Serial.print("WF right target: ");
    Serial.print(WF_RIGHT_NEAR_CM, 1);
    Serial.print("cm  front turn: ");
    Serial.print(WF_FRONT_NEAR_CM, 1);
    Serial.println("cm");
    // A countdown is printed to give the user time to place the robot.
    Serial.println("Starting in 3s...");
    delay(3000);
    Serial.println("GO!");
}

// ============================================================
// MAIN LOOP
// ============================================================

/**
 * MAIN LOOP
 * In normal mode, the loop alternates between sweeping for objects, charging at any detected object,
 * and running a wall-follow burst to explore and reposition.
 * In calibration mode, the robot stays still and repeatedly sweeps with detailed output.
 */
void loop() {
    // In calibration mode, the robot only sweeps and never moves.
    if (CALIBRATION_MODE) {
        performSweep();
        // A 2-second delay between calibration sweeps gives time to read the output.
        delay(2000);
        return;
    }

    // 1. The servo sweeps for objects.
    int objIdx = performSweep();

    // 2. If an object was found, the robot charges at it.
    if (objIdx >= 0) {
        chargeAtObject(objIdx);
    }

    // 3. A wall-follow burst runs to explore and reposition before the next sweep.
    wallFollowBurst();
}
