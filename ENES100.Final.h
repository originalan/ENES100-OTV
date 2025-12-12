/**
 * ENES100 Final Project - Autonomous Firefighting Robot
 *
 * This Arduino sketch implements an autonomous robot for the ENES100 final project.
 * The robot navigates through an obstacle course, detects and extinguishes candles
 * at a mission site, and completes various navigation challenges.
 *
 * Key Features:
 * - ArUco marker pose detection and tracking
 * - Mecanum wheel drive system for omnidirectional movement
 * - Ultrasonic sensors for obstacle detection
 * - Flame sensor and fan system for candle extinguishing
 * - Limit switches for topography detection
 * - State machine-based mission execution
 *
 * Hardware: Arduino Mega 2560 with custom sensor suite
 * Dependencies: Enes100 library for communication
 */

#include "Enes100.h"

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================

// Motor pins - Front Right (FR)
const uint8_t MOTOR_FR_I1 = 22;
const uint8_t MOTOR_FR_I2 = 23;
const uint8_t MOTOR_FR_ENA = 6;

// Motor pins - Back Right (BR)
const uint8_t MOTOR_BR_I1 = 24;
const uint8_t MOTOR_BR_I2 = 25;
const uint8_t MOTOR_BR_ENB = 7;

// Motor pins - Back Left (BL)
const uint8_t MOTOR_BL_I1 = 26;
const uint8_t MOTOR_BL_I2 = 27;
const uint8_t MOTOR_BL_ENA = 8;

// Motor pins - Front Left (FL)
const uint8_t MOTOR_FL_I1 = 28;
const uint8_t MOTOR_FL_I2 = 29;
const uint8_t MOTOR_FL_ENB = 9;

// Ultrasonic sensor pins
const uint8_t US_FRONT_TRIG = 41;
const uint8_t US_FRONT_ECHO = 40;
const uint8_t US_RIGHT_TRIG = 33;
const uint8_t US_RIGHT_ECHO = 32;
const uint8_t US_LEFT_TRIG = 35;
const uint8_t US_LEFT_ECHO = 34;

// Limit switch pins
const uint8_t LIMIT_LEFT = 36;
const uint8_t LIMIT_RIGHT = 37;

// Flame sensor and fan pins
const uint8_t THERMISTOR_PIN = 45;
const uint8_t FAN_PIN = 12;

// WiFi module pins
const uint8_t WIFI_TX = A8;
const uint8_t WIFI_RX = A9;

// ============================================================================
// SYSTEM CONFIGURATION CONSTANTS
// ============================================================================

// Robot identification for ENES100 communication
const int ROOM_NUMBER = 1116;
const uint8_t ARUCO_MARKER_ID = 67;

// Array sizes and system capacities
const uint8_t MOTOR_COUNT = 4;              // Number of motors in mecanum drive system
const uint8_t PATH_STEPS = 5;               // Number of steps in test path sequence

// ============================================================================
// NAVIGATION CONSTANTS
// ============================================================================

// Position and angle thresholds for movement completion
const float POSITION_THRESHOLD = 0.06f;        // meters (~6 cm) - how close to target before stopping
const float ANGLE_THRESHOLD = 0.09f;           // radians (~5 degrees) - angular tolerance for turns

// Maximum allowable speeds for safety
const float MAX_LINEAR_SPEED = 0.5f;           // normalized speed (0.0-1.0)
const float MAX_ROTATIONAL_SPEED = 0.5f;       // normalized rotational speed (0.0-1.0)

// ============================================================================
// SENSOR CONSTANTS
// ============================================================================

// Ultrasonic sensor distance thresholds for obstacle detection
const float OBSTACLE_FRONT_THRESHOLD = 33.0f;  // cm - front obstacle detection distance
const float OBSTACLE_SIDE_THRESHOLD = 15.0f;   // cm - side obstacle detection distance

// Sensor alignment thresholds for mission site positioning
const float SENSOR_DISTANCE_THRESHOLD = 1.0f;     // cm - coarse alignment tolerance
const float SENSOR_DISTANCE_THRESHOLD_FINE = 1.5f; // cm - fine alignment tolerance

// Sensor sampling configuration
const uint8_t OBSTACLE_SAMPLE_COUNT = 8;       // samples for reliable obstacle detection
const uint8_t DEFAULT_SENSOR_SAMPLES = 5;      // default samples for distance averaging
const unsigned long OBSTACLE_SAMPLE_DELAY = 0; // ms between obstacle detection samples
const uint8_t DEFAULT_SAMPLE_DELAY_MS = 0;     // ms between regular distance samples

// Sensor error return values
const float SENSOR_ERROR_DISTANCE = -1.0f;     // returned when sensor reading fails

// ============================================================================
// TIMING CONSTANTS
// ============================================================================

// Communication and pose timeouts
const unsigned long POSE_TIMEOUT = 250;        // ms - how long to keep old pose when marker lost

// Movement timing delays (to allow motors to respond and stabilize)
const unsigned long STRAFE_DELAY = 250;        // ms - delay after strafe movements
const unsigned long TURN_DELAY = 800;          // ms - delay after turning movements
const unsigned long GENERAL_DELAY = 1000;      // ms - general purpose delay

// Obstacle detection timing
const unsigned long OBSTACLE_PAUSE_DURATION = 500;  // ms - pause before checking obstacles

// ============================================================================
// MISSION CONSTANTS
// ============================================================================

// Mission site sensor alignment distances (cm from wall/edge)
const float MISSION_SITE_DISTANCE_RIGHT_TOP = 34.5f;     // right sensor distance for TOP starting position
const float MISSION_SITE_DISTANCE_LEFT_BOTTOM = 45.5f;   // left sensor distance for BOTTOM starting position
const float MISSION_SITE_DISTANCE_CANDLE3_TOP = 45.0f;   // right sensor distance for candle 3 (TOP position)
const float MISSION_SITE_DISTANCE_CANDLE3_BOTTOM = 33.3f; // left sensor distance for candle 3 (BOTTOM position)

// Limit switch detection results
const int NO_LIMIT_SWITCH_DETECTED = -1;  // returned when no limit switches are pressed

// ============================================================================
// ENUMERATIONS
// ============================================================================

// Motor identifiers for the four mecanum drive motors
enum MotorID {
    FR = 1,  // Front Right motor
    FL = 2,  // Front Left motor
    BR = 3,  // Back Right motor
    BL = 4   // Back Left motor
};

// Motor rotation directions
enum MotorDirection {
    FORWARD = 1,   // Clockwise rotation
    BACKWARD = 2,  // Counter-clockwise rotation
    OFF = 3        // Motor stopped
};

enum PathState { START1, MOVE, TOPL, TOPL2,
    CHECK_MISSION_SITE_DISTANCE,
    TOP2, TOP22, TOP222, TOP2222,
    CANDLE1, CANDLE2, CANDLE3, CANDLE3F, CANDLE4,
    TOP3, TOP4, 
    TOP44,
    TOP444,
    TOP5, TOP6, 
    CHECK_OBSTACLE_TOP1,
    CHECK_OBSTACLE_TOP2F, 
    CHECK_OBSTACLE_TOP2FFF,
    CHECK_OBSTACLE_TOP2FF,
    CHECK_OBSTACLE_TOP2, 
    CHECK_OBSTACLE_TOP3F, 
    CHECK_OBSTACLE_TOP3FF,
    CHECK_OBSTACLE_TOP3FFF,
    FORWARD_SECOND_ROW,
    MOVE_TO_SECOND_ROW_TOP,
    MOVE_TO_SECOND_ROW_TOP_HALF,
    MOVE_TO_SECOND_ROW_TOP_HALF_R,
    MOVE_TO_SECOND_ROW_TOP_ROTATE,
    MOVE_TO_SECOND_ROW_TOP_ADJUST,
    SECOND_ROW_TOP,
    SECOND_ROW_MIDDLEFF,
    SECOND_ROW_MIDDLEF,
    SECOND_ROW_MIDDLE,
    SECOND_ROW_BOTTOMF,
    SECOND_ROW_BOTTOMFF,
    SECOND_ROW_MIDDLEFFF,
    FORWARD_THIRD_ROW,
    GO_TO_LIMBO,
    GO_UNDER_LIMBO,
    GO_TO_LIMBO_ROTATE,
    BOTTOM1, BOTTOM2, BOTTOM3, BOTTOM4, BOTTOM5, BOTTOM6,BOTTOM7,BOTTOM8 };

PathState testPath = START1;

// ============================================================================
// STRUCTURES
// ============================================================================

struct Ultrasonic {
    uint8_t trig;
    uint8_t echo;
    float distance;  // cm
    unsigned long lastUpdate;
    static const unsigned long UPDATE_INTERVAL = 40;  // ms between readings
};

struct Pose {
    float x, y, theta;
    unsigned long timestamp;
};

struct PathSegment {
    float x;
    float y;
    float theta;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

bool ENES_INIT;
const bool SERIAL_INIT = true;
Pose lastPose;
bool poseValid;
bool isTopStartingPosition;

// Ultrasonic sensor instances
Ultrasonic ultrasonicFront;
Ultrasonic ultrasonicRight;
Ultrasonic ultrasonicLeft;

// Timing and state variables
// obstaclePauseStartTime and isObstaclePauseActive moved to local static in loop()

// Mission state
int candleCount;

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

/**
 * @brief Initialize all motor control pins and set motors to stopped state
 *
 * Configures all motor driver pins as outputs and ensures all motors start
 * in the OFF state to prevent unintended movement during startup.
 *
 * @return void
 */
void initializeMotors() {
    // Set all motor pins as outputs
    pinMode(MOTOR_FR_I1, OUTPUT);
    pinMode(MOTOR_FR_I2, OUTPUT);
    pinMode(MOTOR_FR_ENA, OUTPUT);
    pinMode(MOTOR_BR_I1, OUTPUT);
    pinMode(MOTOR_BR_I2, OUTPUT);
    pinMode(MOTOR_BR_ENB, OUTPUT);
    pinMode(MOTOR_BL_I1, OUTPUT);
    pinMode(MOTOR_BL_I2, OUTPUT);
    pinMode(MOTOR_BL_ENA, OUTPUT);
    pinMode(MOTOR_FL_I1, OUTPUT);
    pinMode(MOTOR_FL_I2, OUTPUT);
    pinMode(MOTOR_FL_ENB, OUTPUT);

    // Initialize all motors to OFF
    digitalWrite(MOTOR_FR_I1, LOW);
    digitalWrite(MOTOR_FR_I2, LOW);
    analogWrite(MOTOR_FR_ENA, 0);
    digitalWrite(MOTOR_BR_I1, LOW);
    digitalWrite(MOTOR_BR_I2, LOW);
    analogWrite(MOTOR_BR_ENB, 0);
    digitalWrite(MOTOR_BL_I1, LOW);
    digitalWrite(MOTOR_BL_I2, LOW);
    analogWrite(MOTOR_BL_ENA, 0);
    digitalWrite(MOTOR_FL_I1, LOW);
    digitalWrite(MOTOR_FL_I2, LOW);
    analogWrite(MOTOR_FL_ENB, 0);
}

/**
 * @brief Initialize all sensor pins and set initial states
 *
 * Configures ultrasonic sensor trigger/echo pins, limit switch pins with
 * pull-up resistors, flame sensor pin, and fan control pin. Sets fan to OFF state.
 *
 * @return void
 */
void initSensors() {
    // Ultrasonic sensors
    pinMode(US_FRONT_TRIG, OUTPUT);
    pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT);
    pinMode(US_RIGHT_ECHO, INPUT);
    pinMode(US_LEFT_TRIG, OUTPUT);
    pinMode(US_LEFT_ECHO, INPUT);

    // Limit switches (with pull-up resistors)
    pinMode(LIMIT_LEFT, INPUT_PULLUP);
    pinMode(LIMIT_RIGHT, INPUT_PULLUP);

    // Flame sensor (thermistor) and fan
    pinMode(THERMISTOR_PIN, INPUT_PULLUP);
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, LOW);
}

/**
 * @brief Initialize WiFi communication with ENES100 system
 *
 * Establishes connection to the ENES100 communication network using the
 * specified team name, mission type, ArUco marker ID, room number, and
 * serial communication pins. Includes startup delay for stable initialization.
 *
 * @return void
 */
void initWIFI() {
    delay(1000);
    Enes100.begin("Smoke Signal", FIRE, ARUCO_MARKER_ID, ROOM_NUMBER, WIFI_TX, WIFI_RX);
    ENES_INIT = true;
}

// ============================================================================
// VISION AND LOCALIZATION FUNCTIONS
// ============================================================================

/**
 * @brief Get current robot pose from ArUco marker detection
 *
 * Retrieves the robot's position and orientation from the ENES100 vision system.
 * If marker is visible, returns fresh pose data. If not visible, returns the
 * last known pose if it's within the timeout window, otherwise marks pose as invalid.
 *
 * @return Pose structure containing x, y, theta coordinates and timestamp
 */
Pose getArucoMarkerPose() {
    Pose pose;
    if (Enes100.isVisible()) {
        // ArUco marker is visible - get fresh pose data from vision system
        pose = {Enes100.getX(), Enes100.getY(), Enes100.getTheta(), millis()};
        lastPose = pose;  // Cache this pose for when marker is lost
        poseValid = true;
    } else {
        // Marker not visible - use cached pose if it's still recent
        pose = lastPose;
        unsigned long age = millis() - lastPose.timestamp;
        // Mark pose as invalid if it's older than POSE_TIMEOUT (marker lost too long)
        poseValid = (age < POSE_TIMEOUT);
    }
    return pose;
}

// ============================================================================
// ULTRASONIC SENSOR FUNCTIONS
// ============================================================================

/**
 * @brief Read distance from ultrasonic sensor in centimeters
 *
 * Sends ultrasonic pulse and measures echo return time to calculate distance.
 * Uses standard HC-SR04 timing protocol with 10µs trigger pulse and timeout
 * at 35ms (~6m max range).
 *
 * @param trig Trigger pin number for the ultrasonic sensor
 * @param echo Echo pin number for the ultrasonic sensor
 * @return Distance in centimeters, or SENSOR_ERROR_DISTANCE (-1.0f) if timeout
 */
float readUltrasonic(uint8_t trig, uint8_t echo) {
    // HC-SR04 ultrasonic sensor timing protocol:
    // 1. Ensure trigger pin starts LOW (clean state)
    digitalWrite(trig, LOW);
    delayMicroseconds(2);

    // 2. Send 10µs HIGH pulse on trigger to initiate measurement
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // 3. Measure echo pulse duration (time for sound to travel to object and back)
    // Timeout after 35ms (~6m max range at speed of sound)
    long duration = pulseIn(echo, HIGH, 35000);  // Returns microseconds

    if (duration == 0) return SENSOR_ERROR_DISTANCE;  // Timeout = no valid reading

    // 4. Convert echo duration to distance:
    // - Speed of sound in air ≈ 343 m/s = 0.0343 cm/µs
    // - Divide by 2 because sound travels to object and back (round trip)
    // - Formula: distance = (duration * speed_of_sound) / 2
    return (duration * 0.0343f) / 2.0f;
}

/**
 * @brief Get average distance reading from front ultrasonic sensor
 *
 * Takes multiple distance measurements from the front sensor and returns
 * the average to reduce noise and improve reliability. Invalid readings
 * (timeouts) are filtered out.
 *
 * @param samples Number of samples to average (must be > 0)
 * @param sampleDelayMs Delay between samples in milliseconds
 * @return Average distance in cm, or SENSOR_ERROR_DISTANCE if no valid readings
 */
float getFrontDistanceAverage(uint8_t samples, unsigned long sampleDelayMs) {
    if (samples == 0) return SENSOR_ERROR_DISTANCE;

    float sum = 0.0f;
    uint8_t validSamples = 0;
    for (uint8_t i = 0; i < samples; i++) {
        float reading = readUltrasonic(ultrasonicFront.trig, ultrasonicFront.echo);
        if (reading > 0.0f) {
            sum += reading;
            validSamples++;
        }
        if (sampleDelayMs > 0) {
            delay(sampleDelayMs);
        }
    }
    if (validSamples == 0) return SENSOR_ERROR_DISTANCE;
    return sum / static_cast<float>(validSamples);
}

float getRightDistanceAverage(uint8_t samples, unsigned long sampleDelayMs) {
    if (samples == 0) return SENSOR_ERROR_DISTANCE;

    float sum = 0.0f;
    uint8_t validSamples = 0;
    for (uint8_t i = 0; i < samples; i++) {
        float reading = readUltrasonic(ultrasonicRight.trig, ultrasonicRight.echo);
        if (reading > 0.0f) {
            sum += reading;
            validSamples++;
        }
        if (sampleDelayMs > 0) {
            delay(sampleDelayMs);
        }
    }
    if (validSamples == 0) return SENSOR_ERROR_DISTANCE;
    return sum / static_cast<float>(validSamples);
}

float getLeftDistanceAverage(uint8_t samples, unsigned long sampleDelayMs) {
    if (samples == 0) return SENSOR_ERROR_DISTANCE;

    float sum = 0.0f;
    uint8_t validSamples = 0;
    for (uint8_t i = 0; i < samples; i++) {
        float reading = readUltrasonic(ultrasonicLeft.trig, ultrasonicLeft.echo);
        if (reading > 0.0f) {
            sum += reading;
            validSamples++;
        }
        if (sampleDelayMs > 0) {
            delay(sampleDelayMs);
        }
    }
    if (validSamples == 0) return SENSOR_ERROR_DISTANCE;
    return sum / static_cast<float>(validSamples);
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

/**
 * @brief Set speed for individual mecanum wheel motor
 *
 * Controls a single motor in the mecanum drive system. Converts normalized
 * speed (-1.0 to 1.0) to PWM duty cycle and sets appropriate direction pins.
 * Invalid motor IDs are logged and ignored.
 *
 * @param motorNumber Motor identifier (FR, FL, BR, BL)
 * @param normalized Normalized speed (-1.0 = full reverse, 0.0 = stop, 1.0 = full forward)
 * @return void
 */
void setMotorSpeed(MotorID motorNumber, float normalized) {
    // Validate motor number
    if (motorNumber < FR || motorNumber > BL) {
        printlnToConsole("ERROR: Invalid motor number: " + String(motorNumber));
        return;
    }

    // Clamp to [-1, 1]
    normalized = constrain(normalized, -1.0, 1.0);

    // Determine direction
    MotorDirection direction = OFF;
    if (normalized > 0.0) {
        direction = FORWARD;
    } else if (normalized < 0.0) {
        direction = BACKWARD;
    }

    // Convert normalized speed (-1.0 to 1.0) to PWM duty cycle (0-255)
    // fabs() ensures we get positive PWM value regardless of direction
    uint8_t pwm = (uint8_t)(fabs(normalized) * 255.0);

    // Each motor has 2 direction control pins (I1/I2) and 1 PWM enable pin (ENA/ENB)
    // Direction pins control H-bridge logic for forward/reverse rotation
    uint8_t pin1, pin2, pwmPin;
    bool pin1State, pin2State;

    switch (motorNumber) {
        case FR:
            pin1 = MOTOR_FR_I1;
            pin2 = MOTOR_FR_I2;
            pwmPin = MOTOR_FR_ENA;
            pin1State = (direction == BACKWARD);
            pin2State = (direction == FORWARD);
            break;
        case BR:
            pin1 = MOTOR_BR_I1;
            pin2 = MOTOR_BR_I2;
            pwmPin = MOTOR_BR_ENB;
            pin1State = (direction == FORWARD);
            pin2State = (direction == BACKWARD);
            break;
        case BL:
            pin1 = MOTOR_BL_I1;
            pin2 = MOTOR_BL_I2;
            pwmPin = MOTOR_BL_ENA;
            pin1State = (direction == FORWARD);
            pin2State = (direction == BACKWARD);
            break;
        case FL:
            pin1 = MOTOR_FL_I1;
            pin2 = MOTOR_FL_I2;
            pwmPin = MOTOR_FL_ENB;
            pin1State = (direction == BACKWARD);
            pin2State = (direction == FORWARD);
            break;
        default:
            return;  // Invalid motor number
    }

    digitalWrite(pin1, pin1State ? HIGH : LOW);
    digitalWrite(pin2, pin2State ? HIGH : LOW);
    analogWrite(pwmPin, pwm);
}

void turnOffMotors() {
    setMotorSpeed(FL, 0);
    setMotorSpeed(FR, 0);
    setMotorSpeed(BR, 0);
    setMotorSpeed(BL, 0);
}

/**
 * @brief Control omnidirectional movement using mecanum wheel kinematics
 *
 * Converts velocity commands into individual wheel speeds for holonomic movement.
 * Applies kinematic equations for mecanum wheels and normalizes speeds to prevent
 * saturation. Supports simultaneous translation and rotation.
 *
 * @param vx Forward/backward velocity (-1.0 to 1.0)
 * @param vy Left/right strafing velocity (-1.0 to 1.0)
 * @param vr Rotational velocity (-1.0 to 1.0, positive = counter-clockwise)
 * @return void
 */
void mecanumDrive(float vx, float vy, float vr) {
    // Mecanum wheel kinematics
    double fr = vx - vy - vr;
    double fl = vx + vy + vr;
    double br = vx + vy - vr;
    double bl = vx - vy + vr;

    // Normalize speeds if any wheel would exceed ±1.0 to maintain requested velocity ratios
    // This prevents motor saturation while preserving intended motion direction
    double maxVal = max(max(fabs(fl), fabs(fr)), max(fabs(bl), fabs(br)));
    if (maxVal > 1.0) {
        fl /= maxVal;
        fr /= maxVal;
        bl /= maxVal;
        br /= maxVal;
    }

    setMotorSpeed(FL, fl);
    setMotorSpeed(FR, fr);
    setMotorSpeed(BL, bl);
    setMotorSpeed(BR, br);
}

// ============================================================================
// MOTION CONTROL AND NAVIGATION FUNCTIONS
// ============================================================================

// Turn direction: true = clockwise, false = counter-clockwise
bool isTurningClockwise = false;
bool turnToAngleSmall(float thetaTarget, float power) {
    Pose p = {Enes100.getX(), Enes100.getY(), Enes100.getTheta(), millis()};
    double error = (thetaTarget - p.theta);
    isTurningClockwise = (error < 0);
    error = fabs(error);
    if (error < 0.055) {
        return true;
    }
    if (isTurningClockwise) {
        mecanumDrive(0, 0, power);
        delay(175);
        turnOffMotors();
        delay(TURN_DELAY);
    } else {
        mecanumDrive(0, 0, -power);
        delay(175);
        turnOffMotors();
        delay(TURN_DELAY);
    }
    return false;
}
bool turnToAngleSmall(float thetaTarget, float power, int delayMS) {
    Pose p = {Enes100.getX(), Enes100.getY(), Enes100.getTheta(), millis()};
    double error = (thetaTarget - p.theta);
    isTurningClockwise = (error < 0);
    error = fabs(error);
    if (error < 0.07) {
        return true;
    }
    if (isTurningClockwise) {
        mecanumDrive(0, 0, power);
        delay(delayMS);
        turnOffMotors();
        delay(TURN_DELAY);
    } else {
        mecanumDrive(0, 0, -power);
        delay(delayMS);
        turnOffMotors();
        delay(TURN_DELAY);
    }
    return false;
}

bool turnToAngle(float thetaTarget, float power, bool direction) {
    Pose p = getArucoMarkerPose();
    double error = (thetaTarget - p.theta);

    if (fabs(error) < ANGLE_THRESHOLD) {
        turnOffMotors();
        return true;
    }

    double vr = power;
    if (direction == false) {
        vr = vr * -1.0;
    }
    vr = constrain(vr, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);

    if (poseValid) {
        mecanumDrive(0, 0, vr);
    } else {
        turnOffMotors();
    }
    return false;
}

bool moveLongToX(float targetX, float power) {

    Pose p = getArucoMarkerPose();
    power = constrain(power, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

    double errorX = (targetX - p.x);
    double distanceX = fabs(errorX);

    if (distanceX < POSITION_THRESHOLD) {
        turnOffMotors();
        return true;
    }

    // Determine if the target is in front or behind the robot
    double forwardComponent = errorX * cos(p.theta);

    // Auto-select forward/backward direction
    double drivePower = (forwardComponent >= 0) ? power : -power;

    if (poseValid) {
        mecanumDrive(drivePower, 0, 0);
    }else {
        turnOffMotors();
    }
    
    return false; // still moving
}
bool moveLongToY(float targetY, float power) {

    Pose p = getArucoMarkerPose();
    power = constrain(power, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

    double errorY = targetY - p.y;
    double distanceY = fabs(errorY);

    if (distanceY < POSITION_THRESHOLD) {
        turnOffMotors();
        return true;
    }

    // Determine if the target is in front or behind the robot
    double forwardComponent = errorY * sin(p.theta);

    // Auto-select forward/backward direction
    double drivePower = (forwardComponent >= 0) ? power : -power;

    if (poseValid) {
        mecanumDrive(drivePower, 0, 0);
    }else {
        turnOffMotors();
    }
    
    return false; // still moving
}
bool moveStrafeToX(float targetX, float power) {

    Pose p = getArucoMarkerPose();
    power = constrain(power, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

    double errorX = (targetX - p.x);
    double distanceX = fabs(errorX);

    if (distanceX < POSITION_THRESHOLD) {
        turnOffMotors();
        return true;
    }

    // Determine if the target is in front or behind the robot
    double forwardComponent = errorX * sin(p.theta);

    // Auto-select forward/backward direction
    double drivePower = (forwardComponent >= 0) ? power : -power;

    if (poseValid) {
        mecanumDrive(0, drivePower, 0);
    }else {
        turnOffMotors();
    }
    
    return false; // still moving
}
bool moveStrafeToRightSensor(float targetDistanceRight, float power) {

    Pose p = getArucoMarkerPose();
    float right = getRightDistanceAverage(DEFAULT_SENSOR_SAMPLES, DEFAULT_SAMPLE_DELAY_MS);
    power = constrain(power, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

    double errorX = (right - targetDistanceRight);
    double distanceX = fabs(errorX);

    if (distanceX < SENSOR_DISTANCE_THRESHOLD) {
        return true;
    }

    // Determine if the target is in front or behind the robot
    double forwardComponent = errorX;

    // Auto-select forward/backward direction
    double drivePower = (forwardComponent >= 0) ? power : -power;

    if (poseValid) {
        mecanumDrive(0, drivePower, 0);
        delay(STRAFE_DELAY);
        turnOffMotors();
        delay(800);
    }else {
        turnOffMotors();
    }
    return false; // still moving
}
bool moveStrafeToRightSensor(float targetDistanceRight, float power, int delayMS) {

    Pose p = getArucoMarkerPose();
    float right = getRightDistanceAverage(DEFAULT_SENSOR_SAMPLES, DEFAULT_SAMPLE_DELAY_MS);
    power = constrain(power, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

    double errorX = (right - targetDistanceRight);
    double distanceX = fabs(errorX);

    if (distanceX < SENSOR_DISTANCE_THRESHOLD_FINE) {
        return true;
    }

    // Determine if the target is in front or behind the robot
    double forwardComponent = errorX;

    // Auto-select forward/backward direction
    double drivePower = (forwardComponent >= 0) ? power : -power;

    if (poseValid) {
        mecanumDrive(0, drivePower, 0);
        delay(delayMS);
        turnOffMotors();
        delay(800);
    }else {
        turnOffMotors();
    }
    return false; // still moving
}
bool moveStrafeToLeftSensor(float targetDistanceLeft, float power) {

    Pose p = getArucoMarkerPose();
    float left = getLeftDistanceAverage(DEFAULT_SENSOR_SAMPLES, DEFAULT_SAMPLE_DELAY_MS);
    power = constrain(power, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

    double errorX = (left - targetDistanceLeft);
    double distanceX = fabs(errorX);

    if (distanceX < SENSOR_DISTANCE_THRESHOLD) {
        return true;
    }

    // Determine if the target is in front or behind the robot
    double forwardComponent = -1.0 * errorX;

    // Auto-select forward/backward direction
    double drivePower = (forwardComponent >= 0) ? power : -power;

    if (poseValid) {
        mecanumDrive(0, drivePower, 0);
        delay(250);
        turnOffMotors();
        delay(800);
    }else {
        turnOffMotors();
    }
    return false; // still moving
}
bool moveStrafeToY(float targetY, float power) {

    Pose p = getArucoMarkerPose();
    power = constrain(power, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

    double errorY = targetY - p.y;
    double distanceY = fabs(errorY);

    if (distanceY < POSITION_THRESHOLD) {
        turnOffMotors();
        return true;
    }

    // Determine if the target is in front or behind the robot
    double forwardComponent = errorY * -1.0 * cos(p.theta);

    // Auto-select forward/backward direction
    double drivePower = (forwardComponent >= 0) ? power : -power;

    if (poseValid) {
        mecanumDrive(0, drivePower, 0);
    }else {
        turnOffMotors();
    }
    
    return false; // still moving
}

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

/**
 * @brief Check topography limit switches and return terrain type
 *
 * Reads both left and right limit switches to determine the terrain topology
 * at the mission site. Returns specific codes for different terrain configurations:
 * - TOP_A: Left switch only
 * - TOP_B: Right switch only
 * - TOP_C: Both switches
 * - NO_LIMIT_SWITCH_DETECTED (-1): Neither switch
 *
 * @return Terrain topology code or NO_LIMIT_SWITCH_DETECTED
 */
int checkLimitSwitches() {
    bool leftPressed = (digitalRead(LIMIT_LEFT) == LOW);
    bool rightPressed = (digitalRead(LIMIT_RIGHT) == LOW);

    if (leftPressed && rightPressed) return TOP_C;
    if (leftPressed) return TOP_A;
    if (rightPressed) return TOP_B;
    return -1;
}

/**
 * @brief Check if flame is currently detected by thermistor sensor
 *
 * Reads the digital thermistor input and returns true if flame is present.
 * The sensor outputs LOW when flame is detected (heating effect on thermistor).
 *
 * @return true if flame detected, false otherwise
 */
bool updateFlameSensor() {
    int sensorValue = digitalRead(THERMISTOR_PIN);

    // Flame is present when sensor reads LOW
    return (sensorValue == LOW);
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Print message to both ENES100 and serial console
 *
 * Outputs text to available communication channels for debugging and monitoring.
 * Supports both ENES100 wireless communication and local serial output.
 *
 * @param msg Message to print (without newline)
 * @return void
 */
void printToConsole(const String& msg) {
    if (ENES_INIT) {
        Enes100.print(msg);
    }
    if (SERIAL_INIT) {
        Serial.print(msg);
    }
}

/**
 * @brief Print message with newline to both ENES100 and serial console
 *
 * Outputs text followed by newline to available communication channels.
 * Supports both ENES100 wireless communication and local serial output.
 *
 * @param msg Message to print (with newline)
 * @return void
 */
void printlnToConsole(const String& msg) {
    if (ENES_INIT) {
        Enes100.println(msg);
    }
    if (SERIAL_INIT) {
        Serial.println(msg);
    }
}

void testEachMotor(unsigned long duration, float speed) {
    MotorID motors[MOTOR_COUNT] = {FR, FL, BR, BL};
    String motorNames[MOTOR_COUNT] = {"Front Right", "Front Left", "Back Right", "Back Left"};

    for (int i = 0; i < MOTOR_COUNT; i++) {
        printlnToConsole("Testing " + motorNames[i] + " forward");
        setMotorSpeed(motors[i], speed);
        delay(duration);
        turnOffMotors();
        delay(500);

        printlnToConsole("Testing " + motorNames[i] + " backward");
        setMotorSpeed(motors[i], -speed);
        delay(duration);
        turnOffMotors();
        delay(500);
    }
    printlnToConsole("Motor test complete!");
}

// ============================================================================
// HIGH-LEVEL HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Extinguish a candle if flame is detected
 *
 * Checks for flame presence using the thermistor sensor and activates the fan
 * to extinguish the candle if detected. Updates the global candle counter.
 * This function encapsulates the complete candle extinguishing sequence.
 *
 * @return void
 */
void extinguishCandle() {
    bool flamePresent = updateFlameSensor();
    if (!flamePresent) {
        return; // Early return if no flame detected
    }

    candleCount++;
    digitalWrite(FAN_PIN, HIGH);
    delay(3000);  // TODO: Make fan duration configurable via constant
    digitalWrite(FAN_PIN, LOW);
    delay(1000);
    printlnToConsole("Num Candles: " + String(candleCount));
}

/**
 * @brief Execute a motor movement with delay and optional stop
 *
 * Moves the robot using mecanum drive kinematics for the specified duration,
 * then stops the motors. Optionally waits for an additional delay after stopping.
 * This consolidates the common pattern of move-delay-stop sequences.
 *
 * @param vx Forward/backward velocity component (-1.0 to 1.0)
 * @param vy Left/right strafing velocity component (-1.0 to 1.0)
 * @param vr Rotational velocity component (-1.0 to 1.0)
 * @param moveDelayMs Duration to move in milliseconds
 * @param stopDelayMs Optional additional delay after stopping motors (default: 0)
 * @return void
 */
void moveWithDelay(float vx, float vy, float vr, unsigned long moveDelayMs, unsigned long stopDelayMs = 0) {
    mecanumDrive(vx, vy, vr);
    delay(moveDelayMs);
    turnOffMotors();
    if (stopDelayMs > 0) {
        delay(stopDelayMs);
    }
}

/**
 * @brief Check for obstacles and determine navigation path
 *
 * Implements a timed pause for reliable obstacle detection using ultrasonic sensors.
 * Takes multiple readings and determines if the path ahead is clear or blocked.
 * This function manages its own timing state across multiple calls.
 *
 * @param nextStateIfObstacle Reference to PathState that will be set if obstacle detected
 * @param nextStateIfClear Reference to PathState that will be set if path is clear
 * @return true if obstacle detected, false if clear or still checking
 */
bool checkForObstacles(PathState& nextStateIfObstacle, PathState& nextStateIfClear) {
    static unsigned long obstaclePauseStartTime = 0;
    static bool isObstaclePauseActive = false;
    unsigned long now = millis();

    if (!isObstaclePauseActive) {
        isObstaclePauseActive = true;
        obstaclePauseStartTime = now;
        printlnToConsole("Pausing to check for obstacles...");
        return false; // Still checking
    }

    if (now - obstaclePauseStartTime >= OBSTACLE_PAUSE_DURATION) {
        float avgFront = getFrontDistanceAverage(OBSTACLE_SAMPLE_COUNT, OBSTACLE_SAMPLE_DELAY);
        isObstaclePauseActive = false;

        if (avgFront < 0) {
            printlnToConsole("Front sensor readings invalid; assuming clear path");
            nextStateIfClear = FORWARD_SECOND_ROW;
            return false; // Clear path
        } else if (avgFront < OBSTACLE_FRONT_THRESHOLD) {
            printlnToConsole("Obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
            return true; // Obstacle detected
        } else {
            printlnToConsole("No obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
            nextStateIfClear = FORWARD_SECOND_ROW;
            return false; // Clear path
        }
    }

    return false; // Still checking
}

// ============================================================================
// MAIN FUNCTIONS
// ============================================================================

int FORWARD_SECOND_ROW_VALUE = 1.51; 
void setup() {
    // Initialize global variables with clear default values
    ENES_INIT = false;
    lastPose = {0.0f, 0.0f, 0.0f, 0};
    poseValid = false;

    // Initialize ultrasonic sensor instances
    ultrasonicFront = {US_FRONT_TRIG, US_FRONT_ECHO, -1.0f, 0};
    ultrasonicRight = {US_RIGHT_TRIG, US_RIGHT_ECHO, -1.0f, 0};
    ultrasonicLeft = {US_LEFT_TRIG, US_LEFT_ECHO, -1.0f, 0};

    // Initialize timing and state variables
    // obstaclePauseStartTime and isObstaclePauseActive initialized as static locals in loop()

    // Initialize mission state
    candleCount = 1;
    testPath = START1;
    isTopStartingPosition = true;

    initWIFI();
    if (SERIAL_INIT) {
        Serial.begin(9600);
    }
    printlnToConsole("Initializing...");

    initializeMotors();
    initSensors();

    printlnToConsole("Ready!");
    printlnToConsole("Wait 1 second...");
    delay(1000);
    printlnToConsole("GO");
}

void testFlameSensor() {
    unsigned long now = millis();

    int sensorValue = digitalRead(THERMISTOR_PIN);
    if (sensorValue == HIGH) {
        printlnToConsole("No, Flame Not Detected");
        delay(50);
        return;
    }

    printlnToConsole("Yes, Flame Detected");
    delay(50);
}

void testLimitSwitches() {
    bool leftPressed = (digitalRead(LIMIT_LEFT) == LOW);
    bool rightPressed = (digitalRead(LIMIT_RIGHT) == LOW);

    printlnToConsole("limit switch left: " + String(leftPressed));
    printlnToConsole("limit switch right: " + String(rightPressed));
}

void testFan() {
    printlnToConsole("FAN ON");
    digitalWrite(FAN_PIN, HIGH);
    delay(3000);
    printlnToConsole("FAN OFF");
    digitalWrite(FAN_PIN, LOW);
    delay(3000);
}

void testSideSensors() {
    float right = readUltrasonic(ultrasonicRight.trig, ultrasonicRight.echo);
    float left = readUltrasonic(ultrasonicLeft.trig, ultrasonicLeft.echo);
    printlnToConsole(String(left) + ", " + String(right));
}

void loop() {
    unsigned long now = millis();
    static bool printed = false;  // Local static variable for START1 state
    static unsigned long obstaclePauseStartTime = 0;  // Local static for obstacle checking
    static bool isObstaclePauseActive = false;  // Local static for obstacle checking

    if (testPath == START1) {
        if (!printed) {
                printlnToConsole("Scanning for ArUco marker...");
                printed = true;
            }
            if (Enes100.isVisible()) {
                printlnToConsole("ArUco Marker Detected");
                testPath = MOVE;
            }
    } else if (testPath == MOVE) {
        Pose pose = getArucoMarkerPose();
        if (pose.theta > 0) {
            printlnToConsole("Detected OTV at TOP");
            printlnToConsole("Turning to face mission site");
            isTopStartingPosition = true;
            testPath = TOPL;
        }else {
            printlnToConsole("Detected OTV at BOTTOM.");
            printlnToConsole("Turning to face mission site");
            isTopStartingPosition = false;
            testPath = BOTTOM1;
        }
    } else if (testPath == TOPL) {
        // going to mission site - turning around to face it
        moveWithDelay(0, 0, 0.27, 2900, 1000);
        printlnToConsole("turning at smaller speed");
        testPath = TOPL2;
    } else if (testPath == TOPL2) {
        // going to mission site - fine tuning theta
        if (turnToAngleSmall(-1.5708, 0.25, 220)) {
            printlnToConsole("moving to mission site");
            testPath = TOP2;
            delay(1000);
        }
    }else if (testPath == TOP2) {
        // going to mission site - moving forward to get closer to mission site
        if (moveLongToY(0.95, 0.24)) {
            delay(1000);
            testPath = TOP22; // do the mission stuff after this step
        }
    }else if (testPath == TOP22) {
        // going to mission site - strafing to align limit switches
        if (moveStrafeToX(0.25, 0.3)) {
            delay(1000);
            testPath = TOP222;
        }

    }else if (testPath == TOP222) {
        // going to mission site - fine tuning theta after strafing
        if (turnToAngleSmall(-1.5708, 0.25)) {
            testPath = TOP2222;
            delay(1000);
        }

    }

    else if (testPath == TOP2222) {
        // mission site - aligning to right candle(s)
        if (isTopStartingPosition == true) {
            if (moveStrafeToRightSensor(34.5f, 0.32f)) {
                testPath = CHECK_MISSION_SITE_DISTANCE;
                delay(1000);
            }
        }else {
            if (moveStrafeToLeftSensor(45.0f, 0.32f)) {
                testPath = CHECK_MISSION_SITE_DISTANCE;
                delay(1000);
            }
        }
    }
    
    else if (testPath == CHECK_MISSION_SITE_DISTANCE) {
        // mission site - ramming into mission site for limit switches
        double dir = (isTopStartingPosition == true) ? -1.5708 : 1.5458;
        if (turnToAngleSmall(dir, 0.25)) {
            delay(1000);
            moveWithDelay(0.27, 0, 0, 1000, 1000); // 5 second delay
            int detection = checkLimitSwitches();
            if (detection != -1) {
                Enes100.mission(TOPOGRAPHY, detection);
                if (detection == TOP_A) {
                    printlnToConsole("TOPOLOGY: A");
                }else if (detection == TOP_B) {
                    printlnToConsole("TOPOLOGY: B");
                }else {
                    printlnToConsole("TOPOLOGY: C");
                }
            }else {
                printlnToConsole("Invalid topology detected");
            }
            testPath = CANDLE1;
        }
    }else if (testPath == CANDLE1) {
        // mission site - successfully aligned to candle1
        delay(1000);
        extinguishCandle();
        testPath = CANDLE2;
    }else if (testPath == CANDLE2) {
        // mission site - successfully aligned to candle2
        moveWithDelay(-0.25, 0, 0, 700, 1000);
        extinguishCandle();
        testPath = CANDLE3;
    }else if (testPath == CANDLE3) {
        // mission site - aligning to candle3
        if (isTopStartingPosition == true) {
            if (moveStrafeToRightSensor(45.0f, 0.32f, 280)) {
                delay(1000);
                testPath = CANDLE3F;
            }
        }else {
            if (moveStrafeToLeftSensor(33.3f, 0.32f)) {
                delay(1000);
                testPath = CANDLE3F;
            }
        }
    }else if (testPath == CANDLE3F) {
        // mission site - successfully aligned to candle3 with fine tuning theta
        double goal = (isTopStartingPosition == true) ? -1.5708 : 1.5458;
        if (turnToAngleSmall(goal, 0.25)) {
            delay(1000);
            moveWithDelay(0.26, 0, 0, 1100, 1000);
            extinguishCandle();
            testPath = CANDLE4;
        }
    }
    
    else if (testPath == CANDLE4) {
        // mission site - successfully aligned to candle4
        moveWithDelay(-0.25, 0, 0, 700, 1000);
        extinguishCandle();
        Enes100.mission(NUM_CANDLES, candleCount);
        if (isTopStartingPosition == true) {
            testPath = TOP3;
        }else {
            testPath = BOTTOM6;
        }
    }

    switch(testPath) {
        case TOP3:
            // going to obstacles - moving backwards and turning to face obstacles
            if (moveLongToY(1.5, 0.25)) {
                // turn angle
                moveWithDelay(0, 0, -0.25, 1300, 1000);
                printlnToConsole("turning to face obstacles");
                testPath = TOP4;
            }
        break;
        case TOP4:
            // going to obstacles - fine tuning theta
            if (turnToAngleSmall(0, 0.29)) {
                testPath = TOP44;
                delay(1000);
            }
        break;
        case TOP44:
            // going to obstacles - adjusting Y coordinate
            if (moveStrafeToY(1.5, 0.35)) {
                testPath = TOP444;
                delay(1000);
            }
        break;
        case TOP444:
            // going to obstacles - fine tuning theta
            if (turnToAngleSmall(0, 0.25)) {
                testPath = TOP5;
                delay(1000);
            }
        break;
        case TOP5:
            if (isTopStartingPosition == true) {
                // going to obstacles - moving to first obstacle
                if (moveLongToX(0.65, 0.25)) {
                    isObstaclePauseActive = false;
                    testPath = CHECK_OBSTACLE_TOP1;
                    delay(1000);
                }
            }else {
                if (moveLongToX(0.65, 0.25)) {
                    isObstaclePauseActive = false;
                    testPath = CHECK_OBSTACLE_TOP1;
                    delay(1000);
                }
            }
        break;

        case CHECK_OBSTACLE_TOP1:
            if (!isObstaclePauseActive) {
                isObstaclePauseActive = true;
                obstaclePauseStartTime = now;
                printlnToConsole("Pausing to check for obstacles...");
            } else if (now - obstaclePauseStartTime >= OBSTACLE_PAUSE_DURATION) {
                float avgFront = getFrontDistanceAverage(OBSTACLE_SAMPLE_COUNT, OBSTACLE_SAMPLE_DELAY);
                isObstaclePauseActive = false;
                if (avgFront < 0) {
                    printlnToConsole("Front sensor readings invalid; assuming clear path");
                    FORWARD_SECOND_ROW_VALUE = 1.48;
                    testPath = FORWARD_SECOND_ROW;
                    delay(1000);
                } else if (avgFront < OBSTACLE_FRONT_THRESHOLD) {
                    printlnToConsole("Obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    testPath = CHECK_OBSTACLE_TOP2F;
                    delay(1000);
                } else {
                    printlnToConsole("No obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    FORWARD_SECOND_ROW_VALUE = 1.48;
                    testPath = FORWARD_SECOND_ROW;
                    delay(1000);
                }
            }
        break;

        case CHECK_OBSTACLE_TOP2F:
            if (moveStrafeToY(1.0, 0.35)) {
                isObstaclePauseActive = false;
                testPath = CHECK_OBSTACLE_TOP2FF;
                delay(1000);
            }
        break;

        case CHECK_OBSTACLE_TOP2FF:
            // adjusting due to strafe drift
            if (turnToAngleSmall(0, 0.25)) {
                testPath = CHECK_OBSTACLE_TOP2FFF;
            }
        break;

        case CHECK_OBSTACLE_TOP2FFF:
            // adjusting due to strafe drift
            if (moveLongToX(0.64, 0.24)) {
                testPath = CHECK_OBSTACLE_TOP2;
                isObstaclePauseActive = false;
            }
        break;

        case CHECK_OBSTACLE_TOP2:
            if (!isObstaclePauseActive) {
                isObstaclePauseActive = true;
                obstaclePauseStartTime = now;
                printlnToConsole("Pausing to check for obstacles...");
            } else if (now - obstaclePauseStartTime >= OBSTACLE_PAUSE_DURATION) {
                float avgFront = getFrontDistanceAverage(OBSTACLE_SAMPLE_COUNT, OBSTACLE_SAMPLE_DELAY);
                isObstaclePauseActive = false;
                if (avgFront < 0) {
                    printlnToConsole("Front sensor readings invalid; assuming clear path");
                    testPath = FORWARD_SECOND_ROW;
                    delay(1000);
                } else if (avgFront < OBSTACLE_FRONT_THRESHOLD) {
                    printlnToConsole("Obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    testPath = CHECK_OBSTACLE_TOP3F;
                    delay(1000);
                } else {
                    printlnToConsole("No obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    testPath = FORWARD_SECOND_ROW;
                    delay(1000);
                }
            }
        break;

        case CHECK_OBSTACLE_TOP3F:
            if (moveStrafeToY(0.5, 0.41)) {
                isObstaclePauseActive = false;
                testPath = CHECK_OBSTACLE_TOP3FF;
                delay(1000);
            }
        break;

        case CHECK_OBSTACLE_TOP3FF:
            if (turnToAngleSmall(0, 0.25)) {
                isObstaclePauseActive = false;
                testPath = CHECK_OBSTACLE_TOP3FFF;
                delay(1000);
            }
        break;

        case CHECK_OBSTACLE_TOP3FFF:
            // go past the first row of obstacles
            if (moveLongToX(1.45, 0.25)) {
                testPath = MOVE_TO_SECOND_ROW_TOP_HALF;
                delay(1000);
            }
        break;

        case MOVE_TO_SECOND_ROW_TOP_HALF:
            // go half the way, then adjust angle
            if (moveStrafeToY(0.95, 0.35)) {
                testPath = MOVE_TO_SECOND_ROW_TOP_HALF_R;
                delay(1000);
            }
        break;

        case MOVE_TO_SECOND_ROW_TOP_HALF_R:
            // go half the way, then adjust angle
            if (turnToAngleSmall(0, 0.25)) {
                testPath = MOVE_TO_SECOND_ROW_TOP;
                delay(1000);
            }
        break;

        case FORWARD_SECOND_ROW:
            // go past the first row of obstacles
            if (moveLongToX(FORWARD_SECOND_ROW_VALUE, 0.25)) {
                testPath = MOVE_TO_SECOND_ROW_TOP;
                delay(1000);
            }
        break;

        case MOVE_TO_SECOND_ROW_TOP:
            // go to the top of the second row
            if (moveStrafeToY(1.5, 0.35)) {
                testPath = MOVE_TO_SECOND_ROW_TOP_ROTATE;
                delay(1000);
            }
        break;

        case MOVE_TO_SECOND_ROW_TOP_ROTATE:
            // go to the top of the second row
            if (turnToAngleSmall(0, 0.25)) {
                testPath = MOVE_TO_SECOND_ROW_TOP_ADJUST;
                delay(1000);
            }
        break;

        case MOVE_TO_SECOND_ROW_TOP_ADJUST:
            // go to the top of the second row
            if (moveLongToX(1.57, 0.25)) {
                isObstaclePauseActive = false;
                testPath = SECOND_ROW_TOP;
                delay(1000);
            }
        break;

        case SECOND_ROW_TOP:
            // check for obstacles
            if (!isObstaclePauseActive) {
                isObstaclePauseActive = true;
                obstaclePauseStartTime = now;
                printlnToConsole("Pausing to check for obstacles...");
            } else if (now - obstaclePauseStartTime >= OBSTACLE_PAUSE_DURATION) {
                float avgFront = getFrontDistanceAverage(OBSTACLE_SAMPLE_COUNT, OBSTACLE_SAMPLE_DELAY);
                isObstaclePauseActive = false;
                if (avgFront < 0) {
                    printlnToConsole("Front sensor readings invalid; assuming clear path");
                    testPath = FORWARD_THIRD_ROW;
                    delay(1000);
                } else if (avgFront < OBSTACLE_FRONT_THRESHOLD) {
                    printlnToConsole("Obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    testPath = SECOND_ROW_MIDDLEF;
                    delay(1000);
                } else {
                    printlnToConsole("No obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    testPath = FORWARD_THIRD_ROW;
                    delay(1000);
                }
            }
        break;

        case SECOND_ROW_MIDDLEF:
            // strafing to the second row to detect the middle obstacle
            if (moveStrafeToY(1.0, 0.35)) {
                testPath = SECOND_ROW_MIDDLEFF;
                delay(1000);
            }
        break;

        case SECOND_ROW_MIDDLEFF:
            // strafing to the second row to detect the middle obstacle
            if (turnToAngleSmall(0, 0.25)) {
                testPath = SECOND_ROW_MIDDLEFFF;
                delay(1000);
            }
        break;

        case SECOND_ROW_MIDDLEFFF:
            // strafing to the second row to detect the middle obstacle
            if (moveLongToX(1.64, 0.24)) {
                isObstaclePauseActive = false;
                testPath = SECOND_ROW_MIDDLE;
                delay(1000);
            }
        break;

        case SECOND_ROW_MIDDLE:
            // check for obstacles
            if (!isObstaclePauseActive) {
                isObstaclePauseActive = true;
                obstaclePauseStartTime = now;
                printlnToConsole("Pausing to check for obstacles...");
            } else if (now - obstaclePauseStartTime >= OBSTACLE_PAUSE_DURATION) {
                float avgFront = getFrontDistanceAverage(OBSTACLE_SAMPLE_COUNT, OBSTACLE_SAMPLE_DELAY);
                isObstaclePauseActive = false;
                if (avgFront < 0) {
                    printlnToConsole("Front sensor readings invalid; assuming clear path");
                    testPath = FORWARD_THIRD_ROW;
                    delay(1000);
                } else if (avgFront < OBSTACLE_FRONT_THRESHOLD) {
                    printlnToConsole("Obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    testPath = SECOND_ROW_BOTTOMF;
                    delay(1000);
                } else {
                    printlnToConsole("No obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    testPath = FORWARD_THIRD_ROW;
                    delay(1000);
                }
            }
        break;

        case SECOND_ROW_BOTTOMF:
            // strafing to the third row
            if (moveStrafeToY(0.5, 0.35)) {
                testPath = SECOND_ROW_BOTTOMFF;
                delay(1000);
            }
        break;

        case SECOND_ROW_BOTTOMFF:
            // strafing to the third row
            if (turnToAngleSmall(0, 0.25)) {
                isObstaclePauseActive = false;
                testPath = FORWARD_THIRD_ROW;
                delay(1000);
            }
        break;

        case FORWARD_THIRD_ROW:
            // going forward past the second row of obstacles
            if (moveLongToX(2.6, 0.25)) {
                testPath = GO_TO_LIMBO;
            }
        break;

        case GO_TO_LIMBO:
            // strafing to the limbo area
            if (moveStrafeToY(1.54, 0.35)) {
                testPath = GO_TO_LIMBO_ROTATE;
                delay(1000);
            }
        break;

        case GO_TO_LIMBO_ROTATE:
            // adjusting angle
            if (turnToAngleSmall(0, 0.25)) {
                testPath = GO_UNDER_LIMBO;
                delay(1000);
            }
        break;

        case GO_UNDER_LIMBO:
            // going underneath limbo
            if (moveLongToX(3.75, 0.25)) {
                printlnToConsole("DONE!");
            }
        break;

        case BOTTOM1:
            // going to mission site - turning around to face it
            moveWithDelay(0, 0, 0.27, 2900, 1000);
            printlnToConsole("turning at smaller speed");
            testPath = BOTTOM2;
        break;
        case BOTTOM2:
            // going to mission site - fine tuning theta
            if (turnToAngleSmall(1.5708, 0.25, 220)) {
                printlnToConsole("moving to mission site");
                testPath = BOTTOM3;
                delay(1000);
            }
        break;
        case BOTTOM3:
            // going to mission site - moving forward to get closer to mission site
            if (moveLongToY(1.01, 0.24)) {
                delay(1000);
                testPath = BOTTOM4;
            }
        break;
        case BOTTOM4:
            // going to mission site - strafing to align limit switches
            if (moveStrafeToX(0.32, 0.3)) {
                delay(1000);
                testPath = BOTTOM5;
            }
        break;
        case BOTTOM5:
            // going to mission site - fine tuning theta after strafing
            if (turnToAngleSmall(1.5708, 0.25)) {
                testPath = TOP2222;
                delay(1000);
            }
        break;
        case BOTTOM6:
            // going to obstacles - moving backwards and turning to face obstacles
            if (moveLongToY(1.0, 0.25)) {
                // turn angle
                moveWithDelay(0, 0, 0.25, 1300, 1000);
                printlnToConsole("turning to face obstacles");
                testPath = BOTTOM7;
            }
        break;
        case BOTTOM7:
            // going to obstacles - fine tuning theta
            if (turnToAngleSmall(0, 0.29)) {
                testPath = BOTTOM8;
                delay(1000);
            }
        break;
        case BOTTOM8:
            // going to obstacles - x value
            if (moveLongToX(0.54, 0.27)) {
                testPath = TOP44;
                delay(1000);
            }
        break;
    }
}
