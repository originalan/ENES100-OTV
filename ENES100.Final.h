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
// PIN DEFINITIONS
// ============================================================================

const int ROOM_NUMBER = 1120; // 1120

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
const uint8_t ARUCO_MARKER_ID = 67;

// ============================================================================
// CONSTANTS
// ============================================================================

// Motor identifiers and directions
enum MotorID { FR = 1, FL = 2, BR = 3, BL = 4 };
enum MotorDirection { FORWARD = 1, BACKWARD = 2, OFF = 3 };

// Navigation thresholds
const float OBSTACLE_FRONT_THRESHOLD = 33.0f;  // cm
const float OBSTACLE_SIDE_THRESHOLD = 15.0f;   // cm
const double POSITION_THRESHOLD = 0.06;        // meters (~6 cm)
const double ANGLE_THRESHOLD = 0.09;           // radians (~5 degrees)

// Timing constants
const double HOLD_TIME = 1000.0;               // ms
const unsigned long POSE_TIMEOUT = 250;        // ms
const unsigned long OBSTACLE_PAUSE_DURATION = 1000;  // ms
const unsigned long OBSTACLE_SAMPLE_DELAY = 0; // ms

// Speed limits
const double MAX_LINEAR_SPEED = 0.5;
const double MAX_ROTATIONAL_SPEED = 0.5;

// Obstacle detection parameters
const uint8_t OBSTACLE_SAMPLE_COUNT = 8;

// Movement timing constants
const unsigned long STRAFE_DELAY = 250;          // ms
const unsigned long TURN_DELAY = 800;            // ms
const unsigned long GENERAL_DELAY = 1000;        // ms

// ============================================================================
// STRUCTURES
// ============================================================================

struct TimedDrive {
    double vx;
    double vy;
    double vr;
    unsigned long duration;
    bool done;
};

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
    double x;
    double y;
    double theta;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

bool ENES_INIT = false;
bool SERIAL_INIT = true;
Pose lastPose = {0, 0, 0, 0};
bool poseValid = false;

// Ultrasonic sensor instances
Ultrasonic ultrasonicFront = {US_FRONT_TRIG, US_FRONT_ECHO, -1.0f, 0};
Ultrasonic ultrasonicRight = {US_RIGHT_TRIG, US_RIGHT_ECHO, -1.0f, 0};
Ultrasonic ultrasonicLeft = {US_LEFT_TRIG, US_LEFT_ECHO, -1.0f, 0};

// Timing and state variables
unsigned long lastTriggerTime = 0;
unsigned long driveUntil = 0;
TimedDrive currentTimedDrive = {0, 0, 0, 0, true};
unsigned long obstaclePauseStartTime = 0;
bool isObstaclePauseActive = false;

// Mission state
int candleCount = 0;

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

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

void initWIFI() {
    delay(1000);
    Enes100.begin("Smoke Signal", FIRE, ARUCO_MARKER_ID, ROOM_NUMBER, WIFI_TX, WIFI_RX);
    ENES_INIT = true;
}

// ============================================================================
// POSE/NAVIGATION FUNCTIONS
// ============================================================================

Pose getArucoMarkerPose() {
    Pose pose;
    if (Enes100.isVisible()) {
        pose = {Enes100.getX(), Enes100.getY(), Enes100.getTheta(), millis()};
        lastPose = pose;
        poseValid = true;
    } else {
        pose = lastPose;
        unsigned long age = millis() - lastPose.timestamp;
        poseValid = (age < POSE_TIMEOUT);
    }
    return pose;
}

// ============================================================================
// ULTRASONIC SENSOR FUNCTIONS
// ============================================================================

float readUltrasonic(uint8_t trig, uint8_t echo) {
    // Ensure trigger starts LOW
    digitalWrite(trig, LOW);
    delayMicroseconds(2);

    // 10µs HIGH pulse to trigger
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // Wait for echo (timeout 35ms = ~6m max range)
    long duration = pulseIn(echo, HIGH, 35000);

    if (duration == 0) return -1.0f;

    // Convert to distance in cm (speed of sound ≈ 0.0343 cm/µs)
    return (duration * 0.0343f) / 2.0f;
}

float getFrontDistance() { return ultrasonicFront.distance; }
float getLeftDistance() { return ultrasonicLeft.distance; }
float getRightDistance() { return ultrasonicRight.distance; }

float getFrontDistanceAverage(uint8_t samples, unsigned long sampleDelayMs) {
    if (samples == 0) return -1.0f;

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
    if (validSamples == 0) return -1.0f;
    return sum / static_cast<float>(validSamples);
}

float getRightDistanceAverage(uint8_t samples, unsigned long sampleDelayMs) {
    if (samples == 0) return -1.0f;

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
    if (validSamples == 0) return -1.0f;
    return sum / static_cast<float>(validSamples);
}

float getLeftDistanceAverage(uint8_t samples, unsigned long sampleDelayMs) {
    if (samples == 0) return -1.0f;

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
    if (validSamples == 0) return -1.0f;
    return sum / static_cast<float>(validSamples);
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

void setMotorSpeed(MotorID motorNumber, double normalized) {
    // Clamp to [-1, 1]
    normalized = constrain(normalized, -1.0, 1.0);

    // Determine direction
    MotorDirection direction = OFF;
    if (normalized > 0.0) {
        direction = FORWARD;
    } else if (normalized < 0.0) {
        direction = BACKWARD;
    }

    // Convert to PWM value (0-255)
    uint8_t pwm = (uint8_t)(fabs(normalized) * 255.0);

    // Set direction pins
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

void mecanumDrive(double vx, double vy, double vr) {
    // Mecanum wheel kinematics
    double fr = vx - vy - vr;
    double fl = vx + vy + vr;
    double br = vx + vy - vr;
    double bl = vx - vy + vr;

    // Normalize if any wheel exceeds max speed
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
// NAVIGATION FUNCTIONS (FIXED BUGS)
// ============================================================================

// Turn direction: true = clockwise, false = counter-clockwise
bool isTurningClockwise = false;
void setTurnDirection(double thetaTarget) {
    double error = thetaTarget - Enes100.getTheta();
    isTurningClockwise = (error < 0);
}
bool turnToAngleSmall(double thetaTarget, double power) {
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
bool turnToAngleSmall(double thetaTarget, double power, int delayMS) {
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

bool turnToAngle(double thetaTarget, double power, bool direction) {
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

bool moveLongToX(double targetX, double power) {

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
bool moveLongToY(double targetY, double power) {

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
bool moveStrafeToX(double targetX, double power) {

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
bool moveStrafeToRightSensor(double targetDistanceRight, double power) {

    Pose p = getArucoMarkerPose();
    float right = getRightDistanceAverage(5, 0);
    power = constrain(power, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

    double errorX = (right - targetDistanceRight);
    double distanceX = fabs(errorX);

    if (distanceX < 1.0) {
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
bool moveStrafeToRightSensor(double targetDistanceRight, double power, int delayMS) {

    Pose p = getArucoMarkerPose();
    float right = getRightDistanceAverage(5, 0);
    power = constrain(power, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

    double errorX = (right - targetDistanceRight);
    double distanceX = fabs(errorX);

    if (distanceX < 1.5) {
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
bool moveStrafeToLeftSensor(double targetDistanceLeft, double power) {

    Pose p = getArucoMarkerPose();
    float left = getLeftDistanceAverage(5, 0);
    power = constrain(power, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

    double errorX = (left - targetDistanceLeft);
    double distanceX = fabs(errorX);

    if (distanceX < 1.0) {
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
bool moveStrafeToY(double targetY, double power) {

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

int checkLimitSwitches() {
    bool leftPressed = (digitalRead(LIMIT_LEFT) == LOW);
    bool rightPressed = (digitalRead(LIMIT_RIGHT) == LOW);

    if (leftPressed && rightPressed) return TOP_C;
    if (leftPressed) return TOP_A;
    if (rightPressed) return TOP_B;
    return -1;
}

bool updateFlameSensor() {
    int sensorValue = digitalRead(THERMISTOR_PIN);

    // Flame is present when sensor reads LOW
    return (sensorValue == LOW);
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void printe(String msg) {
    if (ENES_INIT) {
        Enes100.print(msg);
    }
    if (SERIAL_INIT) {
        Serial.print(msg);
    }
}

void printeln(String msg) {
    if (ENES_INIT) {
        Enes100.println(msg);
    }
    if (SERIAL_INIT) {
        Serial.println(msg);
    }
}

void testEachMotor(unsigned long duration, double speed) {
    MotorID motors[4] = {FR, FL, BR, BL};
    String motorNames[4] = {"Front Right", "Front Left", "Back Right", "Back Left"};

    for (int i = 0; i < 4; i++) {
        printeln("Testing " + motorNames[i] + " forward");
        setMotorSpeed(motors[i], speed);
        delay(duration);
        turnOffMotors();
        delay(500);

        printeln("Testing " + motorNames[i] + " backward");
        setMotorSpeed(motors[i], -speed);
        delay(duration);
        turnOffMotors();
        delay(500);
    }
    printeln("Motor test complete!");
}

// ============================================================================
// MAIN FUNCTIONS
// ============================================================================

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

    BOTTOM1, BOTTOM2, BOTTOM3, BOTTOM4, BOTTOM5, BOTTOM6,BOTTOM7,BOTTOM8,BOTTOM9,BOTTOM10,BOTTOM11,BOTTOM12 };
PathState testPath = START1;

enum TestPIDState { START_PID, SCAN_PID, MOVE1_PID, MOVE2_PID };
TestPIDState pidState = START_PID;

TimedDrive path[] = {
    {0.2, 0, 0, 2000, false},
    {0, 0, 0.2, 2000, false},
    {0.2, 0, 0, 2000, false},
    {0, 0.2, 0, 2000, false},
    {0, 0, 0, 2000, false}
};
const int numSteps = sizeof(path) / sizeof(path[0]);
int currentStep = 0;
bool printed = false;

void setup() {
    ENES_INIT = false;  // FIXED: was set to true before initWIFI()
    initWIFI();
    if (SERIAL_INIT) {
        Serial.begin(9600);   
    }
    printeln("Initializing...");

    initializeMotors();
    initSensors();
    candleCount = 0;

    printeln("Ready!");
    printeln("Wait 1 second...");
    delay(1000);
    printeln("GO");
}

void testFlameSensor() {
    unsigned long now = millis();

    int sensorValue = digitalRead(THERMISTOR_PIN);
    if (sensorValue == HIGH) {
        printeln("No, Flame Not Detected");
    }else {
        printeln("Yes, Flame Detected");
    }

    delay(50);
}

void testLimitSwitches() {
    bool leftPressed = (digitalRead(LIMIT_LEFT) == LOW);
    bool rightPressed = (digitalRead(LIMIT_RIGHT) == LOW);

    printeln("limit switch left: " + String(leftPressed));
    printeln("limit switch right: " + String(rightPressed));
}

void testFan() {
    printeln("FAN ON");
    digitalWrite(FAN_PIN, HIGH);
    delay(3000);
    printeln("FAN OFF");
    digitalWrite(FAN_PIN, LOW);
    delay(3000);
}

void testSideSensors() {
    float right = readUltrasonic(ultrasonicRight.trig, ultrasonicRight.echo);
    float left = readUltrasonic(ultrasonicLeft.trig, ultrasonicLeft.echo);
    printeln(String(left) + ", " + String(right));
}

bool isTopStartingPosition = true;
void loop() {
    unsigned long now = millis();

    if (testPath == START1) {
        if (!printed) {
                printeln("Scanning for ArUco marker...");
                printed = true;
            }
            if (Enes100.isVisible()) {
                printeln("ArUco Marker Detected");
                testPath = MOVE;
            }
    } else if (testPath == MOVE) {
        Pose pose = getArucoMarkerPose();
        if (pose.theta > 0) {
            printeln("Detected OTV at TOP");
            printeln("Turning to face mission site");
            isTopStartingPosition = true;
            testPath = TOPL;
        }else {
            printeln("Detected OTV at BOTTOM.");
            printeln("Turning to face mission site");
            isTopStartingPosition = false;
            testPath = BOTTOM1;
        }
    } else if (testPath == TOPL) {
        // going to mission site - turning around to face it
        mecanumDrive(0, 0, 0.27);
        delay(2900);
        turnOffMotors();
        delay(1000);
        printeln("turning at smaller speed");
        setTurnDirection(-1.5708);
        delay(1000);
        testPath = TOPL2;
    } else if (testPath == TOPL2) {
        // going to mission site - fine tuning theta
        if (turnToAngleSmall(-1.5708, 0.25, 220)) {
            printeln("moving to mission site");
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
            setTurnDirection(-1.5708);
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
            if (moveStrafeToRightSensor(34.5, 0.32)) {
                testPath = CHECK_MISSION_SITE_DISTANCE;
                setTurnDirection(-1.5708);
                delay(1000);
            }
        }else {
            if (moveStrafeToLeftSensor(45.0, 0.32)) {
                testPath = CHECK_MISSION_SITE_DISTANCE;
                setTurnDirection(1.5458);
                delay(1000);
            }
        }
    }
    
    else if (testPath == CHECK_MISSION_SITE_DISTANCE) {
        // mission site - ramming into mission site for limit switches
        double dir = (isTopStartingPosition == true) ? -1.5708 : 1.5458;
        if (turnToAngleSmall(dir, 0.25)) {
            delay(1000);
            mecanumDrive(0.27, 0, 0); 
            delay(1000);
            turnOffMotors();
            delay(5000); // 5 second delay
            int detection = checkLimitSwitches();
            if (detection != -1) {
                Enes100.mission(TOPOGRAPHY, detection);
                printeln("TOPOLOGY: " + String(detection));
            }else {
                printeln("Invalid topology detected");
            }
            testPath = CANDLE1;
        }
    }else if (testPath == CANDLE1) {
        // mission site - successfully aligned to candle1
        delay(1000);
        bool flamePresent = updateFlameSensor();
        if (flamePresent) {
            candleCount++;
            digitalWrite(FAN_PIN, HIGH);
            delay(3000);
            digitalWrite(FAN_PIN, LOW);
            delay(1000);
        }
        printeln("Num Candles: " + String(candleCount));
        testPath = CANDLE2;
    }else if (testPath == CANDLE2) {
        // mission site - successfully aligned to candle2
        mecanumDrive(-0.25, 0, 0);
        delay(700);
        turnOffMotors();
        delay(1000);
        bool flamePresent = updateFlameSensor();
        if (flamePresent) {
            candleCount++;
            digitalWrite(FAN_PIN, HIGH);
            delay(3000);
            digitalWrite(FAN_PIN, LOW);
            delay(1000);
        }
        printeln("Num Candles: " + String(candleCount));
        testPath = CANDLE3;
    }else if (testPath == CANDLE3) {
        // mission site - aligning to candle3
        if (isTopStartingPosition == true) {
            if (moveStrafeToRightSensor(45.0, 0.32, 280)) {
                delay(1000);
                // setTurnDirection(-1.5708);
                isTurningClockwise = false;
                delay(1000);
                testPath = CANDLE3F;
            }
        }else {
            if (moveStrafeToLeftSensor(33.3, 0.32)) {
                delay(1000);
                // setTurnDirection(-1.5708);
                isTurningClockwise = false;
                delay(1000);
                testPath = CANDLE3F;
            }
        }
    }else if (testPath == CANDLE3F) {
        // mission site - successfully aligned to candle3 with fine tuning theta
        double goal = (isTopStartingPosition == true) ? -1.5708 : 1.5458;
        if (turnToAngleSmall(goal, 0.25)) {
            delay(1000);
            mecanumDrive(0.26, 0, 0);
            delay(1100);
            turnOffMotors();
            delay(1000);
            bool flamePresent = updateFlameSensor();
            if (flamePresent) {
                candleCount++;
                digitalWrite(FAN_PIN, HIGH);
                delay(3000);
                digitalWrite(FAN_PIN, LOW);
                delay(1000);
            }
            printeln("Num Candles: " + String(candleCount));
            testPath = CANDLE4;
        }
    }
    
    else if (testPath == CANDLE4) {
        // mission site - successfully aligned to candle4
        mecanumDrive(-0.25, 0, 0);
        delay(700);
        turnOffMotors();
        delay(1000);
        bool flamePresent = updateFlameSensor();
        if (flamePresent) {
            candleCount++;
            digitalWrite(FAN_PIN, HIGH);
            delay(3000);
            digitalWrite(FAN_PIN, LOW);
            delay(1000);
        }
        printeln("Num Candles: " + String(candleCount));
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
                mecanumDrive(0, 0, -0.25);
                delay(1300);
                turnOffMotors();
                delay(1000);
                printeln("turning to face obstacles");
                testPath = TOP4;
                setTurnDirection(0);
                delay(1000);
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
                setTurnDirection(0);
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
                printeln("Pausing to check for obstacles...");
            } else if (now - obstaclePauseStartTime >= OBSTACLE_PAUSE_DURATION) {
                float avgFront = getFrontDistanceAverage(OBSTACLE_SAMPLE_COUNT, OBSTACLE_SAMPLE_DELAY);
                isObstaclePauseActive = false;
                if (avgFront < 0) {
                    printeln("Front sensor readings invalid; assuming clear path");
                    testPath = FORWARD_SECOND_ROW;
                    delay(1000);
                } else if (avgFront < OBSTACLE_FRONT_THRESHOLD) {
                    printeln("Obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    testPath = CHECK_OBSTACLE_TOP2F;
                    delay(1000);
                } else {
                    printeln("No obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
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
                setTurnDirection(0);
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
                printeln("Pausing to check for obstacles...");
            } else if (now - obstaclePauseStartTime >= OBSTACLE_PAUSE_DURATION) {
                float avgFront = getFrontDistanceAverage(OBSTACLE_SAMPLE_COUNT, OBSTACLE_SAMPLE_DELAY);
                isObstaclePauseActive = false;
                if (avgFront < 0) {
                    printeln("Front sensor readings invalid; assuming clear path");
                    testPath = FORWARD_SECOND_ROW;
                    delay(1000);
                } else if (avgFront < OBSTACLE_FRONT_THRESHOLD) {
                    printeln("Obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    testPath = CHECK_OBSTACLE_TOP3F;
                    delay(1000);
                } else {
                    printeln("No obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    testPath = FORWARD_SECOND_ROW;
                    delay(1000);
                }
            }
        break;

        case CHECK_OBSTACLE_TOP3F:
            if (moveStrafeToY(0.5, 0.41)) {
                isObstaclePauseActive = false;
                testPath = CHECK_OBSTACLE_TOP3FF;
                setTurnDirection(0);
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
                setTurnDirection(0);
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
            if (moveLongToX(1.51, 0.25)) {
                testPath = MOVE_TO_SECOND_ROW_TOP;
                delay(1000);
            }
        break;

        case MOVE_TO_SECOND_ROW_TOP:
            // go to the top of the second row
            if (moveStrafeToY(1.5, 0.35)) {
                testPath = MOVE_TO_SECOND_ROW_TOP_ROTATE;
                setTurnDirection(0);
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
                printeln("Pausing to check for obstacles...");
            } else if (now - obstaclePauseStartTime >= OBSTACLE_PAUSE_DURATION) {
                float avgFront = getFrontDistanceAverage(OBSTACLE_SAMPLE_COUNT, OBSTACLE_SAMPLE_DELAY);
                isObstaclePauseActive = false;
                if (avgFront < 0) {
                    printeln("Front sensor readings invalid; assuming clear path");
                    testPath = FORWARD_THIRD_ROW;
                    delay(1000);
                } else if (avgFront < OBSTACLE_FRONT_THRESHOLD) {
                    printeln("Obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    testPath = SECOND_ROW_MIDDLEF;
                    delay(1000);
                } else {
                    printeln("No obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    testPath = FORWARD_THIRD_ROW;
                    delay(1000);
                }
            }
        break;

        case SECOND_ROW_MIDDLEF:
            // strafing to the second row to detect the middle obstacle
            if (moveStrafeToY(1.0, 0.35)) {
                testPath = SECOND_ROW_MIDDLEFF;
                setTurnDirection(0);
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
                printeln("Pausing to check for obstacles...");
            } else if (now - obstaclePauseStartTime >= OBSTACLE_PAUSE_DURATION) {
                float avgFront = getFrontDistanceAverage(OBSTACLE_SAMPLE_COUNT, OBSTACLE_SAMPLE_DELAY);
                isObstaclePauseActive = false;
                if (avgFront < 0) {
                    printeln("Front sensor readings invalid; assuming clear path");
                    testPath = FORWARD_THIRD_ROW;
                    delay(1000);
                } else if (avgFront < OBSTACLE_FRONT_THRESHOLD) {
                    printeln("Obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    testPath = SECOND_ROW_BOTTOMF;
                    delay(1000);
                } else {
                    printeln("No obstacle detected: avg front distance " + String(avgFront, 2) + " cm");
                    testPath = FORWARD_THIRD_ROW;
                    delay(1000);
                }
            }
        break;

        case SECOND_ROW_BOTTOMF:
            // strafing to the third row
            if (moveStrafeToY(0.5, 0.35)) {
                testPath = SECOND_ROW_BOTTOMFF;
                setTurnDirection(0);
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
                setTurnDirection(0);
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
                printeln("DONE!");
            }
        break;

        case BOTTOM1:
            // going to mission site - turning around to face it
            mecanumDrive(0, 0, 0.27);
            delay(2900);
            turnOffMotors();
            delay(1000);
            printeln("turning at smaller speed");
            setTurnDirection(1.5708);
            delay(1000);
            testPath = BOTTOM2;
        break;
        case BOTTOM2:
            // going to mission site - fine tuning theta
            if (turnToAngleSmall(1.5708, 0.25, 220)) {
                printeln("moving to mission site");
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
                setTurnDirection(1.5708);
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
                mecanumDrive(0, 0, 0.25);
                delay(1300);
                turnOffMotors();
                delay(1000);
                printeln("turning to face obstacles");
                testPath = BOTTOM7;
                setTurnDirection(0);
                delay(1000);
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
        case BOTTOM9:
        break;
        case BOTTOM10:
        break;
    }
}
