#include "Enes100.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Motor pins - FR (Front Right)
const uint8_t MOTOR1_I1 = 22, MOTOR1_I2 = 23, MOTOR1_ENA = 6;

// Motor pins - BR (Back Right)
const uint8_t MOTOR1_I3 = 24, MOTOR1_I4 = 25, MOTOR1_ENB = 7;

// Motor pins - BL (Back Left)
const uint8_t MOTOR2_I1 = 26, MOTOR2_I2 = 27, MOTOR2_ENA = 8;

// Motor pins - FL (Front Left)
const uint8_t MOTOR2_I3 = 28, MOTOR2_I4 = 29, MOTOR2_ENB = 9;

// Ultrasonic sensors
const uint8_t US_FRONT_ECHO = 40, US_FRONT_TRIG = 41;
const uint8_t US_RIGHT_ECHO = 32, US_RIGHT_TRIG = 33;
const uint8_t US_LEFT_ECHO = 34, US_LEFT_TRIG = 35;

// Limit switches
const uint8_t LIMIT_LEFT = 36, LIMIT_RIGHT = 37;

// Thermistor module
const uint8_t THERMISTOR = 45;
const uint8_t FAN_PIN = 12;

// WiFi module pins
const uint8_t wifiModuleTX = A8, wifiModuleRX = A9, ARUCO_MARKER_ID = 67;

// ============================================================================
// CONSTANTS0oi
// ============================================================================

// Motor identifiers
enum MotorID { FR = 1, FL = 2, BR = 3, BL = 4 };
enum MotorDirection { FORWARD = 1, BACKWARD = 2, OFF = 3 };

// Obstacle avoidance thresholds (cm)
const float FRONT_THRESH = 33.0f;
const float SIDE_THRESH = 15.0f;

// Control parameters
const double HOLD_TIME = 1000.0;  // ms
const unsigned long POSE_TIMEOUT = 250;  // ms

// PID gains
const double LONG_K_P = 0.8;
const double STRAFE_K_P = 0.8;
const double TURN_K_P = 1.2;

// Position/angle thresholds
const double POS_THRESHOLD = 0.06;  // 4 cm
const double ANGLE_THRESHOLD = 0.09;  // radians (~3.5 degrees)

// Speed limits
const double MAX_LINEAR_SPEED = 0.5;
const double MAX_ROTATION_SPEED = 0.5;

// Obstacle pause behavior
const unsigned long OBSTACLE_PAUSE_MS = 1000;
const uint8_t OBSTACLE_SAMPLES = 8;
const unsigned long OBSTACLE_SAMPLE_DELAY_MS = 0;

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
Pose lastPose = {0, 0, 0, 0};
bool poseValid = false;

Ultrasonic usFront = {US_FRONT_TRIG, US_FRONT_ECHO, -1.0f, 0};
Ultrasonic usLeft = {US_LEFT_TRIG, US_LEFT_ECHO, -1.0f, 0};
Ultrasonic usRight = {US_RIGHT_TRIG, US_RIGHT_ECHO, -1.0f, 0};

unsigned long lastTriggerTime = 0;
unsigned long driveUntil = 0;
TimedDrive currentTimedDrive = {0, 0, 0, 0, true};
unsigned long obstaclePauseStart = 0;
bool obstaclePauseActive = false;

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

void initializeMotors() {
    // Set all motor pins as outputs
    pinMode(MOTOR1_I1, OUTPUT); pinMode(MOTOR1_I2, OUTPUT); pinMode(MOTOR1_ENA, OUTPUT);
    pinMode(MOTOR1_I3, OUTPUT); pinMode(MOTOR1_I4, OUTPUT); pinMode(MOTOR1_ENB, OUTPUT);
    pinMode(MOTOR2_I1, OUTPUT); pinMode(MOTOR2_I2, OUTPUT); pinMode(MOTOR2_ENA, OUTPUT);
    pinMode(MOTOR2_I3, OUTPUT); pinMode(MOTOR2_I4, OUTPUT); pinMode(MOTOR2_ENB, OUTPUT);

    // Initialize all motors to OFF
    digitalWrite(MOTOR1_I1, LOW); digitalWrite(MOTOR1_I2, LOW); analogWrite(MOTOR1_ENA, 0);
    digitalWrite(MOTOR1_I3, LOW); digitalWrite(MOTOR1_I4, LOW); analogWrite(MOTOR1_ENB, 0);
    digitalWrite(MOTOR2_I1, LOW); digitalWrite(MOTOR2_I2, LOW); analogWrite(MOTOR2_ENA, 0);
    digitalWrite(MOTOR2_I3, LOW); digitalWrite(MOTOR2_I4, LOW); analogWrite(MOTOR2_ENB, 0);
}

void initSensors() {
    // Ultrasonic sensors
    pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_FRONT_TRIG, OUTPUT);
    pinMode(US_RIGHT_ECHO, INPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT);
    pinMode(US_LEFT_ECHO, INPUT);
    pinMode(US_LEFT_TRIG, OUTPUT);

    // Limit switches (with pull-up)
    pinMode(LIMIT_LEFT, INPUT_PULLUP);
    pinMode(LIMIT_RIGHT, INPUT_PULLUP);

    // Thermistor and fan
    pinMode(THERMISTOR, INPUT_PULLUP);
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, LOW);
}

void initWIFI() {
    Enes100.begin("Smoke Signal", FIRE, ARUCO_MARKER_ID, 1120, wifiModuleTX, wifiModuleRX);
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
        // printeln("DEBUG: ArUco visible, pose=(" + String(pose.x, 3) + ", " + String(pose.y, 3) + ", " + String(pose.theta, 4) + ")");
    } else {
        pose = lastPose;
        unsigned long age = millis() - lastPose.timestamp;
        poseValid = (age < POSE_TIMEOUT);
        // printeln("DEBUG: ArUco not visible, using last pose age=" + String(age) + "ms, valid=" + String(poseValid));
    }
    return pose;
}

// Normalize angle to [-PI, PI]
double normalizeAngle(double angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
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

void updateUltrasonic(Ultrasonic& us, unsigned long now) {
    if (now - us.lastUpdate >= Ultrasonic::UPDATE_INTERVAL) {
        us.distance = readUltrasonic(us.trig, us.echo);
        us.lastUpdate = now;
    }
}

void updateAllUltrasonics(unsigned long now) {
    updateUltrasonic(usFront, now);
    updateUltrasonic(usRight, now);
    updateUltrasonic(usLeft, now);
}

float getFrontDistance() { return usFront.distance; }
float getLeftDistance() { return usLeft.distance; }
float getRightDistance() { return usRight.distance; }

float getFrontDistanceAverage(uint8_t samples, unsigned long sampleDelayMs) {
    if (samples == 0) return -1.0f;

    float sum = 0.0f;
    uint8_t validSamples = 0;
    for (uint8_t i = 0; i < samples; i++) {
        float reading = readUltrasonic(usFront.trig, usFront.echo);
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
        float reading = readUltrasonic(usRight.trig, usRight.echo);
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
            pin1 = MOTOR1_I1; pin2 = MOTOR1_I2; pwmPin = MOTOR1_ENA;
            pin1State = (direction == BACKWARD);
            pin2State = (direction == FORWARD);
            break;
        case BR:
            pin1 = MOTOR1_I3; pin2 = MOTOR1_I4; pwmPin = MOTOR1_ENB;
            pin1State = (direction == FORWARD);
            pin2State = (direction == BACKWARD);
            break;
        case BL:
            pin1 = MOTOR2_I1; pin2 = MOTOR2_I2; pwmPin = MOTOR2_ENA;
            pin1State = (direction == FORWARD);
            pin2State = (direction == BACKWARD);
            break;
        case FL:
            pin1 = MOTOR2_I3; pin2 = MOTOR2_I4; pwmPin = MOTOR2_ENB;
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
// TIMED DRIVE FUNCTIONS
// ============================================================================

void startTimedDrive(TimedDrive& drive) {
    driveUntil = millis() + drive.duration;
    currentTimedDrive = drive;
    currentTimedDrive.done = false;
}

void updateDrive(unsigned long now) {
    if (now < driveUntil) {
        mecanumDrive(currentTimedDrive.vx,
                     currentTimedDrive.vy,
                     currentTimedDrive.vr);
    } else {
        driveUntil = 0;
        currentTimedDrive.done = true;
        turnOffMotors();
    }
}

// ============================================================================
// NAVIGATION FUNCTIONS (FIXED BUGS)
// ============================================================================

// direction = true = CW, false = CCW
bool isCW = false;
void setTurnDirection(double thetaTarget) {
    Pose p = {Enes100.getX(), Enes100.getY(), Enes100.getTheta(), millis()};
    double err = thetaTarget - p.theta;

    if (err < 0) {
        isCW = true;
    }else {
        isCW = false;
    }
}
bool turnToAngleSmall(double thetaTarget, double power) {
    Pose p = {Enes100.getX(), Enes100.getY(), Enes100.getTheta(), millis()};
    double error = fabs(thetaTarget - p.theta);
    if (error < 0.06) {
        return true;
    }
    if (isCW == true) {
        mecanumDrive(0, 0, power);
        delay(150);
        turnOffMotors();
        delay(1200);
    }else {
        mecanumDrive(0, 0, -1.0 * power);
        delay(150);
        turnOffMotors();
        delay(1000);
    }
    return false;
}

// direction = true = CW, false = CCW
bool isForward = false;
void setLongDirection(double thetaTarget) {
    Pose p = {Enes100.getX(), Enes100.getY(), Enes100.getTheta(), millis()};
    double err = thetaTarget - p.theta;

    if (err < 0) {
        isCW = true;
    }else {
        isCW = false;
    }
}
bool moveToLongSmall(double target, double power, bool isX) {
    Pose p = {Enes100.getX(), Enes100.getY(), Enes100.getTheta(), millis()};
    double error = fabs(target - p.x);
    if (error < 0.05) {
        return true;
    }
    if (isCW == true) {
        mecanumDrive(0, 0, power);
        delay(150);
        turnOffMotors();
        delay(1000);
    }else {
        mecanumDrive(0, 0, -1.0 * power);
        delay(200);
        turnOffMotors();
        delay(1000);
    }
    return false;
}

bool turnToAngle(double thetaTarget, double power, bool direction) {
    Pose p = getArucoMarkerPose();
    double error = (thetaTarget - p.theta);

    // printeln("D:poseValid=" + String(poseValid) + ", error=" + String(error, 4) + ", target=" + String(thetaTarget, 4) + ", current=" + String(p.theta, 4));

    if (fabs(error) < ANGLE_THRESHOLD) {
        // mecanumDrive(0, 0, power);
        // delay(450);
        turnOffMotors();
        return true;
    }

    double vr = power;
    if (direction == false) {
        vr = vr * -1.0;
    }
    // if (error > 0) {
    //     vr = power;
    // }else {
    //     vr = -1.0 * power;
    // }
    vr = constrain(vr, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

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

    if (distanceX < POS_THRESHOLD) {
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

    if (distanceY < POS_THRESHOLD) {
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

    if (distanceX < POS_THRESHOLD) {
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

    if (distanceX < 1.5) {
        return true;
    }

    // Determine if the target is in front or behind the robot
    double forwardComponent = errorX;

    // Auto-select forward/backward direction
    double drivePower = (forwardComponent >= 0) ? power : -power;

    if (poseValid) {
        mecanumDrive(0, drivePower, 0);
        delay(200);
        turnOffMotors();
        delay(1000);
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

    if (distanceY < POS_THRESHOLD) {
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

char checkLimitSwitches() {
    bool leftPressed = (digitalRead(LIMIT_LEFT) == LOW);
    bool rightPressed = (digitalRead(LIMIT_RIGHT) == LOW);

    if (leftPressed && rightPressed) return 'C';
    if (leftPressed) return 'A';
    if (rightPressed) return 'B';
    return 'N';
}

bool updateFlameSensor(unsigned long now) {
    int sensorValue = digitalRead(THERMISTOR);

    // flame is present when sensor is HIGH
    if (sensorValue == LOW) {
        lastTriggerTime = now;
    }

    bool fanOn = (now - lastTriggerTime < HOLD_TIME);
    digitalWrite(FAN_PIN, fanOn ? HIGH : LOW);
    return fanOn;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void printe(String msg) {
    if (ENES_INIT) {
        Enes100.print(msg);
    }
    Serial.print(msg);
}

void printeln(String msg) {
    if (ENES_INIT) {
        Enes100.println(msg);
    }
    Serial.println(msg);
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
// WAYPOINT FUNCTIONS
// ============================================================================

const PathSegment defaultTopPath[] = {
    {0.5, 1.5, PI / 2.0}, // rotate to correct heading
    {0.5, 0.6, PI / 2.0} // then travel to mission site
    // {0.5, 0.5, 0.0} // 
};
const size_t defaultTopPathLength = sizeof(defaultTopPath) / sizeof(defaultTopPath[0]);

const PathSegment obstacleTop[] = {
    {0.5, 1.5, PI / 2.0}, // move back to start
    {0.5, 1.5, 0}, // turn forward
    {1.2, 1.5, 0} // facing first "obstacle" if it exists
};
const size_t obstacleTopLength = sizeof(obstacleTop) / sizeof(obstacleTop[0]);

size_t currentPathSegment = 0;
void resetPathProgress() {
    currentPathSegment = 0;
}

// ============================================================================
// MAIN FUNCTIONS
// ============================================================================

enum PathState { START1, MOVE, TOPL, TOPL2,
    CHECK_MISSION_SITE_DISTANCE,

    TOP2, TOP22, TOP222, TOP2222,

    CANDLE1, CANDLE2, CANDLE3, CANDLE4,

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

    BOTTOM1, BOTTOM2, BOTTOM3 };
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
    Serial.begin(9600);
    printeln("Initializing...");

    initializeMotors();
    initSensors();

    printeln("Ready!");
    printeln("Wait 2 seconds...");
    delay(2000);
    printeln("GO");
}

void testMissionSensing() {
    unsigned long now = millis();

    int sensorValue = digitalRead(THERMISTOR);
    if (sensorValue == HIGH) {
        printeln("No, Flame Not Detected");
    }else {
        printeln("Yes, Flame Detected");
    }

    delay(50);
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

    float right = readUltrasonic(usRight.trig, usRight.echo);
    float left = readUltrasonic(usLeft.trig, usLeft.echo);
    printeln(String(left) + ", " + String(right));

    

}

// void loop() {
//     testSideSensors();
// }

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
            testPath = TOPL;
        }else {
            printeln("Detected OTV at BOTTOM. Doing nothing (path not created)");
            testPath = BOTTOM1;
        }
    } else if (testPath == TOPL) {
        // going to mission site by turning around
        mecanumDrive(0, 0, 0.27);
        delay(2900);
        turnOffMotors();
        delay(1000);
        printeln("turning at smaller speed");
        setTurnDirection(-1.5708);
        delay(1000);
        testPath = TOPL2;
    } else if (testPath == TOPL2) {
        // adjusting the turn angle
        if (turnToAngleSmall(-1.5708, 0.25)) {
            printeln("moving to mission site");
            testPath = TOP2;
            delay(1000);
        }
    }else if (testPath == TOP2) {
        // going to mission site
        if (moveLongToY(0.95, 0.24)) {
            delay(1000);
            testPath = TOP22; // do the mission stuff after this step
        }
    }else if (testPath == TOP22) {

        if (moveStrafeToX(0.23, 0.3)) {
            setTurnDirection(-1.5708);
            delay(1000);
            testPath = TOP222;
        }

    }else if (testPath == TOP222) {

        if (turnToAngleSmall(-1.5708, 0.25)) {
            testPath = TOP2222;
            delay(1000);
        }

    }

    else if (testPath == TOP2222) {

        // aligning to right candle(s)
        if (moveStrafeToRightSensor(31.5, 0.31)) {
            testPath = CHECK_MISSION_SITE_DISTANCE;
            delay(1000);
        }

    }
    
    else if (testPath == CHECK_MISSION_SITE_DISTANCE) {
        mecanumDrive(0.38, 0, 0); // ram into mission site
        delay(300);
        turnOffMotors();
        delay(1000); // 2 seconds buffer
        printeln("moving backwards");
        testPath = CANDLE1;
    }else if (testPath == CANDLE1) {

        // updateFlameSensor(now);

        // if (updateFlameSensor)
        testPath = CANDLE2;
    }else if (testPath == CANDLE2) {

        // updateFlameSensor(now);

        mecanumDrive(-0.25, 0, 0);
        delay(750);
        turnOffMotors();
        delay(1000);
        testPath = CANDLE3;
    }else if (testPath == CANDLE3) {
        // aligning to left candle(s)
        if (moveStrafeToRightSensor(45.0, 0.31)) {
            testPath = CANDLE4;
            delay(1000);
            mecanumDrive(0.26, 0, 0);
            delay(1100);
            turnOffMotors();
            delay(1000);
        }
    }else if (testPath == CANDLE4) {

        // updateFlameSensor(now);

        mecanumDrive(-0.25, 0, 0);
        delay(750);
        turnOffMotors();
        delay(1000);
        testPath = TOP3;
    }

    switch(testPath) {
        case TOP3:
            // moving backwards
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
            if (turnToAngleSmall(0, 0.25)) {
                testPath = TOP44;
                delay(1000);
            }
        break;

        case TOP44:
            // adjust coordinate back
            if (moveStrafeToY(1.5, 0.35)) {
                testPath = TOP444;
                setTurnDirection(0);
                delay(1000);
            }
        break;

        case TOP444:
            // adjust coordinate back
            if (turnToAngleSmall(0, 0.25)) {
                testPath = TOP5;
                delay(1000);
            }
        break;

        case TOP5:
            if (moveLongToX(0.58, 0.25)) {
                obstaclePauseActive = false;
                testPath = CHECK_OBSTACLE_TOP1;
                delay(1000);
            }
        break;

        case CHECK_OBSTACLE_TOP1:
            if (!obstaclePauseActive) {
                obstaclePauseActive = true;
                obstaclePauseStart = now;
                printeln("Pausing to check for obstacles...");
            } else if (now - obstaclePauseStart >= OBSTACLE_PAUSE_MS) {
                float avgFront = getFrontDistanceAverage(OBSTACLE_SAMPLES, OBSTACLE_SAMPLE_DELAY_MS);
                obstaclePauseActive = false;
                if (avgFront < 0) {
                    printeln("Front sensor readings invalid; assuming clear path");
                    testPath = FORWARD_SECOND_ROW;
                    delay(1000);
                } else if (avgFront < FRONT_THRESH) {
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
                obstaclePauseActive = false;
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
            if (moveLongToX(0.62, 0.24)) {
                testPath = CHECK_OBSTACLE_TOP2;
                obstaclePauseActive = false;
            }
        break;

        case CHECK_OBSTACLE_TOP2:
            if (!obstaclePauseActive) {
                obstaclePauseActive = true;
                obstaclePauseStart = now;
                printeln("Pausing to check for obstacles...");
            } else if (now - obstaclePauseStart >= OBSTACLE_PAUSE_MS) {
                float avgFront = getFrontDistanceAverage(OBSTACLE_SAMPLES, OBSTACLE_SAMPLE_DELAY_MS);
                obstaclePauseActive = false;
                if (avgFront < 0) {
                    printeln("Front sensor readings invalid; assuming clear path");
                    testPath = FORWARD_SECOND_ROW;
                    delay(1000);
                } else if (avgFront < FRONT_THRESH) {
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
            if (moveStrafeToY(0.5, 0.35)) {
                obstaclePauseActive = false;
                testPath = CHECK_OBSTACLE_TOP3FF;
                setTurnDirection(0);
                delay(1000);
            }
        break;

        case CHECK_OBSTACLE_TOP3FF:
            if (turnToAngleSmall(0, 0.25)) {
                obstaclePauseActive = false;
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
            if (moveLongToX(1.45, 0.25)) {
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
                obstaclePauseActive = false;
                testPath = SECOND_ROW_TOP;
                delay(1000);
            }
        break;

        case SECOND_ROW_TOP:
            // check for obstacles
            if (!obstaclePauseActive) {
                obstaclePauseActive = true;
                obstaclePauseStart = now;
                printeln("Pausing to check for obstacles...");
            } else if (now - obstaclePauseStart >= OBSTACLE_PAUSE_MS) {
                float avgFront = getFrontDistanceAverage(OBSTACLE_SAMPLES, OBSTACLE_SAMPLE_DELAY_MS);
                obstaclePauseActive = false;
                if (avgFront < 0) {
                    printeln("Front sensor readings invalid; assuming clear path");
                    testPath = FORWARD_THIRD_ROW;
                    delay(1000);
                } else if (avgFront < FRONT_THRESH) {
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
            if (moveLongToX(1.57, 0.24)) {
                obstaclePauseActive = false;
                testPath = SECOND_ROW_MIDDLE;
                delay(1000);
            }
        break;

        case SECOND_ROW_MIDDLE:
            // check for obstacles
            if (!obstaclePauseActive) {
                obstaclePauseActive = true;
                obstaclePauseStart = now;
                printeln("Pausing to check for obstacles...");
            } else if (now - obstaclePauseStart >= OBSTACLE_PAUSE_MS) {
                float avgFront = getFrontDistanceAverage(OBSTACLE_SAMPLES, OBSTACLE_SAMPLE_DELAY_MS);
                obstaclePauseActive = false;
                if (avgFront < 0) {
                    printeln("Front sensor readings invalid; assuming clear path");
                    testPath = FORWARD_THIRD_ROW;
                    delay(1000);
                } else if (avgFront < FRONT_THRESH) {
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
                obstaclePauseActive = false;
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
            if (moveStrafeToY(1.55, 0.35)) {
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
        break;
        case BOTTOM2:
        break;
        default:
            // printeln("DEBUG: Unknown testPath value: " + String(testPath));
        break;
    }
}
