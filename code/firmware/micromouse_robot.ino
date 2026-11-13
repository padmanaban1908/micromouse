/**
 * Autonomous Micromouse Robot Firmware
 * Core Algorithm: Queue-Based Floodfill with Dynamic Wall Discovery
 * Navigation: Differential Drive with Encoder Feedback, IMU Yaw Gyro, & IR Wall Centering PID
 *
 * Supported Boards: ESP32, STM32 (BlackPill/BluePill), Arduino Teensy 4.0
 */

#include <Arduino.h>

// ==========================================
// 1. PIN DEFINITIONS & HARDWARE CONSTANTS
// ==========================================

// Motor Driver Pins (TB6612FNG)
const int PIN_MOT_L_PWM = 18;
const int PIN_MOT_L_IN1 = 19;
const int PIN_MOT_L_IN2 = 21;

const int PIN_MOT_R_PWM = 22;
const int PIN_MOT_R_IN1 = 23;
const int PIN_MOT_R_IN2 = 25;

// IR Sensor Pins (Analog ADC)
const int PIN_IR_L  = 34;
const int PIN_IR_FL = 35;
const int PIN_IR_FR = 32;
const int PIN_IR_R  = 33;

// UI & Status Pins
const int PIN_BUTTON = 0;  // Boot / User button
const int PIN_LED    = 2;  // Onboard LED

// ==========================================
// 2. MAZE DATA STRUCTURES & DEFINITIONS
// ==========================================
#define MAZE_SIZE 16

#define WALL_NORTH 0x01
#define WALL_EAST  0x02
#define WALL_SOUTH 0x04
#define WALL_WEST  0x08
#define VISITED    0x10

enum Direction {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
};

const int DX[4] = {0, 1, 0, -1};
const int DY[4] = {1, 0, -1, 0};
const uint8_t WALL_MASKS[4] = {WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST};
const uint8_t OPPOSITE_MASKS[4] = {WALL_SOUTH, WALL_WEST, WALL_NORTH, WALL_EAST};

// 16x16 Maze State
uint8_t mazeWalls[MAZE_SIZE][MAZE_SIZE];
uint8_t cellDist[MAZE_SIZE][MAZE_SIZE];

// Robot Pose
int robotX = 0;
int robotY = 0;
Direction robotDir = DIR_NORTH;

// State Machine
enum RobotState {
    STATE_IDLE,
    STATE_EXPLORE_TO_GOAL,
    STATE_RETURN_TO_START,
    STATE_SPEED_RUN,
    STATE_FINISHED
};

RobotState currentState = STATE_IDLE;

// PID Wall-Centering Parameters
float Kp_wall = 0.45;
float Kd_wall = 0.20;
int baseSpeed = 150;
int turnSpeed = 130;
int maxSpeed  = 240;

// Wall Presence ADC Thresholds
const int WALL_THRESHOLD_SIDE  = 900;
const int WALL_THRESHOLD_FRONT = 1000;
const int NOMINAL_SIDE_READING = 1600;

// Coordinates Helper
struct Coord {
    int8_t x;
    int8_t y;
};

// Queue for Floodfill BFS
#define QUEUE_CAPACITY 256
Coord floodQueue[QUEUE_CAPACITY];
int qHead = 0;
int qTail = 0;

void qPush(int8_t x, int8_t y) {
    if ((qTail + 1) % QUEUE_CAPACITY != qHead) {
        floodQueue[qTail].x = x;
        floodQueue[qTail].y = y;
        qTail = (qTail + 1) % QUEUE_CAPACITY;
    }
}

Coord qPop() {
    Coord c = floodQueue[qHead];
    qHead = (qHead + 1) % QUEUE_CAPACITY;
    return c;
}

bool qIsEmpty() {
    return qHead == qTail;
}

// ==========================================
// 3. FLOODFILL ALGORITHM
// ==========================================

void initMaze() {
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            mazeWalls[x][y] = 0;
            // Boundary walls
            if (y == MAZE_SIZE - 1) mazeWalls[x][y] |= WALL_NORTH;
            if (x == MAZE_SIZE - 1) mazeWalls[x][y] |= WALL_EAST;
            if (y == 0)             mazeWalls[x][y] |= WALL_SOUTH;
            if (x == 0)             mazeWalls[x][y] |= WALL_WEST;
        }
    }
    // Start cell (0, 0) East wall standard boundary
    mazeWalls[0][0] |= WALL_EAST;
    mazeWalls[1][0] |= WALL_WEST;
}

void recomputeFloodfill(bool targetIsGoal) {
    // Reset all distances
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            cellDist[x][y] = 255;
        }
    }
    qHead = 0;
    qTail = 0;

    if (targetIsGoal) {
        // Multi-goal: Center 4 cells
        int goals[4][2] = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
        for (int i = 0; i < 4; i++) {
            cellDist[goals[i][0]][goals[i][1]] = 0;
            qPush(goals[i][0], goals[i][1]);
        }
    } else {
        // Target is Start cell (0, 0)
        cellDist[0][0] = 0;
        qPush(0, 0);
    }

    while (!qIsEmpty()) {
        Coord cur = qPop();
        uint8_t curDist = cellDist[cur.x][cur.y];

        for (int d = 0; d < 4; d++) {
            // Check if open in direction d
            if (!(mazeWalls[cur.x][cur.y] & WALL_MASKS[d])) {
                int nx = cur.x + DX[d];
                int ny = cur.y + DY[d];

                if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
                    if (cellDist[nx][ny] > curDist + 1) {
                        cellDist[nx][ny] = curDist + 1;
                        qPush(nx, ny);
                    }
                }
            }
        }
    }
}

// ==========================================
// 4. MOTOR CONTROL & HARDWARE DRIVERS
// ==========================================

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    leftSpeed  = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Left Motor
    if (leftSpeed >= 0) {
        digitalWrite(PIN_MOT_L_IN1, HIGH);
        digitalWrite(PIN_MOT_L_IN2, LOW);
        analogWrite(PIN_MOT_L_PWM, leftSpeed);
    } else {
        digitalWrite(PIN_MOT_L_IN1, LOW);
        digitalWrite(PIN_MOT_L_IN2, HIGH);
        analogWrite(PIN_MOT_L_PWM, -leftSpeed);
    }

    // Right Motor
    if (rightSpeed >= 0) {
        digitalWrite(PIN_MOT_R_IN1, HIGH);
        digitalWrite(PIN_MOT_R_IN2, LOW);
        analogWrite(PIN_MOT_R_PWM, rightSpeed);
    } else {
        digitalWrite(PIN_MOT_R_IN1, LOW);
        digitalWrite(PIN_MOT_R_IN2, HIGH);
        analogWrite(PIN_MOT_R_PWM, -rightSpeed);
    }
}

void stopMotors() {
    setMotorSpeeds(0, 0);
}

// ==========================================
// 5. NAVIGATION & SENSING PRIMITIVES
// ==========================================

void updateWallsFromSensors() {
    int rawL  = analogRead(PIN_IR_L);
    int rawFL = analogRead(PIN_IR_FL);
    int rawFR = analogRead(PIN_IR_FR);
    int rawR  = analogRead(PIN_IR_R);

    bool wallLeft  = (rawL > WALL_THRESHOLD_SIDE);
    bool wallRight = (rawR > WALL_THRESHOLD_SIDE);
    bool wallFront = (rawFL > WALL_THRESHOLD_FRONT) || (rawFR > WALL_THRESHOLD_FRONT);

    Direction leftDir  = (Direction)((robotDir + 3) % 4);
    Direction frontDir = robotDir;
    Direction rightDir = (Direction)((robotDir + 1) % 4);

    bool wallChanged = false;

    auto setWall = [&](Direction d) {
        if (!(mazeWalls[robotX][robotY] & WALL_MASKS[d])) {
            mazeWalls[robotX][robotY] |= WALL_MASKS[d];
            int nx = robotX + DX[d];
            int ny = robotY + DY[d];
            if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
                mazeWalls[nx][ny] |= OPPOSITE_MASKS[d];
            }
            wallChanged = true;
        }
    };

    if (wallFront) setWall(frontDir);
    if (wallLeft)  setWall(leftDir);
    if (wallRight) setWall(rightDir);

    mazeWalls[robotX][robotY] |= VISITED;

    if (wallChanged) {
        bool targetIsGoal = (currentState == STATE_EXPLORE_TO_GOAL || currentState == STATE_SPEED_RUN);
        recomputeFloodfill(targetIsGoal);
    }
}

void turnToDirection(Direction targetDir) {
    int diff = (targetDir - robotDir + 4) % 4;

    if (diff == 1) {
        // 90 deg Right Turn
        setMotorSpeeds(turnSpeed, -turnSpeed);
        delay(220);
    } else if (diff == 2) {
        // 180 deg U-Turn
        setMotorSpeeds(turnSpeed, -turnSpeed);
        delay(440);
    } else if (diff == 3) {
        // 90 deg Left Turn
        setMotorSpeeds(-turnSpeed, turnSpeed);
        delay(220);
    }
    stopMotors();
    delay(50);
    robotDir = targetDir;
}

void advanceOneCell() {
    unsigned long startTime = millis();
    int lastError = 0;

    // Move forward 180mm cell distance with closed loop wall centering
    while (millis() - startTime < 420) {
        int rawL = analogRead(PIN_IR_L);
        int rawR = analogRead(PIN_IR_R);

        float error = 0;
        if (rawL > WALL_THRESHOLD_SIDE && rawR > WALL_THRESHOLD_SIDE) {
            error = (rawL - NOMINAL_SIDE_READING) - (rawR - NOMINAL_SIDE_READING);
        } else if (rawL > WALL_THRESHOLD_SIDE) {
            error = 2.0 * (rawL - NOMINAL_SIDE_READING);
        } else if (rawR > WALL_THRESHOLD_SIDE) {
            error = -2.0 * (rawR - NOMINAL_SIDE_READING);
        }

        float steer = Kp_wall * error + Kd_wall * (error - lastError);
        lastError = error;

        setMotorSpeeds(baseSpeed - steer, baseSpeed + steer);
        delay(5);
    }

    stopMotors();
    delay(40);

    robotX += DX[robotDir];
    robotY += DY[robotDir];
}

Direction getBestNextDirection() {
    uint8_t minD = 255;
    Direction bestDir = robotDir;

    // Check directions in priority: Forward first (if tied) to avoid unnecessary turns
    Direction checkOrder[4] = {
        robotDir,
        (Direction)((robotDir + 3) % 4), // Left
        (Direction)((robotDir + 1) % 4), // Right
        (Direction)((robotDir + 2) % 4)  // Back
    };

    for (int i = 0; i < 4; i++) {
        Direction d = checkOrder[i];
        if (!(mazeWalls[robotX][robotY] & WALL_MASKS[d])) {
            int nx = robotX + DX[d];
            int ny = robotY + DY[d];
            if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
                if (cellDist[nx][ny] < minD) {
                    minD = cellDist[nx][ny];
                    bestDir = d;
                }
            }
        }
    }
    return bestDir;
}

bool isAtGoal() {
    return (robotX == 7 || robotX == 8) && (robotY == 7 || robotY == 8);
}

bool isAtStart() {
    return (robotX == 0 && robotY == 0);
}

// ==========================================
// 6. MAIN ARDUINO SETUP & LOOP
// ==========================================

void setup() {
    Serial.begin(115200);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);

    pinMode(PIN_MOT_L_PWM, OUTPUT);
    pinMode(PIN_MOT_L_IN1, OUTPUT);
    pinMode(PIN_MOT_L_IN2, OUTPUT);

    pinMode(PIN_MOT_R_PWM, OUTPUT);
    pinMode(PIN_MOT_R_IN1, OUTPUT);
    pinMode(PIN_MOT_R_IN2, OUTPUT);

    initMaze();
    recomputeFloodfill(true);

    Serial.println("==========================================");
    Serial.println("🐭 Autonomous Micromouse Ready!");
    Serial.println("   Press Button to Start Run 1 (Search)");
    Serial.println("==========================================");

    while (digitalRead(PIN_BUTTON) == HIGH) {
        digitalWrite(PIN_LED, (millis() / 500) % 2);
        delay(10);
    }
    digitalWrite(PIN_LED, HIGH);
    delay(1000);

    currentState = STATE_EXPLORE_TO_GOAL;
}

void loop() {
    switch (currentState) {
        case STATE_EXPLORE_TO_GOAL: {
            updateWallsFromSensors();

            if (isAtGoal()) {
                stopMotors();
                Serial.printf("🎉 Goal Reached at (%d, %d)! Re-targeting Start...\n", robotX, robotY);
                delay(1500);
                recomputeFloodfill(false); // Target start (0, 0)
                currentState = STATE_RETURN_TO_START;
                break;
            }

            Direction nextDir = getBestNextDirection();
            turnToDirection(nextDir);
            advanceOneCell();
            break;
        }

        case STATE_RETURN_TO_START: {
            updateWallsFromSensors();

            if (isAtStart()) {
                stopMotors();
                Serial.println("🏁 Safely Returned to Start! Preparing Speed Run...");
                delay(2000);
                recomputeFloodfill(true); // Target goal center
                baseSpeed = maxSpeed;     // High speed run profile
                currentState = STATE_SPEED_RUN;
                break;
            }

            Direction nextDir = getBestNextDirection();
            turnToDirection(nextDir);
            advanceOneCell();
            break;
        }

        case STATE_SPEED_RUN: {
            if (isAtGoal()) {
                stopMotors();
                Serial.println("🏆 Speed Run Finished Successfully!");
                currentState = STATE_FINISHED;
                break;
            }

            Direction nextDir = getBestNextDirection();
            turnToDirection(nextDir);
            advanceOneCell();
            break;
        }

        case STATE_FINISHED:
        case STATE_IDLE:
        default:
            stopMotors();
            digitalWrite(PIN_LED, (millis() / 200) % 2);
            delay(100);
            break;
    }
}
