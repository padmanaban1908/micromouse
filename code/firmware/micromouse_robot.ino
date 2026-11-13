/**
 * ============================================================================
 * 🐭 AUTONOMOUS MICROMOUSE HIGH-PERFORMANCE ESP32 FIRMWARE
 * ============================================================================
 * Hardware Pin Specifications:
 *   - Microcontroller : ESP32 (38-pin DevKit)
 *   - Left Motor      : PWMA=27, AIN1=14, AIN2=26
 *   - Right Motor     : PWMB=32, BIN1=25, BIN2=33
 *   - Encoders        : Left=(36, 39), Right=(34, 35) + 4.7k external pull-ups
 *   - I2C Bus         : SDA=21, SCL=22 (400 kHz Fast I2C)
 *   - 5x ToF XSHUT    : 23, 19, 18, 17, 16 (VL53L0X sensors)
 *                       [0]=23 (Left 90°), [1]=19 (Front-Left 45°),
 *                       [2]=18 (Center 0°), [3]=17 (Front-Right 45°),
 *                       [4]=16 (Right 90°)
 *   - IMU             : MPU-6050 (Direct I2C register access @ 0x68)
 *   - Pushbuttons     : BTN1=4 (Search Run), BTN2=13 (High-Speed Smooth Turn Run)
 *
 * Core Features:
 *   - SMOOTH CONTINUOUS 90° CURVATURE TURNS (Arc turns with zero-stop velocity)
 *   - Full 16x16 Queue-based Floodfill Algorithm (Search & Return-to-Start)
 *   - High-speed MPU-6050 Gyro Yaw Integration & Heading PID Controller
 *   - Closed-loop ToF Distance Wall-Centering Controller
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ============================================================================
// 1. PINOUT CONFIGURATION
// ============================================================================

// Left Motor (TB6612FNG Channel A)
const int PIN_PWMA = 27;
const int PIN_AIN1 = 14;
const int PIN_AIN2 = 26;

// Right Motor (TB6612FNG Channel B)
const int PIN_PWMB = 32;
const int PIN_BIN1 = 25;
const int PIN_BIN2 = 33;

// Optical / Magnetic Encoders (GPI Pins with external 4.7k pullups to 3.3V)
const int PIN_ENC_L_A = 36; // VP
const int PIN_ENC_L_B = 39; // VN
const int PIN_ENC_R_A = 34;
const int PIN_ENC_R_B = 35;

// I2C Bus
const int PIN_I2C_SDA = 21;
const int PIN_I2C_SCL = 22;

// 5x VL53L0X Time-of-Flight XSHUT Shutdown Pins
const int NUM_TOF = 5;
const int TOF_XSHUT_PINS[NUM_TOF] = {23, 19, 18, 17, 16};
// Dynamically assigned I2C addresses at boot
const uint8_t TOF_I2C_ADDRS[NUM_TOF]  = {0x30, 0x31, 0x32, 0x33, 0x34};

// User Control Buttons (Active LOW with external or internal pull-up)
const int PIN_BTN1 = 4;   // Button 1: Start Search Run (Floodfill)
const int PIN_BTN2 = 13;  // Button 2: Start High-Speed Smooth Turn Run

// ============================================================================
// 2. MOTOR PWM SETUP (ESP32 Core 2.x & 3.x Compatibility)
// ============================================================================
const int PWM_FREQ = 20000; // 20 kHz ultrasonic PWM (no audible hum)
const int PWM_RES  = 8;     // 8-bit resolution (0 - 255)
const int CH_MOT_L = 0;
const int CH_MOT_R = 1;

void initMotorPWM() {
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
#else
    ledcSetup(CH_MOT_L, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_PWMA, CH_MOT_L);
    ledcSetup(CH_MOT_R, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_PWMB, CH_MOT_R);
#endif
}

void setMotorRaw(int leftPwm, int rightPwm) {
    leftPwm  = constrain(leftPwm, -255, 255);
    rightPwm = constrain(rightPwm, -255, 255);

    // Left Motor
    if (leftPwm >= 0) {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(PIN_PWMA, leftPwm);
#else
        ledcWrite(CH_MOT_L, leftPwm);
#endif
    } else {
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(PIN_PWMA, -leftPwm);
#else
        ledcWrite(CH_MOT_L, -leftPwm);
#endif
    }

    // Right Motor
    if (rightPwm >= 0) {
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(PIN_PWMB, rightPwm);
#else
        ledcWrite(CH_MOT_R, rightPwm);
#endif
    } else {
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, HIGH);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(PIN_PWMB, -rightPwm);
#else
        ledcWrite(CH_MOT_R, -rightPwm);
#endif
    }
}

void stopMotors() {
    setMotorRaw(0, 0);
}

// ============================================================================
// 3. ENCODER ODOMETRY & HARDWARE INTERRUPTS
// ============================================================================
volatile long encLeftTicks  = 0;
volatile long encRightTicks = 0;

// Odometry calibration constants (Adjust based on gear ratio & wheel diameter)
// Standard: 12 CPR * 30:1 / (PI * 32mm) approx 3.58 ticks/mm -> 180mm = ~644 ticks
const float TICKS_PER_MM = 3.58;
const float CELL_TICKS   = 180.0 * TICKS_PER_MM; // ~644 ticks per cell

void IRAM_ATTR isrEncoderLeft() {
    if (digitalRead(PIN_ENC_L_B)) {
        encLeftTicks++;
    } else {
        encLeftTicks--;
    }
}

void IRAM_ATTR isrEncoderRight() {
    if (digitalRead(PIN_ENC_R_B)) {
        encRightTicks--;
    } else {
        encRightTicks++;
    }
}

void initEncoders() {
    pinMode(PIN_ENC_L_A, INPUT);
    pinMode(PIN_ENC_L_B, INPUT);
    pinMode(PIN_ENC_R_A, INPUT);
    pinMode(PIN_ENC_R_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncoderLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncoderRight, RISING);
}

// ============================================================================
// 4. MPU-6050 FAST I2C GYRO REGISTER ACCESS
// ============================================================================
const uint8_t MPU6050_ADDR = 0x68;
float gyroZOffset = 0.0;
float currentYawDeg = 0.0;
unsigned long lastGyroMicros = 0;

void initMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up MPU6050
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B); // GYRO_CONFIG
    Wire.write(0x08); // Full scale range: +/- 500 deg/s (65.5 LSB/(deg/s))
    Wire.endTransmission(true);

    delay(100);

    // Calibrate stationary gyro bias over 500 samples
    Serial.print("Calibrating MPU6050 Gyro... Keep robot stationary...");
    long sum = 0;
    for (int i = 0; i < 500; i++) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x47); // GYRO_ZOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2, (uint8_t)true);
        if (Wire.available() >= 2) {
            int16_t rawZ = (Wire.read() << 8) | Wire.read();
            sum += rawZ;
        }
        delay(2);
    }
    gyroZOffset = (float)sum / 500.0;
    Serial.println(" Done! Bias: " + String(gyroZOffset));
    lastGyroMicros = micros();
}

float readGyroZRate() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x47); // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2, (uint8_t)true);
    if (Wire.available() >= 2) {
        int16_t rawZ = (Wire.read() << 8) | Wire.read();
        return (float)(rawZ - gyroZOffset) / 65.5; // deg/s
    }
    return 0.0;
}

void updateYaw() {
    unsigned long now = micros();
    float dt = (now - lastGyroMicros) / 1000000.0;
    lastGyroMicros = now;
    if (dt > 0.05) dt = 0.005; // Prevent jump if delayed
    float rate = readGyroZRate();
    currentYawDeg += rate * dt;
}

// ============================================================================
// 5. 5x VL53L0X TOF SENSORS INITIALIZATION & READING
// ============================================================================
Adafruit_VL53L0X tofSensors[NUM_TOF];
uint16_t tofDistMm[NUM_TOF] = {999, 999, 999, 999, 999};

// Distance Sensor Indices
#define SENS_LEFT   0  // Left 90° (Pin 23)
#define SENS_FL     1  // Front-Left 45° (Pin 19)
#define SENS_FRONT  2  // Front Center 0° (Pin 18)
#define SENS_FR     3  // Front-Right 45° (Pin 17)
#define SENS_RIGHT  4  // Right 90° (Pin 16)

// Wall presence detection thresholds (in millimeters)
const uint16_t WALL_SIDE_THRESHOLD_MM  = 130; // Side wall present if < 130mm
const uint16_t WALL_FRONT_THRESHOLD_MM = 150; // Front wall present if < 150mm
const uint16_t NOMINAL_SIDE_DIST_MM    = 84;  // Ideal center distance in 168mm corridor

void initToFSensors() {
    // 1. Reset all ToF sensors by asserting XSHUT LOW
    for (int i = 0; i < NUM_TOF; i++) {
        pinMode(TOF_XSHUT_PINS[i], OUTPUT);
        digitalWrite(TOF_XSHUT_PINS[i], LOW);
    }
    delay(20);

    // 2. Power on each sensor sequentially and assign unique I2C address
    for (int i = 0; i < NUM_TOF; i++) {
        digitalWrite(TOF_XSHUT_PINS[i], HIGH);
        delay(15);

        if (!tofSensors[i].begin(TOF_I2C_ADDRS[i])) {
            Serial.printf("⚠️ ToF Sensor %d (XSHUT Pin %d) failed at 0x%02X!\n",
                          i, TOF_XSHUT_PINS[i], TOF_I2C_ADDRS[i]);
        } else {
            Serial.printf("✅ ToF Sensor %d initialized at address 0x%02X\n",
                          i, TOF_I2C_ADDRS[i]);
        }
        delay(10);
    }
}

void readAllToFSensors() {
    for (int i = 0; i < NUM_TOF; i++) {
        VL53L0X_RangingMeasurementData_t measure;
        tofSensors[i].rangingTest(&measure, false);
        if (measure.RangeStatus != 4) { // 4 = out of range / invalid
            tofDistMm[i] = measure.RangeMilliMeter;
        } else {
            tofDistMm[i] = 999;
        }
    }
}

// ============================================================================
// 6. MAZE DATA STRUCTURE & 16x16 FLOODFILL
// ============================================================================
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
const uint8_t WALL_MASKS[4]     = {WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST};
const uint8_t OPPOSITE_MASKS[4] = {WALL_SOUTH, WALL_WEST, WALL_NORTH, WALL_EAST};

uint8_t mazeWalls[MAZE_SIZE][MAZE_SIZE];
uint8_t cellDist[MAZE_SIZE][MAZE_SIZE];

int robotX = 0;
int robotY = 0;
Direction robotDir = DIR_NORTH;

struct CellCoord { int8_t x, y; };
CellCoord floodQueue[256];
int qHead = 0, qTail = 0;

void qPush(int8_t x, int8_t y) {
    floodQueue[qTail] = {x, y};
    qTail = (qTail + 1) % 256;
}

CellCoord qPop() {
    CellCoord c = floodQueue[qHead];
    qHead = (qHead + 1) % 256;
    return c;
}

bool qIsEmpty() { return qHead == qTail; }

void initMaze() {
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            mazeWalls[x][y] = 0;
            if (y == MAZE_SIZE - 1) mazeWalls[x][y] |= WALL_NORTH;
            if (x == MAZE_SIZE - 1) mazeWalls[x][y] |= WALL_EAST;
            if (y == 0)             mazeWalls[x][y] |= WALL_SOUTH;
            if (x == 0)             mazeWalls[x][y] |= WALL_WEST;
        }
    }
    // Starting cell standard right wall
    mazeWalls[0][0] |= WALL_EAST;
    mazeWalls[1][0] |= WALL_WEST;
}

void floodfill(bool targetIsGoal) {
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            cellDist[x][y] = 255;
        }
    }
    qHead = 0; qTail = 0;

    if (targetIsGoal) {
        int goals[4][2] = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
        for (int i = 0; i < 4; i++) {
            cellDist[goals[i][0]][goals[i][1]] = 0;
            qPush(goals[i][0], goals[i][1]);
        }
    } else {
        cellDist[0][0] = 0;
        qPush(0, 0);
    }

    while (!qIsEmpty()) {
        CellCoord cur = qPop();
        uint8_t cd = cellDist[cur.x][cur.y];

        for (int d = 0; d < 4; d++) {
            if (!(mazeWalls[cur.x][cur.y] & WALL_MASKS[d])) {
                int nx = cur.x + DX[d];
                int ny = cur.y + DY[d];
                if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
                    if (cellDist[nx][ny] > cd + 1) {
                        cellDist[nx][ny] = cd + 1;
                        qPush(nx, ny);
                    }
                }
            }
        }
    }
}

// ============================================================================
// 7. SENSOR-TO-WALL MAPPING
// ============================================================================
bool updateWallsFromSensors() {
    readAllToFSensors();

    bool wallLeft  = (tofDistMm[SENS_LEFT] < WALL_SIDE_THRESHOLD_MM);
    bool wallRight = (tofDistMm[SENS_RIGHT] < WALL_SIDE_THRESHOLD_MM);
    bool wallFront = (tofDistMm[SENS_FRONT] < WALL_FRONT_THRESHOLD_MM) ||
                     (tofDistMm[SENS_FL] < WALL_FRONT_THRESHOLD_MM) ||
                     (tofDistMm[SENS_FR] < WALL_FRONT_THRESHOLD_MM);

    Direction leftDir  = (Direction)((robotDir + 3) % 4);
    Direction frontDir = robotDir;
    Direction rightDir = (Direction)((robotDir + 1) % 4);

    bool wallChanged = false;

    auto markWall = [&](Direction d) {
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

    if (wallFront) markWall(frontDir);
    if (wallLeft)  markWall(leftDir);
    if (wallRight) markWall(rightDir);

    mazeWalls[robotX][robotY] |= VISITED;
    return wallChanged;
}

// ============================================================================
// 8. HIGH-PERFORMANCE MOTION & SMOOTH 90° TURN CONTROLLER
// ============================================================================
// Navigation Speeds
int SPEED_SEARCH_CRUISE = 150;
int SPEED_FAST_CRUISE   = 220;
int SPEED_SMOOTH_TURN   = 160;

// Wall Centering PID Gains
float Kp_wall = 0.50;
float Kd_wall = 0.25;

// Gyro Straight Heading PID Gains
float Kp_gyro = 3.20;
float Kd_gyro = 0.80;

/**
 * Closed-Loop Straight Cell Advance
 * Keeps bot centered between walls and aligned to heading using ToF & Gyro
 */
void advanceStraightOneCell(int cruiseSpeed) {
    long startLeftTicks = encLeftTicks;
    long startRightTicks = encRightTicks;
    float targetHeading = currentYawDeg;
    float lastWallError = 0;
    float lastGyroRate = 0;

    while (true) {
        updateYaw();

        long avgTicks = ((encLeftTicks - startLeftTicks) + (encRightTicks - startRightTicks)) / 2;
        if (avgTicks >= CELL_TICKS) {
            break; // Finished 180mm cell distance
        }

        readAllToFSensors();

        // 1. Wall Centering Error Calculation
        float steerCorrection = 0;
        bool hasLeft = (tofDistMm[SENS_LEFT] < WALL_SIDE_THRESHOLD_MM);
        bool hasRight = (tofDistMm[SENS_RIGHT] < WALL_SIDE_THRESHOLD_MM);

        if (hasLeft && hasRight) {
            float wallError = (float)tofDistMm[SENS_LEFT] - (float)tofDistMm[SENS_RIGHT];
            steerCorrection = Kp_wall * wallError + Kd_wall * (wallError - lastWallError);
            lastWallError = wallError;
        } else if (hasLeft) {
            float wallError = (float)tofDistMm[SENS_LEFT] - (float)NOMINAL_SIDE_DIST_MM;
            steerCorrection = Kp_wall * wallError * 1.5;
        } else if (hasRight) {
            float wallError = (float)NOMINAL_SIDE_DIST_MM - (float)tofDistMm[SENS_RIGHT];
            steerCorrection = Kp_wall * wallError * 1.5;
        } else {
            // No side walls -> Hold precise straight heading via Gyro
            float headingError = targetHeading - currentYawDeg;
            float gyroRate = readGyroZRate();
            steerCorrection = Kp_gyro * headingError - Kd_gyro * gyroRate;
        }

        // Apply differential PWM
        int pwmL = cruiseSpeed - (int)steerCorrection;
        int pwmR = cruiseSpeed + (int)steerCorrection;
        setMotorRaw(pwmL, pwmR);
        delay(2);
    }

    robotX += DX[robotDir];
    robotY += DY[robotDir];
}

/**
 * ============================================================================
 * 🔄 CONTINUOUS SMOOTH 90° ARC TURN (Zero-Stop Turn)
 * ============================================================================
 * Traverses a smooth circular arc of radius R ~ 90mm across the intersection.
 * Maintains forward momentum without coming to a dead stop!
 *
 * @param turnRight : true for 90° Right Turn, false for 90° Left Turn
 */
void executeSmoothTurn90(bool turnRight) {
    float startHeading = currentYawDeg;
    float targetAngleChange = turnRight ? -90.0 : +90.0;
    float targetEndHeading = startHeading + targetAngleChange;

    // Differential wheel bias for circular arc trajectory
    // Outer wheel runs faster, inner wheel runs slower (both forward!)
    int outerPwm = SPEED_SMOOTH_TURN + 55;
    int innerPwm = SPEED_SMOOTH_TURN - 65;

    int leftBase  = turnRight ? outerPwm : innerPwm;
    int rightBase = turnRight ? innerPwm : outerPwm;

    // Pre-turn lead angle threshold to smoothly blend into the next cell
    float thresholdDeg = 87.5;

    while (true) {
        updateYaw();
        float turnedAngle = currentYawDeg - startHeading;

        if (turnRight) {
            if (turnedAngle <= -thresholdDeg) break;
        } else {
            if (turnedAngle >= thresholdDeg) break;
        }

        // Dynamic curvature feedback: maintain consistent turn angular velocity
        float targetRate = turnRight ? -180.0 : +180.0; // deg/s
        float currentRate = readGyroZRate();
        float rateError = targetRate - currentRate;
        float rateCorrection = rateError * 0.35;

        if (turnRight) {
            setMotorRaw(leftBase + rateCorrection, rightBase - rateCorrection);
        } else {
            setMotorRaw(leftBase - rateCorrection, rightBase + rateCorrection);
        }
        delay(2);
    }

    // Update robot logical direction
    if (turnRight) {
        robotDir = (Direction)((robotDir + 1) % 4);
    } else {
        robotDir = (Direction)((robotDir + 3) % 4);
    }
}

/**
 * 180° In-Place U-Turn for Dead Ends
 */
void executeUTurn180() {
    float startHeading = currentYawDeg;
    float targetEndHeading = startHeading - 180.0;

    setMotorRaw(140, -140);
    while (true) {
        updateYaw();
        if (currentYawDeg - startHeading <= -176.0) break;
        delay(2);
    }
    stopMotors();
    delay(40);
    robotDir = (Direction)((robotDir + 2) % 4);
}

/**
 * Path Decision & Action Dispatcher
 */
void navigateToNextCell(Direction targetDir, bool useSmoothTurns, int cruiseSpeed) {
    int diff = (targetDir - robotDir + 4) % 4;

    if (diff == 0) {
        // Going straight -> Continue forward
        advanceStraightOneCell(cruiseSpeed);
    } else if (diff == 1) {
        // 90° Turn Right
        if (useSmoothTurns) {
            executeSmoothTurn90(true);
            advanceStraightOneCell(cruiseSpeed);
        } else {
            stopMotors(); delay(50);
            executeSmoothTurn90(true);
            advanceStraightOneCell(cruiseSpeed);
        }
    } else if (diff == 3) {
        // 90° Turn Left
        if (useSmoothTurns) {
            executeSmoothTurn90(false);
            advanceStraightOneCell(cruiseSpeed);
        } else {
            stopMotors(); delay(50);
            executeSmoothTurn90(false);
            advanceStraightOneCell(cruiseSpeed);
        }
    } else if (diff == 2) {
        // 180° Dead End U-turn
        stopMotors();
        delay(40);
        executeUTurn180();
        advanceStraightOneCell(cruiseSpeed);
    }
}

Direction getBestNextDirection() {
    uint8_t minD = 255;
    Direction bestDir = robotDir;

    // Prioritize straight -> left -> right -> back
    Direction checkOrder[4] = {
        robotDir,
        (Direction)((robotDir + 3) % 4),
        (Direction)((robotDir + 1) % 4),
        (Direction)((robotDir + 2) % 4)
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

// ============================================================================
// 9. HIGH-LEVEL STATE MACHINE
// ============================================================================
enum MouseState {
    STATE_WAIT_USER,
    STATE_SEARCH_RUN,
    STATE_RETURN_TO_START,
    STATE_SPEED_RUN,
    STATE_DONE
};

MouseState mouseState = STATE_WAIT_USER;

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n========================================================");
    Serial.println("🐭 ESP32 MICROMOUSE INITIALIZING WITH CUSTOM PINOUT...");
    Serial.println("========================================================");

    // 1. Initialize Buttons
    pinMode(PIN_BTN1, INPUT_PULLUP);
    pinMode(PIN_BTN2, INPUT_PULLUP);

    // 2. Initialize Motors & Encoders
    initMotorPWM();
    initEncoders();

    // 3. Initialize I2C Bus on SDA=21, SCL=22
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

    // 4. Initialize MPU-6050
    initMPU6050();

    // 5. Initialize 5x VL53L0X ToF Sensors
    initToFSensors();

    // 6. Initialize Maze Data
    initMaze();
    floodfill(true); // Pre-fill distances to center goal

    Serial.println("========================================================");
    Serial.println("READY FOR ACTION:");
    Serial.println("  👉 Press BTN1 (Pin 4)  : Start Search Run (Exploration)");
    Serial.println("  👉 Press BTN2 (Pin 13) : Start Speed Run (SMOOTH 90° TURNS)");
    Serial.println("========================================================");
}

void loop() {
    switch (mouseState) {
        case STATE_WAIT_USER: {
            if (digitalRead(PIN_BTN1) == LOW) {
                delay(300); // Debounce
                Serial.println("🚀 [RUN 1] Starting Autonomous Search Run...");
                mouseState = STATE_SEARCH_RUN;
            } else if (digitalRead(PIN_BTN2) == LOW) {
                delay(300); // Debounce
                Serial.println("⚡ [RUN 2] Starting High-Speed Run with Smooth 90° Turns!");
                mouseState = STATE_SPEED_RUN;
            }
            delay(10);
            break;
        }

        case STATE_SEARCH_RUN: {
            bool wallsChanged = updateWallsFromSensors();
            if (wallsChanged) {
                floodfill(true);
            }

            if (isAtGoal()) {
                stopMotors();
                Serial.printf("🎉 Goal Reached at (%d, %d)! Re-targeting Start...\n", robotX, robotY);
                delay(1500);
                floodfill(false); // Destination is (0, 0)
                mouseState = STATE_RETURN_TO_START;
                break;
            }

            Direction nextDir = getBestNextDirection();
            navigateToNextCell(nextDir, false, SPEED_SEARCH_CRUISE);
            break;
        }

        case STATE_RETURN_TO_START: {
            bool wallsChanged = updateWallsFromSensors();
            if (wallsChanged) {
                floodfill(false);
            }

            if (isAtStart()) {
                stopMotors();
                Serial.println("🏁 Safely Returned to Start! Optimal Map Computed.");
                delay(2000);
                floodfill(true); // Prepare optimal path to goal
                mouseState = STATE_WAIT_USER; // Ready for user to press BTN2 for Speed Run!
                break;
            }

            Direction nextDir = getBestNextDirection();
            navigateToNextCell(nextDir, false, SPEED_SEARCH_CRUISE);
            break;
        }

        case STATE_SPEED_RUN: {
            if (isAtGoal()) {
                stopMotors();
                Serial.println("🏆 Speed Run Finished with Maximum Velocity!");
                mouseState = STATE_DONE;
                break;
            }

            Direction nextDir = getBestNextDirection();
            // Engage Smooth Continuous 90° Turns at high speed!
            navigateToNextCell(nextDir, true, SPEED_FAST_CRUISE);
            break;
        }

        case STATE_DONE:
        default:
            stopMotors();
            delay(100);
            break;
    }
}
