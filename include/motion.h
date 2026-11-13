#pragma once
#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "imu.h"
#include "tof.h"
#include "floodfill.h"

inline bool updateWallsFromSensors() {
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

inline void advanceStraightOneCell(int cruiseSpeed) {
    long startLeftTicks = encLeftTicks;
    long startRightTicks = encRightTicks;
    float targetHeading = currentYawDeg;
    float lastWallError = 0;

    while (true) {
        updateYaw();

        long avgTicks = ((encLeftTicks - startLeftTicks) + (encRightTicks - startRightTicks)) / 2;
        if (avgTicks >= CELL_TICKS) {
            break;
        }

        readAllToFSensors();

        float steerCorrection = 0;
        bool hasLeft = (tofDistMm[SENS_LEFT] < WALL_SIDE_THRESHOLD_MM);
        bool hasRight = (tofDistMm[SENS_RIGHT] < WALL_SIDE_THRESHOLD_MM);

        if (hasLeft && hasRight) {
            float wallError = (float)tofDistMm[SENS_LEFT] - (float)tofDistMm[SENS_RIGHT];
            steerCorrection = KP_WALL * wallError + KD_WALL * (wallError - lastWallError);
            lastWallError = wallError;
        } else if (hasLeft) {
            float wallError = (float)tofDistMm[SENS_LEFT] - (float)NOMINAL_SIDE_DIST_MM;
            steerCorrection = KP_WALL * wallError * 1.5f;
        } else if (hasRight) {
            float wallError = (float)NOMINAL_SIDE_DIST_MM - (float)tofDistMm[SENS_RIGHT];
            steerCorrection = KP_WALL * wallError * 1.5f;
        } else {
            float headingError = targetHeading - currentYawDeg;
            float gyroRate = readGyroZRate();
            steerCorrection = KP_GYRO * headingError - KD_GYRO * gyroRate;
        }

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
 * 🔄 CONTINUOUS SMOOTH 90° ARC TURN (50% Controlled Speed)
 * ============================================================================
 */
inline void executeSmoothTurn90(bool turnRight) {
    float startHeading = currentYawDeg;

    // Proportional differential wheel speeds at 50% speed
    int outerPwm = SPEED_SMOOTH_TURN + 30;
    int innerPwm = SPEED_SMOOTH_TURN - 35;

    int leftBase  = turnRight ? outerPwm : innerPwm;
    int rightBase = turnRight ? innerPwm : outerPwm;

    float thresholdDeg = 87.5f;

    while (true) {
        updateYaw();
        float turnedAngle = currentYawDeg - startHeading;

        if (turnRight) {
            if (turnedAngle <= -thresholdDeg) break;
        } else {
            if (turnedAngle >= thresholdDeg) break;
        }

        float targetRate = turnRight ? -140.0f : +140.0f;
        float currentRate = readGyroZRate();
        float rateError = targetRate - currentRate;
        float rateCorrection = rateError * 0.25f;

        if (turnRight) {
            setMotorRaw(leftBase + rateCorrection, rightBase - rateCorrection);
        } else {
            setMotorRaw(leftBase - rateCorrection, rightBase + rateCorrection);
        }
        delay(2);
    }

    if (turnRight) {
        robotDir = (Direction)((robotDir + 1) % 4);
    } else {
        robotDir = (Direction)((robotDir + 3) % 4);
    }
}

inline void executeUTurn180() {
    float startHeading = currentYawDeg;
    setMotorRaw(70, -70); // 50% speed U-turn
    while (true) {
        updateYaw();
        if (currentYawDeg - startHeading <= -176.0f) break;
        delay(2);
    }
    stopMotors();
    delay(40);
    robotDir = (Direction)((robotDir + 2) % 4);
}

inline void navigateToNextCell(Direction targetDir, bool useSmoothTurns, int cruiseSpeed) {
    int diff = (targetDir - robotDir + 4) % 4;

    if (diff == 0) {
        advanceStraightOneCell(cruiseSpeed);
    } else if (diff == 1) {
        // Right Turn
        if (useSmoothTurns) {
            executeSmoothTurn90(true);
            advanceStraightOneCell(cruiseSpeed);
        } else {
            stopMotors(); delay(40);
            executeSmoothTurn90(true);
            advanceStraightOneCell(cruiseSpeed);
        }
    } else if (diff == 3) {
        // Left Turn
        if (useSmoothTurns) {
            executeSmoothTurn90(false);
            advanceStraightOneCell(cruiseSpeed);
        } else {
            stopMotors(); delay(40);
            executeSmoothTurn90(false);
            advanceStraightOneCell(cruiseSpeed);
        }
    } else if (diff == 2) {
        // Dead End U-turn
        stopMotors();
        delay(40);
        executeUTurn180();
        advanceStraightOneCell(cruiseSpeed);
    }
}

inline Direction getBestNextDirection() {
    uint8_t minD = 255;
    Direction bestDir = robotDir;

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
