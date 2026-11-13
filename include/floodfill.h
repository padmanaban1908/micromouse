#pragma once
#include <Arduino.h>
#include "config.h"

extern uint8_t mazeWalls[MAZE_SIZE][MAZE_SIZE];
extern uint8_t cellDist[MAZE_SIZE][MAZE_SIZE];
extern int robotX;
extern int robotY;
extern Direction robotDir;

const int DX[4] = {0, 1, 0, -1};
const int DY[4] = {1, 0, -1, 0};
const uint8_t WALL_MASKS[4]     = {WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST};
const uint8_t OPPOSITE_MASKS[4] = {WALL_SOUTH, WALL_WEST, WALL_NORTH, WALL_EAST};

struct CellCoord { int8_t x, y; };
extern CellCoord floodQueue[256];
extern int qHead, qTail;

inline void qPush(int8_t x, int8_t y) {
    floodQueue[qTail] = {x, y};
    qTail = (qTail + 1) % 256;
}

inline CellCoord qPop() {
    CellCoord c = floodQueue[qHead];
    qHead = (qHead + 1) % 256;
    return c;
}

inline bool qIsEmpty() { return qHead == qTail; }

inline void initMaze() {
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            mazeWalls[x][y] = 0;
            if (y == MAZE_SIZE - 1) mazeWalls[x][y] |= WALL_NORTH;
            if (x == MAZE_SIZE - 1) mazeWalls[x][y] |= WALL_EAST;
            if (y == 0)             mazeWalls[x][y] |= WALL_SOUTH;
            if (x == 0)             mazeWalls[x][y] |= WALL_WEST;
        }
    }
    mazeWalls[0][0] |= WALL_EAST;
    mazeWalls[1][0] |= WALL_WEST;
}

inline void floodfill(bool targetIsGoal) {
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

inline bool isAtGoal() {
    return (robotX == 7 || robotX == 8) && (robotY == 7 || robotY == 8);
}

inline bool isAtStart() {
    return (robotX == 0 && robotY == 0);
}
