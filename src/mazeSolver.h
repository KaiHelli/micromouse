#ifndef MAZESOLVER_H
#define	MAZESOLVER_H

#include <xc.h>

#define N 6
#define MAX_STEPS 50

enum Direction {
    UP = 0,
    RIGHT = 1,
    DOWN = 2,
    LEFT = 3
};

enum Status {
    OK = 0,
    REPLAN = 1,
    TARGET = 2,
    FAIL = 3
};

typedef struct Target_ {
    int count;
    int rows[N * N];
    int cols[N * N];
} Target;

//if turn is 0: move forward
//else turn is +-90: turn left/right
typedef struct Command_ {
    int turn;
} Command;

typedef struct Cell_ {
    int wallTop;
    int wallRight;
    int wallBottom;
    int wallLeft;
} Cell;

typedef struct Maze_ {
    Cell cells[N][N];
} Maze;


typedef struct Mouse_ {
    // memory of the maze
    Maze maze;
    int cell_distances[N][N];
    // current position and direction
    int row, col;
    int dir;
    // target region
    Target target;
    // commands
    int command_count;
    Command commands[N * N];
} Mouse;


typedef struct Point_ {
    int row, col;
} Point;

// ring array based queue
typedef struct Queue_ {
    Point data[N * N];
    int head, tail;
} Queue;

uint8_t dicoverMaze();

uint8_t executeMaze();

#endif	/* MAZESOLVER_H */

