#ifndef MAZESOLVER_H
#define	MAZESOLVER_H

#include <xc.h>

#define N 6
#define MAX_STEPS N*N

enum Turn {
    NO_TURN = -1,
    TURN_LEFT = 0,
    TURN_RIGHT = 1,
    TURN_BACK = 2
};

enum Status {
    OK = 0,
    REPLAN = 1,
    TARGET = 2,
    FAIL = 3
};

typedef enum {
    UP = 0,
    RIGHT,
    DOWN,
    LEFT
} Direction;

typedef struct Target_ {
    int count;
    int rows[4];
    int cols[4];
} Target;


typedef struct Command_ {
    int dir;
    int cells;
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
    unsigned char visited[N][N];
} Mouse;


typedef struct Point_ {
    int row, col;
} Point;

// ring array based queue
typedef struct Queue_ {
    Point data[N * N];
    int head, tail;
} Queue;

/**
 * Top level maze solver:
 * 1. Discovers maze
 * 2. Returns to start position
 * 3. Calculates optimal path 
 * 4. Execute optimal goal reaching path
 */
uint8_t solveMaze();

#endif	/* MAZESOLVER_H */

