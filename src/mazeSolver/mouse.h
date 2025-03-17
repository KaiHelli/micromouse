#pragma once

#include "config.h"
#include "maze.h"

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

typedef struct Command_ {
    int turn;
} Command;

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

void mouse_init(Mouse* mouse, int row, int col, int dir);
void mouse_add_target(Mouse* mouse, int row, int col);

void mouse_print_maze(Mouse* mouse);
void mouse_print_distances(Mouse* mouse);

void mouse_fill_distances(Mouse* mouse);
int mouse_move_next(Mouse* mouse);
