#pragma once

#include "config.h"

typedef struct Cell_ {
    int wallTop;
    int wallRight;
    int wallBottom;
    int wallLeft;
} Cell;

typedef struct Maze_ {
    Cell cells[N][N];
} Maze;

void maze_init(Maze* maze);
void maze_add_borders(Maze* maze);
void maze_add_walls(Maze* maze);
void maze_print(Maze* maze);
void maze_print_ex(Maze* maze, int row, int col, char c);

void maze_set_wall_top(Maze* maze, int row, int col);
void maze_set_wall_right(Maze* maze, int row, int col);
void maze_set_wall_bottom(Maze* maze, int row, int col);
void maze_set_wall_left(Maze* maze, int row, int col);

int maze_get_wall_top(Maze* maze, int row, int col);
int maze_get_wall_right(Maze* maze, int row, int col);
int maze_get_wall_bottom(Maze* maze, int row, int col);
int maze_get_wall_left(Maze* maze, int row, int col);
