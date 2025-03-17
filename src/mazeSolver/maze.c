#include <stdio.h>

#include "maze.h"

static void add_vertical_wall(Maze* maze, int x, int y, int length) {
    for (int i = 0; i < length; i++) {
        maze->cells[y + i][x].wallRight = 1;
        maze->cells[y + i][x + 1].wallLeft = 1;
    }
}

static void add_horizontal_wall(Maze* maze, int x, int y, int length) {
    for (int i = 0; i < length; i++) {
        maze->cells[y][x + i].wallBottom = 1;
        maze->cells[y + 1][x + i].wallTop = 1;
    }
}

void maze_init(Maze* maze) {
    // zero all
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            maze->cells[i][j].wallTop = 0;
            maze->cells[i][j].wallRight = 0;
            maze->cells[i][j].wallBottom = 0;
            maze->cells[i][j].wallLeft = 0;
        }
    }
}

void maze_add_borders(Maze* maze) {
    // set borders
    for (int i = 0; i < N; i++) {
        maze->cells[0][i].wallTop = 1;
        maze->cells[N-1][i].wallBottom = 1;
        maze->cells[i][0].wallLeft = 1;
        maze->cells[i][N-1].wallRight = 1;
    }
}

void maze_add_walls(Maze* maze) {
    // add inner walls
    // addressing cells: row, column (Y, X)
    add_vertical_wall(maze, 0, 4, 2);
    add_horizontal_wall(maze, 0, 2, 1);
    add_vertical_wall(maze, 1, 2, 2);
    add_vertical_wall(maze, 3, 2, 1);
    add_horizontal_wall(maze, 2, 3, 2);
    add_horizontal_wall(maze, 2, 1, 2);    
}


void maze_print_ex(Maze* maze, int row, int col, char c) {
    for (int i = 0; i < N; i++) {
        // upper wall
        for (int j = 0; j < N; j++) {
            printf("+");
            if (maze->cells[i][j].wallTop) {
                printf("---");
            } else {
                printf("   ");
            }
        }
        // cell content + vertical walls
        printf("+\n");
        for (int j = 0; j < N; j++) {
            if (maze->cells[i][j].wallLeft) {
                printf("|");
            } else {
                printf(" ");
            }
            if ( i == row && j == col) {
                printf(" %c ", c);
            } else {
                printf("   ");
            }
        }
        if (maze->cells[i][N - 1].wallRight) {
            printf("|");
        } else {
            printf(" ");
        }
        printf("\n");
    }
    // lower wall
    for (int j = 0; j < N; j++) {
        printf("+");
        if (maze->cells[N - 1][j].wallBottom) {
            printf("---");
        } else {
            printf("   ");
        }
    }
    printf("+\n");
}

void maze_print(Maze* maze) {
    maze_print_ex(maze, -1, -1, ' ');
}

void maze_set_wall_top(Maze* maze, int row, int col) {
    maze->cells[row][col].wallTop = 1;
    if ( row > 0 ) maze->cells[row - 1][col].wallBottom = 1;
}
void maze_set_wall_right(Maze* maze, int row, int col) {
    maze->cells[row][col].wallRight = 1;
    if ( col < N - 1 ) maze->cells[row][col + 1].wallLeft = 1;
}
void maze_set_wall_bottom(Maze* maze, int row, int col) {
    maze->cells[row][col].wallBottom = 1;
    if ( row < N - 1 ) maze->cells[row + 1][col].wallTop = 1;
}
void maze_set_wall_left(Maze* maze, int row, int col) {
    maze->cells[row][col].wallLeft = 1;
    if ( col > 0 ) maze->cells[row][col - 1].wallRight = 1;
}

int maze_get_wall_top(Maze* maze, int row, int col) {
    return maze->cells[row][col].wallTop;
}
int maze_get_wall_right(Maze* maze, int row, int col) {
    return maze->cells[row][col].wallRight;
}
int maze_get_wall_bottom(Maze* maze, int row, int col) {
    return maze->cells[row][col].wallBottom;
}
int maze_get_wall_left(Maze* maze, int row, int col) {
    return maze->cells[row][col].wallLeft;
}
