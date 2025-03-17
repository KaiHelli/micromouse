#include <stdio.h>

#include "mouse.h"
#include "queue.h"

static char mouse_directions[] = {'^', '>', 'v', '<'};

void mouse_init(Mouse* mouse, int row, int col, int dir) {
    maze_init(&mouse->maze);
    maze_add_borders(&mouse->maze);
    mouse->row = row;
    mouse->col = col;
    mouse->dir = dir;
    mouse->target.count = 0;
    mouse->command_count = 0;
}

void mouse_add_target(Mouse* mouse, int row, int col) {
    mouse->target.rows[mouse->target.count] = row;
    mouse->target.cols[mouse->target.count] = col;
    mouse->target.count++;
}

void mouse_print_maze(Mouse* mouse) {
    maze_print_ex(&mouse->maze, mouse->row, mouse->col, mouse_directions[mouse->dir]);
}

void mouse_print_distances(Mouse* mouse) {
    for(int i = 0; i < N; i++) {
        for(int j = 0; j < N; j++) {
            printf("%3d ", mouse->cell_distances[i][j]);
        }
        printf("\n");
    }
}

void mouse_fill_distances(Mouse* mouse) {
    Queue q;

    for(int i = 0; i < N; i++) {
        for(int j = 0; j < N; j++) {
            mouse->cell_distances[i][j] = -1;
        }
    }

    queue_init(&q);
    for(int i = 0; i < mouse->target.count; i++) {
        mouse->cell_distances[mouse->target.rows[i]][mouse->target.cols[i]] = 0;
        enqueue(&q, mouse->target.rows[i], mouse->target.cols[i]);
    }
    while(!isEmpty(&q)) {
        Point p = dequeue(&q);
        int row = p.row;
        int col = p.col;
        int dist = mouse->cell_distances[row][col];
        if(row > 0 && mouse->maze.cells[row][col].wallTop == 0 && mouse->cell_distances[row - 1][col] == -1) {
            mouse->cell_distances[row - 1][col] = dist + 1;
            enqueue(&q, row - 1, col);
        }
        if(col < N - 1 && mouse->maze.cells[row][col].wallRight == 0 && mouse->cell_distances[row][col + 1] == -1) {
            mouse->cell_distances[row][col + 1] = dist + 1;
            enqueue(&q, row, col + 1);
        }
        if(row < N - 1 && mouse->maze.cells[row][col].wallBottom == 0 && mouse->cell_distances[row + 1][col] == -1) {
            mouse->cell_distances[row + 1][col] = dist + 1;
            enqueue(&q, row + 1, col);
        }
        if(col > 0 && mouse->maze.cells[row][col].wallLeft == 0 && mouse->cell_distances[row][col - 1] == -1) {
            mouse->cell_distances[row][col - 1] = dist + 1;
            enqueue(&q, row, col - 1);
        }
    }
}

int mouse_move_next(Mouse* mouse) {
    int row = mouse->row;
    int col = mouse->col;
    int current_dist = mouse->cell_distances[row][col];
    
    int neighbors_count = 0;
    int neighbors_dist[4];
    Point neighbors_points[4];

    // north neighbor
    if ( row > 0 && mouse->maze.cells[row][col].wallTop == 0 ) {
        neighbors_points[neighbors_count].row = row - 1;
        neighbors_points[neighbors_count].col = col;
        neighbors_dist[neighbors_count] = mouse->cell_distances[row - 1][col];
        neighbors_count++;
    }
    // east neighbor
    if ( col < N - 1 && mouse->maze.cells[row][col].wallRight == 0 ) {
        neighbors_points[neighbors_count].row = row;
        neighbors_points[neighbors_count].col = col + 1;
        neighbors_dist[neighbors_count] = mouse->cell_distances[row][col + 1];
        neighbors_count++;
    }
    // south neighbor
    if ( row < N - 1 && mouse->maze.cells[row][col].wallBottom == 0 ) {
        neighbors_points[neighbors_count].row = row + 1;
        neighbors_points[neighbors_count].col = col;
        neighbors_dist[neighbors_count] = mouse->cell_distances[row + 1][col];
        neighbors_count++;
    }
    // west neighbor
    if ( col > 0 && mouse->maze.cells[row][col].wallLeft == 0 ) {
        neighbors_points[neighbors_count].row = row;
        neighbors_points[neighbors_count].col = col - 1;
        neighbors_dist[neighbors_count] = mouse->cell_distances[row][col - 1];
        neighbors_count++;
    }
    if ( neighbors_count == 0 ) {
        return FAIL;
    }
    int min_dist = current_dist;
    int min_index = -1;
    for(int i = 0; i < neighbors_count; i++) {
        if(neighbors_dist[i] < min_dist) {
            min_dist = neighbors_dist[i];
            min_index = i;
        }
    }
    if (min_dist == current_dist) {
        return REPLAN;
    }

    int turn = -1;
    switch(mouse->dir) {
        case UP:
            if (neighbors_points[min_index].row == row - 1) {
                mouse->dir = UP;
            } else if (neighbors_points[min_index].col == col + 1) {
                mouse->dir = RIGHT;
                turn = RIGHT;
            } else if (neighbors_points[min_index].col == col - 1) {
                mouse->dir = LEFT;
                turn = LEFT;
            }
            break;
        case RIGHT:
            if (neighbors_points[min_index].row == row - 1) {
                mouse->dir = UP;
                turn = LEFT;
            } else if (neighbors_points[min_index].col == col + 1) {
                mouse->dir = RIGHT;
            } else if (neighbors_points[min_index].row == row + 1) {
                mouse->dir = DOWN;
                turn = RIGHT;
            }
            break;
        case DOWN:
            if (neighbors_points[min_index].col == col + 1) {
                mouse->dir = RIGHT;
                turn = RIGHT;
            } else if (neighbors_points[min_index].col == col - 1) {
                mouse->dir = LEFT;
                turn = LEFT;
            } else if (neighbors_points[min_index].row == row + 1) {
                mouse->dir = DOWN;
            }
            break;
        case LEFT:
            if (neighbors_points[min_index].row == row - 1) {
                mouse->dir = UP;
                turn = RIGHT;
            } else if (neighbors_points[min_index].row == row + 1) {
                mouse->dir = DOWN;
                turn = LEFT;
            } else if (neighbors_points[min_index].col == col - 1) {
                mouse->dir = LEFT;
            }
            break;
    }

    mouse->row = neighbors_points[min_index].row;
    mouse->col = neighbors_points[min_index].col;
    mouse->commands[mouse->command_count++].turn = turn;

    return min_dist == 0 ? TARGET : OK;
}
