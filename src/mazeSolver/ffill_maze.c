#include <stdio.h>
#include <stdlib.h>

#include "maze.h"
#include "queue.h"
#include "mouse.h"

#define MAX_STEPS 20

// sensor simulation API
int sensor_has_wall_front(Maze* maze, int row, int col, int dir) {
    switch (dir) {
        case UP:
            return maze_get_wall_top(maze, row, col);
        case RIGHT:
            return maze_get_wall_right(maze, row, col);
        case DOWN:
            return maze_get_wall_bottom(maze, row, col);
        case LEFT:
            return maze_get_wall_left(maze, row, col);
    }
    return 1;
}
int sensor_has_wall_right(Maze* maze, int row, int col, int dir) {
    switch (dir) {
        case UP:
            return maze_get_wall_right(maze, row, col);
        case RIGHT:
            return maze_get_wall_bottom(maze, row, col);
        case DOWN:
            return maze_get_wall_left(maze, row, col);
        case LEFT:
            return maze_get_wall_top(maze, row, col);
    }
    return 1;
}
int sensor_has_wall_left(Maze* maze, int row, int col, int dir) {
    switch (dir) {
        case UP:
            return maze_get_wall_left(maze, row, col);
        case RIGHT:
            return maze_get_wall_top(maze, row, col);
        case DOWN:
            return maze_get_wall_right(maze, row, col);
        case LEFT:
            return maze_get_wall_bottom(maze, row, col);
    }
    return 1;
}

int discover_maze_step(Mouse* mouse, Maze* maze) {
    int status = OK;

    // sensor readings
    int sensorWallFront = sensor_has_wall_front(maze, mouse->row, mouse->col, mouse->dir);
    int sensorWallRight = sensor_has_wall_right(maze, mouse->row, mouse->col, mouse->dir);
    int sensorWallLeft = sensor_has_wall_left(maze, mouse->row, mouse->col, mouse->dir);
    // register walls
    if ( sensorWallFront ) {
        switch (mouse->dir) {
            case UP:
                maze_set_wall_top(&mouse->maze, mouse->row, mouse->col);
                break;
            case RIGHT:
                maze_set_wall_right(&mouse->maze, mouse->row, mouse->col);
                break;
            case DOWN:
                maze_set_wall_bottom(&mouse->maze, mouse->row, mouse->col);
                break;
            case LEFT:
                maze_set_wall_left(&mouse->maze, mouse->row, mouse->col);
                break;
        }
    }
    if ( sensorWallRight ) {
        switch (mouse->dir) {
            case UP:
                maze_set_wall_right(&mouse->maze, mouse->row, mouse->col);
                break;
            case RIGHT:
                maze_set_wall_bottom(&mouse->maze, mouse->row, mouse->col);
                break;
            case DOWN:
                maze_set_wall_left(&mouse->maze, mouse->row, mouse->col);
                break;
            case LEFT:
                maze_set_wall_top(&mouse->maze, mouse->row, mouse->col);
                break;
        }
    }
    if ( sensorWallLeft ) {
        switch (mouse->dir) {
            case UP:
                maze_set_wall_left(&mouse->maze, mouse->row, mouse->col);
                break;
            case RIGHT:
                maze_set_wall_top(&mouse->maze, mouse->row, mouse->col);
                break;
            case DOWN:
                maze_set_wall_right(&mouse->maze, mouse->row, mouse->col);
                break;
            case LEFT:
                maze_set_wall_bottom(&mouse->maze, mouse->row, mouse->col);
                break;
        }
    }
    // next cell:
    // list all possible directions, reachable cells
    status = mouse_move_next(mouse);
    printf("status: %d\n", status);
    if (status == REPLAN) {
        mouse_fill_distances(mouse);
    }
    return status;
}

int discover_maze(Mouse* mouse, Maze* maze) {
    int status = OK;
    int steps = 0;
    do {
        status = discover_maze_step(mouse, maze);
        mouse_print_maze(mouse);
        mouse_print_distances(mouse);
    } while (status != TARGET && status != FAIL && steps++ < MAX_STEPS);
    printf("discovery status: %d, steps: %d\n", status, steps);
    return status;
}

void maze_runner(Mouse* mouse) {
    for(int i=0; i<mouse->command_count; i++) {
        Command* cmd = &mouse->commands[i];        
        printf("step: %d", i);
        if ( cmd->turn == LEFT ) {
            printf(" turn left\n");
        } else if ( cmd->turn == RIGHT ) {
            printf(" turn right\n");
        }
        printf(" move forward\n");
    }
}

int main(int argc, char *argv[]) {
    Maze maze;
    Mouse mouse;

    // Setup maze
    maze_init(&maze);
    maze_add_borders(&maze);
    maze_add_walls(&maze);
    maze_print(&maze);

    // Setup mouse
    mouse_init(&mouse, 5, 0, UP);
    mouse_add_target(&mouse, 2, 2);
    mouse_add_target(&mouse, 3, 2);
    mouse_add_target(&mouse, 2, 3);
    mouse_add_target(&mouse, 3, 3);

    mouse_fill_distances(&mouse);
    mouse_print_maze(&mouse);
    mouse_print_distances(&mouse);

#ifdef TEST    
    // assume, walls are detected
    maze_set_wall_right(&mouse.maze, 2, 1);
    maze_set_wall_right(&mouse.maze, 3, 1);
    maze_set_wall_right(&mouse.maze, 2, 3);
    maze_set_wall_top(&mouse.maze, 4, 2);
    maze_set_wall_top(&mouse.maze, 4, 3);
    maze_set_wall_top(&mouse.maze, 2, 2);
    maze_set_wall_top(&mouse.maze, 2, 3);
    maze_set_wall_right(&mouse.maze, 4, 0);
    maze_set_wall_right(&mouse.maze, 5, 0);
    maze_set_wall_top(&mouse.maze, 3, 0);

    mouse_fill_distances(&mouse);
    mouse_print_maze(&mouse);
    mouse_print_distances(&mouse);
#endif

    int status = discover_maze(&mouse, &maze);
    if ( status == TARGET ) {
        maze_runner(&mouse);
    }

    return 0;
}
