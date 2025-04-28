#include "mazeSolver.h"
#include "mouseController.h"
#include "move.h"
#include "sensors.h"
#include "oled.h"
#include "uart.h"

#include <stdlib.h>

#define MAZESOLVER
#ifdef MAZESOLVER

static Mouse mouse;

//TODO verify value
//in mm, distance of a wall if mouse in the centre of the cell 
#define WALL_MAX_DISTANCE 75

void queue_init(Queue *q) {
    q->head = 0;
    q->tail = 0;
}

void enqueue(Queue *q, int row, int col) {
    q->data[q->tail].row = row;
    q->data[q->tail].col = col;
    q->tail++;
}

Point dequeue(Queue *q) {
    return q->data[q->head++];
}

int isEmpty(Queue *q) {
    return q->head == q->tail;
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

//void maze_print(Maze* maze);
//void maze_print_ex(Maze* maze, int row, int col, char c);

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

//void mouse_print_maze(Mouse* mouse);
//void mouse_print_distances(Mouse* mouse);

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

int mouse_plan_next(Mouse* mouse) {
    int row = mouse->row;
    int col = mouse->col;
    int current_dist = mouse->cell_distances[row][col];
    
    int neighbors_count = 0;
    int neighbors_dist[4];
    Point neighbors_points[4];

    // determine reachable neighbors
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
        // impossible?
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

    int turn = NO_TURN;
    int next_row = neighbors_points[min_index].row;
    int next_col = neighbors_points[min_index].col;
    switch(mouse->dir) {
        case UP:
            if (next_row == row - 1) {
                mouse->dir = UP;
            } else if (next_col == col + 1) {
                mouse->dir = RIGHT;
                turn = TURN_RIGHT;
            } else if (next_col == col - 1) {
                mouse->dir = LEFT;
                turn = TURN_LEFT;
            } else {
                mouse->dir = DOWN;
                turn = TURN_BACK;
            }
            break;
        case RIGHT:
            if (next_row == row - 1) {
                mouse->dir = UP;
                turn = TURN_LEFT;
            } else if (next_col == col + 1) {
                mouse->dir = RIGHT;
            } else if (next_row == row + 1) {
                mouse->dir = DOWN;
                turn = TURN_RIGHT;
            } else {
                mouse->dir = LEFT;
                turn = TURN_BACK;
            }
            break;
        case DOWN:
            if (next_col == col + 1) {
                mouse->dir = RIGHT;
                turn = TURN_LEFT;
            } else if (next_col == col - 1) {
                mouse->dir = LEFT;
                turn = TURN_RIGHT;
            } else if (next_row == row + 1) {
                mouse->dir = DOWN;
            } else {
                mouse->dir = UP;
                turn = TURN_BACK;
            }
            break;
        case LEFT:
            if (next_row == row - 1) {
                mouse->dir = UP;
                turn = TURN_RIGHT;
            } else if (next_row == row + 1) {
                mouse->dir = DOWN;
                turn = TURN_LEFT;
            } else if (next_col == col - 1) {
                mouse->dir = LEFT;
            } else {
                mouse->dir = RIGHT;
                turn = TURN_BACK;
            }
            break;
    }

    mouse->row = neighbors_points[min_index].row;
    mouse->col = neighbors_points[min_index].col;
    oledUpdateMouse(mouse);
    
    if ( mouse->command_count >= N * N ) {
        return FAIL;
    }
    mouse->commands[mouse->command_count].cells = 1;
    mouse->commands[mouse->command_count].turn = turn;
    mouse->commands[mouse->command_count].dir = mouse->dir;
    mouse->command_count++;
    
    //uprintf("%d. Turn: %d (%d,%d:%d) -> (%d,%d:%d)\r\n", mouse->command_count, turn, row, col, current_dist, mouse->row, mouse->col, min_dist);

    return min_dist == 0 ? TARGET : OK;
}


int mouse_move_next(Mouse* mouse) {
    int status = mouse_plan_next(mouse);
    if (status == OK || status == TARGET) {
        int turn = mouse->commands[mouse->command_count - 1].turn;
#ifdef RELATIVE_TURN
        if (turn == TURN_LEFT) {
            turnLeft();
        } else if (turn == TURN_RIGHT) {
            turnRight();
        } else if (turn == TURN_BACK) {
            // turn back
//            turnLeft();
//            turnLeft();
            turnAround();
        }
        moveForward(1);
#else
        int turn_direction = mouse->commands[mouse->command_count - 1].dir;
        if (turn != NO_TURN) {
            turnDirection(turn_direction);
        }
        moveForward(1);
#endif        
    }
    return status;
}


int sensor_has_wall_front() {
    uint16_t distance = getSensorDistance(SENSOR_CENTER);
    if (distance > WALL_MAX_DISTANCE) {
        return 0;
    }
    return 1;
}
int sensor_has_wall_right() {
    uint16_t distance = getSensorDistance(SENSOR_RIGHT);
    if (distance > WALL_MAX_DISTANCE) {
        return 0;
    }
    return 1;
}
int sensor_has_wall_left() {
    uint16_t distance = getSensorDistance(SENSOR_LEFT);
    if (distance > WALL_MAX_DISTANCE) {
        return 0;
    }
    return 1;
}

int discover_maze_step(Mouse* mouse) {
    int status = OK;

    // sensor readings
    int sensorWallFront = sensor_has_wall_front();
    int sensorWallRight = sensor_has_wall_right();
    int sensorWallLeft = sensor_has_wall_left();
    
    //uprintf("Sensor front: %d, left: %d, right: %d \r\n", sensorWallFront, sensorWallLeft, sensorWallRight);
    
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
    
    oledDrawCell(&mouse->maze,mouse->row, mouse->col);
    // next cell:
    // list all possible directions, reachable cells
    status = mouse_move_next(mouse);
    //uprintf("status: %d\r\n", status);
    
    if (status == REPLAN) {
        mouse_fill_distances(mouse);
    }
    return status;
}

int discover_maze(Mouse* mouse) {
    int status = OK;
    int steps = 0;
    do {
        mouse->visited[mouse->row][mouse->col] = 1;
        status = discover_maze_step(mouse);
    } while (status != TARGET && status != FAIL && steps++ < MAX_STEPS);
    //uprintf("discovery status: %d, steps: %d\n", status, steps);
    return status;
}


int plan_run(Mouse* mouse) {
    int status = OK;
    int steps = 0;
    do {
        status = mouse_plan_next(mouse);
    } while (status != TARGET && status != FAIL && steps++ < MAX_STEPS);
    if (status == TARGET) {
        int last_turn = TURN_BACK;
        for(int i=0; i<mouse->command_count; i++) {
            Command* cmd = &mouse->commands[i];
            if (cmd->turn == NO_TURN && (last_turn == NO_TURN || last_turn == TURN_LEFT || last_turn == TURN_RIGHT)) {
                mouse->commands[i-1].cells += cmd->cells;
                for(int j=i; j<mouse->command_count; j++) {
                    mouse->commands[j] = mouse->commands[j+1];
                }
                mouse->command_count--;
                i--;
            }
            last_turn = cmd->turn;
        }
    }
    return status;
}

#ifdef RELATIVE_TURN
void maze_runner(Mouse* mouse) {
    for(int i=0; i<mouse->command_count; i++) {
        Command* cmd = &mouse->commands[i];        
        if ( cmd->turn == TURN_LEFT ) {
            turnLeft();
        } else if ( cmd->turn == TURN_RIGHT ) {
            turnRight();
        } else if ( cmd->turn == TURN_BACK ) {
//            turnLeft();
//            turnLeft();
            turnAround();
        }
        moveForward(cmd->cells);
    }
}


void go_back(Mouse* mouse) {
    
//    turnLeft();
//    turnLeft();
    turnAround();
    for (int i = mouse->command_count - 1; i >= 0; i--) {
        Command* cmd = &mouse->commands[i];
        int turn = cmd->turn;
        if (turn == TURN_LEFT) {
            turn = TURN_RIGHT;
        } else if (turn == TURN_RIGHT) {
            turn = TURN_LEFT;
        }
        
        moveForward(cmd->cells);
        
        if ( turn == TURN_LEFT ) {
            turnLeft();
        } else if ( turn == TURN_RIGHT ) {
            turnRight();
        } else if ( turn == TURN_BACK ) {
//            turnLeft();
//            turnLeft();
            turnAround();
        }
    }
    
//    turnLeft();
//    turnLeft();
    turnAround();
}
#else
int oppositeDirection(int dir) {
    switch (dir) {
        case UP: return DOWN;
        case RIGHT: return LEFT;
        case DOWN: return UP;
        case LEFT: return RIGHT;
    }
    return -1;
}

int oppositeTurn(int turn, int dir) {
    switch (turn) {
        case TURN_LEFT: 
            switch (dir) {
                case UP: return LEFT;
                case RIGHT: return UP;
                case DOWN: return RIGHT;
                case LEFT: return DOWN;
            }
            break;
        case TURN_RIGHT:
            switch (dir) {
                case UP: return RIGHT;
                case RIGHT: return DOWN;
                case DOWN: return LEFT;
                case LEFT: return UP;
            }
            break;
        case TURN_BACK: return dir;
    }
    return NO_TURN;
}

void go_back(Mouse* mouse) {
    turnDirection(oppositeDirection(mouse->dir));
    for (int i = mouse->command_count - 1; i >= 0; i--) {
        Command* cmd = &mouse->commands[i];
        int turn = cmd->turn;
        int dir = cmd->dir;
        moveForward(cmd->cells);
        if ( turn != NO_TURN ) {
            turnDirection(oppositeTurn(turn, dir));
        }
    }
    turnDirection(mouse->commands[0].dir);
}

void maze_runner(Mouse* mouse) {
    for(int i=0; i<mouse->command_count; i++) {
        Command* cmd = &mouse->commands[i];        
        if ( cmd->turn != NO_TURN ) {
            turnDirection(cmd->dir);
        }
        moveForward(cmd->cells);
    }
}

#endif

void mouse_final_distances(Mouse* mouse) {
    Queue q;

    for(int i = 0; i < N; i++) {
        for(int j = 0; j < N; j++) {
            mouse->cell_distances[i][j] = 100;
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
        if(row > 0 && mouse->maze.cells[row][col].wallTop == 0 && mouse->cell_distances[row - 1][col] == 100 && mouse->visited[row - 1][col] == 1) {
            mouse->cell_distances[row - 1][col] = dist + 1;
            enqueue(&q, row - 1, col);
        }
        if(col < N - 1 && mouse->maze.cells[row][col].wallRight == 0 && mouse->cell_distances[row][col + 1] == 100 && mouse->visited[row][col + 1] == 1) {
            mouse->cell_distances[row][col + 1] = dist + 1;
            enqueue(&q, row, col + 1);
        }
        if(row < N - 1 && mouse->maze.cells[row][col].wallBottom == 0 && mouse->cell_distances[row + 1][col] == 100 && mouse->visited[row + 1][col] == 1) {
            mouse->cell_distances[row + 1][col] = dist + 1;
            enqueue(&q, row + 1, col);
        }
        if(col > 0 && mouse->maze.cells[row][col].wallLeft == 0 && mouse->cell_distances[row][col - 1] == 100 && mouse->visited[row][col - 1] == 1) {
            mouse->cell_distances[row][col - 1] = dist + 1;
            enqueue(&q, row, col - 1);
        }
    }
    
}

uint8_t solveMaze() {
    // Setup mouse
    uint8_t rc = 0;
    
    mouse_init(&mouse, 5, 0, UP);
    mouse_add_target(&mouse, 2, 2);
    mouse_add_target(&mouse, 3, 2);
    mouse_add_target(&mouse, 2, 3);
    mouse_add_target(&mouse, 3, 3);

    mouse_fill_distances(&mouse);
    //mouse_print_maze(&mouse);
    //mouse_print_distances(&mouse);
    
    int status = discover_maze(&mouse);
    if ( status == TARGET ) {
        go_back(&mouse);
        
        // we are at the start position again
        // we can discover the maze with more walls known
        mouse.command_count = 0;
        mouse.row = 5;
        mouse.col = 0;
        mouse.dir = UP;
        mouse_final_distances(&mouse);
        status = plan_run(&mouse);
        if (status == TARGET) {
            maze_runner(&mouse);
            rc = 1;
        }
    } 
    
    return rc;
}

#else

    uint8_t solveMaze() {
        return 0;
    }

#endif