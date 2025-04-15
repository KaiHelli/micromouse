#include "mazeSolver.h"
#include "mouseController.h"

Mouse mouse;

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

//TODO
int sensor_has_wall_front() {
    return 1;
}
int sensor_has_wall_right() {
    return 1;
}
int sensor_has_wall_left() {
    return 1;
}

int discover_maze_step(Mouse* mouse) {
    int status = OK;

    // sensor readings
    int sensorWallFront = sensor_has_wall_front();
    int sensorWallRight = sensor_has_wall_right();
    int sensorWallLeft = sensor_has_wall_left();
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
    //printf("status: %d\n", status);
    if (status == REPLAN) {
        mouse_fill_distances(mouse);
    }
    return status;
}

int discover_maze(Mouse* mouse) {
    int status = OK;
    int steps = 0;
    do {
        status = discover_maze_step(mouse);
        //mouse_print_maze(mouse);
        //mouse_print_distances(mouse);
    } while (status != TARGET && status != FAIL && steps++ < MAX_STEPS);
    //printf("discovery status: %d, steps: %d\n", status, steps);
    return status;
}

void maze_runner(Mouse* mouse) {
    for(int i=0; i<mouse->command_count; i++) {
        Command* cmd = &mouse->commands[i];        
        //print("step: %d", i);
        if ( cmd->turn == LEFT ) {
            //print(" turn left\n");
            turnLeft();
        } else if ( cmd->turn == RIGHT ) {
            //print(" turn right\n");
            turnRight();
        }
        //moveForward(1);
        moveForward();
        //print(" move forward\n");
    }
}


uint8_t dicoverMaze() {
    // Setup mouse
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
        return 1;
    }

    return 0;
}

uint8_t executeMaze() {
    maze_runner(&mouse);
    return 1;
}

