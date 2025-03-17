#pragma once

#include "config.h"

typedef struct Point_ {
    int row, col;
} Point;

// ring array based queue
typedef struct Queue_ {
    Point data[N * N];
    int head, tail;
} Queue;

void queue_init(Queue *q);
void enqueue(Queue *q, int row, int col);
Point dequeue(Queue *q);
int isEmpty(Queue *q);
