#include "queue.h"

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
