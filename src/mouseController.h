#ifndef MOUSECONTROLLER_H
#define	MOUSECONTROLLER_H

#include <xc.h>

#define CELL_SIZE 180
#define RIGHT 90
#define LEFT -90

//Movement API
void moveForward(uint16_t cells);

void turnLeft();

void turnRight();

void turnAround();

#endif	/* MOUSECONTROLLER_H */

