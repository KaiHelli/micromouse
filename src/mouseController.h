#ifndef MOUSECONTROLLER_H
#define	MOUSECONTROLLER_H

#include <xc.h>
#include "mazeSolver.h"
#include "move.h"

#define CELL_SIZE 180
#define RIGHT_ANG 90
#define LEFT_ANG -90

void centerMouseInCell();

//Movement API
void moveForward(uint16_t cells);

void turnLeft();

void turnRight();

void turnAround();

void turnDirection(Direction direction);

#endif	/* MOUSECONTROLLER_H */

