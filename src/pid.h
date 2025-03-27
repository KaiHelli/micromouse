#ifndef PID_H
#define	PID_H

#include <xc.h>

void init_PID(void);
void resetPID(void);
uint16_t updatePID(uint16_t left, uint16_t right, uint16_t front);
uint16_t updatePID2(void);
void setPIDGoalD(int16_t distance);
void setPIDGoalA(int16_t angle);
int8_t PIDdone(void); 
float powerInPercentLeft(void);
float powerInPercentRight(void);
#endif	/* PID_H */

