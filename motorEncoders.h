/*
 * File:   sensor.h
 * Author: bauma
 *
 * Created on 23. Mai 2019, 11:22
 */

#ifndef SENSOR_H
#define	SENSOR_H


#include<xc.h>

void initQEI1( unsigned int  startPos);
void initQEI2( unsigned int  startPos);
float getPositionInRad();
//float getVelocityInRadPerSecond();
long getPositionInCounts_1();
int getVelocityInCountsPerSample_1();
long getPositionInCounts_2();
int getVelocityInCountsPerSample_2();

extern long rotationCount1;
extern long rotationCount2;

#define GET_ENCODER_1(RIGHT_ENCODER_POSITION_VALUE) (RIGHT_ENCODER_POSITION_VALUE=rotationCount1+POSCNT)
#define GET_ENCODER_2(LEFT_ENCODER_POSITION_VALUE) (LEFT_ENCODER_POSITION_VALUE=rotationCount2+POS2CNT)



#define WHEEL_ROTATIONS_PERROBOT_ROTATION 2.5
#define TICKS_PER_WHEELROTATION (64*33)
#define TICKS_PER_CENTIMETER TICKS_PER_WHEELROTATION/12.566
#define METER_PER_TICkS 0.12566/TICKS_PER_WHEELROTATION
#define DELTATICKS_90_DEGREES  (0.25* WHEEL_ROTATIONS_PERROBOT_ROTATION*TICKS_PER_WHEELROTATION) 
#define DELTATICKS_180_DEGREES (0.5 * WHEEL_ROTATIONS_PERROBOT_ROTATION*TICKS_PER_WHEELROTATION)
#define DELTATICKS_CELL_GAP (11.5*TICKS_PER_CENTIMETER)




#endif	/* SENSOR_H */