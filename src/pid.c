#include "pid.h"
#include "motorEncoders.h"
#include "motors.h"
#include "uart.h"
#include <math.h>
#include <stdio.h>

#define THRESHHOLD 10.0
#define MAX_POWER 80.0
#define MIN_POWER 10.0

typedef struct PID_ {
    float kPw;
    float kIw;
    float kDw;
    float kPx;
    float kIx;
    float kDx;
    float angleError;
    float oldAngleError;
    float distanceError;
    float oldDistanceError;
    float goalAngle;
    float goalDistance;
    long count1;
    long count2;
    int8_t goalReached;
    float powerInPercentLeft;
    float powerInPercentRight;
} PID;

typedef struct PID2_{
    float K_p;
    float K_i;
    float K_d;
    float last_error;
    //output
    float w_l;
    float w_r;
    float w_f;
} PID2;

static PID pid;
static PID2 pid2;

void init_PID() {
    pid2.K_p = 0.00007;
    pid2.K_i = 0;
    pid2.K_d = 0.00001;
    pid2.last_error = 0;
    pid2.w_l = 0;
    pid2.w_r = 0;
    pid2.w_f = 0.35;
}

uint16_t updatePID(uint16_t left, uint16_t right, uint16_t front) {
    //TODO set w_f according to front distance
    float error = (float)(left - right);
    pid2.w_l = pid2.w_f - pid2.K_p * error - pid2.K_d * (error - pid2.last_error);
    pid2.w_r = pid2.w_f + pid2.K_p * error + pid2.K_d * (error - pid2.last_error);
    if(pid2.w_l > 1) pid2.w_l = 1.0;
    if(pid2.w_r > 1) pid2.w_r = 1.0;
    if(pid2.w_l < -1) pid2.w_l = -1.0;
    if(pid2.w_r < -1) pid2.w_r = -1.0;
    pid2.last_error = error;
}

void resetPID(void) {
    //set PID variables
    pid.kPw = 0.006;
    pid.kPx = 0.01;
    pid.kDw = 0.01;
    pid.kDx = 0.0;
    pid.kIw = 0;
    pid.kIx = 0;

    pid.angleError = 0;
    pid.oldAngleError = 0;
    pid.distanceError = 0;
    pid.oldDistanceError = 0;

    pid.count1 = getPositionInCounts_1();
    pid.count2 = getPositionInCounts_2();
    
    pid.goalReached = 0;
    
    pid.powerInPercentLeft = 0;
    pid.powerInPercentRight = 0;

    setPIDGoalD(0);
    setPIDGoalA(0);
}

uint16_t updatePID2(void) {
    
    char buffer[100];
    
    long posInCount1 = getPositionInCounts_1() - pid.count1;
    long posInCount2 = getPositionInCounts_2() - pid.count2;

    pid.angleError = pid.goalAngle - (posInCount1 - posInCount2);
    //pid.angleError = pid.goalAngle - (posInCount2 - posInCount1);
    float angleCorr = pid.kPw * pid.angleError + pid.kDw * (pid.angleError - pid.oldAngleError);
    pid.oldAngleError = pid.angleError;

    pid.distanceError = pid.goalDistance - ((posInCount1 + posInCount2) / 2);
    float distanceCorr = pid.kPx * pid.distanceError + pid.kDx * (pid.distanceError - pid.oldDistanceError);
    pid.oldDistanceError = pid.distanceError;
    
    //pid.powerInPercentLeft = fabs(distanceCorr + angleCorr);
    //pid.powerInPercentRight = fabs(distanceCorr - angleCorr);
    
    //pid.powerInPercentLeft = fabs(distanceCorr - angleCorr);
    //pid.powerInPercentRight = fabs(distanceCorr + angleCorr);
    
    pid.powerInPercentLeft = distanceCorr + angleCorr;
    pid.powerInPercentRight = distanceCorr - angleCorr;
    
    snprintf(buffer, sizeof (buffer), "PWM powerInPercentLeft raw: %f\r\n", pid.powerInPercentLeft);
    putsUART1(buffer);
    snprintf(buffer, sizeof (buffer), "PWM powerInPercentRight raw: %f\r\n", pid.powerInPercentRight);
    putsUART1(buffer);
    
    if (pid.powerInPercentLeft > MAX_POWER) {
        pid.powerInPercentLeft = MAX_POWER;
    } else if (pid.powerInPercentLeft < MIN_POWER) {
        pid.powerInPercentLeft = MIN_POWER;
    }
    
    if (pid.powerInPercentRight > MAX_POWER) {
        pid.powerInPercentRight = MAX_POWER;
    } else if (pid.powerInPercentRight < MIN_POWER) {
        pid.powerInPercentRight = MIN_POWER;
    }
    
    snprintf(buffer, sizeof (buffer), "PWM powerInPercentLeft: %f\r\n", pid.powerInPercentLeft);
    putsUART1(buffer);
    snprintf(buffer, sizeof (buffer), "PWM powerInPercentRight: %f\r\n", pid.powerInPercentRight);
    putsUART1(buffer);
    snprintf(buffer, sizeof (buffer), "posInCount1: %ld\r\n", posInCount1);
    putsUART1(buffer);
    snprintf(buffer, sizeof (buffer), "posInCount2: %ld\r\n", posInCount2);
    putsUART1(buffer);
    snprintf(buffer, sizeof (buffer), "angleError: %f\r\n", pid.angleError);
    putsUART1(buffer);
    snprintf(buffer, sizeof (buffer), "angleCorr: %f\r\n", angleCorr);
    putsUART1(buffer);
    snprintf(buffer, sizeof (buffer), "distanceError: %f\r\n", pid.distanceError);
    putsUART1(buffer);
    snprintf(buffer, sizeof (buffer), "distanceCorr: %f\r\n", distanceCorr);
    putsUART1(buffer);
    
    float errorTotal = pid.distanceError + pid.angleError;
    if (errorTotal < THRESHHOLD) {
        pid.powerInPercentLeft = 0;
        pid.powerInPercentRight = 0;
        pid.goalReached = 1;
        
        snprintf(buffer, sizeof (buffer), "Goal reached! Error total = %f\r\n", errorTotal);
        putsUART1(buffer);
        return 0;
    }
    
    return 1;
}

void setPIDGoalD(int16_t distance) {
    //distance in mm
    pid.goalDistance = distance;
}

void setPIDGoalA(int16_t angle) {
    //angle in degrees
    pid.goalAngle = angle;
}

int8_t PIDdone(void) {
    
    return pid.goalReached;
}

float powerInPercentLeft(void) {
    return pid2.w_l;
    //return pid.powerInPercentLeft;
}
float powerInPercentRight(void) {
    return pid2.w_r;
    //return pid.powerInPercentRight;
}