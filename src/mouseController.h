#ifndef MOUSE_CONTROLLER_H
#define MOUSE_CONTROLLER_H

/*--------------------------------------------------------------------
 *  Required system headers
 *------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "timers.h" 

/*--------------------------------------------------------------------
 *  API - High-level helpers
 *------------------------------------------------------------------*/
void   centerMouseInCell(void);

/*--------------------------------------------------------------------
 *  API - Force / speed limits & query-set helpers
 *------------------------------------------------------------------*/
float  getMaxForce(void);
void   setMaxForce(float value);
void   resetMaxForce(void);

float  getLinearAcceleration(void);
float  getLinearDeceleration(void);

float  getTargetLinearSpeed(void);
void   setTargetLinearSpeed(float value);

float  getIdealLinearSpeed(void);
void   setIdealLinearSpeed(float value);

float  getIdealAngularSpeed(void);
void   setIdealAngularSpeed(float value);

float  getMaxLinearSpeed(void);
void   setMaxLinearSpeed(float value);
void   resetMaxLinearSpeed(void);

/*--------------------------------------------------------------------
 *  API ? Control-loop helpers
 *------------------------------------------------------------------*/
void   updateIdealLinearSpeed(void);

bool   sensorIsWallFarFront(void);
bool   sensorIsWallFront(void);
bool   sensorIsWallRight(void);
bool   sensorIsWallLeft(void);

float getSideSensorsCloseError(void);
float getSideSensorsFarError(void);

/* One control-loop step */
int16_t mouseControlStep(void);

/* Enable / disable sub-controls */
void   sideSensorsCloseControl(bool enable);
void   sideSensorsFarControl(bool enable);
void   disableWallsControl(void);

/* Reset helpers */
void   resetControlErrors(void);
void   resetControlSpeed(void);
void   resetControlAll(void);

/* Enable / disable the main controller */
void   enableMouseControl(void);
void   disableMouseControl(void);

/*--------------------------------------------------------------------
 *  API - Initialization
 *------------------------------------------------------------------*/
void initMouseController(Timer_t timer, uint16_t numTicks);

/*--------------------------------------------------------------------*/

#endif /* MOUSE_CONTROLLER_H */
