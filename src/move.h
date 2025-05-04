/*=============================================================================
 *  move.h ? Public interface for move.c
 *-----------------------------------------------------------------------------
 *  Motion?planning and control helpers for a micromouse?style robot.
 *  All functions are *blocking*: they return only when the requested
 *  operation is complete or an unrecoverable error is detected.
 *============================================================================*/

#ifndef MOVE_H
#define MOVE_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------------- */
/* High-level direction selectors                                            */
/* ------------------------------------------------------------------------- */

/* Direction selector used by pivot / side?turn helpers                      */
typedef enum {
    MOVE_LEFT,              /* turn -> +90° (CCW)  */
    MOVE_RIGHT              /* turn -> 90° (CW)    */
} Movement_t;

/* ------------------------------------------------------------------------- */
/* Miscellaneous helpers                                                     */
/* ------------------------------------------------------------------------- */
void     centerMouseInCell(void);
void     calibrateStartPosition(void);

/* ------------------------------------------------------------------------- */
/* Kinematic helpers                                                         */
/* ------------------------------------------------------------------------- */
void     setStartingPosition(void);
int32_t  requiredMicrometersToSpeed(float speed_mps);
float    requiredTimeToSpeed(float speed_mps);
uint32_t requiredTicksToSpeed(float speed_mps, float controlHz);

/* ------------------------------------------------------------------------- */
/* Motion primitives                                                         */
/* ------------------------------------------------------------------------- */
void targetStraight(int32_t startMicrometers,
                    float    distance,
                    float    endSpeed);

void inplaceTurnDeg(float degrees, float force);
void inplaceTurn   (float radians, float force);

/* ------------------------------------------------------------------------- */
/* Wall-alignment helpers                                                    */
/* ------------------------------------------------------------------------- */
void squareUpByWiggle      (float initStepDeg, uint8_t maxIter, float force);
void keepFrontWallDistance (float distance_um);

/* ------------------------------------------------------------------------- */
/* Stopping helpers                                                          */
/* ------------------------------------------------------------------------- */
void stopEnd(void);
void stopHeadFrontWall(void);
void stopMiddle(void);

/* ------------------------------------------------------------------------- */
/* Composite / high-level moves                                              */
/* ------------------------------------------------------------------------- */
void pivot90(Movement_t dir, float force);
void pivot180(float force);

void moveForwardCenterCells (uint8_t nCells, float cruiseSpeed, float endSpeed);
void moveForwardCenter(float cruiseSpeed, float endSpeed);

void turnLeftCenter (float force);
void turnRightCenter(float force);

void escapeDeadEnd(float force);


#endif /* MOVE_H */
