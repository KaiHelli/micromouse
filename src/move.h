#ifndef MOVE_H
#define MOVE_H

#include <stdint.h>
#include <stdbool.h>

/*--------------------------------------------------------------------
 * High-level direction selectors
 *------------------------------------------------------------------*/

typedef enum {
    MOVE_LEFT,              /* turn -> +90° (CCW)  */
    MOVE_RIGHT              /* turn -> 90° (CW)    */
} Movement_t;

/*--------------------------------------------------------------------
 * Miscellaneous helpers
 *------------------------------------------------------------------*/
void     centerMouseInCell(void);
void     calibrateStartPosition(void);
void     setStartingPosition(void);

/*--------------------------------------------------------------------
 * Kinematic helpers
 *------------------------------------------------------------------*/
int32_t  requiredMicrometersToSpeed(float speed_mps);
float    requiredTimeToSpeed(float currentSpeed, float targetSpeed);

/*--------------------------------------------------------------------
 * Motion primitives
 *------------------------------------------------------------------*/
void targetStraightEncoders(int32_t startMicrometers, int32_t distance, float endSpeed);
void targetStraightSpeed(int32_t startMicrometers, int32_t distance, float endSpeed);

void inplaceTurnDeg(float degrees, float force);
void inplaceTurn   (float radians, float force);

/*--------------------------------------------------------------------
 * Wall-alignment helpers
 *------------------------------------------------------------------*/
void squareUp(float probeAngleDeg, float force, uint32_t numReadings);
void keepFrontWallDistance (int32_t distance_um);

/*--------------------------------------------------------------------
 * Stopping helpers
 *------------------------------------------------------------------*/
void stopEnd(void);
void stopHeadFrontWall(void);
void stopMiddle(void);

/*--------------------------------------------------------------------
 * Composite / high-level moves
 *------------------------------------------------------------------*/
void pivot90(Movement_t dir, float force);
void pivot180(float force);

void moveForwardCenterCells (uint8_t nCells, float cruiseSpeed, float endSpeed);
void moveForwardCenter(float cruiseSpeed, float endSpeed);

void turnLeftCenter (float force);
void turnRightCenter(float force);
void turnAroundCenter(float force);

void escapeDeadEnd(float force);


#endif /* MOVE_H */
