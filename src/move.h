/*=============================================================================
 *  move.h ? Public interface for move.c
 *-----------------------------------------------------------------------------
 *  Motion-planning and control helpers for a micromouse-style robot.
 *  Exposes high-level moves, low-level motion primitives and assorted
 *  kinematic helpers.  All functions are *blocking* ? they return only when
 *  the requested operation has completed or a collision was detected.
 *============================================================================*/

#ifndef MOVE_H
#define MOVE_H

#include <stdint.h>
#include <stdbool.h>

/*--------------------------------------------------------------------*/
/* Symbolic movement codes (used in ASCII movement strings)           */
/*--------------------------------------------------------------------*/
#define MOVE_FRONT  'F'
#define MOVE_LEFT   'L'
#define MOVE_RIGHT  'R'
#define MOVE_STOP   'S'
#define MOVE_END    '\0'

/*--------------------------------------------------------------------*/
/* High-level direction selectors                                     */
/*--------------------------------------------------------------------*/
typedef enum {
    LEFT,
    RIGHT,
    FRONT,
    BACK
} StepDirection_t;

/* Direction selector used by moveSide()                              */
typedef enum {
    MOVE_LEFT_TURN,
    MOVE_RIGHT_TURN
} Movement_t;

/* Optional ?language? selector for path strings                       */
typedef enum {
    PATH_LANG_SIMPLE,   /* e.g. "FFLFR"                                */
    PATH_LANG_COMPACT   /* diagonals / compressed notations            */
} PathLanguage_t;

/*--------------------------------------------------------------------*/
/* Miscellaneous helpers                                              */
/*--------------------------------------------------------------------*/
int      sign(float number);

/*--------------------------------------------------------------------*/
/* Kinematic helpers                                                  */
/*--------------------------------------------------------------------*/
void     setStartingPosition(void);
int32_t  requiredMicrometersToSpeed(float speed);
float    requiredTimeToSpeed(float speed);
uint32_t requiredTicksToSpeed(float speed, float hz);

/*--------------------------------------------------------------------*/
/* Motion primitives                                                  */
/*--------------------------------------------------------------------*/
void targetStraight(int32_t startMicrometers,
                    float    distance_um,
                    float    endSpeed_mps);

void inplaceTurn(float radians, float force);

/*--------------------------------------------------------------------*/
/* Wall-alignment helpers                                             */
/*--------------------------------------------------------------------*/
void squareUpByWiggle(float initStepDeg, uint8_t maxIter, float force);
void keepFrontWallDistance(float distance_um);

/*--------------------------------------------------------------------*/
/* Stopping helpers                                                   */
/*--------------------------------------------------------------------*/
void stopEnd(void);
void stopHeadFrontWall(void);
void stopMiddle(void);

/*--------------------------------------------------------------------*/
/* Composite / high-level moves                                       */
/*--------------------------------------------------------------------*/
void turnBack(float force);
void turnToStartPosition(float force);

void moveFront(void);
void parametricMoveFront(float distance_um, float endLinearSpeed);
void moveSide(Movement_t turn, float force);
void moveBack(float force);
void move(StepDirection_t dir, float force);

/*--------------------------------------------------------------------*/
/* Simple movement-sequence executor                                  */
/*--------------------------------------------------------------------*/
void executeMovementSequence(char            *seq,
                             float           force,
                             PathLanguage_t  lang);

#endif /* MOVE_H */
