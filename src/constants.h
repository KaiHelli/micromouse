#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <math.h>

/* Universal constants */

#define MICROSECONDS_PER_SECOND        1000000UL
#define MILLISECONDS_PER_SECOND        1000UL               

#define MICROMETERS_PER_MILLIMETER     1000UL
#define MILLIMETERS_PER_METER          1000UL
#define MICROMETERS_PER_METER          1000000UL

#define RAD2DEG      (180.0f / (float)M_PI)
#define DEG2RAD      ((float)M_PI / 180.0f)

/* Maze geometry (ISO 18 cm cell) */

#define CELL_DIMENSION_MM            180.0f
#define WALL_WIDTH_MM                12.2f
#define CELL_DIAGONAL_MM             ((sqrtf(2.0f) * CELL_DIMENSION_MM) / 2.0f)
#define MIDDLE_MAZE_DISTANCE_MM      ((CELL_DIMENSION_MM - WALL_WIDTH_MM) / 2.0f)

/* Pre-computed micrometre versions */
#define CELL_DIMENSION_UM            ((int32_t)(CELL_DIMENSION_MM  * MICROMETERS_PER_MILLIMETER))
#define WALL_WIDTH_UM                ((int32_t)(WALL_WIDTH_MM      * MICROMETERS_PER_MILLIMETER))
#define CELL_DIAGONAL_UM             ((int32_t)(CELL_DIAGONAL_MM   * MICROMETERS_PER_MILLIMETER))
#define MIDDLE_MAZE_DISTANCE_UM      ((int32_t)(MIDDLE_MAZE_DISTANCE_MM * MICROMETERS_PER_MILLIMETER))

/* Mouse physical layout */
#define MOUSE_MOTOR_TAIL_MM          50.8f
#define MOUSE_MOTOR_HEAD_MM          50.922f
#define MOUSE_AXIS_SEPARATION_MM     0.0f

#define MOUSE_LENGTH_MM              (MOUSE_MOTOR_TAIL_MM + MOUSE_MOTOR_HEAD_MM)
#define MOUSE_TAIL_MM                (MOUSE_MOTOR_TAIL_MM + MOUSE_AXIS_SEPARATION_MM / 2.0f)
#define MOUSE_HEAD_MM                (MOUSE_LENGTH_MM - MOUSE_TAIL_MM)
#define MOUSE_START_SHIFT_MM         (MOUSE_TAIL_MM + WALL_WIDTH_MM / 2.0f)

/* Drive-wheel parameters */

#define MOUSE_WHEEL_SEPARATION_MM       97.15f
#define MOUSE_WHEEL_RADIUS_MM           30.25f
#define MOUSE_WHEEL_DIAMETER_MM         (2.0f * MOUSE_WHEEL_RADIUS_MM)
#define MOUSE_WHEEL_CIRCUMFERENCE_MM    (MOUSE_WHEEL_DIAMETER_MM * (float)M_PI)

/* Physics / motor capability */

#define MOUSE_MASS_KG                   0.307f
#define MOUSE_MOMENT_OF_INERTIA_KGM2    0.0005641125f   // \frac{0.307}{12}\cdot \left(0.105^2+0.105^2\right)
#define MOUSE_MAX_ANGULAR_VELOCITY_RADPS 32.0f          // \frac{\left(2\cdot 30\cdot 3000\cdot \frac{\pi }{180}\right)}{97}

/* Motor data sheet (33:1 HP + 30 mm wheel) */
#define MOUSE_WHEEL_MAX_ACC_DPS2        3000.0f // dps2 -- exact 3054.976
#define MOUSE_WHEEL_MAX_VEL_DPS         878.65 // dps  -- exact 1022.728 - but takes longer to reach from 878.65 on
#define MOUSE_WHEEL_MAX_VEL_MMPS        (MOUSE_WHEEL_MAX_VEL_DPS * DEG2RAD * MOUSE_WHEEL_RADIUS_MM)

#define MOUSE_WHEEL_TORQUE_CONT_NM      0.03f
#define MOUSE_WHEEL_TORQUE_PEAK_NM      0.10f

/*  F = tau / r - keep everything in metres here */
#define MOUSE_WHEEL_MAX_FORCE_CONT_N \
        (MOUSE_WHEEL_TORQUE_CONT_NM / (MOUSE_WHEEL_RADIUS_MM / (float)MILLIMETERS_PER_METER))
#define MOUSE_WHEEL_MAX_FORCE_PEAK_N \
        (MOUSE_WHEEL_TORQUE_PEAK_NM / (MOUSE_WHEEL_RADIUS_MM / (float)MILLIMETERS_PER_METER))

/* Encoder constants */

#define ENC_TICKS_PER_REV            (4U * 16U * 33U)          /* 4×QEI × 16 slots × 33:1 */
#define ENC_TICKS_TO_RAD             (2.0f * (float)M_PI / ENC_TICKS_PER_REV)
#define ENC_TICKS_TO_DEG             (360.0f / ENC_TICKS_PER_REV)
#define ENC_DIST_PER_TICK_MM         (MOUSE_WHEEL_CIRCUMFERENCE_MM / ENC_TICKS_PER_REV)
#define ENC_DIST_PER_TICK_UM         (ENC_DIST_PER_TICK_MM * MICROMETERS_PER_MILLIMETER)

/* Control & alignment tolerances */

#define KEEP_FRONT_DISTANCE_TOLERANCE_MM   1.0f                    /* was 0.001 m (!): */
#define KEEP_FRONT_DISTANCE_TOLERANCE_UM   (KEEP_FRONT_DISTANCE_TOLERANCE_MM * MICROMETERS_PER_MILLIMETER)

#define SHIFT_AFTER_180_DEG_TURN_MM        0.0f                   /* was 0.010 m */
#define SHIFT_AFTER_180_DEG_TURN_UM        (SHIFT_AFTER_180_DEG_TURN_MM * MICROMETERS_PER_MILLIMETER)

#endif /* CONSTANTS_H */
