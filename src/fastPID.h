// Source based on: https://github.com/mike-matera/FastPID

#ifndef FAST_PID_H
#define FAST_PID_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Macros used by the PID algorithm for integer scaling and limiting.
 * These values help avoid overflow in intermediate calculations.
 */
#define INTEG_MAX    (INT32_MAX)
#define INTEG_MIN    (INT32_MIN)
#define DERIV_MAX    (INT16_MAX)
#define DERIV_MIN    (INT16_MIN)

#define PARAM_SHIFT  12
#define PARAM_BITS   16
#define PARAM_MAX    (((0x1ULL << PARAM_BITS) - 1) >> PARAM_SHIFT)
#define PARAM_MULT   (((0x1ULL << PARAM_BITS)) >> (PARAM_BITS - PARAM_SHIFT))

/* --- Anti?windup selection ---------------------------------------------- */
typedef enum  {
    FASTPID_AW_NONE   = 0,       /* disable integral anti?windup          */
    FASTPID_AW_CLAMP,            /* conditional clamp?on?saturation       */
    FASTPID_AW_BACKCALC          /* back?calculation anti?windup          */
} FastPidAntiWindup;

/*
 * FastPid struct maintains:
 *   - PID tuning parameters (in fixed-point form):
 *       pParameter, iParameter, dParameter
 *   - Output limits, to prevent integral windup and saturation:
 *       outputMax, outputMin
 *   - Internal state for integration and derivation:
 *       integralSum, lastError, lastSetpoint
 *   - A configError flag to indicate invalid configurations
 */
typedef struct {
    // Configuration
    uint32_t pParameter;
    uint32_t iParameter;
    uint32_t dParameter;
    uint32_t fParameter;
    int64_t  outputMax;
    int64_t  outputMin;
    FastPidAntiWindup awMode;       /* anti?windup strategy                 */
    uint32_t          bcParameter;  /* back?calc gain (Kt) ? fixed?point    */
    bool     configError;
    
    // State
    int16_t  lastSetpoint;
    int16_t  lastOutput;
    int64_t  integralSum;
    int32_t  lastError;
} FastPid;

/*
 * Function Prototypes
 */

/**
 * @brief  Initialize all internal fields of the FastPid struct to safe defaults.
 * @param  pid Pointer to the FastPid struct.
 */
void fastPidInit(FastPid *pid);

/**
 * @brief  Configure the PID controller with user-supplied kp, ki, kd, loop frequency, and output format.
 * @param  pid   Pointer to the FastPid struct.
 * @param  kp    Proportional gain.
 * @param  ki    Integral gain.
 * @param  kd    Derivative gain.
 * @param  kf    Feed-forward gain.
 * @param  hz    Control loop frequency in Hertz.
 * @param  bits  Number of bits for the output resolution (1 to 16).
 * @param  sign  True = signed output, false = unsigned output (0-min output).
 * @return True if configuration is valid, false if invalid parameters are encountered.
 */
bool fastPidConfigure(FastPid *pid, float kp, float ki, float kd, float kf, float hz, int bits, bool sign);

/**
 * @brief  Set the PID coefficients (kp, ki, kd) with loop frequency.
 *         Uses float-to-fixed conversions internally.
 * @param  pid  Pointer to the FastPid struct.
 * @param  kp   Proportional gain.
 * @param  ki   Integral gain.
 * @param  kd   Derivative gain.
 * @parm   kf   Feed-forward gain.
 * @param  hz   Control loop frequency in Hertz.
 * @return True if valid, false if an invalid parameter was encountered.
 */
bool fastPidSetCoefficients(FastPid *pid, float kp, float ki, float kd, float kf, float hz);

/**
 * @brief  Configure output bit width and sign (signed vs. unsigned).
 * @param  pid   Pointer to the FastPid struct.
 * @param  bits  Number of bits for output resolution (1 to 16).
 * @param  sign  True for signed output, false for unsigned.
 * @return True if valid, false if the configuration is invalid.
 */
bool fastPidSetOutputConfig(FastPid *pid, int bits, bool sign);

/**
 * @brief  Configure output range.
 * @param  pid  Pointer to the FastPid struct.
 * @param  min  Minimum output.
 * @param  max  Maximum output.
 * @return True if valid, false if the configuration is invalid.
 */
bool fastPidSetOutputRange(FastPid *pid, int16_t min, int16_t max);

/**
 * @brief  Perform one step of the PID control algorithm.
 * @param  pid    Pointer to the FastPid struct.
 * @param  setpoint The desired setpoint (e.g., target temperature).
 * @param  feedback The current feedback value (e.g., actual temperature).
 * @return The controller output, scaled to 16-bit integer range.
 */
int16_t fastPidStep(FastPid *pid, int16_t setpoint, int16_t feedback);

/**
 * @brief  Clear or reset internal PID accumulators and previous error terms.
 * @param  pid Pointer to the FastPid struct.
 */
void fastPidClear(FastPid *pid);

/**
 * @brief  Check if the PID configuration is in error.
 * @param  pid Pointer to the FastPid struct.
 * @return True if there is a configuration error, false if OK.
 */
bool fastPidHasConfigError(FastPid *pid);

bool fastPidSetAntiWindup(FastPid *pid, FastPidAntiWindup mode, float bcGain);

#endif /* FAST_PID_H */
