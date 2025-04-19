// Source based on: https://github.com/mike-matera/FastPID

#include "fastPID.h"
#include <stddef.h>

/*
 * Internal helper function to set the configError flag
 * and reset tuning parameters if an invalid parameter is detected.
 */
static void fastPidSetConfigError(FastPid *pid) {
    pid->configError = true;
    pid->pParameter = 0;
    pid->iParameter = 0;
    pid->dParameter = 0;
    pid->fParameter = 0;
}

/*
 * Internal helper function to convert a floating-point number
 * into a fixed-point parameter. This helps avoid overflow in
 * large scale computations by controlling scaling.
 */
static uint32_t fastPidFloatToParam(FastPid *pid, float value) {
    // Reject out-of-range or negative inputs
    if (value > PARAM_MAX || value < 0) {
        fastPidSetConfigError(pid);
        return 0;
    }

    uint32_t param = (uint32_t)(value * PARAM_MULT);

    // If user provided a nonzero value but it scales to 0, flag as error
    if ((value != 0.0f) && (param == 0)) {
        fastPidSetConfigError(pid);
        return 0;
    }

    return param;
}

/*
 * Initialize internal fields to safe defaults. Useful to call before any configuration,
 * or if you want to reset the PID controller?s accumulated state.
 */
void fastPidInit(FastPid *pid) {
    if (pid == NULL) {
        return;
    }
    pid->pParameter     = 0;
    pid->iParameter     = 0;
    pid->dParameter     = 0;
    pid->fParameter     = 0;
    pid->outputMax      = 0;
    pid->outputMin      = 0;
    pid->awMode         = FASTPID_AW_NONE;
    pid->bcParameter    = 0;
    pid->configError    = false;

    pid->lastSetpoint   = 0;
    pid->lastOutput     = 0;
    pid->integralSum    = 0;
    pid->lastError      = 0;
}

/*
 * Reset runtime integrator and error states. This is often called when
 * the controller is being reinitialized or if the system has significantly changed.
 */
void fastPidClear(FastPid *pid) {
    if (pid == NULL) {
        return;
    }
    pid->lastSetpoint = 0;
    pid->lastOutput   = 0;
    pid->integralSum  = 0;
    pid->lastError    = 0;
}

/*
 * Configure the PID controller all at once with gains, loop frequency, output bits, and sign.
 */
bool fastPidConfigure(FastPid *pid, float kp, float ki, float kd, float kf, float hz, int bits, bool sign) {
    if (pid == NULL) {
        return false;
    }
    // Clear state and reset configError
    fastPidClear(pid);
    pid->configError = false;

    fastPidSetCoefficients(pid, kp, ki, kd, kf, hz);
    fastPidSetOutputConfig(pid, bits, sign);

    // Return the overall config status
    return !pid->configError;
}

/*
 * Set the three main PID gains, factoring in the control loop frequency (hz).
 *   kp = proportional gain
 *   ki = integral gain
 *   kd = derivative gain
 *   kf = feed-forward gain
 * The code uses 1/hz for the integral term and hz for the derivative term.
 */
bool fastPidSetCoefficients(FastPid *pid, float kp, float ki, float kd, float kf, float hz) {
    if (pid == NULL) {
        return false;
    }
    pid->pParameter = fastPidFloatToParam(pid, kp);
    pid->iParameter = fastPidFloatToParam(pid, ki / hz);
    pid->dParameter = fastPidFloatToParam(pid, kd * hz);
    pid->fParameter = fastPidFloatToParam(pid, kf);

    return !pid->configError;
}

/*
 * Set the resolution (in bits) of the PID output, and whether the output is signed.
 * For instance, bits=16 means output can range up to 16 bits (0 to 65535 if unsigned).
 */
bool fastPidSetOutputConfig(FastPid *pid, int bits, bool sign) {
    if (pid == NULL) {
        return false;
    }

    if (bits > 16 || bits < 1) {
        fastPidSetConfigError(pid);
    } else {
        /*
         * The math below ensures that outputMax and outputMin are scaled
         * according to the internal PARAM_MULT and bit range. 
         */
        if (bits == 16) {
            pid->outputMax = ((0xFFFFULL >> (17 - bits)) * PARAM_MULT);
        } else {
            pid->outputMax = ((0xFFFFULL >> (16 - bits)) * PARAM_MULT);
        }

        if (sign) {
            pid->outputMin = -(((0xFFFFULL >> (17 - bits)) + 1) * PARAM_MULT);
        } else {
            pid->outputMin = 0;
        }
    }
    return !pid->configError;
}

/*
 * Set the user-defined output range [min, max].
 * Useful if you need a specific saturated output (e.g., [-1000, 1000]).
 */
bool fastPidSetOutputRange(FastPid *pid, int16_t min, int16_t max) {
    if (pid == NULL) {
        return false;
    }
    if (min >= max) {
        fastPidSetConfigError(pid);
        return false;
    }
    pid->outputMin = (int64_t)min * PARAM_MULT;
    pid->outputMax = (int64_t)max * PARAM_MULT;
    return !pid->configError;
}

/*
 * This is the main PID calculation step called in the control loop.
 *   setpoint: the desired value
 *   feedback: the measured or actual value
 * Returns a 16-bit integer that should be used to drive actuators or
 * other control outputs. The internal accumulation and error state
 * is updated every time you call this function.
 */
int16_t fastPidStep(FastPid *pid, int16_t setpoint, int16_t feedback) {
    if (pid == NULL) {
        return 0;
    }

    // Calculate error (int32_t needed to avoid overflow of int16 + int16)
    int32_t error = (int32_t)setpoint - (int32_t)feedback;

    int32_t proportionalTerm = 0;
    int32_t integralTerm     = 0;
    int32_t derivativeTerm   = 0;
    int32_t feedForwardTerm  = 0;
    
    // Remember ?I so we can undo it if output is saturated
    int64_t  integIncr = 0;                


    // Proportional
    if (pid->pParameter != 0) {
        // pParameter is uint32, error is int32 -> result is int64 for safety
        proportionalTerm = (int32_t)pid->pParameter * error;
    }

    // Integral
    if (pid->iParameter != 0) {
        // Accumulate integral in integralSum (64-bit to avoid overflow).
        // error is int32, iParameter is uint32 -> multiply -> int64
        integIncr        = (int64_t)error * (int64_t)pid->iParameter;
        pid->integralSum += integIncr;

        // Limit sum to 32-bit signed to avoid overflow
        if (pid->integralSum > INTEG_MAX) {
            pid->integralSum = INTEG_MAX;
        } else if (pid->integralSum < INTEG_MIN) {
            pid->integralSum = INTEG_MIN;
        }

        integralTerm = (int32_t)(pid->integralSum);
    }

    // Derivative
    if (pid->dParameter != 0) {
        // Typically derivative is difference of errors, but this implementation
        // also subtracts the difference in setpoints to reduce derivative kick.
        int32_t deriv = (error - pid->lastError) - (int32_t)(setpoint - pid->lastSetpoint);

        // Limit derivative to 16-bit range
        if (deriv > DERIV_MAX) {
            deriv = DERIV_MAX;
        } else if (deriv < DERIV_MIN) {
            deriv = DERIV_MIN;
        }

        derivativeTerm = (int32_t)pid->dParameter * deriv;

        // Store for next iteration
        pid->lastSetpoint = setpoint;
        pid->lastError    = error;
    }
    
    if (pid->fParameter != 0) {
        feedForwardTerm = (int32_t)pid->fParameter * (int32_t)setpoint;
    }

    // Combine P, I, D terms (up to 34-bit internally)
    int64_t pidOnly      = (int64_t)proportionalTerm + (int64_t)integralTerm + (int64_t)derivativeTerm;
    int64_t unsatOutput  = pidOnly + (int64_t)feedForwardTerm;
    int64_t output       = unsatOutput;

    // Saturate output if beyond configured range
    if (output > pid->outputMax) {
        output = pid->outputMax;
    } else if (output < pid->outputMin) {
        output = pid->outputMin;
    }

    // Clamp Integral correction:
    if (pid->awMode == FASTPID_AW_CLAMP && pid->iParameter != 0) {
        bool satHigh = (output == pid->outputMax);
        bool satLow  = (output == pid->outputMin);

        // if controller is saturated and error pushes further into saturation,
        // cancel the integrator increment applied this cycle
        if ((satHigh && error > 0) || (satLow && error < 0)) {
            pid->integralSum -= integIncr;

            /* relimit after removal in case bounds were previously hit */
            if (pid->integralSum > INTEG_MAX)      pid->integralSum = INTEG_MAX;
            else if (pid->integralSum < INTEG_MIN) pid->integralSum = INTEG_MIN;
        }
    }
    
    // Back-Calculation Integral correction:  deltaI = Kt · (u_sat - u_unsat)  (FF auto-cancels)
    if (pid->awMode == FASTPID_AW_BACKCALC && pid->bcParameter != 0) {
        int64_t trackingErr = output - unsatOutput;
        pid->integralSum   += (int64_t)pid->bcParameter * trackingErr;

        if (pid->integralSum > INTEG_MAX)      pid->integralSum = INTEG_MAX;
        else if (pid->integralSum < INTEG_MIN) pid->integralSum = INTEG_MIN;
    }
    
    // Convert back to int16 from the fixed-point representation
    // Right-shift by PARAM_SHIFT, applying half-bit rounding
    int16_t result = (int16_t)(output >> PARAM_SHIFT);

    // If the bit below the shift is 1, we do a +1 for rounding
    if (output & (0x1ULL << (PARAM_SHIFT - 1))) {
        result++;
    }

    pid->lastOutput = result;
    return result;
}

/*
 * Returns whether there has been a configuration error (true/false).
 * This might happen if out-of-range parameters were given.
 */
bool fastPidHasConfigError(FastPid *pid) {
    if (pid == NULL) {
        return true;
    }
    return pid->configError;
}

bool fastPidSetAntiWindup(FastPid *pid, FastPidAntiWindup mode, float bcGain) {
    if (pid == NULL) { return false; }

    pid->awMode = mode;

    if (mode == FASTPID_AW_BACKCALC) {
        pid->bcParameter = fastPidFloatToParam(pid, bcGain);
    } else {
        pid->bcParameter = 0;
    }
    return !pid->configError;
}