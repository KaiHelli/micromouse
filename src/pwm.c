#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include "clock.h"
#include "pwm.h"
#include "IOconfig.h"

// Prescalers for PWM (PTCKPS bits). 1 (00), 4 (01), 16 (10), 64 (11).
static const uint16_t prescalers[] = {1, 4, 16, 64};

/**
 * We store the PTPER*2 (the register range for 0..100% DC)
 * for PWM1 in pwmFrequency[0] and for PWM2 in pwmFrequency[1].
 */
static uint16_t pwmFrequency[2] = {0, 0};

/**
 * Store duty cycle fractions (range 0.0f..1.0f).
 * PWM1 has up to 3 independent channels, PWM2 has 1 channel.
 */
static uint8_t pwmDC1[3] = {0, 0, 0};  // for PWM module #1, channels 1..3
static uint8_t pwmDC2    = 0;                  // for PWM module #2, channel 1

void setupPWM1()
{
    /* PWM1, configured to 1kHz, based on fcy = 40.000 MIPS, Tcycle=25nsec
     * 1ms/25nsec = 40000 (doesn't fit in 15 bits)
     * Therefore, we use a pre-scaler and end up with 10000
     */
    P1TCONbits.PTEN = 0; // Switch off PWM generator
    
    setPWMFrequency(1, 20000); // Sets prescaler and P1TPER to obtain 1kHz.
    
    PWM1CON1bits.PMOD1 = 1; // set PWM unit 1 to independent mode

    PWM1CON1bits.PEN1H = 1; // enable PWM driver PWM1H1
    PWM1CON1bits.PEN2H = 1; // enable PWM driver PWM1H2
    PWM1CON1bits.PEN3H = 1; // enable PWM driver PWM1H3
    PWM1CON1bits.PEN1L = 0; // disable PWM driver
    PWM1CON1bits.PEN2L = 0; // disable PWM driver
    PWM1CON1bits.PEN3L = 0; // disable PWM driver

    P1TCONbits.PTEN = 1; // Switch on PWM generator
    // P1DC1 = .1*PWM_1KHZ; //to get 100% DC, you need to write twice the PER Value (2*10000)

    // Leave channels disabled for now
    P1DC1 = 0;
    P1DC2 = 0;
    P1DC3 = 0;
}

void setupPWM2()
{
    /* PWM2, configured to 1kHz, based on fcy = 40.000 MIPS, Tcycle=25nsec
     * 1ms/25nsec = 40000 (fits in 15 bits)
     * of course, we could use a pre-scaler and end up somewhere else
     */
    P2TCONbits.PTEN = 0; // Switch off PWM generator
    P2TCONbits.PTCKPS = 0b01; // Sets prescaler, available are 1(00),4(01),16(10) or 64(11)
    
    setPWMFrequency(2, 1000); // Sets prescaler and P1TPER to obtain 1kHz.
    
    PWM2CON1bits.PMOD1 = 1; // set PWM unit 2 to independent mode

    PWM2CON1bits.PEN1H = 1; // enable PWM driver PWM1H1
    PWM2CON1bits.PEN1L = 0; // disable PWM driver

    P2TCONbits.PTEN = 1; // Switch on PWM generator
    // P2DC1 = 0.001*PWM_1KHZ; //to get 100% DC, you need to write twice the PER Value (2*40000)

    // Leave channels disabled for now
    P2DC1 = 0;
}


/**
 * @brief Helper to apply the stored duty-cycle fractions to hardware
 *        after we have updated pwmFrequency[].
 *
 * @param pwmModule 1 or 2
 */
static void recalcPWMDutyCycles(uint8_t pwmModule)
{
    switch (pwmModule) {
        case 1:
            // P1DCx must be (fraction of (2 * PTPER))
            P1DC1 = (uint16_t) (((uint32_t) pwmDC1[0] * pwmFrequency[0]) / 100);
            P1DC2 = (uint16_t) (((uint32_t) pwmDC1[1] * pwmFrequency[0]) / 100);
            P1DC3 = (uint16_t) (((uint32_t) pwmDC1[2] * pwmFrequency[0]) / 100);
            break;

        case 2:
            // Only channel 1 in this example
            P2DC1 = (uint16_t) (((uint32_t) pwmDC2 * pwmFrequency[1]) / 100);
            break;

        default:
            // Should never happen if called correctly
            break;
    }
}


/**
 * @brief Sets the PWM frequency for either PWM1 or PWM2.
 *
 * @param pwmModule   Which PWM module to configure (1 or 2).
 * @param desiredFreq The desired PWM frequency in Hz.
 * @return 0 if successful, -1 if no prescaler could be found or invalid module.
 *
 * @note 
 *   - PTPER is 15 bits, so it must be <= 0x7FFF (32767).
 *   - We set pwmFrequency[module - 1] to 2�PTPER so that writing
 *     that value to P1DCx or P2DC1 yields 100% duty cycle.
 */
int8_t setPWMFrequency(uint8_t pwmModule, uint32_t desiredFreq)
{
    // Example: If your device runs at 40 MIPS => FCY = 40e6
    // or compute from TCY_NSEC => FCY = (uint32_t)(1e9 / TCY_NSEC)
    uint32_t fcy = FCY;

    // We'll search for a suitable prescaler and PTPER that fits in 15 bits
    uint16_t chosenPTPER = 0;
    uint16_t chosenPrescalerBits = 0;
    uint8_t foundMatch = 0;

    // Try each prescaler until we find a valid match
    for (uint8_t i = 0; i < 4; i++) {
        uint16_t presc = prescalers[i];
        // PTPER = fcy / (desiredFreq * presc)
        // Use 32-bit arithmetic to avoid overflow
        uint32_t candidate = fcy / (desiredFreq * (uint32_t) presc * 2UL);

        if (candidate <= 0x7FFF) {
            chosenPTPER = (uint16_t) candidate;
            chosenPrescalerBits = i;
            foundMatch = 1;
            break;
        }
    }

    if (!foundMatch) {
        // Desired frequency is too high or too low for 15-bit PTPER with these prescalers
        return -1;
    }

    uint16_t wasEnabled;
    // Now configure the PWM module (1 or 2)
    switch (pwmModule) {
    case 1:
        // Remember if PWM module #1 was enabled
        wasEnabled = (uint16_t)(P1TCONbits.PTEN);
        // Disable before reconfig
        P1TCONbits.PTEN = 0;
        // Set prescaler
        P1TCONbits.PTCKPS = chosenPrescalerBits;
        // Set period
        P1TPER = chosenPTPER;
        // Update stored "maximum DC" for module #1
        pwmFrequency[0] = chosenPTPER * 2;
        // Reapply the stored duty-cycle fractions
        recalcPWMDutyCycles(1);
        // Re-enable only if it was enabled before
        if (wasEnabled) {
            P1TCONbits.PTEN = 1;
        }
        break;
    case 2:
        // Remember if PWM module #2 was enabled
        wasEnabled = (uint16_t)(P2TCONbits.PTEN);
        // Disable before reconfig
        P2TCONbits.PTEN = 0;
        // Set prescaler
        P2TCONbits.PTCKPS = chosenPrescalerBits;
        // Set period
        P2TPER = chosenPTPER;
        // Update stored "maximum DC" for module #2
        pwmFrequency[1] = chosenPTPER * 2;
        // Reapply the stored duty-cycle fraction
        recalcPWMDutyCycles(2);
        // Re-enable only if it was enabled before
        if (wasEnabled) {
            P2TCONbits.PTEN = 1;
        }
        break;
    default:
        // Invalid PWM module number
        return -1;
    }

    // Successfully configured
    return 0;
}

/**
 * @brief Sets the duty cycle (as fraction 0.0..1.0) for a PWM module and channel.
 *
 * @param pwmModule  Which PWM module to configure (1 or 2).
 * @param channel    Channel number within that module (e.g. 1,2,3 for PWM1, or 1 for PWM2).
 * @param fraction   Duty cycle fraction (0 = 0%, 100 = 100%).
 * @return 0 if successful, -1 if invalid module/channel.
 */
int8_t setPWMDutyCycle(uint8_t pwmModule, uint8_t channel, uint8_t fraction)
{
    // Clamp fraction to 0..1
    if (fraction < 0) fraction = 0;
    if (fraction > 100) fraction = 100;

    switch (pwmModule) {
        case 1:
            switch (channel) {
                case 1:
                    pwmDC1[0] = fraction;
                    P1DC1 = (uint16_t) (((uint32_t) fraction * pwmFrequency[0]) / 100);
                    return 0;
                case 2:
                    pwmDC1[1] = fraction;
                    P1DC2 = (uint16_t) (((uint32_t) fraction * pwmFrequency[0]) / 100);
                    return 0;
                case 3:
                    pwmDC1[2] = fraction;
                    P1DC3 = (uint16_t) (((uint32_t) fraction * pwmFrequency[0]) / 100);
                    return 0;
                default:
                    return -1;
            }

        case 2:
            // Only channel 1 is available for pwm module 2
            if (channel == 1) {
                pwmDC2 = fraction;
                P2DC1 = (uint16_t) (((uint32_t) fraction * pwmFrequency[1]) / 100);
                return 0;
            } else {
                return -1;
            }

        default:
            return -1;
    }
}

int8_t setPWMState(uint8_t pwmModule, uint8_t channel, bool state)
{
    // Validate the module number
    if ((pwmModule < 1) || (pwmModule > 2)) {
        return -1; // Invalid module
    }

    // Validate the channel for each module
    if (pwmModule == 1) {
        // PWM1 supports channels 1..3
        if ((channel < 1) || (channel > 3)) {
            return -1; // Invalid channel for PWM1
        }
    } else {
        // pwmModule == 2
        // PWM2 supports only channel 1 in this example
        if (channel != 1) {
            return -1; // Invalid channel for PWM2
        }
    }

    // If inputs are valid, set the corresponding PEN bit.
    // Because 'state' is a bool, we can assign it directly.
    switch (pwmModule) {
        case 1:
            switch (channel) {
                case 1: PWM1CON1bits.PEN1H = state; break;
                case 2: PWM1CON1bits.PEN2H = state; break;
                case 3: PWM1CON1bits.PEN3H = state; break;
            }
            break;

        case 2:
            // Only channel 1 is valid for PWM2
            PWM2CON1bits.PEN1H = state;
            break;
    }

    return 0; // Success
}

