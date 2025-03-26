#include "clock.h"
#include "timers.h"
#include "IOconfig.h"
#include "motorEncoders.h"
#include "pwm.h"
#include "uart.h"
#include "imu.h"
#include "interrupts.h"
#include "dma.h"
#include "rtttl.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <xc.h>

void initTimer1(uint16_t period, uint16_t prescaler)
{
    // unsigned TimerControlValue;

    T1CON = 0; // ensure Timer 1 is in reset state
    // TimerControlValue=T1CON;

    T1CONbits.TCKPS = prescaler & 0b11; // FCY divide by 1/8/64/256: e.g tick = 64*37.5ns = 2.4us (Tcycle=37.5ns)
    T1CONbits.TCS = 0; // select internal FCY clock source
    T1CONbits.TGATE = 0; // gated time accumulation disabled
    T1CONbits.TON = 0; // leave timer disabled initially

    TMR1 = 0; // reset timer counter
    PR1 = period; // set timer period register

    IFS0bits.T1IF = 0; // reset timer interrupt flag
    IPC0bits.T1IP = IP_TIMER1; // set timer interrupt priority level
    IEC0bits.T1IE = 1; // enable timer interrupt
}

void initTimer2(uint16_t period, uint16_t prescaler)
{
    // unsigned TimerControlValue;

    T2CON = 0; // ensure Timer2 is in reset state
    // TimerControlValue=T1CON;

    T2CONbits.TCKPS = prescaler & 0b11; // FCY divide by 1/8/64/256: e.g tick = 64*37.5ns = 2.4us (Tcycle=37.5ns)
    T2CONbits.TCS = 0; // select internal FCY clock source
    T2CONbits.TGATE = 0; // gated time accumulation disabled
    T2CONbits.T32 = 0; // set timer to 16bit mode
    T2CONbits.TON = 0; // leave timer disabled initially

    TMR2 = 0; // reset timer counter
    PR2 = period; // set timer period register

    IFS0bits.T2IF = 0; // reset timer interrupt flag
    IPC1bits.T2IP = IP_TIMER2; // set timer interrupt priority level
    IEC0bits.T2IE = 1; // enable timer interrupt
}

void initTimer3(uint16_t period, uint16_t prescaler)
{
    // unsigned TimerControlValue;

    T3CON = 0; // ensure Timer3 is in reset state
    // TimerControlValue=T1CON;

    T3CONbits.TCKPS = prescaler & 0b11; // FCY divide by 1/8/64/256: e.g tick = 64*37.5ns = 2.4us (Tcycle=37.5ns)
    T3CONbits.TCS = 0; // select internal FCY clock source
    T3CONbits.TGATE = 0; // gated time accumulation disabled
    T3CONbits.TON = 0; // leave timer disabled initially

    TMR3 = 0; // reset timer counter
    PR3 = period; // set timer period register

    IFS0bits.T3IF = 0; // reset timer interrupt flag
    IPC2bits.T3IP = IP_TIMER3; // set timer interrupt priority level
    IEC0bits.T3IE = 1; // enable timer interrupt
}

void initTimer4(uint16_t period, uint16_t prescaler)
{
    // unsigned TimerControlValue;

    T4CON = 0; // ensure Timer4 is in reset state
    // TimerControlValue=T1CON;

    T4CONbits.TCKPS = prescaler & 0b11; // FCY divide by 1/8/64/256: e.g tick = 64*37.5ns = 2.4us (Tcycle=37.5ns)
    T4CONbits.TCS = 0; // select internal FCY clock source
    T4CONbits.TGATE = 0; // gated time accumulation disabled
    T4CONbits.TON = 0; // leave timer disabled initially

    TMR4 = 0; // reset timer counter
    PR4 = period; // set timer period register

    IFS1bits.T4IF = 0; // reset timer interrupt flag
    IPC6bits.T4IP = IP_TIMER4; // set timer interrupt priority level
    IEC1bits.T4IE = 1; // enable timer interrupt
}

void initTimer5(uint16_t period, uint16_t prescaler)
{
    // unsigned TimerControlValue;

    T5CON = 0; // ensure Timer5 is in reset state
    // TimerControlValue=T1CON;

    T5CONbits.TCKPS = prescaler & 0b11; // FCY divide by 1/8/64/256: e.g tick = 64*37.5ns = 2.4us (Tcycle=37.5ns)
    T5CONbits.TCS = 0; // select internal FCY clock source
    T5CONbits.TGATE = 0; // gated time accumulation disabled
    T5CONbits.TON = 0; // leave timer disabled initially

    TMR5 = 0; // reset timer counter
    PR5 = period; // set timer period register

    IFS1bits.T5IF = 0; // reset timer interrupt flag
    IPC7bits.T5IP = IP_TIMER5; // set timer interrupt priority level
    IEC1bits.T5IE = 1; // enable timer interrupt
}

void initTimer32Combined(uint32_t period, uint16_t prescaler)
{
    // ensure Timer 2 and Timer 3 are in reset state
    T2CON = 0;
    T3CON = 0;

    T2CONbits.T32 = 1; // select 32bit timer mode
    T2CONbits.TCKPS = prescaler & 0b11; // set prescaler
    T2CONbits.TCS = 0; // select internal FCY clock source
    T2CONbits.TGATE = 0; // gated time accumulation disabled
    T2CONbits.TON = 0; // leave timer disabled initially

    TMR3HLD = 0; // clear upper bits of 32bit timer (msw)
    TMR2 = 0;    // clear lower bits of 32bit timer (lsw)

    // Set period registers (PR3: upper, PR2: lower)
    PR3 = (uint16_t)(period >> 16); // Upper 16 bits of 32-bit period (msw)
    PR2 = (uint16_t)period; // Lower 16 bits of 32-bit period (lsw)

    IFS0bits.T3IF = 0; // reset timer interrupt flag
    IPC2bits.T3IP = IP_TIMER32; // set interrupt priority level
    IEC0bits.T3IE = 1; // enable timer interrupt

    // NOTE: calls _T3Interrupt(void) on interrupt
}

void initTimer54Combined(uint32_t period, uint16_t prescaler)
{
    // ensure Timer 4 and Timer 5 are in reset state
    T4CON = 0;
    T5CON = 0;

    T4CONbits.T32 = 1; // select 32bit timer mode
    T4CONbits.TCKPS = prescaler & 0b11; // set prescaler
    T4CONbits.TCS = 0; // select internal FCY clock source
    T4CONbits.TGATE = 0; // gated time accumulation disabled
    T4CONbits.TON = 0; // leave timer disabled initially

    TMR5HLD = 0; // clear upper bits of 32bit timer (msw)
    TMR4 = 0;    // clear lower bits of 32bit timer (lsw)

    // Set period registers (PR3: upper, PR2: lower)
    PR5 = (uint16_t)(period >> 16); // Upper 16 bits of 32-bit period (msw)
    PR4 = (uint16_t)period; // Lower 16 bits of 32-bit period (lsw)

    IFS1bits.T5IF = 0; // reset timer interrupt flag
    IPC7bits.T5IP = IP_TIMER5; // set timer interrupt priority level
    IEC1bits.T5IE = 1; // enable timer interrupt

    // NOTE: calls _T5Interrupt(void) on interrupt
}

void setTimerInterruptState(Timer_t timer, bool state) {
    switch (timer) { 
        case TIMER_1:
            IEC0bits.T1IE = state;
            break;
        case TIMER_2:
            IEC0bits.T2IE = state;
            break;
        case TIMER_3:
            IEC0bits.T3IE = state;
            break;
        case TIMER_4:
            IEC1bits.T4IE = state;
            break;
        case TIMER_5:
            IEC1bits.T5IE = state;
            break;
        case TIMER_32_COMBINED:
            IEC0bits.T3IE = state;
            break;
        case TIMER_54_COMBINED:
            IEC1bits.T5IE = 1;
            break;

    }
}

void setTimerState(Timer_t timer, bool state) {
    switch (timer) { 
        case TIMER_1:
            T1CONbits.TON = state;
            break;
        case TIMER_2:
            T2CONbits.TON = state;
            break;
        case TIMER_3:
            T3CONbits.TON = state;
            break;
        case TIMER_4:
            T4CONbits.TON = state;
            break;
        case TIMER_5:
            T5CONbits.TON = state;
            break;
        case TIMER_32_COMBINED:
            T2CONbits.TON = state;
            break;
        case TIMER_54_COMBINED:
            T4CONbits.TON = state;
            break;
    }
}

/**
 * @brief Initializes a hardware timer with a specified period in microseconds.
 *
 * This function calculates the appropriate timer period and prescaler for the given
 * timer number and desired time in milliseconds. It supports both 16-bit and 32-bit
 * timers and calls the corresponding initialization function.
 *
 * @param timeInUs The desired timer period in microseconds. Must be within the
 *                 supported range for the selected timer and prescaler combination.
 * @param timer The timer to configure
 *
 * @note For 32-bit timers, the Timer2 and Timer3 or Timer4 and Timer5 modules are combined.
 *       Ensure that Timer2 and Timer3 or Timer4 and Timer5 are not independently configured 
 *       when using Timer32 or Timer54.
 */
int16_t initTimerInUs(Timer_t timer, uint64_t timeInUs)
{
    // Prescaler options
    const uint16_t prescaler_options[] = { 1, 8, 64, 256 };

    // Base frequency in Hz depending on the oscillator
    const uint32_t fcy = FCY;

    // Max count values for each type of timer
    const uint16_t max_count_16 = 0xffff; // 2^16 - 1 = 65_535
    const uint32_t max_count_32 = 0xffffffff; // 2^32 - 1 = 4_294_967_295

    // Max count based on timer type
    uint32_t max_count = (timer == TIMER_32_COMBINED || timer == TIMER_54_COMBINED) ? max_count_32 : max_count_16;

    // Timer parameters to calculate
    uint32_t count;
    uint16_t prescaler;

    uint8_t match_found = 0;

    // Loop through prescaler options
    for (uint16_t i = 0; i < 4; i++) {
        uint32_t adj_fcy = fcy / prescaler_options[i];

        // Calculate the number of cycles needed to achieve the given period
        // period is given in us
        // Approach: convert frequency to kHz and multiply by the number of ms

        // for 16-bit timers 64bit is safe as timeInMs <= 630_000us and adj_fcy <= 40_000_000 so log2(630_000*40_000_000) = 44.52 bits
        // for 32-bit timers 64bit is safe as timeInMs <= 41_232_000_000us and adj_fcy <= 40_000_000 so log2(41_232_000_000*40_000_000) = 60.52 bits
        uint64_t required_ticks = ((uint64_t)timeInUs * adj_fcy) / 1000000ULL;

        if (required_ticks <= max_count) {
            prescaler = i;
            count = (uint32_t)required_ticks;

            match_found = 1;
            break;
        }
    }

    // In case no prescaler matched, don't initialize any timer.
    if (match_found == 0) {
        return -1;
    }

    switch (timer) {
    case TIMER_1:
        initTimer1((uint16_t)count, prescaler);
        break;
    case TIMER_2:
        initTimer2((uint16_t)count, prescaler);
        break;
    case TIMER_3:
        initTimer3((uint16_t)count, prescaler);
        break;
    case TIMER_4:
        initTimer4((uint16_t)count, prescaler);
        break;
    case TIMER_5:
        initTimer5((uint16_t)count, prescaler);
        break;
    case TIMER_32_COMBINED:
        initTimer32Combined(count, prescaler);
        break;
    case TIMER_54_COMBINED:
        initTimer54Combined(count, prescaler);
        break;
    default:
        return -1;
    }

    // Minimal timer interrupt time
    // In theory 37.5ns but this will likely break as the interrupt is not finished yet.
    // Maximal timer interrupt time (16-bit timers)
    // 37.5 * 10^{-9} * 256 * 65536 = 0.6291456s
    // Maximal timer interrupt time (32-bit timers)
    // 37.5 * 10^{-9} * 256 * (2^{32} - 1) = 41_231.686032s = 687.1947672min = 11.45324612h
    // For longer times: Add a separate counter in software to wait x interrupts of y ms.

    return 0;
}

/**
 * @brief Initializes a hardware timer with a specified period in milliseconds.
 *
 * Convenience wrapper around initTimerInUs().
 *
 * @param timeInMs The desired timer period in milliseconds.
 * @param timer    The timer to configure.
 *
 */
int16_t initTimerInMs(Timer_t timer, uint32_t timeInMs)
{
    // Convert from ms to us using 64-bit arithmetic
    uint64_t timeInUs = (uint64_t)timeInMs * 1000ULL;

    // Now call the 64-bit microsecond-based function
    return initTimerInUs(timer, timeInUs);
}


static volatile TimerCallback_t timerCallbacks[NUM_TIMERS][CALLBACK_BUFFER_SIZE];
static volatile uint8_t registeredTimerCallbacks[NUM_TIMERS];

// CAVEAT:  There is one race condition, when a higher priority interrupt calls 
//          registerCallback while being in the while-loop of the generalTimerISR()
//          callback. TODO: fix.
int16_t registerTimerCallback(Timer_t timer, TimerCallback_t callback) {
    // In case a combined timer is used, select the respective timer that handles 
    // the ISR.
    if (timer == TIMER_32_COMBINED) {
        timer = TIMER_3;
    }
    if (timer == TIMER_54_COMBINED) {
        timer = TIMER_5;
    }

    // Check if there buffer of interrupt callbacks is already full.
    if (registeredTimerCallbacks[timer] == CALLBACK_BUFFER_SIZE) {
        return -1;
    }
    
    // Temporarily disable timer interrupts during modifying callbacks.
    setTimerInterruptState(timer, false);

    timerCallbacks[timer][registeredTimerCallbacks[timer]] = callback;
    registeredTimerCallbacks[timer]++;
    
    // Enable the timer itself, if we are the first callback.
    if (registeredTimerCallbacks[timer] == 1) {
        setTimerState(timer, true);
    }
    
    // Re-enable timer interrupts.
    setTimerInterruptState(timer, true);
    
    return 0;
}

int16_t removeTimerCallback(TimerCallback_t callback) {
    // TODO
    return 0;
}


void clearTimerCallbacks(Timer_t timer) {
    
    // We clear the callbacks, so the timer can be disabled.
    setTimerState(timer, false);
    registeredTimerCallbacks[timer] = 0;
}


static void generalTimerISR(Timer_t timer) {
    uint8_t i = 0;
    while (i < registeredTimerCallbacks[timer]) {
        
        
        TimerCallback_t callback = timerCallbacks[timer][i];
        uint8_t status = callback();

        if (status == 0) {
            // Guard from register_callback being called from an interrupt with
            // higher priority while modifiyng the callback buffers below.
            
            // Save current interrupt enable state
            uint16_t state = __builtin_get_isr_state();
            
            // Disable interrupts
            __builtin_disable_interrupts();
            
            registeredTimerCallbacks[timer]--;
            
            if (registeredTimerCallbacks[timer] == 0) {
                // Disable the timer itself, if there are no callbacks left.
                setTimerState(timer, false);
            }

            // Only swap if we're not already at the last position
            if (i < registeredTimerCallbacks[timer]) {
                timerCallbacks[timer][i] = timerCallbacks[timer][registeredTimerCallbacks[timer]];
                // Do not increment i, as we've moved a new callback into position i
            }
            
            
            // Restore interrupt enable state
            __builtin_set_isr_state(state);
            // If removed callback was already last, just continue without incrementing i
        } else {
            i++;  // move to next callback only if no removal
        }
    }
}


void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0;

    generalTimerISR(TIMER_1);
}


/* ISR for Timer2 */
void __attribute__((__interrupt__, auto_psv)) _T2Interrupt(void)
{
    IFS0bits.T2IF = 0; // reset Timer 2 interrupt flag
    
    generalTimerISR(TIMER_2);
}

/* ISR for Timer3 or when Timer2 and Timer3 are combined */
/*
void __attribute__((__interrupt__, auto_psv)) _T3Interrupt(void)
{
    IFS0bits.T3IF = 0;           // reset Timer 3 interrupt flag

    static uint8_t counter = 0;

    P1DC2 = ((sin((double) counter/0xFF * 2 * M_PI) + 1) / 2) * PWM_1KHZ;

    counter++;

    if (counter == 0xFF) {
        counter = 0;
    }
}
*/

/* ISR for Timer3 or when Timer2 and Timer3 are combined */
void __attribute__((__interrupt__, auto_psv)) _T3Interrupt(void)
{
    IFS0bits.T3IF = 0; // reset Timer 3 interrupt flag
    
    generalTimerISR(TIMER_3);
}

/* ISR for Timer4 */
void __attribute__((__interrupt__, auto_psv)) _T4Interrupt(void)
{
    IFS1bits.T4IF = 0; // reset Timer 4 interrupt flag
    
    generalTimerISR(TIMER_4);
}

/* ISR for Timer5 or when Timer4 and Timer5 are combined */
void __attribute__((__interrupt__, auto_psv)) _T5Interrupt(void)
{
    IFS1bits.T5IF = 0; // reset Timer 5 interrupt flag
    
    generalTimerISR(TIMER_5);
}