#include "clock.h"
#include "myTimers.h"
#include "IOconfig.h"
#include "motorEncoders.h"
#include "myPWM.h"
#include "serialComms.h"
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

    TMR1 = 0; // reset Timer1 counter
    PR1 = period; // set Timer1 period register ()

    IFS0bits.T1IF = 0; // reset Timer1 interrupt flag
    IPC0bits.T1IP = IP_TIMER1; // set Timer1 interrupt priority level
    IEC0bits.T1IE = 1; // enable Timer1 interrupt
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

    TMR2 = 0; // reset Timer2 counter
    PR2 = period; // set Timer2 period register ()

    IFS0bits.T2IF = 0; // reset Timer2 interrupt flag
    IPC1bits.T2IP = IP_TIMER2; // set Timer2 interrupt priority level
    IEC0bits.T2IE = 1; // enable Timer2 interrupt
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

    TMR3 = 0;
    PR3 = period; // set Timer 1 period register ()

    IFS0bits.T3IF = 0; // reset Timer 1 interrupt flag
    IPC2bits.T3IP = IP_TIMER3; // set Timer1 interrupt priority level
    IEC0bits.T3IE = 1; // enable Timer 1 interrupt
}

void initTimer32Combined(uint32_t period, uint16_t prescaler)
{
    // ensure Timer 2 and Timer 3 are in reset state
    T2CON = 0;
    T3CON = 0;

    T2CONbits.T32 = 1; // select 32bit timer mode
    T2CONbits.TCS = 0; // select internal FCY clock source
    T2CONbits.TGATE = 0; // gated time accumulation disabled
    T2CONbits.TCKPS = prescaler & 0b11; // set prescaler

    TMR3 = 0; // clear upper bits of 32bit timer (msw)
    TMR2 = 0; // clear lower bits of 32bit timer (lsw)

    // Set period registers (PR3: upper, PR2: lower)
    PR3 = (uint16_t)(period >> 16); // Upper 16 bits of 32-bit period (msw)
    PR2 = (uint16_t)period; // Lower 16 bits of 32-bit period (lsw)

    IFS0bits.T3IF = 0; // reset timer interrupt flag
    IPC2bits.T3IP = IP_TIMER32; // set interrupt priority level
    IEC0bits.T3IE = 1; // enable timer interrupt
    T2CONbits.TON = 0; // leave timer disabled initially

    // NOTE: calls _T3Interrupt(void) on interrupt
}

void setTimer1State(bool state)
{
    T1CONbits.TON = state; //
}

void setTimer2OState(bool state)
{
    T2CONbits.TON = state; //
}

void setTimer3State(bool state)
{
    T3CONbits.TON = state; //
}

void setTimer32CombinedState(bool state)
{
    T2CONbits.TON = state; //
}

/**
 * @brief Initializes a hardware timer with a specified period in milliseconds.
 *
 * This function calculates the appropriate timer period and prescaler for the given
 * timer number and desired time in milliseconds. It supports both 16-bit and 32-bit
 * timers and calls the corresponding initialization function.
 *
 * @param timeInMS The desired timer period in milliseconds. Must be within the
 *                 supported range for the selected timer and prescaler combination.
 * @param timer The timer to configure:
 *              - 1: Timer1 (16-bit)
 *              - 2: Timer2 (16-bit)
 *              - 3: Timer3 (16-bit)
 *              - 32: Timer2 and Timer3 combined (32-bit)
 *
 * @note For 32-bit timers, the Timer2 and Timer3 modules are combined.
 *       Ensure that Timer2 and Timer3 are not independently configured when using Timer23.
 */
int16_t initTimerInMs(uint32_t timeInMs, uint8_t timer)
{
    // Prescaler options
    const uint16_t prescaler_options[] = { 1, 8, 64, 256 };

    // Base frequency in Hz depending on the oscillator
    const uint32_t FCY = CLOCK_FCY;

    // Max count values for each type of timer
    const uint16_t max_count_16 = 0xffff; // 2^16 - 1 = 65_535
    const uint32_t max_count_32 = 0xffffffff; // 2^32 - 1 = 4_294_967_295

    // Max count based on timer type
    uint32_t max_count = (timer == 32) ? max_count_32 : max_count_16;

    // Timer parameters to calculate
    uint32_t count;
    uint16_t prescaler;

    uint8_t match_found = 0;

    // Loop through prescaler options
    for (uint16_t i = 0; i < 4; i++) {
        uint32_t adj_FCY = FCY / prescaler_options[i];

        // Calculate the number of cycles needed to achieve the given period
        // period is given in ms
        // Approach: convert frequency to kHz and multiply by the number of ms

        // for 16-bit timers 64bit is safe as timeInMs <= 630ms and adj_FCY <= 26_726_400 so log2(630*26_726_400) = 33.97 bits
        // for 32-bit timers 64bit is safe as timeInMs <= 41_232_000ms and adj_FCY <= 26_726_400 so log2(41_232_000*26_726_400) = 49.96 bits
        uint64_t required_ticks = ((uint64_t)timeInMs * adj_FCY) / 1000;

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
    case 1:
        initTimer1((uint16_t)count, prescaler);
        break;
    case 2:
        initTimer2((uint16_t)count, prescaler);
        break;
    case 3:
        initTimer3((uint16_t)count, prescaler);
        break;
    case 32:
        initTimer32Combined(count, prescaler);
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

void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0; // reset Timer 1 interrupt flag
    
    imuReadGyro();
    //imuReadAccel();
    //imuReadMag();
    //imuReadTemp();
}

/* ISR for Timer2 */
void __attribute__((__interrupt__, auto_psv)) _T2Interrupt(void)
{
    IFS0bits.T2IF = 0; // reset Timer 2 interrupt flag
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
    
    // Track the time passed within this timer.
    static uint32_t rtttlTimeCount = 0;
    
    // 1) First, increment the time counter.
    rtttlTimeCount++;
    
    // 2) Check if we've reached the current note's duration.
    if (rtttlTimeCount >= rtttlNotes.notes[rtttlNotes.noteIndex].duration)
    {
        // Advance to next note
        rtttlNotes.noteIndex++;
      
        if (rtttlNotes.noteIndex >= rtttlNotes.notesLen)
        {
          if (songRepeat) {
              rtttlNotes.noteIndex = 0;
          } else {
              stopSong();
          }
        }
      
        // Reset and load the next note
        rtttlTimeCount = 0;

        // Turn of PWM in case frequency is zero, enable otherwise
        if (songPlaying && rtttlNotes.notes[rtttlNotes.noteIndex].frequency > 0) {
            setPWMFrequency(BUZZ_PWM_MODULE, rtttlNotes.notes[rtttlNotes.noteIndex].frequency);
            setPWMState(BUZZ_PWM_MODULE, BUZZ_PWM_CHANNEL, true);
        } else {
            setPWMState(BUZZ_PWM_MODULE, BUZZ_PWM_CHANNEL, false);
        }
        
    }
}