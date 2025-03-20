/* Microchip Technology Inc. and its subsidiaries.  You may use this software
 * and any derivatives exclusively with Microchip products.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
 * TERMS.
 */

/*
 * File:
 * Author: Alexander Lenz
 * Comments:
 * Revision history:
 */

// This is a guard condition so that contents of this file are not included
// more than once.
#ifndef __MYTIMERS_H__
#define __MYTIMERS_H__

#include <stdbool.h>
#include <stdint.h>
#include <xc.h> // include processor files - each processor file is guarded.

#define NUM_TIMERS 3
#define CALLBACK_BUFFER_SIZE 5

typedef enum {
    TIMER_1 = 0,
    TIMER_2,
    TIMER_3,
    TIMER_32_COMBINED
} Timer_t;

// Timer callback function
// When returning 0, the callback will be unregistered
// When returning 1, the callback will remain registered
typedef int16_t (*TimerCallback_t)(void);

void initTimer1(uint16_t period, uint16_t prescaler);
void initTimer2(uint16_t period, uint16_t prescaler);
void initTimer3(uint16_t period, uint16_t prescaler);
void initTimer32Combined(uint32_t period, uint16_t prescaler);

void setTimerInterruptState(Timer_t timer, bool state);
void setTimerState(Timer_t timer, bool state);
int16_t initTimerInMs(Timer_t timer, uint32_t timeInMs);
int16_t registerTimerCallback(Timer_t timer, TimerCallback_t callback);
void clearTimerCallbacks(Timer_t timer);

#endif /* XC_HEADER_TEMPLATE_H */
