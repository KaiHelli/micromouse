#ifndef TIMERS_H
#define TIMERS_H

#include <stdbool.h>
#include <stdint.h>
#include <xc.h> // include processor files - each processor file is guarded.

#define NUM_TIMERS 5
#define TIMER_CALLBACK_BUFFER_SIZE 5

typedef enum {
    TIMER_1 = 0,
    TIMER_2,
    TIMER_3,
    TIMER_4,
    TIMER_5,
    TIMER_32_COMBINED,
    TIMER_54_COMBINED
} Timer_t;

/**
 * @brief Timer callback function pointer type. Returns 0 to unregister
 * the callback, 1 to keep it registered.
 */
typedef int16_t (*TimerCallback_t)(void);

/**
 * @brief Initializes Timer1 with the specified period and prescaler.
 * Takes a 16-bit period value and a 16-bit prescaler.
 */
void initTimer1(uint16_t period, uint16_t prescaler);

/**
 * @brief Initializes Timer2 with the specified period and prescaler.
 * Takes a 16-bit period value and a 16-bit prescaler.
 */
void initTimer2(uint16_t period, uint16_t prescaler);

/**
 * @brief Initializes Timer3 with the specified period and prescaler.
 * Takes a 16-bit period value and a 16-bit prescaler.
 */
void initTimer3(uint16_t period, uint16_t prescaler);


/**
 * @brief Initializes Timer4 with the specified period and prescaler.
 * Takes a 16-bit period value and a 16-bit prescaler.
 */
void initTimer4(uint16_t period, uint16_t prescaler);


/**
 * @brief Initializes Timer5 with the specified period and prescaler.
 * Takes a 16-bit period value and a 16-bit prescaler.
 */
void initTimer5(uint16_t period, uint16_t prescaler);

/**
 * @brief Initializes combined Timer32 with the specified period and prescaler.
 * Takes a 32-bit period value and a 16-bit prescaler.
 */
void initTimer32Combined(uint32_t period, uint16_t prescaler);


/**
 * @brief Initializes combined Timer54 with the specified period and prescaler.
 * Takes a 32-bit period value and a 16-bit prescaler.
 */
void initTimer54Combined(uint32_t period, uint16_t prescaler);

/**
 * @brief Enables or disables the interrupt for the specified timer.
 * Takes a Timer_t and a boolean state (true = enable, false = disable).
 */
void setTimerInterruptState(Timer_t timer, bool state);

/**
 * @brief Starts or stops the specified timer.
 * Takes a Timer_t and a boolean state (true = start, false = stop).
 */
void setTimerState(Timer_t timer, bool state);

/**
 * @brief Initializes the specified timer to run for a duration in milliseconds.
 * Takes a Timer_t and a 32-bit time in milliseconds. 
 * 
 * @return A positive value on success, negative on failure.
 */
int16_t initTimerInMs(Timer_t timer, uint32_t timeInMs);

/**
 * @brief Initializes the specified timer to run for a duration in microseconds.
 * Takes a Timer_t and a 32-bit time in microseconds. 
 * 
 * @return A positive value on success, negative on failure.
 */
int16_t initTimerInUs(Timer_t timer, uint64_t timeInUs);

/**
 * @brief Registers a callback function for the specified timer.
 * Takes a Timer_t and a TimerCallback_t. 
 * 
 * @return A positive value on success, negative on failure.
 */
int16_t registerTimerCallback(Timer_t timer, TimerCallback_t callback);

/**
 * @brief Removes a specific timer callback function.
 *
 * Takes a pointer to the callback function to remove.
 * 
 * @return A positive value on success, negative on failure.
 */
int16_t removeTimerCallback(Timer_t timer, TimerCallback_t callback);

/**
 * @brief Clears all registered callbacks for the specified timer.
 * Takes a Timer_t to identify which callbacks to clear.
 */
void clearTimerCallbacks(Timer_t timer);

#endif /* TIMERS_H */
