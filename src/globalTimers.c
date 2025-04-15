#include <stdint.h>
#include <xc.h>

#include "globalTimers.h"
#include "timers.h"

/**
 * @brief Interrupt service routine for global timer. Called periodically to
 * increment the overflow counter.
 */
static int16_t globalTimerISR(void);

static volatile uint16_t globalTimerOverflows;

void initGlobalTimekeeping(void) {
    // Initialize combined 32-bit timer (Timer 4 and 5)
    // Use 4.25B as period, to have a good value to calculate with.
    // Use prescaling of 8 (option: 0b01), therefore F_CY = 40 MHz -> /8 -> 5 MHz.
    // Thus, the counter is incremented in 200ns steps. 
    initTimer54Combined(4250000000, 1);
    
    // Attach an ISR that counts each counter overflow. 
    // Overflow occurs every: 200 * 10^{-9} * 4.25 * 10^9 = 850s
    registerTimerCallback(TIMER_54_COMBINED, globalTimerISR);
    
    // Initialize overflow counter
    globalTimerOverflows = 0;
    
    // Enable timer
    setTimerState(TIMER_54_COMBINED, true);
}

void resetGlobalTimekeeping(void) {
    // Disable timer
    setTimerState(TIMER_54_COMBINED, false);

    // Reset counter
    TMR5HLD = 0; // clear upper bits of 32bit timer (msw)
    TMR4 = 0;    // clear lower bits of 32bit timer (lsw)
    
    // Reset overflow counter
    globalTimerOverflows = 0;
    
    // Re-enable timer
    setTimerState(TIMER_54_COMBINED, true);
}

uint64_t getTimeInUs(void) {
    
    uint64_t currentTime = TMR4;    // set lower bits of 32bit timer (lsw) - triggers internal copy of TMR5 to TMR5HLD
    currentTime |= (uint32_t) TMR5HLD << 16;   // set upper bits of 32bit timer (msw)
    
    // Scale by division by 5. Each tick is 200ns thus five ticks are 1us.
    currentTime /= 5;
    
    // Add time accounted for each timer overflow, occuring every 850s.
    currentTime += globalTimerOverflows * 850000000;
    
    return currentTime;
}

static int16_t globalTimerISR(void) {
    // Increments every 850s -> 850s * 2^16 = ~644 days before an overflow occurs
    // Thus considered to be safe for our purpose.
    globalTimerOverflows++;
    
    return 1;
}