#ifndef GLOBALTIMERS_H
#define GLOBALTIMERS_H

#include <stdint.h>

/**
 * @brief Initializes the global timekeeping mechanism.
 * 
 * Sets up combined timers 4 and 5 to track elapsed time since device startup.
 */
void initGlobalTimekeeping(void);

/**
 * @brief Resets the global timekeeping counter.
 * 
 * Clears the elapsed time counter to start measuring from zero again.
 */
void resetGlobalTimekeeping(void);

/**
 * @brief Retrieves the elapsed time since startup, in microseconds.
 * 
 * @return The number of microseconds that have elapsed since the device was started.
 */
uint64_t getTimeInUs(void);

#endif /* GLOBALTIMERS_H */
