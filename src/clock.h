#ifndef CLOCK_H
#define CLOCK_H

/**
 * @brief Configures the device clock.
 * 
 * Initializes and sets up the system to use the 80 MHz clock signal.
 */
void setupClock();

#define FOSC 80000000UL   // 80 MHz Clock Signal
#define FCY 40000000UL    // 40 MIPS
#define CLOCK_TCY_NSEC 25 // 25 ns per cycle

#endif /* CLOCK_H */
