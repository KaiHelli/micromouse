#ifndef CLOCK_H
#define	CLOCK_H

void setupClock();

#define FOSC 80000000UL // 80 MHz Clock Signal
#define FCY 40000000UL  // 40 MIPS
#define CLOCK_TCY_NSEC 25        // 25ns per Cycle

#endif	/* CLOCK_H */

