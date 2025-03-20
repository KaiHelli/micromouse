#ifndef GLOBALTIMERS_H
#define	GLOBALTIMERS_H

#include <stdint.h>

void initGlobalTimekeeping();
void resetGlobalTimekeeping();
uint32_t getTimeInUs();

#endif	/* GLOBALTIMERS_H */

