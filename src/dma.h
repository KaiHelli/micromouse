#ifndef DMA_H
#define	DMA_H

#include <xc.h>
    
extern unsigned int adcData[32]__attribute__((space(dma)));

void initDmaChannel4(void);

//add some defines to make accessing data more readable

#define DIST_SENS_3 adcData[0]  //AN4
#define DIST_SENS_1 adcData[1]  //AN5
#define DIST_SENS_2 adcData[2]  //AN6

#endif	/* DMA_H */

