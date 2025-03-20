#ifndef DMA_H
#define DMA_H

#include <xc.h>
#include <stdint.h>

/**
 * @brief Buffer used for storing ADC data.
 *
 * This 32-element array is stored in DMA space and is used to hold
 * ADC conversion results.
 */
extern uint16_t adcData[32] __attribute__((space(dma)));

/**
 * @brief Initializes DMA Channel 4 for ADC data transfer.
 *
 * Configures and sets up DMA Channel 4, preparing it to handle ADC
 * data transfers automatically.
 */
void initDmaChannel4(void);

// Add some defines to make accessing data more readable
#define DIST_SENS_3 adcData[0]  // AN4
#define DIST_SENS_1 adcData[1]  // AN5
#define DIST_SENS_2 adcData[2]  // AN6

#endif /* DMA_H */
