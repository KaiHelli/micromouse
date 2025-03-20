#ifndef ADC_H
#define	ADC_H

/**
 * @brief Configures the 12-bit Analog-to-Digital Converter (ADC1).
 * 
 * Initializes ADC1 peripheral with predefined settings.
 */
void setupADC1(void);

/**
 * @brief Starts the ADC1 conversion process.
 * 
 * Begins sampling and conversion based on the current ADC1 configuration.
 */
void startADC1(void);

#endif	/* ADC_H */
