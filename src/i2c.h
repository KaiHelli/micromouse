#ifndef I2C_H
#define	I2C_H

#include <stdint.h>
#include <stdbool.h>

void setupI2C1(void);
bool getI2C1Status(void);
bool putsI2C1(uint8_t devAddr, const uint8_t *wData, uint8_t wLen, uint8_t *rData, uint8_t rLen, void (*cb)(bool));
bool putsI2C1Sync(uint8_t devAddr, const uint8_t *wData, uint8_t wLen, uint8_t *rData, uint8_t rLen);

#define I2C_IDLE 0 // I2C is idle, ready for a new transmission
#define I2C_BUSY 1 // I2C is busy transmitting

#endif	/* I2C_H */

