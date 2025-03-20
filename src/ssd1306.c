#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "ssd1306.h"
#include "i2c.h"
#include "uart.h"

void oledSetup(void) {
    oledSetDisplayState(0);
    
    
    oledSetDisplayState(1);
    
}

void oledSetDisplayStateCb(bool success) {
    if (success)
    {
        putsUART1("Successful write!\r\n");
    }
    else
    {
        // Handle I2C error (e.g., no sensor found, bus conflict, etc.)
        putsUART1("Asynchronous OLED error!\r\n");
    }
}

void oledSetDisplayState(bool power) {
    //static uint8_t oledStates[2] = {OLED_DISPLAY_OFF, OLED_DISPLAY_ON};
    
    //status |= putsI2C1(I2C_OLED_ADDR, &oledStates[power], 1, NULL, 0, NULL);
    
    static uint8_t test[] = {
        0xAE,
        0xD5,
        0x80,
        0xA8,
        63,
        0xD3,
        0x00,
        0x40,
        0x8D,
        0x14,
        0xA1,
        0xC8,
        0xDA,
        0x12,
        0x81,
        0xFF,
        0xD9,
        0x22,
        0xDB,
        0x40,
        0xA4,
        0xA6,
        0xAF,
    };
    
    putsI2C1(I2C_OLED_ADDR, test, 23, NULL, 0, oledSetDisplayStateCb);
}
