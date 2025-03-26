#include <xc.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>

#include "ssd1306.h"
#include "oled.h"
#include "uart.h"
#include "i2c.h"

static uint8_t displayBuffer[OLED_BUFFSIZE] = {0};


void oledSetup(void) {
    ssd1306Setup();
}

static void oledCommCb(bool success) {
    if (!success)
    {
        // Handle I2C error (e.g., no sensor found, bus conflict, etc.)
        putsUART1("Asynchronous OLED error!\r\n");
    }
}

void oledClearDisplay(void) {
    memset(displayBuffer, 0, OLED_BUFFSIZE);
}

void oledRefresh(void)
{
    ssd1306SetColumnAddress(0, OLED_LAST_COL);
    ssd1306SetPageAddress(0, OLED_LAST_PAGE);
    
    putsI2C1(I2C_OLED_ADDR, displayBuffer, OLED_BUFFSIZE, NULL, 0, oledCommCb);
}


void oledSetPixel(uint8_t x, uint8_t y) {
    // Check for a valid position
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
        return;
    }

    //displayBuffer[x + ((y / 8) * OLED_WIDTH)] = (1 << (y % 8));
    displayBuffer[x + ((y >> 3) * OLED_WIDTH)] |= (1 << (y & 7));
}