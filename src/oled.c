#include <xc.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include "ssd1306.h"
#include "oled.h"
#include "uart.h"
#include "i2c.h"

static uint8_t displayBuffer[OLED_BUFFSIZE + 1] = {0};


void oledSetup(void) {
    ssd1306Setup();
    oledClearDisplay(0x00);
    oledDrawFrame();
    oledRefresh();
    ssd1306SetDisplayState(1);
}

static void oledCommCb(bool success) {
    if (!success)
    {
        putsUART1("Asynchronous OLED error!\r\n");
    }
}

void oledClearDisplay(uint8_t defaultValue) {
    memset(displayBuffer, defaultValue, OLED_BUFFSIZE + 1);
    
    // Control byte - sets I2C transmission to consist of data bytes.
    displayBuffer[0] = 0x40;
}

void oledDrawFrame(void) {
    for (uint8_t i = 0; i < OLED_WIDTH; i++) {
        oledSetPixel(i, 0);
        oledSetPixel(i, OLED_HEIGHT - 1);
    }
    
    for (uint8_t i = 0; i < OLED_HEIGHT; i++) {
        oledSetPixel(0, i);
        oledSetPixel(OLED_WIDTH - 1, i);
    }
}

void oledRefresh(void)
{
    ssd1306SetColumnAddress(OLED_FIRST_COL, OLED_LAST_COL);
    ssd1306SetPageAddress(OLED_FIRST_PAGE, OLED_LAST_PAGE);

    putsI2C1(I2C_OLED_ADDR, displayBuffer, sizeof(displayBuffer), NULL, 0, oledCommCb);
}


void oledSetPixel(uint8_t x, uint8_t y) {
    // Check for a valid position
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
        return;
    }

    // NOTE: +1 due to the first byte in the buffer being reserved for the
    // control byte
    //displayBuffer[x + ((y / 8) * OLED_WIDTH)] = (1 << (y % 8));
    displayBuffer[x + ((y >> 3) * OLED_WIDTH) + 1] |= (1 << (y & 7));
}