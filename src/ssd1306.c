#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include "ssd1306.h"
#include "i2c.h"
#include "uart.h"

void ssd1306Setup(void) {
    bool status = 1; // track the status of all operations
    
    static uint8_t oledSetupData[] = {
        0x00,                                   // All bytes following are command bytes
        SSD1306_DISPLAY_ON_OFF | DISPLAY_OFF,   // Turn display off before doing changes
        SSD1306_SET_DISPLAY_CLOCK_DIV,          // Set display clock division
        0x80,                                   // 0b1000 oscillator / 0b0000 division
        SSD1306_SET_MULTIPLEX,                  // Set multiplex ratio
        0x1f,                                     // 32-row display | changed from default 63
        SSD1306_SET_DISPLAY_OFFSET,             // Set display offset
        0,                                      // 0 offset - display starts from very top
        SSD1306_SET_START_LINE | 0,             // Set display start line to 0
        SSD1306_CHARGE_PUMP,                    // Set charge pump
        CHARGE_ENABLE,                          // Enable charge pump
        SSD1306_SET_SEGMENT_REMAP | ADDR_127,   // Set segment remap to 127
        SSD1306_COM_SCAN | COM_REMAPPED,        // Set COM vertical scanning
        SSD1306_SET_COM_PINS,                   // Set COM hardware config
        COM_SEQUENTIAL_ENABLE_COM_REMAP,        // sequential COM remap
        SSD1306_SET_CONTRAST,                   // Set Contrast
        0xCF,                                   // Mid contrast | switched from default 0xFF
        SSD1306_SET_PRECHARGE,                  // Set pre-charge period
        0x22,                                   // Set pre-charge period param
        SSD1306_SET_VCOM_DESELECT,              // Set deselect Vcomh level
        LEVEL_0_83V,                            // Vcomh level param | switched from default 0x40
        SSD1306_DEACTIVATE_SCROLL,              // Disable scrolling
        SSD1306_SET_MEMORY_MODE,                // Set memory adressing mode
        MODE_HORIZONTAL,                        // Set mode to be horizontal adressing
        SSD1306_ENTIRE_DISPLAYALL_ON | RESUME_TO_RAM,   // Set entire display on, content from RAM
        SSD1306_NORM_INV_DISPLAY | DISPLAY_NORMAL,      // Set normal display
        // SSD1306_DISPLAY_ON_OFF | DISPLAY_ON,    // Turn display on
    };
    
    status &= putsI2C1Sync(I2C_OLED_ADDR, oledSetupData, sizeof(oledSetupData), NULL, 0);
    
    // Wait for settings to take effect.
    __delay_ms(500);
    
    if (status) {
        putsUART1("Display configured.\r\n");
    } else {
        putsUART1("Error setting up the display!\r\n");
    }
}

static void ssd1306CommCb(bool success) {
    if (!success)
    {
        putsUART1("Asynchronous SSD1306 error!\r\n");
    }
}

void ssd1306SetColumnAddress(uint8_t startAddress, uint8_t endAddress) {
    static uint8_t i2cData[] = {0x00, SSD1306_SET_COLUMN_ADDR, 0x0, 0x0};
    i2cData[2] = startAddress & 0x7F;
    i2cData[3] = endAddress & 0x7F;
    
    putsI2C1(I2C_OLED_ADDR, i2cData, 4, NULL, 0, ssd1306CommCb);
}

void ssd1306SetPageAddress(uint8_t startAddress, uint8_t endAddress)
{
    static uint8_t i2cData[] = {0x00, SSD1306_SET_PAGE_ADDR, 0x0, 0x0};
    i2cData[2] = startAddress & 0x07;
    i2cData[3] = endAddress & 0x07;
    
    putsI2C1(I2C_OLED_ADDR, i2cData, 4, NULL, 0, ssd1306CommCb);
}


void ssd1306SetDisplayState(uint8_t state) {
    static uint8_t i2cData[] = {0x00, 0x0};
    i2cData[1] = SSD1306_DISPLAY_ON_OFF | (state & 0x01);
    
    putsI2C1(I2C_OLED_ADDR, i2cData, 4, NULL, 0, ssd1306CommCb);
}
