#ifndef SSD1306_H
#define	SSD1306_H

#include <stdint.h>

#define I2C_OLED_ADDR   0x3C

// Source:
// https://github.com/datgeezus/oled/blob/master/SSD1306.h
// https://github.com/datgeezus/oled/blob/master/SSD1306.c

/* Fundamental ****************************************************************/
// Entire Display ON
#define RESUME_TO_RAM       0
#define ENTIRE_DISPLAY_ON   1
// Set normal/inverse display
#define DISPLAY_NORMAL      0
#define DISPLAY_INVERSE     1
// Set Display ON/OFF
#define DISPLAY_OFF         0
#define DISPLAY_ON          1

/* Scrolling ******************************************************************/
// Continuous horizontal scrolling
#define SCROLL_RIGHT        0
#define SCROLL_LEFT         1
// Continuous vertical and horizontal scrolling
#define SCROLL_DIAG_RIGHT   0x01
#define SCROLL_DIAG_LEFT    0x02

/* Addressing *****************************************************************/
// Set memory addressing mode
#define MODE_HORIZONTAL     0x00
#define MODE_VERTICAL       0x01
#define MODE_PAGE           0x02

/* Hardware configuration *****************************************************/
// Set segment remap
#define ADDR_0              0
#define ADDR_127            1
// Set COM output scan direction
#define COM_NORMAL          0x00
#define COM_REMAPPED        0x08
// Set COM pins hardware configuration
#define COM_SEQUENTIAL_DISABLE_COM_REMAP    0x02
#define COM_SEQUENTIAL_ENABLE_COM_REMAP     0x12
#define COM_ALTERNATIVE_DISABLE_COM_REMAP   0x22
#define COM_ALTERNATIVE_ENABLE_COM_REMAP    0x32
// Set V_comh Deselect
#define LEVEL_0_65V         0x00
#define LEVEL_0_77V         0x20
#define LEVEL_0_83V         0x30

/* Charge Pump ****************************************************************/
#define CHARGE_DISABLE      0x10
#define CHARGE_ENABLE       0x14


/* Command table, page 28 of SSD1306 datasheet ********************************/
// Fundamental
#define SSD1306_SET_CONTRAST            0x81
#define SSD1306_ENTIRE_DISPLAYALL_ON    0xA4
#define SSD1306_NORM_INV_DISPLAY        0xA6
#define SSD1306_DISPLAY_ON_OFF          0xAE

// Scrolling
#define SSD1306_HORIZONTAL_SCROLL           0x26
#define SSD1306_VERTICAL_HORIZONTAL_SCROLL  0x29
#define SSD1306_DEACTIVATE_SCROLL           0x2E
#define SSD1306_ACTIVATE_SCROLL             0x2F
#define SSD1306_SET_VERTICAL_SCROLL_AREA    0xA3

// Address setting
#define SSD1306_SET_LOW_COLUMN          0x00
#define SSD1306_SET_HIGH_COLUMN         0x10
#define SSD1306_SET_MEMORY_MODE         0x20
#define SSD1306_SET_COLUMN_ADDR         0x21
#define SSD1306_SET_PAGE_ADDR           0x22
#define SSD1306_SET_PAGE_START_ADDR     0xB0

// Hardware configuration (Panel resolution and layout
#define SSD1306_SET_START_LINE          0x40
#define SSD1306_SET_SEGMENT_REMAP       0xA0
#define SSD1306_SET_MULTIPLEX           0xA8
#define SSD1306_COM_SCAN                0xC0
#define SSD1306_SET_DISPLAY_OFFSET      0xD3
#define SSD1306_SET_COM_PINS            0xDA

// Timing and driving scheme
#define SSD1306_SET_DISPLAY_CLOCK_DIV   0xD5
#define SSD1306_SET_PRECHARGE           0xD9
#define SSD1306_SET_VCOM_DESELECT       0xDB
#define SSD1306_NOP                     0xE3

// Charge Pump: page 62
#define SSD1306_CHARGE_PUMP             0x8D

/**
 * @brief Initializes the SSD1306 OLED module. Configures the necessary
 *        settings for display operation.
 */
void ssd1306Setup(void);

/**
 * @brief Sets the column address range for subsequent data writes.
 *
 * Defines the horizontal range (columns) to be used for displaying data.
 */
void ssd1306SetColumnAddress(uint8_t startAddress, uint8_t endAddress);

/**
 * @brief Sets the page address range for subsequent data writes.
 *
 * Defines the vertical range (pages) to be used for displaying data.
 */
void ssd1306SetPageAddress(uint8_t startAddress, uint8_t endAddress);

#endif	/* SSD1306_H */
