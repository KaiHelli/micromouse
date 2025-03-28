#ifndef OLED_H
#define	OLED_H

// Source:
// https://github.com/datgeezus/oled/blob/master/oled.h
// https://github.com/datgeezus/oled/blob/master/oled.c

#define OLED_WIDTH        (64U)
#define OLED_HEIGHT       (32U)

#define OLED_BUFFSIZE       (OLED_WIDTH * OLED_HEIGHT / 8U)
#define OLED_NPAGES         (OLED_HEIGHT / 8U)

#define OLED_COL_OFFSET     32U

#define OLED_FIRST_PAGE     0U
#define OLED_LAST_PAGE      (OLED_NPAGES - 1U)

#define OLED_FIRST_COL      0U + OLED_COL_OFFSET
#define OLED_LAST_COL       (OLED_WIDTH - 1U) + OLED_COL_OFFSET

/**
 * @brief Initializes the OLED display.
 *
 * Configures and prepares the OLED display module for use.
 */
void oledSetup(void);

/**
 * @brief Clears the OLED display buffer.
 *
 * Resets the display memory to remove any current pixel data.
 */
void oledClearDisplay(uint8_t defaultValue);

void oledDrawFrame(void);

/**
 * @brief Refreshes the OLED display.
 *
 * Updates the physical display to reflect the current state of the display buffer.
 */
void oledRefresh(void);

/**
 * @brief Sets a pixel on the OLED display.
 *
 * Activates a pixel at the specified (x, y) coordinate.
 */
void oledSetPixel(uint8_t x, uint8_t y);

#endif	/* OLED_H */

