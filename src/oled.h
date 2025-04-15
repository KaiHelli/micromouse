#ifndef OLED_H
#define	OLED_H
#include "mazeSolver.h"

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

#define CELL_WIDTH 5
#define X_OFFSET (OLED_WIDTH - OLED_HEIGHT)/2 + 1

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
 * @brief Refreshes a specific area of the OLED display.
 *
 * Updates the physical display to reflect the current state of the display buffer. 
 * The only area that is updated is the minimum necessary bytes that are needed to specify the area bound by (start_x, start_y) and (end_x, end_y).
 */
void oledPushArea(uint8_t start_x, uint8_t start_y, uint8_t end_x, uint8_t end_y);

/**
 * @brief Sets a pixel on the OLED display.
 *
 * Sets a pixel at the specified (x, y) coordinate to the specified state.
 * Returns if this operation changed anything;
 */
void oledSetPixel(uint8_t x, uint8_t y, bool state);

/**
 * @brief Draws the empty maze.
 *
 * Draws the boundaries of the maze and markers for the walls, as well as the mouse on its initial position.
 */
void oledDrawEmptyMaze(void);

/**
 * @brief Draws the mouse.
 *
 * Updates the display of the mouse by removing it from the previous cell and adding it at the center of its specified cell.
 */
void oledUpdateMouse(Mouse* mouse);

/**
 * @brief Draws the walls of a cell.
 *
 * Draws the walls of a cell in the maze at the specified row and column.
 */
void oledDrawCell(Cell cell,uint8_t row, uint8_t col);

#endif	/* OLED_H */

