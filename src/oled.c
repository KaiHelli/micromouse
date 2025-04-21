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
    oledDrawEmptyMaze();
    oledRefresh();
    ssd1306SetDisplayState(1);
}

static void oledCommCb(bool success) {
    if (!success)
    {
        putsUART1Str("Asynchronous OLED error!\r\n");
    }
}

void oledClearDisplay(uint8_t defaultValue) {
    memset(displayBuffer, defaultValue, OLED_BUFFSIZE + 1);
    
    // Control byte - sets I2C transmission to consist of data bytes.
    displayBuffer[0] = 0x40;
}

void oledDrawFrame(void) {
    for (uint8_t i = 0; i < OLED_WIDTH; i++) {
        oledSetPixel(i, 0, 1);
        oledSetPixel(i, OLED_HEIGHT - 1, 1);
    }
    
    for (uint8_t i = 1; i < OLED_HEIGHT - 1; i++) {
        oledSetPixel(0, i, 1);
        oledSetPixel(OLED_WIDTH - 1, i, 1);
    }
}

void oledRefresh(void)
{
    // Writing the display buffer to the screen takes about 5.805ms to complete
    // Calculation:
    // (4 pages * 64 columns + 1 control byte + 1 i2c address byte) * 9 bits / 400kHz
    ssd1306SetColumnAddress(OLED_FIRST_COL, OLED_LAST_COL);
    ssd1306SetPageAddress(OLED_FIRST_PAGE, OLED_LAST_PAGE);

    putsI2C1(I2C_OLED_ADDR, displayBuffer, sizeof(displayBuffer), NULL, 0, oledCommCb);
}


void oledSetPixel(uint8_t x, uint8_t y, bool state) {
    // Check for a valid position
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
        return;
    }
    uint8_t prev_state = displayBuffer[x + ((y >> 3) * OLED_WIDTH) + 1];
    // NOTE: +1 due to the first byte in the buffer being reserved for the
    // control byte
    //displayBuffer[x + ((y / 8) * OLED_WIDTH)] = (1 << (y % 8));
    if(state)
        displayBuffer[x + ((y >> 3) * OLED_WIDTH) + 1] |= (1 << (y & 7));
    else
        displayBuffer[x + ((y >> 3) * OLED_WIDTH) + 1] &= ~(1 << (y & 7));
    
}

void oledPushArea(uint8_t start_x, uint8_t start_y, uint8_t end_x, uint8_t end_y){
    static uint8_t db[OLED_BUFFSIZE + 1]  = {0};
    db[0] = 0x40;
    uint8_t start_y_byte = start_y >> 3;
    uint8_t end_y_byte = end_y >> 3;
    uint8_t num_bytes = (end_x - start_x + 1)*(end_y_byte  - start_y_byte + 1);
    
    ssd1306SetColumnAddress(OLED_FIRST_COL  + start_x, OLED_FIRST_COL + end_x);
    ssd1306SetPageAddress(OLED_FIRST_PAGE + start_y_byte, OLED_FIRST_PAGE + end_y_byte);
    uint8_t i = 1;
    for(uint8_t y = start_y_byte; y<=end_y_byte; y++){
        for(uint8_t x = start_x; x<=end_x; x++){
            db[i] = displayBuffer[x + (y * OLED_WIDTH) + 1];
            i++;
        }
    }
      putsI2C1(I2C_OLED_ADDR, db, num_bytes +1, NULL, 0, oledCommCb);
}

void oledDrawMouse(uint8_t row, uint8_t col, bool state){
    //Draw 2x2 Block at center of the cell
    oledSetPixel(CELL_WIDTH*col + X_OFFSET + 3, CELL_WIDTH*row + 3, state);
    oledSetPixel(CELL_WIDTH*col +X_OFFSET +4,CELL_WIDTH*row + 4, state);
    oledSetPixel(CELL_WIDTH*col +X_OFFSET + 4,CELL_WIDTH*row + 3, state);
    oledSetPixel(CELL_WIDTH*col +X_OFFSET + 3,CELL_WIDTH*row + 4, state);
   
}

void oledDrawEmptyMaze(void) {
    for (uint8_t i = 1; i < OLED_HEIGHT; i++) {
        oledSetPixel(X_OFFSET + i, 1, 1);
        oledSetPixel(X_OFFSET + i, OLED_HEIGHT - 1, 1);
    }
    
    for (uint8_t i = 1; i < OLED_HEIGHT - 1; i++) {
        oledSetPixel(X_OFFSET+1, i, 1);
        oledSetPixel(X_OFFSET + OLED_HEIGHT-1, i, 1);
    }
    
    //Draw wall markings
    for (uint8_t i = 1; i < N; i++) {
        for (uint8_t j = 1; j < N; j++) {
            oledSetPixel(X_OFFSET + CELL_WIDTH*i + 1, CELL_WIDTH*j + 1, 1);
        }
    }
    
    //Draw reference pixel on the left of bottom left corner
    oledSetPixel(X_OFFSET, CELL_WIDTH*N + 1, 1);
    
    //Draw mouse at initial position (Bottom left corner)
    oledDrawMouse(5,0,1);
}

void oledPushMouse(uint8_t row, uint8_t col){
    oledPushArea(CELL_WIDTH*col + X_OFFSET + 3, CELL_WIDTH*row + 3,CELL_WIDTH*col +X_OFFSET + 4,CELL_WIDTH*row + 4);
}

void oledUpdateMouse(Mouse* mouse){
    static uint8_t prev_row = 5, prev_col = 0;
    
    if(prev_row == mouse->row && prev_col == mouse->col){
        //mouse didn't change position
        return;
    }
    
    oledDrawMouse(prev_row, prev_col, 0);
    oledPushMouse(prev_row, prev_col);
    //short delay to allow the deletion to complete
    __delay_ms(1);        
    prev_row = mouse->row;
    prev_col =  mouse->col;
    
    oledDrawMouse(mouse->row, mouse->col, 1);
    oledPushMouse(prev_row, prev_col);
}

void oledDrawCell(Cell* cell,uint8_t row, uint8_t col) {
    for (uint8_t i = 1; i < N; i++){
        if(cell->wallBottom){
            oledSetPixel(CELL_WIDTH * col + X_OFFSET + i, CELL_WIDTH * (row + 1) + 1, 1);
        }
        
        if(cell->wallTop){
            oledSetPixel(CELL_WIDTH * col + X_OFFSET + i, CELL_WIDTH * row + 1, 1);
        }
        
        if(cell->wallLeft){
            oledSetPixel(CELL_WIDTH * col + X_OFFSET + 1, CELL_WIDTH * row + i, 1);
        }
        
        if(cell->wallRight){
            oledSetPixel(CELL_WIDTH * (col+1) + X_OFFSET + 1, CELL_WIDTH * row + i, 1);
        }   
    }
    oledPushArea(CELL_WIDTH*col + X_OFFSET + 1, CELL_WIDTH*row + 1,CELL_WIDTH*(col+1) +X_OFFSET +1,CELL_WIDTH*(row+1) + 1);
}