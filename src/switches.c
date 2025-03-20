#include "IOconfig.h"
#include <xc.h>

void __attribute__((__interrupt__, auto_psv)) _INT1Interrupt(void) {
    IFS1bits.INT1IF = 0;
    
    LED1 = ~LED1;
}