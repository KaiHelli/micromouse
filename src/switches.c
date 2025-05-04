#include "switches.h"
#include "IOconfig.h"
#include "interrupts.h"
#include "atomic.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

static bool skipStartupCallback = true;

void initSwitch1(void) {
    INTCON2bits.INT1EP = 0;     // interrupt on positive edge for External Interrupt 1
    IEC1bits.INT1IE = 0;        // leave external interrupt 1 disabled for now
    IPC5bits.INT1IP = IP_INT1;  // set low priority for External Interrupt 1
}

void setSwitchInterruptState(Switch_t sw, bool state) {
    switch(sw) {
        case SWITCH_1:
            IEC1bits.INT1IE = state;
            break;
    }
}

typedef enum {
    CB_EVEN   = 0,   /* run when the current ISR cycle is even            */
    CB_ODD    = 1,   /* run when the current ISR cycle is odd             */
    CB_ALWAYS = 2    /* already executed once -> run on every cycle        */
} CbPhase_t;

typedef struct {
    SwitchCallback_t cb;
    CbPhase_t        phase;
} SwitchCbEntry_t;

static volatile SwitchCbEntry_t switchCallbacks[NUM_SWITCHES][SWITCH_CALLBACK_BUFFER_SIZE];

static volatile uint8_t registeredSwitchCallbacks[NUM_SWITCHES];
static volatile uint8_t currentPhase[NUM_SWITCHES];


int16_t registerSwitchCallback(Switch_t sw, SwitchCallback_t callback) {
    // Check if there buffer of interrupt callbacks is already full.
    if (registeredSwitchCallbacks[sw] == SWITCH_CALLBACK_BUFFER_SIZE) {
        return -1;
    }
    
    // Temporarily disable switch interrupts during modifying callbacks.
    setSwitchInterruptState(sw, false);

    uint8_t idx = registeredSwitchCallbacks[sw]++;
    switchCallbacks[sw][idx].cb  = callback;
    
    // run in the *next* generation => currentGeneration+1           
    switchCallbacks[sw][idx].phase = (currentPhase[sw] == 0) ? CB_ODD : CB_EVEN;
           
    // Re-enable switch interrupts.
    setSwitchInterruptState(sw, true);
    
    return 0;
    
}

int16_t removeSwitchCallback(Switch_t sw, SwitchCallback_t callback) {
    // Temporarily disable switch interrupts during modifying callbacks.
    setSwitchInterruptState(sw, false);
    
    bool recoverInterruptState = true;
    int16_t status = -1;
    
    for (uint8_t i = 0; i < registeredSwitchCallbacks[sw]; i++) {
        if (switchCallbacks[sw][i].cb != callback) {
            continue;
        }
        
        registeredSwitchCallbacks[sw]--;

        if (registeredSwitchCallbacks[sw] == 0) {
            // Disable the interrupts itself, if there are no callbacks left.
            recoverInterruptState = false;
        }

        // Only swap if we're not already at the last position
        if (i < registeredSwitchCallbacks[sw]) {
            switchCallbacks[sw][i] = switchCallbacks[sw][registeredSwitchCallbacks[sw]];
        }

        status = 0;
        break;
    }
    
    // Re-enable switch interrupts.
    setSwitchInterruptState(sw, recoverInterruptState);
    
    return status;
}

void clearSwitchCallbacks(Switch_t sw) {
    // We clear the callbacks, so the switch can be disabled.
    setSwitchInterruptState(sw, false);
    registeredSwitchCallbacks[sw] = 0;
}

static void generalSwitchISR(Switch_t sw) {
    if (skipStartupCallback) {
        skipStartupCallback = false;
        return;
    }
    
    uint8_t thisPhase = currentPhase[sw] ^= 1;     // 0 <-> 1
    
    uint8_t i = 0;
    while (i < registeredSwitchCallbacks[sw]) {

        CbPhase_t phase = switchCallbacks[sw][i].phase;

        // Skip if not yet eligible
        if (phase != CB_ALWAYS && phase != (CbPhase_t)thisPhase) {
            i++;
            continue;
        }
        
        SwitchCallback_t callback = switchCallbacks[sw][i].cb;
        uint8_t status = callback();
        
        // First execution completed? -> mark as ?always? 
        if (phase != CB_ALWAYS)
            switchCallbacks[sw][i].phase = CB_ALWAYS;

        if (status == 0) {
            // Guard from registerSwitchCallback being called from an interrupt with
            // higher priority while reading/writing from/to the callback buffers below.
            uint16_t state = _atomic_enter();
            
            registeredSwitchCallbacks[sw]--;
            
            if (registeredSwitchCallbacks[sw] == 0) {
                // Disable the interrupts itself, if there are no callbacks left.
                setSwitchInterruptState(sw, false);
            }

            // Only swap if we're not already at the last position
            if (i < registeredSwitchCallbacks[sw]) {
                switchCallbacks[sw][i] = switchCallbacks[sw][registeredSwitchCallbacks[sw]];
                // Do not increment i, as we've moved a new callback into position i
            }
            // If removed callback was already last, just continue without incrementing i
            
            _atomic_leave(state);
        } else {
            i++;  // move to next callback only if no removal
        }
    }
            
}

void __attribute__((__interrupt__, auto_psv)) _INT1Interrupt(void) {
    IFS1bits.INT1IF = 0;
    
    generalSwitchISR(SWITCH_1);
}