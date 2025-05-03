/* atomic.h  -------------------------------------------------------------- */
#ifndef ATOMIC_H
#define ATOMIC_H

#include <xc.h>
#include <stdint.h>

/* -----------------------------------------------------------------------
 *  Save current SR, disable all maskable interrupts, return the old SR.
 * ---------------------------------------------------------------------*/
static inline uint16_t _atomic_enter(void)
{
    // Save current interrupt enable state
    uint16_t state = __builtin_get_isr_state();

    // Disable interrupts
    __builtin_disable_interrupts();
    
    return state;
}

/* -----------------------------------------------------------------------
 *  Restore the exact SR value that was active before _atomic_enter().
 * ---------------------------------------------------------------------*/
static inline void _atomic_leave(uint16_t state)
{
    // Restore interrupt enable state
    __builtin_set_isr_state(state);
}

/* ------------- 32?bit float atomic helpers ---------------------------- */
static inline float atomic_read_f32(volatile float *addr)
{
    uint16_t state = _atomic_enter();
    float    v  = *addr;          /* now immune to partial update        */
    _atomic_leave(state);
    return v;
}

static inline void atomic_write_f32(volatile float *addr, float v)
{
    uint16_t state = _atomic_enter();
    *addr = v;                    /* write both 16?bit halves atomically */
    _atomic_leave(state);
}

#endif /* ATOMIC_H */