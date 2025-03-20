#ifndef SWITCHES_H
#define	SWITCHES_H

// Switch callback function
// When returning 0, the callback will be unregistered
// When returning 1, the callback will remain registered
typedef int16_t (*SwitchCallback_t)(void);

void registerSwitchCallback(SwitchCallback_t callback);
void remoeSwitchCallback(SwitchCallback_t callback);
void removeSwitchCallbacks();


#endif	/* SWITCHES_H */

