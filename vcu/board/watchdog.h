#ifndef WATCHDOG_H
#define WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

// Starts the independent watchdog (IWDG, on the ~32 kHz LSI). If it isn't
// refreshed for about 250 ms the chip resets. It can't be stopped once started.
// Paused while the debugger has the core halted.
void watchdog_init(void);

void watchdog_refresh(void);

#ifdef __cplusplus
}
#endif

#endif
