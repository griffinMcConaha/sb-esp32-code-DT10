#ifndef DISPERSION_CONTROLLER_H
#define DISPERSION_CONTROLLER_H

// Public entrypoint for the ESP32 spreader-control runtime.
// The implementation owns all peripheral bring-up and task creation.

// Initializes peripherals, starts runtime tasks, and enters controller loop.
void dispersion_controller_start(void);

#endif
