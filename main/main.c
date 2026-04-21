#include "dispersion_controller.h"

void app_main(void)
{
	// Keep the ESP-IDF entrypoint tiny so the real controller lifecycle stays in
	// one place and can be reused/tested as a normal module.
	dispersion_controller_start();
}
