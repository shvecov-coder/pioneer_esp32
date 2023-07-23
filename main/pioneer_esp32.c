#include "pioneer_esp32.h"

void init_pioneer(Pioneer * pio)
{
	pio->ip = "192.168.4.1";
	pio->port = 8001;
	pio->system_id = 1;
}