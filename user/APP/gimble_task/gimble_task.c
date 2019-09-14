#include "gimble_task.h"



void gimble_task(void const * argument)
{
	uint8_t i2;
	for (;;)
	{
		i2++;
		osDelay(500);
	}
}
