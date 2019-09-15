#include "chassis_task.h"


void chassis_task(void const * argument)
{
	uint8_t i3;
	for (;;)
	{
		i3++;
		osDelay(500);
	}
}

