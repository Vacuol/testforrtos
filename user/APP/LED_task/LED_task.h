#ifndef LED_TASK_H
#define LED_TASK_H


#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

extern void Sensor_Task(void const * argument);

#endif
