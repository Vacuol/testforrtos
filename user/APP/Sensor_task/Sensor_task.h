#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H


#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "AHRS.h"

#define INS_TASK_INIT_TIME 7		//任务开始初期 delay 一段时间

extern void Sensor_task(void const * argument);

extern const fp32 *get_INS_angle_point(void);
extern const fp32 *get_MPU6500_Gyro_Data_Point(void);
extern const fp32 *get_MPU6500_Accel_Data_Point(void);

#endif
