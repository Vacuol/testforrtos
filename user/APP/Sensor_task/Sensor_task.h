#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H


#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "AHRS.h"
#include "ist8310.h"
#include "mpu6500.h"

#define INS_TASK_INIT_TIME 7
#define ZERODRIFT_TIMES 10000
//任务开始初期 delay 一段时间

extern void Sensor_task(void const * argument);

void MPU6500_GetData(void);
void IMU_Cali_Slove(float gyro[3], fp32 accel[3], fp32 mag[3], MPU6500 *mpu6500, IST8310 *ist8310);
void Offset_Getdata(void);
void Offset_Cal(void);
void MPU6500_GetAngle(void);
uint8_t MPU6500_Init(void);

extern const fp32 *get_INS_angle_point(void);
extern const fp32 *get_MPU6500_Gyro_Data_Point(void);
extern const fp32 *get_MPU6500_Accel_Data_Point(void);

#endif
