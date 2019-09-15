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

//��ȡ��̬��ָ���ַ�󣬶�Ӧ��̬�ǵĵ�ַƫ���� fp32����
#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

//����ʼ���� delay һ��ʱ��

extern void Sensor_task(void const * argument);

extern const fp32 *get_INS_angle_point(void);
extern const fp32 *get_MPU6500_Gyro_Data_Point(void);
extern const fp32 *get_MPU6500_Accel_Data_Point(void);

#endif
