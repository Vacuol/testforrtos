#ifndef AUTO_AIM_TASK_H
#define AUTO_AIM_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "remote.h"

typedef struct
{
	uint16_t initial;
	uint16_t relative;
	float angle[2];
	float gyro[2];
	float speed;
	float accl;
	float filted_angle;
	float filted_speed;
	
} Target_t;

typedef struct
{
	Target_t x,y;
	const float *gimbal_gyro_point;
	float Ts;
	const RC_ctrl_t *aim_rc_ctrl;
	uint8_t TX2data[9];
	
} Aim_t;



extern uint8_t TX2_data[9];
extern void auto_aim_task(void const * argument);
extern void Getdata_Camera();

#endif