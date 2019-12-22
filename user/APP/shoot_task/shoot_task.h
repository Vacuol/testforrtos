#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "remote.h"
#include "pid.h"
#include "can_receive.h"

#define SHOOT_TASK_CONTROL_TIME 5

//Rammer 速度环 PID参数以及 PID最大输出，积分输出
#define RAMMER_SPEED_PID_MODE_POSITION
#ifdef RAMMER_SPEED_PID_MODE_POSITION
#define RAMMER_SPEED_PID_MODE PID_POSITION
#define RAMMER_SPEED_PID_MAX_OUT 6000.0f
#define RAMMER_SPEED_PID_MAX_IOUT 1000.0f

#define RAMMER_SPEED_PID_KP 2.0f
#define RAMMER_SPEED_PID_KI 0.0f
#define RAMMER_SPEED_PID_KD 0.0f
#endif

typedef struct
{
	const motor_measure_t *gimbal_motor_measure;
	
	float speed;
	float speed_set;
	
	PID_Regulator_t speed_pid;
	
} Rammer_Motor_t;

typedef struct
{
	const RC_ctrl_t *shoot_RC;
	
	Rammer_Motor_t rammer;
	uint8_t shoot_fre_set;
	uint16_t bullet_speed;
	
} Shoot_Control_t;

extern void shoot_task(void const * argument);
extern const float *get_rammer_out_point(void);

#endif
