#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H


#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "can_receive.h"
#include "remote.h"
#include "gimbal_task.h"
#include "pid.h"

//
#define CHASSIS_PID_MODE 0
#define CHASSIS_PID_MAX_OUT 0
#define CHASSIS_PID_MAX_IOUT 0
#define CHASSIS_PID_KP 0
#define CHASSIS_PID_KI 0
#define CHASSIS_PID_KD 0

typedef struct
{
	const motor_measure_t *chassis_motor_measure;
	float accel;
	float speed;
	float speed_set;
	
	PID_Regulator_t speed_pid;
	
} Chassis_Motor_t;

typedef struct
{
	const Gimbal_Motor_t *chassis_yaw_motor;
	const Gimbal_Motor_t *chassis_pitch_motor;
	const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
	
	Chassis_Motor_t chassis_motor[4];
	
}Chassis_Control_t;

extern void chassis_task(void const * argument);

#endif
