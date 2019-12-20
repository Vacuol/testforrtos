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

#define CHASSIS_TASK_CONTROL_TIME 1

//
#define CHASSIS_PID_MODE PID_POSITION
#define CHASSIS_PID_MAX_OUT 3000
#define CHASSIS_PID_MAX_IOUT 500
#define CHASSIS_PID_KP 4
#define CHASSIS_PID_KI 1
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
	float M_global_local[4];
	int16_t global_front;
	int16_t global_left;
	float local_front;
	float local_left;
	int16_t rotate;
	
} Dirction;

typedef struct
{
	const Gimbal_Motor_t *chassis_yaw_motor;	
	const Gimbal_Motor_t *chassis_pitch_motor;
	const RC_ctrl_t *chassis_RC;                //����ʹ�õ�ң����ָ��
	Chassis_Motor_t chassis_motor[4];
	
	Dirction dir;
	
}Chassis_Control_t;

extern void chassis_task(void const * argument);
#endif
