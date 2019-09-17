#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can_receive.h"
#include "remote.h"
#include "Sensor_task.h"
#include "pid.h"

//yaw �ٶȻ� PID�����Լ� PID���������������
#define YAE_SPEED_PID_MODE_POSITION
#ifdef YAE_SPEED_PID_MODE_POSITION
#define YAW_SPEED_PID_MODE PID_POSITION
#define YAW_SPEED_PID_MAX_OUT 30000.0f
#define YAW_SPEED_PID_MAX_IOUT 5000.0f

#define YAW_SPEED_PID_KP 2200.0f
#define YAW_SPEED_PID_KI 20.0f
#define YAW_SPEED_PID_KD 0.0f
#endif

//yaw �ǶȻ� PID���� �⻷λ��ʽPID 
#define YAE_ANGLE_PID_MODE_POSITION
#ifdef YAE_ANGLE_PID_MODE_POSITION
#define YAW_ANGLE_PID_MODE PID_POSITION
#define YAW_ANGLE_PID_MAX_OUT 30000.0f
#define YAW_ANGLE_PID_MAX_IOUT 5000.0f

#define YAW_ANGLE_PID_KP 2200.0f
#define YAW_ANGLE_PID_KI 20.0f
#define YAW_ANGLE_PID_KD 0.0f
#endif


//pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_PID_MODE_POSITION
#ifdef PITCH_SPEED_PID_MODE_POSITION
#define PITCH_SPEED_PID_MODE PID_POSITION
#define PITCH_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 5000.0f

#define PITCH_SPEED_PID_KP 2200.0f
#define PITCH_SPEED_PID_KI 20.0f
#define PITCH_SPEED_PID_KD 0.0f
#endif

//pitch �ǶȻ� PID���� �⻷λ��ʽPID 
#define PITCH_ANGLE_PID_MODE_POSITION
#ifdef PITCH_ANGLE_PID_MODE_POSITION
#define PITCH_ANGLE_PID_MODE PID_POSITION
#define PITCH_ANGLE_PID_MAX_OUT 30000.0f
#define PITCH_ANGLE_PID_MAX_IOUT 5000.0f

#define PITCH_ANGLE_PID_KP 2200.0f
#define PITCH_ANGLE_PID_KI 20.0f
#define PITCH_ANGLE_PID_KD 0.0f
#endif

typedef struct
{
	const motor_measure_t *gimbal_motor_measure;
	
	float gyro;
	float gyro_set;
	float absolute_angle;
<<<<<<< HEAD
	float absolute_angle_set;
	float relative_angle;
	float relative_angle_set;
	
=======
>>>>>>> parent of 5313390... 云台数据更新
	PID_Regulator_t speed_pid;
	PID_Regulator_t angle_pid;
	
} Gimbal_Motor_t;

typedef struct
{
	const RC_ctrl_t *gimbal_rc_ctrl;
    const float	*gimbal_INT_angle_point;
    const float *gimbal_INT_gyro_point;
	Gimbal_Motor_t pitch_motor;
	Gimbal_Motor_t yaw_motor;
	
} Gimbal_Control_t;

 extern void gimbal_task(void const * argument);

#endif
