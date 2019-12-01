#include "chassis_task.h"


static Chassis_Control_t chassis_control;

static void Chassis_Init(Chassis_Control_t *chassis_init);
static void Chassis_Feedback_Update(Chassis_Control_t *chassis_feedback);
static void Chassis_Set_Control(Chassis_Control_t *chassis_set);
static void Chassis_PID(Chassis_Control_t *chassis_pid);

void chassis_task(void const * argument)
{ 
	uint32_t waitetime;
	
	
	//底盘初始化
    Chassis_Init(&chassis_control);
	//等待陀螺仪初始化
	while (chassis_control.chassis_yaw_motor->gyro == 0);
	
	waitetime = xTaskGetTickCount();
	for (;;)
	{
		Chassis_Feedback_Update(&chassis_control);
		Chassis_Set_Control(&chassis_control);
		Chassis_PID(&chassis_control);
		
		Underpan_motor_output(chassis_control.chassis_motor[0].speed_pid.out
							,chassis_control.chassis_motor[1].speed_pid.out
							,chassis_control.chassis_motor[2].speed_pid.out
							,chassis_control.chassis_motor[3].speed_pid.out);
		
		osDelayUntil(&waitetime, GIMBAL_TASK_CONTROL_TIME);
	}
}

void Chassis_Init(Chassis_Control_t *chassis_init)
{
	uint8_t i;
	if (chassis_init == NULL)
	{
		return ;
	}
	
	//获取遥控器指针
	chassis_init->chassis_RC = get_remote_control_point();
	//获取云台电机指针
	chassis_init->chassis_yaw_motor = get_yaw_motor_point();	
	chassis_init->chassis_pitch_motor = get_pitch_motor_point();
	//电机PID初始化
	
	for (i=0 ; i<4 ; i++)
	{
		chassis_init->chassis_motor[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
		PID_Init(&chassis_init->chassis_motor[i].speed_pid, CHASSIS_PID_MODE,CHASSIS_PID_MAX_OUT,CHASSIS_PID_MAX_IOUT,CHASSIS_PID_KP,CHASSIS_PID_KI,CHASSIS_PID_KD);
	}

}

static void Chassis_Feedback_Update(Chassis_Control_t *chassis_feedback)
{
	if (chassis_feedback == NULL)
    {
        return;
    }
	
	uint8_t i;
	for (i=0; i<4; i++)
	{
		chassis_feedback->chassis_motor[i].speed = chassis_feedback->chassis_motor[i].chassis_motor_measure->speed_rpm;	
	}
}

static void Chassis_Set_Control(Chassis_Control_t *chassis_set)
{
	uint8_t i;
	if (chassis_set == NULL)
    {
        return;
    }
	
	for (i=0;i<4;i++)
	{
		chassis_set->chassis_motor[i].speed_set = 3000;
	}
	
}

static void Chassis_PID(Chassis_Control_t *chassis_pid)
{
	uint8_t i;
	if (chassis_pid == NULL)
    {
        return;
    }
	
	for (i=0;i<4;i++)
	{
		PID_Calculate(&chassis_pid->chassis_motor[i].speed_pid, 
			chassis_pid->chassis_motor[i].speed, chassis_pid->chassis_motor[i].speed_set);
	}
}













