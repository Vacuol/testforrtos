#include "chassis_task.h"
#include "arm_math.h"

#define JSCOPE_WATCH_chassis 1 
#if JSCOPE_WATCH_chassis
//j-scope
static void Jscope_Watch_chassis(void);
#endif

static Chassis_Control_t chassis_control;
static float global_to_local[4];

static void Chassis_Init(Chassis_Control_t *chassis_init);
static void Chassis_Feedback_Update(Chassis_Control_t *chassis_feedback);
static void Chassis_Set_Control(Chassis_Control_t *chassis_set);
static void Chassis_dirction_trainsfer(Chassis_Control_t *chassis_trainsfer);
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
		
#if JSCOPE_WATCH_chassis		
		Jscope_Watch_chassis();
#endif
		
		
		osDelayUntil(&waitetime, CHASSIS_TASK_CONTROL_TIME);
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
	
	chassis_set->dir.global_front = chassis_set->chassis_RC->rc.ch[3]*5;
	chassis_set->dir.global_left = -chassis_set->chassis_RC->rc.ch[2]*5;
	chassis_set->dir.rotate = -chassis_set->chassis_RC->rc.wheel*5;
	
	Chassis_dirction_trainsfer(chassis_set);
	
	chassis_set->chassis_motor[0].speed_set = chassis_set->dir.local_front + chassis_set->dir.local_left + chassis_set->dir.rotate;
	chassis_set->chassis_motor[1].speed_set = chassis_set->dir.local_front - chassis_set->dir.local_left + chassis_set->dir.rotate;
	chassis_set->chassis_motor[2].speed_set = -chassis_set->dir.local_front - chassis_set->dir.local_left + chassis_set->dir.rotate;
	chassis_set->chassis_motor[3].speed_set = -chassis_set->dir.local_front + chassis_set->dir.local_left + chassis_set->dir.rotate;
	
}

static void Chassis_dirction_trainsfer(Chassis_Control_t *chassis_trainsfer)
{
	//
	chassis_trainsfer->dir.M_global_local[0] = arm_cos_f32(chassis_trainsfer->chassis_yaw_motor->relative_angle);
	chassis_trainsfer->dir.M_global_local[1] = -arm_sin_f32(chassis_trainsfer->chassis_yaw_motor->relative_angle);
	chassis_trainsfer->dir.M_global_local[2] = -arm_sin_f32(chassis_trainsfer->chassis_yaw_motor->relative_angle);
	chassis_trainsfer->dir.M_global_local[3] = -arm_cos_f32(chassis_trainsfer->chassis_yaw_motor->relative_angle);
	
	chassis_trainsfer->dir.local_front = 
		chassis_trainsfer->dir.global_front * chassis_trainsfer->dir.M_global_local[0] +
		chassis_trainsfer->dir.global_left * chassis_trainsfer->dir.M_global_local[1];
	
	chassis_trainsfer->dir.local_left =
		chassis_trainsfer->dir.global_front * chassis_trainsfer->dir.M_global_local[2] +
		chassis_trainsfer->dir.global_left * chassis_trainsfer->dir.M_global_local[3];
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

#if JSCOPE_WATCH_chassis 
float jlook_reangle,jlook_abangle;
float jlook_ch3;
static void Jscope_Watch_chassis(void)
{
	jlook_reangle = chassis_control.chassis_yaw_motor->relative_angle;
	jlook_abangle = chassis_control.chassis_yaw_motor->absolute_angle;
	jlook_ch3 = chassis_control.chassis_RC->rc.ch[3];
}
#endif











