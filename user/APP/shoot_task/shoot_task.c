#include "shoot_task.h"
#include "tim.h"

static Shoot_Control_t shoot_Control;

static void Shoot_Init(Shoot_Control_t *shoot_init);
static void Shoot_Feedback_Update(Shoot_Control_t *shoot_feedback);
static void Shoot_Set_Control(Shoot_Control_t *shoot_set);
static void Shoot_PID(Shoot_Control_t *shoot_pid);

void shoot_task(void const * argument)
{
	uint32_t waitetime;
	
	Shoot_Init(&shoot_Control);
	/** 开启摩擦轮PWM，初始化速度为0 */
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	TIM1->CCR1=1000;
	TIM1->CCR4=1000;
	osDelay(2000);
	
	waitetime = xTaskGetTickCount();
	
	for (;;)
	{
	
		Shoot_Feedback_Update(&shoot_Control);
		Shoot_Set_Control(&shoot_Control);
		Shoot_PID(&shoot_Control);
//		CAN_CMD_Rammer(shoot_Control.rammer.speed_pid.out);
		osDelayUntil(&waitetime, SHOOT_TASK_CONTROL_TIME);
	}
	
}

const float *get_rammer_out_point(void)
{
	return &shoot_Control.rammer.speed_pid.out;
}

static void Shoot_Init(Shoot_Control_t *shoot_init)
{
	if (shoot_init == NULL)
	{
		return ;
	}
	
	//遥控器获取数据
	shoot_init->shoot_RC = get_remote_control_point();
	//拨弹电机数据指针获取
	shoot_init->rammer.gimbal_motor_measure = get_Rammer_Motor_Measure_Point();
	//拨弹电机速度pid初始化
	PID_Init(&shoot_init->rammer.speed_pid, RAMMER_SPEED_PID_MODE,RAMMER_SPEED_PID_MAX_OUT,RAMMER_SPEED_PID_MAX_IOUT,RAMMER_SPEED_PID_KP,RAMMER_SPEED_PID_KI,RAMMER_SPEED_PID_KD);
}

static void Shoot_Feedback_Update(Shoot_Control_t *shoot_feedback)
{
	if (shoot_feedback == NULL)
    {
        return;
    }
	
	shoot_feedback->rammer.speed = shoot_feedback->rammer.gimbal_motor_measure->speed_rpm;
}

static void Shoot_Set_Control(Shoot_Control_t *shoot_set)
{
	if (shoot_set == NULL)
    {
        return;
    }
	
	if (shoot_set->shoot_RC->rc.ch[1] == RC_UP) 
		shoot_set->rammer.speed_set = 500;
	else shoot_set->rammer.speed_set = 0;
	
	if (shoot_set->shoot_RC->rc.ch[1] == RC_UP || shoot_set->shoot_RC->rc.ch[1] == RC_MID)
	{
		TIM1->CCR1=1500;
		TIM1->CCR4=1500;
	}
}

static void Shoot_PID(Shoot_Control_t *shoot_pid)
{
	if (shoot_pid == NULL)
    {
        return;
    }
	
	PID_Calculate(&shoot_pid->rammer.speed_pid,shoot_pid->rammer.speed,shoot_pid->rammer.speed_set);
	
}

