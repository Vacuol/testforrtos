#include "gimbal_task.h"

#define GIMBAL_TEST_MODE 1
#if GIMBAL_TEST_MODE
//j-scope 帮助pid调参
static void J_scope_gimbal_test(void);
#endif

Gimbal_Control_t gimbal_control;

static void Gimbal_Init(Gimbal_Control_t *gimbal_init);
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_updata);

void gimbal_task(void const * argument)
{
	uint32_t waitetime;
	//云台电机初始化
	Gimbal_Init(&gimbal_control);
	
	
	waitetime = xTaskGetTickCount();
	for (;;)
	{
		GIMBAL_Feedback_Update(&gimbal_control);
		
		
#if GIMBAL_TEST_MODE		
		J_scope_gimbal_test();
#endif
		
		osDelayUntil(&waitetime, 1);
	}
}

static void Gimbal_Init(Gimbal_Control_t *gimbal_init)
{
	//电机数据指针获取
	gimbal_init->pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
	gimbal_init->yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	//陀螺仪数据指针获取
    gimbal_init->gimbal_INT_angle_point = get_INS_angle_point();
    gimbal_init->gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point();
	//遥控器数据指针获取
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
	//yaw电机PID初始化
	PID_Init(&gimbal_init->yaw_motor.speed_pid, YAW_SPEED_PID_MODE,YAW_SPEED_PID_MAX_OUT,YAW_SPEED_PID_MAX_IOUT,YAW_SPEED_PID_KP,YAW_SPEED_PID_KI,YAW_SPEED_PID_KD);
	PID_Init(&gimbal_init->yaw_motor.angle_pid, YAW_ANGLE_PID_MODE,YAW_ANGLE_PID_MAX_OUT,YAW_ANGLE_PID_MAX_IOUT,YAW_ANGLE_PID_KP,YAW_ANGLE_PID_KI,YAW_ANGLE_PID_KD);
	//pitch电机PID初始化
	PID_Init(&gimbal_init->pitch_motor.speed_pid, PITCH_SPEED_PID_MODE,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT,PITCH_SPEED_PID_KP,PITCH_SPEED_PID_KI,PITCH_SPEED_PID_KD);
	PID_Init(&gimbal_init->pitch_motor.angle_pid, PITCH_ANGLE_PID_MODE,PITCH_ANGLE_PID_MAX_OUT,PITCH_ANGLE_PID_MAX_IOUT,PITCH_ANGLE_PID_KP,PITCH_ANGLE_PID_KI,PITCH_ANGLE_PID_KD);
	
}

static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback)
{
	if (gimbal_feedback == NULL)
    {
        return;
    }
	//云台角速度数据更新
	gimbal_feedback->pitch_motor.gyro = *(gimbal_feedback->gimbal_INT_gyro_point  + INS_GYRO_X_ADDRESS_OFFSET);
	gimbal_feedback->yaw_motor.gyro = *(gimbal_feedback->gimbal_INT_gyro_point  + INS_GYRO_Z_ADDRESS_OFFSET);
	//陀螺仪角度数据更新
	gimbal_feedback->pitch_motor.absolute_angle = *(gimbal_feedback->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
	gimbal_feedback->yaw_motor.absolute_angle = *(gimbal_feedback->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
	//云台角度数据更新
	gimbal_feedback
	
}

#if GIMBAL_TEST_MODE
int32_t jlook_p_gyro,jlook_y_gyro;
int32_t jlook_p_angle,jlook_y_angle;
static void J_scope_gimbal_test(void)
{
	jlook_p_gyro = gimbal_control.pitch_motor.gyro * 1000;
	jlook_y_gyro = gimbal_control.yaw_motor.gyro * 1000;
	jlook_p_angle = gimbal_control.pitch_motor.absolute_angle * 1000;
	jlook_y_angle = gimbal_control.yaw_motor.absolute_angle * 1000;
}
#endif

