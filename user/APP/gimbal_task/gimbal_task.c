#include "gimbal_task.h"

Gimbal_Control_t gimbal_control;

static void Gimbal_Init(Gimbal_Control_t *gimbal_init);

void gimbal_task(void const * argument)
{
	
	
	Gimbal_Init(&gimbal_control);
	for (;;)
	{

		osDelay(500);
	}
}

static void Gimbal_Init(Gimbal_Control_t *gimbal_init)
{
	//�������ָ���ȡ
	gimbal_init->pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
	gimbal_init->yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	//����������ָ���ȡ
    gimbal_init->gimbal_INT_angle_point = get_INS_angle_point();
    gimbal_init->gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point();
	//ң��������ָ���ȡ
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
	//yaw���PID��ʼ��
	PID_Init(&gimbal_init->yaw_motor.speed_pid, YAW_SPEED_PID_MODE,YAW_SPEED_PID_MAX_OUT,YAW_SPEED_PID_MAX_IOUT,YAW_SPEED_PID_KP,YAW_SPEED_PID_KI,YAW_SPEED_PID_KD);
	//pitch���PID��ʼ��
	PID_Init(&gimbal_init->pitch_motor.speed_pid, PITCH_SPEED_PID_MODE,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT,PITCH_SPEED_PID_KP,PITCH_SPEED_PID_KI,PITCH_SPEED_PID_KD);
	
}


