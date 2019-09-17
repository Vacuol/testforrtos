#include "gimbal_task.h"

#define GIMBAL_TEST_MODE 1
#if GIMBAL_TEST_MODE
//j-scope ����pid����
static void J_scope_gimbal_test(void);
#endif

Gimbal_Control_t gimbal_control;

static void Gimbal_Init(Gimbal_Control_t *gimbal_init);
<<<<<<< HEAD
static void Gimbal_Feedback_Update(Gimbal_Control_t *gimbal_feedback_updata);
static float Motor_ecd_to_angle_Change(uint16_t ecd, uint16_t offset_ecd);
static void Gimbal_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
static void Gimbal_PID_Calculate(Gimbal_Control_t *gimbal_pid);
=======
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_updata);
>>>>>>> parent of 5313390... 云台数据更新

void gimbal_task(void const * argument)
{
	uint32_t waitetime;
	//��̨�����ʼ��
	Gimbal_Init(&gimbal_control);
	
	
	waitetime = xTaskGetTickCount();
	for (;;)
	{
		Gimbal_Feedback_Update(&gimbal_control);
		Gimbal_Set_Contorl(&gimbal_control);
		Gimbal_PID_Calculate(&gimbal_control);
		
		
		
		
		
		
#if GIMBAL_TEST_MODE		
		J_scope_gimbal_test();
#endif
		
		osDelayUntil(&waitetime, 1);
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
	PID_Init(&gimbal_init->yaw_motor.angle_pid, YAW_ANGLE_PID_MODE,YAW_ANGLE_PID_MAX_OUT,YAW_ANGLE_PID_MAX_IOUT,YAW_ANGLE_PID_KP,YAW_ANGLE_PID_KI,YAW_ANGLE_PID_KD);
	//pitch���PID��ʼ��
	PID_Init(&gimbal_init->pitch_motor.speed_pid, PITCH_SPEED_PID_MODE,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT,PITCH_SPEED_PID_KP,PITCH_SPEED_PID_KI,PITCH_SPEED_PID_KD);
	PID_Init(&gimbal_init->pitch_motor.angle_pid, PITCH_ANGLE_PID_MODE,PITCH_ANGLE_PID_MAX_OUT,PITCH_ANGLE_PID_MAX_IOUT,PITCH_ANGLE_PID_KP,PITCH_ANGLE_PID_KI,PITCH_ANGLE_PID_KD);
	
}

static void Gimbal_Feedback_Update(Gimbal_Control_t *gimbal_feedback)
{
	if (gimbal_feedback == NULL)
    {
        return;
    }
	//��̨���ٶ����ݸ���
	gimbal_feedback->pitch_motor.gyro = *(gimbal_feedback->gimbal_INT_gyro_point  + INS_GYRO_X_ADDRESS_OFFSET);
	gimbal_feedback->yaw_motor.gyro = *(gimbal_feedback->gimbal_INT_gyro_point  + INS_GYRO_Z_ADDRESS_OFFSET);
	//�����ǽǶ����ݸ���
	gimbal_feedback->pitch_motor.absolute_angle = *(gimbal_feedback->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
	gimbal_feedback->yaw_motor.absolute_angle = *(gimbal_feedback->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
	//��̨�Ƕ����ݸ���
<<<<<<< HEAD
	gimbal_feedback->pitch_motor.relative_angle = Motor_ecd_to_angle_Change(gimbal_feedback->pitch_motor.gimbal_motor_measure->ecd,
																			gimbal_feedback->pitch_motor.offset_ecd);
	gimbal_feedback->yaw_motor.relative_angle = Motor_ecd_to_angle_Change(gimbal_feedback->yaw_motor.gimbal_motor_measure->ecd,
																		  gimbal_feedback->yaw_motor.offset_ecd);
}


//��������ˮƽ��ǰ����Ϊ��̨��������
static float Motor_ecd_to_angle_Change(uint16_t ecd, uint16_t offset_ecd) 
{
	//���㵱ǰλ�������Ĳ�
	int32_t relative_ecd = ecd - offset_ecd;
	//����ֵ����4096���򽫽����ȥ8192��˵�������ʱ����㸺һ��
    if (relative_ecd > HALF_RANGE)
    {
        relative_ecd -= FULL_RANGE;
    }
	//����ֵС��-4096���򽫽������8192��˵�������ʱ�������һ��
    else if (relative_ecd < -HALF_RANGE)
    {
        relative_ecd += FULL_RANGE;
    }

    return relative_ecd * Motor_Ecd_to_Rad;
	
}

static void Gimbal_Set_Contorl(Gimbal_Control_t *gimbal_set_control)
{
	gimbal_set_control->pitch_motor.gyro_set = 100;
	
	gimbal_set_control->yaw_motor.gyro_set = 100;
	
}

static void Gimbal_PID_Calculate(Gimbal_Control_t *gimbal_pid)
{
	gimbal_pid->pitch_motor.speed_pid.set = gimbal_pid->pitch_motor.gyro_set;
	gimbal_pid->pitch_motor.speed_pid.fdb = gimbal_pid->pitch_motor.gyro;
	PID_Calculate(&gimbal_pid->pitch_motor.speed_pid);
	
	gimbal_pid->yaw_motor.speed_pid.set = gimbal_pid->yaw_motor.gyro_set;
	gimbal_pid->yaw_motor.speed_pid.fdb = gimbal_pid->yaw_motor.gyro;
	PID_Calculate(&gimbal_pid->yaw_motor.speed_pid);
}


#if JSCOPE_WATCH_gimbal
=======
	gimbal_feedback
	
}

#if GIMBAL_TEST_MODE
>>>>>>> parent of 5313390... 云台数据更新
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

