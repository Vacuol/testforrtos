#include "chassis_task.h"


static Chassis_Control_t chassis_control;

static void chassis_init(Chassis_Control_t *chassis_init);

void chassis_task(void const * argument)
{ 
	
	//���̳�ʼ��
    chassis_init(&chassis_control);
	
	for (;;)
	{
		
	
	}
}

void chassis_init(Chassis_Control_t *chassis_init)
{
	if (chassis_init == NULL)
	{
		return ;
	}
	
	//��ȡң����ָ��
	chassis_init->chassis_RC = get_remote_control_point();
	//��ȡ��̨���ָ��
	chassis_init->chassis_yaw_motor = get_yaw_motor_point();	
	chassis_init->chassis_pitch_motor = get_pitch_motor_point();
	//���PID��ʼ��
	uint8_t i;
	for (uint8_t i=0 ; i<4 ; i++)
	{
		chassis_init->chassis_motor[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
		
	}
		
		
		
}



















