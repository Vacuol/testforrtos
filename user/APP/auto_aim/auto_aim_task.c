#include "auto_aim_task.h"
#include "Sensor_task.h"
#include "kalman_filter.h"

#define AURO_AIM_CONTROL_TIME 2

static void Aim_Init(Aim_t *aim_init, kalman_filter_init_t *I);
static void Aim_Feedback_Update(Aim_t *aim);

uint8_t TX2_data[9];

static kalman_filter_t F_aim;
static kalman_filter_init_t I;
static Aim_t aim;

void auto_aim_task(void const * argument)
{
	uint32_t waitetime;
	
	//初始化各项参数、指针, 定义kalman矩阵
	Aim_Init(&aim, &I);
	//初始化kalman矩阵
	kalman_filter_init(&F_aim, &I);
	
	waitetime = xTaskGetTickCount();
	for(;;)
	{
		Aim_Feedback_Update(&aim);
		
		
		osDelayUntil(&waitetime, AURO_AIM_CONTROL_TIME);
	}
}


static void Aim_Init(Aim_t *aim_init, kalman_filter_init_t *I)
{
	//陀螺仪数据指针获取
	aim_init->gimbal_gyro_point = get_MPU6500_Gyro_Data_Point();
	//delta t
	aim_init->Ts = AURO_AIM_CONTROL_TIME/1000.0;
	//遥控器数据指针获取
	aim_init->aim_rc_ctrl = get_remote_control_point();
	//设定kalman矩阵
	I->A_data[0] = 1; I->A_data[1] = aim_init->Ts;
	I->A_data[2] = 0; I->A_data[3] = 1;
	
	I->B_data[0] = aim_init->Ts/2; I->B_data[1] = 0;
	I->B_data[2] = 0; I->B_data[2] = 1;
	
	I->Q_data[0] = 0.0001; I->Q_data[1] = 0;
	I->Q_data[2] = 0; I->Q_data[3] = 0.0001;
	
	I->R_data[0] = 0.0001; I->R_data[1] = 0;
	I->R_data[2] = 0; I->R_data[3] = 0.0001;
	
	I->H_data[0] = 1; I->H_data[1] = 0; 
	I->H_data[2] = 0; I->H_data[3] = 1;
}

static void Aim_Feedback_Update(Aim_t *aim_feedback)
{
	//角速度存储
	aim_feedback->x.speed[1] = aim_feedback->x.speed[0];
	aim_feedback->y.speed[1] = aim_feedback->y.speed[0];
	//角速度更新
	aim_feedback->x.speed[0] = *(aim_feedback->gimbal_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
	aim_feedback->y.speed[0] = *(aim_feedback->gimbal_gyro_point +INS_GYRO_X_ADDRESS_OFFSET);
	//算出速度增量
	aim_feedback->x.accl = aim_feedback->x.speed[0] - aim_feedback->x.speed[1];
	aim_feedback->y.accl = aim_feedback->y.speed[0] - aim_feedback->y.speed[1];
	
}

void Getdata_Camera()
{
	uint8_t i;
	
	for (i=0;i<9;i++) aim.TX2data[i]=TX2_data[i];
	
	
	
}













