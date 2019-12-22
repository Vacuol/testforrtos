#include "auto_aim_task.h"
#include "Sensor_task.h"
#include "kalman_filter.h"
#include "usart.h"
#include <stdlib.h>

#define int_abs(x) ((x) > 0 ? (x) : (-x))

//卡尔曼的运算周期，单位ms
#define AURO_AIM_CONTROL_TIME 1

#define JSCOPE_WATCH_aim 1
#if JSCOPE_WATCH_aim
//j-scope 帮助pid调参
static void Jscope_Watch_gimbal(void);
#endif

#define INTERNALTEST 0
#if INTERNALTEST
uint16_t point_all,t;
float once_lenth,x,y,p;
float fs[]={
4.0,1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0,
5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 
10.0, 10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 
14.0, 14.5, 15.0, 15.5, 16.0, 16.5, 17.0, 17.5, 
18.0, 18.5, 19.0, 19.5, 20.0, 20.5, 21.0, 21.5,
22.0,24, 26, 28, 30, 32, 34, 36, 38,50, 60, 70, 
80, 90, 100, 110, 120, 200, 250, 333};
#endif 

static void Aim_Init(Aim_t *aim_init, kalman_filter_init_t *I);
static void Aim_Feedback_Update(Aim_t *aim);
static float Aim_Position_to_Angle(int16_t Pos);
static void Getdata_Internal(Aim_t *aim);

static kalman_filter_t F_aim;
static kalman_filter_init_t I;
Aim_t aim;

uint8_t TX2_data[1];

void auto_aim_task(void const * argument)
{
	uint32_t waitetime;
	uint8_t i;
	
	//初始化各项参数、指针, 定义kalman矩阵
	Aim_Init(&aim, &I);
	//初始化kalman矩阵
	kalman_filter_init(&F_aim, &I);
	
	waitetime = xTaskGetTickCount();
	for(;;)
	{
		
#if INTERNALTEST
		for (i=0;i< (sizeof(fs) / 4);i++)
		{
		point_all = 20000/fs[i];
		once_lenth = fs[i]*2*PI/1000;
		t=0;
		x=0;
		while(t<point_all)
		{
			p= rand()%100 -0; 
			y =	arm_sin_f32(x)*2 + p/1000-0.05;
//			if (arm_sin_f32(x)>0) y=2;
//			if (arm_sin_f32(x)<0) y=-2;
			if (i==0 ) y=0;
			x += once_lenth;
			t++;
#endif 
		
		Aim_Feedback_Update(&aim);
		
		kalman_filter_calc(&F_aim, aim.x.angle[0],aim.x.speed,aim.x.accl,aim.x.accl);
		if ( (F_aim.filtered_value[0] * F_aim.filtered_value[1] >0) && (int_abs(aim.x.relative) >100) )
			aim.x.filted_angle = F_aim.filtered_value[0] + F_aim.filtered_value[1]*AURO_AIM_CONTROL_TIME*0.03;
		else aim.x.filted_angle = F_aim.filtered_value[0];
//		aim.x.filted_angle = F_aim.filtered_value[0];
		aim.x.filted_speed = F_aim.filtered_value[1];
		
#if JSCOPE_WATCH_aim		
		Jscope_Watch_gimbal();
#endif

		osDelayUntil(&waitetime, AURO_AIM_CONTROL_TIME);
	}
	
	
#if INTERNALTEST
}}
#endif 
}

const Target_t *get_x_autoaim_point(void)
{
	return &aim.x;
}

const Target_t *get_y_autoaim_point(void)
{
	return &aim.y;
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
	
	I->Q_data[0] = 0.001; I->Q_data[1] = 0;
	I->Q_data[2] = 0; I->Q_data[3] = 0.001;
	
	I->R_data[0] = 0.001; I->R_data[1] = 0;
	I->R_data[2] = 0; I->R_data[3] = 0.001;
	
	I->H_data[0] = 1; I->H_data[1] = 0; 
	I->H_data[2] = 0; I->H_data[3] = 1;
	HAL_UART_Receive_DMA(&huart8,TX2_data,sizeof(TX2_data));
}

static void Aim_Feedback_Update(Aim_t *aim_feedback)
{
	//角速度存储
	aim_feedback->x.gyro[1] = aim_feedback->x.gyro[0];
	aim_feedback->y.gyro[1] = aim_feedback->y.gyro[0];
	//角速度更新
	aim_feedback->x.gyro[0] = *(aim_feedback->gimbal_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
	aim_feedback->y.gyro[0] = -*(aim_feedback->gimbal_gyro_point +INS_GYRO_X_ADDRESS_OFFSET);
	//算出速度增量
	aim_feedback->x.accl = aim_feedback->x.gyro[0] - aim_feedback->x.gyro[1];
	aim_feedback->y.accl = aim_feedback->y.gyro[0] - aim_feedback->y.gyro[1];
	//获取TX2数据
	Getdata_Internal(aim_feedback);
	//获取相对中心的坐标
	if (aim_feedback->x.initial == 0 && aim_feedback->y.initial == 0)
	{
		aim_feedback->x.relative = 0;
		aim_feedback->y.relative = 0;
	}
	else 
	{
		aim_feedback->x.relative = aim_feedback->x.initial - 720;
		aim_feedback->y.relative = aim_feedback->y.initial - 540;

	}
	//将非线性位置数据转为角度，变成线性数据
	aim_feedback->x.angle[1] = aim_feedback->x.angle[0]; 
	aim_feedback->y.angle[1] = aim_feedback->y.angle[0];
	aim_feedback->x.angle[0] = Aim_Position_to_Angle(aim_feedback->x.relative);
	aim_feedback->y.angle[0] = Aim_Position_to_Angle(aim_feedback->y.relative);
#if INTERNALTEST
	aim_feedback->x.angle[0] = y;
#endif
	aim_feedback->x.speed = (aim_feedback->x.angle[0] - aim_feedback->x.angle[1])/aim_feedback->Ts;
	aim_feedback->y.speed = (aim_feedback->x.angle[0] - aim_feedback->x.angle[1])/aim_feedback->Ts;
}

static float Aim_Position_to_Angle(int16_t pos)
{
	float angle;
	if (int_abs(pos) > 300) angle = atan((1.0*pos) /1500);
	else if (int_abs(pos) > 100) angle = atan((1.0*pos) /1400);
	else angle = atan((1.0*pos) /1400)*0.01*int_abs(pos)*0.01*int_abs(pos);
	
	return angle;
}

static void Getdata_Internal(Aim_t *aim)
{
	uint8_t i,j; 
}

float jx,jy;

void Getdata_Camera()
{
	switch (aim.TX2.flag_end)
	{
		case 0:
			aim.TX2.check =0;
			if (TX2_data[0]=='&')
				aim.TX2.flag_end=1;
			else 
				aim.TX2.flag_end=0;
		break;
			
		case 1:
			if (TX2_data[0]=='%')
				aim.TX2.flag_end=2;
			else 
				aim.TX2.flag_end=0;
		break;
			
		case 2:
			//aim.TX2.check =  TX2_data[0];
			aim.x.initial &= 0x00ff;
			aim.x.initial |= (TX2_data[0]<<8);
			aim.TX2.flag_end=3;
		break;
		
		case 3:
			aim.x.initial &= 0xff00;
			aim.TX2.check += TX2_data[0];
			aim.x.initial |= TX2_data[0];
			aim.TX2.flag_end=4;
		break;
	
		case 4:
//			aim.TX2.check += TX2_data[0];
			aim.y.initial &= 0x00ff;
			aim.y.initial |= (TX2_data[0]<<8);
			aim.TX2.flag_end=5;
		break;
		
		case 5:
			aim.y.initial &= 0xff00;
			aim.TX2.check += TX2_data[0];
			aim.y.initial |= TX2_data[0];
			aim.TX2.flag_end=6;
		break;
			
		case 6:
			if (aim.TX2.check != TX2_data[0]) 
			{
				aim.x.initial = 720;
				aim.y.initial = 540;
			}
			aim.TX2.flag_end=0;
		break;
	}
	
}

#if JSCOPE_WATCH_aim
float jlook_yrelative,jlook_yinitial,jlook_yangle;


static void Jscope_Watch_gimbal(void)
{

	jlook_yrelative = aim.y.relative;
	jlook_yinitial = aim.y.initial;
	jlook_yangle = aim.y.angle[0];
	
}
#endif







