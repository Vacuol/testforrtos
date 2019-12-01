#include "gimbal_task.h"
#include "filter.h"
#include "arm_math.h"

#define int_abs(x) ((x) > 0 ? (x) : (-x))

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


#define JSCOPE_WATCH_gimbal 1
#if JSCOPE_WATCH_gimbal
//j-scope 帮助pid调参
static void Jscope_Watch_gimbal(void);
#endif
	
#define SYSTEM_IDENTIFICATION 0
#if SYSTEM_IDENTIFICATION
uint16_t point_all,t;
float once_lenth,x,y;
float fs[]={
4.0,1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0,
5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 
10.0, 10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 
14.0, 14.5, 15.0, 15.5, 16.0, 16.5, 17.0, 17.5, 
18.0, 18.5, 19.0, 19.5, 20.0, 20.5, 21.0, 21.5,
22.0,24, 26, 28, 30, 32, 34, 36, 38,50, 60, 70, 
80, 90, 100, 110, 120, 200, 250, 333};

#endif 
float point_all,t;
float once_lenth,x,y;

Gimbal_Control_t gimbal_control;
Filter_t gimbal_pitch_out_filter;
Filter_t gimbal_yaw_out_filter;
Filter_t corrector_yaw_speed,corrector_yaw_position;


static void Gimbal_Init(Gimbal_Control_t *gimbal_init);
static void Gimbal_Position_Reset(Gimbal_Control_t *gimbal_pos_reset);
static void Gimbal_Feedback_Update(Gimbal_Control_t *gimbal_feedback_updata);
static float Motor_Absolute_to_Relative_Change(float angle,float offset);
static float Motor_ecd_to_angle_Change(uint16_t ecd, uint16_t offset_ecd);
static void Gimbal_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
static void Gimbal_PID(Gimbal_Control_t *gimbal_pid);
static float Gimbal_AnglePID_Calculate(PID_Regulator_t *pid , float fdb, float set, float DELTA);

uint8_t T_aim;

void gimbal_task(void const * argument)
{
	uint32_t waitetime;
	int16_t timeconter;
	char i;
	
	//云台电机初始化
	Gimbal_Init(&gimbal_control);

	while(gimbal_control.pitch_motor.gyro == 0)
	{
		Gimbal_Feedback_Update(&gimbal_control);
	}
	osDelay(1000);									//等待陀螺仪零飘数据收集完毕
	
	Gimbal_Position_Reset(&gimbal_control);
	
	waitetime = xTaskGetTickCount();

	
	for (;;)
	{

#if SYSTEM_IDENTIFICATION
		for (i=0;i< (sizeof(fs) / 4);i++)
		{
		point_all = 20000/fs[i];
		once_lenth = fs[i]*2*PI/1000;
		t=0;
		x=0;
		while(t<point_all)
		{
			
			y =	arm_sin_f32(x)*2;
			if (arm_sin_f32(x)>0) y=2;
			if (arm_sin_f32(x)<0) y=-2;
			if (i==0 ) y=0;
			x += once_lenth;
			t++;
#endif 
//		point_all = 20000/3;
//		once_lenth = 3*2*PI/1000;
//		t=0;
//		x=0;
//		while(t<point_all)
//		{
//			y =	arm_sin_f32(x)*0.03;
//			x += once_lenth;
//			t++;
		Gimbal_Feedback_Update(&gimbal_control);
		Gimbal_Set_Contorl(&gimbal_control);
		
		Gimbal_PID(&gimbal_control);

		gimbal_pitch_out_filter.raw_value = gimbal_control.pitch_motor.speed_pid.out;
		gimbal_yaw_out_filter.raw_value = gimbal_control.yaw_motor.speed_pid.out;//y * 3000;
		Chebyshev100HzLPF(&gimbal_yaw_out_filter);
		Chebyshev100HzLPF(&gimbal_pitch_out_filter);
		CAN_CMD_Gimbal(gimbal_pitch_out_filter.filtered_value, gimbal_yaw_out_filter.filtered_value);

		
#if JSCOPE_WATCH_gimbal		
		Jscope_Watch_gimbal();
#endif
		
		osDelayUntil(&waitetime, GIMBAL_TASK_CONTROL_TIME);
	}
#if SYSTEM_IDENTIFICATION
}}
#endif 
//}//daihuijiushan
}

const Gimbal_Motor_t *get_yaw_motor_point(void)
{
	return &gimbal_control.yaw_motor;
}

const Gimbal_Motor_t *get_pitch_motor_point(void)
{
	return &gimbal_control.pitch_motor;
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
	//自瞄数据指针获取
	gimbal_init->yaw_motor.target = get_x_autoaim_point();
	gimbal_init->pitch_motor.target = get_y_autoaim_point();
	//yaw电机PID初始化
	PID_Init(&gimbal_init->yaw_motor.speed_pid, YAW_SPEED_PID_MODE,YAW_SPEED_PID_MAX_OUT,YAW_SPEED_PID_MAX_IOUT,YAW_SPEED_PID_KP,YAW_SPEED_PID_KI,YAW_SPEED_PID_KD);
	PID_Init(&gimbal_init->yaw_motor.angle_pid, YAW_ANGLE_PID_MODE,YAW_ANGLE_PID_MAX_OUT,YAW_ANGLE_PID_MAX_IOUT,YAW_ANGLE_PID_KP,YAW_ANGLE_PID_KI,YAW_ANGLE_PID_KD);
	//pitch电机PID初始化
	PID_Init(&gimbal_init->pitch_motor.speed_pid, PITCH_SPEED_PID_MODE,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT,PITCH_SPEED_PID_KP,PITCH_SPEED_PID_KI,PITCH_SPEED_PID_KD);
	PID_Init(&gimbal_init->pitch_motor.angle_pid, PITCH_ANGLE_PID_MODE,PITCH_ANGLE_PID_MAX_OUT,PITCH_ANGLE_PID_MAX_IOUT,PITCH_ANGLE_PID_KP,PITCH_ANGLE_PID_KI,PITCH_ANGLE_PID_KD);
	//云台电机中值初始化
	gimbal_init->pitch_motor.offset_ecd = PITCH_OFFSET_ECD;
	gimbal_init->yaw_motor.offset_ecd = YAW_OFFSET_ECD;

}

//电机位置重新设置，每次云台上电之前执行一次
static void Gimbal_Position_Reset(Gimbal_Control_t *gimbal_pos_reset)
{
	uint32_t waitetime;
	
	waitetime = xTaskGetTickCount();
	
	gimbal_pos_reset->pitch_motor.offset_AbToRe = 0;		//将转换offset置0
	gimbal_pos_reset->yaw_motor.offset_AbToRe = 0;
	
	while(int_abs( (gimbal_pos_reset->pitch_motor.gimbal_motor_measure->ecd
					-gimbal_pos_reset->pitch_motor.offset_ecd) ) > 50 )					//pitch电机没有抬到水平位置时执行
	{
		//仿照主任务，让云台抬升到水平位置
		Gimbal_Feedback_Update(gimbal_pos_reset);
		Gimbal_Set_Contorl(&gimbal_control);
									//留给模式设置的地方
		Gimbal_PID(&gimbal_control);//这里要注意用电机角度，注意模式设置
		gimbal_pitch_out_filter.raw_value = gimbal_control.pitch_motor.speed_pid.out;
		Chebyshev100HzLPF(&gimbal_pitch_out_filter);
		CAN_CMD_Gimbal(gimbal_pitch_out_filter.filtered_value, 0);
		
		osDelayUntil(&waitetime, GIMBAL_TASK_CONTROL_TIME);
	}
	
	//pitch到达水平位置后，重置陀螺仪测量的yaw和pitch角，设置转换offset
	gimbal_pos_reset->pitch_motor.offset_AbToRe = gimbal_pos_reset->pitch_motor.absolute_angle
													-gimbal_pos_reset->pitch_motor.relative_angle;
	
	gimbal_pos_reset->yaw_motor.offset_AbToRe = gimbal_pos_reset->yaw_motor.absolute_angle
													-gimbal_pos_reset->yaw_motor.relative_angle;
}

static void Gimbal_Feedback_Update(Gimbal_Control_t *gimbal_feedback)
{
	if (gimbal_feedback == NULL)
    {
        return;
    }
	//云台角速度数据更新
	gimbal_feedback->pitch_motor.gyro = *(gimbal_feedback->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET);
	gimbal_feedback->yaw_motor.gyro = *(gimbal_feedback->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
	//陀螺仪角度数据更新
	gimbal_feedback->pitch_motor.absolute_angle = *(gimbal_feedback->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
	gimbal_feedback->pitch_motor.absolute_angle = Motor_Absolute_to_Relative_Change(gimbal_feedback->pitch_motor.absolute_angle ,
																					gimbal_feedback->pitch_motor.offset_AbToRe);
	gimbal_feedback->yaw_motor.absolute_angle = *(gimbal_feedback->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
	gimbal_feedback->yaw_motor.absolute_angle = Motor_Absolute_to_Relative_Change(gimbal_feedback->yaw_motor.absolute_angle ,
																					gimbal_feedback->yaw_motor.offset_AbToRe);
	//云台角度数据更新
	gimbal_feedback->pitch_motor.relative_angle = Motor_ecd_to_angle_Change(gimbal_feedback->pitch_motor.gimbal_motor_measure->ecd,
																			gimbal_feedback->pitch_motor.offset_ecd);
	gimbal_feedback->yaw_motor.relative_angle = Motor_ecd_to_angle_Change(gimbal_feedback->yaw_motor.gimbal_motor_measure->ecd,
																		  gimbal_feedback->yaw_motor.offset_ecd);

}

//将绝对角度和相对角度整合
static float Motor_Absolute_to_Relative_Change(float angle,float offset)
{
	angle -= offset;
	if (angle>PI) angle -= 2*PI;
	if (angle<-PI) angle += 2*PI;
	return angle;
}


//将机器人水平正前方作为云台电机的零点
static float Motor_ecd_to_angle_Change(uint16_t ecd, uint16_t offset_ecd) 
{
	//计算当前位置与零点的差
	int32_t relative_ecd = ecd - offset_ecd;
	
	//若差值大于4096，则将结果减去8192，说明电机此时在零点负一侧
    if (relative_ecd > HALF_RANGE)
    {
        relative_ecd -= FULL_RANGE;
    }
	//若差值小于-4096，则将结果加上8192，说明电机此时在零点正一侧
    else if (relative_ecd < -HALF_RANGE)
    {
        relative_ecd += FULL_RANGE;
    }
	
    return relative_ecd * Motor_Ecd_to_Rad;
	
}

static void Gimbal_Set_Contorl(Gimbal_Control_t *gimbal_set_control)
{
	
	//右上开关拨到中间，回到中间
	if (gimbal_set_control->gimbal_rc_ctrl->rc.sleft == 0)
	{
		gimbal_set_control->pitch_motor.relative_angle_set = 0;
		gimbal_set_control->yaw_motor.relative_angle_set = 2;
	}

	T_aim++;
	if (T_aim==8)
	{
		T_aim=0;
		//右上开关拨上，开启自瞄
		if (gimbal_set_control->gimbal_rc_ctrl->rc.sleft == RC_UP)
		{
			gimbal_set_control->pitch_motor.relative_angle_set = 0;
			gimbal_set_control->yaw_motor.relative_angle_set = gimbal_set_control->yaw_motor.relative_angle - gimbal_set_control->yaw_motor.target->filted_angle;
		}
	}
	
	//右上开关拨到中间，回到中间
	if (gimbal_set_control->gimbal_rc_ctrl->rc.sleft == RC_MID)
	{
		gimbal_set_control->pitch_motor.relative_angle_set = 0;
		gimbal_set_control->yaw_motor.relative_angle_set =  gimbal_set_control->gimbal_rc_ctrl->rc.ch[0]*1.0/500;
	}
	
	
}

static void Gimbal_PID(Gimbal_Control_t *gimbal_pid)
{
	//pitch 
	Gimbal_AnglePID_Calculate(&gimbal_pid->pitch_motor.angle_pid , gimbal_pid->pitch_motor.relative_angle,
		gimbal_pid->pitch_motor.relative_angle_set, gimbal_pid->pitch_motor.gyro);
	
	gimbal_pid->pitch_motor.gyro_set = gimbal_pid->pitch_motor.angle_pid.out;
	PID_Calculate(&gimbal_pid->pitch_motor.speed_pid, gimbal_pid->pitch_motor.gyro, gimbal_pid->pitch_motor.gyro_set);
	
	//yaw 
#if SYSTEM_IDENTIFICATION
	gimbal_pid->yaw_motor.relative_angle_set = 0-gimbal_pid->yaw_motor.relative_angle;//(float)(gimbal_control.gimbal_rc_ctrl->rc.ch[1])/500.
	Gimbal_AnglePID_Calculate(&gimbal_pid->yaw_motor.angle_pid , 0, 
	gimbal_pid->yaw_motor.relative_angle_set, gimbal_pid->yaw_motor.gyro);
#else	
	Gimbal_AnglePID_Calculate(&gimbal_pid->yaw_motor.angle_pid , gimbal_pid->yaw_motor.relative_angle, 
	gimbal_pid->yaw_motor.relative_angle_set, gimbal_pid->yaw_motor.gyro);
#endif
	corrector_yaw_speed.raw_value = gimbal_pid->yaw_motor.angle_pid.out - gimbal_pid->yaw_motor.gyro;//-gimbal_pid->yaw_motor.gyro;//
	Corrector_Yaw_Speed(&corrector_yaw_speed);
	gimbal_pid->yaw_motor.gyro_set = -corrector_yaw_speed.filtered_value;
	PID_Calculate(&gimbal_pid->yaw_motor.speed_pid, gimbal_pid->yaw_motor.gyro_set, 0);
}

static float Gimbal_AnglePID_Calculate(PID_Regulator_t *pid , float fdb, float set, float DELTA)
{
	if (pid == NULL)
    {
        return 0.0f;
    }
	
	
	pid->err[2] = pid->err[1];
	pid->err[1] = pid->err[0];

	pid->set = set;
	pid->fdb = fdb;
	pid->err[0] = pid->set - pid->fdb;
	if (pid->mode == PID_POSITION)
	{
		pid->Pout = pid->kp * pid->err[0];
		pid->Iout += pid->ki * pid->err[0];
		pid->Dout = pid->kd *DELTA;
		LimitMax(pid->Iout, pid->max_iout);
		pid->out = pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
	}
	else if (pid->mode == PID_DELTA)
	{
		pid->Pout = pid->kp * (pid->err[0] - pid->err[1]);
		pid->Iout = pid->ki * pid->err[0];
		pid->Dout = pid->kd * DELTA;
		pid->out += pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
	}
	return pid->out;
	
}

#if JSCOPE_WATCH_gimbal
float jlook_taget;
float jlook_remote;
float jlook_1,jlook_2,jlook_3;
static void Jscope_Watch_gimbal(void)
{
	jlook_taget = gimbal_control.yaw_motor.target->filted_angle *100;
	jlook_remote = gimbal_control.gimbal_rc_ctrl->rc.sright;
	jlook_1 = gimbal_control.yaw_motor.relative_angle*1000;
	jlook_2 = gimbal_control.yaw_motor.relative_angle_set*100;
}
#endif
