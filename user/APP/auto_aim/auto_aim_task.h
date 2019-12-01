#ifndef AUTO_AIM_TASK_H
#define AUTO_AIM_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "remote.h"



#define TX2_X_HIGHOFFSET 2
#define TX2_X_LOWOFFSET 3
#define TX2_Y_HIGHOFFSET 5
#define TX2_Y_LOWOFFSET 6

typedef struct
{
	int16_t initial;
	int16_t relative;
	float angle[2];
	float gyro[2];
	float speed;
	float accl;
	float filted_angle;
	float filted_speed;
	
} Target_t;

typedef struct
{
	uint8_t data[9];
	uint8_t flag_end;
	char check;

} TX2_t;

typedef struct
{
	Target_t x,y;
	const float *gimbal_gyro_point;
	float Ts;
	const RC_ctrl_t *aim_rc_ctrl;
	TX2_t TX2;
	
} Aim_t;

extern uint8_t TX2_data[1];
extern void auto_aim_task(void const * argument);
extern void Getdata_Camera();
extern const Target_t *get_y_autoaim_point(void);
extern const Target_t *get_x_autoaim_point(void);

#endif