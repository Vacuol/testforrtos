#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "can.h"

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_PIT_MOTOR_ID = 0x205,
    CAN_YAW_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
//    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

static uint8_t aData[8];

extern CAN_RxHeaderTypeDef  Rx1Message;

void CAN1_Init(void);
void CAN2_Init(void);
void get_motor_measure(motor_measure_t *motor,uint8_t aData[]);
void get_gimbal_motor_measuer(motor_measure_t *motor,uint8_t aData[]);
void CAN1_Getdata(CAN_RxHeaderTypeDef *pHeader,uint8_t aData[]);

//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
//void Underpan_motor_output(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);
//void Lift_motor_output(int16_t iq1,int16_t iq2);
//void CatchPro_motor_output(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);
//void Cloud_motor_output(int16_t iq1,int16_t iq2);


#endif
