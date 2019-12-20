#include "can_receive.h"
#include "detect_task.h"

//声明电机变量
static motor_measure_t motor_yaw, motor_pit, motor_rammer, motor_chassis[4];

CAN_RxHeaderTypeDef  Rx1Message;
CAN_TxHeaderTypeDef  Tx1Message;

uint32_t pTxMailbox;

void CAN1_Init()						
{
	CAN_FilterTypeDef canfilter;

	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
	
	//filtrate any ID you want here
	canfilter.FilterIdHigh = 0x0000;
	canfilter.FilterIdLow = 0x0000;
	canfilter.FilterMaskIdHigh = 0x0000;
	canfilter.FilterMaskIdLow = 0x0000;
  
	canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
	canfilter.FilterActivation = ENABLE;
	canfilter.SlaveStartFilterBank = 14;
	//use different filter for can1&can2
	canfilter.FilterBank=0;

	HAL_CAN_ConfigFilter(&hcan1,&canfilter);
	
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);

	HAL_CAN_Start(&hcan1);
}

//void CAN2_Init()						
//{
//	CAN_FilterTypeDef canfilter;
//	
//	//canfilter.FilterNumber = 14;
//	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
//	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
//	
//	//filtrate any ID you want here
//	canfilter.FilterIdHigh = 0x0000;
//	canfilter.FilterIdLow = 0x0000;
//	canfilter.FilterMaskIdHigh = 0x0000;
//	canfilter.FilterMaskIdLow = 0x0000;

//	canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
//	canfilter.FilterActivation = ENABLE;
//	canfilter.SlaveStartFilterBank = 14;
//	//use different filter for can1&can2
//	canfilter.FilterBank=14;

//	HAL_CAN_ConfigFilter(&hcan1,&canfilter);
//	
//	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);

//	HAL_CAN_Start(&hcan2);
//}

//返回yaw电机变量地址，通过指针方式获得原始数据
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//返回pitch电机变量地址，通过指针方式获得原始数据
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//返回trigger电机变量地址，通过指针方式获得原始数据
const motor_measure_t *get_Rammer_Motor_Measure_Point(void)
{
    return &motor_rammer;
}
//返回底盘电机变量地址，通过指针方式获得原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

//统一处理can中断函数，并记录发送数据的时间，作为离线判断的依据
void CAN1_Getdata(CAN_RxHeaderTypeDef *pHeader,uint8_t aData[])
{
	switch (pHeader->StdId)
	{
		case CAN_YAW_MOTOR_ID:
		{
			//处理云台电机数据		
			get_motor_measure(&motor_yaw, aData);
			//记录时间
			//etectHook(YawGimbalMotorTOE);
			
		}break;
		case CAN_PIT_MOTOR_ID:
		{
			//处理云台电机数据
			get_motor_measure(&motor_pit, aData);
			//记录时间
			//DetectHook(YawGimbalMotorTOE);
			
		}break;
		case CAN_3508_M1_ID:
		case CAN_3508_M2_ID:
		case CAN_3508_M3_ID:
		case CAN_3508_M4_ID:
		{
			static uint8_t i = 0;
			//处理电机ID号
			i = pHeader->StdId - CAN_3508_M1_ID;
			//处理电机数据
			get_motor_measure(&motor_chassis[i], aData);
			//记录时间
			//DetectHook(ChassisMotor1TOE + i);
			
		}break;
		case CAN_RAMMER_MOTOR_ID:
		{
			get_motor_measure(&motor_rammer, aData);
			//记录时间
		}
		default:
		{
			
		}break;
	}
}

void get_motor_measure(motor_measure_t *motor,uint8_t aData[])
{
	motor->last_ecd = motor->ecd;
	motor->ecd = aData[0]<<8|aData[1];
	motor->speed_rpm = aData[2]<<8|aData[3];
	motor->given_current = aData[4]<<8|aData[5];
	motor->temperate = aData[6];	
}

void Underpan_motor_output(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
{
	static uint8_t TxData[8];
	Tx1Message.StdId = 0x200;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	TxData[0] = iq1 >> 8;
	TxData[1] = iq1;
	TxData[2] = iq2 >> 8;
	TxData[3] = iq2;
	TxData[4] = iq3 >> 8;
	TxData[5] = iq3;
	TxData[6] = iq4 >> 8;
	TxData[7] = iq4;
	
	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,  TxData, &pTxMailbox);
}

void CAN_CMD_Gimbal(int16_t pitch,int16_t yaw,int16_t rammer)
{
	static uint8_t TxData[8];
	Tx1Message.StdId = 0x1ff;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	TxData[0] = pitch >> 8;
	TxData[1] = pitch;
	TxData[2] = yaw >> 8;
	TxData[3] = yaw;
	TxData[4] = rammer >> 8;
	TxData[5] = rammer;

	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,  TxData, &pTxMailbox);
}

void CAN_CMD_Rammer(int16_t rammer)
{
	static uint8_t TxData[8];
	Tx1Message.StdId = 0x1ff;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	TxData[4] = rammer >> 8;
	TxData[5] = rammer;

	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,  TxData, &pTxMailbox);
}




