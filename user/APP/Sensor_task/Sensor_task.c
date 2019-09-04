/**
  *******************************************************
  * @file       Sensor_task.c/h
  * @brief      主要利用陀螺仪mpu6500，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过mpu6500的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间，提供注释对应的宏定义，关闭DMA，
  * 
  * @note       

  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *******************************************************
  */
#include "Sensor_task.h"

#include "mpu6500.h"
#include "ist8310.h"
#include <math.h>
#include "usart_debug.h"


#define IMU_BOARD_INSTALL_SPIN_MATRIX                           \
                                        { 0.0f, 1.0f, 0.0f},    \
                                        {-1.0f, 0.0f, 0.0f},    \
                                        { 0.0f, 0.0f, 1.0f}    \

										
//static float Gyro_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //陀螺仪校准线性度
//static float gyro_cali_offset[3] ={0.0f, 0.0f, 0.0f};
//static float Gyro_Offset[3] = {0.0f, 0.0f, 0.0f};            //陀螺仪零漂
//static float Accel_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //加速度校准线性度
//static float Accel_Offset[3] = {0.0f, 0.0f, 0.0f};            //加速度零漂
//static float Mag_Scale_Factor[3][3] = {{1.0f, 0.0f, 0.0f},
//                                      {0.0f, 1.0f, 0.0f},
//                                      {0.0f, 0.0f, 1.0f}}; //磁力计校准线性度
//static float Mag_Offset[3] = {0.0f, 0.0f, 0.0f};            //磁力计零漂


//static float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
//static float INS_accel[3] = {0.0f, 0.0f, 0.0f};
//static float INS_mag[3] = {0.0f, 0.0f, 0.0f};

//static float INS_Angle[3] = {0.0f, 0.0f, 0.0f};      //欧拉角 单位 rad
//static float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //四元数

//static MPU6500 mpu6500;
//static IST8310 ist8310;


void Sensor_task(void const * argument)
{
	uint32_t waitetime;
	uint8_t timecounter=0;
	float output[8];
  /* USER CODE BEGIN StartLEDFlashTask */
	osDelay(INS_TASK_INIT_TIME);
	
	MPU6500_Init();
//	ist8310_init();
	while (ist8310_init() != IST8310_NO_ERROR){}
	
	Offset_Getdata();
	
//	waitetime = osKernelSysTick();
	
  /* Infinite loop */
	for(;;)
	{
//		osDelayUntil(&waitetime, 1);
		osDelay(1);
		MPU6500_GetData();
		Offset_Cal();
		IMU_Cali_Slove(INS_gyro, INS_accel, INS_mag, &mpu6500, &ist8310);
		MPU6500_GetAngle();
		
		timecounter++;
		if (timecounter==10) 
		{
			timecounter=0;
			output[0] = INS_Angle[0];
			output[1] = INS_Angle[1];
			output[2] = INS_Angle[2];
//			output[3] = mpu6500.GyroX;
//			output[4] = mpu6500.GyroY;
//			output[5] = mpu6500.GyroZ;
			sendware(output,sizeof(output));
		}
		
	
	}
  /* USER CODE END StartLEDFlashTask */
}




//通过传指针的方式，将数据传输出去
const fp32 *get_INS_angle_point(void)
{
    return INS_Angle;
}
const fp32 *get_MPU6500_Gyro_Data_Point(void)
{
    return INS_gyro;
}

const fp32 *get_MPU6500_Accel_Data_Point(void)
{
    return INS_accel;
}

