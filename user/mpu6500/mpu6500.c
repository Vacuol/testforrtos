#include "mpu6500.h"
#include "spi.h"
#include "gpio.h"
#include <math.h>
#include "AHRS.h"
#include "cmsis_os.h"
	
#define IMU_BOARD_INSTALL_SPIN_MATRIX                           \
                                        { 0.0f, 1.0f, 0.0f},    \
                                        {-1.0f, 0.0f, 0.0f},    \
                                        { 0.0f, 0.0f, 1.0f}    \

//uint8_t MPU_id = 0;
//float Gyro_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //陀螺仪校准线性度
//float gyro_cali_offset[3] ={0.0f, 0.0f, 0.0f};
//float Gyro_Offset[3] = {0.0f, 0.0f, 0.0f};            //陀螺仪零漂
//float Accel_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //加速度校准线性度
//float Accel_Offset[3] = {0.0f, 0.0f, 0.0f};            //加速度零漂
//float Mag_Scale_Factor[3][3] = {{1.0f, 0.0f, 0.0f},
//                                      {0.0f, 1.0f, 0.0f},
//                                      {0.0f, 0.0f, 1.0f}}; //磁力计校准线性度
//float Mag_Offset[3] = {0.0f, 0.0f, 0.0f};            //磁力计零漂


//float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
//float INS_accel[3] = {0.0f, 0.0f, 0.0f};
//float INS_mag[3] = {0.0f, 0.0f, 0.0f};

//float INS_Angle[3] = {0.0f, 0.0f, 0.0f};      //欧拉角 单位 rad
//float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //四元数



//MPU6500 mpu6500,mpu6500_off;
//IST8310 ist8310;

void mpu6500_SPI_NS_H(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
}
void mpu6500_SPI_NS_L(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
}

//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
	static uint8_t MPU_Rx, MPU_Tx;

	mpu6500_SPI_NS_L();

	MPU_Tx = reg & 0x7f;
	HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
	MPU_Tx = data;
	HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);

	mpu6500_SPI_NS_H();
	return 0;
}

//Write registers to MPU6500
uint8_t MPU6500_Write_Regs(uint8_t const regAddr, uint8_t *pData ,uint8_t len)
{
	static uint8_t MPU_Rx, MPU_Tx, MPU_Rx_buff[14] = {0xff};;
	
	mpu6500_SPI_NS_L();
	
	MPU_Tx = regAddr & 0x7f;
	HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
	HAL_SPI_TransmitReceive(&hspi5, pData  , MPU_Rx_buff, len, 55);
	mpu6500_SPI_NS_H();
	return 0;
}

//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
	static uint8_t MPU_Rx, MPU_Tx;

	mpu6500_SPI_NS_L();

	MPU_Tx = reg | 0x80;
	HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
	HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);

	mpu6500_SPI_NS_H();
	return MPU_Rx;
}
//Read registers from MPU6500,address begin with regAddr
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
	static uint8_t MPU_Rx, MPU_Tx, MPU_Tx_buff[14] = {0xff};
	mpu6500_SPI_NS_L();

	MPU_Tx = regAddr | 0x80;
	MPU_Tx_buff[0] = MPU_Tx;
	HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
	HAL_SPI_TransmitReceive(&hspi5, MPU_Tx_buff, pData, len, 55);
	
	mpu6500_SPI_NS_H();
	return 0;
}
//Set the accelerated velocity resolution
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr << 3);
}

//Set the angular velocity resolution
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr << 3);
}

////Initialize the MPU6500
//uint8_t MPU6500_Init(void)
//{
//	uint8_t index = 0;
//	uint8_t MPU6500_Init_Data[10][2] =
//		{
//			{MPU6500_PWR_MGMT_1, 0x80},		// Reset Device
//			{MPU6500_PWR_MGMT_1, 0x03},		// Clock Source - Gyro-Z
//			{MPU6500_PWR_MGMT_2, 0x00},		// Enable Acc & Gyro
//			{MPU6500_CONFIG, 0x02},			// LPF 98Hz
//			{MPU6500_GYRO_CONFIG, 0x18},	// +-2000dps
//			{MPU6500_ACCEL_CONFIG, 0x10},   // +-8G
//			{MPU6500_ACCEL_CONFIG_2, 0x02}, // enable LowPassFilter  Set Acc LPF
//			{MPU6500_USER_CTRL, 0x20},		// Enable AUX
//		};

//	osDelay(100);
//	MPU_id = MPU6500_Read_Reg(MPU6500_WHO_AM_I); //read id of device,check if MPU6500 or not

//	for (index = 0; index < 10; index++)
//	{
//		MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
//		osDelay(1);
//	}
//	
////	KalmanInit(&mpu6500.PitchK);
////	KalmanInit(&mpu6500.RollK);

//	return 0;
//}

//void MPU6500_GetData(void)
//{
//	uint8_t mpu_buff[21];
//	int16_t temp_imu_data = 0;
//	MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 21);
//	
//	temp_imu_data = mpu_buff[0] << 8 | mpu_buff[1];
//	mpu6500.AccX = temp_imu_data * ACCEL_SEN;
//	temp_imu_data = mpu_buff[2] << 8 | mpu_buff[3];
//	mpu6500.AccY = temp_imu_data * ACCEL_SEN;
//	temp_imu_data = mpu_buff[4] << 8 | mpu_buff[5];
//	mpu6500.AccZ = temp_imu_data * ACCEL_SEN;
//	
//	temp_imu_data = mpu_buff[8] << 8 | mpu_buff[9];
//	mpu6500.GyroX = temp_imu_data * GYRO_SEN;
//	temp_imu_data = mpu_buff[10] << 8 | mpu_buff[11];
//	mpu6500.GyroY = temp_imu_data * GYRO_SEN;
//	temp_imu_data = mpu_buff[12] << 8 | mpu_buff[13];
//	mpu6500.GyroZ = temp_imu_data * GYRO_SEN;

//	if (mpu_buff[14] == 1) {
//		temp_imu_data = mpu_buff[16] << 8 | mpu_buff[15];
//		ist8310.mag[0] = temp_imu_data * MAG_SEN;
//		temp_imu_data = mpu_buff[18] << 8 | mpu_buff[17];
//		ist8310.mag[1] = temp_imu_data * MAG_SEN;
//		temp_imu_data = mpu_buff[20] << 8 | mpu_buff[19];
//		ist8310.mag[2] = temp_imu_data * MAG_SEN;
//	}

//}

//void IMU_Cali_Slove(float gyro[3], fp32 accel[3], fp32 mag[3], MPU6500 *mpu6500, IST8310 *ist8310)
//{
//	
//	for (uint8_t i = 0; i < 3; i++)
//    {
//        gyro[i] = mpu6500->GyroX * Gyro_Scale_Factor[i][0] + mpu6500->GyroY * Gyro_Scale_Factor[i][1] + mpu6500->GyroZ * Gyro_Scale_Factor[i][2] + Gyro_Offset[i];
//        accel[i] = mpu6500->AccX * Accel_Scale_Factor[i][0] + mpu6500->AccY * Accel_Scale_Factor[i][1] + mpu6500->AccZ * Accel_Scale_Factor[i][2] + Accel_Offset[i];
//        mag[i] = ist8310->mag[0] * Mag_Scale_Factor[i][0] + ist8310->mag[1] * Mag_Scale_Factor[i][1] + ist8310->mag[2] * Mag_Scale_Factor[i][2] + Mag_Offset[i];
//    }
//	
//}

//void Offset_Getdata(void)
//{
//	uint16_t t=2000;
//	
//	while(t--)
//	{
//		MPU6500_GetData();
//		
//		mpu6500_off.AccX += mpu6500.AccX;
//		mpu6500_off.AccY += mpu6500.AccY;
//		mpu6500_off.AccZ += mpu6500.AccZ;
//		
//		mpu6500_off.GyroX += mpu6500.GyroX;
//		mpu6500_off.GyroY += mpu6500.GyroY;
//		mpu6500_off.GyroZ += mpu6500.GyroZ;
//		
//		osDelay(1);
//		
//	}
//	
//	mpu6500_off.AccX /=2000;
//	mpu6500_off.AccY /=2000;
//	mpu6500_off.AccZ /=2000;
//	mpu6500_off.GyroX /=2000;
//	mpu6500_off.GyroY /=2000;
//	mpu6500_off.GyroZ /=2000;
//	
//	mpu6500.AccX = 0;
//	mpu6500.AccY = 0;
//	mpu6500.AccZ = 0;
//	mpu6500.GyroX = 0;
//	mpu6500.GyroY = 0;
//	mpu6500.GyroZ = 0;
//	
//}

//void Offset_Cal()
//{
////	mpu6500.AccX -= mpu6500_off.AccX;
////	mpu6500.AccY -= mpu6500_off.AccY;
////	mpu6500.AccZ -= mpu6500_off.AccZ;
//	mpu6500.GyroX -= mpu6500_off.GyroX;
//	mpu6500.GyroY -= mpu6500_off.GyroY;
//	mpu6500.GyroZ -= mpu6500_off.GyroZ;
//}

//void MPU6500_GetAngle()
//{
//	static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
//        static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
//        static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
//        static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
//		
//	static uint8_t updata_count = 0;
//		
//	 if (updata_count == 0)
//	{
//		//MPU6500_TEMPERATURE_PWM_INIT();
//		//PID_Init(&imuTempPid, PID_DELTA, imuTempPID, MPU6500_TEMPERATURE_PID_MAX_OUT, MPU6500_TEMPERATURE_PID_MAX_IOUT);

//		//初始化四元数
//		AHRS_init(INS_quat, INS_accel, INS_mag);
//		get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);

//		accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
//		accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
//		accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
//		updata_count++;
//	}
//	else
//	{
//		//加速度计低通滤波
//		accel_fliter_1[0] = accel_fliter_2[0];
//		accel_fliter_2[0] = accel_fliter_3[0];

//		accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

//		accel_fliter_1[1] = accel_fliter_2[1];
//		accel_fliter_2[1] = accel_fliter_3[1];

//		accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

//		accel_fliter_1[2] = accel_fliter_2[2];
//		accel_fliter_2[2] = accel_fliter_3[2];

//		accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];
//		
//		AHRS_update(INS_quat, TimingTime, INS_gyro, accel_fliter_3, INS_mag);
//		get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);
//		
//		
//	}
//}




