/**
  *******************************************************
  * @file       LED_task.c/h
  * @brief      led闪烁的程序，测试程序是否正常
  * @note       

  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *******************************************************
  */
#include "LED_task.h"
#include "Sensor_task.h"

void LEDFlash_Task(void const * argument)
{
	const float *angle;
  /* USER CODE BEGIN StartLEDFlashTask */
  /* Infinite loop */
  for(;;)
  {
//	  angle = get_INS_angle_point();
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
		osDelay(500);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
		osDelay(500);
	  
	  
  }
  /* USER CODE END StartLEDFlashTask */
}
