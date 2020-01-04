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
	
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
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
