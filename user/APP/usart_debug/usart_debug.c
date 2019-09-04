/**
  *******************************************************
  * @file       usart_debug.c/h
  * @brief      串口4的发送数据，用于调试
  * @note       

  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *******************************************************
  */
 
#include "usart_debug.h"

extern UART_HandleTypeDef huart4;


void sendware(void *wareaddr, uint32_t waresize)
{
	#define CMD_WARE     3
	uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    
	uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};   
	HAL_UART_Transmit(&huart4, (uint8_t *)cmdf, sizeof(cmdf), 5000);
	HAL_UART_Transmit(&huart4, (uint8_t *)wareaddr, waresize ,5000);
	HAL_UART_Transmit(&huart4, (uint8_t *)cmdr, sizeof(cmdr), 5000);
}
