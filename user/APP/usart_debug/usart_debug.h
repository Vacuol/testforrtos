#ifndef __USART_DEBUG_H__
#define __USART_DEBUG_H__


#include "stm32f4xx_HAL.h"
	
extern UART_HandleTypeDef huart4;

void sendware(void *wareaddr, uint32_t waresize);

#endif
