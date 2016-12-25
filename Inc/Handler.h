#ifndef __HANDLER__
#define __HANDLER__

#include "stm32f0xx_hal.h"

#define BUF_SIZE		160

void USER_UART_IRQHandler(UART_HandleTypeDef *huart);

#endif
