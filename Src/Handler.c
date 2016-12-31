#include "Handler.h"
extern uint8_t countRx;
extern uint8_t writePointerRx;

void USER_UART_IRQHandler(UART_HandleTypeDef *huart){
	/* UART in mode Receiver ---------------------------------------------------*/
  if((__HAL_UART_GET_IT(huart, UART_IT_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE) != RESET))
  {
  	/* Check that a Rx process is ongoing */
		if(huart->RxState == HAL_UART_STATE_BUSY_RX)
		{
			//*huart->pRxBuffPtr++ = (uint8_t)(huart->Instance->RDR & huart->Mask);
			huart->pRxBuffPtr[writePointerRx] = (uint8_t)(huart->Instance->RDR & huart->Mask);
			writePointerRx=(writePointerRx+1)%BUF_SIZE;
			countRx++;
			
			//if(--huart->RxXferCount == 0)
			if(huart->RxXferCount == 0)
			{
				__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
				/* Disable the UART Parity Error Interrupt */
				__HAL_UART_DISABLE_IT(huart, UART_IT_PE);
				/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
				__HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

				/* Rx process is completed, restore huart->RxState to Ready */
				huart->RxState = HAL_UART_STATE_READY;
				HAL_UART_RxCpltCallback(huart);

				return;
			}
			return;
		}
  }


  /* UART in mode Transmitter ------------------------------------------------*/
  if((__HAL_UART_GET_IT(huart, UART_IT_TXE) != RESET) &&(__HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE) != RESET))
  {
    UART_Transmit_IT(huart);
  }

  /* UART in mode Transmitter (transmission end) -----------------------------*/
 if((__HAL_UART_GET_IT(huart, UART_IT_TC) != RESET) &&(__HAL_UART_GET_IT_SOURCE(huart, UART_IT_TC) != RESET))
  {
    UART_EndTransmit_IT(huart);
  }
}
