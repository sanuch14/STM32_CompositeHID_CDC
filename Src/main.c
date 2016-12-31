/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
/*For using CDC and CustomHID class functions*/
#include "usbd_cdc_hid.h"
#include <stdio.h>
#include "Handler.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//blink LEDs var
uint8_t rxLed=0,txLed=0;

uint8_t uartBusy = 0;

char tmp[BUF_SIZE];
/*USBHID functions and variables declaration*/
extern USBD_HandleTypeDef hUsbDeviceFS;

/*CDC buffers declaration*/

char uart_tx[BUF_SIZE];
uint8_t countTx=0;
uint8_t writePointerTx=0, readPointerTx=0;

char uart_rx[BUF_SIZE];
uint8_t countRx=0;
uint8_t writePointerRx=0, readPointerRx=0;

/*SPI buffers declaration*/

char spi_tx[BUF_SIZE];
uint8_t spiCountTx=0;
uint8_t spiWritePointerTx=0, spiReadPointerTx=0;

char spi_rx[BUF_SIZE];
uint8_t spiCountRx=0;
uint8_t spiWritePointerRx=0, spiReadPointerRx=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void clearStr(char *buf){
	int i;
	for(i = 0; i<BUF_SIZE; i++){
		buf[i]=0;
	}
}

void blinkTX(){
	txLed=5;
	TIM3->CCR3=65535;
}

void blinkRX(){
	rxLed=5;
	TIM3->CCR4=65535;
}
///
void cdcTransmit(uint8_t size){
	blinkRX();
	for(int i=0;i<size;i++){
		tmp[i]=uart_rx[readPointerRx];
		readPointerRx=(readPointerRx+1)%BUF_SIZE;
		if(i==49){
			CDC_Transmit_FS((uint8_t*)tmp, 50);
			countRx-=50;
			size-=50;
			i=-1;
		}
	}
	if(size!=0){
		CDC_Transmit_FS((uint8_t*)tmp, size);
		countRx-=size;
	}
}
///

void usbTransmit(uint8_t id, uint8_t* readPointer, char* buffer, uint8_t* counter, uint8_t size){
	blinkRX();
	tmp[0]=id;
	for(int i=2;i<size+2;i++){//tmp[0] - id, tmp[1] - size, tmp[2...31] - data
		tmp[i]=buffer[*readPointer];
		*readPointer=(*readPointer+1)%BUF_SIZE;
		if(i==31){
			tmp[1]=30;
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)tmp, 32);
			*counter-=30;
			size-=30;
			i=1;
		}
	}
	if(size!=0){
		tmp[1]=size;
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)tmp, 32);
		*counter-=size;
	}
}
///
void uartTransmit(uint8_t size){
	blinkTX();
	for(int i=0;i<size;i++){
		tmp[i]=uart_tx[readPointerTx];
		readPointerTx=(readPointerTx+1)%BUF_SIZE;
		if(i==31){
			HAL_UART_Transmit(&huart1, (uint8_t*)tmp, 32,0xFF);
			countTx-=32;
			size-=32;
			i=-1;
		}
	}
	if(size!=0){
		countTx-=size;
		HAL_UART_Transmit(&huart1, (uint8_t*)tmp, size,0xFF);
	}
}
///SPI////
void spiTransmit(uint8_t size){
	blinkTX();
	for(uint8_t i=0; i<size;i+=4){
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&spi_tx[spiReadPointerTx],
							(uint8_t*)&spi_rx[spiWritePointerRx], 4, 0xFF);
		spiReadPointerTx=(spiReadPointerTx+4)%BUF_SIZE;
		spiWritePointerRx=(spiWritePointerRx+4)%BUF_SIZE;
	}
	spiCountRx+=size;
	spiCountTx-=size;
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
	MX_TIM3_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	
	HAL_UART_Receive_IT(&huart1, (uint8_t*)uart_rx, BUF_SIZE);	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(rxLed){
			rxLed--;
			if(rxLed==0)
				TIM3->CCR4 = 0;
		}
		if(txLed){
			txLed--;
			if(txLed==0)
				TIM3->CCR3 = 0;
		}
		//send received data to uart
		uint8_t cn = countTx;
		if(cn>0){
			uartTransmit(cn);
		}
		//receive data from uart
		cn = countRx;
		if(cn>0){
			if(uartBusy){
				usbTransmit(USB_UART_RX, &readPointerRx, uart_rx, &countRx, cn);
			}else{
				cdcTransmit(cn);//use cdc for data from uart
			}
		}
		////SPI////
		cn=spiCountTx;
		if(cn>=4){
			uint8_t size = cn-(cn%4);
			spiTransmit(size);
			usbTransmit(USB_SPI_RX, &spiReadPointerRx, spi_rx, &spiCountRx, size);
		}
		
		
		HAL_Delay(1);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	//////////////////////////////////////////////////////
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim6){
	/*Switch LEDs on PC side*/
//	if(cnt == 0){
//		dataToSend[0] = 4;
//		dataToSend[1] = 1;
//	}else if(cnt == 1){
//		dataToSend[0] = 4;
//		dataToSend[1] = 0;
//	}else if(cnt == 2){
//		dataToSend[0] = 5;
//		dataToSend[1] = 1;
//	}else if(cnt == 3){
//		dataToSend[0] = 5;
//		dataToSend[1] = 0;
//	}
//	/*Send messade*/
//	dataToSend[2] = 'H';
//	dataToSend[3] = 'I';
//	dataToSend[4] = 'D';
//	dataToSend[5] = 'U';
//	dataToSend[6] = 'S';
//	dataToSend[7] = 'B';
//	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, dataToSend, 32);
//	cnt++;
//	if (cnt>3)
//		cnt=0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
