/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "serial.h"
#include "can.h"
#include "CAN_ID.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

WWDG_HandleTypeDef hwwdg;

osThreadId Rad_TxHandle;
osThreadId Can_TxHandle;
osThreadId Rad_RxHandle;
osMessageQId CanTxBufHandle;
osMessageQId RadTxBufHandle;
osTimerId WWDGTmrHandle;
osTimerId HBTmrHandle;
osMutexId sttswrdMtxHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t statusWord;
static const uint32_t firmwareVersion = 0x00010203;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_WWDG_Init(void);
void doRadTx(void const * argument);
void doCanTx(void const * argument);
void doRadRx(void const * argument);
void TmrKickDog(void const * argument);
void TmrSendHB(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void (*cleanupCB)();		// System shutdown cleanup callback
static inline void shutdown_routine(void *pt){
	// Don't care about locking the statusWord here since we are in Critical Area
	statusWord &= 0xfffffff8;	// Clear the node status field
	statusWord |= SHUTDOWN;		// Set status to shutdown

	// Send status word indicating node shutdown to CC
	static Can_frame_t newFrame;
	newFrame.isExt = 0;
	newFrame.isRemote = 0;
	newFrame.core.id = radio_SW;
	newFrame.core.dlc = CAN_HB_DLC;
	for(int i=0; i<4; i++){
		newFrame.core.Data[3-i] = (statusWord >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
	}

	xQueueSendFromISR(CanTxBufHandle, &newFrame, pdFALSE);
	xQueueSendFromISR(RadTxBufHandle, &newFrame.core, pdFALSE);	// Add this message to the radio queue as well
	HAL_Delay(1);												// 1ms delay to ensure the queue is properly updated before proceeding

	// Loop through CAN Tx buffer to send any pending messages
	while(uxQueueMessagesWaitingFromISR(CanTxBufHandle) != 0){
		while(Can_availableForTx() == 0){	// Wait if bxCAN module is still busy
			HAL_WWDG_Refresh(&hwwdg);		// Since we are in the
			HAL_Delay(1);					// Hard delay to flush CAN tx buffers
		}
		xQueueReceiveFromISR(CanTxBufHandle, &newFrame, pdFALSE);
		Can_sendStd(newFrame.core.id,newFrame.isRemote,newFrame.core.Data,newFrame.core.dlc);
	}

	// Send any pending messages on the radio Tx buffer
	while(uxQueueMessagesWaitingFromISR(RadTxBufHandle) != 0){
		static Can_frame_core_t newCore;
		xQueueReceiveFromISR(RadTxBufHandle, &newCore, pdFALSE);	// Only run the transmit if the queue is non-empty
		static uint8_t radrxmsg[13];
		for(int i=0; i<4; i++){
			radrxmsg[3-i] = (newCore.id >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
		}
		radrxmsg[4] = newCore.dlc;
		for(int i=0; i<newCore.dlc; i++){
			radrxmsg[5+i] = newCore.Data[i];
		}
		while(Serial2_availableForWrite() == 0){
			HAL_WWDG_Refresh(&hwwdg);
			HAL_Delay(1);
		}
		Serial2_writeBytes_Async(radrxmsg, newCore.dlc+5, async_Interval);
		HAL_Delay(5);			// Delay for all the data in the radio buffer to be sent
	}

	// Perform shutdown cleanup
	if(pt != NULL){
		cleanupCB = pt;
		cleanupCB();
	}
}

void node_start(){
	if((statusWord & 0x07) == SHUTDOWN) {
#ifdef DEBUG
	static uint8_t msg[] = "Node starting\n";
	Serial2_writeBytes_Async(msg, sizeof(msg)-1,async_Interval);
#endif
		statusWord &= 0xfffffff8;	// Clear the node status field
		statusWord |= INIT;			// Set status to INIT to reestablish handshake with CC
		xTimerStartFromISR(HBTmrHandle, pdFALSE); // Start the CAN heart-beats
	}
}

void node_shutdown(void * pt){
#ifdef DEBUG
	static uint8_t msg[] = "Node shutting down\n";
	Serial2_writeBytes_Async(msg, sizeof(msg)-1,async_Interval);
#endif
	shutdown_routine(pt);		// Flush the queues and send any remaining messages
	// CAN reception must still be maintained to listen on the CAN interface
	xTimerStopFromISR(HBTmrHandle, pdFALSE); // Stop the CAN heart-beats
	// Watchdog timer should not/cannot be stopped or the system will reset
}

/*
 * Soft reset
 * Will broadcast node shutdown and do any necessary cleanups before resetting the system
 */
void node_reset(void * pt){
#ifdef DEBUG
	static uint8_t msg[] = "Node reset issued\n";
	Serial2_writeBytes_Async(msg, sizeof(msg)-1, async_Interval);
#endif
	node_shutdown(pt);
	NVIC_SystemReset();					// CMSIS System reset function
}

void node_hreset(){
#ifdef DEBUG
	static uint8_t msg[] = "Node hard reset issued\n";
	Serial2_writeBytes(msg, sizeof(msg)-1);
#endif
	NVIC_SystemReset();					// CMSIS System reset function
}

void CanRxCallback(){
	static Can_frame_core_t newCore;
	UBaseType_t x = taskENTER_CRITICAL_FROM_ISR();
	// Parse the CAN frame
	newCore.id = (hcan1.pRxMsg->IDE) ? hcan1.pRxMsg->ExtId : hcan1.pRxMsg->StdId;
	newCore.dlc = hcan1.pRxMsg->DLC;
	if(hcan1.pRxMsg->RTR == 0){
		for(int i=0; i<newCore.dlc; i++){
			newCore.Data[i] = hcan1.pRxMsg->Data[i];
		}
	}

	if(((statusWord & 0x07) == INIT) || ((statusWord & 0x07) == INIT)) {
		xQueueSendFromISR(RadTxBufHandle, &newCore, pdFALSE);
	}

	if(newCore.id == radio_P2P){
		// Check if it is a command
		switch(newCore.Data[0]){
		case NODE_RESET:
			node_reset(NULL);
			break;

		case NODE_HRESET:
			node_hreset();
			break;

		case NODE_SHUTDOWN:
			node_shutdown(NULL);
			break;

		case NODE_START:
			node_start();
			break;

		case CC_ACK:
			statusWord &= 0xfffffff8;	// Clear the node status field
			statusWord |= ACTIVE;		// Set status to active
			break;

		case CC_NACK:
			if((statusWord & 0x07) == INIT){
				statusWord &= 0xfffffff8;	// Clear the node status field
				statusWord |= SHUTDOWN;		// Set status to active
			}
			break;
		}
	}
	taskEXIT_CRITICAL_FROM_ISR(x);
}

void cantxcb(){
	static uint8_t txcpltmsg[] = "\nSENT!\n";
	Serial2_writeBytes_Async(txcpltmsg, sizeof(txcpltmsg)-1, async_Interval);
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	// Cortex-M4F inverted interrupt priority; check FreeRTOS documentation

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_WWDG_Init();

  /* USER CODE BEGIN 2 */
  // Radio startup runonce
  Serial2_begin();
  static uint8_t startmsg[] = "Radio boot\n";
  Serial2_writeBytes(startmsg, sizeof(startmsg)-1);
  statusWord |= INIT;

  Can_begin();
  Can_setRxCallback(CanRxCallback);
#ifdef DEBUG
  Can_setTxCallback(cantxcb);
#endif

  Can_addMaskedFilterStd(0,0,0); // Catch all CAN ID
  Can_addMaskedFilterExt(0,0,0);

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of sttswrdMtx */
  osMutexDef(sttswrdMtx);
  sttswrdMtxHandle = osMutexCreate(osMutex(sttswrdMtx));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of WWDGTmr */
  osTimerDef(WWDGTmr, TmrKickDog);
  WWDGTmrHandle = osTimerCreate(osTimer(WWDGTmr), osTimerPeriodic, NULL);

  /* definition and creation of HBTmr */
  osTimerDef(HBTmr, TmrSendHB);
  HBTmrHandle = osTimerCreate(osTimer(HBTmr), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(WWDGTmrHandle, WD_Interval); //must be < 26
  osTimerStart(HBTmrHandle, HB_Interval);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of Rad_Tx */
  osThreadDef(Rad_Tx, doRadTx, osPriorityLow, 0, 256);
  Rad_TxHandle = osThreadCreate(osThread(Rad_Tx), NULL);

  /* definition and creation of Can_Tx */
  osThreadDef(Can_Tx, doCanTx, osPriorityBelowNormal, 0, 256);
  Can_TxHandle = osThreadCreate(osThread(Can_Tx), NULL);

  /* definition and creation of Rad_Rx */
  osThreadDef(Rad_Rx, doRadRx, osPriorityNormal, 0, 256);
  Rad_RxHandle = osThreadCreate(osThread(Rad_Rx), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of CanTxBuf */
  osMessageQDef(CanTxBuf, 8, Can_frame_t);
  CanTxBufHandle = osMessageCreate(osMessageQ(CanTxBuf), NULL);

  /* definition and creation of RadTxBuf */
  osMessageQDef(RadTxBuf, 8, Can_frame_core_t);
  RadTxBufHandle = osMessageCreate(osMessageQ(RadTxBuf), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_3TQ;
  hcan1.Init.BS1 = CAN_BS1_12TQ;
  hcan1.Init.BS2 = CAN_BS2_3TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_7B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* WWDG init function */
static void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 127;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* doRadTx function */
void doRadTx(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  // TODO: Add RTC Timestamp (if radio baud rate allows)
	  static Can_frame_core_t newCore;
	  xQueueReceive(RadTxBufHandle, &newCore, portMAX_DELAY);	// Only run the transmit if the queue is non-empty
	  if((statusWord & 0x07) == ACTIVE){
		  static uint8_t radrxmsg[13];
		  for(int i=0; i<4; i++){
			radrxmsg[3-i] = (newCore.id >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
		  }
		  radrxmsg[4] = newCore.dlc;
		  for(int i=0; i<newCore.dlc; i++){
			radrxmsg[5+i] = newCore.Data[i];
		  }

		  // TODO: Change the following 4 lines for alternative radio options
		  while(Serial2_availableForWrite()==0){
			osDelay(radTxInterval);
		  }
		  Serial2_writeBytes(radrxmsg, newCore.dlc+5);
	  }
  }
  /* USER CODE END 5 */ 
}

/* doCanTx function */
void doCanTx(void const * argument)
{
  /* USER CODE BEGIN doCanTx */
  /* Infinite loop */
  for(;;)
  {
	  static Can_frame_t newFrame;
	  xQueueReceive(CanTxBufHandle, &newFrame, portMAX_DELAY);	// Only process CAN Transmissions when the queue is non-empty
	  if (((statusWord & 0x07) == ACTIVE) || ((statusWord & 0x07) == INIT)){
		  while(Can_availableForTx() == 0){	// Wait if bxCAN module is still busy
			  osDelay(canTxInterval);
		  }

		  if (newFrame.isExt){
			  Can_sendExt(newFrame.core.id,newFrame.isRemote,newFrame.core.Data,newFrame.core.dlc);
		  } else{
			  Can_sendStd(newFrame.core.id,newFrame.isRemote,newFrame.core.Data,newFrame.core.dlc);
		  }
  	  }
  }
  /* USER CODE END doCanTx */
}

/* doRadRx function */
void doRadRx(void const * argument)
{
  /* USER CODE BEGIN doRadRx */
  /* Infinite loop */
  for(;;)
  {
	  // TODO: Change this function for alternative radio option
	  static uint8_t mode; //state machine: 0=addr, 1=data
	  static Can_frame_core_t newCore;
	  static int radrxavail;
	  radrxavail = Serial2_available();
	  if(mode==0 && radrxavail>=5){
		  newCore.id = 0;
		  for(int i=0; i<4; i++){
			  newCore.id |= (Serial2_read()<<(8*(3-i)));
		  }
		  newCore.dlc = Serial2_read();
		  mode = 1;
	  }else if(mode==1 && radrxavail>=newCore.dlc){
		  for(int i=0; i<newCore.dlc; i++){
			  newCore.Data[i] = Serial2_read();
		  }
#ifdef DEBUG
	static uint8_t radrxmsg[] = "Radio Rxed\n";
	Serial2_writeBytes_Async(radrxmsg, sizeof(radrxmsg)-1,async_Interval);
#endif
		  mode = 0;
		  xQueueSend(CanTxBufHandle, &newCore, portMAX_DELAY);
	  } else{
		  osDelay(radRxInterval);
	  }
  }
  /* USER CODE END doRadRx */
}

/* TmrKickDog function */
void TmrKickDog(void const * argument)
{
  /* USER CODE BEGIN TmrKickDog */
	taskENTER_CRITICAL();
	HAL_WWDG_Refresh(&hwwdg);
	taskEXIT_CRITICAL();
  /* USER CODE END TmrKickDog */
}

/* TmrSendHB function */
void TmrSendHB(void const * argument)
{
  /* USER CODE BEGIN TmrSendHB */
	static Can_frame_t newFrame;
	if((statusWord & 0x07) == ACTIVE){
		xSemaphoreTake(sttswrdMtxHandle, portMAX_DELAY);
		newFrame.isExt = 0;
		newFrame.isRemote = 0;
		newFrame.core.id = radio_SW;
		newFrame.core.dlc = CAN_HB_DLC;
		for(int i=0; i<4; i++){
			newFrame.core.Data[3-i] = (statusWord >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
		}

#ifdef DEBUG
	static uint8_t hbmsg[] = "HB issued\n";
	Serial2_writeBytes_Async(hbmsg, sizeof(hbmsg)-1,async_Interval);
#endif

		xSemaphoreGive(sttswrdMtxHandle);
	} else if ((statusWord & 0x07) == INIT) {
		newFrame.isExt = 0;
		newFrame.isRemote = 0;
		newFrame.core.id = radio_FW;
		newFrame.core.dlc = CAN_FW_DLC;
		for(int i=0; i<4; i++){
			newFrame.core.Data[3-i] = (firmwareVersion >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
		}

#ifdef DEBUG
	static uint8_t hbmsg[] = "FW request issued\n";
	Serial2_writeBytes_Async(hbmsg, sizeof(hbmsg)-1,async_Interval);
#endif
	}
	xQueueSendToFront(CanTxBufHandle, &newFrame, 0);
  /* USER CODE END TmrSendHB */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

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
