/*
 * can_bus_tasks.c
 *
 *  Created on: Mar 6, 2023
 *      Author: saveasmtz
 */

#include <CAN/can_tasks_seb.h>

CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;

uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
uint32_t FreeMailBoxes;

/* USER CODE BEGIN Header_can_rx_task */
/**
  * @brief  Function implementing the canRxTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_can_rx_task */
void can_rx_task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  //osThreadSuspend(canRxTaskHandle);
	  //osSemaphoreAcquire(canRxSemHandle, osWaitForever);
	  if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
		  HAL_GPIO_TogglePin(GPIOA, DEBUG_2_Pin);
	  osDelay(50);
  }
  //osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_can_tx_task */
/**
* @brief Function implementing the canTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can_tx_task */
void can_tx_task(void *argument)
{
  /* USER CODE BEGIN can_tx_task */
  /* Infinite loop */
  for(;;)
  {
	  TxHeader.DLC = 1; // Length of data to send in bytes
		TxHeader.ExtId = 0; // For basic CAN protocol
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA; // transfering data of remote frame
		TxHeader.StdId = 0x600; // ID of this CAN peripheral
		TxHeader.TransmitGlobalTime = DISABLE;

		TxData[0] = 0x11;

		FreeMailBoxes = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &TxData[0], &TxMailbox) == HAL_OK) /*{

		  printf("Transmission requested 1\n");
				  if(!HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox))
					  printf("Message sent 1\n");
				  else
					  printf("Message pending 1\n");
			  } else
				  printf("Error 1\n");
		  */
		osDelay(50);
  }
  osThreadTerminate(NULL);
  /* USER CODE END can_tx_task */
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//osSemaphoreRelease(canRxSemHandle);
	  if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
		  HAL_GPIO_TogglePin(GPIOA, DEBUG_2_Pin);
	  osDelay(50);
}
