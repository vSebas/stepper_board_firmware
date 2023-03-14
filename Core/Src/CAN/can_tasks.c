/*
 * can_task.c
 *
 *  Created on: Jan 9, 2022
 *      Author: abiel
 */

#include <CAN/can_msg_queue.h>
#include <string.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "CAN/can_tasks.h"
#include "CAN/can_bus_parser.h"

#include "Vanttec_CANLib/CANMessage.h"
//#include "SBUS/sbus.h"

extern CAN_HandleTypeDef hcan1;
//extern SBUS_Data sbusData;

osMessageQueueId_t debugMessageQueue;
osMessageQueueId_t txMessageQueue;

volatile uint8_t g_sendPing;

const osThreadAttr_t canRxTask_attributes = {
  .name = "canRxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t canTxTask_attributes = {
  .name = "canTxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

static const osThreadAttr_t hbTask_attributes = {
  .name = "hbTxCan",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


static const osThreadAttr_t pingTask_attributes = {
  .name = "pingMsgTx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static void can_hb_tx_task(void *param){
	for(;;){
		queue_can_msg_short(HEARTBEAT_ID, HAL_GetTick());
		osDelay(HEARTBEAT_DELAY);
	}
}

/*
static const osThreadAttr_t debugVarsTask_attributes = {
	.name = "debugVars",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};

static const osThreadAttr_t sbusTask_attributes = {
  .name = "sbusTxCan",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static const osThreadAttr_t batteryTask_attributes = {
  .name = "batteryMonitorTx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static const osThreadAttr_t debugMsgTask_attributes = {
  .name = "debugMsgTx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
*/


void can_init(){
	CAN_FilterTypeDef filter;
	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterBank = 0;		// Specify filter bank to use
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; //Incoming data is saved here
	filter.FilterIdHigh = 0;
	filter.FilterIdLow = 0;
	filter.FilterMaskIdHigh= 0;
	filter.FilterMaskIdLow = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = 0;

	if(HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) Error_Handler();

	if(HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();

	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

	//Initialize queues
	//First char -> size
	debugMessageQueue = osMessageQueueNew(64, 8, NULL);
	if(debugMessageQueue == NULL)
		Error_Handler();

	txMessageQueue = osMessageQueueNew(64, sizeof(CAN_TX_QUEUE_OBJ), NULL);
	if(txMessageQueue == NULL)
		Error_Handler();
}

void can_tx_task(void * params){
	for(;;){
		can_tx_update();

		osDelay(can_tx_task_delay);
	}
}

void can_tx_update(){
	CAN_TxHeaderTypeDef txHeader;
	uint32_t txMailbox;
	txHeader.IDE = CAN_ID_STD;
	txHeader.StdId = 0x111;
	txHeader.RTR = CAN_RTR_DATA;

	CAN_TX_QUEUE_OBJ txOut;

	while(osMessageQueueGet(txMessageQueue, &txOut, NULL, 0) == osOK){
		txHeader.DLC = txOut.msg_size;
		HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&hcan1, &txHeader, txOut.buf, &txMailbox);
		if(ret != HAL_OK){
			txHeader.DLC++;
		}
	}
}

void can_rx_task(void *params){
	for(;;){
		can_rx_update();
		osDelay(can_rx_task_delay);
	}
}

void can_rx_update(){
	//TODO check both FIFO?
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t buf[8];
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0){
		HAL_StatusTypeDef ret = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, buf);
		if(ret != HAL_OK) continue;
		//Parse can message
		can_parse_msg(&rxHeader, buf);
	}
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1) != 0){
			HAL_StatusTypeDef ret = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &rxHeader, buf);
			if(ret != HAL_OK) continue;
			//Parse can message
			can_parse_msg(&rxHeader, buf);
		}
}

static void can_send_ping_task(void * param){
	for(;;){
		if(g_sendPing == 1){
			queue_can_msg_byte(PING_ID, 0);
			g_sendPing = 0;
		}

		osDelay(1000);
	}
}

/*
extern SBUS_Data sbusData;
static void can_sbus_tx_task(void * param){
	for(;;){
		for(uint8_t i = 0; i < 16; i++){
			queue_can_msg_short(SBUS_ID_START + i, sbusData.channels[i]);
		}

		queue_can_msg_byte(SBUS_FAILSAFE, sbusData.failsafe);

		osDelay(SBUS_DELAY);
	}
}

static void can_debug_msg_tx_task(void *param){
	for(;;){
		while(handle_debug_msg_queue() == osOK);
		osDelay(DEBUG_MSG_DELAY);
	}
}

static void can_battery_monitor_task(void * param){
	battery_monitor_init();
	for(;;){
		float voltage = battery_monitor_poll();
		queue_can_msg_float(BATTERY_VOLTAGE_ID, voltage);
		osDelay(1000);
	}
}
*/

void start_can_tasks(){
	osThreadNew(can_hb_tx_task, NULL, &hbTask_attributes);
	//osThreadNew(can_sbus_tx_task, NULL, &sbusTask_attributes);
	//osThreadNew(can_debug_msg_tx_task, NULL, &debugMsgTask_attributes);
	osThreadNew(can_send_ping_task, NULL, &pingTask_attributes);
	//osThreadNew(can_battery_monitor_task, NULL, &batteryTask_attributes);
	//osThreadNew(debug_vars_task, NULL, &debugVarsTask_attributes);
	osThreadNew(can_rx_task, NULL, &canRxTask_attributes);
	osThreadNew(can_tx_task, NULL, &canTxTask_attributes);
}

