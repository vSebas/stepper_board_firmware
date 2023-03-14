/*
 * can_bus_tasks.h
 *
 *  Created on: Mar 6, 2023
 *      Author: saveasmtz
 */

#ifndef INC_CAN_CAN_TASKS_SEB_H_
#define INC_CAN_CAN_TASKS_SEB_H_

#include "main.h"
#include "cmsis_os.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_RxHeaderTypeDef RxHeader;
extern CAN_TxHeaderTypeDef TxHeader;

extern uint8_t TxData[8];
extern uint8_t RxData[8];
extern uint32_t TxMailbox;
extern uint32_t FreeMailBoxes;

void can_rx_task(void *argument);
void can_tx_task(void *argument);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif /* INC_CAN_CAN_TASKS_SEB_H_ */
