/*
 * can_task.h
 *
 *  Created on: Jan 9, 2022
 *      Author: abiel
 */

#ifndef INC_CAN_CAN_TASKS_H_
#define INC_CAN_CAN_TASKS_H_

#include "can_bus.h"
#include <stdint.h>
#include "cmsis_os.h"

#define HEARTBEAT_DELAY 500
#define DEBUG_MSG_DELAY 10
#define SBUS_DELAY 1000

static const uint32_t can_tx_task_delay = 5;

static const uint32_t can_rx_task_delay = 5;

/**
 * Updates the tx_data struct which is sent over CAN
 */
void can_init();

void can_tx_update();

void can_tx_task(void *);

void can_rx_update();

void can_rx_task(void *);

void start_can_tasks();

#endif /* INC_CAN_CAN_TASKS_H_ */
