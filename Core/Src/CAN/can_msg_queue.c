/*
 * can.c
 *
 *  Created on: Jan 15, 2022
 *      Author: abiel
 */

#include <CAN/can_msg_queue.h>
#include "stm32l4xx_hal.h"
#include <string.h>

extern CAN_HandleTypeDef hcan2;

osStatus_t handle_debug_msg_queue(){
	uint8_t buf[8];
	uint8_t msg_buf[8];
	uint32_t txMailbox;

	osStatus_t ret = osMessageQueueGet(debugMessageQueue, msg_buf, NULL, 0);
	if(ret != osOK) return ret;

	CAN_TxHeaderTypeDef txHeader;
	txHeader.IDE = CAN_ID_STD;
	txHeader.StdId = 0x111;
	txHeader.RTR = CAN_RTR_DATA;

	txHeader.DLC = msg_buf[0] + 1;
	memcpy(buf + 1, msg_buf + 1, txHeader.DLC);
	buf[0] = DEBUG_LOG_ID;

	HAL_CAN_AddTxMessage(&hcan2, &txHeader, buf, &txMailbox);
	return ret;
}

static char debug_buf[8];
static uint8_t debug_buf_len = 0;
void send_can_debug_char(char c){
	debug_buf[debug_buf_len] = c;
	debug_buf_len++;
	if(debug_buf_len >= sizeof(debug_buf) || c == '\n'){
		//Queue buffer
		send_can_debug_msg(debug_buf, debug_buf_len);
		debug_buf_len = 0;
	}
}

void send_can_debug_msg(const char *str, uint32_t len){
	uint8_t buf[8];
	int i;

	for(i = 0; i < len - len % 7; i += 7){
		memcpy(buf + 1, str + i, 7);
		buf[0] = 7;
		osMessageQueuePut(debugMessageQueue, buf, 1, 10);
	}

	//Put remaining bytes into queue
	memcpy(buf + 1, str + i, len % 7);
	buf[0] = len % 7;
	osMessageQueuePut(debugMessageQueue, buf, 1, 10);
}

void queue_can_msg_byte(uint8_t id, uint8_t data){
	CAN_TX_QUEUE_OBJ out;
	out.msg_size = can_pack_byte(id, data, out.buf, sizeof(out.buf));
	osStatus_t ret = osMessageQueuePut(txMessageQueue, &out, 0, 10);
}

void queue_can_msg_long(uint8_t id, uint32_t data){
	CAN_TX_QUEUE_OBJ out;
	out.msg_size = can_pack_long(id, data, out.buf, sizeof(out.buf));
	osMessageQueuePut(txMessageQueue, &out, 0, 10);
}

void queue_can_msg_short(uint8_t id, uint16_t data){
	CAN_TX_QUEUE_OBJ out;
	out.msg_size = can_pack_short(id, data, out.buf, sizeof(out.buf));
	osMessageQueuePut(txMessageQueue, &out, 0, 10);
}

void queue_can_msg_float(uint8_t id, float data){
	CAN_TX_QUEUE_OBJ out;
	out.msg_size = can_pack_float(id, data, out.buf, sizeof(out.buf));
	osMessageQueuePut(txMessageQueue, &out, 0, 10);
}
