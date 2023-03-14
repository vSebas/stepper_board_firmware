/*
 * can_bus_parser.h
 *
 *  Created on: Jan 29, 2022
 *      Author: abiel
 */

#ifndef INC_CAN_CAN_BUS_PARSER_H_
#define INC_CAN_CAN_BUS_PARSER_H_

#include "stm32l4xx_hal.h"

void can_parse_msg(CAN_RxHeaderTypeDef *header, uint8_t *data);

#endif /* INC_CAN_CAN_BUS_PARSER_H_ */
