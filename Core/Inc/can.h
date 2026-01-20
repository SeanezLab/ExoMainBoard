/*
 * can.h
 *
 *  Created on: Jan 19, 2026
 *      Author: k.rodolfo
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct{
	uint8_t id;
	uint8_t data[6];
	CAN_RxHeaderTypeDef rx_header;
	CAN_FilterTypeDef filter;
}CANRxMessage ;

typedef struct{
	uint8_t id;
	uint8_t data[8];
	CAN_TxHeaderTypeDef tx_header;
}CANTxMessage ;

void can_rx_init(CANRxMessage *msg);
void can_tx_init(CANTxMessage *msg);
void can_pack_tx(float* tx_cmd);
void can_unpack_rx(float* rx_reply);


#ifdef __cplusplus
}
#endif

#endif /* INC_CAN_H_ */
