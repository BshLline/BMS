
#ifndef CAN_FUNCTION_H
#define CAN_FUNCTION_H

#include "can.h"

//#define TARGET_DEVICE

#define	TARGET_DEVICE_ID 		0x07F0	// == Main 컨트롤러  == 보낼곳 ID
#define	TARGET_DEVICE_ID_MASK 	0x07FF

#define	MY_DEVICE_ID 			0x07A0	//MY ID == BMS ID
#define	MY_DEVICE_ID_MASK 		0x07FF

void Set_Can_filter(uint8_t Option_id);
void Can_setup_default(uint8_t Option_id);
void Send_Can_Message(void);
void Can_Cmd_check(void);

////*Can 통신용
CAN_TxHeaderTypeDef   	g_Can_TxHeader;
uint8_t               	g_uCan_TxData[8];
uint32_t              	g_uCan_TxMailbox;

CAN_FilterTypeDef  		sFilterConfig;   // ?��?�� ?��?�� 구조�??????????? �????????????��
CAN_RxHeaderTypeDef   	g_Can_RxHeader;
uint8_t               	g_uCan_RxData[8];

uint8_t               	g_uCan_Rxflag;
uint8_t               	g_uCan_RxCmd;



#endif
