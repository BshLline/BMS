
#ifndef SERIAL485_FUNCTION_H
#define SERIAL485_FUNCTION_H

#include "stdio.h"
#include "stdint.h"

#define UART485_TX_SIZE 		20
#define UART485_RX_CMD_SIZRE	256
#define UART485_RX_SIZE 		256

//for uart3
////Serial Data
uint8_t g_u485_Rx_head;
uint8_t g_u485_Rx_tail;
uint8_t g_u485_Rx_now;
uint8_t g_u485_Rx_prev;
uint8_t g_u485_Rx_Flag;
uint8_t g_u485_Rx_buffer[UART485_RX_SIZE];
uint8_t g_u485_Rx_Cmd[UART485_RX_CMD_SIZRE];
uint8_t g_u485_Tx_Cmd[UART485_TX_SIZE];

struct ModBusRTU
{
	uint8_t uDev_id;		//0x10	//+switch value
	uint8_t uFunc_code;	//0x01	//read register
	uint8_t uData_len;
	uint8_t uData;
	uint8_t uCrc_low;
	uint8_t uCrc_High;
}Modbus;

void Setup_uart485_default(void);
void U3_485_Cmd_check(void);
void U3_485_send_data(void );

#endif
