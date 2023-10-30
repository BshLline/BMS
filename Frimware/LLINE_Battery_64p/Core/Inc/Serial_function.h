
#ifndef SERIAL_FUNCTION_H
#define SERIAL_FUNCTION_H

#include "stdio.h"
#include "stdint.h"
#include "my_define.h"

//Usart1
#define UART_TX_SIZE 		20
#define UART_RX_CMD_SIZRE	256
#define UART_RX_SIZE 		256

//for uart1
////Serial Data
uint8_t g_uUart1_Rx_head;
uint8_t g_uUart1_Rx_tail;
uint8_t g_uUart1_Rx_now;
uint8_t g_uUart1_Rx_prev;
uint8_t g_uUart1_Rx_Flag;
uint8_t g_uUart1_Rx_buffer[UART_RX_SIZE];
uint8_t g_uUart1_Rx_Cmd[UART_RX_CMD_SIZRE];
uint8_t g_uUart1_Tx_Cmd[UART_TX_SIZE];

//debug port
uint8_t g_uUart2_Rx_head;
uint8_t g_uUart2_Rx_tail;
uint8_t g_uUart2_Rx_now;
uint8_t g_uUart2_Rx_prev;
uint8_t g_uUart2_Rx_Flag;
uint8_t g_uUart2_Rx_buffer[UART_RX_SIZE];
uint8_t g_uUart2_Rx_Cmd[UART_RX_CMD_SIZRE];
uint8_t g_uUart2_Tx_Cmd[UART_TX_SIZE];

uint8_t g_uPrint_state;	//serial print status

#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_TEST_BSH)
uint8_t g_uDebug_Rx_head;
uint8_t g_uDebug_Rx_tail;
uint8_t g_uDebug_Rx_buffer[UART_RX_SIZE];
uint8_t g_uDebug_Rx_Cmd[UART_RX_CMD_SIZRE];
uint8_t g_uDebug_Tx_Cmd[UART_TX_SIZE];
#endif

void Setup_uart_default(void);
void Uart1_Cmd_check(void);
void Uart2_Cmd_check(void);	//Debug port

void Print_cell_info(void);

#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_TEST_BSH)
void Check_Debug_Serial(void);
#endif

#endif
