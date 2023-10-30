
#include <main.h>

void Setup_uart485_default(void)
{
//uart3
	g_u485_Rx_head = 0;
	g_u485_Rx_tail = 0;

	g_u485_Rx_now = 0;
	g_u485_Rx_prev = 0;
	g_u485_Rx_Flag = 0;

	memset(g_u485_Rx_buffer,0x00, sizeof(g_u485_Rx_buffer));
	memset(g_u485_Rx_Cmd,0x00, sizeof(g_u485_Rx_Cmd));
	memset(g_u485_Tx_Cmd,0x00, sizeof(g_u485_Tx_Cmd));

	HAL_GPIO_WritePin(RS485_nRE_GPIO_Port, RS485_nRE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);

}

void U3_485_Cmd_check(void)
{
	switch(g_u485_Rx_Cmd[0])
	{
	case 0x00:
		break;
	case 0x01:
		break;
	default:
		break;
	}

	U3_485_send_data();

	memset(g_u485_Rx_Cmd, 0x00, sizeof(g_u485_Rx_Cmd));
}

void U3_485_send_data(void)
{
	HAL_GPIO_WritePin(RS485_nRE_GPIO_Port, RS485_nRE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
	//HAL_Delay(1);
	HAL_UART_Transmit(&huart3, g_u485_Rx_Cmd, sizeof(g_u485_Rx_Cmd),100 );
	//HAL_Delay(1);
	HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RS485_nRE_GPIO_Port, RS485_nRE_Pin, GPIO_PIN_RESET);
	g_u485_Rx_Flag = 0;
}




