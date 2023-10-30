
#include "main.h"

void Set_Can_filter(uint8_t uOption_id)
{
	uint32_t uFilter_id = 0;
	uint32_t uFilter_mask=0;

#ifdef TARGET_DEVICE
	uFilter_id = MY_DEVICE_ID+uOption_id;
	uFilter_mask = MY_DEVICE_ID_MASK;		//#define	CAR_PC_MASK 	0x07F0;	//0x07F0~0x07FF 까지 필터링됨?
#else	//컨트롤러에서 받을때는....
	uFilter_id = TARGET_DEVICE_ID;
	uFilter_mask = TARGET_DEVICE_ID_MASK;
#endif

	/* CAN Filter */
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	//sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdHigh = uFilter_id<<5;
	sFilterConfig.FilterIdLow = 0x0000;
	//sFilterConfig.FilterMaskIdHigh = 0x0000;              	// 0x00000000 	= 모든 ID 받기
	sFilterConfig.FilterMaskIdHigh = uFilter_mask<<5;      // 0xFFFFFFFF	= 모든 설정 ID만 받겠다.
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
	  /* Filter configuration Error */
	  Error_Handler();
	}
}

void Can_setup_default(uint8_t uOption_id)
{

/* Configure Transmission process */			//Tx
#ifdef TARGET_DEVICE
	g_Can_TxHeader.StdId = TARGET_DEVICE_ID;         	// Standard Identifier, 0 ~ 0x7FF
#else	//컨트롤러에서 받을때는....
	g_Can_TxHeader.StdId = MY_DEVICE_ID + uOption_id;    // Standard Identifier, 0 ~ 0x7A0 + Option_id
#endif
	g_Can_TxHeader.ExtId = 0x01;                 	 	// Extended Identifier, 0 ~ 0x1FFFFFFF
	g_Can_TxHeader.RTR = CAN_RTR_DATA;
	g_Can_TxHeader.IDE = CAN_ID_STD;
	g_Can_TxHeader.DLC = 8;
	g_Can_TxHeader.TransmitGlobalTime = DISABLE;

	Set_Can_filter(uOption_id);

	/* Can Start */
	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_CAN)
	//HAL_UART_Transmit(&huart1, "Can Setup!\r\n", 12, 100);
	printf("Can Setup!\r\n");
#endif
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}
#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_CAN)
	//HAL_UART_Transmit(&huart1, "Can Ready!!\r\n", 13, 100);
	printf("Can Ready!!\r\n");
#endif
}

//void Send_Can_Message(void)
void Send_Can_Message(void)
{
#ifdef MAIN_PC
	uCan_TxData[0] = 0x07;
	uCan_TxData[1] = 0x01;
	uCan_TxData[2] = 0x01;
	uCan_TxData[3] = 0x02;
	uCan_TxData[4] = 0x01;
	uCan_TxData[5] = 0x03;
	uCan_TxData[6] = 0x01;
	uCan_TxData[7] = 0x04;
#else
	/* Set the data to be transmitted */
	//Single frame
	//Can_TxData[0] //Up 4bit = 0 일때 	//single frame	//Low 4bit = data len
	//Multi frame	//Up 4bit = 1		//multi frame	//reserved
	g_uCan_TxData[0] = 0x07;
	g_uCan_TxData[1] = 0x32;
	g_uCan_TxData[2] = 0x33;
	g_uCan_TxData[3] = 0x34;
	g_uCan_TxData[4] = 0x35;
	g_uCan_TxData[5] = 0x36;
	g_uCan_TxData[6] = 0x37;
	g_uCan_TxData[7] = 0x38;
#endif

	/* Start the Transmission process */

	if (HAL_CAN_AddTxMessage(&hcan, &g_Can_TxHeader, g_uCan_TxData, &g_uCan_TxMailbox) != HAL_OK)
	{
		//HAL_UART_Transmit(&huart1, "\r\n\r\nCan Send Fail\r\n", 19, 100);
		printf("\r\n\r\nCan Send Fail\r\n");
	  	//Error_Handler();

		MX_CAN_Init(); // CAN 초기화를 합니다
		Set_Can_filter(Sw_info.g_uOption_id);  // 수신필터를 설정하는 함수입니다.
		HAL_CAN_Start(&hcan); // CAN 통신을 시작합니다
		HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 인터럽트 함수
	}
	else
	{
#ifndef MAIN_PC
//		printf("%04lx : %d %d %d %d %d %d %d %d \r\n", TxHeader.StdId,TxData[0],TxData[1],TxData[2],TxData[3],TxData[4],TxData[5],TxData[6],TxData[7]);
//		printf("%04lx : %d %d %d %d\r\n",TxHeader.StdId,hvac.Adc_ac_temp,hvac.Adc_heat_temp,hvac.Adc_direction_feed, hvac.Adc_speed_feed);
#endif
	}
}

void Can_Cmd_check(void)
{
	uint16_t uData[4];

	switch(g_uCan_RxData[0])
	{
	case 0x01:
		break;
	case 0x02:
		break;
	case 0x03:
		break;
	case 0x04:
		break;
	default:
		break;
	}
	uData[0] = (g_uCan_RxData[0]<<8) + g_uCan_RxData[1];
	uData[1] = (g_uCan_RxData[2]<<8) + g_uCan_RxData[3];
	uData[2] = (g_uCan_RxData[4]<<8) + g_uCan_RxData[5];
	uData[3] = (g_uCan_RxData[6]<<8) + g_uCan_RxData[7];

#ifdef TARGET_DEVICE
	//printf("PC Recv:");
	HAL_UART_Transmit(&huart1, "PC Recv:\r\n", 10, 100);
#else
	printf("Bms Recv:");
	//HAL_UART_Transmit(&huart1, "Bms Recv:\r\n", 11, 100);
#endif

	//printf("%04lx : %d     %d     %d     %d \r\n", RxHeader.StdId,data[0],data[1],data[2],data[3]);
	sprintf((char *)g_uSerial_tx_buffer, "%04lx : %d     %d     %d     %d \r\n", g_Can_RxHeader.StdId,uData[0],uData[1],uData[2],uData[3]);
	HAL_UART_Transmit(&huart1, g_uSerial_tx_buffer, strlen((char *)g_uSerial_tx_buffer), 100);
	g_uCan_Rxflag = 0;
}



