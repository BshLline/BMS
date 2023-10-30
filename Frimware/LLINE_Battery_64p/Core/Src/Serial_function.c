
#include "main.h"

void Setup_uart_default(void)
{
	g_uPrint_state = 0;
//uart1
	g_uUart1_Rx_head = 0;
	g_uUart1_Rx_tail = 0;
	g_uUart1_Rx_now = 0;
	g_uUart1_Rx_prev = 0;
	g_uUart1_Rx_Flag = 0;
	memset(g_uUart1_Rx_buffer,0x00, sizeof(g_uUart1_Rx_buffer));
	memset(g_uUart1_Rx_Cmd,0x00, sizeof(g_uUart1_Rx_Cmd));
	memset(g_uUart1_Tx_Cmd,0x00, sizeof(g_uUart1_Tx_Cmd));
//uart2
	g_uUart2_Rx_head = 0;
	g_uUart2_Rx_tail = 0;
	g_uUart2_Rx_now = 0;
	g_uUart2_Rx_prev = 0;
	g_uUart2_Rx_Flag = 0;
	memset(g_uUart2_Rx_buffer,0x00, sizeof(g_uUart2_Rx_buffer));
	memset(g_uUart2_Rx_Cmd,0x00, sizeof(g_uUart2_Rx_Cmd));
	memset(g_uUart2_Tx_Cmd,0x00, sizeof(g_uUart2_Tx_Cmd));
}

void Uart1_Cmd_check(void)
{
	static uint8_t flag_cmd;

	while(g_uUart1_Rx_head != g_uUart1_Rx_tail)
	{
		if(flag_cmd)
		{
			flag_cmd = false;
			switch(g_uUart1_Rx_buffer[g_uUart1_Rx_tail++])
			{
			case 0x02:	// Ex-Charge Relay
				//if(IS_EX_CHARGING_RELAY_CHECK)
				if(TIM3->CCR2)
				{
					PWM_Toggle_Relay(PWM_EX_CHARGE, OFF);
					TIM3->CCR2 = 0;
				}
				else
					PWM_Toggle_Relay(PWM_EX_CHARGE, ON);

				#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_TEST_BSH)
				printf("Ex-Charge Relay Cmd\r\n");
				#endif
				break;
			case 0x03:	// SMPS Relay
				//if(IS_SMPS_RELAY_CHECK)
				if(TIM3->CCR3)
				{
					PWM_Toggle_Relay(PWM_SMPS_CHARGE, OFF);
					TIM3->CCR3 = 0;
				}
				else
					PWM_Toggle_Relay(PWM_SMPS_CHARGE, ON);

				#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_TEST_BSH)
				printf("SMPS Relay Cmd\r\n");
				#endif
				break;
			case 0x05:	// Motor Relay
				//if(IS_MOTOR_RELAY_CHECK)
				if(TIM3->CCR1)
				{
					TIM3->CCR1 = 0;
					PWM_Toggle_Relay(PWM_MOTOR, OFF);
				}
				else
					PWM_Toggle_Relay(PWM_MOTOR, ON);

				#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_TEST_BSH)
				printf("Motor Relay Cmd\r\n");
				#endif
				break;
			case 0x06:
				TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = 0;
				PWM_Toggle_Relay(PWM_EX_CHARGE, OFF);
				PWM_Toggle_Relay(PWM_SMPS_CHARGE, OFF);
				PWM_Toggle_Relay(PWM_MOTOR, OFF);
				break;
			case 0x07:
				TIM3->CCR4 = 100;
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
				break;
			}
			break;
		}
		else
		{
			if(g_uUart1_Rx_buffer[g_uUart1_Rx_tail++] == 0x55)
			{
				flag_cmd = true;
			}
		}
	}

#if 0
	switch(g_uUart1_Rx_Cmd[0])
	{
	case 0x00:
		break;
	case 0x01:
		break;
	default:
		break;
	}
	HAL_UART_Transmit(&huart1, g_uUart1_Rx_Cmd, sizeof(g_uUart1_Rx_Cmd),100 );
	g_uUart1_Rx_Flag = 0;
	memset(g_uUart1_Rx_Cmd, 0x00, sizeof(g_uUart1_Rx_Cmd));
#endif
#if 0
	switch(g_uUart1_Rx_Cmd[0])
	{
	case 0x55:	// Relay Toggle Command
		switch(g_uUart1_Rx_Cmd[1])
		{
		case 0x02:	// Top Relay
			if(IS_EX_CHARGING_RELAY_CHECK)
				PWM_Toggle_Relay(PWM_EX_CHARGE, OFF);
			else
				PWM_Toggle_Relay(PWM_EX_CHARGE, ON);
			break;
		case 0x03:	// SMPS Relay
			if(IS_SMPS_RELAY_CHECK)
				PWM_Toggle_Relay(PWM_SMPS_CHARGE, OFF);
			else
				PWM_Toggle_Relay(PWM_SMPS_CHARGE, ON);
			break;
		case 0x05:	// Motor Relay
			if(IS_MOTOR_RELAY_CHECK)
				PWM_Toggle_Relay(PWM_MOTOR, OFF);
			else
				PWM_Toggle_Relay(PWM_MOTOR, ON);
			break;
		case 0x06:	// test
			printf("u1 rx test\r\n");
			HAL_UART_Transmit(&huart1, "u1 rx test\r\n", sizeof("u1 rx test\r\n"),100 );
			break;
		}
		break;
	}
	g_uUart1_Rx_Flag = 0;
	memset(g_uUart1_Rx_Cmd, 0x00, sizeof(g_uUart1_Rx_Cmd));
#endif
}

void Uart2_Cmd_check(void)
{
	switch(g_uUart2_Rx_Cmd[0])
	{
	case 0x53:	//'S'	//sell balance
		Adbms6817_discharge_cell_pwm(g_uUart2_Rx_Cmd[1],	g_uUart2_Rx_Cmd[2]);
		break;
	case 0x50:	//'P'	//print info
		if(g_uUart2_Rx_Cmd[1] == '1') 	g_uPrint_state = 1;
		else						g_uPrint_state = 0;
		break;
	case 0x43:	//'C'
		if(g_uUart2_Rx_Cmd[1] == 0x30)
		{
			Charge_voltage_open(0);//off
			//printf("Charge_voltage off!\r\n");
		}
		else if(g_uUart2_Rx_Cmd[1] == 0x31)
		{
			Charge_voltage_open(1);//on
			//printf("Charge_voltage on!\r\n");
		}

		break;
	case 0x44:	//'D'
		if(g_uUart2_Rx_Cmd[1] == 0x30)
		{
			Discharge_voltage_open(0);//off
			//printf("Discharge_voltage off!\r\n");
		}
		else if(g_uUart2_Rx_Cmd[1] == 0x31)
		{
			Discharge_voltage_open(1);//on
			//printf("Discharge_voltage on!\r\n");
		}
		break;
	default:
		break;
	}

	g_uUart2_Rx_Flag = 0;
	memset(g_uUart2_Rx_Cmd, 0x00, sizeof(g_uUart2_Rx_Cmd));
}

void Print_cell_info()
{
	if(g_uPrint_state == 1)
	{
		//display menu
		printf("voltage_0, v_1, v_2, v_3, v_4, v_5, v_6,");
		printf("Balance_0, b_1, b_2, b_3, b_4, b_5, b_6,");
		printf("\r\n");
		g_uPrint_state = 2;
	}
	else
	{
		for(int i=0;i<7;i++)printf("%d,",Adbms6817.voltage_cell[i]);
		for(int i=0;i<7;i++)printf("%d,",Adbms6817.discharge_state[i]);
		printf("\r\n");

	}
	//float temp = 0;
}

#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_TEST_BSH)
void Check_Debug_Serial(void)
{
	int8_t ch;
	uint8_t Sn;
	uint8_t data[NUM_OF_ADBMS6817][8], i;
	uint8_t flash_data[1024];

	while(1)
	{
		if(g_uDebug_Rx_head == g_uDebug_Rx_tail)
			break;

		ch = (char)g_uDebug_Rx_buffer[g_uDebug_Rx_tail++];
		Sn = ch - 0x30;

		switch(ch)
		{
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
			//Adbms6817_rdcfg(NUM_OF_ADBMS6817,REG_GRP_B,data);
			if(Adbms6817.discharge_state[Sn] == TRUE)
			{
#ifdef USE_CELL_BALANCING_CONTINUOUSLY
				Adbms6817_discharge_cell_continuously(Sn, FALSE);
#else
				Adbms6817_discharge_cell_pwm(Sn, FALSE);
#endif
				Adbms6817.discharge_state[Sn] = FALSE;
				//Adbms6817_discharge_cell(ch, FALSE);
			}
			else
			{
#ifdef USE_CELL_BALANCING_CONTINUOUSLY
				Adbms6817_discharge_cell_continuously(Sn, TRUE);
#else
				Adbms6817_discharge_cell_pwm(Sn, TRUE);
#endif
				Adbms6817.discharge_state[Sn] = TRUE;
			}

			Adbms6817_rdcfg(NUM_OF_ADBMS6817,REG_GRP_B,data);

			printf("Adbms6817_rdcfg : ");
			for(i=0;i<7;i++)
			{
				printf("%02X ",data[0][i]);
			}
			printf("\r\n");

			break;
		case 'b':
			if(Bms.EnableBalancing)
			{
				printf("Cell Balancing Disable\r\n");
				Bms.EnableBalancing = FALSE;
				for(i=0;i<NUM_OF_CELL;i++)
				{
					if(Adbms6817.discharge_state[i])
					{
			#ifdef USE_CELL_BALANCING_CONTINUOUSLY
						Adbms6817_discharge_cell_continuously(i, FALSE);
			#endif
			#ifdef USE_CELL_BALANCING_PWM
						Adbms6817_discharge_cell_pwm(i, FALSE);
			#endif
						Adbms6817.discharge_state[i] = FALSE;
					}
				}
			}
			else
			{
				printf("Cell Balancing Enable\r\n");
				Bms.EnableBalancing = TRUE;
			}
			break;
		case 'f':
			flash_read(0, flash_data, 1024);
#if 0
			flash_data[0] = 0x0A;
			flash_data[1] = 0x11;
			flash_data[2] = 0x43;
			flash_data[3] = 0x0F;
			flash_data[4] = 0x6A;
			flash_data[5] = 0xB0;
#endif
			flash_data[0] = 'E';
			flash_data[1] = 'N';
			flash_data[2] = 'D';
			flash_data[3] = '_';
			flash_data[4] = 'O';
			flash_data[5] = 'F';
			flash_data[6] = '_';
			flash_data[7] = 'F';
			flash_data[8] = 'L';
			flash_data[9] = 'A';
			flash_data[10] = 'S';
			flash_data[11] = 'H';
			flash_data[12] = '_';
			flash_data[13] = 'M';
			flash_data[14] = 'E';
			flash_data[15] = 'M';

			flash_write(0x03F0, flash_data, 16);
			printf("write done\r\n");
			flash_read(0, flash_data, 1024);
			break;
		case 'r':
			flash_read(0x03F0, flash_data, 16);
			printf("flash_data : ");
			for(int i=0;i<16;i++)
			{
				printf("%02X ",flash_data[i]);
			}
			printf("\r\n");

			break;
		case 's':
			//Adbms6817_rdcfg(NUM_OF_ADBMS6817,REG_GRP_B,data);
			// update discharge_state
			for(i=0;i<7;i++)
			{
				if(data[0][4] & (0x1 << i))
					Adbms6817.discharge_state[i] = TRUE;
				else
					Adbms6817.discharge_state[i] = FALSE;
			}
			if(data[0][5] & 0x1)
			{
				Adbms6817.discharge_state[8] = TRUE;
			}
			else
			{
				Adbms6817.discharge_state[8] = FALSE;
			}
			for(i=0;i<8;i++)
			{
				printf("%02X ",data[0][i]);
			}
			printf("\r\n");
			break;
		case 'S':
			for(i=0;i<9;i++)
			{
				printf("%X ",Adbms6817.discharge_state[i]);
			}
			printf("\r\n");
			break;


		}

	}
}
#endif
