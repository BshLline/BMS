/*
 * bms.c
 *
 *  Created on: Oct 26, 2023
 *      Author: Bae
 */


#include "main.h"

void BMS_Cell_Balancing(void)
{
	uint8_t i, j;
	uint16_t low_voltage = 0xFFFF;
	uint16_t sort_cell[2][7] = {0,}, temp_i, temp_voltage;
#if 0
	// Check Charging State
	//if(!IS_EX_CHARGING_RELAY_CHECK)
	if(!IS_EX_CHARGING_RELAY_CHECK)
	{
		return;
	}
#endif

	// All Off cell balancing
	//printf("discharge_state : ");
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

		sort_cell[1][i] = i;
		sort_cell[0][i] = Adbms6817.voltage_cell[i];
	}
	// Read ADC for cell voltage
	Adbms6817_read_adc();

	// sort for cell voltage
	for(i=0;i<NUM_OF_CELL;i++)
	{
		for(j=0;j<NUM_OF_CELL-1;j++)
		{
			if(sort_cell[0][j] < sort_cell[0][j+1])
			{
				temp_voltage = sort_cell[0][j];
				temp_i = sort_cell[1][j];

				sort_cell[0][j] = sort_cell[0][j+1];
				sort_cell[1][j] = sort_cell[1][j+1];

				sort_cell[0][j+1] = temp_voltage;
				sort_cell[1][j+1] = temp_i;
			}

		}

	}
	low_voltage = sort_cell[0][6];

	// Cell Balancing
	for(i=0;i<NUM_OF_CELL;i++)
	{
		if((sort_cell[0][i] >= 40000) && (sort_cell[0][i] - low_voltage) > VOLT_BALANCING_THRESHOLD)	// 1mV
		{
#ifdef USE_CELL_BALANCING_CONTINUOUSLY
			if(sort_cell[1][i] == 0)
			{
				if(Adbms6817.discharge_state[sort_cell[1][i] + 1] == 0)
					Adbms6817.discharge_state[sort_cell[1][i]] = TRUE;

			}
			if(sort_cell[1][i] == 6)
			{
				if(Adbms6817.discharge_state[sort_cell[1][i] - 1] == 0)
					Adbms6817.discharge_state[sort_cell[1][i]] = TRUE;

			}
			else
			{
				if((Adbms6817.discharge_state[sort_cell[1][i] - 1] == 0) && (Adbms6817.discharge_state[sort_cell[1][i] + 1] == 0))
					Adbms6817.discharge_state[sort_cell[1][i]] = TRUE;
			}
#endif
#ifdef USE_CELL_BALANCING_PWM
			Adbms6817.discharge_state[sort_cell[1][i]] = TRUE;
#endif
		}
	}
	for(i=0;i<NUM_OF_CELL;i++)
	{
		if(Adbms6817.discharge_state[i])
		{
#ifdef USE_CELL_BALANCING_CONTINUOUSLY
			Adbms6817_discharge_cell_continuously(i, TRUE);
#endif
#ifdef USE_CELL_BALANCING_PWM
			Adbms6817_discharge_cell_pwm(i, TRUE);
#endif
		}
	}

}

void Broadcast_data(void)
{
	uint8_t data[64], size=0, i;

	//broadcast_data.state_avg_voltage.state_avg_voltage = Adbms6817.voltage_avg;
	broadcast_data.AvgVoltage.data = Adbms6817.voltage_avg;
	broadcast_data.BatCurrent.data = g_Adc_ct_Amp;
	broadcast_data.Soc.data = (uint16_t)(Bms.Soc*100);
	for(i=0;i<8;i++)
	{
		broadcast_data.CellVoltage[i].data = Adbms6817.voltage_cell[i];
	}
	for(i=0;i<2;i++)
	{
		broadcast_data.MaxTemperature[i].data = Bms.MaxTemperature[i];
		broadcast_data.MinTemperature[i].data = Bms.MinTemperature[i];
		broadcast_data.Temperature[i].data = Bms.Temperature[i];
	}
	broadcast_data.CoulombCount.data = Bms.CoulumbCount;
	broadcast_data.status.data = Bms.Status.data;

#if 0
#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_BROADCAST)
	printf("size : %d\r\n", sizeof(broadcast_data));
#endif
	for(i=0;i<sizeof(broadcast_data);i++)
	{

		if(i%2 == 0)
		{
			data[i] = *(p + i + 1);
#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_BROADCAST)
			printf("%02X ", *(p + i + 1));
#endif
		}

		{
			data[i] = *(p + i - 1);
#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_BROADCAST)
			printf("%02X ", *(p + i + -1));
#endif
		}

		data[i] = *(p + i);
		printf("%02X ", *(p + i));

	}
	printf("\r\n");
#endif
	//printf("%X, %X, %X\r\n", broadcast_data.avg_voltage.voltage, broadcast_data.avg_voltage.byte.high, broadcast_data.avg_voltage.byte.low);

	size = 0;

	//Status 8 Byte
	data[size++] = 0xAA;	// HEAD
	data[size++] = 0x00;	// UART_TYPE_BMS_STATUS
	data[size++] = 0x01;	// BMS_OK
	data[size++] = 0x00;	// BMS_FAULT
	data[size++] = IS_EX_CHARGING_RELAY_CHECK;
	data[size++] = IS_MOTOR_RELAY_CHECK;
	//data[size++] = IS_PRE_CHARGE_CHECK;
	data[size++] = 0;
	data[size++] = IS_SMPS_RELAY_CHECK;

	//Temperature 16 Byte
	data[size++] = 0xAA;	// HEAD
	data[size++] = 0x02;	// UART_TYPE_BMS_TEMP
	data[size++] = broadcast_data.MaxTemperature[0].byte.high;
	data[size++] = broadcast_data.MaxTemperature[0].byte.low;
	data[size++] = broadcast_data.MaxTemperature[1].byte.high;
	data[size++] = broadcast_data.MaxTemperature[1].byte.low;
	data[size++] = broadcast_data.MinTemperature[0].byte.high;
	data[size++] = broadcast_data.MinTemperature[0].byte.low;
	data[size++] = broadcast_data.MinTemperature[1].byte.high;
	data[size++] = broadcast_data.MinTemperature[1].byte.low;
	data[size++] = broadcast_data.Temperature[0].byte.high;
	data[size++] = broadcast_data.Temperature[0].byte.low;
	data[size++] = broadcast_data.Temperature[1].byte.high;
	data[size++] = broadcast_data.Temperature[1].byte.low;
	data[size++] = 0;	// reserved
	data[size++] = 0;	// reserved

	//CellVoltage 16 Byte
	data[size++] = 0xAA;	// HEAD
	data[size++] = 0x04;	// UART_TYPE_BMS_VOLTAGE_P1
	for(i=0;i<7;i++)
	{
		data[size++] = broadcast_data.CellVoltage[i].byte.high;
		data[size++] = broadcast_data.CellVoltage[i].byte.low;
	}

#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_TEST_BSH)
#if 0
	printf("data size : %d\r\n");
	for(i=0;i<size;i++)
	{
		printf("%02X ", data[i]);
	}
	printf("\r\n");
#endif
#endif

	// Transmits
	HAL_UART_Transmit(&huart1, data, size, 100);
#if 0
	switch(Sw_info.g_uCom_sel)
	{
	case UART1_RS232:
		// tx
		break;
	case UART3_RS485:
		break;
	case CAN:
		break;
	}
#endif
}
