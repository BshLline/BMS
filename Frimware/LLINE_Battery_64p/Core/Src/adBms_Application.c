/*******************************************************************************
Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc.
and its licensor.
 ******************************************************************************
 * @file:    adbms_Application.c
 * @brief:   adbms application test cases function
 * @version: $Revision$
 * @date:    $Date$
 * Developed by: ADIBMS Software team, Bangalore, India
 *****************************************************************************/
/*! \addtogroup APPLICATION
 *  @{
 */

/*! @addtogroup Application
 *  @{
 */
#include "main.h"

extern uint8_t boot_time_sec, boot_time_min, boot_time_hour;

// admbs6817 cell0 ~ cell7
void Print_Cell_Voltage(void)
{
#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_ADBMS6817)
	uint8_t Cell_Num;
	float voltage = 0;
	float voltage_total = 0;
	float voltage_avg = 0;

	printf("%d:%02d:%02d    ", boot_time_hour, boot_time_min, boot_time_sec);
	for(Cell_Num=1;Cell_Num<8;Cell_Num++)
	{
		voltage = Adbms6817.voltage_cell[Cell_Num-1]/10000.;
		voltage_total += voltage;
#if 0
		if(Cell_Num <= 5)	// CELL 1 ~ 5
		{
			if(Adbms6817.discharge_state[Cell_Num-1])
			{
				printf("*");
			}

		}
		else if(Cell_Num == 7)
		{
			if(Adbms6817.discharge_state[5])
			{
				printf("*");
			}
		}
		else if(Cell_Num == 8)
		{
			if(Adbms6817.discharge_state[8])
			{
				printf("*");
			}
		}

		if(Adbms6817.discharge_state[Cell_Num-1])
		{
			printf("*");
		}
#endif
		//printf("Cell %d : %.3f Volt\r\n",Cell_Num,voltage);
		if(Adbms6817.discharge_state[Cell_Num-1])
		{
			printf("*");
		}

		printf("%d %.3f    ",Cell_Num,voltage);
	}
	//printf("\r\n");
	voltage_avg = voltage_total / 7;

	// temp
	//printf("\r\n");
	//printf("Mean : %d'C %d'C, Max : %d'C %d'C, Min : %d'C %d'C  %dA\r\n", Bms.Temperature[0], Bms.Temperature[1], Bms.MaxTemperature[0], Bms.MaxTemperature[1], Bms.MinTemperature[0], Bms.MinTemperature[1], g_uAdc_ct_Amp);
	//printf("Cur Ct(0.1A): %d\r\n", g_uAdc_ct_Amp);
#if 0
	printf("Discharge Cell : ");
	for(i=0;i<NUM_OF_CELL;i++)
	{
		if(Adbms6817.discharge_state[i])
		{
			printf("%d ", i+1);
		}
	}
	printf("\r\n");
#endif
	printf("Avg:%.2fV   %.2f%% \r\n", voltage_avg, Bms.Soc);

#if 0
	printf("\r\n");
	for(Cell_Num=1;Cell_Num<9;Cell_Num++)
	{
		voltage = Adbms6817.diagnostic_cell[Cell_Num-1]/5000.;
		//printf("Cell %d : %.3f Volt\r\n",Cell_Num,voltage);
		printf("%d %.3f    ",Cell_Num,voltage);
	}
#endif


	#if 0
	printf("voltage_avg : %.3f Volt\r\n", voltage_avg);
	printf("Mean : %d'C %d'C, Max : %d'C %d'C, Min : %d'C %d'C  %dA\r\n", Bms.Temperature[0], Bms.Temperature[1], Bms.MaxTemperature[0], Bms.MaxTemperature[1], Bms.MinTemperature[0], Bms.MinTemperature[1], g_uAdc_ct_Amp);

	Adbms6817_rdcfg(NUM_OF_ADBMS6817,REG_GRP_B,data);

	printf("S: S0  S1  S2  S3  S4  S5  S6  S7\r\n");
	for(i = 0;i<8;i++)
	{
		if(data[0][4] & 0x1<<i)
			printf("   1");
		else
			printf("   0");
		//printf("   %d", (data[0][4] & 0x1<<i));
	}
	printf("\r\n");
	#endif
	//printf("\r\n");
#endif
}

// adbms6817 gpio1 and gpio1
void Print_gpio_temp(void)
{
#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_ADBMS6817)
	uint8_t i;
	//float temp = 0;
	for(i=0;i<2;i++)
	{
		printf("Gpio %d : %d",i+1, Adbms6817.temp_gpio[i]);
		printf("\r\n");
	}
	printf("\r\n");
#endif
}

void Print_6817_info(void)
{
#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_ADBMS6817)
	uint8_t i;
	//float temp = 0;

	//uint8_t	discharge_state[7];
	//uint16_t voltage_cell[7];	// ADBMS6817 100uV/bit
	//uint16_t temp_gpio[2];

	for(i=0;i<2;i++)
	{
		printf("Gpio %d : %d",i+1, Adbms6817.temp_gpio[i]);
		printf("\r\n");
	}
	printf("\r\n");
#endif
}

void Adbms6817_initialize()
{
	/*
	memset(Adbms6817.discharge_state,0x00,sizeof(Adbms6817.discharge_state));
	memset(Adbms6817.discharge_data,0x00,sizeof(Adbms6817.discharge_data));
	memset(Adbms6817.voltage_cell,0x00,sizeof(Adbms6817.voltage_cell));
	memset(Adbms6817.temp_gpio,0x00,sizeof(Adbms6817.temp_gpio));
*/
	memset(&Adbms6817,0x00,sizeof(struct _ADBMS6817));
	memset(&Bms,0x00,sizeof(struct _BMS));

	PEC15_Table_initialize();
	PEC10_Table_initialize();
	//set_adc(MD_NORMAL, DCP_DISABLED, CELL_CH_ALL, AUX_CH_ALL);

	Adbms6817_init_reg();
}

/*!*********************************************************************************************
 \brief Starts cell voltage conversion

 Starts ADC conversions of the LTC6804 Cpin inputs.
 The type of ADC conversion executed can be changed by setting the associated global variables:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CH     | Determines which cell channels are converted |
 | DCP    | Determines if Discharge is Permitted	     |

 Command Code:
 -------------

 |CMD[0:1]	|  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
 |----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
 |ADCV:	    |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |
 ***********************************************************************************************/
void Adbms6817_adcv()
{

	uint8_t cmd[4];
	uint16_t cmd_pec;

	//1
	//cmd[0] = ADCV[0];
	//cmd[1] = ADCV[1];
	cmd[0] = 0x03;
	cmd[1] = 0x60;

	//2
	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	//3
	wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

	//4
	Adbms6817_CS_low
	spi_write_array(4, cmd);
	Adbms6817_CS_high
}

/*
 LTC6804_adcv Function sequence:

 1. Load adcv command into cmd array
 2. Calculate adcv cmd PEC and load pec into cmd array
 3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
 4. send broadcast adcv command to LTC6804 daisy chain
 */

/*!******************************************************************************************************
 \brief Start an GPIO Conversion

 Starts an ADC conversions of the LTC6804 GPIO inputs.
 The type of ADC conversion executed can be changed by setting the associated global variables:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CHG    | Determines which GPIO channels are converted |


 Command Code:
 -------------

 |CMD[0:1]	|  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
 |-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
 |ADAX:	    |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]|
 *********************************************************************************************************/
void Adbms6817_adax()
{
	uint8_t cmd[4];
	uint16_t cmd_pec;

	//cmd[0] = ADAX[0];
	//cmd[1] = ADAX[1];
	cmd[0] = 0x05;
	cmd[1] = 0x60;

	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);
	wakeup_idle();
	wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
	Adbms6817_CS_low
	spi_write_array(4, cmd);
	Adbms6817_CS_high

}

/*
 LTC6804_adcv Function sequence:

 1. Load adcv command into cmd array
 2. Calculate adcv cmd PEC and load pec into cmd array
 3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
 4. send broadcast adcv command to LTC6804 daisy chain
 */

/*!******************************************************************************************************
 \brief Start an cell voltage  GPIO Conversion

 Starts an ADC conversions of the LTC6804 GPIO inputs.
 The type of ADC conversion executed can be changed by setting the associated global variables:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CHG    | Determines which GPIO channels are converted |


 Command Code:
 -------------

 |CMD[0:1]	|  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
 |----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
 |ADAX:	    |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |   0   |   1   |   1   |   1   |   1   |
 *********************************************************************************************************/
void Adbms6817_adcvax()
{
	uint8_t cmd[4];
	uint16_t cmd_pec;
	//

	cmd[0] = 0x05;
	//cmd[1] = 0xef;
	cmd[1] = 0x6f;	//MD[1] = 1 //MD[0] = 0
	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);
	wakeup_idle();
	Adbms6817_CS_low
	spi_write_array(4, cmd);
	Adbms6817_CS_high
}



/*!******************************************************************************************************
Command Code:
-------------

|CMD[0:1]	|  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADAX:	    |   0   |   0   |   0   |   0   |   0   |   0   |   0   | MD[1] | MD[2] |   1   |   1   |   DC  |   1   | CH[2] | CH[1] | CH[0] |
*********************************************************************************************************/
void Adbms6817_adsc(uint8_t cell)
{
	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[0] = 0x01;
	//cmd[1] = 0xef;
	cmd[1] = 0x68 | cell;	//MD[1] = 1 //MD[0] = 0
	//cmd[1] = 0x78 | cell;	//MD[1] = 1 //MD[0] = 0
	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);
	wakeup_idle();
	Adbms6817_CS_low
	spi_write_array(4, cmd);
	Adbms6817_CS_high
}



/***********************************************//**
 \brief Read the raw data from the ADMBS6817 cell voltage register

 The function reads a single cell voltage register and stores the read data
 in the *data point as a byte array. This function is rarely used outside of
 the ADBMS6817_rdcv() command.

 @param[in] uint8_t reg; This controls which cell voltage register is read back.

 1: Read back cell group A

 2: Read back cell group B

 3: Read back cell group C

 4: Read back cell group D

 @param[in] uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)

 @param[out] uint8_t *data; An array of the unparsed cell codes

  Example for Group A
 ---------------------
 |*data  	|    7    |    6    |    5    |    4    |    3    |    2    |    1    |    0    |
 |data[0]  	| C1V[7]  | C1V[6]  | C1V[5]  | C1V[4]  | C1V[3]  | C1V[2]  | C1V[1]  | C1V[0]  |
 |data[1]  	| C1V[15] | C1V[14] | C1V[13] | C1V[12] | C1V[11] | C1V[10] | C1V[9]  | C1V[8]  |
 |data[2]  	| C2V[7]  | C2V[6]  | C2V[5]  | C2V[4]  | C2V[3]  | C2V[2]  | C2V[1]  | C2V[0]  |
 |data[3]  	| C2V[15] | C2V[14] | C2V[13] | C2V[12] | C2V[11] | C2V[10] | C2V[9]  | C2V[8]  |
 |data[4]  	| C2V[7]  | C2V[6]  | C2V[5]  | C2V[4]  | C2V[3]  | C2V[2]  | C2V[1]  | C2V[0]  |
 |data[5]  	| C2V[15] | C2V[14] | C2V[13] | C2V[12] | C2V[11] | C2V[10] | C2V[9]  | C2V[8]  |
 |data[6]  	|                                      PEC                                      |
 |data[7]  	|                                      PEC                                      |

  Command Code:
 -------------
 |CMD[0:1]	|  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
 |----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
 |RDCVA:	|   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |
 |RDCVB:	|   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   0   |
 |RDCVC:	|   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   0   |
 |RDCVD:	|   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   1   |   0   |
  *************************************************/
void Adbms6817_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
		uint8_t total_ic, //the number of ICs in the
		uint8_t *data //An array of the unparsed cell codes
		)
{
	const uint8_t REG_LEN = 8; //number of bytes in each ICs register + 2 bytes for the PEC
	uint8_t cmd[4];
	uint16_t cmd_pec;

	//1
	if (reg == 1)     //1: RDCVA(1~3 CELL)
	{
		cmd[1] = 0x04;
		cmd[0] = 0x00;
	}
	else if (reg == 2) //2: RDCVB(4~6 CELL)
	{
		cmd[1] = 0x06;
		cmd[0] = 0x00;
	}
	else if (reg == 3) //3: RDCVC(7~8 CELL)
	{
		cmd[1] = 0x08;
		cmd[0] = 0x00;
	}
	else if (reg == 4) //4: RDCVD(RESERVED)
	{
		cmd[1] = 0x0A;
		cmd[0] = 0x00;
	}

	//2
	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	//3
	wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
	wakeup_idle();
	wakeup_idle();

	//4
	Adbms6817_CS_low;
	spi_write_read(cmd, 4, data, (REG_LEN * total_ic));
	Adbms6817_CS_high;

	if(reg == 1)
	{
		Adbms6817.voltage_cell[0] = (data[1] << 8) + data[0];
		Adbms6817.voltage_cell[1] = (data[3] << 8) + data[2];
		Adbms6817.voltage_cell[2] = (data[5] << 8) + data[4];
	}
	if(reg == 2)
	{
		Adbms6817.voltage_cell[3] = (data[1] << 8) + data[0];
		Adbms6817.voltage_cell[4] = (data[3] << 8) + data[2];
		Adbms6817.voltage_cell[5] = (data[5] << 8) + data[4];
	}
	if(reg == 3)
	{
		Adbms6817.voltage_cell[6] = (data[1] << 8) + data[0];
		Adbms6817.voltage_cell[7] = (data[3] << 8) + data[2];
	}
}



void Adbms6817_rdcd_reg(uint8_t reg, //Determines which cell voltage register is read back
		uint8_t total_ic, //the number of ICs in the
		uint8_t *data //An array of the unparsed cell codes
		)
{
	const uint8_t REG_LEN = 8; //number of bytes in each ICs register + 2 bytes for the PEC
	uint8_t cmd[4];
	uint16_t cmd_pec;

	//1
	if (reg == 1)     //1: RDCDA(1~3 CELL)
	{
		cmd[1] = 0x30;
		cmd[0] = 0x00;
	}
	else if (reg == 2) //2: RDCDB(4~6 CELL)
	{
		cmd[1] = 0x31;
		cmd[0] = 0x00;
	}
	else if (reg == 3) //3: RDCDC(7~8 CELL)
	{
		cmd[1] = 0x32;
		cmd[0] = 0x00;
	}
	else if (reg == 4) //4: RDCDD(RESERVED)
	{
		cmd[1] = 0x33;
		cmd[0] = 0x00;
	}

	//2
	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	//3
	wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
	wakeup_idle();
	wakeup_idle();

	//4
	Adbms6817_CS_low;
	spi_write_read(cmd, 4, data, (REG_LEN * total_ic));
	Adbms6817_CS_high;

	if(reg == 1)
	{
		Adbms6817.diagnostic_cell[0] = (data[1] << 8) + data[0];
		Adbms6817.diagnostic_cell[1] = (data[3] << 8) + data[2];
		Adbms6817.diagnostic_cell[2] = (data[5] << 8) + data[4];
	}
	if(reg == 2)
	{
		Adbms6817.diagnostic_cell[3] = (data[1] << 8) + data[0];
		Adbms6817.diagnostic_cell[4] = (data[3] << 8) + data[2];
		Adbms6817.diagnostic_cell[5] = (data[5] << 8) + data[4];
	}
	if(reg == 3)
	{
		Adbms6817.diagnostic_cell[6] = (data[1] << 8) + data[0];
		Adbms6817.diagnostic_cell[7] = (data[3] << 8) + data[2];
	}
}




void Adbms6817_rdaux_reg(uint8_t reg, //Determines which cell voltage register is read back
		uint8_t total_ic, //the number of ICs in the
		uint8_t *data //An array of the unparsed cell codes
		)
{
	const uint8_t REG_LEN = 8; //number of bytes in each ICs register + 2 bytes for the PEC
	uint8_t cmd[4];
	uint16_t cmd_pec;

	//1
	if (reg == 1)     //1:
	{
		cmd[1] = 0x0C;
		cmd[0] = 0x00;
	}
	else if (reg == 2) //2:
	{
		cmd[1] = 0x0E;
		cmd[0] = 0x00;
	}
	else if (reg == 3) //3:
	{
		cmd[1] = 0x0D;
		cmd[0] = 0x00;
	}


	//2
	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	//3
	wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
	wakeup_idle();
	wakeup_idle();

	//4
	Adbms6817_CS_low;
	spi_write_read(cmd, 4, data, (REG_LEN * total_ic));
	Adbms6817_CS_high;

	if(reg == 1)
	{
		Adbms6817.temp_gpio[0] = (data[3] << 8) + data[2];
		Adbms6817.temp_gpio[1] = (data[5] << 8) + data[4];
	}
}

void Adbms6817_rdstat_reg(uint8_t reg, //Determines which cell voltage register is read back
		uint8_t total_ic, //the number of ICs in the
		uint8_t *data //An array of the unparsed cell codes
		)
{
	const uint8_t REG_LEN = 8; //number of bytes in each ICs register + 2 bytes for the PEC
	uint8_t cmd[4];
	uint16_t cmd_pec;

	//1
	if (reg == 1)     //1: Group A
	{
		cmd[1] = 0x10;
		cmd[0] = 0x00;
	}
	else if (reg == 2) //2: RDCVB(4~6 CELL)
	{
		cmd[1] = 0x12;
		cmd[0] = 0x00;
	}
	else if (reg == 3) //3: RDCVC(7~8 CELL)
	{
		cmd[1] = 0x13;
		cmd[0] = 0x00;
	}

	//2
	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	//3
	wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
	wakeup_idle();
	wakeup_idle();
	wakeup_idle();
	wakeup_idle();

	//4
	Adbms6817_CS_low;
	spi_write_read(cmd, 4, data, (REG_LEN * total_ic));
	Adbms6817_CS_high;
}

/***********************************************//**
 \brief Read the all raw data from the ADMBS6817 cell voltage register
  @param[out] uint8_t *data; An array of the unparsed cell codes
  *************************************************/
void Adbms6817_rdcv_reg_ALL(uint8_t *data)
{
	const uint8_t REG_LEN = 20; // register(3*6) + 2 bytes for the PEC
	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[1] = 0x38;
	cmd[0] = 0x00;

	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	wakeup_idle();
	wakeup_idle();
	wakeup_idle();
	wakeup_idle();
	wakeup_idle();

	Adbms6817_CS_low;
	spi_write_read(cmd, 4, data, REG_LEN);
	Adbms6817_CS_high;


	Adbms6817.voltage_cell[0] = (data[1] << 8) + data[0];
	Adbms6817.voltage_cell[1] = (data[3] << 8) + data[2];
	Adbms6817.voltage_cell[2] = (data[5] << 8) + data[4];

	Adbms6817.voltage_cell[3] = (data[7] << 8) + data[6];
	Adbms6817.voltage_cell[4] = (data[9] << 8) + data[8];
	Adbms6817.voltage_cell[5] = (data[11] << 8) + data[10];

	Adbms6817.voltage_cell[6] = (data[13] << 8) + data[12];
	Adbms6817.voltage_cell[7] = (data[15] << 8) + data[14];
	printf("5\r\n");
}

/***********************************************//**
 \brief Read the all raw data from the ADMBS6817 cell diagnostic register
  @param[out] uint8_t *data; An array of the unparsed cell codes
  *************************************************/
void Adbms6817_rdcd_reg_ALL(uint8_t *data)
{
	const uint8_t REG_LEN = 20; // register(3*6) + 2 bytes for the PEC
	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[1] = 0x3a;
	cmd[0] = 0x00;

	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	wakeup_idle();
	wakeup_idle();
	wakeup_idle();
	wakeup_idle();
	wakeup_idle();

	Adbms6817_CS_low;
	spi_write_read(cmd, 4, data, REG_LEN);
	Adbms6817_CS_high;
}

/***********************************************//**
 \brief Reads and parses the Adbms6804 cell voltage registers.

 The function is used to read the cell codes of the LTC6804.
 This function will send the requested read commands parse the data
 and store the cell voltages in cell_codes variable.

 @param[in] uint8_t reg; This controls which cell voltage register is read back.

 0: Read back all Cell registers

 1: Read back cell group A

 2: Read back cell group B

 3: Read back cell group C

 4: Read back cell group D

 @param[in] uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)

 @param[out] uint16_t cell_codes[]; An array of the parsed cell codes from lowest to highest. The cell codes will
 be stored in the cell_codes[] array in the following format:
 |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
 |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
 |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |

 @return int8_t, PEC Status.

 0: No PEC error detected

 -1: PEC error detected, retry read


 *************************************************/
uint8_t Adbms_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
		uint8_t total_ic, // the number of ICs in the system
		uint16_t cell_codes[][12] // Array of the parsed cell codes
		)
{

	const uint8_t NUM_RX_BYT = 8;
	const uint8_t BYT_IN_REG = 6;
	const uint8_t CELL_IN_REG = 3;

	uint8_t cell_reg;
	uint8_t current_ic;
	uint8_t current_cell;

	uint8_t cell_data[120];
	uint8_t pec_error = 0;
	uint16_t parsed_cell;
	uint16_t received_pec;
	uint16_t data_pec;
	uint8_t data_counter = 0; //data counter
	//cell_data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));8*15
	//1.a
	if (reg == 0)
	{
		//a.i
		for (cell_reg = 1; cell_reg < 5; cell_reg++) //executes once for each of the LTC6804 cell voltage registers
		{
			data_counter = 0;
			Adbms6817_rdcv_reg(cell_reg, total_ic, cell_data);//Reads a single Cell voltage register

			for (current_ic = 0; current_ic < total_ic; current_ic++) // executes for every LTC6804 in the daisy chain
			{							// current_ic is used as the IC counter

				//a.ii
				for (current_cell = 0; current_cell < CELL_IN_REG;
						current_cell++)	// This loop parses the read back data into cell voltages, it
				{// loops once for each of the 3 cell voltage codes in the register

					parsed_cell = cell_data[data_counter]
							+ (cell_data[data_counter + 1] << 8);//Each cell code is received as two bytes and is combined to
					// create the parsed cell voltage code

					cell_codes[current_ic][current_cell
							+ ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
					data_counter = data_counter + 2;//Because cell voltage codes are two bytes the data counter
					//must increment by two for each parsed cell code
				}
				//a.iii
				received_pec = (cell_data[data_counter] << 8)
						+ cell_data[data_counter + 1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
													   //after the 6 cell voltage data bytes
				data_pec = PEC15_calc(BYT_IN_REG,
						&cell_data[current_ic * NUM_RX_BYT]);
				if (received_pec != data_pec)
				{
					pec_error = -1;	//The pec_error variable is simply set negative if any PEC errors
									//are detected in the serial data
				}
				data_counter = data_counter + 2;//Because the transmitted PEC code is 2 bytes long the data_counter
				//must be incremented by 2 bytes to point to the next ICs cell voltage data
			}
		}
	}
	//1.b
	else
	{
		//b.i
		Adbms6817_rdcv_reg(reg, total_ic, cell_data);
		for (current_ic = 0; current_ic < total_ic; current_ic++) // executes for every LTC6804 in the daisy chain
		{								// current_ic is used as the IC counter
			//b.ii
			for (current_cell = 0; current_cell < CELL_IN_REG; current_cell++) // This loop parses the read back data into cell voltages, it
			{// loops once for each of the 3 cell voltage codes in the register

				parsed_cell = cell_data[data_counter]
						+ (cell_data[data_counter + 1] << 8); //Each cell code is received as two bytes and is combined to
															  // create the parsed cell voltage code

				cell_codes[current_ic][current_cell + ((reg - 1) * CELL_IN_REG)] =
						0x0000FFFF & parsed_cell;
				data_counter = data_counter + 2; //Because cell voltage codes are two bytes the data counter
												 //must increment by two for each parsed cell code
			}
			//b.iii
			received_pec = (cell_data[data_counter] << 8)
					+ cell_data[data_counter + 1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
												   //after the 6 cell voltage data bytes
			data_pec = PEC15_calc(BYT_IN_REG,
					&cell_data[current_ic * NUM_RX_BYT]);
			if (received_pec != data_pec)
			{
				pec_error = -1;	//The pec_error variable is simply set negative if any PEC errors
								//are detected in the serial data
			}
			data_counter = data_counter + 2; //Because the transmitted PEC code is 2 bytes long the data_counter
											 //must be incremented by 2 bytes to point to the next ICs cell voltage data
		}
	}

	//2
	//free(cell_data);
	return (pec_error);
}
/*
 LTC6804_rdcv Sequence

 1. Switch Statement:
 a. Reg = 0
 i. Read cell voltage registers A-D for every IC in the daisy chain
 ii. Parse raw cell voltage data in cell_codes array
 iii. Check the PEC of the data read back vs the calculated PEC for each read register command
 b. Reg != 0
 i.Read single cell voltage register for all ICs in daisy chain
 ii. Parse raw cell voltage data in cell_codes array
 iii. Check the PEC of the data read back vs the calculated PEC for each read register command
 2. Return pec_error flag
 */




/*****************************************************//**
 \brief Write the ADBMS6817 configuration register

 This command will write the configuration registers of the LTC6804-1s
 connected in a daisy chain stack. The configuration is written in descending
 order so the last device's configuration is written first.

 @param[in] uint8_t total_ic; The number of ICs being written to.

 @param[in] uint8_t config[][6] is a two dimensional array of the configuration data that will be written, the array should contain the 6 bytes for each
 IC in the daisy chain. The lowest IC in the daisy chain should be the first 6 byte block in the array. The array should
 have the following format:
 |  config[0][0]| config[0][1] |  config[0][2]|  config[0][3]|  config[0][4]|  config[0][5]| config[1][0] |  config[1][1]|  config[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |

 The function will calculate the needed PEC codes for the write data
 and then transmit data to the ICs on a daisy chain.


 Command Code:
 -------------
 |               |							CMD[0]                               |                            CMD[1]                             |
 |---------------|---------------------------------------------------------------|---------------------------------------------------------------|
 |CMD[0:1]	     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
 |---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
 |WRCFG: GROUP A |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |
 |WRCFG: GROUP B |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   1   |   0   |   0   |
 ********************************************************/


void Adbms6817_wrcfg(uint8_t total_ic, //The number of ICs being written to
		uint8_t cfg_group, // 1:Group A, 2:Group B
		uint8_t config[][8] //A two dimensional array of the configuration data that will be written
		)
{
	uint16_t cmd_pec;
	const uint8_t BYTES_IN_REG = 6;
	const uint8_t CMD_LEN = 4 + (8 * total_ic);
	uint8_t cmd[124];
	uint16_t cfg_pec;
	uint8_t cmd_index; //command counter

	uint8_t current_ic;
	uint8_t current_byte;
	//cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));

	//1
	if(cfg_group == REG_GRP_A)
	{
		cmd[0] = 0x00;
		cmd[1] = 0x01;
	}
	else if(cfg_group == REG_GRP_B)
	{
		cmd[0] = 0x00;
		cmd[1] = 0x24;
	}
	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	//2
	cmd_index = 4;
	for (current_ic = total_ic; current_ic > 0; current_ic--) // executes for each LTC6804 in daisy chain, this loops starts with
	{			// the last IC on the stack. The first configuration written is
				// received by the last IC in the daisy chain

		for (current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each of the 6 bytes in the CFGR register
		{									// current_byte is the byte counter

			cmd[cmd_index] = config[current_ic - 1][current_byte]; //adding the config data to the array to be sent
			cmd_index = cmd_index + 1;
		}
		//3
		//cfg_pec = (uint16_t) PEC15_calc(BYTES_IN_REG, &config[current_ic - 1][0]);// calculating the PEC for each ICs configuration register data
		cfg_pec = (uint16_t) PEC10_calc(&config[current_ic - 1][0] , 0, BYTES_IN_REG);
		cmd[cmd_index] = (uint8_t) (cfg_pec >> 8);
		cmd[cmd_index + 1] = (uint8_t) cfg_pec;
		cmd_index = cmd_index + 2;
	}

	//4
	wakeup_idle();
	wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
	//5
	Adbms6817_CS_low;
	//delayMicroseconds(3);
	spi_write_array(CMD_LEN, cmd);
	Adbms6817_CS_high;
	//free(cmd);
}

/*!******************************************************
 \brief Reads configuration registers of a ADBMS6817 daisy chain

 @param[in] uint8_t total_ic: number of ICs in the daisy chain

 @param[out] uint8_t r_config[][8] is a two dimensional array that the function stores the read configuration data. The configuration data for each IC
 is stored in blocks of 8 bytes with the configuration data of the lowest IC on the stack in the first 8 bytes
 block of the array, the second IC in the second 8 byte etc. Below is an table illustrating the array organization:

 |r_config[0][0]|r_config[0][1]|r_config[0][2]|r_config[0][3]|r_config[0][4]|r_config[0][5]|r_config[0][6]  |r_config[0][7] |r_config[1][0]|r_config[1][1]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|----------------|---------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC1 PEC High    |IC1 PEC Low    |IC2 CFGR0     |IC2 CFGR1     |  .....    |


 @return int8_t, PEC Status.

 0: Data read back has matching PEC

 -1: Data read back has incorrect PEC


 Command Code:
 -------------

 |CMD[0:1]		|  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
 |--------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
 |RDCFG GROUP A |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |
 |RDCFG GROUP B |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   1   |   1   |   0   |
 ********************************************************/
int8_t Adbms6817_rdcfg(uint8_t total_ic, //Number of ICs in the system
		uint8_t cfg_group, // 1:Group A, 2:Group B
		uint8_t r_config[][8] //A two dimensional array that the function stores the read configuration data.
		)
{
	uint16_t cmd_pec;
	const uint8_t BYTES_IN_REG = 8;

	uint8_t cmd[4];
	uint8_t rx_data[20];
	int8_t pec_error = 0;
	uint16_t data_pec;
	uint16_t received_pec;

	uint8_t current_ic;
	uint8_t current_byte;

	//rx_data = (uint8_t *) malloc((8*total_ic)*sizeof(uint8_t));

	//1
	if(cfg_group == REG_GRP_A)
	{
		cmd[0] = 0x00;
		cmd[1] = 0x02;
	}
	else if(cfg_group == REG_GRP_B)
	{
		cmd[0] = 0x00;
		cmd[1] = 0x26;
	}

	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	//2
	wakeup_idle();
	wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

	//3
	Adbms6817_CS_low;

	spi_write_read(cmd, 4, rx_data, (BYTES_IN_REG * total_ic)); //Read the configuration data of all ICs on the daisy chain into
	Adbms6817_CS_high;

	//rx_data[] array
	for (current_ic = 0; current_ic < total_ic; current_ic++) //executes for each LTC6804 in the daisy chain and packs the data
	{ 		//into the r_config array as well as check the received Config data
			//for any bit errors
		//4.a
		for (current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
		{
			r_config[current_ic][current_byte] = rx_data[current_byte
					+ (current_ic * BYTES_IN_REG)];	//
		}
		//4.b
		received_pec = ((r_config[current_ic][6] & 0x03)  << 8) + r_config[current_ic][7];
		//data_pec = PEC15_calc(6, &r_config[current_ic][0]);
		data_pec =  PEC10_calc(&r_config[current_ic][0] , 1, 6);
#if 0
		for(i=0;i<8;i++)
		{
			if(r_config[0][4] & (0x1 << i))
				Adbms6817.discharge_state[i] = TRUE;
			else
				Adbms6817.discharge_state[i] = FALSE;
		}
		if(r_config[0][5] & 0x1)
		{
			Adbms6817.discharge_state[8] = TRUE;
		}
		else
		{
			Adbms6817.discharge_state[8] = FALSE;
		}
#endif

		if (received_pec != data_pec)
		{
			pec_error = -1;
		}
	}

	//free(rx_data);
	//5
	return (pec_error);
}


/***********************************************//**
 \brief Read the unique 48-bit serial identification code(SID) from the ADMBS6817
  @param[out] uint8_t *data; An array of the SID
  *************************************************/
void Adbms6817_rdsid(uint8_t *data)
{
	const uint8_t REG_LEN = 8; // id(6) + 2 bytes for the PEC
	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[0] = 0x00;
	cmd[1] = 0x2C;

	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	wakeup_idle();
	wakeup_idle();
	wakeup_idle();
	wakeup_idle();
	wakeup_idle();

	Adbms6817_CS_low;
	spi_write_read(cmd, 4, data, REG_LEN);
	Adbms6817_CS_high;
}


/***********************************************//**
 \brief Read all the Auxiliary/Status Register from the ADMBS6817
  @param[out] uint8_t *data array of Auxiliary/Status Register data
  *************************************************/
void Adbms6817_rdasall(uint8_t *data)
{
	const uint8_t REG_LEN = 38; // reg(6*6) + 2 bytes for the PEC
	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[1] = 0x3c;
	cmd[0] = 0x00;

	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	wakeup_idle();
	wakeup_idle();
	wakeup_idle();
	wakeup_idle();
	wakeup_idle();

	Adbms6817_CS_low;
	spi_write_read(cmd, 4, data, REG_LEN);
	Adbms6817_CS_high;
}


void Adbms6817_rdpwm(uint8_t *data)
{
	const uint8_t REG_LEN = 8; // id(6) + 2 bytes for the PEC
	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[0] = 0x00;
	cmd[1] = 0x22;

	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	wakeup_idle();
	wakeup_idle();
	wakeup_idle();
	wakeup_idle();
	wakeup_idle();

	Adbms6817_CS_low;
	spi_write_read(cmd, 4, data, REG_LEN);
	Adbms6817_CS_high;
}

void Adbms6817_wrpwm(uint8_t total_ic, //The number of ICs being written to
		uint8_t config[][8] //A two dimensional array of the configuration data that will be written
		)
{
	uint16_t cmd_pec;
	const uint8_t BYTES_IN_REG = 6;
	const uint8_t CMD_LEN = 4 + (8 * total_ic);
	uint8_t cmd[124];
	uint16_t cfg_pec;
	uint8_t cmd_index; //command counter

	uint8_t current_ic;
	uint8_t current_byte;
	//cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));

	//1
	cmd[0] = 0x00;
	cmd[1] = 0x20;

	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	//2
	cmd_index = 4;
	for (current_ic = total_ic; current_ic > 0; current_ic--) // executes for each LTC6804 in daisy chain, this loops starts with
	{			// the last IC on the stack. The first configuration written is
				// received by the last IC in the daisy chain

		for (current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each of the 6 bytes in the CFGR register
		{									// current_byte is the byte counter

			cmd[cmd_index] = config[current_ic - 1][current_byte]; //adding the config data to the array to be sent
			cmd_index = cmd_index + 1;
		}
		//3
		//cfg_pec = (uint16_t) PEC15_calc(BYTES_IN_REG, &config[current_ic - 1][0]);// calculating the PEC for each ICs configuration register data
		cfg_pec = (uint16_t) PEC10_calc(&config[current_ic - 1][0] , 0, BYTES_IN_REG);
		cmd[cmd_index] = (uint8_t) (cfg_pec >> 8);
		cmd[cmd_index + 1] = (uint8_t) cfg_pec;
		cmd_index = cmd_index + 2;
	}

	//4
	wakeup_idle();
	wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
	//5
	Adbms6817_CS_low;
	//delayMicroseconds(3);
	spi_write_array(CMD_LEN, cmd);
	Adbms6817_CS_high;
	//free(cmd);
}

void Adbms6817_discharge_cell_continuously(uint8_t Sx /* x : adbms6817 Sx Pin Number*/, uint8_t state)
{
	uint8_t data[NUM_OF_ADBMS6817][8];

	printf("Sx : %d state: %d\r\n", Sx, state);

#if 0
	//do not use S6, S7
	if((Sx == S6) || (Sx == S7))
		return;
#endif
	Adbms6817_rdcfg(NUM_OF_ADBMS6817,REG_GRP_B,data);
	printf("Adbms6817_rdcfg : ");
		for(int i=0;i<7;i++)
		{
			printf("%02X ",data[0][i]);
		}
		printf("\r\n");

	if(state)
	{
	  //data[0][3] |= 0xBC;	// Discharge Timer Enable, 60min
	  data[0][4] |= (0x1 << Sx);
	}
	else
	{
	  data[0][4] &= (~0x1 << Sx);
	}

	printf("Adbms6817_rdcfg : ");
	for(int i=0;i<7;i++)
	{
		printf("%02X ",data[0][i]);
	}
	printf("\r\n");
	Adbms6817_wrcfg(NUM_OF_ADBMS6817,REG_GRP_B,data);

	Adbms6817_rdcfg(NUM_OF_ADBMS6817,REG_GRP_B,data);
}
/***********************************************//**
 \brief Mute discharge the ADMBS6817
  *************************************************/
void Adbms6817_mute(void)
{
	const uint8_t CMD_LEN = 4;
	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[1] = 0x24;
	cmd[0] = 0x00;

	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	wakeup_idle();
	wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.

	Adbms6817_CS_low;
	spi_write_array(CMD_LEN, cmd);
	Adbms6817_CS_high;
}

/***********************************************//**
 \brief Unmute discharge the ADMBS6817
  *************************************************/
void Adbms6817_unmute(void)
{
	const uint8_t CMD_LEN = 4;
	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[1] = 0x25;
	cmd[0] = 0x00;

	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	wakeup_idle();
	wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.

	Adbms6817_CS_low;
	spi_write_array(CMD_LEN, cmd);
	Adbms6817_CS_high;
}

/***********************************************//**
 \brief Soft Reset the ADMBS6817
  *************************************************/
void Adbms6817_srst(void)
{
	const uint8_t CMD_LEN = 4;
	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[1] = 0x1F;
	cmd[0] = 0x00;

	cmd_pec = PEC15_calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	wakeup_idle();
	wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.

	Adbms6817_CS_low;
	spi_write_array(CMD_LEN, cmd);
	Adbms6817_CS_high;
}

void Adbms6817_discharge_cell_pwm(uint8_t Sx /* x : adbms6817 Sx Pin Number*/, uint8_t state)
{
	switch(Sx)
	{
	case 0x00:
		if(state) Adbms6817.discharge_data[0][0] |= 0x0F;
		else			Adbms6817.discharge_data[0][0] &= 0xF0;
		break;
	case 0x01:
		if(state)	Adbms6817.discharge_data[0][0] |= 0xF0;
		else			Adbms6817.discharge_data[0][0] &= 0x0F;
		break;
	case 0x02:
		if(state)	Adbms6817.discharge_data[0][1] |= 0x0F;
		else			Adbms6817.discharge_data[0][1] &= 0xF0;
		break;
	case 0x03:
		if(state)	Adbms6817.discharge_data[0][1] |= 0xF0;
		else			Adbms6817.discharge_data[0][1] &= 0x0F;
		break;
	case 0x04:
		if(state)	Adbms6817.discharge_data[0][2] |= 0x0F;
		else			Adbms6817.discharge_data[0][2] &= 0xF0;
		break;
	case 0x05:
		if(state)	Adbms6817.discharge_data[0][2] |= 0xF0;
		else			Adbms6817.discharge_data[0][2] &= 0x0F;
		break;
	case 0x06:
		if(state)	Adbms6817.discharge_data[0][3] |= 0x0F;
		else			Adbms6817.discharge_data[0][3] &= 0xF0;
		break;
	case 0x07:
		if(state)	Adbms6817.discharge_data[0][3] |= 0xF0;
		else			Adbms6817.discharge_data[0][3] &= 0x0F;
		break;
	}

	if(state)	Adbms6817.discharge_state[Sx] = 1;
	else		Adbms6817.discharge_state[Sx] = 0;

	Adbms6817_wrpwm(NUM_OF_ADBMS6817,	Adbms6817.discharge_data);
}

void PEC15_Table_initialize()
{
	int16_t remainder;
	int16_t CRC15_POLY = 0x4599;

	for (int i = 0; i < 256; i++)
	{
		remainder = i << 7;
		for (int bit = 8; bit > 0; --bit)
		{
			if (remainder & 0x4000)
			{
				remainder = ((remainder << 1));
				remainder = (remainder ^ CRC15_POLY);
			}
			else
			{
				remainder = ((remainder << 1));
			}
		}
		Adbms6817.crc15Table[i] = remainder&0xFFFF;
	}
}

void PEC10_Table_initialize()
{
	int16_t remainder;
	int16_t CRC10_POLY = 0x8F;

	for (int i = 0; i < 256; i++)
	{
		remainder = i << 2;
		for (int bit = 8; bit > 0; --bit)
		{
			if (remainder & 0x200)
			{
				remainder = ((remainder << 1));
				remainder = (remainder ^ CRC10_POLY);
			}
			else
			{
				remainder = ((remainder << 1));
			}
		}
		Adbms6817.crc10Table[i] = remainder&0x3FF;
	}
}


uint16_t PEC15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
		uint8_t *data //Array of data that will be used to calculate  a PEC
		)
{
	uint16_t remainder, addr;
	uint8_t i;

	remainder = 16; //initialize the PEC
	for (i = 0; i < len; i++) // loops for each byte in data array
	{
		addr = ((remainder >> 7) ^ data[i]) & 0xff; //calculate PEC table address
		remainder = (remainder << 8) ^ Adbms6817.crc15Table[addr];
	}
	return (remainder * 2); //The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

/* Function to calculate PEC10. When data being processed is data received from device, set bIsRxCmd to true, else set to false*/
uint16_t PEC10_calc(uint8_t *pDataBuf , uint8_t bIsRxCmd, uint8_t nLength)
{
	uint16_t nRemainder = 16u;/* PEC_SEED */
	uint16_t nPolynomial = 0x8Fu;/* x10 + x7 + x3 + x2 + x + 1 <- the CRC10 polynomial 100 1000 1111 */
	uint8_t nByteIndex;
	uint8_t nBitIndex;
	uint16_t nTableAddr;
	for (nByteIndex = 0u; nByteIndex < nLength; ++nByteIndex)
	{
		/* calculate PEC table address */
		nTableAddr = (uint16_t)((uint16_t)(nRemainder >> 2) ^ (uint8_t)pDataBuf[nByteIndex]) & (uint8_t)0xff;
		nRemainder = ((uint16_t)(nRemainder << 8)) ^ Adbms6817.crc10Table[nTableAddr];
	}
	if (bIsRxCmd == 1) /* If array is from received buffer add command counter to crc calculation */
	{
		nRemainder ^= (uint16_t)(((uint16_t)pDataBuf[nLength] & (uint8_t)0xFC) << 2u);
	}
	/* Perform modulo-2 division, a bit at a time on rest of the bits */
	for (nBitIndex = 6u; nBitIndex > 0u; --nBitIndex)
	{
		/* Try to divide the current data bit */
		if ((nRemainder & 0x200u) > 0u)
		{
			nRemainder = (uint16_t)((nRemainder << 1u));
			nRemainder = (uint16_t)(nRemainder ^ nPolynomial);
		}
		else
		{
			nRemainder = (uint16_t)((nRemainder << 1u));
		}
	}
	return ((uint16_t)(nRemainder & 0x3FFu));
}
#if 0
//Command Code:
//set_adc(MD_NORMAL,DCP_DISABLED,CELL_CH_ALL,AUX_CH_ALL);
//			2
//|command--|  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
//|---------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
//|ADCV:----|   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |
//|ADAX:----|   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]|
void set_adc(	uint8_t MD,		//ADC Mode
		uint8_t DCP,	//Discharge Permit
		uint8_t CH,		//Cell Channels to be measured
		uint8_t CHG)	//GPIO Channels to be measured
{
	uint8_t md_bits;

	md_bits = (MD & 0x02) >> 1;
	ADCV[0] = md_bits + 0x02;
	md_bits = (MD & 0x01) << 7;
	ADCV[1] = md_bits + 0x60 + (DCP << 4) + CH;

	md_bits = (MD & 0x02) >> 1;
	ADAX[0] = md_bits + 0x04;
	md_bits = (MD & 0x01) << 7;
	ADAX[1] = md_bits + 0x60 + CHG;
}
#endif
uint8_t check_full_charge(void)
{
	uint32_t sum_cell_voltage = 0, i;
	float avg_cell_voltage = 0;

	for(i=0;i<8;i++)
	{
		sum_cell_voltage += Adbms6817.voltage_cell[i];
	}

	avg_cell_voltage = sum_cell_voltage / 10000. / 7.;

	if(avg_cell_voltage > 4.1)
		return 1;
	else
		return 0;
}


void Calc_SOC_Voltage(void)
{
	//float calc_soc = 0;
	uint32_t sum_cell_voltage = 0, i;
	float avg_cell_voltage = 0;

	for(i=0;i<8;i++)
	{
		sum_cell_voltage += Adbms6817.voltage_cell[i];
	}
	avg_cell_voltage = sum_cell_voltage / 10000. / 7.;
#if 0
	if(avg_cell_voltage >= 3.5)			Bms.Soc.fVoltage = (avg_cell_voltage*1000 - 3260)/9;
	else if(avg_cell_voltage >= 3.25)	Bms.Soc.fVoltage = (avg_cell_voltage -3.09)/0.0156;
	else								Bms.Soc.fVoltage = (avg_cell_voltage-2.65)/0.06;
#endif
	if(avg_cell_voltage >= 3.42)			Bms.Soc = (avg_cell_voltage - 3.18)/0.0096;
	else if(avg_cell_voltage >= 3.12)	Bms.Soc = (avg_cell_voltage -2.92)/0.02;
	else								Bms.Soc = (avg_cell_voltage-2.50)/0.062;
#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_BMS)
	printf("SOC : %.2f\r\n",Bms.Soc);
#endif
}

#define FULL_CURRNET 144000	// 4Ah = 144000 100mAsec
int32_t Current_Consumption_Total_100mASec = 0;
#if 0
void Calc_SOC_Current(void)
{
	uint32_t remain;
	int16_t ct_Amp;
	static int32_t uCt_Amp_last = 0;

	ct_Amp = g_Adc_ct_Amp - g_uCurrent_offset;
#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_BMS)
	printf("ct_Amp : %d, Adc_ct_Amp : %d, current_offset : %d\r\n", ct_Amp, g_Adc_ct_Amp, g_uCurrent_offset);
#endif
	if(check_full_charge())
	{
		// full charge check
		remain = FULL_CURRNET;
	}

	if(uCt_Amp_last > ct_Amp)
	{
		Current_Consumption_Total_100mASec += (ct_Amp + (uCt_Amp_last - ct_Amp)/2);
	}
	else if(ct_Amp > uCt_Amp_last)
	{
		Current_Consumption_Total_100mASec += (uCt_Amp_last + (ct_Amp - uCt_Amp_last)/2);
	}
	else
	{
		Current_Consumption_Total_100mASec += ct_Amp;
	}

	remain = FULL_CURRNET - Current_Consumption_Total_100mASec;
	Bms.Soc.fCurrunt = ((float)remain / (float)FULL_CURRNET) * 100;
	uCt_Amp_last = ct_Amp;
#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_BMS)
	printf("Consumption : %d, Total Consumption : %ld\r\n",ct_Amp, Current_Consumption_Total_100mASec);
	printf("remain: %ld, SOC_current : %.2f\r\n", remain, Bms.Soc.fCurrunt);
	printf("\r\n");
#endif
}
#endif
// Cell Balancing Routine
/*
 * Cell Balancing Conditions
 *  1. Charging State
 *  2. Average voltage over 3.5V
 *  3. Cell Balancing Start : Difference between highest and lowest voltage cell more than 500mV
*/


void Adbms6817_read_adc(void)
{
	uint8_t data[64], i;
	uint32_t sum_voltage = 0;;

	//Adbms6817_adcvax();
	Adbms6817_adcv();
#if 1
	Adbms6817_rdcv_reg(REG_GRP_A, NUM_OF_ADBMS6817, data);
	Adbms6817_rdcv_reg(REG_GRP_B, NUM_OF_ADBMS6817, data);
	Adbms6817_rdcv_reg(REG_GRP_C, NUM_OF_ADBMS6817, data);
#else
	Adbms6817_rdcv_reg_ALL(data);
#endif
	for(i=0;i<8;i++)
	{
		sum_voltage += Adbms6817.voltage_cell[i];
	}

	Bms.Voltage_avg = Adbms6817.voltage_avg = sum_voltage/7;

#if 0
	Adbms6817_adsc(1);
	Adbms6817_adsc(2);
	Adbms6817_adsc(3);
	Adbms6817_adsc(4);
	Adbms6817_adsc(5);
	Adbms6817_adsc(6);
	Adbms6817_rdcd_reg(REG_GRP_A, NUM_OF_ADBMS6817, data);
	Adbms6817_rdcd_reg(REG_GRP_B, NUM_OF_ADBMS6817, data);
	Adbms6817_rdcd_reg(REG_GRP_C, NUM_OF_ADBMS6817, data);
#endif
}

void Adbms6817_init_reg(void)
{
	uint8_t i;

	for(i=0;i<NUM_OF_CELL;i++)
	{
		Adbms6817_discharge_cell_continuously(i, FALSE);
		Adbms6817_discharge_cell_pwm(i, FALSE);
		Adbms6817.discharge_state[i] = FALSE;
	}
}

void wakeup_idle()
{
	Adbms6817_CS_low
	HAL_Delay(10);
	Adbms6817_CS_high
}

void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
		uint8_t data[] //Array of bytes to be written on the SPI port
		)
{
	uint8_t i;
	for (i = 0; i < len; i++)
	{
		//SPI1_ReadWriteByte((uint8_t) data[i]);
		HAL_SPI_Transmit(&hspi1, &data[i], 1, 1000);
	}
}

void spi_write_read(uint8_t tx_Data[], //array of data to be written on SPI port
		uint8_t tx_len, //length of the tx data arry
		uint8_t *rx_data, //Input: array that will store the data read by the SPI port
		uint8_t rx_len //Option: number of bytes to be read from the SPI port
		)
{
	uint8_t i;

	for (i = 0; i < tx_len; i++)
	{
		//SPI1_ReadWriteByte(tx_Data[i]);
		HAL_SPI_Transmit(&hspi1, &tx_Data[i], 1, 1000);
	}

	for (i = 0; i < rx_len; i++)
	{
		tx_Data[i] = 0x00;
		HAL_SPI_TransmitReceive(&hspi1, &tx_Data[i], &rx_data[i], 1, 1000);
	}
}
