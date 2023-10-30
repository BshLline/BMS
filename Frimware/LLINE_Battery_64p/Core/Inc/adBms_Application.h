/*******************************************************************************
Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc.
and its licensor.
******************************************************************************
* @file:    adBms_Application.h
* @brief:   Bms application header file
* @version: $Revision$
* @date:    $Date$
* Developed by: ADIBMS Software team, Bangalore, India
*****************************************************************************/
#ifndef __APPLICATION_H
#define __APPLICATION_H


#include <stdint.h>

#define TOTAL_IC 1

#define DISCHARGE_SINGLE_CELL

//Command Bit Descriptions MD[1:0]
//ADCOPT(CFGAR0[6]) = 0
//  |--|------|---------------------|
//  |MD| Dec  | ADC Conversion Model|
//	|00|0     ||422 Hz mode
//	|01|1     | 27 kHz mode (fast)
//	|10|2     | 7 kHz mode (normal)
//	|11|3     | 26 Hz mode (filtered)
#define MD_FAST 1
#define MD_NORMAL 2
#define MD_FILTERED 3

// |DCP | Discharge Permitted During conversion |
// |----|----------------------------------------|
// |0   | No - discharge is not permitted         |
// |1   | Yes - discharge is permitted           |
#define DCP_DISABLED 0
#define DCP_ENABLED 1

// |CH | Dec  | Channels to convert |
// |---|------|---------------------|
// |000| 0    | All Cells  		  |
// |001| 1    | Cell 1 and Cell 7   |
// |010| 2    | Cell 2 and Cell 8   |
// |011| 3    | Cell 3 and Cell 9   |
// |100| 4    | Cell 4 and Cell 10  |
// |101| 5    | Cell 5 and Cell 11  |
// |110| 6    | Cell 6 and Cell 12  |
#define CELL_CH_ALL 0
#define CELL_CH_1and7 1
#define CELL_CH_2and8 2
#define CELL_CH_3and9 3
#define CELL_CH_4and10 4
#define CELL_CH_5and11 5
#define CELL_CH_6and12 6

//  |CHG | Dec  |Channels to convert   |
//  |----|------|----------------------|
//  |000 | 0    | All GPIOS and 2nd Ref|
//  |001 | 1    | GPIO 1 			     |
//  |010 | 2    | GPIO 2               |
//  |011 | 3    | GPIO 3 			  	 |
//  |100 | 4    | GPIO 4 			  	 |
//  |101 | 5    | GPIO 5 			 	 |
//  |110 | 6    | Vref2  			  	 |
#define AUX_CH_ALL 0
#define AUX_CH_GPIO1 1
#define AUX_CH_GPIO2 2
#define AUX_CH_GPIO3 3
#define AUX_CH_GPIO4 4
#define AUX_CH_GPIO5 5
#define AUX_CH_VREF2 6

#define NUM_OF_ADBMS6817	1
#define NUM_OF_CELL			7

#define Adbms6817_CS       SPI1_nCS_Pin
#define Adbms6817_CS_PORT  SPI1_nCS_GPIO_Port
#define Adbms6817_CS_high  {HAL_GPIO_WritePin(Adbms6817_CS_PORT,Adbms6817_CS,GPIO_PIN_SET);}
#define Adbms6817_CS_low   {HAL_GPIO_WritePin(Adbms6817_CS_PORT,Adbms6817_CS,GPIO_PIN_RESET);}

#define VOLT_BALANCING_THRESHOLD	10	// 100uV / bit

//#define USE_CELL_BALANCING

#define USE_CELL_BALANCING_CONTINUOUSLY
#ifndef	USE_CELL_BALANCING_CONTINUOUSLY
#define USE_CELL_BALANCING_PWM
#endif

enum REG_GRP {
	REG_GRP_A = 1,
	REG_GRP_B = 2,
	REG_GRP_C = 3,
	REG_GRP_D = 4
};

enum Sx {
	S0 = 0,
	S1,
	S2,
	S3,
	S4,
	S5,
	S6,
	S7,
	S8
};

enum BOOLEAN {
	FALSE = 0,
	TRUE = 1
};

typedef struct _ADBMS6817{
	int16_t crc15Table[256];
	int16_t crc10Table[256];
	uint16_t voltage_cell[8];	// ADBMS6817 100uV/bit
	uint16_t voltage_avg;	// ADBMS6817 100uV/bit
	uint16_t diagnostic_cell[8];	// ADBMS6817 200uV/bit
	uint16_t temp_gpio[2];
	uint8_t	discharge_state[9];	// Number of Sx Pin that cells in discharge
	uint8_t	discharge_pwm_state[9];	// Number of Sx Pin that cells in discharge pwm
	uint8_t	discharge_data[NUM_OF_ADBMS6817][8];
}_Adbms6817;
_Adbms6817 Adbms6817;

uint8_t ADCV[2]; //!< Cell Voltage conversion command.
uint8_t ADAX[2]; //!< GPIO conversion command.

//uint16_t    cell_codes[15][12];
//unsigned int cell_voltage[50];

void PEC15_Table_initialize();
void PEC10_Table_initialize();
uint16_t PEC15_calc(uint8_t len, uint8_t *data);
uint16_t PEC10_calc(uint8_t *pDataBuf , uint8_t bIsRxCmd, uint8_t nLength);

void set_adc(uint8_t MD, //ADC Mode
		uint8_t DCP, //Discharge Permit,�ŵ�����
		uint8_t CH, //Cell Channels to be measured
		uint8_t CHG //GPIO Channels to be measured
		);


void Print_Cell_Voltage(void);
void Print_gpio_temp(void);
void Print_6817_info(void);

// brief This function will initialize all 6804 variables and the SPI port.
// This function will initialize the Linduino to communicate with the LTC6804 with a 1MHz SPI clock.
// The Function also intializes the ADCV and ADAX commands to convert all cell and GPIO voltages in
// the Normal ADC mode.
void Adbms6817_initialize();
void Adbms6817_adcv(void);
void Adbms6817_adax(void);
void Adbms6817_adcvax(void);
void Adbms6817_rdcv_reg(uint8_t reg, uint8_t total_ic, uint8_t *data);
void Adbms6817_rdaux_reg(uint8_t reg, uint8_t total_ic, uint8_t *data);
void Adbms6817_rdcv_reg_ALL(uint8_t *data);
uint8_t Adbms_rdcv(uint8_t reg, uint8_t total_ic, uint16_t cell_codes[][12]);
void Adbms6817_wrcfg(uint8_t total_ic, uint8_t cfg_group, uint8_t config[][8]);
int8_t Adbms6817_rdcfg(uint8_t total_ic, uint8_t cfg_group, uint8_t r_config[][8]);
void Adbms6817_wrpwm(uint8_t total_ic, uint8_t config[][8]);
void Adbms6817_rdsid(uint8_t *data);
void Adbms6817_rdasall(uint8_t *data);
void Adbms6817_mute(void);
void Adbms6817_unmute(void);
void Adbms6817_srst(void);

void Adbms6817_discharge_cell_pwm(uint8_t Sx /* x : adbms6817 Sx Pin Number*/, uint8_t state);
void Adbms6817_discharge_cell_continuously(uint8_t Sx /* x : adbms6817 Sx Pin Number*/, uint8_t state);
void Adbms6817_read_adc(void);
void Calc_SOC_Voltage(void);
void wakeup_idle();
void Adbms6817_init_reg(void);
void spi_write_array(uint8_t len, uint8_t data[]);
void spi_write_read(uint8_t tx_Data[], uint8_t tx_len, uint8_t *rx_data, uint8_t rx_len);

#endif
/** @}*/
/** @}*/
