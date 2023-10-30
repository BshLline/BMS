/*
 * function.h
 *
 *  Created on: Nov 24, 2022
 *      Author: maker_dell_01
 */

#ifndef INC_MY_ADC_H_
#define INC_MY_ADC_H_

#define ADC_CH_CNT 4
#define ADC_SAMPLE 30
#define ADC_NOT_USE 10

//---------------ADC---------------//

void Setup_ADC_default();
void Adc_dma_start();
void Adc_dma_stop();
void Read_DMA_Current_n_Temperature();
int16_t Adc_to_temperature(double adc_temperature);
int16_t Calc_PCB_current(float Adc_bat);
int16_t Calc_CT_sensor(double Adc_bat);
void quickSort(uint16_t arr[], uint16_t left, uint16_t right);

void Print_adc_value(void);

uint16_t g_uAdc_dma_Data[ADC_CH_CNT * ADC_SAMPLE];

int16_t g_Adc_pcb_Amp;	//ADC1 CH1	// Removed
int16_t g_Adc_temp1;		//ADC1 CH2	// 1'C
int16_t g_Adc_temp0;		//ADC1 CH3	// 1'C
int16_t g_Adc_ct_Amp;		//ADC1 CH4	// 0.1 A

int8_t g_uCurrent_offset;

struct CT_Sensor
{
	uint16_t uR1;
	uint16_t uR2;
	float	fStep_1A;		//mcu input max voltage
	float	fStep_01A;		//mcu input max voltage
	float	fVmax;			//mcu input max voltage
	float	fVmax_half;		//mcu input max voltage
	float	fOffset_v;		//mcu input max voltage
} CT;

#endif
