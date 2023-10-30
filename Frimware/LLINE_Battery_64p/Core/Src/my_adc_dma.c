/*
 * function.c
 *
 *  Created on: Nov 24, 2022
 *      Author: maker_dell_01
 */
#include "main.h"

//adc function
// mcu 4 adc ch

void Setup_ADC_default()
{
	float voltage_fix;
	uint16_t adc_arr_2[ADC_SAMPLE] = {0,};

	uint32_t adc_value=0;
	float adc_v_cur_ct;
	uint16_t adc_dma_copy[ADC_CH_CNT*ADC_SAMPLE] = {0,};
	memcpy(adc_dma_copy,g_uAdc_dma_Data,sizeof(adc_dma_copy));

	for(int i=0; i<ADC_SAMPLE; i++)
	{
		adc_arr_2[i]=  adc_dma_copy[ADC_CH_CNT*i+2];
	}
	quickSort(adc_arr_2, 0, ADC_SAMPLE-1);

	for(int i=ADC_NOT_USE; i<ADC_SAMPLE-ADC_NOT_USE; i++)	// 0~9 버림 // 10~19 사용	//20~29 버림
	{
		adc_value +=  adc_arr_2[i];
	}
	adc_v_cur_ct=(adc_value/10 * 3.30/4095.0);	//ct sensor		= voltage

	CT.uR1 		= 7150;
	CT.uR2 		= 10000;
	CT.fStep_1A 	= 20;									//5V base -> 1A = 20mv

	//voltage_fix = (double)CT.R2/(CT.R1+CT.R2);		//0.5830 ->	fix to 0.626(Real test vlaue)
	voltage_fix = 0.626;
	//CT.Step_1A 	= CT.Step_1A * CT.R2/(CT.R1+CT.R2);		//base voltage down -> 1A= 11.x mv
	CT.fStep_1A 	= CT.fStep_1A * voltage_fix;				//base voltage down -> 1A= 12.52 mv
	CT.fStep_01A	= CT.fStep_1A/10;
	//CT.Vmax		=(double)5*CT.R2/(CT.R1+CT.R2);		//2.9x voltage
	CT.fVmax		= 5 * voltage_fix;						//5v in -> 2.9x voltage //5v in ->(Real test vlaue 3.13)
	CT.fVmax_half =	CT.fVmax/2;
	//CT.Vmax_half 	=	CT.Vmax/2 - 0.1716;			//default value offset	0.1716
	CT.fOffset_v		= 	CT.fVmax_half - adc_v_cur_ct;
}

void Adc_dma_start()
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_uAdc_dma_Data, ADC_CH_CNT * ADC_SAMPLE);
}

void Adc_dma_stop()
{
	HAL_ADC_Stop_DMA(&hadc1);
}

void Read_DMA_Current_n_Temperature()
{
	uint32_t adc_value[ADC_CH_CNT] = {0,};
	uint16_t adc_dma_copy[ADC_CH_CNT*ADC_SAMPLE] = {0,};
	uint16_t adc_arr_0[ADC_SAMPLE] = {0,};
	uint16_t adc_arr_1[ADC_SAMPLE] = {0,};
	uint16_t adc_arr_2[ADC_SAMPLE] = {0,};
	uint16_t adc_arr_3[ADC_SAMPLE] = {0,};

	float adc_v_cur;
	float adc_v_cur_ct;
	float adc_v_ntc0;
	float adc_v_ntc1;

	memcpy(adc_dma_copy,g_uAdc_dma_Data,sizeof(adc_dma_copy));

	for(int i=0; i<ADC_SAMPLE; i++)
	{
		adc_arr_0[i]=  adc_dma_copy[ADC_CH_CNT*i];
		adc_arr_1[i]=  adc_dma_copy[ADC_CH_CNT*i+1];
		adc_arr_2[i]=  adc_dma_copy[ADC_CH_CNT*i+2];
		adc_arr_3[i]=  adc_dma_copy[ADC_CH_CNT*i+3];
	}

	quickSort(adc_arr_0, 0, ADC_SAMPLE-1);
	quickSort(adc_arr_1, 0, ADC_SAMPLE-1);
	quickSort(adc_arr_2, 0, ADC_SAMPLE-1);
	quickSort(adc_arr_3, 0, ADC_SAMPLE-1);

	for(int i=ADC_NOT_USE; i<ADC_SAMPLE-ADC_NOT_USE; i++)	// 0~9 버림 // 10~19 사용	//20~29 버림
	{
		adc_value[0] +=  adc_arr_0[i];		//중간 10개값 저장
		adc_value[1] +=  adc_arr_1[i];
		adc_value[2] +=  adc_arr_2[i];
		adc_value[3] +=  adc_arr_3[i];
	}

	adc_v_ntc1	=(adc_value[0]/10 * 3.30/4095.0);	//nct1			= voltage
	adc_v_ntc0	=(adc_value[1]/10)* 3.30/4095.0;	//nct0			= voltage
	adc_v_cur_ct=(adc_value[2]/10 * 3.30/4095.0);	//ct sensor		= voltage
	adc_v_cur	=(adc_value[3]/10 * 3.30/4095.0);	//PCB current	= voltage

	g_Adc_temp1 	= Adc_to_temperature(adc_v_ntc1);
	g_Adc_temp0 	= Adc_to_temperature(adc_v_ntc0);
	g_Adc_ct_Amp 	= Calc_CT_sensor(adc_v_cur_ct);
	g_Adc_pcb_Amp = Calc_PCB_current(adc_v_cur);

	Bms.Current = (double)(g_Adc_ct_Amp / 10.);

	Bms.Temperature[0] = g_Adc_temp0;
	Bms.Temperature[1] = g_Adc_temp1;

	if(Bms.MaxTemperature[0] < g_Adc_temp0)
	{
		Bms.MaxTemperature[0] = g_Adc_temp0;
	}
	if(Bms.MaxTemperature[1] < g_Adc_temp1)
	{
		Bms.MaxTemperature[1] = g_Adc_temp1;
	}

	if(Bms.MinTemperature[0] > g_Adc_temp0)
	{
		Bms.MinTemperature[0] = g_Adc_temp0;
	}
	if(Bms.MinTemperature[1] > g_Adc_temp1)
	{
		Bms.MinTemperature[1] = g_Adc_temp1;
	}
}

int16_t Calc_PCB_current(float Adc_bat)
{
	float Adc_bat_Cur = Adc_bat * 5 / 3.3;

	Adc_bat_Cur = Adc_bat_Cur-2.5;
	Adc_bat_Cur = Adc_bat_Cur*1000 / 6.67;	//1A = 6.67mv //Amp 로 변환	//2.5v = 0A 기준	//4.5v = 300A
	return (int16_t)Adc_bat_Cur;
}

#if 0
int16_t Calc_CT_sensor(float Adc_bat)
{
	//double Adc_bat_Cur = Adc_bat * 5 / 3.3;
	float Adc_bat_Cur = Adc_bat;
	uint16_t R1 = 7150;
	uint16_t R2 = 10000;
	float	Step_1A;	//mcu input max voltage
	float	Vmax = (float)5*R2/(R1+R2);	//mcu input max voltage

	Step_1A = 20;		//1A = 20mv
	Step_1A = Step_1A * R2/(R1+R2);			//100A ct sensor

	Adc_bat_Cur = Adc_bat_Cur-(Vmax/2);	// - base voltge
	Adc_bat_Cur = Adc_bat_Cur*1000 / Step_1A;	//*1000 = volt to mv change //1A = 20mv

	//offset +6A
	Adc_bat_Cur += 6;
	return (int16_t)Adc_bat_Cur;
}
#else
int16_t Calc_CT_sensor(double Adc_bat)
{
	//double Adc_bat_Cur = Adc_bat * 5 / 3.3;
	float Adc_bat_Cur;

	Adc_bat +=CT.fOffset_v;
	Adc_bat_Cur = Adc_bat - CT.fVmax_half;	// - base voltge
	//Adc_bat_Cur = Adc_bat_Cur*1000 / CT.fStep_1A;
	Adc_bat_Cur = Adc_bat_Cur*1000 / CT.fStep_01A;

	return (int16_t)Adc_bat_Cur;
}
#endif

int16_t Adc_to_temperature(double adc_v)
{
	int i = 0;
	double Rth_KOhm = 0;
	int16_t pullup_resistor = 10;	//10k

//Rth_table -50 ~ 150도
	double Rth_table[201] ={			670.1,	630.42,	590.74,	551.06,	511.38,	471.7,
	444.66,	417.62,	390.58,	363.54,	336.5,	317.72,	298.94,	280.16,	261.38,	242.6,
	229.48,	216.36,	203.24,	190.12,	177,	167.68,	158.36,	149.04,	139.72,	130.4,
	123.73,	117.06,	110.40,	103.73,	97.07,	92.242,	87.414,	82.586,	77.758,	72.93,
	69.41,	65.89,	62.37,	58.85,	55.33,	52.728,	50.126,	47.524,	44.922,	42.32,
	40.386,	38.452,	36.518,	34.584,	32.65,	31.198,	29.746,	28.294,	26.842,	25.39,
	24.292,	23.194,	22.096,	20.998,	19.9,	19.062,	18.224,	17.386,	16.548,	15.71,
	15.066,	14.422,	13.778,	13.134,	12.49,	11.992,	11.494,	10.996,	10.498,	10,
	9.6114,	9.2228,	8.8342,	8.4456,	8.057,	7.7518,	7.4466,	7.1414,	6.8362,	6.531,
	6.2902,	6.0494,	5.8086,	5.5678,	5.327,	5.1354,	4.9438,	4.7522,	4.5606,	4.369,
	4.2158,	4.0626,	3.9094,	3.7562,	3.603,	3.4796,	3.3562,	3.2328,	3.1094,	2.986,
	2.8864,	2.7868,	2.6872,	2.5876,	2.488,	2.407,	2.326,	2.245,	2.164,	2.083,
	2.0168,	1.9506,	1.8844,	1.8182,	1.752,	1.6978,	1.6436,	1.5894,	1.5352,	1.481,
	1.4364,	1.3918,	1.3472,	1.3026,	1.258,	1.2208,	1.1836,	1.1464,	1.1092,	1.072,
	1.0411,	1.0102,	0.9794,	0.9485,	0.9177,	0.8918,	0.8660,	0.8401,	0.8143,	0.7885,
	0.7668,	0.7451,	0.7234,	0.7017,	0.68,	0.6617,	0.6434,	0.6251,	0.6068,	0.5886,
	0.5731,	0.5576,	0.5421,	0.5266,	0.5112,	0.4980,	0.4848,	0.4717,	0.4585,	0.4454,
	0.4341,	0.4229,	0.4117,	0.4005,	0.3893,	0.3797,	0.3702,	0.3607,	0.3512,	0.3417,
	0.3335,	0.3253,	0.3172,	0.3090,	0.3009,	0.2938,	0.2867,	0.2796,	0.2725,	0.2654,
	0.2592,	0.2531,	0.2470,	0.2409,	0.2348,	0.2295,	0.2242,	0.2189,	0.2136,	0.2083,
	0.2037,	0.1991,	0.1945,	0.1899,	0.1853	};

	//Rth_KOhm = (double)(adc_temp * pullup_resistor) /(4096 - adc_temp);
	Rth_KOhm = (double)(adc_v * pullup_resistor) /(3.3 - adc_v);

	for( i = 0 ;i < 201 ;i++)
	{
		if(Rth_KOhm >= Rth_table[i])	break;
	}
	return i-50;
}

void quickSort(uint16_t arr[], uint16_t left, uint16_t right) {
	int i = left, j = right;
	int pivot = arr[(left + right) / 2];
	int temp;

	while (i <= j)
	{
		while (arr[i] < pivot)	i++; // arr[i] ≥ pivot 일 때까지, left에서 오른쪽 방향으로 탐색
		while (arr[j] > pivot)	j--; // arr[j] ≤ pivot 일 때까지, right에서 왼쪽 방향으로 탐색

		if (i <= j) // 큰 값이 작은 값보다 왼쪽에 있으면
		{
			// SWAP(arr[i], arr[j])
			temp = arr[i];
			arr[i] = arr[j];
			arr[j] = temp;

			i++;
			j--;
		}
	}

	if (left < j)	quickSort(arr, left, j);
	if (i < right)	quickSort(arr, i, right);
}

void Print_adc_value(void)
{
	//float temp = 0;
	printf("NCT0 = %d	NCT1 = %d	Cur_ct = %d	Cur = %d \r\n", g_Adc_temp0,g_Adc_temp1,g_Adc_ct_Amp,g_Adc_pcb_Amp);
}

