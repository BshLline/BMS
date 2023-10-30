/*
 * function.c
 *
 *  Created on: Nov 24, 2022
 *      Author: maker_dell_01
 */

#include "main.h"

#define EEPROM_ADDR 0xA0

uint8_t boot_time_sec, boot_time_min, boot_time_hour;

void Setup_default()
{
	tickCnt_100ms = 0;
	tickCnt_1s = 0;
	g_bFlag_1Sec = 0;
	g_bFlag_5Sec = 0;

	// PWM Init
	TIM3->CCR1 =TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = 0;

	// Min/Max Temp Init
	Bms.MaxTemperature[0] = 0;
	Bms.MaxTemperature[1] = 0;
	Bms.MinTemperature[0] = 0x7FFF;
	Bms.MinTemperature[1] = 0x7FFF;

	memset(&Err,0x00,sizeof(struct Error));

	// Fan On
	TIM3->CCR4 = 100;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	// Relays On
	PWM_Toggle_Relay(PWM_MOTOR, ON);
	PWM_Toggle_Relay(PWM_EX_CHARGE, ON);
	PWM_Toggle_Relay(PWM_SMPS_CHARGE, ON);
}

void Get_Switch_info()
{
	//read
	uint8_t state;

	Sw_info.g_uOption_id = 0;
	Sw_info.g_uCom_sel = 0;

	state = HAL_GPIO_ReadPin( CAN_ID8_GPIO_Port,CAN_ID8_Pin );
	state = state<<1;
	state += HAL_GPIO_ReadPin( CAN_ID4_GPIO_Port,CAN_ID4_Pin );
	state = state<<1;
	state += HAL_GPIO_ReadPin( CAN_ID2_GPIO_Port,CAN_ID2_Pin );
	state = state<<1;
	state += HAL_GPIO_ReadPin( CAN_ID1_GPIO_Port,CAN_ID1_Pin );

	Sw_info.g_uOption_id = state;


	state = HAL_GPIO_ReadPin( COM_SEL1_GPIO_Port,COM_SEL1_Pin );
	state = state<<1;
	state += HAL_GPIO_ReadPin( COM_SEL0_GPIO_Port,COM_SEL0_Pin );

	Sw_info.g_uCom_sel = state;
}

void Charge_voltage_open(uint8_t state)
{
	HAL_GPIO_WritePin(CHARGE_PWR_EN_GPIO_Port, CHARGE_PWR_EN_Pin, state);
}

void Discharge_voltage_open(uint8_t state)
{
	HAL_GPIO_WritePin(CHARGE_PWR_EN_GPIO_Port, CHARGE_PWR_EN_Pin, state);
}

void Charge_Discharge_control(void)
{
	if(Err.g_uOver_Current_flag)		//Disconnect output
	{
		Err.g_uOver_Current_flag = 0;
		Start_Relay_pwm(0);
		//Send Message over current Error
	}
	else if(Err.g_uOver_Charge_flag)	//Disconnect input
	{
		Err.g_uOver_Charge_flag = 0;
		//Send Message over Charge Error
	}

	if(Err.g_uOver_Temp_flag)
	{
		Start_Relay_pwm(0);
		Err.g_uOver_Temp_flag = 0;
		//Send Message over Temperature Error
	}
}

void Start_Relay_pwm(uint8_t pwm_state)
{
	//TIM_CHANNEL_1 smps charge open
	//TIM_CHANNEL_2 Ex chag charge open
	//TIM_CHANNEL_3 Motor Relay
	if(pwm_state == 1)	// on
	{
		//first precharge open	//after 100ms -> open Motor Relay
		HAL_GPIO_WritePin(PRE_CHARGE_RELAY_EN_GPIO_Port, PRE_CHARGE_RELAY_EN_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		TIM3->CCR1 =  100;	//smps charge open
		TIM3->CCR2 =  100;	//Ex chag charge open
		TIM3->CCR3 =  100;	//Motor Relay
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		HAL_Delay(100);
		TIM3->CCR1 =  35;
		TIM3->CCR2 =  35;
		TIM3->CCR3 =  35;
		HAL_GPIO_WritePin(PRE_CHARGE_RELAY_EN_GPIO_Port, PRE_CHARGE_RELAY_EN_Pin, GPIO_PIN_RESET);
	}
	else if(pwm_state == 0)	//off
	{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		HAL_GPIO_WritePin(PRE_CHARGE_RELAY_EN_GPIO_Port, PRE_CHARGE_RELAY_EN_Pin, GPIO_PIN_RESET);
	}
}

void PWM_Toggle_Relay(uint8_t nRelay, uint8_t state)
{
	if(state)
	{
		switch (nRelay)
		{
		case PWM_SMPS_CHARGE:
			TIM3->CCR3 =  100;	//smps charge open
			break;
		case PWM_EX_CHARGE:
			TIM3->CCR2 =  100;	//Ex charge open
			break;
		case PWM_MOTOR:
			// pre-charge 100ms
			HAL_GPIO_WritePin(PRE_CHARGE_RELAY_EN_GPIO_Port, PRE_CHARGE_RELAY_EN_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			TIM3->CCR1 =  100;	//Motor Relay
			break;
		}
		HAL_TIM_PWM_Start(&htim3, nRelay);
		HAL_Delay(100);
		switch (nRelay)
		{
		case PWM_SMPS_CHARGE:
			TIM3->CCR3 =  35;	//smps charge open
			break;
		case PWM_EX_CHARGE:
			TIM3->CCR2 =  35;	//Ex charge open
			break;
		case PWM_MOTOR:
			HAL_GPIO_WritePin(PRE_CHARGE_RELAY_EN_GPIO_Port, PRE_CHARGE_RELAY_EN_Pin, GPIO_PIN_RESET);
			TIM3->CCR1 =  35;	//Motor Relay
			break;
		}
	}
	else
	{
		HAL_TIM_PWM_Stop(&htim3, nRelay);
	}
}

void Relay_connect(uint8_t relay, uint8_t on_off)
{
	if(on_off ==1)	//on
	{
		if(relay || RLY_MOT)	//RLY_MOT  // RLY_SMPS	//RLY_EXCHAG
		{

		}
	}
}

void Check_output_pwm(void)
{
	Err.uMotpin = HAL_GPIO_ReadPin( MOTOR_RELAY_PWM_GPIO_Port,MOTOR_RELAY_PWM_Pin );
	Err.uExChagpin = HAL_GPIO_ReadPin(EX_CHARG_RELAY_PWM_GPIO_Port , EX_CHAG_RLY_CHECK_Pin);
	Err.uSmpspin = HAL_GPIO_ReadPin( SMPS_RELAY_PWM_GPIO_Port, SMPS_RELAY_PWM_Pin);
	if(Err.uMotpin || Err.uExChagpin || Err.uSmpspin) Err.uFlag = 1;
}


void Fan_pwm_control(void)
{
	uint8_t temp_max;

	if(g_Adc_temp0 > g_Adc_temp1)	temp_max = g_Adc_temp0;
	else							temp_max = g_Adc_temp1;

	if		(temp_max < 30)	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
	else
	{
		if		(temp_max < 40)	TIM3->CCR1 = 70;
		else 					TIM3->CCR1 = 90;
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	}
}

void Check_bms_error(void)
{
	if (g_Adc_ct_Amp > OVER_CURRENT)	//over 60A discharg
	{
		Err.g_uOver_Current_cnt++;
		if(Err.g_uOver_Current_cnt > ERROR_TIME)
		{
			Err.g_uOver_Current_flag = 1;
			Err.g_uOver_Current_cnt =0;
		}
	}
	else	Err.g_uOver_Current_cnt = 0;

	if (g_Adc_ct_Amp < OVER_CHARGE)	//over 60A charge
	{
		Err.g_uOver_Charge_cnt++;
		if(Err.g_uOver_Charge_cnt > ERROR_TIME)
		{
			Err.g_uOver_Charge_flag = 1;
			Err.g_uOver_Charge_cnt = 0;
		}
	}
	else	Err.g_uOver_Charge_cnt = 0;

	if ((g_Adc_temp1 <= OVER_TEMP_50 ) || (g_Adc_temp0 <= OVER_TEMP_50 ))	//Temperature 50 over
	{
		Err.g_uOver_Temp_cnt++;
		if(Err.g_uOver_Temp_cnt > ERROR_TIME)
		{
			Err.g_uOver_Temp_flag = 1;
			Err.g_uOver_Temp_cnt = 0;
		}
	}
	else	Err.g_uOver_Temp_cnt = 0;
}

void Check_Relay_Contorl(void)
{
	static uint8_t cnt_Vup = 0, cnt_Vdown = 0;
	// SMPS off under 3.5V
	if(Bms.Voltage_avg < 35000)	// 3.5V
	{
		cnt_Vdown++;
		cnt_Vup = 0;
		if(cnt_Vdown > 60)	// 1 min
		{
			PWM_Toggle_Relay(PWM_SMPS_CHARGE, FALSE);
		}
	}
	else
	{
		cnt_Vdown = 0;
		cnt_Vup++;
		if(cnt_Vup > 60)	// 1 min
		{
			PWM_Toggle_Relay(PWM_SMPS_CHARGE, TRUE);
		}
	}
}

HAL_StatusTypeDef Write_eeprom(uint16_t MemAddress, uint8_t *pData, uint16_t DataSize)
{
	HAL_StatusTypeDef error;
	error = HAL_I2C_Mem_Write(&hi2c2, EEPROM_ADDR, MemAddress, 1, pData, DataSize, 100);
	return error;
}

HAL_StatusTypeDef Read_eeprom(uint16_t MemAddress, uint8_t *pData, uint16_t DataSize)
{
	HAL_StatusTypeDef error;
	error = HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDR, MemAddress, 1, pData, DataSize, 100);
	return error;
}

void calc_boot_time(void)
{
	boot_time_sec++;

	if(boot_time_sec>59)
	{
		boot_time_min++;
		boot_time_sec = 0;
	}
	if(boot_time_min > 59)
	{
		boot_time_hour++;
		boot_time_min = 0;
	}
}


//#define StartAddr ((uint32_t) 0x0800FC00)
//#define EndAddr ((uint32_t) 0x0800FCFF)

void flash_read(uint32_t addr, uint8_t* data, uint16_t size)
{
	uint16_t i;

#if 0
	// Test Code
	uint8_t page[1024];
	for(i=0;i<PAGE_SIZE;i++)
	{
		page[i] = *((__IO uint8_t*)STARTADDR+i);
		if(i%16==0)
		{
			printf("\r\n%04X   ",i);
		}
		printf("%02X ", page[i]);
		//if((i%15 == 0) && (i != 0))
			//printf("\r\n");
	}
	printf("\r\n");
#endif

	for(i=0;i<size;i++)
	{
		data[i] = *((__IO uint8_t*)STARTADDR+addr+i);
	}
}

void flash_write(uint32_t addr, uint8_t* data, uint16_t size)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint16_t i;
	uint32_t PageError, program_data;
	uint8_t page[1024];

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = STARTADDR;
	EraseInitStruct.NbPages = 1;

	if((STARTADDR + addr + size - 1) > ENDADDR)
	{
		printf("flash write size over error\r\n");
		return;
	}

	// page read
	for(i=0;i<PAGE_SIZE;i++)
	{
		page[i] = *((__IO uint32_t*)STARTADDR+i);
	}

	for(i=0;i<size;i++)
	{
		page[addr+i] = data[i];
	}

	// Flash Unlock
	HAL_FLASH_Unlock();

	// Flash Erase
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{
		printf("Flash Erase Error\r\n");
		return;
	}

	// program page
	for(i=0;i<PAGE_SIZE;i=i+4)
	{
		program_data = (page[i+3] << 24) | (page[i+2] << 16) | (page[i+1] << 8) | (page[i+0] << 0);
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, STARTADDR+i, program_data)  != HAL_OK)
		{
			printf("Flash Program Error %d\r\n", i);
			return;
		}
	}

	// Flash Lock
	HAL_FLASH_Lock();

}
