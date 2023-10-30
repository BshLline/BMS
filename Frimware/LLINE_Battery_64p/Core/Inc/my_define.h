/*
 * my_define.h
 *
 *  Created on: 2022. 12. 7.
 *      Author: maker_dell_01
 */

#ifndef INC_MY_DEFINE_H_
#define INC_MY_DEFINE_H_

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define BMS_BAT_500W	1
#define BMS_BAT_1KW		2

#define BAT_VOLUME		BMS_BAT_500W


#if BAT_VOLUME == BMS_BAT_500W
	#define OVER_CHARGE		-600
	#define OVER_CURRENT	600

#elif BAT_VOLUME == BMS_BAT_1KW
	#define OVER_CHARGE		-600
	#define OVER_CURRENT	600
#endif

#define OVER_TEMP_50		50
#define OVER_TEMP_60		60

#define ERROR_TIME			6

#define DEBUG_SERIAL
#define DEBUG_SERIAL_BMS
#define DEBUG_SERIAL_ADBMS6817
//#define DEBUG_SERIAL_CAN
#define DEBUG_SERIAL_BROADCAST

#define DEBUG_SERIAL_TEST_BSH

#define IS_SMPS_RELAY_CHECK HAL_GPIO_ReadPin(SMPS_RELAY_CHECK_GPIO_Port, SMPS_RELAY_CHECK_Pin)
#define IS_EX_CHARGING_RELAY_CHECK HAL_GPIO_ReadPin(EX_CHAG_RLY_CHECK_GPIO_Port, EX_CHAG_RLY_CHECK_Pin)
#define IS_MOTOR_RELAY_CHECK HAL_GPIO_ReadPin(MOTOR_RELAY_CHECK_GPIO_Port, MOTOR_RELAY_CHECK_Pin)
#define IS_PRE_CHARGE_CHECK HAL_GPIO_ReadPin(PRE_CHARGE_CHECK_GPIO_Port, PRE_CHARGE_CHECK_Pin)
#define IS_CHARGER_CHECK HAL_GPIO_ReadPin(CHARGER_EN_CHECK_GPIO_Port, CHARGER_EN_CHECK_Pin)

struct Switch_info
{
	uint8_t g_uOption_id;
	uint8_t g_uCom_sel;
} Sw_info;


uint8_t g_uSerial_tx_buffer[256]; //for debug

struct Error
{
	uint8_t uFlag;
	uint8_t uMotpin;
	uint8_t uExChagpin;
	uint8_t uCurrent_over;
	uint8_t uSmpspin;

	uint8_t g_uOver_Current_cnt;
	uint8_t g_uOver_Charge_cnt;
	uint8_t g_uOver_Temp_cnt;

	uint8_t g_uOver_Current_flag;	//check 1s //6 times over? flag set//
	uint8_t g_uOver_Charge_flag;	//check 1s //6 times over? flag set//
	uint8_t g_uOver_Temp_flag;		//check 1s //6 times over? flag set//
}Err;

extern volatile uint32_t tickCnt_100ms;
extern volatile uint32_t tickCnt_1s;
extern uint8_t	g_bFlag_100ms;
extern uint8_t	g_bFlag_1Sec;
extern uint8_t	g_bFlag_5Sec;

extern uint8_t boot_time_sec, boot_time_min, boot_time_hour;

#endif /* INC_MY_DEFINE_H_ */
