/*
 * function.h
 *
 *  Created on: Nov 24, 2022
 *      Author: maker_dell_01
 */

#ifndef INC_MY_FUNCTION_H_
#define INC_MY_FUNCTION_H_

#include "my_define.h"

#define RLY_MOT 	1
#define RLY_SMPS 	2
#define RLY_EXCHAG 	4

#define PWM_MOTOR		TIM_CHANNEL_1
#define PWM_EX_CHARGE	TIM_CHANNEL_2
#define PWM_SMPS_CHARGE	TIM_CHANNEL_3

#define OFF	0
#define ON	1

// internal flash memory last page address
#define STARTADDR ((uint32_t) 0x0800FC00)
#define ENDADDR ((uint32_t) 0x0800FFFF)
#define PAGE_SIZE 0x400
//extern uint8_t boot_time_sec, boot_time_min, boot_time_hour;

void Setup_default();
void Get_Switch_info();						//gwet user select switch infomation


void Charge_voltage_open(uint8_t state);	//charge lope 		//1 open 0 close
void Discharge_voltage_open(uint8_t state);	//discharge lope 	//1 open 0 close
void Charge_Discharge_control(void);

void Start_Relay_pwm(uint8_t pwm_state);	//BDU Relay on off	//1 on 0 off
void Relay_connect(uint8_t relay, uint8_t on_off);

void Check_output_pwm(void);

void Fan_pwm_control(void);

void Check_bms_error(void);
void calc_boot_time(void);

HAL_StatusTypeDef Write_eeprom(uint16_t MemAddress, uint8_t *pData, uint16_t DataSize);
HAL_StatusTypeDef Read_eeprom(uint16_t MemAddress, uint8_t *pData, uint16_t DataSize);

void Print_check_pin(void);
void PWM_Toggle_Relay(uint8_t nRelay, uint8_t state);
void Check_Relay_Contorl(void);

void flash_read(uint32_t addr, uint8_t* data, uint16_t size);
void flash_write(uint32_t addr, uint8_t* data, uint16_t size);

#endif /* INC_MY_FUNCTION_H_ */
